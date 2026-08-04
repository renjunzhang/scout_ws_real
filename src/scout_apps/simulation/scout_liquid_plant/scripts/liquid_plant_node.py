#!/usr/bin/env python3
"""ROS adapter for the development-only independent liquid plant.

The sole subscription is ``/odom``.  It derives excitation from the executed
simulated base motion reported there and publishes unvalidated plant-side
signals under ``/sim_truth`` for recording and development analysis only.
"""

from __future__ import print_function

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, String

from scout_liquid_plant.core import LiquidPlant, OdomSample, PlantConfigError, PlantParameters


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _yaw_from_quaternion(quaternion: Any) -> float:
    """Normalize then decode a planar yaw without importing controller helpers."""

    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise PlantConfigError("received invalid odometry quaternion")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LiquidPlantNode:
    """ROS-only shell around the tested pure-Python plant state machine."""

    def __init__(self) -> None:
        config = rospy.get_param("~")
        self._parameters = PlantParameters.from_mapping(config)
        self._plant = LiquidPlant(self._parameters)

        self._height_publisher = rospy.Publisher(
            self._parameters.height_topic, Float64, queue_size=20
        )
        self._state_publisher = rospy.Publisher(
            self._parameters.state_topic, Float64MultiArray, queue_size=20
        )
        self._metadata_publisher = rospy.Publisher(
            self._parameters.metadata_topic, String, queue_size=1, latch=True
        )
        # This package intentionally creates exactly one subscriber.  The core
        # sees executed odometry only; it never sees commands, monitor output,
        # controller state, or another /sim_truth signal.
        self._odom_subscriber = rospy.Subscriber(
            self._parameters.odom_topic, Odometry, self._odom_callback, queue_size=100
        )
        self._publish_metadata()
        rospy.loginfo(
            "[scout_liquid_plant] development-only plant started: input=%s, "
            "height=%s, fidelity=UNVALIDATED, formal=false",
            self._parameters.odom_topic,
            self._parameters.height_topic,
        )

    def _publish_metadata(self) -> None:
        metadata: Dict[str, Any] = self._parameters.public_metadata()
        config_path_value = rospy.get_param("~config_path", "")
        config_path = Path(str(config_path_value)) if config_path_value else None
        if config_path is not None and config_path.is_file():
            metadata["parameter_template_path"] = str(config_path.resolve())
            metadata["parameter_template_sha256"] = _sha256_file(config_path)
        else:
            metadata["parameter_template_path"] = None
            metadata["parameter_template_sha256"] = None
            metadata["parameter_template_warning"] = (
                "No readable config_path was supplied; this development output is not hash-bound."
            )

        source_files = [Path(__file__).resolve(), Path(sys.modules["scout_liquid_plant.core"].__file__).resolve()]
        metadata["development_code_sha256"] = {
            str(path): _sha256_file(path) for path in source_files
        }
        metadata["state_field_order"] = self._plant.state_field_names()
        metadata["state_message_type"] = "std_msgs/Float64MultiArray"
        metadata["height_message_type"] = "std_msgs/Float64"
        metadata["metadata_message_type"] = "std_msgs/String"
        self._metadata_publisher.publish(String(data=json.dumps(metadata, sort_keys=True)))

    def _odom_callback(self, message: Odometry) -> None:
        try:
            stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = rospy.Time.now().to_sec()
            sample = OdomSample(
                stamp_sec=stamp,
                yaw_rad=_yaw_from_quaternion(message.pose.pose.orientation),
                linear_x_mps=float(message.twist.twist.linear.x),
                linear_y_mps=float(message.twist.twist.linear.y),
                yaw_rate_radps=float(message.twist.twist.angular.z),
            )
            output = self._plant.step(sample)
        except (PlantConfigError, ValueError, OverflowError) as exc:
            rospy.logwarn_throttle(1.0, "[scout_liquid_plant] rejected /odom sample: %s", exc)
            return

        self._height_publisher.publish(Float64(data=output.liquid_height_m))
        state = Float64MultiArray()
        state.layout.dim = [
            MultiArrayDimension(
                label=",".join(self._plant.state_field_names()),
                size=len(output.state_values),
                stride=len(output.state_values),
            )
        ]
        state.layout.data_offset = 0
        state.data = list(output.state_values)
        self._state_publisher.publish(state)
        if not output.integrated:
            rospy.logwarn_throttle(
                1.0,
                "[scout_liquid_plant] state not integrated (%s); output remains development-only",
                output.reason,
            )


def main() -> int:
    rospy.init_node("liquid_plant_development", anonymous=False)
    try:
        LiquidPlantNode()
    except PlantConfigError as exc:
        rospy.logfatal("[scout_liquid_plant] configuration rejected: %s", exc)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
