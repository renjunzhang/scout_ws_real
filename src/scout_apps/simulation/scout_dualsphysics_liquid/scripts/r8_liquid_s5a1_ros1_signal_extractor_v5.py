#!/usr/bin/env python3
"""Relative-SE(3) TF witness adapter for the frozen S5A1 extractor.

The motion source remains odometry only.  TF is a QC witness: both odometry
and Cartographer TF are reduced to ``T(t0)^-1 * T(t)`` before comparison so a
fixed world-frame offset/rotation cannot be mistaken for motion disagreement.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_motion_bridge_v1 as bridge
import r8_liquid_s5a1_ros1_signal_extractor_v4 as base


sys.dont_write_bytecode = True
SignalExtractionError = base.SignalExtractionError
extract_bag_bytes = base.extract_bag_bytes
build_motion_samples = base.build_motion_samples
nearest_clock_alignment = base.nearest_clock_alignment
read_exact_primary = base.read_exact_primary
ROOT = MODULE_DIR.parent
BASE_PATH = Path(base.__file__).resolve()
BASE_SHA256 = "94523bc1d96a089ceaa6d96dd568b795b65fdcce68eab42c40150d3bda3e15f2"
MAXIMUM_TIME_DELTA_S = 0.5
MAXIMUM_POSITION_DELTA_M = 0.05
MAXIMUM_ANGULAR_DELTA_RAD = 0.1
REAL_DIAGNOSTIC_WITNESS = {
    "absolute_maximum_position_delta_m": 3.9934499955863565,
    "maximum_time_delta_s": 0.005,
    "absolute_maximum_angular_delta_rad": 0.023228249162941604,
    "relative_maximum_translation_delta_m": 0.03995630546880967,
    "relative_rms_translation_delta_m": 0.022725548786785066,
    "relative_maximum_angular_delta_rad": 0.003657518055773045,
}


def _pose(row: Mapping[str, Any]) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    position = tuple(float(row[key]) for key in ("x_m", "y_m", "z_m"))
    quaternion = bridge.normalize_quaternion(tuple(float(row[key]) for key in ("qx", "qy", "qz", "qw")))
    return position, quaternion


def _relative(
    origin: tuple[tuple[float, float, float], tuple[float, float, float, float]],
    current: tuple[tuple[float, float, float], tuple[float, float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    origin_position, origin_quaternion = origin
    current_position, current_quaternion = current
    inverse = bridge.quaternion_conjugate(origin_quaternion)
    delta = tuple(current_position[index] - origin_position[index] for index in range(3))
    relative_position = bridge.rotate_vector(inverse, delta)
    relative_quaternion = bridge.quaternion_multiply(inverse, current_quaternion)
    return relative_position, relative_quaternion


def tf_cross_check(
    odom: Sequence[Mapping[str, Any]], transforms: Sequence[Mapping[str, Any]], *,
    maximum_time_delta_s: float, maximum_position_delta_m: float,
    maximum_angular_delta_rad: float,
) -> dict[str, Any]:
    if not odom:
        raise SignalExtractionError("odom witness is empty")
    matching = sorted(
        (row for row in transforms if row["topic"] == "/tf" and row["parent"] == "odom"
         and row["child"] == "base_footprint"), key=lambda row: int(row["stamp_ns"])
    )
    if not matching:
        raise SignalExtractionError("required odom->base_footprint TF witness is absent")
    stamps = [int(row["stamp_ns"]) for row in matching]
    paired: list[tuple[tuple[Any, Any], tuple[Any, Any]]] = []
    maximum_time = 0.0
    absolute_maximum_position = 0.0
    absolute_maximum_angle = 0.0
    for pose in odom:
        stamp = int(pose["odom_header_t_ns"])
        index = bisect.bisect_left(stamps, stamp)
        choices = matching[max(0, index - 1):min(len(matching), index + 1)]
        selected = min(choices, key=lambda row: abs(int(row["stamp_ns"]) - stamp))
        odom_pose, tf_pose = _pose(pose), _pose(selected)
        paired.append((odom_pose, tf_pose))
        maximum_time = max(maximum_time, abs(int(selected["stamp_ns"]) - stamp) / 1e9)
        absolute_maximum_position = max(
            absolute_maximum_position,
            math.sqrt(sum((tf_pose[0][axis] - odom_pose[0][axis]) ** 2 for axis in range(3))),
        )
        absolute_maximum_angle = max(
            absolute_maximum_angle,
            bridge.quaternion_angular_distance(odom_pose[1], tf_pose[1]),
        )
    odom_origin, tf_origin = paired[0]
    relative_translation_errors: list[float] = []
    relative_maximum_angle = 0.0
    for odom_pose, tf_pose in paired:
        odom_relative = _relative(odom_origin, odom_pose)
        tf_relative = _relative(tf_origin, tf_pose)
        relative_translation_errors.append(math.sqrt(sum(
            (tf_relative[0][axis] - odom_relative[0][axis]) ** 2 for axis in range(3))))
        relative_maximum_angle = max(
            relative_maximum_angle,
            bridge.quaternion_angular_distance(odom_relative[1], tf_relative[1]),
        )
    relative_maximum_translation = max(relative_translation_errors)
    relative_rms_translation = math.sqrt(sum(value * value for value in relative_translation_errors)
                                         / len(relative_translation_errors))
    if (maximum_time > maximum_time_delta_s
            or relative_maximum_translation > maximum_position_delta_m
            or relative_maximum_angle > maximum_angular_delta_rad):
        raise SignalExtractionError("relative TF pose witness exceeds frozen mismatch threshold")
    return {
        "status": "PASS_ALIGNMENT_WITNESS", "odom_frame": "odom",
        "base_frame": "base_footprint",
        "container_mount_source": "FROZEN_COMPATIBILITY_CONTRACT",
        "tf_used_as_motion": False, "comparison_frame": "T0_INVERSE_TIMES_T_RELATIVE_SE3",
        "sample_count": len(odom),
        "maximum_time_delta_s": maximum_time,
        "absolute_maximum_position_delta_m": absolute_maximum_position,
        "absolute_maximum_angular_delta_rad": absolute_maximum_angle,
        "relative_maximum_translation_delta_m": relative_maximum_translation,
        "relative_rms_translation_delta_m": relative_rms_translation,
        "relative_maximum_angular_delta_rad": relative_maximum_angle,
        "frozen_thresholds": {
            "maximum_time_delta_s": maximum_time_delta_s,
            "maximum_position_delta_m": maximum_position_delta_m,
            "maximum_angular_delta_rad": maximum_angular_delta_rad,
        },
    }


def self_check() -> dict[str, Any]:
    if hashlib.sha256(BASE_PATH.read_bytes()).hexdigest() != BASE_SHA256:
        raise SignalExtractionError("extractor v4 parent identity drifted")
    diagnostic = REAL_DIAGNOSTIC_WITNESS
    if not (diagnostic["absolute_maximum_position_delta_m"] > MAXIMUM_POSITION_DELTA_M
            and diagnostic["relative_maximum_translation_delta_m"] <= MAXIMUM_POSITION_DELTA_M
            and diagnostic["relative_maximum_angular_delta_rad"] <= MAXIMUM_ANGULAR_DELTA_RAD
            and diagnostic["maximum_time_delta_s"] <= MAXIMUM_TIME_DELTA_S):
        raise SignalExtractionError("real diagnostic boundary no longer proves the relative contract")
    return {
        "status": "PASS_S5A1_ROS1_SIGNAL_EXTRACTOR_V5_RELATIVE_TF_STATIC_CONTRACT",
        "parent_v4_sha256": BASE_SHA256,
        "comparison_frame": "T0_INVERSE_TIMES_T_RELATIVE_SE3",
        "real_diagnostic_witness": diagnostic,
        "thresholds_unchanged": True, "tf_used_as_motion": False,
        "real_bag_read": False, "files_written": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    args = parser.parse_args(argv)
    try:
        result = self_check() if args.command == "self-check" else None
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "error": str(exc)}, sort_keys=True),
              file=sys.stderr)
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


__all__ = ["SignalExtractionError", "extract_bag_bytes", "build_motion_samples",
           "nearest_clock_alignment", "tf_cross_check", "read_exact_primary",
           "REAL_DIAGNOSTIC_WITNESS", "self_check"]


if __name__ == "__main__":
    raise SystemExit(main())
