#!/usr/bin/env python3
"""Publish the H0-development path on an isolated simulation ROS master.

This is a simulation-owned replacement for the legacy proxy path publisher.
It never starts a planner, never publishes ``/cmd_vel``, and has no dependency
on the physical-controller package.  It samples the current ``/odom`` pose
once, generates the H0 runtime path in that odometry frame, and publishes a
latched ``nav_msgs/Path``.

It is deliberately not formal H1/L1 replay: an explicit H0 acknowledgement,
``/use_sim_time=true``, and an H0-only template are all required.  Formal rows
must use hash-bound frozen JSON path replay instead of this runtime generator.
"""

from __future__ import annotations

import math
import threading
from typing import Iterable, List, Sequence, Tuple


H0_DEVELOPMENT_CONDITION = "H0"
H0_EVIDENCE_CLASS = "DEVELOPMENT_SMOKE_NOT_FORMAL"
SUPPORTED_H0_TEMPLATES = frozenset(("s_curve", "straight"))

PointYaw = Tuple[float, float, float]
PointXY = Tuple[float, float]


class H0PathError(RuntimeError):
    """A fail-closed H0 path admission or geometry error."""


def yaw_from_quaternion(quaternion: object) -> float:
    """Return planar yaw without depending on ROS helper packages."""

    x = float(getattr(quaternion, "x"))
    y = float(getattr(quaternion, "y"))
    z = float(getattr(quaternion, "z"))
    w = float(getattr(quaternion, "w"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _require_finite(label: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise H0PathError(f"{label} must be finite")
    return value


def _chaikin_open(points: Sequence[PointXY], iterations: int) -> List[PointXY]:
    """Smooth an open control polyline while retaining exact endpoints."""

    refined = list(points)
    for _ in range(max(0, int(iterations))):
        if len(refined) < 2:
            break
        next_points: List[PointXY] = [refined[0]]
        for first, second in zip(refined[:-1], refined[1:]):
            next_points.append(
                (0.75 * first[0] + 0.25 * second[0], 0.75 * first[1] + 0.25 * second[1])
            )
            next_points.append(
                (0.25 * first[0] + 0.75 * second[0], 0.25 * first[1] + 0.75 * second[1])
            )
        next_points.append(refined[-1])
        refined = next_points
    return refined


def _resample_polyline(points: Sequence[PointXY], spacing_m: float) -> List[PointXY]:
    """Return evenly spaced open-polyline samples including both endpoints."""

    if len(points) < 2:
        raise H0PathError("H0 control polyline needs at least two points")
    spacing_m = _require_finite("spacing_m", spacing_m)
    if not 0.01 <= spacing_m <= 0.50:
        raise H0PathError("spacing_m must be in [0.01, 0.50] m")

    segments: List[Tuple[PointXY, PointXY, float]] = []
    total_length = 0.0
    for first, second in zip(points[:-1], points[1:]):
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        if length > 1e-9:
            segments.append((first, second, length))
            total_length += length
    if total_length < 0.30:
        raise H0PathError("H0 goal is too close to the sampled odom start pose")

    intervals = max(1, int(math.ceil(total_length / spacing_m)))
    samples: List[PointXY] = []
    segment_index = 0
    accumulated = 0.0
    for index in range(intervals + 1):
        target = total_length * float(index) / float(intervals)
        while (
            segment_index < len(segments) - 1
            and target > accumulated + segments[segment_index][2]
        ):
            accumulated += segments[segment_index][2]
            segment_index += 1
        first, second, length = segments[segment_index]
        ratio = min(1.0, max(0.0, (target - accumulated) / length))
        samples.append(
            (first[0] + ratio * (second[0] - first[0]), first[1] + ratio * (second[1] - first[1]))
        )
    samples[0] = points[0]
    samples[-1] = points[-1]
    return samples


def _rotate_from_start(start_x: float, start_y: float, start_yaw: float, points: Iterable[PointXY]) -> List[PointXY]:
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    return [
        (start_x + cosine * x - sine * y, start_y + sine * x + cosine * y)
        for x, y in points
    ]


def _point_yaws(points: Sequence[PointXY], terminal_yaw: float) -> List[float]:
    if len(points) < 2:
        raise H0PathError("H0 path needs at least two samples")
    yaws: List[float] = []
    for index, (x, y) in enumerate(points):
        if index == len(points) - 1:
            yaws.append(wrap_angle(terminal_yaw))
            continue
        next_x, next_y = points[index + 1]
        yaws.append(math.atan2(next_y - y, next_x - x))
    return yaws


def build_h0_path_points(
    *,
    start_x: float,
    start_y: float,
    start_yaw: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    template: str,
    spacing_m: float,
    amplitude_ratio: float,
    min_amplitude_m: float,
    max_amplitude_m: float,
    side: str,
    smooth_iterations: int,
    start_heading: str,
) -> List[PointYaw]:
    """Build one H0-only runtime path in the sampled odometry frame.

    ``s_curve`` follows the old P2 shape contract (current-heading tangent,
    left/right S deflection and open-polyline smoothing).  It is implemented
    locally: no legacy proxy generator is imported or executed.
    """

    start_x = _require_finite("start_x", start_x)
    start_y = _require_finite("start_y", start_y)
    start_yaw = _require_finite("start_yaw", start_yaw)
    goal_x = _require_finite("goal_x", goal_x)
    goal_y = _require_finite("goal_y", goal_y)
    goal_yaw = _require_finite("goal_yaw", goal_yaw)
    template = str(template).strip().lower()
    start_heading = str(start_heading).strip().lower()
    side = str(side).strip().lower()
    if template not in SUPPORTED_H0_TEMPLATES:
        raise H0PathError(f"unsupported H0 runtime template: {template!r}")
    if start_heading != "current":
        raise H0PathError("H0 runtime path requires path_start_heading='current'")
    if side not in ("left", "right"):
        raise H0PathError("path_side must be 'left' or 'right'")
    if int(smooth_iterations) < 0 or int(smooth_iterations) > 8:
        raise H0PathError("path_smooth_iterations must be in [0, 8]")

    # Resolve the configured goal into the robot's sampled odometry tangent
    # frame.  A forward-only goal avoids accidentally turning this H0 route
    # into an implicit turn-in-place experiment.
    dx = goal_x - start_x
    dy = goal_y - start_y
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    goal_x_local = cosine * dx + sine * dy
    goal_y_local = -sine * dx + cosine * dy
    if goal_x_local < 0.30:
        raise H0PathError("H0 goal must be at least 0.30 m in front of the sampled odom pose")

    if template == "straight":
        local_control = [(0.0, 0.0), (goal_x_local, goal_y_local)]
    else:
        chord = math.hypot(goal_x_local, goal_y_local)
        amplitude_ratio = _require_finite("path_amplitude_ratio", amplitude_ratio)
        min_amplitude_m = _require_finite("path_min_amplitude", min_amplitude_m)
        max_amplitude_m = _require_finite("path_max_amplitude", max_amplitude_m)
        if not 0.0 <= amplitude_ratio <= 0.50:
            raise H0PathError("path_amplitude_ratio must be in [0, 0.50]")
        if not 0.0 <= min_amplitude_m <= max_amplitude_m <= 3.0:
            raise H0PathError("H0 path amplitudes must satisfy 0 <= min <= max <= 3 m")
        amplitude = min(max(chord * amplitude_ratio, min_amplitude_m), max_amplitude_m)
        if side == "right":
            amplitude = -amplitude
        local_control = [
            (0.0, 0.0),
            (0.25 * goal_x_local, amplitude),
            (0.50 * goal_x_local, 0.0),
            (0.75 * goal_x_local, -amplitude),
            (goal_x_local, 0.0),
        ]
        # Cubic blend preserves the current tangent and lands exactly at the
        # configured lateral goal coordinate.
        local_control = [
            (x, y + (max(0.0, min(1.0, x / goal_x_local)) ** 3) * goal_y_local)
            for x, y in local_control
        ]
        guard_x = min(0.50, max(0.10, 0.15 * goal_x_local))
        local_control.insert(1, (guard_x, 0.0))
        local_control = _chaikin_open(local_control, int(smooth_iterations))

    local_samples = _resample_polyline(local_control, spacing_m)
    world_samples = _rotate_from_start(start_x, start_y, start_yaw, local_samples)
    # Guard against round-off and preserve the requested endpoint exactly.
    world_samples[0] = (start_x, start_y)
    world_samples[-1] = (goal_x, goal_y)
    return [
        (x, y, yaw)
        for (x, y), yaw in zip(world_samples, _point_yaws(world_samples, goal_yaw))
    ]


class H0FixedPathPublisher:
    """ROS wrapper around the pure H0 geometry above."""

    def __init__(self, rospy: object, odometry_type: object, path_type: object, pose_stamped_type: object) -> None:
        self.rospy = rospy
        self.odometry_type = odometry_type
        self.path_type = path_type
        self.pose_stamped_type = pose_stamped_type
        self._validate_h0_admission()

        self.odom_topic = str(rospy.get_param("~odom_topic", "/odom"))
        self.output_topic = str(rospy.get_param("~output_topic", "/scout/global_path_fixed"))
        self.goal_topic = str(rospy.get_param("~goal_topic", "/scout/goal"))
        self.publish_goal = bool(rospy.get_param("~publish_goal", True))
        self.expected_odom_frame = str(rospy.get_param("~expected_odom_frame", "odom")).strip()
        self.odom_timeout_sec = _require_finite(
            "odom_timeout_sec", rospy.get_param("~odom_timeout_sec", 15.0)
        )
        if not 0.1 <= self.odom_timeout_sec <= 30.0:
            raise H0PathError("odom_timeout_sec must be in [0.1, 30] seconds")

        self.goal_x = _require_finite("goal_x", rospy.get_param("~goal_x", 5.0))
        self.goal_y = _require_finite("goal_y", rospy.get_param("~goal_y", 0.0))
        self.goal_yaw = _require_finite("goal_yaw", rospy.get_param("~goal_yaw", 0.0))
        self.template = str(rospy.get_param("~path_template", "s_curve"))
        self.spacing_m = _require_finite("spacing_m", rospy.get_param("~spacing_m", 0.05))
        self.amplitude_ratio = _require_finite(
            "path_amplitude_ratio", rospy.get_param("~path_amplitude_ratio", 0.18)
        )
        self.min_amplitude_m = _require_finite(
            "path_min_amplitude", rospy.get_param("~path_min_amplitude", 0.25)
        )
        self.max_amplitude_m = _require_finite(
            "path_max_amplitude", rospy.get_param("~path_max_amplitude", 1.20)
        )
        self.side = str(rospy.get_param("~path_side", "left"))
        self.smooth_iterations = int(rospy.get_param("~path_smooth_iterations", 3))
        self.start_heading = str(rospy.get_param("~path_start_heading", "current"))

        self.path_publisher = rospy.Publisher(self.output_topic, path_type, queue_size=1, latch=True)
        self.goal_publisher = (
            rospy.Publisher(self.goal_topic, pose_stamped_type, queue_size=1, latch=True)
            if self.publish_goal
            else None
        )

    def _validate_h0_admission(self) -> None:
        rospy = self.rospy
        if rospy.get_param("~h0_development_ack", False) is not True:
            raise H0PathError("refusing path generation: h0_development_ack must be boolean true")
        if str(rospy.get_param("~development_condition", "")).strip() != H0_DEVELOPMENT_CONDITION:
            raise H0PathError("refusing path generation: development_condition must be exactly H0")
        if rospy.get_param("/use_sim_time", False) is not True:
            raise H0PathError("refusing H0 path generation without /use_sim_time=true")

    def _wait_for_odom(self) -> object:
        received: List[object] = []
        ready = threading.Event()

        def callback(message: object) -> None:
            if not received:
                received.append(message)
                ready.set()

        subscriber = self.rospy.Subscriber(self.odom_topic, self.odometry_type, callback, queue_size=1)
        try:
            if not ready.wait(self.odom_timeout_sec):
                raise H0PathError(f"timed out waiting {self.odom_timeout_sec:g}s for {self.odom_topic}")
        finally:
            subscriber.unregister()
        return received[0]

    def _path_from_odom(self, odom: object) -> Tuple[object, object, List[PointYaw], str]:
        pose = odom.pose.pose
        start_x = _require_finite("odom pose x", pose.position.x)
        start_y = _require_finite("odom pose y", pose.position.y)
        start_yaw = yaw_from_quaternion(pose.orientation)
        frame_id = str(odom.header.frame_id).strip()
        if not frame_id:
            raise H0PathError("received /odom with an empty header.frame_id")
        if self.expected_odom_frame and frame_id != self.expected_odom_frame:
            raise H0PathError(
                f"received /odom frame {frame_id!r}; expected {self.expected_odom_frame!r} for H0 goal coordinates"
            )

        points = build_h0_path_points(
            start_x=start_x,
            start_y=start_y,
            start_yaw=start_yaw,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
            goal_yaw=self.goal_yaw,
            template=self.template,
            spacing_m=self.spacing_m,
            amplitude_ratio=self.amplitude_ratio,
            min_amplitude_m=self.min_amplitude_m,
            max_amplitude_m=self.max_amplitude_m,
            side=self.side,
            smooth_iterations=self.smooth_iterations,
            start_heading=self.start_heading,
        )
        stamp = self.rospy.Time.now()
        path = self.path_type()
        path.header.frame_id = frame_id
        path.header.stamp = stamp
        for x, y, yaw in points:
            pose_stamped = self.pose_stamped_type()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.header.stamp = stamp
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = math.sin(0.5 * yaw)
            pose_stamped.pose.orientation.w = math.cos(0.5 * yaw)
            path.poses.append(pose_stamped)

        goal = self.pose_stamped_type()
        goal.header.frame_id = frame_id
        goal.header.stamp = stamp
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(0.5 * self.goal_yaw)
        goal.pose.orientation.w = math.cos(0.5 * self.goal_yaw)
        return path, goal, points, frame_id

    def publish_and_spin(self) -> None:
        odom = self._wait_for_odom()
        path, goal, points, frame_id = self._path_from_odom(odom)
        self.path_publisher.publish(path)
        if self.goal_publisher is not None:
            self.goal_publisher.publish(goal)
        self.rospy.loginfo(
            "sim_h0_fixed_path_publisher: %s; template=%s frame=%s poses=%d "
            "goal=(%.3f, %.3f, %.3f); evidence=%s",
            self.output_topic,
            str(self.template).strip().lower(),
            frame_id,
            len(points),
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
            H0_EVIDENCE_CLASS,
        )
        # Remain alive so late recorder/controller subscribers receive the
        # latched H0 path.  This node has no motion side effect.
        self.rospy.spin()


def main() -> None:
    # Keep ROS imports here so the deterministic geometry is unit-testable
    # without a sourced ROS environment.
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import Path as NavPath

    rospy.init_node("sim_h0_fixed_path_publisher", anonymous=False)
    try:
        H0FixedPathPublisher(rospy, Odometry, NavPath, PoseStamped).publish_and_spin()
    except H0PathError as error:
        rospy.logfatal("sim_h0_fixed_path_publisher refused: %s", error)
        raise


if __name__ == "__main__":
    main()
