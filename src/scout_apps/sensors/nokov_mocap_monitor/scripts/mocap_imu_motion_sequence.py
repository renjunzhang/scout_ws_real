#!/usr/bin/env python3
"""Publish a deterministic real-robot motion sequence for mocap/IMU calibration.

The process owns one long-lived /cmd_vel publisher for the complete run.  This
avoids the startup and shutdown gaps caused by creating one ``rostopic pub``
process per segment.  Segment/status events are published by the same ROS node,
so event publication cannot extend a motion hold.
"""

import argparse
import datetime as dt
import math
import os
import signal
import sys
import threading
import time
from typing import Optional

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class SequenceAbort(RuntimeError):
    """Raised when an online safety/recording prerequisite is lost."""


class SequenceInterrupted(SequenceAbort):
    """Raised after SIGINT/SIGTERM requests a controlled zero-command exit."""


def positive_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return value


def nonnegative_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("must be a finite non-negative number")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-motion", default="NO")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--segment-topic", default="/mocap_imu_calib/segment")
    parser.add_argument("--status-topic", default="/mocap_imu_calib/status")
    parser.add_argument("--mocap-status-topic", default="/mocap/status")
    parser.add_argument("--mocap-tracker", default="Tracker0")
    parser.add_argument("--raw-mocap-pose-topic", default="/vrpn_client_node/Tracker0/pose")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--required-cmd-subscriber", default="/scout_base_node")
    parser.add_argument("--required-recorder-prefix", default="/record_")
    parser.add_argument("--timeline-path", required=True)
    parser.add_argument("--log-path", required=True)
    parser.add_argument("--recorder-pid", type=int, default=0)
    parser.add_argument("--connection-timeout", type=positive_float, default=12.0)
    parser.add_argument("--mocap-status-max-age", type=positive_float, default=3.0)
    parser.add_argument("--mocap-pose-max-age", type=positive_float, default=0.20)
    parser.add_argument("--imu-max-age", type=positive_float, default=0.20)
    parser.add_argument("--odom-max-age", type=positive_float, default=0.20)
    parser.add_argument("--command-lease-sec", type=positive_float, default=0.25)
    parser.add_argument("--max-publish-gap-sec", type=positive_float, default=0.10)
    parser.add_argument("--final-zero-sec", type=positive_float, default=1.0)

    parser.add_argument("--cmd-hz", type=positive_float, default=50.0)
    parser.add_argument("--countdown-sec", type=int, default=5)
    parser.add_argument("--linear-low", type=positive_float, default=0.10)
    parser.add_argument("--linear-nominal", type=positive_float, default=0.15)
    parser.add_argument("--straight-sec", type=positive_float, default=1.5)
    parser.add_argument("--spin-omega", type=positive_float, default=0.20)
    parser.add_argument("--spin-hold-sec", type=positive_float, default=5.0)
    parser.add_argument("--spin-rev-leg-sec", type=positive_float, default=3.0)
    parser.add_argument("--spin-rev-middle-sec", type=positive_float, default=6.0)
    parser.add_argument("--s-v", type=positive_float, default=0.10)
    parser.add_argument("--s-omega", type=positive_float, default=0.40)
    parser.add_argument("--s-hold-sec", type=positive_float, default=1.0)
    parser.add_argument("--s-repeats", type=int, default=3)
    parser.add_argument("--static-pre-sec", type=positive_float, default=60.0)
    parser.add_argument("--static-post-sec", type=positive_float, default=60.0)
    parser.add_argument("--settle-sec", type=positive_float, default=2.5)
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.arm_motion != "YES":
        raise ValueError("arm_motion must be exactly YES")
    if args.cmd_hz < 50.0:
        raise ValueError("cmd_hz must be >= 50 Hz (Scout SDK command contract)")
    if args.cmd_hz > 100.0:
        raise ValueError("cmd_hz must be <= 100 Hz")
    if args.command_lease_sec >= 0.50:
        raise ValueError("command_lease_sec must be < the 0.50 s base watchdog")
    if args.max_publish_gap_sec >= args.command_lease_sec:
        raise ValueError("max_publish_gap_sec must be < command_lease_sec")
    if args.countdown_sec < 0:
        raise ValueError("countdown_sec must be non-negative")
    if args.countdown_sec > 15:
        raise ValueError("countdown_sec must be <= 15")
    if args.s_repeats < 3 or args.s_repeats > 5:
        raise ValueError("s_repeats must be in [3, 5]")
    if args.static_pre_sec < 60.0 or args.static_post_sec < 60.0:
        raise ValueError("static_pre_sec and static_post_sec must each be >= 60 s")
    if args.static_pre_sec > 300.0 or args.static_post_sec > 300.0:
        raise ValueError("static_pre_sec and static_post_sec must each be <= 300 s")
    if args.linear_nominal < args.linear_low:
        raise ValueError("linear_nominal must be >= linear_low")

    for name in ("linear_low", "linear_nominal", "s_v"):
        if getattr(args, name) > 0.25:
            raise ValueError("{} exceeds the hard limit 0.25 m/s".format(name))
    for name in ("spin_omega", "s_omega"):
        if getattr(args, name) > 0.50:
            raise ValueError("{} exceeds the hard limit 0.50 rad/s".format(name))

    duration_limits = {
        "straight_sec": 2.0,
        "spin_hold_sec": 10.0,
        "spin_rev_leg_sec": 5.0,
        "spin_rev_middle_sec": 10.0,
        "s_hold_sec": 1.5,
        "settle_sec": 10.0,
        "final_zero_sec": 5.0,
        "connection_timeout": 30.0,
    }
    for name, maximum in duration_limits.items():
        if getattr(args, name) > maximum:
            raise ValueError("{} exceeds the hard limit {} s".format(name, maximum))

    straight_max_distance = args.linear_nominal * args.straight_sec
    s_pass_distance = 2.0 * args.s_v * args.s_hold_sec
    if straight_max_distance > 0.40:
        raise ValueError("nominal straight command distance exceeds 0.40 m")
    if s_pass_distance > 0.40:
        raise ValueError("S pass command distance exceeds 0.40 m")

    total_translation = (
        2.0 * (args.linear_low + args.linear_nominal) * args.straight_sec
        + 8.0 * args.s_v * args.s_hold_sec * args.s_repeats
    )
    total_yaw = (
        2.0 * args.spin_omega * args.spin_hold_sec
        + args.spin_omega * (2.0 * args.spin_rev_leg_sec + args.spin_rev_middle_sec)
        + 8.0 * args.s_omega * args.s_hold_sec * args.s_repeats
    )
    total_duration = (
        args.countdown_sec
        + args.static_pre_sec
        + args.static_post_sec
        + 4.0 * args.straight_sec
        + 4.0 * args.settle_sec
        + 2.0 * args.spin_hold_sec
        + 2.0 * args.spin_rev_leg_sec
        + args.spin_rev_middle_sec
        + 3.0 * args.settle_sec
        + args.s_repeats * (8.0 * args.s_hold_sec + 4.0 * args.settle_sec)
        + args.final_zero_sec
    )
    if total_translation > 6.0:
        raise ValueError("total commanded translation exceeds 6.0 m")
    if total_yaw > 30.0:
        raise ValueError("total commanded absolute yaw exceeds 30 rad")
    if total_duration > 600.0:
        raise ValueError("total sequence duration exceeds 600 s")


class MotionSequence:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.abort_requested = threading.Event()
        self.publisher_stop = threading.Event()
        self.command_lock = threading.Lock()
        self.publish_lock = threading.Lock()
        self.status_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_mocap_status = ""
        self.last_mocap_status_monotonic = 0.0
        self.mocap_pose_count = 0
        self.mocap_pose_last_receive_monotonic = 0.0
        self.mocap_pose_last_header_stamp = 0.0
        self.mocap_pose_max_receive_gap = 0.0
        self.mocap_pose_error = ""
        self.imu_count = 0
        self.imu_last_receive_monotonic = 0.0
        self.imu_last_header_stamp = 0.0
        self.imu_max_receive_gap = 0.0
        self.imu_error = ""
        self.odom_count = 0
        self.odom_last_receive_monotonic = 0.0
        self.odom_last_header_stamp = 0.0
        self.odom_max_receive_gap = 0.0
        self.odom_error = ""
        self.publisher_thread: Optional[threading.Thread] = None
        self.publisher_overruns = 0
        self.command_publish_count = 0
        self.last_command_publish_monotonic = 0.0
        self.max_command_publish_gap = 0.0
        self.command_lease_deadline = 0.0
        self.publisher_fault = ""
        self.last_master_check = 0.0
        self.abort_signal = 0
        self.sequence_complete = False

        self.timeline_file = open(args.timeline_path, "w", buffering=1, encoding="utf-8")
        self.timeline_file.write(
            "wall_time_iso\tros_time_sec\tmonotonic_sec\tevent\n"
        )
        self.log_file = open(args.log_path, "w", buffering=1, encoding="utf-8")

        rospy.init_node(
            "mocap_imu_motion_sequence", anonymous=True, disable_signals=True
        )
        self.node_name = rospy.get_name()
        self.master = rosgraph.Master(self.node_name)
        self.cmd_pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=1)
        self.segment_pub = rospy.Publisher(args.segment_topic, String, queue_size=20)
        self.status_pub = rospy.Publisher(
            args.status_topic, String, queue_size=5, latch=True
        )
        self.mocap_status_sub = rospy.Subscriber(
            args.mocap_status_topic,
            String,
            self._mocap_status_callback,
            queue_size=1,
        )
        self.mocap_pose_sub = rospy.Subscriber(
            args.raw_mocap_pose_topic,
            PoseStamped,
            self._mocap_pose_callback,
            queue_size=20,
        )
        self.imu_sub = rospy.Subscriber(
            args.imu_topic, Imu, self._imu_callback, queue_size=20
        )
        self.odom_sub = rospy.Subscriber(
            args.odom_topic, Odometry, self._odom_callback, queue_size=20
        )

    def log(self, message: str) -> None:
        line = "[mocap_imu_motion_sequence] {}".format(message)
        print(line, flush=True)
        self.log_file.write(
            "{}\t{}\n".format(dt.datetime.now().astimezone().isoformat(), line)
        )

    def _mocap_status_callback(self, message: String) -> None:
        with self.status_lock:
            self.last_mocap_status = message.data
            self.last_mocap_status_monotonic = time.monotonic()

    def _update_stream_health(self, prefix: str, stamp: float, values: tuple) -> None:
        now = time.monotonic()
        with self.data_lock:
            error_name = "{}_error".format(prefix)
            last_receive_name = "{}_last_receive_monotonic".format(prefix)
            last_stamp_name = "{}_last_header_stamp".format(prefix)
            max_gap_name = "{}_max_receive_gap".format(prefix)
            count_name = "{}_count".format(prefix)
            previous_receive = getattr(self, last_receive_name)
            previous_stamp = getattr(self, last_stamp_name)
            error = getattr(self, error_name)

            if not error and (not math.isfinite(stamp) or stamp <= 0.0):
                error = "{} has an invalid header timestamp".format(prefix)
            if not error and previous_stamp > 0.0 and stamp + 1e-9 < previous_stamp:
                error = "{} header timestamp regressed".format(prefix)
            if not error and not all(math.isfinite(value) for value in values):
                error = "{} contains a non-finite value".format(prefix)
            if previous_receive > 0.0:
                gap = now - previous_receive
                setattr(self, max_gap_name, max(getattr(self, max_gap_name), gap))
                max_allowed_gap = {
                    "mocap_pose": self.args.mocap_pose_max_age,
                    "imu": self.args.imu_max_age,
                    "odom": self.args.odom_max_age,
                }[prefix]
                if not error and gap > max_allowed_gap:
                    error = "{} receive gap {:.3f}s exceeded {:.3f}s".format(
                        prefix, gap, max_allowed_gap
                    )

            setattr(self, error_name, error)
            setattr(self, last_receive_name, now)
            setattr(self, last_stamp_name, stamp)
            setattr(self, count_name, getattr(self, count_name) + 1)

    def _mocap_pose_callback(self, message: PoseStamped) -> None:
        pose = message.pose
        self._update_stream_health(
            "mocap_pose",
            message.header.stamp.to_sec(),
            (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _imu_callback(self, message: Imu) -> None:
        acceleration = message.linear_acceleration
        angular = message.angular_velocity
        values = (
            acceleration.x,
            acceleration.y,
            acceleration.z,
            angular.x,
            angular.y,
            angular.z,
        )
        self._update_stream_health("imu", message.header.stamp.to_sec(), values)
        if sum(abs(value) for value in values) < 1e-6:
            with self.data_lock:
                if not self.imu_error:
                    self.imu_error = "imu acceleration and angular velocity are all zero"

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        self._update_stream_health(
            "odom",
            message.header.stamp.to_sec(),
            (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                twist.linear.y,
                twist.angular.z,
            ),
        )

    @staticmethod
    def _twist(linear: float, angular: float) -> Twist:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        return message

    def _publish_current_command(self) -> tuple:
        with self.publish_lock:
            applied_monotonic = time.monotonic()
            with self.command_lock:
                if self.last_command_publish_monotonic > 0.0:
                    gap = applied_monotonic - self.last_command_publish_monotonic
                    self.max_command_publish_gap = max(self.max_command_publish_gap, gap)
                    if gap > self.args.max_publish_gap_sec and not self.publisher_fault:
                        self.publisher_fault = (
                            "command publish gap {:.3f}s exceeded {:.3f}s".format(
                                gap, self.args.max_publish_gap_sec
                            )
                        )
                nonzero = (
                    abs(self.current_linear) > 1e-12
                    or abs(self.current_angular) > 1e-12
                )
                lease_expired = (
                    nonzero and applied_monotonic > self.command_lease_deadline
                )
                if lease_expired and not self.publisher_fault:
                    self.publisher_fault = (
                        "nonzero command lease expired; main sequence heartbeat stalled"
                    )
                if self.abort_requested.is_set() or self.publisher_fault or lease_expired:
                    self.current_linear = 0.0
                    self.current_angular = 0.0
                linear = self.current_linear
                angular = self.current_angular
            applied_ros_time = rospy.Time.now().to_sec()
            self.cmd_pub.publish(self._twist(linear, angular))
            with self.command_lock:
                self.last_command_publish_monotonic = applied_monotonic
                self.command_publish_count += 1
        return applied_monotonic, applied_ros_time, linear, angular

    def _command_publisher_loop(self) -> None:
        period = 1.0 / self.args.cmd_hz
        next_tick = time.monotonic()
        while not self.publisher_stop.is_set() and not rospy.is_shutdown():
            try:
                self._publish_current_command()
            except Exception as exc:  # Report a dead command path to the main loop.
                with self.command_lock:
                    self.current_linear = 0.0
                    self.current_angular = 0.0
                    if not self.publisher_fault:
                        self.publisher_fault = "command publish failed: {}".format(exc)
            next_tick += period
            now = time.monotonic()
            if next_tick <= now:
                self.publisher_overruns += 1
                next_tick = now + period
            self.publisher_stop.wait(max(0.0, next_tick - now))

    def start_command_publisher(self) -> None:
        self.publisher_thread = threading.Thread(
            target=self._command_publisher_loop,
            name="cmd_vel_{}_hz".format(int(self.args.cmd_hz)),
            daemon=True,
        )
        self.publisher_thread.start()

    def append_timeline(
        self,
        event: str,
        ros_time: Optional[float] = None,
        monotonic_time: Optional[float] = None,
    ) -> None:
        if ros_time is None:
            ros_time = rospy.Time.now().to_sec()
        if monotonic_time is None:
            monotonic_time = time.monotonic()
        wall_time = dt.datetime.now().astimezone().isoformat(timespec="microseconds")
        self.timeline_file.write(
            "{}\t{:.9f}\t{:.9f}\t{}\n".format(
                wall_time, ros_time, monotonic_time, event
            )
        )

    def publish_event(
        self,
        event: str,
        ros_time: Optional[float] = None,
        monotonic_time: Optional[float] = None,
    ) -> None:
        if ros_time is None:
            ros_time = rospy.Time.now().to_sec()
        if monotonic_time is None:
            monotonic_time = time.monotonic()
        self.segment_pub.publish(String(data=event))
        self.append_timeline(event, ros_time, monotonic_time)

    def publish_status(self, status: str) -> None:
        ros_time = rospy.Time.now().to_sec()
        self.status_pub.publish(String(data=status))
        self.append_timeline("STATUS|{}".format(status), ros_time, time.monotonic())

    def set_command(self, linear: float, angular: float, event: str = "") -> tuple:
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise SequenceAbort("refusing a non-finite motion command")
        if abs(linear) > 0.25 or abs(angular) > 0.50:
            raise SequenceAbort(
                "command exceeds hard envelope: v={}, omega={}".format(linear, angular)
            )
        requested_nonzero = abs(linear) > 1e-12 or abs(angular) > 1e-12
        if requested_nonzero:
            # Do not enter or switch a moving phase on stale prerequisites.
            self.refresh_command_lease()
            self.safety_check()
        with self.command_lock:
            if requested_nonzero and self.abort_requested.is_set():
                raise SequenceInterrupted("operator interrupt before command apply")
            if requested_nonzero and self.publisher_fault:
                raise SequenceAbort(self.publisher_fault)
            self.current_linear = linear
            self.current_angular = angular
            if requested_nonzero:
                self.command_lease_deadline = (
                    time.monotonic() + self.args.command_lease_sec
                )
            else:
                self.command_lease_deadline = 0.0
        # Publish the new value immediately; the long-lived thread maintains it.
        applied_monotonic, applied_ros_time, applied_linear, applied_angular = (
            self._publish_current_command()
        )
        if requested_nonzero and (
            abs(applied_linear - linear) > 1e-12
            or abs(applied_angular - angular) > 1e-12
        ):
            with self.command_lock:
                fault = self.publisher_fault
            if self.abort_requested.is_set():
                raise SequenceInterrupted("operator interrupt during command apply")
            raise SequenceAbort(fault or "nonzero command was rejected fail-closed")
        if event:
            self.publish_event(event, applied_ros_time, applied_monotonic)
        return applied_monotonic, applied_ros_time

    def refresh_command_lease(self) -> None:
        with self.command_lock:
            if abs(self.current_linear) > 1e-12 or abs(self.current_angular) > 1e-12:
                self.command_lease_deadline = (
                    time.monotonic() + self.args.command_lease_sec
                )

    def command_sample_count(self) -> int:
        with self.command_lock:
            return self.command_publish_count

    def finish_command_phase(
        self, label: str, start_monotonic: float, start_sample_count: int
    ) -> float:
        end_monotonic, end_ros_time = self.set_command(0.0, 0.0)
        samples = max(0, self.command_sample_count() - start_sample_count - 1)
        event = "END|{}|actual_duration={:.6f}|cmd_samples={}".format(
            label, end_monotonic - start_monotonic, samples
        )
        self.publish_event(event, end_ros_time, end_monotonic)
        return end_monotonic

    def request_abort(self, signum: int, _frame: object) -> None:
        # Keep the signal handler side-effect free; the main loop performs the
        # logging and controlled zero-command shutdown within at most 50 ms.
        self.abort_signal = signum
        self.abort_requested.set()

    def _check_recorder(self) -> None:
        if self.args.recorder_pid <= 0:
            return
        try:
            os.kill(self.args.recorder_pid, 0)
        except ProcessLookupError as exc:
            raise SequenceAbort("recorder process exited") from exc
        except PermissionError as exc:
            raise SequenceAbort("cannot inspect recorder process") from exc

    def _check_mocap(self) -> None:
        with self.status_lock:
            status = self.last_mocap_status
            age = time.monotonic() - self.last_mocap_status_monotonic
        tokens = status.split()
        fields = {
            key: value
            for token in tokens[1:]
            if "=" in token
            for key, value in (token.split("=", 1),)
        }
        if not tokens or tokens[0] != "OK" or fields.get("tracker") != self.args.mocap_tracker:
            raise SequenceAbort("mocap status is not OK: {}".format(status or "missing"))
        if age > self.args.mocap_status_max_age:
            raise SequenceAbort("mocap status is stale ({:.2f}s)".format(age))

    def _check_data_health(self) -> None:
        now = time.monotonic()
        with self.data_lock:
            streams = (
                (
                    "mocap pose",
                    self.mocap_pose_count,
                    self.mocap_pose_last_receive_monotonic,
                    self.args.mocap_pose_max_age,
                    self.mocap_pose_error,
                ),
                (
                    "IMU",
                    self.imu_count,
                    self.imu_last_receive_monotonic,
                    self.args.imu_max_age,
                    self.imu_error,
                ),
                (
                    "odometry",
                    self.odom_count,
                    self.odom_last_receive_monotonic,
                    self.args.odom_max_age,
                    self.odom_error,
                ),
            )
        for label, count, last_receive, max_age, error in streams:
            if error:
                raise SequenceAbort(error)
            if count <= 0 or last_receive <= 0.0:
                raise SequenceAbort("{} stream has no samples".format(label))
            age = now - last_receive
            if age > max_age:
                raise SequenceAbort("{} stream is stale ({:.3f}s)".format(label, age))

    @staticmethod
    def _nodes_for_topic(system_state: list, wanted_topic: str) -> list:
        for topic, nodes in system_state:
            if topic == wanted_topic:
                return nodes
        return []

    def _recorder_nodes(self, subscribers: list) -> set:
        topics = (
            self.args.cmd_topic,
            self.args.segment_topic,
            self.args.status_topic,
            self.args.imu_topic,
            self.args.odom_topic,
            self.args.raw_mocap_pose_topic,
        )
        node_sets = []
        for topic in topics:
            nodes = self._nodes_for_topic(subscribers, topic)
            node_sets.append(
                {
                    node
                    for node in nodes
                    if node.startswith(self.args.required_recorder_prefix)
                }
            )
        return set.intersection(*node_sets) if node_sets else set()

    def _check_master_state(self) -> None:
        now = time.monotonic()
        if now - self.last_master_check < 0.5:
            return
        self.last_master_check = now
        publishers, subscribers, _services = self.master.getSystemState()
        cmd_publishers = self._nodes_for_topic(publishers, self.args.cmd_topic)
        unexpected = [node for node in cmd_publishers if node != self.node_name]
        if unexpected:
            raise SequenceAbort(
                "unexpected {} publisher(s): {}".format(
                    self.args.cmd_topic, ", ".join(unexpected)
                )
            )
        cmd_subscribers = self._nodes_for_topic(subscribers, self.args.cmd_topic)
        if self.args.required_cmd_subscriber not in cmd_subscribers:
            raise SequenceAbort(
                "required command subscriber disappeared: {}".format(
                    self.args.required_cmd_subscriber
                )
            )
        if not self._recorder_nodes(subscribers):
            raise SequenceAbort("rosbag is no longer subscribed to command/event topics")
        if self.cmd_pub.get_num_connections() < 2:
            raise SequenceAbort("command TCPROS connections dropped below base + rosbag")
        if self.segment_pub.get_num_connections() < 1:
            raise SequenceAbort("segment event TCPROS connection was lost")
        if self.status_pub.get_num_connections() < 1:
            raise SequenceAbort("status event TCPROS connection was lost")

    def safety_check(self) -> None:
        if self.abort_requested.is_set():
            raise SequenceInterrupted(
                "operator interrupt (signal {})".format(self.abort_signal or "unknown")
            )
        if rospy.is_shutdown():
            raise SequenceInterrupted("ROS shutdown")
        with self.command_lock:
            publisher_fault = self.publisher_fault
        if publisher_fault:
            raise SequenceAbort(publisher_fault)
        if self.publisher_thread is None or not self.publisher_thread.is_alive():
            raise SequenceAbort("command publisher thread is not alive")
        self._check_recorder()
        self._check_mocap()
        self._check_data_health()
        self._check_master_state()
        with self.command_lock:
            publisher_fault = self.publisher_fault
        if publisher_fault:
            raise SequenceAbort(publisher_fault)

    def interruptible_hold(self, duration: float, start_time: Optional[float] = None) -> None:
        if start_time is None:
            start_time = time.monotonic()
        deadline = start_time + duration
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            self.refresh_command_lease()
            self.safety_check()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            self.abort_requested.wait(min(0.02, remaining))

    def wait_for_connections(self) -> None:
        deadline = time.monotonic() + self.args.connection_timeout
        while True:
            _publishers, subscribers, _services = self.master.getSystemState()
            missing = []
            cmd_subscribers = self._nodes_for_topic(subscribers, self.args.cmd_topic)
            if self.args.required_cmd_subscriber not in cmd_subscribers:
                missing.append(
                    "{} (required subscriber {} is absent)".format(
                        self.args.cmd_topic, self.args.required_cmd_subscriber
                    )
                )
            recorder_nodes = self._recorder_nodes(subscribers)
            if not recorder_nodes:
                missing.append(
                    "recorder {}* subscribed to {}, {}, and {}".format(
                        self.args.required_recorder_prefix,
                        self.args.cmd_topic,
                        self.args.segment_topic,
                        self.args.status_topic,
                    )
                )
            if self.cmd_pub.get_num_connections() < 2:
                missing.append("{} TCPROS base + recorder connections".format(self.args.cmd_topic))
            if self.segment_pub.get_num_connections() < 1:
                missing.append("{} TCPROS recorder connection".format(self.args.segment_topic))
            if self.status_pub.get_num_connections() < 1:
                missing.append("{} TCPROS recorder connection".format(self.args.status_topic))
            with self.status_lock:
                have_mocap_status = bool(self.last_mocap_status)
            with self.data_lock:
                have_required_data = (
                    self.mocap_pose_count > 0 and self.imu_count > 0 and self.odom_count > 0
                )
            if not missing and have_mocap_status and have_required_data:
                break
            if self.abort_requested.is_set():
                raise SequenceInterrupted("operator interrupt during publisher startup")
            if time.monotonic() >= deadline:
                if not have_mocap_status:
                    missing.append(self.args.mocap_status_topic + " (mocap status)")
                if not have_required_data:
                    missing.append("mocap pose + IMU + odometry samples")
                raise SequenceAbort("startup connection timeout: {}".format(", ".join(missing)))
            time.sleep(0.05)
        self.safety_check()
        self.log(
            "base command, rosbag command/event, and mocap connections are ready"
        )

    def run_segment(self, label: str, linear: float, angular: float, duration: float) -> None:
        event = "START|{}|v={}|omega={}|duration={}".format(
            label, linear, angular, duration
        )
        self.log("{}: v={}, omega={}, duration={}s".format(label, linear, angular, duration))
        start_samples = self.command_sample_count()
        start_monotonic, _start_ros_time = self.set_command(linear, angular, event)
        self.interruptible_hold(duration, start_monotonic)
        self.finish_command_phase(label, start_monotonic, start_samples)

    def hold_zero(self, label: str, duration: float) -> None:
        event = "START|{}|v=0|omega=0|duration={}".format(label, duration)
        self.log("{}: zero command for {}s".format(label, duration))
        start_samples = self.command_sample_count()
        start_monotonic, _start_ros_time = self.set_command(0.0, 0.0, event)
        self.interruptible_hold(duration, start_monotonic)
        end_monotonic = time.monotonic()
        samples = max(0, self.command_sample_count() - start_samples)
        self.publish_event(
            "END|{}|actual_duration={:.6f}|cmd_samples={}".format(
                label, end_monotonic - start_monotonic, samples
            ),
            monotonic_time=end_monotonic,
        )

    def switch_command(
        self,
        label: str,
        linear: float,
        angular: float,
        previous_start_monotonic: float,
    ) -> float:
        applied_monotonic, applied_ros_time = self.set_command(linear, angular)
        event = "SWITCH|{}|v={}|omega={}|previous_actual_duration={:.6f}".format(
            label, linear, angular, applied_monotonic - previous_start_monotonic
        )
        self.publish_event(event, applied_ros_time, applied_monotonic)
        return applied_monotonic

    def run_spin_reversal_sequence(self) -> None:
        self.log("in-place direct reversal: CCW -> CW -> CCW")
        start_samples = self.command_sample_count()
        first_start, _ = self.set_command(
            0.0, self.args.spin_omega, "START|spin_direct_reversal"
        )
        self.interruptible_hold(self.args.spin_rev_leg_sec, first_start)
        middle_start = self.switch_command(
            "spin_reversal_LR", 0.0, -self.args.spin_omega, first_start
        )
        self.interruptible_hold(self.args.spin_rev_middle_sec, middle_start)
        final_start = self.switch_command(
            "spin_reversal_RL", 0.0, self.args.spin_omega, middle_start
        )
        self.interruptible_hold(self.args.spin_rev_leg_sec, final_start)
        self.finish_command_phase("spin_direct_reversal", first_start, start_samples)

    def run_s_pattern_round_trip(self, pattern: str, repetition: int) -> None:
        if pattern == "LR":
            first_omega = self.args.s_omega
            second_omega = -self.args.s_omega
        elif pattern == "RL":
            first_omega = -self.args.s_omega
            second_omega = self.args.s_omega
        else:
            raise SequenceAbort("unsupported S pattern: {}".format(pattern))

        prefix = "s_{}_r{}".format(pattern, repetition)
        self.log("S-{} repetition {}: continuous two-curvature holds".format(pattern, repetition))
        forward_label = "{}_forward".format(prefix)
        forward_samples = self.command_sample_count()
        forward_start, _ = self.set_command(
            self.args.s_v,
            first_omega,
            "START|{}|v={}|omega={}|leg_duration={}|total_duration={}".format(
                forward_label,
                self.args.s_v,
                first_omega,
                self.args.s_hold_sec,
                2.0 * self.args.s_hold_sec,
            ),
        )
        self.interruptible_hold(self.args.s_hold_sec, forward_start)
        forward_second_start = self.switch_command(
            "{}_reversal".format(forward_label),
            self.args.s_v,
            second_omega,
            forward_start,
        )
        self.interruptible_hold(self.args.s_hold_sec, forward_second_start)
        self.finish_command_phase(forward_label, forward_start, forward_samples)

        self.hold_zero("{}_turnaround_settle".format(prefix), self.args.settle_sec)

        return_label = "{}_return".format(prefix)
        return_samples = self.command_sample_count()
        return_start, _ = self.set_command(
            -self.args.s_v,
            first_omega,
            "START|{}|v={}|omega={}|leg_duration={}|total_duration={}".format(
                return_label,
                -self.args.s_v,
                first_omega,
                self.args.s_hold_sec,
                2.0 * self.args.s_hold_sec,
            ),
        )
        self.interruptible_hold(self.args.s_hold_sec, return_start)
        return_second_start = self.switch_command(
            "{}_reversal".format(return_label),
            -self.args.s_v,
            second_omega,
            return_start,
        )
        self.interruptible_hold(self.args.s_hold_sec, return_second_start)
        self.finish_command_phase(return_label, return_start, return_samples)

    def run(self) -> None:
        self.start_command_publisher()
        self.wait_for_connections()
        self.publish_status("READY")
        self.set_command(0.0, 0.0)

        for remaining in range(self.args.countdown_sec, 0, -1):
            self.log("starting in {}s".format(remaining))
            self.interruptible_hold(1.0)

        self.publish_status("RUNNING")
        self.publish_event("SEQUENCE_START")
        self.hold_zero("static_pre", self.args.static_pre_sec)

        self.run_segment("straight_low_forward", self.args.linear_low, 0.0, self.args.straight_sec)
        self.hold_zero("settle_after_straight_low_forward", self.args.settle_sec)
        self.run_segment("straight_low_reverse", -self.args.linear_low, 0.0, self.args.straight_sec)
        self.hold_zero("settle_after_straight_low_reverse", self.args.settle_sec)

        self.run_segment(
            "straight_nominal_forward", self.args.linear_nominal, 0.0, self.args.straight_sec
        )
        self.hold_zero("settle_after_straight_nominal_forward", self.args.settle_sec)
        self.run_segment(
            "straight_nominal_reverse", -self.args.linear_nominal, 0.0, self.args.straight_sec
        )
        self.hold_zero("settle_after_straight_nominal_reverse", self.args.settle_sec)

        self.run_segment("spin_ccw_hold", 0.0, self.args.spin_omega, self.args.spin_hold_sec)
        self.hold_zero("settle_after_spin_ccw", self.args.settle_sec)
        self.run_segment("spin_cw_hold", 0.0, -self.args.spin_omega, self.args.spin_hold_sec)
        self.hold_zero("settle_after_spin_cw", self.args.settle_sec)
        self.run_spin_reversal_sequence()
        self.hold_zero("settle_after_spin_reversal", self.args.settle_sec)

        for repetition in range(1, self.args.s_repeats + 1):
            self.run_s_pattern_round_trip("LR", repetition)
            self.hold_zero(
                "settle_after_s_LR_r{}".format(repetition), self.args.settle_sec
            )
            self.run_s_pattern_round_trip("RL", repetition)
            self.hold_zero(
                "settle_after_s_RL_r{}".format(repetition), self.args.settle_sec
            )

        self.hold_zero("static_post", self.args.static_post_sec)
        self.publish_event("SEQUENCE_END")
        self.publish_status("COMPLETE")
        self.sequence_complete = True

    def force_zero_and_shutdown(self, aborted_reason: str = "") -> None:
        try:
            try:
                self.set_command(0.0, 0.0)
            except Exception as exc:
                self.log("WARN: immediate final zero publish failed: {}".format(exc))
            if aborted_reason and not self.sequence_complete:
                safe_reason = aborted_reason.replace("\n", " ").replace("\t", " ")
                try:
                    self.publish_event("SEQUENCE_ABORT|{}".format(safe_reason))
                    self.publish_status("ABORTED|{}".format(safe_reason))
                except Exception as exc:
                    self.log("WARN: abort marker publish failed: {}".format(exc))
            deadline = time.monotonic() + self.args.final_zero_sec
            while time.monotonic() < deadline and not rospy.is_shutdown():
                time.sleep(max(0.0, min(0.05, deadline - time.monotonic())))
        finally:
            self.publisher_stop.set()
            if self.publisher_thread is not None:
                self.publisher_thread.join(timeout=2.0)
            self.log(
                "command publisher stopped; samples={}, max_gap={:.6f}s, overruns={}".format(
                    self.command_publish_count,
                    self.max_command_publish_gap,
                    self.publisher_overruns,
                )
            )
            self.log(
                "record inputs: mocap_pose={} max_gap={:.6f}s; imu={} max_gap={:.6f}s; odom={} max_gap={:.6f}s".format(
                    self.mocap_pose_count,
                    self.mocap_pose_max_receive_gap,
                    self.imu_count,
                    self.imu_max_receive_gap,
                    self.odom_count,
                    self.odom_max_receive_gap,
                )
            )
            self.timeline_file.close()
            self.log_file.close()


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    sequence: Optional[MotionSequence] = None
    reason = ""
    rc = 1
    try:
        sequence = MotionSequence(args)
        signal.signal(signal.SIGINT, sequence.request_abort)
        signal.signal(signal.SIGTERM, sequence.request_abort)
        sequence.run()
        rc = 0
    except SequenceInterrupted as exc:
        reason = str(exc)
        if sequence is not None:
            sequence.log("interrupted: {}".format(reason))
        rc = 130
    except (SequenceAbort, rosgraph.MasterException, rospy.ROSException) as exc:
        reason = str(exc)
        if sequence is not None:
            sequence.log("ABORT: {}".format(reason))
        else:
            print("[mocap_imu_motion_sequence] ABORT: {}".format(reason), file=sys.stderr)
        rc = 1
    except Exception as exc:  # Keep the real robot zeroing path broad.
        reason = "unexpected error: {}".format(exc)
        if sequence is not None:
            sequence.log("ABORT: {}".format(reason))
        else:
            print("[mocap_imu_motion_sequence] ABORT: {}".format(reason), file=sys.stderr)
        rc = 1
    finally:
        if sequence is not None:
            sequence.force_zero_and_shutdown(reason)
    return rc


if __name__ == "__main__":
    sys.exit(main())
