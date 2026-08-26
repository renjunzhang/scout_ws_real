#!/usr/bin/env python3
"""Fail-closed postflight for an S-MPCC + NOKOV execution-chain bag."""

import argparse
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path


AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"
SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"
CMD_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
IMU_TOPIC = "/imu/data"
MOCAP_POSE_TOPIC = "/mocap/scout_pose"
MOCAP_STATUS_TOPIC = "/mocap/status"
PATH_TOPICS = (
    "/scout/global_path_fixed",
    "/mpc/reference_path",
    "/scout/global_path",
)
IMAGE_TYPES = {"sensor_msgs/Image", "sensor_msgs/CompressedImage"}


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stamp_sec(stamp):
    try:
        return float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return 0.0


def header_stamp(message):
    header = getattr(message, "header", None)
    return stamp_sec(getattr(header, "stamp", None))


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    position = max(0.0, min(1.0, fraction)) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def numeric_summary(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {"count": 0}
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "p95_abs": percentile([abs(value) for value in finite], 0.95),
        "max_abs": max(abs(value) for value in finite),
    }


class StreamStats:
    def __init__(self, use_header):
        self.use_header = bool(use_header)
        self.bag_times = []
        self.header_times = []
        self.invalid_values = 0
        self.invalid_header_stamps = 0
        self.header_regressions = 0
        self.last_header = None

    def update(self, bag_time, header_time, values):
        bag_time = float(bag_time)
        self.bag_times.append(bag_time)
        if not all(math.isfinite(float(value)) for value in values):
            self.invalid_values += 1
        if self.use_header:
            header_time = float(header_time)
            self.header_times.append(header_time)
            if not math.isfinite(header_time) or header_time <= 0.0:
                self.invalid_header_stamps += 1
            else:
                if self.last_header is not None and header_time < self.last_header - 1e-9:
                    self.header_regressions += 1
                self.last_header = header_time

    @staticmethod
    def _gaps(times):
        return [second - first for first, second in zip(times[:-1], times[1:]) if second >= first]

    def summary(self):
        duration = self.bag_times[-1] - self.bag_times[0] if len(self.bag_times) > 1 else 0.0
        rate = (len(self.bag_times) - 1) / duration if duration > 0.0 else 0.0
        bag_gaps = self._gaps(self.bag_times)
        header_gaps = self._gaps(self.header_times)
        return {
            "count": len(self.bag_times),
            "duration_sec": duration,
            "rate_hz": rate,
            "bag_gap_median_sec": percentile(bag_gaps, 0.5),
            "bag_gap_p95_sec": percentile(bag_gaps, 0.95),
            "bag_gap_max_sec": max(bag_gaps) if bag_gaps else 0.0,
            "header_gap_median_sec": percentile(header_gaps, 0.5),
            "header_gap_p95_sec": percentile(header_gaps, 0.95),
            "header_gap_max_sec": max(header_gaps) if header_gaps else 0.0,
            "invalid_values": self.invalid_values,
            "invalid_header_stamps": self.invalid_header_stamps,
            "header_regressions": self.header_regressions,
        }


def stationary_pose_metrics(rows):
    if not rows:
        return {"count": 0, "finite_count": 0, "nonfinite_count": 0}
    total_count = len(rows)
    finite_rows = [
        row for row in rows if all(math.isfinite(float(value)) for value in row)
    ]
    if not finite_rows:
        return {
            "count": total_count,
            "finite_count": 0,
            "nonfinite_count": total_count,
        }
    rows = finite_rows
    xs = [row[0] for row in rows]
    ys = [row[1] for row in rows]
    zs = [row[2] for row in rows]
    yaws = [row[3] for row in rows]
    quaternion_norms = [row[4] for row in rows]
    center_x = statistics.median(xs)
    center_y = statistics.median(ys)
    center_z = statistics.median(zs)
    center_yaw = math.atan2(
        statistics.fmean(math.sin(value) for value in yaws),
        statistics.fmean(math.cos(value) for value in yaws),
    )
    position_deviation = [
        math.sqrt(
            (x - center_x) ** 2 + (y - center_y) ** 2 + (z - center_z) ** 2
        )
        for x, y, z in zip(xs, ys, zs)
    ]
    yaw_deviation = [abs(wrap_angle(value - center_yaw)) for value in yaws]
    position_steps = [
        math.sqrt(
            (second[0] - first[0]) ** 2
            + (second[1] - first[1]) ** 2
            + (second[2] - first[2]) ** 2
        )
        for first, second in zip(rows[:-1], rows[1:])
    ]
    yaw_steps = [
        abs(wrap_angle(second[3] - first[3]))
        for first, second in zip(rows[:-1], rows[1:])
    ]
    norm_errors = [abs(value - 1.0) for value in quaternion_norms]
    return {
        "count": total_count,
        "finite_count": len(finite_rows),
        "nonfinite_count": total_count - len(finite_rows),
        "median_position_m": {"x": center_x, "y": center_y, "z": center_z},
        "circular_mean_yaw_rad": center_yaw,
        "position_deviation_p95_m": percentile(position_deviation, 0.95),
        "position_deviation_max_m": max(position_deviation),
        "position_step_p95_m": percentile(position_steps, 0.95),
        "position_step_max_m": max(position_steps) if position_steps else 0.0,
        "yaw_deviation_p95_rad": percentile(yaw_deviation, 0.95),
        "yaw_deviation_max_rad": max(yaw_deviation),
        "yaw_step_p95_rad": percentile(yaw_steps, 0.95),
        "yaw_step_max_rad": max(yaw_steps) if yaw_steps else 0.0,
        "quaternion_norm_error_p95": percentile(norm_errors, 0.95),
        "quaternion_norm_error_max": max(norm_errors),
    }


def path_points_from_json(path):
    payload = json.loads(path.read_text(encoding="utf-8"))
    return [
        (float(item["x"]), float(item["y"]))
        for item in payload.get("poses", [])
    ]


def path_points_from_message(message):
    return [
        (float(item.pose.position.x), float(item.pose.position.y))
        for item in message.poses
    ]


def compare_paths(expected, actual):
    if len(expected) != len(actual):
        return False, float("inf")
    if not expected:
        return False, float("inf")
    errors = [math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(expected, actual)]
    return max(errors) <= 1e-6, max(errors)


def validate_stream(label, stream, minimum_rate, maximum_gap, failures):
    summary = stream.summary()
    if summary["count"] == 0:
        failures.append("{} has no messages".format(label))
        return summary
    if summary["rate_hz"] < minimum_rate:
        failures.append(
            "{} rate {:.2f}Hz < {:.2f}Hz".format(label, summary["rate_hz"], minimum_rate)
        )
    if summary["bag_gap_max_sec"] > maximum_gap + 1e-9:
        failures.append(
            "{} max bag gap {:.3f}s > {:.3f}s".format(
                label, summary["bag_gap_max_sec"], maximum_gap
            )
        )
    if stream.use_header and summary["header_gap_max_sec"] > maximum_gap + 1e-9:
        failures.append(
            "{} max header gap {:.3f}s > {:.3f}s".format(
                label, summary["header_gap_max_sec"], maximum_gap
            )
        )
    if summary["invalid_values"]:
        failures.append("{} contains non-finite values".format(label))
    if summary["invalid_header_stamps"] or summary["header_regressions"]:
        failures.append("{} has invalid/non-monotonic header timestamps".format(label))
    return summary


def validate(args):
    try:
        import rosbag
    except ImportError as exc:
        print("rosbag is required; source the ROS workspace first: {}".format(exc), file=sys.stderr)
        return 2

    trial_mode = args.mode == "trial"
    bag_path = Path(args.bag).expanduser().resolve()
    report_path = (
        Path(args.report).expanduser().resolve()
        if args.report
        else bag_path.with_name(
            bag_path.stem
            + (
                "_mocap_execution_postflight.json"
                if trial_mode
                else "_mocap_static_smoke_postflight.json"
            )
        )
    )
    failures = []
    if not bag_path.is_file():
        failures.append("bag is missing")
    if Path(str(bag_path) + ".active").exists():
        failures.append("bag still has a .active sibling")

    path_file = None
    path_digest = ""
    if trial_mode:
        if not args.variant:
            failures.append("--variant is required in trial mode")
        if not args.path_file:
            failures.append("--path-file is required in trial mode")
        else:
            path_file = Path(args.path_file).expanduser().resolve()
            if not path_file.is_file():
                failures.append("frozen path is missing")
            else:
                path_digest = sha256_file(path_file)
        if not args.path_sha256:
            failures.append("--path-sha256 is required in trial mode")
        elif path_digest.lower() != args.path_sha256.lower():
            failures.append(
                "path SHA-256 mismatch: expected={}, actual={}".format(
                    args.path_sha256, path_digest
                )
            )

    streams = {
        "audit": StreamStats(True),
        "cmd": StreamStats(False),
        "odom": StreamStats(True),
        "imu": StreamStats(True),
        "raw_mocap": StreamStats(True),
        "bridge_mocap": StreamStats(True),
    }
    raw_mocap_topic = "/vrpn_client_node/{}/pose".format(args.mocap_tracker)
    audits = []
    valid_horizon_ids = set()
    valid_snapshot_ids = set()
    statuses = []
    path_messages = []
    raw_mocap_arrival_minus_header = []
    raw_pose_rows = []
    available = set()
    present_path_topics = []
    image_topics = []

    if bag_path.is_file():
        try:
            with rosbag.Bag(str(bag_path), "r") as bag:
                topic_info = bag.get_type_and_topic_info().topics
                available = set(topic_info.keys())
                required = {
                    ODOM_TOPIC,
                    args.imu_topic,
                    raw_mocap_topic,
                    MOCAP_POSE_TOPIC,
                    MOCAP_STATUS_TOPIC,
                }
                if trial_mode:
                    required.update(
                        {AUDIT_TOPIC, HORIZON_TOPIC, SNAPSHOT_TOPIC, CMD_TOPIC}
                    )
                for topic in sorted(required - available):
                    failures.append("missing required topic {}".format(topic))
                if trial_mode:
                    present_path_topics = [
                        topic for topic in PATH_TOPICS if topic in available
                    ]
                    if not present_path_topics:
                        failures.append(
                            "none of the frozen/reference path topics are present"
                        )
                image_topics = sorted(
                    topic for topic, info in topic_info.items() if info.msg_type in IMAGE_TYPES
                )
                if image_topics:
                    failures.append("image topics recorded: " + ",".join(image_topics))

                wanted = sorted(required | set(present_path_topics))
                for topic, message, bag_stamp in bag.read_messages(topics=wanted):
                    bag_time = bag_stamp.to_sec()
                    if topic == AUDIT_TOPIC:
                        streams["audit"].update(
                            bag_time,
                            header_stamp(message),
                            (
                                message.solver_cmd_v,
                                message.solver_cmd_omega,
                                message.published_cmd_v,
                                message.published_cmd_omega,
                            ),
                        )
                        audits.append(message)
                    elif topic == HORIZON_TOPIC and message.valid:
                        valid_horizon_ids.add(int(message.cycle_id))
                    elif topic == SNAPSHOT_TOPIC and message.valid:
                        valid_snapshot_ids.add(int(message.cycle_id))
                    elif topic == CMD_TOPIC:
                        streams["cmd"].update(
                            bag_time, 0.0, (message.linear.x, message.angular.z)
                        )
                    elif topic == ODOM_TOPIC:
                        twist = message.twist.twist
                        streams["odom"].update(
                            bag_time,
                            header_stamp(message),
                            (twist.linear.x, twist.angular.z),
                        )
                    elif topic == args.imu_topic:
                        accel = message.linear_acceleration
                        gyro = message.angular_velocity
                        streams["imu"].update(
                            bag_time,
                            header_stamp(message),
                            (accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z),
                        )
                    elif topic == raw_mocap_topic:
                        pose = message.pose
                        header_time = header_stamp(message)
                        quaternion = pose.orientation
                        streams["raw_mocap"].update(
                            bag_time,
                            header_time,
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
                        quaternion_norm = math.sqrt(
                            quaternion.x * quaternion.x
                            + quaternion.y * quaternion.y
                            + quaternion.z * quaternion.z
                            + quaternion.w * quaternion.w
                        )
                        raw_pose_rows.append(
                            (
                                float(pose.position.x),
                                float(pose.position.y),
                                float(pose.position.z),
                                quaternion_yaw(quaternion),
                                quaternion_norm,
                            )
                        )
                        if header_time > 0.0:
                            raw_mocap_arrival_minus_header.append(
                                bag_time - header_time
                            )
                    elif topic == MOCAP_POSE_TOPIC:
                        pose = message.pose
                        streams["bridge_mocap"].update(
                            bag_time,
                            header_stamp(message),
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
                    elif topic == MOCAP_STATUS_TOPIC:
                        statuses.append(str(message.data))
                    elif topic in present_path_topics and getattr(message, "poses", None):
                        path_messages.append((topic, message))
        except Exception as exc:  # rosbag exceptions vary by ROS distribution.
            failures.append("failed to read bag: {}".format(exc))

    stream_reports = {
        "odom": validate_stream(
            "odom", streams["odom"], args.min_odom_rate_hz, args.max_sensor_gap_sec, failures
        ),
        "imu": validate_stream(
            "imu", streams["imu"], args.min_imu_rate_hz, args.max_sensor_gap_sec, failures
        ),
        "raw_mocap": validate_stream(
            "raw_mocap", streams["raw_mocap"], args.min_mocap_rate_hz, args.max_mocap_gap_sec, failures
        ),
        "bridge_mocap": validate_stream(
            "bridge_mocap", streams["bridge_mocap"], args.min_mocap_rate_hz, args.max_mocap_gap_sec, failures
        ),
    }
    if trial_mode:
        stream_reports = {
            "audit": validate_stream(
                "audit",
                streams["audit"],
                args.min_control_rate_hz,
                args.max_control_gap_sec,
                failures,
            ),
            "cmd": validate_stream(
                "cmd",
                streams["cmd"],
                args.min_control_rate_hz,
                args.max_control_gap_sec,
                failures,
            ),
            **stream_reports,
        }

    raw_stream = stream_reports["raw_mocap"]
    if raw_stream["count"] and raw_stream["duration_sec"] < args.min_duration_sec:
        failures.append(
            "raw mocap duration {:.2f}s < {:.2f}s".format(
                raw_stream["duration_sec"], args.min_duration_sec
            )
        )

    pose_quality = stationary_pose_metrics(raw_pose_rows)
    if pose_quality.get("finite_count", 0):
        if (
            args.max_mocap_position_step_m > 0.0
            and pose_quality["position_step_max_m"] > args.max_mocap_position_step_m
        ):
            failures.append(
                "raw mocap position step {:.4f}m > {:.4f}m".format(
                    pose_quality["position_step_max_m"],
                    args.max_mocap_position_step_m,
                )
            )
        if (
            args.max_mocap_yaw_step_rad > 0.0
            and pose_quality["yaw_step_max_rad"] > args.max_mocap_yaw_step_rad
        ):
            failures.append(
                "raw mocap yaw step {:.4f}rad > {:.4f}rad".format(
                    pose_quality["yaw_step_max_rad"], args.max_mocap_yaw_step_rad
                )
            )
        if (
            args.max_quaternion_norm_error > 0.0
            and pose_quality["quaternion_norm_error_max"]
            > args.max_quaternion_norm_error
        ):
            failures.append(
                "raw mocap quaternion norm error {:.4f} > {:.4f}".format(
                    pose_quality["quaternion_norm_error_max"],
                    args.max_quaternion_norm_error,
                )
            )
        if not trial_mode:
            if (
                args.max_static_position_p95_m > 0.0
                and pose_quality["position_deviation_p95_m"]
                > args.max_static_position_p95_m
            ):
                failures.append(
                    "static position P95 deviation {:.4f}m > {:.4f}m".format(
                        pose_quality["position_deviation_p95_m"],
                        args.max_static_position_p95_m,
                    )
                )
            if (
                args.max_static_yaw_p95_rad > 0.0
                and pose_quality["yaw_deviation_p95_rad"]
                > args.max_static_yaw_p95_rad
            ):
                failures.append(
                    "static yaw P95 deviation {:.4f}rad > {:.4f}rad".format(
                        pose_quality["yaw_deviation_p95_rad"],
                        args.max_static_yaw_p95_rad,
                    )
                )

    bad_statuses = [value for value in statuses if not value.startswith("OK")]
    if not statuses:
        failures.append("/mocap/status has no messages")
    elif bad_statuses:
        failures.append("non-OK mocap statuses observed: {}".format(bad_statuses[:5]))
    tracker_mismatches = [
        value
        for value in statuses
        if value.startswith("OK")
        and "tracker={}".format(args.mocap_tracker) not in value
    ]
    if tracker_mismatches:
        failures.append("mocap status tracker mismatch")

    if not trial_mode:
        report = {
            "schema": "spmpc_mocap_static_smoke_postflight_v1",
            "mode": args.mode,
            "bag": str(bag_path),
            "mocap_tracker": args.mocap_tracker,
            "raw_mocap_topic": raw_mocap_topic,
            "imu_topic": args.imu_topic,
            "streams": stream_reports,
            "pose_quality": pose_quality,
            "raw_mocap_arrival_minus_header_sec": numeric_summary(
                raw_mocap_arrival_minus_header
            ),
            "mocap_status_messages": len(statuses),
            "pass": not failures,
            "failures": failures,
        }
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["pass"] else 2

    cycle_ids = [int(message.cycle_id) for message in audits]
    if len(cycle_ids) < args.min_audit_cycles:
        failures.append("audit cycles {} < {}".format(len(cycle_ids), args.min_audit_cycles))
    if cycle_ids != sorted(cycle_ids) or len(cycle_ids) != len(set(cycle_ids)):
        failures.append("audit cycle IDs are not strictly increasing and unique")

    solve_audits = [message for message in audits if message.solve_attempted]
    if len(solve_audits) < args.min_solve_cycles:
        failures.append("solve cycles {} < {}".format(len(solve_audits), args.min_solve_cycles))
    nominal_audits = []
    shift_audits = []
    for message in solve_audits:
        prefix = "cycle {}".format(message.cycle_id)
        if message.variant != args.variant:
            failures.append("{} variant={} != {}".format(prefix, message.variant, args.variant))
        if not message.solve_success:
            failures.append("{} solver failed ({})".format(prefix, message.solver_status))
        if not message.command_accepted or message.safety_gate_intervened:
            failures.append("{} command was rejected/intervened by safety gate".format(prefix))
        if not message.publish_cmd_vel or not message.command_was_published:
            failures.append("{} command was not published".format(prefix))
        if message.command_contract_violation:
            failures.append("{} command contract violation".format(prefix))
        if message.linear_limited or message.angular_rate_limited or message.angular_accel_limited:
            failures.append("{} post-solver limiter intervened".format(prefix))
        if message.terminal_controller_intervened and not message.terminal_phase:
            failures.append("{} terminal controller changed a non-terminal command".format(prefix))
        if not message.state_alignment_required or not message.state_time_aligned:
            failures.append("{} common-epoch state alignment failed".format(prefix))
        chain_stamps = [
            stamp_sec(message.solve_start_stamp),
            stamp_sec(message.solve_end_stamp),
            stamp_sec(message.horizon_available_stamp),
            stamp_sec(message.command_publish_stamp),
        ]
        if any(value <= 0.0 for value in chain_stamps) or chain_stamps != sorted(
            chain_stamps
        ):
            failures.append("{} solve/publish timestamps are non-monotonic".format(prefix))
        if message.previous_shifted_plan_available:
            shift_audits.append(message)
            if int(message.previous_plan_cycle_id) + 1 != int(message.cycle_id):
                failures.append("{} shifted plan is not from adjacent cycle".format(prefix))
        if not message.terminal_phase and not message.safety_gate_intervened:
            nominal_audits.append(message)

    if len(shift_audits) < args.min_shift_pairs:
        failures.append("valid shifted-plan pairs {} < {}".format(len(shift_audits), args.min_shift_pairs))
    solve_ids = {int(message.cycle_id) for message in solve_audits if message.solve_success}
    missing_horizons = sorted(solve_ids - valid_horizon_ids)
    missing_snapshots = sorted(solve_ids - valid_snapshot_ids)
    if missing_horizons:
        failures.append("solve cycles missing valid horizons: {}".format(missing_horizons[:10]))
    if missing_snapshots:
        failures.append("solve cycles missing valid snapshots: {}".format(missing_snapshots[:10]))

    positive_omega = sum(message.published_cmd_omega > args.omega_sign_threshold for message in nominal_audits)
    negative_omega = sum(message.published_cmd_omega < -args.omega_sign_threshold for message in nominal_audits)
    moving_v = sum(abs(message.published_cmd_v) > args.linear_motion_threshold for message in nominal_audits)
    if positive_omega < args.min_omega_sign_samples or negative_omega < args.min_omega_sign_samples:
        failures.append(
            "insufficient S excitation: positive/negative omega samples={}/{}".format(
                positive_omega, negative_omega
            )
        )
    if moving_v < args.min_linear_motion_samples:
        failures.append("insufficient linear-motion samples: {}".format(moving_v))

    path_match = False
    path_max_error = None
    path_topic = ""
    if path_file is not None and path_file.is_file() and path_messages:
        expected_points = path_points_from_json(path_file)
        for topic, message in path_messages:
            matched, error = compare_paths(expected_points, path_points_from_message(message))
            if matched:
                path_match = True
                path_max_error = error
                path_topic = topic
                break
            if path_max_error is None or error < path_max_error:
                path_max_error = error
                path_topic = topic
        if not path_match:
            failures.append("recorded reference path does not match frozen path")
    elif not path_messages:
        failures.append("no non-empty reference path message was recorded")

    software_metrics = {
        "solver_to_published_delta_v": numeric_summary(
            message.published_cmd_v - message.solver_cmd_v for message in nominal_audits
        ),
        "solver_to_published_delta_omega": numeric_summary(
            message.published_cmd_omega - message.solver_cmd_omega for message in nominal_audits
        ),
        "replanned_minus_shifted_a": numeric_summary(
            message.replanned_minus_shifted_a for message in shift_audits
        ),
        "replanned_minus_shifted_alpha": numeric_summary(
            message.replanned_minus_shifted_alpha for message in shift_audits
        ),
    }
    report = {
        "schema": "spmpc_mocap_execution_postflight_v1",
        "mode": args.mode,
        "bag": str(bag_path),
        "variant": args.variant,
        "mocap_tracker": args.mocap_tracker,
        "raw_mocap_topic": raw_mocap_topic,
        "imu_topic": args.imu_topic,
        "path": {
            "file": str(path_file) if path_file is not None else "",
            "expected_sha256": args.path_sha256.lower(),
            "actual_sha256": path_digest,
            "recorded_topic": path_topic,
            "matches": path_match,
            "max_position_error_m": path_max_error,
        },
        "counts": {
            "audit_cycles": len(audits),
            "solve_cycles": len(solve_audits),
            "nominal_tracking_cycles": len(nominal_audits),
            "shift_pairs": len(shift_audits),
            "valid_horizons": len(valid_horizon_ids),
            "valid_snapshots": len(valid_snapshot_ids),
            "positive_omega_samples": positive_omega,
            "negative_omega_samples": negative_omega,
            "linear_motion_samples": moving_v,
            "mocap_status_messages": len(statuses),
        },
        "streams": stream_reports,
        "pose_quality": pose_quality,
        "raw_mocap_arrival_minus_header_sec": numeric_summary(
            raw_mocap_arrival_minus_header
        ),
        "software_metrics": software_metrics,
        "pass": not failures,
        "failures": failures,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["pass"] else 2


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--mode", choices=("trial", "static"), default="trial")
    parser.add_argument("--variant", default="")
    parser.add_argument("--mocap-tracker", default="Tracker0")
    parser.add_argument("--imu-topic", default=IMU_TOPIC)
    parser.add_argument("--path-file", default="")
    parser.add_argument("--path-sha256", default="")
    parser.add_argument("--report")
    parser.add_argument("--min-duration-sec", type=float, default=10.0)
    parser.add_argument("--min-audit-cycles", type=int, default=30)
    parser.add_argument("--min-solve-cycles", type=int, default=10)
    parser.add_argument("--min-shift-pairs", type=int, default=5)
    parser.add_argument("--min-control-rate-hz", type=float, default=20.0)
    parser.add_argument("--min-odom-rate-hz", type=float, default=20.0)
    parser.add_argument("--min-imu-rate-hz", type=float, default=40.0)
    parser.add_argument("--min-mocap-rate-hz", type=float, default=60.0)
    parser.add_argument("--max-control-gap-sec", type=float, default=0.20)
    parser.add_argument("--max-sensor-gap-sec", type=float, default=0.20)
    parser.add_argument("--max-mocap-gap-sec", type=float, default=0.10)
    parser.add_argument("--omega-sign-threshold", type=float, default=0.02)
    parser.add_argument("--min-omega-sign-samples", type=int, default=3)
    parser.add_argument("--linear-motion-threshold", type=float, default=0.03)
    parser.add_argument("--min-linear-motion-samples", type=int, default=10)
    parser.add_argument("--max-mocap-position-step-m", type=float, default=0.05)
    parser.add_argument("--max-mocap-yaw-step-rad", type=float, default=0.20)
    parser.add_argument("--max-quaternion-norm-error", type=float, default=0.05)
    parser.add_argument("--max-static-position-p95-m", type=float, default=0.01)
    parser.add_argument("--max-static-yaw-p95-rad", type=float, default=0.03)
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(validate(parse_args()))
