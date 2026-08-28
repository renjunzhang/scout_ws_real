#!/usr/bin/env python3
"""Stationary, fail-closed qualification for mocap-field Cartographer localization."""

import argparse
import datetime
import json
import math
import pathlib
import statistics
import sys
import time


ANALYSIS_DIR = pathlib.Path(__file__).resolve().parent
if str(ANALYSIS_DIR) not in sys.path:
    sys.path.insert(0, str(ANALYSIS_DIR))

import validate_mocap_field_map as field_map  # noqa: E402


def percentile(values, q):
    values = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * float(q) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def static_pose_metrics(rows):
    """Compute robust stationary jitter metrics for (x, y, yaw) rows."""

    if not rows:
        return {
            "count": 0,
            "position_deviation_p95_m": None,
            "position_deviation_max_m": None,
            "yaw_deviation_p95_rad": None,
            "yaw_deviation_max_rad": None,
            "start_to_end_position_m": None,
            "start_to_end_yaw_rad": None,
        }
    x_center = statistics.median(row[0] for row in rows)
    y_center = statistics.median(row[1] for row in rows)
    sin_center = sum(math.sin(row[2]) for row in rows)
    cos_center = sum(math.cos(row[2]) for row in rows)
    yaw_center = math.atan2(sin_center, cos_center)
    position_deviations = [math.hypot(row[0] - x_center, row[1] - y_center) for row in rows]
    yaw_deviations = [abs(wrap_angle(row[2] - yaw_center)) for row in rows]
    return {
        "count": len(rows),
        "center": {"x": x_center, "y": y_center, "yaw": yaw_center},
        "position_deviation_p95_m": percentile(position_deviations, 95.0),
        "position_deviation_max_m": max(position_deviations),
        "yaw_deviation_p95_rad": percentile(yaw_deviations, 95.0),
        "yaw_deviation_max_rad": max(yaw_deviations),
        "start_to_end_position_m": math.hypot(
            rows[-1][0] - rows[0][0], rows[-1][1] - rows[0][1]
        ),
        "start_to_end_yaw_rad": abs(wrap_angle(rows[-1][2] - rows[0][2])),
    }


class StreamStats:
    def __init__(self, require_header=True):
        self.require_header = require_header
        self.arrival_times = []
        self.header_times = []
        self.invalid_header_stamps = 0
        self.header_regressions = 0

    def observe(self, arrival_time, header_time=None):
        self.arrival_times.append(float(arrival_time))
        if not self.require_header:
            return
        if header_time is None or not math.isfinite(header_time) or header_time <= 0.0:
            self.invalid_header_stamps += 1
            return
        if self.header_times and header_time < self.header_times[-1]:
            self.header_regressions += 1
        self.header_times.append(float(header_time))

    @staticmethod
    def _timing(times):
        if len(times) < 2:
            return {"duration_sec": 0.0, "rate_hz": 0.0, "max_gap_sec": None}
        gaps = [right - left for left, right in zip(times[:-1], times[1:])]
        duration = times[-1] - times[0]
        return {
            "duration_sec": duration,
            "rate_hz": (len(times) - 1) / duration if duration > 0.0 else 0.0,
            "max_gap_sec": max(gaps),
        }

    def summary(self):
        return {
            "count": len(self.arrival_times),
            "arrival": self._timing(self.arrival_times),
            "header": self._timing(self.header_times),
            "invalid_header_stamps": self.invalid_header_stamps,
            "header_regressions": self.header_regressions,
        }


def _publisher_nodes(master, topic):
    publishers, _subscribers, _services = master.getSystemState()
    for published_topic, nodes in publishers:
        if published_topic == topic:
            return sorted(nodes)
    return []


def _append_stream_failures(failures, name, summary, minimum_rate, maximum_gap):
    if summary["count"] < 2:
        failures.append("{} delivered fewer than two messages".format(name))
        return
    if summary["arrival"]["rate_hz"] < minimum_rate:
        failures.append(
            "{} rate {:.3f} Hz is below {:.3f} Hz".format(
                name, summary["arrival"]["rate_hz"], minimum_rate
            )
        )
    if (
        summary["arrival"]["max_gap_sec"] is None
        or summary["arrival"]["max_gap_sec"] > maximum_gap
    ):
        failures.append(
            "{} arrival max gap {} exceeds {:.3f} s".format(
                name, summary["arrival"]["max_gap_sec"], maximum_gap
            )
        )
    if summary["invalid_header_stamps"]:
        failures.append(
            "{} has {} invalid header stamps".format(
                name, summary["invalid_header_stamps"]
            )
        )
    if summary["header_regressions"]:
        failures.append(
            "{} has {} regressing header stamps".format(
                name, summary["header_regressions"]
            )
        )


def run_ros_qualification(args, expected_map_path):
    # ROS imports deliberately live here so offline tests and map checks do not
    # depend on a sourced ROS environment.
    import rosgraph
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import String

    failures = []
    streams = {
        "scan": StreamStats(True),
        "odom": StreamStats(True),
        "imu": StreamStats(True),
        "mocap": StreamStats(True),
        "mocap_status": StreamStats(False),
        "map": StreamStats(False),
    }
    map_rows = []
    odom_rows = []
    odom_linear_speed = []
    odom_angular_speed = []
    mocap_rows = []
    mocap_status_rows = []

    rospy.init_node("validate_mocap_field_localization", anonymous=True)
    master = rosgraph.Master(rospy.get_name())
    cmd_vel_publishers_before = _publisher_nodes(master, args.cmd_vel_topic)
    if cmd_vel_publishers_before:
        failures.append(
            "{} has publishers before qualification: {}".format(
                args.cmd_vel_topic, ", ".join(cmd_vel_publishers_before)
            )
        )

    runtime_params = {}
    for name in (
        "/cartographer_node/frozen_map_file",
        "/cartographer_node/frozen_map_expected_sha256",
    ):
        if rospy.has_param(name):
            runtime_params[name] = rospy.get_param(name)
        else:
            runtime_params[name] = None
            failures.append("missing runtime map evidence parameter: {}".format(name))

    runtime_map_file = runtime_params["/cartographer_node/frozen_map_file"]
    if runtime_map_file is not None:
        runtime_resolved = pathlib.Path(str(runtime_map_file)).expanduser().resolve(strict=False)
        if runtime_resolved != expected_map_path:
            failures.append(
                "runtime frozen_map_file resolves to {}, expected {}".format(
                    runtime_resolved, expected_map_path
                )
            )
    runtime_expected_hash = runtime_params["/cartographer_node/frozen_map_expected_sha256"]
    if runtime_expected_hash is not None and str(runtime_expected_hash).lower() != args.expected_pbstream_sha256.lower():
        failures.append(
            "runtime frozen_map_expected_sha256 does not match the qualification hash"
        )

    def header_time(message):
        return message.header.stamp.to_sec()

    def on_scan(message):
        streams["scan"].observe(time.monotonic(), header_time(message))

    def on_imu(message):
        streams["imu"].observe(time.monotonic(), header_time(message))

    def on_map(message):
        streams["map"].observe(time.monotonic())
        map_rows.append(
            {
                "frame_id": message.header.frame_id,
                "resolution": message.info.resolution,
                "width": message.info.width,
                "height": message.info.height,
                "data_count": len(message.data),
            }
        )

    def on_odom(message):
        streams["odom"].observe(time.monotonic(), header_time(message))
        orientation = message.pose.pose.orientation
        odom_rows.append(
            (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                quaternion_yaw(
                    orientation.x, orientation.y, orientation.z, orientation.w
                ),
            )
        )
        odom_linear_speed.append(
            math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        )
        odom_angular_speed.append(abs(message.twist.twist.angular.z))

    def on_mocap(message):
        streams["mocap"].observe(time.monotonic(), header_time(message))
        orientation = message.pose.orientation
        mocap_rows.append(
            (
                message.pose.position.x,
                message.pose.position.y,
                quaternion_yaw(
                    orientation.x, orientation.y, orientation.z, orientation.w
                ),
            )
        )

    def on_mocap_status(message):
        streams["mocap_status"].observe(time.monotonic())
        mocap_status_rows.append(message.data)

    subscribers = [
        rospy.Subscriber(args.map_topic, OccupancyGrid, on_map, queue_size=2),
        rospy.Subscriber(args.scan_topic, LaserScan, on_scan, queue_size=100),
        rospy.Subscriber(args.odom_topic, Odometry, on_odom, queue_size=100),
        rospy.Subscriber(args.imu_topic, Imu, on_imu, queue_size=200),
        rospy.Subscriber(args.mocap_topic, PoseStamped, on_mocap, queue_size=200),
        rospy.Subscriber(args.mocap_status_topic, String, on_mocap_status, queue_size=20),
    ]

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(max(10.0, args.duration_sec + 2.0)))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # Keep subscriber/listener objects referenced until sampling completes.
    tf_rows = []
    tf_age_rows = []
    tf_attempts = 0
    tf_failures = 0
    cmd_vel_publishers_during = set(cmd_vel_publishers_before)
    next_publisher_check = time.monotonic()
    start = time.monotonic()
    interval = 1.0 / args.tf_sample_rate_hz
    while not rospy.is_shutdown() and time.monotonic() - start < args.duration_sec:
        if time.monotonic() >= next_publisher_check:
            cmd_vel_publishers_during.update(
                _publisher_nodes(master, args.cmd_vel_topic)
            )
            next_publisher_check = time.monotonic() + 1.0
        tf_attempts += 1
        try:
            transform = tf_buffer.lookup_transform(
                args.global_frame,
                args.base_frame,
                rospy.Time(0),
                rospy.Duration(min(0.2, interval)),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tf_rows.append(
                (
                    translation.x,
                    translation.y,
                    quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
                )
            )
            stamp = transform.header.stamp.to_sec()
            now = rospy.Time.now().to_sec()
            if stamp > 0.0 and now >= stamp:
                tf_age_rows.append(now - stamp)
            else:
                tf_age_rows.append(float("inf"))
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            tf_failures += 1
        time.sleep(interval)

    cmd_vel_publishers_after = _publisher_nodes(master, args.cmd_vel_topic)
    cmd_vel_publishers_during.update(cmd_vel_publishers_after)
    if cmd_vel_publishers_after:
        failures.append(
            "{} gained publishers during qualification: {}".format(
                args.cmd_vel_topic, ", ".join(cmd_vel_publishers_after)
            )
        )
    transient_publishers = sorted(
        cmd_vel_publishers_during
        - set(cmd_vel_publishers_before)
        - set(cmd_vel_publishers_after)
    )
    if transient_publishers:
        failures.append(
            "{} had transient publishers during qualification: {}".format(
                args.cmd_vel_topic, ", ".join(transient_publishers)
            )
        )
    for subscriber in subscribers:
        subscriber.unregister()

    stream_summaries = {name: stream.summary() for name, stream in streams.items()}
    _append_stream_failures(
        failures, args.scan_topic, stream_summaries["scan"], args.min_scan_rate_hz, args.max_scan_gap_sec
    )
    _append_stream_failures(
        failures, args.odom_topic, stream_summaries["odom"], args.min_odom_rate_hz, args.max_odom_gap_sec
    )
    _append_stream_failures(
        failures, args.imu_topic, stream_summaries["imu"], args.min_imu_rate_hz, args.max_imu_gap_sec
    )
    _append_stream_failures(
        failures,
        args.mocap_topic,
        stream_summaries["mocap"],
        args.min_mocap_rate_hz,
        args.max_mocap_gap_sec,
    )

    if not map_rows:
        failures.append("{} did not deliver an occupancy grid".format(args.map_topic))
        map_summary = {}
    else:
        map_summary = map_rows[-1]
        if map_summary["frame_id"] != args.global_frame:
            failures.append(
                "map frame is {!r}, expected {!r}".format(
                    map_summary["frame_id"], args.global_frame
                )
            )
        if not math.isclose(
            map_summary["resolution"], args.expected_resolution, rel_tol=0.0, abs_tol=1e-9
        ):
            failures.append(
                "runtime map resolution {} does not match {}".format(
                    map_summary["resolution"], args.expected_resolution
                )
            )
        if map_summary["width"] <= 0 or map_summary["height"] <= 0:
            failures.append("runtime occupancy grid has zero dimensions")
        if map_summary["data_count"] != map_summary["width"] * map_summary["height"]:
            failures.append("runtime occupancy grid data length does not match its dimensions")

    if not mocap_status_rows:
        failures.append("{} delivered no status".format(args.mocap_status_topic))
    else:
        invalid_statuses = [
            value
            for value in mocap_status_rows
            if not value.startswith("OK ") or "tracker={}".format(args.mocap_tracker) not in value
        ]
        if invalid_statuses:
            failures.append(
                "mocap status was not continuously OK for {} ({} invalid samples)".format(
                    args.mocap_tracker, len(invalid_statuses)
                )
            )

    tf_metrics = static_pose_metrics(tf_rows)
    tf_success_fraction = len(tf_rows) / float(tf_attempts) if tf_attempts else 0.0
    if tf_success_fraction < args.min_tf_success_fraction:
        failures.append(
            "TF success fraction {:.3f} is below {:.3f}".format(
                tf_success_fraction, args.min_tf_success_fraction
            )
        )
    finite_tf_ages = [value for value in tf_age_rows if math.isfinite(value)]
    tf_age_max = max(finite_tf_ages) if finite_tf_ages else None
    if len(finite_tf_ages) != len(tf_age_rows):
        failures.append("TF samples contained invalid or zero timestamps")
    if tf_age_max is None or tf_age_max > args.max_tf_age_sec:
        failures.append(
            "TF max age {} exceeds {:.3f} s".format(tf_age_max, args.max_tf_age_sec)
        )
    if tf_metrics["position_deviation_p95_m"] is None or tf_metrics[
        "position_deviation_p95_m"
    ] > args.max_tf_position_p95_m:
        failures.append("stationary map->base_link position jitter exceeds its P95 gate")
    if tf_metrics["yaw_deviation_p95_rad"] is None or tf_metrics[
        "yaw_deviation_p95_rad"
    ] > args.max_tf_yaw_p95_rad:
        failures.append("stationary map->base_link yaw jitter exceeds its P95 gate")

    odom_metrics = static_pose_metrics(odom_rows)
    mocap_metrics = static_pose_metrics(mocap_rows)
    if odom_metrics["position_deviation_max_m"] is None or odom_metrics[
        "position_deviation_max_m"
    ] > args.max_odom_position_max_m:
        failures.append("stationary odom position motion exceeds its gate")
    if odom_metrics["yaw_deviation_max_rad"] is None or odom_metrics[
        "yaw_deviation_max_rad"
    ] > args.max_odom_yaw_max_rad:
        failures.append("stationary odom yaw motion exceeds its gate")
    odom_linear_p95 = percentile(odom_linear_speed, 95.0)
    odom_angular_p95 = percentile(odom_angular_speed, 95.0)
    if odom_linear_p95 is None or odom_linear_p95 > args.max_odom_linear_speed_p95_mps:
        failures.append("stationary odom linear speed exceeds its P95 gate")
    if odom_angular_p95 is None or odom_angular_p95 > args.max_odom_angular_speed_p95_radps:
        failures.append("stationary odom angular speed exceeds its P95 gate")
    if mocap_metrics["position_deviation_p95_m"] is None or mocap_metrics[
        "position_deviation_p95_m"
    ] > args.max_mocap_position_p95_m:
        failures.append("stationary mocap position jitter exceeds its P95 gate")
    if mocap_metrics["yaw_deviation_p95_rad"] is None or mocap_metrics[
        "yaw_deviation_p95_rad"
    ] > args.max_mocap_yaw_p95_rad:
        failures.append("stationary mocap yaw jitter exceeds its P95 gate")

    return {
        "pass": not failures,
        "failures": failures,
        "runtime_map_params": runtime_params,
        "cmd_vel_publishers_before": cmd_vel_publishers_before,
        "cmd_vel_publishers_observed": sorted(cmd_vel_publishers_during),
        "cmd_vel_publishers_after": cmd_vel_publishers_after,
        "streams": stream_summaries,
        "map": map_summary,
        "tf": {
            "parent_frame": args.global_frame,
            "child_frame": args.base_frame,
            "attempts": tf_attempts,
            "successes": len(tf_rows),
            "lookup_failures": tf_failures,
            "success_fraction": tf_success_fraction,
            "age_p95_sec": percentile(finite_tf_ages, 95.0),
            "age_max_sec": tf_age_max,
            "stationary_pose": tf_metrics,
        },
        "odom_stationary": {
            "pose": odom_metrics,
            "linear_speed_p95_mps": odom_linear_p95,
            "angular_speed_p95_radps": odom_angular_p95,
        },
        "mocap_stationary": mocap_metrics,
        "mocap_status_samples": mocap_status_rows,
    }


def _write_report(path, payload, allow_overwrite):
    output = pathlib.Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    mode = "w" if allow_overwrite else "x"
    with output.open(mode, encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_stem")
    parser.add_argument("--expected-pbstream-sha256", required=True)
    parser.add_argument("--expected-yaml-sha256", default="")
    parser.add_argument("--expected-pgm-sha256", default="")
    parser.add_argument("--expected-resolution", type=float, default=0.02)
    parser.add_argument("--report", required=True)
    parser.add_argument("--allow-report-overwrite", action="store_true")
    parser.add_argument("--duration-sec", type=float, default=15.0)
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--scan-topic", default="/scan_front")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--mocap-tracker", default="Tracker0")
    parser.add_argument("--mocap-topic", default="")
    parser.add_argument("--mocap-status-topic", default="/mocap/status")
    parser.add_argument("--min-scan-rate-hz", type=float, default=5.0)
    parser.add_argument("--max-scan-gap-sec", type=float, default=0.50)
    parser.add_argument("--min-odom-rate-hz", type=float, default=10.0)
    parser.add_argument("--max-odom-gap-sec", type=float, default=0.30)
    parser.add_argument("--min-imu-rate-hz", type=float, default=40.0)
    parser.add_argument("--max-imu-gap-sec", type=float, default=0.15)
    parser.add_argument("--min-mocap-rate-hz", type=float, default=60.0)
    parser.add_argument("--max-mocap-gap-sec", type=float, default=0.10)
    parser.add_argument("--tf-sample-rate-hz", type=float, default=20.0)
    parser.add_argument("--min-tf-success-fraction", type=float, default=0.95)
    parser.add_argument("--max-tf-age-sec", type=float, default=0.50)
    parser.add_argument("--max-tf-position-p95-m", type=float, default=0.02)
    parser.add_argument("--max-tf-yaw-p95-rad", type=float, default=0.05)
    parser.add_argument("--max-odom-position-max-m", type=float, default=0.02)
    parser.add_argument("--max-odom-yaw-max-rad", type=float, default=0.05)
    parser.add_argument("--max-odom-linear-speed-p95-mps", type=float, default=0.02)
    parser.add_argument("--max-odom-angular-speed-p95-radps", type=float, default=0.05)
    parser.add_argument("--max-mocap-position-p95-m", type=float, default=0.01)
    parser.add_argument("--max-mocap-yaw-p95-rad", type=float, default=0.03)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.duration_sec < 10.0:
        print("[validate_mocap_field_localization] duration must be >= 10 s", file=sys.stderr)
        return 2
    if args.tf_sample_rate_hz <= 0.0:
        print("[validate_mocap_field_localization] TF sample rate must be positive", file=sys.stderr)
        return 2
    if not field_map.SHA256_RE.fullmatch(args.expected_pbstream_sha256):
        print("[validate_mocap_field_localization] expected pbstream SHA-256 is invalid", file=sys.stderr)
        return 2
    args.mocap_topic = args.mocap_topic or "/vrpn_client_node/{}/pose".format(
        args.mocap_tracker
    )

    offline = field_map.validate_map_assets(
        args.map_stem,
        expected_resolution=args.expected_resolution,
        expected_pbstream_sha256=args.expected_pbstream_sha256,
        expected_yaml_sha256=args.expected_yaml_sha256,
        expected_pgm_sha256=args.expected_pgm_sha256,
    )
    report = {
        "schema_version": 1,
        "protocol_id": "SMPCC_mocap_field_localization_static_v1",
        "created_at": datetime.datetime.now(datetime.timezone.utc).astimezone().isoformat(),
        "pass": False,
        "offline_map": offline,
        "runtime": None,
        "failures": list(offline["failures"]),
    }

    if offline["pass"]:
        expected_map_path = pathlib.Path(offline["assets"]["pbstream"]["path"])
        try:
            runtime = run_ros_qualification(args, expected_map_path)
            report["runtime"] = runtime
            report["failures"].extend(runtime["failures"])
        except Exception as exc:  # Keep a machine-readable failure artifact.
            report["failures"].append("runtime qualification raised: {}".format(exc))
    report["pass"] = not report["failures"]

    try:
        _write_report(args.report, report, args.allow_report_overwrite)
    except FileExistsError:
        print(
            "[validate_mocap_field_localization] report exists; choose a new path: {}".format(
                args.report
            ),
            file=sys.stderr,
        )
        return 2

    if report["pass"]:
        print("[validate_mocap_field_localization] PASS: {}".format(args.report))
        return 0
    print("[validate_mocap_field_localization] FAIL: {}".format(args.report), file=sys.stderr)
    for failure in report["failures"]:
        print("  - {}".format(failure), file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
