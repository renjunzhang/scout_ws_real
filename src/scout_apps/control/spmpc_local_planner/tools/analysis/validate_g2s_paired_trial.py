#!/usr/bin/env python3
"""Fail-closed postflight for one Bsmooth H0s G2S paired trial."""

import argparse
import collections
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

import rosbag


REQUIRED_BASE_TOPICS = (
    "/cmd_vel",
    "/odom",
    "/imu/data",
    "/scout/global_path_fixed",
    "/spmpc/status",
    "/spmpc/debug/pre_solve_snapshot",
    "/spmpc/debug/slosh_observer_odom",
    "/spmpc/debug/slosh_observer_imu",
    "/spmpc/debug/slosh_observer_selection",
    "/camera/color/camera_info",
)
IMAGE_MESSAGE_TYPES = {"sensor_msgs/Image", "sensor_msgs/CompressedImage"}


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Validate completeness, IMU READY coverage, paired observer coverage, "
            "source-selection invariants, stamped online RGB quality, and the frozen "
            "image-recording policy in one G2S bag."
        )
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out-json", default="")
    parser.add_argument("--measurement-topic", default="/liquid/measurement")
    parser.add_argument(
        "--image-policy",
        choices=("forbid", "require_raw_rgb"),
        default="forbid",
        help="Forbid every image stream, or require only the raw RGB audit stream.",
    )
    parser.add_argument("--raw-rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--min-raw-rgb-rate-fraction", type=float, default=0.90)
    parser.add_argument("--expected-width", type=int, default=1920)
    parser.add_argument("--expected-height", type=int, default=1080)
    parser.add_argument("--expected-fps", type=float, default=30.0)
    parser.add_argument("--min-camera-info-rate-fraction", type=float, default=0.90)
    parser.add_argument("--process-every", type=int, default=1)
    parser.add_argument("--zero-frames", type=int, default=30)
    parser.add_argument("--min-online-rate-hz", type=float, default=10.0)
    parser.add_argument("--max-online-gap-sec", type=float, default=0.35)
    parser.add_argument("--min-online-valid-fraction", type=float, default=0.90)
    parser.add_argument("--max-p95-publish-lag-sec", type=float, default=0.50)
    parser.add_argument("--max-future-skew-sec", type=float, default=0.05)
    parser.add_argument("--min-duration-sec", type=float, default=80.0)
    parser.add_argument("--min-motion-sec", type=float, default=20.0)
    parser.add_argument("--min-pre-motion-sec", type=float, default=9.0)
    parser.add_argument("--min-post-motion-sec", type=float, default=5.0)
    parser.add_argument("--min-online-pre-motion-sec", type=float, default=2.0)
    parser.add_argument("--min-online-post-motion-sec", type=float, default=5.0)
    parser.add_argument("--min-valid-fraction", type=float, default=0.95)
    parser.add_argument("--hash-bag", action="store_true")
    return parser.parse_args()


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            block = handle.read(8 * 1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def rate_hz(times):
    if len(times) < 2 or times[-1] <= times[0]:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])


def max_gap_sec(times):
    if len(times) < 2:
        return math.inf
    return max(b - a for a, b in zip(times, times[1:]))


def percentile(values, quantile):
    clean = sorted(float(value) for value in values if finite(value))
    if not clean:
        return math.nan
    index = int(round((len(clean) - 1) * quantile))
    return clean[max(0, min(len(clean) - 1, index))]


def fraction(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else 0.0


def in_window(records, start, end):
    if start is None or end is None:
        return []
    return [record for record in records if start <= record[0] <= end]


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    out_json = (
        Path(args.out_json).expanduser().resolve()
        if args.out_json
        else bag_path.with_name(bag_path.stem + "_g2s_postflight.json")
    )
    failures = []
    warnings = []

    if not bag_path.is_file():
        print(f"[G2S postflight] missing bag: {bag_path}", file=sys.stderr)
        return 2
    if Path(str(bag_path) + ".active").exists():
        print(f"[G2S postflight] active bag still exists: {bag_path}.active", file=sys.stderr)
        return 2

    required_topics = REQUIRED_BASE_TOPICS + (args.measurement_topic,)
    counts = collections.Counter()
    topic_times = collections.defaultdict(list)
    cmd_active_times = []
    odom_observer = []
    imu_observer = []
    selections = []
    statuses = []
    camera_shapes = set()
    measurements = []
    image_topics = []
    image_topic_counts = {}
    raw_rgb_message_count = 0
    raw_rgb_rate = 0.0
    start_time = None
    end_time = None

    try:
        with rosbag.Bag(str(bag_path), "r") as bag:
            start_time = bag.get_start_time()
            end_time = bag.get_end_time()
            topic_info = bag.get_type_and_topic_info().topics
            available_topics = set(topic_info)
            image_topics = sorted(
                topic
                for topic, info in topic_info.items()
                if info.msg_type in IMAGE_MESSAGE_TYPES
            )
            image_topic_counts = {
                topic: int(topic_info[topic].message_count) for topic in image_topics
            }
            if args.image_policy == "forbid":
                if image_topics:
                    failures.append(
                        "image-free protocol violated by: " + ", ".join(image_topics)
                    )
            else:
                raw_info = topic_info.get(args.raw_rgb_topic)
                if raw_info is None:
                    failures.append(f"missing required raw RGB topic: {args.raw_rgb_topic}")
                elif raw_info.msg_type != "sensor_msgs/Image":
                    failures.append(
                        f"raw RGB topic has unexpected type: {raw_info.msg_type}"
                    )
                else:
                    raw_rgb_message_count = int(raw_info.message_count)
                unexpected_image_topics = [
                    topic for topic in image_topics if topic != args.raw_rgb_topic
                ]
                if unexpected_image_topics:
                    failures.append(
                        "unexpected compressed/depth/debug image topics: "
                        + ", ".join(unexpected_image_topics)
                    )
            for topic, count in image_topic_counts.items():
                counts[topic] = count
            for topic in required_topics:
                if topic not in available_topics:
                    failures.append(f"missing required topic: {topic}")

            read_topics = [topic for topic in required_topics if topic in available_topics]
            for topic, msg, stamp in bag.read_messages(topics=read_topics):
                bag_time = stamp.to_sec()
                counts[topic] += 1
                topic_times[topic].append(bag_time)
                if topic == "/cmd_vel":
                    if abs(float(msg.linear.x)) > 0.03 or abs(float(msg.angular.z)) > 0.03:
                        cmd_active_times.append(bag_time)
                elif topic == "/spmpc/debug/slosh_observer_odom":
                    odom_observer.append(
                        (bag_time, bool(msg.valid), str(msg.input_status), int(msg.reset_epoch))
                    )
                elif topic == "/spmpc/debug/slosh_observer_imu":
                    imu_observer.append(
                        (
                            bag_time,
                            bool(msg.valid),
                            str(msg.input_status),
                            bool(msg.bias_ready),
                            bool(msg.filter_ready),
                            int(msg.reset_epoch),
                        )
                    )
                elif topic == "/spmpc/debug/slosh_observer_selection":
                    selections.append(
                        (
                            bag_time,
                            bool(msg.solver_consumes_selected_state),
                            bool(msg.valid),
                            bool(msg.fallback_active),
                            str(msg.nominal_source_name),
                            str(msg.effective_source_name),
                            str(msg.status),
                            str(msg.reason),
                            int(msg.selection_epoch),
                            int(msg.imu_reset_epoch),
                        )
                    )
                elif topic == "/spmpc/status":
                    statuses.append(str(msg.data))
                elif topic == "/camera/color/camera_info":
                    camera_shapes.add((int(msg.width), int(msg.height)))
                elif topic == args.measurement_topic:
                    source_time = msg.header.stamp.to_sec()
                    measurements.append(
                        (
                            source_time,
                            bag_time,
                            bool(msg.valid),
                            bool(msg.zero_locked),
                            int(msg.status_code),
                            str(msg.status),
                            float(msg.height_max_lcr_mm),
                            bool(msg.any_clipped),
                            int(msg.image_width),
                            int(msg.image_height),
                            str(msg.image_encoding),
                            int(msg.process_every),
                            float(msg.processing_latency_ms),
                            int(msg.zero_valid_samples),
                            float(msg.zero_max_lcr_mm),
                            msg.zero_window_start_stamp.to_sec(),
                            msg.zero_window_end_stamp.to_sec(),
                            tuple(float(value) for value in msg.zero_samples_max_lcr_raw_mm),
                        )
                    )
    except Exception as exc:
        print(f"[G2S postflight] rosbag read failed: {exc}", file=sys.stderr)
        return 2

    duration = (end_time - start_time) if start_time is not None else 0.0
    if args.image_policy == "require_raw_rgb":
        raw_rgb_rate = fraction(raw_rgb_message_count, duration)
        minimum_raw_rgb_count = int(
            math.floor(args.expected_fps * args.min_raw_rgb_rate_fraction * duration)
        )
        if raw_rgb_message_count < minimum_raw_rgb_count:
            failures.append(
                f"raw RGB count {raw_rgb_message_count} < required "
                f"{minimum_raw_rgb_count} for {duration:.3f}s"
            )
    if duration < args.min_duration_sec:
        failures.append(
            f"bag duration {duration:.3f}s < required {args.min_duration_sec:.3f}s"
        )

    motion_start = min(cmd_active_times) if cmd_active_times else None
    motion_end = max(cmd_active_times) if cmd_active_times else None
    motion_duration = (
        motion_end - motion_start
        if motion_start is not None and motion_end is not None
        else 0.0
    )
    if motion_duration < args.min_motion_sec:
        failures.append(
            f"nonzero-command motion {motion_duration:.3f}s < required {args.min_motion_sec:.3f}s"
        )
    pre_motion = motion_start - start_time if motion_start is not None else 0.0
    post_motion = end_time - motion_end if motion_end is not None else 0.0
    if pre_motion < args.min_pre_motion_sec:
        failures.append(
            f"recorded pre-motion window {pre_motion:.3f}s < {args.min_pre_motion_sec:.3f}s"
        )
    if post_motion < args.min_post_motion_sec:
        failures.append(
            f"recorded post-motion window {post_motion:.3f}s < {args.min_post_motion_sec:.3f}s"
        )

    expected_shape = (args.expected_width, args.expected_height)
    if camera_shapes and camera_shapes != {expected_shape}:
        failures.append(f"camera_info shape mismatch: {sorted(camera_shapes)}")
    camera_info_times = topic_times["/camera/color/camera_info"]
    camera_info_rate = rate_hz(camera_info_times)
    if camera_info_rate < args.expected_fps * args.min_camera_info_rate_fraction:
        failures.append(
            f"camera_info rate {camera_info_rate:.3f}Hz < "
            f"{args.expected_fps * args.min_camera_info_rate_fraction:.3f}Hz"
        )

    measurements.sort(key=lambda item: item[0])
    source_times = [record[0] for record in measurements if record[0] > 0.0]
    source_stamp_missing = len(measurements) - len(source_times)
    if source_stamp_missing:
        failures.append(f"{source_stamp_missing} online measurements have zero source stamps")
    measurement_shapes = {(record[8], record[9]) for record in measurements}
    if measurement_shapes and measurement_shapes != {expected_shape}:
        failures.append(f"online measurement image shape mismatch: {sorted(measurement_shapes)}")
    process_every_values = sorted({record[11] for record in measurements})
    if process_every_values and process_every_values != [args.process_every]:
        failures.append(
            f"online process_every mismatch: expected {args.process_every}, got {process_every_values}"
        )
    locked_measurements = [record for record in measurements if record[3]]
    zero_counts = sorted({record[13] for record in locked_measurements})
    zero_values = sorted({record[14] for record in locked_measurements})
    zero_windows = sorted({(record[15], record[16]) for record in locked_measurements})
    zero_sample_histories = {record[17] for record in locked_measurements}
    if locked_measurements:
        if zero_counts != [args.zero_frames]:
            failures.append(
                f"online zero sample count mismatch: expected {args.zero_frames}, got {zero_counts}"
            )
        if len(zero_values) != 1 or not finite(zero_values[0]):
            failures.append(f"online zero value is not unique/finite: {zero_values}")
        if len(zero_windows) != 1:
            failures.append(f"online zero timestamp window changed: {zero_windows}")
        else:
            zero_start, zero_end = zero_windows[0]
            if zero_start <= 0.0 or zero_end < zero_start:
                failures.append(
                    f"invalid online zero timestamp window: {zero_start:.9f}..{zero_end:.9f}"
                )
            if motion_start is not None and zero_end >= motion_start:
                failures.append("online zero estimation did not finish before first motion")
        if len(zero_sample_histories) != 1:
            failures.append("online zero sample history changed within the bag")
        else:
            zero_history = next(iter(zero_sample_histories))
            if len(zero_history) != args.zero_frames or not all(finite(value) for value in zero_history):
                failures.append(
                    f"online zero history expected {args.zero_frames} finite samples, got {len(zero_history)}"
                )
            elif len(zero_values) == 1 and abs(statistics.median(zero_history) - zero_values[0]) > 1.0e-4:
                failures.append("online zero value does not equal the recorded zero-history median")
    online_rate = rate_hz(source_times)
    online_gap = max_gap_sec(source_times)
    if online_rate < args.min_online_rate_hz:
        failures.append(
            f"online scalar rate {online_rate:.3f}Hz < {args.min_online_rate_hz:.3f}Hz"
        )
    if online_gap > args.max_online_gap_sec:
        failures.append(
            f"online scalar max gap {online_gap:.3f}s > {args.max_online_gap_sec:.3f}s"
        )

    publish_lags = [record[1] - record[0] for record in measurements if record[0] > 0.0]
    p95_publish_lag = percentile(publish_lags, 0.95)
    min_publish_lag = min(publish_lags) if publish_lags else math.nan
    if finite(p95_publish_lag) and p95_publish_lag > args.max_p95_publish_lag_sec:
        failures.append(
            f"online publish-lag P95 {p95_publish_lag:.3f}s > "
            f"{args.max_p95_publish_lag_sec:.3f}s"
        )
    if finite(min_publish_lag) and min_publish_lag < -args.max_future_skew_sec:
        failures.append(
            f"online source stamp leads bag time by {-min_publish_lag:.3f}s > "
            f"{args.max_future_skew_sec:.3f}s"
        )

    measurement_motion = in_window(measurements, motion_start, motion_end)
    measurement_valid = sum(
        1
        for record in measurement_motion
        if record[2]
        and record[3]
        and record[4] == 0
        and finite(record[6])
        and not record[7]
    )
    online_valid_fraction = fraction(measurement_valid, len(measurement_motion))
    if online_valid_fraction < args.min_online_valid_fraction:
        failures.append(
            f"online RGB valid motion coverage {online_valid_fraction:.4f} < "
            f"{args.min_online_valid_fraction:.4f}"
        )
    if any(not record[3] for record in measurement_motion):
        failures.append("online RGB zero lock was lost or absent during motion")
    if measurements and motion_start is not None and motion_end is not None:
        if measurements[0][0] > motion_start - args.min_online_pre_motion_sec:
            failures.append("online RGB scalar lacks the frozen pre-motion coverage")
        if measurements[-1][0] < motion_end + args.min_online_post_motion_sec:
            failures.append("online RGB scalar lacks the frozen post-motion tail")

    odom_motion = in_window(odom_observer, motion_start, motion_end)
    imu_motion = in_window(imu_observer, motion_start, motion_end)
    selection_motion = in_window(selections, motion_start, motion_end)
    odom_valid = sum(1 for record in odom_motion if record[1])
    imu_ready = sum(
        1
        for record in imu_motion
        if record[1] and record[2] == "READY" and record[3] and record[4]
    )
    odom_valid_fraction = fraction(odom_valid, len(odom_motion))
    imu_ready_fraction = fraction(imu_ready, len(imu_motion))
    if odom_valid_fraction < args.min_valid_fraction:
        failures.append(
            f"odom observer motion validity {odom_valid_fraction:.4f} < {args.min_valid_fraction:.4f}"
        )
    if imu_ready_fraction < args.min_valid_fraction:
        failures.append(
            f"processed-IMU READY motion coverage {imu_ready_fraction:.4f} < {args.min_valid_fraction:.4f}"
        )
    imu_motion_epochs = sorted({record[5] for record in imu_motion})
    if len(imu_motion_epochs) != 1:
        failures.append(f"IMU reset epoch changed during motion: {imu_motion_epochs}")

    if not selection_motion:
        failures.append("no source-selection diagnostics during motion")
    else:
        if any(record[1] for record in selection_motion):
            failures.append("Bsmooth G2S unexpectedly reports liquid state consumed by solver")
        if any(record[3] for record in selection_motion):
            failures.append("unexpected observer fallback during paired Bsmooth motion")
        if any(record[4] != "odom" for record in selection_motion):
            failures.append("paired Bsmooth nominal source metadata is not odom")
        if any(not record[2] or record[5] != "odom" for record in selection_motion):
            failures.append("odom selection was not valid/effective throughout paired motion")
        selection_epochs = sorted({record[8] for record in selection_motion})
        if len(selection_epochs) != 1:
            failures.append(f"source selection epoch changed during motion: {selection_epochs}")

    if "GOAL_REACHED" not in statuses:
        failures.append("planner never published GOAL_REACHED")

    snapshot_motion = in_window(
        [(stamp,) for stamp in topic_times["/spmpc/debug/pre_solve_snapshot"]],
        motion_start,
        motion_end,
    )
    if len(snapshot_motion) < 100:
        failures.append(f"only {len(snapshot_motion)} pre-solve snapshots in the motion window")

    report = {
        "schema_version": 2,
        "protocol": "G2S_H0s_source_selection_development_v2_online_scalar",
        "bag": str(bag_path),
        "bag_size_bytes": bag_path.stat().st_size,
        "bag_sha256": sha256_file(bag_path) if args.hash_bag else "",
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "warnings": warnings,
        "duration_sec": duration,
        "motion_start_sec": motion_start,
        "motion_end_sec": motion_end,
        "motion_duration_sec": motion_duration,
        "pre_motion_sec": pre_motion,
        "post_motion_sec": post_motion,
        "topic_counts": dict(sorted(counts.items())),
        "image_stream_audit": {
            "policy": args.image_policy,
            "raw_rgb_topic": args.raw_rgb_topic,
            "recorded_image_topics": image_topics,
            "image_topic_counts": image_topic_counts,
            "count": len(image_topics),
            "raw_rgb_message_count": raw_rgb_message_count,
            "raw_rgb_rate_hz": raw_rgb_rate,
            "minimum_raw_rgb_rate_fraction": args.min_raw_rgb_rate_fraction,
        },
        "online_rgb": {
            "topic": args.measurement_topic,
            "samples": len(measurements),
            "source_stamp_missing": source_stamp_missing,
            "shapes": sorted(measurement_shapes),
            "camera_info_shapes": sorted(camera_shapes),
            "camera_info_rate_hz": camera_info_rate,
            "camera_info_max_gap_sec": max_gap_sec(camera_info_times),
            "encodings": sorted({record[10] for record in measurements}),
            "process_every_values": process_every_values,
            "zero_sample_counts": zero_counts,
            "zero_values_mm": zero_values,
            "zero_windows_sec": zero_windows,
            "zero_history_variants": len(zero_sample_histories),
            "rate_hz": online_rate,
            "max_gap_sec": online_gap,
            "publish_lag_p95_sec": p95_publish_lag,
            "publish_lag_min_sec": min_publish_lag,
            "motion_samples": len(measurement_motion),
            "valid_motion_samples": measurement_valid,
            "valid_motion_fraction": online_valid_fraction,
        },
        "odom_observer": {
            "motion_samples": len(odom_motion),
            "valid_motion_samples": odom_valid,
            "valid_fraction": odom_valid_fraction,
        },
        "processed_imu_observer": {
            "motion_samples": len(imu_motion),
            "ready_motion_samples": imu_ready,
            "ready_fraction": imu_ready_fraction,
            "motion_reset_epochs": imu_motion_epochs,
        },
        "selection_motion_samples": len(selection_motion),
        "planner_statuses": list(dict.fromkeys(statuses)),
    }
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print(f"[G2S postflight] {report['status']}: {bag_path}")
    print(
        f"  duration/motion/pre/post = {duration:.3f}/{motion_duration:.3f}/"
        f"{pre_motion:.3f}/{post_motion:.3f} s"
    )
    print(
        f"  online RGB scalar = {len(measurements)} samples, {online_rate:.3f} Hz, "
        f"valid={online_valid_fraction:.4f}; image policy={args.image_policy}, "
        f"topics={len(image_topics)}, raw RGB={raw_rgb_message_count} "
        f"({raw_rgb_rate:.3f} Hz)"
    )
    print(
        f"  odom valid={odom_valid_fraction:.4f}; IMU READY={imu_ready_fraction:.4f}"
    )
    print(f"  report = {out_json}")
    for failure in failures:
        print(f"  FAIL: {failure}", file=sys.stderr)
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
