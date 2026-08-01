#!/usr/bin/env python3
"""Fail-closed postflight for one processed-IMU G2C W2/W5 development trial."""

import argparse
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path

import rosbag


IMAGE_TYPES = {
    "sensor_msgs/Image",
    "sensor_msgs/CompressedImage",
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--expected-weight", required=True, type=float)
    parser.add_argument("--expected-v-ref", default=0.20, type=float)
    parser.add_argument("--min-duration-sec", default=60.0, type=float)
    parser.add_argument("--min-motion-sec", default=20.0, type=float)
    parser.add_argument("--min-source-fraction", default=0.98, type=float)
    parser.add_argument("--min-ready-fraction", default=0.98, type=float)
    parser.add_argument("--hash-bag", action="store_true")
    return parser.parse_args()


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(8 * 1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def parse_multiarray(msg):
    dims = getattr(getattr(msg, "layout", None), "dim", [])
    label = dims[0].label if dims else ""
    fields = [part.strip() for part in label.split(",") if part.strip()]
    values = list(getattr(msg, "data", []))
    return {
        fields[index] if index < len(fields) else f"field_{index}": float(value)
        for index, value in enumerate(values)
    }


def close(actual, expected, tolerance=1.0e-5):
    return math.isfinite(actual) and abs(float(actual) - float(expected)) <= tolerance


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    report_path = bag_path.with_name(bag_path.stem + "_g2c_postflight.json")
    failures = []
    warnings = []

    if not bag_path.is_file():
        print(f"[G2C postflight] missing bag: {bag_path}", file=sys.stderr)
        return 2
    if bag_path.with_suffix(bag_path.suffix + ".active").exists():
        print(f"[G2C postflight] active bag still exists: {bag_path}.active", file=sys.stderr)
        return 2

    command_times = []
    statuses = Counter()
    selection = []
    imu = []
    effective_configs = []
    topic_counts = {}
    image_topics = []

    try:
        with rosbag.Bag(str(bag_path), "r") as bag:
            start = float(bag.get_start_time())
            end = float(bag.get_end_time())
            info = bag.get_type_and_topic_info().topics
            topic_counts = {
                topic: int(item.message_count) for topic, item in info.items()
            }
            image_topics = sorted(
                topic
                for topic, item in info.items()
                if item.msg_type in IMAGE_TYPES and int(item.message_count) > 0
            )
            read_topics = [
                "/cmd_vel",
                "/spmpc/status",
                "/spmpc/debug/slosh_observer_selection",
                "/spmpc/debug/slosh_observer_imu",
                "/spmpc/debug/effective_config",
            ]
            for topic, msg, stamp in bag.read_messages(topics=read_topics):
                bag_time = stamp.to_sec()
                if topic == "/cmd_vel":
                    if abs(float(msg.linear.x)) > 0.03 or abs(float(msg.angular.z)) > 0.03:
                        command_times.append(bag_time)
                elif topic == "/spmpc/status":
                    statuses[str(msg.data)] += 1
                elif topic == "/spmpc/debug/slosh_observer_selection":
                    selection.append((bag_time, msg))
                elif topic == "/spmpc/debug/slosh_observer_imu":
                    imu.append((bag_time, msg))
                elif topic == "/spmpc/debug/effective_config":
                    effective_configs.append((bag_time, parse_multiarray(msg)))
    except Exception as exc:
        print(f"[G2C postflight] rosbag read failed: {exc}", file=sys.stderr)
        return 2

    duration = end - start
    if duration < args.min_duration_sec:
        failures.append(
            f"bag duration {duration:.3f}s < required {args.min_duration_sec:.3f}s"
        )
    if not command_times:
        failures.append("no nonzero command-motion window")
        motion_start = motion_end = start
    else:
        motion_start = min(command_times)
        motion_end = max(command_times)
        if motion_end - motion_start < args.min_motion_sec:
            failures.append(
                f"nonzero-command motion {motion_end - motion_start:.3f}s < "
                f"required {args.min_motion_sec:.3f}s"
            )

    if image_topics:
        failures.append(f"forbidden image streams recorded: {image_topics}")
    if not any(name == "GOAL_REACHED" for name in statuses):
        failures.append("planner never reported GOAL_REACHED")

    selection_motion = [msg for stamp, msg in selection if motion_start <= stamp <= motion_end]
    selection_good = [
        msg
        for msg in selection_motion
        if bool(msg.configured)
        and bool(msg.valid)
        and bool(msg.solver_consumes_selected_state)
        and str(msg.nominal_source_name) == "processed_imu"
        and str(msg.effective_source_name) == "processed_imu"
        and str(msg.status) == "NOMINAL_PROCESSED_IMU"
        and bool(msg.imu_pipeline_ready)
        and not bool(msg.fallback_active)
    ]
    selection_fraction = (
        len(selection_good) / len(selection_motion) if selection_motion else 0.0
    )
    fallback_samples = sum(bool(msg.fallback_active) for msg in selection_motion)
    if selection_fraction < args.min_source_fraction:
        failures.append(
            f"processed-IMU effective-source motion coverage {selection_fraction:.4f} < "
            f"{args.min_source_fraction:.4f}"
        )
    if fallback_samples:
        failures.append(f"observer fallback active in {fallback_samples} motion samples")

    imu_motion = [msg for stamp, msg in imu if motion_start <= stamp <= motion_end]
    imu_ready = [
        msg
        for msg in imu_motion
        if bool(msg.valid)
        and str(msg.input_status) == "READY"
        and bool(msg.bias_ready)
        and bool(msg.filter_ready)
    ]
    ready_fraction = len(imu_ready) / len(imu_motion) if imu_motion else 0.0
    reset_epochs = sorted({int(msg.reset_epoch) for msg in imu_motion})
    if ready_fraction < args.min_ready_fraction:
        failures.append(
            f"processed-IMU READY motion coverage {ready_fraction:.4f} < "
            f"{args.min_ready_fraction:.4f}"
        )
    if reset_epochs not in ([], [0]):
        failures.append(f"processed-IMU reset epochs during motion: {reset_epochs}")

    configs_motion = [
        config for stamp, config in effective_configs if motion_start <= stamp <= motion_end
    ]
    config = configs_motion[-1] if configs_motion else (
        effective_configs[-1][1] if effective_configs else {}
    )
    if not close(config.get("w_slosh", math.nan), args.expected_weight):
        failures.append(
            f"effective w_slosh={config.get('w_slosh')} != {args.expected_weight}"
        )
    if not close(config.get("v_ref", math.nan), args.expected_v_ref):
        failures.append(
            f"effective v_ref={config.get('v_ref')} != {args.expected_v_ref}"
        )
    if not close(config.get("slosh_enable", math.nan), 1.0):
        failures.append("effective slosh model is not enabled")

    report = {
        "schema_version": 1,
        "protocol": "G2C_processed_imu_W2W5_development_v1",
        "status": "PASS" if not failures else "FAIL",
        "bag": str(bag_path),
        "bag_size_bytes": bag_path.stat().st_size,
        "bag_sha256": file_sha256(bag_path) if args.hash_bag else "",
        "duration_sec": duration,
        "motion_start_sec": motion_start,
        "motion_end_sec": motion_end,
        "motion_duration_sec": max(0.0, motion_end - motion_start),
        "expected_weight": args.expected_weight,
        "expected_v_ref": args.expected_v_ref,
        "effective_config_last": config,
        "planner_statuses": dict(statuses),
        "selection": {
            "motion_samples": len(selection_motion),
            "nominal_processed_imu_samples": len(selection_good),
            "nominal_processed_imu_fraction": selection_fraction,
            "fallback_samples": fallback_samples,
        },
        "processed_imu": {
            "motion_samples": len(imu_motion),
            "ready_motion_samples": len(imu_ready),
            "ready_fraction": ready_fraction,
            "reset_epochs": reset_epochs,
        },
        "image_stream_audit": {
            "count": len(image_topics),
            "recorded_image_topics": image_topics,
        },
        "topic_counts": topic_counts,
        "failures": failures,
        "warnings": warnings,
    }
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"[G2C postflight] {report['status']}: {bag_path}")
    print(
        f"  duration/motion = {duration:.3f}/{report['motion_duration_sec']:.3f}s; "
        f"source={selection_fraction:.4f}; IMU_READY={ready_fraction:.4f}; "
        f"images={len(image_topics)}"
    )
    print(f"  report = {report_path}")
    for failure in failures:
        print(f"  FAIL: {failure}", file=sys.stderr)
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
