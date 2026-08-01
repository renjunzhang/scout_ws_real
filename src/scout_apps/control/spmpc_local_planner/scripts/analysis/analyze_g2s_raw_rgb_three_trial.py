#!/usr/bin/env python3
"""Reproduce the three-trial raw-RGB G2S development source decision."""

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

import rosbag
import yaml

from analyze_g2s_source_selection import (
    decide_source,
    file_sha256,
    load_bag_series,
    source_metrics,
)


IMAGE_MESSAGE_TYPES = {
    "sensor_msgs/Image",
    "sensor_msgs/CompressedImage",
}
EXPECTED_ROWS = ("u01", "u02", "u03")
REPORT_NAME = "G2S_RAW_RGB_THREE_TRIAL_DEVELOPMENT_REPORT.json"
METRICS_NAME = "G2S_RAW_RGB_THREE_TRIAL_METRICS.csv"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trial",
        action="append",
        nargs=3,
        metavar=("ROW", "BAG", "RGB_CSV"),
        default=[],
        help="Repeat for u01, u02 and u03.",
    )
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--prereg", default="")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--hash-bags", action="store_true")
    parser.add_argument("--delta-src", type=float, default=0.10)
    parser.add_argument("--min-directional-trials", type=int, default=3)
    parser.add_argument("--min-coverage", type=float, default=0.90)
    parser.add_argument("--coverage-noninferiority", type=float, default=0.02)
    parser.add_argument("--max-interpolation-gap-sec", type=float, default=0.10)
    parser.add_argument("--expected-raw-rgb-fps", type=float, default=30.0)
    parser.add_argument("--min-raw-rgb-rate-fraction", type=float, default=0.90)
    return parser.parse_args()


def load_env(path):
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value
    return values


def check_prereg(path):
    values = load_env(path)
    expected = {
        "protocol": "G2S_H0s_source_selection_development_v2_online_scalar",
        "planned_paired_units": "4",
        "paired_sources": "odom,processed_imu",
        "raw_or_debug_image_topics_in_bag": (
            "raw_rgb_required_other_image_streams_forbidden"
        ),
        "raw_rgb_recording": "true",
        "raw_rgb_topic": "/camera/color/image_raw",
        "observer_height_field": "total_height_m",
        "per_trial_lag_scale_filter_tuning": "forbidden",
    }
    mismatches = {
        key: {"expected": expected_value, "actual": values.get(key)}
        for key, expected_value in expected.items()
        if values.get(key) != expected_value
    }
    if mismatches:
        raise RuntimeError(f"acquisition prereg mismatch: {mismatches}")
    return values


def check_calibration(path, expected_bags):
    with path.open(encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    validation = calibration.get("offline_validation", {})
    expected = {
        "status": "PASS_OFFLINE_THREE_BAGS_AND_ONLINE_STATIC_SMOKE",
        "processed_frames": 1491,
        "all_three_valid_frames": 1491,
        "clipped_left_frames": 0,
        "clipped_center_frames": 0,
        "clipped_right_frames": 0,
    }
    mismatches = {
        key: {"expected": value, "actual": validation.get(key)}
        for key, value in expected.items()
        if validation.get(key) != value
    }
    if mismatches:
        raise RuntimeError(f"offline calibration validation mismatch: {mismatches}")
    if sorted(validation.get("source_bags", [])) != sorted(expected_bags):
        raise RuntimeError("calibration source_bags differ from the three analysis bags")
    if calibration.get("hsv", {}).get("bottom_touch_rows") != 60:
        raise RuntimeError("frozen relabel calibration must use bottom_touch_rows=60")
    if calibration.get("online_static_smoke", {}).get("status") != "PASS":
        raise RuntimeError("frozen relabel calibration lacks a PASS online static smoke")
    return calibration


def audit_raw_rgb(bag_path, raw_topic, expected_fps, minimum_rate_fraction):
    with rosbag.Bag(str(bag_path), "r") as bag:
        start = float(bag.get_start_time())
        end = float(bag.get_end_time())
        topics = bag.get_type_and_topic_info().topics
    duration = end - start
    image_topics = sorted(
        topic
        for topic, info in topics.items()
        if info.msg_type in IMAGE_MESSAGE_TYPES and int(info.message_count) > 0
    )
    if raw_topic not in topics:
        raise RuntimeError(f"missing raw RGB topic {raw_topic}: {bag_path}")
    raw_info = topics[raw_topic]
    if raw_info.msg_type != "sensor_msgs/Image":
        raise RuntimeError(f"unexpected raw RGB type {raw_info.msg_type}: {bag_path}")
    unexpected = [topic for topic in image_topics if topic != raw_topic]
    if unexpected:
        raise RuntimeError(f"unexpected image streams {unexpected}: {bag_path}")
    raw_count = int(raw_info.message_count)
    raw_rate = raw_count / duration if duration > 0.0 else 0.0
    minimum_rate = expected_fps * minimum_rate_fraction
    if raw_rate < minimum_rate:
        raise RuntimeError(
            f"raw RGB rate {raw_rate:.3f} Hz < required {minimum_rate:.3f} Hz: {bag_path}"
        )
    return {
        "bag_duration_sec": duration,
        "raw_rgb_topic": raw_topic,
        "raw_rgb_message_count": raw_count,
        "raw_rgb_rate_hz": raw_rate,
        "recorded_image_topics": image_topics,
    }


def load_offline_rgb(csv_path, motion_start, motion_end):
    required = {
        "frame_index",
        "stamp_sec",
        "any_clipped",
        "h_mm_max_lcr_smooth_corr",
    }
    all_records = []
    with csv_path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f"offline RGB CSV missing columns {sorted(missing)}: {csv_path}")
        for record in reader:
            frame_index = int(record["frame_index"])
            stamp = float(record["stamp_sec"])
            clipped = int(record["any_clipped"])
            value = float(record["h_mm_max_lcr_smooth_corr"])
            if not (math.isfinite(stamp) and math.isfinite(value)):
                raise RuntimeError(f"non-finite offline RGB row in {csv_path}")
            all_records.append((frame_index, stamp, clipped, max(0.0, value)))
    if len(all_records) < 30:
        raise RuntimeError(f"too few offline RGB records: {csv_path}")
    frame_indices = [record[0] for record in all_records]
    if frame_indices[0] != 0 or any(
        right - left != 5 for left, right in zip(frame_indices, frame_indices[1:])
    ):
        raise RuntimeError(f"offline RGB CSV is not the frozen every-fifth-frame series: {csv_path}")
    clipped_count = sum(record[2] != 0 for record in all_records)
    if clipped_count:
        raise RuntimeError(f"offline RGB CSV contains {clipped_count} clipped records: {csv_path}")
    motion_records = [
        (stamp, value)
        for _, stamp, _, value in all_records
        if motion_start <= stamp <= motion_end
    ]
    if len(motion_records) < 30:
        raise RuntimeError(f"too few motion-window RGB records: {csv_path}")
    return motion_records, {
        "processed_frame_count": len(all_records),
        "all_three_valid_frame_count": len(all_records) - clipped_count,
        "clipped_frame_count": clipped_count,
        "motion_window_sample_count": len(motion_records),
        "frame_stride": 5,
        "value_field": "h_mm_max_lcr_smooth_corr",
    }


def resolve_trials(args):
    if len(args.trial) != 3:
        raise RuntimeError(f"expected exactly three --trial entries, got {len(args.trial)}")
    trials = []
    for row, bag_raw, csv_raw in args.trial:
        bag = Path(bag_raw).expanduser().resolve()
        rgb_csv = Path(csv_raw).expanduser().resolve()
        if row not in EXPECTED_ROWS:
            raise RuntimeError(f"unexpected trial row: {row}")
        if not bag.is_file() or not rgb_csv.is_file():
            raise RuntimeError(f"missing bag or RGB CSV for {row}: {bag}, {rgb_csv}")
        if f"_{row}_" not in bag.stem or f"_{row}_" not in rgb_csv.stem:
            raise RuntimeError(f"row identity differs from filenames for {row}")
        trials.append((row, bag, rgb_csv))
    trials.sort(key=lambda item: item[0])
    if tuple(item[0] for item in trials) != EXPECTED_ROWS:
        raise RuntimeError("trial rows must be exactly u01,u02,u03")
    return trials


def write_metrics_csv(path, trials):
    fields = [
        "row",
        "rgb_motion_samples",
        "odom_coverage",
        "processed_imu_coverage",
        "odom_mae_mm",
        "processed_imu_mae_mm",
        "processed_imu_relative_improvement",
        "odom_rmse_mm",
        "processed_imu_rmse_mm",
        "odom_p95_abs_error_mm",
        "processed_imu_p95_abs_error_mm",
        "odom_corr",
        "processed_imu_corr",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for trial in trials:
            writer.writerow(
                {
                    "row": trial["row"],
                    "rgb_motion_samples": trial["rgb"]["motion_window_sample_count"],
                    "odom_coverage": trial["odom"]["coverage"],
                    "processed_imu_coverage": trial["processed_imu"]["coverage"],
                    "odom_mae_mm": trial["odom"]["mae_mm"],
                    "processed_imu_mae_mm": trial["processed_imu"]["mae_mm"],
                    "processed_imu_relative_improvement": trial["relative_improvement"],
                    "odom_rmse_mm": trial["odom"]["rmse_mm"],
                    "processed_imu_rmse_mm": trial["processed_imu"]["rmse_mm"],
                    "odom_p95_abs_error_mm": trial["odom"]["p95_abs_error_mm"],
                    "processed_imu_p95_abs_error_mm": trial["processed_imu"]["p95_abs_error_mm"],
                    "odom_corr": trial["odom"]["corr"],
                    "processed_imu_corr": trial["processed_imu"]["corr"],
                }
            )


def main():
    args = parse_args()
    trial_inputs = resolve_trials(args)
    bag_dir = trial_inputs[0][1].parent
    if any(bag.parent != bag_dir for _, bag, _ in trial_inputs):
        raise RuntimeError("all three G2S bags must share one acquisition directory")
    prereg_path = (
        Path(args.prereg).expanduser().resolve()
        if args.prereg
        else bag_dir / "G2S_H0s_prereg.env"
    )
    calibration_path = Path(args.calibration).expanduser().resolve()
    if not prereg_path.is_file() or not calibration_path.is_file():
        raise RuntimeError("missing acquisition prereg or frozen relabel calibration")
    prereg = check_prereg(prereg_path)
    calibration = check_calibration(
        calibration_path, [bag.name for _, bag, _ in trial_inputs]
    )

    raw_topic = prereg["raw_rgb_topic"]
    trials = []
    for row, bag, rgb_csv in trial_inputs:
        raw_audit = audit_raw_rgb(
            bag,
            raw_topic,
            args.expected_raw_rgb_fps,
            args.min_raw_rgb_rate_fraction,
        )
        motion_start, motion_end, odom_series, imu_series = load_bag_series(bag)
        rgb, rgb_audit = load_offline_rgb(rgb_csv, motion_start, motion_end)
        odom_metrics = source_metrics(rgb, odom_series, args.max_interpolation_gap_sec)
        imu_metrics = source_metrics(rgb, imu_series, args.max_interpolation_gap_sec)
        odom_mae = odom_metrics["mae_mm"]
        imu_mae = imu_metrics["mae_mm"]
        relative = (odom_mae - imu_mae) / odom_mae
        trials.append(
            {
                "row": row,
                "bag": str(bag),
                "bag_size_bytes": bag.stat().st_size,
                "bag_sha256": file_sha256(bag) if args.hash_bags else "",
                "rgb_csv": str(rgb_csv),
                "rgb_csv_sha256": file_sha256(rgb_csv),
                "motion_start_sec": motion_start,
                "motion_end_sec": motion_end,
                "raw_rgb": raw_audit,
                "rgb": rgb_audit,
                "odom": odom_metrics,
                "processed_imu": imu_metrics,
                "relative_improvement": relative,
            }
        )

    decision, aggregate = decide_source(
        trials,
        args.delta_src,
        args.min_directional_trials,
        args.min_coverage,
        args.coverage_noninferiority,
    )
    status = "PASS_FOR_G2C_DEVELOPMENT" if decision == "processed_imu" else "NO_GO"
    script_path = Path(__file__).resolve()
    shared_script_path = script_path.with_name("analyze_g2s_source_selection.py")
    report = {
        "schema_version": 1,
        "protocol": "G2S_H0s_raw_RGB_relabel_three_trial_development_v1",
        "status": status,
        "decision": decision,
        "decision_scope": "G2C_DEVELOPMENT_ONLY",
        "decision_is_formal_release": False,
        "formal_gate_status": "NOT_RUN_THREE_OF_FOUR_AND_NO_FORMAL_POSTFLIGHT",
        "operator_stop_after_completed_units": 3,
        "planned_formal_units": 4,
        "post_acquisition_rgb_relabel": True,
        "bag_hashes_complete": bool(args.hash_bags),
        "evidence": {
            "acquisition_prereg": str(prereg_path),
            "acquisition_prereg_sha256": file_sha256(prereg_path),
            "acquisition_path_sha256": prereg.get("path_sha256", ""),
            "acquisition_rgb_calibration_sha256": prereg.get(
                "rgb_calibration_sha256", ""
            ),
            "relabel_calibration": str(calibration_path),
            "relabel_calibration_sha256": file_sha256(calibration_path),
            "relabel_offline_validation_status": calibration["offline_validation"][
                "status"
            ],
            "relabel_online_static_smoke_status": calibration["online_static_smoke"][
                "status"
            ],
            "analysis_script": str(script_path),
            "analysis_script_sha256": file_sha256(script_path),
            "shared_analysis_script": str(shared_script_path),
            "shared_analysis_script_sha256": file_sha256(shared_script_path),
        },
        "rules": {
            "primary_metric": "trial_level_motion_window_mae_mm",
            "visual_field": "max(0,h_mm_max_lcr_smooth_corr)",
            "observer_height_field": "total_height_m_times_1000",
            "motion_window": "first_to_last_cmd_vel_above_0.03",
            "additional_lag_sec": {"odom": 0.0, "processed_imu": 0.0},
            "per_trial_lag_scale_filter_tuning": "forbidden",
            "max_interpolation_gap_sec": args.max_interpolation_gap_sec,
            "delta_src": args.delta_src,
            "minimum_directional_trials": args.min_directional_trials,
            "minimum_coverage": args.min_coverage,
            "coverage_noninferiority": args.coverage_noninferiority,
            "single_trial_dominance_rule": (
                "sum_gain_minus_largest_gain_must_be_positive"
            ),
        },
        "aggregate": aggregate,
        "trials": trials,
        "limitations": [
            "Only three of the four preregistered paired units were acquired.",
            "The RGB calibration and HSV mask were relabeled after acquisition from the recorded raw RGB stream.",
            "The report authorizes processed-IMU only for G2C development, not G3 or the formal 40/64/88 matrix.",
        ],
    }

    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    report_path = out_dir / REPORT_NAME
    metrics_path = out_dir / METRICS_NAME
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    write_metrics_csv(metrics_path, trials)

    print("========== G2S raw-RGB three-trial decision ==========")
    for trial in trials:
        print(
            f"  {trial['row']}: RGB={trial['rgb']['motion_window_sample_count']}, "
            f"odom={trial['odom']['mae_mm']:.4f} mm, "
            f"IMU={trial['processed_imu']['mae_mm']:.4f} mm, "
            f"improvement={100.0 * trial['relative_improvement']:.2f}%"
        )
    print(
        f"  aggregate: odom={aggregate['odom_mean_mae_mm']:.4f} mm, "
        f"IMU={aggregate['processed_imu_mean_mae_mm']:.4f} mm, "
        f"improvement={100.0 * aggregate['processed_imu_relative_improvement']:.2f}%"
    )
    print(f"  decision/status: {decision} / {status}")
    print(f"  report: {report_path}")
    print("=======================================================")
    return 0 if status == "PASS_FOR_G2C_DEVELOPMENT" else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[G2S raw-RGB analysis][ERR] {exc}", file=sys.stderr)
        sys.exit(2)
