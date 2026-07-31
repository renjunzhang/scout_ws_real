#!/usr/bin/env python3
"""Paired odom-vs-processed-IMU source decision against frozen RGB output."""

import argparse
import bisect
import csv
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Use four same-trial Bsmooth H0s units to compare odom and "
            "processed-IMU liquid observers against the frozen stamped online RGB proxy."
        )
    )
    parser.add_argument("--bag", action="append", default=[], help="Repeat exactly four times.")
    parser.add_argument("--bag-dir", default="", help="Directory containing the four u01..u04 bags.")
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--measurement-topic", default="/liquid/measurement")
    parser.add_argument("--smooth-frames", type=int, default=5)
    parser.add_argument("--delta-src", type=float, default=0.10)
    parser.add_argument("--min-directional-trials", type=int, default=3)
    parser.add_argument("--min-coverage", type=float, default=0.90)
    parser.add_argument("--coverage-noninferiority", type=float, default=0.02)
    parser.add_argument("--max-interpolation-gap-sec", type=float, default=0.10)
    return parser.parse_args()


def parse_float(raw):
    if raw is None:
        return None
    text = str(raw).strip()
    if not text:
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def percentile(values, fraction):
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return math.nan
    index = int(round((len(clean) - 1) * fraction))
    return clean[max(0, min(len(clean) - 1, index))]


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            block = handle.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def load_prereg(rows, calibration, args):
    if args.smooth_frames != 5:
        raise RuntimeError("frozen G2S protocol requires --smooth-frames 5")
    directories = {bag.parent for _, bag, _ in rows}
    if len(directories) != 1:
        raise RuntimeError("all four planned bags must share one preregistered directory")
    prereg_path = next(iter(directories)) / "G2S_H0s_prereg.env"
    if not prereg_path.is_file():
        raise RuntimeError(f"missing G2S prereg: {prereg_path}")
    values = {}
    for line in prereg_path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value
    expected = {
        "protocol": "G2S_H0s_source_selection_development_v2_online_scalar",
        "planned_paired_units": "4",
        "observer_height_field": "total_height_m",
        "rgb_height_field": "height_max_lcr_mm_centered_rolling_median_5",
        "per_trial_lag_scale_filter_tuning": "forbidden",
        "rgb_calibration_sha256": file_sha256(calibration),
        "online_measurement_topic": args.measurement_topic,
        "raw_or_debug_image_topics_in_bag": "forbidden",
    }
    mismatches = {
        key: {"expected": expected_value, "actual": values.get(key)}
        for key, expected_value in expected.items()
        if values.get(key) != expected_value
    }
    if mismatches:
        raise RuntimeError(f"analysis inputs differ from prereg: {mismatches}")
    prereg_delta = parse_float(values.get("processed_imu_min_relative_improvement"))
    if prereg_delta is None or abs(prereg_delta - args.delta_src) > 1.0e-12:
        raise RuntimeError(
            "analysis delta-src differs from processed_imu_min_relative_improvement prereg"
        )
    try:
        prereg_direction = int(values.get("minimum_same_direction_trials", ""))
    except ValueError as exc:
        raise RuntimeError("invalid minimum_same_direction_trials prereg") from exc
    if prereg_direction != args.min_directional_trials:
        raise RuntimeError("analysis min-directional-trials differs from prereg")
    return prereg_path, values


def correlation(left, right):
    pairs = [(a, b) for a, b in zip(left, right) if math.isfinite(a) and math.isfinite(b)]
    if len(pairs) < 3:
        return math.nan
    xs = [a for a, _ in pairs]
    ys = [b for _, b in pairs]
    mx = statistics.mean(xs)
    my = statistics.mean(ys)
    numerator = sum((x - mx) * (y - my) for x, y in pairs)
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    return numerator / (dx * dy) if dx > 0.0 and dy > 0.0 else math.nan


def interpolate_at(query, times, values, max_gap):
    if len(times) < 2 or query < times[0] or query > times[-1]:
        return None
    index = bisect.bisect_left(times, query)
    if index == 0:
        return values[0]
    if index == len(times):
        return values[-1]
    t0, t1 = times[index - 1], times[index]
    if t1 <= t0 or t1 - t0 > max_gap:
        return None
    ratio = (query - t0) / (t1 - t0)
    return values[index - 1] + ratio * (values[index] - values[index - 1])


def discover_bags(args):
    bags = [Path(item).expanduser().resolve() for item in args.bag]
    if args.bag_dir:
        directory = Path(args.bag_dir).expanduser().resolve()
        bags.extend(sorted(directory.glob("DEV_G2S_H0s_C1_Bsmooth_u*_a*.bag")))
    unique = []
    seen = set()
    for bag in bags:
        if bag not in seen:
            unique.append(bag)
            seen.add(bag)
    if len(unique) != 4:
        raise RuntimeError(f"expected exactly four planned paired bags, found {len(unique)}")
    rows = []
    for bag in unique:
        if not bag.is_file():
            raise RuntimeError(f"missing bag: {bag}")
        postflight = bag.with_name(bag.stem + "_g2s_postflight.json")
        if not postflight.is_file():
            raise RuntimeError(f"missing postflight report: {postflight}")
        report = json.loads(postflight.read_text(encoding="utf-8"))
        if report.get("status") != "PASS":
            raise RuntimeError(f"postflight is not PASS: {postflight}")
        if report.get("protocol") != "G2S_H0s_source_selection_development_v2_online_scalar":
            raise RuntimeError(f"postflight protocol mismatch: {postflight}")
        row_token = next((part for part in bag.stem.split("_") if part.startswith("u")), "")
        rows.append((row_token, bag, report))
    rows.sort(key=lambda item: item[0])
    expected = ["u01", "u02", "u03", "u04"]
    actual = [item[0] for item in rows]
    if actual != expected:
        raise RuntimeError(f"planned row identity mismatch: expected {expected}, got {actual}")
    return rows


def centered_rolling_median(records, window):
    """Smooth clean scalar records without inventing values for invalid center frames."""
    if window < 1 or window % 2 == 0:
        raise RuntimeError("smooth-frames must be a positive odd integer")
    half = window // 2
    output = []
    for index, (stamp, value) in enumerate(records):
        if value is None or not math.isfinite(value):
            output.append((stamp, None))
            continue
        segment = [
            candidate
            for _, candidate in records[max(0, index - half):index + half + 1]
            if candidate is not None and math.isfinite(candidate)
        ]
        output.append((stamp, statistics.median(segment) if segment else None))
    return output


def load_online_rgb(bag, topic, motion_start, motion_end, smooth_frames):
    records = []
    with rosbag.Bag(str(bag), "r") as handle:
        topic_info = handle.get_type_and_topic_info().topics
        if topic not in topic_info:
            raise RuntimeError(f"missing online RGB topic {topic}: {bag}")
        if topic_info[topic].msg_type != "realsense_liquid_measurement/OnlineLiquidMeasurement":
            raise RuntimeError(
                f"unexpected online RGB type {topic_info[topic].msg_type}: {bag}"
            )
        for _, msg, _ in handle.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec()
            clean = bool(
                stamp > 0.0
                and msg.valid
                and msg.zero_locked
                and int(msg.status_code) == 0
                and not msg.any_clipped
                and math.isfinite(float(msg.height_max_lcr_mm))
            )
            records.append(
                (stamp, max(0.0, float(msg.height_max_lcr_mm)) if clean else None)
            )
    records.sort(key=lambda item: item[0])
    smoothed = centered_rolling_median(records, smooth_frames)
    return [
        (stamp, value)
        for stamp, value in smoothed
        if value is not None and motion_start <= stamp <= motion_end
    ]


def load_bag_series(bag):
    command_times = []
    odom = []
    imu = []
    with rosbag.Bag(str(bag), "r") as handle:
        for topic, msg, stamp in handle.read_messages(
            topics=[
                "/cmd_vel",
                "/spmpc/debug/slosh_observer_odom",
                "/spmpc/debug/slosh_observer_imu",
            ]
        ):
            bag_time = stamp.to_sec()
            if topic == "/cmd_vel":
                if abs(float(msg.linear.x)) > 0.03 or abs(float(msg.angular.z)) > 0.03:
                    command_times.append(bag_time)
                continue
            state_stamp = msg.state_stamp.to_sec()
            if state_stamp <= 0.0 or not bool(msg.valid):
                continue
            total_mm = 1000.0 * float(msg.total_height_m)
            modal_mm = 1000.0 * float(msg.modal_height_m)
            if not (math.isfinite(total_mm) and math.isfinite(modal_mm)):
                continue
            if topic.endswith("_imu"):
                if not (
                    str(msg.input_status) == "READY"
                    and bool(msg.bias_ready)
                    and bool(msg.filter_ready)
                ):
                    continue
                imu.append((state_stamp, total_mm, modal_mm))
            else:
                odom.append((state_stamp, total_mm, modal_mm))
    if not command_times:
        raise RuntimeError(f"no command-motion window in {bag}")
    odom.sort(key=lambda item: item[0])
    imu.sort(key=lambda item: item[0])
    return min(command_times), max(command_times), odom, imu


def source_metrics(rgb, observer, max_gap):
    times = [item[0] for item in observer]
    total = [item[1] for item in observer]
    modal = [item[2] for item in observer]
    aligned_rgb = []
    aligned_total = []
    aligned_modal = []
    for stamp, rgb_mm in rgb:
        total_mm = interpolate_at(stamp, times, total, max_gap)
        modal_mm = interpolate_at(stamp, times, modal, max_gap)
        if total_mm is None or modal_mm is None:
            continue
        aligned_rgb.append(rgb_mm)
        aligned_total.append(total_mm)
        aligned_modal.append(modal_mm)
    errors = [model - visual for model, visual in zip(aligned_total, aligned_rgb)]
    abs_errors = [abs(value) for value in errors]
    count = len(aligned_rgb)
    return {
        "eligible_aligned_samples": count,
        "coverage": count / len(rgb) if rgb else 0.0,
        "mae_mm": statistics.mean(abs_errors) if abs_errors else math.nan,
        "rmse_mm": math.sqrt(statistics.mean([value * value for value in errors]))
        if errors
        else math.nan,
        "p95_abs_error_mm": percentile(abs_errors, 0.95),
        "signed_bias_mm": statistics.mean(errors) if errors else math.nan,
        "peak_bias_mm": (max(aligned_total) - max(aligned_rgb)) if aligned_rgb else math.nan,
        "corr": correlation(aligned_total, aligned_rgb),
        "model_total_peak_mm": max(aligned_total) if aligned_total else math.nan,
        "model_modal_peak_mm": max(aligned_modal) if aligned_modal else math.nan,
        "rgb_peak_mm": max(aligned_rgb) if aligned_rgb else math.nan,
    }


def write_trial_csv(path, trials):
    fields = [
        "row",
        "bag",
        "rgb_eligible_samples",
        "odom_coverage",
        "imu_coverage",
        "odom_mae_mm",
        "imu_mae_mm",
        "relative_improvement",
        "odom_rmse_mm",
        "imu_rmse_mm",
        "odom_p95_abs_error_mm",
        "imu_p95_abs_error_mm",
        "odom_peak_bias_mm",
        "imu_peak_bias_mm",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for trial in trials:
            writer.writerow(
                {
                    "row": trial["row"],
                    "bag": trial["bag"],
                    "rgb_eligible_samples": trial["rgb_eligible_samples"],
                    "odom_coverage": trial["odom"]["coverage"],
                    "imu_coverage": trial["processed_imu"]["coverage"],
                    "odom_mae_mm": trial["odom"]["mae_mm"],
                    "imu_mae_mm": trial["processed_imu"]["mae_mm"],
                    "relative_improvement": trial["relative_improvement"],
                    "odom_rmse_mm": trial["odom"]["rmse_mm"],
                    "imu_rmse_mm": trial["processed_imu"]["rmse_mm"],
                    "odom_p95_abs_error_mm": trial["odom"]["p95_abs_error_mm"],
                    "imu_p95_abs_error_mm": trial["processed_imu"]["p95_abs_error_mm"],
                    "odom_peak_bias_mm": trial["odom"]["peak_bias_mm"],
                    "imu_peak_bias_mm": trial["processed_imu"]["peak_bias_mm"],
                }
            )


def decide_source(
    trials,
    delta_src,
    min_directional_trials,
    min_coverage,
    coverage_noninferiority,
):
    odom_mean_mae = statistics.mean(trial["odom"]["mae_mm"] for trial in trials)
    imu_mean_mae = statistics.mean(trial["processed_imu"]["mae_mm"] for trial in trials)
    aggregate_relative = (
        (odom_mean_mae - imu_mean_mae) / odom_mean_mae if odom_mean_mae > 1.0e-12 else math.nan
    )
    gains = [
        trial["odom"]["mae_mm"] - trial["processed_imu"]["mae_mm"]
        for trial in trials
    ]
    directional_count = sum(1 for gain in gains if gain > 0.0)
    improvement_without_best = sum(gains) - max(gains)
    not_single_trial_dominated = improvement_without_best > 0.0
    coverage_pass = all(
        trial["processed_imu"]["coverage"] >= min_coverage
        and trial["processed_imu"]["coverage"]
        >= trial["odom"]["coverage"] - coverage_noninferiority
        for trial in trials
    )
    threshold_pass = aggregate_relative >= delta_src
    direction_pass = directional_count >= min_directional_trials
    imu_selected = (
        threshold_pass
        and direction_pass
        and not_single_trial_dominated
        and coverage_pass
    )
    aggregate = {
        "odom_mean_mae_mm": odom_mean_mae,
        "processed_imu_mean_mae_mm": imu_mean_mae,
        "processed_imu_relative_improvement": aggregate_relative,
        "directional_trial_count": directional_count,
        "gain_mm_by_trial": gains,
        "gain_without_best_trial_mm": improvement_without_best,
        "threshold_pass": threshold_pass,
        "direction_pass": direction_pass,
        "not_single_trial_dominated": not_single_trial_dominated,
        "coverage_pass": coverage_pass,
    }
    return ("processed_imu" if imu_selected else "odom"), aggregate


def main():
    args = parse_args()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    calibration = Path(args.calibration).expanduser().resolve()
    if not calibration.is_file():
        raise RuntimeError(f"missing RGB calibration: {calibration}")
    rows = discover_bags(args)
    prereg_path, prereg = load_prereg(rows, calibration, args)

    trials = []
    for row, bag, postflight in rows:
        motion_start, motion_end, odom_series, imu_series = load_bag_series(bag)
        rgb = load_online_rgb(
            bag,
            args.measurement_topic,
            motion_start,
            motion_end,
            args.smooth_frames,
        )
        if len(rgb) < 30:
            raise RuntimeError(f"too few eligible online RGB motion samples ({len(rgb)}): {bag}")
        odom_metrics = source_metrics(rgb, odom_series, args.max_interpolation_gap_sec)
        imu_metrics = source_metrics(rgb, imu_series, args.max_interpolation_gap_sec)
        odom_mae = odom_metrics["mae_mm"]
        imu_mae = imu_metrics["mae_mm"]
        relative = (odom_mae - imu_mae) / odom_mae if odom_mae > 1.0e-12 else math.nan
        trials.append(
            {
                "row": row,
                "bag": str(bag),
                "postflight": str(bag.with_name(bag.stem + "_g2s_postflight.json")),
                "rgb_topic": args.measurement_topic,
                "motion_start_sec": motion_start,
                "motion_end_sec": motion_end,
                "rgb_eligible_samples": len(rgb),
                "odom": odom_metrics,
                "processed_imu": imu_metrics,
                "relative_improvement": relative,
                "postflight_imu_ready_fraction": postflight["processed_imu_observer"][
                    "ready_fraction"
                ],
            }
        )

    decision, aggregate = decide_source(
        trials,
        args.delta_src,
        args.min_directional_trials,
        args.min_coverage,
        args.coverage_noninferiority,
    )

    report = {
        "schema_version": 2,
        "protocol": "G2S_H0s_source_selection_development_v2_online_scalar",
        "decision": decision,
        "decision_is_formal_release": False,
        "prereg": str(prereg_path),
        "preregistered_path_sha256": prereg.get("path_sha256", ""),
        "calibration": str(calibration),
        "rules": {
            "planned_paired_units": 4,
            "primary_metric": "trial_level_motion_window_mae_mm",
            "observer_height_field": "total_height_m",
            "rgb_field": "height_max_lcr_mm_centered_rolling_median_5",
            "rgb_topic": args.measurement_topic,
            "rgb_smooth_frames": args.smooth_frames,
            "raw_or_debug_image_topics_in_bag": "forbidden",
            "per_trial_lag_scale_filter_tuning": "forbidden",
            "additional_lag_sec": {"odom": 0.0, "processed_imu": 0.0},
            "delta_src": args.delta_src,
            "minimum_directional_trials": args.min_directional_trials,
            "minimum_coverage": args.min_coverage,
            "coverage_noninferiority": args.coverage_noninferiority,
            "single_trial_dominance_rule": "sum_gain_minus_largest_gain_must_be_positive",
            "tie_conflict_or_uncertainty": "odom",
        },
        "aggregate": aggregate,
        "trials": trials,
    }
    report_path = out_dir / "G2S_SOURCE_SELECTION_REPORT.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    csv_path = out_dir / "G2S_SOURCE_SELECTION_TRIAL_METRICS.csv"
    write_trial_csv(csv_path, trials)

    print("================ G2S source decision ================")
    for trial in trials:
        print(
            f"  {trial['row']}: MAE odom={trial['odom']['mae_mm']:.3f} mm, "
            f"IMU={trial['processed_imu']['mae_mm']:.3f} mm, "
            f"relative={100.0 * trial['relative_improvement']:.1f}%"
        )
    print(
        f"  aggregate: odom={aggregate['odom_mean_mae_mm']:.3f} mm, "
        f"IMU={aggregate['processed_imu_mean_mae_mm']:.3f} mm, "
        f"improvement={100.0 * aggregate['processed_imu_relative_improvement']:.1f}%"
    )
    print(
        f"  gates: threshold={aggregate['threshold_pass']}, "
        f"direction={aggregate['direction_pass']} "
        f"({aggregate['directional_trial_count']}/4), "
        f"dominance={aggregate['not_single_trial_dominated']}, "
        f"coverage={aggregate['coverage_pass']}"
    )
    print(f"  decision: {decision}")
    print(f"  report: {report_path}")
    print("======================================================")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[G2S analysis][ERR] {exc}", file=sys.stderr)
        sys.exit(2)
