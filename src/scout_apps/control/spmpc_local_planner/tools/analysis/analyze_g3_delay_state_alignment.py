#!/usr/bin/env python3
"""Audit the failed G3 release and select a safe delay/state mode for G3R.

This is a read-only development analysis.  It compares the current processed-
IMU liquid observer, the legacy fixed-delay prediction, odom response timing,
and the independent online RGB scalar.  It never replays ROS, publishes a
command, or treats RGB frames as independent experimental units.
"""

import argparse
import collections
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path

import numpy as np

try:
    import rosbag  # type: ignore
except ImportError:  # pragma: no cover - depends on sourced ROS environment
    rosbag = None


POSTFLIGHT_GLOB = "DEV_G3_*_g3_postflight.json"
EXPECTED_PROTOCOL = "G3_processed_imu_W5_vs_Bsmooth_online_RGB_development_v1"
RGB_TOPIC = "/liquid/measurement"
IMU_OBSERVER_TOPIC = "/spmpc/debug/slosh_observer_imu"
PREDICTED_STATE_TOPIC = "/spmpc/debug/predicted_state"
CMD_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, help="Directory containing the eight legacy G3 bags/postflights")
    parser.add_argument("--out-dir", default="")
    parser.add_argument("--minimum-eligible-rows", type=int, default=6)
    parser.add_argument("--minimum-raw-rgb-correlation", type=float, default=0.65)
    parser.add_argument("--minimum-correlation-advantage", type=float, default=0.08)
    parser.add_argument("--future-delay-sec", type=float, default=0.22)
    return parser.parse_args()


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def percentile(values, quantile):
    clean = sorted(float(value) for value in values if finite(value))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    position = (len(clean) - 1) * quantile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return clean[lower]
    fraction = position - lower
    return clean[lower] * (1.0 - fraction) + clean[upper] * fraction


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(8 * 1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def parse_multiarray(msg):
    dimensions = getattr(getattr(msg, "layout", None), "dim", [])
    label = dimensions[0].label if dimensions else ""
    fields = [item.strip() for item in label.split(",") if item.strip()]
    return {
        fields[index] if index < len(fields) else "field_{}".format(index): float(value)
        for index, value in enumerate(getattr(msg, "data", []))
    }


def causal_rolling_median(records, window=5, max_gap_sec=0.35):
    history = collections.deque(maxlen=window)
    output = []
    previous = None
    for stamp, value in sorted(records):
        if previous is not None and stamp - previous > max_gap_sec:
            history.clear()
        history.append(float(value))
        output.append((float(stamp), float(statistics.median(history))))
        previous = stamp
    return output


def clean_series(records):
    output = []
    for stamp, value in sorted(
        (float(stamp), float(value))
        for stamp, value in records
        if finite(stamp) and finite(value)
    ):
        if output and stamp <= output[-1][0] + 1.0e-9:
            if abs(stamp - output[-1][0]) <= 1.0e-9:
                output[-1] = (stamp, value)
            continue
        output.append((stamp, value))
    return output


def correlation_at_lag(left, right, lag_sec, sample_dt=0.01, minimum_duration=5.0):
    left = clean_series(left)
    right = clean_series(right)
    if len(left) < 3 or len(right) < 3:
        return math.nan
    lt = np.asarray([item[0] for item in left])
    lv = np.asarray([item[1] for item in left])
    rt = np.asarray([item[0] for item in right])
    rv = np.asarray([item[1] for item in right])
    start = max(lt[0], rt[0] - lag_sec)
    end = min(lt[-1], rt[-1] - lag_sec)
    if end - start < minimum_duration:
        return math.nan
    grid = np.arange(start, end, sample_dt)
    left_values = np.interp(grid, lt, lv)
    right_values = np.interp(grid + lag_sec, rt, rv)
    if np.std(left_values) <= 1.0e-9 or np.std(right_values) <= 1.0e-9:
        return math.nan
    return float(np.corrcoef(left_values, right_values)[0, 1])


def best_positive_lag(left, right, maximum_lag=0.50, sample_dt=0.01):
    best_lag = math.nan
    best_correlation = -math.inf
    for lag in np.arange(0.0, maximum_lag + sample_dt / 2.0, sample_dt):
        correlation = correlation_at_lag(left, right, float(lag), sample_dt)
        if finite(correlation) and correlation > best_correlation:
            best_lag = float(lag)
            best_correlation = float(correlation)
    return best_lag, best_correlation if finite(best_correlation) else math.nan


def derivative_series(records, sample_dt=0.01, smoothing_sec=0.05):
    clean = clean_series(records)
    if len(clean) < 3:
        return []
    stamps = np.asarray([item[0] for item in clean])
    values = np.asarray([item[1] for item in clean])
    grid = np.arange(stamps[0], stamps[-1], sample_dt)
    interpolated = np.interp(grid, stamps, values)
    window = max(1, int(round(smoothing_sec / sample_dt)))
    smoothed = np.convolve(interpolated, np.ones(window) / window, mode="same")
    derivative = np.gradient(smoothed, sample_dt)
    return list(zip(grid.tolist(), derivative.tolist()))


def read_trial(report, future_delay_sec):
    bag_path = Path(report["bag"])
    motion_start = float(report["motion_start_sec"])
    motion_end = float(report["motion_end_sec"])
    window_end = float(report["window_end_sec"])
    read_start = motion_start - 0.60
    read_end = window_end + 0.60
    rgb = []
    raw_height = []
    predicted_height = []
    command_v = []
    command_w = []
    odom_v = []
    odom_w = []
    imu_ax = []
    imu_w = []
    imu_epochs = set()

    topics = (RGB_TOPIC, IMU_OBSERVER_TOPIC, PREDICTED_STATE_TOPIC, CMD_TOPIC, ODOM_TOPIC)
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            bag_time = stamp.to_sec()
            if bag_time < read_start or bag_time > read_end:
                continue
            if topic == RGB_TOPIC:
                if bool(msg.valid) and bool(msg.zero_locked) and int(msg.status_code) == 0:
                    source_time = msg.header.stamp.to_sec()
                    if source_time > 0.0 and finite(msg.height_max_lcr_mm):
                        rgb.append((source_time, max(0.0, float(msg.height_max_lcr_mm))))
            elif topic == IMU_OBSERVER_TOPIC:
                imu_epochs.add(int(msg.reset_epoch))
                if bool(msg.valid) and str(msg.input_status) == "READY":
                    state_time = msg.state_stamp.to_sec()
                    accel_time = msg.accel_effective_stamp.to_sec()
                    gyro_time = msg.gyro_effective_stamp.to_sec()
                    raw_height.append((state_time, 1000.0 * float(msg.modal_height_m)))
                    imu_ax.append((accel_time, float(msg.ax_mps2)))
                    imu_w.append((gyro_time, float(msg.omega_z_radps)))
            elif topic == PREDICTED_STATE_TOPIC:
                parsed = parse_multiarray(msg)
                if parsed.get("valid", 0.0) > 0.5:
                    predicted_height.append((bag_time, parsed.get("h_modal_mm", math.nan)))
            elif topic == CMD_TOPIC and motion_start - 0.5 <= bag_time <= motion_end + 0.5:
                command_v.append((bag_time, float(msg.linear.x)))
                command_w.append((bag_time, float(msg.angular.z)))
            elif topic == ODOM_TOPIC and motion_start - 0.5 <= bag_time <= motion_end + 0.5:
                source_time = msg.header.stamp.to_sec() or bag_time
                odom_v.append((source_time, float(msg.twist.twist.linear.x)))
                odom_w.append((source_time, float(msg.twist.twist.angular.z)))

    rgb = causal_rolling_median(rgb)
    rgb_window = [item for item in rgb if motion_start <= item[0] <= window_end]
    raw_window = [item for item in raw_height if motion_start <= item[0] <= window_end]
    predicted_window = [item for item in predicted_height if motion_start <= item[0] <= window_end]
    cmd_dv = derivative_series(command_v)
    cmd_odom_v_lag, cmd_odom_v_corr = best_positive_lag(command_v, odom_v)
    cmd_odom_w_lag, cmd_odom_w_corr = best_positive_lag(command_w, odom_w)
    cmd_imu_ax_lag, cmd_imu_ax_corr = best_positive_lag(cmd_dv, imu_ax)
    cmd_imu_w_lag, cmd_imu_w_corr = best_positive_lag(command_w, imu_w)

    return {
        "rgb_p95_mm": percentile([value for _, value in rgb_window], 0.95),
        "raw_imu_p95_mm": percentile([value for _, value in raw_window], 0.95),
        "predicted_p95_mm": percentile([value for _, value in predicted_window], 0.95),
        "raw_rgb_corr_zero": correlation_at_lag(raw_window, rgb_window, 0.0),
        "predicted_rgb_corr_zero": correlation_at_lag(predicted_window, rgb_window, 0.0),
        "predicted_rgb_corr_future": correlation_at_lag(
            predicted_window, rgb_window, future_delay_sec
        ),
        "cmd_odom_linear_delay_sec": cmd_odom_v_lag,
        "cmd_odom_linear_corr": cmd_odom_v_corr,
        "cmd_odom_angular_delay_sec": cmd_odom_w_lag,
        "cmd_odom_angular_corr": cmd_odom_w_corr,
        "cmd_imu_accel_delay_sec": cmd_imu_ax_lag,
        "cmd_imu_accel_corr": cmd_imu_ax_corr,
        "cmd_imu_angular_delay_sec": cmd_imu_w_lag,
        "cmd_imu_angular_corr": cmd_imu_w_corr,
        "imu_reset_epochs": sorted(imu_epochs),
        "rgb_samples": len(rgb_window),
        "raw_samples": len(raw_window),
        "predicted_samples": len(predicted_window),
    }


def evaluate_release(rows, minimum_eligible_rows, minimum_raw_corr, minimum_advantage):
    eligible = [row for row in rows if row["eligible"]]
    raw_correlations = [row["metrics"]["raw_rgb_corr_zero"] for row in eligible]
    predicted_correlations = [row["metrics"]["predicted_rgb_corr_zero"] for row in eligible]
    raw_median = statistics.median(raw_correlations) if raw_correlations else math.nan
    predicted_median = (
        statistics.median(predicted_correlations) if predicted_correlations else math.nan
    )
    advantage = raw_median - predicted_median if finite(raw_median) and finite(predicted_median) else math.nan
    blockers = []
    if len(eligible) < minimum_eligible_rows:
        blockers.append("eligible rows {} < {}".format(len(eligible), minimum_eligible_rows))
    if not finite(raw_median) or raw_median < minimum_raw_corr:
        blockers.append("median raw-IMU/RGB correlation is below threshold")
    if not finite(advantage) or advantage < minimum_advantage:
        blockers.append("raw-IMU correlation advantage is below threshold")

    paired_deltas = []
    for block in ("01", "02", "03", "04"):
        group = [row for row in rows if row["block"] == block and row["eligible"]]
        by_condition = {row["condition"]: row for row in group}
        if set(by_condition) == {"Bsmooth", "W5"}:
            paired_deltas.append(
                by_condition["Bsmooth"]["metrics"]["rgb_p95_mm"]
                - by_condition["W5"]["metrics"]["rgb_p95_mm"]
            )
    return {
        "status": "PASS_FOR_G3R_SCREENING" if not blockers else "NO_GO",
        "decision": "USE_RAW_MEASUREMENT_STATE_WITH_DELAY_SHADOW"
        if not blockers
        else "NO_RELEASE_SELECTION",
        "eligible_row_count": len(eligible),
        "median_raw_imu_rgb_corr_zero": raw_median,
        "median_legacy_predicted_rgb_corr_zero": predicted_median,
        "median_correlation_advantage": advantage,
        "eligible_paired_rgb_deltas_bsmooth_minus_w5_mm": paired_deltas,
        "eligible_paired_mean_delta_mm": statistics.mean(paired_deltas)
        if paired_deltas
        else math.nan,
        "recommended_delay_phase_mode": "shadow",
        "recommended_closed_loop_delay_compensation": False,
        "blockers": blockers,
    }


def main():
    args = parse_args()
    if rosbag is None:
        raise SystemExit("source ROS before running: rosbag Python module is unavailable")
    root = Path(args.root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else root / "analysis"
    postflights = sorted(root.glob(POSTFLIGHT_GLOB))
    if len(postflights) != 8:
        raise SystemExit("expected 8 legacy G3 postflights, found {}".format(len(postflights)))

    rows = []
    seen_rows = set()
    for path in postflights:
        with path.open(encoding="utf-8") as stream:
            report = json.load(stream)
        row_id = str(report.get("row"))
        if row_id in seen_rows:
            raise SystemExit("duplicate legacy G3 row {}".format(row_id))
        seen_rows.add(row_id)
        if report.get("protocol") != EXPECTED_PROTOCOL:
            raise SystemExit("legacy G3 row {} protocol mismatch".format(row_id))
        bag_path = Path(str(report.get("bag", ""))).expanduser().resolve()
        if bag_path.parent != root or not bag_path.is_file():
            raise SystemExit("legacy G3 row {} bag is missing/outside source root".format(row_id))
        if report.get("bag_size_bytes") != bag_path.stat().st_size:
            raise SystemExit("legacy G3 row {} bag size differs from postflight".format(row_id))
        expected_bag_sha = str(report.get("bag_sha256", ""))
        actual_bag_sha = sha256_file(bag_path)
        if len(expected_bag_sha) != 64 or actual_bag_sha != expected_bag_sha:
            raise SystemExit("legacy G3 row {} bag SHA-256 differs from postflight".format(row_id))
        metrics = read_trial(report, args.future_delay_sec)
        eligibility_reasons = []
        if report.get("status") != "PASS":
            eligibility_reasons.append("postflight_not_pass")
        for field in (
            "rgb_p95_mm",
            "raw_imu_p95_mm",
            "predicted_p95_mm",
            "raw_rgb_corr_zero",
            "predicted_rgb_corr_zero",
        ):
            if not finite(metrics.get(field)):
                eligibility_reasons.append("nonfinite_{}".format(field))
        rows.append(
            {
                "row": row_id,
                "block": str(report.get("block")),
                "position": str(report.get("position")),
                "condition": str(report.get("condition")),
                "eligible": not eligibility_reasons,
                "eligibility_reasons": eligibility_reasons,
                "postflight_status": report.get("status"),
                "postflight": str(path),
                "postflight_sha256": sha256_file(path),
                "bag": str(bag_path),
                "bag_sha256": actual_bag_sha,
                "bag_integrity_verified": True,
                "postflight_failures": report.get("failures", []),
                "metrics": metrics,
            }
        )
    expected_rows = {"{:02d}".format(index) for index in range(1, 9)}
    if seen_rows != expected_rows:
        raise SystemExit(
            "legacy G3 row set mismatch: expected={} actual={}".format(
                sorted(expected_rows), sorted(seen_rows)
            )
        )
    rows.sort(key=lambda row: int(row["row"]))
    aggregate = evaluate_release(
        rows,
        args.minimum_eligible_rows,
        args.minimum_raw_rgb_correlation,
        args.minimum_correlation_advantage,
    )
    report = {
        "schema_version": 1,
        "report_type": "G3_DELAY_STATE_ALIGNMENT",
        "scope": "DEVELOPMENT_ONLY",
        "source_root": str(root),
        "analysis_script_sha256": sha256_file(Path(__file__).resolve()),
        "status": aggregate["status"],
        "decision": aggregate["decision"],
        "thresholds": {
            "minimum_eligible_rows": args.minimum_eligible_rows,
            "minimum_raw_rgb_correlation": args.minimum_raw_rgb_correlation,
            "minimum_correlation_advantage": args.minimum_correlation_advantage,
            "future_delay_sec": args.future_delay_sec,
        },
        "aggregate": aggregate,
        "rows": rows,
        "release_contract": {
            "delay_phase_mode": "shadow",
            "solver_uses_predicted_robot_state": False,
            "solver_uses_predicted_liquid_state": False,
            "solver_liquid_source": "processed_imu",
            "imu_subscriber_queue_size_minimum": 10,
            "fallback_policy": "fail_closed",
            "purpose": "single-run weight screening before any replicated efficacy claim",
        },
        "limitations": [
            "Online RGB is a nonnegative smoothed envelope and cannot identify signed liquid phase.",
            "Legacy failed rows remain descriptive and are excluded from the eligibility aggregate.",
            "This report selects a development screening mode; it is not a formal efficacy release.",
            "A timestamp-aware actuator model remains future work and must be independently validated before closed-loop promotion.",
        ],
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    out_json = out_dir / "G3_DELAY_STATE_ALIGNMENT_REPORT.json"
    out_csv = out_dir / "G3_DELAY_STATE_ALIGNMENT_ROWS.csv"
    out_json.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with out_csv.open("w", encoding="utf-8", newline="") as stream:
        fieldnames = (
            "row",
            "block",
            "condition",
            "eligible",
            "rgb_p95_mm",
            "raw_imu_p95_mm",
            "predicted_p95_mm",
            "raw_rgb_corr_zero",
            "predicted_rgb_corr_zero",
            "predicted_rgb_corr_future",
            "cmd_odom_linear_delay_sec",
            "cmd_odom_angular_delay_sec",
            "cmd_imu_accel_delay_sec",
            "cmd_imu_angular_delay_sec",
        )
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "row": row["row"],
                    "block": row["block"],
                    "condition": row["condition"],
                    "eligible": row["eligible"],
                    **{field: row["metrics"].get(field) for field in fieldnames if field in row["metrics"]},
                }
            )
    for artifact in (out_json, out_csv):
        artifact.with_suffix(artifact.suffix + ".sha256").write_text(
            "{}  {}\n".format(sha256_file(artifact), artifact), encoding="utf-8"
        )
    print("[G3 alignment] {}: {}".format(report["status"], report["decision"]))
    print("  eligible rows = {}".format(aggregate["eligible_row_count"]))
    print(
        "  median corr raw/predicted = {:.3f}/{:.3f}; advantage={:.3f}".format(
            aggregate["median_raw_imu_rgb_corr_zero"],
            aggregate["median_legacy_predicted_rgb_corr_zero"],
            aggregate["median_correlation_advantage"],
        )
    )
    print("  report = {}".format(out_json))
    return 0 if report["status"] == "PASS_FOR_G3R_SCREENING" else 1


if __name__ == "__main__":
    raise SystemExit(main())
