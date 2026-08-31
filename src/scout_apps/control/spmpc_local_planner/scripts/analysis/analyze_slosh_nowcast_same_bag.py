#!/usr/bin/env python3
"""Compare O0/I0/I1/L22 against frozen online RGB in the same bag.

All four estimates are evaluated at the comparison message target epoch.  The
script fixes lag to zero, scale to one, and RGB smoothing to a centered
five-sample median; it never searches parameters per trial.
"""

import argparse
import csv
import hashlib
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

import rosbag

from slosh_nowcast_analysis_core import (
    METHODS,
    align_and_score,
    centered_rolling_median,
    development_decision,
    interpolate_at,
    json_safe,
    motion_window,
)


COMPARISON_TOPIC = "/spmpc/debug/slosh_estimator_comparison"
MEASUREMENT_TOPIC = "/liquid/measurement"
COMMAND_TOPIC = "/cmd_vel"
COMPARISON_TYPE = "spmpc_local_planner/SloshEstimatorComparison"
MEASUREMENT_TYPE = "realsense_liquid_measurement/OnlineLiquidMeasurement"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", action="append", required=True, help="Repeat for each bag.")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--comparison-topic", default=COMPARISON_TOPIC)
    parser.add_argument("--measurement-topic", default=MEASUREMENT_TOPIC)
    parser.add_argument("--smooth-frames", type=int, default=5)
    parser.add_argument("--max-interpolation-gap-sec", type=float, default=0.10)
    parser.add_argument("--motion-linear-threshold", type=float, default=0.03)
    parser.add_argument("--motion-angular-threshold", type=float, default=0.03)
    parser.add_argument("--motion-tail-sec", type=float, default=3.0)
    parser.add_argument("--min-coverage", type=float, default=0.98)
    parser.add_argument(
        "--evidence-class",
        choices=("development", "held_out"),
        default="development",
        help="Only explicitly declared held_out bags can produce an offline release pass.",
    )
    return parser.parse_args()


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp_sec(stamp):
    try:
        return float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return 0.0


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def clean_visual_measurement(msg):
    value = float(msg.height_max_lcr_mm)
    valid = (
        stamp_sec(msg.header.stamp) > 0.0
        and bool(msg.valid)
        and bool(msg.zero_locked)
        and int(msg.status_code) == 0
        and not bool(msg.any_clipped)
        and finite(value)
    )
    return max(0.0, value) if valid else None


def load_bag(path, comparison_topic, measurement_topic, smooth_frames,
             linear_threshold, angular_threshold, tail_sec):
    commands = []
    visual_raw = []
    estimates = {method: [] for method in METHODS}
    method_diagnostics = {
        method: {"valid": 0, "invalid": 0, "applied_to_solver": 0, "statuses": Counter()}
        for method in METHODS
    }
    comparison_count = 0
    schema_versions = set()
    solver_consumes_liquid_count = 0
    comparison_disabled_count = 0

    with rosbag.Bag(str(path), "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        expected = {
            comparison_topic: COMPARISON_TYPE,
            measurement_topic: MEASUREMENT_TYPE,
            COMMAND_TOPIC: "geometry_msgs/Twist",
        }
        for topic, msg_type in expected.items():
            if topic not in topic_info:
                raise RuntimeError("missing required topic {} in {}".format(topic, path))
            if topic_info[topic].msg_type != msg_type:
                raise RuntimeError(
                    "unexpected type for {}: expected {}, got {}".format(
                        topic, msg_type, topic_info[topic].msg_type
                    )
                )

        for topic, msg, bag_stamp in bag.read_messages(
            topics=[comparison_topic, measurement_topic, COMMAND_TOPIC]
        ):
            if topic == COMMAND_TOPIC:
                commands.append(
                    (bag_stamp.to_sec(), float(msg.linear.x), float(msg.angular.z))
                )
                continue
            if topic == measurement_topic:
                visual_raw.append(
                    (stamp_sec(msg.header.stamp), clean_visual_measurement(msg))
                )
                continue

            comparison_count += 1
            schema_versions.add(int(msg.schema_version))
            if bool(msg.solver_consumes_liquid):
                solver_consumes_liquid_count += 1
            if not bool(msg.comparison_enabled):
                comparison_disabled_count += 1
            target = stamp_sec(msg.target_stamp)
            if target <= 0.0:
                target = stamp_sec(msg.header.stamp)
            for method in METHODS:
                estimate = getattr(msg, method.lower())
                diag = method_diagnostics[method]
                diag["statuses"][str(estimate.status)] += 1
                diag["applied_to_solver"] += int(bool(estimate.applied_to_solver))
                value_mm = 1000.0 * float(estimate.total_height_m)
                valid = bool(estimate.valid) and target > 0.0 and finite(value_mm)
                diag["valid" if valid else "invalid"] += 1
                if valid:
                    estimates[method].append((target, value_mm))

    if comparison_count == 0:
        raise RuntimeError("comparison topic is empty in {}".format(path))
    if schema_versions != {1}:
        raise RuntimeError("comparison schema must be exactly {1}, got {}".format(schema_versions))
    if comparison_disabled_count:
        raise RuntimeError("{} comparison messages report comparison_enabled=false".format(comparison_disabled_count))

    start, end = motion_window(
        commands,
        linear_threshold=linear_threshold,
        angular_threshold=angular_threshold,
        tail_sec=tail_sec,
    )
    visual_raw.sort(key=lambda item: item[0])
    visual_filtered = centered_rolling_median(visual_raw, smooth_frames)
    visual = [
        (stamp, value)
        for stamp, value in visual_filtered
        if value is not None and start <= stamp <= end
    ]
    if not visual:
        raise RuntimeError("no clean RGB liquid samples in the frozen motion window")
    for method in METHODS:
        estimates[method].sort(key=lambda item: item[0])
        if not estimates[method]:
            raise RuntimeError("{} has no valid estimates in {}".format(method, path))

    return {
        "motion_window": (start, end),
        "visual": visual,
        "estimates": estimates,
        "comparison_count": comparison_count,
        "schema_versions": sorted(schema_versions),
        "solver_consumes_liquid_count": solver_consumes_liquid_count,
        "method_diagnostics": method_diagnostics,
    }


def write_aligned_csv(path, bag_name, method_metrics):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["bag", "method", "stamp", "visual_mm", "estimate_mm", "signed_error_mm"]
        )
        for method in METHODS:
            for stamp, visual, estimate, error in method_metrics[method]["aligned_rows"]:
                writer.writerow([bag_name, method, stamp, visual, estimate, error])


def write_metric_csv(path, trials):
    fields = [
        "bag", "method", "eligible_visual_samples", "aligned_samples", "coverage",
        "mae_mm", "rmse_mm", "p95_abs_error_mm", "signed_bias_mm", "corr",
        "visual_peak_mm", "estimate_peak_mm", "peak_timing_error_sec",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for trial in trials:
            for method in METHODS:
                row = {"bag": trial["bag"], "method": method}
                row.update(trial["methods"][method])
                writer.writerow({field: row.get(field) for field in fields})


def plot_trial(path, label, loaded, method_metrics):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    start, end = loaded["motion_window"]
    colors = {"O0": "#7f7f7f", "I0": "#1f77b4", "I1": "#2ca02c", "L22": "#d62728"}
    figure, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    visual_t = [stamp - start for stamp, _ in loaded["visual"]]
    visual_y = [value for _, value in loaded["visual"]]
    axes[0].plot(visual_t, visual_y, color="black", linewidth=2.0, label="RGB (median-5)")
    for method in METHODS:
        rows = loaded["estimates"][method]
        axes[0].plot(
            [stamp - start for stamp, _ in rows],
            [value for _, value in rows],
            color=colors[method], linewidth=1.0, alpha=0.85, label=method,
        )
        aligned = method_metrics[method]["aligned_rows"]
        axes[1].plot(
            [row[0] - start for row in aligned],
            [abs(row[3]) for row in aligned],
            color=colors[method], linewidth=1.0, label=method,
        )
    axes[0].set_ylabel("liquid proxy [mm]")
    axes[0].set_title("{}: fixed lag=0, scale=1, comparison epoch=target_stamp".format(label))
    axes[0].legend(ncol=5, loc="upper right")
    axes[0].grid(alpha=0.25)
    axes[1].set_ylabel("absolute error [mm]")
    axes[1].set_xlabel("time from first effective command [s]")
    axes[1].set_xlim(0.0, max(0.0, end - start))
    axes[1].legend(ncol=4, loc="upper right")
    axes[1].grid(alpha=0.25)
    figure.tight_layout()
    figure.savefig(str(path), dpi=160)
    plt.close(figure)


def metric_without_rows(metrics):
    return {key: value for key, value in metrics.items() if key != "aligned_rows"}


def analyze_one(bag_path, out_dir, args):
    loaded = load_bag(
        bag_path,
        args.comparison_topic,
        args.measurement_topic,
        args.smooth_frames,
        args.motion_linear_threshold,
        args.motion_angular_threshold,
        args.motion_tail_sec,
    )
    method_metrics = {
        method: align_and_score(
            loaded["visual"], loaded["estimates"][method], args.max_interpolation_gap_sec
        )
        for method in METHODS
    }
    trial_dir = out_dir / bag_path.stem
    trial_dir.mkdir(parents=True, exist_ok=False)
    aligned_csv = trial_dir / "aligned_samples.csv"
    plot_path = trial_dir / "O0_I0_I1_L22_vs_RGB.png"
    write_aligned_csv(aligned_csv, bag_path.name, method_metrics)
    plot_trial(plot_path, bag_path.stem, loaded, method_metrics)

    diagnostics = {}
    for method, row in loaded["method_diagnostics"].items():
        diagnostics[method] = dict(row)
        diagnostics[method]["statuses"] = dict(row["statuses"])
    result = {
        "bag": str(bag_path),
        "bag_sha256": file_sha256(bag_path),
        "protocol": "SMPCC_slosh_state_nowcast_dev_v1",
        "analysis_contract": {
            "comparison_epoch": "SloshEstimatorComparison.target_stamp",
            "visual_field": "height_max_lcr_mm",
            "visual_filter": "centered_median_5",
            "lag_sec": 0.0,
            "scale": 1.0,
            "max_interpolation_gap_sec": args.max_interpolation_gap_sec,
            "motion_tail_sec": args.motion_tail_sec,
        },
        "motion_window": {
            "start_sec": loaded["motion_window"][0],
            "end_sec": loaded["motion_window"][1],
        },
        "comparison_count": loaded["comparison_count"],
        "solver_consumes_liquid_count": loaded["solver_consumes_liquid_count"],
        "visual_eligible_samples": len(loaded["visual"]),
        "method_diagnostics": diagnostics,
        "methods": {
            method: metric_without_rows(method_metrics[method]) for method in METHODS
        },
        "artifacts": {"aligned_csv": str(aligned_csv), "plot": str(plot_path)},
    }
    (trial_dir / "report.json").write_text(
        json.dumps(json_safe(result), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return result


def write_markdown(path, trials, decision):
    lines = [
        "# O0 / I0 / I1 / L22 同 bag 比较",
        "",
        "固定口径：RGB 五点居中中位数、lag=0、scale=1；四路均按比较消息的 `target_stamp` 对齐。",
        "",
        "| bag | 方法 | coverage | MAE mm | RMSE mm | P95 abs mm | corr | peak timing ms |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for trial in trials:
        for method in METHODS:
            row = trial["methods"][method]
            lines.append(
                "| {} | {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {} | {} |".format(
                    Path(trial["bag"]).name,
                    method,
                    row["coverage"],
                    row["mae_mm"],
                    row["rmse_mm"],
                    row["p95_abs_error_mm"],
                    "{:.3f}".format(row["corr"]) if finite(row["corr"]) else "N/A",
                    "{:.1f}".format(1000.0 * row["peak_timing_error_sec"])
                    if finite(row["peak_timing_error_sec"]) else "N/A",
                )
            )
    lines.extend(
        [
            "",
            "## 自动判定",
            "",
            "- 状态：`{}`".format(decision["status"]),
            "- 有效 bag：`{}/{}`".format(
                decision["valid_bag_count"], decision["required_independent_bags"]
            ),
            "- I1 优于 I0：`{}/{}`".format(
                decision["i1_better_count"], decision["required_directional_count"]
            ),
            "- 注意：development 数据不会被脚本命名为 held-out 放行证据。",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    if args.smooth_frames != 5:
        raise RuntimeError("frozen protocol requires --smooth-frames 5")
    if args.max_interpolation_gap_sec <= 0.0:
        raise RuntimeError("--max-interpolation-gap-sec must be positive")
    bags = []
    for raw in args.bag:
        path = Path(raw).expanduser().resolve()
        if not path.is_file():
            raise RuntimeError("bag does not exist: {}".format(path))
        if path not in bags:
            bags.append(path)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=False)
    trials = [analyze_one(path, out_dir, args) for path in bags]
    decision = development_decision(
        trials, evidence_class=args.evidence_class, min_bags=3,
        min_coverage=args.min_coverage,
    )
    write_metric_csv(out_dir / "metrics.csv", trials)
    write_markdown(out_dir / "summary.md", trials, decision)
    aggregate = {
        "protocol": "SMPCC_slosh_state_nowcast_dev_v1",
        "evidence_class": args.evidence_class,
        "trials": trials,
        "decision": decision,
    }
    (out_dir / "report.json").write_text(
        json.dumps(json_safe(aggregate), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("[slosh-nowcast-analysis] status={}".format(decision["status"]))
    print("[slosh-nowcast-analysis] report={}".format(out_dir / "report.json"))
    print("[slosh-nowcast-analysis] summary={}".format(out_dir / "summary.md"))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # fail closed with one concise operator-facing line
        print("[slosh-nowcast-analysis] ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(2)
