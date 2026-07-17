#!/usr/bin/env python3
"""Reanalyze real SPMPC fixed-path RGB and model fidelity.

This script applies the 2026-07-14 frozen visual convention to older SPMPC
runs without repeating RGB decoding. It consumes the existing per-frame
red-liquid CSVs and recomputes:

* H_vis = abs(rolling_median(max(left, center, right), 5) - h0)
* fixed-path progress [0.10, 0.90] RGB/model/tracking metrics
* TRACKING start through GOAL_REACHED + 2 s Ferrari-style fidelity
* threshold recall for model-side safety evidence

The default h0 is the calibration YAML zero. The historical first-30-frame
correction is retained only as a sensitivity output.
"""

import argparse
import csv
import math
import re
import statistics
from pathlib import Path

import numpy as np
import rosbag

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from analyze_real_teb_rgb import (
    fidelity_metrics,
    finite_float,
    interpolate,
    nearest_pair,
    percentile,
    read_rgb_csv,
    rms,
    rolling_median,
    write_csv,
)


def method_from_name(name):
    if name.startswith("B0_"):
        return "B0"
    if name.startswith("Bours_"):
        return "B_ours"
    if name.startswith("Bslosh_"):
        return "B_slosh"
    if name.startswith("Bsmooth_"):
        return "B_smooth"
    raise RuntimeError(f"cannot infer method from {name}")


def date_from_path(path):
    match = re.search(r"(2026070[56])_fixed_path_compare", str(path))
    if not match:
        raise RuntimeError(f"cannot infer experiment date from {path}")
    return match.group(1)


def path_length(path_msg):
    points = [pose.pose.position for pose in path_msg.poses]
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))


def find_rgb_csv(roots, bag_stem):
    target = f"{bag_stem}_red_top.csv"
    matches = []
    for root in roots:
        matches.extend(root.rglob(target))
    if len(matches) != 1:
        raise RuntimeError(f"expected one {target}, found {len(matches)}")
    return matches[0]


def read_bag(path):
    fixed_path = None
    stage0 = []
    model = []
    statuses = []
    topics = [
        "/scout/global_path_fixed",
        "/spmpc/debug/stage0_reference",
        "/spmpc/slosh_height",
        "/spmpc/status",
    ]
    with rosbag.Bag(str(path), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            time_s = stamp.to_sec()
            if topic == "/scout/global_path_fixed" and fixed_path is None and len(msg.poses) > 1:
                fixed_path = msg
            elif topic == "/spmpc/debug/stage0_reference" and len(msg.data) >= 11:
                stage0.append((time_s, float(msg.data[0]), float(msg.data[9]), float(msg.data[10])))
            elif topic == "/spmpc/slosh_height":
                model.append((time_s, abs(float(msg.data))))
            elif topic == "/spmpc/status":
                statuses.append((time_s, str(msg.data)))
    if fixed_path is None:
        raise RuntimeError(f"{path}: missing fixed path")
    if not stage0:
        raise RuntimeError(f"{path}: missing /spmpc/debug/stage0_reference")
    if not model:
        raise RuntimeError(f"{path}: missing /spmpc/slosh_height")
    length_m = path_length(fixed_path)
    progress = [(t, max(0.0, min(1.0, s0 / length_m))) for t, s0, _, _ in stage0]
    projection = [(t, math.hypot(contour, lag), ratio) for (t, _, contour, lag), (_, ratio) in zip(stage0, progress)]
    tracking_start = next((t for t, value in statuses if "ACADOS_OK" in value), math.nan)
    goal_reached = next((t for t, value in statuses if "GOAL_REACHED" in value), math.nan)
    return {
        "bag_start": bag_start,
        "bag_end": bag_end,
        "path_length_m": length_m,
        "progress": progress,
        "projection": projection,
        "model": model,
        "tracking_start": tracking_start,
        "goal_reached": goal_reached,
    }


def threshold_sweep(date, method, run, visual, model, thresholds):
    rows = []
    for threshold in thresholds:
        truth = visual > threshold
        true_count = int(np.sum(truth))
        detected = int(np.sum(truth & (model > threshold)))
        rows.append({
            "date": date,
            "method": method,
            "run": run,
            "tau_mm": threshold,
            "true_high_count": true_count,
            "model_detect_count": detected,
            "false_safe_count": true_count - detected,
            "recall": detected / true_count if true_count else math.nan,
        })
    return rows


def analyze_run(bag_path, rgb_roots, args):
    bag = read_bag(bag_path)
    date = date_from_path(bag_path)
    method = method_from_name(bag_path.stem)
    rgb_csv = find_rgb_csv(rgb_roots, bag_path.stem)
    rgb_rows = read_rgb_csv(rgb_csv)
    rgb_times = np.asarray([row["stamp"] for row in rgb_rows], dtype=float)
    rgb_first30 = np.asarray([row["height"] for row in rgb_rows], dtype=float)
    rgb_raw = np.asarray([row["raw_max"] for row in rgb_rows], dtype=float)
    rgb_h0_first30 = float(np.nanmedian(rgb_raw[:30]))
    rgb_smooth = rolling_median(rgb_raw, args.smooth_frames)
    rgb_h0 = 0.0 if args.rgb_zero_mode == "calibration_zero" else rgb_h0_first30
    rgb_values = np.abs(rgb_smooth - rgb_h0)

    rgb_progress = interpolate(bag["progress"], rgb_times)
    core_mask = (
        np.isfinite(rgb_progress)
        & (rgb_progress >= args.progress_lo)
        & (rgb_progress <= args.progress_hi)
    )
    core_indices = np.flatnonzero(core_mask)
    if not core_indices.size:
        raise RuntimeError(f"{bag_path}: no RGB samples in progress window")
    core_values = rgb_values[core_mask]
    core_first30 = rgb_first30[core_mask]
    peak_local = int(np.argmax(core_values))
    peak_index = int(core_indices[peak_local])
    peak_row = rgb_rows[peak_index]

    model_t = np.asarray([item[0] for item in bag["model"]], dtype=float)
    model_v = np.asarray([item[1] for item in bag["model"]], dtype=float)
    model_progress = interpolate(bag["progress"], model_t)
    model_core = model_v[
        np.isfinite(model_progress)
        & (model_progress >= args.progress_lo)
        & (model_progress <= args.progress_hi)
    ]
    projection_core = [
        value for _, value, ratio in bag["projection"]
        if args.progress_lo <= ratio <= args.progress_hi
    ]
    spreads = np.asarray([row["spread"] for row in rgb_rows], dtype=float)[core_mask]
    confidences = np.asarray([row["conf"] for row in rgb_rows], dtype=float)[core_mask]
    upper_clipped = np.asarray([row["upper_clipped"] for row in rgb_rows], dtype=float)[core_mask]

    core_t, core_vis, core_model, core_gaps = nearest_pair(
        rgb_times,
        rgb_values,
        bag["model"],
        args.pair_max_gap_s,
        float(rgb_times[core_indices[0]]),
        float(rgb_times[core_indices[-1]]),
    )
    pair_progress = interpolate(bag["progress"], core_t)
    pair_keep = (
        np.isfinite(pair_progress)
        & (pair_progress >= args.progress_lo)
        & (pair_progress <= args.progress_hi)
    )
    core_fidelity = fidelity_metrics(
        core_t[pair_keep], core_vis[pair_keep], core_model[pair_keep], core_gaps[pair_keep]
    )

    if not math.isfinite(bag["tracking_start"]) or not math.isfinite(bag["goal_reached"]):
        raise RuntimeError(f"{bag_path}: missing ACADOS_OK or GOAL_REACHED")
    fidelity_end = min(bag["bag_end"], bag["goal_reached"] + args.fidelity_tail_s)
    fid_t, fid_vis, fid_model, fid_gaps = nearest_pair(
        rgb_times,
        rgb_values,
        bag["model"],
        args.pair_max_gap_s,
        bag["tracking_start"],
        fidelity_end,
    )
    fidelity = fidelity_metrics(fid_t, fid_vis, fid_model, fid_gaps)
    threshold_rows = threshold_sweep(date, method, bag_path.stem, fid_vis, fid_model, args.thresholds)

    row = {
        "date": date,
        "method": method,
        "run": bag_path.stem,
        "bag": str(bag_path),
        "rgb_csv": str(rgb_csv),
        "success": 1,
        "goal_time_s": bag["goal_reached"] - bag["tracking_start"],
        "path_length_m": bag["path_length_m"],
        "progress_max": max(value for _, value in bag["progress"]),
        "rgb_samples_10_90": int(core_values.size),
        "rgb_zero_mode": args.rgb_zero_mode,
        "rgb_h0_mm": rgb_h0,
        "rgb_h0_first30_mm": rgb_h0_first30,
        "rgb_p95_mm_10_90": percentile(core_values, 95),
        "rgb_peak_mm_10_90": float(np.max(core_values)),
        "rgb_rms_mm_10_90": rms(core_values),
        "rgb_mean_mm_10_90": float(np.mean(core_values)),
        "rgb_first30corr_p95_mm_10_90": percentile(core_first30, 95),
        "rgb_first30corr_peak_mm_10_90": float(np.max(core_first30)),
        "rgb_first30corr_rms_mm_10_90": rms(core_first30),
        "rgb_lcr_spread_p95_mm_10_90": percentile(spreads, 95),
        "rgb_lcr_spread_peak_mm_10_90": float(np.nanmax(spreads)),
        "rgb_conf_mean_10_90": float(np.nanmean(confidences)),
        "rgb_conf_p05_10_90": percentile(confidences, 5),
        "rgb_upper_clip_frac_10_90": float(np.mean(upper_clipped)),
        "rgb_peak_frame_10_90": peak_row["frame_index"],
        "rgb_peak_stamp_10_90": peak_row["stamp"],
        "rgb_peak_bag_offset_s_10_90": peak_row["stamp"] - bag["bag_start"],
        "rgb_peak_progress_10_90": float(rgb_progress[peak_index]),
        "projection_p95_m_10_90": percentile(projection_core, 95),
        "model_p95_mm_10_90": percentile(model_core, 95),
        "model_peak_mm_10_90": float(np.max(model_core)),
        "model_rms_mm_10_90": rms(model_core),
        "core_pair_dt_p95_ms": core_fidelity["pair_dt_p95_ms"],
        "core_gamma_model_pct": core_fidelity["gamma_model_pct"],
        "core_rmse_mm": core_fidelity["rmse_mm"],
        "core_corr": core_fidelity["corr"],
        "core_U_p95_mm": core_fidelity["U_p95_mm"],
        "core_U_max_mm": core_fidelity["U_max_mm"],
        "fidelity_start_bag_offset_s": bag["tracking_start"] - bag["bag_start"],
        "fidelity_end_bag_offset_s": fidelity_end - bag["bag_start"],
    }
    for key, value in fidelity.items():
        row[f"fidelity_{key}"] = value

    series = {
        "date": date,
        "method": method,
        "run": bag_path.stem,
        "progress": rgb_progress[core_mask],
        "visual": core_values,
        "paired_visual": fid_vis,
        "paired_model": fid_model,
    }
    return row, threshold_rows, series


def aggregate(rows):
    metrics = [
        "goal_time_s",
        "projection_p95_m_10_90",
        "rgb_p95_mm_10_90",
        "rgb_peak_mm_10_90",
        "rgb_rms_mm_10_90",
        "rgb_first30corr_p95_mm_10_90",
        "rgb_first30corr_peak_mm_10_90",
        "rgb_first30corr_rms_mm_10_90",
        "model_p95_mm_10_90",
        "model_peak_mm_10_90",
        "model_rms_mm_10_90",
        "core_gamma_model_pct",
        "core_rmse_mm",
        "core_corr",
        "core_U_p95_mm",
        "core_U_max_mm",
        "fidelity_gamma_model_pct",
        "fidelity_rmse_mm",
        "fidelity_corr",
        "fidelity_U_p95_mm",
        "fidelity_U_max_mm",
        "fidelity_under_ratio",
        "fidelity_pair_dt_p95_ms",
        "fidelity_scale_fit_slosh_per_visual",
        "fidelity_visual_p95_mm",
        "fidelity_model_p95_mm",
        "fidelity_visual_peak_mm",
        "fidelity_model_peak_mm",
        "fidelity_visual_rms_mm",
        "fidelity_model_rms_mm",
    ]
    output = []
    keys = sorted({(row["date"], row["method"]) for row in rows})
    for date, method in keys:
        group = [row for row in rows if row["date"] == date and row["method"] == method]
        item = {
            "date": date,
            "method": method,
            "run_count": len(group),
            "success_count": sum(row["success"] for row in group),
        }
        for metric in metrics:
            values = [float(row[metric]) for row in group if math.isfinite(float(row[metric]))]
            item[f"{metric}_mean"] = statistics.fmean(values) if values else math.nan
            item[f"{metric}_std"] = statistics.stdev(values) if len(values) > 1 else 0.0 if values else math.nan
        output.append(item)
    return output


def write_plots(plot_dir, series, aggregate_rows):
    if plt is None:
        return
    plot_dir.mkdir(parents=True, exist_ok=True)
    colors = {
        "B0": "#666666",
        "B_smooth": "#1f77b4",
        "B_slosh": "#2ca02c",
        "B_ours": "#d62728",
    }
    for date in sorted({item["date"] for item in series}):
        selected = [item for item in series if item["date"] == date]
        fig, ax = plt.subplots(figsize=(11, 5.5), dpi=180)
        for item in selected:
            ax.plot(
                item["progress"],
                item["visual"],
                color=colors[item["method"]],
                alpha=0.42,
                linewidth=1.0,
                label=item["run"],
            )
        ax.set_xlabel("fixed-path progress")
        ax.set_ylabel("H_vis = abs(max-LCR smooth) [mm]")
        ax.set_xlim(0.10, 0.90)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(plot_dir / f"{date}_rgb_progress_10_90.png")
        plt.close(fig)

        groups = [row for row in aggregate_rows if row["date"] == date]
        fig, ax = plt.subplots(figsize=(9, 5.4), dpi=180)
        metrics = ["rgb_p95_mm_10_90", "rgb_peak_mm_10_90", "rgb_rms_mm_10_90"]
        labels = ["p95", "peak", "RMS"]
        x = np.arange(len(metrics), dtype=float)
        width = 0.8 / len(groups)
        for index, row in enumerate(groups):
            means = [row[f"{metric}_mean"] for metric in metrics]
            stds = [row[f"{metric}_std"] for metric in metrics]
            ax.bar(
                x + (index - (len(groups) - 1) / 2.0) * width,
                means,
                width,
                yerr=stds,
                capsize=3,
                color=colors[row["method"]],
                alpha=0.82,
                label=row["method"],
            )
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_ylabel("RGB H_vis [mm], mean +/- sample std")
        ax.grid(True, axis="y", alpha=0.25)
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir / f"{date}_rgb_aggregate_metrics.png")
        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--red-infer-dir", action="append", type=Path, required=True)
    parser.add_argument("--run-csv", type=Path, required=True)
    parser.add_argument("--aggregate-csv", type=Path, required=True)
    parser.add_argument("--threshold-csv", type=Path, required=True)
    parser.add_argument("--plot-dir", type=Path, required=True)
    parser.add_argument("--progress-lo", type=float, default=0.10)
    parser.add_argument("--progress-hi", type=float, default=0.90)
    parser.add_argument("--pair-max-gap-s", type=float, default=0.15)
    parser.add_argument("--fidelity-tail-s", type=float, default=2.0)
    parser.add_argument("--rgb-zero-mode", choices=["calibration_zero", "first30"], default="calibration_zero")
    parser.add_argument("--smooth-frames", type=int, default=5)
    parser.add_argument("--thresholds-mm", default="0.5,1.0,1.5,2.0,3.0,4.0")
    args = parser.parse_args()
    args.thresholds = [float(value.strip()) for value in args.thresholds_mm.split(",") if value.strip()]
    return args


def main():
    args = parse_args()
    rows = []
    threshold_rows = []
    series = []
    for raw_path in args.bags:
        row, run_thresholds, run_series = analyze_run(Path(raw_path), args.red_infer_dir, args)
        rows.append(row)
        threshold_rows.extend(run_thresholds)
        series.append(run_series)
        print(
            f"{row['date']} {row['method']} {row['run']}: "
            f"RGB p95/peak/rms={row['rgb_p95_mm_10_90']:.3f}/"
            f"{row['rgb_peak_mm_10_90']:.3f}/{row['rgb_rms_mm_10_90']:.3f} mm, "
            f"model p95={row['model_p95_mm_10_90']:.3f} mm, "
            f"fidelity corr={row['fidelity_corr']:.3f}"
        )
    aggregate_rows = aggregate(rows)
    write_csv(args.run_csv, rows)
    write_csv(args.aggregate_csv, aggregate_rows)
    write_csv(args.threshold_csv, threshold_rows)
    write_plots(args.plot_dir, series, aggregate_rows)
    print(f"wrote {args.run_csv}")
    print(f"wrote {args.aggregate_csv}")
    print(f"wrote {args.threshold_csv}")


if __name__ == "__main__":
    main()
