#!/usr/bin/env python3
"""Analyze real fixed-path TEB RGB liquid-height and model fidelity.

The primary comparison window is normalized fixed-path progress [0.10, 0.90].
RGB truth follows the fixed workflow:

    H_vis = abs(h_mm_max_lcr_smooth_corr)

Ferrari-style fidelity is additionally evaluated from TRACKING start through
GOAL_REACHED + 2 s, with RGB samples paired to the nearest /slosh/height sample.
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


RGB_HEIGHT_COL = "h_mm_max_lcr_smooth_corr"
RGB_RAW_MAX_COL = "h_mm_max_lcr"
RGB_LCR_COLS = ("h_mm_left", "h_mm_center", "h_mm_right")


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def percentile(values, q):
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    return float(np.percentile(array, q)) if array.size else math.nan


def rms(values):
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    return float(np.sqrt(np.mean(array * array))) if array.size else math.nan


def candidate_from_name(name):
    match = re.search(r"_(T[45])_0713_formal_", name)
    if not match:
        raise RuntimeError(f"cannot infer T4/T5 candidate from {name}")
    return match.group(1)


def interpolate(samples, stamps):
    if not samples:
        return np.full(len(stamps), np.nan, dtype=float)
    sample_t = np.asarray([item[0] for item in samples], dtype=float)
    sample_v = np.asarray([item[1] for item in samples], dtype=float)
    query = np.asarray(stamps, dtype=float)
    output = np.interp(query, sample_t, sample_v)
    output[(query < sample_t[0]) | (query > sample_t[-1])] = np.nan
    return output


def find_rgb_csv(root, bag_stem):
    target = f"{bag_stem}_red_top.csv"
    matches = list(root.rglob(target))
    if len(matches) != 1:
        raise RuntimeError(f"expected one {target} below {root}, found {len(matches)}")
    return matches[0]


def read_rgb_csv(path):
    rows = []
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"frame_index", "stamp_sec", RGB_HEIGHT_COL, RGB_RAW_MAX_COL, *RGB_LCR_COLS}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f"{path}: missing columns {sorted(missing)}")
        for row in reader:
            height = finite_float(row.get(RGB_HEIGHT_COL))
            if not math.isfinite(height):
                continue
            lcr = [finite_float(row.get(key)) for key in RGB_LCR_COLS]
            rows.append({
                "frame_index": int(row["frame_index"]),
                "stamp": finite_float(row["stamp_sec"]),
                "height": abs(height),
                "height_signed": height,
                "raw_max": finite_float(row.get(RGB_RAW_MAX_COL)),
                "left": lcr[0],
                "center": lcr[1],
                "right": lcr[2],
                "spread": max(lcr) - min(lcr) if all(math.isfinite(v) for v in lcr) else math.nan,
                "conf": finite_float(row.get("conf_mean")),
                "upper_clipped": int(any(math.isfinite(v) and v >= 7.999 for v in lcr)),
            })
    if not rows:
        raise RuntimeError(f"{path}: no finite {RGB_HEIGHT_COL} samples")
    return rows


def rolling_median(values, window):
    values = np.asarray(values, dtype=float)
    if window <= 1:
        return values.copy()
    half = window // 2
    output = np.full(values.shape, np.nan, dtype=float)
    for index in range(values.size):
        segment = values[max(0, index - half):index + half + 1]
        segment = segment[np.isfinite(segment)]
        if segment.size:
            output[index] = float(np.median(segment))
    return output


def read_bag(path):
    progress = []
    tracking_error = []
    model = []
    statuses = []
    topics = [
        "/baseline/teb/tracking_error",
        "/baseline/teb/status",
        "/slosh/height",
    ]
    with rosbag.Bag(str(path), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            time_s = stamp.to_sec()
            if topic == "/baseline/teb/tracking_error" and len(msg.data) >= 7:
                ratio = float(msg.data[4])
                progress.append((time_s, ratio))
                tracking_error.append((time_s, abs(float(msg.data[0])), ratio))
            elif topic == "/baseline/teb/status":
                statuses.append((time_s, str(msg.data)))
            elif topic == "/slosh/height":
                model.append((time_s, 1000.0 * abs(float(msg.data))))
    if not progress:
        raise RuntimeError(f"{path}: missing /baseline/teb/tracking_error")
    if not model:
        raise RuntimeError(f"{path}: missing /slosh/height")
    tracking_start = next((t for t, value in statuses if value == "TRACKING"), math.nan)
    goal_reached = next((t for t, value in statuses if value == "GOAL_REACHED"), math.nan)
    return {
        "bag_start": bag_start,
        "bag_end": bag_end,
        "progress": progress,
        "tracking_error": tracking_error,
        "model": model,
        "tracking_start": tracking_start,
        "goal_reached": goal_reached,
    }


def nearest_pair(rgb_times, rgb_values, model, max_gap_s, t_lo, t_hi):
    model_t = np.asarray([item[0] for item in model], dtype=float)
    model_v = np.asarray([item[1] for item in model], dtype=float)
    rgb_t = np.asarray(rgb_times, dtype=float)
    rgb_v = np.asarray(rgb_values, dtype=float)
    window = (rgb_t >= t_lo) & (rgb_t <= t_hi) & np.isfinite(rgb_v)
    rgb_t = rgb_t[window]
    rgb_v = rgb_v[window]
    if rgb_t.size == 0:
        return tuple(np.asarray([], dtype=float) for _ in range(4))
    right = np.clip(np.searchsorted(model_t, rgb_t), 0, model_t.size - 1)
    left = np.clip(right - 1, 0, model_t.size - 1)
    gap_right = np.abs(model_t[right] - rgb_t)
    gap_left = np.abs(model_t[left] - rgb_t)
    use_left = gap_left < gap_right
    best = np.where(use_left, left, right)
    gaps = np.where(use_left, gap_left, gap_right)
    keep = gaps <= max_gap_s
    return rgb_t[keep], rgb_v[keep], model_v[best[keep]], gaps[keep]


def fidelity_metrics(times, visual, model, gaps):
    result = {
        "paired_samples": int(len(visual)),
        "pair_dt_median_ms": percentile(1000.0 * gaps, 50),
        "pair_dt_p95_ms": percentile(1000.0 * gaps, 95),
    }
    if len(visual) < 8:
        for key in (
            "gamma_model_pct", "rmse_mm", "corr", "visual_p95_mm",
            "model_p95_mm", "visual_peak_mm", "model_peak_mm",
            "visual_rms_mm", "model_rms_mm", "e_p95_mm", "e_peak_mm",
            "e_rms_mm", "U_p95_mm", "U_peak_mm", "U_max_mm",
            "under_ratio", "scale_fit_slosh_per_visual",
            "visual_p95_over_model_p95", "visual_peak_over_model_peak",
        ):
            result[key] = math.nan
        return result

    visual = np.asarray(visual, dtype=float)
    model = np.asarray(model, dtype=float)
    times = np.asarray(times, dtype=float)
    diff = model - visual
    denom = float(np.trapz(model, times))
    visual_p95 = percentile(visual, 95)
    model_p95 = percentile(model, 95)
    visual_peak = float(np.max(visual))
    model_peak = float(np.max(model))
    visual_rms = rms(visual)
    model_rms = rms(model)
    under = np.maximum(0.0, visual - model)
    visual_energy = float(np.dot(visual, visual))
    result.update({
        "gamma_model_pct": 100.0 * float(np.trapz(diff, times)) / denom if abs(denom) > 1e-12 else math.nan,
        "rmse_mm": rms(diff),
        "corr": float(np.corrcoef(model, visual)[0, 1]) if np.std(model) > 1e-9 and np.std(visual) > 1e-9 else math.nan,
        "visual_p95_mm": visual_p95,
        "model_p95_mm": model_p95,
        "visual_peak_mm": visual_peak,
        "model_peak_mm": model_peak,
        "visual_rms_mm": visual_rms,
        "model_rms_mm": model_rms,
        "e_p95_mm": model_p95 - visual_p95,
        "e_peak_mm": model_peak - visual_peak,
        "e_rms_mm": model_rms - visual_rms,
        "U_p95_mm": max(0.0, visual_p95 - model_p95),
        "U_peak_mm": max(0.0, visual_peak - model_peak),
        "U_max_mm": float(np.max(under)),
        "under_ratio": float(np.mean(model < visual)),
        "scale_fit_slosh_per_visual": float(np.dot(visual, model) / visual_energy) if visual_energy > 1e-12 else math.nan,
        "visual_p95_over_model_p95": visual_p95 / model_p95 if model_p95 > 1e-12 else math.nan,
        "visual_peak_over_model_peak": visual_peak / model_peak if model_peak > 1e-12 else math.nan,
    })
    return result


def threshold_sweep(candidate, run, visual, model, thresholds):
    rows = []
    for threshold in thresholds:
        truth = visual > threshold
        true_count = int(np.sum(truth))
        detected = int(np.sum(truth & (model > threshold)))
        rows.append({
            "candidate": candidate,
            "run": run,
            "tau_mm": threshold,
            "true_high_count": true_count,
            "model_detect_count": detected,
            "false_safe_count": true_count - detected,
            "recall": detected / true_count if true_count else math.nan,
        })
    return rows


def analyze_run(bag_path, rgb_root, args):
    bag = read_bag(bag_path)
    rgb_csv = find_rgb_csv(rgb_root, bag_path.stem)
    rgb_rows = read_rgb_csv(rgb_csv)
    rgb_times = np.asarray([row["stamp"] for row in rgb_rows], dtype=float)
    rgb_values_first30 = np.asarray([row["height"] for row in rgb_rows], dtype=float)
    rgb_raw_max = np.asarray([row["raw_max"] for row in rgb_rows], dtype=float)
    rgb_h0_first30 = float(np.nanmedian(rgb_raw_max[:30]))
    rgb_raw_smooth = rolling_median(rgb_raw_max, args.smooth_frames)
    if args.rgb_zero_mode == "calibration_zero":
        rgb_h0 = 0.0
        rgb_values = np.abs(rgb_raw_smooth)
    else:
        rgb_h0 = rgb_h0_first30
        rgb_values = np.abs(rgb_raw_smooth - rgb_h0)
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
    core_values_first30 = rgb_values_first30[core_mask]
    peak_local = int(np.argmax(core_values))
    peak_row = rgb_rows[int(core_indices[peak_local])]

    model_t = np.asarray([item[0] for item in bag["model"]], dtype=float)
    model_v = np.asarray([item[1] for item in bag["model"]], dtype=float)
    model_progress = interpolate(bag["progress"], model_t)
    model_core = model_v[
        np.isfinite(model_progress)
        & (model_progress >= args.progress_lo)
        & (model_progress <= args.progress_hi)
    ]
    tracking_core = [
        error for _, error, ratio in bag["tracking_error"]
        if args.progress_lo <= ratio <= args.progress_hi
    ]
    spreads = np.asarray([row["spread"] for row in rgb_rows], dtype=float)[core_mask]
    confidences = np.asarray([row["conf"] for row in rgb_rows], dtype=float)[core_mask]
    upper_clipped = np.asarray([row["upper_clipped"] for row in rgb_rows], dtype=float)[core_mask]

    core_t, core_vis, core_model, core_gaps = nearest_pair(
        rgb_times, rgb_values, bag["model"], args.pair_max_gap_s,
        float(rgb_times[core_indices[0]]), float(rgb_times[core_indices[-1]]),
    )
    core_pair_progress = interpolate(bag["progress"], core_t)
    pair_keep = (
        np.isfinite(core_pair_progress)
        & (core_pair_progress >= args.progress_lo)
        & (core_pair_progress <= args.progress_hi)
    )
    core_fidelity = fidelity_metrics(
        core_t[pair_keep], core_vis[pair_keep], core_model[pair_keep], core_gaps[pair_keep]
    )

    if not math.isfinite(bag["tracking_start"]) or not math.isfinite(bag["goal_reached"]):
        raise RuntimeError(f"{bag_path}: missing TRACKING or GOAL_REACHED status")
    fidelity_end = min(bag["bag_end"], bag["goal_reached"] + args.fidelity_tail_s)
    fid_t, fid_vis, fid_model, fid_gaps = nearest_pair(
        rgb_times, rgb_values, bag["model"], args.pair_max_gap_s,
        bag["tracking_start"], fidelity_end,
    )
    fidelity = fidelity_metrics(fid_t, fid_vis, fid_model, fid_gaps)
    candidate = candidate_from_name(bag_path.name)
    threshold_rows = threshold_sweep(candidate, bag_path.stem, fid_vis, fid_model, args.thresholds)

    row = {
        "candidate": candidate,
        "run": bag_path.stem,
        "bag": str(bag_path),
        "rgb_csv": str(rgb_csv),
        "success": 1,
        "goal_time_s": bag["goal_reached"] - bag["tracking_start"],
        "progress_max": max(value for _, value in bag["progress"]),
        "rgb_samples_10_90": int(core_values.size),
        "rgb_zero_mode": args.rgb_zero_mode,
        "rgb_h0_mm": rgb_h0,
        "rgb_h0_first30_mm": rgb_h0_first30,
        "rgb_p95_mm_10_90": percentile(core_values, 95),
        "rgb_peak_mm_10_90": float(np.max(core_values)),
        "rgb_rms_mm_10_90": rms(core_values),
        "rgb_mean_mm_10_90": float(np.mean(core_values)),
        "rgb_first30corr_p95_mm_10_90": percentile(core_values_first30, 95),
        "rgb_first30corr_peak_mm_10_90": float(np.max(core_values_first30)),
        "rgb_first30corr_rms_mm_10_90": rms(core_values_first30),
        "rgb_lcr_spread_p95_mm_10_90": percentile(spreads, 95),
        "rgb_lcr_spread_peak_mm_10_90": float(np.nanmax(spreads)),
        "rgb_conf_mean_10_90": float(np.nanmean(confidences)),
        "rgb_conf_p05_10_90": percentile(confidences, 5),
        "rgb_upper_clip_frac_10_90": float(np.mean(upper_clipped)),
        "rgb_peak_frame_10_90": peak_row["frame_index"],
        "rgb_peak_stamp_10_90": peak_row["stamp"],
        "rgb_peak_bag_offset_s_10_90": peak_row["stamp"] - bag["bag_start"],
        "rgb_peak_progress_10_90": float(rgb_progress[core_indices[peak_local]]),
        "tracking_p95_m_10_90": percentile(tracking_core, 95),
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
        "candidate": candidate,
        "run": bag_path.stem,
        "progress": rgb_progress[core_mask],
        "visual": core_values,
        "paired_visual": fid_vis,
        "paired_model": fid_model,
    }
    return row, threshold_rows, series


def aggregate(rows):
    metrics = [
        "goal_time_s", "tracking_p95_m_10_90",
        "rgb_p95_mm_10_90", "rgb_peak_mm_10_90", "rgb_rms_mm_10_90",
        "model_p95_mm_10_90", "model_peak_mm_10_90", "model_rms_mm_10_90",
        "fidelity_gamma_model_pct", "fidelity_rmse_mm", "fidelity_corr",
        "fidelity_U_p95_mm", "fidelity_U_max_mm", "fidelity_under_ratio",
        "fidelity_pair_dt_p95_ms", "fidelity_scale_fit_slosh_per_visual",
    ]
    output = []
    for candidate in sorted({row["candidate"] for row in rows}):
        group = [row for row in rows if row["candidate"] == candidate]
        item = {"candidate": candidate, "run_count": len(group), "success_count": sum(r["success"] for r in group)}
        for metric in metrics:
            values = [float(row[metric]) for row in group if math.isfinite(float(row[metric]))]
            item[f"{metric}_mean"] = statistics.fmean(values) if values else math.nan
            item[f"{metric}_std"] = statistics.stdev(values) if len(values) > 1 else 0.0 if values else math.nan
        output.append(item)
    return output


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_plots(plot_dir, series, aggregate_rows):
    if plt is None:
        return
    plot_dir.mkdir(parents=True, exist_ok=True)
    colors = {"T4": "#1f77b4", "T5": "#d62728"}

    fig, ax = plt.subplots(figsize=(11, 5.5), dpi=180)
    for item in series:
        ax.plot(item["progress"], item["visual"], color=colors[item["candidate"]], alpha=0.45, linewidth=1.0, label=item["run"])
    ax.set_xlabel("fixed-path progress")
    ax.set_ylabel("H_vis = abs(max-LCR smooth corr) [mm]")
    ax.set_xlim(0.10, 0.90)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(plot_dir / "rgb_progress_10_90.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5.2), dpi=180)
    metrics = ["rgb_p95_mm_10_90", "rgb_peak_mm_10_90", "rgb_rms_mm_10_90"]
    labels = ["p95", "peak", "RMS"]
    x = np.arange(len(metrics), dtype=float)
    width = 0.34
    for index, candidate in enumerate(("T4", "T5")):
        item = next(row for row in aggregate_rows if row["candidate"] == candidate)
        means = [item[f"{metric}_mean"] for metric in metrics]
        stds = [item[f"{metric}_std"] for metric in metrics]
        ax.bar(x + (index - 0.5) * width, means, width, yerr=stds, capsize=4, color=colors[candidate], alpha=0.82, label=candidate)
    ax.set_xticks(x, labels)
    ax.set_ylabel("RGB H_vis [mm], mean +/- sample std")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_dir / "rgb_aggregate_metrics.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.3, 6.0), dpi=180)
    for item in series:
        ax.scatter(item["paired_visual"], item["paired_model"], s=5, alpha=0.12, color=colors[item["candidate"]], label=item["candidate"])
    limits = ax.get_xlim()[1], ax.get_ylim()[1]
    upper = max(limits)
    ax.plot([0, upper], [0, upper], color="#333333", linestyle="--", linewidth=1.0)
    ax.set_xlim(0, upper)
    ax.set_ylim(0, upper)
    ax.set_xlabel("RGB H_vis [mm]")
    ax.set_ylabel("/slosh/height [mm]")
    ax.grid(True, alpha=0.25)
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())
    fig.tight_layout()
    fig.savefig(plot_dir / "rgb_model_fidelity_scatter.png")
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--red-infer-dir", type=Path, required=True)
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
            f"{row['candidate']} {row['run']}: "
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
