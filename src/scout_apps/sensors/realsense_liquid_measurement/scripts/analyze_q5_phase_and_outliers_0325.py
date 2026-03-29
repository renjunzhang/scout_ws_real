#!/usr/bin/env python3
"""Analyze Q5_test1 phase lag and outlier segments against /slosh/height, using Q5_test2 as reference."""

import argparse
import bisect
import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DEFAULT_VERIFY_ROOT = Path("/data/a/realsense_validation_v2/verify/0325")
DEFAULT_BAG_ROOT = Path("/data/a/slosh_bags/0325")
DEFAULT_LABELS = ("Q5_test1", "Q5_test2")

BAG_PATTERNS = {
    "Q5_test1": ("qq5", "test1"),
    "Q5_test2": ("qq5", "test2"),
}


@dataclass
class MatchedRow:
    frame_index: int
    stamp: float
    relative_time_s: float
    center_mm: float
    slosh_mm: float
    raw_error_mm: float
    meniscus_confidence_v2: float
    central_coverage_v2: float
    fit_rms_px_v2: float
    peak_candidate_source_v2: str
    height_mapping_segment_id_v2: str


@dataclass
class PhaseSummary:
    label: str
    bag_path: str
    reportable_frames: int
    zero_lag_mae_mm: float
    zero_lag_rmse_mm: float
    zero_lag_corr: float
    zero_lag_bias_mean_mm: float
    zero_lag_bias_median_mm: float
    best_corr_lag_s: float
    best_corr: float
    best_corr_mae_mm: float
    best_corr_rmse_mm: float
    near_zero_best_corr_lag_s: float
    near_zero_best_corr: float
    near_zero_best_corr_mae_mm: float
    near_zero_best_corr_rmse_mm: float
    best_rmse_lag_s: float
    best_rmse: float
    best_rmse_corr: float
    best_rmse_mae_mm: float


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-root", default=str(DEFAULT_VERIFY_ROOT))
    parser.add_argument("--bag-root", default=str(DEFAULT_BAG_ROOT))
    parser.add_argument("--out-dir", required=True)
    parser.add_argument(
        "--labels",
        default=",".join(DEFAULT_LABELS),
        help="Comma-separated bag labels to analyze. Default: Q5_test1,Q5_test2",
    )
    parser.add_argument(
        "--center-column",
        default="height_center_rel_mm_bias_corrected_v2",
        help="RealSense center column.",
    )
    parser.add_argument(
        "--lag-min-s",
        type=float,
        default=-1.20,
        help="Minimum lag for lag sweep. Positive lag means compare center(t) vs slosh(t + lag).",
    )
    parser.add_argument("--lag-max-s", type=float, default=1.20, help="Maximum lag for lag sweep.")
    parser.add_argument("--lag-step-s", type=float, default=0.01, help="Lag sweep step.")
    parser.add_argument(
        "--near-zero-max-lag-s",
        type=float,
        default=0.30,
        help="Lag window used to report physically-plausible near-zero best lag.",
    )
    parser.add_argument(
        "--outlier-threshold-mm",
        type=float,
        default=0.40,
        help="Base absolute error threshold for segment extraction.",
    )
    parser.add_argument(
        "--segment-frame-gap",
        type=int,
        default=15,
        help="Maximum frame gap used to merge adjacent outliers into one segment.",
    )
    parser.add_argument(
        "--segment-time-gap-s",
        type=float,
        default=0.80,
        help="Maximum time gap used to merge adjacent outliers into one segment.",
    )
    parser.add_argument(
        "--top-segments",
        type=int,
        default=6,
        help="Number of top Q5_test1 outlier segments to export.",
    )
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def finite_float(raw_value) -> float:
    if raw_value is None:
        return math.nan
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def mean_or_nan(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(statistics.mean(clean))


def median_or_nan(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(statistics.median(clean))


def corr(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 2:
        return math.nan
    xs = [x for x, _ in pairs]
    ys = [y for _, y in pairs]
    mean_x = statistics.mean(xs)
    mean_y = statistics.mean(ys)
    den_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs))
    den_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys))
    if den_x <= 0.0 or den_y <= 0.0:
        return math.nan
    num = sum((x - mean_x) * (y - mean_y) for x, y in pairs)
    return num / (den_x * den_y)


def mae(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if not pairs:
        return math.nan
    return sum(abs(x - y) for x, y in pairs) / len(pairs)


def rmse(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if not pairs:
        return math.nan
    return math.sqrt(sum((x - y) ** 2 for x, y in pairs) / len(pairs))


def percentile(values: Sequence[float], q: float) -> float:
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return math.nan
    idx = int(round((len(clean) - 1) * q))
    return clean[idx]


def find_bag_path(label: str, bag_root: Path) -> Path:
    tokens = BAG_PATTERNS.get(label)
    if tokens is None:
        raise RuntimeError(f"no bag pattern for {label}")
    matches = []
    for bag_path in sorted(bag_root.glob("*.bag")):
        bag_name = bag_path.name.lower()
        if all(token in bag_name for token in tokens):
            matches.append(bag_path)
    if not matches:
        raise RuntimeError(f"no bag matched {label} under {bag_root}")
    if len(matches) > 1:
        raise RuntimeError(f"multiple bags matched {label}: {matches}")
    return matches[0]


def load_slosh_series(bag_path: Path) -> Tuple[List[float], List[float]]:
    times: List[float] = []
    values: List[float] = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, t in bag.read_messages(topics=["/slosh/height"]):
            times.append(t.to_sec())
            values.append(float(msg.data) * 1000.0)
    if not times:
        raise RuntimeError(f"/slosh/height missing in {bag_path}")
    return times, values


def interpolate_scalar(series_times: Sequence[float], series_values: Sequence[float], ts: float) -> float:
    if not series_times:
        return math.nan
    idx = bisect.bisect_left(series_times, ts)
    if idx <= 0:
        return float(series_values[0])
    if idx >= len(series_times):
        return float(series_values[-1])
    t0 = float(series_times[idx - 1])
    t1 = float(series_times[idx])
    v0 = float(series_values[idx - 1])
    v1 = float(series_values[idx])
    if t1 <= t0:
        return v1
    ratio = (ts - t0) / (t1 - t0)
    return v0 + ratio * (v1 - v0)


def load_liquid_rows(path: Path, center_column: str) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            rows.append(
                {
                    "frame_index": int(raw["frame_index"]),
                    "stamp": float(raw["stamp"]),
                    "relative_time_s": float(raw["relative_time_s"]),
                    "accept_for_peak_report_v2": int(raw.get("accept_for_peak_report_v2", "0") or "0"),
                    "center_mm": finite_float(raw.get(center_column)),
                    "meniscus_confidence_v2": finite_float(raw.get("meniscus_confidence_v2")),
                    "central_coverage_v2": finite_float(raw.get("central_coverage_v2")),
                    "fit_rms_px_v2": finite_float(raw.get("fit_rms_px_v2")),
                    "peak_candidate_source_v2": raw.get("peak_candidate_source_v2", ""),
                    "height_mapping_segment_id_v2": raw.get("height_mapping_segment_id_v2", ""),
                }
            )
    return rows


def build_matches(liquid_rows: Sequence[Dict[str, object]], slosh_times: Sequence[float], slosh_values: Sequence[float]) -> List[MatchedRow]:
    matches: List[MatchedRow] = []
    series_t0 = float(slosh_times[0])
    series_t1 = float(slosh_times[-1])
    for row in liquid_rows:
        if int(row["accept_for_peak_report_v2"]) != 1:
            continue
        center_mm = float(row["center_mm"])
        stamp = float(row["stamp"])
        if not math.isfinite(center_mm) or stamp < series_t0 or stamp > series_t1:
            continue
        slosh_mm = interpolate_scalar(slosh_times, slosh_values, stamp)
        matches.append(
            MatchedRow(
                frame_index=int(row["frame_index"]),
                stamp=stamp,
                relative_time_s=float(row["relative_time_s"]),
                center_mm=center_mm,
                slosh_mm=slosh_mm,
                raw_error_mm=center_mm - slosh_mm,
                meniscus_confidence_v2=float(row["meniscus_confidence_v2"]),
                central_coverage_v2=float(row["central_coverage_v2"]),
                fit_rms_px_v2=float(row["fit_rms_px_v2"]),
                peak_candidate_source_v2=str(row["peak_candidate_source_v2"]),
                height_mapping_segment_id_v2=str(row["height_mapping_segment_id_v2"]),
            )
        )
    return matches


def metrics_from_values(center_values: Sequence[float], slosh_values: Sequence[float]) -> Dict[str, float]:
    error_values = [c - s for c, s in zip(center_values, slosh_values) if math.isfinite(c) and math.isfinite(s)]
    return {
        "mae_mm": mae(center_values, slosh_values),
        "rmse_mm": rmse(center_values, slosh_values),
        "corr": corr(center_values, slosh_values),
        "bias_mean_mm": mean_or_nan(error_values),
        "bias_median_mm": median_or_nan(error_values),
    }


def compute_lag_metrics(matches: Sequence[MatchedRow], slosh_times: Sequence[float], slosh_values: Sequence[float], lag_s: float) -> Dict[str, float]:
    shifted_center: List[float] = []
    shifted_slosh: List[float] = []
    series_t0 = float(slosh_times[0])
    series_t1 = float(slosh_times[-1])
    for row in matches:
        ts = row.stamp + lag_s
        if ts < series_t0 or ts > series_t1:
            continue
        shifted_center.append(row.center_mm)
        shifted_slosh.append(interpolate_scalar(slosh_times, slosh_values, ts))
    metrics = metrics_from_values(shifted_center, shifted_slosh)
    metrics["n"] = len(shifted_center)
    metrics["lag_s"] = lag_s
    return metrics


def lag_sweep(matches: Sequence[MatchedRow], slosh_times: Sequence[float], slosh_values: Sequence[float], lag_values: Sequence[float]) -> List[Dict[str, float]]:
    rows = [compute_lag_metrics(matches, slosh_times, slosh_values, float(lag_s)) for lag_s in lag_values]
    return [row for row in rows if row["n"] >= 8 and math.isfinite(row["corr"])]


def plot_lag_sweep(out_path: Path, label: str, lag_rows: Sequence[Dict[str, float]]):
    ensure_dir(out_path.parent)
    lags_ms = [1000.0 * float(row["lag_s"]) for row in lag_rows]
    corr_values = [float(row["corr"]) for row in lag_rows]
    mae_values = [float(row["mae_mm"]) for row in lag_rows]
    rmse_values = [float(row["rmse_mm"]) for row in lag_rows]

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    axes[0].plot(lags_ms, corr_values, color="#d62728", linewidth=1.5)
    axes[0].set_ylabel("corr")
    axes[0].set_title(f"{label}: lag sweep of RealSense center vs /slosh/height")
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(lags_ms, mae_values, color="#1f77b4", linewidth=1.5)
    axes[1].set_ylabel("MAE [mm]")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(lags_ms, rmse_values, color="#2ca02c", linewidth=1.5)
    axes[2].axvline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    axes[2].set_ylabel("RMSE [mm]")
    axes[2].set_xlabel("lag [ms], compare center(t) vs slosh(t + lag)")
    axes[2].grid(True, alpha=0.25)

    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def plot_overlay(out_path: Path, label: str, matches: Sequence[MatchedRow], best_corr_lag_s: float, slosh_times: Sequence[float], slosh_values: Sequence[float]):
    ensure_dir(out_path.parent)
    rel_t = [row.relative_time_s for row in matches]
    center = [row.center_mm for row in matches]
    slosh_zero = [row.slosh_mm for row in matches]
    slosh_shift = [interpolate_scalar(slosh_times, slosh_values, row.stamp + best_corr_lag_s) for row in matches]

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True, constrained_layout=True)
    axes[0].plot(rel_t, center, color="#d62728", linewidth=1.5, label="RealSense center")
    axes[0].plot(rel_t, slosh_zero, color="#1f77b4", linewidth=1.1, alpha=0.8, label="/slosh/height (lag=0)")
    axes[0].plot(rel_t, slosh_shift, color="#2ca02c", linewidth=1.1, alpha=0.8, label=f"/slosh/height shifted by {best_corr_lag_s*1000:.0f} ms")
    axes[0].set_ylabel("height [mm]")
    axes[0].set_title(f"{label}: zero-lag vs near-zero best-corr-lag overlay")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")

    err_zero = [c - s for c, s in zip(center, slosh_zero)]
    err_shift = [c - s for c, s in zip(center, slosh_shift)]
    axes[1].plot(rel_t, err_zero, color="#9467bd", linewidth=1.1, label="error at lag=0")
    axes[1].plot(rel_t, err_shift, color="#ff7f0e", linewidth=1.1, label="error at near-zero best corr lag")
    axes[1].axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    axes[1].set_ylabel("error [mm]")
    axes[1].set_xlabel("relative time [s]")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="upper right")

    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def extract_outlier_segments(
    matches: Sequence[MatchedRow],
    base_threshold_mm: float,
    frame_gap: int,
    time_gap_s: float,
) -> Tuple[List[List[MatchedRow]], float]:
    abs_errors = [abs(row.raw_error_mm) for row in matches]
    dynamic_threshold = percentile(abs_errors, 0.95)
    threshold = max(base_threshold_mm, dynamic_threshold if math.isfinite(dynamic_threshold) else 0.0)
    flagged = [row for row in matches if abs(row.raw_error_mm) >= threshold]
    if not flagged:
        return [], threshold

    segments: List[List[MatchedRow]] = []
    current: List[MatchedRow] = [flagged[0]]
    for row in flagged[1:]:
        prev = current[-1]
        if (row.frame_index - prev.frame_index) <= frame_gap or (row.relative_time_s - prev.relative_time_s) <= time_gap_s:
            current.append(row)
        else:
            segments.append(current)
            current = [row]
    segments.append(current)
    return segments, threshold


def summarize_segment(segment: Sequence[MatchedRow], rank: int, slosh_times: Sequence[float], slosh_values: Sequence[float], lag_values: Sequence[float]) -> Dict[str, object]:
    center = [row.center_mm for row in segment]
    slosh = [row.slosh_mm for row in segment]
    errors = [row.raw_error_mm for row in segment]
    lag_rows = lag_sweep(segment, slosh_times, slosh_values, lag_values)
    best_corr_row = max(lag_rows, key=lambda row: row["corr"]) if lag_rows else None
    return {
        "rank": rank,
        "start_frame_index": segment[0].frame_index,
        "end_frame_index": segment[-1].frame_index,
        "start_time_s": f"{segment[0].relative_time_s:.6f}",
        "end_time_s": f"{segment[-1].relative_time_s:.6f}",
        "num_flagged_rows": len(segment),
        "raw_error_abs_max_mm": f"{max(abs(v) for v in errors):.6f}",
        "raw_error_mean_mm": f"{mean_or_nan(errors):.6f}",
        "raw_error_median_mm": f"{median_or_nan(errors):.6f}",
        "center_range_mm": f"{(max(center) - min(center)):.6f}",
        "slosh_range_mm": f"{(max(slosh) - min(slosh)):.6f}",
        "mean_confidence": f"{mean_or_nan(row.meniscus_confidence_v2 for row in segment):.6f}",
        "mean_coverage": f"{mean_or_nan(row.central_coverage_v2 for row in segment):.6f}",
        "mean_fit_rms_px": f"{mean_or_nan(row.fit_rms_px_v2 for row in segment):.6f}",
        "dominant_peak_source_v2": statistics.mode([row.peak_candidate_source_v2 for row in segment]) if segment else "",
        "best_local_corr_lag_ms": f"{1000.0 * float(best_corr_row['lag_s']):.1f}" if best_corr_row else "",
        "best_local_corr": f"{float(best_corr_row['corr']):.6f}" if best_corr_row else "",
    }


def plot_segment(
    out_path: Path,
    label: str,
    rank: int,
    segment: Sequence[MatchedRow],
    best_corr_lag_s: float,
    slosh_times: Sequence[float],
    slosh_values: Sequence[float],
):
    ensure_dir(out_path.parent)
    pad = 0.75
    t0 = segment[0].relative_time_s - pad
    t1 = segment[-1].relative_time_s + pad
    window_rows = [row for row in segment]
    rel_t = [row.relative_time_s for row in window_rows]
    center = [row.center_mm for row in window_rows]
    slosh_zero = [row.slosh_mm for row in window_rows]
    slosh_shift = [interpolate_scalar(slosh_times, slosh_values, row.stamp + best_corr_lag_s) for row in window_rows]
    error_zero = [c - s for c, s in zip(center, slosh_zero)]
    error_shift = [c - s for c, s in zip(center, slosh_shift)]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    axes[0].plot(rel_t, center, color="#d62728", linewidth=1.6, label="RealSense center")
    axes[0].plot(rel_t, slosh_zero, color="#1f77b4", linewidth=1.2, label="/slosh/height (lag=0)")
    axes[0].plot(rel_t, slosh_shift, color="#2ca02c", linewidth=1.1, label=f"/slosh/height shifted {best_corr_lag_s*1000:.0f} ms")
    axes[0].set_title(f"{label} segment #{rank}: frames {segment[0].frame_index}-{segment[-1].frame_index}")
    axes[0].set_ylabel("height [mm]")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")

    axes[1].plot(rel_t, error_zero, color="#9467bd", linewidth=1.2, label="error at lag=0")
    axes[1].plot(rel_t, error_shift, color="#ff7f0e", linewidth=1.2, label="error at best global lag")
    axes[1].axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    axes[1].set_xlim(t0, t1)
    axes[1].set_xlabel("relative time [s]")
    axes[1].set_ylabel("error [mm]")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="upper right")
    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def build_readme(
    path: Path,
    summaries: Sequence[PhaseSummary],
    segment_rows: Sequence[Dict[str, object]],
    threshold_mm: float,
):
    lines = [
        "# Q5 相位差与坏点深挖",
        "",
        "本目录重点分析 `Q5_test1` 和 `Q5_test2` 两包中 `RealSense center` 对 `/slosh/height` 的相位差和坏点分段。",
        "",
        "输出内容：",
        "- `phase_summary.csv`: 每包在 `lag=0`、近零最优 lag、全局最优 lag 下的 MAE/RMSE/corr",
        "- `phase_summary.json`: 同上，JSON 版本",
        "- `<bag>_lag_sweep.png`: lag sweep 曲线",
        "- `<bag>_lag_overlay.png`: `lag=0` 与近零最优 lag 的同图比较",
        "- `Q5_test1_outlier_segments.csv`: `Q5_test1` 的大误差分段",
        "- `Q5_test1_segments/*.png`: 最差分段局部曲线",
        "",
        f"当前坏点分段阈值：`abs(center - /slosh/height) >= {threshold_mm:.3f} mm`",
        "",
        "结论速览：",
    ]
    for row in summaries:
        lines.append(
            f"- `{row.label}`: lag=0 -> MAE={row.zero_lag_mae_mm:.3f} mm, corr={row.zero_lag_corr:.3f}; "
            f"near-zero best lag={row.near_zero_best_corr_lag_s*1000:.0f} ms, corr={row.near_zero_best_corr:.3f}, "
            f"RMSE={row.near_zero_best_corr_rmse_mm:.3f} mm; "
            f"global best-corr lag={row.best_corr_lag_s*1000:.0f} ms"
        )
    if segment_rows:
        lines.extend(
            [
                "",
                "Q5_test1 重点分段：",
            ]
        )
        for row in segment_rows[: min(5, len(segment_rows))]:
            lines.append(
                f"- `#{row['rank']}` frames {row['start_frame_index']}-{row['end_frame_index']}: "
                f"abs_max={row['raw_error_abs_max_mm']} mm, mean_cov={row['mean_coverage']}, "
                f"local_best_lag={row['best_local_corr_lag_ms']} ms"
            )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    verify_root = Path(args.verify_root).expanduser().resolve()
    bag_root = Path(args.bag_root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    ensure_dir(out_dir)

    labels = [token.strip() for token in args.labels.split(",") if token.strip()]
    lag_values = np.arange(args.lag_min_s, args.lag_max_s + 0.5 * args.lag_step_s, args.lag_step_s, dtype=float)
    near_zero_lag_values = [float(v) for v in lag_values if abs(float(v)) <= args.near_zero_max_lag_s + 1e-9]

    all_summaries: List[PhaseSummary] = []
    q5_test1_matches: List[MatchedRow] = []
    q5_test1_slosh_times: List[float] = []
    q5_test1_slosh_values: List[float] = []

    for label in labels:
        liquid_csv = verify_root / label / "liquid_height_v2.csv"
        if not liquid_csv.is_file():
            raise SystemExit(f"[ERROR] missing {liquid_csv}")
        bag_path = find_bag_path(label, bag_root)
        slosh_times, slosh_values = load_slosh_series(bag_path)
        liquid_rows = load_liquid_rows(liquid_csv, args.center_column)
        matches = build_matches(liquid_rows, slosh_times, slosh_values)
        if not matches:
            raise SystemExit(f"[ERROR] no reportable rows for {label}")

        zero_metrics = metrics_from_values([row.center_mm for row in matches], [row.slosh_mm for row in matches])
        lag_rows = lag_sweep(matches, slosh_times, slosh_values, lag_values)
        if not lag_rows:
            raise SystemExit(f"[ERROR] lag sweep failed for {label}")
        near_zero_rows = [row for row in lag_rows if abs(float(row["lag_s"])) <= args.near_zero_max_lag_s + 1e-9]
        if not near_zero_rows:
            raise SystemExit(f"[ERROR] near-zero lag sweep failed for {label}")
        best_corr_row = max(lag_rows, key=lambda row: row["corr"])
        near_zero_best_corr_row = max(near_zero_rows, key=lambda row: row["corr"])
        best_rmse_row = min(lag_rows, key=lambda row: row["rmse_mm"])

        all_summaries.append(
            PhaseSummary(
                label=label,
                bag_path=str(bag_path),
                reportable_frames=len(matches),
                zero_lag_mae_mm=float(zero_metrics["mae_mm"]),
                zero_lag_rmse_mm=float(zero_metrics["rmse_mm"]),
                zero_lag_corr=float(zero_metrics["corr"]),
                zero_lag_bias_mean_mm=float(zero_metrics["bias_mean_mm"]),
                zero_lag_bias_median_mm=float(zero_metrics["bias_median_mm"]),
                best_corr_lag_s=float(best_corr_row["lag_s"]),
                best_corr=float(best_corr_row["corr"]),
                best_corr_mae_mm=float(best_corr_row["mae_mm"]),
                best_corr_rmse_mm=float(best_corr_row["rmse_mm"]),
                near_zero_best_corr_lag_s=float(near_zero_best_corr_row["lag_s"]),
                near_zero_best_corr=float(near_zero_best_corr_row["corr"]),
                near_zero_best_corr_mae_mm=float(near_zero_best_corr_row["mae_mm"]),
                near_zero_best_corr_rmse_mm=float(near_zero_best_corr_row["rmse_mm"]),
                best_rmse_lag_s=float(best_rmse_row["lag_s"]),
                best_rmse=float(best_rmse_row["rmse_mm"]),
                best_rmse_corr=float(best_rmse_row["corr"]),
                best_rmse_mae_mm=float(best_rmse_row["mae_mm"]),
            )
        )

        plot_lag_sweep(out_dir / f"{label}_lag_sweep.png", label, lag_rows)
        plot_overlay(out_dir / f"{label}_lag_overlay.png", label, matches, float(near_zero_best_corr_row["lag_s"]), slosh_times, slosh_values)

        if label == "Q5_test1":
            q5_test1_matches = matches
            q5_test1_slosh_times = list(slosh_times)
            q5_test1_slosh_values = list(slosh_values)

    if not q5_test1_matches:
        raise SystemExit("[ERROR] Q5_test1 is required for outlier analysis")

    segments, threshold_mm = extract_outlier_segments(
        q5_test1_matches,
        base_threshold_mm=args.outlier_threshold_mm,
        frame_gap=args.segment_frame_gap,
        time_gap_s=args.segment_time_gap_s,
    )
    ranked_segments = sorted(segments, key=lambda seg: max(abs(row.raw_error_mm) for row in seg), reverse=True)
    top_segments = ranked_segments[: args.top_segments]
    segment_rows = [
        summarize_segment(segment, rank + 1, q5_test1_slosh_times, q5_test1_slosh_values, near_zero_lag_values)
        for rank, segment in enumerate(top_segments)
    ]

    q5_test1_best_corr_lag = next(row.near_zero_best_corr_lag_s for row in all_summaries if row.label == "Q5_test1")
    segment_dir = out_dir / "Q5_test1_segments"
    ensure_dir(segment_dir)
    for rank, segment in enumerate(top_segments, start=1):
        plot_segment(
            segment_dir / f"segment_{rank:02d}_frames_{segment[0].frame_index}_{segment[-1].frame_index}.png",
            "Q5_test1",
            rank,
            segment,
            q5_test1_best_corr_lag,
            q5_test1_slosh_times,
            q5_test1_slosh_values,
        )

    summary_csv = out_dir / "phase_summary.csv"
    with summary_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(PhaseSummary.__annotations__.keys()))
        writer.writeheader()
        for row in all_summaries:
            writer.writerow(row.__dict__)

    segment_csv = out_dir / "Q5_test1_outlier_segments.csv"
    segment_fields = list(segment_rows[0].keys()) if segment_rows else [
        "rank",
        "start_frame_index",
        "end_frame_index",
        "start_time_s",
        "end_time_s",
        "num_flagged_rows",
        "raw_error_abs_max_mm",
        "raw_error_mean_mm",
        "raw_error_median_mm",
        "center_range_mm",
        "slosh_range_mm",
        "mean_confidence",
        "mean_coverage",
        "mean_fit_rms_px",
        "dominant_peak_source_v2",
        "best_local_corr_lag_ms",
        "best_local_corr",
    ]
    with segment_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=segment_fields)
        writer.writeheader()
        writer.writerows(segment_rows)

    summary_json = out_dir / "phase_summary.json"
    with summary_json.open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "verify_root": str(verify_root),
                "bag_root": str(bag_root),
                "labels": labels,
                "center_column": args.center_column,
                "lag_range_s": [args.lag_min_s, args.lag_max_s],
                "lag_step_s": args.lag_step_s,
                "near_zero_max_lag_s": args.near_zero_max_lag_s,
                "outlier_threshold_mm": threshold_mm,
                "phase_summaries": [row.__dict__ for row in all_summaries],
                "q5_test1_top_segments": segment_rows,
            },
            handle,
            ensure_ascii=False,
            indent=2,
        )

    build_readme(out_dir / "README.md", all_summaries, segment_rows, threshold_mm)

    print(f"[OK] phase summary csv: {summary_csv}")
    print(f"[OK] segment csv: {segment_csv}")
    print(f"[OK] phase summary json: {summary_json}")
    print(f"[OK] readme: {out_dir / 'README.md'}")
    for row in all_summaries:
        print(
            f"[INFO] {row.label}: lag0_MAE={row.zero_lag_mae_mm:.6f} mm, lag0_corr={row.zero_lag_corr:.6f}, "
            f"near0_best_corr_lag={row.near_zero_best_corr_lag_s*1000:.1f} ms, "
            f"near0_best_corr={row.near_zero_best_corr:.6f}, near0_best_corr_RMSE={row.near_zero_best_corr_rmse_mm:.6f} mm, "
            f"global_best_corr_lag={row.best_corr_lag_s*1000:.1f} ms"
        )
    print(f"[INFO] Q5_test1 outlier threshold: {threshold_mm:.6f} mm, segments={len(segment_rows)}")


if __name__ == "__main__":
    main()
