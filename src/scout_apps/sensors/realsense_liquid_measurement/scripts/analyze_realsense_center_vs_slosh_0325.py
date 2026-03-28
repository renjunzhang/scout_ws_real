#!/usr/bin/env python3
"""Analyze per-bag RealSense center vs /slosh/height error, bias, and outliers for 0325."""

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

BAG_PATTERNS = {
    "Q0_static": ("qq0", "static"),
    "Q5_static": ("qq5", "static"),
    "Q0_test1": ("qq0", "test1"),
    "Q5_test1": ("qq5", "test1"),
    "Q5_test2": ("qq5", "test2"),
    "Q5_test3": ("qq5", "test3"),
}


@dataclass
class SummaryRow:
    label: str
    bag_path: str
    total_frames: int
    reportable_frames: int
    reportable_ratio: float
    raw_bias_mean_mm: float
    raw_bias_median_mm: float
    raw_mae_mm: float
    raw_rmse_mm: float
    raw_corr: float
    raw_abs_p90_mm: float
    raw_abs_max_mm: float
    zero_mae_mm: float
    zero_rmse_mm: float
    zero_corr: float
    zero_abs_p90_mm: float
    zero_abs_max_mm: float
    mean_confidence: float
    mean_coverage: float
    mean_fit_rms_px: float


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-root", default=str(DEFAULT_VERIFY_ROOT))
    parser.add_argument("--bag-root", default=str(DEFAULT_BAG_ROOT))
    parser.add_argument("--out-dir", required=True, help="Output directory for summary tables, plots, and README.")
    parser.add_argument(
        "--center-column",
        default="height_center_rel_mm_bias_corrected_v2",
        help="RealSense center column used as the main liquid level.",
    )
    parser.add_argument(
        "--top-outliers",
        type=int,
        default=12,
        help="Number of per-bag worst raw-error frames to export.",
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


def percentile90(values: Sequence[float]) -> float:
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return math.nan
    idx = int(round((len(clean) - 1) * 0.9))
    return clean[idx]


def paired_values(a: Sequence[float], b: Sequence[float]) -> List[Tuple[float, float]]:
    return [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]


def mae(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = paired_values(a, b)
    if not pairs:
        return math.nan
    return sum(abs(x - y) for x, y in pairs) / len(pairs)


def rmse(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = paired_values(a, b)
    if not pairs:
        return math.nan
    return math.sqrt(sum((x - y) ** 2 for x, y in pairs) / len(pairs))


def corr(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = paired_values(a, b)
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


def mean_or_nan(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(statistics.mean(clean))


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


def match_liquid_row_by_rel_time(rows: Sequence[Dict[str, object]], rel_time: float, tolerance_s: float = 0.03) -> Optional[Dict[str, object]]:
    best_row = None
    best_dt = None
    for row in rows:
        if row.get("accept_for_peak_report_v2") != 1:
            continue
        dt = abs(float(row["relative_time_s"]) - rel_time)
        if best_dt is None or dt < best_dt:
            best_dt = dt
            best_row = row
    if best_dt is None or best_dt > tolerance_s:
        return None
    return best_row


def suspected_cause(row: Dict[str, object]) -> str:
    causes: List[str] = []
    confidence = float(row.get("meniscus_confidence_v2", math.nan))
    coverage = float(row.get("central_coverage_v2", math.nan))
    fit_rms = float(row.get("fit_rms_px_v2", math.nan))
    extrapolated = int(row.get("height_mapping_extrapolated_v2", 0))
    if math.isfinite(confidence) and confidence < 0.70:
        causes.append("low_confidence")
    if math.isfinite(coverage) and coverage < 0.30:
        causes.append("low_coverage")
    if math.isfinite(fit_rms) and fit_rms > 5.0:
        causes.append("high_fit_rms")
    if extrapolated == 1:
        causes.append("mapping_extrapolated")
    if not causes:
        causes.append("model_or_bias")
    return ",".join(causes)


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
                    "valid_v2": int(raw.get("valid_v2", "0") or "0"),
                    "accept_for_peak_report_v2": int(raw.get("accept_for_peak_report_v2", "0") or "0"),
                    "center_mm": finite_float(raw.get(center_column)),
                    "meniscus_confidence_v2": finite_float(raw.get("meniscus_confidence_v2")),
                    "central_coverage_v2": finite_float(raw.get("central_coverage_v2")),
                    "fit_rms_px_v2": finite_float(raw.get("fit_rms_px_v2")),
                    "height_mapping_extrapolated_v2": int(raw.get("height_mapping_extrapolated_v2", "0") or "0"),
                    "peak_candidate_source_v2": raw.get("peak_candidate_source_v2", ""),
                    "height_mapping_segment_id_v2": raw.get("height_mapping_segment_id_v2", ""),
                }
            )
    return rows


def load_zero_aligned_rows(path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            rows.append(
                {
                    "relative_time_s": float(raw["relative_time_s"]),
                    "center_zero": finite_float(raw.get("realsense_center_main_rel_mm_v2_zero_aligned")),
                    "slosh_zero": finite_float(raw.get("mpc_slosh_height_mm_zero_aligned")),
                    "error_zero": finite_float(raw.get("error_center_vs_height_mm_zero_aligned")),
                }
            )
    return rows


def plot_bag(out_path: Path, label: str, raw_rows: Sequence[Dict[str, object]], zero_rows: Sequence[Dict[str, float]]):
    ensure_dir(out_path.parent)
    raw_t = [float(row["relative_time_s"]) for row in raw_rows]
    center = [float(row["center_mm"]) for row in raw_rows]
    slosh = [float(row["slosh_mm"]) for row in raw_rows]
    raw_err = [float(row["raw_error_mm"]) for row in raw_rows]
    zero_t = [float(row["relative_time_s"]) for row in zero_rows]
    zero_err = [float(row["error_zero"]) for row in zero_rows if math.isfinite(float(row["error_zero"]))]
    zero_t_err = [float(row["relative_time_s"]) for row in zero_rows if math.isfinite(float(row["error_zero"]))]

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True, constrained_layout=True)
    ax1, ax2 = axes
    ax1.plot(raw_t, center, color="#d62728", linewidth=1.5, label="RealSense center")
    ax1.plot(raw_t, slosh, color="#1f77b4", linewidth=1.3, label="/slosh/height")
    ax1.set_ylabel("height [mm]")
    ax1.set_title(f"{label}: RealSense center vs /slosh/height")
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="upper right")

    ax2.plot(raw_t, raw_err, color="#9467bd", linewidth=1.2, label="raw error = center - slosh")
    if zero_err:
        ax2.plot(zero_t_err, zero_err, color="#2ca02c", linewidth=1.0, label="zero-aligned error")
    ax2.axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    ax2.set_xlabel("relative time [s]")
    ax2.set_ylabel("error [mm]")
    ax2.grid(True, alpha=0.25)
    ax2.legend(loc="upper right")
    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def build_readme(path: Path, summaries: Sequence[SummaryRow], outlier_rows: Sequence[Dict[str, object]]):
    lines = [
        "# RealSense Center vs /slosh/height 分 bag 分析",
        "",
        "本目录汇总 `0325` 批次中 `height_center_rel_mm_bias_corrected_v2` 对 `/slosh/height` 的按 bag 误差、偏置和坏点分析。",
        "",
        "主要输出：",
        "- `center_vs_slosh_summary.csv`: 每个 bag 的误差、偏置、zero-align 指标",
        "- `center_vs_slosh_outliers.csv`: 各 bag 最差帧及启发式原因",
        "- `<bag>_center_vs_slosh.png`: 每包的原始曲线和误差图",
        "",
        "结论速览：",
    ]
    for row in summaries:
        lines.append(
            f"- `{row.label}`: raw_bias_median={row.raw_bias_median_mm:.3f} mm, "
            f"raw_MAE={row.raw_mae_mm:.3f} mm, zero_MAE={row.zero_mae_mm:.3f} mm, "
            f"reportable={row.reportable_frames}/{row.total_frames}"
        )
    lines.extend(
        [
            "",
            "坏点启发式标签：",
            "- `low_confidence`: meniscus_confidence_v2 < 0.70",
            "- `low_coverage`: central_coverage_v2 < 0.30",
            "- `high_fit_rms`: fit_rms_px_v2 > 5.0",
            "- `mapping_extrapolated`: height mapping 发生外推",
            "- `model_or_bias`: 当前检测指标看起来正常，更像模型侧或残余偏置差异",
            "",
            f"导出的 top outlier 总数：{len(outlier_rows)}",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    verify_root = Path(args.verify_root).expanduser().resolve()
    bag_root = Path(args.bag_root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    ensure_dir(out_dir)

    labels = [label for label in BAG_PATTERNS.keys() if (verify_root / label).is_dir()]
    if not labels:
        raise SystemExit(f"[ERROR] no known verify dirs under {verify_root}")

    summaries: List[SummaryRow] = []
    outlier_rows: List[Dict[str, object]] = []

    for label in labels:
        verify_dir = verify_root / label
        liquid_csv = verify_dir / "liquid_height_v2.csv"
        zero_csv = verify_dir / "mpc_realsense_aligned_v2.csv"
        if not liquid_csv.is_file():
            print(f"[WARN] skip {label}: missing liquid_height_v2.csv")
            continue

        bag_path = find_bag_path(label, bag_root)
        slosh_times, slosh_values = load_slosh_series(bag_path)
        liquid_rows = load_liquid_rows(liquid_csv, args.center_column)
        zero_rows = load_zero_aligned_rows(zero_csv) if zero_csv.is_file() else []

        matched_rows: List[Dict[str, object]] = []
        for row in liquid_rows:
            if int(row["accept_for_peak_report_v2"]) != 1:
                continue
            center_mm = float(row["center_mm"])
            if not math.isfinite(center_mm):
                continue
            slosh_mm = interpolate_scalar(slosh_times, slosh_values, float(row["stamp"]))
            raw_error = center_mm - slosh_mm
            merged = dict(row)
            merged["slosh_mm"] = slosh_mm
            merged["raw_error_mm"] = raw_error
            matched_rows.append(merged)

        zero_lookup_rows = []
        for row in zero_rows:
            matched = match_liquid_row_by_rel_time(matched_rows, float(row["relative_time_s"]))
            merged = dict(row)
            if matched is not None:
                merged["frame_index"] = matched["frame_index"]
                merged["meniscus_confidence_v2"] = matched["meniscus_confidence_v2"]
                merged["central_coverage_v2"] = matched["central_coverage_v2"]
                merged["fit_rms_px_v2"] = matched["fit_rms_px_v2"]
            zero_lookup_rows.append(merged)

        raw_center = [float(row["center_mm"]) for row in matched_rows]
        raw_slosh = [float(row["slosh_mm"]) for row in matched_rows]
        raw_error = [float(row["raw_error_mm"]) for row in matched_rows]
        zero_error = [float(row["error_zero"]) for row in zero_rows if math.isfinite(float(row["error_zero"]))]
        zero_center = [float(row["center_zero"]) for row in zero_rows if math.isfinite(float(row["center_zero"]))]
        zero_slosh = [float(row["slosh_zero"]) for row in zero_rows if math.isfinite(float(row["slosh_zero"]))]

        summaries.append(
            SummaryRow(
                label=label,
                bag_path=str(bag_path),
                total_frames=len(liquid_rows),
                reportable_frames=len(matched_rows),
                reportable_ratio=(len(matched_rows) / len(liquid_rows)) if liquid_rows else math.nan,
                raw_bias_mean_mm=mean_or_nan(raw_error),
                raw_bias_median_mm=float(statistics.median(raw_error)) if raw_error else math.nan,
                raw_mae_mm=mae(raw_center, raw_slosh),
                raw_rmse_mm=rmse(raw_center, raw_slosh),
                raw_corr=corr(raw_center, raw_slosh),
                raw_abs_p90_mm=percentile90(abs(v) for v in raw_error),
                raw_abs_max_mm=max((abs(v) for v in raw_error), default=math.nan),
                zero_mae_mm=mae(zero_center, zero_slosh),
                zero_rmse_mm=rmse(zero_center, zero_slosh),
                zero_corr=corr(zero_center, zero_slosh),
                zero_abs_p90_mm=percentile90(abs(v) for v in zero_error),
                zero_abs_max_mm=max((abs(v) for v in zero_error), default=math.nan),
                mean_confidence=mean_or_nan(float(row["meniscus_confidence_v2"]) for row in matched_rows),
                mean_coverage=mean_or_nan(float(row["central_coverage_v2"]) for row in matched_rows),
                mean_fit_rms_px=mean_or_nan(float(row["fit_rms_px_v2"]) for row in matched_rows),
            )
        )

        ranked = sorted(matched_rows, key=lambda row: abs(float(row["raw_error_mm"])), reverse=True)
        for rank, row in enumerate(ranked[: args.top_outliers], start=1):
            outlier_rows.append(
                {
                    "label": label,
                    "rank": rank,
                    "frame_index": int(row["frame_index"]),
                    "relative_time_s": f"{float(row['relative_time_s']):.6f}",
                    "center_mm": f"{float(row['center_mm']):.6f}",
                    "slosh_mm": f"{float(row['slosh_mm']):.6f}",
                    "raw_error_mm": f"{float(row['raw_error_mm']):.6f}",
                    "meniscus_confidence_v2": f"{float(row['meniscus_confidence_v2']):.6f}" if math.isfinite(float(row["meniscus_confidence_v2"])) else "",
                    "central_coverage_v2": f"{float(row['central_coverage_v2']):.6f}" if math.isfinite(float(row["central_coverage_v2"])) else "",
                    "fit_rms_px_v2": f"{float(row['fit_rms_px_v2']):.6f}" if math.isfinite(float(row["fit_rms_px_v2"])) else "",
                    "height_mapping_extrapolated_v2": int(row["height_mapping_extrapolated_v2"]),
                    "peak_candidate_source_v2": str(row["peak_candidate_source_v2"]),
                    "height_mapping_segment_id_v2": str(row["height_mapping_segment_id_v2"]),
                    "suspected_cause": suspected_cause(row),
                }
            )

        plot_bag(out_dir / f"{label}_center_vs_slosh.png", label, matched_rows, zero_rows)

    summary_csv = out_dir / "center_vs_slosh_summary.csv"
    with summary_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(SummaryRow.__annotations__.keys()))
        writer.writeheader()
        for row in summaries:
            writer.writerow(row.__dict__)

    outlier_csv = out_dir / "center_vs_slosh_outliers.csv"
    outlier_fields = [
        "label",
        "rank",
        "frame_index",
        "relative_time_s",
        "center_mm",
        "slosh_mm",
        "raw_error_mm",
        "meniscus_confidence_v2",
        "central_coverage_v2",
        "fit_rms_px_v2",
        "height_mapping_extrapolated_v2",
        "peak_candidate_source_v2",
        "height_mapping_segment_id_v2",
        "suspected_cause",
    ]
    with outlier_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=outlier_fields)
        writer.writeheader()
        writer.writerows(outlier_rows)

    summary_json = out_dir / "center_vs_slosh_summary.json"
    with summary_json.open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "verify_root": str(verify_root),
                "bag_root": str(bag_root),
                "center_column": args.center_column,
                "bags": [row.__dict__ for row in summaries],
                "outlier_count": len(outlier_rows),
            },
            handle,
            ensure_ascii=False,
            indent=2,
        )

    build_readme(out_dir / "README.md", summaries, outlier_rows)
    print(f"[OK] summary csv: {summary_csv}")
    print(f"[OK] outlier csv: {outlier_csv}")
    print(f"[OK] summary json: {summary_json}")
    print(f"[OK] readme: {out_dir / 'README.md'}")
    for row in summaries:
        print(
            f"[INFO] {row.label}: raw_bias_median={row.raw_bias_median_mm:.6f} mm, "
            f"raw_MAE={row.raw_mae_mm:.6f} mm, zero_MAE={row.zero_mae_mm:.6f} mm, "
            f"reportable={row.reportable_frames}/{row.total_frames}"
        )


if __name__ == "__main__":
    main()
