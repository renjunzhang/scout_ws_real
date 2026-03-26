#!/usr/bin/env python3
"""Analyze human height labels against v2 RealSense/MPC outputs."""

import argparse
import csv
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


DEFAULT_COMPARE_COLUMNS = (
    "center_rel_mm_v2",
    "peak_rel_mm_v2",
    "slosh_height_mm",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze human-labeled liquid heights against v2 RealSense/MPC outputs, "
            "and estimate a practical center-height bias correction."
        )
    )
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument(
        "--debug-dir",
        default="",
        help="Debug session directory containing debug_session.csv.",
    )
    source_group.add_argument(
        "--session-csv",
        default="",
        help="Direct path to debug_session.csv.",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to <debug_dir>/human_label_analysis.",
    )
    parser.add_argument(
        "--target-column",
        default="human_height_mm",
        help="Human label target column. Default: human_height_mm.",
    )
    parser.add_argument(
        "--compare-columns",
        default=",".join(DEFAULT_COMPARE_COLUMNS),
        help="Comma-separated prediction columns to compare against the target.",
    )
    parser.add_argument(
        "--bias-column",
        default="center_rel_mm_v2",
        help="Column used to estimate the RealSense center bias correction.",
    )
    parser.add_argument(
        "--min-zero-label-count",
        type=int,
        default=10,
        help="Minimum number of zero-labeled rows required to prefer zero-label bias estimation. Default: 10.",
    )
    parser.add_argument(
        "--show-first",
        type=int,
        default=10,
        help="Print the first N labeled rows in the terminal summary. Default: 10.",
    )
    return parser.parse_args()


def finite_float(raw_value) -> Optional[float]:
    if raw_value is None:
        return None
    text = str(raw_value).strip()
    if text == "":
        return None
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def session_csv_from_args(args) -> Path:
    if args.session_csv:
        return Path(args.session_csv).expanduser().resolve()
    return (Path(args.debug_dir).expanduser().resolve() / "debug_session.csv").resolve()


def output_dir_from_args(args, session_csv_path: Path) -> Path:
    if args.out_dir:
        return Path(args.out_dir).expanduser().resolve()
    return (session_csv_path.parent / "human_label_analysis").resolve()


def read_rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"session csv has no rows: {path}")
    return rows


def paired_values(rows: Sequence[Dict[str, str]], left_key: str, right_key: str) -> List[Tuple[float, float]]:
    pairs = []
    for row in rows:
        left_value = finite_float(row.get(left_key))
        right_value = finite_float(row.get(right_key))
        if left_value is None or right_value is None:
            continue
        pairs.append((left_value, right_value))
    return pairs


def safe_corr(pairs: Sequence[Tuple[float, float]]) -> Optional[float]:
    if len(pairs) < 2:
        return None
    xs = np.asarray([pair[0] for pair in pairs], dtype=np.float64)
    ys = np.asarray([pair[1] for pair in pairs], dtype=np.float64)
    mask = np.isfinite(xs) & np.isfinite(ys)
    xs = xs[mask]
    ys = ys[mask]
    if xs.size < 2:
        return None
    xs_std = float(np.std(xs))
    ys_std = float(np.std(ys))
    if xs_std <= 0.0 or ys_std <= 0.0:
        return None
    return float(np.corrcoef(xs, ys)[0, 1])


def summary_from_pairs(pairs: Sequence[Tuple[float, float]]) -> Dict[str, Optional[float]]:
    if not pairs:
        return {
            "count": 0,
            "target_mean": None,
            "pred_mean": None,
            "bias_mean": None,
            "bias_median": None,
            "mae": None,
            "rmse": None,
            "corr": None,
        }
    targets = np.asarray([pair[0] for pair in pairs], dtype=np.float64)
    preds = np.asarray([pair[1] for pair in pairs], dtype=np.float64)
    residuals = preds - targets
    return {
        "count": int(targets.size),
        "target_mean": float(np.mean(targets)),
        "pred_mean": float(np.mean(preds)),
        "bias_mean": float(np.mean(residuals)),
        "bias_median": float(np.median(residuals)),
        "mae": float(np.mean(np.abs(residuals))),
        "rmse": float(np.sqrt(np.mean(np.square(residuals)))),
        "corr": safe_corr(pairs),
    }


def fit_affine_target_from_prediction(pairs: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    if len(pairs) < 2:
        return None
    targets = np.asarray([pair[0] for pair in pairs], dtype=np.float64)
    preds = np.asarray([pair[1] for pair in pairs], dtype=np.float64)
    fit = np.polyfit(preds, targets, deg=1)
    slope = float(fit[0])
    intercept = float(fit[1])
    return slope, intercept


def corrected_pairs_constant_bias(
    rows: Sequence[Dict[str, str]],
    target_column: str,
    prediction_column: str,
    bias_mm: float,
) -> List[Tuple[float, float]]:
    pairs = []
    for row in rows:
        target_value = finite_float(row.get(target_column))
        prediction_value = finite_float(row.get(prediction_column))
        if target_value is None or prediction_value is None:
            continue
        corrected = max(0.0, prediction_value - bias_mm)
        pairs.append((target_value, corrected))
    return pairs


def corrected_pairs_affine(
    rows: Sequence[Dict[str, str]],
    target_column: str,
    prediction_column: str,
    slope: float,
    intercept: float,
) -> List[Tuple[float, float]]:
    pairs = []
    for row in rows:
        target_value = finite_float(row.get(target_column))
        prediction_value = finite_float(row.get(prediction_column))
        if target_value is None or prediction_value is None:
            continue
        corrected = max(0.0, slope * prediction_value + intercept)
        pairs.append((target_value, corrected))
    return pairs


def recommend_bias_mm(
    labeled_rows: Sequence[Dict[str, str]],
    target_column: str,
    bias_column: str,
    min_zero_label_count: int,
) -> Dict[str, Optional[float]]:
    all_pairs = paired_values(labeled_rows, target_column, bias_column)
    all_residuals = [pair[1] - pair[0] for pair in all_pairs]

    zero_label_rows = []
    for row in labeled_rows:
        target_value = finite_float(row.get(target_column))
        if target_value is None:
            continue
        if abs(target_value) <= 1e-9:
            zero_label_rows.append(row)
    zero_pairs = paired_values(zero_label_rows, target_column, bias_column)
    zero_residuals = [pair[1] - pair[0] for pair in zero_pairs]

    recommended_bias = None
    recommended_source = ""
    if len(zero_residuals) >= min_zero_label_count:
        recommended_bias = float(np.median(np.asarray(zero_residuals, dtype=np.float64)))
        recommended_source = "zero_label_median"
    elif all_residuals:
        recommended_bias = float(np.median(np.asarray(all_residuals, dtype=np.float64)))
        recommended_source = "all_label_median"

    return {
        "zero_label_count": len(zero_residuals),
        "zero_label_bias_mean": None if not zero_residuals else float(np.mean(np.asarray(zero_residuals, dtype=np.float64))),
        "zero_label_bias_median": None if not zero_residuals else float(np.median(np.asarray(zero_residuals, dtype=np.float64))),
        "all_label_count": len(all_residuals),
        "all_label_bias_mean": None if not all_residuals else float(np.mean(np.asarray(all_residuals, dtype=np.float64))),
        "all_label_bias_median": None if not all_residuals else float(np.median(np.asarray(all_residuals, dtype=np.float64))),
        "recommended_bias_mm": recommended_bias,
        "recommended_source": recommended_source,
    }


def write_analysis_rows_csv(
    path: Path,
    rows: Sequence[Dict[str, str]],
    target_column: str,
    recommended_bias_mm: Optional[float],
    affine_fit: Optional[Tuple[float, float]],
):
    fieldnames = [
        "frame_index",
        "relative_time_s",
        target_column,
        "center_rel_mm_v2",
        "peak_rel_mm_v2",
        "slosh_height_mm",
        "confidence_v2",
        "valid_v2",
        "accept_for_peak_report_v2",
        "center_minus_target_mm",
        "peak_minus_target_mm",
        "slosh_minus_target_mm",
        "center_bias_corrected_mm",
        "center_bias_corrected_minus_target_mm",
        "center_affine_corrected_mm",
        "center_affine_corrected_minus_target_mm",
    ]
    slope, intercept = (None, None) if affine_fit is None else affine_fit
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            target_value = finite_float(row.get(target_column))
            center_value = finite_float(row.get("center_rel_mm_v2"))
            peak_value = finite_float(row.get("peak_rel_mm_v2"))
            slosh_value = finite_float(row.get("slosh_height_mm"))
            bias_corrected = None
            affine_corrected = None
            if recommended_bias_mm is not None and center_value is not None:
                bias_corrected = max(0.0, center_value - recommended_bias_mm)
            if slope is not None and intercept is not None and center_value is not None:
                affine_corrected = max(0.0, slope * center_value + intercept)
            writer.writerow(
                {
                    "frame_index": row.get("frame_index", ""),
                    "relative_time_s": row.get("relative_time_s", ""),
                    target_column: "" if target_value is None else f"{target_value:.6f}",
                    "center_rel_mm_v2": row.get("center_rel_mm_v2", ""),
                    "peak_rel_mm_v2": row.get("peak_rel_mm_v2", ""),
                    "slosh_height_mm": row.get("slosh_height_mm", ""),
                    "confidence_v2": row.get("confidence_v2", ""),
                    "valid_v2": row.get("valid_v2", ""),
                    "accept_for_peak_report_v2": row.get("accept_for_peak_report_v2", ""),
                    "center_minus_target_mm": "" if center_value is None or target_value is None else f"{center_value - target_value:.6f}",
                    "peak_minus_target_mm": "" if peak_value is None or target_value is None else f"{peak_value - target_value:.6f}",
                    "slosh_minus_target_mm": "" if slosh_value is None or target_value is None else f"{slosh_value - target_value:.6f}",
                    "center_bias_corrected_mm": "" if bias_corrected is None else f"{bias_corrected:.6f}",
                    "center_bias_corrected_minus_target_mm": "" if bias_corrected is None or target_value is None else f"{bias_corrected - target_value:.6f}",
                    "center_affine_corrected_mm": "" if affine_corrected is None else f"{affine_corrected:.6f}",
                    "center_affine_corrected_minus_target_mm": "" if affine_corrected is None or target_value is None else f"{affine_corrected - target_value:.6f}",
                }
            )


def print_terminal_summary(
    session_csv_path: Path,
    out_dir: Path,
    target_column: str,
    compare_stats: Dict[str, Dict[str, Optional[float]]],
    bias_info: Dict[str, Optional[float]],
    bias_corrected_stats: Optional[Dict[str, Optional[float]]],
    affine_fit: Optional[Tuple[float, float]],
    affine_stats: Optional[Dict[str, Optional[float]]],
    labeled_rows: Sequence[Dict[str, str]],
    show_first: int,
):
    target_values = [finite_float(row.get(target_column)) for row in labeled_rows]
    target_values = [value for value in target_values if value is not None]
    unique_labels = Counter(target_values)

    print(f"[OK] session csv: {session_csv_path}")
    print(f"[OK] output dir: {out_dir}")
    print(f"[OK] labeled rows: {len(labeled_rows)}")
    print(f"[OK] unique human labels: {len(unique_labels)}")
    print("[OK] label histogram:")
    for value, count in sorted(unique_labels.items()):
        print(f"  - {value:.3f} mm: {count}")

    print("[OK] raw comparison metrics:")
    for column_name, stats_dict in compare_stats.items():
        mae_text = "nan" if stats_dict["mae"] is None else f"{stats_dict['mae']:.6f}"
        print(
            f"  - {column_name}: n={stats_dict['count']} mae={mae_text}"
        )

    for column_name, stats_dict in compare_stats.items():
        mae_text = "nan" if stats_dict["mae"] is None else f"{stats_dict['mae']:.6f}"
        rmse_text = "nan" if stats_dict["rmse"] is None else f"{stats_dict['rmse']:.6f}"
        corr_text = "nan" if stats_dict["corr"] is None else f"{stats_dict['corr']:.6f}"
        bias_med_text = "nan" if stats_dict["bias_median"] is None else f"{stats_dict['bias_median']:.6f}"
        print(
            f"    {column_name}: mae={mae_text} rmse={rmse_text} "
            f"corr={corr_text} bias_median={bias_med_text}"
        )

    rec_bias_text = "nan" if bias_info["recommended_bias_mm"] is None else f"{bias_info['recommended_bias_mm']:.6f}"
    zero_med_text = "nan" if bias_info["zero_label_bias_median"] is None else f"{bias_info['zero_label_bias_median']:.6f}"
    all_med_text = "nan" if bias_info["all_label_bias_median"] is None else f"{bias_info['all_label_bias_median']:.6f}"
    print(
        f"[OK] suggested center bias correction: {rec_bias_text} mm "
        f"(source={bias_info['recommended_source']}, zero_med={zero_med_text}, all_med={all_med_text})"
    )
    if bias_corrected_stats is not None:
        bias_corr_text = "nan" if bias_corrected_stats["corr"] is None else f"{bias_corrected_stats['corr']:.6f}"
        print(
            "[OK] center after constant-bias correction: "
            f"mae={bias_corrected_stats['mae']:.6f} "
            f"rmse={bias_corrected_stats['rmse']:.6f} "
            f"corr={bias_corr_text}"
        )
    if affine_fit is not None and affine_stats is not None:
        slope, intercept = affine_fit
        corr_text = "nan" if affine_stats["corr"] is None else f"{affine_stats['corr']:.6f}"
        print(
            "[OK] center affine fit target ~= slope*center + intercept: "
            f"slope={slope:.6f} intercept={intercept:.6f} "
            f"mae={affine_stats['mae']:.6f} rmse={affine_stats['rmse']:.6f} corr={corr_text}"
        )
    print("[OK] first labeled rows:")
    for row in labeled_rows[:show_first]:
        print(
            "  - "
            f"frame={row.get('frame_index')} "
            f"human={row.get(target_column)} "
            f"center={row.get('center_rel_mm_v2')} "
            f"peak={row.get('peak_rel_mm_v2')} "
            f"slosh={row.get('slosh_height_mm')} "
            f"conf={row.get('confidence_v2')}"
        )


def analyze(args) -> int:
    session_csv_path = session_csv_from_args(args)
    out_dir = output_dir_from_args(args, session_csv_path)
    ensure_dir(out_dir)

    rows = read_rows(session_csv_path)
    fields = set(rows[0].keys())
    if args.target_column not in fields:
        raise RuntimeError(f"missing target column in session csv: {args.target_column}")
    if args.bias_column not in fields:
        raise RuntimeError(f"missing bias column in session csv: {args.bias_column}")

    compare_columns = [column.strip() for column in str(args.compare_columns).split(",") if column.strip()]
    for column in compare_columns:
        if column not in fields:
            raise RuntimeError(f"missing compare column in session csv: {column}")

    labeled_rows = [row for row in rows if finite_float(row.get(args.target_column)) is not None]
    if not labeled_rows:
        raise RuntimeError(f"no labeled rows found in target column: {args.target_column}")

    compare_stats = {}
    for column in compare_columns:
        compare_stats[column] = summary_from_pairs(paired_values(labeled_rows, args.target_column, column))

    bias_info = recommend_bias_mm(labeled_rows, args.target_column, args.bias_column, args.min_zero_label_count)
    bias_corrected_stats = None
    if bias_info["recommended_bias_mm"] is not None:
        corrected = corrected_pairs_constant_bias(
            labeled_rows,
            args.target_column,
            args.bias_column,
            float(bias_info["recommended_bias_mm"]),
        )
        bias_corrected_stats = summary_from_pairs(corrected)

    affine_fit = fit_affine_target_from_prediction(paired_values(labeled_rows, args.target_column, args.bias_column))
    affine_stats = None
    if affine_fit is not None:
        affine_corrected = corrected_pairs_affine(
            labeled_rows,
            args.target_column,
            args.bias_column,
            affine_fit[0],
            affine_fit[1],
        )
        affine_stats = summary_from_pairs(affine_corrected)

    summary = {
        "session_csv": str(session_csv_path),
        "out_dir": str(out_dir),
        "target_column": args.target_column,
        "compare_columns": compare_columns,
        "labeled_row_count": len(labeled_rows),
        "compare_stats": compare_stats,
        "bias_column": args.bias_column,
        "bias_info": bias_info,
        "bias_corrected_stats": bias_corrected_stats,
        "affine_fit": None if affine_fit is None else {"slope": affine_fit[0], "intercept": affine_fit[1]},
        "affine_stats": affine_stats,
    }

    with (out_dir / "human_label_analysis_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    write_analysis_rows_csv(
        out_dir / "human_label_analysis_rows.csv",
        labeled_rows,
        args.target_column,
        bias_info["recommended_bias_mm"],
        affine_fit,
    )

    print_terminal_summary(
        session_csv_path,
        out_dir,
        args.target_column,
        compare_stats,
        bias_info,
        bias_corrected_stats,
        affine_fit,
        affine_stats,
        labeled_rows,
        args.show_first,
    )

    return 0


def main():
    try:
        raise SystemExit(analyze(parse_args()))
    except KeyboardInterrupt:
        print("[INFO] interrupted", file=sys.stderr)
        raise SystemExit(130)


if __name__ == "__main__":
    main()
