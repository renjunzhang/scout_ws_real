#!/usr/bin/env python3
"""Project y-rect predictions back to mm and compare against human/slosh baselines."""

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception as exc:  # pylint: disable=broad-except
    plt = None
    PLOT_IMPORT_ERROR = exc
else:
    PLOT_IMPORT_ERROR = None

import yaml


@dataclass
class ReferencePoint:
    x_px: float
    y_px: float
    h_mm: float


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Convert SL peak_y_rect_px predictions to peak_mm using fixed calibration mapping, "
            "then compare against human_peak_mm and /slosh/height."
        )
    )
    parser.add_argument("--predictions-csv", required=True, help="Path to SL_visual_human_predictions.csv.")
    parser.add_argument("--manifest-csv", required=True, help="Path to manifest csv with human_peak_mm.")
    parser.add_argument("--out-dir", required=True, help="Output directory for csv/json/plots.")
    parser.add_argument(
        "--splits",
        default="val,test",
        help="Comma-separated splits to plot. Summary will still include all available splits.",
    )
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


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


def read_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"csv has no rows: {path}")
    return rows


def parse_reference_points(calibration_path: Path) -> List[ReferencePoint]:
    with calibration_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    height_mapping = data.get("height_mapping", {})
    raw_points = height_mapping.get("reference_points", [])
    if len(raw_points) < 2:
        raise RuntimeError(f"invalid reference_points in {calibration_path}")
    points = []
    for raw_point in raw_points:
        points.append(
            ReferencePoint(
                x_px=float(raw_point["x_px"]),
                y_px=float(raw_point["y_px"]),
                h_mm=float(raw_point["h_mm"]),
            )
        )
    return points


def interpolate_h_from_y(first: ReferencePoint, second: ReferencePoint, y_px: float) -> float:
    delta = float(second.y_px - first.y_px)
    if abs(delta) < 1e-9:
        raise RuntimeError("reference point y values repeat")
    alpha = (float(y_px) - float(first.y_px)) / delta
    return float(first.h_mm + alpha * (second.h_mm - first.h_mm))


def evaluate_height_mapping(reference_points: Sequence[ReferencePoint], y_px: Optional[float]) -> Optional[float]:
    if y_px is None or not math.isfinite(float(y_px)):
        return None
    y_value = float(y_px)
    if y_value >= reference_points[0].y_px:
        return interpolate_h_from_y(reference_points[0], reference_points[1], y_value)
    if y_value <= reference_points[-1].y_px:
        return interpolate_h_from_y(reference_points[-2], reference_points[-1], y_value)
    for index in range(len(reference_points) - 1):
        first = reference_points[index]
        second = reference_points[index + 1]
        if first.y_px >= y_value >= second.y_px:
            return interpolate_h_from_y(first, second, y_value)
    raise RuntimeError("failed to map y_rect to height")


def regression_metrics(targets: Sequence[float], preds: Sequence[float]) -> Dict[str, Optional[float]]:
    if len(targets) != len(preds):
        raise RuntimeError("targets/preds length mismatch")
    if not targets:
        return {"count": 0, "mae": None, "rmse": None, "bias_mean": None, "bias_median": None, "corr": None}
    errors = [float(pred - target) for target, pred in zip(targets, preds)]
    abs_errors = [abs(error) for error in errors]
    mae = float(sum(abs_errors) / len(abs_errors))
    rmse = float((sum(error * error for error in errors) / len(errors)) ** 0.5)
    bias_mean = float(sum(errors) / len(errors))
    sorted_errors = sorted(errors)
    mid = len(sorted_errors) // 2
    if len(sorted_errors) % 2 == 0:
        bias_median = float(0.5 * (sorted_errors[mid - 1] + sorted_errors[mid]))
    else:
        bias_median = float(sorted_errors[mid])
    corr = None
    if len(targets) >= 2:
        mean_t = float(sum(targets) / len(targets))
        mean_p = float(sum(preds) / len(preds))
        var_t = float(sum((target - mean_t) ** 2 for target in targets))
        var_p = float(sum((pred - mean_p) ** 2 for pred in preds))
        if var_t > 1e-12 and var_p > 1e-12:
            cov = float(sum((target - mean_t) * (pred - mean_p) for target, pred in zip(targets, preds)))
            corr = cov / math.sqrt(var_t * var_p)
    return {
        "count": len(targets),
        "mae": mae,
        "rmse": rmse,
        "bias_mean": bias_mean,
        "bias_median": bias_median,
        "corr": corr,
    }


def collect_metrics(rows: Sequence[Dict[str, object]], pred_key: str) -> Dict[str, Optional[float]]:
    targets = []
    preds = []
    for row in rows:
        target = finite_float(row.get("human_peak_mm"))
        pred = finite_float(row.get(pred_key))
        if target is None or pred is None:
            continue
        targets.append(float(target))
        preds.append(float(pred))
    return regression_metrics(targets, preds)


def group_rows(rows: Sequence[Dict[str, object]]) -> Dict[Tuple[str, str, str], List[Dict[str, object]]]:
    grouped: Dict[Tuple[str, str, str], List[Dict[str, object]]] = {}
    for row in rows:
        key = (str(row.get("split", "")), str(row.get("session_name", "")), str(row.get("bag_id", "")))
        grouped.setdefault(key, []).append(dict(row))
    for key, value in grouped.items():
        grouped[key] = sorted(
            value,
            key=lambda item: (
                finite_float(item.get("relative_time_s")) or float("inf"),
                finite_float(item.get("frame_index")) or float("inf"),
            ),
        )
    return grouped


def write_csv(path: Path, rows: Sequence[Dict[str, object]], fieldnames: Sequence[str]):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def make_plot(
    path: Path,
    grouped_rows: Dict[Tuple[str, str, str], List[Dict[str, object]]],
    selected_splits: Sequence[str],
):
    if plt is None:
        raise RuntimeError(f"matplotlib unavailable: {PLOT_IMPORT_ERROR}")
    panels = [item for item in grouped_rows.items() if item[0][0] in selected_splits]
    if not panels:
        raise RuntimeError("no rows matched selected splits for plotting")

    fig, axes = plt.subplots(len(panels), 1, figsize=(14, 3.3 * len(panels)), sharex=False)
    if len(panels) == 1:
        axes = [axes]

    for axis, ((split_name, session_name, _bag_id), rows) in zip(axes, panels):
        time_values = [finite_float(row.get("relative_time_s")) or 0.0 for row in rows]
        human_values = [finite_float(row.get("human_peak_mm")) for row in rows]
        visual_values = [finite_float(row.get("pred_peak_mm")) for row in rows]
        slosh_values = [finite_float(row.get("slosh_height_mm")) for row in rows]
        axis.plot(time_values, human_values, color="#202020", linewidth=1.7, label="human peak")
        axis.plot(time_values, visual_values, color="#d9485f", linewidth=1.6, label="SL y->mm")
        axis.plot(time_values, slosh_values, color="#7ba6d0", linewidth=1.4, label="bag /slosh/height")
        axis.set_title(f"{split_name}: {session_name}")
        axis.set_ylabel("height (mm)")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", fontsize=9)
    axes[-1].set_xlabel("relative_time_s")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    try:
        args = parse_args()
        predictions_csv = Path(args.predictions_csv).expanduser().resolve()
        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        pred_rows = read_rows(predictions_csv)
        manifest_rows = read_rows(manifest_csv)
        manifest_by_row_id = {str(row.get("row_id", "")): row for row in manifest_rows}
        calibration_cache: Dict[str, List[ReferencePoint]] = {}

        merged_rows: List[Dict[str, object]] = []
        for pred_row in pred_rows:
            row_id = str(pred_row.get("row_id", ""))
            manifest_row = manifest_by_row_id.get(row_id)
            if manifest_row is None:
                continue
            calibration_path = Path(str(manifest_row.get("calibration_path", "")).strip()).expanduser()
            if not calibration_path.exists():
                raise RuntimeError(f"calibration missing for row_id={row_id}: {calibration_path}")
            cache_key = str(calibration_path.resolve())
            if cache_key not in calibration_cache:
                calibration_cache[cache_key] = parse_reference_points(calibration_path)
            reference_points = calibration_cache[cache_key]

            target_y_px = finite_float(pred_row.get("target_value"))
            pred_y_px = finite_float(pred_row.get("pred_visual"))
            human_peak_mm = finite_float(manifest_row.get("human_peak_mm"))
            human_peak_y_rect_px = finite_float(manifest_row.get("human_peak_y_rect_px"))
            slosh_height_mm = finite_float(manifest_row.get("slosh_height_mm"))

            target_mm_from_target_y = evaluate_height_mapping(reference_points, target_y_px)
            target_mm_from_human_y = evaluate_height_mapping(reference_points, human_peak_y_rect_px)
            pred_peak_mm = evaluate_height_mapping(reference_points, pred_y_px)

            merged_rows.append(
                {
                    "split": str(pred_row.get("split", "")),
                    "row_id": row_id,
                    "session_id": str(pred_row.get("session_id", "")),
                    "session_name": str(pred_row.get("session_name", "")),
                    "bag_id": str(pred_row.get("bag_id", "")),
                    "frame_index": int(float(pred_row.get("frame_index", -1))),
                    "relative_time_s": float(finite_float(pred_row.get("relative_time_s")) or 0.0),
                    "image_path": str(pred_row.get("image_path", "")),
                    "calibration_path": str(calibration_path),
                    "target_peak_y_rect_px": target_y_px,
                    "human_peak_y_rect_px": human_peak_y_rect_px,
                    "pred_peak_y_rect_px": pred_y_px,
                    "human_peak_mm": human_peak_mm,
                    "target_peak_mm_from_target_y": target_mm_from_target_y,
                    "target_peak_mm_from_human_y": target_mm_from_human_y,
                    "pred_peak_mm": pred_peak_mm,
                    "slosh_height_mm": slosh_height_mm,
                    "pred_peak_mm_abs_error": None if human_peak_mm is None or pred_peak_mm is None else abs(pred_peak_mm - human_peak_mm),
                    "slosh_height_mm_abs_error": None if human_peak_mm is None or slosh_height_mm is None else abs(slosh_height_mm - human_peak_mm),
                    "y_rect_to_mm_target_gap": None
                    if human_peak_mm is None or target_mm_from_human_y is None
                    else (target_mm_from_human_y - human_peak_mm),
                }
            )

        if not merged_rows:
            raise RuntimeError("no overlapping row_id entries between predictions and manifest")

        projection_csv = out_dir / "SL_y_rect_projected_mm_predictions.csv"
        write_csv(
            projection_csv,
            merged_rows,
            [
                "split",
                "row_id",
                "session_id",
                "session_name",
                "bag_id",
                "frame_index",
                "relative_time_s",
                "image_path",
                "calibration_path",
                "target_peak_y_rect_px",
                "human_peak_y_rect_px",
                "pred_peak_y_rect_px",
                "human_peak_mm",
                "target_peak_mm_from_target_y",
                "target_peak_mm_from_human_y",
                "pred_peak_mm",
                "slosh_height_mm",
                "pred_peak_mm_abs_error",
                "slosh_height_mm_abs_error",
                "y_rect_to_mm_target_gap",
            ],
        )

        summary: Dict[str, object] = {
            "predictions_csv": str(predictions_csv),
            "manifest_csv": str(manifest_csv),
            "projection_csv": str(projection_csv),
            "num_rows": len(merged_rows),
            "overall": {},
            "human_y_to_mm_consistency": {},
        }

        grouped = group_rows(merged_rows)
        bagwise_rows: List[Dict[str, object]] = []
        for split_name in sorted({str(row.get("split", "")) for row in merged_rows}):
            split_rows = [row for row in merged_rows if str(row.get("split", "")) == split_name]
            summary["overall"][split_name] = {
                "sl_y_rect_projected_mm": collect_metrics(split_rows, "pred_peak_mm"),
                "bag_slosh_height_mm": collect_metrics(split_rows, "slosh_height_mm"),
            }
            consistency_values = [
                abs(float(row["y_rect_to_mm_target_gap"]))
                for row in split_rows
                if finite_float(row.get("y_rect_to_mm_target_gap")) is not None
            ]
            summary["human_y_to_mm_consistency"][split_name] = {
                "count": len(consistency_values),
                "mae": None if not consistency_values else float(sum(consistency_values) / len(consistency_values)),
            }

        for (split_name, session_name, bag_id), rows in grouped.items():
            bagwise_rows.append(
                {
                    "split": split_name,
                    "bag_id": bag_id,
                    "session_name": session_name,
                    "sl_y_rect_projected_mm_mae": collect_metrics(rows, "pred_peak_mm").get("mae"),
                    "sl_y_rect_projected_mm_rmse": collect_metrics(rows, "pred_peak_mm").get("rmse"),
                    "sl_y_rect_projected_mm_corr": collect_metrics(rows, "pred_peak_mm").get("corr"),
                    "bag_slosh_height_mm_mae": collect_metrics(rows, "slosh_height_mm").get("mae"),
                    "bag_slosh_height_mm_rmse": collect_metrics(rows, "slosh_height_mm").get("rmse"),
                    "bag_slosh_height_mm_corr": collect_metrics(rows, "slosh_height_mm").get("corr"),
                    "count": len(rows),
                }
            )
        write_csv(
            out_dir / "SL_y_rect_projected_mm_bagwise.csv",
            bagwise_rows,
            [
                "split",
                "bag_id",
                "session_name",
                "count",
                "sl_y_rect_projected_mm_mae",
                "sl_y_rect_projected_mm_rmse",
                "sl_y_rect_projected_mm_corr",
                "bag_slosh_height_mm_mae",
                "bag_slosh_height_mm_rmse",
                "bag_slosh_height_mm_corr",
            ],
        )

        selected_splits = [text.strip() for text in str(args.splits).split(",") if text.strip()]
        make_plot(out_dir / "SL_y_rect_projected_mm_all.png", grouped, selected_splits)
        for split_name in selected_splits:
            make_plot(out_dir / f"SL_y_rect_projected_mm_{split_name}.png", grouped, [split_name])

        with (out_dir / "SL_y_rect_projected_mm_summary.json").open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] wrote projection csv: {projection_csv}")
        print(f"[OK] wrote summary: {out_dir / 'SL_y_rect_projected_mm_summary.json'}")
        return 0
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
