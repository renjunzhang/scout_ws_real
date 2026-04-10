#!/usr/bin/env python3
"""Plot only SL visual prediction vs human peak labels."""

import argparse
import csv
import json
import math
import sys
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


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot only current SL visual output and human peak labels from an SL predictions CSV."
    )
    parser.add_argument("--predictions-csv", required=True, help="Path to SL_visual_human_predictions.csv.")
    parser.add_argument("--out-dir", required=True, help="Output directory for plots and summary files.")
    parser.add_argument(
        "--splits",
        default="val,test",
        help="Comma-separated split list to include. Default: val,test.",
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


def group_rows(rows: Sequence[Dict[str, str]]) -> Dict[Tuple[str, str, str], List[Dict[str, str]]]:
    grouped: Dict[Tuple[str, str, str], List[Dict[str, str]]] = {}
    for row in rows:
        key = (str(row.get("split", "")), str(row.get("session_name", "")), str(row.get("bag_id", "")))
        grouped.setdefault(key, []).append(dict(row))
    for key in grouped:
        grouped[key] = sorted(
            grouped[key],
            key=lambda item: (
                finite_float(item.get("relative_time_s")) or float("inf"),
                finite_float(item.get("frame_index")) or float("inf"),
            ),
        )
    return grouped


def write_bagwise_csv(path: Path, rows: Sequence[Dict[str, object]]):
    fieldnames = [
        "split",
        "bag_id",
        "session_name",
        "count",
        "mae",
        "rmse",
        "bias_mean",
        "bias_median",
        "corr",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def absolute_error_series(rows: Sequence[Dict[str, str]], pred_column: str) -> List[Optional[float]]:
    errors: List[Optional[float]] = []
    for row in rows:
        target = finite_float(row.get("target_human_peak_mm"))
        pred = finite_float(row.get(pred_column))
        if target is None or pred is None:
            errors.append(None)
            continue
        errors.append(abs(float(pred) - float(target)))
    return errors


def residual_series(rows: Sequence[Dict[str, str]], pred_column: str) -> List[Optional[float]]:
    residuals: List[Optional[float]] = []
    for row in rows:
        target = finite_float(row.get("target_human_peak_mm"))
        pred = finite_float(row.get(pred_column))
        if target is None or pred is None:
            residuals.append(None)
            continue
        residuals.append(float(pred) - float(target))
    return residuals


def bag_mae_text(rows: Sequence[Dict[str, str]], pred_column: str) -> str:
    targets = []
    preds = []
    for row in rows:
        target = finite_float(row.get("target_human_peak_mm"))
        pred = finite_float(row.get(pred_column))
        if target is None or pred is None:
            continue
        targets.append(float(target))
        preds.append(float(pred))
    metrics = regression_metrics(targets, preds) if targets else {"mae": None}
    mae = metrics.get("mae")
    if mae is None:
        return "MAE=n/a"
    return f"MAE={float(mae):.4f} mm"


def bag_bias_text(rows: Sequence[Dict[str, str]], pred_column: str) -> str:
    targets = []
    preds = []
    for row in rows:
        target = finite_float(row.get("target_human_peak_mm"))
        pred = finite_float(row.get(pred_column))
        if target is None or pred is None:
            continue
        targets.append(float(target))
        preds.append(float(pred))
    metrics = regression_metrics(targets, preds) if targets else {"bias_mean": None}
    bias_mean = metrics.get("bias_mean")
    if bias_mean is None:
        return "bias=n/a"
    return f"bias={float(bias_mean):+.4f} mm"


def make_plot(path: Path, grouped_rows: Dict[Tuple[str, str, str], List[Dict[str, str]]], selected_splits: Sequence[str]):
    if plt is None:
        raise RuntimeError(f"matplotlib unavailable: {PLOT_IMPORT_ERROR}")
    panels = [item for item in grouped_rows.items() if item[0][0] in selected_splits]
    if not panels:
        raise RuntimeError("no rows matched selected splits for plotting")

    fig, axes = plt.subplots(len(panels), 1, figsize=(14, 3.2 * len(panels)), sharex=False)
    if len(panels) == 1:
        axes = [axes]

    for axis, ((split_name, session_name, _bag_id), rows) in zip(axes, panels):
        time_values = [finite_float(row.get("relative_time_s")) or 0.0 for row in rows]
        human_values = [finite_float(row.get("target_human_peak_mm")) for row in rows]
        visual_values = [finite_float(row.get("pred_visual")) for row in rows]
        axis.plot(time_values, human_values, color="#202020", linewidth=1.7, label="human peak")
        axis.plot(time_values, visual_values, color="#d9485f", linewidth=1.6, label="SL visual")
        axis.set_title(f"{split_name}: {session_name}")
        axis.set_ylabel("height (mm)")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", fontsize=9)
    axes[-1].set_xlabel("relative_time_s")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def make_residual_plot(path: Path, grouped_rows: Dict[Tuple[str, str, str], List[Dict[str, str]]], selected_splits: Sequence[str]):
    if plt is None:
        raise RuntimeError(f"matplotlib unavailable: {PLOT_IMPORT_ERROR}")
    panels = [item for item in grouped_rows.items() if item[0][0] in selected_splits]
    if not panels:
        raise RuntimeError("no rows matched selected splits for plotting")

    fig, axes = plt.subplots(len(panels), 1, figsize=(14, 3.2 * len(panels)), sharex=False)
    if len(panels) == 1:
        axes = [axes]

    for axis, ((split_name, session_name, _bag_id), rows) in zip(axes, panels):
        time_values = [finite_float(row.get("relative_time_s")) or 0.0 for row in rows]
        visual_residual = residual_series(rows, "pred_visual")
        axis.plot(time_values, visual_residual, color="#d9485f", linewidth=1.6, label="SL visual - human")
        axis.axhline(0.0, color="#666666", linestyle="--", linewidth=1.0)
        axis.set_title(
            f"{split_name}: {session_name} | "
            f"{bag_mae_text(rows, 'pred_visual')}, {bag_bias_text(rows, 'pred_visual')}"
        )
        axis.set_ylabel("residual (mm)")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", fontsize=9)
    axes[-1].set_xlabel("relative_time_s")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def make_error_plot(path: Path, grouped_rows: Dict[Tuple[str, str, str], List[Dict[str, str]]], selected_splits: Sequence[str]):
    if plt is None:
        raise RuntimeError(f"matplotlib unavailable: {PLOT_IMPORT_ERROR}")
    panels = [item for item in grouped_rows.items() if item[0][0] in selected_splits]
    if not panels:
        raise RuntimeError("no rows matched selected splits for plotting")

    fig, axes = plt.subplots(len(panels), 1, figsize=(14, 3.2 * len(panels)), sharex=False)
    if len(panels) == 1:
        axes = [axes]

    for axis, ((split_name, session_name, _bag_id), rows) in zip(axes, panels):
        time_values = [finite_float(row.get("relative_time_s")) or 0.0 for row in rows]
        visual_abs_error = absolute_error_series(rows, "pred_visual")
        axis.plot(time_values, visual_abs_error, color="#d9485f", linewidth=1.6, label="|SL visual - human|")
        axis.set_title(f"{split_name}: {session_name} | {bag_mae_text(rows, 'pred_visual')}")
        axis.set_ylabel("abs error (mm)")
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
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        rows = read_rows(predictions_csv)
        selected_splits = [text.strip() for text in str(args.splits).split(",") if text.strip()]
        filtered_rows = [row for row in rows if str(row.get("split", "")) in selected_splits]
        if not filtered_rows:
            raise RuntimeError(f"no rows matched splits={selected_splits}")

        grouped = group_rows(filtered_rows)
        make_plot(out_dir / "SL_vs_human_only_all.png", grouped, selected_splits)
        make_residual_plot(out_dir / "SL_vs_human_only_residual_all.png", grouped, selected_splits)
        make_error_plot(out_dir / "SL_vs_human_only_error_all.png", grouped, selected_splits)
        for split_name in selected_splits:
            make_plot(out_dir / f"SL_vs_human_only_{split_name}.png", grouped, [split_name])
            make_residual_plot(out_dir / f"SL_vs_human_only_residual_{split_name}.png", grouped, [split_name])
            make_error_plot(out_dir / f"SL_vs_human_only_error_{split_name}.png", grouped, [split_name])

        summary: Dict[str, object] = {
            "predictions_csv": str(predictions_csv),
            "selected_splits": selected_splits,
            "num_rows": len(filtered_rows),
            "overall": {},
        }
        bagwise_rows: List[Dict[str, object]] = []

        for split_name in selected_splits:
            split_rows = [row for row in filtered_rows if str(row.get("split", "")) == split_name]
            targets = [float(row["target_human_peak_mm"]) for row in split_rows]
            visual = [float(row["pred_visual"]) for row in split_rows]
            summary["overall"][split_name] = regression_metrics(targets, visual)

        for (split_name, session_name, bag_id), rows in grouped.items():
            if split_name not in selected_splits:
                continue
            targets = [float(row["target_human_peak_mm"]) for row in rows]
            visual = [float(row["pred_visual"]) for row in rows]
            metrics = regression_metrics(targets, visual)
            bagwise_rows.append(
                {
                    "split": split_name,
                    "bag_id": bag_id,
                    "session_name": session_name,
                    **metrics,
                }
            )

        summary_json = out_dir / "SL_vs_human_only_summary.json"
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)
        write_bagwise_csv(out_dir / "SL_vs_human_only_bagwise.csv", bagwise_rows)
        print(f"[OK] all curve png: {out_dir / 'SL_vs_human_only_all.png'}")
        print(f"[OK] all residual png: {out_dir / 'SL_vs_human_only_residual_all.png'}")
        print(f"[OK] all error png: {out_dir / 'SL_vs_human_only_error_all.png'}")
        return 0
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
