#!/usr/bin/env python3
"""Plot only SL-visual vs /slosh/height curves and summarize their human-label errors."""

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
        description=(
            "Plot only current SL visual output and bag /slosh/height from an SL predictions CSV, "
            "and write a focused summary relative to human_peak_mm."
        )
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
    bias_median = float(sorted_errors[len(sorted_errors) // 2])
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
        "model_name",
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


def summarize_by_bin(rows: Sequence[Dict[str, str]]) -> Dict[str, Dict[str, object]]:
    bins = [
        ("low_<=0.2", lambda target: target <= 0.2),
        ("mid_0.2_0.5", lambda target: 0.2 < target <= 0.5),
        ("high_>0.5", lambda target: target > 0.5),
    ]
    summary: Dict[str, Dict[str, object]] = {}
    for name, predicate in bins:
        targets = []
        slosh = []
        visual = []
        for row in rows:
            target = finite_float(row.get("target_human_peak_mm"))
            slosh_pred = finite_float(row.get("baseline_slosh_height_mm"))
            visual_pred = finite_float(row.get("pred_visual"))
            if target is None or slosh_pred is None or visual_pred is None:
                continue
            if not predicate(float(target)):
                continue
            targets.append(float(target))
            slosh.append(float(slosh_pred))
            visual.append(float(visual_pred))
        if not targets:
            continue
        summary[name] = {
            "count": len(targets),
            "slosh_raw": regression_metrics(targets, slosh),
            "visual": regression_metrics(targets, visual),
        }
    return summary


def summarize_rowwise_wins(rows: Sequence[Dict[str, str]]) -> Dict[str, object]:
    total = 0
    visual_better = 0
    slosh_better = 0
    equal = 0
    gains = []
    for row in rows:
        target = finite_float(row.get("target_human_peak_mm"))
        slosh_pred = finite_float(row.get("baseline_slosh_height_mm"))
        visual_pred = finite_float(row.get("pred_visual"))
        if target is None or slosh_pred is None or visual_pred is None:
            continue
        total += 1
        slosh_error = abs(float(slosh_pred) - float(target))
        visual_error = abs(float(visual_pred) - float(target))
        gains.append(float(slosh_error - visual_error))
        if visual_error + 1e-12 < slosh_error:
            visual_better += 1
        elif slosh_error + 1e-12 < visual_error:
            slosh_better += 1
        else:
            equal += 1
    gains_sorted = sorted(gains)
    return {
        "count": total,
        "visual_better_rows": visual_better,
        "slosh_better_rows": slosh_better,
        "equal_rows": equal,
        "visual_better_ratio": None if total == 0 else float(visual_better / total),
        "slosh_better_ratio": None if total == 0 else float(slosh_better / total),
        "mean_abs_error_gain_mm": None if total == 0 else float(sum(gains) / total),
        "median_abs_error_gain_mm": None if total == 0 else float(gains_sorted[len(gains_sorted) // 2]),
    }


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
        slosh_values = [finite_float(row.get("baseline_slosh_height_mm")) for row in rows]
        visual_values = [finite_float(row.get("pred_visual")) for row in rows]
        axis.plot(time_values, slosh_values, color="#5b8fd1", linewidth=1.6, label="bag /slosh/height")
        axis.plot(time_values, visual_values, color="#d9485f", linewidth=1.6, label="SL visual")
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
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        rows = read_rows(predictions_csv)
        selected_splits = [text.strip() for text in str(args.splits).split(",") if text.strip()]
        filtered_rows = [row for row in rows if str(row.get("split", "")) in selected_splits]
        if not filtered_rows:
            raise RuntimeError(f"no rows matched splits={selected_splits}")

        grouped = group_rows(filtered_rows)
        all_png = out_dir / "SL_vs_slosh_only_all.png"
        make_plot(all_png, grouped, selected_splits)
        for split_name in selected_splits:
            split_png = out_dir / f"SL_vs_slosh_only_{split_name}.png"
            make_plot(split_png, grouped, [split_name])

        summary: Dict[str, object] = {
            "predictions_csv": str(predictions_csv),
            "selected_splits": selected_splits,
            "num_rows": len(filtered_rows),
        }

        summary["overall"] = {}
        bagwise_rows: List[Dict[str, object]] = []
        for split_name in selected_splits:
            split_rows = [row for row in filtered_rows if str(row.get("split", "")) == split_name]
            targets = [float(row["target_human_peak_mm"]) for row in split_rows]
            slosh = [float(row["baseline_slosh_height_mm"]) for row in split_rows]
            visual = [float(row["pred_visual"]) for row in split_rows]
            summary["overall"][split_name] = {
                "slosh_raw": regression_metrics(targets, slosh),
                "visual": regression_metrics(targets, visual),
                "rowwise_win_stats": summarize_rowwise_wins(split_rows),
            }

            split_grouped = group_rows(split_rows)
            for (group_split, session_name, bag_id), group_rows_rows in split_grouped.items():
                group_targets = [float(row["target_human_peak_mm"]) for row in group_rows_rows]
                group_slosh = [float(row["baseline_slosh_height_mm"]) for row in group_rows_rows]
                group_visual = [float(row["pred_visual"]) for row in group_rows_rows]
                bagwise_rows.append(
                    {
                        "split": group_split,
                        "bag_id": bag_id,
                        "session_name": session_name,
                        "model_name": "slosh_raw",
                        **regression_metrics(group_targets, group_slosh),
                    }
                )
                bagwise_rows.append(
                    {
                        "split": group_split,
                        "bag_id": bag_id,
                        "session_name": session_name,
                        "model_name": "visual",
                        **regression_metrics(group_targets, group_visual),
                    }
                )

        test_rows = [row for row in filtered_rows if str(row.get("split", "")) == "test"]
        if test_rows:
            summary["test_target_bins"] = summarize_by_bin(test_rows)

        summary_json = out_dir / "SL_vs_slosh_only_summary.json"
        bagwise_csv = out_dir / "SL_vs_slosh_only_bagwise.csv"
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)
        write_bagwise_csv(bagwise_csv, bagwise_rows)

        print(f"[OK] combined png: {all_png}")
        for split_name in selected_splits:
            print(f"[OK] split png: {out_dir / f'SL_vs_slosh_only_{split_name}.png'}")
        print(f"[OK] summary json: {summary_json}")
        print(f"[OK] bagwise csv: {bagwise_csv}")
        return 0
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
