#!/usr/bin/env python3
"""Evaluate the current FSL visual checkpoint against human-labeled rows."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch

from FSL_train_visual_pseudolabel import MinimalVisualRegressor, load_single_image
from SL_supervised_common import group_metrics, read_csv_rows, regression_metrics
from SL_train_baseline import choose_device


DEFAULT_COMPARE_COLUMNS = "slosh_height_mm,peak_rel_mm_v2,center_rel_mm_v2"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate a trained FSL visual pseudolabel checkpoint against human labels "
            "stored in an SL-style manifest."
        )
    )
    parser.add_argument("--manifest-csv", required=True, help="Path to SL_supervised_manifest.csv.")
    parser.add_argument("--checkpoint", required=True, help="Path to FSL_visual_pseudolabel.pt.")
    parser.add_argument("--out-dir", required=True, help="Output directory for metrics and predictions.")
    parser.add_argument(
        "--target-column",
        default="human_peak_mm",
        help="Human label target column. Default: human_peak_mm.",
    )
    parser.add_argument(
        "--compare-columns",
        default=DEFAULT_COMPARE_COLUMNS,
        help="Comma-separated baseline columns to compare against the target.",
    )
    parser.add_argument(
        "--image-column",
        default="",
        help="Manifest image column override. Defaults to checkpoint image_column.",
    )
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N rows. Default: 5.")
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


def collect_metrics(targets: Sequence[float], preds: Sequence[Optional[float]]) -> Dict[str, Optional[float]]:
    filtered_targets: List[float] = []
    filtered_preds: List[float] = []
    for target, pred in zip(targets, preds):
        if pred is None:
            continue
        filtered_targets.append(float(target))
        filtered_preds.append(float(pred))
    summary = regression_metrics(filtered_targets, filtered_preds)
    summary["coverage"] = 0.0 if not targets else float(len(filtered_preds) / len(targets))
    return summary


def build_samples(
    rows: Sequence[Dict[str, str]],
    image_column: str,
    target_column: str,
    image_height: int,
    image_width: int,
) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
    samples: List[Dict[str, object]] = []
    skip_counts = {
        "missing_target": 0,
        "missing_image_path": 0,
        "image_not_found": 0,
        "image_load_error": 0,
    }
    for row in rows:
        target = finite_float(row.get(target_column))
        if target is None:
            skip_counts["missing_target"] += 1
            continue
        image_raw = str(row.get(image_column, "")).strip()
        if image_raw == "":
            skip_counts["missing_image_path"] += 1
            continue
        image_path = Path(image_raw).expanduser()
        if not image_path.exists():
            skip_counts["image_not_found"] += 1
            continue
        try:
            image_tensor = load_single_image(image_path, image_height=image_height, image_width=image_width)
        except (OSError, ValueError):
            skip_counts["image_load_error"] += 1
            continue
        sample = {
            "row_id": str(row.get("row_id", "")),
            "session_id": str(row.get("session_id", "")),
            "session_name": str(row.get("session_name", "")),
            "bag_id": str(row.get("bag_id", "")),
            "frame_index": int(float(row.get("frame_index", -1))),
            "relative_time_s": float(finite_float(row.get("relative_time_s")) or 0.0),
            "image_path": str(image_path),
            "target": float(target),
            "x": image_tensor.astype(np.float32),
        }
        for key in ("slosh_height_mm", "peak_rel_mm_v2", "center_rel_mm_v2"):
            sample[key] = finite_float(row.get(key))
        samples.append(sample)
    return samples, skip_counts


def predict_samples(
    checkpoint: Dict,
    samples: Sequence[Dict[str, object]],
    device: torch.device,
    batch_size: int = 64,
) -> List[float]:
    model = MinimalVisualRegressor(
        image_height=int(checkpoint["image_height"]),
        image_width=int(checkpoint["image_width"]),
        hidden_dim=int(checkpoint["hidden_dim"]),
    ).to(device=device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    image_mean = np.asarray(checkpoint["image_mean"], dtype=np.float32).reshape(1, -1, 1, 1)
    image_std = np.asarray(checkpoint["image_std"], dtype=np.float32).reshape(1, -1, 1, 1)
    target_mean = float(checkpoint["target_mean"])
    target_std = float(checkpoint["target_std"])

    preds: List[float] = []
    with torch.no_grad():
        for start_idx in range(0, len(samples), max(1, int(batch_size))):
            batch = samples[start_idx : start_idx + max(1, int(batch_size))]
            x_np = np.asarray([sample["x"] for sample in batch], dtype=np.float32)
            x_np = ((x_np - image_mean[0]) / image_std[0]).astype(np.float32)
            x_tensor = torch.from_numpy(x_np).to(device=device)
            pred_norm = model(x_tensor).detach().cpu().numpy()
            preds.extend([float(value * target_std + target_mean) for value in pred_norm])
    return preds


def write_predictions_csv(path: Path, rows: Sequence[Dict[str, object]]):
    fieldnames = [
        "row_id",
        "session_id",
        "session_name",
        "bag_id",
        "frame_index",
        "relative_time_s",
        "image_path",
        "target_human_peak_mm",
        "slosh_height_mm",
        "peak_rel_mm_v2",
        "center_rel_mm_v2",
        "pred_visual_fsl",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def write_bagwise_csv(path: Path, rows: Sequence[Dict[str, object]], pred_keys: Sequence[str]):
    with path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["model_name", "bag_id", "count", "mae", "rmse", "bias_mean", "bias_median", "corr"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for pred_key in pred_keys:
            model_name = pred_key
            metric_rows = group_metrics(
                [
                    {
                        "bag_id": row["bag_id"],
                        "target": row["target_human_peak_mm"],
                        pred_key: row.get(pred_key),
                    }
                    for row in rows
                ],
                pred_key,
            )
            for metric_row in metric_rows:
                writer.writerow({"model_name": model_name, **metric_row})


def main() -> int:
    try:
        args = parse_args()
        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        checkpoint_path = Path(args.checkpoint).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        image_column = str(args.image_column).strip() or str(checkpoint.get("image_column", "roi_debug_path"))
        rows = read_csv_rows(manifest_csv)
        compare_columns = [column.strip() for column in str(args.compare_columns).split(",") if column.strip()]

        samples, skip_counts = build_samples(
            rows,
            image_column=image_column,
            target_column=str(args.target_column),
            image_height=int(checkpoint["image_height"]),
            image_width=int(checkpoint["image_width"]),
        )
        if not samples:
            raise RuntimeError("no evaluable rows after image/target filtering")

        device = choose_device(args.device)
        pred_visual = predict_samples(checkpoint, samples, device=device)

        output_rows: List[Dict[str, object]] = []
        for sample, pred in zip(samples, pred_visual):
            output_rows.append(
                {
                    "row_id": sample["row_id"],
                    "session_id": sample["session_id"],
                    "session_name": sample["session_name"],
                    "bag_id": sample["bag_id"],
                    "frame_index": sample["frame_index"],
                    "relative_time_s": sample["relative_time_s"],
                    "image_path": sample["image_path"],
                    "target_human_peak_mm": sample["target"],
                    "slosh_height_mm": sample["slosh_height_mm"],
                    "peak_rel_mm_v2": sample["peak_rel_mm_v2"],
                    "center_rel_mm_v2": sample["center_rel_mm_v2"],
                    "pred_visual_fsl": float(pred),
                }
            )

        targets = [float(row["target_human_peak_mm"]) for row in output_rows]
        summary_metrics: Dict[str, Dict[str, Optional[float]]] = {}
        for column in compare_columns:
            preds = [finite_float(row.get(column)) for row in output_rows]
            summary_metrics[column] = collect_metrics(targets, preds)
        summary_metrics["pred_visual_fsl"] = collect_metrics(
            targets,
            [float(row["pred_visual_fsl"]) for row in output_rows],
        )

        predictions_csv = out_dir / "FSL_visual_vs_human_predictions.csv"
        bagwise_csv = out_dir / "FSL_visual_vs_human_bagwise.csv"
        summary_json = out_dir / "FSL_visual_vs_human_summary.json"
        write_predictions_csv(predictions_csv, output_rows)
        write_bagwise_csv(bagwise_csv, output_rows, list(compare_columns) + ["pred_visual_fsl"])

        summary = {
            "manifest_csv": str(manifest_csv),
            "checkpoint": str(checkpoint_path),
            "out_dir": str(out_dir),
            "target_column": str(args.target_column),
            "image_column": image_column,
            "num_rows": len(output_rows),
            "skip_counts": skip_counts,
            "metrics": summary_metrics,
            "predictions_csv": str(predictions_csv),
            "bagwise_csv": str(bagwise_csv),
        }
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] predictions csv: {predictions_csv}")
        print(f"[OK] bagwise csv: {bagwise_csv}")
        print(f"[OK] summary json: {summary_json}")
        print(f"[OK] rows: {len(output_rows)} | image_column={image_column} | skip_counts={skip_counts}")
        for model_name, metrics in summary_metrics.items():
            mae_text = "nan" if metrics["mae"] is None else f"{metrics['mae']:.6f}"
            rmse_text = "nan" if metrics["rmse"] is None else f"{metrics['rmse']:.6f}"
            corr_text = "nan" if metrics["corr"] is None else f"{metrics['corr']:.6f}"
            cov_text = "nan" if metrics.get("coverage") is None else f"{metrics['coverage']:.3f}"
            print(f"[OK] {model_name}: mae={mae_text} rmse={rmse_text} corr={corr_text} coverage={cov_text}")
        show_first = max(0, int(args.show_first))
        for row in output_rows[:show_first]:
            print(
                "[OK] row={row_id} bag={bag_id} frame={frame_index} target={target_human_peak_mm:.6f} "
                "visual={pred_visual_fsl:.6f} slosh={slosh_height_mm}".format(**row)
            )
        return 0
    except RuntimeError as exc:
        print(f"[WARN] {exc}")
        return 1
    except KeyboardInterrupt:
        print("\n[INFO] interrupted by Ctrl+C")
        return 130


if __name__ == "__main__":
    sys.exit(main())
