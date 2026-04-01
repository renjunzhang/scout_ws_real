#!/usr/bin/env python3
"""Evaluate B0/B1/B2 baselines on an SL manifest split."""

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np
import torch

from SL_supervised_common import (
    FEATURE_NAMES,
    baseline_columns_for_task,
    build_window_samples,
    filter_rows_for_split,
    group_metrics,
    load_json,
    normalize_samples,
    read_csv_rows,
    regression_metrics,
    target_column_for_task,
)
from SL_train_baseline import build_model_from_checkpoint, choose_device, samples_to_model_array


def parse_args():
    parser = argparse.ArgumentParser(
        description="Evaluate B0/B1/B2 baselines on train/val/test manifest splits."
    )
    parser.add_argument("--manifest-csv", required=True, help="Path to SL_supervised_manifest.csv.")
    parser.add_argument("--split-json", required=True, help="Path to SL_supervised_splits.json.")
    parser.add_argument("--out-dir", required=True, help="Output directory for evaluation artifacts.")
    parser.add_argument("--task", choices=("peak", "center"), default="peak", help="Evaluation task. Default: peak.")
    parser.add_argument("--split", choices=("train", "val", "test"), default="val", help="Split name. Default: val.")
    parser.add_argument(
        "--checkpoint",
        default="",
        help="Optional trained SL checkpoint (.pt). If omitted, only B0/B1 are evaluated.",
    )
    parser.add_argument("--history-frames", type=int, default=30, help="Used when no checkpoint is provided.")
    parser.add_argument("--stride", type=int, default=1, help="Used when no checkpoint is provided.")
    parser.add_argument("--max-window-gap-sec", type=float, default=0.20, help="Used when no checkpoint is provided.")
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N evaluated rows. Default: 5.")
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def predict_with_checkpoint(
    checkpoint: Dict,
    samples: Sequence[Dict[str, object]],
    device: torch.device,
) -> List[float]:
    if not samples:
        return []
    feature_mean = np.asarray(checkpoint["feature_mean"], dtype=np.float32)
    feature_std = np.asarray(checkpoint["feature_std"], dtype=np.float32)
    normalized_samples = normalize_samples(samples, feature_mean, feature_std)
    model_type = str(checkpoint["model_type"])
    x_np = samples_to_model_array(normalized_samples, model_type=model_type)

    model = build_model_from_checkpoint(checkpoint).to(device=device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    with torch.no_grad():
        pred_norm = model(torch.from_numpy(x_np).to(device=device)).detach().cpu().numpy()
    return [
        float(value * float(checkpoint["target_std"]) + float(checkpoint["target_mean"]))
        for value in pred_norm
    ]


def write_predictions_csv(path: Path, rows: Sequence[Dict[str, object]]):
    fieldnames = [
        "row_id",
        "bag_id",
        "frame_index",
        "relative_time_s",
        "target",
        "pred_b0",
        "pred_b1",
        "pred_b2",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def write_bagwise_csv(path: Path, bagwise_by_model: Dict[str, Sequence[Dict[str, object]]]):
    fieldnames = ["model_name", "bag_id", "count", "mae", "rmse", "bias_mean", "bias_median", "corr"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for model_name, rows in bagwise_by_model.items():
            for row in rows:
                writer.writerow({"model_name": model_name, **row})


def main() -> int:
    try:
        args = parse_args()
        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        split_json_path = Path(args.split_json).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        manifest_rows = read_csv_rows(manifest_csv)
        split_json = load_json(split_json_path)
        eval_rows = filter_rows_for_split(manifest_rows, split_json, args.split)

        checkpoint = None
        if args.checkpoint:
            checkpoint = torch.load(
                Path(args.checkpoint).expanduser().resolve(),
                map_location="cpu",
                weights_only=False,
            )
            if str(checkpoint.get("task")) != str(args.task):
                raise RuntimeError(
                    f"checkpoint task mismatch: checkpoint={checkpoint.get('task')} requested={args.task}"
                )
            history_frames = int(checkpoint["history_frames"])
            stride = int(checkpoint["stride"])
            max_window_gap_sec = float(checkpoint["max_window_gap_sec"])
        else:
            history_frames = int(args.history_frames)
            stride = int(args.stride)
            max_window_gap_sec = float(args.max_window_gap_sec)

        samples, skip_counts = build_window_samples(
            eval_rows,
            task=args.task,
            history_frames=history_frames,
            stride=stride,
            max_window_gap_sec=max_window_gap_sec,
        )
        if not samples:
            raise RuntimeError(f"no evaluable samples for split={args.split} task={args.task}")

        device = choose_device(args.device)
        primary_b0, proxy_b1 = baseline_columns_for_task(args.task)
        target_column = target_column_for_task(args.task)

        model_preds: List[Optional[float]]
        if checkpoint is not None:
            model_preds = predict_with_checkpoint(checkpoint, samples, device=device)
        else:
            model_preds = [None] * len(samples)

        prediction_rows: List[Dict[str, object]] = []
        target_values = []
        pred_b0 = []
        pred_b1 = []
        pred_b2 = []

        for sample, model_pred in zip(samples, model_preds):
            row = {
                "row_id": str(sample["row_id"]),
                "bag_id": str(sample["bag_id"]),
                "frame_index": int(sample["frame_index"]),
                "relative_time_s": float(sample["relative_time_s"]),
                "target": float(sample["y"]),
                "pred_b0": sample["baseline_b0"],
                "pred_b1": sample["baseline_b1"],
                "pred_b2": model_pred,
            }
            prediction_rows.append(row)
            target_values.append(float(sample["y"]))
            pred_b0.append(sample["baseline_b0"])
            pred_b1.append(sample["baseline_b1"])
            pred_b2.append(model_pred)

        def collect_metrics(preds: Sequence[Optional[float]]) -> Dict[str, Optional[float]]:
            filtered_targets = []
            filtered_preds = []
            for target, pred in zip(target_values, preds):
                if pred is None:
                    continue
                filtered_targets.append(float(target))
                filtered_preds.append(float(pred))
            summary = regression_metrics(filtered_targets, filtered_preds)
            summary["coverage"] = 0.0 if not target_values else float(len(filtered_preds) / len(target_values))
            return summary

        summary_by_model = {
            "B0": collect_metrics(pred_b0),
            "B1": collect_metrics(pred_b1) if proxy_b1 else None,
            "B2": collect_metrics(pred_b2) if checkpoint is not None else None,
        }

        bagwise_by_model = {
            "B0": group_metrics(prediction_rows, "pred_b0"),
            "B1": group_metrics(prediction_rows, "pred_b1") if proxy_b1 else [],
            "B2": group_metrics(prediction_rows, "pred_b2") if checkpoint is not None else [],
        }

        predictions_csv = out_dir / f"SL_eval_predictions_{args.task}_{args.split}.csv"
        summary_json = out_dir / f"SL_eval_summary_{args.task}_{args.split}.json"
        bagwise_csv = out_dir / f"SL_eval_bagwise_{args.task}_{args.split}.csv"

        write_predictions_csv(predictions_csv, prediction_rows)
        write_bagwise_csv(bagwise_csv, bagwise_by_model)

        summary = {
            "manifest_csv": str(manifest_csv),
            "split_json": str(split_json_path),
            "task": str(args.task),
            "split": str(args.split),
            "target_column": str(target_column),
            "primary_b0_column": str(primary_b0),
            "proxy_b1_column": str(proxy_b1),
            "history_frames": history_frames,
            "stride": stride,
            "max_window_gap_sec": max_window_gap_sec,
            "num_samples": len(samples),
            "skip_counts": skip_counts,
            "checkpoint": str(args.checkpoint) if args.checkpoint else "",
            "summary_by_model": summary_by_model,
        }
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] predictions csv: {predictions_csv}")
        print(f"[OK] bagwise csv: {bagwise_csv}")
        print(f"[OK] summary json: {summary_json}")
        print(f"[OK] samples: {len(samples)} | skip_counts={skip_counts}")
        print(f"[OK] B0 metrics: {summary_by_model['B0']}")
        if summary_by_model["B1"] is not None:
            print(f"[OK] B1 metrics: {summary_by_model['B1']}")
        if summary_by_model["B2"] is not None:
            print(f"[OK] B2 metrics: {summary_by_model['B2']}")

        show_first = max(0, int(args.show_first))
        for row in prediction_rows[:show_first]:
            print(
                "[OK] row={row_id} target={target:.6f} b0={pred_b0} b1={pred_b1} b2={pred_b2}".format(
                    **row
                )
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
