#!/usr/bin/env python3
"""Run an SL checkpoint on a full debug session and export predictions."""

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch

from SL_supervised_common import (
    FEATURE_NAMES,
    baseline_columns_for_task,
    build_feature_table,
    finite_float,
    load_json,
    read_csv_rows,
    sequence_groups,
)
from SL_train_baseline import build_model_from_checkpoint, choose_device, samples_to_model_array


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a trained SL checkpoint on a debug session and export frame-wise predictions."
    )
    parser.add_argument("--debug-dir", required=True, help="Debug-session directory containing debug_session.csv/json.")
    parser.add_argument("--checkpoint", required=True, help="Path to a trained SL checkpoint (.pt).")
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to <debug_dir>/SL_infer_<task>.",
    )
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N predictions. Default: 5.")
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def infer_session_identity(debug_dir: Path, session_meta: Dict) -> Tuple[str, str]:
    bag_raw = str(session_meta.get("bag", "")).strip()
    if bag_raw:
        bag_id = Path(bag_raw).stem
    else:
        bag_id = debug_dir.name
    date_id = debug_dir.parent.name
    session_id = f"{date_id}/{debug_dir.name}" if date_id else debug_dir.name
    return session_id, bag_id


def build_rows_for_inference(debug_dir: Path) -> Tuple[List[Dict[str, str]], Dict]:
    debug_dir = debug_dir.expanduser().resolve()
    session_csv = debug_dir / "debug_session.csv"
    session_json = debug_dir / "debug_session.json"
    rows = read_csv_rows(session_csv)
    session_meta = load_json(session_json)
    session_id, bag_id = infer_session_identity(debug_dir, session_meta)
    prepared: List[Dict[str, str]] = []
    for raw in rows:
        row = dict(raw)
        frame_index = int(float(row.get("frame_index", -1)))
        row["row_id"] = f"{session_id}:{frame_index}"
        row["session_id"] = session_id
        row["bag_id"] = bag_id
        prepared.append(row)
    summary = {
        "bag": str(session_meta.get("bag", "")),
        "calibration": str(session_meta.get("calibration", "")),
        "config": str(session_meta.get("config", "")),
        "image_topic": str(session_meta.get("image_topic", "")),
    }
    return prepared, summary


def build_inference_windows(
    rows: Sequence[Dict[str, str]],
    task: str,
    history_frames: int,
    stride: int,
    max_window_gap_sec: float,
) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
    primary_b0, proxy_b1 = baseline_columns_for_task(task)
    grouped_rows = sequence_groups(rows, sequence_key="session_id")
    history_frames = max(1, int(history_frames))
    stride = max(1, int(stride))
    required_history = (history_frames - 1) * stride
    max_window_gap_sec = max(0.0, float(max_window_gap_sec))

    from collections import Counter

    skip_counts = Counter()
    windows: List[Dict[str, object]] = []

    for _, sequence_rows in grouped_rows.items():
        feature_table = build_feature_table(sequence_rows)
        relative_time_s = feature_table["relative_time_s"]
        feature_matrix = np.stack([feature_table[name] for name in FEATURE_NAMES], axis=1)

        for end_idx, row in enumerate(sequence_rows):
            if end_idx < required_history:
                skip_counts["insufficient_history"] += 1
                continue

            window_indices = list(range(end_idx - required_history, end_idx + 1, stride))
            if len(window_indices) != history_frames:
                skip_counts["window_length_mismatch"] += 1
                continue

            window_times = relative_time_s[window_indices]
            if not np.all(np.isfinite(window_times)):
                skip_counts["nonfinite_window_time"] += 1
                continue
            if history_frames >= 2 and max_window_gap_sec > 0.0 and np.any(np.diff(window_times) > max_window_gap_sec):
                skip_counts["window_gap_too_large"] += 1
                continue

            window_features = feature_matrix[window_indices, :]
            if not np.all(np.isfinite(window_features)):
                skip_counts["nonfinite_window_feature"] += 1
                continue

            windows.append(
                {
                    "x": window_features.astype(np.float32),
                    "row_id": str(row.get("row_id", "")),
                    "bag_id": str(row.get("bag_id", "")),
                    "frame_index": int(float(row.get("frame_index", -1))),
                    "relative_time_s": float(relative_time_s[end_idx]),
                    "target": finite_float(row.get("human_peak_mm" if task == "peak" else "human_height_mm")),
                    "baseline_b0": finite_float(row.get(primary_b0)),
                    "baseline_b1": None if not proxy_b1 else finite_float(row.get(proxy_b1)),
                }
            )
    return windows, dict(skip_counts)


def predict_windows(checkpoint: Dict, windows: Sequence[Dict[str, object]], device: torch.device) -> List[float]:
    if not windows:
        return []
    feature_mean = np.asarray(checkpoint["feature_mean"], dtype=np.float32)
    feature_std = np.asarray(checkpoint["feature_std"], dtype=np.float32)
    normalized_windows = []
    for window in windows:
        x = np.asarray(window["x"], dtype=np.float32)
        updated = dict(window)
        updated["x"] = ((x - feature_mean) / feature_std).astype(np.float32)
        normalized_windows.append(updated)
    model_type = str(checkpoint["model_type"])
    x_np = samples_to_model_array(normalized_windows, model_type=model_type)

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
        "baseline_b0",
        "baseline_b1",
        "pred_b2",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def main() -> int:
    try:
        args = parse_args()
        debug_dir = Path(args.debug_dir).expanduser().resolve()
        checkpoint_path = Path(args.checkpoint).expanduser().resolve()
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        task = str(checkpoint.get("task", "peak"))
        out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else (debug_dir / f"SL_infer_{task}").resolve()
        ensure_dir(out_dir)

        rows, session_meta = build_rows_for_inference(debug_dir)
        windows, skip_counts = build_inference_windows(
            rows,
            task=task,
            history_frames=int(checkpoint["history_frames"]),
            stride=int(checkpoint["stride"]),
            max_window_gap_sec=float(checkpoint["max_window_gap_sec"]),
        )
        if not windows:
            raise RuntimeError("no inference windows could be constructed for this debug session")

        device = choose_device(args.device)
        preds = predict_windows(checkpoint, windows, device=device)
        output_rows: List[Dict[str, object]] = []
        for window, pred in zip(windows, preds):
            output_rows.append(
                {
                    "row_id": window["row_id"],
                    "bag_id": window["bag_id"],
                    "frame_index": window["frame_index"],
                    "relative_time_s": window["relative_time_s"],
                    "target": window["target"],
                    "baseline_b0": window["baseline_b0"],
                    "baseline_b1": window["baseline_b1"],
                    "pred_b2": pred,
                }
            )

        predictions_csv = out_dir / f"SL_infer_predictions_{task}.csv"
        summary_json = out_dir / f"SL_infer_summary_{task}.json"
        write_predictions_csv(predictions_csv, output_rows)
        summary = {
            "debug_dir": str(debug_dir),
            "checkpoint": str(checkpoint_path),
            "task": task,
            "model_type": str(checkpoint.get("model_type", "")),
            "num_windows": len(windows),
            "skip_counts": skip_counts,
            "session_meta": session_meta,
            "predictions_csv": str(predictions_csv),
        }
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] predictions csv: {predictions_csv}")
        print(f"[OK] summary json: {summary_json}")
        print(f"[OK] windows: {len(windows)} | skip_counts={skip_counts}")
        show_first = max(0, int(args.show_first))
        for row in output_rows[:show_first]:
            print(
                "[OK] row={row_id} frame={frame_index} pred_b2={pred_b2:.6f} target={target}".format(
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
