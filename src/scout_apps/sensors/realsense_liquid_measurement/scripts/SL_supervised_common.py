#!/usr/bin/env python3
"""Shared utilities for the SL supervised-learning pipeline."""

import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


FEATURE_NAMES = (
    "v_mps",
    "omega_radps",
    "ax_cmd_mps2",
    "ay_model_mps2",
    "imu_ax_mps2",
    "imu_ay_mps2",
)

TASK_TO_TARGET_COLUMN = {
    "peak": "human_peak_mm",
    "center": "human_height_mm",
}

TASK_TO_PRIMARY_BASELINE = {
    "peak": "peak_rel_mm_v2",
    "center": "center_rel_mm_v2",
}

TASK_TO_PROXY_BASELINE = {
    "peak": "slosh_height_mm",
    "center": "",
}


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


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"csv has no rows: {path}")
    return rows


def load_json(path: Path) -> Dict:
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise RuntimeError(f"json root is not an object: {path}")
    return data


def target_column_for_task(task: str) -> str:
    if task not in TASK_TO_TARGET_COLUMN:
        raise RuntimeError(f"unsupported task: {task}")
    return TASK_TO_TARGET_COLUMN[task]


def baseline_columns_for_task(task: str) -> Tuple[str, str]:
    if task not in TASK_TO_PRIMARY_BASELINE:
        raise RuntimeError(f"unsupported task: {task}")
    return TASK_TO_PRIMARY_BASELINE[task], TASK_TO_PROXY_BASELINE[task]


def split_groups(split_json: Dict, split_name: str) -> Tuple[str, List[str]]:
    splits = split_json.get("splits", {})
    split_entry = splits.get(split_name, {})
    if not isinstance(split_entry, dict):
        raise RuntimeError(f"missing split entry: {split_name}")
    groups = split_entry.get("groups", [])
    if not isinstance(groups, list):
        raise RuntimeError(f"invalid groups list for split: {split_name}")
    group_key = str(split_json.get("group_key", "")).strip()
    if not group_key:
        raise RuntimeError("split json missing group_key")
    return group_key, [str(group) for group in groups]


def filter_rows_for_split(rows: Sequence[Dict[str, str]], split_json: Dict, split_name: str) -> List[Dict[str, str]]:
    group_key, groups = split_groups(split_json, split_name)
    group_set = set(groups)
    filtered = [dict(row) for row in rows if str(row.get(group_key, "")) in group_set]
    if not filtered:
        raise RuntimeError(f"no manifest rows matched split={split_name}")
    return filtered


def nearest_finite_index(values: np.ndarray, start_idx: int, direction: int) -> Optional[int]:
    idx = int(start_idx)
    n = int(values.shape[0])
    while 0 <= idx < n:
        if math.isfinite(float(values[idx])):
            return idx
        idx += int(direction)
    return None


def finite_difference(values: np.ndarray, times: np.ndarray) -> np.ndarray:
    derivatives = np.full(values.shape, np.nan, dtype=np.float64)
    n = int(values.shape[0])
    for idx in range(n):
        if not math.isfinite(float(times[idx])):
            continue
        left_idx = nearest_finite_index(values, idx - 1, -1)
        right_idx = nearest_finite_index(values, idx + 1, +1)
        if left_idx is not None and right_idx is not None:
            dt = float(times[right_idx] - times[left_idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[right_idx] - values[left_idx]) / dt
                continue
        if right_idx is not None and math.isfinite(float(values[idx])):
            dt = float(times[right_idx] - times[idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[right_idx] - values[idx]) / dt
                continue
        if left_idx is not None and math.isfinite(float(values[idx])):
            dt = float(times[idx] - times[left_idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[idx] - values[left_idx]) / dt
                continue
    return derivatives


def sort_rows_for_sequence(rows: Sequence[Dict[str, str]]) -> List[Dict[str, str]]:
    def sort_key(row: Dict[str, str]):
        rel_t = finite_float(row.get("relative_time_s"))
        if rel_t is None:
            rel_t = float("inf")
        frame_index = finite_float(row.get("frame_index"))
        if frame_index is None:
            frame_index = float("inf")
        return (float(rel_t), float(frame_index))

    return sorted((dict(row) for row in rows), key=sort_key)


def build_feature_table(rows: Sequence[Dict[str, str]]) -> Dict[str, np.ndarray]:
    relative_time_s = np.asarray([finite_float(row.get("relative_time_s")) or math.nan for row in rows], dtype=np.float64)
    stamp = np.asarray([finite_float(row.get("stamp")) or math.nan for row in rows], dtype=np.float64)
    frame_index = np.asarray([int((finite_float(row.get("frame_index")) or -1)) for row in rows], dtype=np.int32)

    v_mps = np.asarray([finite_float(row.get("cmd_vel_linear_x")) or math.nan for row in rows], dtype=np.float64)
    omega_radps = np.asarray([finite_float(row.get("cmd_vel_angular_z")) or math.nan for row in rows], dtype=np.float64)
    imu_ax_mps2 = np.asarray([finite_float(row.get("imu_ax")) or math.nan for row in rows], dtype=np.float64)
    imu_ay_mps2 = np.asarray([finite_float(row.get("imu_ay")) or math.nan for row in rows], dtype=np.float64)

    ax_cmd_mps2 = finite_difference(v_mps, relative_time_s)
    ay_model_mps2 = np.full(v_mps.shape, np.nan, dtype=np.float64)
    finite_mask = np.isfinite(v_mps) & np.isfinite(omega_radps)
    ay_model_mps2[finite_mask] = v_mps[finite_mask] * omega_radps[finite_mask]

    return {
        "frame_index": frame_index,
        "stamp": stamp,
        "relative_time_s": relative_time_s,
        "v_mps": v_mps,
        "omega_radps": omega_radps,
        "ax_cmd_mps2": ax_cmd_mps2,
        "ay_model_mps2": ay_model_mps2,
        "imu_ax_mps2": imu_ax_mps2,
        "imu_ay_mps2": imu_ay_mps2,
    }


def sequence_groups(rows: Sequence[Dict[str, str]], sequence_key: str = "session_id") -> Dict[str, List[Dict[str, str]]]:
    grouped: Dict[str, List[Dict[str, str]]] = defaultdict(list)
    for row in rows:
        key = str(row.get(sequence_key, "")).strip()
        if key == "":
            raise RuntimeError(f"row missing sequence key {sequence_key}: {row.get('row_id', '<no-row-id>')}")
        grouped[key].append(dict(row))
    return {key: sort_rows_for_sequence(group_rows) for key, group_rows in grouped.items()}


def build_window_samples_from_columns(
    rows: Sequence[Dict[str, str]],
    target_column: str,
    primary_baseline_column: str,
    proxy_baseline_column: str,
    history_frames: int,
    stride: int,
    max_window_gap_sec: float,
    sequence_key: str = "session_id",
) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
    grouped_rows = sequence_groups(rows, sequence_key=sequence_key)

    history_frames = max(1, int(history_frames))
    stride = max(1, int(stride))
    required_history = (history_frames - 1) * stride
    max_window_gap_sec = max(0.0, float(max_window_gap_sec))

    samples: List[Dict[str, object]] = []
    skip_counts = Counter()

    for _, sequence_rows in grouped_rows.items():
        feature_table = build_feature_table(sequence_rows)
        relative_time_s = feature_table["relative_time_s"]
        feature_matrix = np.stack([feature_table[name] for name in FEATURE_NAMES], axis=1)

        for end_idx, row in enumerate(sequence_rows):
            target_value = finite_float(row.get(target_column))
            if target_value is None:
                skip_counts["missing_target"] += 1
                continue
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

            samples.append(
                {
                    "x": window_features.astype(np.float32),
                    "y": float(target_value),
                    "row_id": str(row.get("row_id", "")),
                    "session_id": str(row.get("session_id", "")),
                    "bag_id": str(row.get("bag_id", "")),
                    "frame_index": int(float(row.get("frame_index", -1))),
                    "stamp": float(feature_table["stamp"][end_idx]),
                    "relative_time_s": float(relative_time_s[end_idx]),
                    "target_column": target_column,
                    "baseline_b0": None if not primary_baseline_column else finite_float(row.get(primary_baseline_column)),
                    "baseline_b1": None if not proxy_baseline_column else finite_float(row.get(proxy_baseline_column)),
                }
            )

    return samples, dict(skip_counts)


def build_window_samples(
    rows: Sequence[Dict[str, str]],
    task: str,
    history_frames: int,
    stride: int,
    max_window_gap_sec: float,
    sequence_key: str = "session_id",
) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
    target_column = target_column_for_task(task)
    primary_baseline, proxy_baseline = baseline_columns_for_task(task)
    return build_window_samples_from_columns(
        rows=rows,
        target_column=target_column,
        primary_baseline_column=primary_baseline,
        proxy_baseline_column=proxy_baseline,
        history_frames=history_frames,
        stride=stride,
        max_window_gap_sec=max_window_gap_sec,
        sequence_key=sequence_key,
    )


def feature_stats_from_samples(samples: Sequence[Dict[str, object]]) -> Tuple[np.ndarray, np.ndarray]:
    x = np.asarray([sample["x"] for sample in samples], dtype=np.float32)
    mean = np.mean(x, axis=(0, 1), dtype=np.float64).astype(np.float32)
    std = np.std(x, axis=(0, 1), dtype=np.float64).astype(np.float32)
    std[std < 1e-6] = 1.0
    return mean, std


def normalize_samples(samples: Sequence[Dict[str, object]], feature_mean: np.ndarray, feature_std: np.ndarray) -> List[Dict[str, object]]:
    normalized: List[Dict[str, object]] = []
    for sample in samples:
        updated = dict(sample)
        updated["x"] = ((np.asarray(sample["x"], dtype=np.float32) - feature_mean) / feature_std).astype(np.float32)
        normalized.append(updated)
    return normalized


def regression_metrics(targets: Sequence[float], preds: Sequence[float]) -> Dict[str, Optional[float]]:
    if not targets:
        return {
            "count": 0,
            "mae": None,
            "rmse": None,
            "bias_mean": None,
            "bias_median": None,
            "corr": None,
        }
    target_np = np.asarray(targets, dtype=np.float64)
    pred_np = np.asarray(preds, dtype=np.float64)
    residuals = pred_np - target_np
    corr = None
    if target_np.size >= 2:
        target_std = float(np.std(target_np))
        pred_std = float(np.std(pred_np))
        if target_std > 0.0 and pred_std > 0.0:
            corr = float(np.corrcoef(target_np, pred_np)[0, 1])
    return {
        "count": int(target_np.size),
        "mae": float(np.mean(np.abs(residuals))),
        "rmse": float(np.sqrt(np.mean(np.square(residuals)))),
        "bias_mean": float(np.mean(residuals)),
        "bias_median": float(np.median(residuals)),
        "corr": corr,
    }


def group_metrics(rows: Sequence[Dict[str, object]], pred_key: str) -> List[Dict[str, object]]:
    grouped_targets: Dict[str, List[float]] = defaultdict(list)
    grouped_preds: Dict[str, List[float]] = defaultdict(list)
    for row in rows:
        pred_value = row.get(pred_key)
        if pred_value is None:
            continue
        bag_id = str(row.get("bag_id", ""))
        grouped_targets[bag_id].append(float(row["target"]))
        grouped_preds[bag_id].append(float(pred_value))
    metrics: List[Dict[str, object]] = []
    for bag_id in sorted(grouped_targets.keys()):
        summary = regression_metrics(grouped_targets[bag_id], grouped_preds[bag_id])
        summary["bag_id"] = bag_id
        metrics.append(summary)
    return metrics
