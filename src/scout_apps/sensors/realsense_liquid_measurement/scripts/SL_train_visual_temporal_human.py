#!/usr/bin/env python3
"""Train a minimal temporal visual regressor from ROI image sequences to human peak labels."""

import argparse
import bisect
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

from FSL_train_visual_pseudolabel import load_single_image
from SL_supervised_common import filter_rows_for_split, group_metrics, load_json, read_csv_rows, regression_metrics
from SL_train_baseline import choose_device, set_seed


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Train a minimal temporal visual regressor from recent K ROI frames to human_peak_mm, "
            "using existing manifest/split artifacts."
        )
    )
    parser.add_argument("--manifest-csv", required=True, help="Path to SL_supervised_manifest.csv.")
    parser.add_argument("--split-json", required=True, help="Path to split json with train/val/test bag ids.")
    parser.add_argument("--out-dir", required=True, help="Output directory for checkpoint and logs.")
    parser.add_argument(
        "--target-column",
        default="human_peak_mm",
        help="Human target column. Default: human_peak_mm.",
    )
    parser.add_argument(
        "--image-column",
        default="roi_debug_path",
        help="Manifest image column to use. Default: roi_debug_path.",
    )
    parser.add_argument("--history-frames", type=int, default=5, help="Number of recent ROI frames. Default: 5.")
    parser.add_argument("--history-step", type=int, default=1, help="Frame step between sequence elements. Default: 1.")
    parser.add_argument("--image-height", type=int, default=96, help="Resized image height. Default: 96.")
    parser.add_argument("--image-width", type=int, default=192, help="Resized image width. Default: 192.")
    parser.add_argument("--hidden-dim", type=int, default=64, help="Frame embedding / MLP hidden dim. Default: 64.")
    parser.add_argument(
        "--temporal-head",
        choices=("mean", "gru", "tcn"),
        default="mean",
        help="Temporal aggregation head. Default: mean.",
    )
    parser.add_argument(
        "--anchor-current-frame",
        action="store_true",
        help=(
            "Concatenate the current-frame feature with the temporal summary before regression. "
            "Recommended when the label is still current-frame peak."
        ),
    )
    parser.add_argument(
        "--temporal-kernel-size",
        type=int,
        default=3,
        help="Kernel size for the TCN head. Used only when --temporal-head=tcn. Default: 3.",
    )
    parser.add_argument("--epochs", type=int, default=60, help="Number of training epochs. Default: 60.")
    parser.add_argument("--batch-size", type=int, default=16, help="Mini-batch size. Default: 16.")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate. Default: 1e-3.")
    parser.add_argument("--weight-decay", type=float, default=1e-5, help="AdamW weight decay. Default: 1e-5.")
    parser.add_argument("--patience", type=int, default=10, help="Early-stop patience on val MAE. Default: 10.")
    parser.add_argument(
        "--enable-target-weighting",
        action="store_true",
        help=(
            "Enable target-amplitude weighting for the training loss. "
            "Recommended for reducing underestimation on rare high-peak frames."
        ),
    )
    parser.add_argument(
        "--target-weight-thresholds",
        default="0.5,1.0",
        help=(
            "Comma-separated target thresholds in target units. "
            "Used only when --enable-target-weighting is set. Default: 0.5,1.0"
        ),
    )
    parser.add_argument(
        "--target-weight-values",
        default="1.0,1.5,3.0",
        help=(
            "Comma-separated loss weights for each target bin. "
            "Count must equal len(thresholds)+1. Default: 1.0,1.5,3.0"
        ),
    )
    parser.add_argument("--seed", type=int, default=7, help="Random seed. Default: 7.")
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N val/test rows. Default: 5.")
    return parser.parse_args()


class TemporalConvHead(nn.Module):
    def __init__(self, hidden_dim: int, kernel_size: int):
        super().__init__()
        kernel_size = max(1, int(kernel_size))
        left_padding = max(0, kernel_size - 1)
        self.net = nn.Sequential(
            nn.ConstantPad1d((left_padding, 0), 0.0),
            nn.Conv1d(int(hidden_dim), int(hidden_dim), kernel_size=kernel_size),
            nn.ReLU(),
            nn.ConstantPad1d((left_padding, 0), 0.0),
            nn.Conv1d(int(hidden_dim), int(hidden_dim), kernel_size=kernel_size),
            nn.ReLU(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class MinimalTemporalVisualRegressor(nn.Module):
    def __init__(
        self,
        hidden_dim: int,
        temporal_head: str = "mean",
        anchor_current_frame: bool = False,
        temporal_kernel_size: int = 3,
    ):
        super().__init__()
        self.temporal_head_type = str(temporal_head)
        self.anchor_current_frame = bool(anchor_current_frame)
        self.features = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.AdaptiveAvgPool2d((8, 16)),
        )
        flattened_dim = 32 * 8 * 16
        self.frame_head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(flattened_dim, int(hidden_dim)),
            nn.ReLU(),
        )
        if self.temporal_head_type == "gru":
            self.temporal_module = nn.GRU(
                input_size=int(hidden_dim),
                hidden_size=int(hidden_dim),
                batch_first=True,
            )
        elif self.temporal_head_type == "tcn":
            self.temporal_module = TemporalConvHead(
                hidden_dim=int(hidden_dim),
                kernel_size=int(temporal_kernel_size),
            )
        elif self.temporal_head_type == "mean":
            self.temporal_module = None
        else:
            raise RuntimeError(f"unsupported temporal head: {temporal_head}")
        regressor_in_dim = int(hidden_dim) * (2 if self.anchor_current_frame else 1)
        self.regressor = nn.Sequential(
            nn.Linear(regressor_in_dim, int(hidden_dim)),
            nn.ReLU(),
            nn.Linear(int(hidden_dim), 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: [B, K, C, H, W]
        batch_size, num_frames, channels, height, width = x.shape
        flat = x.reshape(batch_size * num_frames, channels, height, width)
        frame_features = self.frame_head(self.features(flat))
        frame_features = frame_features.reshape(batch_size, num_frames, -1)
        if self.temporal_head_type == "gru":
            temporal_features, _hidden = self.temporal_module(frame_features)
            summary = temporal_features[:, -1, :]
        elif self.temporal_head_type == "tcn":
            temporal_features = self.temporal_module(frame_features.transpose(1, 2))
            summary = temporal_features[:, :, -1]
        else:
            summary = torch.mean(frame_features, dim=1)
        if self.anchor_current_frame:
            summary = torch.cat([summary, frame_features[:, -1, :]], dim=1)
        return self.regressor(summary).squeeze(-1)


class TemporalSequenceDataset(Dataset):
    def __init__(
        self,
        samples: Sequence[Dict[str, object]],
        image_height: int,
        image_width: int,
        image_mean: np.ndarray,
        image_std: np.ndarray,
        target_stats: Dict[str, float],
    ):
        self.samples = [dict(sample) for sample in samples]
        self.image_height = int(image_height)
        self.image_width = int(image_width)
        self.image_mean = np.asarray(image_mean, dtype=np.float32).reshape(1, 1, 1)
        self.image_std = np.asarray(image_std, dtype=np.float32).reshape(1, 1, 1)
        self.target_mean = float(target_stats["mean"])
        self.target_std = float(target_stats["std"])
        self._image_cache: Dict[str, np.ndarray] = {}

    def __len__(self) -> int:
        return len(self.samples)

    def _load_image(self, path: str) -> np.ndarray:
        cached = self._image_cache.get(path)
        if cached is not None:
            return cached
        image = load_single_image(Path(path), image_height=self.image_height, image_width=self.image_width).astype(np.float32)
        self._image_cache[path] = image
        return image

    def __getitem__(self, index: int):
        sample = self.samples[index]
        seq_images = [self._load_image(path) for path in sample["sequence_paths"]]
        x = np.stack(seq_images, axis=0)
        x = ((x - self.image_mean) / self.image_std).astype(np.float32)
        y = (float(sample["y"]) - self.target_mean) / self.target_std
        loss_weight = float(sample.get("loss_weight", 1.0))
        return (
            torch.from_numpy(x),
            torch.tensor(y, dtype=torch.float32),
            torch.tensor(loss_weight, dtype=torch.float32),
        )


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
    return value if np.isfinite(value) else None


def load_session_frame_table(debug_session_csv: Path, image_column: str) -> Dict[str, object]:
    rows = read_csv_rows(debug_session_csv)
    ordered = []
    for row in rows:
        frame_index = int(float(row.get("frame_index", -1)))
        image_path = str(row.get(image_column, "")).strip()
        if frame_index < 0 or image_path == "":
            continue
        ordered.append((frame_index, image_path))
    if not ordered:
        raise RuntimeError(f"no usable {image_column} rows in session csv: {debug_session_csv}")
    ordered.sort(key=lambda item: item[0])
    frame_indices = [item[0] for item in ordered]
    frame_paths = {frame_idx: image_path for frame_idx, image_path in ordered}
    return {
        "frame_indices": frame_indices,
        "frame_paths": frame_paths,
    }


def resolve_history_indices(frame_indices: Sequence[int], current_frame: int, history_frames: int, history_step: int) -> List[int]:
    if not frame_indices:
        raise RuntimeError("session has no frame indices")
    out = []
    first_frame = int(frame_indices[0])
    for reverse_offset in range(int(history_frames) - 1, -1, -1):
        target = int(current_frame) - reverse_offset * int(history_step)
        if target <= first_frame:
            out.append(first_frame)
            continue
        pos = bisect.bisect_right(frame_indices, target) - 1
        pos = max(0, min(pos, len(frame_indices) - 1))
        out.append(int(frame_indices[pos]))
    return out


def session_key_from_row(row: Dict[str, str]) -> str:
    debug_session_csv = str(row.get("debug_session_csv", "")).strip()
    if debug_session_csv != "":
        return debug_session_csv
    bag_id = str(row.get("bag_id", "")).strip()
    if bag_id != "":
        return bag_id
    return str(row.get("session_id", "")).strip()


def build_session_frame_tables_from_rows(
    rows: Sequence[Dict[str, str]],
    image_column: str,
) -> Dict[str, Dict[str, object]]:
    grouped: Dict[str, List[Tuple[int, str]]] = {}
    for row in rows:
        session_key = session_key_from_row(row)
        if session_key == "":
            continue
        frame_index = int(float(row.get("frame_index", -1)))
        image_path = str(row.get(image_column, "")).strip()
        if frame_index < 0 or image_path == "":
            continue
        if not Path(image_path).exists():
            continue
        grouped.setdefault(session_key, []).append((frame_index, image_path))
    session_tables: Dict[str, Dict[str, object]] = {}
    for session_key, pairs in grouped.items():
        pairs.sort(key=lambda item: item[0])
        frame_indices: List[int] = []
        frame_paths: Dict[int, str] = {}
        for frame_index, image_path in pairs:
            frame_indices.append(int(frame_index))
            frame_paths[int(frame_index)] = image_path
        if frame_indices:
            session_tables[session_key] = {
                "frame_indices": frame_indices,
                "frame_paths": frame_paths,
            }
    return session_tables


def build_sequence_samples(
    rows: Sequence[Dict[str, str]],
    image_column: str,
    target_column: str,
    history_frames: int,
    history_step: int,
) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
    session_cache = build_session_frame_tables_from_rows(rows, image_column)
    samples: List[Dict[str, object]] = []
    skip_counts = {
        "missing_target": 0,
        "missing_session_frames": 0,
        "missing_image_path": 0,
        "image_not_found": 0,
        "session_load_error": 0,
    }
    for row in rows:
        target = finite_float(row.get(target_column))
        if target is None:
            skip_counts["missing_target"] += 1
            continue
        cache_key = session_key_from_row(row)
        frame_table = session_cache.get(cache_key)
        if frame_table is None or not frame_table["frame_indices"]:
            session_csv_raw = str(row.get("debug_session_csv", "")).strip()
            if session_csv_raw == "":
                skip_counts["missing_session_frames"] += 1
                continue
            session_csv = Path(session_csv_raw).expanduser()
            if not session_csv.exists():
                skip_counts["missing_session_frames"] += 1
                continue
            try:
                frame_table = load_session_frame_table(session_csv, image_column)
            except RuntimeError:
                skip_counts["session_load_error"] += 1
                continue
            session_cache[cache_key] = frame_table
        current_frame = int(float(row.get("frame_index", -1)))
        history_indices = resolve_history_indices(
            frame_indices=frame_table["frame_indices"],
            current_frame=current_frame,
            history_frames=history_frames,
            history_step=history_step,
        )
        sequence_paths = []
        missing_path = False
        for frame_idx in history_indices:
            image_path = str(frame_table["frame_paths"].get(frame_idx, "")).strip()
            if image_path == "":
                missing_path = True
                break
            if not Path(image_path).exists():
                skip_counts["image_not_found"] += 1
                missing_path = True
                break
            sequence_paths.append(image_path)
        if missing_path:
            skip_counts["missing_image_path"] += 1
            continue
        samples.append(
            {
                "sequence_paths": sequence_paths,
                "y": float(target),
                "row_id": str(row.get("row_id", "")),
                "session_id": str(row.get("session_id", "")),
                "session_name": str(row.get("session_name", "")),
                "bag_id": str(row.get("bag_id", "")),
                "frame_index": current_frame,
                "relative_time_s": float(finite_float(row.get("relative_time_s")) or 0.0),
                "image_path": sequence_paths[-1],
                "baseline_slosh_raw": finite_float(row.get("slosh_height_mm")),
                "baseline_peak_raw": finite_float(row.get("peak_rel_mm_v2")),
                "baseline_center_raw": finite_float(row.get("center_rel_mm_v2")),
                "history_indices": history_indices,
            }
        )
    return samples, skip_counts


def image_stats_from_sequence_samples(
    samples: Sequence[Dict[str, object]],
    image_height: int,
    image_width: int,
) -> Tuple[np.ndarray, np.ndarray]:
    cache: Dict[str, np.ndarray] = {}
    pixel_sum = 0.0
    pixel_sq_sum = 0.0
    pixel_count = 0
    for sample in samples:
        for image_path in sample["sequence_paths"]:
            image = cache.get(image_path)
            if image is None:
                image = load_single_image(Path(image_path), image_height=image_height, image_width=image_width).astype(np.float32)
                cache[image_path] = image
            pixel_sum += float(np.sum(image, dtype=np.float64))
            pixel_sq_sum += float(np.sum(np.square(image, dtype=np.float64), dtype=np.float64))
            pixel_count += int(image.size)
    if pixel_count <= 0:
        return np.asarray([0.5], dtype=np.float32), np.asarray([0.25], dtype=np.float32)
    mean = pixel_sum / pixel_count
    var = max(0.0, pixel_sq_sum / pixel_count - mean * mean)
    std = math.sqrt(var) if var > 1e-12 else 1.0
    return np.asarray([mean], dtype=np.float32), np.asarray([std], dtype=np.float32)


def target_stats_from_samples(samples: Sequence[Dict[str, object]]) -> Dict[str, float]:
    targets = np.asarray([float(sample["y"]) for sample in samples], dtype=np.float32)
    mean = float(np.mean(targets))
    std = float(np.std(targets))
    if std < 1e-6:
        std = 1.0
    return {"mean": mean, "std": std}


def parse_float_list(raw_text: str, arg_name: str) -> List[float]:
    values: List[float] = []
    text = str(raw_text).strip()
    if text == "":
        return values
    for piece in text.split(","):
        token = piece.strip()
        if token == "":
            continue
        try:
            value = float(token)
        except ValueError as exc:
            raise RuntimeError(f"failed to parse {arg_name}: {raw_text}") from exc
        if not np.isfinite(value):
            raise RuntimeError(f"{arg_name} contains non-finite value: {token}")
        values.append(float(value))
    return values


def resolve_target_weight(target: float, thresholds: Sequence[float], weights: Sequence[float]) -> float:
    for idx, threshold in enumerate(thresholds):
        if float(target) <= float(threshold):
            return float(weights[idx])
    return float(weights[-1])


def build_target_weighting_config(args) -> Dict[str, object]:
    if not bool(args.enable_target_weighting):
        return {"enabled": False, "thresholds": [], "weights": []}
    thresholds = parse_float_list(str(args.target_weight_thresholds), "--target-weight-thresholds")
    weights = parse_float_list(str(args.target_weight_values), "--target-weight-values")
    if not weights:
        raise RuntimeError("target weighting enabled but --target-weight-values is empty")
    if len(weights) != len(thresholds) + 1:
        raise RuntimeError("target weighting requires len(weights) == len(thresholds) + 1")
    prev_threshold = None
    for threshold in thresholds:
        if prev_threshold is not None and float(threshold) <= float(prev_threshold):
            raise RuntimeError("--target-weight-thresholds must be strictly increasing")
        prev_threshold = float(threshold)
    for weight in weights:
        if float(weight) <= 0.0:
            raise RuntimeError("--target-weight-values must be positive")
    return {
        "enabled": True,
        "thresholds": [float(value) for value in thresholds],
        "weights": [float(value) for value in weights],
    }


def attach_loss_weights(
    samples: Sequence[Dict[str, object]],
    weighting_config: Dict[str, object],
) -> List[Dict[str, object]]:
    if not bool(weighting_config.get("enabled", False)):
        return [dict(sample, loss_weight=1.0) for sample in samples]
    thresholds = [float(value) for value in weighting_config.get("thresholds", [])]
    weights = [float(value) for value in weighting_config.get("weights", [])]
    weighted_samples: List[Dict[str, object]] = []
    for sample in samples:
        updated = dict(sample)
        updated["loss_weight"] = resolve_target_weight(float(sample["y"]), thresholds, weights)
        weighted_samples.append(updated)
    return weighted_samples


def summarize_target_weighting(
    samples: Sequence[Dict[str, object]],
    weighting_config: Dict[str, object],
) -> List[Dict[str, object]]:
    thresholds = [float(value) for value in weighting_config.get("thresholds", [])]
    weights = [float(value) for value in weighting_config.get("weights", [])]
    if not bool(weighting_config.get("enabled", False)):
        return [
            {
                "lower_bound": None,
                "upper_bound": None,
                "weight": 1.0,
                "count": len(samples),
                "fraction": 0.0 if not samples else 1.0,
            }
        ]
    summary_rows: List[Dict[str, object]] = []
    lower = None
    total = max(1, len(samples))
    for idx, weight in enumerate(weights):
        upper = thresholds[idx] if idx < len(thresholds) else None
        count = 0
        for sample in samples:
            target = float(sample["y"])
            if lower is None:
                in_bin = target <= float(upper) if upper is not None else True
            elif upper is None:
                in_bin = target > float(lower)
            else:
                in_bin = float(lower) < target <= float(upper)
            if in_bin:
                count += 1
        summary_rows.append(
            {
                "lower_bound": lower,
                "upper_bound": upper,
                "weight": float(weight),
                "count": count,
                "fraction": float(count / total),
            }
        )
        lower = upper
    return summary_rows


def dataset_to_loader(dataset: Dataset, batch_size: int, shuffle: bool) -> DataLoader:
    return DataLoader(dataset, batch_size=max(1, int(batch_size)), shuffle=bool(shuffle))


def predict_dataset(
    model: nn.Module,
    dataset: Dataset,
    device: torch.device,
    target_stats: Dict[str, float],
    batch_size: int = 32,
) -> List[float]:
    loader = DataLoader(dataset, batch_size=max(1, int(batch_size)), shuffle=False)
    preds: List[float] = []
    model.eval()
    with torch.no_grad():
        for batch in loader:
            batch_x = batch[0]
            batch_x = batch_x.to(device=device)
            pred_norm = model(batch_x).detach().cpu().numpy()
            preds.extend(
                [
                    float(value * float(target_stats["std"]) + float(target_stats["mean"]))
                    for value in pred_norm
                ]
            )
    return preds


def collect_metrics(targets: Sequence[float], preds: Sequence[Optional[float]]) -> Dict[str, Optional[float]]:
    filtered_targets = []
    filtered_preds = []
    for target, pred in zip(targets, preds):
        if pred is None:
            continue
        filtered_targets.append(float(target))
        filtered_preds.append(float(pred))
    summary = regression_metrics(filtered_targets, filtered_preds)
    summary["coverage"] = 0.0 if not targets else float(len(filtered_preds) / len(targets))
    return summary


def fit_affine_baseline(
    targets: Sequence[float],
    baseline_values: Sequence[Optional[float]],
) -> Optional[Dict[str, float]]:
    x_values = []
    y_values = []
    for target, baseline in zip(targets, baseline_values):
        if baseline is None:
            continue
        x_values.append(float(baseline))
        y_values.append(float(target))
    if len(x_values) < 2:
        return None
    x_np = np.asarray(x_values, dtype=np.float64)
    y_np = np.asarray(y_values, dtype=np.float64)
    x_mean = float(np.mean(x_np))
    y_mean = float(np.mean(y_np))
    x_var = float(np.var(x_np))
    if x_var <= 1e-12:
        return None
    cov = float(np.mean((x_np - x_mean) * (y_np - y_mean)))
    slope = cov / x_var
    intercept = y_mean - slope * x_mean
    return {"slope": float(slope), "intercept": float(intercept)}


def apply_affine_baseline(
    affine: Optional[Dict[str, float]],
    baseline_values: Sequence[Optional[float]],
) -> List[Optional[float]]:
    if affine is None:
        return [None for _ in baseline_values]
    slope = float(affine["slope"])
    intercept = float(affine["intercept"])
    preds: List[Optional[float]] = []
    for baseline in baseline_values:
        if baseline is None:
            preds.append(None)
            continue
        preds.append(float(slope * float(baseline) + intercept))
    return preds


def write_history_csv(path: Path, history_rows: Sequence[Dict[str, object]]):
    fieldnames = ["epoch", "train_loss", "train_mae", "val_mae"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in history_rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def write_predictions_csv(path: Path, rows: Sequence[Dict[str, object]]):
    fieldnames = [
        "split",
        "row_id",
        "session_id",
        "session_name",
        "bag_id",
        "frame_index",
        "relative_time_s",
        "image_path",
        "history_indices",
        "target_human_peak_mm",
        "baseline_slosh_height_mm",
        "pred_train_mean_baseline",
        "baseline_peak_rel_mm_v2",
        "pred_peak_rel_mm_v2_affine",
        "baseline_center_rel_mm_v2",
        "pred_center_rel_mm_v2_affine",
        "pred_visual",
        "pred_visual_temporal",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def main() -> int:
    try:
        args = parse_args()
        set_seed(int(args.seed))
        device = choose_device(args.device)

        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        split_json_path = Path(args.split_json).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        manifest_rows = read_csv_rows(manifest_csv)
        split_json = load_json(split_json_path)

        train_rows = filter_rows_for_split(manifest_rows, split_json, "train")
        val_rows = filter_rows_for_split(manifest_rows, split_json, "val")
        test_rows = filter_rows_for_split(manifest_rows, split_json, "test")

        history_frames = int(args.history_frames)
        history_step = int(args.history_step)
        train_samples, train_skip_counts = build_sequence_samples(
            train_rows,
            image_column=args.image_column,
            target_column=args.target_column,
            history_frames=history_frames,
            history_step=history_step,
        )
        val_samples, val_skip_counts = build_sequence_samples(
            val_rows,
            image_column=args.image_column,
            target_column=args.target_column,
            history_frames=history_frames,
            history_step=history_step,
        )
        test_samples, test_skip_counts = build_sequence_samples(
            test_rows,
            image_column=args.image_column,
            target_column=args.target_column,
            history_frames=history_frames,
            history_step=history_step,
        )

        if not train_samples:
            raise RuntimeError("no train sequence samples after loading")
        if not val_samples:
            raise RuntimeError("no val sequence samples after loading")
        if not test_samples:
            raise RuntimeError("no test sequence samples after loading")

        image_mean, image_std = image_stats_from_sequence_samples(
            train_samples,
            image_height=int(args.image_height),
            image_width=int(args.image_width),
        )
        target_weighting = build_target_weighting_config(args)
        train_samples = attach_loss_weights(train_samples, target_weighting)
        target_weighting_summary = summarize_target_weighting(train_samples, target_weighting)
        target_stats = target_stats_from_samples(train_samples)

        train_dataset = TemporalSequenceDataset(
            train_samples,
            image_height=int(args.image_height),
            image_width=int(args.image_width),
            image_mean=image_mean,
            image_std=image_std,
            target_stats=target_stats,
        )
        val_dataset = TemporalSequenceDataset(
            val_samples,
            image_height=int(args.image_height),
            image_width=int(args.image_width),
            image_mean=image_mean,
            image_std=image_std,
            target_stats=target_stats,
        )
        test_dataset = TemporalSequenceDataset(
            test_samples,
            image_height=int(args.image_height),
            image_width=int(args.image_width),
            image_mean=image_mean,
            image_std=image_std,
            target_stats=target_stats,
        )

        train_loader = dataset_to_loader(train_dataset, batch_size=int(args.batch_size), shuffle=True)

        model = MinimalTemporalVisualRegressor(
            hidden_dim=int(args.hidden_dim),
            temporal_head=str(args.temporal_head),
            anchor_current_frame=bool(args.anchor_current_frame),
            temporal_kernel_size=int(args.temporal_kernel_size),
        ).to(device=device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=float(args.lr), weight_decay=float(args.weight_decay))
        criterion = nn.HuberLoss(delta=1.0, reduction="none")

        best_state = None
        best_epoch = -1
        best_val_mae = float("inf")
        epochs_without_improvement = 0
        history_rows: List[Dict[str, object]] = []

        for epoch in range(1, int(args.epochs) + 1):
            model.train()
            running_loss = 0.0
            batch_count = 0
            for batch_x, batch_y, batch_weight in train_loader:
                batch_x = batch_x.to(device=device)
                batch_y = batch_y.to(device=device)
                batch_weight = batch_weight.to(device=device)
                optimizer.zero_grad()
                pred_y = model(batch_x)
                per_sample_loss = criterion(pred_y, batch_y)
                loss = torch.sum(per_sample_loss * batch_weight) / torch.clamp(torch.sum(batch_weight), min=1e-6)
                loss.backward()
                optimizer.step()
                running_loss += float(loss.item())
                batch_count += 1

            train_preds = predict_dataset(model, train_dataset, device=device, target_stats=target_stats)
            val_preds = predict_dataset(model, val_dataset, device=device, target_stats=target_stats)
            train_metrics = regression_metrics([float(sample["y"]) for sample in train_samples], train_preds)
            val_metrics = regression_metrics([float(sample["y"]) for sample in val_samples], val_preds)
            val_mae = float(val_metrics["mae"]) if val_metrics["mae"] is not None else float("inf")
            history_rows.append(
                {
                    "epoch": epoch,
                    "train_loss": running_loss / max(1, batch_count),
                    "train_mae": train_metrics["mae"],
                    "val_mae": val_metrics["mae"],
                }
            )
            if val_mae + 1e-9 < best_val_mae:
                best_val_mae = val_mae
                best_epoch = epoch
                best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
                epochs_without_improvement = 0
            else:
                epochs_without_improvement += 1
                if epochs_without_improvement >= int(args.patience):
                    break

        if best_state is None:
            raise RuntimeError("training failed to produce a best checkpoint")

        model.load_state_dict(best_state)
        train_preds = predict_dataset(model, train_dataset, device=device, target_stats=target_stats)
        val_preds = predict_dataset(model, val_dataset, device=device, target_stats=target_stats)
        test_preds = predict_dataset(model, test_dataset, device=device, target_stats=target_stats)

        train_targets = [float(sample["y"]) for sample in train_samples]
        val_targets = [float(sample["y"]) for sample in val_samples]
        test_targets = [float(sample["y"]) for sample in test_samples]
        train_slosh = [sample["baseline_slosh_raw"] for sample in train_samples]
        val_slosh = [sample["baseline_slosh_raw"] for sample in val_samples]
        test_slosh = [sample["baseline_slosh_raw"] for sample in test_samples]
        train_peak = [sample["baseline_peak_raw"] for sample in train_samples]
        val_peak = [sample["baseline_peak_raw"] for sample in val_samples]
        test_peak = [sample["baseline_peak_raw"] for sample in test_samples]
        train_center = [sample["baseline_center_raw"] for sample in train_samples]
        val_center = [sample["baseline_center_raw"] for sample in val_samples]
        test_center = [sample["baseline_center_raw"] for sample in test_samples]

        train_mean = float(np.mean(np.asarray(train_targets, dtype=np.float64)))
        peak_affine = fit_affine_baseline(train_targets, train_peak)
        center_affine = fit_affine_baseline(train_targets, train_center)
        mean_val = [train_mean for _ in val_targets]
        mean_test = [train_mean for _ in test_targets]
        peak_affine_val = apply_affine_baseline(peak_affine, val_peak)
        peak_affine_test = apply_affine_baseline(peak_affine, test_peak)
        center_affine_val = apply_affine_baseline(center_affine, val_center)
        center_affine_test = apply_affine_baseline(center_affine, test_center)

        checkpoint_path = out_dir / "SL_visual_temporal_human.pt"
        history_csv_path = out_dir / "SL_visual_temporal_human_history.csv"
        predictions_csv_path = out_dir / "SL_visual_temporal_human_predictions.csv"
        summary_json_path = out_dir / "SL_visual_temporal_human_summary.json"
        bagwise_csv_path = out_dir / "SL_visual_temporal_human_bagwise.csv"

        torch.save(
            {
                "model_type": "sl_visual_temporal_regressor_v2",
                "target_column": str(args.target_column),
                "image_column": str(args.image_column),
                "history_frames": history_frames,
                "history_step": history_step,
                "image_height": int(args.image_height),
                "image_width": int(args.image_width),
                "hidden_dim": int(args.hidden_dim),
                "temporal_head": str(args.temporal_head),
                "anchor_current_frame": bool(args.anchor_current_frame),
                "temporal_kernel_size": int(args.temporal_kernel_size),
                "target_weighting": target_weighting,
                "image_mean": image_mean.tolist(),
                "image_std": image_std.tolist(),
                "target_mean": float(target_stats["mean"]),
                "target_std": float(target_stats["std"]),
                "best_epoch": int(best_epoch),
                "best_val_mae": float(best_val_mae),
                "model_state_dict": best_state,
                "manifest_csv": str(manifest_csv),
                "split_json": str(split_json_path),
            },
            checkpoint_path,
        )
        write_history_csv(history_csv_path, history_rows)

        prediction_rows: List[Dict[str, object]] = []
        for split_name, samples, preds, peak_affine_preds, center_affine_preds in (
            ("val", val_samples, val_preds, peak_affine_val, center_affine_val),
            ("test", test_samples, test_preds, peak_affine_test, center_affine_test),
        ):
            for sample, pred, peak_affine_pred, center_affine_pred in zip(
                samples, preds, peak_affine_preds, center_affine_preds
            ):
                prediction_rows.append(
                    {
                        "split": split_name,
                        "row_id": sample["row_id"],
                        "session_id": sample["session_id"],
                        "session_name": sample["session_name"],
                        "bag_id": sample["bag_id"],
                        "frame_index": int(sample["frame_index"]),
                        "relative_time_s": float(sample["relative_time_s"]),
                        "image_path": sample["image_path"],
                        "history_indices": ",".join(str(v) for v in sample["history_indices"]),
                        "target_human_peak_mm": float(sample["y"]),
                        "baseline_slosh_height_mm": sample["baseline_slosh_raw"],
                        "pred_train_mean_baseline": train_mean,
                        "baseline_peak_rel_mm_v2": sample["baseline_peak_raw"],
                        "pred_peak_rel_mm_v2_affine": peak_affine_pred,
                        "baseline_center_rel_mm_v2": sample["baseline_center_raw"],
                        "pred_center_rel_mm_v2_affine": center_affine_pred,
                        "pred_visual": float(pred),
                        "pred_visual_temporal": float(pred),
                    }
                )
        write_predictions_csv(predictions_csv_path, prediction_rows)

        bagwise_rows = []
        for model_name, pred_key in (
            ("mean", "pred_train_mean_baseline"),
            ("slosh_raw", "baseline_slosh_height_mm"),
            ("peak_affine", "pred_peak_rel_mm_v2_affine"),
            ("center_affine", "pred_center_rel_mm_v2_affine"),
            ("visual_temporal", "pred_visual_temporal"),
        ):
            for row in group_metrics(
                [
                    {
                        "bag_id": row["bag_id"],
                        "target": row["target_human_peak_mm"],
                        pred_key: row[pred_key],
                    }
                    for row in prediction_rows
                    if row["split"] == "test"
                ],
                pred_key,
            ):
                bagwise_rows.append({"model_name": model_name, **row})
        with bagwise_csv_path.open("w", newline="", encoding="utf-8") as handle:
            fieldnames = ["model_name", "bag_id", "count", "mae", "rmse", "bias_mean", "bias_median", "corr"]
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in bagwise_rows:
                writer.writerow(row)

        summary = {
            "checkpoint_path": str(checkpoint_path),
            "history_csv": str(history_csv_path),
            "predictions_csv": str(predictions_csv_path),
            "bagwise_csv": str(bagwise_csv_path),
            "manifest_csv": str(manifest_csv),
            "split_json": str(split_json_path),
            "task": "visual_temporal_human_peak_regression",
            "target_column": str(args.target_column),
            "image_column": str(args.image_column),
            "device": str(device),
            "history_frames": history_frames,
            "history_step": history_step,
            "image_height": int(args.image_height),
            "image_width": int(args.image_width),
            "hidden_dim": int(args.hidden_dim),
            "temporal_head": str(args.temporal_head),
            "anchor_current_frame": bool(args.anchor_current_frame),
            "temporal_kernel_size": int(args.temporal_kernel_size),
            "target_weighting": target_weighting_summary,
            "best_epoch": int(best_epoch),
            "best_val_mae": float(best_val_mae),
            "num_train_samples": len(train_samples),
            "num_val_samples": len(val_samples),
            "num_test_samples": len(test_samples),
            "train_skip_counts": train_skip_counts,
            "val_skip_counts": val_skip_counts,
            "test_skip_counts": test_skip_counts,
            "baseline_metrics": {
                "val_train_mean": collect_metrics(val_targets, mean_val),
                "test_train_mean": collect_metrics(test_targets, mean_test),
                "val_slosh_raw": collect_metrics(val_targets, val_slosh),
                "test_slosh_raw": collect_metrics(test_targets, test_slosh),
                "val_peak_affine": collect_metrics(val_targets, peak_affine_val),
                "test_peak_affine": collect_metrics(test_targets, peak_affine_test),
                "val_center_affine": collect_metrics(val_targets, center_affine_val),
                "test_center_affine": collect_metrics(test_targets, center_affine_test),
            },
            "visual_metrics": {
                "train": regression_metrics(train_targets, train_preds),
                "val": regression_metrics(val_targets, val_preds),
                "test": regression_metrics(test_targets, test_preds),
            },
        }
        with summary_json_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] checkpoint: {checkpoint_path}")
        print(f"[OK] history csv: {history_csv_path}")
        print(f"[OK] predictions csv: {predictions_csv_path}")
        print(f"[OK] summary json: {summary_json_path}")
        print(
            f"[OK] samples train={len(train_samples)} val={len(val_samples)} test={len(test_samples)} "
            f"| image_column={args.image_column} target={args.target_column} "
            f"| history_frames={history_frames} history_step={history_step} "
            f"| temporal_head={args.temporal_head} anchor_current_frame={bool(args.anchor_current_frame)}"
        )
        if bool(target_weighting.get("enabled", False)):
            print(
                f"[OK] target weighting enabled"
                f" | thresholds={target_weighting['thresholds']}"
                f" | weights={target_weighting['weights']}"
            )
        print(f"[OK] baseline test slosh raw mae: {summary['baseline_metrics']['test_slosh_raw']['mae']}")
        print(f"[OK] baseline test peak affine mae: {summary['baseline_metrics']['test_peak_affine']['mae']}")
        print(f"[OK] baseline test center affine mae: {summary['baseline_metrics']['test_center_affine']['mae']}")
        print(f"[OK] visual temporal test metrics: {summary['visual_metrics']['test']}")
        show_first = max(0, int(args.show_first))
        for row in prediction_rows[:show_first]:
            print(
                "[OK] row={row_id} split={split} target={target_human_peak_mm:.6f} "
                "pred_visual_temporal={pred_visual_temporal:.6f}".format(**row)
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
