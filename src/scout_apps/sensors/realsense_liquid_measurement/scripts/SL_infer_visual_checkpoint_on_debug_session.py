#!/usr/bin/env python3
"""Run a visual SL checkpoint on a debug session and export a continuous prediction curve."""

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import torch

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception as exc:  # pylint: disable=broad-except
    plt = None
    PLOT_IMPORT_ERROR = exc
else:
    PLOT_IMPORT_ERROR = None

from FSL_train_visual_pseudolabel import MinimalVisualRegressor, load_single_image
from SL_train_baseline import choose_device
from SL_train_visual_temporal_human import MinimalTemporalVisualRegressor, resolve_history_indices
from extract_liquid_height_from_bag import rectify_roi_and_calibration
from extract_liquid_height_v2_from_bag import load_v2_calibration


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run a visual SL checkpoint on a full debug session, export frame-wise predictions, "
            "and render a continuous liquid-level curve."
        )
    )
    parser.add_argument("--debug-dir", required=True, help="Debug-session directory containing debug_session.csv/json.")
    parser.add_argument("--checkpoint", required=True, help="Path to a trained visual SL checkpoint (.pt).")
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to <debug_dir>/SL_infer_visual_checkpoint.",
    )
    parser.add_argument(
        "--image-column",
        default="",
        help="Override checkpoint image_column. Default: use checkpoint value.",
    )
    parser.add_argument(
        "--manifest-csv",
        default="",
        help=(
            "Optional manifest csv used to recover image paths when the debug session csv does not contain "
            "the checkpoint image column."
        ),
    )
    parser.add_argument(
        "--raw-roi-root",
        default="",
        help=(
            "Optional raw rectified ROI root, for example "
            "<repo>/sl_artifacts/.../raw_rectified_roi . "
            "Used as a fallback when image_column=raw_rectified_roi_path is absent from debug_session.csv."
        ),
    )
    parser.add_argument("--batch-size", type=int, default=32, help="Inference batch size. Default: 32.")
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--dpi", type=int, default=160, help="Curve figure DPI. Default: 160.")
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N predictions. Default: 5.")
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


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"csv has no rows: {path}")
    return rows


def load_json(path: Path) -> Dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    return data if isinstance(data, dict) else {}


def infer_session_identity(debug_dir: Path, session_meta: Dict) -> Tuple[str, str, str, str]:
    bag_raw = str(session_meta.get("bag", "")).strip()
    if bag_raw:
        bag_id = Path(bag_raw).stem
    else:
        bag_id = debug_dir.name
    session_name = debug_dir.name
    date_id = debug_dir.parent.name
    session_id = f"{date_id}/{session_name}" if date_id else session_name
    return session_id, session_name, date_id, bag_id


def build_rows_for_inference(debug_dir: Path) -> Tuple[List[Dict[str, str]], Dict]:
    debug_dir = debug_dir.expanduser().resolve()
    session_csv = debug_dir / "debug_session.csv"
    session_json = debug_dir / "debug_session.json"
    rows = read_csv_rows(session_csv)
    session_meta = load_json(session_json)
    session_id, session_name, date_id, bag_id = infer_session_identity(debug_dir, session_meta)
    prepared: List[Dict[str, str]] = []
    for raw in rows:
        row = dict(raw)
        frame_index = int(float(row.get("frame_index", -1)))
        row["row_id"] = f"{session_id}:{frame_index}"
        row["session_id"] = session_id
        row["session_name"] = session_name
        row["date_id"] = date_id
        row["bag_id"] = bag_id
        row["calibration_path"] = str(session_meta.get("calibration", "")).strip()
        prepared.append(row)
    summary = {
        "bag": str(session_meta.get("bag", "")),
        "calibration": str(session_meta.get("calibration", "")),
        "config": str(session_meta.get("config", "")),
        "image_topic": str(session_meta.get("image_topic", "")),
    }
    return prepared, summary


def manifest_image_lookup(
    manifest_csv: Path,
    session_id: str,
    bag_id: str,
    image_column: str,
) -> Dict[int, str]:
    lookup: Dict[int, str] = {}
    rows = read_csv_rows(manifest_csv)
    for row in rows:
        row_session_id = str(row.get("session_id", "")).strip()
        row_bag_id = str(row.get("bag_id", "")).strip()
        if row_session_id != session_id and row_bag_id != bag_id:
            continue
        frame_index = int(float(row.get("frame_index", -1)))
        image_path = str(row.get(image_column, "")).strip()
        if frame_index < 0 or image_path == "":
            continue
        if Path(image_path).exists():
            lookup[frame_index] = image_path
    return lookup


def infer_raw_roi_path(raw_roi_root: Path, date_id: str, session_name: str, frame_index: int) -> Optional[str]:
    candidate = raw_roi_root / date_id / session_name / f"frame_{int(frame_index):06d}.png"
    if candidate.exists():
        return str(candidate)
    return None


def infer_raw_roi_root_from_paths(image_paths: Sequence[str]) -> Optional[Path]:
    for image_path in image_paths:
        path = Path(image_path).expanduser().resolve()
        parts = path.parts
        if "raw_rectified_roi" not in parts:
            continue
        anchor = parts.index("raw_rectified_roi")
        root = Path(*parts[: anchor + 1])
        if root.exists():
            return root
    return None


def resolve_row_image_path(
    row: Dict[str, str],
    image_column: str,
    manifest_lookup: Dict[int, str],
    raw_roi_root: Optional[Path],
) -> Optional[str]:
    direct = str(row.get(image_column, "")).strip()
    if direct != "" and Path(direct).exists():
        return direct
    frame_index = int(float(row.get("frame_index", -1)))
    lookup_path = manifest_lookup.get(frame_index, "")
    if lookup_path and Path(lookup_path).exists():
        return lookup_path
    if image_column == "raw_rectified_roi_path" and raw_roi_root is not None:
        date_id = str(row.get("date_id", "")).strip()
        session_name = str(row.get("session_name", "")).strip()
        inferred = infer_raw_roi_path(raw_roi_root, date_id, session_name, frame_index)
        if inferred is not None:
            return inferred
    return None


def can_rectify_from_photo(row: Dict[str, str], image_column: str) -> bool:
    if image_column != "raw_rectified_roi_path":
        return False
    photo_path = str(row.get("photo_path", "")).strip()
    calibration_path = str(row.get("calibration_path", "")).strip()
    return photo_path != "" and calibration_path != "" and Path(photo_path).exists() and Path(calibration_path).exists()


def build_resolved_rows(
    rows: Sequence[Dict[str, str]],
    image_column: str,
    manifest_lookup: Dict[int, str],
    raw_roi_root: Optional[Path],
) -> Tuple[List[Dict[str, str]], Dict[str, int]]:
    resolved: List[Dict[str, str]] = []
    skip_counts = {
        "missing_image_path": 0,
        "image_not_found": 0,
        "rectify_from_photo": 0,
        "direct_image": 0,
    }
    for row in rows:
        updated = dict(row)
        image_path = resolve_row_image_path(
            row=row,
            image_column=image_column,
            manifest_lookup=manifest_lookup,
            raw_roi_root=raw_roi_root,
        )
        if image_path is None:
            if can_rectify_from_photo(row, image_column):
                updated["resolved_image_mode"] = "rectify_from_photo"
                updated["resolved_image_path"] = str(row.get("photo_path", "")).strip()
                skip_counts["rectify_from_photo"] += 1
                resolved.append(updated)
                continue
            skip_counts["missing_image_path"] += 1
            continue
        if not Path(image_path).exists():
            skip_counts["image_not_found"] += 1
            continue
        updated[image_column] = image_path
        updated["resolved_image_mode"] = "direct"
        updated["resolved_image_path"] = image_path
        skip_counts["direct_image"] += 1
        resolved.append(updated)
    if not resolved:
        raise RuntimeError(
            f"no rows have usable image paths for image_column={image_column}. "
            f"If this is raw_rectified_roi_path, try providing --manifest-csv or --raw-roi-root, "
            f"or ensure photo_path + calibration are available for online rectification."
        )
    return resolved, skip_counts


def build_visual_samples(
    rows: Sequence[Dict[str, str]],
    image_column: str,
    model_type: str,
    history_frames: int,
    history_step: int,
) -> List[Dict[str, object]]:
    ordered_rows = sorted(
        rows,
        key=lambda row: (
            float(finite_float(row.get("relative_time_s")) or 0.0),
            int(float(row.get("frame_index", -1))),
        ),
    )
    frame_indices = [int(float(row.get("frame_index", -1))) for row in ordered_rows]
    frame_sources = {
        int(float(row.get("frame_index", -1))): {
            "mode": str(row.get("resolved_image_mode", "direct")),
            "path": str(row.get("resolved_image_path", "")).strip(),
            "calibration_path": str(row.get("calibration_path", "")).strip(),
        }
        for row in ordered_rows
    }
    samples: List[Dict[str, object]] = []
    for row in ordered_rows:
        frame_index = int(float(row.get("frame_index", -1)))
        image_path = str(row.get("resolved_image_path", row.get(image_column, ""))).strip()
        if model_type == "sl_visual_temporal_regressor_v2":
            history_indices = resolve_history_indices(
                frame_indices=frame_indices,
                current_frame=frame_index,
                history_frames=history_frames,
                history_step=history_step,
            )
            sequence_items = [dict(frame_sources[idx]) for idx in history_indices]
        elif model_type in ("sl_visual_cnn_regressor_v1", "fsl_visual_cnn_regressor_v1"):
            history_indices = [frame_index]
            sequence_items = [
                {
                    "mode": str(row.get("resolved_image_mode", "direct")),
                    "path": image_path,
                    "calibration_path": str(row.get("calibration_path", "")).strip(),
                }
            ]
        else:
            raise RuntimeError(f"unsupported visual checkpoint model_type: {model_type}")
        samples.append(
            {
                "row_id": str(row.get("row_id", "")),
                "session_id": str(row.get("session_id", "")),
                "session_name": str(row.get("session_name", "")),
                "bag_id": str(row.get("bag_id", "")),
                "frame_index": frame_index,
                "relative_time_s": float(finite_float(row.get("relative_time_s")) or 0.0),
                "image_path": image_path,
                "history_indices": history_indices,
                "sequence_items": sequence_items,
                "target_human_peak_mm": finite_float(row.get("human_peak_mm")),
                "baseline_slosh_height_mm": finite_float(row.get("slosh_height_mm")),
                "baseline_peak_rel_mm_v2": finite_float(row.get("peak_rel_mm_v2")),
                "baseline_center_rel_mm_v2": finite_float(row.get("center_rel_mm_v2")),
            }
        )
    return samples


def build_model_from_visual_checkpoint(checkpoint: Dict) -> torch.nn.Module:
    model_type = str(checkpoint.get("model_type", "")).strip()
    image_height = int(checkpoint["image_height"])
    image_width = int(checkpoint["image_width"])
    hidden_dim = int(checkpoint.get("hidden_dim", 64))
    if model_type == "sl_visual_temporal_regressor_v2":
        return MinimalTemporalVisualRegressor(
            hidden_dim=hidden_dim,
            temporal_head=str(checkpoint.get("temporal_head", "mean")),
            anchor_current_frame=bool(checkpoint.get("anchor_current_frame", False)),
            temporal_kernel_size=int(checkpoint.get("temporal_kernel_size", 3)),
        )
    if model_type in ("sl_visual_cnn_regressor_v1", "fsl_visual_cnn_regressor_v1"):
        return MinimalVisualRegressor(
            image_height=image_height,
            image_width=image_width,
            hidden_dim=hidden_dim,
        )
    raise RuntimeError(f"unsupported visual checkpoint model_type: {model_type}")


def load_sample_tensor(
    sample: Dict[str, object],
    model_type: str,
    image_height: int,
    image_width: int,
    image_mean: np.ndarray,
    image_std: np.ndarray,
    image_cache: Dict[str, np.ndarray],
    calibration_cache: Dict[str, object],
) -> np.ndarray:
    images = [
        load_sequence_image(
            item=item,
            image_height=image_height,
            image_width=image_width,
            image_cache=image_cache,
            calibration_cache=calibration_cache,
        )
        for item in sample["sequence_items"]
    ]
    if model_type == "sl_visual_temporal_regressor_v2":
        x = np.stack(images, axis=0)
    else:
        x = images[-1]
    return ((x - image_mean) / image_std).astype(np.float32)


def load_sequence_image(
    item: Dict[str, str],
    image_height: int,
    image_width: int,
    image_cache: Dict[str, np.ndarray],
    calibration_cache: Dict[str, object],
) -> np.ndarray:
    mode = str(item.get("mode", "direct")).strip()
    path = str(item.get("path", "")).strip()
    cache_key = f"{mode}:{path}"
    cached = image_cache.get(cache_key)
    if cached is not None:
        return cached
    if mode == "direct":
        image = load_single_image(Path(path), image_height=image_height, image_width=image_width).astype(np.float32)
    elif mode == "rectify_from_photo":
        calibration_path = str(item.get("calibration_path", "")).strip()
        image = load_rectified_image_from_photo(
            photo_path=Path(path),
            calibration_path=Path(calibration_path),
            image_height=image_height,
            image_width=image_width,
            calibration_cache=calibration_cache,
        )
    else:
        raise RuntimeError(f"unsupported image mode: {mode}")
    image_cache[cache_key] = image
    return image


def load_rectified_image_from_photo(
    photo_path: Path,
    calibration_path: Path,
    image_height: int,
    image_width: int,
    calibration_cache: Dict[str, object],
) -> np.ndarray:
    calibration_key = str(calibration_path.expanduser().resolve())
    calibration_v2 = calibration_cache.get(calibration_key)
    if calibration_v2 is None:
        calibration_v2 = load_v2_calibration(Path(calibration_key))
        calibration_cache[calibration_key] = calibration_v2

    photo_bgr = cv2.imread(str(photo_path), cv2.IMREAD_COLOR)
    if photo_bgr is None:
        raise RuntimeError(f"failed to read photo for rectification: {photo_path}")

    legacy = calibration_v2.legacy
    y0 = int(legacy.roi_y)
    y1 = int(legacy.roi_y + legacy.roi_h)
    x0 = int(legacy.roi_x)
    x1 = int(legacy.roi_x + legacy.roi_w)
    roi_bgr = photo_bgr[y0:y1, x0:x1]
    if roi_bgr.shape[0] != int(legacy.roi_h) or roi_bgr.shape[1] != int(legacy.roi_w):
        raise RuntimeError(f"roi crop mismatch for photo: {photo_path}")

    rectified_roi_bgr, _rectified_calibration = rectify_roi_and_calibration(roi_bgr, legacy)
    gray = cv2.cvtColor(rectified_roi_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (int(image_width), int(image_height)), interpolation=cv2.INTER_LINEAR)
    image_np = np.asarray(gray, dtype=np.float32) / 255.0
    return image_np[None, :, :]


def predict_visual_samples(
    checkpoint: Dict,
    samples: Sequence[Dict[str, object]],
    device: torch.device,
    batch_size: int,
) -> List[float]:
    if not samples:
        return []
    model_type = str(checkpoint["model_type"])
    image_height = int(checkpoint["image_height"])
    image_width = int(checkpoint["image_width"])
    image_mean = np.asarray(checkpoint["image_mean"], dtype=np.float32).reshape(1, 1, 1)
    image_std = np.asarray(checkpoint["image_std"], dtype=np.float32).reshape(1, 1, 1)
    target_mean = float(checkpoint["target_mean"])
    target_std = float(checkpoint["target_std"])

    model = build_model_from_visual_checkpoint(checkpoint).to(device=device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    preds: List[float] = []
    batch_size = max(1, int(batch_size))
    image_cache: Dict[str, np.ndarray] = {}
    calibration_cache: Dict[str, object] = {}
    with torch.no_grad():
        for start in range(0, len(samples), batch_size):
            batch_samples = samples[start : start + batch_size]
            x_np = np.asarray(
                [
                    load_sample_tensor(
                        sample=sample,
                        model_type=model_type,
                        image_height=image_height,
                        image_width=image_width,
                        image_mean=image_mean,
                        image_std=image_std,
                        image_cache=image_cache,
                        calibration_cache=calibration_cache,
                    )
                    for sample in batch_samples
                ],
                dtype=np.float32,
            )
            pred_norm = model(torch.from_numpy(x_np).to(device=device)).detach().cpu().numpy().reshape(-1)
            preds.extend([float(value * target_std + target_mean) for value in pred_norm])
    return preds


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


def render_curve_png(path: Path, rows: Sequence[Dict[str, object]], dpi: int):
    if plt is None:
        raise RuntimeError(f"matplotlib unavailable: {PLOT_IMPORT_ERROR}")
    times = [float(row["relative_time_s"]) for row in rows]
    visual_values = [float(row["pred_visual"]) for row in rows]
    human_values = [finite_float(row.get("target_human_peak_mm")) for row in rows]
    slosh_values = [finite_float(row.get("baseline_slosh_height_mm")) for row in rows]

    fig, ax = plt.subplots(1, 1, figsize=(14, 4.8))
    if any(value is not None for value in human_values):
        ax.plot(
            times,
            [float("nan") if value is None else float(value) for value in human_values],
            color="#202020",
            linewidth=1.8,
            label="human peak",
        )
    if any(value is not None for value in slosh_values):
        ax.plot(
            times,
            [float("nan") if value is None else float(value) for value in slosh_values],
            color="#5b8fd1",
            linewidth=1.5,
            alpha=0.95,
            label="bag /slosh/height",
        )
    ax.plot(times, visual_values, color="#d9485f", linewidth=1.8, label="SL visual")
    ax.set_xlabel("relative_time_s")
    ax.set_ylabel("height (mm)")
    ax.set_title("Visual SL continuous liquid-level curve")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=int(dpi), bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    try:
        args = parse_args()
        debug_dir = Path(args.debug_dir).expanduser().resolve()
        checkpoint_path = Path(args.checkpoint).expanduser().resolve()
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)

        image_column = str(args.image_column).strip() or str(checkpoint.get("image_column", "roi_debug_path"))
        manifest_csv_raw = str(args.manifest_csv).strip() or str(checkpoint.get("manifest_csv", "")).strip()
        raw_roi_root = Path(args.raw_roi_root).expanduser().resolve() if str(args.raw_roi_root).strip() else None
        out_dir = (
            Path(args.out_dir).expanduser().resolve()
            if str(args.out_dir).strip()
            else (debug_dir / "SL_infer_visual_checkpoint").resolve()
        )
        ensure_dir(out_dir)

        rows, session_meta = build_rows_for_inference(debug_dir)
        session_id = str(rows[0].get("session_id", "")) if rows else ""
        bag_id = str(rows[0].get("bag_id", "")) if rows else ""

        manifest_lookup: Dict[int, str] = {}
        if manifest_csv_raw:
            manifest_csv = Path(manifest_csv_raw).expanduser().resolve()
            if manifest_csv.exists():
                manifest_lookup = manifest_image_lookup(
                    manifest_csv=manifest_csv,
                    session_id=session_id,
                    bag_id=bag_id,
                    image_column=image_column,
                )
        if raw_roi_root is None and image_column == "raw_rectified_roi_path" and manifest_lookup:
            raw_roi_root = infer_raw_roi_root_from_paths(list(manifest_lookup.values()))

        resolved_rows, resolve_skip_counts = build_resolved_rows(
            rows=rows,
            image_column=image_column,
            manifest_lookup=manifest_lookup,
            raw_roi_root=raw_roi_root,
        )

        model_type = str(checkpoint.get("model_type", "")).strip()
        samples = build_visual_samples(
            rows=resolved_rows,
            image_column=image_column,
            model_type=model_type,
            history_frames=int(checkpoint.get("history_frames", 1)),
            history_step=int(checkpoint.get("history_step", 1)),
        )
        if not samples:
            raise RuntimeError("no visual samples could be constructed for inference")

        device = choose_device(args.device)
        preds = predict_visual_samples(
            checkpoint=checkpoint,
            samples=samples,
            device=device,
            batch_size=int(args.batch_size),
        )

        output_rows: List[Dict[str, object]] = []
        for sample, pred in zip(samples, preds):
            output_rows.append(
                {
                    "split": "infer",
                    "row_id": sample["row_id"],
                    "session_id": sample["session_id"],
                    "session_name": sample["session_name"],
                    "bag_id": sample["bag_id"],
                    "frame_index": sample["frame_index"],
                    "relative_time_s": sample["relative_time_s"],
                    "image_path": sample["image_path"],
                    "history_indices": ",".join(str(idx) for idx in sample["history_indices"]),
                    "target_human_peak_mm": "" if sample["target_human_peak_mm"] is None else float(sample["target_human_peak_mm"]),
                    "baseline_slosh_height_mm": "" if sample["baseline_slosh_height_mm"] is None else float(sample["baseline_slosh_height_mm"]),
                    "pred_train_mean_baseline": "",
                    "baseline_peak_rel_mm_v2": "" if sample["baseline_peak_rel_mm_v2"] is None else float(sample["baseline_peak_rel_mm_v2"]),
                    "pred_peak_rel_mm_v2_affine": "",
                    "baseline_center_rel_mm_v2": "" if sample["baseline_center_rel_mm_v2"] is None else float(sample["baseline_center_rel_mm_v2"]),
                    "pred_center_rel_mm_v2_affine": "",
                    "pred_visual": float(pred),
                    "pred_visual_temporal": float(pred),
                }
            )

        predictions_csv = out_dir / "SL_infer_visual_predictions.csv"
        summary_json = out_dir / "SL_infer_visual_summary.json"
        curve_png = out_dir / "SL_infer_visual_curve.png"
        write_predictions_csv(predictions_csv, output_rows)
        render_curve_png(curve_png, output_rows, dpi=int(args.dpi))

        valid_target_rows = [row for row in output_rows if finite_float(row.get("target_human_peak_mm")) is not None]
        visual_metrics = None
        slosh_metrics = None
        if valid_target_rows:
            targets = [float(row["target_human_peak_mm"]) for row in valid_target_rows]
            visual = [float(row["pred_visual"]) for row in valid_target_rows]
            visual_metrics = regression_metrics(targets, visual)
            slosh_rows = [row for row in valid_target_rows if finite_float(row.get("baseline_slosh_height_mm")) is not None]
            if slosh_rows:
                slosh_metrics = regression_metrics(
                    [float(row["target_human_peak_mm"]) for row in slosh_rows],
                    [float(row["baseline_slosh_height_mm"]) for row in slosh_rows],
                )

        summary = {
            "debug_dir": str(debug_dir),
            "checkpoint": str(checkpoint_path),
            "model_type": model_type,
            "device": str(device),
            "image_column": image_column,
            "num_rows_in_debug_session": len(rows),
            "num_rows_used": len(output_rows),
            "resolve_skip_counts": resolve_skip_counts,
            "session_meta": session_meta,
            "predictions_csv": str(predictions_csv),
            "curve_png": str(curve_png),
            "visual_metrics_on_labeled_rows": visual_metrics,
            "slosh_metrics_on_labeled_rows": slosh_metrics,
        }
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] predictions csv: {predictions_csv}")
        print(f"[OK] curve png: {curve_png}")
        print(f"[OK] summary json: {summary_json}")
        print(
            f"[OK] rows used: {len(output_rows)} / {len(rows)} "
            f"| image_column={image_column} model_type={model_type} device={device}"
        )
        if visual_metrics is not None:
            print(f"[OK] labeled-row visual metrics: {visual_metrics}")
        show_first = max(0, int(args.show_first))
        for row in output_rows[:show_first]:
            print(
                "[OK] row={row_id} frame={frame_index} pred_visual={pred_visual:.6f} target={target_human_peak_mm}".format(
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
