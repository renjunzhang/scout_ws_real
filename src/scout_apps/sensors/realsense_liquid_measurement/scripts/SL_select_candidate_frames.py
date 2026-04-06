#!/usr/bin/env python3
"""Rank high-value unlabeled frames for additional human peak annotation."""

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import torch

from FSL_train_visual_pseudolabel import MinimalVisualRegressor
from SL_train_baseline import choose_device
from SL_supervised_common import finite_float, read_csv_rows
from extract_liquid_height_from_bag import rectify_roi_and_calibration
from extract_liquid_height_v2_from_bag import load_v2_calibration


DEFAULT_OUT_DIR = "/data/a/realsense_validation_v2/sl_candidate_frames/0401_train_hard_examples"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run the current SL visual model on full debug sessions and rank high-value unlabeled frames "
            "for additional human_peak annotation."
        )
    )
    parser.add_argument(
        "--debug-dir",
        action="append",
        default=[],
        help="Path to one debug-session directory. Can be passed multiple times.",
    )
    parser.add_argument(
        "--debug-root",
        action="append",
        default=[],
        help="Root directory searched recursively for debug sessions containing debug_session.csv.",
    )
    parser.add_argument(
        "--checkpoint",
        default="/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/sl_runs/SL_visual_human_0401_raw_roi_v1/SL_visual_human.pt",
        help="Path to a trained visual SL checkpoint. Default: current best raw ROI model.",
    )
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUT_DIR,
        help=f"Output directory for ranked candidates. Default: {DEFAULT_OUT_DIR}",
    )
    parser.add_argument(
        "--skip-labeled",
        action="store_true",
        help="Skip rows that already have human_peak_mm. Recommended for补标.",
    )
    parser.add_argument(
        "--top-k-per-bag",
        type=int,
        default=40,
        help="Number of selected candidate frames per bag after de-dup. Default: 40.",
    )
    parser.add_argument(
        "--min-frame-gap",
        type=int,
        default=10,
        help="Minimum frame gap between selected candidates within the same bag. Default: 10.",
    )
    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.75,
        help="confidence_v2 threshold used to mark low-confidence frames. Default: 0.75.",
    )
    parser.add_argument(
        "--amplitude-weight",
        type=float,
        default=1.0,
        help="Score weight for large predicted/estimated peak amplitude. Default: 1.0.",
    )
    parser.add_argument(
        "--disagreement-weight",
        type=float,
        default=1.5,
        help="Score weight for |SL - /slosh/height| disagreement. Default: 1.5.",
    )
    parser.add_argument(
        "--v2-fail-bonus",
        type=float,
        default=0.35,
        help="Score bonus when v2 accept gate fails. Default: 0.35.",
    )
    parser.add_argument(
        "--low-confidence-bonus",
        type=float,
        default=0.25,
        help="Score bonus multiplier for low confidence_v2. Default: 0.25.",
    )
    parser.add_argument(
        "--peak-v2-cap-mm",
        type=float,
        default=5.0,
        help="Cap for peak_rel_mm_v2 when it participates in amplitude scoring. Default: 5.0.",
    )
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--batch-size", type=int, default=64, help="Prediction batch size. Default: 64.")
    parser.add_argument("--show-first", type=int, default=10, help="Print the first N selected rows. Default: 10.")
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def discover_debug_dirs(args) -> List[Path]:
    paths: List[Path] = []
    seen = set()
    for raw in args.debug_dir:
        path = Path(raw).expanduser().resolve()
        if path.is_dir() and (path / "debug_session.csv").exists():
            key = str(path)
            if key not in seen:
                seen.add(key)
                paths.append(path)
    for raw in args.debug_root:
        root = Path(raw).expanduser().resolve()
        if not root.exists():
            continue
        for session_csv in sorted(root.rglob("debug_session.csv")):
            path = session_csv.parent.resolve()
            key = str(path)
            if key not in seen:
                seen.add(key)
                paths.append(path)
    if not paths:
        raise RuntimeError("no debug-session directories found")
    return paths


def infer_session_identity(debug_dir: Path, session_meta: Dict) -> Tuple[str, str, str]:
    bag_raw = str(session_meta.get("bag", "")).strip()
    bag_id = Path(bag_raw).stem if bag_raw else debug_dir.name
    date_id = debug_dir.parent.name
    session_id = f"{date_id}/{debug_dir.name}" if date_id else debug_dir.name
    return session_id, date_id, bag_id


def load_json(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise RuntimeError(f"json root is not an object: {path}")
    return data


def load_debug_rows(debug_dir: Path) -> List[Dict[str, str]]:
    session_csv = debug_dir / "debug_session.csv"
    session_json = debug_dir / "debug_session.json"
    rows = read_csv_rows(session_csv)
    meta = load_json(session_json)
    session_id, date_id, bag_id = infer_session_identity(debug_dir, meta)
    calibration_path = str(meta.get("calibration", "")).strip()
    bag_path = str(meta.get("bag", "")).strip()

    prepared: List[Dict[str, str]] = []
    for raw in rows:
        row = dict(raw)
        frame_index = int(float(row.get("frame_index", -1) or -1))
        row["row_id"] = f"{session_id}:{frame_index}"
        row["session_id"] = session_id
        row["session_name"] = debug_dir.name
        row["date_id"] = date_id
        row["bag_id"] = bag_id
        row["bag_path"] = bag_path
        row["debug_dir"] = str(debug_dir)
        row["calibration_path"] = calibration_path
        prepared.append(row)
    return prepared


def build_model(checkpoint: Dict, device: torch.device):
    model = MinimalVisualRegressor(
        image_height=int(checkpoint["image_height"]),
        image_width=int(checkpoint["image_width"]),
        hidden_dim=int(checkpoint["hidden_dim"]),
    ).to(device=device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    return model


def preprocess_rectified_roi(
    row: Dict[str, str],
    calibration_cache: Dict[str, object],
    image_height: int,
    image_width: int,
    image_mean: Sequence[float],
    image_std: Sequence[float],
) -> Optional[np.ndarray]:
    photo_path = Path(str(row.get("photo_path", "")).strip()).expanduser()
    calibration_path = Path(str(row.get("calibration_path", "")).strip()).expanduser()
    if not photo_path.exists() or not calibration_path.exists():
        return None

    calibration_key = str(calibration_path.resolve())
    calibration_v2 = calibration_cache.get(calibration_key)
    if calibration_v2 is None:
        calibration_v2 = load_v2_calibration(Path(calibration_key))
        calibration_cache[calibration_key] = calibration_v2

    legacy = calibration_v2.legacy
    photo_bgr = cv2.imread(str(photo_path), cv2.IMREAD_COLOR)
    if photo_bgr is None:
        return None

    y0 = int(legacy.roi_y)
    y1 = int(legacy.roi_y + legacy.roi_h)
    x0 = int(legacy.roi_x)
    x1 = int(legacy.roi_x + legacy.roi_w)
    roi_bgr = photo_bgr[y0:y1, x0:x1]
    if roi_bgr.shape[0] != int(legacy.roi_h) or roi_bgr.shape[1] != int(legacy.roi_w):
        return None

    rectified_roi_bgr, _ = rectify_roi_and_calibration(roi_bgr, legacy)
    gray = cv2.cvtColor(rectified_roi_bgr, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (int(image_width), int(image_height)), interpolation=cv2.INTER_LINEAR)
    image_np = resized.astype(np.float32) / 255.0
    x = image_np[None, :, :]
    mean = np.asarray(image_mean, dtype=np.float32).reshape(-1, 1, 1)
    std = np.asarray(image_std, dtype=np.float32).reshape(-1, 1, 1)
    std[std < 1e-6] = 1.0
    return ((x - mean) / std).astype(np.float32)


def predict_rows(
    rows: Sequence[Dict[str, str]],
    checkpoint: Dict,
    device: torch.device,
    batch_size: int,
) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
    image_height = int(checkpoint["image_height"])
    image_width = int(checkpoint["image_width"])
    image_mean = checkpoint["image_mean"]
    image_std = checkpoint["image_std"]
    target_mean = float(checkpoint["target_mean"])
    target_std = float(checkpoint["target_std"])

    calibration_cache: Dict[str, object] = {}
    prepared: List[Dict[str, object]] = []
    skip_counts = Counter()
    batch_arrays: List[np.ndarray] = []
    batch_meta: List[Dict[str, object]] = []

    model = build_model(checkpoint, device=device)

    def flush_batch():
        if not batch_arrays:
            return
        x_np = np.asarray(batch_arrays, dtype=np.float32)
        with torch.no_grad():
            pred_norm = model(torch.from_numpy(x_np).to(device=device)).detach().cpu().numpy()
        for meta, pred_value in zip(batch_meta, pred_norm):
            meta["pred_sl_visual_mm"] = float(pred_value * target_std + target_mean)
            prepared.append(meta)
        batch_arrays.clear()
        batch_meta.clear()

    for row in rows:
        x = preprocess_rectified_roi(
            row=row,
            calibration_cache=calibration_cache,
            image_height=image_height,
            image_width=image_width,
            image_mean=image_mean,
            image_std=image_std,
        )
        if x is None:
            skip_counts["preprocess_failed"] += 1
            continue
        batch_arrays.append(x)
        batch_meta.append(
            {
                "row_id": str(row.get("row_id", "")),
                "session_id": str(row.get("session_id", "")),
                "session_name": str(row.get("session_name", "")),
                "date_id": str(row.get("date_id", "")),
                "bag_id": str(row.get("bag_id", "")),
                "bag_path": str(row.get("bag_path", "")),
                "debug_dir": str(row.get("debug_dir", "")),
                "frame_index": int(float(row.get("frame_index", -1) or -1)),
                "relative_time_s": float(finite_float(row.get("relative_time_s")) or 0.0),
                "photo_path": str(row.get("photo_path", "")),
                "roi_debug_path": str(row.get("roi_debug_path", "")),
                "human_peak_mm": finite_float(row.get("human_peak_mm")),
                "slosh_height_mm": finite_float(row.get("slosh_height_mm")),
                "peak_rel_mm_v2": finite_float(row.get("peak_rel_mm_v2")),
                "center_rel_mm_v2": finite_float(row.get("center_rel_mm_v2")),
                "accept_for_peak_report_v2": str(row.get("accept_for_peak_report_v2", "")),
                "confidence_v2": finite_float(row.get("confidence_v2")),
                "cmd_vel_linear_x": finite_float(row.get("cmd_vel_linear_x")),
                "cmd_vel_angular_z": finite_float(row.get("cmd_vel_angular_z")),
            }
        )
        if len(batch_arrays) >= max(1, int(batch_size)):
            flush_batch()
    flush_batch()
    return prepared, dict(skip_counts)


def bool_accept(raw_value: str) -> bool:
    text = str(raw_value).strip().lower()
    return text in {"1", "true", "yes"}


def score_row(
    row: Dict[str, object],
    confidence_threshold: float,
    amplitude_weight: float,
    disagreement_weight: float,
    v2_fail_bonus: float,
    low_confidence_bonus: float,
    peak_v2_cap_mm: float,
) -> Dict[str, object]:
    pred = finite_float(row.get("pred_sl_visual_mm"))
    slosh = finite_float(row.get("slosh_height_mm"))
    peak_v2 = finite_float(row.get("peak_rel_mm_v2"))
    confidence = finite_float(row.get("confidence_v2"))
    accept = bool_accept(str(row.get("accept_for_peak_report_v2", "")))

    peak_v2_for_score = 0.0
    if accept and peak_v2 is not None:
        peak_v2_for_score = min(abs(float(peak_v2)), float(peak_v2_cap_mm))

    finite_candidates = [abs(v) for v in [pred, slosh] if v is not None]
    if peak_v2_for_score > 0.0:
        finite_candidates.append(float(peak_v2_for_score))
    amplitude_signal = max(finite_candidates) if finite_candidates else 0.0
    disagreement_signal = 0.0 if pred is None or slosh is None else abs(float(pred) - float(slosh))
    low_conf_signal = 1.0
    if confidence is not None and confidence_threshold > 1e-6:
        low_conf_signal = max(0.0, float(confidence_threshold) - float(confidence)) / float(confidence_threshold)
    v2_fail_signal = 0.0 if accept else 1.0

    score = (
        float(amplitude_weight) * float(amplitude_signal)
        + float(disagreement_weight) * float(disagreement_signal)
        + float(v2_fail_bonus) * float(v2_fail_signal)
        + float(low_confidence_bonus) * float(low_conf_signal)
    )

    reasons: List[str] = []
    if amplitude_signal >= 1.0:
        reasons.append("high_peak")
    elif amplitude_signal >= 0.5:
        reasons.append("mid_peak")
    if disagreement_signal >= 0.5:
        reasons.append("large_sl_gap")
    elif disagreement_signal >= 0.2:
        reasons.append("sl_gap")
    if not accept:
        reasons.append("v2_fail")
    if confidence is None:
        reasons.append("missing_conf")
    elif confidence < confidence_threshold:
        reasons.append("low_conf")
    if not reasons:
        reasons.append("generic_hard_case")

    updated = dict(row)
    updated["score"] = float(score)
    updated["amplitude_signal_mm"] = float(amplitude_signal)
    updated["disagreement_signal_mm"] = float(disagreement_signal)
    updated["v2_fail_signal"] = float(v2_fail_signal)
    updated["low_conf_signal"] = float(low_conf_signal)
    updated["reason"] = "+".join(reasons)
    return updated


def select_top_candidates(
    scored_rows: Sequence[Dict[str, object]],
    top_k_per_bag: int,
    min_frame_gap: int,
) -> List[Dict[str, object]]:
    grouped: Dict[str, List[Dict[str, object]]] = defaultdict(list)
    for row in scored_rows:
        grouped[str(row.get("bag_id", ""))].append(dict(row))

    selected: List[Dict[str, object]] = []
    for bag_id, bag_rows in grouped.items():
        ranked = sorted(
            bag_rows,
            key=lambda row: (
                -float(row.get("score", 0.0)),
                -float(row.get("amplitude_signal_mm", 0.0)),
                int(row.get("frame_index", -1)),
            ),
        )
        chosen: List[Dict[str, object]] = []
        chosen_frames: List[int] = []
        for row in ranked:
            frame_index = int(row.get("frame_index", -1))
            if any(abs(frame_index - prev) < int(min_frame_gap) for prev in chosen_frames):
                continue
            row["rank_in_bag"] = len(chosen) + 1
            chosen.append(row)
            chosen_frames.append(frame_index)
            if len(chosen) >= int(top_k_per_bag):
                break
        selected.extend(chosen)
    return sorted(selected, key=lambda row: (str(row.get("bag_id", "")), int(row.get("rank_in_bag", 0))))


def write_csv(path: Path, rows: Sequence[Dict[str, object]], fieldnames: Sequence[str]):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def main() -> int:
    try:
        args = parse_args()
        debug_dirs = discover_debug_dirs(args)
        checkpoint_path = Path(args.checkpoint).expanduser().resolve()
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        device = choose_device(args.device)

        all_rows: List[Dict[str, str]] = []
        for debug_dir in debug_dirs:
            all_rows.extend(load_debug_rows(debug_dir))

        if args.skip_labeled:
            input_rows = [row for row in all_rows if finite_float(row.get("human_peak_mm")) is None]
        else:
            input_rows = list(all_rows)
        if not input_rows:
            raise RuntimeError("no rows available after skip-labeled filtering")

        predicted_rows, skip_counts = predict_rows(
            rows=input_rows,
            checkpoint=checkpoint,
            device=device,
            batch_size=int(args.batch_size),
        )
        if not predicted_rows:
            raise RuntimeError("no candidate rows could be predicted")

        scored_rows = [
            score_row(
                row=row,
                confidence_threshold=float(args.confidence_threshold),
                amplitude_weight=float(args.amplitude_weight),
                disagreement_weight=float(args.disagreement_weight),
                v2_fail_bonus=float(args.v2_fail_bonus),
                low_confidence_bonus=float(args.low_confidence_bonus),
                peak_v2_cap_mm=float(args.peak_v2_cap_mm),
            )
            for row in predicted_rows
        ]
        selected_rows = select_top_candidates(
            scored_rows=scored_rows,
            top_k_per_bag=int(args.top_k_per_bag),
            min_frame_gap=int(args.min_frame_gap),
        )

        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)
        selected_csv = out_dir / "SL_candidate_frames_selected.csv"
        all_csv = out_dir / "SL_candidate_frames_all_scored.csv"
        summary_json = out_dir / "SL_candidate_frames_summary.json"

        common_fields = [
            "row_id",
            "session_name",
            "bag_id",
            "frame_index",
            "relative_time_s",
            "human_peak_mm",
            "pred_sl_visual_mm",
            "slosh_height_mm",
            "peak_rel_mm_v2",
            "center_rel_mm_v2",
            "confidence_v2",
            "accept_for_peak_report_v2",
            "amplitude_signal_mm",
            "disagreement_signal_mm",
            "score",
            "reason",
            "photo_path",
            "roi_debug_path",
            "debug_dir",
        ]
        write_csv(selected_csv, selected_rows, ["rank_in_bag"] + common_fields)
        write_csv(all_csv, scored_rows, common_fields)

        bag_counts = Counter(str(row.get("bag_id", "")) for row in selected_rows)
        payload = {
            "checkpoint": str(checkpoint_path),
            "device": str(device),
            "num_debug_dirs": len(debug_dirs),
            "num_input_rows": len(input_rows),
            "num_predicted_rows": len(predicted_rows),
            "num_selected_rows": len(selected_rows),
            "skip_counts": skip_counts,
            "selection_config": {
                "skip_labeled": bool(args.skip_labeled),
                "top_k_per_bag": int(args.top_k_per_bag),
                "min_frame_gap": int(args.min_frame_gap),
                "confidence_threshold": float(args.confidence_threshold),
                "amplitude_weight": float(args.amplitude_weight),
                "disagreement_weight": float(args.disagreement_weight),
                "v2_fail_bonus": float(args.v2_fail_bonus),
                "low_confidence_bonus": float(args.low_confidence_bonus),
                "peak_v2_cap_mm": float(args.peak_v2_cap_mm),
            },
            "selected_bag_counts": dict(bag_counts),
            "selected_csv": str(selected_csv),
            "all_scored_csv": str(all_csv),
        }
        summary_json.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

        print(f"[OK] selected csv: {selected_csv}")
        print(f"[OK] all scored csv: {all_csv}")
        print(f"[OK] summary json: {summary_json}")
        print(
            f"[OK] debug_dirs={len(debug_dirs)} input_rows={len(input_rows)} "
            f"predicted_rows={len(predicted_rows)} selected_rows={len(selected_rows)}"
        )
        for row in selected_rows[: max(0, int(args.show_first))]:
            print(
                "[OK] {bag_id} frame={frame_index} score={score:.3f} pred={pred_sl_visual_mm:.3f} "
                "slosh={slosh_height_mm} reason={reason}".format(**row)
            )
        return 0
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
