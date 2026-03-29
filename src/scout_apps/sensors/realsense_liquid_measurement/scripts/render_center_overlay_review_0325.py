#!/usr/bin/env python3
"""Render clean ROI review images for judging whether RealSense center is visually high or /slosh/height is low."""

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

from extract_liquid_height_from_bag import rectify_roi_and_calibration
from extract_liquid_height_v2_from_bag import V2Calibration, evaluate_y_at_height, load_v2_calibration


DEFAULT_DEBUG_DIR = Path("/data/a/realsense_validation_v2/debug/0325/Q5_test1")
DEFAULT_SEGMENT_CSV = Path("/data/a/realsense_validation_v2/verify/0325/q5_phase_outlier_analysis/Q5_test1_outlier_segments.csv")
DEFAULT_CALIBRATION = Path("/data/a/realsense_validation_v2/calibration/0325/scene_0325_multiscale_raw.yaml")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug-dir", default=str(DEFAULT_DEBUG_DIR))
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--segment-csv", default=str(DEFAULT_SEGMENT_CSV))
    parser.add_argument("--out-dir", required=True)
    parser.add_argument(
        "--frame-indices",
        default="",
        help="Optional comma-separated frame indices. If empty, use segments from --segment-csv.",
    )
    parser.add_argument(
        "--context-frames",
        type=int,
        default=2,
        help="Extra frames before/after each segment when using --segment-csv.",
    )
    parser.add_argument(
        "--max-frames-per-segment",
        type=int,
        default=8,
        help="Maximum rendered frames per segment after adding context.",
    )
    parser.add_argument(
        "--visual-zero-offset-mm",
        type=float,
        default=0.0,
        help=(
            "Optional visual-only zero offset in mm. Positive values shift 0 mm / center / /slosh/height "
            "together upward on the rectified ROI."
        ),
    )
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def finite_float(value) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def load_debug_rows(debug_csv: Path) -> Dict[int, Dict[str, object]]:
    rows: Dict[int, Dict[str, object]] = {}
    with debug_csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            frame_index = int(raw["frame_index"])
            rows[frame_index] = {
                "frame_index": frame_index,
                "relative_time_s": finite_float(raw.get("relative_time_s")),
                "photo_path": raw.get("photo_path", ""),
                "center_y_rect": finite_float(raw.get("center_y_rect")),
                "center_rel_mm_v2": finite_float(raw.get("center_rel_mm_v2")),
                "slosh_height_mm": finite_float(raw.get("slosh_height_mm")),
                "confidence_v2": finite_float(raw.get("confidence_v2")),
                "fit_rms_px_v2": finite_float(raw.get("fit_rms_px_v2")),
                "accept_for_peak_report_v2": int(raw.get("accept_for_peak_report_v2", "0") or "0"),
            }
    return rows


def parse_frame_indices(raw_value: str) -> List[int]:
    values = []
    for token in raw_value.split(","):
        token = token.strip()
        if not token:
            continue
        values.append(int(token))
    return sorted(set(values))


def load_segment_ranges(path: Path) -> List[Dict[str, int]]:
    ranges: List[Dict[str, int]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            ranges.append(
                {
                    "rank": int(raw["rank"]),
                    "start_frame_index": int(raw["start_frame_index"]),
                    "end_frame_index": int(raw["end_frame_index"]),
                }
            )
    return ranges


def select_segment_frames(
    start_frame: int,
    end_frame: int,
    context_frames: int,
    max_frames: int,
    available_indices: Sequence[int],
) -> List[int]:
    desired_start = start_frame - context_frames
    desired_end = end_frame + context_frames
    selected = [idx for idx in available_indices if desired_start <= idx <= desired_end]
    if len(selected) <= max_frames:
        return selected
    if max_frames <= 1:
        return [selected[len(selected) // 2]]
    sample_positions = np.linspace(0, len(selected) - 1, num=max_frames)
    sampled = sorted({selected[int(round(pos))] for pos in sample_positions})
    return sampled


def render_review_image(
    photo_bgr: np.ndarray,
    row: Dict[str, object],
    calibration: V2Calibration,
    visual_zero_offset_mm: float,
) -> np.ndarray:
    roi_x = int(calibration.legacy.roi_x)
    roi_y = int(calibration.legacy.roi_y)
    roi_w = int(calibration.legacy.roi_w)
    roi_h = int(calibration.legacy.roi_h)
    photo = photo_bgr.copy()
    cv2.rectangle(photo, (roi_x, roi_y), (roi_x + roi_w - 1, roi_y + roi_h - 1), (0, 255, 255), 2)

    roi_bgr = photo_bgr[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w].copy()
    rectified_roi, _ = rectify_roi_and_calibration(roi_bgr, calibration.legacy)
    roi_panel = rectified_roi.copy()

    panel_h, panel_w = roi_panel.shape[:2]
    zero_y = int(round(evaluate_y_at_height(calibration.reference_points, float(visual_zero_offset_mm))))
    center_y = int(round(float(row["center_y_rect"]))) if math.isfinite(float(row["center_y_rect"])) else None
    slosh_mm = float(row["slosh_height_mm"])
    if center_y is not None and math.isfinite(float(visual_zero_offset_mm)):
        shifted_center_mm = float(row["center_rel_mm_v2"]) + float(visual_zero_offset_mm)
        center_y = int(round(evaluate_y_at_height(calibration.reference_points, shifted_center_mm)))
    slosh_y = (
        int(round(evaluate_y_at_height(calibration.reference_points, slosh_mm + float(visual_zero_offset_mm))))
        if math.isfinite(slosh_mm)
        else None
    )

    def draw_hline(image: np.ndarray, y: Optional[int], color, label: str, x0: int = 0):
        if y is None or y < 0 or y >= image.shape[0]:
            return
        cv2.line(image, (0, y), (image.shape[1] - 1, y), color, 2, cv2.LINE_AA)
        text_y = max(16, min(image.shape[0] - 8, y - 6))
        cv2.putText(image, label, (x0 + 8, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    draw_hline(roi_panel, zero_y, (255, 255, 0), "0 mm")
    draw_hline(roi_panel, center_y, (0, 0, 255), "RealSense center")
    draw_hline(roi_panel, slosh_y, (255, 0, 0), "/slosh/height")

    photo_h = photo.shape[0]
    scale = min(1.0, 720.0 / max(photo_h, panel_h))
    if scale < 1.0:
        photo = cv2.resize(photo, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        roi_panel = cv2.resize(roi_panel, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    photo_h, photo_w = photo.shape[:2]
    panel_h, panel_w = roi_panel.shape[:2]
    footer_h = 120
    canvas_h = max(photo_h, panel_h) + footer_h
    canvas_w = photo_w + panel_w + 24
    canvas = np.full((canvas_h, canvas_w, 3), 248, dtype=np.uint8)
    canvas[:photo_h, :photo_w] = photo
    canvas[:panel_h, photo_w + 24 : photo_w + 24 + panel_w] = roi_panel

    lines = [
        f"frame={int(row['frame_index'])}  t={float(row['relative_time_s']):.3f}s",
        f"center={float(row['center_rel_mm_v2']):.3f} mm   /slosh/height={slosh_mm:.3f} mm",
        f"visual_zero_offset={float(visual_zero_offset_mm):.3f} mm",
        f"confidence={float(row['confidence_v2']):.3f}   fit_rms={float(row['fit_rms_px_v2']):.3f} px   reportable={int(row['accept_for_peak_report_v2'])}",
        "Judgement rule: if the visible meniscus is below the red line while the blue line is closer to the meniscus, then RealSense center is visually too high.",
    ]
    footer_y = max(photo_h, panel_h) + 24
    for index, text in enumerate(lines):
        cv2.putText(
            canvas,
            text,
            (12, footer_y + index * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62 if index < 4 else 0.55,
            (32, 32, 32),
            2 if index < 4 else 1,
            cv2.LINE_AA,
        )

    return canvas


def main():
    args = parse_args()
    debug_dir = Path(args.debug_dir).expanduser().resolve()
    calibration_path = Path(args.calibration).expanduser().resolve()
    segment_csv = Path(args.segment_csv).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    ensure_dir(out_dir)

    debug_csv = debug_dir / "debug_session.csv"
    if not debug_csv.is_file():
        raise SystemExit(f"[ERROR] missing {debug_csv}")

    calibration = load_v2_calibration(calibration_path)
    debug_rows = load_debug_rows(debug_csv)
    available_indices = sorted(debug_rows.keys())
    if not available_indices:
        raise SystemExit(f"[ERROR] no rows in {debug_csv}")

    targets: List[Dict[str, object]] = []
    manual_indices = parse_frame_indices(args.frame_indices)
    if manual_indices:
        targets.append({"name": "manual_frames", "frames": [idx for idx in manual_indices if idx in debug_rows]})
    else:
        if not segment_csv.is_file():
            raise SystemExit(f"[ERROR] missing {segment_csv}")
        for segment in load_segment_ranges(segment_csv):
            frames = select_segment_frames(
                segment["start_frame_index"],
                segment["end_frame_index"],
                args.context_frames,
                args.max_frames_per_segment,
                available_indices,
            )
            targets.append(
                {
                    "name": f"segment_{segment['rank']:02d}_frames_{segment['start_frame_index']}_{segment['end_frame_index']}",
                    "frames": frames,
                }
            )

    summary_rows: List[Dict[str, object]] = []
    for target in targets:
        target_dir = out_dir / str(target["name"])
        ensure_dir(target_dir)
        for frame_index in target["frames"]:
            row = debug_rows[int(frame_index)]
            photo_path = Path(str(row["photo_path"]))
            photo_bgr = cv2.imread(str(photo_path), cv2.IMREAD_COLOR)
            if photo_bgr is None:
                raise SystemExit(f"[ERROR] failed to read image: {photo_path}")
            canvas = render_review_image(photo_bgr, row, calibration, args.visual_zero_offset_mm)
            out_path = target_dir / f"frame_{int(frame_index):06d}_review.png"
            cv2.imwrite(str(out_path), canvas)
            summary_rows.append(
                {
                    "group": str(target["name"]),
                    "frame_index": int(frame_index),
                    "relative_time_s": f"{float(row['relative_time_s']):.6f}",
                    "center_rel_mm_v2": f"{float(row['center_rel_mm_v2']):.6f}",
                    "slosh_height_mm": f"{float(row['slosh_height_mm']):.6f}",
                    "center_y_rect": f"{float(row['center_y_rect']):.6f}",
                    "confidence_v2": f"{float(row['confidence_v2']):.6f}",
                    "fit_rms_px_v2": f"{float(row['fit_rms_px_v2']):.6f}",
                    "visual_zero_offset_mm": f"{float(args.visual_zero_offset_mm):.6f}",
                    "output_path": str(out_path),
                }
            )

    summary_csv = out_dir / "review_summary.csv"
    with summary_csv.open("w", encoding="utf-8", newline="") as handle:
        fieldnames = list(summary_rows[0].keys()) if summary_rows else [
            "group",
            "frame_index",
            "relative_time_s",
            "center_rel_mm_v2",
            "slosh_height_mm",
            "center_y_rect",
            "confidence_v2",
            "fit_rms_px_v2",
            "output_path",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    readme = out_dir / "README.md"
    lines = [
        "# Center Overlay Review",
        "",
        "本目录用于逐帧判断 `RealSense center` 是否在视觉上偏高，还是 `/slosh/height` 更可能偏低。",
        "",
        f"当前可视零位偏移：`{float(args.visual_zero_offset_mm):.3f} mm`",
        "",
        "颜色定义：",
        "- 青色：`0 mm` 参考线",
        "- 红色：`RealSense center`",
        "- 蓝色：`/slosh/height` 映射到标尺后的高度线",
        "",
        "判断建议：",
        "- 如果实际液面明显低于红线，而蓝线更靠近液面，说明 `center` 偏高。",
        "- 如果红线更贴近液面，而蓝线明显更低，说明 `/slosh/height` 偏低。",
        "- 如果两条线都离实际液面远，说明不只是时延，可能还有幅值或建模问题。",
        "",
        f"总导出帧数：{len(summary_rows)}",
        f"汇总表：`{summary_csv.name}`",
        "",
    ]
    readme.write_text("\n".join(lines), encoding="utf-8")

    print(f"[OK] out dir: {out_dir}")
    print(f"[OK] review summary csv: {summary_csv}")
    print(f"[OK] readme: {readme}")
    print(f"[OK] exported frames: {len(summary_rows)}")


if __name__ == "__main__":
    main()
