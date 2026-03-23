#!/usr/bin/env python3
"""Interactively annotate ROI and line-based reference geometry for liquid measurement."""

import argparse
import math
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List

import cv2
import yaml


WINDOW_NAME = "Liquid ROI Annotation"


@dataclass
class Point:
    x: int
    y: int


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactively annotate ROI, wall lines, still-level line, tube axis, and optional ruler points."
    )
    parser.add_argument("--image", required=True, help="Path to the exported PNG/JPG image.")
    parser.add_argument(
        "--mode",
        choices=["px_only", "ruler"],
        default="px_only",
        help="Calibration mode: px_only for pixel-domain calibration, ruler when a background ruler is available.",
    )
    parser.add_argument(
        "--output-yaml",
        default="",
        help="Output YAML path. Defaults to <image_stem>_calibration.yaml next to the image.",
    )
    parser.add_argument(
        "--output-image",
        default="",
        help="Annotated preview image path. Defaults to <image_stem>_annotated.png next to the image.",
    )
    parser.add_argument(
        "--calibration-distance-mm",
        type=float,
        default=None,
        help="Physical distance in mm between the two ruler points. Required only in ruler mode.",
    )
    return parser.parse_args()


def default_output_yaml(image_path: Path) -> Path:
    return image_path.parent / f"{image_path.stem}_calibration.yaml"


def default_output_image(image_path: Path) -> Path:
    return image_path.parent / f"{image_path.stem}_annotated.png"


def point_in_roi(point: Point, roi):
    x, y, w, h = roi
    return x <= point.x <= x + w and y <= point.y <= y + h


def line_complete(state: Dict[str, Point], key_a: str, key_b: str) -> bool:
    return state[key_a] is not None and state[key_b] is not None


def draw_point(canvas, point: Point, color, radius=4):
    if point is None:
        return
    cv2.circle(canvas, (point.x, point.y), radius, color, -1)


def draw_line(canvas, state: Dict[str, Point], key_a: str, key_b: str, color, thickness=2):
    if not line_complete(state, key_a, key_b):
        return
    cv2.line(canvas, (state[key_a].x, state[key_a].y), (state[key_b].x, state[key_b].y), color, thickness)


def sort_line_by_y(p1: Point, p2: Point):
    return (p1, p2) if p1.y <= p2.y else (p2, p1)


def sort_line_by_x(p1: Point, p2: Point):
    return (p1, p2) if p1.x <= p2.x else (p2, p1)


def avg_x(points: List[Point]) -> float:
    return sum(point.x for point in points) / float(len(points))


def avg_y(points: List[Point]) -> float:
    return sum(point.y for point in points) / float(len(points))


def rel_point(point: Point, roi) -> List[int]:
    return [int(point.x - roi[0]), int(point.y - roi[1])]


def rel_line(points, roi) -> List[List[int]]:
    return [rel_point(points[0], roi), rel_point(points[1], roi)]


def evaluate_x_at_y(points, y_value: float) -> float:
    p0, p1 = points
    if p1.y == p0.y:
        return 0.5 * (p0.x + p1.x)
    alpha = (float(y_value) - float(p0.y)) / float(p1.y - p0.y)
    return float(p0.x + alpha * (p1.x - p0.x))


def line_vertical_span(points) -> float:
    return float(abs(points[1].y - points[0].y))


def geometry_checks(roi, left_wall_line, right_wall_line, still_level_line, tube_axis_line) -> List[str]:
    warnings: List[str] = []
    x, y, w, h = roi
    y_samples = [float(y), float(y + 0.5 * h), float(y + h)]
    roi_h = max(1.0, float(h))
    min_wall_coverage_ratio = 0.50
    min_axis_coverage_ratio = 0.50

    left_wall_coverage = line_vertical_span(left_wall_line) / roi_h
    right_wall_coverage = line_vertical_span(right_wall_line) / roi_h
    axis_coverage = line_vertical_span(tube_axis_line) / roi_h

    if left_wall_coverage < min_wall_coverage_ratio:
        warnings.append(
            "Left wall line covers too little vertical range inside ROI; extend it higher/lower."
        )
    if right_wall_coverage < min_wall_coverage_ratio:
        warnings.append(
            "Right wall line covers too little vertical range inside ROI; extend it higher/lower."
        )
    if axis_coverage < min_axis_coverage_ratio:
        warnings.append(
            "Tube axis line covers too little vertical range; extend it across more of the tube height."
        )

    for y_value in y_samples:
        left_x = evaluate_x_at_y(left_wall_line, y_value)
        right_x = evaluate_x_at_y(right_wall_line, y_value)
        if left_x >= right_x:
            warnings.append("Left/right wall lines cross or overlap inside ROI.")
            break

    still_y = avg_y(list(still_level_line))
    left_x_at_still = evaluate_x_at_y(left_wall_line, still_y)
    right_x_at_still = evaluate_x_at_y(right_wall_line, still_y)
    if still_level_line[0].x <= left_x_at_still or still_level_line[1].x >= right_x_at_still:
        warnings.append("Still-level line extends too close to, or beyond, tube walls.")

    axis_center_offsets = []
    for point in tube_axis_line:
        axis_x = float(point.x)
        wall_left_x = evaluate_x_at_y(left_wall_line, point.y)
        wall_right_x = evaluate_x_at_y(right_wall_line, point.y)
        if axis_x <= wall_left_x or axis_x >= wall_right_x:
            warnings.append("Tube axis line falls outside wall lines.")
            break
        half_width = max(1.0, 0.5 * (wall_right_x - wall_left_x))
        center_x = 0.5 * (wall_left_x + wall_right_x)
        axis_center_offsets.append(abs(axis_x - center_x) / half_width)

    if axis_center_offsets and max(axis_center_offsets) > 0.25:
        warnings.append("Tube axis line is noticeably off center (>25% of half-width).")

    wall_widths = []
    for y_value in y_samples:
        wall_widths.append(evaluate_x_at_y(right_wall_line, y_value) - evaluate_x_at_y(left_wall_line, y_value))
    positive_widths = [width for width in wall_widths if width > 1e-3]
    if positive_widths:
        width_ratio = max(positive_widths) / max(1e-6, min(positive_widths))
        if width_ratio > 1.35:
            warnings.append("Tube wall spacing changes a lot along height; recheck wall lines or perspective.")

    wall_coverage_gap = abs(left_wall_coverage - right_wall_coverage)
    if wall_coverage_gap > 0.18:
        warnings.append("Left/right wall lines have noticeably different vertical coverage; recheck endpoints.")

    if still_y <= y + 5 or still_y >= y + h - 5:
        warnings.append("Still-level line is too close to the ROI top/bottom boundary.")

    return warnings


def write_yaml(
    path: Path,
    image_path: Path,
    mode: str,
    roi,
    left_wall_line,
    right_wall_line,
    still_level_line,
    tube_axis_line,
    calib_a,
    calib_b,
    mm_per_pixel,
    calibration_distance_mm,
    geometry_warnings,
):
    x, y, w, h = roi
    left_x = int(round(avg_x(list(left_wall_line)) - x))
    right_x = int(round(avg_x(list(right_wall_line)) - x))
    still_y = int(round(avg_y(list(still_level_line)) - y))

    data = {
        "source": {
            "image": str(image_path),
            "mode": mode,
            "annotated_at": datetime.now().isoformat(timespec="seconds"),
        },
        "roi": {
            "x": int(x),
            "y": int(y),
            "w": int(w),
            "h": int(h),
        },
        "tube_inner": {
            "x_left": int(left_x),
            "x_right": int(right_x),
        },
        "calibration": {
            "still_level_px": float(still_y),
            "mm_per_pixel": None if mm_per_pixel is None else float(mm_per_pixel),
        },
        "geometry_roi": {
            "left_wall_line": rel_line(left_wall_line, roi),
            "right_wall_line": rel_line(right_wall_line, roi),
            "still_level_line": rel_line(still_level_line, roi),
            "tube_axis_line": rel_line(tube_axis_line, roi),
        },
        "reference_points_image": {
            "left_wall_line": [[left_wall_line[0].x, left_wall_line[0].y], [left_wall_line[1].x, left_wall_line[1].y]],
            "right_wall_line": [[right_wall_line[0].x, right_wall_line[0].y], [right_wall_line[1].x, right_wall_line[1].y]],
            "still_level_line": [[still_level_line[0].x, still_level_line[0].y], [still_level_line[1].x, still_level_line[1].y]],
            "tube_axis_line": [[tube_axis_line[0].x, tube_axis_line[0].y], [tube_axis_line[1].x, tube_axis_line[1].y]],
            "ruler_point_a": None if calib_a is None else [int(calib_a.x), int(calib_a.y)],
            "ruler_point_b": None if calib_b is None else [int(calib_b.x), int(calib_b.y)],
            "ruler_distance_mm": None if calibration_distance_mm is None else float(calibration_distance_mm),
        },
        "geometry_checks": {
            "warning_count": int(len(geometry_warnings)),
            "warnings": list(geometry_warnings),
        },
    }
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=False)


def main():
    args = parse_args()
    if args.mode == "ruler" and args.calibration_distance_mm is None:
        print("[ERROR] --calibration-distance-mm is required in ruler mode.", file=sys.stderr)
        return 1

    image_path = Path(args.image).expanduser().resolve()
    if not image_path.exists():
        print(f"[ERROR] image not found: {image_path}", file=sys.stderr)
        return 2

    image = cv2.imread(str(image_path))
    if image is None:
        print(f"[ERROR] failed to read image: {image_path}", file=sys.stderr)
        return 3

    roi = cv2.selectROI("Select ROI", image, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Select ROI")
    if roi[2] <= 0 or roi[3] <= 0:
        print("[ERROR] ROI not selected.", file=sys.stderr)
        return 4

    state = {
        "left_wall_top": None,
        "left_wall_bottom": None,
        "right_wall_top": None,
        "right_wall_bottom": None,
        "still_left": None,
        "still_right": None,
        "axis_top": None,
        "axis_bottom": None,
        "calib_a": None,
        "calib_b": None,
    }

    order = [
        "left_wall_top",
        "left_wall_bottom",
        "right_wall_top",
        "right_wall_bottom",
        "still_left",
        "still_right",
        "axis_top",
        "axis_bottom",
    ]
    if args.mode == "ruler":
        order.extend(["calib_a", "calib_b"])

    labels = {
        "left_wall_top": "Step 1: click LEFT inner wall near top",
        "left_wall_bottom": "Step 2: click LEFT inner wall near bottom",
        "right_wall_top": "Step 3: click RIGHT inner wall near top",
        "right_wall_bottom": "Step 4: click RIGHT inner wall near bottom",
        "still_left": "Step 5: click LEFT point on upper visible still-liquid edge",
        "still_right": "Step 6: click RIGHT point on upper visible still-liquid edge",
        "axis_top": "Step 7: click tube-axis point near top center",
        "axis_bottom": "Step 8: click tube-axis point near bottom center",
        "calib_a": "Step 9: click BACKGROUND ruler point A",
        "calib_b": "Step 10: click BACKGROUND ruler point B",
    }
    pending_warnings: List[str] = []
    warning_ack_required = False

    def current_key():
        for key in order:
            if state[key] is None:
                return key
        return None

    def draw_overlay():
        canvas = image.copy()
        x, y, w, h = roi
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 255), 2)

        draw_line(canvas, state, "left_wall_top", "left_wall_bottom", (255, 0, 0), 2)
        draw_line(canvas, state, "right_wall_top", "right_wall_bottom", (0, 128, 255), 2)
        draw_line(canvas, state, "still_left", "still_right", (0, 255, 0), 2)
        draw_line(canvas, state, "axis_top", "axis_bottom", (255, 0, 255), 2)
        draw_line(canvas, state, "calib_a", "calib_b", (255, 255, 0), 2)

        point_colors = {
            "left_wall_top": (255, 0, 0),
            "left_wall_bottom": (255, 0, 0),
            "right_wall_top": (0, 128, 255),
            "right_wall_bottom": (0, 128, 255),
            "still_left": (0, 255, 0),
            "still_right": (0, 255, 0),
            "axis_top": (255, 0, 255),
            "axis_bottom": (255, 0, 255),
            "calib_a": (255, 255, 0),
            "calib_b": (255, 255, 0),
        }
        for key, color in point_colors.items():
            draw_point(canvas, state[key], color)

        cv2.putText(canvas, f"Mode: {args.mode}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        step = current_key()
        prompt = labels[step] if step is not None else "Done: press Enter to save, r to reset, q/Esc to quit"
        cv2.putText(canvas, prompt, (20, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 255, 0), 2)
        cv2.putText(
            canvas,
            "Inside ROI: wall lines, still-level line, and tube axis. Still level: choose upper visible meniscus edge.",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            canvas,
            "Axis line should follow tube center. This line will be used for ROI rotation / rectification.",
            (20, 112),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (255, 255, 255),
            1,
        )
        ruler_hint = (
            "Ruler mode: click background ruler only; Euclidean point distance is used for mm_per_pixel."
            if args.mode == "ruler"
            else "PX-only mode: no ruler clicks required; mm_per_pixel will be left null."
        )
        cv2.putText(
            canvas,
            ruler_hint,
            (20, 134),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (255, 255, 255),
            1,
        )
        if pending_warnings:
            base_y = 162
            cv2.putText(
                canvas,
                "Geometry warnings:",
                (20, base_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 220, 255),
                1,
            )
            for idx, warning in enumerate(pending_warnings[:4]):
                cv2.putText(
                    canvas,
                    f"- {warning}",
                    (20, base_y + 20 * (idx + 1)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    (0, 220, 255),
                    1,
                )
            if warning_ack_required:
                cv2.putText(
                    canvas,
                    "Press Enter again to save anyway, or r to reset.",
                    (20, base_y + 20 * (min(4, len(pending_warnings)) + 2)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.46,
                    (0, 220, 255),
                    1,
                )
        return canvas

    def on_mouse(event, x, y, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        key = current_key()
        if key is None:
            return

        point = Point(x, y)
        if key not in ("calib_a", "calib_b") and not point_in_roi(point, roi):
            print(f"[WARN] {key} must be clicked inside ROI.", file=sys.stderr)
            return

        state[key] = point

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    while True:
        cv2.imshow(WINDOW_NAME, draw_overlay())
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q")):
            cv2.destroyAllWindows()
            print("[INFO] annotation cancelled.")
            return 5
        if key == ord("r"):
            for name in order:
                state[name] = None
            pending_warnings = []
            warning_ack_required = False
            continue
        if key in (10, 13):
            if current_key() is not None:
                print("[WARN] annotation not complete yet.", file=sys.stderr)
                continue
            if warning_ack_required:
                break
            left_wall_line_preview = sort_line_by_y(state["left_wall_top"], state["left_wall_bottom"])
            right_wall_line_preview = sort_line_by_y(state["right_wall_top"], state["right_wall_bottom"])
            still_level_line_preview = sort_line_by_x(state["still_left"], state["still_right"])
            tube_axis_line_preview = sort_line_by_y(state["axis_top"], state["axis_bottom"])
            if avg_x(list(left_wall_line_preview)) > avg_x(list(right_wall_line_preview)):
                left_wall_line_preview, right_wall_line_preview = right_wall_line_preview, left_wall_line_preview
            pending_warnings = geometry_checks(
                roi,
                left_wall_line_preview,
                right_wall_line_preview,
                still_level_line_preview,
                tube_axis_line_preview,
            )
            if pending_warnings:
                print("[WARN] geometry self-check found potential issues:", file=sys.stderr)
                for warning in pending_warnings:
                    print(f"  - {warning}", file=sys.stderr)
                warning_ack_required = True
                continue
            pending_warnings = []
            break

    cv2.destroyAllWindows()

    left_wall_line = sort_line_by_y(state["left_wall_top"], state["left_wall_bottom"])
    right_wall_line = sort_line_by_y(state["right_wall_top"], state["right_wall_bottom"])
    still_level_line = sort_line_by_x(state["still_left"], state["still_right"])
    tube_axis_line = sort_line_by_y(state["axis_top"], state["axis_bottom"])
    calib_a = state["calib_a"]
    calib_b = state["calib_b"]

    if avg_x(list(left_wall_line)) > avg_x(list(right_wall_line)):
        left_wall_line, right_wall_line = right_wall_line, left_wall_line

    final_geometry_warnings = geometry_checks(
        roi,
        left_wall_line,
        right_wall_line,
        still_level_line,
        tube_axis_line,
    )

    if args.mode == "ruler":
        pixel_span = math.hypot(calib_b.x - calib_a.x, calib_b.y - calib_a.y)
        if pixel_span < 1.0:
            print("[ERROR] ruler points must have non-zero distance.", file=sys.stderr)
            return 6
        mm_per_pixel = args.calibration_distance_mm / float(pixel_span)
    else:
        mm_per_pixel = None
        calib_a = None
        calib_b = None

    output_yaml = Path(args.output_yaml).expanduser().resolve() if args.output_yaml else default_output_yaml(image_path)
    output_image = Path(args.output_image).expanduser().resolve() if args.output_image else default_output_image(image_path)

    write_yaml(
        output_yaml,
        image_path,
        args.mode,
        roi,
        left_wall_line,
        right_wall_line,
        still_level_line,
        tube_axis_line,
        calib_a,
        calib_b,
        mm_per_pixel,
        args.calibration_distance_mm if args.mode == "ruler" else None,
        final_geometry_warnings,
    )

    annotated = draw_overlay()
    if not cv2.imwrite(str(output_image), annotated):
        print(f"[ERROR] failed to save annotated image: {output_image}", file=sys.stderr)
        return 7

    print(f"[OK] saved calibration yaml: {output_yaml}")
    print(f"[OK] saved annotated image: {output_image}")
    print(f"[OK] tube_inner.x_left = {int(round(avg_x(list(left_wall_line)) - roi[0]))}")
    print(f"[OK] tube_inner.x_right = {int(round(avg_x(list(right_wall_line)) - roi[0]))}")
    print(f"[OK] calibration.still_level_px = {int(round(avg_y(list(still_level_line)) - roi[1]))}")
    if final_geometry_warnings:
        print("[WARN] geometry self-check warnings were saved to calibration yaml:")
        for warning in final_geometry_warnings:
            print(f"  - {warning}")
    else:
        print("[OK] geometry self-check passed without warnings.")
    if mm_per_pixel is None:
        print("[OK] calibration.mm_per_pixel = null (px_only mode)")
    else:
        print(f"[OK] calibration.mm_per_pixel = {mm_per_pixel:.8f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
