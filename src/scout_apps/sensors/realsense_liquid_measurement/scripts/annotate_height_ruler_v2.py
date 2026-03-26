#!/usr/bin/env python3
"""Interactively annotate v2 multiscale ruler calibration for liquid-height mapping."""

import argparse
import math
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
import yaml


WINDOW_NAME = "Height Ruler Annotation V2"


@dataclass
class Point:
    x: int
    y: int


@dataclass
class PreparedAnnotation:
    left_wall_line: Tuple[Point, Point]
    right_wall_line: Tuple[Point, Point]
    still_level_line: Tuple[Point, Point]
    tube_axis_line: Tuple[Point, Point]
    ruler_points_image: List[Point]
    ruler_points_rectified_roi: np.ndarray
    warnings: List[str]
    errors: List[str]
    rotation_deg: float


def parse_ruler_heights(raw_value: str) -> List[float]:
    parts = [part.strip() for part in raw_value.split(",") if part.strip()]
    if len(parts) < 2:
        raise argparse.ArgumentTypeError("need at least two comma-separated heights, e.g. 0,5,10")

    try:
        values = [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"failed to parse --ruler-heights-mm: {raw_value}") from exc

    for previous, current in zip(values, values[1:]):
        if current <= previous:
            raise argparse.ArgumentTypeError("--ruler-heights-mm must be strictly increasing")
    return values


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Interactively annotate ROI, line-based geometry, and multiscale ruler points "
            "for the v2 liquid-height mapping workflow."
        )
    )
    parser.add_argument("--image", required=True, help="Path to the exported PNG/JPG image.")
    parser.add_argument(
        "--ruler-heights-mm",
        required=True,
        type=parse_ruler_heights,
        help="Strictly increasing comma-separated height sequence, e.g. 0,5,10,15,20,25",
    )
    parser.add_argument(
        "--output-yaml",
        default="",
        help="Output YAML path. Defaults to <image_stem>_multiscale_raw.yaml next to the image.",
    )
    parser.add_argument(
        "--output-image",
        default="",
        help="Annotated preview image path. Defaults to <image_stem>_multiscale_annotated.png next to the image.",
    )
    parser.add_argument(
        "--physical-ruler-zero-mm",
        type=float,
        default=0.0,
        help="Physical ruler zero recorded as metadata only. Default: 0.0",
    )
    parser.add_argument(
        "--max-ruler-x-spread-px",
        type=float,
        default=8.0,
        help="Warn if rectified ruler-point x spread exceeds this value. Default: 8.0 px",
    )
    return parser.parse_args()


def default_output_yaml(image_path: Path) -> Path:
    return image_path.parent / f"{image_path.stem}_multiscale_raw.yaml"


def default_output_image(image_path: Path) -> Path:
    return image_path.parent / f"{image_path.stem}_multiscale_annotated.png"


def point_in_roi(point: Point, roi) -> bool:
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


def sort_line_by_y(p1: Point, p2: Point) -> Tuple[Point, Point]:
    return (p1, p2) if p1.y <= p2.y else (p2, p1)


def sort_line_by_x(p1: Point, p2: Point) -> Tuple[Point, Point]:
    return (p1, p2) if p1.x <= p2.x else (p2, p1)


def avg_x(points: Sequence[Point]) -> float:
    return sum(point.x for point in points) / float(len(points))


def avg_y(points: Sequence[Point]) -> float:
    return sum(point.y for point in points) / float(len(points))


def rel_point(point: Point, roi) -> List[float]:
    return [float(point.x - roi[0]), float(point.y - roi[1])]


def rel_line(points: Sequence[Point], roi) -> List[List[float]]:
    return [rel_point(points[0], roi), rel_point(points[1], roi)]


def evaluate_x_at_y(points: Sequence[Point], y_value: float) -> float:
    p0, p1 = points
    if p1.y == p0.y:
        return 0.5 * (p0.x + p1.x)
    alpha = (float(y_value) - float(p0.y)) / float(p1.y - p0.y)
    return float(p0.x + alpha * (p1.x - p0.x))


def line_vertical_span(points: Sequence[Point]) -> float:
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
        warnings.append("Left wall line covers too little vertical range inside ROI; extend it higher/lower.")
    if right_wall_coverage < min_wall_coverage_ratio:
        warnings.append("Right wall line covers too little vertical range inside ROI; extend it higher/lower.")
    if axis_coverage < min_axis_coverage_ratio:
        warnings.append("Tube axis line covers too little vertical range; extend it across more of the tube height.")

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


def build_sorted_geometry(state: Dict[str, Point]) -> Tuple[Tuple[Point, Point], Tuple[Point, Point], Tuple[Point, Point], Tuple[Point, Point]]:
    left_wall_line = sort_line_by_y(state["left_wall_top"], state["left_wall_bottom"])
    right_wall_line = sort_line_by_y(state["right_wall_top"], state["right_wall_bottom"])
    still_level_line = sort_line_by_x(state["still_left"], state["still_right"])
    tube_axis_line = sort_line_by_y(state["axis_top"], state["axis_bottom"])

    if avg_x(list(left_wall_line)) > avg_x(list(right_wall_line)):
        left_wall_line, right_wall_line = right_wall_line, left_wall_line

    return left_wall_line, right_wall_line, still_level_line, tube_axis_line


def line_to_roi_array(points: Sequence[Point], roi) -> np.ndarray:
    return np.asarray(rel_line(points, roi), dtype=np.float32)


def points_to_roi_array(points: Sequence[Point], roi) -> np.ndarray:
    return np.asarray([rel_point(point, roi) for point in points], dtype=np.float32)


def identity_affine() -> np.ndarray:
    return np.asarray([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)


def transform_points(points_xy: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    if points_xy.size == 0:
        return points_xy.copy()
    return cv2.transform(points_xy.reshape(1, -1, 2), matrix)[0]


def compute_rectification_matrix(roi, tube_axis_line: Sequence[Point]) -> Tuple[np.ndarray, float]:
    axis_roi = line_to_roi_array(tube_axis_line, roi)
    dx = float(axis_roi[1, 0] - axis_roi[0, 0])
    dy = float(axis_roi[1, 1] - axis_roi[0, 1])
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return identity_affine(), 0.0

    theta_deg = math.degrees(math.atan2(dy, dx))
    rotate_deg = 90.0 - theta_deg
    if abs(rotate_deg) < 1e-4:
        return identity_affine(), 0.0

    roi_w = int(roi[2])
    roi_h = int(roi[3])
    center = ((roi_w - 1) * 0.5, (roi_h - 1) * 0.5)
    matrix = cv2.getRotationMatrix2D(center, rotate_deg, 1.0)
    return np.asarray(matrix, dtype=np.float32), float(rotate_deg)


def prepare_annotation(
    state: Dict[str, Point],
    roi,
    ruler_points_image: Sequence[Point],
    ruler_heights_mm: Sequence[float],
    max_ruler_x_spread_px: float,
) -> PreparedAnnotation:
    left_wall_line, right_wall_line, still_level_line, tube_axis_line = build_sorted_geometry(state)
    warnings = geometry_checks(roi, left_wall_line, right_wall_line, still_level_line, tube_axis_line)
    errors: List[str] = []

    if len(ruler_points_image) != len(ruler_heights_mm):
        errors.append("Ruler point count does not match --ruler-heights-mm count.")
        return PreparedAnnotation(
            left_wall_line=left_wall_line,
            right_wall_line=right_wall_line,
            still_level_line=still_level_line,
            tube_axis_line=tube_axis_line,
            ruler_points_image=list(ruler_points_image),
            ruler_points_rectified_roi=np.empty((0, 2), dtype=np.float32),
            warnings=warnings,
            errors=errors,
            rotation_deg=0.0,
        )

    matrix, rotation_deg = compute_rectification_matrix(roi, tube_axis_line)
    ruler_points_roi = points_to_roi_array(ruler_points_image, roi)
    ruler_points_rectified = transform_points(ruler_points_roi, matrix)

    if ruler_points_rectified.shape[0] >= 2:
        x_spread = float(np.max(ruler_points_rectified[:, 0]) - np.min(ruler_points_rectified[:, 0]))
        if x_spread > float(max_ruler_x_spread_px):
            warnings.append(
                "Rectified ruler-point x spread is {:.2f} px (> {:.2f} px); recheck whether all clicks follow "
                "the same ruler line.".format(x_spread, float(max_ruler_x_spread_px))
            )

    for previous_height, current_height, previous_point, current_point in zip(
        ruler_heights_mm,
        ruler_heights_mm[1:],
        ruler_points_rectified,
        ruler_points_rectified[1:],
    ):
        if current_height <= previous_height:
            errors.append("Ruler heights must stay strictly increasing.")
            break
        if current_point[1] >= previous_point[1]:
            errors.append(
                "Rectified ruler-point y must decrease as h_mm increases. "
                "Recheck click order or clicked line centers."
            )
            break

    return PreparedAnnotation(
        left_wall_line=left_wall_line,
        right_wall_line=right_wall_line,
        still_level_line=still_level_line,
        tube_axis_line=tube_axis_line,
        ruler_points_image=list(ruler_points_image),
        ruler_points_rectified_roi=ruler_points_rectified,
        warnings=warnings,
        errors=errors,
        rotation_deg=rotation_deg,
    )


def write_yaml(
    path: Path,
    image_path: Path,
    roi,
    prepared: PreparedAnnotation,
    ruler_heights_mm: Sequence[float],
    physical_ruler_zero_mm: float,
):
    x, y, _w, _h = roi
    left_x = int(round(avg_x(list(prepared.left_wall_line)) - x))
    right_x = int(round(avg_x(list(prepared.right_wall_line)) - x))

    height_mapping_reference_points = []
    for point_xy, height_mm in zip(prepared.ruler_points_rectified_roi, ruler_heights_mm):
        height_mapping_reference_points.append(
            {
                "x_px": float(point_xy[0]),
                "y_px": float(point_xy[1]),
                "h_mm": float(height_mm),
            }
        )

    image_reference_points = []
    for point, height_mm in zip(prepared.ruler_points_image, ruler_heights_mm):
        image_reference_points.append(
            {
                "x_px": float(point.x),
                "y_px": float(point.y),
                "h_mm": float(height_mm),
            }
        )

    data = {
        "source": {
            "image": str(image_path),
            "mode": "multiscale_ruler",
            "annotated_at": datetime.now().isoformat(timespec="seconds"),
        },
        "roi": {
            "x": int(x),
            "y": int(y),
            "w": int(roi[2]),
            "h": int(roi[3]),
        },
        "tube_inner": {
            "x_left": int(left_x),
            "x_right": int(right_x),
        },
        "geometry_roi": {
            "left_wall_line": rel_line(prepared.left_wall_line, roi),
            "right_wall_line": rel_line(prepared.right_wall_line, roi),
            "still_level_line": rel_line(prepared.still_level_line, roi),
            "tube_axis_line": rel_line(prepared.tube_axis_line, roi),
        },
        "height_mapping": {
            "mode": "piecewise_linear",
            "height_zero_definition": "still_level_reference",
            "mapping_coordinate_frame": "rectified_roi",
            "x_usage": "record_only",
            "ruler_click_semantics": "line_center",
            "physical_ruler_zero_mm": float(physical_ruler_zero_mm),
            "reference_points": height_mapping_reference_points,
        },
        "reference_points_image": {
            "ruler_points": image_reference_points,
        },
        "geometry_checks": {
            "warning_count": int(len(prepared.warnings)),
            "warnings": list(prepared.warnings),
        },
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=False)


def format_height(height_mm: float) -> str:
    if abs(height_mm - round(height_mm)) < 1e-6:
        return f"{int(round(height_mm))} mm"
    return f"{height_mm:g} mm"


def main():
    args = parse_args()

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
    }
    ruler_points_image: List[Point] = []

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
    labels = {
        "left_wall_top": "Step 1: click LEFT inner wall near top",
        "left_wall_bottom": "Step 2: click LEFT inner wall near bottom",
        "right_wall_top": "Step 3: click RIGHT inner wall near top",
        "right_wall_bottom": "Step 4: click RIGHT inner wall near bottom",
        "still_left": "Step 5: click LEFT point on upper visible still-liquid edge",
        "still_right": "Step 6: click RIGHT point on upper visible still-liquid edge",
        "axis_top": "Step 7: click tube-axis point near top center",
        "axis_bottom": "Step 8: click tube-axis point near bottom center",
    }

    pending_warnings: List[str] = []
    pending_errors: List[str] = []
    warning_ack_required = False

    def current_step():
        for key in order:
            if state[key] is None:
                return ("geometry", key)
        if len(ruler_points_image) < len(args.ruler_heights_mm):
            return ("ruler", len(ruler_points_image))
        return None

    def reset_pending_messages():
        nonlocal pending_warnings, pending_errors, warning_ack_required
        pending_warnings = []
        pending_errors = []
        warning_ack_required = False

    def draw_overlay():
        canvas = image.copy()
        x, y, w, h = roi
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 255), 2)

        draw_line(canvas, state, "left_wall_top", "left_wall_bottom", (255, 0, 0), 2)
        draw_line(canvas, state, "right_wall_top", "right_wall_bottom", (0, 128, 255), 2)
        draw_line(canvas, state, "still_left", "still_right", (0, 255, 0), 2)
        draw_line(canvas, state, "axis_top", "axis_bottom", (255, 0, 255), 2)

        point_colors = {
            "left_wall_top": (255, 0, 0),
            "left_wall_bottom": (255, 0, 0),
            "right_wall_top": (0, 128, 255),
            "right_wall_bottom": (0, 128, 255),
            "still_left": (0, 255, 0),
            "still_right": (0, 255, 0),
            "axis_top": (255, 0, 255),
            "axis_bottom": (255, 0, 255),
        }
        for key, color in point_colors.items():
            draw_point(canvas, state[key], color)

        for index, point in enumerate(ruler_points_image):
            draw_point(canvas, point, (255, 255, 0), radius=5)
            label = format_height(args.ruler_heights_mm[index])
            cv2.putText(
                canvas,
                label,
                (point.x + 8, point.y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 0),
                1,
            )

        cv2.putText(canvas, "Mode: multiscale_ruler v2", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        step = current_step()
        if step is None:
            prompt = "Done: press Enter to save, u to undo last click, r to reset, q/Esc to quit"
        elif step[0] == "geometry":
            prompt = labels[step[1]]
        else:
            height_mm = args.ruler_heights_mm[step[1]]
            prompt = f"Step {9 + step[1]}: click ruler point for {format_height(height_mm)} (line center)"
        cv2.putText(canvas, prompt, (20, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 255, 0), 2)

        info_lines = [
            "Geometry clicks must stay inside ROI. Ruler points may be outside ROI.",
            "Ruler heights come from --ruler-heights-mm; click points strictly in that order.",
            "v1 mapping uses rectified ROI y only. x is recorded for consistency checks only.",
            "Keys: Enter=validate/save, u=undo last click, r=reset all, q/Esc=quit",
        ]
        for index, info_line in enumerate(info_lines):
            cv2.putText(
                canvas,
                info_line,
                (20, 90 + 22 * index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (255, 255, 255),
                1,
            )

        message_y = 186
        if pending_errors:
            cv2.putText(canvas, "Blocking errors:", (20, message_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            for index, message in enumerate(pending_errors[:4]):
                cv2.putText(
                    canvas,
                    f"- {message}",
                    (20, message_y + 20 * (index + 1)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    (0, 0, 255),
                    1,
                )
            cv2.putText(
                canvas,
                "Use u to undo the last click or r to restart.",
                (20, message_y + 20 * (min(4, len(pending_errors)) + 2)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                (0, 0, 255),
                1,
            )
        elif pending_warnings:
            cv2.putText(canvas, "Warnings:", (20, message_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
            for index, message in enumerate(pending_warnings[:4]):
                cv2.putText(
                    canvas,
                    f"- {message}",
                    (20, message_y + 20 * (index + 1)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    (0, 220, 255),
                    1,
                )
            if warning_ack_required:
                cv2.putText(
                    canvas,
                    "Press Enter again to save anyway, or u/r to change clicks.",
                    (20, message_y + 20 * (min(4, len(pending_warnings)) + 2)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.46,
                    (0, 220, 255),
                    1,
                )
        return canvas

    def on_mouse(event, x_value, y_value, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        step = current_step()
        if step is None:
            return

        point = Point(x_value, y_value)
        if step[0] == "geometry":
            if not point_in_roi(point, roi):
                print(f"[WARN] {step[1]} must be clicked inside ROI.", file=sys.stderr)
                return
            state[step[1]] = point
        else:
            ruler_points_image.append(point)
        reset_pending_messages()

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
            ruler_points_image.clear()
            reset_pending_messages()
            continue
        if key == ord("u"):
            if ruler_points_image:
                ruler_points_image.pop()
            else:
                for name in reversed(order):
                    if state[name] is not None:
                        state[name] = None
                        break
            reset_pending_messages()
            continue
        if key in (10, 13):
            if current_step() is not None:
                print("[WARN] annotation not complete yet.", file=sys.stderr)
                continue

            prepared = prepare_annotation(
                state=state,
                roi=roi,
                ruler_points_image=ruler_points_image,
                ruler_heights_mm=args.ruler_heights_mm,
                max_ruler_x_spread_px=args.max_ruler_x_spread_px,
            )

            if prepared.errors:
                pending_errors = list(prepared.errors)
                pending_warnings = list(prepared.warnings)
                warning_ack_required = False
                print("[ERROR] annotation checks failed:", file=sys.stderr)
                for message in pending_errors:
                    print(f"  - {message}", file=sys.stderr)
                continue

            if prepared.warnings and not warning_ack_required:
                pending_warnings = list(prepared.warnings)
                pending_errors = []
                warning_ack_required = True
                print("[WARN] annotation checks found potential issues:", file=sys.stderr)
                for message in pending_warnings:
                    print(f"  - {message}", file=sys.stderr)
                continue

            pending_warnings = list(prepared.warnings)
            pending_errors = []
            break

    cv2.destroyAllWindows()

    prepared = prepare_annotation(
        state=state,
        roi=roi,
        ruler_points_image=ruler_points_image,
        ruler_heights_mm=args.ruler_heights_mm,
        max_ruler_x_spread_px=args.max_ruler_x_spread_px,
    )
    if prepared.errors:
        print("[ERROR] annotation checks failed before saving.", file=sys.stderr)
        for message in prepared.errors:
            print(f"  - {message}", file=sys.stderr)
        return 6

    output_yaml = Path(args.output_yaml).expanduser().resolve() if args.output_yaml else default_output_yaml(image_path)
    output_image = Path(args.output_image).expanduser().resolve() if args.output_image else default_output_image(image_path)

    write_yaml(
        output_yaml,
        image_path,
        roi,
        prepared,
        args.ruler_heights_mm,
        args.physical_ruler_zero_mm,
    )

    output_image.parent.mkdir(parents=True, exist_ok=True)
    annotated = draw_overlay()
    if not cv2.imwrite(str(output_image), annotated):
        print(f"[ERROR] failed to save annotated image: {output_image}", file=sys.stderr)
        return 7

    rectified_xs = prepared.ruler_points_rectified_roi[:, 0]
    rectified_ys = prepared.ruler_points_rectified_roi[:, 1]
    x_spread = 0.0 if rectified_xs.size == 0 else float(np.max(rectified_xs) - np.min(rectified_xs))

    print(f"[OK] saved calibration yaml: {output_yaml}")
    print(f"[OK] saved annotated image: {output_image}")
    print(f"[OK] tube_inner.x_left = {int(round(avg_x(list(prepared.left_wall_line)) - roi[0]))}")
    print(f"[OK] tube_inner.x_right = {int(round(avg_x(list(prepared.right_wall_line)) - roi[0]))}")
    print(f"[OK] rectification rotation_deg = {prepared.rotation_deg:.4f}")
    print(f"[OK] rectified ruler-point x spread = {x_spread:.4f} px")
    for index, (height_mm, point_x, point_y) in enumerate(zip(args.ruler_heights_mm, rectified_xs, rectified_ys)):
        print(
            "[OK] reference_points[{idx}] = {{x_px: {x_val:.4f}, y_px: {y_val:.4f}, h_mm: {height:.4f}}}".format(
                idx=index,
                x_val=float(point_x),
                y_val=float(point_y),
                height=float(height_mm),
            )
        )
    if prepared.warnings:
        print("[WARN] annotation warnings were saved to calibration yaml:")
        for message in prepared.warnings:
            print(f"  - {message}")
    else:
        print("[OK] annotation checks passed without warnings.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
