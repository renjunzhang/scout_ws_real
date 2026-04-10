#!/usr/bin/env python3
"""Frame-by-frame offline debugger for v2 RealSense liquid height vs MPC slosh height."""

import argparse
import bisect
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge, CvBridgeError

from extract_liquid_height_from_bag import (
    TemporalState,
    build_temporal_state,
    detect_liquid_profile,
    image_topics_in_bag,
    load_processing_config,
    load_yaml,
    rectify_roi_and_calibration,
)
from extract_liquid_height_v2_from_bag import (
    compute_rectification_matrix,
    evaluate_height_mapping,
    evaluate_v2_gates,
    load_v2_calibration,
    mapping_mm_value,
    peak_x_from_detection,
    positive_only,
    v2_default_config,
    zero_based_rel_px,
)


WINDOW_NAME = "Liquid Debug Review v2"
SMALL_STEP_FRAMES = 5


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Build a cached frame-by-frame v2 debug session that emphasizes the "
            "RealSense visual peak, MPC /slosh/height, and manual human height labels."
        )
    )
    parser.add_argument("--bag", required=True, help="Path to the rosbag.")
    parser.add_argument("--calibration", required=True, help="v2 multiscale calibration YAML used for extraction.")
    parser.add_argument(
        "--config",
        default="",
        help="Optional v2 processing config YAML. Defaults to the package liquid_measurement_v2.yaml.",
    )
    parser.add_argument(
        "--image-topic",
        default="",
        help="RGB image topic. Defaults to config topic_image or /camera/color/image_raw.",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="Cache output directory. Defaults to <bag_stem>_frame_debug_v2 next to the bag.",
    )
    parser.add_argument("--encoding", default="bgr8", help="cv_bridge desired encoding.")
    parser.add_argument("--time-offset", type=float, default=0.0, help="Skip frames before bag_start + offset.")
    parser.add_argument("--every", type=int, default=1, help="Process every N-th frame.")
    parser.add_argument("--max-frames", type=int, default=0, help="Maximum number of frames. 0 means all.")
    parser.add_argument(
        "--reuse-cache",
        action="store_true",
        help="Reuse an existing session manifest in out-dir instead of rebuilding it.",
    )
    parser.add_argument(
        "--skip-viewer",
        action="store_true",
        help="Only build cache/CSV and do not open the interactive viewer.",
    )
    parser.add_argument("--start-index", type=int, default=0, help="Viewer start index in cached records.")
    parser.add_argument(
        "--candidate-csv",
        default="",
        help="Optional candidate-frame CSV. Press J/L in viewer to jump between candidate frames.",
    )
    parser.add_argument(
        "--show-frame-index",
        type=int,
        default=-1,
        help="Print one cached frame's metrics to terminal after build/reuse. Negative disables it.",
    )
    parser.add_argument("--photo-width", type=int, default=960, help="Displayed width of the full photo preview.")
    parser.add_argument("--roi-width", type=int, default=320, help="Displayed width of the ROI debug preview.")
    parser.add_argument(
        "--window-max-width",
        type=int,
        default=1440,
        help="Maximum viewer window width after auto-fit scaling.",
    )
    parser.add_argument(
        "--window-max-height",
        type=int,
        default=920,
        help="Maximum viewer window height after auto-fit scaling.",
    )
    parser.add_argument(
        "--history-window",
        type=int,
        default=180,
        help="Number of frames shown around the current cursor in the mini history plot.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=90,
        help="JPEG quality used for cached photo/debug frames.",
    )
    parser.add_argument(
        "--viewer-zero-offset-px",
        type=float,
        default=None,
        help="Initial manual offset for the displayed zero line in rectified-ROI pixels. Negative moves it up.",
    )
    return parser.parse_args()


def session_default_dir(bag_path: Path) -> Path:
    return bag_path.parent / f"{bag_path.stem}_frame_debug_v2"


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def resize_to_width(image: np.ndarray, target_width: int) -> np.ndarray:
    target_width = max(8, int(target_width))
    if image.shape[1] == target_width:
        return image
    scale = target_width / float(image.shape[1])
    target_height = max(8, int(round(image.shape[0] * scale)))
    return cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)


def resize_to_fit(image: np.ndarray, max_width: int, max_height: int) -> np.ndarray:
    max_width = max(8, int(max_width))
    max_height = max(8, int(max_height))
    image_h, image_w = image.shape[:2]
    scale = min(max_width / float(image_w), max_height / float(image_h), 1.0)
    if scale >= 0.999:
        return image
    resized_w = max(8, int(round(image_w * scale)))
    resized_h = max(8, int(round(image_h * scale)))
    return cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_AREA)


def interpolate_value(ts: float, series_times: List[float], series_values: List[float]) -> Optional[float]:
    if not series_times:
        return None
    if len(series_times) == 1:
        return float(series_values[0])
    idx = bisect.bisect_left(series_times, ts)
    if idx <= 0:
        return float(series_values[0])
    if idx >= len(series_times):
        return float(series_values[-1])
    t0 = float(series_times[idx - 1])
    t1 = float(series_times[idx])
    v0 = float(series_values[idx - 1])
    v1 = float(series_values[idx])
    if t1 <= t0:
        return v1
    alpha = (ts - t0) / (t1 - t0)
    return float(v0 + alpha * (v1 - v0))


def finite_float(value) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def format_value(value, digits=3) -> str:
    parsed = finite_float(value)
    if parsed is None:
        return "n/a"
    return f"{parsed:.{digits}f}"


def normalized_ascii_key(raw_key: int) -> int:
    if raw_key is None:
        return -1
    try:
        key = int(raw_key)
    except (TypeError, ValueError):
        return -1
    if key < 0:
        return -1
    return key & 0xFF


def key_matches_char(raw_key: int, *chars: str) -> bool:
    ascii_key = normalized_ascii_key(raw_key)
    if ascii_key < 0:
        return False
    for char in chars:
        if len(char) == 1 and ascii_key == ord(char):
            return True
    return False


def draw_text_block(canvas: np.ndarray, lines: List[str], origin: Tuple[int, int], color=(245, 245, 245)):
    x0, y0 = origin
    for idx, line in enumerate(lines):
        cv2.putText(
            canvas,
            line,
            (x0, y0 + idx * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            1,
            cv2.LINE_AA,
        )


def save_jpeg(path: Path, image: np.ndarray, jpeg_quality: int):
    cv2.imwrite(str(path), image, [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(jpeg_quality, 60, 100))])


def load_auxiliary_series(bag_path: Path) -> Dict[str, List[float]]:
    topics = {
        "/slosh/height": [],
        "/slosh/height_pred_max": [],
        "/cmd_vel_linear_x": [],
        "/cmd_vel_angular_z": [],
        "/imu_ax": [],
        "/imu_ay": [],
    }
    times = {key: [] for key in topics}

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=["/slosh/height", "/slosh/height_pred_max", "/cmd_vel", "/imu/data"]):
            ts = stamp.to_sec()
            if topic == "/slosh/height":
                times["/slosh/height"].append(ts)
                topics["/slosh/height"].append(float(msg.data) * 1000.0)
            elif topic == "/slosh/height_pred_max":
                times["/slosh/height_pred_max"].append(ts)
                topics["/slosh/height_pred_max"].append(float(msg.data) * 1000.0)
            elif topic == "/cmd_vel":
                times["/cmd_vel_linear_x"].append(ts)
                topics["/cmd_vel_linear_x"].append(float(msg.linear.x))
                times["/cmd_vel_angular_z"].append(ts)
                topics["/cmd_vel_angular_z"].append(float(msg.angular.z))
            elif topic == "/imu/data":
                times["/imu_ax"].append(ts)
                topics["/imu_ax"].append(float(msg.linear_acceleration.x))
                times["/imu_ay"].append(ts)
                topics["/imu_ay"].append(float(msg.linear_acceleration.y))

    return {
        "slosh_height_t": times["/slosh/height"],
        "slosh_height_mm": topics["/slosh/height"],
        "slosh_pred_t": times["/slosh/height_pred_max"],
        "slosh_pred_mm": topics["/slosh/height_pred_max"],
        "cmd_vel_t": times["/cmd_vel_linear_x"],
        "cmd_vel_linear_x": topics["/cmd_vel_linear_x"],
        "cmd_vel_angular_z": topics["/cmd_vel_angular_z"],
        "imu_ax": topics["/imu_ax"],
        "imu_t": times["/imu_ay"],
        "imu_ay": topics["/imu_ay"],
    }


def inverse_transform_point(matrix: np.ndarray, x_px: Optional[float], y_px: Optional[float]) -> Optional[Tuple[float, float]]:
    x_value = finite_float(x_px)
    y_value = finite_float(y_px)
    if x_value is None or y_value is None:
        return None
    inverse_matrix = cv2.invertAffineTransform(matrix)
    point = np.asarray([[[float(x_value), float(y_value)]]], dtype=np.float32)
    transformed = cv2.transform(point, inverse_matrix)[0][0]
    return float(transformed[0]), float(transformed[1])


def draw_rectified_horizontal_reference_on_rectified_roi(
    roi_image: np.ndarray,
    y_rect: Optional[float],
    color: Tuple[int, int, int],
    thickness: int = 2,
):
    y_value = finite_float(y_rect)
    if y_value is None:
        return
    y_px = int(round(np.clip(y_value, 0.0, max(0.0, float(roi_image.shape[0] - 1)))))
    cv2.line(roi_image, (0, y_px), (roi_image.shape[1] - 1, y_px), color, thickness, cv2.LINE_AA)


def draw_rectified_horizontal_reference_on_full_photo(
    photo: np.ndarray,
    rectification_matrix: np.ndarray,
    roi_x: int,
    roi_y: int,
    roi_w: int,
    y_rect: Optional[float],
    color: Tuple[int, int, int],
    thickness: int = 2,
):
    y_value = finite_float(y_rect)
    if y_value is None:
        return
    left_pt = inverse_transform_point(rectification_matrix, 0.0, y_value)
    right_pt = inverse_transform_point(rectification_matrix, float(max(0, roi_w - 1)), y_value)
    if left_pt is None or right_pt is None:
        return
    p0 = (roi_x + int(round(left_pt[0])), roi_y + int(round(left_pt[1])))
    p1 = (roi_x + int(round(right_pt[0])), roi_y + int(round(right_pt[1])))
    cv2.line(photo, p0, p1, color, thickness, cv2.LINE_AA)


def draw_zero_reference_on_rectified_roi(
    roi_image: np.ndarray,
    zero_y_rect: Optional[float],
):
    draw_rectified_horizontal_reference_on_rectified_roi(roi_image, zero_y_rect, (0, 255, 255), thickness=2)


def draw_zero_reference_on_full_photo(
    photo: np.ndarray,
    rectification_matrix: np.ndarray,
    roi_x: int,
    roi_y: int,
    roi_w: int,
    zero_y_rect: Optional[float],
):
    draw_rectified_horizontal_reference_on_full_photo(
        photo,
        rectification_matrix,
        roi_x,
        roi_y,
        roi_w,
        zero_y_rect,
        (0, 255, 255),
        thickness=2,
    )


def persist_manifest_json(out_dir: Path, manifest: Dict):
    with (out_dir / "debug_session.json").open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)


def load_human_labels(path: Path) -> Dict[int, Dict[str, Optional[float]]]:
    if not path.exists():
        return {}
    labels: Dict[int, Dict[str, Optional[float]]] = {}
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            frame_index = int(row["frame_index"])
            height_raw = str(row.get("human_height_mm", "") or "").strip()
            peak_raw = str(row.get("human_peak_mm", "") or "").strip()
            peak_y_rect_raw = str(row.get("human_peak_y_rect_px", "") or "").strip()
            labels[frame_index] = {
                "human_height_mm": None if height_raw == "" else float(height_raw),
                "human_peak_mm": None if peak_raw == "" else float(peak_raw),
                "human_peak_y_rect_px": None if peak_y_rect_raw == "" else float(peak_y_rect_raw),
            }
    return labels


def write_human_labels_csv(path: Path, records: List[Dict]):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "frame_index",
                "stamp",
                "relative_time_s",
                "human_height_mm",
                "human_peak_mm",
                "human_peak_y_rect_px",
                "center_rel_mm_v2",
                "peak_rel_mm_v2",
                "slosh_height_mm",
                "slosh_pred_mm",
            ]
        )
        for record in records:
            height_value = record.get("human_height_mm")
            peak_value = record.get("human_peak_mm")
            peak_y_rect_value = record.get("human_peak_y_rect_px")
            if height_value is None and peak_value is None and peak_y_rect_value is None:
                continue
            writer.writerow(
                [
                    record["frame_index"],
                    f"{record['stamp']:.9f}",
                    f"{record['relative_time_s']:.9f}",
                    "" if height_value is None else f"{float(height_value):.6f}",
                    "" if peak_value is None else f"{float(peak_value):.6f}",
                    "" if peak_y_rect_value is None else f"{float(peak_y_rect_value):.3f}",
                    "" if record.get("center_rel_mm_v2") is None else f"{float(record['center_rel_mm_v2']):.6f}",
                    "" if record.get("peak_rel_mm_v2") is None else f"{float(record['peak_rel_mm_v2']):.6f}",
                    "" if record.get("slosh_height_mm") is None else f"{float(record['slosh_height_mm']):.6f}",
                    "" if record.get("slosh_pred_mm") is None else f"{float(record['slosh_pred_mm']):.6f}",
                ]
            )


def write_session_csv(path: Path, records: List[Dict]):
    fieldnames = [
        "frame_index",
        "stamp",
        "relative_time_s",
        "photo_path",
        "roi_debug_path",
        "valid_v2",
        "accept_for_peak_report_v2",
        "legacy_valid_gate_v2",
        "legacy_accept_for_peak_report_gate_v2",
        "confidence_v2",
        "peak_source_v2",
        "center_y_rect",
        "peak_y_rect",
        "peak_x_rect",
        "center_rel_mm_v2",
        "peak_rel_mm_v2",
        "center_rel_mm_signed_v2",
        "peak_rel_mm_signed_v2",
        "left_rel_mm_v2",
        "right_rel_mm_v2",
        "slosh_height_mm",
        "slosh_pred_mm",
        "human_height_mm",
        "human_peak_mm",
        "human_peak_y_rect_px",
        "cmd_vel_linear_x",
        "cmd_vel_angular_z",
        "imu_ax",
        "imu_ay",
        "fit_rms_px_v2",
        "peak_local_rms_px_v2",
        "left_band_support_ratio_v2",
        "right_band_support_ratio_v2",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            writer.writerow({key: record.get(key, "") for key in fieldnames})


def persist_session(out_dir: Path, manifest: Dict):
    records = manifest.get("records", [])
    persist_manifest_json(out_dir, manifest)
    write_session_csv(out_dir / "debug_session.csv", records)
    write_human_labels_csv(out_dir / "human_labels.csv", records)


def describe_record(record: Dict) -> str:
    lines = [
        f"frame_index: {record['frame_index']}",
        f"stamp: {record['stamp']:.6f}",
        f"relative_time_s: {record['relative_time_s']:.3f}",
        f"valid_v2: {record['valid_v2']}",
        f"accept_for_peak_report_v2: {record['accept_for_peak_report_v2']}",
        f"meniscus_confidence_v2: {record['confidence_v2']:.3f}",
        f"peak_source_v2: {record.get('peak_source_v2', 'n/a')}",
        f"center_rel_mm_v2: {record.get('center_rel_mm_v2')}",
        f"peak_rel_mm_v2: {record.get('peak_rel_mm_v2')}",
        f"human_height_mm: {record.get('human_height_mm')}",
        f"human_peak_mm: {record.get('human_peak_mm')}",
        f"human_peak_y_rect_px: {record.get('human_peak_y_rect_px')}",
        f"/slosh/height [mm]: {record.get('slosh_height_mm')}",
        f"/slosh/height_pred_max [mm]: {record.get('slosh_pred_mm')}",
        f"fit_rms_px_v2: {record.get('fit_rms_px_v2')}",
        f"peak_local_rms_px_v2: {record.get('peak_local_rms_px_v2')}",
        f"photo_path: {record['photo_path']}",
        f"roi_debug_path: {record['roi_debug_path']}",
    ]
    return "\n".join(lines)


def print_selected_frame(manifest: Dict, frame_index: int):
    records = manifest.get("records", [])
    if not records:
        raise RuntimeError("debug session has no records")
    idx = None
    for pos, record in enumerate(records):
        if int(record.get("frame_index", -1)) == int(frame_index):
            idx = pos
            break
    if idx is None:
        idx = int(np.clip(frame_index, 0, len(records) - 1))
    print("[INFO] selected cached frame")
    print(describe_record(records[idx]))


def draw_history_plot(
    canvas: np.ndarray,
    origin: Tuple[int, int],
    size: Tuple[int, int],
    records: List[Dict],
    cursor_idx: int,
):
    plot_x, plot_y = origin
    plot_w, plot_h = size
    plot = canvas[plot_y : plot_y + plot_h, plot_x : plot_x + plot_w]
    plot[:] = (28, 28, 28)
    cv2.rectangle(canvas, (plot_x, plot_y), (plot_x + plot_w - 1, plot_y + plot_h - 1), (90, 90, 90), 1)

    center_mm = [record.get("center_rel_mm_v2") for record in records]
    peak_mm = [record.get("peak_rel_mm_v2") for record in records]
    human_center_mm = [record.get("human_height_mm") for record in records]
    human_peak_mm = [record.get("human_peak_mm") for record in records]
    mpc_mm = [record.get("slosh_height_mm") for record in records]
    pred_mm = [record.get("slosh_pred_mm") for record in records]

    finite_values = [
        float(value)
        for series in (center_mm, peak_mm, human_center_mm, human_peak_mm, mpc_mm, pred_mm)
        for value in series
        if value is not None and math.isfinite(value)
    ]
    if not finite_values:
        return

    y_min = min(0.0, min(finite_values))
    y_max = max(max(finite_values), y_min + 1e-3)
    y_span = max(1e-3, y_max - y_min)
    x_span = max(1, len(records) - 1)

    def to_points(values, color, thick=1, scatter=False, only_reported=False):
        pts = []
        for idx, value in enumerate(values):
            if value is None or not math.isfinite(value):
                continue
            if only_reported and not records[idx]["accept_for_peak_report_v2"]:
                continue
            px = plot_x + int(round((idx / float(x_span)) * (plot_w - 1)))
            py = plot_y + int(round((1.0 - (float(value) - y_min) / y_span) * (plot_h - 1)))
            pts.append((px, py))
        if scatter:
            for pt in pts:
                cv2.circle(canvas, pt, 2, color, -1)
        elif len(pts) >= 2:
            cv2.polylines(canvas, [np.asarray(pts, dtype=np.int32)], False, color, thick, cv2.LINE_AA)
        elif len(pts) == 1:
            cv2.circle(canvas, pts[0], 2, color, -1)

    to_points(center_mm, (255, 120, 60), thick=2, only_reported=True)
    to_points(center_mm, (160, 160, 160), thick=1, only_reported=False)
    to_points(mpc_mm, (64, 180, 255), thick=2)
    to_points(peak_mm, (196, 128, 255), thick=1)
    to_points(pred_mm, (96, 96, 255), thick=1)
    to_points(human_center_mm, (0, 220, 120), scatter=True)
    to_points(human_peak_mm, (0, 180, 255), scatter=True)

    cursor_x = plot_x + int(round((cursor_idx / float(x_span)) * (plot_w - 1)))
    cv2.line(canvas, (cursor_x, plot_y), (cursor_x, plot_y + plot_h - 1), (0, 255, 255), 1)
    cv2.putText(
        canvas,
        "history: center(main) / slosh / peak(diag) / pred / human-center / human-peak",
        (plot_x + 6, plot_y + 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )


def draw_input_box(
    canvas: np.ndarray,
    origin: Tuple[int, int],
    width: int,
    current_value,
    input_mode: bool,
    input_buffer: str,
    title_base: str,
    active_hint: str,
):
    x0, y0 = origin
    box_h = 56
    cv2.rectangle(canvas, (x0, y0), (x0 + width - 1, y0 + box_h - 1), (80, 80, 80), 1)
    if input_mode:
        cv2.rectangle(canvas, (x0 + 1, y0 + 1), (x0 + width - 2, y0 + box_h - 2), (32, 48, 32), -1)
        title = f"{title_base} [editing]"
        body = input_buffer if input_buffer else "_"
        hint = "type number in mm, Enter save, Backspace edit, Esc cancel"
        title_color = (120, 255, 120)
    else:
        cv2.rectangle(canvas, (x0 + 1, y0 + 1), (x0 + width - 2, y0 + box_h - 2), (24, 24, 24), -1)
        title = title_base
        if current_value is None:
            body = "unlabeled"
        else:
            body = f"{float(current_value):.3f} mm"
        hint = active_hint
        title_color = (220, 220, 220)
    cv2.putText(canvas, title, (x0 + 8, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, title_color, 1, cv2.LINE_AA)
    cv2.putText(canvas, body, (x0 + 8, y0 + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, hint, (x0 + 8, y0 + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA)
    return (x0, y0, width, box_h)


def compose_view(
    photo: np.ndarray,
    roi_debug: np.ndarray,
    record: Dict,
    records: List[Dict],
    cursor_idx: int,
    photo_width: int,
    roi_width: int,
    history_window: int,
    input_target: Optional[str],
    input_buffer: str,
    viewer_zero_offset_px: float,
    pick_peak_y_rect_mode: bool,
) -> Tuple[np.ndarray, Dict[str, Tuple[int, int, int, int]], Tuple[int, int, int, int]]:
    del history_window

    photo_show = resize_to_width(photo, photo_width)
    roi_show = resize_to_width(roi_debug, roi_width)

    center_mm = finite_float(record.get("center_rel_mm_v2"))
    peak_mm = finite_float(record.get("peak_rel_mm_v2"))
    slosh_height = finite_float(record.get("slosh_height_mm"))
    slosh_pred = finite_float(record.get("slosh_pred_mm"))
    human_height = finite_float(record.get("human_height_mm"))
    human_peak = finite_float(record.get("human_peak_mm"))
    human_peak_y_rect = finite_float(record.get("human_peak_y_rect_px"))

    err_center_height = None if center_mm is None or slosh_height is None else center_mm - slosh_height
    err_peak_pred = None if peak_mm is None or slosh_pred is None else peak_mm - slosh_pred
    err_human_center = None if human_height is None or center_mm is None else human_height - center_mm
    err_human_slosh = None if human_height is None or slosh_height is None else human_height - slosh_height
    err_human_peak_diag = None if human_peak is None or peak_mm is None else human_peak - peak_mm
    err_human_peak_slosh = None if human_peak is None or slosh_height is None else human_peak - slosh_height

    lines = [
        f"cache_idx={cursor_idx} frame={record['frame_index']} rel_t={format_value(record['relative_time_s'], 2)}s",
        f"valid_v2={record['valid_v2']} report={record['accept_for_peak_report_v2']} conf={format_value(record['confidence_v2'], 2)}",
        f"main center={format_value(center_mm, 3)} mm",
        f"RS visual peak={format_value(peak_mm, 3)} mm src={record.get('peak_source_v2', 'n/a')}",
        f"/slosh/height={format_value(slosh_height, 3)} mm",
        f"/slosh/height_pred_max={format_value(slosh_pred, 3)} mm",
        f"human center={format_value(human_height, 3)} mm",
        f"human peak={format_value(human_peak, 3)} mm",
        f"human peak y_rect={format_value(human_peak_y_rect, 1)} px",
        f"err center-vs-height={format_value(err_center_height, 3)} mm",
        f"err peak-vs-pred={format_value(err_peak_pred, 3)} mm",
        f"err human-center={format_value(err_human_center, 3)} mm",
        f"err human-center-vs-slosh={format_value(err_human_slosh, 3)} mm",
        f"err human-peak-vs-peak={format_value(err_human_peak_diag, 3)} mm",
        f"err human-peak-vs-slosh={format_value(err_human_peak_slosh, 3)} mm",
        f"cmd_vx={format_value(record.get('cmd_vel_linear_x'), 3)}  cmd_wz={format_value(record.get('cmd_vel_angular_z'), 3)}",
        f"imu_ax={format_value(record.get('imu_ax'), 3)}  imu_ay={format_value(record.get('imu_ay'), 3)}",
        f"fit_rms={format_value(record.get('fit_rms_px_v2'), 3)}  peak_local_rms={format_value(record.get('peak_local_rms_px_v2'), 3)}",
        f"Lband={format_value(record.get('left_band_support_ratio_v2'), 2)}  Rband={format_value(record.get('right_band_support_ratio_v2'), 2)}",
        f"viewer zero offset={float(viewer_zero_offset_px):+.1f} px",
        (
            "peak y pick mode=ON: click inside rectified ROI to save human_peak_y_rect_px"
            if pick_peak_y_rect_mode
            else "peak y pick mode=off"
        ),
        (
            f"keys: a/d +/-{SMALL_STEP_FRAMES}, w/s +/-10, e/z +/-50, g jump, "
            "r/f zero +/-1px, t/v zero +/-5px, b reset zero, i/p mm label, y pick peak-y, u clear peak-y, x/c clear, h help, q quit"
        ),
    ]

    panel_w = max(roi_show.shape[1], 620)
    min_history_h = 140
    panel_required_h = 26 + len(lines) * 22 + 6 + 56 + 10 + min_history_h + 16
    panel_h = max(panel_required_h, photo_show.shape[0] - roi_show.shape[0], 360)
    canvas_h = max(photo_show.shape[0], roi_show.shape[0] + panel_h)
    canvas_w = photo_show.shape[1] + panel_w
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
    canvas[:] = (18, 18, 18)

    canvas[: photo_show.shape[0], : photo_show.shape[1]] = photo_show
    right_x = photo_show.shape[1]
    canvas[: roi_show.shape[0], right_x : right_x + roi_show.shape[1]] = roi_show
    roi_rect = (right_x, 0, roi_show.shape[1], roi_show.shape[0])

    photo_status = (
        f"RS visual peak={format_value(peak_mm, 3)} mm    "
        f"/slosh/height={format_value(slosh_height, 3)} mm"
    )
    status_w = min(photo_show.shape[1] - 16, 760)
    cv2.rectangle(canvas, (8, 8), (8 + status_w, 42), (0, 0, 0), -1)
    cv2.putText(
        canvas,
        photo_status,
        (14, 31),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )

    overlay_w = max(260, min(photo_show.shape[1] - 24, 420))
    overlay_x = 12
    peak_overlay_y = max(12, photo_show.shape[0] - 72)
    center_overlay_y = max(12, peak_overlay_y - 64)
    center_rect = draw_input_box(
        canvas,
        (overlay_x, center_overlay_y),
        overlay_w,
        human_height,
        input_target == "human_height_mm",
        input_buffer,
        "Human Center Input",
        "press i to edit, x to clear center label",
    )
    peak_rect = draw_input_box(
        canvas,
        (overlay_x, peak_overlay_y),
        overlay_w,
        human_peak,
        input_target == "human_peak_mm",
        input_buffer,
        "Human Peak Input",
        "press p to edit, c to clear peak label",
    )

    panel_y = roi_show.shape[0]
    cv2.rectangle(canvas, (right_x, panel_y), (canvas_w - 1, canvas_h - 1), (60, 60, 60), 1)
    draw_text_block(canvas, lines, (right_x + 12, panel_y + 26))

    history_y = panel_y + 26 + len(lines) * 22 + 12
    history_h = max(min_history_h, canvas_h - history_y - 16)
    draw_history_plot(canvas, (right_x + 10, history_y), (panel_w - 20, history_h), records, cursor_idx)
    return canvas, {"human_height_mm": center_rect, "human_peak_mm": peak_rect}, roi_rect


def build_session(args, out_dir: Path) -> Dict:
    bag_path = Path(args.bag).expanduser().resolve()
    calibration_path = Path(args.calibration).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve() if args.config else v2_default_config()

    calibration_v2 = load_v2_calibration(calibration_path)
    config_dict = load_yaml(config_path) if config_path.exists() else {}
    _, processing = load_processing_config(config_path if config_path.exists() else None)
    image_topic = args.image_topic or str(config_dict.get("topic_image", "/camera/color/image_raw"))
    bridge = CvBridge()

    photo_dir = out_dir / "photo"
    roi_dir = out_dir / "roi_debug"
    ensure_dir(out_dir)
    ensure_dir(photo_dir)
    ensure_dir(roi_dir)

    human_labels = load_human_labels(out_dir / "human_labels.csv")
    auxiliary = load_auxiliary_series(bag_path)
    rectification_matrix, _ = compute_rectification_matrix(calibration_v2.legacy)
    manifest_records: List[Dict] = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        image_topics = image_topics_in_bag(bag)
        if image_topic not in image_topics:
            raise RuntimeError(f"image topic not found in bag: {image_topic}; available: {image_topics}")
        bag_start = bag.get_start_time()
        min_stamp = bag_start + max(args.time_offset, 0.0)
        previous_state: Optional[TemporalState] = None
        image_counter = 0
        processed = 0

        for _, msg, stamp in bag.read_messages(topics=[image_topic]):
            stamp_sec = stamp.to_sec()
            if stamp_sec < min_stamp:
                continue
            if image_counter % max(1, args.every) != 0:
                image_counter += 1
                continue
            if args.max_frames > 0 and processed >= args.max_frames:
                break

            try:
                image = bridge.imgmsg_to_cv2(msg, desired_encoding=args.encoding)
            except CvBridgeError as exc:
                print(f"[WARN] skip frame {image_counter}: {exc}", file=sys.stderr)
                image_counter += 1
                continue

            y0 = calibration_v2.legacy.roi_y
            y1 = calibration_v2.legacy.roi_y + calibration_v2.legacy.roi_h
            x0 = calibration_v2.legacy.roi_x
            x1 = calibration_v2.legacy.roi_x + calibration_v2.legacy.roi_w
            if y0 < 0 or x0 < 0 or y1 > image.shape[0] or x1 > image.shape[1]:
                raise RuntimeError(f"calibration ROI exceeds frame bounds at frame {image_counter}")

            roi_bgr = image[y0:y1, x0:x1].copy()
            rectified_roi_bgr, rectified_calibration = rectify_roi_and_calibration(roi_bgr, calibration_v2.legacy)
            detection = detect_liquid_profile(
                rectified_roi_bgr,
                rectified_calibration,
                processing,
                previous_state=previous_state,
            )
            gate_decision = evaluate_v2_gates(detection, processing, rectified_calibration.roi_h)
            if gate_decision.legacy_valid:
                new_state = build_temporal_state(detection)
                if new_state is not None:
                    previous_state = new_state

            left_mapping = evaluate_height_mapping(calibration_v2.reference_points, detection["left_y"])
            right_mapping = evaluate_height_mapping(calibration_v2.reference_points, detection["right_y"])
            center_mapping = evaluate_height_mapping(calibration_v2.reference_points, detection["center_y"])
            peak_mapping = evaluate_height_mapping(calibration_v2.reference_points, detection["peak_y"])

            center_rel_px_signed_v2 = zero_based_rel_px(calibration_v2.zero_y_rect, detection["center_y"])
            peak_rel_px_signed_v2 = zero_based_rel_px(calibration_v2.zero_y_rect, detection["peak_y"])
            left_rel_px_signed_v2 = zero_based_rel_px(calibration_v2.zero_y_rect, detection["left_y"])
            right_rel_px_signed_v2 = zero_based_rel_px(calibration_v2.zero_y_rect, detection["right_y"])

            center_rel_mm_signed_v2 = mapping_mm_value(center_mapping)
            peak_rel_mm_signed_v2 = mapping_mm_value(peak_mapping)
            left_rel_mm_signed_v2 = mapping_mm_value(left_mapping)
            right_rel_mm_signed_v2 = mapping_mm_value(right_mapping)

            center_rel_mm_v2 = positive_only(center_rel_mm_signed_v2)
            peak_rel_mm_v2 = positive_only(peak_rel_mm_signed_v2)
            left_rel_mm_v2 = positive_only(left_rel_mm_signed_v2)
            right_rel_mm_v2 = positive_only(right_rel_mm_signed_v2)

            peak_x_rect = peak_x_from_detection(detection)
            peak_y_rect = None if detection["peak_y"] is None else float(detection["peak_y"])
            center_x_rect = None if detection["center_eval_x"] is None else float(detection["center_eval_x"])
            center_y_rect = None if detection["center_y"] is None else float(detection["center_y"])
            peak_point_roi = inverse_transform_point(rectification_matrix, peak_x_rect, peak_y_rect)

            debug_roi = rectified_roi_bgr.copy()

            record = {
                "frame_index": int(image_counter),
                "stamp": float(stamp_sec),
                "relative_time_s": float(stamp_sec - bag_start),
                "valid_v2": int(gate_decision.valid),
                "accept_for_peak_report_v2": int(gate_decision.accept_for_peak_report),
                "legacy_valid_gate_v2": int(gate_decision.legacy_valid),
                "legacy_accept_for_peak_report_gate_v2": int(gate_decision.legacy_accept_for_peak_report),
                "confidence_v2": float(detection["confidence"]),
                "peak_source_v2": detection["peak_source"],
                "center_x_rect": center_x_rect,
                "center_y_rect": center_y_rect,
                "peak_x_rect": peak_x_rect,
                "peak_y_rect": peak_y_rect,
                "peak_x_roi": None if peak_point_roi is None else float(peak_point_roi[0]),
                "peak_y_roi": None if peak_point_roi is None else float(peak_point_roi[1]),
                "center_rel_px_signed_v2": None if center_rel_px_signed_v2 is None else float(center_rel_px_signed_v2),
                "peak_rel_px_signed_v2": None if peak_rel_px_signed_v2 is None else float(peak_rel_px_signed_v2),
                "left_rel_px_signed_v2": None if left_rel_px_signed_v2 is None else float(left_rel_px_signed_v2),
                "right_rel_px_signed_v2": None if right_rel_px_signed_v2 is None else float(right_rel_px_signed_v2),
                "center_rel_mm_signed_v2": None if center_rel_mm_signed_v2 is None else float(center_rel_mm_signed_v2),
                "peak_rel_mm_signed_v2": None if peak_rel_mm_signed_v2 is None else float(peak_rel_mm_signed_v2),
                "left_rel_mm_signed_v2": None if left_rel_mm_signed_v2 is None else float(left_rel_mm_signed_v2),
                "right_rel_mm_signed_v2": None if right_rel_mm_signed_v2 is None else float(right_rel_mm_signed_v2),
                "center_rel_mm_v2": None if center_rel_mm_v2 is None else float(center_rel_mm_v2),
                "peak_rel_mm_v2": None if peak_rel_mm_v2 is None else float(peak_rel_mm_v2),
                "left_rel_mm_v2": None if left_rel_mm_v2 is None else float(left_rel_mm_v2),
                "right_rel_mm_v2": None if right_rel_mm_v2 is None else float(right_rel_mm_v2),
                "fit_rms_px_v2": None if detection["fit_rms_px"] is None else float(detection["fit_rms_px"]),
                "peak_local_rms_px_v2": None if detection["peak_local_rms_px"] is None else float(detection["peak_local_rms_px"]),
                "left_band_support_ratio_v2": float(detection["left_band_support_ratio"]),
                "right_band_support_ratio_v2": float(detection["right_band_support_ratio"]),
                "slosh_height_mm": interpolate_value(stamp_sec, auxiliary["slosh_height_t"], auxiliary["slosh_height_mm"]),
                "slosh_pred_mm": interpolate_value(stamp_sec, auxiliary["slosh_pred_t"], auxiliary["slosh_pred_mm"]),
                "cmd_vel_linear_x": interpolate_value(stamp_sec, auxiliary["cmd_vel_t"], auxiliary["cmd_vel_linear_x"]),
                "cmd_vel_angular_z": interpolate_value(stamp_sec, auxiliary["cmd_vel_t"], auxiliary["cmd_vel_angular_z"]),
                "imu_ax": interpolate_value(stamp_sec, auxiliary["imu_t"], auxiliary["imu_ax"]),
                "imu_ay": interpolate_value(stamp_sec, auxiliary["imu_t"], auxiliary["imu_ay"]),
                "human_height_mm": human_labels.get(int(image_counter), {}).get("human_height_mm"),
                "human_peak_mm": human_labels.get(int(image_counter), {}).get("human_peak_mm"),
                "human_peak_y_rect_px": human_labels.get(int(image_counter), {}).get("human_peak_y_rect_px"),
            }

            full_photo = image.copy()

            photo_path = photo_dir / f"frame_{image_counter:06d}.jpg"
            roi_path = roi_dir / f"frame_{image_counter:06d}.jpg"
            save_jpeg(photo_path, full_photo, args.jpeg_quality)
            save_jpeg(roi_path, debug_roi, args.jpeg_quality)

            record["photo_path"] = str(photo_path)
            record["roi_debug_path"] = str(roi_path)
            manifest_records.append(record)
            image_counter += 1
            processed += 1

    manifest = {
        "bag": str(bag_path),
        "calibration": str(calibration_path),
        "config": str(config_path),
        "image_topic": image_topic,
        "out_dir": str(out_dir),
        "viewer_zero_offset_px": 0.0 if args.viewer_zero_offset_px is None else float(args.viewer_zero_offset_px),
        "records": manifest_records,
    }
    persist_session(out_dir, manifest)
    return manifest


def load_session(out_dir: Path) -> Dict:
    manifest_path = out_dir / "debug_session.json"
    if not manifest_path.exists():
        raise FileNotFoundError(str(manifest_path))
    with manifest_path.open("r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    label_overrides = load_human_labels(out_dir / "human_labels.csv")
    for record in manifest.get("records", []):
        override = label_overrides.get(int(record.get("frame_index", -1)), {})
        record["human_height_mm"] = override.get("human_height_mm", record.get("human_height_mm"))
        record["human_peak_mm"] = override.get("human_peak_mm", record.get("human_peak_mm"))
        record["human_peak_y_rect_px"] = override.get("human_peak_y_rect_px", record.get("human_peak_y_rect_px"))
    return manifest


def save_manual_label(out_dir: Path, manifest: Dict, idx: int, field: str, value: Optional[float]):
    manifest["records"][idx][field] = None if value is None else float(value)
    persist_session(out_dir, manifest)


def save_viewer_zero_offset(out_dir: Path, manifest: Dict, value: float):
    manifest["viewer_zero_offset_px"] = float(value)
    persist_manifest_json(out_dir, manifest)


def load_viewer_zero_context(manifest: Dict) -> Optional[Dict]:
    calibration_raw = str(manifest.get("calibration", "")).strip()
    if not calibration_raw:
        return None
    calibration_path = Path(calibration_raw).expanduser().resolve()
    if not calibration_path.exists():
        return None
    calibration_v2 = load_v2_calibration(calibration_path)
    rectification_matrix, _ = compute_rectification_matrix(calibration_v2.legacy)
    return {
        "base_zero_y_rect": float(calibration_v2.zero_y_rect),
        "rectification_matrix": rectification_matrix,
        "roi_x": int(calibration_v2.legacy.roi_x),
        "roi_y": int(calibration_v2.legacy.roi_y),
        "roi_w": int(calibration_v2.legacy.roi_w),
    }


def load_candidate_cache_indices(manifest: Dict, out_dir: Path, candidate_csv_path: str) -> List[int]:
    csv_raw = str(candidate_csv_path).strip()
    if csv_raw == "":
        return []
    csv_path = Path(csv_raw).expanduser().resolve()
    if not csv_path.exists():
        raise RuntimeError(f"candidate csv not found: {csv_path}")

    records = manifest.get("records", [])
    if not records:
        return []

    session_name = out_dir.name
    bag_raw = str(manifest.get("bag", "")).strip()
    bag_id = Path(bag_raw).stem if bag_raw else ""
    debug_dir = str(out_dir.resolve())

    frame_to_cache_idx: Dict[int, int] = {}
    for cache_idx, record in enumerate(records):
        frame_index = finite_float(record.get("frame_index"))
        if frame_index is None:
            continue
        frame_to_cache_idx[int(frame_index)] = int(cache_idx)

    candidate_indices: List[int] = []
    seen = set()
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            row_session = str(row.get("session_name", "")).strip()
            row_bag_id = str(row.get("bag_id", "")).strip()
            row_debug_dir = str(row.get("debug_dir", "")).strip()
            if row_session != session_name and row_bag_id != bag_id and row_debug_dir != debug_dir:
                continue
            frame_index = finite_float(row.get("frame_index"))
            if frame_index is None:
                continue
            cache_idx = frame_to_cache_idx.get(int(frame_index))
            if cache_idx is None or cache_idx in seen:
                continue
            seen.add(cache_idx)
            candidate_indices.append(int(cache_idx))
    return sorted(candidate_indices)


def find_candidate_position(candidate_indices: Sequence[int], cache_idx: int) -> Optional[int]:
    if not candidate_indices:
        return None
    pos = bisect.bisect_left(candidate_indices, int(cache_idx))
    if pos < len(candidate_indices) and int(candidate_indices[pos]) == int(cache_idx):
        return int(pos)
    return None


def jump_candidate_index(candidate_indices: Sequence[int], current_idx: int, direction: int) -> Optional[int]:
    if not candidate_indices:
        return None
    if direction >= 0:
        pos = bisect.bisect_right(candidate_indices, int(current_idx))
        if pos >= len(candidate_indices):
            pos = 0
        return int(candidate_indices[pos])
    pos = bisect.bisect_left(candidate_indices, int(current_idx)) - 1
    if pos < 0:
        pos = len(candidate_indices) - 1
    return int(candidate_indices[pos])


def open_viewer(manifest: Dict, args, out_dir: Path):
    records = manifest.get("records", [])
    if not records:
        raise RuntimeError("debug session has no records")
    zero_context = load_viewer_zero_context(manifest)
    candidate_indices = load_candidate_cache_indices(manifest, out_dir, args.candidate_csv)
    manifest_offset = finite_float(manifest.get("viewer_zero_offset_px"))
    initial_zero_offset = manifest_offset if args.viewer_zero_offset_px is None else float(args.viewer_zero_offset_px)
    if initial_zero_offset is None:
        initial_zero_offset = 0.0
    manifest["viewer_zero_offset_px"] = float(initial_zero_offset)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    trackbar_name = "cache_idx"
    trackbar_max = max(1, len(records) - 1)
    idx = int(np.clip(args.start_index, 0, len(records) - 1))
    state = {
        "idx": idx,
        "input_target": None,
        "input_buffer": "",
        "overlay_rect_display": {},
        "roi_rect_display": None,
        "roi_source_shape": None,
        "pick_peak_y_rect_mode": False,
        "viewer_zero_offset_px": float(initial_zero_offset),
    }

    def on_trackbar(pos):
        state["idx"] = int(np.clip(pos, 0, len(records) - 1))

    def begin_edit_current(field: str):
        current_record = records[int(np.clip(state["idx"], 0, len(records) - 1))]
        current = current_record.get(field)
        state["pick_peak_y_rect_mode"] = False
        state["input_target"] = field
        state["input_buffer"] = "" if current is None else f"{float(current):.3f}"

    def on_mouse(event, x_pos, y_pos, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        roi_rect = state.get("roi_rect_display")
        roi_source_shape = state.get("roi_source_shape")
        if state.get("pick_peak_y_rect_mode") and roi_rect and roi_source_shape:
            roi_x, roi_y, roi_w, roi_h = roi_rect
            if roi_x <= x_pos <= roi_x + roi_w and roi_y <= y_pos <= roi_y + roi_h:
                local_y = float(np.clip(y_pos - roi_y, 0, max(0, roi_h - 1)))
                src_h = int(roi_source_shape[0])
                if src_h > 0 and roi_h > 0:
                    y_rect = float(np.clip((local_y / float(roi_h)) * src_h, 0.0, max(0.0, float(src_h - 1))))
                    save_manual_label(out_dir, manifest, int(state["idx"]), "human_peak_y_rect_px", y_rect)
                    state["pick_peak_y_rect_mode"] = False
                return
        rects = state.get("overlay_rect_display") or {}
        if not rects:
            return
        for field, rect in rects.items():
            rect_x, rect_y, rect_w, rect_h = rect
            if rect_x <= x_pos <= rect_x + rect_w and rect_y <= y_pos <= rect_y + rect_h:
                begin_edit_current(field)
                break

    cv2.createTrackbar(trackbar_name, WINDOW_NAME, idx, trackbar_max, on_trackbar)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    show_help = True
    while True:
        idx = int(np.clip(state["idx"], 0, len(records) - 1))
        record = dict(records[idx])
        photo = cv2.imread(record["photo_path"], cv2.IMREAD_COLOR)
        roi_debug = cv2.imread(record["roi_debug_path"], cv2.IMREAD_COLOR)
        if photo is None or roi_debug is None:
            raise RuntimeError(f"failed to load cached images for frame {record['frame_index']}")
        if zero_context is not None:
            zero_y_rect = float(zero_context["base_zero_y_rect"]) + float(state["viewer_zero_offset_px"])
            draw_zero_reference_on_rectified_roi(roi_debug, zero_y_rect)
            draw_zero_reference_on_full_photo(
                photo,
                zero_context["rectification_matrix"],
                int(zero_context["roi_x"]),
                int(zero_context["roi_y"]),
                int(zero_context["roi_w"]),
                zero_y_rect,
            )
            human_peak_y_rect = finite_float(record.get("human_peak_y_rect_px"))
            if human_peak_y_rect is not None:
                draw_rectified_horizontal_reference_on_rectified_roi(roi_debug, human_peak_y_rect, (255, 0, 255), thickness=2)
                draw_rectified_horizontal_reference_on_full_photo(
                    photo,
                    zero_context["rectification_matrix"],
                    int(zero_context["roi_x"]),
                    int(zero_context["roi_y"]),
                    int(zero_context["roi_w"]),
                    human_peak_y_rect,
                    (255, 0, 255),
                    thickness=2,
                )

        canvas, overlay_rect_canvas, roi_rect_canvas = compose_view(
            photo,
            roi_debug,
            record,
            records,
            idx,
            args.photo_width,
            args.roi_width,
            args.history_window,
            state["input_target"],
            str(state["input_buffer"]),
            float(state["viewer_zero_offset_px"]),
            bool(state["pick_peak_y_rect_mode"]),
        )
        display = resize_to_fit(canvas, args.window_max_width, args.window_max_height)
        candidate_pos = find_candidate_position(candidate_indices, idx)
        candidate_status = (
            "candidate frames: off"
            if not candidate_indices
            else (
                f"candidate {candidate_pos + 1}/{len(candidate_indices)}"
                if candidate_pos is not None
                else f"between candidates (total={len(candidate_indices)})"
            )
        )
        draw_text_block(display, [candidate_status], (20, 28), color=(255, 220, 120))
        scale_x = display.shape[1] / float(canvas.shape[1])
        scale_y = display.shape[0] / float(canvas.shape[0])
        state["overlay_rect_display"] = {
            field: (
                int(round(rect[0] * scale_x)),
                int(round(rect[1] * scale_y)),
                int(round(rect[2] * scale_x)),
                int(round(rect[3] * scale_y)),
            )
            for field, rect in overlay_rect_canvas.items()
        }
        state["roi_rect_display"] = (
            int(round(roi_rect_canvas[0] * scale_x)),
            int(round(roi_rect_canvas[1] * scale_y)),
            int(round(roi_rect_canvas[2] * scale_x)),
            int(round(roi_rect_canvas[3] * scale_y)),
        )
        state["roi_source_shape"] = tuple(int(v) for v in roi_debug.shape[:2])
        if show_help:
            help_lines = [
                "click viewer once to focus the window",
                "drag cache_idx trackbar to choose frame",
                f"a/d: -{SMALL_STEP_FRAMES} / +{SMALL_STEP_FRAMES} frames",
                "w/s: -10 / +10 frames",
                "z/e: -50 / +50 frames",
                "g: jump to index",
                "r/f: zero line up/down by 1 px",
                "t/v: zero line up/down by 5 px",
                "b: reset zero line offset to 0 px",
                "j/l: previous / next candidate frame",
                "i or click box: edit human center label [mm]",
                "p or click box: edit human peak label [mm]",
                "y: toggle peak-y pick mode, then click rectified ROI",
                "u: clear human_peak_y_rect_px",
                "x: clear center label, c: clear peak label",
                "h: toggle help",
                "q or Esc: quit",
            ]
            help_y = max(28, display.shape[0] - 190)
            draw_text_block(display, help_lines, (20, help_y), color=(180, 255, 180))

        cv2.resizeWindow(WINDOW_NAME, display.shape[1], display.shape[0])
        cv2.imshow(WINDOW_NAME, display)
        key = cv2.waitKeyEx(0)
        ascii_key = normalized_ascii_key(key)

        if state["input_target"] is not None:
            if key in (27,) or ascii_key == 27:
                state["input_target"] = None
                state["input_buffer"] = ""
                continue
            if key in (13, 10) or ascii_key in (13, 10):
                raw = str(state["input_buffer"]).strip()
                value = None if raw == "" else float(raw)
                save_manual_label(out_dir, manifest, idx, str(state["input_target"]), value)
                records = manifest.get("records", records)
                state["input_target"] = None
                state["input_buffer"] = ""
                continue
            if key in (8, 127) or ascii_key in (8, 127):
                state["input_buffer"] = str(state["input_buffer"])[:-1]
                continue
            if ascii_key in (ord("-"), ord(".")) or ord("0") <= ascii_key <= ord("9"):
                state["input_buffer"] = str(state["input_buffer"]) + chr(ascii_key)
                continue
            continue

        if key in (27,) or key_matches_char(key, "q", "Q"):
            break
        if key_matches_char(key, "h", "H"):
            show_help = not show_help
            continue
        if key_matches_char(key, "i", "I"):
            begin_edit_current("human_height_mm")
            continue
        if key_matches_char(key, "p", "P"):
            begin_edit_current("human_peak_mm")
            continue
        if key_matches_char(key, "y", "Y"):
            state["pick_peak_y_rect_mode"] = not bool(state["pick_peak_y_rect_mode"])
            state["input_target"] = None
            state["input_buffer"] = ""
            continue
        if key_matches_char(key, "u", "U"):
            state["pick_peak_y_rect_mode"] = False
            save_manual_label(out_dir, manifest, idx, "human_peak_y_rect_px", None)
            records = manifest.get("records", records)
            continue
        if key_matches_char(key, "x", "X"):
            save_manual_label(out_dir, manifest, idx, "human_height_mm", None)
            records = manifest.get("records", records)
            continue
        if key_matches_char(key, "c", "C"):
            save_manual_label(out_dir, manifest, idx, "human_peak_mm", None)
            records = manifest.get("records", records)
            continue
        if key_matches_char(key, "r", "R"):
            state["viewer_zero_offset_px"] = float(state["viewer_zero_offset_px"]) - 1.0
            save_viewer_zero_offset(out_dir, manifest, float(state["viewer_zero_offset_px"]))
            continue
        if key_matches_char(key, "f", "F"):
            state["viewer_zero_offset_px"] = float(state["viewer_zero_offset_px"]) + 1.0
            save_viewer_zero_offset(out_dir, manifest, float(state["viewer_zero_offset_px"]))
            continue
        if key_matches_char(key, "t", "T"):
            state["viewer_zero_offset_px"] = float(state["viewer_zero_offset_px"]) - 5.0
            save_viewer_zero_offset(out_dir, manifest, float(state["viewer_zero_offset_px"]))
            continue
        if key_matches_char(key, "v", "V"):
            state["viewer_zero_offset_px"] = float(state["viewer_zero_offset_px"]) + 5.0
            save_viewer_zero_offset(out_dir, manifest, float(state["viewer_zero_offset_px"]))
            continue
        if key_matches_char(key, "b", "B"):
            state["viewer_zero_offset_px"] = 0.0
            save_viewer_zero_offset(out_dir, manifest, 0.0)
            continue
        if key_matches_char(key, "d", "D", " ") or key in (83, 2555904):
            idx = min(len(records) - 1, idx + SMALL_STEP_FRAMES)
            state["idx"] = idx
            cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, idx)
            continue
        if key_matches_char(key, "a", "A") or key in (81, 2424832):
            idx = max(0, idx - SMALL_STEP_FRAMES)
            state["idx"] = idx
            cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, idx)
            continue
        if key_matches_char(key, "s", "S") or key in (84, 2621440):
            idx = min(len(records) - 1, idx + 10)
            state["idx"] = idx
            cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, idx)
            continue
        if key_matches_char(key, "w", "W") or key in (82, 2490368):
            idx = max(0, idx - 10)
            state["idx"] = idx
            cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, idx)
            continue
        if key_matches_char(key, "e", "E"):
            idx = min(len(records) - 1, idx + 50)
            state["idx"] = idx
            cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, idx)
            continue
        if key_matches_char(key, "z", "Z"):
            idx = max(0, idx - 50)
            state["idx"] = idx
            cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, idx)
            continue
        if key_matches_char(key, "g", "G"):
            try:
                raw = input(f"jump to cached index [0, {len(records) - 1}]: ").strip()
                if raw:
                    idx = int(np.clip(int(raw), 0, len(records) - 1))
                    state["idx"] = idx
                    cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, idx)
            except Exception:
                pass
            continue
        if key_matches_char(key, "l", "L"):
            next_idx = jump_candidate_index(candidate_indices, idx, +1)
            if next_idx is not None:
                state["idx"] = next_idx
                cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, next_idx)
            continue
        if key_matches_char(key, "j", "J"):
            prev_idx = jump_candidate_index(candidate_indices, idx, -1)
            if prev_idx is not None:
                state["idx"] = prev_idx
                cv2.setTrackbarPos(trackbar_name, WINDOW_NAME, prev_idx)
            continue

    cv2.destroyAllWindows()


def main():
    try:
        args = parse_args()
        bag_path = Path(args.bag).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else session_default_dir(bag_path)

        if args.reuse_cache:
            manifest = load_session(out_dir)
        else:
            manifest = build_session(args, out_dir)
            print(f"[OK] built v2 debug session: {out_dir}")
            print(f"[OK] manifest: {out_dir / 'debug_session.json'}")
            print(f"[OK] summary csv: {out_dir / 'debug_session.csv'}")
            print(f"[OK] human labels csv: {out_dir / 'human_labels.csv'}")

        if args.show_frame_index >= 0:
            print_selected_frame(manifest, args.show_frame_index)

        if args.skip_viewer:
            return 0

        open_viewer(manifest, args, out_dir)
        return 0
    except RuntimeError as exc:
        print(f"[WARN] {exc}")
        return 1
    except KeyboardInterrupt:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        print("\n[INFO] interrupted by Ctrl+C, viewer closed.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
