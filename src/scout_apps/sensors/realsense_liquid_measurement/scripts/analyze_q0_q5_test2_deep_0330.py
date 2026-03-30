#!/usr/bin/env python3
"""Deep paired analysis for 0330 Q0_test2 vs Q5_test2.

This script focuses on two questions:
1. In which time segments is Q5_test2 weaker/stronger than Q0_test2 in slosh amplitude?
2. Is the generated /local_path for Q5_test2 smoother than Q0_test2?
"""

import argparse
import bisect
import csv
import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DEFAULT_BAG_ROOT = Path("/data/a/slosh_bags/0330")
DEFAULT_VERIFY_ROOT = Path("/data/a/realsense_validation_v2/verify/0330_rezero_bias")
DEFAULT_OUT_DIR = Path("/data/a/realsense_validation_v2/verify/0330_rezero_bias/test2_deep_analysis")


@dataclass
class BagSeries:
    label: str
    bag_path: Path
    slosh_t: np.ndarray
    slosh_mm_zero: np.ndarray
    visual_t: np.ndarray
    visual_mm: np.ndarray
    local_path_snapshots: List["PathSnapshot"]
    odom_t: np.ndarray
    odom_xy: np.ndarray
    odom_speed_mps: np.ndarray
    odom_accel_mps2: np.ndarray


@dataclass
class PathSnapshot:
    rel_time_s: float
    pose_count: int
    path_length_m: float
    heading_tv_deg: float
    curvature_abs_p90: float
    curvature_abs_max: float
    curvature_change_abs_p90: float
    endpoint_x_m: float
    endpoint_y_m: float
    endpoint_jitter_prev_m: float
    shape_delta_prev_mean_m: float
    points_xy: np.ndarray


@dataclass
class SegmentRow:
    segment_id: str
    t_start_s: float
    t_end_s: float
    duration_s: float
    q0_slosh_abs_p90_mm: float
    q5_slosh_abs_p90_mm: float
    q5_over_q0_slosh_p90_ratio: float
    q0_slosh_abs_max_mm: float
    q5_slosh_abs_max_mm: float
    q5_over_q0_slosh_max_ratio: float
    q0_slosh_abs_area_mm_s: float
    q5_slosh_abs_area_mm_s: float
    q5_over_q0_slosh_area_ratio: float
    q0_center_abs_p90_mm: float
    q5_center_abs_p90_mm: float
    q5_over_q0_center_p90_ratio: float
    q0_center_abs_max_mm: float
    q5_center_abs_max_mm: float
    q5_over_q0_center_max_ratio: float


@dataclass
class LocalPathSummaryRow:
    label: str
    update_count: int
    pose_count_mean: float
    path_length_mean_m: float
    heading_tv_mean_deg: float
    heading_tv_p90_deg: float
    curvature_abs_p90_mean: float
    curvature_abs_max_mean: float
    curvature_change_abs_p90_mean: float
    endpoint_jitter_prev_mean_m: float
    endpoint_jitter_prev_p90_m: float
    shape_delta_prev_mean_m: float
    shape_delta_prev_p90_m: float


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-root", default=str(DEFAULT_BAG_ROOT))
    parser.add_argument("--verify-root", default=str(DEFAULT_VERIFY_ROOT))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    parser.add_argument("--q0-label", default="Q0_test2")
    parser.add_argument("--q5-label", default="Q5_test2")
    parser.add_argument(
        "--center-column",
        default="height_peak_rel_mm_signed_v2",
        help="Visual proxy column; default uses RS peak signed height.",
    )
    parser.add_argument(
        "--visual-proxy-label",
        default="RS peak",
        help="Legend label for the visual proxy curve.",
    )
    parser.add_argument(
        "--zero-align-window-sec",
        type=float,
        default=1.0,
        help="Initial /slosh/height zero-align window.",
    )
    parser.add_argument(
        "--zero-align-max-samples",
        type=int,
        default=30,
        help="Maximum early samples used for /slosh/height zero alignment.",
    )
    parser.add_argument(
        "--segment-threshold-mm",
        type=float,
        default=0.08,
        help="Activity threshold on max(|Q0 slosh|, |Q5 slosh|) for segment extraction.",
    )
    parser.add_argument(
        "--segment-min-duration-sec",
        type=float,
        default=0.35,
        help="Minimum active segment duration.",
    )
    parser.add_argument(
        "--segment-merge-gap-sec",
        type=float,
        default=0.30,
        help="Merge adjacent active windows separated by less than this gap.",
    )
    parser.add_argument(
        "--segment-sample-dt-sec",
        type=float,
        default=0.05,
        help="Uniform sampling interval used for paired segment extraction.",
    )
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def finite_float(raw_value) -> float:
    if raw_value is None:
        return math.nan
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def median_or_zero(values: Sequence[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return 0.0
    return float(statistics.median(clean))


def percentile90(values: Iterable[float]) -> float:
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return math.nan
    idx = int(round((len(clean) - 1) * 0.9))
    return float(clean[idx])


def safe_ratio(num: float, den: float) -> float:
    if not math.isfinite(num) or not math.isfinite(den):
        return math.nan
    if abs(den) <= 1e-9:
        return math.inf if abs(num) > 1e-9 else 1.0
    return float(num / den)


def zero_align_offset(times: np.ndarray, values: np.ndarray, window_sec: float, max_samples: int) -> float:
    pairs = [(float(t), float(v)) for t, v in zip(times, values) if math.isfinite(float(v)) and math.isfinite(float(t))]
    if not pairs:
        return 0.0
    selected = [v for t, v in pairs if t <= float(window_sec)]
    if len(selected) < 3:
        selected = [v for _, v in pairs[: max(1, int(max_samples))]]
    else:
        selected = selected[: max(1, int(max_samples))]
    return median_or_zero(selected)


def interpolate_scalar(times: Sequence[float], values: Sequence[float], ts: float) -> float:
    if len(times) == 0:
        return math.nan
    idx = bisect.bisect_left(times, ts)
    if idx <= 0:
        return float(values[0])
    if idx >= len(times):
        return float(values[-1])
    t0 = float(times[idx - 1])
    t1 = float(times[idx])
    v0 = float(values[idx - 1])
    v1 = float(values[idx])
    if t1 <= t0:
        return v1
    ratio = (ts - t0) / (t1 - t0)
    return float(v0 + ratio * (v1 - v0))


def path_length(points: np.ndarray) -> float:
    if points.shape[0] < 2:
        return 0.0
    diff = np.diff(points, axis=0)
    return float(np.sum(np.hypot(diff[:, 0], diff[:, 1])))


def heading_series(points: np.ndarray) -> np.ndarray:
    if points.shape[0] < 2:
        return np.zeros((0,), dtype=np.float64)
    diff = np.diff(points, axis=0)
    return np.asarray([math.atan2(float(v[1]), float(v[0])) for v in diff], dtype=np.float64)


def wrapped_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def curvature_series(points: np.ndarray) -> np.ndarray:
    if points.shape[0] < 3:
        return np.zeros((0,), dtype=np.float64)
    headings = heading_series(points)
    seg_len = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))
    out: List[float] = []
    for idx in range(1, len(headings)):
        ds = float(seg_len[idx - 1] + seg_len[idx]) * 0.5
        if ds <= 1e-9:
            continue
        dtheta = wrapped_angle(float(headings[idx] - headings[idx - 1]))
        out.append(float(dtheta / ds))
    return np.asarray(out, dtype=np.float64)


def resample_polyline(points: np.ndarray, num_points: int = 80) -> np.ndarray:
    if points.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64)
    if points.shape[0] == 1:
        return np.repeat(points[:1], num_points, axis=0)
    seg = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total = float(s[-1])
    if total <= 1e-9:
        return np.repeat(points[:1], num_points, axis=0)
    targets = np.linspace(0.0, total, num_points)
    x_interp = np.interp(targets, s, points[:, 0])
    y_interp = np.interp(targets, s, points[:, 1])
    return np.column_stack([x_interp, y_interp])


def mean_path_distance(points_a: np.ndarray, points_b: np.ndarray) -> float:
    if points_a.shape[0] == 0 or points_b.shape[0] == 0:
        return math.nan
    ra = resample_polyline(points_a)
    rb = resample_polyline(points_b)
    n = min(ra.shape[0], rb.shape[0])
    if n == 0:
        return math.nan
    dist = np.hypot(ra[:n, 0] - rb[:n, 0], ra[:n, 1] - rb[:n, 1])
    return float(np.mean(dist))


def compute_path_snapshot(rel_time_s: float, points: np.ndarray, prev_points: Optional[np.ndarray]) -> PathSnapshot:
    headings = heading_series(points)
    heading_tv_deg = float(np.degrees(np.sum(np.abs([wrapped_angle(float(headings[i] - headings[i - 1])) for i in range(1, len(headings))])))) if len(headings) >= 2 else 0.0
    curv = curvature_series(points)
    curv_abs = np.abs(curv) if curv.size else np.zeros((0,), dtype=np.float64)
    curv_diff = np.abs(np.diff(curv)) if curv.size >= 2 else np.zeros((0,), dtype=np.float64)
    endpoint_x = float(points[-1, 0]) if points.shape[0] else math.nan
    endpoint_y = float(points[-1, 1]) if points.shape[0] else math.nan
    if prev_points is not None and prev_points.shape[0] and points.shape[0]:
        endpoint_jitter = float(np.hypot(points[-1, 0] - prev_points[-1, 0], points[-1, 1] - prev_points[-1, 1]))
        shape_delta = mean_path_distance(prev_points, points)
    else:
        endpoint_jitter = math.nan
        shape_delta = math.nan
    return PathSnapshot(
        rel_time_s=float(rel_time_s),
        pose_count=int(points.shape[0]),
        path_length_m=path_length(points),
        heading_tv_deg=heading_tv_deg,
        curvature_abs_p90=percentile90(curv_abs),
        curvature_abs_max=float(np.max(curv_abs)) if curv_abs.size else 0.0,
        curvature_change_abs_p90=percentile90(curv_diff),
        endpoint_x_m=endpoint_x,
        endpoint_y_m=endpoint_y,
        endpoint_jitter_prev_m=endpoint_jitter,
        shape_delta_prev_mean_m=shape_delta,
        points_xy=points.copy(),
    )


def resolve_bag_path(label: str, bag_root: Path) -> Path:
    matches = sorted(p for p in bag_root.glob("*.bag") if label.lower() in p.name.lower())
    if len(matches) != 1:
        raise RuntimeError(f"expected exactly one bag for {label} under {bag_root}, got {matches}")
    return matches[0]


def load_local_path_snapshots(bag_path: Path) -> List[PathSnapshot]:
    snapshots: List[PathSnapshot] = []
    prev_points: Optional[np.ndarray] = None
    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        for _, msg, stamp in bag.read_messages(topics=["/local_path"]):
            rel_time = float(stamp.to_sec() - bag_start)
            poses = getattr(msg, "poses", [])
            points = np.asarray(
                [(float(p.pose.position.x), float(p.pose.position.y)) for p in poses],
                dtype=np.float64,
            )
            snapshots.append(compute_path_snapshot(rel_time, points, prev_points))
            prev_points = points
    return snapshots


def load_bag_series(label: str, bag_root: Path, verify_root: Path, center_column: str, zero_align_window_sec: float, zero_align_max_samples: int) -> BagSeries:
    bag_path = resolve_bag_path(label, bag_root)
    verify_csv = verify_root / label / "liquid_height_v2.csv"
    if not verify_csv.is_file():
        raise RuntimeError(f"missing liquid csv for {label}: {verify_csv}")

    slosh_t: List[float] = []
    slosh_mm: List[float] = []
    odom_t: List[float] = []
    odom_xy: List[Tuple[float, float]] = []
    odom_speed: List[float] = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=["/slosh/height", "/odom"]):
            rel_time = float(stamp.to_sec() - bag_start)
            if topic == "/slosh/height":
                slosh_t.append(rel_time)
                slosh_mm.append(float(msg.data) * 1000.0)
            elif topic == "/odom":
                try:
                    pose = msg.pose.pose.position
                    twist = msg.twist.twist.linear
                    odom_t.append(rel_time)
                    odom_xy.append((float(pose.x), float(pose.y)))
                    odom_speed.append(float(math.hypot(float(twist.x), float(twist.y))))
                except AttributeError:
                    continue

    slosh_t_np = np.asarray(slosh_t, dtype=np.float64)
    slosh_mm_np = np.asarray(slosh_mm, dtype=np.float64)
    slosh_offset = zero_align_offset(slosh_t_np, slosh_mm_np, zero_align_window_sec, zero_align_max_samples)
    slosh_mm_zero = np.asarray([float(v - slosh_offset) for v in slosh_mm_np], dtype=np.float64)

    visual_t: List[float] = []
    visual_raw_mm: List[float] = []
    with verify_csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            if int(raw.get("accept_for_peak_report_v2", "0") or "0") != 1:
                continue
            rel_time = finite_float(raw.get("relative_time_s"))
            center = finite_float(raw.get(center_column))
            if math.isfinite(rel_time) and math.isfinite(center):
                visual_t.append(rel_time)
                visual_raw_mm.append(center)

    visual_t_np = np.asarray(visual_t, dtype=np.float64)
    visual_raw_np = np.asarray(visual_raw_mm, dtype=np.float64)
    visual_offset = zero_align_offset(visual_t_np, visual_raw_np, zero_align_window_sec, zero_align_max_samples)
    visual_mm_np = np.asarray([float(v - visual_offset) for v in visual_raw_np], dtype=np.float64)

    odom_t_np = np.asarray(odom_t, dtype=np.float64)
    odom_xy_np = np.asarray(odom_xy, dtype=np.float64)
    odom_speed_np = np.asarray(odom_speed, dtype=np.float64)
    if odom_t_np.size >= 3:
        odom_accel_np = np.gradient(odom_speed_np, odom_t_np)
    else:
        odom_accel_np = np.zeros_like(odom_speed_np)

    return BagSeries(
        label=label,
        bag_path=bag_path,
        slosh_t=slosh_t_np,
        slosh_mm_zero=slosh_mm_zero,
        visual_t=visual_t_np,
        visual_mm=visual_mm_np,
        local_path_snapshots=load_local_path_snapshots(bag_path),
        odom_t=odom_t_np,
        odom_xy=odom_xy_np,
        odom_speed_mps=odom_speed_np,
        odom_accel_mps2=odom_accel_np,
    )


def extract_segments(q0: BagSeries, q5: BagSeries, threshold_mm: float, min_duration_sec: float, merge_gap_sec: float, sample_dt_sec: float) -> List[Tuple[float, float]]:
    start = max(float(q0.slosh_t[0]), float(q5.slosh_t[0]))
    end = min(float(q0.slosh_t[-1]), float(q5.slosh_t[-1]))
    if end <= start:
        return []
    sample_t = np.arange(start, end + 1e-9, float(sample_dt_sec), dtype=np.float64)
    q0_interp = np.asarray([interpolate_scalar(q0.slosh_t, q0.slosh_mm_zero, float(t)) for t in sample_t], dtype=np.float64)
    q5_interp = np.asarray([interpolate_scalar(q5.slosh_t, q5.slosh_mm_zero, float(t)) for t in sample_t], dtype=np.float64)
    activity = np.maximum(np.abs(q0_interp), np.abs(q5_interp)) >= float(threshold_mm)

    segments: List[Tuple[float, float]] = []
    active_start: Optional[float] = None
    last_active_t: Optional[float] = None
    for t, active in zip(sample_t, activity):
        if bool(active):
            if active_start is None:
                active_start = float(t)
            last_active_t = float(t)
            continue
        if active_start is not None and last_active_t is not None:
            if float(last_active_t - active_start) >= float(min_duration_sec):
                segments.append((active_start, last_active_t))
            active_start = None
            last_active_t = None
    if active_start is not None and last_active_t is not None and float(last_active_t - active_start) >= float(min_duration_sec):
        segments.append((active_start, last_active_t))

    if not segments:
        return [(start, end)]

    merged: List[Tuple[float, float]] = [segments[0]]
    for seg_start, seg_end in segments[1:]:
        prev_start, prev_end = merged[-1]
        if seg_start - prev_end <= float(merge_gap_sec):
            merged[-1] = (prev_start, seg_end)
        else:
            merged.append((seg_start, seg_end))
    return merged


def values_in_window(times: np.ndarray, values: np.ndarray, t_start: float, t_end: float) -> List[float]:
    out = []
    for t, v in zip(times, values):
        if t_start <= float(t) <= t_end and math.isfinite(float(v)):
            out.append(float(v))
    return out


def abs_area_in_window(times: np.ndarray, values: np.ndarray, t_start: float, t_end: float) -> float:
    mask = [(float(t), float(v)) for t, v in zip(times, values) if t_start <= float(t) <= t_end and math.isfinite(float(v))]
    if len(mask) < 2:
        return 0.0
    t = np.asarray([p[0] for p in mask], dtype=np.float64)
    v = np.asarray([abs(p[1]) for p in mask], dtype=np.float64)
    return float(np.trapz(v, t))


def summarize_segments(q0: BagSeries, q5: BagSeries, segments: Sequence[Tuple[float, float]]) -> List[SegmentRow]:
    rows: List[SegmentRow] = []
    for idx, (t_start, t_end) in enumerate(segments, start=1):
        q0_slosh = values_in_window(q0.slosh_t, q0.slosh_mm_zero, t_start, t_end)
        q5_slosh = values_in_window(q5.slosh_t, q5.slosh_mm_zero, t_start, t_end)
        q0_center = values_in_window(q0.visual_t, q0.visual_mm, t_start, t_end)
        q5_center = values_in_window(q5.visual_t, q5.visual_mm, t_start, t_end)

        q0_slosh_abs = [abs(v) for v in q0_slosh]
        q5_slosh_abs = [abs(v) for v in q5_slosh]
        q0_center_abs = [abs(v) for v in q0_center]
        q5_center_abs = [abs(v) for v in q5_center]

        q0_slosh_p90 = percentile90(q0_slosh_abs)
        q5_slosh_p90 = percentile90(q5_slosh_abs)
        q0_slosh_max = max(q0_slosh_abs) if q0_slosh_abs else math.nan
        q5_slosh_max = max(q5_slosh_abs) if q5_slosh_abs else math.nan
        q0_slosh_area = abs_area_in_window(q0.slosh_t, q0.slosh_mm_zero, t_start, t_end)
        q5_slosh_area = abs_area_in_window(q5.slosh_t, q5.slosh_mm_zero, t_start, t_end)
        q0_center_p90 = percentile90(q0_center_abs)
        q5_center_p90 = percentile90(q5_center_abs)
        q0_center_max = max(q0_center_abs) if q0_center_abs else math.nan
        q5_center_max = max(q5_center_abs) if q5_center_abs else math.nan

        rows.append(
            SegmentRow(
                segment_id=f"segment_{idx:02d}",
                t_start_s=float(t_start),
                t_end_s=float(t_end),
                duration_s=float(t_end - t_start),
                q0_slosh_abs_p90_mm=q0_slosh_p90,
                q5_slosh_abs_p90_mm=q5_slosh_p90,
                q5_over_q0_slosh_p90_ratio=safe_ratio(q5_slosh_p90, q0_slosh_p90),
                q0_slosh_abs_max_mm=q0_slosh_max,
                q5_slosh_abs_max_mm=q5_slosh_max,
                q5_over_q0_slosh_max_ratio=safe_ratio(q5_slosh_max, q0_slosh_max),
                q0_slosh_abs_area_mm_s=q0_slosh_area,
                q5_slosh_abs_area_mm_s=q5_slosh_area,
                q5_over_q0_slosh_area_ratio=safe_ratio(q5_slosh_area, q0_slosh_area),
                q0_center_abs_p90_mm=q0_center_p90,
                q5_center_abs_p90_mm=q5_center_p90,
                q5_over_q0_center_p90_ratio=safe_ratio(q5_center_p90, q0_center_p90),
                q0_center_abs_max_mm=q0_center_max,
                q5_center_abs_max_mm=q5_center_max,
                q5_over_q0_center_max_ratio=safe_ratio(q5_center_max, q0_center_max),
            )
        )
    return rows


def summarize_local_path(label: str, snapshots: Sequence[PathSnapshot]) -> LocalPathSummaryRow:
    return LocalPathSummaryRow(
        label=label,
        update_count=len(snapshots),
        pose_count_mean=statistics.mean([s.pose_count for s in snapshots]) if snapshots else math.nan,
        path_length_mean_m=statistics.mean([s.path_length_m for s in snapshots]) if snapshots else math.nan,
        heading_tv_mean_deg=statistics.mean([s.heading_tv_deg for s in snapshots]) if snapshots else math.nan,
        heading_tv_p90_deg=percentile90([s.heading_tv_deg for s in snapshots]),
        curvature_abs_p90_mean=statistics.mean([s.curvature_abs_p90 for s in snapshots if math.isfinite(s.curvature_abs_p90)]) if snapshots else math.nan,
        curvature_abs_max_mean=statistics.mean([s.curvature_abs_max for s in snapshots]) if snapshots else math.nan,
        curvature_change_abs_p90_mean=statistics.mean([s.curvature_change_abs_p90 for s in snapshots if math.isfinite(s.curvature_change_abs_p90)]) if snapshots else math.nan,
        endpoint_jitter_prev_mean_m=statistics.mean([s.endpoint_jitter_prev_m for s in snapshots if math.isfinite(s.endpoint_jitter_prev_m)]) if snapshots else math.nan,
        endpoint_jitter_prev_p90_m=percentile90([s.endpoint_jitter_prev_m for s in snapshots if math.isfinite(s.endpoint_jitter_prev_m)]),
        shape_delta_prev_mean_m=statistics.mean([s.shape_delta_prev_mean_m for s in snapshots if math.isfinite(s.shape_delta_prev_mean_m)]) if snapshots else math.nan,
        shape_delta_prev_p90_m=percentile90([s.shape_delta_prev_mean_m for s in snapshots if math.isfinite(s.shape_delta_prev_mean_m)]),
    )


def summarize_local_path_segments(segment_rows: Sequence[SegmentRow], q0_snapshots: Sequence[PathSnapshot], q5_snapshots: Sequence[PathSnapshot]) -> List[Dict[str, object]]:
    out: List[Dict[str, object]] = []
    for row in segment_rows:
        t0 = row.t_start_s
        t1 = row.t_end_s
        q0_seg = [s for s in q0_snapshots if t0 <= s.rel_time_s <= t1]
        q5_seg = [s for s in q5_snapshots if t0 <= s.rel_time_s <= t1]
        q0_summary = summarize_local_path("Q0_test2", q0_seg)
        q5_summary = summarize_local_path("Q5_test2", q5_seg)
        out.append(
            {
                "segment_id": row.segment_id,
                "t_start_s": row.t_start_s,
                "t_end_s": row.t_end_s,
                "q0_update_count": q0_summary.update_count,
                "q5_update_count": q5_summary.update_count,
                "q0_heading_tv_mean_deg": q0_summary.heading_tv_mean_deg,
                "q5_heading_tv_mean_deg": q5_summary.heading_tv_mean_deg,
                "q5_over_q0_heading_tv_ratio": safe_ratio(q5_summary.heading_tv_mean_deg, q0_summary.heading_tv_mean_deg),
                "q0_curvature_change_abs_p90_mean": q0_summary.curvature_change_abs_p90_mean,
                "q5_curvature_change_abs_p90_mean": q5_summary.curvature_change_abs_p90_mean,
                "q5_over_q0_curvature_change_ratio": safe_ratio(q5_summary.curvature_change_abs_p90_mean, q0_summary.curvature_change_abs_p90_mean),
                "q0_shape_delta_prev_mean_m": q0_summary.shape_delta_prev_mean_m,
                "q5_shape_delta_prev_mean_m": q5_summary.shape_delta_prev_mean_m,
                "q5_over_q0_shape_delta_ratio": safe_ratio(q5_summary.shape_delta_prev_mean_m, q0_summary.shape_delta_prev_mean_m),
                "q0_endpoint_jitter_prev_mean_m": q0_summary.endpoint_jitter_prev_mean_m,
                "q5_endpoint_jitter_prev_mean_m": q5_summary.endpoint_jitter_prev_mean_m,
                "q5_over_q0_endpoint_jitter_ratio": safe_ratio(q5_summary.endpoint_jitter_prev_mean_m, q0_summary.endpoint_jitter_prev_mean_m),
            }
        )
    return out


def plot_segment_amplitude(out_path: Path, q0: BagSeries, q5: BagSeries, segments: Sequence[Tuple[float, float]], visual_proxy_label: str):
    ensure_dir(out_path.parent)
    fig, ax0 = plt.subplots(1, 1, figsize=(13, 5), constrained_layout=True)
    ax0.plot(q0.slosh_t, q0.slosh_mm_zero, color="#1f77b4", linewidth=1.3, label="Q0_test2 /slosh/height")
    ax0.plot(q5.slosh_t, q5.slosh_mm_zero, color="#d62728", linewidth=1.3, label="Q5_test2 /slosh/height")
    ax0.plot(q0.visual_t, q0.visual_mm, color="#1f77b4", linewidth=1.0, linestyle="--", alpha=0.8, label=f"Q0_test2 {visual_proxy_label}")
    ax0.plot(q5.visual_t, q5.visual_mm, color="#d62728", linewidth=1.0, linestyle="--", alpha=0.8, label=f"Q5_test2 {visual_proxy_label}")
    ax0.set_ylabel("height [mm]")
    ax0.set_xlabel("relative time [s]")
    ax0.set_title("Q0_test2 vs Q5_test2: segmented slosh/center amplitude")
    ax0.grid(True, alpha=0.25)
    ax0.legend(loc="upper right", ncol=2)

    summary_lines: List[str] = []
    for idx, (t_start, t_end) in enumerate(segments, start=1):
        q0_vals = [abs(v) for v in values_in_window(q0.slosh_t, q0.slosh_mm_zero, t_start, t_end)]
        q5_vals = [abs(v) for v in values_in_window(q5.slosh_t, q5.slosh_mm_zero, t_start, t_end)]
        q0_p90 = percentile90(q0_vals)
        q5_p90 = percentile90(q5_vals)
        ax0.axvspan(t_start, t_end, color="#bbbbbb", alpha=0.12)
        summary_lines.append(
            f"S{idx}: Q0 p90={q0_p90:.3f} mm, Q5 p90={q5_p90:.3f} mm"
        )

    if summary_lines:
        ax0.text(
            0.015,
            0.98,
            "\n".join(summary_lines),
            transform=ax0.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.75, "edgecolor": "#bbbbbb"},
        )

    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def plot_local_path_smoothness(out_path: Path, q0: BagSeries, q5: BagSeries):
    ensure_dir(out_path.parent)
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True, constrained_layout=True)
    q0_t = [s.rel_time_s for s in q0.local_path_snapshots]
    q5_t = [s.rel_time_s for s in q5.local_path_snapshots]

    axes[0].plot(q0_t, [s.heading_tv_deg for s in q0.local_path_snapshots], color="#1f77b4", linewidth=1.1, label="Q0_test2")
    axes[0].plot(q5_t, [s.heading_tv_deg for s in q5.local_path_snapshots], color="#d62728", linewidth=1.1, label="Q5_test2")
    axes[0].set_ylabel("heading TV [deg]")
    axes[0].set_title("/local_path smoothness over time")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")

    axes[1].plot(q0_t, [s.curvature_change_abs_p90 for s in q0.local_path_snapshots], color="#1f77b4", linewidth=1.1)
    axes[1].plot(q5_t, [s.curvature_change_abs_p90 for s in q5.local_path_snapshots], color="#d62728", linewidth=1.1)
    axes[1].set_ylabel("curvature Δ p90")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(q0_t, [s.shape_delta_prev_mean_m for s in q0.local_path_snapshots], color="#1f77b4", linewidth=1.1)
    axes[2].plot(q5_t, [s.shape_delta_prev_mean_m for s in q5.local_path_snapshots], color="#d62728", linewidth=1.1)
    axes[2].set_ylabel("shape delta [m]")
    axes[2].set_xlabel("relative time [s]")
    axes[2].grid(True, alpha=0.25)

    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def snapshots_in_segments(snapshots: Sequence[PathSnapshot], segments: Sequence[Tuple[float, float]]) -> List[PathSnapshot]:
    out: List[PathSnapshot] = []
    for snap in snapshots:
        for t_start, t_end in segments:
            if t_start <= snap.rel_time_s <= t_end:
                out.append(snap)
                break
    return out


def choose_representative_snapshot(snapshots: Sequence[PathSnapshot]) -> Optional[PathSnapshot]:
    if not snapshots:
        return None
    ordered = sorted(snapshots, key=lambda s: s.rel_time_s)
    return ordered[len(ordered) // 2]


def plot_local_path_topdown(out_path: Path, q0: BagSeries, q5: BagSeries, segments: Sequence[Tuple[float, float]]):
    ensure_dir(out_path.parent)
    q0_seg = snapshots_in_segments(q0.local_path_snapshots, segments)
    q5_seg = snapshots_in_segments(q5.local_path_snapshots, segments)
    q0_rep = choose_representative_snapshot(q0_seg)
    q5_rep = choose_representative_snapshot(q5_seg)

    fig, ax = plt.subplots(figsize=(8, 8), constrained_layout=True)

    def draw_bundle(snaps: Sequence[PathSnapshot], color: str, label: str):
        if not snaps:
            return
        step = max(1, len(snaps) // 18)
        for snap in snaps[::step]:
            pts = snap.points_xy
            if pts.shape[0] < 2:
                continue
            ax.plot(pts[:, 0], pts[:, 1], color=color, alpha=0.12, linewidth=0.9)
        rep = choose_representative_snapshot(snaps)
        if rep is not None and rep.points_xy.shape[0] >= 2:
            pts = rep.points_xy
            ax.plot(pts[:, 0], pts[:, 1], color=color, alpha=0.95, linewidth=2.2, label=label)
            ax.scatter([pts[0, 0]], [pts[0, 1]], color=color, s=30, marker="o")
            ax.scatter([pts[-1, 0]], [pts[-1, 1]], color=color, s=36, marker="x")

    draw_bundle(q0_seg, "#1f77b4", "Q0_test2 representative local_path")
    draw_bundle(q5_seg, "#d62728", "Q5_test2 representative local_path")

    if q0_rep is not None:
        ax.annotate("Q0 start", (q0_rep.points_xy[0, 0], q0_rep.points_xy[0, 1]), color="#1f77b4", fontsize=9)
    if q5_rep is not None:
        ax.annotate("Q5 start", (q5_rep.points_xy[0, 0], q5_rep.points_xy[0, 1]), color="#d62728", fontsize=9)

    ax.set_title("/local_path top-down view in active segment")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.legend(loc="best")
    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def odom_points_in_segments(times: np.ndarray, points: np.ndarray, segments: Sequence[Tuple[float, float]]) -> np.ndarray:
    if times.size == 0 or points.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    mask = np.zeros(times.shape, dtype=bool)
    for t_start, t_end in segments:
        mask |= (times >= float(t_start)) & (times <= float(t_end))
    if not np.any(mask):
        return np.zeros((0, 2), dtype=np.float64)
    return points[mask]


def plot_odom_topdown(out_path: Path, q0: BagSeries, q5: BagSeries, segments: Sequence[Tuple[float, float]]):
    ensure_dir(out_path.parent)
    q0_pts = odom_points_in_segments(q0.odom_t, q0.odom_xy, segments)
    q5_pts = odom_points_in_segments(q5.odom_t, q5.odom_xy, segments)

    fig, ax = plt.subplots(figsize=(8, 8), constrained_layout=True)

    if q0_pts.shape[0] >= 2:
        ax.plot(q0_pts[:, 0], q0_pts[:, 1], color="#1f77b4", linewidth=2.0, label="Q0_test2 odom")
        ax.scatter([q0_pts[0, 0]], [q0_pts[0, 1]], color="#1f77b4", s=34, marker="o")
        ax.scatter([q0_pts[-1, 0]], [q0_pts[-1, 1]], color="#1f77b4", s=40, marker="x")
        ax.annotate("Q0 start", (q0_pts[0, 0], q0_pts[0, 1]), color="#1f77b4", fontsize=9)
    if q5_pts.shape[0] >= 2:
        ax.plot(q5_pts[:, 0], q5_pts[:, 1], color="#d62728", linewidth=2.0, label="Q5_test2 odom")
        ax.scatter([q5_pts[0, 0]], [q5_pts[0, 1]], color="#d62728", s=34, marker="o")
        ax.scatter([q5_pts[-1, 0]], [q5_pts[-1, 1]], color="#d62728", s=40, marker="x")
        ax.annotate("Q5 start", (q5_pts[0, 0], q5_pts[0, 1]), color="#d62728", fontsize=9)

    ax.set_title("Odom top-down trajectory in active segment")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.legend(loc="best")
    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def plot_odom_kinematics(out_path: Path, q0: BagSeries, q5: BagSeries, segments: Sequence[Tuple[float, float]]):
    ensure_dir(out_path.parent)
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True, constrained_layout=True)
    ax0, ax1 = axes

    ax0.plot(q0.odom_t, q0.odom_speed_mps, color="#1f77b4", linewidth=1.4, label="Q0_test2 odom speed")
    ax0.plot(q5.odom_t, q5.odom_speed_mps, color="#d62728", linewidth=1.4, label="Q5_test2 odom speed")
    ax0.set_ylabel("speed [m/s]")
    ax0.set_title("Odom kinematics over time")
    ax0.grid(True, alpha=0.25)
    ax0.legend(loc="upper right")

    ax1.plot(q0.odom_t, q0.odom_accel_mps2, color="#1f77b4", linewidth=1.2, label="Q0_test2 odom accel")
    ax1.plot(q5.odom_t, q5.odom_accel_mps2, color="#d62728", linewidth=1.2, label="Q5_test2 odom accel")
    ax1.set_ylabel("accel [m/s^2]")
    ax1.set_xlabel("relative time [s]")
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="upper right")

    for t_start, t_end in segments:
        ax0.axvspan(t_start, t_end, color="#bbbbbb", alpha=0.12)
        ax1.axvspan(t_start, t_end, color="#bbbbbb", alpha=0.12)

    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def build_readme(path: Path, segment_rows: Sequence[SegmentRow], local_rows: Sequence[LocalPathSummaryRow], segment_local_rows: Sequence[Dict[str, object]], visual_proxy_label: str):
    by_label = {row.label: row for row in local_rows}
    lines = [
        "# 0330 test2 深入分析",
        "",
        "目标：",
        "- 分段比较 `Q0_test2` 与 `Q5_test2` 的液面幅值",
        "- 分析 `/local_path` 是否更平滑，从而影响液体晃动",
        "",
        "当前业务目标：证明在同任务、同路径、可比激励下，`Q=5` 时液体晃动比 `Q=0` 更轻微。",
        "",
        "主要输出：",
        "- `segment_amplitude_summary.csv`",
        "- `local_path_smoothness_summary.csv`",
        "- `segment_local_path_smoothness.csv`",
        "- `test2_segment_amplitude.png`",
        "- `test2_local_path_smoothness.png`",
        "- `test2_local_path_topdown.png`",
        "- `test2_odom_topdown.png`",
        "- `test2_odom_kinematics.png`",
        "",
        f"当前视觉代理：`{visual_proxy_label}`",
        "",
        "整包 /local_path 平滑性摘要：",
    ]
    for label in ("Q0_test2", "Q5_test2"):
        row = by_label.get(label)
        if row is None:
            continue
        lines.append(
            f"- `{label}`: updates={row.update_count}, "
            f"heading_tv_mean={row.heading_tv_mean_deg:.3f} deg, "
            f"curvature_change_p90_mean={row.curvature_change_abs_p90_mean:.6f}, "
            f"shape_delta_prev_mean={row.shape_delta_prev_mean_m:.4f} m"
        )
    lines.append("")
    lines.append("分段幅值摘要：")
    for row in segment_rows:
        lines.append(
            f"- `{row.segment_id}` [{row.t_start_s:.2f}, {row.t_end_s:.2f}] s: "
                f"Q5/Q0 slosh p90 ratio={row.q5_over_q0_slosh_p90_ratio:.3f}, "
                f"Q5/Q0 slosh area ratio={row.q5_over_q0_slosh_area_ratio:.3f}, "
                f"Q5/Q0 {visual_proxy_label} p90 ratio={row.q5_over_q0_center_p90_ratio:.3f}"
            )
    lines.append("")
    if segment_local_rows:
        lines.append("分段 local_path 平滑性摘要：")
        for row in segment_local_rows:
            lines.append(
                f"- `{row['segment_id']}`: "
                f"Q5/Q0 heading_tv ratio={row['q5_over_q0_heading_tv_ratio']:.3f}, "
                f"Q5/Q0 curvature_change ratio={row['q5_over_q0_curvature_change_ratio']:.3f}, "
                f"Q5/Q0 shape_delta ratio={row['q5_over_q0_shape_delta_ratio']:.3f}"
            )
        lines.append("")
    lines.append("明天代办：")
    lines.append("- 用逐帧调试器补 `human_peak_mm` 小批量人工峰值标签，做 `/slosh/height` 的严格峰值验证。")
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    bag_root = Path(args.bag_root).expanduser().resolve()
    verify_root = Path(args.verify_root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    ensure_dir(out_dir)

    q0 = load_bag_series(args.q0_label, bag_root, verify_root, args.center_column, args.zero_align_window_sec, args.zero_align_max_samples)
    q5 = load_bag_series(args.q5_label, bag_root, verify_root, args.center_column, args.zero_align_window_sec, args.zero_align_max_samples)

    segments = extract_segments(q0, q5, args.segment_threshold_mm, args.segment_min_duration_sec, args.segment_merge_gap_sec, args.segment_sample_dt_sec)
    segment_rows = summarize_segments(q0, q5, segments)
    local_rows = [
        summarize_local_path(q0.label, q0.local_path_snapshots),
        summarize_local_path(q5.label, q5.local_path_snapshots),
    ]
    segment_local_rows = summarize_local_path_segments(segment_rows, q0.local_path_snapshots, q5.local_path_snapshots)

    plot_segment_amplitude(out_dir / "test2_segment_amplitude.png", q0, q5, segments, args.visual_proxy_label)
    plot_local_path_smoothness(out_dir / "test2_local_path_smoothness.png", q0, q5)
    plot_local_path_topdown(out_dir / "test2_local_path_topdown.png", q0, q5, segments)
    plot_odom_topdown(out_dir / "test2_odom_topdown.png", q0, q5, segments)
    plot_odom_kinematics(out_dir / "test2_odom_kinematics.png", q0, q5, segments)

    with (out_dir / "segment_amplitude_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(SegmentRow.__annotations__.keys()))
        writer.writeheader()
        for row in segment_rows:
            writer.writerow(asdict(row))

    with (out_dir / "local_path_smoothness_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(LocalPathSummaryRow.__annotations__.keys()))
        writer.writeheader()
        for row in local_rows:
            writer.writerow(asdict(row))

    if segment_local_rows:
        with (out_dir / "segment_local_path_smoothness.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(segment_local_rows[0].keys()))
            writer.writeheader()
            writer.writerows(segment_local_rows)

    with (out_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "q0_label": q0.label,
                "q5_label": q5.label,
                "segments": [asdict(row) for row in segment_rows],
                "local_path": [asdict(row) for row in local_rows],
                "segment_local_path": segment_local_rows,
            },
            handle,
            ensure_ascii=False,
            indent=2,
        )

    build_readme(out_dir / "README.md", segment_rows, local_rows, segment_local_rows, args.visual_proxy_label)

    print(f"[OK] out dir: {out_dir}")
    print(f"[OK] segment csv: {out_dir / 'segment_amplitude_summary.csv'}")
    print(f"[OK] local-path csv: {out_dir / 'local_path_smoothness_summary.csv'}")
    print(f"[OK] segment plot: {out_dir / 'test2_segment_amplitude.png'}")
    print(f"[OK] local-path plot: {out_dir / 'test2_local_path_smoothness.png'}")
    print(f"[OK] topdown plot: {out_dir / 'test2_local_path_topdown.png'}")
    print(f"[OK] odom plot: {out_dir / 'test2_odom_topdown.png'}")
    print(f"[OK] odom kinematics plot: {out_dir / 'test2_odom_kinematics.png'}")
    for row in segment_rows:
        print(
            f"[INFO] {row.segment_id}: "
            f"Q5/Q0 slosh p90 ratio={row.q5_over_q0_slosh_p90_ratio:.6f}, "
            f"Q5/Q0 slosh area ratio={row.q5_over_q0_slosh_area_ratio:.6f}, "
            f"Q5/Q0 {args.visual_proxy_label} p90 ratio={row.q5_over_q0_center_p90_ratio:.6f}"
        )


if __name__ == "__main__":
    main()
