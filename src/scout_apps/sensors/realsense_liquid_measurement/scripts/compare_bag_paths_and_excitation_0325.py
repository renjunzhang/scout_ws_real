#!/usr/bin/env python3
"""Compare odom path, motion excitation, and slosh response across 0325 bags."""

import argparse
import csv
import itertools
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rosbag

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:  # pylint: disable=broad-except
    plt = None


DEFAULT_BAGS = {
    "Q0_test1": "slosh_QQ0_s_test1_20260325_203739.bag",
    "Q5_test1": "slosh_QQ5_sttest1_20260325_204117.bag",
    "Q5_test2": "slosh_QQ5_test2_20260325_204526.bag",
    "Q5_test3": "slosh_QQ5_test3_20260325_204626.bag",
}


@dataclass
class BagSeries:
    label: str
    bag_path: Path
    duration_sec: float
    odom_t: np.ndarray
    odom_x_local_m: np.ndarray
    odom_y_local_m: np.ndarray
    odom_yaw_local_rad: np.ndarray
    cmd_t: np.ndarray
    cmd_v_mps: np.ndarray
    cmd_omega_radps: np.ndarray
    ax_cmd_mps2: np.ndarray
    ay_model_mps2: np.ndarray
    slosh_height_t: np.ndarray
    slosh_height_mm_zero: np.ndarray
    slosh_ax_t: np.ndarray
    slosh_ax_est_mps2: np.ndarray
    slosh_ay_t: np.ndarray
    slosh_ay_est_mps2: np.ndarray
    local_path_count: int
    global_path_smooth_count: int


@dataclass
class BagMetrics:
    label: str
    bag_path: str
    duration_sec: float
    odom_points: int
    path_length_m: float
    start_end_disp_m: float
    total_heading_change_deg: float
    cmd_samples: int
    speed_abs_mean_mps: float
    speed_abs_p90_mps: float
    speed_abs_max_mps: float
    omega_abs_mean_radps: float
    omega_abs_p90_radps: float
    omega_abs_max_radps: float
    stop_ratio: float
    ax_cmd_abs_p90_mps2: float
    ax_cmd_abs_max_mps2: float
    ay_model_abs_p90_mps2: float
    ay_model_abs_max_mps2: float
    ay_model_abs_integral: float
    slosh_height_abs_p90_mm: float
    slosh_height_abs_max_mm: float
    slosh_ax_abs_p90_mps2: float
    slosh_ax_abs_max_mps2: float
    slosh_ay_abs_p90_mps2: float
    slosh_ay_abs_max_mps2: float
    local_path_count: int
    global_path_smooth_count: int
    realsense_center_abs_p90_mm: float
    realsense_center_abs_max_mm: float
    realsense_peak_abs_p90_mm: float
    realsense_peak_abs_max_mm: float
    realsense_reportable_frames: int
    mae_center_vs_slosh_mm: float


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Compare actual odom path, command excitation, and slosh response across 0325 bags "
            "to determine whether Q0 and Q5 experiments are truly comparable."
        )
    )
    parser.add_argument(
        "--bag-root",
        default="/data/a/slosh_bags/0325",
        help="Directory containing the 0325 bag files.",
    )
    parser.add_argument(
        "--verify-root",
        default="/data/a/realsense_validation_v2/verify/0325",
        help="Directory containing liquid_height_v2.csv and aligned compare outputs. Empty disables visual stats.",
    )
    parser.add_argument(
        "--out-dir",
        default="/data/a/realsense_validation_v2/verify/0325/path_compare",
        help="Output directory for metrics, figures, and README.",
    )
    parser.add_argument(
        "--bag",
        action="append",
        default=[],
        help="Optional labeled bag override in the form label=filename_or_path. Repeatable.",
    )
    parser.add_argument(
        "--path-shape-threshold-m",
        type=float,
        default=0.35,
        help="Heuristic threshold for mean aligned odom path distance.",
    )
    parser.add_argument(
        "--path-length-ratio-threshold",
        type=float,
        default=1.20,
        help="Heuristic threshold for longer_path/shorter_path.",
    )
    parser.add_argument(
        "--duration-ratio-threshold",
        type=float,
        default=1.30,
        help="Heuristic threshold for longer_duration/shorter_duration.",
    )
    parser.add_argument(
        "--speed-p90-ratio-threshold",
        type=float,
        default=1.25,
        help="Heuristic threshold for speed p90 ratio.",
    )
    parser.add_argument(
        "--omega-p90-ratio-threshold",
        type=float,
        default=1.35,
        help="Heuristic threshold for omega p90 ratio.",
    )
    parser.add_argument(
        "--ay-p90-ratio-threshold",
        type=float,
        default=1.35,
        help="Heuristic threshold for ay_model p90 ratio.",
    )
    parser.add_argument(
        "--stop-ratio-diff-threshold",
        type=float,
        default=0.15,
        help="Heuristic threshold for stop-ratio absolute difference.",
    )
    parser.add_argument(
        "--zero-align-window-sec",
        type=float,
        default=1.0,
        help="Initial window used to zero-align /slosh/height before amplitude comparison.",
    )
    parser.add_argument(
        "--zero-align-max-samples",
        type=int,
        default=30,
        help="Maximum early samples used for /slosh/height zero alignment.",
    )
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def parse_labeled_bags(args) -> Dict[str, Path]:
    bag_root = Path(args.bag_root).expanduser().resolve()
    bag_map: Dict[str, Path] = {}
    if args.bag:
        for raw in args.bag:
            if "=" not in raw:
                raise ValueError(f"--bag must use label=path format: {raw}")
            label, raw_path = raw.split("=", 1)
            label = label.strip()
            raw_path = raw_path.strip()
            path = Path(raw_path).expanduser()
            if not path.is_absolute():
                path = (bag_root / path).resolve()
            else:
                path = path.resolve()
            bag_map[label] = path
        return bag_map
    for label, filename in DEFAULT_BAGS.items():
        bag_map[label] = (bag_root / filename).resolve()
    return bag_map


def finite_float(raw_value) -> float:
    if raw_value is None:
        return math.nan
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def wrapped_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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
        if not math.isfinite(float(times[idx])) or not math.isfinite(float(values[idx])):
            continue
        left_idx = nearest_finite_index(values, idx - 1, -1)
        right_idx = nearest_finite_index(values, idx + 1, +1)
        if left_idx is not None and right_idx is not None:
            dt = float(times[right_idx] - times[left_idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[right_idx] - values[left_idx]) / dt
                continue
        if right_idx is not None:
            dt = float(times[right_idx] - times[idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[right_idx] - values[idx]) / dt
                continue
        if left_idx is not None:
            dt = float(times[idx] - times[left_idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[idx] - values[left_idx]) / dt
                continue
    return derivatives


def zero_align_offset(times: np.ndarray, values: np.ndarray, window_sec: float, max_samples: int) -> float:
    pairs = [(float(t), float(v)) for t, v in zip(times, values) if math.isfinite(float(v)) and math.isfinite(float(t))]
    if not pairs:
        return 0.0
    selected = [v for t, v in pairs if t <= float(window_sec)]
    if len(selected) < 3:
        selected = [v for _, v in pairs[: max(1, int(max_samples))]]
    else:
        selected = selected[: max(1, int(max_samples))]
    if not selected:
        return 0.0
    return float(statistics.median(selected))


def localize_xy(x_values: np.ndarray, y_values: np.ndarray, yaw_values: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if x_values.size == 0:
        return x_values.copy(), y_values.copy(), yaw_values.copy()
    x0 = float(x_values[0])
    y0 = float(y_values[0])
    yaw0 = float(yaw_values[0]) if yaw_values.size > 0 and math.isfinite(float(yaw_values[0])) else 0.0
    cos_yaw = math.cos(-yaw0)
    sin_yaw = math.sin(-yaw0)
    dx = x_values - x0
    dy = y_values - y0
    x_local = cos_yaw * dx - sin_yaw * dy
    y_local = sin_yaw * dx + cos_yaw * dy
    yaw_local = np.asarray([wrapped_angle(float(yaw) - yaw0) for yaw in yaw_values], dtype=np.float64)
    return x_local, y_local, yaw_local


def polyline_length(x_values: np.ndarray, y_values: np.ndarray) -> float:
    if x_values.size < 2:
        return 0.0
    dx = np.diff(x_values)
    dy = np.diff(y_values)
    return float(np.sum(np.hypot(dx, dy)))


def total_heading_change_deg(yaw_values: np.ndarray) -> float:
    if yaw_values.size < 2:
        return 0.0
    diffs = [abs(wrapped_angle(float(yaw_values[idx]) - float(yaw_values[idx - 1]))) for idx in range(1, yaw_values.size)]
    return float(np.degrees(np.sum(diffs)))


def percentile90(values: Sequence[float]) -> float:
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return math.nan
    idx = int(round((len(clean) - 1) * 0.9))
    return float(clean[idx])


def mean_abs(values: Sequence[float]) -> float:
    clean = [abs(float(v)) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(np.mean(clean))


def max_abs(values: Sequence[float]) -> float:
    clean = [abs(float(v)) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(max(clean))


def trapz_abs(times: np.ndarray, values: np.ndarray) -> float:
    if times.size < 2 or values.size < 2:
        return 0.0
    mask = np.isfinite(times) & np.isfinite(values)
    if np.count_nonzero(mask) < 2:
        return 0.0
    t = times[mask]
    v = np.abs(values[mask])
    order = np.argsort(t)
    t = t[order]
    v = v[order]
    return float(np.trapz(v, t))


def safe_ratio(a: float, b: float) -> float:
    a = float(a)
    b = float(b)
    if not math.isfinite(a) or not math.isfinite(b):
        return math.nan
    lo = min(abs(a), abs(b))
    hi = max(abs(a), abs(b))
    if lo <= 1e-9:
        return 1.0 if hi <= 1e-9 else math.inf
    return hi / lo


def resample_polyline(x_values: np.ndarray, y_values: np.ndarray, num_points: int = 200) -> np.ndarray:
    if x_values.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    if x_values.size == 1:
        return np.repeat(np.asarray([[float(x_values[0]), float(y_values[0])]], dtype=np.float64), num_points, axis=0)
    seg = np.hypot(np.diff(x_values), np.diff(y_values))
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total = float(s[-1])
    if total <= 1e-9:
        return np.repeat(np.asarray([[float(x_values[0]), float(y_values[0])]], dtype=np.float64), num_points, axis=0)
    targets = np.linspace(0.0, total, num_points)
    x_interp = np.interp(targets, s, x_values)
    y_interp = np.interp(targets, s, y_values)
    return np.column_stack([x_interp, y_interp])


def load_visual_compare_metrics(verify_root: Path, label: str) -> Dict[str, float]:
    out = {
        "realsense_center_abs_p90_mm": math.nan,
        "realsense_center_abs_max_mm": math.nan,
        "realsense_peak_abs_p90_mm": math.nan,
        "realsense_peak_abs_max_mm": math.nan,
        "realsense_reportable_frames": math.nan,
        "mae_center_vs_slosh_mm": math.nan,
    }
    aligned_csv = verify_root / label / "mpc_realsense_aligned_v2.csv"
    if not aligned_csv.exists():
        return out
    rows = list(csv.DictReader(aligned_csv.open("r", newline="", encoding="utf-8")))
    if not rows:
        return out
    center_abs = []
    peak_abs = []
    err_center = []
    for row in rows:
        center = finite_float(row.get("realsense_center_main_rel_mm_v2_zero_aligned"))
        peak = finite_float(row.get("realsense_peak_rel_mm_v2_zero_aligned"))
        slosh = finite_float(row.get("mpc_slosh_height_mm_zero_aligned"))
        if math.isfinite(center):
            center_abs.append(abs(center))
        if math.isfinite(peak):
            peak_abs.append(abs(peak))
        if math.isfinite(center) and math.isfinite(slosh):
            err_center.append(abs(center - slosh))
    out["realsense_center_abs_p90_mm"] = percentile90(center_abs)
    out["realsense_center_abs_max_mm"] = max(center_abs) if center_abs else math.nan
    out["realsense_peak_abs_p90_mm"] = percentile90(peak_abs)
    out["realsense_peak_abs_max_mm"] = max(peak_abs) if peak_abs else math.nan
    out["realsense_reportable_frames"] = float(len(rows))
    out["mae_center_vs_slosh_mm"] = float(np.mean(err_center)) if err_center else math.nan
    return out


def load_bag_series(label: str, bag_path: Path, args) -> BagSeries:
    topics = [
        "/odom",
        "/cmd_vel",
        "/local_path",
        "/scout/global_path_smooth",
        "/slosh/height",
        "/slosh/ax_est",
        "/slosh/ay_est",
    ]
    odom_t: List[float] = []
    odom_x: List[float] = []
    odom_y: List[float] = []
    odom_yaw: List[float] = []
    cmd_t: List[float] = []
    cmd_v: List[float] = []
    cmd_omega: List[float] = []
    slosh_height_t: List[float] = []
    slosh_height_mm: List[float] = []
    slosh_ax_t: List[float] = []
    slosh_ax: List[float] = []
    slosh_ay_t: List[float] = []
    slosh_ay: List[float] = []
    local_path_count = 0
    global_path_smooth_count = 0

    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = float(stamp.to_sec() - bag_start)
            if topic == "/odom":
                odom_t.append(ts)
                odom_x.append(float(msg.pose.pose.position.x))
                odom_y.append(float(msg.pose.pose.position.y))
                odom_yaw.append(quaternion_to_yaw(msg.pose.pose.orientation))
            elif topic == "/cmd_vel":
                cmd_t.append(ts)
                cmd_v.append(float(msg.linear.x))
                cmd_omega.append(float(msg.angular.z))
            elif topic == "/slosh/height":
                slosh_height_t.append(ts)
                slosh_height_mm.append(float(msg.data) * 1000.0)
            elif topic == "/slosh/ax_est":
                slosh_ax_t.append(ts)
                slosh_ax.append(float(msg.data))
            elif topic == "/slosh/ay_est":
                slosh_ay_t.append(ts)
                slosh_ay.append(float(msg.data))
            elif topic == "/local_path":
                local_path_count += 1
            elif topic == "/scout/global_path_smooth":
                global_path_smooth_count += 1

    odom_t_np = np.asarray(odom_t, dtype=np.float64)
    odom_x_np = np.asarray(odom_x, dtype=np.float64)
    odom_y_np = np.asarray(odom_y, dtype=np.float64)
    odom_yaw_np = np.asarray(odom_yaw, dtype=np.float64)
    odom_x_local, odom_y_local, odom_yaw_local = localize_xy(odom_x_np, odom_y_np, odom_yaw_np)

    cmd_t_np = np.asarray(cmd_t, dtype=np.float64)
    cmd_v_np = np.asarray(cmd_v, dtype=np.float64)
    cmd_omega_np = np.asarray(cmd_omega, dtype=np.float64)
    ax_cmd_np = finite_difference(cmd_v_np, cmd_t_np)
    ay_model_np = np.full(cmd_v_np.shape, np.nan, dtype=np.float64)
    mask = np.isfinite(cmd_v_np) & np.isfinite(cmd_omega_np)
    ay_model_np[mask] = cmd_v_np[mask] * cmd_omega_np[mask]

    slosh_height_t_np = np.asarray(slosh_height_t, dtype=np.float64)
    slosh_height_mm_np = np.asarray(slosh_height_mm, dtype=np.float64)
    slosh_offset = zero_align_offset(slosh_height_t_np, slosh_height_mm_np, args.zero_align_window_sec, args.zero_align_max_samples)
    slosh_height_zero_np = np.asarray(
        [math.nan if not math.isfinite(v) else float(v - slosh_offset) for v in slosh_height_mm_np],
        dtype=np.float64,
    )

    return BagSeries(
        label=label,
        bag_path=bag_path,
        duration_sec=float(bag_end - bag_start),
        odom_t=odom_t_np,
        odom_x_local_m=odom_x_local,
        odom_y_local_m=odom_y_local,
        odom_yaw_local_rad=odom_yaw_local,
        cmd_t=cmd_t_np,
        cmd_v_mps=cmd_v_np,
        cmd_omega_radps=cmd_omega_np,
        ax_cmd_mps2=ax_cmd_np,
        ay_model_mps2=ay_model_np,
        slosh_height_t=slosh_height_t_np,
        slosh_height_mm_zero=slosh_height_zero_np,
        slosh_ax_t=np.asarray(slosh_ax_t, dtype=np.float64),
        slosh_ax_est_mps2=np.asarray(slosh_ax, dtype=np.float64),
        slosh_ay_t=np.asarray(slosh_ay_t, dtype=np.float64),
        slosh_ay_est_mps2=np.asarray(slosh_ay, dtype=np.float64),
        local_path_count=local_path_count,
        global_path_smooth_count=global_path_smooth_count,
    )


def compute_metrics(series: BagSeries, verify_root: Optional[Path]) -> BagMetrics:
    speed_abs = [abs(v) for v in series.cmd_v_mps if math.isfinite(float(v))]
    omega_abs = [abs(v) for v in series.cmd_omega_radps if math.isfinite(float(v))]
    ax_abs = [abs(v) for v in series.ax_cmd_mps2 if math.isfinite(float(v))]
    ay_abs = [abs(v) for v in series.ay_model_mps2 if math.isfinite(float(v))]
    slosh_height_abs = [abs(v) for v in series.slosh_height_mm_zero if math.isfinite(float(v))]
    slosh_ax_abs = [abs(v) for v in series.slosh_ax_est_mps2 if math.isfinite(float(v))]
    slosh_ay_abs = [abs(v) for v in series.slosh_ay_est_mps2 if math.isfinite(float(v))]
    stop_mask = [
        abs(float(v)) < 0.02 and abs(float(w)) < 0.05
        for v, w in zip(series.cmd_v_mps, series.cmd_omega_radps)
        if math.isfinite(float(v)) and math.isfinite(float(w))
    ]
    visual = load_visual_compare_metrics(verify_root, series.label) if verify_root is not None else {}
    return BagMetrics(
        label=series.label,
        bag_path=str(series.bag_path),
        duration_sec=series.duration_sec,
        odom_points=int(series.odom_t.size),
        path_length_m=polyline_length(series.odom_x_local_m, series.odom_y_local_m),
        start_end_disp_m=0.0
        if series.odom_x_local_m.size == 0
        else float(math.hypot(float(series.odom_x_local_m[-1]), float(series.odom_y_local_m[-1]))),
        total_heading_change_deg=total_heading_change_deg(series.odom_yaw_local_rad),
        cmd_samples=int(series.cmd_t.size),
        speed_abs_mean_mps=float(np.mean(speed_abs)) if speed_abs else math.nan,
        speed_abs_p90_mps=percentile90(speed_abs),
        speed_abs_max_mps=max(speed_abs) if speed_abs else math.nan,
        omega_abs_mean_radps=float(np.mean(omega_abs)) if omega_abs else math.nan,
        omega_abs_p90_radps=percentile90(omega_abs),
        omega_abs_max_radps=max(omega_abs) if omega_abs else math.nan,
        stop_ratio=float(np.mean(stop_mask)) if stop_mask else math.nan,
        ax_cmd_abs_p90_mps2=percentile90(ax_abs),
        ax_cmd_abs_max_mps2=max(ax_abs) if ax_abs else math.nan,
        ay_model_abs_p90_mps2=percentile90(ay_abs),
        ay_model_abs_max_mps2=max(ay_abs) if ay_abs else math.nan,
        ay_model_abs_integral=trapz_abs(series.cmd_t, series.ay_model_mps2),
        slosh_height_abs_p90_mm=percentile90(slosh_height_abs),
        slosh_height_abs_max_mm=max(slosh_height_abs) if slosh_height_abs else math.nan,
        slosh_ax_abs_p90_mps2=percentile90(slosh_ax_abs),
        slosh_ax_abs_max_mps2=max(slosh_ax_abs) if slosh_ax_abs else math.nan,
        slosh_ay_abs_p90_mps2=percentile90(slosh_ay_abs),
        slosh_ay_abs_max_mps2=max(slosh_ay_abs) if slosh_ay_abs else math.nan,
        local_path_count=series.local_path_count,
        global_path_smooth_count=series.global_path_smooth_count,
        realsense_center_abs_p90_mm=float(visual.get("realsense_center_abs_p90_mm", math.nan)),
        realsense_center_abs_max_mm=float(visual.get("realsense_center_abs_max_mm", math.nan)),
        realsense_peak_abs_p90_mm=float(visual.get("realsense_peak_abs_p90_mm", math.nan)),
        realsense_peak_abs_max_mm=float(visual.get("realsense_peak_abs_max_mm", math.nan)),
        realsense_reportable_frames=float(visual.get("realsense_reportable_frames", math.nan)),
        mae_center_vs_slosh_mm=float(visual.get("mae_center_vs_slosh_mm", math.nan)),
    )


def compare_paths(series_a: BagSeries, series_b: BagSeries) -> Tuple[float, float]:
    resampled_a = resample_polyline(series_a.odom_x_local_m, series_a.odom_y_local_m)
    resampled_b = resample_polyline(series_b.odom_x_local_m, series_b.odom_y_local_m)
    if resampled_a.size == 0 or resampled_b.size == 0:
        return math.nan, math.nan
    distances = np.hypot(resampled_a[:, 0] - resampled_b[:, 0], resampled_a[:, 1] - resampled_b[:, 1])
    return float(np.mean(distances)), float(np.max(distances))


def pairwise_metrics(metrics_a: BagMetrics, metrics_b: BagMetrics, series_a: BagSeries, series_b: BagSeries, args) -> Dict[str, object]:
    path_mean_dist_m, path_max_dist_m = compare_paths(series_a, series_b)
    duration_ratio = safe_ratio(metrics_a.duration_sec, metrics_b.duration_sec)
    path_length_ratio = safe_ratio(metrics_a.path_length_m, metrics_b.path_length_m)
    speed_p90_ratio = safe_ratio(metrics_a.speed_abs_p90_mps, metrics_b.speed_abs_p90_mps)
    omega_p90_ratio = safe_ratio(metrics_a.omega_abs_p90_radps, metrics_b.omega_abs_p90_radps)
    ay_p90_ratio = safe_ratio(metrics_a.ay_model_abs_p90_mps2, metrics_b.ay_model_abs_p90_mps2)
    stop_ratio_diff = (
        abs(metrics_a.stop_ratio - metrics_b.stop_ratio)
        if math.isfinite(metrics_a.stop_ratio) and math.isfinite(metrics_b.stop_ratio)
        else math.nan
    )

    reasons = []
    if not math.isfinite(path_mean_dist_m) or path_mean_dist_m > args.path_shape_threshold_m:
        reasons.append("path_shape")
    if not math.isfinite(path_length_ratio) or path_length_ratio > args.path_length_ratio_threshold:
        reasons.append("path_length")
    if not math.isfinite(duration_ratio) or duration_ratio > args.duration_ratio_threshold:
        reasons.append("duration")
    if not math.isfinite(speed_p90_ratio) or speed_p90_ratio > args.speed_p90_ratio_threshold:
        reasons.append("speed_p90")
    if not math.isfinite(omega_p90_ratio) or omega_p90_ratio > args.omega_p90_ratio_threshold:
        reasons.append("omega_p90")
    if not math.isfinite(ay_p90_ratio) or ay_p90_ratio > args.ay_p90_ratio_threshold:
        reasons.append("ay_p90")
    if not math.isfinite(stop_ratio_diff) or stop_ratio_diff > args.stop_ratio_diff_threshold:
        reasons.append("stop_ratio")

    comparable = len(reasons) == 0
    q5_stabler = (
        metrics_b.slosh_height_abs_p90_mm < metrics_a.slosh_height_abs_p90_mm
        if math.isfinite(metrics_a.slosh_height_abs_p90_mm) and math.isfinite(metrics_b.slosh_height_abs_p90_mm)
        else False
    )

    return {
        "reference_label": metrics_a.label,
        "candidate_label": metrics_b.label,
        "duration_ratio": duration_ratio,
        "path_length_ratio": path_length_ratio,
        "path_shape_mean_distance_m": path_mean_dist_m,
        "path_shape_max_distance_m": path_max_dist_m,
        "speed_p90_ratio": speed_p90_ratio,
        "omega_p90_ratio": omega_p90_ratio,
        "ay_p90_ratio": ay_p90_ratio,
        "stop_ratio_diff": stop_ratio_diff,
        "reference_slosh_abs_p90_mm": metrics_a.slosh_height_abs_p90_mm,
        "candidate_slosh_abs_p90_mm": metrics_b.slosh_height_abs_p90_mm,
        "reference_realsense_center_abs_p90_mm": metrics_a.realsense_center_abs_p90_mm,
        "candidate_realsense_center_abs_p90_mm": metrics_b.realsense_center_abs_p90_mm,
        "comparable_overall": int(comparable),
        "reasons_failed": ",".join(reasons),
        "candidate_lower_slosh_p90": int(q5_stabler),
    }


def write_metrics_csv(path: Path, metrics_list: Sequence[BagMetrics]):
    fieldnames = list(BagMetrics.__dataclass_fields__.keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for metrics in metrics_list:
            writer.writerow(metrics.__dict__)


def write_pairwise_csv(path: Path, rows: Sequence[Dict[str, object]]):
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def plot_odom_overlay(path: Path, series_map: Dict[str, BagSeries]) -> bool:
    if plt is None:
        return False
    colors = {
        "Q0_test1": "#1f77b4",
        "Q5_test1": "#d62728",
        "Q5_test2": "#2ca02c",
        "Q5_test3": "#9467bd",
    }
    fig, ax = plt.subplots(1, 1, figsize=(8.5, 7.0))
    for label, series in series_map.items():
        if series.odom_x_local_m.size == 0:
            continue
        color = colors.get(label, None)
        ax.plot(series.odom_x_local_m, series.odom_y_local_m, linewidth=1.6, label=label, color=color)
        ax.scatter([series.odom_x_local_m[0]], [series.odom_y_local_m[0]], color=color, s=32, marker="o")
        ax.scatter([series.odom_x_local_m[-1]], [series.odom_y_local_m[-1]], color=color, s=40, marker="x")
    ax.set_title("Aligned Odom Path Overlay (start pose normalized)")
    ax.set_xlabel("x_local (m)")
    ax.set_ylabel("y_local (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_cmd_vel_compare(path: Path, series_map: Dict[str, BagSeries]) -> bool:
    if plt is None:
        return False
    fig, (ax_v, ax_w) = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
    for label, series in series_map.items():
        ax_v.plot(series.cmd_t, series.cmd_v_mps, linewidth=1.2, label=label)
        ax_w.plot(series.cmd_t, series.cmd_omega_radps, linewidth=1.2, label=label)
    ax_v.set_title("Command Velocity Comparison")
    ax_v.set_ylabel("v (m/s)")
    ax_v.grid(True, alpha=0.25)
    ax_v.legend(loc="upper right")
    ax_w.set_ylabel("omega (rad/s)")
    ax_w.set_xlabel("relative time (s)")
    ax_w.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_excitation_compare(path: Path, series_map: Dict[str, BagSeries]) -> bool:
    if plt is None:
        return False
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=False)
    ax_cmd_ax, ax_cmd_ay, ax_slosh_ax, ax_slosh_ay = axes
    for label, series in series_map.items():
        ax_cmd_ax.plot(series.cmd_t, series.ax_cmd_mps2, linewidth=1.0, label=label)
        ax_cmd_ay.plot(series.cmd_t, series.ay_model_mps2, linewidth=1.0, label=label)
        ax_slosh_ax.plot(series.slosh_ax_t, series.slosh_ax_est_mps2, linewidth=1.0, label=label)
        ax_slosh_ay.plot(series.slosh_ay_t, series.slosh_ay_est_mps2, linewidth=1.0, label=label)
    ax_cmd_ax.set_title("Excitation Comparison")
    ax_cmd_ax.set_ylabel("ax_cmd (m/s^2)")
    ax_cmd_ay.set_ylabel("ay_model (m/s^2)")
    ax_slosh_ax.set_ylabel("/slosh/ax_est")
    ax_slosh_ay.set_ylabel("/slosh/ay_est")
    ax_slosh_ay.set_xlabel("relative time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    ax_cmd_ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_slosh_vs_excitation(path: Path, metrics_list: Sequence[BagMetrics]) -> bool:
    if plt is None:
        return False
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.2))
    for metrics in metrics_list:
        ax1.scatter(metrics.ay_model_abs_p90_mps2, metrics.slosh_height_abs_p90_mm, s=64)
        ax1.annotate(metrics.label, (metrics.ay_model_abs_p90_mps2, metrics.slosh_height_abs_p90_mm), xytext=(5, 5), textcoords="offset points")
        ax2.scatter(metrics.slosh_height_abs_p90_mm, metrics.realsense_center_abs_p90_mm, s=64)
        ax2.annotate(metrics.label, (metrics.slosh_height_abs_p90_mm, metrics.realsense_center_abs_p90_mm), xytext=(5, 5), textcoords="offset points")
    ax1.set_title("Excitation vs /slosh/height amplitude")
    ax1.set_xlabel("|ay_model| p90 (m/s^2)")
    ax1.set_ylabel("|/slosh/height| p90 (mm)")
    ax1.grid(True, alpha=0.25)
    ax2.set_title("/slosh/height vs RealSense amplitude")
    ax2.set_xlabel("|/slosh/height| p90 (mm)")
    ax2.set_ylabel("|RS main center| p90 (mm)")
    ax2.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def write_readme(path: Path, metrics_list: Sequence[BagMetrics], pairwise_rows: Sequence[Dict[str, object]], out_dir: Path):
    by_label = {metrics.label: metrics for metrics in metrics_list}
    lines: List[str] = []
    lines.append("# 0325 路径与激励对比")
    lines.append("")
    lines.append("## 结论")
    lines.append("")
    lines.append("1. 这份分析先判断 bag 是否同任务可比，再决定能不能解释成抑制效果。")
    lines.append("2. 几何路径不是唯一标准，真正决定液体响应的是 `cmd_vel` 与平面激励。")
    lines.append("3. `comparable_overall` 只是第一轮筛查结论，不是最终物理真值。")
    lines.append("")
    lines.append("## 单 bag 指标")
    lines.append("")
    lines.append("| bag | duration | path_length | speed_p90 | omega_p90 | |ay_model| p90 | |/slosh/height| p90 | |RS main center| p90 |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for metrics in metrics_list:
        lines.append(
            f"| `{metrics.label}` | {metrics.duration_sec:.1f}s | {metrics.path_length_m:.3f}m | "
            f"{metrics.speed_abs_p90_mps:.3f} | {metrics.omega_abs_p90_radps:.3f} | "
            f"{metrics.ay_model_abs_p90_mps2:.3f} | {metrics.slosh_height_abs_p90_mm:.3f}mm | "
            f"{metrics.realsense_center_abs_p90_mm:.3f}mm |"
        )
    lines.append("")
    lines.append("## Pairwise 判断")
    lines.append("")
    lines.append("| ref | cand | comparable | failed | duration_ratio | path_ratio | path_mean_dist | speed_p90_ratio | omega_p90_ratio | ay_p90_ratio |")
    lines.append("| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for row in pairwise_rows:
        failed = row["reasons_failed"] if row["reasons_failed"] else "-"
        lines.append(
            f"| `{row['reference_label']}` | `{row['candidate_label']}` | {int(row['comparable_overall'])} | "
            f"{failed} | {float(row['duration_ratio']):.3f} | {float(row['path_length_ratio']):.3f} | "
            f"{float(row['path_shape_mean_distance_m']):.3f}m | {float(row['speed_p90_ratio']):.3f} | "
            f"{float(row['omega_p90_ratio']):.3f} | {float(row['ay_p90_ratio']):.3f} |"
        )
    lines.append("")
    lines.append("## 解释")
    lines.append("")
    lines.append("- `duration_ratio` 和 `path_length_ratio` 反映任务时长与路程是否接近。")
    lines.append("- `path_shape_mean_distance_m` 反映归一化起点后的 odom 路径形状差异。")
    lines.append("- `speed_p90_ratio / omega_p90_ratio / ay_p90_ratio` 反映运动激励是否同量级。")
    lines.append("- 如果 `comparable_overall = 0`，则不建议直接把液体差异解释成 Q 参数效果。")
    lines.append("")
    lines.append("## 先看 Q0 对 Q5")
    lines.append("")
    for row in pairwise_rows:
        if row["reference_label"] != "Q0_test1":
            continue
        candidate = str(row["candidate_label"])
        metrics = by_label.get(candidate)
        if metrics is None:
            continue
        comparable = int(row["comparable_overall"]) == 1
        failed = row["reasons_failed"] if row["reasons_failed"] else "none"
        lines.append(
            f"- `Q0_test1 vs {candidate}`: comparable={int(comparable)}, failed={failed}, "
            f"`|/slosh/height| p90={metrics.slosh_height_abs_p90_mm:.3f} mm`, "
            f"`|RS main center| p90={metrics.realsense_center_abs_p90_mm:.3f} mm`"
        )
    lines.append("")
    lines.append("## 文件")
    lines.append("")
    lines.append(f"- [bag_metrics.csv]({(out_dir / 'bag_metrics.csv').name})")
    lines.append(f"- [pairwise_compare.csv]({(out_dir / 'pairwise_compare.csv').name})")
    lines.append(f"- [odom_overlay.png]({(out_dir / 'odom_overlay.png').name})")
    lines.append(f"- [cmd_vel_compare.png]({(out_dir / 'cmd_vel_compare.png').name})")
    lines.append(f"- [excitation_compare.png]({(out_dir / 'excitation_compare.png').name})")
    lines.append(f"- [slosh_vs_excitation_compare.png]({(out_dir / 'slosh_vs_excitation_compare.png').name})")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    args = parse_args()
    bag_map = parse_labeled_bags(args)
    out_dir = Path(args.out_dir).expanduser().resolve()
    verify_root = Path(args.verify_root).expanduser().resolve() if args.verify_root else None
    ensure_dir(out_dir)

    series_map: Dict[str, BagSeries] = {}
    metrics_list: List[BagMetrics] = []
    for label, bag_path in bag_map.items():
        if not bag_path.exists():
            raise FileNotFoundError(str(bag_path))
        series = load_bag_series(label, bag_path, args)
        series_map[label] = series
        metrics_list.append(compute_metrics(series, verify_root if verify_root and verify_root.exists() else None))

    metrics_list.sort(key=lambda item: item.label)
    metrics_by_label = {metrics.label: metrics for metrics in metrics_list}

    pairwise_rows: List[Dict[str, object]] = []
    reference_label = "Q0_test1"
    if reference_label in metrics_by_label:
        for candidate_label in ("Q5_test1", "Q5_test2", "Q5_test3"):
            if candidate_label in metrics_by_label:
                pairwise_rows.append(
                    pairwise_metrics(
                        metrics_by_label[reference_label],
                        metrics_by_label[candidate_label],
                        series_map[reference_label],
                        series_map[candidate_label],
                        args,
                    )
                )
    q5_labels = [label for label in ("Q5_test1", "Q5_test2", "Q5_test3") if label in metrics_by_label]
    for label_a, label_b in itertools.combinations(q5_labels, 2):
        pairwise_rows.append(
            pairwise_metrics(
                metrics_by_label[label_a],
                metrics_by_label[label_b],
                series_map[label_a],
                series_map[label_b],
                args,
            )
        )

    write_metrics_csv(out_dir / "bag_metrics.csv", metrics_list)
    write_pairwise_csv(out_dir / "pairwise_compare.csv", pairwise_rows)
    plot_odom_overlay(out_dir / "odom_overlay.png", series_map)
    plot_cmd_vel_compare(out_dir / "cmd_vel_compare.png", series_map)
    plot_excitation_compare(out_dir / "excitation_compare.png", series_map)
    plot_slosh_vs_excitation(out_dir / "slosh_vs_excitation_compare.png", metrics_list)
    write_readme(out_dir / "README.md", metrics_list, pairwise_rows, out_dir)

    print(f"[OK] out dir: {out_dir}")
    print(f"[OK] bag metrics: {out_dir / 'bag_metrics.csv'}")
    print(f"[OK] pairwise compare: {out_dir / 'pairwise_compare.csv'}")
    print(f"[OK] odom overlay: {out_dir / 'odom_overlay.png'}")
    print(f"[OK] cmd_vel compare: {out_dir / 'cmd_vel_compare.png'}")
    print(f"[OK] excitation compare: {out_dir / 'excitation_compare.png'}")
    print(f"[OK] slosh vs excitation: {out_dir / 'slosh_vs_excitation_compare.png'}")
    print(f"[OK] readme: {out_dir / 'README.md'}")


if __name__ == "__main__":
    main()
