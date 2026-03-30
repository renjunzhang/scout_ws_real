#!/usr/bin/env python3
"""Paired Q0/Q5 comparison for 0330 bags under the previous static reference."""

import argparse
import bisect
import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DEFAULT_BAG_ROOT = Path("/data/a/slosh_bags/0330")
DEFAULT_VERIFY_ROOT = Path("/data/a/realsense_validation_v2/verify/0330_prev_static_ref")
DEFAULT_PAIRS = (("Q0_test1", "Q5_test1"), ("Q0_test2", "Q5_test2"))


@dataclass
class BagSeries:
    label: str
    bag_path: Path
    duration_sec: float
    odom_t: np.ndarray
    odom_x_local_m: np.ndarray
    odom_y_local_m: np.ndarray
    cmd_t: np.ndarray
    cmd_v_mps: np.ndarray
    cmd_omega_radps: np.ndarray
    ax_cmd_mps2: np.ndarray
    ay_model_mps2: np.ndarray
    slosh_height_t: np.ndarray
    slosh_height_mm_zero: np.ndarray


@dataclass
class VisualMetrics:
    label: str
    total_frames: int
    reportable_frames: int
    reportable_ratio: float
    center_abs_p90_mm: float
    center_abs_max_mm: float
    peak_abs_p90_mm: float
    peak_abs_max_mm: float
    center_vs_slosh_mae_mm: float
    center_vs_slosh_rmse_mm: float
    center_vs_slosh_corr: float
    bias_mean_mm: float
    bias_median_mm: float


@dataclass
class VisualSeries:
    label: str
    rel_time_s: List[float]
    center_mm: List[float]
    slosh_mm: List[float]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-root", default=str(DEFAULT_BAG_ROOT))
    parser.add_argument("--verify-root", default=str(DEFAULT_VERIFY_ROOT))
    parser.add_argument(
        "--out-dir",
        default="/data/a/realsense_validation_v2/verify/0330_prev_static_ref/paired_q0_q5_analysis",
    )
    parser.add_argument(
        "--pairs",
        default="Q0_test1:Q5_test1,Q0_test2:Q5_test2",
        help="Comma-separated Q0:Q5 pairs.",
    )
    parser.add_argument(
        "--center-column",
        default="height_center_rel_mm_bias_corrected_v2",
        help="RealSense main liquid level column.",
    )
    parser.add_argument(
        "--zero-align-window-sec",
        type=float,
        default=1.0,
        help="Initial window used to zero-align /slosh/height for amplitude comparison.",
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


def finite_float(raw_value) -> float:
    if raw_value is None:
        return math.nan
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def percentile90(values: Iterable[float]) -> float:
    clean = sorted(abs(float(v)) for v in values if math.isfinite(float(v)))
    if not clean:
        return math.nan
    idx = int(round((len(clean) - 1) * 0.9))
    return float(clean[idx])


def mean_or_nan(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(statistics.mean(clean))


def median_or_nan(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(statistics.median(clean))


def mae(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if not pairs:
        return math.nan
    return float(sum(abs(x - y) for x, y in pairs) / len(pairs))


def rmse(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if not pairs:
        return math.nan
    return float(math.sqrt(sum((x - y) ** 2 for x, y in pairs) / len(pairs)))


def corr(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 2:
        return math.nan
    xs = [x for x, _ in pairs]
    ys = [y for _, y in pairs]
    mean_x = statistics.mean(xs)
    mean_y = statistics.mean(ys)
    den_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs))
    den_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys))
    if den_x <= 0.0 or den_y <= 0.0:
        return math.nan
    num = sum((x - mean_x) * (y - mean_y) for x, y in pairs)
    return float(num / (den_x * den_y))


def safe_ratio(a: float, b: float) -> float:
    a = float(a)
    b = float(b)
    if not math.isfinite(a) or not math.isfinite(b):
        return math.nan
    lo = min(abs(a), abs(b))
    hi = max(abs(a), abs(b))
    if lo <= 1e-9:
        return 1.0 if hi <= 1e-9 else math.inf
    return float(hi / lo)


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def localize_xy(x_values: np.ndarray, y_values: np.ndarray, yaw_values: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if x_values.size == 0:
        return x_values.copy(), y_values.copy()
    x0 = float(x_values[0])
    y0 = float(y_values[0])
    yaw0 = float(yaw_values[0]) if yaw_values.size > 0 and math.isfinite(float(yaw_values[0])) else 0.0
    cos_yaw = math.cos(-yaw0)
    sin_yaw = math.sin(-yaw0)
    dx = x_values - x0
    dy = y_values - y0
    x_local = cos_yaw * dx - sin_yaw * dy
    y_local = sin_yaw * dx + cos_yaw * dy
    return x_local, y_local


def nearest_finite_index(values: np.ndarray, start_idx: int, direction: int):
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


def polyline_length(x_values: np.ndarray, y_values: np.ndarray) -> float:
    if x_values.size < 2:
        return 0.0
    dx = np.diff(x_values)
    dy = np.diff(y_values)
    return float(np.sum(np.hypot(dx, dy)))


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


def compare_paths(series_a: BagSeries, series_b: BagSeries) -> Tuple[float, float]:
    resampled_a = resample_polyline(series_a.odom_x_local_m, series_a.odom_y_local_m)
    resampled_b = resample_polyline(series_b.odom_x_local_m, series_b.odom_y_local_m)
    if resampled_a.size == 0 or resampled_b.size == 0:
        return math.nan, math.nan
    distances = np.hypot(resampled_a[:, 0] - resampled_b[:, 0], resampled_a[:, 1] - resampled_b[:, 1])
    return float(np.mean(distances)), float(np.max(distances))


def interpolate_scalar(series_times: Sequence[float], series_values: Sequence[float], ts: float) -> float:
    if len(series_times) == 0:
        return math.nan
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
    ratio = (ts - t0) / (t1 - t0)
    return float(v0 + ratio * (v1 - v0))


def load_bag_series(label: str, bag_path: Path, args) -> BagSeries:
    odom_t: List[float] = []
    odom_x: List[float] = []
    odom_y: List[float] = []
    odom_yaw: List[float] = []
    cmd_t: List[float] = []
    cmd_v: List[float] = []
    cmd_omega: List[float] = []
    slosh_height_t: List[float] = []
    slosh_height_mm: List[float] = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=["/odom", "/cmd_vel", "/slosh/height"]):
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

    odom_t_np = np.asarray(odom_t, dtype=np.float64)
    odom_x_np = np.asarray(odom_x, dtype=np.float64)
    odom_y_np = np.asarray(odom_y, dtype=np.float64)
    odom_yaw_np = np.asarray(odom_yaw, dtype=np.float64)
    odom_x_local, odom_y_local = localize_xy(odom_x_np, odom_y_np, odom_yaw_np)

    cmd_t_np = np.asarray(cmd_t, dtype=np.float64)
    cmd_v_np = np.asarray(cmd_v, dtype=np.float64)
    cmd_omega_np = np.asarray(cmd_omega, dtype=np.float64)
    ax_cmd_np = finite_difference(cmd_v_np, cmd_t_np)
    ay_model_np = np.asarray([float(v * w) for v, w in zip(cmd_v_np, cmd_omega_np)], dtype=np.float64)

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
        cmd_t=cmd_t_np,
        cmd_v_mps=cmd_v_np,
        cmd_omega_radps=cmd_omega_np,
        ax_cmd_mps2=ax_cmd_np,
        ay_model_mps2=ay_model_np,
        slosh_height_t=slosh_height_t_np,
        slosh_height_mm_zero=slosh_height_zero_np,
    )


def load_visual_metrics(label: str, verify_root: Path, center_column: str, bag_series: BagSeries, args) -> VisualMetrics:
    liquid_csv = verify_root / label / "liquid_height_v2.csv"
    if not liquid_csv.is_file():
        raise RuntimeError(f"missing liquid_height_v2.csv for {label}: {liquid_csv}")

    total_frames = 0
    reportable_frames = 0
    center_values: List[float] = []
    peak_values: List[float] = []
    matched_center: List[float] = []
    matched_slosh: List[float] = []
    signed_center_pairs: List[Tuple[float, float]] = []

    with liquid_csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            total_frames += 1
            center_mm = finite_float(raw.get(center_column))
            peak_mm = finite_float(raw.get("height_peak_rel_mm_v2"))
            center_signed_mm = finite_float(raw.get("height_center_rel_mm_signed_bias_corrected_v2"))
            if math.isfinite(center_mm):
                center_values.append(center_mm)
            if math.isfinite(peak_mm):
                peak_values.append(peak_mm)
            if int(raw.get("accept_for_peak_report_v2", "0") or "0") != 1:
                continue
            stamp = finite_float(raw.get("stamp"))
            rel_time = finite_float(raw.get("relative_time_s"))
            if not math.isfinite(stamp) or not math.isfinite(center_mm):
                continue
            reportable_frames += 1
            matched_center.append(center_mm)
            if math.isfinite(center_signed_mm) and math.isfinite(rel_time):
                signed_center_pairs.append((rel_time, center_signed_mm))
            if bag_series.slosh_height_t.size == 0:
                matched_slosh.append(math.nan)
            else:
                bag_start = stamp - float(raw.get("relative_time_s", 0.0))
                rel_ts = stamp - bag_start
                matched_slosh.append(interpolate_scalar(bag_series.slosh_height_t, bag_series.slosh_height_mm_zero, rel_ts))

    errors = [c - s for c, s in zip(matched_center, matched_slosh) if math.isfinite(c) and math.isfinite(s)]
    offset = 0.0
    if signed_center_pairs:
        early_values = [v for t, v in signed_center_pairs if t <= float(args.zero_align_window_sec)]
        if len(early_values) < 3:
            early_values = [v for _, v in signed_center_pairs[: max(1, int(args.zero_align_max_samples))]]
        if early_values:
            offset = float(statistics.median(early_values))
    center_zero_aligned = [v - offset for _, v in signed_center_pairs]
    return VisualMetrics(
        label=label,
        total_frames=total_frames,
        reportable_frames=reportable_frames,
        reportable_ratio=(reportable_frames / total_frames) if total_frames else math.nan,
        center_abs_p90_mm=percentile90(center_zero_aligned),
        center_abs_max_mm=max((abs(v) for v in center_zero_aligned), default=math.nan),
        peak_abs_p90_mm=percentile90(peak_values),
        peak_abs_max_mm=max((abs(v) for v in peak_values), default=math.nan),
        center_vs_slosh_mae_mm=mae(matched_center, matched_slosh),
        center_vs_slosh_rmse_mm=rmse(matched_center, matched_slosh),
        center_vs_slosh_corr=corr(matched_center, matched_slosh),
        bias_mean_mm=mean_or_nan(errors),
        bias_median_mm=median_or_nan(errors),
    )


def load_visual_series(label: str, verify_root: Path, center_column: str, bag_series: BagSeries) -> VisualSeries:
    liquid_csv = verify_root / label / "liquid_height_v2.csv"
    if not liquid_csv.is_file():
        raise RuntimeError(f"missing liquid_height_v2.csv for {label}: {liquid_csv}")

    rel_time_s: List[float] = []
    center_signed_pairs: List[Tuple[float, float]] = []
    slosh_mm: List[float] = []
    with liquid_csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            if int(raw.get("accept_for_peak_report_v2", "0") or "0") != 1:
                continue
            stamp = finite_float(raw.get("stamp"))
            rel_time = finite_float(raw.get("relative_time_s"))
            center_signed = finite_float(raw.get("height_center_rel_mm_signed_bias_corrected_v2"))
            if not math.isfinite(stamp) or not math.isfinite(rel_time) or not math.isfinite(center_signed):
                continue
            rel_time_s.append(rel_time)
            center_signed_pairs.append((rel_time, center_signed))
            slosh_mm.append(interpolate_scalar(bag_series.slosh_height_t, bag_series.slosh_height_mm_zero, rel_time))
    offset = 0.0
    if center_signed_pairs:
        early_values = [v for t, v in center_signed_pairs if t <= 1.0]
        if len(early_values) < 3:
            early_values = [v for _, v in center_signed_pairs[:30]]
        if early_values:
            offset = float(statistics.median(early_values))
    center_mm = [v - offset for _, v in center_signed_pairs]
    return VisualSeries(label=label, rel_time_s=rel_time_s, center_mm=center_mm, slosh_mm=slosh_mm)


def parse_pairs(raw_pairs: str) -> List[Tuple[str, str]]:
    out: List[Tuple[str, str]] = []
    for raw in raw_pairs.split(","):
        raw = raw.strip()
        if not raw:
            continue
        if ":" not in raw:
            raise ValueError(f"--pairs must use Q0:Q5 format: {raw}")
        left, right = raw.split(":", 1)
        out.append((left.strip(), right.strip()))
    return out


def plot_pair_path_and_excitation(out_path: Path, pair_name: str, q0_series: BagSeries, q5_series: BagSeries):
    ensure_dir(out_path.parent)
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), constrained_layout=True)
    ax1, ax2, ax3 = axes
    ax1.plot(q0_series.odom_x_local_m, q0_series.odom_y_local_m, color="#1f77b4", linewidth=1.4, label=q0_series.label)
    ax1.plot(q5_series.odom_x_local_m, q5_series.odom_y_local_m, color="#d62728", linewidth=1.4, label=q5_series.label)
    ax1.set_title(f"{pair_name}: odom path overlay")
    ax1.set_xlabel("x local [m]")
    ax1.set_ylabel("y local [m]")
    ax1.axis("equal")
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="best")

    ax2.plot(q0_series.cmd_t, q0_series.cmd_v_mps, color="#1f77b4", linewidth=1.2, label=f"{q0_series.label} v")
    ax2.plot(q5_series.cmd_t, q5_series.cmd_v_mps, color="#d62728", linewidth=1.2, label=f"{q5_series.label} v")
    ax2.plot(q0_series.cmd_t, q0_series.cmd_omega_radps, color="#1f77b4", linewidth=1.0, linestyle="--", alpha=0.8, label=f"{q0_series.label} omega")
    ax2.plot(q5_series.cmd_t, q5_series.cmd_omega_radps, color="#d62728", linewidth=1.0, linestyle="--", alpha=0.8, label=f"{q5_series.label} omega")
    ax2.set_title("cmd_vel comparison")
    ax2.set_xlabel("relative time [s]")
    ax2.set_ylabel("v / omega")
    ax2.grid(True, alpha=0.25)
    ax2.legend(loc="best", ncol=2)

    ax3.plot(q0_series.cmd_t, q0_series.ay_model_mps2, color="#1f77b4", linewidth=1.2, label=f"{q0_series.label} ay_model")
    ax3.plot(q5_series.cmd_t, q5_series.ay_model_mps2, color="#d62728", linewidth=1.2, label=f"{q5_series.label} ay_model")
    ax3.plot(q0_series.slosh_height_t, q0_series.slosh_height_mm_zero, color="#1f77b4", linewidth=1.0, linestyle="--", alpha=0.8, label=f"{q0_series.label} /slosh/height")
    ax3.plot(q5_series.slosh_height_t, q5_series.slosh_height_mm_zero, color="#d62728", linewidth=1.0, linestyle="--", alpha=0.8, label=f"{q5_series.label} /slosh/height")
    ax3.set_title("model excitation and /slosh/height")
    ax3.set_xlabel("relative time [s]")
    ax3.set_ylabel("ay [m/s²] / height [mm]")
    ax3.grid(True, alpha=0.25)
    ax3.legend(loc="best", ncol=2)

    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def plot_pair_center_vs_slosh(out_path: Path, pair_name: str, q0_visual: VisualSeries, q5_visual: VisualSeries):
    ensure_dir(out_path.parent)
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=False, constrained_layout=True)
    for axis, visual in zip(axes, [q0_visual, q5_visual]):
        axis.plot(visual.rel_time_s, visual.center_mm, color="#d62728", linewidth=1.5, label=f"{visual.label} RealSense center")
        axis.plot(visual.rel_time_s, visual.slosh_mm, color="#1f77b4", linewidth=1.3, label=f"{visual.label} /slosh/height")
        axis.set_ylabel("height [mm]")
        axis.set_title(f"{visual.label}: RealSense center vs /slosh/height")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("relative time [s]")
    fig.suptitle(f"{pair_name}: paired RealSense center and /slosh/height", fontsize=12)
    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def build_readme(path: Path, pair_rows: Sequence[Dict[str, object]]):
    lines = [
        "# 0330 Q0/Q5 成对比较",
        "",
        "本目录只比较同一路径下的两对 bag：",
        "- `Q0_test1 vs Q5_test1`",
        "- `Q0_test2 vs Q5_test2`",
        "",
        "重要约束：",
        "- 本轮不使用 `0330/Q0_static` 重新定零位",
        "- 统一沿用上一批的静止基准与 bias",
        "",
        "用途：",
        "- 在同一路径下比较 `Q0/Q5` 的实际路径、激励和液面响应",
        "- 先做相对比较，不对这批绝对零位误差下强结论",
        "- 每对额外导出一张 `RealSense center` 和 `/slosh/height` 的同图比较",
        "",
        "当前结果：",
    ]
    for row in pair_rows:
        lines.append(
            f"- `{row['pair_name']}`: path_mean_dist={row['path_shape_mean_distance_m']:.3f} m, "
            f"speed_p90_ratio={row['speed_p90_ratio']:.3f}, "
            f"ay_p90_ratio={row['ay_p90_ratio']:.3f}, "
            f"Q5/Q0 /slosh/height p90 ratio={row['slosh_height_p90_ratio_q5_over_q0']:.3f}, "
            f"Q5/Q0 RealSense center p90 ratio={row['realsense_center_p90_ratio_q5_over_q0']:.3f}"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    bag_root = Path(args.bag_root).expanduser().resolve()
    verify_root = Path(args.verify_root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    ensure_dir(out_dir)

    pairs = parse_pairs(args.pairs)
    labels = sorted({label for pair in pairs for label in pair})
    bag_files = {label: (bag_root / f"slosh_{label}_20260330".replace("__", "_")) for label in labels}
    # Bag files are looked up by substring to tolerate exact timestamps.
    resolved_bags: Dict[str, Path] = {}
    for label in labels:
        matches = sorted(p for p in bag_root.glob("*.bag") if label.lower() in p.name.lower())
        if len(matches) != 1:
            raise SystemExit(f"[ERROR] expected exactly one bag for {label} under {bag_root}, got: {matches}")
        resolved_bags[label] = matches[0]

    series_map: Dict[str, BagSeries] = {}
    visual_map: Dict[str, VisualMetrics] = {}
    visual_series_map: Dict[str, VisualSeries] = {}
    for label in labels:
        series_map[label] = load_bag_series(label, resolved_bags[label], args)
        visual_map[label] = load_visual_metrics(label, verify_root, args.center_column, series_map[label], args)
        visual_series_map[label] = load_visual_series(label, verify_root, args.center_column, series_map[label])

    pair_rows: List[Dict[str, object]] = []
    for q0_label, q5_label in pairs:
        q0_series = series_map[q0_label]
        q5_series = series_map[q5_label]
        q0_visual = visual_map[q0_label]
        q5_visual = visual_map[q5_label]
        path_mean_dist, path_max_dist = compare_paths(q0_series, q5_series)
        row = {
            "pair_name": f"{q0_label}_vs_{q5_label}",
            "q0_label": q0_label,
            "q5_label": q5_label,
            "path_shape_mean_distance_m": path_mean_dist,
            "path_shape_max_distance_m": path_max_dist,
            "duration_ratio": safe_ratio(q0_series.duration_sec, q5_series.duration_sec),
            "path_length_ratio": safe_ratio(
                polyline_length(q0_series.odom_x_local_m, q0_series.odom_y_local_m),
                polyline_length(q5_series.odom_x_local_m, q5_series.odom_y_local_m),
            ),
            "speed_p90_ratio": safe_ratio(percentile90(q0_series.cmd_v_mps), percentile90(q5_series.cmd_v_mps)),
            "omega_p90_ratio": safe_ratio(percentile90(q0_series.cmd_omega_radps), percentile90(q5_series.cmd_omega_radps)),
            "ay_p90_ratio": safe_ratio(percentile90(q0_series.ay_model_mps2), percentile90(q5_series.ay_model_mps2)),
            "q0_slosh_height_abs_p90_mm": percentile90(q0_series.slosh_height_mm_zero),
            "q5_slosh_height_abs_p90_mm": percentile90(q5_series.slosh_height_mm_zero),
            "slosh_height_p90_ratio_q5_over_q0": (
                float(percentile90(q5_series.slosh_height_mm_zero) / percentile90(q0_series.slosh_height_mm_zero))
                if math.isfinite(percentile90(q0_series.slosh_height_mm_zero)) and abs(percentile90(q0_series.slosh_height_mm_zero)) > 1e-9
                else math.nan
            ),
            "q0_realsense_center_abs_p90_mm": q0_visual.center_abs_p90_mm,
            "q5_realsense_center_abs_p90_mm": q5_visual.center_abs_p90_mm,
            "realsense_center_p90_ratio_q5_over_q0": (
                float(q5_visual.center_abs_p90_mm / q0_visual.center_abs_p90_mm)
                if math.isfinite(q0_visual.center_abs_p90_mm) and abs(q0_visual.center_abs_p90_mm) > 1e-9
                else math.nan
            ),
            "q0_center_vs_slosh_mae_mm": q0_visual.center_vs_slosh_mae_mm,
            "q5_center_vs_slosh_mae_mm": q5_visual.center_vs_slosh_mae_mm,
            "q0_bias_median_mm": q0_visual.bias_median_mm,
            "q5_bias_median_mm": q5_visual.bias_median_mm,
            "q0_reportable_frames": q0_visual.reportable_frames,
            "q5_reportable_frames": q5_visual.reportable_frames,
        }
        pair_rows.append(row)
        plot_pair_path_and_excitation(out_dir / f"{row['pair_name']}.png", row["pair_name"], q0_series, q5_series)
        plot_pair_center_vs_slosh(
            out_dir / f"{row['pair_name']}_center_vs_slosh.png",
            row["pair_name"],
            visual_series_map[q0_label],
            visual_series_map[q5_label],
        )

    summary_csv = out_dir / "paired_q0_q5_summary.csv"
    with summary_csv.open("w", encoding="utf-8", newline="") as handle:
        fieldnames = list(pair_rows[0].keys())
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(pair_rows)

    summary_json = out_dir / "paired_q0_q5_summary.json"
    with summary_json.open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "bag_root": str(bag_root),
                "verify_root": str(verify_root),
                "pairs": pair_rows,
            },
            handle,
            ensure_ascii=False,
            indent=2,
        )

    build_readme(out_dir / "README.md", pair_rows)

    print(f"[OK] summary csv: {summary_csv}")
    print(f"[OK] summary json: {summary_json}")
    print(f"[OK] readme: {out_dir / 'README.md'}")
    for row in pair_rows:
        print(
            f"[INFO] {row['pair_name']}: path_mean_dist={row['path_shape_mean_distance_m']:.6f} m, "
            f"Q5/Q0 slosh_p90_ratio={row['slosh_height_p90_ratio_q5_over_q0']:.6f}, "
            f"Q5/Q0 rs_center_p90_ratio={row['realsense_center_p90_ratio_q5_over_q0']:.6f}"
        )


if __name__ == "__main__":
    main()
