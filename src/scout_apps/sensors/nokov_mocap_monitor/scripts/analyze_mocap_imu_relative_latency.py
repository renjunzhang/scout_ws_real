#!/usr/bin/env python3
"""Estimate NOKOV pose-stream lag relative to a rigidly mounted IMU gyro.

The primary estimate aligns IMU ``angular_velocity.z`` with yaw rate derived
from the raw VRPN pose.  Positive lag means that the NOKOV curve appears later
than the IMU curve.  The result is relative latency only; it cannot establish
absolute NOKOV capture-to-host latency without an independent hardware clock or
trigger.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np


DEFAULT_IMU_TOPIC = "/imu/data"
DEFAULT_MOCAP_TOPIC = "/vrpn_client_node/Tracker0/pose"
DEFAULT_SEGMENT_TOPIC = "/mocap_imu_calib/segment"
MOTION_START_PREFIX = "NOKOV_IMU_LATENCY_MOTION_START"
MOTION_END_PREFIX = "NOKOV_IMU_LATENCY_MOTION_END"


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def header_or_bag_time(message, bag_time):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    try:
        value = float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        value = 0.0
    return value if math.isfinite(value) and value > 0.0 else float(bag_time)


def unique_samples(times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    finite = np.isfinite(times) & np.isfinite(values)
    ordered = np.argsort(times[finite], kind="mergesort")
    times = times[finite][ordered]
    values = values[finite][ordered]
    if not times.size:
        return times, values
    unique_time = []
    unique_value = []
    for stamp, value in zip(times, values):
        if unique_time and stamp <= unique_time[-1] + 1e-12:
            unique_time[-1] = float(stamp)
            unique_value[-1] = float(value)
        else:
            unique_time.append(float(stamp))
            unique_value.append(float(value))
    return np.asarray(unique_time), np.asarray(unique_value)


def stream_stats(times):
    times = np.asarray(times, dtype=float)
    times = times[np.isfinite(times)]
    if times.size < 2:
        return {"count": int(times.size)}
    deltas = np.diff(times)
    positive = deltas[deltas > 0.0]
    duration = max(0.0, float(times[-1] - times[0]))
    return {
        "count": int(times.size),
        "duration_sec": duration,
        "rate_hz": float((times.size - 1) / duration) if duration > 0.0 else 0.0,
        "median_period_sec": float(np.median(positive)) if positive.size else None,
        "p95_period_sec": float(np.percentile(positive, 95.0)) if positive.size else None,
        "max_period_sec": float(np.max(positive)) if positive.size else None,
        "nonpositive_delta_count": int(np.count_nonzero(deltas <= 0.0)),
    }


def finite_summary(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return {"count": 0}
    return {
        "count": int(values.size),
        "median": float(np.median(values)),
        "p05": float(np.percentile(values, 5.0)),
        "p95": float(np.percentile(values, 95.0)),
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
    }


def odd_window_samples(window_sec, sample_rate_hz, available):
    count = max(3, int(round(float(window_sec) * float(sample_rate_hz))))
    if count % 2 == 0:
        count += 1
    maximum = available if available % 2 else available - 1
    return max(3, min(count, maximum))


def centered_moving_average(values, count):
    values = np.asarray(values, dtype=float)
    count = max(1, int(count))
    if count % 2 == 0:
        count += 1
    maximum = values.size if values.size % 2 else values.size - 1
    count = min(count, maximum)
    if count <= 1:
        return values.copy()
    half = count // 2
    padded = np.pad(values, (half, half), mode="edge")
    return np.convolve(padded, np.ones(count) / float(count), mode="valid")


def centered_polynomial_derivative(values, dt_sec, window_sec, order=3):
    values = np.asarray(values, dtype=float)
    count = odd_window_samples(window_sec, 1.0 / dt_sec, values.size)
    half = count // 2
    offsets = np.arange(-half, half + 1, dtype=float) * dt_sec
    design = np.vander(offsets, order + 1, increasing=True)
    coefficients = np.linalg.pinv(design)[1]
    padded = np.pad(values, (half, half), mode="edge")
    return np.convolve(padded, coefficients[::-1], mode="valid")


def correlation(first, second):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    mask = np.isfinite(first) & np.isfinite(second)
    first = first[mask]
    second = second[mask]
    if first.size < 10:
        return 0.0
    first = first - np.mean(first)
    second = second - np.mean(second)
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator <= 1e-12:
        return 0.0
    return float(np.dot(first, second) / denominator)


def lagged_pair(reference, response, lag_samples):
    if lag_samples > 0:
        return reference[:-lag_samples], response[lag_samples:]
    if lag_samples < 0:
        return reference[-lag_samples:], response[:lag_samples]
    return reference, response


def estimate_lag(reference, response, dt_sec, max_lag_sec):
    reference = np.asarray(reference, dtype=float)
    response = np.asarray(response, dtype=float)
    maximum = max(1, int(round(float(max_lag_sec) / float(dt_sec))))
    lag_indices = np.arange(-maximum, maximum + 1, dtype=int)
    correlations = []
    for lag_index in lag_indices:
        left, right = lagged_pair(reference, response, int(lag_index))
        correlations.append(correlation(left, right))
    correlations = np.asarray(correlations, dtype=float)
    best_index = int(np.argmax(np.abs(correlations)))
    lag_index = int(lag_indices[best_index])
    raw_correlation = float(correlations[best_index])
    polarity = 1 if raw_correlation >= 0.0 else -1
    return {
        "lag_sec": float(lag_index * dt_sec),
        "lag_ms": float(lag_index * dt_sec * 1000.0),
        "peak_correlation_abs": float(abs(raw_correlation)),
        "raw_peak_correlation": raw_correlation,
        "polarity": polarity,
        "zero_lag_correlation": float(correlations[maximum]),
        "at_search_boundary": bool(abs(lag_index) == maximum),
        "lag_grid_sec": [float(value * dt_sec) for value in lag_indices],
        "correlation_grid": [float(value) for value in correlations],
    }


def find_marker(events, prefix, first=True):
    matches = [stamp for stamp, value in events if value.startswith(prefix)]
    if not matches:
        return None
    return float(matches[0] if first else matches[-1])


def load_bag(path, imu_topic, mocap_topic, segment_topic):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError(
            "rosbag is unavailable; source /opt/ros/noetic and the workspace: {}".format(exc)
        )

    imu = []
    mocap = []
    events = []
    statuses = []
    topics = [imu_topic, mocap_topic, segment_topic, "/mocap/status"]
    with rosbag.Bag(str(path), "r") as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())
        missing = [topic for topic in (imu_topic, mocap_topic) if topic not in available]
        if missing:
            raise RuntimeError("bag is missing required topics: {}".format(missing))
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            bag_time = float(bag_stamp.to_sec())
            if topic == imu_topic:
                imu.append(
                    (
                        bag_time,
                        header_or_bag_time(message, bag_time),
                        float(message.angular_velocity.z),
                    )
                )
            elif topic == mocap_topic:
                mocap.append(
                    (
                        bag_time,
                        header_or_bag_time(message, bag_time),
                        quaternion_yaw(message.pose.orientation),
                    )
                )
            elif topic == segment_topic:
                events.append((bag_time, str(message.data)))
            elif topic == "/mocap/status":
                statuses.append((bag_time, str(message.data)))

    if len(imu) < 100 or len(mocap) < 100:
        raise RuntimeError(
            "insufficient samples: imu={}, mocap={}".format(len(imu), len(mocap))
        )
    return {
        "imu": np.asarray(imu, dtype=float),
        "mocap": np.asarray(mocap, dtype=float),
        "events": events,
        "statuses": statuses,
    }


def analyze_time_source(raw, time_column, args, motion_start, motion_end):
    imu_time, imu_value = unique_samples(
        raw["imu"][:, time_column], raw["imu"][:, 2]
    )
    mocap_time, mocap_yaw = unique_samples(
        raw["mocap"][:, time_column], raw["mocap"][:, 2]
    )
    if imu_time.size < 100 or mocap_time.size < 100:
        raise RuntimeError("too few unique IMU or NOKOV timestamps")

    dt_sec = 1.0 / float(args.resample_hz)
    full_start = max(float(imu_time[0]), float(mocap_time[0]))
    full_end = min(float(imu_time[-1]), float(mocap_time[-1]))
    if full_end - full_start < args.minimum_duration_sec:
        raise RuntimeError("IMU/NOKOV overlap is shorter than the minimum duration")
    grid = np.arange(full_start, full_end + 0.25 * dt_sec, dt_sec)
    if grid.size < 100:
        raise RuntimeError("resampled overlap is too short")

    imu_grid = np.interp(grid, imu_time, imu_value)
    imu_window = odd_window_samples(
        args.imu_smooth_window_sec, args.resample_hz, imu_grid.size
    )
    imu_grid = centered_moving_average(imu_grid, imu_window)

    unwrapped_yaw = np.unwrap(mocap_yaw)
    yaw_grid = np.interp(grid, mocap_time, unwrapped_yaw)
    mocap_rate = centered_polynomial_derivative(
        yaw_grid, dt_sec, args.mocap_smooth_window_sec
    )

    if motion_start is None or motion_end is None or motion_end <= motion_start:
        trim = min(2.0, 0.10 * (full_end - full_start))
        selected_start = full_start + trim
        selected_end = full_end - trim
        marker_window_used = False
    else:
        selected_start = max(full_start, float(motion_start))
        selected_end = min(full_end, float(motion_end))
        marker_window_used = True
    selected = (grid >= selected_start) & (grid <= selected_end)
    if np.count_nonzero(selected) < int(args.minimum_duration_sec / dt_sec):
        raise RuntimeError("marked motion interval is too short")

    selected_time = grid[selected]
    selected_imu = imu_grid[selected]
    selected_mocap = mocap_rate[selected]
    global_lag = estimate_lag(
        selected_imu, selected_mocap, dt_sec, args.max_lag_sec
    )

    window_count = max(1, int(round(args.local_window_sec / dt_sec)))
    step_count = max(1, int(round(args.local_step_sec / dt_sec)))
    local_rows = []
    if selected_time.size >= window_count:
        for begin in range(0, selected_time.size - window_count + 1, step_count):
            end = begin + window_count
            imu_window_values = selected_imu[begin:end]
            mocap_window_values = selected_mocap[begin:end]
            if (
                float(np.std(imu_window_values)) < args.minimum_motion_std_radps
                or float(np.std(mocap_window_values)) < args.minimum_motion_std_radps
            ):
                continue
            estimate = estimate_lag(
                imu_window_values,
                mocap_window_values,
                dt_sec,
                args.max_lag_sec,
            )
            if estimate["peak_correlation_abs"] < args.minimum_local_correlation:
                continue
            local_rows.append(
                {
                    "start_sec": float(selected_time[begin] - selected_time[0]),
                    "end_sec": float(selected_time[end - 1] - selected_time[0]),
                    "lag_sec": float(estimate["lag_sec"]),
                    "lag_ms": float(estimate["lag_ms"]),
                    "peak_correlation_abs": float(
                        estimate["peak_correlation_abs"]
                    ),
                    "polarity": int(estimate["polarity"]),
                }
            )

    local_lag_summary = finite_summary([row["lag_sec"] for row in local_rows])
    local_correlation_summary = finite_summary(
        [row["peak_correlation_abs"] for row in local_rows]
    )
    imu_stats = stream_stats(imu_time)
    mocap_stats = stream_stats(mocap_time)
    periods = [
        value
        for value in (
            imu_stats.get("median_period_sec"),
            mocap_stats.get("median_period_sec"),
        )
        if value is not None and math.isfinite(value)
    ]
    native_resolution = max(periods) if periods else None

    summary = {
        "marker_window_used": marker_window_used,
        "analysis_start_sec": float(selected_start),
        "analysis_end_sec": float(selected_end),
        "analysis_duration_sec": float(selected_end - selected_start),
        "resample_period_sec": float(dt_sec),
        "native_time_resolution_sec": (
            float(native_resolution) if native_resolution is not None else None
        ),
        "imu_stream": imu_stats,
        "mocap_stream": mocap_stats,
        "imu_motion_std_radps": float(np.std(selected_imu)),
        "mocap_motion_std_radps": float(np.std(selected_mocap)),
        "imu_peak_abs_radps": float(np.max(np.abs(selected_imu))),
        "mocap_peak_abs_radps": float(np.max(np.abs(selected_mocap))),
        "global": {
            key: value
            for key, value in global_lag.items()
            if key not in ("lag_grid_sec", "correlation_grid")
        },
        "local_windows": {
            "accepted_count": len(local_rows),
            "lag_sec_summary": local_lag_summary,
            "correlation_summary": local_correlation_summary,
            "rows": local_rows,
        },
    }
    series = {
        "time": selected_time,
        "imu": selected_imu,
        "mocap": selected_mocap,
        "lag_grid_sec": np.asarray(global_lag["lag_grid_sec"], dtype=float),
        "correlation_grid": np.asarray(
            global_lag["correlation_grid"], dtype=float
        ),
        "polarity": int(global_lag["polarity"]),
    }
    return summary, series


def interpret_lag(lag_sec, native_resolution_sec):
    resolution = native_resolution_sec or 0.0
    if abs(lag_sec) <= 0.5 * resolution:
        return "在原始采样时间分辨率下，NOKOV 与 IMU 基本对齐"
    if lag_sec > 0.0:
        return "NOKOV 的运动曲线比 IMU 晚"
    return "NOKOV 的运动曲线比 IMU 早，等价地说 IMU 更晚"


def write_series_csv(path, series):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["relative_time_sec", "imu_gyro_z_radps", "mocap_yaw_rate_radps"]
        )
        start = float(series["time"][0])
        for stamp, imu_value, mocap_value in zip(
            series["time"], series["imu"], series["mocap"]
        ):
            writer.writerow(
                [
                    "{:.6f}".format(float(stamp - start)),
                    "{:.9f}".format(float(imu_value)),
                    "{:.9f}".format(float(mocap_value)),
                ]
            )


def write_plot(path, series, lag_sec):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    path.parent.mkdir(parents=True, exist_ok=True)
    relative_time = series["time"] - series["time"][0]
    polarity = float(series["polarity"])
    figure, axes = plt.subplots(2, 1, figsize=(12, 7))
    axes[0].plot(relative_time, series["imu"], label="IMU gyro z")
    axes[0].plot(
        relative_time,
        polarity * series["mocap"],
        label="NOKOV yaw rate (polarity aligned)",
        alpha=0.85,
    )
    axes[0].set_ylabel("rad/s")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(
        1000.0 * series["lag_grid_sec"],
        np.abs(series["correlation_grid"]),
    )
    axes[1].axvline(1000.0 * lag_sec, color="r", linestyle="--")
    axes[1].set_xlabel("NOKOV relative to IMU lag (ms); positive = NOKOV later")
    axes[1].set_ylabel("absolute correlation")
    axes[1].grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(str(path), dpi=150)
    plt.close(figure)
    return True


def write_markdown(path, report):
    result = report["result"]
    bag = report["time_sources"]["bag_time"]
    header = report["time_sources"]["header_stamp"]
    local = bag["local_windows"]["lag_sec_summary"]
    recommended_lag = result.get("recommended_lag_sec")
    estimate_label = "互相关" if recommended_lag is not None else "候选互相关（未通过质量检查）"
    lines = [
        "# NOKOV 相对 IMU 延迟快速测量",
        "",
        "- 结论：`{}`".format(result["status"]),
        (
            "- 推荐相对延迟：`{:.1f} ms`".format(1000.0 * recommended_lag)
            if recommended_lag is not None
            else "- 推荐相对延迟：`不可用（本次数据未通过质量检查）`"
        ),
        "- 符号约定：正值表示 NOKOV 比 IMU 晚；负值表示 NOKOV 更早或 IMU 更晚。",
        "- 简单解释：{}。".format(result["interpretation"]),
        "- bag 到达时间{}：`{:.1f} ms`，峰值相关系数 `{:.3f}`。".format(
            estimate_label,
            bag["global"]["lag_ms"],
            bag["global"]["peak_correlation_abs"],
        ),
        "- header 时间{}：`{:.1f} ms`，峰值相关系数 `{:.3f}`。".format(
            estimate_label,
            header["global"]["lag_ms"],
            header["global"]["peak_correlation_abs"],
        ),
        "- 原始采样时间分辨率约：`{:.1f} ms`。".format(
            1000.0 * float(bag["native_time_resolution_sec"] or 0.0)
        ),
        "- 有效局部窗口：`{}`。".format(
            bag["local_windows"]["accepted_count"]
        ),
    ]
    if local.get("count", 0):
        lines.append(
            "- 局部窗口延迟中位数/P05/P95：`{:.1f}/{:.1f}/{:.1f} ms`。".format(
                1000.0 * local["median"],
                1000.0 * local["p05"],
                1000.0 * local["p95"],
            )
        )
    if result["blockers"]:
        lines.extend(["", "## 未通过原因", ""])
        lines.extend("- {}".format(item) for item in result["blockers"])
    if result["warnings"]:
        lines.extend(["", "## 注意", ""])
        lines.extend("- {}".format(item) for item in result["warnings"])
    lines.extend(
        [
            "",
            "## 结论边界",
            "",
            "该结果只能称为“NOKOV 相对 IMU 延迟”，不能称为 NOKOV 绝对延迟。",
            "IMU 自身延迟、两侧时间戳语义、离线零相位滤波和 rosbag 调度都会进入结果。",
            "",
            "- Bag：`{}`".format(report["bag"]),
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--mocap-topic", default=DEFAULT_MOCAP_TOPIC)
    parser.add_argument("--segment-topic", default=DEFAULT_SEGMENT_TOPIC)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-summary", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-plot", required=True)
    parser.add_argument("--resample-hz", type=float, default=200.0)
    parser.add_argument("--mocap-smooth-window-sec", type=float, default=0.11)
    parser.add_argument("--imu-smooth-window-sec", type=float, default=0.03)
    parser.add_argument("--max-lag-sec", type=float, default=0.30)
    parser.add_argument("--minimum-duration-sec", type=float, default=8.0)
    parser.add_argument("--minimum-motion-std-radps", type=float, default=0.03)
    parser.add_argument("--minimum-global-correlation", type=float, default=0.70)
    parser.add_argument("--minimum-local-correlation", type=float, default=0.75)
    parser.add_argument("--local-window-sec", type=float, default=6.0)
    parser.add_argument("--local-step-sec", type=float, default=3.0)
    return parser.parse_args()


def validate_args(args):
    positive = (
        "resample_hz",
        "mocap_smooth_window_sec",
        "imu_smooth_window_sec",
        "max_lag_sec",
        "minimum_duration_sec",
        "minimum_motion_std_radps",
        "local_window_sec",
        "local_step_sec",
    )
    for name in positive:
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and positive".format(name))
    for name in ("minimum_global_correlation", "minimum_local_correlation"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0 or value > 1.0:
            raise ValueError("{} must be in (0, 1]".format(name))


def main():
    args = parse_args()
    validate_args(args)
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_file():
        raise FileNotFoundError(str(bag_path))
    raw = load_bag(
        bag_path, args.imu_topic, args.mocap_topic, args.segment_topic
    )
    motion_start = find_marker(raw["events"], MOTION_START_PREFIX, first=True)
    motion_end = find_marker(raw["events"], MOTION_END_PREFIX, first=False)

    bag_summary, bag_series = analyze_time_source(
        raw, 0, args, motion_start, motion_end
    )
    header_summary, _header_series = analyze_time_source(
        raw, 1, args, motion_start, motion_end
    )

    recommended_lag = float(bag_summary["global"]["lag_sec"])
    blockers = []
    warnings = []
    if not bag_summary["marker_window_used"]:
        warnings.append(
            "缺少运动开始/结束标记，分析改用了去掉首尾后的整包区间"
        )
    if bag_summary["imu_motion_std_radps"] < args.minimum_motion_std_radps:
        blockers.append("IMU 角速度激励太弱")
    if bag_summary["mocap_motion_std_radps"] < args.minimum_motion_std_radps:
        blockers.append("NOKOV 角速度激励太弱")
    if (
        bag_summary["global"]["peak_correlation_abs"]
        < args.minimum_global_correlation
    ):
        blockers.append("IMU 与 NOKOV 的峰值相关系数太低")
    if bag_summary["global"]["at_search_boundary"]:
        blockers.append("最佳延迟落在搜索范围边界")
    if bag_summary["local_windows"]["accepted_count"] < 2:
        warnings.append("通过激励和相关性检查的局部窗口少于两个")
    source_disagreement = abs(
        bag_summary["global"]["lag_sec"]
        - header_summary["global"]["lag_sec"]
    )
    native_resolution = float(bag_summary["native_time_resolution_sec"] or 0.0)
    if source_disagreement > max(0.03, 2.0 * native_resolution):
        warnings.append(
            "bag 到达时间与 header 时间得到的延迟差异较大"
        )
    statuses = [value for _stamp, value in raw["statuses"]]
    if statuses and any(not value.startswith("OK") for value in statuses):
        warnings.append("bag 内的 /mocap/status 出现过非 OK 状态")

    usable = not blockers
    result = {
        "status": "USABLE_RELATIVE_LAG_ESTIMATE" if usable else "INCONCLUSIVE",
        "recommended_time_source": "bag_time",
        "recommended_lag_sec": recommended_lag if usable else None,
        "recommended_lag_ms": 1000.0 * recommended_lag if usable else None,
        "candidate_lag_sec": recommended_lag,
        "candidate_lag_ms": 1000.0 * recommended_lag,
        "sign_convention": "positive means NOKOV appears later than IMU",
        "interpretation": (
            interpret_lag(recommended_lag, native_resolution)
            if usable
            else "本次激励或相关性没有通过检查，不能解释为有效延迟"
        ),
        "blockers": blockers,
        "warnings": warnings,
    }
    report = {
        "schema": "nokov_imu_relative_latency_v1",
        "bag": str(bag_path),
        "topics": {
            "imu": args.imu_topic,
            "mocap_pose": args.mocap_topic,
            "segment": args.segment_topic,
        },
        "processing": {
            "method": "zero-phase yaw differentiation plus IMU/NOKOV cross-correlation",
            "resample_hz": args.resample_hz,
            "mocap_smooth_window_sec": args.mocap_smooth_window_sec,
            "imu_smooth_window_sec": args.imu_smooth_window_sec,
            "max_lag_sec": args.max_lag_sec,
            "motion_start_bag_sec": motion_start,
            "motion_end_bag_sec": motion_end,
        },
        "sample_counts": {
            "imu": int(raw["imu"].shape[0]),
            "mocap": int(raw["mocap"].shape[0]),
            "events": len(raw["events"]),
            "mocap_status": len(raw["statuses"]),
        },
        "arrival_minus_header_sec": {
            "imu": finite_summary(raw["imu"][:, 0] - raw["imu"][:, 1]),
            "mocap": finite_summary(raw["mocap"][:, 0] - raw["mocap"][:, 1]),
        },
        "time_sources": {
            "bag_time": bag_summary,
            "header_stamp": header_summary,
        },
        "result": result,
        "claim_scope": (
            "NOKOV relative to IMU lag only; not NOKOV absolute latency"
        ),
    }

    output_json = Path(args.output_json).expanduser().resolve()
    output_summary = Path(args.output_summary).expanduser().resolve()
    output_csv = Path(args.output_csv).expanduser().resolve()
    output_plot = Path(args.output_plot).expanduser().resolve()
    for path in (output_json, output_summary, output_csv, output_plot):
        path.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_markdown(output_summary, report)
    write_series_csv(output_csv, bag_series)
    plot_written = write_plot(output_plot, bag_series, recommended_lag)

    lag_text = (
        "{:.1f} ms".format(result["recommended_lag_ms"])
        if result["recommended_lag_ms"] is not None
        else "unavailable"
    )
    print(
        "[nokov-relative-latency] status={} lag={} corr={:.3f}".format(
            result["status"], lag_text, bag_summary["global"]["peak_correlation_abs"]
        )
    )
    print("[nokov-relative-latency] {}".format(result["interpretation"]))
    print("[nokov-relative-latency] report={}".format(output_json))
    print("[nokov-relative-latency] summary={}".format(output_summary))
    if plot_written:
        print("[nokov-relative-latency] plot={}".format(output_plot))
    else:
        print("[nokov-relative-latency] plot skipped: matplotlib unavailable")
    return 0 if not blockers else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("[nokov-relative-latency][ERROR] {}".format(exc), file=sys.stderr)
        sys.exit(2)
