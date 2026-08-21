#!/usr/bin/env python3
"""Preliminary dual-channel Scout execution identification from a planar bag.

The input bag is opened read-only.  All generated artifacts are written below
``--out-dir``.  This tool is deliberately conservative: a single planar trial
can produce a simulation-development candidate, but never a formal robot
execution artifact or held-out validation claim.
"""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

try:
    import rosbag
except ImportError:  # pragma: no cover - depends on the ROS runtime
    rosbag = None

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover - plots are optional in minimal envs
    plt = None


TOOL_SCHEMA = "spmpc_planar_execution_identification_v2"
MODEL_SCHEMA = "dual_channel_delay_first_order_deadzone_v1"
COMMAND_HOLD_CONTRACT = "causal_right_continuous_zoh_v1"


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def mount_root(path):
    current = path if path.is_dir() else path.parent
    current = current.resolve()
    while not os.path.ismount(str(current)) and current.parent != current:
        current = current.parent
    return current


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def message_time(message, bag_stamp):
    if hasattr(message, "header") and message.header.stamp.to_nsec() > 0:
        return message.header.stamp.to_sec()
    return bag_stamp.to_sec()


def load_bag(path):
    if rosbag is None:
        raise RuntimeError(
            "rosbag is required; source the ROS workspace first")
    signals = {
        "cmd_t": [], "cmd_v": [], "cmd_w": [],
        "pose_t": [], "pose_x": [], "pose_y": [], "pose_yaw": [],
        "odom_t": [], "odom_v": [], "odom_w": [],
        "events": [],
    }
    topics = [
        "/cmd_vel", "/mocap/scout_pose", "/odom",
        "/mocap_imu_calib/segment",
    ]
    with rosbag.Bag(str(path), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, stamp in bag.read_messages(topics=topics):
            if topic == "/cmd_vel":
                signals["cmd_t"].append(stamp.to_sec())
                signals["cmd_v"].append(message.linear.x)
                signals["cmd_w"].append(message.angular.z)
            elif topic == "/mocap/scout_pose":
                signals["pose_t"].append(message_time(message, stamp))
                signals["pose_x"].append(message.pose.position.x)
                signals["pose_y"].append(message.pose.position.y)
                signals["pose_yaw"].append(
                    yaw_from_quaternion(message.pose.orientation))
            elif topic == "/odom":
                signals["odom_t"].append(message_time(message, stamp))
                signals["odom_v"].append(message.twist.twist.linear.x)
                signals["odom_w"].append(message.twist.twist.angular.z)
            else:
                signals["events"].append((stamp.to_sec(), message.data))
    for key, values in list(signals.items()):
        if key != "events":
            signals[key] = np.asarray(values, dtype=float)
    signals["bag_start"] = float(bag_start)
    signals["bag_end"] = float(bag_end)
    return signals


def require_streams(signals):
    missing = []
    for key in ("cmd_t", "pose_t", "odom_t"):
        if len(signals[key]) < 10:
            missing.append(key)
    if missing:
        raise RuntimeError("missing required streams: " + ", ".join(missing))
    if not signals["events"]:
        raise RuntimeError("segment event stream is empty")


def stream_stats(stamps):
    gaps = np.diff(stamps)
    duration = float(stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
    return {
        "count": int(len(stamps)),
        "duration_sec": duration,
        "rate_hz": float((len(stamps) - 1) / duration)
        if duration > 0.0 else 0.0,
        "max_gap_sec": float(np.max(gaps)) if len(gaps) else 0.0,
        "regressions": int(np.sum(gaps <= 0.0)),
    }


def event_epoch(signals, prefix, default):
    for stamp, value in signals["events"]:
        if value.startswith(prefix):
            return stamp
    return default


def odd_window(sample_rate_hz, width_sec, sample_count):
    value = max(5, int(round(sample_rate_hz * width_sec)))
    if value % 2 == 0:
        value += 1
    maximum = sample_count if sample_count % 2 == 1 else sample_count - 1
    return min(value, maximum)


def sample_zoh(query_t, sample_t, sample_values):
    """Sample a timestamped command as a causal right-continuous ZOH.

    `/cmd_vel` changes at its publication timestamp and is then held until the
    next publication.  Linear interpolation would start every step before it
    was published and would therefore bias the identified delay and actuator
    time constant.  Outside the recorded range, preserve the first/last held
    command, matching the previous boundary convention.
    """
    query = np.asarray(query_t, dtype=float)
    stamps = np.asarray(sample_t, dtype=float)
    values = np.asarray(sample_values, dtype=float)
    if stamps.ndim != 1 or values.ndim != 1 or len(stamps) == 0 or \
            len(stamps) != len(values) or not np.all(np.isfinite(stamps)) or \
            not np.all(np.isfinite(values)) or np.any(np.diff(stamps) <= 0.0):
        raise ValueError("invalid ZOH command stream")
    indices = np.searchsorted(stamps, query, side="right") - 1
    indices = np.clip(indices, 0, len(stamps) - 1)
    return values[indices]


def make_grid(signals, sample_rate_hz, smooth_width_sec):
    start = event_epoch(signals, "SEQUENCE_START", signals["bag_start"])
    end = event_epoch(signals, "SEQUENCE_END", signals["bag_end"])
    start = max(start, signals["cmd_t"][0], signals["pose_t"][0],
                signals["odom_t"][0])
    end = min(end, signals["cmd_t"][-1], signals["pose_t"][-1],
              signals["odom_t"][-1])
    dt = 1.0 / sample_rate_hz
    absolute_t = np.arange(start, end, dt)
    relative_t = absolute_t - start
    cmd_v = sample_zoh(
        absolute_t, signals["cmd_t"], signals["cmd_v"])
    cmd_w = sample_zoh(
        absolute_t, signals["cmd_t"], signals["cmd_w"])
    pose_x = np.interp(absolute_t, signals["pose_t"], signals["pose_x"])
    pose_y = np.interp(absolute_t, signals["pose_t"], signals["pose_y"])
    pose_yaw = np.interp(
        absolute_t, signals["pose_t"], np.unwrap(signals["pose_yaw"]))
    window = odd_window(sample_rate_hz, smooth_width_sec, len(absolute_t))
    velocity_x = savgol_filter(
        pose_x, window, 3, deriv=1, delta=dt, mode="interp")
    velocity_y = savgol_filter(
        pose_y, window, 3, deriv=1, delta=dt, mode="interp")
    yaw_rate = savgol_filter(
        pose_yaw, window, 3, deriv=1, delta=dt, mode="interp")
    body_v = velocity_x * np.cos(pose_yaw) + velocity_y * np.sin(pose_yaw)
    odom_v = np.interp(absolute_t, signals["odom_t"], signals["odom_v"])
    odom_w = np.interp(absolute_t, signals["odom_t"], signals["odom_w"])

    first_motion = np.flatnonzero(
        (np.abs(cmd_v) > 1.0e-4) | (np.abs(cmd_w) > 1.0e-4))
    if len(first_motion) == 0:
        raise RuntimeError("command stream contains no excitation")
    static_end = max(1, int(first_motion[0]) - int(sample_rate_hz))
    body_v -= np.median(body_v[:static_end])
    yaw_rate -= np.median(yaw_rate[:static_end])
    odom_v -= np.median(odom_v[:static_end])
    odom_w -= np.median(odom_w[:static_end])
    return {
        "absolute_t": absolute_t,
        "t": relative_t,
        "dt": dt,
        "sample_rate_hz": sample_rate_hz,
        "savgol_window_samples": window,
        "savgol_width_sec": window * dt,
        "cmd_v": cmd_v,
        "cmd_w": cmd_w,
        "mocap_v": body_v,
        "mocap_w": yaw_rate,
        "odom_v": odom_v,
        "odom_w": odom_w,
    }


def dilate_mask(mask, before_samples, after_samples):
    indices = np.flatnonzero(mask)
    output = np.zeros_like(mask, dtype=bool)
    if len(indices) == 0:
        return output
    breaks = np.flatnonzero(np.diff(indices) > 1)
    starts = np.r_[indices[0], indices[breaks + 1]]
    ends = np.r_[indices[breaks], indices[-1]]
    for start, end in zip(starts, ends):
        lo = max(0, int(start) - before_samples)
        hi = min(len(mask), int(end) + after_samples + 1)
        output[lo:hi] = True
    return output


def build_masks(grid):
    rate = grid["sample_rate_hz"]
    linear_core = ((np.abs(grid["cmd_v"]) >= 0.075)
                   & (np.abs(grid["cmd_w"]) <= 0.02))
    angular_core = ((np.abs(grid["cmd_w"]) >= 0.15)
                    & (np.abs(grid["cmd_v"]) <= 0.02))
    combined_core = ((np.abs(grid["cmd_v"]) >= 0.075)
                     & (np.abs(grid["cmd_w"]) >= 0.15))
    before = int(round(0.35 * rate))
    after = int(round(1.25 * rate))
    return {
        "linear_fit": dilate_mask(linear_core, before, after),
        "angular_fit": dilate_mask(angular_core, before, after),
        "combined_eval": dilate_mask(combined_core, before, after),
        "linear_core": linear_core,
        "angular_core": angular_core,
        "combined_core": combined_core,
    }


def mapped_target(command, positive_gain, negative_gain, deadzone):
    magnitude = np.abs(command)
    gain = np.where(command >= 0.0, positive_gain, negative_gain)
    return np.where(
        magnitude > deadzone,
        np.sign(command) * gain * (magnitude - deadzone),
        0.0,
    )


def simulate_channel(t, command, initial_output, parameters):
    delay, time_constant, positive_gain, negative_gain, deadzone = parameters
    t = np.asarray(t, dtype=float)
    command = np.asarray(command, dtype=float)
    if t.ndim != 1 or command.ndim != 1 or len(t) < 2 or \
            len(t) != len(command) or np.any(np.diff(t) <= 0.0):
        raise ValueError("invalid channel simulation grid")

    # Propagate the first-order actuator exactly between delayed ZOH command
    # events.  Besides being causal, this keeps the response continuous in the
    # candidate delay, so least_squares can identify delay without replacing a
    # physical step by a non-causal linear ramp.
    tau = max(float(time_constant), 1.0e-5)
    change_indices = np.flatnonzero(command[1:] != command[:-1]) + 1
    event_times = t[change_indices] + float(delay)
    event_targets = mapped_target(
        command[change_indices], positive_gain, negative_gain, deadzone)
    active_target = float(mapped_target(
        command[0], positive_gain, negative_gain, deadzone))
    output = np.empty_like(t)
    output[0] = float(initial_output)
    event_index = 0
    epsilon = max(1.0e-12, 1.0e-12 * float(t[-1] - t[0]))

    def propagate(value, target, duration):
        if duration <= 0.0:
            return value
        decay = math.exp(-duration / tau)
        return target + (value - target) * decay

    for sample_index in range(1, len(t)):
        interval_start = float(t[sample_index - 1])
        interval_end = float(t[sample_index])
        cursor = interval_start
        value = float(output[sample_index - 1])
        while event_index < len(event_times) and \
                float(event_times[event_index]) <= interval_end + epsilon:
            event_time = max(cursor, float(event_times[event_index]))
            value = propagate(value, active_target, event_time - cursor)
            cursor = event_time
            active_target = float(event_targets[event_index])
            event_index += 1
        output[sample_index] = propagate(
            value, active_target, interval_end - cursor)
    return output


def metrics(actual, predicted, mask):
    residual = predicted[mask] - actual[mask]
    centered = actual[mask] - np.mean(actual[mask])
    denominator = float(np.sum(centered * centered))
    return {
        "samples": int(np.sum(mask)),
        "rmse": float(np.sqrt(np.mean(residual * residual))),
        "mae": float(np.mean(np.abs(residual))),
        "p95_abs": float(np.percentile(np.abs(residual), 95.0)),
        "max_abs": float(np.max(np.abs(residual))),
        "r2": float(1.0 - np.sum(residual * residual) / denominator)
        if denominator > 1.0e-15 else None,
        "bias": float(np.mean(residual)),
    }


def fit_channel(grid, command_key, measured_key, mask, channel,
                structure="directional_gain_no_deadzone"):
    t = grid["t"]
    command = grid[command_key]
    measured = grid[measured_key]
    scale = max(float(np.std(measured[mask])), 1.0e-3)
    if channel == "linear":
        full_lower = np.asarray([0.0, 0.003, 0.25, 0.25, 0.0])
        full_upper = np.asarray([0.45, 0.80, 2.0, 2.0, 0.075])
        delay_starts = (0.08, 0.14, 0.20, 0.28)
        tau_starts = (0.025, 0.08, 0.18)
    else:
        full_lower = np.asarray([0.0, 0.003, 0.25, 0.25, 0.0])
        full_upper = np.asarray([0.55, 1.00, 2.0, 2.0, 0.15])
        delay_starts = (0.08, 0.16, 0.24, 0.34)
        tau_starts = (0.025, 0.10, 0.25)

    all_names = [
        "delay_sec", "time_constant_sec", "positive_gain",
        "negative_gain", "deadzone",
    ]
    if structure == "unity_gain_no_deadzone":
        names = all_names[:2]
        lower = full_lower[:2]
        upper = full_upper[:2]

        def decode(values):
            return np.asarray([values[0], values[1], 1.0, 1.0, 0.0])
    elif structure == "directional_gain_no_deadzone":
        names = all_names[:4]
        lower = full_lower[:4]
        upper = full_upper[:4]

        def decode(values):
            return np.asarray(
                [values[0], values[1], values[2], values[3], 0.0])
    elif structure == "directional_gain_with_deadzone":
        names = all_names
        lower = full_lower
        upper = full_upper

        def decode(values):
            return np.asarray(values)
    else:
        raise ValueError("unsupported model structure: " + structure)

    def residual(parameters):
        predicted = simulate_channel(
            t, command, measured[0], decode(parameters))
        return (predicted[mask] - measured[mask]) / scale

    best = None
    for delay in delay_starts:
        for time_constant in tau_starts:
            if structure == "unity_gain_no_deadzone":
                start = np.asarray([delay, time_constant])
            elif structure == "directional_gain_no_deadzone":
                start = np.asarray([delay, time_constant, 1.0, 1.0])
            else:
                start = np.asarray(
                    [delay, time_constant, 1.0, 1.0, 0.01])
            candidate = least_squares(
                residual, start, bounds=(lower, upper),
                max_nfev=500, xtol=1.0e-11, ftol=1.0e-11,
                gtol=1.0e-11,
            )
            if best is None or candidate.cost < best.cost:
                best = candidate
    parameters = decode(best.x)
    predicted = simulate_channel(t, command, measured[0], parameters)
    covariance = None
    standard_error = {name: None for name in all_names}
    dof = int(np.sum(mask)) - len(best.x)
    if dof > 0 and best.jac.size:
        try:
            covariance = np.linalg.pinv(best.jac.T @ best.jac)
            covariance *= 2.0 * best.cost / dof
            for name, value in zip(
                    names, np.sqrt(np.diag(covariance)).tolist()):
                standard_error[name] = float(value)
        except np.linalg.LinAlgError:
            covariance = None
    return {
        "model_structure": structure,
        "parameters": {name: float(value)
                       for name, value in zip(all_names, parameters)},
        "optimistic_standard_error": standard_error,
        "optimizer": {
            "success": bool(best.success),
            "status": int(best.status),
            "message": str(best.message),
            "cost": float(best.cost),
            "optimality": float(best.optimality),
            "active_mask": [int(value) for value in best.active_mask],
        },
        "fit_metrics": metrics(measured, predicted, mask),
        "prediction": predicted,
        "covariance_available": covariance is not None,
    }


def command_plateaus(grid, masks, predicted_v, predicted_w):
    records = []
    core = masks["linear_core"] | masks["angular_core"]
    indices = np.flatnonzero(core)
    if len(indices) == 0:
        return records
    breaks = np.flatnonzero(np.diff(indices) > 1)
    starts = np.r_[indices[0], indices[breaks + 1]]
    ends = np.r_[indices[breaks], indices[-1]]
    rate = grid["sample_rate_hz"]
    tail = max(3, int(round(0.30 * rate)))
    for start, end in zip(starts, ends):
        if end - start + 1 < int(0.5 * rate):
            continue
        is_linear = bool(masks["linear_core"][start])
        command = grid["cmd_v"] if is_linear else grid["cmd_w"]
        actual = grid["mocap_v"] if is_linear else grid["mocap_w"]
        predicted = predicted_v if is_linear else predicted_w
        tail_start = max(int(start), int(end) - tail + 1)
        command_level = float(np.median(command[start:end + 1]))
        actual_level = float(np.median(actual[tail_start:end + 1]))
        records.append({
            "channel": "linear" if is_linear else "angular",
            "start_sec": float(grid["t"][start]),
            "end_sec": float(grid["t"][end]),
            "command": command_level,
            "mocap_steady": actual_level,
            "model_steady": float(np.median(predicted[tail_start:end + 1])),
            "apparent_ratio": actual_level / command_level
            if abs(command_level) > 1.0e-12 else None,
        })
    return records


def json_safe_report(report):
    return json.loads(json.dumps(report, allow_nan=False))


def write_signals(path, grid, masks, linear_prediction, angular_prediction):
    columns = [
        "t_sec", "cmd_v", "cmd_omega", "mocap_v", "mocap_omega",
        "odom_v", "odom_omega", "model_v", "model_omega",
        "linear_fit", "angular_fit", "combined_eval",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for index in range(len(grid["t"])):
            writer.writerow([
                format(float(grid["t"][index]), ".9f"),
                format(float(grid["cmd_v"][index]), ".9g"),
                format(float(grid["cmd_w"][index]), ".9g"),
                format(float(grid["mocap_v"][index]), ".9g"),
                format(float(grid["mocap_w"][index]), ".9g"),
                format(float(grid["odom_v"][index]), ".9g"),
                format(float(grid["odom_w"][index]), ".9g"),
                format(float(linear_prediction[index]), ".9g"),
                format(float(angular_prediction[index]), ".9g"),
                int(masks["linear_fit"][index]),
                int(masks["angular_fit"][index]),
                int(masks["combined_eval"][index]),
            ])


def make_plot(path, grid, masks, linear_prediction, angular_prediction):
    if plt is None:
        return False
    motion = masks["linear_fit"] | masks["angular_fit"] | masks["combined_eval"]
    indices = np.flatnonzero(motion)
    lo = max(0, int(indices[0]) - int(grid["sample_rate_hz"]))
    hi = min(len(motion), int(indices[-1]) + int(grid["sample_rate_hz"]))
    time = grid["t"][lo:hi]
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    axes[0].plot(time, grid["cmd_v"][lo:hi], "k--", lw=1.0,
                 label="cmd v")
    axes[0].plot(time, grid["mocap_v"][lo:hi], color="tab:blue",
                 lw=1.0, label="mocap body v")
    axes[0].plot(time, linear_prediction[lo:hi], color="tab:red",
                 lw=1.2, label="fitted independent model")
    axes[0].set_ylabel("v [m/s]")
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc="upper right")
    axes[1].plot(time, grid["cmd_w"][lo:hi], "k--", lw=1.0,
                 label="cmd omega")
    axes[1].plot(time, grid["mocap_w"][lo:hi], color="tab:blue",
                 lw=1.0, label="mocap yaw rate")
    axes[1].plot(time, angular_prediction[lo:hi], color="tab:red",
                 lw=1.2, label="fitted independent model")
    axes[1].set_xlabel("seconds from sequence start")
    axes[1].set_ylabel("omega [rad/s]")
    axes[1].grid(alpha=0.25)
    axes[1].legend(loc="upper right")
    fig.suptitle("planar Scout execution identification (S-turns are evaluation only)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return True


def format_metric(metric):
    r2 = "n/a" if metric["r2"] is None else f'{metric["r2"]:.4f}'
    return (
        f'RMSE={metric["rmse"]:.5f}, P95={metric["p95_abs"]:.5f}, '
        f'max={metric["max_abs"]:.5f}, R2={r2}'
    )


def write_markdown(path, report):
    linear = report["fits"]["linear"]
    angular = report["fits"]["angular"]
    combined = report["combined_s_turn_evaluation"]
    lines = [
        "# planar_r03 双通道执行参数初步辨识",
        "",
        f'- 工具合同：`{TOOL_SCHEMA}`',
        f'- 命令保持合同：`{COMMAND_HOLD_CONTRACT}`',
        f'- 源 bag：`{report["source"]["bag"]}`',
        f'- SHA-256：`{report["source"]["sha256"]}`',
        "- 证据等级：**single-trial development candidate；不是实车 formal 参数，也不是 held-out 验证**",
        "",
        "## 初步结果",
        "",
        "| 通道 | delay [s] | tau [s] | K+ | K- | deadzone | 纯通道拟合 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for name, fit in (("linear", linear), ("angular", angular)):
        params = fit["parameters"]
        lines.append(
            f'| {name} | {params["delay_sec"]:.5f} | '
            f'{params["time_constant_sec"]:.5f} | '
            f'{params["positive_gain"]:.5f} | '
            f'{params["negative_gain"]:.5f} | '
            f'{params["deadzone"]:.5f} | '
            f'{format_metric(fit["fit_metrics"])} |'
        )
    lines += [
        "",
        "## 模型结构敏感性",
        "",
        "| 通道 | 结构 | delay [s] | tau [s] | K+ | K- | deadzone | RMSE |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for channel in ("linear", "angular"):
        for fit in report["model_structure_sensitivity"][channel]:
            params = fit["parameters"]
            lines.append(
                f'| {channel} | {fit["model_structure"]} | '
                f'{params["delay_sec"]:.5f} | '
                f'{params["time_constant_sec"]:.5f} | '
                f'{params["positive_gain"]:.5f} | '
                f'{params["negative_gain"]:.5f} | '
                f'{params["deadzone"]:.5f} | '
                f'{fit["fit_metrics"]["rmse"]:.5f} |'
            )
    lines += [
        "",
        "## 组合 S 形段（不参与拟合）",
        "",
        f'- 线通道：{format_metric(combined["linear"])}',
        f'- 角通道：{format_metric(combined["angular"])}',
        "",
        "## 可辨识性与使用限制",
        "",
        "- 只有一个 trial，无法估计跨 trial、电量、载荷、地面和重复安装漂移。",
        "- 线通道只有 ±0.10/±0.15 m/s，角通道只有 ±0.20/±0.40 rad/s；deadzone、方向增益与 tau 存在相关性。",
        "- 未触及平台饱和边界，因此 saturation 不可由本数据辨识。",
        "- S 形段只检查独立双通道模型的组合外推；不能把它重新用于调参后再声称 held-out。",
        "- 协方差标准误只反映局部最小二乘曲率，忽略有色噪声与单 trial 系统误差，不能当正式置信区间。",
        "- 本结果可用于仿真开发候选；实物 `enforce` 必须继续关闭。",
        "",
        "## 后续精确辨识",
        "",
        "需要按完整 trial 分离 D_id/D_fid，增加多幅值 PRBS/阶跃、正反向、载荷、电量和地面分层；以最终 `u_pub`、mocap source time 和 driver/odom 时间轴联合拟合，并在独立 trial 报告 RMSE/P95/max 与参数漂移。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def identify(args):
    bag_path = Path(args.bag).resolve()
    out_dir = Path(args.out_dir).resolve()
    if not bag_path.is_file():
        raise RuntimeError(f"bag does not exist: {bag_path}")
    source_mount = mount_root(bag_path)
    try:
        out_dir.relative_to(source_mount)
        output_on_source_mount = True
    except ValueError:
        output_on_source_mount = False
    if source_mount != Path("/") and output_on_source_mount:
        raise RuntimeError("output directory must not be on the source disk")
    if out_dir.exists():
        raise RuntimeError(
            "output directory already exists; identification evidence is "
            "create-new only")
    out_dir.mkdir(parents=True, exist_ok=False)
    signals = load_bag(bag_path)
    require_streams(signals)
    grid = make_grid(signals, args.sample_rate_hz, args.smooth_width_sec)
    masks = build_masks(grid)
    if np.sum(masks["linear_fit"]) < 100 or np.sum(masks["angular_fit"]) < 100:
        raise RuntimeError("insufficient isolated channel excitation")
    structures = (
        "unity_gain_no_deadzone",
        "directional_gain_no_deadzone",
        "directional_gain_with_deadzone",
    )
    linear_variants = [
        fit_channel(
            grid, "cmd_v", "mocap_v", masks["linear_fit"], "linear",
            structure)
        for structure in structures
    ]
    angular_variants = [
        fit_channel(
            grid, "cmd_w", "mocap_w", masks["angular_fit"], "angular",
            structure)
        for structure in structures
    ]
    # The middle structure is the most complex model this single trial can
    # support without confounding deadzone with gain, tau and delay.
    linear_fit = linear_variants[1]
    angular_fit = angular_variants[1]
    linear_prediction = linear_fit.pop("prediction")
    angular_prediction = angular_fit.pop("prediction")
    for fit in linear_variants:
        fit.pop("prediction", None)
    for fit in angular_variants:
        fit.pop("prediction", None)
    combined_mask = masks["combined_eval"]
    if np.sum(combined_mask) == 0:
        raise RuntimeError("combined-channel evaluation segment is missing")
    report = {
        "schema": TOOL_SCHEMA,
        "model_schema": MODEL_SCHEMA,
        "evidence_level": "single_trial_development_candidate",
        "formal_robot_release_allowed": False,
        "source": {
            "bag": str(bag_path),
            "sha256": sha256_file(bag_path),
            "size_bytes": int(bag_path.stat().st_size),
            "bag_start_sec": signals["bag_start"],
            "bag_end_sec": signals["bag_end"],
            "streams": {
                "cmd": stream_stats(signals["cmd_t"]),
                "mocap_pose": stream_stats(signals["pose_t"]),
                "odom": stream_stats(signals["odom_t"]),
                "segment_events": {"count": len(signals["events"])},
            },
        },
        "preprocessing": {
            "sample_rate_hz": grid["sample_rate_hz"],
            "dt_sec": grid["dt"],
            "command_hold_contract": COMMAND_HOLD_CONTRACT,
            "channel_simulation": (
                "exact_first_order_propagation_between_delayed_zoh_events"
            ),
            "mocap_velocity_method": "zero_phase_savgol_pose_derivative_body_projection",
            "savgol_window_samples": grid["savgol_window_samples"],
            "savgol_width_sec": grid["savgol_width_sec"],
            "linear_fit_samples": int(np.sum(masks["linear_fit"])),
            "angular_fit_samples": int(np.sum(masks["angular_fit"])),
            "combined_eval_samples": int(np.sum(combined_mask)),
        },
        "fits": {"linear": linear_fit, "angular": angular_fit},
        "model_structure_sensitivity": {
            "linear": linear_variants,
            "angular": angular_variants,
            "selected_structure": "directional_gain_no_deadzone",
            "selection_reason": (
                "deadzone is weakly identifiable from only two nonzero "
                "magnitudes; keep directional gains but fix deadzone to zero"
            ),
        },
        "combined_s_turn_evaluation": {
            "used_for_fit": False,
            "linear": metrics(
                grid["mocap_v"], linear_prediction, combined_mask),
            "angular": metrics(
                grid["mocap_w"], angular_prediction, combined_mask),
        },
        "odom_cross_check": {
            "linear_fit_mask": metrics(
                grid["odom_v"], linear_prediction, masks["linear_fit"]),
            "angular_fit_mask": metrics(
                grid["odom_w"], angular_prediction, masks["angular_fit"]),
            "note": "wheel odom is a cross-check, not the identification truth",
        },
        "plateaus": command_plateaus(
            grid, masks, linear_prediction, angular_prediction),
        "identifiability": {
            "trial_count": 1,
            "held_out_trial_count": 0,
            "saturation_identified": False,
            "deadzone_strength": "weak; only two nonzero magnitudes per channel",
            "cross_condition_stability_identified": False,
        },
    }
    report = json_safe_report(report)
    json_path = out_dir / "planar_r03_preliminary_identification.json"
    json_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_markdown(out_dir / "planar_r03_preliminary_identification.md", report)
    write_signals(
        out_dir / "planar_r03_resampled_signals.csv", grid, masks,
        linear_prediction, angular_prediction,
    )
    plot_written = make_plot(
        out_dir / "planar_r03_fit_overlay.png", grid, masks,
        linear_prediction, angular_prediction,
    )
    manifest = {
        "schema": "spmpc_execution_identification_output_manifest_v1",
        "source_bag": str(bag_path),
        "source_sha256": report["source"]["sha256"],
        "artifacts": {},
    }
    for artifact in sorted(out_dir.iterdir()):
        if artifact.is_file() and artifact.name != "manifest.json":
            manifest["artifacts"][artifact.name] = {
                "bytes": artifact.stat().st_size,
                "sha256": sha256_file(artifact),
            }
    manifest["plot_written"] = plot_written
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--sample-rate-hz", type=float, default=100.0)
    parser.add_argument("--smooth-width-sec", type=float, default=0.15)
    args = parser.parse_args()
    if not np.isfinite(args.sample_rate_hz) or args.sample_rate_hz < 20.0:
        parser.error("--sample-rate-hz must be finite and >= 20")
    if not np.isfinite(args.smooth_width_sec) or not (
            0.05 <= args.smooth_width_sec <= 0.5):
        parser.error("--smooth-width-sec must be in [0.05, 0.5]")
    try:
        report = identify(args)
    except Exception as exc:  # pylint: disable=broad-except
        print(f"identification failed: {exc}", file=sys.stderr)
        return 1
    for channel in ("linear", "angular"):
        params = report["fits"][channel]["parameters"]
        metric = report["fits"][channel]["fit_metrics"]
        print(
            f"{channel}: delay={params['delay_sec']:.5f}s "
            f"tau={params['time_constant_sec']:.5f}s "
            f"K+={params['positive_gain']:.4f} "
            f"K-={params['negative_gain']:.4f} "
            f"deadzone={params['deadzone']:.5f} "
            f"rmse={metric['rmse']:.5f}"
        )
    print(f"artifacts: {Path(args.out_dir).resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
