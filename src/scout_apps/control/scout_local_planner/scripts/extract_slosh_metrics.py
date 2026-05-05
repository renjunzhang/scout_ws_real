#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import bisect
import csv
import math
import os
import re
import statistics
import sys
from collections import defaultdict

import rosbag

STATUS_BUCKETS = ("TRACKING", "SETTLING", "REACHED", "IDLE")
KNOWN_CONDITIONS = (
    "FAS_Q5_DKAPPA_CAP",
    "FAS_Q5_SPEED_CAP",
    "FAS_Q5_TERM",
    "FAS_Q5_DOT",
    "SMOOTH_DOMEGA",
    "SMOOTH_CTRL",
    "DKAPPA_CAP",
    "SPEED_CAP",
    "GOV_AY",
    "AY_COST",
    "ENERGY_WIN",
    "PROFILE_WINDOW",
    "PROFILE_RISK",
    "PROFILE_REF_V2",
    "PROFILE_ENERGY_GEO",
    "PROFILE_ENERGY",
    "OUTPUT_GUARD",
    "PMG_COMBINED",
    "PMG_LONG",
    "PMG_LAT",
    "PMG",
    "PROFILE_SELECTIVE",
    "PROFILE_SAFE",
    "PROP_Q5",
    "FAS_Q10",
    "FAS_Q5",
    "NOM",
    "ISR",
    "CUSTOM",
)
D0_GROUP_METRICS = (
    "tracking_time_s",
    "height_rms_m",
    "height_p95_m",
    "height_max_m",
    "eta_dot_norm_rms_mps",
    "modal_energy_norm_rms",
    "odom_ay_abs_p95",
    "odom_kappa_abs_p95",
    "track_dist_p95_m",
    "solve_success_ratio",
    "governor_ay_active_ratio",
    "gov_ay_first_to_height_peak_s",
    "gov_ay_first_to_eta_dot_peak_s",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="提取 slosh/MPC 实验 bag 指标，默认仅统计 TRACKING 且 terminal/mode==NONE 段。"
    )
    parser.add_argument("inputs", nargs="+", help="bag 文件或包含 bag 的目录")
    parser.add_argument("--csv", dest="csv_path", default="", help="可选：导出汇总 CSV 路径")
    parser.add_argument("--all-segments", action="store_true", help="统计整包而不是仅 TRACKING 段")
    parser.add_argument(
        "--tracking-with-terminal",
        action="store_true",
        help="旧口径：仅按 /mpc_status==TRACKING 过滤，不排除 terminal recovery",
    )
    parser.add_argument("--per-episode", action="store_true", help="额外输出每个 TRACKING episode 的明细")
    parser.add_argument("--lag-window-s", type=float, default=2.0, help="相位诊断最大滞后窗口，单位 s")
    parser.add_argument("--lag-step-s", type=float, default=0.05, help="相位诊断滞后扫描步长，单位 s")
    parser.add_argument("--group-summary", action="store_true", help="按 bag 名中的条件分组，并输出相对基线变化")
    parser.add_argument("--baseline-condition", default="NOM", help="--group-summary 使用的基线条件，默认 NOM")
    return parser.parse_args()


def expand_inputs(paths):
    bag_paths = []
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.endswith(".bag"):
                    bag_paths.append(os.path.join(path, name))
        elif os.path.isfile(path) and path.endswith(".bag"):
            bag_paths.append(path)
    return sorted(dict.fromkeys(bag_paths))


def rms(values):
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def safe_mean(values):
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return float("nan")
    return statistics.mean(values)


def safe_max(values):
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return float("nan")
    return max(values)


def percentile(values, p):
    values = sorted(v for v in values if not math.isnan(v))
    if not values:
        return float("nan")
    index = max(0, min(len(values) - 1, int(p / 100.0 * (len(values) - 1))))
    return values[index]


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def path_points(msg):
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def nearest_value(series, ts, start_idx=0):
    if not series:
        return start_idx, None
    idx = min(start_idx, len(series) - 1)
    while idx + 1 < len(series) and abs(series[idx + 1][0] - ts) <= abs(series[idx][0] - ts):
        idx += 1
    return idx, series[idx][1]


def nearest_path_error(pose_xy, pose_yaw, points):
    if len(points) < 2:
        return None
    px, py = pose_xy
    best_dist = float("inf")
    best_heading = None
    for idx in range(len(points) - 1):
        ax, ay = points[idx]
        bx, by = points[idx + 1]
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-12:
            continue
        apx = px - ax
        apy = py - ay
        u = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
        qx = ax + u * abx
        qy = ay + u * aby
        dist = math.hypot(px - qx, py - qy)
        if dist < best_dist:
            best_dist = dist
            best_heading = math.atan2(aby, abx)
    if best_heading is None:
        return None
    return best_dist, abs(wrap_to_pi(pose_yaw - best_heading))


def transform_pose_2d(pose, transform):
    tx, ty, yaw = transform
    c = math.cos(yaw)
    s = math.sin(yaw)
    x = tx + c * pose["x"] - s * pose["y"]
    y = ty + s * pose["x"] + c * pose["y"]
    return {
        "x": x,
        "y": y,
        "yaw": wrap_to_pi(pose["yaw"] + yaw),
        "vx": pose["vx"],
        "wz": pose["wz"],
    }


def pearson(xs, ys):
    if len(xs) < 5 or len(xs) != len(ys):
        return float("nan")
    mx = safe_mean(xs)
    my = safe_mean(ys)
    dx = [x - mx for x in xs]
    dy = [y - my for y in ys]
    sx = math.sqrt(sum(x * x for x in dx))
    sy = math.sqrt(sum(y * y for y in dy))
    if sx <= 1e-12 or sy <= 1e-12:
        return float("nan")
    return sum(x * y for x, y in zip(dx, dy)) / (sx * sy)


def interpolate_series(series, ts):
    if not series:
        return float("nan")
    times = [item[0] for item in series]
    values = [item[1] for item in series]
    if ts < times[0] or ts > times[-1]:
        return float("nan")
    index = bisect.bisect_left(times, ts)
    if index == 0:
        return values[0]
    if index >= len(series):
        return values[-1]
    t0, v0 = times[index - 1], values[index - 1]
    t1, v1 = times[index], values[index]
    if t1 <= t0:
        return v0
    ratio = (ts - t0) / (t1 - t0)
    return v0 + ratio * (v1 - v0)


def best_lag_corr(driver, response, max_lag_s=2.0, step_s=0.05):
    """Positive lag means response follows driver by lag seconds."""
    if len(driver) < 5 or len(response) < 5 or step_s <= 0.0:
        return float("nan"), float("nan")

    driver = sorted(driver)
    response = sorted(response)
    best_lag = float("nan")
    best_corr = float("nan")
    best_abs_corr = -1.0
    steps = int(round(max_lag_s / step_s))

    for step in range(-steps, steps + 1):
        lag = step * step_s
        start = max(driver[0][0], response[0][0] - lag)
        end = min(driver[-1][0], response[-1][0] - lag)
        if end <= start:
            continue

        xs = []
        ys = []
        count = int((end - start) / step_s) + 1
        for idx in range(count):
            ts = start + idx * step_s
            x = interpolate_series(driver, ts)
            y = interpolate_series(response, ts + lag)
            if not math.isnan(x) and not math.isnan(y):
                xs.append(x)
                ys.append(y)

        corr = pearson(xs, ys)
        if math.isnan(corr):
            continue
        if abs(corr) > best_abs_corr:
            best_abs_corr = abs(corr)
            best_lag = lag
            best_corr = corr

    return best_lag, best_corr


def add_lag_metrics(row, series, max_lag_s, step_s):
    pairs = {
        "ay_to_eta_y": ("ay", "eta_y"),
        "ay_to_eta_y_dot": ("ay", "eta_y_dot"),
        "alpha_to_eta_x": ("alpha", "eta_x"),
        "alpha_to_eta_x_dot": ("alpha", "eta_x_dot"),
        "abs_ay_to_height": ("abs_ay", "height"),
        "abs_alpha_to_height": ("abs_alpha", "height"),
        "abs_ay_to_energy": ("abs_ay", "modal_energy_norm"),
        "abs_alpha_to_energy": ("abs_alpha", "modal_energy_norm"),
    }
    for name, (driver_key, response_key) in pairs.items():
        lag, corr = best_lag_corr(
            series[driver_key],
            series[response_key],
            max_lag_s=max_lag_s,
            step_s=step_s,
        )
        row[f"{name}_lag_s"] = round(lag, 3) if not math.isnan(lag) else float("nan")
        row[f"{name}_corr"] = round(corr, 3) if not math.isnan(corr) else float("nan")


def time_of_max_abs(series):
    if not series:
        return float("nan")
    ts, _ = max(series, key=lambda item: abs(item[1]))
    return ts


def first_active_time(series):
    for ts, value in series:
        if int(value) == 1:
            return ts
    return float("nan")


def add_governor_timing_metrics(row, series, start_time):
    gov_ay_first = first_active_time(series["governor_ay"])
    height_peak = time_of_max_abs(series["height"])
    eta_dot_peak = time_of_max_abs(series["eta_dot_norm"])
    odom_ay_peak = time_of_max_abs(series["odom_ay_abs"])

    row["gov_ay_first_active_s"] = (
        round(gov_ay_first - start_time, 3) if not math.isnan(gov_ay_first) else float("nan")
    )
    row["height_peak_time_s"] = (
        round(height_peak - start_time, 3) if not math.isnan(height_peak) else float("nan")
    )
    row["eta_dot_peak_time_s"] = (
        round(eta_dot_peak - start_time, 3) if not math.isnan(eta_dot_peak) else float("nan")
    )
    row["odom_ay_peak_time_s"] = (
        round(odom_ay_peak - start_time, 3) if not math.isnan(odom_ay_peak) else float("nan")
    )
    row["gov_ay_first_to_height_peak_s"] = (
        round(height_peak - gov_ay_first, 3)
        if not math.isnan(gov_ay_first) and not math.isnan(height_peak)
        else float("nan")
    )
    row["gov_ay_first_to_eta_dot_peak_s"] = (
        round(eta_dot_peak - gov_ay_first, 3)
        if not math.isnan(gov_ay_first) and not math.isnan(eta_dot_peak)
        else float("nan")
    )
    row["gov_ay_first_to_odom_ay_peak_s"] = (
        round(odom_ay_peak - gov_ay_first, 3)
        if not math.isnan(gov_ay_first) and not math.isnan(odom_ay_peak)
        else float("nan")
    )


def q_from_name(path):
    match = re.search(r"Q([0-9]+(?:\.[0-9]+)?)", os.path.basename(path))
    return match.group(1) if match else ""


def condition_from_name(path):
    name = os.path.basename(path)
    for condition in KNOWN_CONDITIONS:
        if condition in name:
            return condition
    return "UNKNOWN"


def get_string_segments(bag_path, topic):
    transitions = []
    with rosbag.Bag(bag_path) as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        for _, msg, t in bag.read_messages(topics=[topic]):
            status = str(msg.data)
            ts = t.to_sec()
            if not transitions or transitions[-1][1] != status:
                transitions.append((ts, status))

    segments = []
    for idx, (ts, status) in enumerate(transitions):
        end_ts = transitions[idx + 1][0] if idx + 1 < len(transitions) else end_time
        segments.append((ts, end_ts, status))

    return start_time, end_time, transitions, segments


def get_status_segments(bag_path):
    return get_string_segments(bag_path, "/mpc_status")


def in_tracking(segments, ts):
    for start, end, status in segments:
        if status == "TRACKING" and start <= ts < end:
            return True
    return False


def in_terminal_none(segments, ts):
    if not segments:
        return True
    for start, end, status in segments:
        if start <= ts < end:
            return status == "NONE"
    return False


def status_at(segments, ts):
    for start, end, status in segments:
        if start <= ts < end:
            return status
    if segments and ts >= segments[-1][1]:
        return segments[-1][2]
    return "UNKNOWN"


def strict_tracking_duration(mpc_segments, terminal_segments):
    total = 0.0
    for mpc_start, mpc_end, mpc_status in mpc_segments:
        if mpc_status != "TRACKING":
            continue
        if not terminal_segments:
            total += max(0.0, mpc_end - mpc_start)
            continue
        for term_start, term_end, term_status in terminal_segments:
            if term_status != "NONE":
                continue
            start = max(mpc_start, term_start)
            end = min(mpc_end, term_end)
            if end > start:
                total += end - start
    return total


def summarize_status_buckets(samples):
    summary = {}
    for status, values in samples.items():
        total = len(values)
        if total <= 0:
            continue
        solved = sum(1 for value in values if value == 1)
        summary[status] = {
            "count": total,
            "solve_fail_count": total - solved,
            "success_ratio": round(solved / total, 3),
        }
    return summary


def format_status_breakdown(summary):
    parts = []
    seen = set()
    for status in STATUS_BUCKETS:
        if status not in summary:
            continue
        seen.add(status)
        item = summary[status]
        parts.append(
            f"{status}: n={item['count']} success={item['success_ratio']:.3f} fail={item['solve_fail_count']}"
        )
    for status in sorted(summary.keys()):
        if status in seen:
            continue
        item = summary[status]
        parts.append(
            f"{status}: n={item['count']} success={item['success_ratio']:.3f} fail={item['solve_fail_count']}"
        )
    return " | ".join(parts)


def collect_metrics(
    bag_path,
    tracking_only=True,
    strict_terminal_none=True,
    lag_window_s=2.0,
    lag_step_s=0.05,
):
    start_time, end_time, transitions, segments = get_status_segments(bag_path)
    _, _, terminal_transitions, terminal_segments = get_string_segments(bag_path, "/terminal/mode")

    metrics = defaultdict(list)
    series = defaultdict(list)
    odom_samples = []
    map_odom_tf = []
    odom_base_tf = []
    reference_paths = []
    status_val_by_status = defaultdict(list)
    goals = []
    global_path_count = 0
    episodes = set()
    metadata_topics = {"/slosh/episode_id", "/scout/goal", "/scout/global_path"}
    prev_cmd_ts = None
    prev_cmd_vx = None
    prev_cmd_wz = None
    prev_odom_ts = None
    prev_odom_vx = None
    prev_odom_ax = None

    topics = [
        "/slosh/state",
        "/slosh/height",
        "/slosh/height_pred_max",
        "/slosh/modal_energy",
        "/slosh/modal_energy_norm",
        "/slosh/ay_est",
        "/slosh/alpha_est",
        "/mpc/solve_ms",
        "/mpc/status_val",
        "/slosh/speed_governor_active",
        "/slosh/speed_governor_ay_active",
        "/slosh/speed_cap_active",
        "/slosh/speed_cap_v_limit",
        "/slosh/output_guard_active",
        "/slosh/output_guard_ay_limit",
        "/slosh/v_des_eff",
        "/slosh/constraint_active",
        "/reference/v_ref",
        "/reference/v_path",
        "/reference/kappa",
        "/reference/s",
        "/reference/implied_ax",
        "/reference/implied_ay",
        "/reference/implied_jerk",
        "/reference/implied_ax_abs_p95",
        "/reference/implied_ay_abs_p95",
        "/reference/implied_jerk_abs_p95",
        "/slosh/episode_id",
        "/cmd_vel",
        "/odom",
        "/tf",
        "/mpc/reference_path",
        "/scout/goal",
        "/scout/global_path",
        "/terminal/mode",
    ]

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            if tracking_only and topic not in metadata_topics:
                keep = in_tracking(segments, ts)
                if strict_terminal_none:
                    keep = keep and in_terminal_none(terminal_segments, ts)
                if not keep:
                    continue

            if topic == "/slosh/height":
                value = float(msg.data)
                metrics["height"].append(value)
                series["height"].append((ts, value))
            elif topic == "/slosh/state":
                if len(msg.data) >= 4:
                    eta_x = float(msg.data[0])
                    eta_x_dot = float(msg.data[1])
                    eta_y = float(msg.data[2])
                    eta_y_dot = float(msg.data[3])
                    eta_norm = math.hypot(eta_x, eta_y)
                    eta_dot_norm = math.hypot(eta_x_dot, eta_y_dot)
                    metrics["eta_norm"].append(eta_norm)
                    metrics["eta_dot_norm"].append(eta_dot_norm)
                    series["eta_dot_norm"].append((ts, eta_dot_norm))
                    series["eta_x"].append((ts, eta_x))
                    series["eta_x_dot"].append((ts, eta_x_dot))
                    series["eta_y"].append((ts, eta_y))
                    series["eta_y_dot"].append((ts, eta_y_dot))
            elif topic == "/slosh/height_pred_max":
                metrics["pred"].append(float(msg.data))
            elif topic == "/slosh/modal_energy":
                metrics["modal_energy"].append(float(msg.data))
            elif topic == "/slosh/modal_energy_norm":
                value = float(msg.data)
                metrics["modal_energy_norm"].append(value)
                series["modal_energy_norm"].append((ts, value))
            elif topic == "/slosh/ay_est":
                value = float(msg.data)
                metrics["excitation_ay_abs"].append(abs(value))
                series["ay"].append((ts, value))
                series["abs_ay"].append((ts, abs(value)))
            elif topic == "/slosh/alpha_est":
                value = float(msg.data)
                metrics["excitation_alpha_abs"].append(abs(value))
                series["alpha"].append((ts, value))
                series["abs_alpha"].append((ts, abs(value)))
            elif topic == "/mpc/solve_ms":
                metrics["solve_ms"].append(float(msg.data))
            elif topic == "/mpc/status_val":
                value = int(msg.data)
                metrics["status_val"].append(value)
                status_val_by_status[status_at(segments, ts)].append(value)
            elif topic == "/slosh/speed_governor_active":
                metrics["governor"].append(int(msg.data))
            elif topic == "/slosh/speed_governor_ay_active":
                value = int(msg.data)
                metrics["governor_ay"].append(value)
                series["governor_ay"].append((ts, value))
            elif topic == "/slosh/speed_cap_active":
                metrics["speed_cap_active"].append(int(msg.data))
            elif topic == "/slosh/speed_cap_v_limit":
                metrics["speed_cap_v_limit"].append(float(msg.data))
            elif topic == "/slosh/output_guard_active":
                metrics["output_guard_active"].append(int(msg.data))
            elif topic == "/slosh/output_guard_ay_limit":
                metrics["output_guard_ay_limit"].append(float(msg.data))
            elif topic == "/slosh/v_des_eff":
                metrics["v_des_eff"].append(float(msg.data))
            elif topic == "/slosh/constraint_active":
                metrics["constraint"].append(int(msg.data))
            elif topic == "/reference/v_ref":
                metrics["ref_v_ref"].append(float(msg.data))
            elif topic == "/reference/v_path":
                metrics["ref_v_path"].append(float(msg.data))
            elif topic == "/reference/kappa":
                metrics["ref_kappa_abs"].append(abs(float(msg.data)))
            elif topic == "/reference/s":
                metrics["ref_s"].append(float(msg.data))
            elif topic == "/reference/implied_ax":
                metrics["ref_implied_ax_abs"].append(abs(float(msg.data)))
            elif topic == "/reference/implied_ay":
                metrics["ref_implied_ay_abs"].append(abs(float(msg.data)))
            elif topic == "/reference/implied_jerk":
                metrics["ref_implied_jerk_abs"].append(abs(float(msg.data)))
            elif topic == "/reference/implied_ax_abs_p95":
                metrics["ref_implied_ax_abs_p95_horizon"].append(float(msg.data))
            elif topic == "/reference/implied_ay_abs_p95":
                metrics["ref_implied_ay_abs_p95_horizon"].append(float(msg.data))
            elif topic == "/reference/implied_jerk_abs_p95":
                metrics["ref_implied_jerk_abs_p95_horizon"].append(float(msg.data))
            elif topic == "/slosh/episode_id":
                episodes.add(int(msg.data))
            elif topic == "/cmd_vel":
                vx = float(msg.linear.x)
                wz = float(msg.angular.z)
                metrics["vx"].append(vx)
                metrics["wz"].append(wz)
                if prev_cmd_ts is not None:
                    dt = ts - prev_cmd_ts
                    if dt > 1e-6:
                        metrics["cmd_dvx"].append((vx - prev_cmd_vx) / dt)
                        metrics["cmd_dwz"].append((wz - prev_cmd_wz) / dt)
                prev_cmd_ts = ts
                prev_cmd_vx = vx
                prev_cmd_wz = wz
            elif topic == "/odom":
                vx = float(msg.twist.twist.linear.x)
                wz = float(msg.twist.twist.angular.z)
                ay = vx * wz
                metrics["odom_vx"].append(vx)
                metrics["odom_wz"].append(wz)
                metrics["odom_ay"].append(ay)
                metrics["odom_ay_abs"].append(abs(ay))
                series["odom_ay_abs"].append((ts, abs(ay)))
                if prev_odom_ts is not None:
                    dt = ts - prev_odom_ts
                    if dt > 1e-6:
                        ax = (vx - prev_odom_vx) / dt
                        metrics["odom_ax_abs"].append(abs(ax))
                        if prev_odom_ax is not None:
                            metrics["odom_jerk_abs"].append(abs((ax - prev_odom_ax) / dt))
                        prev_odom_ax = ax
                prev_odom_ts = ts
                prev_odom_vx = vx
                if abs(vx) > 0.05:
                    metrics["odom_kappa_abs"].append(abs(wz / vx))
                odom_samples.append((ts, {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": yaw_from_quat(msg.pose.pose.orientation),
                    "vx": vx,
                    "wz": wz,
                }))
            elif topic == "/tf":
                for transform in msg.transforms:
                    if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                        tr = transform.transform.translation
                        rot = transform.transform.rotation
                        map_odom_tf.append((ts, (float(tr.x), float(tr.y), yaw_from_quat(rot))))
                    elif (
                        transform.header.frame_id == "odom" and
                        transform.child_frame_id in ("base_footprint", "base_link")
                    ):
                        tr = transform.transform.translation
                        rot = transform.transform.rotation
                        odom_base_tf.append((ts, {
                            "x": float(tr.x),
                            "y": float(tr.y),
                            "yaw": yaw_from_quat(rot),
                            "vx": 0.0,
                            "wz": 0.0,
                        }))
            elif topic == "/mpc/reference_path":
                reference_paths.append((ts, path_points(msg)))
            elif topic == "/scout/goal":
                p = msg.pose.position
                goals.append((round(p.x, 3), round(p.y, 3)))
            elif topic == "/scout/global_path":
                global_path_count += 1

    tracking_segments = [seg for seg in segments if seg[2] == "TRACKING"]
    tracking_time = (
        strict_tracking_duration(segments, terminal_segments)
        if tracking_only and strict_terminal_none
        else sum(end - start for start, end, _ in tracking_segments)
    )
    reached_count = sum(
        1 for index, (_, status) in enumerate(transitions) if index > 0 and status == "REACHED"
    )
    governor_on = sum(1 for x in metrics["governor"] if x == 1)
    governor_total = len(metrics["governor"])
    governor_ay_on = sum(1 for x in metrics["governor_ay"] if x == 1)
    governor_ay_total = len(metrics["governor_ay"])
    speed_cap_on = sum(1 for x in metrics["speed_cap_active"] if x == 1)
    speed_cap_total = len(metrics["speed_cap_active"])
    output_guard_on = sum(1 for x in metrics["output_guard_active"] if x == 1)
    output_guard_total = len(metrics["output_guard_active"])
    status_breakdown = summarize_status_buckets(status_val_by_status)
    ref_idx = 0
    map_tf_idx = 0
    base_tf_idx = 0
    for ts, odom_state in odom_samples:
        ref_idx, ref_pts = nearest_value(reference_paths, ts, ref_idx)
        if not ref_pts:
            continue
        pose = odom_state
        map_tf_idx, map_transform = nearest_value(map_odom_tf, ts, map_tf_idx)
        base_tf_idx, base_pose = nearest_value(odom_base_tf, ts, base_tf_idx)
        if map_transform is not None and base_pose is not None:
            pose = transform_pose_2d(base_pose, map_transform)
        elif map_transform is not None:
            pose = transform_pose_2d(odom_state, map_transform)
        sample = nearest_path_error(
            (pose["x"], pose["y"]),
            pose["yaw"],
            ref_pts,
        )
        if sample is None:
            continue
        dist, heading_err = sample
        metrics["track_dist"].append(dist)
        metrics["track_heading_err_deg"].append(math.degrees(heading_err))

    row = {
        "bag_path": bag_path,
        "bag_name": os.path.basename(bag_path),
        "condition": condition_from_name(bag_path),
        "Q_label": q_from_name(bag_path),
        "mode": (
            "tracking_terminal_none"
            if tracking_only and strict_terminal_none
            else ("tracking_only" if tracking_only else "whole_bag")
        ),
        "duration_s": round(end_time - start_time, 3),
        "tracking_time_s": round(tracking_time, 3),
        "episode_count": len(episodes),
        "goal_count": len(goals),
        "global_path_count": global_path_count,
        "reached_count": reached_count,
        "height_rms_m": round(rms(metrics["height"]), 6),
        "height_p95_m": round(percentile(metrics["height"], 95), 6),
        "height_max_m": round(safe_max(metrics["height"]), 6),
        "height_pred_rms_m": round(rms(metrics["pred"]), 6),
        "height_pred_max_m": round(safe_max(metrics["pred"]), 6),
        "eta_norm_rms_m": round(rms(metrics["eta_norm"]), 6),
        "eta_dot_norm_rms_mps": round(rms(metrics["eta_dot_norm"]), 6),
        "modal_energy_rms": round(rms(metrics["modal_energy"]), 6),
        "modal_energy_norm_rms": round(rms(metrics["modal_energy_norm"]), 6),
        "excitation_ay_abs_mean": round(safe_mean(metrics["excitation_ay_abs"]), 3),
        "excitation_alpha_abs_mean": round(safe_mean(metrics["excitation_alpha_abs"]), 3),
        "solve_ms_mean": round(safe_mean(metrics["solve_ms"]), 3),
        "solve_ms_max": round(safe_max(metrics["solve_ms"]), 3),
        "solve_fail_count": sum(1 for x in metrics["status_val"] if x != 1),
        "solve_success_ratio": round(
            sum(1 for x in metrics["status_val"] if x == 1) / len(metrics["status_val"]), 3
        ) if metrics["status_val"] else float("nan"),
        "constraint_active_count": sum(1 for x in metrics["constraint"] if x == 1),
        "governor_active_count": governor_on,
        "governor_active_ratio": round(governor_on / governor_total, 3) if governor_total else 0.0,
        "governor_ay_active_count": governor_ay_on,
        "governor_ay_active_ratio": round(governor_ay_on / governor_ay_total, 3) if governor_ay_total else 0.0,
        "speed_cap_active_count": speed_cap_on,
        "speed_cap_active_ratio": round(speed_cap_on / speed_cap_total, 3) if speed_cap_total else 0.0,
        "speed_cap_v_limit_mean": round(safe_mean(metrics["speed_cap_v_limit"]), 3),
        "speed_cap_v_limit_min": round(min(metrics["speed_cap_v_limit"]), 3) if metrics["speed_cap_v_limit"] else float("nan"),
        "output_guard_active_count": output_guard_on,
        "output_guard_active_ratio": round(output_guard_on / output_guard_total, 3) if output_guard_total else 0.0,
        "output_guard_ay_limit_mean": round(safe_mean(metrics["output_guard_ay_limit"]), 3),
        "v_des_eff_mean": round(safe_mean(metrics["v_des_eff"]), 3),
        "v_des_eff_min": round(min(metrics["v_des_eff"]), 3) if metrics["v_des_eff"] else float("nan"),
        "ref_v_ref_mean": round(safe_mean(metrics["ref_v_ref"]), 3),
        "ref_v_path_mean": round(safe_mean(metrics["ref_v_path"]), 3),
        "ref_kappa_abs_p95": round(percentile(metrics["ref_kappa_abs"], 95), 3),
        "ref_implied_ax_abs_p95": round(percentile(metrics["ref_implied_ax_abs_p95_horizon"], 95), 3),
        "ref_implied_ay_abs_p95": round(percentile(metrics["ref_implied_ay_abs_p95_horizon"], 95), 3),
        "ref_implied_jerk_abs_p95": round(percentile(metrics["ref_implied_jerk_abs_p95_horizon"], 95), 3),
        "cmd_vx_rms": round(rms(metrics["vx"]), 3),
        "cmd_wz_rms": round(rms(metrics["wz"]), 3),
        "cmd_dvx_rms_mps2": round(rms(metrics["cmd_dvx"]), 3),
        "cmd_dwz_rms_radps2": round(rms(metrics["cmd_dwz"]), 3),
        "odom_vx_rms": round(rms(metrics["odom_vx"]), 3),
        "odom_wz_rms": round(rms(metrics["odom_wz"]), 3),
        "odom_ay_abs_mean": round(safe_mean(metrics["odom_ay_abs"]), 3),
        "odom_ay_abs_rms": round(rms(metrics["odom_ay"]), 3),
        "odom_ay_abs_p95": round(percentile(metrics["odom_ay_abs"], 95), 3),
        "odom_ax_abs_p95": round(percentile(metrics["odom_ax_abs"], 95), 3),
        "odom_jerk_abs_p95": round(percentile(metrics["odom_jerk_abs"], 95), 3),
        "odom_kappa_abs_p95": round(percentile(metrics["odom_kappa_abs"], 95), 3),
        "track_dist_rms_m": round(rms(metrics["track_dist"]), 3),
        "track_dist_p95_m": round(percentile(metrics["track_dist"], 95), 3),
        "track_dist_max_m": round(safe_max(metrics["track_dist"]), 3),
        "track_heading_err_rms_deg": round(rms(metrics["track_heading_err_deg"]), 3),
        "track_heading_err_p95_deg": round(percentile(metrics["track_heading_err_deg"], 95), 3),
        "track_heading_err_max_deg": round(safe_max(metrics["track_heading_err_deg"]), 3),
        "status_val_breakdown": format_status_breakdown(status_breakdown),
        "status_transitions": " | ".join(
            f"{round(ts - start_time, 3)}:{status}" for ts, status in transitions
        ),
        "terminal_transitions": " | ".join(
            f"{round(ts - start_time, 3)}:{status}" for ts, status in terminal_transitions
        ),
        "goals": " | ".join(f"({x},{y})" for x, y in goals),
    }
    row["exec_ax_to_ref_p95_ratio"] = round(
        row["odom_ax_abs_p95"] / row["ref_implied_ax_abs_p95"], 3
    ) if row["ref_implied_ax_abs_p95"] and not math.isnan(row["ref_implied_ax_abs_p95"]) else float("nan")
    row["exec_ay_to_ref_p95_ratio"] = round(
        row["odom_ay_abs_p95"] / row["ref_implied_ay_abs_p95"], 3
    ) if row["ref_implied_ay_abs_p95"] and not math.isnan(row["ref_implied_ay_abs_p95"]) else float("nan")
    row["exec_jerk_to_ref_p95_ratio"] = round(
        row["odom_jerk_abs_p95"] / row["ref_implied_jerk_abs_p95"], 3
    ) if row["ref_implied_jerk_abs_p95"] and not math.isnan(row["ref_implied_jerk_abs_p95"]) else float("nan")
    for status in STATUS_BUCKETS:
        item = status_breakdown.get(status)
        key = status.lower()
        row[f"{key}_status_val_count"] = item["count"] if item else 0
        row[f"{key}_success_ratio"] = item["success_ratio"] if item else float("nan")
        row[f"{key}_solve_fail_count"] = item["solve_fail_count"] if item else 0
    add_lag_metrics(row, series, lag_window_s, lag_step_s)
    add_governor_timing_metrics(row, series, start_time)

    episode_rows = []
    for index, (seg_start, seg_end, status) in enumerate(tracking_segments, start=1):
        if status != "TRACKING":
            continue
        seg_metrics = defaultdict(list)
        prev_cmd_ts = None
        prev_cmd_vx = None
        prev_cmd_wz = None
        with rosbag.Bag(bag_path) as bag:
            for topic, msg, t in bag.read_messages(topics=[
                "/slosh/height",
                "/slosh/height_pred_max",
                "/mpc/solve_ms",
                "/mpc/status_val",
                "/slosh/speed_governor_active",
                "/slosh/speed_governor_ay_active",
                "/slosh/v_des_eff",
                "/cmd_vel",
            ]):
                ts = t.to_sec()
                if not (seg_start <= ts < seg_end):
                    continue
                if topic == "/slosh/height":
                    seg_metrics["height"].append(float(msg.data))
                elif topic == "/slosh/height_pred_max":
                    seg_metrics["pred"].append(float(msg.data))
                elif topic == "/mpc/solve_ms":
                    seg_metrics["solve_ms"].append(float(msg.data))
                elif topic == "/mpc/status_val":
                    seg_metrics["status_val"].append(int(msg.data))
                elif topic == "/slosh/speed_governor_active":
                    seg_metrics["governor"].append(int(msg.data))
                elif topic == "/slosh/speed_governor_ay_active":
                    seg_metrics["governor_ay"].append(int(msg.data))
                elif topic == "/slosh/v_des_eff":
                    seg_metrics["v_des_eff"].append(float(msg.data))
                elif topic == "/cmd_vel":
                    vx = float(msg.linear.x)
                    wz = float(msg.angular.z)
                    seg_metrics["vx"].append(vx)
                    seg_metrics["wz"].append(wz)
                    if prev_cmd_ts is not None:
                        dt = ts - prev_cmd_ts
                        if dt > 1e-6:
                            seg_metrics["cmd_dvx"].append((vx - prev_cmd_vx) / dt)
                            seg_metrics["cmd_dwz"].append((wz - prev_cmd_wz) / dt)
                    prev_cmd_ts = ts
                    prev_cmd_vx = vx
                    prev_cmd_wz = wz

        seg_governor_on = sum(1 for x in seg_metrics["governor"] if x == 1)
        seg_governor_total = len(seg_metrics["governor"])
        seg_governor_ay_on = sum(1 for x in seg_metrics["governor_ay"] if x == 1)
        seg_governor_ay_total = len(seg_metrics["governor_ay"])
        episode_rows.append({
            "bag_name": os.path.basename(bag_path),
            "episode_index": index,
            "start_s": round(seg_start - start_time, 3),
            "duration_s": round(seg_end - seg_start, 3),
            "height_rms_m": round(rms(seg_metrics["height"]), 6),
            "height_p95_m": round(percentile(seg_metrics["height"], 95), 6),
            "height_max_m": round(safe_max(seg_metrics["height"]), 6),
            "height_pred_rms_m": round(rms(seg_metrics["pred"]), 6),
            "height_pred_max_m": round(safe_max(seg_metrics["pred"]), 6),
            "solve_ms_mean": round(safe_mean(seg_metrics["solve_ms"]), 3),
            "solve_fail_count": sum(1 for x in seg_metrics["status_val"] if x != 1),
            "governor_active_ratio": round(seg_governor_on / seg_governor_total, 3) if seg_governor_total else 0.0,
            "governor_ay_active_ratio": round(seg_governor_ay_on / seg_governor_ay_total, 3) if seg_governor_ay_total else 0.0,
            "v_des_eff_mean": round(safe_mean(seg_metrics["v_des_eff"]), 3),
            "cmd_vx_rms": round(rms(seg_metrics["vx"]), 3),
            "cmd_wz_rms": round(rms(seg_metrics["wz"]), 3),
            "cmd_dvx_rms_mps2": round(rms(seg_metrics["cmd_dvx"]), 3),
            "cmd_dwz_rms_radps2": round(rms(seg_metrics["cmd_dwz"]), 3),
        })

    return row, episode_rows


def write_csv(csv_path, rows):
    fieldnames = list(rows[0].keys()) if rows else []
    if not fieldnames:
        return
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def finite_mean(rows, key):
    values = [row.get(key, float("nan")) for row in rows]
    return safe_mean(values)


def pct_change(value, baseline):
    if math.isnan(value) or math.isnan(baseline) or abs(baseline) <= 1e-12:
        return float("nan")
    return 100.0 * (value - baseline) / abs(baseline)


def fmt_value(value, digits=3):
    if math.isnan(value):
        return "nan"
    return f"{value:.{digits}f}"


def fmt_pct(value):
    if math.isnan(value):
        return "nan"
    return f"{value:+.1f}%"


def d0_verdict(means, baseline):
    checks = []
    checks.append(("solve", means["solve_success_ratio"] >= 0.97))
    checks.append(("time", pct_change(means["tracking_time_s"], baseline["tracking_time_s"]) <= 15.0))
    checks.append(("h_rms", pct_change(means["height_rms_m"], baseline["height_rms_m"]) <= -8.0))
    checks.append(("h_p95", pct_change(means["height_p95_m"], baseline["height_p95_m"]) <= -8.0))
    checks.append(("energy", pct_change(means["modal_energy_norm_rms"], baseline["modal_energy_norm_rms"]) <= -8.0))
    checks.append(("eta_dot", pct_change(means["eta_dot_norm_rms_mps"], baseline["eta_dot_norm_rms_mps"]) <= 0.0))
    checks.append(("ay_p95", pct_change(means["odom_ay_abs_p95"], baseline["odom_ay_abs_p95"]) <= -10.0))
    failed = [name for name, ok in checks if not ok]
    return "PASS" if not failed else "FAIL(" + ",".join(failed) + ")"


def print_group_summary(rows, baseline_condition):
    groups = defaultdict(list)
    for row in rows:
        groups[row["condition"]].append(row)

    if baseline_condition not in groups:
        print(f"\n分组汇总: 未找到基线条件 {baseline_condition}")
        return

    means_by_condition = {
        condition: {key: finite_mean(condition_rows, key) for key in D0_GROUP_METRICS}
        for condition, condition_rows in sorted(groups.items())
    }
    baseline = means_by_condition[baseline_condition]

    print(f"\n分组汇总（baseline={baseline_condition}）:")
    header = (
        "condition n verdict tracking_s h_rms h_p95 energy eta_dot "
        "ay_p95 kappa_p95 gov_ay gov_to_h gov_to_eta_dot"
    )
    print(header)
    for condition, condition_rows in sorted(groups.items()):
        means = means_by_condition[condition]
        if condition == baseline_condition:
            verdict = "BASE"
            deltas = {key: float("nan") for key in D0_GROUP_METRICS}
        else:
            verdict = d0_verdict(means, baseline)
            deltas = {
                key: pct_change(means[key], baseline[key])
                for key in D0_GROUP_METRICS
            }
        print(
            f"{condition} {len(condition_rows)} {verdict} "
            f"{fmt_value(means['tracking_time_s'])}({fmt_pct(deltas['tracking_time_s'])}) "
            f"{fmt_value(means['height_rms_m'], 6)}({fmt_pct(deltas['height_rms_m'])}) "
            f"{fmt_value(means['height_p95_m'], 6)}({fmt_pct(deltas['height_p95_m'])}) "
            f"{fmt_value(means['modal_energy_norm_rms'], 6)}({fmt_pct(deltas['modal_energy_norm_rms'])}) "
            f"{fmt_value(means['eta_dot_norm_rms_mps'], 6)}({fmt_pct(deltas['eta_dot_norm_rms_mps'])}) "
            f"{fmt_value(means['odom_ay_abs_p95'])}({fmt_pct(deltas['odom_ay_abs_p95'])}) "
            f"{fmt_value(means['odom_kappa_abs_p95'])}({fmt_pct(deltas['odom_kappa_abs_p95'])}) "
            f"{fmt_value(means['governor_ay_active_ratio'])} "
            f"{fmt_value(means['gov_ay_first_to_height_peak_s'])}s "
            f"{fmt_value(means['gov_ay_first_to_eta_dot_peak_s'])}s"
        )


def print_summary(rows, per_episode_rows, per_episode):
    print("汇总结果:")
    for row in rows:
        print(
            f"- {row['bag_name']}: Q={row['Q_label'] or '-'} "
            f"tracking={row['tracking_time_s']}s "
            f"height_rms={row['height_rms_m']}m "
            f"height_p95={row['height_p95_m']}m "
            f"height_max={row['height_max_m']}m "
            f"pred_rms={row['height_pred_rms_m']}m "
            f"eta_dot_rms={row['eta_dot_norm_rms_mps']}m/s "
            f"energy_norm_rms={row['modal_energy_norm_rms']} "
            f"solve_mean={row['solve_ms_mean']}ms "
            f"success={row['solve_success_ratio']} "
            f"fail={row['solve_fail_count']} "
            f"gov_ratio={row['governor_active_ratio']}"
            f" gov_ay_ratio={row['governor_ay_active_ratio']}"
            f" speed_cap_ratio={row['speed_cap_active_ratio']}"
            f" output_guard_ratio={row['output_guard_active_ratio']}"
            f" cmd_dwz_rms={row['cmd_dwz_rms_radps2']}"
            f" ay_abs={row['excitation_ay_abs_mean']}"
            f" alpha_abs={row['excitation_alpha_abs_mean']}"
            f" ay_height_lag={row['abs_ay_to_height_lag_s']}"
            f" ay_height_corr={row['abs_ay_to_height_corr']}"
            f" gov_to_h_peak={row['gov_ay_first_to_height_peak_s']}s"
            f" gov_to_eta_dot_peak={row['gov_ay_first_to_eta_dot_peak_s']}s"
            f" track_p95={row['track_dist_p95_m']}m"
            f" heading_p95={row['track_heading_err_p95_deg']}deg"
            f" odom_ay_p95={row['odom_ay_abs_p95']}"
        )
        print(f"  status_val by mpc_status: {row['status_val_breakdown']}")
    if per_episode and per_episode_rows:
        print("\n按 episode 明细:")
        for row in per_episode_rows:
            print(
                f"- {row['bag_name']} ep{row['episode_index']}: "
                f"dur={row['duration_s']}s "
                f"height_rms={row['height_rms_m']}m "
                f"height_p95={row['height_p95_m']}m "
                f"pred_rms={row['height_pred_rms_m']}m "
                f"fail={row['solve_fail_count']} "
                f"gov_ratio={row['governor_active_ratio']} "
                f"gov_ay_ratio={row['governor_ay_active_ratio']} "
                f"cmd_dwz_rms={row['cmd_dwz_rms_radps2']}"
            )


def main():
    args = parse_args()
    bag_paths = expand_inputs(args.inputs)
    if not bag_paths:
        print("未找到可用的 .bag 文件", file=sys.stderr)
        return 1

    rows = []
    episode_rows = []
    tracking_only = not args.all_segments
    strict_terminal_none = tracking_only and not args.tracking_with_terminal

    for bag_path in bag_paths:
        row, episodes = collect_metrics(
            bag_path,
            tracking_only=tracking_only,
            strict_terminal_none=strict_terminal_none,
            lag_window_s=args.lag_window_s,
            lag_step_s=args.lag_step_s,
        )
        rows.append(row)
        if args.per_episode:
            episode_rows.extend(episodes)

    print_summary(rows, episode_rows, args.per_episode)
    if args.group_summary:
        print_group_summary(rows, args.baseline_condition)

    if args.csv_path:
        write_csv(args.csv_path, rows)
        print(f"\n已写入 CSV: {args.csv_path}")
        if args.per_episode and episode_rows:
            base, ext = os.path.splitext(args.csv_path)
            episode_csv = f"{base}_episodes{ext or '.csv'}"
            write_csv(episode_csv, episode_rows)
            print(f"已写入 episode CSV: {episode_csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
