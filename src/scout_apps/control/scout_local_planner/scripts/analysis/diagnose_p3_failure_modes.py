#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import bisect
import csv
import math
import os
import statistics
from collections import defaultdict
from pathlib import Path

import rosbag
import yaml


XI_11 = 1.8412
G = 9.81


def parse_args():
    parser = argparse.ArgumentParser(
        description="Diagnose remaining P3 anti-slosh failure hypotheses from existing bags."
    )
    parser.add_argument("bags", nargs="+", help="P2/P3 bag files or directories")
    parser.add_argument(
        "--config",
        default="src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml",
        help="Planner YAML used for slosh height coefficient and omega_n",
    )
    parser.add_argument("--csv", default="", help="Optional CSV output path")
    parser.add_argument("--peak-window", type=float, default=1.0, help="Window around height peak, seconds")
    parser.add_argument("--lookback", type=float, default=1.0, help="Lookback window before |eta_x| peak")
    return parser.parse_args()


def expand_bags(paths):
    bags = []
    for path in paths:
        if os.path.isdir(path):
            bags.extend(os.path.join(path, name) for name in sorted(os.listdir(path)) if name.endswith(".bag"))
        elif os.path.isfile(path) and path.endswith(".bag"):
            bags.append(path)
    return sorted(dict.fromkeys(bags))


def infer_path_id(path):
    name = Path(path).name
    for path_id in ("P3_mixed", "P2_s_curve", "P1_single_turn", "P0_straight"):
        if path_id in name:
            return path_id
    return "UNKNOWN"


def infer_condition(path):
    name = Path(path).stem
    for condition in (
        "OUTPUT_GUARD",
        "PMG_COMBINED",
        "PMG_LONG",
        "PMG_LAT",
        "PMG",
        "NOM",
        "FAS_Q5",
        "GOV_AY",
        "ENERGY_WIN",
    ):
        if condition in name:
            return condition
    return "UNKNOWN"


def load_config(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    slosh = data.get("slosh", {}) or {}
    return {
        "R": float(slosh.get("container_radius", 0.0185)),
        "h": float(slosh.get("liquid_height", 0.058)),
        "rho": float(slosh.get("liquid_density", 1000.0)),
    }


def modal_params(cfg):
    r = cfg["R"]
    h = cfg["h"]
    rho = cfg["rho"]
    xi = XI_11
    m_f = rho * math.pi * r * r * h
    omega_n = math.sqrt(max(G * (xi / r) * math.tanh(xi * h / r), 0.0))
    m_n = m_f * (2.0 * r * math.tanh(xi * h / r)) / (xi * h * (xi * xi - 1.0))
    height_coeff = (4.0 * h * m_n) / (m_f * r)
    return omega_n, height_coeff


def msg_string(msg):
    if hasattr(msg, "data"):
        return str(msg.data)
    return str(msg)


def get_segments(bag_path, topic):
    values = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            values.append((stamp.to_sec(), msg_string(msg)))
    if not values:
        return []
    segments = []
    start, current = values[0]
    last = start
    for ts, value in values[1:]:
        if value != current:
            segments.append((start, ts, current))
            start, current = ts, value
        last = ts
    segments.append((start, last, current))
    return segments


def main_windows(bag_path):
    mpc = get_segments(bag_path, "/mpc_status")
    terminal = get_segments(bag_path, "/terminal/mode")
    windows = []
    for a0, a1, av in mpc:
        if av != "TRACKING":
            continue
        for b0, b1, bv in terminal:
            if bv != "NONE":
                continue
            start = max(a0, b0)
            end = min(a1, b1)
            if end > start:
                windows.append((start, end))
    return windows


def in_windows(ts, windows):
    return any(start <= ts <= end for start, end in windows)


def interp(series, ts):
    if not series:
        return float("nan")
    times = [x[0] for x in series]
    idx = bisect.bisect_left(times, ts)
    if idx == 0:
        return series[0][1]
    if idx >= len(series):
        return series[-1][1]
    t0, v0 = series[idx - 1]
    t1, v1 = series[idx]
    if t1 <= t0:
        return v0
    r = (ts - t0) / (t1 - t0)
    return v0 + r * (v1 - v0)


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def path_points(msg):
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def nearest_path_error(x, y, yaw, points):
    if len(points) < 2:
        return float("nan"), float("nan")
    best_dist = float("inf")
    best_heading = float("nan")
    for idx in range(len(points) - 1):
        ax, ay = points[idx]
        bx, by = points[idx + 1]
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 <= 1e-12:
            continue
        u = max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / ab2))
        qx = ax + u * abx
        qy = ay + u * aby
        dist = math.hypot(x - qx, y - qy)
        if dist < best_dist:
            best_dist = dist
            best_heading = math.atan2(aby, abx)
    return best_dist, abs(wrap_to_pi(yaw - best_heading))


def percentile(values, pct):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return float("nan")
    return values[int((pct / 100.0) * (len(values) - 1))]


def rms(values):
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def mean(values):
    values = [v for v in values if math.isfinite(v)]
    return statistics.mean(values) if values else float("nan")


def pearson(xs, ys):
    if len(xs) < 5 or len(xs) != len(ys):
        return float("nan")
    mx = mean(xs)
    my = mean(ys)
    dx = [x - mx for x in xs]
    dy = [y - my for y in ys]
    sx = math.sqrt(sum(x * x for x in dx))
    sy = math.sqrt(sum(y * y for y in dy))
    if sx <= 1e-12 or sy <= 1e-12:
        return float("nan")
    return sum(x * y for x, y in zip(dx, dy)) / (sx * sy)


def derivative(series):
    out = []
    for (t0, v0), (t1, v1) in zip(series, series[1:]):
        dt = t1 - t0
        if dt <= 1e-6:
            continue
        out.append(((t0 + t1) * 0.5, (v1 - v0) / dt))
    return out


def integrate_abs(series, start, end):
    samples = [(t, v) for t, v in series if start <= t <= end and math.isfinite(v)]
    if len(samples) < 2:
        return float("nan")
    total = 0.0
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        total += 0.5 * (abs(v0) + abs(v1)) * (t1 - t0)
    return total


def max_abs_with_time(series, start, end):
    samples = [(t, v) for t, v in series if start <= t <= end and math.isfinite(v)]
    if not samples:
        return float("nan"), float("nan")
    return max(samples, key=lambda item: abs(item[1]))


def read_series(bag_path):
    series = defaultdict(list)
    topics = [
        "/slosh/state",
        "/slosh/height",
        "/cmd_vel",
        "/odom",
        "/mpc/status_val",
        "/terminal/recovery_latched",
        "/mpc/reference_path",
        "/slosh/v_des_eff",
    ]
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = stamp.to_sec()
            if topic == "/slosh/state" and len(msg.data) >= 4:
                series["state"].append((ts, [float(x) for x in msg.data[:4]]))
            elif topic == "/slosh/height":
                series["height"].append((ts, abs(float(msg.data))))
            elif topic == "/cmd_vel":
                series["cmd_w"].append((ts, float(msg.angular.z)))
                series["cmd_v"].append((ts, float(msg.linear.x)))
            elif topic == "/odom":
                series["odom_w"].append((ts, float(msg.twist.twist.angular.z)))
                series["odom_v"].append((ts, float(msg.twist.twist.linear.x)))
                series["odom_pose"].append(
                    (
                        ts,
                        (
                            float(msg.pose.pose.position.x),
                            float(msg.pose.pose.position.y),
                            yaw_from_quat(msg.pose.pose.orientation),
                        ),
                    )
                )
            elif topic == "/mpc/status_val":
                series["status_val"].append((ts, float(msg.data)))
            elif topic == "/terminal/recovery_latched":
                series["recovery_latched"].append((ts, bool(msg.data)))
            elif topic == "/mpc/reference_path":
                pts = path_points(msg)
                if pts:
                    series["reference_path"].append((ts, pts))
            elif topic == "/slosh/v_des_eff":
                series["v_des_eff"].append((ts, float(msg.data)))
    return series


def peak_window_times(series, windows, half_width):
    heights = [(t, h) for t, h in series["height"] if in_windows(t, windows)]
    if not heights:
        return float("nan"), float("nan"), float("nan")
    peak_t, _ = max(heights, key=lambda item: item[1])
    return peak_t, peak_t - half_width, peak_t + half_width


def analyze_bag(bag_path, omega_n, height_coeff, peak_window, lookback):
    windows = main_windows(bag_path)
    series = read_series(bag_path)
    peak_t, win0, win1 = peak_window_times(series, windows, peak_window)
    eta_x_samples = [(t, abs(s[0])) for t, s in series["state"] if in_windows(t, windows)]
    eta_x_peak_t = max(eta_x_samples, key=lambda item: item[1])[0] if eta_x_samples else float("nan")
    lb0 = eta_x_peak_t - lookback
    lb1 = eta_x_peak_t

    state_window = [(t, s) for t, s in series["state"] if win0 <= t <= win1 and in_windows(t, windows)]
    ex = []
    ey = []
    height_model_diff = []
    for ts, state in state_window:
        eta_x = state[0]
        eta_y = state[2]
        ex.append((omega_n * eta_x) ** 2)
        ey.append((omega_n * eta_y) ** 2)
        measured_h = interp(series["height"], ts)
        reconstructed_h = height_coeff * math.hypot(eta_x, eta_y)
        if math.isfinite(measured_h):
            height_model_diff.append(abs(measured_h - reconstructed_h))

    cmd_odom_errors = []
    cmd_odom_errors_all = []
    track_errors = []
    heading_errors = []
    status_fail = []
    recovery_vals = []
    track_error_series = []
    for ts, cmd_w in series["cmd_w"]:
        if in_windows(ts, windows):
            odom_w = interp(series["odom_w"], ts)
            if math.isfinite(odom_w):
                err = cmd_w - odom_w
                cmd_odom_errors_all.append(err)
                if win0 <= ts <= win1:
                    cmd_odom_errors.append(err)
    for ts, pose in series["odom_pose"]:
        if not in_windows(ts, windows):
            continue
        pts = interp_path(series["reference_path"], ts)
        if pts:
            dist, heading = nearest_path_error(pose[0], pose[1], pose[2], pts)
            track_errors.append(dist)
            heading_errors.append(heading)
            track_error_series.append((ts, dist))
    for ts, value in series["status_val"]:
        if in_windows(ts, windows):
            status_fail.append(value < 0.5)
    for ts, value in series["recovery_latched"]:
        if in_windows(ts, windows):
            recovery_vals.append(value)

    ex_mean = mean(ex)
    ey_mean = mean(ey)
    total_e = ex_mean + ey_mean
    eta_x_energy_ratio = ex_mean / total_e if total_e > 1e-12 else float("nan")
    odom_ax = derivative(series["odom_v"])
    v_ref_dot = derivative(series["v_des_eff"])
    ax_peak_t, ax_peak = max_abs_with_time(odom_ax, lb0, lb1)
    vref_dot_peak_t, vref_dot_peak = max_abs_with_time(v_ref_dot, lb0, lb1)
    ax_samples = []
    vref_dot_samples = []
    track_samples = []
    for ts, ax in odom_ax:
        if lb0 <= ts <= lb1:
            ax_samples.append(ax)
            vref_dot_samples.append(interp(v_ref_dot, ts))
            track_samples.append(interp(track_error_series, ts))

    return {
        "bag": bag_path,
        "path_id": infer_path_id(bag_path),
        "condition": infer_condition(bag_path),
        "peak_t": peak_t,
        "peak_window_samples": len(state_window),
        "eta_x_energy_ratio": eta_x_energy_ratio,
        "eta_y_energy_ratio": ey_mean / total_e if total_e > 1e-12 else float("nan"),
        "cmd_odom_w_err_rms_peak": rms(cmd_odom_errors),
        "cmd_odom_w_err_rms_main": rms(cmd_odom_errors_all),
        "track_dist_p95": percentile(track_errors, 95),
        "heading_err_p95_deg": math.degrees(percentile(heading_errors, 95)),
        "status_fail_ratio": mean([1.0 if x else 0.0 for x in status_fail]),
        "recovery_latched_ratio": mean([1.0 if x else 0.0 for x in recovery_vals]),
        "height_model_abs_diff_max": max(height_model_diff) if height_model_diff else float("nan"),
        "height_model_abs_diff_p95": percentile(height_model_diff, 95),
        "eta_x_peak_t": eta_x_peak_t,
        "ax_peak_abs": abs(ax_peak) if math.isfinite(ax_peak) else float("nan"),
        "ax_peak_lead_s": eta_x_peak_t - ax_peak_t if math.isfinite(ax_peak_t) else float("nan"),
        "ax_abs_integral": integrate_abs(odom_ax, lb0, lb1),
        "vref_dot_peak_abs": abs(vref_dot_peak) if math.isfinite(vref_dot_peak) else float("nan"),
        "vref_dot_peak_lead_s": eta_x_peak_t - vref_dot_peak_t if math.isfinite(vref_dot_peak_t) else float("nan"),
        "vref_dot_abs_integral": integrate_abs(v_ref_dot, lb0, lb1),
        "corr_ax_vref_dot": pearson(ax_samples, vref_dot_samples),
        "corr_ax_track_error": pearson(ax_samples, track_samples),
    }


def interp_path(series, ts):
    if not series:
        return []
    times = [x[0] for x in series]
    idx = bisect.bisect_right(times, ts) - 1
    if idx < 0:
        idx = 0
    return series[idx][1]


def write_csv(path, rows):
    if not path or not rows:
        return
    fields = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def print_group_summary(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["path_id"], row["condition"])].append(row)
    print("\nGroup summary")
    for key in sorted(groups):
        group = groups[key]
        print(
            f"- {key[0]} {key[1]} n={len(group)} "
            f"eta_x_ratio={mean([r['eta_x_energy_ratio'] for r in group]):.3f} "
            f"ax_peak={mean([r['ax_peak_abs'] for r in group]):.3f} "
            f"ax_lead={mean([r['ax_peak_lead_s'] for r in group]):.3f} "
            f"corr_ax_vref={mean([r['corr_ax_vref_dot'] for r in group]):.3f} "
            f"corr_ax_track={mean([r['corr_ax_track_error'] for r in group]):.3f} "
            f"w_err_peak={mean([r['cmd_odom_w_err_rms_peak'] for r in group]):.3f} "
            f"track_p95={mean([r['track_dist_p95'] for r in group]):.3f} "
            f"status_fail={mean([r['status_fail_ratio'] for r in group]):.3f} "
            f"h_diff_p95={mean([r['height_model_abs_diff_p95'] for r in group]):.6f}"
        )


def main():
    args = parse_args()
    cfg = load_config(args.config)
    omega_n, height_coeff = modal_params(cfg)
    bags = expand_bags(args.bags)
    if not bags:
        raise SystemExit("No bag files found")
    rows = [analyze_bag(path, omega_n, height_coeff, args.peak_window, args.lookback) for path in bags]
    for row in rows:
        print(
            f"{Path(row['bag']).name}: path={row['path_id']} condition={row['condition']} "
            f"eta_x_ratio={row['eta_x_energy_ratio']:.3f} "
            f"ax_peak={row['ax_peak_abs']:.3f} ax_lead={row['ax_peak_lead_s']:.3f} "
            f"corr_ax_vref={row['corr_ax_vref_dot']:.3f} "
            f"corr_ax_track={row['corr_ax_track_error']:.3f} "
            f"w_err_peak={row['cmd_odom_w_err_rms_peak']:.3f} "
            f"track_p95={row['track_dist_p95']:.3f} "
            f"status_fail={row['status_fail_ratio']:.3f} "
            f"h_diff_p95={row['height_model_abs_diff_p95']:.6f}"
        )
    print_group_summary(rows)
    write_csv(args.csv, rows)
    if args.csv:
        print(f"\ncsv: {args.csv}")


if __name__ == "__main__":
    main()
