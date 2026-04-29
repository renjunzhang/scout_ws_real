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
from scipy.linalg import expm


XI_11 = 1.8412
G = 9.81


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Step-0 offline check for PMG: validate lateral slosh replay against "
            "recorded /slosh/state and /slosh/height before any PMG counterfactual."
        )
    )
    parser.add_argument("bags", nargs="+", help="Bag files or directories containing bag files")
    parser.add_argument(
        "--config",
        default="src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml",
        help="Planner YAML used for slosh parameters",
    )
    parser.add_argument("--csv", default="", help="Optional output CSV path")
    parser.add_argument(
        "--input-source",
        choices=("slosh_ay", "odom", "cmd"),
        default="slosh_ay",
        help=(
            "Input used as u_y for baseline replay. slosh_ay replays /slosh/ax_est and /slosh/ay_est; "
            "odom/cmd use v*omega. Default: slosh_ay."
        ),
    )
    parser.add_argument(
        "--max-p95-error",
        type=float,
        default=0.15,
        help="Fail threshold for reconstructed h_p95 relative error. Default: 0.15.",
    )
    parser.add_argument("--low-v", type=float, default=0.05, help="Low-input |v| threshold")
    parser.add_argument("--low-omega", type=float, default=0.10, help="Low-input |omega| threshold")
    parser.add_argument("--low-duration", type=float, default=0.5, help="Minimum low-input segment duration")
    parser.add_argument("--pmg-replay", action="store_true", help="Run signed lateral PMG counterfactual replay")
    parser.add_argument("--pmg-ay-scale", type=float, default=0.88, help="ay_proxy scale from odom_v*odom_omega")
    parser.add_argument("--pmg-ay-lag", type=float, default=0.10, help="ay_proxy lag in seconds")
    parser.add_argument("--pmg-ay-intercept", type=float, default=0.0, help="ay_proxy intercept")
    parser.add_argument("--pmg-tau", type=float, default=0.0, help="PMG prediction window. 0 means one damped period")
    parser.add_argument("--pmg-grid", type=int, default=24, help="PMG signed prediction grid samples")
    parser.add_argument("--pmg-omega-scale", type=float, default=1.0, help="Omega_n scale used inside PMG only")
    parser.add_argument("--pmg-zeta-scale", type=float, default=1.0, help="Zeta scale used inside PMG only")
    parser.add_argument(
        "--longitudinal-replay",
        action="store_true",
        help="Run signed longitudinal counterfactual replay on eta_x/ax.",
    )
    parser.add_argument(
        "--combined-replay",
        action="store_true",
        help="Run signed longitudinal + lateral PMG counterfactual replay.",
    )
    return parser.parse_args()


def expand_bags(paths):
    bags = []
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.endswith(".bag"):
                    bags.append(os.path.join(path, name))
        elif os.path.isfile(path) and path.endswith(".bag"):
            bags.append(path)
    return sorted(dict.fromkeys(bags))


def load_config(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    slosh = data.get("slosh", {}) or {}
    return {
        "R": float(slosh.get("container_radius", 0.0185)),
        "h": float(slosh.get("liquid_height", 0.058)),
        "rho": float(slosh.get("liquid_density", 1000.0)),
        "zeta": float(slosh.get("damping_ratio", 0.05)),
        "mode_index": int(slosh.get("mode_index", 1)),
        "use_linear_model": bool(slosh.get("use_linear_model", True)),
    }


def modal_params(cfg):
    R = cfg["R"]
    h = cfg["h"]
    rho = cfg["rho"]
    xi = XI_11
    m_f = rho * math.pi * R * R * h
    omega_n = math.sqrt(max(G * (xi / R) * math.tanh(xi * h / R), 0.0))
    m_n = m_f * (2.0 * R * math.tanh(xi * h / R)) / (xi * h * (xi * xi - 1.0))
    if cfg["use_linear_model"]:
        height_coeff = (4.0 * h * m_n) / (m_f * R)
    else:
        height_coeff = (xi * xi * h * m_n) / (m_f * R)
    return omega_n, height_coeff


def infer_path_id(path):
    name = Path(path).name
    for path_id in ("P3_mixed", "P2_s_curve", "P1_single_turn", "P0_straight"):
        if path_id in name:
            return path_id
    return "UNKNOWN"


def infer_condition(path):
    name = Path(path).stem
    known = (
        "OUTPUT_GUARD",
        "PMG_COMBINED",
        "PMG_LONG",
        "PMG_LAT",
        "PMG",
        "PROFILE_SELECTIVE",
        "PROFILE_SAFE",
        "PROFILE_RISK",
        "PROFILE_WINDOW",
        "ENERGY_WIN",
        "GOV_AY",
        "FAS_Q5",
        "NOM",
    )
    for condition in known:
        if condition in name:
            return condition
    return "UNKNOWN"


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
    start_t, current = values[0]
    last_t = start_t
    for ts, value in values[1:]:
        if value != current:
            segments.append((start_t, ts, current))
            start_t, current = ts, value
        last_t = ts
    segments.append((start_t, last_t, current))
    return segments


def main_windows(bag_path):
    mpc_segments = get_segments(bag_path, "/mpc_status")
    terminal_segments = get_segments(bag_path, "/terminal/mode")
    windows = []
    for m_start, m_end, m_value in mpc_segments:
        if m_value != "TRACKING":
            continue
        for t_start, t_end, t_value in terminal_segments:
            if t_value != "NONE":
                continue
            start = max(m_start, t_start)
            end = min(m_end, t_end)
            if end > start:
                windows.append((start, end))
    return windows


def in_windows(ts, windows):
    return any(start <= ts <= end for start, end in windows)


def percentile(values, pct):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return float("nan")
    idx = int((pct / 100.0) * (len(values) - 1))
    return values[max(0, min(len(values) - 1, idx))]


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


def linear_fit(xs, ys):
    if len(xs) < 5 or len(xs) != len(ys):
        return float("nan"), float("nan")
    mx = mean(xs)
    my = mean(ys)
    var_x = sum((x - mx) ** 2 for x in xs)
    if var_x <= 1e-12:
        return float("nan"), float("nan")
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = cov / var_x
    intercept = my - slope * mx
    return slope, intercept


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


def discrete_lateral_step(eta, eta_dot, uy, dt, omega_n, zeta):
    if dt <= 0.0:
        return eta, eta_dot
    a = [[0.0, 1.0], [-omega_n * omega_n, -2.0 * zeta * omega_n]]
    mat = [[a[0][0], a[0][1], 0.0], [a[1][0], a[1][1], -1.0], [0.0, 0.0, 0.0]]
    phi = expm([[v * dt for v in row] for row in mat])
    eta_next = phi[0][0] * eta + phi[0][1] * eta_dot + phi[0][2] * uy
    dot_next = phi[1][0] * eta + phi[1][1] * eta_dot + phi[1][2] * uy
    return float(eta_next), float(dot_next)


def eta_after_constant_u(eta, eta_dot, uy, dt, omega_n, zeta):
    next_eta, _ = discrete_lateral_step(eta, eta_dot, uy, dt, omega_n, zeta)
    return next_eta


def signed_pmg_u(eta, eta_dot, u_nom, eta_threshold, tau, grid, omega_n, zeta):
    if eta_threshold <= 0.0 or grid <= 0:
        return u_nom, False, float("-inf"), float("inf")

    lower = float("-inf")
    upper = float("inf")
    for idx in range(1, grid + 1):
        dt = tau * idx / grid
        free_eta = eta_after_constant_u(eta, eta_dot, 0.0, dt, omega_n, zeta)
        unit_eta = eta_after_constant_u(eta, eta_dot, 1.0, dt, omega_n, zeta)
        gain = unit_eta - free_eta
        if abs(gain) <= 1e-12:
            if free_eta < -eta_threshold or free_eta > eta_threshold:
                return 0.0, True, 0.0, 0.0
            continue
        a = (-eta_threshold - free_eta) / gain
        b = (eta_threshold - free_eta) / gain
        lo = min(a, b)
        hi = max(a, b)
        lower = max(lower, lo)
        upper = min(upper, hi)

    if lower > upper:
        # No constant input can satisfy all grid points; choose the closest boundary to zero.
        chosen = lower if abs(lower) < abs(upper) else upper
        return chosen, True, lower, upper
    if u_nom < lower:
        return lower, True, lower, upper
    if u_nom > upper:
        return upper, True, lower, upper
    return u_nom, False, lower, upper


def read_bag_series(bag_path):
    series = defaultdict(list)
    topics = [
        "/slosh/state",
        "/slosh/height",
        "/slosh/ax_est",
        "/slosh/ay_est",
        "/slosh/output_guard_active",
        "/odom",
        "/cmd_vel",
    ]
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = stamp.to_sec()
            if topic == "/slosh/state" and len(msg.data) >= 4:
                series["state"].append((ts, [float(x) for x in msg.data[:4]]))
            elif topic == "/slosh/height":
                series["height"].append((ts, abs(float(msg.data))))
            elif topic == "/slosh/ax_est":
                series["slosh_ax"].append((ts, float(msg.data)))
            elif topic == "/slosh/ay_est":
                series["slosh_ay"].append((ts, float(msg.data)))
            elif topic == "/slosh/output_guard_active":
                series["fixed_guard"].append((ts, bool(msg.data)))
            elif topic == "/odom":
                series["odom_v"].append((ts, float(msg.twist.twist.linear.x)))
                series["odom_w"].append((ts, float(msg.twist.twist.angular.z)))
            elif topic == "/cmd_vel":
                series["cmd_v"].append((ts, float(msg.linear.x)))
                series["cmd_w"].append((ts, float(msg.angular.z)))
    return series


def detect_low_input_segments(samples, low_v, low_w, min_duration):
    segments = []
    start = None
    last = None
    for ts, v, w in samples:
        low = abs(v) < low_v and abs(w) < low_w
        if low and start is None:
            start = ts
        if not low and start is not None and last is not None:
            if last - start >= min_duration:
                segments.append((start, last))
            start = None
        last = ts
    if start is not None and last is not None and last - start >= min_duration:
        segments.append((start, last))
    return segments


def mapping_samples(series, state_samples, source, lag=0.0):
    xs = []
    ys = []
    for ts, _ in state_samples:
        target = interp(series["slosh_ay"], ts)
        v = interp(series[f"{source}_v"], ts - lag)
        w = interp(series[f"{source}_w"], ts - lag)
        if math.isfinite(target) and math.isfinite(v) and math.isfinite(w):
            xs.append(v * w)
            ys.append(target)
    return xs, ys


def best_lag(series, state_samples, source):
    best = (float("nan"), float("nan"))
    for step in range(-6, 7):
        lag = step * 0.05
        xs, ys = mapping_samples(series, state_samples, source, lag)
        corr = pearson(xs, ys)
        if not math.isfinite(corr):
            continue
        if not math.isfinite(best[1]) or abs(corr) > abs(best[1]):
            best = (lag, corr)
    return best


def summarize_ay_mapping(series, state_samples, source):
    xs, ys = mapping_samples(series, state_samples, source, 0.0)
    slope, intercept = linear_fit(xs, ys)
    fitted = [slope * x + intercept for x in xs] if math.isfinite(slope) else []
    errors = [y - x for x, y in zip(xs, ys)]
    fit_errors = [y - y_hat for y, y_hat in zip(ys, fitted)]
    lag, lag_corr = best_lag(series, state_samples, source)
    corr = pearson(xs, ys)
    return {
        f"{source}_ay_samples": len(xs),
        f"{source}_ay_corr": corr,
        f"{source}_ay_slope": slope,
        f"{source}_ay_intercept": intercept,
        f"{source}_ay_rmse": rms(errors),
        f"{source}_ay_fit_rmse": rms(fit_errors),
        f"{source}_ay_abs_p95": percentile([abs(x) for x in xs], 95),
        f"{source}_ay_target_abs_p95": percentile([abs(y) for y in ys], 95),
        f"{source}_ay_error_abs_p95": percentile([abs(e) for e in errors], 95),
        f"{source}_ay_best_lag_s": lag,
        f"{source}_ay_best_lag_corr": lag_corr,
    }


def analyze_bag(bag_path, cfg, args):
    omega_n, height_coeff = modal_params(cfg)
    windows = main_windows(bag_path)
    series = read_bag_series(bag_path)
    if args.input_source == "slosh_ay":
        source_v = []
        source_w = []
        source_ax = series["slosh_ax"]
        source_ay = series["slosh_ay"]
    else:
        source_v = series[f"{args.input_source}_v"]
        source_w = series[f"{args.input_source}_w"]
        source_ax = []
        source_ay = []
    state_samples = [(t, s) for t, s in series["state"] if in_windows(t, windows)]
    height_samples = [(t, h) for t, h in series["height"] if in_windows(t, windows)]

    reconstructed_h = []
    reconstructed_lateral_h = []
    reconstructed_eta_dot_abs = []
    eta_x = None
    eta_x_dot = None
    eta_y = None
    eta_y_dot = None
    last_t = None
    for ts, state in state_samples:
        if eta_y is None:
            eta_x = float(state[0])
            eta_x_dot = float(state[1])
            eta_y = float(state[2])
            eta_y_dot = float(state[3])
            last_t = ts
        else:
            dt = min(max(ts - last_t, 0.0), 0.2)
            if args.input_source == "slosh_ay":
                ux = interp(source_ax, last_t)
                uy = interp(source_ay, last_t)
                ux = ux if math.isfinite(ux) else 0.0
                uy = uy if math.isfinite(uy) else 0.0
            else:
                v = interp(source_v, last_t)
                w = interp(source_w, last_t)
                ux = 0.0
                uy = v * w if math.isfinite(v) and math.isfinite(w) else 0.0
            eta_x, eta_x_dot = discrete_lateral_step(eta_x, eta_x_dot, ux, dt, omega_n, cfg["zeta"])
            eta_y, eta_y_dot = discrete_lateral_step(eta_y, eta_y_dot, uy, dt, omega_n, cfg["zeta"])
            last_t = ts
        reconstructed_h.append((ts, height_coeff * math.hypot(eta_x, eta_y)))
        reconstructed_lateral_h.append((ts, abs(height_coeff * eta_y)))
        reconstructed_eta_dot_abs.append(math.hypot(eta_x_dot, eta_y_dot))

    measured_h = [h for _, h in height_samples]
    recon_h = [h for _, h in reconstructed_h]
    measured_lateral_h = [abs(height_coeff * state[2]) for _, state in state_samples]
    measured_eta_x_values = [abs(state[0]) for _, state in state_samples]
    recon_lateral_h = [h for _, h in reconstructed_lateral_h]
    measured_h_p95 = percentile(measured_h, 95)
    recon_h_p95 = percentile(recon_h, 95)
    measured_lateral_h_p95 = percentile(measured_lateral_h, 95)
    recon_lateral_h_p95 = percentile(recon_lateral_h, 95)
    p95_rel_error = abs(recon_h_p95 - measured_h_p95) / measured_h_p95 if measured_h_p95 > 1e-9 else float("nan")
    lateral_p95_rel_error = (
        abs(recon_lateral_h_p95 - measured_lateral_h_p95) / measured_lateral_h_p95
        if measured_lateral_h_p95 > 1e-9
        else float("nan")
    )

    input_samples = []
    for ts, _ in state_samples:
        v = interp(series["odom_v"], ts)
        w = interp(series["odom_w"], ts)
        if math.isfinite(v) and math.isfinite(w):
            input_samples.append((ts, v, w))
    low_segments = detect_low_input_segments(input_samples, args.low_v, args.low_omega, args.low_duration)
    odom_mapping = summarize_ay_mapping(series, state_samples, "odom")
    cmd_mapping = summarize_ay_mapping(series, state_samples, "cmd")

    row = {
        "bag": bag_path,
        "path_id": infer_path_id(bag_path),
        "condition": infer_condition(bag_path),
        "windows": len(windows),
        "samples": len(state_samples),
        "omega_n": omega_n,
        "zeta": cfg["zeta"],
        "height_coeff": height_coeff,
        "measured_h_p95": measured_h_p95,
        "recon_h_p95": recon_h_p95,
        "h_p95_rel_error": p95_rel_error,
        "measured_lateral_h_p95": measured_lateral_h_p95,
        "measured_eta_x_p95": percentile(measured_eta_x_values, 95),
        "measured_eta_x_energy_p95": percentile([(omega_n * x) ** 2 for x in measured_eta_x_values], 95),
        "recon_lateral_h_p95": recon_lateral_h_p95,
        "lateral_h_p95_rel_error": lateral_p95_rel_error,
        "measured_h_rms": rms(measured_h),
        "recon_h_rms": rms(recon_h),
        "recon_eta_dot_abs_mean": mean(reconstructed_eta_dot_abs),
        "low_input_segments": len(low_segments),
        "low_input_total_s": sum(end - start for start, end in low_segments),
        "verdict": "PASS" if math.isfinite(p95_rel_error) and p95_rel_error <= args.max_p95_error else "FAIL",
    }
    row.update(odom_mapping)
    row.update(cmd_mapping)
    return row


def pmg_replay_bag(row, cfg, args, eta_threshold):
    omega_n, height_coeff = modal_params(cfg)
    pmg_omega_n = omega_n * args.pmg_omega_scale
    pmg_zeta = cfg["zeta"] * args.pmg_zeta_scale
    tau = args.pmg_tau
    if tau <= 0.0:
        omega_d = pmg_omega_n * math.sqrt(max(1e-9, 1.0 - pmg_zeta * pmg_zeta))
        tau = 2.0 * math.pi / omega_d

    series = read_bag_series(row["bag"])
    windows = main_windows(row["bag"])
    state_samples = [(t, s) for t, s in series["state"] if in_windows(t, windows)]
    if not state_samples:
        return {}

    eta_x = float(state_samples[0][1][0])
    eta_x_dot = float(state_samples[0][1][1])
    eta_y = float(state_samples[0][1][2])
    eta_y_dot = float(state_samples[0][1][3])
    last_t = state_samples[0][0]

    h_values = []
    lateral_h_values = []
    eta_dot_values = []
    pmg_active = []
    fixed_active = []
    ay_nom_values = []
    ay_pmg_values = []
    pmg_intersections = 0
    pmg_union = 0
    pmg_before_peak_times = []
    fixed_before_peak_times = []
    measured_height = [(t, h) for t, h in series["height"] if in_windows(t, windows)]
    peak_time = max(measured_height, key=lambda item: item[1])[0] if measured_height else float("nan")

    for ts, _ in state_samples:
        if ts != last_t:
            dt = min(max(ts - last_t, 0.0), 0.2)
            ux = interp(series["slosh_ax"], last_t)
            ux = ux if math.isfinite(ux) else 0.0
            odom_v = interp(series["odom_v"], last_t - args.pmg_ay_lag)
            odom_w = interp(series["odom_w"], last_t - args.pmg_ay_lag)
            ay_nom = (
                args.pmg_ay_scale * odom_v * odom_w + args.pmg_ay_intercept
                if math.isfinite(odom_v) and math.isfinite(odom_w)
                else 0.0
            )
            ay_pmg, active, _, _ = signed_pmg_u(
                eta_y,
                eta_y_dot,
                ay_nom,
                eta_threshold,
                tau,
                args.pmg_grid,
                pmg_omega_n,
                pmg_zeta,
            )
            eta_x, eta_x_dot = discrete_lateral_step(eta_x, eta_x_dot, ux, dt, omega_n, cfg["zeta"])
            eta_y, eta_y_dot = discrete_lateral_step(eta_y, eta_y_dot, ay_pmg, dt, omega_n, cfg["zeta"])
            last_t = ts
        else:
            odom_v = interp(series["odom_v"], ts - args.pmg_ay_lag)
            odom_w = interp(series["odom_w"], ts - args.pmg_ay_lag)
            ay_nom = (
                args.pmg_ay_scale * odom_v * odom_w + args.pmg_ay_intercept
                if math.isfinite(odom_v) and math.isfinite(odom_w)
                else 0.0
            )
            ay_pmg, active, _, _ = signed_pmg_u(
                eta_y,
                eta_y_dot,
                ay_nom,
                eta_threshold,
                tau,
                args.pmg_grid,
                pmg_omega_n,
                pmg_zeta,
            )

        fixed = bool(interp_bool(series["fixed_guard"], ts))
        h_values.append(height_coeff * math.hypot(eta_x, eta_y))
        lateral_h_values.append(abs(height_coeff * eta_y))
        eta_dot_values.append(math.hypot(eta_x_dot, eta_y_dot))
        pmg_active.append(active)
        fixed_active.append(fixed)
        ay_nom_values.append(ay_nom)
        ay_pmg_values.append(ay_pmg)
        if active or fixed:
            pmg_union += 1
        if active and fixed:
            pmg_intersections += 1
        if active and math.isfinite(peak_time) and ts <= peak_time:
            pmg_before_peak_times.append(ts)
        if fixed and math.isfinite(peak_time) and ts <= peak_time:
            fixed_before_peak_times.append(ts)

    pmg_count = sum(1 for x in pmg_active if x)
    fixed_count = sum(1 for x in fixed_active if x)
    pmg_lead = peak_time - min(pmg_before_peak_times) if pmg_before_peak_times else float("nan")
    fixed_lead = peak_time - min(fixed_before_peak_times) if fixed_before_peak_times else float("nan")
    return {
        "pmg_tau_s": tau,
        "pmg_eta_threshold": eta_threshold,
        "pmg_active_ratio": pmg_count / len(pmg_active) if pmg_active else float("nan"),
        "fixed_guard_active_ratio": fixed_count / len(fixed_active) if fixed_active else float("nan"),
        "pmg_fixed_overlap_jaccard": pmg_intersections / pmg_union if pmg_union else 0.0,
        "pmg_peak_lead_s": pmg_lead,
        "fixed_guard_peak_lead_s": fixed_lead,
        "pmg_vs_fixed_lead_diff_s": abs(pmg_lead - fixed_lead) if math.isfinite(pmg_lead) and math.isfinite(fixed_lead) else float("nan"),
        "pmg_h_rms": rms(h_values),
        "pmg_h_p95": percentile(h_values, 95),
        "pmg_lateral_h_p95": percentile(lateral_h_values, 95),
        "pmg_eta_dot_rms": rms(eta_dot_values),
        "pmg_ay_nom_abs_p95": percentile([abs(x) for x in ay_nom_values], 95),
        "pmg_ay_out_abs_p95": percentile([abs(x) for x in ay_pmg_values], 95),
        "pmg_ay_delta_abs_mean": mean([abs(a - b) for a, b in zip(ay_nom_values, ay_pmg_values)]),
    }


def longitudinal_replay_bag(row, cfg, args, eta_threshold):
    omega_n, height_coeff = modal_params(cfg)
    pmg_omega_n = omega_n * args.pmg_omega_scale
    pmg_zeta = cfg["zeta"] * args.pmg_zeta_scale
    tau = args.pmg_tau
    if tau <= 0.0:
        omega_d = pmg_omega_n * math.sqrt(max(1e-9, 1.0 - pmg_zeta * pmg_zeta))
        tau = 2.0 * math.pi / omega_d

    series = read_bag_series(row["bag"])
    windows = main_windows(row["bag"])
    state_samples = [(t, s) for t, s in series["state"] if in_windows(t, windows)]
    if not state_samples:
        return {}

    eta_x = float(state_samples[0][1][0])
    eta_x_dot = float(state_samples[0][1][1])
    eta_y = float(state_samples[0][1][2])
    eta_y_dot = float(state_samples[0][1][3])
    last_t = state_samples[0][0]

    h_values = []
    eta_x_values = []
    eta_x_dot_values = []
    eta_dot_values = []
    ax_nom_values = []
    ax_out_values = []
    active_values = []

    for ts, _ in state_samples:
        if ts != last_t:
            dt = min(max(ts - last_t, 0.0), 0.2)
            ax_nom = interp(series["slosh_ax"], last_t)
            ay = interp(series["slosh_ay"], last_t)
            ax_nom = ax_nom if math.isfinite(ax_nom) else 0.0
            ay = ay if math.isfinite(ay) else 0.0
            ax_out, active, _, _ = signed_pmg_u(
                eta_x,
                eta_x_dot,
                ax_nom,
                eta_threshold,
                tau,
                args.pmg_grid,
                pmg_omega_n,
                pmg_zeta,
            )
            eta_x, eta_x_dot = discrete_lateral_step(eta_x, eta_x_dot, ax_out, dt, omega_n, cfg["zeta"])
            eta_y, eta_y_dot = discrete_lateral_step(eta_y, eta_y_dot, ay, dt, omega_n, cfg["zeta"])
            last_t = ts
        else:
            ax_nom = interp(series["slosh_ax"], ts)
            ax_nom = ax_nom if math.isfinite(ax_nom) else 0.0
            ax_out, active, _, _ = signed_pmg_u(
                eta_x,
                eta_x_dot,
                ax_nom,
                eta_threshold,
                tau,
                args.pmg_grid,
                pmg_omega_n,
                pmg_zeta,
            )

        h_values.append(height_coeff * math.hypot(eta_x, eta_y))
        eta_x_values.append(abs(eta_x))
        eta_x_dot_values.append(abs(eta_x_dot))
        eta_dot_values.append(math.hypot(eta_x_dot, eta_y_dot))
        ax_nom_values.append(ax_nom)
        ax_out_values.append(ax_out)
        active_values.append(active)

    return {
        "long_tau_s": tau,
        "long_pmg_omega_scale": args.pmg_omega_scale,
        "long_pmg_zeta_scale": args.pmg_zeta_scale,
        "long_eta_threshold": eta_threshold,
        "long_active_ratio": sum(1 for x in active_values if x) / len(active_values) if active_values else float("nan"),
        "long_h_rms": rms(h_values),
        "long_h_p95": percentile(h_values, 95),
        "long_eta_x_p95": percentile(eta_x_values, 95),
        "long_eta_x_energy_p95": percentile([(omega_n * x) ** 2 for x in eta_x_values], 95),
        "long_eta_x_dot_rms": rms(eta_x_dot_values),
        "long_eta_dot_rms": rms(eta_dot_values),
        "long_ax_nom_abs_p95": percentile([abs(x) for x in ax_nom_values], 95),
        "long_ax_out_abs_p95": percentile([abs(x) for x in ax_out_values], 95),
        "long_ax_delta_abs_mean": mean([abs(a - b) for a, b in zip(ax_nom_values, ax_out_values)]),
    }


def combined_replay_bag(row, cfg, args, eta_x_threshold, eta_y_threshold):
    omega_n, height_coeff = modal_params(cfg)
    pmg_omega_n = omega_n * args.pmg_omega_scale
    pmg_zeta = cfg["zeta"] * args.pmg_zeta_scale
    tau = args.pmg_tau
    if tau <= 0.0:
        omega_d = pmg_omega_n * math.sqrt(max(1e-9, 1.0 - pmg_zeta * pmg_zeta))
        tau = 2.0 * math.pi / omega_d

    series = read_bag_series(row["bag"])
    windows = main_windows(row["bag"])
    state_samples = [(t, s) for t, s in series["state"] if in_windows(t, windows)]
    if not state_samples:
        return {}

    eta_x = float(state_samples[0][1][0])
    eta_x_dot = float(state_samples[0][1][1])
    eta_y = float(state_samples[0][1][2])
    eta_y_dot = float(state_samples[0][1][3])
    last_t = state_samples[0][0]

    h_values = []
    eta_x_values = []
    eta_y_values = []
    eta_dot_values = []
    active_x_values = []
    active_y_values = []
    ax_nom_values = []
    ax_out_values = []
    ay_nom_values = []
    ay_out_values = []

    for ts, _ in state_samples:
        ax_nom = interp(series["slosh_ax"], last_t if ts != last_t else ts)
        ax_nom = ax_nom if math.isfinite(ax_nom) else 0.0
        odom_t = (last_t if ts != last_t else ts) - args.pmg_ay_lag
        odom_v = interp(series["odom_v"], odom_t)
        odom_w = interp(series["odom_w"], odom_t)
        ay_nom = (
            args.pmg_ay_scale * odom_v * odom_w + args.pmg_ay_intercept
            if math.isfinite(odom_v) and math.isfinite(odom_w)
            else 0.0
        )

        ax_out, active_x, _, _ = signed_pmg_u(
            eta_x,
            eta_x_dot,
            ax_nom,
            eta_x_threshold,
            tau,
            args.pmg_grid,
            pmg_omega_n,
            pmg_zeta,
        )
        ay_out, active_y, _, _ = signed_pmg_u(
            eta_y,
            eta_y_dot,
            ay_nom,
            eta_y_threshold,
            tau,
            args.pmg_grid,
            pmg_omega_n,
            pmg_zeta,
        )

        if ts != last_t:
            dt = min(max(ts - last_t, 0.0), 0.2)
            eta_x, eta_x_dot = discrete_lateral_step(eta_x, eta_x_dot, ax_out, dt, omega_n, cfg["zeta"])
            eta_y, eta_y_dot = discrete_lateral_step(eta_y, eta_y_dot, ay_out, dt, omega_n, cfg["zeta"])
            last_t = ts

        h_values.append(height_coeff * math.hypot(eta_x, eta_y))
        eta_x_values.append(abs(eta_x))
        eta_y_values.append(abs(eta_y))
        eta_dot_values.append(math.hypot(eta_x_dot, eta_y_dot))
        active_x_values.append(active_x)
        active_y_values.append(active_y)
        ax_nom_values.append(ax_nom)
        ax_out_values.append(ax_out)
        ay_nom_values.append(ay_nom)
        ay_out_values.append(ay_out)

    active_any = [x or y for x, y in zip(active_x_values, active_y_values)]
    return {
        "combined_tau_s": tau,
        "combined_pmg_omega_scale": args.pmg_omega_scale,
        "combined_pmg_zeta_scale": args.pmg_zeta_scale,
        "combined_eta_x_threshold": eta_x_threshold,
        "combined_eta_y_threshold": eta_y_threshold,
        "combined_active_ratio": sum(1 for x in active_any if x) / len(active_any) if active_any else float("nan"),
        "combined_active_x_ratio": sum(1 for x in active_x_values if x) / len(active_x_values) if active_x_values else float("nan"),
        "combined_active_y_ratio": sum(1 for x in active_y_values if x) / len(active_y_values) if active_y_values else float("nan"),
        "combined_h_rms": rms(h_values),
        "combined_h_p95": percentile(h_values, 95),
        "combined_eta_x_p95": percentile(eta_x_values, 95),
        "combined_eta_y_p95": percentile(eta_y_values, 95),
        "combined_eta_x_energy_p95": percentile([(omega_n * x) ** 2 for x in eta_x_values], 95),
        "combined_eta_y_energy_p95": percentile([(omega_n * y) ** 2 for y in eta_y_values], 95),
        "combined_eta_dot_rms": rms(eta_dot_values),
        "combined_ax_delta_abs_mean": mean([abs(a - b) for a, b in zip(ax_nom_values, ax_out_values)]),
        "combined_ay_delta_abs_mean": mean([abs(a - b) for a, b in zip(ay_nom_values, ay_out_values)]),
    }


def eta_x_p95_for_bag(bag_path):
    windows = main_windows(bag_path)
    series = read_bag_series(bag_path)
    values = [abs(state[0]) for ts, state in series["state"] if in_windows(ts, windows)]
    return percentile(values, 95)


def eta_y_p95_for_bag(bag_path):
    windows = main_windows(bag_path)
    series = read_bag_series(bag_path)
    values = [abs(state[2]) for ts, state in series["state"] if in_windows(ts, windows)]
    return percentile(values, 95)


def interp_bool(series, ts):
    value = interp([(t, 1.0 if v else 0.0) for t, v in series], ts)
    return math.isfinite(value) and value >= 0.5


def write_csv(path, rows):
    if not path or not rows:
        return
    fields = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    cfg = load_config(args.config)
    bags = expand_bags(args.bags)
    if not bags:
        raise SystemExit("No bag files found")

    rows = [analyze_bag(path, cfg, args) for path in bags]
    if args.pmg_replay:
        thresholds = {}
        for path_id in sorted(set(row["path_id"] for row in rows)):
            nom_values = [
                row["measured_lateral_h_p95"] / row["height_coeff"]
                for row in rows
                if row["path_id"] == path_id
                and row["condition"] == "NOM"
                and row["height_coeff"] > 1e-9
                and math.isfinite(row["measured_lateral_h_p95"])
            ]
            if nom_values:
                thresholds[path_id] = percentile(nom_values, 95)
        for row in rows:
            threshold = thresholds.get(row["path_id"], float("nan"))
            if math.isfinite(threshold):
                row.update(pmg_replay_bag(row, cfg, args, threshold))
    if args.longitudinal_replay:
        thresholds = {}
        for path_id in sorted(set(row["path_id"] for row in rows)):
            nom_values = [
                eta_x_p95_for_bag(row["bag"])
                for row in rows
                if row["path_id"] == path_id and row["condition"] == "NOM"
            ]
            nom_values = [v for v in nom_values if math.isfinite(v)]
            if nom_values:
                thresholds[path_id] = percentile(nom_values, 95)
        for row in rows:
            threshold = thresholds.get(row["path_id"], float("nan"))
            if math.isfinite(threshold):
                row.update(longitudinal_replay_bag(row, cfg, args, threshold))
    if args.combined_replay:
        x_thresholds = {}
        y_thresholds = {}
        for path_id in sorted(set(row["path_id"] for row in rows)):
            x_values = [
                eta_x_p95_for_bag(row["bag"])
                for row in rows
                if row["path_id"] == path_id and row["condition"] == "NOM"
            ]
            y_values = [
                eta_y_p95_for_bag(row["bag"])
                for row in rows
                if row["path_id"] == path_id and row["condition"] == "NOM"
            ]
            x_values = [v for v in x_values if math.isfinite(v)]
            y_values = [v for v in y_values if math.isfinite(v)]
            if x_values:
                x_thresholds[path_id] = percentile(x_values, 95)
            if y_values:
                y_thresholds[path_id] = percentile(y_values, 95)
        for row in rows:
            x_threshold = x_thresholds.get(row["path_id"], float("nan"))
            y_threshold = y_thresholds.get(row["path_id"], float("nan"))
            if math.isfinite(x_threshold) and math.isfinite(y_threshold):
                row.update(combined_replay_bag(row, cfg, args, x_threshold, y_threshold))

    for row in rows:
        print(os.path.basename(row["bag"]))
        print(
            "- baseline_replay: "
            f"{row['verdict']} samples={row['samples']} windows={row['windows']} "
            f"measured_h_p95={row['measured_h_p95']:.6g} "
            f"recon_h_p95={row['recon_h_p95']:.6g} "
            f"rel_error={row['h_p95_rel_error']:.3f}"
        )
        print(
            "- lateral_replay: "
            f"measured_p95={row['measured_lateral_h_p95']:.6g} "
            f"recon_p95={row['recon_lateral_h_p95']:.6g} "
            f"rel_error={row['lateral_h_p95_rel_error']:.3f}"
        )
        print(
            "- low_input: "
            f"segments={row['low_input_segments']} total_s={row['low_input_total_s']:.2f}"
        )
        print(
            "- ay_mapping: "
            f"odom_corr={row['odom_ay_corr']:.3f} odom_slope={row['odom_ay_slope']:.3f} "
            f"odom_fit_rmse={row['odom_ay_fit_rmse']:.3f} "
            f"cmd_corr={row['cmd_ay_corr']:.3f} cmd_slope={row['cmd_ay_slope']:.3f} "
            f"cmd_fit_rmse={row['cmd_ay_fit_rmse']:.3f}"
        )
        if args.pmg_replay and "pmg_active_ratio" in row:
            print(
                "- pmg_replay: "
                f"threshold={row['pmg_eta_threshold']:.6g} tau={row['pmg_tau_s']:.3f}s "
                f"active={row['pmg_active_ratio']:.3f} fixed={row['fixed_guard_active_ratio']:.3f} "
                f"overlap={row['pmg_fixed_overlap_jaccard']:.3f} "
                f"h_p95={row['pmg_h_p95']:.6g} ay_p95={row['pmg_ay_out_abs_p95']:.3f}"
            )
        if args.longitudinal_replay and "long_active_ratio" in row:
            print(
                "- longitudinal_replay: "
                f"threshold={row['long_eta_threshold']:.6g} tau={row['long_tau_s']:.3f}s "
                f"active={row['long_active_ratio']:.3f} "
                f"h_p95={row['long_h_p95']:.6g} "
                f"eta_x_energy_p95={row['long_eta_x_energy_p95']:.6g} "
                f"ax_p95={row['long_ax_out_abs_p95']:.3f}"
            )
        if args.combined_replay and "combined_active_ratio" in row:
            print(
                "- combined_replay: "
                f"tau={row['combined_tau_s']:.3f}s "
                f"active={row['combined_active_ratio']:.3f} "
                f"active_x={row['combined_active_x_ratio']:.3f} "
                f"active_y={row['combined_active_y_ratio']:.3f} "
                f"h_p95={row['combined_h_p95']:.6g} "
                f"eta_x_energy_p95={row['combined_eta_x_energy_p95']:.6g}"
            )

    pass_count = sum(1 for row in rows if row["verdict"] == "PASS")
    print(f"\noverall: {pass_count}/{len(rows)} bags passed baseline replay")
    if args.pmg_replay:
        pmg_rows = [row for row in rows if "pmg_active_ratio" in row]
        if pmg_rows:
            print(
                "pmg summary: "
                f"active_mean={mean([r['pmg_active_ratio'] for r in pmg_rows]):.3f} "
                f"overlap_mean={mean([r['pmg_fixed_overlap_jaccard'] for r in pmg_rows]):.3f} "
                f"ay_delta_mean={mean([r['pmg_ay_delta_abs_mean'] for r in pmg_rows]):.3f}"
            )
    if args.longitudinal_replay:
        long_rows = [row for row in rows if "long_active_ratio" in row]
        if long_rows:
            print(
                "longitudinal summary: "
                f"active_mean={mean([r['long_active_ratio'] for r in long_rows]):.3f} "
                f"ax_delta_mean={mean([r['long_ax_delta_abs_mean'] for r in long_rows]):.3f}"
            )
    if args.combined_replay:
        combined_rows = [row for row in rows if "combined_active_ratio" in row]
        if combined_rows:
            print(
                "combined summary: "
                f"active_mean={mean([r['combined_active_ratio'] for r in combined_rows]):.3f} "
                f"active_x_mean={mean([r['combined_active_x_ratio'] for r in combined_rows]):.3f} "
                f"active_y_mean={mean([r['combined_active_y_ratio'] for r in combined_rows]):.3f}"
            )
    write_csv(args.csv, rows)
    if args.csv:
        print(f"csv: {args.csv}")


if __name__ == "__main__":
    main()
