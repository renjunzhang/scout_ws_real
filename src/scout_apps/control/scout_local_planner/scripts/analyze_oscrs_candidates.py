#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline OSCRS Step-0 candidate evaluator.

This script rebuilds GeoRef candidates from recorded raw paths, rolls out the
observer-compatible slosh height model, and sweeps eta_lim thresholds before
any online OSCRS code is added.
"""

import argparse
import csv
import math
import os
import re
from pathlib import Path

import rosbag
import yaml

from extract_slosh_metrics import get_status_segments, get_string_segments, in_terminal_none, in_tracking, percentile, rms, safe_max
from generate_anti_slosh_path_candidates import cumulative_s, curvature_series, dist, dkappa_series, path_metrics, resample_path, smooth_path

G = 9.81
XI_11 = 1.841


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate OSCRS hard-gate feasibility for recorded paths.")
    parser.add_argument("bags", nargs="+", help="Bag files or directories")
    parser.add_argument("--config", default="src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml")
    parser.add_argument("--csv", default="docs/Claude/分析数据/oscrs_candidates.csv")
    parser.add_argument("--raw-topic", default="/scout/global_path")
    parser.add_argument("--selected-topic", default="/scout/global_path_anti_slosh")
    parser.add_argument("--ds", type=float, default=0.10)
    parser.add_argument("--max-candidate-level", default="strong", choices=("original", "mild", "medium", "mid", "strong"))
    parser.add_argument("--min-segment-length", type=float, default=0.02)
    parser.add_argument("--max-drift", type=float, default=0.18)
    parser.add_argument("--max-length-ratio", type=float, default=1.15)
    parser.add_argument("--min-length-ratio", type=float, default=0.995)
    parser.add_argument("--min-kappa-ratio", type=float, default=0.20)
    parser.add_argument("--max-endpoint-error", type=float, default=0.05)
    parser.add_argument("--ay-ratio-limit", type=float, default=1.0)
    parser.add_argument("--predict-v-max", type=float, default=2.0)
    parser.add_argument("--predict-ay-max", type=float, default=2.0)
    parser.add_argument("--predict-a-max", type=float, default=1.0)
    parser.add_argument("--predict-v-init", type=float, default=0.0)
    parser.add_argument("--eta-lims-mm", default="15,20,25,30")
    return parser.parse_args()


def expand_bags(paths):
    out = []
    for item in paths:
        if os.path.isdir(item):
            for name in sorted(os.listdir(item)):
                if name.endswith(".bag"):
                    out.append(os.path.join(item, name))
        elif os.path.isfile(item) and item.endswith(".bag"):
            out.append(item)
    return sorted(dict.fromkeys(out))


def load_config(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    slosh = data.get("slosh", {}) or {}
    oscrs = data.get("oscrs", {}) or {}
    cfg = {
        "R": float(slosh.get("container_radius", 0.0185)),
        "h": float(slosh.get("liquid_height", 0.058)),
        "rho": float(slosh.get("liquid_density", 1000.0)),
        "zeta": float(slosh.get("damping_ratio", 0.05)),
        "offset_x": float(slosh.get("offset_x", 0.0)),
        "offset_y": float(slosh.get("offset_y", 0.0)),
        "use_linear_model": bool(slosh.get("use_linear_model", True)),
        "use_parabola_term": bool(slosh.get("use_parabola_term", True)),
        "height_coeff_mode": str(oscrs.get("height_coeff_mode", "observer_linear")),
        "height_coeff_manual": float(oscrs.get("height_coeff_manual", 0.0) or 0.0),
        "rollout_dt": float(oscrs.get("rollout_dt", 0.05)),
        "v_floor": float(oscrs.get("v_floor", 0.05)),
        "settle_duration": float(oscrs.get("settle_duration", 2.0)),
        "residual_ratio": float(oscrs.get("residual_ratio", 0.2)),
    }
    return cfg


def modal_params(cfg):
    R = cfg["R"]
    h = cfg["h"]
    rho = cfg["rho"]
    xi = XI_11
    m_f = rho * math.pi * R * R * h
    m_n = m_f * (2.0 * R * math.tanh(xi * h / R)) / (xi * h * (xi * xi - 1.0))
    omega_n = math.sqrt(max(G * (xi / R) * math.tanh(xi * h / R), 0.0))
    mode = cfg["height_coeff_mode"]
    if mode == "manual":
        height_coeff = cfg["height_coeff_manual"]
    elif mode == "ferrari_closed_form":
        height_coeff = (xi * xi * h * m_n) / (m_f * R)
    else:
        height_coeff = (4.0 * h * m_n) / (m_f * R)
    return omega_n, height_coeff


def points_from_path(msg):
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def sanitize_points(points, min_segment_length):
    if not points:
        return []
    out = [points[0]]
    for point in points[1:]:
        if dist(out[-1], point) >= min_segment_length:
            out.append(point)
    return out


def latest_path_from_bag(bag_path, topic):
    latest = None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            pts = points_from_path(msg)
            if len(pts) >= 2:
                latest = pts
    return latest


def selected_candidate_from_report(bag_path):
    latest = ""
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=["/anti_slosh_path/candidate_report"]):
            latest = str(msg.data)
    if not latest:
        return ""
    summary = re.search(r"summary:selected=([^,;]+)", latest)
    if summary:
        return summary.group(1).strip()
    best = None
    for item in latest.split(";"):
        item = item.strip()
        if not item or ":" not in item:
            continue
        name, payload = item.split(":", 1)
        accepted = re.search(r"accepted=([01])", payload)
        score = re.search(r"score=([-+0-9.eE]+)", payload)
        if not accepted or accepted.group(1) != "1" or not score:
            continue
        value = float(score.group(1))
        if best is None or value < best[1]:
            best = (name.strip(), value)
    return best[0] if best else ""


def endpoint_error(points, reference):
    if not points or not reference:
        return float("inf")
    return max(dist(points[0], reference[0]), dist(points[-1], reference[-1]))


def direction_preserved(points, reference):
    if len(points) < 2 or len(reference) < 2:
        return False
    ax = points[-1][0] - points[0][0]
    ay = points[-1][1] - points[0][1]
    bx = reference[-1][0] - reference[0][0]
    by = reference[-1][1] - reference[0][1]
    return ax * bx + ay * by >= 0.0


def mean_nearest_distance(a, b):
    if not a or not b:
        return float("inf")
    return sum(min(dist(point, other) for other in b) for point in a) / len(a)


def safe_ratio(value, reference):
    if abs(reference) < 1e-9:
        return 1.0 if abs(value) < 1e-9 else 1e6
    return value / reference


def interp_piecewise(times, values, query_t):
    if not times:
        return 0.0
    if query_t <= times[0]:
        return values[0]
    if query_t >= times[-1]:
        return values[-1]
    lo = 0
    hi = len(times) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] < query_t:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    t0 = times[i - 1]
    t1 = times[i]
    if t1 <= t0:
        return values[i]
    r = (query_t - t0) / (t1 - t0)
    return values[i - 1] * (1.0 - r) + values[i] * r


def forward_profile(points, args, cfg):
    s = cumulative_s(points)
    kappa = curvature_series(points)
    dkappa = dkappa_series(points, kappa)
    v_prev = min(args.predict_v_init, args.predict_v_max)
    v_values = []
    ax_values = []
    ay_values = []
    for i, k in enumerate(kappa):
        ds = max(0.0, s[i] - s[i - 1]) if i > 0 else 0.0
        v_curv = math.sqrt(args.predict_ay_max / abs(k)) if abs(k) > 1e-6 else args.predict_v_max
        v_accel = math.sqrt(max(0.0, v_prev * v_prev + 2.0 * args.predict_a_max * ds))
        v = min(args.predict_v_max, v_curv, v_accel)
        ax = (v * v - v_prev * v_prev) / (2.0 * ds) if ds > 1e-6 else 0.0
        omega = v * k
        alpha = ax * k + v * v * dkappa[i]
        ax_eff = ax - alpha * cfg["offset_y"] - omega * omega * cfg["offset_x"]
        ay_eff = v * v * k + alpha * cfg["offset_x"] - omega * omega * cfg["offset_y"]
        v_values.append(v)
        ax_values.append(ax_eff)
        ay_values.append(ay_eff)
        v_prev = v
    return s, kappa, v_values, ax_values, ay_values


def rollout_candidate(points, args, cfg, omega_n, height_coeff):
    if len(points) < 3:
        return {}
    s, kappa, v_values, ax_values, ay_values = forward_profile(points, args, cfg)
    times = [0.0]
    for i in range(1, len(s)):
        ds = max(0.0, s[i] - s[i - 1])
        times.append(times[-1] + ds / max(cfg["v_floor"], v_values[i]))
    omega_values = [v * k for v, k in zip(v_values, kappa)]
    dkappa = dkappa_series(points, kappa)
    jerk = []
    for i in range(1, len(ax_values)):
        ds = max(1e-6, s[i] - s[i - 1])
        jerk.append(abs(v_values[i] * (ax_values[i] - ax_values[i - 1]) / ds))

    eta_x = eta_x_dot = eta_y = eta_y_dot = 0.0
    height_tracking = []
    height_all = []
    eta_dot = []
    energy = []
    wn2 = omega_n * omega_n
    damping = 2.0 * cfg["zeta"] * omega_n
    radius2_over_4g = cfg["R"] * cfg["R"] / (4.0 * G)
    dt = max(0.005, cfg["rollout_dt"])
    t_end = times[-1] if times else 0.0
    t_final = t_end + max(0.0, cfg["settle_duration"])
    steps = max(1, int(math.ceil(t_final / dt)))
    for step in range(steps + 1):
        query_t = step * dt
        in_motion = query_t <= t_end
        if in_motion:
            ux = interp_piecewise(times, ax_values, query_t)
            uy = interp_piecewise(times, ay_values, query_t)
            omega = interp_piecewise(times, omega_values, query_t)
        else:
            ux = uy = omega = 0.0
        ddx = -damping * eta_x_dot - wn2 * eta_x - ux
        ddy = -damping * eta_y_dot - wn2 * eta_y - uy
        eta_x_dot += ddx * dt
        eta_y_dot += ddy * dt
        eta_x += eta_x_dot * dt
        eta_y += eta_y_dot * dt
        modal = height_coeff * math.hypot(eta_x, eta_y)
        parabola = radius2_over_4g * omega * omega if cfg["use_parabola_term"] else 0.0
        h_total = modal + parabola
        height_all.append(h_total)
        if in_motion:
            height_tracking.append(h_total)
            eta_dot.append(math.hypot(eta_x_dot, eta_y_dot))
            energy.append(wn2 * (eta_x * eta_x + eta_y * eta_y) + eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot)
    residual = height_all[len(height_tracking):] if len(height_all) > len(height_tracking) else []
    return {
        "time_proxy": t_end,
        "h_max_pred": safe_max(height_tracking),
        "h_p95_pred": percentile(height_tracking, 95.0),
        "h_residual_max_pred": safe_max(residual),
        "h_terminal_pred": height_tracking[-1] if height_tracking else float("nan"),
        "eta_dot_rms_pred": rms(eta_dot),
        "energy_rms_pred": rms(energy),
        "ay_p95_pred": percentile([abs(v) for v in ay_values], 95.0),
        "jerk_p95_pred": percentile(jerk, 95.0),
        "omega_n_used": omega_n,
        "zeta_used": cfg["zeta"],
        "height_coeff_used": height_coeff,
        "R_m_used": cfg["R"],
        "h_m_used": cfg["h"],
        "rollout_dt_used": dt,
        "settle_duration_used": cfg["settle_duration"],
        "kappa_p95": percentile([abs(v) for v in kappa], 95.0),
        "dkappa_p95": percentile([abs(v) for v in dkappa], 95.0),
    }


def closed_loop_metrics(bag_path):
    _, _, _, mpc_segments = get_status_segments(bag_path)
    _, _, _, terminal_segments = get_string_segments(bag_path, "/mpc/terminal_mode")
    height = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=["/slosh/height"]):
            ts = t.to_sec()
            if in_tracking(mpc_segments, ts) and in_terminal_none(terminal_segments, ts):
                height.append(float(msg.data))
    return {"h_p95_closed_loop": percentile(height, 95.0), "h_max_closed_loop": safe_max(height)}


def build_candidates(raw, args):
    base = resample_path(sanitize_points(raw, args.min_segment_length), max(0.02, args.ds))
    specs = [
        ("original", 0, 0.0, 0.0),
        ("mild", 18, 0.35, 0.08),
        ("medium", 40, 0.45, 0.12),
        ("mid", 24, 0.30, 0.07),
        ("strong", 56, 0.55, 0.18),
    ]
    levels = {name: index for index, (name, _, _, _) in enumerate(specs)}
    out = []
    for name, iters, gain, drift in specs:
        points = base if name == "original" else smooth_path(base, iters, gain, drift)
        points = sanitize_points(points, args.min_segment_length)
        out.append((name, points, levels[name], levels[args.max_candidate_level], base))
    return out


def evaluate_bag(bag_path, args, cfg, omega_n, height_coeff, eta_lims_mm):
    raw = latest_path_from_bag(bag_path, args.raw_topic)
    if raw is None:
        raise RuntimeError(f"{bag_path}: no raw path on {args.raw_topic}")
    selected = latest_path_from_bag(bag_path, args.selected_topic)
    candidates = build_candidates(raw, args)
    base = candidates[0][4]
    base_metrics = path_metrics(base, base)
    base_length = max(1e-6, base_metrics["length_m"])
    base_rollout = rollout_candidate(base, args, cfg, omega_n, height_coeff)
    closed = closed_loop_metrics(bag_path)
    report_selected = selected_candidate_from_report(bag_path)
    selected_name = report_selected or "original"
    if selected and not report_selected:
        selected_name = min(
            ((name, mean_nearest_distance(points, selected)) for name, points, _, _, _ in candidates),
            key=lambda item: item[1],
        )[0]
    rows = []
    raw_time_proxy = max(1e-9, base_rollout.get("time_proxy", float("inf")))
    for name, points, level, max_level, _ in candidates:
        metrics = path_metrics(points, base)
        rollout = rollout_candidate(points, args, cfg, omega_n, height_coeff)
        length_ratio = metrics["length_m"] / base_length
        base_ay = base_rollout.get("ay_p95_pred", float("nan"))
        ay_ratio = safe_ratio(rollout.get("ay_p95_pred", float("nan")), base_ay)
        geom_reasons = []
        if len(points) < 3:
            geom_reasons.append("too_few_points")
        if metrics["min_seg_m"] < args.min_segment_length:
            geom_reasons.append("min_seg")
        if metrics["max_drift_m"] > args.max_drift:
            geom_reasons.append("drift")
        if length_ratio > args.max_length_ratio:
            geom_reasons.append("length_high")
        if length_ratio < args.min_length_ratio:
            geom_reasons.append("length_low")
        if level > max_level:
            geom_reasons.append("level")
        if ay_ratio > args.ay_ratio_limit:
            geom_reasons.append("ay")
        if endpoint_error(points, base) > args.max_endpoint_error:
            geom_reasons.append("endpoint")
        if not direction_preserved(points, base):
            geom_reasons.append("direction")
        geom_pass = not geom_reasons
        for eta_lim_mm in eta_lims_mm:
            eta_lim = eta_lim_mm / 1000.0
            residual_lim = cfg["residual_ratio"] * eta_lim
            height_pass = rollout.get("h_max_pred", float("inf")) <= eta_lim
            residual_pass = rollout.get("h_residual_max_pred", float("inf")) <= residual_lim
            violation = (
                max(0.0, rollout.get("h_max_pred", float("inf")) / max(eta_lim, 1e-9) - 1.0)
                + max(0.0, rollout.get("h_residual_max_pred", float("inf")) / max(residual_lim, 1e-9) - 1.0)
                + 0.2 * max(0.0, rollout.get("time_proxy", float("inf")) / (1.15 * raw_time_proxy) - 1.0)
            )
            row = {
                "bag": bag_path,
                "condition": infer_condition(bag_path),
                "candidate": name,
                "selected_by_existing": int(name == selected_name),
                "eta_lim_mm": eta_lim_mm,
                "eta_p95_lim_mm": "",
                "residual_ratio": cfg["residual_ratio"],
                "geom_pass": int(geom_pass),
                "collision_pass": 1,
                "height_pass": int(height_pass),
                "residual_pass": int(residual_pass),
                "oscrs_feasible": int(geom_pass and height_pass and residual_pass),
                "violation": violation,
                "time_violation": max(0.0, rollout.get("time_proxy", float("inf")) / (1.15 * raw_time_proxy) - 1.0),
                "length_ratio": length_ratio,
                "drift_max": metrics["max_drift_m"],
                "reject_reason": "accepted" if geom_pass else "|".join(geom_reasons),
                "collision_check_mode": "not_evaluated_offline",
                **rollout,
                **(closed if name == selected_name else {}),
            }
            rows.append(row)
    annotate_fallback_codes(rows, eta_lims_mm)
    return rows


def annotate_fallback_codes(rows, eta_lims_mm):
    for eta_lim_mm in eta_lims_mm:
        group = [row for row in rows if row["eta_lim_mm"] == eta_lim_mm]
        full = [row for row in group if row["candidate"] != "original" and row["oscrs_feasible"]]
        original = [row for row in group if row["candidate"] == "original"]
        original_safe = bool(original and original[0]["height_pass"] and original[0]["residual_pass"])
        geom = [row for row in group if row["candidate"] != "original" and row["geom_pass"]]
        if full:
            selected = min(full, key=lambda row: row["violation"])
            fallback_code = 0
        elif original_safe:
            selected = original[0]
            fallback_code = 1
        elif geom:
            selected = min(geom, key=lambda row: row["violation"])
            fallback_code = 2
        else:
            selected = min(group, key=lambda row: row["violation"]) if group else None
            fallback_code = 3
        for row in group:
            row["offline_oscrs_selected"] = int(selected is not None and row["candidate"] == selected["candidate"])
            row["fallback_code"] = fallback_code


def infer_condition(path):
    name = os.path.basename(path)
    for key in ("GEOREF_TUNED", "RAW_TUNED", "RAW_SLOW", "GEOREF_ORIGINAL", "GEOREF_SLOSH_SCORE"):
        if key in name:
            return key
    return "UNKNOWN"


def write_csv(path, rows):
    if not rows:
        return
    keys = []
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    bags = expand_bags(args.bags)
    if not bags:
        raise SystemExit("No bag files found")
    cfg = load_config(args.config)
    omega_n, height_coeff = modal_params(cfg)
    eta_lims_mm = [float(item) for item in args.eta_lims_mm.split(",") if item.strip()]
    all_rows = []
    for bag_path in bags:
        rows = evaluate_bag(bag_path, args, cfg, omega_n, height_coeff, eta_lims_mm)
        all_rows.extend(rows)
        feasible = sum(1 for row in rows if row["oscrs_feasible"])
        selected_rows = [row for row in rows if row["selected_by_existing"] and row["eta_lim_mm"] == eta_lims_mm[-1]]
        selected = selected_rows[0]["candidate"] if selected_rows else "unknown"
        print(f"{Path(bag_path).name}: rows={len(rows)} feasible={feasible} selected_existing={selected}")
    write_csv(args.csv, all_rows)
    print(f"csv: {args.csv}")
    print(f"omega_n={omega_n:.6g} height_coeff={height_coeff:.6g} zeta={cfg['zeta']:.6g}")


if __name__ == "__main__":
    main()
