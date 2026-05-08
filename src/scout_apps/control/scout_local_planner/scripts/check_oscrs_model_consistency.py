#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check OSCRS rollout consistency with recorded /slosh/height.

This is the Step-0 prerequisite for OSCRS: the candidate rollout model must
match the observer's height convention before it can be used for hard gates.
"""

import argparse
import bisect
import csv
import math
import os
from pathlib import Path

import rosbag
import yaml

G = 9.81
XI_11 = 1.841


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay /slosh/ax_est and /slosh/ay_est and compare predicted h_total with /slosh/height."
    )
    parser.add_argument("bags", nargs="+", help="Bag files or directories containing .bag files")
    parser.add_argument(
        "--config",
        default="src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml",
        help="OSCRS container YAML",
    )
    parser.add_argument("--csv", default="", help="Optional output CSV path")
    parser.add_argument("--all-segments", action="store_true", help="Use all samples instead of TRACKING + terminal NONE")
    parser.add_argument("--min-rho", type=float, default=0.5, help="PASS threshold for Spearman rho")
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
        "mode_index": int(slosh.get("mode_index", 1)),
        "use_linear_model": bool(slosh.get("use_linear_model", True)),
        "use_parabola_term": bool(slosh.get("use_parabola_term", True)),
        "height_coeff_mode": str(oscrs.get("height_coeff_mode", "observer_linear")),
        "height_coeff_manual": float(oscrs.get("height_coeff_manual", 0.0) or 0.0),
        "dt": float(oscrs.get("rollout_dt", 0.05)),
    }
    return cfg


def modal_params(cfg):
    R = cfg["R"]
    h = cfg["h"]
    rho = cfg["rho"]
    xi = XI_11
    m_f = rho * math.pi * R * R * h
    omega_n = math.sqrt(max(G * (xi / R) * math.tanh(xi * h / R), 0.0))
    m_n = m_f * (2.0 * R * math.tanh(xi * h / R)) / (xi * h * (xi * xi - 1.0))
    mode = cfg["height_coeff_mode"]
    if mode == "manual":
        height_coeff = cfg["height_coeff_manual"]
    elif mode == "ferrari_closed_form":
        height_coeff = (xi * xi * h * m_n) / (m_f * R)
    else:
        # Matches the linear observer convention used by existing offline replay.
        height_coeff = (4.0 * h * m_n) / (m_f * R)
    return omega_n, height_coeff


def read_string_segments(bag_path, topic):
    transitions = []
    try:
        with rosbag.Bag(bag_path) as bag:
            start_time = bag.get_start_time()
            end_time = bag.get_end_time()
            for _, msg, t in bag.read_messages(topics=[topic]):
                value = str(msg.data)
                ts = t.to_sec()
                if not transitions or transitions[-1][1] != value:
                    transitions.append((ts, value))
    except rosbag.ROSBagException:
        return []
    segments = []
    for idx, (ts, value) in enumerate(transitions):
        end = transitions[idx + 1][0] if idx + 1 < len(transitions) else end_time
        segments.append((ts, end, value))
    return segments


def in_segment(segments, ts, wanted):
    if not segments:
        return True
    for start, end, value in segments:
        if start <= ts < end:
            return value == wanted
    return False


def interp(series, ts):
    if not series:
        return float("nan")
    times = [item[0] for item in series]
    values = [item[1] for item in series]
    if ts <= times[0]:
        return values[0]
    if ts >= times[-1]:
        return values[-1]
    idx = bisect.bisect_left(times, ts)
    t0, v0 = times[idx - 1], values[idx - 1]
    t1, v1 = times[idx], values[idx]
    if t1 <= t0:
        return v1
    r = (ts - t0) / (t1 - t0)
    return v0 * (1.0 - r) + v1 * r


def ranks(values):
    indexed = sorted((value, idx) for idx, value in enumerate(values))
    out = [0.0] * len(values)
    i = 0
    while i < len(indexed):
        j = i + 1
        while j < len(indexed) and indexed[j][0] == indexed[i][0]:
            j += 1
        rank = 0.5 * (i + j - 1) + 1.0
        for _, idx in indexed[i:j]:
            out[idx] = rank
        i = j
    return out


def pearson(xs, ys):
    if len(xs) < 5 or len(xs) != len(ys):
        return float("nan")
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    dx = [x - mx for x in xs]
    dy = [y - my for y in ys]
    sx = math.sqrt(sum(x * x for x in dx))
    sy = math.sqrt(sum(y * y for y in dy))
    if sx <= 1e-12 or sy <= 1e-12:
        return float("nan")
    return sum(x * y for x, y in zip(dx, dy)) / (sx * sy)


def spearman(xs, ys):
    if len(xs) < 5 or len(xs) != len(ys):
        return float("nan")
    return pearson(ranks(xs), ranks(ys))


def percentile(values, p):
    values = sorted(v for v in values if not math.isnan(v))
    if not values:
        return float("nan")
    idx = max(0, min(len(values) - 1, int((p / 100.0) * (len(values) - 1))))
    return values[idx]


def read_series(bag_path):
    topics = ["/slosh/ax_est", "/slosh/ay_est", "/slosh/height", "/odom"]
    series = {"ax": [], "ay": [], "height": [], "omega": []}
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            if topic == "/slosh/ax_est":
                series["ax"].append((ts, float(msg.data)))
            elif topic == "/slosh/ay_est":
                series["ay"].append((ts, float(msg.data)))
            elif topic == "/slosh/height":
                series["height"].append((ts, float(msg.data)))
            elif topic == "/odom":
                series["omega"].append((ts, float(msg.twist.twist.angular.z)))
    return series


def replay_height(series, cfg, omega_n, height_coeff, mpc_segments, terminal_segments, all_segments):
    if not series["ax"] or not series["ay"] or not series["height"]:
        return [], []
    start = max(series["ax"][0][0], series["ay"][0][0], series["height"][0][0])
    end = min(series["ax"][-1][0], series["ay"][-1][0], series["height"][-1][0])
    if end <= start:
        return [], []
    dt = max(0.005, cfg["dt"])
    damping = 2.0 * cfg["zeta"] * omega_n
    wn2 = omega_n * omega_n
    radius2_over_4g = (cfg["R"] * cfg["R"]) / (4.0 * G)
    eta_x = eta_x_dot = eta_y = eta_y_dot = 0.0
    pred = []
    obs = []
    steps = int(math.floor((end - start) / dt))
    for step in range(steps + 1):
        ts = start + step * dt
        ax = interp(series["ax"], ts)
        ay = interp(series["ay"], ts)
        omega = interp(series["omega"], ts)
        ddx = -damping * eta_x_dot - wn2 * eta_x - ax
        ddy = -damping * eta_y_dot - wn2 * eta_y - ay
        eta_x_dot += ddx * dt
        eta_y_dot += ddy * dt
        eta_x += eta_x_dot * dt
        eta_y += eta_y_dot * dt
        if not all_segments:
            if not in_segment(mpc_segments, ts, "TRACKING"):
                continue
            if not in_segment(terminal_segments, ts, "NONE"):
                continue
        modal = height_coeff * math.hypot(eta_x, eta_y)
        parabola = radius2_over_4g * omega * omega if cfg["use_parabola_term"] else 0.0
        measured = interp(series["height"], ts)
        if not math.isnan(measured):
            pred.append(modal + parabola)
            obs.append(measured)
    return pred, obs


def analyze_bag(bag_path, cfg, omega_n, height_coeff, all_segments):
    series = read_series(bag_path)
    mpc_segments = read_string_segments(bag_path, "/mpc_status")
    terminal_segments = read_string_segments(bag_path, "/mpc/terminal_mode")
    pred, obs = replay_height(series, cfg, omega_n, height_coeff, mpc_segments, terminal_segments, all_segments)
    rho = spearman(pred, obs)
    rel_p95_err = float("nan")
    if pred and obs:
        obs_p95 = percentile(obs, 95.0)
        if abs(obs_p95) > 1e-12:
            rel_p95_err = abs(percentile(pred, 95.0) - obs_p95) / abs(obs_p95)
    return {
        "bag": bag_path,
        "samples": len(pred),
        "spearman_rho_pred_vs_obs": rho,
        "pred_h_p95": percentile(pred, 95.0),
        "obs_h_p95": percentile(obs, 95.0),
        "rel_p95_err": rel_p95_err,
    }


def main():
    args = parse_args()
    bags = expand_bags(args.bags)
    if not bags:
        raise SystemExit("No bag files found")
    cfg = load_config(args.config)
    omega_n, height_coeff = modal_params(cfg)
    rows = []
    for bag_path in bags:
        row = analyze_bag(bag_path, cfg, omega_n, height_coeff, args.all_segments)
        row.update(
            {
                "omega_n_used": omega_n,
                "zeta_used": cfg["zeta"],
                "height_coeff_used": height_coeff,
                "R_m_used": cfg["R"],
                "h_m_used": cfg["h"],
                "pass_rho": int(row["spearman_rho_pred_vs_obs"] >= args.min_rho),
            }
        )
        rows.append(row)
        print(
            f"{Path(bag_path).name}: samples={row['samples']} "
            f"rho={row['spearman_rho_pred_vs_obs']:.3f} "
            f"pred_h_p95={row['pred_h_p95']:.6g} obs_h_p95={row['obs_h_p95']:.6g} "
            f"rel_p95_err={row['rel_p95_err']:.3f} pass={bool(row['pass_rho'])}"
        )
    passed = sum(row["pass_rho"] for row in rows)
    print(
        f"overall: {passed}/{len(rows)} pass rho>={args.min_rho}; "
        f"omega_n={omega_n:.6g}; height_coeff={height_coeff:.6g}; zeta={cfg['zeta']:.6g}"
    )
    if args.csv:
        fieldnames = [
            "bag",
            "samples",
            "spearman_rho_pred_vs_obs",
            "pred_h_p95",
            "obs_h_p95",
            "rel_p95_err",
            "omega_n_used",
            "zeta_used",
            "height_coeff_used",
            "R_m_used",
            "h_m_used",
            "pass_rho",
        ]
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        with open(args.csv, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"csv: {args.csv}")


if __name__ == "__main__":
    main()
