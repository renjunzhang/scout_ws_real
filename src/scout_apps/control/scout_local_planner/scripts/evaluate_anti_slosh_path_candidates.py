#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
import os

import yaml


XI_11 = 1.8412
G = 9.81


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline signed slosh rollout and geometry proxy for anti-slosh path candidates."
    )
    parser.add_argument("inputs", nargs="+", help="candidate/original fixed path JSON files")
    parser.add_argument(
        "--config",
        default="src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml",
        help="Planner YAML used for slosh parameters",
    )
    parser.add_argument("--csv", default="", help="optional CSV output")
    parser.add_argument("--v-ref", type=float, default=1.2, help="constant proxy speed")
    parser.add_argument(
        "--timing-mode",
        choices=("constant", "low_jerk", "both"),
        default="both",
        help="Timing profile used for rollout. Default: both.",
    )
    parser.add_argument("--ramp-length", type=float, default=1.0, help="low_jerk start/end ramp length in meters")
    parser.add_argument("--v-floor", type=float, default=0.15, help="minimum speed used for rollout dt")
    parser.add_argument("--ds", type=float, default=0.05, help="rollout resampling spacing")
    parser.add_argument("--dt-max", type=float, default=0.02, help="max internal rollout step")
    return parser.parse_args()


def load_config(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    slosh = data.get("slosh", {}) or {}
    return {
        "R": float(slosh.get("container_radius", 0.0185)),
        "h": float(slosh.get("liquid_height", 0.058)),
        "rho": float(slosh.get("liquid_density", 1000.0)),
        "zeta": float(slosh.get("damping_ratio", 0.05)),
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


def load_path(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    points = [(float(p["x"]), float(p["y"])) for p in data.get("poses", [])]
    if len(points) < 3:
        raise RuntimeError(f"{path}: path needs at least 3 poses")
    return data, points


def candidate_name(data, path):
    if "candidate" in data:
        return str(data["candidate"])
    return "original"


def path_id_from_file(path):
    name = os.path.splitext(os.path.basename(path))[0]
    for suffix in ("_smooth", "_radius"):
        if name.endswith(suffix):
            return name[: -len(suffix)]
    return name


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def cumulative_s(points):
    s = [0.0]
    for i in range(1, len(points)):
        s.append(s[-1] + dist(points[i - 1], points[i]))
    return s


def interpolate(points, s_values, target_s):
    if target_s <= 0.0:
        return points[0]
    if target_s >= s_values[-1]:
        return points[-1]
    lo = 0
    hi = len(s_values) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if s_values[mid] < target_s:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    s0 = s_values[i - 1]
    s1 = s_values[i]
    r = 0.0 if s1 <= s0 else (target_s - s0) / (s1 - s0)
    return (
        points[i - 1][0] * (1.0 - r) + points[i][0] * r,
        points[i - 1][1] * (1.0 - r) + points[i][1] * r,
    )


def resample(points, ds):
    s_values = cumulative_s(points)
    total = s_values[-1]
    n = max(2, int(math.ceil(total / ds)) + 1)
    return [interpolate(points, s_values, min(total, i * ds)) for i in range(n)]


def curvature(points):
    out = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        ax, ay = points[i - 1]
        bx, by = points[i]
        cx, cy = points[i + 1]
        ab = dist(points[i - 1], points[i])
        bc = dist(points[i], points[i + 1])
        ac = dist(points[i - 1], points[i + 1])
        denom = ab * bc * ac
        if denom <= 1e-9:
            continue
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        out[i] = 2.0 * cross / denom
    return out


def dkappa(points, kappa):
    s = cumulative_s(points)
    out = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        ds = max(1e-6, s[i + 1] - s[i - 1])
        out[i] = (kappa[i + 1] - kappa[i - 1]) / ds
    return out


def percentile(values, pct):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    pos = (len(values) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    r = pos - lo
    return values[lo] * (1.0 - r) + values[hi] * r


def rms(values):
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def modal_step(eta, eta_dot, u, dt, omega_n, zeta):
    def deriv(x0, x1):
        return x1, -2.0 * zeta * omega_n * x1 - omega_n * omega_n * x0 - u

    k1 = deriv(eta, eta_dot)
    k2 = deriv(eta + 0.5 * dt * k1[0], eta_dot + 0.5 * dt * k1[1])
    k3 = deriv(eta + 0.5 * dt * k2[0], eta_dot + 0.5 * dt * k2[1])
    k4 = deriv(eta + dt * k3[0], eta_dot + dt * k3[1])
    eta += (dt / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
    eta_dot += (dt / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    return eta, eta_dot


def smootherstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)


def speed_profile(s_values, v_ref, mode, ramp_length):
    if mode == "constant":
        return [v_ref for _ in s_values]
    total = s_values[-1] if s_values else 0.0
    ramp = max(1e-6, ramp_length)
    values = []
    for s in s_values:
        start_scale = smootherstep(s / ramp)
        end_scale = smootherstep((total - s) / ramp)
        values.append(v_ref * min(1.0, start_scale, end_scale))
    return values


def rollout(points, v_ref, timing_mode, ramp_length, v_floor, omega_n, zeta, height_coeff, dt_max):
    kappa = curvature(points)
    dk = dkappa(points, kappa)
    s_values = cumulative_s(points)
    v_values = speed_profile(s_values, v_ref, timing_mode, ramp_length)
    ds_values = [dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    eta_x = eta_x_dot = 0.0
    eta_y = eta_y_dot = 0.0
    h_values = []
    eta_dot_values = []
    energy_values = []
    ay_values = []
    alpha_values = []
    ax_values = []
    jerk_values = []
    time_proxy = 0.0
    prev_ax = None

    for i in range(len(points)):
        eta_norm = math.hypot(eta_x, eta_y)
        eta_dot_norm = math.hypot(eta_x_dot, eta_y_dot)
        h_values.append(height_coeff * eta_norm)
        eta_dot_values.append(eta_dot_norm)
        energy_values.append(omega_n * omega_n * eta_norm * eta_norm + eta_dot_norm * eta_dot_norm)

        if i + 1 >= len(points):
            break
        ds = max(1e-6, ds_values[i])
        v0 = v_values[i]
        v1 = v_values[i + 1]
        v_mid = max(v_floor, 0.5 * (v0 + v1))
        dt = ds / max(1e-6, v_mid)
        time_proxy += dt
        steps = max(1, int(math.ceil(dt / dt_max)))
        sub_dt = dt / steps
        ax = (v1 * v1 - v0 * v0) / (2.0 * ds)
        ay = v_mid * v_mid * kappa[i]
        alpha = ax * kappa[i] + v_mid * v_mid * dk[i]
        ax_values.append(abs(ax))
        ay_values.append(abs(ay))
        alpha_values.append(abs(alpha))
        if prev_ax is not None:
            jerk_values.append(abs((ax - prev_ax) / dt))
        prev_ax = ax
        for _ in range(steps):
            eta_x, eta_x_dot = modal_step(eta_x, eta_x_dot, ax, sub_dt, omega_n, zeta)
            eta_y, eta_y_dot = modal_step(eta_y, eta_y_dot, ay, sub_dt, omega_n, zeta)

    return {
        "v_min": min(v_values) if v_values else float("nan"),
        "time_proxy_s": time_proxy,
        "h_p95": percentile(h_values, 95.0),
        "h_rms": rms(h_values),
        "eta_dot_p95": percentile(eta_dot_values, 95.0),
        "eta_dot_rms": rms(eta_dot_values),
        "energy_p95": percentile(energy_values, 95.0),
        "energy_rms": rms(energy_values),
        "ay_p95": percentile(ay_values, 95.0),
        "ay_max": max(ay_values) if ay_values else float("nan"),
        "alpha_p95": percentile(alpha_values, 95.0),
        "alpha_max": max(alpha_values) if alpha_values else float("nan"),
        "ax_p95": percentile(ax_values, 95.0),
        "ax_max": max(ax_values) if ax_values else float("nan"),
        "jerk_p95": percentile(jerk_values, 95.0),
        "jerk_max": max(jerk_values) if jerk_values else float("nan"),
    }


def evaluate_path(path, cfg, args, timing_mode):
    data, raw_points = load_path(path)
    points = resample(raw_points, max(0.01, args.ds))
    omega_n, height_coeff = modal_params(cfg)
    result = rollout(
        points,
        args.v_ref,
        timing_mode,
        args.ramp_length,
        args.v_floor,
        omega_n,
        cfg["zeta"],
        height_coeff,
        args.dt_max,
    )
    kappa_values = curvature(points)
    dk_values = dkappa(points, kappa_values)
    segs = [dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    row = {
        "path_id": path_id_from_file(path),
        "candidate": candidate_name(data, path),
        "path": path,
        "n": len(points),
        "length_m": sum(segs),
        "v_ref": args.v_ref,
        "timing_mode": timing_mode,
        "ramp_length_m": args.ramp_length if timing_mode == "low_jerk" else 0.0,
        "omega_n": omega_n,
        "zeta": cfg["zeta"],
        "kappa_p95": percentile([abs(v) for v in kappa_values], 95.0),
        "kappa_max": max(abs(v) for v in kappa_values),
        "dkappa_p95": percentile([abs(v) for v in dk_values], 95.0),
        "dkappa_max": max(abs(v) for v in dk_values),
    }
    row.update(result)
    return row


def fmt(value):
    return "nan" if not math.isfinite(value) else f"{value:.4g}"


def print_rows(rows):
    baselines = {}
    for row in rows:
        if row["candidate"] in ("original", "original_resampled") and row["timing_mode"] == "constant":
            baselines[row["path_id"]] = row
    for row in rows:
        base = baselines.get(row["path_id"])
        h_delta = float("nan")
        e_delta = float("nan")
        ay_delta = float("nan")
        if base:
            h_delta = 100.0 * (row["h_p95"] - base["h_p95"]) / max(1e-12, base["h_p95"])
            e_delta = 100.0 * (row["energy_p95"] - base["energy_p95"]) / max(1e-12, base["energy_p95"])
            ay_delta = 100.0 * (row["ay_p95"] - base["ay_p95"]) / max(1e-12, base["ay_p95"])
        print(
            f"{row['path_id']} {row['candidate']}: "
            f"timing={row['timing_mode']} "
            f"time={fmt(row['time_proxy_s'])}s "
            f"k_p95={fmt(row['kappa_p95'])} dk_p95={fmt(row['dkappa_p95'])} "
            f"h_p95={fmt(row['h_p95'])} ({fmt(h_delta)}%) "
            f"E_p95={fmt(row['energy_p95'])} ({fmt(e_delta)}%) "
            f"eta_dot_p95={fmt(row['eta_dot_p95'])} "
            f"ay_p95={fmt(row['ay_p95'])} ({fmt(ay_delta)}%) "
            f"ax_p95={fmt(row['ax_p95'])} "
            f"ax_max={fmt(row['ax_max'])} "
            f"jerk_p95={fmt(row['jerk_p95'])} "
            f"jerk_max={fmt(row['jerk_max'])} "
            f"alpha_p95={fmt(row['alpha_p95'])}"
        )


def write_csv(path, rows):
    if not path or not rows:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    cfg = load_config(args.config)
    timing_modes = ["constant", "low_jerk"] if args.timing_mode == "both" else [args.timing_mode]
    rows = []
    for path in args.inputs:
        for timing_mode in timing_modes:
            rows.append(evaluate_path(path, cfg, args, timing_mode))
    print_rows(rows)
    write_csv(args.csv, rows)
    if args.csv:
        print(f"csv: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
