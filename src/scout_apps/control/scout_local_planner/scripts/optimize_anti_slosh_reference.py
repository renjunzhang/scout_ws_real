#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os

from evaluate_anti_slosh_path_candidates import (
    cumulative_s,
    curvature,
    dist,
    dkappa,
    load_config,
    load_path,
    modal_params,
    modal_step,
    percentile,
    resample,
    rms,
)


KNOWN_PATHS = (
    "P2_s_curve",
    "P3_mixed",
    "P1_single_turn",
    "P0_straight",
)


def parse_float_list(text):
    return [float(item) for item in text.split(",") if item.strip()]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline constrained anti-slosh reference candidate search."
    )
    parser.add_argument("--metrics-csv", required=True, help="CSV from extract_slosh_metrics.py")
    parser.add_argument("--baseline-csv", default="", help="optional derived NOM baseline CSV output")
    parser.add_argument("--csv", required=True, help="candidate sweep CSV output")
    parser.add_argument("--path-file", action="append", required=True, help="fixed path JSON; repeatable")
    parser.add_argument(
        "--config",
        default="src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml",
        help="Planner YAML used for slosh parameters",
    )
    parser.add_argument("--cruise-speeds", default="1.6,1.8,2.0,2.2,2.4")
    parser.add_argument("--ay-ratios", default="0.6,0.8,1.0")
    parser.add_argument("--accel-limits", default="1.0,1.5,2.0")
    parser.add_argument("--max-time-ratio", type=float, default=1.15)
    parser.add_argument("--ds", type=float, default=0.05)
    parser.add_argument("--dt-max", type=float, default=0.02)
    parser.add_argument("--v-floor", type=float, default=0.15)
    parser.add_argument(
        "--stop-ends",
        action="store_true",
        help="force start/end speed to zero; off by default to match TRACKING+NONE metrics",
    )
    return parser.parse_args()


def base_path_id(name):
    base = os.path.splitext(os.path.basename(name))[0]
    for path_id in KNOWN_PATHS:
        if path_id in base:
            return path_id
    for suffix in ("_smooth", "_radius"):
        if base.endswith(suffix):
            return base[: -len(suffix)]
    return base


def candidate_name(path):
    try:
        data, _ = load_path(path)
        if data.get("candidate"):
            return str(data["candidate"])
    except Exception:
        pass
    base = os.path.splitext(os.path.basename(path))[0]
    for suffix in ("_smooth", "_radius"):
        if base.endswith(suffix):
            return suffix[1:]
    return "original"


def finite_mean(values):
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return float("nan")
    return sum(values) / len(values)


def read_float(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def load_nom_baselines(path):
    groups = {}
    with open(path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("condition") != "NOM":
                continue
            path_id = base_path_id(row.get("bag_name", ""))
            if not path_id:
                continue
            groups.setdefault(path_id, []).append(row)

    baselines = {}
    fields = {
        "tracking_time_s": "tracking_time_s",
        "height_p95_m": "h_p95",
        "height_max_m": "h_max",
        "eta_dot_norm_rms_mps": "eta_dot_rms",
        "odom_ay_abs_p95": "ay_p95",
        "track_dist_p95_m": "track_dist_p95",
        "v_des_eff_mean": "v_des_eff_mean",
    }
    for path_id, rows in groups.items():
        baseline = {"path_id": path_id, "n_nom": len(rows)}
        for csv_key, out_key in fields.items():
            baseline[out_key] = finite_mean([read_float(row, csv_key) for row in rows])
        baselines[path_id] = baseline
    return baselines


def capped_speed_profile(points, cruise_speed, ay_limit, accel_limit, decel_limit, stop_ends):
    kappa_values = curvature(points)
    speeds = []
    for kappa_value in kappa_values:
        v = cruise_speed
        k_abs = abs(kappa_value)
        if ay_limit > 0.0 and k_abs > 1e-6:
            v = min(v, math.sqrt(ay_limit / k_abs))
        speeds.append(max(0.0, v))

    if stop_ends and speeds:
        speeds[0] = 0.0
        speeds[-1] = 0.0

    s_values = cumulative_s(points)
    for i in range(1, len(speeds)):
        ds = max(1e-6, s_values[i] - s_values[i - 1])
        limit = math.sqrt(max(0.0, speeds[i - 1] * speeds[i - 1] + 2.0 * accel_limit * ds))
        speeds[i] = min(speeds[i], limit)
    for i in range(len(speeds) - 2, -1, -1):
        ds = max(1e-6, s_values[i + 1] - s_values[i])
        limit = math.sqrt(max(0.0, speeds[i + 1] * speeds[i + 1] + 2.0 * decel_limit * ds))
        speeds[i] = min(speeds[i], limit)
    return speeds


def rollout_profile(points, speeds, v_floor, omega_n, zeta, height_coeff, dt_max):
    kappa_values = curvature(points)
    dkappa_values = dkappa(points, kappa_values)
    ds_values = [dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    eta_x = eta_x_dot = 0.0
    eta_y = eta_y_dot = 0.0
    h_values = []
    eta_dot_values = []
    energy_values = []
    ay_values = []
    ax_values = []
    jerk_values = []
    alpha_values = []
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
        v0 = speeds[i]
        v1 = speeds[i + 1]
        v_mid = max(v_floor, 0.5 * (v0 + v1))
        dt = ds / max(1e-6, v_mid)
        time_proxy += dt
        ax = (v1 * v1 - v0 * v0) / (2.0 * ds)
        ay = v_mid * v_mid * kappa_values[i]
        alpha = ax * kappa_values[i] + v_mid * v_mid * dkappa_values[i]
        ax_values.append(abs(ax))
        ay_values.append(abs(ay))
        alpha_values.append(abs(alpha))
        if prev_ax is not None:
            jerk_values.append(abs((ax - prev_ax) / dt))
        prev_ax = ax
        steps = max(1, int(math.ceil(dt / dt_max)))
        sub_dt = dt / steps
        for _ in range(steps):
            eta_x, eta_x_dot = modal_step(eta_x, eta_x_dot, ax, sub_dt, omega_n, zeta)
            eta_y, eta_y_dot = modal_step(eta_y, eta_y_dot, ay, sub_dt, omega_n, zeta)

    return {
        "time_proxy_s": time_proxy,
        "h_p95": percentile(h_values, 95.0),
        "h_max": max(h_values) if h_values else float("nan"),
        "eta_dot_rms": rms(eta_dot_values),
        "energy_rms_model": rms(energy_values),
        "ay_p95": percentile(ay_values, 95.0),
        "ax_p95": percentile(ax_values, 95.0),
        "ax_max": max(ax_values) if ax_values else float("nan"),
        "jerk_p95": percentile(jerk_values, 95.0),
        "jerk_max": max(jerk_values) if jerk_values else float("nan"),
        "alpha_p95": percentile(alpha_values, 95.0),
        "v_mean": finite_mean(speeds),
        "v_max": max(speeds) if speeds else float("nan"),
    }


def delta_pct(value, baseline):
    if not math.isfinite(value) or not math.isfinite(baseline) or abs(baseline) <= 1e-12:
        return float("nan")
    return 100.0 * (value - baseline) / baseline


def hard_gate(row, baseline, model_baseline, max_time_ratio):
    slosh_baseline = model_baseline or baseline
    row["time_delta_pct"] = delta_pct(row["time_proxy_s"], baseline["tracking_time_s"])
    row["h_p95_delta_pct"] = delta_pct(row["h_p95"], slosh_baseline["h_p95"])
    row["h_max_delta_pct"] = delta_pct(row["h_max"], slosh_baseline["h_max"])
    row["eta_dot_delta_pct"] = delta_pct(row["eta_dot_rms"], slosh_baseline["eta_dot_rms"])
    row["ay_delta_pct"] = delta_pct(row["ay_p95"], baseline["ay_p95"])
    row["pass_time"] = row["time_proxy_s"] <= baseline["tracking_time_s"] * max_time_ratio
    row["pass_h"] = row["h_p95"] < slosh_baseline["h_p95"] and row["h_max"] <= slosh_baseline["h_max"]
    row["pass_eta_dot"] = row["eta_dot_rms"] <= slosh_baseline["eta_dot_rms"]
    row["pass_ay"] = row["ay_p95"] <= baseline["ay_p95"]
    row["pass_all"] = row["pass_time"] and row["pass_h"] and row["pass_eta_dot"] and row["pass_ay"]
    return row


def original_model_baselines(paths, baselines, args, omega_n, height_coeff, zeta):
    rows = {}
    for path in paths:
        if candidate_name(path) != "original":
            continue
        path_id = base_path_id(path)
        baseline = baselines.get(path_id)
        if not baseline:
            continue
        v_ref = baseline.get("v_des_eff_mean")
        if not math.isfinite(v_ref) or v_ref <= 0.0:
            continue
        _, raw_points = load_path(path)
        points = resample(raw_points, max(0.01, args.ds))
        speeds = [v_ref for _ in points]
        rows[path_id] = rollout_profile(
            points,
            speeds,
            args.v_floor,
            omega_n,
            zeta,
            height_coeff,
            args.dt_max,
        )
    return rows


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
    baselines = load_nom_baselines(args.metrics_csv)
    write_csv(args.baseline_csv, list(baselines.values()))
    omega_n, height_coeff = modal_params(cfg)
    original_models = original_model_baselines(args.path_file, baselines, args, omega_n, height_coeff, cfg["zeta"])

    rows = []
    for path in args.path_file:
        path_id = base_path_id(path)
        baseline = baselines.get(path_id)
        if not baseline:
            print(f"skip {path}: no NOM baseline")
            continue
        _, raw_points = load_path(path)
        points = resample(raw_points, max(0.01, args.ds))
        original_model = original_models.get(path_id, {})
        for cruise_speed in parse_float_list(args.cruise_speeds):
            for ay_ratio in parse_float_list(args.ay_ratios):
                for accel_limit in parse_float_list(args.accel_limits):
                    ay_limit = baseline["ay_p95"] * ay_ratio
                    speeds = capped_speed_profile(
                        points,
                        cruise_speed,
                        ay_limit,
                        accel_limit,
                        accel_limit,
                        args.stop_ends,
                    )
                    row = {
                        "path_id": path_id,
                        "candidate": candidate_name(path),
                        "path": path,
                        "cruise_speed": cruise_speed,
                        "ay_ratio": ay_ratio,
                        "ay_limit": ay_limit,
                        "accel_limit": accel_limit,
                        "baseline_n_nom": baseline["n_nom"],
                        "baseline_time_s": baseline["tracking_time_s"],
                        "baseline_h_p95": baseline["h_p95"],
                        "baseline_h_max": baseline["h_max"],
                        "baseline_eta_dot_rms": baseline["eta_dot_rms"],
                        "baseline_ay_p95": baseline["ay_p95"],
                        "baseline_v_des_eff_mean": baseline["v_des_eff_mean"],
                        "model_baseline_h_p95": original_model.get("h_p95", float("nan")),
                        "model_baseline_h_max": original_model.get("h_max", float("nan")),
                        "model_baseline_eta_dot_rms": original_model.get("eta_dot_rms", float("nan")),
                        "model_baseline_energy_rms": original_model.get("energy_rms_model", float("nan")),
                    }
                    row.update(rollout_profile(
                        points,
                        speeds,
                        args.v_floor,
                        omega_n,
                        cfg["zeta"],
                        height_coeff,
                        args.dt_max,
                    ))
                    row["energy_rms_model_delta_pct"] = delta_pct(
                        row["energy_rms_model"],
                        original_model.get("energy_rms_model", float("nan")),
                    )
                    hard_gate(row, baseline, original_model, args.max_time_ratio)
                    rows.append(row)

    rows.sort(
        key=lambda row: (
            not row["pass_all"],
            row["path_id"],
            row["candidate"],
            row["time_delta_pct"],
            row["h_p95_delta_pct"],
            row["eta_dot_delta_pct"],
        )
    )
    write_csv(args.csv, rows)

    pass_rows = [row for row in rows if row["pass_all"]]
    print(f"pass_all={len(pass_rows)}/{len(rows)}")
    for row in pass_rows[:20]:
        print(
            f"{row['path_id']} {row['candidate']} "
            f"v={row['cruise_speed']:.2f} ay_ratio={row['ay_ratio']:.2f} "
            f"a={row['accel_limit']:.2f} "
            f"time={row['time_delta_pct']:.1f}% "
            f"h={row['h_p95_delta_pct']:.1f}% "
            f"hmax={row['h_max_delta_pct']:.1f}% "
            f"eta_dot={row['eta_dot_delta_pct']:.1f}% "
            f"ay={row['ay_delta_pct']:.1f}%"
        )
    print(f"csv: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
