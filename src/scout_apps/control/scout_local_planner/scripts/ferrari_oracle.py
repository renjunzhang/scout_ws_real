#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ferrari 2026 RA-L offline NLP oracle for Scout (differential drive).

Solves the assigned-path time-optimal problem (§III-A of Ferrari et al.)
adapted to Scout: heading is tied to the path tangent, so the only decision
variable is the motion law s(t). The slosh ODE is identical to the online
OSCRS rollout: linear MSD with mode 1 frequency omega_n and damping zeta,
plus a centripetal parabola term in container-frame ax/ay.

Status: SKELETON. The CasADi/IPOPT formulation builds and runs on simple
toy paths but has not been validated against sim/real bags. Convergence on
long curves and collocation node tuning are W3 work per the RA-L plan §6.1.
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path

try:
    import casadi as ca
except ImportError:
    sys.stderr.write("error: casadi not installed; run `pip install casadi`\n")
    sys.exit(2)

import yaml

G = 9.81
XI_11 = 1.841


def parse_args():
    parser = argparse.ArgumentParser(description="Ferrari NLP oracle for Scout assigned-path.")
    parser.add_argument("path_input", help="CSV with columns x,y or a rosbag containing a nav_msgs/Path")
    parser.add_argument("--config",
                        default="src/scout_apps/control/scout_local_planner/config/ferrari_oracle.yaml")
    parser.add_argument("--container",
                        default="src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml")
    parser.add_argument("--scenario", default="open_user_goal")
    parser.add_argument("--path-topic", default="/scout/global_path",
                        help="Path topic to extract when path_input is a rosbag")
    parser.add_argument("--n-nodes", type=int, default=0,
                        help="Override oracle.collocation.n_nodes for smoke/debug runs")
    parser.add_argument("--max-iter", type=int, default=0,
                        help="Override IPOPT max_iter")
    parser.add_argument("--print-level", type=int, default=-1,
                        help="Override IPOPT print_level")
    parser.add_argument("--out-csv", default="")
    parser.add_argument("--out-summary", default="")
    return parser.parse_args()


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def load_path_csv(path):
    pts = []
    with open(path, "r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, ValueError):
                continue
            pts.append((x, y))
    if len(pts) < 3:
        raise SystemExit(f"path csv {path} has fewer than 3 points")
    return pts


def load_path_bag(path, topic):
    try:
        import rosbag
    except ImportError as exc:
        raise SystemExit("rosbag is required when path_input is a .bag file") from exc

    last_msg = None
    with rosbag.Bag(path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            last_msg = msg
    if last_msg is None:
        raise SystemExit(f"no nav_msgs/Path messages found on {topic} in {path}")
    pts = [(float(p.pose.position.x), float(p.pose.position.y)) for p in last_msg.poses]
    if len(pts) < 3:
        raise SystemExit(f"path topic {topic} in {path} has fewer than 3 points")
    return pts


def load_path_input(path, topic):
    if path.endswith(".bag"):
        return load_path_bag(path, topic)
    return load_path_csv(path)


def cumulative_arclength(points):
    s = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        s.append(s[-1] + math.hypot(dx, dy))
    return s


def resample_uniform_arclength(points, n_target):
    """Linearly interpolate input path to exactly n_target nodes spaced at
    uniform arclength steps. Used to honour collocation.n_nodes from the
    oracle config — without this the NLP grid was tied to the input csv length."""
    if n_target <= 0 or len(points) <= 2:
        return list(points)
    s = cumulative_arclength(points)
    total = s[-1]
    if total <= 1e-9:
        return list(points)
    out = []
    for k in range(n_target):
        target_s = total * k / (n_target - 1)
        idx = 0
        while idx + 1 < len(s) and s[idx + 1] < target_s:
            idx += 1
        if idx + 1 >= len(s):
            out.append(points[-1])
            continue
        s0 = s[idx]
        s1 = s[idx + 1]
        denom = max(1e-9, s1 - s0)
        r = (target_s - s0) / denom
        x = points[idx][0] * (1.0 - r) + points[idx + 1][0] * r
        y = points[idx][1] * (1.0 - r) + points[idx + 1][1] * r
        out.append((x, y))
    return out


def curvature_from_path(points):
    """Signed curvature from three consecutive points using the Menger formula.
    Positive kappa => left turn (CCW); negative => right turn (CW). Sign is
    required so the slosh ODE's lateral excitation ay = v^2 * kappa flips
    correctly across S-curve inflection points."""
    n = len(points)
    kappa = [0.0] * n
    for i in range(1, n - 1):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        a = math.hypot(x1 - x0, y1 - y0)
        b = math.hypot(x2 - x1, y2 - y1)
        c = math.hypot(x2 - x0, y2 - y0)
        denom = max(1e-9, a * b * c)
        cross = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        kappa[i] = 2.0 * cross / denom
    if n >= 2:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    return kappa


def dkappa_from_path(points, kappa):
    s = cumulative_arclength(points)
    dkappa = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        ds = max(1e-9, s[i + 1] - s[i - 1])
        dkappa[i] = (kappa[i + 1] - kappa[i - 1]) / ds
    if len(points) >= 2:
        dkappa[0] = dkappa[1]
        dkappa[-1] = dkappa[-2]
    return dkappa


def slosh_modal_params(container):
    slosh = container.get("slosh", {}) or {}
    R = float(slosh.get("container_radius", 0.0185))
    h = float(slosh.get("liquid_height", 0.058))
    rho = float(slosh.get("liquid_density", 1000.0))
    zeta = float(slosh.get("damping_ratio", 0.05))
    offset_x = float(slosh.get("offset_x", 0.0))
    offset_y = float(slosh.get("offset_y", 0.0))
    xi = XI_11
    m_f = rho * math.pi * R * R * h
    m_n = m_f * (2.0 * R * math.tanh(xi * h / R)) / (xi * h * (xi * xi - 1.0))
    omega_n = math.sqrt(max(G * (xi / R) * math.tanh(xi * h / R), 0.0))
    mode = (container.get("oscrs", {}) or {}).get("height_coeff_mode", "observer_linear")
    if mode == "ferrari_closed_form":
        height_coeff = (xi * xi * h * m_n) / (m_f * R)
    else:
        height_coeff = (4.0 * h * m_n) / (m_f * R)
    return {
        "R": R,
        "h": h,
        "zeta": zeta,
        "omega_n": omega_n,
        "height_coeff": height_coeff,
        "offset_x": offset_x,
        "offset_y": offset_y,
    }


def build_nlp(path_pts, oracle_cfg, modal, args):
    # NOTE: collocation.poly_degree is currently inert. The NLP uses direct
    # multiple-shooting with semi-implicit Euler integration; honouring
    # poly_degree would require switching to Lagrange polynomial collocation,
    # tracked as W3+ work. n_nodes is honoured via uniform-arclength resample.
    bounds = oracle_cfg["oracle"]["bounds"]
    cons = oracle_cfg["oracle"]["constraints"]
    coll = oracle_cfg["oracle"].get("collocation", {}) or {}
    n_nodes_cfg = int(coll.get("n_nodes", 0))
    if args.n_nodes >= 4:
        n_nodes_cfg = args.n_nodes
    if n_nodes_cfg >= 4 and n_nodes_cfg != len(path_pts):
        path_pts = resample_uniform_arclength(path_pts, n_nodes_cfg)
    s_grid = cumulative_arclength(path_pts)
    kappa_arr = curvature_from_path(path_pts)
    dkappa_arr = dkappa_from_path(path_pts, kappa_arr)
    n = len(path_pts)
    v_max = float(bounds["v_max"])
    v_min = float(bounds.get("v_min", 0.0))
    dt_max = float(bounds.get("dt_max", 0.0))
    a_max = float(bounds["a_max"])
    jerk_max = float(bounds.get("jerk_max", 0.0))
    omega_max = float(bounds["omega_max"])
    eta_lim = cons["eta_lim_mm"] / 1000.0
    residual_lim = cons["residual_ratio"] * eta_lim

    opti = ca.Opti()
    v = opti.variable(n)              # speed at each path node (m/s)
    a = opti.variable(n - 1)          # tangential accel between nodes (m/s^2)
    eta_x = opti.variable(n)
    eta_y = opti.variable(n)
    eta_x_dot = opti.variable(n)
    eta_y_dot = opti.variable(n)
    dt = opti.variable(n - 1)         # per-segment travel time

    omega_n = modal["omega_n"]
    zeta = modal["zeta"]
    height_coeff = modal["height_coeff"]
    R = modal["R"]
    offset_x = modal["offset_x"]
    offset_y = modal["offset_y"]
    parabola_coeff = (R * R) / (4.0 * G)

    # Boundary: start and end at rest
    opti.subject_to(v[0] == 0.0)
    opti.subject_to(v[n - 1] == 0.0)
    opti.subject_to(eta_x[0] == 0.0)
    opti.subject_to(eta_y[0] == 0.0)
    opti.subject_to(eta_x_dot[0] == 0.0)
    opti.subject_to(eta_y_dot[0] == 0.0)

    # Path-parameterised dynamics: ds = (v_i + v_{i+1})/2 * dt
    for i in range(n - 1):
        ds = s_grid[i + 1] - s_grid[i]
        opti.subject_to(ds == 0.5 * (v[i] + v[i + 1]) * dt[i])
        opti.subject_to(dt[i] >= 1e-3)
        if dt_max > 0.0:
            opti.subject_to(dt[i] <= dt_max)
        # Tangential accel link: v_{i+1}^2 = v_i^2 + 2 a ds (Ferrari §III-A eq for s(t))
        opti.subject_to(v[i + 1] * v[i + 1] == v[i] * v[i] + 2.0 * a[i] * ds)
        opti.subject_to(a[i] <= a_max)
        opti.subject_to(a[i] >= -a_max)

    # Per-node bounds. v_min applies only to interior nodes; boundaries stay
    # pinned at zero for start/stop. omega is bounded by magnitude.
    for i in range(n):
        if 0 < i < n - 1 and v_min > 0.0:
            opti.subject_to(v[i] >= v_min)
        else:
            opti.subject_to(v[i] >= 0.0)
        opti.subject_to(v[i] <= v_max)
        if abs(kappa_arr[i]) > 1e-9:
            opti.subject_to(v[i] * abs(kappa_arr[i]) <= omega_max)

    # Jerk bound between consecutive tangential-accel segments.
    # j ≈ (a[i+1] - a[i]) / dt_avg, with dt_avg = (dt[i] + dt[i+1]) / 2.
    if jerk_max > 0.0:
        for i in range(n - 2):
            dt_avg = 0.5 * (dt[i] + dt[i + 1])
            opti.subject_to(a[i + 1] - a[i] <= jerk_max * dt_avg)
            opti.subject_to(a[i + 1] - a[i] >= -jerk_max * dt_avg)

    # Slosh ODE in container frame: ax = a_tangential, ay = v^2 * kappa (centripetal)
    for i in range(n - 1):
        omega_i = v[i] * kappa_arr[i]
        omega_ip1 = v[i + 1] * kappa_arr[i + 1]
        omega_sq = 0.5 * (omega_i * omega_i + omega_ip1 * omega_ip1)
        alpha_i = a[i] * kappa_arr[i] + v[i] * v[i] * dkappa_arr[i]
        alpha_ip1 = a[i] * kappa_arr[i + 1] + v[i + 1] * v[i + 1] * dkappa_arr[i + 1]
        alpha = 0.5 * (alpha_i + alpha_ip1)
        ax = a[i] - alpha * offset_y - omega_sq * offset_x
        ay_i = v[i] * v[i] * kappa_arr[i]
        ay_ip1 = v[i + 1] * v[i + 1] * kappa_arr[i + 1]
        ay = 0.5 * (ay_i + ay_ip1) + alpha * offset_x - omega_sq * offset_y
        # Semi-implicit Euler over dt[i] for modal coords
        ddx = -2.0 * zeta * omega_n * eta_x_dot[i] - omega_n * omega_n * eta_x[i] - ax
        ddy = -2.0 * zeta * omega_n * eta_y_dot[i] - omega_n * omega_n * eta_y[i] - ay
        opti.subject_to(eta_x_dot[i + 1] == eta_x_dot[i] + ddx * dt[i])
        opti.subject_to(eta_y_dot[i + 1] == eta_y_dot[i] + ddy * dt[i])
        opti.subject_to(eta_x[i + 1] == eta_x[i] + eta_x_dot[i + 1] * dt[i])
        opti.subject_to(eta_y[i + 1] == eta_y[i] + eta_y_dot[i + 1] * dt[i])

    # Hard gate: eta_pred(t) <= eta_lim throughout the path
    for i in range(n):
        modal_norm_sq = eta_x[i] * eta_x[i] + eta_y[i] * eta_y[i]
        omega_path = v[i] * kappa_arr[i]
        h_total = height_coeff * ca.sqrt(modal_norm_sq + 1e-18) + parabola_coeff * omega_path * omega_path
        opti.subject_to(h_total <= eta_lim)

    # Residual segment: zero-input free response across n_settle steps
    n_settle = max(1, int(cons["settle_duration"] / 0.05))
    eta_xr = opti.variable(n_settle + 1)
    eta_yr = opti.variable(n_settle + 1)
    eta_xr_dot = opti.variable(n_settle + 1)
    eta_yr_dot = opti.variable(n_settle + 1)
    opti.subject_to(eta_xr[0] == eta_x[n - 1])
    opti.subject_to(eta_yr[0] == eta_y[n - 1])
    opti.subject_to(eta_xr_dot[0] == eta_x_dot[n - 1])
    opti.subject_to(eta_yr_dot[0] == eta_y_dot[n - 1])
    dt_settle = 0.05
    for j in range(n_settle):
        ddxr = -2.0 * zeta * omega_n * eta_xr_dot[j] - omega_n * omega_n * eta_xr[j]
        ddyr = -2.0 * zeta * omega_n * eta_yr_dot[j] - omega_n * omega_n * eta_yr[j]
        opti.subject_to(eta_xr_dot[j + 1] == eta_xr_dot[j] + ddxr * dt_settle)
        opti.subject_to(eta_yr_dot[j + 1] == eta_yr_dot[j] + ddyr * dt_settle)
        opti.subject_to(eta_xr[j + 1] == eta_xr[j] + eta_xr_dot[j + 1] * dt_settle)
        opti.subject_to(eta_yr[j + 1] == eta_yr[j] + eta_yr_dot[j + 1] * dt_settle)
        h_resid = height_coeff * ca.sqrt(eta_xr[j + 1] ** 2 + eta_yr[j + 1] ** 2 + 1e-18)
        opti.subject_to(h_resid <= residual_lim)

    # Objective: minimise total travel time
    opti.minimize(ca.sum1(dt))

    # Initial guesses consistent with path-parameterised kinematics.
    v_guess = []
    for i in range(n):
        # Rest-to-rest profile with nonzero interior speed. A constant interior
        # guess avoids huge endpoint dt values that make the slosh constraints
        # numerically ill-conditioned.
        v_guess.append(0.5 * v_max)
    v_guess[0] = 0.0
    v_guess[-1] = 0.0
    a_guess = []
    dt_guess = []
    for i in range(n - 1):
        ds = max(1e-6, s_grid[i + 1] - s_grid[i])
        avg_v = max(0.05, 0.5 * (v_guess[i] + v_guess[i + 1]))
        dt_guess.append(ds / avg_v)
        a_guess.append((v_guess[i + 1] * v_guess[i + 1] - v_guess[i] * v_guess[i]) / (2.0 * ds))
    opti.set_initial(v, ca.DM(v_guess))
    opti.set_initial(a, ca.DM(a_guess))
    opti.set_initial(dt, ca.DM(dt_guess))
    opti.set_initial(eta_x, ca.DM.zeros(n))
    opti.set_initial(eta_y, ca.DM.zeros(n))
    opti.set_initial(eta_x_dot, ca.DM.zeros(n))
    opti.set_initial(eta_y_dot, ca.DM.zeros(n))

    ipopt_cfg = oracle_cfg["oracle"].get("ipopt", {})
    max_iter = int(args.max_iter) if args.max_iter > 0 else int(ipopt_cfg.get("max_iter", 500))
    print_level = int(args.print_level) if args.print_level >= 0 else int(ipopt_cfg.get("print_level", 3))
    opti.solver(
        "ipopt",
        {"print_time": True},
        {
            "max_iter": max_iter,
            "tol": float(ipopt_cfg.get("tol", 1e-6)),
            "acceptable_tol": float(ipopt_cfg.get("acceptable_tol", 1e-4)),
            "print_level": print_level,
            "linear_solver": str(ipopt_cfg.get("linear_solver", "mumps")),
        },
    )
    return opti, {
        "v": v, "a": a, "dt": dt,
        "eta_x": eta_x, "eta_y": eta_y,
        "eta_x_dot": eta_x_dot, "eta_y_dot": eta_y_dot,
        "s_grid": s_grid, "kappa": kappa_arr,
        "modal": modal,
    }


def solve_straight_analytic(path_pts, oracle_cfg, modal, args):
    """Minimal oracle fallback for straight assigned paths.

    It gives a deterministic rest-to-rest trapezoidal speed law under the same
    v/a bounds, then rolls out the same linear slosh model. This keeps the
    oracle output/metric chain testable before the full nonlinear IPOPT oracle
    is tuned for curved paths.
    """
    bounds = oracle_cfg["oracle"]["bounds"]
    cons = oracle_cfg["oracle"]["constraints"]
    coll = oracle_cfg["oracle"].get("collocation", {}) or {}
    n_nodes = args.n_nodes if args.n_nodes >= 4 else int(coll.get("n_nodes", 80))
    points = resample_uniform_arclength(path_pts, n_nodes)
    s_grid = cumulative_arclength(points)
    total_s = s_grid[-1]
    v_max = float(bounds["v_max"])
    a_max = float(bounds["a_max"])
    eta_lim = cons["eta_lim_mm"] / 1000.0
    residual_lim = float(cons["residual_ratio"]) * eta_lim
    v_peak = min(v_max, math.sqrt(max(0.0, a_max * total_s)))
    s_acc = v_peak * v_peak / (2.0 * a_max) if a_max > 1e-9 else 0.0
    s_dec_start = total_s - s_acc

    v_values = []
    for s in s_grid:
        if s <= s_acc:
            v_values.append(math.sqrt(max(0.0, 2.0 * a_max * s)))
        elif s >= s_dec_start:
            v_values.append(math.sqrt(max(0.0, 2.0 * a_max * (total_s - s))))
        else:
            v_values.append(v_peak)
    v_values[0] = 0.0
    v_values[-1] = 0.0

    omega_n = modal["omega_n"]
    zeta = modal["zeta"]
    damping = 2.0 * zeta * omega_n
    wn2 = omega_n * omega_n
    h_coeff = modal["height_coeff"]
    eta_x = eta_y = eta_x_dot = eta_y_dot = 0.0

    def advance(dt_total, ux, uy):
        nonlocal eta_x, eta_y, eta_x_dot, eta_y_dot
        n_sub = max(1, int(math.ceil(dt_total / 0.005)))
        dt_sub = dt_total / n_sub
        for _ in range(n_sub):
            ddx = -damping * eta_x_dot - wn2 * eta_x - ux
            ddy = -damping * eta_y_dot - wn2 * eta_y - uy
            eta_x_dot += ddx * dt_sub
            eta_y_dot += ddy * dt_sub
            eta_x += eta_x_dot * dt_sub
            eta_y += eta_y_dot * dt_sub

    rows = []
    t = 0.0
    h_max = 0.0
    for i, s in enumerate(s_grid):
        h_total = h_coeff * math.hypot(eta_x, eta_y)
        h_max = max(h_max, h_total)
        rows.append({
            "s": s,
            "t": t,
            "v": v_values[i],
            "a": 0.0,
            "omega": 0.0,
            "kappa": 0.0,
            "eta_x": eta_x,
            "eta_y": eta_y,
            "h_total": h_total,
        })
        if i >= len(s_grid) - 1:
            break
        ds = max(1e-9, s_grid[i + 1] - s_grid[i])
        a_seg = (v_values[i + 1] ** 2 - v_values[i] ** 2) / (2.0 * ds)
        avg_v = 0.5 * (v_values[i] + v_values[i + 1])
        dt = ds / max(1e-6, avg_v)
        rows[-1]["a"] = a_seg
        advance(dt, a_seg, 0.0)
        t += dt

    residual_max = 0.0
    for _ in range(max(1, int(float(cons["settle_duration"]) / 0.05))):
        advance(0.05, 0.0, 0.0)
        residual_max = max(residual_max, h_coeff * math.hypot(eta_x, eta_y))

    out_csv = args.out_csv or f"docs/Claude/分析数据/ferrari_oracle_{args.scenario}.csv"
    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
    with open(out_csv, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "scenario": args.scenario,
        "status": "analytic_straight",
        "n_nodes": len(rows),
        "tracking_time_s": rows[-1]["t"],
        "h_total_max_mm": h_max * 1000.0,
        "h_residual_max_mm": residual_max * 1000.0,
        "eta_lim_mm": eta_lim * 1000.0,
        "residual_lim_mm": residual_lim * 1000.0,
        "constraint_compliant": bool(h_max <= eta_lim and residual_max <= residual_lim),
        "v_max_m_s": max(r["v"] for r in rows),
        "a_abs_max_m_s2": max(abs(r["a"]) for r in rows),
        "omega_n_used": modal["omega_n"],
        "zeta_used": modal["zeta"],
        "height_coeff_used": modal["height_coeff"],
    }
    if args.out_summary:
        os.makedirs(os.path.dirname(args.out_summary) or ".", exist_ok=True)
        with open(args.out_summary, "w", encoding="utf-8") as handle:
            yaml.safe_dump(summary, handle, sort_keys=False)
    print("oracle summary:", summary)
    print(f"oracle csv: {out_csv}")
    return summary


def write_outputs(opti, vars_, sol, args):
    v_val = sol.value(vars_["v"])
    a_val = sol.value(vars_["a"])
    dt_val = sol.value(vars_["dt"])
    eta_x_val = sol.value(vars_["eta_x"])
    eta_y_val = sol.value(vars_["eta_y"])
    s_grid = vars_["s_grid"]
    kappa_arr = vars_["kappa"]
    modal = vars_["modal"]
    n = len(s_grid)
    R = modal["R"]
    parabola_coeff = (R * R) / (4.0 * G)
    h_coeff = modal["height_coeff"]

    rows = []
    t = 0.0
    for i in range(n):
        omega_path = v_val[i] * kappa_arr[i]
        h_total = (
            h_coeff * math.sqrt(eta_x_val[i] ** 2 + eta_y_val[i] ** 2)
            + parabola_coeff * omega_path * omega_path
        )
        rows.append({
            "s": s_grid[i],
            "t": t,
            "v": float(v_val[i]),
            "a": float(a_val[i - 1]) if i > 0 else 0.0,
            "omega": omega_path,
            "kappa": kappa_arr[i],
            "eta_x": float(eta_x_val[i]),
            "eta_y": float(eta_y_val[i]),
            "h_total": h_total,
        })
        if i < n - 1:
            t += float(dt_val[i])
    out_csv = args.out_csv or f"docs/Claude/分析数据/ferrari_oracle_{args.scenario}.csv"
    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
    with open(out_csv, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "scenario": args.scenario,
        "n_nodes": n,
        "tracking_time_s": float(sum(dt_val)),
        "h_total_max_mm": max(r["h_total"] for r in rows) * 1000.0,
        "v_max_m_s": max(r["v"] for r in rows),
        "a_abs_max_m_s2": max(abs(r["a"]) for r in rows),
        "omega_n_used": modal["omega_n"],
        "zeta_used": modal["zeta"],
        "height_coeff_used": modal["height_coeff"],
    }
    if args.out_summary:
        os.makedirs(os.path.dirname(args.out_summary) or ".", exist_ok=True)
        with open(args.out_summary, "w", encoding="utf-8") as handle:
            yaml.safe_dump(summary, handle, sort_keys=False)
    print("oracle summary:", summary)
    print(f"oracle csv: {out_csv}")


def main():
    args = parse_args()
    container = load_yaml(args.container)
    oracle_cfg = load_yaml(args.config)
    modal = slosh_modal_params(container)
    path_pts = load_path_input(args.path_input, args.path_topic)
    if max(abs(k) for k in curvature_from_path(path_pts)) < 1e-6:
        solve_straight_analytic(path_pts, oracle_cfg, modal, args)
        return
    opti, vars_ = build_nlp(path_pts, oracle_cfg, modal, args)
    try:
        sol = opti.solve()
    except RuntimeError as exc:
        if args.out_summary:
            os.makedirs(os.path.dirname(args.out_summary) or ".", exist_ok=True)
            with open(args.out_summary, "w", encoding="utf-8") as handle:
                yaml.safe_dump({
                    "scenario": args.scenario,
                    "status": "failed",
                    "reason": "ipopt_not_converged",
                    "path_input": args.path_input,
                    "path_topic": args.path_topic,
                    "n_nodes_override": args.n_nodes,
                    "max_iter_override": args.max_iter,
                    "error": str(exc).splitlines()[0],
                }, handle, sort_keys=False)
        sys.stderr.write(f"oracle failed to converge: {exc}\n")
        sys.exit(1)
    write_outputs(opti, vars_, sol, args)


if __name__ == "__main__":
    main()
