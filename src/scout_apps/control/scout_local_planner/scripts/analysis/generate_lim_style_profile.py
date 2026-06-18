#!/usr/bin/env python3
"""Generate a Lim-style offline slosh-aware speed profile for a fixed path.

This is a lightweight fixed-path retiming baseline: it predicts slosh with a
linear second-order model using only offline model parameters, then iteratively
reduces local speed ceilings where the predicted open-loop slosh risk is high.
It does not subscribe to ROS topics, does not use live monitor feedback, and does
not adapt the profile during a test case.
"""

import argparse
import csv
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from advanced_profile_common import (  # noqa: E402
    build_s_grid,
    compute_curvature,
    compute_time_accel_jerk,
    retime_accel_limited,
    rows_from_profile,
    simulate_slosh_profile,
    smooth_series,
)
from path_profile_utils import (  # noqa: E402
    cumulative_s,
    interp_points,
    load_path_points,
    maybe_plot,
    write_csv,
)


def write_debug_slosh_csv(out_file, s_grid, t, v, a_tan, a_lat, state, iteration):
    if not out_file:
        return
    path = Path(out_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "iteration",
                "t_s",
                "s_m",
                "v_ref_m_s",
                "a_tan_m_s2",
                "a_lat_m_s2",
                "eta_x_m",
                "eta_y_m",
                "h_model_mm",
            ],
        )
        writer.writeheader()
        for i in range(len(s_grid)):
            writer.writerow(
                {
                    "iteration": iteration,
                    "t_s": t[i],
                    "s_m": s_grid[i],
                    "v_ref_m_s": v[i],
                    "a_tan_m_s2": a_tan[i],
                    "a_lat_m_s2": a_lat[i],
                    "eta_x_m": state["eta_x"][i],
                    "eta_y_m": state["eta_y"][i],
                    "h_model_mm": 1000.0 * state["h_model"][i],
                }
            )


def apply_risk_slowdown(speed_caps, risk_m, limit_m, args):
    if limit_m <= 0.0:
        raise RuntimeError("slosh limit must be positive")
    n = len(speed_caps)
    updated = list(speed_caps)
    changed = False
    radius = max(0, int(args.cap_radius_samples))
    for i, risk in enumerate(risk_m):
        if risk <= limit_m:
            continue
        excess = risk / limit_m - 1.0
        factor = max(args.max_slowdown_factor, 1.0 / (1.0 + args.slowdown_gain * excess))
        for j in range(max(0, i - radius), min(n, i + radius + 1)):
            distance = abs(j - i)
            weight = 1.0 if radius == 0 else 1.0 - distance / float(radius + 1)
            local_factor = 1.0 - (1.0 - factor) * weight
            floor = 0.0 if j == 0 or j == n - 1 else args.min_speed
            new_cap = max(floor, min(updated[j], speed_caps[j] * local_factor))
            if new_cap < updated[j] - 1e-9:
                updated[j] = new_cap
                changed = True
    if not changed:
        return updated, False

    smoothed = smooth_series(updated, passes=args.cap_smooth_passes)
    final = []
    for i, cap in enumerate(updated):
        floor = 0.0 if i == 0 or i == n - 1 else args.min_speed
        final.append(max(floor, min(cap, smoothed[i], args.v_max)))
    reduction = max(abs(speed_caps[i] - final[i]) for i in range(n))
    return final, reduction > 1e-5


def evaluate_profile(s_grid, pose_grid, v, args):
    t, a_tan, j = compute_time_accel_jerk(s_grid, v)
    kappa = compute_curvature(s_grid, pose_grid)
    a_lat = [v[i] * v[i] * kappa[i] for i in range(len(v))]
    state = simulate_slosh_profile(t, a_tan, a_lat, args.omega_n, args.damping_ratio, args.height_coeff)
    peak_mm = max(1000.0 * value for value in state["h_model"])
    return t, a_tan, j, a_lat, state, peak_mm


def optimize_profile(s_grid, pose_grid, args):
    speed_caps = [args.v_max] * len(s_grid)
    best = None
    limit_m = args.slosh_limit_mm / 1000.0
    iterations_done = 0
    stopped_reason = "iter_max"

    for iteration in range(max(1, args.iter_max + 1)):
        v = retime_accel_limited(
            s_grid,
            args.v_max,
            args.a_max,
            args.decel_max,
            args.start_speed,
            args.goal_speed,
            speed_limits=speed_caps,
        )
        t, a_tan, j, a_lat, state, peak_mm = evaluate_profile(s_grid, pose_grid, v, args)
        best = (v, t, a_tan, j, a_lat, state, peak_mm, iteration)
        iterations_done = iteration
        if peak_mm <= args.slosh_limit_mm:
            stopped_reason = "below_limit"
            break
        if iteration >= args.iter_max:
            stopped_reason = "iter_max"
            break
        risk_m = state["h_model"]
        speed_caps, changed = apply_risk_slowdown(speed_caps, risk_m, limit_m, args)
        if not changed:
            stopped_reason = "caps_converged"
            break

    return best, iterations_done, stopped_reason


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a Lim-style offline slosh-aware retimed speed profile CSV for a fixed path."
    )
    parser.add_argument("--path-file", required=True, help="Fixed path JSON from fixed_global_path_runner.py")
    parser.add_argument("--out-csv", required=True, help="Output CSV consumed by path_handler/external_speed_profile_csv")
    parser.add_argument("--plot", default="", help="Optional preview PNG path")
    parser.add_argument("--debug-prefix", default="", help="Optional prefix for offline slosh-risk debug CSVs")
    parser.add_argument("--v-max", type=float, default=0.8, help="Maximum path speed [m/s]")
    parser.add_argument("--a-max", type=float, default=0.6, help="Maximum tangential acceleration [m/s^2]")
    parser.add_argument("--decel-max", type=float, default=0.6, help="Maximum tangential deceleration [m/s^2]")
    parser.add_argument("--start-speed", type=float, default=0.0, help="Start speed [m/s]")
    parser.add_argument("--goal-speed", type=float, default=0.0, help="Goal speed [m/s]")
    parser.add_argument("--num-samples", type=int, default=201, help="Number of output samples if --ds is not set")
    parser.add_argument("--ds", type=float, default=0.0, help="Output spacing in meters; overrides --num-samples when >0")
    parser.add_argument("--omega-n", type=float, required=True, help="Liquid natural frequency [rad/s]")
    parser.add_argument("--damping-ratio", type=float, default=0.05, help="Liquid modal damping ratio [-]")
    parser.add_argument("--height-coeff", type=float, default=1.0, help="Height proxy coefficient applied to eta norm")
    parser.add_argument("--slosh-limit-mm", type=float, default=5.0, help="Offline predicted slosh-risk target [mm]")
    parser.add_argument("--iter-max", type=int, default=6, help="Maximum risk-retiming iterations")
    parser.add_argument("--slowdown-gain", type=float, default=0.6, help="Speed-cap reduction gain for risk excess")
    parser.add_argument("--max-slowdown-factor", type=float, default=0.45,
                        help="Lower bound for one-step cap multiplier in risky regions")
    parser.add_argument("--cap-radius-samples", type=int, default=3, help="Neighbor samples slowed around a risk peak")
    parser.add_argument("--cap-smooth-passes", type=int, default=3, help="Smoothing passes for local speed caps")
    parser.add_argument("--min-speed", type=float, default=0.05, help="Minimum interior speed cap [m/s]")
    parser.add_argument("--method-name", default="LIM_STYLE", help="Method label stored in the CSV")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.slosh_limit_mm <= 0.0:
        raise RuntimeError("--slosh-limit-mm must be positive")
    if args.max_slowdown_factor <= 0.0 or args.max_slowdown_factor > 1.0:
        raise RuntimeError("--max-slowdown-factor must be in (0, 1]")

    points = load_path_points(args.path_file)
    path_s = cumulative_s(points)
    total_len = path_s[-1]
    s_grid = build_s_grid(total_len, args.num_samples, args.ds)
    pose_grid = interp_points(points, path_s, s_grid)

    best, iterations_done, stopped_reason = optimize_profile(s_grid, pose_grid, args)
    v, t, a_tan, j, a_lat, state, peak_mm, final_iteration = best
    rows = rows_from_profile(s_grid, pose_grid, t, v, a_tan, j, args.method_name)
    write_csv(args.out_csv, rows)
    maybe_plot(args.plot, rows)
    if args.debug_prefix:
        write_debug_slosh_csv(
            f"{args.debug_prefix}_slosh_prediction.csv",
            s_grid,
            t,
            v,
            a_tan,
            a_lat,
            state,
            final_iteration,
        )
    print(
        f"[OK] wrote {args.out_csv}: samples={len(rows)} length={total_len:.3f}m "
        f"duration={t[-1]:.3f}s vmax={max(v):.3f}m/s peak_h={peak_mm:.3f}mm "
        f"iterations={iterations_done} stop={stopped_reason}"
    )


if __name__ == "__main__":
    main()
