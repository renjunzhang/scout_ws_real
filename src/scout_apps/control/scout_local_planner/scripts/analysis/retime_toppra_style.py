#!/usr/bin/env python3
"""Generate a TOPPRA-style acceleration-limited v_ref(s) CSV for a fixed path.

This is a dependency-free retiming baseline helper. It keeps the fixed path
geometry unchanged and only computes a scalar path-speed profile.
"""

import argparse
import math
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from path_profile_utils import (  # noqa: E402
    cumulative_s,
    interp_points,
    load_path_points,
    maybe_plot,
    write_csv,
)


def build_s_grid(total_len, num_samples, ds):
    if ds and ds > 0.0:
        n = max(2, int(math.ceil(total_len / ds)) + 1)
    else:
        n = max(2, int(num_samples))
    return [total_len * i / (n - 1) for i in range(n)]


def retime_accel_limited(s_grid, v_max, a_max, decel_max, start_speed, goal_speed):
    if v_max < 0.0 or a_max < 0.0 or decel_max < 0.0:
        raise RuntimeError("v_max/a_max/decel_max must be non-negative")
    n = len(s_grid)
    v = [float(v_max)] * n
    v[0] = min(v[0], max(0.0, start_speed))
    v[-1] = min(v[-1], max(0.0, goal_speed))

    if a_max > 0.0:
        for i in range(1, n):
            ds = max(0.0, s_grid[i] - s_grid[i - 1])
            v_lim = math.sqrt(max(0.0, v[i - 1] * v[i - 1] + 2.0 * a_max * ds))
            v[i] = min(v[i], v_lim)

    if decel_max > 0.0:
        for i in range(n - 2, -1, -1):
            ds = max(0.0, s_grid[i + 1] - s_grid[i])
            v_lim = math.sqrt(max(0.0, v[i + 1] * v[i + 1] + 2.0 * decel_max * ds))
            v[i] = min(v[i], v_lim)
    return v


def compute_time_accel_jerk(s_grid, v):
    n = len(s_grid)
    t = [0.0] * n
    a = [0.0] * n
    j = [0.0] * n
    dt_seg = [0.0] * n
    for i in range(1, n):
        ds = max(0.0, s_grid[i] - s_grid[i - 1])
        v_sum = v[i] + v[i - 1]
        dt = 2.0 * ds / v_sum if v_sum > 1e-9 else 0.0
        dt_seg[i] = dt
        t[i] = t[i - 1] + dt
        a[i] = (v[i] - v[i - 1]) / dt if dt > 1e-9 else 0.0
    for i in range(1, n):
        j[i] = (a[i] - a[i - 1]) / dt_seg[i] if dt_seg[i] > 1e-9 else 0.0
    return t, a, j


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a TOPPRA-style acceleration-limited speed profile CSV for fixed-path MPC."
    )
    parser.add_argument("--path-file", required=True, help="Fixed path JSON from fixed_global_path_runner.py")
    parser.add_argument("--out-csv", required=True, help="Output CSV consumed by path_handler/external_speed_profile_csv")
    parser.add_argument("--plot", default="", help="Optional preview PNG path")
    parser.add_argument("--v-max", type=float, default=0.8, help="Maximum path speed [m/s]")
    parser.add_argument("--a-max", type=float, default=0.6, help="Maximum tangential acceleration [m/s^2]")
    parser.add_argument("--decel-max", type=float, default=0.8, help="Maximum tangential deceleration [m/s^2]")
    parser.add_argument("--start-speed", type=float, default=0.0, help="Start speed [m/s]")
    parser.add_argument("--goal-speed", type=float, default=0.0, help="Goal speed [m/s]")
    parser.add_argument("--num-samples", type=int, default=201, help="Number of output samples if --ds is not set")
    parser.add_argument("--ds", type=float, default=0.0, help="Output spacing in meters; overrides --num-samples when >0")
    parser.add_argument("--method-name", default="TOPPRA_STYLE", help="Method label stored in the CSV")
    return parser.parse_args()


def main():
    args = parse_args()
    points = load_path_points(args.path_file)
    path_s = cumulative_s(points)
    total_len = path_s[-1]
    s_grid = build_s_grid(total_len, args.num_samples, args.ds)
    pose_grid = interp_points(points, path_s, s_grid)
    v = retime_accel_limited(
        s_grid,
        args.v_max,
        args.a_max,
        args.decel_max,
        args.start_speed,
        args.goal_speed,
    )
    t, a, j = compute_time_accel_jerk(s_grid, v)
    rows = []
    for i, s in enumerate(s_grid):
        x, y, yaw = pose_grid[i]
        rows.append(
            {
                "s_normalized": s / total_len,
                "s_m": s,
                "t_s": t[i],
                "x": x,
                "y": y,
                "yaw": yaw,
                "v_ref_m_s": v[i],
                "a_ref_m_s2": a[i],
                "jerk_ref_m_s3": j[i],
                "method": args.method_name,
            }
        )
    write_csv(args.out_csv, rows)
    maybe_plot(args.plot, rows)
    print(
        f"[OK] wrote {args.out_csv}: samples={len(rows)} length={total_len:.3f}m "
        f"duration={t[-1]:.3f}s vmax={max(v):.3f}m/s"
    )


if __name__ == "__main__":
    main()
