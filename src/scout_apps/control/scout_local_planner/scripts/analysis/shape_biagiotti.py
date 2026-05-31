#!/usr/bin/env python3
"""Generate a Biagiotti-style slosh-aware v_ref(s) CSV for a fixed path.

This is an open-loop reference-shaping baseline. It keeps the fixed path
geometry unchanged, builds an acceleration-limited scalar path-speed profile,
then applies a slosh-frequency-aware ZVD input shaper to the tangential
acceleration profile. It is a practical baseline inspired by sloshing-free
feed-forward shaping work, not a full reproduction of vessel-tilting methods.
"""

import argparse
import csv
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


def interp_series(t_src, y_src, t):
    if not t_src:
        return 0.0
    if t <= t_src[0]:
        return y_src[0]
    if t >= t_src[-1]:
        return y_src[-1]
    lo = 0
    hi = len(t_src) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if t_src[mid] <= t:
            lo = mid
        else:
            hi = mid
    dt = t_src[hi] - t_src[lo]
    ratio = 0.0 if dt <= 1e-12 else (t - t_src[lo]) / dt
    return y_src[lo] + ratio * (y_src[hi] - y_src[lo])


def build_zvd_shaper(omega_n, damping_ratio):
    if omega_n <= 0.0:
        raise RuntimeError("omega_n must be positive")
    if damping_ratio < 0.0 or damping_ratio >= 1.0:
        raise RuntimeError("damping_ratio must be in [0, 1)")

    omega_d = omega_n * math.sqrt(max(1e-12, 1.0 - damping_ratio * damping_ratio))
    td = math.pi / omega_d
    k = math.exp(-damping_ratio * math.pi / math.sqrt(max(1e-12, 1.0 - damping_ratio * damping_ratio)))
    denom = 1.0 + 2.0 * k + k * k
    return [
        (0.0, 1.0 / denom),
        (td, 2.0 * k / denom),
        (2.0 * td, k * k / denom),
    ]


def shape_acceleration_time_law(t_base, a_base, total_len, args):
    shaper = build_zvd_shaper(args.omega_n, args.damping_ratio)
    t_end = t_base[-1] + shaper[-1][0]
    n = max(2, int(math.ceil(t_end / args.dt)) + 1)
    t = [min(t_end, i * args.dt) for i in range(n)]
    t[-1] = t_end

    a_shaped = []
    for ti in t:
        acc = 0.0
        for delay, amp in shaper:
            src_t = ti - delay
            if 0.0 <= src_t <= t_base[-1]:
                acc += amp * interp_series(t_base, a_base, src_t)
        a_shaped.append(acc)

    v = [max(0.0, args.start_speed)]
    s = [0.0]
    jerk = [0.0]
    for i in range(1, len(t)):
        dt = max(0.0, t[i] - t[i - 1])
        vi = max(0.0, v[-1] + 0.5 * (a_shaped[i - 1] + a_shaped[i]) * dt)
        si = s[-1] + 0.5 * (v[-1] + vi) * dt
        ji = (a_shaped[i] - a_shaped[i - 1]) / dt if dt > 1e-9 else 0.0
        v.append(vi)
        s.append(si)
        jerk.append(ji)

    if s[-1] <= 1e-9:
        raise RuntimeError("shaped profile produced zero path progress")
    scale = total_len / s[-1]
    s_scaled = [max(0.0, min(total_len, si * scale)) for si in s]
    return t, s_scaled, v, a_shaped, jerk, shaper, scale


def write_debug_time_csv(out_file, rows):
    if not out_file:
        return
    out_path = Path(out_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "t_s",
                "s_m",
                "v_ref_m_s",
                "a_ref_m_s2",
                "jerk_ref_m_s3",
                "method",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a Biagiotti-style slosh-aware shaped speed profile CSV for fixed-path MPC."
    )
    parser.add_argument("--path-file", required=True, help="Fixed path JSON from fixed_global_path_runner.py")
    parser.add_argument("--out-csv", required=True, help="Output CSV consumed by path_handler/external_speed_profile_csv")
    parser.add_argument("--plot", default="", help="Optional preview PNG path")
    parser.add_argument("--debug-prefix", default="", help="Optional prefix for raw/shaped time-law CSVs")
    parser.add_argument("--v-max", type=float, default=0.8, help="Maximum path speed [m/s]")
    parser.add_argument("--a-max", type=float, default=0.6, help="Maximum tangential acceleration [m/s^2]")
    parser.add_argument("--decel-max", type=float, default=0.8, help="Maximum tangential deceleration [m/s^2]")
    parser.add_argument("--start-speed", type=float, default=0.0, help="Start speed [m/s]")
    parser.add_argument("--goal-speed", type=float, default=0.0, help="Goal speed [m/s]")
    parser.add_argument("--num-samples", type=int, default=201, help="Number of output samples if --ds is not set")
    parser.add_argument("--ds", type=float, default=0.0, help="Output spacing in meters; overrides --num-samples when >0")
    parser.add_argument("--omega-n", type=float, required=True, help="Liquid natural frequency [rad/s]")
    parser.add_argument("--damping-ratio", type=float, default=0.05, help="Liquid modal damping ratio [-]")
    parser.add_argument("--dt", "--delta-time", dest="dt", type=float, default=0.02,
                        help="Uniform shaping time step [s]")
    parser.add_argument("--method-name", default="BIAGIOTTI_STYLE", help="Method label stored in the CSV")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.dt <= 0.0:
        raise RuntimeError("--dt must be positive")

    points = load_path_points(args.path_file)
    path_s = cumulative_s(points)
    total_len = path_s[-1]
    s_grid = build_s_grid(total_len, args.num_samples, args.ds)
    base_v = retime_accel_limited(
        s_grid,
        args.v_max,
        args.a_max,
        args.decel_max,
        args.start_speed,
        args.goal_speed,
    )
    base_t, base_a, base_j = compute_time_accel_jerk(s_grid, base_v)
    t, shaped_s, shaped_v, shaped_a, shaped_j, shaper, distance_scale = shape_acceleration_time_law(
        base_t,
        base_a,
        total_len,
        args,
    )

    filtered = []
    last_s = -1.0
    for sample in zip(t, shaped_s, shaped_v, shaped_a, shaped_j):
        if sample[1] > last_s + 1e-9:
            filtered.append(sample)
            last_s = sample[1]
    if filtered[-1][1] < total_len:
        filtered.append((filtered[-1][0], total_len, max(0.0, args.goal_speed), 0.0, 0.0))

    pose_grid = interp_points(points, path_s, [sample[1] for sample in filtered])
    rows = []
    for sample, pose in zip(filtered, pose_grid):
        ti, s, v, a, j = sample
        x, y, yaw = pose
        rows.append(
            {
                "s_normalized": s / total_len,
                "s_m": s,
                "t_s": ti,
                "x": x,
                "y": y,
                "yaw": yaw,
                "v_ref_m_s": v,
                "a_ref_m_s2": a,
                "jerk_ref_m_s3": j,
                "method": args.method_name,
            }
        )

    rows[0]["v_ref_m_s"] = max(0.0, args.start_speed)
    rows[-1]["s_normalized"] = 1.0
    rows[-1]["s_m"] = total_len
    rows[-1]["v_ref_m_s"] = max(0.0, args.goal_speed)
    rows[-1]["a_ref_m_s2"] = 0.0
    rows[-1]["jerk_ref_m_s3"] = 0.0

    write_csv(args.out_csv, rows)
    maybe_plot(args.plot, rows)

    if args.debug_prefix:
        prefix = Path(args.debug_prefix)
        raw_rows = [
            {
                "t_s": base_t[i],
                "s_m": s_grid[i],
                "v_ref_m_s": base_v[i],
                "a_ref_m_s2": base_a[i],
                "jerk_ref_m_s3": base_j[i],
                "method": "RAW_ACCEL_LIMITED",
            }
            for i in range(len(s_grid))
        ]
        shaped_rows = [
            {
                "t_s": r["t_s"],
                "s_m": r["s_m"],
                "v_ref_m_s": r["v_ref_m_s"],
                "a_ref_m_s2": r["a_ref_m_s2"],
                "jerk_ref_m_s3": r["jerk_ref_m_s3"],
                "method": args.method_name,
            }
            for r in rows
        ]
        write_debug_time_csv(f"{prefix}_raw_time_law.csv", raw_rows)
        write_debug_time_csv(f"{prefix}_shaped_time_law.csv", shaped_rows)

    shaper_text = ", ".join(f"(t={delay:.3f}, A={amp:.3f})" for delay, amp in shaper)
    print(
        f"[OK] wrote {args.out_csv}: samples={len(rows)} length={total_len:.3f}m "
        f"duration={rows[-1]['t_s']:.3f}s vmax={max(r['v_ref_m_s'] for r in rows):.3f}m/s "
        f"omega_n={args.omega_n:.3f}rad/s zeta={args.damping_ratio:.3f} "
        f"distance_scale={distance_scale:.4f} shaper=[{shaper_text}]"
    )


if __name__ == "__main__":
    main()
