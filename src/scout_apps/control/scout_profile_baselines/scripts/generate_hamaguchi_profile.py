#!/usr/bin/env python3
"""Generate a Hamaguchi-style open-loop profile for a fixed path.

Hamaguchi and Taniguchi (2002) combine transfer input shaping with curved path
design for cylindrical liquid containers. For fixed-path benchmark fairness this
script does not redesign the path geometry. It keeps the supplied path fixed and
only generates an offline scalar speed profile using the paper's two-impulse
shaping timing plus conservative curvature-aware speed caps.

The script is ROS-free and monitor-free: it only reads a fixed path JSON and
writes a profile CSV for the shared external-profile tracker.
"""

import argparse
import math
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from advanced_profile_common import (  # noqa: E402
    build_s_grid,
    compute_curvature,
    compute_time_accel_jerk,
    hamaguchi_two_impulse_shaper,
    interp_series,
    retime_accel_limited,
    rows_from_profile,
    shape_time_law_with_impulses,
    smooth_series,
)
from path_profile_utils import (  # noqa: E402
    cumulative_s,
    interp_points,
    load_path_points,
    maybe_plot,
    write_csv,
)


def build_speed_limits_from_hamaguchi_shape(s_grid, pose_grid, base_t, base_a, total_len, args):
    delta_t, k, impulses = hamaguchi_two_impulse_shaper(args.omega_n, args.damping_ratio)
    _shape_t, shaped_s, shaped_v, _shape_a, distance_scale = shape_time_law_with_impulses(
        base_t,
        base_a,
        total_len,
        args.start_speed,
        args.dt,
        impulses,
    )

    shaped_caps = [min(args.v_max, max(0.0, interp_series(shaped_s, shaped_v, s))) for s in s_grid]
    curvature_caps = [args.v_max] * len(s_grid)
    if not args.no_curvature_cap:
        kappa = compute_curvature(s_grid, pose_grid)
        cap_scale = args.curvature_cap_scale
        if cap_scale <= 0.0:
            cap_scale = 1.0 / (1.0 + k)
        lateral_limit = max(1e-6, min(args.lateral_accel_max, args.a_max) * cap_scale)
        for i, curv in enumerate(kappa):
            abs_kappa = abs(curv)
            if abs_kappa > 1e-9:
                curvature_caps[i] = min(args.v_max, math.sqrt(lateral_limit / abs_kappa))

    raw_caps = [min(args.v_max, shaped_caps[i], curvature_caps[i]) for i in range(len(s_grid))]
    smoothed = smooth_series(raw_caps, passes=args.cap_smooth_passes)
    caps = []
    for i, cap in enumerate(raw_caps):
        if i == 0 or i == len(raw_caps) - 1:
            caps.append(cap)
        else:
            caps.append(max(args.min_speed, min(cap, smoothed[i], args.v_max)))
    return caps, delta_t, k, distance_scale


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a Hamaguchi-style two-impulse shaped speed profile CSV for a fixed path."
    )
    parser.add_argument("--path-file", required=True, help="Fixed path JSON from fixed_global_path_runner.py")
    parser.add_argument("--out-csv", required=True, help="Output CSV consumed by path_handler/external_speed_profile_csv")
    parser.add_argument("--plot", default="", help="Optional preview PNG path")
    parser.add_argument("--v-max", type=float, default=0.8, help="Maximum path speed [m/s]")
    parser.add_argument("--a-max", type=float, default=0.6, help="Maximum tangential acceleration [m/s^2]")
    parser.add_argument("--decel-max", type=float, default=0.6, help="Maximum tangential deceleration [m/s^2]")
    parser.add_argument("--start-speed", type=float, default=0.0, help="Start speed [m/s]")
    parser.add_argument("--goal-speed", type=float, default=0.0, help="Goal speed [m/s]")
    parser.add_argument("--num-samples", type=int, default=201, help="Number of output samples if --ds is not set")
    parser.add_argument("--ds", type=float, default=0.0, help="Output spacing in meters; overrides --num-samples when >0")
    parser.add_argument("--omega-n", type=float, required=True, help="Liquid natural frequency [rad/s]")
    parser.add_argument("--damping-ratio", type=float, default=0.05, help="Liquid modal damping ratio [-]")
    parser.add_argument("--dt", type=float, default=0.02, help="Uniform shaping integration time step [s]")
    parser.add_argument("--lateral-accel-max", type=float, default=0.6, help="Curvature-induced acceleration cap [m/s^2]")
    parser.add_argument("--curvature-cap-scale", type=float, default=0.0,
                        help="Optional cap scale; <=0 uses 1/(1+k) from the two-impulse shaper")
    parser.add_argument("--cap-smooth-passes", type=int, default=3, help="Smoothing passes for local speed caps")
    parser.add_argument("--min-speed", type=float, default=0.05, help="Minimum interior speed cap [m/s]")
    parser.add_argument("--no-curvature-cap", action="store_true", help="Disable conservative curvature speed caps")
    parser.add_argument("--method-name", default="HAMAGUCHI_STYLE", help="Method label stored in the CSV")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.dt <= 0.0:
        raise RuntimeError("--dt must be positive")
    if args.lateral_accel_max <= 0.0:
        raise RuntimeError("--lateral-accel-max must be positive")

    points = load_path_points(args.path_file)
    path_s = cumulative_s(points)
    total_len = path_s[-1]
    s_grid = build_s_grid(total_len, args.num_samples, args.ds)
    pose_grid = interp_points(points, path_s, s_grid)

    base_v = retime_accel_limited(
        s_grid,
        args.v_max,
        args.a_max,
        args.decel_max,
        args.start_speed,
        args.goal_speed,
    )
    base_t, base_a, _base_j = compute_time_accel_jerk(s_grid, base_v)
    caps, delta_t, k, distance_scale = build_speed_limits_from_hamaguchi_shape(
        s_grid,
        pose_grid,
        base_t,
        base_a,
        total_len,
        args,
    )
    v = retime_accel_limited(
        s_grid,
        args.v_max,
        args.a_max,
        args.decel_max,
        args.start_speed,
        args.goal_speed,
        speed_limits=caps,
    )
    t, a, j = compute_time_accel_jerk(s_grid, v)
    rows = rows_from_profile(s_grid, pose_grid, t, v, a, j, args.method_name)
    write_csv(args.out_csv, rows)
    maybe_plot(args.plot, rows)
    print(
        f"[OK] wrote {args.out_csv}: samples={len(rows)} length={total_len:.3f}m "
        f"duration={t[-1]:.3f}s vmax={max(v):.3f}m/s "
        f"DeltaT={delta_t:.4f}s k={k:.4f} distance_scale={distance_scale:.4f}"
    )


if __name__ == "__main__":
    main()
