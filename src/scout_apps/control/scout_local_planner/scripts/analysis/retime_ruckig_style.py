#!/usr/bin/env python3
"""Generate a Ruckig-style jerk-limited v_ref(s) CSV for a fixed path.

This script uses the Python `ruckig` package when available. It treats the
fixed-path arc length as a single DoF position and keeps path geometry unchanged.
"""

import argparse
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from retime_toppra_style import (
    cumulative_s,
    interp_points,
    load_path_points,
    maybe_plot,
    write_csv,
)


def require_ruckig():
    try:
        from ruckig import InputParameter, OutputParameter, Result, Ruckig
    except ImportError as ex:
        raise SystemExit(
            "Python package `ruckig` is required for Ruckig-style retiming. "
            "Install it in the experiment environment, then rerun this script."
        ) from ex
    return InputParameter, OutputParameter, Result, Ruckig


def generate_ruckig_profile(total_len, args):
    InputParameter, OutputParameter, Result, Ruckig = require_ruckig()

    otg = Ruckig(1, args.delta_time)
    inp = InputParameter(1)
    out = OutputParameter(1)

    inp.current_position = [0.0]
    inp.current_velocity = [max(0.0, args.start_speed)]
    inp.current_acceleration = [0.0]
    inp.target_position = [total_len]
    inp.target_velocity = [max(0.0, args.goal_speed)]
    inp.target_acceleration = [0.0]
    inp.max_velocity = [max(0.0, args.v_max)]
    inp.max_acceleration = [max(0.0, args.a_max)]
    inp.max_jerk = [max(0.0, args.j_max)]

    samples = [(0.0, 0.0, inp.current_velocity[0], inp.current_acceleration[0], 0.0)]
    result = Result.Working
    last_acc = inp.current_acceleration[0]
    steps = 0
    while result == Result.Working and steps < args.max_steps:
        result = otg.update(inp, out)
        if result not in (Result.Working, Result.Finished):
            raise RuntimeError(f"Ruckig failed with result code {result}")
        t = samples[-1][0] + args.delta_time
        s = max(0.0, min(total_len, out.new_position[0]))
        v = max(0.0, out.new_velocity[0])
        a = out.new_acceleration[0]
        j = (a - last_acc) / args.delta_time if args.delta_time > 1e-9 else 0.0
        samples.append((t, s, v, a, j))
        last_acc = a
        out.pass_to_input(inp)
        steps += 1

    if result == Result.Working:
        raise RuntimeError(
            f"Ruckig did not finish within max_steps={args.max_steps}; "
            "increase --max-steps or check constraints."
        )

    if samples[-1][1] < total_len - 1e-6:
        t = samples[-1][0] + args.delta_time
        samples.append((t, total_len, max(0.0, args.goal_speed), 0.0, 0.0))
    return samples


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a Ruckig-style jerk-limited speed profile CSV for fixed-path MPC."
    )
    parser.add_argument("--path-file", required=True, help="Fixed path JSON from fixed_global_path_runner.py")
    parser.add_argument("--out-csv", required=True, help="Output CSV consumed by path_handler/external_speed_profile_csv")
    parser.add_argument("--plot", default="", help="Optional preview PNG path")
    parser.add_argument("--v-max", type=float, default=0.8, help="Maximum path speed [m/s]")
    parser.add_argument("--a-max", type=float, default=0.6, help="Maximum tangential acceleration [m/s^2]")
    parser.add_argument("--j-max", type=float, default=1.5, help="Maximum tangential jerk [m/s^3]")
    parser.add_argument("--start-speed", type=float, default=0.0, help="Start speed [m/s]")
    parser.add_argument("--goal-speed", type=float, default=0.0, help="Goal speed [m/s]")
    parser.add_argument("--delta-time", type=float, default=0.02, help="Ruckig discretization time [s]")
    parser.add_argument("--max-steps", type=int, default=20000, help="Safety iteration limit")
    parser.add_argument("--method-name", default="RUCKIG_STYLE", help="Method label stored in the CSV")
    return parser.parse_args()


def main():
    args = parse_args()
    points = load_path_points(args.path_file)
    path_s = cumulative_s(points)
    total_len = path_s[-1]
    samples = generate_ruckig_profile(total_len, args)
    filtered = []
    last_s = -1.0
    for sample in samples:
        if sample[1] > last_s + 1e-9:
            filtered.append(sample)
            last_s = sample[1]
    samples = filtered
    pose_grid = interp_points(points, path_s, [sample[1] for sample in samples])

    rows = []
    for sample, pose in zip(samples, pose_grid):
        t, s, v, a, j = sample
        x, y, yaw = pose
        rows.append(
            {
                "s_normalized": s / total_len,
                "s_m": s,
                "t_s": t,
                "x": x,
                "y": y,
                "yaw": yaw,
                "v_ref_m_s": v,
                "a_ref_m_s2": a,
                "jerk_ref_m_s3": j,
                "method": args.method_name,
            }
        )

    write_csv(args.out_csv, rows)
    maybe_plot(args.plot, rows)
    print(
        f"[OK] wrote {args.out_csv}: samples={len(rows)} length={total_len:.3f}m "
        f"duration={rows[-1]['t_s']:.3f}s vmax={max(r['v_ref_m_s'] for r in rows):.3f}m/s"
    )


if __name__ == "__main__":
    main()
