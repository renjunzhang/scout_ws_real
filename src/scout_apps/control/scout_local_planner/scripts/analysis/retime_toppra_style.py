#!/usr/bin/env python3
"""Generate a TOPPRA-style acceleration-limited v_ref(s) CSV for a fixed path.

This is a dependency-free retiming baseline helper. It keeps the fixed path
geometry unchanged and only computes a scalar path-speed profile.
"""

import argparse
import csv
import json
import math
from pathlib import Path


def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def load_path_points(path_file):
    data = json.loads(Path(path_file).read_text())
    poses = data.get("poses", data if isinstance(data, list) else [])
    points = []
    for item in poses:
        if "pose" in item:
            pose = item["pose"]
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            x = float(pos.get("x", 0.0))
            y = float(pos.get("y", 0.0))
            yaw = yaw_from_quat(
                float(ori.get("x", 0.0)),
                float(ori.get("y", 0.0)),
                float(ori.get("z", 0.0)),
                float(ori.get("w", 1.0)),
            )
        else:
            x = float(item["x"])
            y = float(item["y"])
            if "yaw" in item:
                yaw = float(item["yaw"])
            else:
                yaw = yaw_from_quat(
                    float(item.get("qx", 0.0)),
                    float(item.get("qy", 0.0)),
                    float(item.get("qz", 0.0)),
                    float(item.get("qw", 1.0)),
                )
        points.append((x, y, yaw))
    if len(points) < 2:
        raise RuntimeError(f"path needs at least 2 poses: {path_file}")
    return points


def cumulative_s(points):
    s = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        s.append(s[-1] + math.hypot(dx, dy))
    if s[-1] <= 1e-9:
        raise RuntimeError("path length is zero")
    return s


def interp_points(points, path_s, s_grid):
    out = []
    idx = 1
    for s in s_grid:
        while idx + 1 < len(path_s) and path_s[idx] < s:
            idx += 1
        s0 = path_s[idx - 1]
        s1 = path_s[idx]
        ratio = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        x = points[idx - 1][0] + ratio * (points[idx][0] - points[idx - 1][0])
        y = points[idx - 1][1] + ratio * (points[idx][1] - points[idx - 1][1])
        dx = points[idx][0] - points[idx - 1][0]
        dy = points[idx][1] - points[idx - 1][1]
        yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else points[idx - 1][2]
        out.append((x, y, yaw))
    return out


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


def write_csv(out_file, rows):
    out_path = Path(out_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "s_normalized",
                "s_m",
                "t_s",
                "x",
                "y",
                "yaw",
                "v_ref_m_s",
                "a_ref_m_s2",
                "jerk_ref_m_s3",
                "method",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def maybe_plot(plot_file, rows):
    if not plot_file:
        return
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as ex:
        print(f"[WARN] matplotlib unavailable, skip plot: {ex}")
        return

    s = [r["s_normalized"] for r in rows]
    v = [r["v_ref_m_s"] for r in rows]
    a = [r["a_ref_m_s2"] for r in rows]
    j = [r["jerk_ref_m_s3"] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    axes[0].plot(s, v)
    axes[0].set_ylabel("v_ref [m/s]")
    axes[1].plot(s, a)
    axes[1].set_ylabel("a_ref [m/s^2]")
    axes[2].plot(s, j)
    axes[2].set_ylabel("jerk_ref [m/s^3]")
    axes[2].set_xlabel("normalized path progress s")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    plot_path = Path(plot_file)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)


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
