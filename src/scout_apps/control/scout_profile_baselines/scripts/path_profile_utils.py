#!/usr/bin/env python3
"""Shared helpers for fixed-path speed-profile generators.

Used by retime_toppra_style.py / retime_ruckig_style.py / shape_biagiotti.py.
All of them keep the fixed path geometry unchanged and only compute a scalar
path-speed profile, then write the same CSV schema consumed by
`path_handler/external_speed_profile_csv`.
"""

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
