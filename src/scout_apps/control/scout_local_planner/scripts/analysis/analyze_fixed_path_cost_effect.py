#!/usr/bin/env python3
"""Analyze fixed-path MPC cost experiments from real robot bags.

The script is intentionally independent of OSCRS.  It summarizes the behavior
that matters for SloshPriorityMPC validation:

* normalized trajectory overlay
* odom speed, longitudinal acceleration, lateral acceleration and jerk
* /slosh/height model-side curves
* optional RGB red-liquid CSV metrics exported by red_liquid_infer_from_bag.py

Main-effect statistics are computed from TRACKING start to the first terminal
or capture-related mode.  Terminal stopping is a safety/convergence mechanism
and should not be mixed into the slosh-cost effect window.
"""

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

try:
    import rosbag
except ImportError:
    print("ERROR: source ROS environment first.", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


TOPICS = ["/odom", "/cmd_vel", "/mpc_status", "/terminal/mode", "/slosh/height"]
COLORS = {
    "C": "#1f77b4",
    "D": "#d62728",
    "Q0": "#1f77b4",
    "Q5": "#d62728",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bags", nargs="*", help="Input bag files. If omitted, use --bag-dir/--glob.")
    p.add_argument("--bag-dir", default="", help="Directory containing bags.")
    p.add_argument("--glob", default="*.bag", help="Glob for --bag-dir. Default: *.bag")
    p.add_argument("--out-dir", required=True, help="Output analysis directory.")
    p.add_argument("--red-infer-dir", default="", help="Directory containing per-bag red-liquid CSV outputs.")
    p.add_argument("--exclude-static", action="store_true", help="Exclude bags whose name contains 'static'.")
    p.add_argument("--smooth-window", type=int, default=5, help="Moving average window for ax/jerk.")
    return p.parse_args()


def condition_from_name(name: str) -> str:
    if "_D_" in name or "_D_run" in name:
        return "D"
    if "_C_" in name or "_C_run" in name:
        return "C"
    m = re.search(r"_Q(\d+)_", name)
    if m:
        return f"Q{m.group(1)}"
    return "UNK"


def label_from_name(name: str) -> str:
    label = name.replace(".bag", "")
    label = label.replace("slosh_Q0_20260518_", "").replace("slosh_Q5_20260518_", "")
    label = label.replace("_P2_s_curve", "")
    return label


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def smooth(vals: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or len(vals) < window:
        return vals
    return np.convolve(vals, np.ones(window) / float(window), mode="same")


def derivative(ts: np.ndarray, vals: np.ndarray) -> np.ndarray:
    if len(ts) < 3:
        return np.zeros_like(vals)
    return np.gradient(vals, ts, edge_order=1)


def p95_abs(vals: np.ndarray) -> float:
    return float(np.percentile(np.abs(vals), 95)) if len(vals) else float("nan")


def rms(vals: np.ndarray) -> float:
    return float(math.sqrt(np.mean(vals * vals))) if len(vals) else float("nan")


def first_tracking_start(modes: List[Tuple[float, str]]) -> float:
    for t, m in modes:
        if m == "TRACKING":
            return t
    return 0.0


def first_terminal_time(terminal_modes: List[Tuple[float, str]], fallback: float) -> float:
    for t, m in terminal_modes:
        if m not in ("IDLE", "NONE", ""):
            return t
    return fallback


def load_bag(path: Path, smooth_window: int) -> Dict:
    odom = {"t": [], "x": [], "y": [], "yaw": [], "v": [], "w": []}
    cmd = {"t": [], "v": [], "w": []}
    modes: List[Tuple[float, str]] = []
    terminal_modes: List[Tuple[float, str]] = []
    slosh = {"t": [], "height_mm": []}

    with rosbag.Bag(str(path), "r") as bag:
        t0 = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            t = stamp.to_sec() - t0
            if topic == "/odom":
                odom["t"].append(t)
                odom["x"].append(float(msg.pose.pose.position.x))
                odom["y"].append(float(msg.pose.pose.position.y))
                odom["yaw"].append(yaw_from_quat(msg.pose.pose.orientation))
                odom["v"].append(float(msg.twist.twist.linear.x))
                odom["w"].append(float(msg.twist.twist.angular.z))
            elif topic == "/cmd_vel":
                cmd["t"].append(t)
                cmd["v"].append(float(msg.linear.x))
                cmd["w"].append(float(msg.angular.z))
            elif topic == "/mpc_status":
                modes.append((t, str(getattr(msg, "data", ""))))
            elif topic == "/terminal/mode":
                terminal_modes.append((t, str(getattr(msg, "data", ""))))
            elif topic == "/slosh/height":
                slosh["t"].append(t)
                slosh["height_mm"].append(abs(float(getattr(msg, "data", 0.0))) * 1000.0)

    t = np.asarray(odom["t"], dtype=float)
    x = np.asarray(odom["x"], dtype=float)
    y = np.asarray(odom["y"], dtype=float)
    yaw = np.asarray(odom["yaw"], dtype=float)
    v = np.asarray(odom["v"], dtype=float)
    w = np.asarray(odom["w"], dtype=float)

    if len(x):
        x0, y0, th0 = x[0], y[0], yaw[0]
        c, s = math.cos(-th0), math.sin(-th0)
        dx, dy = x - x0, y - y0
        nx = c * dx - s * dy
        ny = s * dx + c * dy
        ds = np.hypot(np.diff(nx), np.diff(ny))
        arc = np.concatenate([[0.0], np.cumsum(ds)])
    else:
        nx = ny = arc = np.asarray([], dtype=float)

    ax = smooth(derivative(t, v), smooth_window)
    ay = v * w
    jerk = smooth(derivative(t, ax), smooth_window)

    tracking_start = first_tracking_start(modes)
    terminal_time = first_terminal_time(terminal_modes, float(t[-1]) if len(t) else 0.0)

    return {
        "bag": path,
        "condition": condition_from_name(path.name),
        "label": label_from_name(path.name),
        "bag_start_abs": t0,
        "odom": {"t": t, "x": nx, "y": ny, "v": v, "w": w, "arc": arc, "ax": ax, "ay": ay, "jerk": jerk},
        "cmd": {k: np.asarray(vv, dtype=float) for k, vv in cmd.items()},
        "slosh": {k: np.asarray(vv, dtype=float) for k, vv in slosh.items()},
        "tracking_start": tracking_start,
        "terminal_time": terminal_time,
    }


def read_red_csv(red_infer_dir: Path, bag_stem: str) -> Optional[Dict[str, np.ndarray]]:
    csv_path = red_infer_dir / bag_stem / f"{bag_stem}_red_top.csv"
    if not csv_path.exists():
        return None
    rows = []
    with csv_path.open("r", newline="") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    if not rows:
        return None

    def arr(key: str) -> np.ndarray:
        vals = []
        for r in rows:
            s = r.get(key, "")
            vals.append(float(s) if s not in ("", None) else float("nan"))
        return np.asarray(vals, dtype=float)

    return {
        "t": arr("stamp_sec"),
        "h_smooth_corr": arr("h_mm_smooth_corr"),
        "h_max_lcr": np.nanmax(np.vstack([arr("h_mm_left"), arr("h_mm_center"), arr("h_mm_right")]), axis=0),
    }


def window_mask(t: np.ndarray, start: float, end: float) -> np.ndarray:
    return (t >= start) & (t <= end)


def compute_metrics(item: Dict, red: Optional[Dict[str, np.ndarray]]) -> Dict[str, float]:
    odom = item["odom"]
    m = window_mask(odom["t"], item["tracking_start"], item["terminal_time"])
    sh = item["slosh"]
    sm = window_mask(sh["t"], item["tracking_start"], item["terminal_time"]) if len(sh["t"]) else np.asarray([], dtype=bool)
    out: Dict[str, float] = {
        "tracking_start_s": item["tracking_start"],
        "first_terminal_s": item["terminal_time"],
        "pre_terminal_duration_s": max(0.0, item["terminal_time"] - item["tracking_start"]),
        "path_len_m": float(odom["arc"][-1]) if len(odom["arc"]) else 0.0,
        "v_p95": p95_abs(odom["v"][m]),
        "v_max": float(np.max(np.abs(odom["v"][m]))) if np.any(m) else float("nan"),
        "ax_p95": p95_abs(odom["ax"][m]),
        "ax_rms": rms(odom["ax"][m]),
        "ay_p95": p95_abs(odom["ay"][m]),
        "ay_rms": rms(odom["ay"][m]),
        "jerk_p95": p95_abs(odom["jerk"][m]),
        "jerk_rms": rms(odom["jerk"][m]),
        "slosh_height_p95_mm": float(np.percentile(sh["height_mm"][sm], 95)) if np.any(sm) else float("nan"),
        "slosh_height_rms_mm": rms(sh["height_mm"][sm]) if np.any(sm) else float("nan"),
        "slosh_height_peak_mm": float(np.max(sh["height_mm"][sm])) if np.any(sm) else float("nan"),
        "visual_p95_mm": float("nan"),
        "visual_rms_mm": float("nan"),
        "visual_peak_mm": float("nan"),
        "visual_max_lcr_peak_mm": float("nan"),
    }
    if red is not None:
        # red_liquid_infer_from_bag writes absolute ROS stamps. Convert to bag-relative.
        t = red["t"] - float(item["bag_start_abs"])
        vm = window_mask(t, item["tracking_start"], item["terminal_time"])
        vals = np.abs(red["h_smooth_corr"][vm])
        vals = vals[np.isfinite(vals)]
        max_vals = red["h_max_lcr"][vm]
        max_vals = max_vals[np.isfinite(max_vals)]
        if len(vals):
            out["visual_p95_mm"] = float(np.percentile(vals, 95))
            out["visual_rms_mm"] = rms(vals)
            out["visual_peak_mm"] = float(np.max(vals))
        if len(max_vals):
            out["visual_max_lcr_peak_mm"] = float(np.max(max_vals))
    return out


def red_relative_series(item: Dict, red_infer_dir: Optional[Path]) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    if red_infer_dir is None:
        return None
    red = read_red_csv(red_infer_dir, item["bag"].stem)
    if red is None:
        return None
    t = red["t"] - float(item["bag_start_abs"])
    h = np.abs(red["h_smooth_corr"])
    hmax = red["h_max_lcr"]
    return t, h, hmax


def write_csv(path: Path, rows: List[Dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(rows[0].keys()) if rows else []
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def plot_series(items: List[Dict], out_dir: Path, key: str, ylabel: str, getter, window: str) -> None:
    if plt is None:
        return
    fig, ax = plt.subplots(figsize=(11, 5), dpi=150)
    for item in items:
        t, y = getter(item)
        if window == "pre_terminal":
            m = window_mask(t, item["tracking_start"], item["terminal_time"])
            x = t[m] - item["tracking_start"]
            yy = y[m]
        else:
            x = t
            yy = y
        color = COLORS.get(item["condition"], "#666666")
        ax.plot(x, yy, color=color, alpha=0.55, linewidth=1.2, label=item["label"])
    ax.set_title(f"{ylabel} - {window}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / f"{key}_{window}.png")
    plt.close(fig)


def plot_red_visual(items: List[Dict], out_dir: Path, red_infer_dir: Optional[Path], window: str) -> None:
    if plt is None or red_infer_dir is None:
        return
    fig, ax = plt.subplots(figsize=(11, 5), dpi=150)
    for item in items:
        red = red_relative_series(item, red_infer_dir)
        if red is None:
            continue
        t, h, hmax = red
        if window == "pre_terminal":
            m = window_mask(t, item["tracking_start"], item["terminal_time"])
            x = t[m] - item["tracking_start"]
            y = h[m]
            ymax = hmax[m]
        else:
            x = t
            y = h
            ymax = hmax
        color = COLORS.get(item["condition"], "#666666")
        ax.plot(x, y, color=color, alpha=0.65, linewidth=1.4, label=item["label"])
        ax.plot(x, ymax, color=color, alpha=0.18, linewidth=0.8)
    ax.set_title(f"RGB visual liquid height - {window}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("abs(h_smooth_corr) mm; faint=max(L,C,R)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / f"rgb_visual_height_{window}.png")
    plt.close(fig)


def plot_trajectory(items: List[Dict], out_dir: Path) -> None:
    if plt is None:
        return
    fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
    for item in items:
        odom = item["odom"]
        color = COLORS.get(item["condition"], "#666666")
        ax.plot(odom["x"], odom["y"], color=color, alpha=0.65, linewidth=1.4, label=item["label"])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x normalized (m)")
    ax.set_ylabel("y normalized (m)")
    ax.set_title("Normalized trajectory overlay")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(out_dir / "trajectory_overlay.png")
    plt.close(fig)


def write_summary(path: Path, rows: List[Dict]) -> None:
    by_cond: Dict[str, List[Dict]] = defaultdict(list)
    for r in rows:
        by_cond[r["condition"]].append(r)

    def mean(vals: Iterable[float]) -> float:
        arr = np.asarray([v for v in vals if np.isfinite(v)], dtype=float)
        return float(np.mean(arr)) if len(arr) else float("nan")

    cols = ["v_p95", "ax_p95", "ay_p95", "jerk_p95", "slosh_height_p95_mm", "visual_p95_mm", "pre_terminal_duration_s"]
    with path.open("w", encoding="utf-8") as f:
        f.write("# Fixed Path Cost Effect Summary\n\n")
        f.write("统计窗口：TRACKING start 到第一次 terminal/capture 相关状态之前；terminal 停车段不进入主效果统计。\n\n")
        f.write("| condition | n | v_p95 | ax_p95 | ay_p95 | jerk_p95 | /slosh p95 mm | RGB p95 mm | tracking duration s |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for cond in sorted(by_cond.keys()):
            group = by_cond[cond]
            vals = {c: mean(r[c] for r in group) for c in cols}
            f.write(
                f"| {cond} | {len(group)} | {vals['v_p95']:.3f} | {vals['ax_p95']:.3f} | "
                f"{vals['ay_p95']:.3f} | {vals['jerk_p95']:.3f} | {vals['slosh_height_p95_mm']:.3f} | "
                f"{vals['visual_p95_mm']:.3f} | {vals['pre_terminal_duration_s']:.2f} |\n"
            )


def main() -> int:
    args = parse_args()
    bag_paths = [Path(p).expanduser().resolve() for p in args.bags]
    if args.bag_dir:
        bag_paths.extend(sorted(Path(args.bag_dir).expanduser().glob(args.glob)))
    if args.exclude_static:
        bag_paths = [p for p in bag_paths if "static" not in p.name]
    bag_paths = [p for p in bag_paths if p.exists()]
    if not bag_paths:
        print("ERROR: no bags found.", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    red_dir = Path(args.red_infer_dir).expanduser().resolve() if args.red_infer_dir else None

    items = [load_bag(p, args.smooth_window) for p in bag_paths]
    rows = []
    for item in items:
        red = read_red_csv(red_dir, item["bag"].stem) if red_dir else None
        metrics = compute_metrics(item, red)
        row = {"bag": item["bag"].name, "condition": item["condition"], **metrics}
        rows.append(row)

    write_csv(out_dir / "fixed_path_cost_effect_summary.csv", rows)
    write_summary(out_dir / "FIXED_PATH_COST_EFFECT_SUMMARY.md", rows)

    plot_trajectory(items, out_dir)
    plot_specs = [
        ("odom_v", "Odom v (m/s)", lambda it: (it["odom"]["t"], it["odom"]["v"])),
        ("odom_ax", "Odom ax (m/s^2)", lambda it: (it["odom"]["t"], it["odom"]["ax"])),
        ("odom_ay", "Odom ay=v*w (m/s^2)", lambda it: (it["odom"]["t"], it["odom"]["ay"])),
        ("odom_jerk", "Odom jerk (m/s^3)", lambda it: (it["odom"]["t"], it["odom"]["jerk"])),
        ("slosh_height", "/slosh/height abs (mm)", lambda it: (it["slosh"]["t"], it["slosh"]["height_mm"])),
    ]
    for window in ("full_bag", "pre_terminal"):
        for key, ylabel, getter in plot_specs:
            plot_series(items, out_dir, key, ylabel, getter, window)
        plot_red_visual(items, out_dir, red_dir, window)

    print(out_dir)
    print(out_dir / "FIXED_PATH_COST_EFFECT_SUMMARY.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
