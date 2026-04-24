#!/usr/bin/env python3
"""Trajectory smoothness / speed / acceleration analysis for 10 slosh experiment bags.

Outputs:
  - trajectory_overlay.png    : normalized x-y paths (all bags)
  - speed_profiles.png        : v(t) per bag, TRACKING phase highlighted
  - accel_profiles.png        : a(t) = dv/dt (odom-derived, smoothed)
  - omega_profiles.png        : ω(t) from cmd_vel
  - jerk_profiles.png         : d(a)/dt
  - speed_vs_arclen.png       : v(s) vs arc-length (path-progress axis)
  - metrics_summary.csv       : per-bag numeric metrics
  - metrics_table.png         : visual summary table

Usage:
    python3 trajectory_analysis.py --out-dir /tmp/traj_0422
"""

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import csv
import statistics

try:
    import rosbag
except ImportError:
    print("ERROR: source ROS environment first.", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    _MPL = True
except ImportError:
    _MPL = False
    print("[WARN] matplotlib not found; plots skipped.")

# ── Bag list ─────────────────────────────────────────────────────────────────
BAG_DIR = Path("/data/a/slosh_bags/real/0422")

BAG_LIST = [
    ("NOM",        BAG_DIR / "slosh_Q0_20260422_145046_block1_NOM.bag",        "#aaaaaa", "-"),
    ("ISR",        BAG_DIR / "slosh_Q0_20260422_145304_block1_ISR.bag",        "#88ccff", "-"),
    ("PROP1_Q5",   BAG_DIR / "slosh_Q0_20260422_145652_5.bag",                 "#ffaa44", "--"),
    ("PROP2_Q5",   BAG_DIR / "slosh_Q0_20260422_145809_5.bag",                 "#ff6600", "-"),
    ("FAS_Q5",     BAG_DIR / "slosh_Q0_20260422_145913_5.bag",                 "#44ff88", "-"),
    ("PROP2_Q10",  BAG_DIR / "slosh_Q0_20260422_150105_10.bag",                "#ff3333", "-"),
    ("PROP2_Q0",   BAG_DIR / "slosh_Q0_20260422_150240_10block1_PROP2.bag",    "#ffff44", "--"),
    ("b2_Q0",      BAG_DIR / "slosh_Q0_20260422_150442_10block2_Q0.bag",       "#aaaaaa", ":"),
    ("b2_Q5",      BAG_DIR / "slosh_Q0_20260422_150601_5.bag",                 "#44aaff", ":"),
    ("b2_Q10",     BAG_DIR / "slosh_Q0_20260422_150717_10.bag",                "#aa44ff", ":"),
]

GROUPS = {
    "group1_main_compare": ["NOM", "ISR", "FAS_Q5", "PROP2_Q5"],
    "group2_block2_q_sweep": ["b2_Q0", "b2_Q5", "b2_Q10"],
    "group3_scheduler_compare": ["PROP1_Q5", "FAS_Q5", "PROP2_Q5"],
}

TOPICS = ["/odom", "/cmd_vel", "/mpc_status"]

# ── Helpers ───────────────────────────────────────────────────────────────────

def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def smooth(vals: List[float], window: int = 5) -> List[float]:
    """Simple moving average."""
    if window <= 1 or len(vals) < window:
        return vals
    out = []
    half = window // 2
    for i in range(len(vals)):
        seg = vals[max(0, i - half): i + half + 1]
        out.append(sum(seg) / len(seg))
    return out


def derivative(ts: List[float], vs: List[float]) -> List[float]:
    """First derivative dv/dt via central differences."""
    n = len(ts)
    if n < 2:
        return [0.0] * n
    out = [0.0] * n
    for i in range(1, n - 1):
        dt = ts[i + 1] - ts[i - 1]
        out[i] = (vs[i + 1] - vs[i - 1]) / dt if dt > 1e-9 else 0.0
    out[0] = out[1]
    out[-1] = out[-2]
    return out


def arc_length(xs: List[float], ys: List[float]) -> List[float]:
    s = [0.0]
    for i in range(1, len(xs)):
        ds = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        s.append(s[-1] + ds)
    return s


def normalize_trajectory(xs, ys, yaws):
    """Translate to origin, rotate so initial heading = +x."""
    x0, y0, th0 = xs[0], ys[0], yaws[0]
    cos_th, sin_th = math.cos(-th0), math.sin(-th0)
    nx, ny = [], []
    for x, y in zip(xs, ys):
        dx, dy = x - x0, y - y0
        nx.append(cos_th * dx - sin_th * dy)
        ny.append(sin_th * dx + cos_th * dy)
    return nx, ny


def percentile(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    idx = max(0, min(len(s) - 1, int(p / 100.0 * (len(s) - 1))))
    return s[idx]


# ── Data loading ──────────────────────────────────────────────────────────────

def load_bag(bag_path: Path) -> Optional[Dict]:
    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"  [ERROR] {e}")
        return None

    t0 = bag.get_start_time()
    odom, cmds, modes = [], [], []

    with bag:
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            t = stamp.to_sec() - t0
            if topic == "/odom":
                odom.append((t, {
                    "x":   float(msg.pose.pose.position.x),
                    "y":   float(msg.pose.pose.position.y),
                    "yaw": yaw_from_quat(msg.pose.pose.orientation),
                    "v":   float(msg.twist.twist.linear.x),
                    "w":   float(msg.twist.twist.angular.z),
                }))
            elif topic == "/cmd_vel":
                cmds.append((t, float(msg.linear.x), float(msg.angular.z)))
            elif topic == "/mpc_status":
                modes.append((t, str(msg.data)))

    if not odom:
        return None

    return {"odom": odom, "cmds": cmds, "modes": modes,
            "duration": bag.get_end_time() - t0}


# ── Per-bag derived series ────────────────────────────────────────────────────

def derive_series(data: Dict) -> Dict:
    odom = data["odom"]
    cmds = data["cmds"]
    modes = data["modes"]

    ts_o  = [t for t, _ in odom]
    xs    = [s["x"]   for _, s in odom]
    ys    = [s["y"]   for _, s in odom]
    yaws  = [s["yaw"] for _, s in odom]
    vs    = [s["v"]   for _, s in odom]
    ws_o  = [s["w"]   for _, s in odom]

    # Normalize trajectory
    nx, ny = normalize_trajectory(xs, ys, yaws)
    arc    = arc_length(nx, ny)

    # Acceleration from odom_v (smoothed before differentiation)
    vs_smooth  = smooth(vs, window=7)
    accel_raw  = derivative(ts_o, vs_smooth)
    accel      = smooth(accel_raw, window=9)
    jerk_raw   = derivative(ts_o, accel)
    jerk       = smooth(jerk_raw, window=9)

    # cmd_vel series
    ts_c  = [t for t, _, _ in cmds]
    cmd_v = [v for _, v, _ in cmds]
    cmd_w = [w for _, _, w in cmds]

    # TRACKING mask on odom timestamps
    mode_idx = 0
    tracking_mask = []
    for t in ts_o:
        while mode_idx + 1 < len(modes) and modes[mode_idx + 1][0] <= t:
            mode_idx += 1
        cur_mode = modes[mode_idx][1] if modes else ""
        tracking_mask.append(cur_mode == "TRACKING")

    # TRACKING-only series
    trk_ts  = [t  for t, m in zip(ts_o, tracking_mask) if m]
    trk_v   = [v  for v, m in zip(vs,   tracking_mask) if m]
    trk_a   = [a  for a, m in zip(accel, tracking_mask) if m]
    trk_j   = [j  for j, m in zip(jerk,  tracking_mask) if m]
    trk_arc = [s  for s, m in zip(arc,   tracking_mask) if m]
    trk_w   = [w  for w, m in zip(ws_o,  tracking_mask) if m]

    return {
        "ts": ts_o, "xs": nx, "ys": ny, "vs": vs, "arc": arc,
        "accel": accel, "jerk": jerk,
        "ts_c": ts_c, "cmd_v": cmd_v, "cmd_w": cmd_w,
        "tracking_mask": tracking_mask,
        "trk_ts": trk_ts, "trk_v": trk_v, "trk_a": trk_a,
        "trk_j": trk_j, "trk_arc": trk_arc, "trk_w": trk_w,
    }


# ── Metrics ───────────────────────────────────────────────────────────────────

def compute_metrics(label: str, d: Dict) -> Dict:
    trk_v = d["trk_v"]
    trk_a = d["trk_a"]
    trk_j = d["trk_j"]
    arc   = d["arc"]
    vs    = d["vs"]
    cmd_v = d["cmd_v"]

    path_len = arc[-1] if arc else 0.0

    def rms(vals):
        if not vals:
            return float("nan")
        return math.sqrt(sum(v * v for v in vals) / len(vals))

    return {
        "label":        label,
        "path_len_m":   f"{path_len:.2f}",
        "v_mean":       f"{statistics.mean(trk_v):.3f}"  if trk_v else "-",
        "v_max":        f"{max(trk_v):.3f}"              if trk_v else "-",
        "v_p95":        f"{percentile(trk_v, 95):.3f}"  if trk_v else "-",
        "a_rms":        f"{rms(trk_a):.4f}"              if trk_a else "-",
        "a_max":        f"{max(abs(a) for a in trk_a):.4f}" if trk_a else "-",
        "jerk_rms":     f"{rms(trk_j):.4f}"              if trk_j else "-",
        "jerk_max":     f"{max(abs(j) for j in trk_j):.4f}" if trk_j else "-",
        "cmd_v_p95":    f"{percentile(cmd_v, 95):.3f}"  if cmd_v else "-",
    }


# ── Plots ─────────────────────────────────────────────────────────────────────

_BG = "#1a1a1a"
_GRID = dict(color="#333333", linewidth=0.5, linestyle="--", alpha=0.6)


def _ax_style(ax, xlabel="", ylabel="", title=""):
    ax.set_facecolor(_BG)
    ax.tick_params(colors="white", labelsize=8)
    for sp in ax.spines.values():
        sp.set_color("#444444")
    ax.grid(**_GRID)
    if xlabel:
        ax.set_xlabel(xlabel, color="white", fontsize=8)
    if ylabel:
        ax.set_ylabel(ylabel, color="white", fontsize=8)
    if title:
        ax.set_title(title, color="white", fontsize=9)


def plot_trajectory_overlay(all_series, out_path):
    fig, ax = plt.subplots(figsize=(9, 7))
    fig.patch.set_facecolor(_BG)
    _ax_style(ax, "x (m, normalized)", "y (m, normalized)", "Trajectory overlay (start=origin, initial heading=+x)")

    for label, color, ls, d in all_series:
        xs, ys = d["xs"], d["ys"]
        ax.plot(xs, ys, color=color, linestyle=ls, linewidth=1.2, label=label, alpha=0.85)
        ax.plot(xs[0], ys[0], "o", color=color, markersize=5)
        ax.plot(xs[-1], ys[-1], "s", color=color, markersize=4)

    ax.set_aspect("equal")
    ax.legend(fontsize=7, facecolor="#222222", loc="best", ncol=2)
    plt.tight_layout()
    plt.savefig(out_path, dpi=180, facecolor=_BG)
    plt.close()
    print(f"[OK] {out_path}")


def plot_time_series_panel(all_series, key_trk, key_cmd, ylabel, title, out_path,
                           trk_color_override=None, yline=None):
    """Generic 2-row panel: top=TRACKING series, bottom=full-bag cmd series."""
    n = len(all_series)
    cols = 5
    rows = math.ceil(n / cols) * 2  # 2 rows per band: tracking + cmd
    # Simpler: just overlay all bags on 2 axes
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 6), sharex=False)
    fig.patch.set_facecolor(_BG)
    _ax_style(ax1, "", ylabel, f"{title} — TRACKING phase (odom)")
    _ax_style(ax2, "time (s)", ylabel, f"{title} — full bag (cmd_vel)")

    for label, color, ls, d in all_series:
        if key_trk in d and d[key_trk]:
            ax1.plot(d["trk_ts"], d[key_trk], color=color, linestyle=ls,
                     linewidth=1.0, label=label, alpha=0.8)
        if key_cmd in d and d[key_cmd]:
            ax2.plot(d["ts_c"], d[key_cmd], color=color, linestyle=ls,
                     linewidth=1.0, label=label, alpha=0.8)

    if yline is not None:
        for ax in (ax1, ax2):
            ax.axhline(yline, color="#ff4444", linewidth=0.8, linestyle="--", alpha=0.5)

    ax1.legend(fontsize=7, facecolor="#222222", ncol=5)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor=_BG)
    plt.close()
    print(f"[OK] {out_path}")


def plot_speed_accel_panel(all_series, out_path):
    """4-panel: v, a, |jerk|, ω — all TRACKING phase, arc-length x-axis."""
    fig = plt.figure(figsize=(14, 10))
    fig.patch.set_facecolor(_BG)
    gs = gridspec.GridSpec(2, 2, hspace=0.45, wspace=0.35)

    ax_v = fig.add_subplot(gs[0, 0])
    ax_a = fig.add_subplot(gs[0, 1])
    ax_j = fig.add_subplot(gs[1, 0])
    ax_w = fig.add_subplot(gs[1, 1])

    _ax_style(ax_v, "arc length (m)", "v (m/s)", "Speed vs path progress")
    _ax_style(ax_a, "arc length (m)", "a (m/s²)", "Acceleration vs path progress")
    _ax_style(ax_j, "arc length (m)", "|jerk| (m/s³)", "Jerk magnitude vs path progress")
    _ax_style(ax_w, "arc length (m)", "ω (rad/s)", "Angular velocity vs path progress")

    for label, color, ls, d in all_series:
        arc = d["trk_arc"]
        if not arc:
            continue
        s0 = arc[0]
        arc_rel = [s - s0 for s in arc]

        if d["trk_v"]:
            ax_v.plot(arc_rel, d["trk_v"], color=color, linestyle=ls,
                      linewidth=1.0, label=label, alpha=0.82)
        if d["trk_a"]:
            ax_a.plot(arc_rel, d["trk_a"], color=color, linestyle=ls,
                      linewidth=1.0, alpha=0.75)
        if d["trk_j"]:
            ax_j.plot(arc_rel, [abs(j) for j in d["trk_j"]], color=color, linestyle=ls,
                      linewidth=0.9, alpha=0.75)
        if d["trk_w"]:
            ax_w.plot(arc_rel, d["trk_w"], color=color, linestyle=ls,
                      linewidth=1.0, alpha=0.82)

    ax_a.axhline(0, color="#555555", linewidth=0.6)
    ax_w.axhline(0, color="#555555", linewidth=0.6)
    ax_v.legend(fontsize=7, facecolor="#222222", ncol=2)

    plt.savefig(out_path, dpi=180, facecolor=_BG)
    plt.close()
    print(f"[OK] {out_path}")


def plot_metrics_table(metrics: List[Dict], out_path: Path):
    cols = ["label", "path_len_m", "v_mean", "v_max", "v_p95",
            "a_rms", "a_max", "jerk_rms", "jerk_max"]
    col_labels = ["label", "path_len(m)", "v_mean", "v_max", "v_p95",
                  "a_RMS", "a_max", "jerk_RMS", "jerk_max"]

    fig, ax = plt.subplots(figsize=(13, 4))
    fig.patch.set_facecolor(_BG)
    ax.set_facecolor(_BG)
    ax.axis("off")

    rows = [[m[c] for c in cols] for m in metrics]
    tbl = ax.table(
        cellText=rows,
        colLabels=col_labels,
        cellLoc="center",
        loc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    for (row, col), cell in tbl.get_celld().items():
        if row == 0:
            cell.set_facecolor("#2a4a6a")
            cell.set_text_props(color="white", fontweight="bold")
        else:
            cell.set_facecolor("#1e2a36" if row % 2 == 0 else "#243040")
            cell.set_text_props(color="white")
        cell.set_edgecolor("#444444")

    ax.set_title("Trajectory metrics – TRACKING phase (odom-derived)",
                 color="white", fontsize=10, pad=10)
    plt.tight_layout()
    plt.savefig(out_path, dpi=180, facecolor=_BG)
    plt.close()
    print(f"[OK] {out_path}")


def plot_block_comparison(all_series, out_path):
    """Side-by-side: block1 vs block2 trajectories."""
    block1 = [(lbl, c, ls, d) for lbl, c, ls, d in all_series if not lbl.startswith("b2")]
    block2 = [(lbl, c, ls, d) for lbl, c, ls, d in all_series if lbl.startswith("b2")]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor(_BG)

    for ax, group, title in [
        (axes[0], block1, "Block 1 (control conditions)"),
        (axes[1], block2, "Block 2 (Q sweep)"),
    ]:
        _ax_style(ax, "x (m)", "y (m)", title)
        for label, color, ls, d in group:
            ax.plot(d["xs"], d["ys"], color=color, linestyle=ls,
                    linewidth=1.4, label=label, alpha=0.85)
            ax.plot(d["xs"][0], d["ys"][0], "o", color=color, markersize=6)
            ax.plot(d["xs"][-1], d["ys"][-1], "s", color=color, markersize=4)
        ax.set_aspect("equal")
        ax.legend(fontsize=8, facecolor="#222222")

    plt.tight_layout()
    plt.savefig(out_path, dpi=180, facecolor=_BG)
    plt.close()
    print(f"[OK] {out_path}")


def select_group(all_series, all_metrics, labels):
    label_set = set(labels)
    group_series = [item for item in all_series if item[0] in label_set]
    group_metrics = [item for item in all_metrics if item["label"] in label_set]
    return group_series, group_metrics


def write_metrics_csv(metrics: List[Dict], out_path: Path):
    if not metrics:
        return
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=metrics[0].keys())
        writer.writeheader()
        writer.writerows(metrics)
    print(f"[OK] {out_path}")


def plot_group_outputs(group_name: str, group_series, group_metrics, out_dir: Path):
    if not group_series:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    if _MPL:
        plot_trajectory_overlay(group_series, out_dir / "trajectory_overlay.png")
        plot_speed_accel_panel(group_series, out_dir / "speed_accel_panel.png")
        plot_time_series_panel(
            group_series, "trk_v", "cmd_v", "v (m/s)", f"{group_name} speed",
            out_dir / "speed_time_series.png"
        )
        plot_time_series_panel(
            group_series, "trk_w", "cmd_w", "ω (rad/s)", f"{group_name} angular velocity",
            out_dir / "omega_time_series.png", yline=0.0
        )
        plot_metrics_table(group_metrics, out_dir / "metrics_table.png")
    write_metrics_csv(group_metrics, out_dir / "metrics_summary.csv")


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out-dir", default="/tmp/traj_0422")
    p.add_argument("--smooth-window", type=int, default=7,
                   help="Moving-average window for acceleration smoothing")
    return p.parse_args()


def main():
    args = parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    all_series = []
    all_metrics = []

    for label, bag_path, color, ls in BAG_LIST:
        if not bag_path.exists():
            print(f"[SKIP] {label}: not found")
            continue
        print(f"[{label}] loading ...")
        raw = load_bag(bag_path)
        if raw is None:
            continue
        d = derive_series(raw)
        m = compute_metrics(label, d)
        all_series.append((label, color, ls, d))
        all_metrics.append(m)
        print(f"  path={m['path_len_m']}m  v_mean={m['v_mean']}  "
              f"a_rms={m['a_rms']}  jerk_rms={m['jerk_rms']}")

    if not all_series:
        print("No bags loaded.")
        return 1

    if _MPL:
        print("\nGenerating plots ...")
        plot_trajectory_overlay(all_series, out / "trajectory_overlay.png")
        plot_block_comparison(all_series, out / "trajectory_block_compare.png")
        plot_speed_accel_panel(all_series, out / "speed_accel_panel.png")

        # v(t) time-series
        plot_time_series_panel(
            all_series, "trk_v", "cmd_v", "v (m/s)", "Speed",
            out / "speed_time_series.png"
        )
        # ω(t) time-series
        plot_time_series_panel(
            all_series, "trk_w", "cmd_w", "ω (rad/s)", "Angular velocity",
            out / "omega_time_series.png", yline=0.0
        )

        plot_metrics_table(all_metrics, out / "metrics_table.png")

    # CSV
    csv_path = out / "metrics_summary.csv"
    if all_metrics:
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=all_metrics[0].keys())
            writer.writeheader()
            writer.writerows(all_metrics)
        print(f"[OK] {csv_path}")

    for group_name, labels in GROUPS.items():
        group_series, group_metrics = select_group(all_series, all_metrics, labels)
        plot_group_outputs(group_name, group_series, group_metrics, out / group_name)

    # Print table
    print("\nMETRICS SUMMARY")
    print(f"{'label':12s} {'len(m)':>7} {'v_mean':>7} {'v_max':>7} "
          f"{'a_rms':>8} {'a_max':>8} {'jerk_rms':>9} {'jerk_max':>9}")
    print("-" * 75)
    for m in all_metrics:
        print(f"{m['label']:12s} {m['path_len_m']:>7} {m['v_mean']:>7} {m['v_max']:>7} "
              f"{m['a_rms']:>8} {m['a_max']:>8} {m['jerk_rms']:>9} {m['jerk_max']:>9}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
