#!/usr/bin/env python3
"""MPC behavior bug analysis for real-world slosh experiment bags.

Checks six known failure modes from the MPC architecture:
  A. u_prev desync   – large |Δcmd| immediately after a solve failure
  B. v_plan/v_exec   – odom slowdown cascades into v_des_eff cascade
  C. governor+sched  – speed_governor and risk_scheduler both active (PROP2)
  D. SETTLING time   – SETTLING duration vs expected sloshing decay period
  E. reentry ramp    – first 10 cmd_vel frames after TRACKING entry
  F. failure+kappa   – solve failures cluster at high-curvature reference

Also validates the dist/heading tracking-error metric from diagnose_real_tuning.py.

Usage:
    python3 mpc_bug_analysis.py --out-dir /tmp/mpc_bug_0422

Edit BAG_LIST below for your 10 bags.
"""

import argparse
import math
import os
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import rosbag
except ImportError:
    print("ERROR: rosbag not found. Source your ROS environment.", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    _HAS_MPL = True
except ImportError:
    _HAS_MPL = False
    print("[WARN] matplotlib not available; plots will be skipped.")

import csv
import statistics

# ── Bag list ────────────────────────────────────────────────────────────────
BAG_DIR = Path("/data/a/slosh_bags/real/0422")

BAG_LIST = [
    ("NOM",          BAG_DIR / "slosh_Q0_20260422_145046_block1_NOM.bag"),
    ("ISR",          BAG_DIR / "slosh_Q0_20260422_145304_block1_ISR.bag"),
    ("PROP1_Q5",     BAG_DIR / "slosh_Q0_20260422_145652_5.bag"),
    ("PROP2_Q5",     BAG_DIR / "slosh_Q0_20260422_145809_5.bag"),
    ("FAS_Q5",       BAG_DIR / "slosh_Q0_20260422_145913_5.bag"),
    ("PROP2_Q10",    BAG_DIR / "slosh_Q0_20260422_150105_10.bag"),
    ("PROP2_Q0",     BAG_DIR / "slosh_Q0_20260422_150240_10block1_PROP2.bag"),
    ("block2_Q0",    BAG_DIR / "slosh_Q0_20260422_150442_10block2_Q0.bag"),
    ("block2_Q5",    BAG_DIR / "slosh_Q0_20260422_150601_5.bag"),
    ("block2_Q10",   BAG_DIR / "slosh_Q0_20260422_150717_10.bag"),
]

TOPICS = [
    "/cmd_vel",
    "/odom",
    "/mpc/status_val",
    "/mpc/solve_ms",
    "/mpc_status",
    "/mpc/reference_path",
    "/slosh/v_des_eff",
    "/slosh/speed_governor_active",
    "/risk_scheduler/rho_k",
    "/risk_scheduler/fallback_active",
    "/terminal/mode",
    "/terminal/recovery_latched",
]

# ── Helpers ──────────────────────────────────────────────────────────────────

def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def path_points(msg) -> List[Tuple[float, float]]:
    pts = []
    for ps in msg.poses:
        x, y = float(ps.pose.position.x), float(ps.pose.position.y)
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    return pts


def max_kappa_of_path(pts: List[Tuple[float, float]]) -> float:
    if len(pts) < 3:
        return 0.0
    best = 0.0
    for i in range(1, len(pts) - 1):
        ax, ay = pts[i - 1]
        bx, by = pts[i]
        cx, cy = pts[i + 1]
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        denom = ab * bc * ac
        if denom < 1e-9:
            continue
        area2 = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
        kappa = 2.0 * area2 / denom
        if kappa > best:
            best = kappa
    return best


def nearest_value(series, t, idx=0):
    """Return (new_idx, value) nearest in time to t."""
    if not series:
        return idx, None
    while idx + 1 < len(series) and abs(series[idx + 1][0] - t) <= abs(series[idx][0] - t):
        idx += 1
    return idx, series[idx][1]


def percentile(vals: List[float], p: float) -> float:
    if not vals:
        return float("nan")
    s = sorted(vals)
    idx = max(0, min(len(s) - 1, int(p / 100.0 * (len(s) - 1))))
    return s[idx]


def _fmt(v) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return f"{v:.3f}"


# ── Per-bag data extraction ──────────────────────────────────────────────────

def load_bag(bag_path: Path) -> Dict:
    """Read all relevant topics into lists of (t_rel, value)."""
    d: Dict[str, list] = defaultdict(list)
    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"  [ERROR] cannot open {bag_path.name}: {e}")
        return {}

    t0 = bag.get_start_time()
    with bag:
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            t = stamp.to_sec() - t0
            if topic == "/cmd_vel":
                d[topic].append((t, (float(msg.linear.x), float(msg.angular.z))))
            elif topic == "/odom":
                d[topic].append((t, {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": yaw_from_quat(msg.pose.pose.orientation),
                    "v": float(msg.twist.twist.linear.x),
                }))
            elif topic == "/mpc/reference_path":
                pts = path_points(msg)
                d[topic].append((t, pts))
            elif topic in ("/mpc_status", "/terminal/mode"):
                d[topic].append((t, str(msg.data)))
            elif topic in ("/mpc/status_val", "/slosh/speed_governor_active",
                           "/terminal/recovery_latched"):
                d[topic].append((t, int(msg.data)))
            elif topic == "/risk_scheduler/fallback_active":
                d[topic].append((t, bool(msg.data)))
            else:
                d[topic].append((t, float(msg.data)))

    d["_duration"] = bag.get_end_time() - t0
    return dict(d)


# ── Check A: u_prev desync ───────────────────────────────────────────────────

def check_A_uprev(data: Dict) -> Dict:
    """
    For each solve failure (status_val=0), measure |Δcmd_v| and |Δcmd_ω|
    to the next cmd_vel sample. Compare distribution to baseline (non-failure).
    """
    status = data.get("/mpc/status_val", [])
    cmds = data.get("/cmd_vel", [])
    if len(status) < 2 or len(cmds) < 2:
        return {"ok": None, "note": "insufficient data"}

    failure_delta_v, failure_delta_w = [], []
    baseline_delta_v, baseline_delta_w = [], []

    cmd_idx = 0
    for i in range(len(status) - 1):
        t_fail, sv = status[i]
        # find cmd just before and just after this status sample
        cmd_idx, cmd_cur = nearest_value(cmds, t_fail, cmd_idx)
        if cmd_cur is None:
            continue
        # next cmd
        if cmd_idx + 1 >= len(cmds):
            continue
        _, cmd_next = cmds[cmd_idx + 1]
        dv = abs(cmd_next[0] - cmd_cur[0])
        dw = abs(cmd_next[1] - cmd_cur[1])

        if sv == 0:  # failure
            failure_delta_v.append(dv)
            failure_delta_w.append(dw)
        else:
            baseline_delta_v.append(dv)
            baseline_delta_w.append(dw)

    n_fail = len(failure_delta_v)
    p95_fail_v = percentile(failure_delta_v, 95)
    p95_base_v = percentile(baseline_delta_v, 95)
    p95_fail_w = percentile(failure_delta_w, 95)
    p95_base_w = percentile(baseline_delta_w, 95)

    # Bug indicator: failure Δcmd p95 > 3× baseline p95
    v_ratio = p95_fail_v / max(1e-6, p95_base_v)
    w_ratio = p95_fail_w / max(1e-6, p95_base_w)
    suspicious = (v_ratio > 3.0 or w_ratio > 3.0) and n_fail > 0

    return {
        "n_failures": n_fail,
        "p95_fail_dv": p95_fail_v,
        "p95_base_dv": p95_base_v,
        "p95_fail_dw": p95_fail_w,
        "p95_base_dw": p95_base_w,
        "v_ratio": v_ratio,
        "w_ratio": w_ratio,
        "suspicious": suspicious,
    }


# ── Check B: v_plan/v_exec cascade ──────────────────────────────────────────

def check_B_cascade(data: Dict) -> Dict:
    """
    Detect if odom velocity drops correlate with subsequent v_des_eff drops
    (would indicate v_plan/v_exec coupling regression).
    """
    odom = data.get("/odom", [])
    vdes = data.get("/slosh/v_des_eff", [])
    if len(odom) < 10 or len(vdes) < 10:
        return {"ok": None, "note": "insufficient data"}

    # Find events where odom_v drops by > 0.15 m/s within 0.3s
    # Then check if v_des_eff also drops within the next 0.5s
    cascade_events = 0
    total_drop_events = 0
    vdes_idx = 0

    for i in range(1, len(odom)):
        t_prev, s_prev = odom[i - 1]
        t_cur, s_cur = odom[i]
        if t_cur - t_prev > 0.3:
            continue
        dv = s_prev["v"] - s_cur["v"]
        if dv < 0.15:
            continue
        total_drop_events += 1
        # check v_des_eff in next 0.5s
        vdes_idx, vdes_at_drop = nearest_value(vdes, t_cur, vdes_idx)
        if vdes_at_drop is None:
            continue
        # look ahead 0.5s
        vdes_future = [v for t, v in vdes if t_cur <= t <= t_cur + 0.5]
        if not vdes_future:
            continue
        vdes_min_future = min(vdes_future)
        if vdes_at_drop - vdes_min_future > 0.1:  # v_des_eff also dropped ≥0.1m/s
            cascade_events += 1

    cascade_ratio = cascade_events / max(1, total_drop_events)
    suspicious = cascade_ratio > 0.3 and total_drop_events >= 3

    return {
        "total_odom_drops": total_drop_events,
        "cascade_events": cascade_events,
        "cascade_ratio": cascade_ratio,
        "suspicious": suspicious,
    }


# ── Check C: governor + scheduler overlap ───────────────────────────────────

def check_C_overlap(data: Dict) -> Dict:
    """
    Fraction of time both speed_governor_active=1 and rho_k > 0.1 simultaneously.
    Only meaningful for PROP2 bags (others have rho_k=0 by design).
    """
    gov = data.get("/slosh/speed_governor_active", [])
    rho = data.get("/risk_scheduler/rho_k", [])
    if not gov or not rho:
        return {"ok": None, "note": "no governor/scheduler data"}

    rho_nonzero = sum(1 for _, v in rho if v > 0.1)
    scheduler_active = rho_nonzero > 5  # bag has meaningful scheduler activity

    if not scheduler_active:
        return {
            "scheduler_active": False,
            "note": "scheduler not active in this bag (expected for non-PROP2)",
        }

    gov_idx = 0
    both_active = 0
    gov_only = 0
    sched_only = 0
    neither = 0

    for t, rho_val in rho:
        gov_idx, gov_val = nearest_value(gov, t, gov_idx)
        if gov_val is None:
            continue
        g = gov_val > 0
        s = rho_val > 0.1
        if g and s:
            both_active += 1
        elif g:
            gov_only += 1
        elif s:
            sched_only += 1
        else:
            neither += 1

    total = both_active + gov_only + sched_only + neither
    overlap_ratio = both_active / max(1, total)
    suspicious = overlap_ratio > 0.3

    return {
        "scheduler_active": True,
        "both_active_frac": overlap_ratio,
        "gov_only_frac": gov_only / max(1, total),
        "sched_only_frac": sched_only / max(1, total),
        "neither_frac": neither / max(1, total),
        "suspicious": suspicious,
    }


# ── Check D: SETTLING duration ───────────────────────────────────────────────

def check_D_settling(data: Dict) -> Dict:
    """
    Extract each TRACKING→SETTLING→REACHED transition.
    Returns list of (t_settling_start, settling_duration_s, ended_by_timeout).
    """
    modes = data.get("/mpc_status", [])
    if len(modes) < 3:
        return {"segments": [], "note": "insufficient data"}

    segments = []
    i = 0
    while i < len(modes):
        t_i, m_i = modes[i]
        if m_i != "SETTLING":
            i += 1
            continue
        # found start of SETTLING
        t_start = t_i
        j = i + 1
        while j < len(modes) and modes[j][1] == "SETTLING":
            j += 1
        t_end = modes[j][0] if j < len(modes) else modes[-1][0]
        duration = t_end - t_start
        next_mode = modes[j][1] if j < len(modes) else "END"
        segments.append({
            "t_start": t_start,
            "duration": duration,
            "next_mode": next_mode,
        })
        i = j

    durations = [s["duration"] for s in segments]
    return {
        "segments": segments,
        "n_settling": len(segments),
        "mean_s": statistics.mean(durations) if durations else None,
        "min_s": min(durations) if durations else None,
        "max_s": max(durations) if durations else None,
    }


# ── Check E: tracking_reentry ramp ──────────────────────────────────────────

def check_E_reentry(data: Dict) -> Dict:
    """
    For each IDLE→TRACKING transition, extract cmd_vel linear.x for
    the first 15 frames and the steady-state mean (frames 40-80).
    """
    modes = data.get("/mpc_status", [])
    cmds = data.get("/cmd_vel", [])
    if len(modes) < 3 or len(cmds) < 20:
        return {"events": [], "note": "insufficient data"}

    # find IDLE→TRACKING transitions
    transitions = []
    for i in range(1, len(modes)):
        if modes[i - 1][1] != "TRACKING" and modes[i][1] == "TRACKING":
            transitions.append(modes[i][0])

    events = []
    for t_entry in transitions:
        # collect cmd_vel from t_entry onward (up to 3s)
        ramp_vs = [v for t, (v, _) in cmds if t_entry <= t <= t_entry + 3.0]
        steady_vs = [v for t, (v, _) in cmds if t_entry + 2.0 <= t <= t_entry + 6.0]
        if len(ramp_vs) < 5:
            continue
        first10_mean = statistics.mean(ramp_vs[:10])
        steady_mean = statistics.mean(steady_vs) if steady_vs else None
        suppression = (1.0 - first10_mean / steady_mean) if steady_mean else None
        events.append({
            "t_entry": t_entry,
            "first10_mean": first10_mean,
            "steady_mean": steady_mean,
            "suppression": suppression,  # fraction speed is suppressed vs steady
        })

    suppressions = [e["suppression"] for e in events if e["suppression"] is not None]
    return {
        "n_entries": len(transitions),
        "events": events,
        "mean_suppression": statistics.mean(suppressions) if suppressions else None,
        "suspicious": any(s > 0.25 for s in suppressions) if suppressions else False,
    }


# ── Check F: failure clustering vs kappa ────────────────────────────────────

def check_F_failure_kappa(data: Dict) -> Dict:
    """
    For each solve failure, record the max_kappa of the current reference path.
    If failures cluster at high kappa, it's geometry-driven (not a bug).
    If failures are random / at low kappa, it may be u_prev desync or other bug.
    """
    status = data.get("/mpc/status_val", [])
    refpath = data.get("/mpc/reference_path", [])
    if not status or not refpath:
        return {"ok": None, "note": "insufficient data"}

    ref_idx = 0
    failure_kappas = []
    success_kappas = []

    for t, sv in status:
        ref_idx, pts = nearest_value(refpath, t, ref_idx)
        if pts is None:
            continue
        kappa = max_kappa_of_path(pts)
        if sv == 0:
            failure_kappas.append(kappa)
        else:
            success_kappas.append(kappa)

    n_fail = len(failure_kappas)
    if n_fail == 0:
        return {
            "n_failures": 0,
            "note": "no failures",
            "suspicious": False,
        }

    median_fail_k = statistics.median(failure_kappas)
    median_succ_k = statistics.median(success_kappas) if success_kappas else 0.0
    kappa_ratio = median_fail_k / max(1e-6, median_succ_k)

    # If failures have higher kappa than successes → geometry-driven (expected)
    # If failures at similar/lower kappa → suspicious (bug)
    geometry_driven = kappa_ratio > 1.5 and median_fail_k > 0.3

    return {
        "n_failures": n_fail,
        "median_fail_kappa": median_fail_k,
        "median_succ_kappa": median_succ_k,
        "kappa_ratio": kappa_ratio,
        "geometry_driven": geometry_driven,
        "suspicious": not geometry_driven and n_fail > 2,
        "failure_kappas": failure_kappas,
        "success_kappas": success_kappas,
    }


# ── dist/heading frame validation ────────────────────────────────────────────

def check_dist_heading_frame(data: Dict) -> Dict:
    """
    Print a few sample odom (x,y) vs reference_path first/last point.
    If they're in very different ranges, it's a frame mismatch.
    """
    odom = data.get("/odom", [])
    refpath = data.get("/mpc/reference_path", [])
    if not odom or not refpath:
        return {}

    # sample at bag mid-point
    mid = len(odom) // 2
    t_mid, s_mid = odom[mid]
    ref_idx, pts = nearest_value(refpath, t_mid)
    if not pts:
        return {}

    odom_xy = (s_mid["x"], s_mid["y"])
    ref_first = pts[0]
    ref_last = pts[-1]
    dist_to_first = math.hypot(odom_xy[0] - ref_first[0], odom_xy[1] - ref_first[1])
    dist_to_last = math.hypot(odom_xy[0] - ref_last[0], odom_xy[1] - ref_last[1])
    dist_to_nearest = min(
        math.hypot(odom_xy[0] - p[0], odom_xy[1] - p[1]) for p in pts
    )

    # odom range
    xs = [s["x"] for _, s in odom]
    ys = [s["y"] for _, s in odom]
    # ref path range (all published messages)
    ref_xs = [p[0] for _, pts_ in refpath for p in pts_]
    ref_ys = [p[1] for _, pts_ in refpath for p in pts_]

    odom_range = (min(xs), max(xs), min(ys), max(ys))
    ref_range = (min(ref_xs) if ref_xs else 0, max(ref_xs) if ref_xs else 0,
                 min(ref_ys) if ref_ys else 0, max(ref_ys) if ref_ys else 0)

    # heuristic: if odom bbox and ref bbox barely overlap, frame mismatch
    x_overlap = max(0, min(odom_range[1], ref_range[1]) - max(odom_range[0], ref_range[0]))
    y_overlap = max(0, min(odom_range[3], ref_range[3]) - max(odom_range[2], ref_range[2]))
    frame_ok = x_overlap > 0.5 and y_overlap > 0.5

    return {
        "odom_mid": odom_xy,
        "ref_first": ref_first,
        "ref_last": ref_last,
        "dist_to_nearest_ref": dist_to_nearest,
        "dist_to_first_ref": dist_to_first,
        "odom_range": odom_range,
        "ref_range": ref_range,
        "frame_ok": frame_ok,
    }


# ── State machine summary ────────────────────────────────────────────────────

def state_machine_summary(data: Dict) -> Dict:
    modes = data.get("/mpc_status", [])
    if not modes:
        return {}
    duration = data.get("_duration", 0.0)
    counts: Dict[str, float] = defaultdict(float)
    for i in range(len(modes) - 1):
        dt = modes[i + 1][0] - modes[i][0]
        counts[modes[i][1]] += dt
    counts[modes[-1][1]] += 0.05  # last sample ~1 dt
    return {m: counts[m] / max(1e-6, duration) for m in counts}


# ── Plotting ─────────────────────────────────────────────────────────────────

def plot_state_timeline(label: str, data: Dict, out_path: Path) -> None:
    if not _HAS_MPL:
        return
    modes = data.get("/mpc_status", [])
    status = data.get("/mpc/status_val", [])
    vdes = data.get("/slosh/v_des_eff", [])
    odom = data.get("/odom", [])
    if not modes:
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 7), sharex=True)
    fig.patch.set_facecolor("#1a1a1a")
    for ax in axes:
        ax.set_facecolor("#1a1a1a")
        ax.tick_params(colors="white")
        for sp in ax.spines.values():
            sp.set_color("#444444")

    color_map = {
        "IDLE": "#555555", "TRACKING": "#22aa44", "SETTLING": "#ffaa00",
        "REACHED": "#4488ff",
    }

    # Panel 1: mode bands
    ax = axes[0]
    for i in range(len(modes) - 1):
        t0, m = modes[i]
        t1 = modes[i + 1][0]
        c = color_map.get(m, "#888888")
        ax.axvspan(t0, t1, color=c, alpha=0.4)
    ax.set_ylabel("Mode", color="white")
    ax.set_yticks([])
    patches = [mpatches.Patch(color=c, label=m) for m, c in color_map.items()]
    ax.legend(handles=patches, loc="upper right", fontsize=7,
              facecolor="#333333")
    ax.set_title(f"{label} – state machine & control signals", color="white")

    # Panel 2: solve status + solve_ms
    ax2 = axes[1]
    fail_ts = [t for t, v in status if v == 0]
    ax2.scatter(fail_ts, [1] * len(fail_ts), color="red", s=20, zorder=5,
                label=f"fail (n={len(fail_ts)})")
    solve_ms_data = data.get("/mpc/solve_ms", [])
    if solve_ms_data:
        ts_ms = [t for t, _ in solve_ms_data]
        vs_ms = [v for _, v in solve_ms_data]
        ax2_r = ax2.twinx()
        ax2_r.plot(ts_ms, vs_ms, color="#88aaff", linewidth=0.8, alpha=0.7, label="solve_ms")
        ax2_r.set_ylabel("solve (ms)", color="#88aaff")
        ax2_r.tick_params(colors="#88aaff")
    ax2.set_ylabel("failures", color="white")
    ax2.set_ylim(0.5, 1.5)
    ax2.legend(loc="upper left", fontsize=7, facecolor="#333333")

    # Panel 3: v_des_eff vs odom_v
    ax3 = axes[2]
    if vdes:
        ax3.plot([t for t, _ in vdes], [v for _, v in vdes],
                 color="#ffaa00", linewidth=1.2, label="v_des_eff")
    if odom:
        ax3.plot([t for t, s in odom], [s["v"] for _, s in odom],
                 color="#44ff88", linewidth=0.8, alpha=0.7, label="odom_v")
    gov = data.get("/slosh/speed_governor_active", [])
    for t, gv in gov:
        if gv > 0:
            ax3.axvline(t, color="#ff4444", alpha=0.15, linewidth=0.5)
    ax3.set_ylabel("speed (m/s)", color="white")
    ax3.set_xlabel("time (s)", color="white")
    ax3.legend(loc="upper right", fontsize=7, facecolor="#333333")

    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor="#1a1a1a")
    plt.close()


def plot_failure_kappa(label: str, result_F: Dict, out_path: Path) -> None:
    if not _HAS_MPL:
        return
    fail_k = result_F.get("failure_kappas", [])
    succ_k = result_F.get("success_kappas", [])
    if not fail_k and not succ_k:
        return

    fig, ax = plt.subplots(figsize=(7, 4))
    fig.patch.set_facecolor("#1a1a1a")
    ax.set_facecolor("#1a1a1a")
    ax.tick_params(colors="white")
    for sp in ax.spines.values():
        sp.set_color("#444444")

    bins = [i * 0.2 for i in range(20)]
    if succ_k:
        ax.hist(succ_k, bins=bins, color="#44aa44", alpha=0.6, label=f"success (n={len(succ_k)})")
    if fail_k:
        ax.hist(fail_k, bins=bins, color="#ff4444", alpha=0.8, label=f"failure (n={len(fail_k)})")
    ax.set_xlabel("max_kappa of reference_path (1/m)", color="white")
    ax.set_ylabel("count", color="white")
    ax.set_title(f"{label} – failure vs kappa", color="white")
    ax.legend(facecolor="#333333", fontsize=8)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor="#1a1a1a")
    plt.close()


def plot_delta_cmd_distributions(all_results: List[Tuple[str, Dict]], out_path: Path) -> None:
    if not _HAS_MPL:
        return
    labels = [lbl for lbl, _ in all_results]
    fail_dv = [r.get("p95_fail_dv", float("nan")) for _, r in all_results]
    base_dv = [r.get("p95_base_dv", float("nan")) for _, r in all_results]

    x = range(len(labels))
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.patch.set_facecolor("#1a1a1a")
    ax.set_facecolor("#1a1a1a")
    ax.tick_params(colors="white")
    for sp in ax.spines.values():
        sp.set_color("#444444")

    w = 0.35
    bars_fail = ax.bar([i - w / 2 for i in x], fail_dv, width=w,
                       color="#ff6666", label="|Δcmd_v| p95 after failure")
    bars_base = ax.bar([i + w / 2 for i in x], base_dv, width=w,
                       color="#66bb66", label="|Δcmd_v| p95 baseline")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=30, ha="right", color="white")
    ax.set_ylabel("|Δcmd_v| (m/s)", color="white")
    ax.set_title("Check A: u_prev desync – |Δcmd_v| after failure vs baseline", color="white")
    ax.legend(facecolor="#333333", fontsize=8)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor="#1a1a1a")
    plt.close()


def plot_settling_durations(all_results: List[Tuple[str, Dict]], out_path: Path) -> None:
    if not _HAS_MPL:
        return
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.patch.set_facecolor("#1a1a1a")
    ax.set_facecolor("#1a1a1a")
    ax.tick_params(colors="white")
    for sp in ax.spines.values():
        sp.set_color("#444444")

    for i, (lbl, res) in enumerate(all_results):
        segs = res.get("segments", [])
        for seg in segs:
            ax.barh(i, seg["duration"], left=seg["t_start"], height=0.6,
                    color="#ffaa22", alpha=0.8, edgecolor="#ffcc44")
        ax.text(0.2, i, lbl, va="center", color="white", fontsize=7)

    ax.set_xlabel("time (s)", color="white")
    ax.set_title("Check D: SETTLING segments per bag", color="white")
    ax.set_yticks(range(len(all_results)))
    ax.set_yticklabels([lbl for lbl, _ in all_results], color="white", fontsize=7)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor="#1a1a1a")
    plt.close()


# ── Summary CSV / print ──────────────────────────────────────────────────────

SUMMARY_FIELDS = [
    "label",
    "n_failures", "success_ratio",
    "A_suspicious", "A_fail_dv_p95", "A_base_dv_p95",
    "B_cascade_ratio", "B_suspicious",
    "C_overlap_frac", "C_scheduler_active",
    "D_n_settling", "D_mean_s", "D_min_s",
    "E_mean_suppression", "E_suspicious",
    "F_n_failures", "F_kappa_ratio", "F_geometry_driven",
    "frame_ok", "dist_to_nearest_ref",
]


def row_to_csv(label, n_fail, n_total, A, B, C, D, E, F, frame) -> Dict:
    sr = (n_total - n_fail) / max(1, n_total)
    return {
        "label": label,
        "n_failures": n_fail,
        "success_ratio": f"{sr:.3f}",
        "A_suspicious": A.get("suspicious", "?"),
        "A_fail_dv_p95": _fmt(A.get("p95_fail_dv")),
        "A_base_dv_p95": _fmt(A.get("p95_base_dv")),
        "B_cascade_ratio": _fmt(B.get("cascade_ratio")),
        "B_suspicious": B.get("suspicious", "?"),
        "C_overlap_frac": _fmt(C.get("both_active_frac")),
        "C_scheduler_active": C.get("scheduler_active", False),
        "D_n_settling": D.get("n_settling", 0),
        "D_mean_s": _fmt(D.get("mean_s")),
        "D_min_s": _fmt(D.get("min_s")),
        "E_mean_suppression": _fmt(E.get("mean_suppression")),
        "E_suspicious": E.get("suspicious", "?"),
        "F_n_failures": F.get("n_failures", 0),
        "F_kappa_ratio": _fmt(F.get("kappa_ratio")),
        "F_geometry_driven": F.get("geometry_driven", "?"),
        "frame_ok": frame.get("frame_ok", "?"),
        "dist_to_nearest_ref": _fmt(frame.get("dist_to_nearest_ref")),
    }


def print_summary_table(rows: List[Dict]) -> None:
    cols = ["label", "success_ratio", "A_suspicious", "B_suspicious",
            "C_overlap_frac", "D_mean_s", "E_suspicious", "F_geometry_driven",
            "frame_ok", "dist_to_nearest_ref"]
    header = "  ".join(f"{c:20s}" for c in cols)
    print("\n" + "=" * len(header))
    print("MPC BUG ANALYSIS SUMMARY")
    print("=" * len(header))
    print(header)
    print("-" * len(header))
    for row in rows:
        line = "  ".join(f"{str(row.get(c, '-')):20s}" for c in cols)
        print(line)
    print("=" * len(header))


# ── Main ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out-dir", default="/tmp/mpc_bug_analysis",
                   help="Output directory for plots and CSV")
    p.add_argument("--bags", nargs="+", default=None,
                   help="Override bag list: label:path pairs, e.g. NOM:/data/a/.../nom.bag")
    p.add_argument("--skip-plots", action="store_true",
                   help="Skip generating plots (faster for quick table check)")
    return p.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    bag_list = BAG_LIST
    if args.bags:
        bag_list = []
        for item in args.bags:
            lbl, path = item.split(":", 1)
            bag_list.append((lbl, Path(path)))

    summary_rows = []
    check_A_all, check_D_all = [], []

    for label, bag_path in bag_list:
        if not bag_path.exists():
            print(f"[SKIP] {label}: {bag_path} not found")
            continue
        print(f"\n[{label}] loading {bag_path.name} ...")
        data = load_bag(bag_path)
        if not data:
            continue

        status_vals = [v for _, v in data.get("/mpc/status_val", [])]
        n_total = len(status_vals)
        n_fail = sum(1 for v in status_vals if v == 0)
        print(f"  status: n={n_total}, failures={n_fail}, "
              f"success_ratio={((n_total-n_fail)/max(1,n_total)):.3f}")

        A = check_A_uprev(data)
        B = check_B_cascade(data)
        C = check_C_overlap(data)
        D = check_D_settling(data)
        E = check_E_reentry(data)
        F = check_F_failure_kappa(data)
        frame = check_dist_heading_frame(data)
        sm = state_machine_summary(data)

        print(f"  [A] u_prev: n_fail={A.get('n_failures',0)}, "
              f"fail_dv_p95={_fmt(A.get('p95_fail_dv'))} vs base={_fmt(A.get('p95_base_dv'))} "
              f"{'[SUSPICIOUS]' if A.get('suspicious') else '[ok]'}")
        print(f"  [B] cascade: ratio={_fmt(B.get('cascade_ratio'))} "
              f"({B.get('cascade_events',0)}/{B.get('total_odom_drops',0)} drops) "
              f"{'[SUSPICIOUS]' if B.get('suspicious') else '[ok]'}")
        if C.get("scheduler_active"):
            print(f"  [C] gov+sched overlap: {_fmt(C.get('both_active_frac'))} "
                  f"{'[SUSPICIOUS]' if C.get('suspicious') else '[ok]'}")
        else:
            print(f"  [C] scheduler not active ({C.get('note','')})")
        print(f"  [D] settling: n={D.get('n_settling',0)}, "
              f"mean={_fmt(D.get('mean_s'))}s, min={_fmt(D.get('min_s'))}s")
        print(f"  [E] reentry ramp: mean_suppression={_fmt(E.get('mean_suppression'))} "
              f"{'[SUSPICIOUS]' if E.get('suspicious') else '[ok]'}")
        print(f"  [F] failure kappa: n_fail={F.get('n_failures',0)}, "
              f"kappa_ratio={_fmt(F.get('kappa_ratio'))}, "
              f"geometry_driven={F.get('geometry_driven','?')}")
        print(f"  [frame] frame_ok={frame.get('frame_ok','?')}, "
              f"dist_to_nearest_ref={_fmt(frame.get('dist_to_nearest_ref'))}")
        print(f"  [modes] {sm}")

        if not args.skip_plots:
            plot_state_timeline(label, data, out_dir / f"{label}_timeline.png")
            if F.get("n_failures", 0) > 0:
                plot_failure_kappa(label, F, out_dir / f"{label}_failure_kappa.png")

        row = row_to_csv(label, n_fail, n_total, A, B, C, D, E, F, frame)
        summary_rows.append(row)
        check_A_all.append((label, A))
        check_D_all.append((label, D))

    # Cross-bag plots
    if not args.skip_plots and _HAS_MPL:
        plot_delta_cmd_distributions(check_A_all, out_dir / "check_A_delta_cmd.png")
        plot_settling_durations(check_D_all, out_dir / "check_D_settling.png")

    # Summary CSV
    csv_path = out_dir / "mpc_bug_summary.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(summary_rows)
    print(f"\n[OK] summary CSV: {csv_path}")

    print_summary_table(summary_rows)

    # Quick bug verdict
    print("\nBUG VERDICT:")
    for row in summary_rows:
        flags = []
        if str(row.get("A_suspicious")) == "True":
            flags.append("A:u_prev_desync")
        if str(row.get("B_suspicious")) == "True":
            flags.append("B:v_cascade")
        if row.get("C_overlap_frac") not in ("-", None) and float(row["C_overlap_frac"]) > 0.3:
            flags.append("C:gov+sched_overlap")
        if str(row.get("E_suspicious")) == "True":
            flags.append("E:reentry_ramp")
        if str(row.get("F_geometry_driven")) == "False" and int(row.get("F_n_failures", 0)) > 2:
            flags.append("F:random_failure")
        if str(row.get("frame_ok")) == "False":
            flags.append("FRAME_MISMATCH")
        verdict = " | ".join(flags) if flags else "clean"
        print(f"  {row['label']:20s}: {verdict}")

    if _HAS_MPL:
        print(f"\n[OK] plots saved to {out_dir}/")


if __name__ == "__main__":
    sys.exit(main() or 0)
