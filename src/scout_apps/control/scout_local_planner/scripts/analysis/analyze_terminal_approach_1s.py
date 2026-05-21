#!/usr/bin/env python3
"""Terminal approach diagnostics for fixed-path slosh MPC bags.

This script focuses only on the last N seconds before the first terminal/capture
transition.  It separates terminal problems from the main paper window.
"""

import argparse
import csv
import math
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

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


IDLE_TERMINAL_MODES = {"", "NONE", "IDLE"}
RECOVERY_MODES = {"ALIGN_TO_POINT", "APPROACH_POINT", "ALIGN_FINAL_YAW"}
CAPTURE_MODES = {"CAPTURE_STOP", "CAPTURE_BRAKE", "GOAL_STOP_PENDING", "TERMINAL_MPC_STOP"}
GOAL_INFO_FIELDS = (
    "dx",
    "dy",
    "dist",
    "bearing",
    "goal_yaw_err",
    "has_goal_yaw",
    "position_reached",
    "pose_reached",
)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bags", nargs="*", help="Input bags. If omitted, use --bag-dir/--glob.")
    p.add_argument("--bag-dir", default="")
    p.add_argument("--glob", default="*.bag")
    p.add_argument("--exclude-static", action="store_true")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--window", type=float, default=1.0, help="Seconds before first terminal. Default: 1.0")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--ref-v-high", type=float, default=0.12)
    p.add_argument("--cmd-v-low", type=float, default=0.08)
    p.add_argument("--odom-v-high", type=float, default=0.18)
    p.add_argument("--ax-high", type=float, default=0.8)
    p.add_argument("--jerk-high", type=float, default=8.0)
    return p.parse_args()


def finite(value, default=float("nan")):
    try:
        v = float(value)
        return v if math.isfinite(v) else default
    except Exception:
        return default


def condition_from_name(name):
    if "_G_" in name or "_G_slosh_axjerk_preview" in name:
        return "G"
    if "_F_" in name or "_F_slosh_axjerk" in name:
        return "F"
    if "_E_" in name or "_E_axjerk" in name:
        return "E"
    if "_D_" in name or "_D_run" in name:
        return "D"
    if "_C_" in name or "_C_run" in name:
        return "C"
    m = re.search(r"_Q(\d+)_", name)
    return f"Q{m.group(1)}" if m else "UNK"


def collect_bags(args):
    if args.bags:
        bags = [Path(p) for p in args.bags]
    elif args.bag_dir:
        bags = sorted(Path(args.bag_dir).glob(args.glob))
    else:
        raise SystemExit("provide bags or --bag-dir")
    if args.exclude_static:
        bags = [p for p in bags if "static" not in p.name.lower()]
    return bags


def scalar(msg):
    return finite(getattr(msg, "data", float("nan")))


def add(series, key, t, value):
    series[key][0].append(float(t))
    series[key][1].append(finite(value))


def first_tracking(status):
    for t, s in status:
        if str(s).upper() == "TRACKING":
            return t
    return float("nan")


def first_terminal(modes):
    for t, mode in modes:
        if str(mode).upper() not in IDLE_TERMINAL_MODES:
            return t, str(mode)
    return float("nan"), ""


def arrays(series):
    return {k: (np.asarray(t, dtype=float), np.asarray(v, dtype=float)) for k, (t, v) in series.items()}


def derivative(ts, vals):
    if len(ts) < 3:
        return np.full_like(vals, np.nan, dtype=float)
    return np.gradient(vals, ts, edge_order=1)


def nearest(ts, vals, t):
    if len(ts) == 0 or not math.isfinite(t):
        return float("nan")
    idx = int(np.searchsorted(ts, t))
    if idx <= 0:
        return float(vals[0])
    if idx >= len(ts):
        return float(vals[-1])
    return float(vals[idx] if abs(ts[idx] - t) < abs(ts[idx - 1] - t) else vals[idx - 1])


def step_at(samples, t, default=None):
    if not samples or not math.isfinite(t):
        return default
    lo, hi = 0, len(samples)
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    idx = lo - 1
    return samples[idx][1] if idx >= 0 else default


def window_vals(ts, vals, t0, t1):
    if len(ts) == 0:
        return np.asarray([], dtype=float)
    mask = (ts >= t0) & (ts <= t1) & np.isfinite(vals)
    return vals[mask]


def absmax(ts, vals, t0, t1):
    arr = window_vals(ts, vals, t0, t1)
    return float(np.max(np.abs(arr))) if arr.size else float("nan")


def p95_abs(ts, vals, t0, t1):
    arr = window_vals(ts, vals, t0, t1)
    return float(np.percentile(np.abs(arr), 95)) if arr.size else float("nan")


def min_val(ts, vals, t0, t1):
    arr = window_vals(ts, vals, t0, t1)
    return float(np.min(arr)) if arr.size else float("nan")


def max_val(ts, vals, t0, t1):
    arr = window_vals(ts, vals, t0, t1)
    return float(np.max(arr)) if arr.size else float("nan")


def peak(ts, vals, t0, t1):
    arr = window_vals(ts, vals, t0, t1)
    if not arr.size:
        return float("nan"), float("nan"), float("nan")
    mask = (ts >= t0) & (ts <= t1) & np.isfinite(vals)
    wts = ts[mask]
    idx = int(np.argmax(np.abs(arr)))
    return float(wts[idx]), float(abs(arr[idx])), float(np.percentile(np.abs(arr), 95))


def classify_horizon_k(k):
    if not math.isfinite(k):
        return "UNKNOWN"
    return "CURRENT_K0" if k <= 0.5 else "FUTURE_K_GT_0"


def read_bag(path):
    series = defaultdict(lambda: ([], []))
    modes = []
    status = []
    recovery = []
    goal_info = []
    start_abs = 0.0
    end_rel = 0.0
    topics = [
        "/mpc_status",
        "/terminal/mode",
        "/terminal/recovery_latched",
        "/terminal/goal_info",
        "/terminal/v_envelope",
        "/terminal/envelope_active",
        "/terminal/phase_active",
        "/terminal/cmd_v_pre_clamp",
        "/terminal/cmd_v_post_clamp",
        "/reference/v_ref",
        "/reference/v_ref_horizon",
        "/reference/v_des_raw",
        "/reference/v_des_target",
        "/reference/v_des_eff",
        "/reference/v_des_rate_limited",
        "/reference/implied_ax",
        "/reference/implied_ay",
        "/reference/implied_jerk",
        "/cmd_vel",
        "/odom",
        "/slosh/height",
        "/mpc/slosh_horizon_summary",
        "/mpc/cost_breakdown",
    ]
    with rosbag.Bag(str(path), "r") as bag:
        start_abs = bag.get_start_time()
        end_rel = bag.get_end_time() - start_abs
        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec() - start_abs
            if topic == "/mpc_status":
                status.append((t, str(getattr(msg, "data", ""))))
            elif topic == "/terminal/mode":
                modes.append((t, str(getattr(msg, "data", ""))))
            elif topic == "/terminal/recovery_latched":
                recovery.append((t, bool(getattr(msg, "data", False))))
            elif topic == "/terminal/goal_info":
                vals = list(getattr(msg, "data", []))
                if len(vals) >= len(GOAL_INFO_FIELDS):
                    goal_info.append((t, dict(zip(GOAL_INFO_FIELDS, [finite(v) for v in vals[:8]]))))
            elif topic == "/cmd_vel":
                add(series, "cmd_v", t, msg.linear.x)
            elif topic == "/odom":
                add(series, "odom_v", t, msg.twist.twist.linear.x)
            elif topic == "/slosh/height":
                add(series, "slosh_height_mm", t, abs(scalar(msg)) * 1000.0)
            elif topic == "/reference/v_ref":
                add(series, "v_ref", t, scalar(msg))
            elif topic == "/reference/v_ref_horizon":
                vals = [finite(v) for v in getattr(msg, "data", [])]
                if vals:
                    add(series, "v_ref_horizon_first", t, vals[0])
                    add(series, "v_ref_horizon_last", t, vals[-1])
                    add(series, "v_ref_horizon_max", t, max(v for v in vals if math.isfinite(v)))
            elif topic in (
                "/reference/v_des_raw",
                "/reference/v_des_target",
                "/reference/v_des_eff",
                "/reference/v_des_rate_limited",
                "/reference/implied_ax",
                "/reference/implied_ay",
                "/reference/implied_jerk",
                "/terminal/v_envelope",
                "/terminal/cmd_v_pre_clamp",
                "/terminal/cmd_v_post_clamp",
            ):
                add(series, topic.strip("/").replace("/", "_"), t, scalar(msg))
            elif topic == "/terminal/envelope_active":
                add(series, "terminal_envelope_active", t, 1.0 if getattr(msg, "data", False) else 0.0)
            elif topic == "/terminal/phase_active":
                add(series, "terminal_phase_active", t, 1.0 if getattr(msg, "data", False) else 0.0)
            elif topic == "/mpc/slosh_horizon_summary" and len(getattr(msg, "data", [])) >= 13:
                data = list(msg.data)
                add(series, "horizon_hmax_mm", t, data[5])
                add(series, "horizon_kmax", t, data[6])
                add(series, "horizon_ax_p95", t, data[9])
                add(series, "horizon_ay_p95", t, data[10])
            elif topic == "/mpc/cost_breakdown" and len(getattr(msg, "data", [])) >= 20:
                data = list(msg.data)
                add(series, "pct_v", t, data[13])
                add(series, "pct_slosh", t, data[19])

    arr = arrays(series)
    for key in ("cmd_v", "odom_v"):
        ts, vals = arr.get(key, (np.asarray([]), np.asarray([])))
        arr[key.replace("_v", "_ax")] = (ts, derivative(ts, vals))
    for key in ("cmd_ax", "odom_ax"):
        ts, vals = arr.get(key, (np.asarray([]), np.asarray([])))
        arr[key.replace("_ax", "_jerk")] = (ts, derivative(ts, vals))
    return {
        "series": arr,
        "status": status,
        "modes": modes,
        "recovery": recovery,
        "goal_info": goal_info,
        "tracking_start": first_tracking(status),
        "terminal": first_terminal(modes),
        "end_rel": end_rel,
    }


def problem_flags(row, args):
    flags = []
    if row["reference_speed_problem"]:
        flags.append("参考速度问题")
    if row["execution_problem"]:
        flags.append("执行问题")
    if row["limit_problem"]:
        flags.append("限幅问题")
    if row["state_machine_problem"]:
        flags.append("状态机问题")
    if row["model_peak_problem"]:
        flags.append("模型问题")
    return "；".join(flags) if flags else "未发现明显问题"


def analyze_bag(path, args):
    data = read_bag(path)
    series = data["series"]
    terminal_t, first_mode = data["terminal"]
    if not math.isfinite(terminal_t):
        terminal_t = data["end_rel"]
        first_mode = "NO_TERMINAL"
    t1 = terminal_t
    t0 = max(0.0, t1 - args.window)
    condition = condition_from_name(path.name)

    row = {
        "bag": path.name,
        "condition": condition,
        "window_start_s": t0,
        "terminal_time_s": t1,
        "window_duration_s": t1 - t0,
        "first_terminal_mode": first_mode,
        "mode_counts": ";".join(f"{k}:{v}" for k, v in sorted(Counter(m for _, m in data["modes"]).items())),
        "recovery_triggered": int(any(bool(v) for _, v in data["recovery"])),
    }

    info = step_at(data["goal_info"], terminal_t, {})
    for key in GOAL_INFO_FIELDS:
        row[f"goal_{key}_at_terminal"] = finite(info.get(key)) if isinstance(info, dict) else float("nan")

    for key in (
        "v_ref",
        "v_ref_horizon_first",
        "v_ref_horizon_last",
        "v_ref_horizon_max",
        "reference_v_des_raw",
        "reference_v_des_target",
        "reference_v_des_eff",
        "reference_v_des_rate_limited",
        "terminal_v_envelope",
        "terminal_cmd_v_pre_clamp",
        "terminal_cmd_v_post_clamp",
        "cmd_v",
        "odom_v",
        "cmd_ax",
        "odom_ax",
        "cmd_jerk",
        "odom_jerk",
        "reference_implied_ax",
        "reference_implied_ay",
        "reference_implied_jerk",
        "terminal_envelope_active",
        "terminal_phase_active",
        "pct_v",
        "pct_slosh",
    ):
        ts, vals = series.get(key, (np.asarray([]), np.asarray([])))
        row[f"{key}_start"] = nearest(ts, vals, t0)
        row[f"{key}_terminal"] = nearest(ts, vals, terminal_t)
        row[f"{key}_min"] = min_val(ts, vals, t0, t1)
        row[f"{key}_max"] = max_val(ts, vals, t0, t1)
        row[f"{key}_p95_abs"] = p95_abs(ts, vals, t0, t1)
        row[f"{key}_max_abs"] = absmax(ts, vals, t0, t1)

    ht, hv = series.get("slosh_height_mm", (np.asarray([]), np.asarray([])))
    peak_t, peak_mm, p95_mm = peak(ht, hv, t0, t1)
    row["slosh_peak_t_s"] = peak_t
    row["slosh_peak_mm"] = peak_mm
    row["slosh_p95_mm"] = p95_mm
    for key in ("horizon_hmax_mm", "horizon_kmax", "horizon_ax_p95", "horizon_ay_p95", "pct_v", "pct_slosh"):
        ts, vals = series.get(key, (np.asarray([]), np.asarray([])))
        row[f"{key}_at_slosh_peak"] = nearest(ts, vals, peak_t)
    row["horizon_peak_class"] = classify_horizon_k(finite(row["horizon_kmax_at_slosh_peak"]))

    ref_candidates = [
        finite(row.get("v_ref_terminal")),
        finite(row.get("v_ref_horizon_first_terminal")),
        finite(row.get("v_ref_horizon_max_terminal")),
        finite(row.get("reference_v_des_eff_terminal")),
        finite(row.get("reference_v_des_rate_limited_terminal")),
    ]
    row["reference_speed_problem"] = int(any(v > args.ref_v_high for v in ref_candidates if math.isfinite(v)))
    row["execution_problem"] = int(
        finite(row.get("cmd_v_terminal")) <= args.cmd_v_low
        and finite(row.get("odom_v_terminal")) > args.odom_v_high
    )
    row["limit_problem"] = int(
        finite(row.get("cmd_ax_max_abs")) > args.ax_high
        or finite(row.get("cmd_jerk_max_abs")) > args.jerk_high
        or finite(row.get("odom_ax_max_abs")) > args.ax_high
        or finite(row.get("odom_jerk_max_abs")) > args.jerk_high
    )
    mode_upper = str(first_mode).upper()
    row["state_machine_problem"] = int(
        mode_upper == "NO_TERMINAL"
        or mode_upper in RECOVERY_MODES
        or finite(row.get("goal_dist_at_terminal")) > 0.8
        or (mode_upper == "REACHED" and finite(row.get("odom_v_terminal")) > args.odom_v_high)
    )
    row["model_peak_problem"] = int(row["horizon_peak_class"] in {"CURRENT_K0", "FUTURE_K_GT_0"})
    row["problem_classes"] = problem_flags(row, args)
    return row, data


def fmt(v, nd=3):
    if isinstance(v, str):
        return v
    v = finite(v)
    return "NA" if not math.isfinite(v) else f"{v:.{nd}f}"


def mean(rows, key):
    vals = [finite(r.get(key)) for r in rows]
    vals = [v for v in vals if math.isfinite(v)]
    return float(np.mean(vals)) if vals else float("nan")


def write_csv(path, rows):
    if not rows:
        return
    keys = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def group_summary(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["condition"]].append(row)
    out = []
    for condition, rs in sorted(grouped.items()):
        cls = Counter(r["horizon_peak_class"] for r in rs)
        out.append({
            "condition": condition,
            "n": len(rs),
            "reference_speed_problem_n": sum(int(r["reference_speed_problem"]) for r in rs),
            "execution_problem_n": sum(int(r["execution_problem"]) for r in rs),
            "limit_problem_n": sum(int(r["limit_problem"]) for r in rs),
            "state_machine_problem_n": sum(int(r["state_machine_problem"]) for r in rs),
            "current_k0_n": cls["CURRENT_K0"],
            "future_k_n": cls["FUTURE_K_GT_0"],
            "v_des_eff_terminal_mean": mean(rs, "reference_v_des_eff_terminal"),
            "v_ref_horizon_first_terminal_mean": mean(rs, "v_ref_horizon_first_terminal"),
            "cmd_v_terminal_mean": mean(rs, "cmd_v_terminal"),
            "odom_v_terminal_mean": mean(rs, "odom_v_terminal"),
            "cmd_ax_max_abs_mean": mean(rs, "cmd_ax_max_abs"),
            "odom_ax_max_abs_mean": mean(rs, "odom_ax_max_abs"),
            "cmd_jerk_max_abs_mean": mean(rs, "cmd_jerk_max_abs"),
            "odom_jerk_max_abs_mean": mean(rs, "odom_jerk_max_abs"),
            "slosh_peak_mm_mean": mean(rs, "slosh_peak_mm"),
            "slosh_p95_mm_mean": mean(rs, "slosh_p95_mm"),
            "pct_v_at_slosh_peak_mean": mean(rs, "pct_v_at_slosh_peak"),
            "pct_slosh_at_slosh_peak_mean": mean(rs, "pct_slosh_at_slosh_peak"),
        })
    return out


def write_report(path, rows, groups, args):
    lines = [
        "# Terminal Approach 1s Diagnostic Report",
        "",
        "窗口：第一次 terminal/capture 前 1 秒。该报告只用于终点收敛诊断，不进入 C/D/F 主效果统计。",
        "",
        "## 问题分类",
        "",
        "- 参考速度问题：`v_ref/v_des` 或 horizon first/max 在 terminal 处仍高。",
        "- 执行问题：`cmd_v` 已经低，但 `odom_v` 仍高。",
        "- 限幅问题：`cmd_v/odom_v` 差分得到的 ax 或 jerk 超阈值。",
        "- 状态机问题：terminal 触发太早、进入 recovery、或 REACHED 时速度仍高。",
        "- 模型问题：`/slosh` peak 对应 horizon 的 `k_h_total_max` 是当前 k=0 还是未来 k>0。",
        "",
        "## 分组摘要",
        "",
        "| group | n | ref问题 | 执行问题 | 限幅问题 | 状态机问题 | CURRENT_K0 | FUTURE_K>0 | vdes_term | cmd_v_term | odom_v_term | odom_ax_max | odom_jerk_max | slosh peak mm |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for g in groups:
        lines.append(
            f"| {g['condition']} | {int(g['n'])} | {int(g['reference_speed_problem_n'])} | "
            f"{int(g['execution_problem_n'])} | {int(g['limit_problem_n'])} | {int(g['state_machine_problem_n'])} | "
            f"{int(g['current_k0_n'])} | {int(g['future_k_n'])} | "
            f"{fmt(g['v_des_eff_terminal_mean'])} | {fmt(g['cmd_v_terminal_mean'])} | {fmt(g['odom_v_terminal_mean'])} | "
            f"{fmt(g['odom_ax_max_abs_mean'])} | {fmt(g['odom_jerk_max_abs_mean'])} | {fmt(g['slosh_peak_mm_mean'])} |"
        )
    lines.extend([
        "",
        "## 每包判读",
        "",
        "| bag | group | first mode | problems | vdes_term | cmd_v_term | odom_v_term | odom_ax_max | odom_jerk_max | peak mm | k class |",
        "|---|---|---|---|---:|---:|---:|---:|---:|---:|---|",
    ])
    for r in rows:
        lines.append(
            f"| {r['bag']} | {r['condition']} | {r['first_terminal_mode']} | {r['problem_classes']} | "
            f"{fmt(r.get('reference_v_des_eff_terminal'))} | {fmt(r.get('cmd_v_terminal'))} | {fmt(r.get('odom_v_terminal'))} | "
            f"{fmt(r.get('odom_ax_max_abs'))} | {fmt(r.get('odom_jerk_max_abs'))} | "
            f"{fmt(r.get('slosh_peak_mm'))} | {r.get('horizon_peak_class')} |"
        )
    lines.extend([
        "",
        "## 结构修改原则",
        "",
        "```text",
        "1. 如果参考速度问题占主导：先修 terminal velocity envelope / v_des 分配。",
        "2. 如果执行问题占主导：检查底盘响应、cmd_vel 到 odom 的延迟和底盘限速。",
        "3. 如果限幅问题占主导：只在 terminal 子系统内加 cmd_v rate/jerk 限制。",
        "4. 如果状态机问题占主导：修 capture/reached 条件，不改 slosh cost。",
        "5. 如果模型 peak 是 CURRENT_K0：说明 peak 来自过去激励累积，先查 terminal 前 ax/jerk。",
        "6. 如果模型 peak 是 FUTURE_K_GT_0：才考虑 terminal 内的预测约束或 soft constraint。",
        "```",
        "",
        "注意：terminal approach 的修改只用 terminal smoke 验证，不进入固定路径 C/D/F 主效果统计。",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_bag(path, row, data, out_dir):
    if plt is None:
        return
    t0 = finite(row["window_start_s"])
    t1 = finite(row["terminal_time_s"])
    series = data["series"]
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    panels = [
        ("velocity", ("v_ref", "reference_v_des_eff", "cmd_v", "odom_v", "terminal_v_envelope")),
        ("ax", ("reference_implied_ax", "cmd_ax", "odom_ax")),
        ("jerk", ("reference_implied_jerk", "cmd_jerk", "odom_jerk")),
        ("slosh", ("slosh_height_mm", "horizon_hmax_mm")),
        ("cost/terminal", ("pct_v", "pct_slosh", "terminal_phase_active", "terminal_envelope_active")),
    ]
    for ax, (ylabel, keys) in zip(axes, panels):
        for key in keys:
            ts, vals = series.get(key, (np.asarray([]), np.asarray([])))
            if len(ts) == 0:
                continue
            mask = (ts >= t0) & (ts <= t1)
            if np.any(mask):
                ax.plot(ts[mask] - t1, vals[mask], label=key)
        ax.axvline(0.0, color="k", linestyle="--", linewidth=1)
        ax.grid(True, alpha=0.25)
        ax.set_ylabel(ylabel)
        ax.legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("time to first terminal/capture (s)")
    fig.suptitle(f"{row['condition']} {row['bag']}")
    fig.tight_layout()
    fig.savefig(out_dir / f"{Path(row['bag']).stem}_terminal_approach_1s.png", dpi=160)
    plt.close(fig)


def main():
    args = parse_args()
    bags = collect_bags(args)
    if not bags:
        raise SystemExit("No bags found.")
    out_dir = Path(args.out_dir)
    plot_dir = out_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.plot:
        plot_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    data_by_bag = {}
    for idx, bag in enumerate(bags, 1):
        print(f"[{idx}/{len(bags)}] {bag}")
        row, data = analyze_bag(bag, args)
        rows.append(row)
        data_by_bag[bag.name] = data
        if args.plot:
            plot_bag(bag, row, data, plot_dir)

    groups = group_summary(rows)
    write_csv(out_dir / "terminal_approach_1s.csv", rows)
    write_csv(out_dir / "terminal_approach_1s_group_summary.csv", groups)
    write_report(out_dir / "TERMINAL_APPROACH_1S_REPORT.md", rows, groups, args)
    print(f"[OK] {out_dir / 'TERMINAL_APPROACH_1S_REPORT.md'}")


if __name__ == "__main__":
    main()
