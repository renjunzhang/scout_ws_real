#!/usr/bin/env python3
"""Analyze terminal/capture transitions and near-goal ax/jerk pulses from bags.

The script is intentionally focused on slosh experiments:

* when /terminal/mode changes;
* whether CAPTURE_STOP/CAPTURE_BRAKE or terminal recovery modes appear;
* cmd_v / odom_v / ax / jerk before and after terminal entry;
* whether near-goal reference diagnostics are available and low.

Current historical real bags may not contain /reference/v_ref or /reference/implied_*.
In that case the script reports MISSING_REF_TOPICS instead of guessing horizon v_ref.
"""

import argparse
import csv
import math
import os
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


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

IDLE_TERMINAL_MODES = {"", "NONE", "IDLE"}
RECOVERY_MODES = {"ALIGN_TO_POINT", "APPROACH_POINT", "ALIGN_FINAL_YAW"}


def finite(value):
    try:
        v = float(value)
        return v if math.isfinite(v) else float("nan")
    except Exception:
        return float("nan")


def scalar_msg(msg):
    return finite(getattr(msg, "data", float("nan")))


def time_to_sec(stamp):
    return stamp.to_sec() if hasattr(stamp, "to_sec") else float(stamp)


def percentile_abs(values, q):
    vals = np.asarray([abs(v) for v in values if math.isfinite(v)], dtype=float)
    if vals.size == 0:
        return float("nan")
    return float(np.percentile(vals, q))


def max_abs(values):
    vals = np.asarray([abs(v) for v in values if math.isfinite(v)], dtype=float)
    if vals.size == 0:
        return float("nan")
    return float(np.max(vals))


def min_finite(values):
    vals = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    if vals.size == 0:
        return float("nan")
    return float(np.min(vals))


def max_finite(values):
    vals = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    if vals.size == 0:
        return float("nan")
    return float(np.max(vals))


def interp_step(series, ts, default=None):
    if not series:
        return default
    # series sorted by time; return latest sample at or before ts
    lo, hi = 0, len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= ts:
            lo = mid + 1
        else:
            hi = mid
    idx = lo - 1
    if idx < 0:
        return default
    return series[idx][1]


def derivative(series):
    out = []
    prev = None
    for ts, value in series:
        if prev is not None:
            prev_ts, prev_value = prev
            dt = ts - prev_ts
            if dt > 1e-4 and math.isfinite(value) and math.isfinite(prev_value):
                out.append((ts, (value - prev_value) / dt))
        prev = (ts, value)
    return out


def filter_window(series, t0, t1):
    return [(ts, v) for ts, v in series if t0 <= ts <= t1 and math.isfinite(v)]


def values_window(series, t0, t1):
    return [v for _, v in filter_window(series, t0, t1)]


def first_terminal_event(modes):
    for ts, mode in modes:
        if str(mode).upper() not in IDLE_TERMINAL_MODES:
            return ts, str(mode)
    return float("nan"), ""


def first_tracking_start(status):
    for ts, value in status:
        if str(value).upper() == "TRACKING":
            return ts
    return float("nan")


def read_bag(bag_path, args):
    data = {
        "mode": [],
        "recovery_latched": [],
        "goal_info": [],
        "cmd_v": [],
        "cmd_w": [],
        "odom_v": [],
        "odom_w": [],
        "ref_v": [],
        "ref_v_horizon": [],
        "ref_s_horizon": [],
        "ref_ax": [],
        "ref_ay": [],
        "ref_jerk": [],
        "ref_ax_p95": [],
        "ref_ay_p95": [],
        "ref_jerk_p95": [],
        "mpc_status": [],
    }
    topics = [
        args.mode_topic,
        args.recovery_topic,
        args.goal_info_topic,
        args.cmd_topic,
        args.odom_topic,
        args.ref_v_topic,
        args.ref_v_horizon_topic,
        args.ref_s_horizon_topic,
        args.ref_ax_topic,
        args.ref_ay_topic,
        args.ref_jerk_topic,
        args.ref_ax_p95_topic,
        args.ref_ay_p95_topic,
        args.ref_jerk_p95_topic,
        args.status_topic,
    ]
    with rosbag.Bag(str(bag_path), "r") as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = stamp.to_sec()
            if topic == args.mode_topic:
                data["mode"].append((ts, str(getattr(msg, "data", ""))))
            elif topic == args.recovery_topic:
                data["recovery_latched"].append((ts, int(getattr(msg, "data", 0)) != 0))
            elif topic == args.goal_info_topic:
                values = list(getattr(msg, "data", []))
                if len(values) >= len(GOAL_INFO_FIELDS):
                    info = dict(zip(GOAL_INFO_FIELDS, [finite(v) for v in values[:8]]))
                    data["goal_info"].append((ts, info))
            elif topic == args.cmd_topic:
                data["cmd_v"].append((ts, finite(msg.linear.x)))
                data["cmd_w"].append((ts, finite(msg.angular.z)))
            elif topic == args.odom_topic:
                data["odom_v"].append((ts, finite(msg.twist.twist.linear.x)))
                data["odom_w"].append((ts, finite(msg.twist.twist.angular.z)))
            elif topic == args.ref_v_topic:
                data["ref_v"].append((ts, scalar_msg(msg)))
            elif topic == args.ref_v_horizon_topic:
                data["ref_v_horizon"].append((ts, [finite(v) for v in getattr(msg, "data", [])]))
            elif topic == args.ref_s_horizon_topic:
                data["ref_s_horizon"].append((ts, [finite(v) for v in getattr(msg, "data", [])]))
            elif topic == args.ref_ax_topic:
                data["ref_ax"].append((ts, scalar_msg(msg)))
            elif topic == args.ref_ay_topic:
                data["ref_ay"].append((ts, scalar_msg(msg)))
            elif topic == args.ref_jerk_topic:
                data["ref_jerk"].append((ts, scalar_msg(msg)))
            elif topic == args.ref_ax_p95_topic:
                data["ref_ax_p95"].append((ts, scalar_msg(msg)))
            elif topic == args.ref_ay_p95_topic:
                data["ref_ay_p95"].append((ts, scalar_msg(msg)))
            elif topic == args.ref_jerk_p95_topic:
                data["ref_jerk_p95"].append((ts, scalar_msg(msg)))
            elif topic == args.status_topic:
                data["mpc_status"].append((ts, str(getattr(msg, "data", ""))))
    data["bag_start"] = start
    data["bag_end"] = end
    return data


def near_goal_samples(data, max_dist):
    out = []
    for ts, info in data["goal_info"]:
        dist = info.get("dist", float("nan"))
        if math.isfinite(dist) and dist <= max_dist:
            out.append((ts, info))
    return out


def analyze_bag(bag_path, args, out_dir=None):
    data = read_bag(bag_path, args)
    cmd_ax = derivative(data["cmd_v"])
    odom_ax = derivative(data["odom_v"])
    cmd_jerk = derivative(cmd_ax)
    odom_jerk = derivative(odom_ax)

    terminal_ts, first_mode = first_terminal_event(data["mode"])
    tracking_start = first_tracking_start(data["mpc_status"])
    if not math.isfinite(terminal_ts):
        terminal_ts = data["bag_end"]
        first_mode = "NO_TERMINAL_MODE"

    pre0 = terminal_ts - args.pre_window
    post1 = terminal_ts + args.post_window
    pre2m = near_goal_samples(data, args.near_goal_dist)
    pre1m = near_goal_samples(data, args.near_goal_close_dist)

    mode_counts = Counter(mode for _, mode in data["mode"])
    recovery_counts = Counter(
        mode for _, mode in data["mode"] if str(mode).upper() in RECOVERY_MODES
    )
    capture_triggered = int(any(
        str(mode).upper() in {"CAPTURE_STOP", "CAPTURE_BRAKE", "GOAL_STOP_PENDING"}
        for _, mode in data["mode"]
    ))
    recovery_triggered = int(any(recovery_counts.values()))

    near_ts = [ts for ts, _ in pre2m]
    near1_ts = [ts for ts, _ in pre1m]
    near0 = min(near_ts) if near_ts else float("nan")
    near1 = max(near_ts) if near_ts else float("nan")
    close0 = min(near1_ts) if near1_ts else float("nan")
    close1 = max(near1_ts) if near1_ts else float("nan")

    row = {
        "bag": os.path.basename(str(bag_path)),
        "tracking_start": tracking_start,
        "first_terminal_time": terminal_ts,
        "first_terminal_rel_s": terminal_ts - tracking_start if math.isfinite(tracking_start) else float("nan"),
        "first_terminal_mode": first_mode,
        "capture_stop_triggered": capture_triggered,
        "recovery_triggered": recovery_triggered,
        "mode_counts": ";".join(f"{k}:{v}" for k, v in sorted(mode_counts.items())),
        "recovery_mode_counts": ";".join(f"{k}:{v}" for k, v in sorted(recovery_counts.items())),
        "has_reference_scalar_topics": int(bool(data["ref_v"])),
        "has_reference_horizon_v_topics": int(bool(data["ref_v_horizon"])),
        "has_reference_horizon_p95_topics": int(bool(data["ref_ax_p95"] or data["ref_jerk_p95"])),
    }

    for name, series in (
        ("cmd_v", data["cmd_v"]),
        ("odom_v", data["odom_v"]),
        ("cmd_ax", cmd_ax),
        ("odom_ax", odom_ax),
        ("cmd_jerk", cmd_jerk),
        ("odom_jerk", odom_jerk),
    ):
        pre_vals = values_window(series, pre0, terminal_ts)
        post_vals = values_window(series, terminal_ts, post1)
        row[f"{name}_pre_p95_abs"] = percentile_abs(pre_vals, 95)
        row[f"{name}_pre_max_abs"] = max_abs(pre_vals)
        row[f"{name}_post_p95_abs"] = percentile_abs(post_vals, 95)
        row[f"{name}_post_max_abs"] = max_abs(post_vals)
        row[f"{name}_at_terminal"] = interp_step(series, terminal_ts, float("nan"))

    if math.isfinite(near0) and math.isfinite(near1):
        for dist_label, t0, t1 in (
            ("near2m", near0, near1),
            ("near1m", close0, close1),
        ):
            if math.isfinite(t0) and math.isfinite(t1):
                row[f"{dist_label}_duration_s"] = t1 - t0
                row[f"{dist_label}_ref_v_min"] = min_finite(values_window(data["ref_v"], t0, t1))
                row[f"{dist_label}_ref_v_max"] = max_finite(values_window(data["ref_v"], t0, t1))
                row[f"{dist_label}_ref_v_terminal"] = interp_step(data["ref_v"], terminal_ts, float("nan"))
                horizon_arrays = [
                    arr for ts, arr in data["ref_v_horizon"]
                    if t0 <= ts <= t1 and arr
                ]
                row[f"{dist_label}_horizon_v_min"] = min_finite(
                    [min_finite(arr) for arr in horizon_arrays]
                )
                row[f"{dist_label}_horizon_v_max"] = max_finite(
                    [max_finite(arr) for arr in horizon_arrays]
                )
                term_arr = interp_step(data["ref_v_horizon"], terminal_ts, [])
                row[f"{dist_label}_horizon_v_terminal_first"] = (
                    finite(term_arr[0]) if term_arr else float("nan")
                )
                row[f"{dist_label}_horizon_v_terminal_last"] = (
                    finite(term_arr[-1]) if term_arr else float("nan")
                )
                row[f"{dist_label}_horizon_v_terminal_max"] = (
                    max_finite(term_arr) if term_arr else float("nan")
                )
                row[f"{dist_label}_ref_ax_p95_abs_max"] = max_abs(values_window(data["ref_ax_p95"], t0, t1))
                row[f"{dist_label}_ref_jerk_p95_abs_max"] = max_abs(values_window(data["ref_jerk_p95"], t0, t1))
            else:
                row[f"{dist_label}_duration_s"] = float("nan")
                row[f"{dist_label}_ref_v_min"] = float("nan")
                row[f"{dist_label}_ref_v_max"] = float("nan")
                row[f"{dist_label}_ref_v_terminal"] = float("nan")
                row[f"{dist_label}_horizon_v_min"] = float("nan")
                row[f"{dist_label}_horizon_v_max"] = float("nan")
                row[f"{dist_label}_horizon_v_terminal_first"] = float("nan")
                row[f"{dist_label}_horizon_v_terminal_last"] = float("nan")
                row[f"{dist_label}_horizon_v_terminal_max"] = float("nan")
                row[f"{dist_label}_ref_ax_p95_abs_max"] = float("nan")
                row[f"{dist_label}_ref_jerk_p95_abs_max"] = float("nan")
    else:
        row["near2m_duration_s"] = float("nan")
        row["near2m_ref_v_min"] = float("nan")
        row["near2m_ref_v_max"] = float("nan")
        row["near2m_ref_v_terminal"] = float("nan")
        row["near2m_horizon_v_min"] = float("nan")
        row["near2m_horizon_v_max"] = float("nan")
        row["near2m_horizon_v_terminal_first"] = float("nan")
        row["near2m_horizon_v_terminal_last"] = float("nan")
        row["near2m_horizon_v_terminal_max"] = float("nan")
        row["near2m_ref_ax_p95_abs_max"] = float("nan")
        row["near2m_ref_jerk_p95_abs_max"] = float("nan")

    row["terminal_ref_v_verdict"] = reference_verdict(row, args)
    row["terminal_pulse_verdict"] = pulse_verdict(row, args)
    row["primary_flags"] = primary_flags(row, args)

    if out_dir is not None and args.plots:
        plot_bag(bag_path, data, cmd_ax, odom_ax, cmd_jerk, odom_jerk, terminal_ts, out_dir, args)

    return row


def reference_verdict(row, args):
    if row.get("has_reference_horizon_v_topics"):
        h_first = row.get("near2m_horizon_v_terminal_first", float("nan"))
        h_last = row.get("near2m_horizon_v_terminal_last", float("nan"))
        h_max = row.get("near2m_horizon_v_terminal_max", float("nan"))
        if (
            math.isfinite(h_first) and h_first <= args.ref_v_low and
            math.isfinite(h_last) and h_last <= args.ref_v_low and
            math.isfinite(h_max) and h_max <= args.ref_v_low
        ):
            return "HORIZON_ALL_LOW_AT_TERMINAL"
        if math.isfinite(h_first) and h_first <= args.ref_v_low:
            return "HORIZON_FIRST_LOW_BUT_FUTURE_HIGH"
        if math.isfinite(h_first) and h_first > args.ref_v_high:
            return "HORIZON_FIRST_STAYS_HIGH_AT_TERMINAL"
        return "HORIZON_REF_UNCLEAR"
    if not row.get("has_reference_scalar_topics"):
        return "MISSING_REF_TOPICS"
    near_max = row.get("near2m_ref_v_max", float("nan"))
    term = row.get("near2m_ref_v_terminal", float("nan"))
    close_min = row.get("near1m_ref_v_min", float("nan"))
    if math.isfinite(term) and abs(term) <= args.ref_v_low:
        if math.isfinite(close_min) and close_min <= args.ref_v_low:
            return "REF_LOW_AT_TERMINAL"
        return "REF_LOW_AT_TERMINAL_ONLY"
    if math.isfinite(near_max) and near_max > args.ref_v_high:
        return "REF_STAYS_HIGH_NEAR_GOAL"
    return "REF_UNCLEAR"


def pulse_verdict(row, args):
    flags = []
    if row.get("odom_ax_pre_max_abs", 0.0) > args.ax_high:
        flags.append("HIGH_ODOM_AX_BEFORE_TERMINAL")
    if row.get("odom_ax_post_max_abs", 0.0) > args.ax_high:
        flags.append("HIGH_ODOM_AX_AFTER_TERMINAL")
    if row.get("odom_jerk_pre_max_abs", 0.0) > args.jerk_high:
        flags.append("HIGH_ODOM_JERK_BEFORE_TERMINAL")
    if row.get("odom_jerk_post_max_abs", 0.0) > args.jerk_high:
        flags.append("HIGH_ODOM_JERK_AFTER_TERMINAL")
    if row.get("cmd_ax_post_max_abs", 0.0) > args.ax_high:
        flags.append("HIGH_CMD_AX_AFTER_TERMINAL")
    return ",".join(flags) if flags else "OK_OR_LOW"


def primary_flags(row, args):
    flags = []
    if row.get("capture_stop_triggered"):
        flags.append("CAPTURE_STOP")
    if row.get("recovery_triggered"):
        flags.append("RECOVERY_" + (row.get("recovery_mode_counts") or "UNKNOWN"))
    if row.get("terminal_ref_v_verdict") == "MISSING_REF_TOPICS":
        flags.append("NO_REF_DIAG_TOPICS")
    elif row.get("terminal_ref_v_verdict") == "REF_STAYS_HIGH_NEAR_GOAL":
        flags.append("REF_HIGH_NEAR_GOAL")
    pulse = row.get("terminal_pulse_verdict", "")
    if pulse != "OK_OR_LOW":
        flags.append(pulse)
    return "|".join(flags) if flags else "NO_MAJOR_FLAG"


def plot_bag(bag_path, data, cmd_ax, odom_ax, cmd_jerk, odom_jerk, terminal_ts, out_dir, args):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(bag_path).stem
    t0 = terminal_ts - args.plot_pre
    t1 = terminal_ts + args.plot_post

    fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
    panels = [
        ("velocity (m/s)", (("cmd_v", data["cmd_v"]), ("odom_v", data["odom_v"]), ("ref_v", data["ref_v"]))),
        ("ax (m/s^2)", (("cmd_ax", cmd_ax), ("odom_ax", odom_ax), ("ref_ax", data["ref_ax"]))),
        ("jerk-like (m/s^3)", (("cmd_jerk", cmd_jerk), ("odom_jerk", odom_jerk), ("ref_jerk", data["ref_jerk"]))),
        ("terminal dist/mode", (("goal_dist", [(ts, info["dist"]) for ts, info in data["goal_info"]]),)),
    ]
    for ax, (ylabel, series_list) in zip(axes, panels):
        for label, series in series_list:
            xs = [ts - terminal_ts for ts, _ in series if t0 <= ts <= t1]
            ys = [v for ts, v in series if t0 <= ts <= t1]
            if xs:
                ax.plot(xs, ys, label=label, linewidth=1.5)
        ax.axvline(0.0, color="k", linestyle="--", linewidth=1.0, alpha=0.7)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)

    # Mark terminal mode changes on top axis.
    for ts, mode in data["mode"]:
        if t0 <= ts <= t1 and str(mode).upper() not in IDLE_TERMINAL_MODES:
            axes[0].axvline(ts - terminal_ts, color="tab:red", alpha=0.25)
            axes[0].text(
                ts - terminal_ts,
                axes[0].get_ylim()[1],
                str(mode),
                rotation=90,
                va="top",
                ha="right",
                fontsize=7,
                color="tab:red",
            )

    axes[-1].set_xlabel("time relative to first terminal/capture (s)")
    fig.suptitle(stem)
    fig.tight_layout()
    fig.savefig(out_dir / f"{stem}_terminal_transition.png", dpi=160)
    plt.close(fig)


def discover_bags(args):
    bags = []
    if args.bag:
        bags.append(Path(args.bag))
    if args.bag_dir:
        bags.extend(sorted(Path(args.bag_dir).glob(args.glob)))
    # Deduplicate preserving order.
    seen = set()
    out = []
    for path in bags:
        key = str(path.resolve())
        if key not in seen:
            seen.add(key)
            out.append(path)
    return out


def write_csv(path, rows):
    if not rows:
        return
    keys = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value, nd=3):
    if isinstance(value, str):
        return value
    try:
        v = float(value)
    except Exception:
        return "NA"
    if not math.isfinite(v):
        return "NA"
    return f"{v:.{nd}f}"


def write_markdown(path, rows, args):
    lines = [
        "# Terminal transition diagnostics",
        "",
        "窗口：`first terminal/capture` 前后。`ax/jerk` 由 `cmd_vel.linear.x` 和 `odom.twist.twist.linear.x` 差分得到。",
        "",
        "注意：`terminal_ref_v_verdict=MISSING_REF_TOPICS` 表示 bag 没录 `/reference/v_ref_horizon` 或 `/reference/v_ref`，无法判断最后 1-2m 内参考速度是否真的降到 0。",
        "",
        "| bag | first mode | capture | recovery | ref verdict | pulse verdict | odom ax pre/post max | odom jerk pre/post max | flags |",
        "|---|---|---:|---:|---|---|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            "| {bag} | {mode} | {cap} | {rec} | {ref} | {pulse} | {axpre}/{axpost} | {jpre}/{jpost} | {flags} |".format(
                bag=row.get("bag", ""),
                mode=row.get("first_terminal_mode", ""),
                cap=row.get("capture_stop_triggered", 0),
                rec=row.get("recovery_triggered", 0),
                ref=row.get("terminal_ref_v_verdict", ""),
                pulse=row.get("terminal_pulse_verdict", ""),
                axpre=fmt(row.get("odom_ax_pre_max_abs")),
                axpost=fmt(row.get("odom_ax_post_max_abs")),
                jpre=fmt(row.get("odom_jerk_pre_max_abs")),
                jpost=fmt(row.get("odom_jerk_post_max_abs")),
                flags=row.get("primary_flags", ""),
            )
        )
    lines.extend(
        [
            "",
            "## 判读规则",
            "",
            f"- `ref_v_low <= {args.ref_v_low}` m/s：认为 terminal 处 ref 已经低。",
            f"- `ref_v_high > {args.ref_v_high}` m/s：认为 near-goal ref 仍偏高。",
            f"- `ax_high > {args.ax_high}` m/s^2：标记 ax 脉冲。",
            f"- `jerk_high > {args.jerk_high}` m/s^3：标记 jerk-like 脉冲。",
            "",
            "## 下一步",
            "",
            "- 如果大量 bag 是 `MISSING_REF_TOPICS`，下一轮录包必须包含 `/reference/v_ref`、`/reference/implied_ax`、`/reference/implied_jerk` 和 p95 诊断话题。",
            "- 如果 `CAPTURE_STOP/CAPTURE_BRAKE` 后 `odom_ax_post_max_abs` 或 `cmd_ax_post_max_abs` 仍然高，继续收紧 capture brake 减速度或提前参考减速。",
            "- 如果 recovery 模式大量出现，先限制 recovery 只做兜底，否则 slosh cost 的效果会被终点几何接管污染。",
        ]
    )
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", default="", help="Single bag to analyze.")
    parser.add_argument("--bag-dir", default="", help="Directory containing bags.")
    parser.add_argument("--glob", default="*.bag", help="Glob used with --bag-dir.")
    parser.add_argument("--out-dir", required=True, help="Output directory for CSV/plots/report.")
    parser.add_argument("--pre-window", type=float, default=2.0)
    parser.add_argument("--post-window", type=float, default=2.0)
    parser.add_argument("--plot-pre", type=float, default=4.0)
    parser.add_argument("--plot-post", type=float, default=4.0)
    parser.add_argument("--near-goal-dist", type=float, default=2.0)
    parser.add_argument("--near-goal-close-dist", type=float, default=1.0)
    parser.add_argument("--ref-v-low", type=float, default=0.05)
    parser.add_argument("--ref-v-high", type=float, default=0.12)
    parser.add_argument("--ax-high", type=float, default=0.8)
    parser.add_argument("--jerk-high", type=float, default=8.0)
    parser.add_argument("--plots", action="store_true")

    parser.add_argument("--mode-topic", default="/terminal/mode")
    parser.add_argument("--recovery-topic", default="/terminal/recovery_latched")
    parser.add_argument("--goal-info-topic", default="/terminal/goal_info")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--status-topic", default="/mpc_status")
    parser.add_argument("--ref-v-topic", default="/reference/v_ref")
    parser.add_argument("--ref-v-horizon-topic", default="/reference/v_ref_horizon")
    parser.add_argument("--ref-s-horizon-topic", default="/reference/s_horizon")
    parser.add_argument("--ref-ax-topic", default="/reference/implied_ax")
    parser.add_argument("--ref-ay-topic", default="/reference/implied_ay")
    parser.add_argument("--ref-jerk-topic", default="/reference/implied_jerk")
    parser.add_argument("--ref-ax-p95-topic", default="/reference/implied_ax_abs_p95")
    parser.add_argument("--ref-ay-p95-topic", default="/reference/implied_ay_abs_p95")
    parser.add_argument("--ref-jerk-p95-topic", default="/reference/implied_jerk_abs_p95")
    return parser.parse_args()


def main():
    args = parse_args()
    bags = discover_bags(args)
    if not bags:
        raise SystemExit("No bags found.")

    out_dir = Path(args.out_dir)
    plot_dir = out_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.plots:
        plot_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for idx, bag_path in enumerate(bags, 1):
        print(f"[{idx}/{len(bags)}] {bag_path}")
        rows.append(analyze_bag(bag_path, args, plot_dir if args.plots else None))

    write_csv(out_dir / "terminal_transition_summary.csv", rows)
    write_markdown(out_dir / "TERMINAL_TRANSITION_REPORT.md", rows, args)
    print(out_dir / "TERMINAL_TRANSITION_REPORT.md")


if __name__ == "__main__":
    main()
