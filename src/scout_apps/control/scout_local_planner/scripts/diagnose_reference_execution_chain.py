#!/usr/bin/env python3
"""Diagnose whether a low-excitation reference is executed by cmd/odom."""

import argparse
import csv
import math
import os
from bisect import bisect_right

import numpy as np
import rosbag

from extract_slosh_metrics import (
    get_status_segments,
    get_string_segments,
    in_terminal_none,
    in_tracking,
    percentile,
)


def condition_from_name(path):
    name = os.path.basename(path)
    parts = name.replace(".bag", "").split("_")
    for i, part in enumerate(parts):
        if part in ("NOM", "CUSTOM", "TOPPRA", "RUCKIG"):
            return "_".join(parts[i:-1]) if parts[-1].startswith("run") else "_".join(parts[i:])
    return "UNKNOWN"


def nearest_value(series, ts, idx=0):
    if not series:
        return idx, float("nan")
    while idx + 1 < len(series) and abs(series[idx + 1][0] - ts) <= abs(series[idx][0] - ts):
        idx += 1
    return idx, series[idx][1]


def interp_series(series, grid):
    if len(series) < 2:
        return np.full_like(grid, np.nan, dtype=float)
    ts = np.array([x[0] for x in series], dtype=float)
    vals = np.array([x[1] for x in series], dtype=float)
    return np.interp(grid, ts, vals, left=np.nan, right=np.nan)


def finite_pair(a, b):
    mask = np.isfinite(a) & np.isfinite(b)
    return a[mask], b[mask]


def rmse(a, b):
    a, b = finite_pair(a, b)
    if len(a) == 0:
        return float("nan")
    return float(np.sqrt(np.mean((a - b) ** 2)))


def corr(a, b):
    a, b = finite_pair(a, b)
    if len(a) < 3 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def best_lag_corr(reference, candidate, dt, max_lag_s):
    best_lag = float("nan")
    best_corr = float("nan")
    max_steps = int(round(max_lag_s / dt))
    for lag_steps in range(-max_steps, max_steps + 1):
        if lag_steps < 0:
            a = reference[-lag_steps:]
            b = candidate[:len(a)]
        elif lag_steps > 0:
            a = reference[:-lag_steps]
            b = candidate[lag_steps:]
        else:
            a = reference
            b = candidate
        c = corr(a, b)
        if math.isfinite(c) and (not math.isfinite(best_corr) or abs(c) > abs(best_corr)):
            best_corr = c
            best_lag = lag_steps * dt
    return best_lag, best_corr


def derivative(series):
    out = []
    prev = None
    for ts, value in series:
        if prev is not None:
            prev_ts, prev_value = prev
            dt = ts - prev_ts
            if dt > 1e-6:
                out.append((ts, (value - prev_value) / dt))
        prev = (ts, value)
    return out


def abs_p95(values):
    vals = [abs(v) for v in values if math.isfinite(v)]
    return percentile(vals, 95)


def peak_time(series, start_time):
    finite = [(ts, value) for ts, value in series if math.isfinite(value)]
    if not finite:
        return float("nan"), float("nan")
    ts, value = max(finite, key=lambda x: abs(x[1]))
    return ts - start_time, value


def ratio(num, den):
    if not math.isfinite(num) or not math.isfinite(den) or abs(den) < 1e-9:
        return float("nan")
    return num / den


def read_series(bag_path, tracking_only=True, strict_terminal_none=True):
    _, _, transitions, segments = get_status_segments(bag_path)
    _, _, _, terminal_segments = get_string_segments(bag_path, "/terminal/mode")
    series = {
        "ref_v": [],
        "ref_ax": [],
        "ref_ay": [],
        "ref_jerk": [],
        "cmd_v": [],
        "cmd_w": [],
        "odom_v": [],
        "odom_w": [],
    }
    topics = [
        "/reference/v_ref",
        "/reference/implied_ax",
        "/reference/implied_ay",
        "/reference/implied_jerk",
        "/cmd_vel",
        "/odom",
        "/terminal/mode",
        "/mpc_status",
    ]
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = stamp.to_sec()
            if tracking_only:
                keep = in_tracking(segments, ts)
                if strict_terminal_none:
                    keep = keep and in_terminal_none(terminal_segments, ts)
                if not keep:
                    continue
            if topic == "/reference/v_ref":
                series["ref_v"].append((ts, float(msg.data)))
            elif topic == "/reference/implied_ax":
                series["ref_ax"].append((ts, float(msg.data)))
            elif topic == "/reference/implied_ay":
                series["ref_ay"].append((ts, float(msg.data)))
            elif topic == "/reference/implied_jerk":
                series["ref_jerk"].append((ts, float(msg.data)))
            elif topic == "/cmd_vel":
                series["cmd_v"].append((ts, float(msg.linear.x)))
                series["cmd_w"].append((ts, float(msg.angular.z)))
            elif topic == "/odom":
                series["odom_v"].append((ts, float(msg.twist.twist.linear.x)))
                series["odom_w"].append((ts, float(msg.twist.twist.angular.z)))
    return series


def analyze_bag(bag_path, grid_dt=0.02, lag_window_s=1.0):
    series = read_series(bag_path)
    starts = [s[0][0] for s in series.values() if s]
    ends = [s[-1][0] for s in series.values() if s]
    if not starts or not ends:
        raise RuntimeError(f"no usable samples: {bag_path}")
    start_time = max(starts)
    end_time = min(ends)
    if end_time <= start_time + grid_dt:
        raise RuntimeError(f"insufficient overlap: {bag_path}")

    grid = np.arange(start_time, end_time, grid_dt)
    ref_v = interp_series(series["ref_v"], grid)
    cmd_v = interp_series(series["cmd_v"], grid)
    odom_v = interp_series(series["odom_v"], grid)
    ref_ax_topic = interp_series(series["ref_ax"], grid)
    cmd_ax = np.gradient(cmd_v, grid_dt)
    odom_ax = np.gradient(odom_v, grid_dt)
    ref_ax_dvdt = np.gradient(ref_v, grid_dt)

    raw_cmd_ax = derivative(series["cmd_v"])
    raw_odom_ax = derivative(series["odom_v"])

    ref_peak_t, ref_peak_value = peak_time(series["ref_ax"], start_time)
    cmd_peak_t, cmd_peak_value = peak_time(raw_cmd_ax, start_time)
    odom_peak_t, odom_peak_value = peak_time(raw_odom_ax, start_time)

    cmd_lag_v, cmd_corr_v = best_lag_corr(ref_v, cmd_v, grid_dt, lag_window_s)
    odom_lag_v, odom_corr_v = best_lag_corr(ref_v, odom_v, grid_dt, lag_window_s)
    cmd_odom_lag_v, cmd_odom_corr_v = best_lag_corr(cmd_v, odom_v, grid_dt, lag_window_s)

    cmd_lag_ax, cmd_corr_ax = best_lag_corr(ref_ax_topic, cmd_ax, grid_dt, lag_window_s)
    odom_lag_ax, odom_corr_ax = best_lag_corr(ref_ax_topic, odom_ax, grid_dt, lag_window_s)
    cmd_odom_lag_ax, cmd_odom_corr_ax = best_lag_corr(cmd_ax, odom_ax, grid_dt, lag_window_s)

    row = {
        "bag": os.path.basename(bag_path),
        "condition": condition_from_name(bag_path),
        "duration_s": round(end_time - start_time, 3),
        "ref_samples": len(series["ref_v"]),
        "cmd_samples": len(series["cmd_v"]),
        "odom_samples": len(series["odom_v"]),
        "ref_v_mean": round(float(np.nanmean(ref_v)), 3),
        "cmd_v_mean": round(float(np.nanmean(cmd_v)), 3),
        "odom_v_mean": round(float(np.nanmean(odom_v)), 3),
        "cmd_v_rmse_vs_ref": round(rmse(ref_v, cmd_v), 3),
        "odom_v_rmse_vs_ref": round(rmse(ref_v, odom_v), 3),
        "odom_v_rmse_vs_cmd": round(rmse(cmd_v, odom_v), 3),
        "cmd_v_best_lag_s": round(cmd_lag_v, 3),
        "cmd_v_best_corr": round(cmd_corr_v, 3),
        "odom_v_best_lag_s": round(odom_lag_v, 3),
        "odom_v_best_corr": round(odom_corr_v, 3),
        "odom_v_vs_cmd_best_lag_s": round(cmd_odom_lag_v, 3),
        "odom_v_vs_cmd_best_corr": round(cmd_odom_corr_v, 3),
        "ref_ax_topic_abs_p95": round(abs_p95(ref_ax_topic), 3),
        "ref_ax_dvdt_abs_p95": round(abs_p95(ref_ax_dvdt), 3),
        "cmd_ax_grid_abs_p95": round(abs_p95(cmd_ax), 3),
        "odom_ax_grid_abs_p95": round(abs_p95(odom_ax), 3),
        "cmd_ax_raw_abs_p95": round(abs_p95([v for _, v in raw_cmd_ax]), 3),
        "odom_ax_raw_abs_p95": round(abs_p95([v for _, v in raw_odom_ax]), 3),
        "cmd_ax_to_ref_ratio": round(ratio(abs_p95(cmd_ax), abs_p95(ref_ax_topic)), 3),
        "odom_ax_to_ref_ratio": round(ratio(abs_p95(odom_ax), abs_p95(ref_ax_topic)), 3),
        "odom_ax_to_cmd_ratio": round(ratio(abs_p95(odom_ax), abs_p95(cmd_ax)), 3),
        "cmd_ax_best_lag_s": round(cmd_lag_ax, 3),
        "cmd_ax_best_corr": round(cmd_corr_ax, 3),
        "odom_ax_best_lag_s": round(odom_lag_ax, 3),
        "odom_ax_best_corr": round(odom_corr_ax, 3),
        "odom_ax_vs_cmd_best_lag_s": round(cmd_odom_lag_ax, 3),
        "odom_ax_vs_cmd_best_corr": round(cmd_odom_corr_ax, 3),
        "ref_ax_peak_time_s": round(ref_peak_t, 3),
        "cmd_ax_peak_time_s": round(cmd_peak_t, 3),
        "odom_ax_peak_time_s": round(odom_peak_t, 3),
        "cmd_ax_peak_minus_ref_s": round(cmd_peak_t - ref_peak_t, 3)
        if math.isfinite(cmd_peak_t) and math.isfinite(ref_peak_t)
        else float("nan"),
        "odom_ax_peak_minus_ref_s": round(odom_peak_t - ref_peak_t, 3)
        if math.isfinite(odom_peak_t) and math.isfinite(ref_peak_t)
        else float("nan"),
        "ref_ax_peak_value": round(ref_peak_value, 3),
        "cmd_ax_peak_value": round(cmd_peak_value, 3),
        "odom_ax_peak_value": round(odom_peak_value, 3),
    }
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--csv", required=True)
    parser.add_argument("--grid-dt", type=float, default=0.02)
    parser.add_argument("--lag-window-s", type=float, default=1.0)
    args = parser.parse_args()

    rows = [analyze_bag(path, args.grid_dt, args.lag_window_s) for path in args.bags]
    os.makedirs(os.path.dirname(args.csv), exist_ok=True)
    with open(args.csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    for row in rows:
        print(
            f"{row['bag']}: "
            f"ref_ax_p95={row['ref_ax_topic_abs_p95']} "
            f"cmd_ax_p95={row['cmd_ax_grid_abs_p95']} "
            f"odom_ax_p95={row['odom_ax_grid_abs_p95']} "
            f"cmd/ref={row['cmd_ax_to_ref_ratio']} "
            f"odom/ref={row['odom_ax_to_ref_ratio']} "
            f"v_corr(ref,cmd)={row['cmd_v_best_corr']} "
            f"v_corr(ref,odom)={row['odom_v_best_corr']}"
        )
    print(f"csv: {args.csv}")


if __name__ == "__main__":
    main()
