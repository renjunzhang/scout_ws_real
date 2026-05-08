#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compute Ferrari gamma_model (eq. 25) and gamma_opt (eq. 26) from bags.

gamma_model = integral( |h_pred - h_visual| dt ) / integral( h_pred dt )
  measures how well the slosh model matches the visual ground truth.

gamma_opt = (eta_max,Nopt - eta_max,Opt) / eta_max,Nopt
  measures the relative improvement of the optimised line over the
  non-optimised baseline. eta_max here is taken from /slosh/h_visual,
  with optional fallback to /slosh/height for sim runs without visual.
"""

import argparse
import bisect
import csv
import math
import os
import sys

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-bag", required=True,
                        help="non-optimised baseline bag (RAW or GEOREF_TUNED)")
    parser.add_argument("--optimised-bag", required=True,
                        help="optimised bag (GEOREF_OSCRS_ACTIVE or ORACLE_REPLAY)")
    parser.add_argument("--height-topic", default="/slosh/height",
                        help="model-predicted height topic")
    parser.add_argument("--visual-topic", default="/slosh/h_visual",
                        help="visual ground truth topic; falls back to height_topic if missing")
    parser.add_argument("--baseline-visual-csv", default="",
                        help="RGB_infer_from_bag *_rgb_heights.csv for the baseline bag")
    parser.add_argument("--optimised-visual-csv", default="",
                        help="RGB_infer_from_bag *_rgb_heights.csv for the optimised bag")
    parser.add_argument("--visual-csv-time-column", default="stamp_sec")
    parser.add_argument("--visual-csv-height-column", default="height_mm")
    parser.add_argument("--visual-csv-confidence-column", default="confidence")
    parser.add_argument("--visual-csv-min-confidence", type=float, default=0.0)
    parser.add_argument("--visual-csv-zero-align-window-sec", type=float, default=1.0,
                        help="Subtract median visual height in the first N seconds; 0 disables")
    parser.add_argument("--visual-csv-keep-signed", action="store_true",
                        help="Keep signed zero-aligned RGB height instead of abs(height)")
    parser.add_argument("--out-summary", default="docs/Claude/分析数据/ferrari_indices_summary.csv")
    return parser.parse_args()


def collect(bag_path, topic):
    series = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            ts = stamp.to_sec() if stamp is not None else t.to_sec()
            value = float(getattr(msg, "data", 0.0))
            series.append((ts, value))
    series.sort(key=lambda item: item[0])
    return series


def median(values):
    vals = sorted(values)
    if not vals:
        return 0.0
    mid = len(vals) // 2
    if len(vals) % 2:
        return vals[mid]
    return 0.5 * (vals[mid - 1] + vals[mid])


def collect_visual_csv(path, args):
    series = []
    with open(path, "r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                t = float(row[args.visual_csv_time_column])
                h_mm = float(row[args.visual_csv_height_column])
            except (KeyError, ValueError, TypeError):
                continue
            if not math.isfinite(t) or not math.isfinite(h_mm):
                continue
            conf_raw = row.get(args.visual_csv_confidence_column, "")
            try:
                conf = float(conf_raw) if conf_raw != "" else 1.0
            except ValueError:
                conf = 1.0
            if conf < args.visual_csv_min_confidence:
                continue
            series.append((t, h_mm / 1000.0))
    series.sort(key=lambda item: item[0])
    if not series:
        return []

    if args.visual_csv_zero_align_window_sec > 0.0:
        t0 = series[0][0]
        window = [
            value for t, value in series
            if t - t0 <= args.visual_csv_zero_align_window_sec
        ]
        offset = median(window)
        series = [(t, value - offset) for t, value in series]
    if not args.visual_csv_keep_signed:
        series = [(t, abs(value)) for t, value in series]
    return series


def integrate_abs_diff(pred, vis):
    if not pred or not vis:
        return float("nan"), float("nan")
    vis_times = [t for t, _ in vis]
    vis_values = [v for _, v in vis]
    abs_diff_integral = 0.0
    pred_integral = 0.0
    prev_t = None
    prev_diff = None
    prev_pred = None
    for t, p in pred:
        idx = bisect.bisect_left(vis_times, t)
        if idx == 0:
            v = vis_values[0]
        elif idx >= len(vis_values):
            v = vis_values[-1]
        else:
            t0, t1 = vis_times[idx - 1], vis_times[idx]
            r = (t - t0) / max(1e-9, (t1 - t0))
            v = vis_values[idx - 1] * (1.0 - r) + vis_values[idx] * r
        diff = abs(p - v)
        if prev_t is not None:
            dt = t - prev_t
            abs_diff_integral += 0.5 * (diff + prev_diff) * dt
            pred_integral += 0.5 * (abs(p) + abs(prev_pred)) * dt
        prev_t = t
        prev_diff = diff
        prev_pred = p
    if pred_integral <= 0:
        return float("nan"), abs_diff_integral
    return abs_diff_integral / pred_integral, abs_diff_integral


def gamma_model(bag_path, height_topic, visual_topic, visual_csv, args):
    pred = collect(bag_path, height_topic)
    if visual_csv:
        vis = collect_visual_csv(visual_csv, args)
        visual_used = visual_csv
    else:
        vis = collect(bag_path, visual_topic)
        visual_used = visual_topic
    if not vis:
        sys.stderr.write(f"warn: {bag_path} missing visual data; falling back to model-only\n")
        return float("nan"), 0.0, "<missing>"
    gm, _ = integrate_abs_diff(pred, vis)
    return gm, len(vis), visual_used


def eta_max(bag_path, topic):
    series = collect(bag_path, topic)
    return max((v for _, v in series), default=float("nan"))


def eta_max_visual(bag_path, height_topic, visual_topic, visual_csv, args):
    if visual_csv:
        series = collect_visual_csv(visual_csv, args)
        if series:
            return max((v for _, v in series), default=float("nan")), visual_csv
    if collect_first(bag_path, visual_topic):
        return eta_max(bag_path, visual_topic), visual_topic
    return eta_max(bag_path, height_topic), height_topic


def main():
    args = parse_args()
    eta_n, eta_n_src = eta_max_visual(
        args.baseline_bag, args.height_topic, args.visual_topic, args.baseline_visual_csv, args)
    eta_o, eta_o_src = eta_max_visual(
        args.optimised_bag, args.height_topic, args.visual_topic, args.optimised_visual_csv, args)
    gamma_opt = (eta_n - eta_o) / eta_n if eta_n and eta_n > 1e-9 else float("nan")
    gm_baseline, n_b, src_b = gamma_model(
        args.baseline_bag, args.height_topic, args.visual_topic, args.baseline_visual_csv, args)
    gm_optimised, n_o, src_o = gamma_model(
        args.optimised_bag, args.height_topic, args.visual_topic, args.optimised_visual_csv, args)
    rows = [
        {
            "bag": os.path.basename(args.baseline_bag),
            "role": "baseline",
            "eta_max": f"{eta_n:.6g}",
            "gamma_model": f"{gm_baseline:.4f}" if gm_baseline == gm_baseline else "nan",
            "visual_pairs": n_b,
            "eta_source": eta_n_src,
            "visual_source": src_b,
        },
        {
            "bag": os.path.basename(args.optimised_bag),
            "role": "optimised",
            "eta_max": f"{eta_o:.6g}",
            "gamma_model": f"{gm_optimised:.4f}" if gm_optimised == gm_optimised else "nan",
            "visual_pairs": n_o,
            "eta_source": eta_o_src,
            "visual_source": src_o,
        },
        {
            "bag": "<pair>",
            "role": "gamma_opt",
            "eta_max": "-",
            "gamma_model": f"{gamma_opt:.4f}" if gamma_opt == gamma_opt else "nan",
            "visual_pairs": "-",
            "eta_source": "-",
            "visual_source": "-",
        },
    ]
    print("Ferrari eq. 25/26 indices:")
    for r in rows:
        print(f"  {r['role']:>10s} bag={r['bag']:>40s} eta_max={r['eta_max']} "
              f"gamma_model={r['gamma_model']} pairs={r['visual_pairs']}")
    if args.out_summary:
        os.makedirs(os.path.dirname(args.out_summary) or ".", exist_ok=True)
        with open(args.out_summary, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"summary: {args.out_summary}")


def collect_first(bag_path, topic):
    try:
        with rosbag.Bag(bag_path) as bag:
            for _ in bag.read_messages(topics=[topic]):
                return True
    except rosbag.ROSBagException:
        return False
    return False


if __name__ == "__main__":
    main()
