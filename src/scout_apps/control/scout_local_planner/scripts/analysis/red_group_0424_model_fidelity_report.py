#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate model-vs-RGB fidelity plots/reports for 2026-04-24 red-liquid data."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import rosbag

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


METRICS = (
    ("peak", "model_peak_mm", "visual_peak_mm"),
    ("p95", "model_p95_mm", "visual_p95_mm"),
    ("rms", "model_rms_mm", "visual_rms_mm"),
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".tmp_red_group_compare_0424")
    parser.add_argument("--out-dir", default="docs/Claude/分析数据/0424_model_fidelity_20260513")
    parser.add_argument("--max-gap-sec", type=float, default=0.15)
    parser.add_argument("--sign-eps-mm", type=float, default=0.05)
    parser.add_argument("--eta-lim-mm", type=float, default=25.0)
    parser.add_argument("--thresholds-mm", default="0.5,1.0,1.5,2.0,3.0,4.0")
    return parser.parse_args()


def read_csv(path):
    with open(path, "r", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def as_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def percentile(values, pct):
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return math.nan
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    ratio = pos - lo
    return vals[lo] * (1.0 - ratio) + vals[hi] * ratio


def rms(values):
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return math.nan
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def corr(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return math.nan
    xvals = [p[0] for p in pairs]
    yvals = [p[1] for p in pairs]
    mx = sum(xvals) / len(xvals)
    my = sum(yvals) / len(yvals)
    dx = [x - mx for x in xvals]
    dy = [y - my for y in yvals]
    sx = math.sqrt(sum(x * x for x in dx))
    sy = math.sqrt(sum(y * y for y in dy))
    if sx <= 1e-12 or sy <= 1e-12:
        return math.nan
    return sum(x * y for x, y in zip(dx, dy)) / (sx * sy)


def trapz(values, times):
    pairs = [(t, v) for t, v in zip(times, values) if math.isfinite(t) and math.isfinite(v)]
    if len(pairs) < 2:
        return math.nan
    total = 0.0
    for (t0, v0), (t1, v1) in zip(pairs[:-1], pairs[1:]):
        if t1 > t0:
            total += 0.5 * (v0 + v1) * (t1 - t0)
    return total


def interp(series, ts, max_gap_sec):
    if not series:
        return math.nan, math.nan
    times = [item[0] for item in series]
    values = [item[1] for item in series]
    if ts < times[0] or ts > times[-1]:
        return math.nan, math.nan
    idx = bisect.bisect_left(times, ts)
    if idx <= 0:
        dt = abs(times[0] - ts)
        return (values[0], dt) if dt <= max_gap_sec else (math.nan, math.nan)
    if idx >= len(times):
        dt = abs(times[-1] - ts)
        return (values[-1], dt) if dt <= max_gap_sec else (math.nan, math.nan)
    t0, v0 = times[idx - 1], values[idx - 1]
    t1, v1 = times[idx], values[idx]
    if t1 <= t0 or (t1 - t0) > max_gap_sec:
        return math.nan, math.nan
    ratio = (ts - t0) / (t1 - t0)
    nearest_dt = min(abs(ts - t0), abs(t1 - ts))
    return v0 * (1.0 - ratio) + v1 * ratio, nearest_dt


def read_visual_series(path):
    rows = read_csv(path)
    out = []
    for row in rows:
        t = as_float(row.get("t_rel_sec"))
        h = as_float(row.get("h_smooth_corr"))
        if math.isfinite(t) and math.isfinite(h):
            out.append((t, abs(h)))
    return out


def read_bag_series(path):
    height = []
    tracking_start = None
    tracking_end = None
    with rosbag.Bag(path) as bag:
        bag_start = bag.get_start_time()
        last_tracking = False
        for topic, msg, stamp in bag.read_messages(topics=["/slosh/height", "/mpc_status"]):
            t_rel = stamp.to_sec() - bag_start
            if topic == "/slosh/height":
                height.append((t_rel, abs(float(msg.data)) * 1000.0))
            elif topic == "/mpc_status":
                state = str(getattr(msg, "data", ""))
                is_tracking = state == "TRACKING"
                if is_tracking and tracking_start is None:
                    tracking_start = t_rel
                if last_tracking and not is_tracking and tracking_end is None:
                    tracking_end = t_rel
                last_tracking = is_tracking
    if tracking_start is None:
        tracking_start = 0.0
    if tracking_end is None and height:
        tracking_end = height[-1][0]
    return height, tracking_start, tracking_end


def pair_series(visual, model, t0, t1, max_gap_sec):
    times = []
    vis_vals = []
    model_vals = []
    dts = []
    for t, v in visual:
        if t < t0 or t > t1:
            continue
        mv, dt = interp(model, t, max_gap_sec)
        if not math.isfinite(mv):
            continue
        times.append(t)
        vis_vals.append(v)
        model_vals.append(mv)
        dts.append(dt * 1000.0)
    return times, vis_vals, model_vals, dts


def sign(value, eps):
    if not math.isfinite(value) or abs(value) < eps:
        return 0
    return 1 if value > 0 else -1


def row_metrics(times, visual, model, dts):
    diff = [m - v for m, v in zip(model, visual)]
    denom = trapz(model, times)
    gamma = math.nan
    if math.isfinite(denom) and abs(denom) > 1e-12:
        gamma = 100.0 * trapz(diff, times) / denom
    visual_p95 = percentile(visual, 95.0)
    model_p95 = percentile(model, 95.0)
    visual_peak = max(visual) if visual else math.nan
    model_peak = max(model) if model else math.nan
    visual_rms = rms(visual)
    model_rms = rms(model)
    under = [max(0.0, v - m) for m, v in zip(model, visual)]
    return {
        "paired_samples": len(times),
        "pair_dt_median_ms": percentile(dts, 50.0),
        "pair_dt_p95_ms": percentile(dts, 95.0),
        "gamma_model_pct": gamma,
        "rmse_mm": rms(diff),
        "corr": corr(model, visual),
        "visual_p95_mm": visual_p95,
        "model_p95_mm": model_p95,
        "e_p95_mm": model_p95 - visual_p95,
        "U_p95_mm": max(0.0, visual_p95 - model_p95),
        "visual_peak_mm": visual_peak,
        "model_peak_mm": model_peak,
        "e_peak_mm": model_peak - visual_peak,
        "U_peak_mm": max(0.0, visual_peak - model_peak),
        "visual_rms_mm": visual_rms,
        "model_rms_mm": model_rms,
        "e_rms_mm": model_rms - visual_rms,
        "U_rms_mm": max(0.0, visual_rms - model_rms),
        "U_max_mm": max(under) if under else math.nan,
        "under_ratio": sum(1 for u in under if u > 0.0) / len(under) if under else math.nan,
    }


def fmt(value):
    if isinstance(value, int):
        return str(value)
    if not isinstance(value, float):
        return str(value)
    return "{:.6g}".format(value) if math.isfinite(value) else "nan"


def metric_rows_to_strings(rows):
    out = []
    for row in rows:
        out.append({key: fmt(value) for key, value in row.items()})
    return out


def build_entries(root):
    visual_summary = read_csv(root / "visual_metric_summary.csv")
    with (root / "0424_groups_manifest_expanded.json").open("r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    bag_by_key = {
        (group, label): bag
        for group, items in manifest.items()
        for label, bag in items
    }
    entries = []
    for row in visual_summary:
        group = row["group"]
        condition = row["condition"]
        bag = bag_by_key.get((group, condition))
        if not bag:
            continue
        entries.append({
            "group": group,
            "block": row.get("block", ""),
            "condition": condition,
            "bag": bag,
            "visual_csv": row["csv"],
        })
    return entries


def plot_group(group, items, out_path):
    if not items:
        return
    n = len(items)
    fig, axes = plt.subplots(n, 1, figsize=(10, max(2.4 * n, 3.2)), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, item in zip(axes, items):
        ax.plot(item["paired_times"], item["paired_visual"], color="#d62728", lw=1.3, label="RGB |h_smooth_corr|")
        ax.plot(item["paired_times"], item["paired_model"], color="#1f77b4", lw=1.1, label="/slosh/height")
        ax.set_ylabel("mm")
        ax.set_title(item["condition"], loc="left", fontsize=10)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t_rel_sec (bag start; filtered to tracking_start..tracking_end+2s)")
    fig.suptitle(group + " model vs RGB truth", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_scatter(rows, metric, model_key, visual_key, out_path):
    xs = [as_float(row[visual_key]) for row in rows]
    ys = [as_float(row[model_key]) for row in rows]
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if not pairs:
        return
    fig, ax = plt.subplots(figsize=(5.8, 5.2))
    ax.scatter([p[0] for p in pairs], [p[1] for p in pairs], s=28, alpha=0.75)
    lim = max(max(p[0] for p in pairs), max(p[1] for p in pairs), 1e-6)
    ax.plot([0.0, lim], [0.0, lim], color="black", lw=1.0, alpha=0.6)
    ax.set_xlabel("RGB visual {} (mm)".format(metric))
    ax.set_ylabel("/slosh/height {} (mm)".format(metric))
    ax.set_title("0424 model-vs-RGB {}".format(metric))
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    args = parse_args()
    root = Path(args.root)
    out_dir = Path(args.out_dir)
    plot_dir = out_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)
    thresholds = [float(v.strip()) for v in args.thresholds_mm.split(",") if v.strip()]

    entries = build_entries(root)
    fidelity_rows = []
    gate_rows = []
    plot_items_by_group = {}

    for entry in entries:
        visual = read_visual_series(entry["visual_csv"])
        model, tracking_start, tracking_end = read_bag_series(entry["bag"])
        t0 = tracking_start
        t1 = tracking_end + 2.0
        times, vis_vals, model_vals, dts = pair_series(visual, model, t0, t1, args.max_gap_sec)
        metrics = row_metrics(times, vis_vals, model_vals, dts)
        row = {
            "group": entry["group"],
            "block": entry["block"],
            "condition": entry["condition"],
            "bag": entry["bag"],
            "visual_csv": entry["visual_csv"],
            "tracking_start_sec": tracking_start,
            "tracking_end_plus_2_sec": t1,
            **metrics,
        }
        fidelity_rows.append(row)

        for tau in thresholds + [args.eta_lim_mm]:
            true_high = 0
            detected = 0
            for m, v in zip(model_vals, vis_vals):
                if v > tau:
                    true_high += 1
                    if m > tau:
                        detected += 1
            gate_rows.append({
                "group": entry["group"],
                "condition": entry["condition"],
                "tau_mm": tau,
                "true_high_count": true_high,
                "model_detect_count": detected,
                "false_safe_count": true_high - detected,
                "recall": detected / true_high if true_high else math.nan,
            })

        plot_items_by_group.setdefault(entry["group"], []).append({
            "condition": entry["condition"],
            "paired_times": times,
            "paired_visual": vis_vals,
            "paired_model": model_vals,
        })

    selection_rows = []
    grouped = {}
    for row in fidelity_rows:
        grouped.setdefault(row["group"], []).append(row)
    for group, rows in sorted(grouped.items()):
        for metric, model_key, visual_key in METRICS:
            for i in range(len(rows)):
                for j in range(i + 1, len(rows)):
                    a = rows[i]
                    b = rows[j]
                    md = as_float(b[model_key]) - as_float(a[model_key])
                    vd = as_float(b[visual_key]) - as_float(a[visual_key])
                    ms = sign(md, args.sign_eps_mm)
                    vs = sign(vd, args.sign_eps_mm)
                    if ms == 0 or vs == 0:
                        verdict = "tie"
                    elif ms == vs:
                        verdict = "agree"
                    else:
                        verdict = "disagree"
                    selection_rows.append({
                        "group": group,
                        "metric": metric,
                        "pair": "{}-{}".format(b["condition"], a["condition"]),
                        "model_diff_mm": md,
                        "visual_diff_mm": vd,
                        "verdict": verdict,
                    })

    arank_rows = []
    for metric, _, _ in METRICS:
        rows = [row for row in selection_rows if row["metric"] == metric]
        non_tie = [row for row in rows if row["verdict"] != "tie"]
        agree = sum(1 for row in non_tie if row["verdict"] == "agree")
        arank_rows.append({
            "metric": metric,
            "rank_pairs": len(non_tie),
            "agree": agree,
            "disagree": sum(1 for row in non_tie if row["verdict"] == "disagree"),
            "ties": sum(1 for row in rows if row["verdict"] == "tie"),
            "A_rank": agree / len(non_tie) if non_tie else math.nan,
        })

    gate_summary_rows = []
    for tau in sorted(set(as_float(row["tau_mm"]) for row in gate_rows)):
        rows = [row for row in gate_rows if abs(as_float(row["tau_mm"]) - tau) <= 1e-9]
        true_high = sum(int(row["true_high_count"]) for row in rows)
        detected = sum(int(row["model_detect_count"]) for row in rows)
        false_safe = sum(int(row["false_safe_count"]) for row in rows)
        gate_summary_rows.append({
            "tau_mm": tau,
            "true_high_count": true_high,
            "model_detect_count": detected,
            "false_safe_count": false_safe,
            "recall": detected / true_high if true_high else math.nan,
        })

    write_csv(out_dir / "model_fidelity_summary.csv", metric_rows_to_strings(fidelity_rows))
    write_csv(out_dir / "model_selection_fidelity.csv", metric_rows_to_strings(selection_rows))
    write_csv(out_dir / "A_rank_overall.csv", metric_rows_to_strings(arank_rows))
    write_csv(out_dir / "model_gate_fidelity_threshold_sweep.csv", metric_rows_to_strings(gate_rows))
    write_csv(out_dir / "model_gate_fidelity_threshold_summary.csv", metric_rows_to_strings(gate_summary_rows))

    for group, items in sorted(plot_items_by_group.items()):
        plot_group(group, items, plot_dir / "{}_model_vs_rgb_overlay.png".format(group))
    for metric, model_key, visual_key in METRICS:
        plot_scatter(fidelity_rows, metric, model_key, visual_key, plot_dir / "scatter_{}.png".format(metric))

    mean_gamma = sum(as_float(row["gamma_model_pct"]) for row in fidelity_rows if math.isfinite(as_float(row["gamma_model_pct"])))
    gamma_count = sum(1 for row in fidelity_rows if math.isfinite(as_float(row["gamma_model_pct"])))
    mean_gamma = mean_gamma / gamma_count if gamma_count else math.nan
    worst_u_max = max((as_float(row["U_max_mm"]) for row in fidelity_rows), default=math.nan)
    p95_arank = next((row for row in arank_rows if row["metric"] == "p95"), {})
    pair_p95 = percentile([as_float(row["pair_dt_p95_ms"]) for row in fidelity_rows], 95.0)

    md = [
        "# 2026-04-24 Red Group Model Fidelity",
        "",
        "## Overall",
        "",
        "| metric | value |",
        "|---|---:|",
        "| bags | {} |".format(len(fidelity_rows)),
        "| pair_dt_p95_ms_over_bags | {} |".format(fmt(pair_p95)),
        "| mean_gamma_model_pct | {} |".format(fmt(mean_gamma)),
        "| worst_U_max_mm | {} |".format(fmt(worst_u_max)),
        "",
        "## A Rank",
        "",
        "| metric | rank_pairs | agree | disagree | ties | A_rank |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in arank_rows:
        md.append("| {metric} | {rank_pairs} | {agree} | {disagree} | {ties} | {A_rank} |".format(
            metric=row["metric"],
            rank_pairs=row["rank_pairs"],
            agree=row["agree"],
            disagree=row["disagree"],
            ties=row["ties"],
            A_rank=fmt(row["A_rank"]),
        ))
    md.extend([
        "",
        "## Threshold Sweep Recall",
        "",
        "| tau_mm | true_high_count | model_detect_count | false_safe_count | recall |",
        "|---:|---:|---:|---:|---:|",
    ])
    for row in gate_summary_rows:
        md.append("| {tau_mm} | {true_high_count} | {model_detect_count} | {false_safe_count} | {recall} |".format(
            tau_mm=fmt(row["tau_mm"]),
            true_high_count=row["true_high_count"],
            model_detect_count=row["model_detect_count"],
            false_safe_count=row["false_safe_count"],
            recall=fmt(row["recall"]),
        ))
    md.extend([
        "",
        "## Interpretation",
        "",
        "- RGB visual `|h_smooth_corr|` is treated as the physical liquid-height truth.",
        "- `/slosh/height` is evaluated as a model/observer signal.",
        "- These are old MPC/controller bags, so the result is model-fidelity evidence only, not OSCRS effectiveness evidence.",
        "- Main OSCRS selection proxy uses p95 A_rank. In this run p95 A_rank is `{}`.".format(fmt(as_float(p95_arank.get("A_rank", "nan")))),
        "",
        "## Key Files",
        "",
        "- `model_fidelity_summary.csv`",
        "- `model_selection_fidelity.csv`",
        "- `model_gate_fidelity_threshold_sweep.csv`",
        "- `model_gate_fidelity_threshold_summary.csv`",
        "- `plots/*_model_vs_rgb_overlay.png`",
        "- `plots/scatter_p95.png`",
    ])
    (out_dir / "MODEL_FIDELITY_REPORT.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    print("wrote", out_dir)
    print("bags", len(fidelity_rows))
    for row in arank_rows:
        print(row)


if __name__ == "__main__":
    main()
