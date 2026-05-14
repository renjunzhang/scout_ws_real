#!/usr/bin/env python3
"""Extract MPC cost contribution from rosbag and generate plots.

Input topic:
    /mpc/cost_breakdown

Expected Float32MultiArray fields:
    0  J_total
    1  J_lag
    2  J_contour
    3  J_etheta
    4  J_v
    5  J_omega_ff
    6  J_control
    7  J_smooth
    8  J_slosh_eta
    9  J_slosh_eta_dot
    10 pct_lag
    11 pct_contour
    12 pct_etheta
    13 pct_v
    14 pct_omega_ff
    15 pct_control
    16 pct_smooth
    17 pct_slosh_eta
    18 pct_slosh_eta_dot
    19 pct_slosh_total
"""

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


FIELDS = [
    "J_total",
    "J_lag",
    "J_contour",
    "J_etheta",
    "J_v",
    "J_omega_ff",
    "J_control",
    "J_smooth",
    "J_slosh_eta",
    "J_slosh_eta_dot",
    "pct_lag",
    "pct_contour",
    "pct_etheta",
    "pct_v",
    "pct_omega_ff",
    "pct_control",
    "pct_smooth",
    "pct_slosh_eta",
    "pct_slosh_eta_dot",
    "pct_slosh_total",
]

PCT_FIELDS = [
    "pct_lag",
    "pct_contour",
    "pct_etheta",
    "pct_v",
    "pct_omega_ff",
    "pct_control",
    "pct_smooth",
    "pct_slosh_eta",
    "pct_slosh_eta_dot",
]

PCT_STACK_FIELDS = [
    "pct_lag",
    "pct_contour",
    "pct_etheta",
    "pct_v",
    "pct_omega_ff",
    "pct_control",
    "pct_smooth",
    "pct_slosh_eta",
    "pct_slosh_eta_dot",
]

COLORS = {
    "pct_lag": "#8d99ae",
    "pct_contour": "#2a9d8f",
    "pct_etheta": "#457b9d",
    "pct_v": "#e76f51",
    "pct_omega_ff": "#f4a261",
    "pct_control": "#6d597a",
    "pct_smooth": "#b56576",
    "pct_slosh_eta": "#7b2cbf",
    "pct_slosh_eta_dot": "#c77dff",
    "pct_slosh_total": "#5a189a",
}


def percentile(values, p):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return float("nan")
    idx = int(round((len(values) - 1) * p / 100.0))
    idx = max(0, min(len(values) - 1, idx))
    return values[idx]


def mean(values):
    values = [v for v in values if math.isfinite(v)]
    return statistics.mean(values) if values else float("nan")


def median(values):
    values = [v for v in values if math.isfinite(v)]
    return statistics.median(values) if values else float("nan")


def load_bag(path, args):
    try:
        import rosbag
    except ImportError:
        print("ERROR: rosbag import failed. Run `source /opt/ros/noetic/setup.bash` first.", file=sys.stderr)
        sys.exit(2)

    records = []
    topics = [args.cost_topic, args.status_topic]
    current_status = ""

    with rosbag.Bag(str(path), "r") as bag:
        t0 = bag.get_start_time()
        available = set(bag.get_type_and_topic_info().topics.keys())
        if args.cost_topic not in available:
            raise RuntimeError(f"{path}: missing topic {args.cost_topic}")

        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec() - t0
            if topic == args.status_topic:
                current_status = str(getattr(msg, "data", ""))
                continue
            data = list(getattr(msg, "data", []))
            if len(data) < len(FIELDS):
                continue
            if args.phase != "all" and current_status and current_status != args.phase:
                continue
            row = {"bag": path.name, "t": t, "phase": current_status}
            for key, value in zip(FIELDS, data[:len(FIELDS)]):
                row[key] = float(value)
            records.append(row)

    return records


def write_csv(records, out_path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["bag", "t", "phase"] + FIELDS)
        writer.writeheader()
        for row in records:
            writer.writerow(row)


def summarize(records):
    lines = []
    lines.append("# MPC Cost Contribution Summary")
    lines.append("")
    lines.append(f"samples: {len(records)}")
    if not records:
        return "\n".join(lines) + "\n"

    bags = sorted(set(row["bag"] for row in records))
    phases = sorted(set(row["phase"] for row in records if row["phase"]))
    lines.append(f"bags: {', '.join(bags)}")
    lines.append(f"phases: {', '.join(phases) if phases else '-'}")
    lines.append("")
    lines.append("| field | mean % | median % | p95 % | max % |")
    lines.append("|---|---:|---:|---:|---:|")
    for field in PCT_FIELDS + ["pct_slosh_total"]:
        vals = [row[field] for row in records]
        lines.append(
            f"| `{field}` | {mean(vals):.3f} | {median(vals):.3f} | "
            f"{percentile(vals, 95):.3f} | {max(vals):.3f} |"
        )

    slosh_vals = [row["pct_slosh_total"] for row in records]
    slosh_mean = mean(slosh_vals)
    lines.append("")
    if slosh_mean < 1.0:
        verdict = "晃动项在优化器内基本不可见。"
    elif slosh_mean <= 20.0:
        verdict = "晃动项有可见占比，适合进入 RGB 实物效果对比。"
    elif slosh_mean <= 40.0:
        verdict = "晃动项较强，需要同时检查 tracking 和 completion time。"
    else:
        verdict = "晃动项可能过度主导，需检查慢行、绕行或 tracking 退化。"
    lines.append(f"verdict: {verdict}")
    lines.append("")
    lines.append("说明：该摘要来自 MPC 内部预测 horizon 的 cost breakdown，不是 RGB 真实液面评价。")
    return "\n".join(lines) + "\n"


def plot_records(records, out_path, title):
    if plt is None:
        print("[WARN] matplotlib not available; plot skipped.", file=sys.stderr)
        return
    if not records:
        return

    t = [row["t"] for row in records]
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    ax = axes[0]
    for field in ["pct_v", "pct_contour", "pct_smooth", "pct_slosh_total"]:
        ax.plot(t, [row[field] for row in records], label=field, lw=1.3, color=COLORS.get(field))
    ax.set_ylabel("cost percent (%)")
    ax.set_title("Key MPC cost contribution")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", ncol=2)

    ax = axes[1]
    bottom = [0.0] * len(records)
    for field in PCT_STACK_FIELDS:
        vals = [max(0.0, row[field]) for row in records]
        ax.fill_between(t, bottom, [b + v for b, v in zip(bottom, vals)],
                        label=field, alpha=0.85, color=COLORS.get(field))
        bottom = [b + v for b, v in zip(bottom, vals)]
    ax.set_ylabel("stacked percent (%)")
    ax.set_title("Stacked cost contribution")
    ax.set_ylim(0, max(105.0, max(bottom) * 1.05))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", ncol=3, fontsize=8)

    ax = axes[2]
    ax.plot(t, [row["J_total"] for row in records], label="J_total", lw=1.2, color="#222222")
    ax.plot(t, [row["J_slosh_eta"] + row["J_slosh_eta_dot"] for row in records],
            label="J_slosh_total", lw=1.2, color=COLORS["pct_slosh_total"])
    ax.set_xlabel("time from bag start (s)")
    ax.set_ylabel("raw cost")
    ax.set_title("Raw total and slosh cost")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_bar_summary(records, out_path, title):
    if plt is None or not records:
        return
    fields = PCT_FIELDS + ["pct_slosh_total"]
    means = [mean([row[field] for row in records]) for field in fields]
    p95s = [percentile([row[field] for row in records], 95) for field in fields]

    fig, ax = plt.subplots(figsize=(12, 5))
    x = list(range(len(fields)))
    ax.bar([i - 0.18 for i in x], means, width=0.36, label="mean", color="#457b9d")
    ax.bar([i + 0.18 for i in x], p95s, width=0.36, label="p95", color="#e76f51")
    ax.set_xticks(x)
    ax.set_xticklabels(fields, rotation=35, ha="right")
    ax.set_ylabel("cost percent (%)")
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", help="Input rosbag(s)")
    parser.add_argument("--out-dir", default="", help="Output directory. Default: <first_bag_dir>/cost_breakdown_analysis")
    parser.add_argument("--cost-topic", default="/mpc/cost_breakdown")
    parser.add_argument("--status-topic", default="/mpc_status")
    parser.add_argument("--phase", default="all", help="Filter by /mpc_status, e.g. TRACKING. Default: all")
    parser.add_argument("--prefix", default="cost_contribution", help="Output filename prefix")
    return parser.parse_args()


def main():
    args = parse_args()
    bag_paths = [Path(p) for p in args.bags]
    out_dir = Path(args.out_dir) if args.out_dir else bag_paths[0].parent / "cost_breakdown_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    all_records = []
    for bag_path in bag_paths:
        records = load_bag(bag_path, args)
        print(f"{bag_path}: cost samples={len(records)}")
        all_records.extend(records)

    if not all_records:
        print("ERROR: no /mpc/cost_breakdown samples found. Record a new bag with updated record_slosh_experiment.sh.", file=sys.stderr)
        return 2

    csv_path = out_dir / f"{args.prefix}.csv"
    md_path = out_dir / f"{args.prefix}_summary.md"
    plot_path = out_dir / f"{args.prefix}_timeseries.png"
    bar_path = out_dir / f"{args.prefix}_summary_bar.png"

    write_csv(all_records, csv_path)
    md_path.write_text(summarize(all_records), encoding="utf-8")
    title = f"MPC cost contribution ({len(bag_paths)} bag(s), phase={args.phase})"
    plot_records(all_records, plot_path, title)
    plot_bar_summary(all_records, bar_path, "MPC cost contribution summary")

    print(f"CSV: {csv_path}")
    print(f"Summary: {md_path}")
    if plt is not None:
        print(f"Plot: {plot_path}")
        print(f"Bar: {bar_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
