#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Analyze 2026-05-13 model-truth bags against red-liquid RGB truth."""

import argparse
import bisect
import csv
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import rosbag


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", default="/data/a/slosh_bags/real/20260513_model_truth")
    parser.add_argument("--visual-dir", default="docs/Claude/分析数据/20260513_model_truth_analysis/visual_debug")
    parser.add_argument("--out-dir", default="docs/Claude/分析数据/20260513_model_truth_analysis")
    parser.add_argument("--max-gap-sec", type=float, default=0.15)
    parser.add_argument("--sign-eps-mm", type=float, default=0.05)
    parser.add_argument("--thresholds-mm", default="0.5,1.0,1.5,2.0,3.0,4.0,6.0")
    return parser.parse_args()


def f(value, default=math.nan):
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
    mx = sum(x for x, _ in pairs) / len(pairs)
    my = sum(y for _, y in pairs) / len(pairs)
    dx = [x - mx for x, _ in pairs]
    dy = [y - my for _, y in pairs]
    sx = math.sqrt(sum(x * x for x in dx))
    sy = math.sqrt(sum(y * y for y in dy))
    if sx <= 1e-12 or sy <= 1e-12:
        return math.nan
    return sum(x * y for x, y in zip(dx, dy)) / (sx * sy)


def trapz(values, times):
    if len(values) < 2:
        return math.nan
    total = 0.0
    for idx in range(1, len(values)):
        dt = times[idx] - times[idx - 1]
        if dt > 0.0:
            total += 0.5 * (values[idx] + values[idx - 1]) * dt
    return total


def interp(series, ts, max_gap_sec):
    if not series:
        return math.nan
    times = [item[0] for item in series]
    values = [item[1] for item in series]
    if ts < times[0] or ts > times[-1]:
        return math.nan
    idx = bisect.bisect_left(times, ts)
    if idx <= 0:
        return values[0] if abs(times[0] - ts) <= max_gap_sec else math.nan
    if idx >= len(times):
        return values[-1] if abs(times[-1] - ts) <= max_gap_sec else math.nan
    t0, v0 = times[idx - 1], values[idx - 1]
    t1, v1 = times[idx], values[idx]
    if t1 <= t0 or (t1 - t0) > max_gap_sec:
        return math.nan
    ratio = (ts - t0) / (t1 - t0)
    return v0 * (1.0 - ratio) + v1 * ratio


def nearest_dt(series, ts):
    if not series:
        return math.nan
    times = [item[0] for item in series]
    idx = bisect.bisect_left(times, ts)
    candidates = []
    if idx < len(times):
        candidates.append(abs(times[idx] - ts))
    if idx > 0:
        candidates.append(abs(times[idx - 1] - ts))
    return min(candidates) if candidates else math.nan


def sign(value, eps):
    if not math.isfinite(value) or abs(value) < eps:
        return 0
    return 1 if value > 0.0 else -1


def fmt(value):
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return "{:.6g}".format(value) if math.isfinite(value) else "nan"
    return str(value)


def read_csv(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def stringify(rows):
    return [{key: fmt(value) for key, value in row.items()} for row in rows]


def condition_from_bag(path):
    match = re.search(r"modeltruth_(.+)\.bag$", path.name)
    if match:
        return match.group(1)
    return path.stem


def read_visual(path):
    rows = read_csv(path)
    series = []
    max_series = []
    left0 = percentile([f(row.get("h_mm_left")) for row in rows[:30]], 50.0)
    center0 = percentile([f(row.get("h_mm_center")) for row in rows[:30]], 50.0)
    right0 = percentile([f(row.get("h_mm_right")) for row in rows[:30]], 50.0)
    for row in rows:
        ts = f(row.get("stamp_sec"))
        h = f(row.get("h_mm_smooth_corr"))
        hl = f(row.get("h_mm_left"))
        hc = f(row.get("h_mm_center"))
        hr = f(row.get("h_mm_right"))
        if math.isfinite(ts) and math.isfinite(h):
            series.append((ts, abs(h)))
        vals = []
        if math.isfinite(hl) and math.isfinite(left0):
            vals.append(hl - left0)
        if math.isfinite(hc) and math.isfinite(center0):
            vals.append(hc - center0)
        if math.isfinite(hr) and math.isfinite(right0):
            vals.append(hr - right0)
        if math.isfinite(ts) and vals:
            max_series.append((ts, max(vals)))
    return series, max_series


def read_bag(path):
    series = {"height": [], "ax": [], "ay": [], "omega": [], "speed": [], "cmd": []}
    statuses = []
    with rosbag.Bag(path) as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        topics = ["/slosh/height", "/slosh/ax_est", "/slosh/ay_est", "/odom", "/cmd_vel", "/mpc_status"]
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = stamp.to_sec()
            if topic == "/slosh/height":
                series["height"].append((ts, abs(float(msg.data)) * 1000.0))
            elif topic == "/slosh/ax_est":
                series["ax"].append((ts, float(msg.data)))
            elif topic == "/slosh/ay_est":
                series["ay"].append((ts, float(msg.data)))
            elif topic == "/odom":
                vx = float(msg.twist.twist.linear.x)
                vy = float(msg.twist.twist.linear.y)
                series["speed"].append((ts, math.hypot(vx, vy)))
                series["omega"].append((ts, float(msg.twist.twist.angular.z)))
            elif topic == "/cmd_vel":
                series["cmd"].append((ts, max(abs(float(msg.linear.x)), abs(float(msg.angular.z)))))
            elif topic == "/mpc_status":
                statuses.append((ts, str(getattr(msg, "data", ""))))
    return series, bag_start, bag_end, statuses


def analysis_window(series, bag_start, bag_end, statuses):
    tracking = [ts for ts, state in statuses if state == "TRACKING"]
    if tracking:
        return min(tracking), max(tracking) + 2.0, "mpc_status_TRACKING"
    moving_times = []
    for key, threshold in (("cmd", 0.02), ("speed", 0.03), ("omega", 0.03), ("ax", 0.12), ("ay", 0.12)):
        for ts, value in series.get(key, []):
            if abs(value) > threshold:
                moving_times.append(ts)
    if moving_times:
        return max(bag_start, min(moving_times) - 0.5), min(bag_end, max(moving_times) + 2.0), "motion_detected"
    return bag_start, bag_end, "full_bag_no_motion"


def pair_metrics(visual, model, t0, t1, max_gap_sec):
    times = []
    vis = []
    mod = []
    dts = []
    for ts, value in visual:
        if ts < t0 or ts > t1:
            continue
        mv = interp(model, ts, max_gap_sec)
        if not math.isfinite(mv):
            continue
        times.append(ts)
        vis.append(value)
        mod.append(mv)
        dts.append(nearest_dt(model, ts) * 1000.0)
    if not vis:
        return None
    diff = [m - v for m, v in zip(mod, vis)]
    denom = trapz(mod, times)
    gamma = 100.0 * trapz(diff, times) / denom if math.isfinite(denom) and abs(denom) > 1e-12 else math.nan
    vp95 = percentile(vis, 95.0)
    mp95 = percentile(mod, 95.0)
    vpeak = max(vis)
    mpeak = max(mod)
    vrms = rms(vis)
    mrms = rms(mod)
    under = [max(0.0, v - m) for m, v in zip(mod, vis)]
    return {
        "paired_times": times,
        "paired_visual": vis,
        "paired_model": mod,
        "paired_samples": len(vis),
        "pair_dt_median_ms": percentile(dts, 50.0),
        "pair_dt_p95_ms": percentile(dts, 95.0),
        "gamma_model_pct": gamma,
        "rmse_mm": rms(diff),
        "corr": corr(mod, vis),
        "visual_peak_mm": vpeak,
        "model_peak_mm": mpeak,
        "visual_p95_mm": vp95,
        "model_p95_mm": mp95,
        "visual_rms_mm": vrms,
        "model_rms_mm": mrms,
        "e_peak_mm": mpeak - vpeak,
        "e_p95_mm": mp95 - vp95,
        "e_rms_mm": mrms - vrms,
        "U_peak_mm": max(0.0, vpeak - mpeak),
        "U_p95_mm": max(0.0, vp95 - mp95),
        "U_max_mm": max(under) if under else math.nan,
        "under_ratio": sum(1 for u in under if u > 0.0) / len(under) if under else math.nan,
    }


def threshold_rows(condition, paired, thresholds):
    out = []
    vis = paired["paired_visual"]
    mod = paired["paired_model"]
    for tau in thresholds:
        truth = [v > tau for v in vis]
        true_high = sum(1 for item in truth if item)
        detected = sum(1 for v, m in zip(vis, mod) if v > tau and m > tau)
        false_safe = sum(1 for v, m in zip(vis, mod) if v > tau and m <= tau)
        out.append({
            "condition": condition,
            "tau_mm": tau,
            "true_high_count": true_high,
            "model_detect_count": detected,
            "false_safe_count": false_safe,
            "recall": detected / true_high if true_high else math.nan,
        })
    return out


def summarize_excitation(series, t0, t1):
    def window_values(key, abs_value=True):
        values = []
        for ts, value in series.get(key, []):
            if t0 <= ts <= t1:
                values.append(abs(value) if abs_value else value)
        return values

    ax = window_values("ax")
    ay = window_values("ay")
    omega = window_values("omega")
    speed = window_values("speed")
    return {
        "ax_abs_p95": percentile(ax, 95.0),
        "ax_abs_peak": max(ax) if ax else math.nan,
        "ay_abs_p95": percentile(ay, 95.0),
        "ay_abs_peak": max(ay) if ay else math.nan,
        "omega_abs_p95": percentile(omega, 95.0),
        "omega_abs_peak": max(omega) if omega else math.nan,
        "speed_p95": percentile(speed, 95.0),
        "speed_peak": max(speed) if speed else math.nan,
    }


def plot_overlay(row, out_path):
    fig, ax = plt.subplots(figsize=(9.5, 3.8))
    t0 = row["paired_times"][0]
    times = [ts - t0 for ts in row["paired_times"]]
    ax.plot(times, row["paired_visual"], color="#d62728", lw=1.4, label="RGB |h_smooth_corr|")
    ax.plot(times, row["paired_model"], color="#1f77b4", lw=1.1, label="/slosh/height")
    ax.set_xlabel("seconds from analysis window start")
    ax.set_ylabel("mm")
    ax.set_title(row["condition"])
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_summary(rows, out_path):
    labels = [row["condition"] for row in rows]
    x = list(range(len(rows)))
    width = 0.36
    fig, ax = plt.subplots(figsize=(9.5, 4.4))
    ax.bar([i - width / 2 for i in x], [row["visual_p95_mm"] for row in rows], width, label="RGB p95", color="#d62728", alpha=0.85)
    ax.bar([i + width / 2 for i in x], [row["model_p95_mm"] for row in rows], width, label="/slosh/height p95", color="#1f77b4", alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=25, ha="right")
    ax.set_ylabel("mm")
    ax.set_title("20260513 model-truth p95: RGB vs /slosh/height")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def rank_pairs(rows, group_name, include_fn, sign_eps):
    selected = [row for row in rows if include_fn(row["condition"])]
    out = []
    for idx in range(len(selected)):
        for jdx in range(idx + 1, len(selected)):
            a = selected[idx]
            b = selected[jdx]
            for metric, key in (("p95", "p95_mm"), ("peak", "peak_mm"), ("rms", "rms_mm")):
                md = b["model_" + key] - a["model_" + key]
                vd = b["visual_" + key] - a["visual_" + key]
                ms = sign(md, sign_eps)
                vs = sign(vd, sign_eps)
                verdict = "tie" if ms == 0 or vs == 0 else ("agree" if ms == vs else "disagree")
                out.append({
                    "group": group_name,
                    "metric": metric,
                    "pair": "{}-{}".format(b["condition"], a["condition"]),
                    "model_diff_mm": md,
                    "visual_diff_mm": vd,
                    "verdict": verdict,
                })
    return out


def build_report(out_dir, rows, rank_rows, threshold_rows_all, note):
    p95_pairs = [row for row in rank_rows if row["metric"] == "p95" and row["verdict"] != "tie"]
    p95_agree = sum(1 for row in p95_pairs if row["verdict"] == "agree")
    arank = p95_agree / len(p95_pairs) if p95_pairs else math.nan
    worst_u = max((row["U_max_mm"] for row in rows), default=math.nan)
    finite_gamma = [row["gamma_model_pct"] for row in rows if math.isfinite(row["gamma_model_pct"])]
    mean_gamma = sum(finite_gamma) / len(finite_gamma) if finite_gamma else math.nan
    pair_dt_p95 = percentile([row["pair_dt_p95_ms"] for row in rows], 95.0)
    md = [
        "# 2026-05-13 模型真值包 /slosh/height 与 RGB 保真度分析",
        "",
        "## 数据范围",
        "",
        "```text",
        note,
        "不包含 RAW / GeoRef / OSCRS 实物策略对比，因此不做 OSCRS 策略有效性结论。",
        "RGB |h_smooth_corr| 是实物液面主指标；/slosh/height 只作为模型侧信号。",
        "```",
        "",
        "## 汇总结论",
        "",
        "```text",
        "bags={}".format(len(rows)),
        "pair_dt_p95_ms_over_bags={}".format(fmt(pair_dt_p95)),
        "mean_gamma_model_pct={}".format(fmt(mean_gamma)),
        "worst_U_max_mm={}".format(fmt(worst_u)),
        "weak_A_rank_p95={}".format(fmt(arank)),
        "static is treated as visual-noise baseline; its model integral is zero, so gamma is undefined.",
        "```",
        "",
        "解释：",
        "",
        "```text",
        "pair_dt_p95_ms 用于判断时间配对质量，<=80 ms 可接受。",
        "gamma_model_pct < 0 表示模型积分意义上低估 RGB 真值。",
        "U_max_mm 表示局部最大低估量。",
        "weak_A_rank_p95 只来自 straight low/mid/high 与 turn left/right 这类动作弱排序，不是 OSCRS 候选选择保真度。",
        "```",
        "",
        "## 单包指标",
        "",
        "| condition | window | visual_p95 | model_p95 | e_p95 | U_p95 | visual_peak | model_peak | gamma% | corr | U_max | ax_p95 | ay_p95 | omega_p95 |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        md.append(
            "| {condition} | {window_source} | {visual_p95_mm} | {model_p95_mm} | {e_p95_mm} | {U_p95_mm} | {visual_peak_mm} | {model_peak_mm} | {gamma_model_pct} | {corr} | {U_max_mm} | {ax_abs_p95} | {ay_abs_p95} | {omega_abs_p95} |".format(
                **{key: fmt(value) for key, value in row.items()}
            )
        )
    md.extend([
        "",
        "## 弱排序诊断",
        "",
        "| group | metric | pair | model_diff_mm | visual_diff_mm | verdict |",
        "|---|---|---|---:|---:|---|",
    ])
    for row in rank_rows:
        md.append("| {group} | {metric} | {pair} | {model_diff_mm} | {visual_diff_mm} | {verdict} |".format(**{key: fmt(value) for key, value in row.items()}))
    md.extend([
        "",
        "## Threshold Sweep",
        "",
        "完整结果见 `model_gate_fidelity_threshold_sweep.csv`。这里列出每个阈值的全包汇总 recall：",
        "",
        "| tau_mm | true_high_count | model_detect_count | false_safe_count | recall |",
        "|---:|---:|---:|---:|---:|",
    ])
    by_tau = {}
    for row in threshold_rows_all:
        tau = row["tau_mm"]
        item = by_tau.setdefault(tau, {"true": 0, "detect": 0, "false": 0})
        item["true"] += int(row["true_high_count"])
        item["detect"] += int(row["model_detect_count"])
        item["false"] += int(row["false_safe_count"])
    for tau, item in sorted(by_tau.items()):
        recall = item["detect"] / item["true"] if item["true"] else math.nan
        md.append("| {} | {} | {} | {} | {} |".format(fmt(tau), item["true"], item["detect"], item["false"], fmt(recall)))
    md.extend([
        "",
        "## 输出文件",
        "",
        "```text",
        "model_truth_fidelity_summary.csv",
        "model_truth_rank_pairs.csv",
        "model_gate_fidelity_threshold_sweep.csv",
        "plots/model_vs_rgb_p95_summary.png",
        "plots/*_model_vs_rgb_overlay.png",
        "```",
    ])
    (out_dir / "MODEL_TRUTH_FIDELITY_REPORT.md").write_text("\n".join(md) + "\n", encoding="utf-8")


def main():
    args = parse_args()
    bag_dir = Path(args.bag_dir)
    visual_dir = Path(args.visual_dir)
    out_dir = Path(args.out_dir)
    plot_dir = out_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)
    thresholds = [float(item.strip()) for item in args.thresholds_mm.split(",") if item.strip()]

    visual_by_stem = {path.name.replace("_red_top.csv", ""): path for path in visual_dir.glob("*/*_red_top.csv")}
    rows = []
    threshold_all = []
    overlay_rows = []
    for bag in sorted(bag_dir.glob("*.bag")):
        condition = condition_from_bag(bag)
        stem = bag.stem
        visual_csv = visual_by_stem.get(stem)
        if visual_csv is None:
            continue
        visual, _visual_max = read_visual(visual_csv)
        series, bag_start, bag_end, statuses = read_bag(bag)
        t0, t1, source = analysis_window(series, bag_start, bag_end, statuses)
        paired = pair_metrics(visual, series["height"], t0, t1, args.max_gap_sec)
        if paired is None:
            continue
        excitation = summarize_excitation(series, t0, t1)
        row = {
            "condition": condition,
            "bag": str(bag),
            "visual_csv": str(visual_csv),
            "window_source": source,
            "t0_rel_sec": t0 - bag_start,
            "t1_rel_sec": t1 - bag_start,
            **{key: value for key, value in paired.items() if not key.startswith("paired_")},
            **excitation,
        }
        rows.append(row)
        overlay_row = {"condition": condition, **paired}
        overlay_rows.append(overlay_row)
        plot_overlay(overlay_row, plot_dir / "{}_model_vs_rgb_overlay.png".format(condition))
        threshold_all.extend(threshold_rows(condition, paired, thresholds))

    order = {
        "static_01": 0,
        "straight_ax_low_01": 1,
        "straight_ax_mid_01": 2,
        "straight_ax_high_01": 3,
        "turn_left_mid_01": 4,
        "turn_right_mid_01": 5,
    }
    rows.sort(key=lambda item: order.get(item["condition"], 99))
    rank_rows = []
    rank_rows.extend(rank_pairs(rows, "straight_intensity", lambda name: name.startswith("straight_ax_"), args.sign_eps_mm))
    rank_rows.extend(rank_pairs(rows, "turn_left_right", lambda name: name.startswith("turn_"), args.sign_eps_mm))

    write_csv(out_dir / "model_truth_fidelity_summary.csv", stringify(rows))
    write_csv(out_dir / "model_truth_rank_pairs.csv", stringify(rank_rows))
    write_csv(out_dir / "model_gate_fidelity_threshold_sweep.csv", stringify(threshold_all))
    plot_summary(rows, plot_dir / "model_vs_rgb_p95_summary.png")
    build_report(
        out_dir,
        rows,
        rank_rows,
        threshold_all,
        "bags: {}".format(bag_dir),
    )
    print("wrote", out_dir)
    for row in rows:
        print(row["condition"], "visual_p95", fmt(row["visual_p95_mm"]), "model_p95", fmt(row["model_p95_mm"]), "gamma", fmt(row["gamma_model_pct"]))


if __name__ == "__main__":
    main()
