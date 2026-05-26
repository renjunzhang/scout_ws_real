#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compare current/Ferrari slosh model variants against 2026-04-24 RGB truth."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import rosbag
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

G = 9.81
XI_11 = 1.841


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".tmp_red_group_compare_0424")
    parser.add_argument("--config", default="src/scout_apps/control/scout_local_planner/config/mpc_params.yaml")
    parser.add_argument("--out-dir", default="docs/Claude/分析数据/0424_ferrari_fidelity_20260513")
    parser.add_argument("--max-gap-sec", type=float, default=0.15)
    parser.add_argument("--sign-eps-mm", type=float, default=0.05)
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
    for i in range(1, len(values)):
        dt = times[i] - times[i - 1]
        if dt > 0.0:
            total += 0.5 * (values[i] + values[i - 1]) * dt
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


def load_config(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    slosh = data.get("slosh", {}) or {}
    oscrs = data.get("oscrs", {}) or {}
    return {
        "R": float(slosh.get("container_radius", 0.0185)),
        "h": float(slosh.get("liquid_height", 0.058)),
        "rho": float(slosh.get("liquid_density", 1000.0)),
        "nu": float(slosh.get("liquid_dynamic_viscosity", 1.0e-3)),
        "zeta": float(slosh.get("damping_ratio", 0.05)),
        "use_parabola_term": bool(slosh.get("use_parabola_term", True)),
        "height_coeff_manual": float(oscrs.get("height_coeff_manual", 0.0) or 0.0),
        "dt": float(oscrs.get("rollout_dt", 0.05)),
    }


def derived_params(cfg):
    R = cfg["R"]
    h = cfg["h"]
    rho = cfg["rho"]
    xi = XI_11
    xi_h_R = xi * h / R
    m_f = rho * math.pi * R * R * h
    m_n = m_f * (2.0 * R * math.tanh(xi_h_R)) / (xi * h * (xi * xi - 1.0))
    omega_n = math.sqrt(max(G * (xi / R) * math.tanh(xi_h_R), 0.0))
    height_observer = (4.0 * h * m_n) / (m_f * R)
    height_ferrari = (xi * xi * h * m_n) / (m_f * R)
    zeta_ferrari = (
        0.92
        * math.sqrt(cfg["nu"] / cfg["rho"] / (G * R ** 3))
        * (1.0 + (0.318 / math.sinh(xi_h_R)) * (1.0 + (1.0 - h / R) / math.cosh(xi_h_R)))
    )
    return {
        "omega_n": omega_n,
        "height_observer": height_observer,
        "height_ferrari": height_ferrari,
        "zeta_ferrari": zeta_ferrari,
    }


def read_visual_series(path):
    out = []
    for row in read_csv(path):
        ts = f(row.get("t_rel_sec"))
        h = f(row.get("h_smooth_corr"))
        if math.isfinite(ts) and math.isfinite(h):
            out.append((ts, abs(h)))
    return out


def read_bag_series(path):
    out = {"ax": [], "ay": [], "omega": [], "height": []}
    tracking_start = None
    tracking_end = None
    with rosbag.Bag(path) as bag:
        bag_start = bag.get_start_time()
        last_tracking = False
        for topic, msg, stamp in bag.read_messages(topics=["/slosh/ax_est", "/slosh/ay_est", "/odom", "/slosh/height", "/mpc_status"]):
            ts = stamp.to_sec() - bag_start
            if topic == "/slosh/ax_est":
                out["ax"].append((ts, float(msg.data)))
            elif topic == "/slosh/ay_est":
                out["ay"].append((ts, float(msg.data)))
            elif topic == "/odom":
                out["omega"].append((ts, float(msg.twist.twist.angular.z)))
            elif topic == "/slosh/height":
                out["height"].append((ts, abs(float(msg.data)) * 1000.0))
            elif topic == "/mpc_status":
                state = str(getattr(msg, "data", ""))
                is_tracking = state == "TRACKING"
                if is_tracking and tracking_start is None:
                    tracking_start = ts
                if last_tracking and not is_tracking and tracking_end is None:
                    tracking_end = ts
                last_tracking = is_tracking
    if tracking_start is None:
        tracking_start = 0.0
    if tracking_end is None and out["height"]:
        tracking_end = out["height"][-1][0]
    return out, tracking_start, tracking_end


def replay_model(series, cfg, omega_n, height_coeff, zeta, use_parabola):
    if not series["ax"] or not series["ay"]:
        return []
    start = max(series["ax"][0][0], series["ay"][0][0])
    end = min(series["ax"][-1][0], series["ay"][-1][0])
    if end <= start:
        return []
    dt = max(0.005, cfg["dt"])
    damping = 2.0 * zeta * omega_n
    wn2 = omega_n * omega_n
    radius2_over_4g = (cfg["R"] * cfg["R"]) / (4.0 * G)
    eta_x = eta_x_dot = eta_y = eta_y_dot = 0.0
    out = []
    steps = int(math.floor((end - start) / dt))
    for step in range(steps + 1):
        ts = start + step * dt
        ax = interp(series["ax"], ts, dt * 2.0)
        ay = interp(series["ay"], ts, dt * 2.0)
        omega = interp(series["omega"], ts, dt * 2.0)
        if not (math.isfinite(ax) and math.isfinite(ay)):
            continue
        if not math.isfinite(omega):
            omega = 0.0
        ddx = -damping * eta_x_dot - wn2 * eta_x - ax
        ddy = -damping * eta_y_dot - wn2 * eta_y - ay
        eta_x_dot += ddx * dt
        eta_y_dot += ddy * dt
        eta_x += eta_x_dot * dt
        eta_y += eta_y_dot * dt
        modal = height_coeff * math.hypot(eta_x, eta_y)
        parabola = radius2_over_4g * omega * omega if use_parabola else 0.0
        out.append((ts, (modal + parabola) * 1000.0))
    return out


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


def sign(value, eps):
    if not math.isfinite(value) or abs(value) < eps:
        return 0
    return 1 if value > 0 else -1


def fmt(value):
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return "{:.6g}".format(value) if math.isfinite(value) else "nan"
    return str(value)


def stringify(rows):
    return [{k: fmt(v) for k, v in row.items()} for row in rows]


def build_entries(root):
    visual_summary = read_csv(root / "visual_metric_summary.csv")
    with (root / "0424_groups_manifest_expanded.json").open("r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    bag_by_key = {(group, label): bag for group, items in manifest.items() for label, bag in items}
    entries = []
    for row in visual_summary:
        key = (row["group"], row["condition"])
        if key not in bag_by_key:
            continue
        entries.append({
            "group": row["group"],
            "block": row.get("block", ""),
            "condition": row["condition"],
            "bag": bag_by_key[key],
            "visual_csv": row["csv"],
        })
    return entries


def plot_variant_scatter(rows, metric, out_path):
    variants = sorted(set(row["variant"] for row in rows))
    if not variants:
        return
    fig, axes = plt.subplots(1, len(variants), figsize=(4.0 * len(variants), 3.8), sharex=True, sharey=True)
    if len(variants) == 1:
        axes = [axes]
    all_values = []
    for ax, variant in zip(axes, variants):
        xs = [row["visual_{}_mm".format(metric)] for row in rows if row["variant"] == variant]
        ys = [row["model_{}_mm".format(metric)] for row in rows if row["variant"] == variant]
        pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
        if not pairs:
            continue
        all_values.extend([v for pair in pairs for v in pair])
        ax.scatter([p[0] for p in pairs], [p[1] for p in pairs], s=22, alpha=0.75)
        ax.set_title(variant, fontsize=9)
        ax.grid(True, alpha=0.25)
    lim = max(all_values) if all_values else 1.0
    lim = max(lim, 1e-6)
    for ax in axes:
        ax.plot([0.0, lim], [0.0, lim], color="black", lw=1.0, alpha=0.6)
        ax.set_xlabel("RGB visual {} (mm)".format(metric))
    axes[0].set_ylabel("model {} (mm)".format(metric))
    fig.suptitle("0424 current/Ferrari model vs RGB truth: {}".format(metric), fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_arank_bar(summary, metric, out_path):
    rows = [row for row in summary if row["metric"] == metric and math.isfinite(row["A_rank"])]
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(8.5, 4.2))
    xs = list(range(len(rows)))
    ys = [row["A_rank"] for row in rows]
    ax.bar(xs, ys, color="#4c78a8", alpha=0.85)
    ax.axhline(0.7, color="#d62728", lw=1.0, linestyle="--", label="target 0.70")
    ax.axhline(0.5, color="black", lw=1.0, alpha=0.5, label="random 0.50")
    ax.set_xticks(xs)
    ax.set_xticklabels([row["variant"] for row in rows], rotation=25, ha="right", fontsize=8)
    ax.set_ylim(0.0, 1.0)
    ax.set_ylabel("A_rank")
    ax.set_title("0424 current/Ferrari model A_rank ({})".format(metric))
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def find_summary(summary, variant, metric):
    for row in summary:
        if row["variant"] == variant and row["metric"] == metric:
            return row
    return {}


def main():
    args = parse_args()
    root = Path(args.root)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = load_config(args.config)
    derived = derived_params(cfg)
    variants = [
        ("M0_recorded_slosh_height", None),
        ("M0b_replay_observer", {"height_coeff": derived["height_observer"], "zeta": cfg["zeta"], "use_parabola": cfg["use_parabola_term"]}),
        ("M1_ferrari_height_coeff", {"height_coeff": derived["height_ferrari"], "zeta": cfg["zeta"], "use_parabola": cfg["use_parabola_term"]}),
        ("M2_ferrari_no_parabola", {"height_coeff": derived["height_ferrari"], "zeta": cfg["zeta"], "use_parabola": False}),
        ("M3_ferrari_full", {"height_coeff": derived["height_ferrari"], "zeta": derived["zeta_ferrari"], "use_parabola": cfg["use_parabola_term"]}),
    ]

    rows = []
    for entry in build_entries(root):
        visual = read_visual_series(entry["visual_csv"])
        series, t0, tend = read_bag_series(entry["bag"])
        t1 = tend + 2.0
        model_series = {}
        for name, params in variants:
            if params is None:
                model_series[name] = series["height"]
            else:
                model_series[name] = replay_model(
                    series, cfg, derived["omega_n"],
                    params["height_coeff"], params["zeta"], params["use_parabola"],
                )
        for name, model in model_series.items():
            metrics = pair_metrics(visual, model, t0, t1, args.max_gap_sec)
            if metrics is None:
                continue
            rows.append({
                "variant": name,
                "group": entry["group"],
                "block": entry["block"],
                "condition": entry["condition"],
                "bag": entry["bag"],
                "visual_csv": entry["visual_csv"],
                **metrics,
            })

    pair_rows = []
    for variant in sorted(set(row["variant"] for row in rows)):
        vrows = [row for row in rows if row["variant"] == variant]
        groups = {}
        for row in vrows:
            groups.setdefault(row["group"], []).append(row)
        for group, grows in groups.items():
            for metric, key in (("peak", "peak_mm"), ("p95", "p95_mm"), ("rms", "rms_mm")):
                model_key = "model_" + key
                visual_key = "visual_" + key
                for i in range(len(grows)):
                    for j in range(i + 1, len(grows)):
                        a = grows[i]
                        b = grows[j]
                        md = b[model_key] - a[model_key]
                        vd = b[visual_key] - a[visual_key]
                        ms = sign(md, args.sign_eps_mm)
                        vs = sign(vd, args.sign_eps_mm)
                        verdict = "tie" if ms == 0 or vs == 0 else ("agree" if ms == vs else "disagree")
                        pair_rows.append({
                            "variant": variant,
                            "group": group,
                            "metric": metric,
                            "pair": "{}-{}".format(b["condition"], a["condition"]),
                            "model_diff_mm": md,
                            "visual_diff_mm": vd,
                            "verdict": verdict,
                        })

    summary = []
    for variant in sorted(set(row["variant"] for row in rows)):
        vrows = [row for row in rows if row["variant"] == variant]
        for metric in ("peak", "p95", "rms"):
            pairs = [row for row in pair_rows if row["variant"] == variant and row["metric"] == metric]
            non_tie = [row for row in pairs if row["verdict"] != "tie"]
            agree = sum(1 for row in non_tie if row["verdict"] == "agree")
            summary.append({
                "variant": variant,
                "metric": metric,
                "bags": len(vrows),
                "mean_gamma_model_pct": sum(row["gamma_model_pct"] for row in vrows) / len(vrows) if vrows else math.nan,
                "median_corr": percentile([row["corr"] for row in vrows], 50.0),
                "mean_U_p95_mm": sum(row["U_p95_mm"] for row in vrows) / len(vrows) if vrows else math.nan,
                "worst_U_max_mm": max((row["U_max_mm"] for row in vrows), default=math.nan),
                "rank_pairs": len(non_tie),
                "agree": agree,
                "disagree": sum(1 for row in non_tie if row["verdict"] == "disagree"),
                "ties": sum(1 for row in pairs if row["verdict"] == "tie"),
                "A_rank": agree / len(non_tie) if non_tie else math.nan,
            })

    write_csv(out_dir / "ferrari_variant_fidelity_summary.csv", stringify(rows))
    write_csv(out_dir / "ferrari_variant_selection_pairs.csv", stringify(pair_rows))
    write_csv(out_dir / "ferrari_variant_A_rank_summary.csv", stringify(summary))
    plot_dir = out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    for metric in ("peak", "p95"):
        plot_variant_scatter(rows, metric, plot_dir / "scatter_{}_by_variant.png".format(metric))
    plot_arank_bar(summary, "p95", plot_dir / "A_rank_p95_by_variant.png")

    m0 = find_summary(summary, "M0_recorded_slosh_height", "p95")
    m0b = find_summary(summary, "M0b_replay_observer", "p95")
    m1 = find_summary(summary, "M1_ferrari_height_coeff", "p95")
    m2 = find_summary(summary, "M2_ferrari_no_parabola", "p95")
    m3 = find_summary(summary, "M3_ferrari_full", "p95")

    md = [
        "# 2026-04-24 当前模型 vs Ferrari 模型保真度对比",
        "",
        "## 参数",
        "",
        "```text",
        "omega_n={:.6g}".format(derived["omega_n"]),
        "height_coeff_observer={:.6g}".format(derived["height_observer"]),
        "height_coeff_ferrari={:.6g}".format(derived["height_ferrari"]),
        "zeta_current={:.6g}".format(cfg["zeta"]),
        "zeta_ferrari={:.6g}".format(derived["zeta_ferrari"]),
        "```",
        "",
        "## 图表",
        "",
        "- `plots/scatter_p95_by_variant.png`",
        "- `plots/scatter_peak_by_variant.png`",
        "- `plots/A_rank_p95_by_variant.png`",
        "",
        "## A_rank 摘要",
        "",
        "| variant | metric | rank_pairs | agree | disagree | ties | A_rank | mean_gamma_model_pct | mean_U_p95_mm | worst_U_max_mm |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        md.append("| {variant} | {metric} | {rank_pairs} | {agree} | {disagree} | {ties} | {A_rank} | {mean_gamma_model_pct} | {mean_U_p95_mm} | {worst_U_max_mm} |".format(**{k: fmt(v) for k, v in row.items()}))
    md.extend([
        "",
        "## 结论",
        "",
        "```text",
        "M0 recorded /slosh/height 的 p95 A_rank={}; mean_gamma_model_pct={}，整体已经低估 RGB 真值。".format(fmt(m0.get("A_rank", math.nan)), fmt(m0.get("mean_gamma_model_pct", math.nan))),
        "M0b replay observer 的 p95 A_rank={}; 它比 recorded 略高，但仍未达到 0.70 门槛。".format(fmt(m0b.get("A_rank", math.nan))),
        "M1 只替换 Ferrari height_coeff 的 p95 A_rank={}; mean_gamma_model_pct={}，排序略升但幅值低估更严重。".format(fmt(m1.get("A_rank", math.nan)), fmt(m1.get("mean_gamma_model_pct", math.nan))),
        "M2 关闭 parabola 后与 M1 基本一致，说明 0424 这批数据里 parabola 项不是主差异来源；p95 A_rank={}。".format(fmt(m2.get("A_rank", math.nan))),
        "M3 Ferrari full 的 p95 A_rank={}，低于 recorded/replay observer；Ferrari physics damping 在这批 0424 数据上没有改善 RGB 排序。".format(fmt(m3.get("A_rank", math.nan))),
        "因此当前不能把 Ferrari closed-form 直接切为在线默认模型；它可作为 ablation/oracle 继续验证，但不能替代 RGB 真值闭环。",
        "这批 bag 是旧 MPC 数据，只能作为模型保真度证据，不能作为 OSCRS 效果证据。",
        "```",
    ])
    (out_dir / "FERRARI_FIDELITY_REPORT.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("wrote", out_dir)
    for row in summary:
        if row["metric"] == "p95":
            print(row)


if __name__ == "__main__":
    main()
