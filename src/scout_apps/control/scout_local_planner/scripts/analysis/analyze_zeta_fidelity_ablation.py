#!/usr/bin/env python3
"""Offline zeta ablation against RGB liquid-height truth.

This script replays the same measured acceleration sequence with two damping
ratios:

  - manual zeta from yaml, currently 0.05
  - Ferrari-style physical zeta derived from container dimensions

The online planner is not changed.  The output answers whether Ferrari zeta
improves model/RGB fidelity enough to justify adding an online switch later.
"""

import argparse
import bisect
import csv
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag
import yaml


G = 9.81
XI_11 = 1.8412
RGB_LCR_COLS = ("h_mm_left", "h_mm_center", "h_mm_right")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", required=True)
    parser.add_argument("--glob", default="*.bag")
    parser.add_argument("--red-infer-dir", required=True)
    parser.add_argument("--config", default="src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--terminal-exclusion-s", type=float, default=1.0)
    parser.add_argument("--pair-max-gap-s", type=float, default=0.15)
    parser.add_argument("--replay-dt", type=float, default=0.005)
    parser.add_argument("--sign-eps-mm", type=float, default=0.05)
    return parser.parse_args()


def read_csv_rows(path):
    with Path(path).open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def f(value, default=math.nan):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def fmt(value):
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.6g}" if math.isfinite(value) else "nan"
    return str(value)


def condition_from_name(name):
    match = re.search(r"_P2_s_curve_([A-Z])(?:_|$)", name)
    return match.group(1) if match else "UNKNOWN"


def load_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    slosh = data.get("slosh", {}) or {}
    oscrs = data.get("oscrs", {}) or {}
    return {
        "R": f(slosh.get("container_radius"), 0.0185),
        "h": f(slosh.get("liquid_height"), 0.058),
        "rho": f(slosh.get("liquid_density"), 1000.0),
        "mu": f(slosh.get("liquid_dynamic_viscosity"), 1.0e-3),
        "zeta_manual": f(slosh.get("damping_ratio"), 0.05),
        "use_parabola_term": bool(slosh.get("use_parabola_term", True)),
        "height_coeff_manual": f(oscrs.get("height_coeff_manual"), 0.0),
    }


def derived_params(cfg):
    R = cfg["R"]
    h = cfg["h"]
    rho = cfg["rho"]
    mu = cfg["mu"]
    xi_h_R = XI_11 * h / R
    m_f = rho * math.pi * R * R * h
    m_n = m_f * (2.0 * R * math.tanh(xi_h_R)) / (XI_11 * h * (XI_11 * XI_11 - 1.0))
    omega_n = math.sqrt(G * (XI_11 / R) * math.tanh(xi_h_R))
    height_observer = (4.0 * h * m_n) / (m_f * R)
    height_ferrari = (XI_11 * XI_11 * h * m_n) / (m_f * R)
    zeta_ferrari = (
        0.92
        * math.sqrt(mu / rho / (G * R ** 3))
        * (1.0 + (0.318 / math.sinh(xi_h_R)) * (1.0 + (1.0 - h / R) / math.cosh(xi_h_R)))
    )
    return {
        "omega_n": omega_n,
        "height_observer": height_observer,
        "height_ferrari": height_ferrari,
        "zeta_ferrari": zeta_ferrari,
    }


def interp(series, ts, max_gap):
    if not series:
        return math.nan
    times = [item[0] for item in series]
    idx = bisect.bisect_left(times, ts)
    if idx <= 0:
        return series[0][1] if abs(series[0][0] - ts) <= max_gap else math.nan
    if idx >= len(series):
        return series[-1][1] if abs(series[-1][0] - ts) <= max_gap else math.nan
    t0, v0 = series[idx - 1]
    t1, v1 = series[idx]
    if min(abs(ts - t0), abs(t1 - ts)) > max_gap:
        return math.nan
    if t1 <= t0:
        return v0
    ratio = (ts - t0) / (t1 - t0)
    return v0 * (1.0 - ratio) + v1 * ratio


def read_rgb_series(path):
    rows = read_csv_rows(path)
    out = []
    for row in rows:
        ts = f(row.get("stamp_sec"))
        vals = [abs(f(row.get(col))) for col in RGB_LCR_COLS]
        vals = [v for v in vals if math.isfinite(v)]
        if math.isfinite(ts) and vals:
            out.append((ts, max(vals)))
    return out


def find_rgb_csv(red_infer_dir, bag_path):
    target = f"{bag_path.stem}_red_top.csv"
    direct = Path(red_infer_dir) / target
    if direct.exists():
        return direct
    matches = sorted(Path(red_infer_dir).rglob(target))
    return matches[0] if matches else None


def read_bag_series(path):
    topics = [
        "/slosh/ax_est", "/slosh/ay_est", "/odom", "/slosh/height",
        "/mpc_status", "/terminal/mode",
    ]
    out = {"ax": [], "ay": [], "omega": [], "recorded": []}
    tracking_start = None
    first_terminal = None
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = stamp.to_sec()
            if topic == "/slosh/ax_est":
                out["ax"].append((ts, f(getattr(msg, "data", math.nan))))
            elif topic == "/slosh/ay_est":
                out["ay"].append((ts, f(getattr(msg, "data", math.nan))))
            elif topic == "/odom":
                out["omega"].append((ts, f(msg.twist.twist.angular.z)))
            elif topic == "/slosh/height":
                out["recorded"].append((ts, abs(f(getattr(msg, "data", 0.0))) * 1000.0))
            elif topic == "/mpc_status":
                data = str(getattr(msg, "data", ""))
                if tracking_start is None and data == "TRACKING":
                    tracking_start = ts
            elif topic == "/terminal/mode":
                data = str(getattr(msg, "data", ""))
                if first_terminal is None and data not in ("", "IDLE", "NONE"):
                    first_terminal = ts
    return out, tracking_start, first_terminal


def replay_model(series, cfg, omega_n, height_coeff, zeta, replay_dt):
    if not series["ax"] or not series["ay"]:
        return []
    start = max(series["ax"][0][0], series["ay"][0][0])
    end = min(series["ax"][-1][0], series["ay"][-1][0])
    if end <= start:
        return []
    dt = max(0.001, replay_dt)
    damping = 2.0 * zeta * omega_n
    wn2 = omega_n * omega_n
    radius2_over_4g = (cfg["R"] * cfg["R"]) / (4.0 * G)
    eta_x = eta_x_dot = eta_y = eta_y_dot = 0.0
    out = []
    steps = int(math.floor((end - start) / dt))

    def deriv(state, ax, ay):
        ex, exd, ey, eyd = state
        return (
            exd,
            -damping * exd - wn2 * ex - ax,
            eyd,
            -damping * eyd - wn2 * ey - ay,
        )

    state = (eta_x, eta_x_dot, eta_y, eta_y_dot)
    for step in range(steps + 1):
        ts = start + step * dt
        ax = interp(series["ax"], ts, dt * 4.0)
        ay = interp(series["ay"], ts, dt * 4.0)
        omega = interp(series["omega"], ts, dt * 8.0)
        if not (math.isfinite(ax) and math.isfinite(ay)):
            continue
        if not math.isfinite(omega):
            omega = 0.0

        if step > 0:
            t_prev = ts - dt
            ax0 = interp(series["ax"], t_prev, dt * 4.0)
            ay0 = interp(series["ay"], t_prev, dt * 4.0)
            axm = interp(series["ax"], t_prev + 0.5 * dt, dt * 4.0)
            aym = interp(series["ay"], t_prev + 0.5 * dt, dt * 4.0)
            if math.isfinite(ax0) and math.isfinite(ay0) and math.isfinite(axm) and math.isfinite(aym):
                k1 = deriv(state, ax0, ay0)
                s2 = tuple(state[i] + 0.5 * dt * k1[i] for i in range(4))
                k2 = deriv(s2, axm, aym)
                s3 = tuple(state[i] + 0.5 * dt * k2[i] for i in range(4))
                k3 = deriv(s3, axm, aym)
                s4 = tuple(state[i] + dt * k3[i] for i in range(4))
                k4 = deriv(s4, ax, ay)
                state = tuple(state[i] + dt * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0
                              for i in range(4))

        eta_x, eta_x_dot, eta_y, eta_y_dot = state
        modal = height_coeff * math.hypot(eta_x, eta_y)
        parabola = radius2_over_4g * omega * omega if cfg["use_parabola_term"] else 0.0
        out.append((ts, (modal + parabola) * 1000.0))
    return out


def percentile(values, q):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    pos = (len(values) - 1) * q / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    return values[lo] * (hi - pos) + values[hi] * (pos - lo)


def rms(values):
    values = [v for v in values if math.isfinite(v)]
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else math.nan


def corr(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return math.nan
    mx = sum(x for x, _ in pairs) / len(pairs)
    my = sum(y for _, y in pairs) / len(pairs)
    vx = sum((x - mx) ** 2 for x, _ in pairs)
    vy = sum((y - my) ** 2 for _, y in pairs)
    if vx <= 1e-12 or vy <= 1e-12:
        return math.nan
    cov = sum((x - mx) * (y - my) for x, y in pairs)
    return cov / math.sqrt(vx * vy)


def trapz(values, times):
    if len(values) < 2:
        return math.nan
    total = 0.0
    for i in range(1, len(values)):
        dt = times[i] - times[i - 1]
        if dt > 0.0:
            total += 0.5 * (abs(values[i]) + abs(values[i - 1])) * dt
    return total


def pair_metrics(visual, model, t0, t1, max_gap_sec):
    times = []
    vis = []
    mod = []
    for ts, value in visual:
        if ts < t0 or ts > t1:
            continue
        mv = interp(model, ts, max_gap_sec)
        if not math.isfinite(mv):
            continue
        times.append(ts)
        vis.append(value)
        mod.append(mv)
    if not vis:
        return None
    diff = [m - v for m, v in zip(mod, vis)]
    denom = trapz(mod, times)
    gamma = 100.0 * trapz(diff, times) / denom if math.isfinite(denom) and abs(denom) > 1e-12 else math.nan
    under = [max(0.0, v - m) for m, v in zip(mod, vis)]
    return {
        "paired_samples": len(vis),
        "gamma_model_pct": gamma,
        "rmse_mm": rms(diff),
        "corr": corr(mod, vis),
        "rgb_peak_mm": max(vis),
        "model_peak_mm": max(mod),
        "rgb_p95_mm": percentile(vis, 95.0),
        "model_p95_mm": percentile(mod, 95.0),
        "rgb_rms_mm": rms(vis),
        "model_rms_mm": rms(mod),
        "U_p95_mm": max(0.0, percentile(vis, 95.0) - percentile(mod, 95.0)),
        "U_max_mm": max(under) if under else math.nan,
    }


def sign(value, eps):
    if not math.isfinite(value) or abs(value) < eps:
        return 0
    return 1 if value > 0 else -1


def mean(values):
    values = [v for v in values if math.isfinite(v)]
    return sum(values) / len(values) if values else math.nan


def make_plots(rows, summary_rows, out_dir):
    plot_dir = Path(out_dir) / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    variants = ["recorded_online", "replay_manual_zeta", "replay_ferrari_zeta", "replay_ferrari_full"]
    labels = {
        "recorded_online": "recorded /slosh",
        "replay_manual_zeta": "manual zeta",
        "replay_ferrari_zeta": "Ferrari zeta",
        "replay_ferrari_full": "Ferrari full",
    }
    metrics = [
        ("gamma_model_pct", "gamma_model lower is better"),
        ("rmse_mm", "RMSE lower is better"),
        ("corr", "corr higher is better"),
        ("U_p95_mm", "p95 underestimation lower is better"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    axes = axes.ravel()
    for ax, (metric, title) in zip(axes, metrics):
        ys = [mean([r[metric] for r in rows if r["variant"] == v]) for v in variants]
        ax.bar(range(len(variants)), ys, color=["#666666", "#4c78a8", "#f58518", "#54a24b"])
        ax.set_xticks(range(len(variants)))
        ax.set_xticklabels([labels[v] for v in variants], rotation=25, ha="right", fontsize=8)
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.25)
    fig.suptitle("Zeta ablation fidelity vs RGB, main tracking window")
    fig.tight_layout()
    fig.savefig(plot_dir / "zeta_ablation_summary.png", dpi=180)
    fig.savefig(plot_dir / "zeta_ablation_summary.pdf")
    plt.close(fig)

    arank = [r for r in summary_rows if r["metric"] in ("p95", "peak")]
    if arank:
        fig, ax = plt.subplots(figsize=(8, 4))
        xs = range(len(arank))
        ax.bar(xs, [r["A_rank"] for r in arank], color="#4c78a8")
        ax.axhline(0.5, color="black", linestyle="--", linewidth=1.0, label="random 0.5")
        ax.axhline(0.7, color="#d62728", linestyle="--", linewidth=1.0, label="target 0.7")
        ax.set_xticks(list(xs))
        ax.set_xticklabels([f"{r['variant']} {r['metric']}" for r in arank], rotation=25, ha="right", fontsize=8)
        ax.set_ylim(0, 1)
        ax.set_ylabel("A_rank")
        ax.set_title("Model/RGB ordering agreement")
        ax.grid(True, axis="y", alpha=0.25)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(plot_dir / "zeta_ablation_arank.png", dpi=180)
        fig.savefig(plot_dir / "zeta_ablation_arank.pdf")
        plt.close(fig)


def main():
    args = parse_args()
    bag_dir = Path(args.bag_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg = load_config(args.config)
    derived = derived_params(cfg)

    variants = [
        ("recorded_online", None, math.nan, math.nan),
        ("replay_manual_zeta", "observer", cfg["zeta_manual"], derived["height_observer"]),
        ("replay_ferrari_zeta", "observer", derived["zeta_ferrari"], derived["height_observer"]),
        ("replay_ferrari_full", "ferrari", derived["zeta_ferrari"], derived["height_ferrari"]),
    ]

    rows = []
    bags = sorted(bag_dir.glob(args.glob))
    for bag_path in bags:
        rgb_path = find_rgb_csv(args.red_infer_dir, bag_path)
        if rgb_path is None:
            print(f"[warn] skip {bag_path.name}: RGB CSV missing")
            continue
        series, tracking_start, first_terminal = read_bag_series(bag_path)
        if tracking_start is None or first_terminal is None:
            print(f"[warn] skip {bag_path.name}: missing tracking/terminal markers")
            continue
        window_end = max(tracking_start, first_terminal - args.terminal_exclusion_s)
        visual = read_rgb_series(rgb_path)
        replay_cache = {}
        for variant, height_mode, zeta, h_coeff in variants:
            if variant == "recorded_online":
                model = series["recorded"]
                zeta_used = math.nan
                coeff_used = math.nan
            else:
                model = replay_cache.get(variant)
                if model is None:
                    model = replay_model(series, cfg, derived["omega_n"], h_coeff, zeta, args.replay_dt)
                    replay_cache[variant] = model
                zeta_used = zeta
                coeff_used = h_coeff
            metrics = pair_metrics(visual, model, tracking_start, window_end, args.pair_max_gap_s)
            if metrics is None:
                print(f"[warn] no pairs {bag_path.name} {variant}")
                continue
            rows.append({
                "variant": variant,
                "condition": condition_from_name(bag_path.name),
                "bag": bag_path.name,
                "rgb_csv": str(rgb_path),
                "window": "tracking_pre_terminal_minus_1s",
                "tracking_start_s": tracking_start,
                "first_terminal_s": first_terminal,
                "window_end_s": window_end,
                "zeta_used": zeta_used,
                "height_coeff_used": coeff_used,
                "omega_n": derived["omega_n"],
                **metrics,
            })

    summary = []
    for variant in sorted(set(r["variant"] for r in rows)):
        vrows = [r for r in rows if r["variant"] == variant]
        for metric in ("peak", "p95", "rms"):
            model_key = f"model_{metric}_mm"
            rgb_key = f"rgb_{metric}_mm"
            pairs = []
            for i in range(len(vrows)):
                for j in range(i + 1, len(vrows)):
                    md = vrows[j][model_key] - vrows[i][model_key]
                    vd = vrows[j][rgb_key] - vrows[i][rgb_key]
                    ms = sign(md, args.sign_eps_mm)
                    vs = sign(vd, args.sign_eps_mm)
                    if ms == 0 or vs == 0:
                        verdict = "tie"
                    else:
                        verdict = "agree" if ms == vs else "disagree"
                    pairs.append(verdict)
            non_tie = [p for p in pairs if p != "tie"]
            agree = sum(1 for p in non_tie if p == "agree")
            summary.append({
                "variant": variant,
                "metric": metric,
                "bags": len(vrows),
                "mean_gamma_model_pct": mean([r["gamma_model_pct"] for r in vrows]),
                "mean_rmse_mm": mean([r["rmse_mm"] for r in vrows]),
                "median_corr": percentile([r["corr"] for r in vrows], 50.0),
                "mean_U_p95_mm": mean([r["U_p95_mm"] for r in vrows]),
                "rank_pairs": len(non_tie),
                "agree": agree,
                "disagree": sum(1 for p in non_tie if p == "disagree"),
                "ties": sum(1 for p in pairs if p == "tie"),
                "A_rank": agree / len(non_tie) if non_tie else math.nan,
            })

    group_summary = []
    for variant in sorted(set(r["variant"] for r in rows)):
        for condition in sorted(set(r["condition"] for r in rows if r["variant"] == variant)):
            grows = [r for r in rows if r["variant"] == variant and r["condition"] == condition]
            group_summary.append({
                "variant": variant,
                "condition": condition,
                "bags": len(grows),
                "mean_gamma_model_pct": mean([r["gamma_model_pct"] for r in grows]),
                "mean_rmse_mm": mean([r["rmse_mm"] for r in grows]),
                "mean_corr": mean([r["corr"] for r in grows]),
                "mean_rgb_p95_mm": mean([r["rgb_p95_mm"] for r in grows]),
                "mean_model_p95_mm": mean([r["model_p95_mm"] for r in grows]),
                "mean_rgb_peak_mm": mean([r["rgb_peak_mm"] for r in grows]),
                "mean_model_peak_mm": mean([r["model_peak_mm"] for r in grows]),
                "mean_U_p95_mm": mean([r["U_p95_mm"] for r in grows]),
            })

    rows_s = [{k: fmt(v) for k, v in row.items()} for row in rows]
    summary_s = [{k: fmt(v) for k, v in row.items()} for row in summary]
    group_s = [{k: fmt(v) for k, v in row.items()} for row in group_summary]
    write_csv(out_dir / "zeta_ablation_per_bag.csv", rows_s)
    write_csv(out_dir / "zeta_ablation_summary.csv", summary_s)
    write_csv(out_dir / "zeta_ablation_group_summary.csv", group_s)
    make_plots(rows, summary, out_dir)

    def pick(variant, metric="p95"):
        for row in summary:
            if row["variant"] == variant and row["metric"] == metric:
                return row
        return {}

    manual = pick("replay_manual_zeta")
    ferrari = pick("replay_ferrari_zeta")
    recorded = pick("recorded_online")
    md = [
        "# ζ 离线保真度对比报告",
        "",
        "窗口：`TRACKING -> first terminal - 1s`，RGB 真值使用 `max(left, center, right)`。",
        "",
        "## 参数",
        "",
        "```text",
        f"R={cfg['R']:.6g} m",
        f"h={cfg['h']:.6g} m",
        f"omega_n={derived['omega_n']:.6g} rad/s",
        f"height_coeff_observer={derived['height_observer']:.6g}",
        f"height_coeff_ferrari={derived['height_ferrari']:.6g}",
        f"zeta_manual={cfg['zeta_manual']:.6g}",
        f"zeta_ferrari={derived['zeta_ferrari']:.6g}",
        "```",
        "",
        "## 输出文件",
        "",
        "- `zeta_ablation_per_bag.csv`",
        "- `zeta_ablation_summary.csv`",
        "- `zeta_ablation_group_summary.csv`",
        "- `plots/zeta_ablation_summary.png`",
        "- `plots/zeta_ablation_arank.png`",
        "",
        "## p95 总结",
        "",
        "| variant | A_rank | mean_gamma_model_pct | mean_rmse_mm | median_corr | mean_U_p95_mm |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        if row["metric"] == "p95":
            md.append("| {variant} | {A_rank} | {mean_gamma_model_pct} | {mean_rmse_mm} | {median_corr} | {mean_U_p95_mm} |".format(**{k: fmt(v) for k, v in row.items()}))
    md.extend([
        "",
        "## 判断",
        "",
        "```text",
        "recorded_online p95: A_rank={}, gamma={}, rmse={}, corr={}".format(
            fmt(recorded.get("A_rank", math.nan)),
            fmt(recorded.get("mean_gamma_model_pct", math.nan)),
            fmt(recorded.get("mean_rmse_mm", math.nan)),
            fmt(recorded.get("median_corr", math.nan)),
        ),
        "manual zeta replay p95: A_rank={}, gamma={}, rmse={}, corr={}".format(
            fmt(manual.get("A_rank", math.nan)),
            fmt(manual.get("mean_gamma_model_pct", math.nan)),
            fmt(manual.get("mean_rmse_mm", math.nan)),
            fmt(manual.get("median_corr", math.nan)),
        ),
        "Ferrari zeta replay p95: A_rank={}, gamma={}, rmse={}, corr={}".format(
            fmt(ferrari.get("A_rank", math.nan)),
            fmt(ferrari.get("mean_gamma_model_pct", math.nan)),
            fmt(ferrari.get("mean_rmse_mm", math.nan)),
            fmt(ferrari.get("median_corr", math.nan)),
        ),
        "```",
        "",
        "若 Ferrari zeta 没有同时降低 gamma/RMSE 且提升 A_rank/corr，则不建议接入在线默认。",
    ])
    (out_dir / "ZETA_FIDELITY_ABLATION_REPORT.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"[OK] wrote {out_dir}")


if __name__ == "__main__":
    main()
