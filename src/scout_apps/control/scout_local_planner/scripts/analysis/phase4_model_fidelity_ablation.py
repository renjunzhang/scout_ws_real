#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phase4 model-fidelity ablation against RGB visual ground truth.

This is the L2.6 Step-0 gate for OSCRS candidate-diversity work.  It does not
change online parameters; it replays recorded slosh excitations with several
model variants and checks whether model ranking agrees with RGB visual metrics.
"""

import argparse
import bisect
import csv
import math
import os
from pathlib import Path

import rosbag
import yaml

G = 9.81
XI_11 = 1.841


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--summary-csv",
        default="docs/Claude/分析数据/phase4_visual_20260509/phase4_visual_metric_summary_0424style.csv",
        help="Phase4 visual metric summary CSV with bag and visual_csv columns.",
    )
    parser.add_argument(
        "--bag-dir",
        default="/data/a/slosh_bags/real/20260508_phase4",
        help="Directory containing phase4 bag files.",
    )
    parser.add_argument(
        "--config",
        default="src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml",
        help="OSCRS container YAML used to derive model parameters.",
    )
    parser.add_argument(
        "--out-dir",
        default="docs/Claude/分析数据/phase4_model_fidelity_20260513",
        help="Output directory for ablation CSV/Markdown reports.",
    )
    parser.add_argument("--max-pair-gap-sec", type=float, default=0.15)
    parser.add_argument("--visual-column", default="h_mm_smooth_corr")
    parser.add_argument("--min-confidence", type=float, default=0.0)
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
        "height_coeff_mode": str(oscrs.get("height_coeff_mode", "observer_linear")),
        "dt": float(oscrs.get("rollout_dt", 0.05)),
    }


def derived_params(cfg):
    R = cfg["R"]
    h = cfg["h"]
    rho = cfg["rho"]
    nu = cfg["nu"]
    xi = XI_11
    xi_h_R = xi * h / R
    m_f = rho * math.pi * R * R * h
    m_n = m_f * (2.0 * R * math.tanh(xi_h_R)) / (xi * h * (xi * xi - 1.0))
    omega_n = math.sqrt(max(G * (xi / R) * math.tanh(xi_h_R), 0.0))
    height_coeff_observer = (4.0 * h * m_n) / (m_f * R)
    height_coeff_ferrari = (xi * xi * h * m_n) / (m_f * R)
    zeta_ferrari = (
        0.92
        * math.sqrt(nu / rho / (G * R ** 3))
        * (1.0 + (0.318 / math.sinh(xi_h_R)) * (1.0 + (1.0 - h / R) / math.cosh(xi_h_R)))
    )
    return {
        "omega_n": omega_n,
        "height_coeff_observer": height_coeff_observer,
        "height_coeff_ferrari": height_coeff_ferrari,
        "zeta_ferrari": zeta_ferrari,
    }


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
    r = pos - lo
    return vals[lo] * (1.0 - r) + vals[hi] * r


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


def fit_scale(source, target):
    pairs = [(s, t) for s, t in zip(source, target) if math.isfinite(s) and math.isfinite(t)]
    denom = sum(s * s for s, _ in pairs)
    if denom <= 1e-12:
        return math.nan
    return sum(s * t for s, t in pairs) / denom


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
    if t1 <= t0 or min(abs(ts - t0), abs(t1 - ts)) > max_gap_sec:
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


def read_start_times(bag_path, motion_threshold=0.03, motion_consecutive=3):
    tracking_start = None
    motion_start = None
    consec = 0
    with rosbag.Bag(str(bag_path)) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=["/mpc_status", "/odom"]):
            rel_t = stamp.to_sec() - start
            if topic == "/mpc_status" and tracking_start is None:
                if str(getattr(msg, "data", "")).strip() == "TRACKING":
                    tracking_start = rel_t
            elif topic == "/odom" and motion_start is None:
                v = float(msg.twist.twist.linear.x)
                if abs(v) > motion_threshold:
                    consec += 1
                    if consec >= max(1, motion_consecutive):
                        motion_start = rel_t
                else:
                    consec = 0
            if tracking_start is not None and motion_start is not None:
                break
    return motion_start if motion_start is not None else (tracking_start or 0.0)


def read_bag_series(bag_path, static_median_m):
    out = {"ax": [], "ay": [], "omega": [], "height": []}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, stamp in bag.read_messages(
            topics=["/slosh/ax_est", "/slosh/ay_est", "/odom", "/slosh/height"]
        ):
            ts = stamp.to_sec()
            if topic == "/slosh/ax_est":
                out["ax"].append((ts, float(msg.data)))
            elif topic == "/slosh/ay_est":
                out["ay"].append((ts, float(msg.data)))
            elif topic == "/odom":
                out["omega"].append((ts, float(msg.twist.twist.angular.z)))
            elif topic == "/slosh/height":
                out["height"].append((ts, max(0.0, float(msg.data) - static_median_m) * 1000.0))
    return out


def static_slosh_median(bag_dir):
    static = Path(bag_dir) / "slosh_Q0_20260509_204602_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_static.bag"
    if not static.exists():
        return 0.0
    vals = []
    with rosbag.Bag(str(static)) as bag:
        for _, msg, _ in bag.read_messages(topics=["/slosh/height"]):
            vals.append(float(msg.data))
    return percentile(vals, 50.0) if vals else 0.0


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


def read_visual_series(path, column, min_confidence):
    rows = []
    for row in read_csv(path):
        try:
            conf = float(row.get("conf_mean", "nan"))
            if conf < min_confidence:
                continue
            ts = float(row["stamp_sec"])
            value = abs(float(row[column]))
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(ts) and math.isfinite(value):
            rows.append((ts, value))
    return rows


def paired_metrics(visual_series, model_series, analysis_start_abs, max_gap_sec):
    visual = []
    model = []
    dts = []
    for ts, value in visual_series:
        if ts < analysis_start_abs:
            continue
        pred = interp(model_series, ts, max_gap_sec)
        if not math.isfinite(pred):
            continue
        visual.append(value)
        model.append(pred)
        dts.append(nearest_dt(model_series, ts))
    if not visual:
        return None
    visual_p95 = percentile(visual, 95.0)
    model_p95 = percentile(model, 95.0)
    visual_peak = max(visual)
    model_peak = max(model)
    visual_rms = rms(visual)
    model_rms = rms(model)
    return {
        "paired_samples": len(visual),
        "pair_dt_median_ms": percentile(dts, 50.0) * 1000.0,
        "pair_dt_p95_ms": percentile(dts, 95.0) * 1000.0,
        "visual_p95_mm": visual_p95,
        "visual_peak_mm": visual_peak,
        "visual_rms_mm": visual_rms,
        "model_p95_mm": model_p95,
        "model_peak_mm": model_peak,
        "model_rms_mm": model_rms,
        "corr_model_visual": corr(model, visual),
        "scale_fit_model_per_visual": fit_scale(visual, model),
        "U_p95_mm": max(0.0, visual_p95 - model_p95),
        "U_max_mm": max(0.0, visual_peak - model_peak),
    }


def sign(value, eps=1e-9):
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def build_rank_rows(fidelity_rows):
    by_key = {}
    for row in fidelity_rows:
        by_key[(row["run"], row["model_variant"], row["condition"])] = row
    rank_rows = []
    conditions = ["RAW", "FIXED_MILD", "OSCRS_MEDIUM_ACTIVE"]
    for run in sorted(set(row["run"] for row in fidelity_rows)):
        variants = sorted(set(row["model_variant"] for row in fidelity_rows if row["run"] == run))
        for variant in variants:
            for metric in ["p95", "peak", "rms"]:
                for i in range(len(conditions)):
                    for j in range(i + 1, len(conditions)):
                        a = by_key.get((run, variant, conditions[i]))
                        b = by_key.get((run, variant, conditions[j]))
                        if a is None or b is None:
                            continue
                        visual_diff = float(a[f"visual_{metric}_mm"]) - float(b[f"visual_{metric}_mm"])
                        model_diff = float(a[f"model_{metric}_mm"]) - float(b[f"model_{metric}_mm"])
                        rank_rows.append(
                            {
                                "run": run,
                                "model_variant": variant,
                                "metric": metric,
                                "condition_a": conditions[i],
                                "condition_b": conditions[j],
                                "visual_diff_mm": format_float(visual_diff),
                                "model_diff_mm": format_float(model_diff),
                                "visual_sign": sign(visual_diff),
                                "model_sign": sign(model_diff),
                                "sign_match": int(sign(visual_diff) == sign(model_diff)),
                            }
                        )
    return rank_rows


def summarize_rank(rank_rows):
    summary = {}
    for row in rank_rows:
        key = (row["model_variant"], row["metric"])
        item = summary.setdefault(key, {"matched": 0, "total": 0})
        item["matched"] += int(row["sign_match"])
        item["total"] += 1
    rows = []
    for (variant, metric), item in sorted(summary.items()):
        total = item["total"]
        rows.append(
            {
                "model_variant": variant,
                "metric": metric,
                "matched": item["matched"],
                "total": total,
                "A_rank": item["matched"] / total if total else math.nan,
            }
        )
    return rows


def format_float(value):
    if value is None or not math.isfinite(float(value)):
        return "nan"
    return f"{float(value):.6g}"


def write_report(path, fidelity_rows, rank_summary, cfg, derived):
    lines = [
        "# Phase4 Model Fidelity Ablation",
        "",
        "## Inputs",
        "",
        "```text",
        f"R={cfg['R']:.6g} h={cfg['h']:.6g} rho={cfg['rho']:.6g} nu={cfg['nu']:.6g}",
        f"omega_n={derived['omega_n']:.6g}",
        f"height_coeff_observer={derived['height_coeff_observer']:.6g}",
        f"height_coeff_ferrari={derived['height_coeff_ferrari']:.6g}",
        f"zeta_current={cfg['zeta']:.6g}",
        f"zeta_ferrari={derived['zeta_ferrari']:.6g}",
        "```",
        "",
        "## A Rank",
        "",
        "| model_variant | metric | matched | total | A_rank | gate |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for row in rank_summary:
        a_rank = row["A_rank"]
        if a_rank >= 0.80:
            gate = "PASS"
        elif a_rank >= 0.70:
            gate = "SMOKE_ONLY"
        else:
            gate = "BLOCK_G_EXPANSION"
        lines.append(
            f"| {row['model_variant']} | {row['metric']} | {row['matched']} | "
            f"{row['total']} | {a_rank:.3f} | {gate} |"
        )
    lines.extend(
        [
            "",
            "## Notes",
            "",
        "- RGB visual metrics are the ground truth for real liquid height.",
        "- `/slosh/height` is used only as one model variant, not as the real-effect metric.",
        "- Visual values use `h_mm_smooth_corr` after bag motion start; this may differ from older 0424-style summary windows.",
        "- `A_rank < 0.70` blocks mainline G-family expansion; only report-only/offline smoke is allowed.",
            "",
            "## Per-Bag Snapshot",
            "",
            "| run | condition | model_variant | paired | dt_p95_ms | visual_p95 | model_p95 | corr | U_p95 | U_max |",
            "|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in fidelity_rows:
        lines.append(
            f"| {row['run']} | {row['condition']} | {row['model_variant']} | "
            f"{row['paired_samples']} | {row['pair_dt_p95_ms']} | "
            f"{row['visual_p95_mm']} | {row['model_p95_mm']} | "
            f"{row['corr_model_visual']} | {row['U_p95_mm']} | {row['U_max_mm']} |"
        )
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    args = parse_args()
    cfg = load_config(args.config)
    derived = derived_params(cfg)
    static_median = static_slosh_median(args.bag_dir)
    summary_rows = [
        row for row in read_csv(args.summary_csv)
        if row.get("run") in ("run01", "run02")
        and row.get("condition") in ("RAW", "FIXED_MILD", "OSCRS_MEDIUM_ACTIVE")
    ]
    variants = [
        ("M0_online_recorded", None),
        (
            "M0b_replay_observer",
            {
                "height_coeff": derived["height_coeff_observer"],
                "zeta": cfg["zeta"],
                "use_parabola": cfg["use_parabola_term"],
            },
        ),
        (
            "M1_ferrari_height_coeff",
            {
                "height_coeff": derived["height_coeff_ferrari"],
                "zeta": cfg["zeta"],
                "use_parabola": cfg["use_parabola_term"],
            },
        ),
        (
            "M2_parabola_off",
            {
                "height_coeff": derived["height_coeff_ferrari"],
                "zeta": cfg["zeta"],
                "use_parabola": False,
            },
        ),
        (
            "M3_ferrari_full",
            {
                "height_coeff": derived["height_coeff_ferrari"],
                "zeta": derived["zeta_ferrari"],
                "use_parabola": True,
            },
        ),
    ]

    fidelity_rows = []
    for item in summary_rows:
        bag_path = Path(args.bag_dir) / item["bag"]
        if not bag_path.exists():
            raise RuntimeError(f"missing bag: {bag_path}")
        visual_path = Path(item["visual_csv"])
        if not visual_path.exists():
            raise RuntimeError(f"missing visual csv: {visual_path}")
        visual = read_visual_series(visual_path, args.visual_column, args.min_confidence)
        series = read_bag_series(bag_path, static_median)
        with rosbag.Bag(str(bag_path)) as bag:
            bag_start = bag.get_start_time()
        analysis_start_abs = bag_start + read_start_times(bag_path)
        replay_cache = {}
        for variant, params in variants:
            if params is None:
                model_series = series["height"]
            else:
                key = (
                    params["height_coeff"],
                    params["zeta"],
                    params["use_parabola"],
                )
                if key not in replay_cache:
                    replay_cache[key] = replay_model(
                        series,
                        cfg,
                        derived["omega_n"],
                        params["height_coeff"],
                        params["zeta"],
                        params["use_parabola"],
                    )
                model_series = replay_cache[key]
            metrics = paired_metrics(visual, model_series, analysis_start_abs, args.max_pair_gap_sec)
            if metrics is None:
                continue
            row = {
                "run": item["run"],
                "condition": item["condition"],
                "bag": item["bag"],
                "visual_csv": str(visual_path),
                "model_variant": variant,
            }
            row.update({k: format_float(v) if isinstance(v, float) else v for k, v in metrics.items()})
            fidelity_rows.append(row)

    if not fidelity_rows:
        raise RuntimeError("no fidelity rows produced")
    rank_rows = build_rank_rows(fidelity_rows)
    rank_summary = summarize_rank(rank_rows)

    out_dir = Path(args.out_dir)
    write_csv(out_dir / "model_fidelity_ablation_phase4.csv", fidelity_rows)
    write_csv(out_dir / "model_selection_fidelity_phase4.csv", rank_rows)
    write_csv(out_dir / "A_rank_by_model_phase4.csv", [
        {
            "model_variant": row["model_variant"],
            "metric": row["metric"],
            "matched": row["matched"],
            "total": row["total"],
            "A_rank": format_float(row["A_rank"]),
        }
        for row in rank_summary
    ])
    write_report(out_dir / "A_rank_by_model_phase4.md", fidelity_rows, rank_summary, cfg, derived)
    print(f"wrote: {out_dir / 'model_fidelity_ablation_phase4.csv'}")
    print(f"wrote: {out_dir / 'model_selection_fidelity_phase4.csv'}")
    print(f"wrote: {out_dir / 'A_rank_by_model_phase4.md'}")
    for row in rank_summary:
        if row["metric"] == "p95":
            print(
                f"{row['model_variant']}: A_rank_p95={row['A_rank']:.3f} "
                f"({row['matched']}/{row['total']})"
            )


if __name__ == "__main__":
    main()
