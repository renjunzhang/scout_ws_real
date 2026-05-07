#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Offline Step 0 check for slosh-model-guided GeoRef candidate scoring.

This script does not modify the online post-processor. It rebuilds the same
geometry candidates from each bag's raw MBF path, rolls out a linear modal
slosh model for every accepted candidate, and reports whether the slosh-score
winner agrees with the currently selected geometry-only candidate.
"""

import argparse
import csv
import math
import os
import re
import statistics

import rosbag

from extract_slosh_metrics import (
    get_status_segments,
    get_string_segments,
    in_terminal_none,
    in_tracking,
    percentile,
    rms,
    safe_max,
)
from generate_anti_slosh_path_candidates import (
    cumulative_s,
    curvature_series,
    dist,
    path_metrics,
    resample_path,
    smooth_path,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Diagnose whether linear slosh rollout is useful for GeoRef candidate scoring."
    )
    parser.add_argument("georef_bags", nargs="+", help="GEOREF bags to diagnose")
    parser.add_argument("--raw-bags", nargs="*", default=[], help="RAW baseline bags for actual direction check")
    parser.add_argument("--csv", default="/data/a/slosh_bags/analysis/slosh_guided_georef_step0.csv")
    parser.add_argument("--ds", type=float, default=0.03)
    parser.add_argument("--max-candidate-level", default="medium", choices=("original", "mild", "medium", "strong"))
    parser.add_argument("--min-segment-length", type=float, default=0.02)
    parser.add_argument("--max-drift", type=float, default=0.18)
    parser.add_argument("--max-length-ratio", type=float, default=1.15)
    parser.add_argument("--min-length-ratio", type=float, default=0.985)
    parser.add_argument("--min-kappa-ratio", type=float, default=0.20)
    parser.add_argument("--target-kappa-ratio", type=float, default=0.35)
    parser.add_argument("--max-endpoint-error", type=float, default=0.05)
    parser.add_argument("--ay-ratio-limit", type=float, default=1.0)
    parser.add_argument("--predict-v-max", type=float, default=2.0)
    parser.add_argument("--predict-ay-max", type=float, default=2.0)
    parser.add_argument("--predict-a-max", type=float, default=1.0)
    parser.add_argument("--predict-v-init", type=float, default=0.0)
    parser.add_argument("--omega-n", type=float, default=31.25)
    parser.add_argument("--zeta", type=float, default=0.05)
    parser.add_argument("--rollout-dt", type=float, default=0.05)
    parser.add_argument("--v-floor", type=float, default=0.05)
    parser.add_argument("--height-coeff", type=float, default=1.0)
    parser.add_argument("--w-h", type=float, default=0.0, help="Keep 0 by default; height coeff is not calibrated.")
    parser.add_argument("--w-energy", type=float, default=1.0)
    parser.add_argument("--w-eta-dot", type=float, default=0.5)
    parser.add_argument("--w-terminal", type=float, default=0.2)
    parser.add_argument("--w-kappa", type=float, default=1.0)
    parser.add_argument("--w-dkappa", type=float, default=0.5)
    parser.add_argument("--w-length", type=float, default=0.3)
    parser.add_argument("--w-drift", type=float, default=0.5)
    return parser.parse_args()


def bag_goal_key(path):
    name = os.path.basename(path)
    for key in ("open_user_goal", "open_goal_b", "open_goal_c", "open_goal_d"):
        if key in name:
            return key
    match = re.search(r"(P[0-9][A-Za-z0-9_]*|open_[A-Za-z0-9_]+)", name)
    return match.group(1) if match else "unknown"


def points_from_path(msg):
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def sanitize_points(points, min_segment_length):
    if not points:
        return []
    out = [points[0]]
    for point in points[1:]:
        if dist(out[-1], point) >= min_segment_length:
            out.append(point)
    return out


def endpoint_error(points, reference):
    if not points or not reference:
        return float("inf")
    return max(dist(points[0], reference[0]), dist(points[-1], reference[-1]))


def direction_preserved(points, reference):
    if len(points) < 2 or len(reference) < 2:
        return False
    ax = points[-1][0] - points[0][0]
    ay = points[-1][1] - points[0][1]
    bx = reference[-1][0] - reference[0][0]
    by = reference[-1][1] - reference[0][1]
    return ax * bx + ay * by >= 0.0


def safe_ratio(value, reference):
    denom = abs(reference)
    if denom < 1e-6:
        return 1.0 if abs(value) < 1e-6 else 1e6
    return value / denom


def mean_nearest_distance(a, b):
    if not a or not b:
        return float("inf")
    total = 0.0
    for point in a:
        total += min(dist(point, other) for other in b)
    return total / len(a)


def latest_path_from_bag(bag_path, topic_name):
    latest = None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic_name]):
            pts = points_from_path(msg)
            if len(pts) >= 2:
                latest = pts
    return latest


def closed_loop_metrics(bag_path):
    _, _, _, status_segments = get_status_segments(bag_path)
    _, _, _, terminal_segments = get_string_segments(bag_path, "/terminal/mode")
    values = {
        "height": [],
        "eta_dot": [],
        "energy_norm": [],
    }
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=["/slosh/height", "/slosh/state", "/slosh/modal_energy_norm"]):
            ts = t.to_sec()
            if not (in_tracking(status_segments, ts) and in_terminal_none(terminal_segments, ts)):
                continue
            if topic == "/slosh/height":
                values["height"].append(float(msg.data))
            elif topic == "/slosh/state" and len(msg.data) >= 4:
                values["eta_dot"].append(math.hypot(float(msg.data[1]), float(msg.data[3])))
            elif topic == "/slosh/modal_energy_norm":
                values["energy_norm"].append(float(msg.data))
    return {
        "height_rms": rms(values["height"]),
        "height_p95": percentile(values["height"], 95),
        "height_max": safe_max(values["height"]),
        "eta_dot_rms": rms(values["eta_dot"]),
        "energy_norm_rms": rms(values["energy_norm"]),
    }


def average_metrics(rows):
    if not rows:
        return {}
    out = {}
    for key in rows[0].keys():
        vals = [row[key] for row in rows if math.isfinite(row.get(key, float("nan")))]
        out[key] = statistics.mean(vals) if vals else float("nan")
    return out


def forward_profile(points, args):
    s = cumulative_s(points)
    kappa = curvature_series(points)
    v = []
    ax = []
    ay = []
    v_prev = min(args.predict_v_init, args.predict_v_max)
    for i, k in enumerate(kappa):
        ds = max(0.0, s[i] - s[i - 1]) if i > 0 else 0.0
        k_abs = abs(k)
        v_curv = math.sqrt(args.predict_ay_max / k_abs) if k_abs > 1e-6 else args.predict_v_max
        v_accel = math.sqrt(max(0.0, v_prev * v_prev + 2.0 * args.predict_a_max * ds))
        v_i = min(args.predict_v_max, v_curv, v_accel)
        ax_i = (v_i * v_i - v_prev * v_prev) / (2.0 * ds) if ds > 1e-6 else 0.0
        v.append(v_i)
        ax.append(ax_i)
        ay.append(v_i * v_i * k)
        v_prev = v_i
    return s, kappa, v, ax, ay


def interp_piecewise(times, values, t):
    if not times:
        return 0.0
    if t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    lo = 0
    hi = len(times) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    t0 = times[i - 1]
    t1 = times[i]
    if t1 <= t0:
        return values[i]
    r = (t - t0) / (t1 - t0)
    return values[i - 1] * (1.0 - r) + values[i] * r


def rollout_candidate(points, args):
    s, kappa, v, ax, ay = forward_profile(points, args)
    times = [0.0]
    for i in range(1, len(s)):
        ds = max(0.0, s[i] - s[i - 1])
        times.append(times[-1] + ds / max(args.v_floor, v[i]))

    eta_x = eta_x_dot = eta_y = eta_y_dot = 0.0
    eta_norm = []
    eta_dot_norm = []
    energy = []
    height = []
    t_end = times[-1] if times else 0.0
    steps = max(1, int(math.ceil(t_end / args.rollout_dt)))
    wn2 = args.omega_n * args.omega_n
    damp = 2.0 * args.zeta * args.omega_n
    for step in range(steps + 1):
        t = min(t_end, step * args.rollout_dt)
        ux = interp_piecewise(times, ax, t)
        uy = interp_piecewise(times, ay, t)
        ddx = -damp * eta_x_dot - wn2 * eta_x - ux
        ddy = -damp * eta_y_dot - wn2 * eta_y - uy
        eta_x_dot += ddx * args.rollout_dt
        eta_y_dot += ddy * args.rollout_dt
        eta_x += eta_x_dot * args.rollout_dt
        eta_y += eta_y_dot * args.rollout_dt
        en = math.hypot(eta_x, eta_y)
        ed = math.hypot(eta_x_dot, eta_y_dot)
        e = wn2 * (eta_x * eta_x + eta_y * eta_y) + eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot
        eta_norm.append(en)
        eta_dot_norm.append(ed)
        energy.append(e)
        height.append(args.height_coeff * en)
    dkappa = []
    for i in range(len(kappa)):
        if i == 0 or i + 1 >= len(kappa):
            dkappa.append(0.0)
        else:
            denom = max(1e-6, s[i + 1] - s[i - 1])
            dkappa.append((kappa[i + 1] - kappa[i - 1]) / denom)
    return {
        "pred_h_p95": percentile(height, 95),
        "pred_h_max": safe_max(height),
        "pred_eta_norm_p95": percentile(eta_norm, 95),
        "pred_eta_dot_rms": rms(eta_dot_norm),
        "pred_energy_rms": rms(energy),
        "pred_path_terminal_E": energy[-1] if energy else float("nan"),
        "pred_vmax": safe_max(v),
        "pred_active_time": t_end,
        "pred_ay_p95": percentile([abs(x) for x in ay], 95),
        "kappa_p95": percentile([abs(x) for x in kappa], 95),
        "dkappa_p95": percentile([abs(x) for x in dkappa], 95),
    }


def build_candidates(raw_points, args):
    raw = sanitize_points(raw_points, args.min_segment_length)
    base = resample_path(raw, max(0.02, args.ds))
    return [
        ("original", base),
        ("mild", smooth_path(base, 8, 0.20, 0.04)),
        ("medium", smooth_path(base, 40, 0.45, 0.12)),
        ("strong", smooth_path(base, 56, 0.55, 0.18)),
    ]


def evaluate_candidate(name, index, points, base, base_metrics, base_length, base_ay_p95, args):
    metrics = path_metrics(points, base)
    length_ratio = metrics["length_m"] / max(1e-6, base_length)
    kappa_ratio = safe_ratio(metrics["kappa_p95"], base_metrics["kappa_p95"])
    dkappa_ratio = safe_ratio(metrics["dkappa_p95"], base_metrics["dkappa_p95"])
    rollout = rollout_candidate(points, args)
    ay_ratio = safe_ratio(rollout["pred_ay_p95"], base_ay_p95)
    reasons = []
    levels = {"original": 0, "mild": 1, "medium": 2, "strong": 3}
    if levels[name] > levels[args.max_candidate_level]:
        reasons.append("level")
    if metrics["min_seg_m"] < args.min_segment_length:
        reasons.append("min_seg")
    if metrics["max_drift_m"] > args.max_drift:
        reasons.append("drift")
    if length_ratio > args.max_length_ratio:
        reasons.append("length")
    if length_ratio < args.min_length_ratio:
        reasons.append("short")
    if ay_ratio > args.ay_ratio_limit:
        reasons.append("ay")
    if endpoint_error(points, base) > args.max_endpoint_error:
        reasons.append("endpoint")
    if not direction_preserved(points, base):
        reasons.append("direction")
    shortening_penalty = max(0.0, args.min_length_ratio - length_ratio)
    over_smooth_penalty = max(0.0, args.min_kappa_ratio - kappa_ratio)
    geometry_score = (
        abs(kappa_ratio - args.target_kappa_ratio)
        + 0.5 * dkappa_ratio
        + 0.3 * max(0.0, length_ratio - 1.0)
        + 0.5 * metrics["max_drift_m"]
        + 10.0 * shortening_penalty
        + 2.0 * over_smooth_penalty
    )
    row = {
        "candidate": name,
        "candidate_index": index,
        "accepted": not reasons,
        "reject_reason": "accepted" if not reasons else "|".join(reasons),
        "geometry_score": geometry_score,
        "length_ratio": length_ratio,
        "max_drift_m": metrics["max_drift_m"],
        "kappa_ratio": kappa_ratio,
        "dkappa_ratio": dkappa_ratio,
        "pred_ay_ratio": ay_ratio,
    }
    row.update(rollout)
    return row


def add_norm_scores(rows, args):
    accepted = [r for r in rows if r["accepted"]]
    if len(accepted) < 2:
        for row in rows:
            row["slosh_score"] = row["geometry_score"]
        return
    keys = (
        "pred_h_p95",
        "pred_energy_rms",
        "pred_eta_dot_rms",
        "pred_path_terminal_E",
        "kappa_p95",
        "dkappa_p95",
        "length_ratio",
        "max_drift_m",
    )
    maxes = {}
    for key in keys:
        vals = [r[key] for r in accepted if math.isfinite(r[key])]
        maxes[key] = max(vals) if vals else 1.0
    for row in rows:
        if not row["accepted"]:
            row["slosh_score"] = float("inf")
            continue
        norm = lambda key: row[key] / max(1e-6, maxes[key])
        length_penalty = max(0.0, row["length_ratio"] - 1.0)
        row["slosh_score"] = (
            args.w_h * norm("pred_h_p95")
            + args.w_energy * norm("pred_energy_rms")
            + args.w_eta_dot * norm("pred_eta_dot_rms")
            + args.w_terminal * norm("pred_path_terminal_E")
            + args.w_kappa * norm("kappa_p95")
            + args.w_dkappa * norm("dkappa_p95")
            + args.w_length * length_penalty
            + args.w_drift * norm("max_drift_m")
        )


def diagnose_bag(bag_path, raw_baseline, args):
    raw_path = latest_path_from_bag(bag_path, "/scout/global_path")
    selected_path = latest_path_from_bag(bag_path, "/scout/global_path_anti_slosh")
    if not raw_path:
        raise RuntimeError(f"{bag_path}: missing /scout/global_path")
    candidates = build_candidates(raw_path, args)
    base = candidates[0][1]
    base_metrics = path_metrics(base, base)
    base_length = base_metrics["length_m"]
    base_ay_p95 = rollout_candidate(base, args)["pred_ay_p95"]
    rows = [
        evaluate_candidate(name, i, points, base, base_metrics, base_length, base_ay_p95, args)
        for i, (name, points) in enumerate(candidates)
    ]
    add_norm_scores(rows, args)
    geometry_winner = min((r for r in rows if r["accepted"]), key=lambda r: r["geometry_score"], default=rows[0])
    slosh_winner = min((r for r in rows if r["accepted"]), key=lambda r: r["slosh_score"], default=rows[0])
    actual_selected = "unknown"
    selected_match_m = float("nan")
    if selected_path:
        match = min(((name, mean_nearest_distance(selected_path, pts)) for name, pts in candidates), key=lambda x: x[1])
        actual_selected, selected_match_m = match
    actual = closed_loop_metrics(bag_path)
    raw = raw_baseline or {}
    goal = bag_goal_key(bag_path)
    selected_row = next((r for r in rows if r["candidate"] == actual_selected), geometry_winner)
    base_row = rows[0]
    out = {
        "bag": os.path.basename(bag_path),
        "goal_key": goal,
        "actual_selected": actual_selected,
        "selected_match_m": selected_match_m,
        "geometry_winner": geometry_winner["candidate"],
        "slosh_winner": slosh_winner["candidate"],
        "winner_agrees": int(slosh_winner["candidate"] == actual_selected),
        "slosh_non_original": int(slosh_winner["candidate"] != "original"),
        "selected_pred_h_ratio": safe_ratio(selected_row["pred_h_p95"], base_row["pred_h_p95"]),
        "selected_pred_energy_ratio": safe_ratio(selected_row["pred_energy_rms"], base_row["pred_energy_rms"]),
        "selected_pred_eta_dot_ratio": safe_ratio(selected_row["pred_eta_dot_rms"], base_row["pred_eta_dot_rms"]),
        "best_pred_h_extra_ratio": safe_ratio(slosh_winner["pred_h_p95"], selected_row["pred_h_p95"]),
        "best_pred_energy_extra_ratio": safe_ratio(slosh_winner["pred_energy_rms"], selected_row["pred_energy_rms"]),
        "actual_h_ratio_vs_raw": safe_ratio(actual.get("height_p95", float("nan")), raw.get("height_p95", float("nan"))),
        "actual_energy_ratio_vs_raw": safe_ratio(actual.get("energy_norm_rms", float("nan")), raw.get("energy_norm_rms", float("nan"))),
        "actual_eta_dot_ratio_vs_raw": safe_ratio(actual.get("eta_dot_rms", float("nan")), raw.get("eta_dot_rms", float("nan"))),
        "actual_h_p95": actual["height_p95"],
        "actual_energy_norm_rms": actual["energy_norm_rms"],
        "actual_eta_dot_rms": actual["eta_dot_rms"],
    }
    for row in rows:
        prefix = row["candidate"]
        out[f"{prefix}_accepted"] = int(row["accepted"])
        out[f"{prefix}_reject"] = row["reject_reason"]
        out[f"{prefix}_geom_score"] = row["geometry_score"]
        out[f"{prefix}_slosh_score"] = row["slosh_score"]
        out[f"{prefix}_pred_h_p95"] = row["pred_h_p95"]
        out[f"{prefix}_pred_energy_rms"] = row["pred_energy_rms"]
        out[f"{prefix}_pred_eta_dot_rms"] = row["pred_eta_dot_rms"]
        out[f"{prefix}_length_ratio"] = row["length_ratio"]
        out[f"{prefix}_pred_active_time"] = row["pred_active_time"]
    return out


def write_csv(path, rows):
    if not rows:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    keys = []
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value, digits=3):
    if isinstance(value, str):
        return value
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def main():
    args = parse_args()
    raw_by_goal = {}
    for bag in args.raw_bags:
        raw_by_goal.setdefault(bag_goal_key(bag), []).append(closed_loop_metrics(bag))
    raw_mean_by_goal = {goal: average_metrics(rows) for goal, rows in raw_by_goal.items()}
    rows = []
    for bag in args.georef_bags:
        row = diagnose_bag(bag, raw_mean_by_goal.get(bag_goal_key(bag)), args)
        rows.append(row)
        print(
            "{bag}: selected={sel} geom={geom} slosh={slosh} agree={agree} "
            "pred_h={ph} pred_E={pe} actual_h={ah}".format(
                bag=row["bag"],
                sel=row["actual_selected"],
                geom=row["geometry_winner"],
                slosh=row["slosh_winner"],
                agree=row["winner_agrees"],
                ph=fmt(row["selected_pred_h_ratio"]),
                pe=fmt(row["selected_pred_energy_ratio"]),
                ah=fmt(row["actual_h_ratio_vs_raw"]),
            )
        )
    write_csv(args.csv, rows)
    print(f"csv: {args.csv}")
    agreed = sum(row["winner_agrees"] for row in rows)
    non_original = sum(row["slosh_non_original"] for row in rows)
    print(f"summary: winner_agree={agreed}/{len(rows)} slosh_non_original={non_original}/{len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
