#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Step 0a diagnostic for GeoRef reference-budget execution gaps.

This script compares reference budgets derived from /scout/global_path_anti_slosh
against actual /cmd_vel and /odom execution on existing bags. It intentionally
does not use PathHandler's internal reference topics, because the Step 1 budget
source is the post-processor selected path.
"""

import argparse
import csv
import math
import os
import sys

import rosbag

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from generate_anti_slosh_path_candidates import cumulative_s, curvature_series, percentile


def parse_args():
    parser = argparse.ArgumentParser(
        description="Diagnose ref/cmd/odom a/jerk/omega/alpha gaps for GeoRef bags."
    )
    parser.add_argument("bags", nargs="+", help="GEOREF_TUNED bag files")
    parser.add_argument(
        "--csv",
        default="/data/a/slosh_bags/analysis/georef_budget_gap_step0a.csv",
        help="Output CSV path",
    )
    parser.add_argument("--dt", type=float, default=0.05, help="Resampling dt in seconds")
    parser.add_argument("--v-max", type=float, default=2.0, help="Post-processor prediction/v_max")
    parser.add_argument("--ay-budget", type=float, default=2.0, help="Post-processor prediction/ay_max_budget")
    parser.add_argument("--a-max", type=float, default=1.0, help="Post-processor prediction/a_max")
    parser.add_argument("--v-init", type=float, default=0.0, help="Post-processor prediction/v_init")
    parser.add_argument("--vehicle-a-max", type=float, default=2.0)
    parser.add_argument("--vehicle-j-max", type=float, default=4.0)
    parser.add_argument("--vehicle-omega-max", type=float, default=1.5)
    parser.add_argument("--vehicle-alpha-max", type=float, default=5.0)
    parser.add_argument("--gap-threshold", type=float, default=0.15)
    parser.add_argument("--bin-m", type=float, default=0.5, help="Path-s bin size for gap localization")
    parser.add_argument(
        "--min-top-bin-frac",
        type=float,
        default=0.25,
        help="Minimum positive-gap fraction in the dominant s-bin to count as localized",
    )
    return parser.parse_args()


def finite(values):
    return [v for v in values if math.isfinite(v)]


def safe_p95(values):
    values = finite(values)
    return percentile(values, 95.0) if values else float("nan")


def safe_max(values):
    values = finite(values)
    return max(values) if values else float("nan")


def safe_mean(values):
    values = finite(values)
    return sum(values) / len(values) if values else float("nan")


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def path_points(msg):
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def time_of(t):
    return t.to_sec() if hasattr(t, "to_sec") else float(t)


def interp_series(series, ts):
    if not series:
        return None
    if ts <= series[0][0]:
        return series[0][1]
    if ts >= series[-1][0]:
        return series[-1][1]
    lo = 0
    hi = len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < ts:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    t0, v0 = series[i - 1]
    t1, v1 = series[i]
    if t1 <= t0:
        return v1
    r = (ts - t0) / (t1 - t0)
    if isinstance(v0, tuple):
        return tuple(a * (1.0 - r) + b * r for a, b in zip(v0, v1))
    return v0 * (1.0 - r) + v1 * r


def nearest_discrete(series, ts):
    if not series:
        return None
    best = min(series, key=lambda item: abs(item[0] - ts))
    return best[1]


def nearest_path_s(point, points, s_values):
    if len(points) < 2:
        return 0.0
    px, py = point
    best_d2 = float("inf")
    best_s = 0.0
    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 <= 1e-12:
            continue
        u = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
        qx = ax + u * abx
        qy = ay + u * aby
        d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy)
        if d2 < best_d2:
            best_d2 = d2
            best_s = s_values[i] + u * (s_values[i + 1] - s_values[i])
    return best_s


def interp_profile(profile, s_query, key):
    s = profile["s"]
    values = profile[key]
    if not s:
        return float("nan")
    if s_query <= s[0]:
        return values[0]
    if s_query >= s[-1]:
        return values[-1]
    lo = 0
    hi = len(s) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if s[mid] < s_query:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    s0 = s[i - 1]
    s1 = s[i]
    if s1 <= s0:
        return values[i]
    r = (s_query - s0) / (s1 - s0)
    return values[i - 1] * (1.0 - r) + values[i] * r


def dkappa_series(points, kappa):
    s = cumulative_s(points)
    out = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        ds = max(1e-6, s[i + 1] - s[i - 1])
        out[i] = (kappa[i + 1] - kappa[i - 1]) / ds
    return out


def compute_ref_profile(points, v_max, ay_budget, a_max, v_init):
    s = cumulative_s(points)
    kappa = curvature_series(points)
    dkappa = dkappa_series(points, kappa)
    v = []
    a = [0.0] * len(points)
    jerk = [0.0] * len(points)
    omega = [0.0] * len(points)
    alpha = [0.0] * len(points)
    v_prev = min(max(0.0, v_init), v_max)
    for i, k in enumerate(kappa):
        ds = max(0.0, s[i] - s[i - 1]) if i > 0 else 0.0
        k_abs = abs(k)
        v_curv = math.sqrt(ay_budget / k_abs) if k_abs > 1e-6 else v_max
        v_accel = math.sqrt(max(0.0, v_prev * v_prev + 2.0 * a_max * ds))
        vi = min(v_max, v_curv, v_accel)
        v.append(vi)
        if i > 0:
            ds_eff = max(1e-6, ds)
            a[i] = (vi * vi - v[i - 1] * v[i - 1]) / (2.0 * ds_eff)
            jerk[i] = vi * (a[i] - a[i - 1]) / ds_eff
        omega[i] = vi * k
        alpha[i] = a[i] * k + vi * vi * dkappa[i]
        v_prev = vi
    return {
        "s": s,
        "v": v,
        "a": a,
        "jerk": jerk,
        "omega": omega,
        "alpha": alpha,
    }


def median_filter(values, window=5):
    if window <= 1 or len(values) < 3:
        return list(values)
    half = window // 2
    out = []
    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        chunk = sorted(values[lo:hi])
        out.append(chunk[len(chunk) // 2])
    return out


def diff(values, dt):
    out = [0.0] * len(values)
    for i in range(1, len(values)):
        out[i] = (values[i] - values[i - 1]) / dt
    return out


def load_bag(bag_path):
    topics = [
        "/scout/global_path_anti_slosh",
        "/cmd_vel",
        "/odom",
        "/mpc_status",
        "/terminal/mode",
        "/anti_slosh_path/candidate_report",
    ]
    data = {
        "path": None,
        "cmd": [],
        "odom": [],
        "status": [],
        "terminal": [],
        "report": "",
    }
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = time_of(t)
            if topic == "/scout/global_path_anti_slosh":
                data["path"] = path_points(msg)
            elif topic == "/cmd_vel":
                data["cmd"].append((ts, (float(msg.linear.x), float(msg.angular.z))))
            elif topic == "/odom":
                yaw = yaw_from_quat(msg.pose.pose.orientation)
                data["odom"].append((
                    ts,
                    (
                        float(msg.pose.pose.position.x),
                        float(msg.pose.pose.position.y),
                        float(msg.twist.twist.linear.x),
                        float(msg.twist.twist.angular.z),
                        yaw,
                    ),
                ))
            elif topic == "/mpc_status":
                data["status"].append((ts, str(msg.data)))
            elif topic == "/terminal/mode":
                data["terminal"].append((ts, str(msg.data)))
            elif topic == "/anti_slosh_path/candidate_report":
                data["report"] = str(msg.data)
    return data


def sample_execution(data, profile, dt):
    cmd = data["cmd"]
    odom = data["odom"]
    if not cmd or not odom or data["path"] is None:
        return []
    t0 = max(cmd[0][0], odom[0][0])
    t1 = min(cmd[-1][0], odom[-1][0])
    samples = []
    n = int(max(0.0, t1 - t0) / dt) + 1
    path_s = cumulative_s(data["path"])
    for i in range(n):
        ts = t0 + i * dt
        status = nearest_discrete(data["status"], ts)
        terminal = nearest_discrete(data["terminal"], ts)
        if status != "TRACKING" or terminal != "NONE":
            continue
        cmd_val = interp_series(cmd, ts)
        odom_val = interp_series(odom, ts)
        if cmd_val is None or odom_val is None:
            continue
        ox, oy, ov, ow, _ = odom_val
        s_proj = nearest_path_s((ox, oy), data["path"], path_s)
        samples.append({
            "t": ts,
            "s": s_proj,
            "cmd_v": cmd_val[0],
            "cmd_w": cmd_val[1],
            "odom_v": ov,
            "odom_w": ow,
            "ref_a": interp_profile(profile, s_proj, "a"),
            "ref_jerk": interp_profile(profile, s_proj, "jerk"),
            "ref_omega": interp_profile(profile, s_proj, "omega"),
            "ref_alpha": interp_profile(profile, s_proj, "alpha"),
        })
    return samples


def positive_gap_distribution(s_values, ref_values, cmd_values, odom_values, bin_m):
    bins = {}
    positive_count = 0
    for s, ref, cmd, odom in zip(s_values, ref_values, cmd_values, odom_values):
        if not all(math.isfinite(x) for x in (s, ref, cmd, odom)):
            continue
        gap = max(abs(cmd), abs(odom)) - abs(ref)
        if gap <= 0.0:
            continue
        positive_count += 1
        idx = int(math.floor(s / max(1e-6, bin_m)))
        if idx not in bins:
            bins[idx] = {"count": 0, "gap_sum": 0.0, "gap_max": 0.0}
        bins[idx]["count"] += 1
        bins[idx]["gap_sum"] += gap
        bins[idx]["gap_max"] = max(bins[idx]["gap_max"], gap)
    if not bins or positive_count == 0:
        return {
            "positive_count": 0,
            "top_center_s": float("nan"),
            "top_fraction": 0.0,
            "top_gap_mean": float("nan"),
            "top_gap_max": float("nan"),
        }
    top_idx, top = max(bins.items(), key=lambda item: (item[1]["count"], item[1]["gap_max"]))
    return {
        "positive_count": positive_count,
        "top_center_s": (top_idx + 0.5) * bin_m,
        "top_fraction": top["count"] / float(positive_count),
        "top_gap_mean": top["gap_sum"] / float(top["count"]),
        "top_gap_max": top["gap_max"],
    }


def gap_summary(name, s_values, ref_values, cmd_values, odom_values, vehicle_limit, threshold, bin_m, min_top_bin_frac):
    ref_abs = [abs(x) for x in ref_values]
    cmd_abs = [abs(x) for x in cmd_values]
    odom_abs = [abs(x) for x in odom_values]
    ref_p95 = safe_p95(ref_abs)
    cmd_p95 = safe_p95(cmd_abs)
    odom_p95 = safe_p95(odom_abs)
    abs_gap = max(cmd_p95 - ref_p95, odom_p95 - ref_p95)
    rel_gap = abs_gap / vehicle_limit if vehicle_limit > 1e-9 else float("nan")
    dist = positive_gap_distribution(s_values, ref_values, cmd_values, odom_values, bin_m)
    localized = dist["top_fraction"] >= min_top_bin_frac and dist["positive_count"] > 0
    triggered = bool(math.isfinite(rel_gap) and rel_gap >= threshold and localized)
    return {
        f"ref_{name}_p95": ref_p95,
        f"cmd_{name}_p95": cmd_p95,
        f"odom_{name}_p95": odom_p95,
        f"gap_{name}_abs": abs_gap,
        f"gap_{name}_rel": rel_gap,
        f"gap_{name}_positive_count": dist["positive_count"],
        f"gap_{name}_top_s": dist["top_center_s"],
        f"gap_{name}_top_fraction": dist["top_fraction"],
        f"gap_{name}_top_mean": dist["top_gap_mean"],
        f"gap_{name}_top_max": dist["top_gap_max"],
        f"trigger_{name}": int(triggered),
    }


def analyze_bag(bag_path, args):
    data = load_bag(bag_path)
    if not data["path"] or len(data["path"]) < 3:
        raise RuntimeError(f"{bag_path}: missing /scout/global_path_anti_slosh")
    profile = compute_ref_profile(data["path"], args.v_max, args.ay_budget, args.a_max, args.v_init)
    samples = sample_execution(data, profile, args.dt)
    raw_count = len(samples)
    if len(samples) < 3:
        raise RuntimeError(f"{bag_path}: too few TRACKING/NONE samples ({len(samples)})")

    cmd_v = median_filter([s["cmd_v"] for s in samples])
    odom_v = median_filter([s["odom_v"] for s in samples])
    cmd_w = median_filter([s["cmd_w"] for s in samples])
    odom_w = median_filter([s["odom_w"] for s in samples])
    cmd_a = median_filter(diff(cmd_v, args.dt))
    odom_a = median_filter(diff(odom_v, args.dt))
    cmd_jerk = median_filter(diff(cmd_a, args.dt))
    odom_jerk = median_filter(diff(odom_a, args.dt))
    cmd_alpha = median_filter(diff(cmd_w, args.dt))
    odom_alpha = median_filter(diff(odom_w, args.dt))

    ref = {
        "a": [s["ref_a"] for s in samples],
        "jerk": [s["ref_jerk"] for s in samples],
        "omega": [s["ref_omega"] for s in samples],
        "alpha": [s["ref_alpha"] for s in samples],
    }
    s_values = [s["s"] for s in samples]
    row = {
        "bag": os.path.basename(bag_path),
        "samples": raw_count,
        "path_points": len(data["path"]),
        "path_length_m": profile["s"][-1] if profile["s"] else float("nan"),
        "candidate_report": data["report"],
        "inconclusive": int(raw_count < 100),
    }
    row.update(gap_summary("a", s_values, ref["a"], cmd_a, odom_a, args.vehicle_a_max, args.gap_threshold, args.bin_m, args.min_top_bin_frac))
    row.update(gap_summary("jerk", s_values, ref["jerk"], cmd_jerk, odom_jerk, args.vehicle_j_max, args.gap_threshold, args.bin_m, args.min_top_bin_frac))
    row.update(gap_summary("omega", s_values, ref["omega"], cmd_w, odom_w, args.vehicle_omega_max, args.gap_threshold, args.bin_m, args.min_top_bin_frac))
    row.update(gap_summary("alpha", s_values, ref["alpha"], cmd_alpha, odom_alpha, args.vehicle_alpha_max, args.gap_threshold, args.bin_m, args.min_top_bin_frac))

    triggers = [name for name in ("a", "jerk", "omega", "alpha") if row[f"trigger_{name}"]]
    if not triggers:
        verdict = "STOP_NO_GAP"
    elif any(x in triggers for x in ("omega", "alpha")) and not any(x in triggers for x in ("a", "jerk")):
        verdict = "OMEGA_ALPHA_BRANCH"
    elif any(x in triggers for x in ("a", "jerk")) and not any(x in triggers for x in ("omega", "alpha")):
        verdict = "A_JERK_BRANCH"
    else:
        verdict = "MULTI_CHANNEL_BRANCH"
    row["triggered_channels"] = ",".join(triggers)
    row["verdict"] = verdict
    return row


def write_csv(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fieldnames = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows):
    print("Step 0a GeoRef budget gap diagnostic")
    for row in rows:
        print(
            f"- {row['bag']}: samples={row['samples']} verdict={row['verdict']} "
            f"triggers={row['triggered_channels'] or '-'} "
            f"gap_a={row['gap_a_rel']:.3f} gap_j={row['gap_jerk_rel']:.3f} "
            f"gap_w={row['gap_omega_rel']:.3f} gap_alpha={row['gap_alpha_rel']:.3f} "
            f"top_s(j/w)=({row['gap_jerk_top_s']:.2f},{row['gap_omega_top_s']:.2f})"
        )
    trigger_counts = {name: 0 for name in ("a", "jerk", "omega", "alpha")}
    for row in rows:
        for name in trigger_counts.keys():
            trigger_counts[name] += int(row[f"trigger_{name}"])
    min_consistent = 2 if len(rows) >= 3 else 1
    a_jerk_consistent = any(trigger_counts[name] >= min_consistent for name in ("a", "jerk"))
    omega_alpha_consistent = any(trigger_counts[name] >= min_consistent for name in ("omega", "alpha"))
    if not a_jerk_consistent and not omega_alpha_consistent:
        any_single = any(count > 0 for count in trigger_counts.values())
        overall = "EDGE_INCONCLUSIVE" if any_single else "STOP_NO_GAP"
    elif a_jerk_consistent and omega_alpha_consistent:
        overall = "MULTI_CHANNEL_BRANCH"
    elif omega_alpha_consistent:
        overall = "OMEGA_ALPHA_BRANCH"
    else:
        overall = "A_JERK_BRANCH"
    if all(row["verdict"] == "STOP_NO_GAP" for row in rows) and overall == "EDGE_INCONCLUSIVE":
        overall = "STOP_NO_GAP"
    print(
        "trigger_counts: "
        + " ".join(f"{name}={count}" for name, count in trigger_counts.items())
        + f" min_consistent={min_consistent}"
    )
    print(f"overall: {overall}")


def main():
    args = parse_args()
    rows = []
    for bag_path in args.bags:
        rows.append(analyze_bag(bag_path, args))
    write_csv(args.csv, rows)
    print_summary(rows)
    print(f"csv: {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
