#!/usr/bin/env python3
"""Diagnose TRACKING infeasible episodes from a rosbag by correlating solver failures
with path geometry spikes on /scout/global_path and /local_path.
"""

import argparse
import math
import statistics
from collections import defaultdict

import rosbag


TOPICS = [
    "/mpc/status_val",
    "/mpc_status",
    "/odom",
    "/scout/global_path",
    "/scout/global_path_smooth",
    "/mpc/reference_path",
    "/local_path",
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--tracking-only", action="store_true", default=True,
                        help="Only treat status_val failures during TRACKING as primary events")
    parser.add_argument("--curvature-threshold", type=float, default=5.0,
                        help="Heuristic suspicious max curvature threshold [1/m]")
    parser.add_argument("--dkappa-threshold", type=float, default=20.0,
                        help="Heuristic suspicious max curvature-rate threshold [1/m^2]")
    return parser.parse_args()


def summarize(values):
    if not values:
        return None
    vals = sorted(values)
    idx = max(0, int(0.95 * (len(vals) - 1)))
    return {
        "n": len(values),
        "min": min(values),
        "max": max(values),
        "mean": statistics.mean(values),
        "p95": vals[idx],
    }


def print_stats(name, values, unit=""):
    s = summarize(values)
    if s is None:
        print(f"- {name}: n=0")
        return
    print(
        f"- {name}: n={s['n']} min={s['min']:.4g}{unit} "
        f"max={s['max']:.4g}{unit} mean={s['mean']:.4g}{unit} "
        f"p95={s['p95']:.4g}{unit}"
    )


def nearest_value(series, t, idx=0):
    if not series:
        return idx, None
    while idx + 1 < len(series) and abs(series[idx + 1][0] - t) <= abs(series[idx][0] - t):
        idx += 1
    return idx, series[idx][1]


def path_points(msg):
    pts = []
    for pose_stamped in msg.poses:
        x = float(pose_stamped.pose.position.x)
        y = float(pose_stamped.pose.position.y)
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    while len(pts) >= 2 and dist(pts[-2], pts[-1]) < 1e-6:
        pts.pop()
    return pts


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def triangle_area2(a, b, c):
    return abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))


def curvature_three_points(a, b, c):
    ab = dist(a, b)
    bc = dist(b, c)
    ac = dist(a, c)
    denom = ab * bc * ac
    if denom < 1e-9:
        return None
    return 2.0 * triangle_area2(a, b, c) / denom


def analyze_polyline(points):
    if len(points) < 3:
        return {
            "n": len(points),
            "length": 0.0,
            "min_seg": 0.0,
            "max_kappa": 0.0,
            "p95_kappa": 0.0,
            "max_dkappa": 0.0,
            "short_seg_count": 0,
        }

    segs = [dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    length = sum(segs)
    min_seg = min(segs) if segs else 0.0
    short_seg_count = sum(1 for s in segs if s < 1e-3)

    kappas = []
    centers_s = []
    s_acc = 0.0
    for i in range(1, len(points) - 1):
        kappa = curvature_three_points(points[i - 1], points[i], points[i + 1])
        if kappa is None:
            s_acc += segs[i - 1]
            continue
        kappas.append(kappa)
        centers_s.append(s_acc + 0.5 * segs[i - 1])
        s_acc += segs[i - 1]

    if not kappas:
        return {
            "n": len(points),
            "length": length,
            "min_seg": min_seg,
            "max_kappa": 0.0,
            "p95_kappa": 0.0,
            "max_dkappa": 0.0,
            "short_seg_count": short_seg_count,
        }

    kappas_sorted = sorted(kappas)
    p95_idx = max(0, int(0.95 * (len(kappas_sorted) - 1)))
    max_dkappa = 0.0
    for i in range(1, len(kappas)):
        ds = max(1e-6, centers_s[i] - centers_s[i - 1])
        max_dkappa = max(max_dkappa, abs(kappas[i] - kappas[i - 1]) / ds)

    return {
        "n": len(points),
        "length": length,
        "min_seg": min_seg,
        "max_kappa": max(kappas),
        "p95_kappa": kappas_sorted[p95_idx],
        "max_dkappa": max_dkappa,
        "short_seg_count": short_seg_count,
    }


def main():
    args = parse_args()
    path_series = defaultdict(list)
    mpc_status = []
    status_val = []
    odom = []

    with rosbag.Bag(args.bag) as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            t = stamp.to_sec() - start
            if topic in ("/scout/global_path", "/scout/global_path_smooth", "/mpc/reference_path", "/local_path"):
                metrics = analyze_polyline(path_points(msg))
                path_series[topic].append((t, metrics))
            elif topic == "/mpc_status":
                mpc_status.append((t, str(msg.data)))
            elif topic == "/mpc/status_val":
                status_val.append((t, int(msg.data)))
            elif topic == "/odom":
                odom.append((t, (
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.angular.z),
                )))

    print(f"bag: {args.bag}")
    print(f"duration: {end - start:.2f}s")

    print("\nPath Geometry Overview")
    for topic in ("/scout/global_path", "/scout/global_path_smooth", "/mpc/reference_path", "/local_path"):
        max_k = [m["max_kappa"] for _, m in path_series[topic]]
        max_dk = [m["max_dkappa"] for _, m in path_series[topic]]
        min_seg = [m["min_seg"] for _, m in path_series[topic]]
        shorts = [m["short_seg_count"] for _, m in path_series[topic]]
        print(f"- {topic}:")
        print_stats("  max_kappa", max_k, " 1/m")
        print_stats("  max_dkappa", max_dk, " 1/m^2")
        print_stats("  min_seg", min_seg, " m")
        print_stats("  short_seg_count", shorts)

    mode_idx = 0
    odom_idx = 0
    path_idx = {topic: 0 for topic in path_series}
    fail_events = []
    for t, val in status_val:
        mode_idx, mode = nearest_value(mpc_status, t, mode_idx)
        if val == 1:
            continue
        if args.tracking_only and mode != "TRACKING":
            continue
        odom_idx, odom_sample = nearest_value(odom, t, odom_idx)
        event = {"t": t, "mode": mode, "odom": odom_sample}
        for topic in ("/scout/global_path", "/scout/global_path_smooth", "/mpc/reference_path", "/local_path"):
            path_idx[topic], metrics = nearest_value(path_series[topic], t, path_idx[topic])
            event[topic] = metrics
        fail_events.append(event)

    print("\nTRACKING Fail Events")
    if not fail_events:
        print("- none")
        return

    suspicious_local = 0
    suspicious_ref = 0
    suspicious_global = 0
    for i, event in enumerate(fail_events[:20], start=1):
        odom_sample = event["odom"] or (float("nan"), float("nan"))
        refp = event["/mpc/reference_path"] or {}
        local = event["/local_path"] or {}
        glob = event["/scout/global_path"] or {}
        if refp.get("max_kappa", 0.0) > args.curvature_threshold or \
           refp.get("max_dkappa", 0.0) > args.dkappa_threshold:
            suspicious_ref += 1
        if local.get("max_kappa", 0.0) > args.curvature_threshold or \
           local.get("max_dkappa", 0.0) > args.dkappa_threshold:
            suspicious_local += 1
        if glob.get("max_kappa", 0.0) > args.curvature_threshold or \
           glob.get("max_dkappa", 0.0) > args.dkappa_threshold:
            suspicious_global += 1
        print(
            f"- t={event['t']:.2f}s mode={event['mode']} "
            f"odom_v={odom_sample[0]:.3f} odom_w={odom_sample[1]:.3f} | "
            f"ref(max_k={refp.get('max_kappa', float('nan')):.3f}, "
            f"max_dk={refp.get('max_dkappa', float('nan')):.3f}, "
            f"min_seg={refp.get('min_seg', float('nan')):.4f}, "
            f"short={refp.get('short_seg_count', -1)}) | "
            f"local(max_k={local.get('max_kappa', float('nan')):.3f}, "
            f"max_dk={local.get('max_dkappa', float('nan')):.3f}, "
            f"min_seg={local.get('min_seg', float('nan')):.4f}, "
            f"short={local.get('short_seg_count', -1)}) | "
            f"global(max_k={glob.get('max_kappa', float('nan')):.3f}, "
            f"max_dk={glob.get('max_dkappa', float('nan')):.3f}, "
            f"min_seg={glob.get('min_seg', float('nan')):.4f})"
        )
    if len(fail_events) > 20:
        print(f"- ... {len(fail_events) - 20} more events")

    print("\nHeuristic Verdict")
    ref_ratio = suspicious_ref / len(fail_events)
    local_ratio = suspicious_local / len(fail_events)
    global_ratio = suspicious_global / len(fail_events)
    if ref_ratio > 0.5:
        print("- Likely primary issue: MPC reference path geometry spikes near TRACKING failures.")
    if local_ratio > 0.5:
        print("- Predicted local path also looks pathological near failures, likely downstream consequence.")
    if global_ratio > 0.5 and ref_ratio <= 0.5:
        print("- Global path geometry also looks aggressive near failures; inspect planner output or smoothing.")
    if ref_ratio <= 0.5 and local_ratio <= 0.5 and global_ratio <= 0.5:
        print("- No dominant path-geometry spike found from rosbag alone; inspect warm-start, constraints, and runtime logs.")


if __name__ == "__main__":
    main()
