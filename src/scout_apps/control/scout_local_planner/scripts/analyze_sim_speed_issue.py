#!/usr/bin/env python3
"""Diagnose why a simulated MPC run is slow from a rosbag."""

import argparse
import math
import statistics
from collections import Counter, defaultdict

import rosbag


TOPICS = [
    "/cmd_vel",
    "/odom",
    "/slosh/v_des_eff",
    "/risk_scheduler/rho_k",
    "/risk_scheduler/Q_eta_k",
    "/risk_scheduler/fallback_active",
    "/mpc/status_val",
    "/mpc/solve_ms",
    "/mpc_status",
    "/terminal/mode",
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--slow-threshold", type=float, default=0.25,
                        help="Linear speed below this is considered slow [m/s]")
    return parser.parse_args()


def summarize(values):
    if not values:
        return None
    return {
        "n": len(values),
        "min": min(values),
        "max": max(values),
        "mean": statistics.mean(values),
        "p95": sorted(values)[max(0, int(0.95 * (len(values) - 1)))],
    }


def print_stats(name, values, unit=""):
    stats = summarize(values)
    if stats is None:
        print(f"- {name}: n=0")
        return
    print(
        f"- {name}: n={stats['n']} min={stats['min']:.4g}{unit} "
        f"max={stats['max']:.4g}{unit} mean={stats['mean']:.4g}{unit} "
        f"p95={stats['p95']:.4g}{unit}"
    )


def nearest(series, t, idx):
    if not series:
        return idx, None
    while idx + 1 < len(series) and abs(series[idx + 1][0] - t) <= abs(series[idx][0] - t):
        idx += 1
    return idx, series[idx][1]


def main():
    args = parse_args()
    data = defaultdict(list)

    with rosbag.Bag(args.bag) as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            t = stamp.to_sec() - start
            if topic == "/cmd_vel":
                data[topic].append((t, (float(msg.linear.x), float(msg.angular.z))))
            elif topic == "/odom":
                data[topic].append((t, (
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.angular.z),
                )))
            elif topic in ("/mpc_status", "/terminal/mode"):
                data[topic].append((t, str(msg.data)))
            else:
                data[topic].append((t, msg.data))

    duration = end - start if "end" in locals() else 0.0
    print(f"bag: {args.bag}")
    print(f"duration: {duration:.2f}s")

    cmd_v = [v for _, (v, _) in data["/cmd_vel"]]
    cmd_w = [w for _, (_, w) in data["/cmd_vel"]]
    odom_v = [v for _, (v, _) in data["/odom"]]
    v_des = [float(v) for _, v in data["/slosh/v_des_eff"]]
    rho = [float(v) for _, v in data["/risk_scheduler/rho_k"]]
    q_eta = [float(v) for _, v in data["/risk_scheduler/Q_eta_k"]]
    solve_ms = [float(v) for _, v in data["/mpc/solve_ms"]]
    status = [int(v) for _, v in data["/mpc/status_val"]]
    fallback = [bool(v) for _, v in data["/risk_scheduler/fallback_active"]]

    print("\nSignals")
    print_stats("/cmd_vel.linear.x", cmd_v, " m/s")
    print_stats("/cmd_vel.angular.z", cmd_w, " rad/s")
    print_stats("/odom.twist.linear.x", odom_v, " m/s")
    print_stats("/slosh/v_des_eff", v_des, " m/s")
    print_stats("/risk_scheduler/rho_k", rho)
    print_stats("/risk_scheduler/Q_eta_k", q_eta)
    print_stats("/mpc/solve_ms", solve_ms, " ms")

    if status:
        print(f"- /mpc/status_val success_ratio={sum(1 for v in status if v == 1) / len(status):.3f}")
    if fallback:
        print(f"- /risk_scheduler/fallback_active true_ratio={sum(1 for v in fallback if v) / len(fallback):.3f}")

    print("\nModes")
    for topic in ("/mpc_status", "/terminal/mode"):
        counts = Counter(v for _, v in data[topic])
        total = sum(counts.values())
        print(f"- {topic}:")
        for mode, count in counts.most_common():
            print(f"  {mode}: {count} ({count / total:.3f})")

    print("\nSlow Samples")
    terminal_idx = 0
    vdes_idx = 0
    rho_idx = 0
    fallback_idx = 0
    samples = []
    for t, (v, w) in data["/cmd_vel"]:
        if abs(v) >= args.slow_threshold:
            continue
        terminal_idx, terminal_mode = nearest(data["/terminal/mode"], t, terminal_idx)
        vdes_idx, vdes_sample = nearest(data["/slosh/v_des_eff"], t, vdes_idx)
        rho_idx, rho_sample = nearest(data["/risk_scheduler/rho_k"], t, rho_idx)
        fallback_idx, fallback_sample = nearest(data["/risk_scheduler/fallback_active"], t, fallback_idx)
        samples.append((t, v, w, terminal_mode, vdes_sample, rho_sample, fallback_sample))

    for row in samples[:20]:
        t, v, w, terminal_mode, vdes_sample, rho_sample, fallback_sample = row
        print(
            f"- t={t:.2f}s cmd_v={v:.3f} cmd_w={w:.3f} "
            f"terminal={terminal_mode} v_des_eff={vdes_sample} "
            f"rho={rho_sample} fallback={fallback_sample}"
        )
    if len(samples) > 20:
        print(f"- ... {len(samples) - 20} more slow samples")

    print("\nHeuristic Verdict")
    max_cmd = max(cmd_v) if cmd_v else float("nan")
    max_vdes = max(v_des) if v_des else float("nan")
    terminal_counts = Counter(v for _, v in data["/terminal/mode"])
    terminal_total = sum(terminal_counts.values()) or 1
    terminal_ratio = sum(
        terminal_counts.get(mode, 0)
        for mode in ("APPROACH_POINT", "ALIGN_TO_POINT", "ALIGN_FINAL_YAW", "TERMINAL_LATCHED", "GOAL_STOP_PENDING")
    ) / terminal_total
    fallback_ratio = sum(1 for v in fallback if v) / len(fallback) if fallback else 0.0

    if terminal_ratio > 0.3:
        print("- Likely speed bottleneck: terminal recovery is active for a large fraction of the run.")
    if not math.isnan(max_vdes) and max_vdes < args.slow_threshold:
        print("- Likely speed bottleneck: effective reference speed is low before MPC output.")
    if fallback_ratio > 0.3:
        print("- Risk scheduler fallback is active often; check IMU topic/bias readiness.")
    if not math.isnan(max_cmd) and not math.isnan(max_vdes) and max_vdes > args.slow_threshold and max_cmd < args.slow_threshold:
        print("- MPC/control output is slower than the reference; inspect constraints, terminal mode, and acceleration limits.")
    if terminal_ratio <= 0.3 and fallback_ratio <= 0.3 and (math.isnan(max_vdes) or max_vdes >= args.slow_threshold):
        print("- No single obvious software bottleneck from these topics; inspect path curvature and Gazebo physics limits.")


if __name__ == "__main__":
    main()
