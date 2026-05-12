#!/usr/bin/env python3
"""Check Day 3 T2 settling validation signals from a rosbag."""

import argparse
import statistics
from collections import Counter, defaultdict

import rosbag


TOPICS = [
    "/terminal/mode",
    "/mpc_status",
    "/slosh/settling_time",
    "/slosh/state",
    "/slosh/height",
    "/cmd_vel",
    "/odom",
    "/mpc/status_val",
    "/mpc/solve_ms",
    "/risk_scheduler/rho_k",
    "/risk_scheduler/u_k",
    "/risk_scheduler/Q_eta_k",
    "/risk_scheduler/fallback_active",
    "/scout/global_path",
    "/local_path",
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Require SETTLING, REACHED, and /slosh/settling_time for PASS",
    )
    return parser.parse_args()


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    idx = max(0, min(len(values) - 1, int(q * (len(values) - 1))))
    return values[idx]


def summarize(values):
    if not values:
        return None
    return {
        "n": len(values),
        "min": min(values),
        "max": max(values),
        "mean": statistics.mean(values),
        "p95": percentile(values, 0.95),
    }


def print_stats(name, values, unit=""):
    stats = summarize(values)
    if not stats:
        print(f"- {name}: n=0")
        return
    print(
        f"- {name}: n={stats['n']} "
        f"min={stats['min']:.4g}{unit} max={stats['max']:.4g}{unit} "
        f"mean={stats['mean']:.4g}{unit} p95={stats['p95']:.4g}{unit}"
    )


def contiguous_segments(samples, value):
    segments = []
    start_t = None
    last_t = None
    for t, sample in samples:
        if sample == value:
            if start_t is None:
                start_t = t
            last_t = t
        elif start_t is not None:
            segments.append((start_t, last_t))
            start_t = None
            last_t = None
    if start_t is not None:
        segments.append((start_t, last_t))
    return segments


def transitions(samples):
    result = []
    last = None
    for t, value in samples:
        if value != last:
            result.append((t, value))
            last = value
    return result


def main():
    args = parse_args()
    data = defaultdict(list)

    with rosbag.Bag(args.bag) as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            t = stamp.to_sec() - start
            if topic in ("/terminal/mode", "/mpc_status"):
                data[topic].append((t, str(msg.data)))
            elif topic == "/slosh/settling_time":
                data[topic].append((t, float(msg.data)))
            elif topic == "/slosh/state":
                data[topic].append((t, [float(v) for v in msg.data]))
            elif topic == "/cmd_vel":
                data[topic].append((t, (float(msg.linear.x), float(msg.angular.z))))
            elif topic == "/odom":
                data[topic].append((t, (
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.angular.z),
                )))
            elif topic in ("/scout/global_path", "/local_path"):
                data[topic].append((t, len(msg.poses)))
            else:
                data[topic].append((t, msg.data))

    duration = end - start
    print(f"bag: {args.bag}")
    print(f"duration: {duration:.2f}s")

    terminal = data["/terminal/mode"]
    mpc_status = data["/mpc_status"]
    settling_time = [v for _, v in data["/slosh/settling_time"]]
    cmd_v = [v for _, (v, _) in data["/cmd_vel"]]
    cmd_w = [w for _, (_, w) in data["/cmd_vel"]]
    odom_v = [v for _, (v, _) in data["/odom"]]
    odom_w = [w for _, (_, w) in data["/odom"]]
    solve_ms = [float(v) for _, v in data["/mpc/solve_ms"]]
    status_val = [int(v) for _, v in data["/mpc/status_val"]]
    fallback = [bool(v) for _, v in data["/risk_scheduler/fallback_active"]]
    rho = [float(v) for _, v in data["/risk_scheduler/rho_k"]]
    u_k = [float(v) for _, v in data["/risk_scheduler/u_k"]]
    q_eta = [float(v) for _, v in data["/risk_scheduler/Q_eta_k"]]
    height = [float(v) for _, v in data["/slosh/height"]]
    global_path_sizes = [int(v) for _, v in data["/scout/global_path"]]
    local_path_sizes = [int(v) for _, v in data["/local_path"]]

    print("\nTopic Counts")
    for topic in TOPICS:
        print(f"- {topic}: {len(data[topic])}")

    print("\nModes")
    for topic, samples in (("/terminal/mode", terminal), ("/mpc_status", mpc_status)):
        counts = Counter(v for _, v in samples)
        total = sum(counts.values()) or 1
        print(f"- {topic}:")
        for mode, count in counts.most_common():
            print(f"  {mode}: {count} ({count / total:.3f})")

    print("\nMode Transitions")
    for topic, samples in (("/terminal/mode", terminal), ("/mpc_status", mpc_status)):
        changes = transitions(samples)
        print(f"- {topic}: {len(changes)} transitions")
        for t, value in changes[:20]:
            print(f"  t={t:.2f}s -> {value}")
        if len(changes) > 20:
            print(f"  ... {len(changes) - 20} more transitions")
        if changes:
            t, value = changes[-1]
            print(f"  last: t={t:.2f}s -> {value}")

    settling_segments = contiguous_segments(terminal, "SETTLING")
    print("\nSettling")
    if settling_segments:
        for i, (t0, t1) in enumerate(settling_segments, 1):
            print(f"- SETTLING segment {i}: start={t0:.2f}s end={t1:.2f}s observed_dt={max(0.0, t1 - t0):.2f}s")
    else:
        print("- SETTLING segment: none")
    print_stats("/slosh/settling_time", settling_time, " s")

    print("\nSignals")
    print_stats("/cmd_vel.linear.x", cmd_v, " m/s")
    print_stats("/cmd_vel.angular.z", cmd_w, " rad/s")
    print_stats("/odom.twist.linear.x", odom_v, " m/s")
    print_stats("/odom.twist.angular.z", odom_w, " rad/s")
    print_stats("/slosh/height", height, " m")
    print_stats("/mpc/solve_ms", solve_ms, " ms")
    print_stats("/risk_scheduler/rho_k", rho)
    print_stats("/risk_scheduler/u_k", u_k)
    print_stats("/risk_scheduler/Q_eta_k", q_eta)
    print_stats("/scout/global_path poses", global_path_sizes)
    print_stats("/local_path poses", local_path_sizes)

    if status_val:
        success_ratio = sum(1 for v in status_val if v == 1) / len(status_val)
        print(f"- /mpc/status_val success_ratio={success_ratio:.3f}")
    else:
        success_ratio = 0.0
        print("- /mpc/status_val success_ratio=n/a")

    if fallback:
        fallback_ratio = sum(1 for v in fallback if v) / len(fallback)
        print(f"- /risk_scheduler/fallback_active true_ratio={fallback_ratio:.3f}")
    else:
        fallback_ratio = 0.0
        print("- /risk_scheduler/fallback_active true_ratio=n/a")

    print("\nVerdict")
    saw_settling = any(v == "SETTLING" for _, v in terminal)
    saw_reached = any(v == "REACHED" for _, v in terminal)
    saw_settling_time = bool(settling_time)
    saw_tracking = any(v == "TRACKING" for _, v in mpc_status)
    has_cmd = bool(cmd_v)

    checks = [
        ("terminal/mode has SETTLING", saw_settling),
        ("terminal/mode has REACHED", saw_reached),
        ("slosh/settling_time published", saw_settling_time),
        ("mpc_status has TRACKING", saw_tracking),
        ("cmd_vel present", has_cmd),
    ]
    for label, ok in checks:
        print(f"- {'PASS' if ok else 'FAIL'}: {label}")

    terminal_counts = Counter(v for _, v in terminal)
    terminal_total = sum(terminal_counts.values()) or 1
    terminal_recovery_ratio = sum(
        terminal_counts.get(mode, 0)
        for mode in (
            "APPROACH_POINT",
            "ALIGN_TO_POINT",
            "ALIGN_FINAL_YAW",
            "TERMINAL_LATCHED",
            "GOAL_STOP_PENDING",
        )
    ) / terminal_total
    error_ratio = terminal_counts.get("ERROR", 0) / terminal_total

    if args.strict:
        passed = all(ok for _, ok in checks)
    else:
        passed = saw_settling_time and saw_tracking and has_cmd
        if saw_settling_time and not saw_settling:
            print("- NOTE: /slosh/settling_time was present but SETTLING mode was not sampled in /terminal/mode.")
        if not saw_reached:
            print("- NOTE: REACHED was not sampled; check whether bag stopped immediately after settling.")
        if not saw_settling and terminal_recovery_ratio > 0.3:
            print("- HINT: terminal recovery occupied a large fraction of the run before goal pose was reached.")
        if not saw_settling and error_ratio > 0.1:
            print("- HINT: ERROR mode is frequent; inspect path validity, goal availability, or solver infeasibility near the terminal area.")
        if not saw_settling and data["/scout/global_path"]:
            last_path_t = data["/scout/global_path"][-1][0]
            if duration - last_path_t > 5.0:
                print("- HINT: global path was not refreshed near the end; sim path_timeout may be too short.")

    print(f"- overall: {'PASS' if passed else 'INCOMPLETE'}")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
