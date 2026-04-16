#!/usr/bin/env python3
"""Offline Day3 A/B/C simulation smoke-test checker."""

import argparse
import os
import statistics
from collections import Counter, defaultdict

import rosbag


TOPICS = [
    "/imu/data",
    "/slosh/imu_ay_filtered",
    "/slosh/imu_ay_bias",
    "/slosh/imu_ay_bias_ready",
    "/slosh/ay_est",
    "/slosh/omega_est_used",
    "/slosh/height",
    "/slosh/settling_time",
    "/risk_scheduler/rho_k",
    "/risk_scheduler/u_k",
    "/risk_scheduler/Q_eta_k",
    "/risk_scheduler/fallback_active",
    "/mpc/status_val",
    "/mpc/solve_ms",
    "/mpc_status",
    "/terminal/mode",
    "/cmd_vel",
    "/odom",
]


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    idx = max(0, min(len(values) - 1, int(q * (len(values) - 1))))
    return values[idx]


def stats(values):
    if not values:
        return None
    return {
        "n": len(values),
        "min": min(values),
        "max": max(values),
        "mean": statistics.mean(values),
        "p95": percentile(values, 0.95),
    }


def fmt_stats(name, values, unit=""):
    s = stats(values)
    if not s:
        return f"- {name}: n=0"
    return (
        f"- {name}: n={s['n']} min={s['min']:.4g}{unit} "
        f"max={s['max']:.4g}{unit} mean={s['mean']:.4g}{unit} "
        f"p95={s['p95']:.4g}{unit}"
    )


def infer_case(path):
    name = os.path.basename(path).lower()
    tokens = name.replace("-", "_").replace(".", "_").split("_")
    for token in tokens:
        if token in ("a", "casea", "abcA".lower()):
            return "A"
        if token in ("b", "caseb", "abcB".lower()):
            return "B"
        if token in ("c", "casec", "abcC".lower()):
            return "C"
    if "case_a" in name or "abc_a" in name:
        return "A"
    if "case_b" in name or "abc_b" in name:
        return "B"
    if "case_c" in name or "abc_c" in name:
        return "C"
    return "?"


def parse_case_arg(value):
    if "=" not in value:
        return infer_case(value), value
    case, path = value.split("=", 1)
    return case.strip().upper(), path.strip()


def expand_inputs(values):
    cases = []
    for value in values:
        case, path = parse_case_arg(value)
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.endswith(".bag"):
                    bag_path = os.path.join(path, name)
                    inferred = infer_case(bag_path)
                    cases.append((inferred if case == "?" else case, bag_path))
        elif os.path.isfile(path) and path.endswith(".bag"):
            cases.append((case, path))
    return cases


def rate(samples):
    if len(samples) < 2:
        return None
    dt = samples[-1][0] - samples[0][0]
    if dt <= 0.0:
        return None
    return (len(samples) - 1) / dt


def analyze_bag(path):
    data = defaultdict(list)
    with rosbag.Bag(path) as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            t = stamp.to_sec() - start
            if topic == "/imu/data":
                data[topic].append((t, {
                    "frame_id": msg.header.frame_id,
                    "ay": float(msg.linear_acceleration.y),
                    "omega_z": float(msg.angular_velocity.z),
                }))
            elif topic in ("/mpc_status", "/terminal/mode"):
                data[topic].append((t, str(msg.data)))
            elif topic == "/cmd_vel":
                data[topic].append((t, (float(msg.linear.x), float(msg.angular.z))))
            elif topic == "/odom":
                data[topic].append((t, (
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.angular.z),
                )))
            elif topic == "/risk_scheduler/fallback_active":
                data[topic].append((t, bool(msg.data)))
            else:
                data[topic].append((t, msg.data))

    duration = end - start
    imu = data["/imu/data"]
    imu_ay = [row["ay"] for _, row in imu]
    imu_omega = [row["omega_z"] for _, row in imu]
    imu_frames = sorted(set(row["frame_id"] for _, row in imu))
    bias_ready = [int(v) for _, v in data["/slosh/imu_ay_bias_ready"]]
    fallback = [bool(v) for _, v in data["/risk_scheduler/fallback_active"]]
    status_val = [int(v) for _, v in data["/mpc/status_val"]]
    mpc_modes = [v for _, v in data["/mpc_status"]]
    terminal_modes = [v for _, v in data["/terminal/mode"]]
    cmd_v = [v for _, (v, _) in data["/cmd_vel"]]
    cmd_w = [w for _, (_, w) in data["/cmd_vel"]]
    odom_v = [v for _, (v, _) in data["/odom"]]

    result = {
        "duration": duration,
        "counts": {topic: len(data[topic]) for topic in TOPICS},
        "imu_rate": rate(imu),
        "imu_frames": imu_frames,
        "imu_ay": imu_ay,
        "imu_omega": imu_omega,
        "imu_ay_filtered": [float(v) for _, v in data["/slosh/imu_ay_filtered"]],
        "imu_ay_bias": [float(v) for _, v in data["/slosh/imu_ay_bias"]],
        "bias_ready_ratio": (sum(1 for v in bias_ready if v > 0) / len(bias_ready)) if bias_ready else None,
        "ay_est": [float(v) for _, v in data["/slosh/ay_est"]],
        "omega_est_used": [float(v) for _, v in data["/slosh/omega_est_used"]],
        "height": [float(v) for _, v in data["/slosh/height"]],
        "settling_time": [float(v) for _, v in data["/slosh/settling_time"]],
        "rho_k": [float(v) for _, v in data["/risk_scheduler/rho_k"]],
        "u_k": [float(v) for _, v in data["/risk_scheduler/u_k"]],
        "q_eta": [float(v) for _, v in data["/risk_scheduler/Q_eta_k"]],
        "fallback_ratio": (sum(1 for v in fallback if v) / len(fallback)) if fallback else None,
        "mpc_success_ratio": (sum(1 for v in status_val if v == 1) / len(status_val)) if status_val else None,
        "solve_ms": [float(v) for _, v in data["/mpc/solve_ms"]],
        "mpc_modes": Counter(mpc_modes),
        "terminal_modes": Counter(terminal_modes),
        "cmd_v": cmd_v,
        "cmd_w": cmd_w,
        "odom_v": odom_v,
    }
    return result


def case_verdict(case, result):
    checks = []
    checks.append(("IMU /imu/data present", result["counts"]["/imu/data"] > 0))
    checks.append(("IMU frame is imu_link", result["imu_frames"] == ["imu_link"]))
    checks.append(("MPC produced cmd_vel", result["counts"]["/cmd_vel"] > 0))
    checks.append(("MPC reached TRACKING", result["mpc_modes"].get("TRACKING", 0) > 0))
    checks.append(("MPC did not obviously fail all samples", (result["mpc_success_ratio"] or 0.0) > 0.05))

    if case in ("B", "C"):
        checks.append(("IMU ay filtered topic present", result["counts"]["/slosh/imu_ay_filtered"] > 0))
        checks.append(("IMU ay bias became ready", (result["bias_ready_ratio"] or 0.0) > 0.5))
    if case == "C":
        checks.append(("risk u_k topic present", result["counts"]["/risk_scheduler/u_k"] > 0))
        checks.append(("risk u_k has nonzero response", max(result["u_k"] or [0.0]) > 1e-6))
        checks.append(("risk fallback not dominant", (result["fallback_ratio"] or 0.0) < 0.5))
    return checks


def print_case(case, path, result):
    print("=" * 72)
    print(f"case {case}: {path}")
    print(f"duration: {result['duration']:.2f}s")
    print("\nTopic Counts")
    for topic in TOPICS:
        print(f"- {topic}: {result['counts'][topic]}")
    print("\nSignals")
    print(f"- /imu/data rate: {result['imu_rate']:.3f} Hz" if result["imu_rate"] else "- /imu/data rate: n/a")
    print(f"- /imu/data frames: {result['imu_frames']}")
    print(fmt_stats("/imu/data.linear_acceleration.y", result["imu_ay"], " m/s^2"))
    print(fmt_stats("/slosh/imu_ay_filtered", result["imu_ay_filtered"], " m/s^2"))
    print(fmt_stats("/slosh/imu_ay_bias", result["imu_ay_bias"], " m/s^2"))
    if result["bias_ready_ratio"] is not None:
        print(f"- /slosh/imu_ay_bias_ready true_ratio={result['bias_ready_ratio']:.3f}")
    else:
        print("- /slosh/imu_ay_bias_ready true_ratio=n/a")
    print(fmt_stats("/slosh/ay_est", result["ay_est"], " m/s^2"))
    print(fmt_stats("/slosh/omega_est_used", result["omega_est_used"], " rad/s"))
    print(fmt_stats("/slosh/height", result["height"], " m"))
    print(fmt_stats("/risk_scheduler/u_k", result["u_k"]))
    print(fmt_stats("/risk_scheduler/rho_k", result["rho_k"]))
    print(fmt_stats("/risk_scheduler/Q_eta_k", result["q_eta"]))
    if result["fallback_ratio"] is not None:
        print(f"- /risk_scheduler/fallback_active true_ratio={result['fallback_ratio']:.3f}")
    else:
        print("- /risk_scheduler/fallback_active true_ratio=n/a")
    if result["mpc_success_ratio"] is not None:
        print(f"- /mpc/status_val success_ratio={result['mpc_success_ratio']:.3f}")
    else:
        print("- /mpc/status_val success_ratio=n/a")
    print(fmt_stats("/mpc/solve_ms", result["solve_ms"], " ms"))
    print(fmt_stats("/cmd_vel.linear.x", result["cmd_v"], " m/s"))
    print(fmt_stats("/cmd_vel.angular.z", result["cmd_w"], " rad/s"))
    print(fmt_stats("/odom.twist.linear.x", result["odom_v"], " m/s"))
    print("\nModes")
    print(f"- /mpc_status: {dict(result['mpc_modes'])}")
    print(f"- /terminal/mode: {dict(result['terminal_modes'])}")
    print("\nVerdict")
    checks = case_verdict(case, result)
    for label, ok in checks:
        print(f"- {'PASS' if ok else 'FAIL'}: {label}")
    overall = all(ok for _, ok in checks)
    print(f"- overall: {'PASS' if overall else 'INCOMPLETE'}")
    return overall


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="+",
        help="Bag paths or directories. Use A=/path.bag B=/path.bag C=/path.bag for explicit labels.",
    )
    args = parser.parse_args()

    cases = expand_inputs(args.inputs)
    if not cases:
        print("No bag files found.")
        return 1

    overall = True
    seen_cases = set()
    for case, path in cases:
        result = analyze_bag(path)
        seen_cases.add(case)
        overall = print_case(case, path, result) and overall

    expected = {"A", "B", "C"}
    missing = sorted(expected - seen_cases)
    if missing:
        print("=" * 72)
        print(f"Missing cases: {', '.join(missing)}")
        overall = False

    return 0 if overall else 2


if __name__ == "__main__":
    raise SystemExit(main())
