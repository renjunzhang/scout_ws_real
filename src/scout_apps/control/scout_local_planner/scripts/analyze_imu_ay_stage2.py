#!/usr/bin/env python3
"""Stage-2 offline checker for IMU lateral acceleration quality."""

import argparse
import math
import os
import statistics
import sys

import rosbag


def expand_inputs(inputs):
    bag_paths = []
    for path in inputs:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.endswith(".bag"):
                    bag_paths.append(os.path.join(path, name))
        elif os.path.isfile(path) and path.endswith(".bag"):
            bag_paths.append(path)
    return sorted(dict.fromkeys(bag_paths))


def nearest(seq, t, i0=0):
    def stamp(item):
        return item["t"] if isinstance(item, dict) else item[0]

    i = i0
    while i + 1 < len(seq) and abs(stamp(seq[i + 1]) - t) <= abs(stamp(seq[i]) - t):
        i += 1
    return i, seq[i]


def summarize(values):
    if not values:
        return None
    pos_ratio = sum(1 for v in values if v > 0.0) / len(values)
    neg_ratio = sum(1 for v in values if v < 0.0) / len(values)
    return {
        "n": len(values),
        "mean": statistics.mean(values),
        "std": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
        "pos_ratio": pos_ratio,
        "neg_ratio": neg_ratio,
    }


def format_summary(name, stats):
    if stats is None:
        return f"- {name}: n=0"
    return (
        f"- {name}: n={stats['n']} "
        f"mean={stats['mean']:.4f} std={stats['std']:.4f} "
        f"min={stats['min']:.4f} max={stats['max']:.4f} "
        f"pos_ratio={stats['pos_ratio']:.3f} neg_ratio={stats['neg_ratio']:.3f}"
    )


def build_verdict(static_stats, left_stats, right_stats, straight_stats, filtered=False):
    sign_ok = (
        left_stats is not None and right_stats is not None and
        left_stats["mean"] > 0.0 and right_stats["mean"] < 0.0 and
        left_stats["pos_ratio"] >= 0.60 and right_stats["neg_ratio"] >= 0.60
    )
    bias_small = (
        static_stats is not None and straight_stats is not None and
        abs(static_stats["mean"]) < 0.05 and abs(straight_stats["mean"]) < 0.05
    )

    if filtered:
        if sign_ok and bias_small:
            return "bias-corrected imu_ay looks usable for an IMU-ay-enabled trial."
        if sign_ok:
            return "bias correction improves sign consistency, but residual bias is still visible; validate again before enabling IMU ay."
        return "bias-corrected imu_ay sign quality is still not reliable enough."

    if sign_ok and bias_small:
        return "ay sign/bias look usable; still prefer a dedicated stage-2 bag before enabling IMU ay."
    if sign_ok:
        return "ay sign is directionally useful, but static/straight bias remains; do not enable directly, proceed to stage-3 preprocessing."
    return "ay sign quality is not yet reliable enough from this bag; record a cleaner dedicated stage-2 bag."


def classify_rows(rows, args):
    groups = {
        "static": [],
        "left_turn": [],
        "right_turn": [],
        "straight": [],
        "moving": [],
    }
    for row in rows:
        v = row["odom_v"]
        omega = row["odom_omega"]
        if abs(v) < args.static_v_max and abs(omega) < args.static_omega_max:
            groups["static"].append(row)
        if abs(v) > args.turn_v_min:
            groups["moving"].append(row)
        if v > args.turn_v_min and omega > args.turn_omega_min:
            groups["left_turn"].append(row)
        if v > args.turn_v_min and omega < -args.turn_omega_min:
            groups["right_turn"].append(row)
        if v > args.turn_v_min and abs(omega) < args.straight_omega_max:
            groups["straight"].append(row)
    return groups


def correlation(xs, ys):
    if not xs or len(xs) != len(ys):
        return float("nan")
    x_std = statistics.pstdev(xs)
    y_std = statistics.pstdev(ys)
    if x_std < 1e-9 or y_std < 1e-9:
        return float("nan")
    x_mean = statistics.mean(xs)
    y_mean = statistics.mean(ys)
    cov = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / len(xs)
    return cov / (x_std * y_std)


def analyze_bag(bag_path, args):
    imu = []
    odom = []
    imu_ay_filtered = []
    imu_ay_bias = []
    imu_ay_bias_ready = []
    with rosbag.Bag(bag_path) as bag:
        start = bag.get_start_time()
        for topic, msg, t in bag.read_messages(topics=[
                "/imu/data",
                "/odom",
                "/slosh/imu_ay_filtered",
                "/slosh/imu_ay_bias",
                "/slosh/imu_ay_bias_ready",
        ]):
            ts = t.to_sec() - start
            if topic == "/imu/data":
                imu.append({
                    "t": ts,
                    "imu_ay": float(msg.linear_acceleration.y),
                    "imu_omega": float(msg.angular_velocity.z),
                    "imu_az": float(msg.linear_acceleration.z),
                })
            elif topic == "/odom":
                odom.append({
                    "t": ts,
                    "odom_v": float(msg.twist.twist.linear.x),
                    "odom_omega": float(msg.twist.twist.angular.z),
                })
            elif topic == "/slosh/imu_ay_filtered":
                imu_ay_filtered.append({"t": ts, "imu_ay_filtered": float(msg.data)})
            elif topic == "/slosh/imu_ay_bias":
                imu_ay_bias.append({"t": ts, "imu_ay_bias": float(msg.data)})
            elif topic == "/slosh/imu_ay_bias_ready":
                imu_ay_bias_ready.append({"t": ts, "imu_ay_bias_ready": int(msg.data)})

    if not imu or not odom:
        return {"error": "missing /imu/data or /odom"}

    rows = []
    odom_i = 0
    filtered_i = 0
    bias_i = 0
    ready_i = 0
    for sample in imu:
        odom_i, odom_sample = nearest(odom, sample["t"], odom_i)
        row = dict(sample)
        row.update(odom_sample)
        row["odom_ay_model"] = row["odom_v"] * row["odom_omega"]
        if imu_ay_filtered:
            filtered_i, filtered_sample = nearest(imu_ay_filtered, sample["t"], filtered_i)
            row.update(filtered_sample)
        if imu_ay_bias:
            bias_i, bias_sample = nearest(imu_ay_bias, sample["t"], bias_i)
            row.update(bias_sample)
        if imu_ay_bias_ready:
            ready_i, ready_sample = nearest(imu_ay_bias_ready, sample["t"], ready_i)
            row.update(ready_sample)
        rows.append(row)

    groups = classify_rows(rows, args)

    static_ay = summarize([r["imu_ay"] for r in groups["static"]])
    static_az = summarize([r["imu_az"] for r in groups["static"]])
    left_ay = summarize([r["imu_ay"] for r in groups["left_turn"]])
    right_ay = summarize([r["imu_ay"] for r in groups["right_turn"]])
    straight_ay = summarize([r["imu_ay"] for r in groups["straight"]])

    moving_ay = [r["imu_ay"] for r in groups["moving"]]
    moving_model = [r["odom_ay_model"] for r in groups["moving"]]
    corr = correlation(moving_ay, moving_model)
    diff_stats = summarize([a - b for a, b in zip(moving_ay, moving_model)]) if moving_ay else None

    verdict = build_verdict(static_ay, left_ay, right_ay, straight_ay, filtered=False)

    filtered_result = None
    if imu_ay_filtered:
        static_filtered = summarize([r["imu_ay_filtered"] for r in groups["static"] if "imu_ay_filtered" in r])
        left_filtered = summarize([r["imu_ay_filtered"] for r in groups["left_turn"] if "imu_ay_filtered" in r])
        right_filtered = summarize([r["imu_ay_filtered"] for r in groups["right_turn"] if "imu_ay_filtered" in r])
        straight_filtered = summarize([r["imu_ay_filtered"] for r in groups["straight"] if "imu_ay_filtered" in r])
        moving_filtered = [r["imu_ay_filtered"] for r in groups["moving"] if "imu_ay_filtered" in r]
        filtered_corr = correlation(moving_filtered, moving_model) if moving_filtered else float("nan")
        filtered_diff = summarize(
            [a - b for a, b in zip(moving_filtered, moving_model)]
        ) if moving_filtered else None
        bias_values = [r["imu_ay_bias"] for r in rows if "imu_ay_bias" in r]
        ready_values = [r["imu_ay_bias_ready"] for r in rows if "imu_ay_bias_ready" in r]
        filtered_result = {
            "static": static_filtered,
            "left": left_filtered,
            "right": right_filtered,
            "straight": straight_filtered,
            "corr": filtered_corr,
            "diff": filtered_diff,
            "bias": summarize(bias_values) if bias_values else None,
            "ready_ratio": (sum(1 for v in ready_values if v > 0) / len(ready_values)) if ready_values else None,
            "verdict": build_verdict(static_filtered, left_filtered, right_filtered, straight_filtered, filtered=True),
        }

    return {
        "rows": len(rows),
        "static_ay": static_ay,
        "static_az": static_az,
        "left_ay": left_ay,
        "right_ay": right_ay,
        "straight_ay": straight_ay,
        "moving_corr": corr,
        "moving_diff": diff_stats,
        "verdict": verdict,
        "filtered": filtered_result,
    }


def build_parser():
    parser = argparse.ArgumentParser(
        description="Stage-2 check for IMU lateral acceleration quality from rosbag."
    )
    parser.add_argument("inputs", nargs="+", help="bag file or directory containing bag files")
    parser.add_argument("--static-v-max", type=float, default=0.03,
                        help="max |v| for static samples (default: 0.03)")
    parser.add_argument("--static-omega-max", type=float, default=0.03,
                        help="max |omega| for static samples (default: 0.03)")
    parser.add_argument("--turn-v-min", type=float, default=0.08,
                        help="min forward speed for turn/straight classification (default: 0.08)")
    parser.add_argument("--turn-omega-min", type=float, default=0.12,
                        help="min |omega| for left/right turn classification (default: 0.12)")
    parser.add_argument("--straight-omega-max", type=float, default=0.05,
                        help="max |omega| for straight classification (default: 0.05)")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    bag_paths = expand_inputs(args.inputs)
    if not bag_paths:
        print("未找到可用的 .bag 文件", file=sys.stderr)
        return 1

    for bag_path in bag_paths:
        result = analyze_bag(bag_path, args)
        print(os.path.basename(bag_path))
        if "error" in result:
            print(f"- error: {result['error']}")
            print()
            continue
        print(f"- matched_samples: {result['rows']}")
        print(format_summary("static imu_ay", result["static_ay"]))
        print(format_summary("static imu_az", result["static_az"]))
        print(format_summary("left-turn imu_ay", result["left_ay"]))
        print(format_summary("right-turn imu_ay", result["right_ay"]))
        print(format_summary("straight imu_ay", result["straight_ay"]))
        if result["moving_diff"] is not None:
            print(f"- corr(imu_ay, v*omega): {result['moving_corr']:.4f}")
            print(format_summary("imu_ay - v*omega", result["moving_diff"]))
        else:
            print("- corr(imu_ay, v*omega): n/a")
        print(f"- verdict: {result['verdict']}")
        if result["filtered"] is not None:
            filtered = result["filtered"]
            print(format_summary("static imu_ay_filtered", filtered["static"]))
            print(format_summary("left-turn imu_ay_filtered", filtered["left"]))
            print(format_summary("right-turn imu_ay_filtered", filtered["right"]))
            print(format_summary("straight imu_ay_filtered", filtered["straight"]))
            if filtered["diff"] is not None:
                print(f"- corr(imu_ay_filtered, v*omega): {filtered['corr']:.4f}")
                print(format_summary("imu_ay_filtered - v*omega", filtered["diff"]))
            else:
                print("- corr(imu_ay_filtered, v*omega): n/a")
            if filtered["bias"] is not None:
                print(format_summary("imu_ay_bias", filtered["bias"]))
            if filtered["ready_ratio"] is not None:
                print(f"- imu_ay_bias_ready_ratio: {filtered['ready_ratio']:.3f}")
            print(f"- filtered_verdict: {filtered['verdict']}")
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
