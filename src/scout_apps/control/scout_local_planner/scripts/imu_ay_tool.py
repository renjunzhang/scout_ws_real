#!/usr/bin/env python3
"""IMU lateral-acceleration sequence, analysis, and scale calibration tool."""

import argparse
import math
import os
import statistics
import sys


def expand_bags(paths):
    bags = []
    for path in paths:
        if os.path.isdir(path):
            bags.extend(os.path.join(path, n) for n in sorted(os.listdir(path)) if n.endswith(".bag"))
        elif os.path.isfile(path) and path.endswith(".bag"):
            bags.append(path)
    return sorted(dict.fromkeys(bags))


def nearest(seq, t, i0=0):
    i = i0
    while i + 1 < len(seq) and abs(seq[i + 1]["t"] - t) <= abs(seq[i]["t"] - t):
        i += 1
    return i, seq[i]


def summarize(values):
    if not values:
        return None
    return {
        "n": len(values),
        "mean": statistics.mean(values),
        "std": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
        "pos_ratio": sum(1 for v in values if v > 0.0) / len(values),
        "neg_ratio": sum(1 for v in values if v < 0.0) / len(values),
    }


def fmt_summary(name, stats):
    if stats is None:
        return f"- {name}: n=0"
    return (
        f"- {name}: n={stats['n']} mean={stats['mean']:.4f} std={stats['std']:.4f} "
        f"min={stats['min']:.4f} max={stats['max']:.4f} "
        f"pos_ratio={stats['pos_ratio']:.3f} neg_ratio={stats['neg_ratio']:.3f}"
    )


def corr(xs, ys):
    if not xs or len(xs) != len(ys):
        return float("nan")
    sx, sy = statistics.pstdev(xs), statistics.pstdev(ys)
    if sx < 1e-9 or sy < 1e-9:
        return float("nan")
    mx, my = statistics.mean(xs), statistics.mean(ys)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / len(xs) / sx / sy


def verdict(static_stats, left_stats, right_stats, straight_stats, filtered=False):
    sign_ok = (
        left_stats is not None and right_stats is not None
        and left_stats["mean"] > 0.0 and right_stats["mean"] < 0.0
        and left_stats["pos_ratio"] >= 0.60 and right_stats["neg_ratio"] >= 0.60
    )
    bias_ok = (
        static_stats is not None and straight_stats is not None
        and abs(static_stats["mean"]) < 0.05 and abs(straight_stats["mean"]) < 0.05
    )
    if filtered:
        if sign_ok and bias_ok:
            return "PASS: bias-corrected imu_ay is usable for an IMU-ay trial."
        if sign_ok:
            return "WARN: sign is usable, but residual static/straight bias remains."
        return "FAIL: bias-corrected imu_ay sign is not reliable."
    if sign_ok and bias_ok:
        return "PASS: raw ay sign/bias looks usable, but filtered/bias-corrected ay is still preferred."
    if sign_ok:
        return "WARN: raw ay sign is useful, but bias remains; use bias correction."
    return "FAIL: raw ay sign quality is not reliable from this bag."


def read_bag_rows(path, imu_topic, odom_topic):
    import rosbag

    imu, odom, filtered, bias, ready = [], [], [], [], []
    with rosbag.Bag(path) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(
            topics=[imu_topic, odom_topic, "/slosh/imu_ay_filtered", "/slosh/imu_ay_bias", "/slosh/imu_ay_bias_ready"]
        ):
            t = stamp.to_sec() - start
            if topic == imu_topic:
                imu.append({
                    "t": t,
                    "imu_ay": float(msg.linear_acceleration.y),
                    "imu_omega": float(msg.angular_velocity.z),
                    "imu_az": float(msg.linear_acceleration.z),
                })
            elif topic == odom_topic:
                odom.append({
                    "t": t,
                    "odom_v": float(msg.twist.twist.linear.x),
                    "odom_omega": float(msg.twist.twist.angular.z),
                })
            elif topic == "/slosh/imu_ay_filtered":
                filtered.append({"t": t, "imu_ay_filtered": float(msg.data)})
            elif topic == "/slosh/imu_ay_bias":
                bias.append({"t": t, "imu_ay_bias": float(msg.data)})
            elif topic == "/slosh/imu_ay_bias_ready":
                ready.append({"t": t, "imu_ay_bias_ready": int(msg.data)})

    if not imu or not odom:
        raise RuntimeError("missing /imu/data or /odom")

    rows = []
    odom_i = filt_i = bias_i = ready_i = 0
    for sample in imu:
        odom_i, o = nearest(odom, sample["t"], odom_i)
        row = dict(sample)
        row.update(o)
        row["odom_ay_model"] = row["odom_v"] * row["odom_omega"]
        if filtered:
            filt_i, f = nearest(filtered, sample["t"], filt_i)
            row.update(f)
        if bias:
            bias_i, b = nearest(bias, sample["t"], bias_i)
            row.update(b)
        if ready:
            ready_i, r = nearest(ready, sample["t"], ready_i)
            row.update(r)
        rows.append(row)
    return rows


def classify(rows, args):
    groups = {"static": [], "left": [], "right": [], "straight": [], "moving": []}
    for r in rows:
        v, w = r["odom_v"], r["odom_omega"]
        if abs(v) < args.static_v_max and abs(w) < args.static_omega_max:
            groups["static"].append(r)
        if abs(v) > args.turn_v_min:
            groups["moving"].append(r)
        if v > args.turn_v_min and w > args.turn_omega_min:
            groups["left"].append(r)
        if v > args.turn_v_min and w < -args.turn_omega_min:
            groups["right"].append(r)
        if v > args.turn_v_min and abs(w) < args.straight_omega_max:
            groups["straight"].append(r)
    return groups


def analyze_one(path, args):
    rows = read_bag_rows(path, args.imu_topic, args.odom_topic)
    groups = classify(rows, args)

    raw = {
        "static": summarize([r["imu_ay"] for r in groups["static"]]),
        "static_az": summarize([r["imu_az"] for r in groups["static"]]),
        "left": summarize([r["imu_ay"] for r in groups["left"]]),
        "right": summarize([r["imu_ay"] for r in groups["right"]]),
        "straight": summarize([r["imu_ay"] for r in groups["straight"]]),
    }
    moving_model = [r["odom_ay_model"] for r in groups["moving"]]
    moving_raw = [r["imu_ay"] for r in groups["moving"]]
    raw_corr = corr(moving_raw, moving_model)

    filtered = None
    if any("imu_ay_filtered" in r for r in rows):
        moving_filtered = [r["imu_ay_filtered"] for r in groups["moving"] if "imu_ay_filtered" in r]
        filtered = {
            "static": summarize([r["imu_ay_filtered"] for r in groups["static"] if "imu_ay_filtered" in r]),
            "left": summarize([r["imu_ay_filtered"] for r in groups["left"] if "imu_ay_filtered" in r]),
            "right": summarize([r["imu_ay_filtered"] for r in groups["right"] if "imu_ay_filtered" in r]),
            "straight": summarize([r["imu_ay_filtered"] for r in groups["straight"] if "imu_ay_filtered" in r]),
            "corr": corr(moving_filtered, moving_model[: len(moving_filtered)]),
            "bias": summarize([r["imu_ay_bias"] for r in rows if "imu_ay_bias" in r]),
            "ready_ratio": None,
        }
        ready_vals = [r["imu_ay_bias_ready"] for r in rows if "imu_ay_bias_ready" in r]
        if ready_vals:
            filtered["ready_ratio"] = sum(1 for v in ready_vals if v > 0) / len(ready_vals)

    return rows, raw, raw_corr, filtered


def cmd_analyze(args):
    bags = expand_bags(args.inputs)
    if not bags:
        print("No bag files found.", file=sys.stderr)
        return 1
    for path in bags:
        print(os.path.basename(path))
        try:
            rows, raw, raw_corr, filtered = analyze_one(path, args)
        except Exception as exc:
            print(f"- error: {exc}\n")
            continue
        print(f"- matched_samples: {len(rows)}")
        print(fmt_summary("static imu_ay", raw["static"]))
        print(fmt_summary("static imu_az", raw["static_az"]))
        print(fmt_summary("left-turn imu_ay", raw["left"]))
        print(fmt_summary("right-turn imu_ay", raw["right"]))
        print(fmt_summary("straight imu_ay", raw["straight"]))
        print(f"- corr(imu_ay, v*omega): {raw_corr:.4f}")
        print(f"- verdict: {verdict(raw['static'], raw['left'], raw['right'], raw['straight'])}")
        if filtered:
            print(fmt_summary("static imu_ay_filtered", filtered["static"]))
            print(fmt_summary("left-turn imu_ay_filtered", filtered["left"]))
            print(fmt_summary("right-turn imu_ay_filtered", filtered["right"]))
            print(fmt_summary("straight imu_ay_filtered", filtered["straight"]))
            print(f"- corr(imu_ay_filtered, v*omega): {filtered['corr']:.4f}")
            print(fmt_summary("imu_ay_bias", filtered["bias"]))
            if filtered["ready_ratio"] is not None:
                print(f"- imu_ay_bias_ready_ratio: {filtered['ready_ratio']:.3f}")
            print(f"- filtered_verdict: {verdict(filtered['static'], filtered['left'], filtered['right'], filtered['straight'], filtered=True)}")
        print()
    return 0


def cmd_sequence(args):
    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node("imu_ay_tool_sequence", anonymous=True)
    pub = rospy.Publisher(args.topic, Twist, queue_size=1)
    rate = rospy.Rate(max(1.0, args.rate))
    rospy.sleep(0.5)

    def publish(duration, vx, wz, label):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        end = rospy.Time.now() + rospy.Duration.from_sec(duration)
        rospy.loginfo("IMU ay sequence: %s %.2fs vx=%.3f wz=%.3f", label, duration, vx, wz)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            pub.publish(msg)
            rate.sleep()

    try:
        publish(args.stop_1, 0.0, 0.0, "stop-1")
        publish(args.left_duration, args.linear, abs(args.omega), "left-arc")
        publish(args.stop_2, 0.0, 0.0, "stop-2")
        publish(args.right_duration, args.linear, -abs(args.omega), "right-arc")
        publish(args.stop_3, 0.0, 0.0, "stop-3")
    finally:
        for _ in range(10):
            pub.publish(Twist())
            rate.sleep()
    return 0


def load_arrays(path, imu_topic, odom_topic):
    import numpy as np
    import rosbag

    imu, odom = [], []
    with rosbag.Bag(path) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=[imu_topic, odom_topic]):
            t = stamp.to_sec() - start
            if topic == imu_topic:
                imu.append((t, float(msg.linear_acceleration.y)))
            elif topic == odom_topic:
                odom.append((t, float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z)))
    if not imu or not odom:
        raise RuntimeError(f"missing topics: imu={len(imu)}, odom={len(odom)}")
    imu = np.array(imu)
    odom = np.array(odom)
    v = np.interp(imu[:, 0], odom[:, 0], odom[:, 1])
    w = np.interp(imu[:, 0], odom[:, 0], odom[:, 2])
    return imu[:, 0], imu[:, 1], v, w


def ema_np(values, alpha):
    import numpy as np
    out = np.empty_like(values)
    out[0] = values[0]
    for i in range(1, len(values)):
        out[i] = alpha * values[i] + (1.0 - alpha) * out[i - 1]
    return out


def long_mask(t, mask, min_duration):
    import numpy as np
    out = np.zeros_like(mask, dtype=bool)
    i = 0
    while i < len(mask):
        if not mask[i]:
            i += 1
            continue
        j = i
        while j < len(mask) and mask[j]:
            j += 1
        if t[j - 1] - t[i] >= min_duration:
            out[i:j] = True
        i = j
    return out


def trimmed_mean_np(values, ratio):
    import numpy as np
    arr = np.sort(values)
    k = int(len(arr) * ratio)
    if len(arr) - 2 * k > 0:
        arr = arr[k: len(arr) - k]
    return float(np.mean(arr))


def cmd_calibrate(args):
    import numpy as np

    t, ay_raw, v, w = load_arrays(args.bag, args.imu_topic, args.odom_topic)
    ay_model = v * w
    ay_ema = ema_np(ay_raw, args.ema_alpha)
    static = long_mask(t, (abs(v) < args.static_v_max) & (abs(w) < args.static_omega_max), args.min_static_s)
    motion = long_mask(t, ~((abs(v) < args.static_v_max) & (abs(w) < args.static_omega_max)), args.min_motion_s)
    n_static, n_motion = int(np.sum(static)), int(np.sum(motion))

    bias = trimmed_mean_np(ay_ema[static], args.trim_ratio) if n_static >= 50 else 0.0
    ay_unbiased = ay_ema - bias
    excite = motion & (abs(ay_model) > args.min_excitation)
    if int(np.sum(excite)) >= 20:
        scale = float(np.dot(ay_unbiased[excite], ay_model[excite]) / np.dot(ay_unbiased[excite], ay_unbiased[excite]))
    else:
        scale = 1.0
    ay_calib = ay_unbiased * scale

    def rms(x):
        return float(np.sqrt(np.mean(x * x))) if len(x) else float("nan")

    raw_rms = rms(ay_unbiased[motion] - ay_model[motion])
    calib_rms = rms(ay_calib[motion] - ay_model[motion])
    raw_corr = float(np.corrcoef(ay_unbiased[motion], ay_model[motion])[0, 1]) if n_motion > 1 else float("nan")
    calib_corr = float(np.corrcoef(ay_calib[motion], ay_model[motion])[0, 1]) if n_motion > 1 else float("nan")

    yaml = f"""# Generated by imu_ay_tool.py calibrate
# bag: {os.path.basename(args.bag)}
slosh_estimator:
  imu_ay_scale: {scale:.6f}

# diagnostics:
#   static_samples: {n_static}
#   motion_samples: {n_motion}
#   bias_estimate: {bias:.6f} m/s^2
#   corr_unscaled: {raw_corr:.4f}
#   corr_scaled: {calib_corr:.4f}
#   rms_unscaled: {raw_rms:.6f} m/s^2
#   rms_scaled: {calib_rms:.6f} m/s^2
"""
    with open(args.output, "w") as f:
        f.write(yaml)

    print(f"- bag: {args.bag}")
    print(f"- static_samples: {n_static}")
    print(f"- motion_samples: {n_motion}")
    print(f"- bias_estimate: {bias:.6f} m/s^2")
    print(f"- imu_ay_scale: {scale:.6f}")
    print(f"- corr unscaled/scaled: {raw_corr:.4f} / {calib_corr:.4f}")
    print(f"- RMS unscaled/scaled: {raw_rms:.6f} / {calib_rms:.6f} m/s^2")
    print(f"- wrote: {args.output}")

    if args.plot:
        save_plot(args.output, t, ay_raw, ay_ema, ay_model, ay_calib, v, w, static, motion, bias, scale)
    return 0


def save_plot(output_yaml, t, ay_raw, ay_ema, ay_model, ay_calib, v, w, static, motion, bias, scale):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("- plot skipped: matplotlib not available")
        return
    x = t - t[0]
    plot_path = output_yaml.rsplit(".", 1)[0] + "_plot.png"
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(x, ay_raw, alpha=0.35, label="ay_raw")
    axes[0].plot(x, ay_ema, label="ay_ema")
    axes[0].plot(x, ay_model, label="v*omega")
    axes[0].axhline(bias, linestyle="--", color="gray", label=f"bias={bias:.4f}")
    axes[0].legend(fontsize=8)
    axes[0].set_ylabel("m/s^2")
    axes[1].plot(x, ay_calib, label=f"ay_scaled scale={scale:.4f}")
    axes[1].plot(x, ay_model, label="v*omega")
    axes[1].legend(fontsize=8)
    axes[1].set_ylabel("m/s^2")
    axes[2].plot(x, v, label="v")
    axes[2].plot(x, w, label="omega")
    axes[2].fill_between(x, -0.5, 0.5, where=static, alpha=0.15, label="static")
    axes[2].fill_between(x, -0.5, 0.5, where=motion, alpha=0.15, label="motion")
    axes[2].legend(fontsize=8)
    axes[2].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(plot_path, dpi=120)
    print(f"- wrote: {plot_path}")


def add_analysis_args(parser):
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--static-v-max", type=float, default=0.03)
    parser.add_argument("--static-omega-max", type=float, default=0.03)
    parser.add_argument("--turn-v-min", type=float, default=0.08)
    parser.add_argument("--turn-omega-min", type=float, default=0.12)
    parser.add_argument("--straight-omega-max", type=float, default=0.05)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    seq = sub.add_parser("sequence", help="publish the standard IMU ay calibration cmd_vel sequence")
    seq.add_argument("--topic", default="/cmd_vel")
    seq.add_argument("--rate", type=float, default=20.0)
    seq.add_argument("--stop-1", type=float, default=5.0)
    seq.add_argument("--left-duration", type=float, default=5.0)
    seq.add_argument("--stop-2", type=float, default=3.0)
    seq.add_argument("--right-duration", type=float, default=5.0)
    seq.add_argument("--stop-3", type=float, default=5.0)
    seq.add_argument("--linear", type=float, default=0.30)
    seq.add_argument("--omega", type=float, default=0.30)
    seq.set_defaults(func=cmd_sequence)

    ana = sub.add_parser("analyze", help="analyze IMU ay quality from bag files")
    ana.add_argument("inputs", nargs="+")
    add_analysis_args(ana)
    ana.set_defaults(func=cmd_analyze)

    cal = sub.add_parser("calibrate", help="estimate imu_ay_scale from one calibration bag")
    cal.add_argument("bag")
    cal.add_argument("--output", default="imu_ay_calibration.yaml")
    cal.add_argument("--plot", action="store_true")
    cal.add_argument("--ema-alpha", type=float, default=0.3)
    cal.add_argument("--min-static-s", type=float, default=3.0)
    cal.add_argument("--min-motion-s", type=float, default=1.0)
    cal.add_argument("--min-excitation", type=float, default=0.01)
    cal.add_argument("--trim-ratio", type=float, default=0.10)
    add_analysis_args(cal)
    cal.set_defaults(func=cmd_calibrate)

    return parser


def main():
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
