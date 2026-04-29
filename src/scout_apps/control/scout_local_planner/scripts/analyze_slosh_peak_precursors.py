#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze pre-peak drivers before slosh height/eta_dot peaks."
    )
    parser.add_argument("inputs", nargs="+", help="bag files or directories")
    parser.add_argument("--csv", default="", help="optional CSV output path")
    parser.add_argument(
        "--peak-signal",
        choices=("height", "eta_dot", "energy"),
        default="height",
        help="signal used to pick slosh peaks",
    )
    parser.add_argument("--top-k", type=int, default=3, help="number of separated peaks per bag")
    parser.add_argument("--min-sep-s", type=float, default=0.8, help="minimum separation between peaks")
    parser.add_argument(
        "--lookbacks",
        default="0.5,1.0,1.5",
        help="comma-separated pre-peak windows in seconds",
    )
    parser.add_argument("--tracking-with-terminal", action="store_true", help="do not exclude terminal recovery")
    return parser.parse_args()


def expand_inputs(paths):
    bags = []
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.endswith(".bag"):
                    bags.append(os.path.join(path, name))
        elif os.path.isfile(path) and path.endswith(".bag"):
            bags.append(path)
    return sorted(dict.fromkeys(bags))


def parse_lookbacks(text):
    values = []
    for item in text.split(","):
        item = item.strip()
        if item:
            values.append(float(item))
    return values or [1.0]


def msg_text(msg):
    return str(getattr(msg, "data", msg))


def yaw_rate_from_odom(msg):
    return float(msg.twist.twist.angular.z)


def vx_from_odom(msg):
    return float(msg.twist.twist.linear.x)


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def path_points(msg):
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def nearest_value(series, ts, start_idx=0):
    if not series:
        return start_idx, None
    idx = min(start_idx, len(series) - 1)
    while idx + 1 < len(series) and abs(series[idx + 1][0] - ts) <= abs(series[idx][0] - ts):
        idx += 1
    return idx, series[idx][1]


def interpolate_series(series, ts):
    if not series:
        return float("nan")
    if ts < series[0][0] or ts > series[-1][0]:
        return float("nan")
    lo = 0
    hi = len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < ts:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return series[0][1]
    t0, v0 = series[lo - 1]
    t1, v1 = series[lo]
    if t1 <= t0:
        return v0
    ratio = (ts - t0) / (t1 - t0)
    return v0 + ratio * (v1 - v0)


def transform_pose_2d(pose, transform):
    tx, ty, yaw = transform
    c = math.cos(yaw)
    s = math.sin(yaw)
    return {
        "x": tx + c * pose["x"] - s * pose["y"],
        "y": ty + s * pose["x"] + c * pose["y"],
        "yaw": wrap_to_pi(pose["yaw"] + yaw),
        "vx": pose["vx"],
        "wz": pose["wz"],
    }


def point_curvature(points, idx):
    if idx <= 0 or idx + 1 >= len(points):
        return 0.0
    ax, ay = points[idx - 1]
    bx, by = points[idx]
    cx, cy = points[idx + 1]
    ab = math.hypot(bx - ax, by - ay)
    bc = math.hypot(cx - bx, cy - by)
    ac = math.hypot(cx - ax, cy - ay)
    denom = ab * bc * ac
    if denom <= 1e-9:
        return 0.0
    area2 = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    return 2.0 * area2 / denom


def local_path_metrics(pose, points):
    if len(points) < 2:
        return None
    px = pose["x"]
    py = pose["y"]
    best_dist = float("inf")
    best_heading = 0.0
    best_seg = 0
    for idx in range(len(points) - 1):
        ax, ay = points[idx]
        bx, by = points[idx + 1]
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 <= 1e-12:
            continue
        u = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
        qx = ax + u * abx
        qy = ay + u * aby
        dist = math.hypot(px - qx, py - qy)
        if dist < best_dist:
            best_dist = dist
            best_heading = math.atan2(aby, abx)
            best_seg = idx
    center = max(1, min(len(points) - 2, best_seg + 1))
    start = max(1, center - 2)
    end = min(len(points) - 2, center + 2)
    kappa = max(point_curvature(points, idx) for idx in range(start, end + 1))
    return {
        "kappa_abs": kappa,
        "track_dist": best_dist,
        "heading_err_abs": abs(wrap_to_pi(pose["yaw"] - best_heading)),
    }


def make_segments(bag_path, topic):
    transitions = []
    with rosbag.Bag(bag_path) as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            value = msg_text(msg)
            ts = stamp.to_sec()
            if not transitions or transitions[-1][1] != value:
                transitions.append((ts, value))
    segments = []
    for idx, (ts, value) in enumerate(transitions):
        end_ts = transitions[idx + 1][0] if idx + 1 < len(transitions) else end_time
        segments.append((ts, end_ts, value))
    return start_time, end_time, segments


def in_segment(segments, ts, wanted):
    for start, end, value in segments:
        if start <= ts < end:
            return value == wanted
    return False


def keep_sample(mpc_segments, terminal_segments, ts, include_terminal):
    if not in_segment(mpc_segments, ts, "TRACKING"):
        return False
    if include_terminal or not terminal_segments:
        return True
    return in_segment(terminal_segments, ts, "NONE")


def append(series, key, ts, value):
    if not math.isnan(value):
        series.setdefault(key, []).append((ts, value))


def collect_series(bag_path, include_terminal):
    start_time, end_time, mpc_segments = make_segments(bag_path, "/mpc_status")
    _, _, terminal_segments = make_segments(bag_path, "/terminal/mode")
    series = {}
    prev_cmd = None
    prev_odom = None
    odom_samples = []
    reference_paths = []
    map_odom_tf = []
    odom_base_tf = []

    topics = [
        "/slosh/height",
        "/slosh/state",
        "/slosh/modal_energy_norm",
        "/slosh/ay_est",
        "/slosh/alpha_est",
        "/slosh/output_guard_active",
        "/slosh/v_des_eff",
        "/cmd_vel",
        "/odom",
        "/tf",
        "/mpc/reference_path",
    ]

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = stamp.to_sec()
            if not keep_sample(mpc_segments, terminal_segments, ts, include_terminal):
                continue

            if topic == "/slosh/height":
                append(series, "height", ts, abs(float(msg.data)))
            elif topic == "/slosh/state" and len(msg.data) >= 4:
                eta_dot = math.hypot(float(msg.data[1]), float(msg.data[3]))
                append(series, "eta_dot", ts, eta_dot)
            elif topic == "/slosh/modal_energy_norm":
                append(series, "energy", ts, float(msg.data))
            elif topic == "/slosh/ay_est":
                append(series, "slosh_ay_abs", ts, abs(float(msg.data)))
            elif topic == "/slosh/alpha_est":
                append(series, "slosh_alpha_abs", ts, abs(float(msg.data)))
            elif topic == "/slosh/output_guard_active":
                append(series, "output_guard_active", ts, float(msg.data))
            elif topic == "/slosh/v_des_eff":
                append(series, "v_des_eff", ts, float(msg.data))
            elif topic == "/cmd_vel":
                vx = float(msg.linear.x)
                wz = float(msg.angular.z)
                append(series, "cmd_vx_abs", ts, abs(vx))
                append(series, "cmd_wz_abs", ts, abs(wz))
                append(series, "cmd_ay_abs", ts, abs(vx * wz))
                if abs(vx) > 0.05:
                    append(series, "cmd_kappa_abs", ts, abs(wz / vx))
                if prev_cmd is not None:
                    prev_ts, prev_wz = prev_cmd
                    dt = ts - prev_ts
                    if dt > 1e-6:
                        append(series, "cmd_dwz_abs", ts, abs((wz - prev_wz) / dt))
                prev_cmd = (ts, wz)
            elif topic == "/odom":
                vx = vx_from_odom(msg)
                wz = yaw_rate_from_odom(msg)
                append(series, "odom_vx_abs", ts, abs(vx))
                append(series, "odom_wz_abs", ts, abs(wz))
                append(series, "odom_ay_abs", ts, abs(vx * wz))
                if abs(vx) > 0.05:
                    append(series, "odom_kappa_abs", ts, abs(wz / vx))
                if prev_odom is not None:
                    prev_ts, prev_wz = prev_odom
                    dt = ts - prev_ts
                    if dt > 1e-6:
                        append(series, "odom_dwz_abs", ts, abs((wz - prev_wz) / dt))
                prev_odom = (ts, wz)
                odom_samples.append((ts, {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": yaw_from_quat(msg.pose.pose.orientation),
                    "vx": vx,
                    "wz": wz,
                }))
            elif topic == "/tf":
                for transform in msg.transforms:
                    if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                        tr = transform.transform.translation
                        rot = transform.transform.rotation
                        map_odom_tf.append((ts, (float(tr.x), float(tr.y), yaw_from_quat(rot))))
                    elif (
                        transform.header.frame_id == "odom" and
                        transform.child_frame_id in ("base_footprint", "base_link")
                    ):
                        tr = transform.transform.translation
                        rot = transform.transform.rotation
                        odom_base_tf.append((ts, {
                            "x": float(tr.x),
                            "y": float(tr.y),
                            "yaw": yaw_from_quat(rot),
                            "vx": 0.0,
                            "wz": 0.0,
                        }))
            elif topic == "/mpc/reference_path":
                reference_paths.append((ts, path_points(msg)))

    ref_idx = 0
    map_tf_idx = 0
    base_tf_idx = 0
    for ts, odom_pose in odom_samples:
        ref_idx, ref_points = nearest_value(reference_paths, ts, ref_idx)
        if not ref_points:
            continue
        pose = odom_pose
        map_tf_idx, map_transform = nearest_value(map_odom_tf, ts, map_tf_idx)
        base_tf_idx, base_pose = nearest_value(odom_base_tf, ts, base_tf_idx)
        if map_transform is not None and base_pose is not None:
            pose = transform_pose_2d(base_pose, map_transform)
        elif map_transform is not None:
            pose = transform_pose_2d(odom_pose, map_transform)
        metrics = local_path_metrics(pose, ref_points)
        if metrics is None:
            continue
        v_des = interpolate_series(series.get("v_des_eff", []), ts)
        ref_kappa = metrics["kappa_abs"]
        append(series, "ref_kappa_abs", ts, ref_kappa)
        append(series, "track_dist", ts, metrics["track_dist"])
        append(series, "track_heading_err_abs", ts, metrics["heading_err_abs"])
        if not math.isnan(v_des):
            append(series, "ref_expected_ay_abs", ts, v_des * v_des * ref_kappa)
            append(series, "ref_expected_omega_abs", ts, v_des * ref_kappa)

    return start_time, end_time, series


def pick_peaks(series, top_k, min_sep_s):
    if not series or top_k <= 0:
        return []
    candidates = sorted(series, key=lambda item: item[1], reverse=True)
    picked = []
    for ts, value in candidates:
        if all(abs(ts - old_ts) >= min_sep_s for old_ts, _ in picked):
            picked.append((ts, value))
            if len(picked) >= top_k:
                break
    return sorted(picked)


def window_values(series, start_ts, end_ts):
    return [(ts, value) for ts, value in series if start_ts <= ts <= end_ts]


def mean(values):
    if not values:
        return float("nan")
    return sum(values) / len(values)


def integrate_values(samples):
    if len(samples) < 2:
        return 0.0
    total = 0.0
    prev_ts, prev_value = samples[0]
    for ts, value in samples[1:]:
        dt = ts - prev_ts
        if dt > 0.0:
            total += 0.5 * (prev_value + value) * dt
        prev_ts, prev_value = ts, value
    return total


def round_or_nan(value, digits=6):
    if math.isnan(value):
        return float("nan")
    return round(value, digits)


def window_row(bag_path, start_time, peak_signal, peak_index, peak_ts, peak_value, lookback, series):
    row = {
        "bag_path": bag_path,
        "bag_name": os.path.basename(bag_path),
        "peak_signal": peak_signal,
        "peak_index": peak_index,
        "peak_time_s": round(peak_ts - start_time, 3),
        "peak_value": round_or_nan(peak_value),
        "lookback_s": lookback,
    }

    drivers = (
        "odom_ay_abs",
        "odom_kappa_abs",
        "odom_wz_abs",
        "odom_dwz_abs",
        "cmd_ay_abs",
        "cmd_kappa_abs",
        "cmd_wz_abs",
        "cmd_dwz_abs",
        "v_des_eff",
        "ref_kappa_abs",
        "ref_expected_ay_abs",
        "ref_expected_omega_abs",
        "track_dist",
        "track_heading_err_abs",
        "slosh_ay_abs",
        "slosh_alpha_abs",
    )
    start_ts = peak_ts - lookback
    for key in drivers:
        samples = window_values(series.get(key, []), start_ts, peak_ts)
        values = [value for _, value in samples]
        if values:
            max_ts, max_value = max(samples, key=lambda item: item[1])
            row[f"{key}_max"] = round_or_nan(max_value)
            row[f"{key}_mean"] = round_or_nan(mean(values))
            row[f"{key}_lead_s"] = round(peak_ts - max_ts, 3)
        else:
            row[f"{key}_max"] = float("nan")
            row[f"{key}_mean"] = float("nan")
            row[f"{key}_lead_s"] = float("nan")

    for key in ("cmd_ay_abs", "odom_ay_abs", "cmd_dwz_abs", "odom_dwz_abs", "odom_wz_abs"):
        samples = window_values(series.get(key, []), start_ts, peak_ts)
        row[f"{key}_integral"] = round_or_nan(integrate_values(samples))

    guard_samples = window_values(series.get("output_guard_active", []), start_ts, peak_ts)
    guard_on = [(ts, value) for ts, value in guard_samples if value >= 0.5]
    row["output_guard_active_ratio"] = round_or_nan(
        mean([1.0 if value >= 0.5 else 0.0 for _, value in guard_samples]), 3
    )
    row["output_guard_first_lead_s"] = (
        round(peak_ts - guard_on[0][0], 3) if guard_on else float("nan")
    )
    row["output_guard_last_lead_s"] = (
        round(peak_ts - guard_on[-1][0], 3) if guard_on else float("nan")
    )

    for key in ("height", "energy", "eta_dot"):
        start_value = interpolate_series(series.get(key, []), start_ts)
        peak_window_value = interpolate_series(series.get(key, []), peak_ts)
        delta = (
            peak_window_value - start_value
            if not math.isnan(start_value) and not math.isnan(peak_window_value)
            else float("nan")
        )
        row[f"{key}_delta"] = round_or_nan(delta)
        row[f"{key}_growth_rate"] = round_or_nan(delta / lookback if not math.isnan(delta) else float("nan"))
    return row


def analyze_bag(bag_path, peak_signal, top_k, min_sep_s, lookbacks, include_terminal):
    start_time, _, series = collect_series(bag_path, include_terminal)
    peaks = pick_peaks(series.get(peak_signal, []), top_k, min_sep_s)
    rows = []
    for idx, (peak_ts, peak_value) in enumerate(peaks, start=1):
        for lookback in lookbacks:
            rows.append(
                window_row(
                    bag_path,
                    start_time,
                    peak_signal,
                    idx,
                    peak_ts,
                    peak_value,
                    lookback,
                    series,
                )
            )
    return rows


def print_rows(rows):
    if not rows:
        print("no peaks found")
        return
    current = None
    for row in rows:
        key = (row["bag_name"], row["peak_index"])
        if key != current:
            current = key
            print(
                f"{row['bag_name']} peak{row['peak_index']} "
                f"{row['peak_signal']}@{row['peak_time_s']:.3f}s={row['peak_value']}"
            )
        if abs(float(row["lookback_s"]) - 1.0) > 1e-6:
            continue
        print(
            "  1.0s: "
            f"guard_ratio={row['output_guard_active_ratio']} "
            f"guard_first={row['output_guard_first_lead_s']} "
            f"ref_ay_max={row['ref_expected_ay_abs_max']} "
            f"ref_kappa_max={row['ref_kappa_abs_max']} "
            f"odom_ay_max={row['odom_ay_abs_max']} "
            f"odom_ay_int={row['odom_ay_abs_integral']} "
            f"cmd_ay_max={row['cmd_ay_abs_max']} "
            f"cmd_ay_int={row['cmd_ay_abs_integral']} "
            f"wz_max={row['odom_wz_abs_max']} "
            f"cmd_dwz_max={row['cmd_dwz_abs_max']} "
            f"energy_d={row['energy_delta']} "
            f"track_max={row['track_dist_max']} "
            f"heading_max={row['track_heading_err_abs_max']} "
            f"v_des_mean={row['v_des_eff_mean']}"
        )


def write_csv(path, rows):
    if not path or not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    bag_paths = expand_inputs(args.inputs)
    if not bag_paths:
        print("no bag files found", file=os.sys.stderr)
        return 1
    lookbacks = parse_lookbacks(args.lookbacks)
    rows = []
    for bag_path in bag_paths:
        rows.extend(
            analyze_bag(
                bag_path,
                args.peak_signal,
                args.top_k,
                args.min_sep_s,
                lookbacks,
                args.tracking_with_terminal,
            )
        )
    print_rows(rows)
    write_csv(args.csv, rows)
    if args.csv:
        print(f"csv: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
