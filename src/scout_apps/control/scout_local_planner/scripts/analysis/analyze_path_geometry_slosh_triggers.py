#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
import os
import re

import rosbag


KNOWN_PATHS = (
    "P2_s_curve_radius",
    "P2_s_curve_smooth",
    "P3_mixed_radius",
    "P3_mixed_smooth",
    "P0_straight",
    "P1_single_turn",
    "P2_s_curve",
    "P3_mixed",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Step0 diagnostic: align slosh peaks with path geometry and execution triggers."
    )
    parser.add_argument("inputs", nargs="+", help="bag files or directories")
    parser.add_argument("--fixed-path-dir", default="/data/a/fixed_paths/sim")
    parser.add_argument("--csv", default="", help="optional CSV output")
    parser.add_argument("--peak-signal", choices=("height", "eta_dot", "energy"), default="height")
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--min-sep-s", type=float, default=0.8)
    parser.add_argument("--lookback-s", type=float, default=1.0)
    parser.add_argument("--tracking-with-terminal", action="store_true")
    return parser.parse_args()


def expand_inputs(paths):
    out = []
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.endswith(".bag"):
                    out.append(os.path.join(path, name))
        elif os.path.isfile(path) and path.endswith(".bag"):
            out.append(path)
    return sorted(dict.fromkeys(out))


def path_id_from_name(path):
    name = os.path.basename(path)
    for path_id in KNOWN_PATHS:
        if path_id in name:
            return path_id
    return ""


def condition_from_name(path):
    name = os.path.basename(path)
    match = re.search(r"_(NOM|CUSTOM|TOPPRA|RUCKIG|FAS_[^_]+(?:_[^_]+)?|[^_]+)_run", name)
    return match.group(1) if match else ""


def msg_text(msg):
    return str(getattr(msg, "data", msg))


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def percentile(values, pct):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    pos = (len(values) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    ratio = pos - lo
    return values[lo] * (1.0 - ratio) + values[hi] * ratio


def mean(values):
    values = [v for v in values if math.isfinite(v)]
    return sum(values) / len(values) if values else float("nan")


def max_abs(values):
    values = [abs(v) for v in values if math.isfinite(v)]
    return max(values) if values else float("nan")


def integrate_abs(samples):
    if len(samples) < 2:
        return 0.0
    total = 0.0
    prev_t, prev_v = samples[0]
    for ts, value in samples[1:]:
        dt = ts - prev_t
        if dt > 0.0:
            total += 0.5 * (abs(prev_v) + abs(value)) * dt
        prev_t, prev_v = ts, value
    return total


def append(series, key, ts, value):
    if math.isfinite(value):
        series.setdefault(key, []).append((ts, value))


def nearest_value(series, ts, start_idx=0):
    if not series:
        return start_idx, None
    idx = min(start_idx, len(series) - 1)
    while idx + 1 < len(series) and abs(series[idx + 1][0] - ts) <= abs(series[idx][0] - ts):
        idx += 1
    return idx, series[idx][1]


def interpolate_series(series, ts):
    if not series or ts < series[0][0] or ts > series[-1][0]:
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
    r = (ts - t0) / (t1 - t0)
    return v0 + r * (v1 - v0)


def window_samples(series, start_ts, end_ts):
    return [(ts, value) for ts, value in series if start_ts <= ts <= end_ts]


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


def load_fixed_path(path_id, fixed_path_dir):
    if not path_id:
        return None
    path = os.path.join(fixed_path_dir, f"{path_id}.json")
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    points = [(float(p["x"]), float(p["y"])) for p in data.get("poses", [])]
    return build_path_geometry(points)


def build_path_geometry(points):
    if len(points) < 3:
        return None
    s = [0.0]
    for i in range(1, len(points)):
        s.append(s[-1] + math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]))
    kappa = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        ax, ay = points[i - 1]
        bx, by = points[i]
        cx, cy = points[i + 1]
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        denom = ab * bc * ac
        if denom <= 1e-9:
            continue
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        kappa[i] = 2.0 * cross / denom
    dkappa = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        ds = max(1e-6, s[i + 1] - s[i - 1])
        dkappa[i] = (kappa[i + 1] - kappa[i - 1]) / ds
    return {"points": points, "s": s, "kappa": kappa, "dkappa": dkappa}


def nearest_path_metric(geom, x, y):
    if geom is None:
        return None
    points = geom["points"]
    best_dist = float("inf")
    best_idx = 0
    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 <= 1e-12:
            continue
        u = max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / ab2))
        qx = ax + u * abx
        qy = ay + u * aby
        dist = math.hypot(x - qx, y - qy)
        if dist < best_dist:
            best_dist = dist
            best_idx = i
    idx = max(1, min(len(points) - 2, best_idx + 1))
    return {
        "fixed_kappa": geom["kappa"][idx],
        "fixed_dkappa": geom["dkappa"][idx],
        "fixed_path_dist": best_dist,
        "fixed_s": geom["s"][idx],
    }


def path_points(msg):
    return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]


def reference_track_metrics(pose, points):
    if len(points) < 2:
        return None
    px = pose["x"]
    py = pose["y"]
    best_dist = float("inf")
    best_heading = 0.0
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
        dist = math.hypot(px - qx, py - qy)
        if dist < best_dist:
            best_dist = dist
            best_heading = math.atan2(aby, abx)
    return {
        "track_dist": best_dist,
        "heading_err_abs": abs(wrap_to_pi(pose["yaw"] - best_heading)),
    }


def transform_pose_2d(pose, transform):
    tx, ty, yaw = transform
    c = math.cos(yaw)
    s = math.sin(yaw)
    return {
        "x": tx + c * pose["x"] - s * pose["y"],
        "y": ty + s * pose["x"] + c * pose["y"],
        "yaw": wrap_to_pi(pose["yaw"] + yaw),
    }


def collect_series(bag_path, include_terminal, fixed_geom):
    start_time, _, mpc_segments = make_segments(bag_path, "/mpc_status")
    _, _, terminal_segments = make_segments(bag_path, "/terminal/mode")
    series = {}
    odom_samples = []
    reference_paths = []
    map_odom_tf = []
    odom_base_tf = []
    prev_odom = None
    prev_odom_ax = None
    prev_cmd = None

    topics = [
        "/slosh/height",
        "/slosh/state",
        "/slosh/modal_energy_norm",
        "/slosh/v_des_eff",
        "/odom",
        "/cmd_vel",
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
                eta_x = float(msg.data[0])
                eta_x_dot = float(msg.data[1])
                eta_y = float(msg.data[2])
                eta_y_dot = float(msg.data[3])
                append(series, "eta_dot", ts, math.hypot(eta_x_dot, eta_y_dot))
                append(series, "eta_x_abs", ts, abs(eta_x))
                append(series, "eta_y_abs", ts, abs(eta_y))
                append(series, "eta_x_dot_abs", ts, abs(eta_x_dot))
                append(series, "eta_y_dot_abs", ts, abs(eta_y_dot))
            elif topic == "/slosh/modal_energy_norm":
                append(series, "energy", ts, float(msg.data))
            elif topic == "/slosh/v_des_eff":
                append(series, "v_des_eff", ts, float(msg.data))
            elif topic == "/odom":
                vx = float(msg.twist.twist.linear.x)
                wz = float(msg.twist.twist.angular.z)
                ax = float("nan")
                jerk = float("nan")
                dwz = float("nan")
                if prev_odom is not None:
                    prev_ts, prev_vx, prev_wz = prev_odom
                    dt = ts - prev_ts
                    if dt > 1e-6:
                        ax = (vx - prev_vx) / dt
                        dwz = (wz - prev_wz) / dt
                        if prev_odom_ax is not None:
                            jerk = (ax - prev_odom_ax[1]) / dt
                append(series, "odom_v", ts, vx)
                append(series, "odom_wz_abs", ts, abs(wz))
                append(series, "odom_ay_abs", ts, abs(vx * wz))
                append(series, "odom_ax", ts, ax)
                append(series, "odom_ax_abs", ts, abs(ax))
                append(series, "odom_jerk", ts, jerk)
                append(series, "odom_jerk_abs", ts, abs(jerk))
                append(series, "odom_dwz_abs", ts, abs(dwz))
                if abs(vx) > 0.05:
                    append(series, "odom_kappa_abs", ts, abs(wz / vx))
                prev_odom = (ts, vx, wz)
                if math.isfinite(ax):
                    prev_odom_ax = (ts, ax)
                odom_samples.append((ts, {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": yaw_from_quat(msg.pose.pose.orientation),
                }))
            elif topic == "/cmd_vel":
                vx = float(msg.linear.x)
                wz = float(msg.angular.z)
                if prev_cmd is not None:
                    prev_ts, prev_vx = prev_cmd
                    dt = ts - prev_ts
                    if dt > 1e-6:
                        append(series, "cmd_ax_abs", ts, abs((vx - prev_vx) / dt))
                append(series, "cmd_ay_abs", ts, abs(vx * wz))
                prev_cmd = (ts, vx)
            elif topic == "/tf":
                for transform in msg.transforms:
                    if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                        tr = transform.transform.translation
                        rot = transform.transform.rotation
                        map_odom_tf.append((ts, (float(tr.x), float(tr.y), yaw_from_quat(rot))))
                    elif transform.header.frame_id == "odom" and transform.child_frame_id in ("base_footprint", "base_link"):
                        tr = transform.transform.translation
                        rot = transform.transform.rotation
                        odom_base_tf.append((ts, {
                            "x": float(tr.x),
                            "y": float(tr.y),
                            "yaw": yaw_from_quat(rot),
                        }))
            elif topic == "/mpc/reference_path":
                reference_paths.append((ts, path_points(msg)))

    ref_idx = 0
    map_idx = 0
    base_idx = 0
    for ts, odom_pose in odom_samples:
        pose = odom_pose
        map_idx, map_transform = nearest_value(map_odom_tf, ts, map_idx)
        base_idx, base_pose = nearest_value(odom_base_tf, ts, base_idx)
        if map_transform is not None and base_pose is not None:
            pose = transform_pose_2d(base_pose, map_transform)
        elif map_transform is not None:
            pose = transform_pose_2d(odom_pose, map_transform)

        fixed = nearest_path_metric(fixed_geom, pose["x"], pose["y"])
        if fixed is not None:
            append(series, "fixed_kappa_abs", ts, abs(fixed["fixed_kappa"]))
            append(series, "fixed_dkappa_abs", ts, abs(fixed["fixed_dkappa"]))
            append(series, "fixed_path_dist", ts, fixed["fixed_path_dist"])
            v = interpolate_series(series.get("odom_v", []), ts)
            ax = interpolate_series(series.get("odom_ax", []), ts)
            kappa = fixed["fixed_kappa"]
            dkappa = fixed["fixed_dkappa"]
            if math.isfinite(v):
                append(series, "fixed_ay_proxy_abs", ts, abs(v * v * kappa))
                append(series, "fixed_omega_proxy_abs", ts, abs(v * kappa))
            if math.isfinite(v) and math.isfinite(ax):
                append(series, "fixed_alpha_proxy_abs", ts, abs(ax * kappa + v * v * dkappa))

        ref_idx, ref_points = nearest_value(reference_paths, ts, ref_idx)
        if ref_points:
            metrics = reference_track_metrics(pose, ref_points)
            if metrics is not None:
                append(series, "track_dist", ts, metrics["track_dist"])
                append(series, "track_heading_err_abs", ts, metrics["heading_err_abs"])

    return start_time, series


def pick_peaks(samples, top_k, min_sep_s):
    candidates = sorted(samples, key=lambda item: item[1], reverse=True)
    picked = []
    for ts, value in candidates:
        if all(abs(ts - old_ts) >= min_sep_s for old_ts, _ in picked):
            picked.append((ts, value))
            if len(picked) >= top_k:
                break
    return sorted(picked)


def window_feature(series, key, start_ts, peak_ts):
    samples = window_samples(series.get(key, []), start_ts, peak_ts)
    values = [value for _, value in samples]
    if not values:
        return {
            f"{key}_max": float("nan"),
            f"{key}_mean": float("nan"),
            f"{key}_p95": float("nan"),
            f"{key}_lead_s": float("nan"),
            f"{key}_integral": float("nan"),
        }
    max_ts, max_value = max(samples, key=lambda item: item[1])
    return {
        f"{key}_max": max_value,
        f"{key}_mean": mean(values),
        f"{key}_p95": percentile(values, 95.0),
        f"{key}_lead_s": peak_ts - max_ts,
        f"{key}_integral": integrate_abs(samples),
    }


def classify_hint(row):
    hints = []
    if row.get("fixed_kappa_abs_max", 0.0) > 1.5:
        hints.append("high_kappa")
    if row.get("fixed_dkappa_abs_max", 0.0) > 12.0:
        hints.append("sharp_dkappa")
    if row.get("odom_ax_abs_p95", 0.0) > 2.5 or row.get("odom_jerk_abs_p95", 0.0) > 50.0:
        hints.append("longitudinal_timing")
    if row.get("odom_ay_abs_max", 0.0) > 2.5 or row.get("fixed_ay_proxy_abs_max", 0.0) > 2.5:
        hints.append("lateral_excitation")
    if row.get("track_dist_max", 0.0) > 1.5:
        hints.append("tracking_amplification")
    return "+".join(hints) if hints else "mixed_or_low_proxy"


def analyze_bag(bag_path, args):
    path_id = path_id_from_name(bag_path)
    fixed_geom = load_fixed_path(path_id, args.fixed_path_dir)
    start_time, series = collect_series(bag_path, args.tracking_with_terminal, fixed_geom)
    peaks = pick_peaks(series.get(args.peak_signal, []), args.top_k, args.min_sep_s)
    rows = []
    for peak_index, (peak_ts, peak_value) in enumerate(peaks, start=1):
        row = {
            "bag_path": bag_path,
            "bag_name": os.path.basename(bag_path),
            "path_id": path_id,
            "condition": condition_from_name(bag_path),
            "peak_signal": args.peak_signal,
            "peak_index": peak_index,
            "peak_time_s": peak_ts - start_time,
            "peak_value": peak_value,
            "lookback_s": args.lookback_s,
        }
        start_ts = peak_ts - args.lookback_s
        for key in (
            "height",
            "eta_dot",
            "energy",
            "eta_x_abs",
            "eta_y_abs",
            "eta_x_dot_abs",
            "eta_y_dot_abs",
            "fixed_kappa_abs",
            "fixed_dkappa_abs",
            "fixed_ay_proxy_abs",
            "fixed_alpha_proxy_abs",
            "odom_ax_abs",
            "odom_ay_abs",
            "odom_jerk_abs",
            "odom_dwz_abs",
            "cmd_ax_abs",
            "cmd_ay_abs",
            "track_dist",
            "track_heading_err_abs",
        ):
            row.update(window_feature(series, key, start_ts, peak_ts))
        row["trigger_hint"] = classify_hint(row)
        rows.append(row)
    return rows


def fmt(value, digits=3):
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def print_rows(rows):
    for row in rows:
        print(
            f"{row['bag_name']} peak{row['peak_index']} "
            f"{row['peak_signal']}@{fmt(row['peak_time_s'])}s={fmt(row['peak_value'], 6)} "
            f"hint={row['trigger_hint']}"
        )
        print(
            "  1.0s: "
            f"k={fmt(row['fixed_kappa_abs_max'])} "
            f"dk={fmt(row['fixed_dkappa_abs_max'])} "
            f"ay_ref={fmt(row['fixed_ay_proxy_abs_max'])} "
            f"alpha_ref={fmt(row['fixed_alpha_proxy_abs_p95'])} "
            f"ax_p95={fmt(row['odom_ax_abs_p95'])} "
            f"ay_p95={fmt(row['odom_ay_abs_p95'])} "
            f"jerk_p95={fmt(row['odom_jerk_abs_p95'])} "
            f"track={fmt(row['track_dist_max'])} "
            f"eta_x={fmt(row['eta_x_abs_max'], 5)} "
            f"eta_y={fmt(row['eta_y_abs_max'], 5)} "
            f"eta_dot={fmt(row['eta_dot_max'], 5)}"
        )
    if not rows:
        print("no peaks found")


def print_group_summary(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row["path_id"], row["condition"]), []).append(row)
    if not groups:
        return
    print("\nGroup summary:")
    for (path_id, condition), group in sorted(groups.items()):
        print(
            f"- {path_id or '?'} {condition or '?'} n={len(group)} "
            f"k_p95={fmt(percentile([r['fixed_kappa_abs_p95'] for r in group], 95))} "
            f"dk_p95={fmt(percentile([r['fixed_dkappa_abs_p95'] for r in group], 95))} "
            f"ax_p95={fmt(percentile([r['odom_ax_abs_p95'] for r in group], 95))} "
            f"ay_p95={fmt(percentile([r['odom_ay_abs_p95'] for r in group], 95))} "
            f"jerk_p95={fmt(percentile([r['odom_jerk_abs_p95'] for r in group], 95))} "
            f"track_p95={fmt(percentile([r['track_dist_p95'] for r in group], 95))} "
            f"hints={','.join(sorted(set(r['trigger_hint'] for r in group)))}"
        )


def write_csv(path, rows):
    if not path or not rows:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    bag_paths = expand_inputs(args.inputs)
    if not bag_paths:
        print("no bag files found", file=os.sys.stderr)
        return 1
    rows = []
    for bag_path in bag_paths:
        rows.extend(analyze_bag(bag_path, args))
    print_rows(rows)
    print_group_summary(rows)
    write_csv(args.csv, rows)
    if args.csv:
        print(f"csv: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
