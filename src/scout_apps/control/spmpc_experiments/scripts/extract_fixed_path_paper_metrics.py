#!/usr/bin/env python3
"""Extract fixed-path paper metrics with pre_terminal / terminal split.

This script is intentionally conservative: missing topics produce NaN fields instead of
silent zeros. It supports SPMPC topics and external baseline bags recorded by the
fixed-path suite scripts.
"""

import argparse
import bisect
import csv
import math
from pathlib import Path
from statistics import mean

import rosbag


BAD_STATUS_KEYS = (
    "FAILED",
    "NO_VALID_CMD",
    "WAITING",
    "ACADOS_NOT",
    "SET_PLAN_FAILED",
    "PLUGIN_ERROR",
)


def nan():
    return float("nan")


def finite_values(values):
    return [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]


def safe_mean(values):
    vals = finite_values(values)
    return mean(vals) if vals else nan()


def rms(values):
    vals = finite_values(values)
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else nan()


def percentile(values, pct):
    vals = sorted(finite_values(values))
    if not vals:
        return nan()
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    ratio = pos - lo
    return vals[lo] * (1.0 - ratio) + vals[hi] * ratio


def max_abs(values):
    vals = finite_values(values)
    return max((abs(v) for v in vals), default=nan())


def stamp_to_sec(stamp):
    return stamp.to_sec()


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def read_meta_for_bag(bag_path):
    candidates = list(bag_path.parent.glob("*_meta.yaml"))
    meta = {}
    for path in candidates:
        for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            meta[key.strip()] = value.strip().strip('"').strip("'")
        if meta:
            return meta
    return meta


def infer_method(bag_path, meta):
    method = meta.get("method", "").strip()
    variant = meta.get("variant", "").strip()
    if method == "spmpc" and variant:
        return f"spmpc_{variant}"
    if method:
        return method
    name = bag_path.name.lower()
    for key in ("b_ours", "b_slosh", "b_smooth", "b0"):
        if key in name:
            return f"spmpc_{key}"
    for key in ("teb", "dwa", "mpc_local_planner", "mpc"):
        if key in name:
            return "mpc_local_planner" if key == "mpc" else key
    return "unknown"


def path_points(path_msg):
    pts = []
    for pose in path_msg.poses:
        p = pose.pose.position
        pts.append((float(p.x), float(p.y), yaw_from_quat(pose.pose.orientation)))
    return pts


def distance_to_segment(px, py, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay), 0.0
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy), math.atan2(vy, vx)


def nearest_path_error(x, y, yaw, pts):
    if len(pts) < 2:
        return nan(), nan()
    best_dist = float("inf")
    best_heading = 0.0
    for i in range(len(pts) - 1):
        dist, heading = distance_to_segment(x, y, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
        if dist < best_dist:
            best_dist = dist
            best_heading = heading
    return best_dist, abs(wrap_angle(yaw - best_heading))


def derivative(series):
    out = []
    for (t0, v0), (t1, v1) in zip(series[:-1], series[1:]):
        dt = t1 - t0
        if dt > 1e-6:
            out.append((t1, (v1 - v0) / dt))
    return out


def latest_phase_from_debug(terminal_samples, t):
    if not terminal_samples:
        return None
    times = [item[0] for item in terminal_samples]
    idx = bisect.bisect_right(times, t) - 1
    if idx < 0:
        return None
    _, terminal_phase, pre_terminal_phase = terminal_samples[idx]
    if terminal_phase:
        return "terminal"
    if pre_terminal_phase:
        return "pre_terminal"
    return "other"


def read_bag_series(bag_path, path_topic, terminal_threshold):
    data = {
        "path": [],
        "odom": [],
        "cmd": [],
        "slosh_height_mm": [],
        "eta_dot_norm": [],
        "solve_time_ms": [],
        "status": [],
        "terminal_debug": [],
        "topics": set(),
    }
    candidate_topics = [
        path_topic,
        "/odom",
        "/cmd_vel",
        "/spmpc/slosh_height",
        "/slosh/height",
        "/spmpc/debug/slosh_state",
        "/slosh/state",
        "/spmpc/solver_time_ms",
        "/spmpc/status",
        "/baseline/status",
        "/baseline/teb/status",
        "/baseline/dwa/status",
        "/baseline/mpc_local_planner/status",
        "/spmpc/terminal/debug",
    ]
    with rosbag.Bag(str(bag_path), "r") as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())
        read_topics = [topic for topic in candidate_topics if topic in available]
        data["topics"] = available
        for topic, msg, stamp in bag.read_messages(topics=read_topics):
            t = stamp_to_sec(stamp)
            if topic == path_topic and getattr(msg, "poses", None):
                if not data["path"]:
                    data["path"] = path_points(msg)
            elif topic == "/odom":
                pose = msg.pose.pose
                twist = msg.twist.twist
                data["odom"].append(
                    {
                        "t": t,
                        "x": float(pose.position.x),
                        "y": float(pose.position.y),
                        "yaw": yaw_from_quat(pose.orientation),
                        "v": float(twist.linear.x),
                        "omega": float(twist.angular.z),
                    }
                )
            elif topic == "/cmd_vel":
                data["cmd"].append({"t": t, "v": float(msg.linear.x), "omega": float(msg.angular.z)})
            elif topic == "/spmpc/slosh_height":
                value = float(getattr(msg, "data", nan()))
                data["slosh_height_mm"].append((t, value))
            elif topic == "/slosh/height":
                value = float(getattr(msg, "data", nan()))
                data["slosh_height_mm"].append((t, value * 1000.0))
            elif topic in ("/spmpc/debug/slosh_state", "/slosh/state"):
                arr = list(getattr(msg, "data", []))
                if len(arr) >= 4:
                    data["eta_dot_norm"].append((t, math.hypot(float(arr[1]), float(arr[3]))))
                elif len(arr) >= 2:
                    data["eta_dot_norm"].append((t, abs(float(arr[1]))))
            elif topic == "/spmpc/solver_time_ms":
                data["solve_time_ms"].append((t, float(getattr(msg, "data", nan()))))
            elif topic.endswith("/status"):
                data["status"].append((t, str(getattr(msg, "data", ""))))
            elif topic == "/spmpc/terminal/debug":
                arr = list(getattr(msg, "data", []))
                if len(arr) >= 3:
                    data["terminal_debug"].append((t, bool(arr[1] >= 0.5), bool(arr[2] >= 0.5)))

    if not data["terminal_debug"]:
        derive_terminal_phase(data, terminal_threshold)
    return data


def derive_terminal_phase(data, terminal_threshold):
    pts = data["path"]
    if len(pts) < 1:
        return
    gx, gy, _ = pts[-1]
    terminal_latched = False
    for row in data["odom"]:
        dist = math.hypot(row["x"] - gx, row["y"] - gy)
        if dist <= terminal_threshold:
            terminal_latched = True
        data["terminal_debug"].append((row["t"], terminal_latched, not terminal_latched))


def phase_filter(samples, phase, terminal_debug):
    if phase == "all":
        return samples
    out = []
    for item in samples:
        t = item["t"] if isinstance(item, dict) else item[0]
        sample_phase = latest_phase_from_debug(terminal_debug, t)
        if sample_phase == phase:
            out.append(item)
    return out


def tuple_phase_filter(samples, phase, terminal_debug):
    if phase == "all":
        return samples
    out = []
    for item in samples:
        sample_phase = latest_phase_from_debug(terminal_debug, item[0])
        if sample_phase == phase:
            out.append(item)
    return out


def summarize_bag_phase(bag_path, data, meta, method, phase):
    odom = phase_filter(data["odom"], phase, data["terminal_debug"])
    cmd = phase_filter(data["cmd"], phase, data["terminal_debug"])
    slosh = tuple_phase_filter(data["slosh_height_mm"], phase, data["terminal_debug"])
    eta_dot = tuple_phase_filter(data["eta_dot_norm"], phase, data["terminal_debug"])
    solve = tuple_phase_filter(data["solve_time_ms"], phase, data["terminal_debug"])

    track_errors = []
    heading_errors = []
    for row in odom:
        e, h = nearest_path_error(row["x"], row["y"], row["yaw"], data["path"])
        track_errors.append(e)
        heading_errors.append(abs(h))

    cmd_v_acc = derivative([(row["t"], row["v"]) for row in cmd])
    cmd_w_acc = derivative([(row["t"], row["omega"]) for row in cmd])
    odom_ax = derivative([(row["t"], row["v"]) for row in odom])
    odom_ay_values = [row["v"] * row["omega"] for row in odom]

    if odom:
        duration = max(0.0, odom[-1]["t"] - odom[0]["t"])
    else:
        duration = 0.0

    final_dist = nan()
    if data["path"] and data["odom"]:
        gx, gy, _ = data["path"][-1]
        final = data["odom"][-1]
        final_dist = math.hypot(final["x"] - gx, final["y"] - gy)

    status_values = [s for _, s in data["status"]]
    failure_count = sum(1 for s in status_values if any(key in s for key in BAD_STATUS_KEYS))
    # Prefer geometric success. Wrapper GOAL_REACHED can be misleading in smoke runs
    # where the robot is not aligned to the fixed-path start yet.
    if math.isfinite(final_dist):
        success = int(final_dist <= 0.35)
    else:
        success = int(any("GOAL_REACHED" in s for s in status_values))

    final_speed = abs(data["odom"][-1]["v"]) if data["odom"] else nan()
    final_omega = abs(data["odom"][-1]["omega"]) if data["odom"] else nan()
    stable_stop = int(
        math.isfinite(final_speed)
        and math.isfinite(final_omega)
        and final_speed <= 0.05
        and final_omega <= 0.08
        and math.isfinite(final_dist)
        and final_dist <= 0.35
    )

    row = {
        "bag": str(bag_path),
        "method": method,
        "variant": meta.get("variant", ""),
        "path_id": meta.get("path_id", ""),
        "run_id": meta.get("run_id", bag_path.stem),
        "phase": phase,
        "duration_s": duration,
        "success": success,
        "failure_count": failure_count,
        "tracking_error_rms_m": rms(track_errors),
        "tracking_error_p95_m": percentile(track_errors, 95),
        "heading_error_p95_deg": math.degrees(percentile(heading_errors, 95)) if finite_values(heading_errors) else nan(),
        "slosh_height_peak_mm": max_abs([v for _, v in slosh]),
        "slosh_height_p95_mm": percentile([abs(v) for _, v in slosh], 95),
        "eta_dot_norm_peak": max_abs([v for _, v in eta_dot]),
        "eta_dot_norm_rms": rms([v for _, v in eta_dot]),
        "cmd_dvx_rms_mps2": rms([v for _, v in cmd_v_acc]),
        "cmd_dwz_rms_radps2": rms([v for _, v in cmd_w_acc]),
        "odom_ax_abs_p95_mps2": percentile([abs(v) for _, v in odom_ax], 95),
        "odom_ay_abs_p95_mps2": percentile([abs(v) for v in odom_ay_values], 95),
        "solver_time_mean_ms": safe_mean([v for _, v in solve]),
        "solver_time_p95_ms": percentile([v for _, v in solve], 95),
        "final_stop_distance_m": final_dist,
        "final_speed_mps": final_speed,
        "final_omega_radps": final_omega,
        "stable_stop_bool": stable_stop,
        "has_spmpc_terminal_debug": int("/spmpc/terminal/debug" in data["topics"]),
        "has_slosh_height": int(bool(data["slosh_height_mm"])),
    }
    return row


def collect_bags(inputs):
    bags = []
    for item in inputs:
        path = Path(item).expanduser().resolve()
        if path.is_file() and path.suffix == ".bag":
            bags.append(path)
        elif path.is_dir():
            bags.extend(sorted(path.rglob("*.bag")))
    return sorted(set(bags))


def write_csv(path, rows):
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def group_summary(rows):
    groups = {}
    for row in rows:
        key = (row["method"], row["path_id"], row["phase"])
        groups.setdefault(key, []).append(row)
    summary = []
    numeric_keys = [
        "success",
        "failure_count",
        "duration_s",
        "tracking_error_rms_m",
        "tracking_error_p95_m",
        "heading_error_p95_deg",
        "slosh_height_peak_mm",
        "slosh_height_p95_mm",
        "eta_dot_norm_peak",
        "eta_dot_norm_rms",
        "cmd_dvx_rms_mps2",
        "cmd_dwz_rms_radps2",
        "odom_ax_abs_p95_mps2",
        "odom_ay_abs_p95_mps2",
        "solver_time_mean_ms",
        "solver_time_p95_ms",
        "final_stop_distance_m",
        "final_speed_mps",
        "final_omega_radps",
        "stable_stop_bool",
    ]
    for (method, path_id, phase), items in sorted(groups.items()):
        out = {"method": method, "path_id": path_id, "phase": phase, "run_count": len(items)}
        for key in numeric_keys:
            out[f"{key}_mean"] = safe_mean([item[key] for item in items])
            out[f"{key}_p95"] = percentile([item[key] for item in items], 95)
        summary.append(out)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="Bag file(s) or directories containing .bag files")
    parser.add_argument("--csv", required=True, help="Main CSV output path")
    parser.add_argument("--phase", choices=("pre_terminal", "terminal", "all", "both"), default="both")
    parser.add_argument("--path-topic", default="/scout/global_path_fixed")
    parser.add_argument("--terminal-distance", type=float, default=0.70)
    args = parser.parse_args()

    bags = collect_bags(args.inputs)
    if not bags:
        raise SystemExit("No .bag files found")

    phases = ["pre_terminal", "terminal"] if args.phase == "both" else [args.phase]
    all_rows = []
    for bag_path in bags:
        meta = read_meta_for_bag(bag_path)
        method = infer_method(bag_path, meta)
        print(f"[bag] {bag_path} method={method}")
        data = read_bag_series(bag_path, args.path_topic, args.terminal_distance)
        for phase in phases:
            all_rows.append(summarize_bag_phase(bag_path, data, meta, method, phase))

    csv_path = Path(args.csv).expanduser().resolve()
    write_csv(csv_path, all_rows)

    if args.phase == "both":
        pre_rows = [row for row in all_rows if row["phase"] == "pre_terminal"]
        terminal_rows = [row for row in all_rows if row["phase"] == "terminal"]
        write_csv(csv_path.with_name(csv_path.stem + "_pre_terminal.csv"), pre_rows)
        write_csv(csv_path.with_name(csv_path.stem + "_terminal.csv"), terminal_rows)

    summary_rows = group_summary(all_rows)
    write_csv(csv_path.with_name(csv_path.stem + "_group_summary.csv"), summary_rows)
    print(f"[done] wrote {csv_path}")


if __name__ == "__main__":
    main()
