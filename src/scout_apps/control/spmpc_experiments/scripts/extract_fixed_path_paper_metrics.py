#!/usr/bin/env python3
"""Extract fixed-path paper metrics with explicit evaluation windows.

This script is intentionally conservative: missing topics produce NaN fields instead of
silent zeros. It supports SPMPC topics, older /slosh/* and /mpc_status bags, and
external baseline bags recorded by the fixed-path suite scripts.
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

STATUS_TOPICS = (
    "/spmpc/status",
    "/baseline/status",
    "/baseline/teb/status",
    "/baseline/dwa/status",
    "/baseline/mpc_local_planner/status",
    "/baseline/lt_dwa/status",
    "/mpc_status",
)

WAITING_STATUS_KEYS = (
    "WAITING",
    "NO_ODOM",
    "NO_REFERENCE",
    "NO_PATH",
    "NO_TF",
)

SAFETY_ABORT_STATUS_KEYS = (
    "SAFETY_ABORT",
    "UNSAFE",
    "SPIN_FAIL",
    "TRACKING_SPIN_FAIL",
    "TERMINAL_SPIN_FAIL",
)

SOLVER_FAIL_STATUS_KEYS = (
    "SOLVER_FAIL",
    "SOLVE_FAILED",
    "ACADOS_SOLVE_FAILED",
    "ACADOS_FAILED",
    "ACADOS_NOT",
    "NO_VALID_CMD",
    "SET_PLAN_FAILED",
    "PLUGIN_ERROR",
)

REACHED_STATUS_KEYS = (
    "GOAL_REACHED",
    "REACHED",
)

TERMINAL_MODE_REACHED_KEYS = (
    "REACHED",
)

TERMINAL_MODE_ACTIVE_KEYS = (
    "TERMINAL",
    "SLOWDOWN",
    "CAPTURE",
)

PHASE_GROUPS = {
    "start": {"start"},
    "tracking": {"tracking"},
    "terminal": {"terminal"},
    "reached": {"reached"},
    "waiting": {"waiting"},
    "safety_abort": {"safety_abort"},
    "solver_fail": {"solver_fail"},
    "motion": {"start", "tracking", "terminal"},
    "core": {"tracking", "terminal"},
    "pre_terminal": {"start", "tracking"},
    "all": None,
}



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



def positive_ratio(values, threshold=1e-3):
    vals = finite_values(values)
    if not vals:
        return nan()
    return sum(1 for v in vals if v > threshold) / len(vals)



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
    for key in ("b_ours", "b_slosh", "b_smooth", "b_accel", "b_inst_excitation", "b0"):
        if key in name:
            return f"spmpc_{key}"
    for key in ("teb", "dwa", "mpc_local_planner", "mpc"):
        if key in name:
            return "mpc_local_planner" if key == "mpc" else key
    return "unknown"



def path_topic_from_meta(meta, default="/scout/global_path_fixed"):
    return meta.get("path_topic", default)



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



def terminal_debug_sample(t, terminal_phase, pre_terminal_phase, reached=False, distance_to_goal=nan(), source="debug"):
    return {
        "t": t,
        "terminal_phase": bool(terminal_phase),
        "pre_terminal_phase": bool(pre_terminal_phase),
        "reached": bool(reached),
        "distance_to_goal": distance_to_goal,
        "source": source,
    }



def latest_item(samples, t, time_getter):
    if not samples:
        return None
    times = [time_getter(item) for item in samples]
    idx = bisect.bisect_right(times, t) - 1
    if idx < 0:
        return None
    return samples[idx]



def latest_terminal_sample(terminal_samples, t):
    return latest_item(terminal_samples, t, lambda item: item["t"])



def latest_string_sample(samples, t):
    item = latest_item(samples, t, lambda item: item[0])
    return item[1] if item is not None else ""



def read_bag_series(bag_path, path_topic, terminal_threshold, reached_threshold):
    data = {
        "path": [],
        "odom": [],
        "cmd": [],
        "slosh_height_mm": [],
        "eta_dot_norm": [],
        "solve_time_ms": [],
        "projector_distance": [],
        "stage0_reference": [],
        "status": [],
        "status_topics": set(),
        "terminal_debug": [],
        "terminal_mode": [],
        "topics": set(),
        "start_time": nan(),
        "eval": {},
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
        "/spmpc/debug/projector",
        "/spmpc/debug/stage0_reference",
        "/spmpc/terminal/debug",
        "/spmpc/terminal/mode",
        *STATUS_TOPICS,
    ]
    with rosbag.Bag(str(bag_path), "r") as bag:
        data["start_time"] = bag.get_start_time()
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
            elif topic == "/spmpc/debug/projector":
                arr = list(getattr(msg, "data", []))
                if len(arr) >= 10:
                    data["projector_distance"].append((t, abs(float(arr[9]))))
                elif len(arr) >= 3:
                    data["projector_distance"].append((t, abs(float(arr[2]))))
            elif topic == "/spmpc/debug/stage0_reference":
                arr = list(getattr(msg, "data", []))
                if len(arr) >= 10:
                    data["stage0_reference"].append(
                        {
                            "t": t,
                            "yaw_error": abs(float(arr[8])),
                            "contour_error": abs(float(arr[9])),
                        }
                    )
            elif topic in STATUS_TOPICS:
                data["status"].append((t, str(getattr(msg, "data", ""))))
                data["status_topics"].add(topic)
            elif topic == "/spmpc/terminal/mode":
                data["terminal_mode"].append((t, str(getattr(msg, "data", ""))))
            elif topic == "/spmpc/terminal/debug":
                arr = list(getattr(msg, "data", []))
                if len(arr) >= 3:
                    reached = bool(arr[8] >= 0.5) if len(arr) >= 9 else False
                    distance_to_goal = float(arr[9]) if len(arr) >= 10 else nan()
                    data["terminal_debug"].append(
                        terminal_debug_sample(
                            t,
                            terminal_phase=arr[1] >= 0.5,
                            pre_terminal_phase=arr[2] >= 0.5,
                            reached=reached,
                            distance_to_goal=distance_to_goal,
                            source="terminal_debug",
                        )
                    )

    if not data["terminal_debug"]:
        derive_terminal_phase(data, terminal_threshold, reached_threshold)
    return data



def derive_terminal_phase(data, terminal_threshold, reached_threshold):
    pts = data["path"]
    if len(pts) < 1:
        return
    gx, gy, _ = pts[-1]
    terminal_latched = False
    reached_latched = False
    for row in data["odom"]:
        dist = math.hypot(row["x"] - gx, row["y"] - gy)
        if dist <= terminal_threshold:
            terminal_latched = True
        if (
            dist <= reached_threshold
            and abs(row["v"]) <= 0.05
            and abs(row["omega"]) <= 0.08
        ):
            reached_latched = True
            terminal_latched = True
        data["terminal_debug"].append(
            terminal_debug_sample(
                row["t"],
                terminal_phase=terminal_latched,
                pre_terminal_phase=not terminal_latched,
                reached=reached_latched,
                distance_to_goal=dist,
                source="geometry_fallback",
            )
        )



def first_sample_time(data):
    times = []
    for key in ("odom", "cmd"):
        times.extend(row["t"] for row in data[key])
    for key in ("slosh_height_mm", "eta_dot_norm", "solve_time_ms", "projector_distance", "status", "terminal_mode"):
        times.extend(row[0] for row in data[key])
    times.extend(row["t"] for row in data["stage0_reference"])
    times.extend(row["t"] for row in data["terminal_debug"])
    return min(times) if times else nan()



def status_window(status):
    upper = status.strip().upper()
    if not upper:
        return None
    if any(key in upper for key in SAFETY_ABORT_STATUS_KEYS):
        return "safety_abort"
    if any(key in upper for key in SOLVER_FAIL_STATUS_KEYS):
        return "solver_fail"
    if any(key in upper for key in WAITING_STATUS_KEYS):
        return "waiting"
    if any(key in upper for key in REACHED_STATUS_KEYS):
        return "reached"
    return None



def active_tracking_status(status):
    upper = status.strip().upper()
    if not upper or status_window(upper) is not None:
        return False
    return "TRACKING" in upper or "OK" in upper or "ACADOS" in upper



def infer_motion_start(odom, motion_threshold, motion_consecutive):
    consec = 0
    for row in odom:
        if abs(row["v"]) > motion_threshold or abs(row["omega"]) > motion_threshold:
            consec += 1
            if consec >= max(1, motion_consecutive):
                return row["t"]
        else:
            consec = 0
    return nan()



def infer_tracking_start(status_samples):
    for t, status in status_samples:
        if active_tracking_status(status):
            return t
    return nan()



def preferred_status_topic(data):
    for topic in STATUS_TOPICS:
        if topic in data["status_topics"]:
            return topic
    return ""



def annotate_eval_windows(data, motion_threshold, motion_consecutive):
    motion_start = infer_motion_start(data["odom"], motion_threshold, motion_consecutive)
    tracking_start = infer_tracking_start(data["status"])
    first_time = first_sample_time(data)

    if math.isfinite(motion_start):
        analysis_start = motion_start
        source = "odom_motion"
    elif math.isfinite(tracking_start):
        analysis_start = tracking_start
        source = "status_tracking"
    else:
        analysis_start = first_time
        source = "first_sample" if math.isfinite(first_time) else "missing"

    data["eval"] = {
        "tracking_start_sec": tracking_start,
        "motion_start_sec": motion_start,
        "analysis_start_sec": analysis_start,
        "window_source": source,
        "status_topic_used": preferred_status_topic(data),
        "has_legacy_mpc_status": int("/mpc_status" in data["topics"]),
        "has_eval_window": int("/spmpc/eval/window" in data["topics"]),
    }



def terminal_mode_window(mode):
    upper = mode.strip().upper()
    if not upper:
        return None
    if any(key in upper for key in TERMINAL_MODE_REACHED_KEYS):
        return "reached"
    if any(key in upper for key in TERMINAL_MODE_ACTIVE_KEYS):
        return "terminal"
    if "TRACKING" in upper:
        return "tracking"
    return None



def eval_window_at(data, t):
    status_state = status_window(latest_string_sample(data["status"], t))
    if status_state in ("safety_abort", "solver_fail", "waiting"):
        return status_state
    if status_state == "reached":
        return "reached"

    terminal = latest_terminal_sample(data["terminal_debug"], t)
    if terminal is not None and terminal.get("reached", False):
        return "reached"

    mode_state = terminal_mode_window(latest_string_sample(data["terminal_mode"], t))
    if mode_state == "reached":
        return "reached"
    if terminal is not None and terminal.get("terminal_phase", False):
        return "terminal"
    if mode_state == "terminal":
        return "terminal"

    analysis_start = data.get("eval", {}).get("analysis_start_sec", nan())
    if math.isfinite(analysis_start) and t < analysis_start:
        return "start"
    return "tracking"



def phase_matches(window, phase):
    allowed = PHASE_GROUPS.get(phase)
    if allowed is None:
        return True
    return window in allowed



def sample_time(item):
    return item["t"] if isinstance(item, dict) else item[0]



def phase_filter(samples, phase, data):
    if phase == "all":
        return samples
    out = []
    for item in samples:
        t = sample_time(item)
        if phase_matches(eval_window_at(data, t), phase):
            out.append(item)
    return out



def tuple_phase_filter(samples, phase, data):
    return phase_filter(samples, phase, data)



def window_duration(data, phase):
    odom = phase_filter(data["odom"], phase, data)
    if len(odom) >= 2:
        return max(0.0, odom[-1]["t"] - odom[0]["t"])
    cmd = phase_filter(data["cmd"], phase, data)
    if len(cmd) >= 2:
        return max(0.0, cmd[-1]["t"] - cmd[0]["t"])
    return 0.0



def rel_time(data, t):
    start = data.get("start_time", nan())
    if math.isfinite(t) and math.isfinite(start):
        return t - start
    return nan()



def summarize_bag_phase(bag_path, data, meta, method, phase):
    odom = phase_filter(data["odom"], phase, data)
    cmd = phase_filter(data["cmd"], phase, data)
    slosh = tuple_phase_filter(data["slosh_height_mm"], phase, data)
    eta_dot = tuple_phase_filter(data["eta_dot_norm"], phase, data)
    solve = tuple_phase_filter(data["solve_time_ms"], phase, data)

    stage0 = phase_filter(data["stage0_reference"], phase, data)
    projector = tuple_phase_filter(data["projector_distance"], phase, data)
    track_errors = []
    heading_errors = []
    if stage0:
        track_errors = [row["contour_error"] for row in stage0]
        heading_errors = [row["yaw_error"] for row in stage0]
    elif projector:
        track_errors = [value for _, value in projector]
    else:
        for row in odom:
            e, h = nearest_path_error(row["x"], row["y"], row["yaw"], data["path"])
            track_errors.append(e)
            heading_errors.append(abs(h))

    cmd_v_values = [row["v"] for row in cmd]
    odom_v_values = [row["v"] for row in odom]
    odom_v_abs_values = [abs(row["v"]) for row in odom]
    cmd_v_acc = derivative([(row["t"], row["v"]) for row in cmd])
    cmd_w_acc = derivative([(row["t"], row["omega"]) for row in cmd])
    odom_ax = derivative([(row["t"], row["v"]) for row in odom])
    odom_ay_values = [row["v"] * row["omega"] for row in odom]

    if odom:
        duration = max(0.0, odom[-1]["t"] - odom[0]["t"])
    else:
        duration = 0.0

    final_dist = nan()
    terminal_distances = [row["distance_to_goal"] for row in data["terminal_debug"]]
    terminal_distances = finite_values(terminal_distances)
    if terminal_distances:
        final_dist = terminal_distances[-1]
    elif data["path"] and data["odom"]:
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

    eval_info = data.get("eval", {})
    row = {
        "bag": str(bag_path),
        "method": method,
        "variant": meta.get("variant", ""),
        "experiment_group": meta.get("experiment_group", ""),
        "evidence_chain_version": meta.get("evidence_chain_version", ""),
        "solver_backend": meta.get("solver_backend", meta.get("spmpc_solver_backend", "")),
        "slosh_monitor_enable": meta.get("slosh_monitor_enable", ""),
        "slosh_eval_only": meta.get("slosh_eval_only", ""),
        "baseline_config": meta.get("baseline_config", ""),
        "speed_tier": meta.get("speed_tier", ""),
        "limit_profile": meta.get("limit_profile", ""),
        "target_v_max_mps": meta.get("target_v_max_mps", ""),
        "target_omega_max_radps": meta.get("target_omega_max_radps", ""),
        "target_acc_lim_x_mps2": meta.get("target_acc_lim_x_mps2", ""),
        "target_acc_lim_theta_radps2": meta.get("target_acc_lim_theta_radps2", ""),
        "v_ref_override": meta.get("v_ref_override", ""),
        "path_id": meta.get("path_id", ""),
        "run_id": meta.get("run_id", bag_path.stem),
        "phase": phase,
        "duration_s": duration,
        "phase_sample_count": len(odom),
        "cmd_v_mean_mps": safe_mean(cmd_v_values),
        "cmd_v_p95_mps": percentile(cmd_v_values, 95),
        "cmd_v_max_mps": max(finite_values(cmd_v_values), default=nan()),
        "cmd_v_positive_ratio": positive_ratio(cmd_v_values),
        "odom_v_abs_mean_mps": safe_mean(odom_v_abs_values),
        "odom_v_abs_p95_mps": percentile(odom_v_abs_values, 95),
        "odom_v_abs_max_mps": max(finite_values(odom_v_abs_values), default=nan()),
        "odom_v_positive_ratio": positive_ratio(odom_v_values),
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
        "cmd_dvx_abs_p95_mps2": percentile([abs(v) for _, v in cmd_v_acc], 95),
        "cmd_dvx_abs_max_mps2": max_abs([v for _, v in cmd_v_acc]),
        "cmd_dwz_rms_radps2": rms([v for _, v in cmd_w_acc]),
        "cmd_dwz_abs_p95_radps2": percentile([abs(v) for _, v in cmd_w_acc], 95),
        "cmd_dwz_abs_max_radps2": max_abs([v for _, v in cmd_w_acc]),
        "odom_ax_abs_p95_mps2": percentile([abs(v) for _, v in odom_ax], 95),
        "odom_ay_abs_p95_mps2": percentile([abs(v) for v in odom_ay_values], 95),
        "solver_time_mean_ms": safe_mean([v for _, v in solve]),
        "solver_time_p95_ms": percentile([v for _, v in solve], 95),
        "final_stop_distance_m": final_dist,
        "final_speed_mps": final_speed,
        "final_omega_radps": final_omega,
        "stable_stop_bool": stable_stop,
        "analysis_start_sec": rel_time(data, eval_info.get("analysis_start_sec", nan())),
        "tracking_start_sec": rel_time(data, eval_info.get("tracking_start_sec", nan())),
        "motion_start_sec": rel_time(data, eval_info.get("motion_start_sec", nan())),
        "window_source": eval_info.get("window_source", ""),
        "status_topic_used": eval_info.get("status_topic_used", ""),
        "has_legacy_mpc_status": eval_info.get("has_legacy_mpc_status", 0),
        "has_eval_window": eval_info.get("has_eval_window", 0),
        "reached_tail_duration_s": window_duration(data, "reached"),
        "has_odom": int("/odom" in data["topics"]),
        "has_cmd_vel": int("/cmd_vel" in data["topics"]),
        "has_path": int(path_topic_from_meta(meta) in data["topics"] or bool(data["path"])),
        "has_slosh_state": int("/spmpc/debug/slosh_state" in data["topics"] or "/slosh/state" in data["topics"]),
        "has_spmpc_projector": int("/spmpc/debug/projector" in data["topics"]),
        "has_spmpc_stage0_reference": int("/spmpc/debug/stage0_reference" in data["topics"]),
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
        "phase_sample_count",
        "cmd_v_mean_mps",
        "cmd_v_p95_mps",
        "cmd_v_max_mps",
        "cmd_v_positive_ratio",
        "odom_v_abs_mean_mps",
        "odom_v_abs_p95_mps",
        "odom_v_abs_max_mps",
        "odom_v_positive_ratio",
        "tracking_error_rms_m",
        "tracking_error_p95_m",
        "heading_error_p95_deg",
        "slosh_height_peak_mm",
        "slosh_height_p95_mm",
        "eta_dot_norm_peak",
        "eta_dot_norm_rms",
        "cmd_dvx_rms_mps2",
        "cmd_dvx_abs_p95_mps2",
        "cmd_dvx_abs_max_mps2",
        "cmd_dwz_rms_radps2",
        "cmd_dwz_abs_p95_radps2",
        "cmd_dwz_abs_max_radps2",
        "odom_ax_abs_p95_mps2",
        "odom_ay_abs_p95_mps2",
        "solver_time_mean_ms",
        "solver_time_p95_ms",
        "final_stop_distance_m",
        "final_speed_mps",
        "final_omega_radps",
        "stable_stop_bool",
        "analysis_start_sec",
        "tracking_start_sec",
        "motion_start_sec",
        "reached_tail_duration_s",
    ]
    for (method, path_id, phase), items in sorted(groups.items()):
        out = {"method": method, "path_id": path_id, "phase": phase, "run_count": len(items)}
        for key in numeric_keys:
            out[f"{key}_mean"] = safe_mean([item[key] for item in items])
            out[f"{key}_p95"] = percentile([item[key] for item in items], 95)
        summary.append(out)
    return summary



def expand_phases(phase):
    if phase == "both":
        return ["pre_terminal", "terminal"]
    if phase == "windows":
        return ["start", "tracking", "terminal", "reached", "motion", "core", "safety_abort", "solver_fail"]
    return [phase]



def write_phase_sidecars(csv_path, phase, rows):
    if phase == "both":
        pre_rows = [row for row in rows if row["phase"] == "pre_terminal"]
        terminal_rows = [row for row in rows if row["phase"] == "terminal"]
        write_csv(csv_path.with_name(csv_path.stem + "_pre_terminal.csv"), pre_rows)
        write_csv(csv_path.with_name(csv_path.stem + "_terminal.csv"), terminal_rows)
    elif phase == "windows":
        for name in ("start", "tracking", "terminal", "reached", "motion", "core"):
            phase_rows = [row for row in rows if row["phase"] == name]
            write_csv(csv_path.with_name(csv_path.stem + f"_{name}.csv"), phase_rows)



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="Bag file(s) or directories containing .bag files")
    parser.add_argument("--csv", required=True, help="Main CSV output path")
    parser.add_argument(
        "--phase",
        choices=(
            "pre_terminal",
            "terminal",
            "all",
            "both",
            "start",
            "tracking",
            "reached",
            "motion",
            "core",
            "waiting",
            "safety_abort",
            "solver_fail",
            "windows",
        ),
        default="both",
    )
    parser.add_argument("--path-topic", default="/scout/global_path_fixed")
    parser.add_argument("--terminal-distance", type=float, default=0.70)
    parser.add_argument("--reached-distance", type=float, default=0.35)
    parser.add_argument("--motion-threshold", type=float, default=0.03)
    parser.add_argument("--motion-consecutive", type=int, default=3)
    args = parser.parse_args()

    bags = collect_bags(args.inputs)
    if not bags:
        raise SystemExit("No .bag files found")

    phases = expand_phases(args.phase)
    all_rows = []
    for bag_path in bags:
        meta = read_meta_for_bag(bag_path)
        method = infer_method(bag_path, meta)
        path_topic = path_topic_from_meta(meta, args.path_topic)
        print(f"[bag] {bag_path} method={method}")
        data = read_bag_series(bag_path, path_topic, args.terminal_distance, args.reached_distance)
        annotate_eval_windows(data, args.motion_threshold, args.motion_consecutive)
        for phase in phases:
            all_rows.append(summarize_bag_phase(bag_path, data, meta, method, phase))

    csv_path = Path(args.csv).expanduser().resolve()
    write_csv(csv_path, all_rows)
    write_phase_sidecars(csv_path, args.phase, all_rows)

    summary_rows = group_summary(all_rows)
    write_csv(csv_path.with_name(csv_path.stem + "_group_summary.csv"), summary_rows)
    print(f"[done] wrote {csv_path}")


if __name__ == "__main__":
    main()
