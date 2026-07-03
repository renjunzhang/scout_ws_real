#!/usr/bin/env python3
"""Summarize one SPMPC real fixed-path trial bag under the model-as-truth debug frame.

The script is offline-only: it reads rosbag + recorder sidecars, then writes a
JSON and Markdown summary next to the bag unless --out-dir is provided.  It uses
Float32MultiArray layout labels instead of hard-coded indices so new diagnostic
topics can stay additive.
"""

import argparse
import json
import math
import statistics as st
import sys
from collections import Counter, defaultdict
from pathlib import Path

try:
    import rosbag  # type: ignore
except ImportError:  # pragma: no cover - depends on sourced ROS environment
    rosbag = None


STRING_TOPICS = {
    "/spmpc/status",
    "/spmpc/controller_variant",
    "/spmpc/solver_backend",
    "/spmpc/debug/map_vref_status",
    "/spmpc/debug/execution_alignment_status",
}
FLOAT_TOPICS = {
    "/spmpc/solver_time_ms": "solver_time_ms",
    "/spmpc/debug/progress_s": "progress_s",
    "/spmpc/debug/v_ref_current": "v_ref_current",
    "/spmpc/slosh_height": "slosh_height_mm",
}
MULTIARRAY_TOPICS = {
    "/spmpc/debug/effective_config",
    "/spmpc/debug/cmd_odom_alignment",
    "/spmpc/debug/raw_state",
    "/spmpc/debug/predicted_state",
    "/spmpc/debug/solver_input_state",
    "/spmpc/debug/command_intervention",
    "/spmpc/debug/slosh_cost_monitor",
    "/spmpc/slosh_horizon_summary",
    "/spmpc/cost_breakdown",
    "/spmpc/debug/slosh_hard_constraint_effective",
    "/spmpc/debug/cmd_vel_output",
}
CMD_TOPIC = "/cmd_vel"
CRITICAL_TOPICS = [
    "/spmpc/debug/raw_state",
    "/spmpc/debug/predicted_state",
    "/spmpc/debug/solver_input_state",
    "/spmpc/debug/command_intervention",
    "/spmpc/debug/effective_config",
    "/spmpc/debug/cmd_odom_alignment",
    "/spmpc/debug/slosh_cost_monitor",
    "/spmpc/slosh_horizon_summary",
    "/spmpc/cost_breakdown",
]
ZERO_REASON_FIELDS = [
    "zero_due_to_solver_failure",
    "zero_due_to_waiting_for_odom",
    "zero_due_to_waiting_for_reference",
    "zero_due_to_waiting_for_tf",
    "zero_due_to_terminal_spin_fail",
    "zero_due_to_tracking_safety",
]


def finite(value):
    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def clean_json(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [clean_json(v) for v in value]
    return value


def percentile(values, q):
    xs = sorted(float(v) for v in values if finite(v))
    if not xs:
        return float("nan")
    if len(xs) == 1:
        return xs[0]
    pos = (len(xs) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return xs[lo]
    frac = pos - lo
    return xs[lo] * (1.0 - frac) + xs[hi] * frac


def numeric_summary(values):
    xs = [float(v) for v in values if finite(v)]
    if not xs:
        return {"n": 0}
    return {
        "n": len(xs),
        "mean": st.mean(xs),
        "min": min(xs),
        "p50": percentile(xs, 0.50),
        "p95": percentile(xs, 0.95),
        "max": max(xs),
        "rms": math.sqrt(sum(v * v for v in xs) / len(xs)),
    }


def fraction_true(values):
    xs = [float(v) for v in values if finite(v)]
    if not xs:
        return float("nan")
    return sum(1 for v in xs if abs(v) > 0.5) / len(xs)


def last_finite(values):
    for value in reversed(values):
        if finite(value):
            return float(value)
    return float("nan")


def fmt(value, digits=3):
    if not finite(value):
        return "NA"
    return f"{float(value):.{digits}f}"


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_scalar_text(text):
    value = text.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        value = value[1:-1]
    return value


def parse_kv_sidecar(path):
    data = {}
    if not path or not path.exists():
        return data
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return data
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        key = None
        value = None
        if "=" in line:
            key, value = line.split("=", 1)
        elif ":" in line:
            key, value = line.split(":", 1)
        if key is None:
            continue
        key = key.strip()
        if not key:
            continue
        data[key] = parse_scalar_text(value)
    return data


def parse_list_sidecar(path):
    if not path or not path.exists():
        return []
    try:
        return [line.strip() for line in path.read_text(encoding="utf-8", errors="replace").splitlines() if line.strip()]
    except OSError:
        return []


def sidecar_paths_for_bag(bag_path):
    stem = bag_path.stem
    parent = bag_path.parent
    paths = {
        "info": parent / f"{stem}_info.txt",
        "one_click_meta": parent / f"{stem}_one_click_meta.env",
        "rosparam": parent / f"{stem}_rosparam.yaml",
        "bag_info": parent / f"{stem}_bag_info.txt",
        "recorded_topics": parent / f"{stem}_recorded_topics.txt",
        "selected_topics_not_recorded": parent / f"{stem}_selected_topics_not_recorded.txt",
    }
    return {name: path for name, path in paths.items() if path.exists()}


def collect_sidecar_meta(bag_path):
    paths = sidecar_paths_for_bag(bag_path)
    meta = {}
    for key in ("info", "one_click_meta"):
        meta.update(parse_kv_sidecar(paths.get(key)))
    for key, value in list(meta.items()):
        meta.setdefault(key.lower(), value)
    return {
        "paths": {name: str(path) for name, path in paths.items()},
        "meta": meta,
        "selected_topics_not_recorded": parse_list_sidecar(paths.get("selected_topics_not_recorded")),
        "recorded_topics_sidecar": parse_list_sidecar(paths.get("recorded_topics")),
    }


def find_bags(paths):
    bags = []
    for raw in paths:
        path = Path(raw).expanduser()
        if path.is_file() and path.suffix == ".bag":
            bags.append(path)
        elif path.is_dir():
            direct = sorted(path.glob("*.bag"))
            bags.extend(direct if direct else sorted(path.rglob("*.bag")))
        else:
            print(f"[WARN] skip non-bag path: {path}", file=sys.stderr)
    return sorted(dict.fromkeys(bags))


def parse_multiarray(msg):
    dims = getattr(getattr(msg, "layout", None), "dim", [])
    label = dims[0].label if dims else ""
    fields = [part.strip() for part in label.split(",") if part.strip()]
    data = list(getattr(msg, "data", []))
    parsed = {}
    for idx, value in enumerate(data):
        name = fields[idx] if idx < len(fields) else f"field_{idx}"
        try:
            parsed[name] = float(value)
        except (TypeError, ValueError):
            parsed[name] = float("nan")
    return parsed


def bag_topic_info(bag):
    try:
        info = bag.get_type_and_topic_info()
        topics = info.topics
    except Exception:
        return {}
    out = {}
    for topic, item in topics.items():
        out[topic] = {
            "type": getattr(item, "msg_type", ""),
            "messages": int(getattr(item, "message_count", 0)),
        }
    return out


def read_bag_data(bag_path):
    if rosbag is None:
        raise RuntimeError("需要 rosbag：请先 source /opt/ros/noetic/setup.bash 和工作区 devel/setup.bash")

    data = {
        "topic_info": {},
        "topic_counts": Counter(),
        "strings": defaultdict(list),
        "floats": defaultdict(list),
        "multi": defaultdict(list),
        "cmd_vel": [],
        "start_time": None,
        "end_time": None,
    }
    topics = set(STRING_TOPICS) | set(FLOAT_TOPICS) | MULTIARRAY_TOPICS | {CMD_TOPIC}
    with rosbag.Bag(str(bag_path)) as bag:
        data["topic_info"] = bag_topic_info(bag)
        try:
            data["start_time"] = float(bag.get_start_time())
            data["end_time"] = float(bag.get_end_time())
        except Exception:
            pass
        for topic, msg, _ in bag.read_messages(topics=sorted(topics)):
            data["topic_counts"][topic] += 1
            if topic in STRING_TOPICS:
                data["strings"][topic].append(str(getattr(msg, "data", "")))
            elif topic in FLOAT_TOPICS:
                data["floats"][FLOAT_TOPICS[topic]].append(float(getattr(msg, "data", float("nan"))))
            elif topic in MULTIARRAY_TOPICS:
                data["multi"][topic].append(parse_multiarray(msg))
            elif topic == CMD_TOPIC:
                data["cmd_vel"].append({"v": float(msg.linear.x), "omega": float(msg.angular.z)})
    return data


def series_field(samples, field):
    return [sample[field] for sample in samples if field in sample and finite(sample[field])]


def series_field_any(samples, fields):
    values = []
    for sample in samples:
        for field in fields:
            if field in sample and finite(sample[field]):
                values.append(sample[field])
                break
    return values


def status_counts(strings):
    return dict(Counter(strings))


def pair_state_deltas(raw_samples, solver_samples):
    n = min(len(raw_samples), len(solver_samples))
    yaw = []
    omega = []
    eta = []
    eta_dot = []
    h = []
    for raw, solver in zip(raw_samples[:n], solver_samples[:n]):
        if finite(raw.get("yaw")) and finite(solver.get("yaw")):
            yaw.append(abs(wrap_angle(solver["yaw"] - raw["yaw"])))
        if finite(raw.get("omega")) and finite(solver.get("omega")):
            omega.append(abs(solver["omega"] - raw["omega"]))
        if all(finite(x) for x in (raw.get("eta_x"), raw.get("eta_y"), solver.get("eta_x"), solver.get("eta_y"))):
            eta.append(math.hypot(solver["eta_x"] - raw["eta_x"], solver["eta_y"] - raw["eta_y"]))
        if all(finite(x) for x in (raw.get("eta_x_dot"), raw.get("eta_y_dot"), solver.get("eta_x_dot"), solver.get("eta_y_dot"))):
            eta_dot.append(math.hypot(solver["eta_x_dot"] - raw["eta_x_dot"], solver["eta_y_dot"] - raw["eta_y_dot"]))
        if finite(raw.get("h_modal_mm")) and finite(solver.get("h_modal_mm")):
            h.append(abs(solver["h_modal_mm"] - raw["h_modal_mm"]))
    return {
        "pairs": n,
        "abs_yaw_rad": numeric_summary(yaw),
        "abs_omega": numeric_summary(omega),
        "eta_norm": numeric_summary(eta),
        "eta_dot_norm": numeric_summary(eta_dot),
        "h_modal_mm": numeric_summary(h),
    }


def summarize_command_intervention(samples):
    published_v = series_field(samples, "published_cmd_v")
    published_w = series_field(samples, "published_cmd_omega")
    post_gate_v = series_field(samples, "post_gate_cmd_v")
    post_gate_w = series_field(samples, "post_gate_cmd_omega")
    n = min(len(samples), len(post_gate_v), len(post_gate_w), len(published_v), len(published_w))
    mismatch_v = []
    mismatch_w = []
    for sample in samples[:n]:
        if finite(sample.get("published_cmd_v")) and finite(sample.get("post_gate_cmd_v")):
            mismatch_v.append(abs(sample["published_cmd_v"] - sample["post_gate_cmd_v"]))
        if finite(sample.get("published_cmd_omega")) and finite(sample.get("post_gate_cmd_omega")):
            mismatch_w.append(abs(sample["published_cmd_omega"] - sample["post_gate_cmd_omega"]))
    zero_samples = [1.0 if abs(s.get("published_cmd_v", 0.0)) <= 1e-6 and abs(s.get("published_cmd_omega", 0.0)) <= 1e-6 else 0.0 for s in samples]
    linear_limited_frac = fraction_true(series_field(samples, "linear_limited"))
    angular_rate_limited_frac = fraction_true(series_field(samples, "angular_rate_limited"))
    angular_accel_limited_frac = fraction_true(series_field(samples, "angular_accel_limited"))
    limiter_candidates = [
        v for v in (linear_limited_frac, angular_rate_limited_frac, angular_accel_limited_frac)
        if finite(v)
    ]
    return {
        "samples": len(samples),
        "published_zero_frac": fraction_true(zero_samples),
        "linear_limited_frac": linear_limited_frac,
        "angular_rate_limited_frac": angular_rate_limited_frac,
        "angular_accel_limited_frac": angular_accel_limited_frac,
        "command_limiter_frac": max(limiter_candidates) if limiter_candidates else float("nan"),
        "published_minus_post_gate_v_abs": numeric_summary(mismatch_v),
        "published_minus_post_gate_omega_abs": numeric_summary(mismatch_w),
        "zero_reason_counts": {field: int(sum(1 for s in samples if abs(s.get(field, 0.0)) > 0.5)) for field in ZERO_REASON_FIELDS},
    }


def compare_intent_effective(sidecar_meta, effective_samples):
    result = {"mismatches": []}
    if not effective_samples:
        return result
    last = effective_samples[-1]
    checks = [
        ("delay_phase_linear_delay_sec", "delay_linear_sec", 1e-3),
        ("delay_phase_angular_delay_sec", "delay_angular_sec", 1e-3),
        ("v_ref", "v_ref", 1e-3),
        ("w_slosh", "w_slosh", 1e-3),
        ("slosh_height_max", "slosh_height_max", 1e-4),
    ]
    for meta_key, field, tol in checks:
        if meta_key not in sidecar_meta or field not in last:
            continue
        try:
            intent = float(sidecar_meta[meta_key])
        except (TypeError, ValueError):
            continue
        effective = last[field]
        if finite(effective) and abs(intent - effective) > tol:
            result["mismatches"].append({"intent": meta_key, "effective_field": field, "intent_value": intent, "effective_value": effective})
    return result


def build_red_flags(summary):
    flags = []
    missing = summary["topics"].get("critical_missing", [])
    if missing:
        flags.append({"code": "critical_topic_missing", "detail": ", ".join(missing)})

    delay = summary["metrics"].get("delay_state", {})
    intent_mode = str(summary.get("intent", {}).get("delay_phase_mode") or "").lower()
    effective_mode = summary["metrics"].get("effective_config_last", {}).get("delay_phase_mode_code")
    fixed_closed_loop_expected = "fixed_closed_loop" in intent_mode or effective_mode == 3.0
    if fixed_closed_loop_expected and finite(delay.get("delay_compensation_applied_frac")) and delay["delay_compensation_applied_frac"] < 0.95:
        flags.append({"code": "fixed_closed_loop_not_applied", "detail": f"applied_frac={fmt(delay['delay_compensation_applied_frac'])}"})
    if fixed_closed_loop_expected and finite(delay.get("history_complete_frac")) and delay["history_complete_frac"] < 0.95:
        flags.append({"code": "delay_history_incomplete", "detail": f"history_complete_frac={fmt(delay['history_complete_frac'])}"})
    state_delta = delay.get("solver_minus_raw", {})
    yaw_p95 = state_delta.get("abs_yaw_rad", {}).get("p95")
    omega_p95 = state_delta.get("abs_omega", {}).get("p95")
    if (finite(yaw_p95) and yaw_p95 > 0.20) or (finite(omega_p95) and omega_p95 > 0.30):
        flags.append({"code": "solver_input_phase_shift_large", "detail": f"yaw_p95={fmt(yaw_p95)} rad, omega_p95={fmt(omega_p95)}"})

    opt = summary["metrics"].get("optimizer_pressure", {})
    slosh_pct = opt.get("pct_slosh_total_abs_sum", {}).get("p50")
    eta_dot_pct = opt.get("pct_eta_dot_in_slosh", {}).get("p50")
    if finite(slosh_pct) and abs(slosh_pct) < 1.0:
        flags.append({"code": "slosh_cost_inactive", "detail": f"median slosh pct={fmt(slosh_pct)}%"})
    if finite(eta_dot_pct) and abs(eta_dot_pct) < 5.0:
        flags.append({"code": "eta_dot_cost_inactive", "detail": f"median eta_dot share={fmt(eta_dot_pct)}%"})
    if opt.get("h_modal_peak_pred_mm", {}).get("n", 0) == 0:
        flags.append({"code": "horizon_peak_not_reduced_or_missing", "detail": "missing /spmpc/slosh_horizon_summary"})

    cmd = summary["metrics"].get("command_intervention", {})
    limited = cmd.get("command_limiter_frac")
    if finite(limited) and limited > 0.2:
        flags.append({"code": "command_limited_often", "detail": f"max limiter frac={fmt(limited)}"})
    if finite(cmd.get("published_zero_frac")) and cmd["published_zero_frac"] > 0.05:
        flags.append({"code": "published_zero_often", "detail": f"zero_frac={fmt(cmd['published_zero_frac'])}"})
    reason_counts = cmd.get("zero_reason_counts", {})
    bad_reasons = {k: v for k, v in reason_counts.items() if v > 0 and k not in {"zero_due_to_waiting_for_odom", "zero_due_to_waiting_for_reference"}}
    status_counts_map = summary.get("observed", {}).get("status_counts", {})
    solver_fail_count = sum(v for k, v in status_counts_map.items() if "FAIL" in str(k).upper() or "FAILED" in str(k).upper())
    if bad_reasons or solver_fail_count > 0:
        detail = {"zero_reasons": bad_reasons, "status_fail_count": int(solver_fail_count)}
        flags.append({"code": "solver_fail_or_gate_fail", "detail": str(detail)})

    mismatch = summary["metrics"].get("intent_effective", {}).get("mismatches", [])
    if mismatch:
        flags.append({"code": "sidecar_effective_config_mismatch", "detail": str(mismatch[:3])})
    return flags


def summarize_bag(bag_path):
    sidecars = collect_sidecar_meta(bag_path)
    data = read_bag_data(bag_path)
    present_topics = set(data["topic_info"].keys())
    if not present_topics:
        present_topics = set(data["topic_counts"].keys())

    multi = data["multi"]
    raw_samples = multi.get("/spmpc/debug/raw_state", [])
    predicted_samples = multi.get("/spmpc/debug/predicted_state", [])
    solver_input_samples = multi.get("/spmpc/debug/solver_input_state", [])
    cmd_samples = multi.get("/spmpc/debug/command_intervention", [])
    cmd_odom = multi.get("/spmpc/debug/cmd_odom_alignment", [])
    slosh_cost = multi.get("/spmpc/debug/slosh_cost_monitor", [])
    cost_breakdown = multi.get("/spmpc/cost_breakdown", [])
    horizon = multi.get("/spmpc/slosh_horizon_summary", [])
    effective = multi.get("/spmpc/debug/effective_config", [])
    j_slosh_eta = series_field(slosh_cost, "J_slosh_eta") or series_field(cost_breakdown, "J_slosh_eta")
    j_slosh_eta_dot = series_field(slosh_cost, "J_slosh_eta_dot") or series_field(cost_breakdown, "J_slosh_eta_dot")
    j_slosh_total = series_field(slosh_cost, "J_slosh_total")
    if not j_slosh_total:
        j_slosh_total = [s.get("J_slosh_eta", 0.0) + s.get("J_slosh_eta_dot", 0.0)
                         for s in cost_breakdown
                         if finite(s.get("J_slosh_eta")) and finite(s.get("J_slosh_eta_dot"))]
    pct_slosh = series_field(slosh_cost, "pct_slosh_total_abs_sum") or series_field(cost_breakdown, "pct_slosh_total")
    pct_progress = series_field(cost_breakdown, "pct_progress")
    pct_smooth = series_field(cost_breakdown, "pct_smooth")
    pct_control = series_field(cost_breakdown, "pct_control")
    eta_dot_share = series_field(slosh_cost, "pct_eta_dot_in_slosh")
    if not eta_dot_share:
        eta_dot_share = [100.0 * abs(s.get("J_slosh_eta_dot", 0.0)) /
                         max(1e-9, abs(s.get("J_slosh_eta", 0.0)) + abs(s.get("J_slosh_eta_dot", 0.0)))
                         for s in cost_breakdown
                         if finite(s.get("J_slosh_eta")) and finite(s.get("J_slosh_eta_dot"))]

    status = data["strings"].get("/spmpc/status", [])
    duration = None
    if finite(data.get("start_time")) and finite(data.get("end_time")):
        duration = data["end_time"] - data["start_time"]

    summary = {
        "bag": str(bag_path),
        "run_label": sidecars["meta"].get("run_label", bag_path.stem),
        "duration_sec": duration,
        "sidecars": sidecars,
        "topics": {
            "critical_missing": [topic for topic in CRITICAL_TOPICS if topic not in present_topics],
            "counts": {topic: data["topic_info"].get(topic, {}).get("messages", int(data["topic_counts"].get(topic, 0))) for topic in sorted(CRITICAL_TOPICS + [CMD_TOPIC])},
        },
        "intent": {
            "variant": sidecars["meta"].get("variant") or sidecars["meta"].get("VARIANT"),
            "solver_backend": sidecars["meta"].get("solver_backend") or sidecars["meta"].get("SOLVER_BACKEND"),
            "v_ref": sidecars["meta"].get("v_ref") or sidecars["meta"].get("V_REF"),
            "w_slosh": sidecars["meta"].get("w_slosh") or sidecars["meta"].get("W_SLOSH"),
            "delay_phase_mode": sidecars["meta"].get("delay_phase_mode") or sidecars["meta"].get("DELAY_PHASE_MODE"),
            "delay_phase_linear_delay_sec": sidecars["meta"].get("delay_phase_linear_delay_sec") or sidecars["meta"].get("DELAY_PHASE_LINEAR_DELAY_SEC"),
            "delay_phase_angular_delay_sec": sidecars["meta"].get("delay_phase_angular_delay_sec") or sidecars["meta"].get("DELAY_PHASE_ANGULAR_DELAY_SEC"),
            "goal_x": sidecars["meta"].get("goal_x") or sidecars["meta"].get("GOAL_X"),
            "goal_y": sidecars["meta"].get("goal_y") or sidecars["meta"].get("GOAL_Y"),
        },
        "observed": {
            "controller_variant_last": data["strings"].get("/spmpc/controller_variant", [None])[-1] if data["strings"].get("/spmpc/controller_variant") else None,
            "solver_backend_last": data["strings"].get("/spmpc/solver_backend", [None])[-1] if data["strings"].get("/spmpc/solver_backend") else None,
            "status_counts": status_counts(status),
            "goal_reached": "GOAL_REACHED" in status,
            "map_vref_status_counts": status_counts(data["strings"].get("/spmpc/debug/map_vref_status", [])),
        },
        "metrics": {
            "solver_time_ms": numeric_summary(data["floats"].get("solver_time_ms", [])),
            "progress_s": numeric_summary(data["floats"].get("progress_s", [])),
            "v_ref_current": numeric_summary(data["floats"].get("v_ref_current", [])),
            "slosh_height_mm": numeric_summary(data["floats"].get("slosh_height_mm", [])),
            "delay_state": {
                "delay_compensation_applied_frac": fraction_true(series_field(solver_input_samples, "delay_compensation_applied")),
                "history_complete_frac": fraction_true(series_field(cmd_odom, "history_complete")),
                "shadow_valid_frac": fraction_true(series_field(cmd_odom, "shadow_valid")),
                "fixed_closed_loop_applied_frac": fraction_true(series_field(cmd_odom, "fixed_closed_loop_applied")),
                "predicted_valid_frac": fraction_true(series_field(predicted_samples, "valid")),
                "solver_minus_raw": pair_state_deltas(raw_samples, solver_input_samples),
                "cmd_odom_alignment_status_counts": status_counts(data["strings"].get("/spmpc/debug/execution_alignment_status", [])),
            },
            "optimizer_pressure": {
                "J_slosh_eta": numeric_summary(j_slosh_eta),
                "J_slosh_eta_dot": numeric_summary(j_slosh_eta_dot),
                "J_slosh_total": numeric_summary(j_slosh_total),
                "pct_slosh_total_abs_sum": numeric_summary(pct_slosh),
                "pct_eta_dot_in_slosh": numeric_summary(eta_dot_share),
                "pct_progress": numeric_summary(pct_progress),
                "pct_smooth": numeric_summary(pct_smooth),
                "pct_control": numeric_summary(pct_control),
                "eta_norm_peak": numeric_summary(series_field(slosh_cost, "eta_norm_peak")),
                "eta_dot_norm_peak": numeric_summary(series_field(slosh_cost, "eta_dot_norm_peak")),
                "h_modal_peak_pred_mm": numeric_summary(series_field_any(horizon, ["h_modal_peak_pred_mm", "h_peak_pred_mm", "h_peak_pred", "h_peak"])),
                "h_modal_p95_pred_mm": numeric_summary(series_field_any(horizon, ["h_modal_p95_pred_mm", "h_p95_pred_mm", "h_p95_pred", "h_p95"])),
            },
            "command_intervention": summarize_command_intervention(cmd_samples),
            "intent_effective": compare_intent_effective(sidecars["meta"], effective),
            "effective_config_last": effective[-1] if effective else {},
        },
    }
    summary["red_flags"] = build_red_flags(summary)
    return clean_json(summary)


def render_markdown(summary):
    flags = summary.get("red_flags", [])
    metrics = summary.get("metrics", {})
    delay = metrics.get("delay_state", {})
    state_delta = delay.get("solver_minus_raw", {})
    opt = metrics.get("optimizer_pressure", {})
    cmd = metrics.get("command_intervention", {})
    lines = []
    lines.append(f"# SPMPC real trial summary: {summary.get('run_label')}")
    lines.append("")
    lines.append(f"- Bag: `{summary.get('bag')}`")
    lines.append(f"- Duration: {fmt(summary.get('duration_sec'), 1)} s")
    intent = summary.get("intent", {})
    lines.append(f"- Intent: variant=`{intent.get('variant')}`, backend=`{intent.get('solver_backend')}`, v_ref=`{intent.get('v_ref')}`, delay=`{intent.get('delay_phase_linear_delay_sec')}/{intent.get('delay_phase_angular_delay_sec')}`")
    observed = summary.get("observed", {})
    lines.append(f"- Observed: variant=`{observed.get('controller_variant_last')}`, backend=`{observed.get('solver_backend_last')}`, goal_reached=`{observed.get('goal_reached')}`")
    lines.append("")
    lines.append("## Red flags")
    if flags:
        for flag in flags:
            lines.append(f"- `{flag.get('code')}` — {flag.get('detail')}")
    else:
        lines.append("- None from first-pass thresholds.")
    lines.append("")
    lines.append("## Topic coverage")
    missing = summary.get("topics", {}).get("critical_missing", [])
    lines.append(f"- Critical missing: {', '.join(f'`{t}`' for t in missing) if missing else 'none'}")
    counts = summary.get("topics", {}).get("counts", {})
    for topic in CRITICAL_TOPICS:
        lines.append(f"  - `{topic}`: {counts.get(topic, 0)}")
    lines.append("")
    lines.append("## Delay / state input")
    lines.append(f"- delay_compensation_applied_frac: {fmt(delay.get('delay_compensation_applied_frac'))}")
    lines.append(f"- history_complete_frac: {fmt(delay.get('history_complete_frac'))}")
    lines.append(f"- shadow_valid_frac: {fmt(delay.get('shadow_valid_frac'))}")
    lines.append(f"- fixed_closed_loop_applied_frac: {fmt(delay.get('fixed_closed_loop_applied_frac'))}")
    lines.append(f"- solver-input minus raw yaw p95: {fmt(state_delta.get('abs_yaw_rad', {}).get('p95'))} rad")
    lines.append(f"- solver-input minus raw omega p95: {fmt(state_delta.get('abs_omega', {}).get('p95'))}")
    lines.append(f"- solver-input minus raw eta_norm p95: {fmt(state_delta.get('eta_norm', {}).get('p95'))}")
    lines.append("")
    slosh_height = metrics.get("slosh_height_mm", {})
    lines.append("## Optimizer pressure")
    lines.append(f"- internal slosh p95/max: {fmt(slosh_height.get('p95'))} / {fmt(slosh_height.get('max'))} mm")
    lines.append(f"- J_slosh_eta median: {fmt(opt.get('J_slosh_eta', {}).get('p50'))}")
    lines.append(f"- J_slosh_eta_dot median: {fmt(opt.get('J_slosh_eta_dot', {}).get('p50'))}")
    lines.append(f"- J_slosh_total mean: {fmt(opt.get('J_slosh_total', {}).get('mean'))}")
    lines.append(f"- slosh cost pct median/p95: {fmt(opt.get('pct_slosh_total_abs_sum', {}).get('p50'))}% / {fmt(opt.get('pct_slosh_total_abs_sum', {}).get('p95'))}%")
    lines.append(f"- eta_dot share in slosh median: {fmt(opt.get('pct_eta_dot_in_slosh', {}).get('p50'))}%")
    lines.append(f"- progress/smooth/control pct median: {fmt(opt.get('pct_progress', {}).get('p50'))}% / {fmt(opt.get('pct_smooth', {}).get('p50'))}% / {fmt(opt.get('pct_control', {}).get('p50'))}%")
    lines.append(f"- horizon h_peak p95: {fmt(opt.get('h_modal_peak_pred_mm', {}).get('p95'))} mm")
    lines.append(f"- horizon h_p95 p95: {fmt(opt.get('h_modal_p95_pred_mm', {}).get('p95'))} mm")
    lines.append("")
    lines.append("## Command intervention")
    lines.append(f"- published_zero_frac: {fmt(cmd.get('published_zero_frac'))}")
    lines.append(f"- command_limiter_frac: {fmt(cmd.get('command_limiter_frac'))}")
    lines.append(f"- linear_limited_frac: {fmt(cmd.get('linear_limited_frac'))}")
    lines.append(f"- angular_rate_limited_frac: {fmt(cmd.get('angular_rate_limited_frac'))}")
    lines.append(f"- angular_accel_limited_frac: {fmt(cmd.get('angular_accel_limited_frac'))}")
    lines.append(f"- |published_v - post_gate_v| p95: {fmt(cmd.get('published_minus_post_gate_v_abs', {}).get('p95'))}")
    lines.append(f"- |published_omega - post_gate_omega| p95: {fmt(cmd.get('published_minus_post_gate_omega_abs', {}).get('p95'))}")
    lines.append(f"- zero reasons: `{cmd.get('zero_reason_counts', {})}`")
    lines.append("")
    lines.append("## Status counts")
    for status, count in sorted(summary.get("observed", {}).get("status_counts", {}).items(), key=lambda kv: (-kv[1], kv[0])):
        lines.append(f"- `{status}`: {count}")
    lines.append("")
    return "\n".join(lines)


def write_summary(summary, out_dir):
    bag_path = Path(summary["bag"])
    target_dir = Path(out_dir).expanduser() if out_dir else bag_path.parent
    target_dir.mkdir(parents=True, exist_ok=True)
    stem = bag_path.stem
    json_path = target_dir / f"{stem}_summary.json"
    md_path = target_dir / f"{stem}_summary.md"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    md_path.write_text(render_markdown(summary), encoding="utf-8")
    return json_path, md_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", help="bag file(s) or run directory/directories")
    parser.add_argument("--out-dir", default=None, help="directory for *_summary.json/md; default: next to each bag")
    args = parser.parse_args()

    bags = find_bags(args.paths)
    if not bags:
        print("[ERR] no .bag files found", file=sys.stderr)
        return 2

    exit_code = 0
    for bag in bags:
        try:
            summary = summarize_bag(bag)
            json_path, md_path = write_summary(summary, args.out_dir)
            print(f"[OK] {bag} -> {json_path}, {md_path}")
        except Exception as exc:
            print(f"[ERR] failed to summarize {bag}: {exc}", file=sys.stderr)
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
