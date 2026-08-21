#!/usr/bin/env python3
"""Read-only monitor/enforce rosbag acceptance for Phase-Rejoining.

The comparison window is defined independently for each bag:

1. the first non-stationary command that the control-cycle audit marks as a
   successful, accepted and actually published solve; then
2. the first ``GOAL_REACHED``/``GOAL_REACHED_LATCHED`` status plus exactly
   two seconds of settling data.

``/slosh/height`` is converted from metres to millimetres while
``/spmpc/slosh_height`` is already published in millimetres.  Both signals
are reduced to ``abs(value - initial_offset)`` before RMS/P95/peak are
computed.  Missing evidence is an error; the script never opens a bag for
writing and never publishes a ROS message.
"""

import argparse
import json
import math
import statistics
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

try:
    import rosbag  # type: ignore
except ImportError:  # pragma: no cover - exercised only outside ROS
    rosbag = None


EXTERNAL_HEIGHT_TOPIC = "/slosh/height"
INTERNAL_HEIGHT_TOPIC = "/spmpc/slosh_height"
STATUS_TOPIC = "/spmpc/status"
AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
PHASE_TOPIC = "/spmpc/debug/phase_rejoin"
PROJECTOR_TOPIC = "/spmpc/debug/projector"
DEFAULT_COMMAND_TOPIC = "/cmd_vel_drive"

TAIL_AFTER_GOAL_SEC = 2.0
OFFSET_LOOKBACK_SEC = 1.0
COMMAND_NONSTATIONARY_EPS = 1.0e-4
COMMAND_MATCH_TIME_TOLERANCE_SEC = 0.10
COMMAND_MATCH_LINEAR_TOLERANCE = 1.0e-4
COMMAND_MATCH_ANGULAR_TOLERANCE = 1.0e-4
TOPIC_COVERAGE_TOLERANCE_SEC = 0.20

AUDIT_FIELDS = (
    "cycle_id", "status", "solver_status", "solve_attempted",
    "solve_success", "command_accepted", "publish_cmd_vel",
    "command_was_published", "command_contract_violation",
    "safety_gate_intervened", "linear_limited", "angular_rate_limited",
    "angular_accel_limited", "command_publish_stamp",
    "published_cmd_v", "published_cmd_omega",
)

PHASE_FIELDS = (
    "cycle_id", "mode_name", "artifact_loaded", "contract_valid", "ready",
    "candidate_count", "phase_lead_steps", "front_steps", "liquid_steps",
    "solver_terminal_step", "terminal_gate_accepted",
    "command_contract_consistent", "recovery_command_used",
    "controlled_stop_used", "prediction_valid", "status",
)

HARD_STATUS_MARKERS = (
    "FAIL", "INVALID", "VIOLATION", "INFEASIBLE", "UNSAFE", "ERROR",
    "NOT_READY", "UNAVAILABLE", "MISSING", "TIMEOUT", "CONTROLLED_STOP",
)


class AnalysisError(RuntimeError):
    """The evidence contract is incomplete or internally inconsistent."""


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def percentile(values, quantile):
    """Linear-interpolated percentile; pure and NumPy-independent."""
    data = sorted(float(value) for value in values if finite(value))
    if not data:
        raise AnalysisError("cannot compute a percentile from no finite samples")
    if len(data) == 1:
        return data[0]
    position = (len(data) - 1) * float(quantile)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return data[lower]
    ratio = position - lower
    return data[lower] * (1.0 - ratio) + data[upper] * ratio


def is_goal_status(status):
    value = str(status).strip().upper()
    return value == "GOAL_REACHED" or value.startswith("GOAL_REACHED_")


def is_nonstationary(linear, angular, epsilon=COMMAND_NONSTATIONARY_EPS):
    return max(abs(float(linear)), abs(float(angular))) > float(epsilon)


def audit_event_time(record):
    """Return the command-effective audit time, or receive time if unpublished."""
    published = record.get("command_publish_t")
    if finite(published) and float(published) > 0.0:
        return float(published)
    return float(record["t"])


def count_strings(records, key="value"):
    return dict(sorted(Counter(str(record[key]) for record in records).items()))


def filter_records(records, start, end):
    return [record for record in records if start <= float(record["t"]) <= end]


def signal_metrics(series, start, end, scale_to_mm, native_unit,
                   offset_lookback_sec=OFFSET_LOOKBACK_SEC):
    """Return abs(offset-corrected) sample metrics in millimetres.

    The initial offset is the median of the last second strictly before the
    active command.  Some controller-owned topics do not exist before the
    first solve; in that case the first in-window sample is the explicit,
    reported fallback instead of silently assuming zero.
    """
    invalid = [(stamp, value) for stamp, value in series
               if not finite(stamp) or not finite(value)]
    if invalid:
        raise AnalysisError("height series contains non-finite samples")

    ordered = sorted((float(stamp), float(value)) for stamp, value in series)
    window = [(stamp, value) for stamp, value in ordered
              if start <= stamp <= end]
    if not window:
        raise AnalysisError("height series has no samples in the acceptance window")

    pre_window = [value for stamp, value in ordered
                  if start - offset_lookback_sec <= stamp < start]
    if pre_window:
        offset = float(statistics.median(pre_window))
        offset_source = "pre_window_median"
        offset_samples = len(pre_window)
    else:
        offset = window[0][1]
        offset_source = "first_window_sample"
        offset_samples = 1

    magnitudes = [abs((value - offset) * scale_to_mm)
                  for _, value in window]
    rms = math.sqrt(sum(value * value for value in magnitudes) / len(magnitudes))
    return {
        "definition": "abs(value - initial_offset)",
        "native_unit": native_unit,
        "comparison_unit": "mm",
        "scale_to_mm": float(scale_to_mm),
        "samples": len(magnitudes),
        "first_sample_rel_sec": window[0][0] - start,
        "last_sample_rel_sec": window[-1][0] - start,
        "initial_offset_native": offset,
        "initial_offset_mm": offset * scale_to_mm,
        "initial_offset_source": offset_source,
        "initial_offset_samples": offset_samples,
        "rms_mm": rms,
        "p95_mm": percentile(magnitudes, 0.95),
        "peak_mm": max(magnitudes),
    }


def _absolute_metrics(values, unit):
    magnitudes = [abs(float(value)) for value in values if finite(value)]
    if not magnitudes:
        raise AnalysisError("cannot summarize an empty finite motion series")
    return {
        "definition": "absolute magnitude",
        "unit": unit,
        "samples": len(magnitudes),
        "mean": statistics.mean(magnitudes),
        "rms": math.sqrt(
            sum(value * value for value in magnitudes) / len(magnitudes)),
        "p95": percentile(magnitudes, 0.95),
        "peak": max(magnitudes),
    }


def command_motion_metrics(commands, start, goal):
    """Summarize actual command speed and finite-difference acceleration.

    Speed samples use the active-motion interval from the first accepted,
    published moving command through the first goal status.  Acceleration uses
    command-topic receive timestamps and includes the latest pre-window sample
    as an anchor when available, so the initial launch transition is retained.
    These are descriptive fairness metrics and do not change the slosh
    acceptance criterion.
    """
    ordered = sorted(
        ({"t": float(record["t"]),
          "v": float(record["v"]),
          "omega": float(record["omega"])} for record in commands
         if finite(record.get("t")) and finite(record.get("v")) and
         finite(record.get("omega"))),
        key=lambda record: record["t"])
    active = [record for record in ordered
              if float(start) <= record["t"] <= float(goal)]
    if len(active) < 2:
        raise AnalysisError(
            "actual command topic has fewer than two motion-window samples")

    before = [record for record in ordered if record["t"] < float(start)]
    anchor = before[-1] if (
        before and float(start) - before[-1]["t"] <=
        TOPIC_COVERAGE_TOLERANCE_SEC) else None
    derivative_records = ([anchor] if anchor is not None else []) + active
    linear_accel = []
    angular_accel = []
    skipped_nonpositive_dt = 0
    for previous, current in zip(
            derivative_records[:-1], derivative_records[1:]):
        dt = current["t"] - previous["t"]
        if dt <= 1.0e-9:
            skipped_nonpositive_dt += 1
            continue
        linear_accel.append((current["v"] - previous["v"]) / dt)
        angular_accel.append(
            (current["omega"] - previous["omega"]) / dt)
    if not linear_accel:
        raise AnalysisError(
            "actual command topic has no positive-dt acceleration samples")

    return {
        "window_definition": "first accepted moving command through first goal",
        "start": float(start),
        "goal": float(goal),
        "duration_sec": float(goal) - float(start),
        "command_samples": len(active),
        "derivative_samples": len(linear_accel),
        "pre_window_anchor_used": anchor is not None,
        "skipped_nonpositive_dt": skipped_nonpositive_dt,
        "linear_speed_abs": _absolute_metrics(
            [record["v"] for record in active], "m/s"),
        "angular_speed_abs": _absolute_metrics(
            [record["omega"] for record in active], "rad/s"),
        "linear_accel_abs": _absolute_metrics(linear_accel, "m/s^2"),
        "angular_accel_abs": _absolute_metrics(
            angular_accel, "rad/s^2"),
    }


def find_active_window(audits, statuses, commands, bag_end,
                       tail_after_goal_sec=TAIL_AFTER_GOAL_SEC):
    """Find the command/goal window from synthetic or bag-derived records."""
    candidates = []
    for audit in sorted(audits, key=audit_event_time):
        if not (
                audit.get("solve_attempted") and
                audit.get("solve_success") and
                audit.get("command_accepted") and
                audit.get("publish_cmd_vel") and
                audit.get("command_was_published")):
            continue
        if not is_nonstationary(
                audit.get("published_cmd_v", 0.0),
                audit.get("published_cmd_omega", 0.0)):
            continue
        candidates.append(audit)
    if not candidates:
        raise AnalysisError(
            "no successful, accepted, published non-stationary audit command")

    first = candidates[0]
    start = float(first.get("command_publish_t", 0.0))
    if not finite(start) or start <= 0.0:
        raise AnalysisError(
            "the first audited moving command has no valid command_publish_stamp")
    nearby_commands = [command for command in commands
                       if abs(float(command["t"]) - start) <=
                       COMMAND_MATCH_TIME_TOLERANCE_SEC and
                       is_nonstationary(command["v"], command["omega"])]
    if not nearby_commands:
        raise AnalysisError(
            "the first audited moving command has no matching command-topic sample")
    command = min(nearby_commands, key=lambda item: abs(float(item["t"]) - start))
    linear_error = abs(float(command["v"]) - float(first["published_cmd_v"]))
    angular_error = abs(float(command["omega"]) - float(first["published_cmd_omega"]))
    if (linear_error > COMMAND_MATCH_LINEAR_TOLERANCE or
            angular_error > COMMAND_MATCH_ANGULAR_TOLERANCE):
        raise AnalysisError(
            "first audited command does not match the command topic "
            "(dv={:.6g}, domega={:.6g})".format(linear_error, angular_error))

    goal_records = [record for record in sorted(
        statuses, key=lambda item: float(item["t"]))
        if float(record["t"]) >= start and is_goal_status(record["value"])]
    if not goal_records:
        raise AnalysisError("no GOAL_REACHED status after motion start")
    goal = float(goal_records[0]["t"])
    end = goal + float(tail_after_goal_sec)
    if float(bag_end) + 1.0e-9 < end:
        raise AnalysisError(
            "bag ends {:.3f}s before the required GOAL_REACHED + {:.1f}s tail".format(
                end - float(bag_end), tail_after_goal_sec))

    return {
        "start": start,
        "goal": goal,
        "end": end,
        "completion_time_sec": goal - start,
        "window_duration_sec": end - start,
        "tail_after_goal_sec": float(tail_after_goal_sec),
        "start_cycle_id": int(first["cycle_id"]),
        "start_status": str(first["status"]),
        "start_command_v": float(first["published_cmd_v"]),
        "start_command_omega": float(first["published_cmd_omega"]),
        "command_topic_match_dt_sec": float(command["t"]) - start,
        "command_topic_match_linear_error": linear_error,
        "command_topic_match_angular_error": angular_error,
        "goal_status": str(goal_records[0]["value"]),
    }


def coverage_check(records, start, end,
                   tolerance_sec=TOPIC_COVERAGE_TOLERANCE_SEC):
    """Check that a required stream spans the whole acceptance window."""
    timestamps = sorted(float(record["t"]) if isinstance(record, dict)
                        else float(record[0]) for record in records)
    in_window = [stamp for stamp in timestamps if start <= stamp <= end]
    if not in_window:
        return {"ok": False, "samples": 0, "reason": "no in-window samples"}
    first_gap = max(0.0, in_window[0] - start)
    last_gap = max(0.0, end - in_window[-1])
    ok = first_gap <= tolerance_sec and last_gap <= tolerance_sec
    return {
        "ok": ok,
        "samples": len(in_window),
        "first_gap_sec": first_gap,
        "last_gap_sec": last_gap,
        "reason": "ok" if ok else "stream does not span the full window",
    }


def hard_status(status):
    upper = str(status).strip().upper()
    return any(marker in upper for marker in HARD_STATUS_MARKERS)


def summarize_phase_contract(audits, phase_records, start, goal,
                             expected_mode, expected_liquid_steps=3,
                             phase_lead_min=-1, phase_lead_max=1):
    """Summarize typed phase/audit evidence; no ROS dependency.

    The audit stream owns the control-window boundary.  Phase diagnostics and
    audit diagnostics are published separately within one control cycle, so
    their bag receive timestamps can straddle ``start`` by a few milliseconds.
    Select active cycles from the audit stream first, then join phase evidence
    by the controller-assigned ``cycle_id`` instead of comparing cross-topic
    receive timestamps.
    """
    active_audits = [record for record in audits
                     if start <= audit_event_time(record) < goal]
    active_cycle_ids = {int(record["cycle_id"]) for record in active_audits}
    active_phase = [record for record in phase_records
                    if int(record["cycle_id"]) in active_cycle_ids]
    phase_by_cycle = {}
    duplicate_phase_cycle_ids = []
    for record in active_phase:
        cycle_id = int(record["cycle_id"])
        if cycle_id in phase_by_cycle:
            duplicate_phase_cycle_ids.append(cycle_id)
        phase_by_cycle[cycle_id] = record

    successful_audits = [record for record in active_audits
                         if record.get("solve_attempted") and
                         record.get("solve_success") and
                         record.get("command_accepted") and
                         record.get("publish_cmd_vel") and
                         record.get("command_was_published") and
                         not is_goal_status(record.get("status", ""))]
    joined = []
    missing_cycles = []
    for audit in successful_audits:
        cycle_id = int(audit["cycle_id"])
        phase = phase_by_cycle.get(cycle_id)
        if phase is None:
            missing_cycles.append(cycle_id)
        else:
            joined.append(phase)

    ready = [record for record in active_phase if record.get("ready")]
    horizons = [int(record["liquid_steps"]) for record in ready]
    terminal_steps = [int(record["solver_terminal_step"]) for record in ready]
    leads = [int(record["phase_lead_steps"]) for record in ready]
    horizon_violations = [int(record["cycle_id"]) for record in ready
                          if int(record["liquid_steps"]) != expected_liquid_steps]
    lead_violations = [int(record["cycle_id"]) for record in ready
                       if not phase_lead_min <= int(record["phase_lead_steps"]) <= phase_lead_max]

    # The command contract is meaningful on an accepted solver/terminal-gate
    # cycle.  Recovery cycles intentionally use a saved policy and are counted
    # separately instead of being mislabeled as solver-contract successes.
    contract_applicable = [record for record in joined
                           if record.get("terminal_gate_accepted") or
                           str(record.get("status")) == "ENFORCE_TERMINAL_ACCEPTED"]
    inconsistent_cycles = [int(record["cycle_id"]) for record in contract_applicable
                           if not record.get("command_contract_consistent")]

    mode_counts = count_strings(active_phase, "mode_name") if active_phase else {}
    modes_valid = bool(active_phase) and set(mode_counts) == {expected_mode}
    return {
        "active_records": len(active_phase),
        "mode_counts": mode_counts,
        "expected_mode": expected_mode,
        "mode_valid": modes_valid,
        "ready_records": len(ready),
        "ready_fraction": len(ready) / len(active_phase) if active_phase else None,
        "artifact_loaded_fraction": (
            sum(bool(record.get("artifact_loaded")) for record in active_phase) /
            len(active_phase) if active_phase else None),
        "contract_valid_fraction": (
            sum(bool(record.get("contract_valid")) for record in active_phase) /
            len(active_phase) if active_phase else None),
        "prediction_valid_fraction": (
            sum(bool(record.get("prediction_valid")) for record in active_phase) /
            len(active_phase) if active_phase else None),
        "liquid_steps": {
            "expected": int(expected_liquid_steps),
            "unique": sorted(set(horizons)),
            "min": min(horizons) if horizons else None,
            "max": max(horizons) if horizons else None,
            "violation_count": len(horizon_violations),
            "violation_cycle_ids": horizon_violations[:50],
        },
        "solver_terminal_step": {
            "unique": sorted(set(terminal_steps)),
            "min": min(terminal_steps) if terminal_steps else None,
            "max": max(terminal_steps) if terminal_steps else None,
        },
        "phase_lead_steps": {
            "allowed_min": int(phase_lead_min),
            "allowed_max": int(phase_lead_max),
            "unique": sorted(set(leads)),
            "min": min(leads) if leads else None,
            "max": max(leads) if leads else None,
            "violation_count": len(lead_violations),
            "violation_cycle_ids": lead_violations[:50],
        },
        "successful_audit_cycles": len(successful_audits),
        "joined_successful_cycles": len(joined),
        "missing_phase_cycle_count": len(missing_cycles),
        "missing_phase_cycle_ids": missing_cycles[:50],
        "duplicate_phase_cycle_count": len(duplicate_phase_cycle_ids),
        "duplicate_phase_cycle_ids": duplicate_phase_cycle_ids[:50],
        "command_contract": {
            "applicable_cycles": len(contract_applicable),
            "consistent_cycles": len(contract_applicable) - len(inconsistent_cycles),
            "inconsistent_count": len(inconsistent_cycles),
            "inconsistent_cycle_ids": inconsistent_cycles[:50],
        },
        "recovery_command_count": sum(
            bool(record.get("recovery_command_used")) for record in active_phase),
        "controlled_stop_count": sum(
            bool(record.get("controlled_stop_used")) for record in active_phase),
        "status_counts": count_strings(active_phase, "status") if active_phase else {},
    }


def _require_fields(message, fields, topic):
    missing = [field for field in fields if not hasattr(message, field)]
    if missing:
        raise AnalysisError(
            "{} message is missing required fields: {}".format(
                topic, ", ".join(missing)))


def _projector_distance(message):
    data = list(getattr(message, "data", []))
    dimensions = list(getattr(getattr(message, "layout", None), "dim", []))
    if not data or not dimensions:
        return None
    labels = [part.strip() for part in str(dimensions[0].label).split(",")]
    fields = dict(zip(labels, data))
    guarded_valid = fields.get("guarded_valid", 0.0) > 0.5
    raw_valid = fields.get("raw_valid", 0.0) > 0.5
    if guarded_valid and finite(fields.get("guarded_distance")):
        return abs(float(fields["guarded_distance"]))
    if raw_valid and finite(fields.get("raw_distance")):
        return abs(float(fields["raw_distance"]))
    return None


def read_bag(path, command_topic=DEFAULT_COMMAND_TOPIC):
    if rosbag is None:
        raise AnalysisError(
            "Python rosbag is unavailable; source ROS Noetic and the workspace")
    bag_path = Path(path).expanduser().resolve()
    if not bag_path.is_file() or bag_path.suffix != ".bag":
        raise AnalysisError("not a readable .bag file: {}".format(bag_path))

    required_topics = {
        EXTERNAL_HEIGHT_TOPIC, INTERNAL_HEIGHT_TOPIC, STATUS_TOPIC,
        AUDIT_TOPIC, PHASE_TOPIC, command_topic,
    }
    selected_topics = sorted(required_topics | {PROJECTOR_TOPIC})
    raw = {
        "bag": str(bag_path),
        "external_height": [],
        "internal_height": [],
        "statuses": [],
        "audits": [],
        "phase": [],
        "commands": [],
        "projector": [],
        "audit_projection": [],
    }

    with rosbag.Bag(str(bag_path), "r") as bag:
        topic_info = bag.get_type_and_topic_info()[1]
        missing = sorted(required_topics - set(topic_info))
        if missing:
            raise AnalysisError(
                "{} missing required topic(s): {}".format(
                    bag_path.name, ", ".join(missing)))
        raw["bag_start"] = float(bag.get_start_time())
        raw["bag_end"] = float(bag.get_end_time())
        raw["topic_types"] = {
            topic: topic_info[topic].msg_type
            for topic in selected_topics if topic in topic_info
        }
        for topic, message, stamp in bag.read_messages(topics=selected_topics):
            timestamp = float(stamp.to_sec())
            if topic == EXTERNAL_HEIGHT_TOPIC:
                if not hasattr(message, "data"):
                    raise AnalysisError("{} has no data field".format(topic))
                raw["external_height"].append((timestamp, float(message.data)))
            elif topic == INTERNAL_HEIGHT_TOPIC:
                if not hasattr(message, "data"):
                    raise AnalysisError("{} has no data field".format(topic))
                raw["internal_height"].append((timestamp, float(message.data)))
            elif topic == STATUS_TOPIC:
                if not hasattr(message, "data"):
                    raise AnalysisError("{} has no data field".format(topic))
                raw["statuses"].append({"t": timestamp, "value": str(message.data)})
            elif topic == command_topic:
                try:
                    linear = float(message.linear.x)
                    angular = float(message.angular.z)
                except AttributeError as exc:
                    raise AnalysisError(
                        "{} is not a geometry_msgs/Twist-like message".format(
                            command_topic)) from exc
                raw["commands"].append(
                    {"t": timestamp, "v": linear, "omega": angular})
            elif topic == AUDIT_TOPIC:
                _require_fields(message, AUDIT_FIELDS, topic)
                raw["audits"].append({
                    "t": timestamp,
                    "command_publish_t": float(
                        message.command_publish_stamp.to_sec()),
                    "cycle_id": int(message.cycle_id),
                    "status": str(message.status),
                    "solver_status": str(message.solver_status),
                    "solve_attempted": bool(message.solve_attempted),
                    "solve_success": bool(message.solve_success),
                    "command_accepted": bool(message.command_accepted),
                    "publish_cmd_vel": bool(message.publish_cmd_vel),
                    "command_was_published": bool(message.command_was_published),
                    "command_contract_violation": bool(message.command_contract_violation),
                    "safety_gate_intervened": bool(message.safety_gate_intervened),
                    "linear_limited": bool(message.linear_limited),
                    "angular_rate_limited": bool(message.angular_rate_limited),
                    "angular_accel_limited": bool(message.angular_accel_limited),
                    "published_cmd_v": float(message.published_cmd_v),
                    "published_cmd_omega": float(message.published_cmd_omega),
                })
                for field in (
                        "path_projection_distance", "projection_distance",
                        "projector_distance", "tracking_projection_distance"):
                    if hasattr(message, field) and finite(getattr(message, field)):
                        raw["audit_projection"].append({
                            "t": timestamp,
                            "distance_m": abs(float(getattr(message, field))),
                            "source": AUDIT_TOPIC + "." + field,
                        })
                        break
            elif topic == PHASE_TOPIC:
                _require_fields(message, PHASE_FIELDS, topic)
                raw["phase"].append({
                    "t": timestamp,
                    "cycle_id": int(message.cycle_id),
                    "mode_name": str(message.mode_name),
                    "artifact_loaded": bool(message.artifact_loaded),
                    "contract_valid": bool(message.contract_valid),
                    "ready": bool(message.ready),
                    "candidate_count": int(message.candidate_count),
                    "phase_lead_steps": int(message.phase_lead_steps),
                    "front_steps": int(message.front_steps),
                    "liquid_steps": int(message.liquid_steps),
                    "solver_terminal_step": int(message.solver_terminal_step),
                    "terminal_gate_accepted": bool(message.terminal_gate_accepted),
                    "command_contract_consistent": bool(
                        message.command_contract_consistent),
                    "recovery_command_used": bool(message.recovery_command_used),
                    "controlled_stop_used": bool(message.controlled_stop_used),
                    "prediction_valid": bool(message.prediction_valid),
                    "status": str(message.status),
                })
            elif topic == PROJECTOR_TOPIC:
                distance = _projector_distance(message)
                if distance is not None:
                    raw["projector"].append({"t": timestamp, "distance_m": distance})

    raw["topic_counts"] = {
        EXTERNAL_HEIGHT_TOPIC: len(raw["external_height"]),
        INTERNAL_HEIGHT_TOPIC: len(raw["internal_height"]),
        STATUS_TOPIC: len(raw["statuses"]),
        AUDIT_TOPIC: len(raw["audits"]),
        PHASE_TOPIC: len(raw["phase"]),
        command_topic: len(raw["commands"]),
        PROJECTOR_TOPIC: len(raw["projector"]),
    }
    return raw


def _anomalous_counts(records, key):
    return dict(sorted(Counter(str(record[key]) for record in records
                               if hard_status(record[key])).items()))


def analyze_run(raw, role, command_topic=DEFAULT_COMMAND_TOPIC,
                expected_liquid_steps=3, phase_lead_min=-1,
                phase_lead_max=1):
    expected_mode = "monitor" if role == "baseline" else "enforce"
    window = find_active_window(
        raw["audits"], raw["statuses"], raw["commands"], raw["bag_end"])
    start, goal, end = window["start"], window["goal"], window["end"]

    stream_records = {
        EXTERNAL_HEIGHT_TOPIC: raw["external_height"],
        INTERNAL_HEIGHT_TOPIC: raw["internal_height"],
        STATUS_TOPIC: raw["statuses"],
        AUDIT_TOPIC: raw["audits"],
        PHASE_TOPIC: raw["phase"],
        command_topic: raw["commands"],
    }
    coverage = {
        topic: coverage_check(records, start, end)
        for topic, records in stream_records.items()
    }
    coverage_failures = [topic for topic, check in coverage.items()
                         if not check["ok"]]

    external = signal_metrics(
        raw["external_height"], start, end, 1000.0, "m")
    internal = signal_metrics(
        raw["internal_height"], start, end, 1.0, "mm")
    motion = command_motion_metrics(raw["commands"], start, goal)

    statuses = filter_records(raw["statuses"], start, end)
    audits = [record for record in raw["audits"]
              if start <= audit_event_time(record) <= end]
    phase = filter_records(raw["phase"], start, end)
    phase_before_goal = [record for record in phase if float(record["t"]) < goal]
    audits_before_goal = [record for record in audits
                          if audit_event_time(record) < goal]
    phase_contract = summarize_phase_contract(
        raw["audits"], raw["phase"], start, goal, expected_mode,
        expected_liquid_steps, phase_lead_min, phase_lead_max)

    planner_anomalies = _anomalous_counts(statuses, "value")
    solver_anomalies = _anomalous_counts(audits, "solver_status")
    phase_anomalies = _anomalous_counts(phase_before_goal, "status")
    pre_goal_bypass = sum(
        str(record["status"]) == "BYPASSED_TERMINAL_PRIORITY"
        for record in phase_before_goal)
    post_goal_bypass = sum(
        str(record["status"]) == "BYPASSED_TERMINAL_PRIORITY" and
        float(record["t"]) >= goal for record in phase)
    audit_contract_violations = sum(
        bool(record["command_contract_violation"])
        for record in audits_before_goal)
    safety_interventions = sum(
        bool(record["safety_gate_intervened"])
        for record in audits_before_goal)
    limiter_interventions = sum(
        bool(record["linear_limited"] or record["angular_rate_limited"] or
             record["angular_accel_limited"])
        for record in audits_before_goal)

    projection_records = [
        dict(record, source=PROJECTOR_TOPIC) for record in raw["projector"]
    ] + list(raw.get("audit_projection", []))
    projection_records = [record for record in projection_records
                          if start <= float(record["t"]) <= end and
                          finite(record["distance_m"])]
    projection = [record["distance_m"] for record in projection_records]

    evidence_failures = []
    contract_failures = []
    if coverage_failures:
        evidence_failures.append("required topic coverage incomplete: " +
                                 ", ".join(coverage_failures))
    if not phase_contract["mode_valid"]:
        evidence_failures.append(
            "phase mode mismatch: expected {}, observed {}".format(
                expected_mode, phase_contract["mode_counts"]))
    if phase_contract["ready_records"] == 0:
        evidence_failures.append("no ready phase-rejoin records before GOAL_REACHED")
    if phase_contract["active_records"] and (
            phase_contract["artifact_loaded_fraction"] != 1.0 or
            phase_contract["contract_valid_fraction"] != 1.0):
        evidence_failures.append(
            "phase artifact/contract was not valid on every active cycle")
    if planner_anomalies:
        contract_failures.append("planner anomalous statuses: {}".format(planner_anomalies))
    if solver_anomalies:
        contract_failures.append("solver anomalous statuses: {}".format(solver_anomalies))
    if phase_anomalies:
        contract_failures.append("phase anomalous statuses before goal: {}".format(
            phase_anomalies))
    if pre_goal_bypass:
        contract_failures.append(
            "BYPASSED_TERMINAL_PRIORITY occurred {} time(s) before goal".format(
                pre_goal_bypass))
    if audit_contract_violations:
        contract_failures.append("audit command contract violations: {}".format(
            audit_contract_violations))
    if safety_interventions:
        contract_failures.append("safety gate interventions before goal: {}".format(
            safety_interventions))
    if limiter_interventions:
        contract_failures.append("post-solver limiter interventions before goal: {}".format(
            limiter_interventions))
    if phase_contract["duplicate_phase_cycle_count"]:
        evidence_failures.append("duplicate phase cycle IDs: {}".format(
            phase_contract["duplicate_phase_cycle_count"]))
    if phase_contract["missing_phase_cycle_count"]:
        evidence_failures.append("successful audit cycles missing phase records: {}".format(
            phase_contract["missing_phase_cycle_count"]))

    if role == "enforce":
        liquid = phase_contract["liquid_steps"]
        lead = phase_contract["phase_lead_steps"]
        contract = phase_contract["command_contract"]
        if phase_contract["ready_records"] != phase_contract["active_records"]:
            contract_failures.append("phase-rejoin was not ready on every enforce cycle")
        if phase_contract["prediction_valid_fraction"] != 1.0:
            contract_failures.append("execution-front prediction was invalid on one or more cycles")
        if liquid["violation_count"]:
            contract_failures.append("liquid horizon is not {} on {} ready cycle(s)".format(
                expected_liquid_steps, liquid["violation_count"]))
        if lead["violation_count"]:
            contract_failures.append("phase lead outside [{}, {}] on {} cycle(s)".format(
                phase_lead_min, phase_lead_max, lead["violation_count"]))
        if contract["applicable_cycles"] == 0:
            contract_failures.append("no accepted enforce cycles for command-contract audit")
        if contract["inconsistent_count"]:
            contract_failures.append("command_contract_consistent=false on {} accepted cycle(s)".format(
                contract["inconsistent_count"]))
        if phase_contract["controlled_stop_count"]:
            contract_failures.append("controlled stop used on {} cycle(s)".format(
                phase_contract["controlled_stop_count"]))

    failures = evidence_failures + contract_failures

    return {
        "role": role,
        "bag": raw["bag"],
        "bag_start": raw["bag_start"],
        "bag_end": raw["bag_end"],
        "topic_counts": raw["topic_counts"],
        "window": window,
        "topic_window_coverage": coverage,
        "height_metrics": {
            "external_slosh_height": external,
            "internal_spmpc_slosh_height": internal,
        },
        "motion_metrics": motion,
        "path_projection_distance": {
            "available": bool(projection),
            "sources": sorted(set(record["source"] for record in projection_records)),
            "samples": len(projection),
            "max_m": max(projection) if projection else None,
        },
        "status_counts": {
            "planner": count_strings(statuses, "value"),
            "solver": count_strings(audits, "solver_status"),
            "phase": count_strings(phase, "status"),
        },
        "anomalous_status_counts": {
            "planner": planner_anomalies,
            "solver": solver_anomalies,
            "phase_before_goal": phase_anomalies,
        },
        "phase_contract": phase_contract,
        "interventions": {
            "pre_goal_terminal_priority_bypass": pre_goal_bypass,
            "post_goal_terminal_priority_bypass": post_goal_bypass,
            "audit_command_contract_violations": audit_contract_violations,
            "safety_gate_interventions": safety_interventions,
            "post_solver_limiter_interventions": limiter_interventions,
        },
        "evidence_valid": not evidence_failures,
        "run_contract_valid": not contract_failures,
        "evidence_failures": evidence_failures,
        "contract_failures": contract_failures,
        "failures": failures,
    }


def _metric_comparison(baseline, enforce):
    delta = float(enforce) - float(baseline)
    relative = None
    if abs(float(baseline)) > 1.0e-12:
        relative = 100.0 * delta / float(baseline)
    return {
        "baseline_mm": float(baseline),
        "enforce_mm": float(enforce),
        "delta_mm": delta,
        "relative_change_percent": relative,
        "improved": float(enforce) < float(baseline),
    }


def _descriptive_comparison(baseline, enforce, unit):
    delta = float(enforce) - float(baseline)
    relative = None
    if abs(float(baseline)) > 1.0e-12:
        relative = 100.0 * delta / float(baseline)
    return {
        "unit": unit,
        "baseline": float(baseline),
        "enforce": float(enforce),
        "delta": delta,
        "relative_change_percent": relative,
    }


def compare_runs(baseline, enforce):
    """Build the strict two-signal verdict from two analyzed runs."""
    comparisons = {}
    for signal in ("external_slosh_height", "internal_spmpc_slosh_height"):
        comparisons[signal] = {}
        for metric in ("rms_mm", "p95_mm", "peak_mm"):
            comparisons[signal][metric] = _metric_comparison(
                baseline["height_metrics"][signal][metric],
                enforce["height_metrics"][signal][metric])

    external_all = all(item["improved"] for item in
                       comparisons["external_slosh_height"].values())
    internal_all = all(item["improved"] for item in
                       comparisons["internal_spmpc_slosh_height"].values())
    comparisons["external_slosh_height"]["all_metrics_improved"] = external_all
    comparisons["external_slosh_height"]["direction"] = (
        "POSITIVE" if external_all else "NON_POSITIVE")
    comparisons["internal_spmpc_slosh_height"]["all_metrics_improved"] = internal_all
    comparisons["internal_spmpc_slosh_height"]["direction"] = (
        "POSITIVE" if internal_all else "NON_POSITIVE")
    motion_comparisons = {}
    for quantity in (
            "linear_speed_abs", "angular_speed_abs",
            "linear_accel_abs", "angular_accel_abs"):
        motion_comparisons[quantity] = {}
        unit = baseline["motion_metrics"][quantity]["unit"]
        for metric in ("rms", "p95", "peak"):
            motion_comparisons[quantity][metric] = _descriptive_comparison(
                baseline["motion_metrics"][quantity][metric],
                enforce["motion_metrics"][quantity][metric], unit)
    checks = {
        "baseline_evidence_valid": bool(baseline.get("evidence_valid")),
        "enforce_evidence_valid": bool(enforce.get("evidence_valid")),
        "enforce_run_contract_valid": bool(
            enforce.get("run_contract_valid", enforce.get("evidence_valid"))),
        "external_rms_p95_peak_all_improved": external_all,
        "internal_rms_p95_peak_all_improved": internal_all,
    }
    failures = []
    if not checks["baseline_evidence_valid"]:
        failures.append("baseline evidence invalid")
    if not checks["enforce_evidence_valid"]:
        failures.append("enforce evidence invalid")
    if not checks["enforce_run_contract_valid"]:
        failures.append("enforce execution/phase contract invalid")
    if not external_all:
        failures.append("external /slosh/height did not improve on RMS, P95 and peak")
    if not internal_all:
        failures.append("internal /spmpc/slosh_height did not improve on RMS, P95 and peak")

    return {
        "height": comparisons,
        "motion": motion_comparisons,
        "completion_time": {
            "baseline_sec": baseline["window"]["completion_time_sec"],
            "enforce_sec": enforce["window"]["completion_time_sec"],
            "delta_sec": (enforce["window"]["completion_time_sec"] -
                          baseline["window"]["completion_time_sec"]),
        },
        "path_projection_max": {
            "baseline_m": baseline["path_projection_distance"]["max_m"],
            "enforce_m": enforce["path_projection_distance"]["max_m"],
        },
        "acceptance": {
            "criterion": (
                "fail closed; valid paired evidence, valid enforce execution/phase "
                "contract, and enforce lower than baseline for RMS/P95/peak on both "
                "height channels"),
            "checks": checks,
            "pass": all(checks.values()),
            "result": "POSITIVE" if all(checks.values()) else "NON_POSITIVE",
            "failures": failures,
        },
    }


def clean_json(value):
    if isinstance(value, dict):
        return {str(key): clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _fmt(value, digits=3):
    return "NA" if value is None or not finite(value) else (
        "{:.{}f}".format(float(value), digits))


def render_summary(report):
    if report.get("analysis_error"):
        return "Phase-Rejoining 配对验收: ERROR\n\n{}\n".format(
            report["analysis_error"])

    baseline = report["baseline"]
    enforce = report["enforce"]
    comparison = report["comparison"]
    accepted = comparison["acceptance"]["pass"]
    lines = [
        "Phase-Rejoining monitor/enforce 配对验收: {}".format(
            "PASS" if accepted else "FAIL"),
        "",
        "窗口口径: 首个 solve-success、accepted、实际发布的非静止命令，"
        "至首次 GOAL_REACHED 后 2.0 s。",
        "液面口径: abs(value - initial_offset)，统一为 mm；严格要求双通道的 "
        "RMS/P95/peak 全部下降。",
        "",
    ]
    for run in (baseline, enforce):
        window = run["window"]
        lines.append(
            "{}: completion={} s, window={} s, evidence={}, contract={}".format(
                run["role"], _fmt(window["completion_time_sec"]),
                _fmt(window["window_duration_sec"]),
                "VALID" if run["evidence_valid"] else "INVALID",
                "VALID" if run["run_contract_valid"] else "INVALID"))
        if run["failures"]:
            for failure in run["failures"]:
                lines.append("  - {}".format(failure))
    lines.append("")

    labels = (
        ("external_slosh_height", "/slosh/height (external)"),
        ("internal_spmpc_slosh_height", "/spmpc/slosh_height (internal)"),
    )
    for key, label in labels:
        lines.append("{}: {}".format(
            label, comparison["height"][key]["direction"]))
        for metric, metric_label in (
                ("rms_mm", "RMS"), ("p95_mm", "P95"),
                ("peak_mm", "peak")):
            item = comparison["height"][key][metric]
            direction = "改善" if item["improved"] else "未改善"
            lines.append(
                "  {}: {} -> {} mm ({}%, {})".format(
                    metric_label, _fmt(item["baseline_mm"], 5),
                    _fmt(item["enforce_mm"], 5),
                    _fmt(item["relative_change_percent"], 2), direction))
    lines.append("")

    lines.append("实际发布命令运动学（首个有效运动命令至首次到达；仅描述，不改变验收门）")
    for key, label in (
            ("linear_speed_abs", "|v|"),
            ("angular_speed_abs", "|omega|"),
            ("linear_accel_abs", "|a|"),
            ("angular_accel_abs", "|alpha|")):
        metric_items = comparison["motion"][key]
        unit = metric_items["rms"]["unit"]
        rendered = []
        for metric, metric_label in (
                ("rms", "RMS"), ("p95", "P95"), ("peak", "peak")):
            item = metric_items[metric]
            rendered.append("{} {} -> {} ({}%)".format(
                metric_label, _fmt(item["baseline"], 5),
                _fmt(item["enforce"], 5),
                _fmt(item["relative_change_percent"], 2)))
        lines.append("  {} [{}]: {}".format(label, unit, "; ".join(rendered)))
    lines.append("")

    phase = enforce["phase_contract"]
    lines.extend([
        "enforce phase evidence",
        "  mode={}, ready={}/{}".format(
            phase["mode_counts"], phase["ready_records"],
            phase["active_records"]),
        "  liquid_steps unique={}, violations={}".format(
            phase["liquid_steps"]["unique"],
            phase["liquid_steps"]["violation_count"]),
        "  phase_lead_steps min/max={}/{}, violations={}".format(
            phase["phase_lead_steps"]["min"],
            phase["phase_lead_steps"]["max"],
            phase["phase_lead_steps"]["violation_count"]),
        "  command_contract_consistent={}/{}, inconsistent={}".format(
            phase["command_contract"]["consistent_cycles"],
            phase["command_contract"]["applicable_cycles"],
            phase["command_contract"]["inconsistent_count"]),
        "  recovery={}, controlled_stop={}, pre-goal bypass={}".format(
            phase["recovery_command_count"], phase["controlled_stop_count"],
            enforce["interventions"]["pre_goal_terminal_priority_bypass"]),
        "  anomalous statuses={}".format(enforce["anomalous_status_counts"]),
        "  max projection distance={} m ({})".format(
            _fmt(enforce["path_projection_distance"]["max_m"], 5),
            "available" if enforce["path_projection_distance"]["available"]
            else "topic unavailable"),
        "",
    ])
    if comparison["acceptance"]["failures"]:
        lines.append("验收失败原因")
        for failure in comparison["acceptance"]["failures"]:
            lines.append("  - {}".format(failure))
    else:
        lines.append("双液面通道及执行合同均满足本轮正向验收。")
    return "\n".join(lines) + "\n"


def output_paths(args):
    baseline = Path(args.baseline).expanduser()
    enforce = Path(args.enforce).expanduser()
    stem = "{}_vs_{}_phase_rejoin_acceptance".format(
        baseline.stem, enforce.stem)
    json_path = (Path(args.output_json).expanduser() if args.output_json
                 else enforce.resolve().parent / (stem + ".json"))
    summary_path = (Path(args.output_summary).expanduser()
                    if args.output_summary
                    else enforce.resolve().parent / (stem + ".txt"))
    return json_path, summary_path


def write_outputs(report, summary, json_path, summary_path):
    json_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(clean_json(report), ensure_ascii=False, indent=2,
                   sort_keys=True) + "\n", encoding="utf-8")
    summary_path.write_text(summary, encoding="utf-8")


def run(args):
    json_path, summary_path = output_paths(args)
    try:
        baseline_raw = read_bag(args.baseline, args.command_topic)
        enforce_raw = read_bag(args.enforce, args.command_topic)
        baseline = analyze_run(
            baseline_raw, "baseline", args.command_topic,
            args.expected_liquid_steps, args.phase_lead_min,
            args.phase_lead_max)
        enforce = analyze_run(
            enforce_raw, "enforce", args.command_topic,
            args.expected_liquid_steps, args.phase_lead_min,
            args.phase_lead_max)
        comparison = compare_runs(baseline, enforce)
        report = {
            "schema_version": 1,
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "read_only_analysis": True,
            "baseline": baseline,
            "enforce": enforce,
            "comparison": comparison,
        }
        summary = render_summary(report)
        write_outputs(report, summary, json_path, summary_path)
        print(summary, end="")
        print("JSON: {}".format(json_path))
        print("Summary: {}".format(summary_path))
        return 0 if comparison["acceptance"]["pass"] else 1
    except (AnalysisError, OSError) as exc:
        report = {
            "schema_version": 1,
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "read_only_analysis": True,
            "analysis_error": str(exc),
            "comparison": {
                "acceptance": {
                    "pass": False,
                    "result": "ERROR",
                    "failures": [str(exc)],
                },
            },
        }
        summary = render_summary(report)
        write_outputs(report, summary, json_path, summary_path)
        print(summary, file=sys.stderr, end="")
        print("JSON: {}".format(json_path), file=sys.stderr)
        print("Summary: {}".format(summary_path), file=sys.stderr)
        return 2


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", help="monitor/baseline rosbag")
    parser.add_argument("enforce", help="phase-rejoin enforce rosbag")
    parser.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC,
                        help="actual published command topic")
    parser.add_argument("--expected-liquid-steps", type=int, default=3)
    parser.add_argument("--phase-lead-min", type=int, default=-1)
    parser.add_argument("--phase-lead-max", type=int, default=1)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--output-summary", default="")
    return parser


def main():
    args = build_parser().parse_args()
    if args.expected_liquid_steps <= 0:
        print("[ERR] --expected-liquid-steps must be positive", file=sys.stderr)
        return 2
    if args.phase_lead_min > args.phase_lead_max:
        print("[ERR] --phase-lead-min must be <= --phase-lead-max", file=sys.stderr)
        return 2
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
