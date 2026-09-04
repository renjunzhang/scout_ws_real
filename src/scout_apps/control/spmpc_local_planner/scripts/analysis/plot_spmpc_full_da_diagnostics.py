#!/usr/bin/env python3
"""Render the fixed offline diagnostic figure set for a full-Delta-a SPMPC bag.

The analyzer is deliberately offline.  Sensor samples are placed using their
effective source stamps, and the joinable solver diagnostics are associated by
``cycle_id``.  ROS bag receive times are never used as signal timestamps.

``/cmd_vel`` and the cost topics are headerless.  Their values are therefore
joined by publication order to timestamped audit/horizon records; the plotted
time always comes from the corresponding audit or horizon record.
"""

from __future__ import annotations

import argparse
import bisect
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"
HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"
COST_TOPIC = "/spmpc/cost_breakdown"
SLOSH_COST_TOPIC = "/spmpc/debug/slosh_cost_monitor"
OBSERVER_TOPIC = "/spmpc/debug/slosh_observer_imu"
ODOM_TOPIC = "/odom"
IMU_TOPIC = "/imu/data"
CMD_TOPIC = "/cmd_vel"
RGB_TOPIC = "/liquid/measurement"

READ_TOPICS = (
    AUDIT_TOPIC,
    SNAPSHOT_TOPIC,
    HORIZON_TOPIC,
    COST_TOPIC,
    SLOSH_COST_TOPIC,
    OBSERVER_TOPIC,
    ODOM_TOPIC,
    IMU_TOPIC,
    CMD_TOPIC,
    RGB_TOPIC,
)

FIGURE_NAMES = (
    "01_timing.png",
    "02_command_chain.png",
    "03_command_actual.png",
    "04_prediction.png",
    "05_liquid.png",
    "06_cost_frequency.png",
)

LEADS_SEC = (1.0 / 30.0, 0.100, 1.0 / 6.0, 4.0 / 15.0, 1.0 / 3.0)
LEAD_LABELS = ("33 ms", "100 ms", "167 ms", "267 ms", "333 ms")

COST_TERMS = (
    "J_contour",
    "J_lag",
    "J_progress",
    "J_v",
    "J_control",
    "J_smooth",
    "J_terminal",
    "J_corridor",
    "J_obstacle",
    "J_slosh_eta",
    "J_slosh_eta_dot",
)

# Okabe-Ito first, followed by restrained extensions for the larger cost set.
COLORS = {
    "blue": "#0072B2",
    "orange": "#E69F00",
    "green": "#009E73",
    "vermillion": "#D55E00",
    "purple": "#CC79A7",
    "sky": "#56B4E9",
    "yellow": "#F0E442",
    "black": "#202020",
    "gray": "#777777",
}

SERIES_COLORS = (
    COLORS["blue"],
    COLORS["orange"],
    COLORS["green"],
    COLORS["vermillion"],
    COLORS["purple"],
    COLORS["sky"],
    "#8C564B",
    "#7F7F7F",
    "#BCBD22",
    "#17BECF",
    "#9467BD",
)

LINE_STYLES = ("-", "--", "-.", ":", (0, (5, 2, 1, 2)))

PLOT_STYLE = {
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "axes.grid": True,
    "axes.axisbelow": True,
    "grid.alpha": 0.22,
    "grid.linewidth": 0.6,
    "legend.fontsize": 8,
    "lines.linewidth": 1.45,
    "savefig.facecolor": "white",
    "figure.facecolor": "white",
}


@dataclass
class AuditRecord:
    cycle_id: int
    cycle_start_sec: float
    solver_input_sec: float
    solve_start_sec: float
    solve_end_sec: float
    publish_sec: float
    solve_attempted: bool
    solve_success: bool
    command_accepted: bool
    command_was_published: bool
    status: str
    solver_status: str
    a0: float
    previous_a1_available: bool
    previous_a1: float
    delta_a0: float
    solver_v: float
    solver_omega: float
    post_gate_v: float
    post_gate_omega: float
    published_v: float
    published_omega: float
    state_alignment_required: bool = False
    state_time_aligned: bool = True
    terminal_intervened: bool = False
    safety_intervened: bool = False
    command_contract_violation: bool = False
    linear_limited: bool = False
    angular_rate_limited: bool = False
    angular_accel_limited: bool = False
    zero_due_to_speed_safety: bool = False

    @property
    def decision_sec(self) -> float:
        return first_valid_stamp(self.solver_input_sec, self.cycle_start_sec)


@dataclass
class SnapshotRecord:
    cycle_id: int
    solver_input_sec: float
    valid: bool
    actuator_state_valid: bool
    a_cmd_memory: float
    eta_x: float
    eta_x_dot: float
    eta_y: float
    eta_y_dot: float


@dataclass
class HorizonRecord:
    cycle_id: int
    solver_input_sec: float
    available_sec: float
    valid: bool
    dt: float
    t: np.ndarray
    fields: Dict[str, np.ndarray]


@dataclass
class OdomRecord:
    stamp_sec: float
    v: float
    omega: float


@dataclass
class ImuRecord:
    stamp_sec: float
    ax: float
    ay: float


@dataclass
class ObserverRecord:
    stamp_sec: float
    valid: bool
    source: int
    input_status: str
    ax: float
    ay: float
    eta_x: float
    eta_x_dot: float
    eta_y: float
    eta_y_dot: float
    modal_height_m: float


@dataclass
class RgbRecord:
    stamp_sec: float
    height_mm: float


@dataclass
class CostRecord:
    cycle_id: int
    stamp_sec: float
    values: Dict[str, float]


@dataclass
class RawCommandRecord:
    cycle_id: int
    stamp_sec: float
    v: float
    omega: float


@dataclass
class DiagnosticData:
    bag_path: Optional[Path] = None
    audits: List[AuditRecord] = field(default_factory=list)
    snapshots: List[SnapshotRecord] = field(default_factory=list)
    horizons: List[HorizonRecord] = field(default_factory=list)
    odom: List[OdomRecord] = field(default_factory=list)
    imu: List[ImuRecord] = field(default_factory=list)
    observers: List[ObserverRecord] = field(default_factory=list)
    rgb: List[RgbRecord] = field(default_factory=list)
    costs: List[CostRecord] = field(default_factory=list)
    raw_commands: List[RawCommandRecord] = field(default_factory=list)
    topic_counts: Dict[str, int] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class MotionAxis:
    origin_sec: float
    end_sec: float
    source: str

    @property
    def duration_sec(self) -> float:
        return max(1.0 / 30.0, self.end_sec - self.origin_sec)

    @property
    def limits(self) -> Tuple[float, float]:
        return (0.0, self.duration_sec)

    def relative(self, stamp_sec: float) -> float:
        return float(stamp_sec) - self.origin_sec


@dataclass(frozen=True)
class EventMarker:
    stamp_sec: float
    kind: str


EVENT_STYLE = {
    "strong_flip": (COLORS["purple"], ":", "strong a0 flip"),
    "fail_closed": (COLORS["orange"], "-.", "fail-closed"),
    "solver_failure": (COLORS["vermillion"], "--", "solver failure"),
    "intervention": (COLORS["blue"], (0, (2, 2)), "output intervention"),
}


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def number(value: Any, default: float = math.nan) -> float:
    return float(value) if finite(value) else float(default)


def stamp_sec(stamp: Any) -> float:
    """Convert a ROS-like stamp without inventing a receive-time fallback."""
    if stamp is None:
        return math.nan
    try:
        value = float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError, OverflowError):
        try:
            value = float(stamp.secs) + 1.0e-9 * float(stamp.nsecs)
        except (AttributeError, TypeError, ValueError, OverflowError):
            value = number(stamp)
    return value if finite(value) and value > 0.0 else math.nan


def first_valid_stamp(*values: float) -> float:
    for value in values:
        if finite(value) and float(value) > 0.0:
            return float(value)
    return math.nan


def explicit_stamp(message: Any, *field_names: str) -> float:
    for field_name in field_names:
        value = stamp_sec(getattr(message, field_name, None))
        if finite(value):
            return value
    return math.nan


def float_array(message: Any, field_name: str) -> np.ndarray:
    raw = getattr(message, field_name, [])
    try:
        return np.asarray(list(raw), dtype=float)
    except (TypeError, ValueError):
        return np.asarray([], dtype=float)


def parse_multiarray(message: Any) -> Dict[str, float]:
    dimensions = getattr(getattr(message, "layout", None), "dim", [])
    label = str(getattr(dimensions[0], "label", "")) if dimensions else ""
    names = [part.strip() for part in label.split(",") if part.strip()]
    values = list(getattr(message, "data", []))
    return {
        names[index] if index < len(names) else "field_{}".format(index): number(value)
        for index, value in enumerate(values)
    }


def audit_from_message(message: Any) -> AuditRecord:
    return AuditRecord(
        cycle_id=int(getattr(message, "cycle_id", 0)),
        cycle_start_sec=explicit_stamp(message, "cycle_start_stamp"),
        solver_input_sec=explicit_stamp(message, "solver_input_epoch"),
        solve_start_sec=explicit_stamp(message, "solve_start_stamp"),
        solve_end_sec=explicit_stamp(message, "solve_end_stamp"),
        publish_sec=explicit_stamp(message, "command_publish_stamp"),
        solve_attempted=bool(getattr(message, "solve_attempted", False)),
        solve_success=bool(getattr(message, "solve_success", False)),
        command_accepted=bool(getattr(message, "command_accepted", False)),
        command_was_published=bool(getattr(message, "command_was_published", False)),
        status=str(getattr(message, "status", "")),
        solver_status=str(getattr(message, "solver_status", "")),
        a0=number(getattr(message, "solver_u0_a", math.nan)),
        previous_a1_available=bool(
            getattr(message, "previous_shifted_plan_available", False)
        ),
        previous_a1=number(getattr(message, "previous_shifted_plan_a", math.nan)),
        delta_a0=number(getattr(message, "replanned_minus_shifted_a", math.nan)),
        solver_v=number(getattr(message, "solver_cmd_v", math.nan)),
        solver_omega=number(getattr(message, "solver_cmd_omega", math.nan)),
        post_gate_v=number(getattr(message, "post_gate_cmd_v", math.nan)),
        post_gate_omega=number(getattr(message, "post_gate_cmd_omega", math.nan)),
        published_v=number(getattr(message, "published_cmd_v", math.nan)),
        published_omega=number(getattr(message, "published_cmd_omega", math.nan)),
        state_alignment_required=bool(
            getattr(message, "state_alignment_required", False)
        ),
        state_time_aligned=bool(getattr(message, "state_time_aligned", True)),
        terminal_intervened=bool(
            getattr(message, "terminal_controller_intervened", False)
        ),
        safety_intervened=bool(getattr(message, "safety_gate_intervened", False)),
        command_contract_violation=bool(
            getattr(message, "command_contract_violation", False)
        ),
        linear_limited=bool(getattr(message, "linear_limited", False)),
        angular_rate_limited=bool(getattr(message, "angular_rate_limited", False)),
        angular_accel_limited=bool(getattr(message, "angular_accel_limited", False)),
        zero_due_to_speed_safety=bool(
            getattr(message, "zero_due_to_speed_safety", False)
        ),
    )


def snapshot_from_message(message: Any) -> SnapshotRecord:
    return SnapshotRecord(
        cycle_id=int(getattr(message, "cycle_id", 0)),
        solver_input_sec=explicit_stamp(
            message, "solver_input_epoch", "cycle_start_stamp"
        ),
        valid=bool(getattr(message, "valid", False)),
        actuator_state_valid=bool(getattr(message, "actuator_state_valid", False)),
        a_cmd_memory=number(getattr(message, "actuator_a_cmd_memory", math.nan)),
        eta_x=number(getattr(message, "eta_x", math.nan)),
        eta_x_dot=number(getattr(message, "eta_x_dot", math.nan)),
        eta_y=number(getattr(message, "eta_y", math.nan)),
        eta_y_dot=number(getattr(message, "eta_y_dot", math.nan)),
    )


def horizon_from_message(message: Any) -> HorizonRecord:
    fields = {
        name: float_array(message, name)
        for name in (
            "v",
            "omega",
            "eta_x",
            "eta_x_dot",
            "eta_y",
            "eta_y_dot",
            "h_modal",
            "a_cmd_memory",
            "a_actual",
            "alpha_actual",
            "a",
        )
    }
    return HorizonRecord(
        cycle_id=int(getattr(message, "cycle_id", 0)),
        solver_input_sec=explicit_stamp(
            message, "solver_input_epoch", "cycle_start_stamp"
        ),
        available_sec=explicit_stamp(message, "horizon_available_stamp"),
        valid=bool(getattr(message, "valid", False)),
        dt=number(getattr(message, "dt", math.nan)),
        t=float_array(message, "t"),
        fields=fields,
    )


def observer_from_message(message: Any) -> Optional[ObserverRecord]:
    effective = explicit_stamp(message, "state_stamp")
    if not finite(effective):
        return None
    return ObserverRecord(
        stamp_sec=effective,
        valid=bool(getattr(message, "valid", False)),
        source=int(getattr(message, "source", 0)),
        input_status=str(getattr(message, "input_status", "")),
        ax=number(getattr(message, "ax_mps2", math.nan)),
        ay=number(getattr(message, "ay_mps2", math.nan)),
        eta_x=number(getattr(message, "eta_x", math.nan)),
        eta_x_dot=number(getattr(message, "eta_x_dot", math.nan)),
        eta_y=number(getattr(message, "eta_y", math.nan)),
        eta_y_dot=number(getattr(message, "eta_y_dot", math.nan)),
        modal_height_m=number(getattr(message, "modal_height_m", math.nan)),
    )


def rgb_from_message(message: Any) -> Optional[RgbRecord]:
    header = getattr(message, "header", None)
    effective = stamp_sec(getattr(header, "stamp", None))
    clean = bool(
        finite(effective)
        and getattr(message, "valid", False)
        and getattr(message, "zero_locked", False)
        and int(getattr(message, "status_code", -1)) == 0
        and not getattr(message, "any_clipped", True)
    )
    value = number(getattr(message, "height_max_lcr_mm", math.nan))
    if not (clean and finite(value)):
        return None
    return RgbRecord(effective, value)


def load_bag(bag_path: Path) -> DiagnosticData:
    """Read and normalize a ROS1 bag without using bag receive timestamps."""
    try:
        import rosbag  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "需要 rosbag；请 source /opt/ros/noetic/setup.bash "
            "和工作区 devel/setup.bash"
        ) from exc

    data = DiagnosticData(bag_path=bag_path)
    cost_messages: List[Dict[str, float]] = []
    slosh_cost_messages: List[Dict[str, float]] = []
    raw_command_values: List[Tuple[float, float]] = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, message, _unused_bag_receive_stamp in bag.read_messages(
            topics=list(READ_TOPICS)
        ):
            data.topic_counts[topic] = data.topic_counts.get(topic, 0) + 1
            if topic == AUDIT_TOPIC:
                data.audits.append(audit_from_message(message))
            elif topic == SNAPSHOT_TOPIC:
                data.snapshots.append(snapshot_from_message(message))
            elif topic == HORIZON_TOPIC:
                data.horizons.append(horizon_from_message(message))
            elif topic == COST_TOPIC:
                cost_messages.append(parse_multiarray(message))
            elif topic == SLOSH_COST_TOPIC:
                slosh_cost_messages.append(parse_multiarray(message))
            elif topic == ODOM_TOPIC:
                header = getattr(message, "header", None)
                effective = stamp_sec(getattr(header, "stamp", None))
                twist = getattr(getattr(message, "twist", None), "twist", None)
                if finite(effective) and twist is not None:
                    data.odom.append(
                        OdomRecord(
                            effective,
                            number(
                                getattr(getattr(twist, "linear", None), "x", math.nan)
                            ),
                            number(
                                getattr(getattr(twist, "angular", None), "z", math.nan)
                            ),
                        )
                    )
            elif topic == IMU_TOPIC:
                header = getattr(message, "header", None)
                effective = stamp_sec(getattr(header, "stamp", None))
                acceleration = getattr(message, "linear_acceleration", None)
                if finite(effective) and acceleration is not None:
                    data.imu.append(
                        ImuRecord(
                            effective,
                            number(getattr(acceleration, "x", math.nan)),
                            number(getattr(acceleration, "y", math.nan)),
                        )
                    )
            elif topic == OBSERVER_TOPIC:
                record = observer_from_message(message)
                if record is not None:
                    data.observers.append(record)
            elif topic == RGB_TOPIC:
                record = rgb_from_message(message)
                if record is not None:
                    data.rgb.append(record)
            elif topic == CMD_TOPIC:
                raw_command_values.append(
                    (
                        number(
                            getattr(getattr(message, "linear", None), "x", math.nan)
                        ),
                        number(
                            getattr(getattr(message, "angular", None), "z", math.nan)
                        ),
                    )
                )

    # Cost messages have no header/cycle_id.  The diagnostics publisher emits
    # one horizon followed by one cost record per solved cycle, so publication
    # order binds the values to the horizon cycle.  Time remains the horizon's
    # explicit solver_input_epoch, never the bag receive stamp.
    paired_cost_count = min(
        len(data.horizons), max(len(cost_messages), len(slosh_cost_messages))
    )
    for index in range(paired_cost_count):
        horizon = data.horizons[index]
        merged = dict(cost_messages[index]) if index < len(cost_messages) else {}
        if index < len(slosh_cost_messages):
            merged.update(slosh_cost_messages[index])
        data.costs.append(
            CostRecord(horizon.cycle_id, horizon.solver_input_sec, merged)
        )
    if len(cost_messages) != len(data.horizons):
        data.warnings.append(
            (
                "headerless cost/horizon count mismatch: {} vs {}; "
                "only ordered pairs are plotted"
            ).format(len(cost_messages), len(data.horizons))
        )
    if slosh_cost_messages and len(slosh_cost_messages) != len(data.horizons):
        data.warnings.append(
            "headerless slosh-cost/horizon count mismatch: {} vs {}".format(
                len(slosh_cost_messages), len(data.horizons)
            )
        )

    # /cmd_vel is also headerless.  Audit records are the authoritative output
    # ledger and supply the publish stamp; raw values are an order-only check.
    published_audits = [
        audit
        for audit in data.audits
        if audit.command_was_published and finite(audit.publish_sec)
    ]
    for audit, (v, omega) in zip(published_audits, raw_command_values):
        data.raw_commands.append(
            RawCommandRecord(audit.cycle_id, audit.publish_sec, v, omega)
        )
    if len(raw_command_values) != len(published_audits):
        data.warnings.append(
            (
                "headerless /cmd_vel/audit count mismatch: {} vs {}; "
                "audit output remains authoritative"
            ).format(len(raw_command_values), len(published_audits))
        )

    for required in (AUDIT_TOPIC, SNAPSHOT_TOPIC, HORIZON_TOPIC, ODOM_TOPIC):
        if not data.topic_counts.get(required):
            data.warnings.append("missing topic: {}".format(required))

    data.audits.sort(key=lambda record: sort_stamp(record.decision_sec))
    data.snapshots.sort(key=lambda record: sort_stamp(record.solver_input_sec))
    data.horizons.sort(key=lambda record: sort_stamp(record.solver_input_sec))
    data.odom.sort(key=lambda record: sort_stamp(record.stamp_sec))
    data.imu.sort(key=lambda record: sort_stamp(record.stamp_sec))
    data.observers.sort(key=lambda record: sort_stamp(record.stamp_sec))
    data.rgb.sort(key=lambda record: sort_stamp(record.stamp_sec))
    data.costs.sort(key=lambda record: sort_stamp(record.stamp_sec))
    data.raw_commands.sort(key=lambda record: sort_stamp(record.stamp_sec))
    return data


def sort_stamp(value: float) -> float:
    return float(value) if finite(value) else math.inf


def _active_command_stamps(
    audits: Iterable[AuditRecord], linear_threshold: float, angular_threshold: float
) -> List[float]:
    return [
        record.publish_sec
        for record in audits
        if record.command_was_published
        and finite(record.publish_sec)
        and (
            abs(record.published_v) > linear_threshold
            or abs(record.published_omega) > angular_threshold
        )
    ]


def determine_motion_axis(
    data: DiagnosticData,
    linear_threshold: float = 0.03,
    angular_threshold: float = 0.03,
) -> MotionAxis:
    command_active = _active_command_stamps(
        data.audits, linear_threshold, angular_threshold
    )
    odom_active = [
        record.stamp_sec
        for record in data.odom
        if finite(record.stamp_sec)
        and (abs(record.v) > linear_threshold or abs(record.omega) > angular_threshold)
    ]
    active = command_active + odom_active
    if active:
        start = min(active)
        end = max(active)
        if end <= start:
            end = start + 1.0
        return MotionAxis(start, end, "nonzero command/odom")

    effective = []
    effective.extend(
        record.decision_sec for record in data.audits if finite(record.decision_sec)
    )
    effective.extend(
        record.stamp_sec for record in data.odom if finite(record.stamp_sec)
    )
    effective.extend(
        record.stamp_sec for record in data.observers if finite(record.stamp_sec)
    )
    effective.extend(
        record.solver_input_sec
        for record in data.snapshots
        if finite(record.solver_input_sec)
    )
    effective.extend(
        record.solver_input_sec
        for record in data.horizons
        if finite(record.solver_input_sec)
    )
    effective.extend(
        record.stamp_sec for record in data.imu if finite(record.stamp_sec)
    )
    effective.extend(
        record.stamp_sec for record in data.rgb if finite(record.stamp_sec)
    )
    effective.extend(
        record.stamp_sec for record in data.costs if finite(record.stamp_sec)
    )
    if effective:
        start = min(effective)
        end = max(effective)
        if end <= start:
            end = start + 1.0
        return MotionAxis(start, end, "effective timestamps (no motion detected)")
    return MotionAxis(0.0, 1.0, "no valid effective timestamps")


def _status_fail_closed(record: AuditRecord) -> bool:
    status = "{} {}".format(record.status, record.solver_status).upper()
    tokens = (
        "FAIL_CLOSED",
        "WAITING_FOR",
        "CONTRACT_VIOLATION",
        "SAFETY_CONTRACT",
        "STALE_STATE",
        "INVALID_STATE",
    )
    return bool(
        record.zero_due_to_speed_safety
        or (record.state_alignment_required and not record.state_time_aligned)
        or any(token in status for token in tokens)
    )


def _output_intervened(record: AuditRecord, tolerance: float = 1.0e-6) -> bool:
    explicit = bool(
        record.terminal_intervened
        or record.safety_intervened
        or record.command_contract_violation
        or record.linear_limited
        or record.angular_rate_limited
        or record.angular_accel_limited
    )
    changed = bool(
        finite(record.solver_v)
        and finite(record.published_v)
        and (
            abs(record.solver_v - record.published_v) > tolerance
            or (
                finite(record.solver_omega)
                and finite(record.published_omega)
                and abs(record.solver_omega - record.published_omega) > tolerance
            )
        )
    )
    return explicit or changed


def build_event_markers(
    audits: Sequence[AuditRecord], strong_a0_threshold: float = 0.5
) -> List[EventMarker]:
    records = sorted(audits, key=lambda record: sort_stamp(record.decision_sec))
    events: List[EventMarker] = []
    previous_audit: Optional[AuditRecord] = None
    previous_flags = {
        "fail_closed": False,
        "solver_failure": False,
        "intervention": False,
    }
    for record in records:
        decision_stamp = first_valid_stamp(record.decision_sec, record.publish_sec)
        publish_stamp = first_valid_stamp(record.publish_sec, record.decision_sec)
        if not finite(decision_stamp):
            continue
        if (
            previous_audit is not None
            and finite(previous_audit.a0)
            and finite(record.a0)
            and previous_audit.a0 * record.a0 < 0.0
            and abs(previous_audit.a0) >= strong_a0_threshold
            and abs(record.a0) >= strong_a0_threshold
        ):
            events.append(EventMarker(decision_stamp, "strong_flip"))

        flags = {
            "fail_closed": _status_fail_closed(record),
            "solver_failure": bool(record.solve_attempted and not record.solve_success),
            "intervention": _output_intervened(record),
        }
        for kind, active in flags.items():
            # Persistent gates are marked at onset, while a cleared-and-retried
            # gate produces a new marker on its next onset.  Every failed solve
            # remains a distinct event even when failures are consecutive.
            if active and (kind == "solver_failure" or not previous_flags[kind]):
                stamp = (
                    first_valid_stamp(record.solve_end_sec, decision_stamp)
                    if kind == "solver_failure"
                    else publish_stamp
                )
                events.append(EventMarker(stamp, kind))
            previous_flags[kind] = active
        previous_audit = record
    return sorted(events, key=lambda event: (event.stamp_sec, event.kind))


def clean_xy(
    times: Sequence[float], values: Sequence[float]
) -> Tuple[np.ndarray, np.ndarray]:
    pairs = sorted(
        (float(time), float(value))
        for time, value in zip(times, values)
        if finite(time) and finite(value)
    )
    if not pairs:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)
    # Keep the last value for duplicate effective stamps.
    unique: Dict[float, float] = {}
    for time, value in pairs:
        unique[time] = value
    return np.asarray(list(unique.keys()), dtype=float), np.asarray(
        list(unique.values()), dtype=float
    )


def bounded_interpolate(
    target_sec: float,
    times: Sequence[float],
    values: Sequence[float],
    max_bracket_gap_sec: float,
) -> float:
    x, y = clean_xy(times, values)
    return bounded_interpolate_prepared(target_sec, x, y, max_bracket_gap_sec)


def bounded_interpolate_prepared(
    target_sec: float,
    times: np.ndarray,
    values: np.ndarray,
    max_bracket_gap_sec: float,
) -> float:
    """Interpolate sorted/finite arrays without repeatedly preprocessing them."""
    x = times
    y = values
    if x.size == 0 or not finite(target_sec):
        return math.nan
    index = bisect.bisect_left(x, float(target_sec))
    if index < x.size and abs(x[index] - target_sec) <= 1.0e-9:
        return float(y[index])
    if index == 0 or index >= x.size:
        return math.nan
    left = index - 1
    if x[index] - x[left] > max_bracket_gap_sec:
        return math.nan
    weight = (float(target_sec) - x[left]) / (x[index] - x[left])
    return float(y[left] + weight * (y[index] - y[left]))


def sample_horizon(record: HorizonRecord, field_name: str, lead_sec: float) -> float:
    values = record.fields.get(field_name, np.asarray([], dtype=float))
    if values.size == 0 or not finite(lead_sec) or lead_sec < 0.0:
        return math.nan
    if record.t.size == values.size:
        times = record.t
    elif finite(record.dt) and record.dt > 0.0:
        times = np.arange(values.size, dtype=float) * record.dt
    else:
        return math.nan
    mask = np.isfinite(times) & np.isfinite(values)
    times = times[mask]
    values = values[mask]
    if times.size == 0 or lead_sec < times[0] - 1.0e-9 or lead_sec > times[-1] + 1.0e-9:
        return math.nan
    return float(np.interp(float(lead_sec), times, values))


def compute_spectrum(
    times: Sequence[float], values: Sequence[float]
) -> Tuple[np.ndarray, np.ndarray]:
    x, y = clean_xy(times, values)
    if x.size < 4:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)
    deltas = np.diff(x)
    deltas = deltas[np.isfinite(deltas) & (deltas > 1.0e-6)]
    if deltas.size == 0:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)
    dt = float(np.median(deltas))
    grid = np.arange(x[0], x[-1] + 0.5 * dt, dt)
    if grid.size < 4:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)
    uniform = np.interp(grid, x, y)
    uniform = uniform - float(np.mean(uniform))
    window = np.hanning(uniform.size)
    normalization = float(np.sum(window))
    if normalization <= 0.0:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)
    amplitude = 2.0 * np.abs(np.fft.rfft(uniform * window)) / normalization
    frequencies = np.fft.rfftfreq(uniform.size, d=dt)
    if amplitude.size:
        amplitude[0] *= 0.5
    return frequencies, amplitude


def _relative_array(axis: MotionAxis, values: Sequence[float]) -> np.ndarray:
    return np.asarray([axis.relative(value) for value in values], dtype=float)


def _annotate_empty(axes: Any, message: str) -> None:
    axes.text(
        0.5,
        0.5,
        message,
        ha="center",
        va="center",
        color=COLORS["gray"],
        transform=axes.transAxes,
        bbox={"facecolor": "white", "edgecolor": "#BBBBBB", "alpha": 0.9},
    )


def _plot_if_any(
    axes: Any,
    times: Sequence[float],
    values: Sequence[float],
    axis: MotionAxis,
    *args: Any,
    **kwargs: Any,
) -> bool:
    x, y = clean_xy(times, values)
    if x.size == 0:
        return False
    axes.plot(_relative_array(axis, x), y, *args, **kwargs)
    return True


def _decorate_time_axes(
    figure: Any,
    axes: Sequence[Any],
    motion: MotionAxis,
    events: Sequence[EventMarker],
) -> None:
    kinds_present = []
    for event in events:
        x = motion.relative(event.stamp_sec)
        if x < motion.limits[0] or x > motion.limits[1]:
            continue
        color, linestyle, _label = EVENT_STYLE[event.kind]
        for current in axes:
            current.axvline(
                x,
                color=color,
                linestyle=linestyle,
                linewidth=0.9,
                alpha=0.58,
                zorder=0,
            )
        if event.kind not in kinds_present:
            kinds_present.append(event.kind)
    for current in axes:
        current.set_xlim(*motion.limits)
    axes[-1].set_xlabel("Motion-relative time [s]")
    if kinds_present:
        handles = [
            Line2D(
                [0],
                [0],
                color=EVENT_STYLE[kind][0],
                linestyle=EVENT_STYLE[kind][1],
                linewidth=1.2,
                label=EVENT_STYLE[kind][2],
            )
            for kind in kinds_present
        ]
        figure.legend(
            handles=handles,
            loc="upper right",
            bbox_to_anchor=(0.995, 0.998),
            ncol=len(handles),
            frameon=False,
        )


def _save(figure: Any, path: Path, dpi: int) -> Path:
    figure.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(figure)
    return path


def plot_timing(
    data: DiagnosticData,
    motion: MotionAxis,
    events: Sequence[EventMarker],
    output: Path,
    dpi: int,
) -> Path:
    figure, axes = plt.subplots(
        4, 1, figsize=(13.5, 10.0), sharex=True, constrained_layout=True
    )
    figure.suptitle(
        "SPMPC full-Delta-a timing and failure events",
        x=0.01,
        ha="left",
        fontweight="bold",
    )

    solver_times, solver_ms = [], []
    callback_times, callback_ms = [], []
    for record in data.audits:
        if (
            finite(record.solve_start_sec)
            and finite(record.solve_end_sec)
            and record.solve_end_sec >= record.solve_start_sec
        ):
            solver_times.append(record.solve_end_sec)
            solver_ms.append(1000.0 * (record.solve_end_sec - record.solve_start_sec))
        if (
            finite(record.cycle_start_sec)
            and finite(record.publish_sec)
            and record.publish_sec >= record.cycle_start_sec
        ):
            callback_times.append(record.publish_sec)
            callback_ms.append(1000.0 * (record.publish_sec - record.cycle_start_sec))
    have = _plot_if_any(
        axes[0], solver_times, solver_ms, motion, color=COLORS["blue"], label="solver"
    )
    have |= _plot_if_any(
        axes[0],
        callback_times,
        callback_ms,
        motion,
        color=COLORS["orange"],
        linestyle="--",
        label="full callback",
    )
    axes[0].axhline(
        1000.0 / 30.0,
        color=COLORS["gray"],
        linestyle=":",
        linewidth=1.0,
        label="30 Hz period",
    )
    axes[0].set_ylabel("Duration [ms]")
    axes[0].set_title("Compute latency")
    if have:
        axes[0].legend(ncol=3, loc="upper left")
    else:
        _annotate_empty(axes[0], "No valid solve/callback timestamps")

    publish = sorted(
        record.publish_sec for record in data.audits if finite(record.publish_sec)
    )
    odom = sorted(record.stamp_sec for record in data.odom if finite(record.stamp_sec))
    imu = sorted(record.stamp_sec for record in data.imu if finite(record.stamp_sec))
    interval_sets = (
        (publish, "publish interval", COLORS["blue"], "-"),
        (odom, "odom interval", COLORS["green"], "--"),
        (imu, "IMU interval", COLORS["purple"], ":"),
    )
    have = False
    for stamps, label, color, linestyle in interval_sets:
        if len(stamps) >= 2:
            have |= _plot_if_any(
                axes[1],
                stamps[1:],
                1000.0 * np.diff(stamps),
                motion,
                color=color,
                linestyle=linestyle,
                label=label,
            )
    axes[1].set_ylabel("Interval [ms]")
    axes[1].set_title("Effective sample/publication intervals")
    if have:
        axes[1].legend(ncol=3, loc="upper left")
    else:
        _annotate_empty(axes[1], "No interval series with two effective stamps")

    _plot_if_any(
        axes[2],
        [record.stamp_sec for record in data.odom],
        [record.v for record in data.odom],
        motion,
        color=COLORS["green"],
        label="odom v",
    )
    _plot_if_any(
        axes[2],
        [record.stamp_sec for record in data.odom],
        [record.omega for record in data.odom],
        motion,
        color=COLORS["purple"],
        linestyle="--",
        label="odom omega",
    )
    axes[2].set_ylabel("v / omega")
    axes[2].set_title("Motion reference for timing interpretation")
    if data.odom:
        axes[2].legend(ncol=2, loc="upper left")
    else:
        _annotate_empty(axes[2], "No odom samples with header.stamp")

    event_levels = {
        "strong_flip": 3,
        "fail_closed": 2,
        "solver_failure": 1,
        "intervention": 0,
    }
    for kind, level in event_levels.items():
        selected = [event for event in events if event.kind == kind]
        if selected:
            axes[3].scatter(
                [motion.relative(event.stamp_sec) for event in selected],
                [level] * len(selected),
                s=28,
                marker="|",
                linewidths=2.2,
                color=EVENT_STYLE[kind][0],
                label=EVENT_STYLE[kind][2],
            )
    axes[3].set_yticks(list(event_levels.values()))
    axes[3].set_yticklabels([EVENT_STYLE[kind][2] for kind in event_levels])
    axes[3].set_ylim(-0.6, 3.6)
    axes[3].set_title("Discrete event onsets")
    if not events:
        _annotate_empty(axes[3], "No marked events")

    _decorate_time_axes(figure, list(axes), motion, events)
    return _save(figure, output, dpi)


def plot_command_chain(
    data: DiagnosticData,
    motion: MotionAxis,
    events: Sequence[EventMarker],
    output: Path,
    dpi: int,
) -> Path:
    figure, axes = plt.subplots(
        3, 1, figsize=(13.5, 9.0), sharex=True, constrained_layout=True
    )
    figure.suptitle(
        "SPMPC replanning command chain (cycle_id join)",
        x=0.01,
        ha="left",
        fontweight="bold",
    )
    audits = [record for record in data.audits if finite(record.decision_sec)]
    decision_times = [record.decision_sec for record in audits]

    have = _plot_if_any(
        axes[0],
        decision_times,
        [record.a0 for record in audits],
        motion,
        color=COLORS["blue"],
        label="current a0",
    )
    previous_times = [
        record.decision_sec for record in audits if record.previous_a1_available
    ]
    previous_values = [
        record.previous_a1 for record in audits if record.previous_a1_available
    ]
    have |= _plot_if_any(
        axes[0],
        previous_times,
        previous_values,
        motion,
        color=COLORS["orange"],
        linestyle="--",
        label="previous plan a1",
    )
    axes[0].axhline(0.0, color=COLORS["gray"], linewidth=0.8)
    axes[0].set_ylabel("Acceleration [m/s²]")
    axes[0].set_title("Shifted-plan action versus replanned first action")
    if have:
        axes[0].legend(ncol=2, loc="upper left")
    else:
        _annotate_empty(axes[0], "No audit command-chain samples")

    delta_records = [record for record in audits if record.previous_a1_available]
    have = _plot_if_any(
        axes[1],
        [record.decision_sec for record in delta_records],
        [record.delta_a0 for record in delta_records],
        motion,
        color=COLORS["purple"],
        label="reported Δa0",
    )
    have |= _plot_if_any(
        axes[1],
        [record.decision_sec for record in delta_records],
        [record.a0 - record.previous_a1 for record in delta_records],
        motion,
        color=COLORS["green"],
        linestyle=":",
        label="a0 - previous a1",
    )
    axes[1].axhline(0.0, color=COLORS["gray"], linewidth=0.8)
    axes[1].set_ylabel("Δ acceleration [m/s²]")
    axes[1].set_title("Replanning discontinuity")
    if have:
        axes[1].legend(ncol=2, loc="upper left")
    else:
        _annotate_empty(axes[1], "No previous-plan continuity records")

    snapshot_by_cycle = {
        record.cycle_id: record
        for record in data.snapshots
        if record.valid and record.actuator_state_valid
    }
    joined = [
        (audit, snapshot_by_cycle[audit.cycle_id])
        for audit in audits
        if audit.cycle_id in snapshot_by_cycle
    ]
    have = _plot_if_any(
        axes[2],
        [audit.decision_sec for audit, _snapshot in joined],
        [snapshot.a_cmd_memory for _audit, snapshot in joined],
        motion,
        color=COLORS["green"],
        label="a_cmd_memory (pre-solve)",
    )
    axes[2].axhline(0.0, color=COLORS["gray"], linewidth=0.8)
    axes[2].set_ylabel("Acceleration [m/s²]")
    axes[2].set_title("Explicit actuator memory, joined by cycle_id")
    if have:
        axes[2].legend(loc="upper left")
    else:
        _annotate_empty(axes[2], "No valid audit/snapshot cycle_id pairs")

    _decorate_time_axes(figure, list(axes), motion, events)
    return _save(figure, output, dpi)


def plot_command_actual(
    data: DiagnosticData,
    motion: MotionAxis,
    events: Sequence[EventMarker],
    output: Path,
    dpi: int,
) -> Path:
    figure, axes = plt.subplots(
        2, 1, figsize=(13.5, 7.2), sharex=True, constrained_layout=True
    )
    figure.suptitle(
        "Solver command → final /cmd_vel → realized odom",
        x=0.01,
        ha="left",
        fontweight="bold",
    )
    audits = data.audits
    decision_times = [record.decision_sec for record in audits]
    publish_times = [record.publish_sec for record in audits]

    fields = (
        (axes[0], "v", "Linear speed [m/s]"),
        (axes[1], "omega", "Angular speed [rad/s]"),
    )
    for current, suffix, ylabel in fields:
        have = _plot_if_any(
            current,
            decision_times,
            [getattr(record, "solver_{}".format(suffix)) for record in audits],
            motion,
            color=COLORS["blue"],
            label="solver_cmd",
        )
        have |= _plot_if_any(
            current,
            publish_times,
            [getattr(record, "post_gate_{}".format(suffix)) for record in audits],
            motion,
            color=COLORS["orange"],
            linestyle="--",
            label="post_gate",
        )
        have |= _plot_if_any(
            current,
            publish_times,
            [getattr(record, "published_{}".format(suffix)) for record in audits],
            motion,
            color=COLORS["purple"],
            linestyle="-.",
            label="final /cmd_vel (audit)",
        )
        have |= _plot_if_any(
            current,
            [record.stamp_sec for record in data.raw_commands],
            [getattr(record, suffix) for record in data.raw_commands],
            motion,
            color=COLORS["black"],
            linestyle="None",
            marker=".",
            markersize=2.5,
            alpha=0.55,
            label="raw /cmd_vel (order check)",
        )
        have |= _plot_if_any(
            current,
            [record.stamp_sec for record in data.odom],
            [getattr(record, suffix) for record in data.odom],
            motion,
            color=COLORS["green"],
            linewidth=1.1,
            alpha=0.9,
            label="odom actual",
        )
        current.axhline(0.0, color=COLORS["gray"], linewidth=0.8)
        current.set_ylabel(ylabel)
        if have:
            current.legend(ncol=5, loc="upper left")
        else:
            _annotate_empty(current, "No command or odom data")
    axes[0].set_title("Linear channel")
    axes[1].set_title("Angular channel")
    figure.text(
        0.01,
        0.005,
        "Audit command_publish_stamp is authoritative; headerless /cmd_vel "
        "is aligned only by publication order.",
        fontsize=8,
        color=COLORS["gray"],
    )
    _decorate_time_axes(figure, list(axes), motion, events)
    return _save(figure, output, dpi)


def _prediction_rows(
    data: DiagnosticData,
    field_name: str,
    lead_sec: float,
    odom_gap_sec: float,
) -> Tuple[List[float], List[float], List[float]]:
    odom_times, odom_values = clean_xy(
        [record.stamp_sec for record in data.odom],
        [getattr(record, field_name) for record in data.odom],
    )
    targets, predicted, realized = [], [], []
    for horizon in data.horizons:
        if not (horizon.valid and finite(horizon.solver_input_sec)):
            continue
        value = sample_horizon(horizon, field_name, lead_sec)
        if not finite(value):
            continue
        target = horizon.solver_input_sec + lead_sec
        targets.append(target)
        predicted.append(value)
        realized.append(
            bounded_interpolate_prepared(target, odom_times, odom_values, odom_gap_sec)
        )
    return targets, predicted, realized


def plot_prediction(
    data: DiagnosticData,
    motion: MotionAxis,
    events: Sequence[EventMarker],
    output: Path,
    dpi: int,
    odom_gap_sec: float,
) -> Path:
    figure, axes = plt.subplots(
        2, 5, figsize=(19.0, 7.6), sharex=True, constrained_layout=True
    )
    figure.suptitle(
        "Predicted actuator-state velocity versus future odom",
        x=0.01,
        ha="left",
        fontweight="bold",
    )
    for column, (lead, lead_label) in enumerate(zip(LEADS_SEC, LEAD_LABELS)):
        for row, (field_name, ylabel) in enumerate(
            (("v", "v [m/s]"), ("omega", "omega [rad/s]"))
        ):
            current = axes[row, column]
            targets, predicted, realized = _prediction_rows(
                data, field_name, lead, odom_gap_sec
            )
            have = _plot_if_any(
                current,
                targets,
                predicted,
                motion,
                color=COLORS["blue"],
                label="MPC predicted actual",
            )
            have |= _plot_if_any(
                current,
                targets,
                realized,
                motion,
                color=COLORS["orange"],
                linestyle="--",
                label="future odom",
            )
            current.axhline(0.0, color=COLORS["gray"], linewidth=0.7)
            current.set_title("Lead {}".format(lead_label))
            if column == 0:
                current.set_ylabel(ylabel)
                if have:
                    current.legend(loc="upper left")
            if not have:
                _annotate_empty(current, "No valid\nprediction pairs")
    time_axes = [current for row in axes for current in row]
    _decorate_time_axes(figure, time_axes, motion, events)
    return _save(figure, output, dpi)


def _horizon_forecast(
    data: DiagnosticData, field_name: str, lead_sec: float
) -> Tuple[List[float], List[float]]:
    times, values = [], []
    for horizon in data.horizons:
        if not (horizon.valid and finite(horizon.solver_input_sec)):
            continue
        value = sample_horizon(horizon, field_name, lead_sec)
        if finite(value):
            times.append(horizon.solver_input_sec + lead_sec)
            values.append(value)
    return times, values


def plot_liquid(
    data: DiagnosticData,
    motion: MotionAxis,
    events: Sequence[EventMarker],
    output: Path,
    dpi: int,
) -> Path:
    figure, axes = plt.subplots(
        4, 1, figsize=(13.5, 11.0), sharex=True, constrained_layout=True
    )
    figure.suptitle(
        "Processed-IMU I0 state and MPC liquid forecasts",
        x=0.01,
        ha="left",
        fontweight="bold",
    )
    observers = [
        record
        for record in data.observers
        if record.valid
        and record.source == 2
        and record.input_status.upper() == "READY"
    ]
    observer_times = [record.stamp_sec for record in observers]

    have_height = _plot_if_any(
        axes[0],
        observer_times,
        [1000.0 * record.modal_height_m for record in observers],
        motion,
        color=COLORS["black"],
        linewidth=1.7,
        label="processed-IMU I0",
    )
    for index, (lead, label) in enumerate(zip(LEADS_SEC, LEAD_LABELS)):
        times, values = _horizon_forecast(data, "h_modal", lead)
        have_height |= _plot_if_any(
            axes[0],
            times,
            1000.0 * np.asarray(values),
            motion,
            color=SERIES_COLORS[index],
            linestyle=LINE_STYLES[index],
            alpha=0.82,
            label="MPC +{}".format(label),
        )
    rgb_plotted = _plot_if_any(
        axes[0],
        [record.stamp_sec for record in data.rgb],
        [record.height_mm for record in data.rgb],
        motion,
        color=COLORS["vermillion"],
        linestyle="None",
        marker="o",
        markersize=2.6,
        alpha=0.65,
        label="RGB valid truth",
    )
    axes[0].set_ylabel("Modal height [mm]")
    axes[0].set_title("Modal height forecast at fixed leads")
    if have_height or rgb_plotted:
        axes[0].legend(ncol=4, loc="upper left")
    else:
        _annotate_empty(axes[0], "No valid I0/horizon/RGB liquid height")
    if not data.rgb:
        axes[0].text(
            0.99,
            0.05,
            "RGB unavailable or no sample passed validity gates",
            ha="right",
            va="bottom",
            transform=axes[0].transAxes,
            fontsize=8,
            color=COLORS["gray"],
        )

    for axis_index, field_name in ((1, "eta_x"), (2, "eta_y")):
        current = axes[axis_index]
        have = _plot_if_any(
            current,
            observer_times,
            [1000.0 * getattr(record, field_name) for record in observers],
            motion,
            color=COLORS["black"],
            linewidth=1.7,
            label="processed-IMU I0",
        )
        for index, (lead, label) in enumerate(zip(LEADS_SEC, LEAD_LABELS)):
            times, values = _horizon_forecast(data, field_name, lead)
            have |= _plot_if_any(
                current,
                times,
                1000.0 * np.asarray(values),
                motion,
                color=SERIES_COLORS[index],
                linestyle=LINE_STYLES[index],
                alpha=0.78,
                label="MPC +{}".format(label),
            )
        current.axhline(0.0, color=COLORS["gray"], linewidth=0.8)
        current.set_ylabel("{} [mm]".format(field_name))
        current.set_title("{} observer and forecasts".format(field_name))
        if have:
            current.legend(ncol=6, loc="upper left")
        else:
            _annotate_empty(current, "No valid {} samples".format(field_name))

    have = _plot_if_any(
        axes[3],
        observer_times,
        [record.ax for record in observers],
        motion,
        color=COLORS["blue"],
        label="processed ax",
    )
    have |= _plot_if_any(
        axes[3],
        observer_times,
        [record.ay for record in observers],
        motion,
        color=COLORS["orange"],
        linestyle="--",
        label="processed ay",
    )
    axes[3].axhline(0.0, color=COLORS["gray"], linewidth=0.8)
    axes[3].set_ylabel("Excitation [m/s²]")
    axes[3].set_title("Processed-IMU excitation driving I0")
    if have:
        axes[3].legend(ncol=2, loc="upper left")
    else:
        _annotate_empty(axes[3], "No valid processed-IMU excitation")

    _decorate_time_axes(figure, list(axes), motion, events)
    return _save(figure, output, dpi)


def plot_cost_frequency(
    data: DiagnosticData,
    motion: MotionAxis,
    events: Sequence[EventMarker],
    output: Path,
    dpi: int,
) -> Path:
    figure, axes = plt.subplots(5, 1, figsize=(14.5, 14.0), constrained_layout=True)
    figure.suptitle(
        "Cost evolution and a0 frequency — approximate diagnostic",
        x=0.01,
        ha="left",
        fontweight="bold",
    )
    costs = [record for record in data.costs if finite(record.stamp_sec)]
    cost_times = [record.stamp_sec for record in costs]
    color_by_term = dict(zip(COST_TERMS, SERIES_COLORS))
    groups = (
        ("Tracking / progress", ("J_contour", "J_lag", "J_progress", "J_v")),
        ("Control / terminal", ("J_control", "J_smooth", "J_terminal")),
        (
            "Constraints / liquid",
            ("J_corridor", "J_obstacle", "J_slosh_eta", "J_slosh_eta_dot"),
        ),
    )
    for current, (title, terms) in zip(axes[:3], groups):
        have = False
        if current is axes[0]:
            have |= _plot_if_any(
                current,
                cost_times,
                [record.values.get("total", math.nan) for record in costs],
                motion,
                color=COLORS["black"],
                linewidth=1.8,
                label="total",
            )
        for index, term in enumerate(terms):
            have |= _plot_if_any(
                current,
                cost_times,
                [record.values.get(term, math.nan) for record in costs],
                motion,
                color=color_by_term[term],
                linestyle=LINE_STYLES[index % len(LINE_STYLES)],
                label=term,
            )
        current.axhline(0.0, color=COLORS["gray"], linewidth=0.8)
        current.set_ylabel("Published value")
        current.set_title(title + " (signed)")
        if have:
            current.legend(ncol=5, loc="upper left")
        else:
            _annotate_empty(current, "No cycle-associated cost records")

    if costs:
        matrix = np.asarray(
            [
                [
                    abs(record.values.get(term, 0.0))
                    if finite(record.values.get(term, 0.0))
                    else 0.0
                    for term in COST_TERMS
                ]
                for record in costs
            ],
            dtype=float,
        )
        denominator = np.sum(matrix, axis=1)
        shares = np.divide(
            100.0 * matrix,
            denominator[:, None],
            out=np.zeros_like(matrix),
            where=denominator[:, None] > 1.0e-12,
        )
        x = _relative_array(motion, cost_times)
        if len(costs) >= 2:
            axes[3].stackplot(
                x,
                *[shares[:, index] for index in range(len(COST_TERMS))],
                labels=COST_TERMS,
                colors=[color_by_term[term] for term in COST_TERMS],
                alpha=0.82,
            )
        else:
            bottom = 0.0
            for index, term in enumerate(COST_TERMS):
                axes[3].bar(
                    x[0],
                    shares[0, index],
                    bottom=bottom,
                    width=0.02,
                    color=color_by_term[term],
                    label=term,
                )
                bottom += shares[0, index]
        axes[3].set_ylim(0.0, 100.0)
        axes[3].legend(ncol=4, loc="upper left")
    else:
        _annotate_empty(axes[3], "No cost data; shares unavailable")
    axes[3].set_ylabel("Absolute share [%]")
    axes[3].set_title(
        "Absolute-value share (approximate diagnostic, not root-cause attribution)"
    )

    a0_records = [
        record
        for record in data.audits
        if finite(record.decision_sec)
        and motion.origin_sec <= record.decision_sec <= motion.end_sec
        and finite(record.a0)
    ]
    frequencies, amplitudes = compute_spectrum(
        [record.decision_sec for record in a0_records],
        [record.a0 for record in a0_records],
    )
    axes[4].axvspan(4.5, 5.5, color=COLORS["yellow"], alpha=0.35, label="4.5–5.5 Hz")
    if frequencies.size:
        axes[4].plot(
            frequencies, amplitudes, color=COLORS["blue"], label="a0 amplitude"
        )
        nyquist = float(frequencies[-1])
        axes[4].set_xlim(0.0, min(15.0, nyquist) if nyquist > 0.0 else 15.0)
        axes[4].legend(loc="upper right")
    else:
        axes[4].set_xlim(0.0, 15.0)
        _annotate_empty(axes[4], "At least four timestamped a0 samples are required")
    axes[4].set_xlabel("Frequency [Hz]")
    axes[4].set_ylabel("Amplitude [m/s²]")
    axes[4].set_title("a0 spectrum over the common motion window")

    figure.text(
        0.01,
        0.002,
        "Published cost terms do not exactly reproduce acados stage and "
        "terminal scaling; use trends only.",
        fontsize=8,
        color=COLORS["gray"],
    )
    _decorate_time_axes(figure, list(axes[:4]), motion, events)
    return _save(figure, output, dpi)


def render_diagnostics(
    data: DiagnosticData,
    output_dir: Path,
    *,
    dpi: int = 150,
    strong_a0_threshold: float = 0.5,
    odom_gap_sec: float = 0.080,
) -> List[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    motion = determine_motion_axis(data)
    events = build_event_markers(data.audits, strong_a0_threshold)
    with plt.rc_context(PLOT_STYLE):
        paths = [
            plot_timing(data, motion, events, output_dir / FIGURE_NAMES[0], dpi),
            plot_command_chain(data, motion, events, output_dir / FIGURE_NAMES[1], dpi),
            plot_command_actual(
                data, motion, events, output_dir / FIGURE_NAMES[2], dpi
            ),
            plot_prediction(
                data,
                motion,
                events,
                output_dir / FIGURE_NAMES[3],
                dpi,
                odom_gap_sec,
            ),
            plot_liquid(data, motion, events, output_dir / FIGURE_NAMES[4], dpi),
            plot_cost_frequency(
                data, motion, events, output_dir / FIGURE_NAMES[5], dpi
            ),
        ]
    return paths


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS1 bag to analyze")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("diagnostic_plots"),
        help="directory for the six fixed PNG files (default: %(default)s)",
    )
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("--strong-a0-threshold", type=float, default=0.5)
    parser.add_argument("--odom-interpolation-max-gap-ms", type=float, default=80.0)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    bag_path = args.bag.expanduser().resolve()
    if not bag_path.is_file():
        print(
            "[plot_spmpc_full_da_diagnostics][ERR] bag 不存在: {}".format(bag_path),
            file=sys.stderr,
        )
        return 2
    if (
        args.dpi < 72
        or args.strong_a0_threshold < 0.0
        or args.odom_interpolation_max_gap_ms <= 0.0
    ):
        print("[plot_spmpc_full_da_diagnostics][ERR] 非法绘图参数", file=sys.stderr)
        return 2
    try:
        data = load_bag(bag_path)
        outputs = render_diagnostics(
            data,
            args.output_dir.expanduser().resolve(),
            dpi=args.dpi,
            strong_a0_threshold=args.strong_a0_threshold,
            odom_gap_sec=args.odom_interpolation_max_gap_ms / 1000.0,
        )
    except Exception as exc:  # keep smoke failure reporting concise
        print("[plot_spmpc_full_da_diagnostics][ERR] {}".format(exc), file=sys.stderr)
        return 1
    for warning in data.warnings:
        print(
            "[plot_spmpc_full_da_diagnostics][WARN] {}".format(warning), file=sys.stderr
        )
    print(
        "[plot_spmpc_full_da_diagnostics] wrote {} fixed figures to {}".format(
            len(outputs), outputs[0].parent
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
