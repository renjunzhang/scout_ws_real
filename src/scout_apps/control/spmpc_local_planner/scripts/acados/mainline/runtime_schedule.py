"""Pure-Python fixed-width runtime delay schedule compiler.

The implementation mirrors ``discrete_delay_queue.h`` and has no solver,
ROS, CasADi, or legacy-contract dependency. Capacity is validated before any
runtime value is consumed; selector widths then come from that pinned typed
snapshot.
"""

from __future__ import annotations

import math
from dataclasses import InitVar, dataclass
from fractions import Fraction
from typing import Any

from .development_capacity import (
    EXECUTION_SUBSEGMENT_SLOTS,
    DevelopmentCapacityContract,
    DevelopmentCapacityError,
    require_pinned_development_capacity,
)
from .identity import sha256_json

RUNTIME_SCHEDULE_SCHEMA_VERSION = "spmpc_mainline_runtime_fractional_delay_schedule_v1"
_CONSTRUCTION_TOKEN = object()


class RuntimeScheduleError(ValueError):
    """Raised when an input or canonical schedule is invalid."""


def _float(value: Any, label: str, *, nonnegative: bool = False) -> float:
    if type(value) is not float:
        raise RuntimeScheduleError(f"{label} must be a float")
    result = 0.0 if value == 0.0 else value
    if not math.isfinite(result) or (nonnegative and result < 0.0):
        raise RuntimeScheduleError(f"{label} must be finite and valid")
    return result


def _equal(left: Any, right: Any) -> bool:
    """Strictly compare canonical values (bool is not an integer alias)."""

    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(
            _equal(left[key], right[key]) for key in left
        )
    if type(left) in (list, tuple):
        return len(left) == len(right) and all(
            _equal(a, b) for a, b in zip(left, right)
        )
    if type(left) is float and left == 0.0 and right == 0.0:
        return math.copysign(1.0, left) == math.copysign(1.0, right)
    return bool(left == right)


def _object(value: Any, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != keys:
        raise RuntimeScheduleError(f"{label} keys do not match the v1 schema")
    return value


def _capacity(value: Any) -> DevelopmentCapacityContract:
    try:
        return require_pinned_development_capacity(value)
    except DevelopmentCapacityError as exc:
        raise RuntimeScheduleError(str(exc)) from exc


def _dt(capacity: DevelopmentCapacityContract) -> float:
    period = capacity.release_period_sec
    if type(period) is not Fraction or period <= 0:
        raise RuntimeScheduleError("capacity release period is invalid")
    result = float(period)
    if not math.isfinite(result) or result <= 0.0:
        raise RuntimeScheduleError("capacity release period is not finite")
    return result


def _one_hot(width: int, selected: int) -> tuple[float, ...]:
    if selected < 0 or selected >= width:
        raise RuntimeScheduleError("selector index exceeds pinned width")
    return tuple(1.0 if index == selected else 0.0 for index in range(width))


def _channel(
    delay_sec: float,
    maximum_delay_sec: Fraction,
    width: int,
    dt_sec: float,
    snap_sec: float,
    label: str,
) -> tuple[int, float, tuple[float, ...], tuple[tuple[float, ...], ...]]:
    """Compile one ``ChannelDelaySchedule`` using the C++ queue rules."""

    delay = _float(delay_sec, label, nonnegative=True)
    if delay > float(maximum_delay_sec):
        raise RuntimeScheduleError(f"{label} is outside the pinned delay range")
    ratio = delay / dt_sec
    lower = math.floor(ratio)
    beta = ratio - lower
    snap_ratio = snap_sec / dt_sec
    if beta <= snap_ratio:
        beta = 0.0
    elif 1.0 - beta <= snap_ratio:
        beta = 0.0
        lower += 1.0
    if lower < 0.0 or lower >= float(width):
        raise RuntimeScheduleError(f"{label} exceeds its selector width")
    m = int(lower)
    if beta > 0.0 and m + 1 >= width:
        raise RuntimeScheduleError(
            f"{label} fractional part needs an unavailable selector"
        )
    selectors = tuple(
        _one_hot(width, index) for index in (m + 1 if beta > 0.0 else 0, m, 0)
    )
    switched = beta * dt_sec
    return m, beta, (switched, dt_sec - switched, 0.0), selectors


def _selector_at(
    selectors: tuple[tuple[float, ...], ...], switch_duration: float, left: float
) -> tuple[float, ...]:
    return selectors[0] if left < switch_duration else selectors[1]


def _merge(
    v: tuple[int, float, tuple[float, ...], tuple[tuple[float, ...], ...]],
    omega: tuple[int, float, tuple[float, ...], tuple[tuple[float, ...], ...]],
    dt_sec: float,
    duration_tolerance_sec: float,
) -> tuple[
    tuple[float, float, float],
    tuple[tuple[float, ...], ...],
    tuple[tuple[float, ...], ...],
]:
    """Merge exact switch times into up to three common ZOH segments."""

    v_switch, omega_switch = v[2][0], omega[2][0]
    boundaries = sorted((0.0, v_switch, omega_switch, dt_sec))
    unique: list[float] = []
    for boundary in boundaries:
        if not unique or boundary != unique[-1]:
            unique.append(boundary)
    active_count = len(unique) - 1
    if not 1 <= active_count <= EXECUTION_SUBSEGMENT_SLOTS:
        raise RuntimeScheduleError("delay switch union has an invalid size")

    duration = [0.0] * EXECUTION_SUBSEGMENT_SLOTS
    selector_v = [_one_hot(len(v[3][0]), 0) for _ in duration]
    selector_omega = [_one_hot(len(omega[3][0]), 0) for _ in duration]
    accumulated = 0.0
    for slot in range(active_count):
        left, right = unique[slot], unique[slot + 1]
        if not right > left:
            raise RuntimeScheduleError("delay schedule contains an empty segment")
        segment = dt_sec - accumulated if slot + 1 == active_count else right - left
        if not math.isfinite(segment) or segment <= 0.0:
            raise RuntimeScheduleError("delay segment duration is invalid")
        duration[slot] = segment
        selector_v[slot] = _selector_at(v[3], v_switch, left)
        selector_omega[slot] = _selector_at(omega[3], omega_switch, left)
        accumulated += segment
    if abs(accumulated - dt_sec) > duration_tolerance_sec:
        raise RuntimeScheduleError("merged durations do not sum to dt")
    return tuple(duration), tuple(selector_v), tuple(selector_omega)


@dataclass(frozen=True)
class RuntimeFractionalDelaySchedule:
    """Immutable canonical snapshot emitted by the sole public builder."""

    capacity_contract_sha256: str
    release_frequency_hz: int
    dt_sec: float
    nq_v: int
    nq_omega: int
    delay_v_sec: float
    delay_omega_sec: float
    integer_snap_tolerance_sec: float
    duration_tolerance_sec: float
    m_v: int
    beta_v: float
    m_omega: int
    beta_omega: float
    duration: tuple[float, ...]
    selector_v: tuple[tuple[float, ...], ...]
    selector_omega: tuple[tuple[float, ...], ...]
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _CONSTRUCTION_TOKEN:
            raise RuntimeScheduleError(
                "RuntimeFractionalDelaySchedule requires the pinned builder"
            )

    @property
    def sha256(self) -> str:
        """Hash only the canonical document; no duplicate hash is stored."""

        return sha256_json(self.to_dict())

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible copy with no mutable references."""

        return {
            "schema_version": RUNTIME_SCHEDULE_SCHEMA_VERSION,
            "capacity_identity": {
                "raw_bytes_sha256": self.capacity_contract_sha256,
                "release_frequency_hz": self.release_frequency_hz,
                "period_sec": {
                    "numerator": 1,
                    "denominator": self.release_frequency_hz,
                },
                "v": {"NQ": self.nq_v},
                "omega": {"NQ": self.nq_omega},
            },
            "inputs": {
                "delay_v_sec": self.delay_v_sec,
                "delay_omega_sec": self.delay_omega_sec,
                "integer_snap_tolerance_sec": self.integer_snap_tolerance_sec,
                "duration_tolerance_sec": self.duration_tolerance_sec,
            },
            "channel_v": {"m": self.m_v, "beta": self.beta_v},
            "channel_omega": {"m": self.m_omega, "beta": self.beta_omega},
            "merged_schedule": {
                "duration_sec": list(self.duration),
                "selector_v": [list(slot) for slot in self.selector_v],
                "selector_omega": [list(slot) for slot in self.selector_omega],
            },
        }


def build_runtime_fractional_delay_schedule(
    capacity: DevelopmentCapacityContract,
    delay_v_sec: float,
    delay_omega_sec: float,
    integer_snap_tolerance_sec: float,
    duration_tolerance_sec: float,
) -> RuntimeFractionalDelaySchedule:
    """Build a three-slot schedule from a pinned development capacity."""

    checked = _capacity(capacity)
    dt_sec = _dt(checked)
    snap = _float(
        integer_snap_tolerance_sec, "integer_snap_tolerance_sec", nonnegative=True
    )
    duration_tolerance = _float(
        duration_tolerance_sec, "duration_tolerance_sec", nonnegative=True
    )
    if snap >= 0.5 * dt_sec:
        raise RuntimeScheduleError("integer snap tolerance must be < 0.5*dt")
    if duration_tolerance >= dt_sec:
        raise RuntimeScheduleError("duration tolerance must be < dt")

    v = _channel(
        delay_v_sec, checked.v.l_max_sec, checked.NQ_v, dt_sec, snap, "delay_v_sec"
    )
    omega = _channel(
        delay_omega_sec,
        checked.omega.l_max_sec,
        checked.NQ_omega,
        dt_sec,
        snap,
        "delay_omega_sec",
    )
    duration, selector_v, selector_omega = _merge(v, omega, dt_sec, duration_tolerance)
    return RuntimeFractionalDelaySchedule(
        capacity_contract_sha256=checked.contract_sha256,
        release_frequency_hz=checked.release_frequency_hz,
        dt_sec=dt_sec,
        nq_v=checked.NQ_v,
        nq_omega=checked.NQ_omega,
        delay_v_sec=_float(delay_v_sec, "delay_v_sec", nonnegative=True),
        delay_omega_sec=_float(delay_omega_sec, "delay_omega_sec", nonnegative=True),
        integer_snap_tolerance_sec=snap,
        duration_tolerance_sec=duration_tolerance,
        m_v=v[0],
        beta_v=v[1],
        m_omega=omega[0],
        beta_omega=omega[1],
        duration=duration,
        selector_v=selector_v,
        selector_omega=selector_omega,
        _construction_token=_CONSTRUCTION_TOKEN,
    )


def runtime_fractional_delay_schedule_from_dict(
    value: Any, capacity: DevelopmentCapacityContract
) -> RuntimeFractionalDelaySchedule:
    """Strictly rebuild a serialized schedule against pinned capacity."""

    _object(
        value,
        {
            "schema_version",
            "capacity_identity",
            "inputs",
            "channel_v",
            "channel_omega",
            "merged_schedule",
        },
        "runtime schedule",
    )
    inputs = _object(
        value["inputs"],
        {
            "delay_v_sec",
            "delay_omega_sec",
            "integer_snap_tolerance_sec",
            "duration_tolerance_sec",
        },
        "runtime schedule.inputs",
    )
    arguments = tuple(
        _float(inputs[key], key, nonnegative=True)
        for key in (
            "delay_v_sec",
            "delay_omega_sec",
            "integer_snap_tolerance_sec",
            "duration_tolerance_sec",
        )
    )
    rebuilt = build_runtime_fractional_delay_schedule(capacity, *arguments)
    if not _equal(value, rebuilt.to_dict()):
        raise RuntimeScheduleError(
            "runtime schedule does not exactly match its canonical expansion"
        )
    return rebuilt


def require_runtime_fractional_delay_schedule(
    value: Any, capacity: DevelopmentCapacityContract
) -> RuntimeFractionalDelaySchedule:
    """Reject force-mutated snapshots before parameter assembly."""

    if type(value) is not RuntimeFractionalDelaySchedule:
        raise RuntimeScheduleError("schedule must be the exact runtime schedule type")
    try:
        expected = build_runtime_fractional_delay_schedule(
            capacity,
            value.delay_v_sec,
            value.delay_omega_sec,
            value.integer_snap_tolerance_sec,
            value.duration_tolerance_sec,
        )
        actual = value.to_dict()
    except (RuntimeScheduleError, TypeError, ValueError, OverflowError) as exc:
        if isinstance(exc, RuntimeScheduleError):
            raise
        raise RuntimeScheduleError("runtime schedule is malformed") from exc
    if not _equal(actual, expected.to_dict()):
        raise RuntimeScheduleError(
            "runtime schedule does not match its canonical expansion"
        )
    return value


__all__ = [
    "RUNTIME_SCHEDULE_SCHEMA_VERSION",
    "RuntimeFractionalDelaySchedule",
    "RuntimeScheduleError",
    "build_runtime_fractional_delay_schedule",
    "require_runtime_fractional_delay_schedule",
    "runtime_fractional_delay_schedule_from_dict",
]
