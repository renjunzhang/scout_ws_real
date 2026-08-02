#!/usr/bin/env python3
"""Pure numerical kernels for liquid-horizon and observer replay.

This module intentionally has no ROS imports, filesystem access, logging, or
global mutable state.  It provides four small, independently testable pieces:

* cubic-Hermite interpolation of ``q = [eta_x, eta_x_dot, eta_y, eta_y_dot]``;
* an exact zero-order-hold step of the forced two-axis modal oscillator;
* strict replay of accepted observer inputs from an already-updated ``q0``;
* high-accuracy replay of piecewise-constant planned ``a/alpha`` controls while
  integrating ``v`` and ``omega`` continuously with RK4.

The observer replay contract is deliberately explicit.  ``ObserverAnchor.q``
is the state *after* the input identified by the anchor's ``update_count`` has
already been consumed.  If that same sample is included as the first replay
sample, it is recognized as one anchor echo and skipped.  It is never applied
twice.  Subsequent samples must have strictly increasing state stamps and
consecutive update counts within one reset epoch.

All times used by the observer API are integer nanoseconds except
``sample_dt_sec``.  This avoids losing ordering precision when ROS epoch-sized
timestamps are converted to floating point.  Planned-horizon times are relative
seconds and do not carry any ROS clock semantics.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


DEFAULT_PLANNED_RK4_SUBSTEP_SEC = 1.0 / 300.0  # 3.333... ms


class ReplayContractError(ValueError):
    """Raised when replay inputs violate their declared temporal contract."""


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ReplayContractError("{} must be finite".format(name))
    return result


def _nonnegative_int(name: str, value: int) -> int:
    result = int(value)
    if result < 0 or result != value:
        raise ReplayContractError("{} must be a nonnegative integer".format(name))
    return result


@dataclass(frozen=True)
class ModalState:
    """Two independent first-mode liquid coordinates and their derivatives."""

    eta_x: float = 0.0
    eta_x_dot: float = 0.0
    eta_y: float = 0.0
    eta_y_dot: float = 0.0

    def __post_init__(self) -> None:
        for name, value in zip(
            ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot"), self.as_tuple()
        ):
            _finite(name, value)

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.eta_x, self.eta_x_dot, self.eta_y, self.eta_y_dot)

    @classmethod
    def zero(cls) -> "ModalState":
        return cls()


@dataclass(frozen=True)
class ModalParameters:
    """Continuous modal parameters shared by observer and planned replay.

    The per-axis dynamics are

    ``eta_ddot + two_zeta_omega_n * eta_dot + omega_n_sq * eta = -kappa * a``.
    """

    two_zeta_omega_n: float
    omega_n_sq: float
    kappa_x: float = 1.0
    kappa_y: float = 1.0

    def __post_init__(self) -> None:
        damping = _finite("two_zeta_omega_n", self.two_zeta_omega_n)
        stiffness = _finite("omega_n_sq", self.omega_n_sq)
        _finite("kappa_x", self.kappa_x)
        _finite("kappa_y", self.kappa_y)
        if damping < 0.0:
            raise ReplayContractError("two_zeta_omega_n must be nonnegative")
        if stiffness <= 0.0:
            raise ReplayContractError("omega_n_sq must be positive")


def cubic_hermite_q(
    left: ModalState,
    right: ModalState,
    tau_sec: float,
    interval_sec: float,
) -> ModalState:
    """Interpolate modal position and velocity between two state nodes.

    ``tau_sec`` is measured from ``left`` and must lie in the closed interval
    ``[0, interval_sec]``.  The endpoint modal velocities are used as Hermite
    derivatives, so both position and velocity are reproduced exactly at each
    node.  The function does not silently clamp an out-of-range query.
    """

    duration = _finite("interval_sec", interval_sec)
    tau = _finite("tau_sec", tau_sec)
    if duration <= 0.0:
        raise ReplayContractError("interval_sec must be positive")
    tolerance = 1.0e-12 * max(1.0, duration)
    if tau < -tolerance or tau > duration + tolerance:
        raise ReplayContractError("tau_sec lies outside the Hermite interval")
    tau = min(duration, max(0.0, tau))
    s = tau / duration
    s2 = s * s
    s3 = s2 * s

    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    dh00 = (6.0 * s2 - 6.0 * s) / duration
    dh10 = 3.0 * s2 - 4.0 * s + 1.0
    dh01 = (-6.0 * s2 + 6.0 * s) / duration
    dh11 = 3.0 * s2 - 2.0 * s

    def interpolate_axis(p0: float, v0: float, p1: float, v1: float) -> Tuple[float, float]:
        position = h00 * p0 + h10 * duration * v0 + h01 * p1 + h11 * duration * v1
        velocity = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1
        return position, velocity

    eta_x, eta_x_dot = interpolate_axis(
        left.eta_x, left.eta_x_dot, right.eta_x, right.eta_x_dot
    )
    eta_y, eta_y_dot = interpolate_axis(
        left.eta_y, left.eta_y_dot, right.eta_y, right.eta_y_dot
    )
    return ModalState(eta_x, eta_x_dot, eta_y, eta_y_dot)


def sample_cubic_hermite_nodes(
    nodes: Sequence[ModalState], node_dt_sec: float, query_sec: float
) -> ModalState:
    """Sample a uniform modal-node sequence without clamping its time range."""

    if not nodes:
        raise ReplayContractError("nodes must not be empty")
    dt = _finite("node_dt_sec", node_dt_sec)
    query = _finite("query_sec", query_sec)
    if dt <= 0.0:
        raise ReplayContractError("node_dt_sec must be positive")
    duration = float(len(nodes) - 1) * dt
    tolerance = 1.0e-12 * max(1.0, duration)
    if query < -tolerance or query > duration + tolerance:
        raise ReplayContractError("query_sec lies outside the node horizon")
    query = min(duration, max(0.0, query))
    if len(nodes) == 1 or query == duration:
        return nodes[-1]
    lower = min(len(nodes) - 2, int(math.floor(query / dt)))
    return cubic_hermite_q(nodes[lower], nodes[lower + 1], query - lower * dt, dt)


def _homogeneous_transition(
    duration_sec: float, damping: float, stiffness: float
) -> Tuple[float, float, float, float]:
    """Return exp(A*dt) for A=[[0,1],[-stiffness,-damping]]."""

    duration = float(duration_sec)
    half_damping = 0.5 * damping
    discriminant = stiffness - half_damping * half_damping
    scale = max(1.0, stiffness, half_damping * half_damping)
    critical_tolerance = 1.0e-12 * scale

    if discriminant > critical_tolerance:
        omega_d = math.sqrt(discriminant)
        decay = math.exp(-half_damping * duration)
        cosine = math.cos(omega_d * duration)
        sine_over_omega = math.sin(omega_d * duration) / omega_d
        common = decay * sine_over_omega
        cosine_term = decay * cosine
        return (
            cosine_term + half_damping * common,
            common,
            -stiffness * common,
            cosine_term - half_damping * common,
        )

    if discriminant < -critical_tolerance:
        rate = math.sqrt(-discriminant)
        # This form avoids overflow from exp(-a*t)*cosh(rate*t).
        slow = math.exp((-half_damping + rate) * duration)
        fast = math.exp((-half_damping - rate) * duration)
        cosine_term = 0.5 * (slow + fast)
        sine_over_rate = 0.5 * (slow - fast) / rate
        return (
            cosine_term + half_damping * sine_over_rate,
            sine_over_rate,
            -stiffness * sine_over_rate,
            cosine_term - half_damping * sine_over_rate,
        )

    decay = math.exp(-half_damping * duration)
    return (
        decay * (1.0 + half_damping * duration),
        decay * duration,
        -decay * stiffness * duration,
        decay * (1.0 - half_damping * duration),
    )


def exact_zoh_forced_modal_step(
    state: ModalState,
    ax: float,
    ay: float,
    dt_sec: float,
    parameters: ModalParameters,
) -> ModalState:
    """Advance both modal axes exactly for constant acceleration inputs.

    This is the analytical zero-order-hold solution of the continuous linear
    oscillator, not an Euler or RK approximation.  It supports underdamped,
    critically damped, and overdamped parameters.  A zero duration returns the
    input state unchanged.
    """

    duration = _finite("dt_sec", dt_sec)
    input_x = _finite("ax", ax)
    input_y = _finite("ay", ay)
    if duration < 0.0:
        raise ReplayContractError("dt_sec must be nonnegative")
    if duration == 0.0:
        return state

    damping = parameters.two_zeta_omega_n
    stiffness = parameters.omega_n_sq
    phi11, phi12, phi21, phi22 = _homogeneous_transition(
        duration, damping, stiffness
    )

    def advance(position: float, velocity: float, excitation: float, gain: float) -> Tuple[float, float]:
        equilibrium = -gain * excitation / stiffness
        shifted = position - equilibrium
        return (
            equilibrium + phi11 * shifted + phi12 * velocity,
            phi21 * shifted + phi22 * velocity,
        )

    eta_x, eta_x_dot = advance(state.eta_x, state.eta_x_dot, input_x, parameters.kappa_x)
    eta_y, eta_y_dot = advance(state.eta_y, state.eta_y_dot, input_y, parameters.kappa_y)
    return ModalState(eta_x, eta_x_dot, eta_y, eta_y_dot)


@dataclass(frozen=True)
class ObserverAnchor:
    """An accepted observer state after ``update_count`` has been consumed."""

    state_stamp_ns: int
    update_count: int
    reset_epoch: int
    q: ModalState

    def __post_init__(self) -> None:
        _nonnegative_int("state_stamp_ns", self.state_stamp_ns)
        _nonnegative_int("update_count", self.update_count)
        _nonnegative_int("reset_epoch", self.reset_epoch)


@dataclass(frozen=True)
class ObserverInputSample:
    """One accepted processed-input event as published by the observer debug path."""

    state_stamp_ns: int
    sample_dt_sec: float
    update_count: int
    reset_epoch: int
    ax: float
    ay: float

    def __post_init__(self) -> None:
        _nonnegative_int("state_stamp_ns", self.state_stamp_ns)
        _nonnegative_int("update_count", self.update_count)
        _nonnegative_int("reset_epoch", self.reset_epoch)
        if _finite("sample_dt_sec", self.sample_dt_sec) <= 0.0:
            raise ReplayContractError("sample_dt_sec must be positive")
        _finite("ax", self.ax)
        _finite("ay", self.ay)


@dataclass(frozen=True)
class ObserverReplayPoint:
    """One state in an observer replay trace."""

    state_stamp_ns: int
    update_count: int
    reset_epoch: int
    q: ModalState
    sample_dt_sec: Optional[float] = None
    ax: Optional[float] = None
    ay: Optional[float] = None
    epoch_reset_applied: bool = False


@dataclass(frozen=True)
class ObserverReplayResult:
    """Strict observer replay trace and contract bookkeeping."""

    points: Tuple[ObserverReplayPoint, ...]
    skipped_anchor_echo: bool
    epoch_reset_count: int


def replay_observer_inputs(
    anchor: ObserverAnchor,
    samples: Sequence[ObserverInputSample],
    parameters: ModalParameters,
    *,
    state_dt_tolerance_sec: float = 1.0e-7,
    allow_epoch_reset: bool = False,
) -> ObserverReplayResult:
    """Replay accepted observer inputs exactly from an already-updated anchor.

    Within an epoch this function requires all of the following:

    * input order is already strictly increasing; it is never silently sorted;
    * ``update_count`` increments by exactly one;
    * ``state_stamp`` increments by the published ``sample_dt_sec`` within the
      caller-selected tolerance;
    * an optional first event equal to the anchor is skipped exactly once, so
      the input that produced ``q0`` cannot be applied twice.

    Epoch changes fail closed by default.  With ``allow_epoch_reset=True``, the
    software reset behavior is reproduced: the modal state is cleared before
    the first accepted input in the new epoch, whose update count must be one.
    The gap before that event is not required to equal its sample interval.
    """

    tolerance = _finite("state_dt_tolerance_sec", state_dt_tolerance_sec)
    if tolerance < 0.0:
        raise ReplayContractError("state_dt_tolerance_sec must be nonnegative")

    points = [
        ObserverReplayPoint(
            anchor.state_stamp_ns,
            anchor.update_count,
            anchor.reset_epoch,
            anchor.q,
        )
    ]
    current_state = anchor.q
    current_stamp = anchor.state_stamp_ns
    current_count = anchor.update_count
    current_epoch = anchor.reset_epoch
    skipped_anchor_echo = False
    reset_count = 0
    applied_count = 0

    for sample in samples:
        is_anchor_echo = (
            sample.state_stamp_ns == anchor.state_stamp_ns
            and sample.update_count == anchor.update_count
            and sample.reset_epoch == anchor.reset_epoch
        )
        if is_anchor_echo:
            if applied_count != 0 or skipped_anchor_echo:
                raise ReplayContractError("anchor input echo appears more than once or out of order")
            skipped_anchor_echo = True
            continue

        if sample.reset_epoch < current_epoch:
            raise ReplayContractError("observer reset_epoch moved backwards")

        epoch_changed = sample.reset_epoch != current_epoch
        if epoch_changed:
            if not allow_epoch_reset:
                raise ReplayContractError("observer reset_epoch changed during strict replay")
            if sample.update_count != 1:
                raise ReplayContractError("first input after an epoch reset must have update_count=1")
            if sample.state_stamp_ns <= current_stamp:
                raise ReplayContractError("state_stamp must increase across an epoch reset")
            current_state = ModalState.zero()
            current_epoch = sample.reset_epoch
            reset_count += 1
        else:
            if sample.update_count != current_count + 1:
                raise ReplayContractError(
                    "observer update_count is not consecutive: {} after {}".format(
                        sample.update_count, current_count
                    )
                )
            if sample.state_stamp_ns <= current_stamp:
                raise ReplayContractError("observer state_stamp is not strictly increasing")
            stamp_dt = float(sample.state_stamp_ns - current_stamp) * 1.0e-9
            if abs(stamp_dt - sample.sample_dt_sec) > tolerance:
                raise ReplayContractError(
                    "sample_dt_sec does not match the state_stamp increment: "
                    "{:.12g} vs {:.12g}".format(sample.sample_dt_sec, stamp_dt)
                )

        current_state = exact_zoh_forced_modal_step(
            current_state, sample.ax, sample.ay, sample.sample_dt_sec, parameters
        )
        current_stamp = sample.state_stamp_ns
        current_count = sample.update_count
        current_epoch = sample.reset_epoch
        applied_count += 1
        points.append(
            ObserverReplayPoint(
                current_stamp,
                current_count,
                current_epoch,
                current_state,
                sample.sample_dt_sec,
                sample.ax,
                sample.ay,
                epoch_changed,
            )
        )

    return ObserverReplayResult(tuple(points), skipped_anchor_echo, reset_count)


def sample_observer_replay(
    points: Sequence[ObserverReplayPoint],
    query_stamp_ns: int,
    parameters: ModalParameters,
) -> ModalState:
    """Sample a strict observer replay at an integer-nanosecond target.

    An endpoint returns its already-updated state.  Between two endpoints, the
    input attached to the right endpoint is applied to the left state for the
    partial interval.  This matches the online software's ZOH convention; it
    is a software-time reconstruction, not a claim about the acceleration's
    physical effective phase.
    """

    if not points:
        raise ReplayContractError("points must not be empty")
    query = _nonnegative_int("query_stamp_ns", query_stamp_ns)
    stamps = [point.state_stamp_ns for point in points]
    if any(right <= left for left, right in zip(stamps, stamps[1:])):
        raise ReplayContractError("observer replay points are not strictly ordered")
    if query < stamps[0] or query > stamps[-1]:
        raise ReplayContractError("query_stamp_ns lies outside the observer replay")
    index = bisect.bisect_left(stamps, query)
    if index < len(points) and stamps[index] == query:
        return points[index].q
    left = points[index - 1]
    right = points[index]
    if right.ax is None or right.ay is None:
        raise ReplayContractError("right replay point has no applied input")
    partial_dt = float(query - left.state_stamp_ns) * 1.0e-9
    return exact_zoh_forced_modal_step(
        left.q, right.ax, right.ay, partial_dt, parameters
    )


@dataclass(frozen=True)
class PlannedControl:
    """One interval of constant longitudinal acceleration and yaw acceleration."""

    a: float
    alpha: float
    duration_sec: float

    def __post_init__(self) -> None:
        _finite("a", self.a)
        _finite("alpha", self.alpha)
        if _finite("duration_sec", self.duration_sec) <= 0.0:
            raise ReplayContractError("duration_sec must be positive")


@dataclass(frozen=True)
class PlannedReplayPoint:
    """Dense robot/modal state produced by planned-control RK4 replay."""

    time_sec: float
    v: float
    omega: float
    q: ModalState
    control_index: int


def _planned_derivative(
    state: Tuple[float, float, float, float, float, float],
    control: PlannedControl,
    parameters: ModalParameters,
) -> Tuple[float, float, float, float, float, float]:
    v, omega, eta_x, eta_x_dot, eta_y, eta_y_dot = state
    return (
        control.a,
        control.alpha,
        eta_x_dot,
        -parameters.two_zeta_omega_n * eta_x_dot
        - parameters.omega_n_sq * eta_x
        - parameters.kappa_x * control.a,
        eta_y_dot,
        -parameters.two_zeta_omega_n * eta_y_dot
        - parameters.omega_n_sq * eta_y
        - parameters.kappa_y * v * omega,
    )


def _add_scaled(
    left: Tuple[float, ...], right: Tuple[float, ...], scale: float
) -> Tuple[float, ...]:
    return tuple(a + scale * b for a, b in zip(left, right))


def replay_planned_controls(
    initial_q: ModalState,
    initial_v: float,
    initial_omega: float,
    controls: Sequence[PlannedControl],
    parameters: ModalParameters,
    *,
    max_substep_sec: float = DEFAULT_PLANNED_RK4_SUBSTEP_SEC,
) -> Tuple[PlannedReplayPoint, ...]:
    """Replay a planned horizon with continuous robot/modal RK4 dynamics.

    ``a`` and ``alpha`` are constant inside each control interval.  ``v`` and
    ``omega`` remain states and therefore change continuously; lateral modal
    forcing is evaluated as ``v(t) * omega(t)`` at every RK4 stage.  Each
    interval is divided into equal steps no larger than ``max_substep_sec`` and
    ends exactly on its declared boundary.  The default maximum step is
    3.333... ms, one tenth of the current 30 Hz OCP interval.

    The returned tuple includes the initial point with ``control_index=-1`` and
    one point after every RK4 substep.  No wall-clock or ROS timestamp is
    inferred; ``time_sec`` is relative to the horizon origin.
    """

    v0 = _finite("initial_v", initial_v)
    omega0 = _finite("initial_omega", initial_omega)
    max_step = _finite("max_substep_sec", max_substep_sec)
    if max_step <= 0.0:
        raise ReplayContractError("max_substep_sec must be positive")

    state: Tuple[float, float, float, float, float, float] = (
        v0,
        omega0,
        initial_q.eta_x,
        initial_q.eta_x_dot,
        initial_q.eta_y,
        initial_q.eta_y_dot,
    )
    elapsed = 0.0
    output = [PlannedReplayPoint(0.0, v0, omega0, initial_q, -1)]

    for control_index, control in enumerate(controls):
        step_count = max(1, int(math.ceil(control.duration_sec / max_step)))
        step = control.duration_sec / float(step_count)
        interval_end = elapsed + control.duration_sec
        for substep_index in range(step_count):
            k1 = _planned_derivative(state, control, parameters)
            k2 = _planned_derivative(_add_scaled(state, k1, 0.5 * step), control, parameters)
            k3 = _planned_derivative(_add_scaled(state, k2, 0.5 * step), control, parameters)
            k4 = _planned_derivative(_add_scaled(state, k3, step), control, parameters)
            state = tuple(
                value
                + step
                * (slope1 + 2.0 * slope2 + 2.0 * slope3 + slope4)
                / 6.0
                for value, slope1, slope2, slope3, slope4 in zip(state, k1, k2, k3, k4)
            )
            elapsed = (
                interval_end
                if substep_index + 1 == step_count
                else elapsed + step
            )
            q = ModalState(state[2], state[3], state[4], state[5])
            output.append(
                PlannedReplayPoint(elapsed, state[0], state[1], q, control_index)
            )

    return tuple(output)


def sample_planned_replay(
    points: Sequence[PlannedReplayPoint], query_sec: float
) -> PlannedReplayPoint:
    """Sample a dense planned replay, using modal cubic Hermite interpolation."""

    if not points:
        raise ReplayContractError("points must not be empty")
    query = _finite("query_sec", query_sec)
    stamps = [point.time_sec for point in points]
    if any(right <= left for left, right in zip(stamps, stamps[1:])):
        raise ReplayContractError("planned replay points are not strictly ordered")
    tolerance = 1.0e-12 * max(1.0, stamps[-1])
    if query < stamps[0] - tolerance or query > stamps[-1] + tolerance:
        raise ReplayContractError("query_sec lies outside the planned replay")
    query = min(stamps[-1], max(stamps[0], query))
    index = bisect.bisect_left(stamps, query)
    if index < len(points) and abs(stamps[index] - query) <= tolerance:
        return points[index]
    left = points[index - 1]
    right = points[index]
    interval = right.time_sec - left.time_sec
    weight = (query - left.time_sec) / interval
    q = cubic_hermite_q(left.q, right.q, query - left.time_sec, interval)
    return PlannedReplayPoint(
        query,
        left.v + weight * (right.v - left.v),
        left.omega + weight * (right.omega - left.omega),
        q,
        right.control_index,
    )


__all__ = (
    "DEFAULT_PLANNED_RK4_SUBSTEP_SEC",
    "ModalParameters",
    "ModalState",
    "ObserverAnchor",
    "ObserverInputSample",
    "ObserverReplayPoint",
    "ObserverReplayResult",
    "PlannedControl",
    "PlannedReplayPoint",
    "ReplayContractError",
    "cubic_hermite_q",
    "exact_zoh_forced_modal_step",
    "replay_observer_inputs",
    "replay_planned_controls",
    "sample_observer_replay",
    "sample_cubic_hermite_nodes",
    "sample_planned_replay",
)
