#!/usr/bin/env python3
"""Typed Python facade for the shared C++ liquid/observer replay core.

The public dataclasses and functions remain import-compatible with the former
pure-Python implementation. Numerical propagation and contract validation are
owned by ``spmpc_analysis_replay`` and reached through its narrow C ABI. Build
``spmpc_analysis_replay_c_api`` before running analyses that import this module.
Neither analysis library is part of the robot runtime installation.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import math
import os
from pathlib import Path
import shutil
from typing import Optional, Sequence, Tuple


DEFAULT_PLANNED_RK4_SUBSTEP_SEC = 1.0 / 300.0
_ERROR_CAPACITY = 1024


class ReplayContractError(ValueError):
    """Raised when the shared C++ core rejects a replay contract."""


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ReplayContractError(f"{name} must be finite")
    return result


def _nonnegative_int(name: str, value: int) -> int:
    result = int(value)
    if result < 0 or result != value:
        raise ReplayContractError(f"{name} must be a nonnegative integer")
    if result > (1 << 64) - 1:
        raise ReplayContractError(f"{name} exceeds uint64")
    return result


@dataclass(frozen=True)
class ModalState:
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


@dataclass(frozen=True)
class ObserverAnchor:
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
    points: Tuple[ObserverReplayPoint, ...]
    skipped_anchor_echo: bool
    epoch_reset_count: int


@dataclass(frozen=True)
class PlannedControl:
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
    time_sec: float
    v: float
    omega: float
    q: ModalState
    control_index: int


class _CModalState(ctypes.Structure):
    _fields_ = [
        ("eta_x", ctypes.c_double),
        ("eta_x_dot", ctypes.c_double),
        ("eta_y", ctypes.c_double),
        ("eta_y_dot", ctypes.c_double),
    ]


class _CModalParameters(ctypes.Structure):
    _fields_ = [
        ("two_zeta_omega_n", ctypes.c_double),
        ("omega_n_sq", ctypes.c_double),
        ("kappa_x", ctypes.c_double),
        ("kappa_y", ctypes.c_double),
    ]


class _CObserverAnchor(ctypes.Structure):
    _fields_ = [
        ("state_stamp_ns", ctypes.c_uint64),
        ("update_count", ctypes.c_uint64),
        ("reset_epoch", ctypes.c_uint64),
        ("state", _CModalState),
    ]


class _CObserverInput(ctypes.Structure):
    _fields_ = [
        ("state_stamp_ns", ctypes.c_uint64),
        ("sample_dt_sec", ctypes.c_double),
        ("update_count", ctypes.c_uint64),
        ("reset_epoch", ctypes.c_uint64),
        ("ax", ctypes.c_double),
        ("ay", ctypes.c_double),
    ]


class _CObserverPoint(ctypes.Structure):
    _fields_ = [
        ("state_stamp_ns", ctypes.c_uint64),
        ("update_count", ctypes.c_uint64),
        ("reset_epoch", ctypes.c_uint64),
        ("state", _CModalState),
        ("has_input", ctypes.c_int),
        ("sample_dt_sec", ctypes.c_double),
        ("ax", ctypes.c_double),
        ("ay", ctypes.c_double),
        ("epoch_reset_applied", ctypes.c_int),
    ]


class _CPlannedControl(ctypes.Structure):
    _fields_ = [
        ("a", ctypes.c_double),
        ("alpha", ctypes.c_double),
        ("duration_sec", ctypes.c_double),
    ]


class _CPlannedPoint(ctypes.Structure):
    _fields_ = [
        ("time_sec", ctypes.c_double),
        ("v", ctypes.c_double),
        ("omega", ctypes.c_double),
        ("state", _CModalState),
        ("control_index", ctypes.c_int),
    ]


_LIBRARY = None


def _library_path() -> Path:
    configured = os.environ.get("SPMPC_ANALYSIS_REPLAY_C_API", "").strip()
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    discovered = shutil.which("libspmpc_analysis_replay_c_api.so")
    if discovered:
        candidates.append(Path(discovered))
    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / "devel/lib/libspmpc_analysis_replay_c_api.so")
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise ReplayContractError(
        "C++ replay library is unavailable; build spmpc_analysis_replay_c_api "
        "or set SPMPC_ANALYSIS_REPLAY_C_API"
    )


def _configure_library(library):
    state_p = ctypes.POINTER(_CModalState)
    params_p = ctypes.POINTER(_CModalParameters)
    char_p = ctypes.POINTER(ctypes.c_char)
    observer_point_p = ctypes.POINTER(_CObserverPoint)
    planned_point_p = ctypes.POINTER(_CPlannedPoint)
    library.spmpc_modal_cubic_hermite.argtypes = [
        state_p, state_p, ctypes.c_double, ctypes.c_double, state_p,
        char_p, ctypes.c_size_t,
    ]
    library.spmpc_modal_sample_nodes.argtypes = [
        state_p, ctypes.c_size_t, ctypes.c_double, ctypes.c_double, state_p,
        char_p, ctypes.c_size_t,
    ]
    library.spmpc_modal_exact_zoh_step.argtypes = [
        state_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, params_p,
        state_p, char_p, ctypes.c_size_t,
    ]
    library.spmpc_modal_replay_observer.argtypes = [
        ctypes.POINTER(_CObserverAnchor), ctypes.POINTER(_CObserverInput),
        ctypes.c_size_t, params_p, ctypes.c_double, ctypes.c_int,
        observer_point_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_size_t),
        ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_size_t),
        char_p, ctypes.c_size_t,
    ]
    library.spmpc_modal_sample_observer.argtypes = [
        observer_point_p, ctypes.c_size_t, ctypes.c_uint64, params_p, state_p,
        char_p, ctypes.c_size_t,
    ]
    library.spmpc_modal_replay_planned.argtypes = [
        state_p, ctypes.c_double, ctypes.c_double,
        ctypes.POINTER(_CPlannedControl), ctypes.c_size_t, params_p,
        ctypes.c_double, planned_point_p, ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t), char_p, ctypes.c_size_t,
    ]
    library.spmpc_modal_sample_planned.argtypes = [
        planned_point_p, ctypes.c_size_t, ctypes.c_double, planned_point_p,
        char_p, ctypes.c_size_t,
    ]
    for name in (
        "spmpc_modal_cubic_hermite", "spmpc_modal_sample_nodes",
        "spmpc_modal_exact_zoh_step", "spmpc_modal_replay_observer",
        "spmpc_modal_sample_observer", "spmpc_modal_replay_planned",
        "spmpc_modal_sample_planned",
    ):
        getattr(library, name).restype = ctypes.c_int
    return library


def _lib():
    global _LIBRARY
    if _LIBRARY is None:
        try:
            _LIBRARY = _configure_library(ctypes.CDLL(str(_library_path())))
        except OSError as exc:
            raise ReplayContractError("failed to load C++ replay library") from exc
    return _LIBRARY


def _c_state(state: ModalState) -> _CModalState:
    return _CModalState(*state.as_tuple())


def _state(state: _CModalState) -> ModalState:
    return ModalState(state.eta_x, state.eta_x_dot, state.eta_y, state.eta_y_dot)


def _c_parameters(parameters: ModalParameters) -> _CModalParameters:
    return _CModalParameters(
        parameters.two_zeta_omega_n, parameters.omega_n_sq,
        parameters.kappa_x, parameters.kappa_y,
    )


def _check(code: int, error) -> None:
    if code != 0:
        detail = error.value.decode("utf-8", errors="replace")
        raise ReplayContractError(detail or f"C++ replay failed with code {code}")


def cubic_hermite_q(
    left: ModalState, right: ModalState, tau_sec: float, interval_sec: float
) -> ModalState:
    c_left, c_right = _c_state(left), _c_state(right)
    output, error = _CModalState(), ctypes.create_string_buffer(_ERROR_CAPACITY)
    code = _lib().spmpc_modal_cubic_hermite(
        ctypes.byref(c_left), ctypes.byref(c_right), float(tau_sec),
        float(interval_sec), ctypes.byref(output), error, len(error)
    )
    _check(code, error)
    return _state(output)


def sample_cubic_hermite_nodes(
    nodes: Sequence[ModalState], node_dt_sec: float, query_sec: float
) -> ModalState:
    values = (_CModalState * len(nodes))(*(_c_state(node) for node in nodes))
    output, error = _CModalState(), ctypes.create_string_buffer(_ERROR_CAPACITY)
    code = _lib().spmpc_modal_sample_nodes(
        values, len(nodes), float(node_dt_sec), float(query_sec),
        ctypes.byref(output), error, len(error)
    )
    _check(code, error)
    return _state(output)


def exact_zoh_forced_modal_step(
    state: ModalState,
    ax: float,
    ay: float,
    dt_sec: float,
    parameters: ModalParameters,
) -> ModalState:
    c_state, c_parameters = _c_state(state), _c_parameters(parameters)
    output, error = _CModalState(), ctypes.create_string_buffer(_ERROR_CAPACITY)
    code = _lib().spmpc_modal_exact_zoh_step(
        ctypes.byref(c_state), float(ax), float(ay), float(dt_sec),
        ctypes.byref(c_parameters), ctypes.byref(output), error, len(error)
    )
    _check(code, error)
    return _state(output)


def replay_observer_inputs(
    anchor: ObserverAnchor,
    samples: Sequence[ObserverInputSample],
    parameters: ModalParameters,
    *,
    state_dt_tolerance_sec: float = 1.0e-7,
    allow_epoch_reset: bool = False,
) -> ObserverReplayResult:
    c_anchor = _CObserverAnchor(
        anchor.state_stamp_ns, anchor.update_count, anchor.reset_epoch,
        _c_state(anchor.q),
    )
    inputs = (_CObserverInput * len(samples))(*(
        _CObserverInput(
            sample.state_stamp_ns, sample.sample_dt_sec, sample.update_count,
            sample.reset_epoch, sample.ax, sample.ay,
        )
        for sample in samples
    ))
    output = (_CObserverPoint * (len(samples) + 1))()
    output_count, skipped, resets = ctypes.c_size_t(), ctypes.c_int(), ctypes.c_size_t()
    c_parameters = _c_parameters(parameters)
    error = ctypes.create_string_buffer(_ERROR_CAPACITY)
    code = _lib().spmpc_modal_replay_observer(
        ctypes.byref(c_anchor), inputs, len(samples), ctypes.byref(c_parameters),
        float(state_dt_tolerance_sec), int(bool(allow_epoch_reset)), output,
        len(output), ctypes.byref(output_count), ctypes.byref(skipped),
        ctypes.byref(resets), error, len(error)
    )
    _check(code, error)
    points = tuple(
        ObserverReplayPoint(
            point.state_stamp_ns, point.update_count, point.reset_epoch,
            _state(point.state),
            point.sample_dt_sec if point.has_input else None,
            point.ax if point.has_input else None,
            point.ay if point.has_input else None,
            bool(point.epoch_reset_applied),
        )
        for point in output[:output_count.value]
    )
    return ObserverReplayResult(points, bool(skipped.value), resets.value)


def _c_observer_points(points: Sequence[ObserverReplayPoint]):
    return (_CObserverPoint * len(points))(*(
        _CObserverPoint(
            point.state_stamp_ns, point.update_count, point.reset_epoch,
            _c_state(point.q), int(point.ax is not None and point.ay is not None),
            0.0 if point.sample_dt_sec is None else point.sample_dt_sec,
            0.0 if point.ax is None else point.ax,
            0.0 if point.ay is None else point.ay,
            int(point.epoch_reset_applied),
        )
        for point in points
    ))


def sample_observer_replay(
    points: Sequence[ObserverReplayPoint],
    query_stamp_ns: int,
    parameters: ModalParameters,
) -> ModalState:
    query = _nonnegative_int("query_stamp_ns", query_stamp_ns)
    c_points, c_parameters = _c_observer_points(points), _c_parameters(parameters)
    output, error = _CModalState(), ctypes.create_string_buffer(_ERROR_CAPACITY)
    code = _lib().spmpc_modal_sample_observer(
        c_points, len(points), query, ctypes.byref(c_parameters),
        ctypes.byref(output), error, len(error)
    )
    _check(code, error)
    return _state(output)


def replay_planned_controls(
    initial_q: ModalState,
    initial_v: float,
    initial_omega: float,
    controls: Sequence[PlannedControl],
    parameters: ModalParameters,
    *,
    max_substep_sec: float = DEFAULT_PLANNED_RK4_SUBSTEP_SEC,
) -> Tuple[PlannedReplayPoint, ...]:
    c_initial, c_parameters = _c_state(initial_q), _c_parameters(parameters)
    c_controls = (_CPlannedControl * len(controls))(*(
        _CPlannedControl(control.a, control.alpha, control.duration_sec)
        for control in controls
    ))
    required = ctypes.c_size_t()
    error = ctypes.create_string_buffer(_ERROR_CAPACITY)
    code = _lib().spmpc_modal_replay_planned(
        ctypes.byref(c_initial), float(initial_v), float(initial_omega),
        c_controls, len(controls), ctypes.byref(c_parameters),
        float(max_substep_sec), None, 0, ctypes.byref(required), error, len(error)
    )
    _check(code, error)
    output = (_CPlannedPoint * required.value)()
    error = ctypes.create_string_buffer(_ERROR_CAPACITY)
    code = _lib().spmpc_modal_replay_planned(
        ctypes.byref(c_initial), float(initial_v), float(initial_omega),
        c_controls, len(controls), ctypes.byref(c_parameters),
        float(max_substep_sec), output, len(output), ctypes.byref(required),
        error, len(error)
    )
    _check(code, error)
    return tuple(
        PlannedReplayPoint(
            point.time_sec, point.v, point.omega,
            _state(point.state), point.control_index,
        )
        for point in output[:required.value]
    )


def _c_planned_points(points: Sequence[PlannedReplayPoint]):
    return (_CPlannedPoint * len(points))(*(
        _CPlannedPoint(
            point.time_sec, point.v, point.omega,
            _c_state(point.q), point.control_index,
        )
        for point in points
    ))


def sample_planned_replay(
    points: Sequence[PlannedReplayPoint], query_sec: float
) -> PlannedReplayPoint:
    c_points = _c_planned_points(points)
    output, error = _CPlannedPoint(), ctypes.create_string_buffer(_ERROR_CAPACITY)
    code = _lib().spmpc_modal_sample_planned(
        c_points, len(points), float(query_sec), ctypes.byref(output),
        error, len(error)
    )
    _check(code, error)
    return PlannedReplayPoint(
        output.time_sec, output.v, output.omega,
        _state(output.state), output.control_index,
    )


__all__ = (
    "DEFAULT_PLANNED_RK4_SUBSTEP_SEC",
    "ModalParameters", "ModalState", "ObserverAnchor", "ObserverInputSample",
    "ObserverReplayPoint", "ObserverReplayResult", "PlannedControl",
    "PlannedReplayPoint", "ReplayContractError", "cubic_hermite_q",
    "exact_zoh_forced_modal_step", "replay_observer_inputs",
    "replay_planned_controls", "sample_observer_replay",
    "sample_cubic_hermite_nodes", "sample_planned_replay",
)
