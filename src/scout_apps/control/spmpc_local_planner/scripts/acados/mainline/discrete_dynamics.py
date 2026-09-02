"""Backend-neutral numeric oracle for the Stage 3-D single-step map.

The function in this module mirrors the frozen Stage 2 execution model while
reading state, control, and parameter values only through the typed Stage 3-D
layouts.  It neither constructs a symbolic graph nor imports CasADi/acados.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .development_capacity import DevelopmentCapacityContract
from .development_layout import DevelopmentLayout
from .model_contract import DISCRETIZATION_SCHEMA
from .solver_parameter_layout import (
    SolverParameterLayout,
    SolverParameterLayoutError,
    build_solver_parameter_layout,
)


class DiscreteDynamicsError(ValueError):
    """Raised when a vector or intermediate map value violates the contract."""


@dataclass(frozen=True)
class IssuedCommandValues:
    q_issue_v: float
    q_issue_omega: float
    a_issue: float
    alpha_issue: float

    def to_dict(self) -> dict[str, float]:
        return {
            "q_issue_v": self.q_issue_v,
            "q_issue_omega": self.q_issue_omega,
            "a_issue": self.a_issue,
            "alpha_issue": self.alpha_issue,
        }


@dataclass(frozen=True)
class ZohTargetSegmentValues:
    duration_sec: float
    q_target_v: float
    q_target_omega: float

    def to_dict(self) -> dict[str, float]:
        return {
            "duration_sec": self.duration_sec,
            "q_target_v": self.q_target_v,
            "q_target_omega": self.q_target_omega,
        }


@dataclass(frozen=True)
class DiscreteMapEvaluation:
    """Complete numeric result plus diagnostics used for parity tests."""

    next_state: tuple[float, ...]
    issued: IssuedCommandValues
    segments: tuple[ZohTargetSegmentValues, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "discretization_schema": DISCRETIZATION_SCHEMA,
            "next_state": list(self.next_state),
            "issued": self.issued.to_dict(),
            "segments": [segment.to_dict() for segment in self.segments],
        }


def _finite_vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if type(value) is not tuple or len(value) != length:
        raise DiscreteDynamicsError(
            f"{label} must be a tuple of exactly {length} floats"
        )
    for index, item in enumerate(value):
        if type(item) is not float or not math.isfinite(item):
            raise DiscreteDynamicsError(
                f"{label}[{index}] must be a finite float"
            )
    return value


def _finite_result(value: float, label: str) -> float:
    if not math.isfinite(value):
        raise DiscreteDynamicsError(f"{label} is non-finite")
    return value


def _parameter(
    values: tuple[float, ...],
    layout: SolverParameterLayout,
    name: str,
) -> float:
    return values[layout.parameter_offsets[name]]


def _one_hot_index(
    parameters: tuple[float, ...],
    offsets: tuple[int, ...],
    label: str,
) -> int:
    selected = -1
    for index, offset in enumerate(offsets):
        value = parameters[offset]
        if value not in (0.0, 1.0):
            raise DiscreteDynamicsError(f"{label} must be finite and one-hot")
        if value == 1.0:
            if selected >= 0:
                raise DiscreteDynamicsError(f"{label} must contain exactly one 1")
            selected = index
    if selected < 0:
        raise DiscreteDynamicsError(f"{label} must contain exactly one 1")
    return selected


def _issue_command(
    q_prev_v: float,
    q_prev_omega: float,
    a_prev: float,
    alpha_prev: float,
    jerk_v: float,
    jerk_omega: float,
    dt_sec: float,
) -> IssuedCommandValues:
    half_dt_squared = 0.5 * dt_sec * dt_sec
    issued = IssuedCommandValues(
        q_issue_v=q_prev_v + dt_sec * a_prev + half_dt_squared * jerk_v,
        q_issue_omega=(
            q_prev_omega + dt_sec * alpha_prev + half_dt_squared * jerk_omega
        ),
        a_issue=a_prev + dt_sec * jerk_v,
        alpha_issue=alpha_prev + dt_sec * jerk_omega,
    )
    for name, value in issued.to_dict().items():
        _finite_result(value, name)
    return issued


def _fopdt_at_elapsed(
    actual: float,
    target: float,
    elapsed_sec: float,
    inverse_tau: float,
    gain: float,
) -> float:
    if elapsed_sec == 0.0:
        return actual
    exponent = _finite_result(elapsed_sec * inverse_tau, "FOPDT exponent")
    steady_state = _finite_result(gain * target, "FOPDT steady state")
    rho = _finite_result(math.exp(-exponent), "FOPDT decay")
    one_minus_rho = _finite_result(-math.expm1(-exponent), "FOPDT gain")
    return _finite_result(
        rho * actual + one_minus_rho * steady_state,
        "FOPDT state",
    )


def _liquid_derivative(
    liquid: tuple[float, float, float, float],
    initial_actual: tuple[float, float],
    segment: ZohTargetSegmentValues,
    elapsed_sec: float,
    inverse_tau_v: float,
    gain_v: float,
    inverse_tau_omega: float,
    gain_omega: float,
    two_zeta_omega_n: float,
    omega_n_sq: float,
    kappa_x: float,
    kappa_y: float,
) -> tuple[float, float, float, float]:
    actual_v = _fopdt_at_elapsed(
        initial_actual[0],
        segment.q_target_v,
        elapsed_sec,
        inverse_tau_v,
        gain_v,
    )
    actual_omega = _fopdt_at_elapsed(
        initial_actual[1],
        segment.q_target_omega,
        elapsed_sec,
        inverse_tau_omega,
        gain_omega,
    )
    longitudinal_acceleration = _finite_result(
        inverse_tau_v * (gain_v * segment.q_target_v - actual_v),
        "longitudinal acceleration",
    )
    lateral_acceleration = _finite_result(
        actual_v * actual_omega,
        "lateral acceleration",
    )
    result = (
        liquid[1],
        -two_zeta_omega_n * liquid[1]
        - omega_n_sq * liquid[0]
        - kappa_x * longitudinal_acceleration,
        liquid[3],
        -two_zeta_omega_n * liquid[3]
        - omega_n_sq * liquid[2]
        - kappa_y * lateral_acceleration,
    )
    return tuple(
        _finite_result(value, f"liquid derivative[{index}]")
        for index, value in enumerate(result)
    )  # type: ignore[return-value]


def _add_scaled(
    initial: tuple[float, float, float, float],
    derivative: tuple[float, float, float, float],
    scale: float,
) -> tuple[float, float, float, float]:
    return tuple(
        _finite_result(
            initial[index] + scale * derivative[index],
            f"liquid intermediate[{index}]",
        )
        for index in range(4)
    )  # type: ignore[return-value]


def _rk4_liquid(
    initial: tuple[float, float, float, float],
    initial_actual: tuple[float, float],
    segment: ZohTargetSegmentValues,
    dynamic_parameters: tuple[float, ...],
) -> tuple[float, float, float, float]:
    if segment.duration_sec == 0.0:
        return initial
    half_duration = 0.5 * segment.duration_sec

    def derivative(
        liquid: tuple[float, float, float, float], elapsed_sec: float
    ) -> tuple[float, float, float, float]:
        return _liquid_derivative(
            liquid,
            initial_actual,
            segment,
            elapsed_sec,
            *dynamic_parameters,
        )

    k1 = derivative(initial, 0.0)
    k2 = derivative(_add_scaled(initial, k1, half_duration), half_duration)
    k3 = derivative(_add_scaled(initial, k2, half_duration), half_duration)
    k4 = derivative(
        _add_scaled(initial, k3, segment.duration_sec),
        segment.duration_sec,
    )
    scale = segment.duration_sec / 6.0
    return tuple(
        _finite_result(
            initial[index]
            + scale
            * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]),
            f"liquid state[{index}]",
        )
        for index in range(4)
    )  # type: ignore[return-value]


def _propagate_segment(
    physical: tuple[
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
    ],
    segment: ZohTargetSegmentValues,
    dynamic_parameters: tuple[float, ...],
) -> tuple[
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
]:
    if segment.duration_sec == 0.0:
        return physical
    px, py, theta, actual_v, actual_omega, *liquid_values = physical
    liquid = tuple(liquid_values)
    inverse_tau_v, gain_v, inverse_tau_omega, gain_omega = dynamic_parameters[:4]
    initial_actual = (actual_v, actual_omega)
    midpoint_v = _fopdt_at_elapsed(
        actual_v,
        segment.q_target_v,
        0.5 * segment.duration_sec,
        inverse_tau_v,
        gain_v,
    )
    midpoint_omega = _fopdt_at_elapsed(
        actual_omega,
        segment.q_target_omega,
        0.5 * segment.duration_sec,
        inverse_tau_omega,
        gain_omega,
    )
    end_v = _fopdt_at_elapsed(
        actual_v,
        segment.q_target_v,
        segment.duration_sec,
        inverse_tau_v,
        gain_v,
    )
    end_omega = _fopdt_at_elapsed(
        actual_omega,
        segment.q_target_omega,
        segment.duration_sec,
        inverse_tau_omega,
        gain_omega,
    )
    next_liquid = _rk4_liquid(
        liquid,  # type: ignore[arg-type]
        initial_actual,
        segment,
        dynamic_parameters,
    )
    heading_delta = _finite_result(
        segment.duration_sec * midpoint_omega,
        "heading delta",
    )
    heading_midpoint = _finite_result(
        theta + 0.5 * heading_delta,
        "heading midpoint",
    )
    result = (
        px + segment.duration_sec * midpoint_v * math.cos(heading_midpoint),
        py + segment.duration_sec * midpoint_v * math.sin(heading_midpoint),
        theta + heading_delta,
        end_v,
        end_omega,
        *next_liquid,
    )
    return tuple(
        _finite_result(value, f"physical state[{index}]")
        for index, value in enumerate(result)
    )  # type: ignore[return-value]


def _validate_dynamic_parameters(
    parameters: tuple[float, ...],
    layout: SolverParameterLayout,
) -> tuple[float, ...]:
    values = tuple(
        _parameter(parameters, layout, name)
        for name in (
            "act_inv_tau_v",
            "act_gain_v",
            "act_inv_tau_omega",
            "act_gain_omega",
            "slosh_two_zeta_omega_n",
            "slosh_omega_n_sq",
            "slosh_kappa_x",
            "slosh_kappa_y",
        )
    )
    if any(value <= 0.0 for value in (*values[:4], *values[5:])):
        raise DiscreteDynamicsError(
            "inverse tau, gain, slosh stiffness, and coupling must be positive"
        )
    if values[4] < 0.0:
        raise DiscreteDynamicsError("slosh damping coefficient must be non-negative")
    return values


def evaluate_discrete_map(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    state: tuple[float, ...],
    control: tuple[float, ...],
    stage_parameters: tuple[float, ...],
) -> DiscreteMapEvaluation:
    """Evaluate one frozen pre-issue stage without a symbolic backend."""

    try:
        parameter_layout = build_solver_parameter_layout(
            capacity,
            development_layout,
        )
    except SolverParameterLayoutError as exc:
        raise DiscreteDynamicsError(
            "capacity/development layout sources are not canonical"
        ) from exc
    checked_state = _finite_vector(state, development_layout.NX, "state")
    checked_control = _finite_vector(control, development_layout.NU, "control")
    parameters = _finite_vector(
        stage_parameters,
        parameter_layout.NP,
        "stage_parameters",
    )
    offsets = development_layout.state_offsets
    control_offsets = development_layout.control_offsets
    dt_sec = float(development_layout.release_period_sec)
    jerk_v = checked_control[control_offsets["j_issue_v"]]
    jerk_omega = checked_control[control_offsets["j_issue_omega"]]
    progress_velocity = checked_control[control_offsets["v_s"]]
    if progress_velocity < 0.0:
        raise DiscreteDynamicsError("v_s must be non-negative")

    issued = _issue_command(
        checked_state[offsets["q_prev_v"]],
        checked_state[offsets["q_prev_omega"]],
        checked_state[offsets["a_prev"]],
        checked_state[offsets["alpha_prev"]],
        jerk_v,
        jerk_omega,
        dt_sec,
    )
    taps_v = (
        issued.q_issue_v,
        checked_state[offsets["q_prev_v"]],
        *(
            checked_state[offsets[f"older_v[{index}]"]]
            for index in range(development_layout.d_v)
        ),
    )
    taps_omega = (
        issued.q_issue_omega,
        checked_state[offsets["q_prev_omega"]],
        *(
            checked_state[offsets[f"older_omega[{index}]"]]
            for index in range(development_layout.d_omega)
        ),
    )
    if len(taps_v) != development_layout.nq_v or len(taps_omega) != (
        development_layout.nq_omega
    ):
        raise DiscreteDynamicsError("delay taps do not match typed selector widths")

    durations = tuple(
        _parameter(parameters, parameter_layout, f"act_seg_dt[{slot}]")
        for slot in range(capacity.execution_subsegment_slots)
    )
    if any(duration < 0.0 for duration in durations) or sum(durations) != dt_sec:
        raise DiscreteDynamicsError(
            "segment durations must be non-negative and sum exactly to dt"
        )
    reached_unused = False
    segments: list[ZohTargetSegmentValues] = []
    for slot, duration in enumerate(durations):
        v_index = _one_hot_index(
            parameters,
            development_layout.selector_offsets["v"][slot],
            f"v selector[{slot}]",
        )
        omega_index = _one_hot_index(
            parameters,
            development_layout.selector_offsets["omega"][slot],
            f"omega selector[{slot}]",
        )
        if duration == 0.0:
            reached_unused = True
            if v_index != 0 or omega_index != 0:
                raise DiscreteDynamicsError(
                    "unused segment selectors must select Q(0)"
                )
        elif reached_unused:
            raise DiscreteDynamicsError(
                "active segment cannot follow an unused segment"
            )
        segments.append(
            ZohTargetSegmentValues(
                duration_sec=duration,
                q_target_v=taps_v[v_index],
                q_target_omega=taps_omega[omega_index],
            )
        )

    dynamic_parameters = _validate_dynamic_parameters(
        parameters,
        parameter_layout,
    )
    physical = (
        checked_state[offsets["px"]],
        checked_state[offsets["py"]],
        checked_state[offsets["theta"]],
        checked_state[offsets["v_actual"]],
        checked_state[offsets["omega_actual"]],
        checked_state[offsets["eta_x"]],
        checked_state[offsets["eta_x_dot"]],
        checked_state[offsets["eta_y"]],
        checked_state[offsets["eta_y_dot"]],
    )
    progress = checked_state[offsets["s"]]
    for segment in segments:
        physical = _propagate_segment(physical, segment, dynamic_parameters)
        progress = _finite_result(
            progress + segment.duration_sec * progress_velocity,
            "progress",
        )

    next_by_name = {
        "px": physical[0],
        "py": physical[1],
        "theta": physical[2],
        "s": progress,
        "v_actual": physical[3],
        "omega_actual": physical[4],
        "q_prev_v": issued.q_issue_v,
        "q_prev_omega": issued.q_issue_omega,
        "a_prev": issued.a_issue,
        "alpha_prev": issued.alpha_issue,
        "eta_x": physical[5],
        "eta_x_dot": physical[6],
        "eta_y": physical[7],
        "eta_y_dot": physical[8],
    }
    for index in range(development_layout.d_v):
        next_by_name[f"older_v[{index}]"] = (
            checked_state[offsets["q_prev_v"]]
            if index == 0
            else checked_state[offsets[f"older_v[{index - 1}]"]]
        )
    for index in range(development_layout.d_omega):
        next_by_name[f"older_omega[{index}]"] = (
            checked_state[offsets["q_prev_omega"]]
            if index == 0
            else checked_state[offsets[f"older_omega[{index - 1}]"]]
        )
    if set(next_by_name) != set(development_layout.state_names):
        raise DiscreteDynamicsError("next state does not assign the complete layout")
    next_state = tuple(next_by_name[name] for name in development_layout.state_names)
    return DiscreteMapEvaluation(
        next_state=next_state,
        issued=issued,
        segments=tuple(segments),
    )


__all__ = [
    "DISCRETIZATION_SCHEMA",
    "DiscreteDynamicsError",
    "DiscreteMapEvaluation",
    "IssuedCommandValues",
    "ZohTargetSegmentValues",
    "evaluate_discrete_map",
]
