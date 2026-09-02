"""Symbolic issue and discrete dynamics builders with no direct CasADi import."""

from __future__ import annotations

from typing import Any

from .casadi_graph_contract import SymbolicExecutionSlot, SymbolicIssuedValues
from .development_layout import DevelopmentLayout
from .solver_parameter_layout import SolverParameterLayout


class SymbolicDynamicsConstructionError(ValueError):
    """Typed dynamics structure cannot be expanded consistently."""


def _parameter(p: Any, layout: SolverParameterLayout, name: str) -> Any:
    return p[layout.parameter_offsets[name]]


def _state(x: Any, layout: DevelopmentLayout, name: str) -> Any:
    return x[layout.state_offsets[name]]


def _control(u: Any, layout: DevelopmentLayout, name: str) -> Any:
    return u[layout.control_offsets[name]]


def build_symbolic_issue(
    x: Any,
    u: Any,
    development_layout: DevelopmentLayout,
) -> SymbolicIssuedValues:
    """Build the pre-issue sampled-data command map."""

    dt_sec = float(development_layout.release_period_sec)
    jerk_v = _control(u, development_layout, "j_issue_v")
    jerk_omega = _control(u, development_layout, "j_issue_omega")
    q_issue_v = (
        _state(x, development_layout, "q_prev_v")
        + dt_sec * _state(x, development_layout, "a_prev")
        + 0.5 * dt_sec * dt_sec * jerk_v
    )
    q_issue_omega = (
        _state(x, development_layout, "q_prev_omega")
        + dt_sec * _state(x, development_layout, "alpha_prev")
        + 0.5 * dt_sec * dt_sec * jerk_omega
    )
    return SymbolicIssuedValues(
        q_issue_v=q_issue_v,
        q_issue_omega=q_issue_omega,
        a_issue=_state(x, development_layout, "a_prev") + dt_sec * jerk_v,
        alpha_issue=(_state(x, development_layout, "alpha_prev") + dt_sec * jerk_omega),
    )


def _fopdt_at_elapsed(
    ca: Any,
    actual: Any,
    target: Any,
    elapsed_sec: Any,
    inverse_tau: Any,
    gain: Any,
) -> Any:
    z = elapsed_sec * inverse_tau
    rho = ca.exp(-z)
    one_minus_rho = -ca.expm1(-z)
    steady_state = gain * target
    return rho * actual + one_minus_rho * steady_state


def _liquid_derivative(
    ca: Any,
    liquid: tuple[Any, Any, Any, Any],
    initial_actual: tuple[Any, Any],
    target: tuple[Any, Any],
    elapsed_sec: Any,
    dynamic_parameters: tuple[Any, ...],
) -> tuple[Any, Any, Any, Any]:
    (
        inverse_tau_v,
        gain_v,
        inverse_tau_omega,
        gain_omega,
        two_zeta_omega_n,
        omega_n_sq,
        kappa_x,
        kappa_y,
    ) = dynamic_parameters
    actual_v = _fopdt_at_elapsed(
        ca,
        initial_actual[0],
        target[0],
        elapsed_sec,
        inverse_tau_v,
        gain_v,
    )
    actual_omega = _fopdt_at_elapsed(
        ca,
        initial_actual[1],
        target[1],
        elapsed_sec,
        inverse_tau_omega,
        gain_omega,
    )
    longitudinal_acceleration = inverse_tau_v * (gain_v * target[0] - actual_v)
    lateral_acceleration = actual_v * actual_omega
    return (
        liquid[1],
        -two_zeta_omega_n * liquid[1]
        - omega_n_sq * liquid[0]
        - kappa_x * longitudinal_acceleration,
        liquid[3],
        -two_zeta_omega_n * liquid[3]
        - omega_n_sq * liquid[2]
        - kappa_y * lateral_acceleration,
    )


def _add_scaled(
    initial: tuple[Any, Any, Any, Any],
    derivative: tuple[Any, Any, Any, Any],
    scale: Any,
) -> tuple[Any, Any, Any, Any]:
    return tuple(initial[index] + scale * derivative[index] for index in range(4))  # type: ignore[return-value]


def _advance_liquid(
    ca: Any,
    initial: tuple[Any, Any, Any, Any],
    initial_actual: tuple[Any, Any],
    target: tuple[Any, Any],
    duration_sec: Any,
    dynamic_parameters: tuple[Any, ...],
) -> tuple[Any, Any, Any, Any]:
    half_duration = 0.5 * duration_sec

    def derivative(
        liquid: tuple[Any, Any, Any, Any],
        elapsed_sec: Any,
    ) -> tuple[Any, Any, Any, Any]:
        return _liquid_derivative(
            ca,
            liquid,
            initial_actual,
            target,
            elapsed_sec,
            dynamic_parameters,
        )

    k1 = derivative(initial, 0.0)
    k2 = derivative(_add_scaled(initial, k1, half_duration), half_duration)
    k3 = derivative(_add_scaled(initial, k2, half_duration), half_duration)
    k4 = derivative(
        _add_scaled(initial, k3, duration_sec),
        duration_sec,
    )
    scale = duration_sec / 6.0
    return tuple(
        initial[index]
        + scale * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index])
        for index in range(4)
    )  # type: ignore[return-value]


def _advance_physical_state(
    ca: Any,
    physical: tuple[Any, ...],
    slot: SymbolicExecutionSlot,
    dynamic_parameters: tuple[Any, ...],
) -> tuple[Any, ...]:
    px, py, theta, actual_v, actual_omega, *liquid_values = physical
    inverse_tau_v, gain_v, inverse_tau_omega, gain_omega = dynamic_parameters[:4]
    initial_actual = (actual_v, actual_omega)
    target = (slot.q_target_v, slot.q_target_omega)
    midpoint_v = _fopdt_at_elapsed(
        ca,
        actual_v,
        target[0],
        0.5 * slot.duration_sec,
        inverse_tau_v,
        gain_v,
    )
    midpoint_omega = _fopdt_at_elapsed(
        ca,
        actual_omega,
        target[1],
        0.5 * slot.duration_sec,
        inverse_tau_omega,
        gain_omega,
    )
    end_v = _fopdt_at_elapsed(
        ca,
        actual_v,
        target[0],
        slot.duration_sec,
        inverse_tau_v,
        gain_v,
    )
    end_omega = _fopdt_at_elapsed(
        ca,
        actual_omega,
        target[1],
        slot.duration_sec,
        inverse_tau_omega,
        gain_omega,
    )
    liquid_next = _advance_liquid(
        ca,
        tuple(liquid_values),  # type: ignore[arg-type]
        initial_actual,
        target,
        slot.duration_sec,
        dynamic_parameters,
    )
    heading_delta = slot.duration_sec * midpoint_omega
    heading_midpoint = theta + 0.5 * heading_delta
    candidate = (
        px + slot.duration_sec * midpoint_v * ca.cos(heading_midpoint),
        py + slot.duration_sec * midpoint_v * ca.sin(heading_midpoint),
        theta + heading_delta,
        end_v,
        end_omega,
        *liquid_next,
    )
    return tuple(
        ca.if_else(slot.duration_sec == 0.0, current, advanced)
        for current, advanced in zip(physical, candidate)
    )


def _selected_target(
    p: Any,
    parameter_layout: SolverParameterLayout,
    channel: str,
    slot: int,
    taps: tuple[Any, ...],
) -> Any:
    return sum(
        _parameter(
            p,
            parameter_layout,
            f"act_sel_{channel}[{slot}][{tap}]",
        )
        * value
        for tap, value in enumerate(taps)
    )


def build_symbolic_discrete_map(
    ca: Any,
    x: Any,
    u: Any,
    p: Any,
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
    issued: SymbolicIssuedValues,
) -> tuple[Any, tuple[SymbolicExecutionSlot, ...]]:
    """Build issue-aware three-slot FOPDT, pose, liquid, and queue dynamics."""

    taps_v = (
        issued.q_issue_v,
        _state(x, development_layout, "q_prev_v"),
        *(
            _state(x, development_layout, f"older_v[{index}]")
            for index in range(development_layout.d_v)
        ),
    )
    taps_omega = (
        issued.q_issue_omega,
        _state(x, development_layout, "q_prev_omega"),
        *(
            _state(x, development_layout, f"older_omega[{index}]")
            for index in range(development_layout.d_omega)
        ),
    )
    if (
        len(taps_v) != development_layout.nq_v
        or len(taps_omega) != development_layout.nq_omega
    ):
        raise SymbolicDynamicsConstructionError(
            "delay taps do not match typed selector widths"
        )
    slot_count = len(development_layout.selector_offsets["v"])
    if slot_count != len(development_layout.selector_offsets["omega"]):
        raise SymbolicDynamicsConstructionError("selector channel slot counts differ")

    dynamic_parameters = tuple(
        _parameter(p, parameter_layout, name)
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
    physical = tuple(
        _state(x, development_layout, name)
        for name in (
            "px",
            "py",
            "theta",
            "v_actual",
            "omega_actual",
            "eta_x",
            "eta_x_dot",
            "eta_y",
            "eta_y_dot",
        )
    )
    progress = _state(x, development_layout, "s")
    progress_velocity = _control(u, development_layout, "v_s")
    slots: list[SymbolicExecutionSlot] = []
    for slot_index in range(slot_count):
        slot = SymbolicExecutionSlot(
            duration_sec=_parameter(
                p,
                parameter_layout,
                f"act_seg_dt[{slot_index}]",
            ),
            q_target_v=_selected_target(
                p,
                parameter_layout,
                "v",
                slot_index,
                taps_v,
            ),
            q_target_omega=_selected_target(
                p,
                parameter_layout,
                "omega",
                slot_index,
                taps_omega,
            ),
        )
        slots.append(slot)
        physical = _advance_physical_state(
            ca,
            physical,
            slot,
            dynamic_parameters,
        )
        progress = progress + slot.duration_sec * progress_velocity

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
            _state(x, development_layout, "q_prev_v")
            if index == 0
            else _state(x, development_layout, f"older_v[{index - 1}]")
        )
    for index in range(development_layout.d_omega):
        next_by_name[f"older_omega[{index}]"] = (
            _state(x, development_layout, "q_prev_omega")
            if index == 0
            else _state(x, development_layout, f"older_omega[{index - 1}]")
        )
    return (
        ca.vertcat(*(next_by_name[name] for name in development_layout.state_names)),
        tuple(slots),
    )


__all__ = [
    "SymbolicDynamicsConstructionError",
    "build_symbolic_discrete_map",
    "build_symbolic_issue",
]
