"""Symbolic cost and constraint builders with no direct CasADi import."""

from __future__ import annotations

from typing import Any

from .casadi_graph_contract import (
    REFERENCE_DOMAIN_LOWER,
    REFERENCE_DOMAIN_UPPER,
    STAGE_REFERENCE_DOMAIN_ORDER,
    TERMINAL_REFERENCE_DOMAIN_ORDER,
    SymbolicIssuedValues,
    SymbolicReferenceValues,
    SymbolicStageConstraints,
    SymbolicStageCosts,
    SymbolicTerminalExpressions,
)
from .constraints_oracle import (
    CONSTRAINT_RESIDUAL_ORDER,
    CONTROL_BOX_ORDER,
    STAGE_NONLINEAR_H_ORDER,
    ConstraintBounds,
)
from .development_layout import DevelopmentLayout
from .solver_parameter_layout import SolverParameterLayout


class SymbolicObjectiveConstructionError(ValueError):
    """Typed cost or constraint structure cannot be expanded consistently."""


def _parameter(p: Any, layout: SolverParameterLayout, name: str) -> Any:
    return p[layout.parameter_offsets[name]]


def _state(x: Any, layout: DevelopmentLayout, name: str) -> Any:
    return x[layout.state_offsets[name]]


def _control(u: Any, layout: DevelopmentLayout, name: str) -> Any:
    return u[layout.control_offsets[name]]


def _liquid_state_cost(
    state: Any,
    p: Any,
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
    *,
    running: bool,
) -> Any:
    eta_ref = _parameter(p, parameter_layout, "slosh_eta_ref")
    omega_n_sq = _parameter(p, parameter_layout, "slosh_omega_n_sq")
    ratio = (
        _parameter(p, parameter_layout, "slosh_running_eta_dot_ratio")
        if running
        else 1.0
    )
    position = (
        _state(state, development_layout, "eta_x") ** 2
        + _state(state, development_layout, "eta_y") ** 2
    ) / (eta_ref * eta_ref)
    velocity_ratio = (
        _state(state, development_layout, "eta_x_dot") ** 2
        + _state(state, development_layout, "eta_y_dot") ** 2
    ) / (omega_n_sq * eta_ref * eta_ref)
    return position + ratio * velocity_ratio


def build_symbolic_stage_costs(
    x: Any,
    u: Any,
    p: Any,
    x_next: Any,
    issued: SymbolicIssuedValues,
    reference: SymbolicReferenceValues,
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> SymbolicStageCosts:
    """Build robot, right-endpoint liquid, boundary, and total stage costs."""

    norm = {
        name: _parameter(p, parameter_layout, name)
        for name in (
            "norm_contour",
            "norm_lag",
            "norm_v_actual",
            "norm_v_s",
            "norm_a_issue",
            "norm_alpha_issue",
            "norm_jerk_v",
            "norm_jerk_omega",
        )
    }
    weight = {
        name: _parameter(p, parameter_layout, name)
        for name in (
            "weight_contour",
            "weight_lag",
            "weight_progress",
            "weight_v_actual",
            "weight_v_s",
            "weight_a_issue",
            "weight_alpha_issue",
            "weight_jerk_v",
            "weight_jerk_omega",
        )
    }
    robot_running = (
        weight["weight_contour"] * (reference.e_contour / norm["norm_contour"]) ** 2
        + weight["weight_lag"] * (reference.e_lag / norm["norm_lag"]) ** 2
        - weight["weight_progress"]
        * (_control(u, development_layout, "v_s") / norm["norm_v_s"])
        + weight["weight_v_actual"]
        * (
            (_state(x, development_layout, "v_actual") - reference.ref_speed)
            / norm["norm_v_actual"]
        )
        ** 2
        + weight["weight_v_s"]
        * (
            (_control(u, development_layout, "v_s") - reference.ref_speed)
            / norm["norm_v_s"]
        )
        ** 2
        + weight["weight_a_issue"] * (issued.a_issue / norm["norm_a_issue"]) ** 2
        + weight["weight_alpha_issue"]
        * (issued.alpha_issue / norm["norm_alpha_issue"]) ** 2
        + weight["weight_jerk_v"]
        * (_control(u, development_layout, "j_issue_v") / norm["norm_jerk_v"]) ** 2
        + weight["weight_jerk_omega"]
        * (_control(u, development_layout, "j_issue_omega") / norm["norm_jerk_omega"])
        ** 2
    ) / float(development_layout.horizon_steps)
    liquid_running = _parameter(
        p,
        parameter_layout,
        "liquid_run_coeff",
    ) * _liquid_state_cost(
        x_next,
        p,
        development_layout,
        parameter_layout,
        running=True,
    )
    liquid_boundary = _parameter(
        p,
        parameter_layout,
        "liquid_boundary_coeff",
    ) * _liquid_state_cost(
        x,
        p,
        development_layout,
        parameter_layout,
        running=False,
    )
    return SymbolicStageCosts(
        robot_running=robot_running,
        liquid_running=liquid_running,
        liquid_boundary=liquid_boundary,
        total=robot_running + liquid_running + liquid_boundary,
    )


def _terminal_cost(
    x: Any,
    p: Any,
    reference: SymbolicReferenceValues,
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> Any:
    return (
        _parameter(p, parameter_layout, "weight_terminal_contour")
        * (reference.e_contour / _parameter(p, parameter_layout, "norm_contour")) ** 2
        + _parameter(p, parameter_layout, "weight_terminal_lag")
        * (reference.e_lag / _parameter(p, parameter_layout, "norm_lag")) ** 2
        + _parameter(p, parameter_layout, "weight_terminal_v_actual")
        * (
            (_state(x, development_layout, "v_actual") - reference.ref_speed)
            / _parameter(p, parameter_layout, "norm_v_actual")
        )
        ** 2
        + _parameter(
            p,
            parameter_layout,
            "weight_terminal_omega_actual",
        )
        * (
            _state(x, development_layout, "omega_actual")
            / _parameter(p, parameter_layout, "norm_omega_actual")
        )
        ** 2
    )


def build_symbolic_terminal(
    ca: Any,
    x: Any,
    p: Any,
    reference: SymbolicReferenceValues,
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> SymbolicTerminalExpressions:
    """Build terminal cost and xi_N domain using only x_N and p_N."""

    return SymbolicTerminalExpressions(
        reference=reference,
        total_cost=_terminal_cost(
            x,
            p,
            reference,
            development_layout,
            parameter_layout,
        ),
        reference_domain_h=ca.vertcat(reference.xi),
        reference_domain_order=TERMINAL_REFERENCE_DOMAIN_ORDER,
        reference_domain_lower=(REFERENCE_DOMAIN_LOWER,),
        reference_domain_upper=(REFERENCE_DOMAIN_UPPER,),
    )


def build_symbolic_stage_constraints(
    ca: Any,
    issued: SymbolicIssuedValues,
    u: Any,
    reference: SymbolicReferenceValues,
    bounds: ConstraintBounds,
    development_layout: DevelopmentLayout,
) -> SymbolicStageConstraints:
    """Build h, idxbu bounds, xi_k domain, and diagnostic residuals."""

    nonlinear_values = tuple(getattr(issued, name) for name in STAGE_NONLINEAR_H_ORDER)
    nonlinear_limits = tuple(
        getattr(bounds, f"{name}_max") for name in STAGE_NONLINEAR_H_ORDER
    )
    nonlinear_lower = tuple(-value for value in nonlinear_limits)
    nonlinear_upper = nonlinear_limits

    control_values = tuple(
        _control(u, development_layout, name) for name in CONTROL_BOX_ORDER
    )
    control_indices = tuple(
        development_layout.control_offsets[name] for name in CONTROL_BOX_ORDER
    )
    control_lower = (
        -bounds.jerk_v_max,
        -bounds.jerk_omega_max,
        0.0,
    )
    control_upper = (
        bounds.jerk_v_max,
        bounds.jerk_omega_max,
        bounds.v_s_max,
    )
    diagnostic_values: list[Any] = []
    diagnostic_names: list[str] = []
    for name, value, lower, upper in zip(
        STAGE_NONLINEAR_H_ORDER,
        nonlinear_values,
        nonlinear_lower,
        nonlinear_upper,
    ):
        diagnostic_names.extend((f"{name}_lower", f"{name}_upper"))
        diagnostic_values.extend((lower - value, value - upper))
    for name, value, lower, upper in zip(
        CONTROL_BOX_ORDER,
        control_values,
        control_lower,
        control_upper,
    ):
        diagnostic_names.extend((f"{name}_lower", f"{name}_upper"))
        diagnostic_values.extend((lower - value, value - upper))
    if tuple(diagnostic_names) != CONSTRAINT_RESIDUAL_ORDER:
        raise SymbolicObjectiveConstructionError("constraint residual order drifted")

    return SymbolicStageConstraints(
        nonlinear_h=ca.vertcat(*nonlinear_values),
        nonlinear_h_order=STAGE_NONLINEAR_H_ORDER,
        nonlinear_lower=nonlinear_lower,
        nonlinear_upper=nonlinear_upper,
        control_indices=control_indices,
        control_order=CONTROL_BOX_ORDER,
        control_lower=control_lower,
        control_upper=control_upper,
        reference_domain_h=ca.vertcat(reference.xi),
        reference_domain_order=STAGE_REFERENCE_DOMAIN_ORDER,
        reference_domain_lower=(REFERENCE_DOMAIN_LOWER,),
        reference_domain_upper=(REFERENCE_DOMAIN_UPPER,),
        diagnostic_residual=ca.vertcat(*diagnostic_values),
        diagnostic_residual_order=CONSTRAINT_RESIDUAL_ORDER,
        diagnostic_feasible_upper=tuple(0.0 for _ in diagnostic_values),
    )


__all__ = [
    "SymbolicObjectiveConstructionError",
    "build_symbolic_stage_constraints",
    "build_symbolic_stage_costs",
    "build_symbolic_terminal",
]
