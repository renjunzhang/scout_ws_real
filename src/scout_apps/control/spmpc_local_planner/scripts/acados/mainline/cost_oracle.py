"""Backend-neutral running, liquid, and terminal cost oracle.

All terms here are numeric counterparts of the frozen Stage 3-D cost
contract.  In particular, the liquid running term uses the right endpoint
returned by :func:`evaluate_discrete_map`; it is not evaluated at an
arbitrary neighbouring shooting node.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .development_capacity import DevelopmentCapacityContract
from .development_layout import DevelopmentLayout
from .discrete_dynamics import (
    DiscreteDynamicsError,
    DiscreteMapEvaluation,
    IssuedCommandValues,
    evaluate_discrete_map,
)
from .reference_oracle import (
    ReferenceEvaluation,
    ReferenceOracleError,
    evaluate_reference,
)
from .solver_parameter_layout import SolverParameterLayout


class CostOracleError(ValueError):
    """Raised when a cost input or derived term is invalid."""


@dataclass(frozen=True)
class StageCostEvaluation:
    """All contributions for one ordinary shooting stage.

    The three contribution fields are deliberately separate: at the
    configured liquid boundary stage the same ordinary stage contains the
    robot running term and the boundary term evaluated at the shooting state,
    while its running liquid coefficient is normally zero.
    """

    robot_running_cost: float
    liquid_running_cost: float
    liquid_boundary_cost: float
    total_cost: float
    reference: ReferenceEvaluation
    map_evaluation: DiscreteMapEvaluation

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot_running_cost": self.robot_running_cost,
            "liquid_running_cost": self.liquid_running_cost,
            "liquid_boundary_cost": self.liquid_boundary_cost,
            "total_cost": self.total_cost,
            "reference": self.reference.to_dict(),
            "map": self.map_evaluation.to_dict(),
        }


@dataclass(frozen=True)
class TerminalCostEvaluation:
    total_cost: float
    reference: ReferenceEvaluation

    def to_dict(self) -> dict[str, Any]:
        return {"total_cost": self.total_cost, "reference": self.reference.to_dict()}


def _finite(value: Any, label: str) -> float:
    if type(value) is not float or not math.isfinite(value):
        raise CostOracleError(f"{label} must be a finite float")
    return 0.0 if value == 0.0 else value


def _values(stage_parameters: Any, layout: SolverParameterLayout) -> tuple[float, ...]:
    if type(layout) is not SolverParameterLayout:
        raise CostOracleError("parameter_layout must be SolverParameterLayout")
    if type(stage_parameters) is not tuple or len(stage_parameters) != layout.NP:
        raise CostOracleError(f"stage_parameters must have length {layout.NP}")
    return tuple(
        _finite(v, f"stage_parameters[{i}]") for i, v in enumerate(stage_parameters)
    )


def _p(values: tuple[float, ...], layout: SolverParameterLayout, name: str) -> float:
    return values[layout.parameter_offsets[name]]


def _state(state: Any, layout: DevelopmentLayout) -> tuple[float, ...]:
    if type(layout) is not DevelopmentLayout:
        raise CostOracleError("development_layout must be DevelopmentLayout")
    if type(state) is not tuple or len(state) != layout.NX:
        raise CostOracleError(f"state must have length {layout.NX}")
    return tuple(_finite(v, f"state[{i}]") for i, v in enumerate(state))


def _control(control: Any, layout: DevelopmentLayout) -> tuple[float, ...]:
    if type(layout) is not DevelopmentLayout:
        raise CostOracleError("development_layout must be DevelopmentLayout")
    if type(control) is not tuple or len(control) != layout.NU:
        raise CostOracleError(f"control must have length {layout.NU}")
    return tuple(_finite(v, f"control[{i}]") for i, v in enumerate(control))


def _positive_parameter(
    values: tuple[float, ...], layout: SolverParameterLayout, name: str
) -> float:
    value = _p(values, layout, name)
    if value <= 0.0:
        raise CostOracleError(f"{name} must be strictly positive")
    return value


def _nonnegative_parameter(
    values: tuple[float, ...], layout: SolverParameterLayout, name: str
) -> float:
    value = _p(values, layout, name)
    if value < 0.0:
        raise CostOracleError(f"{name} must be non-negative")
    return value


def _issued_values(issued: IssuedCommandValues) -> tuple[float, float, float, float]:
    """Validate the typed map diagnostics before they enter a cost term."""

    if type(issued) is not IssuedCommandValues:
        raise CostOracleError("issued must be IssuedCommandValues")
    names = ("q_issue_v", "q_issue_omega", "a_issue", "alpha_issue")
    result = tuple(_finite(getattr(issued, name), f"issued.{name}") for name in names)
    return result  # type: ignore[return-value]


def liquid_state_cost(
    state: tuple[float, ...],
    stage_parameters: tuple[float, ...],
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
    *,
    running: bool,
) -> float:
    """Return the normalized quadratic liquid state cost (without coefficient)."""

    if type(development_layout) is not DevelopmentLayout:
        raise CostOracleError("development_layout must be DevelopmentLayout")
    if type(parameter_layout) is not SolverParameterLayout:
        raise CostOracleError("parameter_layout must be SolverParameterLayout")
    if type(running) is not bool:
        raise CostOracleError("running must be bool")
    x = _state(state, development_layout)
    p = _values(stage_parameters, parameter_layout)
    offsets = development_layout.state_offsets
    eta_x = x[offsets["eta_x"]]
    eta_x_dot = x[offsets["eta_x_dot"]]
    eta_y = x[offsets["eta_y"]]
    eta_y_dot = x[offsets["eta_y_dot"]]
    eta_ref = _positive_parameter(p, parameter_layout, "slosh_eta_ref")
    omega_sq = _positive_parameter(p, parameter_layout, "slosh_omega_n_sq")
    ratio = _nonnegative_parameter(p, parameter_layout, "slosh_running_eta_dot_ratio")
    position = (eta_x * eta_x + eta_y * eta_y) / (eta_ref * eta_ref)
    velocity_ratio = (eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot) / (
        omega_sq * eta_ref * eta_ref
    )
    try:
        result = position + (ratio if running else 1.0) * velocity_ratio
    except (OverflowError, ZeroDivisionError) as exc:
        raise CostOracleError("liquid state cost is not finite") from exc
    return _finite(result, "liquid state cost")


def evaluate_boundary_cost(
    state: tuple[float, ...],
    stage_parameters: tuple[float, ...],
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> float:
    """Evaluate the liquid boundary term at the shooting state ``x_K``.

    This deliberately does not accept a control or a discrete-map result:
    the boundary policy is defined on ``x_K`` and its velocity coefficient is
    fixed to one, independently of the running ``r_dot`` parameter.
    """

    if type(development_layout) is not DevelopmentLayout:
        raise CostOracleError("development_layout must be DevelopmentLayout")
    if type(parameter_layout) is not SolverParameterLayout:
        raise CostOracleError("parameter_layout must be SolverParameterLayout")
    x = _state(state, development_layout)
    p = _values(stage_parameters, parameter_layout)
    coefficient = _nonnegative_parameter(p, parameter_layout, "liquid_boundary_coeff")
    try:
        result = coefficient * liquid_state_cost(
            x, p, development_layout, parameter_layout, running=False
        )
    except (OverflowError, ZeroDivisionError) as exc:
        raise CostOracleError("liquid boundary cost is not finite") from exc
    return _finite(result, "liquid boundary cost")


def _robot_running_cost(
    state: tuple[float, ...],
    control: tuple[float, ...],
    issued: IssuedCommandValues,
    reference: ReferenceEvaluation,
    stage_parameters: tuple[float, ...],
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> float:
    """Compute the normalized robot running contribution for one stage."""

    if type(development_layout) is not DevelopmentLayout:
        raise CostOracleError("development_layout must be DevelopmentLayout")
    if type(parameter_layout) is not SolverParameterLayout:
        raise CostOracleError("parameter_layout must be SolverParameterLayout")
    if type(reference) is not ReferenceEvaluation:
        raise CostOracleError("reference must be ReferenceEvaluation")
    x = _state(state, development_layout)
    u = _control(control, development_layout)
    p = _values(stage_parameters, parameter_layout)
    issued_values = _issued_values(issued)
    xo = development_layout.state_offsets
    uo = development_layout.control_offsets
    ref_speed = _p(p, parameter_layout, "ref_speed")
    norm = {
        name: _positive_parameter(p, parameter_layout, name)
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
    weights = {
        name: _nonnegative_parameter(p, parameter_layout, name)
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
    if (
        type(development_layout.horizon_steps) is not int
        or development_layout.horizon_steps <= 0
    ):
        raise CostOracleError("horizon_steps must be a positive integer")
    try:
        terms = (
            weights["weight_contour"]
            * (reference.e_contour / norm["norm_contour"]) ** 2,
            weights["weight_lag"] * (reference.e_lag / norm["norm_lag"]) ** 2,
            -weights["weight_progress"] * (u[uo["v_s"]] / norm["norm_v_s"]),
            weights["weight_v_actual"]
            * ((x[xo["v_actual"]] - ref_speed) / norm["norm_v_actual"]) ** 2,
            weights["weight_v_s"]
            * ((u[uo["v_s"]] - ref_speed) / norm["norm_v_s"]) ** 2,
            weights["weight_a_issue"] * (issued_values[2] / norm["norm_a_issue"]) ** 2,
            weights["weight_alpha_issue"]
            * (issued_values[3] / norm["norm_alpha_issue"]) ** 2,
            weights["weight_jerk_v"] * (u[uo["j_issue_v"]] / norm["norm_jerk_v"]) ** 2,
            weights["weight_jerk_omega"]
            * (u[uo["j_issue_omega"]] / norm["norm_jerk_omega"]) ** 2,
        )
        result = math.fsum(terms) / float(development_layout.horizon_steps)
    except (OverflowError, ZeroDivisionError) as exc:
        raise CostOracleError("robot running cost is not finite") from exc
    return _finite(result, "robot running cost")


def evaluate_stage_cost(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
    state: tuple[float, ...],
    control: tuple[float, ...],
    stage_parameters: tuple[float, ...],
) -> StageCostEvaluation:
    """Evaluate all ordinary-stage contributions from one typed map step."""

    if type(capacity) is not DevelopmentCapacityContract:
        raise CostOracleError("capacity must be DevelopmentCapacityContract")
    if type(development_layout) is not DevelopmentLayout:
        raise CostOracleError("development_layout must be DevelopmentLayout")
    if type(parameter_layout) is not SolverParameterLayout:
        raise CostOracleError("parameter_layout must be SolverParameterLayout")
    x = _state(state, development_layout)
    u = _control(control, development_layout)
    p = _values(stage_parameters, parameter_layout)
    try:
        result_map = evaluate_discrete_map(capacity, development_layout, x, u, p)
        reference = evaluate_reference(x, p, development_layout, parameter_layout)
    except (DiscreteDynamicsError, ReferenceOracleError) as exc:
        raise CostOracleError(
            "stage model inputs violate the numeric contract"
        ) from exc
    robot = _robot_running_cost(
        x, u, result_map.issued, reference, p, development_layout, parameter_layout
    )
    coeff = _nonnegative_parameter(p, parameter_layout, "liquid_run_coeff")
    liquid = coeff * liquid_state_cost(
        result_map.next_state, p, development_layout, parameter_layout, running=True
    )
    liquid = _finite(liquid, "liquid running cost")
    boundary = evaluate_boundary_cost(x, p, development_layout, parameter_layout)
    total = _finite(robot + liquid + boundary, "stage cost")
    return StageCostEvaluation(robot, liquid, boundary, total, reference, result_map)


def evaluate_terminal_cost(
    state: tuple[float, ...],
    stage_parameters: tuple[float, ...],
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> TerminalCostEvaluation:
    """Evaluate terminal cost at x_N only (there is no u_N access)."""

    if type(development_layout) is not DevelopmentLayout:
        raise CostOracleError("development_layout must be DevelopmentLayout")
    if type(parameter_layout) is not SolverParameterLayout:
        raise CostOracleError("parameter_layout must be SolverParameterLayout")
    x = _state(state, development_layout)
    p = _values(stage_parameters, parameter_layout)
    try:
        reference = evaluate_reference(x, p, development_layout, parameter_layout)
    except ReferenceOracleError as exc:
        raise CostOracleError(
            "terminal reference inputs violate the numeric contract"
        ) from exc
    xo = development_layout.state_offsets
    ref_speed = _p(p, parameter_layout, "ref_speed")
    residuals = (
        reference.e_contour / _positive_parameter(p, parameter_layout, "norm_contour"),
        reference.e_lag / _positive_parameter(p, parameter_layout, "norm_lag"),
        (x[xo["v_actual"]] - ref_speed)
        / _positive_parameter(p, parameter_layout, "norm_v_actual"),
        x[xo["omega_actual"]]
        / _positive_parameter(p, parameter_layout, "norm_omega_actual"),
    )
    names = (
        "weight_terminal_contour",
        "weight_terminal_lag",
        "weight_terminal_v_actual",
        "weight_terminal_omega_actual",
    )
    try:
        result = math.fsum(
            _nonnegative_parameter(p, parameter_layout, name) * residual * residual
            for name, residual in zip(names, residuals)
        )
    except (OverflowError, ZeroDivisionError) as exc:
        raise CostOracleError("terminal cost is not finite") from exc
    return TerminalCostEvaluation(_finite(result, "terminal cost"), reference)


__all__ = [
    "CostOracleError",
    "StageCostEvaluation",
    "TerminalCostEvaluation",
    "evaluate_boundary_cost",
    "evaluate_stage_cost",
    "evaluate_terminal_cost",
    "liquid_state_cost",
]
