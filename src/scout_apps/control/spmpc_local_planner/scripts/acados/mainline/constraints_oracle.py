"""Explicit-bound constraint oracle for the Stage 3-D numeric model.

The oracle freezes only the *shape* of the hard constraints.  Every numerical
limit is supplied by the caller: this module has no production preset and no
numeric defaults.  Until the relevant evidence is promoted, these values are
``EXPLICIT_CALLER_SUPPLIED/DEV_UNVALIDATED`` inputs.

Each two-sided bound is emitted as separate lower and upper residuals.  A
residual is feasible when it is less than or equal to zero; using separate
residuals keeps this interface suitable for a future symbolic graph and avoids
replacing the two inequalities with a non-smooth ``abs`` expression.  Liquid
hard constraints remain disabled for both B0 and Bslosh.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any

from .development_capacity import DevelopmentCapacityContract
from .development_layout import DevelopmentLayout
from .discrete_dynamics import (
    DiscreteDynamicsError,
    IssuedCommandValues,
    evaluate_discrete_map,
)

CONSTRAINT_SCHEMA = "mainline_explicit_hard_constraints_v1"
CONSTRAINT_VALUE_STATUS = "EXPLICIT_CALLER_SUPPLIED/DEV_UNVALIDATED"
CONSTRAINT_RESIDUAL_ORDER = (
    "q_issue_v_lower",
    "q_issue_v_upper",
    "q_issue_omega_lower",
    "q_issue_omega_upper",
    "a_issue_lower",
    "a_issue_upper",
    "alpha_issue_lower",
    "alpha_issue_upper",
    "j_issue_v_lower",
    "j_issue_v_upper",
    "j_issue_omega_lower",
    "j_issue_omega_upper",
    "v_s_lower",
    "v_s_upper",
)


class ConstraintOracleError(ValueError):
    """Raised for malformed vectors or an incomplete/invalid bounds object."""


def _bound(value: Any, label: str) -> float:
    if type(value) is not float or not math.isfinite(value) or value <= 0.0:
        raise ConstraintOracleError(f"{label} must be a strictly positive finite float")
    return value


@dataclass(frozen=True)
class ConstraintBounds:
    """Explicit caller-supplied symmetric bounds.

    All values are required at construction time and are strictly positive
    finite floats.  They describe development-time constraints only; no value
    here is a production preset or a claim of hardware validation.
    """

    q_issue_v_max: float
    q_issue_omega_max: float
    a_issue_max: float
    alpha_issue_max: float
    jerk_v_max: float
    jerk_omega_max: float
    v_s_max: float

    def __post_init__(self) -> None:
        for field in fields(self):
            object.__setattr__(
                self,
                field.name,
                _bound(getattr(self, field.name), field.name),
            )

    def to_dict(self) -> dict[str, Any]:
        return {
            "constraint_schema": CONSTRAINT_SCHEMA,
            "value_status": CONSTRAINT_VALUE_STATUS,
            "values": {field.name: getattr(self, field.name) for field in fields(self)},
        }


@dataclass(frozen=True)
class ConstraintEvaluation:
    """Residuals use ``<= 0`` as the feasible convention."""

    residuals: tuple[tuple[str, float], ...]

    @property
    def feasible(self) -> bool:
        return all(value <= 0.0 for _, value in self.residuals)

    def to_dict(self) -> dict[str, Any]:
        return {
            "constraint_schema": CONSTRAINT_SCHEMA,
            "feasible": self.feasible,
            "residuals": {name: value for name, value in self.residuals},
        }


def _control(
    control: Any,
    development_layout: DevelopmentLayout,
) -> tuple[float, ...]:
    if type(development_layout) is not DevelopmentLayout:
        raise ConstraintOracleError(
            "development_layout must be the exact DevelopmentLayout type"
        )
    if type(control) is not tuple or len(control) != development_layout.NU:
        raise ConstraintOracleError(f"control must have length {development_layout.NU}")
    if any(type(value) is not float or not math.isfinite(value) for value in control):
        raise ConstraintOracleError("control must contain finite floats")
    return control


def _issued_values(issued: Any) -> IssuedCommandValues:
    """Validate the exact issue-map result consumed by the constraints."""

    if type(issued) is not IssuedCommandValues:
        raise ConstraintOracleError("issued must be the exact IssuedCommandValues type")
    try:
        values = issued.to_dict()
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise ConstraintOracleError("issued command values are malformed") from exc
    for name, value in values.items():
        if type(value) is not float or not math.isfinite(value):
            raise ConstraintOracleError(f"issued.{name} must be a finite float")
    return issued


def _bounds_values(bounds: Any) -> ConstraintBounds:
    if type(bounds) is not ConstraintBounds:
        raise ConstraintOracleError("bounds must be the exact ConstraintBounds type")
    # Re-run the checks in case a forged instance bypassed __post_init__.
    try:
        for field in fields(bounds):
            _bound(getattr(bounds, field.name), field.name)
    except (AttributeError, TypeError) as exc:
        raise ConstraintOracleError("bounds fields are malformed") from exc
    return bounds


def evaluate_constraint_residuals(
    control: tuple[float, ...],
    issued: IssuedCommandValues,
    bounds: ConstraintBounds,
    development_layout: DevelopmentLayout,
) -> ConstraintEvaluation:
    """Evaluate explicit hard bounds; no liquid hard constraint is emitted."""

    u = _control(control, development_layout)
    issued_values = _issued_values(issued)
    checked_bounds = _bounds_values(bounds)
    uo = development_layout.control_offsets
    values = (
        ("q_issue_v_lower", -checked_bounds.q_issue_v_max - issued_values.q_issue_v),
        ("q_issue_v_upper", issued_values.q_issue_v - checked_bounds.q_issue_v_max),
        (
            "q_issue_omega_lower",
            -checked_bounds.q_issue_omega_max - issued_values.q_issue_omega,
        ),
        (
            "q_issue_omega_upper",
            issued_values.q_issue_omega - checked_bounds.q_issue_omega_max,
        ),
        ("a_issue_lower", -checked_bounds.a_issue_max - issued_values.a_issue),
        ("a_issue_upper", issued_values.a_issue - checked_bounds.a_issue_max),
        (
            "alpha_issue_lower",
            -checked_bounds.alpha_issue_max - issued_values.alpha_issue,
        ),
        (
            "alpha_issue_upper",
            issued_values.alpha_issue - checked_bounds.alpha_issue_max,
        ),
        (
            "j_issue_v_lower",
            -checked_bounds.jerk_v_max - u[uo["j_issue_v"]],
        ),
        (
            "j_issue_v_upper",
            u[uo["j_issue_v"]] - checked_bounds.jerk_v_max,
        ),
        (
            "j_issue_omega_lower",
            -checked_bounds.jerk_omega_max - u[uo["j_issue_omega"]],
        ),
        (
            "j_issue_omega_upper",
            u[uo["j_issue_omega"]] - checked_bounds.jerk_omega_max,
        ),
        ("v_s_lower", -u[uo["v_s"]]),
        ("v_s_upper", u[uo["v_s"]] - checked_bounds.v_s_max),
    )
    checked = []
    for name, value in values:
        if not math.isfinite(value):
            raise ConstraintOracleError(f"{name} residual is non-finite")
        checked.append((name, value))
    if tuple(name for name, _ in checked) != CONSTRAINT_RESIDUAL_ORDER:
        raise ConstraintOracleError("constraint residual order drifted")
    return ConstraintEvaluation(tuple(checked))


def evaluate_constraints(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    state: tuple[float, ...],
    control: tuple[float, ...],
    stage_parameters: tuple[float, ...],
    bounds: ConstraintBounds,
) -> ConstraintEvaluation:
    """Propagate the pre-issue map and evaluate explicit hard bounds."""

    if type(capacity) is not DevelopmentCapacityContract:
        raise ConstraintOracleError(
            "capacity must be the exact DevelopmentCapacityContract type"
        )
    _control(control, development_layout)
    _bounds_values(bounds)
    try:
        result = evaluate_discrete_map(
            capacity, development_layout, state, control, stage_parameters
        )
    except DiscreteDynamicsError as exc:
        raise ConstraintOracleError(
            "stage model inputs violate the numeric contract"
        ) from exc
    return evaluate_constraint_residuals(
        control, result.issued, bounds, development_layout
    )


__all__ = [
    "CONSTRAINT_RESIDUAL_ORDER",
    "CONSTRAINT_SCHEMA",
    "CONSTRAINT_VALUE_STATUS",
    "ConstraintBounds",
    "ConstraintEvaluation",
    "ConstraintOracleError",
    "evaluate_constraint_residuals",
    "evaluate_constraints",
]
