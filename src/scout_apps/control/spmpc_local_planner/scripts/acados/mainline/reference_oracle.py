"""Numerical reference-path oracle for the Stage 3-D model.

The reference is deliberately evaluated from the same normalized cubic that is
injected into a stage parameter vector.  This module has no CasADi/acados
dependency and is useful both for diagnostics and for checking a future
symbolic graph.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .development_layout import DevelopmentLayout
from .solver_parameter_layout import SolverParameterLayout


class ReferenceOracleError(ValueError):
    """Raised when reference inputs are not finite or leave the effective path."""


@dataclass(frozen=True)
class ReferenceGeometry:
    """Reference geometry at one path coordinate (no vehicle state needed)."""

    s: float
    xi: float
    x_ref: float
    y_ref: float
    dx_ref_ds: float
    dy_ref_ds: float
    psi_ref: float
    ref_speed: float

    def to_dict(self) -> dict[str, float]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


@dataclass(frozen=True)
class ReferenceEvaluation(ReferenceGeometry):
    """Reference geometry and frozen MPCC tracking errors at one state."""

    e_contour: float
    e_lag: float


def _finite(value: Any, label: str) -> float:
    if type(value) is not float or not math.isfinite(value):
        raise ReferenceOracleError(f"{label} must be a finite float")
    return 0.0 if value == 0.0 else value


def _parameters(
    stage_parameters: Any, parameter_layout: SolverParameterLayout
) -> tuple[float, ...]:
    if type(parameter_layout) is not SolverParameterLayout:
        raise ReferenceOracleError("parameter_layout must be SolverParameterLayout")
    if (
        type(stage_parameters) is not tuple
        or len(stage_parameters) != parameter_layout.NP
    ):
        raise ReferenceOracleError(
            f"stage_parameters must be a tuple of exactly {parameter_layout.NP} floats"
        )
    return tuple(
        _finite(value, f"stage_parameters[{index}]")
        for index, value in enumerate(stage_parameters)
    )


def _parameter(
    values: tuple[float, ...], layout: SolverParameterLayout, name: str
) -> float:
    try:
        offset = layout.parameter_offsets[name]
    except KeyError as exc:  # pragma: no cover - protects forged layouts
        raise ReferenceOracleError(f"layout is missing {name}") from exc
    return values[offset]


def evaluate_reference_at_s(
    s: float,
    stage_parameters: tuple[float, ...],
    parameter_layout: SolverParameterLayout,
) -> ReferenceGeometry:
    """Evaluate the normalized cubic at ``s``.

    The effective domain is closed ``xi in [0, 1]``.  Values outside it are
    rejected; silently clamping would make solver and diagnostic semantics
    disagree.
    """

    s_value = _finite(s, "s")
    values = _parameters(stage_parameters, parameter_layout)
    origin = _parameter(values, parameter_layout, "ref_s_origin")
    scale = _parameter(values, parameter_layout, "ref_s_scale")
    if scale <= 0.0:
        raise ReferenceOracleError("ref_s_scale must be strictly positive")
    xi = (s_value - origin) / scale
    if not math.isfinite(xi):
        raise ReferenceOracleError("reference normalized coordinate is non-finite")
    if xi < 0.0 or xi > 1.0:
        raise ReferenceOracleError("s lies outside the effective reference domain")

    x = tuple(
        _parameter(values, parameter_layout, f"ref_x_coeff[{i}]") for i in range(4)
    )
    y = tuple(
        _parameter(values, parameter_layout, f"ref_y_coeff[{i}]") for i in range(4)
    )
    x_ref = ((x[3] * xi + x[2]) * xi + x[1]) * xi + x[0]
    y_ref = ((y[3] * xi + y[2]) * xi + y[1]) * xi + y[0]
    dx = (x[1] + 2.0 * x[2] * xi + 3.0 * x[3] * xi * xi) / scale
    dy = (y[1] + 2.0 * y[2] * xi + 3.0 * y[3] * xi * xi) / scale
    tangent_norm = math.hypot(dx, dy)
    if not math.isfinite(tangent_norm) or tangent_norm <= 0.0:
        raise ReferenceOracleError("reference tangent is degenerate")
    heading = math.atan2(dy, dx)
    ref_speed = _parameter(values, parameter_layout, "ref_speed")
    if ref_speed < 0.0:
        raise ReferenceOracleError("ref_speed must be non-negative")
    result = ReferenceGeometry(
        s=s_value,
        xi=xi,
        x_ref=_finite(x_ref, "x_ref"),
        y_ref=_finite(y_ref, "y_ref"),
        dx_ref_ds=_finite(dx, "dx_ref_ds"),
        dy_ref_ds=_finite(dy, "dy_ref_ds"),
        psi_ref=_finite(heading, "psi_ref"),
        ref_speed=ref_speed,
    )
    return result


def evaluate_reference(
    state: tuple[float, ...],
    stage_parameters: tuple[float, ...],
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> ReferenceEvaluation:
    """Evaluate reference geometry and MPCC errors for a pre-issue state."""

    if type(development_layout) is not DevelopmentLayout:
        raise ReferenceOracleError("development_layout must be DevelopmentLayout")
    if type(parameter_layout) is not SolverParameterLayout:
        raise ReferenceOracleError("parameter_layout must be SolverParameterLayout")
    if type(state) is not tuple or len(state) != development_layout.NX:
        raise ReferenceOracleError(
            f"state must be a tuple of exactly {development_layout.NX} floats"
        )
    state_values = tuple(_finite(value, f"state[{i}]") for i, value in enumerate(state))
    s = state_values[development_layout.state_offsets["s"]]
    px = state_values[development_layout.state_offsets["px"]]
    py = state_values[development_layout.state_offsets["py"]]
    geometry = evaluate_reference_at_s(s, stage_parameters, parameter_layout)
    dx = px - geometry.x_ref
    dy = py - geometry.y_ref
    sin_psi = math.sin(geometry.psi_ref)
    cos_psi = math.cos(geometry.psi_ref)
    # These signs are part of the frozen mainline MPCC cost contract.
    contour = sin_psi * dx - cos_psi * dy
    lag = -cos_psi * dx - sin_psi * dy
    return ReferenceEvaluation(
        **geometry.__dict__,
        e_contour=_finite(contour, "e_contour"),
        e_lag=_finite(lag, "e_lag"),
    )


__all__ = [
    "ReferenceEvaluation",
    "ReferenceGeometry",
    "ReferenceOracleError",
    "evaluate_reference",
    "evaluate_reference_at_s",
]
