"""CasADi-agnostic builder for the normalized local-cubic reference graph."""

from __future__ import annotations

from typing import Any

from .casadi_graph_contract import SymbolicReferenceValues
from .development_layout import DevelopmentLayout
from .solver_parameter_layout import SolverParameterLayout


def _parameter(p: Any, layout: SolverParameterLayout, name: str) -> Any:
    return p[layout.parameter_offsets[name]]


def _state(x: Any, layout: DevelopmentLayout, name: str) -> Any:
    return x[layout.state_offsets[name]]


def build_symbolic_reference(
    ca: Any,
    x: Any,
    p: Any,
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> SymbolicReferenceValues:
    """Build reference geometry and errors without importing CasADi here."""

    s = _state(x, development_layout, "s")
    origin = _parameter(p, parameter_layout, "ref_s_origin")
    scale = _parameter(p, parameter_layout, "ref_s_scale")
    xi = (s - origin) / scale
    x_coeff = [
        _parameter(p, parameter_layout, f"ref_x_coeff[{index}]") for index in range(4)
    ]
    y_coeff = [
        _parameter(p, parameter_layout, f"ref_y_coeff[{index}]") for index in range(4)
    ]
    x_ref = ((x_coeff[3] * xi + x_coeff[2]) * xi + x_coeff[1]) * xi + x_coeff[0]
    y_ref = ((y_coeff[3] * xi + y_coeff[2]) * xi + y_coeff[1]) * xi + y_coeff[0]
    dx_ref_ds = (
        x_coeff[1] + 2.0 * x_coeff[2] * xi + 3.0 * x_coeff[3] * xi * xi
    ) / scale
    dy_ref_ds = (
        y_coeff[1] + 2.0 * y_coeff[2] * xi + 3.0 * y_coeff[3] * xi * xi
    ) / scale
    psi_ref = ca.atan2(dy_ref_ds, dx_ref_ds)
    error_x = _state(x, development_layout, "px") - x_ref
    error_y = _state(x, development_layout, "py") - y_ref
    e_contour = ca.sin(psi_ref) * error_x - ca.cos(psi_ref) * error_y
    e_lag = -ca.cos(psi_ref) * error_x - ca.sin(psi_ref) * error_y
    return SymbolicReferenceValues(
        xi=xi,
        x_ref=x_ref,
        y_ref=y_ref,
        dx_ref_ds=dx_ref_ds,
        dy_ref_ds=dy_ref_ds,
        psi_ref=psi_ref,
        e_contour=e_contour,
        e_lag=e_lag,
        ref_speed=_parameter(p, parameter_layout, "ref_speed"),
    )


__all__ = ["build_symbolic_reference"]
