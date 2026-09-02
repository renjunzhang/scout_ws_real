"""Strongly typed numeric blocks consumed by the Stage 3-D assembler.

This module validates already-loaded values.  It does not read YAML, choose
weights, build delay schedules, construct a solver graph, or generate an
artifact.  All public dataclasses intentionally have no numeric defaults.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields, is_dataclass
from typing import Any

from .development_layout import HORIZON_STEPS

PARAMETER_VALUES_SCHEMA_VERSION = "spmpc_mainline_parameter_values_v1"
REFERENCE_SPEED_COUNT = HORIZON_STEPS + 1


class ParameterValueError(ValueError):
    """Raised when an explicit runtime parameter block is not canonical."""


def _finite_float(value: Any, label: str) -> float:
    if type(value) is not float or not math.isfinite(value):
        raise ParameterValueError(f"{label} must be a finite float")
    return 0.0 if value == 0.0 else value


def _positive_float(value: Any, label: str) -> float:
    result = _finite_float(value, label)
    if result <= 0.0:
        raise ParameterValueError(f"{label} must be strictly positive")
    return result


def _positive_divisor_float(value: Any, label: str) -> float:
    result = _positive_float(value, label)
    reciprocal = 1.0 / result
    if not math.isfinite(reciprocal) or reciprocal <= 0.0:
        raise ParameterValueError(
            f"{label} reciprocal must be finite and strictly positive"
        )
    return result


def _nonnegative_float(value: Any, label: str) -> float:
    result = _finite_float(value, label)
    if result < 0.0:
        raise ParameterValueError(f"{label} must be non-negative")
    return result


def _float_tuple(value: Any, length: int, label: str) -> tuple[float, ...]:
    if type(value) is not tuple or len(value) != length:
        raise ParameterValueError(
            f"{label} must be a tuple of exactly {length} floats"
        )
    return tuple(
        _finite_float(item, f"{label}[{index}]")
        for index, item in enumerate(value)
    )


def _strict_typed_equal(left: Any, right: Any) -> bool:
    if type(left) is not type(right):
        return False
    if is_dataclass(left):
        return all(
            _strict_typed_equal(
                getattr(left, field.name),
                getattr(right, field.name),
            )
            for field in fields(left)
        )
    if type(left) is tuple:
        return len(left) == len(right) and all(
            _strict_typed_equal(left_item, right_item)
            for left_item, right_item in zip(left, right)
        )
    return bool(left == right)


def _require_canonical_dataclass(value: Any, expected_type: type) -> Any:
    if type(value) is not expected_type:
        raise ParameterValueError(
            f"parameter block must have exact type {expected_type.__name__}"
        )
    expected_fields = {field.name for field in fields(value)}
    if set(vars(value)) != expected_fields:
        raise ParameterValueError(
            f"{expected_type.__name__} contains unknown or missing stored fields"
        )
    try:
        rebuilt = expected_type(
            **{field.name: getattr(value, field.name) for field in fields(value)}
        )
    except (AttributeError, OverflowError, TypeError, ValueError) as exc:
        raise ParameterValueError(
            f"{expected_type.__name__} is not a valid canonical block"
        ) from exc
    if not _strict_typed_equal(value, rebuilt):
        raise ParameterValueError(
            f"{expected_type.__name__} does not match its canonical expansion"
        )
    return rebuilt


def _polynomial_value(coefficients: tuple[float, ...], value: float) -> float:
    result = 0.0
    for coefficient in reversed(coefficients):
        result = result * value + coefficient
    return result


def _quadratic_real_roots(
    constant: float,
    linear: float,
    quadratic: float,
) -> tuple[float, ...]:
    if quadratic == 0.0:
        if linear == 0.0:
            return ()
        return (-constant / linear,)
    discriminant = linear * linear - 4.0 * quadratic * constant
    if not math.isfinite(discriminant):
        raise ParameterValueError("reference tangent root calculation overflowed")
    if discriminant < 0.0:
        return ()
    square_root = math.sqrt(discriminant)
    q = -0.5 * (linear + math.copysign(square_root, linear))
    if q == 0.0:
        return (-linear / (2.0 * quadratic),)
    return (q / quadratic, constant / q)


def _stationary_points_of_quartic(
    coefficients: tuple[float, float, float, float, float],
) -> tuple[float, ...]:
    """Find all candidate roots of the quartic derivative on [0, 1]."""

    derivative = (
        coefficients[1],
        2.0 * coefficients[2],
        3.0 * coefficients[3],
        4.0 * coefficients[4],
    )
    if derivative[3] == 0.0:
        roots = _quadratic_real_roots(*derivative[:3])
        return tuple(root for root in roots if 0.0 <= root <= 1.0)

    critical = _quadratic_real_roots(
        derivative[1],
        2.0 * derivative[2],
        3.0 * derivative[3],
    )
    boundaries = [0.0]
    boundaries.extend(sorted(root for root in critical if 0.0 < root < 1.0))
    boundaries.append(1.0)

    roots: list[float] = []
    for boundary in boundaries:
        if _polynomial_value(derivative, boundary) == 0.0:
            roots.append(boundary)
    # itertools.pairwise is unavailable on the supported Python 3.8 runtime.
    for left, right in zip(boundaries, boundaries[1:]):  # noqa: RUF007
        left_value = _polynomial_value(derivative, left)
        right_value = _polynomial_value(derivative, right)
        if left_value == 0.0 or right_value == 0.0:
            continue
        if (left_value < 0.0) == (right_value < 0.0):
            continue
        lower = left
        upper = right
        lower_value = left_value
        for _ in range(96):
            midpoint = 0.5 * (lower + upper)
            midpoint_value = _polynomial_value(derivative, midpoint)
            if midpoint_value == 0.0:
                lower = midpoint
                upper = midpoint
                break
            if (lower_value < 0.0) != (midpoint_value < 0.0):
                upper = midpoint
            else:
                lower = midpoint
                lower_value = midpoint_value
        roots.append(0.5 * (lower + upper))

    unique: list[float] = []
    for root in sorted(roots):
        if not unique or abs(root - unique[-1]) > 1.0e-14:
            unique.append(root)
    return tuple(unique)


def _minimum_reference_tangent_norm(
    x_coefficients: tuple[float, ...],
    y_coefficients: tuple[float, ...],
    s_scale: float,
) -> float:
    dx = (x_coefficients[1], 2.0 * x_coefficients[2], 3.0 * x_coefficients[3])
    dy = (y_coefficients[1], 2.0 * y_coefficients[2], 3.0 * y_coefficients[3])
    coefficient_scale = max(abs(value) for value in (*dx, *dy))
    if coefficient_scale == 0.0:
        return 0.0
    scaled_dx = tuple(value / coefficient_scale for value in dx)
    scaled_dy = tuple(value / coefficient_scale for value in dy)
    quartic = (
        scaled_dx[0] * scaled_dx[0] + scaled_dy[0] * scaled_dy[0],
        2.0
        * (scaled_dx[0] * scaled_dx[1] + scaled_dy[0] * scaled_dy[1]),
        scaled_dx[1] * scaled_dx[1]
        + scaled_dy[1] * scaled_dy[1]
        + 2.0
        * (scaled_dx[0] * scaled_dx[2] + scaled_dy[0] * scaled_dy[2]),
        2.0
        * (scaled_dx[1] * scaled_dx[2] + scaled_dy[1] * scaled_dy[2]),
        scaled_dx[2] * scaled_dx[2] + scaled_dy[2] * scaled_dy[2],
    )
    if not all(math.isfinite(value) for value in quartic):
        raise ParameterValueError("reference tangent norm polynomial is non-finite")
    candidates = (0.0, 1.0, *_stationary_points_of_quartic(quartic))
    minimum = math.inf
    for xi in candidates:
        dx_dxi = _polynomial_value(dx, xi)
        dy_dxi = _polynomial_value(dy, xi)
        tangent_norm = math.hypot(dx_dxi, dy_dxi) / s_scale
        if not math.isfinite(tangent_norm):
            raise ParameterValueError("reference tangent norm is non-finite")
        minimum = min(minimum, tangent_norm)
    return minimum


@dataclass(frozen=True)
class ActuatorResponseParameters:
    """Runtime FOPDT response values; delay is compiled separately."""

    tau_v_sec: float
    gain_v: float
    tau_omega_sec: float
    gain_omega: float

    def __post_init__(self) -> None:
        for name in ("tau_v_sec", "gain_v", "tau_omega_sec", "gain_omega"):
            object.__setattr__(self, name, _positive_float(getattr(self, name), name))
        if not math.isfinite(1.0 / self.tau_v_sec) or not math.isfinite(
            1.0 / self.tau_omega_sec
        ):
            raise ParameterValueError("inverse actuator time constant is non-finite")

    @property
    def execution_scalar_values(self) -> tuple[float, ...]:
        return (
            1.0 / self.tau_v_sec,
            self.gain_v,
            1.0 / self.tau_omega_sec,
            self.gain_omega,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "tau_v_sec": self.tau_v_sec,
            "gain_v": self.gain_v,
            "tau_omega_sec": self.tau_omega_sec,
            "gain_omega": self.gain_omega,
            "injected": {
                "act_inv_tau_v": self.execution_scalar_values[0],
                "act_gain_v": self.execution_scalar_values[1],
                "act_inv_tau_omega": self.execution_scalar_values[2],
                "act_gain_omega": self.execution_scalar_values[3],
            },
        }


@dataclass(frozen=True)
class ReferencePolynomialParameters:
    """One normalized local cubic used by the complete horizon."""

    ref_s_origin: float
    ref_s_scale: float
    ref_x_coeff: tuple[float, float, float, float]
    ref_y_coeff: tuple[float, float, float, float]
    minimum_tangent_norm_per_s: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "ref_s_origin",
            _finite_float(self.ref_s_origin, "ref_s_origin"),
        )
        object.__setattr__(
            self,
            "ref_s_scale",
            _positive_float(self.ref_s_scale, "ref_s_scale"),
        )
        if not math.isfinite(self.ref_s_origin + self.ref_s_scale):
            raise ParameterValueError("reference effective s domain is non-finite")
        object.__setattr__(
            self,
            "ref_x_coeff",
            _float_tuple(self.ref_x_coeff, 4, "ref_x_coeff"),
        )
        object.__setattr__(
            self,
            "ref_y_coeff",
            _float_tuple(self.ref_y_coeff, 4, "ref_y_coeff"),
        )
        object.__setattr__(
            self,
            "minimum_tangent_norm_per_s",
            _positive_float(
                self.minimum_tangent_norm_per_s,
                "minimum_tangent_norm_per_s",
            ),
        )
        actual_minimum = _minimum_reference_tangent_norm(
            self.ref_x_coeff,
            self.ref_y_coeff,
            self.ref_s_scale,
        )
        if actual_minimum < self.minimum_tangent_norm_per_s:
            raise ParameterValueError(
                "reference tangent is degenerate or below the explicit full-domain threshold"
            )

    @property
    def geometry_values(self) -> tuple[float, ...]:
        return (
            self.ref_s_origin,
            self.ref_s_scale,
            *self.ref_x_coeff,
            *self.ref_y_coeff,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "ref_s_origin": self.ref_s_origin,
            "ref_s_scale": self.ref_s_scale,
            "ref_x_coeff": list(self.ref_x_coeff),
            "ref_y_coeff": list(self.ref_y_coeff),
            "validation_policy": {
                "minimum_tangent_norm_per_s": self.minimum_tangent_norm_per_s,
                "validated_minimum_tangent_norm_per_s": (
                    _minimum_reference_tangent_norm(
                        self.ref_x_coeff,
                        self.ref_y_coeff,
                        self.ref_s_scale,
                    )
                ),
                "effective_xi_domain": {"lower": 0, "upper": 1},
            },
        }


@dataclass(frozen=True)
class ReferenceHorizonParameters:
    polynomial: ReferencePolynomialParameters
    ref_speed: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "polynomial",
            _require_canonical_dataclass(
                self.polynomial,
                ReferencePolynomialParameters,
            ),
        )
        speeds = _float_tuple(
            self.ref_speed,
            REFERENCE_SPEED_COUNT,
            "ref_speed",
        )
        for index, speed in enumerate(speeds):
            _nonnegative_float(speed, f"ref_speed[{index}]")
        object.__setattr__(self, "ref_speed", speeds)

    def values_for_stage(self, stage: int) -> tuple[float, ...]:
        if type(stage) is not int or not 0 <= stage < REFERENCE_SPEED_COUNT:
            raise ParameterValueError("reference stage index is out of range")
        return (*self.polynomial.geometry_values, self.ref_speed[stage])

    def to_dict(self) -> dict[str, Any]:
        return {
            "polynomial": self.polynomial.to_dict(),
            "ref_speed": list(self.ref_speed),
        }


@dataclass(frozen=True)
class SloshParameters:
    omega_n_rad_per_sec: float
    damping_ratio: float
    slosh_kappa_x: float
    slosh_kappa_y: float
    slosh_eta_ref: float
    slosh_running_eta_dot_ratio: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "omega_n_rad_per_sec",
            _positive_float(
                self.omega_n_rad_per_sec,
                "omega_n_rad_per_sec",
            ),
        )
        object.__setattr__(
            self,
            "damping_ratio",
            _nonnegative_float(self.damping_ratio, "damping_ratio"),
        )
        for name in (
            "slosh_kappa_x",
            "slosh_kappa_y",
            "slosh_eta_ref",
        ):
            object.__setattr__(self, name, _positive_float(getattr(self, name), name))
        object.__setattr__(
            self,
            "slosh_running_eta_dot_ratio",
            _nonnegative_float(
                self.slosh_running_eta_dot_ratio,
                "slosh_running_eta_dot_ratio",
            ),
        )
        damping = self.slosh_two_zeta_omega_n
        stiffness = self.slosh_omega_n_sq
        if (
            not math.isfinite(damping)
            or not math.isfinite(stiffness)
            or stiffness <= 0.0
            or (self.damping_ratio > 0.0 and damping <= 0.0)
        ):
            raise ParameterValueError(
                "derived slosh coefficients are non-finite or underflowed"
            )

    @property
    def slosh_two_zeta_omega_n(self) -> float:
        return 2.0 * self.damping_ratio * self.omega_n_rad_per_sec

    @property
    def slosh_omega_n_sq(self) -> float:
        return self.omega_n_rad_per_sec * self.omega_n_rad_per_sec

    @property
    def ordered_values(self) -> tuple[float, ...]:
        return (
            self.slosh_two_zeta_omega_n,
            self.slosh_omega_n_sq,
            self.slosh_kappa_x,
            self.slosh_kappa_y,
            self.slosh_eta_ref,
            self.slosh_running_eta_dot_ratio,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "source": {
                "omega_n_rad_per_sec": self.omega_n_rad_per_sec,
                "damping_ratio": self.damping_ratio,
                "slosh_kappa_x": self.slosh_kappa_x,
                "slosh_kappa_y": self.slosh_kappa_y,
                "slosh_eta_ref": self.slosh_eta_ref,
                "slosh_running_eta_dot_ratio": (
                    self.slosh_running_eta_dot_ratio
                ),
            },
            "injected": {
                "slosh_two_zeta_omega_n": self.slosh_two_zeta_omega_n,
                "slosh_omega_n_sq": self.slosh_omega_n_sq,
                "slosh_kappa_x": self.slosh_kappa_x,
                "slosh_kappa_y": self.slosh_kappa_y,
                "slosh_eta_ref": self.slosh_eta_ref,
                "slosh_running_eta_dot_ratio": (
                    self.slosh_running_eta_dot_ratio
                ),
            },
        }


@dataclass(frozen=True)
class NormalizationParameters:
    norm_contour: float
    norm_lag: float
    norm_v_actual: float
    norm_omega_actual: float
    norm_v_s: float
    norm_a_issue: float
    norm_alpha_issue: float
    norm_jerk_v: float
    norm_jerk_omega: float

    def __post_init__(self) -> None:
        for field in fields(self):
            object.__setattr__(
                self,
                field.name,
                _positive_divisor_float(
                    getattr(self, field.name),
                    field.name,
                ),
            )

    @property
    def ordered_values(self) -> tuple[float, ...]:
        return tuple(getattr(self, field.name) for field in fields(self))

    def to_dict(self) -> dict[str, float]:
        return {field.name: getattr(self, field.name) for field in fields(self)}


@dataclass(frozen=True)
class RunningWeightParameters:
    weight_contour: float
    weight_lag: float
    weight_progress: float
    weight_v_actual: float
    weight_v_s: float
    weight_a_issue: float
    weight_alpha_issue: float
    weight_jerk_v: float
    weight_jerk_omega: float

    def __post_init__(self) -> None:
        for field in fields(self):
            object.__setattr__(
                self,
                field.name,
                _nonnegative_float(getattr(self, field.name), field.name),
            )

    @property
    def ordered_values(self) -> tuple[float, ...]:
        return tuple(getattr(self, field.name) for field in fields(self))

    def to_dict(self) -> dict[str, float]:
        return {field.name: getattr(self, field.name) for field in fields(self)}


@dataclass(frozen=True)
class TerminalWeightParameters:
    weight_terminal_contour: float
    weight_terminal_lag: float
    weight_terminal_v_actual: float
    weight_terminal_omega_actual: float

    def __post_init__(self) -> None:
        for field in fields(self):
            object.__setattr__(
                self,
                field.name,
                _nonnegative_float(getattr(self, field.name), field.name),
            )

    @property
    def ordered_values(self) -> tuple[float, ...]:
        return tuple(getattr(self, field.name) for field in fields(self))

    def to_dict(self) -> dict[str, float]:
        return {field.name: getattr(self, field.name) for field in fields(self)}


@dataclass(frozen=True)
class CommonStageParameterValues:
    actuator: ActuatorResponseParameters
    reference: ReferenceHorizonParameters
    slosh: SloshParameters
    normalization: NormalizationParameters
    running_weight: RunningWeightParameters
    terminal_weight: TerminalWeightParameters

    def __post_init__(self) -> None:
        for field, expected_type in (
            ("actuator", ActuatorResponseParameters),
            ("reference", ReferenceHorizonParameters),
            ("slosh", SloshParameters),
            ("normalization", NormalizationParameters),
            ("running_weight", RunningWeightParameters),
            ("terminal_weight", TerminalWeightParameters),
        ):
            object.__setattr__(
                self,
                field,
                _require_canonical_dataclass(
                    getattr(self, field),
                    expected_type,
                ),
            )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": PARAMETER_VALUES_SCHEMA_VERSION,
            "actuator": self.actuator.to_dict(),
            "reference": self.reference.to_dict(),
            "slosh": self.slosh.to_dict(),
            "normalization": self.normalization.to_dict(),
            "running_weight": self.running_weight.to_dict(),
            "terminal_weight": self.terminal_weight.to_dict(),
        }


def require_common_stage_parameter_values(
    value: Any,
) -> CommonStageParameterValues:
    """Return a canonical copy or reject inconsistent/type-aliased values."""

    return _require_canonical_dataclass(value, CommonStageParameterValues)


__all__ = [
    "PARAMETER_VALUES_SCHEMA_VERSION",
    "REFERENCE_SPEED_COUNT",
    "ActuatorResponseParameters",
    "CommonStageParameterValues",
    "NormalizationParameters",
    "ParameterValueError",
    "ReferenceHorizonParameters",
    "ReferencePolynomialParameters",
    "RunningWeightParameters",
    "SloshParameters",
    "TerminalWeightParameters",
    "require_common_stage_parameter_values",
]
