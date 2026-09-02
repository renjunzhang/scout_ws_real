"""Fixed liquid-cost coefficients for the Stage 3-D development model.

This module only turns an already-selected experiment condition and liquid
window/weight values into the numeric coefficients consumed by a future
solver wrapper.  It does not select ``K_liquid`` or either weight, construct a
CasADi graph, or produce a solver artifact.

The schedule follows the mainline right-endpoint contract:

* running coefficients occupy stages ``0 .. K_liquid - 1``;
* the boundary coefficient occupies stage ``K_liquid`` (the state
  ``x_K_liquid``); and
* all remaining entries, including terminal index ``N``, are zero.

The two coefficient vectors deliberately do not contain an additional
horizon or discretization scaling factor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from typing import Any

from .development_layout import HORIZON_STEPS

if HORIZON_STEPS != 60:  # pragma: no cover - protects the contract itself
    raise RuntimeError("mainline liquid schedule requires N=60")


LIQUID_COST_SCHEDULE_SCHEMA_VERSION = (
    "spmpc_mainline_right_endpoint_liquid_schedule_v1"
)
LIQUID_SCHEDULE_LENGTH = HORIZON_STEPS + 1
SCHEDULE_SUM_RELATIVE_TOLERANCE = 1.0e-12
SCHEDULE_SUM_ABSOLUTE_TOLERANCE = 0.0


class LiquidCostScheduleError(ValueError):
    """Raised when a liquid cost schedule input or output is invalid."""


class ExperimentCondition(str, Enum):
    """The only experiment conditions admitted by the mainline schedule."""

    B0 = "B0"
    Bslosh = "Bslosh"


def liquid_objective_scale(condition: ExperimentCondition) -> float:
    """Return the fixed objective scale for a strongly typed condition."""

    if type(condition) is not ExperimentCondition:
        raise LiquidCostScheduleError(
            "condition must be ExperimentCondition.B0 or "
            "ExperimentCondition.Bslosh"
        )
    if condition is ExperimentCondition.B0:
        return 0.0
    if condition is ExperimentCondition.Bslosh:
        return 1.0
    # This is unreachable for the current Enum, but keeps future additions
    # fail-closed instead of silently changing the experiment contract.
    raise LiquidCostScheduleError("unsupported experiment condition")


def _strict_int(value: Any, label: str) -> int:
    if type(value) is not int:
        raise LiquidCostScheduleError(f"{label} must be an integer")
    return value


def _finite_nonnegative(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LiquidCostScheduleError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise LiquidCostScheduleError(
            f"{label} must be representable as a finite number"
        ) from exc
    if not math.isfinite(result):
        raise LiquidCostScheduleError(f"{label} must be finite")
    if result < 0.0:
        raise LiquidCostScheduleError(f"{label} must be non-negative")
    if result == 0.0:
        return 0.0
    return result


def _validate_vector(
    values: tuple[float, ...], label: str, expected_length: int
) -> None:
    if type(values) is not tuple or len(values) != expected_length:
        raise LiquidCostScheduleError(
            f"{label} must be an immutable vector of length {expected_length}"
        )
    for index, value in enumerate(values):
        if type(value) is not float or not math.isfinite(value):
            raise LiquidCostScheduleError(
                f"{label}[{index}] must be a finite float"
            )


@dataclass(frozen=True)
class LiquidCostSchedule:
    """Immutable numeric schedule for one solver horizon.

    ``K_liquid`` and the weights are recorded for auditability.  The vectors
    are the only values intended for direct injection into a future ``p[k]``.
    """

    condition: ExperimentCondition
    K_liquid: int
    W_run: float
    W_boundary: float
    objective_scale: float
    liquid_run_coeff: tuple[float, ...]
    liquid_boundary_coeff: tuple[float, ...]

    def __post_init__(self) -> None:
        # Normalize tuples even when a caller constructs the dataclass
        # directly, so the frozen result cannot contain a mutable list.
        try:
            run_coeff = tuple(self.liquid_run_coeff)
            boundary_coeff = tuple(self.liquid_boundary_coeff)
        except TypeError as exc:
            raise LiquidCostScheduleError(
                "coefficient vectors must be iterable"
            ) from exc
        object.__setattr__(self, "liquid_run_coeff", run_coeff)
        object.__setattr__(self, "liquid_boundary_coeff", boundary_coeff)

        scale = liquid_objective_scale(self.condition)
        if type(self.objective_scale) is not float or not math.isfinite(
            self.objective_scale
        ):
            raise LiquidCostScheduleError(
                "objective_scale must be a finite float"
            )
        if self.objective_scale != scale:
            raise LiquidCostScheduleError(
                "objective_scale must be derived from condition"
            )
        if type(self.K_liquid) is not int or not (
            1 <= self.K_liquid < HORIZON_STEPS
        ):
            raise LiquidCostScheduleError(
                "K_liquid must satisfy 1 <= K_liquid < N"
            )
        if type(self.W_run) is not float or not math.isfinite(self.W_run):
            raise LiquidCostScheduleError("W_run must be a finite float")
        if type(self.W_boundary) is not float or not math.isfinite(
            self.W_boundary
        ):
            raise LiquidCostScheduleError(
                "W_boundary must be a finite float"
            )
        if self.W_run < 0.0 or self.W_boundary < 0.0:
            raise LiquidCostScheduleError("weights must be non-negative")

        _validate_vector(
            run_coeff, "liquid_run_coeff", LIQUID_SCHEDULE_LENGTH
        )
        _validate_vector(
            boundary_coeff,
            "liquid_boundary_coeff",
            LIQUID_SCHEDULE_LENGTH,
        )

        expected_run = scale * self.W_run
        expected_run_value = expected_run / float(self.K_liquid)
        if expected_run > 0.0 and expected_run_value == 0.0:
            raise LiquidCostScheduleError(
                "positive running weight underflows the per-stage coefficient"
            )
        for stage, value in enumerate(run_coeff):
            expected = expected_run_value if stage < self.K_liquid else 0.0
            if value != expected:
                raise LiquidCostScheduleError(
                    "running coefficients must be uniform on stages "
                    "0..K_liquid-1 and zero elsewhere"
                )
        try:
            actual_run = math.fsum(run_coeff)
        except OverflowError as exc:
            raise LiquidCostScheduleError(
                "liquid_run_coeff sum is not representable as a finite float"
            ) from exc
        if not math.isclose(
            actual_run,
            expected_run,
            rel_tol=SCHEDULE_SUM_RELATIVE_TOLERANCE,
            abs_tol=SCHEDULE_SUM_ABSOLUTE_TOLERANCE,
        ):
            raise LiquidCostScheduleError(
                "liquid_run_coeff sum must equal objective_scale*W_run"
            )

        expected_boundary = scale * self.W_boundary
        for stage, value in enumerate(boundary_coeff):
            expected = expected_boundary if stage == self.K_liquid else 0.0
            if value != expected:
                raise LiquidCostScheduleError(
                    "boundary coefficient must occur only at K_liquid"
                )
        nonzero_boundary = tuple(
            index for index, value in enumerate(boundary_coeff) if value != 0.0
        )
        if scale == 1.0 and self.W_boundary > 0.0:
            if nonzero_boundary != (self.K_liquid,) or (
                boundary_coeff[self.K_liquid] != expected_boundary
            ):
                raise LiquidCostScheduleError(
                    "Bslosh boundary coefficient must occur exactly once "
                    "at K_liquid"
                )
        elif any(value != 0.0 for value in boundary_coeff):
            raise LiquidCostScheduleError(
                "boundary coefficients must be zero when the scale is zero "
                "or W_boundary is zero"
            )

    @property
    def N(self) -> int:
        return HORIZON_STEPS

    @property
    def boundary_stage(self) -> int:
        return self.K_liquid

    @property
    def k_liquid(self) -> int:
        """PEP 8 alias for callers that use lowercase field conventions."""

        return self.K_liquid

    def to_dict(self) -> dict[str, Any]:
        """Return the complete JSON-compatible schedule snapshot."""

        return {
            "schema_version": LIQUID_COST_SCHEDULE_SCHEMA_VERSION,
            "horizon": {
                "N": HORIZON_STEPS,
                "parameter_vector_count": LIQUID_SCHEDULE_LENGTH,
            },
            "condition": self.condition.value,
            "inputs": {
                "K_liquid": self.K_liquid,
                "W_run": self.W_run,
                "W_boundary": self.W_boundary,
            },
            "objective_scale": self.objective_scale,
            "boundary_stage": self.boundary_stage,
            "liquid_run_coeff": list(self.liquid_run_coeff),
            "liquid_boundary_coeff": list(self.liquid_boundary_coeff),
        }


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
    if type(left) is float and left == 0.0 and right == 0.0:
        return math.copysign(1.0, left) == math.copysign(1.0, right)
    return bool(left == right)


def build_liquid_cost_schedule(
    condition: ExperimentCondition,
    K_liquid: int,
    W_run: float,
    W_boundary: float,
) -> LiquidCostSchedule:
    """Build the fixed ``N+1`` right-endpoint coefficient vectors.

    The caller must provide the already-selected window and weights.  This
    builder validates them and computes values only; it never selects a
    candidate, reads configuration/evidence, or constructs a solver graph.
    """

    scale = liquid_objective_scale(condition)
    k_liquid = _strict_int(K_liquid, "K_liquid")
    if not 1 <= k_liquid < HORIZON_STEPS:
        raise LiquidCostScheduleError(
            "K_liquid must satisfy 1 <= K_liquid < N"
        )
    w_run = _finite_nonnegative(W_run, "W_run")
    w_boundary = _finite_nonnegative(W_boundary, "W_boundary")

    run_value = scale * w_run / float(k_liquid)
    if not math.isfinite(run_value):
        raise LiquidCostScheduleError("running coefficient is non-finite")
    boundary_value = scale * w_boundary
    if not math.isfinite(boundary_value):
        raise LiquidCostScheduleError("boundary coefficient is non-finite")

    run_coeff = [0.0] * LIQUID_SCHEDULE_LENGTH
    boundary_coeff = [0.0] * LIQUID_SCHEDULE_LENGTH
    for stage in range(k_liquid):
        run_coeff[stage] = run_value
    boundary_coeff[k_liquid] = boundary_value

    return LiquidCostSchedule(
        condition=condition,
        K_liquid=k_liquid,
        W_run=w_run,
        W_boundary=w_boundary,
        objective_scale=scale,
        liquid_run_coeff=tuple(run_coeff),
        liquid_boundary_coeff=tuple(boundary_coeff),
    )


def require_liquid_cost_schedule(value: Any) -> LiquidCostSchedule:
    """Reject force-mutated schedules before they reach parameter assembly."""

    if type(value) is not LiquidCostSchedule:
        raise LiquidCostScheduleError(
            "schedule must be the exact LiquidCostSchedule type"
        )
    expected = build_liquid_cost_schedule(
        value.condition,
        value.K_liquid,
        value.W_run,
        value.W_boundary,
    )
    if not _strict_typed_equal(value, expected):
        raise LiquidCostScheduleError(
            "typed liquid cost schedule does not match its canonical expansion"
        )
    return value


__all__ = [
    "LIQUID_COST_SCHEDULE_SCHEMA_VERSION",
    "LIQUID_SCHEDULE_LENGTH",
    "ExperimentCondition",
    "LiquidCostSchedule",
    "LiquidCostScheduleError",
    "build_liquid_cost_schedule",
    "liquid_objective_scale",
    "require_liquid_cost_schedule",
]
