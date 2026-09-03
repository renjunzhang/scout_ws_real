"""Assemble complete immutable Stage 3-D runtime parameter horizons.

The assembler is deliberately backend-neutral: it validates typed inputs and
fills every field of every ``p[k]`` vector, but it does not import CasADi,
acados, ROS, configuration files, or a generated solver.  Parameter names and
offsets come exclusively from :mod:`solver_parameter_layout`.
"""

from __future__ import annotations

import math
from dataclasses import InitVar, dataclass
from typing import Any

from .cost_schedule import (
    ExperimentCondition,
    LiquidCostSchedule,
    LiquidCostScheduleError,
    require_liquid_cost_schedule,
)
from .development_capacity import DevelopmentCapacityContract
from .development_layout import DevelopmentLayout
from .identity import IdentityError, sha256_json, strict_json_equal
from .parameter_values import (
    CommonStageParameterValues,
    ParameterValueError,
    require_common_stage_parameter_values,
)
from .runtime_schedule import (
    RuntimeFractionalDelaySchedule,
    RuntimeScheduleError,
    require_runtime_fractional_delay_schedule,
)
from .solver_parameter_layout import (
    ARTIFACT_STATUS,
    GRAPH_STATUS,
    NORMALIZATION_PARAMETER_ORDER,
    REFERENCE_PARAMETER_ORDER,
    RUNNING_WEIGHT_PARAMETER_ORDER,
    SLOSH_PARAMETER_ORDER,
    STAGE_COST_PARAMETER_ORDER,
    TERMINAL_WEIGHT_PARAMETER_ORDER,
    SolverParameterLayout,
    SolverParameterLayoutError,
    build_solver_parameter_layout,
)

RUNTIME_PARAMETER_SNAPSHOT_SCHEMA_VERSION = (
    "spmpc_mainline_runtime_parameter_snapshot_v1"
)
RUNTIME_PARAMETER_SNAPSHOT_SCOPE = "COMPLETE_N_PLUS_ONE_PARAMETER_VALUES"
MATCHED_ARM_PAIR_SCHEMA_VERSION = "spmpc_mainline_matched_arm_pair_v1"
MATCHED_ARM_PAIR_SCOPE = "B0_BSLOSH_PARAMETER_ONLY_COMPARISON"
ALLOWED_ARM_PARAMETER_DIFFERENCES = STAGE_COST_PARAMETER_ORDER

_SNAPSHOT_TOKEN = object()
_PAIR_TOKEN = object()


class RuntimeParameterAssemblyError(ValueError):
    """Raised when a typed input or complete parameter snapshot is invalid."""


def _semantic_sha256(value: Any, label: str) -> str:
    try:
        return sha256_json(value)
    except IdentityError as exc:
        raise RuntimeParameterAssemblyError(
            f"{label} cannot be encoded as canonical JSON"
        ) from exc


def _finite_float_tuple(values: Any, label: str) -> tuple[float, ...]:
    if type(values) is not tuple:
        raise RuntimeParameterAssemblyError(f"{label} must be an immutable tuple")
    for index, value in enumerate(values):
        if type(value) is not float or not math.isfinite(value):
            raise RuntimeParameterAssemblyError(
                f"{label}[{index}] must be a finite float"
            )
    return values


@dataclass(frozen=True)
class RuntimeParameterSnapshot:
    """One complete, immutable ``(N+1) x NP`` numeric parameter snapshot."""

    capacity_contract_sha256: str
    development_layout_sha256: str
    solver_parameter_layout_sha256: str
    runtime_schedule_sha256: str
    parameter_values_sha256: str
    liquid_cost_schedule_sha256: str
    condition: ExperimentCondition
    parameter_names: tuple[str, ...]
    stage_parameters: tuple[tuple[float, ...], ...]
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _SNAPSHOT_TOKEN:
            raise RuntimeParameterAssemblyError(
                "RuntimeParameterSnapshot requires the canonical assembler"
            )

    @property
    def N(self) -> int:
        return len(self.stage_parameters) - 1

    @property
    def NP(self) -> int:
        return len(self.parameter_names)

    @property
    def sha256(self) -> str:
        return _semantic_sha256(self.to_dict(), "runtime parameter snapshot")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": RUNTIME_PARAMETER_SNAPSHOT_SCHEMA_VERSION,
            "scope": RUNTIME_PARAMETER_SNAPSHOT_SCOPE,
            "capability_status": {
                "graph": GRAPH_STATUS,
                "artifact": ARTIFACT_STATUS,
            },
            "source_identity": {
                "development_capacity_raw_bytes_sha256": (
                    self.capacity_contract_sha256
                ),
                "development_layout_semantic_sha256": (self.development_layout_sha256),
                "solver_parameter_layout_semantic_sha256": (
                    self.solver_parameter_layout_sha256
                ),
                "runtime_schedule_semantic_sha256": self.runtime_schedule_sha256,
                "parameter_values_semantic_sha256": self.parameter_values_sha256,
                "liquid_cost_schedule_semantic_sha256": (
                    self.liquid_cost_schedule_sha256
                ),
            },
            "experiment_condition": self.condition.value,
            "dimensions": {
                "N": self.N,
                "parameter_vector_count": len(self.stage_parameters),
                "NP": self.NP,
            },
            "parameter_order": list(self.parameter_names),
            "stage_parameters": [list(row) for row in self.stage_parameters],
        }


@dataclass(frozen=True)
class MatchedRuntimeParameterPair:
    """B0/Bslosh snapshots whose only admitted differences are cost slots."""

    b0: RuntimeParameterSnapshot
    bslosh: RuntimeParameterSnapshot
    allowed_parameter_names: tuple[str, ...]
    allowed_parameter_offsets: tuple[int, ...]
    differing_stage_offsets: tuple[tuple[int, int], ...]
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _PAIR_TOKEN:
            raise RuntimeParameterAssemblyError(
                "MatchedRuntimeParameterPair requires the canonical pair builder"
            )

    @property
    def sha256(self) -> str:
        return _semantic_sha256(self.to_dict(), "matched runtime parameter pair")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": MATCHED_ARM_PAIR_SCHEMA_VERSION,
            "scope": MATCHED_ARM_PAIR_SCOPE,
            "arms": {
                "B0_runtime_parameter_sha256": self.b0.sha256,
                "Bslosh_runtime_parameter_sha256": self.bslosh.sha256,
            },
            "difference_whitelist": {
                "parameter_names": list(self.allowed_parameter_names),
                "parameter_offsets": list(self.allowed_parameter_offsets),
            },
            "actual_differences": [
                {
                    "stage": stage,
                    "offset": offset,
                    "parameter_name": self.b0.parameter_names[offset],
                }
                for stage, offset in self.differing_stage_offsets
            ],
        }


def _checked_inputs(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    runtime_schedule: RuntimeFractionalDelaySchedule,
    values: CommonStageParameterValues,
    liquid_schedule: LiquidCostSchedule,
) -> tuple[
    SolverParameterLayout,
    RuntimeFractionalDelaySchedule,
    CommonStageParameterValues,
    LiquidCostSchedule,
]:
    try:
        layout = build_solver_parameter_layout(capacity, development_layout)
    except SolverParameterLayoutError as exc:
        raise RuntimeParameterAssemblyError(
            "solver parameter layout sources are not canonical"
        ) from exc
    try:
        checked_runtime = require_runtime_fractional_delay_schedule(
            runtime_schedule,
            capacity,
        )
    except RuntimeScheduleError as exc:
        raise RuntimeParameterAssemblyError(
            "runtime fractional-delay schedule is not canonical"
        ) from exc
    try:
        checked_values = require_common_stage_parameter_values(values)
    except ParameterValueError as exc:
        raise RuntimeParameterAssemblyError(
            "common stage parameter values are not canonical"
        ) from exc
    try:
        checked_liquid = require_liquid_cost_schedule(liquid_schedule)
    except LiquidCostScheduleError as exc:
        raise RuntimeParameterAssemblyError(
            "liquid cost schedule is not canonical"
        ) from exc
    return layout, checked_runtime, checked_values, checked_liquid


def _execution_values(
    runtime_schedule: RuntimeFractionalDelaySchedule,
    values: CommonStageParameterValues,
) -> tuple[float, ...]:
    result = (
        *values.actuator.execution_scalar_values,
        *runtime_schedule.duration,
        *(value for selector in runtime_schedule.selector_v for value in selector),
        *(value for selector in runtime_schedule.selector_omega for value in selector),
    )
    return _finite_float_tuple(result, "execution parameter values")


def _assign_block(
    row: list[float | None],
    layout: SolverParameterLayout,
    block_name: str,
    expected_names: tuple[str, ...],
    values: tuple[float, ...],
) -> None:
    block_range = layout.block_ranges[block_name]
    actual_names = layout.parameter_names[block_range.begin : block_range.end_exclusive]
    if actual_names != expected_names or len(values) != block_range.size:
        raise RuntimeParameterAssemblyError(
            f"{block_name} values do not match the sole typed layout"
        )
    checked_values = _finite_float_tuple(values, f"{block_name} values")
    for offset, value in zip(
        range(block_range.begin, block_range.end_exclusive),
        checked_values,
    ):
        if row[offset] is not None:
            raise RuntimeParameterAssemblyError(
                f"parameter offset {offset} was assigned more than once"
            )
        row[offset] = value


def _assemble_row(
    stage: int,
    layout: SolverParameterLayout,
    development_layout: DevelopmentLayout,
    execution_values: tuple[float, ...],
    values: CommonStageParameterValues,
    liquid_schedule: LiquidCostSchedule,
) -> tuple[float, ...]:
    row: list[float | None] = [None] * layout.NP
    blocks = (
        (
            "execution_prefix",
            development_layout.execution_parameter_names,
            execution_values,
        ),
        (
            "reference",
            REFERENCE_PARAMETER_ORDER,
            values.reference.values_for_stage(stage),
        ),
        ("slosh", SLOSH_PARAMETER_ORDER, values.slosh.ordered_values),
        (
            "normalization",
            NORMALIZATION_PARAMETER_ORDER,
            values.normalization.ordered_values,
        ),
        (
            "running_weight",
            RUNNING_WEIGHT_PARAMETER_ORDER,
            values.running_weight.ordered_values,
        ),
        (
            "terminal_weight",
            TERMINAL_WEIGHT_PARAMETER_ORDER,
            values.terminal_weight.ordered_values,
        ),
        (
            "stage_cost",
            STAGE_COST_PARAMETER_ORDER,
            (
                liquid_schedule.liquid_run_coeff[stage],
                liquid_schedule.liquid_boundary_coeff[stage],
            ),
        ),
    )
    for block_name, block_names, block_values in blocks:
        _assign_block(row, layout, block_name, block_names, block_values)
    if any(value is None for value in row):
        raise RuntimeParameterAssemblyError(
            f"stage {stage} does not explicitly assign every parameter"
        )
    return tuple(value for value in row if value is not None)


def assemble_runtime_stage_parameters(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    runtime_schedule: RuntimeFractionalDelaySchedule,
    values: CommonStageParameterValues,
    liquid_schedule: LiquidCostSchedule,
) -> RuntimeParameterSnapshot:
    """Build all ``p[0] .. p[N]`` rows from canonical typed inputs."""

    layout, checked_runtime, checked_values, checked_liquid = _checked_inputs(
        capacity,
        development_layout,
        runtime_schedule,
        values,
        liquid_schedule,
    )
    execution_values = _execution_values(checked_runtime, checked_values)
    rows = tuple(
        _assemble_row(
            stage,
            layout,
            development_layout,
            execution_values,
            checked_values,
            checked_liquid,
        )
        for stage in range(layout.parameter_vector_count)
    )
    if len(rows) != layout.parameter_vector_count or any(
        len(row) != layout.NP for row in rows
    ):
        raise RuntimeParameterAssemblyError(
            "assembled horizon does not match the typed layout dimensions"
        )
    terminal = rows[-1]
    if any(
        terminal[layout.parameter_offsets[name]] != 0.0
        for name in STAGE_COST_PARAMETER_ORDER
    ):
        raise RuntimeParameterAssemblyError(
            "terminal liquid coefficients must be identically zero"
        )
    return RuntimeParameterSnapshot(
        capacity_contract_sha256=layout.capacity_contract_sha256,
        development_layout_sha256=layout.development_layout_sha256,
        solver_parameter_layout_sha256=_semantic_sha256(
            layout.to_dict(),
            "solver parameter layout",
        ),
        runtime_schedule_sha256=checked_runtime.sha256,
        parameter_values_sha256=_semantic_sha256(
            checked_values.to_dict(),
            "common stage parameter values",
        ),
        liquid_cost_schedule_sha256=_semantic_sha256(
            checked_liquid.to_dict(),
            "liquid cost schedule",
        ),
        condition=checked_liquid.condition,
        parameter_names=layout.parameter_names,
        stage_parameters=rows,
        _construction_token=_SNAPSHOT_TOKEN,
    )


def runtime_parameter_snapshot_from_dict(
    value: Any,
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    runtime_schedule: RuntimeFractionalDelaySchedule,
    parameter_values: CommonStageParameterValues,
    liquid_schedule: LiquidCostSchedule,
) -> RuntimeParameterSnapshot:
    """Strictly round-trip a serialized horizon against all typed sources."""

    if type(value) is not dict:
        raise RuntimeParameterAssemblyError(
            "runtime parameter snapshot must be a JSON object"
        )
    expected_keys = {
        "schema_version",
        "scope",
        "capability_status",
        "source_identity",
        "experiment_condition",
        "dimensions",
        "parameter_order",
        "stage_parameters",
    }
    if set(value) != expected_keys:
        raise RuntimeParameterAssemblyError(
            "runtime parameter snapshot keys do not match the v1 schema"
        )
    rebuilt = assemble_runtime_stage_parameters(
        capacity,
        development_layout,
        runtime_schedule,
        parameter_values,
        liquid_schedule,
    )
    if not strict_json_equal(value, rebuilt.to_dict()):
        raise RuntimeParameterAssemblyError(
            "runtime parameter snapshot does not match its typed sources"
        )
    return rebuilt


def require_runtime_parameter_snapshot(
    value: Any,
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    runtime_schedule: RuntimeFractionalDelaySchedule,
    parameter_values: CommonStageParameterValues,
    liquid_schedule: LiquidCostSchedule,
) -> RuntimeParameterSnapshot:
    """Reject a force-mutated typed runtime parameter snapshot."""

    if type(value) is not RuntimeParameterSnapshot:
        raise RuntimeParameterAssemblyError(
            "snapshot must be the exact RuntimeParameterSnapshot type"
        )
    rebuilt = assemble_runtime_stage_parameters(
        capacity,
        development_layout,
        runtime_schedule,
        parameter_values,
        liquid_schedule,
    )
    if not strict_json_equal(value, rebuilt):
        raise RuntimeParameterAssemblyError(
            "typed runtime parameter snapshot is not canonical"
        )
    return value


def build_matched_runtime_parameter_pair(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    runtime_schedule: RuntimeFractionalDelaySchedule,
    parameter_values: CommonStageParameterValues,
    b0_liquid_schedule: LiquidCostSchedule,
    bslosh_liquid_schedule: LiquidCostSchedule,
) -> MatchedRuntimeParameterPair:
    """Build a fair B0/Bslosh pair with a two-field difference whitelist."""

    try:
        b0_schedule = require_liquid_cost_schedule(b0_liquid_schedule)
        bslosh_schedule = require_liquid_cost_schedule(bslosh_liquid_schedule)
    except LiquidCostScheduleError as exc:
        raise RuntimeParameterAssemblyError(
            "matched pair liquid schedules are not canonical"
        ) from exc
    if (
        b0_schedule.condition is not ExperimentCondition.B0
        or bslosh_schedule.condition is not ExperimentCondition.Bslosh
    ):
        raise RuntimeParameterAssemblyError(
            "matched pair requires B0 first and Bslosh second"
        )
    if (
        b0_schedule.K_liquid != bslosh_schedule.K_liquid
        or b0_schedule.W_run != bslosh_schedule.W_run
        or b0_schedule.W_boundary != bslosh_schedule.W_boundary
    ):
        raise RuntimeParameterAssemblyError(
            "matched pair must share K_liquid, W_run, and W_boundary"
        )

    b0 = assemble_runtime_stage_parameters(
        capacity,
        development_layout,
        runtime_schedule,
        parameter_values,
        b0_schedule,
    )
    bslosh = assemble_runtime_stage_parameters(
        capacity,
        development_layout,
        runtime_schedule,
        parameter_values,
        bslosh_schedule,
    )
    try:
        layout = build_solver_parameter_layout(capacity, development_layout)
    except SolverParameterLayoutError as exc:
        raise RuntimeParameterAssemblyError(
            "matched pair layout sources are not canonical"
        ) from exc
    allowed_offsets = tuple(
        layout.parameter_offsets[name] for name in ALLOWED_ARM_PARAMETER_DIFFERENCES
    )
    differences: list[tuple[int, int]] = []
    for stage, (b0_row, bslosh_row) in enumerate(
        zip(b0.stage_parameters, bslosh.stage_parameters)
    ):
        for offset, (b0_value, bslosh_value) in enumerate(zip(b0_row, bslosh_row)):
            if not strict_json_equal(b0_value, bslosh_value):
                if offset not in allowed_offsets:
                    raise RuntimeParameterAssemblyError(
                        "matched pair differs outside the liquid coefficient whitelist"
                    )
                differences.append((stage, offset))
    return MatchedRuntimeParameterPair(
        b0=b0,
        bslosh=bslosh,
        allowed_parameter_names=ALLOWED_ARM_PARAMETER_DIFFERENCES,
        allowed_parameter_offsets=allowed_offsets,
        differing_stage_offsets=tuple(differences),
        _construction_token=_PAIR_TOKEN,
    )


def require_matched_runtime_parameter_pair(
    value: Any,
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    runtime_schedule: RuntimeFractionalDelaySchedule,
    parameter_values: CommonStageParameterValues,
    b0_liquid_schedule: LiquidCostSchedule,
    bslosh_liquid_schedule: LiquidCostSchedule,
) -> MatchedRuntimeParameterPair:
    """Reject a force-mutated typed B0/Bslosh comparison pair."""

    if type(value) is not MatchedRuntimeParameterPair:
        raise RuntimeParameterAssemblyError(
            "pair must be the exact MatchedRuntimeParameterPair type"
        )
    rebuilt = build_matched_runtime_parameter_pair(
        capacity,
        development_layout,
        runtime_schedule,
        parameter_values,
        b0_liquid_schedule,
        bslosh_liquid_schedule,
    )
    if not strict_json_equal(value, rebuilt):
        raise RuntimeParameterAssemblyError(
            "typed matched runtime parameter pair is not canonical"
        )
    return value


__all__ = [
    "ALLOWED_ARM_PARAMETER_DIFFERENCES",
    "MATCHED_ARM_PAIR_SCHEMA_VERSION",
    "RUNTIME_PARAMETER_SNAPSHOT_SCHEMA_VERSION",
    "MatchedRuntimeParameterPair",
    "RuntimeParameterAssemblyError",
    "RuntimeParameterSnapshot",
    "assemble_runtime_stage_parameters",
    "build_matched_runtime_parameter_pair",
    "require_matched_runtime_parameter_pair",
    "require_runtime_parameter_snapshot",
    "runtime_parameter_snapshot_from_dict",
]
