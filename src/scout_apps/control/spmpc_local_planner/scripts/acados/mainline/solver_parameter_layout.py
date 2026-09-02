"""Full per-stage parameter order without a solver graph or artifact."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import InitVar, dataclass
from types import MappingProxyType
from typing import Any

from .development_capacity import (
    DevelopmentCapacityContract,
    DevelopmentCapacityError,
    require_pinned_development_capacity,
)
from .development_layout import (
    DEVELOPMENT_LAYOUT_SCHEMA_VERSION,
    DEVELOPMENT_LAYOUT_SCOPE,
    DevelopmentLayout,
    DevelopmentLayoutError,
    validate_development_layout_snapshot,
)
from .identity import IdentityError, sha256_json

SOLVER_PARAMETER_LAYOUT_SCHEMA_VERSION = (
    "spmpc_mainline_solver_parameter_layout_v1"
)
SOLVER_PARAMETER_LAYOUT_SCOPE = "FULL_STAGE_PARAMETER_ORDER"
GRAPH_STATUS = "NO_GRAPH"
ARTIFACT_STATUS = "NO_ARTIFACT"
REFERENCE_SCHEMA = "normalized_local_cubic_xy_v1"
TERMINAL_ASSIGNMENT_POLICY = (
    "ALL_FIELDS_EXPLICITLY_ASSIGNED_INCLUDING_UNUSED_TERMINAL_FIELDS"
)
REFERENCE_COORDINATE_FORMULA = "xi=(s-ref_s_origin)/ref_s_scale"
REFERENCE_POLYNOMIAL_BASIS = ("1", "xi", "xi^2", "xi^3")
REFERENCE_POSITION_FORMULA = "coord_ref=c0+c1*xi+c2*xi^2+c3*xi^3"
REFERENCE_TANGENT_FORMULA = (
    "dcoord_ref/ds=(c1+2*c2*xi+3*c3*xi^2)/ref_s_scale"
)
REFERENCE_HEADING_FORMULA = "atan2(dy_ref/ds,dx_ref/ds)"
REFERENCE_DOMAIN_POLICY = "REJECT_NO_CLAMP_NO_EXTRAPOLATION"
REFERENCE_SCALE_POLICY = "STRICTLY_POSITIVE"
REFERENCE_TANGENT_POLICY = (
    "ASSEMBLER_REQUIRES_POSITIVE_TANGENT_NORM_OVER_FULL_EFFECTIVE_DOMAIN"
)
REF_SPEED_ROLE = "NON_GEOMETRY_STAGE_VALUE"
REF_SPEED_SCHEDULE_POLICY = "EXPLICITLY_ASSIGNED_IN_EVERY_STAGE_VECTOR"
TERMINAL_STAGE_POLICY = "STATE_X_N_ONLY"
TERMINAL_CONTROL_POLICY = "NO_U_N_ACCESS"
TERMINAL_LIQUID_POLICY = "LIQUID_TERMINAL_COST_IDENTICALLY_ZERO"
LIQUID_BOUNDARY_POLICY = "BOUNDARY_COEFFICIENT_ONLY_AT_STAGE_K"
LIQUID_RUNNING_STATE_POLICY = "DISCRETE_MAP_RIGHT_ENDPOINT_X_K_PLUS_1"
LIQUID_BOUNDARY_STATE_POLICY = "SHOOTING_STATE_X_K"
LIQUID_RUNNING_RATIO_POLICY = (
    "PARAMETER_SLOSH_RUNNING_ETA_DOT_RATIO_ONLY_IN_RUNNING_COST"
)
LIQUID_BOUNDARY_RATIO_POLICY = "FIXED_ONE_INDEPENDENT_OF_RUNNING_RATIO"
SLOSH_RUNNING_RATIO_PARAMETER = "slosh_running_eta_dot_ratio"
RUNNING_OMEGA_POLICY = "NO_OMEGA_MAGNITUDE_TERM_HARD_BOUND_AND_TERMINAL_ONLY"

REFERENCE_PARAMETER_ORDER = (
    "ref_s_origin",
    "ref_s_scale",
    "ref_x_coeff[0]",
    "ref_x_coeff[1]",
    "ref_x_coeff[2]",
    "ref_x_coeff[3]",
    "ref_y_coeff[0]",
    "ref_y_coeff[1]",
    "ref_y_coeff[2]",
    "ref_y_coeff[3]",
    "ref_speed",
)
SLOSH_PARAMETER_ORDER = (
    "slosh_two_zeta_omega_n",
    "slosh_omega_n_sq",
    "slosh_kappa_x",
    "slosh_kappa_y",
    "slosh_eta_ref",
    "slosh_running_eta_dot_ratio",
)
NORMALIZATION_PARAMETER_ORDER = (
    "norm_contour",
    "norm_lag",
    "norm_v_actual",
    "norm_omega_actual",
    "norm_v_s",
    "norm_a_issue",
    "norm_alpha_issue",
    "norm_jerk_v",
    "norm_jerk_omega",
)
RUNNING_WEIGHT_PARAMETER_ORDER = (
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
TERMINAL_WEIGHT_PARAMETER_ORDER = (
    "weight_terminal_contour",
    "weight_terminal_lag",
    "weight_terminal_v_actual",
    "weight_terminal_omega_actual",
)
STAGE_COST_PARAMETER_ORDER = (
    "liquid_run_coeff",
    "liquid_boundary_coeff",
)
ADDITIONAL_PARAMETER_BLOCKS = (
    ("reference", REFERENCE_PARAMETER_ORDER),
    ("slosh", SLOSH_PARAMETER_ORDER),
    ("normalization", NORMALIZATION_PARAMETER_ORDER),
    ("running_weight", RUNNING_WEIGHT_PARAMETER_ORDER),
    ("terminal_weight", TERMINAL_WEIGHT_PARAMETER_ORDER),
    ("stage_cost", STAGE_COST_PARAMETER_ORDER),
)
PARAMETER_BLOCK_ORDER = (
    "execution_prefix",
    *(name for name, _ in ADDITIONAL_PARAMETER_BLOCKS),
)
FULL_STAGE_PARAMETER_COUNT = 162

_CONSTRUCTION_TOKEN = object()


class SolverParameterLayoutError(ValueError):
    """Raised when a source snapshot or full parameter ordering drifts."""


def _strict_equal(left: Any, right: Any) -> bool:
    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(
            _strict_equal(left[key], right[key]) for key in left
        )
    if type(left) is list:
        return len(left) == len(right) and all(
            _strict_equal(left_item, right_item)
            for left_item, right_item in zip(left, right)
        )
    return bool(left == right)


def _freeze_mapping(value: Mapping[str, Any]) -> Mapping[str, Any]:
    return MappingProxyType(dict(value))


def _checked_sources(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
) -> tuple[DevelopmentCapacityContract, DevelopmentLayout]:
    try:
        checked_capacity = require_pinned_development_capacity(capacity)
    except DevelopmentCapacityError as exc:
        raise SolverParameterLayoutError(
            "capacity does not match the complete pinned snapshot"
        ) from exc
    try:
        checked_layout = validate_development_layout_snapshot(
            development_layout,
            checked_capacity,
        )
    except DevelopmentLayoutError as exc:
        raise SolverParameterLayoutError(
            "development layout does not match its complete typed snapshot"
        ) from exc
    return checked_capacity, checked_layout


def _layout_identity_sha256(layout: DevelopmentLayout) -> str:
    try:
        return sha256_json(layout.to_dict())
    except IdentityError as exc:
        raise SolverParameterLayoutError(
            "development layout identity cannot be encoded"
        ) from exc


@dataclass(frozen=True)
class ParameterBlockRange:
    """Half-open immutable range in the common stage parameter vector."""

    begin: int
    end_exclusive: int

    @property
    def size(self) -> int:
        return self.end_exclusive - self.begin

    def to_dict(self) -> dict[str, int]:
        return {
            "begin": self.begin,
            "end_exclusive": self.end_exclusive,
        }


@dataclass(frozen=True)
class SolverParameterLayout:
    """Immutable common ordering for all N+1 parameter vectors."""

    capacity_contract_sha256: str
    development_layout_sha256: str
    horizon_steps: int
    parameter_vector_count: int
    np: int
    parameter_names: tuple[str, ...]
    parameter_offsets: Mapping[str, int]
    block_ranges: Mapping[str, ParameterBlockRange]
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _CONSTRUCTION_TOKEN:
            raise SolverParameterLayoutError(
                "SolverParameterLayout requires build_solver_parameter_layout"
            )

    @property
    def NP(self) -> int:
        return self.np

    def to_dict(self) -> dict[str, Any]:
        """Return the strict semantic identity of the full parameter order."""

        return {
            "schema_version": SOLVER_PARAMETER_LAYOUT_SCHEMA_VERSION,
            "scope": SOLVER_PARAMETER_LAYOUT_SCOPE,
            "capability_status": {
                "graph": GRAPH_STATUS,
                "artifact": ARTIFACT_STATUS,
            },
            "source_identity": {
                "development_capacity_raw_bytes_sha256": (
                    self.capacity_contract_sha256
                ),
                "development_layout": {
                    "schema_version": DEVELOPMENT_LAYOUT_SCHEMA_VERSION,
                    "scope": DEVELOPMENT_LAYOUT_SCOPE,
                    "semantic_sha256": self.development_layout_sha256,
                },
            },
            "reference_schema": REFERENCE_SCHEMA,
            "semantic_contracts": {
                "reference": {
                    "coordinate_formula": REFERENCE_COORDINATE_FORMULA,
                    "polynomial_basis": list(REFERENCE_POLYNOMIAL_BASIS),
                    "position_formula": REFERENCE_POSITION_FORMULA,
                    "tangent_formula": REFERENCE_TANGENT_FORMULA,
                    "heading_formula": REFERENCE_HEADING_FORMULA,
                    "effective_domain": {"lower": 0, "upper": 1},
                    "domain_policy": REFERENCE_DOMAIN_POLICY,
                    "scale_policy": REFERENCE_SCALE_POLICY,
                    "tangent_policy": REFERENCE_TANGENT_POLICY,
                    "ref_speed_role": REF_SPEED_ROLE,
                    "ref_speed_schedule_policy": REF_SPEED_SCHEDULE_POLICY,
                },
                "terminal": {
                    "stage_policy": TERMINAL_STAGE_POLICY,
                    "residual_order": [
                        "contour",
                        "lag",
                        "v_actual-ref_speed",
                        "omega_actual",
                    ],
                    "normalization_order": [
                        "norm_contour",
                        "norm_lag",
                        "norm_v_actual",
                        "norm_omega_actual",
                    ],
                    "control_policy": TERMINAL_CONTROL_POLICY,
                    "liquid_policy": TERMINAL_LIQUID_POLICY,
                },
                "running_cost": {
                    "residual_order": [
                        "contour",
                        "lag",
                        "negative_progress",
                        "v_actual-ref_speed",
                        "v_s-ref_speed",
                        "a_issue",
                        "alpha_issue",
                        "j_issue_v",
                        "j_issue_omega",
                    ],
                    "omega_actual_policy": RUNNING_OMEGA_POLICY,
                },
                "liquid_schedule": {
                    "running_state_policy": LIQUID_RUNNING_STATE_POLICY,
                    "boundary_policy": LIQUID_BOUNDARY_POLICY,
                    "boundary_state_policy": LIQUID_BOUNDARY_STATE_POLICY,
                    "running_eta_dot_ratio_policy": (
                        LIQUID_RUNNING_RATIO_POLICY
                    ),
                    "running_eta_dot_ratio_parameter": (
                        SLOSH_RUNNING_RATIO_PARAMETER
                    ),
                    "boundary_eta_dot_ratio_policy": (
                        LIQUID_BOUNDARY_RATIO_POLICY
                    ),
                },
                "comparison_identity": {
                    "arms": ["B0", "Bslosh"],
                    "only_parameter_fields_allowed_to_differ": [
                        "liquid_run_coeff",
                        "liquid_boundary_coeff",
                    ],
                    "must_be_identical": [
                        "dynamics",
                        "constraints",
                        "all_other_stage_parameters",
                    ],
                },
            },
            "stage_vector_contract": {
                "shooting_intervals": self.horizon_steps,
                "parameter_vector_count": self.parameter_vector_count,
                "common_width": self.np,
                "common_order_for_all_stages": True,
                "terminal_assignment_policy": TERMINAL_ASSIGNMENT_POLICY,
            },
            "parameter_layout": {
                "ordered": list(self.parameter_names),
                "offsets": dict(self.parameter_offsets),
                "block_order": list(PARAMETER_BLOCK_ORDER),
                "block_ranges": {
                    name: block_range.to_dict()
                    for name, block_range in self.block_ranges.items()
                },
                "dimension": self.np,
            },
        }


def build_solver_parameter_layout(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
) -> SolverParameterLayout:
    """Build the sole full stage-parameter order from pinned typed inputs."""

    checked_capacity, checked_layout = _checked_sources(
        capacity,
        development_layout,
    )
    parameter_names = list(checked_layout.execution_parameter_names)
    block_ranges: dict[str, ParameterBlockRange] = {
        "execution_prefix": ParameterBlockRange(0, len(parameter_names))
    }
    for block_name, block_names in ADDITIONAL_PARAMETER_BLOCKS:
        begin = len(parameter_names)
        parameter_names.extend(block_names)
        block_ranges[block_name] = ParameterBlockRange(begin, len(parameter_names))

    if len(parameter_names) != FULL_STAGE_PARAMETER_COUNT:
        raise SolverParameterLayoutError(
            "full parameter expansion does not equal NP=162"
        )
    if len(set(parameter_names)) != FULL_STAGE_PARAMETER_COUNT:
        raise SolverParameterLayoutError("full parameter names must be unique")
    parameter_offsets = {
        name: index for index, name in enumerate(parameter_names)
    }
    if tuple(block_ranges) != PARAMETER_BLOCK_ORDER:
        raise SolverParameterLayoutError("parameter block order drifted")
    previous_end = 0
    for block_range in block_ranges.values():
        if block_range.begin != previous_end or block_range.size <= 0:
            raise SolverParameterLayoutError(
                "parameter block ranges must be positive and contiguous"
            )
        previous_end = block_range.end_exclusive
    if previous_end != FULL_STAGE_PARAMETER_COUNT:
        raise SolverParameterLayoutError(
            "parameter block ranges do not cover the complete vector"
        )

    return SolverParameterLayout(
        capacity_contract_sha256=checked_capacity.contract_sha256,
        development_layout_sha256=_layout_identity_sha256(checked_layout),
        horizon_steps=checked_layout.horizon_steps,
        parameter_vector_count=checked_layout.horizon_steps + 1,
        np=FULL_STAGE_PARAMETER_COUNT,
        parameter_names=tuple(parameter_names),
        parameter_offsets=_freeze_mapping(parameter_offsets),
        block_ranges=_freeze_mapping(block_ranges),
        _construction_token=_CONSTRUCTION_TOKEN,
    )


def solver_parameter_layout_from_dict(
    value: Any,
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
) -> SolverParameterLayout:
    """Strictly round-trip a layout against both complete typed snapshots."""

    if type(value) is not dict:
        raise SolverParameterLayoutError(
            "solver parameter layout must be a JSON object"
        )
    expected_keys = {
        "schema_version",
        "scope",
        "capability_status",
        "source_identity",
        "reference_schema",
        "semantic_contracts",
        "stage_vector_contract",
        "parameter_layout",
    }
    if set(value) != expected_keys:
        raise SolverParameterLayoutError(
            "solver parameter layout keys do not match the v1 schema"
        )
    rebuilt = build_solver_parameter_layout(capacity, development_layout)
    if not _strict_equal(value, rebuilt.to_dict()):
        raise SolverParameterLayoutError(
            "solver parameter layout does not exactly match its typed sources"
        )
    return rebuilt
