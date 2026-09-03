"""Dependency-free public contract for the Stage 3-D CasADi graph."""

from __future__ import annotations

from dataclasses import InitVar, dataclass
from fractions import Fraction
from typing import Any

from .constraints_oracle import (
    CONSTRAINT_RESIDUAL_ORDER,
    CONSTRAINT_SCHEMA,
    CONSTRAINT_VALUE_STATUS,
    CONTROL_BOX_ORDER,
    STAGE_NONLINEAR_H_ORDER,
    ConstraintBounds,
)
from .development_capacity import EXECUTION_SUBSEGMENT_SLOTS
from .development_layout import STAGE_SEMANTICS
from .identity import IdentityError, sha256_json
from .model_contract import COST_SCHEMA, DISCRETIZATION_SCHEMA, MODEL_ID
from .solver_parameter_layout import REFERENCE_SCHEMA

CASADI_GRAPH_SCHEMA = "spmpc_mainline_casadi_graph_v1"
GRAPH_STATUS = "GRAPH_BUILT"
ARTIFACT_STATUS = "NO_ARTIFACT"
TERMINAL_CONTROL_POLICY = "NO_U_N_ACCESS"
GRAPH_IDENTITY_SCOPE = "TYPED_GRAPH_STRUCTURE_ONLY_RUNTIME_P_AND_BOUND_VALUES_EXCLUDED"
BOUND_SNAPSHOT_SCOPE = "EXPLICIT_CALLER_SUPPLIED_DEV_UNVALIDATED"
DIAGNOSTIC_RESIDUAL_ROLE = "PARITY_AND_DIAGNOSTICS_ONLY_NOT_ACADOS_H"
RUNTIME_PARAMETER_INPUT_POLICY = (
    "CALLER_VALIDATES_CANONICAL_RUNTIME_SNAPSHOT_BEFORE_GRAPH_INJECTION"
)
RUNTIME_PARAMETER_VALUES_POLICY = "EXCLUDED_FROM_GRAPH_IDENTITY"
EXECUTION_SCHEDULE_POLICY = "FIXED_WIDTH_RUNTIME_VALUES_ALL_SLOTS_EXPANDED"
REFERENCE_DOMAIN_LOWER = 0.0
REFERENCE_DOMAIN_UPPER = 1.0
STAGE_REFERENCE_DOMAIN_ORDER = ("xi_k",)
TERMINAL_REFERENCE_DOMAIN_ORDER = ("xi_N",)
TERMINAL_INPUT_ORDER = ("x_N", "p_N")
TERMINAL_LIQUID_COST_POLICY = "IDENTICALLY_ZERO"
COMPARISON_ARMS = ("B0", "Bslosh")
COMPARISON_PARAMETER_FIELDS = (
    "liquid_run_coeff",
    "liquid_boundary_coeff",
)
COMPARISON_IDENTICAL_FIELDS = (
    "dynamics",
    "reference",
    "robot_cost",
    "constraints",
    "all_other_stage_parameters",
)
LIQUID_HARD_CONSTRAINT_POLICY = "DISABLED_FOR_B0_AND_BSLOSH"
_GRAPH_BUNDLE_TOKEN = object()


def graph_semantic_payload(
    *,
    capacity_contract_sha256: str,
    development_layout_sha256: str,
    solver_parameter_layout_sha256: str,
    horizon_steps: int,
    parameter_vector_count: int,
    nx: int,
    nu: int,
    np: int,
    state_order: tuple[str, ...],
    control_order: tuple[str, ...],
    parameter_order: tuple[str, ...],
    control_indices: tuple[int, ...],
) -> dict[str, Any]:
    """Return the dependency-light, structure-only graph identity payload.

    This is the single authority for the graph semantic hash.  It accepts
    plain serialized values so the typed graph builder and an offline artifact
    parser can execute exactly the same algorithm.  Backend version, bounds,
    and other runtime metadata deliberately do not belong to this payload;
    callers must bind those fields through their owning contracts.
    """

    return {
        "schema_version": CASADI_GRAPH_SCHEMA,
        "model_id": MODEL_ID,
        "source_identity": {
            "capacity_contract_raw_bytes_sha256": capacity_contract_sha256,
            "development_layout_semantic_sha256": development_layout_sha256,
            "solver_parameter_layout_semantic_sha256": solver_parameter_layout_sha256,
        },
        "dimensions": {
            "N": horizon_steps,
            "parameter_vector_count": parameter_vector_count,
            "NX": nx,
            "NU": nu,
            "NP": np,
        },
        "orders": {
            "state": list(state_order),
            "control": list(control_order),
            "parameter": list(parameter_order),
        },
        "schemas": {
            "discretization": DISCRETIZATION_SCHEMA,
            "reference": REFERENCE_SCHEMA,
            "cost": COST_SCHEMA,
            "constraints": CONSTRAINT_SCHEMA,
        },
        "runtime_parameter_input_policy": RUNTIME_PARAMETER_INPUT_POLICY,
        "stage_semantics": STAGE_SEMANTICS,
        "stage_nonlinear_h_order": list(STAGE_NONLINEAR_H_ORDER),
        "stage_nonlinear_h_bound_policy": "SYMMETRIC_EXPLICIT_CALLER_VALUES",
        "stage_reference_domain": {
            "order": list(STAGE_REFERENCE_DOMAIN_ORDER),
            "lower": [REFERENCE_DOMAIN_LOWER],
            "upper": [REFERENCE_DOMAIN_UPPER],
        },
        "terminal_reference_domain": {
            "order": list(TERMINAL_REFERENCE_DOMAIN_ORDER),
            "lower": [REFERENCE_DOMAIN_LOWER],
            "upper": [REFERENCE_DOMAIN_UPPER],
        },
        "control_box": {
            "indices": list(control_indices),
            "order": list(CONTROL_BOX_ORDER),
            "bound_policy": [
                "SYMMETRIC_EXPLICIT_JERK_V",
                "SYMMETRIC_EXPLICIT_JERK_OMEGA",
                "FIXED_ZERO_LOWER_EXPLICIT_UPPER_V_S",
            ],
            "v_s_fixed_lower": 0.0,
        },
        "diagnostic_residuals": {
            "role": DIAGNOSTIC_RESIDUAL_ROLE,
            "order": list(CONSTRAINT_RESIDUAL_ORDER),
            "feasible_upper": 0.0,
        },
        "terminal_policy": TERMINAL_CONTROL_POLICY,
        "liquid_hard_constraints": LIQUID_HARD_CONSTRAINT_POLICY,
        "comparison_identity": {
            "arms": list(COMPARISON_ARMS),
            "only_parameter_fields_allowed_to_differ": list(
                COMPARISON_PARAMETER_FIELDS
            ),
        },
    }


def graph_semantic_sha256(
    *,
    capacity_contract_sha256: str,
    development_layout_sha256: str,
    solver_parameter_layout_sha256: str,
    horizon_steps: int,
    parameter_vector_count: int,
    nx: int,
    nu: int,
    np: int,
    state_order: tuple[str, ...],
    control_order: tuple[str, ...],
    parameter_order: tuple[str, ...],
    control_indices: tuple[int, ...],
) -> str:
    """Hash :func:`graph_semantic_payload` without importing CasADi."""

    return sha256_json(
        graph_semantic_payload(
            capacity_contract_sha256=capacity_contract_sha256,
            development_layout_sha256=development_layout_sha256,
            solver_parameter_layout_sha256=solver_parameter_layout_sha256,
            horizon_steps=horizon_steps,
            parameter_vector_count=parameter_vector_count,
            nx=nx,
            nu=nu,
            np=np,
            state_order=state_order,
            control_order=control_order,
            parameter_order=parameter_order,
            control_indices=control_indices,
        )
    )


@dataclass(frozen=True)
class SymbolicIssuedValues:
    """Pre-issue command values shared by dynamics, cost, and constraints."""

    q_issue_v: Any
    q_issue_omega: Any
    a_issue: Any
    alpha_issue: Any


@dataclass(frozen=True)
class SymbolicReferenceValues:
    """Normalized local-cubic geometry and MPCC tracking errors."""

    xi: Any
    x_ref: Any
    y_ref: Any
    dx_ref_ds: Any
    dy_ref_ds: Any
    psi_ref: Any
    e_contour: Any
    e_lag: Any
    ref_speed: Any


@dataclass(frozen=True)
class SymbolicExecutionSlot:
    """One fixed-width physical-time ZOH slot in the discrete map."""

    duration_sec: Any
    q_target_v: Any
    q_target_omega: Any


@dataclass(frozen=True)
class SymbolicStageCosts:
    """Ordinary shooting-stage cost decomposition."""

    robot_running: Any
    liquid_running: Any
    liquid_boundary: Any
    total: Any


@dataclass(frozen=True)
class SymbolicStageConstraints:
    """Acados-ready stage constraints plus a diagnostic-only residual view."""

    nonlinear_h: Any
    nonlinear_h_order: tuple[str, ...]
    nonlinear_lower: tuple[float, ...]
    nonlinear_upper: tuple[float, ...]
    control_indices: tuple[int, ...]
    control_order: tuple[str, ...]
    control_lower: tuple[float, ...]
    control_upper: tuple[float, ...]
    reference_domain_h: Any
    reference_domain_order: tuple[str, ...]
    reference_domain_lower: tuple[float, ...]
    reference_domain_upper: tuple[float, ...]
    diagnostic_residual: Any
    diagnostic_residual_order: tuple[str, ...]
    diagnostic_feasible_upper: tuple[float, ...]


@dataclass(frozen=True)
class SymbolicTerminalExpressions:
    """Terminal-only graph; its expression tree must not contain ``u``."""

    reference: SymbolicReferenceValues
    total_cost: Any
    reference_domain_h: Any
    reference_domain_order: tuple[str, ...]
    reference_domain_lower: tuple[float, ...]
    reference_domain_upper: tuple[float, ...]
    input_order: tuple[str, ...] = TERMINAL_INPUT_ORDER
    control_policy: str = TERMINAL_CONTROL_POLICY


@dataclass(frozen=True)
class CasadiGraphBundle:
    """Immutable symbolic graph and metadata derived from typed authorities."""

    x: Any
    u: Any
    p: Any
    x_next: Any
    issued: SymbolicIssuedValues
    reference: SymbolicReferenceValues
    execution_slots: tuple[SymbolicExecutionSlot, ...]
    stage_costs: SymbolicStageCosts
    stage_constraints: SymbolicStageConstraints
    terminal: SymbolicTerminalExpressions
    state_order: tuple[str, ...]
    control_order: tuple[str, ...]
    parameter_order: tuple[str, ...]
    horizon_steps: int
    nx: int
    nu: int
    np: int
    capacity_contract_sha256: str
    development_layout_sha256: str
    solver_parameter_layout_sha256: str
    bounds: ConstraintBounds
    casadi_version: str
    graph_semantic_sha256: str
    parameter_vector_count: int
    release_frequency_hz: int
    release_period_sec: Fraction
    stage_semantics: str = STAGE_SEMANTICS
    _construction_token: InitVar[object] = None
    model_id: str = MODEL_ID
    graph_status: str = GRAPH_STATUS
    artifact_status: str = ARTIFACT_STATUS

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _GRAPH_BUNDLE_TOKEN:
            raise ValueError("CasadiGraphBundle requires build_casadi_graph")
        if self.model_id != MODEL_ID:
            raise ValueError("graph model identity is not canonical")
        if self.graph_status != GRAPH_STATUS or self.artifact_status != ARTIFACT_STATUS:
            raise ValueError("graph/artifact status is not canonical")
        if self.parameter_vector_count != self.horizon_steps + 1:
            raise ValueError("parameter vector count must equal N+1")
        if len(self.execution_slots) != EXECUTION_SUBSEGMENT_SLOTS:
            raise ValueError("graph execution slot count is not canonical")

    def to_dict(self) -> dict[str, Any]:
        """Serialize graph metadata without unstable CasADi expression reprs."""

        bounds_snapshot = self.bounds.to_dict()
        constraints = self.stage_constraints
        return {
            "schema_version": CASADI_GRAPH_SCHEMA,
            "status": {
                "graph": self.graph_status,
                "artifact": self.artifact_status,
            },
            "model_id": self.model_id,
            "casadi_version": self.casadi_version,
            "graph_identity_scope": GRAPH_IDENTITY_SCOPE,
            "runtime_parameter_values": RUNTIME_PARAMETER_VALUES_POLICY,
            "runtime_parameter_input_policy": RUNTIME_PARAMETER_INPUT_POLICY,
            "graph_semantic_identity": {
                "sha256": self.graph_semantic_sha256,
                "scope": GRAPH_IDENTITY_SCOPE,
            },
            "source_identity": {
                "capacity_contract_raw_bytes_sha256": (self.capacity_contract_sha256),
                "development_layout_semantic_sha256": (self.development_layout_sha256),
                "solver_parameter_layout_semantic_sha256": (
                    self.solver_parameter_layout_sha256
                ),
            },
            "dimensions": {
                "N": self.horizon_steps,
                "NX": self.nx,
                "NU": self.nu,
                "NP": self.np,
            },
            "horizon": {
                "N": self.horizon_steps,
                "parameter_vector_count": self.parameter_vector_count,
                "release_frequency_hz": self.release_frequency_hz,
                "release_period_sec": {
                    "numerator": self.release_period_sec.numerator,
                    "denominator": self.release_period_sec.denominator,
                },
            },
            "stage_semantics": self.stage_semantics,
            "orders": {
                "state": list(self.state_order),
                "control": list(self.control_order),
                "parameter": list(self.parameter_order),
            },
            "schemas": {
                "discretization": DISCRETIZATION_SCHEMA,
                "reference": REFERENCE_SCHEMA,
                "cost": COST_SCHEMA,
                "constraints": CONSTRAINT_SCHEMA,
            },
            "execution": {
                "slot_count": len(self.execution_slots),
                "schedule_policy": EXECUTION_SCHEDULE_POLICY,
            },
            "constraints": {
                "value_status": CONSTRAINT_VALUE_STATUS,
                "stage_nonlinear_h": {
                    "order": list(constraints.nonlinear_h_order),
                    "lower": list(constraints.nonlinear_lower),
                    "upper": list(constraints.nonlinear_upper),
                },
                "stage_reference_domain": {
                    "order": list(constraints.reference_domain_order),
                    "lower": list(constraints.reference_domain_lower),
                    "upper": list(constraints.reference_domain_upper),
                },
                "terminal_reference_domain": {
                    "order": list(self.terminal.reference_domain_order),
                    "lower": list(self.terminal.reference_domain_lower),
                    "upper": list(self.terminal.reference_domain_upper),
                },
                "control_box": {
                    "indices": list(constraints.control_indices),
                    "order": list(constraints.control_order),
                    "lower": list(constraints.control_lower),
                    "upper": list(constraints.control_upper),
                },
                "diagnostic_residuals": {
                    "role": DIAGNOSTIC_RESIDUAL_ROLE,
                    "order": list(constraints.diagnostic_residual_order),
                    "feasible_upper": list(constraints.diagnostic_feasible_upper),
                },
                "bounds_snapshot_scope": BOUND_SNAPSHOT_SCOPE,
                "bounds_snapshot": bounds_snapshot,
                "bounds_snapshot_sha256": sha256_json(bounds_snapshot),
            },
            "terminal": {
                "input_order": list(self.terminal.input_order),
                "control_policy": self.terminal.control_policy,
                "liquid_cost_policy": TERMINAL_LIQUID_COST_POLICY,
            },
            "comparison_identity": {
                "arms": list(COMPARISON_ARMS),
                "same_symbolic_graph": True,
                "only_parameter_fields_allowed_to_differ": list(
                    COMPARISON_PARAMETER_FIELDS
                ),
                "must_be_identical": list(COMPARISON_IDENTICAL_FIELDS),
                "liquid_hard_constraints": LIQUID_HARD_CONSTRAINT_POLICY,
            },
        }


class CasadiGraphContractError(ValueError):
    """A typed graph bundle no longer matches its serialized contract."""


def require_casadi_graph_bundle(
    value: Any,
    *,
    expected_capacity_contract_raw_bytes_sha256: str | None = None,
    expected_development_layout_semantic_sha256: str | None = None,
    expected_solver_parameter_layout_semantic_sha256: str | None = None,
    expected_state_order: tuple[str, ...] | None = None,
    expected_control_order: tuple[str, ...] | None = None,
    expected_parameter_order: tuple[str, ...] | None = None,
) -> CasadiGraphBundle:
    """Validate typed metadata through the dependency-free graph schema."""

    if type(value) is not CasadiGraphBundle:
        raise CasadiGraphContractError(
            "graph must have the exact CasadiGraphBundle type"
        )
    if (
        type(value.issued) is not SymbolicIssuedValues
        or type(value.reference) is not SymbolicReferenceValues
        or type(value.stage_costs) is not SymbolicStageCosts
        or type(value.stage_constraints) is not SymbolicStageConstraints
        or type(value.terminal) is not SymbolicTerminalExpressions
        or type(value.execution_slots) is not tuple
        or any(
            type(item) is not SymbolicExecutionSlot for item in value.execution_slots
        )
        or type(value.bounds) is not ConstraintBounds
        or type(value.release_period_sec) is not Fraction
    ):
        raise CasadiGraphContractError("graph typed records are malformed")
    try:
        from .casadi_graph_schema import validate_casadi_graph_document

        validate_casadi_graph_document(
            value.to_dict(),
            expected_capacity_contract_raw_bytes_sha256=(
                expected_capacity_contract_raw_bytes_sha256
            ),
            expected_development_layout_semantic_sha256=(
                expected_development_layout_semantic_sha256
            ),
            expected_solver_parameter_layout_semantic_sha256=(
                expected_solver_parameter_layout_semantic_sha256
            ),
            expected_state_order=expected_state_order,
            expected_control_order=expected_control_order,
            expected_parameter_order=expected_parameter_order,
            expected_casadi_version=value.casadi_version,
            expected_bounds_snapshot_sha256=sha256_json(value.bounds.to_dict()),
        )
    except (AttributeError, IdentityError, TypeError, ValueError) as exc:
        raise CasadiGraphContractError(
            "typed graph does not match the serialized graph contract"
        ) from exc
    return value


__all__ = [
    "ARTIFACT_STATUS",
    "BOUND_SNAPSHOT_SCOPE",
    "CASADI_GRAPH_SCHEMA",
    "COMPARISON_ARMS",
    "COMPARISON_IDENTICAL_FIELDS",
    "COMPARISON_PARAMETER_FIELDS",
    "DIAGNOSTIC_RESIDUAL_ROLE",
    "EXECUTION_SCHEDULE_POLICY",
    "GRAPH_IDENTITY_SCOPE",
    "GRAPH_STATUS",
    "LIQUID_HARD_CONSTRAINT_POLICY",
    "REFERENCE_DOMAIN_LOWER",
    "REFERENCE_DOMAIN_UPPER",
    "RUNTIME_PARAMETER_INPUT_POLICY",
    "RUNTIME_PARAMETER_VALUES_POLICY",
    "STAGE_REFERENCE_DOMAIN_ORDER",
    "STAGE_SEMANTICS",
    "TERMINAL_CONTROL_POLICY",
    "TERMINAL_INPUT_ORDER",
    "TERMINAL_LIQUID_COST_POLICY",
    "TERMINAL_REFERENCE_DOMAIN_ORDER",
    "CasadiGraphBundle",
    "CasadiGraphContractError",
    "SymbolicExecutionSlot",
    "SymbolicIssuedValues",
    "SymbolicReferenceValues",
    "SymbolicStageConstraints",
    "SymbolicStageCosts",
    "SymbolicTerminalExpressions",
    "graph_semantic_payload",
    "graph_semantic_sha256",
    "require_casadi_graph_bundle",
]
