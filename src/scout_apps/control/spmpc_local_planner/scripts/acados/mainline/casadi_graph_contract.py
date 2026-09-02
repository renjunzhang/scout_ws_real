"""Dependency-free public contract for the Stage 3-D CasADi graph."""

from __future__ import annotations

from dataclasses import InitVar, dataclass
from fractions import Fraction
from typing import Any

from .constraints_oracle import (
    CONSTRAINT_SCHEMA,
    CONSTRAINT_VALUE_STATUS,
    ConstraintBounds,
)
from .development_layout import STAGE_SEMANTICS
from .identity import sha256_json
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
REFERENCE_DOMAIN_LOWER = 0.0
REFERENCE_DOMAIN_UPPER = 1.0
STAGE_REFERENCE_DOMAIN_ORDER = ("xi_k",)
TERMINAL_REFERENCE_DOMAIN_ORDER = ("xi_N",)
_GRAPH_BUNDLE_TOKEN = object()


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
    input_order: tuple[str, ...] = ("x_N", "p_N")
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
            "runtime_parameter_values": "EXCLUDED_FROM_GRAPH_IDENTITY",
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
                "schedule_policy": ("FIXED_WIDTH_RUNTIME_VALUES_ALL_SLOTS_EXPANDED"),
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
                "liquid_cost_policy": "IDENTICALLY_ZERO",
            },
            "comparison_identity": {
                "arms": ["B0", "Bslosh"],
                "same_symbolic_graph": True,
                "only_parameter_fields_allowed_to_differ": [
                    "liquid_run_coeff",
                    "liquid_boundary_coeff",
                ],
                "must_be_identical": [
                    "dynamics",
                    "reference",
                    "robot_cost",
                    "constraints",
                    "all_other_stage_parameters",
                ],
                "liquid_hard_constraints": "DISABLED_FOR_B0_AND_BSLOSH",
            },
        }


__all__ = [
    "ARTIFACT_STATUS",
    "BOUND_SNAPSHOT_SCOPE",
    "CASADI_GRAPH_SCHEMA",
    "DIAGNOSTIC_RESIDUAL_ROLE",
    "GRAPH_IDENTITY_SCOPE",
    "GRAPH_STATUS",
    "REFERENCE_DOMAIN_LOWER",
    "REFERENCE_DOMAIN_UPPER",
    "RUNTIME_PARAMETER_INPUT_POLICY",
    "STAGE_REFERENCE_DOMAIN_ORDER",
    "STAGE_SEMANTICS",
    "TERMINAL_CONTROL_POLICY",
    "TERMINAL_REFERENCE_DOMAIN_ORDER",
    "CasadiGraphBundle",
    "SymbolicExecutionSlot",
    "SymbolicIssuedValues",
    "SymbolicReferenceValues",
    "SymbolicStageConstraints",
    "SymbolicStageCosts",
    "SymbolicTerminalExpressions",
]
