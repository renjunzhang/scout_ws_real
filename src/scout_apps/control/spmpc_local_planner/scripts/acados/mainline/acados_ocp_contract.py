"""Dependency-free public contract for an assembled Stage 3-D Acados OCP."""

from __future__ import annotations

from dataclasses import InitVar, dataclass
from typing import Any

from .acados_ocp_schema import (
    ACADOS_D4_SOLVER_OPTION_FIELDS,
    ACADOS_INTERFACE_CONTRACT,
    ACADOS_INTERFACE_SOURCE_SCHEMA,
    ACADOS_OCP_SCHEMA,
    ACADOS_SOLVER_OPTIONS_BASELINE_SCHEMA,
    ACADOS_SOLVER_OPTIONS_BASELINE_SCOPE,
    ASSEMBLY_SIDE_EFFECT_POLICY,
    CODEGEN_OPTIONS_POLICY,
    CONTINUOUS_DYNAMICS_POLICY,
    DIAGNOSTIC_RESIDUAL_POLICY,
    DYNAMICS_IDENTITY_FUNCTION,
    INITIAL_STATE_POLICY,
    LIQUID_HARD_CONSTRAINT_POLICY,
    NODE_COVERAGE_POLICY,
    OCP_ARTIFACT_STATUS,
    OCP_IDENTITY_SCOPE,
    OCP_PROMOTION_STATUS,
    OCP_STATUS,
    PARAMETER_INITIALIZATION_POLICY,
    STAGE_COST_IDENTITY_FUNCTION,
    STAGE_H_IDENTITY_FUNCTION,
    SYMBOLIC_EXPRESSION_IDENTITY_SCHEMA,
    TERMINAL_COST_IDENTITY_FUNCTION,
    TERMINAL_H_IDENTITY_FUNCTION,
    UNLISTED_SOLVER_OPTIONS_POLICY,
    validate_acados_ocp_document,
)
from .identity import IdentityError, require_sha256, sha256_json
from .model_contract import MODEL_ID

_ASSEMBLY_TOKEN = object()


@dataclass(frozen=True)
class AcadosOcpAssembly:
    """Frozen metadata handle plus the mutable in-memory backend OCP object.

    ``ocp`` is intentionally excluded from serialization because backend
    objects contain expressions, paths, and mutable implementation details.
    The stable node mapping and all source identities are serialized instead.
    """

    ocp: Any
    horizon_steps: int
    nx: int
    nu: int
    np: int
    graph_semantic_sha256: str
    bounds_snapshot_sha256: str
    solver_options_semantic_sha256: str
    backend_solver_options_baseline_sha256: str
    capacity_contract_sha256: str
    development_layout_sha256: str
    solver_parameter_layout_sha256: str
    casadi_version: str
    acados_git_commit: str
    acados_interface_source_sha256: str
    acados_backend_binding_status: str
    dynamics_expression_sha256: str
    stage_cost_expression_sha256: str
    terminal_cost_expression_sha256: str
    stage_h_expression_sha256: str
    terminal_h_expression_sha256: str
    stage_constraint_order: tuple[str, ...]
    stage_constraint_lower: tuple[float, ...]
    stage_constraint_upper: tuple[float, ...]
    terminal_constraint_order: tuple[str, ...]
    terminal_constraint_lower: tuple[float, ...]
    terminal_constraint_upper: tuple[float, ...]
    control_order: tuple[str, ...]
    control_indices: tuple[int, ...]
    control_lower: tuple[float, ...]
    control_upper: tuple[float, ...]
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = ACADOS_OCP_SCHEMA
    model_id: str = MODEL_ID
    ocp_status: str = OCP_STATUS
    artifact_status: str = OCP_ARTIFACT_STATUS
    promotion_status: str = OCP_PROMOTION_STATUS

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _ASSEMBLY_TOKEN:
            raise ValueError("AcadosOcpAssembly requires assemble_acados_ocp")
        _validate_acados_ocp_assembly_structure(self)

    def to_dict(self) -> dict[str, Any]:
        """Serialize stable assembly metadata without backend object reprs."""

        payload = _assembly_payload(self)
        payload["semantic_identity"] = {
            "sha256": self.semantic_sha256,
            "scope": OCP_IDENTITY_SCOPE,
        }
        return payload


def _validate_acados_ocp_assembly_structure(assembly: AcadosOcpAssembly) -> None:
    if assembly.schema_version != ACADOS_OCP_SCHEMA or assembly.model_id != MODEL_ID:
        raise ValueError("Acados OCP schema/model identity is not canonical")
    if (
        assembly.ocp_status != OCP_STATUS
        or assembly.artifact_status != OCP_ARTIFACT_STATUS
        or assembly.promotion_status != OCP_PROMOTION_STATUS
    ):
        raise ValueError("Acados OCP status is not canonical")
    if assembly.horizon_steps <= 0 or min(assembly.nx, assembly.nu, assembly.np) <= 0:
        raise ValueError("Acados OCP dimensions must be positive")
    if assembly.acados_backend_binding_status not in {
        "MATCHED_SOURCE_ROOT",
        "UNRESOLVED_SOURCE_ROOT_MISMATCH",
    }:
        raise ValueError("Acados backend binding status is invalid")
    for order, lower, upper, label in (
        (
            assembly.stage_constraint_order,
            assembly.stage_constraint_lower,
            assembly.stage_constraint_upper,
            "stage constraint",
        ),
        (
            assembly.terminal_constraint_order,
            assembly.terminal_constraint_lower,
            assembly.terminal_constraint_upper,
            "terminal constraint",
        ),
        (
            assembly.control_order,
            assembly.control_lower,
            assembly.control_upper,
            "control",
        ),
    ):
        if len(order) == 0 or len(order) != len(lower) or len(order) != len(upper):
            raise ValueError(f"Acados OCP {label} dimensions are inconsistent")
    if len(assembly.control_indices) != len(assembly.control_order):
        raise ValueError("Acados OCP control indices are inconsistent")
    for name in (
        "graph_semantic_sha256",
        "bounds_snapshot_sha256",
        "solver_options_semantic_sha256",
        "backend_solver_options_baseline_sha256",
        "capacity_contract_sha256",
        "development_layout_sha256",
        "solver_parameter_layout_sha256",
        "acados_interface_source_sha256",
        "dynamics_expression_sha256",
        "stage_cost_expression_sha256",
        "terminal_cost_expression_sha256",
        "stage_h_expression_sha256",
        "terminal_h_expression_sha256",
        "semantic_sha256",
    ):
        try:
            require_sha256(getattr(assembly, name), f"Acados OCP {name}")
        except IdentityError as exc:
            raise ValueError("Acados OCP identity is invalid") from exc


def _assembly_payload(assembly: AcadosOcpAssembly) -> dict[str, Any]:
    return {
        "schema_version": assembly.schema_version,
        "model_id": assembly.model_id,
        "status": {
            "ocp": assembly.ocp_status,
            "artifact": assembly.artifact_status,
            "promotion": assembly.promotion_status,
        },
        "identity_scope": OCP_IDENTITY_SCOPE,
        "backend": {
            "interface_contract": ACADOS_INTERFACE_CONTRACT,
            "acados_git_commit": assembly.acados_git_commit,
            "interface_source_identity": {
                "schema": ACADOS_INTERFACE_SOURCE_SCHEMA,
                "sha256": assembly.acados_interface_source_sha256,
            },
            "python_interface_and_library_binding": (
                assembly.acados_backend_binding_status
            ),
            "casadi_version": assembly.casadi_version,
        },
        "source_identity": {
            "graph_semantic_sha256": assembly.graph_semantic_sha256,
            "bounds_snapshot_sha256": assembly.bounds_snapshot_sha256,
            "solver_options_semantic_sha256": (assembly.solver_options_semantic_sha256),
            "backend_solver_options_baseline": {
                "schema_version": ACADOS_SOLVER_OPTIONS_BASELINE_SCHEMA,
                "scope": ACADOS_SOLVER_OPTIONS_BASELINE_SCOPE,
                "excluded_d4_fields": list(ACADOS_D4_SOLVER_OPTION_FIELDS),
                "sha256": assembly.backend_solver_options_baseline_sha256,
            },
            "capacity_contract_raw_bytes_sha256": (assembly.capacity_contract_sha256),
            "development_layout_semantic_sha256": (assembly.development_layout_sha256),
            "solver_parameter_layout_semantic_sha256": (
                assembly.solver_parameter_layout_sha256
            ),
        },
        "dimensions": {
            "N": assembly.horizon_steps,
            "NX": assembly.nx,
            "NU": assembly.nu,
            "NP": assembly.np,
            "parameter_vector_count": assembly.horizon_steps + 1,
        },
        "model_mapping": {
            "dynamics": "model.disc_dyn_expr",
            "continuous_dynamics": CONTINUOUS_DYNAMICS_POLICY,
            "cost": {
                "initial": "model.cost_expr_ext_cost_0",
                "path": "model.cost_expr_ext_cost",
                "terminal": "model.cost_expr_ext_cost_e",
                "type": "EXTERNAL",
            },
            "constraints": {
                "initial": "model.con_h_expr_0",
                "path": "model.con_h_expr",
                "terminal": "model.con_h_expr_e",
                "type": "BGH",
            },
            "node_coverage": NODE_COVERAGE_POLICY,
        },
        "symbolic_expression_identity": {
            "serialization": SYMBOLIC_EXPRESSION_IDENTITY_SCHEMA,
            "dynamics": {
                "function_name": DYNAMICS_IDENTITY_FUNCTION,
                "sha256": assembly.dynamics_expression_sha256,
            },
            "stage_cost": {
                "function_name": STAGE_COST_IDENTITY_FUNCTION,
                "sha256": assembly.stage_cost_expression_sha256,
            },
            "terminal_cost": {
                "function_name": TERMINAL_COST_IDENTITY_FUNCTION,
                "sha256": assembly.terminal_cost_expression_sha256,
            },
            "stage_h": {
                "function_name": STAGE_H_IDENTITY_FUNCTION,
                "sha256": assembly.stage_h_expression_sha256,
            },
            "terminal_h": {
                "function_name": TERMINAL_H_IDENTITY_FUNCTION,
                "sha256": assembly.terminal_h_expression_sha256,
            },
        },
        "assembly_boundary": {
            "make_consistent": True,
            "side_effects": ASSEMBLY_SIDE_EFFECT_POLICY,
            "unlisted_solver_options": UNLISTED_SOLVER_OPTIONS_POLICY,
            "codegen_options": CODEGEN_OPTIONS_POLICY,
        },
        "constraints": {
            "initial_and_path_nonlinear_h": {
                "order": list(assembly.stage_constraint_order),
                "lower": list(assembly.stage_constraint_lower),
                "upper": list(assembly.stage_constraint_upper),
            },
            "terminal_nonlinear_h": {
                "order": list(assembly.terminal_constraint_order),
                "lower": list(assembly.terminal_constraint_lower),
                "upper": list(assembly.terminal_constraint_upper),
            },
            "control_box": {
                "order": list(assembly.control_order),
                "indices": list(assembly.control_indices),
                "lower": list(assembly.control_lower),
                "upper": list(assembly.control_upper),
            },
            "initial_state": INITIAL_STATE_POLICY,
            "diagnostic_residuals": DIAGNOSTIC_RESIDUAL_POLICY,
            "liquid_hard_constraints": LIQUID_HARD_CONSTRAINT_POLICY,
        },
        "runtime_parameters": {
            "initialization": PARAMETER_INITIALIZATION_POLICY,
            "required_stage_range_inclusive": [0, assembly.horizon_steps],
            "row_width": assembly.np,
        },
        "comparison_identity": {
            "arms": ["B0", "Bslosh"],
            "same_ocp": True,
            "future_artifact_policy": "ONE_SHARED_ARTIFACT_REQUIRED",
            "runtime_only_difference": [
                "liquid_run_coeff",
                "liquid_boundary_coeff",
            ],
        },
    }


def _build_acados_ocp_assembly(
    *,
    ocp: Any,
    horizon_steps: int,
    nx: int,
    nu: int,
    np: int,
    graph_semantic_sha256: str,
    bounds_snapshot_sha256: str,
    solver_options_semantic_sha256: str,
    backend_solver_options_baseline_sha256: str,
    capacity_contract_sha256: str,
    development_layout_sha256: str,
    solver_parameter_layout_sha256: str,
    casadi_version: str,
    acados_git_commit: str,
    acados_interface_source_sha256: str,
    acados_backend_binding_status: str,
    dynamics_expression_sha256: str,
    stage_cost_expression_sha256: str,
    terminal_cost_expression_sha256: str,
    stage_h_expression_sha256: str,
    terminal_h_expression_sha256: str,
    stage_constraint_order: tuple[str, ...],
    stage_constraint_lower: tuple[float, ...],
    stage_constraint_upper: tuple[float, ...],
    terminal_constraint_order: tuple[str, ...],
    terminal_constraint_lower: tuple[float, ...],
    terminal_constraint_upper: tuple[float, ...],
    control_order: tuple[str, ...],
    control_indices: tuple[int, ...],
    control_lower: tuple[float, ...],
    control_upper: tuple[float, ...],
) -> AcadosOcpAssembly:
    values = {
        "ocp": ocp,
        "horizon_steps": horizon_steps,
        "nx": nx,
        "nu": nu,
        "np": np,
        "graph_semantic_sha256": graph_semantic_sha256,
        "bounds_snapshot_sha256": bounds_snapshot_sha256,
        "solver_options_semantic_sha256": solver_options_semantic_sha256,
        "backend_solver_options_baseline_sha256": (
            backend_solver_options_baseline_sha256
        ),
        "capacity_contract_sha256": capacity_contract_sha256,
        "development_layout_sha256": development_layout_sha256,
        "solver_parameter_layout_sha256": solver_parameter_layout_sha256,
        "casadi_version": casadi_version,
        "acados_git_commit": acados_git_commit,
        "acados_interface_source_sha256": acados_interface_source_sha256,
        "acados_backend_binding_status": acados_backend_binding_status,
        "dynamics_expression_sha256": dynamics_expression_sha256,
        "stage_cost_expression_sha256": stage_cost_expression_sha256,
        "terminal_cost_expression_sha256": terminal_cost_expression_sha256,
        "stage_h_expression_sha256": stage_h_expression_sha256,
        "terminal_h_expression_sha256": terminal_h_expression_sha256,
        "stage_constraint_order": stage_constraint_order,
        "stage_constraint_lower": stage_constraint_lower,
        "stage_constraint_upper": stage_constraint_upper,
        "terminal_constraint_order": terminal_constraint_order,
        "terminal_constraint_lower": terminal_constraint_lower,
        "terminal_constraint_upper": terminal_constraint_upper,
        "control_order": control_order,
        "control_indices": control_indices,
        "control_lower": control_lower,
        "control_upper": control_upper,
    }
    provisional = AcadosOcpAssembly(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_ASSEMBLY_TOKEN,
    )
    semantic_sha256 = sha256_json(_assembly_payload(provisional))
    return AcadosOcpAssembly(
        **values,
        semantic_sha256=semantic_sha256,
        _construction_token=_ASSEMBLY_TOKEN,
    )


def require_acados_ocp_assembly(value: Any) -> AcadosOcpAssembly:
    """Reject wrong-type or force-mutated OCP assembly metadata."""

    if type(value) is not AcadosOcpAssembly:
        raise ValueError("assembly must be the exact AcadosOcpAssembly type")
    try:
        _validate_acados_ocp_assembly_structure(value)
        document = validate_acados_ocp_document(value.to_dict())
        semantic_sha256 = document["semantic_identity"]["sha256"]
    except (AttributeError, IdentityError, TypeError, ValueError) as exc:
        raise ValueError("Acados OCP assembly metadata is malformed") from exc
    if semantic_sha256 != value.semantic_sha256:
        raise ValueError("Acados OCP assembly semantic identity is inconsistent")
    return value


__all__ = [
    "ACADOS_INTERFACE_CONTRACT",
    "ACADOS_INTERFACE_SOURCE_SCHEMA",
    "ACADOS_OCP_SCHEMA",
    "ASSEMBLY_SIDE_EFFECT_POLICY",
    "CODEGEN_OPTIONS_POLICY",
    "CONTINUOUS_DYNAMICS_POLICY",
    "DIAGNOSTIC_RESIDUAL_POLICY",
    "DYNAMICS_IDENTITY_FUNCTION",
    "INITIAL_STATE_POLICY",
    "LIQUID_HARD_CONSTRAINT_POLICY",
    "NODE_COVERAGE_POLICY",
    "OCP_ARTIFACT_STATUS",
    "OCP_IDENTITY_SCOPE",
    "OCP_PROMOTION_STATUS",
    "OCP_STATUS",
    "PARAMETER_INITIALIZATION_POLICY",
    "STAGE_COST_IDENTITY_FUNCTION",
    "STAGE_H_IDENTITY_FUNCTION",
    "SYMBOLIC_EXPRESSION_IDENTITY_SCHEMA",
    "TERMINAL_COST_IDENTITY_FUNCTION",
    "TERMINAL_H_IDENTITY_FUNCTION",
    "UNLISTED_SOLVER_OPTIONS_POLICY",
    "AcadosOcpAssembly",
    "require_acados_ocp_assembly",
    "validate_acados_ocp_document",
]
