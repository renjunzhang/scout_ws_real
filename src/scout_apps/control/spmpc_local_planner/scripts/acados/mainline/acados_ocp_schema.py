"""Dependency-free serialized schema for the Stage 3-D Acados OCP."""

from __future__ import annotations

import math
from typing import Any

from .acados_solver_options_identity import (
    ACADOS_D4_SOLVER_OPTION_FIELDS,
    ACADOS_SOLVER_OPTIONS_BASELINE_SCHEMA,
    ACADOS_SOLVER_OPTIONS_BASELINE_SCOPE,
)
from .casadi_graph_contract import (
    STAGE_REFERENCE_DOMAIN_ORDER,
    TERMINAL_REFERENCE_DOMAIN_ORDER,
)
from .constraints_oracle import CONTROL_BOX_ORDER, STAGE_NONLINEAR_H_ORDER
from .identity import IdentityError, require_sha256, sha256_json, strict_json_equal
from .model_contract import MODEL_ID

ACADOS_OCP_SCHEMA = "spmpc_mainline_acados_ocp_v1"
ACADOS_INTERFACE_CONTRACT = "acados_template_v0.5.4_compatible"
ACADOS_INTERFACE_SOURCE_SCHEMA = "selected_acados_template_python_sources_v1"
ACADOS_INTERFACE_SOURCE_PATHS = (
    "__init__.py",
    "acados_code_gen_opts.py",
    "acados_model.py",
    "acados_ocp.py",
    "acados_ocp_constraints.py",
    "acados_ocp_cost.py",
    "acados_ocp_options.py",
)
SYMBOLIC_EXPRESSION_IDENTITY_SCHEMA = "casadi_function_serialize_v1"
DYNAMICS_IDENTITY_FUNCTION = "mainline_identity_discrete_dynamics"
STAGE_COST_IDENTITY_FUNCTION = "mainline_identity_stage_cost"
TERMINAL_COST_IDENTITY_FUNCTION = "mainline_identity_terminal_cost"
STAGE_H_IDENTITY_FUNCTION = "mainline_identity_stage_h"
TERMINAL_H_IDENTITY_FUNCTION = "mainline_identity_terminal_h"
OCP_STATUS = "OCP_ASSEMBLED_AND_CONSISTENT"
OCP_ARTIFACT_STATUS = "NO_ARTIFACT"
OCP_PROMOTION_STATUS = "DEV_UNVALIDATED"
OCP_IDENTITY_SCOPE = "GRAPH_BOUNDS_SOLVER_OPTIONS_NODE_MAPPING_AND_BACKEND"
INITIAL_STATE_POLICY = "FULL_NX_ZERO_PLACEHOLDER_RUNTIME_REPLACES_LBX_UBX"
PARAMETER_INITIALIZATION_POLICY = (
    "ZERO_PLACEHOLDER_RUNTIME_MUST_SET_ALL_N_PLUS_ONE_ROWS"
)
NODE_COVERAGE_POLICY = "EXPLICIT_INITIAL_PATH_AND_TERMINAL_FIELDS"
CONTINUOUS_DYNAMICS_POLICY = "FORBIDDEN_DISCRETE_MAP_ONLY"
DIAGNOSTIC_RESIDUAL_POLICY = "NOT_CONNECTED_TO_ACADOS_CONSTRAINTS"
LIQUID_HARD_CONSTRAINT_POLICY = "DISABLED_FOR_B0_AND_BSLOSH"
UNLISTED_SOLVER_OPTIONS_POLICY = (
    "FROZEN_BY_COMPLETE_BACKEND_OPTIONS_IDENTITY_EXCEPT_D4_FIELDS"
)
CODEGEN_OPTIONS_POLICY = "OUT_OF_SCOPE_UNTIL_ARTIFACT_GENERATION_STAGE"
ASSEMBLY_SIDE_EFFECT_POLICY = "READS_BACKEND_METADATA_NO_CODEGEN_OR_OUTPUT_WRITES"


def _document_object(value: Any, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != keys:
        raise ValueError(f"{label} keys do not match the v1 schema")
    return value


def _document_text(value: Any, label: str) -> str:
    if type(value) is not str or not value or "\x00" in value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _document_digest(value: Any, label: str) -> str:
    try:
        return require_sha256(value, label)
    except IdentityError as exc:
        raise ValueError(str(exc)) from exc


def acados_interface_source_sha256_from_inventory(value: Any) -> str:
    """Project a full interface inventory onto the OCP-selected source set."""

    if type(value) is not dict or any(type(path) is not str for path in value):
        raise ValueError("Acados interface source inventory must be a string map")
    missing = set(ACADOS_INTERFACE_SOURCE_PATHS).difference(value)
    if missing:
        raise ValueError("Acados interface source inventory is incomplete")
    files = []
    for path in ACADOS_INTERFACE_SOURCE_PATHS:
        files.append(
            {
                "relative_path": path,
                "raw_bytes_sha256": _document_digest(
                    value[path], f"Acados interface source {path}"
                ),
            }
        )
    return sha256_json(
        {
            "schema": ACADOS_INTERFACE_SOURCE_SCHEMA,
            "files": files,
        }
    )


def _constraint_document(
    value: Any,
    *,
    label: str,
    with_indices: bool,
) -> tuple[list[str], list[float], list[float], list[int] | None]:
    keys = (
        {"order", "lower", "upper", "indices"}
        if with_indices
        else {
            "order",
            "lower",
            "upper",
        }
    )
    document = _document_object(value, keys, label)
    order = document["order"]
    lower = document["lower"]
    upper = document["upper"]
    if (
        type(order) is not list
        or not order
        or any(type(item) is not str or not item for item in order)
        or len(set(order)) != len(order)
        or type(lower) is not list
        or type(upper) is not list
        or len(lower) != len(order)
        or len(upper) != len(order)
    ):
        raise ValueError(f"{label} dimensions or order are invalid")
    for bound in (*lower, *upper):
        if type(bound) is not float or not math.isfinite(bound):
            raise ValueError(f"{label} bounds must be explicit finite floats")
    indices: list[int] | None = None
    if with_indices:
        indices = document["indices"]
        if (
            type(indices) is not list
            or len(indices) != len(order)
            or any(type(index) is not int or index < 0 for index in indices)
            or len(set(indices)) != len(indices)
        ):
            raise ValueError(f"{label} indices are invalid")
    return order, lower, upper, indices


def validate_acados_ocp_document(
    value: Any,
    *,
    require_matched_source_root: bool = False,
) -> dict[str, Any]:
    """Validate one serialized assembly without loading CasADi or Acados.

    Backend versions, expression bytes, bounds, and source hashes are captured
    observations.  Their values may vary, but their schemas and identities are
    strict.  Model mapping, node coverage, initialization, and comparison
    policies are fixed D3/D4 authorities and cannot be changed by re-signing a
    JSON document with another public SHA-256.
    """

    document = _document_object(
        value,
        {
            "schema_version",
            "model_id",
            "status",
            "identity_scope",
            "backend",
            "source_identity",
            "dimensions",
            "model_mapping",
            "symbolic_expression_identity",
            "assembly_boundary",
            "constraints",
            "runtime_parameters",
            "comparison_identity",
            "semantic_identity",
        },
        "Acados OCP document",
    )
    if not strict_json_equal(document["schema_version"], ACADOS_OCP_SCHEMA):
        raise ValueError("Acados OCP document schema drifted")
    if not strict_json_equal(document["model_id"], MODEL_ID):
        raise ValueError("Acados OCP document model identity drifted")
    if not strict_json_equal(
        document["status"],
        {
            "ocp": OCP_STATUS,
            "artifact": OCP_ARTIFACT_STATUS,
            "promotion": OCP_PROMOTION_STATUS,
        },
    ):
        raise ValueError("Acados OCP document status drifted")
    if not strict_json_equal(document["identity_scope"], OCP_IDENTITY_SCOPE):
        raise ValueError("Acados OCP document identity scope drifted")

    backend = _document_object(
        document["backend"],
        {
            "interface_contract",
            "acados_git_commit",
            "interface_source_identity",
            "python_interface_and_library_binding",
            "casadi_version",
        },
        "Acados OCP backend",
    )
    if not strict_json_equal(backend["interface_contract"], ACADOS_INTERFACE_CONTRACT):
        raise ValueError("Acados interface contract drifted")
    commit = _document_text(backend["acados_git_commit"], "Acados Git marker")
    if (
        len(commit) < 7
        or len(commit) > 40
        or any(character not in "0123456789abcdef" for character in commit)
    ):
        raise ValueError("Acados Git marker must be lowercase hexadecimal")
    interface_source = _document_object(
        backend["interface_source_identity"],
        {"schema", "sha256"},
        "Acados interface source identity",
    )
    if not strict_json_equal(
        interface_source["schema"], ACADOS_INTERFACE_SOURCE_SCHEMA
    ):
        raise ValueError("Acados interface source schema drifted")
    _document_digest(interface_source["sha256"], "Acados interface source identity")
    binding = backend["python_interface_and_library_binding"]
    if binding not in {"MATCHED_SOURCE_ROOT", "UNRESOLVED_SOURCE_ROOT_MISMATCH"}:
        raise ValueError("Acados source-root binding status is invalid")
    if require_matched_source_root and binding != "MATCHED_SOURCE_ROOT":
        raise ValueError("Acados source root is not matched")
    _document_text(backend["casadi_version"], "CasADi version")

    source = _document_object(
        document["source_identity"],
        {
            "graph_semantic_sha256",
            "bounds_snapshot_sha256",
            "solver_options_semantic_sha256",
            "backend_solver_options_baseline",
            "capacity_contract_raw_bytes_sha256",
            "development_layout_semantic_sha256",
            "solver_parameter_layout_semantic_sha256",
        },
        "Acados OCP source identity",
    )
    for name in (
        "graph_semantic_sha256",
        "bounds_snapshot_sha256",
        "solver_options_semantic_sha256",
        "capacity_contract_raw_bytes_sha256",
        "development_layout_semantic_sha256",
        "solver_parameter_layout_semantic_sha256",
    ):
        _document_digest(source[name], f"Acados OCP {name}")
    baseline = _document_object(
        source["backend_solver_options_baseline"],
        {"schema_version", "scope", "excluded_d4_fields", "sha256"},
        "Acados backend solver-options baseline",
    )
    if (
        not strict_json_equal(
            baseline["schema_version"], ACADOS_SOLVER_OPTIONS_BASELINE_SCHEMA
        )
        or not strict_json_equal(
            baseline["scope"], ACADOS_SOLVER_OPTIONS_BASELINE_SCOPE
        )
        or not strict_json_equal(
            baseline["excluded_d4_fields"], list(ACADOS_D4_SOLVER_OPTION_FIELDS)
        )
    ):
        raise ValueError("Acados backend solver-options baseline policy drifted")
    _document_digest(baseline["sha256"], "Acados backend solver-options baseline")

    dimensions = _document_object(
        document["dimensions"],
        {"N", "NX", "NU", "NP", "parameter_vector_count"},
        "Acados OCP dimensions",
    )
    if not strict_json_equal(
        dimensions,
        {
            "N": 60,
            "NX": 48,
            "NU": 3,
            "NP": 162,
            "parameter_vector_count": 61,
        },
    ):
        raise ValueError("Acados OCP dimensions drifted from the D4 model")

    if not strict_json_equal(
        document["model_mapping"],
        {
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
    ):
        raise ValueError("Acados OCP model mapping drifted")

    expressions = _document_object(
        document["symbolic_expression_identity"],
        {
            "serialization",
            "dynamics",
            "stage_cost",
            "terminal_cost",
            "stage_h",
            "terminal_h",
        },
        "Acados symbolic expression identity",
    )
    if not strict_json_equal(
        expressions["serialization"], SYMBOLIC_EXPRESSION_IDENTITY_SCHEMA
    ):
        raise ValueError("Acados expression serialization policy drifted")
    for key, function_name in (
        ("dynamics", DYNAMICS_IDENTITY_FUNCTION),
        ("stage_cost", STAGE_COST_IDENTITY_FUNCTION),
        ("terminal_cost", TERMINAL_COST_IDENTITY_FUNCTION),
        ("stage_h", STAGE_H_IDENTITY_FUNCTION),
        ("terminal_h", TERMINAL_H_IDENTITY_FUNCTION),
    ):
        identity = _document_object(
            expressions[key],
            {"function_name", "sha256"},
            f"Acados {key} expression identity",
        )
        if not strict_json_equal(identity["function_name"], function_name):
            raise ValueError(f"Acados {key} identity function drifted")
        _document_digest(identity["sha256"], f"Acados {key} expression identity")

    if not strict_json_equal(
        document["assembly_boundary"],
        {
            "make_consistent": True,
            "side_effects": ASSEMBLY_SIDE_EFFECT_POLICY,
            "unlisted_solver_options": UNLISTED_SOLVER_OPTIONS_POLICY,
            "codegen_options": CODEGEN_OPTIONS_POLICY,
        },
    ):
        raise ValueError("Acados OCP assembly boundary drifted")

    constraints = _document_object(
        document["constraints"],
        {
            "initial_and_path_nonlinear_h",
            "terminal_nonlinear_h",
            "control_box",
            "initial_state",
            "diagnostic_residuals",
            "liquid_hard_constraints",
        },
        "Acados OCP constraints",
    )
    stage_order, stage_lower, stage_upper, _ = _constraint_document(
        constraints["initial_and_path_nonlinear_h"],
        label="Acados initial/path constraints",
        with_indices=False,
    )
    terminal_order, terminal_lower, terminal_upper, _ = _constraint_document(
        constraints["terminal_nonlinear_h"],
        label="Acados terminal constraints",
        with_indices=False,
    )
    control_order, control_lower, control_upper, control_indices = _constraint_document(
        constraints["control_box"],
        label="Acados control box",
        with_indices=True,
    )
    if (
        not strict_json_equal(
            stage_order,
            list(STAGE_NONLINEAR_H_ORDER) + list(STAGE_REFERENCE_DOMAIN_ORDER),
        )
        or len(stage_lower) != 5
        or any(
            lower != -upper or upper <= 0.0
            for lower, upper in zip(stage_lower[:4], stage_upper[:4])
        )
        or not strict_json_equal(stage_lower[4:], [0.0])
        or not strict_json_equal(stage_upper[4:], [1.0])
        or not strict_json_equal(terminal_order, list(TERMINAL_REFERENCE_DOMAIN_ORDER))
        or not strict_json_equal(terminal_lower, [0.0])
        or not strict_json_equal(terminal_upper, [1.0])
        or not strict_json_equal(control_order, list(CONTROL_BOX_ORDER))
        or not strict_json_equal(control_indices, [0, 1, 2])
        or any(
            lower != -upper or upper <= 0.0
            for lower, upper in zip(control_lower[:2], control_upper[:2])
        )
        or not strict_json_equal(control_lower[2:], [0.0])
        or control_upper[2] <= 0.0
    ):
        raise ValueError("Acados OCP constraint layout or bound policy drifted")
    if (
        not strict_json_equal(constraints["initial_state"], INITIAL_STATE_POLICY)
        or not strict_json_equal(
            constraints["diagnostic_residuals"], DIAGNOSTIC_RESIDUAL_POLICY
        )
        or not strict_json_equal(
            constraints["liquid_hard_constraints"], LIQUID_HARD_CONSTRAINT_POLICY
        )
    ):
        raise ValueError("Acados OCP constraint policy drifted")

    if not strict_json_equal(
        document["runtime_parameters"],
        {
            "initialization": PARAMETER_INITIALIZATION_POLICY,
            "required_stage_range_inclusive": [0, dimensions["N"]],
            "row_width": dimensions["NP"],
        },
    ):
        raise ValueError("Acados runtime parameter policy drifted")
    if not strict_json_equal(
        document["comparison_identity"],
        {
            "arms": ["B0", "Bslosh"],
            "same_ocp": True,
            "future_artifact_policy": "ONE_SHARED_ARTIFACT_REQUIRED",
            "runtime_only_difference": [
                "liquid_run_coeff",
                "liquid_boundary_coeff",
            ],
        },
    ):
        raise ValueError("Acados B0/Bslosh comparison identity drifted")

    semantic = _document_object(
        document["semantic_identity"],
        {"sha256", "scope"},
        "Acados OCP semantic identity",
    )
    if not strict_json_equal(semantic["scope"], OCP_IDENTITY_SCOPE):
        raise ValueError("Acados OCP semantic scope drifted")
    _document_digest(semantic["sha256"], "Acados OCP semantic identity")
    payload = {
        key: item for key, item in document.items() if key != "semantic_identity"
    }
    try:
        actual_sha256 = sha256_json(payload)
    except IdentityError as exc:
        raise ValueError("Acados OCP document is not canonicalizable") from exc
    if semantic["sha256"] != actual_sha256:
        raise ValueError("Acados OCP document semantic identity is inconsistent")
    return document


__all__ = [
    "ACADOS_INTERFACE_CONTRACT",
    "ACADOS_INTERFACE_SOURCE_PATHS",
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
    "acados_interface_source_sha256_from_inventory",
    "validate_acados_ocp_document",
]
