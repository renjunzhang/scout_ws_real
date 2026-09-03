"""Typed-authority binding and public API for the Stage 3-D4 artifact contract.

The serialized schema and content-identity implementation lives in
:mod:`artifact_contract_schema`.  This module only binds the typed D4
authorities, composes their payload, and exposes the immutable contract and
its JSON renderer.  It remains separate from the historical Stage 3A
``manifest`` module.
"""

from __future__ import annotations

import json
from dataclasses import InitVar, dataclass
from fractions import Fraction
from typing import Any

from .acados_codegen_backend import (
    AcadosCodegenError,
    AcadosCodegenResult,
    require_acados_codegen_result,
)
from .acados_ocp_contract import (
    AcadosOcpAssembly,
    require_acados_ocp_assembly,
)
from .artifact_contract_schema import (
    ARTIFACT_CLASS,
    ARTIFACT_CONTRACT_SCHEMA,
    ARTIFACT_CONTRACT_SCOPE,
    ARTIFACT_IDENTITY_SCHEMA,
    ARTIFACT_IDENTITY_SCOPE,
    ARTIFACT_STATUS,
    COMPARISON_ARMS,
    COMPARISON_RUNTIME_DIFFERENCES,
    CONTRACT_OUTPUT_POLICY,
    GENERATED_HEADER_BINDING_POLICY,
    NON_CIRCULAR_IDENTITY_POLICY,
    PROMOTION_STATUS,
    RUNTIME_PARAMETER_ASSIGNMENT_POLICY,
    TARGET_PERFORMANCE_STATUS,
    ArtifactContractError,
    _artifact_sha256,
    _require_object,
    _validate_serialized_contract,
)
from .artifact_files import validate_generated_tree
from .casadi_graph_contract import (
    CasadiGraphBundle,
    graph_semantic_sha256,
    require_casadi_graph_bundle,
)
from .codegen_options import (
    GENERATED_HEADER_FILENAME,
    MODEL_CONTRACT_FILENAME,
    RUNTIME_IDENTITY_EXCLUSIONS,
    CodegenOptionsSnapshot,
    require_codegen_options_snapshot,
)
from .constraints_oracle import CONSTRAINT_SCHEMA
from .development_capacity import (
    DevelopmentCapacityContract,
    require_pinned_development_capacity,
)
from .development_layout import (
    STAGE_SEMANTICS,
    DevelopmentLayout,
    validate_development_layout_snapshot,
)
from .identity import IdentityError, canonical_json, sha256_json
from .model_contract import COST_SCHEMA, DISCRETIZATION_SCHEMA, MODEL_ID
from .provenance import CodegenProvenance, require_codegen_provenance
from .provenance_common import ProvenanceError
from .runtime_schedule import RUNTIME_SCHEDULE_SCHEMA_VERSION
from .solver_options import SolverOptionsSnapshot, require_solver_options_snapshot
from .solver_parameter_layout import (
    REFERENCE_SCHEMA,
    SolverParameterLayout,
    solver_parameter_layout_from_dict,
)

_CONSTRUCTION_TOKEN = object()


def _fraction(value: Fraction) -> dict[str, int]:
    return {"numerator": value.numerator, "denominator": value.denominator}


def _layout_sha256(layout: DevelopmentLayout) -> str:
    try:
        return sha256_json(layout.to_dict())
    except (AttributeError, IdentityError, TypeError, ValueError) as exc:
        raise ArtifactContractError(
            "development layout identity cannot be encoded"
        ) from exc


def _parameter_layout_sha256(layout: SolverParameterLayout) -> str:
    try:
        return sha256_json(layout.to_dict())
    except (AttributeError, IdentityError, TypeError, ValueError) as exc:
        raise ArtifactContractError(
            "solver parameter layout identity cannot be encoded"
        ) from exc


def _checked_authorities(
    capacity: Any,
    development_layout: Any,
    parameter_layout: Any,
    graph: Any,
    assembly: Any,
    solver_options: Any,
    codegen_options: Any,
    provenance: Any,
    codegen_result: Any,
) -> tuple[
    DevelopmentCapacityContract,
    DevelopmentLayout,
    SolverParameterLayout,
    CasadiGraphBundle,
    AcadosOcpAssembly,
    SolverOptionsSnapshot,
    CodegenOptionsSnapshot,
    CodegenProvenance,
    AcadosCodegenResult,
]:
    """Validate every typed input and all cross-layer identity bindings."""

    try:
        checked_capacity = require_pinned_development_capacity(capacity)
        checked_layout = validate_development_layout_snapshot(
            development_layout,
            checked_capacity,
        )
        if type(parameter_layout) is not SolverParameterLayout:
            raise ArtifactContractError(
                "parameter_layout must be the exact SolverParameterLayout type"
            )
        checked_parameter_layout = solver_parameter_layout_from_dict(
            parameter_layout.to_dict(),
            checked_capacity,
            checked_layout,
        )
        checked_assembly = require_acados_ocp_assembly(assembly)
        checked_solver_options = require_solver_options_snapshot(solver_options)
        checked_codegen_options = require_codegen_options_snapshot(codegen_options)
        checked_provenance = require_codegen_provenance(provenance)
        checked_codegen_result = require_acados_codegen_result(codegen_result)
    except ArtifactContractError:
        raise
    except (
        AcadosCodegenError,
        AttributeError,
        ProvenanceError,
        TypeError,
        ValueError,
    ) as exc:
        raise ArtifactContractError("D4 typed authorities are malformed") from exc

    layout_sha256 = _layout_sha256(checked_layout)
    parameter_sha256 = _parameter_layout_sha256(checked_parameter_layout)
    try:
        checked_graph = require_casadi_graph_bundle(
            graph,
            expected_capacity_contract_raw_bytes_sha256=(
                checked_capacity.contract_sha256
            ),
            expected_development_layout_semantic_sha256=layout_sha256,
            expected_solver_parameter_layout_semantic_sha256=parameter_sha256,
            expected_state_order=checked_layout.state_names,
            expected_control_order=checked_layout.control_names,
            expected_parameter_order=checked_parameter_layout.parameter_names,
        )
    except (TypeError, ValueError) as exc:
        raise ArtifactContractError("CasADi graph authority is malformed") from exc
    from .constraints_oracle import CONTROL_BOX_ORDER

    expected_graph_sha256 = graph_semantic_sha256(
        capacity_contract_sha256=checked_capacity.contract_sha256,
        development_layout_sha256=layout_sha256,
        solver_parameter_layout_sha256=parameter_sha256,
        horizon_steps=checked_layout.horizon_steps,
        parameter_vector_count=checked_layout.horizon_steps + 1,
        nx=checked_layout.NX,
        nu=checked_layout.NU,
        np=checked_parameter_layout.NP,
        state_order=checked_layout.state_names,
        control_order=checked_layout.control_names,
        parameter_order=checked_parameter_layout.parameter_names,
        control_indices=tuple(
            checked_layout.control_offsets[name] for name in CONTROL_BOX_ORDER
        ),
    )
    dimensions = (
        checked_layout.horizon_steps,
        checked_layout.nx,
        checked_layout.nu,
        checked_parameter_layout.np,
    )
    if dimensions != (60, 48, 3, 162):
        raise ArtifactContractError("D4 dimensions must be N=60/NX=48/NU=3/NP=162")
    if (
        checked_parameter_layout.parameter_vector_count != 61
        or checked_layout.np_exec != 121
    ):
        raise ArtifactContractError("D4 parameter dimensions are inconsistent")
    if (
        checked_graph.state_order != checked_layout.state_names
        or checked_graph.control_order != checked_layout.control_names
        or checked_graph.parameter_order != checked_parameter_layout.parameter_names
    ):
        raise ArtifactContractError("graph orders differ from the typed layouts")
    if (
        checked_graph.capacity_contract_sha256 != checked_capacity.contract_sha256
        or checked_graph.development_layout_sha256 != layout_sha256
        or checked_graph.solver_parameter_layout_sha256 != parameter_sha256
        or checked_graph.graph_semantic_sha256 != expected_graph_sha256
        or (
            checked_graph.horizon_steps,
            checked_graph.nx,
            checked_graph.nu,
            checked_graph.np,
        )
        != dimensions
        or checked_graph.parameter_vector_count != 61
    ):
        raise ArtifactContractError("graph identity differs from its typed sources")
    if (
        checked_assembly.graph_semantic_sha256 != checked_graph.graph_semantic_sha256
        or checked_assembly.bounds_snapshot_sha256
        != sha256_json(checked_graph.bounds.to_dict())
        or checked_assembly.solver_options_semantic_sha256
        != checked_solver_options.semantic_sha256
        or checked_assembly.capacity_contract_sha256 != checked_capacity.contract_sha256
        or checked_assembly.development_layout_sha256 != layout_sha256
        or checked_assembly.solver_parameter_layout_sha256 != parameter_sha256
        or (
            checked_assembly.horizon_steps,
            checked_assembly.nx,
            checked_assembly.nu,
            checked_assembly.np,
        )
        != dimensions
    ):
        raise ArtifactContractError("OCP assembly differs from the graph/options")
    if checked_assembly.acados_backend_binding_status != "MATCHED_SOURCE_ROOT":
        raise ArtifactContractError("Acados Python interface and libraries are unbound")
    if (
        checked_solver_options.horizon_steps != checked_layout.horizon_steps
        or checked_solver_options.release_frequency_hz
        != checked_layout.release_frequency_hz
        or checked_solver_options.time_step_sec != checked_layout.release_period_sec
        or checked_codegen_options.horizon_steps != checked_layout.horizon_steps
        or checked_codegen_options.release_frequency_hz
        != checked_layout.release_frequency_hz
        or checked_codegen_options.release_period_sec
        != checked_layout.release_period_sec
        or checked_codegen_options.development_layout_sha256 != layout_sha256
    ):
        raise ArtifactContractError("solver/codegen options differ from the layout")
    if (
        checked_provenance.compiler_environment
        != checked_codegen_options.compiler_environment
        or checked_provenance.acados.commit_marker != checked_assembly.acados_git_commit
        or checked_provenance.python_runtime.packages[0].name != "casadi"
        or checked_provenance.python_runtime.packages[0].version
        != checked_assembly.casadi_version
    ):
        raise ArtifactContractError("provenance differs from the generated OCP")
    if checked_codegen_result.output_directory.name in {
        MODEL_CONTRACT_FILENAME,
        GENERATED_HEADER_FILENAME,
    }:
        raise ArtifactContractError("codegen output directory name is invalid")
    try:
        validate_generated_tree(
            checked_codegen_result.output_directory,
            checked_codegen_result.files,
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise ArtifactContractError(
            "codegen output directory differs from its recorded inventory"
        ) from exc

    return (
        checked_capacity,
        checked_layout,
        checked_parameter_layout,
        checked_graph,
        checked_assembly,
        checked_solver_options,
        checked_codegen_options,
        checked_provenance,
        checked_codegen_result,
    )


def _ordered_layouts(
    layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> dict[str, Any]:
    return {
        "state": {
            "count": layout.nx,
            "ordered": list(layout.state_names),
            "offsets": dict(layout.state_offsets),
        },
        "control": {
            "count": layout.nu,
            "ordered": list(layout.control_names),
            "offsets": dict(layout.control_offsets),
        },
        "parameter": {
            "count": parameter_layout.np,
            "ordered": list(parameter_layout.parameter_names),
            "offsets": dict(parameter_layout.parameter_offsets),
        },
        "parameter_blocks": {
            name: block.to_dict()
            for name, block in parameter_layout.block_ranges.items()
        },
    }


def _capacity_payload(capacity: DevelopmentCapacityContract) -> dict[str, Any]:
    return {
        "release_intervals": {"v": capacity.R_v, "omega": capacity.R_omega},
        "delay_state_count": {"v": capacity.D_v, "omega": capacity.D_omega},
        "selector_width": {"v": capacity.NQ_v, "omega": capacity.NQ_omega},
        "maximum_delay_sec": {
            "v": _fraction(capacity.v.l_max_sec),
            "omega": _fraction(capacity.omega.l_max_sec),
        },
        "execution_subsegment_slots": capacity.execution_subsegment_slots,
        "NP_exec": capacity.np_exec,
    }


def _runtime_schedule_payload(
    capacity: DevelopmentCapacityContract,
    codegen_options: CodegenOptionsSnapshot,
) -> dict[str, Any]:
    """Freeze schedule syntax and tolerances, never one trial's values."""

    return {
        "schema_version": RUNTIME_SCHEDULE_SCHEMA_VERSION,
        "time_grid": {
            "dt_sec": _fraction(capacity.release_period_sec),
            "release_frequency_hz": capacity.release_frequency_hz,
        },
        "channel_compilation": {
            "ratio": "r=delay_sec/dt_sec",
            "integer_part": "m=floor(r)",
            "fractional_part": "beta=r-m",
            "integer_snap": (
                "if beta<=snap_ratio or 1-beta<=snap_ratio, snap to adjacent integer"
            ),
            "selector_order": ["m+1", "m", "0"],
            "selector_encoding": "one_hot_fixed_width",
            "delay_range_policy": "0<=delay<=L_max",
        },
        "channels": {
            "v": {
                "release_intervals": capacity.R_v,
                "delay_state_count": capacity.D_v,
                "selector_width": capacity.NQ_v,
                "L_max_sec": _fraction(capacity.v.l_max_sec),
                "selector_parameter_prefix": "act_sel_v[slot][selector]",
            },
            "omega": {
                "release_intervals": capacity.R_omega,
                "delay_state_count": capacity.D_omega,
                "selector_width": capacity.NQ_omega,
                "L_max_sec": _fraction(capacity.omega.l_max_sec),
                "selector_parameter_prefix": "act_sel_omega[slot][selector]",
            },
        },
        "merged_schedule": {
            "slot_count": capacity.execution_subsegment_slots,
            "duration_parameter_prefix": "act_seg_dt[slot]",
            "duration_sum": "sum(act_seg_dt[:slot_count])==dt_sec",
            "switch_union": "sorted unique {0,v_switch,omega_switch,dt_sec}",
            "empty_slot_policy": "zero_duration_and_zero_selector",
        },
        "tolerances": {
            "integer_snap_tolerance_sec": codegen_options.integer_snap_tolerance_sec,
            "duration_tolerance_sec": codegen_options.duration_tolerance_sec,
        },
        "actual_schedule_values": "EXCLUDED_FROM_ARTIFACT_IDENTITY",
    }


def _artifact_payload(
    capacity: DevelopmentCapacityContract,
    layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
    graph: CasadiGraphBundle,
    assembly: AcadosOcpAssembly,
    solver_options: SolverOptionsSnapshot,
    codegen_options: CodegenOptionsSnapshot,
    provenance: CodegenProvenance,
    result: AcadosCodegenResult,
) -> dict[str, Any]:
    return {
        "schema_version": ARTIFACT_CONTRACT_SCHEMA,
        "scope": ARTIFACT_CONTRACT_SCOPE,
        "model_id": MODEL_ID,
        "status": {
            "artifact": ARTIFACT_STATUS,
            "artifact_class": ARTIFACT_CLASS,
            "promotion": PROMOTION_STATUS,
            "target_performance": TARGET_PERFORMANCE_STATUS,
        },
        "contract_outputs": {
            "policy": CONTRACT_OUTPUT_POLICY,
            "model_contract_json": MODEL_CONTRACT_FILENAME,
            "generated_cpp_header": GENERATED_HEADER_FILENAME,
            "generated_header_binding": GENERATED_HEADER_BINDING_POLICY,
            "non_circular_identity": NON_CIRCULAR_IDENTITY_POLICY,
        },
        "dimensions": {
            "N": layout.horizon_steps,
            "NX": layout.nx,
            "NU": layout.nu,
            "NP": parameter_layout.np,
            "NP_exec": layout.np_exec,
            "parameter_vector_count": parameter_layout.parameter_vector_count,
        },
        "release_grid": {
            "frequency_hz": layout.release_frequency_hz,
            "period_sec": _fraction(layout.release_period_sec),
            "stage_semantics": STAGE_SEMANTICS,
        },
        "development_capacity": _capacity_payload(capacity),
        "layouts": _ordered_layouts(layout, parameter_layout),
        "schemas": {
            "discretization": DISCRETIZATION_SCHEMA,
            "reference": REFERENCE_SCHEMA,
            "cost": COST_SCHEMA,
            "constraints": CONSTRAINT_SCHEMA,
            "runtime_fractional_delay_schedule": RUNTIME_SCHEDULE_SCHEMA_VERSION,
        },
        "runtime_schedule_contract": _runtime_schedule_payload(
            capacity, codegen_options
        ),
        "runtime_parameter_contract": {
            "assignment": RUNTIME_PARAMETER_ASSIGNMENT_POLICY,
            "required_stage_range_inclusive": [0, layout.horizon_steps],
            "row_width": parameter_layout.np,
            "runtime_identity_exclusions": list(RUNTIME_IDENTITY_EXCLUSIONS),
        },
        "comparison_contract": {
            "arms": list(COMPARISON_ARMS),
            "shared_artifact_required": True,
            "runtime_only_fields_allowed_to_differ": list(
                COMPARISON_RUNTIME_DIFFERENCES
            ),
            "graph_ocp_solver_codegen_and_layout_must_match": True,
        },
        "typed_authorities": {
            "development_capacity": {
                "raw_bytes_sha256": capacity.contract_sha256,
                "snapshot": capacity.to_dict(),
            },
            "development_layout": {
                "semantic_sha256": _layout_sha256(layout),
                "snapshot": layout.to_dict(),
            },
            "solver_parameter_layout": {
                "semantic_sha256": _parameter_layout_sha256(parameter_layout),
                "snapshot": parameter_layout.to_dict(),
            },
            "casadi_graph": graph.to_dict(),
            "acados_ocp": assembly.to_dict(),
            "solver_options": solver_options.to_dict(),
            "codegen_options": codegen_options.to_dict(),
            "provenance": provenance.to_dict(),
        },
        "artifact": result.to_dict(),
    }


@dataclass(frozen=True)
class ArtifactContract:
    """Frozen contract whose semantic payload no longer references backends."""

    _payload_json: str
    semantic_sha256: str
    artifact_sha256: str
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _CONSTRUCTION_TOKEN:
            raise ArtifactContractError(
                "ArtifactContract requires build_artifact_contract or parser"
            )
        _validate_contract_instance(self)

    def to_dict(self) -> dict[str, Any]:
        payload = json.loads(self._payload_json)
        payload["semantic_identity"] = {
            "sha256": self.semantic_sha256,
            "scope": ARTIFACT_CONTRACT_SCOPE,
        }
        payload["artifact_identity"] = {
            "schema_version": ARTIFACT_IDENTITY_SCHEMA,
            "sha256": self.artifact_sha256,
            "scope": ARTIFACT_IDENTITY_SCOPE,
        }
        return payload


def _validate_contract_instance(contract: ArtifactContract) -> dict[str, Any]:
    return _validate_serialized_contract(
        contract._payload_json,
        contract.semantic_sha256,
        contract.artifact_sha256,
    )


def build_artifact_contract(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
    graph: CasadiGraphBundle,
    assembly: AcadosOcpAssembly,
    solver_options: SolverOptionsSnapshot,
    codegen_options: CodegenOptionsSnapshot,
    provenance: CodegenProvenance,
    codegen_result: AcadosCodegenResult,
) -> ArtifactContract:
    """Bind all D4 typed authorities to one non-circular artifact contract."""

    checked = _checked_authorities(
        capacity,
        development_layout,
        parameter_layout,
        graph,
        assembly,
        solver_options,
        codegen_options,
        provenance,
        codegen_result,
    )
    payload = _artifact_payload(*checked)
    try:
        payload_json = canonical_json(payload)
        semantic_sha256 = sha256_json(payload)
        artifact_sha256 = _artifact_sha256(semantic_sha256, payload)
    except (IdentityError, KeyError, TypeError, ValueError) as exc:
        raise ArtifactContractError(
            "model contract identity cannot be encoded"
        ) from exc
    return ArtifactContract(
        payload_json,
        semantic_sha256,
        artifact_sha256,
        _construction_token=_CONSTRUCTION_TOKEN,
    )


def artifact_contract_from_dict(value: Any) -> ArtifactContract:
    """Strictly parse a serialized contract without loading numeric backends."""

    if type(value) is not dict:
        raise ArtifactContractError("model contract must be a JSON object")
    expected = {
        "schema_version",
        "scope",
        "model_id",
        "status",
        "contract_outputs",
        "dimensions",
        "release_grid",
        "development_capacity",
        "layouts",
        "schemas",
        "runtime_schedule_contract",
        "runtime_parameter_contract",
        "comparison_contract",
        "typed_authorities",
        "artifact",
        "semantic_identity",
        "artifact_identity",
    }
    if set(value) != expected:
        raise ArtifactContractError("model contract keys do not match the v1 schema")
    semantic = _require_object(
        value["semantic_identity"], {"sha256", "scope"}, "semantic_identity"
    )
    artifact = _require_object(
        value["artifact_identity"],
        {"schema_version", "sha256", "scope"},
        "artifact_identity",
    )
    if semantic["scope"] != ARTIFACT_CONTRACT_SCOPE:
        raise ArtifactContractError("model contract semantic scope drifted")
    if (
        artifact["schema_version"] != ARTIFACT_IDENTITY_SCHEMA
        or artifact["scope"] != ARTIFACT_IDENTITY_SCOPE
    ):
        raise ArtifactContractError("artifact identity schema/scope drifted")
    payload = {
        key: item
        for key, item in value.items()
        if key not in {"semantic_identity", "artifact_identity"}
    }
    try:
        payload_json = canonical_json(payload)
    except IdentityError as exc:
        raise ArtifactContractError("model contract is not canonicalizable") from exc
    return ArtifactContract(
        payload_json,
        semantic["sha256"],
        artifact["sha256"],
        _construction_token=_CONSTRUCTION_TOKEN,
    )


def require_artifact_contract(value: Any) -> ArtifactContract:
    """Reject wrong-type or force-mutated frozen contract objects."""

    if type(value) is not ArtifactContract:
        raise ArtifactContractError("contract must be the exact ArtifactContract type")
    _validate_contract_instance(value)
    return value


def render_model_contract_json(value: ArtifactContract) -> bytes:
    """Render deterministic UTF-8 bytes for ``model_contract.json``."""

    checked = require_artifact_contract(value)
    return (
        json.dumps(
            checked.to_dict(),
            ensure_ascii=False,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")


def validate_model_contract_json(
    value: ArtifactContract,
    rendered: Any,
) -> bytes:
    """Require candidate JSON bytes to equal the deterministic projection."""

    if type(rendered) is not bytes:
        raise ArtifactContractError("model contract JSON must be exact bytes")
    if rendered != render_model_contract_json(value):
        raise ArtifactContractError("model contract JSON is stale or modified")
    return rendered


__all__ = [
    "ARTIFACT_CLASS",
    "ARTIFACT_CONTRACT_SCHEMA",
    "ARTIFACT_CONTRACT_SCOPE",
    "ARTIFACT_IDENTITY_SCHEMA",
    "ARTIFACT_IDENTITY_SCOPE",
    "ARTIFACT_STATUS",
    "COMPARISON_ARMS",
    "COMPARISON_RUNTIME_DIFFERENCES",
    "GENERATED_HEADER_BINDING_POLICY",
    "NON_CIRCULAR_IDENTITY_POLICY",
    "PROMOTION_STATUS",
    "TARGET_PERFORMANCE_STATUS",
    "ArtifactContract",
    "ArtifactContractError",
    "artifact_contract_from_dict",
    "build_artifact_contract",
    "render_model_contract_json",
    "require_artifact_contract",
    "validate_model_contract_json",
]
