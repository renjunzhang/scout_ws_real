"""Serialized schema and content-identity checks for the D4 artifact contract.

This module contains no typed-authority assembly and no public artifact
object.  It validates canonical serialized payloads, embedded child
identities, generated-file inventories, and the non-circular artifact hash.
The higher-level :mod:`artifact_contract` module owns authority binding and
the public ``ArtifactContract`` API.
"""

from __future__ import annotations

import json
import math
from typing import Any

from .acados_codegen_result_schema import (
    validate_acados_codegen_result_document,
)
from .acados_ocp_schema import (
    acados_interface_source_sha256_from_inventory,
    validate_acados_ocp_document,
)
from .casadi_graph_contract import (
    COMPARISON_ARMS,
    COMPARISON_PARAMETER_FIELDS,
)
from .casadi_graph_schema import validate_casadi_graph_document
from .codegen_options import (
    GENERATED_HEADER_FILENAME,
    MODEL_CONTRACT_FILENAME,
    RUNTIME_IDENTITY_EXCLUSIONS,
    validate_codegen_options_document,
)
from .constraints_oracle import CONSTRAINT_SCHEMA
from .development_capacity import (
    DEVELOPMENT_ARTIFACT_CLASS,
    DEVELOPMENT_CAPACITY_SHA256,
    development_capacity_from_dict,
)
from .development_layout import STAGE_SEMANTICS, development_layout_from_dict
from .identity import IdentityError, canonical_json, require_sha256, sha256_json
from .model_contract import COST_SCHEMA, DISCRETIZATION_SCHEMA, MODEL_ID
from .provenance_common import ProvenanceError
from .provenance_schema import (
    PROVENANCE_ARTIFACT_CLASS,
    PROVENANCE_PROMOTION_STATUS,
    PROVENANCE_STATUS,
    validate_codegen_provenance_document,
)
from .runtime_schedule import RUNTIME_SCHEDULE_SCHEMA_VERSION
from .solver_options import validate_solver_options_document
from .solver_parameter_layout import (
    REFERENCE_SCHEMA,
    solver_parameter_layout_from_dict,
)

ARTIFACT_CONTRACT_SCHEMA = "spmpc_mainline_model_contract_v1"
ARTIFACT_CONTRACT_SCOPE = "TYPED_MODEL_OCP_TOOLCHAIN_AND_GENERATED_ARTIFACT_IDENTITY"
ARTIFACT_IDENTITY_SCHEMA = "spmpc_mainline_artifact_identity_v1"
ARTIFACT_IDENTITY_SCOPE = (
    "MODEL_CONTRACT_SEMANTIC_SHA256_PLUS_GENERATED_TREE_AND_SOLVER_LIBRARY"
)
ARTIFACT_STATUS = "GENERATED_AND_BUILT"
ARTIFACT_CLASS = DEVELOPMENT_ARTIFACT_CLASS
PROMOTION_STATUS = "NOT_PROMOTED"
TARGET_PERFORMANCE_STATUS = "NOT_BENCHMARKED"

CONTRACT_OUTPUT_POLICY = "DETERMINISTIC_PROJECTIONS_OF_ONE_TYPED_CONTRACT"
GENERATED_HEADER_BINDING_POLICY = "EMBEDS_MODEL_CONTRACT_SEMANTIC_AND_ARTIFACT_SHA256"
NON_CIRCULAR_IDENTITY_POLICY = (
    "MODEL_CONTRACT_JSON_AND_GENERATED_HEADER_EXCLUDED_FROM_GENERATED_TREE"
)
RUNTIME_PARAMETER_ASSIGNMENT_POLICY = (
    "ALL_N_PLUS_ONE_ROWS_AND_ALL_NP_FIELDS_EXPLICITLY_ASSIGNED"
)
COMPARISON_RUNTIME_DIFFERENCES = COMPARISON_PARAMETER_FIELDS


class ArtifactContractError(ValueError):
    """A serialized D4 contract or identity is inconsistent."""


def _require_object(value: Any, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != keys:
        raise ArtifactContractError(f"{label} keys do not match the v1 schema")
    return value


def _strict_equal(left: Any, right: Any) -> bool:
    """Compare JSON values without Python's bool/int equivalence."""

    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(
            _strict_equal(left[key], right[key]) for key in left
        )
    if type(left) is list:
        return len(left) == len(right) and all(
            _strict_equal(item, other) for item, other in zip(left, right)
        )
    return bool(left == right)


def _require_order_and_offsets(
    value: Any,
    *,
    label: str,
    expected_count: int,
) -> tuple[tuple[str, ...], dict[str, int]]:
    entry = _require_object(
        value,
        {"count", "ordered", "offsets"},
        label,
    )
    if entry["count"] != expected_count:
        raise ArtifactContractError(f"{label}.count is inconsistent")
    ordered = entry["ordered"]
    offsets = entry["offsets"]
    if (
        type(ordered) is not list
        or len(ordered) != expected_count
        or any(type(item) is not str or not item for item in ordered)
        or len(set(ordered)) != expected_count
    ):
        raise ArtifactContractError(f"{label}.ordered is invalid")
    if type(offsets) is not dict or set(offsets) != set(ordered):
        raise ArtifactContractError(f"{label}.offsets do not cover the order")
    if any(
        type(offsets[name]) is not int or offsets[name] != index
        for index, name in enumerate(ordered)
    ):
        raise ArtifactContractError(f"{label}.offsets are not contiguous")
    return tuple(ordered), offsets


def _validate_graph_ocp_binding(
    graph: dict[str, Any],
    ocp: dict[str, Any],
) -> None:
    """Validate only relationships jointly owned by graph and OCP."""

    graph_constraints = graph["constraints"]
    ocp_source = ocp["source_identity"]
    if (
        ocp_source["bounds_snapshot_sha256"]
        != graph_constraints["bounds_snapshot_sha256"]
    ):
        raise ArtifactContractError("graph/OCP bounds identity drifted")

    graph_stage = graph_constraints["stage_nonlinear_h"]
    graph_stage_domain = graph_constraints["stage_reference_domain"]
    graph_terminal_domain = graph_constraints["terminal_reference_domain"]
    graph_control = graph_constraints["control_box"]
    ocp_constraints = ocp["constraints"]
    ocp_stage = ocp_constraints["initial_and_path_nonlinear_h"]
    ocp_terminal = ocp_constraints["terminal_nonlinear_h"]
    ocp_control = ocp_constraints["control_box"]
    stage_width = len(graph_stage["order"])
    if not all(
        (
            _strict_equal(
                ocp_stage["order"],
                graph_stage["order"] + graph_stage_domain["order"],
            ),
            _strict_equal(graph_stage["lower"], ocp_stage["lower"][:stage_width]),
            _strict_equal(graph_stage["upper"], ocp_stage["upper"][:stage_width]),
            _strict_equal(
                graph_stage_domain["lower"], ocp_stage["lower"][stage_width:]
            ),
            _strict_equal(
                graph_stage_domain["upper"], ocp_stage["upper"][stage_width:]
            ),
            _strict_equal(graph_terminal_domain["order"], ocp_terminal["order"]),
            _strict_equal(graph_terminal_domain["lower"], ocp_terminal["lower"]),
            _strict_equal(graph_terminal_domain["upper"], ocp_terminal["upper"]),
            _strict_equal(graph_control["order"], ocp_control["order"]),
            _strict_equal(graph_control["indices"], ocp_control["indices"]),
            _strict_equal(graph_control["lower"], ocp_control["lower"]),
            _strict_equal(graph_control["upper"], ocp_control["upper"]),
        )
    ):
        raise ArtifactContractError("embedded graph constraints differ from OCP")


def _validate_provenance_bindings(
    provenance: dict[str, Any],
    ocp: dict[str, Any],
    codegen_options: dict[str, Any],
) -> None:
    """Bind captured dependencies to the OCP and codegen authorities."""

    status = provenance["status"]
    if (
        status["provenance"] != PROVENANCE_STATUS
        or status["artifact_class"] != PROVENANCE_ARTIFACT_CLASS
        or status["promotion"] != PROVENANCE_PROMOTION_STATUS
        or status["artifact_class"] != ARTIFACT_CLASS
        or status["promotion"] != PROMOTION_STATUS
    ):
        raise ArtifactContractError("embedded provenance status drifted")

    casadi_package = provenance["python_runtime"]["packages"][0]
    if (
        casadi_package["version"] != ocp["backend"]["casadi_version"]
        or provenance["acados"]["commit_marker"] != ocp["backend"]["acados_git_commit"]
        or not _strict_equal(
            provenance["compiler_environment"],
            codegen_options["acados_codegen"]["build"]["compiler_environment"],
        )
    ):
        raise ArtifactContractError(
            "embedded provenance differs from OCP/codegen options"
        )

    interface_files = provenance["acados"]["interface_tree"]["files"]
    interface_inventory = {
        item["relative_path"]: item["raw_sha256"] for item in interface_files
    }
    try:
        interface_sha256 = acados_interface_source_sha256_from_inventory(
            interface_inventory
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ArtifactContractError(
            "embedded provenance Acados interface inventory is malformed"
        ) from exc
    if interface_sha256 != ocp["backend"]["interface_source_identity"]["sha256"]:
        raise ArtifactContractError(
            "Acados OCP interface identity differs from provenance"
        )


def _artifact_sha256(semantic_sha256: str, payload: dict[str, Any]) -> str:
    """Compute the non-circular artifact identity from contract and binaries."""

    artifact = payload["artifact"]
    return sha256_json(
        {
            "schema_version": ARTIFACT_IDENTITY_SCHEMA,
            "model_contract_semantic_sha256": semantic_sha256,
            "generated_tree_sha256": artifact["generated_tree"]["sha256"],
            "solver_library_raw_sha256": artifact["solver_library"]["raw_sha256"],
        }
    )


def _validate_payload(value: Any) -> dict[str, Any]:
    """Validate a complete serialized contract and all child identities."""

    payload = _require_object(
        value,
        {
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
        },
        "model contract payload",
    )
    if (
        payload["schema_version"] != ARTIFACT_CONTRACT_SCHEMA
        or payload["scope"] != ARTIFACT_CONTRACT_SCOPE
        or payload["model_id"] != MODEL_ID
    ):
        raise ArtifactContractError("model contract schema/model identity drifted")
    status = _require_object(
        payload["status"],
        {"artifact", "artifact_class", "promotion", "target_performance"},
        "status",
    )
    if not _strict_equal(
        status,
        {
            "artifact": ARTIFACT_STATUS,
            "artifact_class": ARTIFACT_CLASS,
            "promotion": PROMOTION_STATUS,
            "target_performance": TARGET_PERFORMANCE_STATUS,
        },
    ):
        raise ArtifactContractError("model contract status is not development-only")
    outputs = _require_object(
        payload["contract_outputs"],
        {
            "policy",
            "model_contract_json",
            "generated_cpp_header",
            "generated_header_binding",
            "non_circular_identity",
        },
        "contract_outputs",
    )
    if not _strict_equal(
        outputs,
        {
            "policy": CONTRACT_OUTPUT_POLICY,
            "model_contract_json": MODEL_CONTRACT_FILENAME,
            "generated_cpp_header": GENERATED_HEADER_FILENAME,
            "generated_header_binding": GENERATED_HEADER_BINDING_POLICY,
            "non_circular_identity": NON_CIRCULAR_IDENTITY_POLICY,
        },
    ):
        raise ArtifactContractError("contract output policy drifted")

    dimensions = _require_object(
        payload["dimensions"],
        {"N", "NX", "NU", "NP", "NP_exec", "parameter_vector_count"},
        "dimensions",
    )
    if not _strict_equal(
        dimensions,
        {
            "N": 60,
            "NX": 48,
            "NU": 3,
            "NP": 162,
            "NP_exec": 121,
            "parameter_vector_count": 61,
        },
    ):
        raise ArtifactContractError("model contract dimensions drifted")
    release_grid = _require_object(
        payload["release_grid"],
        {"frequency_hz", "period_sec", "stage_semantics"},
        "release_grid",
    )
    if not _strict_equal(
        release_grid,
        {
            "frequency_hz": 30,
            "period_sec": {"numerator": 1, "denominator": 30},
            "stage_semantics": STAGE_SEMANTICS,
        },
    ):
        raise ArtifactContractError("release grid drifted")

    capacity = _require_object(
        payload["development_capacity"],
        {
            "release_intervals",
            "delay_state_count",
            "selector_width",
            "maximum_delay_sec",
            "execution_subsegment_slots",
            "NP_exec",
        },
        "development_capacity",
    )
    if (
        not _strict_equal(capacity["release_intervals"], {"v": 12, "omega": 24})
        or not _strict_equal(capacity["delay_state_count"], {"v": 11, "omega": 23})
        or not _strict_equal(capacity["selector_width"], {"v": 13, "omega": 25})
        or not _strict_equal(
            capacity["maximum_delay_sec"],
            {
                "v": {"numerator": 2, "denominator": 5},
                "omega": {"numerator": 4, "denominator": 5},
            },
        )
        or capacity["execution_subsegment_slots"] != 3
        or capacity["NP_exec"] != 121
    ):
        raise ArtifactContractError("development capacity drifted")

    layouts = _require_object(
        payload["layouts"],
        {"state", "control", "parameter", "parameter_blocks"},
        "layouts",
    )
    state_order, _ = _require_order_and_offsets(
        layouts["state"], label="layouts.state", expected_count=48
    )
    control_order, _ = _require_order_and_offsets(
        layouts["control"], label="layouts.control", expected_count=3
    )
    parameter_order, _ = _require_order_and_offsets(
        layouts["parameter"], label="layouts.parameter", expected_count=162
    )
    if control_order != ("j_issue_v", "j_issue_omega", "v_s"):
        raise ArtifactContractError("control order drifted")
    if state_order[-4:] != ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot"):
        raise ArtifactContractError("liquid state order drifted")
    if parameter_order[-2:] != COMPARISON_RUNTIME_DIFFERENCES:
        raise ArtifactContractError("liquid coefficient offsets drifted")
    blocks = layouts["parameter_blocks"]
    expected_blocks = {
        "execution_prefix": {"begin": 0, "end_exclusive": 121},
        "reference": {"begin": 121, "end_exclusive": 132},
        "slosh": {"begin": 132, "end_exclusive": 138},
        "normalization": {"begin": 138, "end_exclusive": 147},
        "running_weight": {"begin": 147, "end_exclusive": 156},
        "terminal_weight": {"begin": 156, "end_exclusive": 160},
        "stage_cost": {"begin": 160, "end_exclusive": 162},
    }
    if not _strict_equal(blocks, expected_blocks):
        raise ArtifactContractError("parameter block layout drifted")

    schemas = _require_object(
        payload["schemas"],
        {
            "discretization",
            "reference",
            "cost",
            "constraints",
            "runtime_fractional_delay_schedule",
        },
        "schemas",
    )
    if not _strict_equal(
        schemas,
        {
            "discretization": DISCRETIZATION_SCHEMA,
            "reference": REFERENCE_SCHEMA,
            "cost": COST_SCHEMA,
            "constraints": CONSTRAINT_SCHEMA,
            "runtime_fractional_delay_schedule": RUNTIME_SCHEDULE_SCHEMA_VERSION,
        },
    ):
        raise ArtifactContractError("model schema set drifted")

    runtime_schedule = _require_object(
        payload["runtime_schedule_contract"],
        {
            "schema_version",
            "time_grid",
            "channel_compilation",
            "channels",
            "merged_schedule",
            "tolerances",
            "actual_schedule_values",
        },
        "runtime_schedule_contract",
    )
    expected_schedule_structure = {
        "schema_version": RUNTIME_SCHEDULE_SCHEMA_VERSION,
        "time_grid": {
            "dt_sec": {"numerator": 1, "denominator": 30},
            "release_frequency_hz": 30,
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
                "release_intervals": 12,
                "delay_state_count": 11,
                "selector_width": 13,
                "L_max_sec": {"numerator": 2, "denominator": 5},
                "selector_parameter_prefix": "act_sel_v[slot][selector]",
            },
            "omega": {
                "release_intervals": 24,
                "delay_state_count": 23,
                "selector_width": 25,
                "L_max_sec": {"numerator": 4, "denominator": 5},
                "selector_parameter_prefix": "act_sel_omega[slot][selector]",
            },
        },
        "merged_schedule": {
            "slot_count": 3,
            "duration_parameter_prefix": "act_seg_dt[slot]",
            "duration_sum": "sum(act_seg_dt[:slot_count])==dt_sec",
            "switch_union": "sorted unique {0,v_switch,omega_switch,dt_sec}",
            "empty_slot_policy": "zero_duration_and_zero_selector",
        },
        "actual_schedule_values": "EXCLUDED_FROM_ARTIFACT_IDENTITY",
    }
    if any(
        not _strict_equal(runtime_schedule[key], expected)
        for key, expected in expected_schedule_structure.items()
    ):
        raise ArtifactContractError("runtime schedule values entered artifact identity")
    tolerances = _require_object(
        runtime_schedule["tolerances"],
        {"integer_snap_tolerance_sec", "duration_tolerance_sec"},
        "runtime schedule tolerances",
    )
    for key, upper in (
        ("integer_snap_tolerance_sec", 1.0 / 60.0),
        ("duration_tolerance_sec", 1.0 / 30.0),
    ):
        tolerance = tolerances[key]
        if (
            type(tolerance) is not float
            or not math.isfinite(tolerance)
            or tolerance < 0.0
            or tolerance >= upper
        ):
            raise ArtifactContractError(
                f"runtime_schedule_contract.tolerances.{key} is invalid"
            )

    runtime_parameters = _require_object(
        payload["runtime_parameter_contract"],
        {
            "assignment",
            "required_stage_range_inclusive",
            "row_width",
            "runtime_identity_exclusions",
        },
        "runtime_parameter_contract",
    )
    if not _strict_equal(
        runtime_parameters,
        {
            "assignment": RUNTIME_PARAMETER_ASSIGNMENT_POLICY,
            "required_stage_range_inclusive": [0, 60],
            "row_width": 162,
            "runtime_identity_exclusions": list(RUNTIME_IDENTITY_EXCLUSIONS),
        },
    ):
        raise ArtifactContractError("runtime parameter boundary drifted")
    comparison = _require_object(
        payload["comparison_contract"],
        {
            "arms",
            "shared_artifact_required",
            "runtime_only_fields_allowed_to_differ",
            "graph_ocp_solver_codegen_and_layout_must_match",
        },
        "comparison_contract",
    )
    if not _strict_equal(
        comparison,
        {
            "arms": list(COMPARISON_ARMS),
            "shared_artifact_required": True,
            "runtime_only_fields_allowed_to_differ": list(
                COMPARISON_RUNTIME_DIFFERENCES
            ),
            "graph_ocp_solver_codegen_and_layout_must_match": True,
        },
    ):
        raise ArtifactContractError("B0/Bslosh comparison boundary drifted")

    authorities = _require_object(
        payload["typed_authorities"],
        {
            "development_capacity",
            "development_layout",
            "solver_parameter_layout",
            "casadi_graph",
            "acados_ocp",
            "solver_options",
            "codegen_options",
            "provenance",
        },
        "typed_authorities",
    )
    capacity_authority = _require_object(
        authorities["development_capacity"],
        {"raw_bytes_sha256", "snapshot"},
        "development capacity authority",
    )
    layout_authority = _require_object(
        authorities["development_layout"],
        {"semantic_sha256", "snapshot"},
        "development layout authority",
    )
    parameter_authority = _require_object(
        authorities["solver_parameter_layout"],
        {"semantic_sha256", "snapshot"},
        "solver parameter layout authority",
    )
    try:
        require_sha256(capacity_authority["raw_bytes_sha256"], "capacity identity")
        require_sha256(layout_authority["semantic_sha256"], "layout identity")
        require_sha256(
            parameter_authority["semantic_sha256"], "parameter layout identity"
        )
    except IdentityError as exc:
        raise ArtifactContractError(str(exc)) from exc
    if capacity_authority["raw_bytes_sha256"] != DEVELOPMENT_CAPACITY_SHA256:
        raise ArtifactContractError("embedded capacity is not the pinned D4 snapshot")
    try:
        checked_capacity = development_capacity_from_dict(
            capacity_authority["snapshot"],
            capacity_authority["raw_bytes_sha256"],
        )
        checked_layout = development_layout_from_dict(
            layout_authority["snapshot"],
            checked_capacity,
        )
        checked_parameter_layout = solver_parameter_layout_from_dict(
            parameter_authority["snapshot"],
            checked_capacity,
            checked_layout,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ArtifactContractError(
            "embedded capacity/layout authority is malformed"
        ) from exc
    if sha256_json(checked_layout.to_dict()) != layout_authority["semantic_sha256"]:
        raise ArtifactContractError("embedded development layout hash is inconsistent")
    if (
        sha256_json(checked_parameter_layout.to_dict())
        != parameter_authority["semantic_sha256"]
    ):
        raise ArtifactContractError("embedded parameter layout hash is inconsistent")
    if (
        checked_layout.to_dict()["state_layout"]["ordered"] != list(state_order)
        or checked_layout.to_dict()["control_layout"]["ordered"] != list(control_order)
        or checked_parameter_layout.to_dict()["parameter_layout"]["ordered"]
        != list(parameter_order)
    ):
        raise ArtifactContractError("direct layouts differ from typed authorities")

    try:
        ocp = validate_acados_ocp_document(
            authorities["acados_ocp"],
            require_matched_source_root=True,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ArtifactContractError("embedded Acados OCP is malformed") from exc
    try:
        solver = validate_solver_options_document(authorities["solver_options"])
        options = validate_codegen_options_document(
            authorities["codegen_options"],
            expected_development_layout_sha256=layout_authority["semantic_sha256"],
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ArtifactContractError(
            "embedded solver/codegen options are malformed"
        ) from exc
    try:
        provenance = validate_codegen_provenance_document(authorities["provenance"])
    except (KeyError, ProvenanceError, TypeError, ValueError) as exc:
        raise ArtifactContractError("embedded provenance is malformed") from exc
    try:
        graph = validate_casadi_graph_document(
            authorities["casadi_graph"],
            expected_capacity_contract_raw_bytes_sha256=capacity_authority[
                "raw_bytes_sha256"
            ],
            expected_development_layout_semantic_sha256=layout_authority[
                "semantic_sha256"
            ],
            expected_solver_parameter_layout_semantic_sha256=parameter_authority[
                "semantic_sha256"
            ],
            expected_state_order=state_order,
            expected_control_order=control_order,
            expected_parameter_order=parameter_order,
            expected_casadi_version=ocp["backend"]["casadi_version"],
            expected_bounds_snapshot_sha256=ocp["source_identity"][
                "bounds_snapshot_sha256"
            ],
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ArtifactContractError("embedded CasADi graph is malformed") from exc
    _validate_graph_ocp_binding(graph, ocp)
    if not _strict_equal(
        ocp["dimensions"],
        {
            "N": dimensions["N"],
            "NX": dimensions["NX"],
            "NU": dimensions["NU"],
            "NP": dimensions["NP"],
            "parameter_vector_count": dimensions["parameter_vector_count"],
        },
    ):
        raise ArtifactContractError("embedded OCP dimensions differ from contract")
    ocp_source = ocp["source_identity"]
    if (
        ocp_source["capacity_contract_raw_bytes_sha256"]
        != capacity_authority["raw_bytes_sha256"]
        or ocp_source["development_layout_semantic_sha256"]
        != layout_authority["semantic_sha256"]
        or ocp_source["solver_parameter_layout_semantic_sha256"]
        != parameter_authority["semantic_sha256"]
        or ocp_source["graph_semantic_sha256"]
        != graph["graph_semantic_identity"]["sha256"]
        or ocp_source["solver_options_semantic_sha256"]
        != solver["semantic_identity"]["sha256"]
        or options["source_identity"]["development_layout_semantic_sha256"]
        != layout_authority["semantic_sha256"]
    ):
        raise ArtifactContractError("embedded authority hash bindings drifted")
    if options["runtime_identity_exclusions"] != list(RUNTIME_IDENTITY_EXCLUSIONS):
        raise ArtifactContractError("embedded codegen runtime exclusions drifted")
    codegen_schedule = options["runtime_schedule_contract"]
    if not _strict_equal(
        runtime_schedule["tolerances"],
        {
            "integer_snap_tolerance_sec": codegen_schedule[
                "integer_snap_tolerance_sec"
            ],
            "duration_tolerance_sec": codegen_schedule["duration_tolerance_sec"],
        },
    ):
        raise ArtifactContractError(
            "runtime schedule tolerances differ from codegen options"
        )
    _validate_provenance_bindings(provenance, ocp, options)
    try:
        validate_acados_codegen_result_document(payload["artifact"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ArtifactContractError(
            "embedded Acados codegen result is malformed"
        ) from exc
    return payload


def _validate_serialized_contract(
    payload_json: Any,
    semantic_sha256: Any,
    artifact_sha256: Any,
) -> dict[str, Any]:
    """Validate canonical payload bytes and both contract identities."""

    if type(payload_json) is not str:
        raise ArtifactContractError(
            "model contract payload must be canonical JSON text"
        )
    try:
        payload = json.loads(payload_json)
        if canonical_json(payload) != payload_json:
            raise ArtifactContractError("model contract payload is not canonical JSON")
        checked = _validate_payload(payload)
        require_sha256(semantic_sha256, "model contract semantic identity")
        require_sha256(artifact_sha256, "artifact identity")
        if sha256_json(checked) != semantic_sha256:
            raise ArtifactContractError(
                "model contract semantic identity is inconsistent"
            )
        if _artifact_sha256(semantic_sha256, checked) != artifact_sha256:
            raise ArtifactContractError("artifact identity is inconsistent")
        return checked
    except ArtifactContractError:
        raise
    except (IdentityError, KeyError, TypeError, ValueError) as exc:
        raise ArtifactContractError("model contract payload is malformed") from exc


__all__ = [
    "ARTIFACT_CLASS",
    "ARTIFACT_CONTRACT_SCHEMA",
    "ARTIFACT_CONTRACT_SCOPE",
    "ARTIFACT_IDENTITY_SCHEMA",
    "ARTIFACT_IDENTITY_SCOPE",
    "ARTIFACT_STATUS",
    "COMPARISON_ARMS",
    "COMPARISON_RUNTIME_DIFFERENCES",
    "CONTRACT_OUTPUT_POLICY",
    "GENERATED_HEADER_BINDING_POLICY",
    "MODEL_ID",
    "NON_CIRCULAR_IDENTITY_POLICY",
    "PROMOTION_STATUS",
    "RUNTIME_PARAMETER_ASSIGNMENT_POLICY",
    "TARGET_PERFORMANCE_STATUS",
    "ArtifactContractError",
    "_artifact_sha256",
    "_require_object",
    "_require_order_and_offsets",
    "_validate_payload",
    "_validate_serialized_contract",
]
