"""Dependency-free validation for a serialized Stage 3-D CasADi graph.

The symbolic builder owns expression construction and
``casadi_graph_contract`` owns the structure-only graph hash.  This module
owns the other half of that boundary: strict validation of the stable,
JSON-serializable graph metadata.  It deliberately does not import CasADi,
Acados, or an OCP document.  Bindings to an OCP, generated artifact, or
backend remain the responsibility of their owning contract.
"""

from __future__ import annotations

import math
from typing import Any

from .casadi_graph_contract import (
    ARTIFACT_STATUS,
    BOUND_SNAPSHOT_SCOPE,
    CASADI_GRAPH_SCHEMA,
    COMPARISON_ARMS,
    COMPARISON_IDENTICAL_FIELDS,
    COMPARISON_PARAMETER_FIELDS,
    DIAGNOSTIC_RESIDUAL_ROLE,
    EXECUTION_SCHEDULE_POLICY,
    GRAPH_IDENTITY_SCOPE,
    GRAPH_STATUS,
    LIQUID_HARD_CONSTRAINT_POLICY,
    REFERENCE_DOMAIN_LOWER,
    REFERENCE_DOMAIN_UPPER,
    RUNTIME_PARAMETER_INPUT_POLICY,
    RUNTIME_PARAMETER_VALUES_POLICY,
    STAGE_REFERENCE_DOMAIN_ORDER,
    TERMINAL_CONTROL_POLICY,
    TERMINAL_INPUT_ORDER,
    TERMINAL_LIQUID_COST_POLICY,
    TERMINAL_REFERENCE_DOMAIN_ORDER,
    graph_semantic_sha256,
)
from .constraints_oracle import (
    CONSTRAINT_RESIDUAL_ORDER,
    CONSTRAINT_SCHEMA,
    CONSTRAINT_VALUE_STATUS,
    CONTROL_BOX_ORDER,
    STAGE_NONLINEAR_H_ORDER,
)
from .development_capacity import (
    EXECUTION_SUBSEGMENT_SLOTS,
    NU,
    NX,
    RELEASE_FREQUENCY_HZ,
    RELEASE_PERIOD_SEC,
)
from .development_layout import HORIZON_STEPS, STAGE_SEMANTICS
from .identity import IdentityError, require_sha256, sha256_json
from .model_contract import COST_SCHEMA, DISCRETIZATION_SCHEMA, MODEL_ID
from .solver_parameter_layout import FULL_STAGE_PARAMETER_COUNT, REFERENCE_SCHEMA

GRAPH_N = HORIZON_STEPS
GRAPH_NX = NX
GRAPH_NU = NU
GRAPH_NP = FULL_STAGE_PARAMETER_COUNT
GRAPH_PARAMETER_VECTOR_COUNT = GRAPH_N + 1
GRAPH_RELEASE_FREQUENCY_HZ = RELEASE_FREQUENCY_HZ
GRAPH_RELEASE_PERIOD = {
    "numerator": RELEASE_PERIOD_SEC.numerator,
    "denominator": RELEASE_PERIOD_SEC.denominator,
}
GRAPH_EXECUTION_SLOT_COUNT = EXECUTION_SUBSEGMENT_SLOTS


class CasadiGraphSchemaError(ValueError):
    """A serialized CasADi graph is malformed or policy-inconsistent."""


def _object(value: Any, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != keys:
        raise CasadiGraphSchemaError(f"{label} keys do not match the v1 schema")
    return value


def _text(value: Any, label: str) -> str:
    if type(value) is not str or not value or "\x00" in value:
        raise CasadiGraphSchemaError(f"{label} must be a non-empty string")
    return value


def _digest(value: Any, label: str) -> str:
    try:
        return require_sha256(value, label)
    except IdentityError as exc:
        raise CasadiGraphSchemaError(str(exc)) from exc


def _finite_float(value: Any, label: str, *, positive: bool = False) -> float:
    if type(value) is not float or not math.isfinite(value):
        raise CasadiGraphSchemaError(f"{label} must be an explicit finite float")
    if positive and value <= 0.0:
        raise CasadiGraphSchemaError(f"{label} must be strictly positive")
    return value


def _float_vector(value: Any, *, length: int, label: str) -> list[float]:
    if type(value) is not list or len(value) != length:
        raise CasadiGraphSchemaError(f"{label} has invalid dimensions")
    return [
        _finite_float(item, f"{label}[{index}]") for index, item in enumerate(value)
    ]


def _text_order(value: Any, *, length: int, label: str) -> list[str]:
    if type(value) is not list or len(value) != length:
        raise CasadiGraphSchemaError(f"{label} has invalid dimensions")
    order = [_text(item, f"{label}[{index}]") for index, item in enumerate(value)]
    if len(set(order)) != length:
        raise CasadiGraphSchemaError(f"{label} contains duplicate names")
    return order


def _integer(value: Any, label: str, *, positive: bool = False) -> int:
    if type(value) is not int or (positive and value <= 0):
        requirement = "positive integer" if positive else "integer"
        raise CasadiGraphSchemaError(f"{label} must be an {requirement}")
    return value


def _check_expected_digest(
    actual: str,
    expected: str | None,
    *,
    label: str,
) -> None:
    if expected is None:
        return
    _digest(expected, f"expected {label}")
    if actual != expected:
        raise CasadiGraphSchemaError(
            f"graph {label} differs from the expected authority"
        )


def _check_expected_order(
    actual: list[str],
    expected: list[str] | tuple[str, ...] | None,
    *,
    label: str,
) -> None:
    if expected is None:
        return
    if type(expected) not in {list, tuple} or any(
        type(item) is not str or not item for item in expected
    ):
        raise CasadiGraphSchemaError(f"expected {label} is not a string order")
    expected_tuple = tuple(expected)
    if actual != list(expected_tuple):
        raise CasadiGraphSchemaError(
            f"graph {label} differs from the expected authority"
        )


def _check_expected_text(actual: str, expected: str | None, *, label: str) -> None:
    if expected is None:
        return
    _text(expected, f"expected {label}")
    if actual != expected:
        raise CasadiGraphSchemaError(
            f"graph {label} differs from the expected authority"
        )


def _validate_reference_domain(
    value: Any,
    *,
    order: tuple[str, ...],
    label: str,
) -> None:
    domain = _object(value, {"order", "lower", "upper"}, label)
    actual_order = _text_order(
        domain["order"], length=len(order), label=f"{label} order"
    )
    lower = _float_vector(domain["lower"], length=len(order), label=f"{label} lower")
    upper = _float_vector(domain["upper"], length=len(order), label=f"{label} upper")
    if actual_order != list(order):
        raise CasadiGraphSchemaError(f"{label} order drifted")
    if lower != [REFERENCE_DOMAIN_LOWER] * len(order) or upper != [
        REFERENCE_DOMAIN_UPPER
    ] * len(order):
        raise CasadiGraphSchemaError(f"{label} bounds drifted")


def _validate_bounds_snapshot(
    value: Any,
    *,
    expected_sha256: str | None,
) -> dict[str, Any]:
    snapshot = _object(
        value,
        {"constraint_schema", "value_status", "values"},
        "graph bounds snapshot",
    )
    if snapshot["constraint_schema"] != CONSTRAINT_SCHEMA:
        raise CasadiGraphSchemaError("graph bounds snapshot schema drifted")
    if snapshot["value_status"] != CONSTRAINT_VALUE_STATUS:
        raise CasadiGraphSchemaError("graph bounds snapshot value status drifted")
    values = _object(
        snapshot["values"],
        {
            "q_issue_v_max",
            "q_issue_omega_max",
            "a_issue_max",
            "alpha_issue_max",
            "jerk_v_max",
            "jerk_omega_max",
            "v_s_max",
        },
        "graph bounds snapshot values",
    )
    for name, item in values.items():
        _finite_float(item, f"graph bounds snapshot values.{name}", positive=True)
    try:
        actual_sha256 = sha256_json(snapshot)
    except IdentityError as exc:
        raise CasadiGraphSchemaError(
            "graph bounds snapshot is not canonicalizable"
        ) from exc
    _check_expected_digest(
        actual_sha256,
        expected_sha256,
        label="bounds snapshot identity",
    )
    return snapshot


def _validate_graph_constraints(
    value: Any,
    *,
    control_order: list[str],
    bounds_snapshot: dict[str, Any],
) -> None:
    constraints = _object(
        value,
        {
            "value_status",
            "stage_nonlinear_h",
            "stage_reference_domain",
            "terminal_reference_domain",
            "control_box",
            "diagnostic_residuals",
            "bounds_snapshot_scope",
            "bounds_snapshot",
            "bounds_snapshot_sha256",
        },
        "graph constraints",
    )
    if constraints["value_status"] != CONSTRAINT_VALUE_STATUS:
        raise CasadiGraphSchemaError("graph constraint value status drifted")

    stage = _object(
        constraints["stage_nonlinear_h"],
        {"order", "lower", "upper"},
        "graph nonlinear constraints",
    )
    stage_order = _text_order(
        stage["order"],
        length=len(STAGE_NONLINEAR_H_ORDER),
        label="graph nonlinear order",
    )
    if stage_order != list(STAGE_NONLINEAR_H_ORDER):
        raise CasadiGraphSchemaError("graph nonlinear order drifted")
    stage_lower = _float_vector(
        stage["lower"],
        length=len(STAGE_NONLINEAR_H_ORDER),
        label="graph nonlinear lower",
    )
    stage_upper = _float_vector(
        stage["upper"],
        length=len(STAGE_NONLINEAR_H_ORDER),
        label="graph nonlinear upper",
    )

    _validate_reference_domain(
        constraints["stage_reference_domain"],
        order=STAGE_REFERENCE_DOMAIN_ORDER,
        label="graph stage reference domain",
    )
    _validate_reference_domain(
        constraints["terminal_reference_domain"],
        order=TERMINAL_REFERENCE_DOMAIN_ORDER,
        label="graph terminal reference domain",
    )

    control = _object(
        constraints["control_box"],
        {"indices", "order", "lower", "upper"},
        "graph control box",
    )
    control_order_document = _text_order(
        control["order"], length=len(CONTROL_BOX_ORDER), label="graph control order"
    )
    if control_order_document != list(CONTROL_BOX_ORDER):
        raise CasadiGraphSchemaError("graph control order drifted")
    indices = control["indices"]
    if (
        type(indices) is not list
        or len(indices) != len(CONTROL_BOX_ORDER)
        or any(
            type(index) is not int or index < 0 or index >= len(control_order)
            for index in indices
        )
        or len(set(indices)) != len(indices)
    ):
        raise CasadiGraphSchemaError("graph control indices are invalid")
    expected_indices = [control_order.index(name) for name in CONTROL_BOX_ORDER]
    if indices != expected_indices:
        raise CasadiGraphSchemaError("graph control indices drifted")
    control_lower = _float_vector(
        control["lower"], length=len(CONTROL_BOX_ORDER), label="graph control lower"
    )
    control_upper = _float_vector(
        control["upper"], length=len(CONTROL_BOX_ORDER), label="graph control upper"
    )

    diagnostic = _object(
        constraints["diagnostic_residuals"],
        {"role", "order", "feasible_upper"},
        "graph diagnostic residuals",
    )
    if diagnostic["role"] != DIAGNOSTIC_RESIDUAL_ROLE:
        raise CasadiGraphSchemaError("graph diagnostic residual role drifted")
    diagnostic_order = _text_order(
        diagnostic["order"],
        length=len(CONSTRAINT_RESIDUAL_ORDER),
        label="graph diagnostic residual order",
    )
    if diagnostic_order != list(CONSTRAINT_RESIDUAL_ORDER):
        raise CasadiGraphSchemaError("graph diagnostic residual order drifted")
    feasible_upper = _float_vector(
        diagnostic["feasible_upper"],
        length=len(CONSTRAINT_RESIDUAL_ORDER),
        label="graph diagnostic feasible upper",
    )
    if feasible_upper != [0.0] * len(CONSTRAINT_RESIDUAL_ORDER):
        raise CasadiGraphSchemaError("graph diagnostic feasible upper drifted")

    if constraints["bounds_snapshot_scope"] != BOUND_SNAPSHOT_SCOPE:
        raise CasadiGraphSchemaError("graph bounds snapshot scope drifted")
    if constraints["bounds_snapshot"] != bounds_snapshot:
        raise CasadiGraphSchemaError(
            "graph bounds snapshot was not validated consistently"
        )
    _digest(constraints["bounds_snapshot_sha256"], "graph bounds snapshot identity")
    try:
        if sha256_json(bounds_snapshot) != constraints["bounds_snapshot_sha256"]:
            raise CasadiGraphSchemaError(
                "graph bounds snapshot identity is inconsistent"
            )
    except IdentityError as exc:
        raise CasadiGraphSchemaError(
            "graph bounds snapshot is not canonicalizable"
        ) from exc

    values = bounds_snapshot["values"]
    expected_stage_upper = [
        values["q_issue_v_max"],
        values["q_issue_omega_max"],
        values["a_issue_max"],
        values["alpha_issue_max"],
    ]
    if stage_upper != expected_stage_upper or stage_lower != [
        -item for item in expected_stage_upper
    ]:
        raise CasadiGraphSchemaError("graph nonlinear bounds differ from the snapshot")
    expected_control_lower = [
        -values["jerk_v_max"],
        -values["jerk_omega_max"],
        0.0,
    ]
    expected_control_upper = [
        values["jerk_v_max"],
        values["jerk_omega_max"],
        values["v_s_max"],
    ]
    if (
        control_lower != expected_control_lower
        or control_upper != expected_control_upper
    ):
        raise CasadiGraphSchemaError("graph control bounds differ from the snapshot")


def validate_casadi_graph_document(
    value: Any,
    *,
    expected_capacity_contract_raw_bytes_sha256: str | None = None,
    expected_development_layout_semantic_sha256: str | None = None,
    expected_solver_parameter_layout_semantic_sha256: str | None = None,
    expected_state_order: list[str] | tuple[str, ...] | None = None,
    expected_control_order: list[str] | tuple[str, ...] | None = None,
    expected_parameter_order: list[str] | tuple[str, ...] | None = None,
    expected_casadi_version: str | None = None,
    expected_bounds_snapshot_sha256: str | None = None,
) -> dict[str, Any]:
    """Validate one serialized graph and recompute its structure-only hash.

    The optional ``expected_*`` values are cross-bind checks for callers that
    already hold typed authorities.  They do not make this validator depend
    on those authorities.  In particular, CasADi version and bounds are
    validated as graph metadata but are intentionally not included in the
    graph semantic hash; callers may bind them to an OCP or another contract
    by passing the corresponding expectation here.
    """

    graph = _object(
        value,
        {
            "schema_version",
            "status",
            "model_id",
            "casadi_version",
            "graph_identity_scope",
            "runtime_parameter_values",
            "runtime_parameter_input_policy",
            "graph_semantic_identity",
            "source_identity",
            "dimensions",
            "horizon",
            "stage_semantics",
            "orders",
            "schemas",
            "execution",
            "constraints",
            "terminal",
            "comparison_identity",
        },
        "CasADi graph document",
    )
    if graph["schema_version"] != CASADI_GRAPH_SCHEMA:
        raise CasadiGraphSchemaError("graph schema drifted")
    if graph["status"] != {"graph": GRAPH_STATUS, "artifact": ARTIFACT_STATUS}:
        raise CasadiGraphSchemaError("graph status drifted")
    if graph["model_id"] != MODEL_ID:
        raise CasadiGraphSchemaError("graph model identity drifted")
    casadi_version = _text(graph["casadi_version"], "graph CasADi version")
    _check_expected_text(
        casadi_version, expected_casadi_version, label="CasADi version"
    )
    if graph["graph_identity_scope"] != GRAPH_IDENTITY_SCOPE:
        raise CasadiGraphSchemaError("graph semantic scope drifted")
    if graph["runtime_parameter_values"] != RUNTIME_PARAMETER_VALUES_POLICY:
        raise CasadiGraphSchemaError("graph runtime values policy drifted")
    if graph["runtime_parameter_input_policy"] != RUNTIME_PARAMETER_INPUT_POLICY:
        raise CasadiGraphSchemaError("graph parameter input policy drifted")

    identity = _object(
        graph["graph_semantic_identity"],
        {"sha256", "scope"},
        "graph semantic identity",
    )
    if identity["scope"] != GRAPH_IDENTITY_SCOPE:
        raise CasadiGraphSchemaError("graph semantic identity scope drifted")
    _digest(identity["sha256"], "graph semantic identity")

    source = _object(
        graph["source_identity"],
        {
            "capacity_contract_raw_bytes_sha256",
            "development_layout_semantic_sha256",
            "solver_parameter_layout_semantic_sha256",
        },
        "graph source identity",
    )
    capacity_sha256 = _digest(
        source["capacity_contract_raw_bytes_sha256"],
        "graph capacity source identity",
    )
    layout_sha256 = _digest(
        source["development_layout_semantic_sha256"],
        "graph development-layout source identity",
    )
    parameter_sha256 = _digest(
        source["solver_parameter_layout_semantic_sha256"],
        "graph parameter-layout source identity",
    )
    _check_expected_digest(
        capacity_sha256,
        expected_capacity_contract_raw_bytes_sha256,
        label="capacity source identity",
    )
    _check_expected_digest(
        layout_sha256,
        expected_development_layout_semantic_sha256,
        label="development-layout source identity",
    )
    _check_expected_digest(
        parameter_sha256,
        expected_solver_parameter_layout_semantic_sha256,
        label="parameter-layout source identity",
    )

    dimensions = _object(
        graph["dimensions"],
        {"N", "NX", "NU", "NP"},
        "graph dimensions",
    )
    for name in ("N", "NX", "NU", "NP"):
        _integer(dimensions[name], f"graph dimensions.{name}", positive=True)
    if dimensions != {"N": GRAPH_N, "NX": GRAPH_NX, "NU": GRAPH_NU, "NP": GRAPH_NP}:
        raise CasadiGraphSchemaError("graph dimensions drifted from the D4 model")

    horizon = _object(
        graph["horizon"],
        {"N", "parameter_vector_count", "release_frequency_hz", "release_period_sec"},
        "graph horizon",
    )
    for name in ("N", "parameter_vector_count", "release_frequency_hz"):
        _integer(horizon[name], f"graph horizon.{name}", positive=True)
    period = _object(
        horizon["release_period_sec"],
        {"numerator", "denominator"},
        "graph release period",
    )
    _integer(period["numerator"], "graph release period numerator", positive=True)
    _integer(period["denominator"], "graph release period denominator", positive=True)
    if horizon != {
        "N": GRAPH_N,
        "parameter_vector_count": GRAPH_PARAMETER_VECTOR_COUNT,
        "release_frequency_hz": GRAPH_RELEASE_FREQUENCY_HZ,
        "release_period_sec": GRAPH_RELEASE_PERIOD,
    }:
        raise CasadiGraphSchemaError("graph horizon drifted")
    if graph["stage_semantics"] != STAGE_SEMANTICS:
        raise CasadiGraphSchemaError("graph stage semantics drifted")

    orders = _object(
        graph["orders"],
        {"state", "control", "parameter"},
        "graph orders",
    )
    state_order = _text_order(
        orders["state"], length=GRAPH_NX, label="graph state order"
    )
    control_order = _text_order(
        orders["control"], length=GRAPH_NU, label="graph control order"
    )
    parameter_order = _text_order(
        orders["parameter"], length=GRAPH_NP, label="graph parameter order"
    )
    _check_expected_order(state_order, expected_state_order, label="state order")
    _check_expected_order(control_order, expected_control_order, label="control order")
    _check_expected_order(
        parameter_order, expected_parameter_order, label="parameter order"
    )

    if graph["schemas"] != {
        "discretization": DISCRETIZATION_SCHEMA,
        "reference": REFERENCE_SCHEMA,
        "cost": COST_SCHEMA,
        "constraints": CONSTRAINT_SCHEMA,
    }:
        raise CasadiGraphSchemaError("graph schema references drifted")
    execution = _object(
        graph["execution"],
        {"slot_count", "schedule_policy"},
        "graph execution",
    )
    _integer(execution["slot_count"], "graph execution slot count", positive=True)
    _text(execution["schedule_policy"], "graph execution schedule policy")
    if execution != {
        "slot_count": GRAPH_EXECUTION_SLOT_COUNT,
        "schedule_policy": EXECUTION_SCHEDULE_POLICY,
    }:
        raise CasadiGraphSchemaError("graph execution policy drifted")

    bounds_snapshot = _validate_bounds_snapshot(
        graph["constraints"]["bounds_snapshot"]
        if type(graph["constraints"]) is dict
        and "bounds_snapshot" in graph["constraints"]
        else None,
        expected_sha256=expected_bounds_snapshot_sha256,
    )
    _validate_graph_constraints(
        graph["constraints"],
        control_order=control_order,
        bounds_snapshot=bounds_snapshot,
    )

    terminal = _object(
        graph["terminal"],
        {"input_order", "control_policy", "liquid_cost_policy"},
        "graph terminal policy",
    )
    _text_order(terminal["input_order"], length=2, label="graph terminal input order")
    _text(terminal["control_policy"], "graph terminal control policy")
    _text(terminal["liquid_cost_policy"], "graph terminal liquid-cost policy")
    if terminal != {
        "input_order": list(TERMINAL_INPUT_ORDER),
        "control_policy": TERMINAL_CONTROL_POLICY,
        "liquid_cost_policy": TERMINAL_LIQUID_COST_POLICY,
    }:
        raise CasadiGraphSchemaError("graph terminal policy drifted")
    comparison = _object(
        graph["comparison_identity"],
        {
            "arms",
            "same_symbolic_graph",
            "only_parameter_fields_allowed_to_differ",
            "must_be_identical",
            "liquid_hard_constraints",
        },
        "graph comparison identity",
    )
    _text_order(comparison["arms"], length=2, label="graph comparison arms")
    if type(comparison["same_symbolic_graph"]) is not bool:
        raise CasadiGraphSchemaError("graph comparison identity flag is not boolean")
    _text_order(
        comparison["only_parameter_fields_allowed_to_differ"],
        length=2,
        label="graph comparison differing fields",
    )
    _text_order(
        comparison["must_be_identical"],
        length=5,
        label="graph comparison identical fields",
    )
    _text(comparison["liquid_hard_constraints"], "graph liquid hard-constraint policy")
    if comparison != {
        "arms": list(COMPARISON_ARMS),
        "same_symbolic_graph": True,
        "only_parameter_fields_allowed_to_differ": list(COMPARISON_PARAMETER_FIELDS),
        "must_be_identical": list(COMPARISON_IDENTICAL_FIELDS),
        "liquid_hard_constraints": LIQUID_HARD_CONSTRAINT_POLICY,
    }:
        raise CasadiGraphSchemaError("graph comparison identity drifted")

    try:
        expected_graph_sha256 = graph_semantic_sha256(
            capacity_contract_sha256=capacity_sha256,
            development_layout_sha256=layout_sha256,
            solver_parameter_layout_sha256=parameter_sha256,
            horizon_steps=dimensions["N"],
            parameter_vector_count=horizon["parameter_vector_count"],
            nx=dimensions["NX"],
            nu=dimensions["NU"],
            np=dimensions["NP"],
            state_order=tuple(state_order),
            control_order=tuple(control_order),
            parameter_order=tuple(parameter_order),
            control_indices=tuple(graph["constraints"]["control_box"]["indices"]),
        )
    except (IdentityError, TypeError, ValueError) as exc:
        raise CasadiGraphSchemaError(
            "graph semantic identity cannot be recomputed"
        ) from exc
    if identity["sha256"] != expected_graph_sha256:
        raise CasadiGraphSchemaError("graph semantic identity is inconsistent")
    return graph


__all__ = [
    "CasadiGraphSchemaError",
    "validate_casadi_graph_document",
]
