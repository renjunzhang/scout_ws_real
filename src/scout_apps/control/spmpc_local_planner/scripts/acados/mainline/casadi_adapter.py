"""Lazy entry point for the Stage 3-D actuator-aware CasADi graph.

The imported modules contain only Python structure builders. CasADi itself is
loaded inside :func:`build_casadi_graph`; acados OCP assembly, code generation,
and artifact publication are deliberately outside this stage.
"""

from __future__ import annotations

from typing import Any

from .casadi_dynamics import (
    SymbolicDynamicsConstructionError,
    build_symbolic_discrete_map,
    build_symbolic_issue,
)
from .casadi_graph_contract import (
    _GRAPH_BUNDLE_TOKEN,
    ARTIFACT_STATUS,
    CASADI_GRAPH_SCHEMA,
    DIAGNOSTIC_RESIDUAL_ROLE,
    GRAPH_IDENTITY_SCOPE,
    GRAPH_STATUS,
    REFERENCE_DOMAIN_LOWER,
    REFERENCE_DOMAIN_UPPER,
    RUNTIME_PARAMETER_INPUT_POLICY,
    STAGE_REFERENCE_DOMAIN_ORDER,
    STAGE_SEMANTICS,
    TERMINAL_CONTROL_POLICY,
    TERMINAL_REFERENCE_DOMAIN_ORDER,
    CasadiGraphBundle,
    SymbolicExecutionSlot,
    SymbolicIssuedValues,
    SymbolicReferenceValues,
    SymbolicStageConstraints,
    SymbolicStageCosts,
    SymbolicTerminalExpressions,
    graph_semantic_sha256,
    require_casadi_graph_bundle,
)
from .casadi_objective import (
    SymbolicObjectiveConstructionError,
    build_symbolic_stage_constraints,
    build_symbolic_stage_costs,
    build_symbolic_terminal,
)
from .casadi_reference import build_symbolic_reference
from .constraints_oracle import (
    CONTROL_BOX_ORDER,
    ConstraintBounds,
    ConstraintOracleError,
    require_constraint_bounds,
)
from .development_capacity import DevelopmentCapacityContract
from .development_layout import DevelopmentLayout
from .identity import IdentityError, sha256_json
from .solver_parameter_layout import (
    SolverParameterLayout,
    SolverParameterLayoutError,
    build_solver_parameter_layout,
)


class CasadiAdapterError(RuntimeError):
    """Base class for graph-adapter failures."""


class CasadiDependencyError(CasadiAdapterError):
    """CasADi is unavailable or cannot be loaded in the active interpreter."""


class CasadiGraphConstructionError(CasadiAdapterError):
    """Typed inputs or symbolic expression construction are invalid."""


def _require_casadi() -> Any:
    try:
        import casadi as ca
    except (ImportError, OSError) as exc:
        raise CasadiDependencyError(
            "CasADi is required for Stage 3-D symbolic graph construction"
        ) from exc
    return ca


def _canonical_parameter_layout(
    capacity: Any,
    development_layout: Any,
) -> SolverParameterLayout:
    if type(capacity) is not DevelopmentCapacityContract:
        raise CasadiGraphConstructionError(
            "capacity must be the exact DevelopmentCapacityContract type"
        )
    if type(development_layout) is not DevelopmentLayout:
        raise CasadiGraphConstructionError(
            "development_layout must be the exact DevelopmentLayout type"
        )
    try:
        return build_solver_parameter_layout(capacity, development_layout)
    except SolverParameterLayoutError as exc:
        raise CasadiGraphConstructionError(
            "capacity/development layout sources are not canonical"
        ) from exc


def _typed_source_hashes(
    layout: DevelopmentLayout,
    parameter_layout: SolverParameterLayout,
) -> tuple[str, str]:
    try:
        return sha256_json(layout.to_dict()), sha256_json(parameter_layout.to_dict())
    except IdentityError as exc:
        raise CasadiGraphConstructionError(
            "typed layout identity cannot be encoded"
        ) from exc


def build_casadi_graph(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    constraint_bounds: ConstraintBounds,
) -> CasadiGraphBundle:
    """Build the complete symbolic graph without OCP or file generation."""

    parameter_layout = _canonical_parameter_layout(capacity, development_layout)
    try:
        checked_bounds = require_constraint_bounds(constraint_bounds)
    except ConstraintOracleError as exc:
        raise CasadiGraphConstructionError(
            "constraint bounds are not canonical"
        ) from exc
    layout_sha256, parameter_sha256 = _typed_source_hashes(
        development_layout,
        parameter_layout,
    )
    control_indices = tuple(
        development_layout.control_offsets[name] for name in CONTROL_BOX_ORDER
    )
    graph_semantic_identity = graph_semantic_sha256(
        capacity_contract_sha256=capacity.contract_sha256,
        development_layout_sha256=layout_sha256,
        solver_parameter_layout_sha256=parameter_sha256,
        horizon_steps=development_layout.horizon_steps,
        parameter_vector_count=development_layout.horizon_steps + 1,
        nx=development_layout.NX,
        nu=development_layout.NU,
        np=parameter_layout.NP,
        state_order=development_layout.state_names,
        control_order=development_layout.control_names,
        parameter_order=parameter_layout.parameter_names,
        control_indices=control_indices,
    )
    ca = _require_casadi()
    try:
        x = ca.SX.sym("x", development_layout.NX)
        u = ca.SX.sym("u", development_layout.NU)
        p = ca.SX.sym("p", parameter_layout.NP)
        issued = build_symbolic_issue(x, u, development_layout)
        x_next, execution_slots = build_symbolic_discrete_map(
            ca,
            x,
            u,
            p,
            development_layout,
            parameter_layout,
            issued,
        )
        reference = build_symbolic_reference(
            ca,
            x,
            p,
            development_layout,
            parameter_layout,
        )
        terminal_reference = build_symbolic_reference(
            ca,
            x,
            p,
            development_layout,
            parameter_layout,
        )
        stage_costs = build_symbolic_stage_costs(
            x,
            u,
            p,
            x_next,
            issued,
            reference,
            development_layout,
            parameter_layout,
        )
        stage_constraints = build_symbolic_stage_constraints(
            ca,
            issued,
            u,
            reference,
            checked_bounds,
            development_layout,
        )
        terminal = build_symbolic_terminal(
            ca,
            x,
            p,
            terminal_reference,
            development_layout,
            parameter_layout,
        )
        if stage_constraints.control_indices != control_indices:
            raise CasadiGraphConstructionError(
                "symbolic control indices drifted from the typed layout"
            )
        bundle = CasadiGraphBundle(
            x=x,
            u=u,
            p=p,
            x_next=x_next,
            issued=issued,
            reference=reference,
            execution_slots=execution_slots,
            stage_costs=stage_costs,
            stage_constraints=stage_constraints,
            terminal=terminal,
            state_order=development_layout.state_names,
            control_order=development_layout.control_names,
            parameter_order=parameter_layout.parameter_names,
            horizon_steps=development_layout.horizon_steps,
            nx=development_layout.NX,
            nu=development_layout.NU,
            np=parameter_layout.NP,
            capacity_contract_sha256=capacity.contract_sha256,
            development_layout_sha256=layout_sha256,
            solver_parameter_layout_sha256=parameter_sha256,
            bounds=checked_bounds,
            casadi_version=str(getattr(ca, "__version__", "unknown")),
            graph_semantic_sha256=graph_semantic_identity,
            parameter_vector_count=development_layout.horizon_steps + 1,
            release_frequency_hz=development_layout.release_frequency_hz,
            release_period_sec=development_layout.release_period_sec,
            stage_semantics=STAGE_SEMANTICS,
            _construction_token=_GRAPH_BUNDLE_TOKEN,
        )
        return require_casadi_graph_bundle(
            bundle,
            expected_capacity_contract_raw_bytes_sha256=capacity.contract_sha256,
            expected_development_layout_semantic_sha256=layout_sha256,
            expected_solver_parameter_layout_semantic_sha256=parameter_sha256,
            expected_state_order=development_layout.state_names,
            expected_control_order=development_layout.control_names,
            expected_parameter_order=parameter_layout.parameter_names,
        )
    except CasadiAdapterError:
        raise
    except (
        SymbolicDynamicsConstructionError,
        SymbolicObjectiveConstructionError,
        AttributeError,
        ArithmeticError,
        KeyError,
        RuntimeError,
        TypeError,
        ValueError,
    ) as exc:
        raise CasadiGraphConstructionError(
            "CasADi symbolic graph construction failed"
        ) from exc


__all__ = [
    "ARTIFACT_STATUS",
    "CASADI_GRAPH_SCHEMA",
    "DIAGNOSTIC_RESIDUAL_ROLE",
    "GRAPH_IDENTITY_SCOPE",
    "GRAPH_STATUS",
    "REFERENCE_DOMAIN_LOWER",
    "REFERENCE_DOMAIN_UPPER",
    "RUNTIME_PARAMETER_INPUT_POLICY",
    "STAGE_REFERENCE_DOMAIN_ORDER",
    "TERMINAL_CONTROL_POLICY",
    "TERMINAL_REFERENCE_DOMAIN_ORDER",
    "CasadiAdapterError",
    "CasadiDependencyError",
    "CasadiGraphBundle",
    "CasadiGraphConstructionError",
    "SymbolicExecutionSlot",
    "SymbolicIssuedValues",
    "SymbolicReferenceValues",
    "SymbolicStageConstraints",
    "SymbolicStageCosts",
    "SymbolicTerminalExpressions",
    "build_casadi_graph",
]
