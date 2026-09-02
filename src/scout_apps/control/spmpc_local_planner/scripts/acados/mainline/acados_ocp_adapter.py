"""Lazy, artifact-free Acados OCP assembly for the Stage 3-D graph."""

from __future__ import annotations

from typing import Any

from .acados_backend import (
    AcadosBackend,
    AcadosOcpAdapterError,
    AcadosOcpConstructionError,
    AcadosOcpDependencyError,
    build_symbolic_expression_identity,
    read_acados_backend_identity,
)
from .acados_backend import require_acados_backend as _require_acados_backend
from .acados_ocp_contract import AcadosOcpAssembly, _build_acados_ocp_assembly
from .acados_ocp_validation import validate_consistent_ocp
from .acados_solver_options_adapter import apply_solver_options
from .casadi_graph_contract import ARTIFACT_STATUS as GRAPH_ARTIFACT_STATUS
from .casadi_graph_contract import GRAPH_STATUS, CasadiGraphBundle
from .identity import IdentityError, sha256_json
from .model_contract import MODEL_ID
from .solver_options import SolverOptionsSnapshot


def _validate_typed_inputs(
    graph: Any,
    solver_options: Any,
) -> tuple[CasadiGraphBundle, SolverOptionsSnapshot]:
    if type(graph) is not CasadiGraphBundle:
        raise AcadosOcpConstructionError(
            "graph must be the exact CasadiGraphBundle type"
        )
    if type(solver_options) is not SolverOptionsSnapshot:
        raise AcadosOcpConstructionError(
            "solver_options must be the exact SolverOptionsSnapshot type"
        )
    if (
        graph.graph_status != GRAPH_STATUS
        or graph.artifact_status != GRAPH_ARTIFACT_STATUS
    ):
        raise AcadosOcpConstructionError(
            "graph must be GRAPH_BUILT/NO_ARTIFACT before OCP assembly"
        )
    if (
        graph.horizon_steps != solver_options.horizon_steps
        or graph.release_frequency_hz != solver_options.release_frequency_hz
        or graph.release_period_sec != solver_options.time_step_sec
    ):
        raise AcadosOcpConstructionError(
            "graph horizon differs from the solver-option snapshot"
        )
    try:
        expected_options_sha256 = solver_options.to_dict()["semantic_identity"][
            "sha256"
        ]
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise AcadosOcpConstructionError("solver-option snapshot is malformed") from exc
    if expected_options_sha256 != solver_options.semantic_sha256:
        raise AcadosOcpConstructionError(
            "solver-option semantic identity is inconsistent"
        )
    return graph, solver_options


def _build_acados_model(
    backend: AcadosBackend,
    graph: CasadiGraphBundle,
    stage_h: Any,
) -> Any:
    model = backend.model_type()
    model.name = MODEL_ID
    model.x = graph.x
    model.u = graph.u
    model.p = graph.p
    model.disc_dyn_expr = graph.x_next
    model.cost_expr_ext_cost_0 = graph.stage_costs.total
    model.cost_expr_ext_cost = graph.stage_costs.total
    model.cost_expr_ext_cost_e = graph.terminal.total_cost
    model.con_h_expr_0 = stage_h
    model.con_h_expr = stage_h
    model.con_h_expr_e = graph.terminal.reference_domain_h
    return model


def _build_backend_ocp(
    backend: AcadosBackend,
    graph: CasadiGraphBundle,
    solver_options: SolverOptionsSnapshot,
    stage_h: Any,
    stage_lower: tuple[float, ...],
    stage_upper: tuple[float, ...],
) -> Any:
    constraints = graph.stage_constraints
    ocp = backend.ocp_type()
    ocp.model = _build_acados_model(backend, graph, stage_h)
    ocp.cost.cost_type_0 = "EXTERNAL"
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.constraints.constr_type_0 = "BGH"
    ocp.constraints.constr_type = "BGH"
    ocp.constraints.constr_type_e = "BGH"
    ocp.constraints.x0 = backend.np.zeros(graph.nx, dtype=float)
    ocp.constraints.idxbu = backend.np.asarray(
        constraints.control_indices,
        dtype=int,
    )
    ocp.constraints.lbu = backend.np.asarray(constraints.control_lower, dtype=float)
    ocp.constraints.ubu = backend.np.asarray(constraints.control_upper, dtype=float)
    ocp.constraints.lh_0 = backend.np.asarray(stage_lower, dtype=float)
    ocp.constraints.uh_0 = backend.np.asarray(stage_upper, dtype=float)
    ocp.constraints.lh = backend.np.asarray(stage_lower, dtype=float)
    ocp.constraints.uh = backend.np.asarray(stage_upper, dtype=float)
    ocp.constraints.lh_e = backend.np.asarray(
        graph.terminal.reference_domain_lower,
        dtype=float,
    )
    ocp.constraints.uh_e = backend.np.asarray(
        graph.terminal.reference_domain_upper,
        dtype=float,
    )
    ocp.parameter_values = backend.np.zeros(graph.np, dtype=float)
    apply_solver_options(backend, ocp.solver_options, solver_options)
    return ocp


def assemble_acados_ocp(
    graph: CasadiGraphBundle,
    solver_options: SolverOptionsSnapshot,
) -> AcadosOcpAssembly:
    """Assemble and validate an ``AcadosOcp`` entirely in memory.

    This function does not instantiate ``AcadosOcpSolver`` and never calls
    code generation, JSON dumping, template rendering, compilation, or artifact
    publication. ``make_consistent`` performs backend validation and read-only
    discovery of the installed acados metadata.
    """

    checked_graph, checked_options = _validate_typed_inputs(graph, solver_options)
    backend = _require_acados_backend()
    if str(getattr(backend.ca, "__version__", "unknown")) != graph.casadi_version:
        raise AcadosOcpConstructionError(
            "loaded CasADi version differs from the symbolic graph"
        )
    try:
        constraints = checked_graph.stage_constraints
        stage_h = backend.ca.vertcat(
            constraints.nonlinear_h,
            constraints.reference_domain_h,
        )
        stage_lower = constraints.nonlinear_lower + constraints.reference_domain_lower
        stage_upper = constraints.nonlinear_upper + constraints.reference_domain_upper
        stage_order = constraints.nonlinear_h_order + constraints.reference_domain_order
        ocp = _build_backend_ocp(
            backend,
            checked_graph,
            checked_options,
            stage_h,
            stage_lower,
            stage_upper,
        )
        ocp.make_consistent(verbose=False)
        validate_consistent_ocp(backend, ocp, checked_graph, checked_options)

        bounds_snapshot_sha256 = sha256_json(checked_graph.bounds.to_dict())
        acados_git_commit = str(ocp.code_gen_opts.acados_version or "unknown")
        interface_sha256, backend_binding_status = read_acados_backend_identity(
            backend,
            ocp,
        )
        expression_identity = build_symbolic_expression_identity(
            backend,
            checked_graph,
            stage_h,
        )
        return _build_acados_ocp_assembly(
            ocp=ocp,
            horizon_steps=checked_graph.horizon_steps,
            nx=checked_graph.nx,
            nu=checked_graph.nu,
            np=checked_graph.np,
            graph_semantic_sha256=checked_graph.graph_semantic_sha256,
            bounds_snapshot_sha256=bounds_snapshot_sha256,
            solver_options_semantic_sha256=checked_options.semantic_sha256,
            capacity_contract_sha256=checked_graph.capacity_contract_sha256,
            development_layout_sha256=checked_graph.development_layout_sha256,
            solver_parameter_layout_sha256=(
                checked_graph.solver_parameter_layout_sha256
            ),
            casadi_version=checked_graph.casadi_version,
            acados_git_commit=acados_git_commit,
            acados_interface_source_sha256=interface_sha256,
            acados_backend_binding_status=backend_binding_status,
            dynamics_expression_sha256=expression_identity["dynamics"],
            stage_cost_expression_sha256=expression_identity["stage_cost"],
            terminal_cost_expression_sha256=expression_identity["terminal_cost"],
            stage_h_expression_sha256=expression_identity["stage_h"],
            terminal_h_expression_sha256=expression_identity["terminal_h"],
            stage_constraint_order=stage_order,
            stage_constraint_lower=stage_lower,
            stage_constraint_upper=stage_upper,
            terminal_constraint_order=checked_graph.terminal.reference_domain_order,
            terminal_constraint_lower=checked_graph.terminal.reference_domain_lower,
            terminal_constraint_upper=checked_graph.terminal.reference_domain_upper,
            control_order=constraints.control_order,
            control_indices=constraints.control_indices,
            control_lower=constraints.control_lower,
            control_upper=constraints.control_upper,
        )
    except AcadosOcpAdapterError:
        raise
    except IdentityError as exc:
        raise AcadosOcpConstructionError(
            "Acados OCP identity cannot be encoded"
        ) from exc
    except Exception as exc:
        raise AcadosOcpConstructionError(
            "Acados OCP in-memory assembly failed"
        ) from exc


__all__ = [
    "AcadosOcpAdapterError",
    "AcadosOcpAssembly",
    "AcadosOcpConstructionError",
    "AcadosOcpDependencyError",
    "assemble_acados_ocp",
]
