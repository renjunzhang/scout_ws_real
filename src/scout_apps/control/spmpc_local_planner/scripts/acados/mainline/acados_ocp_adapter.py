"""Lazy, artifact-free Acados OCP assembly for the Stage 3-D graph."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .acados_ocp_contract import (
    ACADOS_INTERFACE_SOURCE_SCHEMA,
    DYNAMICS_IDENTITY_FUNCTION,
    STAGE_COST_IDENTITY_FUNCTION,
    STAGE_H_IDENTITY_FUNCTION,
    TERMINAL_COST_IDENTITY_FUNCTION,
    TERMINAL_H_IDENTITY_FUNCTION,
    AcadosOcpAssembly,
    _build_acados_ocp_assembly,
)
from .casadi_graph_contract import ARTIFACT_STATUS as GRAPH_ARTIFACT_STATUS
from .casadi_graph_contract import (
    GRAPH_STATUS,
    CasadiGraphBundle,
)
from .identity import IdentityError, sha256_bytes, sha256_json
from .model_contract import MODEL_ID
from .solver_options import SolverOptionsSnapshot


class AcadosOcpAdapterError(RuntimeError):
    """Base class for Stage 3-D in-memory OCP assembly failures."""


class AcadosOcpDependencyError(AcadosOcpAdapterError):
    """CasADi, NumPy, or acados-template cannot be loaded."""


class AcadosOcpConstructionError(AcadosOcpAdapterError):
    """Typed inputs or the assembled backend OCP are inconsistent."""


@dataclass(frozen=True)
class _AcadosBackend:
    ca: Any
    np: Any
    template_module: Any
    model_type: Any
    ocp_type: Any


def _require_acados_backend() -> _AcadosBackend:
    try:
        import acados_template
        import casadi as ca
        import numpy as np
    except (ImportError, OSError) as exc:
        raise AcadosOcpDependencyError(
            "CasADi, NumPy, and acados_template are required for OCP assembly"
        ) from exc
    return _AcadosBackend(
        ca,
        np,
        acados_template,
        acados_template.AcadosModel,
        acados_template.AcadosOcp,
    )


def _acados_backend_identity(
    backend: _AcadosBackend,
    ocp: Any,
) -> tuple[str, str]:
    module_file = getattr(backend.template_module, "__file__", None)
    if type(module_file) is not str:
        raise AcadosOcpConstructionError(
            "acados_template source location is unavailable"
        )
    package_directory = Path(module_file).resolve().parent
    selected_sources = (
        "__init__.py",
        "acados_code_gen_opts.py",
        "acados_model.py",
        "acados_ocp.py",
        "acados_ocp_constraints.py",
        "acados_ocp_cost.py",
        "acados_ocp_options.py",
    )
    source_identity = []
    try:
        for name in selected_sources:
            source_identity.append(
                {
                    "relative_path": name,
                    "raw_bytes_sha256": sha256_bytes(
                        (package_directory / name).read_bytes()
                    ),
                }
            )
        interface_sha256 = sha256_json(
            {
                "schema": ACADOS_INTERFACE_SOURCE_SCHEMA,
                "files": source_identity,
            }
        )
        interface_source_root = package_directory.parents[2]
        library_commit_file = (
            Path(ocp.code_gen_opts.acados_lib_path) / "git_commit_hash"
        ).resolve()
        library_source_root = library_commit_file.parent.parent
    except (IndexError, OSError) as exc:
        raise AcadosOcpConstructionError(
            "acados backend source identity cannot be read"
        ) from exc
    binding_status = (
        "MATCHED_SOURCE_ROOT"
        if interface_source_root == library_source_root
        else "UNRESOLVED_SOURCE_ROOT_MISMATCH"
    )
    return interface_sha256, binding_status


def _casadi_function_sha256(
    backend: _AcadosBackend,
    name: str,
    inputs: list[Any],
    outputs: list[Any],
) -> str:
    serialized = backend.ca.Function(name, inputs, outputs).serialize()
    if type(serialized) is not str:
        raise AcadosOcpConstructionError(
            "CasADi expression serialization did not return text"
        )
    return sha256_bytes(serialized.encode("utf-8"))


def _symbolic_expression_identity(
    backend: _AcadosBackend,
    graph: CasadiGraphBundle,
    stage_h: Any,
) -> dict[str, str]:
    return {
        "dynamics": _casadi_function_sha256(
            backend,
            DYNAMICS_IDENTITY_FUNCTION,
            [graph.x, graph.u, graph.p],
            [graph.x_next],
        ),
        "stage_cost": _casadi_function_sha256(
            backend,
            STAGE_COST_IDENTITY_FUNCTION,
            [graph.x, graph.u, graph.p],
            [graph.stage_costs.total],
        ),
        "terminal_cost": _casadi_function_sha256(
            backend,
            TERMINAL_COST_IDENTITY_FUNCTION,
            [graph.x, graph.p],
            [graph.terminal.total_cost],
        ),
        "stage_h": _casadi_function_sha256(
            backend,
            STAGE_H_IDENTITY_FUNCTION,
            [graph.x, graph.u, graph.p],
            [stage_h],
        ),
        "terminal_h": _casadi_function_sha256(
            backend,
            TERMINAL_H_IDENTITY_FUNCTION,
            [graph.x, graph.p],
            [graph.terminal.reference_domain_h],
        ),
    }


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


def _apply_solver_options(
    backend: _AcadosBackend,
    target: Any,
    snapshot: SolverOptionsSnapshot,
) -> None:
    target.N_horizon = snapshot.horizon_steps
    target.tf = float(snapshot.time_horizon_sec)
    target.time_steps = backend.np.asarray(
        [float(value) for value in snapshot.time_steps],
        dtype=float,
    )
    target.cost_scaling = backend.np.asarray(snapshot.cost_scaling, dtype=float)
    for name in (
        "integrator_type",
        "cost_discretization",
        "nlp_solver_type",
        "nlp_solver_max_iter",
        "nlp_solver_tol_stat",
        "nlp_solver_tol_eq",
        "nlp_solver_tol_ineq",
        "nlp_solver_tol_comp",
        "hessian_approx",
        "ext_cost_num_hess",
        "regularize_method",
        "reg_epsilon",
        "levenberg_marquardt",
        "globalization",
        "globalization_fixed_step_length",
        "qp_solver",
        "qp_solver_cond_N",
        "qp_solver_cond_ric_alg",
        "qp_solver_ric_alg",
        "qp_solver_iter_max",
        "qp_solver_tol_stat",
        "qp_solver_tol_eq",
        "qp_solver_tol_ineq",
        "qp_solver_tol_comp",
        "qp_solver_warm_start",
        "nlp_solver_warm_start_first_qp",
        "nlp_solver_warm_start_first_qp_from_nlp",
        "hpipm_mode",
        "print_level",
    ):
        setattr(target, name, getattr(snapshot, name))
    # Acados serializes these flags as integer options.  Keep the typed public
    # snapshot boolean while making the backend representation unambiguous.
    target.exact_hess_dyn = int(snapshot.exact_hess_dyn)
    target.exact_hess_cost = int(snapshot.exact_hess_cost)
    target.exact_hess_constr = int(snapshot.exact_hess_constr)


def _validate_applied_solver_options(
    backend: _AcadosBackend,
    actual: Any,
    expected: SolverOptionsSnapshot,
) -> None:
    scalar_names = (
        "N_horizon",
        "integrator_type",
        "cost_discretization",
        "nlp_solver_type",
        "nlp_solver_max_iter",
        "nlp_solver_tol_stat",
        "nlp_solver_tol_eq",
        "nlp_solver_tol_ineq",
        "nlp_solver_tol_comp",
        "hessian_approx",
        "ext_cost_num_hess",
        "regularize_method",
        "reg_epsilon",
        "levenberg_marquardt",
        "globalization",
        "globalization_fixed_step_length",
        "qp_solver",
        "qp_solver_cond_N",
        "qp_solver_cond_ric_alg",
        "qp_solver_ric_alg",
        "qp_solver_iter_max",
        "qp_solver_tol_stat",
        "qp_solver_tol_eq",
        "qp_solver_tol_ineq",
        "qp_solver_tol_comp",
        "qp_solver_warm_start",
        "nlp_solver_warm_start_first_qp",
        "nlp_solver_warm_start_first_qp_from_nlp",
        "hpipm_mode",
        "print_level",
    )
    for name in scalar_names:
        expected_value = (
            expected.horizon_steps if name == "N_horizon" else getattr(expected, name)
        )
        if getattr(actual, name) != expected_value:
            raise AcadosOcpConstructionError(
                f"acados solver option {name} differs from the typed snapshot"
            )
    for name in ("exact_hess_dyn", "exact_hess_cost", "exact_hess_constr"):
        if getattr(actual, name) != int(getattr(expected, name)):
            raise AcadosOcpConstructionError(
                f"acados solver option {name} differs from the typed snapshot"
            )
    if actual.tf != float(expected.time_horizon_sec):
        raise AcadosOcpConstructionError("acados time horizon differs from snapshot")
    expected_steps = backend.np.asarray(
        [float(value) for value in expected.time_steps], dtype=float
    )
    expected_scaling = backend.np.asarray(expected.cost_scaling, dtype=float)
    if not backend.np.array_equal(actual.time_steps, expected_steps):
        raise AcadosOcpConstructionError("acados time steps differ from snapshot")
    if not backend.np.array_equal(actual.cost_scaling, expected_scaling):
        raise AcadosOcpConstructionError("acados cost scaling differs from snapshot")


def _validate_consistent_ocp(
    backend: _AcadosBackend,
    ocp: Any,
    graph: CasadiGraphBundle,
    solver_options: SolverOptionsSnapshot,
) -> None:
    constraints = graph.stage_constraints
    expected_dimensions = {
        "nx": graph.nx,
        "nu": graph.nu,
        "np": graph.np,
        "nx_next": graph.nx,
        "nbx_0": graph.nx,
        "nbu": len(graph.stage_constraints.control_indices),
        "nh_0": len(graph.stage_constraints.nonlinear_h_order) + 1,
        "nh": len(graph.stage_constraints.nonlinear_h_order) + 1,
        "nh_e": 1,
    }
    for name, expected in expected_dimensions.items():
        if getattr(ocp.dims, name) != expected:
            raise AcadosOcpConstructionError(
                f"acados dimension {name} differs from the graph contract"
            )
    if ocp.solver_options.N_horizon != graph.horizon_steps:
        raise AcadosOcpConstructionError(
            "acados horizon differs from the graph contract"
        )
    expression_pairs = (
        (ocp.model.x, graph.x, "state symbol"),
        (ocp.model.u, graph.u, "control symbol"),
        (ocp.model.p, graph.p, "parameter symbol"),
        (ocp.model.disc_dyn_expr, graph.x_next, "discrete dynamics"),
        (
            ocp.model.cost_expr_ext_cost_0,
            graph.stage_costs.total,
            "initial cost",
        ),
        (ocp.model.cost_expr_ext_cost, graph.stage_costs.total, "path cost"),
        (
            ocp.model.cost_expr_ext_cost_e,
            graph.terminal.total_cost,
            "terminal cost",
        ),
        (
            ocp.model.con_h_expr_0,
            backend.ca.vertcat(
                constraints.nonlinear_h,
                constraints.reference_domain_h,
            ),
            "initial nonlinear constraint",
        ),
        (
            ocp.model.con_h_expr,
            backend.ca.vertcat(
                constraints.nonlinear_h,
                constraints.reference_domain_h,
            ),
            "path nonlinear constraint",
        ),
        (
            ocp.model.con_h_expr_e,
            graph.terminal.reference_domain_h,
            "terminal nonlinear constraint",
        ),
    )
    for actual, expected, label in expression_pairs:
        if not backend.ca.is_equal(actual, expected, 2):
            raise AcadosOcpConstructionError(
                f"acados {label} differs from the graph contract"
            )
    if ocp.model.name != MODEL_ID:
        raise AcadosOcpConstructionError("acados model identity is not canonical")
    if ocp.cost.cost_type_0 != "EXTERNAL" or ocp.cost.cost_type != "EXTERNAL":
        raise AcadosOcpConstructionError("initial/path costs are not EXTERNAL")
    if ocp.cost.cost_type_e != "EXTERNAL":
        raise AcadosOcpConstructionError("terminal cost is not EXTERNAL")
    if not ocp.constraints.has_x0 or len(ocp.constraints.x0) != graph.nx:
        raise AcadosOcpConstructionError("full initial-state placeholder is missing")
    if len(ocp.parameter_values) != graph.np:
        raise AcadosOcpConstructionError("parameter placeholder width is incorrect")

    def require_array_equal(actual: Any, expected: Any, label: str) -> None:
        if not backend.np.array_equal(actual, backend.np.asarray(expected)):
            raise AcadosOcpConstructionError(
                f"acados {label} differs from the graph contract"
            )

    stage_lower = constraints.nonlinear_lower + constraints.reference_domain_lower
    stage_upper = constraints.nonlinear_upper + constraints.reference_domain_upper
    require_array_equal(
        ocp.constraints.idxbx_0,
        backend.np.arange(graph.nx),
        "initial-state indices",
    )
    require_array_equal(
        ocp.constraints.idxbxe_0,
        backend.np.arange(graph.nx),
        "initial-state equality indices",
    )
    require_array_equal(ocp.constraints.lbx_0, backend.np.zeros(graph.nx), "x0 lower")
    require_array_equal(ocp.constraints.ubx_0, backend.np.zeros(graph.nx), "x0 upper")
    require_array_equal(
        ocp.parameter_values,
        backend.np.zeros(graph.np),
        "parameter placeholder",
    )
    require_array_equal(
        ocp.constraints.idxbu,
        constraints.control_indices,
        "control indices",
    )
    require_array_equal(ocp.constraints.lbu, constraints.control_lower, "control lower")
    require_array_equal(ocp.constraints.ubu, constraints.control_upper, "control upper")
    require_array_equal(ocp.constraints.lh_0, stage_lower, "initial h lower")
    require_array_equal(ocp.constraints.uh_0, stage_upper, "initial h upper")
    require_array_equal(ocp.constraints.lh, stage_lower, "path h lower")
    require_array_equal(ocp.constraints.uh, stage_upper, "path h upper")
    require_array_equal(
        ocp.constraints.lh_e,
        graph.terminal.reference_domain_lower,
        "terminal h lower",
    )
    require_array_equal(
        ocp.constraints.uh_e,
        graph.terminal.reference_domain_upper,
        "terminal h upper",
    )
    if any(
        value != "BGH"
        for value in (
            ocp.constraints.constr_type_0,
            ocp.constraints.constr_type,
            ocp.constraints.constr_type_e,
        )
    ):
        raise AcadosOcpConstructionError("constraint type is not explicit BGH")

    def is_empty_expression(value: Any) -> bool:
        if value is None:
            return True
        if isinstance(value, (list, tuple)):
            return len(value) == 0
        try:
            return bool(value.is_empty())
        except AttributeError:
            return False

    if not is_empty_expression(ocp.model.f_expl_expr) or not is_empty_expression(
        ocp.model.f_impl_expr
    ):
        raise AcadosOcpConstructionError(
            "continuous dynamics expressions are forbidden in the discrete OCP"
        )
    terminal_symbols = {
        str(symbol).split("_")[0]
        for symbol in backend.ca.symvar(
            backend.ca.vertcat(
                ocp.model.cost_expr_ext_cost_e,
                ocp.model.con_h_expr_e,
            )
        )
    }
    if terminal_symbols != {"x", "p"}:
        raise AcadosOcpConstructionError(
            "terminal expressions must depend on x_N and p_N only"
        )
    _validate_applied_solver_options(backend, ocp.solver_options, solver_options)


def assemble_acados_ocp(
    graph: CasadiGraphBundle,
    solver_options: SolverOptionsSnapshot,
) -> AcadosOcpAssembly:
    """Assemble and validate an ``AcadosOcp`` entirely in memory.

    This function does not instantiate ``AcadosOcpSolver`` and never calls
    code generation, JSON dumping, template rendering, compilation, or artifact
    publication.  ``make_consistent`` performs backend validation and read-only
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
        model = backend.model_type()
        model.name = MODEL_ID
        model.x = checked_graph.x
        model.u = checked_graph.u
        model.p = checked_graph.p
        model.disc_dyn_expr = checked_graph.x_next

        model.cost_expr_ext_cost_0 = checked_graph.stage_costs.total
        model.cost_expr_ext_cost = checked_graph.stage_costs.total
        model.cost_expr_ext_cost_e = checked_graph.terminal.total_cost

        stage_h = backend.ca.vertcat(
            constraints.nonlinear_h,
            constraints.reference_domain_h,
        )
        stage_lower = constraints.nonlinear_lower + constraints.reference_domain_lower
        stage_upper = constraints.nonlinear_upper + constraints.reference_domain_upper
        stage_order = constraints.nonlinear_h_order + constraints.reference_domain_order
        model.con_h_expr_0 = stage_h
        model.con_h_expr = stage_h
        model.con_h_expr_e = checked_graph.terminal.reference_domain_h

        ocp = backend.ocp_type()
        ocp.model = model
        ocp.cost.cost_type_0 = "EXTERNAL"
        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"
        ocp.constraints.constr_type_0 = "BGH"
        ocp.constraints.constr_type = "BGH"
        ocp.constraints.constr_type_e = "BGH"
        ocp.constraints.x0 = backend.np.zeros(checked_graph.nx, dtype=float)
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
            checked_graph.terminal.reference_domain_lower,
            dtype=float,
        )
        ocp.constraints.uh_e = backend.np.asarray(
            checked_graph.terminal.reference_domain_upper,
            dtype=float,
        )
        ocp.parameter_values = backend.np.zeros(checked_graph.np, dtype=float)
        _apply_solver_options(backend, ocp.solver_options, checked_options)
        ocp.make_consistent(verbose=False)
        _validate_consistent_ocp(backend, ocp, checked_graph, checked_options)

        bounds_snapshot_sha256 = sha256_json(checked_graph.bounds.to_dict())
        acados_git_commit = str(ocp.code_gen_opts.acados_version or "unknown")
        interface_sha256, backend_binding_status = _acados_backend_identity(
            backend,
            ocp,
        )
        expression_identity = _symbolic_expression_identity(
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
            terminal_constraint_order=(checked_graph.terminal.reference_domain_order),
            terminal_constraint_lower=(checked_graph.terminal.reference_domain_lower),
            terminal_constraint_upper=(checked_graph.terminal.reference_domain_upper),
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
