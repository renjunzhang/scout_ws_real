"""Post-``make_consistent`` validation for the assembled mainline OCP."""

from __future__ import annotations

from typing import Any

from .acados_backend import AcadosBackend, AcadosOcpConstructionError
from .acados_solver_options_adapter import validate_applied_solver_options
from .casadi_graph_contract import CasadiGraphBundle
from .model_contract import MODEL_ID
from .solver_options import SolverOptionsSnapshot


def _is_empty_expression(value: Any) -> bool:
    if value is None:
        return True
    if isinstance(value, (list, tuple)):
        return len(value) == 0
    try:
        return bool(value.is_empty())
    except AttributeError:
        return False


def _require_array_equal(
    backend: AcadosBackend,
    actual: Any,
    expected: Any,
    label: str,
) -> None:
    if not backend.np.array_equal(actual, backend.np.asarray(expected)):
        raise AcadosOcpConstructionError(
            f"acados {label} differs from the graph contract"
        )


def _validate_dimensions(ocp: Any, graph: CasadiGraphBundle) -> None:
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
    expected_zero_dimensions = (
        "nbx",
        "nbx_e",
        "ng",
        "ng_e",
        "nphi_0",
        "nphi",
        "nphi_e",
        "ns_0",
        "ns",
        "ns_e",
    )
    for name, expected in expected_dimensions.items():
        if getattr(ocp.dims, name) != expected:
            raise AcadosOcpConstructionError(
                f"acados dimension {name} differs from the graph contract"
            )
    for name in expected_zero_dimensions:
        if getattr(ocp.dims, name) != 0:
            raise AcadosOcpConstructionError(
                f"acados dimension {name} adds an undeclared constraint or slack"
            )
    if ocp.solver_options.N_horizon != graph.horizon_steps:
        raise AcadosOcpConstructionError(
            "acados horizon differs from the graph contract"
        )


def _validate_expressions(
    backend: AcadosBackend,
    ocp: Any,
    graph: CasadiGraphBundle,
) -> None:
    constraints = graph.stage_constraints
    expected_stage_h = backend.ca.vertcat(
        constraints.nonlinear_h,
        constraints.reference_domain_h,
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
        (ocp.model.con_h_expr_0, expected_stage_h, "initial nonlinear constraint"),
        (ocp.model.con_h_expr, expected_stage_h, "path nonlinear constraint"),
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
    if not _is_empty_expression(ocp.model.f_expl_expr) or not _is_empty_expression(
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


def _validate_cost_and_constraint_types(ocp: Any) -> None:
    if ocp.cost.cost_type_0 != "EXTERNAL" or ocp.cost.cost_type != "EXTERNAL":
        raise AcadosOcpConstructionError("initial/path costs are not EXTERNAL")
    if ocp.cost.cost_type_e != "EXTERNAL":
        raise AcadosOcpConstructionError("terminal cost is not EXTERNAL")
    if any(
        value != "BGH"
        for value in (
            ocp.constraints.constr_type_0,
            ocp.constraints.constr_type,
            ocp.constraints.constr_type_e,
        )
    ):
        raise AcadosOcpConstructionError("constraint type is not explicit BGH")


def _validate_numeric_contract(
    backend: AcadosBackend,
    ocp: Any,
    graph: CasadiGraphBundle,
) -> None:
    constraints = graph.stage_constraints
    if not ocp.constraints.has_x0 or len(ocp.constraints.x0) != graph.nx:
        raise AcadosOcpConstructionError("full initial-state placeholder is missing")
    if len(ocp.parameter_values) != graph.np:
        raise AcadosOcpConstructionError("parameter placeholder width is incorrect")
    stage_lower = constraints.nonlinear_lower + constraints.reference_domain_lower
    stage_upper = constraints.nonlinear_upper + constraints.reference_domain_upper
    _require_array_equal(
        backend,
        ocp.constraints.idxbx_0,
        backend.np.arange(graph.nx),
        "initial-state indices",
    )
    _require_array_equal(
        backend,
        ocp.constraints.idxbxe_0,
        backend.np.arange(graph.nx),
        "initial-state equality indices",
    )
    _require_array_equal(
        backend, ocp.constraints.lbx_0, backend.np.zeros(graph.nx), "x0 lower"
    )
    _require_array_equal(
        backend, ocp.constraints.ubx_0, backend.np.zeros(graph.nx), "x0 upper"
    )
    _require_array_equal(
        backend,
        ocp.constraints.x0,
        backend.np.zeros(graph.nx),
        "x0 placeholder",
    )
    _require_array_equal(
        backend,
        ocp.parameter_values,
        backend.np.zeros(graph.np),
        "parameter placeholder",
    )
    for actual, expected, label in (
        (ocp.constraints.idxbu, constraints.control_indices, "control indices"),
        (ocp.constraints.lbu, constraints.control_lower, "control lower"),
        (ocp.constraints.ubu, constraints.control_upper, "control upper"),
        (ocp.constraints.lh_0, stage_lower, "initial h lower"),
        (ocp.constraints.uh_0, stage_upper, "initial h upper"),
        (ocp.constraints.lh, stage_lower, "path h lower"),
        (ocp.constraints.uh, stage_upper, "path h upper"),
        (
            ocp.constraints.lh_e,
            graph.terminal.reference_domain_lower,
            "terminal h lower",
        ),
        (
            ocp.constraints.uh_e,
            graph.terminal.reference_domain_upper,
            "terminal h upper",
        ),
    ):
        _require_array_equal(backend, actual, expected, label)


def validate_consistent_ocp(
    backend: AcadosBackend,
    ocp: Any,
    graph: CasadiGraphBundle,
    solver_options: SolverOptionsSnapshot,
) -> None:
    """Validate every graph-to-OCP mapping after backend normalization."""

    _validate_dimensions(ocp, graph)
    _validate_expressions(backend, ocp, graph)
    _validate_cost_and_constraint_types(ocp)
    _validate_numeric_contract(backend, ocp, graph)
    validate_applied_solver_options(backend, ocp.solver_options, solver_options)


__all__ = ["validate_consistent_ocp"]
