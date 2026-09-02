"""Bind and verify typed solver options on an Acados backend object."""

from __future__ import annotations

from typing import Any

from .acados_backend import AcadosBackend, AcadosOcpConstructionError
from .solver_options import SolverOptionsSnapshot

ACADOS_SCALAR_SOLVER_OPTION_FIELDS = (
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
ACADOS_INTEGER_BOOLEAN_SOLVER_OPTION_FIELDS = (
    "exact_hess_dyn",
    "exact_hess_cost",
    "exact_hess_constr",
)


def apply_solver_options(
    backend: AcadosBackend,
    target: Any,
    snapshot: SolverOptionsSnapshot,
) -> None:
    """Apply the typed snapshot with the conversions required by Acados 0.5.4."""

    target.N_horizon = snapshot.horizon_steps
    target.tf = float(snapshot.time_horizon_sec)
    target.time_steps = backend.np.asarray(
        [float(value) for value in snapshot.time_steps],
        dtype=float,
    )
    target.cost_scaling = backend.np.asarray(snapshot.cost_scaling, dtype=float)
    for name in ACADOS_SCALAR_SOLVER_OPTION_FIELDS:
        setattr(target, name, getattr(snapshot, name))
    # Acados serializes these flags as integer options. Keep the typed public
    # snapshot boolean while making the backend representation unambiguous.
    for name in ACADOS_INTEGER_BOOLEAN_SOLVER_OPTION_FIELDS:
        setattr(target, name, int(getattr(snapshot, name)))


def validate_applied_solver_options(
    backend: AcadosBackend,
    actual: Any,
    expected: SolverOptionsSnapshot,
) -> None:
    """Fail if ``make_consistent`` changes an explicitly frozen option."""

    if actual.N_horizon != expected.horizon_steps:
        raise AcadosOcpConstructionError(
            "acados solver option N_horizon differs from the typed snapshot"
        )
    for name in ACADOS_SCALAR_SOLVER_OPTION_FIELDS:
        expected_value = getattr(expected, name)
        if getattr(actual, name) != expected_value:
            raise AcadosOcpConstructionError(
                f"acados solver option {name} differs from the typed snapshot"
            )
    for name in ACADOS_INTEGER_BOOLEAN_SOLVER_OPTION_FIELDS:
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


__all__ = [
    "ACADOS_INTEGER_BOOLEAN_SOLVER_OPTION_FIELDS",
    "ACADOS_SCALAR_SOLVER_OPTION_FIELDS",
    "apply_solver_options",
    "validate_applied_solver_options",
]
