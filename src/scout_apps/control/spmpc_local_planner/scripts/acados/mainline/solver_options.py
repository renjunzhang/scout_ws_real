"""Dependency-free solver-option authority for the Stage 3-D OCP.

This module freezes the numerical choices that affect the development solver.
It deliberately does not import CasADi or ``acados_template``; the backend
adapter is responsible only for applying this typed snapshot verbatim.
"""

from __future__ import annotations

from dataclasses import InitVar, dataclass
from fractions import Fraction
from typing import Any

from .development_layout import DevelopmentLayout
from .identity import IdentityError, sha256_json

SOLVER_OPTIONS_SCHEMA = "spmpc_mainline_solver_options_v1"
SOLVER_OPTIONS_STATUS = "DEV_UNVALIDATED"
TARGET_PERFORMANCE_STATUS = "NOT_BENCHMARKED"
SOLVER_OPTIONS_ARTIFACT_STATUS = "NO_ARTIFACT"
SOLVER_OPTIONS_SCOPE = "TYPED_DEVELOPMENT_NUMERICAL_OPTIONS"
TIME_STEP_POLICY = "UNIFORM_RELEASE_PERIOD"
COST_SCALING_POLICY = "EXPLICIT_ONE_FOR_EVERY_STAGE_AND_TERMINAL"
CONTINUOUS_INTEGRATOR_OPTIONS_POLICY = "NOT_APPLICABLE_TO_DISCRETE_DYNAMICS"

_SNAPSHOT_TOKEN = object()


class SolverOptionsError(ValueError):
    """The unique development solver-option snapshot cannot be built."""


@dataclass(frozen=True)
class SolverOptionsSnapshot:
    """Immutable, backend-independent numerical option snapshot."""

    horizon_steps: int
    release_frequency_hz: int
    time_step_sec: Fraction
    time_horizon_sec: Fraction
    time_steps: tuple[Fraction, ...]
    cost_scaling: tuple[float, ...]
    integrator_type: str
    cost_discretization: str
    nlp_solver_type: str
    nlp_solver_max_iter: int
    nlp_solver_tol_stat: float
    nlp_solver_tol_eq: float
    nlp_solver_tol_ineq: float
    nlp_solver_tol_comp: float
    hessian_approx: str
    exact_hess_dyn: bool
    exact_hess_cost: bool
    exact_hess_constr: bool
    ext_cost_num_hess: int
    regularize_method: str
    reg_epsilon: float
    levenberg_marquardt: float
    globalization: str
    globalization_fixed_step_length: float
    qp_solver: str
    qp_solver_cond_N: int
    qp_solver_cond_ric_alg: int
    qp_solver_ric_alg: int
    qp_solver_iter_max: int
    qp_solver_tol_stat: float
    qp_solver_tol_eq: float
    qp_solver_tol_ineq: float
    qp_solver_tol_comp: float
    qp_solver_warm_start: int
    nlp_solver_warm_start_first_qp: bool
    nlp_solver_warm_start_first_qp_from_nlp: bool
    hpipm_mode: str
    print_level: int
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = SOLVER_OPTIONS_SCHEMA
    status: str = SOLVER_OPTIONS_STATUS
    target_performance_status: str = TARGET_PERFORMANCE_STATUS
    artifact_status: str = SOLVER_OPTIONS_ARTIFACT_STATUS

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _SNAPSHOT_TOKEN:
            raise SolverOptionsError(
                "SolverOptionsSnapshot requires build_solver_options_snapshot"
            )
        if self.schema_version != SOLVER_OPTIONS_SCHEMA:
            raise SolverOptionsError("solver-option schema is not canonical")
        if (
            self.status != SOLVER_OPTIONS_STATUS
            or self.target_performance_status != TARGET_PERFORMANCE_STATUS
            or self.artifact_status != SOLVER_OPTIONS_ARTIFACT_STATUS
        ):
            raise SolverOptionsError("solver-option status is not canonical")
        if len(self.semantic_sha256) != 64:
            raise SolverOptionsError("solver-option semantic identity is invalid")

    def to_dict(self) -> dict[str, Any]:
        """Return the canonical JSON-compatible option snapshot."""

        payload = _snapshot_payload(self)
        payload["semantic_identity"] = {
            "sha256": self.semantic_sha256,
            "scope": SOLVER_OPTIONS_SCOPE,
        }
        return payload


def _fraction_dict(value: Fraction) -> dict[str, int]:
    return {"numerator": value.numerator, "denominator": value.denominator}


def _snapshot_payload(snapshot: SolverOptionsSnapshot) -> dict[str, Any]:
    return {
        "schema_version": snapshot.schema_version,
        "scope": SOLVER_OPTIONS_SCOPE,
        "status": {
            "solver_options": snapshot.status,
            "target_performance": snapshot.target_performance_status,
            "artifact": snapshot.artifact_status,
        },
        "horizon": {
            "N": snapshot.horizon_steps,
            "release_frequency_hz": snapshot.release_frequency_hz,
            "time_step_sec": _fraction_dict(snapshot.time_step_sec),
            "time_horizon_sec": _fraction_dict(snapshot.time_horizon_sec),
            "time_steps": {
                "policy": TIME_STEP_POLICY,
                "count": len(snapshot.time_steps),
                "uniform_value_sec": _fraction_dict(snapshot.time_step_sec),
            },
            "cost_scaling": {
                "policy": COST_SCALING_POLICY,
                "count": len(snapshot.cost_scaling),
                "uniform_value": 1.0,
            },
        },
        "discretization": {
            "integrator_type": snapshot.integrator_type,
            "cost_discretization": snapshot.cost_discretization,
            "continuous_integrator_options": (CONTINUOUS_INTEGRATOR_OPTIONS_POLICY),
        },
        "nlp": {
            "solver_type": snapshot.nlp_solver_type,
            "max_iter": snapshot.nlp_solver_max_iter,
            "tolerances": {
                "stat": snapshot.nlp_solver_tol_stat,
                "eq": snapshot.nlp_solver_tol_eq,
                "ineq": snapshot.nlp_solver_tol_ineq,
                "comp": snapshot.nlp_solver_tol_comp,
            },
            "warm_start_first_qp": snapshot.nlp_solver_warm_start_first_qp,
            "warm_start_first_qp_from_nlp": (
                snapshot.nlp_solver_warm_start_first_qp_from_nlp
            ),
        },
        "hessian": {
            "approximation": snapshot.hessian_approx,
            "exact_dyn": snapshot.exact_hess_dyn,
            "exact_cost": snapshot.exact_hess_cost,
            "exact_constr": snapshot.exact_hess_constr,
            "external_cost_numerical_hessian": snapshot.ext_cost_num_hess,
        },
        "regularization": {
            "method": snapshot.regularize_method,
            "epsilon": snapshot.reg_epsilon,
            "levenberg_marquardt": snapshot.levenberg_marquardt,
        },
        "globalization": {
            "method": snapshot.globalization,
            "fixed_step_length": snapshot.globalization_fixed_step_length,
        },
        "qp": {
            "solver": snapshot.qp_solver,
            "condensed_horizon": snapshot.qp_solver_cond_N,
            "condensing_riccati_algorithm": snapshot.qp_solver_cond_ric_alg,
            "riccati_algorithm": snapshot.qp_solver_ric_alg,
            "max_iter": snapshot.qp_solver_iter_max,
            "tolerances": {
                "stat": snapshot.qp_solver_tol_stat,
                "eq": snapshot.qp_solver_tol_eq,
                "ineq": snapshot.qp_solver_tol_ineq,
                "comp": snapshot.qp_solver_tol_comp,
            },
            "warm_start_level": snapshot.qp_solver_warm_start,
            "hpipm_mode": snapshot.hpipm_mode,
        },
        "diagnostics": {"print_level": snapshot.print_level},
    }


def build_solver_options_snapshot(
    development_layout: DevelopmentLayout,
) -> SolverOptionsSnapshot:
    """Build the sole Stage 3-D numerical option candidate.

    These are development choices, not measured performance evidence.  Target
    benchmarking may replace them only by creating a new typed snapshot and a
    new artifact identity.
    """

    if type(development_layout) is not DevelopmentLayout:
        raise SolverOptionsError(
            "development_layout must be the exact DevelopmentLayout type"
        )
    horizon_steps = development_layout.horizon_steps
    time_step_sec = development_layout.release_period_sec
    if (
        horizon_steps != 60
        or development_layout.release_frequency_hz != 30
        or time_step_sec != Fraction(1, 30)
    ):
        raise SolverOptionsError("development horizon/release grid is not canonical")

    values = {
        "horizon_steps": horizon_steps,
        "release_frequency_hz": development_layout.release_frequency_hz,
        "time_step_sec": time_step_sec,
        "time_horizon_sec": horizon_steps * time_step_sec,
        "time_steps": (time_step_sec,) * horizon_steps,
        "cost_scaling": (1.0,) * (horizon_steps + 1),
        "integrator_type": "DISCRETE",
        "cost_discretization": "EULER",
        "nlp_solver_type": "SQP_RTI",
        "nlp_solver_max_iter": 1,
        "nlp_solver_tol_stat": 1.0e-6,
        "nlp_solver_tol_eq": 1.0e-6,
        "nlp_solver_tol_ineq": 1.0e-6,
        "nlp_solver_tol_comp": 1.0e-6,
        "hessian_approx": "EXACT",
        "exact_hess_dyn": True,
        "exact_hess_cost": True,
        "exact_hess_constr": True,
        "ext_cost_num_hess": 0,
        "regularize_method": "PROJECT",
        "reg_epsilon": 1.0e-4,
        "levenberg_marquardt": 1.0e-3,
        "globalization": "FIXED_STEP",
        "globalization_fixed_step_length": 1.0,
        "qp_solver": "PARTIAL_CONDENSING_HPIPM",
        # No condensing is the conservative development baseline. Target-side
        # profiling may choose a smaller horizon under a new artifact identity.
        "qp_solver_cond_N": horizon_steps,
        "qp_solver_cond_ric_alg": 1,
        "qp_solver_ric_alg": 1,
        "qp_solver_iter_max": 50,
        "qp_solver_tol_stat": 1.0e-6,
        "qp_solver_tol_eq": 1.0e-6,
        "qp_solver_tol_ineq": 1.0e-6,
        "qp_solver_tol_comp": 1.0e-6,
        "qp_solver_warm_start": 0,
        "nlp_solver_warm_start_first_qp": False,
        "nlp_solver_warm_start_first_qp_from_nlp": False,
        "hpipm_mode": "BALANCE",
        "print_level": 0,
    }
    provisional = SolverOptionsSnapshot(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_SNAPSHOT_TOKEN,
    )
    try:
        semantic_sha256 = sha256_json(_snapshot_payload(provisional))
    except IdentityError as exc:
        raise SolverOptionsError(
            "solver-option semantic identity cannot be encoded"
        ) from exc
    return SolverOptionsSnapshot(
        **values,
        semantic_sha256=semantic_sha256,
        _construction_token=_SNAPSHOT_TOKEN,
    )


__all__ = [
    "CONTINUOUS_INTEGRATOR_OPTIONS_POLICY",
    "COST_SCALING_POLICY",
    "SOLVER_OPTIONS_ARTIFACT_STATUS",
    "SOLVER_OPTIONS_SCHEMA",
    "SOLVER_OPTIONS_SCOPE",
    "SOLVER_OPTIONS_STATUS",
    "TARGET_PERFORMANCE_STATUS",
    "TIME_STEP_POLICY",
    "SolverOptionsError",
    "SolverOptionsSnapshot",
    "build_solver_options_snapshot",
]
