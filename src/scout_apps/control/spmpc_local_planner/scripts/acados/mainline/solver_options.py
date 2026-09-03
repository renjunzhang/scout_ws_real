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
from .identity import IdentityError, require_sha256, sha256_json, strict_json_equal

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
        _validate_solver_options_structure(self)

    def to_dict(self) -> dict[str, Any]:
        """Return the canonical JSON-compatible option snapshot."""

        payload = _snapshot_payload(self)
        payload["semantic_identity"] = {
            "sha256": self.semantic_sha256,
            "scope": SOLVER_OPTIONS_SCOPE,
        }
        return payload


def _validate_solver_options_structure(snapshot: SolverOptionsSnapshot) -> None:
    if snapshot.schema_version != SOLVER_OPTIONS_SCHEMA:
        raise SolverOptionsError("solver-option schema is not canonical")
    if (
        snapshot.status != SOLVER_OPTIONS_STATUS
        or snapshot.target_performance_status != TARGET_PERFORMANCE_STATUS
        or snapshot.artifact_status != SOLVER_OPTIONS_ARTIFACT_STATUS
    ):
        raise SolverOptionsError("solver-option status is not canonical")
    if type(snapshot.semantic_sha256) is not str or len(snapshot.semantic_sha256) != 64:
        raise SolverOptionsError("solver-option semantic identity is invalid")
    if (
        type(snapshot.time_steps) is not tuple
        or len(snapshot.time_steps) != snapshot.horizon_steps
        or any(
            type(value) is not Fraction or value != snapshot.time_step_sec
            for value in snapshot.time_steps
        )
    ):
        raise SolverOptionsError(
            "solver-option time steps must be the uniform typed release grid"
        )
    if (
        type(snapshot.cost_scaling) is not tuple
        or len(snapshot.cost_scaling) != snapshot.horizon_steps + 1
        or any(
            type(value) is not float or value != 1.0 for value in snapshot.cost_scaling
        )
    ):
        raise SolverOptionsError(
            "solver-option cost scaling must contain explicit 1.0 floats"
        )


def _canonical_solver_options_payload() -> dict[str, Any]:
    """Return the complete, serialized D4 solver-options policy.

    This is deliberately independent of ``SolverOptionsSnapshot`` so an
    offline artifact parser can validate the same policy without constructing
    a typed authority or importing a numerical backend.
    """

    return {
        "schema_version": SOLVER_OPTIONS_SCHEMA,
        "scope": SOLVER_OPTIONS_SCOPE,
        "status": {
            "solver_options": SOLVER_OPTIONS_STATUS,
            "target_performance": TARGET_PERFORMANCE_STATUS,
            "artifact": SOLVER_OPTIONS_ARTIFACT_STATUS,
        },
        "horizon": {
            "N": 60,
            "release_frequency_hz": 30,
            "time_step_sec": {"numerator": 1, "denominator": 30},
            "time_horizon_sec": {"numerator": 2, "denominator": 1},
            "time_steps": {
                "policy": TIME_STEP_POLICY,
                "count": 60,
                "uniform_value_sec": {"numerator": 1, "denominator": 30},
            },
            "cost_scaling": {
                "policy": COST_SCALING_POLICY,
                "count": 61,
                "uniform_value": 1.0,
            },
        },
        "discretization": {
            "integrator_type": "DISCRETE",
            "cost_discretization": "EULER",
            "continuous_integrator_options": (CONTINUOUS_INTEGRATOR_OPTIONS_POLICY),
        },
        "nlp": {
            "solver_type": "SQP_RTI",
            "max_iter": 1,
            "tolerances": {
                "stat": 1.0e-6,
                "eq": 1.0e-6,
                "ineq": 1.0e-6,
                "comp": 1.0e-6,
            },
            "warm_start_first_qp": False,
            "warm_start_first_qp_from_nlp": False,
        },
        "hessian": {
            "approximation": "EXACT",
            "exact_dyn": True,
            "exact_cost": True,
            "exact_constr": True,
            "external_cost_numerical_hessian": 0,
        },
        "regularization": {
            "method": "PROJECT",
            "epsilon": 1.0e-4,
            "levenberg_marquardt": 1.0e-3,
        },
        "globalization": {
            "method": "FIXED_STEP",
            "fixed_step_length": 1.0,
        },
        "qp": {
            "solver": "PARTIAL_CONDENSING_HPIPM",
            "condensed_horizon": 60,
            "condensing_riccati_algorithm": 1,
            "riccati_algorithm": 1,
            "max_iter": 50,
            "tolerances": {
                "stat": 1.0e-6,
                "eq": 1.0e-6,
                "ineq": 1.0e-6,
                "comp": 1.0e-6,
            },
            "warm_start_level": 0,
            "hpipm_mode": "BALANCE",
        },
        "diagnostics": {"print_level": 0},
    }


def _validate_solver_options_payload(payload: Any) -> dict[str, Any]:
    """Validate a serialized payload excluding ``semantic_identity``."""

    if type(payload) is not dict:
        raise SolverOptionsError("solver-option payload must be a JSON object")
    expected = _canonical_solver_options_payload()
    if not strict_json_equal(payload, expected):
        raise SolverOptionsError("solver-option payload is not the canonical D4 policy")
    return payload


def validate_solver_options_document(value: Any) -> dict[str, Any]:
    """Validate a complete dependency-free serialized solver-options document.

    The returned mapping is the original validated document.  Its semantic
    identity covers every policy field, making this function suitable for an
    artifact parser that cannot construct ``SolverOptionsSnapshot``.
    """

    if type(value) is not dict:
        raise SolverOptionsError("solver-options document must be a JSON object")
    expected_keys = {
        "schema_version",
        "scope",
        "status",
        "horizon",
        "discretization",
        "nlp",
        "hessian",
        "regularization",
        "globalization",
        "qp",
        "diagnostics",
        "semantic_identity",
    }
    if set(value) != expected_keys:
        raise SolverOptionsError(
            "solver-options document keys do not match the v1 schema"
        )
    identity = value["semantic_identity"]
    if type(identity) is not dict or set(identity) != {"sha256", "scope"}:
        raise SolverOptionsError("solver-options semantic identity is malformed")
    if identity["scope"] != SOLVER_OPTIONS_SCOPE:
        raise SolverOptionsError("solver-options semantic scope drifted")
    try:
        require_sha256(identity["sha256"], "solver-option semantic identity")
        payload = {
            key: item for key, item in value.items() if key != "semantic_identity"
        }
        _validate_solver_options_payload(payload)
        if sha256_json(payload) != identity["sha256"]:
            raise SolverOptionsError("solver-option semantic identity is inconsistent")
    except SolverOptionsError:
        raise
    except (IdentityError, KeyError, TypeError, ValueError) as exc:
        raise SolverOptionsError("solver-options document is malformed") from exc
    return value


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


def require_solver_options_snapshot(value: Any) -> SolverOptionsSnapshot:
    """Reject wrong-type or force-mutated numerical options."""

    if type(value) is not SolverOptionsSnapshot:
        raise SolverOptionsError(
            "solver_options must be the exact SolverOptionsSnapshot type"
        )
    try:
        _validate_solver_options_structure(value)
        _validate_solver_options_payload(_snapshot_payload(value))
        semantic_sha256 = sha256_json(_snapshot_payload(value))
    except SolverOptionsError:
        raise
    except (AttributeError, IdentityError, TypeError, ValueError) as exc:
        raise SolverOptionsError("solver-option snapshot is malformed") from exc
    if semantic_sha256 != value.semantic_sha256:
        raise SolverOptionsError("solver-option semantic identity is inconsistent")
    return value


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
    "require_solver_options_snapshot",
    "validate_solver_options_document",
]
