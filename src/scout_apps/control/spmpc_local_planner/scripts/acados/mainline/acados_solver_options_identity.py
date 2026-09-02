"""Canonical identity for the complete Acados solver-options mapping."""

from __future__ import annotations

import math
from collections.abc import Mapping
from numbers import Integral, Real
from typing import Any

from .identity import IdentityError, require_sha256, sha256_json

ACADOS_SOLVER_OPTIONS_BASELINE_SCHEMA = (
    "spmpc_mainline_acados_solver_options_baseline_v1"
)
ACADOS_SOLVER_OPTIONS_BASELINE_SCOPE = (
    "COMPLETE_BACKEND_SOLVER_OPTIONS_EXCEPT_D4_CODEGEN_FIELDS"
)
ACADOS_D4_SOLVER_OPTION_FIELDS = (
    "ext_fun_compile_flags",
    "ext_fun_expand_constr",
    "ext_fun_expand_cost",
    "ext_fun_expand_dyn",
    "ext_fun_expand_precompute",
    "custom_update_filename",
    "custom_update_header_filename",
    "custom_update_copy",
    "custom_templates",
)


class AcadosSolverOptionsIdentityError(ValueError):
    """The backend solver-options mapping cannot form a stable identity."""


def _canonical_backend_value(value: Any, label: str) -> Any:
    if value is None or type(value) in {bool, int, float, str}:
        if type(value) is float and not math.isfinite(value):
            raise AcadosSolverOptionsIdentityError(f"{label} is non-finite")
        return 0.0 if type(value) is float and value == 0.0 else value
    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if type(key) is not str or key in result:
                raise AcadosSolverOptionsIdentityError(
                    f"{label} contains a non-canonical mapping key"
                )
            result[key] = _canonical_backend_value(item, f"{label}.{key}")
        return result
    if isinstance(value, (list, tuple)):
        return [
            _canonical_backend_value(item, f"{label}[{index}]")
            for index, item in enumerate(value)
        ]
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        converted = float(value)
        if not math.isfinite(converted):
            raise AcadosSolverOptionsIdentityError(f"{label} is non-finite")
        return 0.0 if converted == 0.0 else converted
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        try:
            converted = tolist()
        except (TypeError, ValueError) as exc:
            raise AcadosSolverOptionsIdentityError(
                f"{label} cannot be converted to JSON-compatible values"
            ) from exc
        if converted is value:
            raise AcadosSolverOptionsIdentityError(
                f"{label} returned itself while converting to a list"
            )
        return _canonical_backend_value(converted, label)
    raise AcadosSolverOptionsIdentityError(
        f"{label} has unsupported type {type(value)!r}"
    )


def acados_solver_options_baseline_payload(value: Any) -> dict[str, Any]:
    """Normalize every non-D4 field from ``AcadosOcp.to_dict()``."""

    if type(value) is not dict:
        raise AcadosSolverOptionsIdentityError(
            "Acados solver options must be an exact dictionary"
        )
    missing = [name for name in ACADOS_D4_SOLVER_OPTION_FIELDS if name not in value]
    if missing:
        raise AcadosSolverOptionsIdentityError(
            "Acados solver options are missing D4 fields: " + ", ".join(missing)
        )
    options: dict[str, Any] = {}
    for name, item in value.items():
        if type(name) is not str:
            raise AcadosSolverOptionsIdentityError(
                "Acados solver-option names must be strings"
            )
        if name not in ACADOS_D4_SOLVER_OPTION_FIELDS:
            options[name] = _canonical_backend_value(
                item,
                f"Acados solver option {name}",
            )
    if not options or "N_horizon" not in options:
        raise AcadosSolverOptionsIdentityError(
            "Acados solver-options baseline is incomplete"
        )
    return {
        "schema_version": ACADOS_SOLVER_OPTIONS_BASELINE_SCHEMA,
        "scope": ACADOS_SOLVER_OPTIONS_BASELINE_SCOPE,
        "excluded_d4_fields": list(ACADOS_D4_SOLVER_OPTION_FIELDS),
        "solver_options": options,
    }


def acados_solver_options_baseline_sha256(value: Any) -> str:
    """Hash a full backend solver-options mapping except explicit D4 fields."""

    try:
        return sha256_json(acados_solver_options_baseline_payload(value))
    except IdentityError as exc:
        raise AcadosSolverOptionsIdentityError(
            "Acados solver-options baseline cannot be encoded"
        ) from exc


def acados_ocp_solver_options_baseline_sha256(ocp: Any) -> str:
    """Extract the public solver-options mapping from an ``AcadosOcp``."""

    try:
        document = ocp.to_dict()
    except (AttributeError, TypeError, ValueError) as exc:
        raise AcadosSolverOptionsIdentityError(
            "Acados OCP cannot expose its solver-options mapping"
        ) from exc
    if type(document) is not dict or "solver_options" not in document:
        raise AcadosSolverOptionsIdentityError(
            "Acados OCP dictionary has no solver_options object"
        )
    return acados_solver_options_baseline_sha256(document["solver_options"])


def require_acados_ocp_solver_options_baseline(
    ocp: Any,
    expected_sha256: Any,
) -> str:
    """Reject any non-D4 solver-option mutation after OCP assembly."""

    try:
        expected = require_sha256(
            expected_sha256,
            "Acados solver-options baseline identity",
        )
    except IdentityError as exc:
        raise AcadosSolverOptionsIdentityError(str(exc)) from exc
    actual = acados_ocp_solver_options_baseline_sha256(ocp)
    if actual != expected:
        raise AcadosSolverOptionsIdentityError(
            "Acados solver options changed after OCP assembly"
        )
    return actual


__all__ = [
    "ACADOS_D4_SOLVER_OPTION_FIELDS",
    "ACADOS_SOLVER_OPTIONS_BASELINE_SCHEMA",
    "ACADOS_SOLVER_OPTIONS_BASELINE_SCOPE",
    "AcadosSolverOptionsIdentityError",
    "acados_ocp_solver_options_baseline_sha256",
    "acados_solver_options_baseline_payload",
    "acados_solver_options_baseline_sha256",
    "require_acados_ocp_solver_options_baseline",
]
