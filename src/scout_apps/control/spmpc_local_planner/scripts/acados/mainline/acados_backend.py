"""Lazy backend loading and reproducible backend/expression identities."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .acados_ocp_schema import (
    ACADOS_INTERFACE_SOURCE_PATHS,
    DYNAMICS_IDENTITY_FUNCTION,
    STAGE_COST_IDENTITY_FUNCTION,
    STAGE_H_IDENTITY_FUNCTION,
    TERMINAL_COST_IDENTITY_FUNCTION,
    TERMINAL_H_IDENTITY_FUNCTION,
    acados_interface_source_sha256_from_inventory,
)
from .casadi_graph_contract import CasadiGraphBundle
from .identity import IdentityError, read_stable_regular_file, sha256_bytes


class AcadosOcpAdapterError(RuntimeError):
    """Base class for Stage 3-D in-memory OCP assembly failures."""


class AcadosOcpDependencyError(AcadosOcpAdapterError):
    """CasADi, NumPy, or acados-template cannot be loaded."""


class AcadosOcpConstructionError(AcadosOcpAdapterError):
    """Typed inputs or the assembled backend OCP are inconsistent."""


@dataclass(frozen=True)
class AcadosBackend:
    """Loaded backend modules and constructors; no solver is instantiated."""

    ca: Any
    np: Any
    template_module: Any
    model_type: Any
    ocp_type: Any


def require_acados_backend() -> AcadosBackend:
    """Load only the model/OCP description API, never ``AcadosOcpSolver``."""

    try:
        import acados_template
        import casadi as ca
        import numpy as np

        model_type = acados_template.AcadosModel
        ocp_type = acados_template.AcadosOcp
    except (AttributeError, ImportError, OSError) as exc:
        raise AcadosOcpDependencyError(
            "CasADi, NumPy, and a complete acados_template interface are required "
            "for OCP assembly"
        ) from exc
    return AcadosBackend(
        ca,
        np,
        acados_template,
        model_type,
        ocp_type,
    )


def read_acados_backend_identity(
    backend: AcadosBackend,
    ocp: Any,
) -> tuple[str, str]:
    """Hash the selected Python interface and compare its source root to lib."""

    module_file = getattr(backend.template_module, "__file__", None)
    if type(module_file) is not str:
        raise AcadosOcpConstructionError(
            "acados_template source location is unavailable"
        )
    package_directory = Path(module_file).resolve().parent
    source_inventory = {}
    try:
        for name in ACADOS_INTERFACE_SOURCE_PATHS:
            source_inventory[name] = sha256_bytes(
                read_stable_regular_file(
                    package_directory / name,
                    label=f"Acados interface source {name}",
                )
            )
        interface_sha256 = acados_interface_source_sha256_from_inventory(
            source_inventory
        )
        interface_source_root = package_directory.parents[2]
        library_commit_file = (
            Path(ocp.code_gen_opts.acados_lib_path) / "git_commit_hash"
        ).resolve()
        library_source_root = library_commit_file.parent.parent
    except (IdentityError, IndexError, OSError) as exc:
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
    backend: AcadosBackend,
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


def build_symbolic_expression_identity(
    backend: AcadosBackend,
    graph: CasadiGraphBundle,
    stage_h: Any,
) -> dict[str, str]:
    """Hash all symbolic expressions that are connected to the Acados OCP."""

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


__all__ = [
    "AcadosBackend",
    "AcadosOcpAdapterError",
    "AcadosOcpConstructionError",
    "AcadosOcpDependencyError",
    "build_symbolic_expression_identity",
    "read_acados_backend_identity",
    "require_acados_backend",
]
