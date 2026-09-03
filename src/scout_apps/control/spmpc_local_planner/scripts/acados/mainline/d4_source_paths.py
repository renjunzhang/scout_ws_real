"""Single repository-source inventory for Stage 3-D4 generation."""

from __future__ import annotations

from pathlib import Path, PurePosixPath

from .identity import IdentityError, read_stable_regular_file

D4_SOURCE_PATHS = (
    "src/scout_apps/control/spmpc_local_planner/config/mainline/contracts/development_capacity_v1.json",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/generate_mainline_d4.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/__init__.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_backend.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_codegen_backend.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_codegen_result_schema.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_codegen_validation.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_ocp_adapter.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_ocp_contract.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_ocp_schema.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_ocp_validation.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_solver_options_adapter.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/acados_solver_options_identity.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/artifact_contract.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/artifact_contract_schema.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/artifact_files.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/artifact_publication.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/casadi_adapter.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/casadi_dynamics.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/casadi_graph_contract.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/casadi_graph_schema.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/casadi_objective.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/casadi_reference.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/codegen_options.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/constraints_oracle.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/d4_generator.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/d4_source_paths.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/development_capacity.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/development_layout.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/discrete_dynamics.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/identity.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/model_contract.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/model_contract_header.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/provenance.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/provenance_acados.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/provenance_common.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/provenance_files.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/provenance_git.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/provenance_python.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/provenance_schema.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/runtime_schedule.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/solver_options.py",
    "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/solver_parameter_layout.py",
)


class D4SourcePathsError(ValueError):
    """The D4 source inventory or its repository root is unsafe."""


def _require_inventory() -> tuple[str, ...]:
    if (
        type(D4_SOURCE_PATHS) is not tuple
        or any(type(item) is not str or not item for item in D4_SOURCE_PATHS)
        or D4_SOURCE_PATHS != tuple(sorted(D4_SOURCE_PATHS))
        or len(D4_SOURCE_PATHS) != len(set(D4_SOURCE_PATHS))
    ):
        raise D4SourcePathsError("D4 source paths must be an immutable sorted set")
    for item in D4_SOURCE_PATHS:
        path = PurePosixPath(item)
        if (
            path.is_absolute()
            or path.as_posix() != item
            or any(part in {"", ".", ".."} for part in path.parts)
        ):
            raise D4SourcePathsError(f"D4 source path is not canonical: {item}")
    return D4_SOURCE_PATHS


def require_d4_source_paths(repository_root: Path | str) -> tuple[str, ...]:
    """Validate and stably read every source before provenance capture."""

    if not isinstance(repository_root, (str, Path)):
        raise D4SourcePathsError("repository root must be str or Path")
    root = Path(repository_root)
    if not root.is_absolute() or ".." in root.parts:
        raise D4SourcePathsError("repository root must be a canonical absolute path")
    try:
        resolved = root.resolve(strict=True)
    except OSError as exc:
        raise D4SourcePathsError(f"repository root cannot be resolved: {exc}") from exc
    if resolved != root or not resolved.is_dir():
        raise D4SourcePathsError(
            "repository root must be a real directory without symbolic links"
        )
    inventory = _require_inventory()
    try:
        for relative in inventory:
            read_stable_regular_file(
                root.joinpath(*PurePosixPath(relative).parts),
                label=f"D4 repository source {relative}",
            )
    except IdentityError as exc:
        raise D4SourcePathsError(str(exc)) from exc
    return inventory


__all__ = [
    "D4_SOURCE_PATHS",
    "D4SourcePathsError",
    "require_d4_source_paths",
]
