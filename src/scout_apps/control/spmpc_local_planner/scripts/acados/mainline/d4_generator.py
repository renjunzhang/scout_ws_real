"""Top-level Stage 3-D4 development artifact generation and publication."""

from __future__ import annotations

import math
from dataclasses import InitVar, dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from .acados_backend import explicit_acados_environment
from .acados_codegen_backend import generate_and_build_acados
from .acados_ocp_adapter import assemble_acados_ocp
from .artifact_contract import (
    ArtifactContract,
    build_artifact_contract,
    require_artifact_contract,
)
from .artifact_files import require_artifact_directory
from .artifact_publication import (
    ArtifactPublicationCommittedError,
    create_sibling_staging_directory,
    load_artifact_contract_directory,
    publish_staging_directory,
    require_unpublished_artifact_target,
    write_artifact_contract_outputs,
)
from .casadi_adapter import build_casadi_graph
from .codegen_options import build_codegen_options_snapshot
from .constraints_oracle import ConstraintBounds
from .d4_source_paths import require_d4_source_paths
from .development_capacity import load_development_capacity
from .development_layout import build_development_layout
from .identity import strict_json_equal
from .provenance import (
    CodegenProvenance,
    capture_codegen_provenance,
    require_codegen_provenance,
    require_codegen_provenance_current,
)
from .solver_options import build_solver_options_snapshot
from .solver_parameter_layout import build_solver_parameter_layout

CAPACITY_CONTRACT_RELATIVE_PATH = (
    "src/scout_apps/control/spmpc_local_planner/config/mainline/contracts/"
    "development_capacity_v1.json"
)

_RESULT_TOKEN = object()


class D4GenerationError(RuntimeError):
    """Stage 3-D4 preflight, generation, validation, or publication failed."""


@dataclass(frozen=True)
class D4GenerationRequest:
    """All caller-controlled D4 inputs; no production parameter is implied."""

    repository_root: Path
    output_directory: Path
    acados_install_root: Path
    tera_executable: Path
    constraint_bounds: ConstraintBounds
    integer_snap_tolerance_sec: float
    duration_tolerance_sec: float
    ext_fun_compile_flags: str


@dataclass(frozen=True)
class D4GenerationResult:
    """Published development artifact and the exact provenance embedded in it."""

    output_directory: Path
    contract: ArtifactContract
    provenance: CodegenProvenance
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _RESULT_TOKEN:
            raise D4GenerationError(
                "D4GenerationResult requires generate_d4_development_artifact"
            )

    def to_dict(self) -> dict[str, Any]:
        checked = require_d4_generation_result(self)
        return {
            "status": "PUBLISHED_DEV_UNVALIDATED",
            "output_directory": str(checked.output_directory),
            "model_contract_semantic_sha256": checked.contract.semantic_sha256,
            "artifact_sha256": checked.contract.artifact_sha256,
            "provenance_semantic_sha256": checked.provenance.semantic_sha256,
        }


def _require_request(value: Any) -> D4GenerationRequest:
    if type(value) is not D4GenerationRequest:
        raise D4GenerationError("request must be the exact D4GenerationRequest type")
    for name in (
        "repository_root",
        "output_directory",
        "acados_install_root",
        "tera_executable",
    ):
        path = getattr(value, name)
        if not isinstance(path, Path) or not path.is_absolute() or ".." in path.parts:
            raise D4GenerationError(f"{name} must be a canonical absolute Path")
    if type(value.constraint_bounds) is not ConstraintBounds:
        raise D4GenerationError(
            "constraint_bounds must be the exact ConstraintBounds type"
        )
    for name in ("integer_snap_tolerance_sec", "duration_tolerance_sec"):
        tolerance = getattr(value, name)
        if type(tolerance) is not float or not math.isfinite(tolerance):
            raise D4GenerationError(f"{name} must be an explicit finite float")
    if type(value.ext_fun_compile_flags) is not str:
        raise D4GenerationError("ext_fun_compile_flags must be an explicit string")
    return value


def _capacity_path(repository_root: Path) -> Path:
    return repository_root.joinpath(
        *PurePosixPath(CAPACITY_CONTRACT_RELATIVE_PATH).parts
    )


def require_d4_generation_result(value: Any) -> D4GenerationResult:
    """Reload the published directory and reject a force-mutated result."""

    if type(value) is not D4GenerationResult:
        raise D4GenerationError("result must be the exact D4GenerationResult type")
    try:
        root = require_artifact_directory(value.output_directory)
        contract = require_artifact_contract(value.contract)
        provenance = require_codegen_provenance(value.provenance)
        reloaded = load_artifact_contract_directory(root)
    except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
        raise D4GenerationError("published D4 result is malformed") from exc
    if root != value.output_directory or not strict_json_equal(
        reloaded.to_dict(), contract.to_dict()
    ):
        raise D4GenerationError("published D4 contract identity drifted")
    embedded = contract.to_dict()["typed_authorities"]["provenance"]
    if not strict_json_equal(embedded, provenance.to_dict()):
        raise D4GenerationError("published D4 provenance identity drifted")
    return value


def generate_d4_development_artifact(
    request: D4GenerationRequest,
) -> D4GenerationResult:
    """Build D0-D3 fresh, generate once, then atomically publish one D4 artifact."""

    checked = _require_request(request)
    try:
        repository_root = require_artifact_directory(checked.repository_root)
        target = require_unpublished_artifact_target(checked.output_directory)
        source_paths = require_d4_source_paths(repository_root)
        capacity = load_development_capacity(_capacity_path(repository_root))
        layout = build_development_layout(capacity)
        parameter_layout = build_solver_parameter_layout(capacity, layout)
        solver_options = build_solver_options_snapshot(layout)
        codegen_options = build_codegen_options_snapshot(
            capacity,
            layout,
            checked.integer_snap_tolerance_sec,
            checked.duration_tolerance_sec,
            checked.ext_fun_compile_flags,
        )
        with explicit_acados_environment(
            checked.acados_install_root,
            checked.tera_executable,
        ):
            provenance = capture_codegen_provenance(
                repository_root,
                source_paths,
                codegen_options.compiler_environment,
                checked.acados_install_root,
                checked.tera_executable,
            )
            graph = build_casadi_graph(
                capacity,
                layout,
                checked.constraint_bounds,
            )
            assembly = assemble_acados_ocp(graph, solver_options)
    except D4GenerationError:
        raise
    except Exception as exc:
        raise D4GenerationError(
            "D4 preflight, provenance capture, or fresh D0-D3 assembly failed"
        ) from exc

    try:
        staging = create_sibling_staging_directory(target)
    except Exception as exc:
        raise D4GenerationError(
            "D4 sibling staging directory could not be created"
        ) from exc
    try:
        codegen_result = generate_and_build_acados(
            graph,
            assembly,
            solver_options,
            codegen_options,
            staging,
            checked.acados_install_root,
            checked.tera_executable,
        )
        require_codegen_provenance_current(provenance)
        contract = build_artifact_contract(
            capacity,
            layout,
            parameter_layout,
            graph,
            assembly,
            solver_options,
            codegen_options,
            provenance,
            codegen_result,
        )
        written_contract = write_artifact_contract_outputs(staging, contract)
        if not strict_json_equal(written_contract.to_dict(), contract.to_dict()):
            raise D4GenerationError("staged D4 contract identity drifted")
        load_artifact_contract_directory(staging)
        require_codegen_provenance_current(provenance)
        published = publish_staging_directory(staging, target)
    except ArtifactPublicationCommittedError as exc:
        raise D4GenerationError(
            f"D4 artifact reached its final target but post-commit verification "
            f"failed: {exc.published_target}"
        ) from exc
    except Exception as exc:
        raise D4GenerationError(
            f"D4 generation failed; partial staging retained at {staging}"
        ) from exc

    try:
        published_contract = load_artifact_contract_directory(published)
        if not strict_json_equal(published_contract.to_dict(), contract.to_dict()):
            raise D4GenerationError("published contract differs from staged contract")
        result = D4GenerationResult(
            published,
            published_contract,
            provenance,
            _construction_token=_RESULT_TOKEN,
        )
        return require_d4_generation_result(result)
    except D4GenerationError:
        raise
    except Exception as exc:
        raise D4GenerationError(
            f"D4 directory was published but failed final validation: {published}"
        ) from exc


__all__ = [
    "CAPACITY_CONTRACT_RELATIVE_PATH",
    "D4GenerationError",
    "D4GenerationRequest",
    "D4GenerationResult",
    "generate_d4_development_artifact",
    "require_d4_generation_result",
]
