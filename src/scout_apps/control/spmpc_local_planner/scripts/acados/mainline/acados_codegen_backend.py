"""Narrow lazy boundary for Acados generation and a checked fixed-make build."""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
import sys
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import InitVar, dataclass
from pathlib import Path
from typing import Any

from .acados_backend import require_acados_backend
from .acados_codegen_result_schema import (
    ACADOS_CODEGEN_ARTIFACT_CLASS,
    ACADOS_CODEGEN_PERFORMANCE_STATUS,
    ACADOS_CODEGEN_PROMOTION_STATUS,
    ACADOS_CODEGEN_RESULT_SCHEMA,
    ACADOS_CODEGEN_RESULT_SCOPE,
    ACADOS_CODEGEN_STATUS,
    CODEGEN_FAILURE_OUTPUT_POLICY,
    ELF_MACHINE_BY_PLATFORM,
    OUTPUT_ROOT_IDENTITY_POLICY,
    REQUIRED_SOLVER_SYMBOL_SUFFIXES,
    REQUIRED_SOLVER_SYMBOLS,
    SOLVER_LIBRARY_FORMAT,
    SOLVER_LOAD_CHECK_POLICY,
    SUPPORTED_ELF_IDENTITIES,
    validate_acados_codegen_result_document,
)
from .acados_codegen_validation import (
    AcadosCodegenValidationError,
    canonicalize_generated_output_root,
    validate_generated_acados_json,
    validate_no_embedded_output_root,
    validate_solver_shared_library,
)
from .acados_ocp_contract import AcadosOcpAssembly, require_acados_ocp_assembly
from .acados_ocp_validation import validate_consistent_ocp
from .acados_solver_options_adapter import validate_applied_solver_options
from .acados_solver_options_identity import (
    require_acados_ocp_solver_options_baseline,
)
from .artifact_files import (
    GeneratedFileRecord,
    generated_file_records_from_dict,
    generated_tree_sha256,
    inventory_codegen_tree,
    inventory_generated_tree,
    prepare_empty_codegen_directory,
    solver_library_record,
    validate_generated_tree,
)
from .casadi_graph_contract import CasadiGraphBundle
from .codegen_options import (
    ACADOS_JSON_FILENAME,
    CodegenOptionsSnapshot,
    apply_acados_solver_codegen_options,
    require_codegen_compiler_environment,
    require_codegen_options_snapshot,
    validate_applied_acados_solver_codegen_options,
)
from .identity import IdentityError, require_sha256, sha256_json
from .model_contract import MODEL_ID
from .solver_options import SolverOptionsSnapshot, require_solver_options_snapshot

MAKE_CLEAN_TARGET = "clean_ocp_shared_lib"
MAKE_BUILD_TARGET = "ocp_shared_lib"

_RESULT_TOKEN = object()


class AcadosCodegenError(RuntimeError):
    """Typed input, generation, compilation, or validation failed."""


class AcadosCodegenDependencyError(AcadosCodegenError):
    """The solver-generation backend or a required host tool is unavailable."""


@dataclass(frozen=True)
class AcadosCodegenResult:
    """Validated generated tree; absolute staging location is non-semantic."""

    output_directory: Path
    files: tuple[GeneratedFileRecord, ...]
    generated_tree_sha256: str
    solver_library_relative_path: str
    solver_library_raw_sha256: str
    solver_library_size_bytes: int
    elf_class: int
    elf_machine: int
    required_exported_symbols: tuple[str, ...]
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = ACADOS_CODEGEN_RESULT_SCHEMA
    model_id: str = MODEL_ID
    status: str = ACADOS_CODEGEN_STATUS
    artifact_class: str = ACADOS_CODEGEN_ARTIFACT_CLASS
    promotion_status: str = ACADOS_CODEGEN_PROMOTION_STATUS
    target_performance_status: str = ACADOS_CODEGEN_PERFORMANCE_STATUS

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _RESULT_TOKEN:
            raise AcadosCodegenError(
                "AcadosCodegenResult requires generate_and_build_acados"
            )
        _validate_codegen_result_structure(self)

    def to_dict(self) -> dict[str, Any]:
        payload = _result_payload(self)
        payload["semantic_identity"] = {
            "sha256": self.semantic_sha256,
            "scope": ACADOS_CODEGEN_RESULT_SCOPE,
        }
        return payload


def _validate_codegen_result_structure(result: AcadosCodegenResult) -> None:
    if (
        result.schema_version != ACADOS_CODEGEN_RESULT_SCHEMA
        or result.model_id != MODEL_ID
        or result.status != ACADOS_CODEGEN_STATUS
        or result.artifact_class != ACADOS_CODEGEN_ARTIFACT_CLASS
        or result.promotion_status != ACADOS_CODEGEN_PROMOTION_STATUS
        or result.target_performance_status != ACADOS_CODEGEN_PERFORMANCE_STATUS
    ):
        raise AcadosCodegenError("Acados codegen result status drifted")
    if not isinstance(result.output_directory, Path) or not (
        result.output_directory.is_absolute()
    ):
        raise AcadosCodegenError("codegen result directory must be absolute")
    try:
        resolved_output = result.output_directory.resolve(strict=True)
    except OSError as exc:
        raise AcadosCodegenError("codegen result directory cannot be resolved") from exc
    if resolved_output != result.output_directory or not resolved_output.is_dir():
        raise AcadosCodegenError(
            "codegen result directory must be an existing canonical directory"
        )
    if type(result.files) is not tuple:
        raise AcadosCodegenError("codegen result files must be an immutable tuple")
    try:
        checked_files = generated_file_records_from_dict(
            [item.to_dict() for item in result.files]
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise AcadosCodegenError("codegen result file inventory is malformed") from exc
    if checked_files != result.files:
        raise AcadosCodegenError("codegen result file inventory drifted")
    if generated_tree_sha256(checked_files) != result.generated_tree_sha256:
        raise AcadosCodegenError("generated tree identity is inconsistent")
    try:
        validate_generated_tree(result.output_directory, checked_files)
    except (AttributeError, TypeError, ValueError) as exc:
        raise AcadosCodegenError(
            "codegen result directory differs from its recorded inventory"
        ) from exc
    library = solver_library_record(checked_files)
    if (
        result.solver_library_relative_path != library.relative_path
        or result.solver_library_raw_sha256 != library.raw_sha256
        or result.solver_library_size_bytes != library.size_bytes
    ):
        raise AcadosCodegenError("solver library identity is inconsistent")
    if (
        type(result.elf_class) is not int
        or type(result.elf_machine) is not int
        or (result.elf_class, result.elf_machine) not in SUPPORTED_ELF_IDENTITIES
    ):
        raise AcadosCodegenError("solver ELF identity is invalid")
    if result.required_exported_symbols != REQUIRED_SOLVER_SYMBOLS:
        raise AcadosCodegenError("solver exported-symbol set is inconsistent")
    try:
        require_sha256(result.generated_tree_sha256, "generated tree identity")
        require_sha256(result.solver_library_raw_sha256, "solver library identity")
        require_sha256(result.semantic_sha256, "codegen result identity")
    except IdentityError as exc:
        raise AcadosCodegenError(str(exc)) from exc


def _result_payload(result: AcadosCodegenResult) -> dict[str, Any]:
    return {
        "schema_version": result.schema_version,
        "scope": ACADOS_CODEGEN_RESULT_SCOPE,
        "model_id": result.model_id,
        "status": {
            "codegen": result.status,
            "artifact_class": result.artifact_class,
            "promotion": result.promotion_status,
            "target_performance": result.target_performance_status,
        },
        "output_directory": OUTPUT_ROOT_IDENTITY_POLICY,
        "failure_output_policy": CODEGEN_FAILURE_OUTPUT_POLICY,
        "generated_tree": {
            "sha256": result.generated_tree_sha256,
            "files": [item.to_dict() for item in result.files],
        },
        "solver_library": {
            "relative_path": result.solver_library_relative_path,
            "size_bytes": result.solver_library_size_bytes,
            "raw_sha256": result.solver_library_raw_sha256,
            "format": SOLVER_LIBRARY_FORMAT,
            "elf_class": result.elf_class,
            "elf_machine": result.elf_machine,
            "required_exported_symbols": list(result.required_exported_symbols),
            "load_check": SOLVER_LOAD_CHECK_POLICY,
        },
    }


def require_acados_codegen_result(value: Any) -> AcadosCodegenResult:
    if type(value) is not AcadosCodegenResult:
        raise AcadosCodegenError(
            "codegen_result must be the exact AcadosCodegenResult type"
        )
    try:
        _validate_codegen_result_structure(value)
        document = validate_acados_codegen_result_document(value.to_dict())
        semantic_sha256 = document["semantic_identity"]["sha256"]
    except AcadosCodegenError:
        raise
    except (AttributeError, IdentityError, TypeError, ValueError) as exc:
        raise AcadosCodegenError("Acados codegen result is malformed") from exc
    if semantic_sha256 != value.semantic_sha256:
        raise AcadosCodegenError("Acados codegen result identity is inconsistent")
    return value


def _safe_existing_path(
    value: Path | str,
    label: str,
    *,
    directory: bool,
    executable: bool = False,
) -> Path:
    if not isinstance(value, (str, Path)):
        raise AcadosCodegenError(f"{label} must be str or Path")
    candidate = Path(value)
    if not candidate.is_absolute() or ".." in candidate.parts:
        raise AcadosCodegenError(f"{label} must be an absolute canonical path")
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise AcadosCodegenError(f"{label} cannot be resolved: {exc}") from exc
    if resolved != candidate:
        raise AcadosCodegenError(f"{label} cannot contain symbolic-link components")
    if directory and not resolved.is_dir():
        raise AcadosCodegenError(f"{label} must be a directory")
    if not directory and (resolved.is_symlink() or not resolved.is_file()):
        raise AcadosCodegenError(f"{label} must be a regular file")
    if executable and not os.access(resolved, os.X_OK):
        raise AcadosCodegenError(f"{label} must be executable")
    return resolved


def _require_tool_binding(name: str, expected: Path) -> None:
    """Require a PATH lookup to keep resolving to the preflight executable."""

    current = shutil.which(name)
    if current is None:
        raise AcadosCodegenDependencyError(f"required build tool disappeared: {name}")
    try:
        resolved = _safe_existing_path(
            Path(current).resolve(strict=True),
            name,
            directory=False,
            executable=True,
        )
    except OSError as exc:
        raise AcadosCodegenDependencyError(
            f"required build tool cannot be resolved: {name}"
        ) from exc
    if resolved != expected:
        raise AcadosCodegenDependencyError(
            f"required build tool changed after preflight: {name}"
        )


def _build_with_fixed_make(
    make_path: Path,
    output_root: Path,
    *,
    verbose: bool,
) -> None:
    """Build with one absolute make executable and checked return codes.

    Acados 0.5.4 delegates these targets through unchecked ``subprocess.call``.
    The generated alias target also has no same-named file, so ``make -q``
    cannot prove it current. Execute both targets directly and fail on either
    nonzero status instead.
    """

    _require_tool_binding("make", make_path)
    for target in (MAKE_CLEAN_TARGET, MAKE_BUILD_TARGET):
        try:
            subprocess.run(
                [str(make_path), "--no-print-directory", target],
                cwd=output_root,
                check=True,
                capture_output=not verbose,
                text=True,
            )
        except OSError as exc:
            raise AcadosCodegenDependencyError(
                f"cannot execute the generated Acados make target: {target}"
            ) from exc
        except subprocess.CalledProcessError as exc:
            raise AcadosCodegenError(
                f"generated Acados make target failed: {target}"
            ) from exc


def _checked_authorities(
    graph: Any,
    assembly: Any,
    solver_options: Any,
    codegen_options: Any,
) -> tuple[
    CasadiGraphBundle,
    AcadosOcpAssembly,
    SolverOptionsSnapshot,
    CodegenOptionsSnapshot,
]:
    if type(graph) is not CasadiGraphBundle:
        raise AcadosCodegenError("graph must be the exact CasadiGraphBundle type")
    try:
        checked_assembly = require_acados_ocp_assembly(assembly)
        checked_solver = require_solver_options_snapshot(solver_options)
        checked_codegen = require_codegen_options_snapshot(codegen_options)
    except (TypeError, ValueError) as exc:
        raise AcadosCodegenError("codegen authorities are not canonical") from exc
    if (
        checked_assembly.graph_semantic_sha256 != graph.graph_semantic_sha256
        or checked_assembly.bounds_snapshot_sha256
        != sha256_json(graph.bounds.to_dict())
        or checked_assembly.solver_options_semantic_sha256
        != checked_solver.semantic_sha256
        or checked_assembly.capacity_contract_sha256 != graph.capacity_contract_sha256
        or checked_assembly.development_layout_sha256 != graph.development_layout_sha256
        or checked_assembly.solver_parameter_layout_sha256
        != graph.solver_parameter_layout_sha256
        or checked_codegen.development_layout_sha256 != graph.development_layout_sha256
        or (
            checked_assembly.horizon_steps,
            checked_assembly.nx,
            checked_assembly.nu,
            checked_assembly.np,
        )
        != (graph.horizon_steps, graph.nx, graph.nu, graph.np)
        or checked_codegen.horizon_steps != graph.horizon_steps
        or checked_solver.horizon_steps != graph.horizon_steps
    ):
        raise AcadosCodegenError("graph/OCP/options authorities are inconsistent")
    if checked_assembly.acados_backend_binding_status != "MATCHED_SOURCE_ROOT":
        raise AcadosCodegenError("Acados Python interface and library roots differ")
    return graph, checked_assembly, checked_solver, checked_codegen


def _require_codegen_backend(backend: Any) -> Any:
    solver_type = getattr(backend.template_module, "AcadosOcpSolver", None)
    if solver_type is None:
        raise AcadosCodegenDependencyError(
            "acados_template.AcadosOcpSolver is unavailable"
        )
    if not callable(getattr(solver_type, "generate", None)):
        raise AcadosCodegenDependencyError("AcadosOcpSolver.generate is unavailable")
    return solver_type


def _bind_acados_source_root(ocp: Any, source_root: Path, assembly: Any) -> None:
    codegen = ocp.code_gen_opts
    expected_include = source_root / "include"
    expected_lib = source_root / "lib"
    try:
        actual_include = Path(codegen.acados_include_path).resolve(strict=True)
        actual_lib = Path(codegen.acados_lib_path).resolve(strict=True)
    except (AttributeError, OSError, TypeError, ValueError) as exc:
        raise AcadosCodegenError(
            "Acados include/library paths are unavailable"
        ) from exc
    if actual_include != expected_include or actual_lib != expected_lib:
        raise AcadosCodegenError(
            "explicit Acados source root differs from the assembled OCP"
        )
    if str(codegen.acados_version or "unknown") != assembly.acados_git_commit:
        raise AcadosCodegenError("Acados commit changed after OCP assembly")
    codegen.acados_include_path = str(expected_include)
    codegen.acados_lib_path = str(expected_lib)


@contextmanager
def _codegen_environment(source_root: Path, tera_path: Path) -> Iterator[None]:
    saved = {name: os.environ.get(name) for name in ("ACADOS_SOURCE_DIR", "TERA_PATH")}
    os.environ["ACADOS_SOURCE_DIR"] = str(source_root)
    os.environ["TERA_PATH"] = str(tera_path)
    try:
        yield
    finally:
        for name, value in saved.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def _validate_solver_load(
    source_root: Path,
    output_root: Path,
    library: GeneratedFileRecord,
) -> None:
    loader = r"""
import ctypes
import sys
from pathlib import Path
libdir = Path(sys.argv[1])
solver = Path(sys.argv[2])
for name in ("libblasfeo.so", "libhpipm.so", "libacados.so"):
    ctypes.CDLL(str(libdir / name), mode=ctypes.RTLD_GLOBAL)
handle = ctypes.CDLL(str(solver), mode=ctypes.RTLD_LOCAL)
for symbol in sys.argv[3:]:
    getattr(handle, symbol)
"""
    required = [f"{MODEL_ID}_{suffix}" for suffix in REQUIRED_SOLVER_SYMBOL_SUFFIXES]
    library_path = output_root / library.relative_path
    environment = dict(os.environ)
    previous = environment.get("LD_LIBRARY_PATH", "")
    environment["LD_LIBRARY_PATH"] = str(source_root / "lib") + (
        os.pathsep + previous if previous else ""
    )
    try:
        subprocess.run(
            [
                sys.executable,
                "-I",
                "-c",
                loader,
                str(source_root / "lib"),
                str(library_path),
                *required,
            ],
            check=True,
            capture_output=True,
            text=True,
            env=environment,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise AcadosCodegenError(
            "solver shared library failed isolated dependency/symbol loading"
        ) from exc


def _new_result(
    output_root: Path,
    files: tuple[GeneratedFileRecord, ...],
    library_validation: dict[str, Any],
) -> AcadosCodegenResult:
    tree_sha256 = generated_tree_sha256(files)
    library = solver_library_record(files)
    values = {
        "output_directory": output_root,
        "files": files,
        "generated_tree_sha256": tree_sha256,
        "solver_library_relative_path": library.relative_path,
        "solver_library_raw_sha256": library.raw_sha256,
        "solver_library_size_bytes": library.size_bytes,
        "elf_class": library_validation["elf_class"],
        "elf_machine": library_validation["elf_machine"],
        "required_exported_symbols": tuple(
            library_validation["required_exported_symbols"]
        ),
    }
    provisional = AcadosCodegenResult(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_RESULT_TOKEN,
    )
    semantic_sha256 = sha256_json(_result_payload(provisional))
    return AcadosCodegenResult(
        **values,
        semantic_sha256=semantic_sha256,
        _construction_token=_RESULT_TOKEN,
    )


def generate_and_build_acados(
    graph: CasadiGraphBundle,
    assembly: AcadosOcpAssembly,
    solver_options: SolverOptionsSnapshot,
    codegen_options: CodegenOptionsSnapshot,
    output_directory: Path | str,
    acados_source_root: Path | str,
    tera_executable: Path | str,
) -> AcadosCodegenResult:
    """Generate, compile, canonicalize, and verify one development artifact."""

    checked_graph, checked_assembly, checked_solver, checked_codegen = (
        _checked_authorities(graph, assembly, solver_options, codegen_options)
    )
    try:
        require_codegen_compiler_environment(checked_codegen)
    except ValueError as exc:
        raise AcadosCodegenError(
            "compiler environment changed before code generation"
        ) from exc
    source_root = _safe_existing_path(
        acados_source_root,
        "Acados source root",
        directory=True,
    )
    tera = _safe_existing_path(
        tera_executable,
        "Tera renderer",
        directory=False,
        executable=True,
    )
    nm_path_value = shutil.which("nm")
    make_path_value = shutil.which("make")
    if nm_path_value is None or make_path_value is None:
        raise AcadosCodegenDependencyError("make and nm are required for codegen")
    nm_path = _safe_existing_path(
        Path(nm_path_value).resolve(strict=True),
        "nm",
        directory=False,
        executable=True,
    )
    make_path = _safe_existing_path(
        Path(make_path_value).resolve(strict=True),
        "make",
        directory=False,
        executable=True,
    )
    machine_name = platform.machine().lower()
    expected_machine = ELF_MACHINE_BY_PLATFORM.get(machine_name)
    if not sys.platform.startswith("linux") or expected_machine is None:
        raise AcadosCodegenDependencyError(
            f"unsupported development codegen platform: {sys.platform}/{machine_name}"
        )

    try:
        backend = require_acados_backend()
        solver_type = _require_codegen_backend(backend)
        if type(checked_assembly.ocp) is not backend.ocp_type:
            raise AcadosCodegenError("assembly does not hold the loaded AcadosOcp type")
        validate_consistent_ocp(
            backend,
            checked_assembly.ocp,
            checked_graph,
            checked_solver,
        )
        validate_applied_solver_options(
            backend,
            checked_assembly.ocp.solver_options,
            checked_solver,
        )
        require_acados_ocp_solver_options_baseline(
            checked_assembly.ocp,
            checked_assembly.backend_solver_options_baseline_sha256,
        )
        _bind_acados_source_root(
            checked_assembly.ocp,
            source_root,
            checked_assembly,
        )
        apply_acados_solver_codegen_options(
            checked_assembly.ocp.solver_options,
            checked_codegen,
        )
    except AcadosCodegenError:
        raise
    except Exception as exc:
        raise AcadosCodegenError("Acados OCP changed before code generation") from exc

    try:
        require_codegen_compiler_environment(checked_codegen)
        output_root = prepare_empty_codegen_directory(output_directory)
    except ValueError as exc:
        raise AcadosCodegenError(str(exc)) from exc
    json_path = output_root / ACADOS_JSON_FILENAME
    ocp = checked_assembly.ocp
    ocp.code_gen_opts.code_export_directory = str(output_root)
    ocp.code_gen_opts.json_file = str(json_path)

    try:
        with _codegen_environment(source_root, tera):
            solver_type.generate(
                ocp,
                str(json_path),
                simulink_opts=None,
                cmake_builder=None,
                verbose=checked_codegen.verbose,
            )
            validate_applied_solver_options(backend, ocp.solver_options, checked_solver)
            validate_applied_acados_solver_codegen_options(
                ocp.solver_options,
                checked_codegen,
            )
            require_acados_ocp_solver_options_baseline(
                ocp,
                checked_assembly.backend_solver_options_baseline_sha256,
            )
            inventory_codegen_tree(output_root)
            validate_generated_acados_json(
                json_path,
                output_root,
                source_root,
                checked_assembly,
                checked_solver,
                checked_codegen,
                canonicalized_output_root=False,
            )
            require_codegen_compiler_environment(checked_codegen)
            _build_with_fixed_make(
                make_path,
                output_root,
                verbose=checked_codegen.verbose,
            )
            require_codegen_compiler_environment(checked_codegen)
    except (AcadosCodegenValidationError, ValueError) as exc:
        raise AcadosCodegenError(
            "Acados generation/build validation failed; partial staging was retained"
        ) from exc
    except Exception as exc:
        raise AcadosCodegenError(
            "Acados generate/build failed; partial staging was retained"
        ) from exc

    try:
        inventory_generated_tree(output_root)
        canonicalize_generated_output_root(
            output_root,
            checked_assembly,
            checked_solver,
            checked_codegen,
            source_root,
        )
        files = inventory_generated_tree(output_root)
        library = solver_library_record(files)
        library_validation = validate_solver_shared_library(
            output_root,
            library,
            nm_path,
            expected_machine,
        )
        _validate_solver_load(source_root, output_root, library)
        validate_no_embedded_output_root(output_root, files)
        validate_generated_tree(output_root, files)
        result = _new_result(output_root, files, library_validation)
        return require_acados_codegen_result(result)
    except (AcadosCodegenValidationError, ValueError) as exc:
        raise AcadosCodegenError(
            "built artifact failed validation; partial staging was retained"
        ) from exc


__all__ = [
    "ACADOS_CODEGEN_ARTIFACT_CLASS",
    "ACADOS_CODEGEN_PERFORMANCE_STATUS",
    "ACADOS_CODEGEN_PROMOTION_STATUS",
    "ACADOS_CODEGEN_RESULT_SCHEMA",
    "ACADOS_CODEGEN_RESULT_SCOPE",
    "ACADOS_CODEGEN_STATUS",
    "CODEGEN_FAILURE_OUTPUT_POLICY",
    "OUTPUT_ROOT_IDENTITY_POLICY",
    "AcadosCodegenDependencyError",
    "AcadosCodegenError",
    "AcadosCodegenResult",
    "generate_and_build_acados",
    "require_acados_codegen_result",
]
