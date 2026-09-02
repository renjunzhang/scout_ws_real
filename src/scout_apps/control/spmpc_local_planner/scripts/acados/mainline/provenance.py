"""Top-level Stage 3-D4 source, toolchain, and Acados provenance snapshot.

The module remains lazy and independent of CasADi, NumPy, acados_template,
ROS, and the historical Stage 0/1 gates until capture is explicitly invoked.
"""

from __future__ import annotations

import os
import platform
import sys
from dataclasses import InitVar, dataclass
from pathlib import Path
from typing import Any

from .identity import sha256_json
from .provenance_acados import (
    ACADOS_PREFIX_POLICY,
    ACADOS_REQUIRED_TAG,
    AcadosInstallIdentity,
    capture_acados_install,
    require_acados_install,
)
from .provenance_common import (
    GIT_DIRTY_POLICY,
    ProvenanceError,
    canonical_absolute_path,
    relative_path,
    require_digest,
    require_text,
)
from .provenance_files import (
    LinkedFileIdentity,
    SourceTreeIdentity,
    ToolIdentity,
    capture_linked_file,
    capture_selected_source_files,
    capture_source_tree,
    capture_tool_identity,
    require_tool_identity,
)
from .provenance_git import (
    MAINLINE_BASE_SHA,
    MAINLINE_BRANCH,
    RepositoryIdentity,
    capture_repository_identity,
    require_repository_identity,
)
from .provenance_python import (
    PythonRuntimeIdentity,
    capture_python_runtime,
    require_python_runtime,
)

PROVENANCE_SCHEMA = "spmpc_mainline_codegen_provenance_v1"
PROVENANCE_SCOPE = "SOURCE_TOOLCHAIN_AND_ACADOS_INSTALL_IDENTITY"
PROVENANCE_STATUS = "CAPTURED_FOR_DEV_UNVALIDATED"
PROVENANCE_ARTIFACT_CLASS = "DEV_UNVALIDATED"
PROVENANCE_PROMOTION_STATUS = "NOT_PROMOTED"
STAGING_LOCATION_POLICY = "ABSOLUTE_STAGING_ROOT_EXCLUDED_FROM_IDENTITY"

GENERATOR_API = "AcadosOcpSolver.generate"
GENERATOR_ARGUMENTS = (
    ("simulink_opts", None),
    ("cmake_builder", None),
    ("verbose", False),
)
BUILD_COMMANDS = (
    ("make", "--no-print-directory", "clean_ocp_shared_lib"),
    ("make", "--no-print-directory", "ocp_shared_lib"),
)
TOOL_ROLES = (
    "git",
    "python",
    "tera",
    "make",
    "nm",
    "readelf",
    "cc",
    "cxx",
    "ar",
    "ranlib",
)

_TOKEN = object()


def _compiler_command(
    environment: tuple[tuple[str, str | None], ...],
    name: str,
    fallback: str,
) -> str:
    value = dict(environment).get(name)
    return value if value else fallback


def _canonical_compiler_environment_names() -> tuple[str, ...]:
    """Load the single compiler-environment authority only when capturing.

    Keeping this import local preserves the dependency-light/lazy top-level
    provenance import while ensuring that this layer cannot drift from the
    D4 codegen-options list.
    """

    try:
        from .codegen_options import COMPILER_ENVIRONMENT_NAMES
    except (ImportError, AttributeError) as exc:
        raise ProvenanceError(
            "cannot load the canonical compiler environment authority"
        ) from exc
    if (
        type(COMPILER_ENVIRONMENT_NAMES) is not tuple
        or len(COMPILER_ENVIRONMENT_NAMES) != 28
        or any(type(name) is not str or not name for name in COMPILER_ENVIRONMENT_NAMES)
        or len(set(COMPILER_ENVIRONMENT_NAMES)) != 28
    ):
        raise ProvenanceError(
            "codegen_options compiler environment authority must contain "
            "28 unique names"
        )
    return COMPILER_ENVIRONMENT_NAMES


def _capture_tools(
    compiler_environment: tuple[tuple[str, str | None], ...],
    tera_executable: Path | str,
) -> tuple[ToolIdentity, ...]:
    tera_path = str(canonical_absolute_path(tera_executable, "Tera executable"))
    requests = (
        ("git", "git", (("version", ("--version",)),)),
        ("python", sys.executable, (("version", ("--version",)),)),
        ("tera", tera_path, (("version", ("--version",)),)),
        ("make", "make", (("version", ("--version",)),)),
        ("nm", "nm", (("version", ("--version",)),)),
        ("readelf", "readelf", (("version", ("--version",)),)),
        (
            "cc",
            _compiler_command(compiler_environment, "CC", "cc"),
            (
                ("version", ("--version",)),
                ("target", ("-dumpmachine",)),
                ("full_version", ("-dumpfullversion", "-dumpversion")),
            ),
        ),
        (
            "cxx",
            _compiler_command(compiler_environment, "CXX", "c++"),
            (
                ("version", ("--version",)),
                ("target", ("-dumpmachine",)),
                ("full_version", ("-dumpfullversion", "-dumpversion")),
            ),
        ),
        (
            "ar",
            _compiler_command(compiler_environment, "AR", "ar"),
            (("version", ("--version",)),),
        ),
        (
            "ranlib",
            _compiler_command(compiler_environment, "RANLIB", "ranlib"),
            (("version", ("--version",)),),
        ),
    )
    tools = tuple(capture_tool_identity(*item) for item in requests)
    if tuple(item.role for item in tools) != TOOL_ROLES:
        raise ProvenanceError("captured tool role order drifted")
    return tools


def _validate_compiler_environment(
    value: tuple[tuple[str, str | None], ...],
    *,
    require_current: bool,
) -> None:
    expected_names = _canonical_compiler_environment_names()
    if type(value) is not tuple or len(value) != len(expected_names):
        raise ProvenanceError(
            "compiler environment must contain exactly the canonical 28 entries"
        )
    names: list[str] = []
    settings: list[str | None] = []
    for item in value:
        if type(item) is not tuple or len(item) != 2:
            raise ProvenanceError("compiler environment entries must be pairs")
        name, setting = item
        require_text(name, "compiler environment name")
        if setting is not None and type(setting) is not str:
            raise ProvenanceError("compiler environment values must be strings or null")
        names.append(name)
        settings.append(setting)
    if tuple(names) != expected_names:
        raise ProvenanceError(
            "compiler environment names must exactly match codegen_options"
        )
    if require_current:
        for name, setting in zip(names, settings):
            if os.environ.get(name) != setting:
                raise ProvenanceError(f"compiler environment changed at {name}")


@dataclass(frozen=True)
class CodegenProvenance:
    repository: RepositoryIdentity
    tools: tuple[ToolIdentity, ...]
    python_runtime: PythonRuntimeIdentity
    acados: AcadosInstallIdentity
    compiler_environment: tuple[tuple[str, str | None], ...]
    host_system: str
    host_release: str
    host_machine: str
    host_libc: tuple[str, str]
    host_byteorder: str
    source_paths: tuple[str, ...]
    tera_executable: Path
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = PROVENANCE_SCHEMA
    status: str = PROVENANCE_STATUS
    artifact_class: str = PROVENANCE_ARTIFACT_CLASS
    promotion_status: str = PROVENANCE_PROMOTION_STATUS

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("CodegenProvenance requires provenance capture")
        _validate_codegen_provenance(self)

    def to_dict(self) -> dict[str, Any]:
        payload = _provenance_payload(self)
        payload["semantic_identity"] = {
            "sha256": self.semantic_sha256,
            "scope": PROVENANCE_SCOPE,
        }
        return payload


def _provenance_payload(value: CodegenProvenance) -> dict[str, Any]:
    return {
        "schema_version": value.schema_version,
        "scope": PROVENANCE_SCOPE,
        "status": {
            "provenance": value.status,
            "artifact_class": value.artifact_class,
            "promotion": value.promotion_status,
        },
        "repository": value.repository.to_dict(),
        "compiler_environment": dict(value.compiler_environment),
        "tools": [item.to_dict() for item in value.tools],
        "python_runtime": value.python_runtime.to_dict(),
        "acados": value.acados.to_dict(),
        "host": {
            "system": value.host_system,
            "release": value.host_release,
            "machine": value.host_machine,
            "libc": list(value.host_libc),
            "byteorder": value.host_byteorder,
        },
        "logical_codegen_commands": {
            "generator_api": GENERATOR_API,
            "generator_arguments": dict(GENERATOR_ARGUMENTS),
            "build_commands": [list(item) for item in BUILD_COMMANDS],
            "staging_location": STAGING_LOCATION_POLICY,
        },
    }


def _validate_codegen_provenance(value: CodegenProvenance) -> None:
    if (
        value.schema_version != PROVENANCE_SCHEMA
        or value.status != PROVENANCE_STATUS
        or value.artifact_class != PROVENANCE_ARTIFACT_CLASS
        or value.promotion_status != PROVENANCE_PROMOTION_STATUS
    ):
        raise ProvenanceError("codegen provenance status or schema drifted")
    require_repository_identity(value.repository)
    if type(value.tools) is not tuple or tuple(item.role for item in value.tools) != TOOL_ROLES:
        raise ProvenanceError("codegen provenance tool set is incomplete or unordered")
    for item in value.tools:
        require_tool_identity(item)
    require_python_runtime(value.python_runtime)
    if value.python_runtime.executable_tool_sha256 != value.tools[1].semantic_sha256:
        raise ProvenanceError("Python runtime does not bind the captured interpreter")
    require_acados_install(value.acados)
    _validate_compiler_environment(value.compiler_environment, require_current=False)
    for field, label in (
        (value.host_system, "host system"),
        (value.host_release, "host release"),
        (value.host_machine, "host machine"),
    ):
        require_text(field, label)
    if type(value.host_libc) is not tuple or len(value.host_libc) != 2:
        raise ProvenanceError("host libc identity must be a pair")
    for item in value.host_libc:
        require_text(item, "host libc field", nonempty=False)
    if value.host_byteorder not in {"little", "big"}:
        raise ProvenanceError("host byte order is invalid")
    canonical_paths = tuple(
        relative_path(item, "provenance source path") for item in value.source_paths
    )
    if canonical_paths != tuple(sorted(canonical_paths)) or len(set(canonical_paths)) != len(
        canonical_paths
    ):
        raise ProvenanceError("provenance source paths must be uniquely sorted")
    if tuple(item.relative_path for item in value.repository.sources.files) != canonical_paths:
        raise ProvenanceError("provenance source paths differ from repository inventory")
    if not isinstance(value.tera_executable, Path) or not value.tera_executable.is_absolute():
        raise ProvenanceError("Tera capture path must be an absolute Path")
    if str(value.tera_executable) != value.tools[2].executable.requested_path:
        raise ProvenanceError("Tera capture path differs from its tool identity")
    require_digest(value.semantic_sha256, "codegen provenance semantic identity")


def require_codegen_provenance(value: Any) -> CodegenProvenance:
    if type(value) is not CodegenProvenance:
        raise ProvenanceError("provenance must have the exact CodegenProvenance type")
    _validate_codegen_provenance(value)
    if sha256_json(_provenance_payload(value)) != value.semantic_sha256:
        raise ProvenanceError("codegen provenance semantic identity is inconsistent")
    return value


def capture_codegen_provenance(
    repository_root: Path | str,
    source_paths: tuple[str, ...],
    compiler_environment: tuple[tuple[str, str | None], ...],
    acados_install_root: Path | str,
    tera_executable: Path | str,
) -> CodegenProvenance:
    """Capture one complete development generation environment.

    The compiler environment is supplied by the already-frozen D4 codegen
    options. It must still match the process, but this layer does not import
    that higher-level contract type.
    """

    _validate_compiler_environment(compiler_environment, require_current=True)
    tools = _capture_tools(compiler_environment, tera_executable)
    by_role = {item.role: item for item in tools}
    repository = capture_repository_identity(
        repository_root,
        source_paths,
        by_role["git"],
    )
    python_runtime = capture_python_runtime(by_role["python"])
    acados = capture_acados_install(
        acados_install_root,
        by_role["git"],
        by_role["readelf"],
    )
    acados_after = capture_acados_install(
        acados_install_root,
        by_role["git"],
        by_role["readelf"],
    )
    if acados_after.semantic_sha256 != acados.semantic_sha256:
        raise ProvenanceError("Acados install changed during environment capture")
    repository_after = capture_repository_identity(
        repository_root,
        source_paths,
        by_role["git"],
    )
    if repository_after.semantic_sha256 != repository.semantic_sha256:
        raise ProvenanceError("repository changed during environment capture")
    _validate_compiler_environment(compiler_environment, require_current=True)
    values = {
        "repository": repository,
        "tools": tools,
        "python_runtime": python_runtime,
        "acados": acados,
        "compiler_environment": compiler_environment,
        "host_system": platform.system(),
        "host_release": platform.release(),
        "host_machine": platform.machine(),
        "host_libc": platform.libc_ver(),
        "host_byteorder": sys.byteorder,
        "source_paths": source_paths,
        "tera_executable": Path(by_role["tera"].executable.requested_path),
    }
    provisional = CodegenProvenance(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_TOKEN,
    )
    return CodegenProvenance(
        **values,
        semantic_sha256=sha256_json(_provenance_payload(provisional)),
        _construction_token=_TOKEN,
    )


def require_codegen_provenance_current(value: Any) -> CodegenProvenance:
    """Recapture all inputs and reject drift since the first snapshot."""

    checked = require_codegen_provenance(value)
    current = capture_codegen_provenance(
        checked.repository.repository_root,
        checked.source_paths,
        checked.compiler_environment,
        checked.acados.install_root,
        checked.tera_executable,
    )
    if current.semantic_sha256 != checked.semantic_sha256:
        raise ProvenanceError("codegen provenance changed after capture")
    return checked


__all__ = [
    "ACADOS_PREFIX_POLICY",
    "ACADOS_REQUIRED_TAG",
    "BUILD_COMMANDS",
    "GENERATOR_API",
    "GIT_DIRTY_POLICY",
    "MAINLINE_BASE_SHA",
    "MAINLINE_BRANCH",
    "PROVENANCE_ARTIFACT_CLASS",
    "PROVENANCE_SCHEMA",
    "PROVENANCE_SCOPE",
    "PROVENANCE_STATUS",
    "AcadosInstallIdentity",
    "CodegenProvenance",
    "LinkedFileIdentity",
    "ProvenanceError",
    "RepositoryIdentity",
    "SourceTreeIdentity",
    "ToolIdentity",
    "capture_codegen_provenance",
    "capture_linked_file",
    "capture_repository_identity",
    "capture_selected_source_files",
    "capture_source_tree",
    "capture_tool_identity",
    "require_codegen_provenance",
    "require_codegen_provenance_current",
    "require_repository_identity",
    "require_tool_identity",
]
