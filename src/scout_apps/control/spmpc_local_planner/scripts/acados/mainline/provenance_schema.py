"""Serialized authority and offline validation for Stage 3-D4 provenance.

This module deliberately deals only in JSON-compatible values.  It does not
construct any provenance dataclass and it never imports CasADi, NumPy, or
``acados_template``.  Capture-time objects remain owned by the provenance
modules; this module is the single authority for the bytes that are later
embedded in an artifact contract.
"""

from __future__ import annotations

import posixpath
import shlex
from pathlib import PurePosixPath
from typing import Any

from .identity import IdentityError, require_sha256, sha256_json, strict_json_equal
from .provenance_acados import (
    ACADOS_COMMIT_MARKER_LOGICAL_NAME,
    ACADOS_IDENTITY_SCHEMA,
    ACADOS_INCLUDE_TREE_LOGICAL_ROOT,
    ACADOS_INTERFACE_TREE_LOGICAL_ROOT,
    ACADOS_LIBRARY_NAMES,
    ACADOS_LIBRARY_SONAMES,
    ACADOS_LINK_LIBS_LOGICAL_NAME,
    ACADOS_PREFIX_POLICY,
    ACADOS_REQUIRED_TAG,
    REQUIRED_ACADOS_SUBMODULES,
    TERA_SOURCE_BINDING_STATUS,
)
from .provenance_common import (
    GIT_DIRTY_POLICY,
    REPOSITORY_LOCATION_POLICY,
    ProvenanceError,
    relative_path,
    require_absolute_text,
    require_git_sha,
    require_text,
)
from .provenance_files import (
    FILE_IDENTITY_SCHEMA,
    TOOL_IDENTITY_SCHEMA,
    TREE_IDENTITY_SCHEMA,
)
from .provenance_git import (
    MAINLINE_BASE_SHA,
    MAINLINE_BRANCH,
    REPOSITORY_IDENTITY_SCHEMA,
    REPOSITORY_SOURCE_LOGICAL_ROOT,
)
from .provenance_python import (
    CASADI_PACKAGE_FILE_LOGICAL_NAMES,
    NUMPY_PACKAGE_FILE_LOGICAL_NAMES,
    PYTHON_IDENTITY_SCHEMA,
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

_EXPECTED_TOOL_PROBES = {
    "git": [("version", ["--version"])],
    "python": [("version", ["--version"])],
    "tera": [("version", ["--version"])],
    "make": [("version", ["--version"])],
    "nm": [("version", ["--version"])],
    "readelf": [("version", ["--version"])],
    "cc": [
        ("version", ["--version"]),
        ("target", ["-dumpmachine"]),
        ("full_version", ["-dumpfullversion", "-dumpversion"]),
    ],
    "cxx": [
        ("version", ["--version"]),
        ("target", ["-dumpmachine"]),
        ("full_version", ["-dumpfullversion", "-dumpversion"]),
    ],
    "ar": [("version", ["--version"])],
    "ranlib": [("version", ["--version"])],
}


def _object(value: Any, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != keys:
        raise ProvenanceError(f"{label} keys do not match the v1 schema")
    return value


def _digest(value: Any, label: str) -> str:
    try:
        return require_sha256(value, label)
    except IdentityError as exc:
        raise ProvenanceError(str(exc)) from exc


def _text(value: Any, label: str, *, nonempty: bool = True) -> str:
    return require_text(value, label, nonempty=nonempty)


def _absolute(value: Any, label: str) -> str:
    try:
        return require_absolute_text(value, label)
    except (ProvenanceError, TypeError, ValueError):
        raise ProvenanceError(f"{label} must be an absolute canonical path") from None


def _git_sha(value: Any, label: str) -> str:
    return require_git_sha(value, label)


def _check_hash(payload: dict[str, Any], digest: Any, label: str) -> None:
    _digest(digest, label)
    try:
        actual = sha256_json(payload)
    except (IdentityError, TypeError, ValueError) as exc:
        raise ProvenanceError(f"{label} payload is not canonicalizable") from exc
    if actual != digest:
        raise ProvenanceError(f"{label} is inconsistent")


def _semantic_document(
    value: Any,
    *,
    label: str,
    scope: str | None,
) -> dict[str, Any]:
    if type(value) is not dict or "semantic_identity" not in value:
        raise ProvenanceError(f"{label} is not an identity document")
    identity_keys = {"sha256", "scope"} if scope is not None else {"sha256"}
    identity = _object(value["semantic_identity"], identity_keys, f"{label} identity")
    _digest(identity["sha256"], f"{label} semantic identity")
    if scope is not None and identity["scope"] != scope:
        raise ProvenanceError(f"{label} semantic scope drifted")
    payload = {key: item for key, item in value.items() if key != "semantic_identity"}
    _check_hash(payload, identity["sha256"], f"{label} semantic identity")
    return value


def _validate_linked_file(value: Any, label: str) -> dict[str, Any]:
    document = _object(
        value,
        {
            "schema_version",
            "logical_name",
            "requested_path",
            "resolved_path",
            "leaf_symlink_chain",
            "size_bytes",
            "raw_sha256",
            "executable",
        },
        label,
    )
    if document["schema_version"] != FILE_IDENTITY_SCHEMA:
        raise ProvenanceError(f"{label} schema drifted")
    _text(document["logical_name"], f"{label} logical name")
    _absolute(document["requested_path"], f"{label} requested path")
    _absolute(document["resolved_path"], f"{label} resolved path")
    chain = document["leaf_symlink_chain"]
    if type(chain) is not list or len(chain) >= 32:
        raise ProvenanceError(
            f"{label} symlink chain must be an array shorter than 32 hops"
        )
    expected_path = document["requested_path"]
    visited: set[str] = set()
    for index, hop in enumerate(chain):
        hop = _object(hop, {"path", "target"}, f"{label} symlink hop {index}")
        hop_path = _absolute(hop["path"], f"{label} symlink hop path")
        if hop_path != expected_path or hop_path in visited:
            raise ProvenanceError(f"{label} symlink chain is discontinuous")
        visited.add(hop_path)
        target = _text(hop["target"], f"{label} symlink target")
        if PurePosixPath(target).is_absolute():
            expected_path = posixpath.normpath(target)
        else:
            expected_path = posixpath.normpath(
                posixpath.join(posixpath.dirname(hop_path), target)
            )
        _absolute(expected_path, f"{label} symlink target path")
    if expected_path != document["resolved_path"]:
        raise ProvenanceError(
            f"{label} requested path, symlink chain, and resolved path disagree"
        )
    if type(document["size_bytes"]) is not int or document["size_bytes"] < 0:
        raise ProvenanceError(f"{label} size must be a nonnegative integer")
    _digest(document["raw_sha256"], f"{label} raw identity")
    if type(document["executable"]) is not bool:
        raise ProvenanceError(f"{label} executable flag must be bool")
    return document


def _validate_source_tree(
    value: Any,
    label: str,
    *,
    logical_root: str,
) -> dict[str, Any]:
    document = _semantic_document(value, label=label, scope=None)
    _object(
        document,
        {
            "schema_version",
            "logical_root",
            "capture_root",
            "files",
            "semantic_identity",
        },
        label,
    )
    if document["schema_version"] != TREE_IDENTITY_SCHEMA:
        raise ProvenanceError(f"{label} schema drifted")
    if document["logical_root"] != logical_root:
        raise ProvenanceError(f"{label} logical root drifted")
    if document["capture_root"] != REPOSITORY_LOCATION_POLICY:
        raise ProvenanceError(f"{label} capture-root policy drifted")
    files = document["files"]
    if type(files) is not list or not files:
        raise ProvenanceError(f"{label} inventory must be a non-empty array")
    previous = None
    for index, item in enumerate(files):
        item = _object(
            item,
            {"relative_path", "size_bytes", "raw_sha256"},
            f"{label} file {index}",
        )
        path = relative_path(item["relative_path"], f"{label} relative path")
        if previous is not None and path <= previous:
            raise ProvenanceError(f"{label} files must be uniquely sorted")
        previous = path
        if type(item["size_bytes"]) is not int or item["size_bytes"] < 0:
            raise ProvenanceError(f"{label} file size must be a nonnegative integer")
        _digest(item["raw_sha256"], f"{label} file identity")
    return document


def _validate_probe(value: Any, label: str) -> dict[str, Any]:
    probe = _object(
        value,
        {"name", "arguments", "output_text", "output_raw_sha256"},
        label,
    )
    _text(probe["name"], f"{label} name")
    arguments = probe["arguments"]
    if type(arguments) is not list or any(
        type(item) is not str or "\x00" in item for item in arguments
    ):
        raise ProvenanceError(f"{label} arguments are malformed")
    _text(probe["output_text"], f"{label} output")
    _digest(probe["output_raw_sha256"], f"{label} output identity")
    return probe


def _command_executable_name(value: str, label: str) -> str:
    try:
        parts = shlex.split(value)
    except ValueError as exc:
        raise ProvenanceError(f"{label} cannot be parsed") from exc
    if len(parts) != 1:
        raise ProvenanceError(f"{label} must name exactly one executable")
    name = parts[0]
    if "/" in name and not PurePosixPath(name).is_absolute():
        raise ProvenanceError(f"{label} path must be absolute")
    return name


def _validate_tool(
    value: Any,
    index: int,
    compiler_environment: dict[str, Any],
) -> dict[str, Any]:
    label = f"provenance tool {index}"
    tool = _semantic_document(value, label=label, scope=None)
    _object(
        tool,
        {
            "schema_version",
            "role",
            "requested_command",
            "executable",
            "probes",
            "semantic_identity",
        },
        label,
    )
    if tool["schema_version"] != TOOL_IDENTITY_SCHEMA:
        raise ProvenanceError(f"{label} schema drifted")
    role = _text(tool["role"], f"{label} role")
    if role != TOOL_ROLES[index]:
        raise ProvenanceError(f"{label} role order drifted")
    requested_command = _text(tool["requested_command"], f"{label} requested command")
    executable_name = _command_executable_name(
        requested_command, f"{label} requested command"
    )
    executable = _validate_linked_file(tool["executable"], f"{label} executable")
    if executable["logical_name"] != f"tool:{role}":
        raise ProvenanceError(f"{label} executable logical name drifted")
    if executable["executable"] is not True:
        raise ProvenanceError(f"{label} executable flag is not true")
    if role in {"python", "tera"} and requested_command != executable["requested_path"]:
        raise ProvenanceError(f"{label} requested path does not bind its executable")
    if role in {"git", "make", "nm", "readelf"} and requested_command != role:
        raise ProvenanceError(f"{label} requested command drifted")
    if role in {"cc", "cxx", "ar", "ranlib"}:
        environment_name = {
            "cc": "CC",
            "cxx": "CXX",
            "ar": "AR",
            "ranlib": "RANLIB",
        }[role]
        fallback = {"cc": "cc", "cxx": "c++", "ar": "ar", "ranlib": "ranlib"}[role]
        expected_command = compiler_environment[environment_name] or fallback
        if requested_command != expected_command:
            raise ProvenanceError(f"{label} does not bind its compiler environment")
    if "/" in executable_name:
        expected_executable_path = posixpath.normpath(executable_name)
        if executable["requested_path"] != expected_executable_path:
            raise ProvenanceError(
                f"{label} executable path does not bind its requested command"
            )
    probes = tool["probes"]
    if type(probes) is not list or not probes:
        raise ProvenanceError(f"{label} probes must be a non-empty array")
    expected = _EXPECTED_TOOL_PROBES[role]
    if len(probes) != len(expected):
        raise ProvenanceError(f"{label} probe set is incomplete")
    names: list[str] = []
    for probe_value, expected_probe in zip(probes, expected):
        probe = _validate_probe(probe_value, f"{label} probe")
        name, arguments = expected_probe
        if probe["name"] != name or probe["arguments"] != arguments:
            raise ProvenanceError(f"{label} probe arguments drifted")
        names.append(probe["name"])
    if len(names) != len(set(names)):
        raise ProvenanceError(f"{label} probe names are duplicated")
    payload = {key: item for key, item in tool.items() if key != "semantic_identity"}
    _check_hash(
        payload, tool["semantic_identity"]["sha256"], f"{label} semantic identity"
    )
    return tool


def _validate_python_runtime(value: Any) -> dict[str, Any]:
    runtime = _semantic_document(value, label="Python runtime", scope=None)
    _object(
        runtime,
        {
            "schema_version",
            "implementation",
            "version",
            "version_info",
            "executable_tool_sha256",
            "sys_prefix",
            "sys_base_prefix",
            "sys_path",
            "PYTHONPATH",
            "packages",
            "semantic_identity",
        },
        "Python runtime",
    )
    if runtime["schema_version"] != PYTHON_IDENTITY_SCHEMA:
        raise ProvenanceError("Python runtime schema drifted")
    _text(runtime["implementation"], "Python implementation")
    _text(runtime["version"], "Python version")
    version_info = runtime["version_info"]
    if (
        type(version_info) is not list
        or len(version_info) != 3
        or any(type(item) is not int or item < 0 for item in version_info)
    ):
        raise ProvenanceError("Python version_info must contain three integers")
    _digest(runtime["executable_tool_sha256"], "Python executable tool identity")
    _absolute(runtime["sys_prefix"], "Python sys.prefix")
    _absolute(runtime["sys_base_prefix"], "Python sys.base_prefix")
    sys_path = runtime["sys_path"]
    if type(sys_path) is not list or not sys_path:
        raise ProvenanceError("Python sys.path must be a non-empty array")
    for item in sys_path:
        _absolute(item, "Python sys.path entry")
    if runtime["PYTHONPATH"] is not None:
        _text(runtime["PYTHONPATH"], "PYTHONPATH", nonempty=False)
    packages = runtime["packages"]
    if (
        type(packages) is not list
        or len(packages) != 2
        or any(type(item) is not dict for item in packages)
        or [item.get("name") for item in packages] != ["casadi", "numpy"]
    ):
        raise ProvenanceError("Python package identities must be CasADi then NumPy")
    expected_package_files = {
        "casadi": CASADI_PACKAGE_FILE_LOGICAL_NAMES,
        "numpy": NUMPY_PACKAGE_FILE_LOGICAL_NAMES,
    }
    for package in packages:
        package = _object(package, {"name", "version", "files"}, "Python package")
        _text(package["name"], "Python package name")
        _text(package["version"], "Python package version")
        files = package["files"]
        expected_names = expected_package_files[package["name"]]
        if type(files) is not list or len(files) != len(expected_names):
            raise ProvenanceError("Python package file identity set is incomplete")
        logical_names: list[str] = []
        for file_value in files:
            file_value = _validate_linked_file(file_value, "Python package file")
            logical_names.append(file_value["logical_name"])
        if tuple(logical_names) != expected_names:
            raise ProvenanceError("Python package file logical names drifted")
    payload = {key: item for key, item in runtime.items() if key != "semantic_identity"}
    _check_hash(
        payload,
        runtime["semantic_identity"]["sha256"],
        "Python runtime semantic identity",
    )
    return runtime


def _validate_acados_install(value: Any) -> dict[str, Any]:
    acados = _semantic_document(value, label="Acados install", scope=None)
    _object(
        acados,
        {
            "schema_version",
            "install_root",
            "install_prefix_policy",
            "source_repository",
            "commit_marker",
            "commit_marker_file",
            "link_libs",
            "interface_source_binding_status",
            "tera_source_binding_status",
            "interface_tree",
            "include_tree",
            "submodules",
            "libraries",
            "semantic_identity",
        },
        "Acados install",
    )
    if acados["schema_version"] != ACADOS_IDENTITY_SCHEMA:
        raise ProvenanceError("Acados install schema drifted")
    _absolute(acados["install_root"], "Acados install root")
    if acados["install_prefix_policy"] != ACADOS_PREFIX_POLICY:
        raise ProvenanceError("Acados install-prefix identity policy drifted")

    source = _object(
        acados["source_repository"],
        {
            "root",
            "head_sha",
            "exact_tag",
            "worktree_clean",
            "dirty_policy",
            "status_porcelain_sha256",
        },
        "Acados source repository",
    )
    _absolute(source["root"], "Acados source root")
    _git_sha(source["head_sha"], "Acados source HEAD")
    if source["exact_tag"] != ACADOS_REQUIRED_TAG:
        raise ProvenanceError("Acados exact source tag is not v0.5.4")
    if (
        type(source["worktree_clean"]) is not bool
        or source["dirty_policy"] != GIT_DIRTY_POLICY
    ):
        raise ProvenanceError("Acados source worktree policy drifted")
    _digest(source["status_porcelain_sha256"], "Acados source status identity")

    marker = _text(acados["commit_marker"], "Acados commit marker")
    if (
        len(marker) < 7
        or len(marker) > 40
        or any(character not in "0123456789abcdef" for character in marker)
    ):
        raise ProvenanceError("Acados commit marker must be lowercase hexadecimal")
    if not source["head_sha"].startswith(marker):
        raise ProvenanceError("Acados source HEAD does not match its commit marker")
    marker_file = _validate_linked_file(
        acados["commit_marker_file"], "Acados commit marker file"
    )
    if marker_file["logical_name"] != ACADOS_COMMIT_MARKER_LOGICAL_NAME:
        raise ProvenanceError("Acados commit marker logical name drifted")
    expected_marker_path = str(
        PurePosixPath(acados["install_root"]) / "lib" / "git_commit_hash"
    )
    if marker_file["requested_path"] != expected_marker_path:
        raise ProvenanceError("Acados commit marker path does not bind install root")
    if str(PurePosixPath(marker_file["resolved_path"]).parent.parent) != source["root"]:
        raise ProvenanceError("Acados commit marker does not bind source root")

    link_libs = _object(
        acados["link_libs"],
        {"file", "canonical_json_sha256"},
        "Acados link-libs identity",
    )
    link_libs_file = _validate_linked_file(link_libs["file"], "Acados link-libs file")
    if link_libs_file["logical_name"] != ACADOS_LINK_LIBS_LOGICAL_NAME:
        raise ProvenanceError("Acados link-libs logical name drifted")
    expected_link_libs_path = str(
        PurePosixPath(acados["install_root"]) / "lib" / "link_libs.json"
    )
    if link_libs_file["requested_path"] != expected_link_libs_path:
        raise ProvenanceError("Acados link-libs path does not bind install root")
    _digest(link_libs["canonical_json_sha256"], "Acados link-libs identity")
    if acados["interface_source_binding_status"] != "MATCHED_SOURCE_ROOT":
        raise ProvenanceError(
            "Acados interface and installed libraries are not source-bound"
        )
    if acados["tera_source_binding_status"] != TERA_SOURCE_BINDING_STATUS:
        raise ProvenanceError("Tera binary/source binding status drifted")
    _validate_source_tree(
        acados["interface_tree"],
        "Acados interface tree",
        logical_root=ACADOS_INTERFACE_TREE_LOGICAL_ROOT,
    )
    _validate_source_tree(
        acados["include_tree"],
        "Acados include tree",
        logical_root=ACADOS_INCLUDE_TREE_LOGICAL_ROOT,
    )

    submodules = acados["submodules"]
    if type(submodules) is not list:
        raise ProvenanceError("Acados submodules must be an array")
    paths: list[str] = []
    for item in submodules:
        item = _object(
            item,
            {"path", "commit_sha", "initialized", "worktree_matches_index"},
            "Acados submodule",
        )
        path = relative_path(item["path"], "Acados submodule path")
        if paths and path <= paths[-1]:
            raise ProvenanceError("Acados submodules must be uniquely sorted")
        paths.append(path)
        _git_sha(item["commit_sha"], "Acados submodule commit")
        if (
            type(item["initialized"]) is not bool
            or type(item["worktree_matches_index"]) is not bool
        ):
            raise ProvenanceError("Acados submodule states must be bool")
    by_path = {item["path"]: item for item in submodules}
    for required in REQUIRED_ACADOS_SUBMODULES:
        item = by_path.get(required)
        if (
            item is None
            or item["initialized"] is not True
            or item["worktree_matches_index"] is not True
        ):
            raise ProvenanceError(
                f"required Acados submodule is not pinned: {required}"
            )

    libraries = acados["libraries"]
    if (
        type(libraries) is not list
        or len(libraries) != len(ACADOS_LIBRARY_NAMES)
        or any(type(item) is not dict for item in libraries)
        or [item.get("logical_name") for item in libraries]
        != list(ACADOS_LIBRARY_NAMES)
    ):
        raise ProvenanceError("Acados library identities are incomplete or unordered")
    for item in libraries:
        item = _object(
            item,
            {
                "logical_name",
                "file",
                "soname",
                "needed",
                "rpath",
                "runpath",
                "dynamic_section_sha256",
            },
            "Acados dynamic library",
        )
        name = item["logical_name"]
        if (
            name not in ACADOS_LIBRARY_NAMES
            or item["soname"] != ACADOS_LIBRARY_SONAMES[name]
        ):
            raise ProvenanceError("Acados dynamic library SONAME drifted")
        library_file = _validate_linked_file(
            item["file"], "Acados dynamic library file"
        )
        if library_file["logical_name"] != f"acados/lib/{name}":
            raise ProvenanceError("Acados dynamic library file logical name drifted")
        expected_library_path = str(
            PurePosixPath(acados["install_root"]) / "lib" / name
        )
        if library_file["requested_path"] != expected_library_path:
            raise ProvenanceError(
                "Acados dynamic library path does not bind install root"
            )
        needed = item["needed"]
        if type(needed) is not list or any(
            type(entry) is not str or not entry for entry in needed
        ):
            raise ProvenanceError("Acados NEEDED entries must be strings")
        for key in ("rpath", "runpath"):
            if item[key] is not None:
                _text(item[key], f"Acados dynamic library {key}")
        _digest(item["dynamic_section_sha256"], "Acados dynamic-section identity")

    payload = {key: item for key, item in acados.items() if key != "semantic_identity"}
    _check_hash(
        payload, acados["semantic_identity"]["sha256"], "Acados semantic identity"
    )
    return acados


def _validate_repository(value: Any) -> dict[str, Any]:
    repository = _semantic_document(value, label="repository", scope=None)
    _object(
        repository,
        {
            "schema_version",
            "repository_root",
            "branch",
            "head_sha",
            "base_sha",
            "worktree",
            "sources",
            "semantic_identity",
        },
        "repository",
    )
    if repository["schema_version"] != REPOSITORY_IDENTITY_SCHEMA:
        raise ProvenanceError("repository identity schema drifted")
    if repository["repository_root"] != REPOSITORY_LOCATION_POLICY:
        raise ProvenanceError("repository location policy drifted")
    if repository["branch"] != MAINLINE_BRANCH:
        raise ProvenanceError("repository branch is not the canonical mainline")
    _git_sha(repository["head_sha"], "repository HEAD")
    if repository["base_sha"] != MAINLINE_BASE_SHA:
        raise ProvenanceError("repository base SHA drifted")
    _git_sha(repository["base_sha"], "repository base SHA")
    worktree = _object(
        repository["worktree"],
        {"clean", "dirty_policy", "status_entry_count", "status_porcelain_sha256"},
        "repository worktree",
    )
    if (
        type(worktree["clean"]) is not bool
        or worktree["dirty_policy"] != GIT_DIRTY_POLICY
        or type(worktree["status_entry_count"]) is not int
        or worktree["status_entry_count"] < 0
        or worktree["clean"] != (worktree["status_entry_count"] == 0)
    ):
        raise ProvenanceError("repository worktree metadata drifted")
    _digest(worktree["status_porcelain_sha256"], "repository status identity")
    _validate_source_tree(
        repository["sources"],
        "repository source inventory",
        logical_root=REPOSITORY_SOURCE_LOGICAL_ROOT,
    )
    payload = {
        key: item for key, item in repository.items() if key != "semantic_identity"
    }
    _check_hash(
        payload,
        repository["semantic_identity"]["sha256"],
        "repository semantic identity",
    )
    return repository


def _compiler_environment_names() -> tuple[str, ...]:
    try:
        from .codegen_options import COMPILER_ENVIRONMENT_NAMES
    except (ImportError, AttributeError) as exc:
        raise ProvenanceError("cannot load compiler environment authority") from exc
    if (
        type(COMPILER_ENVIRONMENT_NAMES) is not tuple
        or len(COMPILER_ENVIRONMENT_NAMES) != 28
        or any(type(item) is not str or not item for item in COMPILER_ENVIRONMENT_NAMES)
        or len(set(COMPILER_ENVIRONMENT_NAMES)) != 28
    ):
        raise ProvenanceError(
            "compiler environment authority must contain 28 unique names"
        )
    return COMPILER_ENVIRONMENT_NAMES


def _validate_compiler_environment(value: Any) -> dict[str, Any]:
    names = _compiler_environment_names()
    if type(value) is not dict or set(value) != set(names):
        raise ProvenanceError(
            "compiler environment must contain exactly the canonical 28 entries"
        )
    if any(item is not None and type(item) is not str for item in value.values()):
        raise ProvenanceError("compiler environment values must be strings or null")
    return value


def _validate_logical_commands(value: Any) -> dict[str, Any]:
    commands = _object(
        value,
        {"generator_api", "generator_arguments", "build_commands", "staging_location"},
        "logical codegen commands",
    )
    expected = {
        "generator_api": GENERATOR_API,
        "generator_arguments": dict(GENERATOR_ARGUMENTS),
        "build_commands": [list(item) for item in BUILD_COMMANDS],
        "staging_location": STAGING_LOCATION_POLICY,
    }
    if not strict_json_equal(commands, expected):
        raise ProvenanceError("logical codegen command policy drifted")
    return commands


def validate_codegen_provenance_document(value: Any) -> dict[str, Any]:
    """Validate a complete serialized provenance snapshot offline.

    Paths, package versions, tool output, compiler values, and source hashes
    are captured observations and therefore vary between hosts.  Their types,
    nested identities, fixed policies, command syntax, and cross-layer links
    do not vary and are rejected if changed, even when all enclosing hashes
    are recomputed.
    """

    document = _semantic_document(
        value,
        label="codegen provenance",
        scope=PROVENANCE_SCOPE,
    )
    _object(
        document,
        {
            "schema_version",
            "scope",
            "status",
            "repository",
            "compiler_environment",
            "tools",
            "python_runtime",
            "acados",
            "host",
            "logical_codegen_commands",
            "semantic_identity",
        },
        "codegen provenance",
    )
    if (
        document["schema_version"] != PROVENANCE_SCHEMA
        or document["scope"] != PROVENANCE_SCOPE
    ):
        raise ProvenanceError("codegen provenance schema/scope drifted")
    if document["status"] != {
        "provenance": PROVENANCE_STATUS,
        "artifact_class": PROVENANCE_ARTIFACT_CLASS,
        "promotion": PROVENANCE_PROMOTION_STATUS,
    }:
        raise ProvenanceError("codegen provenance status drifted")

    _validate_repository(document["repository"])
    environment = _validate_compiler_environment(document["compiler_environment"])
    tools = document["tools"]
    if type(tools) is not list or len(tools) != len(TOOL_ROLES):
        raise ProvenanceError("codegen provenance tool set is incomplete")
    checked_tools = [
        _validate_tool(item, index, environment) for index, item in enumerate(tools)
    ]
    runtime = _validate_python_runtime(document["python_runtime"])
    if (
        runtime["executable_tool_sha256"]
        != checked_tools[1]["semantic_identity"]["sha256"]
    ):
        raise ProvenanceError("Python runtime does not bind the captured interpreter")
    _validate_acados_install(document["acados"])
    host = _object(
        document["host"],
        {"system", "release", "machine", "libc", "byteorder"},
        "provenance host",
    )
    for key in ("system", "release", "machine"):
        _text(host[key], f"provenance host {key}")
    if type(host["libc"]) is not list or len(host["libc"]) != 2:
        raise ProvenanceError("provenance host libc identity must be a pair")
    for item in host["libc"]:
        _text(item, "provenance host libc field", nonempty=False)
    if host["byteorder"] not in {"little", "big"}:
        raise ProvenanceError("provenance host byte order is invalid")
    _validate_logical_commands(document["logical_codegen_commands"])
    return document


__all__ = [
    "BUILD_COMMANDS",
    "GENERATOR_API",
    "GENERATOR_ARGUMENTS",
    "MAINLINE_BASE_SHA",
    "MAINLINE_BRANCH",
    "PROVENANCE_ARTIFACT_CLASS",
    "PROVENANCE_PROMOTION_STATUS",
    "PROVENANCE_SCHEMA",
    "PROVENANCE_SCOPE",
    "PROVENANCE_STATUS",
    "STAGING_LOCATION_POLICY",
    "TOOL_ROLES",
    "validate_codegen_provenance_document",
]
