"""Acados v0.5.4 source, installed headers, and shared-library identity."""

from __future__ import annotations

import importlib
import json
import re
from dataclasses import InitVar, dataclass
from pathlib import Path
from typing import Any

from .identity import IdentityError, read_stable_regular_file, sha256_bytes, sha256_json
from .provenance_common import (
    GIT_DIRTY_POLICY,
    SHORT_GIT_SHA_RE,
    ProvenanceError,
    canonical_absolute_path,
    relative_path,
    require_absolute_text,
    require_digest,
    require_git_sha,
    require_text,
)
from .provenance_files import (
    LinkedFileIdentity,
    SourceTreeIdentity,
    ToolIdentity,
    capture_command_probe,
    capture_linked_file,
    capture_source_tree,
    require_source_tree,
    require_tool_identity,
)
from .provenance_git import decode_git, git_command
from .provenance_python import module_file

ACADOS_REQUIRED_TAG = "v0.5.4"
ACADOS_IDENTITY_SCHEMA = "spmpc_mainline_acados_install_identity_v1"
ACADOS_PREFIX_POLICY = "ABSOLUTE_ACADOS_PREFIX_EMBEDDED_IN_GENERATED_TREE"
TERA_SOURCE_BINDING_STATUS = "BINARY_AND_SUBMODULE_IDENTITIES_RECORDED_SEPARATELY"

ACADOS_LIBRARY_NAMES = ("libacados.so", "libhpipm.so", "libblasfeo.so")
ACADOS_LIBRARY_SONAMES = {
    "libacados.so": "libacados.so",
    "libhpipm.so": "libhpipm.so",
    "libblasfeo.so": "libblasfeo.so.0",
}
REQUIRED_ACADOS_SUBMODULES = (
    "external/blasfeo",
    "external/hpipm",
    "interfaces/acados_template/tera_renderer",
)

_DYNAMIC_ENTRY_RE = re.compile(
    r"\((NEEDED|SONAME|RPATH|RUNPATH)\).*?\[(.*?)\]"
)
_TOKEN = object()


@dataclass(frozen=True)
class AcadosSubmoduleIdentity:
    path: str
    commit_sha: str
    initialized: bool
    worktree_matches_index: bool
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("AcadosSubmoduleIdentity requires provenance capture")
        _validate_submodule(self)

    def to_dict(self) -> dict[str, Any]:
        return {
            "path": self.path,
            "commit_sha": self.commit_sha,
            "initialized": self.initialized,
            "worktree_matches_index": self.worktree_matches_index,
        }


def _validate_submodule(value: AcadosSubmoduleIdentity) -> None:
    relative_path(value.path, "Acados submodule path")
    require_git_sha(value.commit_sha, "Acados submodule commit")
    if type(value.initialized) is not bool or type(value.worktree_matches_index) is not bool:
        raise ProvenanceError("Acados submodule states must be bool")


@dataclass(frozen=True)
class DynamicLibraryIdentity:
    logical_name: str
    file: LinkedFileIdentity
    soname: str
    needed: tuple[str, ...]
    rpath: str | None
    runpath: str | None
    dynamic_section_sha256: str
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("DynamicLibraryIdentity requires provenance capture")
        _validate_dynamic_library(self)

    def to_dict(self) -> dict[str, Any]:
        return {
            "logical_name": self.logical_name,
            "file": self.file.to_dict(),
            "soname": self.soname,
            "needed": list(self.needed),
            "rpath": self.rpath,
            "runpath": self.runpath,
            "dynamic_section_sha256": self.dynamic_section_sha256,
        }


def _validate_dynamic_library(value: DynamicLibraryIdentity) -> None:
    require_text(value.logical_name, "dynamic-library logical name")
    if type(value.file) is not LinkedFileIdentity:
        raise ProvenanceError("dynamic-library file has the wrong identity type")
    require_text(value.soname, "dynamic-library SONAME")
    if type(value.needed) is not tuple or any(
        type(item) is not str or not item for item in value.needed
    ):
        raise ProvenanceError("dynamic-library NEEDED entries must be strings")
    if value.rpath is not None:
        require_text(value.rpath, "dynamic-library RPATH")
    if value.runpath is not None:
        require_text(value.runpath, "dynamic-library RUNPATH")
    require_digest(value.dynamic_section_sha256, "dynamic-section identity")


def _capture_dynamic_library(
    root: Path,
    name: str,
    readelf_tool: ToolIdentity,
) -> DynamicLibraryIdentity:
    file_identity = capture_linked_file(f"acados/lib/{name}", root / "lib" / name)
    probe = capture_command_probe(
        readelf_tool.executable,
        f"dynamic:{name}",
        ("-d", file_identity.resolved_path),
    )
    entries: dict[str, list[str]] = {
        "NEEDED": [],
        "SONAME": [],
        "RPATH": [],
        "RUNPATH": [],
    }
    for kind, entry in _DYNAMIC_ENTRY_RE.findall(probe.output_text):
        entries[kind].append(entry)
    if entries["SONAME"] != [ACADOS_LIBRARY_SONAMES[name]]:
        raise ProvenanceError(f"dynamic library {name} has an unexpected SONAME")
    if len(entries["RPATH"]) > 1 or len(entries["RUNPATH"]) > 1:
        raise ProvenanceError(f"dynamic library {name} has ambiguous runtime paths")
    return DynamicLibraryIdentity(
        logical_name=name,
        file=file_identity,
        soname=entries["SONAME"][0],
        needed=tuple(entries["NEEDED"]),
        rpath=entries["RPATH"][0] if entries["RPATH"] else None,
        runpath=entries["RUNPATH"][0] if entries["RUNPATH"] else None,
        dynamic_section_sha256=probe.output_raw_sha256,
        _construction_token=_TOKEN,
    )


@dataclass(frozen=True)
class AcadosInstallIdentity:
    install_root: Path
    source_repository_root: str
    source_head_sha: str
    source_tag: str
    source_worktree_clean: bool
    source_status_sha256: str
    commit_marker: str
    commit_marker_file: LinkedFileIdentity
    link_libs_file: LinkedFileIdentity
    link_libs_canonical_sha256: str
    interface_tree: SourceTreeIdentity
    include_tree: SourceTreeIdentity
    submodules: tuple[AcadosSubmoduleIdentity, ...]
    libraries: tuple[DynamicLibraryIdentity, ...]
    interface_source_binding_status: str
    tera_source_binding_status: str
    install_prefix_policy: str
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = ACADOS_IDENTITY_SCHEMA

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("AcadosInstallIdentity requires provenance capture")
        _validate_acados_install(self)

    def to_dict(self) -> dict[str, Any]:
        payload = _acados_payload(self)
        payload["semantic_identity"] = {"sha256": self.semantic_sha256}
        return payload


def _acados_payload(value: AcadosInstallIdentity) -> dict[str, Any]:
    return {
        "schema_version": value.schema_version,
        "install_root": str(value.install_root),
        "install_prefix_policy": value.install_prefix_policy,
        "source_repository": {
            "root": value.source_repository_root,
            "head_sha": value.source_head_sha,
            "exact_tag": value.source_tag,
            "worktree_clean": value.source_worktree_clean,
            "dirty_policy": GIT_DIRTY_POLICY,
            "status_porcelain_sha256": value.source_status_sha256,
        },
        "commit_marker": value.commit_marker,
        "commit_marker_file": value.commit_marker_file.to_dict(),
        "link_libs": {
            "file": value.link_libs_file.to_dict(),
            "canonical_json_sha256": value.link_libs_canonical_sha256,
        },
        "interface_source_binding_status": value.interface_source_binding_status,
        "tera_source_binding_status": value.tera_source_binding_status,
        "interface_tree": value.interface_tree.to_dict(),
        "include_tree": value.include_tree.to_dict(),
        "submodules": [item.to_dict() for item in value.submodules],
        "libraries": [item.to_dict() for item in value.libraries],
    }


def _validate_acados_install(value: AcadosInstallIdentity) -> None:
    if value.schema_version != ACADOS_IDENTITY_SCHEMA:
        raise ProvenanceError("Acados install schema drifted")
    if not isinstance(value.install_root, Path) or not value.install_root.is_absolute():
        raise ProvenanceError("Acados install root must be an absolute Path")
    require_absolute_text(value.source_repository_root, "Acados source root")
    require_git_sha(value.source_head_sha, "Acados source HEAD")
    if value.source_tag != ACADOS_REQUIRED_TAG:
        raise ProvenanceError("Acados exact source tag is not v0.5.4")
    if type(value.source_worktree_clean) is not bool:
        raise ProvenanceError("Acados source clean state must be bool")
    require_digest(value.source_status_sha256, "Acados source status identity")
    if SHORT_GIT_SHA_RE.fullmatch(value.commit_marker) is None:
        raise ProvenanceError("Acados commit marker must be a lowercase Git prefix")
    if not value.source_head_sha.startswith(value.commit_marker):
        raise ProvenanceError("Acados full source HEAD does not match its commit marker")
    if type(value.commit_marker_file) is not LinkedFileIdentity:
        raise ProvenanceError("Acados commit marker file has the wrong identity type")
    if type(value.link_libs_file) is not LinkedFileIdentity:
        raise ProvenanceError("Acados link-libs file has the wrong identity type")
    require_digest(value.link_libs_canonical_sha256, "Acados link-libs identity")
    require_source_tree(value.interface_tree)
    require_source_tree(value.include_tree)
    if type(value.submodules) is not tuple:
        raise ProvenanceError("Acados submodules must be an immutable tuple")
    paths = tuple(item.path for item in value.submodules)
    if paths != tuple(sorted(paths)) or len(paths) != len(set(paths)):
        raise ProvenanceError("Acados submodule identities must be uniquely sorted")
    by_path = {item.path: item for item in value.submodules}
    for item in value.submodules:
        _validate_submodule(item)
    for path in REQUIRED_ACADOS_SUBMODULES:
        item = by_path.get(path)
        if item is None or not item.initialized or not item.worktree_matches_index:
            raise ProvenanceError(f"required Acados submodule is not pinned: {path}")
    if type(value.libraries) is not tuple or tuple(
        item.logical_name for item in value.libraries
    ) != ACADOS_LIBRARY_NAMES:
        raise ProvenanceError("Acados library identities are incomplete or unordered")
    for item in value.libraries:
        if type(item) is not DynamicLibraryIdentity:
            raise ProvenanceError("Acados library has the wrong record type")
        _validate_dynamic_library(item)
    if value.interface_source_binding_status != "MATCHED_SOURCE_ROOT":
        raise ProvenanceError("Acados interface and installed libraries are not source-bound")
    if value.tera_source_binding_status != TERA_SOURCE_BINDING_STATUS:
        raise ProvenanceError("Tera binary/source binding status drifted")
    if value.install_prefix_policy != ACADOS_PREFIX_POLICY:
        raise ProvenanceError("Acados install-prefix identity policy drifted")
    require_digest(value.semantic_sha256, "Acados semantic identity")


def require_acados_install(value: Any) -> AcadosInstallIdentity:
    if type(value) is not AcadosInstallIdentity:
        raise ProvenanceError(
            "Acados install must have the exact AcadosInstallIdentity type"
        )
    _validate_acados_install(value)
    if sha256_json(_acados_payload(value)) != value.semantic_sha256:
        raise ProvenanceError("Acados install identity is inconsistent")
    return value


def _parse_submodules(payload: bytes) -> tuple[AcadosSubmoduleIdentity, ...]:
    try:
        lines = payload.decode("utf-8").splitlines()
    except UnicodeDecodeError as exc:
        raise ProvenanceError("Acados submodule status is not UTF-8") from exc
    records: list[AcadosSubmoduleIdentity] = []
    for line in lines:
        match = re.fullmatch(r"([ +\-U])([0-9a-f]{40}) ([^ ]+)(?: .*)?", line)
        if match is None:
            raise ProvenanceError(f"cannot parse Acados submodule status: {line!r}")
        marker, commit, path = match.groups()
        records.append(
            AcadosSubmoduleIdentity(
                path=path,
                commit_sha=commit,
                initialized=marker != "-",
                worktree_matches_index=marker == " ",
                _construction_token=_TOKEN,
            )
        )
    records.sort(key=lambda item: item.path)
    return tuple(records)


def _strict_json_bytes(payload: bytes, label: str) -> Any:
    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ProvenanceError(f"duplicate key in {label}: {key!r}")
            result[key] = value
        return result

    def reject_constant(value: str) -> None:
        raise ProvenanceError(f"non-finite number in {label}: {value}")

    try:
        return json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=reject_duplicates,
            parse_constant=reject_constant,
        )
    except ProvenanceError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProvenanceError(f"invalid JSON in {label}") from exc


def capture_acados_install(
    install_root: Path | str,
    git_tool: ToolIdentity,
    readelf_tool: ToolIdentity,
) -> AcadosInstallIdentity:
    checked_git = require_tool_identity(git_tool)
    checked_readelf = require_tool_identity(readelf_tool)
    if checked_git.role != "git" or checked_readelf.role != "readelf":
        raise ProvenanceError("Acados capture requires git and readelf tool identities")
    root = canonical_absolute_path(install_root, "Acados install root")
    marker_file = capture_linked_file(
        "acados/lib/git_commit_hash",
        root / "lib" / "git_commit_hash",
    )
    try:
        marker_payload = read_stable_regular_file(
            marker_file.resolved_path,
            label="Acados commit marker",
        )
        marker = marker_payload.decode("ascii").strip()
    except (IdentityError, UnicodeDecodeError) as exc:
        raise ProvenanceError("cannot read Acados commit marker") from exc
    if SHORT_GIT_SHA_RE.fullmatch(marker) is None:
        raise ProvenanceError("Acados commit marker is not a Git SHA prefix")
    source_root = Path(marker_file.resolved_path).parent.parent

    def source_snapshot() -> tuple[str, str, bytes, bytes]:
        head = decode_git(
            git_command(checked_git, source_root, ("rev-parse", "HEAD")),
            "Acados source HEAD",
        )
        tag = decode_git(
            git_command(
                checked_git,
                source_root,
                ("describe", "--tags", "--exact-match", "HEAD"),
            ),
            "Acados exact tag",
        )
        status = git_command(
            checked_git,
            source_root,
            ("status", "--porcelain=v1", "--untracked-files=all"),
        )
        submodule_status = git_command(
            checked_git,
            source_root,
            ("submodule", "status", "--recursive"),
        )
        return head, tag, status, submodule_status

    before = source_snapshot()
    source_head, source_tag, source_status, submodule_status = before
    submodules = _parse_submodules(submodule_status)
    try:
        acados_template = importlib.import_module("acados_template")
    except (ImportError, OSError) as exc:
        raise ProvenanceError("acados_template is required for provenance") from exc
    interface_root = module_file(acados_template, "acados_template").parent
    expected_interface_root = (
        source_root / "interfaces" / "acados_template" / "acados_template"
    ).resolve(strict=True)
    binding_status = (
        "MATCHED_SOURCE_ROOT"
        if interface_root == expected_interface_root
        else "UNRESOLVED_SOURCE_ROOT_MISMATCH"
    )
    interface_tree = capture_source_tree(
        interface_root,
        logical_root="ACADOS_TEMPLATE_PYTHON_AND_TEMPLATES",
        include=lambda relative: not relative.endswith((".pyc", ".pyo")),
        excluded_directories=("__pycache__",),
    )
    include_tree = capture_source_tree(
        root / "include",
        logical_root="ACADOS_INSTALLED_INCLUDE_TREE",
    )
    link_file = capture_linked_file(
        "acados/lib/link_libs.json",
        root / "lib" / "link_libs.json",
    )
    try:
        link_payload = read_stable_regular_file(
            link_file.resolved_path,
            label="Acados link_libs.json",
        )
    except IdentityError as exc:
        raise ProvenanceError(str(exc)) from exc
    link_value = _strict_json_bytes(link_payload, "Acados link_libs.json")
    libraries = tuple(
        _capture_dynamic_library(root, name, checked_readelf)
        for name in ACADOS_LIBRARY_NAMES
    )
    if before != source_snapshot():
        raise ProvenanceError("Acados source changed during provenance capture")
    values = {
        "install_root": root,
        "source_repository_root": str(source_root),
        "source_head_sha": source_head,
        "source_tag": source_tag,
        "source_worktree_clean": not source_status.splitlines(),
        "source_status_sha256": sha256_bytes(source_status),
        "commit_marker": marker,
        "commit_marker_file": marker_file,
        "link_libs_file": link_file,
        "link_libs_canonical_sha256": sha256_json(link_value),
        "interface_tree": interface_tree,
        "include_tree": include_tree,
        "submodules": submodules,
        "libraries": libraries,
        "interface_source_binding_status": binding_status,
        "tera_source_binding_status": TERA_SOURCE_BINDING_STATUS,
        "install_prefix_policy": ACADOS_PREFIX_POLICY,
    }
    provisional = AcadosInstallIdentity(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_TOKEN,
    )
    return AcadosInstallIdentity(
        **values,
        semantic_sha256=sha256_json(_acados_payload(provisional)),
        _construction_token=_TOKEN,
    )


__all__ = [
    "ACADOS_PREFIX_POLICY",
    "ACADOS_REQUIRED_TAG",
    "AcadosInstallIdentity",
    "AcadosSubmoduleIdentity",
    "DynamicLibraryIdentity",
    "capture_acados_install",
    "require_acados_install",
]
