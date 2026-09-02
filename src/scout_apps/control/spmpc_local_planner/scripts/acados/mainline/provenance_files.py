"""Stable file, source-tree, executable, and command-output identities."""

from __future__ import annotations

import os
import shlex
import shutil
import stat
import subprocess
from collections.abc import Callable
from dataclasses import InitVar, dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from .identity import IdentityError, read_stable_regular_file, sha256_bytes, sha256_json
from .provenance_common import (
    REPOSITORY_LOCATION_POLICY,
    ProvenanceError,
    canonical_absolute_path,
    relative_path,
    require_absolute_text,
    require_digest,
    require_no_symlink_directory_components,
    require_text,
)

FILE_IDENTITY_SCHEMA = "spmpc_mainline_linked_file_identity_v1"
TREE_IDENTITY_SCHEMA = "spmpc_mainline_source_tree_identity_v1"
TOOL_IDENTITY_SCHEMA = "spmpc_mainline_tool_identity_v1"

_TOKEN = object()


@dataclass(frozen=True)
class SymlinkHop:
    path: str
    target: str
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("SymlinkHop requires provenance capture")
        _validate_symlink_hop(self)

    def to_dict(self) -> dict[str, str]:
        return {"path": self.path, "target": self.target}


def _validate_symlink_hop(value: SymlinkHop) -> None:
    require_absolute_text(value.path, "symlink path")
    require_text(value.target, "symlink target")


@dataclass(frozen=True)
class LinkedFileIdentity:
    logical_name: str
    requested_path: str
    resolved_path: str
    leaf_symlink_chain: tuple[SymlinkHop, ...]
    size_bytes: int
    raw_sha256: str
    executable: bool
    _construction_token: InitVar[object] = None
    schema_version: str = FILE_IDENTITY_SCHEMA

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("LinkedFileIdentity requires provenance capture")
        _validate_linked_file(self)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "logical_name": self.logical_name,
            "requested_path": self.requested_path,
            "resolved_path": self.resolved_path,
            "leaf_symlink_chain": [item.to_dict() for item in self.leaf_symlink_chain],
            "size_bytes": self.size_bytes,
            "raw_sha256": self.raw_sha256,
            "executable": self.executable,
        }


def _validate_linked_file(value: LinkedFileIdentity) -> None:
    if value.schema_version != FILE_IDENTITY_SCHEMA:
        raise ProvenanceError("linked-file schema drifted")
    require_text(value.logical_name, "linked-file logical name")
    require_absolute_text(value.requested_path, "linked-file requested path")
    require_absolute_text(value.resolved_path, "linked-file resolved path")
    if type(value.leaf_symlink_chain) is not tuple:
        raise ProvenanceError("leaf symlink chain must be an immutable tuple")
    for item in value.leaf_symlink_chain:
        if type(item) is not SymlinkHop:
            raise ProvenanceError("leaf symlink chain contains a wrong record type")
        _validate_symlink_hop(item)
    if type(value.size_bytes) is not int or value.size_bytes < 0:
        raise ProvenanceError("linked-file size must be a nonnegative integer")
    require_digest(value.raw_sha256, "linked-file raw identity")
    if type(value.executable) is not bool:
        raise ProvenanceError("linked-file executable flag must be bool")


def capture_linked_file(
    logical_name: str,
    path: Path | str,
    *,
    executable: bool = False,
) -> LinkedFileIdentity:
    """Record a leaf symlink chain and hash its stable final regular file."""

    require_text(logical_name, "linked-file logical name")
    if not isinstance(path, (str, Path)):
        raise ProvenanceError("linked-file path must be str or Path")
    requested = Path(path)
    if not requested.is_absolute() or ".." in requested.parts:
        raise ProvenanceError("linked-file path must be absolute without '..'")
    requested = Path(os.path.normpath(str(requested)))
    # Only the final path component may be a symbolic link.  In particular,
    # do not let Path.lstat below silently follow a directory link while
    # looking up the requested leaf.
    require_no_symlink_directory_components(requested, "linked-file path")
    current = requested
    hops: list[SymlinkHop] = []
    visited: set[str] = set()
    for _ in range(32):
        current_text = str(current)
        if current_text in visited:
            raise ProvenanceError(f"symbolic-link cycle at {current}")
        visited.add(current_text)
        require_no_symlink_directory_components(current, "linked-file path")
        try:
            metadata = current.lstat()
        except OSError as exc:
            raise ProvenanceError(f"cannot inspect linked file {current}: {exc}") from exc
        if not stat.S_ISLNK(metadata.st_mode):
            break
        try:
            target = os.readlink(current)
        except OSError as exc:
            raise ProvenanceError(f"cannot read symbolic link {current}: {exc}") from exc
        hops.append(SymlinkHop(str(current), target, _construction_token=_TOKEN))
        next_path = Path(target) if Path(target).is_absolute() else current.parent / target
        current = Path(os.path.normpath(str(next_path)))
    else:
        raise ProvenanceError("symbolic-link chain exceeds 32 hops")
    require_no_symlink_directory_components(current, "linked-file path")
    try:
        resolved = current.resolve(strict=True)
        payload = read_stable_regular_file(resolved, label=logical_name)
    except (OSError, IdentityError) as exc:
        raise ProvenanceError(f"cannot identify linked-file target {current}: {exc}") from exc
    if executable and not os.access(resolved, os.X_OK):
        raise ProvenanceError(f"linked file is not executable: {resolved}")
    return LinkedFileIdentity(
        logical_name=logical_name,
        requested_path=str(requested),
        resolved_path=str(resolved),
        leaf_symlink_chain=tuple(hops),
        size_bytes=len(payload),
        raw_sha256=sha256_bytes(payload),
        executable=executable,
        _construction_token=_TOKEN,
    )


@dataclass(frozen=True)
class SourceFileIdentity:
    relative_path: str
    size_bytes: int
    raw_sha256: str
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("SourceFileIdentity requires provenance capture")
        _validate_source_file(self)

    def to_dict(self) -> dict[str, Any]:
        return {
            "relative_path": self.relative_path,
            "size_bytes": self.size_bytes,
            "raw_sha256": self.raw_sha256,
        }


def _validate_source_file(value: SourceFileIdentity) -> None:
    relative_path(value.relative_path, "source relative path")
    if type(value.size_bytes) is not int or value.size_bytes < 0:
        raise ProvenanceError("source size must be a nonnegative integer")
    require_digest(value.raw_sha256, "source raw identity")


@dataclass(frozen=True)
class SourceTreeIdentity:
    logical_root: str
    capture_root: Path
    files: tuple[SourceFileIdentity, ...]
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = TREE_IDENTITY_SCHEMA

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("SourceTreeIdentity requires provenance capture")
        _validate_source_tree(self)

    def to_dict(self) -> dict[str, Any]:
        payload = _source_tree_payload(self)
        payload["semantic_identity"] = {"sha256": self.semantic_sha256}
        return payload


def _source_tree_payload(value: SourceTreeIdentity) -> dict[str, Any]:
    return {
        "schema_version": value.schema_version,
        "logical_root": value.logical_root,
        "capture_root": REPOSITORY_LOCATION_POLICY,
        "files": [item.to_dict() for item in value.files],
    }


def _validate_source_tree(value: SourceTreeIdentity) -> None:
    if value.schema_version != TREE_IDENTITY_SCHEMA:
        raise ProvenanceError("source-tree schema drifted")
    require_text(value.logical_root, "source-tree logical root")
    if not isinstance(value.capture_root, Path) or not value.capture_root.is_absolute():
        raise ProvenanceError("source-tree capture root must be an absolute Path")
    if type(value.files) is not tuple or not value.files:
        raise ProvenanceError("source tree must contain an immutable non-empty inventory")
    paths: list[str] = []
    for item in value.files:
        if type(item) is not SourceFileIdentity:
            raise ProvenanceError("source tree contains a wrong record type")
        _validate_source_file(item)
        paths.append(item.relative_path)
    if paths != sorted(paths) or len(paths) != len(set(paths)):
        raise ProvenanceError("source-tree inventory must be uniquely sorted")
    require_digest(value.semantic_sha256, "source-tree semantic identity")


def require_source_tree(value: Any) -> SourceTreeIdentity:
    if type(value) is not SourceTreeIdentity:
        raise ProvenanceError("source tree must have the exact SourceTreeIdentity type")
    _validate_source_tree(value)
    if sha256_json(_source_tree_payload(value)) != value.semantic_sha256:
        raise ProvenanceError("source-tree semantic identity is inconsistent")
    return value


def _new_source_tree(
    logical_root: str,
    capture_root: Path,
    records: tuple[SourceFileIdentity, ...],
) -> SourceTreeIdentity:
    values = {
        "logical_root": logical_root,
        "capture_root": capture_root,
        "files": records,
    }
    provisional = SourceTreeIdentity(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_TOKEN,
    )
    return SourceTreeIdentity(
        **values,
        semantic_sha256=sha256_json(_source_tree_payload(provisional)),
        _construction_token=_TOKEN,
    )


def _source_record(root: Path, path: Path) -> SourceFileIdentity:
    try:
        relative = path.relative_to(root).as_posix()
        payload = read_stable_regular_file(path, label="source file")
    except (ValueError, IdentityError) as exc:
        raise ProvenanceError(f"cannot identify source file {path}: {exc}") from exc
    return SourceFileIdentity(
        relative_path=relative_path(relative, "source relative path"),
        size_bytes=len(payload),
        raw_sha256=sha256_bytes(payload),
        _construction_token=_TOKEN,
    )


def capture_selected_source_files(
    root: Path | str,
    relative_paths: tuple[str, ...],
    *,
    logical_root: str,
) -> SourceTreeIdentity:
    """Hash one explicit, sorted repository-relative source/config set."""

    checked_root = canonical_absolute_path(root, "selected-source root")
    if type(relative_paths) is not tuple or not relative_paths:
        raise ProvenanceError("selected source paths must be a non-empty tuple")
    canonical = tuple(relative_path(item, "selected source path") for item in relative_paths)
    if canonical != tuple(sorted(canonical)) or len(set(canonical)) != len(canonical):
        raise ProvenanceError("selected source paths must be uniquely sorted")
    records: list[SourceFileIdentity] = []
    for relative in canonical:
        current = checked_root
        metadata = None
        for part in PurePosixPath(relative).parts:
            current /= part
            try:
                metadata = current.lstat()
            except OSError as exc:
                raise ProvenanceError(f"cannot inspect selected source {current}: {exc}") from exc
            if stat.S_ISLNK(metadata.st_mode):
                raise ProvenanceError(f"selected source cannot traverse a symlink: {current}")
        if metadata is None or not stat.S_ISREG(metadata.st_mode):
            raise ProvenanceError(f"selected source is not a regular file: {current}")
        records.append(_source_record(checked_root, current))
    return _new_source_tree(logical_root, checked_root, tuple(records))


def capture_source_tree(
    root: Path | str,
    *,
    logical_root: str,
    include: Callable[[str], bool] | None = None,
    excluded_directories: tuple[str, ...] = (),
) -> SourceTreeIdentity:
    """Recursively inventory a source tree without following any symlink."""

    checked_root = canonical_absolute_path(root, "source-tree root")
    if type(excluded_directories) is not tuple or any(
        type(item) is not str or not item for item in excluded_directories
    ):
        raise ProvenanceError("excluded source directories must be a tuple of names")
    records: list[SourceFileIdentity] = []
    pending = [checked_root]
    while pending:
        directory = pending.pop()
        try:
            with os.scandir(directory) as entries:
                scanned = list(entries)
        except OSError as exc:
            raise ProvenanceError(f"cannot enumerate source tree {directory}: {exc}") from exc
        for entry in scanned:
            path = Path(entry.path)
            relative = path.relative_to(checked_root).as_posix()
            try:
                metadata = entry.stat(follow_symlinks=False)
            except OSError as exc:
                raise ProvenanceError(f"cannot inspect source-tree entry {path}: {exc}") from exc
            if stat.S_ISLNK(metadata.st_mode):
                raise ProvenanceError(f"source tree cannot contain symlinks: {path}")
            if stat.S_ISDIR(metadata.st_mode):
                if entry.name not in excluded_directories:
                    pending.append(path)
            elif stat.S_ISREG(metadata.st_mode):
                if include is None or include(relative):
                    records.append(_source_record(checked_root, path))
            else:
                raise ProvenanceError(f"source tree contains a non-regular entry: {path}")
    records.sort(key=lambda item: item.relative_path)
    if not records:
        raise ProvenanceError("source-tree inventory cannot be empty")
    return _new_source_tree(logical_root, checked_root, tuple(records))


@dataclass(frozen=True)
class CommandProbeIdentity:
    name: str
    arguments: tuple[str, ...]
    output_text: str
    output_raw_sha256: str
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("CommandProbeIdentity requires provenance capture")
        _validate_command_probe(self)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "arguments": list(self.arguments),
            "output_text": self.output_text,
            "output_raw_sha256": self.output_raw_sha256,
        }


def _validate_command_probe(value: CommandProbeIdentity) -> None:
    require_text(value.name, "tool probe name")
    if type(value.arguments) is not tuple or any(
        type(item) is not str or "\x00" in item for item in value.arguments
    ):
        raise ProvenanceError("tool probe arguments must be an immutable string tuple")
    require_text(value.output_text, "tool probe output")
    require_digest(value.output_raw_sha256, "tool probe output identity")


@dataclass(frozen=True)
class ToolIdentity:
    role: str
    requested_command: str
    executable: LinkedFileIdentity
    probes: tuple[CommandProbeIdentity, ...]
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = TOOL_IDENTITY_SCHEMA

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("ToolIdentity requires provenance capture")
        _validate_tool(self)

    def to_dict(self) -> dict[str, Any]:
        payload = _tool_payload(self)
        payload["semantic_identity"] = {"sha256": self.semantic_sha256}
        return payload


def _tool_payload(value: ToolIdentity) -> dict[str, Any]:
    return {
        "schema_version": value.schema_version,
        "role": value.role,
        "requested_command": value.requested_command,
        "executable": value.executable.to_dict(),
        "probes": [item.to_dict() for item in value.probes],
    }


def _validate_tool(value: ToolIdentity) -> None:
    if value.schema_version != TOOL_IDENTITY_SCHEMA:
        raise ProvenanceError("tool identity schema drifted")
    require_text(value.role, "tool role")
    require_text(value.requested_command, "requested tool command")
    if type(value.executable) is not LinkedFileIdentity:
        raise ProvenanceError("tool executable identity has the wrong type")
    _validate_linked_file(value.executable)
    if not value.executable.executable:
        raise ProvenanceError("tool identity must reference an executable file")
    if type(value.probes) is not tuple or not value.probes:
        raise ProvenanceError("tool probes must be an immutable non-empty tuple")
    names: list[str] = []
    for item in value.probes:
        if type(item) is not CommandProbeIdentity:
            raise ProvenanceError("tool probe has the wrong record type")
        _validate_command_probe(item)
        names.append(item.name)
    if len(names) != len(set(names)):
        raise ProvenanceError("tool probe names must be unique")
    require_digest(value.semantic_sha256, "tool semantic identity")


def require_tool_identity(value: Any) -> ToolIdentity:
    if type(value) is not ToolIdentity:
        raise ProvenanceError("tool must have the exact ToolIdentity type")
    _validate_tool(value)
    if sha256_json(_tool_payload(value)) != value.semantic_sha256:
        raise ProvenanceError("tool semantic identity is inconsistent")
    return value


def _resolve_command(requested_command: str) -> Path:
    command = require_text(requested_command, "requested command")
    try:
        parts = shlex.split(command)
    except ValueError as exc:
        raise ProvenanceError(f"cannot parse requested command {command!r}: {exc}") from exc
    if len(parts) != 1:
        raise ProvenanceError(
            f"requested command must name exactly one executable: {command!r}"
        )
    name = parts[0]
    if os.sep in name:
        path = Path(name)
        if not path.is_absolute():
            raise ProvenanceError("tool paths containing '/' must be absolute")
    else:
        found = shutil.which(name)
        if found is None:
            raise ProvenanceError(f"required tool is unavailable: {name}")
        path = Path(found)
        if not path.is_absolute():
            path = Path.cwd() / path
    return Path(os.path.normpath(str(path)))


def _require_tool_executable_unchanged(executable: LinkedFileIdentity) -> None:
    """Re-capture the executable and its complete leaf-link chain.

    The subprocess is launched using the first capture's resolved path, so a
    replacement of either the executable bytes or any leaf link must be
    detected on both sides of the probe.  A directory link is rejected by
    ``capture_linked_file`` before this comparison.
    """

    try:
        current = capture_linked_file(
            executable.logical_name,
            executable.requested_path,
            executable=executable.executable,
        )
    except ProvenanceError as exc:
        raise ProvenanceError(
            f"tool executable changed or became unavailable during probe: "
            f"{executable.logical_name}"
        ) from exc
    if current != executable:
        raise ProvenanceError(
            f"tool executable bytes or symlink chain changed during probe: "
            f"{executable.logical_name}"
        )


def capture_command_probe(
    executable: LinkedFileIdentity,
    name: str,
    arguments: tuple[str, ...],
) -> CommandProbeIdentity:
    _require_tool_executable_unchanged(executable)
    environment = dict(os.environ)
    environment.update({"LC_ALL": "C", "LANG": "C"})
    try:
        completed = subprocess.run(
            [executable.resolved_path, *arguments],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15.0,
            env=environment,
        )
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        raise ProvenanceError(
            f"tool probe failed for {executable.logical_name}/{name}"
        ) from exc
    payload = completed.stdout
    if type(payload) is not bytes or not payload or len(payload) > 1024 * 1024:
        raise ProvenanceError("tool probe output must contain at most 1 MiB")
    try:
        text = payload.decode("utf-8").strip()
    except UnicodeDecodeError as exc:
        raise ProvenanceError("tool probe output is not UTF-8") from exc
    require_text(text, "tool probe output")
    _require_tool_executable_unchanged(executable)
    return CommandProbeIdentity(
        name=name,
        arguments=arguments,
        output_text=text,
        output_raw_sha256=sha256_bytes(payload),
        _construction_token=_TOKEN,
    )


def capture_tool_identity(
    role: str,
    requested_command: str,
    probes: tuple[tuple[str, tuple[str, ...]], ...],
) -> ToolIdentity:
    path = _resolve_command(requested_command)
    executable = capture_linked_file(f"tool:{role}", path, executable=True)
    captured = tuple(
        capture_command_probe(executable, name, arguments)
        for name, arguments in probes
    )
    # Keep a final boundary check at the caller as well.  This catches a
    # replacement performed by a probe wrapper or a test double around the
    # low-level probe function, in addition to the checks surrounding the
    # real subprocess in capture_command_probe.
    _require_tool_executable_unchanged(executable)
    values = {
        "role": role,
        "requested_command": requested_command,
        "executable": executable,
        "probes": captured,
    }
    provisional = ToolIdentity(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_TOKEN,
    )
    return ToolIdentity(
        **values,
        semantic_sha256=sha256_json(_tool_payload(provisional)),
        _construction_token=_TOKEN,
    )


__all__ = [
    "CommandProbeIdentity",
    "LinkedFileIdentity",
    "SourceFileIdentity",
    "SourceTreeIdentity",
    "SymlinkHop",
    "ToolIdentity",
    "capture_command_probe",
    "capture_linked_file",
    "capture_selected_source_files",
    "capture_source_tree",
    "capture_tool_identity",
    "require_source_tree",
    "require_tool_identity",
]
