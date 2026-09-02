"""Safe, root-independent inventory of one generated Acados output tree."""

from __future__ import annotations

import os
import stat
from dataclasses import InitVar, dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from .codegen_options import (
    ACADOS_JSON_FILENAME,
    GENERATED_HEADER_FILENAME,
    MODEL_CONTRACT_FILENAME,
)
from .identity import IdentityError, require_sha256, sha256_bytes, sha256_json
from .model_contract import MODEL_ID

GENERATED_FILE_RECORD_SCHEMA = "spmpc_mainline_generated_file_record_v1"
GENERATED_TREE_IDENTITY_SCHEMA = "spmpc_mainline_generated_tree_identity_v1"
GENERATED_TREE_IDENTITY_SCOPE = "SORTED_RELATIVE_PATH_SIZE_AND_RAW_BYTES_SHA256"
SOLVER_LIBRARY_ROLE = "SOLVER_SHARED_LIBRARY"
ACADOS_JSON_ROLE = "ACADOS_OCP_JSON"
BUILD_RECIPE_ROLE = "BUILD_RECIPE"
GENERATED_C_SOURCE_ROLE = "GENERATED_C_SOURCE"
GENERATED_C_HEADER_ROLE = "GENERATED_C_HEADER"
BUILD_OBJECT_ROLE = "BUILD_OBJECT"
BUILD_OUTPUT_ROLE = "BUILD_OUTPUT"
SOLVER_LIBRARY_BASENAMES = (
    f"libacados_ocp_solver_{MODEL_ID}.so",
    f"libacados_ocp_solver_{MODEL_ID}.dylib",
    f"acados_ocp_solver_{MODEL_ID}.dll",
)
EXCLUDED_ROOT_FILES = (
    MODEL_CONTRACT_FILENAME,
    GENERATED_HEADER_FILENAME,
)

_RECORD_TOKEN = object()


class ArtifactFilesError(ValueError):
    """The generated tree is unsafe, incomplete, or differs from its inventory."""


def _relative_path(value: Any) -> str:
    if type(value) is not str or not value:
        raise ArtifactFilesError("artifact relative path must be a non-empty string")
    if "\\" in value:
        raise ArtifactFilesError("artifact relative path must use POSIX separators")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise ArtifactFilesError("artifact relative path must stay below its root")
    if path.as_posix() != value:
        raise ArtifactFilesError("artifact relative path is not canonical")
    return value


@dataclass(frozen=True)
class GeneratedFileRecord:
    """Immutable identity of one regular file below the output directory."""

    relative_path: str
    size_bytes: int
    raw_sha256: str
    role: str
    _construction_token: InitVar[object] = None
    schema_version: str = GENERATED_FILE_RECORD_SCHEMA

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _RECORD_TOKEN:
            raise ArtifactFilesError(
                "GeneratedFileRecord requires inventory_generated_tree"
            )
        if self.schema_version != GENERATED_FILE_RECORD_SCHEMA:
            raise ArtifactFilesError("generated file record schema drifted")
        _relative_path(self.relative_path)
        if type(self.size_bytes) is not int or self.size_bytes < 0:
            raise ArtifactFilesError(
                "generated file size must be a nonnegative integer"
            )
        if type(self.role) is not str or not self.role:
            raise ArtifactFilesError("generated file role must be a non-empty string")
        try:
            require_sha256(self.raw_sha256, "generated file raw_sha256")
        except IdentityError as exc:
            raise ArtifactFilesError(str(exc)) from exc

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "relative_path": self.relative_path,
            "size_bytes": self.size_bytes,
            "raw_sha256": self.raw_sha256,
            "role": self.role,
        }


def _role(relative_path: str) -> str:
    path = PurePosixPath(relative_path)
    if relative_path == ACADOS_JSON_FILENAME:
        return ACADOS_JSON_ROLE
    if relative_path == "Makefile":
        return BUILD_RECIPE_ROLE
    if relative_path in SOLVER_LIBRARY_BASENAMES:
        return SOLVER_LIBRARY_ROLE
    if path.suffix == ".c":
        return GENERATED_C_SOURCE_ROLE
    if path.suffix == ".h":
        return GENERATED_C_HEADER_ROLE
    if path.suffix == ".o":
        return BUILD_OBJECT_ROLE
    return BUILD_OUTPUT_ROLE


def _read_regular_file(path: Path) -> bytes:
    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise ArtifactFilesError(f"cannot read generated file {path}: {exc}") from exc
    try:
        with os.fdopen(descriptor, "rb") as stream:
            before = os.fstat(stream.fileno())
            if not stat.S_ISREG(before.st_mode):
                raise ArtifactFilesError(
                    f"generated tree contains a non-regular entry: {path}"
                )
            payload = stream.read()
            after = os.fstat(stream.fileno())
    except OSError as exc:
        raise ArtifactFilesError(f"cannot read generated file {path}: {exc}") from exc
    if (
        before.st_dev != after.st_dev
        or before.st_ino != after.st_ino
        or before.st_size != after.st_size
        or before.st_mtime_ns != after.st_mtime_ns
        or after.st_size != len(payload)
    ):
        raise ArtifactFilesError(f"generated file changed while hashing: {path}")
    return payload


def _record(root: Path, path: Path) -> GeneratedFileRecord:
    try:
        relative_path = path.relative_to(root).as_posix()
    except ValueError as exc:
        raise ArtifactFilesError(f"generated file escapes its root: {path}") from exc
    payload = _read_regular_file(path)
    return GeneratedFileRecord(
        relative_path=_relative_path(relative_path),
        size_bytes=len(payload),
        raw_sha256=sha256_bytes(payload),
        role=_role(relative_path),
        _construction_token=_RECORD_TOKEN,
    )


def _checked_root(value: Path | str) -> Path:
    if not isinstance(value, (str, Path)):
        raise ArtifactFilesError("generated tree root must be str or Path")
    candidate = Path(value)
    if not candidate.is_absolute():
        raise ArtifactFilesError("generated tree root must be an absolute path")
    if ".." in candidate.parts:
        raise ArtifactFilesError("generated tree root cannot contain parent traversal")
    current = Path(candidate.anchor)
    try:
        for part in candidate.parts[1:]:
            current /= part
            if stat.S_ISLNK(current.lstat().st_mode):
                raise ArtifactFilesError(
                    f"generated tree path cannot contain symbolic links: {current}"
                )
        root = candidate.resolve(strict=True)
    except OSError as exc:
        raise ArtifactFilesError(
            f"generated tree root cannot be resolved: {exc}"
        ) from exc
    if not root.is_dir():
        raise ArtifactFilesError("generated tree root must be a directory")
    return root


def _regular_files(root: Path) -> tuple[Path, ...]:
    files: list[Path] = []
    pending = [root]
    while pending:
        directory = pending.pop()
        try:
            with os.scandir(directory) as entries:
                scanned = list(entries)
        except OSError as exc:
            raise ArtifactFilesError(
                f"cannot enumerate generated directory {directory}: {exc}"
            ) from exc
        for entry in scanned:
            path = Path(entry.path)
            try:
                metadata = entry.stat(follow_symlinks=False)
            except OSError as exc:
                raise ArtifactFilesError(
                    f"cannot inspect generated tree entry {path}: {exc}"
                ) from exc
            if stat.S_ISLNK(metadata.st_mode):
                raise ArtifactFilesError(
                    f"generated tree cannot contain symbolic links: {path}"
                )
            if stat.S_ISDIR(metadata.st_mode):
                pending.append(path)
            elif stat.S_ISREG(metadata.st_mode):
                files.append(path)
            else:
                raise ArtifactFilesError(
                    f"generated tree contains a non-regular entry: {path}"
                )
    return tuple(sorted(files, key=lambda path: path.relative_to(root).as_posix()))


def inventory_generated_tree(root: Path | str) -> tuple[GeneratedFileRecord, ...]:
    """Hash every generated/build file except the non-circular contract outputs."""

    checked_root = _checked_root(root)
    records: list[GeneratedFileRecord] = []
    for path in _regular_files(checked_root):
        relative_path = path.relative_to(checked_root).as_posix()
        if relative_path in EXCLUDED_ROOT_FILES:
            continue
        records.append(_record(checked_root, path))
    records.sort(key=lambda item: item.relative_path)
    if len({item.relative_path for item in records}) != len(records):
        raise ArtifactFilesError("generated file inventory contains duplicate paths")
    if sum(item.role == ACADOS_JSON_ROLE for item in records) != 1:
        raise ArtifactFilesError(
            "generated tree must contain the canonical Acados JSON"
        )
    if sum(item.role == BUILD_RECIPE_ROLE for item in records) != 1:
        raise ArtifactFilesError(
            "generated tree must contain exactly one root Makefile"
        )
    if sum(item.role == SOLVER_LIBRARY_ROLE for item in records) != 1:
        raise ArtifactFilesError(
            "generated tree must contain exactly one canonical solver shared library"
        )
    return tuple(records)


def generated_file_records_from_dict(value: Any) -> tuple[GeneratedFileRecord, ...]:
    """Strictly reconstruct a serialized, sorted file inventory."""

    if type(value) is not list or not value:
        raise ArtifactFilesError("generated file inventory must be a non-empty array")
    records: list[GeneratedFileRecord] = []
    expected_keys = {
        "schema_version",
        "relative_path",
        "size_bytes",
        "raw_sha256",
        "role",
    }
    for item in value:
        if type(item) is not dict or set(item) != expected_keys:
            raise ArtifactFilesError("generated file record keys do not match schema")
        record = GeneratedFileRecord(
            relative_path=_relative_path(item["relative_path"]),
            size_bytes=item["size_bytes"],
            raw_sha256=item["raw_sha256"],
            role=item["role"],
            schema_version=item["schema_version"],
            _construction_token=_RECORD_TOKEN,
        )
        if record.role != _role(record.relative_path):
            raise ArtifactFilesError("generated file role does not match its path")
        records.append(record)
    if tuple(item.relative_path for item in records) != tuple(
        sorted(item.relative_path for item in records)
    ):
        raise ArtifactFilesError("generated file inventory must be path-sorted")
    if len({item.relative_path for item in records}) != len(records):
        raise ArtifactFilesError("generated file inventory contains duplicate paths")
    if sum(item.role == ACADOS_JSON_ROLE for item in records) != 1:
        raise ArtifactFilesError("generated inventory has no canonical Acados JSON")
    if sum(item.role == BUILD_RECIPE_ROLE for item in records) != 1:
        raise ArtifactFilesError("generated inventory has no canonical Makefile")
    if sum(item.role == SOLVER_LIBRARY_ROLE for item in records) != 1:
        raise ArtifactFilesError("generated inventory has no unique solver library")
    return tuple(records)


def generated_tree_sha256(records: tuple[GeneratedFileRecord, ...]) -> str:
    """Hash an inventory without including its machine-specific absolute root."""

    checked = generated_file_records_from_dict([item.to_dict() for item in records])
    return sha256_json(
        {
            "schema_version": GENERATED_TREE_IDENTITY_SCHEMA,
            "scope": GENERATED_TREE_IDENTITY_SCOPE,
            "files": [item.to_dict() for item in checked],
        }
    )


def solver_library_record(
    records: tuple[GeneratedFileRecord, ...],
) -> GeneratedFileRecord:
    """Return the sole deployed solver library from a validated inventory."""

    checked = generated_file_records_from_dict([item.to_dict() for item in records])
    matches = [item for item in checked if item.role == SOLVER_LIBRARY_ROLE]
    if len(matches) != 1:
        raise ArtifactFilesError("generated inventory has no unique solver library")
    return matches[0]


def validate_generated_tree(
    root: Path | str,
    expected: tuple[GeneratedFileRecord, ...],
) -> None:
    """Fail if a generated file is missing, modified, added, or replaced by a link."""

    checked_expected = generated_file_records_from_dict(
        [item.to_dict() for item in expected]
    )
    if inventory_generated_tree(root) != checked_expected:
        raise ArtifactFilesError("generated tree differs from its recorded inventory")


__all__ = [
    "ACADOS_JSON_ROLE",
    "BUILD_OBJECT_ROLE",
    "BUILD_OUTPUT_ROLE",
    "BUILD_RECIPE_ROLE",
    "EXCLUDED_ROOT_FILES",
    "GENERATED_C_HEADER_ROLE",
    "GENERATED_C_SOURCE_ROLE",
    "GENERATED_FILE_RECORD_SCHEMA",
    "GENERATED_TREE_IDENTITY_SCHEMA",
    "GENERATED_TREE_IDENTITY_SCOPE",
    "SOLVER_LIBRARY_BASENAMES",
    "SOLVER_LIBRARY_ROLE",
    "ArtifactFilesError",
    "GeneratedFileRecord",
    "generated_file_records_from_dict",
    "generated_tree_sha256",
    "inventory_generated_tree",
    "solver_library_record",
    "validate_generated_tree",
]
