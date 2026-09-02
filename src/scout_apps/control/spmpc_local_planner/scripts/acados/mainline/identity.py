"""Strict JSON and content-identity primitives for mainline contracts.

This module has no knowledge of Stage 0, Stage 1, layouts, manifests, or
artifacts.  Contract layers depend on it for byte identity and canonical JSON;
it must never import a higher-level mainline module.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import stat
from collections.abc import Mapping
from pathlib import Path
from typing import Any

SHA256_HEX_LENGTH = 64


class IdentityError(ValueError):
    """Raised when a JSON document or content identity is malformed."""


def read_stable_regular_file(path: Path | str, *, label: str) -> bytes:
    """Read one non-symlink regular file and reject a concurrent rewrite.

    Callers that intentionally accept a symbolic-link chain must resolve and
    record that chain first, then pass the final target to this function.
    """

    if not isinstance(path, (str, Path)):
        raise IdentityError(f"{label} path must be str or Path")
    candidate = Path(path)
    if candidate.is_absolute():
        for parent in candidate.parents:
            try:
                metadata = parent.lstat()
            except OSError as exc:
                raise IdentityError(
                    f"cannot inspect {label} directory component {parent}: {exc}"
                ) from exc
            if stat.S_ISLNK(metadata.st_mode):
                raise IdentityError(
                    f"{label} cannot traverse a symbolic-link directory: {parent}"
                )
            if not stat.S_ISDIR(metadata.st_mode):
                raise IdentityError(f"{label} parent is not a directory: {parent}")
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        descriptor = os.open(candidate, flags)
    except OSError as exc:
        raise IdentityError(f"cannot open {label} {candidate}: {exc}") from exc
    try:
        with os.fdopen(descriptor, "rb") as stream:
            before = os.fstat(stream.fileno())
            if not stat.S_ISREG(before.st_mode):
                raise IdentityError(f"{label} is not a regular file: {candidate}")
            path_before = os.stat(candidate, follow_symlinks=False)
            if (
                path_before.st_dev != before.st_dev
                or path_before.st_ino != before.st_ino
                or not stat.S_ISREG(path_before.st_mode)
            ):
                raise IdentityError(f"{label} path changed before read: {candidate}")
            payload = stream.read()
            after = os.fstat(stream.fileno())
            path_after = os.stat(candidate, follow_symlinks=False)
    except OSError as exc:
        raise IdentityError(f"cannot read {label} {candidate}: {exc}") from exc
    if (
        before.st_dev != after.st_dev
        or before.st_ino != after.st_ino
        or before.st_size != after.st_size
        or before.st_mtime_ns != after.st_mtime_ns
        or after.st_size != len(payload)
        or path_after.st_dev != after.st_dev
        or path_after.st_ino != after.st_ino
        or not stat.S_ISREG(path_after.st_mode)
    ):
        raise IdentityError(f"{label} changed while being read: {candidate}")
    return payload


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise IdentityError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def _reject_nonfinite_constant(value: str) -> None:
    raise IdentityError(f"non-finite JSON number is forbidden: {value}")


def read_strict_json(path: Path | str, *, label: str) -> tuple[Any, bytes]:
    """Read UTF-8 JSON while preserving the exact bytes used for identity."""

    if not isinstance(path, (str, Path)):
        raise IdentityError(f"{label} path must be str or Path")
    document_path = Path(path)
    try:
        payload = document_path.read_bytes()
        text = payload.decode("utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise IdentityError(f"cannot read {label} {document_path}: {exc}") from exc
    try:
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
    except IdentityError:
        raise
    except json.JSONDecodeError as exc:
        raise IdentityError(f"invalid JSON in {label} {document_path}: {exc}") from exc
    return value, payload


def require_sha256(value: Any, label: str) -> str:
    """Validate the lowercase hexadecimal representation of a SHA-256."""

    if type(value) is not str or len(value) != SHA256_HEX_LENGTH:
        raise IdentityError(f"{label} must be a 64-character SHA-256")
    if any(character not in "0123456789abcdef" for character in value):
        raise IdentityError(f"{label} must be lowercase hexadecimal")
    return value


def sha256_bytes(value: bytes) -> str:
    if type(value) is not bytes:
        raise IdentityError("SHA-256 byte input must have type bytes")
    return hashlib.sha256(value).hexdigest()


def _canonical_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if type(key) is not str:
                raise IdentityError("canonical mapping keys must be strings")
            if key in result:
                raise IdentityError(f"duplicate canonical key: {key}")
            result[key] = _canonical_value(item)
        return result
    if isinstance(value, (list, tuple)):
        return [_canonical_value(item) for item in value]
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise IdentityError("canonical JSON cannot contain NaN or infinity")
        return 0.0 if value == 0.0 else value
    raise IdentityError(f"unsupported canonical value: {type(value)!r}")


def canonical_json(value: Any) -> str:
    """Return the deterministic JSON representation used for identities."""

    return json.dumps(
        _canonical_value(value),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def sha256_json(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()
