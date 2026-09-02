"""Dependency-free validation primitives shared by D4 provenance layers."""

from __future__ import annotations

import re
from pathlib import Path, PurePosixPath
from typing import Any

from .identity import IdentityError, require_sha256

GIT_DIRTY_POLICY = "RECORDED_NOT_GATED"
REPOSITORY_LOCATION_POLICY = "ABSOLUTE_CAPTURE_ROOT_EXCLUDED_FROM_IDENTITY"

GIT_SHA_RE = re.compile(r"[0-9a-f]{40}")
SHORT_GIT_SHA_RE = re.compile(r"[0-9a-f]{7,40}")


class ProvenanceError(RuntimeError):
    """A source, executable, checkout, or installed dependency is ambiguous."""


def require_text(value: Any, label: str, *, nonempty: bool = True) -> str:
    if type(value) is not str or (nonempty and not value):
        qualifier = "a non-empty string" if nonempty else "a string"
        raise ProvenanceError(f"{label} must be {qualifier}")
    if "\x00" in value:
        raise ProvenanceError(f"{label} cannot contain NUL")
    return value


def require_absolute_text(value: Any, label: str) -> str:
    text = require_text(value, label)
    path = Path(text)
    if not path.is_absolute() or ".." in path.parts or str(path) != text:
        raise ProvenanceError(f"{label} must be an absolute canonical path")
    return text


def require_git_sha(value: Any, label: str) -> str:
    text = require_text(value, label)
    if GIT_SHA_RE.fullmatch(text) is None:
        raise ProvenanceError(f"{label} must be a lowercase 40-character Git SHA")
    return text


def require_digest(value: Any, label: str) -> str:
    try:
        return require_sha256(value, label)
    except IdentityError as exc:
        raise ProvenanceError(str(exc)) from exc


def canonical_absolute_path(value: Path | str, label: str) -> Path:
    if not isinstance(value, (str, Path)):
        raise ProvenanceError(f"{label} must be str or Path")
    candidate = Path(value)
    if not candidate.is_absolute() or ".." in candidate.parts:
        raise ProvenanceError(f"{label} must be an absolute path without '..'")
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise ProvenanceError(f"cannot resolve {label}: {exc}") from exc
    if resolved != candidate:
        raise ProvenanceError(f"{label} cannot contain symbolic-link components")
    return resolved


def relative_path(value: Any, label: str) -> str:
    text = require_text(value, label)
    if "\\" in text:
        raise ProvenanceError(f"{label} must use POSIX separators")
    path = PurePosixPath(text)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise ProvenanceError(f"{label} must remain below its declared root")
    if path.as_posix() != text:
        raise ProvenanceError(f"{label} is not canonical")
    return text


__all__ = [
    "GIT_DIRTY_POLICY",
    "GIT_SHA_RE",
    "REPOSITORY_LOCATION_POLICY",
    "SHORT_GIT_SHA_RE",
    "ProvenanceError",
    "canonical_absolute_path",
    "relative_path",
    "require_absolute_text",
    "require_digest",
    "require_git_sha",
    "require_text",
]
