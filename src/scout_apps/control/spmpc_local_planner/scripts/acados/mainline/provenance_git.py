"""Git and selected repository-source provenance for Stage 3-D4."""

from __future__ import annotations

import os
import subprocess
from dataclasses import InitVar, dataclass
from pathlib import Path
from typing import Any

from .identity import sha256_bytes, sha256_json
from .provenance_common import (
    GIT_DIRTY_POLICY,
    REPOSITORY_LOCATION_POLICY,
    ProvenanceError,
    canonical_absolute_path,
    require_digest,
    require_git_sha,
    require_text,
)
from .provenance_files import (
    SourceTreeIdentity,
    ToolIdentity,
    capture_selected_source_files,
    require_source_tree,
    require_tool_identity,
)

MAINLINE_BRANCH = "spmpc-mainline"
MAINLINE_BASE_SHA = "e8d10325ac6138042fdb6e707192568a6e12cbbd"
REPOSITORY_IDENTITY_SCHEMA = "spmpc_mainline_repository_identity_v1"

_TOKEN = object()


def git_command(tool: ToolIdentity, root: Path, arguments: tuple[str, ...]) -> bytes:
    checked_tool = require_tool_identity(tool)
    if checked_tool.role != "git":
        raise ProvenanceError("Git command requires the captured git tool")
    environment = dict(os.environ)
    environment.update({"LC_ALL": "C", "LANG": "C", "GIT_OPTIONAL_LOCKS": "0"})
    try:
        completed = subprocess.run(
            [checked_tool.executable.resolved_path, "-C", str(root), *arguments],
            check=True,
            capture_output=True,
            timeout=15.0,
            env=environment,
        )
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        raise ProvenanceError(
            f"Git provenance command failed: {' '.join(arguments)}"
        ) from exc
    return completed.stdout


def decode_git(payload: bytes, label: str) -> str:
    try:
        value = payload.decode("utf-8").strip()
    except UnicodeDecodeError as exc:
        raise ProvenanceError(f"{label} is not UTF-8") from exc
    return require_text(value, label)


@dataclass(frozen=True)
class RepositoryIdentity:
    repository_root: Path
    branch: str
    head_sha: str
    base_sha: str
    worktree_clean: bool
    status_entry_count: int
    status_porcelain_sha256: str
    sources: SourceTreeIdentity
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = REPOSITORY_IDENTITY_SCHEMA
    dirty_policy: str = GIT_DIRTY_POLICY

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("RepositoryIdentity requires provenance capture")
        _validate_repository(self)

    def to_dict(self) -> dict[str, Any]:
        payload = _repository_payload(self)
        payload["semantic_identity"] = {"sha256": self.semantic_sha256}
        return payload


def _repository_payload(value: RepositoryIdentity) -> dict[str, Any]:
    return {
        "schema_version": value.schema_version,
        "repository_root": REPOSITORY_LOCATION_POLICY,
        "branch": value.branch,
        "head_sha": value.head_sha,
        "base_sha": value.base_sha,
        "worktree": {
            "clean": value.worktree_clean,
            "dirty_policy": value.dirty_policy,
            "status_entry_count": value.status_entry_count,
            "status_porcelain_sha256": value.status_porcelain_sha256,
        },
        "sources": value.sources.to_dict(),
    }


def _validate_repository(value: RepositoryIdentity) -> None:
    if value.schema_version != REPOSITORY_IDENTITY_SCHEMA:
        raise ProvenanceError("repository identity schema drifted")
    if value.dirty_policy != GIT_DIRTY_POLICY:
        raise ProvenanceError("repository dirty policy drifted")
    if not isinstance(value.repository_root, Path) or not value.repository_root.is_absolute():
        raise ProvenanceError("repository root must be an absolute Path")
    require_text(value.branch, "repository branch")
    require_git_sha(value.head_sha, "repository HEAD")
    require_git_sha(value.base_sha, "repository base SHA")
    if type(value.worktree_clean) is not bool:
        raise ProvenanceError("repository clean state must be bool")
    if type(value.status_entry_count) is not int or value.status_entry_count < 0:
        raise ProvenanceError("Git status entry count must be nonnegative")
    if value.worktree_clean != (value.status_entry_count == 0):
        raise ProvenanceError("Git clean state and status entry count disagree")
    require_digest(value.status_porcelain_sha256, "Git status identity")
    require_source_tree(value.sources)
    require_digest(value.semantic_sha256, "repository semantic identity")


def require_repository_identity(value: Any) -> RepositoryIdentity:
    if type(value) is not RepositoryIdentity:
        raise ProvenanceError("repository must have the exact RepositoryIdentity type")
    _validate_repository(value)
    if sha256_json(_repository_payload(value)) != value.semantic_sha256:
        raise ProvenanceError("repository semantic identity is inconsistent")
    return value


def capture_repository_identity(
    repository_root: Path | str,
    source_paths: tuple[str, ...],
    git_tool: ToolIdentity,
    *,
    expected_branch: str = MAINLINE_BRANCH,
    base_sha: str = MAINLINE_BASE_SHA,
) -> RepositoryIdentity:
    root = canonical_absolute_path(repository_root, "repository root")
    if not root.joinpath(".git").exists():
        raise ProvenanceError("repository root does not contain .git")
    require_git_sha(base_sha, "repository base SHA")
    actual_root = decode_git(
        git_command(git_tool, root, ("rev-parse", "--show-toplevel")),
        "Git root",
    )
    if actual_root != str(root):
        raise ProvenanceError("declared repository root differs from Git")

    def snapshot() -> tuple[str, str, bytes]:
        branch = decode_git(
            git_command(git_tool, root, ("symbolic-ref", "--short", "HEAD")),
            "Git branch",
        )
        head = decode_git(
            git_command(git_tool, root, ("rev-parse", "HEAD")),
            "Git HEAD",
        )
        status = git_command(
            git_tool,
            root,
            ("status", "--porcelain=v1", "--untracked-files=all"),
        )
        return branch, head, status

    before = snapshot()
    branch, head, status = before
    if branch != expected_branch:
        raise ProvenanceError(
            f"codegen branch must be {expected_branch!r}, got {branch!r}"
        )
    require_git_sha(head, "Git HEAD")
    git_command(git_tool, root, ("cat-file", "-e", f"{base_sha}^{{commit}}"))
    git_command(git_tool, root, ("merge-base", "--is-ancestor", base_sha, head))
    sources = capture_selected_source_files(
        root,
        source_paths,
        logical_root="MAINLINE_REPOSITORY_SELECTED_SOURCES",
    )
    if before != snapshot():
        raise ProvenanceError("repository changed while provenance was captured")
    sources_after = capture_selected_source_files(
        root,
        source_paths,
        logical_root="MAINLINE_REPOSITORY_SELECTED_SOURCES",
    )
    if sources_after.semantic_sha256 != sources.semantic_sha256:
        raise ProvenanceError("selected source bytes changed during provenance capture")
    if before != snapshot():
        raise ProvenanceError("repository changed after source identity capture")
    entries = status.splitlines()
    values = {
        "repository_root": root,
        "branch": branch,
        "head_sha": head,
        "base_sha": base_sha,
        "worktree_clean": not entries,
        "status_entry_count": len(entries),
        "status_porcelain_sha256": sha256_bytes(status),
        "sources": sources,
    }
    provisional = RepositoryIdentity(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_TOKEN,
    )
    return RepositoryIdentity(
        **values,
        semantic_sha256=sha256_json(_repository_payload(provisional)),
        _construction_token=_TOKEN,
    )


__all__ = [
    "MAINLINE_BASE_SHA",
    "MAINLINE_BRANCH",
    "RepositoryIdentity",
    "capture_repository_identity",
    "decode_git",
    "git_command",
    "require_repository_identity",
]
