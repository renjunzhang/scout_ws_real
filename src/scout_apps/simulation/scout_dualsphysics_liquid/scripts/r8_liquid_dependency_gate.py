#!/usr/bin/env python3
"""Acquire and verify one pinned DualSPHysics source tree without executing it.

This gate deliberately stops before build or binary execution.  The pinned
upstream commit is unsigned and contains prebuilt tools/libraries, so the P0
admission in this file is provenance and static inventory only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import stat
import subprocess
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_safety as safety  # noqa: E402


REPOSITORY_URL = "https://github.com/DualSPHysics/DualSPHysics.git"
PINNED_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
SCHEMA_VERSION = "smpcc-r8-liquid-dependency-manifest-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_DEPENDENCY_MANIFEST"
SOURCE_DIR_NAME = f"DualSPHysics_{PINNED_COMMIT}"
SHA1_RE = re.compile(r"^[0-9a-f]{40}$")
UTC_TIMESTAMP_RE = re.compile(
    r"^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{6}\+00:00$"
)
MAX_FUTURE_CLOCK_SKEW = timedelta(minutes=5)
LICENSE_NAMES = frozenset(("license", "license.txt", "license.md", "copying", "copyright"))
ACQUIRE_RECEIPT_RE = re.compile(
    r"^dependency_acquire_preflight_[0-9]{8}T[0-9]{6}Z\.json$"
)
GIT = Path("/usr/bin/git")
GIT_SAFE_OPTIONS = (
    "--no-replace-objects",
    "-c",
    "core.fsmonitor=false",
    "-c",
    "core.hooksPath=/dev/null",
    "-c",
    "credential.helper=",
)
MANIFEST_CORE_KEYS = frozenset(
    (
        "schema_version",
        "document_type",
        "created_utc",
        "development_only",
        "formal",
        "fidelity_validation_status",
        "physical_primary_eligible",
        "repository_url",
        "requested_commit",
        "resolved_commit",
        "git_tree_hash",
        "working_tree_clean",
        "commit_signature_status",
        "commit_cryptographically_authenticated",
        "submodule_status",
        "submodules_initialized",
        "source_path",
        "source_inventory_hash",
        "source_file_count",
        "source_total_file_bytes",
        "precompiled_elf_artifacts",
        "license_artifacts",
        "network_used_for",
        "upstream_code_executed",
        "upstream_precompiled_binaries_executed",
        "build_status",
        "solver_status",
        "gencase_status",
        "status",
    )
)
MANIFEST_KEYS = MANIFEST_CORE_KEYS | {"manifest_hash"}


class DependencyGateError(RuntimeError):
    """Pinned-source provenance failure."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise DependencyGateError(message)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def require_utc_timestamp(value: Any, field_name: str) -> None:
    require(isinstance(value, str), f"{field_name} must be a string")
    require(
        UTC_TIMESTAMP_RE.fullmatch(value) is not None,
        f"{field_name} must use YYYY-MM-DDTHH:MM:SS.ffffff+00:00",
    )
    try:
        parsed = datetime.fromisoformat(value)
    except ValueError as exc:
        raise DependencyGateError(f"{field_name} is not ISO-8601: {value}") from exc
    require(parsed.tzinfo is not None, f"{field_name} must include a timezone")
    require(parsed.utcoffset() == timezone.utc.utcoffset(parsed), f"{field_name} must be UTC")
    require(
        parsed <= datetime.now(timezone.utc) + MAX_FUTURE_CLOCK_SKEW,
        f"{field_name} is implausibly in the future",
    )


def hash_regular_file_nofollow(path: Path) -> Tuple[str, os.stat_result, bytes]:
    """Hash one regular file through an O_NOFOLLOW descriptor."""

    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    parent_fd = safety.open_directory_nofollow(path.parent)
    try:
        fd = os.open(path.name, flags, dir_fd=parent_fd)
    finally:
        os.close(parent_fd)
    digest = hashlib.sha256()
    with os.fdopen(fd, "rb") as stream:
        before = os.fstat(stream.fileno())
        require(stat.S_ISREG(before.st_mode), f"hash input is not a regular file: {path}")
        magic = stream.read(4)
        digest.update(magic)
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
        after = os.fstat(stream.fileno())
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            f"file changed while hashing: {path}",
        )
    return digest.hexdigest(), after, magic


def sha256_file(path: Path) -> str:
    return hash_regular_file_nofollow(path)[0]


def read_json_regular_nofollow(path: Path) -> Any:
    """Read JSON from one stable regular-file descriptor."""

    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    parent_fd = safety.open_directory_nofollow(path.parent)
    try:
        fd = os.open(path.name, flags, dir_fd=parent_fd)
    finally:
        os.close(parent_fd)
    with os.fdopen(fd, "rb") as stream:
        before = os.fstat(stream.fileno())
        require(stat.S_ISREG(before.st_mode), f"JSON input is not a regular file: {path}")
        payload = stream.read()
        after = os.fstat(stream.fileno())
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            f"JSON input changed while reading: {path}",
        )
    return json.loads(payload.decode("utf-8"))


def _clean_git_environment() -> Dict[str, str]:
    """Discard user/system Git configuration and prohibit interactive prompts."""

    return {
        "PATH": "/usr/bin:/bin",
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_SYSTEM": "/dev/null",
        "GIT_CONFIG_GLOBAL": "/dev/null",
        "GIT_TERMINAL_PROMPT": "0",
        "GIT_ASKPASS": "/bin/false",
        "GIT_LFS_SKIP_SMUDGE": "1",
        "GIT_NO_REPLACE_OBJECTS": "1",
        "GIT_OPTIONAL_LOCKS": "0",
        "GIT_CONFIG_COUNT": "3",
        "GIT_CONFIG_KEY_0": "core.fsmonitor",
        "GIT_CONFIG_VALUE_0": "false",
        "GIT_CONFIG_KEY_1": "core.hooksPath",
        "GIT_CONFIG_VALUE_1": "/dev/null",
        "GIT_CONFIG_KEY_2": "credential.helper",
        "GIT_CONFIG_VALUE_2": "",
    }


def git_command(*args: str) -> Tuple[str, ...]:
    return (str(GIT), *GIT_SAFE_OPTIONS, *args)


def _live_process_group_members(process_group_id: int) -> List[int]:
    live = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text(encoding="utf-8", errors="replace").rsplit(") ", 1)[1].split()
            state = fields[0]
            process_group = int(fields[2])
        except (FileNotFoundError, IndexError, PermissionError, ValueError):
            continue
        if process_group == process_group_id and state != "Z":
            live.append(int(entry.name))
    return sorted(live)


def run_checked(
    argv: Sequence[str],
    *,
    cwd: Optional[Path] = None,
    timeout_sec: float = 300.0,
) -> subprocess.CompletedProcess[str]:
    """Run one argv-only command with a bounded timeout and no shell."""

    require(argv and all(isinstance(item, str) and item for item in argv), "command argv is invalid")
    require(Path(argv[0]).is_absolute(), f"command executable must be an absolute path: {argv[0]}")
    process = subprocess.Popen(
        list(argv),
        cwd=None if cwd is None else str(cwd),
        env=_clean_git_environment(),
        shell=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout_sec)
    except subprocess.TimeoutExpired as exc:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            try:
                stdout, stderr = process.communicate(timeout=5.0)
            except subprocess.TimeoutExpired:
                stdout, stderr = "", ""
        finally:
            # The process-group kill is deliberately unconditional after the
            # TERM grace period.  communicate() can return because the direct
            # child exited and a TERM-ignoring descendant closed its stdio.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        try:
            tail_stdout, tail_stderr = process.communicate(timeout=5.0)
            stdout = stdout or tail_stdout
            stderr = stderr or tail_stderr
        except subprocess.TimeoutExpired as kill_exc:
            raise DependencyGateError(
                f"owned process group could not be reaped after timeout: {list(argv)!r}"
            ) from kill_exc
        deadline = time.monotonic() + 5.0
        live_members = _live_process_group_members(process.pid)
        while live_members and time.monotonic() < deadline:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            time.sleep(0.05)
            live_members = _live_process_group_members(process.pid)
        if live_members:
            raise DependencyGateError(
                f"owned process group still has live members after SIGKILL: {live_members}"
            )
        raise DependencyGateError(
            f"command timed out after {timeout_sec:.1f}s and its owned process group was stopped: {list(argv)!r}"
        ) from exc
    completed = subprocess.CompletedProcess(list(argv), process.returncode, stdout, stderr)
    if completed.returncode != 0:
        stdout = completed.stdout[-4000:]
        stderr = completed.stderr[-4000:]
        raise DependencyGateError(
            f"command failed rc={completed.returncode}: {list(argv)!r}\nstdout={stdout}\nstderr={stderr}"
        )
    return completed


def git_text(source: Path, *args: str, timeout_sec: float = 30.0) -> str:
    return run_checked(git_command("-C", str(source), *args), timeout_sec=timeout_sec).stdout.strip()


def dependency_paths(root: Optional[Path] = None) -> Dict[str, Path]:
    root = safety.validate_approved_root(
        safety.APPROVED_ROOT if root is None else root,
        require_exists=False,
    )
    paths = {
        "source": root / "dependency/source" / SOURCE_DIR_NAME,
        "partial": root / "scratch" / f"dependency-acquire-{PINNED_COMMIT}.partial",
        "manifest": root / "dependency/manifests" / f"DualSPHysics_{PINNED_COMMIT}.json",
    }
    for path in paths.values():
        safety.ensure_within_approved_root(path, root=root)
    return paths


def require_dependency_layout(root: Path) -> None:
    """Require the prepared fixed parents and reject post-prepare symlinks."""

    for relative in ("dependency/source", "dependency/manifests", "scratch"):
        directory = root / relative
        safety.ensure_within_approved_root(directory, root=root, require_exists=True)
        require(
            directory.is_dir() and not directory.is_symlink(),
            f"dependency layout directory is unsafe: {directory}",
        )


def _iter_source_entries(source: Path) -> Iterable[Tuple[str, Path]]:
    for parent, dirnames, filenames in os.walk(source, topdown=True, followlinks=False):
        parent_path = Path(parent)
        if parent_path == source:
            dirnames[:] = sorted(name for name in dirnames if name != ".git")
        else:
            dirnames[:] = sorted(dirnames)
        for name in sorted(filenames):
            path = parent_path / name
            if ".git" in path.relative_to(source).parts:
                continue
            yield path.relative_to(source).as_posix(), path
        for name in list(dirnames):
            path = parent_path / name
            if path.is_symlink():
                yield path.relative_to(source).as_posix(), path
                dirnames.remove(name)


def _safe_symlink_record(source: Path, relative: str, path: Path) -> Dict[str, Any]:
    target_text = os.readlink(path)
    resolved = path.resolve(strict=True)
    try:
        resolved_relative = resolved.relative_to(source.resolve())
    except ValueError as exc:
        raise DependencyGateError(f"upstream symlink escapes source tree: {relative} -> {target_text}") from exc
    require(".git" not in resolved_relative.parts, f"upstream symlink points into Git metadata: {relative} -> {target_text}")
    return {
        "path": relative,
        "type": "symlink",
        "target": target_text,
        "sha256": hashlib.sha256(target_text.encode("utf-8")).hexdigest(),
    }


def inventory_source(source: Path) -> Dict[str, Any]:
    """Hash every non-.git source entry and identify unexecuted ELF files."""

    require(source.is_dir() and not source.is_symlink(), f"source is not a real directory: {source}")
    records: List[Dict[str, Any]] = []
    elf_files: List[Dict[str, Any]] = []
    licenses: List[Dict[str, Any]] = []
    total_bytes = 0
    for relative, path in _iter_source_entries(source):
        if path.is_symlink():
            record = _safe_symlink_record(source, relative, path)
        else:
            file_hash, file_stat, magic = hash_regular_file_nofollow(path)
            size = file_stat.st_size
            total_bytes += size
            mode = stat.S_IMODE(file_stat.st_mode)
            record = {
                "path": relative,
                "type": "file",
                "size": size,
                "executable": bool(mode & (stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)),
                "sha256": file_hash,
            }
            if magic == b"\x7fELF":
                elf_files.append(
                    {
                        "path": relative,
                        "size": size,
                        "sha256": file_hash,
                        "execution_status": "NOT_EXECUTED_NOT_ADMITTED",
                    }
                )
            if path.name.lower() in LICENSE_NAMES or "license" in path.name.lower():
                licenses.append({"path": relative, "sha256": file_hash, "size": size})
        records.append(record)

    inventory_core = {
        "records": records,
        "file_count": len(records),
        "total_file_bytes": total_bytes,
    }
    return {
        "source_inventory_hash": safety.canonical_hash(inventory_core),
        "source_file_count": len(records),
        "source_total_file_bytes": total_bytes,
        "precompiled_elf_artifacts": elf_files,
        "license_artifacts": licenses,
    }


def verify_git_source(source: Path, *, timeout_sec: float = 60.0) -> Dict[str, Any]:
    source = Path(source)
    require(source.is_dir() and not source.is_symlink(), f"source directory is missing: {source}")
    resolved_commit = git_text(source, "rev-parse", "HEAD", timeout_sec=timeout_sec)
    require(resolved_commit == PINNED_COMMIT, f"resolved commit differs from pin: {resolved_commit}")
    tree_hash = git_text(source, "rev-parse", "HEAD^{tree}", timeout_sec=timeout_sec)
    require(SHA1_RE.fullmatch(tree_hash) is not None, f"invalid git tree hash: {tree_hash}")
    origin = git_text(source, "remote", "get-url", "origin", timeout_sec=timeout_sec)
    require(origin == REPOSITORY_URL, f"origin URL differs from pin: {origin}")
    status = git_text(source, "status", "--porcelain=v1", "--untracked-files=all", timeout_sec=timeout_sec)
    require(status == "", "pinned source working tree is not clean")
    commit_object = git_text(source, "cat-file", "-p", "HEAD", timeout_sec=timeout_sec)
    signature_present = any(line.startswith("gpgsig ") for line in commit_object.splitlines())
    submodule_status = git_text(source, "submodule", "status", "--recursive", timeout_sec=timeout_sec)
    submodule_lines = submodule_status.splitlines() if submodule_status else []
    require(
        all(line.startswith("-") for line in submodule_lines),
        "one or more upstream submodules are unexpectedly initialized or modified",
    )
    return {
        "repository_url": origin,
        "requested_commit": PINNED_COMMIT,
        "resolved_commit": resolved_commit,
        "git_tree_hash": tree_hash,
        "working_tree_clean": True,
        "commit_signature_status": ("PRESENT_NOT_VERIFIED" if signature_present else "N"),
        "commit_cryptographically_authenticated": False,
        "submodule_status": submodule_lines,
        "submodules_initialized": False,
    }


def build_manifest(source: Path, *, timeout_sec: float = 60.0) -> Dict[str, Any]:
    git_info = verify_git_source(source, timeout_sec=timeout_sec)
    inventory = inventory_source(source)
    git_info_after = verify_git_source(source, timeout_sec=timeout_sec)
    require(git_info_after == git_info, "Git source identity changed during inventory")
    core: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "created_utc": utc_now(),
        "development_only": True,
        "formal": False,
        "fidelity_validation_status": "UNVALIDATED",
        "physical_primary_eligible": False,
        **git_info,
        "source_path": str(source.resolve()),
        **inventory,
        "network_used_for": "exact_commit_git_fetch_only",
        "upstream_code_executed": False,
        "upstream_precompiled_binaries_executed": False,
        "build_status": "NOT_BUILT",
        "solver_status": "NOT_RUN",
        "gencase_status": "NOT_RUN_NOT_ADMITTED",
        "status": "PINNED_SOURCE_VERIFIED_DEVELOPMENT_ONLY",
    }
    require(set(core) == MANIFEST_CORE_KEYS, "generated dependency manifest key set is not frozen")
    return dict(core, manifest_hash=safety.canonical_hash(core))


def acquire(
    *,
    root: Optional[Path] = None,
    preflight_receipt: Path,
    manifest_output: Path,
    timeout_sec: float = 300.0,
) -> Dict[str, Any]:
    """Fetch one commit into a new partial directory, audit, then atomically publish."""

    root = safety.validate_approved_root(
        safety.APPROVED_ROOT if root is None else root,
        require_exists=True,
    )
    paths = dependency_paths(root)
    require_dependency_layout(root)
    manifest_output = safety.ensure_within_approved_root(manifest_output, root=root)
    require(manifest_output == paths["manifest"], f"manifest path must be fixed: {paths['manifest']}")
    preflight_receipt = safety.ensure_within_approved_root(preflight_receipt, root=root)
    require(preflight_receipt.parent == root / "dependency/manifests", "preflight receipt must be directly under dependency/manifests")
    require(ACQUIRE_RECEIPT_RE.fullmatch(preflight_receipt.name) is not None, "dependency acquisition receipt filename is invalid")
    require(not paths["source"].is_symlink(), f"pinned source path is a symlink: {paths['source']}")
    require(not paths["partial"].exists() and not paths["partial"].is_symlink(), f"partial acquisition already exists and requires audit: {paths['partial']}")
    require(not manifest_output.is_symlink(), f"dependency manifest path is a symlink: {manifest_output}")
    require(preflight_receipt != manifest_output, "preflight receipt and dependency manifest must differ")
    require(not preflight_receipt.exists() and not preflight_receipt.is_symlink(), f"preflight receipt already exists: {preflight_receipt}")
    source_exists = paths["source"].exists()
    manifest_exists = manifest_output.exists()
    require(not (manifest_exists and not source_exists), "orphan dependency manifest exists without its source")
    require(not (manifest_exists and source_exists), "dependency is already published; use read-only verify")
    recover_source_only = source_exists and not manifest_exists
    if recover_source_only:
        require(paths["source"].is_dir(), f"recoverable source path is not a directory: {paths['source']}")

    preflight = safety.build_preflight(
        estimated_case_bytes=2 * safety.GIB,
        root=root,
        require_simulation_stopped=True,
        require_gpu_idle=False,
    )
    require(preflight["status"] == "PASS", f"dependency acquisition safety preflight is NO-GO: {preflight['errors']}")
    receipt = dict(
        preflight,
        action="acquire-pinned-dualsphysics-source",
        repository_url=REPOSITORY_URL,
        requested_commit=PINNED_COMMIT,
        upstream_execution_forbidden=True,
        transaction_status=("RECOVERY_PREFLIGHT_SOURCE_ONLY" if recover_source_only else "PREFLIGHT_ONLY"),
        acquisition_committed=False,
    )
    receipt_core = dict(receipt)
    receipt_core.pop("receipt_hash", None)
    receipt["receipt_hash"] = safety.canonical_hash(receipt_core)
    safety.atomic_write_json_new(preflight_receipt, receipt, root=root)

    if recover_source_only:
        manifest = build_manifest(paths["source"], timeout_sec=timeout_sec)
        safety.atomic_write_json_new(manifest_output, manifest, root=root)
        return manifest

    require(safety.mkdir_exact(paths["partial"], root=root), f"partial path appeared unexpectedly: {paths['partial']}")
    try:
        run_checked(git_command("init", str(paths["partial"])), timeout_sec=30.0)
        run_checked(git_command("-C", str(paths["partial"]), "remote", "add", "origin", REPOSITORY_URL), timeout_sec=30.0)
        run_checked(
            git_command(
                "-C",
                str(paths["partial"]),
                "fetch",
                "--depth=1",
                "--no-tags",
                "origin",
                PINNED_COMMIT,
            ),
            timeout_sec=timeout_sec,
        )
        run_checked(
            git_command("-C", str(paths["partial"]), "checkout", "--detach", "FETCH_HEAD"),
            timeout_sec=timeout_sec,
        )
        run_checked(
            git_command("-C", str(paths["partial"]), "fsck", "--strict", "--no-reflogs"),
            timeout_sec=timeout_sec,
        )
        # Audit while still quarantined.  No upstream file is executed.
        manifest = build_manifest(paths["partial"], timeout_sec=timeout_sec)
        safety.rename_directory_noreplace(paths["partial"], paths["source"], root=root)
        manifest = dict(manifest, source_path=str(paths["source"].resolve()))
        manifest_core = dict(manifest)
        manifest_core.pop("manifest_hash", None)
        manifest["manifest_hash"] = safety.canonical_hash(manifest_core)
        safety.atomic_write_json_new(manifest_output, manifest, root=root)
        return manifest
    except Exception:
        # Fail closed and retain the partial directory for explicit audit.
        raise


def verify_existing(*, root: Optional[Path] = None) -> Dict[str, Any]:
    root = safety.validate_approved_root(
        safety.APPROVED_ROOT if root is None else root,
        require_exists=True,
    )
    paths = dependency_paths(root)
    require_dependency_layout(root)
    manifest = build_manifest(paths["source"])
    expected_path = paths["manifest"]
    require(expected_path.is_file() and not expected_path.is_symlink(), f"dependency manifest is missing: {expected_path}")
    stored = read_json_regular_nofollow(expected_path)
    require(isinstance(stored, dict), "stored dependency manifest is not a JSON object")
    require(set(stored) == MANIFEST_KEYS, "stored dependency manifest key set differs from the frozen schema")
    require_utc_timestamp(stored.get("created_utc"), "created_utc")
    stored_core = dict(stored)
    stored_manifest_hash = stored_core.pop("manifest_hash", None)
    require(
        stored_manifest_hash == safety.canonical_hash(stored_core),
        "stored dependency manifest hash is invalid",
    )
    for key in sorted(MANIFEST_KEYS.difference(("created_utc", "manifest_hash"))):
        require(stored.get(key) == manifest.get(key), f"stored dependency manifest differs at {key}")
    return {
        "document_type": "SMPCC_R8_LIQUID_DEPENDENCY_VERIFICATION",
        "created_utc": utc_now(),
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "manifest_path": str(expected_path),
        "manifest_file_hash": sha256_file(expected_path),
        "source_inventory_hash": manifest["source_inventory_hash"],
        "status": "PASS",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    acquire_parser = sub.add_parser("acquire", help="fetch and statically audit only the exact pinned commit")
    acquire_parser.add_argument("--preflight-receipt", type=Path, required=True)
    acquire_parser.add_argument("--manifest-output", type=Path, required=True)
    acquire_parser.add_argument("--timeout-sec", type=float, default=300.0)
    sub.add_parser("verify", help="read-only re-verification of the acquired source")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "acquire":
            require(30.0 <= args.timeout_sec <= 900.0, "timeout must be between 30 and 900 seconds")
            result = acquire(
                preflight_receipt=args.preflight_receipt,
                manifest_output=args.manifest_output,
                timeout_sec=args.timeout_sec,
            )
        elif args.command == "verify":
            result = verify_existing()
        else:
            raise DependencyGateError(f"unsupported command: {args.command}")
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
        return 0
    except (
        DependencyGateError,
        safety.LiquidSafetyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        subprocess.SubprocessError,
    ) as exc:
        print(f"R8_LIQUID_DEPENDENCY_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
