#!/usr/bin/env python3
"""Fail-closed exact-commit full-blob fetch for the MSI liquid workstation.

This is not a checkout or build tool.  It can create exactly one new bare Git
repository, fetch exactly one pinned commit over HTTPS, and verify that no Git
objects are missing.  It never invokes a source file, hook, submodule, LFS
smudge, CMake, Make, GenCase, solver, ROS, Gazebo or GPU program.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pwd
import re
import signal
import stat
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_profile_gate as profile_gate  # noqa: E402


POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_source_fetch_policy_v1.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_source_fetch_policy_v1.json"
FETCH_RECEIPT_RE = re.compile(r"^u2_full_source_fetch_attempt_3_[0-9]{8}T[0-9]{6}Z\.json$")
SHA1_RE = re.compile(r"^[0-9a-f]{40}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
GIT_PATH = Path("/usr/bin/git")
GIT_EXEC_PATH = Path("/usr/lib/git-core")
GIT_REMOTE_HTTPS_LINK = GIT_EXEC_PATH / "git-remote-https"
GIT_REMOTE_HTTP = GIT_EXEC_PATH / "git-remote-http"
EXPECTED_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
EXPECTED_TREE = "cef458cb358712f4694b9d2148f638440418e9dc"
EXPECTED_URL = "https://github.com/DualSPHysics/DualSPHysics.git"


class SourceFetchGateError(RuntimeError):
    """Any ambiguity is a fail-closed NO-GO."""


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_json_object(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise SourceFetchGateError(f"JSON file is missing or unsafe: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SourceFetchGateError(f"JSON file is not an object: {path}")
    return value


def _is_clean_absolute(path: Path) -> bool:
    return path.is_absolute() and all(part not in ("", ".", "..", "~") for part in path.parts[1:])


def _is_clean_relative(path: Path) -> bool:
    return not path.is_absolute() and bool(path.parts) and all(
        part not in ("", ".", "..", "~") for part in path.parts
    )


def _has_symlink_component(path: Path) -> bool:
    if not _is_clean_absolute(path):
        return True
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        if current.is_symlink():
            return True
        if not current.exists():
            break
    return False


def _sha256_regular_file(path: Path) -> str:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(path), flags)
    digest = hashlib.sha256()
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise SourceFetchGateError(f"hash input is not a regular file: {path}")
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        after = os.fstat(descriptor)
        if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
        ):
            raise SourceFetchGateError(f"file changed while hashing: {path}")
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def _trusted_git(expected: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(expected, Mapping):
        return {"trusted": False, "error": "Git declaration is invalid"}
    path = Path(str(expected.get("path", "")))
    evidence: dict[str, Any] = {"path": str(path)}
    try:
        if path != GIT_PATH or not _is_clean_absolute(path):
            raise SourceFetchGateError("Git path differs from the frozen system path")
        if not path.is_file() or path.is_symlink():
            raise SourceFetchGateError("Git is missing, unsafe, or a symlink")
        metadata = path.stat()
        observed_hash = _sha256_regular_file(path)
        evidence.update(
            {
                "uid": metadata.st_uid,
                "gid": metadata.st_gid,
                "mode_octal": f"{stat.S_IMODE(metadata.st_mode):04o}",
                "regular_file": stat.S_ISREG(metadata.st_mode),
                "executable": bool(metadata.st_mode & 0o111),
                "sha256": observed_hash,
                "expected_sha256": expected.get("sha256"),
            }
        )
        evidence["trusted"] = bool(
            metadata.st_uid == 0
            and metadata.st_gid == 0
            and stat.S_ISREG(metadata.st_mode)
            and (metadata.st_mode & 0o111)
            and observed_hash == expected.get("sha256")
        )
    except (OSError, SourceFetchGateError) as exc:
        evidence["trusted"] = False
        evidence["error"] = str(exc)
    return evidence


def _trusted_git_remote_https(expected: Mapping[str, Any]) -> dict[str, Any]:
    """Verify the only network transport helper Git may resolve for HTTPS."""

    if not isinstance(expected, Mapping):
        return {"trusted": False, "error": "Git HTTPS helper declaration is invalid"}
    link = Path(str(expected.get("path", "")))
    evidence: dict[str, Any] = {"path": str(link)}
    try:
        if GIT_EXEC_PATH.is_symlink() or not GIT_EXEC_PATH.is_dir() or _has_symlink_component(GIT_EXEC_PATH):
            raise SourceFetchGateError("Git exec-path is missing, unsafe, or traverses a symlink")
        exec_metadata = GIT_EXEC_PATH.stat()
        if exec_metadata.st_uid != 0 or exec_metadata.st_gid != 0:
            raise SourceFetchGateError("Git exec-path is not root-owned")
        if link != GIT_REMOTE_HTTPS_LINK or not link.is_symlink():
            raise SourceFetchGateError("Git HTTPS helper link differs from the frozen path")
        target = os.readlink(link)
        resolved = link.resolve(strict=True)
        if (
            target != expected.get("symlink_target")
            or resolved != GIT_REMOTE_HTTP
            or str(resolved) != expected.get("resolved_path")
            or not resolved.is_file()
            or resolved.is_symlink()
        ):
            raise SourceFetchGateError("Git HTTPS helper resolution differs from the frozen system helper")
        metadata = resolved.stat()
        observed_hash = _sha256_regular_file(resolved)
        evidence.update(
            {
                "symlink_target": target,
                "resolved_path": str(resolved),
                "uid": metadata.st_uid,
                "gid": metadata.st_gid,
                "mode_octal": f"{stat.S_IMODE(metadata.st_mode):04o}",
                "regular_file": stat.S_ISREG(metadata.st_mode),
                "executable": bool(metadata.st_mode & 0o111),
                "sha256": observed_hash,
                "expected_sha256": expected.get("sha256"),
            }
        )
        evidence["trusted"] = bool(
            metadata.st_uid == 0
            and metadata.st_gid == 0
            and stat.S_ISREG(metadata.st_mode)
            and (metadata.st_mode & 0o111)
            and observed_hash == expected.get("sha256")
        )
    except (OSError, RuntimeError, SourceFetchGateError) as exc:
        evidence["trusted"] = False
        evidence["error"] = str(exc)
    return evidence


def _trusted_git_suite(expected: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(expected, Mapping):
        return {"trusted": False, "error": "Git suite declaration is invalid"}
    git = _trusted_git(expected.get("git", {}))
    remote_https = _trusted_git_remote_https(expected.get("git_remote_https", {}))
    return {
        "git": git,
        "git_remote_https": remote_https,
        "trusted": bool(git.get("trusted") and remote_https.get("trusted")),
    }


def _clean_git_environment() -> dict[str, str]:
    """Use only system Git and its policy-local configuration, with no prompts."""

    return {
        "PATH": "/usr/bin:/bin",
        "GIT_EXEC_PATH": str(GIT_EXEC_PATH),
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "GIT_ALLOW_PROTOCOL": "https",
        "GIT_ASKPASS": "/bin/false",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_SYSTEM": "/dev/null",
        "GIT_CONFIG_GLOBAL": "/dev/null",
        "GIT_LFS_SKIP_SMUDGE": "1",
        "GIT_NO_LAZY_FETCH": "1",
        "GIT_NO_REPLACE_OBJECTS": "1",
        "GIT_OPTIONAL_LOCKS": "0",
        "GIT_TERMINAL_PROMPT": "0",
        "GIT_CONFIG_COUNT": "5",
        "GIT_CONFIG_KEY_0": "core.fsmonitor",
        "GIT_CONFIG_VALUE_0": "false",
        "GIT_CONFIG_KEY_1": "core.hooksPath",
        "GIT_CONFIG_VALUE_1": "/dev/null",
        "GIT_CONFIG_KEY_2": "credential.helper",
        "GIT_CONFIG_VALUE_2": "",
        "GIT_CONFIG_KEY_3": "protocol.file.allow",
        "GIT_CONFIG_VALUE_3": "never",
        "GIT_CONFIG_KEY_4": "fetch.fsckObjects",
        "GIT_CONFIG_VALUE_4": "true",
    }


def _git_argv(*args: str) -> tuple[str, ...]:
    return (str(GIT_PATH), "--no-replace-objects", *args)


def _stop_owned_group(process: subprocess.Popen[bytes], grace_seconds: int) -> None:
    """Stop only the freshly-created process group owned by this gate."""

    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + grace_seconds
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def _run_owned_bounded(
    argv: Sequence[str],
    *,
    timeout_seconds: int,
    grace_seconds: int,
    output_max_bytes: int,
    environment: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Run an exact trusted argv in its own group with bounded combined output."""

    if not argv or argv[0] != str(GIT_PATH):
        raise SourceFetchGateError("only the frozen system Git executable may be run")
    if output_max_bytes < 1:
        raise SourceFetchGateError("output cap must be positive")
    output = {"stdout": bytearray(), "stderr": bytearray()}
    output_lock = threading.Lock()
    output_limit_hit = threading.Event()
    readers_done = [threading.Event(), threading.Event()]
    process: subprocess.Popen[bytes] | None = None
    executor_error: str | None = None

    def reader(name: str, stream: Any, done: threading.Event) -> None:
        try:
            while True:
                block = stream.read(1024)
                if not block:
                    return
                with output_lock:
                    current = len(output["stdout"]) + len(output["stderr"])
                    remaining = output_max_bytes - current
                    if remaining > 0:
                        output[name].extend(block[:remaining])
                    if len(block) > remaining:
                        output_limit_hit.set()
                        return
        finally:
            done.set()

    started = time.monotonic()
    timed_out = False
    killed_for_output_limit = False
    try:
        process = subprocess.Popen(
            tuple(argv),
            cwd="/",
            env=dict(environment) if environment is not None else _clean_git_environment(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            close_fds=True,
        )
        if process.stdout is None or process.stderr is None:
            raise SourceFetchGateError("Git output pipes were not created")
        threads = (
            threading.Thread(target=reader, args=("stdout", process.stdout, readers_done[0]), daemon=True),
            threading.Thread(target=reader, args=("stderr", process.stderr, readers_done[1]), daemon=True),
        )
        for thread in threads:
            thread.start()
        deadline = started + timeout_seconds
        while process.poll() is None:
            if output_limit_hit.is_set():
                killed_for_output_limit = True
                _stop_owned_group(process, grace_seconds)
                break
            if time.monotonic() >= deadline:
                timed_out = True
                _stop_owned_group(process, grace_seconds)
                break
            time.sleep(0.05)
        try:
            process.wait(timeout=grace_seconds + 5)
        except subprocess.TimeoutExpired:
            timed_out = True
            _stop_owned_group(process, 0)
            process.wait(timeout=5)
        for stream, done, thread in (
            (process.stdout, readers_done[0], threads[0]),
            (process.stderr, readers_done[1], threads[1]),
        ):
            thread.join(timeout=1)
            if not done.is_set():
                stream.close()
                thread.join(timeout=1)
            if not done.is_set():
                raise SourceFetchGateError("Git output reader did not terminate")
    except (OSError, subprocess.SubprocessError, SourceFetchGateError) as exc:
        executor_error = str(exc)
        if process is not None and process.poll() is None:
            try:
                _stop_owned_group(process, grace_seconds)
                process.wait(timeout=grace_seconds + 5)
            except (OSError, subprocess.SubprocessError):
                pass

    with output_lock:
        stdout = bytes(output["stdout"])
        stderr = bytes(output["stderr"])
    return {
        "argv": list(argv),
        "returncode": process.returncode if process is not None else None,
        "hard_timeout_triggered": timed_out,
        "hard_kill_for_output_limit": killed_for_output_limit,
        "output_max_bytes": output_max_bytes,
        "stdout_bytes": len(stdout),
        "stderr_bytes": len(stderr),
        "stdout_sha256": hashlib.sha256(stdout).hexdigest(),
        "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
        "stdout_utf8": stdout.decode("utf-8", errors="replace"),
        "stderr_utf8": stderr.decode("utf-8", errors="replace"),
        "executor_error": executor_error,
        "elapsed_seconds": round(time.monotonic() - started, 6),
    }


def _git_text(args: Sequence[str], *, timeout_seconds: int, output_max_bytes: int) -> str:
    result = _run_owned_bounded(
        _git_argv(*args),
        timeout_seconds=timeout_seconds,
        grace_seconds=5,
        output_max_bytes=output_max_bytes,
    )
    if result["returncode"] != 0 or result["executor_error"] is not None:
        raise SourceFetchGateError(f"Git read-only query failed: {result}")
    if result["hard_timeout_triggered"] or result["hard_kill_for_output_limit"]:
        raise SourceFetchGateError(f"Git read-only query exceeded policy limits: {result}")
    return str(result["stdout_utf8"]).strip()


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate frozen semantic constants; no filesystem or process access occurs."""

    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-source-fetch-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_FETCH_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_SOURCE_FETCH_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "source_fetch_authorized": True,
        "upstream_code_execution_authorized": False,
        "allowed_gate_commands": ["self-check", "fetch-full-bare-source"],
    }
    for key, value in expected_top.items():
        if policy.get(key) != value:
            errors.append(f"policy field mismatch: {key}")

    profile = policy.get("required_target_profile")
    if not isinstance(profile, dict):
        errors.append("required target profile is missing or invalid")
    else:
        if profile.get("profile_id") != "LIQUID_ZRJ_MSI_U2404_P0_V1":
            errors.append("target profile ID mismatch")
        if profile.get("required_status") != "PASS_PROFILE_READ_ONLY_ONLY":
            errors.append("target profile status mismatch")
        if not SHA256_RE.fullmatch(str(profile.get("canonical_sha256", ""))):
            errors.append("target profile hash is invalid")

    bwrap = policy.get("required_bwrap_smoke")
    if not isinstance(bwrap, dict):
        errors.append("required bwrap smoke evidence is missing or invalid")
    else:
        expected_bwrap = {
            "receipt_relative_path": "audits/u2_bwrap_system_true_smoke_v3_20260806T072112Z.json",
            "file_sha256": "02de50da3e7e88b7dcc3150e06a51ccd06e9bd12978e61479cf2166deb5dec3b",
            "status": "PASS_SYSTEM_TRUE_BWRAP_SMOKE",
            "returncode": 0,
        }
        for key, value in expected_bwrap.items():
            if bwrap.get(key) != value:
                errors.append(f"bwrap smoke evidence mismatch: {key}")

    source = policy.get("source")
    expected_source = {
        "repository_url": EXPECTED_URL,
        "commit": EXPECTED_COMMIT,
        "tree": EXPECTED_TREE,
        "no_checkout": True,
        "no_submodules": True,
        "no_lfs_smudge": True,
        "no_hooks": True,
        "precompiled_binaries_execution": "forbidden",
    }
    if not isinstance(source, dict):
        errors.append("source is missing or invalid")
    else:
        for key, value in expected_source.items():
            if source.get(key) != value:
                errors.append(f"source field mismatch: {key}")
        for key in ("metadata_bare_relative_path", "full_bare_relative_path"):
            relative = Path(str(source.get(key, "")))
            if not _is_clean_relative(relative) or not str(relative).startswith("dependency/source/"):
                errors.append(f"unsafe source relative path: {key}")
        if source.get("metadata_bare_relative_path") == source.get("full_bare_relative_path"):
            errors.append("metadata and full bare source paths must differ")

    tools = policy.get("trusted_system_tools")
    if not isinstance(tools, dict) or set(tools) != {"git", "git_remote_https"}:
        errors.append("trusted tool set must contain the frozen Git HTTPS suite only")
    else:
        git = tools["git"]
        helper = tools["git_remote_https"]
        if not isinstance(git, dict) or git.get("path") != str(GIT_PATH) or not SHA256_RE.fullmatch(
            str(git.get("sha256", ""))
        ):
            errors.append("trusted Git declaration is invalid")
        if not isinstance(helper, dict) or helper.get("path") != str(GIT_REMOTE_HTTPS_LINK) or helper.get(
            "symlink_target"
        ) != "git-remote-http" or helper.get("resolved_path") != str(GIT_REMOTE_HTTP) or not SHA256_RE.fullmatch(
            str(helper.get("sha256", ""))
        ):
            errors.append("trusted Git HTTPS helper declaration is invalid")

    network = policy.get("network")
    if not isinstance(network, dict) or network != {
        "allowed_protocols": ["https"],
        "allowed_remote_url": EXPECTED_URL,
        "interactive_credentials": False,
        "network_use": "exact_commit_git_fetch_only",
    }:
        errors.append("network policy differs from the exact HTTPS-only fetch")

    commands = policy.get("git_commands")
    expected_commands = {
        "init": ["init", "--bare"],
        "remote_add": ["remote", "add", "origin", EXPECTED_URL],
        "fetch": ["fetch", "--quiet", "--no-tags", "--depth=1", "origin", EXPECTED_COMMIT],
        "fsck": ["fsck", "--full", "--strict", "--no-reflogs"],
        "object_walk": [
            "rev-list",
            "--objects",
            "--missing=print",
            "--no-object-names",
            EXPECTED_COMMIT,
        ],
    }
    if commands != expected_commands:
        errors.append("Git argv policy differs from the frozen exact fetch")

    resources = policy.get("resources")
    expected_resources = {
        "fetch_timeout_seconds": 900,
        "short_command_timeout_seconds": 60,
        "kill_grace_seconds": 5,
        "stdout_stderr_max_bytes": 16384,
        "object_walk_max_stdout_bytes": 131072,
        "object_walk_max_records": 4096,
        "object_walk_max_line_bytes": 128,
        "object_walk_missing_sample_limit": 16,
        "directory_mode_octal": "0750",
        "receipt_mode_octal": "0640",
    }
    if resources != expected_resources:
        errors.append("resource policy differs from the frozen values")

    receipt = policy.get("receipt")
    if not isinstance(receipt, dict) or receipt.get("directory_relative_path") != "audits":
        errors.append("receipt directory policy is invalid")
    elif receipt.get("filename_regex") != FETCH_RECEIPT_RE.pattern:
        errors.append("receipt filename policy is invalid")

    invariants = policy.get("invariants")
    required_invariants = {
        "no_sudo",
        "no_apt",
        "no_checkout",
        "no_source_file_execution",
        "no_precompiled_binary_execution",
        "no_cmake_or_make",
        "no_ros_or_gazebo",
        "no_gpu_device",
        "no_destructive_cleanup",
        "no_retry_in_existing_attempt_directory",
    }
    if not isinstance(invariants, dict) or any(invariants.get(key) is not True for key in required_invariants):
        errors.append("source-fetch safety invariants are incomplete")
    return errors


def _approved_root(profile: Mapping[str, Any], policy: Mapping[str, Any]) -> Path:
    root = Path(str(profile.get("storage", {}).get("approved_root", "")))
    if root != Path("/home/zrj/scout_liquid_lab"):
        raise SourceFetchGateError("target profile approved root differs from the MSI root")
    return root


def _relative_under_root(root: Path, text: str) -> Path:
    relative = Path(text)
    if not _is_clean_relative(relative):
        raise SourceFetchGateError(f"unsafe relative path: {text}")
    result = root / relative
    if _has_symlink_component(result):
        raise SourceFetchGateError(f"path traverses a symlink: {result}")
    return result


def _verify_bwrap_smoke(root: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    receipt = _relative_under_root(root, str(expected.get("receipt_relative_path", "")))
    if not receipt.is_file() or receipt.is_symlink():
        raise SourceFetchGateError("required bwrap smoke receipt is missing or unsafe")
    observed_hash = _sha256_regular_file(receipt)
    if observed_hash != expected.get("file_sha256"):
        raise SourceFetchGateError("required bwrap smoke receipt file hash mismatch")
    value = _read_json_object(receipt)
    execution = value.get("execution")
    if not isinstance(execution, dict):
        raise SourceFetchGateError("required bwrap smoke receipt lacks execution evidence")
    if value.get("status") != expected.get("status") or execution.get("returncode") != expected.get("returncode"):
        raise SourceFetchGateError("required bwrap smoke receipt status mismatch")
    return {
        "path": str(receipt),
        "sha256": observed_hash,
        "status": value["status"],
        "returncode": execution["returncode"],
    }


def _verify_metadata_bare(source: Path, policy: Mapping[str, Any]) -> dict[str, Any]:
    if _has_symlink_component(source) or source.is_symlink() or not source.is_dir():
        raise SourceFetchGateError("metadata bare repository is missing or unsafe")
    resources = policy["resources"]
    source_policy = policy["source"]
    tree = _git_text(
        ("--git-dir", str(source), "rev-parse", f"{source_policy['commit']}^{{tree}}"),
        timeout_seconds=resources["short_command_timeout_seconds"],
        output_max_bytes=resources["stdout_stderr_max_bytes"],
    )
    origin = _git_text(
        ("--git-dir", str(source), "remote", "get-url", "origin"),
        timeout_seconds=resources["short_command_timeout_seconds"],
        output_max_bytes=resources["stdout_stderr_max_bytes"],
    )
    if tree != source_policy["tree"] or origin != source_policy["repository_url"]:
        raise SourceFetchGateError("metadata bare repository identity differs from the frozen source")
    return {"path": str(source), "tree": tree, "origin": origin}


def build_preflight(policy: Mapping[str, Any]) -> dict[str, Any]:
    """Read-only source-fetch admission; it makes no network connection or write."""

    errors = validate_policy(policy)
    observations: dict[str, Any] = {}
    try:
        profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
        profile_hash = canonical_hash(profile)
        observations["target_profile"] = {
            "path": str(profile_gate.PROFILE_PATH),
            "profile_id": profile.get("profile_id"),
            "canonical_sha256": profile_hash,
        }
        if profile_hash != policy["required_target_profile"]["canonical_sha256"]:
            errors.append("target profile canonical hash differs from source-fetch policy")
        profile_report = profile_gate.build_report(profile)
        observations["target_profile"]["status"] = profile_report["status"]
        observations["target_profile"]["report_sha256"] = profile_report["report_sha256"]
        if profile_report["status"] != policy["required_target_profile"]["required_status"]:
            errors.append("target profile read-only self-check did not pass")

        root = _approved_root(profile, policy)
        observations["approved_root"] = str(root)
        if _has_symlink_component(root) or root.is_symlink() or not root.is_dir():
            errors.append("approved root is missing or unsafe")
        else:
            metadata = root.stat()
            owner = pwd.getpwnam(str(profile["storage"]["owner"]))
            observations["invoker"] = {"uid": os.getuid(), "euid": os.geteuid(), "gid": os.getgid()}
            if os.getuid() != owner.pw_uid or os.geteuid() != owner.pw_uid:
                errors.append("source fetch gate must run as the target ordinary user, never through sudo")
            if metadata.st_uid != owner.pw_uid or metadata.st_gid != owner.pw_gid:
                errors.append("approved root ownership differs from target profile")
            observations["required_bwrap_smoke"] = _verify_bwrap_smoke(root, policy["required_bwrap_smoke"])

            source_policy = policy["source"]
            metadata_bare = _relative_under_root(root, source_policy["metadata_bare_relative_path"])
            full_bare = _relative_under_root(root, source_policy["full_bare_relative_path"])
            observations["metadata_bare"] = _verify_metadata_bare(metadata_bare, policy)
            observations["full_bare_attempt"] = {
                "path": str(full_bare),
                "exists": full_bare.exists(),
                "is_symlink": full_bare.is_symlink(),
            }
            if full_bare.exists() or full_bare.is_symlink():
                errors.append("full bare attempt path already exists; retry or overwrite is forbidden")

        git_evidence = _trusted_git_suite(policy["trusted_system_tools"])
        observations["trusted_git_suite"] = git_evidence
        if not git_evidence.get("trusted"):
            errors.append("system Git HTTPS suite does not match the frozen root-owned hashes")
    except (
        KeyError,
        OSError,
        ValueError,
        profile_gate.TargetProfileError,
        SourceFetchGateError,
    ) as exc:
        errors.append(f"host inspection failed: {exc}")

    core = {
        "schema_version": "smpcc-r8-liquid-target-source-fetch-preflight-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_FETCH_PREFLIGHT",
        "captured_at_utc": utc_now(),
        "policy_path": str(POLICY_PATH),
        "policy_sha256": _sha256_regular_file(POLICY_PATH),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": _sha256_regular_file(SCHEMA_PATH),
        "policy_id": policy.get("policy_id"),
        "host_id": policy.get("host_id"),
        "status": "PASS_FULL_SOURCE_FETCH_ADMITTED" if not errors else "NO_GO_FULL_SOURCE_FETCH",
        "errors": errors,
        "observations": observations,
        "network_will_be_used_only_for": policy.get("network", {}).get("network_use"),
        "upstream_code_execution_authorized": False,
        "upstream_code_executed": False,
        "source_checkout_created": False,
        "precompiled_binary_executed": False,
        "next_allowed_stage": "FETCH_ONE_NEW_FULL_BARE_ATTEMPT" if not errors else "REVIEW_NO_GO",
    }
    return dict(core, preflight_sha256=canonical_hash(core))


def validate_receipt_path(path: Path, root: Path) -> tuple[Path, Path]:
    if not _is_clean_absolute(path) or not _is_clean_absolute(root):
        raise SourceFetchGateError("receipt and approved root must be clean absolute paths")
    audits = root / "audits"
    if _has_symlink_component(audits) or audits.is_symlink() or not audits.is_dir():
        raise SourceFetchGateError("audit directory is missing or unsafe")
    if path.parent != audits or FETCH_RECEIPT_RE.fullmatch(path.name) is None:
        raise SourceFetchGateError("receipt path is not the fixed source-fetch audit filename")
    partial = Path(f"{path}.partial")
    if path.exists() or path.is_symlink() or partial.exists() or partial.is_symlink():
        raise SourceFetchGateError("receipt final or partial path already exists")
    return path, partial


def _write_json_new(path: Path, value: Mapping[str, Any], mode: int = 0o640) -> str:
    encoded = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(path), flags, mode)
    try:
        os.fchmod(descriptor, mode)
        written = 0
        while written < len(encoded):
            written += os.write(descriptor, encoded[written:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return hashlib.sha256(encoded).hexdigest()


def _reserve_new_attempt_directory(path: Path, mode: int) -> dict[str, Any]:
    """Atomically reserve the one allowed attempt directory without reuse."""

    if _has_symlink_component(path.parent) or path.parent.is_symlink() or not path.parent.is_dir():
        raise SourceFetchGateError("full bare attempt parent is missing or unsafe")
    try:
        os.mkdir(path, mode)
    except FileExistsError as exc:
        raise SourceFetchGateError("full bare attempt path appeared during admission; reuse is forbidden") from exc
    except OSError as exc:
        raise SourceFetchGateError(f"could not reserve full bare attempt directory: {exc}") from exc

    flags = os.O_RDONLY | os.O_DIRECTORY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(str(path), flags)
    except OSError as exc:
        raise SourceFetchGateError(f"reserved attempt directory cannot be opened safely: {exc}") from exc
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISDIR(metadata.st_mode) or _has_symlink_component(path) or path.is_symlink():
            raise SourceFetchGateError("reserved attempt directory became unsafe")
        if metadata.st_uid != os.getuid() or metadata.st_gid != os.getgid():
            raise SourceFetchGateError("reserved attempt directory ownership differs from the ordinary invoker")
        os.fchmod(descriptor, mode)
        metadata = os.fstat(descriptor)
        if stat.S_IMODE(metadata.st_mode) != mode:
            raise SourceFetchGateError("reserved attempt directory mode differs from the frozen policy")
    finally:
        os.close(descriptor)
    return {
        "path": str(path),
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "uid": metadata.st_uid,
        "gid": metadata.st_gid,
        "mode_octal": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "created_new": True,
    }


def _scan_reachable_objects(source: Path, policy: Mapping[str, Any]) -> dict[str, Any]:
    """Stream a bounded, local object walk and reject even promisor omissions.

    ``git fsck`` intentionally tolerates promisor-object omissions.  The
    explicit ``rev-list --missing=print`` walk therefore provides the full
    blob-completeness assertion without checking out, opening, or executing a
    source file.  It retains only aggregate evidence and a small missing-ID
    sample, rather than an unbounded object listing in the receipt.
    """

    resources = policy["resources"]
    argv = _git_argv("--git-dir", str(source), *policy["git_commands"]["object_walk"])
    timeout_seconds = int(resources["short_command_timeout_seconds"])
    grace_seconds = int(resources["kill_grace_seconds"])
    stdout_max_bytes = int(resources["object_walk_max_stdout_bytes"])
    max_records = int(resources["object_walk_max_records"])
    max_line_bytes = int(resources["object_walk_max_line_bytes"])
    missing_sample_limit = int(resources["object_walk_missing_sample_limit"])
    stderr_max_bytes = int(resources["stdout_stderr_max_bytes"])
    if (
        timeout_seconds < 1
        or grace_seconds < 0
        or stdout_max_bytes < 1
        or max_records < 1
        or max_line_bytes < 41
        or missing_sample_limit < 1
        or stderr_max_bytes < 1
    ):
        raise SourceFetchGateError("object-walk resource limits are invalid")

    stdout_digest = hashlib.sha256()
    stderr = bytearray()
    stdout_state: dict[str, Any] = {
        "stdout_total_bytes": 0,
        "reachable_object_records": 0,
        "present_object_count": 0,
        "missing_object_count": 0,
        "missing_object_ids_sample": [],
        "malformed_record_count": 0,
        "malformed_record_sha256_sample": [],
        "expected_commit_seen": False,
        "expected_tree_seen": False,
    }
    stderr_lock = threading.Lock()
    reader_errors: list[str] = []
    reader_error_lock = threading.Lock()
    stdout_limit_hit = threading.Event()
    record_limit_hit = threading.Event()
    malformed_record_hit = threading.Event()
    stderr_limit_hit = threading.Event()
    process: subprocess.Popen[bytes] | None = None
    stdout_thread: threading.Thread | None = None
    stderr_thread: threading.Thread | None = None
    executor_error: str | None = None
    timed_out = False

    def remember_reader_error(label: str, exc: BaseException) -> None:
        with reader_error_lock:
            reader_errors.append(f"{label}: {exc}")

    def record_malformed(line: bytes) -> None:
        stdout_state["malformed_record_count"] += 1
        if len(stdout_state["malformed_record_sha256_sample"]) < missing_sample_limit:
            stdout_state["malformed_record_sha256_sample"].append(hashlib.sha256(line).hexdigest())
        malformed_record_hit.set()

    def consume_record(line: bytes) -> None:
        stdout_state["reachable_object_records"] += 1
        if stdout_state["reachable_object_records"] > max_records:
            record_limit_hit.set()
            return
        if len(line) > max_line_bytes:
            record_malformed(line)
            return
        try:
            text = line.decode("ascii")
        except UnicodeDecodeError:
            record_malformed(line)
            return
        if SHA1_RE.fullmatch(text):
            stdout_state["present_object_count"] += 1
            stdout_state["expected_commit_seen"] = bool(
                stdout_state["expected_commit_seen"] or text == EXPECTED_COMMIT
            )
            stdout_state["expected_tree_seen"] = bool(stdout_state["expected_tree_seen"] or text == EXPECTED_TREE)
            return
        if text.startswith("?") and SHA1_RE.fullmatch(text[1:]):
            stdout_state["missing_object_count"] += 1
            if len(stdout_state["missing_object_ids_sample"]) < missing_sample_limit:
                stdout_state["missing_object_ids_sample"].append(text[1:])
            return
        record_malformed(line)

    def read_stdout(stream: Any) -> None:
        pending = bytearray()
        try:
            while True:
                block = stream.read(4096)
                if not block:
                    break
                stdout_state["stdout_total_bytes"] += len(block)
                stdout_digest.update(block)
                if stdout_state["stdout_total_bytes"] > stdout_max_bytes:
                    stdout_limit_hit.set()
                    return
                pending.extend(block)
                while True:
                    newline_index = pending.find(b"\n")
                    if newline_index < 0:
                        break
                    line = bytes(pending[:newline_index])
                    del pending[: newline_index + 1]
                    consume_record(line)
                    if record_limit_hit.is_set() or malformed_record_hit.is_set():
                        return
                if len(pending) > max_line_bytes:
                    record_malformed(bytes(pending))
                    return
            if pending:
                record_malformed(bytes(pending))
        except (OSError, ValueError) as exc:
            remember_reader_error("stdout reader", exc)

    def read_stderr(stream: Any) -> None:
        try:
            while True:
                block = stream.read(1024)
                if not block:
                    return
                with stderr_lock:
                    remaining = stderr_max_bytes - len(stderr)
                    if remaining > 0:
                        stderr.extend(block[:remaining])
                    if len(block) > remaining:
                        stderr_limit_hit.set()
                        return
        except (OSError, ValueError) as exc:
            remember_reader_error("stderr reader", exc)

    started = time.monotonic()
    try:
        process = subprocess.Popen(
            tuple(argv),
            cwd="/",
            env=_clean_git_environment(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            close_fds=True,
        )
        if process.stdout is None or process.stderr is None:
            raise SourceFetchGateError("Git object-walk output pipes were not created")
        stdout_thread = threading.Thread(target=read_stdout, args=(process.stdout,), daemon=True)
        stderr_thread = threading.Thread(target=read_stderr, args=(process.stderr,), daemon=True)
        stdout_thread.start()
        stderr_thread.start()
        deadline = started + timeout_seconds
        while process.poll() is None:
            if (
                stdout_limit_hit.is_set()
                or record_limit_hit.is_set()
                or malformed_record_hit.is_set()
                or stderr_limit_hit.is_set()
            ):
                _stop_owned_group(process, grace_seconds)
                break
            if time.monotonic() >= deadline:
                timed_out = True
                _stop_owned_group(process, grace_seconds)
                break
            time.sleep(0.05)
        try:
            process.wait(timeout=grace_seconds + 5)
        except subprocess.TimeoutExpired:
            timed_out = True
            _stop_owned_group(process, 0)
            process.wait(timeout=5)
    except (OSError, subprocess.SubprocessError, SourceFetchGateError) as exc:
        executor_error = str(exc)
        if process is not None and process.poll() is None:
            try:
                _stop_owned_group(process, grace_seconds)
                process.wait(timeout=grace_seconds + 5)
            except (OSError, subprocess.SubprocessError):
                pass
    finally:
        for stream, thread in (
            (process.stdout if process is not None else None, stdout_thread),
            (process.stderr if process is not None else None, stderr_thread),
        ):
            if thread is None:
                continue
            thread.join(timeout=1)
            if thread.is_alive() and stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass
                thread.join(timeout=1)
            if thread.is_alive():
                reader_errors.append("object-walk output reader did not terminate")

    with stderr_lock:
        stderr_bytes = bytes(stderr)
    return {
        "argv": list(argv),
        "returncode": process.returncode if process is not None else None,
        "hard_timeout_triggered": timed_out,
        "hard_kill_for_stdout_limit": stdout_limit_hit.is_set(),
        "hard_kill_for_record_limit": record_limit_hit.is_set(),
        "hard_kill_for_malformed_record": malformed_record_hit.is_set(),
        "hard_kill_for_stderr_limit": stderr_limit_hit.is_set(),
        "object_walk_max_stdout_bytes": stdout_max_bytes,
        "object_walk_max_records": max_records,
        "object_walk_max_line_bytes": max_line_bytes,
        "object_walk_missing_sample_limit": missing_sample_limit,
        "stdout_sha256": stdout_digest.hexdigest(),
        "stderr_captured_bytes": len(stderr_bytes),
        "stderr_max_bytes": stderr_max_bytes,
        "stderr_sha256": hashlib.sha256(stderr_bytes).hexdigest(),
        "stderr_utf8": stderr_bytes.decode("utf-8", errors="replace"),
        "reader_errors": reader_errors,
        "executor_error": executor_error,
        "elapsed_seconds": round(time.monotonic() - started, 6),
        **stdout_state,
    }


def _verify_full_bare(source: Path, policy: Mapping[str, Any]) -> dict[str, Any]:
    """Verify identity and full reachable-object completeness without checkout."""

    source_policy = policy["source"]
    resources = policy["resources"]
    if _has_symlink_component(source) or source.is_symlink() or not source.is_dir():
        raise SourceFetchGateError("full bare repository is missing or unsafe")
    tree = _git_text(
        ("--git-dir", str(source), "rev-parse", f"{source_policy['commit']}^{{tree}}"),
        timeout_seconds=resources["short_command_timeout_seconds"],
        output_max_bytes=resources["stdout_stderr_max_bytes"],
    )
    if tree != source_policy["tree"]:
        raise SourceFetchGateError("full bare Git tree differs from the exact pin")
    origin = _git_text(
        ("--git-dir", str(source), "remote", "get-url", "origin"),
        timeout_seconds=resources["short_command_timeout_seconds"],
        output_max_bytes=resources["stdout_stderr_max_bytes"],
    )
    if origin != source_policy["repository_url"]:
        raise SourceFetchGateError("full bare Git origin differs from the frozen source")
    fsck = _run_owned_bounded(
        _git_argv("--git-dir", str(source), *policy["git_commands"]["fsck"]),
        timeout_seconds=resources["short_command_timeout_seconds"],
        grace_seconds=resources["kill_grace_seconds"],
        output_max_bytes=resources["stdout_stderr_max_bytes"],
    )
    if fsck["returncode"] != 0 or fsck["executor_error"] or fsck["hard_timeout_triggered"] or fsck["hard_kill_for_output_limit"]:
        raise SourceFetchGateError(f"strict Git fsck failed: {fsck}")
    object_walk = _scan_reachable_objects(source, policy)
    if (
        object_walk["returncode"] != 0
        or object_walk["executor_error"] is not None
        or object_walk["hard_timeout_triggered"]
        or object_walk["hard_kill_for_stdout_limit"]
        or object_walk["hard_kill_for_record_limit"]
        or object_walk["hard_kill_for_malformed_record"]
        or object_walk["hard_kill_for_stderr_limit"]
        or object_walk["reader_errors"]
    ):
        raise SourceFetchGateError(f"bounded reachable-object walk failed: {object_walk}")
    if not object_walk["expected_commit_seen"] or not object_walk["expected_tree_seen"]:
        raise SourceFetchGateError("reachable-object walk did not include the frozen commit and tree")
    if object_walk["missing_object_count"] != 0:
        raise SourceFetchGateError(
            f"full bare repository has {object_walk['missing_object_count']} missing reachable Git objects"
        )
    return {
        "tree": tree,
        "origin": origin,
        "fsck": fsck,
        "reachable_object_completeness": object_walk,
    }


def fetch_full_bare_source(policy: Mapping[str, Any], receipt_path: Path) -> dict[str, Any]:
    """Create one new full bare attempt and fetch only the pinned commit."""

    preflight = build_preflight(policy)
    profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
    root = _approved_root(profile, policy)
    final_path, partial_path = validate_receipt_path(receipt_path, root)
    if preflight["status"] != "PASS_FULL_SOURCE_FETCH_ADMITTED":
        receipt = {
            "schema_version": "smpcc-r8-liquid-target-source-fetch-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_FETCH_RECEIPT",
            "created_at_utc": utc_now(),
            "status": "NO_GO_STATIC_PRECHECK",
            "preflight": preflight,
            "network_used": False,
            "network_attempted": False,
            "network_fetch_completed": False,
            "upstream_code_executed": False,
            "source_checkout_created": False,
        }
        receipt["receipt_sha256"] = canonical_hash(receipt)
        _write_json_new(final_path, receipt)
        return receipt

    source_policy = policy["source"]
    attempt = _relative_under_root(root, source_policy["full_bare_relative_path"])
    started_at_utc = utc_now()
    start_record = {
        "schema_version": "smpcc-r8-liquid-target-source-fetch-partial-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_FETCH_PARTIAL",
        "created_at_utc": started_at_utc,
        "status": "STARTED_IMMUTABLE_PARTIAL",
        "preflight_sha256": preflight["preflight_sha256"],
        "attempt_path": str(attempt),
        "network_use": policy["network"]["network_use"],
        "upstream_code_executed": False,
        "source_checkout_created": False,
    }
    start_record["partial_sha256"] = canonical_hash(start_record)
    _write_json_new(partial_path, start_record)

    resources = policy["resources"]
    results: dict[str, Any] = {}
    failure: str | None = None
    network_attempted = False
    network_fetch_completed = False
    old_umask = os.umask(0o027)
    try:
        results["attempt_reservation"] = _reserve_new_attempt_directory(
            attempt, int(str(resources["directory_mode_octal"]), 8)
        )
        results["init"] = _run_owned_bounded(
            _git_argv(*policy["git_commands"]["init"], str(attempt)),
            timeout_seconds=resources["short_command_timeout_seconds"],
            grace_seconds=resources["kill_grace_seconds"],
            output_max_bytes=resources["stdout_stderr_max_bytes"],
        )
        if (
            results["init"]["returncode"] != 0
            or results["init"]["executor_error"]
            or results["init"]["hard_timeout_triggered"]
            or results["init"]["hard_kill_for_output_limit"]
        ):
            raise SourceFetchGateError(f"bare Git init failed: {results['init']}")
        if _has_symlink_component(attempt) or attempt.is_symlink() or not attempt.is_dir():
            raise SourceFetchGateError("Git init did not create a safe bare attempt directory")
        if stat.S_IMODE(attempt.stat().st_mode) != int(resources["directory_mode_octal"], 8):
            raise SourceFetchGateError("bare attempt directory mode differs from the frozen policy")

        results["remote_add"] = _run_owned_bounded(
            _git_argv("-C", str(attempt), *policy["git_commands"]["remote_add"]),
            timeout_seconds=resources["short_command_timeout_seconds"],
            grace_seconds=resources["kill_grace_seconds"],
            output_max_bytes=resources["stdout_stderr_max_bytes"],
        )
        if (
            results["remote_add"]["returncode"] != 0
            or results["remote_add"]["executor_error"]
            or results["remote_add"]["hard_timeout_triggered"]
            or results["remote_add"]["hard_kill_for_output_limit"]
        ):
            raise SourceFetchGateError(f"Git remote setup failed: {results['remote_add']}")

        network_attempted = True
        results["fetch"] = _run_owned_bounded(
            _git_argv("-C", str(attempt), *policy["git_commands"]["fetch"]),
            timeout_seconds=resources["fetch_timeout_seconds"],
            grace_seconds=resources["kill_grace_seconds"],
            output_max_bytes=resources["stdout_stderr_max_bytes"],
        )
        if (
            results["fetch"]["returncode"] != 0
            or results["fetch"]["executor_error"]
            or results["fetch"]["hard_timeout_triggered"]
            or results["fetch"]["hard_kill_for_output_limit"]
        ):
            raise SourceFetchGateError(f"exact full-blob Git fetch did not complete: {results['fetch']}")
        network_fetch_completed = True
        results["verification"] = _verify_full_bare(attempt, policy)
    except (OSError, SourceFetchGateError) as exc:
        failure = str(exc)
    finally:
        os.umask(old_umask)

    status = "PASS_FULL_BARE_SOURCE_FETCH" if failure is None else "NO_GO_FULL_BARE_SOURCE_FETCH"
    receipt = {
        "schema_version": "smpcc-r8-liquid-target-source-fetch-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_FETCH_RECEIPT",
        "created_at_utc": utc_now(),
        "status": status,
        "preflight": preflight,
        "partial_start_record": str(partial_path),
        "attempt_path": str(attempt),
        "results": results,
        "failure": failure,
        "network_used": network_attempted,
        "network_attempted": network_attempted,
        "network_fetch_completed": network_fetch_completed,
        "network_use": policy["network"]["network_use"],
        "upstream_code_executed": False,
        "source_checkout_created": False,
        "precompiled_binary_executed": False,
        "cmake_or_make_executed": False,
        "ros_or_gazebo_started": False,
        "gpu_device_exposed": False,
        "system_packages_changed": False,
        "sudo_used": False,
        "next_allowed_stage": "OFFLINE_FULL_BLOB_STATIC_INVENTORY_ONLY" if failure is None else "REVIEW_FETCH_FAILURE",
    }
    receipt["receipt_sha256"] = canonical_hash(receipt)
    _write_json_new(final_path, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="read-only admission check")
    fetch = subparsers.add_parser(
        "fetch-full-bare-source", help="fetch the one frozen commit into one new bare attempt"
    )
    fetch.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        policy = _read_json_object(POLICY_PATH)
        if args.command == "self-check":
            report = build_preflight(policy)
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
            return 0 if report["status"] == "PASS_FULL_SOURCE_FETCH_ADMITTED" else 2
        receipt = fetch_full_bare_source(policy, args.receipt)
        print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if receipt["status"] == "PASS_FULL_BARE_SOURCE_FETCH" else 2
    except (
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        profile_gate.TargetProfileError,
        SourceFetchGateError,
    ) as exc:
        print(f"R8_LIQUID_TARGET_SOURCE_FETCH_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
