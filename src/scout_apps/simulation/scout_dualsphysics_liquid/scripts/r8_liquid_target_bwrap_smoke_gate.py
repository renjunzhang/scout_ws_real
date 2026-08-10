#!/usr/bin/env python3
"""Fail-closed MSI-only bwrap smoke gate.

This v3 gate has exactly two behaviours:

* ``self-check`` reads the frozen local profile and policy and reports whether
  the *system-true-only* smoke command is admissible.  It creates no namespace
  and executes no upstream file.
* ``run-system-true-smoke`` can execute one literal ``/usr/bin/true`` through
  the fully frozen bwrap argv after the read-only check passes.  It never
  accepts a caller-supplied command, bind mount, environment or output path.

The earlier v2 diagnostic is an immutable prerequisite, not an executable
subcommand of this v3 gate. v3 retains at most 4096 bytes of stderr for its
single smoke attempt.

It is intentionally not a generic sandbox runner.  In particular, it has no
checkout, CMake, Make, GenCase, solver, ROS or Gazebo entry point.
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


POLICY_PATH = (
    PACKAGE_DIR
    / "config/target_hosts/liquid_zrj_msi_u2404_bwrap_system_true_smoke_policy_v3.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_bwrap_system_true_smoke_policy_v3.json"
RECEIPT_PATTERNS = {
    "smoke": re.compile(r"^u2_bwrap_system_true_smoke_v3_[0-9]{8}T[0-9]{6}Z\.json$"),
}
SYSTEM_TOOL_NAMES = ("bwrap", "timeout", "prlimit", "system_true")
EXPECTED_BWRAP_ARGS = (
    "--die-with-parent",
    "--new-session",
    "--unshare-user",
    "--unshare-pid",
    "--unshare-net",
    "--unshare-ipc",
    "--unshare-uts",
    "--disable-userns",
    "--assert-userns-disabled",
    "--hostname",
    "r8-liquid-system-true-smoke",
    "--clearenv",
    "--setenv",
    "HOME",
    "/work",
    "--setenv",
    "PATH",
    "/usr/bin:/bin",
    "--uid",
    "65534",
    "--gid",
    "65534",
    "--ro-bind",
    "/usr",
    "/usr",
    "--symlink",
    "usr/bin",
    "/bin",
    "--symlink",
    "usr/lib",
    "/lib",
    "--symlink",
    "usr/lib64",
    "/lib64",
    "--proc",
    "/proc",
    "--dev",
    "/dev",
    "--tmpfs",
    "/tmp",
    "--tmpfs",
    "/work",
    "--chdir",
    "/work",
    "--",
    "/usr/bin/true",
)


class BwrapSmokeGateError(RuntimeError):
    """A policy or runtime ambiguity is a hard NO-GO."""


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_json_object(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise BwrapSmokeGateError(f"JSON file is missing or unsafe: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise BwrapSmokeGateError(f"JSON file is not an object: {path}")
    return value


def _is_clean_absolute(path: Path) -> bool:
    return path.is_absolute() and all(part not in ("", ".", "..", "~") for part in path.parts[1:])


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
            raise BwrapSmokeGateError(f"hash input is not a regular file: {path}")
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
            raise BwrapSmokeGateError(f"file changed while hashing: {path}")
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def _trusted_system_tool(expected: Mapping[str, Any]) -> dict[str, Any]:
    path = Path(str(expected.get("path", "")))
    evidence: dict[str, Any] = {"path": str(path)}
    try:
        if not _is_clean_absolute(path) or not str(path).startswith("/usr/"):
            raise BwrapSmokeGateError("trusted tool path is not a clean /usr path")
        if not path.is_file() or path.is_symlink():
            raise BwrapSmokeGateError("trusted tool is missing, non-regular, or a symlink")
        metadata = path.stat()
        evidence.update(
            {
                "uid": metadata.st_uid,
                "gid": metadata.st_gid,
                "mode_octal": f"{stat.S_IMODE(metadata.st_mode):04o}",
                "regular_file": stat.S_ISREG(metadata.st_mode),
                "executable": bool(metadata.st_mode & 0o111),
            }
        )
        if metadata.st_uid != 0 or metadata.st_gid != 0:
            raise BwrapSmokeGateError("trusted tool is not root-owned")
        if not stat.S_ISREG(metadata.st_mode) or not (metadata.st_mode & 0o111):
            raise BwrapSmokeGateError("trusted tool is not a root-owned executable file")
        evidence["sha256"] = _sha256_regular_file(path)
        evidence["expected_sha256"] = str(expected.get("sha256", ""))
        evidence["trusted"] = evidence["sha256"] == evidence["expected_sha256"]
    except (OSError, BwrapSmokeGateError) as exc:
        evidence["trusted"] = False
        evidence["error"] = str(exc)
    return evidence


def _read_nonnegative_int(path: Path) -> int:
    value = int(path.read_text(encoding="ascii").strip())
    if value < 0:
        raise BwrapSmokeGateError(f"negative kernel limit: {path}")
    return value


def apparmor_userns_is_admitted(restriction: int, label: str) -> bool:
    """Return whether the frozen *unconfined* route may create a user namespace."""

    return restriction == 0 and "(unconfined)" in label


def _thread_count_for_uid(uid: int) -> int:
    """Read a best-effort current thread count for one uid without spawning a process."""

    total = 0
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            values: dict[str, str] = {}
            for line in (entry / "status").read_text(encoding="utf-8").splitlines():
                if ":" in line:
                    key, value = line.split(":", 1)
                    values[key] = value.strip()
            uid_fields = values.get("Uid", "").split()
            if not uid_fields or int(uid_fields[0]) != uid:
                continue
            total += int(values["Threads"])
        except (OSError, KeyError, ValueError):
            continue
    if total < 1:
        raise BwrapSmokeGateError("unable to observe any threads for the target user")
    return total


def _verify_v2_diagnostic(root: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    relative = Path(str(expected.get("receipt_relative_path", "")))
    if relative.is_absolute() or any(part in ("", ".", "..") for part in relative.parts):
        raise BwrapSmokeGateError("v2 diagnostic receipt relative path is unsafe")
    receipt = root / relative
    if _has_symlink_component(receipt) or receipt.is_symlink() or not receipt.is_file():
        raise BwrapSmokeGateError("required v2 diagnostic receipt is missing or unsafe")
    observed_hash = _sha256_regular_file(receipt)
    if observed_hash != expected.get("file_sha256"):
        raise BwrapSmokeGateError("required v2 diagnostic receipt hash mismatch")
    value = _read_json_object(receipt)
    execution = value.get("execution")
    if not isinstance(execution, dict):
        raise BwrapSmokeGateError("required v2 diagnostic receipt lacks execution evidence")
    if value.get("status") != expected.get("status"):
        raise BwrapSmokeGateError("required v2 diagnostic status mismatch")
    if execution.get("returncode") != expected.get("returncode"):
        raise BwrapSmokeGateError("required v2 diagnostic return code mismatch")
    if execution.get("stderr_sha256") != expected.get("stderr_sha256"):
        raise BwrapSmokeGateError("required v2 diagnostic stderr hash mismatch")
    return {
        "path": str(receipt),
        "sha256": observed_hash,
        "status": value["status"],
        "returncode": execution["returncode"],
        "stderr_sha256": execution["stderr_sha256"],
    }


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate frozen semantics only; this function has no host side effects."""

    errors: list[str] = []
    expected_top_level = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-system-true-smoke-policy-v3",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_SYSTEM_TRUE_SMOKE_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_BWRAP_SYSTEM_TRUE_SMOKE_V3",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "system_binary_smoke_authorized": True,
        "upstream_code_execution_authorized": False,
        "allowed_gate_commands": [
            "self-check",
            "run-system-true-smoke",
        ],
    }
    for key, value in expected_top_level.items():
        if policy.get(key) != value:
            errors.append(f"policy field mismatch: {key}")

    prerequisite = policy.get("required_target_profile")
    if not isinstance(prerequisite, dict):
        errors.append("required_target_profile is missing or invalid")
    else:
        if prerequisite.get("profile_id") != "LIQUID_ZRJ_MSI_U2404_P0_V1":
            errors.append("target profile ID mismatch")
        if prerequisite.get("required_status") != "PASS_PROFILE_READ_ONLY_ONLY":
            errors.append("target profile required status mismatch")
        if not re.fullmatch(r"[0-9a-f]{64}", str(prerequisite.get("canonical_sha256", ""))):
            errors.append("target profile canonical hash is invalid")

    diagnostic = policy.get("required_v2_diagnostic")
    expected_diagnostic = {
        "receipt_relative_path": "audits/u2_bwrap_system_true_diagnostic_v2_20260806T071055Z.json",
        "file_sha256": "6a26e01b796f2df597d3ec6100cabdd9dac19673fe6f9a91a1ec9fae2bc88b0d",
        "status": "COMPLETED_SYSTEM_TRUE_DIAGNOSTIC",
        "returncode": 1,
        "stderr_sha256": "4c3f257165f772a36b3ab46ed42d9202c9d1108dd374053c9971c37780338fa3",
    }
    if not isinstance(diagnostic, dict):
        errors.append("required_v2_diagnostic is missing or invalid")
    else:
        for key, value in expected_diagnostic.items():
            if diagnostic.get(key) != value:
                errors.append(f"v2 diagnostic field mismatch: {key}")

    storage = policy.get("storage")
    if not isinstance(storage, dict):
        errors.append("storage is missing or invalid")
    else:
        if storage.get("approved_root") != "/home/zrj/scout_liquid_lab":
            errors.append("approved root differs from the MSI target root")
        if storage.get("receipt_directory") != "audits":
            errors.append("receipt directory must be the fixed audits directory")
        if storage.get("receipt_filename_regex") != RECEIPT_PATTERNS["smoke"].pattern:
            errors.append("smoke receipt filename pattern mismatch")
        if storage.get("receipt_mode_octal") != "0640":
            errors.append("receipt mode must be 0640")

    tools = policy.get("trusted_system_tools")
    if not isinstance(tools, dict) or set(tools) != set(SYSTEM_TOOL_NAMES):
        errors.append("trusted system tool set is not exact")
    else:
        expected_paths = {
            "bwrap": "/usr/bin/bwrap",
            "timeout": "/usr/bin/timeout",
            "prlimit": "/usr/bin/prlimit",
            "system_true": "/usr/bin/true",
        }
        for name, path in expected_paths.items():
            item = tools.get(name)
            if not isinstance(item, dict) or item.get("path") != path:
                errors.append(f"trusted system tool path mismatch: {name}")
            elif not re.fullmatch(r"[0-9a-f]{64}", str(item.get("sha256", ""))):
                errors.append(f"trusted system tool hash is invalid: {name}")

    isolation = policy.get("isolation")
    if not isinstance(isolation, dict):
        errors.append("isolation is missing or invalid")
    else:
        expected_isolation = {
            "network": "none",
            "host_writable_mounts": [],
            "host_read_only_mounts": ["/usr"],
            "ephemeral_tmpfs_paths": ["/tmp", "/work"],
            "user_namespace": "required_and_disabled_inside",
            "pid_namespace": "required",
            "ipc_namespace": "required",
            "uts_namespace": "required",
            "parent_death_kill": True,
            "environment": "cleared_then_fixed_allowlist",
            "apparmor": "observed_not_bypassed_or_modified",
        }
        for key, value in expected_isolation.items():
            if isolation.get(key) != value:
                errors.append(f"isolation field mismatch: {key}")
        forbidden = isolation.get("forbidden_host_path_prefixes")
        if not isinstance(forbidden, list) or not {"/", "/home", "/data", "/home/zrj/scout_ws"}.issubset(
            set(forbidden)
        ):
            errors.append("forbidden host path coverage is incomplete")

    if tuple(policy.get("bwrap_argv", [])) != EXPECTED_BWRAP_ARGS:
        errors.append("bwrap argv differs from the only admitted system-true command")

    resources = policy.get("resources")
    expected_resources = {
        "wall_timeout_seconds": 30,
        "parent_timeout_seconds": 40,
        "kill_after_seconds": 5,
        "cpu_time_seconds": 10,
        "address_space_bytes": 1073741824,
        "process_limit": 4096,
        "minimum_process_headroom": 512,
        "open_file_limit": 128,
        "file_size_bytes": 1048576,
        "core_dump_bytes": 0,
        "stdout": "discarded_not_captured",
        "stderr_max_bytes": 4096,
    }
    if not isinstance(resources, dict):
        errors.append("resources is missing or invalid")
    else:
        for key, value in expected_resources.items():
            if resources.get(key) != value:
                errors.append(f"resource field mismatch: {key}")

    invariants = policy.get("invariants")
    if not isinstance(invariants, dict):
        errors.append("invariants is missing or invalid")
    else:
        for key in (
            "no_sudo",
            "no_apt",
            "no_network",
            "no_aa_exec",
            "no_apparmor_or_sysctl_change",
            "no_source_checkout",
            "no_cmake_or_make",
            "no_upstream_binary",
            "no_ros_or_gazebo",
            "no_gpu_device",
            "no_host_output_mount",
            "no_destructive_cleanup",
        ):
            if invariants.get(key) is not True:
                errors.append(f"safety invariant missing: {key}")
    return errors


def _approved_root(profile: Mapping[str, Any], policy: Mapping[str, Any]) -> Path:
    root = Path(str(policy["storage"]["approved_root"]))
    if root != Path(str(profile["storage"]["approved_root"])):
        raise BwrapSmokeGateError("policy and profile approved roots differ")
    return root


def build_preflight(policy: Mapping[str, Any]) -> dict[str, Any]:
    """Perform read-only checks for the one v3 fixed system-true smoke."""

    errors = validate_policy(policy)
    observations: dict[str, Any] = {}
    try:
        profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
        expected_profile = policy["required_target_profile"]
        profile_hash = canonical_hash(profile)
        observations["target_profile"] = {
            "path": str(profile_gate.PROFILE_PATH),
            "profile_id": profile.get("profile_id"),
            "canonical_sha256": profile_hash,
        }
        if profile_hash != expected_profile["canonical_sha256"]:
            errors.append("target profile canonical hash differs from the system-true policy")
        profile_report = profile_gate.build_report(profile)
        observations["target_profile"]["status"] = profile_report["status"]
        observations["target_profile"]["report_sha256"] = profile_report["report_sha256"]
        if profile_report["status"] != expected_profile["required_status"]:
            errors.append("target profile read-only self-check did not pass")

        root = _approved_root(profile, policy)
        observations["approved_root"] = str(root)
        if _has_symlink_component(root) or root.is_symlink() or not root.is_dir():
            errors.append("approved root is missing, unsafe, or traverses a symlink")
        else:
            root_metadata = root.stat()
            audits = root / str(policy["storage"]["receipt_directory"])
            observations["approved_root_identity"] = {
                "uid": root_metadata.st_uid,
                "gid": root_metadata.st_gid,
                "mode_octal": f"{stat.S_IMODE(root_metadata.st_mode):04o}",
                "device": root_metadata.st_dev,
                "inode": root_metadata.st_ino,
            }
            if _has_symlink_component(audits) or audits.is_symlink() or not audits.is_dir():
                errors.append("fixed audits directory is missing or unsafe")
            owner = pwd.getpwnam(str(profile["storage"]["owner"]))
            observations["invoker"] = {"uid": os.getuid(), "euid": os.geteuid(), "gid": os.getgid()}
            if os.getuid() != owner.pw_uid or os.geteuid() != owner.pw_uid:
                errors.append("gate must run as the target ordinary user, never through sudo")
            if root_metadata.st_uid != owner.pw_uid or root_metadata.st_gid != owner.pw_gid:
                errors.append("approved root ownership differs from target profile")
            try:
                observations["required_v2_diagnostic"] = _verify_v2_diagnostic(
                    root, policy["required_v2_diagnostic"]
                )
            except BwrapSmokeGateError as exc:
                errors.append(f"v2 diagnostic evidence failed: {exc}")

        tools = policy["trusted_system_tools"]
        tool_evidence = {name: _trusted_system_tool(tools[name]) for name in SYSTEM_TOOL_NAMES}
        observations["trusted_system_tools"] = tool_evidence
        for name, evidence in tool_evidence.items():
            if not evidence.get("trusted"):
                errors.append(f"trusted system tool verification failed: {name}")

        userns_switch = _read_nonnegative_int(Path("/proc/sys/kernel/unprivileged_userns_clone"))
        max_user_namespaces = _read_nonnegative_int(Path("/proc/sys/user/max_user_namespaces"))
        apparmor_restriction = _read_nonnegative_int(
            Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns")
        )
        apparmor_label = Path("/proc/self/attr/current").read_text(encoding="utf-8").strip()
        observations["user_namespaces"] = {
            "kernel_unprivileged_userns_clone": userns_switch,
            "max_user_namespaces": max_user_namespaces,
            "apparmor_restrict_unprivileged_userns": apparmor_restriction,
            "parent_apparmor_label": apparmor_label,
            "smoke_apparmor_admitted": apparmor_userns_is_admitted(
                apparmor_restriction, apparmor_label
            ),
        }
        if userns_switch != 1:
            errors.append("kernel does not permit unprivileged user namespaces")
        if max_user_namespaces < 1:
            errors.append("kernel user namespace capacity is zero")
        if "(unconfined)" not in apparmor_label:
            errors.append("parent AppArmor label is no longer the reviewed unconfined invocation")
        if not apparmor_userns_is_admitted(apparmor_restriction, apparmor_label):
            errors.append(
                "AppArmor unprivileged-userns restriction is nonzero or the reviewed label is absent"
            )
        thread_count = _thread_count_for_uid(os.getuid())
        resources = policy["resources"]
        observations["process_limit"] = {
            "current_user_thread_count": thread_count,
            "rlimit_nproc": resources["process_limit"],
            "minimum_headroom": resources["minimum_process_headroom"],
        }
        if thread_count + int(resources["minimum_process_headroom"]) > int(
            resources["process_limit"]
        ):
            errors.append("current user thread count leaves insufficient RLIMIT_NPROC headroom")
    except (KeyError, OSError, ValueError, profile_gate.TargetProfileError, BwrapSmokeGateError) as exc:
        errors.append(f"host inspection failed: {exc}")

    core = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-system-true-smoke-preflight-v3",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_SYSTEM_TRUE_SMOKE_PREFLIGHT",
        "captured_at_utc": utc_now(),
        "policy_path": str(POLICY_PATH),
        "policy_sha256": _sha256_regular_file(POLICY_PATH),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": _sha256_regular_file(SCHEMA_PATH),
        "policy_id": policy.get("policy_id"),
        "host_id": policy.get("host_id"),
        "status": "PASS_SYSTEM_TRUE_SMOKE_ADMITTED" if not errors else "NO_GO_SYSTEM_TRUE_SMOKE",
        "errors": errors,
        "observations": observations,
        "system_binary_smoke_authorized": bool(policy.get("system_binary_smoke_authorized")),
        "upstream_code_execution_authorized": False,
        "upstream_code_executed": False,
        "namespace_attempted": False,
        "namespace_created": False,
        "network_accessed": False,
        "host_writable_mounts": [],
        "next_allowed_stage": "RUN_ONLY_FROZEN_SYSTEM_TRUE_SMOKE" if not errors else "REVIEW_NO_GO",
    }
    return dict(core, preflight_sha256=canonical_hash(core))


def bwrap_argv(policy: Mapping[str, Any]) -> tuple[str, ...]:
    if validate_policy(policy):
        raise BwrapSmokeGateError("refusing to construct argv from an invalid policy")
    return (str(policy["trusted_system_tools"]["bwrap"]["path"]), *EXPECTED_BWRAP_ARGS)


def guarded_argv(policy: Mapping[str, Any]) -> tuple[str, ...]:
    """Return the exact hard-timeout and rlimit wrapped smoke command."""

    if validate_policy(policy):
        raise BwrapSmokeGateError("refusing to construct guarded argv from an invalid policy")
    tools = policy["trusted_system_tools"]
    resources = policy["resources"]
    return (
        str(tools["timeout"]["path"]),
        "--signal=TERM",
        f"--kill-after={resources['kill_after_seconds']}s",
        f"{resources['wall_timeout_seconds']}s",
        str(tools["prlimit"]["path"]),
        f"--cpu={resources['cpu_time_seconds']}:{resources['cpu_time_seconds']}",
        f"--as={resources['address_space_bytes']}:{resources['address_space_bytes']}",
        f"--nproc={resources['process_limit']}:{resources['process_limit']}",
        f"--nofile={resources['open_file_limit']}:{resources['open_file_limit']}",
        f"--fsize={resources['file_size_bytes']}:{resources['file_size_bytes']}",
        f"--core={resources['core_dump_bytes']}:{resources['core_dump_bytes']}",
        "--",
        *bwrap_argv(policy),
    )


def validate_receipt_path(path: Path, approved_root: Path, *, operation: str) -> tuple[Path, Path]:
    if operation not in RECEIPT_PATTERNS:
        raise BwrapSmokeGateError(f"unsupported receipt operation: {operation}")
    if not _is_clean_absolute(path) or not _is_clean_absolute(approved_root):
        raise BwrapSmokeGateError("receipt and approved root must be clean absolute paths")
    if _has_symlink_component(approved_root) or approved_root.is_symlink() or not approved_root.is_dir():
        raise BwrapSmokeGateError("approved root is missing or unsafe")
    audit_directory = approved_root / "audits"
    if _has_symlink_component(audit_directory) or audit_directory.is_symlink() or not audit_directory.is_dir():
        raise BwrapSmokeGateError("fixed audit directory is missing or unsafe")
    if path.parent != audit_directory or RECEIPT_PATTERNS[operation].fullmatch(path.name) is None:
        raise BwrapSmokeGateError("receipt path is not the fixed target audit filename")
    partial = Path(f"{path}.partial")
    if path.exists() or path.is_symlink() or partial.exists() or partial.is_symlink():
        raise BwrapSmokeGateError("receipt final or partial path already exists")
    return path, partial


def _write_json_new(path: Path, value: Mapping[str, Any], mode: int = 0o640) -> str:
    encoded = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(path), flags, mode)
    try:
        os.fchmod(descriptor, mode)
        total = 0
        while total < len(encoded):
            total += os.write(descriptor, encoded[total:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return hashlib.sha256(encoded).hexdigest()


def _run_with_bounded_stderr(
    command: Sequence[str], *, parent_timeout_seconds: int, stderr_max_bytes: int
) -> dict[str, Any]:
    """Execute an owned process group while retaining at most a fixed stderr prefix."""

    if stderr_max_bytes < 1:
        raise BwrapSmokeGateError("bounded stderr limit must be positive")
    captured = bytearray()
    capture_lock = threading.Lock()
    output_limit_hit = threading.Event()
    reader_done = threading.Event()
    process: subprocess.Popen[bytes] | None = None
    executor_error: str | None = None

    def read_stderr(stream: Any) -> None:
        try:
            while True:
                block = stream.read(512)
                if not block:
                    return
                with capture_lock:
                    remaining = stderr_max_bytes - len(captured)
                    if remaining > 0:
                        captured.extend(block[:remaining])
                    if len(block) > remaining:
                        output_limit_hit.set()
                        return
        finally:
            reader_done.set()

    started = time.monotonic()
    timed_out = False
    group_killed_for_output_limit = False
    try:
        process = subprocess.Popen(
            tuple(command),
            cwd="/",
            env={},
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
            close_fds=True,
        )
        if process.stderr is None:
            raise BwrapSmokeGateError("bounded stderr pipe was not created")
        reader = threading.Thread(target=read_stderr, args=(process.stderr,), daemon=True)
        reader.start()
        deadline = started + parent_timeout_seconds
        while process.poll() is None:
            if output_limit_hit.is_set():
                group_killed_for_output_limit = True
                os.killpg(process.pid, signal.SIGKILL)
                break
            if time.monotonic() >= deadline:
                timed_out = True
                os.killpg(process.pid, signal.SIGKILL)
                break
            time.sleep(0.02)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        reader.join(timeout=1)
        if not reader_done.is_set():
            process.stderr.close()
            reader.join(timeout=1)
            if not reader_done.is_set():
                raise BwrapSmokeGateError("bounded stderr reader did not terminate")
    except (OSError, subprocess.SubprocessError, BwrapSmokeGateError) as exc:
        executor_error = str(exc)
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            except (OSError, subprocess.SubprocessError):
                pass

    with capture_lock:
        stderr_bytes = bytes(captured)
    return {
        "returncode": process.returncode if process is not None else None,
        "hard_timeout_triggered": timed_out,
        "hard_kill_for_stderr_limit": group_killed_for_output_limit,
        "stderr_max_bytes": stderr_max_bytes,
        "stderr_captured_bytes": len(stderr_bytes),
        "stderr_sha256": hashlib.sha256(stderr_bytes).hexdigest(),
        "stderr_utf8": stderr_bytes.decode("utf-8", errors="replace"),
        "executor_error": executor_error,
        "elapsed_seconds": round(time.monotonic() - started, 6),
    }


def run_system_true_smoke(policy: Mapping[str, Any], receipt_path: Path) -> dict[str, Any]:
    """Run the one admitted system binary after writing an immutable start record."""

    preflight = build_preflight(policy)
    profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
    root = _approved_root(profile, policy)
    final_path, partial_path = validate_receipt_path(receipt_path, root, operation="smoke")

    if preflight["status"] != "PASS_SYSTEM_TRUE_SMOKE_ADMITTED":
        receipt = {
            "schema_version": "smpcc-r8-liquid-target-bwrap-system-true-smoke-receipt-v3",
            "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_SYSTEM_TRUE_SMOKE_RECEIPT",
            "created_at_utc": utc_now(),
            "status": "NO_GO_STATIC_PRECHECK",
            "preflight": preflight,
            "namespace_attempted": False,
            "namespace_created": False,
            "network_accessed": False,
            "upstream_code_executed": False,
            "command_executed": None,
        }
        receipt["receipt_sha256"] = canonical_hash(receipt)
        _write_json_new(final_path, receipt)
        return receipt

    command = guarded_argv(policy)
    started_at_utc = utc_now()
    start_record = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-system-true-smoke-partial-v3",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_SYSTEM_TRUE_SMOKE_PARTIAL",
        "created_at_utc": started_at_utc,
        "status": "STARTED_IMMUTABLE_PARTIAL",
        "preflight_sha256": preflight["preflight_sha256"],
        "guarded_argv": list(command),
        "guarded_argv_sha256": canonical_hash(list(command)),
        "stdout": policy["resources"]["stdout"],
        "stderr_max_bytes": policy["resources"]["stderr_max_bytes"],
        "upstream_code_executed": False,
        "namespace_attempted": False,
        "host_writable_mounts": [],
    }
    start_record["partial_sha256"] = canonical_hash(start_record)
    _write_json_new(partial_path, start_record)

    execution = _run_with_bounded_stderr(
        command,
        parent_timeout_seconds=int(policy["resources"]["parent_timeout_seconds"]),
        stderr_max_bytes=int(policy["resources"]["stderr_max_bytes"]),
    )
    completed = (
        execution["executor_error"] is None
        and not execution["hard_timeout_triggered"]
        and not execution["hard_kill_for_stderr_limit"]
        and execution["returncode"] is not None
    )
    status = (
        "PASS_SYSTEM_TRUE_BWRAP_SMOKE"
        if execution["returncode"] == 0 and completed
        else "NO_GO_SYSTEM_TRUE_BWRAP_SMOKE"
    )
    receipt = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-system-true-smoke-receipt-v3",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_SYSTEM_TRUE_SMOKE_RECEIPT",
        "created_at_utc": utc_now(),
        "status": status,
        "preflight": preflight,
        "partial_start_record": str(partial_path),
        "guarded_argv": list(command),
        "guarded_argv_sha256": canonical_hash(list(command)),
        "started_at_utc": started_at_utc,
        "execution": execution,
        "stdout": policy["resources"]["stdout"],
        "namespace_attempted": True,
        "namespace_created": execution["returncode"] == 0 and completed,
        "network_accessed": False,
        "upstream_code_executed": False,
        "source_checkout_created": False,
        "cmake_or_make_executed": False,
        "ros_or_gazebo_started": False,
        "gpu_device_exposed": False,
        "host_writable_mounts": [],
        "system_packages_changed": False,
        "sudo_used": False,
        "apparmor_or_sysctl_changed": False,
        "next_allowed_stage": "REVIEW_BWRAP_SMOKE_RECEIPT_ONLY",
    }
    receipt["receipt_sha256"] = canonical_hash(receipt)
    _write_json_new(final_path, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="read-only policy and host check")
    run = subparsers.add_parser(
        "run-system-true-smoke",
        help="run only the frozen /usr/bin/true bwrap smoke and publish a new receipt",
    )
    run.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        policy = _read_json_object(POLICY_PATH)
        if args.command == "self-check":
            report = build_preflight(policy)
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
            return 0 if report["status"] == "PASS_SYSTEM_TRUE_SMOKE_ADMITTED" else 2
        receipt = run_system_true_smoke(policy, args.receipt)
        print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if receipt["status"] == "PASS_SYSTEM_TRUE_BWRAP_SMOKE" else 2
    except (
        BwrapSmokeGateError,
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        profile_gate.TargetProfileError,
    ) as exc:
        print(f"R8_LIQUID_TARGET_BWRAP_SMOKE_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
