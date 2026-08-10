#!/usr/bin/env python3
"""Fail-closed, one-marker bwrap writable-output smoke for the MSI host.

The only admitted executable is the root-owned system ``/usr/bin/touch``. It
may create exactly one empty marker in a new directory below the approved
liquid root.  No source path, checkout, compiler, build system, GPU, ROS,
Gazebo, network, or precompiled binary is available to the namespace.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pwd
import re
import stat
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_bwrap_smoke_gate as base_smoke  # noqa: E402
import r8_liquid_target_profile_gate as profile_gate  # noqa: E402
import r8_liquid_target_source_cpu_build_policy_gate as source_build_policy  # noqa: E402


POLICY_PATH = (
    PACKAGE_DIR
    / "config/target_hosts/liquid_zrj_msi_u2404_bwrap_output_bind_smoke_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_bwrap_output_bind_smoke_policy_v1.json"
ATTEMPT_RE = re.compile(r"^u3_bwrap_output_bind_smoke_v1_[0-9]{8}T[0-9]{6}Z$")
RECEIPT_RE = re.compile(r"^u3_bwrap_output_bind_smoke_v1_[0-9]{8}T[0-9]{6}Z\.json$")


class OutputBindSmokeError(RuntimeError):
    """Any ambiguous path or execution state is a hard no-go."""


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def utc_compact() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _read_json_object(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise OutputBindSmokeError(f"JSON file is missing or unsafe: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise OutputBindSmokeError(f"JSON file is not an object: {path}")
    return value


def _expected_evidence() -> dict[str, Any]:
    return {
        "target_profile": {
            "profile_id": "LIQUID_ZRJ_MSI_U2404_P0_V1",
            "canonical_sha256": "fec0b980ca46df04d877290b73b3f3406af97887ccc20c9d52d58092c36f8838",
            "required_status": "PASS_PROFILE_READ_ONLY_ONLY",
        },
        "bwrap_system_true_smoke_v3": {
            "receipt_relative_path": "audits/u2_bwrap_system_true_smoke_v3_20260806T072112Z.json",
            "file_sha256": "02de50da3e7e88b7dcc3150e06a51ccd06e9bd12978e61479cf2166deb5dec3b",
            "receipt_sha256": "3adc42b366da120402e0886fb72586e14fa306f253c6494fda6c2847ee0c887e",
            "status": "PASS_SYSTEM_TRUE_BWRAP_SMOKE",
            "returncode": 0,
            "namespace_created": True,
            "network_accessed": False,
            "host_writable_mounts": [],
        },
        "source_cpu_build_policy": {
            "policy_canonical_sha256": "bcad3eed1bda70e24e6e0bd317a2241eef54637b5fb83f1411f10cb1cf944dcb",
            "required_status": "PASS_SOURCE_CPU_BUILD_POLICY_STATIC_ONLY",
            "build_execution_authorized": False,
        },
    }


def _expected_storage() -> dict[str, Any]:
    return {
        "approved_root": "/home/zrj/scout_liquid_lab",
        "build_directory": "build",
        "audit_directory": "audits",
        "minimum_available_bytes": 1073741824,
        "attempt_directory_mode_octal": "0750",
        "output_directory_mode_octal": "0700",
        "receipt_mode_octal": "0640",
    }


def _expected_tools() -> dict[str, Any]:
    return {
        "bwrap": {
            "path": "/usr/bin/bwrap",
            "sha256": "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712",
        },
        "timeout": {
            "path": "/usr/bin/timeout",
            "sha256": "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08",
        },
        "prlimit": {
            "path": "/usr/bin/prlimit",
            "sha256": "f27cfd8c1512a4cc6541b59b80cb4cdfd6ef28c34aa21db4299b48264cd0d128",
        },
        "touch": {
            "path": "/usr/bin/touch",
            "sha256": "5244e873c97db0bc4cd9d491eac48d08fba366a7e6584b3fc6225e8aa9d2acaf",
        },
    }


def _expected_bwrap_template() -> list[str]:
    return [
        "/usr/bin/bwrap",
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
        "r8-liquid-output-bind-smoke",
        "--clearenv",
        "--setenv",
        "HOME",
        "/work",
        "--setenv",
        "PATH",
        "/usr/bin:/bin",
        "--uid",
        "0",
        "--gid",
        "0",
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
        "--bind",
        "<ATTEMPT_OUTPUT_DIRECTORY>",
        "/work/output",
        "--chdir",
        "/work",
        "--",
        "/usr/bin/touch",
        "--",
        "/work/output/.r8_output_bind_smoke_v1",
    ]


def _expected_isolation() -> dict[str, Any]:
    return {
        "network": "none",
        "host_read_only_mounts": ["/usr"],
        "host_writable_mounts": ["<ATTEMPT_OUTPUT_DIRECTORY>"],
        "forbidden_host_path_prefixes": [
            "/home/zrj/scout_ws",
            "/home/zrj/ros2_ws",
            "/opt/ros/jazzy",
            "/data",
            "/home/zrj/scout_liquid_lab/dependency",
            "/home/zrj/scout_liquid_lab/incoming",
            "/home/zrj/scout_liquid_lab/outgoing",
            "/home/zrj/scout_liquid_lab/results",
        ],
        "user_namespace": "required_and_disabled_inside",
        "pid_namespace": "required",
        "ipc_namespace": "required",
        "uts_namespace": "required",
        "parent_death_kill": True,
        "environment": "cleared_then_fixed_allowlist",
        "apparmor": "observed_not_bypassed_or_modified",
        "require_apparmor_restrict_unprivileged_userns_zero": True,
        "gpu_device_nodes": [],
        "bwrap_argv_template": _expected_bwrap_template(),
    }


def _expected_resources() -> dict[str, Any]:
    return {
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
        "stderr_max_bytes": 4096,
        "stdout": "discarded_not_captured",
    }


def _expected_attempt() -> dict[str, Any]:
    return {
        "directory_name_regex": ATTEMPT_RE.pattern,
        "output_child_name": "output",
        "marker_name": ".r8_output_bind_smoke_v1",
        "marker_mode_octal": "0600",
        "marker_size_bytes": 0,
        "only_expected_output_entry": True,
        "create_new_only": True,
        "no_symlink_or_hardlink": True,
    }


def _expected_receipt() -> dict[str, Any]:
    return {"filename_regex": RECEIPT_RE.pattern, "partial_suffix": ".partial"}


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate the frozen one-marker contract without inspecting the host."""

    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-output-bind-smoke-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_OUTPUT_BIND_SMOKE_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_BWRAP_OUTPUT_BIND_SMOKE_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "output_bind_smoke_authorized": True,
        "upstream_code_execution_authorized": False,
        "allowed_gate_commands": ["self-check", "run-output-bind-smoke"],
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            errors.append(f"policy field mismatch: {key}")
    sections = (
        ("required_evidence", _expected_evidence(), "required evidence differs"),
        ("storage", _expected_storage(), "storage policy differs"),
        ("trusted_system_tools", _expected_tools(), "trusted system tool policy differs"),
        ("isolation", _expected_isolation(), "isolation or bwrap argv policy differs"),
        ("resources", _expected_resources(), "resource policy differs"),
        ("attempt", _expected_attempt(), "attempt path or marker policy differs"),
        ("receipt", _expected_receipt(), "receipt policy differs"),
    )
    for key, expected, message in sections:
        if policy.get(key) != expected:
            errors.append(message)
    expected_invariants = {
        "no_sudo",
        "no_apt",
        "no_network",
        "no_source_checkout",
        "no_source_materialization",
        "no_source_or_precompiled_binary_mount",
        "no_upstream_code_execution",
        "no_cmake_or_make",
        "no_compiler",
        "no_ros_or_gazebo",
        "no_gpu_device",
        "single_new_host_writable_output_directory",
        "single_empty_marker_file_only",
        "no_destructive_cleanup",
        "no_retry_in_existing_attempt_or_receipt",
        "no_apparmor_or_sysctl_change",
        "no_output_execution",
    }
    invariants = policy.get("invariants")
    if not isinstance(invariants, dict) or set(invariants) != expected_invariants or any(
        invariants.get(key) is not True for key in expected_invariants
    ):
        errors.append("output-bind smoke invariants are incomplete")
    return errors


def validate_schema(schema: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        errors.append("schema draft differs")
    if schema.get("type") != "object" or schema.get("additionalProperties") is not False:
        errors.append("schema must be a closed top-level object")
    required = schema.get("required")
    expected_required = {
        "schema_version",
        "document_type",
        "policy_id",
        "host_id",
        "development_only",
        "formal",
        "physical_primary_eligible",
        "output_bind_smoke_authorized",
        "upstream_code_execution_authorized",
        "allowed_gate_commands",
        "required_evidence",
        "storage",
        "trusted_system_tools",
        "isolation",
        "resources",
        "attempt",
        "receipt",
        "invariants",
    }
    if not isinstance(required, list) or set(required) != expected_required:
        errors.append("schema required-field set differs")
    properties = schema.get("properties")
    if not isinstance(properties, dict):
        return [*errors, "schema properties are missing"]
    expected_consts = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-output-bind-smoke-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_OUTPUT_BIND_SMOKE_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_BWRAP_OUTPUT_BIND_SMOKE_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "output_bind_smoke_authorized": True,
        "upstream_code_execution_authorized": False,
        "allowed_gate_commands": ["self-check", "run-output-bind-smoke"],
    }
    for key, expected in expected_consts.items():
        if properties.get(key, {}).get("const") != expected:
            errors.append(f"schema const differs: {key}")
    return errors


def _safe_directory(path: Path, *, uid: int, gid: int, mode: int) -> dict[str, Any]:
    if base_smoke._has_symlink_component(path) or path.is_symlink() or not path.is_dir():
        raise OutputBindSmokeError(f"directory is missing, non-directory, or unsafe: {path}")
    metadata = path.stat()
    observed = {
        "path": str(path),
        "uid": metadata.st_uid,
        "gid": metadata.st_gid,
        "mode_octal": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
    }
    if metadata.st_uid != uid or metadata.st_gid != gid or stat.S_IMODE(metadata.st_mode) != mode:
        raise OutputBindSmokeError(f"directory ownership or mode differs from policy: {path}")
    return observed


def _verify_v3_smoke_receipt(root: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    relative = Path(str(expected.get("receipt_relative_path", "")))
    if relative.is_absolute() or any(part in ("", ".", "..") for part in relative.parts):
        raise OutputBindSmokeError("v3 smoke receipt relative path is unsafe")
    receipt_path = root / relative
    if base_smoke._has_symlink_component(receipt_path) or receipt_path.is_symlink() or not receipt_path.is_file():
        raise OutputBindSmokeError("v3 smoke receipt is missing or unsafe")
    file_sha256 = base_smoke._sha256_regular_file(receipt_path)
    if file_sha256 != expected.get("file_sha256"):
        raise OutputBindSmokeError("v3 smoke receipt file hash mismatch")
    value = _read_json_object(receipt_path)
    execution = value.get("execution")
    if not isinstance(execution, dict):
        raise OutputBindSmokeError("v3 smoke receipt has no execution record")
    checks = {
        "receipt_sha256": value.get("receipt_sha256"),
        "status": value.get("status"),
        "returncode": execution.get("returncode"),
        "namespace_created": value.get("namespace_created"),
        "network_accessed": value.get("network_accessed"),
        "host_writable_mounts": value.get("host_writable_mounts"),
    }
    for key, observed in checks.items():
        if observed != expected.get(key):
            raise OutputBindSmokeError(f"v3 smoke receipt field mismatch: {key}")
    return {"path": str(receipt_path), "file_sha256": file_sha256, **checks}


def _read_nonnegative_int(path: Path) -> int:
    value = int(path.read_text(encoding="ascii").strip())
    if value < 0:
        raise OutputBindSmokeError(f"negative kernel value: {path}")
    return value


def build_preflight(policy: Mapping[str, Any]) -> dict[str, Any]:
    """Read-only admission; it creates no output directory or namespace."""

    errors = [*validate_policy(policy)]
    observations: dict[str, Any] = {}
    try:
        profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
        expected_profile = policy["required_evidence"]["target_profile"]
        profile_hash = canonical_hash(profile)
        profile_report = profile_gate.build_report(profile)
        observations["target_profile"] = {
            "path": str(profile_gate.PROFILE_PATH),
            "canonical_sha256": profile_hash,
            "status": profile_report.get("status"),
        }
        if profile_hash != expected_profile["canonical_sha256"]:
            errors.append("target profile canonical hash differs")
        if profile_report.get("status") != expected_profile["required_status"]:
            errors.append("target profile read-only self-check did not pass")

        root = Path(str(policy["storage"]["approved_root"]))
        if root != Path(str(profile.get("storage", {}).get("approved_root", ""))):
            errors.append("policy and profile approved roots differ")
        owner = pwd.getpwnam(str(profile["storage"]["owner"]))
        observations["invoker"] = {"uid": os.getuid(), "euid": os.geteuid(), "gid": os.getgid()}
        if os.getuid() != owner.pw_uid or os.geteuid() != owner.pw_uid:
            errors.append("gate must run as the target ordinary user, never through sudo")
        observations["approved_root"] = _safe_directory(root, uid=owner.pw_uid, gid=owner.pw_gid, mode=0o750)
        build_directory = root / str(policy["storage"]["build_directory"])
        audit_directory = root / str(policy["storage"]["audit_directory"])
        observations["build_directory"] = _safe_directory(
            build_directory, uid=owner.pw_uid, gid=owner.pw_gid, mode=0o750
        )
        observations["audit_directory"] = _safe_directory(
            audit_directory, uid=owner.pw_uid, gid=owner.pw_gid, mode=0o750
        )
        space = os.statvfs(build_directory)
        available = space.f_bavail * space.f_frsize
        observations["available_build_bytes"] = available
        if available < int(policy["storage"]["minimum_available_bytes"]):
            errors.append("build directory free space is below the smoke minimum")

        observations["required_v3_smoke"] = _verify_v3_smoke_receipt(
            root, policy["required_evidence"]["bwrap_system_true_smoke_v3"]
        )

        source_policy = source_build_policy._read_json_object(source_build_policy.POLICY_PATH)
        source_schema = source_build_policy._read_json_object(source_build_policy.SCHEMA_PATH)
        source_report = source_build_policy.static_report(source_policy, source_schema)
        expected_source = policy["required_evidence"]["source_cpu_build_policy"]
        observations["source_cpu_build_policy"] = {
            "policy_canonical_sha256": source_report.get("policy_canonical_sha256"),
            "required_status": source_report.get("status"),
            "build_execution_authorized": source_policy.get("build_execution_authorized"),
        }
        if observations["source_cpu_build_policy"] != expected_source:
            errors.append("source-only CPU build policy evidence differs")

        tools = policy["trusted_system_tools"]
        observations["trusted_system_tools"] = {
            name: base_smoke._trusted_system_tool(expected) for name, expected in tools.items()
        }
        for name, evidence in observations["trusted_system_tools"].items():
            if not evidence.get("trusted"):
                errors.append(f"trusted system tool verification failed: {name}")

        userns_clone = _read_nonnegative_int(Path("/proc/sys/kernel/unprivileged_userns_clone"))
        maximum_userns = _read_nonnegative_int(Path("/proc/sys/user/max_user_namespaces"))
        apparmor_restriction = _read_nonnegative_int(
            Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns")
        )
        apparmor_label = Path("/proc/self/attr/current").read_text(encoding="utf-8").strip()
        observations["user_namespaces"] = {
            "kernel_unprivileged_userns_clone": userns_clone,
            "max_user_namespaces": maximum_userns,
            "apparmor_restrict_unprivileged_userns": apparmor_restriction,
            "parent_apparmor_label": apparmor_label,
        }
        if userns_clone != 1 or maximum_userns < 1:
            errors.append("kernel user namespace support is not admitted")
        if "(unconfined)" not in apparmor_label:
            errors.append("parent AppArmor label is not the reviewed unconfined label")
        if policy["isolation"]["require_apparmor_restrict_unprivileged_userns_zero"] and apparmor_restriction != 0:
            errors.append("AppArmor unprivileged-userns restriction is nonzero; do not bypass or modify it")

        thread_count = base_smoke._thread_count_for_uid(os.getuid())
        resources = policy["resources"]
        observations["process_limit"] = {
            "current_user_thread_count": thread_count,
            "rlimit_nproc": resources["process_limit"],
            "minimum_headroom": resources["minimum_process_headroom"],
        }
        if thread_count + int(resources["minimum_process_headroom"]) > int(resources["process_limit"]):
            errors.append("current user thread count leaves insufficient RLIMIT_NPROC headroom")
    except (KeyError, OSError, ValueError, profile_gate.TargetProfileError, OutputBindSmokeError) as exc:
        errors.append(f"host inspection failed: {exc}")

    core = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-output-bind-smoke-preflight-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_OUTPUT_BIND_SMOKE_PREFLIGHT",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "policy_path": str(POLICY_PATH),
        "policy_sha256": base_smoke._sha256_regular_file(POLICY_PATH),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": base_smoke._sha256_regular_file(SCHEMA_PATH),
        "policy_id": policy.get("policy_id"),
        "host_id": policy.get("host_id"),
        "status": "PASS_OUTPUT_BIND_SMOKE_ADMITTED" if not errors else "NO_GO_OUTPUT_BIND_SMOKE",
        "errors": errors,
        "observations": observations,
        "source_checkout_created": False,
        "source_materialized": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "cmake_or_make_executed": False,
        "compiler_executed": False,
        "namespace_attempted": False,
        "namespace_created": False,
        "network_accessed": False,
        "gpu_device_exposed": False,
        "host_writable_mounts": [],
        "next_allowed_stage": "RUN_ONLY_FROZEN_OUTPUT_BIND_SMOKE" if not errors else "REVIEW_NO_GO",
    }
    return dict(core, preflight_sha256=canonical_hash(core))


def validate_attempt_paths(root: Path, timestamp: str, policy: Mapping[str, Any]) -> dict[str, Path]:
    """Construct and validate the only create-new attempt and receipt paths."""

    if validate_policy(policy):
        raise OutputBindSmokeError("refusing paths from an invalid policy")
    attempt_name = f"u3_bwrap_output_bind_smoke_v1_{timestamp}"
    if ATTEMPT_RE.fullmatch(attempt_name) is None:
        raise OutputBindSmokeError("generated attempt name is invalid")
    storage = policy["storage"]
    build_directory = root / str(storage["build_directory"])
    audit_directory = root / str(storage["audit_directory"])
    attempt = build_directory / attempt_name
    output = attempt / str(policy["attempt"]["output_child_name"])
    marker = output / str(policy["attempt"]["marker_name"])
    receipt = audit_directory / f"{attempt_name}.json"
    partial = Path(f"{receipt}{policy['receipt']['partial_suffix']}")
    if RECEIPT_RE.fullmatch(receipt.name) is None:
        raise OutputBindSmokeError("generated receipt name is invalid")
    for directory in (root, build_directory, audit_directory):
        if base_smoke._has_symlink_component(directory) or directory.is_symlink() or not directory.is_dir():
            raise OutputBindSmokeError(f"required parent directory is unsafe: {directory}")
    for path in (attempt, output, marker, receipt, partial):
        if path.exists() or path.is_symlink():
            raise OutputBindSmokeError(f"create-new path already exists or is unsafe: {path}")
    return {
        "attempt": attempt,
        "output": output,
        "marker": marker,
        "receipt": receipt,
        "partial": partial,
    }


def bwrap_argv(policy: Mapping[str, Any], output_directory: Path) -> tuple[str, ...]:
    if validate_policy(policy):
        raise OutputBindSmokeError("refusing to construct argv from an invalid policy")
    template = policy["isolation"]["bwrap_argv_template"]
    if template.count("<ATTEMPT_OUTPUT_DIRECTORY>") != 1:
        raise OutputBindSmokeError("output path placeholder count is not exactly one")
    return tuple(str(output_directory) if item == "<ATTEMPT_OUTPUT_DIRECTORY>" else item for item in template)


def guarded_argv(policy: Mapping[str, Any], output_directory: Path) -> tuple[str, ...]:
    if validate_policy(policy):
        raise OutputBindSmokeError("refusing to construct guarded argv from an invalid policy")
    resources = policy["resources"]
    tools = policy["trusted_system_tools"]
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
        *bwrap_argv(policy, output_directory),
    )


def _mkdir_new_child(parent: Path, name: str, mode: int) -> Path:
    if "/" in name or name in ("", ".", ".."):
        raise OutputBindSmokeError("new directory name is unsafe")
    flags = os.O_RDONLY | os.O_DIRECTORY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    parent_fd = os.open(str(parent), flags)
    try:
        os.mkdir(name, mode, dir_fd=parent_fd)
        child_fd = os.open(name, flags, dir_fd=parent_fd)
        try:
            metadata = os.fstat(child_fd)
            if not stat.S_ISDIR(metadata.st_mode):
                raise OutputBindSmokeError("new path is not a directory")
            os.fchmod(child_fd, mode)
        finally:
            os.close(child_fd)
    finally:
        os.close(parent_fd)
    child = parent / name
    if child.is_symlink() or not child.is_dir():
        raise OutputBindSmokeError("new directory became unsafe")
    return child


def _create_attempt_layout(paths: Mapping[str, Path], policy: Mapping[str, Any]) -> None:
    attempt_mode = int(str(policy["storage"]["attempt_directory_mode_octal"]), 8)
    output_mode = int(str(policy["storage"]["output_directory_mode_octal"]), 8)
    attempt = _mkdir_new_child(paths["attempt"].parent, paths["attempt"].name, attempt_mode)
    output = _mkdir_new_child(attempt, str(policy["attempt"]["output_child_name"]), output_mode)
    if output != paths["output"]:
        raise OutputBindSmokeError("created output directory path differs from frozen path")


def _verify_marker(paths: Mapping[str, Path], policy: Mapping[str, Any], *, uid: int, gid: int) -> dict[str, Any]:
    output = paths["output"]
    marker = paths["marker"]
    expected_mode = int(str(policy["attempt"]["marker_mode_octal"]), 8)
    errors: list[str] = []
    try:
        output_meta = output.lstat()
        if not stat.S_ISDIR(output_meta.st_mode) or stat.S_ISLNK(output_meta.st_mode):
            errors.append("output path is not a real directory")
        entries = sorted(entry.name for entry in output.iterdir())
    except OSError as exc:
        return {"status": "NO_GO", "errors": [f"unable to inspect output directory: {exc}"]}
    expected_entries = [str(policy["attempt"]["marker_name"])]
    if entries != expected_entries:
        errors.append("output directory contains entries other than the one marker")
    try:
        marker_meta = marker.lstat()
        observed = {
            "path": str(marker),
            "uid": marker_meta.st_uid,
            "gid": marker_meta.st_gid,
            "mode_octal": f"{stat.S_IMODE(marker_meta.st_mode):04o}",
            "size_bytes": marker_meta.st_size,
            "nlink": marker_meta.st_nlink,
            "regular_file": stat.S_ISREG(marker_meta.st_mode),
            "symlink": stat.S_ISLNK(marker_meta.st_mode),
        }
        if not stat.S_ISREG(marker_meta.st_mode) or stat.S_ISLNK(marker_meta.st_mode):
            errors.append("marker is not a regular non-symlink file")
        if marker_meta.st_uid != uid or marker_meta.st_gid != gid:
            errors.append("marker ownership differs from target user")
        if stat.S_IMODE(marker_meta.st_mode) != expected_mode:
            errors.append("marker mode differs from frozen umask policy")
        if marker_meta.st_size != int(policy["attempt"]["marker_size_bytes"]):
            errors.append("marker is not empty")
        if marker_meta.st_nlink != 1:
            errors.append("marker hardlink count is not one")
    except OSError as exc:
        return {"status": "NO_GO", "errors": [*errors, f"unable to inspect marker: {exc}"]}
    return {"status": "PASS" if not errors else "NO_GO", "errors": errors, "entries": entries, "marker": observed}


def _preflight_failure_receipt(preflight: Mapping[str, Any]) -> dict[str, Any]:
    receipt = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-output-bind-smoke-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_OUTPUT_BIND_SMOKE_RECEIPT",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "NO_GO_STATIC_PRECHECK",
        "preflight": preflight,
        "source_checkout_created": False,
        "source_materialized": False,
        "namespace_attempted": False,
        "namespace_created": False,
        "network_accessed": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "cmake_or_make_executed": False,
        "compiler_executed": False,
        "output_executed": False,
        "host_writable_mounts": [],
        "system_packages_changed": False,
        "sudo_used": False,
        "apparmor_or_sysctl_changed": False,
        "next_allowed_stage": "REVIEW_NO_GO",
    }
    return dict(receipt, receipt_sha256=canonical_hash(receipt))


def run_output_bind_smoke(policy: Mapping[str, Any], *, timestamp: str | None = None) -> dict[str, Any]:
    """Run the one admitted marker command, or preserve an immutable no-go receipt."""

    preflight = build_preflight(policy)
    profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
    root = Path(str(profile["storage"]["approved_root"]))
    paths = validate_attempt_paths(root, timestamp or utc_compact(), policy)
    receipt_mode = int(str(policy["storage"]["receipt_mode_octal"]), 8)
    if preflight["status"] != "PASS_OUTPUT_BIND_SMOKE_ADMITTED":
        receipt = _preflight_failure_receipt(preflight)
        base_smoke._write_json_new(paths["receipt"], receipt, mode=receipt_mode)
        return receipt

    command = guarded_argv(policy, paths["output"])
    started_at_utc = datetime.now(timezone.utc).isoformat()
    partial = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-output-bind-smoke-partial-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_OUTPUT_BIND_SMOKE_PARTIAL",
        "created_at_utc": started_at_utc,
        "status": "STARTED_IMMUTABLE_PARTIAL",
        "preflight_sha256": preflight["preflight_sha256"],
        "guarded_argv": list(command),
        "guarded_argv_sha256": canonical_hash(list(command)),
        "attempt_directory": str(paths["attempt"]),
        "output_directory": str(paths["output"]),
        "marker_path": str(paths["marker"]),
        "source_checkout_created": False,
        "source_materialized": False,
        "upstream_code_executed": False,
        "host_writable_mounts": [str(paths["output"])],
    }
    partial = dict(partial, partial_sha256=canonical_hash(partial))
    base_smoke._write_json_new(paths["partial"], partial, mode=receipt_mode)

    layout_error: str | None = None
    execution: dict[str, Any] | None = None
    marker_verification: dict[str, Any] = {"status": "NO_GO", "errors": ["attempt did not start"]}
    try:
        _create_attempt_layout(paths, policy)
        previous_umask = os.umask(0o077)
        try:
            execution = base_smoke._run_with_bounded_stderr(
                command,
                parent_timeout_seconds=int(policy["resources"]["parent_timeout_seconds"]),
                stderr_max_bytes=int(policy["resources"]["stderr_max_bytes"]),
            )
        finally:
            os.umask(previous_umask)
        owner = pwd.getpwnam(str(profile["storage"]["owner"]))
        marker_verification = _verify_marker(paths, policy, uid=owner.pw_uid, gid=owner.pw_gid)
    except (OSError, OutputBindSmokeError) as exc:
        layout_error = str(exc)
    if execution is None:
        execution = {
            "returncode": None,
            "hard_timeout_triggered": False,
            "hard_kill_for_stderr_limit": False,
            "stderr_max_bytes": int(policy["resources"]["stderr_max_bytes"]),
            "stderr_captured_bytes": 0,
            "stderr_sha256": hashlib.sha256(b"").hexdigest(),
            "stderr_utf8": "",
            "executor_error": layout_error or "output-bind smoke did not start",
            "elapsed_seconds": 0.0,
        }
    completed = (
        execution.get("executor_error") is None
        and not execution.get("hard_timeout_triggered")
        and not execution.get("hard_kill_for_stderr_limit")
        and execution.get("returncode") == 0
    )
    status = "PASS_OUTPUT_BIND_SMOKE" if completed and marker_verification.get("status") == "PASS" else "NO_GO_OUTPUT_BIND_SMOKE"
    receipt = {
        "schema_version": "smpcc-r8-liquid-target-bwrap-output-bind-smoke-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_BWRAP_OUTPUT_BIND_SMOKE_RECEIPT",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "preflight": preflight,
        "partial_start_record": str(paths["partial"]),
        "guarded_argv": list(command),
        "guarded_argv_sha256": canonical_hash(list(command)),
        "started_at_utc": started_at_utc,
        "attempt_directory": str(paths["attempt"]),
        "output_directory": str(paths["output"]),
        "marker_verification": marker_verification,
        "execution": execution,
        "source_checkout_created": False,
        "source_materialized": False,
        "namespace_attempted": True,
        "namespace_created": completed,
        "network_accessed": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "cmake_or_make_executed": False,
        "compiler_executed": False,
        "output_executed": False,
        "gpu_device_exposed": False,
        "ros_or_gazebo_started": False,
        "host_writable_mounts": [str(paths["output"])],
        "system_packages_changed": False,
        "sudo_used": False,
        "apparmor_or_sysctl_changed": False,
        "next_allowed_stage": (
            "HUMAN_REVIEW_SOURCE_MATERIALIZATION_POLICY_ONLY"
            if status == "PASS_OUTPUT_BIND_SMOKE"
            else "REVIEW_NO_GO"
        ),
    }
    receipt = dict(receipt, receipt_sha256=canonical_hash(receipt))
    base_smoke._write_json_new(paths["receipt"], receipt, mode=receipt_mode)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="read-only policy and host preflight")
    subparsers.add_parser(
        "run-output-bind-smoke",
        help="run only the frozen /usr/bin/touch smoke in one create-new output directory",
    )
    args = parser.parse_args(argv)
    try:
        policy = _read_json_object(POLICY_PATH)
        if args.command == "self-check":
            report = build_preflight(policy)
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
            return 0 if report["status"] == "PASS_OUTPUT_BIND_SMOKE_ADMITTED" else 2
        receipt = run_output_bind_smoke(policy)
        print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if receipt["status"] == "PASS_OUTPUT_BIND_SMOKE" else 2
    except (
        OutputBindSmokeError,
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        profile_gate.TargetProfileError,
    ) as exc:
        print(f"R8_LIQUID_TARGET_OUTPUT_BIND_SMOKE_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
