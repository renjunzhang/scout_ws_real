#!/usr/bin/env python3
"""Static-only review gate for the MSI source-only CPU-build AppArmor draft.

The gate reads policy, schema, draft text, immutable receipts, and the sealed
source tree. It has no command path to parse/load/select a profile, create a
namespace, copy a source file, start a build tool, or write a receipt.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pwd
import stat
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
POLICY_PATH = (
    PACKAGE_DIR
    / "config/target_hosts/liquid_zrj_msi_u2404_apparmor_source_cpu_build_draft_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_apparmor_source_cpu_build_draft_policy_v1.json"
DRAFT_RELATIVE_PATH = "config/apparmor_drafts/r8-liquid-source-cpu-build-draft-v1.profile"
DRAFT_PATH = PACKAGE_DIR / DRAFT_RELATIVE_PATH
DRAFT_SHA256 = "58a9b8cdf9307b4588324a64f742ab0a633949c1996cc9ddcb01388bfb1607ca"

LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
MATERIALIZATION_ATTEMPT_RELATIVE = (
    "dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial"
)
MATERIALIZATION_RECEIPT_RELATIVE = "audits/u3_source_materialization_v1_20260806T155752Z.json"

EXPECTED_PROFILE_LINES = (
    "abi <abi/4.0>,",
    "include <tunables/global>",
    "profile r8-liquid-source-cpu-build-draft-v1 flags=(attach_disconnected,mediate_deleted) {",
    "userns,",
    "/home/ r,",
    "/home/zrj/ r,",
    "/home/zrj/scout_liquid_lab/ r,",
    "/home/zrj/scout_liquid_lab/dependency/ r,",
    "/home/zrj/scout_liquid_lab/dependency/materialized/ r,",
    (
        "/home/zrj/scout_liquid_lab/dependency/materialized/"
        "u3_source_materialization_v1_20260806T155752Z.partial/ r,"
    ),
    (
        "/home/zrj/scout_liquid_lab/dependency/materialized/"
        "u3_source_materialization_v1_20260806T155752Z.partial/src/ r,"
    ),
    (
        "/home/zrj/scout_liquid_lab/dependency/materialized/"
        "u3_source_materialization_v1_20260806T155752Z.partial/src/source/ r,"
    ),
    (
        "/home/zrj/scout_liquid_lab/dependency/materialized/"
        "u3_source_materialization_v1_20260806T155752Z.partial/src/source/** r,"
    ),
    "/home/zrj/scout_liquid_lab/build/ r,",
    "/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_*.partial/ rw,",
    "/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_*.partial/output/ rw,",
    "/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_*.partial/output/** rw,",
    "}",
)

FORBIDDEN_PROFILE_TOKENS = (
    "mount",
    "umount",
    "remount",
    "capability",
    "network",
    "dbus",
    "unix",
    "mqueue",
    "ptrace",
    "signal",
    "change_profile",
    "flags=(unconfined)",
    " ix",
    " px",
    " cx",
    " ux",
    "/home/zrj/scout_ws",
    "/home/zrj/ros2_ws",
    "/opt/ros/",
    "/data/",
    "/usr/bin/",
)


class AppArmorSourceCpuBuildDraftError(RuntimeError):
    """Any static mismatch remains a hard no-go."""


def canonical_hash(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _read_json_object(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise AppArmorSourceCpuBuildDraftError(f"JSON file is missing or unsafe: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AppArmorSourceCpuBuildDraftError(f"JSON file is not an object: {path}")
    return value


def _read_draft_text(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise AppArmorSourceCpuBuildDraftError(f"draft file is missing or unsafe: {path}")
    text = path.read_text(encoding="utf-8")
    if len(text.encode("utf-8")) > 65536:
        raise AppArmorSourceCpuBuildDraftError("draft file exceeds its static review size limit")
    return text


def _file_sha256(path: Path) -> str:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(path), flags)
    digest = hashlib.sha256()
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise AppArmorSourceCpuBuildDraftError(f"hash input is not a regular file: {path}")
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
            raise AppArmorSourceCpuBuildDraftError(f"file changed while hashing: {path}")
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def _effective_profile_lines(text: str) -> tuple[str, ...]:
    rules: list[str] = []
    for raw_line in text.splitlines():
        rule = raw_line.split("#", 1)[0].strip()
        if rule:
            rules.append(rule)
    return tuple(rules)


def _expected_evidence() -> dict[str, Any]:
    return {
        "target_profile": {
            "policy_relative_path": "config/target_hosts/liquid_zrj_msi_u2404_profile_v1.json",
            "policy_canonical_sha256": (
                "fec0b980ca46df04d877290b73b3f3406af97887ccc20c9d52d58092c36f8838"
            ),
            "approved_root": "/home/zrj/scout_liquid_lab",
            "required_status": "PASS_PROFILE_READ_ONLY_ONLY",
        },
        "source_materialization": {
            "receipt_relative_path": MATERIALIZATION_RECEIPT_RELATIVE,
            "file_sha256": (
                "90af263fb7ec8b7d6a46a53aa5354dd5b676cd4167d48fde93911e014a70b745"
            ),
            "receipt_sha256": (
                "017a660bd38da43ae14fb97df9f19cc6cd2b90cf56f91ce047869dbf795d93e2"
            ),
            "status": "PASS_STATIC_SOURCE_MATERIALIZATION",
            "materialized_attempt_root": MATERIALIZATION_ATTEMPT_RELATIVE,
            "sealed_manifest_sha256": (
                "a9a3debebff5ae30bb2d6226deb997f15a6eae0d07581a7c147ce95b9c3da9d2"
            ),
            "file_count": 352,
            "total_bytes": 5473917,
        },
        "source_cpu_build_policy": {
            "policy_relative_path": (
                "config/target_hosts/liquid_zrj_msi_u2404_source_cpu_build_policy_v1.json"
            ),
            "policy_canonical_sha256": (
                "bcad3eed1bda70e24e6e0bd317a2241eef54637b5fb83f1411f10cb1cf944dcb"
            ),
            "required_status": "PASS_SOURCE_CPU_BUILD_POLICY_STATIC_ONLY",
            "build_execution_authorized": False,
        },
        "one_marker_output_bind": {
            "receipt_relative_path": "audits/u3_apparmor_output_bind_probe_v1_20260806T152757Z.json",
            "file_sha256": (
                "a6996cf4e685419c6762a9325b3968483610f6b52fa11645d5ce9202bb99abfa"
            ),
            "status": "PASS_RESTRICTED_ONE_MARKER_OUTPUT_BIND_SMOKE",
            "scope": "one_empty_marker_only_not_a_build_authorization",
        },
    }


def _expected_profile_draft() -> dict[str, Any]:
    return {
        "relative_path": DRAFT_RELATIVE_PATH,
        "profile_name": "r8-liquid-source-cpu-build-draft-v1",
        "file_sha256": DRAFT_SHA256,
        "review_state": "DRAFT_ONLY_NOT_APPROVED_FOR_LOADING",
        "attachment_mode": "named_only_no_attachment_path",
        "static_text_validation_only": True,
        "source_read_root": (
            "/home/zrj/scout_liquid_lab/dependency/materialized/"
            "u3_source_materialization_v1_20260806T155752Z.partial/src/source/"
        ),
        "prospective_output_root_template": (
            "/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_*.partial/output/"
        ),
    }


def _expected_future_execution_contract() -> dict[str, Any]:
    return {
        "source_guest_path": "/work/input/src/source",
        "source_mount_mode": "read_only",
        "host_read_only_bind_count": 1,
        "host_writable_bind_count": 1,
        "host_writable_bind_template": (
            "/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_<timestamp>.partial/output"
        ),
        "network": "none",
        "gpu_device_nodes": [],
        "user_pid_ipc_uts_network_namespaces": "required",
        "bwrap_disable_userns_after_setup": "required",
        "source_copy_phase": "separate_exact_argv_and_profile_review_required",
        "make_and_compiler_phase": "separate_exact_argv_and_profile_review_required",
        "output_execution": "forbidden",
        "precompiled_elf_execution": "forbidden",
    }


def _expected_confinement_contract() -> dict[str, Any]:
    return {
        "executables": "default_deny_no_execute_rule",
        "mount_operations": "default_deny_no_mount_umount_or_remount_rule",
        "capabilities": "default_deny_no_capability_rule",
        "network": "default_deny_no_network_rule",
        "unconfined_flag": "forbidden",
        "source_write": "forbidden",
        "workspace_ros_gpu_and_legacy_paths": "forbidden",
        "prospective_output_tree": "review_target_only_not_authorized_by_this_draft",
    }


def _expected_admin_review_requirements() -> list[str]:
    return [
        "Review a new profile revision against Ubuntu 24.04 AppArmor semantics; this draft is not loadable authority.",
        "Freeze exact bwrap setup/mount/pivot/remount transitions from a separately reviewed argv; do not add generic mount or broad host path rules.",
        "Separate the non-executing source-copy phase from the Make/compiler phase, with exact system-tool hashes and argv for each.",
        "Require one fresh output attempt path, no source write, no network/GPU/ROS, resource enforcement evidence, and post-build static ELF audit before any output use.",
        "Create a new execution admission and explicit local authorization before any profile load, aa-exec, bwrap, copy, Make, compiler, GenCase, solver, or artifact execution.",
    ]


def _expected_cross_host_coordination() -> dict[str, Any]:
    return {
        "allowed_exchange": [
            "Git commit or reviewed patch",
            "policy/schema/profile/gate/test SHA-256 manifest",
            "static self-check report",
            "sanitized AppArmor audit excerpt without credentials",
        ],
        "system_profile_or_privileged_command_transfer": "forbidden",
        "source_or_binary_transfer_for_execution": "forbidden",
        "secrets_or_credentials_in_handoff": "forbidden",
        "result_interpretation": (
            "this draft establishes only a review boundary; it cannot authorize a namespace, bind mount, copy, build, GenCase, solver, ROS, GPU, or output execution"
        ),
    }


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-apparmor-source-cpu-build-draft-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_APPARMOR_SOURCE_CPU_BUILD_DRAFT_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_APPARMOR_SOURCE_CPU_BUILD_DRAFT_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "draft_review_authorized": True,
        "profile_installation_authorized": False,
        "profile_loading_authorized": False,
        "profile_execution_authorized": False,
        "source_copy_execution_authorized": False,
        "build_execution_authorized": False,
        "output_execution_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "status": "STATIC_APPARMOR_SOURCE_CPU_BUILD_DRAFT_NOT_APPROVED_TO_LOAD",
        "next_allowed_stage": (
            "INDEPENDENT_ADMIN_REVIEW_OF_EXACT_BUILD_MOUNT_AND_TRANSITION_PROFILE"
        ),
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            errors.append(f"policy field mismatch: {key}")

    sections = (
        ("required_evidence", _expected_evidence(), "required evidence differs"),
        ("profile_draft", _expected_profile_draft(), "profile draft identity or scope differs"),
        (
            "future_execution_contract",
            _expected_future_execution_contract(),
            "future execution contract differs",
        ),
        (
            "confinement_contract",
            _expected_confinement_contract(),
            "confinement contract weakens default-deny boundary",
        ),
        (
            "admin_review_requirements",
            _expected_admin_review_requirements(),
            "administrator review requirements differ",
        ),
        (
            "cross_host_coordination",
            _expected_cross_host_coordination(),
            "two-host review handoff boundary differs",
        ),
    )
    for key, expected, message in sections:
        if policy.get(key) != expected:
            errors.append(message)

    expected_invariants = {
        "no_sudo",
        "no_system_profile_install",
        "no_profile_load",
        "no_profile_enable",
        "no_profile_execution",
        "no_aa_exec",
        "no_profile_parser",
        "no_sysctl_or_apparmor_change",
        "no_network_permission",
        "no_mount_permission",
        "no_capability_permission",
        "no_unconfined_flag",
        "no_source_write_or_chmod",
        "no_source_copy_execution",
        "no_make_or_compiler",
        "no_output_execution",
        "no_precompiled_elf_execution",
        "no_ros_or_gazebo",
        "no_gpu_device",
        "no_destructive_cleanup",
    }
    invariants = policy.get("invariants")
    if (
        not isinstance(invariants, dict)
        or set(invariants) != expected_invariants
        or any(invariants.get(key) is not True for key in expected_invariants)
    ):
        errors.append("source CPU build draft safety invariants are incomplete")
    return errors


def validate_schema(schema: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        errors.append("schema draft differs")
    if schema.get("type") != "object" or schema.get("additionalProperties") is not False:
        errors.append("schema must be a closed top-level object")
    expected_required = {
        "schema_version",
        "document_type",
        "policy_id",
        "host_id",
        "development_only",
        "formal",
        "physical_primary_eligible",
        "draft_review_authorized",
        "profile_installation_authorized",
        "profile_loading_authorized",
        "profile_execution_authorized",
        "source_copy_execution_authorized",
        "build_execution_authorized",
        "output_execution_authorized",
        "allowed_gate_commands",
        "required_evidence",
        "profile_draft",
        "future_execution_contract",
        "confinement_contract",
        "admin_review_requirements",
        "cross_host_coordination",
        "invariants",
        "status",
        "next_allowed_stage",
    }
    if set(schema.get("required", [])) != expected_required:
        errors.append("schema required-field set differs")
    properties = schema.get("properties")
    if not isinstance(properties, dict):
        return [*errors, "schema properties are missing"]
    expected_consts = {
        "schema_version": "smpcc-r8-liquid-target-apparmor-source-cpu-build-draft-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_APPARMOR_SOURCE_CPU_BUILD_DRAFT_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_APPARMOR_SOURCE_CPU_BUILD_DRAFT_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "draft_review_authorized": True,
        "profile_installation_authorized": False,
        "profile_loading_authorized": False,
        "profile_execution_authorized": False,
        "source_copy_execution_authorized": False,
        "build_execution_authorized": False,
        "output_execution_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "status": "STATIC_APPARMOR_SOURCE_CPU_BUILD_DRAFT_NOT_APPROVED_TO_LOAD",
        "next_allowed_stage": (
            "INDEPENDENT_ADMIN_REVIEW_OF_EXACT_BUILD_MOUNT_AND_TRANSITION_PROFILE"
        ),
    }
    for key, expected in expected_consts.items():
        if properties.get(key, {}).get("const") != expected:
            errors.append(f"schema const differs: {key}")
    return errors


def validate_draft_text(draft_text: str) -> list[str]:
    errors: list[str] = []
    for marker in (
        "DRAFT_ONLY: NOT_APPROVED_FOR_LOADING",
        "has no attachment path",
        "deliberately non-operational",
    ):
        if marker not in draft_text:
            errors.append(f"draft header marker is missing: {marker}")
    effective = _effective_profile_lines(draft_text)
    if effective != EXPECTED_PROFILE_LINES:
        errors.append("draft non-comment rules differ from the frozen default-deny body")
    joined = "\n".join(effective)
    for token in FORBIDDEN_PROFILE_TOKENS:
        if token in joined:
            errors.append(f"draft has a forbidden permission token: {token}")
    return errors


def _relative_under_root(root: Path, relative_text: str) -> Path:
    relative = PurePosixPath(relative_text)
    if relative.is_absolute() or not relative.parts or any(
        part in ("", ".", "..", "~") for part in relative.parts
    ):
        raise AppArmorSourceCpuBuildDraftError(f"unsafe relative path: {relative_text}")
    path = root.joinpath(*relative.parts)
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        if current.is_symlink():
            raise AppArmorSourceCpuBuildDraftError(f"path traverses a symlink: {path}")
        if not current.exists():
            break
    return path


def _verify_target_profile(expected: Mapping[str, Any]) -> dict[str, Any]:
    profile_path = PACKAGE_DIR / str(expected["policy_relative_path"])
    profile = _read_json_object(profile_path)
    profile_hash = canonical_hash(profile)
    if (
        profile_hash != expected["policy_canonical_sha256"]
        or profile.get("storage", {}).get("approved_root") != expected["approved_root"]
    ):
        raise AppArmorSourceCpuBuildDraftError("target profile differs from frozen source-build draft evidence")
    return {
        "path": str(profile_path),
        "canonical_sha256": profile_hash,
        "approved_root": profile["storage"]["approved_root"],
        "required_status": expected["required_status"],
    }


def _verify_source_cpu_build_policy(expected: Mapping[str, Any]) -> dict[str, Any]:
    policy_path = PACKAGE_DIR / str(expected["policy_relative_path"])
    prior_policy = _read_json_object(policy_path)
    policy_hash = canonical_hash(prior_policy)
    if (
        policy_hash != expected["policy_canonical_sha256"]
        or prior_policy.get("build_execution_authorized") is not expected["build_execution_authorized"]
        or prior_policy.get("status")
        != "STATIC_POLICY_REVIEWED_BUILD_NOT_AUTHORIZED_TO_RUN"
    ):
        raise AppArmorSourceCpuBuildDraftError("prior CPU build policy differs from frozen review evidence")
    return {
        "path": str(policy_path),
        "canonical_sha256": policy_hash,
        "status": expected["required_status"],
        "build_execution_authorized": prior_policy["build_execution_authorized"],
    }


def _verify_simple_receipt(root: Path, expected: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    receipt = _relative_under_root(root, str(expected["receipt_relative_path"]))
    if receipt.is_symlink() or not receipt.is_file():
        raise AppArmorSourceCpuBuildDraftError("required receipt is missing or unsafe")
    file_hash = _file_sha256(receipt)
    if file_hash != expected["file_sha256"]:
        raise AppArmorSourceCpuBuildDraftError("required receipt file hash mismatch")
    value = _read_json_object(receipt)
    if value.get("status") != expected["status"]:
        raise AppArmorSourceCpuBuildDraftError("required receipt status mismatch")
    return value, {"path": str(receipt), "file_sha256": file_hash, "status": value["status"]}


def _verify_sealed_source_tree(
    root: Path, expected: Mapping[str, Any], receipt: Mapping[str, Any]
) -> dict[str, Any]:
    if (
        receipt.get("receipt_sha256") != expected["receipt_sha256"]
        or receipt.get("materialized_attempt_root") != str(root / expected["materialized_attempt_root"])
        or receipt.get("source_materialized") is not True
        or receipt.get("source_checkout_created") is not False
        or receipt.get("upstream_code_executed") is not False
        or receipt.get("precompiled_binary_executed") is not False
        or receipt.get("cmake_or_make_executed") is not False
        or receipt.get("compiler_executed") is not False
        or receipt.get("network_used") is not False
        or receipt.get("gpu_device_exposed") is not False
        or receipt.get("sudo_used") is not False
    ):
        raise AppArmorSourceCpuBuildDraftError("source materialization receipt does not preserve the no-execution facts")
    try:
        sealed = receipt["results"]["materialization"]["sealed_output"]
    except (KeyError, TypeError) as exc:
        raise AppArmorSourceCpuBuildDraftError("source materialization receipt lacks sealed output evidence") from exc
    required_sealed = {
        "materialized_root": str(root / expected["materialized_attempt_root"]),
        "materialized_tree_relative_path": "src/source",
        "file_count": expected["file_count"],
        "total_bytes": expected["total_bytes"],
        "symlink_count": 0,
        "hardlink_count": 0,
        "all_files_non_executable": True,
        "sealed_directory_mode_octal": "0550",
        "sealed_file_mode_octal": "0440",
        "manifest_sha256": expected["sealed_manifest_sha256"],
    }
    for key, value in required_sealed.items():
        if sealed.get(key) != value:
            raise AppArmorSourceCpuBuildDraftError(f"sealed source evidence differs: {key}")
    entries = sealed.get("entries")
    if not isinstance(entries, list) or len(entries) != expected["file_count"]:
        raise AppArmorSourceCpuBuildDraftError("sealed source entry manifest is missing or incomplete")

    owner = pwd.getpwnam("zrj")
    attempt_root = root / expected["materialized_attempt_root"]
    source_root = attempt_root / "src" / "source"
    for path in (attempt_root, attempt_root / "src", source_root):
        if path.is_symlink() or not path.is_dir():
            raise AppArmorSourceCpuBuildDraftError("sealed source root is missing or unsafe")
        metadata = path.stat()
        if (
            metadata.st_uid != owner.pw_uid
            or metadata.st_gid != owner.pw_gid
            or stat.S_IMODE(metadata.st_mode) != 0o550
        ):
            raise AppArmorSourceCpuBuildDraftError("sealed source root ownership or mode differs")

    expected_by_path: dict[str, Mapping[str, Any]] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise AppArmorSourceCpuBuildDraftError("sealed source entry is not an object")
        path_text = str(entry.get("path", ""))
        relative = PurePosixPath(path_text)
        if (
            relative.is_absolute()
            or not path_text.startswith("src/source/")
            or any(part in ("", ".", "..", "~") for part in relative.parts)
            or path_text in expected_by_path
        ):
            raise AppArmorSourceCpuBuildDraftError("sealed source entry path is unsafe or duplicated")
        expected_by_path[path_text] = entry

    observed_paths: set[str] = set()
    observed_bytes = 0
    for current_root, directory_names, file_names in os.walk(
        attempt_root, topdown=True, followlinks=False
    ):
        current = Path(current_root)
        current_metadata = current.lstat()
        if current.is_symlink() or not stat.S_ISDIR(current_metadata.st_mode):
            raise AppArmorSourceCpuBuildDraftError("sealed source tree contains an unsafe directory")
        if stat.S_IMODE(current_metadata.st_mode) != 0o550:
            raise AppArmorSourceCpuBuildDraftError("sealed source directory is not read-only")
        for directory_name in directory_names:
            directory = current / directory_name
            metadata = directory.lstat()
            if directory.is_symlink() or not stat.S_ISDIR(metadata.st_mode):
                raise AppArmorSourceCpuBuildDraftError("sealed source tree contains a symlink or non-directory")
        for file_name in file_names:
            path = current / file_name
            metadata = path.lstat()
            relative_path = path.relative_to(attempt_root).as_posix()
            expected_entry = expected_by_path.get(relative_path)
            if (
                expected_entry is None
                or path.is_symlink()
                or not stat.S_ISREG(metadata.st_mode)
                or metadata.st_nlink != 1
                or metadata.st_uid != owner.pw_uid
                or metadata.st_gid != owner.pw_gid
                or stat.S_IMODE(metadata.st_mode) != 0o440
                or metadata.st_mode & 0o111
                or metadata.st_size != int(expected_entry.get("size_bytes", -1))
                or _file_sha256(path) != expected_entry.get("sha256")
            ):
                raise AppArmorSourceCpuBuildDraftError("sealed source file differs from its immutable manifest")
            observed_paths.add(relative_path)
            observed_bytes += metadata.st_size
    if observed_paths != set(expected_by_path) or observed_bytes != expected["total_bytes"]:
        raise AppArmorSourceCpuBuildDraftError("sealed source tree topology or byte total differs")
    return {
        "attempt_root": str(attempt_root),
        "source_root": str(source_root),
        "file_count": len(observed_paths),
        "total_bytes": observed_bytes,
        "symlink_count": 0,
        "hardlink_count": 0,
        "file_mode_octal": "0440",
        "directory_mode_octal": "0550",
        "manifest_sha256": expected["sealed_manifest_sha256"],
    }


def static_report(
    policy: Mapping[str, Any], schema: Mapping[str, Any], draft_text: str, draft_file_hash: str
) -> dict[str, Any]:
    errors = [*validate_policy(policy), *validate_schema(schema), *validate_draft_text(draft_text)]
    observations: dict[str, Any] = {
        "policy_canonical_sha256": canonical_hash(policy),
        "schema_canonical_sha256": canonical_hash(schema),
        "draft_file_sha256": draft_file_hash,
    }
    if draft_file_hash != DRAFT_SHA256:
        errors.append("draft file SHA-256 differs from the frozen policy")
    try:
        required = policy["required_evidence"]
        observations["target_profile"] = _verify_target_profile(required["target_profile"])
        source_receipt, source_evidence = _verify_simple_receipt(
            LIQUID_ROOT, required["source_materialization"]
        )
        if source_receipt.get("receipt_sha256") != required["source_materialization"]["receipt_sha256"]:
            raise AppArmorSourceCpuBuildDraftError("source materialization receipt canonical hash mismatch")
        observations["source_materialization_receipt"] = source_evidence
        observations["sealed_source_tree"] = _verify_sealed_source_tree(
            LIQUID_ROOT, required["source_materialization"], source_receipt
        )
        observations["source_cpu_build_policy"] = _verify_source_cpu_build_policy(
            required["source_cpu_build_policy"]
        )
        _, output_bind_evidence = _verify_simple_receipt(LIQUID_ROOT, required["one_marker_output_bind"])
        observations["one_marker_output_bind"] = output_bind_evidence
    except (KeyError, OSError, ValueError, json.JSONDecodeError, AppArmorSourceCpuBuildDraftError) as exc:
        errors.append(f"static build-draft evidence inspection failed: {exc}")

    core = {
        "schema_version": "smpcc-r8-liquid-target-apparmor-source-cpu-build-draft-report-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_APPARMOR_SOURCE_CPU_BUILD_DRAFT_REPORT",
        "policy_id": policy.get("policy_id"),
        "status": (
            "PASS_APPARMOR_SOURCE_CPU_BUILD_DRAFT_STATIC_ONLY"
            if not errors
            else "NO_GO_APPARMOR_SOURCE_CPU_BUILD_DRAFT"
        ),
        "errors": errors,
        "observations": observations,
        "profile_copied_to_system": False,
        "profile_parser_invoked": False,
        "profile_loaded": False,
        "profile_selected_for_execution": False,
        "namespace_attempted": False,
        "source_copied": False,
        "source_checkout_created": False,
        "source_materialized": False,
        "make_executed": False,
        "compiler_executed": False,
        "output_executed": False,
        "precompiled_binary_executed": False,
        "network_used": False,
        "gpu_device_exposed": False,
        "sudo_used": False,
        "sysctl_or_apparmor_changed": False,
        "next_allowed_stage": (
            "INDEPENDENT_ADMIN_REVIEW_OF_EXACT_BUILD_MOUNT_AND_TRANSITION_PROFILE"
            if not errors
            else "REVIEW_NO_GO"
        ),
    }
    return dict(core, report_sha256=canonical_hash(core))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="read-only static source CPU build draft check")
    args = parser.parse_args(argv)
    try:
        if args.command != "self-check":
            raise AppArmorSourceCpuBuildDraftError("unsupported command")
        policy = _read_json_object(POLICY_PATH)
        schema = _read_json_object(SCHEMA_PATH)
        draft_text = _read_draft_text(DRAFT_PATH)
        report = static_report(policy, schema, draft_text, _file_sha256(DRAFT_PATH))
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if report["status"] == "PASS_APPARMOR_SOURCE_CPU_BUILD_DRAFT_STATIC_ONLY" else 2
    except (OSError, ValueError, json.JSONDecodeError, AppArmorSourceCpuBuildDraftError) as exc:
        print(f"R8_LIQUID_TARGET_APPARMOR_SOURCE_CPU_BUILD_DRAFT_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
