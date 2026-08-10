#!/usr/bin/env python3
"""Fail-closed, offline, source-only materialization for the MSI liquid host.

This is deliberately not a checkout or build tool.  It reads one immutable
bare repository through a frozen system Git binary, with every transport
disabled, and copies only "src/source/**" blobs into one fresh attempt
directory.  It never invokes an upstream file, hook, filter, CMake, Make,
compiler, ROS/Gazebo program, GPU program, or a precompiled ELF.

The successful output remains in a ".partial" attempt directory on purpose:
the suffix makes it ineligible for future use unless a separate admission
re-verifies the receipt and the sealed content tree.  Failed attempts are
retained and never cleaned up or reused.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pwd
import re
import shutil
import stat
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_profile_gate as profile_gate  # noqa: E402
import r8_liquid_target_source_fetch_gate as fetch_gate  # noqa: E402


POLICY_PATH = (
    PACKAGE_DIR
    / "config/target_hosts/liquid_zrj_msi_u2404_source_materialization_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_source_materialization_policy_v1.json"

EXPECTED_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
EXPECTED_TREE = "cef458cb358712f4694b9d2148f638440418e9dc"
EXPECTED_URL = "https://github.com/DualSPHysics/DualSPHysics.git"
EXPECTED_ATTEMPT_RE = re.compile(r"^u3_source_materialization_v1_[0-9]{8}T[0-9]{6}Z$")
EXPECTED_RECEIPT_RE = re.compile(
    r"^u3_source_materialization_v1_[0-9]{8}T[0-9]{6}Z\.json$"
)
SHA1_RE = re.compile(r"^[0-9a-f]{40}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class SourceMaterializationError(RuntimeError):
    """Every ambiguity is a fail-closed materialization no-go."""


def canonical_hash(value: Any) -> str:
    """Return the cross-host canonical JSON hash."""

    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_json_object(path: Path) -> dict[str, Any]:
    value = fetch_gate._read_json_object(path)
    if not isinstance(value, dict):
        raise SourceMaterializationError(f"JSON file is not an object: {path}")
    return value


def _offline_git_environment() -> dict[str, str]:
    """Use Git plumbing only, with all transports and user configuration disabled."""

    environment = fetch_gate._clean_git_environment()
    environment.update(
        {
            "GIT_ALLOW_PROTOCOL": "none",
            "GIT_PROTOCOL_FROM_USER": "0",
            "GIT_PAGER": "cat",
        }
    )
    return environment


def _compact_command_evidence(result: Mapping[str, Any]) -> dict[str, Any]:
    keys = (
        "argv",
        "returncode",
        "hard_timeout_triggered",
        "hard_kill_for_output_limit",
        "output_max_bytes",
        "stdout_bytes",
        "stderr_bytes",
        "stdout_sha256",
        "stderr_sha256",
        "stderr_utf8",
        "executor_error",
        "elapsed_seconds",
    )
    return {key: result.get(key) for key in keys}


def _expected_evidence() -> dict[str, Any]:
    return {
        "target_profile": {
            "profile_id": "LIQUID_ZRJ_MSI_U2404_P0_V1",
            "canonical_sha256": "fec0b980ca46df04d877290b73b3f3406af97887ccc20c9d52d58092c36f8838",
            "required_status": "PASS_PROFILE_READ_ONLY_ONLY",
        },
        "full_source_fetch": {
            "receipt_relative_path": "audits/u2_full_source_fetch_attempt_3_20260806T074600Z.json",
            "file_sha256": "23745c0076c2c06fc549665c89f4e039761832bab66f4f568f63eff696ead18d",
            "receipt_sha256": "90bd0c1cc03b022dc42dd4b0368a84f46af10b9b5b8c54108765d1fd16df0426",
            "status": "PASS_FULL_BARE_SOURCE_FETCH",
        },
        "static_inventory": {
            "receipt_relative_path": "audits/u2_full_source_static_inventory_v1_20260806T080500Z.json",
            "file_sha256": "a3be7e2a7a5bbeae8483706dbd4a135c7af80513bd5fbd1f982122b4380efb28",
            "receipt_sha256": "5eac669287c6711de0835409ed9f3ec9dc6829a3b03cef24cb6838ebb07e32ff",
            "status": "PASS_OFFLINE_FULL_SOURCE_STATIC_INVENTORY",
        },
        "elf_metadata": {
            "receipt_relative_path": "audits/u2_elf_metadata_v1_20260806T082300Z.json",
            "file_sha256": "47e411b391afd677fc4d1b60ca6547c73545c287ef080bea92c2e66128ef6a74",
            "receipt_sha256": "7686d2df11332dc4c5809f64985959ed8e05fd96b3b44c5120e516bbdd850024",
            "status": "PASS_OFFLINE_ELF_METADATA",
        },
    }


def _expected_source() -> dict[str, Any]:
    return {
        "repository_url": EXPECTED_URL,
        "commit": EXPECTED_COMMIT,
        "tree": EXPECTED_TREE,
        "full_bare_relative_path": (
            "dependency/source/"
            "DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git"
        ),
        "allowed_source_prefix": "src/source/",
        "materialization_parent_relative_path": "dependency/materialized",
        "attempt_id_regex": EXPECTED_ATTEMPT_RE.pattern,
        "attempt_root_relative_template": "dependency/materialized/<attempt_id>.partial",
        "materialized_tree_relative_path": "src/source",
        "no_checkout": True,
        "no_network": True,
        "no_submodules": True,
        "no_lfs_smudge": True,
        "no_hooks": True,
        "no_filters": True,
        "precompiled_elf_execution": "forbidden",
        "elf_magic_in_materialized_source": "forbidden",
    }


def _expected_git_commands() -> dict[str, list[str]]:
    return {
        "tree_identity": ["rev-parse", f"{EXPECTED_COMMIT}^{{tree}}"],
        "bare_identity": ["rev-parse", "--is-bare-repository"],
        "remote_identity": ["remote", "get-url", "origin"],
        "source_tree_listing": [
            "ls-tree",
            "-r",
            "-l",
            "-z",
            EXPECTED_COMMIT,
            "--",
            "src/source",
        ],
        "blob": ["cat-file", "blob"],
    }


def _expected_subset() -> dict[str, Any]:
    return {
        "source_tree_listing_sha256": (
            "52adb0bc80591775a81a7edaa2217c313d216d59e1b7b13094f94b87c1627f9b"
        ),
        "entry_count": 352,
        "total_blob_bytes": 5473917,
        "symlink_entry_count": 0,
        "gitlink_entry_count": 0,
        "input_executable_mode_paths": [
            "src/source/JDsGaugeItem.cpp",
            "src/source/JDsGaugeItem.h",
            "src/source/JDsGaugeSystem.cpp",
            "src/source/JDsGaugeSystem.h",
            "src/source/JDsGauge_ker.cu",
            "src/source/JDsGauge_ker.h",
        ],
    }


def _expected_output() -> dict[str, Any]:
    return {
        "create_materialization_parent_if_absent": True,
        "directory_mode_before_seal_octal": "0750",
        "file_mode_before_seal_octal": "0640",
        "directory_mode_after_seal_octal": "0550",
        "file_mode_after_seal_octal": "0440",
        "symlinks": "forbidden",
        "hardlinks": "forbidden",
        "preserve_input_executable_mode": False,
        "output_content_execution": "forbidden",
    }


def _expected_resources() -> dict[str, Any]:
    return {
        "minimum_free_bytes": 16777216,
        "materialization_timeout_seconds": 300,
        "short_command_timeout_seconds": 60,
        "tree_listing_timeout_seconds": 60,
        "blob_read_timeout_seconds": 60,
        "kill_grace_seconds": 5,
        "short_command_output_max_bytes": 16384,
        "tree_listing_output_max_bytes": 262144,
        "blob_stderr_max_bytes": 16384,
        "max_tree_entries": 512,
        "max_tree_path_bytes": 512,
        "max_total_blob_bytes": 16777216,
        "max_single_blob_bytes": 1048576,
        "directory_mode_octal": "0750",
        "receipt_mode_octal": "0640",
    }


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate the frozen source-only contract without touching the host."""

    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-source-materialization-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_MATERIALIZATION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_SOURCE_MATERIALIZATION_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "static_source_materialization_authorized": True,
        "build_execution_authorized": False,
        "output_execution_authorized": False,
        "upstream_code_execution_authorized": False,
        "allowed_gate_commands": ["self-check", "materialize-source"],
        "status": "STATIC_SOURCE_MATERIALIZATION_ONLY_BUILD_NOT_AUTHORIZED",
        "next_allowed_stage": "MATERIALIZE_ONE_NEW_SOURCE_ATTEMPT_OR_REVIEW_NO_GO",
    }
    for key, value in expected_top.items():
        if policy.get(key) != value:
            errors.append(f"policy field mismatch: {key}")

    if policy.get("required_evidence") != _expected_evidence():
        errors.append("required receipt/profile evidence differs from the frozen values")
    if policy.get("source") != _expected_source():
        errors.append("source materialization boundary differs from the frozen values")
    if policy.get("trusted_system_tools") != {
        "git": {
            "path": str(fetch_gate.GIT_PATH),
            "sha256": "2a8c18fbf43da9f692d75474c72bea9dfd796c260b0f3dfe456376abc3bbd668",
        }
    }:
        errors.append("trusted Git declaration differs from the frozen system binary")
    if policy.get("git_commands") != _expected_git_commands():
        errors.append("Git argv policy differs from the frozen offline materialization contract")
    if policy.get("source_subset_expectations") != _expected_subset():
        errors.append("source subset expectations differ from the frozen tree listing")
    if policy.get("output") != _expected_output():
        errors.append("create-new or sealed-output policy differs from the frozen contract")
    if policy.get("resources") != _expected_resources():
        errors.append("materialization resource limits differ from the frozen values")
    if policy.get("receipt") != {
        "directory_relative_path": "audits",
        "filename_regex": EXPECTED_RECEIPT_RE.pattern,
    }:
        errors.append("materialization receipt policy is invalid")

    required_invariants = {
        "no_sudo",
        "no_apt",
        "no_network",
        "no_checkout",
        "no_source_file_execution",
        "no_precompiled_binary_execution",
        "no_cmake_or_make",
        "no_compiler",
        "no_ros_or_gazebo",
        "no_gpu_device",
        "no_destructive_cleanup",
        "no_retry_in_existing_attempt_path",
        "new_files_non_executable",
        "successful_tree_sealed_read_only",
    }
    invariants = policy.get("invariants")
    if (
        not isinstance(invariants, dict)
        or set(invariants) != required_invariants
        or any(invariants.get(key) is not True for key in required_invariants)
    ):
        errors.append("source materialization safety invariants are incomplete")
    return errors


def _validate_schema(schema: Mapping[str, Any]) -> list[str]:
    expected_required = {
        "schema_version",
        "document_type",
        "policy_id",
        "host_id",
        "development_only",
        "formal",
        "physical_primary_eligible",
        "static_source_materialization_authorized",
        "build_execution_authorized",
        "output_execution_authorized",
        "upstream_code_execution_authorized",
        "allowed_gate_commands",
        "required_evidence",
        "source",
        "trusted_system_tools",
        "git_commands",
        "source_subset_expectations",
        "output",
        "resources",
        "receipt",
        "invariants",
        "status",
        "next_allowed_stage",
    }
    errors: list[str] = []
    if schema.get("type") != "object" or schema.get("additionalProperties") is not False:
        errors.append("schema must close the materialization policy object")
    if set(schema.get("required", [])) != expected_required:
        errors.append("schema required fields differ from the materialization contract")
    properties = schema.get("properties")
    if not isinstance(properties, dict):
        errors.append("schema properties are missing")
    elif properties.get("static_source_materialization_authorized", {}).get("const") is not True:
        errors.append("schema does not require static materialization authorization")
    elif properties.get("build_execution_authorized", {}).get("const") is not False:
        errors.append("schema does not preserve the build execution no-go")
    return errors


def _approved_root(profile: Mapping[str, Any]) -> Path:
    root = Path(str(profile.get("storage", {}).get("approved_root", "")))
    if root != Path("/home/zrj/scout_liquid_lab"):
        raise SourceMaterializationError("target profile approved root differs from the frozen MSI root")
    return root


def _verify_required_receipt(root: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    receipt = fetch_gate._relative_under_root(root, str(expected.get("receipt_relative_path", "")))
    if not receipt.is_file() or receipt.is_symlink():
        raise SourceMaterializationError("required immutable receipt is missing or unsafe")
    file_sha256 = fetch_gate._sha256_regular_file(receipt)
    if file_sha256 != expected.get("file_sha256"):
        raise SourceMaterializationError("required immutable receipt file hash mismatch")
    receipt_value = _read_json_object(receipt)
    if (
        receipt_value.get("status") != expected.get("status")
        or receipt_value.get("receipt_sha256") != expected.get("receipt_sha256")
        or receipt_value.get("upstream_code_executed") is not False
        or receipt_value.get("precompiled_binary_executed") is not False
    ):
        raise SourceMaterializationError("required immutable receipt content does not match its admission")
    return {
        "path": str(receipt),
        "file_sha256": file_sha256,
        "receipt_sha256": receipt_value["receipt_sha256"],
        "status": receipt_value["status"],
    }


def _git_text(source: Path, args: Sequence[str], policy: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
    resources = policy["resources"]
    result = fetch_gate._run_owned_bounded(
        fetch_gate._git_argv("--git-dir", str(source), *args),
        timeout_seconds=int(resources["short_command_timeout_seconds"]),
        grace_seconds=int(resources["kill_grace_seconds"]),
        output_max_bytes=int(resources["short_command_output_max_bytes"]),
        environment=_offline_git_environment(),
    )
    if (
        result["returncode"] != 0
        or result["executor_error"] is not None
        or result["hard_timeout_triggered"]
        or result["hard_kill_for_output_limit"]
    ):
        raise SourceMaterializationError(
            f"offline Git identity query failed: {_compact_command_evidence(result)}"
        )
    return str(result["stdout_utf8"]).strip(), _compact_command_evidence(result)


def _safe_existing_directory(path: Path, owner: pwd.struct_passwd, expected_mode: int) -> dict[str, Any]:
    if fetch_gate._has_symlink_component(path) or path.is_symlink() or not path.is_dir():
        raise SourceMaterializationError(f"directory is missing or unsafe: {path}")
    metadata = path.stat()
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or metadata.st_uid != owner.pw_uid
        or metadata.st_gid != owner.pw_gid
        or stat.S_IMODE(metadata.st_mode) != expected_mode
    ):
        raise SourceMaterializationError(f"directory ownership or mode differs from policy: {path}")
    return {
        "path": str(path),
        "uid": metadata.st_uid,
        "gid": metadata.st_gid,
        "mode_octal": f"{stat.S_IMODE(metadata.st_mode):04o}",
    }


def build_preflight(policy: Mapping[str, Any]) -> dict[str, Any]:
    """Read-only admission.  It makes no output directory and reads no source blob."""

    errors = validate_policy(policy)
    observations: dict[str, Any] = {}
    try:
        schema = _read_json_object(SCHEMA_PATH)
        errors.extend(_validate_schema(schema))

        profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
        profile_hash = canonical_hash(profile)
        observations["target_profile"] = {
            "path": str(profile_gate.PROFILE_PATH),
            "profile_id": profile.get("profile_id"),
            "canonical_sha256": profile_hash,
        }
        required_profile = policy["required_evidence"]["target_profile"]
        if profile_hash != required_profile["canonical_sha256"]:
            errors.append("target profile canonical hash differs from materialization policy")
        profile_report = profile_gate.build_report(profile)
        observations["target_profile"]["status"] = profile_report["status"]
        observations["target_profile"]["report_sha256"] = profile_report["report_sha256"]
        if profile_report["status"] != required_profile["required_status"]:
            errors.append("target profile read-only self-check did not pass")

        root = _approved_root(profile)
        observations["approved_root"] = str(root)
        owner = pwd.getpwnam(str(profile["storage"]["owner"]))
        observations["invoker"] = {"uid": os.getuid(), "euid": os.geteuid(), "gid": os.getgid()}
        if os.getuid() != owner.pw_uid or os.geteuid() != owner.pw_uid:
            errors.append("source materialization must run as the target ordinary user, never through sudo")
        observations["approved_root_metadata"] = _safe_existing_directory(root, owner, 0o750)

        free_bytes = shutil.disk_usage(root).free
        observations["storage"] = {
            "free_bytes": free_bytes,
            "minimum_free_bytes": int(policy["resources"]["minimum_free_bytes"]),
        }
        if free_bytes < int(policy["resources"]["minimum_free_bytes"]):
            errors.append("approved root has insufficient free space for the bounded materialization")

        required_evidence = policy["required_evidence"]
        observations["required_evidence"] = {
            name: _verify_required_receipt(root, required_evidence[name])
            for name in ("full_source_fetch", "static_inventory", "elf_metadata")
        }

        source = fetch_gate._relative_under_root(root, policy["source"]["full_bare_relative_path"])
        if fetch_gate._has_symlink_component(source) or source.is_symlink() or not source.is_dir():
            errors.append("full bare source repository is missing or unsafe")
        else:
            tree, tree_evidence = _git_text(source, policy["git_commands"]["tree_identity"], policy)
            bare, bare_evidence = _git_text(source, policy["git_commands"]["bare_identity"], policy)
            origin, origin_evidence = _git_text(source, policy["git_commands"]["remote_identity"], policy)
            observations["full_bare_source"] = {
                "path": str(source),
                "tree": tree,
                "is_bare": bare,
                "origin": origin,
                "tree_query": tree_evidence,
                "bare_query": bare_evidence,
                "origin_query": origin_evidence,
            }
            if tree != EXPECTED_TREE or bare != "true" or origin != EXPECTED_URL:
                errors.append("full bare source identity differs from the frozen source")

        materialization_parent = fetch_gate._relative_under_root(
            root, policy["source"]["materialization_parent_relative_path"]
        )
        parent_observation: dict[str, Any] = {"path": str(materialization_parent)}
        if materialization_parent.exists() or materialization_parent.is_symlink():
            parent_observation["exists"] = True
            parent_observation.update(_safe_existing_directory(materialization_parent, owner, 0o750))
        else:
            parent_observation["exists"] = False
            parent_observation["may_be_created_by_materialize_source"] = True
        observations["materialization_parent"] = parent_observation

        git_evidence = fetch_gate._trusted_git(policy["trusted_system_tools"]["git"])
        observations["trusted_git"] = git_evidence
        if not git_evidence.get("trusted"):
            errors.append("system Git does not match the frozen root-owned hash")
    except (
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        profile_gate.TargetProfileError,
        fetch_gate.SourceFetchGateError,
        SourceMaterializationError,
    ) as exc:
        errors.append(f"materialization host inspection failed: {exc}")

    core = {
        "schema_version": "smpcc-r8-liquid-target-source-materialization-preflight-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_MATERIALIZATION_PREFLIGHT",
        "captured_at_utc": utc_now(),
        "policy_path": str(POLICY_PATH),
        "policy_sha256": fetch_gate._sha256_regular_file(POLICY_PATH),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": fetch_gate._sha256_regular_file(SCHEMA_PATH),
        "policy_id": policy.get("policy_id"),
        "host_id": policy.get("host_id"),
        "status": (
            "PASS_STATIC_SOURCE_MATERIALIZATION_ADMITTED"
            if not errors
            else "NO_GO_STATIC_SOURCE_MATERIALIZATION"
        ),
        "errors": errors,
        "observations": observations,
        "network_used": False,
        "source_checkout_created": False,
        "source_materialized": False,
        "upstream_code_execution_authorized": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "cmake_or_make_executed": False,
        "compiler_executed": False,
        "next_allowed_stage": (
            "MATERIALIZE_ONE_NEW_SOURCE_ATTEMPT"
            if not errors
            else "REVIEW_NO_GO"
        ),
    }
    return dict(core, preflight_sha256=canonical_hash(core))


def _attempt_paths(root: Path, policy: Mapping[str, Any], attempt_id: str) -> dict[str, Path]:
    if EXPECTED_ATTEMPT_RE.fullmatch(attempt_id) is None:
        raise SourceMaterializationError("attempt ID is not the fixed create-new materialization format")
    if policy["source"]["attempt_id_regex"] != EXPECTED_ATTEMPT_RE.pattern:
        raise SourceMaterializationError("attempt ID policy is not frozen")
    materialization_parent = fetch_gate._relative_under_root(
        root, policy["source"]["materialization_parent_relative_path"]
    )
    attempt_root = materialization_parent / f"{attempt_id}.partial"
    expected_attempt_root = (
        root
        / policy["source"]["attempt_root_relative_template"].replace("<attempt_id>", attempt_id)
    )
    if attempt_root != expected_attempt_root:
        raise SourceMaterializationError("attempt root template does not resolve to the fixed path")
    audits = fetch_gate._relative_under_root(root, policy["receipt"]["directory_relative_path"])
    receipt = audits / f"{attempt_id}.json"
    if EXPECTED_RECEIPT_RE.fullmatch(receipt.name) is None:
        raise SourceMaterializationError("attempt receipt does not match the fixed filename format")
    return {
        "materialization_parent": materialization_parent,
        "attempt_root": attempt_root,
        "receipt": receipt,
        "partial_record": Path(f"{receipt}.partial"),
    }


def _validate_new_attempt_paths(paths: Mapping[str, Path], owner: pwd.struct_passwd) -> None:
    parent = paths["materialization_parent"]
    if fetch_gate._has_symlink_component(parent.parent) or parent.parent.is_symlink() or not parent.parent.is_dir():
        raise SourceMaterializationError("materialization parent ancestor is missing or unsafe")
    parent_metadata = parent.parent.stat()
    if parent_metadata.st_uid != owner.pw_uid or parent_metadata.st_gid != owner.pw_gid:
        raise SourceMaterializationError("materialization parent ancestor ownership differs from ordinary user")
    if parent.exists() or parent.is_symlink():
        _safe_existing_directory(parent, owner, 0o750)

    audits = paths["receipt"].parent
    _safe_existing_directory(audits, owner, 0o750)
    for key in ("attempt_root", "receipt", "partial_record"):
        path = paths[key]
        if path.exists() or path.is_symlink():
            raise SourceMaterializationError(f"create-new path already exists and cannot be reused: {path}")


def _write_json_new(path: Path, value: Mapping[str, Any], mode: int = 0o640) -> str:
    encoded = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(path), flags, mode)
    try:
        os.fchmod(descriptor, mode)
        offset = 0
        while offset < len(encoded):
            offset += os.write(descriptor, encoded[offset:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return hashlib.sha256(encoded).hexdigest()


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | os.O_DIRECTORY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(path), flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _ensure_materialization_parent(
    path: Path, owner: pwd.struct_passwd, policy: Mapping[str, Any]
) -> dict[str, Any]:
    if path.exists() or path.is_symlink():
        evidence = _safe_existing_directory(path, owner, 0o750)
        return dict(evidence, created_new=False)
    if policy["output"]["create_materialization_parent_if_absent"] is not True:
        raise SourceMaterializationError("policy does not permit creating the fixed materialization parent")
    try:
        os.mkdir(path, 0o750)
    except FileExistsError as exc:
        raise SourceMaterializationError("materialization parent appeared during admission") from exc
    except OSError as exc:
        raise SourceMaterializationError(f"could not create materialization parent: {exc}") from exc
    os.chmod(path, 0o750, follow_symlinks=False)
    evidence = _safe_existing_directory(path, owner, 0o750)
    _fsync_directory(path.parent)
    return dict(evidence, created_new=True)


def _reserve_attempt_root(path: Path, owner: pwd.struct_passwd) -> dict[str, Any]:
    try:
        os.mkdir(path, 0o750)
    except FileExistsError as exc:
        raise SourceMaterializationError("attempt root appeared during admission; reuse is forbidden") from exc
    except OSError as exc:
        raise SourceMaterializationError(f"could not reserve materialization attempt root: {exc}") from exc
    os.chmod(path, 0o750, follow_symlinks=False)
    evidence = _safe_existing_directory(path, owner, 0o750)
    _fsync_directory(path.parent)
    return dict(evidence, created_new=True)


def _validate_tree_path(path_text: str, maximum_bytes: int) -> None:
    encoded = path_text.encode("utf-8")
    if not encoded or len(encoded) > maximum_bytes:
        raise SourceMaterializationError("source tree path byte length is outside the frozen limit")
    if any(ord(character) < 32 or ord(character) == 127 for character in path_text):
        raise SourceMaterializationError("source tree path contains a control character")
    parts = path_text.split("/")
    candidate = PurePosixPath(path_text)
    if candidate.is_absolute() or any(part in ("", ".", "..", "~") for part in parts):
        raise SourceMaterializationError("source tree path is not a clean relative POSIX path")


def _read_source_tree(source: Path, policy: Mapping[str, Any]) -> dict[str, Any]:
    """Read and pin the only allowed subset's NUL-delimited Git tree listing."""

    resources = policy["resources"]
    result = fetch_gate._run_owned_bounded(
        fetch_gate._git_argv(
            "--git-dir", str(source), *policy["git_commands"]["source_tree_listing"]
        ),
        timeout_seconds=int(resources["tree_listing_timeout_seconds"]),
        grace_seconds=int(resources["kill_grace_seconds"]),
        output_max_bytes=int(resources["tree_listing_output_max_bytes"]),
        environment=_offline_git_environment(),
    )
    if (
        result["returncode"] != 0
        or result["executor_error"] is not None
        or result["hard_timeout_triggered"]
        or result["hard_kill_for_output_limit"]
    ):
        raise SourceMaterializationError(
            f"offline source tree listing failed: {_compact_command_evidence(result)}"
        )
    listing_text = str(result["stdout_utf8"])
    if "\ufffd" in listing_text:
        raise SourceMaterializationError("source tree listing contained non-UTF-8 data")
    raw = listing_text.encode("utf-8")
    if len(raw) != result["stdout_bytes"] or not raw.endswith(b"\0"):
        raise SourceMaterializationError("source tree listing is not an exact UTF-8 NUL-delimited stream")

    expected = policy["source_subset_expectations"]
    raw_sha256 = hashlib.sha256(raw).hexdigest()
    if raw_sha256 != expected["source_tree_listing_sha256"]:
        raise SourceMaterializationError("source tree listing hash differs from the frozen subset")
    records = raw[:-1].split(b"\0")
    if not records or len(records) > int(resources["max_tree_entries"]):
        raise SourceMaterializationError("source subset entry count exceeds the frozen limit")

    entries: list[dict[str, Any]] = []
    seen_paths: set[str] = set()
    total_bytes = 0
    symlink_count = 0
    gitlink_count = 0
    executable_paths: list[str] = []
    source_prefix = str(policy["source"]["allowed_source_prefix"])
    for record in records:
        if not record or record.count(b"\t") != 1:
            raise SourceMaterializationError("source tree record has an unsafe header/path delimiter")
        header, path_bytes = record.split(b"\t", 1)
        fields = header.split()
        if len(fields) != 4:
            raise SourceMaterializationError("source tree record has an unexpected field count")
        mode_bytes, object_type, object_id_bytes, size_bytes = fields
        try:
            mode = mode_bytes.decode("ascii")
            object_id = object_id_bytes.decode("ascii")
            path_text = path_bytes.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise SourceMaterializationError("source tree record is not valid UTF-8/ASCII") from exc
        _validate_tree_path(path_text, int(resources["max_tree_path_bytes"]))
        if path_text in seen_paths or not path_text.startswith(source_prefix):
            raise SourceMaterializationError("source tree path is duplicated or outside src/source")
        seen_paths.add(path_text)
        if not SHA1_RE.fullmatch(object_id):
            raise SourceMaterializationError("source tree object ID is invalid")
        if mode == "120000":
            symlink_count += 1
            raise SourceMaterializationError("symlink entries are forbidden in source materialization")
        if mode == "160000":
            gitlink_count += 1
            raise SourceMaterializationError("gitlink entries are forbidden in source materialization")
        if mode not in {"100644", "100755"} or object_type != b"blob" or not size_bytes.isdigit():
            raise SourceMaterializationError("source tree entry is not a regular sized blob")
        size = int(size_bytes)
        if size > int(resources["max_single_blob_bytes"]):
            raise SourceMaterializationError("source blob exceeds the per-blob materialization limit")
        total_bytes += size
        if total_bytes > int(resources["max_total_blob_bytes"]):
            raise SourceMaterializationError("source subset exceeds the total materialization limit")
        if mode == "100755":
            executable_paths.append(path_text)
        relative = PurePosixPath(path_text).relative_to("src/source")
        if not relative.parts:
            raise SourceMaterializationError("source tree entry has no output-relative path")
        entries.append(
            {
                "path": path_text,
                "output_relative_path": relative.as_posix(),
                "original_mode": mode,
                "object_id": object_id,
                "size_bytes": size,
            }
        )

    summary = {
        "source_tree_listing_sha256": raw_sha256,
        "entry_count": len(entries),
        "total_blob_bytes": total_bytes,
        "symlink_entry_count": symlink_count,
        "gitlink_entry_count": gitlink_count,
        "input_executable_mode_paths": executable_paths,
    }
    if summary != expected:
        raise SourceMaterializationError("source subset differs from the frozen expected tree")
    return {
        "command": _compact_command_evidence(result),
        "summary": summary,
        "entry_manifest_sha256": canonical_hash(entries),
        "entries": entries,
    }


def _write_all(descriptor: int, payload: bytes) -> None:
    offset = 0
    while offset < len(payload):
        offset += os.write(descriptor, payload[offset:])


def _stream_blob_to_new_file(
    source: Path,
    entry: Mapping[str, Any],
    destination: Path,
    policy: Mapping[str, Any],
    operation_deadline: float,
) -> dict[str, Any]:
    """Copy one exact Git blob to one O_EXCL file without executing its bytes."""

    object_id = str(entry["object_id"])
    expected_size = int(entry["size_bytes"])
    if not SHA1_RE.fullmatch(object_id) or expected_size < 0:
        raise SourceMaterializationError("materialized blob ID or expected size is invalid")
    resources = policy["resources"]
    maximum_size = int(resources["max_single_blob_bytes"])
    stderr_limit = int(resources["blob_stderr_max_bytes"])
    if expected_size > maximum_size or stderr_limit < 1:
        raise SourceMaterializationError("materialized blob exceeds frozen streaming bounds")
    remaining = operation_deadline - time.monotonic()
    if remaining <= 0:
        raise SourceMaterializationError("total materialization deadline elapsed before blob copy")
    blob_timeout = min(float(resources["blob_read_timeout_seconds"]), remaining)
    if blob_timeout <= 0:
        raise SourceMaterializationError("per-blob materialization deadline is invalid")

    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        output_descriptor = os.open(str(destination), flags, 0o640)
    except FileExistsError as exc:
        raise SourceMaterializationError(f"output file already exists and cannot be reused: {destination}") from exc
    except OSError as exc:
        raise SourceMaterializationError(f"could not create new materialized output file: {exc}") from exc
    os.fchmod(output_descriptor, 0o640)

    process: subprocess.Popen[bytes] | None = None
    stdout_thread: threading.Thread | None = None
    stderr_thread: threading.Thread | None = None
    state_lock = threading.Lock()
    stderr_lock = threading.Lock()
    state: dict[str, Any] = {
        "bytes_read": 0,
        "sha256": None,
        "prefix_hex": "",
        "failure": None,
    }
    digest = hashlib.sha256()
    prefix = bytearray()
    stderr = bytearray()
    reader_errors: list[str] = []
    reader_error_lock = threading.Lock()
    stop_requested = threading.Event()
    timed_out = False
    executor_error: str | None = None
    started = time.monotonic()

    def remember_error(label: str, exc: BaseException | str) -> None:
        message = f"{label}: {exc}"
        with reader_error_lock:
            reader_errors.append(message)
        stop_requested.set()

    def set_failure(message: str) -> None:
        with state_lock:
            if state["failure"] is None:
                state["failure"] = message
        stop_requested.set()

    def read_stdout(stream: Any) -> None:
        try:
            while True:
                block = stream.read(65536)
                if not block:
                    return
                with state_lock:
                    next_size = int(state["bytes_read"]) + len(block)
                    if next_size > expected_size or next_size > maximum_size:
                        state["bytes_read"] = next_size
                        state["failure"] = "blob stream exceeded its frozen size"
                        stop_requested.set()
                        return
                    state["bytes_read"] = next_size
                    if len(prefix) < 4:
                        prefix.extend(block[: 4 - len(prefix)])
                    if len(prefix) == 4 and bytes(prefix) == b"\x7fELF":
                        state["failure"] = "ELF magic appeared in the source-only materialization scope"
                        stop_requested.set()
                        return
                digest.update(block)
                _write_all(output_descriptor, block)
        except (OSError, ValueError) as exc:
            remember_error("blob stdout writer", exc)

    def read_stderr(stream: Any) -> None:
        try:
            while True:
                block = stream.read(1024)
                if not block:
                    return
                with stderr_lock:
                    remaining_stderr = stderr_limit - len(stderr)
                    if remaining_stderr > 0:
                        stderr.extend(block[:remaining_stderr])
                    if len(block) > remaining_stderr:
                        set_failure("Git blob stderr exceeded its frozen bound")
                        return
        except (OSError, ValueError) as exc:
            remember_error("blob stderr reader", exc)

    argv = fetch_gate._git_argv(
        "--git-dir", str(source), *policy["git_commands"]["blob"], object_id
    )
    try:
        process = subprocess.Popen(
            tuple(argv),
            cwd="/",
            env=_offline_git_environment(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            close_fds=True,
        )
        if process.stdout is None or process.stderr is None:
            raise SourceMaterializationError("Git blob pipes were not created")
        stdout_thread = threading.Thread(target=read_stdout, args=(process.stdout,), daemon=True)
        stderr_thread = threading.Thread(target=read_stderr, args=(process.stderr,), daemon=True)
        stdout_thread.start()
        stderr_thread.start()
        per_blob_deadline = started + blob_timeout
        while process.poll() is None:
            if stop_requested.is_set():
                fetch_gate._stop_owned_group(process, int(resources["kill_grace_seconds"]))
                break
            if time.monotonic() >= per_blob_deadline or time.monotonic() >= operation_deadline:
                timed_out = True
                fetch_gate._stop_owned_group(process, int(resources["kill_grace_seconds"]))
                break
            time.sleep(0.02)
        try:
            process.wait(timeout=int(resources["kill_grace_seconds"]) + 5)
        except subprocess.TimeoutExpired:
            timed_out = True
            fetch_gate._stop_owned_group(process, 0)
            process.wait(timeout=5)
    except (OSError, subprocess.SubprocessError, SourceMaterializationError) as exc:
        executor_error = str(exc)
        if process is not None and process.poll() is None:
            try:
                fetch_gate._stop_owned_group(process, int(resources["kill_grace_seconds"]))
                process.wait(timeout=int(resources["kill_grace_seconds"]) + 5)
            except (OSError, subprocess.SubprocessError):
                pass
    finally:
        for stream, thread in (
            (process.stdout if process is not None else None, stdout_thread),
            (process.stderr if process is not None else None, stderr_thread),
        ):
            if thread is None:
                continue
            thread.join(timeout=2)
            if thread.is_alive() and stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass
                thread.join(timeout=2)
            if thread.is_alive():
                remember_error("blob output reader did not terminate", "timeout")
        with state_lock:
            bytes_read = int(state["bytes_read"])
            failure = state["failure"]
        if (
            executor_error is None
            and not timed_out
            and not stop_requested.is_set()
            and not reader_errors
            and failure is None
            and process is not None
            and process.returncode == 0
            and bytes_read == expected_size
        ):
            try:
                os.fsync(output_descriptor)
                with state_lock:
                    state["sha256"] = digest.hexdigest()
                    state["prefix_hex"] = bytes(prefix).hex()
            except OSError as exc:
                set_failure(f"could not fsync materialized source file: {exc}")
        os.close(output_descriptor)

    with state_lock:
        observed_sha256 = state["sha256"]
        prefix_hex = state["prefix_hex"]
        failure = state["failure"]
    with stderr_lock:
        stderr_bytes = bytes(stderr)
    return {
        "path": str(destination),
        "object_id": object_id,
        "expected_size_bytes": expected_size,
        "bytes_read": bytes_read,
        "sha256": observed_sha256,
        "prefix_hex": prefix_hex,
        "argv": list(argv),
        "returncode": process.returncode if process is not None else None,
        "hard_timeout_triggered": timed_out,
        "stop_requested": stop_requested.is_set(),
        "failure": failure,
        "stderr_bytes": len(stderr_bytes),
        "stderr_sha256": hashlib.sha256(stderr_bytes).hexdigest(),
        "stderr_utf8": stderr_bytes.decode("utf-8", errors="replace"),
        "reader_errors": reader_errors,
        "executor_error": executor_error,
        "elapsed_seconds": round(time.monotonic() - started, 6),
    }


def _verify_blob_stream(stream: Mapping[str, Any], expected_size: int) -> None:
    if (
        stream.get("returncode") != 0
        or stream.get("executor_error") is not None
        or stream.get("hard_timeout_triggered")
        or stream.get("stop_requested")
        or stream.get("failure") is not None
        or stream.get("reader_errors")
        or stream.get("bytes_read") != expected_size
        or not SHA256_RE.fullmatch(str(stream.get("sha256", "")))
    ):
        compact = {
            key: stream.get(key)
            for key in (
                "path",
                "object_id",
                "expected_size_bytes",
                "bytes_read",
                "returncode",
                "hard_timeout_triggered",
                "stop_requested",
                "failure",
                "reader_errors",
                "executor_error",
                "stderr_bytes",
                "stderr_sha256",
            )
        }
        raise SourceMaterializationError(f"source blob did not materialize safely: {compact}")


def _ensure_parent_directories(attempt_root: Path, output_relative_path: str) -> Path:
    target = attempt_root / "src" / "source"
    for part in PurePosixPath(output_relative_path).parts[:-1]:
        if part in ("", ".", "..", "~"):
            raise SourceMaterializationError("output-relative path contains an unsafe directory component")
        target /= part
        if target.exists() or target.is_symlink():
            if target.is_symlink() or not target.is_dir():
                raise SourceMaterializationError("materialized output parent became unsafe")
            continue
        try:
            os.mkdir(target, 0o750)
        except FileExistsError as exc:
            raise SourceMaterializationError("materialized output parent appeared during copy") from exc
        os.chmod(target, 0o750, follow_symlinks=False)
    return target


def _verify_and_seal_output(
    attempt_root: Path, entries: Sequence[Mapping[str, Any]], copied: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    """Verify exact regular-file topology, then remove every write/execute bit."""

    copied_by_path = {str(record["path"]): record for record in copied}
    expected_files = {str(entry["path"]): entry for entry in entries}
    if set(copied_by_path) != set(expected_files):
        raise SourceMaterializationError("copied blob manifest does not match the frozen source listing")

    expected_directories = {".", "src", "src/source"}
    for path_text in expected_files:
        parts = PurePosixPath(path_text).parts[:-1]
        for length in range(1, len(parts) + 1):
            expected_directories.add(PurePosixPath(*parts[:length]).as_posix())

    observed_files: set[str] = set()
    observed_directories: set[str] = set()
    output_manifest: list[dict[str, Any]] = []
    total_bytes = 0
    for current_root, directory_names, file_names in os.walk(attempt_root, topdown=True, followlinks=False):
        current = Path(current_root)
        current_relative = current.relative_to(attempt_root)
        current_text = "." if str(current_relative) == "." else current_relative.as_posix()
        current_metadata = current.lstat()
        if current.is_symlink() or not stat.S_ISDIR(current_metadata.st_mode):
            raise SourceMaterializationError("materialized tree contains an unsafe directory")
        observed_directories.add(current_text)
        for directory_name in directory_names:
            child = current / directory_name
            child_metadata = child.lstat()
            if child.is_symlink() or not stat.S_ISDIR(child_metadata.st_mode):
                raise SourceMaterializationError("materialized tree contains a symlink or non-directory")
        for file_name in file_names:
            file_path = current / file_name
            file_metadata = file_path.lstat()
            if (
                file_path.is_symlink()
                or not stat.S_ISREG(file_metadata.st_mode)
                or file_metadata.st_nlink != 1
                or stat.S_IMODE(file_metadata.st_mode) != 0o640
                or file_metadata.st_mode & 0o111
            ):
                raise SourceMaterializationError("materialized tree contains an unsafe non-sealed file")
            relative_path = file_path.relative_to(attempt_root).as_posix()
            if relative_path not in expected_files:
                raise SourceMaterializationError("materialized tree contains an unexpected file")
            expected = expected_files[relative_path]
            copied_record = copied_by_path[relative_path]
            observed_hash = fetch_gate._sha256_regular_file(file_path)
            if (
                file_metadata.st_size != int(expected["size_bytes"])
                or observed_hash != copied_record["sha256"]
                or file_metadata.st_uid != os.getuid()
                or file_metadata.st_gid != os.getgid()
            ):
                raise SourceMaterializationError("materialized file metadata or digest differs from copy evidence")
            observed_files.add(relative_path)
            total_bytes += file_metadata.st_size
            output_manifest.append(
                {
                    "path": relative_path,
                    "source_object_id": expected["object_id"],
                    "size_bytes": file_metadata.st_size,
                    "sha256": observed_hash,
                    "original_mode": expected["original_mode"],
                    "sealed_mode_octal": "0440",
                }
            )

    if observed_files != set(expected_files) or observed_directories != expected_directories:
        raise SourceMaterializationError("materialized tree topology differs from the frozen source listing")
    if total_bytes != sum(int(entry["size_bytes"]) for entry in entries):
        raise SourceMaterializationError("materialized tree byte total differs from the frozen source listing")

    for record in output_manifest:
        path = attempt_root / record["path"]
        os.chmod(path, 0o440, follow_symlinks=False)
    for directory_text in sorted(observed_directories, key=lambda value: value.count("/"), reverse=True):
        directory = attempt_root if directory_text == "." else attempt_root / directory_text
        os.chmod(directory, 0o550, follow_symlinks=False)
        _fsync_directory(directory)

    for record in output_manifest:
        metadata = (attempt_root / record["path"]).lstat()
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != 0o440
            or metadata.st_mode & 0o111
        ):
            raise SourceMaterializationError("sealed source file does not have the required read-only mode")
    for directory_text in observed_directories:
        directory = attempt_root if directory_text == "." else attempt_root / directory_text
        metadata = directory.lstat()
        if directory.is_symlink() or not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o550:
            raise SourceMaterializationError("sealed source directory does not have the required read-only mode")

    return {
        "materialized_root": str(attempt_root),
        "materialized_tree_relative_path": "src/source",
        "file_count": len(output_manifest),
        "total_bytes": total_bytes,
        "symlink_count": 0,
        "hardlink_count": 0,
        "all_files_non_executable": True,
        "sealed_directory_mode_octal": "0550",
        "sealed_file_mode_octal": "0440",
        "manifest_sha256": canonical_hash(output_manifest),
        "entries": output_manifest,
    }


def _materialize_tree(
    source: Path, attempt_root: Path, policy: Mapping[str, Any]
) -> dict[str, Any]:
    source_tree = _read_source_tree(source, policy)
    try:
        os.mkdir(attempt_root / "src", 0o750)
        os.mkdir(attempt_root / "src" / "source", 0o750)
    except FileExistsError as exc:
        raise SourceMaterializationError("fresh attempt root already contains source tree entries") from exc
    os.chmod(attempt_root / "src", 0o750, follow_symlinks=False)
    os.chmod(attempt_root / "src" / "source", 0o750, follow_symlinks=False)
    operation_deadline = time.monotonic() + float(policy["resources"]["materialization_timeout_seconds"])
    copied: list[dict[str, Any]] = []
    for entry in source_tree["entries"]:
        destination_parent = _ensure_parent_directories(attempt_root, str(entry["output_relative_path"]))
        destination = destination_parent / PurePosixPath(str(entry["output_relative_path"])).name
        stream = _stream_blob_to_new_file(source, entry, destination, policy, operation_deadline)
        _verify_blob_stream(stream, int(entry["size_bytes"]))
        copied.append(
            {
                "path": str(entry["path"]),
                "source_object_id": str(entry["object_id"]),
                "size_bytes": int(entry["size_bytes"]),
                "sha256": str(stream["sha256"]),
            }
        )
    sealed = _verify_and_seal_output(attempt_root, source_tree["entries"], copied)
    return {
        "source_tree": {
            "command": source_tree["command"],
            "summary": source_tree["summary"],
            "entry_manifest_sha256": source_tree["entry_manifest_sha256"],
        },
        "sealed_output": sealed,
    }


def materialize_source(policy: Mapping[str, Any], attempt_id: str) -> dict[str, Any]:
    """Materialize one new, sealed source-only attempt, or write one NO_GO receipt."""

    preflight = build_preflight(policy)
    profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
    root = _approved_root(profile)
    owner = pwd.getpwnam(str(profile["storage"]["owner"]))
    paths = _attempt_paths(root, policy, attempt_id)
    _validate_new_attempt_paths(paths, owner)

    if preflight["status"] != "PASS_STATIC_SOURCE_MATERIALIZATION_ADMITTED":
        receipt = {
            "schema_version": "smpcc-r8-liquid-target-source-materialization-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_MATERIALIZATION_RECEIPT",
            "created_at_utc": utc_now(),
            "attempt_id": attempt_id,
            "status": "NO_GO_STATIC_PRECHECK",
            "preflight": preflight,
            "network_used": False,
            "source_checkout_created": False,
            "source_materialized": False,
            "upstream_code_execution_authorized": False,
            "upstream_code_executed": False,
            "precompiled_binary_executed": False,
            "cmake_or_make_executed": False,
            "compiler_executed": False,
            "next_allowed_stage": "REVIEW_NO_GO",
        }
        receipt["receipt_sha256"] = canonical_hash(receipt)
        _write_json_new(paths["receipt"], receipt)
        return receipt

    partial_record = {
        "schema_version": "smpcc-r8-liquid-target-source-materialization-partial-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_MATERIALIZATION_PARTIAL",
        "created_at_utc": utc_now(),
        "attempt_id": attempt_id,
        "status": "STARTED_IMMUTABLE_PARTIAL",
        "preflight_sha256": preflight["preflight_sha256"],
        "planned_attempt_root": str(paths["attempt_root"]),
        "source_commit": EXPECTED_COMMIT,
        "source_tree": EXPECTED_TREE,
        "network_used": False,
        "source_checkout_created": False,
        "upstream_code_executed": False,
    }
    partial_record["partial_sha256"] = canonical_hash(partial_record)
    _write_json_new(paths["partial_record"], partial_record)

    results: dict[str, Any] = {}
    failure: str | None = None
    try:
        results["materialization_parent"] = _ensure_materialization_parent(
            paths["materialization_parent"], owner, policy
        )
        results["attempt_root"] = _reserve_attempt_root(paths["attempt_root"], owner)
        source = fetch_gate._relative_under_root(root, policy["source"]["full_bare_relative_path"])
        results["materialization"] = _materialize_tree(source, paths["attempt_root"], policy)
    except (
        OSError,
        ValueError,
        SourceMaterializationError,
        fetch_gate.SourceFetchGateError,
    ) as exc:
        failure = str(exc)

    status = (
        "PASS_STATIC_SOURCE_MATERIALIZATION"
        if failure is None
        else "NO_GO_STATIC_SOURCE_MATERIALIZATION"
    )
    receipt = {
        "schema_version": "smpcc-r8-liquid-target-source-materialization-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_MATERIALIZATION_RECEIPT",
        "created_at_utc": utc_now(),
        "attempt_id": attempt_id,
        "status": status,
        "preflight": preflight,
        "partial_start_record": str(paths["partial_record"]),
        "materialized_attempt_root": str(paths["attempt_root"]),
        "results": results,
        "failure": failure,
        "network_used": False,
        "source_checkout_created": False,
        "source_materialized": failure is None,
        "upstream_code_execution_authorized": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "cmake_or_make_executed": False,
        "compiler_executed": False,
        "ros_or_gazebo_started": False,
        "gpu_device_exposed": False,
        "system_packages_changed": False,
        "sudo_used": False,
        "next_allowed_stage": (
            "SEPARATE_BUILD_SANDBOX_EXECUTION_ADMISSION_REQUIRED"
            if failure is None
            else "REVIEW_MATERIALIZATION_FAILURE"
        ),
    }
    receipt["receipt_sha256"] = canonical_hash(receipt)
    _write_json_new(paths["receipt"], receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="read-only materialization admission check")
    materialize = subparsers.add_parser(
        "materialize-source",
        help="copy only frozen src/source blobs into one new sealed attempt root",
    )
    materialize.add_argument("--attempt-id", required=True)
    args = parser.parse_args(argv)
    try:
        policy = _read_json_object(POLICY_PATH)
        if args.command == "self-check":
            report = build_preflight(policy)
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
            return 0 if report["status"] == "PASS_STATIC_SOURCE_MATERIALIZATION_ADMITTED" else 2
        receipt = materialize_source(policy, args.attempt_id)
        summary = {
            "attempt_id": receipt["attempt_id"],
            "status": receipt["status"],
            "materialized_attempt_root": receipt.get("materialized_attempt_root"),
            "receipt_sha256": receipt["receipt_sha256"],
            "next_allowed_stage": receipt["next_allowed_stage"],
        }
        print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if receipt["status"] == "PASS_STATIC_SOURCE_MATERIALIZATION" else 2
    except (
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        profile_gate.TargetProfileError,
        fetch_gate.SourceFetchGateError,
        SourceMaterializationError,
    ) as exc:
        print(f"R8_LIQUID_TARGET_SOURCE_MATERIALIZATION_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
