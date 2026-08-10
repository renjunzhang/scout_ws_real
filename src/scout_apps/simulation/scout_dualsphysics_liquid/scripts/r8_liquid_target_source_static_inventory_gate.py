#!/usr/bin/env python3
"""Fail-closed offline inventory of the fetched DualSPHysics bare repository.

This gate is deliberately not a checkout, extraction, build, or execution
tool.  It uses only the frozen system Git suite with every protocol disabled.
It reads Git tree metadata and streams the already-pinned candidate blobs only
to calculate hashes and inspect their first bytes for ELF magic.  Source bytes
are never written outside the existing bare object database or executed.
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
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_profile_gate as profile_gate  # noqa: E402
import r8_liquid_target_source_fetch_gate as fetch_gate  # noqa: E402


POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_source_static_inventory_policy_v1.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_source_static_inventory_policy_v1.json"
RECEIPT_RE = re.compile(r"^u2_full_source_static_inventory_v1_[0-9]{8}T[0-9]{6}Z\.json$")
SHA1_RE = re.compile(r"^[0-9a-f]{40}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
EXPECTED_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
EXPECTED_TREE = "cef458cb358712f4694b9d2148f638440418e9dc"
EXPECTED_URL = "https://github.com/DualSPHysics/DualSPHysics.git"
EXPECTED_FETCH_RECEIPT = "audits/u2_full_source_fetch_attempt_3_20260806T074600Z.json"
EXPECTED_FETCH_FILE_SHA256 = "23745c0076c2c06fc549665c89f4e039761832bab66f4f568f63eff696ead18d"
EXPECTED_FETCH_RECEIPT_SHA256 = "90bd0c1cc03b022dc42dd4b0368a84f46af10b9b5b8c54108765d1fd16df0426"
EXPECTED_FULL_BARE_RELATIVE = (
    "dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git"
)

EXPECTED_EXECUTABLE_MODE_PATHS = [
    "bin/linux/DsphConfig.xml",
    "bin/linux/GenCase_linux64",
    "bin/linux/IsoSurface_linux64",
    "bin/linux/MeasureTool_linux64",
    "bin/linux/PartVTKOut_linux64",
    "bin/linux/PartVTK_linux64",
    "bin/linux/README.txt",
    "bin/linux/VERSION_INFO.txt",
    "bin/linux/libChronoEngine.so",
    "bin/linux/libdsphchrono.so",
    "src/source/JDsGaugeItem.cpp",
    "src/source/JDsGaugeItem.h",
    "src/source/JDsGaugeSystem.cpp",
    "src/source/JDsGaugeSystem.h",
    "src/source/JDsGauge_ker.cu",
    "src/source/JDsGauge_ker.h",
]

EXPECTED_BIN_LINUX_ENTRIES = [
    {
        "path": "bin/linux/DsphConfig.xml",
        "mode": "100755",
        "object_id": "d99d0f85d19a622f7faf2d5f1195b11f17951846",
        "size_bytes": 293,
        "expected_elf": False,
    },
    {
        "path": "bin/linux/GenCase_linux64",
        "mode": "100755",
        "object_id": "4db4dadbae8f6e3885cad3bb752340812036278a",
        "size_bytes": 5809384,
        "expected_elf": True,
    },
    {
        "path": "bin/linux/IsoSurface_linux64",
        "mode": "100755",
        "object_id": "99308e40373cb62cfd0189c8e06e611636b18fd8",
        "size_bytes": 6210152,
        "expected_elf": True,
    },
    {
        "path": "bin/linux/MeasureTool_linux64",
        "mode": "100755",
        "object_id": "99308e40373cb62cfd0189c8e06e611636b18fd8",
        "size_bytes": 6210152,
        "expected_elf": True,
    },
    {
        "path": "bin/linux/PartVTKOut_linux64",
        "mode": "100755",
        "object_id": "99308e40373cb62cfd0189c8e06e611636b18fd8",
        "size_bytes": 6210152,
        "expected_elf": True,
    },
    {
        "path": "bin/linux/PartVTK_linux64",
        "mode": "100755",
        "object_id": "99308e40373cb62cfd0189c8e06e611636b18fd8",
        "size_bytes": 6210152,
        "expected_elf": True,
    },
    {
        "path": "bin/linux/README.txt",
        "mode": "100755",
        "object_id": "04ed5435c5a4898a91a1e4389e51e00ed4ea1d3b",
        "size_bytes": 231,
        "expected_elf": False,
    },
    {
        "path": "bin/linux/VERSION_INFO.txt",
        "mode": "100755",
        "object_id": "92ffbc0b0ef63b353f6bf36a3e96d8630a5ae778",
        "size_bytes": 284,
        "expected_elf": False,
    },
    {
        "path": "bin/linux/libChronoEngine.so",
        "mode": "100755",
        "object_id": "f0892cc22c7ee34bb25a815333c6a14521d77531",
        "size_bytes": 28283072,
        "expected_elf": True,
    },
    {
        "path": "bin/linux/libdsphchrono.so",
        "mode": "100755",
        "object_id": "87330bada49b0e97f636c0c8919d39af5f5f4f82",
        "size_bytes": 646160,
        "expected_elf": True,
    },
]


class StaticInventoryError(RuntimeError):
    """An ambiguity in offline source inspection is a fail-closed NO-GO."""


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_json_object(path: Path) -> dict[str, Any]:
    value = fetch_gate._read_json_object(path)
    if not isinstance(value, dict):
        raise StaticInventoryError(f"JSON file is not an object: {path}")
    return value


def _offline_git_environment() -> dict[str, str]:
    """Disable every transport and interactive configuration for local Git plumbing."""

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
    """Keep command metadata but never serialize untrusted tree/blob content."""

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
        raise StaticInventoryError(f"offline Git identity query failed: {_compact_command_evidence(result)}")
    return str(result["stdout_utf8"]).strip(), _compact_command_evidence(result)


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate frozen semantics without reading the host or a source object."""

    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-source-static-inventory-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_STATIC_INVENTORY_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_SOURCE_STATIC_INVENTORY_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "static_blob_read_authorized": True,
        "upstream_code_execution_authorized": False,
        "allowed_gate_commands": ["self-check", "write-static-inventory-receipt"],
    }
    for key, value in expected_top.items():
        if policy.get(key) != value:
            errors.append(f"policy field mismatch: {key}")

    profile = policy.get("required_target_profile")
    expected_profile = {
        "profile_id": "LIQUID_ZRJ_MSI_U2404_P0_V1",
        "canonical_sha256": "fec0b980ca46df04d877290b73b3f3406af97887ccc20c9d52d58092c36f8838",
        "required_status": "PASS_PROFILE_READ_ONLY_ONLY",
    }
    if profile != expected_profile:
        errors.append("required target profile differs from the frozen MSI profile")

    fetch_receipt = policy.get("required_source_fetch_receipt")
    expected_fetch_receipt = {
        "receipt_relative_path": EXPECTED_FETCH_RECEIPT,
        "file_sha256": EXPECTED_FETCH_FILE_SHA256,
        "receipt_sha256": EXPECTED_FETCH_RECEIPT_SHA256,
        "status": "PASS_FULL_BARE_SOURCE_FETCH",
        "next_allowed_stage": "OFFLINE_FULL_BLOB_STATIC_INVENTORY_ONLY",
    }
    if fetch_receipt != expected_fetch_receipt:
        errors.append("required full-source fetch receipt differs from the frozen evidence")

    source = policy.get("source")
    expected_source = {
        "repository_url": EXPECTED_URL,
        "commit": EXPECTED_COMMIT,
        "tree": EXPECTED_TREE,
        "full_bare_relative_path": EXPECTED_FULL_BARE_RELATIVE,
        "no_checkout": True,
        "no_network": True,
        "no_submodules": True,
        "no_lfs_smudge": True,
        "no_hooks": True,
        "precompiled_binaries_execution": "forbidden",
    }
    if source != expected_source:
        errors.append("source declaration differs from the frozen bare/offline source")

    tools = policy.get("trusted_system_tools")
    expected_tools = {
        "git": {
            "path": str(fetch_gate.GIT_PATH),
            "sha256": "2a8c18fbf43da9f692d75474c72bea9dfd796c260b0f3dfe456376abc3bbd668",
        },
        "git_remote_https": {
            "path": str(fetch_gate.GIT_REMOTE_HTTPS_LINK),
            "symlink_target": "git-remote-http",
            "resolved_path": str(fetch_gate.GIT_REMOTE_HTTP),
            "sha256": "8353a5124ddb2281838bc54bd97c21cbecfe900615fdb60b281198c5058ddb5b",
        },
    }
    if tools != expected_tools:
        errors.append("trusted Git suite differs from the frozen system declarations")

    expected_commands = {
        "tree_identity": ["rev-parse", f"{EXPECTED_COMMIT}^{{tree}}"],
        "bare_identity": ["rev-parse", "--is-bare-repository"],
        "remote_identity": ["remote", "get-url", "origin"],
        "tree_listing": ["ls-tree", "-r", "-l", "-z", EXPECTED_COMMIT],
        "blob": ["cat-file", "blob"],
    }
    if policy.get("git_commands") != expected_commands:
        errors.append("offline Git argv policy differs from the frozen inventory commands")

    expected_inventory = {
        "tree_entry_count": 768,
        "total_blob_bytes": 293416502,
        "executable_mode_paths": EXPECTED_EXECUTABLE_MODE_PATHS,
        "symlink_entry_count": 0,
        "gitlink_entry_count": 0,
        "bin_linux_entries": EXPECTED_BIN_LINUX_ENTRIES,
    }
    if policy.get("inventory_expectations") != expected_inventory:
        errors.append("static inventory expectations differ from the frozen source tree")

    expected_resources = {
        "short_command_timeout_seconds": 60,
        "tree_listing_timeout_seconds": 60,
        "blob_read_timeout_seconds": 60,
        "kill_grace_seconds": 5,
        "short_command_output_max_bytes": 16384,
        "tree_listing_output_max_bytes": 524288,
        "max_tree_entries": 1024,
        "max_tree_path_bytes": 1024,
        "max_total_blob_bytes": 536870912,
        "max_candidate_blob_bytes": 67108864,
        "max_unique_candidate_blob_bytes": 67108864,
        "blob_stderr_max_bytes": 16384,
        "blob_prefix_bytes": 16,
        "directory_mode_octal": "0750",
        "receipt_mode_octal": "0640",
    }
    if policy.get("resources") != expected_resources:
        errors.append("offline static-inventory resource policy differs from frozen limits")

    receipt = policy.get("receipt")
    if receipt != {
        "directory_relative_path": "audits",
        "filename_regex": RECEIPT_RE.pattern,
    }:
        errors.append("static inventory receipt policy is invalid")

    invariants = policy.get("invariants")
    expected_invariants = {
        "no_sudo",
        "no_apt",
        "no_network",
        "no_checkout",
        "no_source_file_execution",
        "no_precompiled_binary_execution",
        "no_cmake_or_make",
        "no_ros_or_gazebo",
        "no_gpu_device",
        "no_destructive_cleanup",
        "no_retry_in_existing_receipt_path",
    }
    if not isinstance(invariants, dict) or set(invariants) != expected_invariants or any(
        invariants.get(key) is not True for key in expected_invariants
    ):
        errors.append("offline static-inventory safety invariants are incomplete")
    return errors


def _approved_root(profile: Mapping[str, Any]) -> Path:
    root = Path(str(profile.get("storage", {}).get("approved_root", "")))
    if root != Path("/home/zrj/scout_liquid_lab"):
        raise StaticInventoryError("target profile approved root differs from the frozen MSI root")
    return root


def _verify_fetch_receipt(root: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    receipt = fetch_gate._relative_under_root(root, str(expected.get("receipt_relative_path", "")))
    if not receipt.is_file() or receipt.is_symlink():
        raise StaticInventoryError("required full-source fetch receipt is missing or unsafe")
    observed_hash = fetch_gate._sha256_regular_file(receipt)
    if observed_hash != expected.get("file_sha256"):
        raise StaticInventoryError("required full-source fetch receipt file hash mismatch")
    value = _read_json_object(receipt)
    results = value.get("results")
    if not isinstance(results, dict):
        raise StaticInventoryError("required full-source fetch receipt lacks result evidence")
    verification = results.get("verification")
    if not isinstance(verification, dict):
        raise StaticInventoryError("required full-source fetch receipt lacks verification evidence")
    object_walk = verification.get("reachable_object_completeness")
    if not isinstance(object_walk, dict):
        raise StaticInventoryError("required full-source fetch receipt lacks reachable-object evidence")
    if (
        value.get("status") != expected.get("status")
        or value.get("receipt_sha256") != expected.get("receipt_sha256")
        or value.get("next_allowed_stage") != expected.get("next_allowed_stage")
        or value.get("source_checkout_created") is not False
        or value.get("upstream_code_executed") is not False
        or verification.get("tree") != EXPECTED_TREE
        or verification.get("origin") != EXPECTED_URL
        or object_walk.get("missing_object_count") != 0
        or object_walk.get("expected_commit_seen") is not True
        or object_walk.get("expected_tree_seen") is not True
    ):
        raise StaticInventoryError("required full-source fetch receipt content does not prove complete bare source")
    return {
        "path": str(receipt),
        "file_sha256": observed_hash,
        "receipt_sha256": value["receipt_sha256"],
        "status": value["status"],
        "next_allowed_stage": value["next_allowed_stage"],
        "reachable_object_records": object_walk.get("reachable_object_records"),
        "missing_object_count": object_walk.get("missing_object_count"),
    }


def build_preflight(policy: Mapping[str, Any]) -> dict[str, Any]:
    """Read-only admission for the offline inventory; no source blob is streamed here."""

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
        required_profile = policy["required_target_profile"]
        if profile_hash != required_profile["canonical_sha256"]:
            errors.append("target profile canonical hash differs from offline inventory policy")
        profile_report = profile_gate.build_report(profile)
        observations["target_profile"]["status"] = profile_report["status"]
        observations["target_profile"]["report_sha256"] = profile_report["report_sha256"]
        if profile_report["status"] != required_profile["required_status"]:
            errors.append("target profile read-only self-check did not pass")

        root = _approved_root(profile)
        observations["approved_root"] = str(root)
        if fetch_gate._has_symlink_component(root) or root.is_symlink() or not root.is_dir():
            errors.append("approved root is missing or unsafe")
        else:
            root_metadata = root.stat()
            owner = pwd.getpwnam(str(profile["storage"]["owner"]))
            observations["invoker"] = {"uid": os.getuid(), "euid": os.geteuid(), "gid": os.getgid()}
            if os.getuid() != owner.pw_uid or os.geteuid() != owner.pw_uid:
                errors.append("offline source inventory must run as the target ordinary user, never through sudo")
            if root_metadata.st_uid != owner.pw_uid or root_metadata.st_gid != owner.pw_gid:
                errors.append("approved root ownership differs from target profile")
            observations["required_source_fetch_receipt"] = _verify_fetch_receipt(
                root, policy["required_source_fetch_receipt"]
            )
            source = fetch_gate._relative_under_root(root, policy["source"]["full_bare_relative_path"])
            if fetch_gate._has_symlink_component(source) or source.is_symlink() or not source.is_dir():
                errors.append("full bare source repository is missing or unsafe")
            else:
                observations["full_bare_source"] = {"path": str(source)}
                tree, tree_evidence = _git_text(source, policy["git_commands"]["tree_identity"], policy)
                bare, bare_evidence = _git_text(source, policy["git_commands"]["bare_identity"], policy)
                origin, origin_evidence = _git_text(source, policy["git_commands"]["remote_identity"], policy)
                observations["full_bare_source"].update(
                    {
                        "tree": tree,
                        "is_bare": bare,
                        "origin": origin,
                        "tree_query": tree_evidence,
                        "bare_query": bare_evidence,
                        "origin_query": origin_evidence,
                    }
                )
                if tree != policy["source"]["tree"] or bare != "true" or origin != policy["source"]["repository_url"]:
                    errors.append("full bare source identity differs from frozen offline policy")

        git_evidence = fetch_gate._trusted_git_suite(policy["trusted_system_tools"])
        observations["trusted_git_suite"] = git_evidence
        if not git_evidence.get("trusted"):
            errors.append("system Git HTTPS suite does not match frozen root-owned hashes")
    except (
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        profile_gate.TargetProfileError,
        fetch_gate.SourceFetchGateError,
        StaticInventoryError,
    ) as exc:
        errors.append(f"offline inventory host inspection failed: {exc}")

    core = {
        "schema_version": "smpcc-r8-liquid-target-source-static-inventory-preflight-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_STATIC_INVENTORY_PREFLIGHT",
        "captured_at_utc": utc_now(),
        "policy_path": str(POLICY_PATH),
        "policy_sha256": fetch_gate._sha256_regular_file(POLICY_PATH),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": fetch_gate._sha256_regular_file(SCHEMA_PATH),
        "policy_id": policy.get("policy_id"),
        "host_id": policy.get("host_id"),
        "status": "PASS_OFFLINE_STATIC_INVENTORY_ADMITTED" if not errors else "NO_GO_OFFLINE_STATIC_INVENTORY",
        "errors": errors,
        "observations": observations,
        "network_used": False,
        "source_checkout_created": False,
        "upstream_code_execution_authorized": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "next_allowed_stage": "WRITE_ONE_STATIC_INVENTORY_RECEIPT" if not errors else "REVIEW_NO_GO",
    }
    return dict(core, preflight_sha256=canonical_hash(core))


def validate_receipt_path(path: Path, root: Path) -> tuple[Path, Path]:
    if not fetch_gate._is_clean_absolute(path) or not fetch_gate._is_clean_absolute(root):
        raise StaticInventoryError("receipt and approved root must be clean absolute paths")
    audits = root / "audits"
    if fetch_gate._has_symlink_component(audits) or audits.is_symlink() or not audits.is_dir():
        raise StaticInventoryError("audit directory is missing or unsafe")
    if path.parent != audits or RECEIPT_RE.fullmatch(path.name) is None:
        raise StaticInventoryError("receipt path is not the fixed offline-inventory audit filename")
    partial = Path(f"{path}.partial")
    if path.exists() or path.is_symlink() or partial.exists() or partial.is_symlink():
        raise StaticInventoryError("receipt final or partial path already exists")
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


def _validate_tree_path(path_text: str, maximum_bytes: int) -> None:
    encoded = path_text.encode("utf-8")
    if len(encoded) < 1 or len(encoded) > maximum_bytes:
        raise StaticInventoryError("tree path byte length is outside the frozen limit")
    if any(ord(character) < 32 or ord(character) == 127 for character in path_text):
        raise StaticInventoryError("tree path contains a control character")
    raw_parts = path_text.split("/")
    candidate = PurePosixPath(path_text)
    if candidate.is_absolute() or any(part in ("", ".", "..") for part in raw_parts):
        raise StaticInventoryError("tree path is not a clean relative POSIX path")


def _read_tree_inventory(source: Path, policy: Mapping[str, Any]) -> dict[str, Any]:
    """Read one NUL-delimited tree listing and reject unexpected tree topology."""

    resources = policy["resources"]
    result = fetch_gate._run_owned_bounded(
        fetch_gate._git_argv("--git-dir", str(source), *policy["git_commands"]["tree_listing"]),
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
        raise StaticInventoryError(f"offline tree listing failed: {_compact_command_evidence(result)}")
    text = str(result["stdout_utf8"])
    if "\ufffd" in text:
        raise StaticInventoryError("tree listing contained non-UTF-8 data and was not parsed")
    raw = text.encode("utf-8")
    if len(raw) != result["stdout_bytes"] or not raw.endswith(b"\0"):
        raise StaticInventoryError("tree listing is not an exact UTF-8 NUL-delimited byte stream")

    records = raw[:-1].split(b"\0")
    maximum_entries = int(resources["max_tree_entries"])
    maximum_path_bytes = int(resources["max_tree_path_bytes"])
    maximum_total_blob_bytes = int(resources["max_total_blob_bytes"])
    if not records or len(records) > maximum_entries:
        raise StaticInventoryError("tree entry count exceeds the frozen inspection limit")
    paths: set[str] = set()
    executable_paths: list[str] = []
    bin_linux_entries: list[dict[str, Any]] = []
    total_blob_bytes = 0
    symlink_entries = 0
    gitlink_entries = 0
    for record in records:
        if not record or record.count(b"\t") != 1:
            raise StaticInventoryError("tree record has an unsafe header/path delimiter")
        header, path_bytes = record.split(b"\t", 1)
        # ``git ls-tree -l`` left-pads the decimal size field; split on a run
        # of ASCII whitespace rather than treating the presentation padding as
        # extra semantic fields.
        fields = header.split()
        if len(fields) != 4:
            raise StaticInventoryError("tree record has an unexpected field count")
        mode_bytes, object_type, object_id_bytes, size_bytes = fields
        try:
            mode = mode_bytes.decode("ascii")
            object_id = object_id_bytes.decode("ascii")
            path_text = path_bytes.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise StaticInventoryError("tree record is not valid UTF-8/ASCII") from exc
        _validate_tree_path(path_text, maximum_path_bytes)
        if path_text in paths:
            raise StaticInventoryError("tree listing has a duplicate path")
        paths.add(path_text)
        if mode not in {"100644", "100755", "120000", "160000"} or not SHA1_RE.fullmatch(object_id):
            raise StaticInventoryError("tree record has an unsupported mode or object ID")
        if mode == "160000":
            if object_type != b"commit" or size_bytes != b"-":
                raise StaticInventoryError("gitlink tree record is malformed")
            gitlink_entries += 1
            continue
        if object_type != b"blob" or not size_bytes.isdigit():
            raise StaticInventoryError("non-gitlink tree record is not a sized blob")
        size_bytes_int = int(size_bytes)
        total_blob_bytes += size_bytes_int
        if total_blob_bytes > maximum_total_blob_bytes:
            raise StaticInventoryError("tree blob total exceeds the frozen inspection limit")
        if mode == "120000":
            symlink_entries += 1
        if mode == "100755":
            executable_paths.append(path_text)
        if path_text.startswith("bin/linux/"):
            bin_linux_entries.append(
                {
                    "path": path_text,
                    "mode": mode,
                    "object_id": object_id,
                    "size_bytes": size_bytes_int,
                }
            )

    summary = {
        "tree_entry_count": len(records),
        "total_blob_bytes": total_blob_bytes,
        "executable_mode_paths": executable_paths,
        "symlink_entry_count": symlink_entries,
        "gitlink_entry_count": gitlink_entries,
        "bin_linux_entries": bin_linux_entries,
    }
    expected = policy["inventory_expectations"]
    expected_summary = {
        "tree_entry_count": expected["tree_entry_count"],
        "total_blob_bytes": expected["total_blob_bytes"],
        "executable_mode_paths": expected["executable_mode_paths"],
        "symlink_entry_count": expected["symlink_entry_count"],
        "gitlink_entry_count": expected["gitlink_entry_count"],
        "bin_linux_entries": [
            {
                "path": entry["path"],
                "mode": entry["mode"],
                "object_id": entry["object_id"],
                "size_bytes": entry["size_bytes"],
            }
            for entry in expected["bin_linux_entries"]
        ],
    }
    if summary != expected_summary:
        raise StaticInventoryError("offline tree inventory differs from the frozen source expectations")
    return {"command": _compact_command_evidence(result), "inventory": summary}


def _stream_candidate_blob(
    source: Path, object_id: str, expected_size: int, policy: Mapping[str, Any]
) -> dict[str, Any]:
    """Hash one known blob through Git without retaining or writing its content."""

    if not SHA1_RE.fullmatch(object_id) or expected_size < 0:
        raise StaticInventoryError("candidate blob ID or expected size is invalid")
    resources = policy["resources"]
    maximum_blob_bytes = int(resources["max_candidate_blob_bytes"])
    prefix_limit = int(resources["blob_prefix_bytes"])
    stderr_limit = int(resources["blob_stderr_max_bytes"])
    timeout_seconds = int(resources["blob_read_timeout_seconds"])
    grace_seconds = int(resources["kill_grace_seconds"])
    if expected_size > maximum_blob_bytes or prefix_limit < 4 or stderr_limit < 1 or timeout_seconds < 1:
        raise StaticInventoryError("candidate blob exceeds the frozen streaming limits")

    argv = fetch_gate._git_argv(
        "--git-dir", str(source), *policy["git_commands"]["blob"], object_id
    )
    digest = hashlib.sha256()
    prefix = bytearray()
    stderr = bytearray()
    stdout_bytes = 0
    stderr_lock = threading.Lock()
    reader_errors: list[str] = []
    reader_error_lock = threading.Lock()
    size_limit_hit = threading.Event()
    stderr_limit_hit = threading.Event()
    process: subprocess.Popen[bytes] | None = None
    stdout_thread: threading.Thread | None = None
    stderr_thread: threading.Thread | None = None
    executor_error: str | None = None
    timed_out = False

    def remember_reader_error(label: str, exc: BaseException) -> None:
        with reader_error_lock:
            reader_errors.append(f"{label}: {exc}")

    def read_stdout(stream: Any) -> None:
        nonlocal stdout_bytes
        try:
            while True:
                block = stream.read(65536)
                if not block:
                    return
                stdout_bytes += len(block)
                if stdout_bytes > expected_size or stdout_bytes > maximum_blob_bytes:
                    size_limit_hit.set()
                    return
                digest.update(block)
                if len(prefix) < prefix_limit:
                    prefix.extend(block[: prefix_limit - len(prefix)])
        except (OSError, ValueError) as exc:
            remember_reader_error("blob stdout reader", exc)

    def read_stderr(stream: Any) -> None:
        try:
            while True:
                block = stream.read(1024)
                if not block:
                    return
                with stderr_lock:
                    remaining = stderr_limit - len(stderr)
                    if remaining > 0:
                        stderr.extend(block[:remaining])
                    if len(block) > remaining:
                        stderr_limit_hit.set()
                        return
        except (OSError, ValueError) as exc:
            remember_reader_error("blob stderr reader", exc)

    started = time.monotonic()
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
            raise StaticInventoryError("candidate blob output pipes were not created")
        stdout_thread = threading.Thread(target=read_stdout, args=(process.stdout,), daemon=True)
        stderr_thread = threading.Thread(target=read_stderr, args=(process.stderr,), daemon=True)
        stdout_thread.start()
        stderr_thread.start()
        deadline = started + timeout_seconds
        while process.poll() is None:
            if size_limit_hit.is_set() or stderr_limit_hit.is_set():
                fetch_gate._stop_owned_group(process, grace_seconds)
                break
            if time.monotonic() >= deadline:
                timed_out = True
                fetch_gate._stop_owned_group(process, grace_seconds)
                break
            time.sleep(0.05)
        try:
            process.wait(timeout=grace_seconds + 5)
        except subprocess.TimeoutExpired:
            timed_out = True
            fetch_gate._stop_owned_group(process, 0)
            process.wait(timeout=5)
    except (OSError, subprocess.SubprocessError, StaticInventoryError) as exc:
        executor_error = str(exc)
        if process is not None and process.poll() is None:
            try:
                fetch_gate._stop_owned_group(process, grace_seconds)
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
                reader_errors.append("candidate blob output reader did not terminate")

    with stderr_lock:
        stderr_bytes = bytes(stderr)
    classification = "ELF" if bytes(prefix).startswith(b"\x7fELF") else "NON_ELF"
    return {
        "argv": list(argv),
        "returncode": process.returncode if process is not None else None,
        "hard_timeout_triggered": timed_out,
        "hard_kill_for_size_limit": size_limit_hit.is_set(),
        "hard_kill_for_stderr_limit": stderr_limit_hit.is_set(),
        "expected_bytes": expected_size,
        "bytes_read": stdout_bytes,
        "sha256": digest.hexdigest(),
        "prefix_hex": bytes(prefix).hex(),
        "classification": classification,
        "stderr_captured_bytes": len(stderr_bytes),
        "stderr_max_bytes": stderr_limit,
        "stderr_sha256": hashlib.sha256(stderr_bytes).hexdigest(),
        "stderr_utf8": stderr_bytes.decode("utf-8", errors="replace"),
        "reader_errors": reader_errors,
        "executor_error": executor_error,
        "elapsed_seconds": round(time.monotonic() - started, 6),
    }


def _verify_candidate_stream(result: Mapping[str, Any], expected_size: int) -> None:
    if (
        result.get("returncode") != 0
        or result.get("executor_error") is not None
        or result.get("hard_timeout_triggered")
        or result.get("hard_kill_for_size_limit")
        or result.get("hard_kill_for_stderr_limit")
        or result.get("reader_errors")
        or result.get("bytes_read") != expected_size
        or not SHA256_RE.fullmatch(str(result.get("sha256", "")))
    ):
        raise StaticInventoryError(f"candidate blob stream did not complete safely: {result}")


def write_static_inventory_receipt(policy: Mapping[str, Any], receipt_path: Path) -> dict[str, Any]:
    """Write one append-only offline inventory receipt after a fresh preflight."""

    preflight = build_preflight(policy)
    profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
    root = _approved_root(profile)
    final_path, partial_path = validate_receipt_path(receipt_path, root)
    if preflight["status"] != "PASS_OFFLINE_STATIC_INVENTORY_ADMITTED":
        receipt = {
            "schema_version": "smpcc-r8-liquid-target-source-static-inventory-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_STATIC_INVENTORY_RECEIPT",
            "created_at_utc": utc_now(),
            "status": "NO_GO_STATIC_PRECHECK",
            "preflight": preflight,
            "network_used": False,
            "source_checkout_created": False,
            "upstream_code_executed": False,
            "precompiled_binary_executed": False,
        }
        receipt["receipt_sha256"] = canonical_hash(receipt)
        _write_json_new(final_path, receipt)
        return receipt

    source = fetch_gate._relative_under_root(root, policy["source"]["full_bare_relative_path"])
    started_record = {
        "schema_version": "smpcc-r8-liquid-target-source-static-inventory-partial-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_STATIC_INVENTORY_PARTIAL",
        "created_at_utc": utc_now(),
        "status": "STARTED_IMMUTABLE_PARTIAL",
        "preflight_sha256": preflight["preflight_sha256"],
        "source_path": str(source),
        "network_used": False,
        "source_checkout_created": False,
        "upstream_code_executed": False,
    }
    started_record["partial_sha256"] = canonical_hash(started_record)
    _write_json_new(partial_path, started_record)

    results: dict[str, Any] = {}
    failure: str | None = None
    try:
        results["tree"] = _read_tree_inventory(source, policy)
        unique_entries: dict[str, dict[str, Any]] = {}
        unique_total = 0
        for entry in policy["inventory_expectations"]["bin_linux_entries"]:
            object_id = str(entry["object_id"])
            if object_id in unique_entries:
                continue
            size = int(entry["size_bytes"])
            unique_total += size
            if unique_total > int(policy["resources"]["max_unique_candidate_blob_bytes"]):
                raise StaticInventoryError("unique candidate blob total exceeds the frozen streaming limit")
            evidence = _stream_candidate_blob(source, object_id, size, policy)
            _verify_candidate_stream(evidence, size)
            unique_entries[object_id] = evidence
        annotated_entries: list[dict[str, Any]] = []
        for entry in policy["inventory_expectations"]["bin_linux_entries"]:
            evidence = unique_entries[str(entry["object_id"])]
            observed_elf = evidence["classification"] == "ELF"
            if observed_elf is not entry["expected_elf"]:
                raise StaticInventoryError(f"candidate ELF magic differs from frozen expectation: {entry['path']}")
            annotated_entries.append(
                {
                    **entry,
                    "observed_classification": evidence["classification"],
                    "object_sha256": evidence["sha256"],
                    "prefix_hex": evidence["prefix_hex"],
                }
            )
        results["candidate_unique_object_count"] = len(unique_entries)
        results["candidate_unique_total_bytes"] = unique_total
        results["candidate_objects"] = unique_entries
        results["bin_linux_entries"] = annotated_entries
    except (OSError, ValueError, StaticInventoryError, fetch_gate.SourceFetchGateError) as exc:
        failure = str(exc)

    status = "PASS_OFFLINE_FULL_SOURCE_STATIC_INVENTORY" if failure is None else "NO_GO_OFFLINE_FULL_SOURCE_STATIC_INVENTORY"
    receipt = {
        "schema_version": "smpcc-r8-liquid-target-source-static-inventory-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_STATIC_INVENTORY_RECEIPT",
        "created_at_utc": utc_now(),
        "status": status,
        "preflight": preflight,
        "partial_start_record": str(partial_path),
        "source_path": str(source),
        "results": results,
        "failure": failure,
        "network_used": False,
        "source_checkout_created": False,
        "upstream_code_execution_authorized": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "cmake_or_make_executed": False,
        "ros_or_gazebo_started": False,
        "gpu_device_exposed": False,
        "system_packages_changed": False,
        "sudo_used": False,
        "next_allowed_stage": (
            "OFFLINE_ELF_METADATA_AND_DYNAMIC_DEPENDENCY_POLICY_ONLY"
            if failure is None
            else "REVIEW_STATIC_INVENTORY_FAILURE"
        ),
    }
    receipt["receipt_sha256"] = canonical_hash(receipt)
    _write_json_new(final_path, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="read-only offline-inventory admission check")
    write = subparsers.add_parser(
        "write-static-inventory-receipt",
        help="read the frozen source offline and write one create-new static-inventory receipt",
    )
    write.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        policy = _read_json_object(POLICY_PATH)
        if args.command == "self-check":
            report = build_preflight(policy)
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
            return 0 if report["status"] == "PASS_OFFLINE_STATIC_INVENTORY_ADMITTED" else 2
        receipt = write_static_inventory_receipt(policy, args.receipt)
        print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if receipt["status"] == "PASS_OFFLINE_FULL_SOURCE_STATIC_INVENTORY" else 2
    except (
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        profile_gate.TargetProfileError,
        fetch_gate.SourceFetchGateError,
        StaticInventoryError,
    ) as exc:
        print(f"R8_LIQUID_TARGET_SOURCE_STATIC_INVENTORY_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
