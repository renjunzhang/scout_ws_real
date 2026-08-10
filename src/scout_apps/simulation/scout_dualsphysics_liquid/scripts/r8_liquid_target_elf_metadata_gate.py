#!/usr/bin/env python3
"""Fail-closed, pure-Python ELF metadata inspection for frozen Git blobs.

The gate does not run ``readelf``, ``file``, an ELF loader, or any upstream
program.  It uses offline Git plumbing to retain one bounded, known ELF blob
in memory at a time, verifies its SHA-256, and parses only the ELF64
little-endian header/program/dynamic tables with explicit range checks.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pwd
import re
import struct
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
import r8_liquid_target_source_fetch_gate as fetch_gate  # noqa: E402
import r8_liquid_target_source_static_inventory_gate as inventory_gate  # noqa: E402


POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_elf_metadata_policy_v1.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_elf_metadata_policy_v1.json"
RECEIPT_RE = re.compile(r"^u2_elf_metadata_v1_[0-9]{8}T[0-9]{6}Z\.json$")
SHA1_RE = re.compile(r"^[0-9a-f]{40}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
EXPECTED_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
EXPECTED_TREE = "cef458cb358712f4694b9d2148f638440418e9dc"
EXPECTED_URL = "https://github.com/DualSPHysics/DualSPHysics.git"
EXPECTED_STATIC_RECEIPT = "audits/u2_full_source_static_inventory_v1_20260806T080500Z.json"
EXPECTED_STATIC_FILE_SHA256 = "a3be7e2a7a5bbeae8483706dbd4a135c7af80513bd5fbd1f982122b4380efb28"
EXPECTED_STATIC_RECEIPT_SHA256 = "5eac669287c6711de0835409ed9f3ec9dc6829a3b03cef24cb6838ebb07e32ff"
EXPECTED_FULL_BARE_RELATIVE = (
    "dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git"
)
EXPECTED_ELF_OBJECTS = [
    {
        "object_id": "4db4dadbae8f6e3885cad3bb752340812036278a",
        "sha256": "a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226",
        "size_bytes": 5809384,
        "paths": ["bin/linux/GenCase_linux64"],
    },
    {
        "object_id": "99308e40373cb62cfd0189c8e06e611636b18fd8",
        "sha256": "ef711ee081613ddb97ac4f86f24652dc92eab3fc37c6e78c3fb9ef4ec02ad400",
        "size_bytes": 6210152,
        "paths": [
            "bin/linux/IsoSurface_linux64",
            "bin/linux/MeasureTool_linux64",
            "bin/linux/PartVTKOut_linux64",
            "bin/linux/PartVTK_linux64",
        ],
    },
    {
        "object_id": "f0892cc22c7ee34bb25a815333c6a14521d77531",
        "sha256": "3adb8a5ef36b988add7717d60ee5e50bf7107a5623b448c3a5300d087c2b32e7",
        "size_bytes": 28283072,
        "paths": ["bin/linux/libChronoEngine.so"],
    },
    {
        "object_id": "87330bada49b0e97f636c0c8919d39af5f5f4f82",
        "sha256": "6a7a94ed7adcdd4e9dddee58cde0f080dee95d930e169bd39cc78894d1a2c937",
        "size_bytes": 646160,
        "paths": ["bin/linux/libdsphchrono.so"],
    },
]

PT_LOAD = 1
PT_DYNAMIC = 2
PT_INTERP = 3
PT_GNU_STACK = 0x6474E551
DT_NULL = 0
DT_NEEDED = 1
DT_STRTAB = 5
DT_STRSZ = 10
DT_SONAME = 14
DT_RPATH = 15
DT_RUNPATH = 29
PF_X = 1


class ElfMetadataError(RuntimeError):
    """Ambiguous binary metadata is a NO-GO, never a reason to execute it."""


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_json_object(path: Path) -> dict[str, Any]:
    value = fetch_gate._read_json_object(path)
    if not isinstance(value, dict):
        raise ElfMetadataError(f"JSON file is not an object: {path}")
    return value


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate only frozen declarations; do not read a blob or the host."""

    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-elf-metadata-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_ELF_METADATA_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_ELF_METADATA_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "offline_elf_metadata_read_authorized": True,
        "upstream_code_execution_authorized": False,
        "allowed_gate_commands": ["self-check", "write-elf-metadata-receipt"],
    }
    for key, value in expected_top.items():
        if policy.get(key) != value:
            errors.append(f"policy field mismatch: {key}")
    if policy.get("required_target_profile") != {
        "profile_id": "LIQUID_ZRJ_MSI_U2404_P0_V1",
        "canonical_sha256": "fec0b980ca46df04d877290b73b3f3406af97887ccc20c9d52d58092c36f8838",
        "required_status": "PASS_PROFILE_READ_ONLY_ONLY",
    }:
        errors.append("required target profile differs from the frozen MSI profile")
    if policy.get("required_static_inventory_receipt") != {
        "receipt_relative_path": EXPECTED_STATIC_RECEIPT,
        "file_sha256": EXPECTED_STATIC_FILE_SHA256,
        "receipt_sha256": EXPECTED_STATIC_RECEIPT_SHA256,
        "status": "PASS_OFFLINE_FULL_SOURCE_STATIC_INVENTORY",
        "next_allowed_stage": "OFFLINE_ELF_METADATA_AND_DYNAMIC_DEPENDENCY_POLICY_ONLY",
    }:
        errors.append("required static inventory receipt differs from frozen evidence")
    if policy.get("source") != {
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
    }:
        errors.append("source declaration differs from the frozen bare/offline source")
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
    if policy.get("trusted_system_tools") != expected_tools:
        errors.append("trusted Git suite differs from frozen system declarations")
    if policy.get("git_commands") != {
        "tree_identity": ["rev-parse", f"{EXPECTED_COMMIT}^{{tree}}"],
        "bare_identity": ["rev-parse", "--is-bare-repository"],
        "remote_identity": ["remote", "get-url", "origin"],
        "blob": ["cat-file", "blob"],
    }:
        errors.append("offline Git argv policy differs from frozen ELF metadata commands")
    if policy.get("elf_objects") != EXPECTED_ELF_OBJECTS:
        errors.append("ELF object set differs from the frozen static-inventory hashes")
    if policy.get("elf_expectations") != {
        "class": "ELF64",
        "endianness": "little",
        "machine": 62,
        "maximum_program_headers": 256,
        "maximum_dynamic_entries": 2048,
        "maximum_string_table_bytes": 8388608,
        "maximum_dynamic_string_bytes": 1024,
        "maximum_needed_entries": 128,
    }:
        errors.append("ELF parser limits or architecture expectation differ from frozen policy")
    if policy.get("resources") != {
        "short_command_timeout_seconds": 60,
        "blob_read_timeout_seconds": 60,
        "kill_grace_seconds": 5,
        "short_command_output_max_bytes": 16384,
        "max_blob_bytes": 67108864,
        "max_total_elf_blob_bytes": 67108864,
        "blob_stderr_max_bytes": 16384,
        "receipt_mode_octal": "0640",
    }:
        errors.append("ELF metadata resource policy differs from frozen limits")
    if policy.get("receipt") != {
        "directory_relative_path": "audits",
        "filename_regex": RECEIPT_RE.pattern,
    }:
        errors.append("ELF metadata receipt policy is invalid")
    expected_invariants = {
        "no_sudo",
        "no_apt",
        "no_network",
        "no_checkout",
        "no_source_file_execution",
        "no_precompiled_binary_execution",
        "no_external_elf_parser",
        "no_cmake_or_make",
        "no_ros_or_gazebo",
        "no_gpu_device",
        "no_destructive_cleanup",
        "no_retry_in_existing_receipt_path",
    }
    invariants = policy.get("invariants")
    if not isinstance(invariants, dict) or set(invariants) != expected_invariants or any(
        invariants.get(key) is not True for key in expected_invariants
    ):
        errors.append("ELF metadata safety invariants are incomplete")
    return errors


def _approved_root(profile: Mapping[str, Any]) -> Path:
    root = Path(str(profile.get("storage", {}).get("approved_root", "")))
    if root != Path("/home/zrj/scout_liquid_lab"):
        raise ElfMetadataError("target profile approved root differs from frozen MSI root")
    return root


def _verify_static_inventory_receipt(root: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    receipt = fetch_gate._relative_under_root(root, str(expected.get("receipt_relative_path", "")))
    if not receipt.is_file() or receipt.is_symlink():
        raise ElfMetadataError("required static inventory receipt is missing or unsafe")
    observed_file_hash = fetch_gate._sha256_regular_file(receipt)
    if observed_file_hash != expected.get("file_sha256"):
        raise ElfMetadataError("required static inventory receipt file hash mismatch")
    value = _read_json_object(receipt)
    results = value.get("results")
    if not isinstance(results, dict):
        raise ElfMetadataError("static inventory receipt lacks result evidence")
    candidates = results.get("candidate_objects")
    if not isinstance(candidates, dict):
        raise ElfMetadataError("static inventory receipt lacks candidate object evidence")
    for entry in EXPECTED_ELF_OBJECTS:
        observed = candidates.get(entry["object_id"])
        if not isinstance(observed, dict) or (
            observed.get("sha256") != entry["sha256"]
            or observed.get("bytes_read") != entry["size_bytes"]
            or observed.get("classification") != "ELF"
        ):
            raise ElfMetadataError("static inventory receipt does not prove the frozen ELF object set")
    if (
        value.get("status") != expected.get("status")
        or value.get("receipt_sha256") != expected.get("receipt_sha256")
        or value.get("next_allowed_stage") != expected.get("next_allowed_stage")
        or value.get("network_used") is not False
        or value.get("source_checkout_created") is not False
        or value.get("upstream_code_executed") is not False
        or value.get("precompiled_binary_executed") is not False
    ):
        raise ElfMetadataError("static inventory receipt content does not satisfy frozen offline evidence")
    return {
        "path": str(receipt),
        "file_sha256": observed_file_hash,
        "receipt_sha256": value["receipt_sha256"],
        "status": value["status"],
        "next_allowed_stage": value["next_allowed_stage"],
    }


def _git_text(source: Path, args: Sequence[str], policy: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
    resources = policy["resources"]
    result = fetch_gate._run_owned_bounded(
        fetch_gate._git_argv("--git-dir", str(source), *args),
        timeout_seconds=int(resources["short_command_timeout_seconds"]),
        grace_seconds=int(resources["kill_grace_seconds"]),
        output_max_bytes=int(resources["short_command_output_max_bytes"]),
        environment=inventory_gate._offline_git_environment(),
    )
    if (
        result["returncode"] != 0
        or result["executor_error"] is not None
        or result["hard_timeout_triggered"]
        or result["hard_kill_for_output_limit"]
    ):
        raise ElfMetadataError("offline Git source identity query failed")
    compact = {
        key: result.get(key)
        for key in (
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
    }
    return str(result["stdout_utf8"]).strip(), compact


def build_preflight(policy: Mapping[str, Any]) -> dict[str, Any]:
    """Read-only admission; this does not retain or parse an ELF blob."""

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
            errors.append("target profile canonical hash differs from ELF metadata policy")
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
                errors.append("ELF metadata gate must run as target ordinary user, never through sudo")
            if root_metadata.st_uid != owner.pw_uid or root_metadata.st_gid != owner.pw_gid:
                errors.append("approved root ownership differs from target profile")
            observations["required_static_inventory_receipt"] = _verify_static_inventory_receipt(
                root, policy["required_static_inventory_receipt"]
            )
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
                    errors.append("full bare source identity differs from frozen ELF metadata policy")
        git_evidence = fetch_gate._trusted_git_suite(policy["trusted_system_tools"])
        observations["trusted_git_suite"] = git_evidence
        if not git_evidence.get("trusted"):
            errors.append("system Git suite does not match frozen root-owned hashes")
    except (
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        profile_gate.TargetProfileError,
        fetch_gate.SourceFetchGateError,
        ElfMetadataError,
    ) as exc:
        errors.append(f"ELF metadata host inspection failed: {exc}")
    core = {
        "schema_version": "smpcc-r8-liquid-target-elf-metadata-preflight-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_ELF_METADATA_PREFLIGHT",
        "captured_at_utc": utc_now(),
        "policy_path": str(POLICY_PATH),
        "policy_sha256": fetch_gate._sha256_regular_file(POLICY_PATH),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": fetch_gate._sha256_regular_file(SCHEMA_PATH),
        "policy_id": policy.get("policy_id"),
        "host_id": policy.get("host_id"),
        "status": "PASS_OFFLINE_ELF_METADATA_ADMITTED" if not errors else "NO_GO_OFFLINE_ELF_METADATA",
        "errors": errors,
        "observations": observations,
        "network_used": False,
        "source_checkout_created": False,
        "upstream_code_execution_authorized": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "external_elf_parser_executed": False,
        "next_allowed_stage": "WRITE_ONE_ELF_METADATA_RECEIPT" if not errors else "REVIEW_NO_GO",
    }
    return dict(core, preflight_sha256=canonical_hash(core))


def validate_receipt_path(path: Path, root: Path) -> tuple[Path, Path]:
    if not fetch_gate._is_clean_absolute(path) or not fetch_gate._is_clean_absolute(root):
        raise ElfMetadataError("receipt and approved root must be clean absolute paths")
    audits = root / "audits"
    if fetch_gate._has_symlink_component(audits) or audits.is_symlink() or not audits.is_dir():
        raise ElfMetadataError("audit directory is missing or unsafe")
    if path.parent != audits or RECEIPT_RE.fullmatch(path.name) is None:
        raise ElfMetadataError("receipt path is not the fixed ELF metadata audit filename")
    partial = Path(f"{path}.partial")
    if path.exists() or path.is_symlink() or partial.exists() or partial.is_symlink():
        raise ElfMetadataError("receipt final or partial path already exists")
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


def _read_blob_bytes(
    source: Path, object_id: str, expected_size: int, policy: Mapping[str, Any]
) -> tuple[dict[str, Any], bytes]:
    """Retain one exact-size Git blob in memory with bounded output and time."""

    resources = policy["resources"]
    maximum_blob = int(resources["max_blob_bytes"])
    stderr_limit = int(resources["blob_stderr_max_bytes"])
    timeout_seconds = int(resources["blob_read_timeout_seconds"])
    grace_seconds = int(resources["kill_grace_seconds"])
    if not SHA1_RE.fullmatch(object_id) or expected_size < 64 or expected_size > maximum_blob:
        raise ElfMetadataError("ELF blob ID or size is outside frozen limits")
    argv = fetch_gate._git_argv("--git-dir", str(source), *policy["git_commands"]["blob"], object_id)
    payload = bytearray()
    digest = hashlib.sha256()
    stderr = bytearray()
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
        try:
            while True:
                block = stream.read(65536)
                if not block:
                    return
                if len(payload) + len(block) > expected_size or len(payload) + len(block) > maximum_blob:
                    size_limit_hit.set()
                    return
                payload.extend(block)
                digest.update(block)
        except (OSError, ValueError) as exc:
            remember_reader_error("ELF blob stdout reader", exc)

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
            remember_reader_error("ELF blob stderr reader", exc)

    started = time.monotonic()
    try:
        process = subprocess.Popen(
            tuple(argv),
            cwd="/",
            env=inventory_gate._offline_git_environment(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            close_fds=True,
        )
        if process.stdout is None or process.stderr is None:
            raise ElfMetadataError("ELF blob output pipes were not created")
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
    except (OSError, subprocess.SubprocessError, ElfMetadataError) as exc:
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
                reader_errors.append("ELF blob output reader did not terminate")
    with stderr_lock:
        stderr_bytes = bytes(stderr)
    evidence = {
        "argv": list(argv),
        "returncode": process.returncode if process is not None else None,
        "hard_timeout_triggered": timed_out,
        "hard_kill_for_size_limit": size_limit_hit.is_set(),
        "hard_kill_for_stderr_limit": stderr_limit_hit.is_set(),
        "expected_bytes": expected_size,
        "bytes_read": len(payload),
        "sha256": digest.hexdigest(),
        "stderr_captured_bytes": len(stderr_bytes),
        "stderr_max_bytes": stderr_limit,
        "stderr_sha256": hashlib.sha256(stderr_bytes).hexdigest(),
        "stderr_utf8": stderr_bytes.decode("utf-8", errors="replace"),
        "reader_errors": reader_errors,
        "executor_error": executor_error,
        "elapsed_seconds": round(time.monotonic() - started, 6),
    }
    if (
        evidence["returncode"] != 0
        or evidence["executor_error"] is not None
        or evidence["hard_timeout_triggered"]
        or evidence["hard_kill_for_size_limit"]
        or evidence["hard_kill_for_stderr_limit"]
        or evidence["reader_errors"]
        or evidence["bytes_read"] != expected_size
    ):
        raise ElfMetadataError(f"ELF blob stream did not complete safely: {evidence}")
    return evidence, bytes(payload)


def _safe_range(offset: int, size: int, total: int, label: str) -> tuple[int, int]:
    if offset < 0 or size < 0 or offset > total or size > total - offset:
        raise ElfMetadataError(f"ELF {label} lies outside the bounded blob")
    return offset, offset + size


def _decode_dynamic_string(data: bytes, start: int, end: int, maximum: int, label: str) -> str:
    if start < 0 or start >= end or end > len(data):
        raise ElfMetadataError(f"ELF {label} string offset is invalid")
    terminator = data.find(b"\0", start, end)
    if terminator < 0 or terminator - start > maximum:
        raise ElfMetadataError(f"ELF {label} string lacks a bounded terminator")
    raw = data[start:terminator]
    try:
        text = raw.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ElfMetadataError(f"ELF {label} string is not ASCII") from exc
    if not text or any(ord(character) < 32 or ord(character) == 127 for character in text):
        raise ElfMetadataError(f"ELF {label} string is empty or contains control data")
    return text


def parse_elf64_little_endian(data: bytes, expectations: Mapping[str, Any]) -> dict[str, Any]:
    """Parse bounded ELF metadata without invoking a native ELF parser."""

    if len(data) < 64 or data[:4] != b"\x7fELF":
        raise ElfMetadataError("blob does not begin with an ELF header")
    ident = data[:16]
    if ident[4] != 2 or ident[5] != 1 or ident[6] != 1:
        raise ElfMetadataError("only ELF64 little-endian version-1 blobs are admitted")
    fields = struct.unpack_from("<16sHHIQQQIHHHHHH", data, 0)
    e_type, e_machine, e_version = fields[1], fields[2], fields[3]
    e_phoff, e_ehsize, e_phentsize, e_phnum = fields[5], fields[8], fields[9], fields[10]
    maximum_phnum = int(expectations["maximum_program_headers"])
    if (
        e_version != 1
        or e_ehsize != 64
        or e_phentsize != 56
        or e_phnum < 1
        or e_phnum > maximum_phnum
        or e_machine != int(expectations["machine"])
    ):
        raise ElfMetadataError("ELF header differs from frozen architecture/size limits")
    phoff, phend = _safe_range(e_phoff, e_phnum * e_phentsize, len(data), "program header table")
    del phoff, phend
    program_headers: list[tuple[int, int, int, int, int, int, int, int]] = []
    load_segments: list[tuple[int, int, int, int]] = []
    dynamic_headers: list[tuple[int, int]] = []
    interpreter: str | None = None
    gnu_stack_executable: bool | None = None
    for index in range(e_phnum):
        offset = e_phoff + index * e_phentsize
        header = struct.unpack_from("<IIQQQQQQ", data, offset)
        p_type, p_flags, p_offset, p_vaddr, _, p_filesz, p_memsz, _ = header
        _safe_range(p_offset, p_filesz, len(data), "program segment")
        program_headers.append(header)
        if p_type == PT_LOAD:
            load_segments.append((p_vaddr, p_filesz, p_offset, p_memsz))
        elif p_type == PT_DYNAMIC:
            dynamic_headers.append((p_offset, p_filesz))
        elif p_type == PT_INTERP:
            if interpreter is not None:
                raise ElfMetadataError("ELF contains multiple PT_INTERP segments")
            interpreter = _decode_dynamic_string(
                data,
                p_offset,
                p_offset + p_filesz,
                int(expectations["maximum_dynamic_string_bytes"]),
                "interpreter",
            )
        elif p_type == PT_GNU_STACK:
            if gnu_stack_executable is not None:
                raise ElfMetadataError("ELF contains multiple PT_GNU_STACK segments")
            gnu_stack_executable = bool(p_flags & PF_X)
    if len(dynamic_headers) != 1:
        raise ElfMetadataError("ELF must contain exactly one PT_DYNAMIC segment for this audit")

    dynamic_offset, dynamic_size = dynamic_headers[0]
    if dynamic_size % 16 != 0 or dynamic_size // 16 > int(expectations["maximum_dynamic_entries"]):
        raise ElfMetadataError("ELF dynamic table exceeds frozen entry limits")
    dynamic_values: dict[int, list[int]] = {}
    dynamic_entry_count = 0
    terminated = False
    for index in range(dynamic_size // 16):
        tag, value = struct.unpack_from("<qQ", data, dynamic_offset + index * 16)
        dynamic_entry_count += 1
        if tag == DT_NULL:
            terminated = True
            break
        dynamic_values.setdefault(tag, []).append(value)
    if not terminated:
        raise ElfMetadataError("ELF dynamic table lacks DT_NULL within bounded segment")
    strtab_values = dynamic_values.get(DT_STRTAB, [])
    strsz_values = dynamic_values.get(DT_STRSZ, [])
    if len(strtab_values) != 1 or len(strsz_values) != 1:
        raise ElfMetadataError("ELF dynamic string table declarations are ambiguous")
    string_table_virtual, string_table_size = strtab_values[0], strsz_values[0]
    if string_table_size < 1 or string_table_size > int(expectations["maximum_string_table_bytes"]):
        raise ElfMetadataError("ELF dynamic string table exceeds frozen size limit")
    string_table_offset: int | None = None
    for virtual, filesz, file_offset, _ in load_segments:
        relative = string_table_virtual - virtual
        if string_table_virtual >= virtual and relative <= filesz and string_table_size <= filesz - relative:
            candidate = file_offset + relative
            _safe_range(candidate, string_table_size, len(data), "dynamic string table")
            if string_table_offset is not None:
                raise ElfMetadataError("ELF dynamic string table maps through multiple PT_LOAD segments")
            string_table_offset = candidate
    if string_table_offset is None:
        raise ElfMetadataError("ELF dynamic string table is not covered by a PT_LOAD file segment")
    string_table_end = string_table_offset + string_table_size

    def dynamic_string(value: int, label: str) -> str:
        if value >= string_table_size:
            raise ElfMetadataError(f"ELF {label} offset lies beyond dynamic string table")
        return _decode_dynamic_string(
            data,
            string_table_offset + value,
            string_table_end,
            int(expectations["maximum_dynamic_string_bytes"]),
            label,
        )

    needed_values = dynamic_values.get(DT_NEEDED, [])
    if len(needed_values) > int(expectations["maximum_needed_entries"]):
        raise ElfMetadataError("ELF DT_NEEDED count exceeds frozen limit")
    needed = [dynamic_string(value, "DT_NEEDED") for value in needed_values]
    if len(set(needed)) != len(needed):
        raise ElfMetadataError("ELF DT_NEEDED list contains a duplicate dependency")
    soname_values = dynamic_values.get(DT_SONAME, [])
    rpath_values = dynamic_values.get(DT_RPATH, [])
    runpath_values = dynamic_values.get(DT_RUNPATH, [])
    if len(soname_values) > 1 or len(rpath_values) > 1 or len(runpath_values) > 1:
        raise ElfMetadataError("ELF SONAME/RPATH/RUNPATH declarations are ambiguous")
    return {
        "class": "ELF64",
        "endianness": "little",
        "type": e_type,
        "machine": e_machine,
        "osabi": ident[7],
        "abi_version": ident[8],
        "program_header_count": e_phnum,
        "dynamic_entry_count": dynamic_entry_count,
        "interpreter": interpreter,
        "soname": dynamic_string(soname_values[0], "DT_SONAME") if soname_values else None,
        "needed": needed,
        "rpath": dynamic_string(rpath_values[0], "DT_RPATH") if rpath_values else None,
        "runpath": dynamic_string(runpath_values[0], "DT_RUNPATH") if runpath_values else None,
        "gnu_stack_executable": gnu_stack_executable,
    }


def write_elf_metadata_receipt(policy: Mapping[str, Any], receipt_path: Path) -> dict[str, Any]:
    """Write one append-only, no-network ELF metadata receipt."""

    preflight = build_preflight(policy)
    profile = profile_gate._read_json(profile_gate.PROFILE_PATH)
    root = _approved_root(profile)
    final_path, partial_path = validate_receipt_path(receipt_path, root)
    if preflight["status"] != "PASS_OFFLINE_ELF_METADATA_ADMITTED":
        receipt = {
            "schema_version": "smpcc-r8-liquid-target-elf-metadata-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_TARGET_ELF_METADATA_RECEIPT",
            "created_at_utc": utc_now(),
            "status": "NO_GO_STATIC_PRECHECK",
            "preflight": preflight,
            "network_used": False,
            "source_checkout_created": False,
            "upstream_code_executed": False,
            "precompiled_binary_executed": False,
            "external_elf_parser_executed": False,
        }
        receipt["receipt_sha256"] = canonical_hash(receipt)
        _write_json_new(final_path, receipt)
        return receipt

    source = fetch_gate._relative_under_root(root, policy["source"]["full_bare_relative_path"])
    started_record = {
        "schema_version": "smpcc-r8-liquid-target-elf-metadata-partial-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_ELF_METADATA_PARTIAL",
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
        total_bytes = 0
        objects: list[dict[str, Any]] = []
        for entry in policy["elf_objects"]:
            size = int(entry["size_bytes"])
            total_bytes += size
            if total_bytes > int(policy["resources"]["max_total_elf_blob_bytes"]):
                raise ElfMetadataError("total ELF blob bytes exceed frozen in-memory limit")
            stream, payload = _read_blob_bytes(source, str(entry["object_id"]), size, policy)
            if stream["sha256"] != entry["sha256"]:
                raise ElfMetadataError(f"ELF blob SHA-256 differs from static inventory: {entry['object_id']}")
            metadata = parse_elf64_little_endian(payload, policy["elf_expectations"])
            if (
                metadata["class"] != policy["elf_expectations"]["class"]
                or metadata["endianness"] != policy["elf_expectations"]["endianness"]
                or metadata["machine"] != policy["elf_expectations"]["machine"]
            ):
                raise ElfMetadataError("ELF metadata differs from frozen architecture policy")
            objects.append({**entry, "stream": stream, "metadata": metadata})
        results = {"total_elf_blob_bytes": total_bytes, "objects": objects}
    except (OSError, ValueError, struct.error, ElfMetadataError, fetch_gate.SourceFetchGateError) as exc:
        failure = str(exc)
    status = "PASS_OFFLINE_ELF_METADATA" if failure is None else "NO_GO_OFFLINE_ELF_METADATA"
    receipt = {
        "schema_version": "smpcc-r8-liquid-target-elf-metadata-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_ELF_METADATA_RECEIPT",
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
        "external_elf_parser_executed": False,
        "cmake_or_make_executed": False,
        "ros_or_gazebo_started": False,
        "gpu_device_exposed": False,
        "system_packages_changed": False,
        "sudo_used": False,
        "next_allowed_stage": "HUMAN_REVIEW_ELF_METADATA_NO_EXECUTION" if failure is None else "REVIEW_ELF_METADATA_FAILURE",
    }
    receipt["receipt_sha256"] = canonical_hash(receipt)
    _write_json_new(final_path, receipt)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="read-only ELF metadata admission check")
    write = subparsers.add_parser(
        "write-elf-metadata-receipt",
        help="read frozen ELF blobs in memory and write one static metadata receipt",
    )
    write.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        policy = _read_json_object(POLICY_PATH)
        if args.command == "self-check":
            report = build_preflight(policy)
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
            return 0 if report["status"] == "PASS_OFFLINE_ELF_METADATA_ADMITTED" else 2
        receipt = write_elf_metadata_receipt(policy, args.receipt)
        print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if receipt["status"] == "PASS_OFFLINE_ELF_METADATA" else 2
    except (
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        struct.error,
        profile_gate.TargetProfileError,
        fetch_gate.SourceFetchGateError,
        ElfMetadataError,
    ) as exc:
        print(f"R8_LIQUID_TARGET_ELF_METADATA_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
