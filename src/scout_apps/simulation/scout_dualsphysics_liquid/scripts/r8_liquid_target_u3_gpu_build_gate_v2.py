#!/usr/bin/env python3
"""Fail-closed one-shot execution gate for RTX 5080 Attempt A.

This revision is intentionally separate from the frozen G1 contract.  It
imports the byte-pinned G1 module only for pure validation and argv building;
it never loads or removes AppArmor profiles and never invokes sudo itself.
The authorized outer supervisor must load exactly one reviewed profile, drop
to uid/gid 1000 with no supplementary groups, invoke one fixed phase, remove
that profile, and prove zero residue.  No command in this file can execute the
candidate or expose a GPU device.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import selectors
import shutil
import signal
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from types import ModuleType
from typing import Any, Iterable, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True

SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parents[1]
WORKSPACE_ROOT = PACKAGE_DIR.parents[3]
POLICY_PATH = (
    PACKAGE_DIR
    / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v2.json"
)
POLICY_SHA256 = "17472c30021bd70d81180fca6be1317a84cf5ae6f104e0c7cc707dc88ed4b58d"
RECEIPT_SCHEMA_PATH = (
    PACKAGE_DIR / "schema/target_host_u3_gpu_build_execution_receipt_v1.json"
)
RECEIPT_SCHEMA_SHA256 = "10ede832aae19632b9b5543b9916a7d1803e210c743db90910a66a37a1e02173"
G1_GATE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_gpu_build_gate_v1.py")
G1_GATE_SHA256 = "e34ab0facc3c665c6befbb792c7344e0af6e652fd3c6b30dfa81ca8cbea5b502"

CAMPAIGN_ID = "u3_source_gpu_build_sm120_20260810T102641Z"
BUILD_ID_A = CAMPAIGN_ID + "_a"
BUILD_ID_B = CAMPAIGN_ID + "_b"
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
ROOT_A = LIQUID_ROOT / "build" / f"{BUILD_ID_A}.partial"
ROOT_B = LIQUID_ROOT / "build" / f"{BUILD_ID_B}.partial"
OUTPUT_ROOT = ROOT_A / "output"
OUTPUT_SOURCE = OUTPUT_ROOT / "buildtree/src/source"
ARTIFACTS_ROOT = OUTPUT_ROOT / "artifacts"
CANDIDATE_PATH = ARTIFACTS_ROOT / "DualSPHysics5.4_linux64"

SOURCE_COPY_START = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_source_copy.json.partial"
SOURCE_COPY_RUN = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_source_copy_run.json"
SOURCE_COPY_FINAL = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_source_copy.json"
BUILD_START = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_gpu_build.json.partial"
BUILD_FINAL = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_gpu_build.json"
BUILD_STDOUT = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_gpu_build.stdout.log"
BUILD_STDERR = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_gpu_build.stderr.log"
BUILD_MONITOR = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_gpu_build.resource.jsonl"
STATIC_AUDIT_FINAL = LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_gpu_static_audit.json"
STATIC_PARSER_OUTPUT = (
    LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_gpu_static_audit.parser-output.bin"
)
STATIC_COMMAND_SUMMARY = (
    LIQUID_ROOT / "audits" / f"{BUILD_ID_A}_gpu_static_audit.commands.jsonl"
)

T0_TEXT = "2026-08-10T17:17:51.731823359+08:00"
CANDIDATE_DEADLINE_TEXT = "2026-08-10T23:17:51.731823359+08:00"
FINAL_DEADLINE_TEXT = "2026-08-11T01:17:51.731823359+08:00"
CANDIDATE_DEADLINE_EPOCH = datetime.fromisoformat(CANDIDATE_DEADLINE_TEXT).timestamp()
FINAL_DEADLINE_EPOCH = datetime.fromisoformat(FINAL_DEADLINE_TEXT).timestamp()

MINIMUM_AVAILABLE_MEMORY_BYTES = 4_294_967_296
MINIMUM_AVAILABLE_DISK_BYTES = 21_474_836_480
MONITOR_INTERVAL_SECONDS = 20
STATIC_AGGREGATE_OUTPUT_LIMIT = 268_435_456
STATIC_PER_COMMAND_OUTPUT_LIMIT = 268_435_456
STREAM_PREFIX_LIMIT = 65_536
WRAPPER_NAME = "U3GpuBuild.mk"

EXPECTED_POLICY_KEYS = {
    "schema_version",
    "document_type",
    "policy_id",
    "host_id",
    "goal",
    "execution_scope",
    "plan_identity",
    "g1_parent_hashes",
    "receipt_schema",
    "campaign",
    "deadlines",
    "runtime_identity",
    "system_tools",
    "command_surface",
    "receipt_paths",
    "phase_machine",
    "source_copy",
    "build",
    "static_audit",
    "profiles",
    "invariants",
    "status",
    "next_allowed_stage",
}

COMMANDS = (
    "self-check",
    "preflight",
    "prepare-source-copy",
    "run-source-copy",
    "finalize-source-copy",
    "prepare-build",
    "run-build",
    "run-static-audit",
)


class GateError(RuntimeError):
    """A fail-closed static or runtime gate condition."""


class AuditError(GateError):
    """A reproducible G4 static-audit failure."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def canonical_json(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(canonical_json(value)).hexdigest()


def sha256_fd(descriptor: int) -> str:
    position = os.lseek(descriptor, 0, os.SEEK_CUR)
    os.lseek(descriptor, 0, os.SEEK_SET)
    digest = hashlib.sha256()
    while True:
        block = os.read(descriptor, 1 << 20)
        if not block:
            break
        digest.update(block)
    os.lseek(descriptor, position, os.SEEK_SET)
    return digest.hexdigest()


def sha256_file(path: Path, *, limit: int | None = None) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise GateError(f"unsafe regular-file identity: {path}")
        if limit is not None and info.st_size > limit:
            raise GateError(f"bounded file is too large: {path}")
        return sha256_fd(descriptor)
    finally:
        os.close(descriptor)


def read_json_object(path: Path, *, limit: int = 16 * 1024 * 1024) -> dict[str, Any]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise GateError(f"unsafe JSON identity: {path}")
        if info.st_size > limit:
            raise GateError(f"JSON exceeds bounded read: {path}")
        chunks: list[bytes] = []
        remaining = info.st_size
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                raise GateError(f"short bounded JSON read: {path}")
            chunks.append(block)
            remaining -= len(block)
        raw = b"".join(chunks)
    finally:
        os.close(descriptor)
    value = json.loads(raw.decode("utf-8"))
    if not isinstance(value, dict):
        raise GateError(f"JSON root is not an object: {path}")
    return value


def load_g1() -> ModuleType:
    if sha256_file(G1_GATE_PATH, limit=4 * 1024 * 1024) != G1_GATE_SHA256:
        raise RuntimeError("byte-pinned G1 gate differs")
    spec = importlib.util.spec_from_file_location("r8_liquid_u3_gpu_g1_frozen", G1_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load byte-pinned G1 gate")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


g1 = load_g1()


def policy() -> dict[str, Any]:
    if sha256_file(POLICY_PATH, limit=1024 * 1024) != POLICY_SHA256:
        raise GateError("execution policy SHA-256 drift")
    return read_json_object(POLICY_PATH, limit=1024 * 1024)


def receipt_schema() -> dict[str, Any]:
    if sha256_file(RECEIPT_SCHEMA_PATH, limit=1024 * 1024) != RECEIPT_SCHEMA_SHA256:
        raise GateError("receipt schema SHA-256 drift")
    return read_json_object(RECEIPT_SCHEMA_PATH, limit=1024 * 1024)


def verify_recursive_object_closure(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object":
            properties = value.get("properties")
            required = value.get("required")
            if value.get("additionalProperties") is not False:
                raise GateError(f"receipt schema object is open at {location}")
            if not isinstance(properties, dict) or set(required or ()) != set(properties):
                raise GateError(f"receipt schema required/properties drift at {location}")
        for key, child in value.items():
            verify_recursive_object_closure(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            verify_recursive_object_closure(child, f"{location}/{index}")


def verify_policy_contract() -> dict[str, Any]:
    current = policy()
    if set(current) != EXPECTED_POLICY_KEYS:
        raise GateError("execution policy top-level key set drift")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-gpu-build-execution-policy-v2",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_GPU_BUILD_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_GPU_BUILD_EXECUTION_V2",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "goal": "G2A_TO_G4_ATTEMPT_A_ONE_SHOT",
        "status": "G2A_EXECUTION_LAYER_FROZEN_PENDING_STATIC_VALIDATION",
    }
    for key, expected in expected_top.items():
        if current[key] != expected:
            raise GateError(f"execution policy identity drift: {key}")
    if tuple(current["command_surface"]) != COMMANDS:
        raise GateError("execution command surface drift")
    plan = current["plan_identity"]
    if plan != {
        "commit": g1.PLAN_COMMIT,
        "blob": g1.PLAN_BLOB,
        "sha256": g1.PLAN_SHA256,
    }:
        raise GateError("plan identity in execution policy drift")
    campaign = current["campaign"]
    if campaign != {
        "campaign_id": CAMPAIGN_ID,
        "build_id_a": BUILD_ID_A,
        "root_a": str(ROOT_A),
        "build_id_b": BUILD_ID_B,
        "root_b": str(ROOT_B),
        "attempt_b_forbidden": True,
        "parallel_jobs": 1,
    }:
        raise GateError("execution campaign identity drift")
    deadlines = current["deadlines"]
    if (
        deadlines["t0"] != T0_TEXT
        or deadlines["candidate_deadline"] != CANDIDATE_DEADLINE_TEXT
        or deadlines["final_deadline"] != FINAL_DEADLINE_TEXT
        or deadlines["t0_reset_forbidden"] is not True
    ):
        raise GateError("execution deadline identity drift")
    schema_contract = current["receipt_schema"]
    if (
        schema_contract["path"]
        != "src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_gpu_build_execution_receipt_v1.json"
        or schema_contract["sha256"] != RECEIPT_SCHEMA_SHA256
        or schema_contract["recursive_object_closure_required"] is not True
    ):
        raise GateError("receipt schema contract drift")
    receipts = current["receipt_paths"]
    expected_receipts = {
        "source_copy_start": str(SOURCE_COPY_START),
        "source_copy_run": str(SOURCE_COPY_RUN),
        "source_copy_final": str(SOURCE_COPY_FINAL),
        "gpu_build_start": str(BUILD_START),
        "gpu_build_final": str(BUILD_FINAL),
        "gpu_build_stdout": str(BUILD_STDOUT),
        "gpu_build_stderr": str(BUILD_STDERR),
        "gpu_build_monitor": str(BUILD_MONITOR),
        "static_audit_final": str(STATIC_AUDIT_FINAL),
        "static_audit_parser_output": str(STATIC_PARSER_OUTPUT),
        "static_audit_command_summary": str(STATIC_COMMAND_SUMMARY),
    }
    if receipts != expected_receipts:
        raise GateError("execution receipt path set drift")
    build = current["build"]
    if (
        build["make_invocation_limit"] != 1
        or build["parallel_jobs"] != 1
        or build["wall_timeout_seconds"] != 5400
        or build["cpu_limit_seconds"] != 5400
        or build["address_space_limit_bytes"] != 8_589_934_592
        or build["minimum_available_memory_bytes"] != MINIMUM_AVAILABLE_MEMORY_BYTES
        or build["monitor_interval_seconds"] != MONITOR_INTERVAL_SECONDS
        or build["candidate_path"] != str(CANDIDATE_PATH)
        or build["candidate_disarm_mode_octal"] != "0400"
    ):
        raise GateError("execution build/resource contract drift")
    static_contract = current["static_audit"]
    if (
        static_contract["candidate_command_count"] != 11
        or static_contract["object_command_count"] != 546
        or static_contract["total_command_count"] != 557
        or static_contract["object_count"] != 131
        or static_contract["cpp_object_count"] != 120
        or static_contract["cuda_object_count"] != 11
        or static_contract["aggregate_parser_output_limit_bytes"]
        != STATIC_AGGREGATE_OUTPUT_LIMIT
        or static_contract["host_binary_content_parser_forbidden"] is not True
    ):
        raise GateError("execution static-audit contract drift")
    return current


def verify_parent_hashes(current: Mapping[str, Any]) -> list[dict[str, str]]:
    expected = current["g1_parent_hashes"]
    if not isinstance(expected, Mapping) or len(expected) != 9:
        raise GateError("G1 parent hash set must contain exactly nine files")
    observed: list[dict[str, str]] = []
    for relative, digest in expected.items():
        path = WORKSPACE_ROOT / relative
        actual = sha256_file(path, limit=2 * 1024 * 1024)
        if actual != digest:
            raise GateError(f"G1 parent file drift: {relative}")
        observed.append({"path": relative, "sha256": actual})
    return observed


def verify_system_tools(current: Mapping[str, Any]) -> list[dict[str, str]]:
    expected = current["system_tools"]
    if not isinstance(expected, list) or len(expected) != 8:
        raise GateError("authorized system tool set drift")
    observed: list[dict[str, str]] = []
    for item in expected:
        if set(item) != {"path", "sha256"}:
            raise GateError("system tool identity object is not closed")
        path = Path(item["path"])
        actual = sha256_file(path, limit=64 * 1024 * 1024)
        if actual != item["sha256"]:
            raise GateError(f"authorized system tool drift: {path}")
        observed.append({"path": str(path), "sha256": actual})
    return observed


def verify_g1_contract(*, require_a_absent: bool) -> dict[str, Any]:
    current = verify_policy_contract()
    parents = verify_parent_hashes(current)
    g1_policy = g1.validate_policy_schema()
    result = {
        "plan": g1.verify_plan_identity(),
        "frozen_references": g1.verify_frozen_references(g1_policy),
        "source_inputs": g1.verify_source_inputs(g1_policy),
        "objects": g1.verify_object_contract(g1_policy),
        "wrapper": g1.verify_wrapper_contract(g1_policy),
        "resources": g1.verify_resources_and_make_argv(g1_policy),
        "tools": g1.verify_tool_identities(g1_policy),
        "profiles": g1.verify_profiles(g1_policy),
        "static_audit": g1.verify_static_audit_contract(g1_policy),
        "parents": parents,
    }
    if require_a_absent and os.path.lexists(ROOT_A):
        raise GateError("Attempt A root exists before its create-new transition")
    if os.path.lexists(ROOT_B):
        raise GateError("Attempt B root exists")
    for key in ("attempt_b_copy", "attempt_b_build"):
        planned = WORKSPACE_ROOT / g1_policy["profiles"][key]["planned_path"]
        if os.path.lexists(planned):
            raise GateError(f"forbidden B profile exists: {planned}")
    return result


def source_copy_argv(current: Mapping[str, Any] | None = None) -> list[str]:
    current = dict(current or policy())
    g1_policy = g1.read_json_object(g1.POLICY_PATH)
    argv = list(g1_policy["source_copy_contract"]["full_execution_argv"])
    delta = current["source_copy"]["only_delta"]
    if argv.count(delta["token_from"]) != delta["replacement_count"]:
        raise GateError("source-copy PATH delta source token count drift")
    if argv.count(delta["token_from"]) != 1 or delta["token_to"] != "PATH=/usr/bin":
        raise GateError("source-copy PATH is not the unique /usr/bin tightening")
    argv[argv.index(delta["token_from"])] = delta["token_to"]
    if "PATH=/usr/bin:/usr/local/cuda-12.8/bin" in argv:
        raise GateError("source-copy child retains CUDA on PATH")
    if argv.count("PATH=/usr/bin") != 1:
        raise GateError("source-copy child PATH is not uniquely /usr/bin")
    return argv


def build_argv() -> list[str]:
    g1_policy = g1.read_json_object(g1.POLICY_PATH)
    argv = list(g1_policy["build_contract"]["full_execution_argv"])
    make_argv = list(g1_policy["build_contract"]["make_argv"])
    if argv[-len(make_argv) :] != make_argv or tuple(make_argv) != g1.MAKE_ARGV:
        raise GateError("G1 build/Make argv reuse drift")
    if make_argv.count("-j1") != 1 or any(token in "\n".join(make_argv) for token in ("-j2", "-j4")):
        raise GateError("Attempt A Make concurrency drift")
    return argv


def static_command_plan() -> list[tuple[str, str]]:
    current = g1.read_json_object(g1.POLICY_PATH)
    candidate = str(CANDIDATE_PATH)
    plan: list[tuple[str, str]] = [
        (candidate, item["id"])
        for item in current["static_audit_contract"]["candidate_tool_suffixes"]
    ]
    cuda = set(current["object_contract"]["cuda_object_names"])
    for name in current["object_contract"]["object_names"]:
        host = str(OUTPUT_SOURCE / name)
        for item in current["static_audit_contract"]["object_tool_suffix_templates"]:
            if item.get("cuda_only", False) and name not in cuda:
                continue
            plan.append((host, item["id"]))
    if len(plan) != 557:
        raise GateError(f"static command count drift: {len(plan)}")
    for host, suffix in plan:
        g1.validate_static_audit_argv(g1.build_static_audit_argv(host, suffix), current)
    return plan


def assert_runtime_identity() -> dict[str, Any]:
    observed = {
        "uid": os.getuid(),
        "euid": os.geteuid(),
        "gid": os.getgid(),
        "egid": os.getegid(),
        "supplementary_groups": os.getgroups(),
    }
    expected = {
        "uid": 1000,
        "euid": 1000,
        "gid": 1000,
        "egid": 1000,
        "supplementary_groups": [],
    }
    if observed != expected:
        raise GateError(f"setpriv identity drift: {observed!r}")
    return observed


def assert_no_symlink_components(path: Path, *, require_leaf: bool) -> None:
    absolute = path.absolute()
    current = Path("/")
    for index, part in enumerate(absolute.parts[1:]):
        current /= part
        leaf = index == len(absolute.parts[1:]) - 1
        try:
            info = os.lstat(current)
        except FileNotFoundError:
            if leaf and not require_leaf:
                return
            raise GateError(f"required path component absent: {current}")
        if stat.S_ISLNK(info.st_mode):
            raise GateError(f"symlink path component forbidden: {current}")


def assert_directory(path: Path, *, mode: int | None = None) -> os.stat_result:
    assert_no_symlink_components(path, require_leaf=True)
    info = os.stat(path, follow_symlinks=False)
    if not stat.S_ISDIR(info.st_mode):
        raise GateError(f"not a directory: {path}")
    if mode is not None and stat.S_IMODE(info.st_mode) != mode:
        raise GateError(f"directory mode drift at {path}: {stat.S_IMODE(info.st_mode):04o}")
    return info


def open_new(path: Path, *, mode: int) -> int:
    assert_no_symlink_components(path.parent, require_leaf=True)
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        mode,
    )
    os.fchmod(descriptor, mode)
    return descriptor


def write_all(descriptor: int, data: bytes) -> None:
    offset = 0
    while offset < len(data):
        written = os.write(descriptor, data[offset:])
        if written <= 0:
            raise GateError("short write to create-new evidence")
        offset += written


def write_bytes_new(path: Path, data: bytes, *, mode: int) -> None:
    descriptor = open_new(path, mode=mode)
    try:
        write_all(descriptor, data)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def gate_identity() -> dict[str, Any]:
    info = os.lstat(SCRIPT_PATH)
    if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode) or info.st_nlink != 1:
        raise GateError("execution gate file identity is unsafe")
    return {
        "path": str(SCRIPT_PATH),
        "mode_octal": format(stat.S_IMODE(info.st_mode), "04o"),
        "size_bytes": info.st_size,
        "sha256": sha256_file(SCRIPT_PATH, limit=4 * 1024 * 1024),
    }


def references(extra: Iterable[Path] = ()) -> list[dict[str, str]]:
    current = policy()
    result = [
        {"path": str(WORKSPACE_ROOT / relative), "sha256": digest}
        for relative, digest in current["g1_parent_hashes"].items()
    ]
    result.extend(
        [
            {"path": str(POLICY_PATH), "sha256": POLICY_SHA256},
            {"path": str(RECEIPT_SCHEMA_PATH), "sha256": RECEIPT_SCHEMA_SHA256},
        ]
    )
    for path in extra:
        result.append({"path": str(path), "sha256": sha256_file(path)})
    return result


def safety(profile_name: str, make_count: int = 0) -> dict[str, Any]:
    return {
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "candidate_executed": False,
        "network_used": False,
        "gpu_device_exposed": False,
        "binary_content_parser_executed_on_host": False,
        "make_invocation_count": make_count,
        "expected_profile_name": profile_name,
    }


def make_receipt(
    *,
    document_type: str,
    phase: str,
    status: str,
    preflight_data: Mapping[str, Any] | None = None,
    execution: Mapping[str, Any] | None = None,
    inventories: Sequence[Mapping[str, Any]] = (),
    trace: Sequence[Mapping[str, Any]] = (),
    resource_samples: Sequence[Mapping[str, Any]] = (),
    artifacts: Sequence[Mapping[str, Any]] = (),
    logs: Sequence[Mapping[str, Any]] = (),
    static_checks: Sequence[Mapping[str, Any]] = (),
    receipt_references: Sequence[Mapping[str, str]] = (),
    safety_data: Mapping[str, Any],
    next_allowed_stage: str,
) -> dict[str, Any]:
    return {
        "schema_version": "smpcc-r8-liquid-u3-gpu-build-execution-receipt-v1",
        "document_type": document_type,
        "campaign_id": CAMPAIGN_ID,
        "build_id": BUILD_ID_A,
        "attempt_root": str(ROOT_A),
        "phase": phase,
        "status": status,
        "created_at_utc": utc_now(),
        "gate_identity": gate_identity(),
        "preflight": dict(preflight_data) if preflight_data is not None else None,
        "execution": dict(execution) if execution is not None else None,
        "inventories": [dict(item) for item in inventories],
        "trace": [dict(item) for item in trace],
        "resource_samples": [dict(item) for item in resource_samples],
        "artifacts": [dict(item) for item in artifacts],
        "logs": [dict(item) for item in logs],
        "static_checks": [dict(item) for item in static_checks],
        "references": [dict(item) for item in receipt_references],
        "safety": dict(safety_data),
        "next_allowed_stage": next_allowed_stage,
    }


def validate_receipt(value: Mapping[str, Any]) -> None:
    schema = receipt_schema()
    errors = sorted(
        Draft202012Validator(schema).iter_errors(dict(value)),
        key=lambda error: tuple(str(part) for part in error.absolute_path),
    )
    if errors:
        first = errors[0]
        location = "/".join(str(part) for part in first.absolute_path)
        raise GateError(f"closed receipt schema rejection at {location}: {first.message}")


def write_receipt(path: Path, value: Mapping[str, Any]) -> None:
    validate_receipt(value)
    data = json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2).encode("utf-8") + b"\n"
    write_bytes_new(path, data, mode=0o640)


def read_receipt(path: Path) -> dict[str, Any]:
    value = read_json_object(path, limit=64 * 1024 * 1024)
    validate_receipt(value)
    return value


def static_check(identifier: str, passed: bool, evidence: str) -> dict[str, Any]:
    bounded = evidence[:STREAM_PREFIX_LIMIT]
    return {
        "id": identifier,
        "passed": passed,
        "evidence": bounded,
        "evidence_sha256": hashlib.sha256(evidence.encode("utf-8")).hexdigest(),
    }


def next_event_ns(previous: int = 0) -> int:
    return max(time.time_ns(), previous + 1)


def trace_event(state: str, sequence: int, evidence: Mapping[str, Any], previous: int = 0) -> dict[str, Any]:
    encoded = canonical_json(evidence)
    return {
        "state": state,
        "sequence": sequence,
        "captured_at_ns": next_event_ns(previous),
        "evidence_json": encoded.decode("utf-8"),
        "evidence_sha256": hashlib.sha256(encoded).hexdigest(),
    }


def g1_trace(schema_trace: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for item in schema_trace:
        evidence_text = item["evidence_json"]
        if hashlib.sha256(evidence_text.encode("utf-8")).hexdigest() != item["evidence_sha256"]:
            raise GateError("source-copy trace evidence hash drift")
        evidence = json.loads(evidence_text)
        if not isinstance(evidence, dict):
            raise GateError("source-copy trace evidence is not an object")
        result.append(
            {
                "state": item["state"],
                "sequence": item["sequence"],
                "captured_at_ns": item["captured_at_ns"],
                "evidence": evidence,
            }
        )
    return result


def source_contract() -> tuple[Path, dict[str, dict[str, Any]]]:
    current = g1.read_json_object(g1.POLICY_PATH)["source_input"]
    receipt_path = Path(current["source_receipt"])
    if sha256_file(receipt_path, limit=16 * 1024 * 1024) != current["source_receipt_sha256"]:
        raise GateError("sealed source receipt SHA-256 drift")
    receipt = read_json_object(receipt_path, limit=16 * 1024 * 1024)
    try:
        entries = receipt["results"]["materialization"]["sealed_output"]["entries"]
    except (KeyError, TypeError) as exc:
        raise GateError("sealed source receipt lacks its entry manifest") from exc
    if not isinstance(entries, list) or len(entries) != 352:
        raise GateError("sealed source manifest count drift")
    expected: dict[str, dict[str, Any]] = {}
    for item in entries:
        if not isinstance(item, dict):
            raise GateError("sealed source entry is malformed")
        path = item.get("path")
        if not isinstance(path, str) or not path.startswith("src/source/"):
            raise GateError("sealed source entry leaves src/source")
        relative = path.removeprefix("src/source/")
        if not relative or relative.startswith("/") or ".." in Path(relative).parts:
            raise GateError("unsafe sealed source relative path")
        if relative in expected:
            raise GateError("duplicate sealed source path")
        digest = item.get("sha256")
        size = item.get("size_bytes")
        if not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None:
            raise GateError("malformed sealed source SHA-256")
        if not isinstance(size, int) or size < 0:
            raise GateError("malformed sealed source size")
        expected[relative] = {"sha256": digest, "size_bytes": size}
    if sum(item["size_bytes"] for item in expected.values()) != 5_473_917:
        raise GateError("sealed source total bytes drift")
    return Path(current["sealed_source_root"]), expected


def inventory_input_tree(
    root: Path,
    expected: Mapping[str, Mapping[str, Any]],
    *,
    wrapper: bool,
    label: str,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    assert_no_symlink_components(root, require_leaf=True)
    if root.is_symlink() or not root.is_dir():
        raise GateError(f"unsafe input tree root: {root}")
    observed: dict[str, dict[str, Any]] = {}
    entries: list[dict[str, Any]] = []
    wrapper_count = 0
    total_bytes = 0
    for directory, subdirs, files in os.walk(root, topdown=True, followlinks=False):
        directory_path = Path(directory)
        directory_info = os.lstat(directory_path)
        if not stat.S_ISDIR(directory_info.st_mode) or stat.S_ISLNK(directory_info.st_mode):
            raise GateError(f"unsafe directory in input tree: {directory_path}")
        for name in subdirs:
            info = os.lstat(directory_path / name)
            if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
                raise GateError(f"symlink/special input directory forbidden: {directory_path / name}")
        for name in files:
            path = directory_path / name
            info = os.lstat(path)
            relative = str(path.relative_to(root))
            if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
                raise GateError(f"symlink/special input file forbidden: {relative}")
            if info.st_nlink != 1:
                raise GateError(f"hardlinked input file forbidden: {relative}")
            mode = stat.S_IMODE(info.st_mode)
            if mode & 0o111:
                raise GateError(f"executable input file forbidden: {relative}")
            descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
            try:
                magic = os.read(descriptor, 4)
                digest = sha256_fd(descriptor)
            finally:
                os.close(descriptor)
            is_elf = magic == b"\x7fELF"
            if is_elf:
                raise GateError(f"ELF input forbidden before build: {relative}")
            if relative in expected:
                expected_item = expected[relative]
                if info.st_size != expected_item["size_bytes"] or digest != expected_item["sha256"]:
                    raise GateError(f"copied source identity drift: {relative}")
                observed[relative] = {"sha256": digest, "size_bytes": info.st_size}
            elif wrapper and relative == WRAPPER_NAME:
                if (
                    info.st_size != len(g1.WRAPPER_BYTES)
                    or digest != g1.WRAPPER_SHA256
                    or mode != g1.WRAPPER_MODE
                ):
                    raise GateError("wrapper mode/size/SHA-256 drift")
                wrapper_count += 1
            else:
                raise GateError(f"extra input file forbidden: {relative}")
            total_bytes += info.st_size
            entries.append(
                {
                    "path": relative,
                    "kind": "regular",
                    "symlink": False,
                    "nlink": info.st_nlink,
                    "mode_octal": format(mode, "04o"),
                    "size_bytes": info.st_size,
                    "sha256": digest,
                    "is_elf": is_elf,
                }
            )
    if set(observed) != set(expected):
        missing = sorted(set(expected) - set(observed))
        raise GateError(f"input tree missing sealed source files: {missing[:8]}")
    entries.sort(key=lambda item: item["path"])
    summary = {
        "label": label,
        "entry_count": len(entries),
        "sealed_entry_count": len(observed),
        "wrapper_entry_count": wrapper_count,
        "extra_count": 0,
        "symlink_count": 0,
        "hardlink_count": 0,
        "elf_count": 0,
        "executable_count": 0,
        "total_bytes": total_bytes,
        "manifest_sha256": canonical_hash(entries),
    }
    required_count = 353 if wrapper else 352
    if len(entries) != required_count or wrapper_count != (1 if wrapper else 0):
        raise GateError(f"input inventory count drift for {label}")
    return summary, entries


def memory_fields() -> tuple[int, int]:
    values: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        key, _, value = line.partition(":")
        parts = value.split()
        if len(parts) >= 2 and parts[1] == "kB" and parts[0].isdigit():
            values[key] = int(parts[0]) * 1024
    if "MemAvailable" not in values or "SwapFree" not in values:
        raise GateError("MemAvailable or SwapFree is unavailable")
    return values["MemAvailable"], values["SwapFree"]


def vmstat_swap() -> tuple[int, int]:
    values: dict[str, int] = {}
    for line in Path("/proc/vmstat").read_text(encoding="ascii").splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] in {"pswpin", "pswpout"}:
            values[parts[0]] = int(parts[1])
    return values.get("pswpin", 0), values.get("pswpout", 0)


def memory_psi() -> tuple[str, str]:
    some = ""
    full = ""
    for line in Path("/proc/pressure/memory").read_text(encoding="ascii").splitlines():
        if line.startswith("some "):
            some = line
        elif line.startswith("full "):
            full = line
    return some, full


def process_rss() -> list[dict[str, Any]]:
    wanted = {"cicc", "ptxas", "cc1plus"}
    result: list[dict[str, Any]] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            name = (entry / "comm").read_text(encoding="utf-8").strip()
            if name not in wanted:
                continue
            rss_pages = int((entry / "statm").read_text(encoding="ascii").split()[1])
            result.append(
                {
                    "name": name,
                    "pid": int(entry.name),
                    "rss_bytes": rss_pages * os.sysconf("SC_PAGE_SIZE"),
                }
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
            continue
    return sorted(result, key=lambda item: (item["name"], item["pid"]))


def temperature_and_power() -> list[dict[str, str]]:
    result: list[dict[str, str]] = []
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        try:
            sensor_type = (zone / "type").read_text(encoding="utf-8").strip()
            value = (zone / "temp").read_text(encoding="ascii").strip()
            result.append({"name": f"thermal:{zone.name}:{sensor_type}", "value": value})
        except (FileNotFoundError, PermissionError, OSError):
            continue
    for hwmon in sorted(Path("/sys/class/hwmon").glob("hwmon*")):
        try:
            hwname = (hwmon / "name").read_text(encoding="utf-8").strip()
        except (FileNotFoundError, PermissionError, OSError):
            hwname = hwmon.name
        for sensor in sorted(hwmon.glob("temp*_input")):
            try:
                value = sensor.read_text(encoding="ascii").strip()
                result.append({"name": f"hwmon:{hwname}:{sensor.name}", "value": value})
            except (FileNotFoundError, PermissionError, OSError):
                continue
    for supply in sorted(Path("/sys/class/power_supply").glob("*")):
        for field in ("online", "status", "power_now"):
            path = supply / field
            try:
                value = path.read_text(encoding="utf-8").strip()
                result.append({"name": f"power:{supply.name}:{field}", "value": value})
            except (FileNotFoundError, PermissionError, OSError):
                continue
    return result[:1024]


def resource_sample(started: float) -> dict[str, Any]:
    available, swap = memory_fields()
    si, so = vmstat_swap()
    psi_some, psi_full = memory_psi()
    return {
        "captured_at_utc": utc_now(),
        "monotonic_seconds": round(max(0.0, time.monotonic() - started), 6),
        "memory_available_bytes": available,
        "swap_free_bytes": swap,
        "vmstat_si_pages": si,
        "vmstat_so_pages": so,
        "memory_psi_some": psi_some,
        "memory_psi_full": psi_full,
        "disk_free_bytes": shutil.disk_usage(LIQUID_ROOT).free,
        "compiler_processes": process_rss(),
        "temperature_and_power": temperature_and_power(),
    }


def ancestor_pids() -> set[int]:
    result: set[int] = set()
    pid = os.getpid()
    while pid > 1 and pid not in result:
        result.add(pid)
        try:
            raw = (Path("/proc") / str(pid) / "stat").read_text(encoding="utf-8")
            tail = raw[raw.rfind(")") + 2 :].split()
            pid = int(tail[1])
        except (FileNotFoundError, PermissionError, ValueError, IndexError):
            break
    result.add(1)
    return result


def conflicting_processes() -> list[str]:
    exact_names = {
        "make",
        "nvcc",
        "cicc",
        "ptxas",
        "fatbinary",
        "nvlink",
        "cc1plus",
        "bwrap",
        "aa-exec",
        "DualSPHysics5.4_linux64",
    }
    ancestors = ancestor_pids()
    result: list[str] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal() or int(entry.name) in ancestors:
            continue
        try:
            comm = (entry / "comm").read_text(encoding="utf-8").strip()
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", "replace"
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if comm in exact_names or str(ROOT_A) in cmdline or str(ROOT_B) in cmdline:
            result.append(f"pid={entry.name} comm={comm} argv={cmdline[:3072]}")
    return sorted(result)


def preflight_common(*, require_a_absent: bool, candidate_may_exist: bool) -> dict[str, Any]:
    invoker = assert_runtime_identity()
    review = verify_g1_contract(require_a_absent=require_a_absent)
    current = policy()
    verify_system_tools(current)
    schema = receipt_schema()
    Draft202012Validator.check_schema(schema)
    verify_recursive_object_closure(schema)
    source_root, expected = source_contract()
    source_summary, _ = inventory_input_tree(
        source_root, expected, wrapper=False, label="sealed_source_352"
    )
    root_info = assert_directory(LIQUID_ROOT, mode=0o750)
    if root_info.st_uid != 1000 or root_info.st_gid != 1000:
        raise GateError("scout_liquid_lab ownership drift")
    for path in (LIQUID_ROOT / "build", LIQUID_ROOT / "audits"):
        info = assert_directory(path, mode=0o750)
        if info.st_uid != 1000 or info.st_gid != 1000:
            raise GateError(f"liquid execution directory ownership drift: {path}")
    available, swap = memory_fields()
    disk = shutil.disk_usage(LIQUID_ROOT).free
    if available < MINIMUM_AVAILABLE_MEMORY_BYTES:
        raise GateError(f"MemAvailable below 4 GiB: {available}")
    if disk < MINIMUM_AVAILABLE_DISK_BYTES:
        raise GateError(f"free disk below 20 GiB: {disk}")
    conflicts = conflicting_processes()
    if conflicts:
        raise GateError(f"conflicting build processes: {conflicts}")
    candidate_exists = os.path.lexists(CANDIDATE_PATH)
    if candidate_exists and not candidate_may_exist:
        raise GateError("candidate exists before its admitted phase")
    candidate_remaining = CANDIDATE_DEADLINE_EPOCH - time.time()
    final_remaining = FINAL_DEADLINE_EPOCH - time.time()
    if final_remaining <= 0:
        raise GateError("T+8 final deadline has elapsed")
    if not candidate_may_exist and candidate_remaining <= 0:
        raise GateError("T+6 candidate deadline has elapsed")
    parent_map = {
        item["path"]: item["sha256"] for item in review["parents"]
    }
    source_input = g1.read_json_object(g1.POLICY_PATH)["source_input"]
    return {
        "captured_at_utc": utc_now(),
        "invoker": invoker,
        "plan_sha256": g1.PLAN_SHA256,
        "g1_parent_set_canonical_sha256": canonical_hash(parent_map),
        "execution_policy_sha256": POLICY_SHA256,
        "receipt_schema_sha256": RECEIPT_SCHEMA_SHA256,
        "source_receipt_sha256": source_input["source_receipt_sha256"],
        "sealed_manifest_sha256": source_input["sealed_manifest_sha256"],
        "sealed_file_count": source_summary["sealed_entry_count"],
        "sealed_total_bytes": source_input["sealed_total_bytes"],
        "memory_available_bytes": available,
        "swap_free_bytes": swap,
        "disk_free_bytes": disk,
        "candidate_deadline_remaining_seconds": round(candidate_remaining, 6),
        "final_deadline_remaining_seconds": round(final_remaining, 6),
        "conflicting_processes": conflicts,
        "attempt_b_root_exists": False,
        "candidate_exists": candidate_exists,
    }


def campaign_paths() -> tuple[Path, ...]:
    return (
        SOURCE_COPY_START,
        SOURCE_COPY_RUN,
        SOURCE_COPY_FINAL,
        BUILD_START,
        BUILD_FINAL,
        BUILD_STDOUT,
        BUILD_STDERR,
        BUILD_MONITOR,
        STATIC_AUDIT_FINAL,
        STATIC_PARSER_OUTPUT,
        STATIC_COMMAND_SUMMARY,
    )


def assert_zero_materialization() -> None:
    for path in (ROOT_A, ROOT_B, *campaign_paths()):
        if os.path.lexists(path):
            raise GateError(f"create-new baseline path exists: {path}")
    audits = LIQUID_ROOT / "audits"
    unexpected = [
        path
        for path in audits.iterdir()
        if path.name.startswith(CAMPAIGN_ID) and path not in campaign_paths()
    ]
    if unexpected:
        raise GateError(f"unexpected campaign receipt path exists: {unexpected}")


def execution_result(
    argv: Sequence[str],
    return_code: int,
    elapsed: float,
    stdout: bytes,
    stderr: bytes,
    *,
    timed_out: bool,
    deadline_hit: bool,
) -> dict[str, Any]:
    return {
        "argv": list(argv),
        "return_code": return_code,
        "elapsed_seconds": round(elapsed, 6),
        "stdout_bytes": len(stdout),
        "stdout_sha256": hashlib.sha256(stdout).hexdigest(),
        "stdout_prefix_utf8": stdout[:STREAM_PREFIX_LIMIT].decode("utf-8", "replace"),
        "stderr_bytes": len(stderr),
        "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
        "stderr_prefix_utf8": stderr[:STREAM_PREFIX_LIMIT].decode("utf-8", "replace"),
        "timed_out": timed_out,
        "candidate_deadline_hit": deadline_hit,
    }


def terminate_process_group(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def run_capture_bounded(
    argv: Sequence[str],
    *,
    output_limit: int,
    deadline_epoch: float | None = None,
) -> tuple[dict[str, Any], bytes, bytes]:
    started = time.monotonic()
    process = subprocess.Popen(
        list(argv),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
        close_fds=True,
    )
    if process.stdout is None or process.stderr is None:
        terminate_process_group(process)
        raise GateError("cannot capture child stdout/stderr")
    selector = selectors.DefaultSelector()
    outputs = {process.stdout.fileno(): bytearray(), process.stderr.fileno(): bytearray()}
    selector.register(process.stdout, selectors.EVENT_READ)
    selector.register(process.stderr, selectors.EVENT_READ)
    deadline_hit = False
    exceeded = False
    while selector.get_map():
        if deadline_epoch is not None and time.time() >= deadline_epoch:
            deadline_hit = True
            terminate_process_group(process)
        events = selector.select(timeout=0.25)
        for key, _ in events:
            try:
                block = os.read(key.fd, 65536)
            except BlockingIOError:
                continue
            if not block:
                selector.unregister(key.fileobj)
                continue
            outputs[key.fd].extend(block)
            if sum(len(value) for value in outputs.values()) > output_limit:
                exceeded = True
                terminate_process_group(process)
        if exceeded:
            for key in list(selector.get_map().values()):
                try:
                    block = os.read(key.fd, 65536)
                except (BlockingIOError, OSError):
                    block = b""
                if not block:
                    try:
                        selector.unregister(key.fileobj)
                    except KeyError:
                        pass
            if process.poll() is not None and not selector.get_map():
                break
    return_code = process.wait()
    stdout = bytes(outputs[process.stdout.fileno()])
    stderr = bytes(outputs[process.stderr.fileno()])
    if exceeded:
        raise GateError(f"child parser output exceeded {output_limit} bytes")
    result = execution_result(
        argv,
        return_code,
        time.monotonic() - started,
        stdout,
        stderr,
        timed_out=return_code in {124, 137} or deadline_hit,
        deadline_hit=deadline_hit,
    )
    return result, stdout, stderr


def create_attempt_tree() -> None:
    assert_no_symlink_components(ROOT_A.parent, require_leaf=True)
    layout = (
        (ROOT_A, 0o750),
        (OUTPUT_ROOT, 0o700),
        (OUTPUT_ROOT / "buildtree", 0o700),
        (OUTPUT_ROOT / "buildtree/src", 0o700),
        (OUTPUT_SOURCE, 0o700),
        (ARTIFACTS_ROOT, 0o700),
    )
    for path, mode in layout:
        os.mkdir(path, mode)
        os.chmod(path, mode, follow_symlinks=False)


def create_wrapper() -> dict[str, Any]:
    path = OUTPUT_SOURCE / WRAPPER_NAME
    descriptor = os.open(
        path,
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | os.O_NOFOLLOW
        | os.O_CLOEXEC,
        g1.WRAPPER_MODE,
    )
    try:
        os.fchmod(descriptor, g1.WRAPPER_MODE)
        write_all(descriptor, g1.WRAPPER_BYTES)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    identity = artifact_identity(path, kind="wrapper")
    if (
        identity["mode_octal"] != "0600"
        or identity["size_bytes"] != 84
        or identity["sha256"] != g1.WRAPPER_SHA256
    ):
        raise GateError("materialized wrapper contract drift")
    return identity


def artifact_identity(path: Path, *, kind: str) -> dict[str, Any]:
    info = os.lstat(path)
    if stat.S_ISLNK(info.st_mode):
        raise GateError(f"artifact symlink forbidden: {path}")
    regular = stat.S_ISREG(info.st_mode)
    if not regular or info.st_nlink != 1:
        raise GateError(f"artifact must be regular nlink=1: {path}")
    return {
        "path": str(path),
        "kind": kind,
        "realpath": str(path.resolve(strict=True)),
        "parent_realpath": str(path.parent.resolve(strict=True)),
        "uid": info.st_uid,
        "gid": info.st_gid,
        "st_dev": info.st_dev,
        "st_ino": info.st_ino,
        "regular": regular,
        "symlink": False,
        "nlink": info.st_nlink,
        "size_bytes": info.st_size,
        "mode_octal": format(stat.S_IMODE(info.st_mode), "04o"),
        "sha256": sha256_file(path, limit=2 * 1024 * 1024 * 1024),
    }


def log_identity(path: Path, role: str) -> dict[str, Any]:
    info = os.lstat(path)
    if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode) or info.st_nlink != 1:
        raise GateError(f"unsafe log identity: {path}")
    return {
        "role": role,
        "path": str(path),
        "mode_octal": format(stat.S_IMODE(info.st_mode), "04o"),
        "size_bytes": info.st_size,
        "sha256": sha256_file(path, limit=4 * 1024 * 1024 * 1024),
    }


def self_check() -> dict[str, Any]:
    current = verify_policy_contract()
    schema = receipt_schema()
    Draft202012Validator.check_schema(schema)
    verify_recursive_object_closure(schema)
    review = verify_g1_contract(require_a_absent=True)
    g1_static = g1.self_check()
    tools = verify_system_tools(current)
    assert_zero_materialization()
    copy_argv = source_copy_argv(current)
    make = build_argv()
    audit_plan = static_command_plan()
    source_root, expected = source_contract()
    sealed, _ = inventory_input_tree(
        source_root, expected, wrapper=False, label="sealed_source_352"
    )
    return {
        "status": "PASS_G2A_EXECUTION_LAYER_STATIC_SELF_CHECK",
        "gate_identity": gate_identity(),
        "policy": {"path": str(POLICY_PATH), "sha256": POLICY_SHA256},
        "receipt_schema": {
            "path": str(RECEIPT_SCHEMA_PATH),
            "sha256": RECEIPT_SCHEMA_SHA256,
            "recursive_object_closed": True,
        },
        "g1_status": g1_static["status"],
        "g1_parent_count": len(review["parents"]),
        "system_tool_count": len(tools),
        "source_copy_path": copy_argv[copy_argv.index("PATH=/usr/bin")],
        "build_argv_sha256": canonical_hash(make),
        "make_argv": list(g1.MAKE_ARGV),
        "static_command_count": len(audit_plan),
        "sealed_source": sealed,
        "system_actions_performed": False,
        "make_nvcc_compiler_run": False,
        "profile_loaded": False,
        "candidate_executed": False,
    }


def preflight() -> dict[str, Any]:
    assert_zero_materialization()
    result = preflight_common(require_a_absent=True, candidate_may_exist=False)
    return {"status": "PASS_G2_ATTEMPT_A_PREFLIGHT", "preflight": result}


def prepare_source_copy() -> dict[str, Any]:
    assert_zero_materialization()
    preflight_data = preflight_common(require_a_absent=True, candidate_may_exist=False)
    event = trace_event(
        g1.SOURCE_COPY_STATES[0],
        1,
        {"create_new": True, "path": str(SOURCE_COPY_START)},
    )
    receipt = make_receipt(
        document_type="SMPCC_R8_LIQUID_U3_GPU_SOURCE_COPY_START",
        phase="prepare-source-copy",
        status="STARTED_A_GPU_SOURCE_COPY",
        preflight_data=preflight_data,
        trace=[event],
        static_checks=[static_check("root_a_absent_before_start_record", True, str(ROOT_A))],
        receipt_references=references(),
        safety_data=safety("NONE"),
        next_allowed_stage="CREATE_FRESH_ROOT_A_THEN_LOAD_EXACT_A_COPY_PROFILE",
    )
    write_receipt(SOURCE_COPY_START, receipt)
    create_attempt_tree()
    return {
        "status": "SOURCE_COPY_START_AND_ROOT_A_CREATE_NEW",
        "start_record": str(SOURCE_COPY_START),
        "start_record_sha256": sha256_file(SOURCE_COPY_START),
        "root_a": str(ROOT_A),
    }


def run_source_copy() -> int:
    assert_runtime_identity()
    verify_g1_contract(require_a_absent=False)
    if os.path.lexists(SOURCE_COPY_RUN) or os.path.lexists(SOURCE_COPY_FINAL):
        raise GateError("source-copy run/final receipt already exists")
    start = read_receipt(SOURCE_COPY_START)
    if start["status"] != "STARTED_A_GPU_SOURCE_COPY" or start["phase"] != "prepare-source-copy":
        raise GateError("source-copy start receipt does not admit run-source-copy")
    for path, mode in (
        (ROOT_A, 0o750),
        (OUTPUT_ROOT, 0o700),
        (OUTPUT_SOURCE, 0o700),
        (ARTIFACTS_ROOT, 0o700),
    ):
        assert_directory(path, mode=mode)
    source_root, expected = source_contract()
    del source_root
    argv = source_copy_argv()
    host_userns_before = Path("/proc/sys/user/max_user_namespaces").read_text(encoding="ascii").strip()
    execution, stdout, stderr = run_capture_bounded(argv, output_limit=16 * 1024 * 1024)
    host_userns_after = Path("/proc/sys/user/max_user_namespaces").read_text(encoding="ascii").strip()
    trace = list(start["trace"])
    inventories: list[dict[str, Any]] = []
    status = "A_GPU_SOURCE_COPY_FAILED_NO_RETRY"
    next_stage = "STOP_PRESERVE_ROOT_AND_RECEIPTS"
    checks = [
        static_check(
            "host_userns_unchanged",
            host_userns_before == host_userns_after,
            f"before={host_userns_before};after={host_userns_after}",
        )
    ]
    if host_userns_before != host_userns_after:
        execution["return_code"] = execution["return_code"] or 125
    if execution["return_code"] == 0 and host_userns_before == host_userns_after:
        try:
            copied, _ = inventory_input_tree(
                OUTPUT_SOURCE,
                expected,
                wrapper=False,
                label="copy_profile_loaded_352",
            )
            inventories.append(copied)
            previous = trace[-1]["captured_at_ns"]
            trace.append(
                trace_event(
                    g1.SOURCE_COPY_STATES[1],
                    2,
                    {"isolated": True, "copied_entry_count": 352},
                    previous,
                )
            )
            trace.append(
                trace_event(
                    g1.SOURCE_COPY_STATES[2],
                    3,
                    {
                        "entry_count": 352,
                        "extra_count": 0,
                        "symlink_count": 0,
                        "hardlink_count": 0,
                        "elf_count": 0,
                        "executable_count": 0,
                    },
                    trace[-1]["captured_at_ns"],
                )
            )
            status = "PASS_A_GPU_SOURCE_COPY_RUN_AWAITING_PROFILE_UNLOAD"
            next_stage = "UNLOAD_A_COPY_PROFILE_VERIFY_ZERO_RESIDUE_THEN_FINALIZE_SOURCE_COPY"
        except (GateError, OSError, UnicodeError, ValueError) as exc:
            checks.append(static_check("copy_inventory_352", False, str(exc)))
            status = "A_GPU_SOURCE_COPY_POSTFLIGHT_FAILED_NO_RETRY"
    receipt = make_receipt(
        document_type="SMPCC_R8_LIQUID_U3_GPU_SOURCE_COPY_RUN",
        phase="run-source-copy",
        status=status,
        preflight_data=start["preflight"],
        execution=execution,
        inventories=inventories,
        trace=trace,
        static_checks=checks,
        receipt_references=references([SOURCE_COPY_START]),
        safety_data=safety(policy()["profiles"]["a_copy"]["name"]),
        next_allowed_stage=next_stage,
    )
    write_receipt(SOURCE_COPY_RUN, receipt)
    print(
        json.dumps(
            {
                "status": status,
                "receipt": str(SOURCE_COPY_RUN),
                "stdout_bytes": len(stdout),
                "stderr_bytes": len(stderr),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0 if status.startswith("PASS_") else 3


def finalize_source_copy() -> dict[str, Any]:
    assert_runtime_identity()
    verify_g1_contract(require_a_absent=False)
    if os.path.lexists(SOURCE_COPY_FINAL):
        raise GateError("source-copy final receipt already exists")
    run = read_receipt(SOURCE_COPY_RUN)
    if run["status"] != "PASS_A_GPU_SOURCE_COPY_RUN_AWAITING_PROFILE_UNLOAD":
        raise GateError("source-copy run receipt does not admit finalization")
    source_root, expected = source_contract()
    del source_root
    after_unload, _ = inventory_input_tree(
        OUTPUT_SOURCE, expected, wrapper=False, label="after_copy_profile_unload_352"
    )
    trace = list(run["trace"])
    trace.append(
        trace_event(
            g1.SOURCE_COPY_STATES[3],
            4,
            {
                "profile_name": policy()["profiles"]["a_copy"]["name"],
                "unloaded": True,
                "zero_residue": True,
            },
            trace[-1]["captured_at_ns"],
        )
    )
    wrapper = create_wrapper()
    trace.append(
        trace_event(
            g1.SOURCE_COPY_STATES[4],
            5,
            {
                "create_flags": ["O_WRONLY", "O_CREAT", "O_EXCL", "O_NOFOLLOW"],
                "mode_octal": "0600",
                "sha256": g1.WRAPPER_SHA256,
                "size_bytes": 84,
                "after_unload_sequence": 4,
            },
            trace[-1]["captured_at_ns"],
        )
    )
    complete, entries = inventory_input_tree(
        OUTPUT_SOURCE, expected, wrapper=True, label="complete_build_input_353"
    )
    g1.validate_build_input_inventory(entries, expected.keys())
    trace.append(
        trace_event(
            g1.SOURCE_COPY_STATES[5],
            6,
            {
                "entry_count": 353,
                "sealed_entry_count": 352,
                "wrapper_entry_count": 1,
                "extra_count": 0,
                "symlink_count": 0,
                "hardlink_count": 0,
                "elf_count": 0,
                "executable_count": 0,
                "wrapper_sha256": g1.WRAPPER_SHA256,
                "wrapper_mode_octal": "0600",
            },
            trace[-1]["captured_at_ns"],
        )
    )
    trace.append(
        trace_event(
            g1.SOURCE_COPY_STATES[6],
            7,
            {
                "create_new": True,
                "path": str(SOURCE_COPY_FINAL),
                "published": True,
                "after_complete_inventory_sequence": 6,
            },
            trace[-1]["captured_at_ns"],
        )
    )
    g1.validate_source_copy_trace(g1_trace(trace))
    receipt = make_receipt(
        document_type="SMPCC_R8_LIQUID_U3_GPU_SOURCE_COPY_RECEIPT",
        phase="finalize-source-copy",
        status="PASS_U3_GPU_SOURCE_COPY",
        preflight_data=run["preflight"],
        execution=run["execution"],
        inventories=[*run["inventories"], after_unload, complete],
        trace=trace,
        artifacts=[wrapper],
        static_checks=[
            static_check(
                "copy_profile_unloaded_zero_residue",
                True,
                "outer authorized supervisor verified exact profile absence before this fixed phase",
            ),
            static_check("g1_source_copy_trace_validator", True, "seven ordered events accepted"),
            static_check("g1_build_input_validator", True, "353 exact entries accepted"),
        ],
        receipt_references=references([SOURCE_COPY_START, SOURCE_COPY_RUN]),
        safety_data=safety("NONE"),
        next_allowed_stage="G2_STATE_A_GPU_BUILD_ADMISSION_READY",
    )
    write_receipt(SOURCE_COPY_FINAL, receipt)
    return {
        "SOURCE_COPY_STATUS": "PASS_U3_GPU_SOURCE_COPY",
        "G2_STATE": "A_GPU_BUILD_ADMISSION_READY",
        "receipt": str(SOURCE_COPY_FINAL),
        "receipt_sha256": sha256_file(SOURCE_COPY_FINAL),
    }


def prepare_build() -> dict[str, Any]:
    assert_runtime_identity()
    preflight_data = preflight_common(require_a_absent=False, candidate_may_exist=False)
    source_receipt = read_receipt(SOURCE_COPY_FINAL)
    if source_receipt["status"] != "PASS_U3_GPU_SOURCE_COPY":
        raise GateError("source-copy final receipt does not admit build")
    for path in (
        BUILD_START,
        BUILD_FINAL,
        BUILD_STDOUT,
        BUILD_STDERR,
        BUILD_MONITOR,
        STATIC_AUDIT_FINAL,
        STATIC_PARSER_OUTPUT,
        STATIC_COMMAND_SUMMARY,
    ):
        if os.path.lexists(path):
            raise GateError(f"build/static create-new path exists: {path}")
    source_root, expected = source_contract()
    del source_root
    inventory, entries = inventory_input_tree(
        OUTPUT_SOURCE, expected, wrapper=True, label="pre_build_complete_353"
    )
    g1.validate_build_input_inventory(entries, expected.keys())
    sample = resource_sample(time.monotonic())
    if sample["memory_available_bytes"] < MINIMUM_AVAILABLE_MEMORY_BYTES:
        raise GateError("MemAvailable fell below 4 GiB at build admission")
    receipt = make_receipt(
        document_type="SMPCC_R8_LIQUID_U3_GPU_BUILD_START",
        phase="prepare-build",
        status="STARTED_ATTEMPT_A_GPU_BUILD",
        preflight_data=preflight_data,
        inventories=[inventory],
        resource_samples=[sample],
        static_checks=[static_check("parallel_jobs_literal", True, "-j1")],
        receipt_references=references([SOURCE_COPY_FINAL]),
        safety_data=safety("NONE"),
        next_allowed_stage="LOAD_EXACT_A_BUILD_PROFILE_THEN_RUN_BUILD_ONCE",
    )
    write_receipt(BUILD_START, receipt)
    return {
        "status": "ATTEMPT_A_GPU_BUILD_START_CREATE_NEW",
        "receipt": str(BUILD_START),
        "receipt_sha256": sha256_file(BUILD_START),
        "memory_available_bytes": sample["memory_available_bytes"],
    }


def try_disarm_candidate() -> dict[str, Any] | None:
    if not os.path.lexists(CANDIDATE_PATH):
        return None
    info = os.lstat(CANDIDATE_PATH)
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise GateError("candidate is not regular/non-symlink/nlink=1")
    descriptor = os.open(CANDIDATE_PATH, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        if before.st_size <= 0:
            return None
        os.fchmod(descriptor, 0o400)
        after = os.fstat(descriptor)
        if stat.S_IMODE(after.st_mode) != 0o400:
            raise GateError("candidate immediate fchmod(0400) failed")
    finally:
        os.close(descriptor)
    identity = artifact_identity(CANDIDATE_PATH, kind="candidate")
    if identity["size_bytes"] <= 0 or identity["mode_octal"] != "0400":
        raise GateError("candidate disarm postcondition failed")
    return identity


def run_build_capture(argv: Sequence[str]) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any] | None, str | None]:
    stdout_fd = open_new(BUILD_STDOUT, mode=0o640)
    stderr_fd = open_new(BUILD_STDERR, mode=0o640)
    monitor_fd = open_new(BUILD_MONITOR, mode=0o640)
    started = time.monotonic()
    process: subprocess.Popen[bytes] | None = None
    first_sample: dict[str, Any] | None = None
    last_sample: dict[str, Any] | None = None
    stdout_hash = hashlib.sha256()
    stderr_hash = hashlib.sha256()
    stdout_bytes = 0
    stderr_bytes = 0
    stdout_prefix = bytearray()
    stderr_prefix = bytearray()
    deadline_hit = False
    first_disarm: dict[str, Any] | None = None
    first_disarm_time: str | None = None
    try:
        process = subprocess.Popen(
            list(argv),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            close_fds=True,
        )
        if process.stdout is None or process.stderr is None:
            raise GateError("cannot capture Make stdout/stderr")
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        selector.register(process.stderr, selectors.EVENT_READ, "stderr")
        next_sample = 0.0
        while selector.get_map() or process.poll() is None:
            now_monotonic = time.monotonic()
            if now_monotonic >= next_sample:
                sample = resource_sample(started)
                if first_sample is None:
                    first_sample = sample
                last_sample = sample
                write_all(monitor_fd, canonical_json(sample) + b"\n")
                next_sample = now_monotonic + MONITOR_INTERVAL_SECONDS
            if first_disarm is None:
                candidate = try_disarm_candidate()
                if candidate is not None:
                    first_disarm = candidate
                    first_disarm_time = utc_now()
            if time.time() >= CANDIDATE_DEADLINE_EPOCH and process.poll() is None:
                deadline_hit = True
                terminate_process_group(process)
            for key, _ in selector.select(timeout=1.0):
                try:
                    block = os.read(key.fd, 65536)
                except BlockingIOError:
                    continue
                if not block:
                    selector.unregister(key.fileobj)
                    continue
                if key.data == "stdout":
                    write_all(stdout_fd, block)
                    stdout_hash.update(block)
                    stdout_bytes += len(block)
                    if len(stdout_prefix) < STREAM_PREFIX_LIMIT:
                        stdout_prefix.extend(block[: STREAM_PREFIX_LIMIT - len(stdout_prefix)])
                else:
                    write_all(stderr_fd, block)
                    stderr_hash.update(block)
                    stderr_bytes += len(block)
                    if len(stderr_prefix) < STREAM_PREFIX_LIMIT:
                        stderr_prefix.extend(block[: STREAM_PREFIX_LIMIT - len(stderr_prefix)])
            if process.poll() is not None and not selector.get_map():
                break
        return_code = process.wait()
        if first_disarm is None:
            first_disarm = try_disarm_candidate()
            if first_disarm is not None:
                first_disarm_time = utc_now()
        final_candidate = try_disarm_candidate()
        if first_disarm is not None and final_candidate is not None:
            if (first_disarm["st_dev"], first_disarm["st_ino"]) != (
                final_candidate["st_dev"],
                final_candidate["st_ino"],
            ):
                raise GateError("candidate inode changed after immediate disarm")
        final_sample = resource_sample(started)
        last_sample = final_sample
        write_all(monitor_fd, canonical_json(final_sample) + b"\n")
        for descriptor in (stdout_fd, stderr_fd, monitor_fd):
            os.fsync(descriptor)
        result = {
            "argv": list(argv),
            "return_code": return_code,
            "elapsed_seconds": round(time.monotonic() - started, 6),
            "stdout_bytes": stdout_bytes,
            "stdout_sha256": stdout_hash.hexdigest(),
            "stdout_prefix_utf8": bytes(stdout_prefix).decode("utf-8", "replace"),
            "stderr_bytes": stderr_bytes,
            "stderr_sha256": stderr_hash.hexdigest(),
            "stderr_prefix_utf8": bytes(stderr_prefix).decode("utf-8", "replace"),
            "timed_out": return_code in {124, 137} or deadline_hit,
            "candidate_deadline_hit": deadline_hit,
        }
        samples = [item for item in (first_sample, last_sample) if item is not None]
        return result, samples, final_candidate, first_disarm_time
    finally:
        if process is not None and process.poll() is None:
            terminate_process_group(process)
        for descriptor in (stdout_fd, stderr_fd, monitor_fd):
            try:
                os.close(descriptor)
            except OSError:
                pass


def run_build() -> int:
    assert_runtime_identity()
    verify_g1_contract(require_a_absent=False)
    start = read_receipt(BUILD_START)
    if start["status"] != "STARTED_ATTEMPT_A_GPU_BUILD":
        raise GateError("build start receipt does not admit run-build")
    if os.path.lexists(BUILD_FINAL):
        raise GateError("GPU-build final/failure receipt already exists")
    source_root, expected = source_contract()
    del source_root
    inventory, entries = inventory_input_tree(
        OUTPUT_SOURCE, expected, wrapper=True, label="run_build_input_353"
    )
    g1.validate_build_input_inventory(entries, expected.keys())
    available, _ = memory_fields()
    if available < MINIMUM_AVAILABLE_MEMORY_BYTES:
        raise GateError("MemAvailable below 4 GiB immediately before Make")
    argv = build_argv()
    execution: dict[str, Any] | None = None
    samples: list[dict[str, Any]] = []
    candidate: dict[str, Any] | None = None
    first_disarm_time: str | None = None
    status = "ATTEMPT_A_BUILD_SUPERVISOR_FAILED_NO_RETRY"
    next_stage = "G4_LOG_ONLY_STATIC_CAUSE_ANALYSIS_THEN_STOP"
    error: str | None = None
    try:
        execution, samples, candidate, first_disarm_time = run_build_capture(argv)
        if execution["candidate_deadline_hit"]:
            status = "TIMEBOX_EXHAUSTED"
            next_stage = "STOP_NO_ATTEMPT_B"
        elif execution["return_code"] != 0:
            status = "ATTEMPT_A_BUILD_FAILED_NO_RETRY"
        elif candidate is None:
            status = "ATTEMPT_A_BUILD_RETURNED_ZERO_WITHOUT_CANDIDATE"
        elif candidate["mode_octal"] != "0400" or candidate["size_bytes"] <= 0:
            status = "ATTEMPT_A_CANDIDATE_DISARM_FAILED"
        else:
            status = "ATTEMPT_A_BUILD_COMMAND_SUCCEEDED_CANDIDATE_DISARMED"
            next_stage = "UNLOAD_A_BUILD_PROFILE_VERIFY_ZERO_RESIDUE_THEN_RUN_G4_STATIC_AUDIT"
    except (GateError, OSError, ValueError, subprocess.SubprocessError) as exc:
        error = str(exc)
        if execution is None:
            execution = execution_result(
                argv,
                125,
                0.0,
                b"",
                error.encode("utf-8", "replace"),
                timed_out=False,
                deadline_hit=False,
            )
        if os.path.lexists(CANDIDATE_PATH):
            try:
                candidate = try_disarm_candidate()
            except (GateError, OSError) as disarm_exc:
                error = f"{error}; candidate_disarm={disarm_exc}"
    logs = [
        log_identity(BUILD_STDOUT, "gpu_build_stdout"),
        log_identity(BUILD_STDERR, "gpu_build_stderr"),
        log_identity(BUILD_MONITOR, "gpu_build_resource_monitor"),
    ]
    checks = [
        static_check("make_invocation_count", True, "1"),
        static_check("parallel_jobs", True, "-j1"),
        static_check(
            "candidate_immediate_disarm",
            candidate is not None and candidate.get("mode_octal") == "0400",
            first_disarm_time or error or "candidate absent",
        ),
    ]
    if error is not None:
        checks.append(static_check("build_supervisor_error", False, error))
    receipt = make_receipt(
        document_type="SMPCC_R8_LIQUID_U3_GPU_BUILD_RECEIPT",
        phase="run-build",
        status=status,
        preflight_data=start["preflight"],
        execution=execution,
        inventories=[inventory],
        resource_samples=samples,
        artifacts=[] if candidate is None else [candidate],
        logs=logs,
        static_checks=checks,
        receipt_references=references([SOURCE_COPY_FINAL, BUILD_START]),
        safety_data=safety(policy()["profiles"]["a_build"]["name"], make_count=1),
        next_allowed_stage=next_stage,
    )
    write_receipt(BUILD_FINAL, receipt)
    print(
        json.dumps(
            {
                "status": status,
                "receipt": str(BUILD_FINAL),
                "return_code": execution["return_code"],
                "elapsed_seconds": execution["elapsed_seconds"],
                "candidate": None if candidate is None else candidate["path"],
                "candidate_sha256": None if candidate is None else candidate["sha256"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0 if status == "ATTEMPT_A_BUILD_COMMAND_SUCCEEDED_CANDIDATE_DISARMED" else 3


def candidate_for_g1(identity: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "realpath": identity["realpath"],
        "parent_realpath": identity["parent_realpath"],
        "uid": identity["uid"],
        "gid": identity["gid"],
        "st_dev": identity["st_dev"],
        "st_ino": identity["st_ino"],
        "regular": identity["regular"],
        "symlink": identity["symlink"],
        "nlink": identity["nlink"],
        "size": identity["size_bytes"],
        "mode_octal": identity["mode_octal"],
        "sha256": identity["sha256"],
    }


def append_framed_output(
    descriptor: int,
    index: int,
    argv: Sequence[str],
    result: Mapping[str, Any],
    stdout: bytes,
    stderr: bytes,
) -> None:
    header = {
        "index": index,
        "argv": list(argv),
        "return_code": result["return_code"],
        "stdout_bytes": len(stdout),
        "stdout_sha256": hashlib.sha256(stdout).hexdigest(),
        "stderr_bytes": len(stderr),
        "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
    }
    write_all(descriptor, b"R8GPUAUDIT1 " + canonical_json(header) + b"\n")
    write_all(descriptor, stdout)
    write_all(descriptor, stderr)
    write_all(descriptor, b"\nR8GPUAUDIT1-END\n")


def validate_candidate_text(outputs: Mapping[str, str], candidate_sha: str) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []

    def require(identifier: str, condition: bool, evidence: str) -> None:
        checks.append(static_check(identifier, condition, evidence))
        if not condition:
            raise AuditError(f"{identifier}: {evidence}")

    file_text = outputs["file"]
    header = outputs["readelf_header"]
    programs = outputs["readelf_program_headers"]
    dynamic = outputs["readelf_dynamic"]
    list_elf = outputs["cuobjdump_list_elf"]
    dump_elf = outputs["cuobjdump_dump_elf"]
    symbols = outputs["cuobjdump_dump_elf_symbols"]
    list_ptx = outputs["cuobjdump_list_ptx"]
    dump_ptx = outputs["cuobjdump_dump_ptx"]
    sha_text = outputs["sha256sum"]

    require("candidate_file_elf64_x86_64", "ELF 64-bit" in file_text and "x86-64" in file_text, file_text)
    require("candidate_elf_type_et_dyn", re.search(r"Type:\s+DYN\b", header) is not None, header)
    require("candidate_machine_x86_64", "Advanced Micro Devices X86-64" in header, header)
    require(
        "candidate_interpreter",
        "/lib64/ld-linux-x86-64.so.2" in programs,
        programs,
    )
    stack_lines = [line for line in programs.splitlines() if "GNU_STACK" in line]
    require(
        "candidate_non_executable_gnu_stack",
        len(stack_lines) == 1 and re.search(r"\bE\b", stack_lines[0]) is None,
        "\n".join(stack_lines),
    )
    needed = re.findall(r"\(NEEDED\).*?\[([^]]+)\]", dynamic)
    required_needed = {"libgomp.so.1", "libstdc++.so.6", "libm.so.6", "libgcc_s.so.1", "libc.so.6"}
    require("candidate_needed_set", required_needed.issubset(needed), json.dumps(needed, sort_keys=True))
    require(
        "candidate_no_rpath_runpath_dynamic_cudart",
        "(RPATH)" not in dynamic
        and "(RUNPATH)" not in dynamic
        and not any("libcudart.so" in item for item in needed),
        dynamic,
    )
    native_arches = set(re.findall(r"\bsm_[0-9]+\b", list_elf))
    require("candidate_unique_native_sm120", native_arches == {"sm_120"}, list_elf)
    ptx_arches = set(re.findall(r"\bsm_[0-9]+\b", list_ptx + "\n" + dump_ptx))
    require("candidate_compute120_ptx", ptx_arches == {"sm_120"} and ".target sm_120" in dump_ptx, list_ptx + "\n" + dump_ptx)
    text_section = re.search(r"\.text\.[^\s,\"]+.*(?:PROGBITS|@progbits)", dump_elf, re.IGNORECASE)
    require("candidate_nonzero_executable_kernel_text", text_section is not None and len(dump_elf) > 256, dump_elf)
    require(
        "candidate_function_symbol_auxiliary",
        ".text." in symbols and re.search(r"FUNC|STT_FUNC|@function", symbols, re.IGNORECASE) is not None,
        symbols,
    )
    require("candidate_sandbox_sha_matches_host", candidate_sha in sha_text, sha_text)
    return checks


def validate_object_text(
    name: str,
    outputs: Mapping[str, str],
    expected_sha: str,
    *,
    cuda: bool,
) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []

    def require(identifier: str, condition: bool, evidence: str) -> None:
        checks.append(static_check(identifier, condition, evidence))
        if not condition:
            raise AuditError(f"{name}:{identifier}: {evidence[:4096]}")

    require(
        f"object:{name}:file",
        "ELF 64-bit" in outputs["object_file"] and "relocatable" in outputs["object_file"],
        outputs["object_file"],
    )
    require(
        f"object:{name}:et_rel",
        re.search(r"Type:\s+REL\b", outputs["object_readelf_header"]) is not None
        and "Advanced Micro Devices X86-64" in outputs["object_readelf_header"],
        outputs["object_readelf_header"],
    )
    require(
        f"object:{name}:sections",
        "Section Headers:" in outputs["object_readelf_sections"],
        outputs["object_readelf_sections"],
    )
    require(
        f"object:{name}:sandbox_sha",
        expected_sha in outputs["object_sha256sum"],
        outputs["object_sha256sum"],
    )
    if cuda:
        elf_arches = set(re.findall(r"\bsm_[0-9]+\b", outputs["cuda_object_list_elf"]))
        ptx_arches = set(re.findall(r"\bsm_[0-9]+\b", outputs["cuda_object_list_ptx"]))
        require(
            f"object:{name}:cuda_native_sm120",
            elf_arches == {"sm_120"},
            outputs["cuda_object_list_elf"],
        )
        require(
            f"object:{name}:cuda_ptx_sm120",
            ptx_arches == {"sm_120"},
            outputs["cuda_object_list_ptx"],
        )
    return checks


def run_static_audit() -> int:
    assert_runtime_identity()
    verify_g1_contract(require_a_absent=False)
    if any(os.path.lexists(path) for path in (STATIC_AUDIT_FINAL, STATIC_PARSER_OUTPUT, STATIC_COMMAND_SUMMARY)):
        raise GateError("static-audit create-new output already exists")
    build = read_receipt(BUILD_FINAL)
    if build["status"] != "ATTEMPT_A_BUILD_COMMAND_SUCCEEDED_CANDIDATE_DISARMED":
        raise GateError("build receipt does not admit candidate static audit")
    candidate_before = artifact_identity(CANDIDATE_PATH, kind="candidate")
    if candidate_before["mode_octal"] != "0400" or candidate_before["size_bytes"] <= 0:
        raise GateError("candidate is not disarmed 0400 with positive size")
    current = g1.read_json_object(g1.POLICY_PATH)
    object_names = current["object_contract"]["object_names"]
    cuda_names = set(current["object_contract"]["cuda_object_names"])
    objects_before = [artifact_identity(OUTPUT_SOURCE / name, kind="object") for name in object_names]
    if len(objects_before) != 131:
        raise GateError("post-build object identity count is not 131")
    for item in objects_before:
        if item["mode_octal"][-1] in "1357" or int(item["mode_octal"], 8) & 0o111:
            raise GateError(f"object execute bits are present: {item['path']}")
    parser_fd = open_new(STATIC_PARSER_OUTPUT, mode=0o640)
    summary_fd = open_new(STATIC_COMMAND_SUMMARY, mode=0o640)
    command_plan = static_command_plan()
    outputs: dict[tuple[str, str], str] = {}
    command_results: list[dict[str, Any]] = []
    total_output = 0
    audit_error: str | None = None
    checks: list[dict[str, Any]] = []
    started = time.monotonic()
    try:
        for index, (host, suffix) in enumerate(command_plan, start=1):
            if time.time() >= FINAL_DEADLINE_EPOCH:
                raise AuditError("T+8 final deadline reached during static audit")
            argv = g1.build_static_audit_argv(host, suffix, current)
            g1.validate_static_audit_argv(argv, current)
            result, stdout, stderr = run_capture_bounded(
                argv,
                output_limit=STATIC_PER_COMMAND_OUTPUT_LIMIT,
                deadline_epoch=FINAL_DEADLINE_EPOCH,
            )
            total_output += len(stdout) + len(stderr)
            if total_output > STATIC_AGGREGATE_OUTPUT_LIMIT:
                raise AuditError("aggregate parser output exceeded 256 MiB")
            append_framed_output(parser_fd, index, argv, result, stdout, stderr)
            summary = {
                "index": index,
                "host_input": host,
                "suffix_id": suffix,
                "argv": argv,
                "return_code": result["return_code"],
                "elapsed_seconds": result["elapsed_seconds"],
                "stdout_bytes": len(stdout),
                "stdout_sha256": hashlib.sha256(stdout).hexdigest(),
                "stderr_bytes": len(stderr),
                "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
            }
            write_all(summary_fd, canonical_json(summary) + b"\n")
            command_results.append(summary)
            if result["return_code"] != 0:
                raise AuditError(
                    f"static parser command {index}/{len(command_plan)} failed rc={result['return_code']} suffix={suffix} host={host} stderr={stderr[:4096]!r}"
                )
            outputs[(host, suffix)] = (stdout + b"\n" + stderr).decode("utf-8", "replace")
        os.fsync(parser_fd)
        os.fsync(summary_fd)
        candidate_outputs = {
            suffix: outputs[(str(CANDIDATE_PATH), suffix)]
            for suffix in (
                "file",
                "readelf_header",
                "readelf_program_headers",
                "readelf_dynamic",
                "readelf_notes",
                "cuobjdump_list_elf",
                "cuobjdump_dump_elf",
                "cuobjdump_dump_elf_symbols",
                "cuobjdump_list_ptx",
                "cuobjdump_dump_ptx",
                "sha256sum",
            )
        }
        checks.extend(validate_candidate_text(candidate_outputs, candidate_before["sha256"]))
        before_by_name = {Path(item["path"]).name: item for item in objects_before}
        for name in object_names:
            host = str(OUTPUT_SOURCE / name)
            suffixes = {
                suffix: outputs[(host, suffix)]
                for suffix in (
                    "object_file",
                    "object_readelf_header",
                    "object_readelf_sections",
                    "object_sha256sum",
                )
            }
            if name in cuda_names:
                suffixes["cuda_object_list_elf"] = outputs[(host, "cuda_object_list_elf")]
                suffixes["cuda_object_list_ptx"] = outputs[(host, "cuda_object_list_ptx")]
            checks.extend(
                validate_object_text(
                    name,
                    suffixes,
                    before_by_name[name]["sha256"],
                    cuda=name in cuda_names,
                )
            )
        make_argv = build["execution"]["argv"][-len(g1.MAKE_ARGV) :]
        if tuple(make_argv) != g1.MAKE_ARGV or make_argv.count(g1.GENCODE_ARG) != 1:
            raise AuditError("build receipt does not prove the unique frozen GENCODE/Make argv")
        checks.append(static_check("gencode_unique_compute120_sm120", True, g1.GENCODE_ARG))
    except (AuditError, GateError, OSError, ValueError, KeyError) as exc:
        audit_error = str(exc)
        checks.append(static_check("static_audit_error", False, audit_error))
    finally:
        for descriptor in (parser_fd, summary_fd):
            try:
                os.fsync(descriptor)
            except OSError:
                pass
            try:
                os.close(descriptor)
            except OSError:
                pass
    candidate_after = artifact_identity(CANDIDATE_PATH, kind="candidate")
    objects_after = [artifact_identity(OUTPUT_SOURCE / name, kind="object") for name in object_names]
    try:
        g1.validate_candidate_identity_unchanged(
            candidate_for_g1(candidate_before), candidate_for_g1(candidate_after), current
        )
        if objects_before != objects_after:
            raise AuditError("one or more object identities changed during static audit")
        checks.extend(
            [
                static_check("candidate_identity_unchanged", True, candidate_before["sha256"]),
                static_check("objects_131_identity_unchanged", True, canonical_hash(objects_before)),
                static_check("binary_content_parser_executed_on_host", True, "false"),
                static_check("candidate_executed", True, "false"),
                static_check("network_used", True, "false"),
                static_check("gpu_device_exposed", True, "false"),
            ]
        )
    except (GateError, AuditError) as exc:
        audit_error = audit_error or str(exc)
        checks.append(static_check("identity_postcondition", False, str(exc)))
    logs = [
        log_identity(STATIC_PARSER_OUTPUT, "static_audit_framed_parser_output"),
        log_identity(STATIC_COMMAND_SUMMARY, "static_audit_command_summary"),
    ]
    passed = audit_error is None and len(command_results) == 557 and all(item["passed"] for item in checks)
    status = "A_STATIC_AUDIT_PASS" if passed else "FAIL_GPU_BUILD_STATIC_AUDIT"
    next_stage = "G7_REQUIRES_NEW_GOAL" if passed else "STOP"
    receipt = make_receipt(
        document_type="SMPCC_R8_LIQUID_U3_GPU_STATIC_AUDIT_RECEIPT",
        phase="run-static-audit",
        status=status,
        preflight_data=build["preflight"],
        execution=execution_result(
            ["G1_EXACT_STATIC_AUDIT_ARGV_SET", "557_COMMANDS"],
            0 if passed else 3,
            time.monotonic() - started,
            b"",
            b"" if audit_error is None else audit_error.encode("utf-8", "replace"),
            timed_out="deadline" in (audit_error or "").lower(),
            deadline_hit="deadline" in (audit_error or "").lower(),
        ),
        artifacts=[candidate_after, *objects_after],
        logs=logs,
        static_checks=checks,
        receipt_references=references([BUILD_FINAL]),
        safety_data=safety(policy()["profiles"]["static_audit"]["name"], make_count=1),
        next_allowed_stage=next_stage,
    )
    write_receipt(STATIC_AUDIT_FINAL, receipt)
    print(
        json.dumps(
            {
                "G4_STATE": status,
                "receipt": str(STATIC_AUDIT_FINAL),
                "receipt_sha256": sha256_file(STATIC_AUDIT_FINAL),
                "command_count": len(command_results),
                "parser_output_bytes": total_output,
                "candidate_sha256": candidate_after["sha256"],
                "error": audit_error,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0 if passed else 4


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=COMMANDS)
    args = parser.parse_args(argv)
    try:
        if args.phase == "self-check":
            result: Any = self_check()
        elif args.phase == "preflight":
            result = preflight()
        elif args.phase == "prepare-source-copy":
            result = prepare_source_copy()
        elif args.phase == "run-source-copy":
            return run_source_copy()
        elif args.phase == "finalize-source-copy":
            result = finalize_source_copy()
        elif args.phase == "prepare-build":
            result = prepare_build()
        elif args.phase == "run-build":
            return run_build()
        elif args.phase == "run-static-audit":
            return run_static_audit()
        else:  # pragma: no cover
            raise AssertionError("unreachable phase")
    except (GateError, OSError, UnicodeError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(
            json.dumps(
                {
                    "status": "FAIL_GPU_BUILD_ADMISSION",
                    "phase": args.phase,
                    "error": str(exc),
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
