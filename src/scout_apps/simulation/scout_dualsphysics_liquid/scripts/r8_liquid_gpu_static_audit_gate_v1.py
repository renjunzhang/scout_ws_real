#!/usr/bin/env python3
"""Fail-closed RTX 5080 DualSPHysics static-audit gate.

Only the host supervisor performs path/stat/fd/hash operations.  Every tool
that interprets ELF, fatbin, cubin, PTX, or ET_REL content runs in a fresh
AppArmor-confined, networkless bwrap sandbox with no proc/dev/GPU exposure and
no writable host bind.  The candidate is never executed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import selectors
import signal
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
POLICY_PATH = PACKAGE / (
    "config/target_hosts/"
    "liquid_zrj_msi_u2404_gpu_static_audit_20260810T170339Z_v1.json"
)
SCHEMA_PATH = PACKAGE / "schema/target_host_gpu_static_audit_receipt_v1.json"
GATE_PATH = Path(__file__).resolve()

CAMPAIGN_ID = "u3_source_gpu_build_sm120_20260810T170339Z"
BUILD_ID = CAMPAIGN_ID + "_a"
PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"
PROFILE_NAME = "r8-liquid-u3-gpu-static-audit-20260810T170339Z"
PROFILE_SHA256 = "d38092826e9b17d59c20d9ef1be3b30517c5cc106e316c394877c45c72f693ed"
SCHEMA_SHA256 = "cd6d421b91759a3c1ab2af42d2dfa77095d7f1ebd4fa7eeb326bd69f4805d996"
OBJECT_COUNT = 131
CPP_COUNT = 120
CUDA_COUNT = 11
OUTPUT_LIMIT = 268435456
PREFIX_LIMIT = 65536


class AuditError(RuntimeError):
    """A static-audit ambiguity is a hard failure, never a reason to execute."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_json(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_fd(descriptor: int) -> str:
    digest = hashlib.sha256()
    os.lseek(descriptor, 0, os.SEEK_SET)
    while True:
        block = os.read(descriptor, 1024 * 1024)
        if not block:
            break
        digest.update(block)
    os.lseek(descriptor, 0, os.SEEK_SET)
    return digest.hexdigest()


def sha256_regular(path: Path, *, allow_symlink_path: bool = False) -> str:
    target = path.resolve(strict=True) if allow_symlink_path else path
    if not allow_symlink_path and path.is_symlink():
        raise AuditError(f"symlink input forbidden: {path}")
    info = target.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise AuditError(f"unsafe regular input: {path}")
    descriptor = os.open(
        target, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
    )
    try:
        return sha256_fd(descriptor)
    finally:
        os.close(descriptor)


def read_json(path: Path, *, limit: int = 64 * 1024 * 1024) -> dict[str, Any]:
    info = path.lstat()
    if (
        path.is_symlink()
        or not stat.S_ISREG(info.st_mode)
        or info.st_nlink != 1
        or info.st_size > limit
    ):
        raise AuditError(f"unsafe JSON input: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AuditError(f"JSON root is not an object: {path}")
    return value


def repo_or_absolute(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else WORKSPACE / path


def mode_octal(mode: int) -> str:
    return f"0{stat.S_IMODE(mode):03o}"


def file_identity(path: Path) -> dict[str, Any]:
    info = path.lstat()
    if path.is_symlink() or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise AuditError(f"unsafe identity file: {path}")
    return {
        "path": str(path),
        "mode_octal": mode_octal(info.st_mode),
        "size_bytes": info.st_size,
        "sha256": sha256_regular(path),
    }


def candidate_identity(path: Path) -> dict[str, Any]:
    linfo = path.lstat()
    if path.is_symlink():
        raise AuditError("candidate is a symlink")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size <= 0:
            raise AuditError("candidate is not a non-empty single-link regular file")
        digest = sha256_fd(descriptor)
    finally:
        os.close(descriptor)
    return {
        "path": str(path),
        "st_dev": info.st_dev,
        "st_ino": info.st_ino,
        "mode_octal": mode_octal(info.st_mode),
        "nlink": info.st_nlink,
        "size_bytes": info.st_size,
        "sha256": digest,
        "regular": True,
        "symlink": not stat.S_ISREG(linfo.st_mode),
    }


def require_exact_keys(value: Mapping[str, Any], expected: set[str], label: str) -> None:
    if set(value) != expected:
        raise AuditError(
            f"{label} keys drift missing={sorted(expected-set(value))} "
            f"extra={sorted(set(value)-expected)}"
        )


def policy() -> dict[str, Any]:
    return read_json(POLICY_PATH)


def validate_policy(current: Mapping[str, Any]) -> dict[str, Any]:
    require_exact_keys(
        current,
        {
            "schema_version",
            "document_type",
            "policy_id",
            "host_id",
            "development_only",
            "formal",
            "campaign_id",
            "build_id",
            "parent_inputs",
            "implementation",
            "candidate",
            "objects",
            "profile",
            "trusted_tools",
            "sandbox",
            "candidate_commands",
            "object_command",
            "acceptance",
            "outputs",
            "safety",
        },
        "policy",
    )
    expected_top = {
        "schema_version": "r8-liquid-gpu-static-audit-policy-v1",
        "document_type": "R8_LIQUID_GPU_STATIC_AUDIT_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_GPU_STATIC_AUDIT_20260810T170339Z_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "campaign_id": CAMPAIGN_ID,
        "build_id": BUILD_ID,
    }
    for key, expected in expected_top.items():
        if current.get(key) != expected:
            raise AuditError(f"policy identity drift: {key}")

    candidate = current["candidate"]
    require_exact_keys(
        candidate,
        {"path", "st_ino", "mode_octal", "nlink", "size_bytes", "sha256"},
        "candidate contract",
    )
    if (
        candidate["mode_octal"] != "0400"
        or candidate["nlink"] != 1
        or candidate["size_bytes"] != 105654136
        or candidate["sha256"] != "cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f"
    ):
        raise AuditError("candidate contract drift")
    objects = current["objects"]
    if (
        objects.get("count") != OBJECT_COUNT
        or objects.get("cpp_count") != CPP_COUNT
        or objects.get("cuda_count") != CUDA_COUNT
        or objects.get("name_contract_sha256")
        != "38023e8b24f1d1731ba3a7d03bbb5fdf2e74e5623341402d607cba046b08d7e2"
        or objects.get("build_receipt_inventory_sha256")
        != "19fcea3c6207ca00e33e177e9da6f35667dd168951bbd89c188765172fdefd9b"
    ):
        raise AuditError("131-object contract drift")

    profile = current["profile"]
    if profile != {
        "name": PROFILE_NAME,
        "path": "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T170339Z.profile",
        "mode_octal": "0600",
        "size_bytes": 61078,
        "sha256": PROFILE_SHA256,
    }:
        raise AuditError("exact static-audit profile contract drift")

    sandbox = current["sandbox"]
    if sandbox.get("timeout_kill_after_seconds") != 5:
        raise AuditError("timeout kill-after drift")
    if sandbox.get("output_limit_bytes") != OUTPUT_LIMIT or sandbox.get("prefix_limit_bytes") != PREFIX_LIMIT:
        raise AuditError("stream limit drift")
    if sandbox.get("resource_classes") != {
        "metadata": {"wall_seconds": 60, "cpu_seconds": 60, "address_space_bytes": 1073741824},
        "cuobjdump_list": {"wall_seconds": 120, "cpu_seconds": 120, "address_space_bytes": 2147483648},
        "cuobjdump_dump": {"wall_seconds": 300, "cpu_seconds": 300, "address_space_bytes": 4294967296},
    }:
        raise AuditError("static parser resource classes drift")
    if sandbox.get("invariants") != {
        "unshare_user": True,
        "unshare_pid": True,
        "unshare_net": True,
        "unshare_ipc": True,
        "unshare_uts": True,
        "proc_exposed": False,
        "dev_exposed": False,
        "gpu_exposed": False,
        "host_writable_bind_count": 0,
        "candidate_executable": False,
        "shell_allowed": False,
        "compiler_allowed": False,
        "make_allowed": False,
        "ldd_allowed": False,
        "home": "/nonexistent",
    }:
        raise AuditError("sandbox invariants drift")
    if current.get("safety") != {
        "binary_content_parser_executed_on_host": False,
        "candidate_executed": False,
        "network_used": False,
        "gpu_device_exposed": False,
        "compiler_run": False,
        "make_run": False,
        "root_parser_run": False,
    }:
        raise AuditError("safety declaration drift")

    commands = current["candidate_commands"]
    if not isinstance(commands, list) or len(commands) != 11:
        raise AuditError("candidate command catalog must contain exactly 11 commands")
    ids = [item.get("id") for item in commands if isinstance(item, dict)]
    if len(ids) != len(set(ids)) or set(ids) != {
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
    }:
        raise AuditError("candidate command identifiers drift")
    command_tokens = [token for item in commands for token in item.get("argv", [])]
    for forbidden in (
        "/bin/sh",
        "/bin/bash",
        "/usr/bin/make",
        "/usr/local/cuda-12.8/bin/nvcc",
        "/usr/bin/ldd",
        "--bind",
        "--dev-bind",
    ):
        if forbidden in command_tokens:
            raise AuditError(f"forbidden parser command token: {forbidden}")
    if any(token.startswith("/dev/") for token in command_tokens):
        raise AuditError("parser command exposes a device path")
    return dict(current)


def verify_parent_and_tool_hashes(current: Mapping[str, Any]) -> dict[str, Any]:
    parents = current["parent_inputs"]
    expected_parent_keys = {
        "plan",
        "g1_policy",
        "g1_schema",
        "g1_gate",
        "g1_tests",
        "campaign_policy",
        "campaign_gate",
        "build_receipt",
        "build_lifecycle",
        "build_stdout",
        "build_stderr",
    }
    require_exact_keys(parents, expected_parent_keys, "parent inputs")
    observed: dict[str, str] = {}
    for label, item in parents.items():
        require_exact_keys(item, {"path", "sha256"}, f"parent {label}")
        path = repo_or_absolute(item["path"])
        digest = sha256_regular(path)
        if digest != item["sha256"]:
            raise AuditError(f"parent hash drift: {label}")
        observed[label] = digest
    if observed["plan"] != PLAN_SHA256:
        raise AuditError("plan hash drift")

    implementation = current["implementation"]
    schema_entry = implementation["receipt_schema"]
    if (
        repo_or_absolute(schema_entry["path"]) != SCHEMA_PATH
        or schema_entry["sha256"] != SCHEMA_SHA256
        or sha256_regular(SCHEMA_PATH) != SCHEMA_SHA256
    ):
        raise AuditError("receipt schema identity drift")
    generator = implementation["profile_generator"]
    generator_path = repo_or_absolute(generator["path"])
    if sha256_regular(generator_path) != generator["sha256"]:
        raise AuditError("profile generator hash drift")
    profile_path = repo_or_absolute(current["profile"]["path"])
    profile_identity = file_identity(profile_path)
    if (
        profile_identity["mode_octal"] != current["profile"]["mode_octal"]
        or profile_identity["size_bytes"] != current["profile"]["size_bytes"]
        or profile_identity["sha256"] != current["profile"]["sha256"]
    ):
        raise AuditError("static-audit profile bytes/mode drift")

    tools = current["trusted_tools"]
    if not isinstance(tools, list) or len(tools) != 12:
        raise AuditError("trusted tool/data set cardinality drift")
    seen: set[str] = set()
    for item in tools:
        require_exact_keys(item, {"path", "realpath", "sha256"}, "trusted tool")
        path = Path(item["path"])
        resolved = path.resolve(strict=True)
        if str(resolved) != item["realpath"]:
            raise AuditError(f"trusted tool realpath drift: {path}")
        if sha256_regular(resolved) != item["sha256"]:
            raise AuditError(f"trusted tool hash drift: {path}")
        if item["path"] in seen:
            raise AuditError("duplicate trusted tool path")
        seen.add(item["path"])
    return {
        "parent_count": len(observed),
        "tool_count": len(seen),
        "profile_sha256": profile_identity["sha256"],
    }


def build_evidence(current: Mapping[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    receipt = read_json(Path(current["parent_inputs"]["build_receipt"]["path"]))
    lifecycle = read_json(Path(current["parent_inputs"]["build_lifecycle"]["path"]))
    if (
        receipt.get("status") != "PASS_GPU_BUILD_CANDIDATE_DISARMED"
        or receipt.get("returncode") != 0
        or receipt.get("make_count") != 1
        or receipt.get("parallel_jobs") != 1
        or receipt.get("candidate_executed") is not False
        or receipt.get("network_used") is not False
        or receipt.get("gpu_device_exposed") is not False
    ):
        raise AuditError("build receipt does not admit static audit")
    if lifecycle.get("status") != "PASS_PROFILE_LIFECYCLE_AND_GATE" or lifecycle.get("zero_residue") is not True:
        raise AuditError("build profile lifecycle is not closed")
    artifact = receipt.get("artifact")
    if not isinstance(artifact, dict) or not isinstance(artifact.get("objects"), dict):
        raise AuditError("build artifact inventory missing")
    candidate = artifact.get("candidate")
    expected_candidate = current["candidate"]
    if not isinstance(candidate, dict) or (
        candidate.get("path") != expected_candidate["path"]
        or candidate.get("inode") != expected_candidate["st_ino"]
        or candidate.get("mode_octal") != expected_candidate["mode_octal"]
        or candidate.get("size_bytes") != expected_candidate["size_bytes"]
        or candidate.get("sha256") != expected_candidate["sha256"]
    ):
        raise AuditError("candidate differs from successful build receipt")
    entries = artifact["objects"].get("entries")
    if not isinstance(entries, list) or len(entries) != OBJECT_COUNT:
        raise AuditError("build receipt object list cardinality drift")
    normalized: list[dict[str, Any]] = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise AuditError("object receipt entry is not an object")
        require_exact_keys(entry, {"name", "size_bytes", "sha256"}, "build object entry")
        normalized.append(dict(entry))
    normalized.sort(key=lambda item: item["name"])
    if sha256_bytes(canonical_json(normalized)) != current["objects"]["build_receipt_inventory_sha256"]:
        raise AuditError("build object inventory canonical hash drift")
    names = [item["name"] for item in normalized]
    if sha256_bytes(canonical_json(names)) != current["objects"]["name_contract_sha256"]:
        raise AuditError("object-name canonical contract drift")
    return receipt, normalized


def object_inventory(current: Mapping[str, Any], expected: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    root = Path(current["objects"]["root"])
    actual_names = sorted(path.name for path in root.glob("*.o"))
    expected_names = [item["name"] for item in expected]
    if actual_names != expected_names:
        raise AuditError("object root has missing or extra .o files")
    observed: list[dict[str, Any]] = []
    total = 0
    for item in expected:
        path = root / item["name"]
        if path.is_symlink():
            raise AuditError(f"object is symlink: {path}")
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            info = os.fstat(descriptor)
            if (
                not stat.S_ISREG(info.st_mode)
                or info.st_nlink != 1
                or info.st_size <= 0
                or stat.S_IMODE(info.st_mode) & 0o111
            ):
                raise AuditError(f"unsafe object metadata: {path}")
            digest = sha256_fd(descriptor)
        finally:
            os.close(descriptor)
        if info.st_size != item["size_bytes"] or digest != item["sha256"]:
            raise AuditError(f"object identity drift: {path}")
        observed.append({"name": item["name"], "size_bytes": info.st_size, "sha256": digest})
        total += info.st_size
    canonical = sha256_bytes(canonical_json(observed))
    if canonical != current["objects"]["build_receipt_inventory_sha256"]:
        raise AuditError("live object inventory hash differs from build receipt")
    return {"count": len(observed), "total_bytes": total, "canonical_sha256": canonical}


def build_sandbox_argv(
    current: Mapping[str, Any], host_input: Path, guest_input: str, resource_name: str, tool_argv: Sequence[str]
) -> list[str]:
    if not host_input.is_absolute() or not guest_input.startswith("/audit/input/"):
        raise AuditError("static-audit bind path is not exact and absolute")
    resource = current["sandbox"]["resource_classes"].get(resource_name)
    if not isinstance(resource, dict):
        raise AuditError("unknown static-audit resource class")
    argv = [
        "/usr/bin/timeout",
        "--foreground",
        "--kill-after=5s",
        f"{resource['wall_seconds']}s",
        "/usr/bin/aa-exec",
        "-p",
        PROFILE_NAME,
        "--",
        "/usr/bin/bwrap",
        "--die-with-parent",
        "--new-session",
        "--clearenv",
        "--unshare-user",
        "--unshare-pid",
        "--unshare-net",
        "--unshare-ipc",
        "--unshare-uts",
        "--disable-userns",
        "--assert-userns-disabled",
        "--ro-bind",
        "/usr",
        "/usr",
        "--symlink",
        "usr/lib",
        "/lib",
        "--symlink",
        "usr/lib64",
        "/lib64",
        "--dir",
        "/etc",
        "--ro-bind",
        "/etc/magic",
        "/etc/magic",
        "--tmpfs",
        "/audit",
        "--dir",
        "/audit/input",
        "--dir",
        "/audit/tmp",
        "--ro-bind",
        str(host_input),
        guest_input,
        "--chdir",
        "/audit",
        "/usr/bin/env",
        "-i",
        "HOME=/nonexistent",
        "PATH=/usr/bin:/usr/local/cuda-12.8/bin",
        "LC_ALL=C.UTF-8",
        "LANG=C.UTF-8",
        "TZ=UTC",
        "TMPDIR=/audit/tmp",
        "TMP=/audit/tmp",
        "TEMP=/audit/tmp",
        "/usr/bin/prlimit",
        f"--cpu={resource['cpu_seconds']}",
        f"--as={resource['address_space_bytes']}",
        "--nproc=64",
        "--nofile=128",
        "--core=0",
        "--",
        *tool_argv,
    ]
    validate_sandbox_argv(argv, host_input, guest_input, tool_argv)
    return argv


def validate_sandbox_argv(
    argv: Sequence[str], host_input: Path, guest_input: str, tool_argv: Sequence[str]
) -> None:
    mandatory = {
        "--die-with-parent",
        "--new-session",
        "--clearenv",
        "--unshare-user",
        "--unshare-pid",
        "--unshare-net",
        "--unshare-ipc",
        "--unshare-uts",
        "--disable-userns",
        "--assert-userns-disabled",
        "--kill-after=5s",
    }
    if not mandatory.issubset(argv):
        raise AuditError("sandbox argv lacks mandatory isolation")
    if any(token in argv for token in ("--bind", "--proc", "--dev", "--dev-bind")):
        raise AuditError("sandbox argv exposes writable bind/proc/dev")
    ro_pairs = [
        list(argv[index + 1 : index + 3])
        for index, token in enumerate(argv[:-2])
        if token == "--ro-bind"
    ]
    if ro_pairs != [
        ["/usr", "/usr"],
        ["/etc/magic", "/etc/magic"],
        [str(host_input), guest_input],
    ]:
        raise AuditError("sandbox read-only bind set drift")
    if list(argv[-len(tool_argv) :]) != list(tool_argv):
        raise AuditError("sandbox tool suffix drift")
    if any(
        forbidden in argv
        for forbidden in ("/bin/sh", "/bin/bash", "/usr/bin/make", "/usr/local/cuda-12.8/bin/nvcc", "/usr/bin/ldd")
    ):
        raise AuditError("sandbox contains a forbidden executable")


def _write_all(descriptor: int, data: bytes) -> None:
    offset = 0
    while offset < len(data):
        written = os.write(descriptor, data[offset:])
        if written <= 0:
            raise AuditError("short output write")
        offset += written


def _open_new(path: Path, mode: int = 0o640) -> int:
    return os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        mode,
    )


def run_command(argv: Sequence[str], evidence_root: Path, index: int, command_id: str, host_input: Path) -> dict[str, Any]:
    safe_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", command_id)
    stdout_path = evidence_root / f"{index:03d}_{safe_id}.stdout.log"
    stderr_path = evidence_root / f"{index:03d}_{safe_id}.stderr.log"
    stdout_fd = _open_new(stdout_path)
    stderr_fd = _open_new(stderr_path)
    started = time.monotonic()
    process: subprocess.Popen[bytes] | None = None
    summaries: dict[str, dict[str, Any]] = {
        "stdout": {"path": str(stdout_path), "size_bytes": 0, "digest": hashlib.sha256(), "prefix": bytearray()},
        "stderr": {"path": str(stderr_path), "size_bytes": 0, "digest": hashlib.sha256(), "prefix": bytearray()},
    }
    descriptors = {"stdout": stdout_fd, "stderr": stderr_fd}
    output_limit_exceeded = False
    try:
        process = subprocess.Popen(
            argv,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            cwd=WORKSPACE,
        )
        if process.stdout is None or process.stderr is None:
            raise AuditError("parser pipes were not created")
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        selector.register(process.stderr, selectors.EVENT_READ, "stderr")
        while selector.get_map():
            for key, _ in selector.select(timeout=1.0):
                label = key.data
                chunk = os.read(key.fileobj.fileno(), 1024 * 1024)
                if not chunk:
                    selector.unregister(key.fileobj)
                    key.fileobj.close()
                    continue
                summary = summaries[label]
                combined = summaries["stdout"]["size_bytes"] + summaries["stderr"]["size_bytes"]
                allowed = max(0, OUTPUT_LIMIT - combined)
                accepted = chunk[:allowed]
                if accepted:
                    _write_all(descriptors[label], accepted)
                    summary["digest"].update(accepted)
                    summary["size_bytes"] += len(accepted)
                    remaining_prefix = PREFIX_LIMIT - len(summary["prefix"])
                    if remaining_prefix > 0:
                        summary["prefix"].extend(accepted[:remaining_prefix])
                if len(accepted) != len(chunk):
                    output_limit_exceeded = True
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        returncode = process.wait(timeout=10)
    finally:
        for descriptor in descriptors.values():
            try:
                os.fsync(descriptor)
                os.fchmod(descriptor, 0o640)
            finally:
                os.close(descriptor)
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=10)
    result_streams: dict[str, dict[str, Any]] = {}
    for label, summary in summaries.items():
        result_streams[label] = {
            "path": summary["path"],
            "size_bytes": summary["size_bytes"],
            "sha256": summary["digest"].hexdigest(),
            "prefix_utf8": bytes(summary["prefix"]).decode("utf-8", "replace"),
        }
    return {
        "id": command_id,
        "input": str(host_input),
        "argv": list(argv),
        "returncode": returncode,
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "stdout": result_streams["stdout"],
        "stderr": result_streams["stderr"],
        "timed_out": returncode in (124, 137),
        "output_limit_exceeded": output_limit_exceeded,
    }


def output_text(result: Mapping[str, Any], label: str = "stdout") -> str:
    stream = result[label]
    path = Path(stream["path"])
    if sha256_regular(path) != stream["sha256"] or path.lstat().st_size != stream["size_bytes"]:
        raise AuditError(f"captured {label} stream identity drift: {result['id']}")
    return path.read_text(encoding="utf-8", errors="replace")


def parse_readelf_field(text: str, label: str) -> str:
    match = re.search(rf"^\s*{re.escape(label)}:\s*(.+?)\s*$", text, re.MULTILINE)
    if match is None:
        raise AuditError(f"readelf header lacks {label}")
    return match.group(1)


def parse_elf(results: Mapping[str, Mapping[str, Any]], current: Mapping[str, Any]) -> dict[str, Any]:
    file_text = output_text(results["file"])
    if "ELF 64-bit LSB pie executable, x86-64" not in file_text:
        raise AuditError("file output does not identify an x86-64 PIE")
    header = output_text(results["readelf_header"])
    elf_class = parse_readelf_field(header, "Class")
    data = parse_readelf_field(header, "Data")
    elf_type = parse_readelf_field(header, "Type")
    machine = parse_readelf_field(header, "Machine")
    entry_text = parse_readelf_field(header, "Entry point address")
    try:
        entry = int(entry_text, 16)
    except ValueError as exc:
        raise AuditError("invalid ELF entry point") from exc

    program = output_text(results["readelf_program_headers"])
    interpreter_match = re.search(r"Requesting program interpreter:\s*([^\]]+)\]", program)
    if interpreter_match is None:
        raise AuditError("ELF interpreter is absent")
    interpreter = interpreter_match.group(1).strip()
    executable_loads: list[tuple[int, int]] = []
    gnu_stack_executable: bool | None = None
    row_re = re.compile(
        r"^\s*(LOAD|GNU_STACK)\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s+"
        r"0x[0-9a-f]+\s+0x[0-9a-f]+\s+0x([0-9a-f]+)\s+([RWE ]+)\s+0x[0-9a-f]+\s*$",
        re.MULTILINE | re.IGNORECASE,
    )
    for match in row_re.finditer(program):
        kind, vaddr_hex, memsz_hex, flags = match.groups()
        if kind.upper() == "LOAD" and "E" in flags:
            vaddr = int(vaddr_hex, 16)
            executable_loads.append((vaddr, vaddr + int(memsz_hex, 16)))
        elif kind.upper() == "GNU_STACK":
            gnu_stack_executable = "E" in flags
    entry_in_executable_load = any(start <= entry < end for start, end in executable_loads)
    if gnu_stack_executable is None:
        raise AuditError("GNU_STACK program header is absent")

    dynamic = output_text(results["readelf_dynamic"])
    needed = re.findall(r"\(NEEDED\)\s+Shared library:\s*\[([^\]]+)\]", dynamic)
    flags_match = re.search(r"\(FLAGS_1\)\s+Flags:\s*(.+?)\s*$", dynamic, re.MULTILINE)
    flags_1 = flags_match.group(1).split() if flags_match else []
    rpath_match = re.search(r"\(RPATH\)\s+.*?\[([^\]]*)\]", dynamic)
    runpath_match = re.search(r"\(RUNPATH\)\s+.*?\[([^\]]*)\]", dynamic)

    acceptance = current["acceptance"]
    observed = {
        "class": elf_class,
        "data": data,
        "type": elf_type,
        "machine": machine,
        "interpreter": interpreter,
        "entry_point_hex": hex(entry),
        "entry_in_executable_load": entry_in_executable_load,
        "gnu_stack_executable": gnu_stack_executable,
        "flags_1": flags_1,
        "needed": needed,
        "rpath": rpath_match.group(1) if rpath_match else None,
        "runpath": runpath_match.group(1) if runpath_match else None,
        "loader_dependency_revision": {
            "parent_allowlist_count": len(acceptance["parent_needed_allowlist"]),
            "observed_count": len(needed),
            **acceptance["loader_dependency_revision"],
        },
    }
    expected = {
        "class": acceptance["elf_class"],
        "data": acceptance["elf_data"],
        "type": acceptance["elf_type"],
        "machine": acceptance["machine"],
        "interpreter": acceptance["interpreter"],
        "entry_in_executable_load": True,
        "gnu_stack_executable": acceptance["gnu_stack_executable"],
        "flags_1": acceptance["flags_1"],
        "needed": acceptance["revised_needed_allowlist"],
        "rpath": acceptance["rpath"],
        "runpath": acceptance["runpath"],
    }
    for key, value in expected.items():
        if observed[key] != value:
            raise AuditError(f"ELF acceptance mismatch: {key} observed={observed[key]!r}")
    if "libcudart.so" in needed:
        raise AuditError("dynamic libcudart is forbidden")
    return observed


def parse_cuda(results: Mapping[str, Mapping[str, Any]], current: Mapping[str, Any]) -> dict[str, Any]:
    list_elf = output_text(results["cuobjdump_list_elf"])
    cubins = re.findall(r"^ELF file\s+\d+:\s+(\S+\.sm_(\d+)\.cubin)\s*$", list_elf, re.MULTILINE)
    list_ptx = output_text(results["cuobjdump_list_ptx"])
    ptx = re.findall(r"^PTX file\s+\d+:\s+(\S+\.sm_(\d+)\.ptx)\s*$", list_ptx, re.MULTILINE)
    native_arches = sorted({f"sm_{arch}" for _, arch in cubins})
    ptx_arches = sorted({f"sm_{arch}" for _, arch in ptx})
    if len(cubins) != 11 or len(ptx) != 11:
        raise AuditError(f"fatbin cardinality drift cubin={len(cubins)} ptx={len(ptx)}")
    if native_arches != current["acceptance"]["native_arches"] or ptx_arches != current["acceptance"]["ptx_arches"]:
        raise AuditError("fatbin contains a non-sm_120 architecture")

    dump_elf = output_text(results["cuobjdump_dump_elf"])
    declared_elf_arches = sorted(set(re.findall(r"^arch = (sm_\d+)\s*$", dump_elf, re.MULTILINE)))
    text_count = 0
    for line in dump_elf.splitlines():
        tokens = line.split()
        if len(tokens) >= 10 and tokens[5] == "PROGBITS" and tokens[-1].startswith(".text."):
            try:
                size = int(tokens[2], 16)
                flags = int(tokens[6], 16)
            except ValueError:
                continue
            if size > 0 and flags & 0x4:
                text_count += 1
    if declared_elf_arches != ["sm_120"] or text_count < 1:
        raise AuditError("native cubin dump lacks sm_120 executable nonzero .text.*")

    symbols = output_text(results["cuobjdump_dump_elf_symbols"])
    function_symbol_count = sum(
        1 for line in symbols.splitlines() if "STT_FUNC" in line and "STO_ENTRY" in line
    )
    if function_symbol_count < 1:
        raise AuditError("native cubin dump lacks kernel function symbols")

    dump_ptx = output_text(results["cuobjdump_dump_ptx"])
    dump_arches = sorted(set(re.findall(r"^arch = (sm_\d+)\s*$", dump_ptx, re.MULTILINE)))
    targets = sorted(set(re.findall(r"^\.target\s+(sm_\d+)\s*$", dump_ptx, re.MULTILINE)))
    if dump_arches != ["sm_120"] or targets != ["sm_120"] or ".visible .entry " not in dump_ptx:
        raise AuditError("PTX dump lacks the paired sm_120 target/kernel evidence")

    build_stdout = Path(current["parent_inputs"]["build_stdout"]["path"]).read_text(
        encoding="utf-8", errors="replace"
    )
    nvcc_lines = [line for line in build_stdout.splitlines() if line.startswith("/usr/local/cuda-12.8/bin/nvcc ")]
    exact_gencode = current["acceptance"]["exact_gencode"]
    exact = all(line.split().count(exact_gencode) == 1 for line in nvcc_lines)
    if len(nvcc_lines) != current["acceptance"]["nvcc_command_count"] or not exact:
        raise AuditError("NVCC log does not prove 11 exact compute_120/sm_120 translations")
    return {
        "native_cubin_count": len(cubins),
        "native_arches": native_arches,
        "ptx_count": len(ptx),
        "ptx_arches": ptx_arches,
        "nonzero_executable_text_section_count": text_count,
        "function_symbol_count": function_symbol_count,
        "gencode_proof": {
            "nvcc_command_count": len(nvcc_lines),
            "all_use_cuda_12_8": all(line.startswith("/usr/local/cuda-12.8/bin/nvcc ") for line in nvcc_lines),
            "all_use_exact_gencode": exact,
            "compute_120_proven": bool(ptx) and exact,
        },
    }


def parse_objects(
    object_results: Sequence[Mapping[str, Any]], expected: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    if len(object_results) != OBJECT_COUNT:
        raise AuditError("not all 131 objects were sandbox-audited")
    by_name = {Path(result["input"]).name: result for result in object_results}
    entries: list[dict[str, Any]] = []
    for item in expected:
        result = by_name.get(item["name"])
        if result is None:
            raise AuditError(f"missing object audit: {item['name']}")
        text = output_text(result)
        elf_type = parse_readelf_field(text, "Type")
        machine = parse_readelf_field(text, "Machine")
        if elf_type != "REL (Relocatable file)" or machine != "Advanced Micro Devices X86-64":
            raise AuditError(f"object is not x86-64 ET_REL: {item['name']}")
        entries.append({
            "name": item["name"],
            "size_bytes": item["size_bytes"],
            "sha256": item["sha256"],
            "elf_type": elf_type,
            "machine": machine,
        })
    return {
        "count": len(entries),
        "cpp_count": CPP_COUNT,
        "cuda_count": CUDA_COUNT,
        "all_regular": True,
        "all_et_rel": True,
        "all_x86_64": True,
        "entries": entries,
    }


def write_json_new(path: Path, value: Mapping[str, Any]) -> None:
    payload = json.dumps(value, sort_keys=True, indent=2, ensure_ascii=False).encode("utf-8") + b"\n"
    descriptor = _open_new(path)
    try:
        _write_all(descriptor, payload)
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o640)
    finally:
        os.close(descriptor)


def receipt_identities(current: Mapping[str, Any]) -> dict[str, Any]:
    parent = current["parent_inputs"]
    implementation = current["implementation"]
    return {
        "plan": file_identity(repo_or_absolute(parent["plan"]["path"])),
        "policy": file_identity(POLICY_PATH),
        "schema": file_identity(SCHEMA_PATH),
        "gate": file_identity(GATE_PATH),
        "profile": file_identity(repo_or_absolute(current["profile"]["path"])),
        "profile_generator": file_identity(repo_or_absolute(implementation["profile_generator"]["path"])),
        "build_receipt": file_identity(Path(parent["build_receipt"]["path"])),
        "build_lifecycle": file_identity(Path(parent["build_lifecycle"]["path"])),
    }


def self_check() -> dict[str, Any]:
    current = validate_policy(policy())
    parents = verify_parent_and_tool_hashes(current)
    build, entries = build_evidence(current)
    before_candidate = candidate_identity(Path(current["candidate"]["path"]))
    if (
        before_candidate["st_ino"] != current["candidate"]["st_ino"]
        or before_candidate["mode_octal"] != "0400"
        or before_candidate["size_bytes"] != current["candidate"]["size_bytes"]
        or before_candidate["sha256"] != current["candidate"]["sha256"]
    ):
        raise AuditError("live candidate identity drift")
    inventory = object_inventory(current, entries)
    # Build one candidate and one object argv to prove the closed construction
    # without invoking any parser or creating output.
    candidate_command = current["candidate_commands"][0]
    candidate_argv = build_sandbox_argv(
        current,
        Path(current["candidate"]["path"]),
        "/audit/input/DualSPHysics5.4_linux64",
        candidate_command["resource"],
        candidate_command["argv"],
    )
    object_name = entries[0]["name"]
    object_tool = [
        token.replace("<OBJECT_NAME>", object_name)
        for token in current["object_command"]["argv_template"]
    ]
    object_argv = build_sandbox_argv(
        current,
        Path(current["objects"]["root"]) / object_name,
        f"/audit/input/{object_name}",
        current["object_command"]["resource"],
        object_tool,
    )
    return {
        "status": "PASS_GPU_STATIC_AUDIT_CONTRACT_SELF_CHECK",
        "parents": parents,
        "build_status": build["status"],
        "candidate": before_candidate,
        "object_inventory": inventory,
        "candidate_argv_length": len(candidate_argv),
        "object_argv_length": len(object_argv),
        "planned_command_count": 11 + len(entries),
        "candidate_executed": False,
        "binary_content_parser_executed_on_host": False,
    }


def preflight() -> dict[str, Any]:
    result = self_check()
    current = policy()
    for output in current["outputs"].values():
        if os.path.lexists(output):
            raise AuditError(f"create-new static-audit output already exists: {output}")
    if os.getuid() != 1000 or os.geteuid() != 1000 or os.getgid() != 1000 or os.getegid() != 1000:
        raise AuditError("static-audit gate must run as uid/gid 1000")
    if os.getgroups():
        raise AuditError(f"static-audit gate requires zero supplementary groups: {os.getgroups()}")
    if os.environ.get("R8_EXACT_PROFILE_PRELOADED_SHA256") != PROFILE_SHA256:
        raise AuditError("exact preloaded-profile token missing or drifted")
    result["status"] = "PASS_GPU_STATIC_AUDIT_EXECUTION_PREFLIGHT"
    return result


def run_audit() -> dict[str, Any]:
    preflight_result = preflight()
    current = policy()
    build, expected_objects = build_evidence(current)
    evidence_root = Path(current["outputs"]["evidence_root"])
    os.mkdir(evidence_root, 0o700)
    os.chmod(evidence_root, 0o700)
    started_at = utc_now()
    candidate_before = candidate_identity(Path(current["candidate"]["path"]))
    inventory_before = object_inventory(current, expected_objects)
    commands: list[dict[str, Any]] = []
    candidate_results: dict[str, dict[str, Any]] = {}

    for item in current["candidate_commands"]:
        argv = build_sandbox_argv(
            current,
            Path(current["candidate"]["path"]),
            "/audit/input/DualSPHysics5.4_linux64",
            item["resource"],
            item["argv"],
        )
        result = run_command(
            argv,
            evidence_root,
            len(commands),
            item["id"],
            Path(current["candidate"]["path"]),
        )
        commands.append(result)
        if result["returncode"] != 0 or result["timed_out"] or result["output_limit_exceeded"]:
            raise AuditError(f"candidate sandbox parser failed: {item['id']} rc={result['returncode']}")
        candidate_results[item["id"]] = result

    object_results: list[dict[str, Any]] = []
    for item in expected_objects:
        name = item["name"]
        host = Path(current["objects"]["root"]) / name
        guest = f"/audit/input/{name}"
        tool_argv = [
            token.replace("<OBJECT_NAME>", name)
            for token in current["object_command"]["argv_template"]
        ]
        argv = build_sandbox_argv(
            current, host, guest, current["object_command"]["resource"], tool_argv
        )
        result = run_command(
            argv, evidence_root, len(commands), f"object_readelf_header_{name}", host
        )
        commands.append(result)
        object_results.append(result)
        if result["returncode"] != 0 or result["timed_out"] or result["output_limit_exceeded"]:
            raise AuditError(f"object sandbox parser failed: {name} rc={result['returncode']}")

    elf = parse_elf(candidate_results, current)
    cuda = parse_cuda(candidate_results, current)
    objects = parse_objects(object_results, expected_objects)
    sandbox_hash = output_text(candidate_results["sha256sum"]).split()[0]
    if sandbox_hash != current["candidate"]["sha256"]:
        raise AuditError("sandbox candidate SHA-256 differs from host fd hash")

    candidate_after = candidate_identity(Path(current["candidate"]["path"]))
    inventory_after = object_inventory(current, expected_objects)
    if candidate_before != candidate_after or inventory_before != inventory_after:
        raise AuditError("candidate/object identities changed across static audit")

    receipt = {
        "schema_version": "r8-liquid-gpu-static-audit-receipt-v1",
        "document_type": "R8_LIQUID_GPU_STATIC_AUDIT_RECEIPT",
        "campaign_id": CAMPAIGN_ID,
        "build_id": BUILD_ID,
        "status": "PASS_GPU_BUILD_SM120_STATIC_AUDIT",
        "started_at_utc": started_at,
        "completed_at_utc": utc_now(),
        "identities": receipt_identities(current),
        "candidate_before": candidate_before,
        "candidate_after": candidate_after,
        "object_inventory_before": inventory_before,
        "object_inventory_after": inventory_after,
        "commands": commands,
        "elf": elf,
        "cuda": cuda,
        "objects": objects,
        "build_provenance": {
            "make_returncode": build["returncode"],
            "make_count": build["make_count"],
            "parallel_jobs": build["parallel_jobs"],
            "object_count": build["artifact"]["objects"]["count"],
            "object_inventory_sha256": build["artifact"]["objects"]["inventory_sha256"],
            "candidate_disarmed": candidate_before["mode_octal"] == "0400",
            "build_profile_zero_residue": True,
        },
        "safety": {
            "binary_content_parser_executed_on_host": False,
            "candidate_executed": False,
            "network_used": False,
            "gpu_device_exposed": False,
            "proc_exposed": False,
            "dev_exposed": False,
            "host_writable_bind_count": 0,
            "compiler_run": False,
            "make_run": False,
            "root_parser_run": False,
        },
        "next_allowed_stage": "SEPARATE_GPU_RUNTIME_EXECUTION_ADMISSION_REQUIRED",
    }
    schema = read_json(SCHEMA_PATH)
    errors = sorted(Draft202012Validator(schema).iter_errors(receipt), key=lambda error: list(error.path))
    if errors:
        rendered = "; ".join(f"/{'/'.join(map(str, error.path))}: {error.message}" for error in errors[:20])
        raise AuditError(f"closed receipt schema rejected result: {rendered}")
    write_json_new(Path(current["outputs"]["receipt"]), receipt)
    return {
        "status": receipt["status"],
        "receipt": current["outputs"]["receipt"],
        "receipt_sha256": sha256_regular(Path(current["outputs"]["receipt"])),
        "evidence_root": str(evidence_root),
        "command_count": len(commands),
        "candidate_sha256": candidate_after["sha256"],
        "object_inventory_sha256": inventory_after["canonical_sha256"],
        "native_cubin_count": cuda["native_cubin_count"],
        "ptx_count": cuda["ptx_count"],
        "candidate_executed": False,
        "binary_content_parser_executed_on_host": False,
        "preflight": preflight_result["status"],
    }


def validate_receipt(path: Path) -> dict[str, Any]:
    value = read_json(path, limit=32 * 1024 * 1024)
    schema = read_json(SCHEMA_PATH)
    errors = sorted(Draft202012Validator(schema).iter_errors(value), key=lambda error: list(error.path))
    if errors:
        rendered = "; ".join(f"/{'/'.join(map(str, error.path))}: {error.message}" for error in errors[:20])
        raise AuditError(f"receipt schema validation failed: {rendered}")
    if value["candidate_before"] != value["candidate_after"]:
        raise AuditError("receipt candidate before/after identity mismatch")
    if value["object_inventory_before"] != value["object_inventory_after"]:
        raise AuditError("receipt object before/after inventory mismatch")
    return {
        "status": value["status"],
        "receipt": str(path),
        "sha256": sha256_regular(path),
        "command_count": len(value["commands"]),
        "candidate_executed": value["safety"]["candidate_executed"],
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "preflight", "run", "validate-receipt"))
    parser.add_argument("receipt", nargs="?")
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            if args.receipt is not None:
                raise AuditError("self-check takes no receipt")
            result = self_check()
        elif args.command == "preflight":
            if args.receipt is not None:
                raise AuditError("preflight takes no receipt")
            result = preflight()
        elif args.command == "run":
            if args.receipt is not None:
                raise AuditError("run takes no receipt")
            result = run_audit()
        else:
            if args.receipt is None:
                raise AuditError("validate-receipt requires an exact path")
            result = validate_receipt(Path(args.receipt))
    except (
        AuditError,
        OSError,
        UnicodeError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
        subprocess.SubprocessError,
    ) as exc:
        print(
            json.dumps(
                {"status": "FAIL_GPU_BUILD_STATIC_AUDIT", "phase": args.command, "error": str(exc)},
                sort_keys=True,
                ensure_ascii=False,
            ),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, sort_keys=True, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
