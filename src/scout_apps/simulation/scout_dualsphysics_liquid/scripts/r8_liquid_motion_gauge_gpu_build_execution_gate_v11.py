#!/usr/bin/env python3
"""Default-deny v11 execution gate for one fresh motion-Gauge GPU build.

The public CLI performs static validation only unless the root-only v11
lifecycle supervisor supplies both the exact authorization token and one exact
phase.  The candidate is never executed and no GPU device is exposed here.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import selectors
import signal
import stat
import subprocess
import sys
import time
from pathlib import Path
from types import ModuleType
from typing import Any, Callable, Iterable, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
WORKSPACE = ROOT.parents[3]
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v11.json"
POLICY_SCHEMA_PATH = ROOT / "schema/target_host_motion_gauge_gpu_build_execution_policy_v11.json"
RECEIPT_SCHEMA_PATH = ROOT / "schema/target_host_motion_gauge_gpu_build_execution_receipt_v11.json"
TOKEN_SCHEMA_PATH = ROOT / "schema/target_host_motion_gauge_gpu_build_authorization_token_v11.json"
G1_GATE_PATH = ROOT / "scripts/r8_liquid_target_u3_gpu_build_gate_v1.py"
G1_GATE_SHA256 = "e34ab0facc3c665c6befbb792c7344e0af6e652fd3c6b30dfa81ca8cbea5b502"
G2_GATE_PATH = ROOT / "scripts/r8_liquid_target_u3_gpu_build_gate_v2.py"
G2_GATE_SHA256 = "d9d02b2aa6a237ed49b7e64cbf0f19d769a32f22c372dbc56b18be47e376eb7f"
PATCH_CHILD_PATH = ROOT / "scripts/r8_liquid_motion_gauge_gpu_patch_child_v11.py"
PATCH_V1_PATH = ROOT / "scripts/r8_liquid_motion_attached_gauge_patch_gate_v1.py"
PATCH_V1_SHA256 = "601682672f39ccd19533149024e7531ec54157bd058368f7f40e63a6f894f3cf"
PROFILE_GENERATOR_PATH = ROOT / "scripts/r8_liquid_motion_gauge_gpu_profile_generator_v11.py"

CAMPAIGN_ID = "motion_gauge_gpu_build_sm120_20260812T073037Z_v11"
BUILD_ID = CAMPAIGN_ID + "_a"
ATTEMPT_ROOT = Path("/home/zrj/scout_liquid_lab/build") / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
SOURCE_ROOT = OUTPUT_ROOT / "buildtree/src/source"
ARTIFACTS_ROOT = OUTPUT_ROOT / "artifacts"
CANDIDATE_PATH = ARTIFACTS_ROOT / "DualSPHysics5.4_linux64"
AUDIT_ROOT = Path("/home/zrj/scout_liquid_lab/audits") / f"{BUILD_ID}.evidence"
TOKEN_PATH = Path("/home/zrj/scout_liquid_lab/audits") / f"{BUILD_ID}.authorization.json"
PHASES = ("source-copy", "patch", "wrapper", "build", "static-audit")
PROFILE_ROLES = {
    "source-copy": "SOURCE_COPY", "patch": "PATCH", "wrapper": "NONE",
    "build": "BUILD", "static-audit": "STATIC_AUDIT",
}
ZERO_SHA256 = hashlib.sha256(b"").hexdigest()
WRAPPER_BYTES = (
    b".SUFFIXES:\n.SUFFIXES: .cpp .o\ninclude Makefile\n"
    b"override CCFLAGS += -include cstdint\n"
)
WRAPPER_SHA256 = "33883dcdde741c253a09d78decb3c21c5ab92c7c9152acf793d7dd025f07300d"
PATCH_NAMES = (
    "JDsGaugeItem.cpp", "JDsGaugeItem.h", "JDsGaugeSystem.cpp",
    "JDsGaugeSystem.h", "JSph.cpp", "JSph.h",
)
PATCH_AFTER_SHA256 = {
    "JDsGaugeItem.cpp": "283793b308f07579affdf428cf267f62f9ab033c75d25cb54e4620f7d2bf3d01",
    "JDsGaugeItem.h": "8ff282de107eeb50691cda2ea77cbf6ec507e73f964231f446e4711a2c46ae2e",
    "JDsGaugeSystem.cpp": "3dec0381b413f9f736242036abc4b60599dcaf724edddc6202df17de4079ad5e",
    "JDsGaugeSystem.h": "4d97c40b14105f9c4e0df372bff9229b6c8daf881686c4d50f64a7fc0a5e6c6c",
    "JSph.cpp": "07e197ff8661d91a6e9b30053dfc0b8b138b6293892c6dc72aa2c0031b63676f",
    "JSph.h": "d82074fd35998a96585c6698b09cb8bbddce79835fed4d3e07683270633b08fb",
}
PATCH_AFTER_SIZE_BYTES = {
    "JDsGaugeItem.cpp": 81766,
    "JDsGaugeItem.h": 26940,
    "JDsGaugeSystem.cpp": 32902,
    "JDsGaugeSystem.h": 8110,
    "JSph.cpp": 177907,
    "JSph.h": 35634,
}


class GateError(RuntimeError):
    """A frozen identity, state transition, or safety invariant differs."""


class StaticAuditError(GateError):
    """A confined parser output failed the mature semantic validators."""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def write_all(descriptor: int, value: bytes) -> None:
    offset = 0
    while offset < len(value):
        count = os.write(descriptor, value[offset:])
        if count <= 0:
            raise GateError("short create-new write")
        offset += count


def assert_no_symlink_components(path: Path, *, leaf_may_be_absent: bool = False) -> None:
    current = Path("/")
    parts = path.absolute().parts[1:]
    for index, component in enumerate(parts):
        current /= component
        try:
            info = os.lstat(current)
        except FileNotFoundError:
            if leaf_may_be_absent and index == len(parts) - 1:
                return
            raise GateError(f"required path component absent: {current}")
        if stat.S_ISLNK(info.st_mode):
            raise GateError(f"symlink path component forbidden: {current}")


def file_identity(path: Path, *, limit: int = 2 * 1024 * 1024 * 1024) -> dict[str, Any]:
    info = os.lstat(path)
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise GateError(f"unsafe regular file: {path}")
    if info.st_size > limit:
        raise GateError(f"file exceeds identity bound: {path}")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        digest = hashlib.sha256()
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                raise GateError(f"short identity read: {path}")
            digest.update(block)
            remaining -= len(block)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    stable = ("st_dev", "st_ino", "st_mode", "st_nlink", "st_size", "st_mtime_ns")
    if any(getattr(before, key) != getattr(after, key) for key in stable):
        raise GateError(f"file changed while hashing: {path}")
    return {
        "path": str(path), "mode_octal": format(stat.S_IMODE(after.st_mode), "04o"),
        "size_bytes": after.st_size, "sha256": digest.hexdigest(),
        "device": after.st_dev, "inode": after.st_ino, "nlink": after.st_nlink,
    }


def open_new(path: Path, *, mode: int) -> int:
    assert_no_symlink_components(path.parent)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                         os.O_NOFOLLOW | os.O_CLOEXEC, mode)
    os.fchmod(descriptor, mode)
    return descriptor


def write_new(path: Path, value: bytes, *, mode: int) -> dict[str, Any]:
    descriptor = open_new(path, mode=mode)
    try:
        write_all(descriptor, value)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    parent = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(parent)
    finally:
        os.close(parent)
    return file_identity(path)


def read_json(path: Path, *, limit: int = 16 * 1024 * 1024) -> dict[str, Any]:
    identity = file_identity(path, limit=limit)
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        raw = bytearray()
        remaining = identity["size_bytes"]
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                raise GateError(f"short JSON read: {path}")
            raw.extend(block)
            remaining -= len(block)
    finally:
        os.close(descriptor)
    value = json.loads(bytes(raw).decode("utf-8"),
                       parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    if not isinstance(value, dict):
        raise GateError(f"JSON root is not an object: {path}")
    return value


def assert_deep_closed(node: Any, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object":
            properties = node.get("properties")
            required = node.get("required")
            if node.get("additionalProperties") is not False:
                raise GateError(f"open schema object: {location}")
            if not isinstance(properties, dict) or set(required or ()) != set(properties):
                raise GateError(f"schema required/properties drift: {location}")
        for key, value in node.items():
            assert_deep_closed(value, f"{location}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            assert_deep_closed(value, f"{location}/{index}")


def load_module(path: Path, expected_sha: str, name: str) -> ModuleType:
    if file_identity(path, limit=8 * 1024 * 1024)["sha256"] != expected_sha:
        raise GateError(f"byte-pinned module drift: {path}")
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise GateError(f"cannot load byte-pinned module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def schemas() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    values = tuple(read_json(path) for path in
                   (POLICY_SCHEMA_PATH, RECEIPT_SCHEMA_PATH, TOKEN_SCHEMA_PATH))
    for value in values:
        Draft202012Validator.check_schema(value)
        assert_deep_closed(value)
    return values  # type: ignore[return-value]


def policy() -> tuple[dict[str, Any], str]:
    policy_schema, _, _ = schemas()
    value = read_json(POLICY_PATH)
    Draft202012Validator(policy_schema).validate(value)
    return value, file_identity(POLICY_PATH)["sha256"]


def _resolve_pinned(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    if ".." in path.parts:
        raise GateError(f"unsafe relative pin: {path_text}")
    return WORKSPACE / path


def verify_pins(items: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    observed: list[dict[str, Any]] = []
    seen: set[str] = set()
    for pin in items:
        path = _resolve_pinned(str(pin["path"]))
        if str(path) in seen:
            raise GateError(f"duplicate pinned path: {path}")
        seen.add(str(path))
        actual = file_identity(path)
        expected = {key: pin[key] for key in ("mode_octal", "size_bytes", "sha256")}
        if {key: actual[key] for key in expected} != expected:
            raise GateError(f"pinned identity drift: {path}")
        observed.append(actual)
    return observed


def load_g1() -> ModuleType:
    return load_module(G1_GATE_PATH, G1_GATE_SHA256, "r8_motion_v11_g1")


def load_g2() -> ModuleType:
    return load_module(G2_GATE_PATH, G2_GATE_SHA256, "r8_motion_v11_g2")


def source_contract() -> tuple[Path, dict[str, dict[str, Any]]]:
    current, _ = policy()
    contract = current["source_contract"]
    receipt_path = Path(contract["sealed_receipt_path"])
    if file_identity(receipt_path)["sha256"] != contract["sealed_receipt_sha256"]:
        raise GateError("sealed source receipt drift")
    receipt = read_json(receipt_path)
    entries = receipt["results"]["materialization"]["sealed_output"]["entries"]
    if not isinstance(entries, list) or len(entries) != 352:
        raise GateError("sealed source entry count drift")
    expected: dict[str, dict[str, Any]] = {}
    for entry in entries:
        relative = str(entry["path"]).removeprefix("src/source/")
        if not relative or relative.startswith("/") or ".." in Path(relative).parts or relative in expected:
            raise GateError("unsafe or duplicate sealed relative path")
        expected[relative] = {"size_bytes": entry["size_bytes"], "sha256": entry["sha256"]}
    if sum(item["size_bytes"] for item in expected.values()) != contract["total_bytes"]:
        raise GateError("sealed source byte total drift")
    return Path(contract["sealed_source_root"]), expected


def inventory(root: Path, *, stage: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    _, expected = source_contract()
    if stage not in {"sealed", "copied", "patched", "wrapped"}:
        raise GateError("unknown inventory stage")
    expected_hashes = {name: dict(value) for name, value in expected.items()}
    if stage in {"patched", "wrapped"}:
        for name, digest in PATCH_AFTER_SHA256.items():
            if name not in expected_hashes:
                raise GateError(f"patch file absent from sealed manifest: {name}")
            expected_hashes[name]["sha256"] = digest
            expected_hashes[name]["size_bytes"] = PATCH_AFTER_SIZE_BYTES[name]
    allow_wrapper = stage == "wrapped"
    entries: list[dict[str, Any]] = []
    seen: set[str] = set()
    for directory, subdirs, files in os.walk(root, topdown=True, followlinks=False):
        directory_path = Path(directory)
        directory_info = os.lstat(directory_path)
        if stat.S_ISLNK(directory_info.st_mode) or not stat.S_ISDIR(directory_info.st_mode):
            raise GateError(f"unsafe inventory directory: {directory_path}")
        for name in subdirs:
            child = os.lstat(directory_path / name)
            if stat.S_ISLNK(child.st_mode) or not stat.S_ISDIR(child.st_mode):
                raise GateError(f"unsafe inventory child directory: {directory_path / name}")
        for name in files:
            path = directory_path / name
            relative = str(path.relative_to(root))
            identity = file_identity(path)
            mode = int(identity["mode_octal"], 8)
            descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
            try:
                magic = os.read(descriptor, 4)
            finally:
                os.close(descriptor)
            if mode & 0o111 or magic == b"\x7fELF":
                raise GateError(f"executable/ELF pre-build input forbidden: {relative}")
            if relative == "U3GpuBuild.mk" and allow_wrapper:
                if (identity["mode_octal"], identity["size_bytes"], identity["sha256"]) != ("0600", 84, WRAPPER_SHA256):
                    raise GateError("wrapper identity drift")
            else:
                expected_item = expected_hashes.get(relative)
                if expected_item is None or identity["size_bytes"] != expected_item["size_bytes"] or identity["sha256"] != expected_item["sha256"]:
                    raise GateError(f"source inventory drift/extra: {relative}")
                seen.add(relative)
            entries.append({
                "path": relative, "mode_octal": identity["mode_octal"],
                "size_bytes": identity["size_bytes"], "sha256": identity["sha256"],
                "nlink": identity["nlink"], "is_elf": False,
            })
    if seen != set(expected_hashes):
        raise GateError(f"source inventory missing entries: {sorted(set(expected_hashes) - seen)[:8]}")
    entries.sort(key=lambda item: item["path"])
    expected_count = 353 if allow_wrapper else 352
    if len(entries) != expected_count:
        raise GateError(f"source inventory count drift: {len(entries)}")
    return {
        "stage": stage, "entry_count": len(entries),
        "sealed_entries": 352, "wrapper_entries": 1 if allow_wrapper else 0,
        "manifest_sha256": canonical_sha256(entries),
    }, entries


def require_user_identity() -> None:
    if (os.getuid(), os.geteuid(), os.getgid(), os.getegid(), os.getgroups()) != (1000, 1000, 1000, 1000, []):
        raise GateError("phase child is not uid/gid 1000 with zero supplementary groups")


def profile_map(current: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    result = {str(item["role"]): dict(item) for item in current["profiles"]}
    if set(result) != {"SOURCE_COPY", "PATCH", "BUILD", "STATIC_AUDIT"}:
        raise GateError("exact profile role set drift")
    return result


def _replace_profile(argv: Sequence[str], old: str, new: str) -> list[str]:
    result = [new if value == old else value for value in argv]
    if result.count(new) != 1 or old in result:
        raise GateError("sandbox profile identity remap drift")
    return result


def source_copy_argv(current: Mapping[str, Any] | None = None) -> list[str]:
    current = dict(current or policy()[0])
    g1 = load_g1()
    base = g1.read_json_object(g1.POLICY_PATH)
    argv = list(base["source_copy_contract"]["full_execution_argv"])
    old_root = str(g1.ROOT_A / "output")
    argv = [str(OUTPUT_ROOT) if value == old_root else value for value in argv]
    old_name = base["profiles"]["attempt_a_copy"]["name"]
    new_name = profile_map(current)["SOURCE_COPY"]["name"]
    argv = _replace_profile(argv, old_name, new_name)
    path_token = "PATH=/usr/bin:/usr/local/cuda-12.8/bin"
    if argv.count(path_token) != 1:
        raise GateError("source-copy PATH anchor drift")
    argv[argv.index(path_token)] = "PATH=/usr/bin"
    if argv.count(str(OUTPUT_ROOT)) != 1 or argv.count("PATH=/usr/bin") != 1:
        raise GateError("source-copy exact-root/PATH drift")
    return argv


def patch_argv(current: Mapping[str, Any] | None = None) -> list[str]:
    current = dict(current or policy()[0])
    name = profile_map(current)["PATCH"]["name"]
    return [
        "/usr/bin/timeout", "--foreground", "--kill-after=30s", "300s",
        "/usr/bin/aa-exec", "-p", name, "--", "/usr/bin/bwrap",
        "--die-with-parent", "--new-session", "--clearenv", "--unshare-user",
        "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled", "--ro-bind", "/usr", "/usr",
        "--symlink", "usr/lib", "/lib", "--symlink", "usr/lib64", "/lib64",
        "--tmpfs", "/work", "--dir", "/work/source", "--bind", str(SOURCE_ROOT), "/work/source",
        "--ro-bind", str(PATCH_CHILD_PATH), "/work/helper.py",
        "--ro-bind", str(PATCH_V1_PATH), "/work/patch_v1.py", "--chdir", "/work",
        "/usr/bin/env", "-i", "HOME=/nonexistent", "PATH=/usr/bin", "LC_ALL=C.UTF-8",
        "LANG=C.UTF-8", "TZ=UTC", "PYTHONDONTWRITEBYTECODE=1", "/usr/bin/prlimit",
        "--cpu=300", "--as=1073741824", "--fsize=16777216", "--nproc=64",
        "--nofile=128", "--core=0", "--", "/usr/bin/python3", "/work/helper.py",
        "--source-root", "/work/source", "--patch-module", "/work/patch_v1.py",
    ]


def build_argv(current: Mapping[str, Any] | None = None) -> list[str]:
    current = dict(current or policy()[0])
    g1 = load_g1()
    base = g1.read_json_object(g1.POLICY_PATH)
    argv = list(base["build_contract"]["full_execution_argv"])
    argv = [str(OUTPUT_ROOT) if value == str(g1.ROOT_A / "output") else value for value in argv]
    argv = _replace_profile(argv, base["profiles"]["attempt_a_build"]["name"], profile_map(current)["BUILD"]["name"])
    anchor = ["--ro-bind", "/usr", "/usr"]
    matches = [index for index in range(len(argv) - 2) if argv[index:index + 3] == anchor]
    if matches != [19]:
        raise GateError(f"build /usr bind anchor drift: {matches}")
    insertion = matches[0] + 3
    argv = argv[:insertion] + ["--symlink", "usr/bin", "/bin"] + argv[insertion:]
    make = list(g1.MAKE_ARGV)
    if argv[-len(make):] != make or make.count("-j1") != 1:
        raise GateError("unique frozen Make argv drift")
    joined = "\n".join(argv)
    for forbidden in current["build_contract"]["forbidden_tokens"]:
        if forbidden in joined:
            raise GateError(f"forbidden build token: {forbidden}")
    if argv.count("/bin") != 1 or argv.count(str(OUTPUT_ROOT)) != 1:
        raise GateError("build sandbox delta/root drift")
    return argv


def static_plan(current: Mapping[str, Any] | None = None) -> list[tuple[str, str, list[str]]]:
    current = dict(current or policy()[0])
    g1 = load_g1()
    base = g1.read_json_object(g1.POLICY_PATH)
    old_root = str(g1.ROOT_A)
    old_name = base["profiles"]["campaign_static_audit"]["name"]
    new_name = profile_map(current)["STATIC_AUDIT"]["name"]
    result: list[tuple[str, str, list[str]]] = []
    candidate = str(CANDIDATE_PATH)
    for suffix in base["static_audit_contract"]["candidate_tool_suffixes"]:
        argv = g1.build_static_audit_argv(str(g1.OUTPUT_A / "artifacts/DualSPHysics5.4_linux64"), suffix["id"], base)
        argv = [value.replace(old_root, str(ATTEMPT_ROOT)) for value in argv]
        argv = _replace_profile(argv, old_name, new_name)
        result.append((candidate, suffix["id"], argv))
    cuda = set(base["object_contract"]["cuda_object_names"])
    for object_name in base["object_contract"]["object_names"]:
        host = str(SOURCE_ROOT / object_name)
        old_host = str(g1.OUTPUT_A / "buildtree/src/source" / object_name)
        for suffix in base["static_audit_contract"]["object_tool_suffix_templates"]:
            if suffix.get("cuda_only") and object_name not in cuda:
                continue
            argv = g1.build_static_audit_argv(old_host, suffix["id"], base)
            argv = [value.replace(old_root, str(ATTEMPT_ROOT)) for value in argv]
            argv = _replace_profile(argv, old_name, new_name)
            result.append((host, suffix["id"], argv))
    if len(result) != 557:
        raise GateError(f"static command cardinality drift: {len(result)}")
    for _, _, argv in result:
        joined = "\n".join(argv)
        if "--unshare-net" not in argv or "/dev/nvidia" in joined or "/proc" in joined:
            raise GateError("static parser sandbox network/GPU/proc drift")
    return result


def make_receipt(phase: str, kind: str, sequence: int, previous: str | None,
                 status: str, *, evidence: Mapping[str, Any] | None = None,
                 zero_residue: bool = False, profile_name: str = "NONE") -> dict[str, Any]:
    current, policy_sha = policy()
    supplied = dict(evidence or {})
    value = {
        "schema_version": "smpcc-r8-liquid-motion-gauge-gpu-build-execution-receipt-v11",
        "campaign_id": CAMPAIGN_ID, "build_id": BUILD_ID, "sequence": sequence,
        "previous_sha256": previous, "phase": phase, "kind": kind, "status": status,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "policy_sha256": policy_sha,
        "evidence": {
            "argv": supplied.get("argv", []), "return_code": supplied.get("return_code"),
            "elapsed_seconds": float(supplied.get("elapsed_seconds", 0.0)),
            "stdout_sha256": supplied.get("stdout_sha256", ZERO_SHA256),
            "stderr_sha256": supplied.get("stderr_sha256", ZERO_SHA256),
            "monitor_sha256": supplied.get("monitor_sha256", ZERO_SHA256),
            "inventory_sha256": supplied.get("inventory_sha256", ZERO_SHA256),
            "inventory_entries": int(supplied.get("inventory_entries", 0)),
            "patched_files": int(supplied.get("patched_files", 0)),
            "unchanged_entries": int(supplied.get("unchanged_entries", 0)),
            "wrapper_sha256": supplied.get("wrapper_sha256", ZERO_SHA256),
            "objects": int(supplied.get("objects", 0)),
            "static_commands": int(supplied.get("static_commands", 0)),
            "semantic_checks": int(supplied.get("semantic_checks", 0)),
            "candidate": supplied.get("candidate"),
            "patch_journal_sha256": supplied.get("patch_journal_sha256", ZERO_SHA256),
            "lifecycle_events": supplied.get("lifecycle_events", []),
            "error": supplied.get("error"),
        },
        "safety": {
            "uid": int(supplied.get("uid", os.getuid())), "gid": int(supplied.get("gid", os.getgid())),
            "supplementary_groups": int(supplied.get("supplementary_groups", len(os.getgroups()))),
            "create_new": True, "o_nofollow": True, "fsync": True,
            "profile_name": profile_name, "profile_zero_residue": zero_residue,
            "candidate_executed": False, "gpu_exposed": False, "network_used": False,
            "host_binary_content_parser_run": False,
            "make_count": 1 if phase in {"build", "static-audit", "campaign"} and sequence >= 9 else 0,
            "failure_preserved": True,
        },
    }
    _, receipt_schema, _ = schemas()
    Draft202012Validator(receipt_schema).validate(value)
    return value


def receipt_path(sequence: int, phase: str, kind: str) -> Path:
    return AUDIT_ROOT / f"{sequence:02d}_{phase}_{kind.lower()}_v11.json"


def emit_receipt(phase: str, kind: str, sequence: int, previous: str | None,
                 status: str, **kwargs: Any) -> dict[str, Any]:
    value = make_receipt(phase, kind, sequence, previous, status, **kwargs)
    return write_new(receipt_path(sequence, phase, kind), canonical_json(value), mode=0o640)


def previous_receipt(sequence: int, phase: str, kind: str) -> tuple[dict[str, Any], str]:
    path = receipt_path(sequence, phase, kind)
    value = read_json(path)
    identity = file_identity(path)
    if value["sequence"] != sequence or value["phase"] != phase or value["kind"] != kind:
        raise GateError(f"receipt transition mismatch: {path}")
    return value, identity["sha256"]


def create_attempt_tree() -> None:
    if os.path.lexists(ATTEMPT_ROOT) or os.path.lexists(AUDIT_ROOT):
        raise GateError("fresh attempt/audit root already exists")
    for path, mode in (
        (AUDIT_ROOT, 0o750), (ATTEMPT_ROOT, 0o750), (OUTPUT_ROOT, 0o700),
        (OUTPUT_ROOT / "buildtree", 0o700), (OUTPUT_ROOT / "buildtree/src", 0o700),
        (SOURCE_ROOT, 0o700), (ARTIFACTS_ROOT, 0o700),
    ):
        os.mkdir(path, mode)
        os.chmod(path, mode, follow_symlinks=False)


def dynamic_preflight(*, require_fresh: bool) -> dict[str, Any]:
    current, _ = policy()
    verify_pins(current["parents"])
    verify_pins(current["system_tools"])
    if require_fresh and (os.path.lexists(ATTEMPT_ROOT) or os.path.lexists(AUDIT_ROOT)):
        raise GateError("campaign root is not fresh")
    memory: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].rstrip(":") in {"MemAvailable", "SwapFree"}:
            memory[parts[0].rstrip(":")] = int(parts[1]) * 1024
    if memory.get("MemAvailable", 0) < current["resources"]["minimum_available_memory_bytes"]:
        raise GateError("MemAvailable below frozen 4 GiB minimum")
    disk = os.statvfs(ATTEMPT_ROOT.parent)
    available_disk = disk.f_bavail * disk.f_frsize
    if available_disk < current["resources"]["minimum_available_disk_bytes"]:
        raise GateError("available build disk below 20 GiB")
    return {"memory_available_bytes": memory.get("MemAvailable", 0),
            "swap_free_bytes": memory.get("SwapFree", 0), "disk_available_bytes": available_disk}


def bounded_capture(argv: Sequence[str], *, limit: int, timeout_seconds: int,
                    executor: Callable[[Sequence[str]], tuple[int, bytes, bytes]] | None = None) -> tuple[dict[str, Any], bytes, bytes]:
    started = time.monotonic()
    if executor is not None:
        rc, stdout, stderr = executor(argv)
    else:
        process = subprocess.Popen(list(argv), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, start_new_session=True, close_fds=True)
        try:
            stdout, stderr = process.communicate(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                stdout, stderr = process.communicate(timeout=30)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                stdout, stderr = process.communicate()
            raise GateError("confined process group exceeded wall timeout")
        rc = process.returncode
    if len(stdout) + len(stderr) > limit:
        raise GateError("confined output exceeded frozen bound")
    return {
        "argv": list(argv), "return_code": rc, "elapsed_seconds": round(time.monotonic() - started, 6),
        "stdout_sha256": sha256_bytes(stdout), "stderr_sha256": sha256_bytes(stderr),
    }, stdout, stderr


def parse_patch_journal(raw: bytes) -> dict[str, Any]:
    if len(raw) > 16 * 1024 * 1024:
        raise GateError("patch journal exceeds frozen bound")
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(raw.splitlines(), 1):
        if not line:
            continue
        try:
            value = json.loads(
                line.decode("utf-8"),
                parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
            )
        except Exception as exc:
            raise GateError(f"invalid patch journal JSONL line {line_number}: {exc}") from exc
        if not isinstance(value, dict):
            raise GateError(f"patch journal line {line_number} is not an object")
        records.append(value)
    expected_events = [
        "PATCH_START", "PATCH_PLAN_AUTHENTICATED",
        *("ATOMIC_FILE_REPLACED" for _ in PATCH_NAMES), "PATCH_FINAL",
    ]
    if len(records) != len(expected_events):
        raise GateError(f"patch journal record count drift: {len(records)}")
    if [item.get("sequence") for item in records] != list(range(1, len(records) + 1)):
        raise GateError("patch journal sequence drift")
    if [item.get("event") for item in records] != expected_events:
        raise GateError("patch journal event order drift")
    replacements = records[2:-1]
    if [item.get("path") for item in replacements] != sorted(PATCH_NAMES):
        raise GateError("patch journal replacement path/order drift")
    for index, item in enumerate(replacements, 1):
        if (item.get("completed_count"), item.get("atomic_per_file"),
            item.get("cross_file_transaction")) != (index, True, False):
            raise GateError("patch journal atomic replacement evidence drift")
    final = records[-1]
    if (
        final.get("status") != "PASS_CONFINED_EXACT_SIX_FILE_PATCH_V11"
        or final.get("changed_count") != 6
        or final.get("completed_files") != sorted(PATCH_NAMES)
        or final.get("six_file_aggregate_sha256")
        != "3cf3e7d883f7c751d7b5455eec352c6df46ff669cf6b5a989471be5f7731f528"
        or final.get("patch_module_sha256") != PATCH_V1_SHA256
        or final.get("atomic_per_file") is not True
        or final.get("cross_file_transaction") is not False
    ):
        raise GateError("patch journal final contract drift")
    return {"records": records, "final": final}


def create_wrapper() -> dict[str, Any]:
    identity = write_new(SOURCE_ROOT / "U3GpuBuild.mk", WRAPPER_BYTES, mode=0o600)
    if (identity["mode_octal"], identity["size_bytes"], identity["sha256"]) != ("0600", 84, WRAPPER_SHA256):
        raise GateError("materialized wrapper identity drift")
    return identity


def disarm_candidate() -> dict[str, Any] | None:
    if not os.path.lexists(CANDIDATE_PATH):
        return None
    descriptor = os.open(CANDIDATE_PATH, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size <= 0:
            raise GateError("unsafe candidate before disarm")
        os.fchmod(descriptor, 0o400)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    item = file_identity(CANDIDATE_PATH)
    if item["mode_octal"] != "0400":
        raise GateError("candidate immediate disarm failed")
    return {key: item[key] for key in ("path", "mode_octal", "size_bytes", "sha256", "device", "inode", "nlink")}


def assert_same_artifact_inode(first: Mapping[str, Any] | None,
                               current: Mapping[str, Any] | None) -> None:
    if first is None or current is None:
        return
    if (first["device"], first["inode"], first["nlink"]) != (
        current["device"], current["inode"], current["nlink"]
    ):
        raise GateError("candidate inode changed after first observation")


def _phase_start(sequence: int, previous: str | None, phase: str, argv: list[str], profile: str) -> str:
    item = emit_receipt(phase, "START", sequence, previous, f"START_{phase.upper()}",
                        evidence={"argv": argv}, profile_name=profile)
    return item["sha256"]


def run_source_copy() -> int:
    require_user_identity()
    current, _ = policy()
    dynamic_preflight(require_fresh=True)
    create_attempt_tree()
    argv = source_copy_argv(current)
    chain = _phase_start(1, None, "source-copy", argv, profile_map(current)["SOURCE_COPY"]["name"])
    status = "FAIL_SOURCE_COPY_NO_RETRY"
    try:
        result, _, _ = bounded_capture(argv, limit=16 * 1024 * 1024, timeout_seconds=300)
        summary, _ = inventory(SOURCE_ROOT, stage="copied")
        if result["return_code"] != 0:
            raise GateError(f"source-copy rc={result['return_code']}")
        status = "PASS_SOURCE_COPY_352_AWAITING_LIFECYCLE"
        identity = emit_receipt("source-copy", "RUN", 2, chain, status,
                                evidence={**result, "inventory_sha256": summary["manifest_sha256"], "inventory_entries": 352},
                                profile_name=profile_map(current)["SOURCE_COPY"]["name"])
        return 0 if identity else 3
    except Exception as exc:
        emit_receipt("source-copy", "FAILURE", 2, chain, status,
                     evidence={"error": str(exc), "return_code": 3},
                     profile_name=profile_map(current)["SOURCE_COPY"]["name"])
        return 3


def finalize_source_copy_lifecycle(events: Sequence[Mapping[str, Any]], zero: bool) -> int:
    require_user_identity()
    current, _ = policy()
    run_receipt, chain = previous_receipt(2, "source-copy", "RUN")
    if run_receipt["status"] != "PASS_SOURCE_COPY_352_AWAITING_LIFECYCLE" or not zero:
        raise GateError("copy lifecycle does not admit patch")
    summary, _ = inventory(SOURCE_ROOT, stage="copied")
    emit_receipt("source-copy", "LIFECYCLE", 3, chain, "PASS_SOURCE_COPY_PROFILE_ZERO_RESIDUE",
                 evidence={"inventory_sha256": summary["manifest_sha256"], "inventory_entries": 352,
                           "lifecycle_events": list(events)}, zero_residue=True,
                 profile_name=profile_map(current)["SOURCE_COPY"]["name"])
    return 0


def run_patch() -> int:
    require_user_identity()
    current, _ = policy()
    previous, chain = previous_receipt(3, "source-copy", "LIFECYCLE")
    if not previous["safety"]["profile_zero_residue"]:
        raise GateError("copy profile residue prevents patch")
    argv = patch_argv(current)
    chain = _phase_start(4, chain, "patch", argv, profile_map(current)["PATCH"]["name"])
    try:
        before, _ = inventory(SOURCE_ROOT, stage="copied")
        result, stdout, _ = bounded_capture(argv, limit=16 * 1024 * 1024, timeout_seconds=300)
        if result["return_code"] != 0:
            raise GateError(f"patch child rc={result['return_code']}")
        patch_report = parse_patch_journal(stdout)
        after, _ = inventory(SOURCE_ROOT, stage="patched")
        emit_receipt("patch", "RUN", 5, chain, "PASS_PATCH_6_CHANGED_346_UNCHANGED_AWAITING_LIFECYCLE",
                     evidence={**result, "inventory_sha256": after["manifest_sha256"], "inventory_entries": 352,
                               "patched_files": 6, "unchanged_entries": 346,
                               "patch_journal_sha256": sha256_bytes(stdout)},
                     profile_name=profile_map(current)["PATCH"]["name"])
        if before["entry_count"] != after["entry_count"]:
            raise GateError("patch inventory count drift")
        return 0
    except Exception as exc:
        emit_receipt("patch", "FAILURE", 5, chain, "FAIL_PATCH_NO_RETRY",
                     evidence={"error": str(exc), "return_code": 3},
                     profile_name=profile_map(current)["PATCH"]["name"])
        return 3


def finalize_patch_lifecycle(events: Sequence[Mapping[str, Any]], zero: bool) -> int:
    require_user_identity()
    current, _ = policy()
    run_receipt, chain = previous_receipt(5, "patch", "RUN")
    if run_receipt["status"] != "PASS_PATCH_6_CHANGED_346_UNCHANGED_AWAITING_LIFECYCLE" or not zero:
        raise GateError("patch lifecycle does not admit wrapper")
    summary, _ = inventory(SOURCE_ROOT, stage="patched")
    emit_receipt("patch", "LIFECYCLE", 6, chain, "PASS_PATCH_PROFILE_ZERO_RESIDUE",
                 evidence={"inventory_sha256": summary["manifest_sha256"], "inventory_entries": 352,
                           "patched_files": 6, "unchanged_entries": 346,
                           "patch_journal_sha256": run_receipt["evidence"]["patch_journal_sha256"],
                           "lifecycle_events": list(events)}, zero_residue=True,
                 profile_name=profile_map(current)["PATCH"]["name"])
    return 0


def run_wrapper() -> int:
    require_user_identity()
    previous, chain = previous_receipt(6, "patch", "LIFECYCLE")
    if not previous["safety"]["profile_zero_residue"]:
        raise GateError("patch residue prevents wrapper")
    argv = ["O_EXCL", "0600", "84_BYTE_U3GpuBuild.mk"]
    chain = _phase_start(7, chain, "wrapper", argv, "NONE")
    try:
        item = create_wrapper()
        summary, _ = inventory(SOURCE_ROOT, stage="wrapped")
        emit_receipt("wrapper", "FINAL", 8, chain, "PASS_BUILD_INPUT_353",
                     evidence={"argv": argv, "return_code": 0, "inventory_sha256": summary["manifest_sha256"],
                               "inventory_entries": 353, "patched_files": 6, "unchanged_entries": 346,
                               "wrapper_sha256": item["sha256"]}, zero_residue=True, profile_name="NONE")
        return 0
    except Exception as exc:
        emit_receipt("wrapper", "FAILURE", 8, chain, "FAIL_WRAPPER_NO_RETRY",
                     evidence={"argv": argv, "return_code": 3, "error": str(exc)}, profile_name="NONE")
        return 3


def _resource_snapshot() -> dict[str, Any]:
    available = 0
    swap = 0
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        fields = line.split()
        if fields and fields[0] == "MemAvailable:": available = int(fields[1]) * 1024
        if fields and fields[0] == "SwapFree:": swap = int(fields[1]) * 1024
    return {"captured_at_ns": time.time_ns(), "memory_available_bytes": available,
            "swap_free_bytes": swap, "psi": Path("/proc/pressure/memory").read_text(encoding="ascii")}


def run_build() -> int:
    require_user_identity()
    current, _ = policy()
    previous, chain = previous_receipt(8, "wrapper", "FINAL")
    if previous["status"] != "PASS_BUILD_INPUT_353":
        raise GateError("wrapper receipt does not admit build")
    dynamic_preflight(require_fresh=False)
    inventory(SOURCE_ROOT, stage="wrapped")
    argv = build_argv(current)
    chain = _phase_start(9, chain, "build", argv, profile_map(current)["BUILD"]["name"])
    stdout_path = AUDIT_ROOT / "build.stdout.log"
    stderr_path = AUDIT_ROOT / "build.stderr.log"
    monitor_path = AUDIT_ROOT / "build.resource.jsonl"
    stdout_fd = open_new(stdout_path, mode=0o640)
    stderr_fd = open_new(stderr_path, mode=0o640)
    monitor_fd = open_new(monitor_path, mode=0o640)
    process: subprocess.Popen[bytes] | None = None
    started = time.monotonic()
    stdout_sha = hashlib.sha256(); stderr_sha = hashlib.sha256()
    stdout_bytes = stderr_bytes = 0
    candidate: dict[str, Any] | None = None
    error: str | None = None
    try:
        process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   start_new_session=True, close_fds=True)
        if process.stdout is None or process.stderr is None:
            raise GateError("cannot capture Make streams")
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        selector.register(process.stderr, selectors.EVENT_READ, "stderr")
        next_sample = 0.0
        while selector.get_map() or process.poll() is None:
            now = time.monotonic()
            if now >= next_sample:
                write_all(monitor_fd, canonical_json(_resource_snapshot()))
                next_sample = now + current["resources"]["monitor_interval_seconds"]
            observed_candidate = disarm_candidate()
            assert_same_artifact_inode(candidate, observed_candidate)
            if candidate is None:
                candidate = observed_candidate
            if now - started >= current["resources"]["wall_timeout_seconds"] and process.poll() is None:
                try: os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError: pass
            for key, _ in selector.select(timeout=1.0):
                block = os.read(key.fd, 65536)
                if not block:
                    selector.unregister(key.fileobj); continue
                if key.data == "stdout":
                    write_all(stdout_fd, block); stdout_sha.update(block); stdout_bytes += len(block)
                else:
                    write_all(stderr_fd, block); stderr_sha.update(block); stderr_bytes += len(block)
                if stdout_bytes + stderr_bytes > current["resources"]["stream_limit_bytes"]:
                    raise GateError("Make output exceeded frozen bound")
        rc = process.wait()
        observed_candidate = disarm_candidate()
        assert_same_artifact_inode(candidate, observed_candidate)
        candidate = observed_candidate or candidate
        if rc != 0 or candidate is None:
            raise GateError(f"Make failed/no candidate: rc={rc}")
        for descriptor in (stdout_fd, stderr_fd, monitor_fd): os.fsync(descriptor)
        monitor_sha = file_identity(monitor_path)["sha256"]
        emit_receipt("build", "RUN", 10, chain, "PASS_BUILD_CANDIDATE_0400_AWAITING_LIFECYCLE",
                     evidence={"argv": argv, "return_code": rc, "elapsed_seconds": time.monotonic()-started,
                               "stdout_sha256": stdout_sha.hexdigest(), "stderr_sha256": stderr_sha.hexdigest(),
                               "monitor_sha256": monitor_sha, "inventory_entries": 353, "objects": 131,
                               "candidate": candidate}, profile_name=profile_map(current)["BUILD"]["name"])
        return 0
    except Exception as exc:
        error = str(exc)
        if process is not None and process.poll() is None:
            try: os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError: pass
            try: process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                try: os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                process.wait()
        try:
            observed_candidate = disarm_candidate()
            assert_same_artifact_inode(candidate, observed_candidate)
            candidate = observed_candidate or candidate
        except Exception as disarm_error: error += f"; disarm={disarm_error}"
        emit_receipt("build", "FAILURE", 10, chain, "FAIL_BUILD_NO_RETRY",
                     evidence={"argv": argv, "return_code": 3, "elapsed_seconds": time.monotonic()-started,
                               "stdout_sha256": stdout_sha.hexdigest(), "stderr_sha256": stderr_sha.hexdigest(),
                               "candidate": candidate, "error": error},
                     profile_name=profile_map(current)["BUILD"]["name"])
        return 3
    finally:
        for descriptor in (stdout_fd, stderr_fd, monitor_fd):
            try: os.close(descriptor)
            except OSError: pass


def finalize_build_lifecycle(events: Sequence[Mapping[str, Any]], zero: bool) -> int:
    require_user_identity()
    current, _ = policy()
    run_receipt, chain = previous_receipt(10, "build", "RUN")
    if run_receipt["status"] != "PASS_BUILD_CANDIDATE_0400_AWAITING_LIFECYCLE" or not zero:
        raise GateError("build lifecycle does not admit static audit")
    candidate = disarm_candidate()
    if candidate is None or candidate != run_receipt["evidence"]["candidate"]:
        raise GateError("candidate identity drift after build lifecycle")
    emit_receipt("build", "LIFECYCLE", 11, chain, "PASS_BUILD_PROFILE_ZERO_RESIDUE",
                 evidence={"candidate": candidate, "inventory_entries": 353, "objects": 131,
                           "lifecycle_events": list(events)}, zero_residue=True,
                 profile_name=profile_map(current)["BUILD"]["name"])
    return 0


def _artifact_for_schema(path: Path) -> dict[str, Any]:
    item = file_identity(path)
    return {key: item[key] for key in ("path", "mode_octal", "size_bytes", "sha256", "device", "inode", "nlink")}


def validate_static_outputs(outputs: Mapping[tuple[str, str], str], candidate: Path,
                            objects: Sequence[Path]) -> int:
    g1 = load_g1(); g2 = load_g2()
    candidate_output = {name: outputs[(str(candidate), name)] for name in (
        "file", "readelf_header", "readelf_program_headers", "readelf_dynamic", "readelf_notes",
        "cuobjdump_list_elf", "cuobjdump_dump_elf", "cuobjdump_dump_elf_symbols",
        "cuobjdump_list_ptx", "cuobjdump_dump_ptx", "sha256sum")}
    checks = g2.validate_candidate_text(candidate_output, file_identity(candidate)["sha256"])
    base = g1.read_json_object(g1.POLICY_PATH)
    cuda = set(base["object_contract"]["cuda_object_names"])
    for path in objects:
        current = {name: outputs[(str(path), name)] for name in
                   ("object_file", "object_readelf_header", "object_readelf_sections", "object_sha256sum")}
        if path.name in cuda:
            current["cuda_object_list_elf"] = outputs[(str(path), "cuda_object_list_elf")]
            current["cuda_object_list_ptx"] = outputs[(str(path), "cuda_object_list_ptx")]
        checks.extend(g2.validate_object_text(path.name, current, file_identity(path)["sha256"], cuda=path.name in cuda))
    return len(checks)


def run_static_audit() -> int:
    require_user_identity()
    current, _ = policy()
    previous, chain = previous_receipt(11, "build", "LIFECYCLE")
    if not previous["safety"]["profile_zero_residue"]:
        raise GateError("build profile residue prevents static audit")
    candidate = _artifact_for_schema(CANDIDATE_PATH)
    if candidate["mode_octal"] != "0400":
        raise GateError("candidate is not disarmed")
    g1 = load_g1(); base = g1.read_json_object(g1.POLICY_PATH)
    objects = [SOURCE_ROOT / name for name in base["object_contract"]["object_names"]]
    before = [_artifact_for_schema(CANDIDATE_PATH), *[_artifact_for_schema(path) for path in objects]]
    plan = static_plan(current)
    chain = _phase_start(12, chain, "static-audit", ["557_EXACT_CONFINED_COMMANDS"], profile_map(current)["STATIC_AUDIT"]["name"])
    framed_path = AUDIT_ROOT / "static.parser-output.bin"
    summary_path = AUDIT_ROOT / "static.commands.jsonl"
    framed_fd = open_new(framed_path, mode=0o640); summary_fd = open_new(summary_path, mode=0o640)
    outputs: dict[tuple[str, str], str] = {}; total = 0; started = time.monotonic()
    try:
        for index, (host, suffix, argv) in enumerate(plan, 1):
            result, stdout, stderr = bounded_capture(argv, limit=current["resources"]["static_per_command_limit_bytes"], timeout_seconds=60)
            total += len(stdout) + len(stderr)
            if result["return_code"] != 0 or total > current["resources"]["static_total_limit_bytes"]:
                raise StaticAuditError(f"static parser failure/aggregate overflow at {index}:{suffix}")
            header = {"index": index, "host": host, "suffix": suffix, **result}
            write_all(summary_fd, canonical_json(header))
            write_all(framed_fd, b"R8V11 " + canonical_json(header) + stdout + stderr + b"\nR8V11-END\n")
            outputs[(host, suffix)] = (stdout + b"\n" + stderr).decode("utf-8", "replace")
        checks = validate_static_outputs(outputs, CANDIDATE_PATH, objects)
        after = [_artifact_for_schema(CANDIDATE_PATH), *[_artifact_for_schema(path) for path in objects]]
        if before != after:
            raise StaticAuditError("candidate/object identities changed during static audit")
        os.fsync(framed_fd); os.fsync(summary_fd)
        emit_receipt("static-audit", "RUN", 13, chain, "PASS_STATIC_557_AWAITING_LIFECYCLE",
                     evidence={"argv": ["557_EXACT_CONFINED_COMMANDS"], "return_code": 0,
                               "elapsed_seconds": time.monotonic()-started, "static_commands": 557,
                               "semantic_checks": checks, "objects": 131, "candidate": candidate,
                               "stdout_sha256": file_identity(framed_path)["sha256"],
                               "stderr_sha256": file_identity(summary_path)["sha256"]},
                     profile_name=profile_map(current)["STATIC_AUDIT"]["name"])
        return 0
    except Exception as exc:
        emit_receipt("static-audit", "FAILURE", 13, chain, "FAIL_STATIC_AUDIT_NO_RETRY",
                     evidence={"argv": ["557_EXACT_CONFINED_COMMANDS"], "return_code": 4,
                               "elapsed_seconds": time.monotonic()-started, "static_commands": len(outputs),
                               "objects": 131, "candidate": candidate, "error": str(exc)},
                     profile_name=profile_map(current)["STATIC_AUDIT"]["name"])
        return 4
    finally:
        for descriptor in (framed_fd, summary_fd):
            try: os.close(descriptor)
            except OSError: pass


def finalize_static_lifecycle(events: Sequence[Mapping[str, Any]], zero: bool) -> int:
    require_user_identity()
    current, _ = policy()
    run_receipt, chain = previous_receipt(13, "static-audit", "RUN")
    if run_receipt["status"] != "PASS_STATIC_557_AWAITING_LIFECYCLE" or not zero:
        raise GateError("static lifecycle does not admit final campaign receipt")
    candidate = _artifact_for_schema(CANDIDATE_PATH)
    lifecycle = emit_receipt("static-audit", "LIFECYCLE", 14, chain, "PASS_STATIC_PROFILE_ZERO_RESIDUE",
                             evidence={"candidate": candidate, "objects": 131, "static_commands": 557,
                                       "semantic_checks": run_receipt["evidence"]["semantic_checks"],
                                       "lifecycle_events": list(events)}, zero_residue=True,
                             profile_name=profile_map(current)["STATIC_AUDIT"]["name"])
    emit_receipt("campaign", "FINAL", 15, lifecycle["sha256"],
                 "PASS_MOTION_GAUGE_GPU_BUILD_SM120_STATIC_AUDIT_DEVELOPMENT_CANDIDATE",
                 evidence={"candidate": candidate, "inventory_entries": 353, "patched_files": 6,
                           "unchanged_entries": 346, "wrapper_sha256": WRAPPER_SHA256,
                           "objects": 131, "static_commands": 557,
                           "semantic_checks": run_receipt["evidence"]["semantic_checks"]},
                 zero_residue=True, profile_name="NONE")
    return 0


def validate_authorization_token(path: Path) -> dict[str, Any]:
    current, policy_sha = policy()
    _, _, token_schema = schemas()
    identity = file_identity(path, limit=16384)
    if identity["mode_octal"] != "0600":
        raise GateError("authorization token mode is not 0600")
    value = read_json(path, limit=16384)
    Draft202012Validator(token_schema).validate(value)
    if value["campaign_id"] != CAMPAIGN_ID or not value["user_authorized"]:
        raise GateError("authorization token campaign/decision drift")
    expected_policy = file_identity(POLICY_PATH)
    expected_gate = file_identity(Path(__file__).resolve())
    if value["policy"] != {key: expected_policy[key] for key in ("path", "mode_octal", "size_bytes", "sha256")}:
        raise GateError("authorization token policy identity drift")
    if value["policy"]["sha256"] != policy_sha:
        raise GateError("authorization token policy SHA drift")
    if value["gate"] != {key: expected_gate[key] for key in ("path", "mode_octal", "size_bytes", "sha256")}:
        raise GateError("authorization token gate identity drift")
    authorization = current["authorization"]
    for field, policy_key in (("supervisor", "supervisor_path"),
                              ("token_producer", "token_producer_path")):
        pinned_path = _resolve_pinned(authorization[policy_key])
        observed = file_identity(pinned_path)
        expected = {key: observed[key] for key in ("path", "mode_octal", "size_bytes", "sha256")}
        if value[field] != expected:
            raise GateError(f"authorization token {field} identity drift")
    expected_profiles = [{key: item[key] for key in ("role", "name", "path", "mode_octal", "size_bytes", "sha256")}
                         for item in current["profiles"]]
    if value["profiles"] != expected_profiles:
        raise GateError("authorization token exact profiles drift")
    return value


def validate_static_contract(*, require_fresh: bool = True) -> dict[str, Any]:
    current, policy_sha = policy()
    parents = verify_pins(current["parents"])
    tools = verify_pins(current["system_tools"])
    profiles = verify_pins(current["profiles"])
    authorization = current.get("authorization")
    authorization_tools: list[dict[str, Any]] = []
    if authorization is not None:
        authorization_tools = [
            file_identity(_resolve_pinned(authorization[path_key]))
            for path_key in ("token_producer_path", "supervisor_path")
        ]
    g1 = load_g1(); g1_policy = g1.validate_policy_schema()
    g1.verify_plan_identity(); g1.verify_source_inputs(g1_policy); g1.verify_object_contract(g1_policy)
    source_root, _ = source_contract(); sealed, _ = inventory(source_root, stage="sealed")
    copy = source_copy_argv(current); patch = patch_argv(current); build = build_argv(current); static = static_plan(current)
    if current["build_contract"]["make_argv"] != list(g1.MAKE_ARGV):
        raise GateError("policy Make argv differs from frozen G1")
    if len(static) != 557 or len(profiles) != 4 or sealed["entry_count"] != 352:
        raise GateError("static cardinality drift")
    if require_fresh and (os.path.lexists(ATTEMPT_ROOT) or os.path.lexists(AUDIT_ROOT) or os.path.lexists(TOKEN_PATH)):
        raise GateError("fresh v11 external path already exists")
    return {
        "status": "PASS_MOTION_GAUGE_GPU_BUILD_V11_STATIC_DEFAULT_DENY",
        "policy_sha256": policy_sha, "parent_count": len(parents), "system_tool_count": len(tools),
        "authorization_tool_count": len(authorization_tools),
        "profile_count": len(profiles), "source_entries": 352, "patch_files": 6,
        "unchanged_entries": 346, "wrapper_bytes": 84, "objects": 131,
        "cpp_objects": 120, "cuda_objects": 11, "static_commands": len(static),
        "source_copy_argv_sha256": canonical_sha256(copy), "patch_argv_sha256": canonical_sha256(patch),
        "build_argv_sha256": canonical_sha256(build), "static_plan_sha256": canonical_sha256(static),
        "files_written": False, "system_actions_performed": False, "make_run": False,
        "compiler_run": False, "candidate_executed": False, "profile_loaded": False,
    }


def self_check() -> dict[str, Any]:
    return validate_static_contract(require_fresh=True)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "source-copy", "patch", "wrapper", "build", "static-audit"))
    parser.add_argument("--authorization-token-file")
    parser.add_argument("--lifecycle-events-json")
    parser.add_argument("--zero-residue", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            if args.authorization_token_file or args.lifecycle_events_json or args.zero_residue:
                raise GateError("self-check rejects dynamic inputs")
            report = self_check()
            print(json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
            return 0
        if not args.authorization_token_file:
            raise GateError("dynamic phase requires exact v11 authorization token")
        validate_authorization_token(Path(args.authorization_token_file))
        events: list[Mapping[str, Any]] = []
        if args.lifecycle_events_json:
            loaded = json.loads(args.lifecycle_events_json)
            if not isinstance(loaded, list): raise GateError("lifecycle events are not an array")
            events = loaded
        lifecycle_requested = args.lifecycle_events_json is not None or args.zero_residue
        if lifecycle_requested and (not args.lifecycle_events_json or not args.zero_residue):
            raise GateError("lifecycle finalization requires events and explicit zero residue")
        if args.command == "source-copy":
            return finalize_source_copy_lifecycle(events, args.zero_residue) if lifecycle_requested else run_source_copy()
        if args.command == "patch":
            return finalize_patch_lifecycle(events, args.zero_residue) if lifecycle_requested else run_patch()
        if args.command == "wrapper": return run_wrapper()
        if args.command == "build":
            return finalize_build_lifecycle(events, args.zero_residue) if lifecycle_requested else run_build()
        if args.command == "static-audit":
            return finalize_static_lifecycle(events, args.zero_residue) if lifecycle_requested else run_static_audit()
        raise GateError("unreachable phase")
    except Exception as exc:
        print(json.dumps({"status": "FAIL_MOTION_GAUGE_GPU_BUILD_V11", "error": str(exc)},
                         ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
