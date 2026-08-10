#!/usr/bin/env python3
"""One-shot, fail-closed U3 source-copy and CPU-build gate for this MSI host.

This program deliberately does *not* install or load an AppArmor profile and
does not use sudo.  A separately reviewed, transient named profile must have
already been loaded for the exact phase.  The only state-changing commands are
``run-source-copy`` and ``run-build``; both use fixed argv, new paths only and
append-only receipts.  Neither command can execute an upstream binary or the
resulting build artifact.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
BUILD_ID = "u3_source_cpu_build_20260806T182853Z"
ATTEMPT_ROOT = LIQUID_ROOT / "build" / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
SOURCE_ROOT = (
    LIQUID_ROOT
    / "dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/source"
)
SOURCE_RECEIPT = LIQUID_ROOT / "audits/u3_source_materialization_v1_20260806T155752Z.json"
COPY_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_source_copy.json"
BUILD_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_cpu_build.json"
POLICY_PATH = (
    PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v10.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v10.json"
COPY_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-source-copy-v10.profile"
BUILD_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-cpu-build-v10.profile"
COPY_PROFILE = "r8-liquid-u3-source-copy-v10"
BUILD_PROFILE = "r8-liquid-u3-cpu-build-v10"

COPY_PROFILE_SHA256 = "6dac557aab0392251fcf4c71f7f0c40f1c332165e7fbf3ce071b2a9aaa54c349"
BUILD_PROFILE_SHA256 = "8fb23c96493d6757c86283e89ebdc581bbc8f8ef7f9b2496b00570d8956f950c"
SOURCE_RECEIPT_SHA256 = "90af263fb7ec8b7d6a46a53aa5354dd5b676cd4167d48fde93911e014a70b745"
SOURCE_FILE_COUNT = 352
SOURCE_TOTAL_BYTES = 5_473_917
HOST_USERNS_LIMIT_PATH = Path("/proc/sys/user/max_user_namespaces")
WRAPPER_CONTENT = ".SUFFIXES:\n.SUFFIXES: .cpp .o\ninclude Makefile_cpu\noverride CCFLAGS += -include cstdint\n"
WRAPPER_NAME = "U3Build.mk"
ARTIFACT_RELATIVE = Path("artifacts/DualSPHysics5.4CPU_linux64")
OUTPUT_SOURCE_RELATIVE = Path("buildtree/src/source")
OUTPUT_SOURCE = OUTPUT_ROOT / OUTPUT_SOURCE_RELATIVE
ARTIFACT_PATH = OUTPUT_ROOT / ARTIFACT_RELATIVE

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "apparmor_parser": (Path("/usr/sbin/apparmor_parser"), "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"),
    "as": (Path("/usr/bin/x86_64-linux-gnu-as"), "21aff249b692b5c31a44007491f922dcb49f41323e362c57d2ada3f52eddb7f0"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "cc1plus": (Path("/usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus"), "840b332fb62ec6f694ac77d91fe69ef7f80b0d69512ed89374af0ee7a506255d"),
    "collect2": (Path("/usr/libexec/gcc/x86_64-linux-gnu/13/collect2"), "4d1f341ae5b763b513258ee2812422a45e063c30a2f1924a0cf63d3699f3a158"),
    "cp": (Path("/usr/bin/cp"), "6187dab32cd7b6291d96c54425a22915a6eeb39add6d95ee9008e1dbba427131"),
    "dash": (Path("/usr/bin/dash"), "86d31f6fb799e91fa21bad341484564510ca287703a16e9e46c53338776f4f42"),
    "env": (Path("/usr/bin/env"), "0aefff8f912fb75716c5d4de3b6acde93edbe8fa280fc8ee895c1226d3e373ef"),
    "gpp": (Path("/usr/bin/x86_64-linux-gnu-g++-13"), "1353e9bdd29a7295c7226bf6c63abccce056d8cac31f112e5cdbecc3f28c2769"),
    "ld": (Path("/usr/bin/x86_64-linux-gnu-ld.bfd"), "e9ceb054c12207970f2726dfc07e9a66b411602748628baf27399f02a9bbb31b"),
    "make": (Path("/usr/bin/make"), "d78b8f1d099fbcfb6f2f49ab87223b9b68fb3956642f92d6ec6de812e8afa965"),
    "prlimit": (Path("/usr/bin/prlimit"), "f27cfd8c1512a4cc6541b59b80cb4cdfd6ef28c34aa21db4299b48264cd0d128"),
    "setpriv": (Path("/usr/bin/setpriv"), "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}

FORBIDDEN_PROFILE_TOKENS = (
    "flags=(unconfined)",
    " change_profile",
    "/home/zrj/scout_ws",
    "/home/zrj/ros2_ws",
    "/opt/ros/jazzy",
    "/home/zrj/scout_liquid_lab/dependency/source",
    "/home/zrj/scout_liquid_lab/results",
    "/home/zrj/scout_liquid_lab/outgoing",
    "/usr/local/include",
    "/usr/**",
    "GenCase_linux64",
    "DualSPHysics5.4CPU_linux64 rix",
)


class GateError(RuntimeError):
    """A fail-closed preflight or postflight condition."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def sha256_file(path: Path, *, limit: int | None = None) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise GateError(f"unsafe non-regular or linked file: {path}")
        if limit is not None and metadata.st_size > limit:
            raise GateError(f"file exceeds bounded read size: {path}")
        digest = hashlib.sha256()
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            digest.update(block)
        return digest.hexdigest()
    finally:
        os.close(descriptor)


def read_json_object(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise GateError(f"JSON artifact missing or unsafe: {path}")
    raw = path.read_bytes()
    if len(raw) > 8 * 1024 * 1024:
        raise GateError(f"JSON artifact is too large: {path}")
    value = json.loads(raw.decode("utf-8"))
    if not isinstance(value, dict):
        raise GateError(f"JSON artifact is not an object: {path}")
    return value


def assert_no_symlink_components(path: Path, *, require_exists: bool) -> None:
    current = Path("/")
    parts = path.absolute().parts[1:]
    for index, part in enumerate(parts):
        current /= part
        if not current.exists() and index == len(parts) - 1 and not require_exists:
            continue
        try:
            info = os.lstat(current)
        except FileNotFoundError as exc:
            raise GateError(f"required path component is missing: {current}") from exc
        if stat.S_ISLNK(info.st_mode):
            raise GateError(f"symlink component is forbidden: {current}")


def assert_directory(path: Path, *, mode: int | None = None) -> os.stat_result:
    assert_no_symlink_components(path, require_exists=True)
    info = os.stat(path, follow_symlinks=False)
    if not stat.S_ISDIR(info.st_mode):
        raise GateError(f"not a directory: {path}")
    if mode is not None and stat.S_IMODE(info.st_mode) != mode:
        raise GateError(f"unexpected mode on {path}: {oct(stat.S_IMODE(info.st_mode))}")
    return info


def _source_entries_from_receipt() -> dict[str, dict[str, Any]]:
    if sha256_file(SOURCE_RECEIPT, limit=8 * 1024 * 1024) != SOURCE_RECEIPT_SHA256:
        raise GateError("materialization receipt SHA-256 differs")
    receipt = read_json_object(SOURCE_RECEIPT)
    if (
        receipt.get("status") != "PASS_STATIC_SOURCE_MATERIALIZATION"
        or receipt.get("source_materialized") is not True
        or receipt.get("upstream_code_executed") is not False
        or receipt.get("precompiled_binary_executed") is not False
        or receipt.get("cmake_or_make_executed") is not False
        or receipt.get("compiler_executed") is not False
    ):
        raise GateError("materialization receipt does not prove a sealed non-executed source tree")
    try:
        entries = receipt["results"]["materialization"]["sealed_output"]["entries"]
    except (KeyError, TypeError) as exc:
        raise GateError("materialization receipt has no sealed file manifest") from exc
    if not isinstance(entries, list) or len(entries) != SOURCE_FILE_COUNT:
        raise GateError("materialization receipt source file count differs")
    result: dict[str, dict[str, Any]] = {}
    total = 0
    for entry in entries:
        if not isinstance(entry, dict):
            raise GateError("source manifest entry is malformed")
        name = entry.get("path")
        if not isinstance(name, str) or not name.startswith("src/source/"):
            raise GateError("source manifest path leaves src/source")
        relative = name.removeprefix("src/source/")
        if not relative or relative.startswith("/") or ".." in Path(relative).parts:
            raise GateError("unsafe source manifest path")
        if relative in result:
            raise GateError("duplicate source manifest path")
        expected_sha = entry.get("sha256")
        expected_size = entry.get("size_bytes")
        if not isinstance(expected_sha, str) or len(expected_sha) != 64 or not isinstance(expected_size, int):
            raise GateError("source manifest hash or size is malformed")
        result[relative] = {"sha256": expected_sha, "size_bytes": expected_size}
        total += expected_size
    if total != SOURCE_TOTAL_BYTES:
        raise GateError("source manifest byte total differs")
    return result


def inventory_source_tree(root: Path, expected: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    assert_directory(root, mode=0o550)
    observed: dict[str, dict[str, Any]] = {}
    directories = 0
    for directory, subdirs, files in os.walk(root, topdown=True, followlinks=False):
        directory_path = Path(directory)
        directory_info = os.lstat(directory_path)
        if stat.S_ISLNK(directory_info.st_mode) or not stat.S_ISDIR(directory_info.st_mode):
            raise GateError(f"unsafe source directory: {directory_path}")
        if stat.S_IMODE(directory_info.st_mode) != 0o550:
            raise GateError(f"source directory is not sealed 0550: {directory_path}")
        directories += 1
        for name in [*subdirs, *files]:
            candidate = directory_path / name
            if stat.S_ISLNK(os.lstat(candidate).st_mode):
                raise GateError(f"source symlink is forbidden: {candidate}")
        for filename in files:
            path = directory_path / filename
            info = os.lstat(path)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise GateError(f"source file is unsafe: {path}")
            if stat.S_IMODE(info.st_mode) != 0o440 or info.st_mode & 0o111:
                raise GateError(f"source file is not sealed 0440: {path}")
            relative = str(path.relative_to(root))
            if relative not in expected:
                raise GateError(f"unexpected source file: {relative}")
            if info.st_size != expected[relative]["size_bytes"]:
                raise GateError(f"source byte count differs: {relative}")
            observed[relative] = {"sha256": sha256_file(path), "size_bytes": info.st_size}
    if set(observed) != set(expected):
        raise GateError("sealed source file set differs from receipt")
    for relative, facts in observed.items():
        if facts["sha256"] != expected[relative]["sha256"]:
            raise GateError(f"sealed source digest differs: {relative}")
    return {
        "root": str(root),
        "file_count": len(observed),
        "total_bytes": sum(item["size_bytes"] for item in observed.values()),
        "directories": directories,
        "manifest_sha256": canonical_hash(observed),
    }


def inventory_cloned_source(root: Path, expected: Mapping[str, Mapping[str, Any]], *, wrapper: bool) -> dict[str, Any]:
    assert_no_symlink_components(root, require_exists=True)
    if not root.is_dir() or root.is_symlink():
        raise GateError("cloned source root is unsafe")
    observed: dict[str, dict[str, Any]] = {}
    extras: list[str] = []
    for directory, subdirs, files in os.walk(root, topdown=True, followlinks=False):
        directory_path = Path(directory)
        info = os.lstat(directory_path)
        if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
            raise GateError(f"cloned source directory is unsafe: {directory_path}")
        for name in [*subdirs, *files]:
            candidate = directory_path / name
            candidate_info = os.lstat(candidate)
            if stat.S_ISLNK(candidate_info.st_mode):
                raise GateError(f"cloned source symlink is forbidden: {candidate}")
        for filename in files:
            path = directory_path / filename
            info = os.lstat(path)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise GateError(f"cloned source file is unsafe: {path}")
            relative = str(path.relative_to(root))
            if relative in expected:
                if info.st_size != expected[relative]["size_bytes"]:
                    raise GateError(f"cloned source byte count differs: {relative}")
                digest = sha256_file(path)
                if digest != expected[relative]["sha256"]:
                    raise GateError(f"cloned source digest differs: {relative}")
                observed[relative] = {"sha256": digest, "size_bytes": info.st_size}
            elif wrapper and relative == WRAPPER_NAME:
                if path.read_text(encoding="utf-8") != WRAPPER_CONTENT:
                    raise GateError("generated Make wrapper differs")
                extras.append(relative)
            else:
                raise GateError(f"unexpected cloned source file: {relative}")
    if set(observed) != set(expected):
        raise GateError("cloned source file set differs from source receipt")
    return {
        "root": str(root),
        "file_count": len(observed),
        "total_bytes": sum(item["size_bytes"] for item in observed.values()),
        "manifest_sha256": canonical_hash(observed),
        "allowed_extra_files": extras,
    }


def memory_available_bytes() -> int:
    fields: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        key, _, rest = line.partition(":")
        parts = rest.split()
        if len(parts) >= 2 and parts[1] == "kB":
            fields[key] = int(parts[0]) * 1024
    available = fields.get("MemAvailable")
    if available is None:
        raise GateError("MemAvailable is unavailable")
    return available


def host_userns_limit() -> int:
    """Read the initial-namespace userns limit without writing any sysctl.

    Bubblewrap's ``--disable-userns`` writes ``1`` only after it has cloned
    into its first user namespace.  The gate reads this host-visible node
    before and after the child, and treats any change as a hard failure rather
    than trying to restore a potentially global setting.
    """

    raw = HOST_USERNS_LIMIT_PATH.read_text(encoding="ascii").strip()
    if not raw.isdecimal():
        raise GateError("host user.max_user_namespaces is malformed")
    value = int(raw)
    if value < 2:
        raise GateError("host user.max_user_namespaces cannot admit the required two namespace levels")
    return value


def relevant_processes() -> list[dict[str, Any]]:
    needles = ("bwrap", "aa-exec", "DualSPHysics", "GenCase", "g++", "make")
    result: list[dict[str, Any]] = []
    own_pid = os.getpid()
    for candidate in Path("/proc").iterdir():
        if not candidate.name.isdecimal():
            continue
        pid = int(candidate.name)
        if pid == own_pid:
            continue
        try:
            comm = (candidate / "comm").read_text(encoding="utf-8").strip()
            cmdline = (candidate / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        combined = f"{comm} {cmdline}"
        if any(needle in combined for needle in needles):
            result.append({"pid": pid, "comm": comm, "cmdline": cmdline[:4096]})
    return result


def verify_tools() -> dict[str, Any]:
    observations: dict[str, Any] = {}
    for name, (path, expected_sha) in TRUSTED_TOOLS.items():
        if path.is_symlink() or not path.is_file():
            raise GateError(f"trusted tool is missing or unsafe: {path}")
        metadata = os.stat(path, follow_symlinks=False)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0:
            raise GateError(f"trusted tool ownership/type differs: {path}")
        if not metadata.st_mode & stat.S_IXUSR:
            raise GateError(f"trusted tool is not executable: {path}")
        digest = sha256_file(path, limit=512 * 1024 * 1024)
        if digest != expected_sha:
            raise GateError(f"trusted tool digest differs: {path}")
        observations[name] = {"path": str(path), "sha256": digest}
    return observations


def verify_review_artifacts() -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if set(policy) != set(schema.get("required", [])) or schema.get("additionalProperties") is not False:
        raise GateError("execution policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v10",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_CPU_BUILD_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_CPU_BUILD_EXECUTION_V10",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_source_copy_and_one_exact_cpu_build_attempt",
        "status": "REVIEWED_SINGLE_ATTEMPT_CPU_BUILD_V10_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "RUN_ONE_SOURCE_COPY_THEN_ONE_CPU_BUILD_AND_STATIC_ELF_AUDIT",
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            raise GateError(f"policy field differs: {key}")
    if policy.get("allowed_gate_commands") != ["self-check", "preflight", "run-source-copy", "run-build"]:
        raise GateError("policy command surface differs")
    handoff = policy.get("outer_privilege_handoff")
    if not isinstance(handoff, dict) or handoff.get("must_drop_before_aa_exec") is not True:
        raise GateError("policy does not require the pre-aa-exec privilege drop")
    if handoff.get("required_uid") != 1000 or handoff.get("required_gid") != 1000:
        raise GateError("policy privilege-drop identity differs")
    if handoff.get("required_supplementary_group_count") != 0:
        raise GateError("policy does not require empty supplementary groups")
    attempt = policy.get("frozen_attempt")
    if not isinstance(attempt, dict) or attempt.get("build_id") != BUILD_ID:
        raise GateError("policy build attempt differs")
    if attempt.get("attempt_root") != str(ATTEMPT_ROOT) or attempt.get("output_root") != str(OUTPUT_ROOT):
        raise GateError("policy output path differs")
    if attempt.get("source_copy_profile_sha256") != COPY_PROFILE_SHA256:
        raise GateError("policy source-copy profile digest differs")
    if attempt.get("build_profile_sha256") != BUILD_PROFILE_SHA256:
        raise GateError("policy build profile digest differs")
    if (
        attempt.get("source_copy_profile") != COPY_PROFILE
        or attempt.get("build_profile") != BUILD_PROFILE
        or attempt.get("source_copy_profile_path") != "config/apparmor_drafts/r8-liquid-u3-source-copy-v10.profile"
        or attempt.get("build_profile_path") != "config/apparmor_drafts/r8-liquid-u3-cpu-build-v10.profile"
    ):
        raise GateError("policy profile identity differs")
    source = policy.get("source_input")
    if not isinstance(source, dict) or source.get("root") != str(SOURCE_ROOT):
        raise GateError("policy source path differs")
    if source.get("receipt") != str(SOURCE_RECEIPT) or source.get("receipt_sha256") != SOURCE_RECEIPT_SHA256:
        raise GateError("policy source receipt differs")
    if source.get("file_count") != SOURCE_FILE_COUNT or source.get("total_bytes") != SOURCE_TOTAL_BYTES:
        raise GateError("policy source inventory differs")
    if policy.get("generated_wrapper", {}).get("content") != WRAPPER_CONTENT:
        raise GateError("policy generated wrapper differs")
    if policy.get("isolation", {}).get("synthetic_symlinks") != [{"link": "/lib64", "target": "usr/lib64"}]:
        raise GateError("policy synthetic-link boundary differs")
    if policy.get("isolation", {}).get("guest_proc_and_dev") != "absent":
        raise GateError("policy guest proc/dev boundary differs")
    if policy.get("isolation", {}).get("guest_tmpfs") != ["/work"]:
        raise GateError("policy guest tmpfs boundary differs")
    expected_disable_userns = {
        "bubblewrap_package_version": "0.9.0-1ubuntu0.1",
        "proc_path": "/proc/sys/user/max_user_namespaces",
        "open_mode": "O_WRONLY",
        "write_value": "1",
        "write_namespace": "first-level bubblewrap user namespace only",
        "kernel_write_capability": "CAP_SYS_RESOURCE in that first-level user namespace only",
        "host_initial_namespace_postcondition": "exact host value unchanged before and after the child",
    }
    if policy.get("isolation", {}).get("disable_userns_mechanism") != expected_disable_userns:
        raise GateError("policy disable-userns namespace-scoped write contract differs")
    if policy.get("invariants", {}).get("no_host_initial_namespace_sysctl_or_apparmor_global_setting_change") is not True:
        raise GateError("policy does not forbid host/global sysctl changes")
    if policy.get("isolation", {}).get("required_profile_capabilities") != [
        "sys_admin", "sys_ptrace", "sys_resource", "setpcap", "net_admin"
    ]:
        raise GateError("policy capability surface differs")
    if policy.get("build_command", [None])[0] != "/usr/bin/make" or "SHELL=/usr/bin/dash" not in policy.get("build_command", []):
        raise GateError("policy build executable differs")
    effective_profiles: dict[Path, str] = {}
    for profile_path, expected_sha in ((COPY_PROFILE_PATH, COPY_PROFILE_SHA256), (BUILD_PROFILE_PATH, BUILD_PROFILE_SHA256)):
        if sha256_file(profile_path, limit=128 * 1024) != expected_sha:
            raise GateError(f"reviewed profile digest differs: {profile_path}")
        text = profile_path.read_text(encoding="utf-8")
        if "REVIEWED_SINGLE_ATTEMPT_ONLY" not in text or "flags=(attach_disconnected,mediate_deleted)" not in text:
            raise GateError(f"reviewed profile header differs: {profile_path}")
        effective = "\n".join(line.split("#", 1)[0] for line in text.splitlines())
        for token in FORBIDDEN_PROFILE_TOKENS:
            if token in effective:
                raise GateError(f"reviewed profile violates a forbidden boundary: {token}")
        effective_profiles[profile_path] = effective
        proc_sys_writes = [
            line.strip()
            for line in effective.splitlines()
            if "/proc/sys" in line and " w" in line
        ]
        if proc_sys_writes != ["/proc/sys/user/max_user_namespaces w,"]:
            raise GateError("reviewed profile proc-sys write surface differs")
        capabilities = [line.strip() for line in effective.splitlines() if line.strip().startswith("capability ")]
        if capabilities != [
            "capability sys_admin,",
            "capability sys_ptrace,",
            "capability sys_resource,",
            "capability setpcap,",
            "capability net_admin,",
        ]:
            raise GateError("reviewed profile capability surface differs")
    build_rules = {line.strip() for line in effective_profiles[BUILD_PROFILE_PATH].splitlines() if line.strip()}
    required_header_rules = {"/usr/include/ r,", "/usr/include/** r,"}
    if not required_header_rules <= build_rules:
        raise GateError("v10 build profile lacks the exact read-only standard-header rules")
    if any(line.startswith("/usr/include") and line not in required_header_rules for line in build_rules):
        raise GateError("v10 build profile expands the standard-header access surface")
    if "/usr/include" in effective_profiles[COPY_PROFILE_PATH]:
        raise GateError("source-copy profile must not gain compiler header access")
    if "DualSPHysics5.4CPU_linux64 rix" in BUILD_PROFILE_PATH.read_text(encoding="utf-8"):
        raise GateError("build profile permits the output artifact")
    return {
        "policy_path": str(POLICY_PATH),
        "policy_sha256": sha256_file(POLICY_PATH, limit=256 * 1024),
        "policy_canonical_sha256": canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": sha256_file(SCHEMA_PATH, limit=256 * 1024),
        "source_copy_profile": {"path": str(COPY_PROFILE_PATH), "sha256": COPY_PROFILE_SHA256},
        "build_profile": {"path": str(BUILD_PROFILE_PATH), "sha256": BUILD_PROFILE_SHA256},
    }


def preflight(phase: str) -> dict[str, Any]:
    if phase not in {"source-copy", "build"}:
        raise GateError("unknown phase")
    if os.geteuid() == 0 or os.getuid() != os.geteuid():
        raise GateError("gate must run as the ordinary non-root liquid user")
    if os.getgroups():
        raise GateError("gate must begin after the reviewed zero-supplementary-group privilege drop")
    root_info = assert_directory(LIQUID_ROOT, mode=0o750)
    if root_info.st_uid != os.getuid() or root_info.st_gid != os.getgid():
        raise GateError("liquid root ownership differs from invoker")
    assert_directory(LIQUID_ROOT / "build", mode=0o750)
    assert_directory(LIQUID_ROOT / "audits", mode=0o750)
    review = verify_review_artifacts()
    tool_observations = verify_tools()
    expected_source = _source_entries_from_receipt()
    source_observations = inventory_source_tree(SOURCE_ROOT, expected_source)
    apparmor_restriction = Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns").read_text(encoding="ascii").strip()
    if apparmor_restriction != "1":
        raise GateError("unexpected user namespace policy; do not weaken or bypass AppArmor")
    host_limit_before = host_userns_limit()
    free_bytes = shutil.disk_usage(LIQUID_ROOT).free
    available_memory = memory_available_bytes()
    resources = read_json_object(POLICY_PATH)["resources"]
    if free_bytes < resources["minimum_available_disk_bytes"]:
        raise GateError("insufficient free disk for one isolated build attempt")
    if available_memory < resources["minimum_available_memory_bytes"]:
        raise GateError("insufficient available memory for the build reservation")
    busy = relevant_processes()
    if busy:
        raise GateError(f"conflicting build/sandbox/upstream processes are present: {busy}")
    if phase == "source-copy":
        for path in (ATTEMPT_ROOT, COPY_RECEIPT, COPY_RECEIPT.with_suffix(COPY_RECEIPT.suffix + ".partial"), BUILD_RECEIPT, BUILD_RECEIPT.with_suffix(BUILD_RECEIPT.suffix + ".partial")):
            if path.exists() or path.is_symlink():
                raise GateError(f"create-new path already exists: {path}")
    else:
        if not ATTEMPT_ROOT.is_dir() or not OUTPUT_ROOT.is_dir() or not COPY_RECEIPT.is_file():
            raise GateError("source-copy success evidence and output root are required before build")
        if BUILD_RECEIPT.exists() or BUILD_RECEIPT.with_suffix(BUILD_RECEIPT.suffix + ".partial").exists():
            raise GateError("build receipt path already exists")
        copy_receipt = read_json_object(COPY_RECEIPT)
        if copy_receipt.get("status") != "PASS_U3_SOURCE_COPY" or copy_receipt.get("build_id") != BUILD_ID:
            raise GateError("source-copy receipt does not admit the build phase")
        clone_observations = inventory_cloned_source(OUTPUT_SOURCE, expected_source, wrapper=True)
        if clone_observations["allowed_extra_files"] != [WRAPPER_NAME]:
            raise GateError("build wrapper evidence is missing")
    return {
        "captured_at_utc": utc_now(),
        "phase": phase,
        "build_id": BUILD_ID,
        "invoker": {"uid": os.getuid(), "gid": os.getgid(), "groups": os.getgroups()},
        "apparmor_restrict_unprivileged_userns": int(apparmor_restriction),
        "host_initial_namespace_user_max_user_namespaces_before": host_limit_before,
        "disk_free_bytes": free_bytes,
        "memory_available_bytes": available_memory,
        "review": review,
        "trusted_tools": tool_observations,
        "sealed_source": source_observations,
        **({"cloned_source": clone_observations} if phase == "build" else {}),
    }


def host_userns_postcondition(preflight_data: Mapping[str, Any]) -> dict[str, Any]:
    """Report the host-visible userns limit after the exact child exits."""

    before = preflight_data.get("host_initial_namespace_user_max_user_namespaces_before")
    if not isinstance(before, int) or before < 2:
        raise GateError("preflight host userns limit evidence is malformed")
    after = host_userns_limit()
    return {"before": before, "after": after, "unchanged": after == before}


def write_json_new(path: Path, value: Mapping[str, Any]) -> None:
    assert_no_symlink_components(path.parent, require_exists=True)
    serialized = json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n"
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o640)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(serialized)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise


def create_attempt_tree() -> None:
    assert_no_symlink_components(ATTEMPT_ROOT.parent, require_exists=True)
    try:
        os.mkdir(ATTEMPT_ROOT, 0o750)
        os.mkdir(OUTPUT_ROOT, 0o700)
        os.mkdir(OUTPUT_ROOT / "buildtree", 0o700)
        os.mkdir(OUTPUT_ROOT / "buildtree" / "src", 0o700)
        os.mkdir(OUTPUT_SOURCE, 0o700)
        os.mkdir(OUTPUT_ROOT / "artifacts", 0o700)
    except FileExistsError as exc:
        raise GateError("attempt output path unexpectedly already exists") from exc


def bwrap_prefix(*, phase: str) -> list[str]:
    if phase == "source-copy":
        profile = COPY_PROFILE
        timeout_seconds = "300s"
        address_space = "1073741824:1073741824"
        cpu_limit = "120:125"
        child = [
            "/usr/bin/env", "-i", "HOME=/nonexistent", "PATH=/usr/bin", "LC_ALL=C.UTF-8", "LANG=C.UTF-8", "TZ=UTC",
            "/usr/bin/prlimit", f"--as={address_space}", f"--cpu={cpu_limit}", "--nproc=4096:4096", "--nofile=1024:1024", "--fsize=2147483648:2147483648", "--core=0:0", "--",
            "/usr/bin/cp", "--recursive", "--no-preserve=mode,ownership", "--", "/work/input/.", "/work/output/buildtree/src/source/",
        ]
        extra_binds = ["--ro-bind", str(SOURCE_ROOT), "/work/input"]
        chdir = "/work"
        hostname = "r8-liquid-u3-source-copy"
    elif phase == "build":
        profile = BUILD_PROFILE
        timeout_seconds = "3600s"
        address_space = "8589934592:8589934592"
        cpu_limit = "3600:3605"
        child = [
            "/usr/bin/env", "-i", "HOME=/nonexistent", "PATH=/usr/bin", "LC_ALL=C.UTF-8", "LANG=C.UTF-8", "TZ=UTC",
            "/usr/bin/prlimit", f"--as={address_space}", f"--cpu={cpu_limit}", "--nproc=4096:4096", "--nofile=1024:1024", "--fsize=2147483648:2147483648", "--core=0:0", "--",
            "/usr/bin/make", "--no-builtin-rules", "--no-builtin-variables", "-f", WRAPPER_NAME, "-j4", "SHELL=/usr/bin/dash",
            "CC=/usr/bin/x86_64-linux-gnu-g++-13", "USE_DEBUG=NO", "USE_FAST_MATH=NO", "USE_NATIVE_CPU_OPTIMIZATIONS=NO",
            "COMPILE_CHRONO=NO", "COMPILE_WAVEGEN=NO", "COMPILE_MOORDYNPLUS=NO", "LIBS_DIRECTORIES=",
            "EXECS_DIRECTORY=/work/output/artifacts", "/work/output/artifacts/DualSPHysics5.4CPU_linux64",
        ]
        extra_binds = []
        chdir = "/work/output/buildtree/src/source"
        hostname = "r8-liquid-u3-cpu-build"
    else:
        raise GateError("unknown bwrap phase")
    return [
        "/usr/bin/timeout", "--foreground", "--kill-after=5s", timeout_seconds,
        "/usr/bin/aa-exec", "-p", profile, "--",
        "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled", "--hostname", hostname, "--clearenv",
        "--setenv", "HOME", "/nonexistent", "--setenv", "PATH", "/usr/bin", "--setenv", "LC_ALL", "C.UTF-8", "--setenv", "LANG", "C.UTF-8", "--setenv", "TZ", "UTC",
        "--uid", "0", "--gid", "0", "--ro-bind", "/usr", "/usr", "--symlink", "usr/lib64", "/lib64",
        "--tmpfs", "/work",
        *extra_binds,
        "--bind", str(OUTPUT_ROOT), "/work/output", "--chdir", chdir, "--", *child,
    ]


def run_fixed_argv(argv: list[str], output_limit: int = 262_144) -> dict[str, Any]:
    started = time.monotonic()
    process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    if process.stdout is None:
        raise GateError("cannot capture bounded child output")
    digest = hashlib.sha256()
    captured = bytearray()
    total = 0
    while True:
        block = process.stdout.read(65536)
        if not block:
            break
        total += len(block)
        digest.update(block)
        if len(captured) < output_limit:
            captured.extend(block[: output_limit - len(captured)])
    returncode = process.wait()
    return {
        "argv": argv,
        "returncode": returncode,
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "combined_output_bytes": total,
        "combined_output_sha256": digest.hexdigest(),
        "combined_output_truncated": total > len(captured),
        "combined_output_utf8_prefix": bytes(captured).decode("utf-8", "replace"),
    }


def create_wrapper() -> dict[str, Any]:
    path = OUTPUT_SOURCE / WRAPPER_NAME
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(WRAPPER_CONTENT)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise
    info = os.lstat(path)
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or stat.S_IMODE(info.st_mode) != 0o600:
        raise GateError("generated Make wrapper has unsafe metadata")
    return {"path": str(path), "sha256": sha256_file(path), "mode_octal": oct(stat.S_IMODE(info.st_mode))}


def read_exact(path: Path, offset: int, size: int, descriptor: int) -> bytes:
    if offset < 0 or size < 0 or offset + size > os.fstat(descriptor).st_size:
        raise GateError(f"ELF read leaves file bounds: {path}")
    data = os.pread(descriptor, size, offset)
    if len(data) != size:
        raise GateError(f"short ELF read: {path}")
    return data


def c_string(table: bytes, offset: int) -> str:
    if offset < 0 or offset >= len(table):
        raise GateError("ELF dynamic string offset leaves string table")
    terminal = table.find(b"\0", offset)
    if terminal < 0:
        raise GateError("ELF dynamic string is unterminated")
    return table[offset:terminal].decode("utf-8", "strict")


def static_elf_audit(path: Path) -> dict[str, Any]:
    import struct

    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size < 64 or info.st_size > 512 * 1024 * 1024:
            raise GateError("candidate artifact metadata is unsafe")
        header = read_exact(path, 0, 64, descriptor)
        if header[:4] != b"\x7fELF" or header[4] != 2 or header[5] != 1 or header[6] != 1:
            raise GateError("candidate artifact is not a current little-endian ELF64")
        values = struct.unpack("<16sHHIQQQIHHHHHH", header)
        e_type, e_machine, e_phoff, e_phentsize, e_phnum = values[1], values[2], values[5], values[9], values[10]
        if e_type != 2 or e_machine != 62 or e_phentsize != 56 or not 1 <= e_phnum <= 64:
            raise GateError("candidate artifact ELF header differs from a native executable")
        program_headers: list[tuple[int, int, int, int, int, int]] = []
        interpreter: str | None = None
        dynamic_region: tuple[int, int] | None = None
        stack_executable = False
        for index in range(e_phnum):
            entry = read_exact(path, e_phoff + index * e_phentsize, e_phentsize, descriptor)
            p_type, p_flags, p_offset, p_vaddr, _paddr, p_filesz, _p_memsz, _p_align = struct.unpack("<IIQQQQQQ", entry)
            if p_offset + p_filesz > info.st_size:
                raise GateError("candidate artifact program header leaves file bounds")
            program_headers.append((p_type, p_flags, p_offset, p_vaddr, p_filesz, _p_memsz))
            if p_type == 3:
                if interpreter is not None or not 1 <= p_filesz <= 4096:
                    raise GateError("candidate artifact has an unsafe interpreter header")
                raw = read_exact(path, p_offset, p_filesz, descriptor)
                if not raw.endswith(b"\0"):
                    raise GateError("candidate artifact interpreter is unterminated")
                interpreter = raw[:-1].decode("utf-8", "strict")
            elif p_type == 2:
                if dynamic_region is not None or p_filesz > 65536 or p_filesz % 16:
                    raise GateError("candidate artifact dynamic section is unsafe")
                dynamic_region = (p_offset, p_filesz)
            elif p_type == 0x6474E551:
                stack_executable = bool(p_flags & 1)
        if interpreter != "/lib64/ld-linux-x86-64.so.2" or dynamic_region is None or stack_executable:
            raise GateError("candidate artifact interpreter/dynamic/stack policy differs")
        dynamic = read_exact(path, dynamic_region[0], dynamic_region[1], descriptor)
        string_address: int | None = None
        string_size: int | None = None
        needed_offsets: list[int] = []
        rpath_seen = False
        runpath_seen = False
        for offset in range(0, len(dynamic), 16):
            tag, value = struct.unpack("<QQ", dynamic[offset : offset + 16])
            if tag == 0:
                break
            if tag == 1:
                needed_offsets.append(value)
            elif tag == 5:
                string_address = value
            elif tag == 10:
                string_size = value
            elif tag == 15:
                rpath_seen = True
            elif tag == 29:
                runpath_seen = True
        if string_address is None or string_size is None or not 1 <= string_size <= 1_048_576:
            raise GateError("candidate artifact dynamic string table is unsafe")
        string_offset: int | None = None
        for p_type, _flags, p_offset, p_vaddr, p_filesz, _memsz in program_headers:
            if p_type == 1 and p_vaddr <= string_address < p_vaddr + p_filesz:
                string_offset = p_offset + string_address - p_vaddr
                break
        if string_offset is None:
            raise GateError("candidate artifact dynamic string table is unmapped")
        strings = read_exact(path, string_offset, string_size, descriptor)
        needed = [c_string(strings, value) for value in needed_offsets]
        required = {"libgomp.so.1", "libstdc++.so.6", "libgcc_s.so.1", "libc.so.6"}
        forbidden = ("chrono", "dsph", "cuda", "nvidia", "ros", "gazebo", "wavegen", "moordyn")
        if not required.issubset(needed) or rpath_seen or runpath_seen:
            raise GateError("candidate artifact dynamic dependency policy differs")
        if any(token in soname.lower() for soname in needed for token in forbidden):
            raise GateError("candidate artifact needs a forbidden library")
        return {
            "sha256": sha256_file(path, limit=512 * 1024 * 1024),
            "size_bytes": info.st_size,
            "interpreter": interpreter,
            "needed": needed,
            "rpath": rpath_seen,
            "runpath": runpath_seen,
            "gnu_stack_executable": stack_executable,
        }
    finally:
        os.close(descriptor)


def inventory_build_output(expected: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    assert_directory(OUTPUT_ROOT, mode=0o700)
    copied = inventory_cloned_source(OUTPUT_SOURCE, expected, wrapper=True)
    unexpected: list[str] = []
    objects: list[dict[str, Any]] = []
    total = 0
    for directory, subdirs, files in os.walk(OUTPUT_ROOT, topdown=True, followlinks=False):
        directory_path = Path(directory)
        info = os.lstat(directory_path)
        if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
            raise GateError(f"unsafe build output directory: {directory_path}")
        for name in [*subdirs, *files]:
            candidate = directory_path / name
            candidate_info = os.lstat(candidate)
            if stat.S_ISLNK(candidate_info.st_mode) or candidate_info.st_nlink != 1:
                raise GateError(f"unsafe build output entry: {candidate}")
        for name in files:
            path = directory_path / name
            relative = path.relative_to(OUTPUT_ROOT)
            info = os.lstat(path)
            total += info.st_size
            if path == ARTIFACT_PATH:
                continue
            if relative.parts[:3] == ("buildtree", "src", "source"):
                nested = Path(*relative.parts[3:])
                if str(nested) in expected or str(nested) == WRAPPER_NAME:
                    continue
                if len(nested.parts) == 1 and nested.suffix == ".o":
                    objects.append({"path": str(relative), "size_bytes": info.st_size, "sha256": sha256_file(path)})
                    continue
            unexpected.append(str(relative))
    if unexpected:
        raise GateError(f"unexpected build output files: {unexpected}")
    if total > 2 * 1024 * 1024 * 1024:
        raise GateError("build output exceeds the frozen two-GiB ceiling")
    return {"cloned_source": copied, "object_count": len(objects), "objects": objects, "total_bytes": total}


def execute_source_copy() -> int:
    phase = "source-copy"
    try:
        preflight_data = preflight(phase)
    except GateError as exc:
        print(json.dumps({"status": "NO_GO_PREFLIGHT", "phase": phase, "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2
    create_attempt_tree()
    partial = COPY_RECEIPT.with_suffix(COPY_RECEIPT.suffix + ".partial")
    write_json_new(partial, {"status": "STARTED_U3_SOURCE_COPY", "phase": phase, "build_id": BUILD_ID, "started_at_utc": utc_now(), "preflight": preflight_data})
    result = run_fixed_argv(bwrap_prefix(phase=phase))
    final: dict[str, Any] = {"document_type": "SMPCC_R8_LIQUID_U3_SOURCE_COPY_RECEIPT", "build_id": BUILD_ID, "created_at_utc": utc_now(), "phase": phase, "preflight": preflight_data, "execution": result, "partial_start_record": str(partial), "upstream_code_executed": False, "precompiled_binary_executed": False, "compiler_executed": False, "network_used": False, "gpu_device_exposed": False}
    try:
        final["host_initial_namespace_user_max_user_namespaces"] = host_userns_postcondition(preflight_data)
    except GateError as exc:
        final.update({"status": "SOURCE_COPY_HOST_SYSCTL_POSTCONDITION_NO_GO", "postflight_error": str(exc), "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        write_json_new(COPY_RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(COPY_RECEIPT)}, ensure_ascii=False))
        return 5
    if final["host_initial_namespace_user_max_user_namespaces"]["unchanged"] is not True:
        final.update({"status": "SOURCE_COPY_HOST_SYSCTL_POSTCONDITION_NO_GO", "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        write_json_new(COPY_RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(COPY_RECEIPT)}, ensure_ascii=False))
        return 5
    if result["returncode"] != 0:
        final.update({"status": "SOURCE_COPY_FAILED_NO_RETRY", "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        write_json_new(COPY_RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(COPY_RECEIPT)}, ensure_ascii=False))
        return 3
    try:
        expected = _source_entries_from_receipt()
        copied = inventory_cloned_source(OUTPUT_SOURCE, expected, wrapper=False)
        wrapper = create_wrapper()
        final.update({"status": "PASS_U3_SOURCE_COPY", "copied_source": copied, "generated_wrapper": wrapper, "next_allowed_stage": "TRANSIENT_BUILD_PROFILE_LOAD_THEN_RUN_BUILD"})
    except (GateError, OSError, UnicodeError) as exc:
        final.update({"status": "SOURCE_COPY_POSTFLIGHT_NO_GO", "postflight_error": str(exc), "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        write_json_new(COPY_RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(COPY_RECEIPT)}, ensure_ascii=False))
        return 4
    write_json_new(COPY_RECEIPT, final)
    print(json.dumps({"status": final["status"], "receipt": str(COPY_RECEIPT)}, ensure_ascii=False))
    return 0


def execute_build() -> int:
    phase = "build"
    try:
        preflight_data = preflight(phase)
    except GateError as exc:
        print(json.dumps({"status": "NO_GO_PREFLIGHT", "phase": phase, "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2
    partial = BUILD_RECEIPT.with_suffix(BUILD_RECEIPT.suffix + ".partial")
    write_json_new(partial, {"status": "STARTED_U3_CPU_BUILD", "phase": phase, "build_id": BUILD_ID, "started_at_utc": utc_now(), "preflight": preflight_data})
    result = run_fixed_argv(bwrap_prefix(phase=phase))
    final: dict[str, Any] = {"document_type": "SMPCC_R8_LIQUID_U3_CPU_BUILD_RECEIPT", "build_id": BUILD_ID, "created_at_utc": utc_now(), "phase": phase, "preflight": preflight_data, "execution": result, "partial_start_record": str(partial), "upstream_code_executed": False, "precompiled_binary_executed": False, "compiled_artifact_executed": False, "network_used": False, "gpu_device_exposed": False}
    try:
        final["host_initial_namespace_user_max_user_namespaces"] = host_userns_postcondition(preflight_data)
    except GateError as exc:
        final.update({"status": "CPU_BUILD_HOST_SYSCTL_POSTCONDITION_NO_GO", "postflight_error": str(exc), "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        write_json_new(BUILD_RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
        return 5
    if final["host_initial_namespace_user_max_user_namespaces"]["unchanged"] is not True:
        final.update({"status": "CPU_BUILD_HOST_SYSCTL_POSTCONDITION_NO_GO", "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        write_json_new(BUILD_RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
        return 5
    if result["returncode"] != 0:
        final.update({"status": "CPU_BUILD_FAILED_NO_RETRY", "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        write_json_new(BUILD_RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
        return 3
    try:
        expected = _source_entries_from_receipt()
        if not ARTIFACT_PATH.exists() or ARTIFACT_PATH.is_symlink():
            raise GateError("candidate CPU artifact is absent")
        audit = static_elf_audit(ARTIFACT_PATH)
        os.chmod(ARTIFACT_PATH, 0o400, follow_symlinks=False)
        final_mode = stat.S_IMODE(os.lstat(ARTIFACT_PATH).st_mode)
        if final_mode != 0o400:
            raise GateError("candidate artifact could not be disabled at rest")
        output = inventory_build_output(expected)
        final.update({"status": "PASS_U3_CPU_BUILD_STATIC_ELF_AUDIT", "artifact": {**audit, "path": str(ARTIFACT_PATH), "mode_after_audit": oct(final_mode)}, "output_inventory": output, "next_allowed_stage": "SEPARATE_GENCASE_AND_SOLVER_EXECUTION_ADMISSION_REQUIRED"})
    except (GateError, OSError, UnicodeError, ValueError) as exc:
        final.update({"status": "CPU_BUILD_POSTFLIGHT_NO_GO", "postflight_error": str(exc), "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        write_json_new(BUILD_RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
        return 4
    write_json_new(BUILD_RECEIPT, final)
    print(json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
    return 0


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "preflight", "run-source-copy", "run-build"))
    parser.add_argument("--phase", choices=("source-copy", "build"), help="required only with preflight")
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        if args.command == "self-check":
            expected = _source_entries_from_receipt()
            report = {"status": "PASS_U3_CPU_BUILD_EXECUTION_STATIC_REVIEW", "review": verify_review_artifacts(), "trusted_tools": verify_tools(), "sealed_source": inventory_source_tree(SOURCE_ROOT, expected), "gate_sha256": sha256_file(Path(__file__))}
            print(json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2))
            return 0
        if args.command == "preflight":
            if args.phase is None:
                parser.error("--phase is required with preflight")
            print(json.dumps(preflight(args.phase), ensure_ascii=False, sort_keys=True, indent=2))
            return 0
        if args.command == "run-source-copy":
            return execute_source_copy()
        if args.command == "run-build":
            return execute_build()
    except (GateError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "NO_GO_STATIC_OR_RUNTIME_CHECK", "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2
    raise AssertionError("unreachable")


if __name__ == "__main__":
    raise SystemExit(main())
