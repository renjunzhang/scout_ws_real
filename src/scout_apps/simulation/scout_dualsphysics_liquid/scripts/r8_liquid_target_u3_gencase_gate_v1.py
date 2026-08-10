#!/usr/bin/env python3
"""One-shot, fail-closed GenCase runtime gate for the U3 C1 smoke case.

The host copy of GenCase remains mode 0400.  A fixed bwrap command copies it to
guest tmpfs, enables it only there, and AppArmor requires an exec transition
from a bootstrap label into a runtime label with no mount, userns, capability,
network, host-path or system-tool execution surface.  This gate never runs the
locally compiled solver candidate.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import stat
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
SEED_GATE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_gencase_seed_gate_v1.py")
SEED_GATE_SHA256 = "44cc56c580829efe2bf78d1e6ade102fe3dceee77faee66febc307e34b275009"

LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
CASE_ID = "u3_c1_gencase_20260807T032831Z"
ATTEMPT_ROOT = LIQUID_ROOT / "cases" / f"{CASE_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
RECEIPT = LIQUID_ROOT / "audits" / f"{CASE_ID}.json"
PARTIAL_RECEIPT = RECEIPT.with_suffix(RECEIPT.suffix + ".partial")
CONSOLE_LOG = LIQUID_ROOT / "audits" / f"{CASE_ID}.console.log"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_execution_policy_v1.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gencase_execution_policy_v1.json"
PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-gencase-v1.profile"
PROFILE_SHA256 = "32465fecbaf25b181845d12fba776e1b564f0b6ba908f3db7313d00a4a7eb5c3"
BOOTSTRAP_PROFILE = "r8-liquid-u3-gencase-bootstrap-v1"
RUNTIME_PROFILE = "r8-liquid-u3-gencase-runtime-v1"

SEED_ROOT = LIQUID_ROOT / "dependency/runtime/u3_c1_gencase_seed_20260807T031624Z.partial"
INPUT_ROOT = SEED_ROOT / "input"
SEED_RECEIPT = LIQUID_ROOT / "audits/u3_c1_gencase_seed_20260807T031624Z.json"
SEED_RECEIPT_SHA256 = "19ccbcc3c945a3e457b9cce99611c061ac8e6dfaf099129920cb555818fd6b6a"
ELF_RECEIPT = LIQUID_ROOT / "audits/u2_elf_metadata_v1_20260806T082300Z.json"
ELF_RECEIPT_SHA256 = "47e411b391afd677fc4d1b60ca6547c73545c287ef080bea92c2e66128ef6a74"
HOST_USERNS_LIMIT = Path("/proc/sys/user/max_user_namespaces")
HOST_UID = 1000
HOST_GID = 1000

GENCASE_SHA256 = "a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226"
GENCASE_SIZE = 5_809_384
DSPH_CONFIG_SHA256 = "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd"
DSPH_CONFIG_SIZE = 293
CASE_DEFINITION_SHA256 = "ef57582f69714f622e5b8a348c7626d233c203ad0b1eb6332b58fb1ffc09fa1a"

WALL_TIMEOUT_SECONDS = 120
CPU_SECONDS = 90
ADDRESS_SPACE_BYTES = 1_073_741_824
PROCESS_LIMIT = 8
OPEN_FILE_LIMIT = 256
FILE_SIZE_LIMIT = 16_777_216
CONSOLE_LOG_LIMIT = 1_048_576
MINIMUM_MEMORY_BYTES = 2_147_483_648
MINIMUM_DISK_BYTES = 1_073_741_824

SETUP_SHELL = (
    "umask 077; "
    "/usr/bin/cp -- /work/input/GenCase_linux64 /runtime/GenCase_linux64; "
    "/usr/bin/cp -- /work/input/DsphConfig.xml /runtime/DsphConfig.xml; "
    "/usr/bin/chmod 0500 /runtime/GenCase_linux64; "
    "/usr/bin/chmod 0400 /runtime/DsphConfig.xml; "
    "exec /runtime/GenCase_linux64 /work/input/C1_static_Def /work/output/C1_static "
    "-dp:0.002 -threads:1 -save:-all,+bi -createdirs:1"
)
GENCASE_ARGV = [
    "/runtime/GenCase_linux64",
    "/work/input/C1_static_Def",
    "/work/output/C1_static",
    "-dp:0.002",
    "-threads:1",
    "-save:-all,+bi",
    "-createdirs:1",
]

EXPECTED_OUTPUTS: dict[str, tuple[int, int]] = {
    "C1_static.bi4": (1, FILE_SIZE_LIMIT),
    "C1_static.xml": (1, 524_288),
}

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "apparmor_parser": (Path("/usr/sbin/apparmor_parser"), "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "chmod": (Path("/usr/bin/chmod"), "f7ba478ce7ff158ccd3ba01bac1c4231e68cbd490fe49b5562a6ea008a586696"),
    "cp": (Path("/usr/bin/cp"), "6187dab32cd7b6291d96c54425a22915a6eeb39add6d95ee9008e1dbba427131"),
    "dash": (Path("/usr/bin/dash"), "86d31f6fb799e91fa21bad341484564510ca287703a16e9e46c53338776f4f42"),
    "env": (Path("/usr/bin/env"), "0aefff8f912fb75716c5d4de3b6acde93edbe8fa280fc8ee895c1226d3e373ef"),
    "prlimit": (Path("/usr/bin/prlimit"), "f27cfd8c1512a4cc6541b59b80cb4cdfd6ef28c34aa21db4299b48264cd0d128"),
    "setpriv": (Path("/usr/bin/setpriv"), "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}

SECTION_HASHES = {
    "frozen_attempt": "1821606a07592fbbbca5fc6faf2c0c3b0d483245a8aff9c079356e1ed247057b",
    "outer_privilege_handoff": "d1e13fadcbb23b3991f078b0c97f5e400cc260b95874c705962f00209290d5b0",
    "input_provenance": "714dca1223aa6321e12f0d6db6bf3491f2f2c3437e75bb0f8d7a8c3f723f94bd",
    "trusted_tools": "3abdf5266fd0c1a9b929f894376197dfac478c38757c08278f65abb96d6078d6",
    "isolation": "36edb01192fb1da0ccfc0dda5a98bcd1702b2a6c7da152a33186a366e156b4ea",
    "resources": "e3ca4a23f7ccc110d75112243da6d735bc186a0c621ce5262a6789a5cc0d630b",
    "fixed_guest_command": "43b9b1335b5b7487042c01ff588d13172581afa4373925bcdfe64dd1e843e9cc",
    "output_contract": "1f37897b6b2458c6e01e615efe7bc8da0cde6cdb7e99b1624b4b4623f539f42e",
    "profile_lifecycle": "c1c277f8b181f13ba28f0babc7786cbc3dd8caacbb359b666bce0c84836440ae",
    "invariants": "350f83621e30f2156057a940a1b99643e2fcead4327c58dfe0f06923ed1e1fb5",
}

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A fail-closed preflight or postflight condition."""


def _sha256_for_import(path: Path) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > 2 * 1024 * 1024:
            raise GateError(f"unsafe frozen helper layer: {path}")
        digest = hashlib.sha256()
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            digest.update(block)
        return digest.hexdigest()
    finally:
        os.close(descriptor)


def load_seed_core() -> ModuleType:
    if _sha256_for_import(SEED_GATE_PATH) != SEED_GATE_SHA256:
        raise GateError("the byte-pinned seed helper layer differs")
    specification = importlib.util.spec_from_file_location("r8_liquid_u3_gencase_seed_v1_frozen", SEED_GATE_PATH)
    if specification is None or specification.loader is None:
        raise GateError("cannot load the frozen seed helper layer")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


core = load_seed_core()


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def read_host_userns_limit() -> int:
    raw = HOST_USERNS_LIMIT.read_text(encoding="ascii").strip()
    if not raw.isdecimal():
        raise GateError("host user.max_user_namespaces is malformed")
    value = int(raw)
    if value < 2:
        raise GateError("host user.max_user_namespaces cannot admit the reviewed namespace depth")
    return value


def available_memory_bytes() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        if line.startswith("MemAvailable:"):
            parts = line.split()
            if len(parts) == 3 and parts[2] == "kB" and parts[1].isdecimal():
                return int(parts[1]) * 1024
    raise GateError("MemAvailable is unavailable")


def available_disk_bytes() -> int:
    facts = os.statvfs(LIQUID_ROOT)
    return facts.f_bavail * facts.f_frsize


def require_absent(path: Path) -> None:
    current = Path("/")
    for part in path.absolute().parts[1:]:
        current /= part
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            break
        if stat.S_ISLNK(metadata.st_mode):
            raise GateError(f"symlink path component is forbidden: {current}")
    try:
        os.lstat(path)
    except FileNotFoundError:
        return
    raise GateError(f"one-shot path already exists: {path}")


def mkdir_new(path: Path, mode: int) -> None:
    core.assert_no_symlink_components(path.parent, require_exists=True)
    try:
        os.mkdir(path, mode)
    except FileExistsError as exc:
        raise GateError(f"new attempt directory already exists: {path}") from exc
    metadata = os.lstat(path)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != mode:
        raise GateError(f"new attempt directory type/mode differs: {path}")


def require_tool(name: str) -> dict[str, Any]:
    path, expected_sha = TRUSTED_TOOLS[name]
    if path.is_symlink() or not path.is_file():
        raise GateError(f"trusted tool is missing or symlinked: {path}")
    metadata = os.stat(path, follow_symlinks=False)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0:
        raise GateError(f"trusted tool ownership/type differs: {path}")
    if not metadata.st_mode & stat.S_IXUSR:
        raise GateError(f"trusted tool is not executable: {path}")
    digest = core.sha256_regular_file(path, limit=512 * 1024 * 1024)
    if digest != expected_sha:
        raise GateError(f"trusted tool digest differs: {path}")
    return {"path": str(path), "sha256": digest}


def trusted_tool_policy() -> dict[str, dict[str, str]]:
    return {name: {"path": str(path), "sha256": digest} for name, (path, digest) in TRUSTED_TOOLS.items()}


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def verify_profile() -> dict[str, Any]:
    digest = core.sha256_regular_file(PROFILE_PATH, limit=128 * 1024)
    if digest != PROFILE_SHA256:
        raise GateError("reviewed GenCase AppArmor profile digest differs")
    text = core.read_regular_bytes(PROFILE_PATH, limit=128 * 1024).decode("utf-8")
    effective = _effective_profile(text)
    bootstrap_header = f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted)"
    runtime_header = f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted)"
    if effective.count(bootstrap_header) != 1 or effective.count(runtime_header) != 1:
        raise GateError("reviewed AppArmor labels differ")
    transition = f"/runtime/GenCase_linux64 rPx -> {RUNTIME_PROFILE},"
    if effective.count(transition) != 1:
        raise GateError("mandatory GenCase profile transition differs")
    runtime_text = effective.split(runtime_header, 1)[1]
    for forbidden in (
        "userns ",
        "mount ",
        "remount ",
        "pivot_root ",
        "umount ",
        "capability ",
        "network ",
        "/proc",
        "/dev",
        "/home/",
        "/usr/bin/",
        " rix,",
        " rPx",
        " ix,",
    ):
        if forbidden in runtime_text:
            raise GateError(f"runtime label contains forbidden authority: {forbidden}")
    for required in (
        "/runtime/GenCase_linux64 rm,",
        "/runtime/DsphConfig.xml r,",
        "/work/input/** r,",
        "/work/output/** rw,",
        "/usr/lib/** mr,",
    ):
        if required not in runtime_text:
            raise GateError(f"runtime label lacks required narrow rule: {required}")
    for forbidden_global in (
        "flags=(unconfined)",
        "/home/zrj/scout_ws",
        "/home/zrj/ros2_ws",
        "/opt/ros/jazzy",
        "/home/zrj/scout_liquid_lab/build",
        "/home/zrj/scout_liquid_lab/dependency/source",
        "/dev/nvidia",
        "change_profile",
        "ux,",
        "Ux,",
    ):
        if forbidden_global in effective:
            raise GateError(f"reviewed profile violates a forbidden boundary: {forbidden_global}")
    parser_tool = require_tool("apparmor_parser")
    parsed = subprocess.run(
        [str(TRUSTED_TOOLS["apparmor_parser"][0]), "-Q", "-K", "-T", str(PROFILE_PATH)],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env={"PATH": "/usr/bin:/usr/sbin", "HOME": "/nonexistent", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"},
        timeout=15,
        check=False,
    )
    if parsed.returncode != 0 or len(parsed.stdout) > 16_384 or len(parsed.stderr) > 16_384:
        raise GateError("AppArmor parser static check failed")
    return {
        "path": str(PROFILE_PATH),
        "sha256": digest,
        "parser": parser_tool,
        "parser_returncode": parsed.returncode,
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
        "runtime_transition": transition,
    }


def verify_seed_receipt_and_input() -> dict[str, Any]:
    if core.sha256_regular_file(SEED_RECEIPT) != SEED_RECEIPT_SHA256:
        raise GateError("GenCase seed receipt digest differs")
    receipt = core.read_json_object(SEED_RECEIPT)
    if receipt.get("status") != "PASS_NONEXECUTABLE_GENCASE_SEED_MATERIALIZATION":
        raise GateError("GenCase seed receipt is not PASS")
    for field in (
        "upstream_code_executed",
        "precompiled_binary_executed",
        "compiled_artifact_executed",
        "network_used",
        "gpu_device_exposed",
        "source_checkout_created",
        "system_packages_changed",
        "sudo_used",
    ):
        if receipt.get(field) is not False:
            raise GateError(f"GenCase seed receipt violates the non-execution boundary: {field}")
    core.assert_directory(SEED_ROOT, mode=0o500)
    core.assert_directory(INPUT_ROOT, mode=0o500)
    expected = {
        "GenCase_linux64": (GENCASE_SHA256, GENCASE_SIZE),
        "DsphConfig.xml": (DSPH_CONFIG_SHA256, DSPH_CONFIG_SIZE),
        "C1_static_Def.xml": (CASE_DEFINITION_SHA256, None),
    }
    observed: dict[str, dict[str, Any]] = {}
    with os.scandir(INPUT_ROOT) as entries:
        for entry in entries:
            if entry.name not in expected:
                raise GateError(f"unexpected frozen GenCase input: {entry.name}")
            metadata = entry.stat(follow_symlinks=False)
            if entry.is_symlink() or not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise GateError(f"unsafe frozen GenCase input: {entry.name}")
            if stat.S_IMODE(metadata.st_mode) != 0o400 or metadata.st_mode & 0o111:
                raise GateError(f"frozen GenCase input is executable or writable: {entry.name}")
            digest = core.sha256_regular_file(Path(entry.path), limit=16 * 1024 * 1024)
            expected_digest, expected_size = expected[entry.name]
            if digest != expected_digest or (expected_size is not None and metadata.st_size != expected_size):
                raise GateError(f"frozen GenCase input identity differs: {entry.name}")
            observed[entry.name] = {"sha256": digest, "size_bytes": metadata.st_size, "mode": "0400"}
    if set(observed) != set(expected):
        raise GateError("frozen GenCase input file set differs")
    return {"receipt": {"path": str(SEED_RECEIPT), "sha256": SEED_RECEIPT_SHA256}, "input_root": str(INPUT_ROOT), "files": observed}


def verify_elf_metadata_receipt() -> dict[str, Any]:
    if core.sha256_regular_file(ELF_RECEIPT) != ELF_RECEIPT_SHA256:
        raise GateError("offline ELF metadata receipt digest differs")
    receipt = core.read_json_object(ELF_RECEIPT)
    if receipt.get("status") != "PASS_OFFLINE_ELF_METADATA" or receipt.get("precompiled_binary_executed") is not False:
        raise GateError("offline ELF metadata receipt is not a non-executing PASS")
    try:
        objects = receipt["results"]["objects"]
    except (KeyError, TypeError) as exc:
        raise GateError("offline ELF receipt lacks its object list") from exc
    matches = [item for item in objects if isinstance(item, dict) and item.get("sha256") == GENCASE_SHA256]
    if len(matches) != 1:
        raise GateError("offline ELF receipt does not identify exactly one GenCase object")
    item = matches[0]
    metadata = item.get("metadata")
    if item.get("size_bytes") != GENCASE_SIZE or item.get("paths") != ["bin/linux/GenCase_linux64"]:
        raise GateError("offline ELF GenCase identity differs")
    if not isinstance(metadata, dict):
        raise GateError("offline ELF GenCase metadata is missing")
    if (
        metadata.get("class") != "ELF64"
        or metadata.get("endianness") != "little"
        or metadata.get("machine") != 62
        or metadata.get("type") != 2
        or metadata.get("interpreter") != "/lib64/ld-linux-x86-64.so.2"
        or metadata.get("rpath") is not None
        or metadata.get("runpath") is not None
        or metadata.get("gnu_stack_executable") is not False
    ):
        raise GateError("offline ELF GenCase metadata boundary differs")
    needed = metadata.get("needed")
    if not isinstance(needed, list) or any(any(token in name.lower() for token in ("cuda", "nvidia", "ros", "gazebo", "chrono", "dsph")) for name in needed):
        raise GateError("offline ELF GenCase dependencies violate the runtime boundary")
    return {"path": str(ELF_RECEIPT), "sha256": ELF_RECEIPT_SHA256, "gencase": item}


def verify_review_artifacts() -> dict[str, Any]:
    policy = core.read_json_object(POLICY_PATH)
    schema = core.read_json_object(SCHEMA_PATH)
    if set(policy) != set(schema.get("required", [])) or schema.get("additionalProperties") is not False:
        raise GateError("execution policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-gencase-execution-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_GENCASE_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_GENCASE_EXECUTION_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_c1_gencase_generation_attempt",
        "status": "REVIEWED_SINGLE_GENCASE_RUNTIME_V1_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "LOAD_TRANSIENT_PROFILE_RUN_ONE_GENCASE_ATTEMPT_REMOVE_PROFILE_AND_AUDIT",
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            raise GateError(f"execution policy field differs: {key}")
    if policy.get("allowed_gate_commands") != ["self-check", "preflight", "run"]:
        raise GateError("execution policy command surface differs")
    for section, expected_hash in SECTION_HASHES.items():
        if canonical_hash(policy.get(section)) != expected_hash:
            raise GateError(f"execution policy section differs: {section}")
    if policy.get("trusted_tools") != trusted_tool_policy():
        raise GateError("execution policy trusted tool contract differs")
    attempt = policy["frozen_attempt"]
    if (
        attempt.get("case_id") != CASE_ID
        or attempt.get("attempt_root") != str(ATTEMPT_ROOT)
        or attempt.get("output_root") != str(OUTPUT_ROOT)
        or attempt.get("receipt") != str(RECEIPT)
        or attempt.get("partial_receipt") != str(PARTIAL_RECEIPT)
        or attempt.get("console_log") != str(CONSOLE_LOG)
        or attempt.get("bootstrap_profile") != BOOTSTRAP_PROFILE
        or attempt.get("runtime_profile") != RUNTIME_PROFILE
        or attempt.get("profile_sha256") != PROFILE_SHA256
    ):
        raise GateError("execution policy frozen attempt differs")
    fixed = policy["fixed_guest_command"]
    if fixed.get("setup_shell") != ["/usr/bin/dash", "-c", SETUP_SHELL] or fixed.get("gencase_argv") != GENCASE_ARGV:
        raise GateError("execution policy fixed guest command differs")
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": core.sha256_regular_file(POLICY_PATH)},
        "schema": {"path": str(SCHEMA_PATH), "sha256": core.sha256_regular_file(SCHEMA_PATH)},
        "profile": verify_profile(),
        "seed": verify_seed_receipt_and_input(),
        "elf_metadata": verify_elf_metadata_receipt(),
        "frozen_seed_helper": {"path": str(SEED_GATE_PATH), "sha256": SEED_GATE_SHA256},
    }


def relevant_processes() -> list[dict[str, Any]]:
    needles = ("bwrap", "aa-exec", "GenCase_linux64", "DualSPHysics5.4CPU_linux64")
    own_pid = os.getpid()
    result: list[dict[str, Any]] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal() or int(entry.name) == own_pid:
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace")
            comm = (entry / "comm").read_text(encoding="utf-8").strip()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        combined = f"{comm} {command}"
        if any(needle in combined for needle in needles):
            result.append({"pid": int(entry.name), "comm": comm, "cmdline": command[:4096]})
    return result


def bwrap_argv() -> list[str]:
    return [
        "/usr/bin/timeout",
        "--foreground",
        "--kill-after=5s",
        f"{WALL_TIMEOUT_SECONDS}s",
        "/usr/bin/aa-exec",
        "-p",
        BOOTSTRAP_PROFILE,
        "--",
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
        "r8-liquid-u3-gencase",
        "--clearenv",
        "--setenv",
        "HOME",
        "/nonexistent",
        "--setenv",
        "PATH",
        "/usr/bin",
        "--setenv",
        "LC_ALL",
        "C.UTF-8",
        "--setenv",
        "LANG",
        "C.UTF-8",
        "--setenv",
        "TZ",
        "UTC",
        "--uid",
        "0",
        "--gid",
        "0",
        "--ro-bind",
        "/usr",
        "/usr",
        "--symlink",
        "usr/lib",
        "/lib",
        "--symlink",
        "usr/lib64",
        "/lib64",
        "--tmpfs",
        "/work",
        "--ro-bind",
        str(INPUT_ROOT),
        "/work/input",
        "--bind",
        str(OUTPUT_ROOT),
        "/work/output",
        "--tmpfs",
        "/runtime",
        "--chdir",
        "/work/output",
        "--",
        "/usr/bin/prlimit",
        f"--as={ADDRESS_SPACE_BYTES}:{ADDRESS_SPACE_BYTES}",
        f"--cpu={CPU_SECONDS}:{CPU_SECONDS + 5}",
        f"--nproc={PROCESS_LIMIT}:{PROCESS_LIMIT}",
        f"--nofile={OPEN_FILE_LIMIT}:{OPEN_FILE_LIMIT}",
        f"--fsize={FILE_SIZE_LIMIT}:{FILE_SIZE_LIMIT}",
        "--core=0:0",
        "--",
        "/usr/bin/dash",
        "-c",
        SETUP_SHELL,
    ]


def verify_argv_contract(argv: list[str]) -> None:
    ro_binds = [argv[index + 1 : index + 3] for index, value in enumerate(argv[:-2]) if value == "--ro-bind"]
    rw_binds = [argv[index + 1 : index + 3] for index, value in enumerate(argv[:-2]) if value == "--bind"]
    if ro_binds != [["/usr", "/usr"], [str(INPUT_ROOT), "/work/input"]]:
        raise GateError("bwrap read-only bind contract differs")
    if rw_binds != [[str(OUTPUT_ROOT), "/work/output"]]:
        raise GateError("bwrap writable bind contract differs")
    if [argv[index + 1] for index, value in enumerate(argv[:-1]) if value == "--tmpfs"] != ["/work", "/runtime"]:
        raise GateError("bwrap tmpfs contract differs")
    for forbidden in ("--proc", "--dev", "--share-net", "/home/zrj/scout_ws", "/dev/nvidia0", "/opt/ros/jazzy"):
        if forbidden in argv:
            raise GateError(f"bwrap argv violates a forbidden boundary: {forbidden}")
    for required in ("--unshare-user", "--unshare-pid", "--unshare-net", "--disable-userns", "--assert-userns-disabled"):
        if argv.count(required) != 1:
            raise GateError(f"bwrap argv lacks exact isolation option: {required}")


def preflight(*, require_empty_groups: bool) -> dict[str, Any]:
    if os.geteuid() != HOST_UID or os.getegid() != HOST_GID:
        raise GateError("GenCase gate must run as the named unprivileged host user")
    groups = os.getgroups()
    if require_empty_groups and groups:
        raise GateError("supplementary groups were not cleared before aa-exec")
    core.assert_directory(LIQUID_ROOT, mode=0o750)
    core.assert_directory(LIQUID_ROOT / "cases", mode=0o750)
    core.assert_directory(LIQUID_ROOT / "audits", mode=0o750)
    for path in (ATTEMPT_ROOT, OUTPUT_ROOT, RECEIPT, PARTIAL_RECEIPT, CONSOLE_LOG):
        require_absent(path)
    memory = available_memory_bytes()
    disk = available_disk_bytes()
    if memory < MINIMUM_MEMORY_BYTES:
        raise GateError("available memory is below the reviewed GenCase floor")
    if disk < MINIMUM_DISK_BYTES:
        raise GateError("available disk is below the reviewed GenCase floor")
    residual = relevant_processes()
    if residual:
        raise GateError(f"relevant runtime process already exists: {residual}")
    tools = {name: require_tool(name) for name in TRUSTED_TOOLS}
    argv = bwrap_argv()
    verify_argv_contract(argv)
    return {
        "review": verify_review_artifacts(),
        "trusted_tools": tools,
        "identity": {"uid": os.geteuid(), "gid": os.getegid(), "supplementary_groups": groups, "execution_identity_ready": not groups},
        "resources": {"available_memory_bytes": memory, "available_disk_bytes": disk, "host_user_max_user_namespaces": read_host_userns_limit()},
        "fixed_argv": argv,
        "relevant_processes": residual,
    }


def reserve_attempt() -> None:
    mkdir_new(ATTEMPT_ROOT, 0o700)
    mkdir_new(OUTPUT_ROOT, 0o700)


def run_sandbox(argv: list[str]) -> dict[str, Any]:
    descriptor = os.open(
        CONSOLE_LOG,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o640,
    )
    started = time.monotonic()
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as output:
            try:
                completed = subprocess.run(
                    argv,
                    stdin=subprocess.DEVNULL,
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    cwd="/",
                    env={"PATH": "/usr/bin:/usr/sbin", "HOME": "/nonexistent", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"},
                    timeout=WALL_TIMEOUT_SECONDS + 15,
                    check=False,
                )
                returncode = completed.returncode
                python_timeout = False
            except subprocess.TimeoutExpired:
                returncode = 124
                python_timeout = True
            output.flush()
            os.fsync(output.fileno())
    finally:
        os.close(descriptor)
    return {"argv": argv, "returncode": returncode, "python_timeout": python_timeout, "elapsed_seconds": round(time.monotonic() - started, 6)}


def console_log_facts() -> dict[str, Any]:
    metadata = os.lstat(CONSOLE_LOG)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > CONSOLE_LOG_LIMIT:
        raise GateError("GenCase console log violates the bounded regular-file contract")
    core.set_mode_nofollow(CONSOLE_LOG, 0o440, directory=False)
    return {"path": str(CONSOLE_LOG), "sha256": core.sha256_regular_file(CONSOLE_LOG, limit=CONSOLE_LOG_LIMIT), "size_bytes": metadata.st_size, "mode_after_audit": "0440"}


def disarm_output_files() -> None:
    if not OUTPUT_ROOT.exists() or OUTPUT_ROOT.is_symlink():
        return
    with os.scandir(OUTPUT_ROOT) as entries:
        for entry in entries:
            metadata = entry.stat(follow_symlinks=False)
            if not entry.is_symlink() and stat.S_ISREG(metadata.st_mode) and metadata.st_nlink == 1:
                core.set_mode_nofollow(Path(entry.path), 0o400, directory=False)


def inventory_output() -> dict[str, Any]:
    core.assert_directory(ATTEMPT_ROOT, mode=0o700)
    core.assert_directory(OUTPUT_ROOT, mode=0o700)
    disarm_output_files()
    observed: dict[str, dict[str, Any]] = {}
    with os.scandir(OUTPUT_ROOT) as entries:
        for entry in entries:
            if entry.name not in EXPECTED_OUTPUTS:
                raise GateError(f"unexpected GenCase output entry: {entry.name}")
            metadata = entry.stat(follow_symlinks=False)
            if entry.is_symlink() or not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise GateError(f"unsafe GenCase output entry: {entry.name}")
            minimum, maximum = EXPECTED_OUTPUTS[entry.name]
            if not minimum <= metadata.st_size <= maximum:
                raise GateError(f"GenCase output size violates policy: {entry.name}")
            path = Path(entry.path)
            digest = core.sha256_regular_file(path, limit=maximum)
            observed[entry.name] = {"sha256": digest, "size_bytes": metadata.st_size}
    if set(observed) != set(EXPECTED_OUTPUTS):
        raise GateError("GenCase output file set differs")
    xml_path = OUTPUT_ROOT / "C1_static.xml"
    raw = core.read_regular_bytes(xml_path, limit=524_288)
    try:
        xml_root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise GateError("generated C1 XML is malformed") from exc
    if xml_root.tag != "case" or xml_root.find("./casedef/geometry/definition[@dp='0.002']") is None:
        raise GateError("generated C1 XML does not retain the fixed case/dp contract")
    if any(tag in {element.tag for element in xml_root.iter()} for tag in ("motion", "floating", "inout", "wavegen")):
        raise GateError("generated C1 XML unexpectedly enables motion or coupling")
    for name in observed:
        core.set_mode_nofollow(OUTPUT_ROOT / name, 0o440, directory=False)
        observed[name]["mode_after_audit"] = "0440"
    core.set_mode_nofollow(OUTPUT_ROOT, 0o550, directory=True)
    core.set_mode_nofollow(ATTEMPT_ROOT, 0o550, directory=True)
    return {"root": str(OUTPUT_ROOT), "files": observed, "manifest_sha256": canonical_hash(observed), "directory_mode_after_audit": "0550", "classification": "SIM_ONLY_UNVALIDATED"}


def execute() -> int:
    try:
        preflight_data = preflight(require_empty_groups=True)
    except GateError as exc:
        print(json.dumps({"status": "NO_GO_PREFLIGHT", "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2
    core.write_json_new(
        PARTIAL_RECEIPT,
        {
            "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_PARTIAL",
            "case_id": CASE_ID,
            "created_at_utc": core.utc_now(),
            "status": "STARTED_U3_C1_GENCASE",
            "preflight": preflight_data,
            "only_reviewed_gencase_executed": False,
            "compiled_solver_executed": False,
            "network_used": False,
            "gpu_device_exposed": False,
        },
    )
    final: dict[str, Any] = {
        "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_RECEIPT",
        "case_id": CASE_ID,
        "created_at_utc": core.utc_now(),
        "partial_start_record": str(PARTIAL_RECEIPT),
        "preflight": preflight_data,
        "only_reviewed_gencase_executed": True,
        "compiled_solver_executed": False,
        "other_precompiled_binary_executed": False,
        "upstream_shell_script_executed": False,
        "network_used": False,
        "gpu_device_exposed": False,
        "ros_or_gazebo_started": False,
        "system_packages_changed": False,
        "host_initial_namespace_setting_changed": False,
        "profile_cleanup_pending_external_same_sudo_session": True,
    }
    try:
        reserve_attempt()
        before_userns = read_host_userns_limit()
        result = run_sandbox(preflight_data["fixed_argv"])
        after_userns = read_host_userns_limit()
        final["execution"] = result
        final["host_user_max_user_namespaces"] = {"before": before_userns, "after": after_userns, "unchanged": before_userns == after_userns}
        final["console_log"] = console_log_facts()
        residual = relevant_processes()
        final["relevant_processes_after"] = residual
        if before_userns != after_userns:
            raise GateError("host user.max_user_namespaces changed")
        if residual:
            raise GateError(f"runtime process residue remains: {residual}")
        if result["returncode"] != 0 or result["python_timeout"]:
            raise GateError(f"GenCase execution failed or timed out: rc={result['returncode']}")
        final.update(
            {
                "status": "PASS_U3_C1_GENCASE_GENERATION",
                "output": inventory_output(),
                "seed_postcondition": verify_seed_receipt_and_input(),
                "next_allowed_stage": "SEPARATE_COMPILED_SOLVER_RUNTIME_ADMISSION_REQUIRED",
            }
        )
        core.write_json_new(RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(RECEIPT)}, ensure_ascii=False, sort_keys=True))
        return 0
    except (GateError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        try:
            disarm_output_files()
        except (GateError, OSError):
            pass
        final.update(
            {
                "status": "U3_C1_GENCASE_FAILED_NO_RETRY",
                "error": str(exc),
                "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED",
            }
        )
        try:
            if CONSOLE_LOG.exists() and "console_log" not in final:
                final["console_log"] = console_log_facts()
        except (GateError, OSError):
            pass
        try:
            core.write_json_new(RECEIPT, final)
        except (GateError, OSError):
            pass
        print(json.dumps({"status": final["status"], "receipt": str(RECEIPT)}, ensure_ascii=False, sort_keys=True))
        return 3


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "preflight", "run"))
    arguments = parser.parse_args()
    try:
        if arguments.command == "self-check":
            print(json.dumps({"status": "PASS_U3_C1_GENCASE_STATIC_REVIEW", "review": verify_review_artifacts(), "fixed_argv": bwrap_argv()}, ensure_ascii=False, sort_keys=True))
            return 0
        if arguments.command == "preflight":
            print(json.dumps({"status": "PASS_U3_C1_GENCASE_PREFLIGHT", "preflight": preflight(require_empty_groups=False)}, ensure_ascii=False, sort_keys=True))
            return 0
        return execute()
    except GateError as exc:
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
