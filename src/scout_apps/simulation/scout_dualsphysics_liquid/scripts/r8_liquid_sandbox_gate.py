#!/usr/bin/env python3
"""Fail-closed P0-B VM, CPU-build and GenCase sandbox admission gate.

This module does not create or start a VM, install packages, invoke an
upstream build system, or execute any file from the DualSPHysics checkout.
``self-check`` is read-only.  ``write-receipt`` only publishes one create-new
JSON receipt below the fixed liquid audit root.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import stat
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_dependency_gate as dependency  # noqa: E402
import r8_liquid_safety as safety  # noqa: E402


SCHEMA_VERSION = "smpcc-r8-liquid-sandbox-preflight-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_SANDBOX_PREFLIGHT"
QEMU_SYSTEM = Path("/usr/bin/qemu-system-x86_64")
QEMU_IMG = Path("/usr/bin/qemu-img")
KVM_DEVICE = Path("/dev/kvm")
UNSHARE = Path("/usr/bin/unshare")
SETPRIV = Path("/usr/bin/setpriv")
SYSTEMD_RUN = Path("/usr/bin/systemd-run")
TRUSTED_BUILD_TOOLS = (
    Path("/usr/bin/gcc"),
    Path("/usr/bin/g++"),
    Path("/usr/bin/cmake"),
    Path("/usr/bin/make"),
)
POLICY_FILES = {
    "vm": PACKAGE_ROOT / "config/sandbox/p0b_vm_policy_v1.json",
    "cpu_build": PACKAGE_ROOT / "config/sandbox/p0b_cpu_build_policy_v1.json",
    "gencase": PACKAGE_ROOT / "config/sandbox/p0b_gencase_admission_policy_v1.json",
}
SCHEMA_FILE = PACKAGE_ROOT / "schema/sandbox_preflight_v1.json"
REQUIRED_LAYOUT_DIRS = (
    "dependency/vm_images",
    "dependency/toolchains",
    "audits/sandbox",
    "audits/tools/gencase",
    "scratch/vm",
    "scratch/build",
    "quarantine",
)
MODE_POLICY = {
    "vm-host": ("vm",),
    "cpu-build": ("vm", "cpu_build"),
    "gencase": ("vm", "gencase"),
}
RECEIPT_RE = re.compile(
    r"^sandbox_preflight_(vm-host|cpu-build|gencase)_[0-9]{8}T[0-9]{6}Z\.json$"
)


class SandboxGateError(RuntimeError):
    """A missing or ambiguous sandbox condition is a hard NO-GO."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SandboxGateError(message)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(str(path), flags)
    digest = hashlib.sha256()
    with os.fdopen(fd, "rb") as stream:
        before = os.fstat(stream.fileno())
        require(stat.S_ISREG(before.st_mode), f"hash input is not a regular file: {path}")
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
        after = os.fstat(stream.fileno())
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            f"file changed while hashing: {path}",
        )
    return digest.hexdigest()


def read_json_object(path: Path) -> Dict[str, Any]:
    require(path.is_file() and not path.is_symlink(), f"JSON policy is missing or unsafe: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"JSON policy is not an object: {path}")
    return value


def policy_evidence(names: Sequence[str]) -> Dict[str, Any]:
    evidence: Dict[str, Any] = {}
    for name in names:
        path = POLICY_FILES[name]
        policy = read_json_object(path)
        evidence[name] = {
            "path": str(path),
            "sha256": sha256_file(path),
            "schema_version": policy.get("schema_version"),
            "document_type": policy.get("document_type"),
        }
    return evidence


def executable_evidence(path: Path) -> Dict[str, Any]:
    evidence: Dict[str, Any] = {
        "path": str(path),
        "exists": path.exists(),
        "is_symlink": path.is_symlink(),
    }
    if not path.exists():
        return evidence
    resolved = path.resolve(strict=True)
    file_stat = resolved.stat()
    evidence.update(
        {
            "resolved_path": str(resolved),
            "regular_file": stat.S_ISREG(file_stat.st_mode),
            "executable": os.access(resolved, os.X_OK),
            "uid": file_stat.st_uid,
            "gid": file_stat.st_gid,
            "mode": f"{stat.S_IMODE(file_stat.st_mode):04o}",
            "sha256": sha256_file(resolved),
        }
    )
    return evidence


def trusted_system_executable(evidence: Mapping[str, Any]) -> bool:
    resolved_text = str(evidence.get("resolved_path", ""))
    return bool(
        evidence.get("exists")
        and evidence.get("regular_file")
        and evidence.get("executable")
        and evidence.get("uid") == 0
        and (resolved_text == "/usr" or resolved_text.startswith("/usr/"))
    )


def meminfo_bytes() -> Dict[str, int]:
    values: Dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        key, raw = line.split(":", 1)
        fields = raw.split()
        if fields and fields[0].isdigit():
            multiplier = 1024 if len(fields) > 1 and fields[1] == "kB" else 1
            values[key] = int(fields[0]) * multiplier
    return values


def kvm_evidence() -> Dict[str, Any]:
    evidence: Dict[str, Any] = {
        "path": str(KVM_DEVICE),
        "exists": KVM_DEVICE.exists(),
        "readable": os.access(KVM_DEVICE, os.R_OK),
        "writable": os.access(KVM_DEVICE, os.W_OK),
    }
    if KVM_DEVICE.exists():
        device_stat = KVM_DEVICE.stat()
        evidence["character_device"] = stat.S_ISCHR(device_stat.st_mode)
        evidence["mode"] = f"{stat.S_IMODE(device_stat.st_mode):04o}"
    return evidence


def layout_evidence(root: Path) -> Dict[str, Any]:
    entries = []
    missing = []
    for relative in REQUIRED_LAYOUT_DIRS:
        path = root / relative
        safe = False
        try:
            safety.ensure_within_approved_root(path, root=root, require_exists=True)
            safe = path.is_dir() and not path.is_symlink()
        except (safety.LiquidSafetyError, OSError):
            safe = False
        entries.append({"path": str(path), "safe_directory": safe})
        if not safe:
            missing.append(str(path))
    return {"entries": entries, "missing_or_unsafe": missing}


def validate_frozen_policy_semantics(policies: Mapping[str, Dict[str, Any]]) -> List[str]:
    errors: List[str] = []
    vm = policies["vm"]
    if vm.get("runtime", {}).get("system_install_authorized") is not False:
        errors.append("VM policy unexpectedly authorizes system installation")
    guest = vm.get("guest", {})
    if guest.get("network") != "none" or guest.get("shared_directories") != []:
        errors.append("VM policy does not freeze network/share isolation")
    if guest.get("gpu_passthrough") is not False:
        errors.append("VM policy unexpectedly permits GPU passthrough")

    build = policies["cpu_build"]
    features = build.get("features", {})
    if features != {
        "cpu": True,
        "cuda": False,
        "chrono": False,
        "wavegen": False,
        "moordynplus": False,
    }:
        errors.append("minimal CPU feature freeze differs from policy")
    if build.get("build_recipe_status") != "NOT_FROZEN_NOT_AUTHORIZED_TO_RUN":
        errors.append("build recipe status changed without a new admission version")

    gencase = policies["gencase"]
    if gencase.get("binary", {}).get("sha256") != (
        "a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226"
    ):
        errors.append("GenCase hash differs from the P0-A manifest")
    if gencase.get("binary", {}).get("current_admission") != "NOT_EXECUTED_NOT_ADMITTED":
        errors.append("GenCase policy no longer says NOT_EXECUTED_NOT_ADMITTED")
    return errors


def build_preflight(mode: str) -> Dict[str, Any]:
    require(mode in MODE_POLICY, f"unsupported sandbox mode: {mode}")
    errors: List[str] = []
    policies = {name: read_json_object(path) for name, path in POLICY_FILES.items()}
    errors.extend(validate_frozen_policy_semantics(policies))

    host_safety = safety.build_preflight(
        estimated_case_bytes=20 * safety.GIB,
        require_simulation_stopped=True,
        require_gpu_idle=True,
    )
    if host_safety["status"] != "PASS":
        errors.extend(f"host safety: {item}" for item in host_safety["errors"])

    root = safety.APPROVED_ROOT
    layout = layout_evidence(root)
    if layout["missing_or_unsafe"]:
        errors.append("P0-B external sandbox layout is not prepared")

    qemu_system = executable_evidence(QEMU_SYSTEM)
    qemu_img = executable_evidence(QEMU_IMG)
    unshare = executable_evidence(UNSHARE)
    setpriv = executable_evidence(SETPRIV)
    systemd_run = executable_evidence(SYSTEMD_RUN)
    kvm = kvm_evidence()
    if not qemu_system.get("exists"):
        errors.append(f"QEMU system runtime is missing: {QEMU_SYSTEM}")
    elif not trusted_system_executable(qemu_system):
        errors.append("QEMU system runtime is not a trusted root-owned /usr executable")
    if not qemu_img.get("exists"):
        errors.append(f"QEMU image tool is missing: {QEMU_IMG}")
    elif not trusted_system_executable(qemu_img):
        errors.append("QEMU image tool is not a trusted root-owned /usr executable")
    if not (kvm.get("character_device") and kvm.get("readable") and kvm.get("writable")):
        errors.append("KVM device is unavailable to the current user")
    if policies["vm"].get("runtime", {}).get("system_install_authorized") is not True:
        errors.append("QEMU system installation has not been explicitly admitted")

    memory = meminfo_bytes()
    vm_minimum_memory = int(
        policies["vm"]["host_start_gate"]["minimum_available_memory_bytes"]
    )
    minimum_memory = (
        max(
            vm_minimum_memory,
            int(policies["cpu_build"]["resources"]["minimum_available_memory_bytes"]),
        )
        if mode == "cpu-build"
        else vm_minimum_memory
    )
    available_memory = memory.get("MemAvailable", 0)
    if available_memory < minimum_memory:
        errors.append(
            f"available memory {available_memory} is below the mode minimum {minimum_memory}"
        )

    try:
        dependency_verification = dependency.verify_existing()
    except Exception as exc:  # converted to evidence; no upstream file is executed
        dependency_verification = {"status": "NO_GO", "error": str(exc)}
        errors.append(f"pinned dependency verification failed: {exc}")

    manifest_path = dependency.dependency_paths()["manifest"]
    manifest_hash = sha256_file(manifest_path) if manifest_path.is_file() else None
    expected_manifest_hash = policies["cpu_build"]["source"]["manifest_file_sha256"]
    if manifest_hash != expected_manifest_hash:
        errors.append("DualSPHysics manifest file hash differs from the build policy")

    if mode == "cpu-build":
        for tool in TRUSTED_BUILD_TOOLS:
            evidence = executable_evidence(tool)
            if not trusted_system_executable(evidence):
                errors.append(f"build tool is not a trusted root-owned /usr executable: {tool}")
        errors.append("minimal CPU build recipe is not frozen or authorized to run")
    if mode == "gencase":
        errors.append("GenCase first execution remains NOT_EXECUTED_NOT_ADMITTED")

    core: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "created_utc": utc_now(),
        "development_only": True,
        "formal": False,
        "fidelity_validation_status": "UNVALIDATED",
        "physical_primary_eligible": False,
        "approved_root": str(root),
        "mode": mode,
        "implementation": {
            "gate_path": str(Path(__file__).resolve()),
            "gate_sha256": sha256_file(Path(__file__).resolve()),
            "schema_path": str(SCHEMA_FILE),
            "schema_sha256": sha256_file(SCHEMA_FILE),
        },
        "policies": policy_evidence(MODE_POLICY[mode]),
        "host_safety": host_safety,
        "runtime": {
            "qemu_system": qemu_system,
            "qemu_img": qemu_img,
            "kvm": kvm,
            "unshare": unshare,
            "setpriv": setpriv,
            "systemd_run": systemd_run,
            "trusted_build_tools": (
                [executable_evidence(path) for path in TRUSTED_BUILD_TOOLS]
                if mode == "cpu-build"
                else []
            ),
        },
        "resources": {
            "memory": memory,
            "minimum_available_memory_bytes": minimum_memory,
            "host_disk_policy": host_safety.get("resource_policy", {}),
        },
        "source_dependency": {
            "verification": dependency_verification,
            "manifest_path": str(manifest_path),
            "manifest_file_sha256": manifest_hash,
        },
        "layout": layout,
        "upstream_code_executed": False,
        "vm_started": False,
        "build_started": False,
        "gencase_started": False,
        "destructive_cleanup": "forbidden",
        "status": "PASS" if not errors else "NO_GO",
        "errors": errors,
    }
    return dict(core, receipt_hash=safety.canonical_hash(core))


def validate_receipt_path(path: Path, mode: str) -> Path:
    root = safety.validate_approved_root(safety.APPROVED_ROOT, require_exists=True)
    path = safety.ensure_within_approved_root(path, root=root)
    require(path.parent == root / "audits/sandbox", "sandbox receipt must be directly under audits/sandbox")
    match = RECEIPT_RE.fullmatch(path.name)
    require(match is not None and match.group(1) == mode, "sandbox receipt filename does not match its mode")
    require(not path.exists() and not path.is_symlink(), f"sandbox receipt already exists: {path}")
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    check = sub.add_parser("self-check", help="read-only P0-B sandbox admission check")
    check.add_argument("--mode", choices=tuple(MODE_POLICY), required=True)
    receipt = sub.add_parser("write-receipt", help="write one create-new PASS/NO-GO receipt")
    receipt.add_argument("--mode", choices=tuple(MODE_POLICY), required=True)
    receipt.add_argument("--receipt", type=Path, required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "write-receipt":
            receipt_path = validate_receipt_path(args.receipt, args.mode)
        else:
            receipt_path = None
        report = build_preflight(args.mode)
        if receipt_path is not None:
            safety.atomic_write_json_new(receipt_path, report)
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if report["status"] == "PASS" else 2
    except (
        SandboxGateError,
        safety.LiquidSafetyError,
        dependency.DependencyGateError,
        OSError,
        ValueError,
        json.JSONDecodeError,
    ) as exc:
        print(f"R8_LIQUID_SANDBOX_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
