#!/usr/bin/env python3
"""Fresh campaign adapter adding the POSIX /bin alias required by NVCC.

The pinned v1 gate remains the implementation.  This revision inserts exactly
``--symlink usr/bin /bin`` into the otherwise-identical bwrap argv and binds it
to the preserved rc=127 evidence from the previous fresh campaign.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import stat
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping, Sequence


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
BASE_GATE_PATH = PACKAGE / "scripts/r8_liquid_gpu_stage4_campaign_gate_v1.py"
BASE_GATE_SHA256 = "03650ced298f943c69a6a0039fd2feef63049d42180696cec5c39b8b323c79bb"
POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T170339Z_v3.json"
PREVIOUS_POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T165034Z_v2.json"
PREVIOUS_POLICY_SHA256 = "b88e9efaa204047d531c728ac9ab3b250a8ce07fe8a9c5479525a3cee4a3f64f"
SANDBOX_DELTA = ["--symlink", "usr/bin", "/bin"]


class RevisionError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise RevisionError(f"unsafe pinned input: {path}")
    digest = hashlib.sha256()
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            digest.update(block)
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def load_base() -> ModuleType:
    if sha256_file(BASE_GATE_PATH) != BASE_GATE_SHA256:
        raise RevisionError("frozen v1 campaign gate hash drift")
    spec = importlib.util.spec_from_file_location("r8_gpu_stage4_campaign_gate_v1_for_v3", BASE_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RevisionError("cannot load frozen v1 campaign gate")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.POLICY_PATH = POLICY_PATH
    return module


BASE = load_base()
BASE_BUILD_CHILD_ARGV = BASE.build_child_argv


def policy() -> dict[str, Any]:
    return BASE.read_json(POLICY_PATH)


def build_child_argv(current: Mapping[str, Any]) -> list[str]:
    argv = BASE_BUILD_CHILD_ARGV(current)
    anchor = ["--ro-bind", "/usr", "/usr"]
    matches = [index for index in range(len(argv) - 2) if argv[index:index + 3] == anchor]
    if matches != [19]:
        raise RevisionError(f"bwrap /usr anchor drift: {matches}")
    insertion = matches[0] + len(anchor)
    result = argv[:insertion] + SANDBOX_DELTA + argv[insertion:]
    if result.count("/bin") != 1 or result[insertion:insertion + 3] != SANDBOX_DELTA:
        raise RevisionError("non-deterministic /bin alias delta")
    return result


BASE.build_child_argv = build_child_argv


def revision_check() -> dict[str, Any]:
    if sha256_file(PREVIOUS_POLICY_PATH) != PREVIOUS_POLICY_SHA256:
        raise RevisionError("previous corrected-GENCODE policy hash drift")
    current = policy()
    previous = BASE.read_json(PREVIOUS_POLICY_PATH)
    if current.get("schema_version") != "r8-liquid-gpu-stage4-campaign-v3":
        raise RevisionError("campaign schema version drift")
    parent = current["parent"]
    for path_key, digest_key in (
        ("previous_failure_receipt", "previous_failure_receipt_sha256"),
        ("previous_lifecycle_receipt", "previous_lifecycle_receipt_sha256"),
    ):
        path = Path(parent[path_key])
        if sha256_file(path) != parent[digest_key]:
            raise RevisionError(f"previous evidence hash drift: {path}")
    failure = BASE.read_json(Path(parent["previous_failure_receipt"]))
    lifecycle = BASE.read_json(Path(parent["previous_lifecycle_receipt"]))
    if failure.get("status") != "FAIL_GPU_BUILD" or failure.get("returncode") != 2:
        raise RevisionError("previous build failure identity drift")
    stderr = Path(failure["stderr"]["path"])
    if sha256_file(stderr) != failure["stderr"]["sha256"]:
        raise RevisionError("previous stderr hash drift")
    stderr_text = stderr.read_text(encoding="utf-8", errors="replace")
    if "JCellDivGpu_ker.o] Error 127" not in stderr_text or "nvcc fatal" in stderr_text:
        raise RevisionError("previous silent NVCC rc=127 diagnostic drift")
    if lifecycle.get("zero_residue") is not True:
        raise RevisionError("previous profile lifecycle lacks zero residue")

    for key in ("source", "wrapper", "build", "object_contract", "invariants"):
        if current[key] != previous[key]:
            raise RevisionError(f"forbidden non-sandbox contract drift: {key}")
    remediation = current["remediation"]
    if remediation.get("only_sandbox_argv_delta") != SANDBOX_DELTA:
        raise RevisionError("sandbox remediation delta drift")
    if remediation.get("only_profile_permission_delta") != "/newroot/bin wl,":
        raise RevisionError("profile remediation delta drift")
    for key in ("source_delta", "toolchain_delta", "architecture_delta", "network_delta", "gpu_device_delta"):
        if remediation.get(key) is not False:
            raise RevisionError(f"forbidden remediation expansion: {key}")
    child = build_child_argv(current)
    if "--unshare-net" not in child or any(item.startswith("/dev/nvidia") for item in child):
        raise RevisionError("network/GPU isolation drift")
    return {
        "status": "PASS_POSIX_SH_ALIAS_REMEDIATION_CONTRACT",
        "base_gate_sha256": BASE_GATE_SHA256,
        "previous_failure_sha256": parent["previous_failure_receipt_sha256"],
        "previous_zero_residue": True,
        "sandbox_argv_delta": SANDBOX_DELTA,
        "profile_permission_delta": "/newroot/bin wl,",
        "semantic_delta_count": 1,
    }


def main(argv: Sequence[str] | None = None) -> int:
    try:
        revision_check()
    except (RevisionError, OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "FAIL", "phase": "revision-check", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    return BASE.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
