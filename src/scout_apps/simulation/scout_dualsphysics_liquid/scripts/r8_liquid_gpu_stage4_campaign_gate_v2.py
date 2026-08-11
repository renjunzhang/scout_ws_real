#!/usr/bin/env python3
"""Create-new campaign adapter for the CUDA 12.8 GENCODE-list correction.

The frozen v1 campaign gate remains the execution implementation.  This
revision pins that implementation and supplies a fresh campaign policy whose
only build semantic change is the standard CUDA list spelling
``code=[sm_120,compute_120]``.  Historical roots and receipts stay immutable.
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
from typing import Any, Sequence


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
BASE_GATE_PATH = PACKAGE / "scripts/r8_liquid_gpu_stage4_campaign_gate_v1.py"
BASE_GATE_SHA256 = "03650ced298f943c69a6a0039fd2feef63049d42180696cec5c39b8b323c79bb"
POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T165034Z_v2.json"
EXACT_GENCODE = "GENCODE=-gencode=arch=compute_120,code=[sm_120,compute_120]"


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
    spec = importlib.util.spec_from_file_location("r8_gpu_stage4_campaign_gate_v1_pinned", BASE_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RevisionError("cannot load frozen v1 campaign gate")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.POLICY_PATH = POLICY_PATH
    return module


BASE = load_base()


def policy() -> dict[str, Any]:
    return BASE.read_json(POLICY_PATH)


def revision_check() -> dict[str, Any]:
    current = policy()
    if current.get("schema_version") != "r8-liquid-gpu-stage4-campaign-v2":
        raise RevisionError("campaign schema version drift")
    parent = current["parent"]
    pinned = (
        (Path(parent["previous_failure_receipt"]), parent["previous_failure_receipt_sha256"]),
        (Path(parent["previous_lifecycle_receipt"]), parent["previous_lifecycle_receipt_sha256"]),
    )
    for path, expected in pinned:
        if sha256_file(path) != expected:
            raise RevisionError(f"previous-attempt evidence hash drift: {path}")
    failure = BASE.read_json(Path(parent["previous_failure_receipt"]))
    lifecycle = BASE.read_json(Path(parent["previous_lifecycle_receipt"]))
    if failure.get("status") != "FAIL_GPU_BUILD" or failure.get("returncode") != 2:
        raise RevisionError("previous NVCC failure identity drift")
    stderr = Path(failure["stderr"]["path"])
    if sha256_file(stderr) != failure["stderr"]["sha256"]:
        raise RevisionError("previous NVCC stderr hash drift")
    stderr_text = stderr.read_text(encoding="utf-8", errors="replace")
    expected_error = "nvcc fatal   : 'compute_120' is not in 'keyword=value' format"
    if expected_error not in stderr_text:
        raise RevisionError("previous NVCC diagnostic drift")
    if lifecycle.get("zero_residue") is not True:
        raise RevisionError("previous profile lifecycle lacks zero-residue proof")

    gencode = [item for item in current["build"]["make_argv"] if item.startswith("GENCODE=")]
    if gencode != [EXACT_GENCODE]:
        raise RevisionError(f"GENCODE correction drift: {gencode}")
    remediation = current["remediation"]
    if remediation.get("corrected_argument") != EXACT_GENCODE.removeprefix("GENCODE="):
        raise RevisionError("remediation and Make argv disagree")
    for key in ("source_delta", "profile_permission_delta", "toolchain_delta", "architecture_delta"):
        if remediation.get(key) is not False:
            raise RevisionError(f"forbidden remediation expansion: {key}")
    if current["build"]["parallel_jobs"] != 1:
        raise RevisionError("parallel_jobs drift")
    joined = "\n".join(current["build"]["make_argv"])
    for forbidden in ("g++-13", "sm_89", "compute_89", "USE_FAST_MATH=YES", "-j2", "-j4"):
        if forbidden in joined:
            raise RevisionError(f"forbidden build drift: {forbidden}")
    return {
        "status": "PASS_GENCODE_LIST_REMEDIATION_CONTRACT",
        "base_gate_sha256": BASE_GATE_SHA256,
        "previous_failure_sha256": parent["previous_failure_receipt_sha256"],
        "previous_zero_residue": True,
        "exact_gencode": EXACT_GENCODE,
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
