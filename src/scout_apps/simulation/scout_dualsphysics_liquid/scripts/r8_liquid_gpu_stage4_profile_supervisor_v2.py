#!/usr/bin/env python3
"""Pinned AppArmor lifecycle adapter for fresh GENCODE-remediation campaign."""

from __future__ import annotations

import hashlib
import importlib.util
import os
import stat
import sys
from pathlib import Path
from types import ModuleType


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
BASE_SUPERVISOR = PACKAGE / "scripts/r8_liquid_gpu_stage4_profile_supervisor_v1.py"
BASE_SUPERVISOR_SHA256 = "f30b2381c71646a18e569f0eccb34097d982cebd7b7e8d1650022e9b7a0f04e4"
POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T165034Z_v2.json"
POLICY_SHA256 = "b88e9efaa204047d531c728ac9ab3b250a8ce07fe8a9c5479525a3cee4a3f64f"
GATE_PATH = PACKAGE / "scripts/r8_liquid_gpu_stage4_campaign_gate_v2.py"
GATE_SHA256 = "56afaafccec77807a65607ddc2ead36ca2d8e5759429677ba24d70562da37768"


def sha256_file(path: Path) -> str:
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise RuntimeError(f"unsafe pinned execution file: {path}")
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
    expected = (
        (BASE_SUPERVISOR, BASE_SUPERVISOR_SHA256),
        (POLICY_PATH, POLICY_SHA256),
        (GATE_PATH, GATE_SHA256),
    )
    for path, digest in expected:
        if sha256_file(path) != digest:
            raise RuntimeError(f"pinned execution hash drift: {path}")
    spec = importlib.util.spec_from_file_location("r8_gpu_stage4_profile_supervisor_v1_pinned", BASE_SUPERVISOR)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load pinned lifecycle supervisor")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.POLICY_PATH = POLICY_PATH
    module.GATE_PATH = GATE_PATH
    return module


def main() -> int:
    try:
        base = load_base()
    except (RuntimeError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())
