#!/usr/bin/env python3
"""Pinned AppArmor lifecycle adapter for the POSIX-/bin fresh campaign."""

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
POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T170339Z_v3.json"
POLICY_SHA256 = "63b5cf7d00441faf77f946fec0fd4e57a4e8b06df242013b836dd3471860414f"
GATE_PATH = PACKAGE / "scripts/r8_liquid_gpu_stage4_campaign_gate_v3.py"
GATE_SHA256 = "1c1bfc0df0e1959cc723ab22cdbb1163c2307ed4c19e9783f862b63ccfb7643d"


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
    for path, digest in (
        (BASE_SUPERVISOR, BASE_SUPERVISOR_SHA256),
        (POLICY_PATH, POLICY_SHA256),
        (GATE_PATH, GATE_SHA256),
    ):
        if sha256_file(path) != digest:
            raise RuntimeError(f"pinned execution hash drift: {path}")
    spec = importlib.util.spec_from_file_location("r8_gpu_stage4_profile_supervisor_v1_for_v3", BASE_SUPERVISOR)
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
