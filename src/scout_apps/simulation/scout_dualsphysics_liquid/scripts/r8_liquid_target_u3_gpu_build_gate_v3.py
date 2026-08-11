#!/usr/bin/env python3
"""Byte-pinned v3 admission layer for the one-shot RTX 5080 Attempt A gate.

The v2 read-only preflight correctly reached uid/gid 1000 with no groups, but
its process classifier treated every host ``bwrap`` as a competing liquid
build.  Long-lived Zotero desktop sandboxes therefore caused a false deny.
This create-new layer preserves every v2 byte and state transition and narrows
only that classifier: compiler/build processes always conflict, while bwrap or
aa-exec conflicts only when its argv identifies this liquid campaign/build
surface.  No runtime output existed when this revision was created.
"""

from __future__ import annotations

import hashlib
import importlib.util
import os
import stat
import sys
from pathlib import Path
from types import ModuleType


sys.dont_write_bytecode = True

SCRIPT_PATH = Path(__file__).resolve()
PREVIOUS_GATE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_gpu_build_gate_v2.py")
PREVIOUS_GATE_SHA256 = "d9d02b2aa6a237ed49b7e64cbf0f19d769a32f22c372dbc56b18be47e376eb7f"

ALWAYS_CONFLICTING_NAMES = {
    "make",
    "nvcc",
    "cicc",
    "ptxas",
    "fatbinary",
    "nvlink",
    "cc1plus",
    "DualSPHysics5.4_linux64",
}
CONDITIONAL_SANDBOX_NAMES = {"bwrap", "aa-exec"}
LIQUID_SANDBOX_MARKERS = (
    "u3_source_gpu_build_sm120_20260810T102641Z",
    "r8-liquid-u3-",
    "/home/zrj/scout_liquid_lab",
    "DualSPHysics",
    "/usr/bin/make",
    "/usr/local/cuda-12.8/bin/nvcc",
)


def sha256_regular_file(path: Path, *, limit: int = 4 * 1024 * 1024) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size > limit:
            raise RuntimeError(f"unsafe byte-pinned execution core: {path}")
        digest = hashlib.sha256()
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                return digest.hexdigest()
            digest.update(block)
    finally:
        os.close(descriptor)


def load_previous_gate() -> ModuleType:
    if sha256_regular_file(PREVIOUS_GATE_PATH) != PREVIOUS_GATE_SHA256:
        raise RuntimeError("byte-pinned v2 execution core differs")
    spec = importlib.util.spec_from_file_location("r8_liquid_u3_gpu_build_v2_frozen", PREVIOUS_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load byte-pinned v2 execution core")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = load_previous_gate()
base_self_check = core.self_check


def is_conflicting_process(comm: str, cmdline: str) -> bool:
    if comm in ALWAYS_CONFLICTING_NAMES:
        return True
    if str(core.ROOT_A) in cmdline or str(core.ROOT_B) in cmdline:
        return True
    if comm in CONDITIONAL_SANDBOX_NAMES:
        return any(marker in cmdline for marker in LIQUID_SANDBOX_MARKERS)
    return False


def conflicting_processes_v3() -> list[str]:
    ancestors = core.ancestor_pids()
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
        if is_conflicting_process(comm, cmdline):
            result.append(f"pid={entry.name} comm={comm} argv={cmdline[:3072]}")
    return sorted(result)


def self_check_v3():
    result = base_self_check()
    result["status"] = "PASS_G2A_EXECUTION_LAYER_STATIC_SELF_CHECK_V3"
    result["frozen_v2_gate"] = {
        "path": str(PREVIOUS_GATE_PATH),
        "sha256": PREVIOUS_GATE_SHA256,
    }
    result["process_classifier"] = {
        "zotero_bwrap_conflicts": is_conflicting_process(
            "bwrap", "bwrap --args 42 -- zotero -url"
        ),
        "liquid_bwrap_conflicts": is_conflicting_process(
            "bwrap", f"/usr/bin/bwrap --bind {core.OUTPUT_ROOT} /work/output"
        ),
        "make_conflicts": is_conflicting_process("make", "/usr/bin/make -j1"),
    }
    if result["process_classifier"] != {
        "zotero_bwrap_conflicts": False,
        "liquid_bwrap_conflicts": True,
        "make_conflicts": True,
    }:
        raise core.GateError("v3 process-classifier self-check drift")
    return result


# Rebind only the reviewed classifier, self-check evidence, and this create-new
# script identity.  Every state transition, receipt producer, argv builder,
# resource limit and deadline remains the byte-pinned v2 implementation.
core.SCRIPT_PATH = SCRIPT_PATH
core.conflicting_processes = conflicting_processes_v3
core.self_check = self_check_v3


if __name__ == "__main__":
    raise SystemExit(core.main())
