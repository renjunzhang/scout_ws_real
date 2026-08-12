#!/usr/bin/env python3
"""Apply the frozen six-file motion-attached Gauge patch inside a sandbox.

This helper has no subprocess, network, compiler, candidate, or receipt surface.
The outer v10 gate binds only this file, the byte-pinned v1 patch module, and a
fresh source copy into the disconnected patch sandbox.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import stat
import sys
from pathlib import Path
from typing import Mapping, Sequence


sys.dont_write_bytecode = True
EXACT_SOURCE_ROOT = Path("/work/source")
EXACT_PATCH_MODULE = Path("/work/patch_v1.py")
EXACT_AGGREGATE_SHA256 = "3cf3e7d883f7c751d7b5455eec352c6df46ff669cf6b5a989471be5f7731f528"
EXACT_NAMES = (
    "JDsGaugeItem.cpp",
    "JDsGaugeItem.h",
    "JDsGaugeSystem.cpp",
    "JDsGaugeSystem.h",
    "JSph.cpp",
    "JSph.h",
)


class PatchChildError(RuntimeError):
    """The confined patch transition is not the exact frozen transition."""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def read_regular(path: Path) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise PatchChildError(f"unsafe patch input: {path}")
        chunks: list[bytes] = []
        remaining = info.st_size
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                raise PatchChildError(f"short patch input read: {path}")
            chunks.append(block)
            remaining -= len(block)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def replace_exact(path: Path, before: bytes, after: bytes) -> dict[str, object]:
    descriptor = os.open(path, os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise PatchChildError(f"unsafe patch target: {path}")
        actual = os.read(descriptor, info.st_size)
        if actual != before:
            raise PatchChildError(f"patch preimage differs: {path.name}")
        os.ftruncate(descriptor, 0)
        os.lseek(descriptor, 0, os.SEEK_SET)
        offset = 0
        while offset < len(after):
            written = os.write(descriptor, after[offset:])
            if written <= 0:
                raise PatchChildError(f"short patch write: {path.name}")
            offset += written
        os.fsync(descriptor)
        final = os.fstat(descriptor)
        os.lseek(descriptor, 0, os.SEEK_SET)
        observed = os.read(descriptor, final.st_size)
        if observed != after:
            raise PatchChildError(f"patch postimage differs: {path.name}")
        return {
            "path": path.name,
            "before_sha256": sha256_bytes(before),
            "after_sha256": sha256_bytes(after),
            "size_bytes": final.st_size,
            "mode_octal": format(stat.S_IMODE(final.st_mode), "04o"),
            "inode": final.st_ino,
        }
    finally:
        os.close(descriptor)


def load_patch_module(path: Path):
    spec = importlib.util.spec_from_file_location("r8_motion_patch_v1_confined", path)
    if spec is None or spec.loader is None:
        raise PatchChildError("cannot load the confined patch module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def aggregate(module: object, values: Mapping[str, bytes]) -> str:
    result = module.aggregate_source_hash(values)
    if not isinstance(result, str):
        raise PatchChildError("patch aggregate is not a SHA-256 string")
    return result


def run(source_root: Path, patch_module: Path) -> dict[str, object]:
    if source_root != EXACT_SOURCE_ROOT or patch_module != EXACT_PATCH_MODULE:
        raise PatchChildError("confined patch path identity differs")
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups():
        raise PatchChildError("patch child is not uid/gid 1000 with zero groups")
    module = load_patch_module(patch_module)
    before = {name: read_regular(source_root / name) for name in EXACT_NAMES}
    after = module.apply_in_memory(before)
    if tuple(sorted(after)) != tuple(sorted(EXACT_NAMES)):
        raise PatchChildError("patch output file set differs")
    if aggregate(module, after) != EXACT_AGGREGATE_SHA256:
        raise PatchChildError("frozen six-file aggregate differs")
    entries = [replace_exact(source_root / name, before[name], after[name]) for name in EXACT_NAMES]
    observed = {name: read_regular(source_root / name) for name in EXACT_NAMES}
    if aggregate(module, observed) != EXACT_AGGREGATE_SHA256:
        raise PatchChildError("materialized six-file aggregate differs")
    return {
        "status": "PASS_CONFINED_EXACT_SIX_FILE_PATCH_V10",
        "changed_count": 6,
        "six_file_aggregate_sha256": EXACT_AGGREGATE_SHA256,
        "entries": entries,
        "compiler_run": False,
        "candidate_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default=str(EXACT_SOURCE_ROOT))
    parser.add_argument("--patch-module", default=str(EXACT_PATCH_MODULE))
    args = parser.parse_args(argv)
    try:
        result = run(Path(args.source_root), Path(args.patch_module))
    except (PatchChildError, OSError, ValueError, KeyError, TypeError) as exc:
        print(json.dumps({"status": "FAIL_CONFINED_PATCH_V10", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
