#!/usr/bin/env python3
"""Confined, byte-pinned six-file Motion-Gauge patch child v11.

Every target is replaced atomically through a same-directory create-new inode.
The six replacements are deliberately *not* represented as one cross-file
transaction: stdout is an append-only JSONL journal, and any partial transition
must be preserved and rejected by the outer 6-changed/346-unchanged gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
import types
from pathlib import Path
from typing import Any, Mapping, MutableSequence, Sequence


sys.dont_write_bytecode = True
EXACT_SOURCE_ROOT = Path("/work/source")
EXACT_PATCH_MODULE = Path("/work/patch_v1.py")
EXACT_PATCH_MODULE_SHA256 = "601682672f39ccd19533149024e7531ec54157bd058368f7f40e63a6f894f3cf"
EXACT_AGGREGATE_SHA256 = "3cf3e7d883f7c751d7b5455eec352c6df46ff669cf6b5a989471be5f7731f528"
EXPECTED_HASHES = {
    "JDsGaugeItem.cpp": (
        "909d0accec54daf2b767d7ab99338ff92ff15b49e9a0bf4c221d7dc6edecb0a4",
        "283793b308f07579affdf428cf267f62f9ab033c75d25cb54e4620f7d2bf3d01",
    ),
    "JDsGaugeItem.h": (
        "4ceee0be27e86b0686d5530574ab79bd8a99a4f64596b474c9234b8da6df2823",
        "8ff282de107eeb50691cda2ea77cbf6ec507e73f964231f446e4711a2c46ae2e",
    ),
    "JDsGaugeSystem.cpp": (
        "1ccef263531c3757d513284c7187a4e14fec43c88fc1ff4b5ba4092cca2e1e80",
        "3dec0381b413f9f736242036abc4b60599dcaf724edddc6202df17de4079ad5e",
    ),
    "JDsGaugeSystem.h": (
        "473831ed3614277ca7e82f2b2ce065a0c37c6039ecbaf9cfcaddef624a2ff86d",
        "4d97c40b14105f9c4e0df372bff9229b6c8daf881686c4d50f64a7fc0a5e6c6c",
    ),
    "JSph.cpp": (
        "206e4486a3a0d304e02da1a73d2e7c7ed96354d559ce481275d09f5245b8c9e7",
        "07e197ff8661d91a6e9b30053dfc0b8b138b6293892c6dc72aa2c0031b63676f",
    ),
    "JSph.h": (
        "abb087c020641d94c3968507f0ce067da4f45cac246ff6bc586d4e8665e39eda",
        "d82074fd35998a96585c6698b09cb8bbddce79835fed4d3e07683270633b08fb",
    ),
}
EXACT_NAMES = tuple(sorted(EXPECTED_HASHES))
TEMP_PREFIX = ".r8-motion-gauge-v11-"


class PatchChildError(RuntimeError):
    """The confined transition differs from the frozen contract."""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def write_all(descriptor: int, value: bytes) -> None:
    offset = 0
    while offset < len(value):
        written = os.write(descriptor, value[offset:])
        if written <= 0:
            raise PatchChildError("short write while staging atomic replacement")
        offset += written


def read_all(descriptor: int, expected_size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = expected_size
    while remaining:
        block = os.read(descriptor, min(1 << 20, remaining))
        if not block:
            raise PatchChildError("short read from authenticated regular file")
        chunks.append(block)
        remaining -= len(block)
    if os.read(descriptor, 1):
        raise PatchChildError("regular file grew during authenticated read")
    return b"".join(chunks)


def identity(info: os.stat_result, raw: bytes, *, path: str) -> dict[str, Any]:
    return {
        "path": path,
        "device": info.st_dev,
        "inode": info.st_ino,
        "mode_octal": format(stat.S_IMODE(info.st_mode), "04o"),
        "nlink": info.st_nlink,
        "uid": info.st_uid,
        "gid": info.st_gid,
        "size_bytes": info.st_size,
        "sha256": sha256_bytes(raw),
    }


def read_regular_absolute(path: Path) -> tuple[bytes, dict[str, Any]]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise PatchChildError(f"unsafe authenticated input: {path}")
        raw = read_all(descriptor, before.st_size)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    ):
        raise PatchChildError(f"authenticated input drifted while read: {path}")
    return raw, identity(after, raw, path=str(path))


def read_regular_at(directory_fd: int, name: str) -> tuple[bytes, dict[str, Any]]:
    descriptor = os.open(
        name,
        os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC,
        dir_fd=directory_fd,
    )
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise PatchChildError(f"unsafe patch target: {name}")
        raw = read_all(descriptor, before.st_size)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    ):
        raise PatchChildError(f"patch target drifted while read: {name}")
    return raw, identity(after, raw, path=name)


def same_inode(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    fields = ("device", "inode", "mode_octal", "nlink", "uid", "gid", "size_bytes", "sha256")
    return all(left[field] == right[field] for field in fields)


def authenticated_patch_module(path: Path) -> tuple[types.ModuleType, dict[str, Any]]:
    raw, module_identity = read_regular_absolute(path)
    if module_identity["sha256"] != EXACT_PATCH_MODULE_SHA256:
        raise PatchChildError("frozen patch module SHA-256 differs")
    module = types.ModuleType("r8_motion_patch_v1_authenticated_v11")
    module.__file__ = str(path)
    module.__package__ = None
    exec(compile(raw, str(path), "exec", dont_inherit=True, optimize=0), module.__dict__)
    for attribute in ("apply_in_memory", "aggregate_source_hash", "BEFORE_SHA256"):
        if not hasattr(module, attribute):
            raise PatchChildError(f"authenticated patch module lacks {attribute}")
    if dict(module.BEFORE_SHA256) != {name: hashes[0] for name, hashes in EXPECTED_HASHES.items()}:
        raise PatchChildError("authenticated patch module preimage map differs")
    return module, module_identity


def emit(journal: MutableSequence[dict[str, Any]], event: Mapping[str, Any]) -> None:
    record = dict(event)
    record["sequence"] = len(journal) + 1
    journal.append(record)
    print(json.dumps(record, sort_keys=True, separators=(",", ":")), flush=True)


def temp_name(name: str) -> str:
    return f"{TEMP_PREFIX}{name}.partial"


def replace_one_atomic(
    directory_fd: int,
    name: str,
    expected_before: Mapping[str, Any],
    after: bytes,
) -> dict[str, Any]:
    expected_mode = int(expected_before["mode"])
    current_raw, current_identity = read_regular_at(directory_fd, name)
    if not same_inode(expected_before, current_identity):
        raise PatchChildError(f"target identity changed before replacement: {name}")
    if current_raw != expected_before["raw"]:
        raise PatchChildError(f"target bytes changed before replacement: {name}")

    staged_name = temp_name(name)
    descriptor = os.open(
        staged_name,
        os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        expected_mode,
        dir_fd=directory_fd,
    )
    try:
        os.fchmod(descriptor, expected_mode)
        write_all(descriptor, after)
        os.fsync(descriptor)
        os.lseek(descriptor, 0, os.SEEK_SET)
        staged_info = os.fstat(descriptor)
        staged_raw = read_all(descriptor, staged_info.st_size)
        if staged_raw != after or sha256_bytes(staged_raw) != EXPECTED_HASHES[name][1]:
            raise PatchChildError(f"staged postimage differs: {name}")
    finally:
        os.close(descriptor)

    final_pre_raw, final_pre_identity = read_regular_at(directory_fd, name)
    if final_pre_raw != current_raw or not same_inode(current_identity, final_pre_identity):
        raise PatchChildError(f"target changed at atomic-rename boundary: {name}")
    os.replace(staged_name, name, src_dir_fd=directory_fd, dst_dir_fd=directory_fd)
    os.fsync(directory_fd)
    observed_raw, observed_identity = read_regular_at(directory_fd, name)
    if observed_raw != after or observed_identity["sha256"] != EXPECTED_HASHES[name][1]:
        raise PatchChildError(f"atomic replacement postcondition differs: {name}")
    if observed_identity["mode_octal"] != expected_before["mode_octal"]:
        raise PatchChildError(f"atomic replacement mode drift: {name}")
    if (observed_identity["uid"], observed_identity["gid"]) != (
        expected_before["uid"],
        expected_before["gid"],
    ):
        raise PatchChildError(f"atomic replacement ownership drift: {name}")
    return {
        "event": "ATOMIC_FILE_REPLACED",
        "path": name,
        "atomic_per_file": True,
        "cross_file_transaction": False,
        "before": {key: value for key, value in expected_before.items() if key not in {"raw", "mode"}},
        "after": observed_identity,
        "staging_name": staged_name,
    }


def run(
    source_root: Path,
    patch_module: Path,
    journal: MutableSequence[dict[str, Any]],
) -> dict[str, Any]:
    if source_root != EXACT_SOURCE_ROOT or patch_module != EXACT_PATCH_MODULE:
        raise PatchChildError("confined patch path identity differs")
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups():
        raise PatchChildError("patch child is not uid/gid 1000 with zero supplementary groups")
    emit(
        journal,
        {
            "event": "PATCH_START",
            "status": "STARTED_CONFINED_EXACT_SIX_FILE_PATCH_V11",
            "source_root": str(source_root),
            "patch_module": str(patch_module),
            "patch_module_expected_sha256": EXACT_PATCH_MODULE_SHA256,
            "atomic_per_file": True,
            "cross_file_transaction": False,
        },
    )
    source_info = os.lstat(source_root)
    if not stat.S_ISDIR(source_info.st_mode) or stat.S_ISLNK(source_info.st_mode):
        raise PatchChildError("confined source root is not a real directory")

    module, module_identity = authenticated_patch_module(patch_module)
    directory_fd = os.open(source_root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    completed: list[str] = []
    try:
        before_raw: dict[str, bytes] = {}
        before_state: dict[str, dict[str, Any]] = {}
        for name in EXACT_NAMES:
            raw, item = read_regular_at(directory_fd, name)
            if item["sha256"] != EXPECTED_HASHES[name][0]:
                raise PatchChildError(f"frozen preimage differs: {name}")
            before_raw[name] = raw
            before_state[name] = {
                **item,
                "raw": raw,
                "mode": int(item["mode_octal"], 8),
            }
        after_raw = module.apply_in_memory(before_raw)
        if set(after_raw) != set(EXACT_NAMES):
            raise PatchChildError("authenticated patch output file set differs")
        for name in EXACT_NAMES:
            if not isinstance(after_raw[name], bytes) or sha256_bytes(after_raw[name]) != EXPECTED_HASHES[name][1]:
                raise PatchChildError(f"frozen postimage differs before materialization: {name}")
        if module.aggregate_source_hash(after_raw) != EXACT_AGGREGATE_SHA256:
            raise PatchChildError("frozen six-file aggregate differs before materialization")

        emit(
            journal,
            {
                "event": "PATCH_PLAN_AUTHENTICATED",
                "status": "READY_EXACT_SIX_FILE_PATCH_V11",
                "atomic_per_file": True,
                "cross_file_transaction": False,
                "failure_policy": "PRESERVE_PARTIAL_AND_REJECT_OUTER_INVENTORY",
                "patch_module": module_identity,
                "files": [
                    {
                        "path": name,
                        "before_sha256": EXPECTED_HASHES[name][0],
                        "after_sha256": EXPECTED_HASHES[name][1],
                    }
                    for name in EXACT_NAMES
                ],
            },
        )
        for name in EXACT_NAMES:
            event = replace_one_atomic(directory_fd, name, before_state[name], after_raw[name])
            completed.append(name)
            emit(journal, {**event, "completed_count": len(completed)})

        observed = {name: read_regular_at(directory_fd, name)[0] for name in EXACT_NAMES}
        if module.aggregate_source_hash(observed) != EXACT_AGGREGATE_SHA256:
            raise PatchChildError("materialized six-file aggregate differs")
    except Exception as exc:
        emit(
            journal,
            {
                "event": "PATCH_FAILURE",
                "status": "FAIL_CONFINED_EXACT_SIX_FILE_PATCH_V11",
                "error": str(exc),
                "completed_files": completed,
                "completed_count": len(completed),
                "cross_file_transaction": False,
                "partial_tree_must_be_preserved": bool(completed),
            },
        )
        raise
    finally:
        os.close(directory_fd)

    result = {
        "event": "PATCH_FINAL",
        "status": "PASS_CONFINED_EXACT_SIX_FILE_PATCH_V11",
        "changed_count": 6,
        "completed_files": completed,
        "six_file_aggregate_sha256": EXACT_AGGREGATE_SHA256,
        "patch_module_sha256": EXACT_PATCH_MODULE_SHA256,
        "atomic_per_file": True,
        "cross_file_transaction": False,
        "compiler_run": False,
        "candidate_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }
    emit(journal, result)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default=str(EXACT_SOURCE_ROOT))
    parser.add_argument("--patch-module", default=str(EXACT_PATCH_MODULE))
    args = parser.parse_args(argv)
    journal: list[dict[str, Any]] = []
    try:
        run(Path(args.source_root), Path(args.patch_module), journal)
    except Exception as exc:
        if not journal or journal[-1].get("event") != "PATCH_FAILURE":
            emit(
                journal,
                {
                    "event": "PATCH_FAILURE",
                    "status": "FAIL_CONFINED_EXACT_SIX_FILE_PATCH_V11",
                    "error": str(exc),
                    "completed_files": [],
                    "completed_count": 0,
                    "cross_file_transaction": False,
                    "partial_tree_must_be_preserved": False,
                },
            )
        print(
            json.dumps(
                {
                    "status": "FAIL_CONFINED_EXACT_SIX_FILE_PATCH_V11",
                    "error": str(exc),
                    "journal_records": len(journal),
                },
                sort_keys=True,
                separators=(",", ":"),
            ),
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
