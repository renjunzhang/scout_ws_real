#!/usr/bin/env python3
"""Fail-closed S5B0 finalized solver-output inventory and frame reader.

The public CLI is fixture-only.  ``build_manifest`` is a library interface and
requires an externally frozen exact root, inventory, time grid, particle count,
and particle-ID digest.  It never executes a candidate or solver.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import stat
import struct
import sys
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))
import r8_liquid_bi4_reader_v1 as bi4


ROOT = MODULE_DIR.parent
SCHEMA_PATH = ROOT / "schema/target_host_s5b0_finalized_solver_frames_manifest_v1.json"
PART_RE = re.compile(r"data/Part_([0-9]{4})\.bi4")
RUN_FILES = ("Run.csv", "Run.out", "RunPARTs.csv")
STATIC_BINDINGS = (
    "data/Part_Head.ibi4",
    "data/PartInfo.ibi4",
    "data/PartMotionRef.ibi4",
    "data/PartOut_000.obi4",
)
TIME_TOLERANCE_S = 1e-9
MAX_FILE_BYTES = 128 * 1024 * 1024
MAX_PART_BYTES = bi4.MAX_FILE_BYTES


class FinalizedFrameError(ValueError):
    """The finalized output differs from an external frozen contract."""


def canonical_json(value: Any) -> bytes:
    return json.dumps(
        value, allow_nan=False, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value)).hexdigest()


def ids_sha256(ids: Sequence[int]) -> str:
    digest = hashlib.sha256()
    for value in ids:
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**64:
            raise FinalizedFrameError("particle ID is not canonical uint64")
        digest.update(struct.pack("<Q", value))
    return digest.hexdigest()


def assert_deep_closed(node: object, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise FinalizedFrameError(f"open schema object: {location}")
        for key, value in node.items():
            assert_deep_closed(value, f"{location}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            assert_deep_closed(value, f"{location}/{index}")


def _sha(value: object, label: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise FinalizedFrameError(f"invalid SHA-256: {label}")
    return value


def _absolute_exact(path: Path, label: str) -> Path:
    text = str(path)
    if not path.is_absolute() or text != os.path.normpath(text):
        raise FinalizedFrameError(f"{label} is not an exact normalized absolute path")
    return path


def _assert_no_symlink_components(path: Path) -> None:
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        try:
            metadata = os.lstat(current)
        except FileNotFoundError as exc:
            raise FinalizedFrameError(f"root component is absent: {current}") from exc
        if stat.S_ISLNK(metadata.st_mode):
            raise FinalizedFrameError(f"symlink root component: {current}")
        if current != path and not stat.S_ISDIR(metadata.st_mode):
            raise FinalizedFrameError(f"non-directory root component: {current}")
    final = os.lstat(path)
    if not stat.S_ISDIR(final.st_mode):
        raise FinalizedFrameError("output root is not a real directory")


def _metadata(metadata: os.stat_result) -> dict[str, int]:
    return {
        "mode_bits": metadata.st_mode,
        "size_bytes": metadata.st_size,
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
        "mtime_ns": metadata.st_mtime_ns,
    }


def _scan_tree(root_fd: int) -> tuple[dict[str, dict[str, int]], set[str]]:
    files: dict[str, dict[str, int]] = {}
    directories: set[str] = set()

    def walk(directory_fd: int, prefix: str) -> None:
        try:
            names = sorted(os.listdir(directory_fd))
        except OSError as exc:
            raise FinalizedFrameError("cannot scan anchored output directory") from exc
        for name in names:
            if not name or name in {".", ".."} or "/" in name or "\0" in name:
                raise FinalizedFrameError("unsafe output entry name")
            relative = f"{prefix}/{name}" if prefix else name
            observed = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            if stat.S_ISLNK(observed.st_mode):
                raise FinalizedFrameError(f"symlink output entry: {relative}")
            if stat.S_ISDIR(observed.st_mode):
                directories.add(relative)
                child = os.open(
                    name,
                    os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                    dir_fd=directory_fd,
                )
                try:
                    anchored = os.fstat(child)
                    if (anchored.st_dev, anchored.st_ino) != (observed.st_dev, observed.st_ino):
                        raise FinalizedFrameError(f"directory TOCTOU detected: {relative}")
                    walk(child, relative)
                finally:
                    os.close(child)
            elif stat.S_ISREG(observed.st_mode):
                if observed.st_nlink != 1:
                    raise FinalizedFrameError(f"hard-linked output file: {relative}")
                files[relative] = _metadata(observed)
            else:
                raise FinalizedFrameError(f"special output entry: {relative}")
    walk(root_fd, "")
    return files, directories


def _open_file_at(root_fd: int, relative: str) -> int:
    parts = PurePosixPath(relative).parts
    directory_fd = os.dup(root_fd)
    try:
        for part in parts[:-1]:
            child = os.open(
                part,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=directory_fd,
            )
            os.close(directory_fd)
            directory_fd = child
        return os.open(
            parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=directory_fd
        )
    finally:
        os.close(directory_fd)


def _identity(relative: str, metadata: Mapping[str, int], digest: str) -> dict[str, Any]:
    return {
        "relative_path": relative,
        "type": "regular",
        "mode": f"{stat.S_IMODE(metadata['mode_bits']):04o}",
        "size_bytes": metadata["size_bytes"],
        "device": metadata["device"],
        "inode": metadata["inode"],
        "nlink": metadata["nlink"],
        "sha256": digest,
    }


def _read_regular_at(
    root_fd: int, relative: str, scanned: Mapping[str, int], *, maximum: int
) -> tuple[bytes, dict[str, Any]]:
    descriptor = _open_file_at(root_fd, relative)
    try:
        before = _metadata(os.fstat(descriptor))
        if before != dict(scanned):
            raise FinalizedFrameError(f"file changed before open: {relative}")
        if not stat.S_ISREG(before["mode_bits"]) or before["nlink"] != 1:
            raise FinalizedFrameError(f"unsafe opened file: {relative}")
        if before["size_bytes"] > maximum:
            raise FinalizedFrameError(f"file exceeds bound: {relative}")
        chunks: list[bytes] = []
        digest = hashlib.sha256()
        total = 0
        while True:
            block = os.read(descriptor, min(1 << 20, maximum + 1 - total))
            if not block:
                break
            total += len(block)
            if total > maximum:
                raise FinalizedFrameError(f"file exceeds bound while reading: {relative}")
            digest.update(block)
            chunks.append(block)
        after = _metadata(os.fstat(descriptor))
        if before != after or total != before["size_bytes"]:
            raise FinalizedFrameError(f"file TOCTOU detected: {relative}")
        value = digest.hexdigest()
        return b"".join(chunks), _identity(relative, before, value)
    finally:
        os.close(descriptor)


def _normalise_inventory(expected: Mapping[str, str]) -> dict[str, str]:
    if not isinstance(expected, Mapping) or not expected:
        raise FinalizedFrameError("expected inventory is empty")
    result: dict[str, str] = {}
    for raw_path, raw_digest in expected.items():
        if not isinstance(raw_path, str):
            raise FinalizedFrameError("inventory path is not text")
        pure = PurePosixPath(raw_path)
        if (
            pure.is_absolute()
            or str(pure) != raw_path
            or not pure.parts
            or any(part in {"", ".", ".."} for part in pure.parts)
        ):
            raise FinalizedFrameError(f"unsafe inventory path: {raw_path!r}")
        result[raw_path] = _sha(raw_digest, raw_path)
    return dict(sorted(result.items()))


def _expected_directories(paths: Sequence[str]) -> set[str]:
    result: set[str] = set()
    for value in paths:
        parts = PurePosixPath(value).parts[:-1]
        for end in range(1, len(parts) + 1):
            result.add("/".join(parts[:end]))
    return result


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise FinalizedFrameError(f"non-finite {label}")
    return float(value)


def _parse_frame(
    data: bytes,
    index: int,
    expected_time_s: float,
    expected_particle_count: int,
    expected_ids_sha256: str,
) -> dict[str, Any]:
    try:
        root = bi4.parse_jpartdata_bi4(data)
        particles = bi4.extract_u3_particles(root)
        part = root.items[0]
        cpart = bi4.require_int(part.values, "Cpart")
        nout = bi4.require_int(part.values, "Nout")
    except bi4.Bi4FormatError as exc:
        raise FinalizedFrameError(f"invalid Part_{index:04d}.bi4: {exc}") from exc
    if part.name != f"PART_{index:04d}" or cpart != index:
        raise FinalizedFrameError(f"Part internal index differs: {index}")
    time_s = _finite_number(part.values.get("TimeStep"), "PART TimeStep")
    if abs(time_s - expected_time_s) > TIME_TOLERANCE_S:
        raise FinalizedFrameError(f"Part time differs from external grid: {index}")
    if nout != 0:
        raise FinalizedFrameError(f"Part Nout is non-zero: {index}")
    if particles["particle_count"] != expected_particle_count:
        raise FinalizedFrameError(f"particle count drift: {index}")
    observed_ids = ids_sha256(particles["ids"])
    if observed_ids != expected_ids_sha256:
        raise FinalizedFrameError(f"particle ID drift: {index}")
    finite = (
        all(math.isfinite(float(value)) for row in particles["positions_m"] for value in row)
        and all(math.isfinite(float(value)) for row in particles["velocities_m_s"] for value in row)
        and all(math.isfinite(float(value)) for value in particles["densities_kg_m3"])
    )
    if not finite:
        raise FinalizedFrameError(f"non-finite particle value: {index}")
    return {
        "index": index,
        "time_s": time_s,
        "particle_count": particles["particle_count"],
        "nout": nout,
        "ids_sha256": observed_ids,
        "finite": True,
        "class_counts": particles["counts"],
    }


def build_manifest(
    output_root: Path,
    *,
    expected_root: Path,
    expected_inventory: Mapping[str, str],
    expected_start_index: int,
    expected_times_s: Sequence[float],
    expected_particle_count: int,
    expected_ids_sha256: str,
) -> dict[str, Any]:
    """Read one exact frozen output tree and return a validated manifest."""

    output_root = _absolute_exact(Path(output_root), "output_root")
    expected_root = _absolute_exact(Path(expected_root), "expected_root")
    if output_root != expected_root:
        raise FinalizedFrameError("output root differs from exact external root")
    _assert_no_symlink_components(output_root)
    if isinstance(expected_start_index, bool) or not isinstance(expected_start_index, int) or expected_start_index < 0:
        raise FinalizedFrameError("expected start index is invalid")
    if isinstance(expected_particle_count, bool) or not isinstance(expected_particle_count, int) or expected_particle_count < 1:
        raise FinalizedFrameError("expected particle count is invalid")
    expected_ids_sha256 = _sha(expected_ids_sha256, "expected_ids_sha256")
    times = [_finite_number(value, "expected time") for value in expected_times_s]
    if not times or any(right <= left for left, right in zip(times, times[1:])):
        raise FinalizedFrameError("expected time grid is empty or non-increasing")
    inventory = _normalise_inventory(expected_inventory)
    for required in (*RUN_FILES, *STATIC_BINDINGS):
        if required not in inventory:
            raise FinalizedFrameError(f"required output binding is absent: {required}")
    part_paths = sorted(
        (path for path in inventory if PART_RE.fullmatch(path)),
        key=lambda path: int(PART_RE.fullmatch(path).group(1)),
    )
    malformed_parts = [
        path for path in inventory
        if path.startswith("data/Part_") and path.endswith(".bi4") and not PART_RE.fullmatch(path)
    ]
    if malformed_parts:
        raise FinalizedFrameError("malformed Part filename in inventory")
    expected_indices = list(range(expected_start_index, expected_start_index + len(times)))
    observed_indices = [int(PART_RE.fullmatch(path).group(1)) for path in part_paths]
    if observed_indices != expected_indices or len(part_paths) != len(times):
        raise FinalizedFrameError("Part index sequence differs from external grid")

    root_fd = os.open(
        output_root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    )
    try:
        root_before = _metadata(os.fstat(root_fd))
        first_files, first_directories = _scan_tree(root_fd)
        if set(first_files) != set(inventory):
            raise FinalizedFrameError("output file inventory has missing or extra entries")
        if first_directories != _expected_directories(list(inventory)):
            raise FinalizedFrameError("output directory inventory has missing or extra entries")
        identities: dict[str, dict[str, Any]] = {}
        part_payloads: dict[str, bytes] = {}
        for relative, expected_digest in inventory.items():
            is_part = PART_RE.fullmatch(relative) is not None
            payload, identity = _read_regular_at(
                root_fd,
                relative,
                first_files[relative],
                maximum=MAX_PART_BYTES if is_part else MAX_FILE_BYTES,
            )
            if identity["sha256"] != expected_digest:
                raise FinalizedFrameError(f"external inventory SHA differs: {relative}")
            identities[relative] = identity
            if is_part:
                part_payloads[relative] = payload

        frames: list[dict[str, Any]] = []
        class_counts: dict[str, int] | None = None
        for position, relative in enumerate(part_paths):
            index = expected_indices[position]
            frame = _parse_frame(
                part_payloads[relative], index, times[position],
                expected_particle_count, expected_ids_sha256,
            )
            if class_counts is None:
                class_counts = dict(frame["class_counts"])
            elif frame["class_counts"] != class_counts:
                raise FinalizedFrameError(f"particle class count drift: {index}")
            frame["identity"] = identities[relative]
            frames.append(frame)

        final_files, final_directories = _scan_tree(root_fd)
        root_after = _metadata(os.fstat(root_fd))
        if root_before != root_after or first_files != final_files or first_directories != final_directories:
            raise FinalizedFrameError("output tree TOCTOU detected")
    finally:
        os.close(root_fd)

    file_records = [identities[path] for path in sorted(identities)]
    manifest = {
        "schema_version": "smpcc-r8-liquid-s5b0-finalized-solver-frames-manifest-v1",
        "document_type": "SMPCC_R8_LIQUID_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1",
        "status": "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1",
        "root": str(output_root),
        "contract": {
            "expected_root": str(expected_root),
            "expected_inventory_sha256": canonical_sha256([
                {"relative_path": path, "sha256": digest}
                for path, digest in inventory.items()
            ]),
            "expected_start_index": expected_start_index,
            "expected_frame_count": len(times),
            "expected_times_sha256": canonical_sha256(times),
            "expected_particle_count": expected_particle_count,
            "expected_ids_sha256": expected_ids_sha256,
            "time_tolerance_s": TIME_TOLERANCE_S,
        },
        "inventory": {
            "file_count": len(file_records),
            "directory_count": len(first_directories),
            "canonical_sha256": canonical_sha256(file_records),
            "files": file_records,
        },
        "bindings": {
            "run_files": [identities[path] for path in RUN_FILES],
            "part_head": identities[STATIC_BINDINGS[0]],
            "part_info": identities[STATIC_BINDINGS[1]],
            "part_motion_ref": identities[STATIC_BINDINGS[2]],
            "part_out": identities[STATIC_BINDINGS[3]],
        },
        "frames": frames,
        "integrity": {
            "exact_root": True,
            "exact_inventory": True,
            "no_symlinks": True,
            "no_hardlinks": True,
            "no_special_files": True,
            "no_toctou": True,
            "frame_indices_contiguous": True,
            "frame_times_match_external_grid": True,
            "particle_count_stable": True,
            "particle_ids_stable": True,
            "nout_zero": True,
            "finite": True,
            "frame_manifest_sha256": canonical_sha256(frames),
        },
        "claims": {
            "candidate_executed_by_reader": False,
            "solver_executed_by_reader": False,
            "gpu_exposed": False,
            "network_used": False,
            "sudo_used": False,
            "apparmor_loaded": False,
            "real_bag_read": False,
        },
    }
    schema = json.loads(SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    Draft202012Validator(schema).validate(manifest)
    return manifest


def _fixture_string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def _fixture_value(name: str, type_code: int, payload: bytes) -> bytes:
    return _fixture_string(name) + struct.pack("<i", type_code) + payload


def _fixture_array(name: str, type_code: int, count: int, payload: bytes) -> bytes:
    definition = (
        _fixture_string("\nARRAY") + _fixture_string(name)
        + struct.pack("<iiII", 0, type_code, count, len(payload))
    )
    return struct.pack("<I", len(definition)) + definition + payload


def _fixture_item(
    name: str, values: Sequence[bytes], *, arrays: Sequence[bytes] = (), children: Sequence[bytes] = ()
) -> bytes:
    value_block = _fixture_string("\nVALUES") + struct.pack("<I", len(values)) + b"".join(values)
    definition = (
        _fixture_string("\nITEM\n") + _fixture_string(name) + struct.pack("<ii", 0, 0)
        + _fixture_string("%.7E") + _fixture_string("%.15E")
        + struct.pack("<III", len(arrays), len(children), len(value_block))
    )
    return (
        struct.pack("<I", len(definition)) + definition + value_block
        + b"".join(arrays) + b"".join(children)
    )


def _fixture_part_bytes(
    index: int,
    time_s: float,
    *,
    ids: Sequence[int] = (0, 1, 2, 3),
    nout: int = 0,
    nonfinite: bool = False,
    class_counts: tuple[int, int, int, int] = (1, 1, 0, 2),
) -> bytes:
    ids = tuple(ids)
    positions = [(float(value), 0.0, 0.01 * value) for value in ids]
    velocities = [(float("nan") if nonfinite and pos == 0 else 0.0, 0.0, 0.0) for pos, _ in enumerate(ids)]
    densities = [1000.0] * len(ids)
    arrays = (
        _fixture_array("Idp", 8, len(ids), b"".join(struct.pack("<I", value) for value in ids)),
        _fixture_array("Posd", 23, len(ids), b"".join(struct.pack("<ddd", *row) for row in positions)),
        _fixture_array("Vel", 22, len(ids), b"".join(struct.pack("<fff", *row) for row in velocities)),
        _fixture_array("Rhop", 11, len(ids), b"".join(struct.pack("<f", value) for value in densities)),
    )
    part = _fixture_item(
        f"PART_{index:04d}",
        (
            _fixture_value("Cpart", 8, struct.pack("<I", index)),
            _fixture_value("TimeStep", 12, struct.pack("<d", time_s)),
            _fixture_value("Npok", 8, struct.pack("<I", len(ids))),
            _fixture_value("Nout", 8, struct.pack("<I", nout)),
            _fixture_value("Step", 8, struct.pack("<I", index)),
            _fixture_value("RunTime", 12, struct.pack("<d", 0.0)),
        ),
        arrays=arrays,
    )
    fixed, moving, floating, fluid = class_counts
    root = _fixture_item(
        "JPartDataBi4",
        (
            _fixture_value("CaseNp", 10, struct.pack("<Q", len(ids))),
            _fixture_value("CaseNfixed", 10, struct.pack("<Q", fixed)),
            _fixture_value("CaseNmoving", 10, struct.pack("<Q", moving)),
            _fixture_value("CaseNfloat", 10, struct.pack("<Q", floating)),
            _fixture_value("CaseNfluid", 10, struct.pack("<Q", fluid)),
        ),
        children=(part,),
    )
    header = b"#FileJBD JPartDataBi4".ljust(58, b" ") + b"\n\0\0\0\0\0"
    return header + root


def _write_fixture_tree(root: Path, indices: Sequence[int], times: Sequence[float]) -> dict[str, str]:
    root.mkdir(mode=0o700)
    (root / "data").mkdir(mode=0o700)
    payloads: dict[str, bytes] = {
        "Run.csv": b"fixture run csv\n",
        "Run.out": b"fixture run out\n",
        "RunPARTs.csv": b"fixture run parts\n",
        "data/Part_Head.ibi4": b"fixture head",
        "data/PartInfo.ibi4": b"fixture info",
        "data/PartMotionRef.ibi4": b"fixture motion ref",
        "data/PartOut_000.obi4": b"fixture part out",
    }
    for index, time_s in zip(indices, times):
        payloads[f"data/Part_{index:04d}.bi4"] = _fixture_part_bytes(index, time_s)
    for relative, payload in payloads.items():
        path = root / relative
        path.write_bytes(payload)
        path.chmod(0o600)
    return {path: hashlib.sha256(payload).hexdigest() for path, payload in payloads.items()}


def self_check() -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="r8-s5b0-finalized-frame-fixture-") as temporary:
        fixture_root = Path(temporary) / "output"
        times = [1.0, 1.05]
        inventory = _write_fixture_tree(fixture_root, [7, 8], times)
        manifest = build_manifest(
            fixture_root,
            expected_root=fixture_root,
            expected_inventory=inventory,
            expected_start_index=7,
            expected_times_s=times,
            expected_particle_count=4,
            expected_ids_sha256=ids_sha256([0, 1, 2, 3]),
        )
        frame_hash = manifest["integrity"]["frame_manifest_sha256"]
    return {
        "status": "PASS_S5B0_FINALIZED_FRAME_READER_V1_FIXTURE_SELF_CHECK",
        "manifest_status": "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1",
        "fixture_frame_count": 2,
        "frame_manifest_sha256": frame_hash,
        "fixture_root_removed": not fixture_root.exists(),
        "real_solver_output_read": False,
        "real_bag_read": False,
        "candidate_executed": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
        "sudo_used": False,
        "apparmor_loaded": False,
        "files_written_outside_fixture": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_FINALIZED_FRAME_READER_V1", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
