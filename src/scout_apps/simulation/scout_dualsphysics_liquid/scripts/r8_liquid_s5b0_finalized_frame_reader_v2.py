#!/usr/bin/env python3
"""Post-execution exact-inventory BI4 frame reader with ID normalization.

GPU Part files may store the same complete ID set in a different array order.
This reader sorts all particle arrays by ID, freezes the canonical 0..9077
digest, and binds every read to an external inventory receipt and RunPARTs.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
import re
import stat
import struct
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SCHEMA_PATH = ROOT / "schema/target_host_s5b0_finalized_solver_frames_manifest_v2.json"
V1_PATH = SCRIPTS / "r8_liquid_s5b0_finalized_frame_reader_v1.py"
BI4_PATH = SCRIPTS / "r8_liquid_bi4_reader_v1.py"
PART_RE = re.compile(r"data/Part_([0-9]{4})\.bi4")
RUN_FILES = ("Run.csv", "Run.out", "RunPARTs.csv")
STATIC_BINDINGS = ("data/Part_Head.ibi4", "data/PartInfo.ibi4",
                   "data/PartMotionRef.ibi4", "data/PartOut_000.obi4")
EXPECTED_PARTICLES = 9078
EXPECTED_CLASSES = {"fixed_boundary": 0, "moving_boundary": 2669,
                    "floating": 0, "fluid": 6409}
TIME_TOLERANCE_S = 2e-5
MAX_FILE_BYTES = 128 * 1024 * 1024


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


v1 = _load("s5b0_frame_v1_for_v2", V1_PATH)
bi4 = _load("s5b0_bi4_for_frame_v2", BI4_PATH)


class FinalizedFrameV2Error(ValueError):
    """A receipt, tree, BI4, ID, RunPARTs, or TOCTOU invariant failed."""


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":")).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value)).hexdigest()


def ids_sha256(ids: Sequence[int]) -> str:
    digest = hashlib.sha256()
    for value in ids:
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**64:
            raise FinalizedFrameV2Error("particle ID is not canonical uint64")
        digest.update(struct.pack("<Q", value))
    return digest.hexdigest()


CANONICAL_IDS_SHA256 = ids_sha256(list(range(EXPECTED_PARTICLES)))


def _sha(value: object, label: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise FinalizedFrameV2Error(f"invalid SHA-256: {label}")
    return value


def _metadata(value: os.stat_result) -> dict[str, int]:
    return {"mode_bits": value.st_mode, "size_bytes": value.st_size,
            "device": value.st_dev, "inode": value.st_ino, "nlink": value.st_nlink,
            "mtime_ns": value.st_mtime_ns, "ctime_ns": value.st_ctime_ns}


def _assert_root(root: Path) -> None:
    if not root.is_absolute() or str(root) != os.path.normpath(str(root)):
        raise FinalizedFrameV2Error("output root is not exact normalized absolute")
    current = Path(root.anchor)
    for part in root.parts[1:]:
        current /= part
        metadata = os.lstat(current)
        if stat.S_ISLNK(metadata.st_mode):
            raise FinalizedFrameV2Error(f"symlink root component: {current}")
    if not stat.S_ISDIR(os.lstat(root).st_mode):
        raise FinalizedFrameV2Error("output root is not a directory")


def _scan(root: Path) -> tuple[dict[str, dict[str, int]], set[str]]:
    files: dict[str, dict[str, int]] = {}
    directories: set[str] = set()
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root).as_posix()
        metadata = os.lstat(path)
        if stat.S_ISLNK(metadata.st_mode):
            raise FinalizedFrameV2Error(f"symlink output entry: {relative}")
        if stat.S_ISDIR(metadata.st_mode):
            directories.add(relative)
        elif stat.S_ISREG(metadata.st_mode):
            if metadata.st_nlink != 1:
                raise FinalizedFrameV2Error(f"hard-linked output entry: {relative}")
            files[relative] = _metadata(metadata)
        else:
            raise FinalizedFrameV2Error(f"special output entry: {relative}")
    return files, directories


def _read(root: Path, relative: str, scanned: Mapping[str, int], maximum: int) -> tuple[bytes, dict[str, Any]]:
    pure = PurePosixPath(relative)
    if pure.is_absolute() or any(part in ("", ".", "..") for part in pure.parts):
        raise FinalizedFrameV2Error("unsafe relative path")
    descriptor = os.open(root / relative, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = _metadata(os.fstat(descriptor))
        if before != dict(scanned) or not stat.S_ISREG(before["mode_bits"]) or before["nlink"] != 1:
            raise FinalizedFrameV2Error(f"file identity changed before read: {relative}")
        if not 0 < before["size_bytes"] <= maximum:
            raise FinalizedFrameV2Error(f"file size bound differs: {relative}")
        chunks: list[bytes] = []
        digest = hashlib.sha256()
        total = 0
        while total < before["size_bytes"]:
            block = os.read(descriptor, min(1 << 20, before["size_bytes"] - total))
            if not block:
                raise FinalizedFrameV2Error(f"short read: {relative}")
            total += len(block); digest.update(block); chunks.append(block)
        if os.read(descriptor, 1):
            raise FinalizedFrameV2Error(f"file grew during read: {relative}")
        after = _metadata(os.fstat(descriptor))
    finally:
        os.close(descriptor)
    if before != after:
        raise FinalizedFrameV2Error(f"file TOCTOU: {relative}")
    return b"".join(chunks), {
        "relative_path": relative, "type": "regular",
        "mode": f"{stat.S_IMODE(after['mode_bits']):04o}",
        "size_bytes": after["size_bytes"], "device": after["device"],
        "inode": after["inode"], "nlink": after["nlink"],
        "sha256": digest.hexdigest(),
    }


def parse_runparts(raw: bytes) -> list[dict[str, Any]]:
    """Parse the exact DualSPHysics restart-aware PART/Step relation.

    ``Steps`` is the number of calculations since the preceding saved PART.
    A restart stream begins with the cloned PART (for S5B0, PART 0901) and
    therefore has ``Steps=0`` and BI4 ``Step=0``.  DualSPHysics counts the
    first calculation on both sides of the first output interval; consequently
    the second PART's BI4 Step is ``Steps-1``.  Every later interval is added
    without another correction.  Freezing the rule here prevents accepting a
    plausible-looking restart whose CSV and BI4 step clocks are offset.
    """
    try:
        text = raw.decode("utf-8-sig", "strict")
    except UnicodeError as exc:
        raise FinalizedFrameV2Error("RunPARTs is not strict UTF-8") from exc
    lines = [line for line in text.splitlines()
             if line.strip() and not line.lstrip().startswith("#")]
    rows = list(csv.reader(io.StringIO("\n".join(lines)), delimiter=";", strict=True))
    if len(rows) < 2 or len(set(rows[0])) != len(rows[0]):
        raise FinalizedFrameV2Error("RunPARTs header/data is absent or duplicated")
    header = rows[0]
    required = ("Part", "TimeStep [s]", "Steps", "NpSave", "NpSim", "NpOut",
                "NpbSim", "NpfSim", "NpNormal", "NpOutPos", "NpOutRho", "NpOutMov")
    if any(name not in header for name in required):
        raise FinalizedFrameV2Error("RunPARTs required column is absent")
    parsed: list[dict[str, Any]] = []
    cumulative = 0
    for ordinal, source in enumerate(rows[1:]):
        if len(source) != len(header):
            raise FinalizedFrameV2Error("RunPARTs row width differs")
        row = dict(zip(header, source, strict=True))
        try:
            part = int(row["Part"].replace(",", "")); step_count = int(row["Steps"].replace(",", ""))
            time_s = float(row["TimeStep [s]"])
            counts = {name: int(row[name].replace(",", "")) for name in required[3:]}
        except ValueError as exc:
            raise FinalizedFrameV2Error("RunPARTs numeric token is invalid") from exc
        if part < 0 or step_count < 0 or not math.isfinite(time_s) or time_s < 0 \
                or parsed and time_s <= parsed[-1]["time_s"]:
            raise FinalizedFrameV2Error("RunPARTs time is not strict finite")
        expected = {"NpSave": 9078, "NpSim": 9078, "NpOut": 0, "NpbSim": 2669,
                    "NpfSim": 6409, "NpNormal": 9078, "NpOutPos": 0,
                    "NpOutRho": 0, "NpOutMov": 0}
        if counts != expected:
            raise FinalizedFrameV2Error("RunPARTs particle/Nout contract differs")
        if ordinal == 0:
            if step_count != 0:
                raise FinalizedFrameV2Error("RunPARTs restart/initial row Steps is not zero")
            cumulative = 0
        else:
            if step_count < 1:
                raise FinalizedFrameV2Error("RunPARTs post-start Steps is not positive")
            cumulative += step_count - (1 if ordinal == 1 else 0)
        parsed.append({"part": part, "time_s": time_s, "step": cumulative,
                       "steps_since_previous_part": step_count})
    if [row["part"] for row in parsed] != list(range(parsed[0]["part"], parsed[0]["part"] + len(parsed))):
        raise FinalizedFrameV2Error("RunPARTs part sequence differs")
    return parsed


def parse_frame(raw: bytes, *, index: int, runpart: Mapping[str, Any]) -> tuple[dict[str, Any], dict[int, tuple[float, float, float]]]:
    try:
        root = bi4.parse_jpartdata_bi4(raw)
        if len(root.items) != 1:
            raise bi4.Bi4FormatError("particle item cardinality")
        part = root.items[0]
        arrays = part.arrays
        for name, code in {"Idp": 8, "Posd": 23, "Vel": 22, "Rhop": 11}.items():
            if name not in arrays or arrays[name].type_code != code:
                raise bi4.Bi4FormatError(f"array differs: {name}")
        ids = arrays["Idp"].records(); positions = arrays["Posd"].records()
        velocities = arrays["Vel"].records(); densities = arrays["Rhop"].records()
        cpart = bi4.require_int(part.values, "Cpart")
        step = bi4.require_int(part.values, "Step")
        nout = bi4.require_int(part.values, "Nout")
        npok = bi4.require_int(part.values, "Npok")
        time_s = float(part.values["TimeStep"])
        class_counts = {"fixed_boundary": bi4.require_int(root.values, "CaseNfixed"),
                        "moving_boundary": bi4.require_int(root.values, "CaseNmoving"),
                        "floating": bi4.require_int(root.values, "CaseNfloat"),
                        "fluid": bi4.require_int(root.values, "CaseNfluid")}
    except (bi4.Bi4FormatError, KeyError, TypeError, ValueError) as exc:
        raise FinalizedFrameV2Error(f"invalid Part_{index:04d}: {exc}") from exc
    if part.name != f"PART_{index:04d}" or cpart != index or runpart["part"] != index:
        raise FinalizedFrameV2Error("Part internal/RunPARTs index differs")
    if not math.isfinite(time_s) or abs(time_s - float(runpart["time_s"])) > TIME_TOLERANCE_S:
        raise FinalizedFrameV2Error("Part time differs from RunPARTs")
    if step != runpart["step"] or nout != 0 or npok != EXPECTED_PARTICLES:
        raise FinalizedFrameV2Error("Part step/Nout/count differs from RunPARTs")
    if class_counts != EXPECTED_CLASSES or not all(len(values) == EXPECTED_PARTICLES for values in (ids, positions, velocities, densities)):
        raise FinalizedFrameV2Error("Part class/array count differs")
    if len(set(ids)) != EXPECTED_PARTICLES or set(ids) != set(range(EXPECTED_PARTICLES)):
        raise FinalizedFrameV2Error("Part ID set is not complete unique 0..9077")
    source_order_sha = ids_sha256(ids)
    order = sorted(range(EXPECTED_PARTICLES), key=ids.__getitem__)
    ordered_ids = [ids[position] for position in order]
    ordered_positions = [positions[position] for position in order]
    ordered_velocities = [velocities[position] for position in order]
    ordered_densities = [densities[position] for position in order]
    if ids_sha256(ordered_ids) != CANONICAL_IDS_SHA256:
        raise FinalizedFrameV2Error("canonical ID digest differs")
    values = [value for rows in (ordered_positions, ordered_velocities) for row in rows for value in row]
    if any(not math.isfinite(float(value)) for value in values + ordered_densities):
        raise FinalizedFrameV2Error("Part contains non-finite particle values")
    digest = hashlib.sha256()
    for particle_id, position, velocity, density in zip(ordered_ids, ordered_positions, ordered_velocities, ordered_densities):
        digest.update(struct.pack(">Q7d", particle_id, *map(float, position), *map(float, velocity), float(density)))
    return {
        "index": index, "time_s": time_s, "step": step,
        "runparts_steps_since_previous": runpart["steps_since_previous_part"],
        "particle_count": EXPECTED_PARTICLES,
        "nout": 0, "canonical_ids_sha256": CANONICAL_IDS_SHA256,
        "source_order_sha256": source_order_sha,
        "source_order_was_canonical": ids == list(range(EXPECTED_PARTICLES)),
        "canonical_particle_arrays_sha256": digest.hexdigest(), "finite": True,
        "class_counts": EXPECTED_CLASSES,
    }, {particle_id: ordered_positions[particle_id] for particle_id in ordered_ids}


def read_finalized(output_root: Path, *, external_inventory_receipt: Mapping[str, Any],
                   external_inventory_receipt_sha256: str) -> tuple[dict[str, Any],
                                                                    list[dict[int, tuple[float, float, float]]],
                                                                    dict[str, bytes]]:
    root = Path(output_root); _assert_root(root)
    _sha(external_inventory_receipt_sha256, "external inventory receipt")
    if canonical_sha256(external_inventory_receipt) != external_inventory_receipt_sha256:
        raise FinalizedFrameV2Error("external inventory receipt byte/hash binding differs")
    if set(external_inventory_receipt) != {"schema_version", "root", "files", "inventory_sha256", "finalized"}:
        raise FinalizedFrameV2Error("external inventory receipt is open")
    if external_inventory_receipt["schema_version"] != "smpcc-r8-liquid-s5b0-output-inventory-receipt-v9" or external_inventory_receipt["root"] != str(root) or external_inventory_receipt["finalized"] is not True:
        raise FinalizedFrameV2Error("external inventory receipt root/status differs")
    expected = external_inventory_receipt["files"]
    if not isinstance(expected, list) or not expected:
        raise FinalizedFrameV2Error("external inventory file list is empty")
    expected_map: dict[str, str] = {}
    for row in expected:
        if set(row) != {"relative_path", "sha256", "size_bytes", "mode"}:
            raise FinalizedFrameV2Error("external inventory row is open")
        expected_map[str(row["relative_path"])] = _sha(row["sha256"], str(row["relative_path"]))
    if len(expected_map) != len(expected) or canonical_sha256(expected) != external_inventory_receipt["inventory_sha256"]:
        raise FinalizedFrameV2Error("external inventory duplicates/hash differs")
    for required in (*RUN_FILES, *STATIC_BINDINGS):
        if required not in expected_map:
            raise FinalizedFrameV2Error(f"required output is absent: {required}")
    part_paths = sorted(path for path in expected_map if PART_RE.fullmatch(path))
    if not part_paths:
        raise FinalizedFrameV2Error("no Part frames in external inventory")
    files_before, directories_before = _scan(root)
    if set(files_before) != set(expected_map):
        raise FinalizedFrameV2Error("actual output inventory has missing/extra files")
    payloads: dict[str, bytes] = {}; identities: dict[str, dict[str, Any]] = {}
    for relative in sorted(expected_map):
        raw, identity = _read(root, relative, files_before[relative], MAX_FILE_BYTES)
        source_row = next(row for row in expected if row["relative_path"] == relative)
        if identity["sha256"] != expected_map[relative] or identity["size_bytes"] != source_row["size_bytes"] or identity["mode"] != source_row["mode"]:
            raise FinalizedFrameV2Error(f"actual output identity differs: {relative}")
        payloads[relative] = raw; identities[relative] = identity
    runparts = parse_runparts(payloads["RunPARTs.csv"])
    indices = [int(PART_RE.fullmatch(path).group(1)) for path in part_paths]
    if indices != [row["part"] for row in runparts] or len(indices) != len(runparts):
        raise FinalizedFrameV2Error("Part files and RunPARTs are not one-to-one")
    frames = []; positions_by_frame: list[dict[int, tuple[float, float, float]]] = []
    for relative, row in zip(part_paths, runparts, strict=True):
        frame, positions = parse_frame(payloads[relative], index=row["part"], runpart=row)
        frame["identity"] = identities[relative]; frames.append(frame); positions_by_frame.append(positions)
    files_after, directories_after = _scan(root)
    if files_before != files_after or directories_before != directories_after:
        raise FinalizedFrameV2Error("output tree TOCTOU")
    records = [identities[path] for path in sorted(identities)]
    manifest = {
        "schema_version": "smpcc-r8-liquid-s5b0-finalized-solver-frames-manifest-v2",
        "document_type": "SMPCC_R8_LIQUID_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V2",
        "status": "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V2", "root": str(root),
        "contract": {"expected_root": str(root),
            "external_inventory_receipt_sha256": external_inventory_receipt_sha256,
            "expected_inventory_sha256": external_inventory_receipt["inventory_sha256"],
            "expected_start_index": indices[0], "expected_frame_count": len(frames),
            "expected_times_sha256": canonical_sha256([row["time_s"] for row in runparts]),
            "expected_particle_count": EXPECTED_PARTICLES,
            "expected_canonical_ids_sha256": CANONICAL_IDS_SHA256,
            "expected_class_counts": EXPECTED_CLASSES, "time_tolerance_s": TIME_TOLERANCE_S,
            "runparts_step_semantics":
                "RESTART_FIRST_ROW_ZERO_FIRST_INTERVAL_MINUS_ONE_THEN_ACCUMULATE"},
        "inventory": {"file_count": len(records), "directory_count": len(directories_before),
            "canonical_sha256": canonical_sha256(records), "files": records},
        "bindings": {"run_files": [identities[path] for path in RUN_FILES],
            "part_head": identities[STATIC_BINDINGS[0]], "part_info": identities[STATIC_BINDINGS[1]],
            "part_motion_ref": identities[STATIC_BINDINGS[2]], "part_out": identities[STATIC_BINDINGS[3]]},
        "frames": frames,
        "integrity": {"external_inventory_receipt_bound": True, "exact_root": True,
            "exact_inventory": True, "no_symlinks": True, "no_hardlinks": True,
            "no_special_files": True, "no_toctou": True, "frame_indices_contiguous": True,
            "frame_times_match_runparts": True, "runparts_restart_step_contract": True,
            "particle_count_stable": True,
            "particle_id_set_canonical": True, "particle_array_order_normalized": True,
            "particle_classes_stable": True, "nout_zero": True, "finite": True,
            "frame_manifest_sha256": canonical_sha256(frames)},
        "claims": {"reader_is_post_execution": True, "candidate_executed_by_reader": False,
            "solver_executed_by_reader": False, "gpu_exposed": False, "network_used": False,
            "sudo_used": False, "apparmor_loaded": False, "real_bag_read": False,
            "optional_bag_read": False},
    }
    schema = json.loads(SCHEMA_PATH.read_bytes()); Draft202012Validator.check_schema(schema)
    v1.assert_deep_closed(schema); Draft202012Validator(schema).validate(manifest)
    return manifest, positions_by_frame, payloads


def build_manifest(output_root: Path, *, external_inventory_receipt: Mapping[str, Any],
                   external_inventory_receipt_sha256: str) -> dict[str, Any]:
    manifest, _positions, _payloads = read_finalized(
        output_root, external_inventory_receipt=external_inventory_receipt,
        external_inventory_receipt_sha256=external_inventory_receipt_sha256)
    return manifest


def self_check() -> dict[str, Any]:
    schema = json.loads(SCHEMA_PATH.read_bytes()); Draft202012Validator.check_schema(schema)
    v1.assert_deep_closed(schema)
    ids = [2, 0, 3, 1] + list(range(4, EXPECTED_PARTICLES))
    if ids_sha256(sorted(ids)) != CANONICAL_IDS_SHA256 or ids_sha256(ids) == CANONICAL_IDS_SHA256:
        raise FinalizedFrameV2Error("ID normalization fixture drift")
    return {"status": "PASS_S5B0_FINALIZED_FRAME_READER_V2_STATIC_ONLY",
            "canonical_ids_sha256": CANONICAL_IDS_SHA256, "particle_count": EXPECTED_PARTICLES,
            "gpu_id_reordering_accepted_by_canonical_map": True,
            "external_inventory_receipt_required": True, "real_solver_output_read": False,
            "files_written": False, "candidate_executed": False, "gpu_exposed": False,
            "optional_bag_read": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",), nargs="?", default="self-check")
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), sort_keys=True, separators=(",", ":"))); return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_FINALIZED_FRAME_READER_V2", "error": str(exc)}, sort_keys=True), file=sys.stderr); return 2


if __name__ == "__main__":
    raise SystemExit(main())
