#!/usr/bin/env python3
"""Fixture-only adapter from finalized BI4 evidence to surface proxy v3.

The adapter accepts an already validated finalized-frame manifest and particle
arrays already returned by the safe BI4 reader.  All inputs are in-memory; no
output path, BI4 path, bag, solver, GPU, sudo, or network surface is exposed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5b0_container_surface_proxy_v3 as proxy_v3  # noqa: E402


SCHEMA_PATH = ROOT / "schema/target_host_s5b0_container_surface_proxy_from_bi4_v1.json"
MANIFEST_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_finalized_solver_frames_manifest_v1.json"
FINALIZED_READER_PATH = MODULE_DIR / "r8_liquid_s5b0_finalized_frame_reader_v1.py"
BI4_READER_PATH = MODULE_DIR / "r8_liquid_bi4_reader_v1.py"
PROXY_V3_PATH = MODULE_DIR / "r8_liquid_s5b0_container_surface_proxy_v3.py"
FINALIZED_READER_SHA256 = "239bd4a29e5735e194db6e18da2ad658626119687d0cfef7cbb8bdbe3bfe7932"
BI4_READER_SHA256 = "104d271b750eed15985bf29f743a59678d66c50b9dfae9d1a3fd1663510dbc14"
PROXY_V3_SHA256 = "01e43d7bc7641c0e19e1709bb0bb75fc10130f89d0283199f18be8df0448fe44"
H0_SOURCE_METHOD = "FROZEN_SETTLED_REFERENCE_H0_V1"
TIME_TOLERANCE_S = 1e-9


class SurfaceProxyFromBi4Error(ValueError):
    """Manifest, particle binding, reference, proxy, or result failure."""


def canonical_json(value: Any) -> bytes:
    try:
        encoded = json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":"))
    except (TypeError, ValueError) as exc:
        raise SurfaceProxyFromBi4Error("value is not finite canonical JSON") from exc
    return (encoded + "\n").encode("utf-8")


def sha256_json(value: Any) -> str:
    return hashlib.sha256(canonical_json(value)).hexdigest()


def _reader_sha256(value: Any) -> str:
    raw = json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                     separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def ids_sha256(ids: Sequence[int]) -> str:
    digest = hashlib.sha256()
    for value in ids:
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**64:
            raise SurfaceProxyFromBi4Error("particle ID is not canonical uint64")
        digest.update(struct.pack("<Q", value))
    return digest.hexdigest()


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise SurfaceProxyFromBi4Error(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def _load_schema(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SurfaceProxyFromBi4Error(f"schema root is not an object: {path.name}")
    Draft202012Validator.check_schema(value)
    assert_deep_closed(value)
    return value


def _assert_dependencies() -> None:
    for path, expected in (
        (FINALIZED_READER_PATH, FINALIZED_READER_SHA256),
        (BI4_READER_PATH, BI4_READER_SHA256),
        (PROXY_V3_PATH, PROXY_V3_SHA256),
    ):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise SurfaceProxyFromBi4Error(f"frozen dependency hash drifted: {path.name}")


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SurfaceProxyFromBi4Error(f"{label} is not an object")
    return value


def _exact(value: Any, keys: set[str], label: str) -> Mapping[str, Any]:
    result = _mapping(value, label)
    if set(result) != keys:
        raise SurfaceProxyFromBi4Error(f"{label} fields differ")
    return result


def _integer(value: Any, label: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise SurfaceProxyFromBi4Error(f"{label} is not an integer >= {minimum}")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise SurfaceProxyFromBi4Error(f"{label} is non-finite or non-numeric")
    return float(value)


def _sha(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)):
        raise SurfaceProxyFromBi4Error(f"{label} is not a lowercase SHA-256")
    return value


def _validate_manifest(manifest: Any, expected_sha256: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    value = dict(_mapping(manifest, "manifest"))
    try:
        Draft202012Validator(_load_schema(MANIFEST_SCHEMA_PATH)).validate(value)
    except ValidationError as exc:
        raise SurfaceProxyFromBi4Error(f"finalized manifest schema failure: {exc.message}") from exc
    expected = _sha(expected_sha256, "expected manifest")
    if sha256_json(value) != expected:
        raise SurfaceProxyFromBi4Error("finalized manifest hash drift")

    frames = value["frames"]
    contract = value["contract"]
    if _reader_sha256(frames) != value["integrity"]["frame_manifest_sha256"]:
        raise SurfaceProxyFromBi4Error("manifest frame hash is self-contradictory")
    if len(frames) != contract["expected_frame_count"] or len(frames) < 2:
        raise SurfaceProxyFromBi4Error("manifest frame count differs or is insufficient")
    expected_indices = list(range(contract["expected_start_index"],
                                  contract["expected_start_index"] + len(frames)))
    if [row["index"] for row in frames] != expected_indices:
        raise SurfaceProxyFromBi4Error("manifest frame index sequence drift")
    times = [_finite(row["time_s"], "manifest frame time") for row in frames]
    if any(right <= left for left, right in zip(times, times[1:])):
        raise SurfaceProxyFromBi4Error("manifest frame time sequence drift")
    step = times[1] - times[0]
    if any(abs((times[index] - times[0]) - index * step) > TIME_TOLERANCE_S
           for index in range(len(times))):
        raise SurfaceProxyFromBi4Error("manifest frame time grid is not uniform")
    if _reader_sha256(times) != contract["expected_times_sha256"]:
        raise SurfaceProxyFromBi4Error("manifest expected time hash drift")

    files = value["inventory"]["files"]
    if files != sorted(files, key=lambda row: row["relative_path"]):
        raise SurfaceProxyFromBi4Error("manifest inventory order drift")
    paths = [row["relative_path"] for row in files]
    if len(paths) != len(set(paths)):
        raise SurfaceProxyFromBi4Error("manifest inventory contains duplicate paths")
    if _reader_sha256(files) != value["inventory"]["canonical_sha256"]:
        raise SurfaceProxyFromBi4Error("manifest inventory hash is self-contradictory")
    digest_rows = [{"relative_path": row["relative_path"], "sha256": row["sha256"]}
                   for row in files]
    if _reader_sha256(digest_rows) != contract["expected_inventory_sha256"]:
        raise SurfaceProxyFromBi4Error("manifest expected inventory hash drift")
    inventory = {row["relative_path"]: row for row in files}

    first_counts = frames[0]["class_counts"]
    for frame in frames:
        identity = frame["identity"]
        path = f'data/Part_{frame["index"]:04d}.bi4'
        if identity["relative_path"] != path or inventory.get(path) != identity:
            raise SurfaceProxyFromBi4Error("manifest frame identity/inventory binding drift")
        if frame["ids_sha256"] != contract["expected_ids_sha256"]:
            raise SurfaceProxyFromBi4Error("manifest particle ID hash drift")
        if frame["particle_count"] != contract["expected_particle_count"]:
            raise SurfaceProxyFromBi4Error("manifest particle count drift")
        if frame["class_counts"] != first_counts:
            raise SurfaceProxyFromBi4Error("manifest particle class drift")
    return value, [dict(row) for row in frames]


def _positions(rows: Any, count: int, label: str) -> list[list[float]]:
    if not isinstance(rows, list) or len(rows) != count:
        raise SurfaceProxyFromBi4Error(f"{label} count drift")
    result: list[list[float]] = []
    for index, row in enumerate(rows):
        if not isinstance(row, (list, tuple)) or len(row) != 3:
            raise SurfaceProxyFromBi4Error(f"{label}[{index}] is not a three-vector")
        result.append([_finite(value, f"{label}[{index}]") for value in row])
    return result


def _validate_particle_payload(raw: Any, manifest_frame: Mapping[str, Any]) -> dict[str, Any]:
    frame = _exact(raw, {"index", "time_s", "identity", "particles"}, "particle frame")
    if frame["index"] != manifest_frame["index"]:
        raise SurfaceProxyFromBi4Error("particle frame index drift")
    if abs(_finite(frame["time_s"], "particle frame time") - manifest_frame["time_s"]) > TIME_TOLERANCE_S:
        raise SurfaceProxyFromBi4Error("particle frame time drift")
    if frame["identity"] != manifest_frame["identity"]:
        raise SurfaceProxyFromBi4Error("particle frame SHA/inode identity drift")
    particles = _exact(frame["particles"], {
        "part_name", "particle_count", "counts", "limits_m", "ids", "classes",
        "positions_m", "velocities_m_s", "densities_kg_m3", "array_metadata",
    }, "safe BI4 particles")
    count = _integer(particles["particle_count"], "particle_count", 1)
    if count != manifest_frame["particle_count"]:
        raise SurfaceProxyFromBi4Error("safe BI4 particle count drift")
    if particles["part_name"] != f'PART_{manifest_frame["index"]:04d}':
        raise SurfaceProxyFromBi4Error("safe BI4 internal frame index drift")
    if particles["counts"] != manifest_frame["class_counts"]:
        raise SurfaceProxyFromBi4Error("safe BI4 particle class drift")
    counts = particles["counts"]
    if set(counts) != {"fixed_boundary", "moving_boundary", "floating", "fluid"}:
        raise SurfaceProxyFromBi4Error("safe BI4 class fields drift")
    if sum(_integer(value, "class count") for value in counts.values()) != count:
        raise SurfaceProxyFromBi4Error("safe BI4 class count sum drift")
    if counts["moving_boundary"] < 4 or counts["fluid"] < 1:
        raise SurfaceProxyFromBi4Error("safe BI4 classes cannot support surface proxy")
    ids = particles["ids"]
    if not isinstance(ids, list) or ids != list(range(count)):
        raise SurfaceProxyFromBi4Error("safe BI4 particle ID sequence drift")
    if ids_sha256(ids) != manifest_frame["ids_sha256"]:
        raise SurfaceProxyFromBi4Error("safe BI4 particle ID hash drift")
    expected_classes = (
        ["fixed_boundary"] * counts["fixed_boundary"]
        + ["moving_boundary"] * counts["moving_boundary"]
        + ["floating"] * counts["floating"]
        + ["fluid"] * counts["fluid"]
    )
    if particles["classes"] != expected_classes:
        raise SurfaceProxyFromBi4Error("safe BI4 particle class assignment drift")
    positions = _positions(particles["positions_m"], count, "positions_m")
    _positions(particles["velocities_m_s"], count, "velocities_m_s")
    densities = particles["densities_kg_m3"]
    if not isinstance(densities, list) or len(densities) != count:
        raise SurfaceProxyFromBi4Error("density count drift")
    for value in densities:
        _finite(value, "density")
    limits = particles["limits_m"]
    if set(_mapping(limits, "limits_m")) != {"x", "y", "z"}:
        raise SurfaceProxyFromBi4Error("safe BI4 limits fields drift")
    for axis, column in zip(("x", "y", "z"), zip(*positions)):
        if list(limits[axis]) != [min(column), max(column)]:
            raise SurfaceProxyFromBi4Error("safe BI4 position limits drift")
    metadata = _mapping(particles["array_metadata"], "array_metadata")
    if set(metadata) != {"Idp", "Posd", "Vel", "Rhop"}:
        raise SurfaceProxyFromBi4Error("safe BI4 array metadata drift")
    if any(_mapping(row, "array metadata").get("count") != count for row in metadata.values()):
        raise SurfaceProxyFromBi4Error("safe BI4 array metadata count drift")
    return {"ids": ids, "classes": expected_classes, "positions": positions,
            "counts": dict(counts)}


def _validate_reference(value: Any, expected_sha256: str,
                        manifest: Mapping[str, Any], moving_ids: Sequence[int]) -> list[dict[str, Any]]:
    reference = _exact(value, {"source_identity", "particles"}, "boundary reference")
    if reference["source_identity"] != manifest["bindings"]["part_motion_ref"]:
        raise SurfaceProxyFromBi4Error("boundary reference source identity drift")
    if sha256_json(reference) != _sha(expected_sha256, "expected boundary reference"):
        raise SurfaceProxyFromBi4Error("boundary reference hash drift")
    rows = reference["particles"]
    if not isinstance(rows, list) or len(rows) != len(moving_ids):
        raise SurfaceProxyFromBi4Error("boundary reference particle count drift")
    output: list[dict[str, Any]] = []
    for index, (row, expected_id) in enumerate(zip(rows, moving_ids)):
        item = _exact(row, {"id", "xyz_m"}, f"boundary reference[{index}]")
        if item["id"] != expected_id:
            raise SurfaceProxyFromBi4Error("boundary reference ID drift")
        xyz = _positions([item["xyz_m"]], 1, "boundary reference xyz")[0]
        output.append({"id": expected_id, "xyz_m": xyz})
    return output


def validate_result(result: Any) -> dict[str, Any]:
    value = dict(_mapping(result, "result"))
    try:
        Draft202012Validator(_load_schema(SCHEMA_PATH)).validate(value)
    except ValidationError as exc:
        raise SurfaceProxyFromBi4Error(f"result schema failure: {exc.message}") from exc
    source = value["source_contract"]
    if (source["finalized_reader_sha256"] != FINALIZED_READER_SHA256
            or source["safe_bi4_reader_sha256"] != BI4_READER_SHA256
            or source["surface_proxy_v3_sha256"] != PROXY_V3_SHA256):
        raise SurfaceProxyFromBi4Error("result frozen dependency binding drift")
    h0 = value["baseline"]["h0_m"]
    invalid = 0
    for frame in value["frames"]:
        if [row["sector_id"] for row in frame["sectors"]] != list(proxy_v3.SECTOR_IDS):
            raise SurfaceProxyFromBi4Error("result sector sequence is missing or reordered")
        for sector in frame["sectors"]:
            if sector["valid"]:
                if abs(sector["h_proxy_relative_h0_m"] - (sector["q98_z_m"] - h0)) > 1e-12:
                    raise SurfaceProxyFromBi4Error("result h0-relative surface is self-contradictory")
            else:
                invalid += 1
        valid_count = sum(bool(row["valid"]) for row in frame["sectors"])
        if frame["valid_sector_count"] != valid_count or frame["frame_valid"] is not (valid_count == 16):
            raise SurfaceProxyFromBi4Error("result frame validity is self-contradictory")
        if frame["selected_fluid_particle_count"] != sum(row["count"] for row in frame["sectors"]):
            raise SurfaceProxyFromBi4Error("result selected particle count is self-contradictory")
    qc = value["qc"]
    all_valid = invalid == 0
    if (qc["expected_frame_count"] != len(value["frames"])
            or qc["observed_frame_count"] != len(value["frames"])
            or qc["invalid_sector_count"] != invalid
            or qc["all_sectors_valid"] is not all_valid
            or qc["algorithm_valid"] is not all_valid):
        raise SurfaceProxyFromBi4Error("result QC is self-contradictory")
    expected_status = ("FIXTURE_ONLY_BI4_SURFACE_PROXY_VALID_NOT_ADMITTED" if all_valid
                       else "FIXTURE_ONLY_BI4_SURFACE_PROXY_INVALID_SECTORS_NOT_ADMITTED")
    if value["status"] != expected_status:
        raise SurfaceProxyFromBi4Error("result status is self-contradictory")
    if value["integrity"]["output_frames_sha256"] != sha256_json(value["frames"]):
        raise SurfaceProxyFromBi4Error("result output frame hash is self-contradictory")
    canonical_json(value)
    return value


def adapt_manifest_particles(
    manifest: Any,
    particle_frames: Any,
    settled_boundary_reference: Any,
    *,
    expected_manifest_sha256: str,
    expected_boundary_reference_sha256: str,
    case_id: str,
    container_radius_m: float,
    h0_m: float,
    h0_source_method: str,
    fixture_only: bool,
) -> dict[str, Any]:
    """Derive an adapter result from in-memory, already-read fixture evidence."""

    if fixture_only is not True:
        raise SurfaceProxyFromBi4Error("REAL_EXTERNAL_OUTPUT_NOT_ADMITTED_FIXTURE_ONLY")
    if h0_source_method != H0_SOURCE_METHOD:
        raise SurfaceProxyFromBi4Error("h0 source method drift")
    radius = _finite(container_radius_m, "container_radius_m")
    h0 = _finite(h0_m, "h0_m")
    if radius <= 0.0 or h0 < 0.0:
        raise SurfaceProxyFromBi4Error("container radius or h0 is outside the admitted range")
    _assert_dependencies()
    checked_manifest, manifest_frames = _validate_manifest(manifest, expected_manifest_sha256)
    if not isinstance(particle_frames, list) or len(particle_frames) != len(manifest_frames):
        raise SurfaceProxyFromBi4Error("particle frame count drift")
    checked_particles = [
        _validate_particle_payload(raw, manifest_frame)
        for raw, manifest_frame in zip(particle_frames, manifest_frames)
    ]
    first = checked_particles[0]
    if any(row["ids"] != first["ids"] for row in checked_particles[1:]):
        raise SurfaceProxyFromBi4Error("particle IDs drift across frames")
    if any(row["classes"] != first["classes"] for row in checked_particles[1:]):
        raise SurfaceProxyFromBi4Error("particle classes drift across frames")
    moving_ids = [particle_id for particle_id, label in zip(first["ids"], first["classes"])
                  if label == "moving_boundary"]
    reference = _validate_reference(settled_boundary_reference,
                                    expected_boundary_reference_sha256,
                                    checked_manifest, moving_ids)
    times = [row["time_s"] for row in manifest_frames]
    proxy_fixture = {
        "schema_version": proxy_v3.INPUT_SCHEMA_VERSION,
        "fixture_only": True,
        "case_id": case_id,
        "container_radius_m": radius,
        "frame_contract": {
            "start_index": manifest_frames[0]["index"],
            "end_index": manifest_frames[-1]["index"],
            "index_step": 1,
            "start_time_s": times[0],
            "time_step_s": times[1] - times[0],
        },
        "settled_moving_boundary": reference,
        "frames": [],
    }
    for manifest_frame, particles in zip(manifest_frames, checked_particles):
        moving, fluid = [], []
        for particle_id, label, xyz in zip(particles["ids"], particles["classes"], particles["positions"]):
            row = {"id": particle_id, "xyz_m": xyz}
            if label == "moving_boundary":
                moving.append(row)
            elif label == "fluid":
                fluid.append(row)
        proxy_fixture["frames"].append({
            "frame_index": manifest_frame["index"], "time_s": manifest_frame["time_s"],
            "moving_boundary": moving, "fluid": fluid,
        })
    try:
        proxy_result = proxy_v3.analyze_fixture(proxy_fixture)
    except proxy_v3.SurfaceProxyV3Error as exc:
        raise SurfaceProxyFromBi4Error(f"surface proxy v3 rejected adapter fixture: {exc}") from exc

    output_frames: list[dict[str, Any]] = []
    for manifest_frame, proxy_frame in zip(manifest_frames, proxy_result["frames"]):
        sectors = [{
            "sector_id": row["sector_id"], "count": row["count"], "valid": row["valid"],
            "q98_z_m": row["q98_z_m"],
            "h_proxy_relative_h0_m": None if row["q98_z_m"] is None else row["q98_z_m"] - h0,
            "null_reason": row["null_reason"],
        } for row in proxy_frame["sectors"]]
        output_frames.append({
            "frame_index": manifest_frame["index"], "time_s": manifest_frame["time_s"],
            "source_identity": dict(manifest_frame["identity"]),
            "particle_ids_sha256": manifest_frame["ids_sha256"],
            "class_counts": dict(manifest_frame["class_counts"]),
            "transform": proxy_frame["transform"],
            "selected_fluid_particle_count": proxy_frame["selected_fluid_particle_count"],
            "excluded_radial_particle_count": proxy_frame["excluded_radial_particle_count"],
            "sectors": sectors, "valid_sector_count": proxy_frame["valid_sector_count"],
            "frame_valid": proxy_frame["frame_valid"],
        })
    all_valid = proxy_result["qc"]["algorithm_valid"]
    result = {
        "schema_version": "smpcc-r8-liquid-s5b0-container-surface-proxy-from-bi4-v1",
        "document_type": "SMPCC_R8_LIQUID_S5B0_CONTAINER_SURFACE_PROXY_FROM_BI4_V1",
        "status": ("FIXTURE_ONLY_BI4_SURFACE_PROXY_VALID_NOT_ADMITTED" if all_valid
                   else "FIXTURE_ONLY_BI4_SURFACE_PROXY_INVALID_SECTORS_NOT_ADMITTED"),
        "fixture_only": True, "case_id": case_id,
        "source_contract": {
            "finalized_manifest_sha256": sha256_json(checked_manifest),
            "finalized_frame_manifest_sha256": checked_manifest["integrity"]["frame_manifest_sha256"],
            "finalized_reader_sha256": FINALIZED_READER_SHA256,
            "safe_bi4_reader_sha256": BI4_READER_SHA256,
            "surface_proxy_v3_sha256": PROXY_V3_SHA256,
            "input_surface": "IN_MEMORY_VALIDATED_MANIFEST_AND_SAFE_BI4_PARTICLES_ONLY",
        },
        "baseline": {
            "h0_m": h0, "h0_source_method": H0_SOURCE_METHOD,
            "settled_boundary_reference_sha256": sha256_json(settled_boundary_reference),
            "source_identity": dict(checked_manifest["bindings"]["part_motion_ref"]),
        },
        "surface_contract": {
            "surface_source": "OFFLINE_FINALIZED_BI4_CONTAINER_FRAME_PARTICLE_Q98_PROXY",
            "transform_method": proxy_v3.TRANSFORM_METHOD,
            "surface_statistic": "CONTAINER_FRAME_FLUID_PARTICLE_Z_Q98",
            "sector_ids": list(proxy_v3.SECTOR_IDS), "sector_count": 16, "quantile": 0.98,
            "explicit_non_gauge": True, "is_solver_gauge": False,
            "is_jgaugeswl": False, "is_point_probe": False,
        },
        "frames": output_frames,
        "qc": {
            "expected_frame_count": len(manifest_frames), "observed_frame_count": len(output_frames),
            "frame_bindings_exact": True, "time_sequence_complete": True,
            "particle_ids_stable": True, "particle_classes_stable": True,
            "boundary_reference_exact": True, "all_sector_rows_complete": True,
            "all_sectors_valid": all_valid,
            "invalid_sector_count": proxy_result["qc"]["invalid_sector_count"],
            "algorithm_valid": all_valid,
        },
        "integrity": {
            "particle_frames_sha256": sha256_json(particle_frames),
            "surface_proxy_v3_result_sha256": proxy_v3.sha256_json(proxy_result),
            "output_frames_sha256": sha256_json(output_frames),
        },
        "claims": {
            "real_external_output_read_by_adapter": False, "real_bi4_read_by_adapter": False,
            "real_bag_read": False, "solver_executed": False, "gpu_exposed": False,
            "network_used": False, "sudo_used": False, "raw_solver_surface_measured": False,
            "stage5b0_pass": False,
        },
    }
    return validate_result(result)


def verify_result(manifest: Any, particle_frames: Any, settled_boundary_reference: Any,
                  claimed_result: Any, **kwargs: Any) -> dict[str, Any]:
    claimed = validate_result(claimed_result)
    expected = adapt_manifest_particles(manifest, particle_frames, settled_boundary_reference,
                                        **kwargs)
    if canonical_json(claimed) != canonical_json(expected):
        raise SurfaceProxyFromBi4Error("claimed result differs from fresh BI4 particle derivation")
    return expected


def _identity(path: str, inode: int) -> dict[str, Any]:
    return {"relative_path": path, "type": "regular", "mode": "0600", "size_bytes": 128,
            "device": 7, "inode": inode, "nlink": 1,
            "sha256": hashlib.sha256(path.encode("utf-8")).hexdigest()}


def synthetic_inputs() -> dict[str, Any]:
    fixture = proxy_v3.synthetic_fixture()
    boundary_count = len(fixture["settled_moving_boundary"])
    fluid_count = len(fixture["frames"][0]["fluid"])
    count = boundary_count + fluid_count
    remap: dict[int, int] = {}
    for new_id, row in enumerate(fixture["settled_moving_boundary"]):
        remap[row["id"]] = new_id
    for offset, row in enumerate(fixture["frames"][0]["fluid"], start=boundary_count):
        remap[row["id"]] = offset
    reference_particles = [
        {"id": remap[row["id"]], "xyz_m": list(row["xyz_m"])}
        for row in fixture["settled_moving_boundary"]
    ]
    static_paths = ["Run.csv", "Run.out", "RunPARTs.csv", "data/Part_Head.ibi4",
                    "data/PartInfo.ibi4", "data/PartMotionRef.ibi4", "data/PartOut_000.obi4"]
    identities = [_identity(path, 100 + index) for index, path in enumerate(static_paths)]
    frame_identities = [_identity(f'data/Part_{frame["frame_index"]:04d}.bi4', 200 + index)
                        for index, frame in enumerate(fixture["frames"])]
    identities.extend(frame_identities)
    identities.sort(key=lambda row: row["relative_path"])
    counts = {"fixed_boundary": 0, "moving_boundary": boundary_count,
              "floating": 0, "fluid": fluid_count}
    ids = list(range(count))
    ids_hash = ids_sha256(ids)
    manifest_frames, particle_frames = [], []
    for frame, identity in zip(fixture["frames"], frame_identities):
        by_id = {
            remap[row["id"]]: list(row["xyz_m"])
            for row in frame["moving_boundary"] + frame["fluid"]
        }
        positions = [by_id[particle_id] for particle_id in ids]
        limits = {axis: [min(row[column] for row in positions), max(row[column] for row in positions)]
                  for axis, column in (("x", 0), ("y", 1), ("z", 2))}
        manifest_frame = {
            "index": frame["frame_index"], "time_s": frame["time_s"], "identity": dict(identity),
            "particle_count": count, "nout": 0, "ids_sha256": ids_hash, "finite": True,
            "class_counts": counts,
        }
        manifest_frames.append(manifest_frame)
        particle_frames.append({
            "index": frame["frame_index"], "time_s": frame["time_s"], "identity": dict(identity),
            "particles": {
                "part_name": f'PART_{frame["frame_index"]:04d}', "particle_count": count,
                "counts": counts, "limits_m": limits, "ids": ids,
                "classes": ["moving_boundary"] * boundary_count + ["fluid"] * fluid_count,
                "positions_m": positions, "velocities_m_s": [[0.0, 0.0, 0.0] for _ in ids],
                "densities_kg_m3": [1000.0 for _ in ids],
                "array_metadata": {
                    "Idp": {"type": "uint32", "count": count, "payload_bytes": 4 * count},
                    "Posd": {"type": "double3", "count": count, "payload_bytes": 24 * count},
                    "Vel": {"type": "float3", "count": count, "payload_bytes": 12 * count},
                    "Rhop": {"type": "float", "count": count, "payload_bytes": 4 * count},
                },
            },
        })
    times = [row["time_s"] for row in manifest_frames]
    digest_rows = [{"relative_path": row["relative_path"], "sha256": row["sha256"]}
                   for row in identities]
    lookup = {row["relative_path"]: row for row in identities}
    manifest = {
        "schema_version": "smpcc-r8-liquid-s5b0-finalized-solver-frames-manifest-v1",
        "document_type": "SMPCC_R8_LIQUID_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1",
        "status": "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1", "root": "/fixture/not-read",
        "contract": {
            "expected_root": "/fixture/not-read", "expected_inventory_sha256": _reader_sha256(digest_rows),
            "expected_start_index": manifest_frames[0]["index"], "expected_frame_count": len(times),
            "expected_times_sha256": _reader_sha256(times), "expected_particle_count": count,
            "expected_ids_sha256": ids_hash, "time_tolerance_s": TIME_TOLERANCE_S,
        },
        "inventory": {"file_count": len(identities), "directory_count": 1,
                      "canonical_sha256": _reader_sha256(identities), "files": identities},
        "bindings": {
            "run_files": [dict(lookup[path]) for path in ("Run.csv", "Run.out", "RunPARTs.csv")],
            "part_head": dict(lookup["data/Part_Head.ibi4"]),
            "part_info": dict(lookup["data/PartInfo.ibi4"]),
            "part_motion_ref": dict(lookup["data/PartMotionRef.ibi4"]),
            "part_out": dict(lookup["data/PartOut_000.obi4"]),
        },
        "frames": manifest_frames,
        "integrity": {
            "exact_root": True, "exact_inventory": True, "no_symlinks": True,
            "no_hardlinks": True, "no_special_files": True, "no_toctou": True,
            "frame_indices_contiguous": True, "frame_times_match_external_grid": True,
            "particle_count_stable": True, "particle_ids_stable": True, "nout_zero": True,
            "finite": True, "frame_manifest_sha256": _reader_sha256(manifest_frames),
        },
        "claims": {
            "candidate_executed_by_reader": False, "solver_executed_by_reader": False,
            "gpu_exposed": False, "network_used": False, "sudo_used": False,
            "apparmor_loaded": False, "real_bag_read": False,
        },
    }
    boundary_reference = {"source_identity": dict(lookup["data/PartMotionRef.ibi4"]),
                          "particles": reference_particles}
    kwargs = {
        "expected_manifest_sha256": sha256_json(manifest),
        "expected_boundary_reference_sha256": sha256_json(boundary_reference),
        "case_id": "fixture_bi4_surface_adapter_v1", "container_radius_m": 1.0,
        "h0_m": 0.2, "h0_source_method": H0_SOURCE_METHOD, "fixture_only": True,
    }
    return {"manifest": manifest, "particle_frames": particle_frames,
            "settled_boundary_reference": boundary_reference, "kwargs": kwargs}


def self_check() -> dict[str, Any]:
    fixture = synthetic_inputs()
    result = adapt_manifest_particles(fixture["manifest"], fixture["particle_frames"],
                                      fixture["settled_boundary_reference"], **fixture["kwargs"])
    verify_result(fixture["manifest"], fixture["particle_frames"],
                  fixture["settled_boundary_reference"], result, **fixture["kwargs"])
    return {
        "status": "S5B0_CONTAINER_SURFACE_PROXY_FROM_BI4_V1_FIXTURE_SELF_CHECK_OK_NOT_ADMITTED",
        "schema_id": _load_schema(SCHEMA_PATH)["$id"], "frame_count": len(result["frames"]),
        "sector_count": 16, "algorithm_valid": result["qc"]["algorithm_valid"],
        "explicit_non_gauge": True, "real_external_output_read": False,
        "real_bi4_read": False, "real_bag_read": False, "solver_or_gpu_executed": False,
        "network_used": False, "sudo_used": False, "stage5b0_pass": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args(argv)
    if not args.self_check:
        parser.error("only --self-check is supported; no external output path is admitted")
    print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["SurfaceProxyFromBi4Error", "adapt_manifest_particles", "validate_result",
           "verify_result", "synthetic_inputs", "self_check", "canonical_json", "sha256_json"]
