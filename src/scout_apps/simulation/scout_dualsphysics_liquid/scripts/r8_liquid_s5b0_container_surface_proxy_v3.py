#!/usr/bin/env python3
"""Fixture-only S5B0 container-frame particle surface proxy v3.

The module accepts in-memory synthetic particle fixtures only.  It does not
accept filesystem paths, read BI4/bag data, execute a solver, or expose a GPU.
For every frame it recovers the complete world-to-container SE(3) transform
from an ID-exact moving-boundary correspondence using Kabsch, then derives a
16-sector q98 surface proxy from fluid particles in the 0.45R--0.95R annulus.

This is deliberately not a moving solver Gauge, JGaugeSwl, or point probe.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
SCHEMA_PATH = ROOT / "schema/target_host_s5b0_container_surface_proxy_v3.json"

INPUT_SCHEMA_VERSION = "smpcc-r8-liquid-s5b0-container-surface-proxy-fixture-v3"
OUTPUT_SCHEMA_VERSION = "smpcc-r8-liquid-s5b0-container-surface-proxy-v3"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_S5B0_CONTAINER_SURFACE_PROXY_V3"
SURFACE_SOURCE = "OFFLINE_CONTAINER_FRAME_PARTICLE_Q98_PROXY"
TRANSFORM_METHOD = "FULL_SE3_KABSCH_MOVING_BOUNDARY_ID_EXACT_V1"
SECTOR_ASSIGNMENT = "ATAN2_HALF_OPEN_EQUAL_AZIMUTH_V1"
SECTOR_IDS = tuple(f"s{index:02d}" for index in range(16))
SECTOR_COUNT = len(SECTOR_IDS)
ANNULUS_INNER_RADIUS_RATIO = 0.45
ANNULUS_OUTER_RADIUS_RATIO = 0.95
MINIMUM_PARTICLES_PER_SECTOR = 128
QUANTILE = 0.98
QUANTILE_METHOD = "SORTED_LINEAR_TYPE7_V1"
MAXIMUM_RMS_RESIDUAL_M = 1e-8
MAXIMUM_POINT_RESIDUAL_M = 5e-8
MINIMUM_REFERENCE_RANK = 3
MINIMUM_SINGULAR_VALUE_RATIO = 1e-6
TIME_TOLERANCE_S = 1e-9
CASE_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class SurfaceProxyV3Error(ValueError):
    """Closed-fixture, correspondence, rigid-transform, or result failure."""


def canonical_json(value: Any) -> bytes:
    """Return the one admitted deterministic JSON representation."""

    try:
        encoded = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except (TypeError, ValueError) as exc:
        raise SurfaceProxyV3Error("value is not finite canonical JSON") from exc
    return (encoded + "\n").encode("utf-8")


def sha256_json(value: Any) -> str:
    return hashlib.sha256(canonical_json(value)).hexdigest()


def _load_schema() -> dict[str, Any]:
    value = json.loads(
        SCHEMA_PATH.read_text(encoding="utf-8"),
        parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
    )
    if not isinstance(value, dict):
        raise SurfaceProxyV3Error("result schema root is not an object")
    Draft202012Validator.check_schema(value)
    assert_deep_closed(value)
    return value


def assert_deep_closed(value: Any, location: str = "$") -> None:
    """Reject every object schema that does not explicitly close properties."""

    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise SurfaceProxyV3Error(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SurfaceProxyV3Error(f"{label} is not an object")
    return value


def _require_exact_keys(value: Any, expected: set[str], label: str) -> Mapping[str, Any]:
    mapping = _require_mapping(value, label)
    actual = set(mapping)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise SurfaceProxyV3Error(
            f"{label} fields differ; missing={missing}, extra={extra}"
        )
    return mapping


def _integer(value: Any, label: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise SurfaceProxyV3Error(f"{label} is not an integer >= {minimum}")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SurfaceProxyV3Error(f"{label} is not numeric")
    result = float(value)
    if not math.isfinite(result):
        raise SurfaceProxyV3Error(f"{label} is non-finite")
    return result


def _vector3(value: Any, label: str) -> np.ndarray:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
        or len(value) != 3
    ):
        raise SurfaceProxyV3Error(f"{label} is not a three-vector")
    vector = np.asarray(
        [_finite(component, f"{label}[{index}]") for index, component in enumerate(value)],
        dtype=np.float64,
    )
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise SurfaceProxyV3Error(f"{label} is non-finite")
    return vector


def _particle_rows(value: Any, label: str) -> dict[int, np.ndarray]:
    if not isinstance(value, list) or not value:
        raise SurfaceProxyV3Error(f"{label} is empty or not an array")
    result: dict[int, np.ndarray] = {}
    for index, raw_row in enumerate(value):
        row = _require_exact_keys(raw_row, {"id", "xyz_m"}, f"{label}[{index}]")
        particle_id = _integer(row["id"], f"{label}[{index}].id")
        if particle_id in result:
            raise SurfaceProxyV3Error(f"{label} contains duplicate particle ID {particle_id}")
        result[particle_id] = _vector3(row["xyz_m"], f"{label}[{index}].xyz_m")
    return result


def _geometry_quality(points: np.ndarray, label: str) -> tuple[int, float]:
    centered = points - points.mean(axis=0)
    singular_values = np.linalg.svd(centered, compute_uv=False)
    largest = float(singular_values[0]) if len(singular_values) else 0.0
    rank = int(np.linalg.matrix_rank(centered))
    ratio = 0.0 if largest <= 0.0 else float(singular_values[-1] / largest)
    if rank < MINIMUM_REFERENCE_RANK or ratio < MINIMUM_SINGULAR_VALUE_RATIO:
        raise SurfaceProxyV3Error(
            f"{label} geometry is degenerate: rank={rank}, singular_ratio={ratio:.3e}"
        )
    return rank, ratio


def _ordered_points(rows: Mapping[int, np.ndarray], ids: Sequence[int]) -> np.ndarray:
    return np.asarray([rows[particle_id] for particle_id in ids], dtype=np.float64)


def _recover_world_to_container(
    reference: np.ndarray,
    current_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Recover current world -> settled reference using an exact 3-D Kabsch fit."""

    reference_rank, reference_ratio = _geometry_quality(reference, "settled boundary")
    _geometry_quality(current_world, "current boundary")
    reference_centroid = reference.mean(axis=0)
    current_centroid = current_world.mean(axis=0)
    reference_centered = reference - reference_centroid
    current_centered = current_world - current_centroid
    covariance = reference_centered.T @ current_centered
    left, _, right_transpose = np.linalg.svd(covariance)
    reference_to_world_rotation = right_transpose.T @ left.T
    raw_determinant = float(np.linalg.det(reference_to_world_rotation))
    if raw_determinant < 0.0:
        raise SurfaceProxyV3Error("moving-boundary correspondence requires a reflection")
    if abs(raw_determinant - 1.0) > 1e-9:
        raise SurfaceProxyV3Error("Kabsch rotation determinant is not +1")

    reference_to_world_translation = (
        current_centroid - reference_to_world_rotation @ reference_centroid
    )
    fitted = (
        (reference_to_world_rotation @ reference.T).T
        + reference_to_world_translation
    )
    residuals = np.linalg.norm(fitted - current_world, axis=1)
    rms_residual = float(math.sqrt(float(np.mean(residuals * residuals))))
    maximum_residual = float(np.max(residuals))
    if rms_residual > MAXIMUM_RMS_RESIDUAL_M:
        raise SurfaceProxyV3Error(
            f"moving-boundary Kabsch RMS residual exceeds limit: {rms_residual:.3e} m"
        )
    if maximum_residual > MAXIMUM_POINT_RESIDUAL_M:
        raise SurfaceProxyV3Error(
            f"moving-boundary Kabsch maximum residual exceeds limit: {maximum_residual:.3e} m"
        )

    world_to_container_rotation = reference_to_world_rotation.T
    world_to_container_translation = (
        -world_to_container_rotation @ reference_to_world_translation
    )
    determinant = float(np.linalg.det(world_to_container_rotation))
    orthogonality_error = float(
        np.linalg.norm(
            world_to_container_rotation.T @ world_to_container_rotation - np.eye(3),
            ord="fro",
        )
    )
    if abs(determinant - 1.0) > 1e-9 or orthogonality_error > 1e-9:
        raise SurfaceProxyV3Error("recovered world-to-container transform is not rigid")
    transform = {
        "world_to_container_rotation": [
            [float(component) for component in row]
            for row in world_to_container_rotation
        ],
        "world_to_container_translation_m": [
            float(component) for component in world_to_container_translation
        ],
        "determinant": determinant,
        "orthogonality_error": orthogonality_error,
        "rms_residual_m": rms_residual,
        "maximum_residual_m": maximum_residual,
        "reflection_detected": False,
        "reference_rank": reference_rank,
        "singular_value_ratio": reference_ratio,
    }
    return world_to_container_rotation, world_to_container_translation, transform


def _quantile_type7(values: Sequence[float], quantile: float = QUANTILE) -> float:
    if not values:
        raise SurfaceProxyV3Error("quantile input is empty")
    ordered = sorted(_finite(value, "surface z") for value in values)
    position = (len(ordered) - 1) * quantile
    left = math.floor(position)
    right = math.ceil(position)
    if left == right:
        return ordered[left]
    fraction = position - left
    return ordered[left] * (1.0 - fraction) + ordered[right] * fraction


def _surface_sectors(
    container_points: np.ndarray,
    container_radius_m: float,
) -> tuple[list[dict[str, Any]], int, int]:
    inner = ANNULUS_INNER_RADIUS_RATIO * container_radius_m
    outer = ANNULUS_OUTER_RADIUS_RATIO * container_radius_m
    values: list[list[float]] = [[] for _ in SECTOR_IDS]
    excluded = 0
    width = 2.0 * math.pi / SECTOR_COUNT
    for point in container_points:
        x, y, z = (float(component) for component in point)
        if not all(math.isfinite(component) for component in (x, y, z)):
            raise SurfaceProxyV3Error("transformed fluid particle is non-finite")
        radius = math.hypot(x, y)
        if radius < inner or radius > outer:
            excluded += 1
            continue
        angle = math.atan2(y, x)
        if angle < 0.0:
            angle += 2.0 * math.pi
        sector_index = min(int(angle / width), SECTOR_COUNT - 1)
        values[sector_index].append(z)

    sectors: list[dict[str, Any]] = []
    for sector_id, sector_values in zip(SECTOR_IDS, values):
        count = len(sector_values)
        valid = count >= MINIMUM_PARTICLES_PER_SECTOR
        sectors.append({
            "sector_id": sector_id,
            "count": count,
            "valid": valid,
            "q98_z_m": _quantile_type7(sector_values) if valid else None,
            "null_reason": None if valid else "INSUFFICIENT_PARTICLES",
        })
    return sectors, sum(len(item) for item in values), excluded


def _method_contract() -> dict[str, Any]:
    return {
        "transform_method": TRANSFORM_METHOD,
        "container_frame_origin": "SETTLED_MOVING_BOUNDARY_REFERENCE",
        "sector_assignment": SECTOR_ASSIGNMENT,
        "sector_ids": list(SECTOR_IDS),
        "sector_count": SECTOR_COUNT,
        "annulus_inner_radius_ratio": ANNULUS_INNER_RADIUS_RATIO,
        "annulus_outer_radius_ratio": ANNULUS_OUTER_RADIUS_RATIO,
        "minimum_particles_per_sector": MINIMUM_PARTICLES_PER_SECTOR,
        "surface_statistic": "CONTAINER_FRAME_FLUID_PARTICLE_Z_Q98",
        "quantile": QUANTILE,
        "quantile_method": QUANTILE_METHOD,
        "maximum_rms_residual_m": MAXIMUM_RMS_RESIDUAL_M,
        "maximum_point_residual_m": MAXIMUM_POINT_RESIDUAL_M,
        "minimum_reference_rank": MINIMUM_REFERENCE_RANK,
        "minimum_singular_value_ratio": MINIMUM_SINGULAR_VALUE_RATIO,
    }


def _validate_fixture(fixture: Any) -> dict[str, Any]:
    value = _require_exact_keys(
        fixture,
        {
            "schema_version",
            "fixture_only",
            "case_id",
            "container_radius_m",
            "frame_contract",
            "settled_moving_boundary",
            "frames",
        },
        "fixture",
    )
    if value["schema_version"] != INPUT_SCHEMA_VERSION:
        raise SurfaceProxyV3Error("fixture schema_version differs")
    if value["fixture_only"] is not True:
        raise SurfaceProxyV3Error("only fixture_only=true input is admitted")
    case_id = value["case_id"]
    if not isinstance(case_id, str) or CASE_ID_PATTERN.fullmatch(case_id) is None:
        raise SurfaceProxyV3Error("case_id is invalid")
    container_radius = _finite(value["container_radius_m"], "container_radius_m")
    if container_radius <= 0.0:
        raise SurfaceProxyV3Error("container_radius_m is not positive")

    frame_contract = _require_exact_keys(
        value["frame_contract"],
        {"start_index", "end_index", "index_step", "start_time_s", "time_step_s"},
        "frame_contract",
    )
    start_index = _integer(frame_contract["start_index"], "frame_contract.start_index")
    end_index = _integer(frame_contract["end_index"], "frame_contract.end_index")
    index_step = _integer(frame_contract["index_step"], "frame_contract.index_step", minimum=1)
    if index_step != 1:
        raise SurfaceProxyV3Error("frame index_step must be exactly 1")
    if end_index < start_index:
        raise SurfaceProxyV3Error("frame end_index precedes start_index")
    start_time = _finite(frame_contract["start_time_s"], "frame_contract.start_time_s")
    time_step = _finite(frame_contract["time_step_s"], "frame_contract.time_step_s")
    if time_step <= 0.0:
        raise SurfaceProxyV3Error("frame time_step_s is not positive")
    expected_indices = tuple(range(start_index, end_index + 1))

    reference_by_id = _particle_rows(
        value["settled_moving_boundary"], "settled_moving_boundary"
    )
    if len(reference_by_id) < 4:
        raise SurfaceProxyV3Error("settled moving boundary requires at least four points")
    boundary_ids = tuple(sorted(reference_by_id))
    reference = _ordered_points(reference_by_id, boundary_ids)
    _geometry_quality(reference, "settled boundary")

    raw_frames = value["frames"]
    if not isinstance(raw_frames, list) or len(raw_frames) != len(expected_indices):
        raise SurfaceProxyV3Error("frame count differs from the complete registered interval")
    checked_frames: list[dict[str, Any]] = []
    first_fluid_ids: tuple[int, ...] | None = None
    for offset, (raw_frame, expected_index) in enumerate(zip(raw_frames, expected_indices)):
        frame = _require_exact_keys(
            raw_frame,
            {"frame_index", "time_s", "moving_boundary", "fluid"},
            f"frames[{offset}]",
        )
        observed_index = _integer(frame["frame_index"], f"frames[{offset}].frame_index")
        if observed_index != expected_index:
            raise SurfaceProxyV3Error("frame sequence is omitted, reordered, or cherry-picked")
        observed_time = _finite(frame["time_s"], f"frames[{offset}].time_s")
        expected_time = start_time + offset * time_step
        if abs(observed_time - expected_time) > TIME_TOLERANCE_S:
            raise SurfaceProxyV3Error("frame time sequence differs from the registered grid")
        current_boundary = _particle_rows(
            frame["moving_boundary"], f"frames[{offset}].moving_boundary"
        )
        if set(current_boundary) != set(boundary_ids):
            raise SurfaceProxyV3Error("moving-boundary IDs are missing, extra, or cherry-picked")
        fluid = _particle_rows(frame["fluid"], f"frames[{offset}].fluid")
        fluid_ids = tuple(sorted(fluid))
        if first_fluid_ids is None:
            first_fluid_ids = fluid_ids
            if set(fluid_ids) & set(boundary_ids):
                raise SurfaceProxyV3Error("moving-boundary and fluid ID classes overlap")
        elif fluid_ids != first_fluid_ids:
            raise SurfaceProxyV3Error("fluid IDs are missing, extra, or cherry-picked across frames")
        checked_frames.append({
            "frame_index": observed_index,
            "time_s": observed_time,
            "moving_boundary": current_boundary,
            "fluid": fluid,
        })
    if first_fluid_ids is None:
        raise SurfaceProxyV3Error("fixture contains no fluid IDs")
    return {
        "case_id": case_id,
        "container_radius_m": container_radius,
        "start_index": start_index,
        "end_index": end_index,
        "start_time_s": start_time,
        "time_step_s": time_step,
        "boundary_ids": boundary_ids,
        "fluid_ids": first_fluid_ids,
        "reference": reference,
        "frames": checked_frames,
    }


def _validate_result_invariants(result: Mapping[str, Any]) -> None:
    frames = result["frames"]
    expected_indices = tuple(
        range(result["frame_contract"]["start_index"], result["frame_contract"]["end_index"] + 1)
    )
    if len(frames) != len(expected_indices):
        raise SurfaceProxyV3Error("result frame count contradicts frame contract")
    invalid_sector_count = 0
    for offset, (frame, expected_index) in enumerate(zip(frames, expected_indices)):
        if frame["frame_index"] != expected_index:
            raise SurfaceProxyV3Error("result frame sequence is incomplete")
        expected_time = (
            result["frame_contract"]["start_time_s"]
            + offset * result["frame_contract"]["time_step_s"]
        )
        if abs(frame["time_s"] - expected_time) > TIME_TOLERANCE_S:
            raise SurfaceProxyV3Error("result time sequence is incomplete")
        sectors = frame["sectors"]
        if tuple(item["sector_id"] for item in sectors) != SECTOR_IDS:
            raise SurfaceProxyV3Error("result sectors are missing, reordered, or cherry-picked")
        if frame["selected_fluid_particle_count"] != sum(item["count"] for item in sectors):
            raise SurfaceProxyV3Error("result selected particle count contradicts sector rows")
        valid_count = sum(bool(item["valid"]) for item in sectors)
        if frame["valid_sector_count"] != valid_count:
            raise SurfaceProxyV3Error("result valid sector count is self-contradictory")
        if frame["frame_valid"] is not (valid_count == SECTOR_COUNT):
            raise SurfaceProxyV3Error("result frame_valid is self-contradictory")
        invalid_sector_count += SECTOR_COUNT - valid_count
    qc = result["qc"]
    all_valid = invalid_sector_count == 0
    expected_count = len(expected_indices)
    if qc["expected_frame_count"] != expected_count or qc["observed_frame_count"] != len(frames):
        raise SurfaceProxyV3Error("result QC frame counts are self-contradictory")
    if qc["invalid_sector_count"] != invalid_sector_count:
        raise SurfaceProxyV3Error("result QC invalid sector count is self-contradictory")
    if qc["all_sectors_valid"] is not all_valid or qc["algorithm_valid"] is not all_valid:
        raise SurfaceProxyV3Error("result QC validity is self-contradictory")
    expected_status = (
        "FIXTURE_ONLY_PROXY_VALID_NOT_ADMITTED"
        if all_valid
        else "FIXTURE_ONLY_PROXY_INVALID_SECTORS_NOT_ADMITTED"
    )
    if result["result_status"] != expected_status:
        raise SurfaceProxyV3Error("result status contradicts sector validity")
    if result["integrity"]["method_contract_sha256"] != sha256_json(result["method_contract"]):
        raise SurfaceProxyV3Error("method contract hash is self-contradictory")
    if result["integrity"]["derived_frames_sha256"] != sha256_json(frames):
        raise SurfaceProxyV3Error("derived frame hash is self-contradictory")


def validate_result(result: Any) -> dict[str, Any]:
    value = _require_mapping(result, "result")
    schema = _load_schema()
    try:
        Draft202012Validator(schema).validate(value)
    except ValidationError as exc:
        location = "/".join(str(item) for item in exc.absolute_path)
        raise SurfaceProxyV3Error(
            f"result schema validation failed at {location or '$'}: {exc.message}"
        ) from exc
    _validate_result_invariants(value)
    canonical_json(value)
    return dict(value)


def analyze_fixture(fixture: Any) -> dict[str, Any]:
    """Derive a closed result entirely from an in-memory synthetic fixture."""

    checked = _validate_fixture(fixture)
    reference = checked["reference"]
    boundary_ids = checked["boundary_ids"]
    fluid_ids = checked["fluid_ids"]
    output_frames: list[dict[str, Any]] = []
    invalid_sector_count = 0
    for frame in checked["frames"]:
        current_boundary = _ordered_points(frame["moving_boundary"], boundary_ids)
        rotation, translation, transform = _recover_world_to_container(
            reference, current_boundary
        )
        fluid_world = _ordered_points(frame["fluid"], fluid_ids)
        fluid_container = (rotation @ fluid_world.T).T + translation
        sectors, selected_count, excluded_count = _surface_sectors(
            fluid_container, checked["container_radius_m"]
        )
        valid_count = sum(bool(item["valid"]) for item in sectors)
        invalid_sector_count += SECTOR_COUNT - valid_count
        output_frames.append({
            "frame_index": frame["frame_index"],
            "time_s": frame["time_s"],
            "transform": transform,
            "selected_fluid_particle_count": selected_count,
            "excluded_radial_particle_count": excluded_count,
            "sectors": sectors,
            "valid_sector_count": valid_count,
            "frame_valid": valid_count == SECTOR_COUNT,
        })

    method_contract = _method_contract()
    frame_count = checked["end_index"] - checked["start_index"] + 1
    all_valid = invalid_sector_count == 0
    result = {
        "schema_version": OUTPUT_SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "case_id": checked["case_id"],
        "fixture_only": True,
        "raw_moving_solver_gauge_available": False,
        "surface_source": SURFACE_SOURCE,
        "is_solver_gauge": False,
        "is_jgaugeswl": False,
        "is_point_probe": False,
        "method_contract": method_contract,
        "frame_contract": {
            "start_index": checked["start_index"],
            "end_index": checked["end_index"],
            "index_step": 1,
            "start_time_s": checked["start_time_s"],
            "time_step_s": checked["time_step_s"],
            "time_tolerance_s": TIME_TOLERANCE_S,
        },
        "particle_contract": {
            "container_radius_m": checked["container_radius_m"],
            "moving_boundary_reference_count": len(boundary_ids),
            "fluid_particle_count": len(fluid_ids),
            "moving_boundary_ids_sha256": sha256_json(list(boundary_ids)),
            "fluid_ids_sha256": sha256_json(list(fluid_ids)),
            "moving_boundary_ids_complete_unique": True,
            "fluid_ids_complete_unique": True,
            "particle_classes_disjoint": True,
        },
        "frames": output_frames,
        "qc": {
            "expected_frame_count": frame_count,
            "observed_frame_count": len(output_frames),
            "frame_sequence_complete": True,
            "time_sequence_complete": True,
            "finite": True,
            "all_transforms_rigid": True,
            "all_sector_rows_complete": True,
            "all_sectors_valid": all_valid,
            "invalid_sector_count": invalid_sector_count,
            "algorithm_valid": all_valid,
        },
        "integrity": {
            "source_fixture_sha256": sha256_json(fixture),
            "method_contract_sha256": sha256_json(method_contract),
            "derived_frames_sha256": sha256_json(output_frames),
        },
        "claims": {
            "fixture_only": True,
            "real_bi4_read": False,
            "solver_or_gpu_executed": False,
            "raw_solver_surface_measured": False,
            "stage5b0_pass": False,
        },
        "result_status": (
            "FIXTURE_ONLY_PROXY_VALID_NOT_ADMITTED"
            if all_valid
            else "FIXTURE_ONLY_PROXY_INVALID_SECTORS_NOT_ADMITTED"
        ),
    }
    return validate_result(result)


def verify_result(fixture: Any, claimed_result: Any) -> dict[str, Any]:
    """Reject even internally re-hashed claims that differ from fresh derivation."""

    claimed = validate_result(claimed_result)
    expected = analyze_fixture(fixture)
    if claimed["integrity"]["source_fixture_sha256"] != sha256_json(fixture):
        raise SurfaceProxyV3Error("claimed result binds a different source fixture")
    if canonical_json(claimed) != canonical_json(expected):
        raise SurfaceProxyV3Error("claimed result differs from fresh particle derivation")
    return expected


def _rotation_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def _particle_list(ids: Sequence[int], points: np.ndarray) -> list[dict[str, Any]]:
    return [
        {
            "id": int(particle_id),
            "xyz_m": [float(component) for component in point],
        }
        for particle_id, point in zip(ids, points)
    ]


def synthetic_fixture() -> dict[str, Any]:
    """Build a small deterministic full-6-DOF fixture without external inputs."""

    reference = np.asarray([
        [-0.80, -0.60, -0.40],
        [0.70, -0.50, 0.50],
        [-0.60, 0.80, 0.40],
        [0.90, 0.70, -0.50],
        [0.10, -0.20, 0.90],
        [-0.20, 0.10, -0.90],
    ], dtype=np.float64)
    boundary_ids = tuple(range(100, 100 + len(reference)))
    fluid_ids: list[int] = []
    base_fluid: list[list[float]] = []
    next_id = 10_000
    for sector_index in range(SECTOR_COUNT):
        angle = (sector_index + 0.5) * 2.0 * math.pi / SECTOR_COUNT
        for sample_index in range(MINIMUM_PARTICLES_PER_SECTOR):
            radius = 0.60 + 0.20 * sample_index / (MINIMUM_PARTICLES_PER_SECTOR - 1)
            z = 0.20 + 0.01 * sector_index + 0.001 * sample_index
            base_fluid.append([radius * math.cos(angle), radius * math.sin(angle), z])
            fluid_ids.append(next_id)
            next_id += 1
    base_fluid_array = np.asarray(base_fluid, dtype=np.float64)
    poses = (
        (0.00, 0.00, 0.00, (0.00, 0.00, 0.00)),
        (0.21, -0.17, 0.13, (0.12, -0.08, 0.05)),
        (-0.19, 0.11, -0.23, (-0.07, 0.09, 0.04)),
    )
    frames: list[dict[str, Any]] = []
    for offset, (yaw, pitch, roll, translation_raw) in enumerate(poses):
        rotation = _rotation_zyx(yaw, pitch, roll)
        translation = np.asarray(translation_raw, dtype=np.float64)
        current_boundary = (rotation @ reference.T).T + translation
        current_container_fluid = base_fluid_array.copy()
        current_container_fluid[:, 2] += 0.005 * offset
        current_fluid_world = (rotation @ current_container_fluid.T).T + translation
        frames.append({
            "frame_index": 901 + offset,
            "time_s": 45.05 + 0.05 * offset,
            "moving_boundary": _particle_list(boundary_ids, current_boundary),
            "fluid": _particle_list(fluid_ids, current_fluid_world),
        })
    return {
        "schema_version": INPUT_SCHEMA_VERSION,
        "fixture_only": True,
        "case_id": "fixture_full_se3_q98_v3",
        "container_radius_m": 1.0,
        "frame_contract": {
            "start_index": 901,
            "end_index": 903,
            "index_step": 1,
            "start_time_s": 45.05,
            "time_step_s": 0.05,
        },
        "settled_moving_boundary": _particle_list(boundary_ids, reference),
        "frames": frames,
    }


def self_check() -> dict[str, Any]:
    schema = _load_schema()
    result = analyze_fixture(synthetic_fixture())
    verify_result(synthetic_fixture(), result)
    return {
        "status": "S5B0_CONTAINER_SURFACE_PROXY_V3_FIXTURE_SELF_CHECK_OK_NOT_ADMITTED",
        "schema_id": schema["$id"],
        "frame_count": len(result["frames"]),
        "sector_ids": list(SECTOR_IDS),
        "algorithm_valid": result["qc"]["algorithm_valid"],
        "raw_moving_solver_gauge_available": False,
        "surface_source": SURFACE_SOURCE,
        "is_solver_gauge": False,
        "is_jgaugeswl": False,
        "is_point_probe": False,
        "real_bi4_read": False,
        "solver_or_gpu_executed": False,
        "stage5b0_pass": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--self-check",
        action="store_true",
        help="run the in-memory synthetic fixture self-check",
    )
    arguments = parser.parse_args(argv)
    if not arguments.self_check:
        parser.error("only --self-check is supported; no real input path is admitted")
    print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
