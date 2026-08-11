#!/usr/bin/env python3
"""Read-only, fail-closed QC for the completed U3 DDT-ramp experiment.

The experiment compares the same 8.05..10.05 s continuation window from one
common Part_0161 checkpoint.  It never starts a solver and writes only one
explicit create-new JSON result outside both input trees.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import stat
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_solver_output_qc_v3 as qc_v3


SCHEMA_VERSION = "r8-liquid-u3-ddt-ramp-qc-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_U3_DDT_RAMP_QC_V1"
PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"
CASE_BI4_SHA256 = "b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7"
CASE_XML_SHA256 = "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb"
BASELINE_RUN_OUT_SHA256 = "e4c04361c11ed5034b91d40646cad26d8a2d5b0a8054b7ec02b56f9a7bf8359a"
DDT_RUN_OUT_SHA256 = "767f37e5c5f58c84d18206b3d8cf9ed113c100e388c69318f665657bd2f75112"
BASELINE_QC_SHA256 = "5fc4c8161317eb99f128830743311b677765e793a1ccfef4c9a1bca1fd8bc0ea"

EXPECTED_PART_INDICES = tuple(range(161, 202))
EXPECTED_PART_FILES = tuple(f"Part_{index:04d}.bi4" for index in EXPECTED_PART_INDICES)
STATIC_DATA_FILES = ("Part_Head.ibi4", "PartInfo.ibi4", "PartMotionRef.ibi4", "PartOut_000.obi4")
ROOT_FILES = ("CfgInit_Domain.vtk", "CfgInit_MapCells.vtk", "Run.csv", "Run.out", "RunPARTs.csv")
EXPECTED_FILE_COUNT = 50
MAX_FILE_BYTES = 16 * 1024 * 1024
MAX_TREE_BYTES = 64 * 1024 * 1024
MAX_RESULT_BYTES = 4 * 1024 * 1024


class DdtRampQcError(ValueError):
    """Unsafe, incomplete, or semantically invalid frozen input."""


@dataclass(frozen=True)
class FrozenContract:
    expected_start_time_s: float = 8.05
    expected_end_time_s: float = 10.05
    expected_output_period_s: float = 0.05
    # Solver output slots land within one integration step of the requested
    # 0.05 s slot; 50 microseconds is still 1/1000 of the output period.
    physical_time_tolerance_s: float = 5.0e-5
    tail_window_s: float = 1.0
    minimum_tail_coverage_s: float = 0.95
    expected_particle_count: int = 9078
    expected_moving_count: int = 2669
    expected_fluid_count: int = 6409
    baseline_ddt_value: float = 0.1
    candidate_ddt_value: float = 0.1
    candidate_ddt_value_max: float = 0.2
    candidate_ddt_tmax_s: float = 8.05
    candidate_ddt_tramp_s: float = 10.05
    maximum_metric_regression_fraction_of_limit: float = 0.05
    meaningful_metric_improvement_fraction_of_limit: float = 0.02
    minimum_primary_metrics_improved: int = 2
    maximum_primary_normalized_score_ratio: float = 0.95


CONTRACT = FrozenContract()

# All limits are frozen before this comparator reads either continuation tree.
# Lower is better for every entry.
METRIC_LIMITS: dict[str, float] = {
    "speed_rms_m_s": 1.0e-3,
    "speed_p95_m_s": 1.0e-3,
    "speed_max_m_s": 5.0e-3,
    "specific_kinetic_energy_j_kg": 5.0e-7,
    "position_interframe_rms_m_s": 1.0e-3,
    "position_interframe_p95_m_s": 1.0e-3,
    "position_interframe_max_m_s": 5.0e-3,
    "position_net_rms_m_s": 1.0e-3,
    "position_net_p95_m_s": 1.0e-3,
    "position_net_max_m_s": 5.0e-3,
    "density_mean_relative_range": 5.0e-3,
    "density_mean_relative_bias": 1.0e-2,
    "density_p01_p99_relative_deviation": 5.0e-2,
    "surface_abs_bias_m": 1.0e-3,
    "surface_spread_m": 1.0e-3,
    "surface_temporal_range_m": 5.0e-4,
    "surface_abs_drift_m_s": 1.0e-4,
}
PRIMARY_METRICS = (
    "speed_rms_m_s",
    "speed_p95_m_s",
    "speed_max_m_s",
    "specific_kinetic_energy_j_kg",
)


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_sha256(value: Any) -> str:
    return _sha256_bytes(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    )


def _same(left: float, right: float, tolerance: float = 1.0e-9) -> bool:
    return math.isclose(left, right, rel_tol=tolerance, abs_tol=tolerance)


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DdtRampQcError(f"{label} is not numeric")
    result = float(value)
    if not math.isfinite(result):
        raise DdtRampQcError(f"{label} is not finite")
    return result


def _identity(path: Path, *, maximum: int = MAX_FILE_BYTES, expected_sha256: str | None = None) -> dict[str, Any]:
    try:
        source = bi4.read_regular_file(path, max_bytes=maximum)
    except (bi4.Bi4FormatError, ValueError) as exc:
        raise DdtRampQcError(str(exc)) from exc
    if expected_sha256 is not None and source.sha256 != expected_sha256:
        raise DdtRampQcError(f"SHA-256 mismatch for {path}: {source.sha256}")
    metadata = os.lstat(path)
    if metadata.st_nlink != 1:
        raise DdtRampQcError(f"link count differs from one: {path}")
    return {
        "path": str(path),
        "size_bytes": source.size_bytes,
        "sha256": source.sha256,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "nlink": metadata.st_nlink,
    }


def _require_directory(path: Path, label: str) -> Path:
    try:
        metadata = os.lstat(path)
    except OSError as exc:
        raise DdtRampQcError(f"cannot inspect {label} {path}: {exc}") from exc
    if not stat.S_ISDIR(metadata.st_mode):
        raise DdtRampQcError(f"{label} is not a real directory: {path}")
    return path


def _scan_inventory(root: Path) -> dict[str, Any]:
    root = _require_directory(root, "run root")
    data = _require_directory(root / "data", "run data")
    expected_root = set(ROOT_FILES) | {"data"}
    root_entries = {entry.name: entry for entry in os.scandir(root)}
    data_entries = {entry.name: entry for entry in os.scandir(data)}
    expected_data = set(STATIC_DATA_FILES) | set(EXPECTED_PART_FILES)
    if set(root_entries) != expected_root:
        raise DdtRampQcError(
            f"root inventory differs; missing={sorted(expected_root-set(root_entries))}, "
            f"extra={sorted(set(root_entries)-expected_root)}"
        )
    if set(data_entries) != expected_data:
        raise DdtRampQcError(
            f"data inventory differs; missing={sorted(expected_data-set(data_entries))}, "
            f"extra={sorted(set(data_entries)-expected_data)}"
        )
    files: dict[str, dict[str, Any]] = {}
    total = 0
    for relative in [*ROOT_FILES, *(f"data/{name}" for name in (*STATIC_DATA_FILES, *EXPECTED_PART_FILES))]:
        entry = root_entries[relative] if "/" not in relative else data_entries[relative.split("/", 1)[1]]
        metadata = entry.stat(follow_symlinks=False)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise DdtRampQcError(f"unsafe non-regular or linked output: {relative}")
        if not 0 < metadata.st_size <= MAX_FILE_BYTES:
            raise DdtRampQcError(f"output size outside 1..{MAX_FILE_BYTES}: {relative}")
        item = _identity(root / relative)
        files[relative] = {
            "size_bytes": item["size_bytes"],
            "sha256": item["sha256"],
            "mode": item["mode"],
            "nlink": item["nlink"],
        }
        total += metadata.st_size
    if len(files) != EXPECTED_FILE_COUNT or total > MAX_TREE_BYTES:
        raise DdtRampQcError(f"inventory count/size invalid: count={len(files)}, bytes={total}")
    return {
        "file_count": len(files),
        "total_bytes": total,
        "canonical_sha256": _canonical_sha256(files),
        "files": dict(sorted(files.items())),
    }


def _thresholds() -> qc_v3.Thresholds:
    return qc_v3.Thresholds(
        minimum_settle_time_s=CONTRACT.expected_end_time_s,
        tail_window_s=CONTRACT.tail_window_s,
        physical_time_tolerance_s=CONTRACT.physical_time_tolerance_s,
        max_tail_speed_rms_m_s=METRIC_LIMITS["speed_rms_m_s"],
        max_tail_speed_p95_m_s=METRIC_LIMITS["speed_p95_m_s"],
        max_tail_speed_m_s=METRIC_LIMITS["speed_max_m_s"],
        max_tail_specific_kinetic_energy_j_kg=METRIC_LIMITS["specific_kinetic_energy_j_kg"],
        max_surface_bias_dp=0.5,
        max_surface_spread_dp=0.5,
        max_surface_temporal_range_dp=0.25,
        max_surface_drift_dp_per_s=0.05,
        max_density_mean_relative_range=METRIC_LIMITS["density_mean_relative_range"],
        surface_sector_count=16,
        surface_quantile=0.99,
        surface_inner_margin_dp=2.0,
        minimum_surface_particles_per_sector=4,
    )


def _parse_ddt_parameters(run_out: bi4.SecureFile, *, candidate: bool) -> dict[str, Any]:
    try:
        text = run_out.data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise DdtRampQcError("Run.out is not UTF-8") from exc
    fields: dict[str, float | None] = {}
    for key in ("DensityDiffusionValue", "DensityDiffusionTRamp", "DensityDiffusionTMax", "DensityDiffusionValueMax"):
        matches = re.findall(rf"^\s*{re.escape(key)}=([^\s]+)\s*$", text, flags=re.MULTILINE)
        if len(matches) > 1 or (key == "DensityDiffusionValue" and len(matches) != 1):
            raise DdtRampQcError(f"unexpected {key} cardinality in Run.out")
        fields[key] = _finite(float(matches[0]), key) if matches else None
    expected = {
        "DensityDiffusionValue": CONTRACT.candidate_ddt_value if candidate else CONTRACT.baseline_ddt_value,
        "DensityDiffusionTRamp": CONTRACT.candidate_ddt_tramp_s if candidate else None,
        "DensityDiffusionTMax": CONTRACT.candidate_ddt_tmax_s if candidate else None,
        "DensityDiffusionValueMax": CONTRACT.candidate_ddt_value_max if candidate else None,
    }
    if any(
        (observed is None) != (wanted is None)
        or (observed is not None and wanted is not None and not _same(observed, wanted, 1.0e-12))
        for (observed, wanted) in zip(fields.values(), expected.values())
    ):
        raise DdtRampQcError(f"DDT parameter identity differs: {fields!r}")
    return {
        "value": fields["DensityDiffusionValue"],
        "tramp_s": fields["DensityDiffusionTRamp"],
        "tmax_s": fields["DensityDiffusionTMax"],
        "value_max": fields["DensityDiffusionValueMax"],
    }


def _parse_runparts(source: bi4.SecureFile, parts: list[dict[str, Any]]) -> dict[str, Any]:
    try:
        text = source.data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise DdtRampQcError("RunPARTs.csv is not UTF-8") from exc
    lines = [line for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]
    rows = list(csv.reader(lines, delimiter=";"))
    if not rows or len(rows) != len(parts) + 1:
        raise DdtRampQcError("RunPARTs.csv row count differs from PART files")
    header = rows[0]
    required = ("Part", "TimeStep [s]", "Steps", "NpSave", "NpOut")
    if len(set(header)) != len(header) or any(name not in header for name in required):
        raise DdtRampQcError("RunPARTs.csv columns differ")
    indices = {name: header.index(name) for name in required}
    for row, part in zip(rows[1:], parts):
        if len(row) != len(header):
            raise DdtRampQcError("RunPARTs.csv row width differs")
        observed = int(row[indices["Part"]])
        if observed != part["cpart"]:
            raise DdtRampQcError("RunPARTs.csv Part differs from PART Cpart")
        if not _same(float(row[indices["TimeStep [s]"]]), part["time_s"], 2.0e-6):
            raise DdtRampQcError("RunPARTs.csv time differs from PART")
        if int(row[indices["NpSave"]].replace(",", "")) != CONTRACT.expected_particle_count:
            raise DdtRampQcError("RunPARTs.csv NpSave differs")
        if int(row[indices["NpOut"]].replace(",", "")) != 0:
            raise DdtRampQcError("RunPARTs.csv NpOut is nonzero")
    return {"row_count": len(parts), "first_part": parts[0]["cpart"], "last_part": parts[-1]["cpart"], "pass": True}


def _tail(parts: list[dict[str, Any]]) -> list[dict[str, Any]]:
    cutoff = parts[-1]["time_s"] - CONTRACT.tail_window_s
    return [part for part in parts if part["time_s"] >= cutoff - 1.0e-12]


def _position_metrics(parts: list[dict[str, Any]], case: dict[str, Any]) -> dict[str, float]:
    selected = _tail(parts)
    fluid_begin = case["counts"]["fixed_boundary"] + case["counts"]["moving_boundary"] + case["counts"]["floating"]
    fluid_ids = range(fluid_begin, case["particle_count"])
    inter_rms: list[float] = []
    inter_p95: list[float] = []
    inter_max: list[float] = []
    for left, right in zip(selected, selected[1:]):
        dt = right["time_s"] - left["time_s"]
        if dt <= 0:
            raise DdtRampQcError("tail time is not strictly increasing")
        rates = [
            math.sqrt(sum((right["_positions_by_id"][pid][axis] - left["_positions_by_id"][pid][axis]) ** 2 for axis in range(3))) / dt
            for pid in fluid_ids
        ]
        inter_rms.append(math.sqrt(sum(value * value for value in rates) / len(rates)))
        inter_p95.append(qc_v3._quantile(rates, 0.95))
        inter_max.append(max(rates))
    duration = selected[-1]["time_s"] - selected[0]["time_s"]
    if duration <= 0 or not inter_rms:
        raise DdtRampQcError("tail position coverage is incomplete")
    net = [
        math.sqrt(sum((selected[-1]["_positions_by_id"][pid][axis] - selected[0]["_positions_by_id"][pid][axis]) ** 2 for axis in range(3))) / duration
        for pid in fluid_ids
    ]
    return {
        "position_interframe_rms_m_s": max(inter_rms),
        "position_interframe_p95_m_s": max(inter_p95),
        "position_interframe_max_m_s": max(inter_max),
        "position_net_rms_m_s": math.sqrt(sum(value * value for value in net) / len(net)),
        "position_net_p95_m_s": qc_v3._quantile(net, 0.95),
        "position_net_max_m_s": max(net),
    }


def _density_metrics(parts: list[dict[str, Any]], case: dict[str, Any]) -> dict[str, float]:
    selected = _tail(parts)
    rho0 = case["xml"]["rhop0_kg_m3"]
    means = [_finite(item["density"]["mean_kg_m3"], "density mean") for item in selected]
    p01 = [_finite(item["density"]["p01_kg_m3"], "density p01") for item in selected]
    p99 = [_finite(item["density"]["p99_kg_m3"], "density p99") for item in selected]
    return {
        "density_mean_relative_bias": max(abs(value - rho0) / rho0 for value in means),
        "density_p01_p99_relative_deviation": max(max(abs(low-rho0), abs(high-rho0)) / rho0 for low, high in zip(p01, p99)),
    }


def _metric_vector(parts: list[dict[str, Any]], case: dict[str, Any]) -> tuple[dict[str, float], dict[str, Any]]:
    tail = qc_v3._tail_metrics(parts, case, _thresholds())
    surface = tail["surface_proxy"]
    if not surface["all_tail_parts_valid"]:
        raise DdtRampQcError("surface proxy does not cover every tail frame")
    metrics = {
        "speed_rms_m_s": _finite(tail["max_speed_rms_m_s"], "speed RMS"),
        "speed_p95_m_s": _finite(tail["max_speed_p95_m_s"], "speed P95"),
        "speed_max_m_s": _finite(tail["max_speed_m_s"], "speed max"),
        "specific_kinetic_energy_j_kg": _finite(tail["max_specific_kinetic_energy_j_kg"], "specific KE"),
        "density_mean_relative_range": _finite(tail["density_mean_relative_range"], "density range"),
        "surface_abs_bias_m": abs(_finite(surface["bias_from_hswl_m"], "surface bias")),
        "surface_spread_m": _finite(surface["max_spatial_spread_m"], "surface spread"),
        "surface_temporal_range_m": _finite(surface["temporal_range_m"], "surface range"),
        "surface_abs_drift_m_s": abs(_finite(surface["linear_drift_m_s"], "surface drift")),
    }
    metrics.update(_position_metrics(parts, case))
    metrics.update(_density_metrics(parts, case))
    if set(metrics) != set(METRIC_LIMITS):
        raise DdtRampQcError("metric vector differs from frozen contract")
    return dict(sorted(metrics.items())), {
        "part_count": tail["part_count"],
        "start_time_s": tail["start_time_s"],
        "end_time_s": tail["end_time_s"],
        "coverage_s": tail["coverage_s"],
        "surface_valid_part_count": surface["valid_part_count"],
    }


def _configuration_without_ddt(run_csv_source: bi4.SecureFile) -> str:
    text = run_csv_source.data.decode("utf-8")
    lines = [line for line in text.splitlines() if line.strip()]
    rows = list(csv.reader(lines, delimiter=";"))
    values = dict(zip(rows[0], rows[1]))
    configuration = values.get("Configuration")
    if not configuration:
        raise DdtRampQcError("Run.csv Configuration is missing")
    normalized, count = re.subn(r"DDT2\([^)]*\)", "DDT2(<FROZEN_SINGLE_DELTA>)", configuration)
    if count != 1:
        raise DdtRampQcError("Run.csv does not contain exactly one DDT2 token")
    return normalized


def _parse_continuation_motion_ref(
    source: bi4.SecureFile,
    parts: list[dict[str, Any]],
    case: dict[str, Any],
    thresholds: qc_v3.Thresholds,
) -> dict[str, Any]:
    """Validate a continuation PartMotionRef whose item names start at 0161.

    The reviewed v3 reader intentionally accepts only cold runs named from
    PART_0000.  Continuation files retain global Cpart/item names, so the same
    invariants are reproduced here with the name bound to each actual Cpart.
    """

    items = qc_v3._parse_jbd_list(source, "JPartMotRefBi4")
    root, appended = items[0], items[1:]
    if root.hidden or root.values_hidden or root.items:
        raise DdtRampQcError("PartMotionRef root structure differs")
    expected_root_value_types = {
        "AppName": 1,
        "FormatVer": 8,
        "MainFile": 2,
        "TimeOut": 12,
        "MkBoundFirst": 6,
        "MkMovingCount": 8,
        "MkFloatCount": 8,
        "MkCount": 8,
    }
    if set(root.values) != set(expected_root_value_types):
        raise DdtRampQcError("PartMotionRef root value set differs")
    for name, type_code in expected_root_value_types.items():
        qc_v3._require_value_type(root, name, type_code)
    expected_values = {
        "FormatVer": 230729,
        "MainFile": True,
        "MkMovingCount": 1,
        "MkFloatCount": 0,
        "MkCount": 1,
    }
    if any(root.values.get(name) != value for name, value in expected_values.items()):
        raise DdtRampQcError("PartMotionRef root identity differs")
    if not qc_v3._same_float(qc_v3._finite_float(root.values["TimeOut"], "motion TimeOut"), case["xml"]["time_out_xml_s"]):
        raise DdtRampQcError("PartMotionRef TimeOut differs")
    if root.values["MkBoundFirst"] != case["xml"]["mkboundfirst"]:
        raise DdtRampQcError("PartMotionRef MkBoundFirst differs")

    expected_arrays = {
        "MkBound": (6, 1),
        "Nid": (8, 1),
        "Id": (8, 3),
        "Ps": (23, 3),
        "Dis": (12, 3),
    }
    if set(root.arrays) != set(expected_arrays):
        raise DdtRampQcError("PartMotionRef root array set differs")
    for name, (type_code, count) in expected_arrays.items():
        array = root.arrays[name]
        if array.type_code != type_code or array.count != count or array.hidden:
            raise DdtRampQcError(f"PartMotionRef root array differs: {name}")
    mkbounds = root.arrays["MkBound"].records()
    nids = root.arrays["Nid"].records()
    reference_ids = root.arrays["Id"].records()
    root_positions = root.arrays["Ps"].records()
    distances = root.arrays["Dis"].records()
    if mkbounds != [case["xml"]["moving_mkbound"]] or nids != [3]:
        raise DdtRampQcError("PartMotionRef moving block differs")
    moving_end = case["counts"]["moving_boundary"]
    if len(set(reference_ids)) != 3 or not all(0 <= value < moving_end for value in reference_ids):
        raise DdtRampQcError("PartMotionRef reference Ids differ")
    expected_positions = [case["positions_by_id"][value] for value in reference_ids]
    if root_positions != expected_positions:
        raise DdtRampQcError("PartMotionRef root positions differ")
    expected_distances = [
        1.1 * math.sqrt(sum((position[axis] - root_positions[0][axis]) ** 2 for axis in range(3)))
        for position in root_positions
    ]
    if not all(qc_v3._same_float(observed, expected) for observed, expected in zip(distances, expected_distances)):
        raise DdtRampQcError("PartMotionRef root distances differ")
    if len(appended) != len(parts):
        raise DdtRampQcError("PartMotionRef frame count differs")

    first_positions: list[Any] | None = None
    for item, particle_part in zip(appended, parts):
        expected_name = f"PART_{particle_part['cpart']:04d}"
        if item.name != expected_name:
            raise DdtRampQcError(f"unexpected PartMotionRef item {item.name!r}")
        if not item.hidden or item.values_hidden or item.items or set(item.arrays) != {"PosRef"}:
            raise DdtRampQcError(f"PartMotionRef item structure differs: {expected_name}")
        if set(item.values) != {"Cpart", "TimeStep", "Step"}:
            raise DdtRampQcError(f"PartMotionRef item values differ: {expected_name}")
        qc_v3._require_value_type(item, "Cpart", 8)
        qc_v3._require_value_type(item, "TimeStep", 12)
        qc_v3._require_value_type(item, "Step", 8)
        cpart = qc_v3._required_uint(item.values, "Cpart")
        step = qc_v3._required_uint(item.values, "Step")
        time_s = qc_v3._finite_float(item.values["TimeStep"], "motion TimeStep")
        positions_array = item.arrays["PosRef"]
        if positions_array.type_code != 23 or positions_array.count != 3 or positions_array.hidden:
            raise DdtRampQcError(f"PartMotionRef PosRef differs: {expected_name}")
        positions = positions_array.records()
        if not all(math.isfinite(value) for vector in positions for value in vector):
            raise DdtRampQcError(f"PartMotionRef PosRef is non-finite: {expected_name}")
        if cpart != particle_part["cpart"] or step != particle_part["step"]:
            raise DdtRampQcError(f"PartMotionRef index/step differs: {expected_name}")
        if abs(time_s - particle_part["time_s"]) > thresholds.physical_time_tolerance_s:
            raise DdtRampQcError(f"PartMotionRef time differs: {expected_name}")
        if positions != [particle_part["_positions_by_id"].get(value) for value in reference_ids]:
            raise DdtRampQcError(f"PartMotionRef positions differ from Part: {expected_name}")
        if first_positions is None:
            first_positions = positions
        elif positions != first_positions:
            raise DdtRampQcError("PartMotionRef changes during zero-motion continuation")
    return {
        "sha256": source.sha256,
        "part_count": len(appended),
        "all_posref_finite": True,
        "all_posref_unchanged": True,
        "aligned_with_particle_parts": True,
    }


def analyze_continuation(root: Path, case: dict[str, Any], *, candidate: bool, expected_run_out_sha256: str) -> dict[str, Any]:
    inventory = _scan_inventory(root)
    run_out_source = qc_v3._read(root / "Run.out", maximum=MAX_FILE_BYTES, expected_sha256=expected_run_out_sha256)
    run_csv_source = qc_v3._read(root / "Run.csv", maximum=MAX_FILE_BYTES)
    run_csv = qc_v3.parse_run_csv(run_csv_source)
    run_out = qc_v3.parse_run_out(run_out_source)
    parameters = _parse_ddt_parameters(run_out_source, candidate=candidate)
    thresholds = _thresholds()
    parts = [
        qc_v3.parse_part_file(
            qc_v3._read(root / "data" / f"Part_{index:04d}.bi4", maximum=MAX_FILE_BYTES),
            index,
            case,
            thresholds,
        )
        for index in EXPECTED_PART_INDICES
    ]
    partout = qc_v3.parse_partout(qc_v3._read(root / "data/PartOut_000.obi4", maximum=MAX_FILE_BYTES))
    motion = _parse_continuation_motion_ref(
        qc_v3._read(root / "data/PartMotionRef.ibi4", maximum=MAX_FILE_BYTES),
        parts,
        case,
        thresholds,
    )
    runparts = _parse_runparts(qc_v3._read(root / "RunPARTs.csv", maximum=MAX_FILE_BYTES), parts)
    times = [item["time_s"] for item in parts]
    steps = [item["step"] for item in parts]
    fields = run_csv["physics_fields"]
    structural_checks = {
        "exact_inventory": inventory["file_count"] == EXPECTED_FILE_COUNT,
        "exact_part_range": [item["cpart"] for item in parts] == list(EXPECTED_PART_INDICES),
        "strictly_increasing_times": all(right > left for left, right in zip(times, times[1:])),
        "monotonic_steps": all(right >= left for left, right in zip(steps, steps[1:])),
        "start_time_matches": abs(times[0] - CONTRACT.expected_start_time_s) <= CONTRACT.physical_time_tolerance_s,
        "end_time_matches": abs(times[-1] - CONTRACT.expected_end_time_s) <= CONTRACT.physical_time_tolerance_s,
        "all_particle_arrays_finite": all(all(item["finite"].values()) for item in parts),
        "all_ids_complete_unique_in_range": all(item["ids_unique"] and item["ids_in_range"] and item["missing_id_count"] == 0 for item in parts),
        "particle_classes_constant": all(item["counts"]["moving_boundary"] == CONTRACT.expected_moving_count and item["counts"]["fluid"] == CONTRACT.expected_fluid_count and item["npok"] == CONTRACT.expected_particle_count for item in parts),
        "root_invariants_match_case": all(item["root_case_invariants_match"] for item in parts),
        "moving_boundary_zero_motion": all(item["moving_positions_exact"] and item["moving_velocities_zero"] for item in parts),
        "zero_nout": all(item["nout"] == 0 for item in parts) and partout["total_nout"] == 0 and fields["PartsOut"] == 0 and run_out["excluded_particles"] == 0,
        "finite_inside_domain_density": all(item["domain_outside_count"] == 0 and item["density_outside_count"] == 0 for item in parts),
        "motion_reference_zero_aligned": motion["all_posref_finite"] and motion["all_posref_unchanged"] and motion["aligned_with_particle_parts"] and motion["part_count"] == len(parts),
        "runparts_matches": runparts["pass"],
        "run_csv_matches": fields["Np"] == CONTRACT.expected_particle_count and fields["PartFiles"] == 202 and abs(fields["PhysicalTime"] - times[-1]) <= CONTRACT.physical_time_tolerance_s,
        "run_out_clean_complete": run_out["completion_markers_pass"] and not any(run_out["bad_patterns"].values()),
    }
    if not all(structural_checks.values()):
        failed = [name for name, passed in structural_checks.items() if not passed]
        raise DdtRampQcError(f"structural checks failed for {root}: {failed}")
    metrics, tail = _metric_vector(parts, case)
    if tail["coverage_s"] < CONTRACT.minimum_tail_coverage_s:
        raise DdtRampQcError("tail coverage is below the frozen minimum")
    for part in parts:
        part.pop("_positions_by_id", None)
    return {
        "root": str(root),
        "inventory": inventory,
        "run_out_sha256": run_out["sha256"],
        "run_csv_sha256": run_csv["sha256"],
        "configuration_without_ddt": _configuration_without_ddt(run_csv_source),
        "ddt_parameters": parameters,
        "frame_count": len(parts),
        "first_part": parts[0]["cpart"],
        "last_part": parts[-1]["cpart"],
        "first_time_s": parts[0]["time_s"],
        "last_time_s": parts[-1]["time_s"],
        "initial_checkpoint_sha256": inventory["files"]["data/Part_0161.bi4"]["sha256"],
        "tail": tail,
        "metrics": metrics,
        "structural_checks": structural_checks,
    }


def compare_metric_vectors(baseline: dict[str, float], candidate: dict[str, float]) -> dict[str, Any]:
    if set(baseline) != set(METRIC_LIMITS) or set(candidate) != set(METRIC_LIMITS):
        raise DdtRampQcError("comparison metric keys differ from frozen contract")
    records: dict[str, dict[str, Any]] = {}
    for name, limit in sorted(METRIC_LIMITS.items()):
        before = _finite(baseline[name], f"baseline {name}")
        after = _finite(candidate[name], f"candidate {name}")
        slack = CONTRACT.maximum_metric_regression_fraction_of_limit * limit
        improvement = CONTRACT.meaningful_metric_improvement_fraction_of_limit * limit
        records[name] = {
            "baseline": before,
            "candidate": after,
            "delta": after - before,
            "candidate_to_baseline_ratio": after / before if before != 0 else None,
            "percent_change": 100.0 * (after - before) / before if before != 0 else None,
            "absolute_limit": limit,
            "candidate_absolute_pass": after <= limit,
            "no_material_regression": after <= before + slack,
            "meaningful_improvement": after <= before - improvement,
        }
    baseline_score = sum(baseline[name] / METRIC_LIMITS[name] for name in PRIMARY_METRICS) / len(PRIMARY_METRICS)
    candidate_score = sum(candidate[name] / METRIC_LIMITS[name] for name in PRIMARY_METRICS) / len(PRIMARY_METRICS)
    primary_improved = sum(records[name]["meaningful_improvement"] for name in PRIMARY_METRICS)
    checks = {
        "all_candidate_absolute_limits_pass": all(item["candidate_absolute_pass"] for item in records.values()),
        "no_metric_materially_regresses": all(item["no_material_regression"] for item in records.values()),
        "minimum_primary_metrics_meaningfully_improve": primary_improved >= CONTRACT.minimum_primary_metrics_improved,
        "primary_normalized_score_improves_by_at_least_5_percent": candidate_score <= baseline_score * CONTRACT.maximum_primary_normalized_score_ratio,
    }
    return {
        "metrics": records,
        "baseline_primary_normalized_score": baseline_score,
        "candidate_primary_normalized_score": candidate_score,
        "candidate_to_baseline_primary_score_ratio": candidate_score / baseline_score if baseline_score != 0 else None,
        "primary_metrics_meaningfully_improved": primary_improved,
        "checks": checks,
        "pass": all(checks.values()),
    }


def build_report(
    baseline_root: Path,
    ddt_root: Path,
    case_bi4: Path,
    case_xml: Path,
    plan: Path,
    baseline_qc: Path,
) -> dict[str, Any]:
    plan_identity = _identity(plan, expected_sha256=PLAN_SHA256)
    case_bi4_identity = _identity(case_bi4, expected_sha256=CASE_BI4_SHA256)
    case_xml_identity = _identity(case_xml, expected_sha256=CASE_XML_SHA256)
    baseline_qc_identity = _identity(baseline_qc, maximum=MAX_RESULT_BYTES, expected_sha256=BASELINE_QC_SHA256)
    case = qc_v3.load_moving_case(
        case_bi4,
        case_xml,
        expected_bi4_sha256=CASE_BI4_SHA256,
        expected_xml_sha256=CASE_XML_SHA256,
    )
    baseline = analyze_continuation(baseline_root, case, candidate=False, expected_run_out_sha256=BASELINE_RUN_OUT_SHA256)
    candidate = analyze_continuation(ddt_root, case, candidate=True, expected_run_out_sha256=DDT_RUN_OUT_SHA256)
    comparison = compare_metric_vectors(baseline["metrics"], candidate["metrics"])
    pair_checks = {
        "same_initial_checkpoint_sha256": baseline["initial_checkpoint_sha256"] == candidate["initial_checkpoint_sha256"],
        "same_non_ddt_configuration": baseline["configuration_without_ddt"] == candidate["configuration_without_ddt"],
        "same_frame_and_time_window": (
            baseline["frame_count"], baseline["first_part"], baseline["last_part"]
        ) == (
            candidate["frame_count"], candidate["first_part"], candidate["last_part"]
        ) and abs(baseline["first_time_s"] - candidate["first_time_s"]) <= CONTRACT.physical_time_tolerance_s and abs(baseline["last_time_s"] - candidate["last_time_s"]) <= CONTRACT.physical_time_tolerance_s,
        "both_structural_checks_pass": all(baseline["structural_checks"].values()) and all(candidate["structural_checks"].values()),
    }
    selected = all(pair_checks.values()) and comparison["pass"]
    return {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            "plan": plan_identity,
            "case_bi4": case_bi4_identity,
            "case_xml": case_xml_identity,
            "baseline_qc": baseline_qc_identity,
        },
        "contract": {
            "frozen_before_comparison_execution": True,
            "command_line_threshold_overrides_allowed": False,
            "window_part_indices": [161, 201],
            "window_time_s": [CONTRACT.expected_start_time_s, CONTRACT.expected_end_time_s],
            "tail_window_s": CONTRACT.tail_window_s,
            "contract_values": asdict(CONTRACT),
            "metric_limits": dict(sorted(METRIC_LIMITS.items())),
            "primary_metrics": list(PRIMARY_METRICS),
            "single_variable": "DDT2(0.1)_TO_DDT2(0.1:t10.05:t8.05:v0.2)",
        },
        "baseline": baseline,
        "candidate": candidate,
        "pair_checks": pair_checks,
        "comparison": comparison,
        "verdict": {
            "status": "PASS_U3_DDT_RAMP_NUMERICAL_CANDIDATE" if selected else "FAIL_U3_DDT_RAMP_NUMERICAL_CANDIDATE",
            "candidate_selected": selected,
            "next": "S4B_FREEZE_LIQUID_NUMERICAL_CONTRACT" if selected else "STOP_PRE_REGISTER_SINGLE_VARIABLE_REMEDIATION",
            "development_only": True,
            "formal": False,
            "fidelity_validation_status": "SIM_ONLY_UNVALIDATED",
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "inputs_modified": False,
            "settled_state_claim_allowed": False,
        },
    }


def _serialize(report: dict[str, Any]) -> bytes:
    data = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    if len(data) > MAX_RESULT_BYTES:
        raise DdtRampQcError("result exceeds frozen output limit")
    return data


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    parent = _require_directory(path.parent, "result parent")
    if any(path.is_relative_to(Path(report[name]["root"])) for name in ("baseline", "candidate")):
        raise DdtRampQcError("result path is inside an input tree")
    data = _serialize(report)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(parent / path.name, flags, 0o640)
    try:
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise DdtRampQcError("short write while publishing result")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    os.chmod(path, 0o640, follow_symlinks=False)
    return _identity(path, maximum=MAX_RESULT_BYTES)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-root", required=True, type=Path)
    parser.add_argument("--ddt-root", required=True, type=Path)
    parser.add_argument("--case-bi4", required=True, type=Path)
    parser.add_argument("--case-xml", required=True, type=Path)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--baseline-qc", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-check", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = build_report(args.baseline_root, args.ddt_root, args.case_bi4, args.case_xml, args.plan, args.baseline_qc)
        if args.self_check:
            print(json.dumps({
                "status": "PASS_U3_DDT_RAMP_QC_V1_SELF_CHECK",
                "report_schema_version": report["schema_version"],
                "candidate_selected": report["verdict"]["candidate_selected"],
                "solver_executed": False,
            }, sort_keys=True))
            return 0
        if args.output is None:
            raise DdtRampQcError("--output is required unless --self-check is used")
        identity = write_exclusive(args.output, report)
        print(json.dumps({"result": identity, "status": report["verdict"]["status"]}, sort_keys=True))
        return 0 if report["verdict"]["candidate_selected"] else 1
    except (DdtRampQcError, qc_v3.QcError, bi4.Bi4FormatError, OSError, UnicodeError, ValueError) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
