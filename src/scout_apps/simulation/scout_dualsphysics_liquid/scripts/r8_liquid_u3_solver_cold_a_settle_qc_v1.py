#!/usr/bin/env python3
"""Fail-closed offline QC for the frozen U3 C1M cold-A settle campaign.

This is deliberately a thin campaign wrapper around the already reviewed v3
BI4/output QC.  It never starts DualSPHysics and never writes into a case or
solver-output tree.  The only write is an explicitly requested, new JSON file
created with O_EXCL and O_NOFOLLOW.

Passing this tool does not admit a settled candidate, physical gauge,
settled-state freeze, restart, U4, or U5.  A cold-A pass is only a single-run
development settle-QC result and permits creation and execution of one fresh
cold-B identity under a separately frozen policy.  Fresh-B, A/B
reproducibility, and restart-equivalence evidence are still required before a
later tool may consider any settled-candidate claim.
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
from pathlib import Path
from typing import Any

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_solver_output_qc_v3 as qc_v3


SCHEMA_VERSION = "r8-liquid-u3-solver-cold-a-settle-qc-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_U3_C1M_COLD_A_SETTLE_QC_V1"
BASE_QC_SCHEMA_VERSION = "r8-liquid-u3-solver-output-qc-v3"

EXPECTED_ROOT_FILES = frozenset(
    {
        "CfgInit_Domain.vtk",
        "CfgInit_MapCells.vtk",
        "Run.csv",
        "Run.out",
        "RunPARTs.csv",
    }
)
EXPECTED_STATIC_DATA_FILES = frozenset(
    {
        "Part_Head.ibi4",
        "PartInfo.ibi4",
        "PartMotionRef.ibi4",
        "PartOut_000.obi4",
    }
)
EXPECTED_PART_FILES = tuple(f"Part_{index:04d}.bi4" for index in range(162))
EXPECTED_DATA_FILES = EXPECTED_STATIC_DATA_FILES | frozenset(EXPECTED_PART_FILES)
EXPECTED_FILE_COUNT = 171
MAX_SINGLE_OUTPUT_BYTES = 16 * 1024 * 1024
MAX_AGGREGATE_OUTPUT_BYTES = 80 * 1024 * 1024
MAX_REPORT_BYTES = 8 * 1024 * 1024
MAX_TEXT_BYTES = 8 * 1024 * 1024


class ColdSettleQcError(qc_v3.QcError):
    """Raised when a cold-settle input or publication target is unsafe."""


@dataclass(frozen=True)
class FrozenColdAThresholds:
    """Pre-run thresholds; none are exposed as command-line overrides."""

    expected_tmax_s: float = 8.05
    expected_tout_s: float = 0.05
    expected_part_file_count: int = 162
    expected_total_output_file_count: int = 171
    tail_window_s: float = 1.0
    minimum_tail_coverage_s: float = 0.95
    physical_time_tolerance_s: float = 1.0e-6

    # Inherited verbatim from solver-output QC v3.  These are development
    # settle criteria, not physical validation tolerances.
    max_tail_speed_rms_m_s: float = 1.0e-3
    max_tail_speed_p95_m_s: float = 1.0e-3
    max_tail_speed_m_s: float = 5.0e-3
    max_tail_specific_kinetic_energy_j_kg: float = 5.0e-7
    max_surface_bias_dp: float = 0.5
    max_surface_spread_dp: float = 0.5
    max_surface_temporal_range_dp: float = 0.25
    max_surface_drift_dp_per_s: float = 0.05
    max_density_mean_relative_range: float = 0.005

    # Position-rate bounds intentionally mirror the independently measured
    # velocity bounds.  They are computed from Posd, aligned by particle ID.
    max_tail_position_rate_rms_m_s: float = 1.0e-3
    max_tail_position_rate_p95_m_s: float = 1.0e-3
    max_tail_position_rate_m_s: float = 5.0e-3

    # Weakly-compressible development checks: mean density stays within 1%
    # of rho0 and the central p01..p99 envelope stays within 5% of rho0.
    max_tail_density_mean_relative_bias: float = 0.01
    max_tail_density_central_relative_deviation: float = 0.05

    surface_sector_count: int = 16
    surface_quantile: float = 0.99
    surface_inner_margin_dp: float = 2.0
    minimum_surface_particles_per_sector: int = 4


FROZEN_THRESHOLDS = FrozenColdAThresholds()


def _base_thresholds() -> qc_v3.Thresholds:
    limits = FROZEN_THRESHOLDS
    return qc_v3.Thresholds(
        minimum_settle_time_s=limits.expected_tmax_s,
        tail_window_s=limits.tail_window_s,
        physical_time_tolerance_s=limits.physical_time_tolerance_s,
        max_tail_speed_rms_m_s=limits.max_tail_speed_rms_m_s,
        max_tail_speed_p95_m_s=limits.max_tail_speed_p95_m_s,
        max_tail_speed_m_s=limits.max_tail_speed_m_s,
        max_tail_specific_kinetic_energy_j_kg=(
            limits.max_tail_specific_kinetic_energy_j_kg
        ),
        max_surface_bias_dp=limits.max_surface_bias_dp,
        max_surface_spread_dp=limits.max_surface_spread_dp,
        max_surface_temporal_range_dp=limits.max_surface_temporal_range_dp,
        max_surface_drift_dp_per_s=limits.max_surface_drift_dp_per_s,
        max_density_mean_relative_range=limits.max_density_mean_relative_range,
        surface_sector_count=limits.surface_sector_count,
        surface_quantile=limits.surface_quantile,
        surface_inner_margin_dp=limits.surface_inner_margin_dp,
        minimum_surface_particles_per_sector=(
            limits.minimum_surface_particles_per_sector
        ),
    )


def _same_float(left: float, right: float, *, tolerance: float = 1.0e-12) -> bool:
    return math.isclose(left, right, rel_tol=tolerance, abs_tol=tolerance)


def _require_real_directory(path: Path, label: str) -> Path:
    try:
        metadata = os.lstat(path)
    except OSError as exc:
        raise ColdSettleQcError(f"cannot inspect {label} directory {path}: {exc}") from exc
    if not stat.S_ISDIR(metadata.st_mode):
        raise ColdSettleQcError(f"{label} is not a real directory: {path}")
    return path


def _scan_exact_inventory(run_dir: str | os.PathLike[str]) -> dict[str, Any]:
    root = _require_real_directory(Path(run_dir), "run")
    data = _require_real_directory(root / "data", "run data")

    root_entries = {entry.name: entry for entry in os.scandir(root)}
    expected_root_entries = EXPECTED_ROOT_FILES | {"data"}
    if set(root_entries) != expected_root_entries:
        missing = sorted(expected_root_entries - set(root_entries))
        extra = sorted(set(root_entries) - expected_root_entries)
        raise ColdSettleQcError(
            f"cold-A root inventory differs; missing={missing}, extra={extra}"
        )
    data_entries = {entry.name: entry for entry in os.scandir(data)}
    if set(data_entries) != EXPECTED_DATA_FILES:
        missing = sorted(EXPECTED_DATA_FILES - set(data_entries))
        extra = sorted(set(data_entries) - EXPECTED_DATA_FILES)
        raise ColdSettleQcError(
            f"cold-A data inventory differs; missing={missing}, extra={extra}"
        )

    records: dict[str, dict[str, Any]] = {}
    aggregate = 0
    for relative, entry in [
        *((name, root_entries[name]) for name in sorted(EXPECTED_ROOT_FILES)),
        *((f"data/{name}", data_entries[name]) for name in sorted(EXPECTED_DATA_FILES)),
    ]:
        try:
            metadata = entry.stat(follow_symlinks=False)
        except OSError as exc:
            raise ColdSettleQcError(f"cannot inspect output {relative}: {exc}") from exc
        if not stat.S_ISREG(metadata.st_mode):
            raise ColdSettleQcError(f"output is not a regular file: {relative}")
        if metadata.st_nlink != 1:
            raise ColdSettleQcError(f"output link count differs from one: {relative}")
        if not 0 < metadata.st_size <= MAX_SINGLE_OUTPUT_BYTES:
            raise ColdSettleQcError(
                f"output size is outside 1..{MAX_SINGLE_OUTPUT_BYTES}: {relative}"
            )
        aggregate += metadata.st_size
        records[relative] = {
            "size_bytes": metadata.st_size,
            "mode": oct(stat.S_IMODE(metadata.st_mode)),
            "uid": metadata.st_uid,
            "gid": metadata.st_gid,
            "nlink": metadata.st_nlink,
        }
    if len(records) != EXPECTED_FILE_COUNT:
        raise ColdSettleQcError(
            f"output file count {len(records)} differs from {EXPECTED_FILE_COUNT}"
        )
    if aggregate > MAX_AGGREGATE_OUTPUT_BYTES:
        raise ColdSettleQcError(
            f"aggregate output size {aggregate} exceeds {MAX_AGGREGATE_OUTPUT_BYTES}"
        )
    return {
        "exact": True,
        "file_count": len(records),
        "aggregate_size_bytes": aggregate,
        "maximum_single_file_bytes": MAX_SINGLE_OUTPUT_BYTES,
        "maximum_aggregate_bytes": MAX_AGGREGATE_OUTPUT_BYTES,
        "files": records,
    }


def _read_support_files(root: Path) -> dict[str, bi4.SecureFile]:
    paths = {
        "CfgInit_Domain.vtk": root / "CfgInit_Domain.vtk",
        "CfgInit_MapCells.vtk": root / "CfgInit_MapCells.vtk",
        "RunPARTs.csv": root / "RunPARTs.csv",
        "data/Part_Head.ibi4": root / "data" / "Part_Head.ibi4",
        "data/PartInfo.ibi4": root / "data" / "PartInfo.ibi4",
    }
    result: dict[str, bi4.SecureFile] = {}
    for relative, path in paths.items():
        try:
            result[relative] = bi4.read_regular_file(
                path, max_bytes=MAX_SINGLE_OUTPUT_BYTES
            )
        except (bi4.Bi4FormatError, ValueError) as exc:
            raise ColdSettleQcError(str(exc)) from exc
    return result


def _decode_utf8(source: bi4.SecureFile, label: str) -> str:
    try:
        return source.data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ColdSettleQcError(f"{label} is not UTF-8") from exc


def _parse_runparts(
    source: bi4.SecureFile,
    parts: list[dict[str, Any]],
    case: dict[str, Any],
    run_csv: dict[str, Any],
) -> dict[str, Any]:
    text = _decode_utf8(source, "RunPARTs.csv")
    lines = [
        line
        for line in text.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if len(lines) < 2:
        raise ColdSettleQcError("RunPARTs.csv lacks data rows")
    rows = list(csv.reader(lines, delimiter=";"))
    header = rows[0]
    if len(set(header)) != len(header):
        raise ColdSettleQcError("RunPARTs.csv contains duplicate columns")
    required = (
        "Part",
        "TimeStep [s]",
        "Steps",
        "NpSave",
        "NpSim",
        "NpOut",
        "NpbSim",
        "NpfSim",
        "NpNormal",
        "NpOutPos",
        "NpOutRho",
        "NpOutMov",
    )
    missing = [name for name in required if name not in header]
    if missing:
        raise ColdSettleQcError(
            f"RunPARTs.csv lacks columns: {', '.join(missing)}"
        )
    parsed_rows: list[dict[str, Any]] = []
    cumulative_steps = 0
    for row_index, row in enumerate(rows[1:]):
        if len(row) != len(header):
            raise ColdSettleQcError(
                f"RunPARTs.csv row {row_index} width differs from header"
            )
        values = dict(zip(header, row))
        parsed = {
            "part": qc_v3._uint_text(values["Part"], "RunPARTs Part"),
            "time_s": qc_v3._finite_float(
                values["TimeStep [s]"], "RunPARTs TimeStep"
            ),
            "steps_in_part": qc_v3._uint_text(
                values["Steps"], "RunPARTs Steps"
            ),
        }
        for name in required[3:]:
            parsed[name] = qc_v3._uint_text(values[name], f"RunPARTs {name}")
        cumulative_steps += parsed["steps_in_part"]
        parsed["cumulative_steps"] = cumulative_steps
        parsed_rows.append(parsed)

    particle_count = case["particle_count"]
    boundary_count = (
        case["counts"]["fixed_boundary"]
        + case["counts"]["moving_boundary"]
        + case["counts"]["floating"]
    )
    fluid_count = case["counts"]["fluid"]
    row_count_matches = len(parsed_rows) == len(parts) == 162
    aligned_pairs = list(zip(parsed_rows, parts))
    checks = {
        "row_count_is_162": row_count_matches,
        "part_sequence_exact": [row["part"] for row in parsed_rows]
        == list(range(162)),
        "times_match_part_bi4": row_count_matches
        and all(
            abs(row["time_s"] - part["time_s"])
            <= FROZEN_THRESHOLDS.physical_time_tolerance_s
            for row, part in aligned_pairs
        ),
        "cumulative_steps_match_part_bi4": row_count_matches
        and all(
            row["cumulative_steps"] == part["step"]
            for row, part in aligned_pairs
        ),
        "particle_counts_match_case": row_count_matches
        and all(
            row["NpSave"] == particle_count
            and row["NpSim"] == particle_count
            and row["NpNormal"] == particle_count
            and row["NpbSim"] == boundary_count
            and row["NpfSim"] == fluid_count
            for row in parsed_rows
        ),
        "all_exclusion_fields_zero": row_count_matches
        and all(
            row["NpOut"]
            == row["NpOutPos"]
            == row["NpOutRho"]
            == row["NpOutMov"]
            == 0
            for row in parsed_rows
        ),
        "total_steps_match_run_csv": row_count_matches
        and run_csv["physics_fields"].get("Steps") == cumulative_steps,
    }
    return {
        "sha256": source.sha256,
        "size_bytes": source.size_bytes,
        "row_count": len(parsed_rows),
        "first_part": parsed_rows[0]["part"] if parsed_rows else None,
        "last_part": parsed_rows[-1]["part"] if parsed_rows else None,
        "first_time_s": parsed_rows[0]["time_s"] if parsed_rows else None,
        "last_time_s": parsed_rows[-1]["time_s"] if parsed_rows else None,
        "total_steps": cumulative_steps,
        "checks": checks,
        "pass": all(checks.values()),
    }


def _extract_effective_time_parameters(source: bi4.SecureFile) -> dict[str, Any]:
    text = _decode_utf8(source, "Run.out")

    def one(name: str) -> float:
        matches = re.findall(
            rf"^{re.escape(name)}\s*=\s*([^\r\n]+?)\s*$", text, re.MULTILINE
        )
        if len(matches) != 1:
            raise ColdSettleQcError(
                f"Run.out must contain exactly one effective {name} assignment"
            )
        return qc_v3._finite_float(matches[0], f"Run.out {name}")

    time_max = one("TimeMax")
    time_part = one("TimePart")
    checks = {
        "effective_tmax_is_8_05_s": _same_float(
            time_max, FROZEN_THRESHOLDS.expected_tmax_s
        ),
        "effective_tout_is_0_05_s": _same_float(
            time_part, FROZEN_THRESHOLDS.expected_tout_s
        ),
    }
    return {
        "source": "effective parameter block in Run.out",
        "time_max_s": time_max,
        "time_out_s": time_part,
        "checks": checks,
        "pass": all(checks.values()),
    }


def _campaign_time_metrics(parts: list[dict[str, Any]]) -> dict[str, Any]:
    times = [part["time_s"] for part in parts]
    expected = [
        index * FROZEN_THRESHOLDS.expected_tout_s for index in range(162)
    ]
    tolerance = FROZEN_THRESHOLDS.physical_time_tolerance_s
    tout = FROZEN_THRESHOLDS.expected_tout_s
    slot_membership = len(times) == len(expected) and all(
        scheduled - tolerance <= observed < scheduled + tout
        for observed, scheduled in zip(times, expected)
    )
    overshoots = [observed - scheduled for observed, scheduled in zip(times, expected)]
    final_time = times[-1] if times else None
    checks = {
        "part_count_is_162": len(parts) == 162,
        "cpart_sequence_is_0000_through_0161": [part["cpart"] for part in parts]
        == list(range(162)),
        "each_time_is_in_its_0_05_s_output_slot": slot_membership,
        "final_time_is_8_05_to_before_8_10_s": final_time is not None
        and FROZEN_THRESHOLDS.expected_tmax_s - tolerance <= final_time
        < FROZEN_THRESHOLDS.expected_tmax_s + tout,
    }
    return {
        "first_time_s": times[0] if times else None,
        "final_time_s": final_time,
        "maximum_schedule_overshoot_s": max(overshoots) if overshoots else None,
        "minimum_schedule_overshoot_s": min(overshoots) if overshoots else None,
        "checks": checks,
        "pass": all(checks.values()),
    }


def _tail_parts(parts: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not parts:
        return []
    cutoff = parts[-1]["time_s"] - FROZEN_THRESHOLDS.tail_window_s
    return [part for part in parts if part["time_s"] >= cutoff - 1.0e-12]


def _density_tail_metrics(
    parts: list[dict[str, Any]], case: dict[str, Any]
) -> dict[str, Any]:
    tail = _tail_parts(parts)
    rho0 = case["xml"]["rhop0_kg_m3"]
    means = [part["density"]["mean_kg_m3"] for part in tail]
    p01s = [part["density"]["p01_kg_m3"] for part in tail]
    p99s = [part["density"]["p99_kg_m3"] for part in tail]
    complete = bool(tail) and all(
        value is not None and math.isfinite(value)
        for value in [*means, *p01s, *p99s]
    )
    mean_bias = (
        max(abs(value - rho0) / rho0 for value in means) if complete else None
    )
    central_deviation = (
        max(
            max(abs(low - rho0), abs(high - rho0)) / rho0
            for low, high in zip(p01s, p99s)
        )
        if complete
        else None
    )
    checks = {
        "density_tail_metrics_available": complete,
        "density_mean_relative_bias_below_1_percent": mean_bias is not None
        and mean_bias
        <= FROZEN_THRESHOLDS.max_tail_density_mean_relative_bias,
        "density_p01_p99_relative_deviation_below_5_percent": (
            central_deviation is not None
            and central_deviation
            <= FROZEN_THRESHOLDS.max_tail_density_central_relative_deviation
        ),
    }
    return {
        "tail_part_count": len(tail),
        "maximum_mean_relative_bias": mean_bias,
        "maximum_p01_p99_relative_deviation": central_deviation,
        "checks": checks,
        "pass": all(checks.values()),
    }


def _position_tail_metrics(
    run_root: Path,
    parts: list[dict[str, Any]],
    case: dict[str, Any],
    base_thresholds: qc_v3.Thresholds,
) -> dict[str, Any]:
    tail = _tail_parts(parts)
    fluid_begin = (
        case["counts"]["fixed_boundary"]
        + case["counts"]["moving_boundary"]
        + case["counts"]["floating"]
    )
    fluid_ids = list(range(fluid_begin, case["particle_count"]))
    snapshots: list[tuple[float, dict[int, tuple[float, float, float]]]] = []
    complete = len(tail) >= 2 and bool(fluid_ids)
    for summary in tail:
        index = summary["cpart"]
        source = qc_v3._read(
            run_root / "data" / f"Part_{index:04d}.bi4",
            maximum=MAX_SINGLE_OUTPUT_BYTES,
        )
        parsed = qc_v3.parse_part_file(source, index, case, base_thresholds)
        positions = parsed["_positions_by_id"]
        if any(value not in positions for value in fluid_ids):
            complete = False
            continue
        selected = {value: positions[value] for value in fluid_ids}
        if any(
            not all(math.isfinite(component) for component in vector)
            for vector in selected.values()
        ):
            complete = False
        snapshots.append((parsed["time_s"], selected))

    step_rms_rates: list[float] = []
    step_p95_rates: list[float] = []
    step_max_rates: list[float] = []
    if complete and len(snapshots) == len(tail):
        for (left_time, left), (right_time, right) in zip(
            snapshots, snapshots[1:]
        ):
            elapsed = right_time - left_time
            if elapsed <= 0:
                complete = False
                break
            rates = [
                math.sqrt(
                    sum(
                        (right[value][axis] - left[value][axis]) ** 2
                        for axis in range(3)
                    )
                )
                / elapsed
                for value in fluid_ids
            ]
            step_rms_rates.append(
                math.sqrt(sum(value * value for value in rates) / len(rates))
            )
            step_p95_rates.append(qc_v3._quantile(rates, 0.95))
            step_max_rates.append(max(rates))

    net_rms_rate = net_p95_rate = net_max_rate = None
    if complete and snapshots:
        elapsed = snapshots[-1][0] - snapshots[0][0]
        if elapsed <= 0:
            complete = False
        else:
            first, last = snapshots[0][1], snapshots[-1][1]
            net_rates = [
                math.sqrt(
                    sum(
                        (last[value][axis] - first[value][axis]) ** 2
                        for axis in range(3)
                    )
                )
                / elapsed
                for value in fluid_ids
            ]
            net_rms_rate = math.sqrt(
                sum(value * value for value in net_rates) / len(net_rates)
            )
            net_p95_rate = qc_v3._quantile(net_rates, 0.95)
            net_max_rate = max(net_rates)

    max_step_rms = max(step_rms_rates) if complete and step_rms_rates else None
    max_step_p95 = max(step_p95_rates) if complete and step_p95_rates else None
    max_step = max(step_max_rates) if complete and step_max_rates else None
    checks = {
        "tail_position_data_available_by_particle_id": complete,
        "interframe_position_rate_rms_below_threshold": max_step_rms is not None
        and max_step_rms <= FROZEN_THRESHOLDS.max_tail_position_rate_rms_m_s,
        "interframe_position_rate_p95_below_threshold": max_step_p95 is not None
        and max_step_p95 <= FROZEN_THRESHOLDS.max_tail_position_rate_p95_m_s,
        "interframe_position_rate_max_below_threshold": max_step is not None
        and max_step <= FROZEN_THRESHOLDS.max_tail_position_rate_m_s,
        "net_tail_position_rate_rms_below_threshold": net_rms_rate is not None
        and net_rms_rate <= FROZEN_THRESHOLDS.max_tail_position_rate_rms_m_s,
        "net_tail_position_rate_p95_below_threshold": net_p95_rate is not None
        and net_p95_rate <= FROZEN_THRESHOLDS.max_tail_position_rate_p95_m_s,
        "net_tail_position_rate_max_below_threshold": net_max_rate is not None
        and net_max_rate <= FROZEN_THRESHOLDS.max_tail_position_rate_m_s,
    }
    return {
        "method": "fluid_Posd_displacement_rate_aligned_by_Idp",
        "tail_part_count": len(tail),
        "fluid_particle_count": len(fluid_ids),
        "maximum_interframe_rms_rate_m_s": max_step_rms,
        "maximum_interframe_p95_rate_m_s": max_step_p95,
        "maximum_interframe_rate_m_s": max_step,
        "net_tail_rms_rate_m_s": net_rms_rate,
        "net_tail_p95_rate_m_s": net_p95_rate,
        "net_tail_max_rate_m_s": net_max_rate,
        "checks": checks,
        "pass": all(checks.values()),
    }


def _artifact_hashes(
    run: dict[str, Any], support: dict[str, bi4.SecureFile]
) -> dict[str, str]:
    hashes = {relative: source.sha256 for relative, source in support.items()}
    hashes["Run.csv"] = run["run_csv"]["sha256"]
    hashes["Run.out"] = run["run_out"]["sha256"]
    hashes.update(
        {
            f"data/{name}": digest
            for name, digest in run["inventory"]["part_sha256"].items()
        }
    )
    hashes.update(
        {
            f"data/{name}": digest
            for name, digest in run["inventory"]["partout_sha256"].items()
        }
    )
    hashes["data/PartMotionRef.ibi4"] = run["inventory"][
        "motion_reference_sha256"
    ]
    expected = EXPECTED_ROOT_FILES | {
        f"data/{name}" for name in EXPECTED_DATA_FILES
    }
    if set(hashes) != expected or len(hashes) != EXPECTED_FILE_COUNT:
        raise ColdSettleQcError("semantic readers did not hash the exact output inventory")
    return dict(sorted(hashes.items()))


def build_report(
    run_dir: str | os.PathLike[str],
    case_bi4: str | os.PathLike[str],
    case_xml: str | os.PathLike[str],
    *,
    expected_case_bi4_sha256: str | None = None,
    expected_case_xml_sha256: str | None = None,
) -> dict[str, Any]:
    """Audit one completed cold-A tree without writing any files."""

    inventory = _scan_exact_inventory(run_dir)
    root = Path(run_dir)
    support = _read_support_files(root)
    base_thresholds = _base_thresholds()
    case = qc_v3.load_moving_case(
        case_bi4,
        case_xml,
        expected_bi4_sha256=expected_case_bi4_sha256,
        expected_xml_sha256=expected_case_xml_sha256,
    )
    run = qc_v3.analyze_run(root, case, base_thresholds)
    runparts = _parse_runparts(
        support["RunPARTs.csv"], run["parts"], case, run["run_csv"]
    )
    run_out_source = qc_v3._read(root / "Run.out", maximum=MAX_TEXT_BYTES)
    effective = _extract_effective_time_parameters(run_out_source)
    campaign_time = _campaign_time_metrics(run["parts"])
    density = _density_tail_metrics(run["parts"], case)
    position = _position_tail_metrics(root, run["parts"], case, base_thresholds)
    hashes = _artifact_hashes(run, support)
    for relative, digest in hashes.items():
        inventory["files"][relative]["sha256"] = digest

    csv_fields = run["run_csv"]["physics_fields"]
    campaign_checks = {
        "exact_171_file_inventory": inventory["exact"]
        and inventory["file_count"] == EXPECTED_FILE_COUNT,
        "all_171_files_hashed": len(hashes) == EXPECTED_FILE_COUNT,
        "effective_time_parameters_match": effective["pass"],
        "case_xml_tout_matches_campaign": _same_float(
            case["xml"]["time_out_xml_s"], FROZEN_THRESHOLDS.expected_tout_s
        ),
        "part_sequence_and_times_match_campaign": campaign_time["pass"],
        "run_csv_partfiles_is_162": csv_fields["PartFiles"] == 162,
        "runparts_contract_pass": runparts["pass"],
        "motion_reference_has_162_parts": run["motion_reference"]["part_count"]
        == 162,
        "only_partout_000_exists_and_is_empty": run["inventory"][
            "partout_file_count"
        ]
        == 1
        and run["partout"][0]["name"] == "PartOut_000.obi4"
        and run["partout"][0]["total_nout"] == 0,
    }
    settle_checks = {
        "v3_structural_checks_pass": run["structural_pass"],
        "v3_duration_and_tail_checks_pass": run["numeric_settle_qc_pass"],
        "tail_coverage_at_least_0_95_s": run["tail_metrics"]["coverage_s"]
        >= FROZEN_THRESHOLDS.minimum_tail_coverage_s,
        "density_development_checks_pass": density["pass"],
        "position_development_checks_pass": position["pass"],
    }
    cold_a_pass = all(campaign_checks.values()) and all(settle_checks.values())
    status = (
        "PASS_U3_COLD_A_DEVELOPMENT_SETTLE_QC_COLD_B_REQUIRED"
        if cold_a_pass
        else "FAIL_U3_COLD_A_DEVELOPMENT_SETTLE_QC"
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "base_qc_schema_version": BASE_QC_SCHEMA_VERSION,
        "campaign": {
            "identity": "cold_a",
            "expected_tmax_s": FROZEN_THRESHOLDS.expected_tmax_s,
            "expected_tout_s": FROZEN_THRESHOLDS.expected_tout_s,
            "expected_part_range": "Part_0000.bi4..Part_0161.bi4",
            "expected_part_file_count": 162,
            "expected_total_output_file_count": EXPECTED_FILE_COUNT,
            "execution_mode": "CPU_ONLY_SINGLE_CORE",
        },
        "thresholds": asdict(FROZEN_THRESHOLDS),
        "threshold_freeze": {
            "frozen_before_cold_a_output_exists": True,
            "command_line_overrides_allowed": False,
            "speed_and_surface_source": "inherited verbatim from solver-output QC v3",
            "position_source": (
                "Posd displacement rates aligned by Idp, bounded by the same "
                "pre-run limits as velocity"
            ),
            "density_source": (
                "weakly-compressible development screen: mean within 1% and "
                "p01/p99 within 5% of rho0"
            ),
        },
        "moving_case": {
            "bi4": case["bi4"],
            "xml_sha256": case["xml"]["sha256"],
            "particle_count": case["particle_count"],
            "counts": case["counts"],
            "dp_m": case["dp_m"],
            "rhop0_kg_m3": case["xml"]["rhop0_kg_m3"],
            "requested_massfluid_kg": case["massfluid_kg"],
            "solver_effective_massfluid_kg": case[
                "solver_effective_massfluid_kg"
            ],
            "xml_time_max_s": case["xml"]["time_max_xml_s"],
            "xml_time_out_s": case["xml"]["time_out_xml_s"],
        },
        "inventory": inventory,
        "effective_solver_time_parameters": effective,
        "campaign_time": campaign_time,
        "runparts": runparts,
        "density_tail": density,
        "position_tail": position,
        "base_qc_v3_run": run,
        "checks": {
            "campaign": campaign_checks,
            "settle": settle_checks,
        },
        "verdict": {
            "status": status,
            "decision": (
                "COLD_A_PASS_COLD_B_ONLY"
                if cold_a_pass
                else "STOP_NO_DOWNSTREAM_EXECUTION"
            ),
            "cold_a_pass": cold_a_pass,
            "claim_ceiling": "COLD_A_ONLY_NOT_SETTLED",
            "development_only": True,
            "formal": False,
            "fidelity_validation_status": "SIM_ONLY_UNVALIDATED",
            "cold_b_execution_eligible": cold_a_pass,
            "restart_execution_eligible": False,
            "settled_state_claim_allowed": False,
            "settled_state_freeze_eligible": False,
            "u4_authorized": False,
            "u5_authorized": False,
            "physical_primary_eligible": False,
            "limitations": [
                "cold A is one process identity; an independently fresh cold B is required",
                "particle q99 surface is a development proxy, not a DualSPHysics Gauge/SWL admission",
                "Part_Head, PartInfo and VTK support files are hash/inventory audited but not semantically decoded by the v1 reader",
                "no fresh-clone restart-equivalence comparison is performed",
                "no physical vessel, fluid, sensor, or Ubuntu 20.04 bag validation is performed",
            ],
        },
    }


def _serialize_report(report: dict[str, Any]) -> bytes:
    try:
        payload = (
            json.dumps(
                report,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ColdSettleQcError(f"report is not finite JSON: {exc}") from exc
    if not 0 < len(payload) <= MAX_REPORT_BYTES:
        raise ColdSettleQcError(
            f"report size {len(payload)} is outside 1..{MAX_REPORT_BYTES}"
        )
    return payload


def write_json_exclusive(
    output_path: str | os.PathLike[str], report: dict[str, Any]
) -> dict[str, Any]:
    """Publish a bounded JSON report without replacing any existing inode."""

    path = Path(output_path)
    if path.name in {"", ".", ".."}:
        raise ColdSettleQcError("output JSON must have a regular basename")
    parent = _require_real_directory(path.parent, "output parent")
    payload = _serialize_report(report)
    directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        directory_fd = os.open(parent, directory_flags)
    except OSError as exc:
        raise ColdSettleQcError(f"cannot securely open output parent {parent}: {exc}") from exc
    descriptor = -1
    try:
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        try:
            descriptor = os.open(path.name, flags, 0o400, dir_fd=directory_fd)
        except OSError as exc:
            raise ColdSettleQcError(
                f"cannot create exclusive output JSON {path}: {exc}"
            ) from exc
        offset = 0
        while offset < len(payload):
            written = os.write(descriptor, payload[offset:])
            if written <= 0:
                raise ColdSettleQcError(f"short O_EXCL report write: {path}")
            offset += written
        os.fsync(descriptor)
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or metadata.st_size != len(payload)
            or stat.S_IMODE(metadata.st_mode) != 0o400
        ):
            raise ColdSettleQcError(f"exclusive report inode contract differs: {path}")
        os.fsync(directory_fd)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        os.close(directory_fd)
    return {
        "path": str(path),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "size_bytes": len(payload),
        "mode": "0o400",
        "creation": "O_EXCL_O_NOFOLLOW_DIRFD_ANCHORED",
    }


def _validate_output_is_outside_inputs(
    output_path: str | os.PathLike[str],
    run_dir: str | os.PathLike[str],
    case_bi4: str | os.PathLike[str],
    case_xml: str | os.PathLike[str],
) -> None:
    output = Path(output_path).absolute()
    run = Path(run_dir).absolute()
    try:
        output.relative_to(run)
    except ValueError:
        pass
    else:
        raise ColdSettleQcError("output JSON must not be written inside the run tree")
    if output in {Path(case_bi4).absolute(), Path(case_xml).absolute()}:
        raise ColdSettleQcError("output JSON must not replace a case input")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--case-bi4", required=True)
    parser.add_argument("--case-xml", required=True)
    parser.add_argument("--expected-case-bi4-sha256", required=True)
    parser.add_argument("--expected-case-xml-sha256", required=True)
    parser.add_argument("--output-json", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    exit_code = 0
    try:
        _validate_output_is_outside_inputs(
            args.output_json, args.run_dir, args.case_bi4, args.case_xml
        )
        report = build_report(
            args.run_dir,
            args.case_bi4,
            args.case_xml,
            expected_case_bi4_sha256=args.expected_case_bi4_sha256,
            expected_case_xml_sha256=args.expected_case_xml_sha256,
        )
        if not report["verdict"]["cold_a_pass"]:
            exit_code = 1
    except (ColdSettleQcError, qc_v3.QcError) as exc:
        report = {
            "schema_version": SCHEMA_VERSION,
            "document_type": DOCUMENT_TYPE,
            "status": "ERROR_UNSAFE_OR_INVALID_INPUT",
            "error": str(exc),
            "verdict": {
                "cold_a_pass": False,
                "decision": "STOP_NO_DOWNSTREAM_EXECUTION",
                "claim_ceiling": "NONE",
                "cold_b_execution_eligible": False,
                "restart_execution_eligible": False,
                "settled_state_freeze_eligible": False,
                "u4_authorized": False,
            },
        }
        exit_code = 2
    try:
        publication = write_json_exclusive(args.output_json, report)
    except ColdSettleQcError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps({"status": report.get("verdict", {}).get("status", report.get("status")), "publication": publication}, ensure_ascii=False, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
