#!/usr/bin/env python3
"""Fail-closed paired QC and final phase-4 adjudication for CFL 0.2 vs 0.1.

This tool is read-only with respect to both solver output trees.  It verifies
the two exact GPU runs, recomputes the frozen 17-metric settling vector, pairs
particle arrays by Id, and can publish one create-new JSON adjudication.
It never starts DualSPHysics, exposes a GPU, or modifies an input.
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
import struct
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_ddt_ramp_qc_v1 as base
import r8_liquid_u3_extended_settling_qc_v1 as extended
import r8_liquid_u3_solver_output_qc_v3 as qc_v3


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_cfl0p1_sensitivity_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_cfl0p1_sensitivity_qc_v1.py"
SCHEMA_VERSION = "r8-liquid-u3-cfl0p1-sensitivity-qc-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_U3_CFL0P1_SENSITIVITY_QC_V1"

PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"
CONTRACT_SHA256 = "303e0634d0bc2844cf26d2799bb2448266263fafb34a2ba19f4116c35f2a1dec"
BASELINE_POLICY_SHA256 = "a1c6642db2cb7cb586ff25c24f80911ba72ad54f7634d37817b82aa350caf2f4"
CANDIDATE_POLICY_SHA256 = "c803e62471b4bc16dcb31c861382d3a245f8b8de8fc9b56540c48fdfe35f6c9d"
BASELINE_FINAL_SHA256 = "c7f26f9ba2b742718113094b0ef7c24ce742d0063acaa4918c9083810a3143b5"
CANDIDATE_FINAL_SHA256 = "5ebf713fd4f507073fd18fc479a3d03b19e6420658a190d681c76729651f0a16"
BASELINE_QC_SHA256 = "2487ace02470ae7e85338450611648185490e8df0457f015ea091665f0e09e56"
BASELINE_INVENTORY_SHA256 = "d6b70b50b348b8a234b6022c2972bd7bfc6e3892fc7a9c6c8eb9bcb5b808beb5"
CANDIDATE_INVENTORY_SHA256 = "4606d4ab2968fc2608751e7aaa1cd64dc7eb387c52533e3531b940712ee7c918"
BASELINE_RUN_OUT_SHA256 = "e35bb1305bd98460d77660a5502805ed425a02d87303f5ccca7a67efad3fde04"
CANDIDATE_RUN_OUT_SHA256 = "01821b42e210ab7eac24aebd7909bf2cacf70fb82f183b5f541320d501821a45"
BASELINE_RUN_CSV_SHA256 = "a802809c49bc1aa8351d3a0055f9f1a51385fdf07cb465178673e57630daa14d"
CANDIDATE_RUN_CSV_SHA256 = "9dcf433db2dcc90675d37922d25f3ccd24aa29cd8fa56af26654f4ae8cfe7c2b"
CANDIDATE_BINARY_SHA256 = "cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f"
GPU_UUID = "GPU-d29cd22d-8b25-8f4e-9dcf-f2fc4bb113fb"
GPU_NAME = "NVIDIA GeForce RTX 5080 Laptop GPU"
BASELINE_RUN_ID = "u3_c1m_gpu_shift_none_extend_from_0201_v3"
CANDIDATE_RUN_ID = "u3_c1m_gpu_shift_none_cfl0p1_from_0201_v4"
PART_INDICES = tuple(range(201, 402))
ROOT_FILES = extended.ROOT_FILES
STATIC_DATA_FILES = extended.STATIC_DATA_FILES
MAX_FILE_BYTES = 64 * 1024 * 1024
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_TREE_BYTES = 128 * 1024 * 1024
EXPECTED_FILE_COUNT = 210


class CflSensitivityQcError(ValueError):
    """Unsafe input, identity drift, or structurally invalid paired run."""


def _safe_json(path: Path, expected_sha256: str) -> tuple[dict[str, Any], dict[str, Any]]:
    identity = base._identity(path, maximum=MAX_JSON_BYTES, expected_sha256=expected_sha256)
    source = bi4.read_regular_file(path, max_bytes=MAX_JSON_BYTES)
    try:
        value = json.loads(source.data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise CflSensitivityQcError(f"invalid JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise CflSensitivityQcError(f"JSON root is not object: {path}")
    return value, identity


def _inventory(root: Path, expected_sha256: str, receipt: dict[str, Any]) -> dict[str, Any]:
    try:
        root_stat = os.lstat(root)
        data_stat = os.lstat(root / "data")
    except OSError as exc:
        raise CflSensitivityQcError(f"cannot inspect output tree: {exc}") from exc
    if not stat.S_ISDIR(root_stat.st_mode) or not stat.S_ISDIR(data_stat.st_mode):
        raise CflSensitivityQcError("run root/data is not a real directory")
    expected_root = set(ROOT_FILES) | {"data"}
    expected_data = set(STATIC_DATA_FILES) | {f"Part_{index:04d}.bi4" for index in PART_INDICES}
    if {entry.name for entry in os.scandir(root)} != expected_root:
        raise CflSensitivityQcError("run root inventory differs")
    if {entry.name for entry in os.scandir(root / "data")} != expected_data:
        raise CflSensitivityQcError("run data inventory differs")
    relatives = [
        *ROOT_FILES,
        *(f"data/{name}" for name in (*STATIC_DATA_FILES, *(f"Part_{index:04d}.bi4" for index in PART_INDICES))),
    ]
    records: dict[str, dict[str, Any]] = {}
    total = 0
    for relative in relatives:
        item = base._identity(root / relative, maximum=MAX_FILE_BYTES)
        records[relative] = {key: item[key] for key in ("size_bytes", "sha256", "mode", "nlink")}
        total += item["size_bytes"]
    canonical = base._canonical_sha256(records)
    if len(records) != EXPECTED_FILE_COUNT or total > MAX_TREE_BYTES or canonical != expected_sha256:
        raise CflSensitivityQcError("run inventory count, size, or canonical hash differs")
    frozen = receipt.get("output_inventory")
    if not isinstance(frozen, dict) or frozen.get("files") != records:
        raise CflSensitivityQcError("run inventory records differ from final receipt")
    if (frozen.get("file_count"), frozen.get("total_bytes"), frozen.get("canonical_sha256")) != (
        len(records), total, canonical,
    ):
        raise CflSensitivityQcError("run inventory summary differs from final receipt")
    return {"file_count": len(records), "total_bytes": total, "canonical_sha256": canonical}


def _receipt(
    path: Path,
    expected_sha256: str,
    expected_document_type: str,
    expected_run_id: str,
    expected_inventory_sha256: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    value, identity = _safe_json(path, expected_sha256)
    candidate = value.get("candidate_source_after", {})
    gpu = value.get("gpu_after", {})
    required = (
        value.get("document_type") == expected_document_type,
        value.get("status") == "PASS_GPU_STAGE4_RUN_RAW_OUTPUT",
        value.get("run_id") == expected_run_id,
        value.get("returncode") == 0,
        value.get("termination_reason") == "PROCESS_EXIT",
        value.get("candidate_source_executed") is False,
        value.get("staged_candidate_executed") is True,
        value.get("network_used") is False,
        value.get("apparmor_profile_used") is False,
        value.get("compute_processes_after") == [],
        value.get("output_inventory", {}).get("canonical_sha256") == expected_inventory_sha256,
        candidate.get("sha256") == CANDIDATE_BINARY_SHA256,
        candidate.get("mode") == "0400",
        candidate.get("nlink") == 1,
        gpu.get("uuid") == GPU_UUID,
        gpu.get("name") == GPU_NAME,
    )
    if not all(required):
        raise CflSensitivityQcError(f"final receipt semantics differ: {path}")
    return value, identity


def _state_by_id(path: Path) -> tuple[dict[str, Any], dict[int, tuple[Any, Any, float]]]:
    source = bi4.read_regular_file(path, max_bytes=MAX_FILE_BYTES)
    parsed = bi4.parse_jpartdata_bi4(source.data)
    _, ids, positions, velocities, densities = qc_v3._part_arrays(parsed)
    if len(ids) != 9078 or len(set(ids)) != len(ids):
        raise CflSensitivityQcError(f"particle Id set differs: {path}")
    values = {
        particle_id: (positions[index], velocities[index], densities[index])
        for index, particle_id in enumerate(ids)
    }
    digest = hashlib.sha256()
    for particle_id in sorted(values):
        position, velocity, density = values[particle_id]
        digest.update(struct.pack(">Q7d", particle_id, *position, *velocity, density))
    return {
        "path": str(path),
        "part_file_sha256": source.sha256,
        "particle_count": len(ids),
        "canonical_by_id_array_sha256": digest.hexdigest(),
    }, values


def _single_float(text: str, name: str) -> float:
    matches = re.findall(rf"^{re.escape(name)}=([^\s]+)\s*$", text, flags=re.MULTILINE)
    if len(matches) != 1:
        raise CflSensitivityQcError(f"unexpected {name} cardinality")
    value = float(matches[0])
    if not math.isfinite(value):
        raise CflSensitivityQcError(f"non-finite {name}")
    return value


def _single_quoted(text: str, name: str) -> str:
    matches = re.findall(rf'^{re.escape(name)}="([^"]+)"\s*$', text, flags=re.MULTILINE)
    if len(matches) != 1:
        raise CflSensitivityQcError(f"unexpected {name} cardinality")
    return matches[0]


def _run_csv_semantics(source: bi4.SecureFile) -> dict[str, Any]:
    try:
        rows = list(csv.reader(source.data.decode("utf-8").splitlines(), delimiter=";"))
    except UnicodeError as exc:
        raise CflSensitivityQcError("Run.csv is not UTF-8") from exc
    if len(rows) != 2 or len(rows[0]) != len(rows[1]) or len(set(rows[0])) != len(rows[0]):
        raise CflSensitivityQcError("Run.csv structure differs")
    values = dict(zip(rows[0], rows[1]))
    for name in ("Configuration", "Hardware", "RunMode", "Dp"):
        if not values.get(name):
            raise CflSensitivityQcError(f"Run.csv field missing: {name}")
    return {
        "configuration": values["Configuration"],
        "hardware": values["Hardware"],
        "run_mode": values["RunMode"],
        "dp_m": float(values["Dp"]),
    }


def _analyze_run(
    root: Path,
    case: dict[str, Any],
    prior_checkpoint: Path,
    receipt: dict[str, Any],
    expected_inventory_sha256: str,
    expected_run_out_sha256: str,
    expected_run_csv_sha256: str,
    expected_cfl: float,
) -> dict[str, Any]:
    inventory = _inventory(root, expected_inventory_sha256, receipt)
    thresholds = base._thresholds()
    parts = [
        qc_v3.parse_part_file(
            qc_v3._read(root / "data" / f"Part_{index:04d}.bi4", maximum=MAX_FILE_BYTES),
            index,
            case,
            thresholds,
        )
        for index in PART_INDICES
    ]
    run_out_source = qc_v3._read(root / "Run.out", maximum=MAX_FILE_BYTES, expected_sha256=expected_run_out_sha256)
    run_csv_source = qc_v3._read(root / "Run.csv", maximum=MAX_FILE_BYTES, expected_sha256=expected_run_csv_sha256)
    run_out = qc_v3.parse_run_out(run_out_source)
    run_csv = qc_v3.parse_run_csv(run_csv_source)
    csv_semantics = _run_csv_semantics(run_csv_source)
    try:
        run_text = run_out_source.data.decode("utf-8")
    except UnicodeError as exc:
        raise CflSensitivityQcError("Run.out is not UTF-8") from exc
    cfl = _single_float(run_text, "CFLnumber")
    shifting = _single_quoted(run_text, "Shifting")
    run_mode = _single_quoted(run_text, "RunMode")
    devices = re.findall(r'^Device selected: ([0-9]+) "([^"]+)"\s*$', run_text, flags=re.MULTILINE)
    if len(devices) != 1:
        raise CflSensitivityQcError("selected GPU identity cardinality differs")
    if not math.isclose(cfl, expected_cfl, rel_tol=0.0, abs_tol=1.0e-12):
        raise CflSensitivityQcError(f"actual CFL differs: {cfl}")
    if shifting != "None" or devices[0] != ("0", GPU_NAME):
        raise CflSensitivityQcError("Shifting or selected GPU differs")
    ddt = base._parse_ddt_parameters(run_out_source, candidate=False)
    partout = qc_v3.parse_partout(qc_v3._read(root / "data/PartOut_000.obi4", maximum=MAX_FILE_BYTES))
    motion = base._parse_continuation_motion_ref(
        qc_v3._read(root / "data/PartMotionRef.ibi4", maximum=MAX_FILE_BYTES), parts, case, thresholds,
    )
    runparts = base._parse_runparts(qc_v3._read(root / "RunPARTs.csv", maximum=MAX_FILE_BYTES), parts)
    times = [item["time_s"] for item in parts]
    steps = [item["step"] for item in parts]
    fields = run_csv["physics_fields"]
    structural = {
        "exact_inventory": inventory["file_count"] == EXPECTED_FILE_COUNT,
        "exact_part_range": [item["cpart"] for item in parts] == list(PART_INDICES),
        "strictly_increasing_times": all(right > left for left, right in zip(times, times[1:])),
        "monotonic_steps": all(right >= left for left, right in zip(steps, steps[1:])),
        "start_time_matches": abs(times[0] - 10.05) <= base.CONTRACT.physical_time_tolerance_s,
        "end_time_matches": abs(times[-1] - 20.05) <= base.CONTRACT.physical_time_tolerance_s,
        "all_particle_arrays_finite": all(all(item["finite"].values()) for item in parts),
        "all_ids_complete_unique_in_range": all(
            item["ids_unique"] and item["ids_in_range"] and item["missing_id_count"] == 0 for item in parts
        ),
        "particle_classes_constant": all(
            item["counts"]["moving_boundary"] == 2669
            and item["counts"]["fluid"] == 6409
            and item["npok"] == 9078
            for item in parts
        ),
        "root_invariants_match_case": all(item["root_case_invariants_match"] for item in parts),
        "moving_boundary_zero_motion": all(
            item["moving_positions_exact"] and item["moving_velocities_zero"] for item in parts
        ),
        "zero_nout": (
            all(item["nout"] == 0 for item in parts)
            and partout["total_nout"] == 0
            and fields["PartsOut"] == 0
            and run_out["excluded_particles"] == 0
        ),
        "finite_inside_domain_density": all(
            item["domain_outside_count"] == 0 and item["density_outside_count"] == 0 for item in parts
        ),
        "motion_reference_zero_aligned": (
            motion["all_posref_finite"]
            and motion["all_posref_unchanged"]
            and motion["aligned_with_particle_parts"]
            and motion["part_count"] == len(parts)
        ),
        "runparts_matches": runparts["pass"],
        "run_csv_matches": (
            fields["Np"] == 9078
            and fields["PartFiles"] == 402
            and abs(fields["PhysicalTime"] - times[-1]) <= base.CONTRACT.physical_time_tolerance_s
        ),
        "run_out_clean_complete": run_out["completion_markers_pass"] and not any(run_out["bad_patterns"].values()),
    }
    if not all(structural.values()):
        raise CflSensitivityQcError(
            f"structural checks failed for {root}: {[name for name, passed in structural.items() if not passed]}"
        )
    restart_input, _ = _state_by_id(prior_checkpoint)
    restart_output, _ = _state_by_id(root / "data/Part_0201.bi4")
    restart_matches = restart_input["canonical_by_id_array_sha256"] == restart_output["canonical_by_id_array_sha256"]
    if not restart_matches:
        raise CflSensitivityQcError(f"restart state arrays differ for {root}")
    metrics, tail = base._metric_vector(parts, case)
    metric_pass = {name: metrics[name] <= base.METRIC_LIMITS[name] for name in sorted(metrics)}
    trajectory = extended._trajectory(parts)
    for part in parts:
        part.pop("_positions_by_id", None)
    return {
        "root": str(root),
        "inventory": inventory,
        "frame_count": len(parts),
        "first_time_s": times[0],
        "last_time_s": times[-1],
        "run_out_sha256": run_out["sha256"],
        "run_csv_sha256": run_csv["sha256"],
        "actual_cfl": cfl,
        "shifting": shifting,
        "ddt_parameters": ddt,
        "backend": {
            "device_index": int(devices[0][0]),
            "device_name": devices[0][1],
            "gpu_uuid": receipt["gpu_after"]["uuid"],
            "run_mode": run_mode,
            "run_csv_hardware": csv_semantics["hardware"],
            "run_csv_run_mode": csv_semantics["run_mode"],
        },
        "run_configuration": csv_semantics["configuration"],
        "dp_m": csv_semantics["dp_m"],
        "restart_input_state": restart_input,
        "restart_output_state": restart_output,
        "restart_state_arrays_match": restart_matches,
        "structural_checks": structural,
        "tail": tail,
        "metrics": metrics,
        "metric_absolute_pass": metric_pass,
        "trajectory": trajectory,
    }


def _distance_stats(values: list[float]) -> dict[str, float]:
    if not values or any(not math.isfinite(value) or value < 0 for value in values):
        raise CflSensitivityQcError("paired distance values are invalid")
    ordered = sorted(values)
    return {
        "rms": math.sqrt(sum(value * value for value in ordered) / len(ordered)),
        "p95": ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)],
        "max": ordered[-1],
    }


def _paired_state_comparison(baseline_root: Path, candidate_root: Path, case: dict[str, Any]) -> dict[str, Any]:
    fluid_first = case["counts"]["fixed_boundary"] + case["counts"]["moving_boundary"] + case["counts"]["floating"]
    fluid_ids = tuple(range(fluid_first, case["particle_count"]))
    summaries: list[dict[str, Any]] = []
    maximums: dict[str, dict[str, dict[str, Any]]] = {
        statistic: {} for statistic in ("rms", "p95", "max")
    }
    for part_index in PART_INDICES:
        baseline_identity, baseline = _state_by_id(baseline_root / "data" / f"Part_{part_index:04d}.bi4")
        candidate_identity, candidate = _state_by_id(candidate_root / "data" / f"Part_{part_index:04d}.bi4")
        if set(baseline) != set(candidate) or set(fluid_ids) - set(baseline):
            raise CflSensitivityQcError(f"paired particle Id alignment differs at Part_{part_index:04d}")
        position = []
        velocity = []
        density = []
        for particle_id in fluid_ids:
            left = baseline[particle_id]
            right = candidate[particle_id]
            position.append(math.dist(left[0], right[0]))
            velocity.append(math.dist(left[1], right[1]))
            density.append(abs(left[2] - right[2]))
        record = {
            "part": part_index,
            "position_m": _distance_stats(position),
            "velocity_m_s": _distance_stats(velocity),
            "density_kg_m3": _distance_stats(density),
        }
        summaries.append(record)
        for statistic in maximums:
            for quantity in ("position_m", "velocity_m_s", "density_kg_m3"):
                candidate_point = {"part": part_index, "value": record[quantity][statistic]}
                current = maximums[statistic].get(quantity)
                if current is None or candidate_point["value"] > current["value"]:
                    maximums[statistic][quantity] = candidate_point
        if part_index == PART_INDICES[0] and (
            baseline_identity["canonical_by_id_array_sha256"]
            != candidate_identity["canonical_by_id_array_sha256"]
        ):
            raise CflSensitivityQcError("paired initial state arrays differ")
    initial = summaries[0]
    initial_exact = all(
        value == 0.0
        for quantity in ("position_m", "velocity_m_s", "density_kg_m3")
        for value in initial[quantity].values()
    )
    if not initial_exact:
        raise CflSensitivityQcError("paired initial state values differ")
    return {
        "particle_id_alignment": True,
        "fluid_particle_count": len(fluid_ids),
        "frames_compared": len(summaries),
        "initial_arrays_exact": True,
        "per_frame_summary_canonical_sha256": base._canonical_sha256(summaries),
        "initial": initial,
        "final": summaries[-1],
        "trajectory_maximum": maximums,
    }


def _policy_delta(baseline_policy: dict[str, Any], candidate_policy: dict[str, Any]) -> dict[str, Any]:
    baseline_argv = baseline_policy.get("solver", {}).get("argv")
    candidate_argv = candidate_policy.get("solver", {}).get("argv")
    if not isinstance(baseline_argv, list) or not isinstance(candidate_argv, list):
        raise CflSensitivityQcError("solver argv is missing")
    if candidate_argv.count("-cfl:0.1") != 1 or any(str(item).startswith("-cfl:") for item in baseline_argv):
        raise CflSensitivityQcError("policy CFL argv delta differs")
    normalized = [item for item in candidate_argv if item != "-cfl:0.1"]
    same_solver_except_cfl = normalized == baseline_argv
    same_inputs = baseline_policy.get("inputs") == candidate_policy.get("inputs")
    same_restart = baseline_policy.get("restart") == candidate_policy.get("restart")
    same_gpu = baseline_policy.get("gpu") == candidate_policy.get("gpu")
    same_expected_output = baseline_policy.get("expected_output") == candidate_policy.get("expected_output")
    baseline_solver_rest = dict(baseline_policy.get("solver", {}))
    candidate_solver_rest = dict(candidate_policy.get("solver", {}))
    baseline_solver_rest.pop("argv", None)
    candidate_solver_rest.pop("argv", None)
    same_solver_rest = baseline_solver_rest == candidate_solver_rest
    declared = candidate_policy.get("single_delta") == {
        "parameter": "CFL",
        "baseline": "0.2",
        "candidate": "0.1",
        "other_numerical_parameters_changed": False,
    }
    checks = {
        "same_solver_argv_after_removing_exact_cfl_argument": same_solver_except_cfl,
        "same_solver_non_argv_contract": same_solver_rest,
        "same_input_contract": same_inputs,
        "same_restart_contract": same_restart,
        "same_gpu_contract": same_gpu,
        "same_expected_output_contract": same_expected_output,
        "candidate_declares_exact_cfl_single_delta": declared,
    }
    if not all(checks.values()):
        raise CflSensitivityQcError(f"policy numerical delta is not isolated: {checks}")
    return {
        "inserted_argument": "-cfl:0.1",
        "baseline_solver_argv_canonical_sha256": base._canonical_sha256(baseline_argv),
        "candidate_solver_argv_canonical_sha256": base._canonical_sha256(candidate_argv),
        "candidate_without_cfl_canonical_sha256": base._canonical_sha256(normalized),
        "checks": checks,
    }


def build_report(
    baseline_root: Path,
    candidate_root: Path,
    prior_checkpoint: Path,
    case_bi4: Path,
    case_xml: Path,
    plan: Path,
    contract: Path,
    baseline_policy_path: Path,
    candidate_policy_path: Path,
    baseline_final_path: Path,
    candidate_final_path: Path,
    baseline_qc: Path,
) -> dict[str, Any]:
    inputs = {
        "qc_script": base._identity(SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "qc_schema": base._identity(SCHEMA_PATH, maximum=MAX_JSON_BYTES),
        "qc_tests": base._identity(TEST_PATH, maximum=MAX_JSON_BYTES),
        "plan": base._identity(plan, expected_sha256=PLAN_SHA256),
        "contract": base._identity(contract, maximum=MAX_JSON_BYTES, expected_sha256=CONTRACT_SHA256),
        "case_bi4": base._identity(case_bi4, expected_sha256=base.CASE_BI4_SHA256),
        "case_xml": base._identity(case_xml, expected_sha256=base.CASE_XML_SHA256),
        "prior_checkpoint": base._identity(prior_checkpoint, maximum=MAX_FILE_BYTES, expected_sha256="0eee6feb93759f733030601d767521f343ed8cd348f8b2b47104a9ebd4f1d54c"),
        "baseline_qc": base._identity(baseline_qc, maximum=MAX_JSON_BYTES, expected_sha256=BASELINE_QC_SHA256),
    }
    contract_value, _ = _safe_json(contract, CONTRACT_SHA256)
    baseline_policy, inputs["baseline_policy"] = _safe_json(baseline_policy_path, BASELINE_POLICY_SHA256)
    candidate_policy, inputs["candidate_policy"] = _safe_json(candidate_policy_path, CANDIDATE_POLICY_SHA256)
    baseline_final, inputs["baseline_final"] = _receipt(
        baseline_final_path,
        BASELINE_FINAL_SHA256,
        "SMPCC_R8_LIQUID_U3_GPU_STAGE4_RUN_FINAL_V3",
        BASELINE_RUN_ID,
        BASELINE_INVENTORY_SHA256,
    )
    candidate_final, inputs["candidate_final"] = _receipt(
        candidate_final_path,
        CANDIDATE_FINAL_SHA256,
        "SMPCC_R8_LIQUID_U3_GPU_STAGE4_RUN_FINAL_V4",
        CANDIDATE_RUN_ID,
        CANDIDATE_INVENTORY_SHA256,
    )
    acceptance = contract_value.get("acceptance", {})
    expected_acceptance = {
        "all_original_absolute_limits_required": True,
        "no_metric_material_regression_required": True,
        "maximum_regression_fraction_of_limit": base.CONTRACT.maximum_metric_regression_fraction_of_limit,
        "minimum_primary_metrics_meaningfully_improved": base.CONTRACT.minimum_primary_metrics_improved,
        "meaningful_improvement_fraction_of_limit": base.CONTRACT.meaningful_metric_improvement_fraction_of_limit,
        "maximum_primary_normalized_score_ratio": base.CONTRACT.maximum_primary_normalized_score_ratio,
        "same_gpu_backend_required": True,
        "same_initial_state_arrays_required": True,
    }
    if acceptance != expected_acceptance:
        raise CflSensitivityQcError("sensitivity acceptance differs from frozen comparator")
    case = qc_v3.load_moving_case(
        case_bi4,
        case_xml,
        expected_bi4_sha256=base.CASE_BI4_SHA256,
        expected_xml_sha256=base.CASE_XML_SHA256,
    )
    policy_delta = _policy_delta(baseline_policy, candidate_policy)
    baseline = _analyze_run(
        baseline_root,
        case,
        prior_checkpoint,
        baseline_final,
        BASELINE_INVENTORY_SHA256,
        BASELINE_RUN_OUT_SHA256,
        BASELINE_RUN_CSV_SHA256,
        0.2,
    )
    candidate = _analyze_run(
        candidate_root,
        case,
        prior_checkpoint,
        candidate_final,
        CANDIDATE_INVENTORY_SHA256,
        CANDIDATE_RUN_OUT_SHA256,
        CANDIDATE_RUN_CSV_SHA256,
        0.1,
    )
    paired_state = _paired_state_comparison(baseline_root, candidate_root, case)
    comparison = base.compare_metric_vectors(baseline["metrics"], candidate["metrics"])
    pair_checks = {
        "both_final_receipts_pass_raw_output": True,
        "both_exact_210_file_inventories_match_receipts": True,
        "same_candidate_binary_identity": (
            baseline_final["candidate_source_after"]["sha256"]
            == candidate_final["candidate_source_after"]["sha256"]
            == CANDIDATE_BINARY_SHA256
        ),
        "same_gpu_backend": (
            baseline["backend"] == candidate["backend"]
            and baseline_final["gpu_after"]["uuid"] == candidate_final["gpu_after"]["uuid"] == GPU_UUID
        ),
        "same_initial_state_arrays_by_particle_id": (
            baseline["restart_output_state"]["canonical_by_id_array_sha256"]
            == candidate["restart_output_state"]["canonical_by_id_array_sha256"]
            == baseline["restart_input_state"]["canonical_by_id_array_sha256"]
        ),
        "same_window_and_output_schedule": (
            baseline["frame_count"], candidate["frame_count"]
        ) == (201, 201)
        and abs(baseline["first_time_s"] - candidate["first_time_s"]) <= base.CONTRACT.physical_time_tolerance_s
        and abs(baseline["last_time_s"] - candidate["last_time_s"]) <= base.CONTRACT.physical_time_tolerance_s,
        "same_shifting_ddt_dp_and_configuration": (
            baseline["shifting"] == candidate["shifting"] == "None"
            and baseline["ddt_parameters"] == candidate["ddt_parameters"]
            and baseline["dp_m"] == candidate["dp_m"] == 0.002
            and baseline["run_configuration"] == candidate["run_configuration"]
        ),
        "actual_cfl_is_exact_single_delta": baseline["actual_cfl"] == 0.2 and candidate["actual_cfl"] == 0.1,
        "policy_contract_is_exact_single_delta": all(policy_delta["checks"].values()),
        "both_structural_checks_pass": all(baseline["structural_checks"].values()) and all(candidate["structural_checks"].values()),
        "candidate_source_not_executed_and_network_not_used": (
            baseline_final["candidate_source_executed"] is False
            and candidate_final["candidate_source_executed"] is False
            and baseline_final["network_used"] is False
            and candidate_final["network_used"] is False
        ),
    }
    if not all(pair_checks.values()):
        raise CflSensitivityQcError(f"paired evidence checks failed: {pair_checks}")
    failed_absolute = sorted(name for name, passed in candidate["metric_absolute_pass"].items() if not passed)
    material_regressions = sorted(
        name for name, item in comparison["metrics"].items() if not item["no_material_regression"]
    )
    selected = comparison["pass"] and all(pair_checks.values())
    stage4_status = (
        "PASS_PHASE4_LIQUID_STANDALONE_NUMERICAL_PREREQUISITE"
        if selected
        else "FAIL_PHASE4_LIQUID_STANDALONE_NUMERICAL_STABILITY"
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": inputs,
        "contract": {
            "frozen_before_qc": True,
            "threshold_overrides_allowed": False,
            "window_part_indices": [201, 401],
            "window_time_s": [10.05, 20.05],
            "tail_window_s": 1.0,
            "baseline_cfl": 0.2,
            "candidate_cfl": 0.1,
            "single_numerical_delta": "CFL_0P2_TO_0P1",
            "metric_limits": dict(sorted(base.METRIC_LIMITS.items())),
            "primary_metrics": list(base.PRIMARY_METRICS),
            "acceptance": acceptance,
            "policy_delta_proof": policy_delta,
        },
        "baseline": baseline,
        "candidate": candidate,
        "pair_checks": pair_checks,
        "paired_state_comparison": paired_state,
        "metric_comparison": comparison,
        "verdict": {
            "status": (
                "PASS_U3_CFL0P1_SENSITIVITY_NUMERICAL_CANDIDATE"
                if selected
                else "FAIL_U3_CFL0P1_SENSITIVITY_NUMERICAL_STABILITY"
            ),
            "sensitivity_candidate_selected": selected,
            "candidate_settled_state": all(candidate["metric_absolute_pass"].values()),
            "candidate_failed_absolute_metrics": failed_absolute,
            "materially_regressed_metrics": material_regressions,
            "baseline_numerical_rebound_detected": baseline["trajectory"]["rebound_after_minimum"],
            "candidate_numerical_rebound_detected": candidate["trajectory"]["rebound_after_minimum"],
            "stage4_adjudication_status": stage4_status,
            "phase4_execution_and_adjudication_complete": True,
            "phase5_admitted": selected,
            "exact_blocker": (
                "NONE"
                if selected
                else f"CFL_0P1_FAILS_{len(failed_absolute)}_OF_17_ABSOLUTE_SETTLING_LIMITS_AND_REBOUND_PERSISTS"
            ),
            "next": (
                "S4C_REPEATABILITY"
                if selected
                else "STOP_BEFORE_PHASE5_REQUIRES_NUMERICAL_REMEDIATION_AND_NEW_AUTHORIZATION"
            ),
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "candidate_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
        },
    }


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    if not stat.S_ISDIR(os.lstat(path.parent).st_mode):
        raise CflSensitivityQcError("output parent is not a real directory")
    for name in ("baseline", "candidate"):
        if path.is_relative_to(Path(report[name]["root"])):
            raise CflSensitivityQcError("output is inside an input tree")
    data = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    if len(data) > MAX_JSON_BYTES:
        raise CflSensitivityQcError("result exceeds output bound")
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o640,
    )
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise CflSensitivityQcError("short output write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return base._identity(path, maximum=MAX_JSON_BYTES)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    for name in (
        "baseline-root",
        "candidate-root",
        "prior-checkpoint",
        "case-bi4",
        "case-xml",
        "plan",
        "contract",
        "baseline-policy",
        "candidate-policy",
        "baseline-final",
        "candidate-final",
        "baseline-qc",
    ):
        result.add_argument(f"--{name}", required=True, type=Path)
    result.add_argument("--output", type=Path)
    result.add_argument("--self-check", action="store_true")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        report = build_report(
            args.baseline_root,
            args.candidate_root,
            args.prior_checkpoint,
            args.case_bi4,
            args.case_xml,
            args.plan,
            args.contract,
            args.baseline_policy,
            args.candidate_policy,
            args.baseline_final,
            args.candidate_final,
            args.baseline_qc,
        )
        if args.self_check:
            print(json.dumps({
                "status": "PASS_U3_CFL0P1_SENSITIVITY_QC_V1_SELF_CHECK",
                "stage4_adjudication_status": report["verdict"]["stage4_adjudication_status"],
                "candidate_settled_state": report["verdict"]["candidate_settled_state"],
                "solver_executed": False,
            }, sort_keys=True))
            return 0
        if args.output is None:
            raise CflSensitivityQcError("--output is required unless --self-check is used")
        identity = write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["sensitivity_candidate_selected"] else 1
    except (
        CflSensitivityQcError,
        base.DdtRampQcError,
        extended.ExtendedSettlingQcError,
        qc_v3.QcError,
        bi4.Bi4FormatError,
        OSError,
        UnicodeError,
        ValueError,
    ) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
