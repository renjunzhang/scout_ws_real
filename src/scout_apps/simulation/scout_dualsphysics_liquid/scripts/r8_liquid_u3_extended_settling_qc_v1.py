#!/usr/bin/env python3
"""Read-only fail-closed QC for the 10.05..20.05 s GPU settling extension."""

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
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_ddt_ramp_qc_v1 as base
import r8_liquid_u3_solver_output_qc_v3 as qc_v3


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_extended_settling_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_extended_settling_qc_v1.py"
SCHEMA_VERSION = "r8-liquid-u3-extended-settling-qc-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_U3_EXTENDED_SETTLING_QC_V1"
PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"
CONTRACT_SHA256 = "d36997e1a4ab31f9a7826062d2fdf08508f6653918827c32fc6968002ac92c16"
POLICY_SHA256 = "a1c6642db2cb7cb586ff25c24f80911ba72ad54f7634d37817b82aa350caf2f4"
FINAL_SHA256 = "c7f26f9ba2b742718113094b0ef7c24ce742d0063acaa4918c9083810a3143b5"
RUN_OUT_SHA256 = "e35bb1305bd98460d77660a5502805ed425a02d87303f5ccca7a67efad3fde04"
INVENTORY_SHA256 = "d6b70b50b348b8a234b6022c2972bd7bfc6e3892fc7a9c6c8eb9bcb5b808beb5"
PART_INDICES = tuple(range(201, 402))
ROOT_FILES = ("CfgInit_Domain.vtk", "CfgInit_MapCells.vtk", "Run.csv", "Run.out", "RunPARTs.csv")
STATIC_DATA_FILES = ("Part_Head.ibi4", "PartInfo.ibi4", "PartMotionRef.ibi4", "PartOut_000.obi4")
EXPECTED_FILE_COUNT = 210
MAX_FILE_BYTES = 64 * 1024 * 1024
MAX_TREE_BYTES = 128 * 1024 * 1024
MAX_JSON_BYTES = 4 * 1024 * 1024


class ExtendedSettlingQcError(ValueError):
    """Unsafe input or failed structural identity."""


def _safe_json(path: Path, expected_sha256: str) -> tuple[dict[str, Any], dict[str, Any]]:
    identity = base._identity(path, maximum=MAX_JSON_BYTES, expected_sha256=expected_sha256)
    source = bi4.read_regular_file(path, max_bytes=MAX_JSON_BYTES)
    try:
        value = json.loads(source.data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ExtendedSettlingQcError(f"invalid JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ExtendedSettlingQcError(f"JSON root is not object: {path}")
    return value, identity


def _inventory(root: Path) -> dict[str, Any]:
    if not stat.S_ISDIR(os.lstat(root).st_mode) or not stat.S_ISDIR(os.lstat(root / "data").st_mode):
        raise ExtendedSettlingQcError("run root/data is not a real directory")
    expected_root = set(ROOT_FILES) | {"data"}
    expected_data = set(STATIC_DATA_FILES) | {f"Part_{index:04d}.bi4" for index in PART_INDICES}
    if {entry.name for entry in os.scandir(root)} != expected_root:
        raise ExtendedSettlingQcError("run root inventory differs")
    if {entry.name for entry in os.scandir(root / "data")} != expected_data:
        raise ExtendedSettlingQcError("run data inventory differs")
    records: dict[str, dict[str, Any]] = {}
    total = 0
    relatives = [*ROOT_FILES, *(f"data/{name}" for name in (*STATIC_DATA_FILES, *(f"Part_{i:04d}.bi4" for i in PART_INDICES)))]
    for relative in relatives:
        item = base._identity(root / relative, maximum=MAX_FILE_BYTES)
        records[relative] = {key: item[key] for key in ("size_bytes", "sha256", "mode", "nlink")}
        total += item["size_bytes"]
    if len(records) != EXPECTED_FILE_COUNT or total > MAX_TREE_BYTES:
        raise ExtendedSettlingQcError("run tree count/size exceeds contract")
    return {
        "file_count": len(records),
        "total_bytes": total,
        "canonical_sha256": base._canonical_sha256(records),
    }


def _state_array_sha(path: Path) -> dict[str, Any]:
    source = bi4.read_regular_file(path, max_bytes=MAX_FILE_BYTES)
    parsed = bi4.parse_jpartdata_bi4(source.data)
    _, ids, positions, velocities, densities = qc_v3._part_arrays(parsed)
    digest = hashlib.sha256()
    for particle_id, position, velocity, density in zip(ids, positions, velocities, densities):
        digest.update(struct.pack(">Q7d", particle_id, *position, *velocity, density))
    return {
        "path": str(path),
        "part_file_sha256": source.sha256,
        "particle_count": len(ids),
        "canonical_array_sha256": digest.hexdigest(),
    }


def _trajectory(parts: list[dict[str, Any]]) -> dict[str, Any]:
    series = [(item["cpart"], item["time_s"], item["speed"]["rms_m_s"]) for item in parts]
    if any(value is None or not math.isfinite(value) for _, _, value in series):
        raise ExtendedSettlingQcError("speed RMS series is non-finite")
    minimum = min(series, key=lambda item: item[2])
    final = series[-1]
    return {
        "initial": {"part": series[0][0], "time_s": series[0][1], "speed_rms_m_s": series[0][2]},
        "minimum": {"part": minimum[0], "time_s": minimum[1], "speed_rms_m_s": minimum[2]},
        "final": {"part": final[0], "time_s": final[1], "speed_rms_m_s": final[2]},
        "final_to_minimum_ratio": final[2] / minimum[2],
        "rebound_after_minimum": final[2] > minimum[2] * 1.2,
    }


def build_report(
    root: Path,
    prior_checkpoint: Path,
    case_bi4: Path,
    case_xml: Path,
    plan: Path,
    contract: Path,
    policy: Path,
    final_receipt: Path,
) -> dict[str, Any]:
    inputs = {
        "qc_script": base._identity(SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "qc_schema": base._identity(SCHEMA_PATH, maximum=MAX_JSON_BYTES),
        "qc_tests": base._identity(TEST_PATH, maximum=MAX_JSON_BYTES),
        "plan": base._identity(plan, expected_sha256=PLAN_SHA256),
        "contract": base._identity(contract, maximum=MAX_JSON_BYTES, expected_sha256=CONTRACT_SHA256),
        "policy": base._identity(policy, maximum=MAX_JSON_BYTES, expected_sha256=POLICY_SHA256),
        "case_bi4": base._identity(case_bi4, expected_sha256=base.CASE_BI4_SHA256),
        "case_xml": base._identity(case_xml, expected_sha256=base.CASE_XML_SHA256),
    }
    final, inputs["final_receipt"] = _safe_json(final_receipt, FINAL_SHA256)
    if not (
        final.get("status") == "PASS_GPU_STAGE4_RUN_RAW_OUTPUT"
        and final.get("run_id") == "u3_c1m_gpu_shift_none_extend_from_0201_v3"
        and final.get("returncode") == 0
        and final.get("termination_reason") == "PROCESS_EXIT"
        and final.get("candidate_source_executed") is False
        and final.get("staged_candidate_executed") is True
        and final.get("network_used") is False
        and final.get("output_inventory", {}).get("canonical_sha256") == INVENTORY_SHA256
    ):
        raise ExtendedSettlingQcError("final receipt semantics differ")
    inventory = _inventory(root)
    if inventory["canonical_sha256"] != INVENTORY_SHA256:
        raise ExtendedSettlingQcError("output inventory differs from final receipt")
    case = qc_v3.load_moving_case(
        case_bi4, case_xml,
        expected_bi4_sha256=base.CASE_BI4_SHA256,
        expected_xml_sha256=base.CASE_XML_SHA256,
    )
    thresholds = base._thresholds()
    parts = [
        qc_v3.parse_part_file(
            qc_v3._read(root / "data" / f"Part_{index:04d}.bi4", maximum=MAX_FILE_BYTES),
            index, case, thresholds,
        )
        for index in PART_INDICES
    ]
    run_out_source = qc_v3._read(root / "Run.out", maximum=MAX_FILE_BYTES, expected_sha256=RUN_OUT_SHA256)
    run_csv = qc_v3.parse_run_csv(qc_v3._read(root / "Run.csv", maximum=MAX_FILE_BYTES))
    run_out = qc_v3.parse_run_out(run_out_source)
    ddt = base._parse_ddt_parameters(run_out_source, candidate=False)
    text = run_out_source.data.decode("utf-8")
    shifting = re.findall(r'^Shifting="([^"]+)"\s*$', text, flags=re.MULTILINE)
    if shifting != ["None"]:
        raise ExtendedSettlingQcError(f"Shifting identity differs: {shifting!r}")
    partout = qc_v3.parse_partout(qc_v3._read(root / "data/PartOut_000.obi4", maximum=MAX_FILE_BYTES))
    motion = base._parse_continuation_motion_ref(
        qc_v3._read(root / "data/PartMotionRef.ibi4", maximum=MAX_FILE_BYTES), parts, case, thresholds,
    )
    runparts = base._parse_runparts(qc_v3._read(root / "RunPARTs.csv", maximum=MAX_FILE_BYTES), parts)
    fields = run_csv["physics_fields"]
    times = [item["time_s"] for item in parts]
    steps = [item["step"] for item in parts]
    structural = {
        "exact_inventory": inventory["file_count"] == EXPECTED_FILE_COUNT,
        "exact_part_range": [item["cpart"] for item in parts] == list(PART_INDICES),
        "strictly_increasing_times": all(right > left for left, right in zip(times, times[1:])),
        "monotonic_steps": all(right >= left for left, right in zip(steps, steps[1:])),
        "start_time_matches": abs(times[0] - 10.05) <= base.CONTRACT.physical_time_tolerance_s,
        "end_time_matches": abs(times[-1] - 20.05) <= base.CONTRACT.physical_time_tolerance_s,
        "all_particle_arrays_finite": all(all(item["finite"].values()) for item in parts),
        "all_ids_complete_unique_in_range": all(item["ids_unique"] and item["ids_in_range"] and item["missing_id_count"] == 0 for item in parts),
        "particle_classes_constant": all(item["counts"]["moving_boundary"] == 2669 and item["counts"]["fluid"] == 6409 and item["npok"] == 9078 for item in parts),
        "root_invariants_match_case": all(item["root_case_invariants_match"] for item in parts),
        "moving_boundary_zero_motion": all(item["moving_positions_exact"] and item["moving_velocities_zero"] for item in parts),
        "zero_nout": all(item["nout"] == 0 for item in parts) and partout["total_nout"] == 0 and fields["PartsOut"] == 0 and run_out["excluded_particles"] == 0,
        "finite_inside_domain_density": all(item["domain_outside_count"] == 0 and item["density_outside_count"] == 0 for item in parts),
        "motion_reference_zero_aligned": motion["all_posref_finite"] and motion["all_posref_unchanged"] and motion["aligned_with_particle_parts"] and motion["part_count"] == len(parts),
        "runparts_matches": runparts["pass"],
        "run_csv_matches": fields["Np"] == 9078 and fields["PartFiles"] == 402 and abs(fields["PhysicalTime"] - times[-1]) <= base.CONTRACT.physical_time_tolerance_s,
        "run_out_clean_complete": run_out["completion_markers_pass"] and not any(run_out["bad_patterns"].values()),
    }
    if not all(structural.values()):
        raise ExtendedSettlingQcError(f"structural checks failed: {[key for key, value in structural.items() if not value]}")
    prior_state = _state_array_sha(prior_checkpoint)
    output_state = _state_array_sha(root / "data/Part_0201.bi4")
    restart_state_matches = prior_state["canonical_array_sha256"] == output_state["canonical_array_sha256"]
    if not restart_state_matches:
        raise ExtendedSettlingQcError("restart state arrays differ")
    metrics, tail = base._metric_vector(parts, case)
    metric_pass = {name: metrics[name] <= base.METRIC_LIMITS[name] for name in sorted(metrics)}
    trajectory = _trajectory(parts)
    settled = all(metric_pass.values()) and all(structural.values()) and restart_state_matches
    for part in parts:
        part.pop("_positions_by_id", None)
    return {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": inputs,
        "contract": {
            "window_part_indices": [201, 401],
            "window_time_s": [10.05, 20.05],
            "tail_window_s": 1.0,
            "metric_limits": dict(sorted(base.METRIC_LIMITS.items())),
            "threshold_overrides_allowed": False,
        },
        "run": {
            "root": str(root),
            "inventory": inventory,
            "frame_count": len(parts),
            "first_time_s": times[0],
            "last_time_s": times[-1],
            "run_out_sha256": run_out["sha256"],
            "run_csv_sha256": run_csv["sha256"],
            "shifting": shifting[0],
            "ddt_parameters": ddt,
            "restart_input_state": prior_state,
            "restart_output_state": output_state,
            "restart_state_arrays_match": restart_state_matches,
            "structural_checks": structural,
            "tail": tail,
            "metrics": metrics,
            "metric_absolute_pass": metric_pass,
            "trajectory": trajectory,
        },
        "verdict": {
            "status": "PASS_U3_EXTENDED_SETTLING" if settled else "FAIL_U3_EXTENDED_SETTLING_NUMERICAL_STABILITY",
            "settled_state": settled,
            "failed_metrics": sorted(name for name, passed in metric_pass.items() if not passed),
            "numerical_rebound_detected": trajectory["rebound_after_minimum"],
            "next": "S4C_REPEATABILITY" if settled else "CFL_0P1_SENSITIVITY_OR_STOP",
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "inputs_modified": False,
        },
    }


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    if not stat.S_ISDIR(os.lstat(path.parent).st_mode):
        raise ExtendedSettlingQcError("output parent is not directory")
    data = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    if len(data) > MAX_JSON_BYTES:
        raise ExtendedSettlingQcError("result exceeds output bound")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ExtendedSettlingQcError("short output write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return base._identity(path, maximum=MAX_JSON_BYTES)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    for name in ("root", "prior-checkpoint", "case-bi4", "case-xml", "plan", "contract", "policy", "final-receipt"):
        result.add_argument(f"--{name}", required=True, type=Path)
    result.add_argument("--output", type=Path)
    result.add_argument("--self-check", action="store_true")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        report = build_report(args.root, args.prior_checkpoint, args.case_bi4, args.case_xml, args.plan, args.contract, args.policy, args.final_receipt)
        if args.self_check:
            print(json.dumps({
                "status": "PASS_U3_EXTENDED_SETTLING_QC_V1_SELF_CHECK",
                "settled_state": report["verdict"]["settled_state"],
                "numerical_rebound_detected": report["verdict"]["numerical_rebound_detected"],
                "solver_executed": False,
            }, sort_keys=True))
            return 0
        if args.output is None:
            raise ExtendedSettlingQcError("--output is required unless --self-check is used")
        identity = write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["settled_state"] else 1
    except (ExtendedSettlingQcError, base.DdtRampQcError, qc_v3.QcError, bi4.Bi4FormatError, OSError, UnicodeError, ValueError) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
