#!/usr/bin/env python3
"""Read-only QC for the v11 40.05--45.05 s viscosity-0.3 probe.

The tool parses only preserved BI4 outputs.  It proves exact restart-state
equivalence at Part_0801, verifies the closed 101-frame inventory and exact
single viscosity delta, evaluates the unchanged 17 settled-state limits over
the final one-second tail, and publishes one O_EXCL receipt.  It never runs the solver or
uses a GPU.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import struct
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_ddt_ramp_qc_v1 as metric_contract
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy
import r8_liquid_u3_gpu_viscoart0p3_probe_runner_v1 as runner
import r8_liquid_u3_solver_output_qc_v3 as qc_v3


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p3_probe_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p3_probe_qc_v1.py"
PLAN_PATH = PACKAGE_ROOT.parents[3] / "docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md"
PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"
PART_INDICES = tuple(range(801, 902))
METRIC_LIMITS = dict(sorted(metric_contract.METRIC_LIMITS.items()))
METRIC_NAMES = tuple(sorted(METRIC_LIMITS))
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_FILE_BYTES = 64 * 1024 * 1024


class ViscoartProbeQcError(ValueError):
    """Unsafe, drifted, incomplete or inconsistent viscosity-probe evidence."""


def _identity(path: Path, *, maximum: int = MAX_FILE_BYTES) -> dict[str, Any]:
    item = legacy.identity(path, maximum=maximum)
    return {key: item[key] for key in ("path", "sha256", "size_bytes", "mode", "inode", "nlink")}


def _read_json(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    value = legacy.read_json(path)
    return value, _identity(path, maximum=MAX_JSON_BYTES)


def _canonical_state(path: Path) -> dict[str, Any]:
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
    if any(not math.isfinite(value) for _, _, value in series):
        raise ViscoartProbeQcError("speed trajectory is non-finite")
    minimum = min(series, key=lambda item: item[2])
    final = series[-1]
    return {
        "initial_part": series[0][0],
        "initial_time_s": series[0][1],
        "initial_speed_rms_m_s": series[0][2],
        "minimum_part": minimum[0],
        "minimum_time_s": minimum[1],
        "minimum_speed_rms_m_s": minimum[2],
        "final_part": final[0],
        "final_time_s": final[1],
        "final_speed_rms_m_s": final[2],
        "final_to_minimum_ratio": final[2] / minimum[2],
        "rebound_after_minimum": final[2] > minimum[2] * 1.2,
    }


def adjudicate(metrics: dict[str, float], structural_pass: bool, restart_matches: bool) -> dict[str, Any]:
    if tuple(sorted(metrics)) != METRIC_NAMES:
        raise ViscoartProbeQcError("metric vector differs from the frozen 17 items")
    if any(not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) < 0 for value in metrics.values()):
        raise ViscoartProbeQcError("metric vector contains invalid values")
    passes = {name: float(metrics[name]) <= METRIC_LIMITS[name] for name in METRIC_NAMES}
    failed = [name for name in METRIC_NAMES if not passes[name]]
    selected = structural_pass and restart_matches and not failed
    return {
        "metric_absolute_pass": passes,
        "failed_absolute_metrics": failed,
        "selected": selected,
        "status": (
            "PASS_U3_VISCOART0P3_SENSITIVITY_CANDIDATE"
            if selected else "FAIL_U3_VISCOART0P3_SENSITIVITY_NOT_SETTLED"
        ),
    }


def _parse_solver_proof(run_out_source: bi4.SecureFile) -> dict[str, Any]:
    text = run_out_source.data.decode("utf-8")
    shifting = re.findall(r'^Shifting="([^"]+)"\s*$', text, flags=re.MULTILINE)
    viscosity = re.findall(r'^Viscosity="([^"]+)"\s*$', text, flags=re.MULTILINE)
    visco = re.findall(r'^\s*Visco=([^\s]+)\s*$', text, flags=re.MULTILINE)
    if shifting != ["None"] or viscosity != ["Artificial"] or visco != ["0.3"]:
        raise ViscoartProbeQcError(f"solver numerical identity differs: {shifting!r}/{viscosity!r}/{visco!r}")
    ddt = metric_contract._parse_ddt_parameters(run_out_source, candidate=False)
    return {
        "backend": "GPU_0",
        "cfl": 0.1,
        "shifting": shifting[0],
        "viscosity_formulation": viscosity[0],
        "viscosity_value": float(visco[0]),
        "ddt_value": ddt["value"],
        "ddt_tramp_s": ddt["tramp_s"],
        "tmax_s": 45.05,
        "tout_s": 0.05,
        "other_numerical_parameters_changed": False,
    }


def build_report(policy_path: Path) -> dict[str, Any]:
    runner.configure()
    policy = legacy.load_policy(policy_path)
    if policy_path != runner.POLICY_PATH:
        raise ViscoartProbeQcError("policy path differs")
    policy_identity = _identity(policy_path, maximum=MAX_JSON_BYTES)
    final_path = Path(policy["run"]["final_receipt"])
    final, final_identity = _read_json(final_path)
    required_final = (
        final.get("document_type") == "SMPCC_R8_LIQUID_U3_GPU_STAGE4_RUN_FINAL_V4"
        and final.get("status") == "PASS_GPU_STAGE4_RUN_RAW_OUTPUT"
        and final.get("run_id") == runner.RUN_ID
        and final.get("purpose") == "viscoart0p3_sensitivity_probe"
        and final.get("returncode") == 0
        and final.get("termination_reason") == "PROCESS_EXIT"
        and final.get("candidate_source_executed") is False
        and final.get("staged_candidate_executed") is True
        and final.get("network_used") is False
        and final.get("project_workspace_writable_in_guest") is False
        and final.get("apparmor_profile_used") is False
        and final.get("solver_argv") == runner.exact_solver_argv()
        and final.get("policy", {}).get("sha256") == policy_identity["sha256"]
        and final.get("stderr", {}).get("sha256") == hashlib.sha256(b"").hexdigest()
        and final.get("stderr", {}).get("size_bytes") == 0
        and final.get("compute_processes_after") == []
    )
    if not required_final:
        raise ViscoartProbeQcError("final receipt semantics differ")

    root = Path(policy["run"]["output_root"])
    observed_inventory = legacy.safe_output_inventory(root, policy["expected_output"])
    frozen_inventory = final.get("output_inventory", {})
    for key in ("file_count", "part_file_count", "part_first", "part_last", "total_bytes", "canonical_sha256"):
        if observed_inventory.get(key) != frozen_inventory.get(key):
            raise ViscoartProbeQcError(f"output inventory differs at {key}")
    if any(item["mode"] != "0440" or item["nlink"] != 1 for item in observed_inventory["files"].values()):
        raise ViscoartProbeQcError("output mode/link invariant differs")

    case = qc_v3.load_moving_case(
        Path(policy["inputs"]["case_bi4"]["path"]),
        Path(policy["inputs"]["case_xml"]["path"]),
        expected_bi4_sha256=policy["inputs"]["case_bi4"]["sha256"],
        expected_xml_sha256=policy["inputs"]["case_xml"]["sha256"],
    )
    thresholds = metric_contract._thresholds()
    parts = [
        qc_v3.parse_part_file(
            qc_v3._read(root / "data" / f"Part_{index:04d}.bi4", maximum=MAX_FILE_BYTES),
            index,
            case,
            thresholds,
        )
        for index in PART_INDICES
    ]
    times = [item["time_s"] for item in parts]
    steps = [item["step"] for item in parts]
    run_out_source = qc_v3._read(root / "Run.out", maximum=MAX_FILE_BYTES)
    run_out = qc_v3.parse_run_out(run_out_source)
    run_csv = qc_v3.parse_run_csv(qc_v3._read(root / "Run.csv", maximum=MAX_FILE_BYTES))
    fields = run_csv["physics_fields"]
    partout = qc_v3.parse_partout(qc_v3._read(root / "data/PartOut_000.obi4", maximum=MAX_FILE_BYTES))
    motion = metric_contract._parse_continuation_motion_ref(
        qc_v3._read(root / "data/PartMotionRef.ibi4", maximum=MAX_FILE_BYTES), parts, case, thresholds
    )
    runparts = metric_contract._parse_runparts(qc_v3._read(root / "RunPARTs.csv", maximum=MAX_FILE_BYTES), parts)
    structural = {
        "exact_inventory": observed_inventory["file_count"] == 110,
        "exact_part_range": [item["cpart"] for item in parts] == list(PART_INDICES),
        "strictly_increasing_times": all(right > left for left, right in zip(times, times[1:])),
        "monotonic_steps": all(right >= left for left, right in zip(steps, steps[1:])),
        "start_time_matches": abs(times[0] - 40.05) <= metric_contract.CONTRACT.physical_time_tolerance_s,
        "end_time_matches": abs(times[-1] - 45.05) <= metric_contract.CONTRACT.physical_time_tolerance_s,
        "all_particle_arrays_finite": all(all(item["finite"].values()) for item in parts),
        "all_ids_complete_unique_in_range": all(item["ids_unique"] and item["ids_in_range"] and item["missing_id_count"] == 0 for item in parts),
        "particle_classes_constant": all(item["counts"]["moving_boundary"] == 2669 and item["counts"]["fluid"] == 6409 and item["npok"] == 9078 for item in parts),
        "root_invariants_match_case": all(item["root_case_invariants_match"] for item in parts),
        "moving_boundary_zero_motion": all(item["moving_positions_exact"] and item["moving_velocities_zero"] for item in parts),
        "zero_nout": all(item["nout"] == 0 for item in parts) and partout["total_nout"] == 0 and fields["PartsOut"] == 0 and run_out["excluded_particles"] == 0,
        "finite_inside_domain_density": all(item["domain_outside_count"] == 0 and item["density_outside_count"] == 0 for item in parts),
        "motion_reference_zero_aligned": motion["all_posref_finite"] and motion["all_posref_unchanged"] and motion["aligned_with_particle_parts"] and motion["part_count"] == len(parts),
        "runparts_matches": runparts["pass"],
        "run_csv_matches": fields["Np"] == 9078 and fields["PartFiles"] == 902 and abs(fields["PhysicalTime"] - times[-1]) <= metric_contract.CONTRACT.physical_time_tolerance_s,
        "run_out_clean_complete": run_out["completion_markers_pass"] and not any(run_out["bad_patterns"].values()),
    }
    if not all(structural.values()):
        raise ViscoartProbeQcError(f"structural checks failed: {[name for name, value in structural.items() if not value]}")

    restart_input = _canonical_state(runner.RESTART_PART_PATH)
    restart_output = _canonical_state(root / "data/Part_0801.bi4")
    restart_matches = restart_input["canonical_array_sha256"] == restart_output["canonical_array_sha256"]
    if not restart_matches:
        raise ViscoartProbeQcError("restart Part_0801 state arrays differ")
    metrics, tail = metric_contract._metric_vector(parts, case)
    result = adjudicate(metrics, all(structural.values()), restart_matches)
    trajectory = _trajectory(parts)
    solver_proof = _parse_solver_proof(run_out_source)
    for part in parts:
        part.pop("_positions_by_id", None)

    inputs = {
        "plan": _identity(PLAN_PATH, maximum=MAX_JSON_BYTES),
        "policy": policy_identity,
        "runner": _identity(runner.SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "runner_schema": _identity(runner.SCHEMA_PATH, maximum=MAX_JSON_BYTES),
        "runner_tests": _identity(runner.TEST_PATH, maximum=MAX_JSON_BYTES),
        "qc_script": _identity(SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "qc_schema": _identity(SCHEMA_PATH, maximum=MAX_JSON_BYTES),
        "qc_tests": _identity(TEST_PATH, maximum=MAX_JSON_BYTES),
        "prior_qc": _identity(runner.PRIOR_QC_PATH, maximum=MAX_JSON_BYTES),
        "prior_tail_final": _identity(runner.PRIOR_FINAL_PATH, maximum=MAX_JSON_BYTES),
        "restart_part": _identity(runner.RESTART_PART_PATH),
        "restart_head": _identity(runner.RESTART_HEAD_PATH),
        "final_receipt": final_identity,
    }
    if inputs["plan"]["sha256"] != PLAN_SHA256:
        raise ViscoartProbeQcError("frozen plan identity differs")
    report = {
        "schema_version": "r8-liquid-u3-viscoart0p3-probe-qc-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_VISCOART0P3_PROBE_QC_V1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": inputs,
        "contract": {
            "window_start_s": 40.05,
            "window_end_s": 45.05,
            "tail_window_s": 1.0,
            "part_first": 801,
            "part_last": 901,
            "part_count": 101,
            "metric_limits": METRIC_LIMITS,
            "threshold_overrides_allowed": False,
            "single_delta": runner.EXPECTED_DELTA,
        },
        "run": {
            "root": str(root),
            "inventory": {key: observed_inventory[key] for key in ("file_count", "part_file_count", "part_first", "part_last", "total_bytes", "canonical_sha256")},
            "frame_count": len(parts),
            "first_time_s": times[0],
            "last_time_s": times[-1],
            "restart_input_state": restart_input,
            "restart_output_state": restart_output,
            "restart_state_arrays_match": restart_matches,
            "structural_checks": structural,
            "solver_proof": solver_proof,
            "tail": tail,
            "metrics": metrics,
            "metric_absolute_pass": result["metric_absolute_pass"],
            "failed_absolute_metrics": result["failed_absolute_metrics"],
            "trajectory": trajectory,
            "final_checkpoint": _identity(root / "data/Part_0901.bi4"),
        },
        "verdict": {
            "status": result["status"],
            "u3_remediation_candidate_pass": result["selected"],
            "settled_state_frozen": False,
            "stage4_complete": False,
            "phase5_admitted": False,
            "exact_blocker": "NONE" if result["selected"] else f"VISCOART0P3_PROBE_FAILS_{len(result['failed_absolute_metrics'])}_OF_17_ABSOLUTE_SETTLING_LIMITS",
            "next": "FREEZE_FRESH_COLD_AB_REPEATABILITY_RESTART_EQUIVALENCE_CONTRACT" if result["selected"] else "STOP_OR_FREEZE_ONE_NEW_NUMERICAL_DELTA",
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
        },
    }
    validate_report(report)
    return report


def validate_report(report: dict[str, Any]) -> None:
    run = report.get("run", {})
    metrics = run.get("metrics", {})
    passes = run.get("metric_absolute_pass", {})
    failed = run.get("failed_absolute_metrics", [])
    if tuple(sorted(metrics)) != METRIC_NAMES or tuple(sorted(passes)) != METRIC_NAMES:
        raise ViscoartProbeQcError("report metric/pass keys differ")
    expected_failed = [name for name in METRIC_NAMES if float(metrics[name]) > METRIC_LIMITS[name]]
    if failed != expected_failed or any(passes[name] is not (name not in expected_failed) for name in METRIC_NAMES):
        raise ViscoartProbeQcError("metric values, pass vector and failure list disagree")
    selected = not expected_failed and all(run.get("structural_checks", {}).values()) and run.get("restart_state_arrays_match") is True
    verdict = report.get("verdict", {})
    expected_status = "PASS_U3_VISCOART0P3_SENSITIVITY_CANDIDATE" if selected else "FAIL_U3_VISCOART0P3_SENSITIVITY_NOT_SETTLED"
    if verdict.get("status") != expected_status or verdict.get("u3_remediation_candidate_pass") is not selected:
        raise ViscoartProbeQcError("verdict disagrees with frozen adjudication")
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(report), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise ViscoartProbeQcError(f"QC schema failure at {list(first.absolute_path)}: {first.message}")


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    data = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    if len(data) > MAX_JSON_BYTES:
        raise ViscoartProbeQcError("QC output exceeds bound")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ViscoartProbeQcError("short QC write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return _identity(path, maximum=MAX_JSON_BYTES)


def static_self_check() -> dict[str, Any]:
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    if METRIC_LIMITS != dict(sorted(metric_contract.METRIC_LIMITS.items())):
        raise ViscoartProbeQcError("metric contract drifted")
    return {
        "status": "PASS_U3_VISCOART0P3_PROBE_QC_V1_STATIC_SELF_CHECK",
        "metric_count": len(METRIC_LIMITS),
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("policy", type=Path)
    result.add_argument("--output", type=Path)
    result.add_argument("--self-check", action="store_true")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.self_check:
            print(json.dumps(static_self_check(), sort_keys=True))
            return 0
        if args.output is None:
            raise ViscoartProbeQcError("--output is required")
        report = build_report(args.policy)
        identity = write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_remediation_candidate_pass"] else 1
    except (ViscoartProbeQcError, runner.ViscoartProbeRunError, legacy.Stage4RunError, metric_contract.DdtRampQcError, qc_v3.QcError, bi4.Bi4FormatError, OSError, UnicodeError, ValueError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
