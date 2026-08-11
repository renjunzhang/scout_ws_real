#!/usr/bin/env python3
"""Read-only QC for initialization-only damping followed by an undamped tail.

The tool verifies both create-new GPU run receipts and inventories, proves that
damping is present only in the initialization XML/run, verifies exact Part_0201
checkpoint transfer, and evaluates only the 10.05--20.05 s undamped output
against the pre-existing 17 absolute settling limits.  It never invokes the
solver, exposes a GPU, or modifies either run tree.
"""

from __future__ import annotations

import argparse
import json
import os
import stat
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_cfl0p1_sensitivity_qc_v1 as cfl_qc
import r8_liquid_u3_ddt_ramp_qc_v1 as metric_contract
import r8_liquid_u3_gpu_damped_restart_runner_v1 as runner
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy
import r8_liquid_u3_solver_output_qc_v3 as qc_v3


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_damped_restart_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_damped_restart_qc_v1.py"
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_FILE_BYTES = 64 * 1024 * 1024


class DampedRestartQcError(ValueError):
    """Unsafe, drifted, incomplete, or semantically invalid evidence."""


def _receipt(policy: dict[str, Any], phase_name: str) -> tuple[dict[str, Any], dict[str, Any]]:
    phase = policy["phases"][phase_name]
    path = Path(phase["run"]["final_receipt"])
    identity = legacy.identity(path, maximum=MAX_JSON_BYTES)
    value = runner.read_json(path)
    candidate = value.get("candidate_source_after", {})
    gpu = value.get("gpu_after", {})
    required = (
        value.get("schema_version") == "r8-liquid-u3-damped-restart-run-receipt-v1",
        value.get("document_type") == "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_RUN_FINAL_V1",
        value.get("status") == "PASS_GPU_STAGE4_DAMPED_RESTART_RAW_OUTPUT",
        value.get("experiment_id") == policy["experiment_id"],
        value.get("phase") == phase_name,
        value.get("run_id") == phase["run_id"],
        value.get("purpose") == phase["purpose"],
        value.get("returncode") == 0,
        value.get("termination_reason") == "PROCESS_EXIT",
        value.get("policy", {}).get("sha256") == legacy.sha256_file(Path(value.get("policy", {}).get("path", "/missing")), maximum=MAX_JSON_BYTES),
        value.get("candidate_source_executed") is False,
        value.get("staged_candidate_executed") is True,
        value.get("network_used") is False,
        value.get("project_workspace_writable_in_guest") is False,
        value.get("apparmor_profile_used") is False,
        value.get("compute_processes_after") == [],
        candidate.get("sha256") == policy["inputs"]["candidate"]["sha256"],
        candidate.get("mode") == "0400",
        candidate.get("nlink") == 1,
        gpu.get("uuid") == policy["gpu"]["uuid"],
        gpu.get("name") == policy["gpu"]["name"],
    )
    if not all(required):
        raise DampedRestartQcError(f"final receipt semantics differ: {path}")
    return value, identity


def _inventory(policy: dict[str, Any], phase_name: str, receipt: dict[str, Any]) -> dict[str, Any]:
    phase = policy["phases"][phase_name]
    observed = legacy.safe_output_inventory(Path(phase["run"]["output_root"]), phase["expected_output"])
    frozen = receipt.get("output_inventory")
    if observed != frozen:
        raise DampedRestartQcError(f"{phase_name} output inventory differs from final receipt")
    return {key: observed[key] for key in (
        "file_count", "part_file_count", "part_first", "part_last", "total_bytes", "canonical_sha256"
    )}


def _run_text(path: Path) -> str:
    source = bi4.read_regular_file(path, max_bytes=MAX_FILE_BYTES)
    try:
        return source.data.decode("utf-8")
    except UnicodeError as exc:
        raise DampedRestartQcError(f"Run.out is not UTF-8: {path}") from exc


def _checkpoint_state(policy: dict[str, Any], case: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    init = policy["phases"]["damped_init"]
    path = Path(init["run"]["output_root"]) / "data/Part_0201.bi4"
    part = qc_v3.parse_part_file(
        qc_v3._read(path, maximum=MAX_FILE_BYTES),
        201,
        case,
        metric_contract._thresholds(),
    )
    required = (
        part["npok"] == 9078,
        part["counts"]["moving_boundary"] == 2669,
        part["counts"]["fluid"] == 6409,
        part["nout"] == 0,
        all(part["finite"].values()),
        part["domain_outside_count"] == 0,
        part["density_outside_count"] == 0,
        part["ids_unique"],
        part["ids_in_range"],
        part["missing_id_count"] == 0,
        part["root_case_invariants_match"],
        part["moving_positions_exact"],
        part["moving_velocities_zero"],
    )
    if not all(required):
        raise DampedRestartQcError("damped initialization checkpoint structural QC failed")
    return {
        "part": 201,
        "time_s": part["time_s"],
        "particle_count": part["npok"],
        "moving_count": part["counts"]["moving_boundary"],
        "fluid_count": part["counts"]["fluid"],
        "nout": part["nout"],
        "all_finite": all(part["finite"].values()),
        "domain_and_density_valid": part["domain_outside_count"] == part["density_outside_count"] == 0,
    }, legacy.identity(path, maximum=MAX_FILE_BYTES)


def _checkpoint_transfer_exact(policy: dict[str, Any], tail_receipt: dict[str, Any], checkpoint: dict[str, Any]) -> bool:
    dynamic = tail_receipt.get("dynamic_restart", {})
    staged = tail_receipt.get("staged_inputs", {})
    return (
        dynamic.get("restart_part", {}).get("sha256") == checkpoint["sha256"]
        and dynamic.get("restart_part", {}).get("size_bytes") == checkpoint["size_bytes"]
        and staged.get("restart_part", {}).get("sha256") == checkpoint["sha256"]
        and staged.get("restart_head", {}).get("sha256") == dynamic.get("restart_head", {}).get("sha256")
    )


def _tail_analysis(
    policy: dict[str, Any],
    tail_receipt: dict[str, Any],
    case: dict[str, Any],
    checkpoint_path: Path,
) -> dict[str, Any]:
    tail = policy["phases"]["undamped_tail"]
    root = Path(tail["run"]["output_root"])
    frozen = tail_receipt["output_inventory"]
    cfl_qc.PART_INDICES = tuple(range(201, 402))
    cfl_qc.EXPECTED_FILE_COUNT = 210
    analysis = cfl_qc._analyze_run(
        root,
        case,
        checkpoint_path,
        tail_receipt,
        frozen["canonical_sha256"],
        frozen["files"]["Run.out"]["sha256"],
        frozen["files"]["Run.csv"]["sha256"],
        0.1,
    )
    if set(analysis["metrics"]) != set(metric_contract.METRIC_LIMITS):
        raise DampedRestartQcError("tail metric vector is not the frozen 17-item set")
    failed = sorted(name for name, passed in analysis["metric_absolute_pass"].items() if not passed)
    trajectory = analysis["trajectory"]
    return {
        "status": "PASS_UNDAMPED_TAIL_ALL_17_LIMITS" if not failed else "FAIL_UNDAMPED_TAIL_ABSOLUTE_SETTLING_LIMITS",
        "run_id": tail["run_id"],
        "returncode": tail_receipt["returncode"],
        "elapsed_seconds": tail_receipt["elapsed_seconds"],
        "inventory": _inventory(policy, "undamped_tail", tail_receipt),
        "frame_count": analysis["frame_count"],
        "first_time_s": analysis["first_time_s"],
        "last_time_s": analysis["last_time_s"],
        "actual_cfl": analysis["actual_cfl"],
        "shifting": analysis["shifting"],
        "ddt": analysis["ddt_parameters"],
        "dp_m": analysis["dp_m"],
        "structural_checks_all_pass": all(analysis["structural_checks"].values()),
        "metrics": dict(sorted(analysis["metrics"].items())),
        "metric_limits": dict(sorted(metric_contract.METRIC_LIMITS.items())),
        "metric_absolute_pass": dict(sorted(analysis["metric_absolute_pass"].items())),
        "failed_absolute_metrics": failed,
        "trajectory": {
            "minimum_time_s": trajectory["minimum"]["time_s"],
            "minimum_speed_rms_m_s": trajectory["minimum"]["speed_rms_m_s"],
            "final_time_s": trajectory["final"]["time_s"],
            "final_speed_rms_m_s": trajectory["final"]["speed_rms_m_s"],
            "final_to_minimum_ratio": trajectory["final_to_minimum_ratio"],
            "rebound_after_minimum": trajectory["rebound_after_minimum"],
        },
    }


def build_report(policy_path: Path) -> dict[str, Any]:
    policy = runner.load_policy(policy_path)
    runner.verify_frozen_files(policy)
    xml_proof = runner.validate_xml_delta(policy)
    init_receipt, init_receipt_id = _receipt(policy, "damped_init")
    tail_receipt, tail_receipt_id = _receipt(policy, "undamped_tail")
    init_inventory = _inventory(policy, "damped_init", init_receipt)
    undamped_xml = policy["inputs"]["undamped_case_xml"]
    case = qc_v3.load_moving_case(
        Path(policy["inputs"]["case_bi4"]["path"]),
        Path(undamped_xml["path"]),
        expected_bi4_sha256=policy["inputs"]["case_bi4"]["sha256"],
        expected_xml_sha256=undamped_xml["sha256"],
    )
    checkpoint_state, checkpoint_id = _checkpoint_state(policy, case)
    init_run_text = _run_text(Path(policy["phases"]["damped_init"]["run"]["output_root"]) / "Run.out")
    tail_run_text = _run_text(Path(policy["phases"]["undamped_tail"]["run"]["output_root"]) / "Run.out")
    damping_init = "Damping configuration:" in init_run_text and "Damping zone_0" in init_run_text
    damping_tail = "Damping configuration:" in tail_run_text or "Damping zone_0" in tail_run_text
    transfer = _checkpoint_transfer_exact(policy, tail_receipt, checkpoint_id)
    if not damping_init or damping_tail or not transfer:
        raise DampedRestartQcError(
            f"damping separation/checkpoint proof failed: init={damping_init} tail={damping_tail} transfer={transfer}"
        )
    tail = _tail_analysis(policy, tail_receipt, case, Path(checkpoint_id["path"]))
    selected = not tail["failed_absolute_metrics"] and tail["structural_checks_all_pass"]
    inputs = {
        "policy": legacy.identity(policy_path, maximum=MAX_JSON_BYTES),
        "runner": legacy.identity(runner.SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "qc_script": legacy.identity(SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "qc_schema": legacy.identity(SCHEMA_PATH, maximum=MAX_JSON_BYTES),
        "qc_tests": legacy.identity(TEST_PATH, maximum=MAX_JSON_BYTES),
        "init_receipt": init_receipt_id,
        "tail_receipt": tail_receipt_id,
        "checkpoint": checkpoint_id,
    }
    return {
        "schema_version": "r8-liquid-u3-damped-restart-qc-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_QC_V1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": inputs,
        "intervention_proof": {
            "single_intervention": "INITIALIZATION_ONLY_STANDARD_DAMPING_PLANE",
            "case_semantics_equal_after_removing_special": xml_proof["status"] == "PASS_EXACT_INITIALIZATION_ONLY_DAMPING_XML_DELTA",
            "damping_present_in_initialization_xml": True,
            "damping_absent_from_validation_xml": True,
            "damping_reported_by_initialization_solver": damping_init,
            "damping_not_reported_by_validation_solver": not damping_tail,
            "checkpoint_transfer_exact": transfer,
            "cfl_shifting_ddt_dp_unchanged": tail["actual_cfl"] == 0.1 and tail["shifting"] == "None" and tail["dp_m"] == 0.002,
            "evaluation_uses_only_undamped_tail": True,
        },
        "initialization": {
            "status": "PASS_DAMPED_INITIALIZATION_RAW_AND_CHECKPOINT_QC",
            "run_id": policy["phases"]["damped_init"]["run_id"],
            "returncode": init_receipt["returncode"],
            "elapsed_seconds": init_receipt["elapsed_seconds"],
            "inventory": init_inventory,
            "checkpoint_state": checkpoint_state,
        },
        "undamped_tail": tail,
        "verdict": {
            "status": (
                "PASS_U3_DAMPED_INITIALIZATION_UNDAMPED_TAIL_SETTLED_CANDIDATE"
                if selected else "FAIL_U3_DAMPED_INITIALIZATION_UNDAMPED_TAIL_NOT_SETTLED"
            ),
            "u3_remediation_candidate_pass": selected,
            "settled_state_frozen": False,
            "stage4_complete": False,
            "phase5_admitted": False,
            "exact_blocker": "NONE" if selected else f"UNDAMPED_TAIL_FAILS_{len(tail['failed_absolute_metrics'])}_OF_17_ABSOLUTE_SETTLING_LIMITS",
            "next": "COLD_B_RESTART_EQUIVALENCE_THEN_PARITY_U4" if selected else "STOP_AND_PRESERVE_REMEDIATION_EVIDENCE",
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
        },
    }


def validate_report(report: dict[str, Any]) -> None:
    tail = report.get("undamped_tail", {})
    if tail.get("metric_limits") != dict(sorted(metric_contract.METRIC_LIMITS.items())):
        raise DampedRestartQcError("QC report metric limits drifted from the frozen 17-item contract")
    if set(tail.get("metrics", {})) != set(metric_contract.METRIC_LIMITS):
        raise DampedRestartQcError("QC report metric vector is not the frozen 17-item set")
    if set(tail.get("metric_absolute_pass", {})) != set(metric_contract.METRIC_LIMITS):
        raise DampedRestartQcError("QC report metric-pass vector is not the frozen 17-item set")
    schema = runner.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(report), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise DampedRestartQcError(f"QC schema failure at {list(first.absolute_path)}: {first.message}")


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    if not stat.S_ISDIR(os.lstat(path.parent).st_mode):
        raise DampedRestartQcError("QC output parent is not a real directory")
    data = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    if len(data) > MAX_JSON_BYTES:
        raise DampedRestartQcError("QC result exceeds output bound")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise DampedRestartQcError("short QC output write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return legacy.identity(path, maximum=MAX_JSON_BYTES)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("policy", type=Path)
    result.add_argument("--output", type=Path)
    result.add_argument("--self-check", action="store_true")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        report = build_report(args.policy)
        validate_report(report)
        if args.self_check:
            print(json.dumps({
                "status": "PASS_U3_DAMPED_RESTART_QC_V1_SELF_CHECK",
                "verdict": report["verdict"]["status"],
                "solver_executed": False,
            }, sort_keys=True))
            return 0
        if args.output is None:
            raise DampedRestartQcError("--output is required unless --self-check is used")
        identity = write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_remediation_candidate_pass"] else 1
    except (
        DampedRestartQcError,
        runner.DampedRestartRunError,
        legacy.Stage4RunError,
        cfl_qc.CflSensitivityQcError,
        metric_contract.DdtRampQcError,
        qc_v3.QcError,
        bi4.Bi4FormatError,
        OSError,
        ValueError,
    ) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
