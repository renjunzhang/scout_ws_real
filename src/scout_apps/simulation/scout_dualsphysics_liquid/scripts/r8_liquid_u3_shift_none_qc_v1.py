#!/usr/bin/env python3
"""Read-only QC for the create-new GPU Shifting=None Stage-4 probe."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
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
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_shift_none_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_shift_none_qc_v1.py"
SCHEMA_VERSION = "r8-liquid-u3-shift-none-qc-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_U3_SHIFT_NONE_QC_V1"
PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"
CONTRACT_SHA256 = "7b3d7447485d901cd60402f2a7ddd422c587ffa57ba3e5664130d8cd3abd9cd7"
S4A_RESULT_SHA256 = "a9dd201b8cb7a2dbcccc2b645632dc1d1f63673bdb3cae6880c44dffb1a05cf2"
GPU_POLICY_SHA256 = "c4368b7bef18f595cdc347d27c8f65a12858be6fb40f8264a67de56edca530fd"
GPU_FINAL_SHA256 = "66bb86587d7963650cb16685eaae2dc6a8bc5fc0a0a94fbbe71c5316606d6644"
GPU_RUN_OUT_SHA256 = "f9b3d3a12e44bfcef07aeff4daf6b5b46134abf3f859af0cd101bdd80de60200"
EXPECTED_GPU_INVENTORY_SHA256 = "a30e8fddfc84b6ecc2b24e1bce1274d81a9b62e69e0698aa749a5486ef0bd505"
MAX_JSON_BYTES = 4 * 1024 * 1024


class ShiftNoneQcError(ValueError):
    """Frozen identity, structural, or numerical comparison failure."""


def _safe_json(path: Path, expected_sha256: str) -> tuple[dict[str, Any], dict[str, Any]]:
    identity = base._identity(path, maximum=MAX_JSON_BYTES, expected_sha256=expected_sha256)
    source = bi4.read_regular_file(path, max_bytes=MAX_JSON_BYTES)
    try:
        value = json.loads(source.data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ShiftNoneQcError(f"invalid JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ShiftNoneQcError(f"JSON root is not an object: {path}")
    return value, identity


def _run_csv_values(root: Path) -> dict[str, str]:
    source = bi4.read_regular_file(root / "Run.csv", max_bytes=base.MAX_FILE_BYTES)
    try:
        lines = [line for line in source.data.decode("utf-8").splitlines() if line.strip()]
        if lines and lines[0].startswith("#"):
            lines[0] = lines[0][1:]
        rows = list(csv.reader(lines, delimiter=";"))
    except UnicodeError as exc:
        raise ShiftNoneQcError("Run.csv is not UTF-8") from exc
    if len(rows) != 2 or len(rows[0]) != len(rows[1]) or len(set(rows[0])) != len(rows[0]):
        raise ShiftNoneQcError("Run.csv shape differs")
    return dict(zip(rows[0], rows[1]))


def _shifting_identity(root: Path, expected: str) -> dict[str, Any]:
    source = bi4.read_regular_file(root / "Run.out", max_bytes=base.MAX_FILE_BYTES)
    try:
        text = source.data.decode("utf-8")
    except UnicodeError as exc:
        raise ShiftNoneQcError("Run.out is not UTF-8") from exc
    matches = re.findall(r'^Shifting="([^"]+)"\s*$', text, flags=re.MULTILINE)
    if matches != [expected]:
        raise ShiftNoneQcError(f"Shifting identity differs: {matches!r}")
    return {"run_out_sha256": source.sha256, "value": expected}


def _configuration_without_single_delta(root: Path, expected: str) -> str:
    source = bi4.read_regular_file(root / "Run.csv", max_bytes=base.MAX_FILE_BYTES)
    normalized = base._configuration_without_ddt(source)
    if expected == "NoBound(-2,2.75,Full)":
        normalized, count = re.subn(r" - Shifting\(NoBound,-2,2\.75,Full\)", "", normalized)
        if count != 1:
            raise ShiftNoneQcError("baseline Run.csv lacks exact NoBound token")
    elif expected == "None":
        if "Shifting(" in normalized:
            raise ShiftNoneQcError("candidate Run.csv unexpectedly serializes a Shifting token")
    else:
        raise ShiftNoneQcError(f"unsupported Shifting value: {expected}")
    return normalized


def _initial_state_arrays(root: Path) -> dict[str, Any]:
    source = bi4.read_regular_file(root / "data/Part_0161.bi4", max_bytes=base.MAX_FILE_BYTES)
    try:
        parsed = bi4.parse_jpartdata_bi4(source.data)
        _, ids, positions, velocities, densities = qc_v3._part_arrays(parsed)
    except (bi4.Bi4FormatError, qc_v3.QcError, ValueError) as exc:
        raise ShiftNoneQcError(f"cannot parse initial state arrays: {exc}") from exc
    digest = hashlib.sha256()
    for particle_id, position, velocity, density in zip(ids, positions, velocities, densities):
        digest.update(struct.pack(">Q7d", particle_id, *position, *velocity, density))
    return {
        "part_file_sha256": source.sha256,
        "particle_count": len(ids),
        "canonical_array_sha256": digest.hexdigest(),
    }


def _backend(root: Path) -> dict[str, str]:
    values = _run_csv_values(root)
    hardware = values.get("Hardware", "")
    mode = values.get("RunMode", "")
    if not hardware or not mode:
        raise ShiftNoneQcError("Run.csv backend identity is absent")
    return {"hardware": hardware, "run_mode": mode}


def build_report(
    baseline_root: Path,
    candidate_root: Path,
    case_bi4: Path,
    case_xml: Path,
    plan: Path,
    baseline_qc: Path,
    numerical_contract: Path,
    s4a_result: Path,
    gpu_policy: Path,
    gpu_final: Path,
) -> dict[str, Any]:
    inputs = {
        "qc_script": base._identity(SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "qc_schema": base._identity(SCHEMA_PATH, maximum=MAX_JSON_BYTES),
        "qc_tests": base._identity(TEST_PATH, maximum=MAX_JSON_BYTES),
        "plan": base._identity(plan, expected_sha256=PLAN_SHA256),
        "case_bi4": base._identity(case_bi4, expected_sha256=base.CASE_BI4_SHA256),
        "case_xml": base._identity(case_xml, expected_sha256=base.CASE_XML_SHA256),
        "baseline_qc": base._identity(baseline_qc, maximum=MAX_JSON_BYTES, expected_sha256=base.BASELINE_QC_SHA256),
        "numerical_contract": base._identity(numerical_contract, maximum=MAX_JSON_BYTES, expected_sha256=CONTRACT_SHA256),
        "s4a_result": base._identity(s4a_result, maximum=MAX_JSON_BYTES, expected_sha256=S4A_RESULT_SHA256),
        "gpu_policy": base._identity(gpu_policy, maximum=MAX_JSON_BYTES, expected_sha256=GPU_POLICY_SHA256),
    }
    final, inputs["gpu_final"] = _safe_json(gpu_final, GPU_FINAL_SHA256)
    if not (
        final.get("status") == "PASS_GPU_STAGE4_RUN_RAW_OUTPUT"
        and final.get("run_id") == "u3_c1m_gpu_shift_none_from_0161_v2"
        and final.get("returncode") == 0
        and final.get("termination_reason") == "PROCESS_EXIT"
        and final.get("candidate_source_executed") is False
        and final.get("staged_candidate_executed") is True
        and final.get("network_used") is False
        and final.get("output_inventory", {}).get("canonical_sha256") == EXPECTED_GPU_INVENTORY_SHA256
    ):
        raise ShiftNoneQcError("GPU final receipt semantics differ")
    case = qc_v3.load_moving_case(
        case_bi4,
        case_xml,
        expected_bi4_sha256=base.CASE_BI4_SHA256,
        expected_xml_sha256=base.CASE_XML_SHA256,
    )
    baseline = base.analyze_continuation(
        baseline_root,
        case,
        candidate=False,
        expected_run_out_sha256=base.BASELINE_RUN_OUT_SHA256,
    )
    candidate = base.analyze_continuation(
        candidate_root,
        case,
        candidate=False,
        expected_run_out_sha256=GPU_RUN_OUT_SHA256,
    )
    baseline["shifting"] = _shifting_identity(baseline_root, "NoBound")
    candidate["shifting"] = _shifting_identity(candidate_root, "None")
    baseline["configuration_without_single_delta"] = _configuration_without_single_delta(
        baseline_root, "NoBound(-2,2.75,Full)"
    )
    candidate["configuration_without_single_delta"] = _configuration_without_single_delta(candidate_root, "None")
    baseline["initial_state_arrays"] = _initial_state_arrays(baseline_root)
    candidate["initial_state_arrays"] = _initial_state_arrays(candidate_root)
    baseline["backend"] = _backend(baseline_root)
    candidate["backend"] = _backend(candidate_root)
    comparison = base.compare_metric_vectors(baseline["metrics"], candidate["metrics"])
    pair_checks = {
        "same_initial_state_arrays": (
            baseline["initial_state_arrays"]["canonical_array_sha256"]
            == candidate["initial_state_arrays"]["canonical_array_sha256"]
        ),
        "same_non_shifting_numerical_configuration": (
            baseline["configuration_without_single_delta"]
            == candidate["configuration_without_single_delta"]
        ),
        "same_frame_and_time_window": (
            baseline["frame_count"], baseline["first_part"], baseline["last_part"]
        ) == (
            candidate["frame_count"], candidate["first_part"], candidate["last_part"]
        ) and abs(baseline["first_time_s"] - candidate["first_time_s"]) <= base.CONTRACT.physical_time_tolerance_s and abs(baseline["last_time_s"] - candidate["last_time_s"]) <= base.CONTRACT.physical_time_tolerance_s,
        "both_structural_checks_pass": (
            all(baseline["structural_checks"].values())
            and all(candidate["structural_checks"].values())
        ),
        "candidate_inventory_matches_gpu_receipt": (
            candidate["inventory"]["canonical_sha256"] == EXPECTED_GPU_INVENTORY_SHA256
        ),
        "same_backend": baseline["backend"] == candidate["backend"],
    }
    causal_isolation = all(value for key, value in pair_checks.items() if key != "same_backend") and pair_checks["same_backend"]
    selected = all(pair_checks.values()) and comparison["pass"]
    directional = (
        all(pair_checks[key] for key in pair_checks if key != "same_backend")
        and comparison["checks"]["no_metric_materially_regresses"]
        and comparison["checks"]["minimum_primary_metrics_meaningfully_improve"]
        and comparison["checks"]["primary_normalized_score_improves_by_at_least_5_percent"]
    )
    failed_checks = sorted(name for name, passed in comparison["checks"].items() if not passed)
    return {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": inputs,
        "contract": {
            "frozen_before_comparison_execution": True,
            "command_line_threshold_overrides_allowed": False,
            "window_part_indices": [161, 201],
            "window_time_s": [8.05, 10.05],
            "tail_window_s": 1.0,
            "metric_limits": dict(sorted(base.METRIC_LIMITS.items())),
            "primary_metrics": list(base.PRIMARY_METRICS),
            "single_variable": "SHIFTING_NOBOUND_TO_NONE",
            "baseline_shifting": "NoBound(-2,2.75,Full)",
            "candidate_shifting": "None",
            "other_numerical_parameters_changed": False,
            "backend_must_match_for_causal_single_variable_claim": True,
        },
        "baseline": baseline,
        "candidate": candidate,
        "pair_checks": pair_checks,
        "comparison": comparison,
        "verdict": {
            "status": "PASS_U3_SHIFT_NONE_NUMERICAL_CANDIDATE" if selected else "FAIL_U3_SHIFT_NONE_NUMERICAL_CANDIDATE",
            "candidate_selected": selected,
            "directionally_promising": directional,
            "causal_single_variable_isolation_pass": causal_isolation,
            "failed_acceptance_checks": failed_checks,
            "next": "S4C_SETTLED_STATE_MATRIX" if selected else "FRESH_GPU_CONTROL_AND_EXTENDED_SETTLING_REQUIRED",
            "development_only": True,
            "formal": False,
            "fidelity_validation_status": "SIM_ONLY_UNVALIDATED",
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "inputs_modified": False,
            "settled_state_claim_allowed": selected,
        },
    }


def _serialize(report: dict[str, Any]) -> bytes:
    data = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    if len(data) > MAX_JSON_BYTES:
        raise ShiftNoneQcError("result exceeds output bound")
    return data


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    metadata = os.lstat(path.parent)
    if not stat.S_ISDIR(metadata.st_mode):
        raise ShiftNoneQcError("result parent is not a real directory")
    if any(path.is_relative_to(Path(report[name]["root"])) for name in ("baseline", "candidate")):
        raise ShiftNoneQcError("result path is inside an input tree")
    data = _serialize(report)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ShiftNoneQcError("short result write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return base._identity(path, maximum=MAX_JSON_BYTES)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    for name in (
        "baseline-root", "candidate-root", "case-bi4", "case-xml", "plan",
        "baseline-qc", "numerical-contract", "s4a-result", "gpu-policy", "gpu-final",
    ):
        result.add_argument(f"--{name}", required=True, type=Path)
    result.add_argument("--output", type=Path)
    result.add_argument("--self-check", action="store_true")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        report = build_report(
            args.baseline_root, args.candidate_root, args.case_bi4, args.case_xml,
            args.plan, args.baseline_qc, args.numerical_contract, args.s4a_result,
            args.gpu_policy, args.gpu_final,
        )
        if args.self_check:
            print(json.dumps({
                "status": "PASS_U3_SHIFT_NONE_QC_V1_SELF_CHECK",
                "candidate_selected": report["verdict"]["candidate_selected"],
                "directionally_promising": report["verdict"]["directionally_promising"],
                "solver_executed": False,
            }, sort_keys=True))
            return 0
        if args.output is None:
            raise ShiftNoneQcError("--output is required unless --self-check is used")
        identity = write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["candidate_selected"] else 1
    except (ShiftNoneQcError, base.DdtRampQcError, qc_v3.QcError, bi4.Bi4FormatError, OSError, UnicodeError, ValueError) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
