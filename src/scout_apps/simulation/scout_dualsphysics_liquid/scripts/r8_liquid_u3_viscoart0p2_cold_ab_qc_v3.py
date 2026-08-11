#!/usr/bin/env python3
"""Zero-baseline-safe QC revision for fresh viscosity-0.2 cold-A/B runs.

V2 repaired the branch-specific run identity.  Real cold-A then proved that a
fresh, motionless Part_0000 has an exact zero speed, so the inherited trajectory
summary's ``final/minimum`` value is undefined.  V3 represents that one case as
``minimum_is_zero=true`` and a null ratio.  It does not change the 17 metrics,
their thresholds, BI4 parsing, structural gates, or pass/fail verdict logic.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_ddt_ramp_qc_v1 as metric_contract
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy
import r8_liquid_u3_gpu_viscoart0p2_cold_ab_runner_v1 as runner
import r8_liquid_u3_solver_output_qc_v3 as qc_v3
import r8_liquid_u3_viscoart0p2_cold_ab_qc_v1 as prior
import r8_liquid_u3_viscoart0p2_cold_ab_qc_v2 as v2


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p2_cold_ab_qc_v3.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p2_cold_ab_qc_v3.py"
V2_SCRIPT_SHA256 = "1cfca849a956b0667e5bd1ba123124d34608f6444c943352cfce6fa6feb1168d"
V2_SCHEMA_SHA256 = "64201ed06b7081f9b5c22cdd05f5746ee79c4779386ce43dd9c9f031e93e2689"
V2_TEST_SHA256 = "70bb6d188b1fb7c38419d4d65bcaa41b90af15b79cc17edb908666d9146972dd"
V2_SCRIPT_PATH = v2.SCRIPT_PATH
V2_SCHEMA_PATH = v2.SCHEMA_PATH
V2_TEST_PATH = v2.TEST_PATH
ORIGINAL_VALIDATE = prior.validate_report
ORIGINAL_TRAJECTORY = prior._trajectory


class ColdAbQcV3Error(v2.ColdAbQcV2Error):
    """Drifted v2 parent or inconsistent zero-baseline representation."""


def _trajectory(parts: list[dict[str, Any]]) -> dict[str, Any]:
    series = [(item["cpart"], item["time_s"], item["speed"]["rms_m_s"]) for item in parts]
    if not series or any(not math.isfinite(value) or value < 0 for _, _, value in series):
        raise ColdAbQcV3Error("speed trajectory is empty, negative or non-finite")
    minimum = min(series, key=lambda item: item[2])
    final = series[-1]
    minimum_is_zero = minimum[2] == 0.0
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
        "final_to_minimum_ratio": None if minimum_is_zero else final[2] / minimum[2],
        "rebound_after_minimum": final[2] > minimum[2] * 1.2,
        "minimum_is_zero": minimum_is_zero,
    }


def _validate_trajectory_representation(value: dict[str, Any]) -> None:
    minimum = value.get("minimum_speed_rms_m_s")
    ratio = value.get("final_to_minimum_ratio")
    flag = value.get("minimum_is_zero")
    if not isinstance(minimum, (int, float)) or not math.isfinite(float(minimum)) or minimum < 0:
        raise ColdAbQcV3Error("trajectory minimum is invalid")
    is_zero = float(minimum) == 0.0
    if flag is not is_zero:
        raise ColdAbQcV3Error("minimum_is_zero disagrees with the minimum")
    if is_zero and ratio is not None:
        raise ColdAbQcV3Error("zero minimum must have a null ratio")
    if not is_zero and (not isinstance(ratio, (int, float)) or not math.isfinite(float(ratio)) or ratio < 1):
        raise ColdAbQcV3Error("positive minimum must have a finite ratio >= 1")


def _schemas_differ_only_by_zero_baseline_representation() -> bool:
    old = legacy.read_json(V2_SCHEMA_PATH)
    new = legacy.read_json(SCHEMA_PATH)
    old.pop("$id")
    new.pop("$id")
    for document in (old, new):
        document["properties"]["schema_version"]["const"] = "REVISION"
        document["properties"]["document_type"]["const"] = "REVISION"
    trajectory = old["$defs"]["trajectory"]
    trajectory["required"].append("minimum_is_zero")
    trajectory["properties"]["minimum_speed_rms_m_s"] = {"type": "number", "minimum": 0}
    trajectory["properties"]["final_to_minimum_ratio"] = {
        "oneOf": [{"type": "number", "minimum": 1}, {"type": "null"}]
    }
    trajectory["properties"]["minimum_is_zero"] = {"type": "boolean"}
    return old == new


def configure() -> None:
    for path, expected in (
        (V2_SCRIPT_PATH, V2_SCRIPT_SHA256),
        (V2_SCHEMA_PATH, V2_SCHEMA_SHA256),
        (V2_TEST_PATH, V2_TEST_SHA256),
    ):
        if legacy.sha256_file(path, maximum=prior.MAX_JSON_BYTES) != expected:
            raise ColdAbQcV3Error(f"v2 QC parent identity drifted: {path}")
    v2.configure()
    prior.SCRIPT_PATH = SCRIPT_PATH
    prior.SCHEMA_PATH = SCHEMA_PATH
    prior.TEST_PATH = TEST_PATH


def validate_report(report: dict[str, Any]) -> None:
    configure()
    _validate_trajectory_representation(report.get("run", {}).get("trajectory", {}))
    ORIGINAL_VALIDATE(report)


def build_report(policy_path: Path) -> dict[str, Any]:
    configure()
    expected_run_id = v2._expected_run_id(policy_path)
    saved_validate = prior.validate_report
    saved_trajectory = prior._trajectory
    missing = object()
    saved_run_id = getattr(runner, "RUN_ID", missing)
    runner.RUN_ID = expected_run_id
    prior.validate_report = lambda _report: None
    prior._trajectory = _trajectory
    try:
        report = prior.build_report(policy_path)
    finally:
        prior.validate_report = saved_validate
        prior._trajectory = saved_trajectory
        if saved_run_id is missing:
            delattr(runner, "RUN_ID")
        else:
            runner.RUN_ID = saved_run_id
    report["schema_version"] = "r8-liquid-u3-viscoart0p2-cold-ab-qc-v3"
    report["document_type"] = "SMPCC_R8_LIQUID_U3_VISCOART0P2_COLD_AB_QC_V3"
    validate_report(report)
    return report


def static_self_check() -> dict[str, Any]:
    configure()
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    if not _schemas_differ_only_by_zero_baseline_representation():
        raise ColdAbQcV3Error("v3 schema changed beyond the zero-baseline representation")
    probe = _trajectory([
        {"cpart": 0, "time_s": 0.0, "speed": {"rms_m_s": 0.0}},
        {"cpart": 701, "time_s": 35.05, "speed": {"rms_m_s": 0.0005}},
    ])
    _validate_trajectory_representation(probe)
    return {
        "status": "PASS_U3_VISCOART0P2_COLD_AB_QC_V3_STATIC_SELF_CHECK",
        "single_representation_delta": "ZERO_MINIMUM_TO_EXPLICIT_NULL_RATIO",
        "metric_count": len(prior.METRIC_LIMITS),
        "threshold_changed": False,
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
            raise ColdAbQcV3Error("--output is required")
        report = build_report(args.policy)
        identity = prior.write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_remediation_candidate_pass"] else 1
    except (
        ColdAbQcV3Error,
        v2.ColdAbQcV2Error,
        prior.ColdAbQcError,
        runner.ColdAbRunError,
        legacy.Stage4RunError,
        metric_contract.DdtRampQcError,
        qc_v3.QcError,
        bi4.Bi4FormatError,
        OSError,
        UnicodeError,
        ValueError,
        ZeroDivisionError,
        subprocess.SubprocessError,
    ) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
