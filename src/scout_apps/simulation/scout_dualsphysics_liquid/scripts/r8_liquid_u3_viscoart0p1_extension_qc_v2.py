#!/usr/bin/env python3
"""Representation-only QC revision for the v6 extension evidence.

V1 correctly computed a 0.95 s final-tail window but its schema admitted a
start time only through 29.06 s.  With floating output timestamps the first
included frame is 29.100009... s.  This create-new revision widens only that
representation interval to 29.04..29.11 s; all inputs, 17 limits, BI4 parsing,
restart proof, verdict logic and output semantics remain inherited unchanged.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_ddt_ramp_qc_v1 as metric_contract
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy
import r8_liquid_u3_gpu_viscoart0p1_extension_runner_v1 as runner
import r8_liquid_u3_solver_output_qc_v3 as qc_v3
import r8_liquid_u3_viscoart0p1_extension_qc_v1 as prior


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p1_extension_qc_v2.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p1_extension_qc_v2.py"
PRIOR_SCRIPT_SHA256 = "83b561c016bcf28533dbff684c34c6fba0149024d67eab7289c463dfb375573b"
PRIOR_SCHEMA_SHA256 = "03c66b730e5e73fcafea0620c913f0bba3ccd3fe99f2fab19b826af3f525139b"
PRIOR_TEST_SHA256 = "c816fdbcb5cdaea11fa7d9dcfd7f4074ce5264634071f8b00b9e9a4dd1e1c2b6"
PRIOR_SCRIPT_PATH = prior.SCRIPT_PATH
PRIOR_SCHEMA_PATH = prior.SCHEMA_PATH
PRIOR_TEST_PATH = prior.TEST_PATH
ORIGINAL_VALIDATE = prior.validate_report


class ExtensionQcV2Error(prior.ExtensionQcError):
    """Drifted parent or invalid v2 representation."""


def configure() -> None:
    parents = (
        (PRIOR_SCRIPT_PATH, PRIOR_SCRIPT_SHA256),
        (PRIOR_SCHEMA_PATH, PRIOR_SCHEMA_SHA256),
        (PRIOR_TEST_PATH, PRIOR_TEST_SHA256),
    )
    for path, expected in parents:
        if legacy.sha256_file(path, maximum=prior.MAX_JSON_BYTES) != expected:
            raise ExtensionQcV2Error(f"v1 QC parent identity drifted: {path}")
    prior.SCRIPT_PATH = SCRIPT_PATH
    prior.SCHEMA_PATH = SCHEMA_PATH
    prior.TEST_PATH = TEST_PATH


def validate_report(report: dict[str, Any]) -> None:
    configure()
    ORIGINAL_VALIDATE(report)


def build_report(policy_path: Path) -> dict[str, Any]:
    configure()
    saved_validate = prior.validate_report
    prior.validate_report = lambda _report: None
    try:
        report = prior.build_report(policy_path)
    finally:
        prior.validate_report = saved_validate
    report["schema_version"] = "r8-liquid-u3-viscoart0p1-extension-qc-v2"
    report["document_type"] = "SMPCC_R8_LIQUID_U3_VISCOART0P1_EXTENSION_QC_V2"
    validate_report(report)
    return report


def static_self_check() -> dict[str, Any]:
    configure()
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    tail_schema = schema["$defs"]["tail"]
    probe = {
        "part_count": 20,
        "start_time_s": 29.100009555084092,
        "end_time_s": 30.050009,
        "coverage_s": 0.95,
        "surface_valid_part_count": 20,
    }
    errors = list(Draft202012Validator(tail_schema).iter_errors(probe))
    if errors:
        raise ExtensionQcV2Error(f"v2 tail representation is not admitted: {errors[0].message}")
    return {
        "status": "PASS_U3_VISCOART0P1_EXTENSION_QC_V2_STATIC_SELF_CHECK",
        "single_representation_delta": "TAIL_START_MAX_29P06_TO_29P11",
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
            raise ExtensionQcV2Error("--output is required")
        report = build_report(args.policy)
        identity = prior.write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_remediation_candidate_pass"] else 1
    except (ExtensionQcV2Error, prior.ExtensionQcError, runner.ExtensionRunError, legacy.Stage4RunError, metric_contract.DdtRampQcError, qc_v3.QcError, bi4.Bi4FormatError, OSError, UnicodeError, ValueError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
