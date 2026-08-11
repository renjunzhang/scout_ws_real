#!/usr/bin/env python3
"""Floating timestamp representation revision for the v10 extension QC.

The exact 21-frame final tail spans 0.9499972596 s because saved BI4 times are
floating solver timestamps.  V3 widens only the schema coverage lower bound
from 0.95 to 0.9499 s.  Parsing, structural checks, 17 thresholds and verdict
logic are inherited unchanged from v2.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_ddt_ramp_qc_v1 as metric_contract
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy
import r8_liquid_u3_gpu_viscoart0p2_settle_extension_runner_v2 as runner
import r8_liquid_u3_solver_output_qc_v3 as qc_v3
import r8_liquid_u3_viscoart0p2_settle_extension_qc_v2 as prior


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p2_settle_extension_qc_v3.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p2_settle_extension_qc_v3.py"
PRIOR_SCRIPT_PATH = prior.SCRIPT_PATH
PRIOR_SCHEMA_PATH = prior.SCHEMA_PATH
PRIOR_TEST_PATH = prior.TEST_PATH
PRIOR_SCRIPT_SHA256 = "ff620d4e04a517895b2117a19ebafd76f073552f10bc31397219a55e695efa3f"
PRIOR_SCHEMA_SHA256 = "adcb25518a2166e93ba4920c734b1d91015ef238f035caa0641321c0bd1fd9c3"
PRIOR_TEST_SHA256 = "a0b02507cbaeb9ea0be1f0d553c28b014ef451f975db99e8e14ed1a0d9ff9141"
ORIGINAL_VALIDATE = prior.validate_report
COVERAGE_MINIMUM_S = 0.9499


class SettleExtensionQcV3Error(prior.SettleExtensionQcError):
    """Drifted v2 parent or invalid floating-time representation."""


def configure() -> None:
    for path, expected in (
        (PRIOR_SCRIPT_PATH, PRIOR_SCRIPT_SHA256),
        (PRIOR_SCHEMA_PATH, PRIOR_SCHEMA_SHA256),
        (PRIOR_TEST_PATH, PRIOR_TEST_SHA256),
    ):
        if legacy.sha256_file(path, maximum=prior.MAX_JSON_BYTES) != expected:
            raise SettleExtensionQcV3Error(f"v2 QC parent identity drifted: {path}")
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
    report["schema_version"] = "r8-liquid-u3-viscoart0p2-settle-extension-qc-v3"
    report["document_type"] = "SMPCC_R8_LIQUID_U3_VISCOART0P2_SETTLE_EXTENSION_QC_V3"
    validate_report(report)
    return report


def _schemas_differ_only_by_coverage_minimum() -> bool:
    old = legacy.read_json(PRIOR_SCHEMA_PATH)
    new = legacy.read_json(SCHEMA_PATH)
    old.pop("$id")
    new.pop("$id")
    for document in (old, new):
        document["properties"]["schema_version"]["const"] = "REVISION"
        document["properties"]["document_type"]["const"] = "REVISION"
    old["$defs"]["tail"]["properties"]["coverage_s"]["minimum"] = COVERAGE_MINIMUM_S
    return old == new


def static_self_check() -> dict[str, Any]:
    configure()
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    coverage = schema["$defs"]["tail"]["properties"]["coverage_s"]
    if list(Draft202012Validator(coverage).iter_errors(0.9499972596497273)):
        raise SettleExtensionQcV3Error("observed floating coverage is not admitted")
    if not list(Draft202012Validator(coverage).iter_errors(0.9498)):
        raise SettleExtensionQcV3Error("coverage relaxation exceeds the frozen representation interval")
    if not _schemas_differ_only_by_coverage_minimum():
        raise SettleExtensionQcV3Error("v3 schema changed beyond coverage representation")
    return {
        "status": "PASS_U3_VISCOART0P2_SETTLE_EXTENSION_QC_V3_STATIC_SELF_CHECK",
        "single_representation_delta": "TAIL_COVERAGE_MIN_0P95_TO_0P9499",
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
            raise SettleExtensionQcV3Error("--output is required")
        report = build_report(args.policy)
        identity = prior.write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_remediation_candidate_pass"] else 1
    except (
        SettleExtensionQcV3Error,
        prior.SettleExtensionQcError,
        runner.SettleExtensionRunError,
        legacy.Stage4RunError,
        metric_contract.DdtRampQcError,
        qc_v3.QcError,
        bi4.Bi4FormatError,
        OSError,
        UnicodeError,
        ValueError,
        subprocess.SubprocessError,
    ) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
