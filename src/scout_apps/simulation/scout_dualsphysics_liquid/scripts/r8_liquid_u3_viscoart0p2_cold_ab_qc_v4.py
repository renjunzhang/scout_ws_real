#!/usr/bin/env python3
"""Inventory-bound representation revision for viscosity-0.2 cold-A/B QC.

The real cold-A output is 282,745,515 bytes: below the frozen runner limit of
402,653,184 bytes, but above the v3 QC schema's unrelated 256 MiB inventory
bound.  V4 aligns only that schema maximum with the already frozen run limit.
All branch identity, zero-baseline handling, BI4 parsing, 17 thresholds and
verdict logic remain unchanged.
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
import r8_liquid_u3_gpu_viscoart0p2_cold_ab_runner_v1 as runner
import r8_liquid_u3_solver_output_qc_v3 as qc_v3
import r8_liquid_u3_viscoart0p2_cold_ab_qc_v1 as prior
import r8_liquid_u3_viscoart0p2_cold_ab_qc_v2 as v2
import r8_liquid_u3_viscoart0p2_cold_ab_qc_v3 as v3


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p2_cold_ab_qc_v4.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p2_cold_ab_qc_v4.py"
V3_SCRIPT_SHA256 = "d00a6e0d74687fc38cddd0e7e0d8d0fc9f44bbe6914ad3f25c128adb9f6eebcf"
V3_SCHEMA_SHA256 = "6e5f15c84e7c5768559bcc787d20c8d5856a976405b1f1a97527c5f7c11bb7a4"
V3_TEST_SHA256 = "09897e7485b2ee11cf2b4302e3b453487273875515a6f96511447873ad304eea"
V3_SCRIPT_PATH = v3.SCRIPT_PATH
V3_SCHEMA_PATH = v3.SCHEMA_PATH
V3_TEST_PATH = v3.TEST_PATH
ORIGINAL_VALIDATE = prior.validate_report
INVENTORY_MAXIMUM_BYTES = runner.EXPECTED_OUTPUT["maximum_output_bytes"]


class ColdAbQcV4Error(v3.ColdAbQcV3Error):
    """Drifted v3 parent or invalid inventory-bound representation."""


def _schemas_differ_only_by_inventory_maximum() -> bool:
    old = legacy.read_json(V3_SCHEMA_PATH)
    new = legacy.read_json(SCHEMA_PATH)
    old.pop("$id")
    new.pop("$id")
    for document in (old, new):
        document["properties"]["schema_version"]["const"] = "REVISION"
        document["properties"]["document_type"]["const"] = "REVISION"
    old["$defs"]["inventory"]["properties"]["total_bytes"]["maximum"] = INVENTORY_MAXIMUM_BYTES
    return old == new


def configure() -> None:
    for path, expected in (
        (V3_SCRIPT_PATH, V3_SCRIPT_SHA256),
        (V3_SCHEMA_PATH, V3_SCHEMA_SHA256),
        (V3_TEST_PATH, V3_TEST_SHA256),
    ):
        if legacy.sha256_file(path, maximum=prior.MAX_JSON_BYTES) != expected:
            raise ColdAbQcV4Error(f"v3 QC parent identity drifted: {path}")
    v3.configure()
    prior.SCRIPT_PATH = SCRIPT_PATH
    prior.SCHEMA_PATH = SCHEMA_PATH
    prior.TEST_PATH = TEST_PATH


def validate_report(report: dict[str, Any]) -> None:
    configure()
    v3._validate_trajectory_representation(report.get("run", {}).get("trajectory", {}))
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
    prior._trajectory = v3._trajectory
    try:
        report = prior.build_report(policy_path)
    finally:
        prior.validate_report = saved_validate
        prior._trajectory = saved_trajectory
        if saved_run_id is missing:
            delattr(runner, "RUN_ID")
        else:
            runner.RUN_ID = saved_run_id
    report["schema_version"] = "r8-liquid-u3-viscoart0p2-cold-ab-qc-v4"
    report["document_type"] = "SMPCC_R8_LIQUID_U3_VISCOART0P2_COLD_AB_QC_V4"
    validate_report(report)
    return report


def static_self_check() -> dict[str, Any]:
    configure()
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    observed = schema["$defs"]["inventory"]["properties"]["total_bytes"]["maximum"]
    if observed != INVENTORY_MAXIMUM_BYTES:
        raise ColdAbQcV4Error("inventory maximum does not match the frozen runner output limit")
    if not _schemas_differ_only_by_inventory_maximum():
        raise ColdAbQcV4Error("v4 schema changed beyond inventory maximum and revision identity")
    return {
        "status": "PASS_U3_VISCOART0P2_COLD_AB_QC_V4_STATIC_SELF_CHECK",
        "single_representation_delta": "INVENTORY_MAX_256MIB_TO_FROZEN_384MIB",
        "inventory_maximum_bytes": INVENTORY_MAXIMUM_BYTES,
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
            raise ColdAbQcV4Error("--output is required")
        report = build_report(args.policy)
        identity = prior.write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_remediation_candidate_pass"] else 1
    except (
        ColdAbQcV4Error,
        v3.ColdAbQcV3Error,
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
