#!/usr/bin/env python3
"""Representation-only QC revision for the viscosity-0.2 cold-A/B evidence.

The frozen v1 QC used the singular ``runner.RUN_ID`` while the A/B runner
intentionally exposes only ``RUN_IDS[role]``.  V2 fixes only that branch-aware
identity lookup.  All BI4 parsing, 17 numerical limits, structural checks,
verdict logic and O_EXCL output semantics remain inherited from v1.
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


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p2_cold_ab_qc_v2.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p2_cold_ab_qc_v2.py"
PRIOR_SCRIPT_SHA256 = "b122e6d0c720af22bc0a5b46f771816c21006bfaefc90f3bd2d9e24894365c29"
PRIOR_SCHEMA_SHA256 = "5d167b55a6be049c6d84d7bb4b46e1afb390c394dc2556f7e8b0a2daa5add9f4"
PRIOR_TEST_SHA256 = "0cf1ebd06e7fe922233dd946796d359b37717d703b5e4c1c8f20368c0347ca6b"
PRIOR_SCRIPT_PATH = prior.SCRIPT_PATH
PRIOR_SCHEMA_PATH = prior.SCHEMA_PATH
PRIOR_TEST_PATH = prior.TEST_PATH
ORIGINAL_VALIDATE = prior.validate_report


class ColdAbQcV2Error(prior.ColdAbQcError):
    """Drifted v1 parent, unknown A/B identity or invalid v2 report."""


def _expected_run_id(policy_path: Path) -> str:
    roles = [role for role, path in runner.POLICY_PATHS.items() if policy_path == path]
    if len(roles) != 1:
        raise ColdAbQcV2Error("policy path does not select exactly one frozen A/B role")
    return runner.RUN_IDS[roles[0]]


def _schemas_differ_only_by_revision_identity() -> bool:
    v1 = legacy.read_json(PRIOR_SCHEMA_PATH)
    v2 = legacy.read_json(SCHEMA_PATH)
    v1.pop("$id")
    v2.pop("$id")
    for document in (v1, v2):
        document["properties"]["schema_version"]["const"] = "REVISION"
        document["properties"]["document_type"]["const"] = "REVISION"
    return v1 == v2


def configure() -> None:
    for path, expected in (
        (PRIOR_SCRIPT_PATH, PRIOR_SCRIPT_SHA256),
        (PRIOR_SCHEMA_PATH, PRIOR_SCHEMA_SHA256),
        (PRIOR_TEST_PATH, PRIOR_TEST_SHA256),
    ):
        if legacy.sha256_file(path, maximum=prior.MAX_JSON_BYTES) != expected:
            raise ColdAbQcV2Error(f"v1 QC parent identity drifted: {path}")
    prior.SCRIPT_PATH = SCRIPT_PATH
    prior.SCHEMA_PATH = SCHEMA_PATH
    prior.TEST_PATH = TEST_PATH


def validate_report(report: dict[str, Any]) -> None:
    configure()
    ORIGINAL_VALIDATE(report)


def build_report(policy_path: Path) -> dict[str, Any]:
    configure()
    expected_run_id = _expected_run_id(policy_path)
    saved_validate = prior.validate_report
    missing = object()
    saved_run_id = getattr(runner, "RUN_ID", missing)
    runner.RUN_ID = expected_run_id
    prior.validate_report = lambda _report: None
    try:
        report = prior.build_report(policy_path)
    finally:
        prior.validate_report = saved_validate
        if saved_run_id is missing:
            delattr(runner, "RUN_ID")
        else:
            runner.RUN_ID = saved_run_id
    report["schema_version"] = "r8-liquid-u3-viscoart0p2-cold-ab-qc-v2"
    report["document_type"] = "SMPCC_R8_LIQUID_U3_VISCOART0P2_COLD_AB_QC_V2"
    validate_report(report)
    return report


def static_self_check() -> dict[str, Any]:
    configure()
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    if not _schemas_differ_only_by_revision_identity():
        raise ColdAbQcV2Error("v2 schema changed beyond revision identity")
    if _expected_run_id(runner.POLICY_PATHS["a"]) != runner.RUN_IDS["a"]:
        raise ColdAbQcV2Error("cold-A run-id mapping differs")
    if _expected_run_id(runner.POLICY_PATHS["b"]) != runner.RUN_IDS["b"]:
        raise ColdAbQcV2Error("cold-B run-id mapping differs")
    return {
        "status": "PASS_U3_VISCOART0P2_COLD_AB_QC_V2_STATIC_SELF_CHECK",
        "single_representation_delta": "RUN_ID_TO_RUN_IDS_BY_FROZEN_ROLE",
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
            raise ColdAbQcV2Error("--output is required")
        report = build_report(args.policy)
        identity = prior.write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_remediation_candidate_pass"] else 1
    except (
        ColdAbQcV2Error,
        prior.ColdAbQcError,
        runner.ColdAbRunError,
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
