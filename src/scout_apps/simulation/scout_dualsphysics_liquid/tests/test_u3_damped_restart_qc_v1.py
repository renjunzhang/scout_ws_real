#!/usr/bin/env python3
"""Closed-schema and negative tests for damped-restart Stage-4 QC."""

from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = PACKAGE_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import r8_liquid_u3_damped_restart_qc_v1 as qc
import r8_liquid_u3_ddt_ramp_qc_v1 as metric_contract


def identity(name: str) -> dict[str, object]:
    return {
        "path": f"/evidence/{name}",
        "inode": 1,
        "mode": "0440",
        "size_bytes": 1,
        "nlink": 1,
        "uid": 1000,
        "gid": 1000,
        "sha256": "0" * 64,
    }


def inventory(parts: int, first: int, last: int) -> dict[str, object]:
    return {
        "file_count": parts + 9,
        "part_file_count": parts,
        "part_first": first,
        "part_last": last,
        "total_bytes": 1,
        "canonical_sha256": "1" * 64,
    }


def valid_report() -> dict[str, object]:
    metrics = {name: 0.0 for name in metric_contract.METRIC_LIMITS}
    limits = dict(sorted(metric_contract.METRIC_LIMITS.items()))
    metric_pass = {name: True for name in metric_contract.METRIC_LIMITS}
    return {
        "schema_version": "r8-liquid-u3-damped-restart-qc-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_QC_V1",
        "captured_at_utc": "2026-08-11T04:00:00+00:00",
        "inputs": {name: identity(name) for name in (
            "policy", "runner", "qc_script", "qc_schema", "qc_tests",
            "init_receipt", "tail_receipt", "checkpoint",
        )},
        "intervention_proof": {
            "single_intervention": "INITIALIZATION_ONLY_STANDARD_DAMPING_PLANE",
            "case_semantics_equal_after_removing_special": True,
            "damping_present_in_initialization_xml": True,
            "damping_absent_from_validation_xml": True,
            "damping_reported_by_initialization_solver": True,
            "damping_not_reported_by_validation_solver": True,
            "checkpoint_transfer_exact": True,
            "cfl_shifting_ddt_dp_unchanged": True,
            "evaluation_uses_only_undamped_tail": True,
        },
        "initialization": {
            "status": "PASS_DAMPED_INITIALIZATION_RAW_AND_CHECKPOINT_QC",
            "run_id": "u3_damped_init",
            "returncode": 0,
            "elapsed_seconds": 1.0,
            "inventory": inventory(202, 0, 201),
            "checkpoint_state": {
                "part": 201,
                "time_s": 10.05,
                "particle_count": 9078,
                "moving_count": 2669,
                "fluid_count": 6409,
                "nout": 0,
                "all_finite": True,
                "domain_and_density_valid": True,
            },
        },
        "undamped_tail": {
            "status": "PASS_UNDAMPED_TAIL_ALL_17_LIMITS",
            "run_id": "u3_undamped_tail",
            "returncode": 0,
            "elapsed_seconds": 1.0,
            "inventory": inventory(201, 201, 401),
            "frame_count": 201,
            "first_time_s": 10.05,
            "last_time_s": 20.05,
            "actual_cfl": 0.1,
            "shifting": "None",
            "ddt": {},
            "dp_m": 0.002,
            "structural_checks_all_pass": True,
            "metrics": metrics,
            "metric_limits": limits,
            "metric_absolute_pass": metric_pass,
            "failed_absolute_metrics": [],
            "trajectory": {
                "minimum_time_s": 10.05,
                "minimum_speed_rms_m_s": 0.0,
                "final_time_s": 20.05,
                "final_speed_rms_m_s": 0.0,
                "final_to_minimum_ratio": 0.0,
                "rebound_after_minimum": False,
            },
        },
        "verdict": {
            "status": "PASS_U3_DAMPED_INITIALIZATION_UNDAMPED_TAIL_SETTLED_CANDIDATE",
            "u3_remediation_candidate_pass": True,
            "settled_state_frozen": False,
            "stage4_complete": False,
            "phase5_admitted": False,
            "exact_blocker": "NONE",
            "next": "COLD_B_RESTART_EQUIVALENCE_THEN_PARITY_U4",
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
        },
    }


class DampedRestartQcTests(unittest.TestCase):
    def test_valid_closed_fixture(self) -> None:
        qc.validate_report(valid_report())

    def test_rejects_missing_metric(self) -> None:
        report = valid_report()
        del report["undamped_tail"]["metrics"]["speed_rms_m_s"]
        with self.assertRaises(qc.DampedRestartQcError):
            qc.validate_report(report)

    def test_rejects_relaxed_limit_vector(self) -> None:
        report = valid_report()
        report["undamped_tail"]["metric_limits"]["speed_rms_m_s"] = 0.01
        with self.assertRaises(qc.DampedRestartQcError):
            qc.validate_report(report)

    def test_rejects_stage4_or_phase5_overclaim(self) -> None:
        for field in ("stage4_complete", "phase5_admitted"):
            report = valid_report()
            report["verdict"][field] = True
            with self.assertRaises(qc.DampedRestartQcError):
                qc.validate_report(report)

    def test_rejects_damping_in_validation(self) -> None:
        report = valid_report()
        report["intervention_proof"]["damping_not_reported_by_validation_solver"] = False
        with self.assertRaises(qc.DampedRestartQcError):
            qc.validate_report(report)

    def test_rejects_extra_property(self) -> None:
        report = valid_report()
        report["verdict"]["extra"] = "forbidden"
        with self.assertRaises(qc.DampedRestartQcError):
            qc.validate_report(report)


if __name__ == "__main__":
    unittest.main()
