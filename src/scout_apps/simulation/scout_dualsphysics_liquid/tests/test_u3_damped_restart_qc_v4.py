#!/usr/bin/env python3
"""Closed-schema and negative tests for the v4 viscosity remediation QC."""

from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = PACKAGE_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import r8_liquid_u3_damped_restart_qc_v4 as qc


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


def delta_checks() -> dict[str, bool]:
    result = {
        f"{name}_unchanged": True
        for name in (
            "acceptance", "damping_contract", "gpu", "inputs",
            "result_boundary", "solver_invariants", "tools",
        )
    }
    for phase_name in qc.base.PHASE_KEYS:
        result[f"{phase_name}_argv_only_inserts_viscoart"] = True
        for name in ("case_xml_input", "expected_output", "limits", "purpose", "restart"):
            result[f"{phase_name}_{name}_unchanged"] = True
        result[f"{phase_name}_sandbox_boundary_unchanged"] = True
    return dict(sorted(result.items()))


def viscosity(name: str, value: float, phase_name: str) -> dict[str, object]:
    return {"name": name, "value": value, "run_out": identity(f"{phase_name}.Run.out")}


def valid_report(*, selected: bool = True) -> dict[str, object]:
    metrics = {name: 0.0 for name in qc.METRIC_NAMES}
    passes = {name: True for name in qc.METRIC_NAMES}
    failed: list[str] = []
    settling_status = "PASS_UNDAMPED_TAIL_ALL_17_LIMITS"
    verdict_status = "PASS_U3_VISCOART_DAMPED_INITIALIZATION_UNDAMPED_TAIL_SETTLED_CANDIDATE"
    blocker = "NONE"
    next_stage = "COLD_B_RESTART_EQUIVALENCE_THEN_PARITY_U4"
    if not selected:
        passes["speed_rms_m_s"] = False
        failed = ["speed_rms_m_s"]
        settling_status = "FAIL_UNDAMPED_TAIL_ABSOLUTE_SETTLING_LIMITS"
        verdict_status = "FAIL_U3_VISCOART_DAMPED_INITIALIZATION_UNDAMPED_TAIL_NOT_SETTLED"
        blocker = "UNDAMPED_TAIL_FAILS_1_OF_17_ABSOLUTE_SETTLING_LIMITS"
        next_stage = "STOP_AND_PRESERVE_REMEDIATION_EVIDENCE"
    argv_hashes = {
        name: {
            "prior_canonical_sha256": "1" * 64,
            "current_canonical_sha256": "2" * 64,
            "current_without_viscoart_canonical_sha256": "1" * 64,
        }
        for name in qc.base.PHASE_KEYS
    }
    return {
        "schema_version": "r8-liquid-u3-damped-restart-viscoart-qc-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_VISCOART_QC_V1",
        "captured_at_utc": "2026-08-11T05:23:42+00:00",
        "inputs": {name: identity(name) for name in (
            "policy", "runner", "qc_script", "qc_schema", "qc_tests",
            "prior_policy", "prior_qc", "init_receipt", "tail_receipt",
        )},
        "remediation_delta": {
            "parameter": "VISCOSITY_FORMULATION",
            "baseline": "LAMINAR_1E-6",
            "candidate": "UPSTREAM_DEFAULT_ARTIFICIAL_0P01",
            "inserted_argument": "-viscoart:0.01",
            "applied_to_phases": ["damped_init", "undamped_tail"],
            "other_numerical_parameters_changed": False,
            "checks": delta_checks(),
            "argv_canonical_sha256": argv_hashes,
        },
        "upstream_template_proof": {
            "commit": qc.runner.UPSTREAM_COMMIT,
            "path": qc.UPSTREAM_TEMPLATE,
            "blob": qc.runner.UPSTREAM_PARAMETER_TEMPLATE_BLOB,
            "content_sha256": qc.runner.UPSTREAM_PARAMETER_TEMPLATE_SHA256,
            "declared_visco_treatment": 1,
            "declared_visco": 0.01,
            "network_used": False,
        },
        "runtime_viscosity_proof": {
            "prior": {
                phase: viscosity("Laminar", 0.000001, f"prior-{phase}")
                for phase in qc.base.PHASE_KEYS
            },
            "candidate": {
                phase: viscosity("Artificial", 0.01, f"candidate-{phase}")
                for phase in qc.base.PHASE_KEYS
            },
            "candidate_init_and_tail_exact": True,
        },
        "inherited_qc_canonical_sha256": "3" * 64,
        "settling_evaluation": {
            "status": settling_status,
            "structural_checks_all_pass": True,
            "frame_count": 201,
            "first_time_s": 10.05,
            "last_time_s": 20.05,
            "metrics": metrics,
            "metric_limits": copy.deepcopy(qc.METRIC_LIMITS),
            "metric_absolute_pass": passes,
            "failed_absolute_metrics": failed,
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
            "status": verdict_status,
            "u3_remediation_candidate_pass": selected,
            "settled_state_frozen": False,
            "stage4_complete": False,
            "phase5_admitted": False,
            "exact_blocker": blocker,
            "next": next_stage,
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
        },
    }


class DampedRestartViscoartQcV4Tests(unittest.TestCase):
    def test_valid_pass_and_failure_closed_fixtures(self) -> None:
        qc.validate_report(valid_report(selected=True))
        qc.validate_report(valid_report(selected=False))

    def test_rejects_missing_or_extra_metric_key(self) -> None:
        for mutation in ("missing", "extra"):
            report = valid_report()
            if mutation == "missing":
                del report["settling_evaluation"]["metrics"]["speed_rms_m_s"]
            else:
                report["settling_evaluation"]["metrics"]["invented"] = 0.0
            with self.subTest(mutation=mutation), self.assertRaises(qc.ViscoartQcError):
                qc.validate_report(report)

    def test_rejects_relaxed_threshold(self) -> None:
        report = valid_report()
        report["settling_evaluation"]["metric_limits"]["speed_rms_m_s"] = 0.01
        with self.assertRaises(qc.ViscoartQcError):
            qc.validate_report(report)

    def test_rejects_failed_list_or_verdict_inconsistency(self) -> None:
        mutations = []
        missing_failed = valid_report(selected=False)
        missing_failed["settling_evaluation"]["failed_absolute_metrics"] = []
        mutations.append(missing_failed)
        overclaim = valid_report(selected=False)
        overclaim["verdict"]["u3_remediation_candidate_pass"] = True
        mutations.append(overclaim)
        for report in mutations:
            with self.subTest(report=report), self.assertRaises(qc.ViscoartQcError):
                qc.validate_report(report)

    def test_rejects_wrong_runtime_viscosity(self) -> None:
        for field, value in (("name", "Laminar"), ("value", 0.02)):
            report = valid_report()
            report["runtime_viscosity_proof"]["candidate"]["undamped_tail"][field] = value
            with self.subTest(field=field), self.assertRaises(qc.ViscoartQcError):
                qc.validate_report(report)

    def test_rejects_numerical_delta_or_parent_overclaim(self) -> None:
        mutations = []
        delta = valid_report()
        delta["remediation_delta"]["checks"]["cfl_unchanged"] = True
        mutations.append(delta)
        overclaim = valid_report()
        overclaim["verdict"]["stage4_complete"] = True
        mutations.append(overclaim)
        extra = valid_report()
        extra["verdict"]["extra"] = "forbidden"
        mutations.append(extra)
        for report in mutations:
            with self.subTest(report=report), self.assertRaises(qc.ViscoartQcError):
                qc.validate_report(report)


if __name__ == "__main__":
    unittest.main()
