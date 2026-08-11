#!/usr/bin/env python3
"""Frozen-evidence tests for paired CFL=0.1 QC and phase-4 adjudication."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_u3_cfl0p1_sensitivity_qc_v1.py"
SCHEMA = ROOT / "schema/target_host_u3_cfl0p1_sensitivity_qc_v1.json"
BASELINE_ROOT = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_shift_none_extend_from_0201_v3.partial/output")
CANDIDATE_ROOT = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_shift_none_cfl0p1_from_0201_v4.partial/output")
PRIOR = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_shift_none_from_0161_v2.partial/output/data/Part_0201.bi4")
CASE_BI4 = Path("/home/zrj/scout_liquid_lab/cases/u3_c1m_gencase_v8_20260808T153753Z.partial/output/C1M_zero.bi4")
CASE_XML = CASE_BI4.with_suffix(".xml")
PLAN = Path("/home/zrj/scout_ws/docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md")
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_cfl0p1_sensitivity_contract_v1.json"
BASELINE_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_shift_none_extend_from_0201_v3.json"
CANDIDATE_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_shift_none_cfl0p1_from_0201_v4.json"
BASELINE_FINAL = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_shift_none_extend_from_0201_v3.final.json")
CANDIDATE_FINAL = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_shift_none_cfl0p1_from_0201_v4.final.json")
BASELINE_QC = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_shift_none_extend_from_0201_v3.qc_v1.json")


def load_module():
    scripts = str(ROOT / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location("u3_cfl0p1_sensitivity_qc_v1", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def assert_deep_closed(test: unittest.TestCase, value, path: str = "$") -> None:
    if isinstance(value, dict):
        value_type = value.get("type")
        if value_type == "object" or (isinstance(value_type, list) and "object" in value_type):
            test.assertIs(value.get("additionalProperties"), False, path)
        for key, item in value.items():
            assert_deep_closed(test, item, f"{path}/{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            assert_deep_closed(test, item, f"{path}/{index}")


class Cfl0p1SensitivityQcV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module()
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)
        cls.report = cls.module.build_report(
            BASELINE_ROOT,
            CANDIDATE_ROOT,
            PRIOR,
            CASE_BI4,
            CASE_XML,
            PLAN,
            CONTRACT,
            BASELINE_POLICY,
            CANDIDATE_POLICY,
            BASELINE_FINAL,
            CANDIDATE_FINAL,
            BASELINE_QC,
        )

    def test_actual_report_matches_deep_closed_schema(self) -> None:
        assert_deep_closed(self, self.schema)
        errors = list(Draft202012Validator(self.schema).iter_errors(self.report))
        self.assertEqual([], [f"{list(item.absolute_path)}: {item.message}" for item in errors])

    def test_exact_pair_identity_and_single_delta_pass(self) -> None:
        self.assertTrue(all(self.report["pair_checks"].values()))
        proof = self.report["contract"]["policy_delta_proof"]
        self.assertTrue(all(proof["checks"].values()))
        self.assertEqual(
            proof["baseline_solver_argv_canonical_sha256"],
            proof["candidate_without_cfl_canonical_sha256"],
        )
        self.assertEqual(self.report["baseline"]["actual_cfl"], 0.2)
        self.assertEqual(self.report["candidate"]["actual_cfl"], 0.1)
        self.assertEqual(self.report["baseline"]["backend"], self.report["candidate"]["backend"])

    def test_particle_id_pairing_proves_exact_restart_and_divergence(self) -> None:
        paired = self.report["paired_state_comparison"]
        self.assertTrue(paired["particle_id_alignment"])
        self.assertTrue(paired["initial_arrays_exact"])
        self.assertEqual(paired["frames_compared"], 201)
        self.assertEqual(paired["fluid_particle_count"], 6409)
        for quantity in ("position_m", "velocity_m_s", "density_kg_m3"):
            self.assertEqual(set(paired["initial"][quantity].values()), {0.0})
        self.assertAlmostEqual(paired["final"]["position_m"]["rms"], 0.0034417598433519373)
        self.assertAlmostEqual(paired["final"]["velocity_m_s"]["rms"], 0.0067361523864440205)
        self.assertAlmostEqual(paired["final"]["density_kg_m3"]["rms"], 0.19699577890835926)

    def test_cfl_reduction_improves_primary_score_but_fails_acceptance(self) -> None:
        comparison = self.report["metric_comparison"]
        self.assertAlmostEqual(comparison["candidate_to_baseline_primary_score_ratio"], 0.8794251120690693)
        self.assertEqual(comparison["primary_metrics_meaningfully_improved"], 4)
        self.assertTrue(comparison["checks"]["minimum_primary_metrics_meaningfully_improve"])
        self.assertTrue(comparison["checks"]["primary_normalized_score_improves_by_at_least_5_percent"])
        self.assertFalse(comparison["checks"]["all_candidate_absolute_limits_pass"])
        self.assertFalse(comparison["checks"]["no_metric_materially_regresses"])
        self.assertFalse(comparison["pass"])

    def test_phase4_fails_closed_on_nine_absolute_limits_and_rebound(self) -> None:
        verdict = self.report["verdict"]
        self.assertEqual(len(verdict["candidate_failed_absolute_metrics"]), 9)
        self.assertEqual(verdict["materially_regressed_metrics"], [
            "position_interframe_max_m_s",
            "surface_abs_drift_m_s",
            "surface_spread_m",
        ])
        self.assertTrue(verdict["baseline_numerical_rebound_detected"])
        self.assertTrue(verdict["candidate_numerical_rebound_detected"])
        self.assertFalse(verdict["candidate_settled_state"])
        self.assertFalse(verdict["sensitivity_candidate_selected"])
        self.assertEqual(verdict["stage4_adjudication_status"], "FAIL_PHASE4_LIQUID_STANDALONE_NUMERICAL_STABILITY")
        self.assertEqual(verdict["exact_blocker"], "CFL_0P1_FAILS_9_OF_17_ABSOLUTE_SETTLING_LIMITS_AND_REBOUND_PERSISTS")
        self.assertFalse(verdict["phase5_admitted"])

    def test_qc_has_no_execution_or_physical_fidelity_claim(self) -> None:
        verdict = self.report["verdict"]
        for name in (
            "formal",
            "physical_primary_eligible",
            "solver_executed_by_this_tool",
            "candidate_executed_by_this_tool",
            "gpu_exposed_by_this_tool",
            "network_used_by_this_tool",
            "inputs_modified",
        ):
            self.assertFalse(verdict[name], name)
        self.assertTrue(verdict["development_only"])
        self.assertTrue(verdict["phase4_execution_and_adjudication_complete"])

    def test_schema_rejects_unknown_fields_or_pass_escalation(self) -> None:
        mutations = (
            lambda value: value.update({"unreviewed": True}),
            lambda value: value["candidate"].update({"ignored_frames": [390, 391]}),
            lambda value: value["verdict"].update({"phase5_admitted": True, "production_pass": True}),
            lambda value: value["contract"].update({"candidate_cfl": 0.05}),
        )
        for mutate in mutations:
            tampered = copy.deepcopy(self.report)
            mutate(tampered)
            self.assertTrue(list(Draft202012Validator(self.schema).iter_errors(tampered)))

    def test_create_new_writer_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "result.json"
            identity = self.module.write_exclusive(output, self.report)
            self.assertEqual(identity["mode"], "0640")
            with self.assertRaises(FileExistsError):
                self.module.write_exclusive(output, self.report)

    def test_cli_exposes_no_threshold_or_solver_controls(self) -> None:
        destinations = {action.dest for action in self.module.parser()._actions}
        forbidden = ("threshold", "metric", "limit", "cfl", "solver", "gpu")
        self.assertFalse(any(any(token in name for token in forbidden) for name in destinations))


if __name__ == "__main__":
    unittest.main()
