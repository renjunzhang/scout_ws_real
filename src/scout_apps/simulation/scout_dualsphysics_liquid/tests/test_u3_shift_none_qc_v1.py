#!/usr/bin/env python3
"""Static, negative, and frozen-evidence tests for Shifting=None QC v1."""

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
SCRIPT = ROOT / "scripts/r8_liquid_u3_shift_none_qc_v1.py"
SCHEMA = ROOT / "schema/target_host_u3_shift_none_qc_v1.json"
BASELINE = Path("/home/zrj/scout_liquid_lab/direct_runs/u3_c1m_settle_extension_from_0161_retry1_20260809T113242Z.partial/output")
CANDIDATE = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_shift_none_from_0161_v2.partial/output")
CASE_BI4 = Path("/home/zrj/scout_liquid_lab/cases/u3_c1m_gencase_v8_20260808T153753Z.partial/output/C1M_zero.bi4")
CASE_XML = CASE_BI4.with_suffix(".xml")
PLAN = Path("/home/zrj/scout_ws/docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md")
BASELINE_QC = Path("/home/zrj/scout_liquid_lab/visualizations/u3_c1m_cold_a_plus_extension_20260809T121934Z_dynamic_v1/reports/extension_tail_qc.json")
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_numerical_contract_v1.json"
S4A = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_ddt_ramp_0p2_to_0p1_readonly_qc_v1.json")
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_shift_none_from_0161_v2.json"
FINAL = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_shift_none_from_0161_v2.final.json")


def load_module():
    scripts = str(ROOT / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location("u3_shift_none_qc_v1", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def assert_deep_closed(test: unittest.TestCase, value, path: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" or (isinstance(value.get("type"), list) and "object" in value["type"]):
            test.assertIs(value.get("additionalProperties"), False, path)
        for key, item in value.items():
            assert_deep_closed(test, item, f"{path}/{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            assert_deep_closed(test, item, f"{path}/{index}")


class ShiftNoneQcV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module()
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)
        cls.report = cls.module.build_report(
            BASELINE, CANDIDATE, CASE_BI4, CASE_XML, PLAN, BASELINE_QC,
            CONTRACT, S4A, POLICY, FINAL,
        )

    def test_actual_report_matches_deep_closed_schema(self) -> None:
        assert_deep_closed(self, self.schema)
        errors = list(Draft202012Validator(self.schema).iter_errors(self.report))
        self.assertEqual([], [f"{list(item.absolute_path)}: {item.message}" for item in errors])

    def test_structural_and_initial_state_invariants_are_exact(self) -> None:
        checks = self.report["pair_checks"]
        self.assertTrue(checks["same_initial_state_arrays"])
        self.assertTrue(checks["same_non_shifting_numerical_configuration"])
        self.assertTrue(checks["same_frame_and_time_window"])
        self.assertTrue(checks["both_structural_checks_pass"])
        self.assertTrue(checks["candidate_inventory_matches_gpu_receipt"])
        self.assertNotEqual(
            self.report["baseline"]["initial_state_arrays"]["part_file_sha256"],
            self.report["candidate"]["initial_state_arrays"]["part_file_sha256"],
        )
        self.assertEqual(
            self.report["baseline"]["initial_state_arrays"]["canonical_array_sha256"],
            self.report["candidate"]["initial_state_arrays"]["canonical_array_sha256"],
        )

    def test_directional_improvement_is_not_mislabeled_as_settled_pass(self) -> None:
        comparison = self.report["comparison"]
        verdict = self.report["verdict"]
        self.assertEqual(comparison["primary_metrics_meaningfully_improved"], 4)
        self.assertLess(comparison["candidate_to_baseline_primary_score_ratio"], 0.95)
        self.assertTrue(comparison["checks"]["no_metric_materially_regresses"])
        self.assertFalse(comparison["checks"]["all_candidate_absolute_limits_pass"])
        self.assertFalse(comparison["pass"])
        self.assertTrue(verdict["directionally_promising"])
        self.assertFalse(verdict["candidate_selected"])
        self.assertFalse(verdict["settled_state_claim_allowed"])
        self.assertEqual(verdict["failed_acceptance_checks"], ["all_candidate_absolute_limits_pass"])

    def test_backend_confound_is_explicit_and_causal_claim_is_rejected(self) -> None:
        self.assertFalse(self.report["pair_checks"]["same_backend"])
        self.assertFalse(self.report["verdict"]["causal_single_variable_isolation_pass"])
        self.assertEqual(self.report["baseline"]["backend"]["hardware"], "CPU")
        self.assertIn("Gpu", self.report["candidate"]["backend"]["hardware"])

    def test_schema_rejects_unknown_and_claim_escalation_fields(self) -> None:
        for mutate in (
            lambda value: value.update({"unreviewed": True}),
            lambda value: value["verdict"].update({"production_pass": True}),
            lambda value: value["candidate"].update({"raw_override": True}),
        ):
            tampered = copy.deepcopy(self.report)
            mutate(tampered)
            self.assertTrue(list(Draft202012Validator(self.schema).iter_errors(tampered)))

    def test_create_new_result_writer_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "result.json"
            identity = self.module.write_exclusive(output, self.report)
            self.assertEqual(identity["mode"], "0640")
            with self.assertRaises(FileExistsError):
                self.module.write_exclusive(output, self.report)

    def test_cli_has_no_threshold_or_metric_override(self) -> None:
        destinations = {action.dest for action in self.module.parser()._actions}
        self.assertFalse(any("threshold" in name or "limit" in name or "metric" in name for name in destinations))


if __name__ == "__main__":
    unittest.main()
