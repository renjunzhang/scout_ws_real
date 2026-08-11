#!/usr/bin/env python3
"""Static and frozen-evidence tests for extended-settling QC v1."""

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
SCRIPT = ROOT / "scripts/r8_liquid_u3_extended_settling_qc_v1.py"
SCHEMA = ROOT / "schema/target_host_u3_extended_settling_qc_v1.json"
RUN = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_shift_none_extend_from_0201_v3.partial/output")
PRIOR = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_shift_none_from_0161_v2.partial/output/data/Part_0201.bi4")
CASE_BI4 = Path("/home/zrj/scout_liquid_lab/cases/u3_c1m_gencase_v8_20260808T153753Z.partial/output/C1M_zero.bi4")
CASE_XML = CASE_BI4.with_suffix(".xml")
PLAN = Path("/home/zrj/scout_ws/docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md")
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_extended_settling_contract_v1.json"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_shift_none_extend_from_0201_v3.json"
FINAL = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_shift_none_extend_from_0201_v3.final.json")


def load_module():
    scripts = str(ROOT / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location("u3_extended_settling_qc_v1", SCRIPT)
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


class ExtendedSettlingQcV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module()
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)
        cls.report = cls.module.build_report(RUN, PRIOR, CASE_BI4, CASE_XML, PLAN, CONTRACT, POLICY, FINAL)

    def test_actual_report_matches_deep_closed_schema(self) -> None:
        assert_deep_closed(self, self.schema)
        errors = list(Draft202012Validator(self.schema).iter_errors(self.report))
        self.assertEqual([], [f"{list(item.absolute_path)}: {item.message}" for item in errors])

    def test_full_inventory_and_restart_state_are_exact(self) -> None:
        run = self.report["run"]
        self.assertEqual(run["inventory"]["file_count"], 210)
        self.assertTrue(run["restart_state_arrays_match"])
        self.assertNotEqual(run["restart_input_state"]["part_file_sha256"], run["restart_output_state"]["part_file_sha256"])
        self.assertEqual(run["restart_input_state"]["canonical_array_sha256"], run["restart_output_state"]["canonical_array_sha256"])
        self.assertTrue(all(run["structural_checks"].values()))

    def test_numerical_rebound_and_absolute_failure_are_not_mislabeled(self) -> None:
        run = self.report["run"]
        verdict = self.report["verdict"]
        trajectory = run["trajectory"]
        self.assertTrue(trajectory["rebound_after_minimum"])
        self.assertGreater(trajectory["final_to_minimum_ratio"], 1.2)
        self.assertLess(trajectory["minimum"]["time_s"], 14.0)
        self.assertFalse(verdict["settled_state"])
        self.assertEqual(verdict["status"], "FAIL_U3_EXTENDED_SETTLING_NUMERICAL_STABILITY")
        self.assertIn("speed_rms_m_s", verdict["failed_metrics"])
        self.assertIn("specific_kinetic_energy_j_kg", verdict["failed_metrics"])
        self.assertFalse(run["metric_absolute_pass"]["position_net_rms_m_s"])

    def test_density_surface_and_particle_integrity_remain_clean(self) -> None:
        passes = self.report["run"]["metric_absolute_pass"]
        for name in (
            "density_mean_relative_bias", "density_mean_relative_range",
            "density_p01_p99_relative_deviation", "surface_abs_bias_m",
            "surface_abs_drift_m_s", "surface_spread_m", "surface_temporal_range_m",
        ):
            self.assertTrue(passes[name], name)
        self.assertTrue(self.report["run"]["structural_checks"]["zero_nout"])

    def test_schema_rejects_unknown_or_pass_escalation_fields(self) -> None:
        for mutate in (
            lambda value: value.update({"unreviewed": True}),
            lambda value: value["run"].update({"ignored_frames": [390, 391]}),
            lambda value: value["verdict"].update({"production_pass": True}),
        ):
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

    def test_cli_exposes_no_threshold_override(self) -> None:
        destinations = {action.dest for action in self.module.parser()._actions}
        self.assertFalse(any("threshold" in name or "metric" in name or "limit" in name for name in destinations))


if __name__ == "__main__":
    unittest.main()
