#!/usr/bin/env python3
"""Static, negative, and frozen-input tests for DDT-ramp QC v1."""

from __future__ import annotations

import copy
import importlib.util
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_u3_ddt_ramp_qc_v1.py"
SCHEMA = ROOT / "schema/target_host_u3_ddt_ramp_qc_v1.json"
WORKSPACE = ROOT.parents[3]
PLAN = WORKSPACE / "docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md"
CASE_ROOT = Path("/home/zrj/scout_liquid_lab/cases/u3_c1m_gencase_v8_20260808T153753Z.partial/output")
BASELINE_ROOT = Path("/home/zrj/scout_liquid_lab/direct_runs/u3_c1m_settle_extension_from_0161_retry1_20260809T113242Z.partial/output")
DDT_ROOT = Path("/home/zrj/scout_liquid_lab/direct_runs/u3_c1m_ddtramp_0p2_to_0p1_from_0161_20260809T123534Z.partial/output")
BASELINE_QC = Path("/home/zrj/scout_liquid_lab/visualizations/u3_c1m_cold_a_plus_extension_20260809T121934Z_dynamic_v1/reports/extension_tail_qc.json")


def load_module():
    scripts = str(ROOT / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location("u3_ddt_ramp_qc_v1", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def assert_deep_closed(test: unittest.TestCase, value, path: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" or (
            isinstance(value.get("type"), list) and "object" in value["type"]
        ):
            test.assertIs(value.get("additionalProperties"), False, path)
        for key, item in value.items():
            assert_deep_closed(test, item, f"{path}/{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            assert_deep_closed(test, item, f"{path}/{index}")


class DdtRampQcV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module()
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)
        cls.validator = Draft202012Validator(cls.schema)
        cls.report = cls.module.build_report(
            BASELINE_ROOT,
            DDT_ROOT,
            CASE_ROOT / "C1M_zero.bi4",
            CASE_ROOT / "C1M_zero.xml",
            PLAN,
            BASELINE_QC,
        )

    def test_schema_is_valid_and_deep_closed(self) -> None:
        self.assertFalse(self.schema["additionalProperties"])
        assert_deep_closed(self, self.schema)

    def test_actual_frozen_report_matches_closed_schema(self) -> None:
        errors = sorted(self.validator.iter_errors(self.report), key=lambda item: list(item.path))
        self.assertEqual([], [f"{list(item.path)}: {item.message}" for item in errors])

    def test_frozen_parent_identities_and_single_delta_match(self) -> None:
        self.assertEqual(self.report["inputs"]["plan"]["sha256"], self.module.PLAN_SHA256)
        self.assertEqual(self.report["baseline"]["run_out_sha256"], self.module.BASELINE_RUN_OUT_SHA256)
        self.assertEqual(self.report["candidate"]["run_out_sha256"], self.module.DDT_RUN_OUT_SHA256)
        self.assertTrue(self.report["pair_checks"]["same_initial_checkpoint_sha256"])
        self.assertTrue(self.report["pair_checks"]["same_non_ddt_configuration"])
        self.assertEqual(self.report["baseline"]["ddt_parameters"], {"value": 0.1, "tramp_s": None, "tmax_s": None, "value_max": None})
        self.assertEqual(self.report["candidate"]["ddt_parameters"], {"value": 0.1, "tramp_s": 10.05, "tmax_s": 8.05, "value_max": 0.2})

    def test_both_exact_41_frame_windows_are_structurally_clean(self) -> None:
        for name in ("baseline", "candidate"):
            run = self.report[name]
            self.assertEqual(run["frame_count"], 41)
            self.assertEqual((run["first_part"], run["last_part"]), (161, 201))
            self.assertTrue(all(run["structural_checks"].values()))
            self.assertEqual(run["inventory"]["file_count"], 50)
            self.assertGreaterEqual(run["tail"]["coverage_s"], 0.95)

    def _vector(self, fraction: float = 0.8) -> dict[str, float]:
        return {name: limit * fraction for name, limit in self.module.METRIC_LIMITS.items()}

    def test_effect_contract_accepts_multi_metric_primary_improvement(self) -> None:
        baseline = self._vector()
        candidate = dict(baseline)
        candidate["speed_rms_m_s"] = self.module.METRIC_LIMITS["speed_rms_m_s"] * 0.7
        candidate["speed_p95_m_s"] = self.module.METRIC_LIMITS["speed_p95_m_s"] * 0.7
        result = self.module.compare_metric_vectors(baseline, candidate)
        self.assertTrue(result["pass"])
        self.assertGreaterEqual(result["primary_metrics_meaningfully_improved"], 2)

    def test_single_metric_improvement_is_rejected(self) -> None:
        baseline = self._vector()
        candidate = dict(baseline)
        candidate["speed_rms_m_s"] = self.module.METRIC_LIMITS["speed_rms_m_s"] * 0.5
        result = self.module.compare_metric_vectors(baseline, candidate)
        self.assertFalse(result["pass"])
        self.assertFalse(result["checks"]["minimum_primary_metrics_meaningfully_improve"])

    def test_material_regression_is_rejected_even_if_primary_metrics_improve(self) -> None:
        baseline = self._vector()
        candidate = dict(baseline)
        candidate["speed_rms_m_s"] *= 0.5
        candidate["speed_p95_m_s"] *= 0.5
        candidate["surface_spread_m"] = self.module.METRIC_LIMITS["surface_spread_m"] * 0.9
        result = self.module.compare_metric_vectors(baseline, candidate)
        self.assertFalse(result["pass"])
        self.assertFalse(result["checks"]["no_metric_materially_regresses"])

    def test_absolute_limit_failure_is_rejected(self) -> None:
        baseline = {name: limit * 1.2 for name, limit in self.module.METRIC_LIMITS.items()}
        candidate = {name: limit * 0.7 for name, limit in self.module.METRIC_LIMITS.items()}
        candidate["speed_max_m_s"] = self.module.METRIC_LIMITS["speed_max_m_s"] * 1.01
        result = self.module.compare_metric_vectors(baseline, candidate)
        self.assertFalse(result["pass"])
        self.assertFalse(result["checks"]["all_candidate_absolute_limits_pass"])

    def test_nonfinite_metric_is_rejected(self) -> None:
        baseline = self._vector()
        candidate = dict(baseline)
        candidate["speed_rms_m_s"] = math.nan
        with self.assertRaises(self.module.DdtRampQcError):
            self.module.compare_metric_vectors(baseline, candidate)

    def test_schema_rejects_unknown_result_field(self) -> None:
        tampered = copy.deepcopy(self.report)
        tampered["verdict"]["unreviewed"] = True
        self.assertTrue(list(self.validator.iter_errors(tampered)))

    def test_parser_has_no_threshold_or_window_overrides(self) -> None:
        parser = self.module._parser()
        destinations = {action.dest for action in parser._actions}
        forbidden = {"threshold", "tail_window", "start_time", "end_time", "metric_limit"}
        self.assertTrue(destinations.isdisjoint(forbidden))

    def test_create_new_writer_refuses_overwrite_and_uses_0640(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = root / "baseline"
            candidate = root / "candidate"
            baseline.mkdir()
            candidate.mkdir()
            output = root / "result.json"
            report = {"baseline": {"root": str(baseline)}, "candidate": {"root": str(candidate)}, "value": 1}
            identity = self.module.write_exclusive(output, report)
            self.assertEqual(identity["mode"], "0640")
            with self.assertRaises(FileExistsError):
                self.module.write_exclusive(output, report)


if __name__ == "__main__":
    unittest.main()
