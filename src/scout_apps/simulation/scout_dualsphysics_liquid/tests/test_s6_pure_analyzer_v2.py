#!/usr/bin/env python3
"""Golden and fail-closed tests for the synthetic-only S6 v2 analyzer."""

from __future__ import annotations

import ast
import copy
import json
import math
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
FIXTURE = ROOT / "tests/fixtures/s6_primary_golden_inputs_v2.json"
EXPECTED = ROOT / "tests/fixtures/s6_primary_golden_expected_v2.json"
SCRIPT = SCRIPTS / "r8_liquid_s6_pure_analyzer_v2.py"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s6_pure_analyzer_v2 as analyzer  # noqa: E402


class S6PureAnalyzerV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
        cls.expected = json.loads(EXPECTED.read_text(encoding="utf-8"))

    def encode(self, fixture: dict[str, object] | None = None) -> tuple[bytes, bytes]:
        value = fixture or self.fixture
        return analyzer.canonical_json(value["selected"]), analyzer.canonical_json(value["replay"])

    def run_fixture(self, fixture: dict[str, object] | None = None) -> dict[str, object]:
        selected_raw, replay_raw = self.encode(fixture)
        return analyzer.analyze_bytes(
            selected_raw,
            replay_raw,
            expected_selected_sha256=analyzer.sha256_bytes(selected_raw),
            expected_replay_sha256=analyzer.sha256_bytes(replay_raw),
        )

    def assertAlmostMapping(self, observed: dict[str, float], expected: dict[str, float]) -> None:
        for key, value in expected.items():
            self.assertAlmostEqual(observed[key], value, places=10, msg=key)

    def test_golden_raw_probe_derivation_windows_spectral_and_claim_ceiling(self) -> None:
        selected_raw, replay_raw = self.encode()
        self.assertEqual(analyzer.sha256_bytes(selected_raw), self.expected["selected_document_sha256"])
        self.assertEqual(analyzer.sha256_bytes(replay_raw), self.expected["replay_document_sha256"])
        result = self.run_fixture()
        self.assertAlmostMapping(result["derived_surface_series"][1], self.expected["derived_t1"])
        self.assertEqual(result["qc"]["output_slot_count"], self.expected["qc"]["output_slot_count"])
        self.assertEqual(result["qc"]["invalid_probe_values"], 1)
        self.assertAlmostEqual(result["qc"]["invalid_probe_ratio"], 0.01)
        self.assertFalse(result["qc"]["precomputed_surface_columns_consumed"])
        for name, count in self.expected["window_slot_counts"].items():
            self.assertEqual(result["window_summaries"][name]["slot_count"], count)
        self.assertAlmostMapping(
            result["window_summaries"]["full_motion"]["H_abs"],
            self.expected["full_motion_H_abs"],
        )
        spectral = result["spectral_metrics"]
        self.assertAlmostEqual(spectral["H_crest"]["dominant_frequency_hz"], 0.24)
        self.assertAlmostEqual(spectral["H_abs"]["dominant_frequency_hz"], 0.48)
        self.assertAlmostEqual(spectral["H_abs"]["damping_rate_per_s"], 0.05, places=10)
        self.assertAlmostEqual(spectral["H_peak_to_peak"]["dominant_frequency_hz"], 0.48)
        for name in ("H_proxy", "H_modal"):
            self.assertGreaterEqual(result["model_comparisons"][name]["phase_error_rad"], -math.pi)
            self.assertLessEqual(result["model_comparisons"][name]["phase_error_rad"], math.pi)
        self.assertEqual(result["planned_denominator"], 1)
        self.assertTrue(result["claims"]["physical_reference_pending"])
        self.assertFalse(result["claims"]["paired_ranking"])
        self.assertFalse(result["claims"]["cpu_selected_trajectory_comparison"])
        self.assertFalse(result["claims"]["stage6_pass"])
        self.assertEqual(result["materialization"]["status"], "NOT_ADMITTED_SYNTHETIC_FIXTURE_ONLY")

    def test_phase_wrap_boundaries_and_golden_model_phase(self) -> None:
        self.assertAlmostEqual(analyzer._wrap_phase(3.5), 3.5 - 2 * math.pi)
        self.assertAlmostEqual(analyzer._wrap_phase(-3.5), -3.5 + 2 * math.pi)
        result = self.run_fixture()
        self.assertAlmostEqual(
            result["model_comparisons"]["H_proxy"]["phase_error_rad"],
            self.expected["model_comparisons"]["H_proxy_phase_error_rad"],
            places=10,
        )
        self.assertAlmostEqual(
            result["model_comparisons"]["H_modal"]["phase_error_rad"],
            self.expected["model_comparisons"]["H_modal_phase_error_rad"],
            places=10,
        )

    def test_provenance_origin_units_denominator_and_claim_promotion_fail_closed(self) -> None:
        mutations = [
            lambda item: item["replay"]["parents"]["source_bag"].__setitem__("sha256", "9" * 64),
            lambda item: item["selected"]["time_alignment"].__setitem__("odom_header_origin_ns", 2_000_000_000),
            lambda item: item["selected"]["series"]["H_proxy"].__setitem__("scale_to_comparison", 999.0),
            lambda item: item["replay"].__setitem__("planned_denominator", 2),
            lambda item: item["selected"]["claims"].__setitem__("paired_ranking", True),
            lambda item: item["replay"]["claims"].__setitem__("cpu_selected_trajectory_comparison", True),
            lambda item: item["selected"]["claims"].__setitem__("physical_reference_pending", False),
        ]
        for mutate in mutations:
            changed = copy.deepcopy(self.fixture)
            mutate(changed)
            with self.subTest(mutation=mutate), self.assertRaises(analyzer.S6V2AnalysisError):
                self.run_fixture(changed)

    def test_nonuniform_grid_and_forbidden_extrapolation_fail_closed(self) -> None:
        changed = copy.deepcopy(self.fixture)
        changed["replay"]["time_contract"]["output_slots_s"][2] = 2.25
        changed["replay"]["raw_probe_zsurf"][2]["t_s"] = 2.25
        grid_sha = analyzer.sha256_json(changed["replay"]["time_contract"]["output_slots_s"])
        changed["replay"]["time_contract"]["output_grid_sha256"] = grid_sha
        changed["replay"]["parents"]["s5b0_finalized_manifest"]["output_grid_sha256"] = grid_sha
        changed["selected"]["time_alignment"]["output_grid_sha256"] = grid_sha
        with self.assertRaisesRegex(analyzer.S6V2AnalysisError, "non-uniform"):
            self.run_fixture(changed)

        changed = copy.deepcopy(self.fixture)
        changed["selected"]["series"]["H_proxy"]["samples"].pop()
        changed["selected"]["time_alignment"]["overlap_end_s"] = 23
        with self.assertRaisesRegex(analyzer.S6V2AnalysisError, "extrapolation"):
            self.run_fixture(changed)

    def test_window_probe_cherry_pick_missing_and_precomputed_columns_fail_closed(self) -> None:
        changed = copy.deepcopy(self.fixture)
        changed["replay"]["time_contract"]["windows"]["first15"]["end_s"] = 14
        window_sha = analyzer.sha256_json(changed["replay"]["time_contract"]["windows"])
        changed["replay"]["time_contract"]["window_contract_sha256"] = window_sha
        changed["replay"]["parents"]["s5b0_finalized_manifest"]["window_contract_sha256"] = window_sha
        with self.assertRaisesRegex(analyzer.S6V2AnalysisError, "first15 end"):
            self.run_fixture(changed)

        changed = copy.deepcopy(self.fixture)
        changed["replay"]["probe_contract"]["probe_ids"] = ["p00", "p01", "p02"]
        changed["replay"]["probe_contract"]["probe_set_sha256"] = analyzer.sha256_json(
            changed["replay"]["probe_contract"]["probe_ids"]
        )
        with self.assertRaises(analyzer.S6V2AnalysisError):
            self.run_fixture(changed)

        changed = copy.deepcopy(self.fixture)
        for probe in ("p01", "p02", "p03"):
            changed["replay"]["raw_probe_zsurf"][7]["zsurf_mm"][probe] = None
        with self.assertRaisesRegex(analyzer.S6V2AnalysisError, "too few valid"):
            self.run_fixture(changed)

        changed = copy.deepcopy(self.fixture)
        changed["replay"]["raw_probe_zsurf"][0]["H_crest_mm"] = 999
        with self.assertRaises(analyzer.S6V2AnalysisError):
            self.run_fixture(changed)

    def test_exact_document_hash_is_mandatory(self) -> None:
        selected_raw, replay_raw = self.encode()
        with self.assertRaisesRegex(analyzer.S6V2AnalysisError, "SHA-256 differs"):
            analyzer.analyze_bytes(
                selected_raw + b" ",
                replay_raw,
                expected_selected_sha256=analyzer.sha256_bytes(selected_raw),
                expected_replay_sha256=analyzer.sha256_bytes(replay_raw),
            )

    def test_schemas_self_check_and_ast_are_closed_and_static_only(self) -> None:
        schemas = analyzer.load_schemas()
        for schema in schemas:
            Draft202012Validator.check_schema(schema)
            analyzer.assert_deep_closed(schema)
        report = analyzer.self_check()
        self.assertEqual(report["status"], "S6_V2_PURE_ANALYZER_STATIC_SELF_CHECK_OK_NOT_ADMITTED")
        self.assertFalse(report["real_bag_read"])
        self.assertFalse(report["real_bi4_read"])
        self.assertFalse(report["solver_or_gpu_executed"])
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports, forbidden = set(), []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                if node.func.attr in {"write_text", "write_bytes", "open", "system", "popen", "run", "Popen", "twinx"}:
                    forbidden.append(node.func.attr)
        self.assertFalse(imports & {"rosbag", "rospy", "socket", "requests", "subprocess"})
        self.assertEqual([], forbidden)


if __name__ == "__main__":
    unittest.main()
