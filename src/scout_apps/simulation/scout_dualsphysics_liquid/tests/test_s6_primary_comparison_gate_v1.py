#!/usr/bin/env python3
"""Static/synthetic tests for the S6 primary-only comparison contract."""

from __future__ import annotations

import ast
import copy
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SCRIPT = SCRIPTS / "r8_liquid_s6_primary_comparison_gate_v1.py"
FIXTURE = ROOT / "tests/fixtures/s6_primary_synthetic_series_v1.csv"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s6_primary_comparison_gate_v1 as gate  # noqa: E402


def fixture_rows() -> list[dict[str, float]]:
    with FIXTURE.open(encoding="utf-8", newline="") as stream:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def replay_fixture() -> tuple[bytes, dict[str, object]]:
    rows = fixture_rows()
    value = {
        "schema_version": "smpcc-r8-liquid-s5b0-finalized-replay-fixture-v1",
        "document_type": "SYNTHETIC_FINALIZED_REPLAY_FIXTURE_V1",
        "status": "S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY",
        "finalized": True, "synthetic_fixture": True,
        "attempt_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01", "planned_denominator": 1,
        "time_origin_ns": 1_000_000_000,
        "numeric_sources": ["finalized_bi4", "gauge_zsurf_csv", "slosh_height_csv"],
        "series": [{key: row[key] for key in ("t_s", "H_crest_mm", "H_abs_mm", "H_peak_to_peak_mm")} for row in rows],
        "claims": {"cpu_selected_trajectory_comparison": False, "paired_ranking": False,
                   "physical_fidelity_validated": False, "physical_reference_pending": True},
    }
    return gate.canonical_json(value), value


def selected_fixture() -> dict[str, object]:
    rows = fixture_rows()
    def samples(column: str, scale: float) -> list[dict[str, float | int]]:
        return [{"bag_record_t_ns": 1_000_000_000 + round(row["t_s"] * 1e9),
                 "value_native": row[column] / scale, "value_comparison_mm": row[column]} for row in rows]
    return {
        "planned_denominator": 1, "source": {"sha256": "1" * 64},
        "series": {"H_proxy": {"samples": samples("H_proxy_mm", 1000.0)},
                   "H_modal": {"samples": samples("H_modal_mm", 1.0)}},
        "claims": {"stage6_pass": False, "physical_reference_pending": True,
                   "physical_fidelity_validated": False, "paired_ranking": False,
                   "cpu_selected_trajectory_comparison": False},
    }


class PrimaryComparisonGateTests(unittest.TestCase):
    def test_closed_policy_result_metrics_and_planned_delivery(self) -> None:
        policy, schema = gate.load_contracts()
        raw, _ = replay_fixture()
        replay = gate.read_finalized_replay_bytes(raw, synthetic_fixture=True)
        result, aligned = gate.build_result(policy, raw, replay, selected_fixture())
        Draft202012Validator(schema).validate(result)
        self.assertEqual(set(result["surface_metrics"]), {"H_crest", "H_abs", "H_peak_to_peak"})
        for model in ("H_proxy", "H_modal"):
            self.assertEqual(set(result["model_comparisons"][model]), {
                "sample_count", "amplitude_error_mm", "frequency_error_hz", "damping_error_per_s",
                "phase_error_rad", "rmse_mm", "correlation"})
        self.assertEqual(len(aligned), 21)
        self.assertFalse(result["visualization"]["dual_y_axes"])
        self.assertFalse(result["delivery"]["artifacts_materialized"])
        self.assertIn("animation/primary.mp4", result["delivery"]["planned_inventory"])
        self.assertIn("animation/primary_preview.gif", result["delivery"]["planned_inventory"])
        self.assertIn("secondary_ledger_entry.json", result["delivery"]["planned_inventory"])
        bundle = gate.build_planned_bundle(result)
        self.assertEqual(set(bundle), {"comparison_manifest.json", "evidence_index.json", "secondary_ledger_entry.json", "checksums.sha256"})

    def test_preview_is_shared_x_three_panel_and_test_only(self) -> None:
        policy, _ = gate.load_contracts()
        raw, _ = replay_fixture()
        replay = gate.read_finalized_replay_bytes(raw, synthetic_fixture=True)
        _result, aligned = gate.build_result(policy, raw, replay, selected_fixture())
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "synthetic_preview.png"
            audit = gate.render_synthetic_preview(aligned, output)
            self.assertEqual(audit["axes_count"], 3)
            self.assertTrue(audit["shared_x"])
            self.assertFalse(audit["dual_y_axes"])
            self.assertGreater(output.stat().st_size, 10_000)
            with self.assertRaisesRegex(gate.S6ComparisonError, "fresh"):
                gate.render_synthetic_preview(aligned, output)

    def test_real_parent_denominator_sources_claims_and_nonfinite_fail_closed(self) -> None:
        raw, value = replay_fixture()
        with self.assertRaisesRegex(gate.S6ComparisonError, "NOT_ADMITTED"):
            gate.read_finalized_replay_bytes(raw, synthetic_fixture=False)
        mutations = [
            lambda item: item.__setitem__("planned_denominator", 2),
            lambda item: item.__setitem__("numeric_sources", ["finalized_bi4"]),
            lambda item: item["claims"].__setitem__("paired_ranking", True),
            lambda item: item["series"][0].__setitem__("H_crest_mm", float("nan")),
        ]
        for mutate in mutations:
            changed = copy.deepcopy(value)
            mutate(changed)
            encoded = json.dumps(changed, allow_nan=True).encode()
            with self.subTest(mutation=mutate), self.assertRaises(gate.S6ComparisonError):
                gate.read_finalized_replay_bytes(encoded, synthetic_fixture=True)

    def test_schema_rejects_dual_y_paired_cpu_physical_and_final_claims(self) -> None:
        policy, schema = gate.load_contracts()
        raw, _ = replay_fixture()
        result, _ = gate.build_result(policy, raw, gate.read_finalized_replay_bytes(raw, synthetic_fixture=True), selected_fixture())
        mutations = [
            lambda item: item["visualization"].__setitem__("dual_y_axes", True),
            lambda item: item["claims"].__setitem__("paired_ranking", True),
            lambda item: item["claims"].__setitem__("cpu_selected_trajectory_comparison", True),
            lambda item: item["claims"].__setitem__("physical_fidelity_validated", True),
            lambda item: item["claims"].__setitem__("stage6_pass", True),
            lambda item: item["delivery"].__setitem__("artifacts_materialized", True),
        ]
        for mutate in mutations:
            changed = copy.deepcopy(result)
            mutate(changed)
            self.assertTrue(list(Draft202012Validator(schema).iter_errors(changed)))

    def test_self_check_and_ast_are_static_only(self) -> None:
        result = gate.self_check()
        self.assertEqual(result["status"], "S6_STATIC_SELF_CHECK_OK_NOT_ADMITTED")
        self.assertFalse(result["real_bag_read"])
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = set()
        forbidden_calls = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr in {"system", "popen", "run", "Popen", "twinx"}:
                forbidden_calls.append(node.func.attr)
        self.assertFalse(imports & {"rosbag", "rospy", "socket", "requests", "subprocess"})
        self.assertEqual([], forbidden_calls)


if __name__ == "__main__":
    unittest.main()
