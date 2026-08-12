#!/usr/bin/env python3
"""Static-contract tests for S6 synthetic-only visualization and delivery."""

from __future__ import annotations

import ast
import copy
import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
FIXTURE = ROOT / "tests/fixtures/s6_primary_golden_inputs_v2.json"
PLANNER_SCRIPT = SCRIPTS / "r8_liquid_s6_readonly_delivery_planner_v3.py"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s6_pure_analyzer_v2 as analyzer  # noqa: E402
import r8_liquid_s6_readonly_delivery_planner_v3 as planner  # noqa: E402


class S6ReadonlyDeliveryPlannerV3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
        selected_raw = analyzer.canonical_json(fixture["selected"])
        replay_raw = analyzer.canonical_json(fixture["replay"])
        cls.analysis = analyzer.analyze_bytes(
            selected_raw,
            replay_raw,
            expected_selected_sha256=analyzer.sha256_bytes(selected_raw),
            expected_replay_sha256=analyzer.sha256_bytes(replay_raw),
        )
        cls.analysis_raw = analyzer.canonical_json(cls.analysis)
        cls.analysis_sha = analyzer.sha256_bytes(cls.analysis_raw)

    def test_plan_freezes_three_shared_x_panels_palette_styles_and_pending_qa(self) -> None:
        plan = planner.build_plan(self.analysis_raw, expected_sha256=self.analysis_sha)
        _, schema = planner.load_schemas()
        Draft202012Validator(schema).validate(plan)
        visual = plan["visualization"]
        self.assertEqual(visual["layout"], "THREE_VERTICAL_SHARED_X_PANELS")
        self.assertTrue(visual["shared_x"])
        self.assertEqual(visual["axes_count"], 3)
        self.assertFalse(visual["dual_y_axes"])
        self.assertEqual(visual["palette"]["name"], "OKABE_ITO_COLORBLIND_SAFE")
        self.assertTrue(visual["line_styles"]["H_abs"] != visual["line_styles"]["H_crest"])
        self.assertTrue(visual["grayscale_qa"]["required"])
        self.assertFalse(visual["grayscale_qa"]["executed"])
        self.assertFalse(visual["grayscale_qa"]["pass_claimed"])
        self.assertFalse(visual["layout_qa"]["executed"])

    def test_media_keyframe_checksum_and_evidence_are_planned_not_materialized(self) -> None:
        plan = planner.build_plan(self.analysis_raw, expected_sha256=self.analysis_sha)
        delivery = plan["delivery"]
        self.assertIn("animation/primary.mp4", delivery["planned_inventory"])
        self.assertIn("animation/primary_preview.gif", delivery["planned_inventory"])
        self.assertIn("keyframes/primary_t000.png", delivery["planned_inventory"])
        self.assertIn("checksums.sha256", delivery["planned_inventory"])
        self.assertEqual(delivery["animation"]["fact_source"], "FINALIZED_SOLVER_FRAMES_ONLY")
        self.assertEqual(
            delivery["animation"]["keyframe_overlay"],
            ["container_coordinates", "time", "particle_classes", "liquid_surface_probes"],
        )
        self.assertFalse(delivery["animation"]["encoder_executed"])
        self.assertFalse(delivery["artifacts_materialized"])
        self.assertFalse(delivery["final_artifacts_allowed"])
        self.assertEqual(delivery["checksums"]["status"], "PLANNED_NOT_MATERIALIZED")
        self.assertEqual(delivery["checksums"]["entries"], [])
        self.assertEqual(plan["evidence"]["secondary_ledger"]["status"], "PLANNED_NOT_APPENDED")
        self.assertFalse(plan["evidence"]["secondary_ledger"]["append_performed"])

    def test_denominator_physical_pending_and_forbidden_claims_remain_frozen(self) -> None:
        plan = planner.build_plan(self.analysis_raw, expected_sha256=self.analysis_sha)
        self.assertEqual(plan["planned_denominator"], 1)
        self.assertTrue(plan["claims"]["physical_reference_pending"])
        self.assertFalse(plan["claims"]["paired_ranking"])
        self.assertFalse(plan["claims"]["cpu_selected_trajectory_comparison"])
        self.assertFalse(plan["claims"]["physical_fidelity_validated"])
        self.assertFalse(plan["claims"]["stage6_pass"])

        _, schema = planner.load_schemas()
        mutations = [
            lambda item: item.__setitem__("planned_denominator", 2),
            lambda item: item["claims"].__setitem__("paired_ranking", True),
            lambda item: item["claims"].__setitem__("cpu_selected_trajectory_comparison", True),
            lambda item: item["claims"].__setitem__("physical_reference_pending", False),
            lambda item: item["visualization"].__setitem__("dual_y_axes", True),
            lambda item: item["visualization"]["grayscale_qa"].__setitem__("pass_claimed", True),
            lambda item: item["delivery"].__setitem__("artifacts_materialized", True),
            lambda item: item["delivery"]["animation"].__setitem__("encoder_executed", True),
            lambda item: item["delivery"]["checksums"].__setitem__("entries", ["fake"]),
        ]
        for mutate in mutations:
            changed = copy.deepcopy(plan)
            mutate(changed)
            with self.subTest(mutation=mutate):
                self.assertTrue(list(Draft202012Validator(schema).iter_errors(changed)))

    def test_exact_analysis_hash_and_analysis_claim_promotion_fail_closed(self) -> None:
        with self.assertRaisesRegex(planner.S6DeliveryPlanError, "SHA-256"):
            planner.build_plan(self.analysis_raw, expected_sha256="0" * 64)
        changed = copy.deepcopy(self.analysis)
        changed["claims"]["paired_ranking"] = True
        raw = analyzer.canonical_json(changed)
        with self.assertRaises(planner.S6DeliveryPlanError):
            planner.build_plan(raw, expected_sha256=analyzer.sha256_bytes(raw))

    def test_schema_self_check_and_ast_have_no_render_write_or_execution_surface(self) -> None:
        schemas = planner.load_schemas()
        for schema in schemas:
            Draft202012Validator.check_schema(schema)
            planner.assert_deep_closed(schema)
        report = planner.self_check()
        self.assertEqual(
            report["status"],
            "S6_READONLY_DELIVERY_PLANNER_V3_SELF_CHECK_OK_NOT_MATERIALIZED",
        )
        self.assertFalse(report["real_bag_or_bi4_read"])
        self.assertFalse(report["renderer_executed"])
        self.assertFalse(report["animation_executed"])
        self.assertFalse(report["external_write_performed"])
        tree = ast.parse(PLANNER_SCRIPT.read_text(encoding="utf-8"))
        imports, forbidden = set(), []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                if node.func.attr in {
                    "write_text", "write_bytes", "mkdir", "open", "system", "popen",
                    "run", "Popen", "savefig", "twinx",
                }:
                    forbidden.append(node.func.attr)
        self.assertFalse(imports & {
            "matplotlib", "rosbag", "rospy", "socket", "requests", "subprocess",
            "imageio", "cv2",
        })
        self.assertEqual([], forbidden)


if __name__ == "__main__":
    unittest.main()
