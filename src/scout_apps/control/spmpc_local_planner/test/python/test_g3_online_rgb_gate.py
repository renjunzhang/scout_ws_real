#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ANALYSIS_DIR = PACKAGE_ROOT / "tools" / "analysis"


def load_module(name):
    path = ANALYSIS_DIR / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VALIDATOR = load_module("validate_g3_online_rgb_trial")
ANALYZER = load_module("analyze_g3_w5_vs_bsmooth")


def reports_from_pairs(pairs):
    reports = {}
    for row, (block, position, condition) in ANALYZER.EXPECTED_ROWS.items():
        bsmooth, w5 = pairs[block]
        value = bsmooth if condition == "Bsmooth" else w5
        reports[row] = (
            Path("row_{}.json".format(row)),
            {
                "row": row,
                "block": block,
                "position": position,
                "condition": condition,
                "status": "PASS",
                "online_rgb": {"h_vis_p95_mm": value},
            },
        )
    return reports


class G3OnlineRgbGateTest(unittest.TestCase):
    def test_causal_median_uses_no_future_sample(self):
        output = VALIDATOR.causal_rolling_median(
            [(0.0, 1.0), (0.1, 100.0), (0.2, 3.0)], window=5, max_gap=0.35
        )
        self.assertEqual(output[0], (0.0, 1.0))
        self.assertEqual(output[1], (0.1, 50.5))
        self.assertEqual(output[2], (0.2, 3.0))

    def test_causal_median_resets_after_gap(self):
        output = VALIDATOR.causal_rolling_median(
            [(0.0, 1.0), (0.1, 3.0), (1.0, 9.0)], window=5, max_gap=0.35
        )
        self.assertEqual(output[-1], (1.0, 9.0))

    def test_robot_only_delay_application_contract(self):
        self.assertIsNone(
            VALIDATOR.application_requirement_failure(
                "robot-delay-compensation", [1.0] * 98 + [0.0] * 2, "true"
            )
        )
        self.assertIsNone(
            VALIDATOR.application_requirement_failure(
                "liquid-delay-compensation", [0.0] * 100, "false"
            )
        )

    def test_required_split_delay_flag_cannot_silently_use_legacy_bag(self):
        self.assertFalse(VALIDATOR.finite(VALIDATOR.application_fraction([])))
        failure = VALIDATOR.application_requirement_failure(
            "robot-delay-compensation", [], "true"
        )
        self.assertIn("no application flags", failure)

    def test_split_delay_application_gate_rejects_wrong_direction(self):
        self.assertIsNotNone(
            VALIDATOR.application_requirement_failure(
                "robot-delay-compensation", [1.0] * 97 + [0.0] * 3, "true"
            )
        )
        self.assertIsNotNone(
            VALIDATOR.application_requirement_failure(
                "liquid-delay-compensation", [0.0] * 97 + [1.0] * 3, "false"
            )
        )

    def test_four_consistent_pairs_pass(self):
        reports = reports_from_pairs(
            {block: (1.0, 0.8) for block in ("01", "02", "03", "04")}
        )
        blocks, aggregate, failures = ANALYZER.evaluate_blocks(reports, 0.10, 3)
        self.assertEqual(len(blocks), 4)
        self.assertAlmostEqual(aggregate["mean_delta_mm"], 0.2)
        self.assertTrue(aggregate["effect_pass"])
        self.assertTrue(aggregate["direction_pass"])
        self.assertTrue(aggregate["single_block_dominance_pass"])
        self.assertTrue(aggregate["leave_one_block_out_pass"])
        self.assertEqual(failures, [])

    def test_single_block_dominance_fails(self):
        reports = reports_from_pairs(
            {
                "01": (1.0, 0.4),
                "02": (1.0, 1.1),
                "03": (1.0, 1.1),
                "04": (1.0, 1.1),
            }
        )
        _, aggregate, failures = ANALYZER.evaluate_blocks(reports, 0.01, 1)
        self.assertFalse(aggregate["single_block_dominance_pass"])
        self.assertFalse(aggregate["leave_one_block_out_pass"])
        self.assertTrue(failures)


if __name__ == "__main__":
    unittest.main()
