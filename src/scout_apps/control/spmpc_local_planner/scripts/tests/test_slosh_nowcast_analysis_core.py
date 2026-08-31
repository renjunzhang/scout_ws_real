#!/usr/bin/env python3

import importlib.util
import math
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "analysis/slosh_nowcast_analysis_core.py"
SPEC = importlib.util.spec_from_file_location("slosh_nowcast_analysis_core", MODULE_PATH)
CORE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CORE)


def trial(i0_mae, i1_mae, coverage=0.99):
    return {
        "methods": {
            method: {
                "mae_mm": i0_mae if method != "I1" else i1_mae,
                "coverage": coverage,
            }
            for method in CORE.METHODS
        }
    }


class SloshNowcastAnalysisCoreTest(unittest.TestCase):
    def test_centered_median_preserves_invalid_center(self):
        rows = [(0.0, 1.0), (1.0, 100.0), (2.0, None), (3.0, 3.0), (4.0, 5.0)]
        output = CORE.centered_rolling_median(rows, 5)
        self.assertEqual(output[0][1], 50.5)
        self.assertIsNone(output[2][1])
        self.assertEqual(output[3][1], 5.0)

    def test_centered_median_rejects_even_window(self):
        with self.assertRaises(ValueError):
            CORE.centered_rolling_median([(0.0, 1.0)], 4)

    def test_interpolation_rejects_large_gap_and_extrapolation(self):
        self.assertAlmostEqual(CORE.interpolate_at(0.5, [0.0, 1.0], [2.0, 4.0], 1.0), 3.0)
        self.assertIsNone(CORE.interpolate_at(0.5, [0.0, 1.0], [2.0, 4.0], 0.9))
        self.assertIsNone(CORE.interpolate_at(-0.1, [0.0, 1.0], [2.0, 4.0], 1.0))

    def test_motion_window_includes_frozen_tail(self):
        window = CORE.motion_window([(1.0, 0.0, 0.0), (2.0, 0.04, 0.0), (5.0, 0.0, -0.1)], tail_sec=3.0)
        self.assertEqual(window, (2.0, 8.0))

    def test_no_motion_is_not_silently_accepted(self):
        with self.assertRaises(ValueError):
            CORE.motion_window([(1.0, 0.0, 0.0)])

    def test_metrics_use_fixed_zero_lag_and_unit_scale(self):
        visual = [(0.0, 1.0), (1.0, 3.0), (2.0, 5.0)]
        estimate = [(0.0, 2.0), (1.0, 4.0), (2.0, 6.0)]
        metrics = CORE.align_and_score(visual, estimate, max_gap_sec=1.0)
        self.assertEqual(metrics["aligned_samples"], 3)
        self.assertEqual(metrics["coverage"], 1.0)
        self.assertEqual(metrics["mae_mm"], 1.0)
        self.assertEqual(metrics["signed_bias_mm"], 1.0)
        self.assertEqual(metrics["peak_timing_error_sec"], 0.0)

    def test_three_held_out_bags_can_pass(self):
        result = CORE.development_decision(
            [trial(10.0, 8.0), trial(9.0, 8.5), trial(11.0, 9.0)],
            evidence_class="held_out",
        )
        self.assertEqual(result["status"], "I1_OFFLINE_GATE_PASS")
        self.assertEqual(result["i1_better_count"], 3)

    def test_development_bags_cannot_be_called_release_evidence(self):
        result = CORE.development_decision(
            [trial(10.0, 8.0), trial(9.0, 8.0), trial(11.0, 9.0)],
            evidence_class="development",
        )
        self.assertEqual(result["status"], "DEVELOPMENT_ONLY_NOT_RELEASE_EVIDENCE")

    def test_low_coverage_fails_held_out_gate(self):
        result = CORE.development_decision(
            [trial(10.0, 8.0, 0.97) for _ in range(3)],
            evidence_class="held_out",
        )
        self.assertEqual(result["status"], "I1_OFFLINE_GATE_FAIL")
        self.assertFalse(result["coverage_pass"])

    def test_json_safe_replaces_non_finite_values(self):
        self.assertEqual(CORE.json_safe({"x": math.nan, "y": [math.inf, 1.0]}), {"x": None, "y": [None, 1.0]})


if __name__ == "__main__":
    unittest.main()
