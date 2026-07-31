#!/usr/bin/env python3

import importlib.util
import math
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "analysis/analyze_g2s_source_selection.py"
)
SPEC = importlib.util.spec_from_file_location("analyze_g2s_source_selection", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def trial(odom_mae, imu_mae, odom_coverage=0.98, imu_coverage=0.98):
    return {
        "odom": {"mae_mm": odom_mae, "coverage": odom_coverage},
        "processed_imu": {"mae_mm": imu_mae, "coverage": imu_coverage},
    }


class AnalyzeG2sSourceSelectionTest(unittest.TestCase):
    def test_centered_rolling_median_preserves_invalid_center(self):
        records = [
            (0.0, 1.0),
            (1.0, 100.0),
            (2.0, None),
            (3.0, 3.0),
            (4.0, 5.0),
        ]
        output = MODULE.centered_rolling_median(records, 3)
        self.assertAlmostEqual(output[1][1], 50.5)
        self.assertIsNone(output[2][1])
        self.assertAlmostEqual(output[3][1], 4.0)

    def test_centered_rolling_median_rejects_even_window(self):
        with self.assertRaises(RuntimeError):
            MODULE.centered_rolling_median([(0.0, 1.0)], 4)

    def test_interpolation_is_bounded_and_rejects_large_gaps(self):
        self.assertAlmostEqual(
            MODULE.interpolate_at(0.5, [0.0, 1.0], [2.0, 4.0], 1.0),
            3.0,
        )
        self.assertIsNone(
            MODULE.interpolate_at(0.5, [0.0, 1.0], [2.0, 4.0], 0.9)
        )
        self.assertIsNone(
            MODULE.interpolate_at(-0.1, [0.0, 1.0], [2.0, 4.0], 1.0)
        )

    def test_processed_imu_requires_all_four_gates(self):
        trials = [trial(10.0, 8.0) for _ in range(4)]
        decision, aggregate = MODULE.decide_source(trials, 0.10, 3, 0.90, 0.02)
        self.assertEqual(decision, "processed_imu")
        self.assertAlmostEqual(aggregate["processed_imu_relative_improvement"], 0.20)
        self.assertTrue(aggregate["threshold_pass"])
        self.assertTrue(aggregate["direction_pass"])
        self.assertTrue(aggregate["not_single_trial_dominated"])
        self.assertTrue(aggregate["coverage_pass"])

    def test_single_trial_dominance_defaults_to_odom(self):
        trials = [
            trial(10.0, 5.0),
            trial(10.0, 11.0),
            trial(10.0, 11.0),
            trial(10.0, 11.0),
        ]
        decision, aggregate = MODULE.decide_source(trials, 0.01, 1, 0.90, 0.02)
        self.assertEqual(decision, "odom")
        self.assertFalse(aggregate["not_single_trial_dominated"])

    def test_coverage_degradation_defaults_to_odom(self):
        trials = [trial(10.0, 7.0, 0.99, 0.85) for _ in range(4)]
        decision, aggregate = MODULE.decide_source(trials, 0.10, 3, 0.90, 0.02)
        self.assertEqual(decision, "odom")
        self.assertFalse(aggregate["coverage_pass"])


if __name__ == "__main__":
    unittest.main()
