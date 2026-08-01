#!/usr/bin/env python3

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "analysis" / "prepare_g5_comparators.py"
SPEC = importlib.util.spec_from_file_location("prepare_g5_comparators", MODULE_PATH)
G5 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = G5
SPEC.loader.exec_module(G5)


def report(row, condition, duration):
    return {
        "row": row,
        "condition": condition,
        "status": "PASS",
        "motion_start_sec": 100.0,
        "first_arrival_sec": 100.0 + duration,
    }


class G5ComparatorTest(unittest.TestCase):
    def test_smooth_match_uses_paired_completion_ratio(self):
        reports = {
            "01": report("01", "Bsmooth", 34.0),
            "02": report("02", "W5", 36.0),
            "03": report("03", "W5", 36.0),
            "04": report("04", "Bsmooth", 34.0),
            "05": report("05", "W5", 36.0),
            "06": report("06", "Bsmooth", 34.0),
            "07": report("07", "Bsmooth", 34.0),
            "08": report("08", "W5", 36.0),
        }
        result = G5.derive_smooth_match(
            reports,
            {
                "nominal_bsmooth_v_ref_m_s": 0.20,
                "safe_v_ref_min_m_s": 0.15,
                "safe_v_ref_max_m_s": 0.20,
                "rounding_step_m_s": 0.005,
                "completion_relative_tolerance": 0.05,
            },
        )
        self.assertEqual(result["status"], "READY")
        self.assertAlmostEqual(result["frozen_v_ref_m_s"], 0.19)
        self.assertEqual(len(result["paired_blocks"]), 4)

    def test_failed_row_is_not_silently_used(self):
        reports = {
            "01": report("01", "Bsmooth", 34.0),
            "02": report("02", "W5", 36.0),
        }
        result = G5.derive_smooth_match(
            reports,
            {
                "nominal_bsmooth_v_ref_m_s": 0.20,
                "safe_v_ref_min_m_s": 0.15,
                "safe_v_ref_max_m_s": 0.20,
                "rounding_step_m_s": 0.005,
                "completion_relative_tolerance": 0.05,
            },
        )
        self.assertEqual(result["status"], "NO-GO")
        self.assertTrue(result["failures"])

    def test_round_to_step(self):
        self.assertAlmostEqual(G5.round_to_step(0.187, 0.005), 0.185)
        self.assertAlmostEqual(G5.round_to_step(0.188, 0.005), 0.19)


if __name__ == "__main__":
    unittest.main()
