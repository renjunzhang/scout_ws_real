#!/usr/bin/env python3

import importlib.util
import math
import unittest
from pathlib import Path


ANALYSIS_DIR = Path(__file__).resolve().parents[1] / "analysis"


def load_module(name):
    path = ANALYSIS_DIR / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ALIGNMENT = load_module("analyze_g3_delay_state_alignment")
SCREEN = load_module("analyze_g3r_weight_screen")


def screen_report(row, condition, w_slosh, smooth, rgb_p95, rgb_rms, raw_p95):
    return {
        "row": row,
        "condition": condition,
        "status": "PASS",
        "online_rgb": {"h_vis_p95_mm": rgb_p95, "h_vis_rms_mm": rgb_rms},
        "internal_state": {
            "imu_modal_height_p95_mm": raw_p95,
            "delay_compensation_applied_fraction": 0.0,
            "solver_source_code_fraction": 1.0,
        },
        "effective_config_last": {
            "w_slosh": w_slosh,
            "w_smooth": smooth,
            "w_alpha": smooth,
            "w_du_a": smooth,
            "w_du_vs": smooth,
            "delay_phase_mode_code": 2.0,
        },
        "tracking": {"contour_p95_m": 0.02},
        "runtime": {"solver_p95_ms": 10.0},
    }


class DelayAlignmentTest(unittest.TestCase):
    def test_positive_lag_uses_response_after_command(self):
        command = [(index * 0.01, math.sin(index * 0.071)) for index in range(2000)]
        response = [(stamp + 0.18, value) for stamp, value in command]
        lag, correlation = ALIGNMENT.best_positive_lag(command, response, maximum_lag=0.4)
        self.assertAlmostEqual(lag, 0.18, places=2)
        self.assertGreater(correlation, 0.99)

    def test_alignment_gate_selects_shadow_when_raw_is_better(self):
        rows = []
        for index in range(6):
            rows.append(
                {
                    "eligible": True,
                    "block": "{:02d}".format(index // 2 + 1),
                    "condition": "Bsmooth" if index % 2 == 0 else "W5",
                    "metrics": {
                        "raw_rgb_corr_zero": 0.80,
                        "predicted_rgb_corr_zero": 0.60,
                        "rgb_p95_mm": 0.7 + 0.1 * (index % 2),
                    },
                }
            )
        result = ALIGNMENT.evaluate_release(rows, 6, 0.65, 0.08)
        self.assertEqual(result["status"], "PASS_FOR_G3R_SCREENING")
        self.assertEqual(result["recommended_delay_phase_mode"], "shadow")
        self.assertFalse(result["recommended_closed_loop_delay_compensation"])


class WeightScreenTest(unittest.TestCase):
    def make_reports(self):
        reports = {}
        values = {
            "01": (0.80, 0.40, 1.20),
            "02": (0.78, 0.39, 1.21),
            "03": (0.60, 0.30, 1.15),
            "04": (0.76, 0.38, 1.30),
            "05": (0.68, 0.34, 1.18),
        }
        for row, expected in SCREEN.EXPECTED_ROWS.items():
            p95, rms, raw = values[row]
            reports[row] = screen_report(
                row,
                expected["condition"],
                expected["w_slosh"],
                expected["smooth"],
                p95,
                rms,
                raw,
            )
        return reports

    def test_single_positive_candidate_is_promoted_for_confirmation(self):
        result = SCREEN.evaluate_screen(self.make_reports())
        self.assertEqual(result["status"], "PROMOTE_FOR_PAIRED_CONFIRMATION")
        self.assertEqual(result["selected"]["condition"], "W5_S10")
        plan = SCREEN.build_confirmation_plan(result["selected"], "a" * 64, "b" * 64)
        self.assertEqual(plan["paired_blocks"], 3)
        self.assertEqual(len(plan["planned_rows"]), 6)
        self.assertEqual(
            sum(row["condition"] == "W5_S10" for row in plan["planned_rows"]), 3
        )

    def test_single_run_never_claims_efficacy(self):
        result = SCREEN.evaluate_screen(self.make_reports())
        self.assertNotIn("PASS", result["status"])

    def test_no_rgb_direction_means_no_promotion(self):
        reports = self.make_reports()
        for row in ("02", "03", "04", "05"):
            reports[row]["online_rgb"]["h_vis_p95_mm"] = 0.90
        result = SCREEN.evaluate_screen(reports)
        self.assertEqual(result["status"], "NO_PROMOTION")
        self.assertIsNone(result["selected"])

    def test_missing_or_failed_row_is_invalid_not_no_promotion(self):
        reports = self.make_reports()
        reports["03"]["status"] = "FAIL"
        result = SCREEN.evaluate_screen(reports)
        self.assertEqual(result["status"], "SCREEN_INVALID")
        self.assertIsNone(result["selected"])

        reports = self.make_reports()
        del reports["05"]
        result = SCREEN.evaluate_screen(reports)
        self.assertEqual(result["status"], "SCREEN_INVALID")
        self.assertIsNone(result["selected"])

    def test_nonfinite_metric_is_invalid_without_crashing(self):
        reports = self.make_reports()
        reports["02"]["online_rgb"]["h_vis_p95_mm"] = None
        result = SCREEN.evaluate_screen(reports)
        self.assertEqual(result["status"], "SCREEN_INVALID")
        self.assertIsNone(result["selected"])


if __name__ == "__main__":
    unittest.main()
