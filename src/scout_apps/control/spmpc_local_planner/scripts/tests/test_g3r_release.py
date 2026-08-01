#!/usr/bin/env python3

import importlib.util
import hashlib
import json
import math
import subprocess
import sys
import tempfile
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
SCREEN_V2 = load_module("analyze_g3r2_weight_screen")


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


def g3r2_screen_report(row, condition, w_slosh, smooth, rgb_p95, rgb_rms, raw_p95):
    report = screen_report(row, condition, w_slosh, smooth, rgb_p95, rgb_rms, raw_p95)
    report["internal_state"].update(
        {
            "delay_compensation_applied_fraction": 1.0,
            "robot_delay_compensation_applied_fraction": 1.0,
            "liquid_delay_compensation_applied_fraction": 0.0,
        }
    )
    report["effective_config_last"]["delay_phase_mode_code"] = 4.0
    report["tracking"]["yaw_p95_rad"] = 0.05
    return report


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


class RobotOnlyWeightScreenTest(unittest.TestCase):
    def make_reports(self):
        reports = {}
        values = {
            "01": (0.80, 0.40, 1.20),
            "02": (0.78, 0.39, 1.21),
            "03": (0.60, 0.30, 1.15),
            "04": (0.76, 0.38, 1.30),
            "05": (0.68, 0.34, 1.18),
        }
        for row, expected in SCREEN_V2.EXPECTED_ROWS.items():
            p95, rms, raw = values[row]
            reports[row] = g3r2_screen_report(
                row,
                expected["condition"],
                expected["w_slosh"],
                expected["smooth"],
                p95,
                rms,
                raw,
            )
        return reports

    def test_robot_only_screen_promotes_positive_candidate(self):
        result = SCREEN_V2.evaluate_screen(self.make_reports())
        self.assertEqual(result["status"], "PROMOTE_FOR_PAIRED_CONFIRMATION")
        self.assertEqual(result["selected"]["condition"], "W5_S10")

    def test_liquid_delay_rollout_invalidates_screen(self):
        reports = self.make_reports()
        reports["03"]["internal_state"][
            "liquid_delay_compensation_applied_fraction"
        ] = 0.03
        result = SCREEN_V2.evaluate_screen(reports)
        self.assertEqual(result["status"], "SCREEN_INVALID")
        self.assertTrue(
            any("forbidden liquid delay rollout" in failure for failure in result["failures"])
        )

    def test_missing_candidate_is_invalid(self):
        reports = self.make_reports()
        del reports["05"]
        result = SCREEN_V2.evaluate_screen(reports)
        self.assertEqual(result["status"], "SCREEN_INVALID")

    def test_cli_combines_frozen_external_baseline_and_four_candidates(self):
        reports = self.make_reports()
        screen_prereg = "a" * 64
        source_report = "b" * 64
        with tempfile.TemporaryDirectory() as temporary:
            temporary_path = Path(temporary)
            baseline_root = temporary_path / "baseline"
            screen_root = temporary_path / "screen"
            baseline_root.mkdir()
            screen_root.mkdir()

            report_paths = {}
            for row, report in reports.items():
                parent = baseline_root if row == "01" else screen_root
                bag_path = parent / "row_{}.bag".format(row)
                bag_path.write_bytes(("bag-" + row).encode("ascii"))
                bag_sha = hashlib.sha256(bag_path.read_bytes()).hexdigest()
                report.update(
                    {
                        "bag": str(bag_path),
                        "bag_size_bytes": bag_path.stat().st_size,
                        "bag_sha256": bag_sha,
                        "protocol": (
                            SCREEN_V2.BASELINE_PROTOCOL
                            if row == "01"
                            else SCREEN_V2.SCREEN_PROTOCOL
                        ),
                        "bindings": {
                            "prereg_sha256": screen_prereg,
                            "source_report_sha256": source_report,
                        },
                    }
                )
                report_path = (
                    baseline_root / "baseline_g3r2_postflight.json"
                    if row == "01"
                    else screen_root
                    / "DEV_G3R2_row_{}_g3r2_screen_postflight.json".format(row)
                )
                report_path.write_text(json.dumps(report), encoding="utf-8")
                report_paths[row] = report_path

            baseline_sha = hashlib.sha256(report_paths["01"].read_bytes()).hexdigest()
            completed = subprocess.run(
                [
                    sys.executable,
                    str(ANALYSIS_DIR / "analyze_g3r2_weight_screen.py"),
                    "--root",
                    str(screen_root),
                    "--baseline-report",
                    str(report_paths["01"]),
                    "--baseline-report-sha256",
                    baseline_sha,
                    "--screen-prereg-sha256",
                    screen_prereg,
                    "--source-report-sha256",
                    source_report,
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            output = json.loads(
                (screen_root / "G3R2_WEIGHT_SCREEN_REPORT.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(output["status"], "PROMOTE_FOR_PAIRED_CONFIRMATION")
            self.assertEqual(output["selected_candidate"]["condition"], "W5_S10")


if __name__ == "__main__":
    unittest.main()
