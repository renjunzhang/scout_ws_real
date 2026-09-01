#!/usr/bin/env python3
"""Unit tests for the literal I0/fail-closed/fixed ABBA RGB analyzer."""

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
ANALYZER = (
    SCRIPT_DIR.parent
    / "analysis"
    / "analyze_i0_failclosed_fixed_abba_rgb.py"
)
SPEC = importlib.util.spec_from_file_location("i0_fixed_abba_rgb", ANALYZER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_report(row, p95, rms, status="PASS", **overrides):
    expected = MODULE.EXPECTED_ROWS[row]
    report = {
        "schema_version": 1,
        "protocol": MODULE.PROTOCOL,
        "status": status,
        "condition": expected["condition"],
        "row": row,
        "block": expected["block"],
        "position": expected["position"],
        "motion_to_arrival_sec": 30.0,
        "t_hvis_tail_sec": 5.0,
        "online_rgb": {
            "rolling_window": 5,
            "h_vis_p95_mm": p95,
            "h_vis_rms_mm": rms,
        },
    }
    report.update(overrides)
    return report


class I0FixedAbbaRgbAnalysisTest(unittest.TestCase):
    def test_block1_promotes_at_inclusive_thresholds(self):
        reports = {
            "01": make_report("01", 0.80, 0.50),
            "02": make_report("02", 0.75, 0.50),
        }
        result = MODULE.evaluate_reports(reports)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["decision"], "PROMOTE_BLOCK2")
        self.assertEqual(result["exit_code"], 0)
        self.assertAlmostEqual(result["blocks"][0]["delta_p95_mm"], 0.05)
        self.assertAlmostEqual(result["blocks"][0]["delta_rms_mm"], 0.0)

    def test_block1_stops_when_either_rapid_gate_fails(self):
        p95_failure = MODULE.evaluate_reports(
            {
                "01": make_report("01", 0.80, 0.50),
                "02": make_report("02", 0.751, 0.49),
            }
        )
        self.assertEqual(p95_failure["decision"], "STOP_BLOCK1_FUTILITY")
        self.assertEqual(p95_failure["exit_code"], 10)

        rms_failure = MODULE.evaluate_reports(
            {
                "01": make_report("01", 0.80, 0.50),
                "02": make_report("02", 0.74, 0.51),
            }
        )
        self.assertEqual(rms_failure["decision"], "STOP_BLOCK1_FUTILITY")
        self.assertFalse(rms_failure["gates"]["delta_rms_nonnegative"])

    def test_complete_abba_uses_frozen_final_gates(self):
        # Block 1 delta P95/RMS = +0.06/+0.02.
        # Block 2 delta P95/RMS = +0.04/-0.01.
        # Both P95 blocks are positive and the means are +0.05/+0.005.
        reports = {
            "01": make_report("01", 0.80, 0.50),
            "02": make_report("02", 0.74, 0.48),
            "03": make_report("03", 0.76, 0.51),
            "04": make_report("04", 0.80, 0.50),
        }
        result = MODULE.evaluate_reports(reports)
        self.assertEqual(result["phase"], "COMPLETE_ABBA")
        self.assertEqual(result["decision"], "DEVELOPMENT_POSITIVE")
        self.assertEqual(result["exit_code"], 0)
        self.assertAlmostEqual(result["aggregate"]["mean_delta_p95_mm"], 0.05)
        self.assertAlmostEqual(result["aggregate"]["mean_delta_rms_mm"], 0.005)

    def test_complete_abba_rejects_discordant_blocks(self):
        reports = {
            "01": make_report("01", 0.90, 0.60),  # block 1: +0.11
            "02": make_report("02", 0.79, 0.55),
            "03": make_report("03", 0.801, 0.50),  # block 2: -0.001
            "04": make_report("04", 0.80, 0.50),
        }
        result = MODULE.evaluate_reports(reports)
        self.assertEqual(result["decision"], "NO_DEVELOPMENT_POSITIVE")
        self.assertEqual(result["exit_code"], 10)
        self.assertFalse(result["gates"]["both_blocks_delta_p95_positive"])

    def test_complete_rgb_positive_is_labeled_when_slowdown_confounded(self):
        reports = {
            "01": make_report("01", 0.80, 0.50, motion_to_arrival_sec=30.0),
            "02": make_report("02", 0.74, 0.48, motion_to_arrival_sec=33.0),
            "03": make_report("03", 0.74, 0.48, motion_to_arrival_sec=33.0),
            "04": make_report("04", 0.80, 0.50, motion_to_arrival_sec=30.0),
        }
        result = MODULE.evaluate_reports(reports)
        self.assertEqual(result["status"], "STOP")
        self.assertEqual(result["decision"], "RGB_POSITIVE_SLOWDOWN_CONFOUNDED")
        self.assertEqual(result["exit_code"], 10)
        self.assertTrue(result["aggregate"]["rgb_effect_gates_pass"])
        self.assertFalse(
            result["gates"]["median_goal_time_slowdown_at_most_maximum"]
        )

    def test_complete_abba_rejects_mean_below_threshold(self):
        reports = {
            "01": make_report("01", 0.80, 0.50),  # block 1: +0.05, promotes
            "02": make_report("02", 0.75, 0.49),
            "03": make_report("03", 0.79, 0.49),  # block 2: +0.01
            "04": make_report("04", 0.80, 0.50),
        }
        result = MODULE.evaluate_reports(reports)
        self.assertEqual(result["decision"], "NO_DEVELOPMENT_POSITIVE")
        self.assertTrue(result["gates"]["block1_rapid_screen_pass"])
        self.assertTrue(result["gates"]["both_blocks_delta_p95_positive"])
        self.assertFalse(result["gates"]["mean_delta_p95_at_least_minimum"])

    def test_complete_abba_cannot_bypass_failed_block1_stop(self):
        reports = {
            "01": make_report("01", 0.80, 0.50),  # block 1: +0.04; no promotion
            "02": make_report("02", 0.76, 0.49),
            "03": make_report("03", 0.70, 0.40),  # block 2: +0.08
            "04": make_report("04", 0.78, 0.50),
        }
        result = MODULE.evaluate_reports(reports)
        self.assertEqual(result["decision"], "NO_DEVELOPMENT_POSITIVE")
        self.assertFalse(result["gates"]["block1_rapid_screen_pass"])
        self.assertGreaterEqual(result["aggregate"]["mean_delta_p95_mm"], 0.05)

    def test_failed_postflight_or_metric_contract_is_invalid(self):
        failed = {
            "01": make_report("01", 0.80, 0.50, status="FAIL"),
            "02": make_report("02", 0.74, 0.48),
        }
        result = MODULE.evaluate_reports(failed)
        self.assertEqual(result["status"], "INVALID")
        self.assertIn("row 01 postflight status is not PASS", result["failures"])

        wrong_filter = {
            "01": make_report(
                "01", 0.80, 0.50, online_rgb={
                    "rolling_window": 3,
                    "h_vis_p95_mm": 0.80,
                    "h_vis_rms_mm": 0.50,
                }
            ),
            "02": make_report("02", 0.74, 0.48),
        }
        result = MODULE.evaluate_reports(wrong_filter)
        self.assertEqual(result["status"], "INVALID")
        self.assertTrue(any("causal median5" in item for item in result["failures"]))

    def test_order_and_supported_row_sets_are_fail_closed(self):
        wrong_order = {
            "01": make_report("01", 0.80, 0.50, condition="Bslosh"),
            "02": make_report("02", 0.74, 0.48),
        }
        result = MODULE.evaluate_reports(wrong_order)
        self.assertEqual(result["status"], "INVALID")
        self.assertTrue(any("condition" in item for item in result["failures"]))

        incomplete = {
            "01": make_report("01", 0.80, 0.50),
            "02": make_report("02", 0.74, 0.48),
            "03": make_report("03", 0.76, 0.49),
        }
        result = MODULE.evaluate_reports(incomplete)
        self.assertEqual(result["status"], "INVALID")
        self.assertTrue(any("unsupported row set" in item for item in result["failures"]))

        wrong_protocol = {
            "01": make_report("01", 0.80, 0.50, protocol="wrong"),
            "02": make_report("02", 0.74, 0.48),
        }
        result = MODULE.evaluate_reports(wrong_protocol)
        self.assertEqual(result["status"], "INVALID")
        self.assertTrue(any("protocol mismatch" in item for item in result["failures"]))

    def test_cli_discovers_postflights_and_writes_report(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for row, p95, rms in (("01", 0.80, 0.50), ("02", 0.74, 0.48)):
                prefix = "DEV_{}".format(row)
                path = root / "{}_i0_fixed_rgb_postflight.json".format(prefix)
                path.write_text(
                    json.dumps(make_report(row, p95, rms)) + "\n",
                    encoding="utf-8",
                )
                (root / "{}_unit_pass.env".format(prefix)).write_text(
                    "status=PASS\nprotocol={}\nrow={}\ncondition={}\n".format(
                        MODULE.PROTOCOL,
                        row,
                        MODULE.EXPECTED_ROWS[row]["condition"],
                    ),
                    encoding="utf-8",
                )
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--root", str(root)],
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            output_path = root / MODULE.DEFAULT_REPORT_NAME
            output = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(output["decision"], "PROMOTE_BLOCK2")
            self.assertEqual(
                output["metric_contract"]["window"],
                "motion_start_to_first_GOAL_REACHED_plus_5s_tail",
            )
            self.assertEqual(output["thresholds"]["required_order"], "B0,Bslosh,Bslosh,B0")

    def test_cli_rejects_a_missing_unit_pass(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for row, p95, rms in (("01", 0.80, 0.50), ("02", 0.74, 0.48)):
                prefix = "DEV_{}".format(row)
                (root / "{}_i0_fixed_rgb_postflight.json".format(prefix)).write_text(
                    json.dumps(make_report(row, p95, rms)) + "\n",
                    encoding="utf-8",
                )
                if row == "01":
                    (root / "{}_unit_pass.env".format(prefix)).write_text(
                        "status=PASS\nprotocol={}\nrow=01\ncondition=B0\n".format(
                            MODULE.PROTOCOL
                        ),
                        encoding="utf-8",
                    )
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--root", str(root)],
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 2)
            output = json.loads(
                (root / MODULE.DEFAULT_REPORT_NAME).read_text(encoding="utf-8")
            )
            self.assertEqual(output["decision"], "INVALID_EVIDENCE")
            self.assertTrue(any("missing" in item for item in output["failures"]))


if __name__ == "__main__":
    unittest.main()
