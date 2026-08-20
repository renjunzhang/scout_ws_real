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


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ANALYSIS_DIR = PACKAGE_ROOT / "tools" / "analysis"


def load_module(name):
    path = ANALYSIS_DIR / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ALIGNMENT = load_module("analyze_g3_delay_state_alignment")
SCREEN = load_module("analyze_g3r_weight_screen")
SCREEN_V2 = load_module("analyze_g3r2_weight_screen")
CONFIRM = load_module("analyze_g3r2_paired_confirmation")


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

    def test_authorized_method_negative_is_retained_but_not_selectable(self):
        reports = self.make_reports()
        reports["04"]["status"] = "FAIL"
        reports["04"]["failures"] = [
            "stage-0 contour P95 0.053296m > 0.050000m"
        ]
        reports["04"]["tracking"]["contour_p95_m"] = 0.053296396508812904
        result = SCREEN_V2.evaluate_screen(reports, method_negative_rows={"04"})
        self.assertEqual(result["status"], "PROMOTE_FOR_PAIRED_CONFIRMATION")
        self.assertEqual(result["selected"]["condition"], "W5_S10")
        row04 = next(item for item in result["candidates"] if item["row"] == "04")
        self.assertTrue(row04["method_negative"])
        self.assertFalse(row04["eligible_for_selection"])
        self.assertFalse(row04["screen_positive"])

    def test_method_negative_without_authorization_invalidates_screen(self):
        reports = self.make_reports()
        reports["04"]["status"] = "FAIL"
        reports["04"]["failures"] = [
            "stage-0 contour P95 0.053296m > 0.050000m"
        ]
        reports["04"]["tracking"]["contour_p95_m"] = 0.053296396508812904
        result = SCREEN_V2.evaluate_screen(reports)
        self.assertEqual(result["status"], "SCREEN_INVALID")
        self.assertTrue(any("row 04 postflight is not PASS" in item for item in result["failures"]))

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

    def test_cli_excludes_authorized_failed_attempt_and_uses_retry(self):
        reports = self.make_reports()
        screen_prereg = "a" * 64
        source_report = "b" * 64
        with tempfile.TemporaryDirectory() as temporary:
            temporary_path = Path(temporary)
            baseline_root = temporary_path / "baseline"
            screen_root = temporary_path / "screen"
            baseline_root.mkdir()
            screen_root.mkdir()

            def bind_report(report, bag_path, protocol):
                bag_path.write_bytes(("bag-" + bag_path.stem).encode("ascii"))
                bag_sha = hashlib.sha256(bag_path.read_bytes()).hexdigest()
                report.update(
                    {
                        "bag": str(bag_path),
                        "bag_size_bytes": bag_path.stat().st_size,
                        "bag_sha256": bag_sha,
                        "protocol": protocol,
                        "bindings": {
                            "prereg_sha256": screen_prereg,
                            "source_report_sha256": source_report,
                        },
                    }
                )
                return report

            baseline = bind_report(
                reports["01"], baseline_root / "row_01.bag", SCREEN_V2.BASELINE_PROTOCOL
            )
            baseline_path = baseline_root / "baseline_g3r2_postflight.json"
            baseline_path.write_text(json.dumps(baseline), encoding="utf-8")
            baseline_sha = hashlib.sha256(baseline_path.read_bytes()).hexdigest()

            for row in ("02", "05"):
                report = bind_report(
                    reports[row], screen_root / "row_{}.bag".format(row), SCREEN_V2.SCREEN_PROTOCOL
                )
                (screen_root / "DEV_G3R2_row_{}_g3r2_screen_postflight.json".format(row)).write_text(
                    json.dumps(report), encoding="utf-8"
                )

            method_negative = json.loads(json.dumps(reports["04"]))
            method_negative["status"] = "FAIL"
            method_negative["failures"] = [
                "stage-0 contour P95 0.053296m > 0.050000m"
            ]
            method_negative["tracking"][
                "contour_p95_m"
            ] = 0.053296396508812904
            method_negative = bind_report(
                method_negative,
                screen_root / "DEV_G3R2_H0_C1_W2_S10_r04_a01.bag",
                SCREEN_V2.SCREEN_PROTOCOL,
            )
            method_negative_path = (
                screen_root
                / "DEV_G3R2_H0_C1_W2_S10_r04_a01_g3r2_screen_postflight.json"
            )
            method_negative_path.write_text(
                json.dumps(method_negative), encoding="utf-8"
            )
            method_negative_sha = hashlib.sha256(
                method_negative_path.read_bytes()
            ).hexdigest()

            failed = json.loads(json.dumps(reports["03"]))
            failed["status"] = "FAIL"
            failed["failures"] = ["timestamp acquisition failure"]
            failed = bind_report(
                failed,
                screen_root / "DEV_G3R2_H0_C1_W5_S10_r03_a01.bag",
                SCREEN_V2.SCREEN_PROTOCOL,
            )
            failed_path = (
                screen_root
                / "DEV_G3R2_H0_C1_W5_S10_r03_a01_g3r2_screen_postflight.json"
            )
            failed_path.write_text(json.dumps(failed), encoding="utf-8")
            failed_sha = hashlib.sha256(failed_path.read_bytes()).hexdigest()

            retry = bind_report(
                json.loads(json.dumps(reports["03"])),
                screen_root / "DEV_G3R2_H0_C1_W5_S10_r03_a02.bag",
                SCREEN_V2.SCREEN_PROTOCOL,
            )
            retry_path = (
                screen_root
                / "DEV_G3R2_H0_C1_W5_S10_r03_a02_g3r2_screen_retry_postflight.json"
            )
            retry_path.write_text(json.dumps(retry), encoding="utf-8")

            authorization_path = temporary_path / "retry.env"
            authorization_path.write_text(
                "\n".join(
                    (
                        "report_type=DEVELOPMENT_RETRY_AUTHORIZATION",
                        "status=PASS",
                        "retry_authorized=true",
                        "retry_of_attempt_id=DEV_G3R2_H0_C1_W5_S10_r03_a01",
                        "maximum_authorized_attempt=02",
                        "failure_class=METHOD_INDEPENDENT_ACQUISITION",
                        "failure_reason_code=REALSENSE_SOURCE_TIMESTAMP_UNSTABLE_AT_VISUAL_START",
                        "method_failure=false",
                        "failed_attempt_id=DEV_G3R2_H0_C1_W5_S10_r03_a01",
                        "authorized_attempt_id=DEV_G3R2_H0_C1_W5_S10_r03_a02",
                        "screen_prereg_sha256={}".format(screen_prereg),
                        "failed_postflight_sha256={}".format(failed_sha),
                        "failed_bag_sha256={}".format(failed["bag_sha256"]),
                    )
                )
                + "\n",
                encoding="utf-8",
            )
            authorization_sha = hashlib.sha256(authorization_path.read_bytes()).hexdigest()

            method_evidence_path = temporary_path / "method_outcome.env"
            method_evidence_path.write_text(
                "\n".join(
                    (
                        "report_type=DEVELOPMENT_METHOD_OUTCOME_EVIDENCE",
                        "status=PASS",
                        "outcome_class=METHOD_PERFORMANCE_FAILURE",
                        "failure_reason_code=TRACKING_CONTOUR_P95_GATE_EXCEEDED",
                        "method_failure=true",
                        "acquisition_failure=false",
                        "planned_row=04",
                        "condition=W2_S10",
                        "attempt_id=DEV_G3R2_H0_C1_W2_S10_r04_a01",
                        "screen_prereg_sha256={}".format(screen_prereg),
                        "bag_sha256={}".format(method_negative["bag_sha256"]),
                        "postflight_sha256={}".format(method_negative_sha),
                        "candidate_eligible_for_promotion=false",
                        "row04_retry_authorized=false",
                        "artifacts_must_be_preserved=true",
                    )
                )
                + "\n",
                encoding="utf-8",
            )
            method_evidence_sha = hashlib.sha256(
                method_evidence_path.read_bytes()
            ).hexdigest()
            continuation_path = temporary_path / "continuation.env"
            continuation_path.write_text(
                "\n".join(
                    (
                        "report_type=DEVELOPMENT_SCREEN_CONTINUATION_AUTHORIZATION",
                        "status=PASS",
                        "scope=G3R2_DEVELOPMENT_SINGLE_RUN_SCREEN",
                        "continuation_authorized=true",
                        "failed_method_row=04",
                        "failed_method_condition=W2_S10",
                        "failed_method_attempt_id=DEV_G3R2_H0_C1_W2_S10_r04_a01",
                        "next_authorized_row=05",
                        "next_authorized_condition=W5_S03",
                        "next_authorized_attempt_id=DEV_G3R2_H0_C1_W5_S03_r05_a01",
                        "outcome_class=METHOD_PERFORMANCE_FAILURE",
                        "failure_reason_code=TRACKING_CONTOUR_P95_GATE_EXCEEDED",
                        "method_failure=true",
                        "acquisition_failure=false",
                        "row04_retry_authorized=false",
                        "row04_candidate_eligible_for_promotion=false",
                        "row04_must_remain_in_dataset=true",
                        "row05_must_keep_original_configuration=true",
                        "screen_prereg_is_unchanged=true",
                        "formal_efficacy_claim_authorized=false",
                        "screen_prereg_sha256={}".format(screen_prereg),
                        "method_outcome_evidence_path={}".format(
                            method_evidence_path
                        ),
                        "method_outcome_evidence_sha256={}".format(
                            method_evidence_sha
                        ),
                        "failed_bag_sha256={}".format(
                            method_negative["bag_sha256"]
                        ),
                        "failed_postflight_sha256={}".format(
                            method_negative_sha
                        ),
                        "analysis_policy=RETAIN_ROW04_AS_INELIGIBLE_METHOD_NEGATIVE",
                    )
                )
                + "\n",
                encoding="utf-8",
            )
            continuation_sha = hashlib.sha256(
                continuation_path.read_bytes()
            ).hexdigest()

            completed = subprocess.run(
                [
                    sys.executable,
                    str(ANALYSIS_DIR / "analyze_g3r2_weight_screen.py"),
                    "--root",
                    str(screen_root),
                    "--baseline-report",
                    str(baseline_path),
                    "--baseline-report-sha256",
                    baseline_sha,
                    "--screen-prereg-sha256",
                    screen_prereg,
                    "--source-report-sha256",
                    source_report,
                    "--retry-authorization",
                    str(authorization_path),
                    "--retry-authorization-sha256",
                    authorization_sha,
                    "--method-failure-authorization",
                    str(continuation_path),
                    "--method-failure-authorization-sha256",
                    continuation_sha,
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
            self.assertEqual(len(output["excluded_acquisition_attempts"]), 1)
            self.assertEqual(output["excluded_acquisition_attempts"][0]["attempt"], "01")
            eligible_row03 = next(item for item in output["dataset"] if item["row"] == "03")
            self.assertEqual(eligible_row03["attempt"], "02")
            self.assertEqual(len(output["method_negative_candidates"]), 1)
            self.assertEqual(output["method_negative_candidates"][0]["row"], "04")
            row04 = next(item for item in output["dataset"] if item["row"] == "04")
            self.assertFalse(row04["eligible_for_selection"])


class RobotOnlyPairedConfirmationTest(unittest.TestCase):
    def make_reports(self):
        values = {
            "01": (0.80, 0.40, 1.20),
            "02": (0.70, 0.34, 1.17),
            "03": (0.65, 0.31, 1.14),
            "04": (0.75, 0.38, 1.19),
            "05": (0.82, 0.41, 1.22),
            "06": (0.70, 0.35, 1.18),
        }
        reports = {}
        for row, (block, position, condition) in CONFIRM.EXPECTED_ROWS.items():
            p95, rms, raw = values[row]
            report = g3r2_screen_report(
                row,
                condition,
                0.0 if condition == "Bsmooth" else 5.0,
                1.0,
                p95,
                rms,
                raw,
            )
            report.update({"block": block, "position": position})
            report["processed_imu"] = {
                "ready_fraction": 1.0,
                "fallback_samples": 0,
                "reset_epochs": [0],
            }
            reports[row] = report
        return reports

    def test_three_consistent_pairs_confirm_w5_s10(self):
        result = CONFIRM.evaluate_confirmation(self.make_reports())
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["decision"], "W5_S10_PAIRED_CONFIRMED")
        self.assertEqual(result["aggregate"]["positive_block_count"], 3)
        self.assertGreaterEqual(
            result["aggregate"]["mean_rgb_p95_improvement_mm"], 0.05
        )

    def test_single_block_dominated_effect_does_not_confirm(self):
        reports = self.make_reports()
        reports["02"]["online_rgb"]["h_vis_p95_mm"] = 0.82
        reports["03"]["online_rgb"]["h_vis_p95_mm"] = 0.77
        reports["06"]["online_rgb"]["h_vis_p95_mm"] = 0.52
        result = CONFIRM.evaluate_confirmation(reports)
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["aggregate"]["gates"]["positive_blocks_pass"])
        self.assertFalse(
            result["aggregate"]["gates"]["single_block_dominance_pass"]
        )

    def test_liquid_delay_rollout_invalidates_confirmation(self):
        reports = self.make_reports()
        reports["03"]["internal_state"][
            "liquid_delay_compensation_applied_fraction"
        ] = 0.03
        result = CONFIRM.evaluate_confirmation(reports)
        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(
            any("forbidden liquid delay rollout" in item for item in result["failures"])
        )

    def test_cli_audits_bags_and_writes_confirmation_report(self):
        reports = self.make_reports()
        prereg = "a" * 64
        window = "b" * 64
        source = "c" * 64
        screen = "d" * 64
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for row, report in reports.items():
                bag = root / "DEV_G3R2C_row_{}.bag".format(row)
                bag.write_bytes(("confirmation-bag-" + row).encode("ascii"))
                report.update(
                    {
                        "protocol": CONFIRM.PROTOCOL,
                        "bag": str(bag),
                        "bag_size_bytes": bag.stat().st_size,
                        "bag_sha256": hashlib.sha256(bag.read_bytes()).hexdigest(),
                        "bindings": {
                            "prereg_sha256": prereg,
                            "outcome_window_rule_sha256": window,
                            "source_report_sha256": source,
                        },
                    }
                )
                (root / "DEV_G3R2C_row_{}_g3r2c_postflight.json".format(row)).write_text(
                    json.dumps(report), encoding="utf-8"
                )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(ANALYSIS_DIR / "analyze_g3r2_paired_confirmation.py"),
                    "--root",
                    str(root),
                    "--prereg-sha256",
                    prereg,
                    "--outcome-window-rule-sha256",
                    window,
                    "--source-report-sha256",
                    source,
                    "--screen-report-sha256",
                    screen,
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            output = json.loads(
                (root / "G3R2_PAIRED_CONFIRMATION_REPORT.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(output["status"], "PASS")
            self.assertEqual(output["decision"], "W5_S10_PAIRED_CONFIRMED")
            self.assertEqual(len(output["dataset"]), 6)


if __name__ == "__main__":
    unittest.main()
