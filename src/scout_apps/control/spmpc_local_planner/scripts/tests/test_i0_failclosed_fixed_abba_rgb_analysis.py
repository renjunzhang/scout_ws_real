#!/usr/bin/env python3
"""Unit tests for the literal I0/fail-closed/fixed ABBA RGB analyzer."""

import importlib.util
import hashlib
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


SHORT100_PROTOCOL = "SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2"
SHORT100_RGB_SUFFIX = "_i0_fixed_short100_v2_rgb_postflight.json"
SHORT100_UNIT_SUFFIX = "_short100_v2_unit_pass.env"
SHORT100_REPORT_TYPE = "I0_FAILCLOSED_FIXED_SHORT100_ABBA_RGB_ANALYSIS"
SHORT100_EXACT_SUFFIX = "_i0_fixed_short100_v2_postflight.json"
SHORT100_OBSERVER_SUFFIX = "_short100_v2_observer_postflight.json"
SHORT100_CHAIN_SUFFIX = "_short100_v2_mocap_chain_postflight.json"
SHORT100_EXACT_SCHEMA = "spmpc_i0_failclosed_fixed_short100_abba_postflight_v2"


def file_sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_short100_evidence(
    root,
    row,
    p95,
    rms,
    exact_overrides=None,
    marker_overrides=None,
):
    expected = MODULE.EXPECTED_ROWS[row]
    treatment = expected["condition"] == "Bslosh"
    variant = "B_slosh_short100" if treatment else "B0"
    condition_label = "Bslosh-short100" if treatment else "B0"
    steps = 3 if treatment else -1
    tail = 0.0 if treatment else 1.0
    prefix = "S100_{}".format(row)
    bag_path = root / (prefix + ".bag")
    bag_path.write_bytes(b"short100 test bag\n")

    rgb_path = root / (prefix + SHORT100_RGB_SUFFIX)
    rgb_path.write_text(
        json.dumps(
            make_report(
                row,
                p95,
                rms,
                protocol=SHORT100_PROTOCOL,
            )
        )
        + "\n",
        encoding="utf-8",
    )
    observer_path = root / (prefix + SHORT100_OBSERVER_SUFFIX)
    observer_path.write_text(
        json.dumps({"status": "PASS", "protocol": SHORT100_PROTOCOL}) + "\n",
        encoding="utf-8",
    )
    chain_path = root / (prefix + SHORT100_CHAIN_SUFFIX)
    chain_path.write_text(
        json.dumps({"status": "PASS", "variant": variant}) + "\n",
        encoding="utf-8",
    )
    exact_path = root / (prefix + SHORT100_EXACT_SUFFIX)
    exact = {
        "schema": SHORT100_EXACT_SCHEMA,
        "protocol": SHORT100_PROTOCOL,
        "status": "PASS",
        "condition": expected["condition"],
        "expected_variant": variant,
        "bag": str(bag_path.resolve()),
        "contracts": {
            "deep_liquid_cost_contract": True,
            "expected_robot_horizon_steps": 60,
            "expected_slosh_cost_horizon_steps": steps,
            "expected_slosh_cost_tail_discount": tail,
            "expected_slosh_eta_dot_ratio": 0.3,
            "expected_dt_sec": 1.0 / 30.0,
            "expected_control_frequency_hz": 30.0,
            "expected_effective_config": {
                "w_smooth": 0.1,
                "w_alpha": 0.1,
                "w_du_a": 0.1,
                "w_du_vs": 0.1,
                "slosh_height_max": 0.001,
                "alpha_max": 1.2,
            },
            "effective_config_field_failure_count": 0,
            "liquid_config_failure_count": 0,
            "cycle_audit_coverage_failure_count": 0,
            "selection_cycle_failure_count": 0,
            "liquid_artifact_failure_count": 0,
            "solve_cycle_count": 20,
        },
        "failures": [],
    }
    if exact_overrides:
        for field, value in exact_overrides.items():
            if field == "contracts":
                exact["contracts"].update(value)
            else:
                exact[field] = value
    exact_path.write_text(json.dumps(exact) + "\n", encoding="utf-8")

    marker = {
        "status": "PASS",
        "protocol": SHORT100_PROTOCOL,
        "profile": "short100_v2",
        "row": row,
        "condition": expected["condition"],
        "condition_label": condition_label,
        "variant": variant,
        "slosh_cost_horizon_steps": str(steps),
        "slosh_cost_tail_discount": str(tail),
        "bag": str(bag_path.resolve()),
        "exact_postflight_sha256": file_sha256(exact_path),
        "observer_postflight_sha256": file_sha256(observer_path),
        "chain_postflight_sha256": file_sha256(chain_path),
        "rgb_postflight_sha256": file_sha256(rgb_path),
    }
    if marker_overrides:
        marker.update(marker_overrides)
    unit_path = root / (prefix + SHORT100_UNIT_SUFFIX)
    unit_path.write_text(
        "".join("{}={}\n".format(key, value) for key, value in marker.items()),
        encoding="utf-8",
    )
    return {
        "bag": bag_path,
        "rgb": rgb_path,
        "observer": observer_path,
        "chain": chain_path,
        "exact": exact_path,
        "unit": unit_path,
    }


def short100_analyzer_command(root, output_path):
    return [
        sys.executable,
        str(ANALYZER),
        "--root",
        str(root),
        "--report",
        str(output_path),
        "--protocol",
        SHORT100_PROTOCOL,
        "--profile-id",
        "short100_v2",
        "--postflight-suffix",
        SHORT100_RGB_SUFFIX,
        "--unit-pass-suffix",
        SHORT100_UNIT_SUFFIX,
        "--report-type",
        SHORT100_REPORT_TYPE,
        "--minimum-p95-improvement-mm",
        "0.03",
    ]


class I0FixedAbbaRgbAnalysisTest(unittest.TestCase):
    def test_non_object_report_is_invalid_evidence_not_an_exception(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "bad_i0_fixed_rgb_postflight.json"
            path.write_text("[]\n", encoding="utf-8")
            reports, paths, failures = MODULE.load_reports(root)
            self.assertEqual(reports, {})
            self.assertEqual(paths, {})
            self.assertTrue(any("root must be a JSON object" in item for item in failures))

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
            self.assertNotIn("deep_evidence_gate", output["input_contract"])
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

    def test_short100_profile_uses_only_its_own_artifact_suffixes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for row, p95, rms in (("01", 0.80, 0.50), ("02", 0.76, 0.49)):
                # A foreign v1 artifact in the same directory must not be pooled.
                (root / "V1_{}{}".format(row, MODULE.REPORT_SUFFIX)).write_text(
                    json.dumps(make_report(row, 9.0, 9.0)) + "\n",
                    encoding="utf-8",
                )
                write_short100_evidence(root, row, p95, rms)

            output_path = root / "SHORT100_ANALYSIS.json"
            completed = subprocess.run(
                short100_analyzer_command(root, output_path),
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            output = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(output["profile_id"], "short100_v2")
            self.assertEqual(output["report_type"], SHORT100_REPORT_TYPE)
            self.assertEqual(output["decision"], "PROMOTE_BLOCK2")
            self.assertEqual(
                output["input_contract"]["postflight_suffix"],
                SHORT100_RGB_SUFFIX,
            )
            self.assertTrue(output["input_contract"]["deep_evidence_gate"])
            self.assertTrue(
                all("S100_" in path for path in output["postflights"].values())
            )

    def test_short100_protocol_cannot_use_legacy_profile_to_skip_deep_gate(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            output_path = root / "BYPASS_ANALYSIS.json"
            command = short100_analyzer_command(root, output_path)
            profile_index = command.index("--profile-id") + 1
            command[profile_index] = "legacy_v1"
            completed = subprocess.run(
                command,
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 2)
            output = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(output["decision"], "INVALID_EVIDENCE")
            self.assertTrue(
                any(
                    "profile legacy_v1 protocol" in item
                    for item in output["failures"]
                )
            )

    def test_short100_rejects_wrong_marker_identity_and_tampered_artifact(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            evidence = write_short100_evidence(
                root,
                "01",
                0.80,
                0.50,
                marker_overrides={"variant": "B_slosh"},
            )
            write_short100_evidence(root, "02", 0.76, 0.49)
            evidence["observer"].write_text(
                '{"status":"TAMPERED"}\n', encoding="utf-8"
            )
            output_path = root / "SHORT100_ANALYSIS.json"
            completed = subprocess.run(
                short100_analyzer_command(root, output_path),
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 2)
            output = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(output["decision"], "INVALID_EVIDENCE")
            self.assertTrue(
                any("unit pass variant" in item for item in output["failures"])
            )
            self.assertTrue(
                any(
                    "observer_postflight_sha256" in item
                    for item in output["failures"]
                )
            )

    def test_short100_rejects_exact_report_with_wrong_deep_contract(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_short100_evidence(root, "01", 0.80, 0.50)
            write_short100_evidence(
                root,
                "02",
                0.76,
                0.49,
                exact_overrides={
                    "contracts": {
                        "expected_slosh_cost_horizon_steps": 60,
                    }
                },
            )
            output_path = root / "SHORT100_ANALYSIS.json"
            completed = subprocess.run(
                short100_analyzer_command(root, output_path),
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 2)
            output = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(output["decision"], "INVALID_EVIDENCE")
            self.assertTrue(
                any(
                    "contracts.expected_slosh_cost_horizon_steps=60" in item
                    for item in output["failures"]
                )
            )

    def test_short100_rejects_wrong_expected_effective_weight(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_short100_evidence(root, "01", 0.80, 0.50)
            write_short100_evidence(
                root,
                "02",
                0.76,
                0.49,
                exact_overrides={
                    "contracts": {
                        "expected_effective_config": {
                            "w_smooth": 0.1,
                            "w_alpha": 9.0,
                            "w_du_a": 0.1,
                            "w_du_vs": 0.1,
                            "slosh_height_max": 0.001,
                            "alpha_max": 1.2,
                        }
                    }
                },
            )
            output_path = root / "SHORT100_ANALYSIS.json"
            completed = subprocess.run(
                short100_analyzer_command(root, output_path),
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 2)
            output = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertTrue(
                any(
                    "expected_effective_config.w_alpha=9.0" in item
                    for item in output["failures"]
                )
            )

    def test_cli_rejects_unsafe_artifact_suffix(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            completed = subprocess.run(
                [
                    sys.executable,
                    str(ANALYZER),
                    "--root",
                    temp_dir,
                    "--postflight-suffix",
                    "../*.json",
                ],
                check=False,
                text=True,
                capture_output=True,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn("invalid --postflight-suffix", completed.stderr)


if __name__ == "__main__":
    unittest.main()
