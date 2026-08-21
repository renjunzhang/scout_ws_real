#!/usr/bin/env python3

import contextlib
import copy
import csv
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = PACKAGE_ROOT / "tools" / "simulation" / \
    "run_independent_plant_campaign.py"
CONFIG_PATH = PACKAGE_ROOT / "config" / "simulation" / \
    "phase_rejoin_development_v1.yaml"
PLANNER_PATH = PACKAGE_ROOT / "config" / "planner" / "common.yaml"

SPEC = importlib.util.spec_from_file_location(
    "run_independent_plant_campaign", TOOL_PATH)
CAMPAIGN = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(CAMPAIGN)


class IndependentPlantCampaignTest(unittest.TestCase):

    @staticmethod
    def complete_formal_config():
        config = copy.deepcopy(CAMPAIGN.load_yaml(CONFIG_PATH))
        config["status"] = CAMPAIGN.FORMAL_STATUS
        config["provenance"]["runtime_verified"] = True
        config["formal_simulation"] = {
            "conditions": {
                name: {"binding_id": "formal_{}".format(name.lower())}
                for name in ("C0", "C1", "C2", "C3", "C4", "IS")
            },
            "held_out_execution_compatibility_report":
                "/frozen/held_out_gate.json",
            "held_out_execution_compatibility_report_sha256": "b" * 64,
        }
        return config

    @staticmethod
    def complete_formal_planner():
        planner = copy.deepcopy(CAMPAIGN.load_yaml(PLANNER_PATH))
        planner["delay_augmented_phase"][
            "expected_recovery_artifact_hash"] = "a" * 64
        planner["phase_rejoin"]["artifact_path"] = \
            "/frozen/formal_nominal.csv"
        return planner

    def test_frozen_config_is_simulation_only(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        CAMPAIGN.require_simulation_development_config(config)
        self.assertEqual(config["freeze_id"], "SIM_EXEC_PLANAR_R03_DEV_V1")
        self.assertEqual(
            config["status"], "development_candidate_unbound")
        self.assertEqual(config["experiment"]["control_rate_hz"], 30.0)
        self.assertEqual(config["experiment"]["fixed_tail_sec"], 4.0)
        self.assertFalse(
            config["scope"]["real_robot_enforce_allowed"])
        self.assertEqual(len(config["experiment"]["pilot_seeds"]), 8)
        self.assertTrue(config["experiment"]["formal_seeds_locked"])
        self.assertFalse(config["provenance"]["runtime_verified"])
        self.assertTrue(
            config["provenance"]["identification_known_bias"])
        self.assertEqual(
            config["provenance"]["corrected_tool_schema"],
            "spmpc_planar_execution_identification_v2")
        self.assertNotIn("controller_internal_model", config)
        self.assertNotIn(
            "real_robot_launch_overrides", config["experiment"])

    def test_physical_authorization_is_rejected(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        config["scope"]["real_robot_enforce_allowed"] = True
        with self.assertRaisesRegex(ValueError, "physical use"):
            CAMPAIGN.require_simulation_development_config(config)

    def test_seed_types_duplicates_and_cross_group_overlap_are_rejected(self):
        base = CAMPAIGN.load_yaml(CONFIG_PATH)
        cases = []

        value = copy.deepcopy(base)
        value["experiment"]["smoke_seed"] = True
        cases.append((value, "invalid smoke seed"))

        value = copy.deepcopy(base)
        value["experiment"]["pilot_seeds"][0] = False
        cases.append((value, "invalid pilot seed"))

        value = copy.deepcopy(base)
        value["experiment"]["reserved_formal_seeds"][0] = True
        cases.append((value, "invalid formal seed"))

        value = copy.deepcopy(base)
        value["experiment"]["smoke_seed"] = 1 << 32
        cases.append((value, "invalid smoke seed"))

        value = copy.deepcopy(base)
        value["experiment"]["pilot_seeds"][0] = 1 << 32
        cases.append((value, "invalid pilot seed"))

        value = copy.deepcopy(base)
        value["experiment"]["reserved_formal_seeds"][0] = 1 << 32
        cases.append((value, "invalid formal seed"))

        value = copy.deepcopy(base)
        value["experiment"]["pilot_seeds"][1] = \
            value["experiment"]["pilot_seeds"][0]
        cases.append((value, "duplicate pilot seed"))

        for left, right in (("smoke", "pilot"),
                            ("smoke", "formal"),
                            ("pilot", "formal")):
            value = copy.deepcopy(base)
            experiment = value["experiment"]
            smoke = experiment["smoke_seed"]
            pilot = experiment["pilot_seeds"][0]
            formal = experiment["reserved_formal_seeds"][0]
            selected = {"smoke": smoke, "pilot": pilot, "formal": formal}
            if left == "smoke":
                experiment["smoke_seed"] = selected[right]
            elif left == "pilot":
                experiment["pilot_seeds"][0] = selected[right]
            cases.append((value, "seeds overlap"))

        for config, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(ValueError, message):
                    CAMPAIGN.require_simulation_development_config(config)

    def test_unlocked_formal_seeds_are_rejected(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        config["experiment"]["formal_seeds_locked"] = False
        with self.assertRaisesRegex(ValueError, "not locked"):
            CAMPAIGN.require_simulation_development_config(config)

    def test_smoke_timing_waits_for_zero_command_arrival_and_uses_ceil(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        timing = CAMPAIGN.expected_smoke_timing(config)
        self.assertAlmostEqual(timing["zero_command_publish_sec"], 7.5)
        self.assertAlmostEqual(timing["tail_window_start_sec"], 7.606)
        self.assertAlmostEqual(timing["duration_sec"], 11.606)
        self.assertEqual(timing["samples"], 349)
        self.assertEqual(timing["tail_samples"], 121)

        changed = copy.deepcopy(config)
        changed["experiment"]["control_rate_hz"] = 17.0
        changed["experiment"]["fixed_tail_sec"] = 1.25
        changed_timing = CAMPAIGN.expected_smoke_timing(changed)
        self.assertAlmostEqual(
            changed_timing["zero_command_publish_sec"], 128.0 / 17.0)
        self.assertAlmostEqual(
            changed_timing["tail_window_start_sec"], 128.0 / 17.0 + 0.106)
        self.assertNotEqual(changed_timing["samples"], timing["samples"])
        self.assertAlmostEqual(
            changed_timing["tail_window_end_sec"] -
            changed_timing["tail_window_start_sec"], 1.25)

    def test_output_scope_is_restricted_to_identification_root(self):
        accepted = CAMPAIGN.ensure_output_scope(
            Path("/data/a/spmpc_exec_identification/unit_test"))
        self.assertEqual(
            accepted,
            Path("/data/a/spmpc_exec_identification/unit_test"))
        with self.assertRaisesRegex(ValueError, "must remain"):
            CAMPAIGN.ensure_output_scope(Path("/media/a/ZRJ/forbidden"))

    def test_failed_executable_preflight_does_not_consume_output_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "not_consumed"
            with mock.patch.object(CAMPAIGN, "ALLOWED_OUTPUT_ROOT", root), \
                    contextlib.redirect_stderr(io.StringIO()):
                result = CAMPAIGN.main([
                    "--campaign", "smoke",
                    "--config", str(CONFIG_PATH),
                    "--executable", str(root / "missing_executable"),
                    "--out-dir", str(output),
                ])
            self.assertEqual(result, 2)
            self.assertFalse(output.exists())

    def test_existing_campaign_directory_is_not_reused(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "existing"
            output.mkdir()
            sentinel = output / "sentinel"
            sentinel.write_text("keep\n", encoding="utf-8")
            executable = root / "unused_executable"
            executable.write_text("#!/bin/sh\nexit 99\n", encoding="utf-8")
            executable.chmod(0o755)
            stderr = io.StringIO()
            with mock.patch.object(CAMPAIGN, "ALLOWED_OUTPUT_ROOT", root), \
                    contextlib.redirect_stderr(stderr):
                result = CAMPAIGN.main([
                    "--campaign", "smoke",
                    "--config", str(CONFIG_PATH),
                    "--executable", str(executable),
                    "--out-dir", str(output),
                ])
            self.assertEqual(result, 2)
            self.assertIn("already exists", stderr.getvalue())
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "keep\n")

    def test_current_formal_request_is_explicit_no_go(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        planner = CAMPAIGN.load_yaml(PLANNER_PATH)
        reasons = CAMPAIGN.formal_no_go_reasons(config, planner)
        self.assertIn("simulation config is a development candidate", reasons)
        self.assertIn("C0-C4/IS condition bindings are absent", reasons)
        self.assertIn("frozen recovery artifact hash is absent", reasons)
        self.assertIn("formal nominal/tail artifact path is absent", reasons)
        self.assertIn(
            "held-out execution compatibility/gate report is absent", reasons)

    def test_incomplete_formal_release_remains_no_go(self):
        config = self.complete_formal_config()
        del config["formal_simulation"]["conditions"]["C4"]
        planner = self.complete_formal_planner()
        reasons = CAMPAIGN.formal_no_go_reasons(config, planner)
        self.assertIn("C0-C4/IS condition bindings are absent", reasons)

    def test_complete_formal_audit_reaches_ready_without_running_trials(self):
        config = self.complete_formal_config()
        planner = self.complete_formal_planner()
        self.assertEqual(CAMPAIGN.formal_no_go_reasons(config, planner), [])
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "formal.yaml"
            planner_path = root / "planner.yaml"
            config_path.write_text(
                yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
            planner_path.write_text(
                yaml.safe_dump(planner, sort_keys=False), encoding="utf-8")
            output = root / "formal_audit"
            stdout = io.StringIO()
            with mock.patch.object(CAMPAIGN, "ALLOWED_OUTPUT_ROOT", root), \
                    mock.patch.object(CAMPAIGN.subprocess, "run") as run, \
                    contextlib.redirect_stdout(stdout):
                result = CAMPAIGN.main([
                    "--campaign", "formal",
                    "--config", str(config_path),
                    "--planner-config", str(planner_path),
                    "--out-dir", str(output),
                ])
            self.assertEqual(result, 5)
            run.assert_not_called()
            self.assertIn("does not execute C0-C4", stdout.getvalue())
            with (output / "formal_readiness.json").open(
                    "r", encoding="utf-8") as stream:
                report = json.load(stream)
            self.assertEqual(report["status"], "READY_NOT_EXECUTED")
            self.assertFalse(report["formal_trials_started"])
            self.assertEqual(report["reasons"], [])

    def test_metric_summary_is_trial_level(self):
        summary = CAMPAIGN.metric_summary([3.0, 1.0, 2.0, 4.0])
        self.assertEqual(summary, {"min": 1.0, "median": 2.5, "max": 4.0})

    @unittest.skipUnless(
        os.environ.get("SPMPC_INDEPENDENT_PLANT_SMOKE"),
        "set SPMPC_INDEPENDENT_PLANT_SMOKE for the C++ integration contract")
    def test_real_smoke_contract_changes_timing_and_refuses_overwrite(self):
        executable = Path(os.environ["SPMPC_INDEPENDENT_PLANT_SMOKE"])
        self.assertTrue(executable.is_file())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = CAMPAIGN.load_yaml(CONFIG_PATH)
            config["experiment"]["control_rate_hz"] = 17.0
            config["experiment"]["fixed_tail_sec"] = 1.25
            config_path = root / "changed_timing.yaml"
            config_path.write_text(
                yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
            output = root / "campaign"
            with mock.patch.object(CAMPAIGN, "ALLOWED_OUTPUT_ROOT", root):
                self.assertEqual(CAMPAIGN.main([
                    "--campaign", "smoke",
                    "--config", str(config_path),
                    "--executable", str(executable),
                    "--out-dir", str(output),
                ]), 0)

            timing = CAMPAIGN.expected_smoke_timing(config)
            summary_path = output / "seed1001.json"
            csv_path = output / "seed1001.csv"
            with summary_path.open("r", encoding="utf-8") as stream:
                summary = json.load(stream)
            self.assertEqual(
                summary["schema"], "spmpc_independent_plant_smoke_v3")
            self.assertEqual(
                summary["csv_timestamp_contract"],
                "publish_sample_effective_epochs_v1")
            self.assertEqual(summary["samples"], timing["samples"])
            self.assertEqual(summary["tail_samples"], timing["tail_samples"])
            self.assertAlmostEqual(
                summary["tail_window_start_sec"],
                timing["tail_window_start_sec"])
            self.assertAlmostEqual(summary["duration_sec"],
                                   timing["duration_sec"])
            with (output / "campaign_manifest.json").open(
                    "r", encoding="utf-8") as stream:
                manifest = json.load(stream)
            self.assertEqual(manifest["status"],
                             "COMPLETE_DEVELOPMENT_SMOKE")
            self.assertEqual(
                manifest["completion_scope"],
                "execution_and_artifact_integrity_only")
            self.assertFalse(manifest["effect_claim"])
            self.assertEqual(
                manifest["schema"], "spmpc_independent_plant_campaign_v3")

            with csv_path.open(
                    "r", encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(tuple(reader.fieldnames or ()),
                                 CAMPAIGN.CSV_COLUMNS)
                first = next(reader)
            self.assertAlmostEqual(float(first["publish_time_sec"]), 0.0)
            self.assertAlmostEqual(
                float(first["sample_time_sec"]), 1.0 / 17.0)
            for channel in ("linear", "angular"):
                publish = float(first["publish_time_sec"])
                jitter = float(
                    first[channel + "_transport_jitter_sec"])
                effective = float(first[channel + "_effective_time_sec"])
                self.assertAlmostEqual(
                    effective,
                    publish + config["external_plant"][channel]["delay_sec"] +
                    jitter)

            csv_hash = CAMPAIGN.sha256_file(csv_path)
            summary_hash = CAMPAIGN.sha256_file(summary_path)
            repeated = subprocess.run([
                str(executable), str(config_path),
                str(output / "seed1001"), "1001"],
                check=False, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
            self.assertEqual(repeated.returncode, 5)
            self.assertIn("refusing to overwrite", repeated.stderr)
            self.assertEqual(CAMPAIGN.sha256_file(csv_path), csv_hash)
            self.assertEqual(CAMPAIGN.sha256_file(summary_path), summary_hash)


if __name__ == "__main__":
    unittest.main()
