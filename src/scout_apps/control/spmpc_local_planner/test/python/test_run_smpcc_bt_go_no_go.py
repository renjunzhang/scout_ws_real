#!/usr/bin/env python3

import hashlib
import importlib.util
import json
import pathlib
import subprocess
import tempfile
import unittest
from unittest import mock


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[2]
CAMPAIGN = (
    PACKAGE_ROOT / "tools" / "simulation" / "run_smpcc_bt_go_no_go.py")
SPEC = importlib.util.spec_from_file_location("run_smpcc_bt_go_no_go", CAMPAIGN)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def file_sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def valid_summary(key, seed):
    mode, disturbance = MODULE.TRIAL_CONTRACTS[key]
    return {
        "mode": mode,
        "seed": seed,
        "go_no_go_disturbance": {"id": disturbance},
        "final_command_transaction": True,
        "command_history_source": "final_published_command",
        "dual_channel_execution_model": True,
        "trial": {
            "sequence_completed": True,
            "task_success": True,
            "runtime_error": "",
        },
        "controller_audit": {
            "publications": 100,
            "publication_failures": 0,
            "receipt_inconsistent_cycles": 0,
            "history_not_committed_cycles": 0,
            "controlled_stops": 0,
        },
        "smpcc_bt_go_no_go_audit": {
            "minimum_effective_correction_fraction": 0.10,
            "bt_timed_reference_cycles": 10,
            "bt_reference_padded_stage_count": 4,
            "candidate_attempts": 10,
            "candidate_failures": 0,
            "correction_eligible_cycles": 10,
            "effective_correction_cycles": 2,
            "effective_correction_fraction": 0.20,
        },
    }


class SyntheticRunner:
    def __init__(self, fail_at=None):
        self.fail_at = fail_at
        self.calls = []
        self.environments = []
        self.filename_to_key = {
            filename: key for key, filename in MODULE.CONDITION_FILES.items()
        }

    def __call__(self, command, **kwargs):
        condition = pathlib.Path(command[command.index("--condition") + 1])
        key = self.filename_to_key[condition.name]
        seed = int(command[command.index("--seed") + 1])
        self.calls.append((key, seed))
        self.environments.append(kwargs.get("env"))
        payload = valid_summary(key, seed)
        if (key, seed) == self.fail_at:
            payload["smpcc_bt_go_no_go_audit"]["candidate_failures"] = 1
        summary_path = pathlib.Path(
            command[command.index("--summary-json") + 1])
        summary_path.write_text(json.dumps(payload), encoding="utf-8")
        cycle_path = pathlib.Path(command[command.index("--cycle-csv") + 1])
        cycle_path.write_text("synthetic\n", encoding="utf-8")
        return subprocess.CompletedProcess(command, 0, "synthetic", "")


class RunSmpccBtGoNoGoTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temporary.name)
        self.runner = self.root / "runner"
        self.plant = self.root / "plant.yaml"
        self.path = self.root / "path.json"
        self.artifact = self.root / "artifact.csv"
        for path, content in (
                (self.runner, b"runner"),
                (self.plant, b"plant"),
                (self.path, b"path"),
                (self.artifact, b"artifact")):
            path.write_bytes(content)
        self.frozen_hashes = {
            "plant": file_sha256(self.plant),
            "path": file_sha256(self.path),
            "artifact": file_sha256(self.artifact),
        }

    def tearDown(self):
        self.temporary.cleanup()

    def invoke(self, out_dir, fake_runner):
        source_freeze = {
            "baseline_head": MODULE.FROZEN_BASELINE_HEAD,
            "observed_head": MODULE.FROZEN_BASELINE_HEAD,
            "working_diff": {
                "sha256": "a" * 64,
                "algorithm": "spmpc-working-diff-v1",
                "scope": "src/scout_apps/control/spmpc_local_planner",
                "testing_directories_excluded": True,
                "untracked_paths": [],
            },
        }
        arguments = [
            "--out-dir", str(out_dir),
            "--plant", str(self.plant),
            "--path", str(self.path),
            "--artifact", str(self.artifact),
        ]
        with mock.patch.object(
                MODULE, "default_runner", return_value=self.runner), \
                mock.patch.object(
                    MODULE, "source_freeze_evidence",
                    return_value=source_freeze), \
                mock.patch.object(
                    MODULE, "FROZEN_INPUT_SHA256", self.frozen_hashes), \
                mock.patch.object(MODULE.subprocess, "run", fake_runner):
            return MODULE.main(arguments)

    def test_full_order_and_frozen_hash_manifest(self):
        output = self.root / "campaign"
        fake_runner = SyntheticRunner()
        result = self.invoke(output, fake_runner)

        expected = MODULE.campaign_steps("all")
        self.assertEqual(result, 0)
        self.assertEqual(fake_runner.calls, expected)
        self.assertEqual(len(expected), 26)
        self.assertEqual(
            [item for item in expected if item[0].startswith("monitor_")],
            [("monitor_d1", 9911)])
        manifest = json.loads(
            (output / "campaign_manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["status"], "COMPLETE_AWAITING_ANALYZER")
        self.assertEqual(
            manifest["source_freeze"]["baseline_head"],
            MODULE.FROZEN_BASELINE_HEAD)
        self.assertEqual(manifest["frozen_input_sha256"], self.frozen_hashes)
        for name, path in (
                ("runner_executable", self.runner),
                ("plant", self.plant),
                ("path", self.path),
                ("artifact", self.artifact),
                ("campaign_script", CAMPAIGN),
                ("analyzer_script", PACKAGE_ROOT / "tools" / "simulation" /
                 "analyze_smpcc_bt_go_no_go.py")):
            self.assertEqual(manifest["assets"][name]["sha256"],
                             file_sha256(path))
        self.assertIn("condition_monitor_d1", manifest["assets"])
        self.assertNotIn("condition_monitor_d2", manifest["assets"])
        expected_loader_path = ":".join((
            str(PACKAGE_ROOT.parents[3] /
                "devel/.private/spmpc_local_planner/lib"),
            "/home/a/acados/lib",
            "/opt/ros/noetic/lib",
            "/opt/ros/noetic/lib/x86_64-linux-gnu",
        ))
        self.assertEqual(
            manifest["runner_environment"]["LD_LIBRARY_PATH"],
            expected_loader_path)
        self.assertTrue(all(
            environment["LD_LIBRARY_PATH"] == expected_loader_path
            for environment in fake_runner.environments))

    def test_monitor_candidate_failure_writes_one_no_go_and_stops(self):
        output = self.root / "monitor_failure"
        fake_runner = SyntheticRunner(("monitor_d1", 9911))
        result = self.invoke(output, fake_runner)

        self.assertEqual(result, 4)
        self.assertEqual(fake_runner.calls, MODULE.campaign_steps("all")[:16])
        decisions = list(output.glob("*decision*.json"))
        self.assertEqual(len(decisions), 1)
        decision = json.loads(decisions[0].read_text(encoding="utf-8"))
        self.assertEqual(decision["decision"], "NO_GO_ROUTE_B")
        self.assertIn("monitor_d1/seed9911:CANDIDATE_FAILURE",
                      decision["reasons"])

    def test_direct_canary_failure_does_not_start_remaining_direct(self):
        output = self.root / "direct_failure"
        fake_runner = SyntheticRunner(("direct_d1", 9911))
        result = self.invoke(output, fake_runner)

        self.assertEqual(result, 4)
        self.assertEqual(fake_runner.calls, MODULE.campaign_steps("all")[:17])
        manifest = json.loads(
            (output / "campaign_manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["status"], "FAILED_STOPPED")
        self.assertTrue(manifest["monitor_canary_gate"]["passed"])
        self.assertNotIn("direct_canary_gate", manifest)

    def test_existing_output_directory_is_never_reused(self):
        output = self.root / "existing"
        output.mkdir()
        sentinel = output / "sentinel"
        sentinel.write_text("keep", encoding="utf-8")
        fake_runner = mock.Mock(side_effect=AssertionError("must not run"))

        with self.assertRaisesRegex(SystemExit, "refusing to reuse"):
            self.invoke(output, fake_runner)
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "keep")
        fake_runner.assert_not_called()

    def test_working_diff_hash_includes_untracked_and_excludes_testing(self):
        workspace = self.root / "workspace"
        package = workspace / "src/pkg"
        testing = package / "Testing"
        testing.mkdir(parents=True)
        keep = package / "keep.txt"
        ignored = testing / "ignored.txt"
        keep.write_text("one", encoding="utf-8")
        ignored.write_text("ignored-one", encoding="utf-8")

        def fake_git(_workspace, *arguments):
            if arguments[0] == "diff":
                return b"tracked-diff"
            if arguments[0] == "ls-files":
                return (b"src/pkg/keep.txt\0"
                        b"src/pkg/Testing/ignored.txt\0")
            raise AssertionError(arguments)

        with mock.patch.object(MODULE, "_git_bytes", side_effect=fake_git):
            first = MODULE.working_diff_evidence(workspace, package)
            ignored.write_text("ignored-two", encoding="utf-8")
            second = MODULE.working_diff_evidence(workspace, package)
            keep.write_text("two", encoding="utf-8")
            third = MODULE.working_diff_evidence(workspace, package)
        self.assertEqual(first["sha256"], second["sha256"])
        self.assertNotEqual(second["sha256"], third["sha256"])
        self.assertEqual(first["untracked_paths"], ["src/pkg/keep.txt"])

    def test_runtime_and_candidate_contract_failures_are_explicit(self):
        bt = valid_summary("bt_nominal", 9911)
        bt["trial"]["runtime_error"] = "publication failed"
        self.assertIn(
            "bt_nominal/seed9911:TRIAL_DID_NOT_COMPLETE",
            MODULE.trial_contract_reasons(bt, "bt_nominal", 9911))

        direct = valid_summary("direct_d1", 9911)
        direct["smpcc_bt_go_no_go_audit"].update({
            "bt_timed_reference_cycles": 9,
            "bt_reference_padded_stage_count": 0,
            "effective_correction_cycles": 0,
            "effective_correction_fraction": 0.0,
        })
        reasons = MODULE.candidate_contract_reasons(
            direct, "direct_d1", 9911)
        self.assertIn("direct_d1/seed9911:REFERENCE_CYCLE_MISMATCH", reasons)
        self.assertIn("direct_d1/seed9911:TERMINAL_PADDING_CONTRACT", reasons)
        self.assertIn("direct_d1/seed9911:NO_EFFECTIVE_CORRECTION", reasons)

        monitor = valid_summary("monitor_d1", 9911)
        monitor["controller_audit"]["publication_failures"] = 1
        self.assertIn(
            "monitor_d1/seed9911:PUBLICATION_SEMANTICS_CONTRACT",
            MODULE.candidate_contract_reasons(
                monitor, "monitor_d1", 9911))


if __name__ == "__main__":
    unittest.main()
