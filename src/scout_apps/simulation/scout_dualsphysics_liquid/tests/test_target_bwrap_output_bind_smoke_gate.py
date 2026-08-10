#!/usr/bin/env python3
"""Mock-only tests for the MSI one-marker writable-output bwrap smoke gate."""

from __future__ import annotations

import copy
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_bwrap_output_bind_smoke_gate as output_gate  # noqa: E402


class TargetBwrapOutputBindSmokePolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(output_gate.POLICY_PATH.read_text(encoding="utf-8"))
        self.schema = json.loads(output_gate.SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_checked_in_policy_is_one_marker_only(self) -> None:
        self.assertEqual(output_gate.validate_policy(self.policy), [])
        self.assertEqual(output_gate.validate_schema(self.schema), [])
        self.assertTrue(self.policy["output_bind_smoke_authorized"])
        self.assertFalse(self.policy["upstream_code_execution_authorized"])
        self.assertEqual(self.policy["attempt"]["marker_size_bytes"], 0)
        self.assertEqual(self.policy["isolation"]["host_writable_mounts"], ["<ATTEMPT_OUTPUT_DIRECTORY>"])
        self.assertTrue(self.policy["isolation"]["require_apparmor_restrict_unprivileged_userns_zero"])

    def test_policy_rejects_network_source_mount_and_other_command(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["isolation"]["network"] = "host"
        changed["isolation"]["host_read_only_mounts"].append("/home/zrj/scout_liquid_lab/dependency")
        changed["isolation"]["bwrap_argv_template"][-3] = "/bin/sh"
        changed["attempt"]["marker_size_bytes"] = 1
        changed["invariants"].pop("no_compiler")
        errors = output_gate.validate_policy(changed)
        self.assertIn("isolation or bwrap argv policy differs", errors)
        self.assertIn("attempt path or marker policy differs", errors)
        self.assertIn("output-bind smoke invariants are incomplete", errors)

    def test_guarded_argv_has_one_bind_and_literal_touch(self) -> None:
        command = output_gate.guarded_argv(self.policy, Path("/safe/output"))
        self.assertEqual(command[0], "/usr/bin/timeout")
        self.assertIn("--nproc=4096:4096", command)
        self.assertEqual(command.count("--bind"), 1)
        self.assertNotIn("--dev-bind", command)
        self.assertIn("--unshare-net", command)
        self.assertEqual(command[-3:], ("/usr/bin/touch", "--", "/work/output/.r8_output_bind_smoke_v1"))
        self.assertNotIn("/home/zrj/scout_liquid_lab/dependency", command)

    def test_generated_paths_require_new_fixed_names(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-output-bind-path-") as temporary:
            root = Path(temporary) / "root"
            (root / "build").mkdir(parents=True)
            (root / "audits").mkdir()
            paths = output_gate.validate_attempt_paths(root, "20260806T120000Z", self.policy)
            self.assertEqual(paths["attempt"].parent, root / "build")
            self.assertEqual(paths["output"], paths["attempt"] / "output")
            self.assertEqual(paths["marker"].name, ".r8_output_bind_smoke_v1")
            paths["receipt"].write_text("reserved", encoding="utf-8")
            with self.assertRaises(output_gate.OutputBindSmokeError):
                output_gate.validate_attempt_paths(root, "20260806T120000Z", self.policy)

    def test_run_is_mocked_and_verifies_only_one_empty_marker(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-output-bind-run-") as temporary:
            root = Path(temporary) / "root"
            (root / "build").mkdir(parents=True)
            (root / "audits").mkdir()
            preflight = {"status": "PASS_OUTPUT_BIND_SMOKE_ADMITTED", "preflight_sha256": "f" * 64}
            profile = {"storage": {"approved_root": str(root), "owner": "zrj"}}
            execution = {
                "returncode": 0,
                "hard_timeout_triggered": False,
                "hard_kill_for_stderr_limit": False,
                "stderr_max_bytes": 4096,
                "stderr_captured_bytes": 0,
                "stderr_sha256": "0" * 64,
                "stderr_utf8": "",
                "executor_error": None,
                "elapsed_seconds": 0.01,
            }

            def fake_run(command: tuple[str, ...], **_: object) -> dict[str, object]:
                marker = Path(command[-1])
                self.assertEqual(marker, Path("/work/output/.r8_output_bind_smoke_v1"))
                # The real output path is known from the frozen attempt name and is only used in this temp fixture.
                real_marker = root / "build/u3_bwrap_output_bind_smoke_v1_20260806T120001Z/output/.r8_output_bind_smoke_v1"
                descriptor = os.open(real_marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                os.close(descriptor)
                return execution

            with mock.patch.object(output_gate, "build_preflight", return_value=preflight), mock.patch.object(
                output_gate.profile_gate, "_read_json", return_value=profile
            ), mock.patch.object(output_gate.base_smoke, "_run_with_bounded_stderr", side_effect=fake_run):
                receipt = output_gate.run_output_bind_smoke(self.policy, timestamp="20260806T120001Z")
            self.assertEqual(receipt["status"], "PASS_OUTPUT_BIND_SMOKE")
            self.assertEqual(receipt["marker_verification"]["entries"], [".r8_output_bind_smoke_v1"])
            self.assertTrue((root / "audits/u3_bwrap_output_bind_smoke_v1_20260806T120001Z.json").is_file())
            self.assertTrue((root / "audits/u3_bwrap_output_bind_smoke_v1_20260806T120001Z.json.partial").is_file())
            self.assertFalse(receipt["source_checkout_created"])
            self.assertFalse(receipt["upstream_code_executed"])

    def test_no_go_preflight_writes_no_attempt_directory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-output-bind-nogo-") as temporary:
            root = Path(temporary) / "root"
            (root / "build").mkdir(parents=True)
            (root / "audits").mkdir()
            profile = {"storage": {"approved_root": str(root)}}
            preflight = {"status": "NO_GO_OUTPUT_BIND_SMOKE", "errors": ["mock AppArmor mismatch"]}
            with mock.patch.object(output_gate, "build_preflight", return_value=preflight), mock.patch.object(
                output_gate.profile_gate, "_read_json", return_value=profile
            ):
                receipt = output_gate.run_output_bind_smoke(self.policy, timestamp="20260806T120002Z")
            self.assertEqual(receipt["status"], "NO_GO_STATIC_PRECHECK")
            self.assertFalse((root / "build/u3_bwrap_output_bind_smoke_v1_20260806T120002Z").exists())
            self.assertTrue((root / "audits/u3_bwrap_output_bind_smoke_v1_20260806T120002Z.json").is_file())


if __name__ == "__main__":
    unittest.main()
