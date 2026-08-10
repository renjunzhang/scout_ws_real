#!/usr/bin/env python3
"""Mock-only tests for the MSI bwrap system-true smoke gate."""

from __future__ import annotations

import copy
import inspect
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_bwrap_smoke_gate as smoke_gate  # noqa: E402


class TargetBwrapSmokePolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(smoke_gate.POLICY_PATH.read_text(encoding="utf-8"))

    def test_checked_in_policy_is_exactly_system_true_only(self) -> None:
        self.assertEqual(smoke_gate.validate_policy(self.policy), [])
        self.assertTrue(self.policy["system_binary_smoke_authorized"])
        self.assertFalse(self.policy["upstream_code_execution_authorized"])
        self.assertEqual(smoke_gate.bwrap_argv(self.policy)[-1], "/usr/bin/true")
        self.assertNotIn("--bind", smoke_gate.bwrap_argv(self.policy))
        self.assertNotIn("--dev-bind", smoke_gate.bwrap_argv(self.policy))
        self.assertIn("--unshare-net", smoke_gate.bwrap_argv(self.policy))

    def test_policy_rejects_a_writable_mount_or_any_other_command(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["isolation"]["host_writable_mounts"] = ["/home/zrj/scout_liquid_lab"]
        changed["bwrap_argv"][-1] = "/bin/sh"
        errors = smoke_gate.validate_policy(changed)
        self.assertIn("isolation field mismatch: host_writable_mounts", errors)
        self.assertIn("bwrap argv differs from the only admitted system-true command", errors)

    def test_policy_rejects_network_relaxation_and_upstream_authorization(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["isolation"]["network"] = "host"
        changed["isolation"]["apparmor"] = "modified"
        changed["upstream_code_execution_authorized"] = True
        errors = smoke_gate.validate_policy(changed)
        self.assertIn("isolation field mismatch: network", errors)
        self.assertIn("isolation field mismatch: apparmor", errors)
        self.assertIn("policy field mismatch: upstream_code_execution_authorized", errors)

    def test_guarded_argv_has_hard_timeout_and_rlimits(self) -> None:
        argv = smoke_gate.guarded_argv(self.policy)
        self.assertEqual(argv[0], "/usr/bin/timeout")
        self.assertIn("30s", argv)
        self.assertIn("--cpu=10:10", argv)
        self.assertIn("--as=1073741824:1073741824", argv)
        self.assertIn("--nproc=4096:4096", argv)
        self.assertIn("--nofile=128:128", argv)
        self.assertEqual(argv[-1], "/usr/bin/true")

    def test_receipt_path_is_fixed_and_partial_reuse_is_refused(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-liquid-bwrap-smoke-") as temporary:
            root = Path(temporary) / "root"
            audits = root / "audits"
            audits.mkdir(parents=True)
            receipt = audits / "u2_bwrap_system_true_smoke_v3_20260806T120000Z.json"
            self.assertEqual(
                smoke_gate.validate_receipt_path(receipt, root, operation="smoke"),
                (receipt, Path(f"{receipt}.partial")),
            )
            with self.assertRaises(smoke_gate.BwrapSmokeGateError):
                smoke_gate.validate_receipt_path(root / "results" / receipt.name, root, operation="smoke")
            partial = Path(f"{receipt}.partial")
            partial.write_text("reserved", encoding="utf-8")
            with self.assertRaises(smoke_gate.BwrapSmokeGateError):
                smoke_gate.validate_receipt_path(receipt, root, operation="smoke")

    def test_run_uses_only_frozen_command_and_no_captured_output(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-liquid-bwrap-run-") as temporary:
            root = Path(temporary) / "root"
            audits = root / "audits"
            audits.mkdir(parents=True)
            receipt_path = audits / "u2_bwrap_system_true_smoke_v3_20260806T120001Z.json"
            preflight = {
                "status": "PASS_SYSTEM_TRUE_SMOKE_ADMITTED",
                "preflight_sha256": "f" * 64,
            }
            profile = {"storage": {"approved_root": str(root)}}
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
            with mock.patch.object(smoke_gate, "build_preflight", return_value=preflight), mock.patch.object(
                smoke_gate.profile_gate, "_read_json", return_value=profile
            ), mock.patch.object(
                smoke_gate, "_approved_root", return_value=root
            ), mock.patch.object(
                smoke_gate, "guarded_argv", return_value=("/usr/bin/timeout", "30s", "/usr/bin/true")
            ), mock.patch.object(
                smoke_gate, "_run_with_bounded_stderr", return_value=execution
            ) as run:
                receipt = smoke_gate.run_system_true_smoke(self.policy, receipt_path)
            self.assertEqual(receipt["status"], "PASS_SYSTEM_TRUE_BWRAP_SMOKE")
            run.assert_called_once()
            kwargs = run.call_args.kwargs
            self.assertEqual(kwargs["parent_timeout_seconds"], 40)
            self.assertEqual(kwargs["stderr_max_bytes"], 4096)
            self.assertTrue(Path(f"{receipt_path}.partial").is_file())
            self.assertTrue(receipt_path.is_file())
            self.assertFalse(receipt["upstream_code_executed"])
            self.assertTrue(receipt["namespace_attempted"])
            self.assertTrue(receipt["namespace_created"])

    def test_v2_diagnostic_evidence_and_process_headroom_are_frozen(self) -> None:
        self.assertEqual(self.policy["required_v2_diagnostic"]["returncode"], 1)
        self.assertEqual(self.policy["resources"]["process_limit"], 4096)
        self.assertEqual(self.policy["resources"]["minimum_process_headroom"], 512)
        changed = copy.deepcopy(self.policy)
        changed["required_v2_diagnostic"]["stderr_sha256"] = "0" * 64
        self.assertIn("v2 diagnostic field mismatch: stderr_sha256", smoke_gate.validate_policy(changed))

    def test_apparmor_restriction_is_observed_without_a_bypass(self) -> None:
        self.assertTrue(smoke_gate.apparmor_userns_is_admitted(0, "vscode (unconfined)"))
        self.assertFalse(smoke_gate.apparmor_userns_is_admitted(1, "vscode (unconfined)"))
        self.assertFalse(smoke_gate.apparmor_userns_is_admitted(0, "other-profile (enforce)"))
        self.assertIn(
            "if not apparmor_userns_is_admitted(", inspect.getsource(smoke_gate.build_preflight)
        )


if __name__ == "__main__":
    unittest.main()
