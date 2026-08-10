#!/usr/bin/env python3
"""Mock-only tests for the MSI exact full-source fetch gate."""

from __future__ import annotations

import copy
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_source_fetch_gate as source_gate  # noqa: E402


class _FakePopen:
    """In-memory completed process; it never starts a real command."""

    def __init__(self, stdout: bytes, stderr: bytes = b"", returncode: int = 0) -> None:
        self.stdout = io.BytesIO(stdout)
        self.stderr = io.BytesIO(stderr)
        self.returncode = returncode
        self.pid = 424242

    def poll(self) -> int:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:  # noqa: ARG002
        return self.returncode


class TargetSourceFetchPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(source_gate.POLICY_PATH.read_text(encoding="utf-8"))

    def test_checked_in_policy_is_https_only_and_requires_full_object_walk(self) -> None:
        self.assertEqual(source_gate.validate_policy(self.policy), [])
        self.assertFalse(self.policy["upstream_code_execution_authorized"])
        self.assertEqual(
            self.policy["git_commands"]["object_walk"],
            [
                "rev-list",
                "--objects",
                "--missing=print",
                "--no-object-names",
                source_gate.EXPECTED_COMMIT,
            ],
        )
        self.assertEqual(self.policy["git_commands"]["fsck"][1], "--full")
        self.assertEqual(
            self.policy["trusted_system_tools"]["git_remote_https"]["resolved_path"],
            str(source_gate.GIT_REMOTE_HTTP),
        )

    def test_policy_rejects_network_relaxation_and_missing_object_walk(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["network"]["allowed_protocols"] = ["https", "ssh"]
        changed["git_commands"].pop("object_walk")
        changed["source"]["no_checkout"] = False
        errors = source_gate.validate_policy(changed)
        self.assertIn("network policy differs from the exact HTTPS-only fetch", errors)
        self.assertIn("Git argv policy differs from the frozen exact fetch", errors)
        self.assertIn("source field mismatch: no_checkout", errors)

    def test_receipt_and_attempt_paths_refuse_reuse(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-liquid-source-gate-") as temporary:
            root = Path(temporary) / "root"
            audits = root / "audits"
            parent = root / "dependency" / "source"
            audits.mkdir(parents=True)
            parent.mkdir(parents=True)
            receipt = audits / "u2_full_source_fetch_attempt_3_20260806T120000Z.json"
            self.assertEqual(
                source_gate.validate_receipt_path(receipt, root),
                (receipt, Path(f"{receipt}.partial")),
            )
            existing_attempt = parent / "already-exists.git"
            existing_attempt.mkdir()
            with self.assertRaises(source_gate.SourceFetchGateError):
                source_gate._reserve_new_attempt_directory(existing_attempt, 0o750)

    def test_mock_object_walk_counts_missing_objects_without_running_git(self) -> None:
        output = (
            f"{source_gate.EXPECTED_COMMIT}\n"
            f"{source_gate.EXPECTED_TREE}\n"
            f"{'a' * 40}\n"
            f"?{'b' * 40}\n"
        ).encode("ascii")
        fake = _FakePopen(output)
        with mock.patch.object(source_gate.subprocess, "Popen", return_value=fake) as popen:
            evidence = source_gate._scan_reachable_objects(Path("/mock/bare.git"), self.policy)
        popen.assert_called_once()
        self.assertEqual(evidence["returncode"], 0)
        self.assertTrue(evidence["expected_commit_seen"])
        self.assertTrue(evidence["expected_tree_seen"])
        self.assertEqual(evidence["present_object_count"], 3)
        self.assertEqual(evidence["missing_object_count"], 1)
        self.assertEqual(evidence["missing_object_ids_sample"], ["b" * 40])
        self.assertFalse(evidence["hard_timeout_triggered"])

    def test_full_bare_verification_rejects_any_missing_object(self) -> None:
        object_walk = {
            "returncode": 0,
            "executor_error": None,
            "hard_timeout_triggered": False,
            "hard_kill_for_stdout_limit": False,
            "hard_kill_for_record_limit": False,
            "hard_kill_for_malformed_record": False,
            "hard_kill_for_stderr_limit": False,
            "reader_errors": [],
            "expected_commit_seen": True,
            "expected_tree_seen": True,
            "missing_object_count": 1,
        }
        fsck = {
            "returncode": 0,
            "executor_error": None,
            "hard_timeout_triggered": False,
            "hard_kill_for_output_limit": False,
        }
        with tempfile.TemporaryDirectory(prefix="r8-liquid-full-verify-") as temporary:
            bare = Path(temporary) / "bare.git"
            bare.mkdir()
            with mock.patch.object(
                source_gate,
                "_git_text",
                side_effect=[source_gate.EXPECTED_TREE, source_gate.EXPECTED_URL],
            ), mock.patch.object(source_gate, "_run_owned_bounded", return_value=fsck), mock.patch.object(
                source_gate, "_scan_reachable_objects", return_value=object_walk
            ):
                with self.assertRaisesRegex(source_gate.SourceFetchGateError, "1 missing reachable Git objects"):
                    source_gate._verify_full_bare(bare, self.policy)


if __name__ == "__main__":
    unittest.main()
