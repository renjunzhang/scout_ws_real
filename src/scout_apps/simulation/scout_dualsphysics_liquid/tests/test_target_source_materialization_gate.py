#!/usr/bin/env python3
"""Mock-only tests for the MSI static source materialization gate."""

from __future__ import annotations

import copy
import hashlib
import io
import json
import stat
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_source_materialization_gate as materialization_gate  # noqa: E402


class _FakePopen:
    """Completed in-memory process; it never starts a host command."""

    def __init__(self, stdout: bytes, stderr: bytes = b"", returncode: int = 0) -> None:
        self.stdout = io.BytesIO(stdout)
        self.stderr = io.BytesIO(stderr)
        self.returncode = returncode
        self.pid = 424242

    def poll(self) -> int:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:  # noqa: ARG002
        return self.returncode


class TargetSourceMaterializationPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(materialization_gate.POLICY_PATH.read_text(encoding="utf-8"))

    def test_checked_in_policy_is_source_only_and_build_remains_no_go(self) -> None:
        self.assertEqual(materialization_gate.validate_policy(self.policy), [])
        schema = json.loads(materialization_gate.SCHEMA_PATH.read_text(encoding="utf-8"))
        self.assertEqual(materialization_gate._validate_schema(schema), [])
        self.assertTrue(self.policy["static_source_materialization_authorized"])
        self.assertFalse(self.policy["build_execution_authorized"])
        self.assertFalse(self.policy["upstream_code_execution_authorized"])
        self.assertEqual(self.policy["source"]["allowed_source_prefix"], "src/source/")

    def test_policy_rejects_network_checkout_and_writable_relaxations(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["source"]["no_network"] = False
        changed["source"]["no_checkout"] = False
        changed["output"]["file_mode_after_seal_octal"] = "0640"
        changed["invariants"]["new_files_non_executable"] = False
        errors = materialization_gate.validate_policy(changed)
        self.assertIn("source materialization boundary differs from the frozen values", errors)
        self.assertIn("create-new or sealed-output policy differs from the frozen contract", errors)
        self.assertIn("source materialization safety invariants are incomplete", errors)

    def test_attempt_paths_are_fixed_and_reject_arbitrary_names(self) -> None:
        root = Path("/home/zrj/scout_liquid_lab")
        attempt_id = "u3_source_materialization_v1_20260806T160000Z"
        paths = materialization_gate._attempt_paths(root, self.policy, attempt_id)
        self.assertEqual(
            paths["attempt_root"],
            root / "dependency/materialized/u3_source_materialization_v1_20260806T160000Z.partial",
        )
        self.assertEqual(
            paths["receipt"],
            root / "audits/u3_source_materialization_v1_20260806T160000Z.json",
        )
        with self.assertRaises(materialization_gate.SourceMaterializationError):
            materialization_gate._attempt_paths(root, self.policy, "../not-an-attempt")

    def test_mock_listing_rejects_symlink_before_any_blob_copy(self) -> None:
        raw = (
            b"120000 blob 1111111111111111111111111111111111111111 8\t"
            b"src/source/unsafe-link\0"
        )
        changed = copy.deepcopy(self.policy)
        changed["source_subset_expectations"] = {
            "source_tree_listing_sha256": hashlib.sha256(raw).hexdigest(),
            "entry_count": 1,
            "total_blob_bytes": 8,
            "symlink_entry_count": 0,
            "gitlink_entry_count": 0,
            "input_executable_mode_paths": [],
        }
        result = {
            "returncode": 0,
            "executor_error": None,
            "hard_timeout_triggered": False,
            "hard_kill_for_output_limit": False,
            "stdout_utf8": raw.decode("utf-8"),
            "stdout_bytes": len(raw),
            "stderr_bytes": 0,
            "stdout_sha256": hashlib.sha256(raw).hexdigest(),
            "stderr_sha256": hashlib.sha256(b"").hexdigest(),
            "stderr_utf8": "",
            "argv": ["/usr/bin/git"],
            "output_max_bytes": 262144,
            "elapsed_seconds": 0.0,
        }
        with mock.patch.object(
            materialization_gate.fetch_gate, "_run_owned_bounded", return_value=result
        ) as runner:
            with self.assertRaisesRegex(
                materialization_gate.SourceMaterializationError, "symlink entries are forbidden"
            ):
                materialization_gate._read_source_tree(Path("/mock/source.git"), changed)
        runner.assert_called_once()

    def test_mock_blob_copy_forces_non_executable_file_and_never_runs_source(self) -> None:
        payload = b"int main() { return 0; }\n"
        entry = {
            "path": "src/source/example.cpp",
            "output_relative_path": "example.cpp",
            "original_mode": "100755",
            "object_id": "a" * 40,
            "size_bytes": len(payload),
        }
        with tempfile.TemporaryDirectory(prefix="r8-materialization-test-") as temporary:
            destination = Path(temporary) / "example.cpp"
            fake = _FakePopen(payload)
            with mock.patch.object(
                materialization_gate.subprocess, "Popen", return_value=fake
            ) as popen:
                result = materialization_gate._stream_blob_to_new_file(
                    Path("/mock/source.git"),
                    entry,
                    destination,
                    self.policy,
                    time.monotonic() + 5,
                )
            popen.assert_called_once()
            materialization_gate._verify_blob_stream(result, len(payload))
            self.assertEqual(destination.read_bytes(), payload)
            self.assertEqual(stat.S_IMODE(destination.stat().st_mode), 0o640)
            self.assertFalse(destination.stat().st_mode & 0o111)
            self.assertEqual(result["argv"][-2:], ["blob", "a" * 40])

    def test_mock_elf_magic_is_rejected_without_a_successful_stream(self) -> None:
        payload = b"\x7fELFbad"
        entry = {
            "path": "src/source/unexpected.bin",
            "output_relative_path": "unexpected.bin",
            "original_mode": "100644",
            "object_id": "b" * 40,
            "size_bytes": len(payload),
        }
        with tempfile.TemporaryDirectory(prefix="r8-materialization-elf-test-") as temporary:
            destination = Path(temporary) / "unexpected.bin"
            with mock.patch.object(
                materialization_gate.subprocess, "Popen", return_value=_FakePopen(payload)
            ):
                result = materialization_gate._stream_blob_to_new_file(
                    Path("/mock/source.git"),
                    entry,
                    destination,
                    self.policy,
                    time.monotonic() + 5,
                )
            self.assertIn("ELF magic", str(result["failure"]))
            with self.assertRaises(materialization_gate.SourceMaterializationError):
                materialization_gate._verify_blob_stream(result, len(payload))


if __name__ == "__main__":
    unittest.main()
