#!/usr/bin/env python3
"""Mock-only tests for the MSI offline full-source static inventory gate."""

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

import r8_liquid_target_source_static_inventory_gate as inventory_gate  # noqa: E402


class _FakePopen:
    """An in-memory completed process used to prove tests run no system Git."""

    def __init__(self, stdout: bytes, stderr: bytes = b"", returncode: int = 0) -> None:
        self.stdout = io.BytesIO(stdout)
        self.stderr = io.BytesIO(stderr)
        self.returncode = returncode
        self.pid = 424242

    def poll(self) -> int:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:  # noqa: ARG002
        return self.returncode


def _command_result(stdout: bytes) -> dict[str, object]:
    return {
        "argv": ["/usr/bin/git", "--no-replace-objects"],
        "returncode": 0,
        "hard_timeout_triggered": False,
        "hard_kill_for_output_limit": False,
        "output_max_bytes": 524288,
        "stdout_bytes": len(stdout),
        "stderr_bytes": 0,
        "stdout_sha256": "0" * 64,
        "stderr_sha256": "0" * 64,
        "stdout_utf8": stdout.decode("utf-8"),
        "stderr_utf8": "",
        "executor_error": None,
        "elapsed_seconds": 0.001,
    }


class TargetSourceStaticInventoryPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(inventory_gate.POLICY_PATH.read_text(encoding="utf-8"))

    def test_checked_in_policy_is_offline_only_and_exact(self) -> None:
        self.assertEqual(inventory_gate.validate_policy(self.policy), [])
        self.assertTrue(self.policy["static_blob_read_authorized"])
        self.assertFalse(self.policy["upstream_code_execution_authorized"])
        environment = inventory_gate._offline_git_environment()
        self.assertEqual(environment["GIT_ALLOW_PROTOCOL"], "none")
        self.assertEqual(environment["GIT_PROTOCOL_FROM_USER"], "0")
        self.assertEqual(self.policy["inventory_expectations"]["tree_entry_count"], 768)

    def test_policy_rejects_network_and_candidate_elf_relaxation(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["source"]["no_network"] = False
        changed["inventory_expectations"]["bin_linux_entries"][1]["expected_elf"] = False
        changed["invariants"].pop("no_network")
        errors = inventory_gate.validate_policy(changed)
        self.assertIn("source declaration differs from the frozen bare/offline source", errors)
        self.assertIn("static inventory expectations differ from the frozen source tree", errors)
        self.assertIn("offline static-inventory safety invariants are incomplete", errors)

    def test_tree_parser_accepts_only_clean_nul_delimited_records(self) -> None:
        policy = copy.deepcopy(self.policy)
        policy["inventory_expectations"] = {
            "tree_entry_count": 1,
            "total_blob_bytes": 12,
            "executable_mode_paths": [],
            "symlink_entry_count": 0,
            "gitlink_entry_count": 0,
            "bin_linux_entries": [],
        }
        listing = b"100644 blob aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa       12\tsrc/file.txt\0"
        with mock.patch.object(
            inventory_gate.fetch_gate, "_run_owned_bounded", return_value=_command_result(listing)
        ):
            evidence = inventory_gate._read_tree_inventory(Path("/mock/bare.git"), policy)
        self.assertEqual(evidence["inventory"]["tree_entry_count"], 1)
        self.assertEqual(evidence["inventory"]["total_blob_bytes"], 12)

        unsafe = b"100644 blob aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa       12\t../escape\0"
        with mock.patch.object(
            inventory_gate.fetch_gate, "_run_owned_bounded", return_value=_command_result(unsafe)
        ):
            with self.assertRaises(inventory_gate.StaticInventoryError):
                inventory_gate._read_tree_inventory(Path("/mock/bare.git"), policy)

    def test_blob_stream_is_mocked_and_classifies_elf_without_writing_content(self) -> None:
        blob = b"\x7fELF" + b"fake-static-metadata"
        fake = _FakePopen(blob)
        with mock.patch.object(inventory_gate.subprocess, "Popen", return_value=fake) as popen:
            evidence = inventory_gate._stream_candidate_blob(
                Path("/mock/bare.git"), "a" * 40, len(blob), self.policy
            )
        popen.assert_called_once()
        self.assertEqual(evidence["returncode"], 0)
        self.assertEqual(evidence["bytes_read"], len(blob))
        self.assertEqual(evidence["classification"], "ELF")
        self.assertFalse(evidence["hard_timeout_triggered"])
        inventory_gate._verify_candidate_stream(evidence, len(blob))

    def test_receipt_path_and_write_flow_refuse_reuse_without_real_source_access(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-liquid-static-inventory-") as temporary:
            root = Path(temporary) / "root"
            audits = root / "audits"
            audits.mkdir(parents=True)
            receipt_path = audits / "u2_full_source_static_inventory_v1_20260806T120000Z.json"
            self.assertEqual(
                inventory_gate.validate_receipt_path(receipt_path, root),
                (receipt_path, Path(f"{receipt_path}.partial")),
            )

            preflight = {
                "status": "PASS_OFFLINE_STATIC_INVENTORY_ADMITTED",
                "preflight_sha256": "f" * 64,
            }
            profile = {"storage": {"approved_root": str(root)}}
            tree = {"inventory": {"tree_entry_count": 768}}

            def fake_blob(_: Path, object_id: str, expected_size: int, __: dict[str, object]) -> dict[str, object]:
                non_elf = {
                    "d99d0f85d19a622f7faf2d5f1195b11f17951846",
                    "04ed5435c5a4898a91a1e4389e51e00ed4ea1d3b",
                    "92ffbc0b0ef63b353f6bf36a3e96d8630a5ae778",
                }
                return {
                    "returncode": 0,
                    "executor_error": None,
                    "hard_timeout_triggered": False,
                    "hard_kill_for_size_limit": False,
                    "hard_kill_for_stderr_limit": False,
                    "reader_errors": [],
                    "bytes_read": expected_size,
                    "sha256": "c" * 64,
                    "prefix_hex": "7f454c46" if object_id not in non_elf else "3c3f786d",
                    "classification": "NON_ELF" if object_id in non_elf else "ELF",
                }

            with mock.patch.object(inventory_gate, "build_preflight", return_value=preflight), mock.patch.object(
                inventory_gate.profile_gate, "_read_json", return_value=profile
            ), mock.patch.object(inventory_gate, "_approved_root", return_value=root), mock.patch.object(
                inventory_gate, "_read_tree_inventory", return_value=tree
            ), mock.patch.object(inventory_gate, "_stream_candidate_blob", side_effect=fake_blob):
                receipt = inventory_gate.write_static_inventory_receipt(self.policy, receipt_path)
            self.assertEqual(receipt["status"], "PASS_OFFLINE_FULL_SOURCE_STATIC_INVENTORY")
            self.assertFalse(receipt["network_used"])
            self.assertFalse(receipt["upstream_code_executed"])
            self.assertTrue(receipt_path.is_file())
            self.assertTrue(Path(f"{receipt_path}.partial").is_file())
            with self.assertRaises(inventory_gate.StaticInventoryError):
                inventory_gate.validate_receipt_path(receipt_path, root)


if __name__ == "__main__":
    unittest.main()
