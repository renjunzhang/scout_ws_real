#!/usr/bin/env python3
"""Static/mock-only tests for S5A1 execution supervisor v4."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_execution_supervisor_v4 as supervisor  # noqa: E402


class S5A1ExecutionSupervisorV4Tests(unittest.TestCase):
    def setUp(self) -> None:
        supervisor._SEMANTIC_REPORT = None
        supervisor._install_revision()

    def test_schema_is_deep_closed_and_mock_failure_validates(self) -> None:
        schema = json.loads(supervisor.RECEIPT_SCHEMA.read_bytes())
        Draft202012Validator.check_schema(schema)
        supervisor.gate_v3.assert_schema_deep_closed(schema)
        failure = supervisor.mock_failure_receipt()
        Draft202012Validator(schema).validate(failure)
        self.assertFalse(failure["semantic_validation"]["validated"])
        self.assertEqual(failure["execution"]["supplementary_groups"], [])
        changed = copy.deepcopy(failure)
        changed["semantic_validation"]["unexpected"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)

    def test_self_check_is_static_and_binds_exact_s5a0_parents(self) -> None:
        result = supervisor.static_self_check()
        self.assertEqual(result["status"], "PASS_S5A1_EXECUTION_SUPERVISOR_V4_STATIC_CONTRACT")
        self.assertEqual(result["execution_id"], supervisor.EXECUTION_ID)
        self.assertEqual(result["worker_supplementary_groups"], [])
        self.assertEqual(result["writable_binds"], [(str(supervisor.PARTIAL_ROOT), str(supervisor.PARTIAL_ROOT))])
        self.assertTrue(result["source_parent_present"])
        self.assertTrue(result["transfer_roots_fresh"])
        for name in ("real_bag_read", "optional_bag_read", "worker_run", "bwrap_run", "files_written", "network_used", "gpu_exposed", "ros_started", "solver_executed", "sudo_used"):
            self.assertFalse(result[name], name)
        parents = supervisor._parent_map()
        self.assertEqual(parents["s5a0_outer_final_receipt"]["sha256"], supervisor.S5A0_FINAL_SHA256)
        self.assertEqual(parents["s5a0_inner_receipt"]["sha256"], supervisor.S5A0_INNER_SHA256)
        self.assertEqual(parents["handoff_gate_v3"]["sha256"], supervisor.GATE_V3_SHA256)
        self.assertEqual(parents["signal_extractor_v3"]["sha256"], supervisor.EXTRACTOR_V3_SHA256)

    def test_bwrap_is_networkless_capability_free_and_only_partial_writable(self) -> None:
        source = Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag")
        argv = supervisor.build_bwrap_argv(supervisor.S5A0_INNER_RECEIPT, source, supervisor.PARTIAL_ROOT)
        joined = "\0".join(argv).lower()
        self.assertEqual(argv.count("--unshare-all"), 1)
        self.assertEqual(argv.count("--unshare-net"), 1)
        self.assertEqual(argv.count("--cap-drop"), 1)
        self.assertEqual(argv[argv.index("--cap-drop") + 1], "ALL")
        self.assertEqual(argv[argv.index("--uid") + 1], "1000")
        self.assertEqual(argv[argv.index("--gid") + 1], "1000")
        self.assertNotIn("--proc", argv)
        self.assertNotIn("--dev", argv)
        self.assertNotIn("/dev/nvidia", joined)
        self.assertNotIn("sudo", joined)
        writable = [(argv[index + 1], argv[index + 2]) for index, token in enumerate(argv) if token == "--bind"]
        self.assertEqual(writable, [(str(supervisor.PARTIAL_ROOT), str(supervisor.PARTIAL_ROOT))])
        path_indexes = [index for index, token in enumerate(argv) if token == "--setenv" and argv[index + 1] == "PATH"]
        self.assertEqual(path_indexes, [path_indexes[0]])
        self.assertEqual(argv[path_indexes[0] + 2], "/usr/bin")

    def test_same_fresh_transfer_root_and_new_receipt_identity_are_frozen(self) -> None:
        expected = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1"
        self.assertEqual(supervisor.TRANSFER_ID, expected)
        self.assertEqual(supervisor.PARTIAL_ROOT, Path(f"/home/zrj/scout_liquid_lab/incoming/{expected}.partial"))
        self.assertEqual(supervisor.FINAL_ROOT, Path(f"/home/zrj/scout_liquid_lab/incoming/{expected}"))
        self.assertEqual(supervisor.EXECUTION_RECEIPT.name, f"{expected}.execution_v4.json")
        self.assertEqual(supervisor.EXECUTION_ID, "s5a1_primary_bsmooth_b01_20260811T204651Z_v4")

    def test_host_load_calls_public_strong_directory_validator(self) -> None:
        report = {
            "status": "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS",
            "package_root": "/mock/package",
            "source_bag_read": False,
            "external_write": False,
        }
        with mock.patch.object(supervisor, "_BASE_LOAD_PACKAGE", return_value={"artifact": b"x"}) as structural:
            with mock.patch.object(supervisor.gate_v3, "validate_package_root", return_value=report) as semantic:
                result = supervisor.load_package(Path("/mock/package"))
        self.assertEqual(result, {"artifact": b"x"})
        structural.assert_called_once_with(Path("/mock/package"))
        semantic.assert_called_once_with(Path("/mock/package"), trusted_policy_path=supervisor.PACKAGE_POLICY)
        self.assertEqual(supervisor._SEMANTIC_REPORT, report)

    def test_directory_validator_failure_propagates_and_cannot_be_claimed(self) -> None:
        with mock.patch.object(supervisor, "_BASE_LOAD_PACKAGE", return_value={}):
            with mock.patch.object(supervisor.gate_v3, "validate_package_root", return_value={"status": "WRONG"}):
                with self.assertRaises(supervisor.ExecutionV4Error):
                    supervisor.load_package(Path("/mock/package"))
        candidate = supervisor._BASE_MOCK_FAILURE()
        candidate["status"] = "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
        candidate["failure"] = None
        candidate["next"] = "S5B0_REQUIRES_EXACT_TRANSFER_HASH_AND_NEW_GPU_AUTHORIZATION"
        with self.assertRaisesRegex(supervisor.ExecutionV4Error, "lacks host strong semantic"):
            supervisor.transform_receipt(candidate)

    def test_semantic_pass_is_hashed_and_closed_in_success_receipt(self) -> None:
        supervisor._SEMANTIC_REPORT = {
            "status": "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS",
            "package_root": str(supervisor.FINAL_ROOT),
            "package_digest": "a" * 64,
            "source_bag_read": False,
            "external_write": False,
        }
        source = supervisor._BASE_MOCK_FAILURE()
        source["status"] = "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
        source["failure"] = None
        source["next"] = "S5B0_REQUIRES_EXACT_TRANSFER_HASH_AND_NEW_GPU_AUTHORIZATION"
        source["execution"]["return_code"] = 0
        source["safety"]["real_bag_read_attempted"] = True
        source["safety"]["real_bag_read_confirmed"] = True
        source["safety"]["failure_preserved"] = False
        source["package"] = {
            "manifest_sha256": "b" * 64,
            "checksums_sha256": "c" * 64,
            "inventory_sha256": "d" * 64,
            "entry_count": 20,
            "all_file_modes_0600": True,
            "validated": True,
        }
        source["roots"]["atomic_rename"] = True
        source["roots"]["partial_after"] = {
            "path": str(supervisor.PARTIAL_ROOT), "exists": False,
            "mode": "ABSENT", "entry_count": 0, "inventory_sha256": None,
        }
        source["roots"]["final_after"] = {
            "path": str(supervisor.FINAL_ROOT), "exists": True,
            "mode": "0700", "entry_count": 20, "inventory_sha256": "d" * 64,
        }
        receipt = supervisor.transform_receipt(source)
        Draft202012Validator(json.loads(supervisor.RECEIPT_SCHEMA.read_bytes())).validate(receipt)
        self.assertTrue(receipt["semantic_validation"]["validated"])
        self.assertEqual(receipt["semantic_validation"]["status"], "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS")
        self.assertEqual(len(receipt["semantic_validation"]["report_sha256"]), 64)
        self.assertEqual(receipt["execution"]["supplementary_groups"], [])

    def test_worker_checks_zero_supplementary_groups_before_base_worker(self) -> None:
        with mock.patch.object(supervisor.os, "getuid", return_value=1000), mock.patch.object(supervisor.os, "getgid", return_value=1000), mock.patch.object(supervisor.os, "getgroups", return_value=[27]):
            with self.assertRaisesRegex(supervisor.ExecutionV4Error, "zero groups"):
                supervisor.worker(Path("/mock/receipt"), Path("/mock/source"), Path("/mock/partial"))
        with mock.patch.object(supervisor.os, "getuid", return_value=1000), mock.patch.object(supervisor.os, "getgid", return_value=1000), mock.patch.object(supervisor.os, "getgroups", return_value=[]), mock.patch.object(supervisor, "_BASE_WORKER", return_value={"status": "WORKER_PASS"}) as base_worker:
            result = supervisor.worker(Path("/mock/receipt"), Path("/mock/source"), Path("/mock/partial"))
        self.assertEqual(result["status"], "WORKER_PASS")
        base_worker.assert_called_once()


if __name__ == "__main__":
    unittest.main()
