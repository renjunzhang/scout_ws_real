#!/usr/bin/env python3
"""Static/mock-only tests for the pre-frozen S5A1 execution v5 contract."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_execution_supervisor_v5 as supervisor  # noqa: E402


class S5A1ExecutionSupervisorV5Tests(unittest.TestCase):
    def setUp(self) -> None:
        supervisor._ACTIVE_POLICY = None
        supervisor._ACTIVE_POLICY_IDENTITY = None
        supervisor._SEMANTIC_ROOTS = []
        supervisor.prior._SEMANTIC_REPORT = None

    def policy(self) -> dict:
        return json.loads(supervisor.POLICY.read_bytes())

    def test_closed_policy_and_receipt_schemas(self) -> None:
        for path in (supervisor.POLICY_SCHEMA, supervisor.RECEIPT_SCHEMA):
            schema = json.loads(path.read_bytes())
            Draft202012Validator.check_schema(schema)
            supervisor.prior.gate_v3.assert_schema_deep_closed(schema)
        schema = json.loads(supervisor.POLICY_SCHEMA.read_bytes())
        policy = self.policy()
        Draft202012Validator(schema).validate(policy)
        changed = copy.deepcopy(policy)
        changed["unexpected"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)

    def test_static_self_check_freezes_complete_contract_without_execution(self) -> None:
        result = supervisor.static_self_check()
        self.assertEqual(result["status"], "PASS_S5A1_EXECUTION_SUPERVISOR_V5_STATIC_CONTRACT")
        self.assertEqual(result["execution_id"], supervisor.EXECUTION_ID)
        self.assertEqual(result["contract_file_count"], 31)
        self.assertEqual(result["tool_count"], 4)
        for key in ("real_bag_read", "worker_run", "bwrap_run", "files_written",
                    "network_used", "gpu_exposed", "solver_executed", "sudo_used"):
            self.assertFalse(result[key], key)
        self.assertEqual(result["apparmor"], "NOT_USED")

    def test_policy_tool_and_tests_hash_tampering_fail_closed(self) -> None:
        policy = self.policy()
        for mutate in (
            lambda value: value["tools"]["bwrap"].__setitem__("sha256", "0" * 64),
            lambda value: value["contracts"]["execution_supervisor_v5_tests"].__setitem__("sha256", "0" * 64),
            lambda value: value["contracts"]["handoff_gate_v3_tests"].__setitem__("sha256", "0" * 64),
        ):
            changed = copy.deepcopy(policy)
            mutate(changed)
            with self.assertRaises(supervisor.ExecutionV5Error):
                supervisor._verify_policy_contract(changed)

    def test_argv_and_environment_tampering_fail_closed(self) -> None:
        policy = self.policy()
        changed = copy.deepcopy(policy)
        changed["sandbox"]["canonical_argv"].append("--share-net")
        with self.assertRaisesRegex(supervisor.ExecutionV5Error, "canonical argv"):
            supervisor._verify_policy_contract(changed)
        changed = copy.deepcopy(policy)
        changed["sandbox"]["child_environment"]["LC_ALL"] = "C"
        with self.assertRaises((ValidationError, supervisor.ExecutionV5Error)):
            Draft202012Validator(json.loads(supervisor.POLICY_SCHEMA.read_bytes())).validate(changed)
            supervisor._verify_policy_contract(changed)

    def test_runtime_fd_expansion_matches_canonical_policy(self) -> None:
        supervisor._admit_policy()
        argv = supervisor.build_bwrap_argv(
            supervisor.prior.S5A0_INNER_RECEIPT,
            Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
            supervisor.PARTIAL_ROOT, receipt_fd=57, source_fd=91,
        )
        canonical = supervisor._canonicalize_fds(argv, 57, 91)
        self.assertEqual(canonical, supervisor._ACTIVE_POLICY["sandbox"]["canonical_argv"])
        self.assertEqual(supervisor._nul_hash(canonical), supervisor._ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"])
        self.assertEqual(argv.count("--bind"), 1)
        index = argv.index("--bind")
        self.assertEqual(argv[index + 1:index + 3], [str(supervisor.PARTIAL_ROOT)] * 2)

    def test_host_subprocess_always_receives_exact_sanitized_environment(self) -> None:
        sentinel = object()
        with mock.patch.object(supervisor, "_REAL_SUBPROCESS_RUN", return_value=sentinel) as run:
            self.assertIs(supervisor._subprocess_run(["/mock/tool"], check=False), sentinel)
        run.assert_called_once_with(["/mock/tool"], env=supervisor.HOST_ENV, check=False)
        with self.assertRaises(supervisor.ExecutionV5Error):
            supervisor._subprocess_run(["/mock/tool"], env={"PATH": "/tmp"})

    def test_success_requires_semantic_validation_on_partial_and_final(self) -> None:
        supervisor._admit_policy()
        source = supervisor.prior._BASE_MOCK_FAILURE()
        source["status"] = "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
        source["failure"] = None
        source["next"] = "S5B0_REQUIRES_EXACT_TRANSFER_HASH_AND_NEW_GPU_AUTHORIZATION"
        source["execution"]["return_code"] = 0
        source["safety"].update(real_bag_read_attempted=True, real_bag_read_confirmed=True,
                                failure_preserved=False)
        source["package"] = {"manifest_sha256": "a" * 64, "checksums_sha256": "b" * 64,
                             "inventory_sha256": "c" * 64, "entry_count": 20,
                             "all_file_modes_0600": True, "validated": True}
        source["roots"]["atomic_rename"] = True
        source["roots"]["partial_after"] = {"path": str(supervisor.PARTIAL_ROOT), "exists": False,
                                                    "mode": "ABSENT", "entry_count": 0, "inventory_sha256": None}
        source["roots"]["final_after"] = {"path": str(supervisor.FINAL_ROOT), "exists": True,
                                                  "mode": "0700", "entry_count": 20,
                                                  "inventory_sha256": "c" * 64}
        supervisor.prior._SEMANTIC_REPORT = {
            "status": "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS",
            "package_root": str(supervisor.FINAL_ROOT), "source_bag_read": False,
            "external_write": False,
        }
        supervisor._SEMANTIC_ROOTS = [str(supervisor.PARTIAL_ROOT)]
        with self.assertRaisesRegex(supervisor.ExecutionV5Error, "before rename and after"):
            supervisor.transform_receipt(source)
        supervisor._SEMANTIC_ROOTS.append(str(supervisor.FINAL_ROOT))
        receipt = supervisor.transform_receipt(source)
        Draft202012Validator(json.loads(supervisor.RECEIPT_SCHEMA.read_bytes())).validate(receipt)
        self.assertTrue(receipt["semantic_validation"]["before_atomic_rename"])
        self.assertTrue(receipt["semantic_validation"]["after_atomic_rename"])

    def test_worker_rejects_supplementary_groups_before_bag_access(self) -> None:
        with mock.patch.object(supervisor.os, "getuid", return_value=1000), \
             mock.patch.object(supervisor.os, "getgid", return_value=1000), \
             mock.patch.object(supervisor.os, "getgroups", return_value=[27]), \
             mock.patch.object(supervisor.prior, "_BASE_WORKER") as worker:
            with self.assertRaisesRegex(supervisor.ExecutionV5Error, "zero groups"):
                supervisor.worker(Path("/mock/receipt"), Path("/mock/bag"), Path("/mock/root"))
        worker.assert_not_called()


if __name__ == "__main__":
    unittest.main()
