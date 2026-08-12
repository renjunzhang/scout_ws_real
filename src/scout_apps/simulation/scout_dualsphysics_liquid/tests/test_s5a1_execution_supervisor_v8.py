#!/usr/bin/env python3
"""Static, mock and CLI-negative tests for S5A1 execution v8."""

from __future__ import annotations

import copy
import json
import os
import subprocess
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_execution_supervisor_v8 as supervisor  # noqa: E402


class ExecutionSupervisorV8Tests(unittest.TestCase):
    def test_policy_and_effective_receipt_are_deep_closed(self) -> None:
        policy_schema = json.loads(supervisor.POLICY_SCHEMA.read_bytes())
        policy = json.loads(supervisor.POLICY.read_bytes())
        Draft202012Validator.check_schema(policy_schema)
        supervisor.gate.v1.assert_schema_deep_closed(policy_schema)
        Draft202012Validator(policy_schema).validate(policy)
        receipt_schema = supervisor.receipt_schema()
        Draft202012Validator.check_schema(receipt_schema)
        supervisor.gate.v1.assert_schema_deep_closed(receipt_schema)

    def test_self_check_and_mock_failure_receipt(self) -> None:
        report = supervisor.static_self_check()
        self.assertEqual(report["status"],
                         "PASS_S5A1_EXECUTION_SUPERVISOR_V8_STATIC_CONTRACT")
        self.assertTrue(report["receipt_schema_deep_closed"])
        receipt = supervisor.mock_failure_receipt()
        Draft202012Validator(supervisor.receipt_schema()).validate(receipt)
        self.assertEqual(receipt["preserved_v6_failure"]["failure_receipt"]["sha256"],
                         supervisor.V6_RECEIPT_SHA256)

    def test_all_self_and_parent_identities_are_frozen(self) -> None:
        supervisor.admit_policy()
        policy = supervisor.ACTIVE_POLICY
        self.assertEqual(set(policy["v8_contract"]),
                         {"policy_schema", "receipt_schema", "supervisor", "tests"})
        self.assertEqual(set(policy["parent_v7"]),
                         {"policy", "policy_schema", "receipt_schema", "supervisor", "tests"})

    def test_canonical_argv_uses_v8_worker_and_one_writable_bind(self) -> None:
        report = supervisor.admit_policy()
        argv = supervisor.canonical_argv()
        self.assertIn(str(supervisor.SCRIPT), argv)
        self.assertEqual(argv.count("--bind"), 1)
        self.assertEqual(supervisor.v6._argv_hash(argv), report["canonical_argv_sha256"])
        self.assertEqual(len(argv), report["canonical_token_count"])

    def test_fresh_paths_and_v6_evidence_are_distinct(self) -> None:
        self.assertNotEqual(supervisor.PARTIAL_ROOT, supervisor.V6_PARTIAL)
        for path in (supervisor.PARTIAL_ROOT, supervisor.FINAL_ROOT,
                     supervisor.EXECUTION_RECEIPT, supervisor.EXECUTION_RECEIPT_PARTIAL):
            self.assertFalse(os.path.lexists(path), path)
        self.assertEqual(supervisor.preserved_v6()["failure_receipt"]["sha256"],
                         supervisor.V6_RECEIPT_SHA256)

    def test_unknown_cli_command_returns_two_without_execution(self) -> None:
        completed = subprocess.run(
            [sys.executable, "-B", str(supervisor.SCRIPT), "not-a-command"],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, check=False,
            env={"PATH": "/usr/bin", "HOME": "/nonexistent", "PYTHONDONTWRITEBYTECODE": "1"},
        )
        self.assertEqual(completed.returncode, 2)
        self.assertNotIn("PASS_S5A1_EXECUTION_SUPERVISOR_V8", completed.stdout)

    def test_receipt_delta_and_policy_reject_extra_fields(self) -> None:
        schema = json.loads(supervisor.POLICY_SCHEMA.read_bytes())
        policy = json.loads(supervisor.POLICY.read_bytes())
        policy["unexpected"] = True
        with self.assertRaises(Exception):
            Draft202012Validator(schema).validate(policy)
        delta = supervisor._receipt_delta()
        changed = copy.deepcopy(delta); changed["unexpected"] = True
        self.assertNotEqual(set(changed), set(delta))


if __name__ == "__main__":
    unittest.main()
