#!/usr/bin/env python3
"""Static and negative tests for S5A1 execution v10."""

import copy
import json
import os
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator

ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT/"scripts"))
import r8_liquid_s5a1_execution_supervisor_v10 as supervisor  # noqa: E402


class ExecutionSupervisorV10Tests(unittest.TestCase):
    def test_policy_receipt_and_mock_failure_are_deep_closed(self):
        schema=json.loads(supervisor.POLICY_SCHEMA.read_bytes());policy=json.loads(supervisor.POLICY.read_bytes())
        Draft202012Validator(schema).validate(policy);supervisor.gate.v1.assert_schema_deep_closed(schema)
        supervisor.static_self_check();receipt_schema=supervisor.receipt_schema()
        supervisor.gate.v1.assert_schema_deep_closed(receipt_schema)
        receipt=supervisor.mock_failure_receipt();Draft202012Validator(receipt_schema).validate(receipt)
        self.assertEqual(receipt["preserved_v9_preflight"]["return_code"],2)
        self.assertFalse(receipt["preserved_v9_preflight"]["real_bag_read"])
        self.assertFalse(receipt["preserved_v9_preflight"]["files_written"])

    def test_canonical_worker_identity_and_fresh_paths(self):
        report=supervisor.admit_policy();argv=supervisor.canonical_argv()
        self.assertIn(str(supervisor.SCRIPT),argv);self.assertEqual(argv.count("--bind"),1)
        self.assertEqual(supervisor.v6._argv_hash(argv),report["canonical_argv_sha256"])
        for path in (supervisor.PARTIAL_ROOT,supervisor.FINAL_ROOT,supervisor.EXECUTION_RECEIPT,supervisor.EXECUTION_RECEIPT_PARTIAL):
            self.assertFalse(os.path.lexists(path))

    def test_exact_v8_empty_reservation_is_specialized_and_excluded_once(self):
        policy=json.loads(supervisor.POLICY.read_bytes())
        with mock.patch.object(supervisor.v9,"preserved_v8",wraps=supervisor.v9.preserved_v8) as specialized:
            expected,witness=supervisor.expected_nonempty_parent_map(policy)
        specialized.assert_called_once()
        self.assertNotIn(supervisor.V8_RESERVATION,expected)
        self.assertEqual(witness["excluded_count"],1)
        self.assertTrue(witness["all_other_zero_byte_file_refs_fail_closed"])
        self.assertIn("O_NOFOLLOW",witness["specialized_validator"])

    def test_any_other_zero_byte_file_reference_fails_closed(self):
        policy=json.loads(supervisor.POLICY.read_bytes())
        for mutate in (
            lambda p: p["parent_v9"]["tests"].update(size_bytes=0),
            lambda p: p["preserved_v8"]["empty_receipt_reservation"].update(path="/tmp/not-the-v8-reservation"),
            lambda p: p["preserved_v8"]["empty_receipt_reservation"].update(sha256="0"*64),
        ):
            changed=copy.deepcopy(policy);mutate(changed)
            with self.assertRaises(supervisor.ExecutionV10Error):
                supervisor.expected_nonempty_parent_map(changed)

    def test_parent_hashes_and_v9_preflight_are_exact(self):
        with mock.patch.object(supervisor.v9,"admit_policy",wraps=supervisor.v9.admit_policy) as recursive:
            report=supervisor.admit_policy()
        recursive.assert_called_once_with(json.loads(supervisor.POLICY.read_bytes())["parent_v9"]["policy"]["sha256"])
        self.assertTrue(report["parent_v9_recursive_admission"])
        policy=json.loads(supervisor.POLICY.read_bytes())
        self.assertEqual(supervisor.preserved_v9_preflight(),policy["preserved_v9_preflight"])
        for item in policy["parent_v9"].values():
            self.assertEqual(supervisor.file_identity(Path(item["path"])),item)

    def test_parent_v9_recursive_admission_failure_is_fatal(self):
        with mock.patch.object(supervisor.v9,"admit_policy",side_effect=RuntimeError("nested drift")):
            with self.assertRaisesRegex(RuntimeError,"nested drift"):
                supervisor.admit_policy()

    def test_unknown_cli_is_rc2(self):
        completed=subprocess.run([sys.executable,"-B",str(supervisor.SCRIPT),"unknown"],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False,text=True,env={"PATH":"/usr/bin","HOME":"/nonexistent","PYTHONDONTWRITEBYTECODE":"1"})
        self.assertEqual(completed.returncode,2)


if __name__=="__main__":unittest.main()
