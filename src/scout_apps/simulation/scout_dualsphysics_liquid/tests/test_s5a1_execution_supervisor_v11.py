#!/usr/bin/env python3
"""Static and negative tests for recursive S5A1 execution v11."""

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
import r8_liquid_s5a1_execution_supervisor_v11 as supervisor  # noqa: E402


class ExecutionSupervisorV11Tests(unittest.TestCase):
    def test_policy_receipt_and_mock_failure_are_deep_closed(self):
        schema=json.loads(supervisor.POLICY_SCHEMA.read_bytes());policy=json.loads(supervisor.POLICY.read_bytes())
        Draft202012Validator(schema).validate(policy);supervisor.gate.v1.assert_schema_deep_closed(schema)
        supervisor.static_self_check();receipt_schema=supervisor.receipt_schema();supervisor.gate.v1.assert_schema_deep_closed(receipt_schema)
        receipt=supervisor.mock_failure_receipt();Draft202012Validator(receipt_schema).validate(receipt)
        self.assertEqual(receipt["preserved_v10"]["real_bag_read_attempted"],"UNKNOWN")
        self.assertEqual(receipt["preserved_v10"]["bwrap_run"],"UNKNOWN")
        self.assertFalse(receipt["safety"]["real_bag_read_attempted"])
        self.assertFalse(receipt["safety"]["real_bag_read_confirmed"])
        self.assertFalse(receipt["safety"]["bwrap_run"])

    def test_current_v11_read_evidence_is_boolean_and_required_for_success(self):
        supervisor.static_self_check()
        value=supervisor.v6._BASE_MOCK_FAILURE()
        value["safety"]["real_bag_read_attempted"]=False
        value["safety"]["real_bag_read_confirmed"]=True
        with self.assertRaisesRegex(supervisor.ExecutionV11Error,"confirmed read without attempted"):
            supervisor.transform_receipt(value)
        transformed=supervisor._parent_v10_transform(supervisor.v6._BASE_MOCK_FAILURE())
        transformed["status"]="S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
        transformed["safety"]["real_bag_read_attempted"]=False
        transformed["safety"]["real_bag_read_confirmed"]=False
        with mock.patch.object(supervisor,"_parent_v10_transform",return_value=transformed):
            with self.assertRaisesRegex(supervisor.ExecutionV11Error,"lacks current real-bag"):
                supervisor.transform_receipt(value)

    def test_preserved_v10_is_exact_and_conservative(self):
        value=supervisor.preserved_v10();self.assertEqual(value["partial_root"]["inode"],15748816)
        self.assertEqual(value["empty_receipt_reservation"]["size_bytes"],0)
        self.assertEqual(value["empty_receipt_reservation"]["sha256"],supervisor.EMPTY_SHA256)
        self.assertEqual(value["surface_error"],"'/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_s5a1_ros1_signal_extractor_v5.py'")
        self.assertEqual(value["real_bag_read_attempted"],"UNKNOWN");self.assertEqual(value["real_bag_read_confirmed"],"UNKNOWN")
        self.assertEqual(value["bwrap_run"],"UNKNOWN");self.assertEqual(value["package_entries"],0)

    def test_recursive_parent_refs_and_exact_two_exclusions(self):
        policy=json.loads(supervisor.POLICY.read_bytes());expected,witness=supervisor.recursive_expected_map(policy)
        p10=json.loads(supervisor.v10.POLICY.read_bytes());p9=json.loads(supervisor.v9.POLICY.read_bytes())
        for parent in (p10,p9):
            for item in supervisor._iter_file_refs(parent):
                if item["size_bytes"]>0:self.assertIn(Path(item["path"]),expected)
        self.assertEqual(witness["excluded_unique_count"],2)
        self.assertEqual(set(witness["excluded_paths"]),{str(supervisor.V8_RESERVATION),str(supervisor.V10_RESERVATION)})

    def test_all_other_zero_byte_refs_fail_closed(self):
        policy=json.loads(supervisor.POLICY.read_bytes())
        for mutate in (lambda p:p["parent_v10"]["tests"].update(size_bytes=0),
                       lambda p:p["preserved_v10"]["empty_receipt_reservation"].update(path="/tmp/wrong"),
                       lambda p:p["preserved_v10"]["empty_receipt_reservation"].update(sha256="0"*64)):
            changed=copy.deepcopy(policy);mutate(changed)
            with self.assertRaises(supervisor.ExecutionV11Error):supervisor.recursive_expected_map(changed)

    def test_every_base_receipt_parent_is_mandatory(self):
        policy=json.loads(supervisor.POLICY.read_bytes());expected,_=supervisor.recursive_expected_map(policy);supervisor.configure_paths()
        required=supervisor.required_base_receipt_parent_paths();supervisor.assert_base_receipt_parent_coverage(expected)
        for path in required:
            changed=dict(expected);changed.pop(path,None)
            with self.assertRaises(supervisor.ExecutionV11Error):supervisor.assert_base_receipt_parent_coverage(changed)

    def test_admission_recursively_calls_v10_and_paths_are_fresh(self):
        with mock.patch.object(supervisor.v10,"admit_policy",wraps=supervisor.v10.admit_policy) as parent:
            report=supervisor.admit_policy()
        parent.assert_called_once();self.assertEqual(report["base_coverage"]["status"],"PASS_BASE_RECEIPT_PARENT_COVERAGE")
        for path in (supervisor.PARTIAL_ROOT,supervisor.FINAL_ROOT,supervisor.EXECUTION_RECEIPT,supervisor.EXECUTION_RECEIPT_PARTIAL):
            self.assertFalse(os.path.lexists(path))

    def test_unknown_cli_is_rc2(self):
        completed=subprocess.run([sys.executable,"-B",str(supervisor.SCRIPT),"unknown"],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False,text=True,env={"PATH":"/usr/bin","HOME":"/nonexistent","PYTHONDONTWRITEBYTECODE":"1"})
        self.assertEqual(completed.returncode,2)


if __name__=="__main__":unittest.main()
