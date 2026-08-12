#!/usr/bin/env python3
"""Static, publisher and negative tests for S5A1 execution v9."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from jsonschema import Draft202012Validator

ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT/"scripts"))
import r8_liquid_s5a1_execution_supervisor_v9 as supervisor  # noqa: E402


class ExecutionSupervisorV9Tests(unittest.TestCase):
    def setUp(self):
        supervisor.CAPTURED_TF_WITNESS=None

    def test_policy_receipt_and_mock_failure_are_deep_closed(self):
        schema=json.loads(supervisor.POLICY_SCHEMA.read_bytes());policy=json.loads(supervisor.POLICY.read_bytes())
        Draft202012Validator(schema).validate(policy);supervisor.gate.v1.assert_schema_deep_closed(schema)
        supervisor.static_self_check();receipt_schema=supervisor.receipt_schema()
        supervisor.gate.v1.assert_schema_deep_closed(receipt_schema)
        Draft202012Validator(receipt_schema).validate(supervisor.mock_failure_receipt())

    def test_canonical_worker_identity_and_fresh_paths(self):
        report=supervisor.admit_policy();argv=supervisor.canonical_argv()
        self.assertIn(str(supervisor.SCRIPT),argv);self.assertEqual(argv.count("--bind"),1)
        self.assertEqual(supervisor.v6._argv_hash(argv),report["canonical_argv_sha256"])
        for path in (supervisor.PARTIAL_ROOT,supervisor.FINAL_ROOT,supervisor.EXECUTION_RECEIPT,supervisor.EXECUTION_RECEIPT_PARTIAL):self.assertFalse(os.path.lexists(path))

    def test_v8_empty_failure_evidence_is_exact(self):
        value=supervisor.preserved_v8();self.assertEqual(value["partial_root"]["inode"],15748711)
        self.assertEqual(value["empty_receipt_reservation"]["sha256"],supervisor.EMPTY_SHA256)
        self.assertEqual(value["empty_receipt_reservation"]["size_bytes"],0)

    def test_worker_report_captures_complete_tf_witness(self):
        witness={"status":"PASS_ALIGNMENT_WITNESS","tf_used_as_motion":False}
        completed=SimpleNamespace(returncode=0,stdout=json.dumps({"worker_supplementary_groups":supervisor.MAPPED_GROUPS,"tf_qc":witness}).encode(),stderr=b"")
        with mock.patch.object(supervisor.v6.v5,"_REAL_SUBPROCESS_RUN",return_value=completed):
            result=supervisor._sanitized_subprocess_run(["fixture"])
        self.assertIs(result,completed);self.assertEqual(supervisor.CAPTURED_TF_WITNESS,witness)

    def test_direct_failure_publisher_never_calls_parent_and_publishes_fixture(self):
        supervisor.static_self_check()
        with tempfile.TemporaryDirectory() as directory:
            partial=Path(directory)/"receipt.partial";final=Path(directory)/"receipt.json"
            descriptor=supervisor.reserve_receipt(partial)
            parent=mock.Mock(side_effect=AssertionError("parent publisher called"))
            with mock.patch.object(supervisor.v6,"_BASE_PUBLISH",parent):
                supervisor.publish_reserved_receipt(descriptor,partial,final,supervisor.v6._BASE_MOCK_FAILURE())
            parent.assert_not_called();self.assertFalse(partial.exists());self.assertTrue(final.exists())
            value=json.loads(final.read_bytes());Draft202012Validator(supervisor.receipt_schema()).validate(value)
            self.assertEqual(value["status"],"S5A1_EXECUTION_FAILED_PRESERVED")
            self.assertIsNone(value["tf_alignment_witness"]);self.assertFalse(value["safety"]["parent_publisher_used"])

    def test_complete_relative_tf_witness_validates_and_over_limit_fails(self):
        supervisor.static_self_check();receipt=supervisor.mock_failure_receipt()
        witness={"status":"PASS_ALIGNMENT_WITNESS","odom_frame":"odom","base_frame":"base_footprint",
                 "container_mount_source":"FROZEN_COMPATIBILITY_CONTRACT","tf_used_as_motion":False,
                 "comparison_frame":"T0_INVERSE_TIMES_T_RELATIVE_SE3","sample_count":10,
                 "maximum_time_delta_s":0.005,"absolute_maximum_position_delta_m":3.9934499955863565,
                 "absolute_maximum_angular_delta_rad":0.023228249162941604,
                 "relative_maximum_translation_delta_m":0.03995630546880967,
                 "relative_rms_translation_delta_m":0.022725548786785066,
                 "relative_maximum_angular_delta_rad":0.003657518055773045,
                 "frozen_thresholds":{"maximum_time_delta_s":0.5,"maximum_position_delta_m":0.05,"maximum_angular_delta_rad":0.1}}
        receipt["tf_alignment_witness"]=witness;validator=Draft202012Validator(supervisor.receipt_schema());validator.validate(receipt)
        receipt["tf_alignment_witness"]["relative_maximum_translation_delta_m"]=0.0500001
        with self.assertRaises(Exception):validator.validate(receipt)

    def test_unknown_cli_is_rc2(self):
        completed=subprocess.run([sys.executable,"-B",str(supervisor.SCRIPT),"unknown"],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False,text=True,env={"PATH":"/usr/bin","HOME":"/nonexistent","PYTHONDONTWRITEBYTECODE":"1"})
        self.assertEqual(completed.returncode,2)


if __name__=="__main__":unittest.main()
