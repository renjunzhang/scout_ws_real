#!/usr/bin/env python3
"""Static and module-state regression tests for the S5A1 v14 recovery."""

import copy
import json
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_execution_supervisor_v14 as supervisor


class ExecutionSupervisorV14Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.self_check = supervisor.static_self_check()
        cls.policy = json.loads(supervisor.POLICY.read_bytes())

    def setUp(self):
        supervisor.configure_paths()
        if supervisor.ACTIVE_POLICY is not None:
            supervisor.install_runtime()

    def raw_v1_failure(self):
        value = supervisor.v6._BASE_MOCK_FAILURE()
        value["execution"]["supplementary_groups"] = supervisor.HOST_GROUPS
        return value

    def complete_tf_witness(self):
        diagnostic = self.policy["tf_contract"]["real_diagnostic_witness"]
        return {
            "status": "PASS_ALIGNMENT_WITNESS",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "container_mount_source": "FROZEN_COMPATIBILITY_CONTRACT",
            "tf_used_as_motion": False,
            "comparison_frame": self.policy["tf_contract"]["comparison_frame"],
            "sample_count": 10,
            **diagnostic,
            "frozen_thresholds": {
                "maximum_time_delta_s": self.policy["tf_contract"][
                    "maximum_time_delta_s"
                ],
                "maximum_position_delta_m": self.policy["tf_contract"][
                    "maximum_position_delta_m"
                ],
                "maximum_angular_delta_rad": self.policy["tf_contract"][
                    "maximum_angular_delta_rad"
                ],
            },
        }

    def raw_v1_success(self):
        value = self.raw_v1_failure()
        value["status"] = "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
        value["failure"] = None
        value["next"] = "S5B0_REQUIRES_EXACT_TRANSFER_HASH_AND_NEW_GPU_AUTHORIZATION"
        value["execution"]["return_code"] = 0
        value["safety"].update(
            real_bag_read_attempted=True,
            real_bag_read_confirmed=True,
            failure_preserved=False,
        )
        value["package"] = {
            "manifest_sha256": "a" * 64,
            "checksums_sha256": "b" * 64,
            "inventory_sha256": "c" * 64,
            "entry_count": 20,
            "all_file_modes_0600": True,
            "validated": True,
        }
        value["roots"]["atomic_rename"] = True
        value["roots"]["partial_after"] = {
            "path": str(supervisor.PARTIAL_ROOT),
            "exists": False,
            "mode": "ABSENT",
            "entry_count": 0,
            "inventory_sha256": None,
        }
        value["roots"]["final_after"] = {
            "path": str(supervisor.FINAL_ROOT),
            "exists": True,
            "mode": "0700",
            "entry_count": 20,
            "inventory_sha256": "c" * 64,
        }
        return value

    def test_v6_module_state_fields_are_exact(self):
        self.assertEqual(
            ("PARTIAL_ROOT", "FINAL_ROOT", "EXECUTION_ID"),
            supervisor.V6_MODULE_STATE_FIELDS,
        )
        self.assertEqual(
            list(supervisor.V6_MODULE_STATE_FIELDS),
            self.policy["sandbox"]["v6_module_state_fields"],
        )

    def test_preserved_v13_is_exact_and_semantically_valid(self):
        value = supervisor.preserved_v13()
        self.assertTrue(value["partial_root_absent"])
        self.assertTrue(value["final_receipt_absent"])
        self.assertEqual(20, value["package_entries"])
        self.assertEqual(
            "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS",
            value["package_gate_status"],
        )
        self.assertEqual(supervisor.V13_PUBLICATION_ERROR, value["publication_error"])

    def test_policy_receipt_schema_and_self_check_are_closed(self):
        schema = json.loads(supervisor.POLICY_SCHEMA.read_bytes())
        Draft202012Validator.check_schema(schema)
        supervisor.gate.v1.assert_schema_deep_closed(schema)
        Draft202012Validator(schema).validate(self.policy)
        receipt = supervisor.receipt_schema()
        Draft202012Validator.check_schema(receipt)
        supervisor.gate.v1.assert_schema_deep_closed(receipt)
        self.assertEqual(
            "PASS_S5A1_EXECUTION_SUPERVISOR_V14_STATIC_CONTRACT",
            self.self_check["status"],
        )
        self.assertFalse(self.self_check["real_bag_read_attempted"])
        self.assertFalse(self.self_check["files_written"])

    def test_parent_v13_raw_build_clobbers_v6_state_without_guard(self):
        supervisor.configure_paths()
        runtime, v6_state = supervisor._install_runtime_bindings()
        try:
            supervisor.parent.raw_build(
                supervisor.parent.S5A0_INNER_RECEIPT,
                supervisor.PRIMARY_BAG,
                supervisor.PARTIAL_ROOT,
                receipt_fd=101,
                source_fd=102,
            )
            self.assertNotEqual(
                supervisor.v6.PARTIAL_ROOT, v6_state["PARTIAL_ROOT"]
            )
        finally:
            supervisor._restore_all(runtime, v6_state)
        supervisor.assert_v6_module_state(v6_state)

    def test_v14_raw_build_restores_base_and_v6_state(self):
        runtime = supervisor.parent.snapshot_runtime_bindings()
        v6_state = supervisor.snapshot_v6_module_state()
        argv = supervisor.raw_build(
            supervisor.parent.S5A0_INNER_RECEIPT,
            supervisor.PRIMARY_BAG,
            supervisor.PARTIAL_ROOT,
            receipt_fd=101,
            source_fd=102,
        )
        supervisor.parent.assert_runtime_bindings(runtime)
        supervisor.assert_v6_module_state(v6_state)
        self.assertEqual([], supervisor.assert_import_closure_bound(argv)["missing"])
        self.assertEqual(
            str(supervisor.SCRIPT), argv[argv.index("--") + 2]
        )

    def test_each_v6_module_field_drift_fails_closed(self):
        for field in supervisor.V6_MODULE_STATE_FIELDS:
            with self.subTest(field=field):
                snapshot = supervisor.snapshot_v6_module_state()
                try:
                    setattr(supervisor.v6, field, object())
                    with self.assertRaises(supervisor.ExecutionV14Error):
                        supervisor.assert_v6_module_state(snapshot)
                finally:
                    supervisor.restore_v6_module_state(snapshot)

    def test_v6_state_restores_on_parent_exception(self):
        caller_runtime = supervisor.parent.snapshot_runtime_bindings()
        caller_v6 = supervisor.snapshot_v6_module_state()

        def fail(*_args, **_kwargs):
            supervisor.v6.PARTIAL_ROOT = Path("/tmp/drift")
            supervisor.v6.FINAL_ROOT = Path("/tmp/drift-final")
            supervisor.v6.EXECUTION_ID = "drift"
            raise RuntimeError("synthetic v13 parent failure")

        with mock.patch.object(supervisor.parent, "raw_build", side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, "synthetic v13 parent failure"):
                supervisor.raw_build(
                    supervisor.parent.S5A0_INNER_RECEIPT,
                    supervisor.PRIMARY_BAG,
                    supervisor.PARTIAL_ROOT,
                    receipt_fd=101,
                    source_fd=102,
                )
        supervisor.parent.assert_runtime_bindings(caller_runtime)
        supervisor.assert_v6_module_state(caller_v6)

    def test_success_transform_has_two_semantic_roots_and_v14_status(self):
        raw = self.raw_v1_success()
        old_roots, old_report = supervisor.v6._SEMANTIC_ROOTS, supervisor.v6._SEMANTIC_REPORT
        old_witness = supervisor.CAPTURED_TF_WITNESS
        old_parent_witness = supervisor.parent.CAPTURED_TF_WITNESS
        parent_sentinel = {"sentinel": "restore-parent-v13-witness"}
        supervisor.v6._SEMANTIC_ROOTS = [
            str(supervisor.PARTIAL_ROOT), str(supervisor.FINAL_ROOT)
        ]
        supervisor.v6._SEMANTIC_REPORT = {
            "status": "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS"
        }
        witness = self.complete_tf_witness()
        supervisor.CAPTURED_TF_WITNESS = copy.deepcopy(witness)
        supervisor.parent.CAPTURED_TF_WITNESS = parent_sentinel
        try:
            value = supervisor.transform_receipt(raw)
            self.assertIs(supervisor.parent.CAPTURED_TF_WITNESS, parent_sentinel)
        finally:
            supervisor.v6._SEMANTIC_ROOTS, supervisor.v6._SEMANTIC_REPORT = (
                old_roots, old_report
            )
            supervisor.CAPTURED_TF_WITNESS = old_witness
            supervisor.parent.CAPTURED_TF_WITNESS = old_parent_witness
        self.assertEqual(
            "PASS_S5A1_HANDOFF_GATE_V12_STRONG_SEMANTICS",
            value["semantic_validation"]["status"],
        )
        self.assertTrue(value["execution"]["v6_module_state_guarded"])
        self.assertEqual(witness, value["tf_alignment_witness"])
        Draft202012Validator(supervisor.receipt_schema()).validate(value)

    def test_success_without_tf_witness_fails_closed(self):
        raw = self.raw_v1_success()
        old_roots, old_report = supervisor.v6._SEMANTIC_ROOTS, supervisor.v6._SEMANTIC_REPORT
        old_witness = supervisor.CAPTURED_TF_WITNESS
        supervisor.v6._SEMANTIC_ROOTS = [
            str(supervisor.PARTIAL_ROOT), str(supervisor.FINAL_ROOT)
        ]
        supervisor.v6._SEMANTIC_REPORT = {
            "status": "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS"
        }
        supervisor.CAPTURED_TF_WITNESS = None
        try:
            with self.assertRaisesRegex(
                supervisor.ExecutionV14Error, "success lacks complete TF witness"
            ):
                supervisor.transform_receipt(raw)
        finally:
            supervisor.v6._SEMANTIC_ROOTS, supervisor.v6._SEMANTIC_REPORT = (
                old_roots, old_report
            )
            supervisor.CAPTURED_TF_WITNESS = old_witness

    def test_mock_subprocess_captures_complete_tf_witness_without_execution(self):
        witness = self.complete_tf_witness()
        completed = SimpleNamespace(
            returncode=0,
            stdout=json.dumps(
                {
                    "worker_supplementary_groups": supervisor.MAPPED_GROUPS,
                    "tf_qc": witness,
                }
            ).encode(),
            stderr=b"",
        )
        old_witness = supervisor.CAPTURED_TF_WITNESS
        try:
            with mock.patch.object(
                supervisor.v6.v5, "_REAL_SUBPROCESS_RUN", return_value=completed
            ):
                result = supervisor._sanitized_subprocess_run(["fixture"])
            self.assertIs(result, completed)
            self.assertEqual(witness, supervisor.CAPTURED_TF_WITNESS)
        finally:
            supervisor.CAPTURED_TF_WITNESS = old_witness

    def test_mock_failure_and_fresh_paths_are_static(self):
        paths = (
            supervisor.PARTIAL_ROOT,
            supervisor.FINAL_ROOT,
            supervisor.EXECUTION_RECEIPT,
            supervisor.EXECUTION_RECEIPT_PARTIAL,
        )
        self.assertTrue(all(not os.path.lexists(path) for path in paths))
        supervisor.assert_fresh_v14()
        Draft202012Validator(supervisor.receipt_schema()).validate(
            supervisor.mock_failure_receipt()
        )
        self.assertTrue(all(not os.path.lexists(path) for path in paths))

    def test_raw_origin_rejects_already_transformed_shape(self):
        value = self.raw_v1_failure()
        value["execution_id"] = "already-transformed"
        with self.assertRaises(Exception):
            supervisor.transform_receipt(value)


if __name__ == "__main__":
    unittest.main()
