#!/usr/bin/env python3
"""Static, preserved-evidence and full runtime-binding tests for S5A1 v13."""

from __future__ import annotations

import copy
import json
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_execution_supervisor_v13 as supervisor


class ExecutionSupervisorV13Tests(unittest.TestCase):
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

    def test_runtime_binding_field_list_is_exact_26(self):
        expected = (
            "SCRIPT", "POLICY", "PACKAGE_SCHEMA", "RECEIPT_SCHEMA",
            "GATE", "EXTRACTOR", "BRIDGE", "READER", "COMPAT_GATE",
            "TRANSFER_ID", "PARTIAL_ROOT", "FINAL_ROOT",
            "EXECUTION_RECEIPT", "EXECUTION_RECEIPT_PARTIAL",
            "S5A0_FINAL_RECEIPT", "S5A0_EVIDENCE_ROOT", "S5A0_INNER_RECEIPT",
            "EXPECTED", "gate", "extractor", "build_bwrap_argv", "load_package",
            "reserve_receipt", "publish_reserved_receipt", "mock_failure_receipt",
            "subprocess",
        )
        self.assertEqual(expected, supervisor.RUNTIME_BINDING_FIELDS)
        self.assertEqual(26, len(expected))
        self.assertEqual(list(expected), self.policy["sandbox"]["runtime_binding_fields"])

    def test_preserved_v12_identity_failure_and_package_are_exact(self):
        value = supervisor.preserved_v12()
        self.assertEqual("S5A1_EXECUTION_FAILED_PRESERVED", value["status"])
        self.assertEqual("manifest policy parent.path differs", value["failure"])
        self.assertEqual(20, value["package_entries"])
        self.assertTrue(value["real_bag_read_attempted"])
        self.assertTrue(value["real_bag_read_confirmed"])
        self.assertTrue(value["bwrap_run"])
        self.assertEqual(
            supervisor.V12_PARTIAL_INVENTORY_SHA256,
            value["partial_root"]["inventory_sha256"],
        )
        self.assertEqual(
            supervisor.V12_RECEIPT_SHA256, value["receipt"]["sha256"]
        )

    def test_policy_receipt_schema_and_self_check_are_closed(self):
        policy_schema = json.loads(supervisor.POLICY_SCHEMA.read_bytes())
        Draft202012Validator.check_schema(policy_schema)
        supervisor.gate.v1.assert_schema_deep_closed(policy_schema)
        Draft202012Validator(policy_schema).validate(self.policy)
        receipt_schema = supervisor.receipt_schema()
        Draft202012Validator.check_schema(receipt_schema)
        supervisor.gate.v1.assert_schema_deep_closed(receipt_schema)
        self.assertEqual(
            "PASS_S5A1_EXECUTION_SUPERVISOR_V13_STATIC_CONTRACT",
            self.self_check["status"],
        )
        self.assertEqual(26, self.self_check["runtime_binding_guard"]["field_count"])
        self.assertFalse(self.self_check["real_bag_read_attempted"])
        self.assertFalse(self.self_check["worker_run"])
        self.assertFalse(self.self_check["bwrap_run"])
        self.assertFalse(self.self_check["files_written"])

    def test_parent_raw_build_reproduces_clobber_and_test_restores_it(self):
        supervisor.configure_paths()
        runtime = supervisor._install_runtime_bindings()
        try:
            supervisor.v12.raw_build(
                supervisor.S5A0_INNER_RECEIPT,
                supervisor.PRIMARY_BAG,
                supervisor.PARTIAL_ROOT,
                receipt_fd=101,
                source_fd=102,
            )
            drifted = {
                "EXPECTED": supervisor.base.EXPECTED is not runtime["bindings"]["EXPECTED"],
                "build_bwrap_argv": (
                    supervisor.base.build_bwrap_argv
                    is not runtime["bindings"]["build_bwrap_argv"]
                ),
                "load_package": (
                    supervisor.base.load_package is not runtime["bindings"]["load_package"]
                ),
            }
            self.assertEqual(
                {"EXPECTED": True, "build_bwrap_argv": True, "load_package": True},
                drifted,
            )
        finally:
            supervisor.restore_runtime_bindings(runtime)
        supervisor.assert_runtime_bindings(runtime)

    def test_v13_raw_build_restores_all_26_and_binds_full_closure(self):
        argv = supervisor.raw_build(
            supervisor.S5A0_INNER_RECEIPT,
            supervisor.PRIMARY_BAG,
            supervisor.PARTIAL_ROOT,
            receipt_fd=101,
            source_fd=102,
        )
        report = supervisor.assert_runtime_bindings(
            supervisor.snapshot_runtime_bindings()
        )
        self.assertEqual(26, report["field_count"])
        self.assertEqual([], supervisor.assert_import_closure_bound(argv)["missing"])
        marker = argv.index("--")
        self.assertEqual(str(supervisor.SCRIPT), argv[marker + 2])

    def test_expected_in_place_mutation_is_detected_and_restored(self):
        runtime = supervisor.snapshot_runtime_bindings()
        supervisor.base.EXPECTED[Path("/tmp/forbidden-v13-drift")] = "0" * 64
        with self.assertRaisesRegex(
            supervisor.ExecutionV13Error, "EXPECTED content drifted"
        ):
            supervisor.assert_runtime_bindings(runtime)
        supervisor.restore_runtime_bindings(runtime)
        self.assertNotIn(Path("/tmp/forbidden-v13-drift"), supervisor.base.EXPECTED)

    def test_each_runtime_binding_drift_fails_closed(self):
        for field in supervisor.RUNTIME_BINDING_FIELDS:
            with self.subTest(field=field):
                runtime = supervisor.snapshot_runtime_bindings()
                try:
                    setattr(supervisor.base, field, object())
                    with self.assertRaises(supervisor.ExecutionV13Error):
                        supervisor.assert_runtime_bindings(runtime)
                finally:
                    supervisor.restore_runtime_bindings(runtime)

    def test_parent_exception_restores_all_26(self):
        caller_runtime = supervisor.snapshot_runtime_bindings()

        def fail_after_clobber(*_args, **_kwargs):
            supervisor.base.EXPECTED = {}
            supervisor.base.build_bwrap_argv = supervisor.parent_v4.build_bwrap_argv
            supervisor.base.load_package = supervisor.parent_v4.load_package
            supervisor.base.subprocess = object()
            raise RuntimeError("synthetic parent failure")

        with mock.patch.object(supervisor.v12, "raw_build", side_effect=fail_after_clobber):
            with self.assertRaisesRegex(RuntimeError, "synthetic parent failure"):
                supervisor.raw_build(
                    supervisor.S5A0_INNER_RECEIPT,
                    supervisor.PRIMARY_BAG,
                    supervisor.PARTIAL_ROOT,
                    receipt_fd=101,
                    source_fd=102,
                )
        supervisor.assert_runtime_bindings(caller_runtime)

    def test_load_package_exception_restores_all_26(self):
        runtime = supervisor.snapshot_runtime_bindings()

        def fail_after_clobber(_root):
            supervisor.base.EXPECTED = {}
            supervisor.base.load_package = supervisor.parent_v4.load_package
            raise RuntimeError("synthetic package failure")

        with mock.patch.object(
            supervisor.v6, "_BASE_LOAD_PACKAGE", side_effect=fail_after_clobber
        ):
            with self.assertRaisesRegex(RuntimeError, "synthetic package failure"):
                supervisor.load_package(supervisor.V12_PARTIAL)
        supervisor.assert_runtime_bindings(runtime)

    def test_exact_raw_v1_origin_and_single_parent_transform(self):
        raw = self.raw_v1_failure()
        with mock.patch.object(
            supervisor.v12,
            "transform_receipt",
            wraps=supervisor.v12.transform_receipt,
        ) as parent_transform:
            transformed = supervisor.transform_receipt(raw)
        self.assertEqual(1, parent_transform.call_count)
        self.assertEqual(
            "smpcc-r8-liquid-s5a1-execution-receipt-v13",
            transformed["schema_version"],
        )
        self.assertTrue(transformed["execution"]["runtime_bindings_guarded"])
        self.assertTrue(transformed["safety"]["v12_evidence_preserved"])
        Draft202012Validator(supervisor.receipt_schema()).validate(transformed)
        for mutation in (
            lambda value: value.update(schema_version="wrong"),
            lambda value: value.update(extra=True),
            lambda value: value.update(execution_id="already-transformed"),
            lambda value: value.update(semantic_validation={}),
        ):
            changed = copy.deepcopy(raw)
            mutation(changed)
            with self.assertRaises(supervisor.ExecutionV13Error):
                supervisor.transform_receipt(changed)

    def test_mock_failure_receipt_validates_without_files(self):
        before = {
            path: os.path.lexists(path)
            for path in (
                supervisor.PARTIAL_ROOT,
                supervisor.FINAL_ROOT,
                supervisor.EXECUTION_RECEIPT,
                supervisor.EXECUTION_RECEIPT_PARTIAL,
            )
        }
        value = supervisor.mock_failure_receipt()
        Draft202012Validator(supervisor.receipt_schema()).validate(value)
        after = {path: os.path.lexists(path) for path in before}
        self.assertEqual(before, after)
        self.assertEqual({path: False for path in before}, after)

    def test_closure_deletion_and_runtime_argv_drift_fail_closed(self):
        argv = supervisor.canonical_argv()
        closure, witness = supervisor.local_import_closure()
        self.assertEqual(
            self.policy["sandbox"]["local_python_import_closure_count"],
            witness["count"],
        )
        self.assertEqual(
            self.policy["sandbox"]["local_python_import_closure_sha256"],
            witness["sha256"],
        )
        bound = supervisor._ro_bind_paths(argv)
        for path in sorted(closure, key=str):
            with self.subTest(path=path):
                changed = list(argv)
                index = next(
                    i
                    for i in range(len(changed) - 2)
                    if changed[i] == "--ro-bind"
                    and changed[i + 1] == str(path)
                    and changed[i + 2] == str(path)
                )
                del changed[index:index + 3]
                with self.assertRaises(supervisor.ExecutionV13Error):
                    supervisor.assert_import_closure_bound(changed)
        self.assertTrue(set(closure).issubset(bound))

    def test_fresh_v13_paths_and_preserved_v12_paths_are_disjoint(self):
        current = {
            supervisor.PARTIAL_ROOT,
            supervisor.FINAL_ROOT,
            supervisor.EXECUTION_RECEIPT,
            supervisor.EXECUTION_RECEIPT_PARTIAL,
        }
        preserved = {
            supervisor.V12_PARTIAL,
            supervisor.V12_FINAL,
            supervisor.V12_RECEIPT,
            supervisor.V12_RESERVATION,
        }
        self.assertTrue(current.isdisjoint(preserved))
        self.assertTrue(all(not os.path.lexists(path) for path in current))
        supervisor.assert_fresh_v13()


if __name__ == "__main__":
    unittest.main()
