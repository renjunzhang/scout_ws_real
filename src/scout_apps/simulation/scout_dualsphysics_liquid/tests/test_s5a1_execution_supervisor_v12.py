#!/usr/bin/env python3
"""Static, closure, hook and publisher tests for S5A1 execution v12."""

import copy
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_execution_supervisor_v12 as supervisor


class ExecutionSupervisorV12Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.self_check = supervisor.static_self_check()

    def raw_v1_failure(self):
        value = supervisor.v6._BASE_MOCK_FAILURE()
        value["execution"]["supplementary_groups"] = list(supervisor.HOST_GROUPS)
        return value

    def test_policy_receipt_and_mock_failure_are_deep_closed(self):
        policy_schema = json.loads(supervisor.POLICY_SCHEMA.read_bytes())
        policy = json.loads(supervisor.POLICY.read_bytes())
        Draft202012Validator.check_schema(policy_schema)
        supervisor.gate.v1.assert_schema_deep_closed(policy_schema)
        Draft202012Validator(policy_schema).validate(policy)
        receipt_schema = supervisor.receipt_schema()
        supervisor.gate.v1.assert_schema_deep_closed(receipt_schema)
        receipt = supervisor.mock_failure_receipt()
        Draft202012Validator(receipt_schema).validate(receipt)
        self.assertEqual(receipt["preserved_v11"]["real_bag_read_attempted"], "UNKNOWN")
        self.assertFalse(receipt["safety"]["real_bag_read_attempted"])
        self.assertFalse(receipt["safety"]["bwrap_run"])

    def test_preserved_v11_is_exact_and_conservative(self):
        value = supervisor.preserved_v11()
        self.assertEqual(value["partial_root"]["inode"], 15748876)
        self.assertEqual(value["empty_receipt_reservation"]["sha256"], supervisor.EMPTY_SHA256)
        self.assertEqual(value["empty_receipt_reservation"]["size_bytes"], 0)
        self.assertEqual(value["real_bag_read_attempted"], "UNKNOWN")
        self.assertEqual(value["real_bag_read_confirmed"], "UNKNOWN")
        self.assertEqual(value["bwrap_run"], "UNKNOWN")

    def test_v11_missing_import_bind_set_is_exact(self):
        closure, _ = supervisor.local_import_closure(supervisor.v11.SCRIPT)
        bound = supervisor._ro_bind_paths(supervisor.v11.canonical_argv())
        missing = {path.name for path in closure if path not in bound}
        self.assertEqual(missing, {
            "r8_liquid_s5a1_execution_supervisor_v9.py",
            "r8_liquid_s5a1_handoff_gate_v7.py",
            "r8_liquid_s5a1_handoff_gate_v6.py",
        })

    def test_v12_import_closure_is_fully_bound_and_frozen(self):
        argv = supervisor.canonical_argv()
        report = supervisor.assert_import_closure_bound(argv)
        closure, witness = supervisor.local_import_closure()
        self.assertEqual(report["missing"], [])
        self.assertEqual(witness["count"], supervisor.ACTIVE_POLICY["sandbox"]["local_python_import_closure_count"])
        self.assertEqual(witness["sha256"], supervisor.ACTIVE_POLICY["sandbox"]["local_python_import_closure_sha256"])
        self.assertIn(supervisor.SCRIPT, closure)

    def test_removing_any_local_import_bind_fails_closed(self):
        argv = supervisor.canonical_argv()
        closure, _ = supervisor.local_import_closure()
        for path in sorted(closure, key=str):
            changed = list(argv)
            for index in range(len(changed) - 2):
                if changed[index:index + 3] == ["--ro-bind", str(path), str(path)]:
                    del changed[index:index + 3]
                    break
            else:
                self.fail(f"missing expected ro-bind in canonical argv: {path}")
            with self.assertRaisesRegex(supervisor.ExecutionV12Error, "closure is incomplete"):
                supervisor.assert_import_closure_bound(changed)

    def test_build_argv_restores_v12_hooks(self):
        before = supervisor.publication_hooks()
        supervisor.build_bwrap_argv(
            supervisor.v6.v5.prior.S5A0_INNER_RECEIPT,
            supervisor.PRIMARY_BAG,
            supervisor.PARTIAL_ROOT,
            receipt_fd=101,
            source_fd=102,
        )
        self.assertEqual(supervisor.publication_hooks(), before)
        supervisor.assert_publication_hooks()

    def test_parent_builder_exception_also_restores_v12_hooks(self):
        supervisor._install_publication_hooks()
        with mock.patch.object(supervisor.v11, "raw_build", side_effect=RuntimeError("injected")):
            with self.assertRaisesRegex(RuntimeError, "injected"):
                supervisor.raw_build(
                    supervisor.v6.v5.prior.S5A0_INNER_RECEIPT,
                    supervisor.PRIMARY_BAG,
                    supervisor.PARTIAL_ROOT,
                    receipt_fd=101,
                    source_fd=102,
                )
        supervisor.assert_publication_hooks()

    def test_origin_guard_accepts_raw_v1_and_rejects_transformed_shapes(self):
        raw = self.raw_v1_failure()
        supervisor.assert_raw_v1_origin(raw)
        for changed in (
            {**copy.deepcopy(raw), "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v4", "execution_id": "v4", "semantic_validation": {}},
            {**copy.deepcopy(raw), "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v6", "contract": {}},
            supervisor._parent_v11_transform(raw),
        ):
            with self.assertRaisesRegex(supervisor.ExecutionV12Error, "raw v1"):
                supervisor.assert_raw_v1_origin(changed)

    def test_module_import_failure_publishes_closed_v12_receipt_once(self):
        raw = self.raw_v1_failure()
        raw["execution"]["return_code"] = 1
        raw["safety"]["real_bag_read_attempted"] = True
        raw["safety"]["real_bag_read_confirmed"] = False
        raw["failure"] = "worker rc=1: ModuleNotFoundError: injected"
        supervisor._install_publication_hooks()
        with tempfile.TemporaryDirectory() as directory:
            partial = Path(directory) / "receipt.json.partial"
            final = Path(directory) / "receipt.json"
            descriptor = os.open(partial, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o440)
            with mock.patch.object(supervisor, "_parent_v11_transform", wraps=supervisor._parent_v11_transform) as transform:
                with mock.patch.object(supervisor.v6.v5.prior, "publish_reserved_receipt") as v4_publisher:
                    supervisor.publish_reserved_receipt(descriptor, partial, final, raw)
            transform.assert_called_once()
            v4_publisher.assert_not_called()
            self.assertFalse(partial.exists())
            value = json.loads(final.read_bytes())
            Draft202012Validator(supervisor.receipt_schema()).validate(value)
            self.assertEqual(value["status"], "S5A1_EXECUTION_FAILED_PRESERVED")
            self.assertTrue(value["safety"]["real_bag_read_attempted"])
            self.assertFalse(value["safety"]["real_bag_read_confirmed"])
            self.assertTrue(value["safety"]["bwrap_run"])

    def test_rejected_double_transform_does_not_rename_or_write(self):
        bad = supervisor._parent_v11_transform(self.raw_v1_failure())
        supervisor._install_publication_hooks()
        with tempfile.TemporaryDirectory() as directory:
            partial = Path(directory) / "receipt.json.partial"
            final = Path(directory) / "receipt.json"
            descriptor = os.open(partial, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o440)
            with self.assertRaisesRegex(supervisor.ExecutionV12Error, "raw v1"):
                supervisor.publish_reserved_receipt(descriptor, partial, final, bad)
            self.assertTrue(partial.exists())
            self.assertEqual(partial.stat().st_size, 0)
            self.assertFalse(final.exists())

    def test_recursive_parent_refs_and_three_exclusions(self):
        expected, witness = supervisor.recursive_expected_map(supervisor.ACTIVE_POLICY)
        self.assertEqual(witness["excluded_unique_count"], 3)
        self.assertEqual(set(witness["excluded_paths"]), {
            str(supervisor.v11.V8_RESERVATION),
            str(supervisor.v11.V10_RESERVATION),
            str(supervisor.V11_RESERVATION),
        })
        for item in supervisor._iter_file_refs(supervisor.ACTIVE_POLICY):
            if item["size_bytes"] > 0:
                self.assertIn(Path(item["path"]), expected)

    def test_current_read_evidence_is_boolean_and_required_for_success(self):
        raw = self.raw_v1_failure()
        raw["safety"]["real_bag_read_confirmed"] = True
        with self.assertRaisesRegex(supervisor.ExecutionV12Error, "confirmed read without attempted"):
            supervisor.transform_receipt(raw)
        transformed = supervisor._parent_v11_transform(self.raw_v1_failure())
        transformed["status"] = "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
        transformed["safety"]["real_bag_read_attempted"] = False
        transformed["safety"]["real_bag_read_confirmed"] = False
        with mock.patch.object(supervisor, "_parent_v11_transform", return_value=transformed):
            with self.assertRaisesRegex(supervisor.ExecutionV12Error, "lacks current real-bag"):
                supervisor.transform_receipt(self.raw_v1_failure())

    def test_self_check_and_unknown_cli_are_static(self):
        self.assertEqual(self.self_check["status"], "PASS_S5A1_EXECUTION_SUPERVISOR_V12_STATIC_CONTRACT")
        for key in ("real_bag_read_attempted", "real_bag_read_confirmed", "worker_run", "bwrap_run", "files_written", "gpu_exposed", "solver_executed", "sudo_used", "network_used"):
            self.assertFalse(self.self_check[key])
        completed = subprocess.run(
            [sys.executable, "-B", str(supervisor.SCRIPT), "unknown"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            text=True,
            env={"PATH": "/usr/bin", "HOME": "/nonexistent", "PYTHONDONTWRITEBYTECODE": "1"},
        )
        self.assertEqual(completed.returncode, 2)


if __name__ == "__main__":
    unittest.main()
