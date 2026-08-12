#!/usr/bin/env python3
"""Static/mock regression tests for S5A1 execution supervisor v6."""

from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_execution_supervisor_v6 as supervisor  # noqa: E402


class ExecutionSupervisorV6Tests(unittest.TestCase):
    def setUp(self) -> None:
        supervisor._ACTIVE_POLICY = None
        supervisor._ACTIVE_POLICY_IDENTITY = None
        supervisor._SEMANTIC_ROOTS = []
        supervisor._SEMANTIC_REPORT = None
        supervisor._WORKER_GROUP_WITNESS = False

    def test_schemas_are_closed_and_static_self_check_passes(self) -> None:
        for path in (supervisor.POLICY_SCHEMA, supervisor.RECEIPT_SCHEMA):
            schema = json.loads(path.read_bytes())
            Draft202012Validator.check_schema(schema)
            supervisor.gate_v4.v1.assert_schema_deep_closed(schema)
        result = supervisor.static_self_check()
        self.assertEqual(result["status"], "PASS_S5A1_EXECUTION_SUPERVISOR_V6_STATIC_CONTRACT")
        self.assertFalse(result["real_bag_read"])
        self.assertFalse(result["bwrap_run"])

    def test_host_and_worker_groups_are_distinct_exact_closed_lists(self) -> None:
        self.assertEqual(supervisor.HOST_GROUPS, [4,24,27,30,46,122,135,136,1000])
        self.assertEqual(supervisor.MAPPED_GROUPS, [65534] * 8 + [1000])
        self.assertEqual(set(supervisor.MAPPED_GROUPS), {1000, 65534})
        self.assertEqual(supervisor.MAPPED_GROUPS.count(65534), 8)
        self.assertEqual(supervisor.MAPPED_GROUPS.count(1000), 1)

    def test_host_group_drift_blocks_policy_admission_before_execution(self) -> None:
        with mock.patch.object(supervisor.os, "getgroups", return_value=[1000]):
            with self.assertRaisesRegex(supervisor.ExecutionV6Error, "host supplementary"):
                supervisor._admit_policy()

    def test_worker_report_proves_exact_mapped_groups_and_rejects_drift(self) -> None:
        with mock.patch.object(supervisor.os, "getuid", return_value=1000), \
             mock.patch.object(supervisor.os, "getgid", return_value=1000), \
             mock.patch.object(supervisor.os, "getgroups", return_value=supervisor.MAPPED_GROUPS), \
             mock.patch.object(supervisor, "_BASE_WORKER", return_value={"status":"WORKER_PASS"}):
            report = supervisor.worker(Path("/r"), Path("/s"), Path("/p"))
        self.assertEqual(report["worker_supplementary_groups"], supervisor.MAPPED_GROUPS)
        with mock.patch.object(supervisor.os, "getuid", return_value=1000), \
             mock.patch.object(supervisor.os, "getgid", return_value=1000), \
             mock.patch.object(supervisor.os, "getgroups", return_value=[1000] + [65534] * 8):
            with self.assertRaisesRegex(supervisor.ExecutionV6Error, "mapped supplementary"):
                supervisor.worker(Path("/r"), Path("/s"), Path("/p"))

    def test_subprocess_worker_report_witness_is_captured(self) -> None:
        payload = json.dumps({"status":"WORKER_PASS","worker_supplementary_groups":supervisor.MAPPED_GROUPS}).encode()
        completed = subprocess.CompletedProcess(["mock"], 0, stdout=payload, stderr=b"")
        with mock.patch.object(supervisor.v5, "_REAL_SUBPROCESS_RUN", return_value=completed) as run:
            supervisor._sanitized_subprocess_run(["mock"], stdout=subprocess.PIPE)
        self.assertTrue(supervisor._WORKER_GROUP_WITNESS)
        run.assert_called_once_with(["mock"], env=supervisor.HOST_ENV, stdout=subprocess.PIPE)

    def test_build_argv_does_not_clobber_v6_publication_hooks(self) -> None:
        supervisor._admit_policy()
        supervisor._install_runtime()
        before = (supervisor.base.reserve_receipt, supervisor.base.publish_reserved_receipt,
                  supervisor.base.mock_failure_receipt)
        supervisor.build_bwrap_argv(supervisor.v5.prior.S5A0_INNER_RECEIPT,
            Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
            supervisor.PARTIAL_ROOT, receipt_fd=51, source_fd=52)
        self.assertEqual(before, (supervisor.base.reserve_receipt,
                                  supervisor.base.publish_reserved_receipt,
                                  supervisor.base.mock_failure_receipt))
        self.assertIs(supervisor.base.publish_reserved_receipt, supervisor.publish_reserved_receipt)

    def test_failure_receipt_uses_v6_schema_and_v6_publish_hook(self) -> None:
        supervisor._admit_policy()
        supervisor._install_runtime()
        source = supervisor._BASE_MOCK_FAILURE()
        source["execution"]["supplementary_groups"] = supervisor.HOST_GROUPS
        with mock.patch.object(supervisor, "_BASE_PUBLISH") as publish:
            supervisor.publish_reserved_receipt(7, Path("/mock/partial"), Path("/mock/final"), source)
        value = publish.call_args.args[3]
        self.assertEqual(value["schema_version"], "smpcc-r8-liquid-s5a1-execution-receipt-v6")
        self.assertEqual(value["execution"]["host_supplementary_groups"], supervisor.HOST_GROUPS)
        self.assertEqual(value["execution"]["worker_supplementary_groups"], supervisor.MAPPED_GROUPS)
        self.assertFalse(value["execution"]["worker_group_witness"])
        self.assertEqual(value["inputs"], source["inputs"])
        self.assertEqual(value["compatibility"], source["compatibility"])
        Draft202012Validator(json.loads(supervisor.RECEIPT_SCHEMA.read_bytes())).validate(value)

    def test_input_and_compatibility_roundtrip_and_host_receipt_drift_rejected(self) -> None:
        supervisor._admit_policy()
        source = supervisor._BASE_MOCK_FAILURE()
        source["execution"]["supplementary_groups"] = supervisor.HOST_GROUPS
        receipt = supervisor.transform_receipt(source)
        self.assertIs(receipt["inputs"], source["inputs"])
        self.assertIs(receipt["compatibility"], source["compatibility"])
        source["execution"]["supplementary_groups"] = [1000]
        with self.assertRaisesRegex(supervisor.ExecutionV6Error, "base receipt host"):
            supervisor.transform_receipt(source)

    def test_v5_failure_evidence_is_exact_and_never_reused_as_v6_root(self) -> None:
        evidence = supervisor._preserved_v5()
        self.assertEqual(evidence["partial_root"]["entry_count"], 0)
        self.assertEqual(evidence["empty_reservation"]["size_bytes"], 0)
        self.assertTrue(evidence["final_receipt_absent"])
        self.assertNotEqual(supervisor.V5_PARTIAL, supervisor.PARTIAL_ROOT)
        self.assertIn("v3_v2", str(supervisor.PARTIAL_ROOT))


if __name__ == "__main__":
    unittest.main()
