#!/usr/bin/env python3
"""Static/negative tests for S5A0 supervisor v4 and its outer schema."""

from __future__ import annotations

import copy
import hashlib
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a0_primary_one_shot_supervisor_v4 as supervisor  # noqa: E402


class SupervisorV4Tests(unittest.TestCase):
    def test_self_check_binds_reader_worker_and_schema(self) -> None:
        result = supervisor.self_check()
        self.assertEqual(
            result["status"],
            "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_V4_STATIC_CONTRACT",
        )
        self.assertEqual(result["reader_revision"], "v3")
        self.assertEqual(result["worker_revision"], "v6")
        self.assertTrue(result["prior_v2_v3_evidence_preserved"])
        self.assertFalse(result["real_bag_read"])
        self.assertFalse(result["bwrap_executed"])

    def test_exact_sandbox_binds_have_no_optional_or_host_source(self) -> None:
        policy, _ = supervisor.load_policy()
        argv, digest = supervisor.build_bwrap_argv(policy)
        self.assertEqual(argv.count(policy["inputs"]["reader_v3"]["path"]), 1)
        self.assertEqual(argv.count(policy["inputs"]["worker_v6"]["path"]), 1)
        self.assertEqual(argv.count("--file"), 1)
        self.assertEqual(argv.count("--remount-ro"), 1)
        self.assertNotIn("--ro-bind-data", argv)
        self.assertNotIn(policy["selection"]["absolute_path"], argv)
        self.assertNotIn(policy["selection"]["forbidden_optional_path"], argv)
        self.assertEqual(len(digest), 64)

    def test_failure_receipt_validates_against_exact_v4_identity(self) -> None:
        policy, policy_sha = supervisor.load_policy()
        schema, schema_sha = supervisor.load_schema()
        base = supervisor.v3.v2.base
        saved = (base.EXECUTION_ID, base.POLICY_PATH, base.SCHEMA_PATH)
        try:
            base.EXECUTION_ID = supervisor.EXECUTION_ID
            base.POLICY_PATH = supervisor.POLICY_PATH
            base.SCHEMA_PATH = supervisor.SCHEMA_PATH
            context = base._empty_context(policy, policy_sha, schema_sha)
            failure = base.SupervisorError("MOCK_FAILURE", "static-test", "synthetic only")
            receipt = supervisor.build_execution_receipt(
                policy, schema_sha, context, failure=failure,
                published_path=policy["paths"]["failure_receipt_path"],
            )
            base.validate_execution_receipt(receipt, schema)
        finally:
            base.EXECUTION_ID, base.POLICY_PATH, base.SCHEMA_PATH = saved
        self.assertEqual(receipt["execution_id"], supervisor.EXECUTION_ID)
        self.assertEqual(receipt["failure"]["code"], "MOCK_FAILURE")

    def test_policy_self_hash_and_prior_partial_are_exact(self) -> None:
        policy, _ = supervisor.load_policy()
        self.assertEqual(
            policy["inputs"]["supervisor_v4"]["sha256"],
            hashlib.sha256(Path(supervisor.__file__).read_bytes()).hexdigest(),
        )
        self.assertEqual(
            policy["inputs"]["prior_v3_worker_stderr"]["sha256"],
            "cf5e7bd84886e59cde11a34129b9aee73aa3e454882711fe47f3236d536002c4",
        )
        self.assertTrue(policy["contract"]["connection_data_topic_header_match_required"])
        self.assertTrue(policy["contract"]["outer_schema_execution_identity_exact"])

    def test_topic_or_schema_control_drift_fails_closed(self) -> None:
        policy, _ = supervisor.load_policy()
        for name in (
            "connection_data_topic_header_match_required",
            "outer_schema_execution_identity_exact",
            "prior_v3_partial_preserved",
        ):
            changed = copy.deepcopy(policy)
            changed["contract"][name] = False
            with self.assertRaises(supervisor.v3.v2.base.SupervisorError):
                supervisor.validate_policy(changed)


if __name__ == "__main__":
    unittest.main()
