#!/usr/bin/env python3
"""Static/negative tests for S5A0 padded-header supervisor v5."""

from __future__ import annotations

import copy
import hashlib
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a0_primary_one_shot_supervisor_v5 as supervisor  # noqa: E402


class SupervisorV5Tests(unittest.TestCase):
    def test_self_check_binds_exact_padded_header_revision(self) -> None:
        result = supervisor.self_check()
        self.assertEqual(
            result["status"],
            "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_V5_STATIC_CONTRACT",
        )
        self.assertEqual(result["reader_revision"], "v4")
        self.assertEqual(result["worker_revision"], "v7")
        self.assertTrue(result["file_header_padding_preserved_in_memory"])
        self.assertFalse(result["source_repaired_or_written"])
        self.assertTrue(result["prior_v1_v2_v3_v4_evidence_preserved"])
        self.assertFalse(result["real_bag_read"])
        self.assertFalse(result["bwrap_executed"])

    def test_exact_sandbox_binds_and_optional_is_never_exposed(self) -> None:
        policy, _ = supervisor.load_policy()
        argv, digest = supervisor.build_bwrap_argv(policy)
        for name in ("reader_v4", "worker_v6", "worker_v7"):
            self.assertEqual(argv.count(policy["inputs"][name]["path"]), 1)
        self.assertEqual(argv.count("--file"), 1)
        self.assertEqual(argv.count("--remount-ro"), 1)
        self.assertNotIn("--ro-bind-data", argv)
        self.assertNotIn("--proc", argv[:argv.index("--")])
        self.assertNotIn("--dev", argv[:argv.index("--")])
        self.assertNotIn(policy["selection"]["absolute_path"], argv)
        self.assertNotIn(policy["selection"]["forbidden_optional_path"], argv)
        self.assertFalse(policy["selection"]["optional_pair_authorized"])
        self.assertEqual(len(digest), 64)

    def test_failure_receipt_validates_against_exact_v5_identity(self) -> None:
        policy, policy_sha = supervisor.load_policy()
        schema, schema_sha = supervisor.load_schema()
        base = supervisor.v4.v3.v2.base
        saved = (base.EXECUTION_ID, base.POLICY_PATH, base.SCHEMA_PATH)
        try:
            base.EXECUTION_ID = supervisor.EXECUTION_ID
            base.POLICY_PATH = supervisor.POLICY_PATH
            base.SCHEMA_PATH = supervisor.SCHEMA_PATH
            context = base._empty_context(policy, policy_sha, schema_sha)
            failure = base.SupervisorError(
                "MOCK_FAILURE", "static-test", "synthetic only"
            )
            receipt = supervisor.build_execution_receipt(
                policy,
                schema_sha,
                context,
                failure=failure,
                published_path=policy["paths"]["failure_receipt_path"],
            )
            base.validate_execution_receipt(receipt, schema)
        finally:
            base.EXECUTION_ID, base.POLICY_PATH, base.SCHEMA_PATH = saved
        self.assertEqual(receipt["execution_id"], supervisor.EXECUTION_ID)
        self.assertEqual(
            receipt["schema_version"],
            "smpcc-r8-liquid-s5a0-primary-supervisor-execution-receipt-v5",
        )
        self.assertEqual(receipt["failure"]["code"], "MOCK_FAILURE")

    def test_policy_hashes_bind_supervisor_and_v4_failure(self) -> None:
        policy, _ = supervisor.load_policy()
        self.assertEqual(
            policy["inputs"]["supervisor_v5"]["sha256"],
            hashlib.sha256(Path(supervisor.__file__).read_bytes()).hexdigest(),
        )
        self.assertEqual(
            policy["inputs"]["prior_v4_failure_receipt"]["sha256"],
            "49a571be2feddfa8b1d66a2742ced2be72449d3b607ec254b58f64417fc5e8c9",
        )
        self.assertEqual(
            policy["inputs"]["prior_v4_worker_stderr"]["sha256"],
            "2f94b9964c2844ef3aab92e36a261b3bf42dd27986eb890c3ae13fc85cd74371",
        )
        self.assertEqual(
            policy["inputs"]["outer_schema_v5"]["sha256"],
            hashlib.sha256(supervisor.SCHEMA_PATH.read_bytes()).hexdigest(),
        )

    def test_padding_or_immutability_control_drift_fails_closed(self) -> None:
        policy, _ = supervisor.load_policy()
        for name in (
            "file_header_padding_preserved_in_memory",
            "file_header_padding_exact_boundary_verified",
            "prior_v4_failure_preserved",
            "prior_v4_worker_failure_bound",
        ):
            changed = copy.deepcopy(policy)
            changed["contract"][name] = False
            with self.assertRaises(supervisor.v4.v3.v2.base.SupervisorError):
                supervisor.validate_policy(changed)
        changed = copy.deepcopy(policy)
        changed["contract"]["source_repaired_or_written"] = True
        with self.assertRaises(supervisor.v4.v3.v2.base.SupervisorError):
            supervisor.validate_policy(changed)


if __name__ == "__main__":
    unittest.main()
