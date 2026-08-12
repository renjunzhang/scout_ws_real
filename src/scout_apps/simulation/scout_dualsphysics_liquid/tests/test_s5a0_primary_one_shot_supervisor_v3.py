#!/usr/bin/env python3
"""Static/negative tests for the private-tmpfs S5A0 supervisor v3."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a0_primary_one_shot_supervisor_v3 as supervisor  # noqa: E402
import r8_liquid_s5a0_selected_bag_worker_v5 as worker_v5  # noqa: E402


class SupervisorV3Tests(unittest.TestCase):
    def test_self_check_is_static_and_preserves_prior_failure(self) -> None:
        result = supervisor.self_check()
        self.assertEqual(
            result["status"],
            "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_V3_STATIC_CONTRACT",
        )
        self.assertTrue(result["prior_v2_failure_preserved"])
        self.assertFalse(result["real_bag_read"])
        self.assertFalse(result["bwrap_executed"])
        self.assertEqual(result["source_guest_mode"], "0400")
        self.assertEqual(result["source_guest_nlink"], 1)
        self.assertTrue(result["source_tmpfs_read_only"])

    def test_private_copy_and_read_only_remount_order_is_exact(self) -> None:
        policy, _ = supervisor.load_policy()
        argv, digest = supervisor.build_bwrap_argv(policy)
        expected = [
            "--tmpfs", "/selected", "--perms", "0400", "--file", "9",
            "/selected/capture.bag", "--remount-ro", "/selected",
        ]
        start = argv.index("--tmpfs", argv.index("--tmpfs") + 1)
        self.assertEqual(argv[start:start + len(expected)], expected)
        self.assertNotIn("--ro-bind-data", argv)
        self.assertNotIn("--proc", argv[:argv.index("--")])
        self.assertNotIn("--dev", argv[:argv.index("--")])
        self.assertNotIn(policy["selection"]["absolute_path"], argv)
        self.assertNotIn(policy["selection"]["forbidden_optional_path"], argv)
        self.assertEqual(len(digest), 64)

    def test_policy_self_hash_and_v2_failure_hashes_are_bound(self) -> None:
        policy, _ = supervisor.load_policy()
        inputs = policy["inputs"]
        self.assertEqual(
            inputs["supervisor_v3"]["sha256"],
            hashlib.sha256(Path(supervisor.__file__).read_bytes()).hexdigest(),
        )
        self.assertEqual(
            inputs["prior_v2_failure_receipt"]["sha256"],
            "a1f1101a29ca9be16818f5cc946cb341b4a6d2082b3de7d83739255ad70ef20f",
        )
        self.assertEqual(
            inputs["prior_v2_worker_stderr"]["sha256"],
            "8b8c680ec137432329de7243e48d6a0f5f8a2866f4c7c85e64ad63485153a8cb",
        )
        self.assertTrue(policy["contract"]["prior_v2_failure_preserved"])

    def test_policy_rejects_any_optional_or_materialization_drift(self) -> None:
        policy, _ = supervisor.load_policy()
        for mutate in (
            lambda value: value["selection"].__setitem__("optional_pair_authorized", True),
            lambda value: value["contract"].__setitem__("sandbox_source_tmpfs_remount_ro", False),
            lambda value: value["contract"].__setitem__("sandbox_source_nlink_one", False),
            lambda value: value["limits"].__setitem__("nproc_limit", 2),
        ):
            changed = copy.deepcopy(policy)
            mutate(changed)
            with self.assertRaises(supervisor.v2.base.SupervisorError):
                supervisor.validate_policy(changed)

    def test_source_contains_no_alternate_execution_surface(self) -> None:
        source = Path(supervisor.__file__).read_text(encoding="utf-8")
        tree = ast.parse(source)
        commands = {
            choice
            for node in ast.walk(tree)
            if isinstance(node, ast.Tuple)
            for element in node.elts
            if isinstance(element, ast.Constant) and isinstance(element.value, str)
            for choice in (element.value,)
        }
        self.assertNotIn("optional", commands)
        self.assertNotIn("solver", commands)
        self.assertNotIn("gpu", commands)
        self.assertNotIn("sudo", commands)
        policy = json.loads(supervisor.POLICY_PATH.read_text(encoding="utf-8"))
        self.assertEqual(policy["selection"]["selected_count"], 1)

    def test_canonical_projection_preserves_raw_definition_witnesses(self) -> None:
        raw = [
            {
                "topic": "/cmd_vel", "type": "geometry_msgs/Twist",
                "md5sum": "9" * 32, "message_definition_sha256": "1" * 64,
                "message_definition_size_bytes": 605,
                "canonical_message_definition_sha256": "a" * 64,
                "canonical_message_definition_size_bytes": 604,
            },
            {
                "topic": "/cmd_vel", "type": "geometry_msgs/Twist",
                "md5sum": "9" * 32, "message_definition_sha256": "2" * 64,
                "message_definition_size_bytes": 604,
                "canonical_message_definition_sha256": "a" * 64,
                "canonical_message_definition_size_bytes": 604,
            },
        ]
        bag = {"connections": copy.deepcopy(raw)}
        observed: dict[str, object] = {}
        original = worker_v5._BASE_VALIDATE_TOPIC_CONTRACT
        try:
            def fake(_policy: object, normalized: object) -> dict[str, object]:
                observed["bag"] = normalized
                return {"validated": True}
            worker_v5._BASE_VALIDATE_TOPIC_CONTRACT = fake
            self.assertTrue(worker_v5.validate_topic_contract_v3({}, bag)["validated"])
        finally:
            worker_v5._BASE_VALIDATE_TOPIC_CONTRACT = original
        projected = observed["bag"]
        self.assertIsInstance(projected, dict)
        self.assertEqual(
            [row["message_definition_sha256"] for row in projected["connections"]],
            ["a" * 64, "a" * 64],
        )
        self.assertEqual(
            [row["message_definition_sha256"] for row in bag["connections"]],
            ["1" * 64, "2" * 64],
        )

    def test_canonical_substantive_conflict_fails_closed(self) -> None:
        bag = {"connections": [
            {
                "topic": "/cmd_vel", "type": "geometry_msgs/Twist",
                "md5sum": "9" * 32, "message_definition_sha256": "1" * 64,
                "message_definition_size_bytes": 605,
                "canonical_message_definition_sha256": "a" * 64,
                "canonical_message_definition_size_bytes": 604,
            },
            {
                "topic": "/cmd_vel", "type": "geometry_msgs/Twist",
                "md5sum": "9" * 32, "message_definition_sha256": "2" * 64,
                "message_definition_size_bytes": 604,
                "canonical_message_definition_sha256": "b" * 64,
                "canonical_message_definition_size_bytes": 604,
            },
        ]}
        with self.assertRaisesRegex(worker_v5.WorkerV5Error, "canonical same-topic"):
            worker_v5.validate_topic_contract_v3({}, bag)


if __name__ == "__main__":
    unittest.main()
