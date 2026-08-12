#!/usr/bin/env python3
"""Static/mock tests for the create-new S5A0 remediation execution layer."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
import sys
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s5a0_primary_one_shot_supervisor_v2 as supervisor  # noqa: E402
import r8_liquid_s5a0_selected_bag_worker_v4 as worker  # noqa: E402


class SupervisorV2Tests(unittest.TestCase):
    def test_self_check_is_static_and_fd_bind_is_exact(self) -> None:
        result = supervisor.self_check()
        self.assertEqual(result["status"], "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_V2_STATIC_CONTRACT")
        self.assertFalse(result["real_bag_read"])
        self.assertFalse(result["bwrap_executed"])
        policy, _ = supervisor.load_policy()
        argv, digest = supervisor.build_bwrap_argv(policy)
        self.assertEqual(argv.count("--ro-bind-data"), 1)
        marker = argv.index("--ro-bind-data")
        self.assertEqual(argv[marker + 1:marker + 3], ["9", "/selected/capture.bag"])
        self.assertEqual(argv[argv.index("--perms") + 1], "0400")
        self.assertNotIn(policy["selection"]["absolute_path"], argv)
        self.assertNotIn(policy["selection"]["forbidden_optional_path"], argv)
        self.assertEqual(len(digest), 64)

    def test_policy_freezes_four_fresh_targets_and_primary_only(self) -> None:
        policy, _ = supervisor.load_policy()
        targets = [policy["paths"][name] for name in (
            "partial_evidence_root", "final_evidence_root", "final_receipt_path", "failure_receipt_path",
        )]
        self.assertEqual(len(set(targets)), 4)
        self.assertEqual(policy["selection"]["selected_count"], 1)
        self.assertFalse(policy["selection"]["optional_pair_authorized"])
        for key in ("gpu_exposed", "solver_started", "sudo_used", "ros_started", "source_bag_executed"):
            self.assertFalse(policy["contract"][key])
        changed = copy.deepcopy(policy)
        changed["selection"]["optional_pair_authorized"] = True
        with self.assertRaises(supervisor.base.SupervisorError):
            supervisor.validate_policy(changed)

    def test_both_schemas_are_valid_and_deep_closed(self) -> None:
        for path in (supervisor.SCHEMA_PATH, supervisor.INNER_SCHEMA_PATH):
            schema = json.loads(path.read_text(encoding="utf-8"))
            Draft202012Validator.check_schema(schema)
            supervisor.gate_v1.assert_deep_closed(schema)

    def test_source_fd_pread_hash_is_mocked_without_bwrap(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "capture.bag"
            source.write_bytes(b"mock-only-not-a-real-bag")
            source.chmod(0o755)
            policy, _ = supervisor.load_policy()
            mock_policy = copy.deepcopy(policy)
            selection = mock_policy["selection"]
            selection["absolute_path"] = str(source)
            selection["expected_size_bytes"] = source.stat().st_size
            selection["expected_sha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
            descriptor = supervisor._open_frozen_source(mock_policy)
            try:
                self.assertEqual(descriptor, 9)
                self.assertEqual(os.pread(descriptor, source.stat().st_size, 0), source.read_bytes())
            finally:
                os.close(descriptor)

    def test_worker_v4_freezes_dual_host_guest_identity(self) -> None:
        source = Path(worker.__file__).read_text(encoding="utf-8")
        self.assertIn('runtime_policy["selection"]["expected_mode"] = "0400"', source)
        self.assertIn('"expected_mode": policy["selection"]["expected_mode"]', source)
        self.assertIn('"guest_copy_mode": "0400"', source)
        tree = ast.parse(source)
        imported = {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
        self.assertFalse(imported & {"subprocess", "socket", "rosbag", "rospy"})


if __name__ == "__main__":
    unittest.main()
