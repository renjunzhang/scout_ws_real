#!/usr/bin/env python3
"""Synthetic-only tests for S5A1 v2 static admission and one-shot boundaries."""

from __future__ import annotations

import copy
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_extractor_worker_v2 as worker  # noqa: E402
import r8_liquid_s5a1_one_shot_supervisor_v2 as supervisor  # noqa: E402


class S5A1StaticLayerV2Tests(unittest.TestCase):
    def test_self_check_is_not_admitted_without_final_parent(self) -> None:
        result = supervisor.self_check()
        policy = json.loads(supervisor.POLICY.read_text(encoding="utf-8"))
        present = os.path.lexists(policy["s5a0_v2"]["outer_final_receipt"])
        self.assertEqual(result["s5a0_outer_final_present"], present)
        if not present:
            self.assertEqual(result["status"], "NOT_ADMITTED_S5A0_V2_FINAL_RECEIPT_ABSENT")
        self.assertFalse(result["real_input_read"])
        self.assertFalse(result["external_write"])
        self.assertFalse(result["bwrap_run"])

    def test_sandbox_argv_is_isolated_bounded_and_capability_free(self) -> None:
        policy, _schema, _sha = supervisor.load_static()
        argv = supervisor.build_bwrap_argv(policy)
        self.assertIn("--unshare-net", argv)
        self.assertEqual(argv[argv.index("--cap-drop") + 1], "ALL")
        self.assertNotIn("--proc", argv)
        self.assertNotIn("--dev", argv)
        python = argv.index("/usr/bin/python3")
        self.assertEqual(argv[python:python + 3], ["/usr/bin/python3", "-I", "-B"])
        order = policy["execution_contract"]["state_order"]
        self.assertLess(order.index("RESERVE_FAILURE_RECEIPT"), order.index("RUN_ONE_SANDBOX_WORKER"))

    def test_worker_is_not_directly_host_runnable(self) -> None:
        with self.assertRaisesRegex(worker.WorkerV2Error, "not directly host-runnable"):
            worker.sandbox_worker(worker.EXPECTED_TOKEN)

    def test_exact_preroll_is_equality_not_minimum(self) -> None:
        worker.validate_exact_preroll(5.0, 6.0)
        for first in (5.999, 6.001):
            with self.assertRaisesRegex(worker.WorkerV2Error, "exactly"):
                worker.validate_exact_preroll(5.0, first)

    def test_outer_host_and_inner_guest_identities_are_distinct(self) -> None:
        result = worker.synthetic_self_check()
        self.assertEqual(result["status"], "PASS_S5A1_V2_WORKER_SYNTHETIC_SELF_CHECK")
        host = {"path": worker.HOST_PATH, "mode": "0755", "size_bytes": worker.HOST_SIZE, "sha256": worker.HOST_SHA, "nlink": 1}
        guest = {"path": worker.GUEST_PATH, "mode": "0400", "size_bytes": worker.HOST_SIZE, "sha256": worker.HOST_SHA, "nlink": 1}
        outer = {"status": "PASS_S5A0_PRIMARY_ONE_SHOT_DEVELOPMENT_ONLY", "failure": None, "host_source": {
            "file_before": host, "file_after": copy.deepcopy(host), "path_unchanged": True,
            "mount_unchanged": True, "file_unchanged": True, "root_unchanged": True}}
        inner = {"status": "S5A0_SELECTED_R7_BAG_SEALED_DEVELOPMENT_ONLY", "source": {"selected_after": guest},
                 "host_source_provenance": {"absolute_path": worker.HOST_PATH, "expected_mode": "0755", "guest_copy_mode": "0400", "guest_copy_distinct_inode": True}}
        worker.validate_s5a0_v2_parent(outer, inner)
        inner["source"]["selected_after"]["path"] = worker.HOST_PATH
        with self.assertRaises(worker.WorkerV2Error):
            worker.validate_s5a0_v2_parent(outer, inner)

    def test_failure_receipt_reservation_is_o_excl_0600_and_fsynced(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "failure.json.partial"
            final = Path(directory) / "failure.json"
            descriptor = supervisor.reserve_failure_receipt(path)
            self.assertEqual(stat.S_IMODE(os.fstat(descriptor).st_mode), 0o600)
            with self.assertRaises(FileExistsError):
                supervisor.reserve_failure_receipt(path)
            supervisor.publish_reserved(descriptor, path, final, {"status": "MOCK_FAILURE"})
            self.assertFalse(path.exists())
            self.assertEqual(json.loads(final.read_text())["status"], "MOCK_FAILURE")


if __name__ == "__main__":
    unittest.main()
