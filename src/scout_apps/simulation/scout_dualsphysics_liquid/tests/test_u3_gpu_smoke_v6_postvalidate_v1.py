#!/usr/bin/env python3
"""Static/read-only tests for the V6 GPU smoke postvalidator."""

from __future__ import annotations

import hashlib
import importlib.util
import os
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
SCRIPT_PATH = ROOT / "scripts/r8_liquid_u3_gpu_smoke_v6_postvalidate_v1.py"


def load_validator():
    spec = importlib.util.spec_from_file_location("gpu_smoke_v6_postvalidate", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuSmokeV6PostvalidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.validator = load_validator()

    def test_empty_regular_file_is_admitted_and_hashed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.log"
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            os.close(descriptor)
            observed = self.validator.identity_allow_empty(path)
            self.assertEqual(observed["size_bytes"], 0)
            self.assertEqual(observed["sha256"], hashlib.sha256(b"").hexdigest())

    def test_all_frozen_parent_hashes_match(self) -> None:
        observed = self.validator.verify_parents()
        self.assertEqual(set(observed), set(self.validator.PARENTS))
        self.assertEqual(observed["v6_stderr"]["size_bytes"], 0)

    def test_log_safety_admits_only_empty_stderr_and_completion_markers(self) -> None:
        evidence = self.validator.log_safety()
        self.assertTrue(evidence["empty_stderr_admitted"])
        self.assertEqual(evidence["stderr_bytes"], 0)
        self.assertTrue(all(evidence["completion_markers"].values()))
        self.assertFalse(any(evidence["forbidden_matches"].values()))

    def test_resource_evidence_proves_active_gpu_and_memory_floor(self) -> None:
        evidence = self.validator.resource_evidence()
        self.assertGreaterEqual(evidence["sample_count"], 2)
        self.assertGreater(evidence["gpu_memory_used_mib_max"], evidence["gpu_memory_used_mib_min"])
        self.assertGreaterEqual(evidence["gpu_power_draw_w_max"], 60)
        self.assertGreaterEqual(evidence["minimum_mem_available_bytes"], 4294967296)

    def test_exact_output_inventory_is_30_files_and_21_parts(self) -> None:
        policy, _ = self.validator.v6.load_contract()
        inventory = self.validator.base.scan_exact_output(
            self.validator.OUTPUT_ROOT,
            maximum_total=policy["limits"]["maximum_output_bytes"],
        )
        self.assertEqual(inventory["file_count"], 30)
        self.assertEqual(inventory["part_file_count"], 21)

    def test_self_check_is_read_only_and_candidate_stays_disarmed(self) -> None:
        result = self.validator.self_check()
        self.assertEqual(result["status"], "PASS_GPU_SMOKE_V6_POSTVALIDATION_SELF_CHECK")
        self.assertFalse(result["candidate_executed"])
        self.assertEqual(result["candidate"]["mode"], "0400")
        self.assertEqual(result["candidate"]["sha256"], self.validator.CANDIDATE["sha256"])


if __name__ == "__main__":
    unittest.main()
