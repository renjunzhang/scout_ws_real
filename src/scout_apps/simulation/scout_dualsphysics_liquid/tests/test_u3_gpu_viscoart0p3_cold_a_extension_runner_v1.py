#!/usr/bin/env python3
"""Static and negative tests for the 35.05--45.05 s extension runner."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_gpu_viscoart0p3_cold_a_extension_runner_v1 as runner  # noqa: E402


class SettleExtensionRunnerTests(unittest.TestCase):
    def test_exact_single_gpu_command(self) -> None:
        argv = runner.exact_solver_argv()
        self.assertEqual(argv.count("-gpu:0"), 1)
        self.assertEqual(argv.count("-ompthreads:1"), 1)
        self.assertEqual(argv.count("-partbegin:701:701"), 1)
        self.assertEqual(argv.count("-tmax:45.05"), 1)
        self.assertEqual([x for x in argv if x.startswith("-visco")], ["-viscoart:0.3"])

    def test_only_end_time_delta_is_frozen(self) -> None:
        self.assertEqual(runner.EXPECTED_DELTA["parameter"], "SIMULATION_END_TIME_S")
        self.assertEqual(runner.EXPECTED_DELTA["baseline"], "35.05")
        self.assertEqual(runner.EXPECTED_DELTA["candidate"], "45.05")
        self.assertFalse(runner.EXPECTED_DELTA["other_numerical_parameters_changed"])

    def test_output_and_resource_bounds_are_closed(self) -> None:
        self.assertEqual(runner.EXPECTED_OUTPUT["part_count"], 201)
        self.assertEqual(runner.EXPECTED_OUTPUT["part_first"], 701)
        self.assertEqual(runner.EXPECTED_OUTPUT["part_last"], 901)
        self.assertEqual(runner.EXPECTED_LIMITS["minimum_mem_available_bytes"], 4294967296)
        self.assertEqual(runner.EXPECTED_LIMITS["wall_timeout_seconds"], 1200)

    def test_alias_policy_path_is_rejected_before_execution(self) -> None:
        with self.assertRaises(runner.SettleExtensionRunError):
            runner.semantic_validate({"run_id": runner.RUN_ID}, Path("/tmp/alias.json"))


if __name__ == "__main__":
    unittest.main()
