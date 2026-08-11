#!/usr/bin/env python3
"""Focused tests for the evidence-based v3 memory-floor revision."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = PACKAGE_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import r8_liquid_u3_gpu_damped_restart_runner_v1 as base
import r8_liquid_u3_gpu_damped_restart_runner_v3 as runner


POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_damped_restart_20260811T040906Z_v3.json"


class DampedRestartRunnerV3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        runner.configure()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))

    def test_v3_policy_loads_with_exact_three_gib_floor(self) -> None:
        loaded = base.load_policy(POLICY_PATH)
        floors = {phase["limits"]["minimum_mem_available_bytes"] for phase in loaded["phases"].values()}
        self.assertEqual(floors, {runner.HARD_MEMORY_FLOOR_BYTES})
        proof = runner.semantic_validate_v3(loaded, POLICY_PATH)
        self.assertFalse(proof["resource_revision"]["numerical_contract_changed"])

    def test_rejects_any_other_memory_floor(self) -> None:
        for value in (2147483648, 4294967296):
            candidate = copy.deepcopy(self.policy)
            for phase in candidate["phases"].values():
                phase["limits"]["minimum_mem_available_bytes"] = value
            with self.assertRaises(base.DampedRestartRunError):
                runner.semantic_validate_v3(candidate, POLICY_PATH)

    def test_solver_argv_damping_and_thresholds_remain_v1_exact(self) -> None:
        loaded = base.load_policy(POLICY_PATH)
        for phase_name in base.PHASE_KEYS:
            self.assertEqual(loaded["phases"][phase_name]["solver_argv"], base._exact_solver_argv(phase_name))
        self.assertEqual(loaded["damping_contract"]["redumax_per_s"], 10)
        self.assertTrue(loaded["acceptance"]["all_17_absolute_limits_required"])

    def test_v2_inventory_semantics_remain_active(self) -> None:
        self.assertIs(base.legacy.safe_output_inventory, runner.inventory_v2.safe_output_inventory_v2)


if __name__ == "__main__":
    unittest.main()
