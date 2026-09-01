#!/usr/bin/env python3
"""Static contracts for the optional pre-path runtime identity gate."""

import sys
import unittest
from pathlib import Path


TEST_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = TEST_DIR.parent
ANALYSIS_DIR = SCRIPTS_DIR / "analysis"
RUNNER = SCRIPTS_DIR / "run_spmpc_real_fixed_path_trial.sh"
ENGINE = SCRIPTS_DIR / "lib" / "run_spmpc_i0_failclosed_fixed_abba_engine.sh"
DIAGNOSTICS = SCRIPTS_DIR.parent / "src" / "ros" / "diagnostics_publisher.cpp"

sys.path.insert(0, str(ANALYSIS_DIR))
import i0_failclosed_fixed_abba_profile as profiles  # noqa: E402


class Short100RuntimeGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = RUNNER.read_text(encoding="utf-8")
        cls.engine = ENGINE.read_text(encoding="utf-8")
        cls.diagnostics = DIAGNOSTICS.read_text(encoding="utf-8")

    def test_gate_is_optional_for_generic_and_legacy_runs(self):
        self.assertIn(
            'EXPECTED_RUNTIME_VARIANT="${EXPECTED_RUNTIME_VARIANT:-}"',
            self.runner,
        )
        self.assertIsNone(
            profiles.get_profile("legacy_v1").strict_runtime_contract
        )

    def test_short100_pins_runtime_inputs_and_expected_variant(self):
        runtime = profiles.get_profile("short100_v2").strict_runtime_contract
        self.assertIsNotNone(runtime)
        self.assertEqual(runtime.solver_backend, "continuous_mpcc_acados")
        self.assertEqual(runtime.cmd_topic, "/cmd_vel")
        self.assertEqual(runtime.reference_path_topic, "/scout/global_path_fixed")
        self.assertEqual(runtime.imu_topic, "/imu/data")
        self.assertEqual(runtime.mocap_tracker, "Tracker0")
        self.assertEqual(
            (runtime.w_smooth, runtime.w_alpha, runtime.w_du_a, runtime.w_du_vs),
            (0.1, 0.1, 0.1, 0.1),
        )
        self.assertEqual(runtime.slosh_height_max, 0.001)
        self.assertEqual(runtime.alpha_max, 1.2)
        self.assertIn('"EXPECTED_RUNTIME_VARIANT=${VARIANT}"', self.engine)
        self.assertIn('"SOLVER_BACKEND=${I0FC_RUNTIME_SOLVER_BACKEND}"', self.engine)
        self.assertIn('"MATRIX_PRESET="', self.engine)
        self.assertIn(
            '"RECORDER_SCRIPT=${SCRIPT_DIR}/record_spmpc_full_rgb_bag.sh"',
            self.engine,
        )

    def test_runtime_variant_passes_before_path_release(self):
        start = self.runner.index("  start_planner\n", self.runner.index("if truthy \"${IMU_SHADOW_ENABLE}\""))
        ready = self.runner.index("  wait_for_imu_shadow_ready\n", start)
        variant = self.runner.index("  wait_for_runtime_variant\n", ready)
        path = self.runner.index("  start_path_source true\n", variant)
        self.assertLess(start, ready)
        self.assertLess(ready, variant)
        self.assertLess(variant, path)
        self.assertIn(
            'EXPECTED_RUNTIME_VARIANT requires the no-reference IMU startup gate',
            self.runner,
        )

    def test_solver_evidence_publishers_have_burst_queues(self):
        self.assertIn(
            'advertise<PredictedHorizon>("debug/predicted_horizon", 10)',
            self.diagnostics,
        )
        self.assertIn(
            'advertise<PreSolveSnapshot>("debug/pre_solve_snapshot", 10)',
            self.diagnostics,
        )


if __name__ == "__main__":
    unittest.main()
