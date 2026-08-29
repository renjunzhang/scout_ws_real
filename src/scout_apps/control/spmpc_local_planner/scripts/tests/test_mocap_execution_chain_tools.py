#!/usr/bin/env python3

import importlib.util
import json
import math
import pathlib
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPT_ROOT = pathlib.Path(__file__).resolve().parents[1]
ANALYSIS_ROOT = SCRIPT_ROOT / "analysis"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PATH_TOOL = load_module("validate_mocap_s_path", ANALYSIS_ROOT / "validate_mocap_s_path.py")
ANALYZER = load_module(
    "analyze_mocap_execution_chain", ANALYSIS_ROOT / "analyze_mocap_execution_chain.py"
)
POSTFLIGHT = load_module(
    "validate_mocap_execution_chain_bag",
    ANALYSIS_ROOT / "validate_mocap_execution_chain_bag.py",
)


class MocapPathValidationTest(unittest.TestCase):
    def write_path(self, points):
        directory = tempfile.TemporaryDirectory()
        path = pathlib.Path(directory.name) / "path.json"
        payload = {
            "frame_id": "map",
            "poses": [
                {
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                    "qw": 1.0,
                }
                for x, y in points
            ],
        }
        path.write_text(json.dumps(payload), encoding="utf-8")
        return directory, path

    def test_s_path_has_both_turn_directions(self):
        x_values = np.linspace(0.0, 2.0, 81)
        points = [(float(x), float(0.22 * math.sin(math.pi * x))) for x in x_values]
        directory, path = self.write_path(points)
        self.addCleanup(directory.cleanup)
        _payload, loaded, nonfinite = PATH_TOOL.load_points(path)
        metrics = PATH_TOOL.path_metrics(loaded, 1e-5)
        self.assertEqual(nonfinite, 0)
        self.assertGreater(metrics["positive_turn_rad"], 0.10)
        self.assertGreater(metrics["negative_turn_rad"], 0.10)
        self.assertGreaterEqual(metrics["turn_sign_changes"], 1)

    def test_straight_path_does_not_pass_s_geometry(self):
        points = [(0.05 * index, 0.0) for index in range(30)]
        directory, path = self.write_path(points)
        self.addCleanup(directory.cleanup)
        _payload, loaded, _nonfinite = PATH_TOOL.load_points(path)
        metrics = PATH_TOOL.path_metrics(loaded, 1e-5)
        self.assertEqual(metrics["positive_turn_rad"], 0.0)
        self.assertEqual(metrics["negative_turn_rad"], 0.0)
        self.assertEqual(metrics["turn_sign_changes"], 0)


class ExecutionAnalysisMathTest(unittest.TestCase):
    def test_first_order_delay_fit_recovers_synthetic_model(self):
        dt = 0.02
        time = np.arange(0.0, 16.0, dt)
        command = np.zeros_like(time)
        command[(time >= 1.0) & (time < 4.0)] = 0.20
        command[(time >= 5.0) & (time < 8.0)] = -0.12
        command[(time >= 9.0) & (time < 13.0)] = 0.16
        response = 0.01 + 0.92 * ANALYZER.simulate_first_order(
            time, command, delay_sec=0.12, tau_sec=0.18
        )
        result = ANALYZER.fit_first_order_delay(time, command, response, 0.40)
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["delay_sec"], 0.12, delta=0.03)
        self.assertAlmostEqual(result["tau_sec"], 0.18, delta=0.05)
        self.assertAlmostEqual(result["gain"], 0.92, delta=0.03)
        self.assertGreater(result["r2"], 0.99)

        validation = ANALYZER.evaluate_first_order_model(time, command, response, result)
        self.assertTrue(validation["valid"])
        self.assertGreater(validation["r2"], 0.99)

    def test_heldout_uses_frozen_training_model_without_refit(self):
        time = np.arange(0.0, 8.0, 0.02)
        command = np.zeros_like(time)
        command[(time >= 1.0) & (time < 3.0)] = 0.2
        command[(time >= 4.0) & (time < 6.0)] = -0.1
        training_response = ANALYZER.simulate_first_order(time, command, 0.1, 0.2)
        heldout_response = -training_response
        model = {
            "valid": True,
            "delay_sec": 0.1,
            "tau_sec": 0.2,
            "gain": 1.0,
            "offset": 0.0,
            "r2": 0.99,
        }
        reports = [
            {
                "bag": "training_{}.bag".format(index),
                "execution_chain": {
                    "linear": {"first_order_plus_delay": dict(model)},
                    "angular": {"first_order_plus_delay": dict(model)},
                },
            }
            for index in range(3)
        ]
        reports.append(
            {
                "bag": "heldout.bag",
                "execution_chain": {
                    "linear": {"first_order_plus_delay": dict(model)},
                    "angular": {"first_order_plus_delay": dict(model)},
                },
            }
        )
        inputs = [
            {
                "linear": {"time": time, "command": command, "response": training_response},
                "angular": {"time": time, "command": command, "response": training_response},
            }
            for _ in range(3)
        ]
        inputs.append(
            {
                "linear": {"time": time, "command": command, "response": heldout_response},
                "angular": {"time": time, "command": command, "response": heldout_response},
            }
        )
        aggregate = ANALYZER.aggregate_reports(reports, inputs, 3, 0.05, 0.0)
        self.assertFalse(aggregate["channels"]["linear"]["held_out_pass"])
        self.assertEqual(aggregate["decision"], "INCONCLUSIVE_OR_REDESIGN")

    def test_mocap_pose_derives_longitudinal_speed(self):
        time = np.arange(0.0, 5.0, 0.01)
        rows = [(float(value), float(0.2 * value), 0.0, 0.0) for value in time]
        result = ANALYZER.derive_mocap_kinematics(rows, 100.0, 0.05)
        interior = result["v"][20:-20]
        self.assertAlmostEqual(float(np.median(interior)), 0.2, delta=0.005)
        self.assertLess(float(np.max(np.abs(result["omega"][20:-20]))), 1e-6)


class PostflightMathTest(unittest.TestCase):
    def test_goal_reached_hold_does_not_require_solver_artifacts(self):
        terminal_hold = SimpleNamespace(
            solve_attempted=True,
            solve_success=True,
            solver_status="GOAL_REACHED",
        )
        active_solve = SimpleNamespace(
            solve_attempted=True,
            solve_success=True,
            solver_status="B_slosh_matched5_ACADOS_OK",
        )
        failed_solve = SimpleNamespace(
            solve_attempted=True,
            solve_success=False,
            solver_status="ACADOS_FAILURE",
        )

        self.assertFalse(POSTFLIGHT.requires_solver_artifacts(terminal_hold))
        self.assertTrue(POSTFLIGHT.requires_solver_artifacts(active_solve))
        self.assertFalse(POSTFLIGHT.requires_solver_artifacts(failed_solve))

    def test_state_alignment_accepts_common_epoch_or_explicit_no_liquid(self):
        common_epoch = SimpleNamespace(
            state_alignment_required=True,
            state_time_aligned=True,
            state_alignment_status="INTERPOLATED",
        )
        no_liquid = SimpleNamespace(
            state_alignment_required=False,
            state_time_aligned=True,
            state_alignment_status="LIQUID_NOT_CONSUMED",
        )
        unsafe_bypass = SimpleNamespace(
            state_alignment_required=False,
            state_time_aligned=True,
            state_alignment_status="COMMON_EPOCH_DISABLED",
        )

        self.assertTrue(POSTFLIGHT.state_alignment_contract_ok(common_epoch))
        self.assertTrue(POSTFLIGHT.state_alignment_contract_ok(no_liquid))
        self.assertFalse(POSTFLIGHT.state_alignment_contract_ok(unsafe_bypass))

    def test_stream_stats_reports_rate_and_gap(self):
        stream = POSTFLIGHT.StreamStats(True)
        for index in range(11):
            stamp = 0.01 * index + 1.0
            stream.update(stamp, stamp, (float(index),))
        summary = stream.summary()
        self.assertEqual(summary["count"], 11)
        self.assertAlmostEqual(summary["rate_hz"], 100.0, delta=1e-6)
        self.assertAlmostEqual(summary["bag_gap_max_sec"], 0.01, delta=1e-9)
        self.assertEqual(summary["header_regressions"], 0)

    def test_stream_stats_rejects_missing_and_regressing_headers(self):
        stream = POSTFLIGHT.StreamStats(True)
        stream.update(1.0, 1.0, (0.0,))
        stream.update(1.1, 0.0, (0.0,))
        stream.update(1.2, 0.9, (0.0,))
        summary = stream.summary()
        self.assertEqual(summary["invalid_header_stamps"], 1)
        self.assertEqual(summary["header_regressions"], 1)

    def test_stationary_pose_metrics_detects_small_jitter(self):
        rows = [
            (1.0 + index * 1e-4, 2.0, 0.5, 0.2 + index * 1e-5, 1.0)
            for index in range(20)
        ]
        metrics = POSTFLIGHT.stationary_pose_metrics(rows)
        self.assertEqual(metrics["count"], 20)
        self.assertLess(metrics["position_deviation_p95_m"], 0.002)
        self.assertLess(metrics["yaw_deviation_p95_rad"], 0.001)


class ReleaseWiringTest(unittest.TestCase):
    def test_runner_requires_motion_arm_hash_and_mocap(self):
        wrapper = (SCRIPT_ROOT / "run_spmpc_mocap_execution_chain_trial.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('ARM_MOTION="${ARM_MOTION:-NO}"', wrapper)
        self.assertIn("REQUIRE_PATH_HASH=true", wrapper)
        self.assertIn("RECORD_MOCAP=true", wrapper)
        self.assertIn("RECORD_MOCAP_PATH=false", wrapper)
        self.assertIn("validate_mocap_execution_chain_bag.py", wrapper)

    def test_common_runner_forwards_mocap_contract(self):
        runner = (SCRIPT_ROOT / "run_spmpc_real_fixed_path_trial.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('RECORD_MOCAP="${RECORD_MOCAP}"', runner)
        self.assertIn('RECORD_MOCAP_PATH="${RECORD_MOCAP_PATH}"', runner)
        self.assertIn('MOCAP_TRACKER="${MOCAP_TRACKER}"', runner)

    def test_static_smoke_cannot_start_motion(self):
        smoke = (SCRIPT_ROOT / "record_spmpc_mocap_static_smoke.sh").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("roslaunch spmpc_local_planner", smoke)
        self.assertNotIn("rostopic pub", smoke)
        self.assertIn("--mode static", smoke)
        self.assertIn("RECORD_MOCAP_PATH=false", smoke)


if __name__ == "__main__":
    unittest.main()
