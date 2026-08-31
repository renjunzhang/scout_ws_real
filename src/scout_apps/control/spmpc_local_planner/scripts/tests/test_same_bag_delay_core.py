#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest

import numpy as np


CORE_PATH = pathlib.Path(__file__).resolve().parents[1] / "analysis" / "same_bag_delay_core.py"
SPEC = importlib.util.spec_from_file_location("same_bag_delay_core", str(CORE_PATH))
CORE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CORE)


class SameBagDelayCoreTest(unittest.TestCase):
    def test_vectorized_fopdt_fit_recovers_synthetic_model(self):
        time = np.arange(0.0, 18.0, 0.02)
        command = np.zeros_like(time)
        command[(time >= 1.0) & (time < 5.0)] = 0.20
        command[(time >= 6.0) & (time < 10.0)] = -0.12
        command[(time >= 11.0) & (time < 15.0)] = 0.16
        basis = CORE.simulate_fopdt(time, command, 0.12, 0.24)
        response = 0.01 + 0.93 * basis
        model = CORE.fit_fopdt_grid(
            time,
            command,
            response,
            max_delay_sec=0.40,
            min_tau_sec=0.02,
            max_tau_sec=0.60,
            tau_count=80,
        )
        self.assertTrue(model["valid"])
        self.assertAlmostEqual(model["delay_sec"], 0.12, delta=0.021)
        self.assertAlmostEqual(model["tau_sec"], 0.24, delta=0.03)
        self.assertAlmostEqual(model["gain"], 0.93, delta=0.02)
        self.assertGreater(model["r2"], 0.999)
        self.assertAlmostEqual(model["response_time_sec"]["t90"], 0.672, delta=0.07)

    def test_frozen_model_is_evaluated_without_heldout_refit(self):
        time = np.arange(0.0, 10.0, 0.02)
        command = np.zeros_like(time)
        command[(time >= 1.0) & (time < 4.0)] = 0.2
        command[(time >= 5.0) & (time < 8.0)] = -0.1
        models = [
            {"valid": True, "delay_sec": 0.10, "tau_sec": 0.20, "gain": 1.0, "offset": 0.0},
            {"valid": True, "delay_sec": 0.12, "tau_sec": 0.22, "gain": 0.98, "offset": 0.01},
        ]
        frozen = CORE.frozen_median_model(models)
        self.assertAlmostEqual(frozen["delay_sec"], 0.11)
        heldout = -CORE.simulate_fopdt(time, command, 0.11, 0.21)
        metrics = CORE.evaluate_fopdt(time, command, heldout, frozen)
        self.assertTrue(metrics["valid"])
        self.assertLess(metrics["r2"], 0.0)

    def test_relative_lag_sign_matches_nokov_convention(self):
        dt = 0.005
        time = np.arange(0.0, 8.0, dt)
        imu = np.sin(2.0 * np.pi * 0.8 * time) + 0.2 * np.sin(2.0 * np.pi * 1.7 * time)
        # NOKOV appears 15 ms earlier, so its value at t matches IMU at t+15 ms.
        nokov = np.interp(time + 0.015, time, imu, right=imu[-1])
        estimate = CORE.estimate_relative_lag(imu, nokov, dt, 0.10)
        self.assertAlmostEqual(estimate["lag_sec"], -0.015, delta=dt)
        self.assertGreater(estimate["peak_correlation_abs"], 0.99)

    def test_crossing_match_reports_positive_response_delay(self):
        time = np.arange(0.0, 4.0, 0.01)
        command = np.zeros_like(time)
        response = np.zeros_like(time)
        command[time >= 1.0] = 1.0
        response[time >= 1.18] = 1.0
        command_events = CORE.sustained_crossings(time, command, 0.5, 1)
        response_events = CORE.sustained_crossings(time, response, 0.5, 1)
        matches = CORE.match_crossing_events(command_events, response_events)
        self.assertEqual(len(matches), 1)
        self.assertAlmostEqual(matches[0][2], 0.18, delta=0.011)

    def test_crossing_match_does_not_accept_pre_command_response(self):
        matches = CORE.match_crossing_events([1.0], [0.95])
        self.assertEqual(matches, [(1.0, None, None)])

    def test_near_optimal_boundary_check(self):
        model = {
            "valid": True,
            "grid": {
                "delay_min_sec": 0.0,
                "delay_max_sec": 0.6,
                "tau_min_sec": 0.01,
                "tau_max_sec": 1.5,
            },
            "near_optimal_profile": {
                "delay_min_sec": 0.02,
                "delay_max_sec": 0.16,
                "tau_min_sec": 0.02,
                "tau_max_sec": 0.20,
            },
        }
        self.assertFalse(CORE.near_optimal_touches_grid_boundary(model))
        model["near_optimal_profile"]["delay_min_sec"] = 0.0
        self.assertTrue(CORE.near_optimal_touches_grid_boundary(model))


if __name__ == "__main__":
    unittest.main()
