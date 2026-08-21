#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = PACKAGE_ROOT / "tools" / "analysis" / \
    "identify_planar_execution_model.py"
SPEC = importlib.util.spec_from_file_location(
    "identify_planar_execution_model", TOOL_PATH)
IDENTIFY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(IDENTIFY)


class IdentifyPlanarExecutionModelTest(unittest.TestCase):

    def test_tool_schema_freezes_causal_command_contract(self):
        self.assertEqual(
            IDENTIFY.TOOL_SCHEMA,
            "spmpc_planar_execution_identification_v2")
        self.assertEqual(
            IDENTIFY.COMMAND_HOLD_CONTRACT,
            "causal_right_continuous_zoh_v1")

    def test_command_sampling_is_right_continuous_zoh(self):
        stamps = np.asarray([0.0, 1.0, 2.0])
        values = np.asarray([0.0, 1.0, -1.0])
        query = np.asarray([-0.5, 0.0, 0.999, 1.0, 1.5, 2.0, 3.0])
        sampled = IDENTIFY.sample_zoh(query, stamps, values)
        np.testing.assert_array_equal(
            sampled, [0.0, 0.0, 0.0, 1.0, 1.0, -1.0, -1.0])

    def test_delayed_step_never_changes_output_before_effective_epoch(self):
        t = np.arange(0.0, 2.01, 0.25)
        command = np.where(t >= 1.0, 1.0, 0.0)
        output = IDENTIFY.simulate_channel(
            t, command, 0.0, [0.5, 1.0, 1.0, 1.0, 0.0])
        effective_index = int(np.flatnonzero(t >= 1.5)[0])
        np.testing.assert_allclose(output[:effective_index + 1], 0.0)
        self.assertAlmostEqual(
            output[effective_index + 1], 1.0 - np.exp(-0.25), places=12)

    def test_initial_output_is_not_advanced_before_first_sample(self):
        t = np.asarray([0.0, 0.1, 0.2])
        command = np.ones_like(t)
        output = IDENTIFY.simulate_channel(
            t, command, 0.25, [0.0, 0.5, 1.0, 1.0, 0.0])
        self.assertEqual(output[0], 0.25)
        self.assertGreater(output[1], output[0])

    def test_noise_free_zoh_steps_recover_delay_tau_and_directional_gains(self):
        t = np.arange(0.0, 12.0, 0.05)
        command = np.zeros_like(t)
        command[(t >= 1.0) & (t < 3.0)] = 0.20
        command[(t >= 4.0) & (t < 6.0)] = -0.15
        command[(t >= 7.0) & (t < 9.0)] = 0.10
        actual_parameters = [0.15, 0.20, 1.10, 0.90, 0.0]
        measured = IDENTIFY.simulate_channel(
            t, command, 0.0, actual_parameters)
        fit = IDENTIFY.fit_channel(
            {"t": t, "cmd_v": command, "mocap_v": measured},
            "cmd_v", "mocap_v", np.ones_like(t, dtype=bool), "linear")
        recovered = fit["parameters"]
        self.assertAlmostEqual(recovered["delay_sec"], 0.15, places=7)
        self.assertAlmostEqual(
            recovered["time_constant_sec"], 0.20, places=7)
        self.assertAlmostEqual(recovered["positive_gain"], 1.10, places=7)
        self.assertAlmostEqual(recovered["negative_gain"], 0.90, places=7)
        self.assertLess(fit["fit_metrics"]["rmse"], 1.0e-10)

    def test_non_monotonic_command_stamps_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "invalid ZOH"):
            IDENTIFY.sample_zoh(
                [0.0], [0.0, 0.0], [0.0, 1.0])


if __name__ == "__main__":
    unittest.main()
