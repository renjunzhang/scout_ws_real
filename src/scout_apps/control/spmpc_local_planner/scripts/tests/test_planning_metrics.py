"""Substep peaks must survive when sampled endpoints are deceptively small."""
import sys
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# Import task first to load the shared production-model path.
from planning.task import load_task
from planning.metrics import dense_heights, stopping_metrics


class LiquidMetricsTest(unittest.TestCase):
    def test_dense_propagation_keeps_interior_peak_and_tail_window(self):
        task = dict(dt=1/30, height_coeff=1., actuator_parameters=[1.]*4,
                    liquid_parameters=[.1, 1., 1., 1.], stop_window=1/30,
                    goal_pose=[0., 0., 0.], goal_position_tolerance=.05,
                    goal_yaw_tolerance=.1, stop_speed_tolerance=.01, stop_omega_tolerance=.02)
        states = np.zeros((3, 28))
        controls = np.zeros((3, 3))
        states[0, 0] = .1
        # A held-input interval with zero node heights and a mid-interval peak.
        def oscillation(z, command, actuator, liquid, dt):
            value = z.copy()
            value[6] += np.pi/2
            value[5] = abs(np.sin(value[6])) * .003
            return value
        with patch('planning.metrics.motion_functions', return_value=(oscillation,)):
            times, heights = dense_heights(task, states)
        self.assertAlmostEqual(max(heights), .003)
        result = stopping_metrics(task, states, controls, times, heights)
        self.assertAlmostEqual(result['t_stop_sec'], 1/30)
        self.assertAlmostEqual(result['windows']['tail_peak_m'], .003)
        self.assertTrue(result['windows']['covered'])
        self.assertLess(abs(heights[-1]), 1e-12)
