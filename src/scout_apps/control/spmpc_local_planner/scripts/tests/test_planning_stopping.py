import numpy as np
import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planning.stopping import find_stop_time, stopping_windows


def task():
    return dict(goal_pose=[1., 2., np.pi], goal_position_tolerance=.01,
                goal_yaw_tolerance=.01, stop_speed_tolerance=.02,
                stop_omega_tolerance=.02)


def samples(n=5):
    x = np.zeros((n, 28)); u = np.zeros((n, 2)); x[:, :3] = [1, 2, np.pi]
    return x, u


class StoppingTest(unittest.TestCase):
    def test_stop_requires_persistent_quiet_suffix_and_wraps_yaw(self):
        x, u = samples(); x[:, 2] = -np.pi
        x[1, 3] = .1; x[2, 3] = 0.03; x[3, 3] = 0.0
        # A later movement invalidates the earlier apparent stop.
        x[3, 6] = 1e-3; x[4, 6] = 0
        self.assertEqual(find_stop_time(np.arange(5.), x, u, task()), 4.)


    def test_commands_and_memory_use_clear_tolerance(self):
        x, u = samples(2); u[1, 0] = 3e-6
        self.assertIsNone(find_stop_time([0., 1.], x, u, task()))


    def test_windows_empty_transport_and_missing_middle_gap(self):
        r = stopping_windows([2., 3., 5.], [1., 4., 2.], 2., 3., max_sample_gap_sec=1.1)
        self.assertIsNone(r["transport_peak_m"])
        self.assertEqual(r["tail_peak_m"], 4.)
        self.assertFalse(r["covered"])


    def test_windows_reject_bad_timebase(self):
        with self.assertRaises(ValueError):
            stopping_windows([0., 0.], [1., 2.], 0., 1.)

    def test_interpolated_stop_and_full_windows(self):
        r = stopping_windows([0., .4, .8, 1.2, 1.6, 2.0], [1., 2., 8., 3., 4., 1.], 1.1, .7, 0.5)
        self.assertTrue(r["covered"])
        self.assertAlmostEqual(r["tail_peak_m"], 4.25)

    def test_missing_first_or_last_sample_is_incomplete(self):
        self.assertFalse(stopping_windows([.1, .5, 1.0], [1, 2, 1], .4, .5)["covered"])
        self.assertFalse(stopping_windows([0., .5, .9], [1, 2, 1], .4, .6)["covered"])

    def test_gap_crossing_window_boundary_is_not_hidden_by_interpolation(self):
        result = stopping_windows([0., 1., 3., 4.], [0., 1., 2., 0.], 1.8, .4, 1.1)
        self.assertFalse(result["tail_covered"])

    def test_empty_transport_at_zero_and_invalid_inputs(self):
        result = stopping_windows([0., .5, 1.], [1., 3., 0.], 0., 1., .6)
        self.assertTrue(result["covered"])
        self.assertIsNone(result["transport_peak_m"])
        self.assertEqual(result["tail_peak_m"], 3.)
        with self.assertRaises(ValueError):
            stopping_windows([0., 1.], [0., -1.], 0., 1.)

    def test_virtual_progress_does_not_delay_physical_stop_but_fifo_does(self):
        x, _ = samples(3)
        u = np.zeros((3, 3))
        u[:, 2] = .01
        x[0, 13] = .02
        self.assertEqual(find_stop_time([0., 1., 2.], x, u, task()), 1.)
