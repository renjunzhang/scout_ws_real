import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))
from simulation_observation import ObservationWindow


class ObservationTest(unittest.TestCase):
    def test_fixed_clock_horizon_includes_deadline_and_entire_tail(self):
        window = ObservationWindow(100., 45., 5.)
        self.assertEqual(window.duration_sec, 51.)
        self.assertFalse(window.complete(145.))
        self.assertFalse(window.complete(149.99))
        self.assertTrue(window.complete(151.))

    def test_paused_clock_cannot_finish_and_regression_is_rejected(self):
        window = ObservationWindow(100., 45., 5.)
        for _ in range(20):
            self.assertFalse(window.complete(120.))
        with self.assertRaises(ValueError):
            window.complete(99.)
