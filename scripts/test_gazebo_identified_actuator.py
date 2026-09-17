#!/usr/bin/env python3
"""Analytic step/hold/timeout checks for the development vehicle adapter."""
import math
import unittest

from gazebo_identified_actuator import DelayedAxis


class ActuatorTest(unittest.TestCase):
    def test_step_waits_for_delay_then_tracks_identified_gain_and_tau(self):
        axis = DelayedAxis(.2, .1, 1.1, .5, 0.)
        axis.command(0., .3)
        self.assertEqual(axis.sample(.199), 0.)
        self.assertEqual(axis.sample(.2), 0.)
        self.assertAlmostEqual(axis.sample(.3), .33*(1-math.exp(-1)))
        at_timeout = .33*(1-math.exp(-5))
        self.assertAlmostEqual(axis.sample(.8), at_timeout*math.exp(-1))

    def test_sparse_updates_integrate_every_command_edge(self):
        axis = DelayedAxis(.2, .1, 1., .5, 0.)
        axis.command(0., 1.)
        axis.command(.1, -1.)
        after_positive = 1.-math.exp(-1.)
        self.assertAlmostEqual(axis.sample(.4), -1.+(after_positive+1.)*math.exp(-1.))

    def test_new_command_after_timeout_and_clock_regression(self):
        axis = DelayedAxis(.2, .1, 1., .5, 0.)
        axis.command(0., 1.)
        axis.command(1., .2)
        before_restart = (1.-math.exp(-5.))*math.exp(-5.)
        self.assertAlmostEqual(axis.sample(1.3), .2+(before_restart-.2)*math.exp(-1.))
        with self.assertRaises(ValueError):
            axis.sample(1.2)


if __name__ == '__main__':
    unittest.main()
