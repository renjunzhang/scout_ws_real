import copy
from pathlib import Path
import sys
import unittest
import casadi as ca
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning.terminal_speed import normalize_policy, speed_limits
from planning.validation import validate_plan
from planning_fixture import make_plan

POLICY = dict(mode='terminal_speed_envelope_v1', slowdown_distance_m=1.2,
              slowdown_speed_mps=.18)


class TerminalSpeedPolicyTest(unittest.TestCase):
    def task(self):
        task = make_plan()['task']
        task['terminal_speed_policy'] = dict(POLICY)
        task['actuator_parameters'][2] = 1.018
        return task

    def test_progress_and_distance_both_anticipate_runtime_command_cap(self):
        task = self.task()
        for remaining, distance in ((1.2, 2.), (2., 1.2)):
            speed, command_squared = speed_limits(task, remaining, distance)
            self.assertAlmostEqual(float(speed), .18)
            self.assertAlmostEqual(float(command_squared)**.5, .18/1.018)
        # The continuous extension remains finite outside the terminal zone.
        before = float(speed_limits(task, 1.2+1e-8, 2.)[0])
        after = float(speed_limits(task, 1.2-1e-8, 2.)[0])
        self.assertLess(abs(before-after), 1e-7)

    def test_goal_braking_requires_zero_command_and_has_finite_derivative(self):
        task = self.task()
        d = ca.MX.sym('distance')
        _, cap_squared = speed_limits(task, .01, d)
        check = ca.Function('cap_and_derivative', [d], [cap_squared, ca.jacobian(cap_squared,d)])
        at_goal = check(0.)
        self.assertEqual(float(at_goal[0]), 0.)
        self.assertTrue(np.isfinite(float(at_goal[1])))

    def test_new_bound_rejects_legacy_motion_without_changing_dynamics(self):
        plan = make_plan()
        validate_plan(plan)
        policy = dict(POLICY, slowdown_speed_mps=.001)
        plan['task']['terminal_speed_policy'] = copy.deepcopy(policy)
        with self.assertRaisesRegex(ValueError, 'contract mismatch'):
            validate_plan(plan)
        plan['terminal_speed_policy'] = policy
        with self.assertRaisesRegex(ValueError, 'terminal_.*speed'):
            validate_plan(plan)

    def test_rejects_invalid_policy(self):
        task = self.task()
        with self.assertRaises(ValueError):
            normalize_policy(dict(POLICY, slowdown_distance_m=True), task)
        with self.assertRaises(ValueError):
            normalize_policy(dict(POLICY, slowdown_distance_m=task['goal_position_tolerance']), task)


if __name__ == '__main__':
    unittest.main()
