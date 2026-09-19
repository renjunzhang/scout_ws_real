"""Frozen geometry is independent of a warm trajectory and of solver weights."""
from copy import deepcopy
import unittest
import numpy as np
import casadi as ca
from baselines.fixed_path import fixed_geometry, geometry_error, release_schedule, drain_schedule, progress_jerk_terms, solve_fixed_path
from planning.optimizer import dynamics, PlanValidationError
from baselines.path_transcription import cubic_chord, normalized_displacement, seed_progress_from_motion
from test_zvd_baseline import straight_task
from planning_fixture import make_plan


class FixedPathTest(unittest.TestCase):
    def test_failed_solver_retains_finite_candidate_without_executable_return(self):
        task = straight_task()
        task.update(deadline=5., transport_duration=5., max_iterations=1)
        task['route'] = [[0., 0.], [.1, 0.]]
        task['goal_pose'] = [.1, 0., 0.]
        task['region']['cells'][0]['s_end'] = .1
        task['liquid_policy'] = dict(mode='transport_tail_v1', move_height_m=.0009,
                                    tail_height_m=.00008, height_tolerance_m=.000001)
        with self.assertRaises(PlanValidationError) as caught:
            solve_fixed_path(task)
        candidate = caught.exception.candidate
        self.assertEqual(candidate['optimization']['status'], 'Maximum_Iterations_Exceeded')
        self.assertEqual(candidate['optimization']['iterations'], 1)
        self.assertTrue(np.isfinite([row['state'] for row in candidate['samples']]).all())

    def test_curved_draining_keeps_commands_reachable_until_arrival(self):
        task = straight_task()
        moving = 40
        linear, angular = drain_schedule(moving, False)
        self.assertEqual((linear, angular), (36, 31))
        step = dynamics(task)
        x = np.asarray(task['start_state']).copy()
        history = [x.copy()]
        for k in range(moving+1):
            commands = [.01 if k+1 < linear else 0., .02 if k+1 < angular else 0.]
            u = [(commands[0]-x[6])/task['dt'], (commands[1]-x[7])/task['dt'], 0.]
            x = np.asarray(step(x, u)).ravel(); history.append(x.copy())
        self.assertGreater(history[moving-1][8], 0.)
        self.assertGreater(history[moving-1][13], 0.)
        np.testing.assert_allclose(history[moving][6:23], 0., atol=1e-15)

    def test_seed_progress_includes_delayed_braking_after_command_clear(self):
        task = straight_task()
        _, spline, _ = fixed_geometry(task)
        states = np.zeros((9, 28))
        states[:, 0] = [0., 0., 0., 1., 2., 3., 3.5, 3.75, 3.9]
        early_stopped_coordinate = np.array([0., 0., 0., 1., 3., 5., 5., 5., 5.])
        progress = seed_progress_from_motion(states, spline, early_stopped_coordinate, 7)
        np.testing.assert_allclose(progress[:3], 0.)
        self.assertGreater(progress[7], progress[6])
        self.assertEqual(progress[7], 5.)
        self.assertEqual(progress[8], 5.)

    def test_chord_is_exact_across_knots_and_has_stationary_limit(self):
        task = straight_task()
        task['route'] = [[float(x), .3*float(np.sin(2*np.pi*x/5))] for x in np.linspace(0., 5., 101)]
        _, spline, _ = fixed_geometry(task)
        chord = cubic_chord(spline)
        for knot in spline.x:
            for s, h in [(knot, 0.), (knot-1e-8, 2e-8), (knot-.01, .03), (knot, .1)]:
                actual = np.asarray(chord(s, h)).ravel()
                if h:
                    np.testing.assert_allclose(h*actual, spline(s+h)-spline(s), atol=3e-13)
                else:
                    np.testing.assert_allclose(actual, spline(s, 1), atol=3e-12)

    def test_normalized_displacement_preserves_production_step_at_low_speed(self):
        task = straight_task()
        step = dynamics(task)
        normalized = normalized_displacement(step)
        for speed in [0., 1e-8, .002, .2]:
            x = np.asarray(task['start_state']).copy()
            x[2] = .3; x[5] = .2; x[13] = -.1
            x[3] = speed*.8; x[8] = speed*1.3
            actual = np.asarray(step(x, [0., 0., speed])).ravel()[:2]-x[:2]
            factored = speed*np.asarray(normalized(x, [0., 0., speed], .8, 1.3)).ravel()
            np.testing.assert_allclose(actual, factored, atol=1e-15)

    def test_curved_release_preserves_stationary_geometry_until_steering_reachable(self):
        task = straight_task()
        first, hold = release_schedule(False)
        self.assertEqual((first, hold), (11, 6))
        self.assertEqual(release_schedule(True), (5, 0))
        step = dynamics(task)
        x = np.array(task['start_state'])
        for k in range(first+1):
            x = np.asarray(step(x, [0. if k < hold else .01, .01, 0.])).ravel()
            if k < first:
                np.testing.assert_allclose(x[:2], task['start_state'][:2], atol=1e-14)
                self.assertAlmostEqual(x[3], 0., places=14)
                if k == first-1:
                    self.assertGreater(x[5], 0.)  # Steering can precede translation.
            else:
                self.assertGreater(x[3], 0.)
                self.assertGreater(x[5], 0.)

    def test_lifted_jerk_matches_old_differences_at_release_and_stop(self):
        speed = np.r_[np.zeros(10), [.01, .03, .04, .03, .01], np.zeros(4)]
        dt = 1/30
        opt = ca.Opti()
        v = opt.variable(1, len(speed))
        a, j = progress_jerk_terms(opt, v, dt, 100.)
        acceleration = np.diff(np.r_[0., speed])/dt
        jerk = np.diff(np.r_[0., acceleration])/dt
        opt.set_initial(v, speed); opt.set_initial(a, acceleration); opt.set_initial(j, jerk)
        residual = np.asarray(opt.debug.value(opt.g, opt.initial())).ravel()
        np.testing.assert_allclose(residual[:2*len(speed)], 0., atol=1e-14)
        np.testing.assert_allclose(jerk, np.diff(np.r_[0., 0., speed], n=2)/dt**2, atol=1e-13)

    def test_straight_invariant_and_endpoint_derivatives(self):
        task = straight_task()
        function, spline, metadata = fixed_geometry(task)
        self.assertTrue(metadata['straight'])
        self.assertLess(metadata['maximum_route_edit_m'], 1e-12)
        for s in [-1e-10, 0., .5, 5., 5.+1e-10]:
            np.testing.assert_allclose(np.asarray(function(s)).ravel(), [s, 0.], atol=1e-12)
        np.testing.assert_allclose(spline([0., 5.], 1), [[1., 0.], [1., 0.]], atol=1e-12)

    def test_curved_geometry_is_frozen_before_objective_and_keeps_tangents(self):
        task = straight_task()
        task['route'] = [[float(x), .3*float(np.sin(2*np.pi*x/5))] for x in np.linspace(0., 5., 101)]
        _, spline, first = fixed_geometry(task)
        self.assertFalse(first['straight'])
        changed = deepcopy(task)
        changed['objective']['liquid'] *= 100
        _, _, second = fixed_geometry(changed)
        self.assertEqual(first, second)
        np.testing.assert_allclose(spline([first['knots'][0], first['knots'][-1]], 1), [[1., 0.], [1., 0.]], atol=1e-12)
        self.assertGreater(first['maximum_route_edit_m'], 0.)

    def test_substep_geometry_check_detects_departure_from_frozen_line(self):
        plan = make_plan()
        _, spline, _ = fixed_geometry(plan['task'])
        states = np.asarray([row['state'] for row in plan['samples']])
        controls = np.asarray([row['control'] for row in plan['samples']])
        checked = geometry_error(plan['task'], states, controls, spline)
        self.assertLess(checked['maximum_substep_path_error_m'], checked['substep_tolerance_m'])
        states[40, 1] += .02
        rejected = geometry_error(plan['task'], states, controls, spline)
        self.assertGreaterEqual(rejected['maximum_node_path_error_m'], .02)


if __name__ == '__main__':
    unittest.main()
