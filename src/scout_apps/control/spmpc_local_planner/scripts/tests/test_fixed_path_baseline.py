"""Frozen geometry is independent of a warm trajectory and of solver weights."""
from copy import deepcopy
import unittest
import numpy as np
from baselines.fixed_path import fixed_geometry, geometry_error
from test_zvd_baseline import straight_task
from planning_fixture import make_plan


class FixedPathTest(unittest.TestCase):
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
