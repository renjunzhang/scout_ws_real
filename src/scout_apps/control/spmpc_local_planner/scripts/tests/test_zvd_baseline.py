"""Physical/algorithmic checks, not a Gazebo repetition matrix."""
from copy import deepcopy
import numpy as np
import unittest

from baselines.zvd import impulses, generate_plan
from planning_fixture import make_plan


def straight_task():
    task = deepcopy(make_plan()["task"])
    task.update(deadline=30., transport_duration=25.2, stop_window=5.,
                route=[[0., 0.], [5., 0.]], goal_pose=[5., 0., 0.])
    task["region"]["cells"][0].update(s_end=5., vertices=[[-1., -1.], [6., -1.], [6., 1.], [-1., 1.]])
    return task


class ZvdBaselineTest(unittest.TestCase):
    def test_zvd_cancels_mode_and_first_frequency_derivative(self):
        task = straight_task()
        times, weights, kernel = impulses(task["liquid_parameters"], task["dt"])
        damping, omega_sq = task["liquid_parameters"][:2]
        pole = -.5*damping + 1j*np.sqrt(omega_sq-.25*damping*damping)
        residual = np.exp(pole*(times[-1]-times))
        assert abs(weights @ residual) < 1e-12
        assert abs(weights @ ((times[-1]-times)*residual)) < 1e-12
        self.assertAlmostEqual(np.sum(kernel), 1.)
        self.assertAlmostEqual(np.arange(len(kernel))*task["dt"] @ kernel, times @ weights)


    def test_discrete_zvd_preserves_distance_and_production_history(self):
        plan = generate_plan(straight_task(), base_duration=24.)
        assert plan["baseline"]["ideal_residual_ratio"] < .01
        self.assertAlmostEqual(plan["validation"]["actual_path_length"], 5., places=8)
        assert plan["validation"]["dynamics_max_error"] < 2e-6
        assert plan["validation"]["stopping_evaluation"]["windows"]["covered"]
        assert plan["samples"][1]["state"][3] == 0.  # Real FIFO latency.
        task = straight_task()
        task["start_state"][24] = 1e-5
        disturbed = generate_plan(task, base_duration=24.)
        assert disturbed["samples"][0]["state"][24] == 1e-5
        assert disturbed["samples"][1]["state"][24] != 0.  # Never reset residual liquid.


    def test_zvd_rejects_curves_policy_aliasing_and_truncated_tail(self):
        task = straight_task()
        task["route"].insert(1, [2., .1])
        task["region"]["cells"][0]["s_end"] = 6.
        with self.assertRaisesRegex(ValueError, "straight"):
            generate_plan(task, base_duration=24.)
        task = straight_task()
        task["liquid_constraint_enable"] = True
        with self.assertRaisesRegex(ValueError, "qualification"):
            generate_plan(task, base_duration=24.)
        task = straight_task()
        task["transport_duration"] = 24.3
        with self.assertRaisesRegex(ValueError, "drain"):
            generate_plan(task, base_duration=24.)


if __name__ == "__main__":
    unittest.main()
