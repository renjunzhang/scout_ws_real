import copy
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parents[2] / "src/scout_apps/control/spmpc_local_planner/scripts"))
from planning.warm_start import warm_start_values


class WarmStartTest(unittest.TestCase):
    def setUp(self):
        self.task = dict(dt=1/30, progress_scale_min=.1, progress_scale_max=10.)
        self.plan = dict(samples=[dict(t=k/30, state=[float(k)] * 28, control=[.1, .2, .3])
                                  for k in range(4)],
                         progress_parameterization=dict(method="scaled_actual_speed_trapezoid", scale=1.046))

    def test_preserves_entire_guess_and_progress_scale(self):
        original = copy.deepcopy(self.plan)
        x, u, scale = warm_start_values(self.plan, self.task, 3)
        np.testing.assert_array_equal(x.T, [r["state"] for r in self.plan["samples"]])
        np.testing.assert_array_equal(u.T, [r["control"] for r in self.plan["samples"][:-1]])
        self.assertEqual(scale, 1.046)
        self.assertEqual(self.plan, original)

    def test_rejects_same_shape_but_compressed_time_and_nonfinite_guess(self):
        self.plan["samples"][1]["t"] *= .9
        with self.assertRaisesRegex(ValueError, "grid"):
            warm_start_values(self.plan, self.task, 3)
        self.plan["samples"][1]["t"] = 1/30
        self.plan["samples"][1]["state"][24] = float("nan")
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            warm_start_values(self.plan, self.task, 3)

    def test_legacy_metadata_absence_and_bad_scale(self):
        self.plan["progress_parameterization"]["scale"] = -1
        with self.assertRaisesRegex(ValueError, "scale"):
            warm_start_values(self.plan, self.task, 3)
        del self.plan["progress_parameterization"]
        self.assertEqual(warm_start_values(self.plan, self.task, 3)[2], 1.)
