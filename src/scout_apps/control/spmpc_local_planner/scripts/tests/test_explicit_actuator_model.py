#!/usr/bin/env python3
"""Contracts for the generated explicit command/actual OCP model."""

import sys
import unittest
from pathlib import Path

import casadi as ca
import numpy as np


ACADOS_DIR = Path(__file__).resolve().parents[1] / "acados"
sys.path.insert(0, str(ACADOS_DIR))

from generate_spmpc_acados import default_parameter_values, load_config  # noqa: E402
from spmpc_acados_model import (  # noqa: E402
    ANGULAR_QUEUE_START,
    LINEAR_QUEUE_START,
    NP,
    NP_SLOSH,
    NX,
    NX_SLOSH,
    PIDX,
    export_spmpc_b0_symbols,
    export_spmpc_slosh_symbols,
)


def transition(symbols):
    return ca.Function(
        "transition_{}".format(symbols["name"]),
        [symbols["x"], symbols["u"], symbols["p"]],
        [symbols["disc_dyn"]],
    )


class ExplicitActuatorModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config()

    def test_dimensions_and_parameter_contract(self):
        b0 = export_spmpc_b0_symbols()
        slosh = export_spmpc_slosh_symbols()
        self.assertEqual((b0["nx"], b0["nu"], b0["np"]), (23, 3, 28))
        self.assertEqual(
            (slosh["nx"], slosh["nu"], slosh["np"]), (27, 3, 37)
        )
        self.assertEqual((NX, NX_SLOSH, NP, NP_SLOSH), (23, 27, 28, 37))

    def test_partial_condensing_horizon_is_frozen(self):
        self.assertEqual(self.cfg["qp_solver_cond_N"], 10)

    def test_fifo_shifts_and_appends_next_command_state(self):
        symbols = export_spmpc_b0_symbols()
        step = transition(symbols)
        x = np.zeros(NX)
        x[6] = 0.10
        x[7] = -0.20
        x[LINEAR_QUEUE_START:ANGULAR_QUEUE_START] = np.arange(5) + 1.0
        x[ANGULAR_QUEUE_START:23] = np.arange(10) + 11.0
        u = np.array([0.30, -0.60, 0.10])
        p = default_parameter_values(self.cfg, with_slosh=False)

        result = np.asarray(step(x, u, p)).reshape(-1)
        dt = p[PIDX["actuator_dt"]]
        np.testing.assert_allclose(
            result[LINEAR_QUEUE_START:ANGULAR_QUEUE_START - 1],
            x[LINEAR_QUEUE_START + 1:ANGULAR_QUEUE_START],
        )
        self.assertAlmostEqual(
            result[ANGULAR_QUEUE_START - 1], x[6] + u[0] * dt, places=12
        )
        np.testing.assert_allclose(
            result[ANGULAR_QUEUE_START:22], x[ANGULAR_QUEUE_START + 1:23]
        )
        self.assertAlmostEqual(result[22], x[7] + u[1] * dt, places=12)

    def test_new_command_cannot_change_actual_before_fifo_delay(self):
        symbols = export_spmpc_b0_symbols()
        step = transition(symbols)
        x = np.zeros(NX)
        p = default_parameter_values(self.cfg, with_slosh=False)

        for k in range(5):
            u = np.array([0.60 if k == 0 else 0.0, 0.0, 0.0])
            x = np.asarray(step(x, u, p)).reshape(-1)
            self.assertAlmostEqual(x[3], 0.0, places=12)
        x = np.asarray(step(x, np.zeros(3), p)).reshape(-1)
        self.assertGreater(x[3], 0.0)

    def test_b0_and_slosh_share_actual_robot_dynamics(self):
        b0 = export_spmpc_b0_symbols()
        slosh = export_spmpc_slosh_symbols()
        b0_step = transition(b0)
        slosh_step = transition(slosh)
        x_b0 = np.zeros(NX)
        x_b0[3] = 0.12
        x_b0[5] = -0.08
        x_b0[LINEAR_QUEUE_START] = 0.20
        x_b0[ANGULAR_QUEUE_START] = 0.15
        x_slosh = np.concatenate((x_b0, np.zeros(4)))
        u = np.array([0.25, -0.30, 0.18])
        p_b0 = default_parameter_values(self.cfg, with_slosh=False)
        p_slosh = default_parameter_values(self.cfg, with_slosh=True)

        result_b0 = np.asarray(b0_step(x_b0, u, p_b0)).reshape(-1)
        result_slosh = np.asarray(slosh_step(x_slosh, u, p_slosh)).reshape(-1)
        np.testing.assert_allclose(result_b0, result_slosh[:NX], atol=1.0e-12)

    def test_liquid_is_not_excited_by_command_difference_directly(self):
        symbols = export_spmpc_slosh_symbols()
        step = transition(symbols)
        x = np.zeros(NX_SLOSH)
        p = default_parameter_values(self.cfg, with_slosh=True)

        result = np.asarray(step(x, np.array([0.60, 0.0, 0.0]), p)).reshape(-1)
        np.testing.assert_allclose(result[23:27], np.zeros(4), atol=1.0e-12)


if __name__ == "__main__":
    unittest.main()
