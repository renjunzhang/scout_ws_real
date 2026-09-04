#!/usr/bin/env python3
"""Contracts for the generated explicit command/actual OCP model."""

import json
import sys
import unittest
from pathlib import Path

import casadi as ca
import numpy as np


ACADOS_DIR = Path(__file__).resolve().parents[1] / "acados"
GENERATED_DIR = Path(__file__).resolve().parents[2] / "generated" / "acados"
sys.path.insert(0, str(ACADOS_DIR))

from generate_spmpc_acados import default_parameter_values, load_config  # noqa: E402
from spmpc_acados_cost import stage_cost_expr  # noqa: E402
from spmpc_acados_model import (  # noqa: E402
    ACCEL_MEMORY_INDEX,
    ANGULAR_QUEUE_START,
    LINEAR_QUEUE_START,
    NP,
    NP_SLOSH,
    NX,
    NX_SLOSH,
    PIDX,
    SLOSH_STATE_OFFSET,
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
        self.assertEqual((b0["nx"], b0["nu"], b0["np"]), (24, 3, 28))
        self.assertEqual(
            (slosh["nx"], slosh["nu"], slosh["np"]), (28, 3, 37)
        )
        self.assertEqual((NX, NX_SLOSH, NP, NP_SLOSH), (24, 28, 28, 37))
        self.assertEqual((ACCEL_MEMORY_INDEX, SLOSH_STATE_OFFSET), (23, 24))

    def test_partial_condensing_horizon_is_frozen(self):
        self.assertEqual(self.cfg["qp_solver_cond_N"], 10)

    def test_generated_solver_json_contract(self):
        for name, expected_nx in (("spmpc_b0", 24), ("spmpc_slosh", 28)):
            path = GENERATED_DIR / name / "acados_ocp_{}.json".format(name)
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dims"]["nx"], expected_nx)
            self.assertEqual(payload["dims"]["N"], 60)
            self.assertEqual(payload["solver_options"]["qp_solver_cond_N"], 10)

    def test_fifo_shifts_and_appends_next_command_state(self):
        symbols = export_spmpc_b0_symbols()
        step = transition(symbols)
        x = np.zeros(NX)
        x[6] = 0.10
        x[7] = -0.20
        x[LINEAR_QUEUE_START:ANGULAR_QUEUE_START] = np.arange(5) + 1.0
        x[ANGULAR_QUEUE_START:ACCEL_MEMORY_INDEX] = np.arange(10) + 11.0
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
            result[ANGULAR_QUEUE_START:ACCEL_MEMORY_INDEX - 1],
            x[ANGULAR_QUEUE_START + 1:ACCEL_MEMORY_INDEX],
        )
        self.assertAlmostEqual(
            result[ACCEL_MEMORY_INDEX - 1], x[7] + u[1] * dt, places=12
        )

    def test_acceleration_memory_advances_to_current_command_on_both_models(self):
        for symbols, nx in (
            (export_spmpc_b0_symbols(), NX),
            (export_spmpc_slosh_symbols(), NX_SLOSH),
        ):
            step = transition(symbols)
            x = np.zeros(nx)
            x[ACCEL_MEMORY_INDEX] = -0.41
            u = np.array([0.27, 0.0, 0.0])
            p = default_parameter_values(
                self.cfg, with_slosh=symbols["with_slosh"]
            )
            result = np.asarray(step(x, u, p)).reshape(-1)
            self.assertAlmostEqual(result[ACCEL_MEMORY_INDEX], u[0], places=12)

    def test_stage_cost_uses_state_memory_not_legacy_a_prev(self):
        symbols = export_spmpc_b0_symbols()
        cost = ca.Function(
            "full_horizon_da_cost",
            [symbols["x"], symbols["u"], symbols["p"]],
            [stage_cost_expr(symbols, self.cfg)],
        )
        x = np.zeros(NX)
        u = np.array([0.30, 0.0, 0.0])
        p = default_parameter_values(self.cfg, with_slosh=False)
        p[PIDX["a_prev"]] = u[0]
        p[PIDX["w_du_a"]] = 0.0
        without_continuity = float(cost(x, u, p))
        p[PIDX["w_du_a"]] = 1.0
        with_state_delta = float(cost(x, u, p))
        self.assertAlmostEqual(
            with_state_delta - without_continuity,
            (u[0] / self.cfg["a_max"]) ** 2 / self.cfg["N"],
            places=12,
        )
        x[ACCEL_MEMORY_INDEX] = u[0]
        self.assertAlmostEqual(float(cost(x, u, p)), without_continuity, places=12)

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
        np.testing.assert_allclose(
            result[SLOSH_STATE_OFFSET:NX_SLOSH], np.zeros(4), atol=1.0e-12
        )


if __name__ == "__main__":
    unittest.main()
