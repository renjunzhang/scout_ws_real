#!/usr/bin/env python3
"""Development-only OCP accuracy checks using independent SciPy references.

Run in acados_venv; acquisition preflight only uses test_explicit_actuator_model.
"""
import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.linalg import expm
from scipy.integrate import solve_ivp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_explicit_actuator_model import transition
from generate_spmpc_acados import default_parameter_values, load_config
from spmpc_acados_model import (
    export_spmpc_slosh_symbols, NX_SLOSH, PIDX, PIDX_SLOSH,
    SLOSH_STATE_OFFSET, LINEAR_QUEUE_START, ANGULAR_QUEUE_START,
    ACCEL_MEMORY_INDEX, EXPLICIT_ACTUATOR_RK4_SUBSTEPS,
)


class ExplicitActuatorIntegrationAccuracyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = load_config()

    def physical_slosh_case(self):
        symbols = export_spmpc_slosh_symbols()
        self.assertEqual(symbols["integration_substeps"], EXPLICIT_ACTUATOR_RK4_SUBSTEPS)
        p = default_parameter_values(self.cfg, with_slosh=True)
        # Recorded C03 tube parameters; codegen's omega_n=5 placeholder is too slow
        # to catch this discretization error at the deployed tube frequency.
        p[PIDX_SLOSH["omega_n_sq"]] = 1047.0568854135606
        p[PIDX_SLOSH["two_zeta_omega_n"]] = 3.2358258380412885
        return symbols, transition(symbols), p

    def test_free_liquid_decay_matches_exact_reference_over_two_seconds(self):
        _, step, p = self.physical_slosh_case()
        dt = p[PIDX["actuator_dt"]]
        w2 = p[PIDX_SLOSH["omega_n_sq"]]
        damping = p[PIDX_SLOSH["two_zeta_omega_n"]]
        matrix = np.array([[0., 1.], [-w2, -damping]])
        initial = np.array([[.002, -.001], [.015, -.02]])
        x = np.zeros(NX_SLOSH)
        x[SLOSH_STATE_OFFSET:] = initial.T.ravel()
        # Compare full signed state, scaled by natural frequency, not just a
        # height sample which may happen to coincide at an oscillation zero.
        scale = np.array([1., 1/np.sqrt(w2)])[:, None]
        for k in range(1, 61):
            x = np.asarray(step(x, np.zeros(3), p)).ravel()
            expected = expm(matrix * k * dt) @ initial
            actual = x[SLOSH_STATE_OFFSET:].reshape(2, 2).T
            error = np.linalg.norm((actual-expected)*scale)/np.linalg.norm(expected*scale)
            self.assertLess(error, .004, "2 s relative modal-state accuracy")

    def test_forced_actuator_and_liquid_match_independent_fine_ode(self):
        _, step, p = self.physical_slosh_case()
        dt = p[PIDX["actuator_dt"]]
        tau_v, tau_w = p[PIDX["actuator_tau_v"]], p[PIDX["actuator_tau_omega"]]
        gain_v, gain_w = p[PIDX["actuator_gain_v"]], p[PIDX["actuator_gain_omega"]]
        w2, damp = p[PIDX_SLOSH["omega_n_sq"]], p[PIDX_SLOSH["two_zeta_omega_n"]]
        x = np.zeros(NX_SLOSH)
        x[:8] = [.1, -.2, .3, .12, .2, .08, .13, .1]
        x[LINEAR_QUEUE_START:ANGULAR_QUEUE_START] = .13
        x[ANGULAR_QUEUE_START:ACCEL_MEMORY_INDEX] = .1
        x[SLOSH_STATE_OFFSET:] = [.001, .01, -.0005, -.02]
        reference = np.r_[x[:8], x[SLOSH_STATE_OFFSET:]]
        for k in range(60):
            u = np.array([.05*np.sin(.3*k), .2*np.cos(.2*k), .2])
            delayed_v, delayed_w = x[LINEAR_QUEUE_START], x[ANGULAR_QUEUE_START]

            def independent_rhs(_, z):
                ax = (gain_v*delayed_v-z[3])/tau_v
                alpha = (gain_w*delayed_w-z[5])/tau_w
                return [z[3]*np.cos(z[2]), z[3]*np.sin(z[2]), z[5], ax, u[2], alpha,
                        u[0], u[1], z[9], -damp*z[9]-w2*z[8]-ax,
                        z[11], -damp*z[11]-w2*z[10]-z[3]*z[5]]

            exact = solve_ivp(independent_rhs, (0, dt), reference,
                              method="DOP853", rtol=1e-11, atol=1e-13)
            self.assertTrue(exact.success)
            reference = exact.y[:, -1]
            x = np.asarray(step(x, u, p)).ravel()
            np.testing.assert_allclose(x[:8], reference[:8], rtol=0, atol=2e-8)
            liquid_error = (x[SLOSH_STATE_OFFSET:]-reference[8:])*[1, 1/np.sqrt(w2), 1, 1/np.sqrt(w2)]
            self.assertLess(np.linalg.norm(liquid_error), 2e-6)


if __name__ == "__main__":
    unittest.main()
