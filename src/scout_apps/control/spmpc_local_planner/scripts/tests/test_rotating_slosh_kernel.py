#!/usr/bin/env python3
"""Independent physics checks and real C++ / symbolic OCP propagation tests.

No ROS runtime is used. A temporary header suppresses only ROS log macros in
the existing modal-parameter provider; no dynamics or observer is mocked.
"""

import ctypes as ct
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import casadi as ca
import numpy as np
from scipy.linalg import expm
from scipy.integrate import solve_ivp
from types import SimpleNamespace

PLANNER = Path(__file__).resolve().parents[2]
CONTROL = PLANNER.parent
sys.path.insert(0, str(PLANNER / "scripts/acados"))
sys.path.insert(0, str(PLANNER / "scripts/analysis"))
from generate_slosh_kernel import generate, generated_functions
from generate_spmpc_acados import default_parameter_values, load_config
import horizon_liquid_replay as replay
from rotating_liquid_replay import replay_ocp
from spmpc_acados_model import (export_spmpc_slosh_symbols, PIDX_SLOSH,
                               LINEAR_QUEUE_START, ANGULAR_QUEUE_START, SLOSH_STATE_OFFSET)


def array(value):
    return np.ascontiguousarray(value, dtype=np.float64)


class RotatingSloshKernelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="spmpc_rotating_slosh_")
        cls.build = Path(cls.temp.name)
        logging = cls.build / "ros/ros.h"
        logging.parent.mkdir()
        logging.write_text("#pragma once\n" + "".join(
            f"#define {name}(...) ((void)0)\n" for name in
            ("ROS_INFO", "ROS_WARN", "ROS_ERROR", "ROS_WARN_THROTTLE", "ROS_ERROR_THROTTLE")))
        cls.generated = PLANNER / "src/dynamics/generated"
        subprocess.run(["gcc", "-O2", "-fPIC", "-c", str(cls.generated / "slosh_kernel_generated.c"),
                        "-o", str(cls.build / "kernel.o")], check=True)
        sources = [PLANNER / "src" / p for p in
                   ("dynamics/slosh_dynamics.cpp", "dynamics/actual_motion_propagator.cpp",
                    "estimation/slosh_observer_bank.cpp",
                    "estimation/liquid_state_nowcaster.cpp")]
        sources += [CONTROL / "slosh_models/src/liquid_slosh_model.cpp",
                    Path(__file__).with_name("rotating_slosh_bridge.cpp")]
        subprocess.run(["g++", "-std=c++14", "-O2", "-shared", "-fPIC",
                        "-I" + str(cls.build), "-I/usr/include/eigen3",
                        "-I" + str(PLANNER / "include"), "-I" + str(CONTROL / "slosh_models/include"),
                        *map(str, sources), str(cls.build / "kernel.o"),
                        "-o", str(cls.build / "bridge.so")], check=True)
        cls.lib = ct.CDLL(str(cls.build / "bridge.so"))
        pointer = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        cls.lib.liquid_parameters.argtypes = [pointer]
        cls.lib.liquid_step.argtypes = [pointer, pointer, ct.c_double, pointer]
        cls.lib.actual_motion_step.argtypes = [pointer, pointer, pointer, ct.c_double, pointer]
        cls.lib.observer_steps.argtypes = [pointer, ct.c_int, pointer]
        cls.lib.nowcast_step.argtypes = [pointer, pointer, ct.c_double, pointer]
        cls.physical = np.zeros(5)
        cls.lib.liquid_parameters(cls.physical)
        cls.rhs, cls.rk4, cls.motion_rk4 = generated_functions()
        cls.symbols = export_spmpc_slosh_symbols()
        cls.ocp = ca.Function("ocp_step", [cls.symbols[k] for k in ("x", "u", "p")],
                             [cls.symbols["disc_dyn"]])

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def step(self, x, excitation, dt):
        result = np.zeros(4)
        self.assertEqual(self.lib.liquid_step(array(x), array(excitation), dt, result), 1)
        return result

    def test_generated_c_is_reproducible(self):
        destination = self.build / "regenerated"
        generate(destination)
        for name in ("slosh_kernel_generated.c", "slosh_kernel_generated.h", "slosh_kernel_contract.h"):
            self.assertEqual((destination / name).read_bytes(), (self.generated / name).read_bytes())

    def test_rhs_matches_independent_rotating_coordinate_derivative(self):
        # Undamped isotropic oscillator in world coordinates; rotate its exact
        # solution into a frame with nonzero omega and alpha, then differentiate.
        q0, v0 = array([.003, -.002]), array([.011, .017])
        wn, omega, alpha = 7.0, -.8, 1.7
        j = np.array([[0., -1.], [1., 0.]])
        def q(t):
            theta = omega * t + .5 * alpha * t*t
            rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            return rotation.T @ (q0 * np.cos(wn*t) + v0 * np.sin(wn*t)/wn)
        relative_v = v0 - omega * j @ q0
        state = array([q0[0], relative_v[0], q0[1], relative_v[1]])
        h = 2e-5
        expected = (q(h) - 2*q(0.) + q(-h)) / h**2
        result = np.asarray(self.rhs(state, [0, 0, omega, alpha], [0, wn*wn, 1, 1])).ravel()
        np.testing.assert_allclose(result[[1, 3]], expected, atol=1e-8, rtol=1e-7)

    def test_zero_rotation_agrees_with_old_exact_zoh_within_rk4_error(self):
        damping, wn2 = self.physical[:2]
        a = np.array([[0, 1, 0, 0], [-wn2, -damping, 0, 0],
                      [0, 0, 0, 1], [0, 0, -wn2, -damping]])
        augmented = np.zeros((6, 6)); augmented[:4, :4] = a
        augmented[1, 4] = -1; augmented[3, 5] = -1
        x, excitation, dt = array([.001, .02, -.003, .01]), array([.2, -.3, 0, 0]), 1/30
        expected = (expm(augmented*dt) @ np.r_[x, excitation[:2]])[:4]
        actual = self.step(x, excitation, dt)
        np.testing.assert_allclose(actual, expected, atol=4e-6, rtol=1e-4)
        # Halving the RK4 substep must improve the numerical approximation.
        refined = x.copy()
        for _ in range(8):
            refined = np.asarray(self.rk4(refined, excitation, self.physical[:4], dt/8)).ravel()
        self.assertLess(np.linalg.norm(refined-expected), np.linalg.norm(actual-expected)/10)

    def test_zero_installation_offset_does_not_remove_rotation(self):
        x = array([.003, .012, -.002, -.008])
        rotating = self.step(x, [0, 0, .8, -.6], 1/30)
        stationary = self.step(x, [0, 0, 0, 0], 1/30)
        self.assertGreater(np.linalg.norm(rotating - stationary), 1e-5)

    def test_cpp_matches_symbolic_rhs_integration_with_variable_sensor_intervals(self):
        rng = np.random.default_rng(610)
        for dt in (.01, .02, 1/30, .035):
            x = rng.normal(size=4) * .005
            excitation = rng.normal(size=4) * .4
            expected = x.copy()
            count = max(4, int(np.ceil(dt*120-1e-12)))
            for _ in range(count):
                expected = np.asarray(self.rk4(expected, excitation, self.physical[:4], dt/count)).ravel()
            np.testing.assert_allclose(self.step(x, excitation, dt), expected, atol=1e-13)

    def test_joint_cpp_actual_rollout_matches_ocp_with_alpha_actual_not_command(self):
        p = default_parameter_values(load_config(), with_slosh=True)
        for key, value in zip(("two_zeta_omega_n", "omega_n_sq", "kappa_x", "kappa_y"), self.physical):
            p[PIDX_SLOSH[key]] = value
        actuator = array([p[PIDX_SLOSH[k]] for k in
                          ("actuator_tau_v", "actuator_tau_omega", "actuator_gain_v", "actuator_gain_omega")])
        rng = np.random.default_rng(614)
        dt = float(p[PIDX_SLOSH["actuator_dt"]])
        for _ in range(12):
            x = np.zeros(28); x[:6] = rng.uniform(-.2, .2, 6)
            x[3] = .12; x[LINEAR_QUEUE_START] = .18
            x[ANGULAR_QUEUE_START] = -.15
            x[SLOSH_STATE_OFFSET:] = rng.normal(size=4)*.003
            cmd = array([x[LINEAR_QUEUE_START], x[ANGULAR_QUEUE_START]])
            motion = array(np.r_[x[:4], x[5], x[SLOSH_STATE_OFFSET:]])
            actual = np.zeros(9)
            self.assertEqual(self.lib.actual_motion_step(motion, cmd, actuator, dt, actual), 1)
            for alpha_cmd in (-1.2, 1.2):
                expected = np.asarray(self.ocp(x, [.2, alpha_cmd, .2], p)).ravel()
                expected = np.r_[expected[:4], expected[5], expected[SLOSH_STATE_OFFSET:]]
                np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_observers_share_formula_and_consume_angular_acceleration(self):
        rows = array([[.2, -.1, .6, -.4, .02], [.1, .3, -.2, 1.0, .018],
                      [-.3, .2, .5, .7, .025]])
        actual = np.zeros(8)
        self.assertEqual(self.lib.observer_steps(rows, len(rows), actual), 1)
        expected = np.zeros(4)
        for row in rows:
            expected = self.step(expected, row[:4], row[4])
        np.testing.assert_allclose(actual[:4], expected, atol=1e-13)
        np.testing.assert_allclose(actual[4:], expected, atol=1e-13)

    def test_nowcast_uses_angular_acceleration_and_preserves_input(self):
        x = array([.003, -.01, .002, .008]); excitation = array([.1, -.2, .7, -1.1])
        original = x.copy(); actual = np.zeros(4)
        self.assertEqual(self.lib.nowcast_step(x, excitation, .03, actual), 1)
        expected = self.step(x, excitation, .03)
        np.testing.assert_allclose(actual, expected, atol=2e-6, rtol=1e-4)
        np.testing.assert_array_equal(x, original)

    def test_nonfinite_angular_input_is_rejected(self):
        for index in (2, 3):
            excitation = np.zeros(4); excitation[index] = np.nan
            out = np.zeros(4)
            self.assertEqual(self.lib.liquid_step(np.zeros(4), excitation, .02, out), 0)

    def test_versioned_observer_replay_matches_cpp_and_requires_angular_inputs(self):
        params = replay.ModalParameters(*self.physical[:4], liquid_model_version=1)
        anchor = replay.ObserverAnchor(1000000000, 3, 0, replay.ModalState(.001, .01, -.002, .02))
        sample = replay.ObserverInputSample(1030000000, .03, 4, 0, .2, -.1, .6, -.8)
        result = replay.replay_observer_inputs(anchor, [sample], params)
        expected = self.step(anchor.q.as_tuple(), [.2, -.1, .6, -.8], .03)
        np.testing.assert_allclose(result.points[-1].q.as_tuple(), expected, atol=1e-13)
        partial = replay.sample_observer_replay(result.points, 1015000000, params)
        np.testing.assert_allclose(partial.as_tuple(), self.step(anchor.q.as_tuple(), [.2, -.1, .6, -.8], .015), atol=1e-13)
        missing = replay.ObserverInputSample(1030000000, .03, 4, 0, .2, -.1)
        with self.assertRaises(replay.ReplayContractError):
            replay.replay_observer_inputs(anchor, [missing], params)
        with self.assertRaises(replay.ReplayContractError):
            replay.exact_zoh_forced_modal_step(anchor.q, .2, -.1, .03, params)
        with self.assertRaises(replay.ReplayContractError):
            replay.replay_planned_controls(anchor.q, .2, .3, [], params)
        replay.require_legacy_liquid_model(SimpleNamespace())
        with self.assertRaises(replay.ReplayContractError):
            replay.require_legacy_liquid_model(SimpleNamespace(liquid_model_version=1))

    def test_ocp_replay_preserves_recorded_queues_and_initial_state(self):
        p = default_parameter_values(load_config(), with_slosh=True)
        x = np.zeros(28); x[3] = .1; x[5] = .2
        x[LINEAR_QUEUE_START] = .18; x[ANGULAR_QUEUE_START] = -.1
        x[SLOSH_STATE_OFFSET:] = [.003, .01, -.002, -.01]
        u = array([[.1, .4, .2], [-.1, -.5, .1]])
        states = replay_ocp(x, u, np.tile(p, (3, 1)), liquid_model_version=1)
        np.testing.assert_array_equal(states[0], x)
        expected = np.asarray(self.ocp(self.ocp(x, u[0], p), u[1], p)).ravel()
        np.testing.assert_allclose(states[-1], expected, atol=1e-13)
        with self.assertRaises(ValueError):
            replay_ocp(x, u, np.tile(p, (3, 1)), liquid_model_version=0)

    def test_two_second_rotating_propagation_converges_against_adaptive_integrator(self):
        # Independent matrix form with frozen nonzero actual omega and alpha.
        # Modal stiffness here is the real configured narrow container, making
        # the 30 Hz / 4-substep discretization error relevant to deployment.
        d, wn2 = self.physical[:2]; w, alpha = .8, 1.1
        a = array([[0, 1, 0, 0], [w*w-wn2, -d, alpha, 2*w],
                   [0, 0, 0, 1], [-alpha, -2*w, w*w-wn2, -d]])
        force = array([0, -.2, 0, .15])
        initial = array([.003, .01, -.002, .02])
        exact = solve_ivp(lambda t, z: a@z+force, (0, 2), initial,
                          method="DOP853", rtol=1e-12, atol=1e-14).y[:, -1]
        state = initial.copy()
        for _ in range(60):
            state = self.step(state, [.2, -.15, w, alpha], 1/30)
        height_error_m = np.linalg.norm((state-exact)[[0, 2]]) * self.physical[4]
        self.assertLess(height_error_m, 1e-5)  # 0.01 mm over a full 2 s horizon.

    def test_generated_acados_transition_matches_runtime_kernel(self):
        source = PLANNER / "generated/acados/spmpc_slosh/spmpc_slosh_model/spmpc_slosh_dyn_disc_phi_fun.c"
        if not source.exists():
            self.skipTest("generate the full acados slosh solver to verify its emitted C")
        library = self.build / "ocp_transition.so"
        subprocess.run(["gcc", "-O2", "-shared", "-fPIC", str(source), "-lm", "-o", str(library)], check=True)
        lib = ct.CDLL(str(library)); function = lib.spmpc_slosh_dyn_disc_phi_fun
        sizes = [ct.c_int() for _ in range(4)]
        lib.spmpc_slosh_dyn_disc_phi_fun_work(*(ct.byref(n) for n in sizes))
        p = default_parameter_values(load_config(), with_slosh=True)
        rng = np.random.default_rng(55)
        for _ in range(10):
            x = rng.normal(size=28) * .02; u = array([.1, -.3, .2]); out = np.zeros(28)
            inputs = (ct.POINTER(ct.c_double) * sizes[0].value)()
            outputs = (ct.POINTER(ct.c_double) * sizes[1].value)()
            for index, value in enumerate((x, u, p)):
                inputs[index] = value.ctypes.data_as(ct.POINTER(ct.c_double))
            outputs[0] = out.ctypes.data_as(ct.POINTER(ct.c_double))
            iw = (ct.c_int * sizes[2].value)(); work = (ct.c_double * sizes[3].value)()
            self.assertEqual(function(inputs, outputs, iw, work, 0), 0)
            expected = np.asarray(self.ocp(x, u, p)).ravel()
            np.testing.assert_allclose(out, expected, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
