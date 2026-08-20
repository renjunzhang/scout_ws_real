#!/usr/bin/env python3

import importlib.util
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ANALYSIS_DIR = PACKAGE_ROOT / "tools" / "analysis"


def load_module(name):
    path = ANALYSIS_DIR / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


REPLAY = load_module("horizon_liquid_replay")
ALIGNMENT = load_module("analyze_horizon_future_liquid_alignment")


class HorizonFutureAlignmentTest(unittest.TestCase):
    def assert_state_close(self, actual, expected, tolerance=1.0e-12):
        for actual_value, expected_value in zip(actual.as_tuple(), expected):
            self.assertAlmostEqual(
                actual_value, expected_value, delta=tolerance
            )

    def test_cubic_hermite_reproduces_endpoint_values_and_derivatives(self):
        left = REPLAY.ModalState(0.3, -0.7, 1.2, 0.4)
        right = REPLAY.ModalState(-0.8, 0.9, 0.2, -1.1)
        duration = 0.37

        at_left = REPLAY.cubic_hermite_q(left, right, 0.0, duration)
        at_right = REPLAY.cubic_hermite_q(left, right, duration, duration)

        self.assert_state_close(at_left, left.as_tuple())
        self.assert_state_close(at_right, right.as_tuple())

    def test_exact_forced_zoh_matches_undamped_analytic_solution(self):
        omega = 2.0
        parameters = REPLAY.ModalParameters(
            two_zeta_omega_n=0.0,
            omega_n_sq=omega * omega,
            kappa_x=1.5,
            kappa_y=0.75,
        )
        initial = REPLAY.ModalState(0.3, -0.4, -0.2, 0.5)
        ax = 0.8
        ay = -0.6
        duration = 0.37

        actual = REPLAY.exact_zoh_forced_modal_step(
            initial, ax, ay, duration, parameters
        )

        def analytic(position, velocity, acceleration, gain):
            equilibrium = -gain * acceleration / (omega * omega)
            phase = omega * duration
            shifted = position - equilibrium
            return (
                equilibrium
                + shifted * math.cos(phase)
                + velocity * math.sin(phase) / omega,
                -omega * shifted * math.sin(phase)
                + velocity * math.cos(phase),
            )

        eta_x, eta_x_dot = analytic(
            initial.eta_x, initial.eta_x_dot, ax, parameters.kappa_x
        )
        eta_y, eta_y_dot = analytic(
            initial.eta_y, initial.eta_y_dot, ay, parameters.kappa_y
        )
        self.assert_state_close(
            actual, (eta_x, eta_x_dot, eta_y, eta_y_dot)
        )

    def test_q0_anchor_echo_is_skipped_and_never_applied_twice(self):
        parameters = REPLAY.ModalParameters(0.4, 9.0, 1.0, 1.0)
        anchor = REPLAY.ObserverAnchor(
            state_stamp_ns=1_000_000_000,
            update_count=7,
            reset_epoch=3,
            q=REPLAY.ModalState(0.1, -0.2, 0.3, -0.4),
        )
        echo = REPLAY.ObserverInputSample(
            state_stamp_ns=anchor.state_stamp_ns,
            sample_dt_sec=0.01,
            update_count=anchor.update_count,
            reset_epoch=anchor.reset_epoch,
            ax=100.0,
            ay=-100.0,
        )
        following = REPLAY.ObserverInputSample(
            state_stamp_ns=1_010_000_000,
            sample_dt_sec=0.01,
            update_count=8,
            reset_epoch=3,
            ax=0.6,
            ay=-0.2,
        )

        with_echo = REPLAY.replay_observer_inputs(
            anchor, [echo, following], parameters
        )
        without_echo = REPLAY.replay_observer_inputs(
            anchor, [following], parameters
        )

        self.assertTrue(with_echo.skipped_anchor_echo)
        self.assertEqual(len(with_echo.points), 2)
        self.assert_state_close(
            with_echo.points[-1].q, without_echo.points[-1].q.as_tuple()
        )
        with self.assertRaises(REPLAY.ReplayContractError):
            REPLAY.replay_observer_inputs(
                anchor, [echo, echo, following], parameters
            )

    def test_observer_count_epoch_and_dt_mismatches_fail_closed(self):
        parameters = REPLAY.ModalParameters(0.4, 9.0)
        anchor = REPLAY.ObserverAnchor(
            state_stamp_ns=1_000_000_000,
            update_count=7,
            reset_epoch=3,
            q=REPLAY.ModalState.zero(),
        )

        invalid_samples = {
            "update_count_gap": REPLAY.ObserverInputSample(
                1_010_000_000, 0.01, 9, 3, 0.0, 0.0
            ),
            "epoch_change": REPLAY.ObserverInputSample(
                1_010_000_000, 0.01, 1, 4, 0.0, 0.0
            ),
            "stamp_dt_mismatch": REPLAY.ObserverInputSample(
                1_010_000_000, 0.02, 8, 3, 0.0, 0.0
            ),
        }
        for case, sample in invalid_samples.items():
            with self.subTest(case=case):
                with self.assertRaises(REPLAY.ReplayContractError):
                    REPLAY.replay_observer_inputs(
                        anchor, [sample], parameters
                    )

    def test_planned_rk4_matches_longitudinal_exact_zoh(self):
        parameters = REPLAY.ModalParameters(0.6, 6.25, 0.9, 1.1)
        initial = REPLAY.ModalState(0.12, -0.08, 0.0, 0.0)
        controls = (
            REPLAY.PlannedControl(a=0.8, alpha=0.0, duration_sec=0.07),
            REPLAY.PlannedControl(a=-0.3, alpha=0.0, duration_sec=0.05),
        )
        planned = REPLAY.replay_planned_controls(
            initial,
            initial_v=0.4,
            initial_omega=0.0,
            controls=controls,
            parameters=parameters,
            max_substep_sec=5.0e-4,
        )

        expected = initial
        elapsed = 0.0
        for control in controls:
            expected = REPLAY.exact_zoh_forced_modal_step(
                expected,
                ax=control.a,
                ay=0.0,
                dt_sec=control.duration_sec,
                parameters=parameters,
            )
            elapsed += control.duration_sec
            actual = REPLAY.sample_planned_replay(planned, elapsed).q
            self.assertAlmostEqual(actual.eta_x, expected.eta_x, delta=1.0e-10)
            self.assertAlmostEqual(
                actual.eta_x_dot, expected.eta_x_dot, delta=1.0e-10
            )

    def test_availability_classes_and_floor_lead_bins(self):
        epsilon = 1.0e-6
        dt = 1.0 / 30.0

        self.assertEqual(ALIGNMENT.availability_class(-0.001, epsilon), "hindcast")
        self.assertEqual(ALIGNMENT.availability_class(0.032, epsilon), "causal")
        self.assertEqual(ALIGNMENT.availability_class(0.0, epsilon), "boundary")
        self.assertEqual(int(math.floor(-0.001 / dt)), -1)
        self.assertEqual(int(math.floor(0.032 / dt)), 0)
        self.assertEqual(int(math.floor(0.034 / dt)), 1)

    def test_metric_store_reports_skill_against_zero_input(self):
        store = ALIGNMENT.MetricStore()
        targets = (1.0, 2.0, 3.0)
        for target in targets:
            store.add(
                "run_01",
                "native_rgb",
                "primary_delta0",
                3,
                "causal",
                "zero_input",
                0.0,
                target,
            )
            store.add(
                "run_01",
                "native_rgb",
                "primary_delta0",
                3,
                "causal",
                "solver",
                0.5 * target,
                target,
            )

        row = next(
            item
            for item in store.rows()
            if item["run_id"] == "run_01"
            and item["target_type"] == "native_rgb"
            and item["lag_mode"] == "primary_delta0"
            and item["bin_j"] == 3
            and item["scope"] == "causal"
            and item["model"] == "solver"
        )
        self.assertAlmostEqual(row["skill_vs_zero_input_mse"], 0.75)

    def test_validate_horizon_rejects_a_state_shape_mismatch(self):
        steps = 60
        dt = 1.0 / 30.0
        attributes = {
            "valid": True,
            "slosh_enabled": True,
            "variant": ALIGNMENT.EXPECTED_VARIANT,
            "solver_status": ALIGNMENT.EXPECTED_HORIZON_STATUS,
            "backend": "continuous_mpcc_acados",
            "control_semantics": "alpha",
            "horizon_steps": steps,
            "dt": dt,
            "t": [index * dt for index in range(steps + 1)],
        }
        for field in (
            "x",
            "y",
            "yaw",
            "v",
            "omega",
            "s",
            "eta_x",
            "eta_x_dot",
            "eta_y",
            "eta_y_dot",
            "h_modal",
        ):
            attributes[field] = [0.0] * (steps + 1)
        for field in ("a", "alpha_or_omega", "v_s"):
            attributes[field] = [0.0] * steps

        valid = SimpleNamespace(**attributes)
        ALIGNMENT.validate_horizon(valid)

        invalid = SimpleNamespace(**vars(valid))
        invalid.eta_y = invalid.eta_y[:-1]
        with self.assertRaisesRegex(RuntimeError, "incomplete horizon state arrays"):
            ALIGNMENT.validate_horizon(invalid)


if __name__ == "__main__":
    unittest.main()
