#!/usr/bin/env python3

import pathlib
import sys
import tempfile
import unittest
from unittest import mock

import casadi as ca
import numpy as np


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[2]
TOOLS = PACKAGE_ROOT / "tools" / "codegen" / "acados"
sys.path.insert(0, str(TOOLS))

from generate_delay_augmented_phase_acados import (  # noqa: E402
    CAP_EXECUTION_COMPATIBILITY_SET,
    CAP_PUBLISHED_RESIDUAL_BOUNDS,
    CAP_TERMINAL_EMPIRICAL_GATE,
    FORMAL_REQUIRED_CAPABILITIES,
    WP3C_CAPABILITIES,
    build_symbolic_spec,
    emit_solver_manifest,
    load_solver_spec,
    main,
    state_scaling_vectors,
)


class DelayAugmentedPhaseAcadosCodegenTest(unittest.TestCase):
    def test_discrete_ocp_shape_and_hard_command_envelope(self):
        spec = load_solver_spec()
        symbolic = build_symbolic_spec(spec)
        layout = spec["layout"]
        contract = spec["contract"]
        scale_x, scale_u = state_scaling_vectors(spec)

        self.assertEqual((15, 1), symbolic["x_next"].shape)
        self.assertEqual((2, 1), symbolic["published"].shape)
        self.assertEqual(
            (6 + 2 * len(layout["execution_indices"]), 1),
            symbolic["stage_constraints"].shape)
        self.assertEqual(7, contract["horizon_steps"])
        self.assertTrue(spec["use_linear_model"])
        self.assertFalse(spec["use_parabola_term"])
        self.assertEqual(0, len(symbolic["idxbx"]))
        self.assertEqual([0, 1, 2], symbolic["idxbu"].tolist())
        self.assertEqual([], symbolic["idxbx"].tolist())
        self.assertEqual(
            -spec["terminal_empirical_numerical_inner_margin"],
            symbolic["terminal_upper"][0])
        self.assertGreater(
            spec["terminal_empirical_numerical_inner_margin"],
            spec["solver_config"]["tol_ineq"])

        published = ca.Function(
            "candidate_published_command_check",
            [symbolic["x"], symbolic["q"]],
            [symbolic["published"]],
        )
        state_physical = np.zeros(layout["state_width"])
        state_physical[layout["linear_buffer_offset"]
                       + layout["linear_buffer_count"] - 1] = 0.2
        state_physical[layout["angular_buffer_offset"]
                       + layout["angular_buffer_count"] - 1] = -0.1
        control_physical = np.array([0.3, -0.6, 0.2])
        state = state_physical / scale_x
        control = control_physical / scale_u
        actual = np.asarray(published(state, control)).reshape(-1)
        self.assertAlmostEqual(
            0.2 + contract["dt"] * control_physical[0], actual[0])
        self.assertAlmostEqual(
            -0.1 + contract["dt"] * control_physical[1], actual[1])

        image = np.zeros(spec["parameters"]["parameter_width"])
        names = spec["parameters"]["names"]
        image[names.index("nom_u_pub_v")] = actual[0]
        image[names.index("nom_u_pub_omega")] = actual[1]
        image[names.index("max_residual_v")] = 0.01
        image[names.index("max_residual_omega")] = 0.02
        image[:layout["state_width"]] = state_physical
        image[spec["parameters"]["execution_bound_offset"]:] = 0.5
        constrained = ca.Function(
            "candidate_published_residual_check",
            [symbolic["x"], symbolic["q"], symbolic["p"]],
            [symbolic["stage_constraints"]],
        )
        values = np.asarray(constrained(state, control, image)).reshape(-1)
        np.testing.assert_allclose(
            values[2:6], [-0.01, -0.01, -0.02, -0.02], atol=1e-12)
        np.testing.assert_allclose(values[6:], -0.5, atol=1e-12)

        # Reproduce the stopped-tail failure mode: a command that was valid
        # when published has shifted to the final angular pending slot while
        # the phase-indexed radius has tightened.  The path constraint must
        # expose the exact named component before the command can strand the
        # selector.
        shifted = np.zeros(layout["state_width"])
        shifted_index = (layout["angular_buffer_offset"]
                         + layout["angular_buffer_count"] - 1)
        shifted[shifted_index] = 0.00105380837
        shifted_image = np.zeros(spec["parameters"]["parameter_width"])
        shifted_image[names.index("max_residual_v")] = 1.0
        shifted_image[names.index("max_residual_omega")] = 1.0
        shifted_image[spec["parameters"]["execution_bound_offset"]:] = 0.09
        shifted_bound = layout["execution_indices"].index(shifted_index)
        shifted_image[
            spec["parameters"]["execution_bound_offset"] + shifted_bound
        ] = 0.000719077435
        shifted_values = np.asarray(constrained(
            shifted / scale_x, np.zeros(layout["control_width"]),
            shifted_image)).reshape(-1)
        upper = 6 + 2 * shifted_bound
        self.assertGreater(shifted_values[upper], 0.0)
        self.assertLess(shifted_values[upper + 1], 0.0)

    def test_capability_mask_requires_terminal_and_execution_contracts(self):
        self.assertEqual(WP3C_CAPABILITIES,
                         WP3C_CAPABILITIES & FORMAL_REQUIRED_CAPABILITIES)
        self.assertEqual(0, WP3C_CAPABILITIES & CAP_TERMINAL_EMPIRICAL_GATE)
        self.assertEqual(
            0, WP3C_CAPABILITIES & CAP_EXECUTION_COMPATIBILITY_SET)
        self.assertEqual(
            0, WP3C_CAPABILITIES & CAP_PUBLISHED_RESIDUAL_BOUNDS)
        self.assertNotEqual(WP3C_CAPABILITIES, FORMAL_REQUIRED_CAPABILITIES)
        self.assertEqual(
            FORMAL_REQUIRED_CAPABILITIES,
            WP3C_CAPABILITIES
            | CAP_TERMINAL_EMPIRICAL_GATE
            | CAP_EXECUTION_COMPATIBILITY_SET
            | CAP_PUBLISHED_RESIDUAL_BOUNDS,
        )

    def test_nominal_relative_cost_and_terminal_parameter_order_are_exact(self):
        spec = load_solver_spec()
        symbolic = build_symbolic_spec(spec)
        layout = spec["layout"]
        parameters = spec["parameters"]
        names = parameters["names"]
        scale_x, scale_u = state_scaling_vectors(spec)
        self.assertEqual(50, parameters["parameter_width"])
        self.assertEqual(15, parameters["nominal_control_offset"])
        self.assertEqual(18, parameters["nominal_publish_offset"])
        self.assertEqual(20, parameters["residual_bound_offset"])
        self.assertEqual(22, parameters["weight_offset"])
        self.assertEqual(34, parameters["gate_radius_offset"])
        self.assertEqual(43, parameters["execution_bound_offset"])

        stage_cost = ca.Function(
            "delay_augmented_stage_cost_test",
            [symbolic["x"], symbolic["q"], symbolic["p"]],
            [symbolic["stage_cost"]],
        )
        terminal_cost = ca.Function(
            "delay_augmented_terminal_cost_test",
            [symbolic["x"], symbolic["q"], symbolic["p"]],
            [symbolic["terminal_cost"]],
        )
        terminal_constraints = ca.Function(
            "delay_augmented_terminal_constraints_test",
            [symbolic["x"], symbolic["p"]],
            [symbolic["terminal_constraints"]],
        )

        nominal = np.linspace(-0.2, 0.2, layout["state_width"])
        nominal[4] = 1.0
        nominal_q = np.array([0.1, -0.2, 0.3])
        image = np.zeros(parameters["parameter_width"])
        image[:layout["state_width"]] = nominal
        image[parameters["nominal_control_offset"]:
              parameters["nominal_publish_offset"]] = nominal_q
        image[parameters["nominal_publish_offset"]:
              parameters["residual_bound_offset"]] = (0.2, -0.1)
        image[parameters["residual_bound_offset"]:
              parameters["weight_offset"]] = (0.08, 0.20)
        for name in (
            "w_position", "w_yaw", "w_progress", "w_v", "w_omega",
            "w_slosh_eta", "w_slosh_eta_dot", "w_linear_pending",
            "w_angular_pending", "w_a", "w_alpha", "w_v_s",
        ):
            image[names.index(name)] = 2.0
        image[parameters["gate_radius_offset"]:
              parameters["execution_bound_offset"]] = 1.0
        image[parameters["execution_bound_offset"]:] = 1.0

        self.assertAlmostEqual(
            0.0, float(stage_cost(
                nominal / scale_x, nominal_q / scale_u, image)))
        self.assertAlmostEqual(
            0.0, float(terminal_cost(
                nominal / scale_x, (nominal_q + 1.0) / scale_u, image)))
        np.testing.assert_allclose(
            np.asarray(terminal_constraints(
                nominal / scale_x, image)).reshape(-1),
            -np.ones(1 + 2 * len(layout["execution_indices"])),
        )

        actual = nominal.copy()
        actual[0] += 0.05
        actual[1] -= 0.04
        actual[2] += 0.03
        actual[4] += 0.02
        actual[layout["linear_buffer_offset"]] += 0.01
        control = nominal_q + np.array([0.02, -0.03, 0.04])
        scales = spec["cost_scales"]
        manual = 2.0 * (
            (0.05 ** 2 + 0.04 ** 2) / scales["position"] ** 2
            + 0.03 ** 2 / scales["yaw"] ** 2
            + 0.02 ** 2 / scales["progress"] ** 2
            + 0.01 ** 2 / scales["v"] ** 2
            + 0.02 ** 2 / scales["a"] ** 2
            + 0.03 ** 2 / scales["alpha"] ** 2
            + 0.04 ** 2 / scales["v_s"] ** 2
        )
        self.assertAlmostEqual(
            manual, float(stage_cost(
                actual / scale_x, control / scale_u, image)), places=11)

    def test_committed_solver_manifest_is_deterministic(self):
        committed = (
            PACKAGE_ROOT / "generated" / "acados" /
            "spmpc_delay_augmented_phase_solver_manifest.h")
        with tempfile.TemporaryDirectory() as temporary:
            generated = emit_solver_manifest(load_solver_spec(), temporary)
            self.assertEqual(committed.read_bytes(), generated.read_bytes())

    def test_check_mode_does_not_create_or_rewrite_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = pathlib.Path(temporary) / "must_not_be_created"
            argv = ["generate_delay_augmented_phase_acados.py",
                    "--check", "--output-dir", str(output)]
            with mock.patch.object(sys, "argv", argv):
                self.assertEqual(0, main())
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
