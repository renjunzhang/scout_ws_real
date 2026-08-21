#!/usr/bin/env python3

import pathlib
import sys
import tempfile
import unittest

import casadi as ca
import numpy as np


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[2]
TOOLS = PACKAGE_ROOT / "tools" / "codegen" / "acados"
sys.path.insert(0, str(TOOLS))

from generate_delay_augmented_phase_acados import (  # noqa: E402
    CAP_EXECUTION_COMPATIBILITY_SET,
    CAP_TERMINAL_EMPIRICAL_GATE,
    FORMAL_REQUIRED_CAPABILITIES,
    WP3C_CAPABILITIES,
    build_symbolic_spec,
    emit_solver_manifest,
    load_solver_spec,
)


class DelayAugmentedPhaseAcadosCodegenTest(unittest.TestCase):
    def test_discrete_ocp_shape_and_hard_command_envelope(self):
        spec = load_solver_spec()
        symbolic = build_symbolic_spec(spec)
        layout = spec["layout"]
        contract = spec["contract"]

        self.assertEqual((22, 1), symbolic["x_next"].shape)
        self.assertEqual((2, 1), symbolic["published"].shape)
        self.assertEqual(10, contract["horizon_steps"])
        self.assertEqual(14, len(symbolic["idxbx"]))
        self.assertEqual([0, 1, 2], symbolic["idxbu"].tolist())
        self.assertEqual(
            [3, 5] + list(range(layout["linear_buffer_offset"], 22)),
            symbolic["idxbx"].tolist(),
        )

        published = ca.Function(
            "candidate_published_command_check",
            [symbolic["x"], symbolic["q"]],
            [symbolic["published"]],
        )
        state = np.zeros(layout["state_width"])
        state[layout["linear_buffer_offset"]
              + layout["linear_buffer_count"] - 1] = 0.2
        state[layout["angular_buffer_offset"]
              + layout["angular_buffer_count"] - 1] = -0.1
        control = np.array([0.3, -0.6, 0.2])
        actual = np.asarray(published(state, control)).reshape(-1)
        self.assertAlmostEqual(
            0.2 + contract["dt"] * control[0], actual[0])
        self.assertAlmostEqual(
            -0.1 + contract["dt"] * control[1], actual[1])

    def test_capability_mask_blocks_formal_admission(self):
        self.assertEqual(WP3C_CAPABILITIES,
                         WP3C_CAPABILITIES & FORMAL_REQUIRED_CAPABILITIES)
        self.assertEqual(0, WP3C_CAPABILITIES & CAP_TERMINAL_EMPIRICAL_GATE)
        self.assertEqual(
            0, WP3C_CAPABILITIES & CAP_EXECUTION_COMPATIBILITY_SET)
        self.assertNotEqual(WP3C_CAPABILITIES, FORMAL_REQUIRED_CAPABILITIES)

    def test_committed_solver_manifest_is_deterministic(self):
        committed = (
            PACKAGE_ROOT / "generated" / "acados" /
            "spmpc_delay_augmented_phase_solver_manifest.h")
        with tempfile.TemporaryDirectory() as temporary:
            generated = emit_solver_manifest(load_solver_spec(), temporary)
            self.assertEqual(committed.read_bytes(), generated.read_bytes())


if __name__ == "__main__":
    unittest.main()
