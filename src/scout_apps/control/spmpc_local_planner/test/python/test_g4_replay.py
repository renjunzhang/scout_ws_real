#!/usr/bin/env python3

import importlib.util
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[2]
    / "tools/analysis/g4_replay_from_g3.py"
)
SPEC = importlib.util.spec_from_file_location("g4_replay_from_g3", MODULE_PATH)
G4 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = G4
SPEC.loader.exec_module(G4)


class G4ReplayTest(unittest.TestCase):
    def test_cpp_bridge_rejects_stale_generated_dimensions(self):
        try:
            executable = G4.replay_executable_path()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        snapshot = SimpleNamespace(
            horizon_steps=60,
            state_width=10,
            control_width=3,
            parameter_width=32,
            dt=1.0 / 30.0,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            robot_v=0.0,
            s0=0.0,
            robot_omega=0.0,
            eta_x=0.0,
            eta_x_dot=0.0,
            eta_y=0.0,
            eta_y_dot=0.0,
            a_min=-0.6,
            a_max=0.6,
            alpha_or_omega_min=-1.2,
            alpha_or_omega_max=1.2,
            v_s_min=0.0,
            v_s_max=0.8,
            v_min=0.0,
            v_max=0.8,
            omega_min=-1.2,
            omega_max=1.2,
            stage_parameters=[],
            initial_guess_states=[],
            initial_guess_controls=[],
        )
        pair = G4.SnapshotPair(0, snapshot, None)
        with tempfile.TemporaryDirectory(prefix="spmpc_g4_bridge_test_") as directory:
            request = Path(directory) / "request.txt"
            result = Path(directory) / "result.txt"
            G4.write_replay_request(
                request,
                [pair],
                {"longitudinal": pair},
                {"phases_rad": []},
            )
            completed = subprocess.run(
                [str(executable), "--input", str(request), "--output", str(result)],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            replayed = G4.read_replay_result(result)
        self.assertFalse(replayed["success"])
        self.assertEqual(replayed["detail"], "GENERATED_DIMENSION_MISMATCH")

    def test_continuous_projection(self):
        xy = np.asarray([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
        cumulative, lengths, _ = G4.path_geometry(xy)
        progress, distance, index = G4.project_point_to_polyline(
            np.asarray([1.1, 0.4]), xy, cumulative, lengths
        )
        self.assertAlmostEqual(progress, 1.4)
        self.assertAlmostEqual(distance, 0.1)
        self.assertEqual(index, 1)

    def test_first_crossing_interpolates(self):
        stamp = np.asarray([10.0, 11.0, 12.0])
        progress = np.asarray([0.0, 0.4, 1.0])
        self.assertAlmostEqual(G4.first_crossing_time(stamp, progress, 0.7), 11.5)

    def test_four_phases_have_equal_energy(self):
        names = ["unused"] * 32
        names[24] = "two_zeta_omega_n"
        names[25] = "omega_n_sq"
        names[27] = "eta_ref"
        params = np.zeros((61, 32))
        params[:, 25] = 31.246 ** 2
        params[:, 27] = 0.00275
        snapshot = SimpleNamespace(
            parameter_width=32,
            horizon_steps=60,
            parameter_names=names,
            stage_parameters=params.reshape(-1).tolist(),
        )
        energies = []
        for phase in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
            eta_x, eta_x_dot, eta_y, eta_y_dot = G4.modal_override_for_phase(
                snapshot, "longitudinal", phase, 0.5, 0.005
            )
            energies.append(
                31.246 ** 2 * (eta_x ** 2 + eta_y ** 2)
                + eta_x_dot ** 2
                + eta_y_dot ** 2
            )
        self.assertLess(max(energies) - min(energies), 1e-12)

    def test_checkpoint_selection_is_geometry_excitation_only(self):
        def pair(index, sigma, omega, accel, v=0.2):
            snapshot = SimpleNamespace(
                reference_length=10.0,
                s0=10.0 * sigma,
                robot_omega=omega,
                robot_v=v,
            )
            horizon = SimpleNamespace(solver_status="B_slosh_ACADOS_OK", a=[accel])
            return G4.SnapshotPair(index, snapshot, horizon)

        pairs = [
            pair(0, 0.2, 0.02, 0.1),
            pair(1, 0.3, 0.03, -0.4),
            pair(2, 0.5, 0.5, 0.2),
            pair(3, 0.6, -0.8, 0.1),
        ]
        selected = G4.select_checkpoints(
            pairs,
            {
                "minimum_sigma": 0.1,
                "maximum_sigma": 0.9,
                "longitudinal_max_abs_omega_rad_s": 0.15,
                "lateral_min_abs_omega_rad_s": 0.15,
                "minimum_contiguous_pairs": 1,
                "maximum_pair_gap_sec": 1.0,
            },
        )
        self.assertEqual(selected["longitudinal"].index, 1)
        self.assertEqual(selected["lateral"].index, 3)


if __name__ == "__main__":
    unittest.main()
