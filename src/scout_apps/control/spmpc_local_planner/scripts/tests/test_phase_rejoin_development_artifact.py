#!/usr/bin/env python3

import dataclasses
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "prepare_phase_rejoin_development_artifact.py"
)
SPEC = importlib.util.spec_from_file_location(
    "prepare_phase_rejoin_development_artifact", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Stamp:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_nsec(self):
        return self.nanoseconds


def parameters(cycle_ids):
    return {
        cycle_id: MODULE.DevelopmentParameters(
            cycle_id=cycle_id,
            kappa_v=0.02 * cycle_id,
            kappa_omega=-0.01 * cycle_id,
            radii=(0.5,) * 9,
        )
        for cycle_id in cycle_ids
    }


def pair(cycle_id, epoch_ns, dt=0.1):
    offset = cycle_id - 10
    stamps = {
        "cycle_start_stamp": Stamp(epoch_ns - 10_000_000),
        "raw_robot_state_stamp": Stamp(epoch_ns - 20_000_000),
        "raw_liquid_state_stamp": Stamp(epoch_ns - 20_000_000),
        "robot_state_stamp": Stamp(epoch_ns),
        "liquid_state_stamp": Stamp(epoch_ns),
        "solver_input_epoch": Stamp(epoch_ns),
        "solve_start_stamp": Stamp(epoch_ns - 8_000_000),
        "solve_end_stamp": Stamp(epoch_ns - 5_000_000),
        "horizon_available_stamp": Stamp(epoch_ns - 4_000_000),
    }
    header = SimpleNamespace(frame_id="map")
    joined_values = {
        "raw_state_skew_sec": 0.0,
        "aligned_state_skew_sec": 0.0,
        "state_alignment_required": True,
        "state_time_aligned": True,
        "robot_state_interpolated": False,
        "robot_state_extrapolated": True,
        "state_alignment_status": "ALIGNED",
        "solver_status": "OK",
    }
    horizon = SimpleNamespace(
        header=header,
        schema_version=2,
        cycle_id=cycle_id,
        valid=True,
        slosh_enabled=True,
        control_semantics="alpha",
        variant="B_development_proxy",
        dt=dt,
        horizon_steps=2,
        t=[0.0, dt, 2.0 * dt],
        x=[0.1 * offset, 0.1 * offset + 0.01, 0.1 * offset + 0.02],
        y=[0.0, 0.0, 0.0],
        yaw=[0.0, 0.0, 0.0],
        v=[0.1, 0.1, 0.1],
        omega=[0.0, 0.0, 0.0],
        s=[0.1 * offset, 0.1 * offset + 0.01, 0.1 * offset + 0.02],
        eta_x=[0.01, 0.01, 0.01],
        eta_x_dot=[0.0, 0.0, 0.0],
        eta_y=[0.0, 0.0, 0.0],
        eta_y_dot=[0.0, 0.0, 0.0],
        h_modal=[0.01, 0.01, 0.01],
        a=[0.2, 0.1],
        alpha_or_omega=[0.03, 0.02],
        v_s=[0.1, 0.1],
        **stamps,
        **joined_values,
    )
    audit = SimpleNamespace(
        header=header,
        schema_version=1,
        cycle_id=cycle_id,
        variant=horizon.variant,
        solve_attempted=True,
        solve_success=True,
        command_accepted=True,
        publish_cmd_vel=True,
        command_was_published=True,
        command_contract_violation=False,
        terminal_controller_intervened=False,
        safety_gate_intervened=False,
        linear_limited=False,
        angular_rate_limited=False,
        angular_accel_limited=False,
        solver_u0_a=horizon.a[0],
        solver_u0_alpha=horizon.alpha_or_omega[0],
        published_cmd_v=0.12,
        published_cmd_omega=0.01,
        **stamps,
        **joined_values,
    )
    return horizon, audit


class DevelopmentArtifactTest(unittest.TestCase):
    def setUp(self):
        self.horizons = []
        self.audits = []
        for cycle_id in (10, 11, 12):
            horizon, audit = pair(
                cycle_id,
                1_000_000_000 + (cycle_id - 10) * 100_000_000,
            )
            self.horizons.append(horizon)
            self.audits.append(audit)
        self.options = MODULE.PreparationOptions(
            contract_id="development_contract",
            expected_dt=0.1,
            path_length=2.0,
            expected_frame_id="map",
        )

    def prepare(self, horizons=None, audits=None, params=None):
        return MODULE.prepare_artifact(
            self.horizons if horizons is None else horizons,
            self.audits if audits is None else audits,
            parameters((10, 11, 12)) if params is None else params,
            self.options,
            bag_sha256="a" * 64,
            parameter_sha256="b" * 64,
        )

    def test_exports_fixed_development_markers_and_loader_schema(self):
        prepared = self.prepare()
        self.assertEqual(prepared.metadata["evidence_level"], "development_only")
        self.assertEqual(prepared.metadata["source"], "development_proxy_replay")
        self.assertEqual(prepared.metadata["offline_slosh_ocp"], "false")
        self.assertEqual(prepared.metadata["hardware_formal_release"], "false")
        self.assertEqual(len(prepared.rows), 3)
        self.assertEqual(prepared.rows[0][0], 0)
        self.assertAlmostEqual(prepared.rows[1][1], 0.1)
        self.assertAlmostEqual(prepared.rows[0][15], 0.12)

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "development_only.csv"
            MODULE.write_artifact(output, prepared)
            metadata = MODULE.validate_artifact_csv(output)
            self.assertEqual(metadata["schema"], "phase_rejoin_empirical_v1")
            lines = output.read_text(encoding="utf-8").splitlines()
            header = next(line for line in lines if not line.startswith("#"))
            self.assertEqual(tuple(header.split(",")), MODULE.ARTIFACT_HEADER)

    def test_rejects_duplicate_and_missing_cycle_pairs(self):
        with self.assertRaisesRegex(MODULE.ArtifactPreparationError, "duplicate"):
            self.prepare(horizons=self.horizons + [self.horizons[0]])
        with self.assertRaisesRegex(MODULE.ArtifactPreparationError, "pairing mismatch"):
            self.prepare(audits=self.audits[:-1])

    def test_rejects_nonconsecutive_cycles_and_missing_explicit_parameters(self):
        with self.assertRaisesRegex(MODULE.ArtifactPreparationError, "missing cycle"):
            self.prepare(
                horizons=[self.horizons[0], self.horizons[2]],
                audits=[self.audits[0], self.audits[2]],
            )
        with self.assertRaisesRegex(
            MODULE.ArtifactPreparationError, "missing explicit development parameters"
        ):
            self.prepare(params=parameters((10, 11)))

    def test_rejects_join_timestamp_or_first_control_mismatch(self):
        bad_audit = SimpleNamespace(**vars(self.audits[1]))
        bad_audit.solve_end_stamp = Stamp(
            self.audits[1].solve_end_stamp.to_nsec() + 2
        )
        audits = [self.audits[0], bad_audit, self.audits[2]]
        with self.assertRaisesRegex(MODULE.ArtifactPreparationError, "solve_end_stamp"):
            self.prepare(audits=audits)

        bad_control = SimpleNamespace(**vars(self.audits[1]))
        bad_control.solver_u0_a += 0.01
        audits = [self.audits[0], bad_control, self.audits[2]]
        with self.assertRaisesRegex(MODULE.ArtifactPreparationError, "first a"):
            self.prepare(audits=audits)

    def test_rejects_safety_or_limiter_intervention(self):
        for field in (
            "command_contract_violation",
            "terminal_controller_intervened",
            "safety_gate_intervened",
            "linear_limited",
            "angular_rate_limited",
            "angular_accel_limited",
        ):
            with self.subTest(field=field):
                bad = SimpleNamespace(**vars(self.audits[1]))
                setattr(bad, field, True)
                audits = [self.audits[0], bad, self.audits[2]]
                with self.assertRaisesRegex(MODULE.ArtifactPreparationError, field):
                    self.prepare(audits=audits)

    def test_parameter_csv_has_no_implicit_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            valid = Path(directory) / "parameters.csv"
            valid.write_text(
                ",".join(MODULE.PARAMETER_HEADER)
                + "\n10,0,0,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5\n",
                encoding="utf-8",
            )
            loaded = MODULE.load_development_parameters(valid)
            self.assertIn(10, loaded)

            missing_radius = Path(directory) / "missing_radius.csv"
            missing_radius.write_text(
                ",".join(MODULE.PARAMETER_HEADER)
                + "\n10,0,0,,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5\n",
                encoding="utf-8",
            )
            with self.assertRaises(MODULE.ArtifactPreparationError):
                MODULE.load_development_parameters(missing_radius)

    def test_accepts_quantized_period_on_relative_tolerance_boundary(self):
        dt = 0.0333333333
        first_horizon, first_audit = pair(10, 1_000_000_000, dt=dt)
        second_horizon, second_audit = pair(11, 1_040_000_000, dt=dt)
        third_horizon, third_audit = pair(12, 1_060_000_000, dt=dt)
        options = dataclasses.replace(self.options, expected_dt=dt)
        prepared = MODULE.prepare_artifact(
            [first_horizon, second_horizon, third_horizon],
            [first_audit, second_audit, third_audit],
            parameters((10, 11, 12)),
            options,
            bag_sha256="a" * 64,
            parameter_sha256="b" * 64,
        )

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "quantized_period.csv"
            MODULE.write_artifact(output, prepared)
            MODULE.validate_artifact_csv(output)

    def test_rejects_accumulating_quantized_period_drift(self):
        dt = 0.0333333333
        pairs = [
            pair(10 + offset, 1_000_000_000 + offset * 20_000_000, dt=dt)
            for offset in range(4)
        ]
        options = dataclasses.replace(self.options, expected_dt=dt)
        with self.assertRaisesRegex(
            MODULE.ArtifactPreparationError, "cumulative solver epoch drift"
        ):
            MODULE.prepare_artifact(
                [item[0] for item in pairs],
                [item[1] for item in pairs],
                parameters((10, 11, 12, 13)),
                options,
                bag_sha256="a" * 64,
                parameter_sha256="b" * 64,
            )

    def test_validator_rejects_relabeling_as_stronger_evidence(self):
        prepared = self.prepare()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "development_only.csv"
            MODULE.write_artifact(output, prepared)
            text = output.read_text(encoding="utf-8").replace(
                "# evidence_level=development_only",
                "# evidence_level=empirical_held_out",
            )
            output.write_text(text, encoding="utf-8")
            with self.assertRaisesRegex(
                MODULE.ArtifactPreparationError, "evidence_level"
            ):
                MODULE.validate_artifact_csv(output)

    def test_cycle_range_still_requires_one_to_one_inputs(self):
        options = dataclasses.replace(
            self.options, start_cycle_id=11, end_cycle_id=12
        )
        prepared = MODULE.prepare_artifact(
            self.horizons,
            self.audits,
            parameters((11, 12)),
            options,
            bag_sha256="a" * 64,
            parameter_sha256="b" * 64,
        )
        self.assertEqual(prepared.metadata["cycle_id_first"], "11")
        self.assertEqual(prepared.metadata["cycle_id_last"], "12")


if __name__ == "__main__":
    unittest.main()
