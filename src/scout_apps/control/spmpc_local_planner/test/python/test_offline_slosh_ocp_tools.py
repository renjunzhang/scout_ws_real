#!/usr/bin/env python3

import contextlib
import csv
import importlib.util
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[3]
GENERATOR_PATH = (
    PACKAGE_ROOT
    / "tools"
    / "simulation"
    / "generate_offline_slosh_ocp_plan.py"
)
FITTER_PATH = (
    PACKAGE_ROOT / "tools" / "simulation" / "fit_phase_rejoin_recovery.py"
)
BUILDER_PATH = (
    WORKSPACE_ROOT
    / "devel"
    / "lib"
    / "spmpc_local_planner"
    / "spmpc_build_formal_phase_rejoin_nominal"
)
VALIDATOR_PATH = (
    WORKSPACE_ROOT
    / "devel"
    / "lib"
    / "spmpc_local_planner"
    / "spmpc_phase_rejoin_artifact_tool"
)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


GENERATOR = load_module("generate_offline_slosh_ocp_plan_test", GENERATOR_PATH)
FITTER = load_module("fit_phase_rejoin_recovery_ocp_test", FITTER_PATH)


class OfflineSloshOcpToolsTest(unittest.TestCase):

    @staticmethod
    def write_path(path):
        path.write_text(
            json.dumps(
                {
                    "frame_id": "map",
                    "poses": [
                        {
                            "x": 0.0,
                            "y": 0.0,
                            "qx": 0.0,
                            "qy": 0.0,
                            "qz": 0.0,
                            "qw": 1.0,
                        },
                        {
                            "x": 0.5,
                            "y": 0.0,
                            "qx": 0.0,
                            "qy": 0.0,
                            "qz": 0.0,
                            "qw": 1.0,
                        },
                    ],
                }
            ),
            encoding="utf-8",
        )

    @staticmethod
    def run_quiet(function, arguments):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            return_code = function(arguments)
        return return_code, stdout.getvalue(), stderr.getvalue()

    @staticmethod
    def make_recovery_dataset(path, phase_count):
        rows = []

        def add(split, label, count, recovered, error, seed_base):
            for rollout_index in range(count):
                record = {
                    "split": split,
                    "rollout_id": "{}-{}-{}".format(
                        split, label, rollout_index
                    ),
                    "seed": str(seed_base + rollout_index),
                    "recovered": "1" if recovered else "0",
                }
                record.update(
                    {
                        name: format(error, ".17g")
                        for name in FITTER.ERROR_COLUMNS
                    }
                )
                for phase_index in range(phase_count):
                    phase_record = dict(record)
                    phase_record["phase_index"] = str(phase_index)
                    rows.append(phase_record)

        add("fit", "recovered", 1, True, 0.10, 1000)
        add("tune", "recovered", 2, True, 0.04, 2000)
        add("tune", "unrecovered", 20, False, 0.08, 2100)
        add("held_out", "recovered", 2, True, 0.04, 3000)
        add("held_out", "unrecovered", 20, False, 0.08, 3100)
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FITTER.INPUT_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)

    def test_quaternion_is_required_and_normalized_without_overflow(self):
        missing = {"x": 0.0, "y": 0.0}
        with self.assertRaisesRegex(GENERATOR.PlanError, "recorded quaternion"):
            GENERATOR.yaw_from_quaternion(missing)
        huge = {
            "qx": 0.0,
            "qy": 0.0,
            "qz": 1.0e308,
            "qw": 1.0e308,
        }
        self.assertAlmostEqual(
            GENERATOR.yaw_from_quaternion(huge), math.pi / 2.0
        )

    def test_rollout_aligns_goal_yaw_before_zero_hold(self):
        class PathStub:
            length = 1.0
            x = np.asarray([0.0, 1.0])
            y = np.asarray([0.0, 0.0])
            yaw_unwrapped = np.asarray([0.0, math.pi / 2.0])

            def sample(self, progress):
                bounded = max(0.0, min(1.0, float(progress)))
                yaw = math.pi / 2.0 if bounded >= 1.0 - 1.0e-8 else 0.0
                return bounded, 0.0, yaw, 0.0

            def phase_distance(self, x, y, progress):
                bounded = max(0.0, min(1.0, float(progress)))
                return math.hypot(x - bounded, y)

        layout = {
            "state_width": 22,
            "linear_buffer_offset": 10,
            "linear_buffer_count": 5,
            "angular_buffer_offset": 15,
            "angular_buffer_count": 7,
        }
        contract = {
            "dt": 0.1,
            "slosh": {"container_radius": 0.1, "liquid_height": 0.1},
        }

        def transition(state, control):
            next_state = state.copy()
            published_v = state[14] + control[0] * 0.1
            published_omega = state[21] + control[1] * 0.1
            next_state[3] = published_v
            next_state[5] = published_omega
            next_state[0] += published_v * math.cos(next_state[2]) * 0.1
            next_state[1] += published_v * math.sin(next_state[2]) * 0.1
            next_state[2] = GENERATOR.wrap(
                next_state[2] + published_omega * 0.1
            )
            next_state[4] += control[2] * 0.1
            next_state[10:15] = np.r_[state[11:15], published_v]
            next_state[15:22] = np.r_[state[16:22], published_omega]
            return (next_state,)

        result = GENERATOR.rollout(
            PathStub(), contract, transition, layout, [0.9, 0.9], 2.0, 20.0, 0.1
        )
        self.assertTrue(result["valid"], result["reason"])
        self.assertLessEqual(result["terminal_yaw_error_rad"], 0.1)
        tail = result["rows"][-result["zero_hold_steps"] :]
        self.assertTrue(
            all(row[2] == 0.0 and row[3] == 0.0 and row[4] == 0.0 for row in tail)
        )

    def test_exclusive_output_and_alias_guards(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "input.json"
            output_path = root / "output.bin"
            input_path.write_text("input\n", encoding="utf-8")
            GENERATOR.write_exclusive(output_path, b"first\n")
            with self.assertRaisesRegex(GENERATOR.PlanError, "already exists"):
                GENERATOR.write_exclusive(output_path, b"second\n")
            self.assertEqual(output_path.read_bytes(), b"first\n")
            self.assertFalse(any(".tmp." in path.name for path in root.iterdir()))
            with self.assertRaisesRegex(GENERATOR.PlanError, "must not alias"):
                GENERATOR.validate_path_separation(
                    input_path, (input_path, output_path)
                )

    @unittest.skipUnless(
        BUILDER_PATH.is_file() and VALIDATOR_PATH.is_file(),
        "build the formal nominal builder and artifact validator",
    )
    def test_real_generator_fitter_builder_and_validator_pipeline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "path.json"
            plan = root / "offline_plan.csv"
            plan_report = root / "offline_plan_report.json"
            self.write_path(path)
            generator_code, _, generator_stderr = self.run_quiet(
                GENERATOR.main,
                [
                    "--path-json",
                    str(path),
                    "--plan-output",
                    str(plan),
                    "--report-output",
                    str(plan_report),
                    "--no-optimize",
                    "--maximum-motion-sec",
                    "20",
                ],
            )
            self.assertEqual(generator_code, 0, generator_stderr)
            phase_count = json.loads(
                plan_report.read_text(encoding="utf-8")
            )["plan"]["rows"]

            dataset = root / "recovery.csv"
            evidence = root / "recovery_evidence"
            self.make_recovery_dataset(dataset, phase_count)
            fitter_code, _, fitter_stderr = self.run_quiet(
                FITTER.main,
                [
                    "fit",
                    "--input",
                    str(dataset),
                    "--out-dir",
                    str(evidence),
                    "--shrinkage-grid",
                    "1,0.5,0.25",
                    "--max-false-accept",
                    "0.2",
                ],
            )
            self.assertEqual(fitter_code, 0, fitter_stderr)

            artifact = root / "phase_rejoin_v3.csv"
            nominal_report = root / "nominal_report.json"
            arguments = [
                str(BUILDER_PATH),
                "--path-json",
                str(path),
                "--plan-csv",
                str(plan),
                "--plan-report",
                str(plan_report),
                "--recovery-scales",
                str(evidence / "phase_rejoin_recovery_radii_bounds.csv"),
                "--recovery-manifest",
                str(evidence / "manifest.json"),
                "--held-out-report",
                str(evidence / "held_out_report.json"),
                "--artifact-validator",
                str(VALIDATOR_PATH),
                "--output",
                str(artifact),
                "--report-output",
                str(nominal_report),
                "--contract-id",
                "unit_test_formal_simulation_v1",
            ]

            alias_arguments = list(arguments)
            alias_arguments[alias_arguments.index(str(nominal_report))] = str(artifact)
            alias = subprocess.run(
                alias_arguments, text=True, capture_output=True, check=False
            )
            self.assertEqual(alias.returncode, 2)
            self.assertIn("must not alias", alias.stderr)
            self.assertFalse(artifact.exists())

            sidecar = evidence / "manifest.sha256"
            sidecar_contents = sidecar.read_bytes()
            sidecar.write_bytes(sidecar_contents + b"tampered\n")
            tampered = subprocess.run(
                arguments, text=True, capture_output=True, check=False
            )
            self.assertEqual(tampered.returncode, 2)
            self.assertIn("sidecar mismatch", tampered.stderr)
            self.assertFalse(artifact.exists())
            sidecar.write_bytes(sidecar_contents)

            completed = subprocess.run(
                arguments, text=True, capture_output=True, check=False
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            report = json.loads(nominal_report.read_text(encoding="utf-8"))
            self.assertTrue(report["all_constraints_satisfied"])
            self.assertTrue(report["dynamics_consistency_passed"])
            self.assertTrue(report["has_publish_zero_settle_hold"])
            self.assertFalse(report["validation_data_used_for_optimization"])
            self.assertFalse(report["physical_parameter_claim"])
            self.assertTrue(report["source_limitations_acknowledged"])
            self.assertEqual(
                report["artifact_validator_sha256"],
                FITTER.sha256_file(VALIDATOR_PATH),
            )
            self.assertIn("validate --artifact", report["validation_command"])
            validated = subprocess.run(
                [str(VALIDATOR_PATH), "validate", "--artifact", str(artifact)],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(validated.returncode, 0, validated.stderr)


if __name__ == "__main__":
    unittest.main()
