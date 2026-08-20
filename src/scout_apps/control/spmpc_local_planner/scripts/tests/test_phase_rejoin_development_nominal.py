#!/usr/bin/env python3

import csv
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / \
    "generate_phase_rejoin_development_nominal.py"
SPEC = importlib.util.spec_from_file_location("phase_rejoin_dev_nominal", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DevelopmentNominalTest(unittest.TestCase):
    def setUp(self):
        self.points = tuple(
            MODULE.Point(0.05 * index, 0.15 * math.sin(0.15 * index))
            for index in range(81)
        )
        omega_n = 31.246035078551724
        self.model = MODULE.LiquidModel(
            2.0 * 0.05 * omega_n, omega_n * omega_n, 1.0, 1.0)
        self.radii = (5.0, 5.0, 6.3, 2.0, 3.0, 0.5, 10.0, 0.5, 10.0)

    def generate(self):
        return MODULE.generate_rows(
            points=self.points,
            dt=1.0 / 30.0,
            requested_speed=0.30,
            ramp_sec=2.0,
            lookahead=0.30,
            heading_gain=3.0,
            omega_max=1.0,
            alpha_max=1.0,
            zero_hold_sec=0.5,
            terminal_eta_norm_max=2.0e-6,
            terminal_eta_dot_norm_max=1.0e-4,
            model=self.model,
            radii=self.radii,
        )

    def test_generates_uniform_dynamics_consistent_complete_tail(self):
        rows, hold_steps, deviation, path_length = self.generate()
        self.assertGreater(len(rows), 100)
        self.assertGreaterEqual(hold_steps, 5)
        self.assertLess(deviation, 0.20)
        self.assertGreater(path_length, 4.0)

        for index, row in enumerate(rows):
            self.assertEqual(row[0], index)
            self.assertAlmostEqual(row[1], index / 30.0, places=12)
            self.assertEqual(len(row), len(MODULE.HEADER))
            self.assertAlmostEqual(row[17], row[15], places=12)
            self.assertAlmostEqual(row[18], row[16], places=12)

        for row in rows[-hold_steps:]:
            for column in (6, 7, 12, 13, 14, 15, 16, 17, 18):
                self.assertAlmostEqual(row[column], 0.0, places=12)
            self.assertAlmostEqual(row[2], path_length, places=12)

        final = rows[-1]
        self.assertLessEqual(math.hypot(final[8], final[10]), 2.0e-6)
        self.assertLessEqual(math.hypot(final[9], final[11]), 1.0e-4)

    def test_serialization_preserves_fixed_schema_and_markers(self):
        rows, hold_steps, deviation, path_length = self.generate()
        metadata = (
            ("schema", MODULE.SCHEMA),
            ("evidence_level", MODULE.EVIDENCE_LEVEL),
            ("source", MODULE.SOURCE),
            ("terminal_contract", MODULE.TERMINAL_CONTRACT),
            ("recovery_contract", MODULE.RECOVERY_CONTRACT),
            ("terminal_zero_hold_steps", str(hold_steps)),
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "development_v2.csv"
            MODULE.write_artifact(output, metadata, rows, overwrite=False)
            text = output.read_text(encoding="utf-8")
            self.assertIn("# schema=phase_rejoin_empirical_v2\n", text)
            self.assertIn("# evidence_level=development_only\n", text)
            self.assertIn(
                "# recovery_contract=nominal_command_v1\n", text)
            with output.open(encoding="utf-8", newline="") as stream:
                reader = csv.reader(line for line in stream if not line.startswith("#"))
                self.assertEqual(tuple(next(reader)), MODULE.HEADER)
                self.assertEqual(sum(1 for _ in reader), len(rows))

    def test_rejects_implicit_or_invalid_generation_inputs(self):
        with self.assertRaisesRegex(MODULE.GenerationError, "three distinct"):
            MODULE.clean_points([MODULE.Point(0.0, 0.0)] * 3)
        with self.assertRaisesRegex(MODULE.GenerationError, "ramp is too long"):
            MODULE.speed_schedule(0.2, 0.1, 1.0, 1.0)


if __name__ == "__main__":
    unittest.main()
