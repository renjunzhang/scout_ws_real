#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / \
    "tools/analysis/generate_phase_rejoin_development_nominal.py"
SPEC = importlib.util.spec_from_file_location("phase_rejoin_dev_nominal", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DevelopmentNominalWrapperTest(unittest.TestCase):
    def points(self):
        return tuple(
            MODULE.Point(0.05 * index, 0.15 * math.sin(0.15 * index))
            for index in range(81)
        )

    def arguments(self, output, overwrite=False):
        argv = [
            "--bag", "/unused/by-unit-test.bag",
            "--output", str(output),
            "--contract-id", "development_contract",
            "--frame-id", "map",
            "--dt", str(1.0 / 30.0),
            "--cruise-speed", "0.30",
            "--ramp-sec", "2.0",
            "--lookahead", "0.30",
            "--heading-gain", "3.0",
            "--omega-max", "1.0",
            "--alpha-max", "1.0",
            "--omega-n", "31.246035078551724",
            "--damping-ratio", "0.05",
            "--kappa-x", "1.0",
            "--kappa-y", "1.0",
            "--zero-hold-sec", "0.5",
            "--terminal-eta-norm-max", "2e-6",
            "--terminal-eta-dot-norm-max", "1e-4",
            "--gate-radii",
            "5", "5", "6.3", "2", "3", "0.5", "10", "0.5", "10",
        ]
        if overwrite:
            argv.append("--overwrite")
        return MODULE.build_parser().parse_args(argv)

    def test_wrapper_delegates_generation_validation_and_atomic_write(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "development_v2.csv"
            stdout = MODULE.generate_from_points(
                self.points(), self.arguments(output), "a" * 64
            )
            self.assertIn("rows=525", stdout)
            text = output.read_text(encoding="utf-8")
            self.assertIn("# schema=phase_rejoin_empirical_v2\n", text)
            self.assertIn("# evidence_level=development_only\n", text)
            self.assertIn(
                "# source=development_dynamics_consistent_nominal\n", text
            )
            self.assertIn("# terminal_zero_hold_steps=16\n", text)
            data_lines = [line for line in text.splitlines() if not line.startswith("#")]
            self.assertEqual(len(data_lines), 526)

            with self.assertRaisesRegex(MODULE.GenerationError, "OUTPUT_EXISTS"):
                MODULE.generate_from_points(
                    self.points(), self.arguments(output), "a" * 64
                )
            MODULE.generate_from_points(
                self.points(), self.arguments(output, overwrite=True), "a" * 64
            )

    def test_wrapper_surfaces_cpp_fail_closed_reason(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "invalid.csv"
            with self.assertRaisesRegex(
                MODULE.GenerationError, "INVALID_REFERENCE_PATH"
            ):
                MODULE.generate_from_points(
                    (MODULE.Point(0.0, 0.0),) * 3,
                    self.arguments(output),
                    "a" * 64,
                )
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
