#!/usr/bin/env python3

import contextlib
import io
import pathlib
import sys
import tempfile
import unittest

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[2]
TOOLS = PACKAGE_ROOT / "tools" / "codegen" / "acados"
sys.path.insert(0, str(TOOLS))

from generate_delay_augmented_phase_transition import (  # noqa: E402
    generate,
    load_contract,
)


class DelayAugmentedPhaseCodegenTest(unittest.TestCase):
    def test_contract_uses_dedicated_execution_model(self):
        common = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "planner" / "common.yaml")
            .read_text(encoding="utf-8"))
        self.assertEqual(common["delay_phase"]["linear_delay_sec"], 0.15)
        self.assertEqual(common["delay_phase"]["angular_delay_sec"], 0.22)
        self.assertEqual(
            common["delay_phase"]["linear_time_constant_sec"], 0.0)
        self.assertEqual(
            common["delay_phase"]["angular_time_constant_sec"], 0.0)
        execution_model = common["delay_augmented_phase"]["execution_model"]
        self.assertEqual(execution_model["linear_delay_sec"], 0.102)
        self.assertEqual(execution_model["angular_delay_sec"], 0.010)
        self.assertEqual(
            execution_model["linear_time_constant_sec"], 0.091)
        self.assertEqual(
            execution_model["angular_time_constant_sec"], 0.342)

        contract = load_contract()
        self.assertEqual(
            contract["contract_hash"],
            "b7bbc6e4b921b513e798e4faaf22a8b147b6aea526408e2ed6c13ef9f8e0344a",
        )
        self.assertEqual(contract["linear"]["delay_sec"], 0.102)
        self.assertEqual(contract["angular"]["delay_sec"], 0.010)
        self.assertEqual(
            contract["linear"]["time_constant_sec"], 0.091)
        self.assertEqual(
            contract["angular"]["time_constant_sec"], 0.342)

    def test_committed_transition_is_exact_codegen_output(self):
        committed_root = PACKAGE_ROOT / "generated" / "casadi"
        filenames = (
            "spmpc_delay_augmented_phase_transition.c",
            "spmpc_delay_augmented_phase_transition.h",
            "spmpc_delay_augmented_phase_manifest.h",
        )
        with tempfile.TemporaryDirectory() as temporary:
            with contextlib.redirect_stdout(io.StringIO()):
                generate(temporary)
            generated_root = pathlib.Path(temporary)
            for filename in filenames:
                self.assertEqual(
                    (committed_root / filename).read_bytes(),
                    (generated_root / filename).read_bytes(),
                    filename,
                )


if __name__ == "__main__":
    unittest.main()
