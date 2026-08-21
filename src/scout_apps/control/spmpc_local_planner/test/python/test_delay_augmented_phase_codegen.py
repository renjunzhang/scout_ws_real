#!/usr/bin/env python3

import contextlib
import io
import pathlib
import sys
import tempfile
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[2]
TOOLS = PACKAGE_ROOT / "tools" / "codegen" / "acados"
sys.path.insert(0, str(TOOLS))

from generate_delay_augmented_phase_transition import generate  # noqa: E402


class DelayAugmentedPhaseCodegenTest(unittest.TestCase):
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
