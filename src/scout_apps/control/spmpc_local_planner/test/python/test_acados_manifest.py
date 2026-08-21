#!/usr/bin/env python3

import contextlib
import io
import pathlib
import sys
import tempfile
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[2]
ACADOS_TOOLS = PACKAGE_ROOT / "tools" / "codegen" / "acados"
sys.path.insert(0, str(ACADOS_TOOLS))

from generate_spmpc_acados import emit_cpp_manifest, load_config  # noqa: E402


class AcadosManifestTest(unittest.TestCase):
    def test_committed_cpp_manifest_is_exact_codegen_output(self):
        committed = (
            PACKAGE_ROOT / "generated" / "acados" /
            "spmpc_parameter_manifest.h")
        self.assertTrue(committed.is_file())
        with tempfile.TemporaryDirectory() as temporary:
            with contextlib.redirect_stdout(io.StringIO()):
                generated_path = emit_cpp_manifest(
                    load_config(), temporary)
            self.assertEqual(
                committed.read_bytes(),
                pathlib.Path(generated_path).read_bytes())

    def test_phase_rejoin_specialization_preserves_main_horizon_manifest(self):
        config = load_config()
        config["N"] = config["phase_rejoin_N"]
        config["Tf"] = config["dt"] * config["N"]
        with tempfile.TemporaryDirectory() as temporary:
            with contextlib.redirect_stdout(io.StringIO()):
                generated_path = emit_cpp_manifest(config, temporary)
            generated = pathlib.Path(generated_path).read_text()
        self.assertIn("constexpr int kMainHorizonSteps = 60;", generated)
        self.assertIn("constexpr int kPhaseRejoinHorizonSteps = 3;", generated)


if __name__ == "__main__":
    unittest.main()
