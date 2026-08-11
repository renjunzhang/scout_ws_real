#!/usr/bin/env python3
"""Static and negative tests for the v5 viscosity-strength revision."""

from __future__ import annotations

import copy
import hashlib
import json
import subprocess
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = PACKAGE_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import r8_liquid_u3_gpu_damped_restart_runner_v1 as base
import r8_liquid_u3_gpu_damped_restart_runner_v5 as runner


POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_damped_restart_viscoart0p1_20260811T061307Z_v5.json"
PRIOR_POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_damped_restart_viscoart_20260811T052342Z_v4.json"
UPSTREAM_GIT = Path("/home/zrj/scout_liquid_lab/dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git")


class DampedRestartRunnerV5Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        runner.configure()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.prior = json.loads(PRIOR_POLICY_PATH.read_text(encoding="utf-8"))

    def test_policy_loads_with_exact_official_3d_example_strength(self) -> None:
        loaded = base.load_policy(POLICY_PATH)
        for phase_name in base.PHASE_KEYS:
            argv = loaded["phases"][phase_name]["solver_argv"]
            self.assertEqual(argv, runner.exact_solver_argv_v5(phase_name))
            self.assertEqual(argv.count(runner.VISCO_ARG), 1)
        proof = runner.semantic_validate_v5(loaded, POLICY_PATH)
        revision = proof["viscosity_strength_revision"]
        self.assertEqual(revision["single_delta_from_v4"], "ARTIFICIAL_0P01_TO_UPSTREAM_3D_EXAMPLE_0P1")
        self.assertFalse(revision["other_numerical_contract_changed"])

    def test_rejects_default_intermediate_or_second_viscosity(self) -> None:
        mutations = []
        for value in ("-viscoart:0.01", "-viscoart:0.02"):
            candidate = copy.deepcopy(self.policy)
            index = candidate["phases"]["undamped_tail"]["solver_argv"].index(runner.VISCO_ARG)
            candidate["phases"]["undamped_tail"]["solver_argv"][index] = value
            mutations.append(candidate)
        second = copy.deepcopy(self.policy)
        second["phases"]["damped_init"]["solver_argv"].append("-viscolam:0.000001")
        mutations.append(second)
        for candidate in mutations:
            with self.subTest(candidate=candidate), self.assertRaises(base.DampedRestartRunError):
                runner.semantic_validate_v5(candidate, POLICY_PATH)

    def test_only_strength_and_identities_differ_from_v4(self) -> None:
        for phase_name in base.PHASE_KEYS:
            current = self.policy["phases"][phase_name]
            prior = self.prior["phases"][phase_name]
            normalized = [runner.prior_v4.VISCO_ARG if item == runner.VISCO_ARG else item for item in current["solver_argv"]]
            self.assertEqual(normalized, prior["solver_argv"])
            for key in ("purpose", "case_xml_input", "restart", "expected_output", "limits"):
                self.assertEqual(current[key], prior[key])
            current_sandbox = dict(current["sandbox"])
            prior_sandbox = dict(prior["sandbox"])
            current_sandbox.pop("stage_root")
            prior_sandbox.pop("stage_root")
            self.assertEqual(current_sandbox, prior_sandbox)
        for key in ("tools", "gpu", "inputs", "damping_contract", "solver_invariants", "acceptance", "result_boundary"):
            self.assertEqual(self.policy[key], self.prior[key])

    def test_sealed_upstream_3d_example_uses_artificial_0p1(self) -> None:
        revision = f"{runner.UPSTREAM_COMMIT}:{runner.UPSTREAM_EXAMPLE_PATH}"
        completed = subprocess.run(
            ["/usr/bin/git", f"--git-dir={UPSTREAM_GIT}", "show", revision],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
            env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"},
        )
        self.assertEqual(completed.returncode, 0, completed.stderr.decode("utf-8", "replace"))
        self.assertEqual(hashlib.sha256(completed.stdout).hexdigest(), runner.UPSTREAM_EXAMPLE_SHA256)
        text = completed.stdout.decode("utf-8")
        self.assertIn('key="ViscoTreatment" value="1"', text)
        self.assertIn('key="Visco" value="0.1"', text)
        self.assertIn('<size x="0.12" y="0.12" z="0.45"', text)


if __name__ == "__main__":
    unittest.main()
