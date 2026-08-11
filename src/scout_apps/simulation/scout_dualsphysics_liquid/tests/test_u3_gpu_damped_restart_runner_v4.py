#!/usr/bin/env python3
"""Static and negative tests for the artificial-viscosity v4 revision."""

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
import r8_liquid_u3_gpu_damped_restart_runner_v4 as runner


POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_damped_restart_viscoart_20260811T052342Z_v4.json"
UPSTREAM_GIT = Path("/home/zrj/scout_liquid_lab/dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git")
UPSTREAM_TEMPLATE = "doc/xml_format/_FmtXML__Parameters.xml"


class DampedRestartRunnerV4Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        runner.configure()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))

    def test_policy_loads_with_exact_upstream_default_viscosity(self) -> None:
        loaded = base.load_policy(POLICY_PATH)
        for phase_name in base.PHASE_KEYS:
            argv = loaded["phases"][phase_name]["solver_argv"]
            self.assertEqual(argv, runner.exact_solver_argv_v4(phase_name))
            self.assertEqual(argv.count(runner.VISCO_ARG), 1)
        proof = runner.semantic_validate_v4(loaded, POLICY_PATH)
        self.assertFalse(proof["viscosity_revision"]["other_numerical_contract_changed"])

    def test_rejects_missing_alternate_or_second_viscosity_override(self) -> None:
        mutations = []
        missing = copy.deepcopy(self.policy)
        missing["phases"]["undamped_tail"]["solver_argv"].remove(runner.VISCO_ARG)
        mutations.append(missing)
        alternate = copy.deepcopy(self.policy)
        index = alternate["phases"]["damped_init"]["solver_argv"].index(runner.VISCO_ARG)
        alternate["phases"]["damped_init"]["solver_argv"][index] = "-viscoart:0.02"
        mutations.append(alternate)
        second = copy.deepcopy(self.policy)
        second["phases"]["undamped_tail"]["solver_argv"].append("-viscolam:0.000001")
        mutations.append(second)
        for candidate in mutations:
            with self.subTest(candidate=candidate):
                with self.assertRaises(base.DampedRestartRunError):
                    runner.semantic_validate_v4(candidate, POLICY_PATH)

    def test_all_prior_numerical_and_resource_contracts_remain_exact(self) -> None:
        loaded = base.load_policy(POLICY_PATH)
        self.assertEqual(loaded["solver_invariants"]["cfl"], 0.1)
        self.assertEqual(loaded["solver_invariants"]["shifting"], "None")
        self.assertEqual(loaded["solver_invariants"]["ddt"], "DDT2(0.1)")
        self.assertEqual(loaded["solver_invariants"]["dp_m"], 0.002)
        self.assertTrue(loaded["acceptance"]["all_17_absolute_limits_required"])
        self.assertEqual(
            {phase["limits"]["minimum_mem_available_bytes"] for phase in loaded["phases"].values()},
            {runner.resource_v3.HARD_MEMORY_FLOOR_BYTES},
        )

    def test_sealed_upstream_template_proves_default_artificial_0p01(self) -> None:
        completed = subprocess.run(
            ["/usr/bin/git", f"--git-dir={UPSTREAM_GIT}", "show", f"{runner.UPSTREAM_COMMIT}:{UPSTREAM_TEMPLATE}"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
            env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"},
        )
        self.assertEqual(completed.returncode, 0, completed.stderr.decode("utf-8", "replace"))
        self.assertEqual(hashlib.sha256(completed.stdout).hexdigest(), runner.UPSTREAM_PARAMETER_TEMPLATE_SHA256)
        text = completed.stdout.decode("utf-8")
        self.assertIn('key="ViscoTreatment" value="1"', text)
        self.assertIn('key="Visco" value="0.01"', text)


if __name__ == "__main__":
    unittest.main()
