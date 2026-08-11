#!/usr/bin/env python3
"""Static and negative tests for the v7 artificial-viscosity probe runner."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_gpu_viscoart0p2_probe_runner_v1 as runner  # noqa: E402


class ViscoartProbeRunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        runner.configure()
        cls.policy = json.loads(runner.POLICY_PATH.read_text(encoding="utf-8"))

    def test_schema_and_exact_policy_pass(self) -> None:
        schema = json.loads(runner.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertEqual(list(Draft202012Validator(schema).iter_errors(self.policy)), [])
        runner.semantic_validate(self.policy, runner.POLICY_PATH)

    def test_exact_argv_has_one_official_viscosity_and_restart(self) -> None:
        argv = runner.exact_solver_argv()
        self.assertEqual([item for item in argv if item.startswith("-visco")], ["-viscoart:0.2"])
        self.assertEqual(argv.count("-partbegin:601:601"), 1)
        self.assertEqual(argv.count("-cfl:0.1"), 1)
        self.assertEqual(argv.count("-shifting:none"), 1)
        self.assertNotIn("-cpu", argv)

    def _reject(self, mutate) -> None:
        candidate = copy.deepcopy(self.policy)
        mutate(candidate)
        with self.assertRaises(runner.ViscoartProbeRunError):
            runner.semantic_validate(candidate, runner.POLICY_PATH)

    def test_rejects_viscosity_drift(self) -> None:
        self._reject(lambda value: value["solver"]["argv"].__setitem__(12, "-viscoart:0.3"))

    def test_rejects_restart_drift(self) -> None:
        self._reject(lambda value: value["restart"].__setitem__("part_index", 600))

    def test_rejects_numerical_delta_drift(self) -> None:
        self._reject(lambda value: value["single_delta"].__setitem__("parameter", "OBSERVATION_WINDOW"))

    def test_rejects_network_or_extra_schema_key(self) -> None:
        self._reject(lambda value: value["sandbox"].__setitem__("network", True))
        schema = json.loads(runner.SCHEMA_PATH.read_text(encoding="utf-8"))
        candidate = copy.deepcopy(self.policy)
        candidate["unexpected"] = True
        self.assertTrue(list(Draft202012Validator(schema).iter_errors(candidate)))

    def test_bwrap_has_one_output_bind_and_no_network(self) -> None:
        argv = runner.base.bwrap_argv(self.policy)
        self.assertIn("--unshare-net", argv)
        self.assertNotIn("--share-net", argv)
        binds = [argv[index + 1:index + 3] for index, item in enumerate(argv) if item == "--bind"]
        self.assertEqual(binds, [[self.policy["sandbox"]["stage_root"] + "/output", "/output"]])


if __name__ == "__main__":
    unittest.main()
