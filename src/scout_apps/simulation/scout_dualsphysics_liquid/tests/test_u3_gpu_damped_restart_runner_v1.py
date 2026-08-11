#!/usr/bin/env python3
"""Static and negative tests for the two-phase damped-restart runner."""

from __future__ import annotations

import copy
import json
import os
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = PACKAGE_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import r8_liquid_u3_gpu_damped_restart_runner_v1 as runner


POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_damped_restart_20260811T032441Z_v1.json"


class DampedRestartRunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.schema = json.loads(runner.SCHEMA_PATH.read_text(encoding="utf-8"))
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)

    def test_frozen_policy_loads_and_self_semantics_pass(self) -> None:
        loaded = runner.load_policy(POLICY_PATH)
        self.assertEqual(loaded["experiment_id"], self.policy["experiment_id"])
        proof = runner.semantic_validate(loaded, POLICY_PATH)
        self.assertTrue(proof["xml_delta"]["uniform_max_reduction_coverage"])
        self.assertTrue(proof["source_semantics"]["gpu_damping_call"])

    def test_schema_is_closed_at_top_and_phase(self) -> None:
        validator = Draft202012Validator(self.schema)
        for mutation in (
            lambda value: value.update({"unexpected": True}),
            lambda value: value["phases"]["damped_init"].update({"unexpected": True}),
        ):
            candidate = copy.deepcopy(self.policy)
            mutation(candidate)
            self.assertTrue(list(validator.iter_errors(candidate)))

    def test_rejects_any_damping_permission_or_value_drift(self) -> None:
        candidate = copy.deepcopy(self.policy)
        candidate["damping_contract"]["redumax_per_s"] = 9
        with self.assertRaises(runner.DampedRestartRunError):
            runner.semantic_validate(candidate, POLICY_PATH)

    def test_rejects_cfl_or_shifting_drift(self) -> None:
        for phase_name, replacement in (
            ("damped_init", "-cfl:0.2"),
            ("undamped_tail", "-shifting:nobound"),
        ):
            candidate = copy.deepcopy(self.policy)
            argv = candidate["phases"][phase_name]["solver_argv"]
            index = next(index for index, item in enumerate(argv) if item.startswith("-cfl:") if replacement.startswith("-cfl:")) if replacement.startswith("-cfl:") else next(index for index, item in enumerate(argv) if item.startswith("-shifting:"))
            argv[index] = replacement
            with self.assertRaises(runner.DampedRestartRunError):
                runner.semantic_validate(candidate, POLICY_PATH)

    def test_rejects_metric_threshold_drift(self) -> None:
        candidate = copy.deepcopy(self.policy)
        candidate["acceptance"]["metric_limits"]["speed_rms_m_s"] *= 2
        with self.assertRaises(runner.DampedRestartRunError):
            runner.semantic_validate(candidate, POLICY_PATH)

    def test_exact_phase_argv_has_one_gpu_no_network_and_no_home_path(self) -> None:
        for phase_name in runner.PHASE_KEYS:
            argv = runner._exact_solver_argv(phase_name)
            self.assertEqual(argv.count("-gpu:0"), 1)
            self.assertEqual(argv.count("-ompthreads:1"), 1)
            self.assertEqual(argv.count("-cfl:0.1"), 1)
            self.assertEqual(argv.count("-shifting:none"), 1)
            self.assertFalse(any("/home/" in item or "http" in item for item in argv))

    def test_tail_is_only_phase_with_restart(self) -> None:
        init_argv = runner._exact_solver_argv("damped_init")
        tail_argv = runner._exact_solver_argv("undamped_tail")
        self.assertFalse(any(item.startswith("-partbegin:") for item in init_argv))
        self.assertEqual([item for item in tail_argv if item.startswith("-partbegin:")], ["-partbegin:201:201"])

    def test_bwrap_has_one_writable_bind_and_unshared_network(self) -> None:
        for phase_name in runner.PHASE_KEYS:
            argv = runner._bwrap_argv(self.policy, phase_name)
            self.assertIn("--unshare-net", argv)
            self.assertNotIn("--share-net", argv)
            self.assertEqual(argv.count("--bind"), 1)
            self.assertNotIn(str(PACKAGE_ROOT), argv)

    def test_static_helpers_do_not_require_root(self) -> None:
        self.assertNotEqual(os.geteuid(), 0)
        self.assertEqual(runner.validate_xml_delta(self.policy)["status"], "PASS_EXACT_INITIALIZATION_ONLY_DAMPING_XML_DELTA")


if __name__ == "__main__":
    unittest.main()
