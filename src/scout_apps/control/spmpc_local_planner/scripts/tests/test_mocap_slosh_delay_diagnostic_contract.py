#!/usr/bin/env python3

import importlib.util
import pathlib
import subprocess
import unittest
from types import SimpleNamespace


SCRIPT_ROOT = pathlib.Path(__file__).resolve().parents[1]
RUNNER = SCRIPT_ROOT / "run_spmpc_mocap_slosh_delay_diagnostic_trial.sh"
POSTFLIGHT_PATH = SCRIPT_ROOT / "analysis/validate_slosh_nowcast_shadow_bag.py"

SPEC = importlib.util.spec_from_file_location(
    "validate_slosh_observer_contract", POSTFLIGHT_PATH
)
POSTFLIGHT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(POSTFLIGHT)


def row(applied):
    return SimpleNamespace(applied_to_solver=applied)


class AppliedMethodContractTest(unittest.TestCase):
    def test_i0_is_the_only_required_solver_input(self):
        self.assertIsNone(
            POSTFLIGHT.applied_method_contract_failure(
                "I0", [row(True), row(True)], "I0"
            )
        )
        self.assertIsNone(
            POSTFLIGHT.applied_method_contract_failure(
                "L22", [row(False), row(False)], "I0"
            )
        )

    def test_missing_i0_application_fails(self):
        failure = POSTFLIGHT.applied_method_contract_failure(
            "I0", [row(True), row(False)], "I0"
        )
        self.assertIn("applied in 1/2", failure)

    def test_unexpected_legacy_application_fails(self):
        failure = POSTFLIGHT.applied_method_contract_failure(
            "L22", [row(False), row(True)], "I0"
        )
        self.assertIn("unexpectedly entered", failure)

    def test_original_b0_none_applied_contract_is_preserved(self):
        for method in ("O0", "I0", "I1", "L22"):
            self.assertIsNone(
                POSTFLIGHT.applied_method_contract_failure(
                    method, [row(False), row(False)], "NONE"
                )
            )


class RunnerContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = RUNNER.read_text(encoding="utf-8")

    def test_shell_syntax(self):
        result = subprocess.run(
            ["bash", "-n", str(RUNNER)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_safe_c02_defaults_are_frozen(self):
        for fragment in (
            "SLOSH_CONDITION=\"${SLOSH_CONDITION:-matched0}\"",
            "V_REF=0.10",
            "V_SAFE_MAX=0.15",
            "DELAY_PHASE_MODE=shadow",
            "CURRENT_OBSERVER_SOURCE=processed_imu",
            "OBSERVER_FALLBACK_POLICY=fail_closed",
            "STATE_TIMING_REQUIRE_COMMON_EPOCH=true",
        ):
            self.assertIn(fragment, self.text)

    def test_candidate_model_is_metadata_only(self):
        self.assertIn(
            "ACTUATOR_CANDIDATE_PRESET_ID=actuator_fopdt_20260831_same_bag_v1",
            self.text,
        )
        self.assertIn("actuator_candidate_applied=false", self.text)
        self.assertIn("legacy_delay_applied=false", self.text)

    def test_postflight_requires_i0_and_rejects_legacy_rollout(self):
        self.assertIn("--expected-solver-consumes-liquid", self.text)
        self.assertIn("--expected-applied-method I0", self.text)


if __name__ == "__main__":
    unittest.main()
