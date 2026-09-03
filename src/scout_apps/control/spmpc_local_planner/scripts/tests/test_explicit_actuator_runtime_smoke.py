#!/usr/bin/env python3
"""Contract tests for the one-bag explicit-actuator runtime smoke."""

import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
WRAPPER = SCRIPTS_DIR / "run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
ANALYZER = SCRIPTS_DIR / "analysis" / "validate_explicit_actuator_runtime_smoke.py"

SPEC = importlib.util.spec_from_file_location("runtime_smoke", ANALYZER)
runtime_smoke = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime_smoke)


class FakeStamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


def audit(when, duration_ms=10.0, active=True, **overrides):
    values = {
        "command_was_published": True,
        "published_cmd_v": 0.10 if active else 0.0,
        "published_cmd_omega": 0.0,
        "cycle_start_stamp": FakeStamp(when),
        "command_publish_stamp": FakeStamp(when + duration_ms / 1000.0),
        "state_alignment_required": True,
        "state_time_aligned": True,
        "solve_attempted": True,
        "solve_success": True,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def odom(when):
    return SimpleNamespace(
        measurement_stamp=FakeStamp(when), source_stamp=FakeStamp(when)
    )


def clean_intervention():
    return {field: 0.0 for field in runtime_smoke.BAD_ZERO_FIELDS}


class RuntimeAnalysisTest(unittest.TestCase):
    def test_percentile_matches_linear_definition(self):
        self.assertEqual(runtime_smoke.percentile_linear([1.0], 95.0), 1.0)
        self.assertAlmostEqual(
            runtime_smoke.percentile_linear([0.0, 10.0], 95.0), 9.5
        )

    def test_consecutive_overrun_counter(self):
        self.assertEqual(
            runtime_smoke.max_consecutive_true(
                [False, True, True, False, True]
            ),
            2,
        )

    def test_clean_runtime_passes(self):
        audits = [
            (1.0 + index / 30.0, audit(1.0 + index / 30.0))
            for index in range(20)
        ]
        odom_rows = [
            (1.0 + index * 0.02, odom(1.0 + index * 0.02))
            for index in range(34)
        ]
        interventions = [
            (when, clean_intervention()) for when, _ in audits
        ]
        report = runtime_smoke.compute_runtime_report(
            audits,
            odom_rows,
            interventions,
            protocol="TEST",
            bag="fake.bag",
        )
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["common_epoch_failure_count"], 0)
        self.assertEqual(report["planner_odom_gap_over_50ms_count"], 0)
        self.assertAlmostEqual(report["control_callback_p95_ms"], 10.0)

    def test_failures_are_reported_in_the_motion_window(self):
        times = [1.0 + index / 30.0 for index in range(20)]
        audits = [(when, audit(when)) for when in times]
        audits[5] = (
            times[5],
            audit(times[5], duration_ms=40.0, state_time_aligned=False),
        )
        audits[6] = (
            times[6],
            audit(times[6], duration_ms=45.0, solve_success=False),
        )
        odom_times = [1.0 + index * 0.02 for index in range(10)]
        odom_times.extend(1.25 + index * 0.02 for index in range(20))
        odom_rows = [(when, odom(when)) for when in odom_times]
        interventions = [(when, clean_intervention()) for when in times]
        interventions[7][1]["zero_due_to_waiting_for_slosh_observer"] = 1.0

        report = runtime_smoke.compute_runtime_report(
            audits,
            odom_rows,
            interventions,
            protocol="TEST",
            bag="fake.bag",
        )
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["common_epoch_failure_count"], 1)
        self.assertEqual(report["solver_failure_count"], 1)
        self.assertEqual(report["fault_zero_count"], 1)
        self.assertEqual(report["planner_odom_gap_over_50ms_count"], 1)
        self.assertEqual(report["max_consecutive_callback_overrun"], 2)


class WrapperContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wrapper = WRAPPER.read_text(encoding="utf-8")

    def test_is_an_independent_one_bslosh_protocol(self):
        for token in (
            "SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_RUNTIME_SMOKE_DEV_V1",
            "VARIANT=B_slosh",
            "W_SLOSH=5.0",
            "one bag only",
            "NOT B0/Bslosh efficacy",
        ):
            self.assertIn(token, self.wrapper)
        self.assertNotIn("run_spmpc_i0_failclosed_fixed_abba_engine.sh", self.wrapper)
        self.assertNotIn("PAIR_ROW", self.wrapper)

    def test_motion_is_validate_only_by_default_and_triply_armed(self):
        for token in (
            'VALIDATE_ONLY="${VALIDATE_ONLY:-true}"',
            '[[ "${ARM_MOTION}" == "YES" ]]',
            '[[ "${CONFIRM_RUNTIME_SMOKE}" == "YES" ]]',
            '[[ "${CONFIRM_PATH_CLEAR}" == "YES" ]]',
            "validate-only PASS; motion NOT started",
        ):
            self.assertIn(token, self.wrapper)
        self.assertLess(
            self.wrapper.index('if truthy "${VALIDATE_ONLY}"'),
            self.wrapper.index('bash "${RUNNER}"'),
        )

    def test_runtime_and_weights_are_frozen(self):
        for token in (
            "EXECUTION_MODEL_MODE=explicit_actuator",
            "DELAY_PHASE_MODE=off",
            "ACTUATOR_LINEAR_DELAY_SEC=0.1666666665",
            "ACTUATOR_ANGULAR_DELAY_SEC=0.3333333330",
            "ACTUATOR_LINEAR_TAU_SEC=0.112",
            "ACTUATOR_ANGULAR_TAU_SEC=0.119",
            "W_SMOOTH=0.1",
            "W_ALPHA=0.1",
            "W_DU_A=0.1",
            "W_DU_VS=0.1",
            "STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=0.050",
            "qp_solver_cond_N=10",
            "odom_subscriber_queue_size=10",
        ):
            self.assertIn(token, self.wrapper)
        self.assertNotIn("W_ACCEL", self.wrapper)

    def test_rgb_is_disabled_and_both_postflights_are_automatic(self):
        for token in (
            "PILOT_RECORD_RGB=false",
            "PILOT_RECORD_ONLINE_LIQUID=false",
            "RECORD_CAMERA=false",
            "RECORD_CAMERA_INFO=false",
            "FORBID_IMAGE_STREAMS=true",
            "validate_i0_failclosed_fixed_abba_bag.py",
            "validate_explicit_actuator_runtime_smoke.py",
            "--max-control-callback-p95-ms 30.0",
            "--max-consecutive-callback-overrun 1",
        ):
            self.assertIn(token, self.wrapper)

    def test_frozen_path_map_and_non_overwrite_gate_are_present(self):
        for token in (
            "mocap_compact_s_C02.json",
            "1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164",
            "map_carto_20260829_mocap_exec_v1.pbstream",
            "34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595",
            "preserve existing output",
            "runtime/evidence paths are dirty; commit and rebuild before motion",
        ):
            self.assertIn(token, self.wrapper)


if __name__ == "__main__":
    unittest.main()
