#!/usr/bin/env python3
"""Contract tests for the one-bag explicit-actuator runtime smoke."""

import importlib.util
import subprocess
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
WRAPPER = SCRIPTS_DIR / "run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
SHORT_WRAPPER = SCRIPTS_DIR / "run_spmpc_weight_smoke.sh"
ANALYZER = SCRIPTS_DIR / "analysis" / "validate_explicit_actuator_runtime_smoke.py"
EXACT_VALIDATOR = SCRIPTS_DIR / "analysis" / "validate_i0_failclosed_fixed_abba_bag.py"
FULL_DA_WRAPPER = SCRIPTS_DIR / "run_spmpc_full_da_smoke.sh"

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
        "solver_status": "B_slosh_ACADOS_OK",
        "terminal_phase": False,
        "solver_u0_a": 0.0,
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

    def test_five_hz_metric_reproduces_frozen_amplitude_convention(self):
        values = [
            0.07821 * __import__("math").sin(2.0 * __import__("math").pi * 5.0 * k / 30.0)
            for k in range(330)
        ]
        frequency, amplitude = runtime_smoke.dominant_uniform_dft(
            values, 30.0, 4.5, 5.5
        )
        self.assertAlmostEqual(frequency, 5.0, places=12)
        self.assertAlmostEqual(amplitude, 0.07821, places=10)

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

    def test_full_da_control_gates_pass_for_smooth_a0(self):
        import math

        times = [1.0 + index / 30.0 for index in range(330)]
        audits = [
            (
                when,
                audit(
                    when,
                    published_cmd_omega=0.10,
                    solver_u0_a=0.02 * math.sin(2.0 * math.pi * 5.0 * index / 30.0),
                ),
            )
            for index, when in enumerate(times)
        ]
        odom_rows = [(when, odom(when)) for when in times]
        interventions = [(when, clean_intervention()) for when in times]
        report = runtime_smoke.compute_runtime_report(
            audits,
            odom_rows,
            interventions,
            protocol="FULL_DA_TEST",
            bag="fake.bag",
            max_delta_a0_p95=0.0785,
            max_turning_a0_5hz_amplitude=0.0391,
            max_strong_a0_sign_flips=0,
        )
        self.assertEqual(report["status"], "PASS")
        self.assertAlmostEqual(
            report["control_continuity"]["turning_a0_band_peak_amplitude_mps2"],
            0.02,
            places=10,
        )

    def test_full_da_control_gates_reject_baseline_oscillation(self):
        import math

        times = [1.0 + index / 30.0 for index in range(330)]
        audits = [
            (
                when,
                audit(
                    when,
                    published_cmd_omega=0.10,
                    solver_u0_a=0.07821 * math.sin(
                        2.0 * math.pi * 5.0 * index / 30.0
                    ),
                ),
            )
            for index, when in enumerate(times)
        ]
        report = runtime_smoke.compute_runtime_report(
            audits,
            [(when, odom(when)) for when in times],
            [(when, clean_intervention()) for when in times],
            protocol="FULL_DA_TEST",
            bag="fake.bag",
            max_delta_a0_p95=0.0785,
            max_turning_a0_5hz_amplitude=0.0391,
            max_strong_a0_sign_flips=0,
        )
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(
            any("~5 Hz amplitude" in failure for failure in report["failures"])
        )


class WrapperContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wrapper = WRAPPER.read_text(encoding="utf-8")
        cls.short_wrapper = SHORT_WRAPPER.read_text(encoding="utf-8")
        cls.full_da_wrapper = FULL_DA_WRAPPER.read_text(encoding="utf-8")
        cls.exact_validator = EXACT_VALIDATOR.read_text(encoding="utf-8")

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
            'SMOKE_PROFILE="${SMOKE_PROFILE:-runtime_baseline}"',
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
            "W_ACCEL=0.0",
            "STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=0.050",
            "qp_solver_cond_N=10",
            "odom_subscriber_queue_size=10",
        ):
            self.assertIn(token, self.wrapper)

    def test_waccel03_is_a_distinct_single_variable_profile(self):
        for token in (
            "waccel03)",
            "SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_WACCEL03_SMOKE_DEV_V1",
            "spmpc_i0_failclosed_explicit_actuator_waccel03_smoke_v1",
            "DEV_I0FC_EXPACT_WACCEL03_SMOKE_V1",
            "W_ACCEL=0.3",
            'w_accel:="${W_ACCEL}"',
            '--expected-config "w_accel=${W_ACCEL}"',
        ):
            self.assertIn(token, self.wrapper)

    def test_weight_tuning_profile_exposes_only_cost_knobs(self):
        for token in (
            "weight_tuning)",
            "SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_WEIGHT_TUNING_SMOKE_DEV_V1",
            'W_SLOSH="${TUNE_W_SLOSH:-1.0}"',
            'W_ACCEL="${TUNE_W_ACCEL:-0.3}"',
            'W_DU_A="${TUNE_W_DU_A:-0.1}"',
            'W_ALPHA="${TUNE_W_ALPHA:-0.1}"',
            '--expected-w-slosh "${W_SLOSH}"',
        ):
            self.assertIn(token, self.wrapper)
        self.assertIn('"--expected-w-slosh"', self.exact_validator)

    def test_full_da_profile_is_frozen_and_gates_the_two_baseline_metrics(self):
        for token in (
            "full_da)",
            "SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_FULL_DA_SMOKE_DEV_V1",
            "W_SLOSH=1.0",
            "W_ACCEL=0.3",
            "W_DU_A=0.1",
            "W_ALPHA=0.1",
            "EXPECTED_B0_STATE_WIDTH=24",
            "EXPECTED_SLOSH_STATE_WIDTH=28",
            "--max-delta-a0-p95 0.0785",
            "--max-turning-a0-5hz-amplitude 0.0391",
            "--max-strong-a0-sign-flips 0",
        ):
            self.assertIn(token, self.wrapper)
        subprocess.run(["bash", "-n", str(FULL_DA_WRAPPER)], check=True)
        for token in (
            "SMOKE_PROFILE=full_da",
            "VALIDATE_ONLY=true",
            "VALIDATE_ONLY=false",
            "ARM_MOTION=YES",
            "CONFIRM_RUNTIME_SMOKE=YES",
            "CONFIRM_PATH_CLEAR=YES",
        ):
            self.assertIn(token, self.full_da_wrapper)

    def test_short_cli_is_validate_only_by_default_and_requires_run_flag(self):
        subprocess.run(["bash", "-n", str(SHORT_WRAPPER)], check=True)
        for token in (
            "--run",
            "--w-slosh",
            "--w-accel",
            "--w-du-a",
            "--w-alpha",
            "VALIDATE_ONLY=false",
            "CONFIRM_PATH_CLEAR=YES",
            "VALIDATE_ONLY=true",
        ):
            self.assertIn(token, self.short_wrapper)
        self.assertLess(
            self.short_wrapper.index('run_motion=false'),
            self.short_wrapper.index('if [[ "${run_motion}" == "true" ]]'),
        )

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
