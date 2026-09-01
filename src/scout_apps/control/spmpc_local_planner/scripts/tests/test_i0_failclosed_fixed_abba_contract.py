#!/usr/bin/env python3
"""Static contract tests for the literal I0/fail-closed/fixed ABBA wrapper."""

import py_compile
import subprocess
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
PLANNER_SCRIPTS = SCRIPT_DIR.parent
WRAPPER = PLANNER_SCRIPTS / "run_spmpc_i0_failclosed_fixed_abba_trial.sh"
VALIDATOR = PLANNER_SCRIPTS / "analysis" / "validate_i0_failclosed_fixed_abba_bag.py"
ANALYZER = PLANNER_SCRIPTS / "analysis" / "analyze_i0_failclosed_fixed_abba_rgb.py"
RGB_VALIDATOR = PLANNER_SCRIPTS / "analysis" / "validate_g3_online_rgb_trial.py"


class I0FailClosedFixedAbbaContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wrapper = WRAPPER.read_text(encoding="utf-8")
        cls.validator = VALIDATOR.read_text(encoding="utf-8")

    def test_shell_and_python_syntax(self):
        subprocess.run(["bash", "-n", str(WRAPPER)], check=True)
        py_compile.compile(str(VALIDATOR), doraise=True)
        py_compile.compile(str(ANALYZER), doraise=True)
        py_compile.compile(str(RGB_VALIDATOR), doraise=True)

    def test_default_is_disarmed_validate_only(self):
        self.assertIn('VALIDATE_ONLY="${VALIDATE_ONLY:-true}"', self.wrapper)
        self.assertIn('ARM_MOTION="${ARM_MOTION:-NO}"', self.wrapper)
        self.assertIn('CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"', self.wrapper)
        self.assertIn('CONFIRM_NEW_SPEED_PROFILE="${CONFIRM_NEW_SPEED_PROFILE:-NO}"', self.wrapper)
        self.assertIn('[[ "${ARM_MOTION}" == "YES" ]]', self.wrapper)
        self.assertIn('[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]]', self.wrapper)
        self.assertIn('[[ "${CONFIRM_NEW_SPEED_PROFILE}" == "YES" ]]', self.wrapper)

    def test_literal_abba_order(self):
        expected = (
            "01) BLOCK=01; POSITION=01; CONDITION=B0",
            "02) BLOCK=01; POSITION=02; CONDITION=Bslosh",
            "03) BLOCK=02; POSITION=01; CONDITION=Bslosh",
            "04) BLOCK=02; POSITION=02; CONDITION=B0",
        )
        positions = [self.wrapper.index(item) for item in expected]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("row_order=B0,Bslosh,Bslosh,B0", self.wrapper)

    def test_exact_runtime_contract_is_frozen(self):
        required = (
            "V_REF=0.20",
            "V_SAFE_MAX=0.25",
            "DELAY_PHASE_MODE=fixed_closed_loop",
            "DELAY_PHASE_LINEAR_DELAY_SEC=0.15",
            "DELAY_PHASE_ANGULAR_DELAY_SEC=0.22",
            "CURRENT_OBSERVER_SOURCE=processed_imu",
            "OBSERVER_FALLBACK_POLICY=fail_closed",
            "OBSERVER_LATCH_FALLBACK=false",
            "STATE_TIMING_REQUIRE_COMMON_EPOCH=true",
            "SHARED_LINEAR_ACCEL_LIMIT_ENABLE=false",
            "SHARED_ANGULAR_LIMIT_ENABLE=false",
            "EXECUTION_CONTRACT_FAIL_CLOSED=true",
            "SPEED_SAFETY_ENABLE=true",
            "LIQUID_NOWCAST_PUBLISH_COMPARISON=true",
            "START_GATE_TIMEOUT_SEC=120",
            "IMU_SHADOW_READY_TIMEOUT_SEC=20",
        )
        for item in required:
            self.assertIn(item, self.wrapper)
        self.assertIn("run_spmpc_real_fixed_path_trial.sh", self.wrapper)
        self.assertIn("continuous_mpcc_acados", self.wrapper)

    def test_frozen_c02_artifacts_and_image_free_evidence(self):
        required = (
            "mocap_compact_s_C02.json",
            "1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164",
            "map_carto_20260829_mocap_exec_v1.pbstream",
            "34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595",
            "red_3ruler_g2s_20260731_relabel_frozen_v2.yaml",
            "7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01",
            "RECORD_MOCAP=true",
            "RECORD_MOCAP_PATH=false",
            "RECORD_ONLINE_LIQUID=true",
            "FORBID_IMAGE_STREAMS=true",
            "RECORD_CAMERA_INFO=true",
        )
        for item in required:
            self.assertIn(item, self.wrapper)

    def test_i0_to_l22_semantics_are_not_mislabeled(self):
        self.assertIn("OBSERVER_APPLIED=none", self.wrapper)
        self.assertIn("OBSERVER_APPLIED=L22", self.wrapper)
        self.assertIn("selected/raw liquid observer is processed-IMU (I0)", self.validator)
        self.assertIn("fixed_closed_loop", self.validator)
        self.assertIn("L22_command_history_rollout", self.validator)
        self.assertIn("solver_consumes_selected_state", self.validator)

    def test_postflights_cover_rgb_nokov_observer_and_fixed_application(self):
        required_wrapper_tokens = (
            "validate_i0_failclosed_fixed_abba_bag.py",
            "validate_slosh_nowcast_shadow_bag.py",
            "validate_g3_online_rgb_trial.py",
            "analyze_i0_failclosed_fixed_abba_rgb.py",
            "validate_mocap_execution_chain_bag.py",
            "summarize_spmpc_real_trial.py",
            "--require-robot-delay-compensation-applied true",
            "--require-liquid-delay-compensation-applied true",
            "--require-state-diagnostics",
            "--minimum-application-fraction 1.0",
            "--min-online-valid-fraction 0.98",
            "--max-zero-window-spread-mm 0.25",
            "--initial-stability-sec 5.0",
            "--min-initial-stability-valid-fraction 0.98",
            "--max-initial-h-vis-p95-mm 0.25",
            "--max-initial-abs-height-p95-mm 0.25",
            "--max-initial-half-median-drift-mm 0.05",
        )
        for item in required_wrapper_tokens:
            self.assertIn(item, self.wrapper)
        required_validator_tokens = (
            '"solver_backend_code": 1.0',
            '"delay_phase_mode_code": 3.0',
            '"smooth_priority_enable": 0.0',
            '"slosh_constraint_enable": 0.0',
            '"history_complete"',
            '"fixed_closed_loop_applied"',
            '"robot_delay_compensation_applied"',
            '"liquid_delay_compensation_applied"',
            '"DELAY_PREDICTED_COMMON_EPOCH"',
            '"GOAL_REACHED was not recorded"',
            '"processed-IMU/fail-closed selection mismatch',
            'WARM_START_TOPIC = "/spmpc/debug/warm_start"',
            'default=1.0',
            '"minimum application fraction must be exactly 1.0 for this protocol"',
            '"application_unreadable_counts"',
            '"warm-start fallback used during motion count={}"',
            '"warm_start_fallback_count"',
            '"warm_start_unreadable_count"',
        )
        for item in required_validator_tokens:
            self.assertIn(item, self.validator)

    def test_rgb_analysis_is_a_fail_closed_block_transition(self):
        required_tokens = (
            'RGB_ANALYSIS_REPORT="${RUN_OUT_DIR}/I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS.json"',
            '--report "${RGB_ANALYSIS_REPORT}" --protocol "${PROTOCOL_ID}"',
            "--maximum-slowdown-ratio 1.05",
            'return 10',
            'exit 10',
            'BLOCK1_RAPID_SCREEN PASS PROMOTE_BLOCK2 01,02',
            'BLOCK1_RAPID_SCREEN PROMOTE_BLOCK2 STOP_BLOCK1_FUTILITY 01,02',
            'COMPLETE_ABBA DEVELOPMENT_POSITIVE NO_DEVELOPMENT_POSITIVE,RGB_POSITIVE_SLOWDOWN_CONFOUNDED 01,02,03,04',
        )
        for item in required_tokens:
            self.assertIn(item, self.wrapper)

        marker = self.wrapper.index('> "${UNIT_PASS}"')
        block1_analysis = self.wrapper.index(
            'BLOCK1_RAPID_SCREEN PROMOTE_BLOCK2 STOP_BLOCK1_FUTILITY 01,02',
            marker,
        )
        final_analysis = self.wrapper.index(
            'COMPLETE_ABBA DEVELOPMENT_POSITIVE NO_DEVELOPMENT_POSITIVE,RGB_POSITIVE_SLOWDOWN_CONFOUNDED 01,02,03,04',
            marker,
        )
        self.assertLess(marker, block1_analysis)
        self.assertLess(marker, final_analysis)

        row3_guard = self.wrapper.index('if [[ "${PAIR_ROW}" == "03" ]]')
        promotion_check = self.wrapper.index(
            'BLOCK1_RAPID_SCREEN PASS PROMOTE_BLOCK2 01,02', row3_guard
        )
        motion_runner = self.wrapper.index('bash "${RUNNER}"', promotion_check)
        self.assertLess(row3_guard, promotion_check)
        self.assertLess(promotion_check, motion_runner)


if __name__ == "__main__":
    unittest.main()
