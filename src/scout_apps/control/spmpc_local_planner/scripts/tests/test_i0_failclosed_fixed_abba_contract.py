#!/usr/bin/env python3
"""Contracts for the shared I0/fail-closed ABBA engine and profiles."""

import py_compile
import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPT_DIR = Path(__file__).resolve().parent
PLANNER_SCRIPTS = SCRIPT_DIR.parent
ANALYSIS_DIR = PLANNER_SCRIPTS / "analysis"
LEGACY_WRAPPER = PLANNER_SCRIPTS / "run_spmpc_i0_failclosed_fixed_abba_trial.sh"
SHORT100_WRAPPER = (
    PLANNER_SCRIPTS / "run_spmpc_i0_failclosed_fixed_short100_abba_trial.sh"
)
EXPLICIT_WRAPPER = (
    PLANNER_SCRIPTS
    / "run_spmpc_i0_failclosed_explicit_actuator_abba_trial.sh"
)
ENGINE = (
    PLANNER_SCRIPTS
    / "lib"
    / "run_spmpc_i0_failclosed_fixed_abba_engine.sh"
)
PROFILE_TOOL = ANALYSIS_DIR / "i0_failclosed_fixed_abba_profile.py"
WINDOW_CONTRACT = ANALYSIS_DIR / "liquid_cost_window_contract.py"
VALIDATOR = ANALYSIS_DIR / "validate_i0_failclosed_fixed_abba_bag.py"
ANALYZER = ANALYSIS_DIR / "analyze_i0_failclosed_fixed_abba_rgb.py"
RGB_VALIDATOR = ANALYSIS_DIR / "validate_g3_online_rgb_trial.py"

sys.path.insert(0, str(ANALYSIS_DIR))
import i0_failclosed_fixed_abba_profile as profiles  # noqa: E402
import liquid_cost_window_contract as window  # noqa: E402


class I0FailClosedFixedAbbaContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.legacy_wrapper = LEGACY_WRAPPER.read_text(encoding="utf-8")
        cls.short100_wrapper = SHORT100_WRAPPER.read_text(encoding="utf-8")
        cls.explicit_wrapper = EXPLICIT_WRAPPER.read_text(encoding="utf-8")
        cls.engine = ENGINE.read_text(encoding="utf-8")
        cls.validator = VALIDATOR.read_text(encoding="utf-8")

    def test_shell_and_python_syntax(self):
        for script in (
            LEGACY_WRAPPER,
            SHORT100_WRAPPER,
            EXPLICIT_WRAPPER,
            ENGINE,
        ):
            subprocess.run(["bash", "-n", str(script)], check=True)
        for script in (
            PROFILE_TOOL,
            WINDOW_CONTRACT,
            VALIDATOR,
            ANALYZER,
            RGB_VALIDATOR,
        ):
            py_compile.compile(str(script), doraise=True)

    def test_thin_wrappers_select_frozen_profiles(self):
        self.assertIn("I0FC_ABBA_PROFILE=legacy_v1", self.legacy_wrapper)
        self.assertIn("I0FC_ABBA_PROFILE=short100_v2", self.short100_wrapper)
        self.assertIn(
            "I0FC_ABBA_PROFILE=explicit_actuator_v1",
            self.explicit_wrapper,
        )
        shared_engine = "lib/run_spmpc_i0_failclosed_fixed_abba_engine.sh"
        self.assertIn(shared_engine, self.legacy_wrapper)
        self.assertIn(shared_engine, self.short100_wrapper)
        self.assertIn(shared_engine, self.explicit_wrapper)
        self.assertLess(len(self.legacy_wrapper.splitlines()), 20)
        self.assertLess(len(self.short100_wrapper.splitlines()), 20)
        self.assertLess(len(self.explicit_wrapper.splitlines()), 20)

    def test_explicit_actuator_profile_is_isolated_and_disables_l22(self):
        profile = profiles.get_profile("explicit_actuator_v1")
        self.assertEqual(
            profile.protocol_id,
            "SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_ABBA_DEV_V1",
        )
        self.assertEqual(profile.treatment_variant, "B_slosh")
        self.assertEqual(profile.execution_model_mode, "explicit_actuator")
        self.assertEqual(profile.delay_phase_mode, "off")
        self.assertFalse(profile.require_legacy_delay_application)
        self.assertEqual(profile.expected_execution_model_code, 1)
        self.assertEqual(
            (profile.expected_b0_state_width, profile.expected_slosh_state_width),
            (23, 27),
        )
        rows = list(profiles.iter_rows(profile))
        self.assertEqual(
            [row.variant for row in rows],
            ["B0", "B_slosh", "B_slosh", "B0"],
        )
        self.assertEqual(rows[1].observer_applied, "I0")

    def test_legacy_profile_is_a_golden_copy_of_v1_identity(self):
        profile = profiles.get_profile("legacy_v1")
        self.assertEqual(profile.protocol_id, "SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1")
        self.assertEqual(profile.output_tag, "spmpc_i0_failclosed_fixed_abba")
        self.assertEqual(profile.treatment_variant, "B_slosh")
        self.assertEqual(profile.runner_selector_mode, "pilot_method")
        expected = {
            "01": ("B0", "B0", "B0", ""),
            "02": ("Bslosh", "W5", "B_slosh", "DEV_I0FC_FIXED_01_B0_b01_p01_a01"),
            "03": ("Bslosh", "W5", "B_slosh", "DEV_I0FC_FIXED_02_Bslosh_b01_p02_a01"),
            "04": ("B0", "B0", "B0", "DEV_I0FC_FIXED_03_Bslosh_b02_p01_a01"),
        }
        for row_id, values in expected.items():
            row = profiles.resolve_row(profile, row_id)
            self.assertEqual(
                (row.condition, row.pilot_method, row.variant), values[:3]
            )
            self.assertEqual(profiles.previous_run_label(profile, row), values[3])

    def test_short100_profile_has_independent_identity_and_direct_variant(self):
        profile = profiles.get_profile("short100_v2")
        self.assertEqual(
            profile.protocol_id,
            "SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2",
        )
        self.assertEqual(profile.treatment_variant, "B_slosh_short100")
        self.assertEqual(profile.treatment_cost_horizon_steps, 3)
        self.assertEqual(profile.treatment_cost_tail_discount, 0.0)
        self.assertEqual(profile.runner_selector_mode, "direct_variant")
        self.assertTrue(profile.require_fresh_session)
        self.assertNotEqual(
            profile.rgb_report_suffix,
            profiles.get_profile("legacy_v1").rgb_report_suffix,
        )
        rows = list(profiles.iter_rows(profile))
        self.assertEqual(
            [row.condition for row in rows],
            ["B0", "Bslosh", "Bslosh", "B0"],
        )
        self.assertTrue(all(row.pilot_method == "" for row in rows))
        self.assertEqual(rows[1].variant, "B_slosh_short100")
        self.assertEqual(rows[1].cost_horizon_steps, 3)
        self.assertEqual(rows[1].cost_tail_discount, 0.0)
        self.assertEqual(rows[0].variant, "B0")
        self.assertEqual(rows[0].cost_horizon_steps, -1)

    def test_engine_keeps_delay_speed_and_disarm_contract(self):
        required = (
            'VALIDATE_ONLY="${VALIDATE_ONLY:-true}"',
            'ARM_MOTION="${ARM_MOTION:-NO}"',
            'CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"',
            'CONFIRM_NEW_SPEED_PROFILE="${CONFIRM_NEW_SPEED_PROFILE:-NO}"',
            "V_REF=0.20",
            "V_SAFE_MAX=0.25",
            'DELAY_PHASE_MODE="${I0FC_DELAY_PHASE_MODE}"',
            'EXECUTION_MODEL_MODE="${I0FC_EXECUTION_MODEL_MODE}"',
            "CURRENT_OBSERVER_SOURCE=processed_imu",
            "OBSERVER_FALLBACK_POLICY=fail_closed",
            "STATE_TIMING_REQUIRE_COMMON_EPOCH=true",
            "SHARED_LINEAR_ACCEL_LIMIT_ENABLE=false",
            "SHARED_ANGULAR_LIMIT_ENABLE=false",
        )
        for token in required:
            self.assertIn(token, self.engine)
        for gate in (
            '[[ "${ARM_MOTION}" == "YES" ]]',
            '[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]]',
            '[[ "${CONFIRM_NEW_SPEED_PROFILE}" == "YES" ]]',
        ):
            self.assertIn(gate, self.engine)
        for frozen_runtime in (
            "OBSERVER_LATCH_FALLBACK=false",
            "LIQUID_NOWCAST_PUBLISH_COMPARISON=true",
            "EXECUTION_CONTRACT_FAIL_CLOSED=true",
            "SPEED_SAFETY_ENABLE=true",
            "START_GATE_TIMEOUT_SEC=120",
            "IMU_SHADOW_READY_TIMEOUT_SEC=20",
        ):
            self.assertIn(frozen_runtime, self.engine)

    def test_engine_preserves_frozen_assets_and_image_free_evidence(self):
        for token in (
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
        ):
            self.assertIn(token, self.engine)

    def test_i0_l22_and_explicit_semantics_have_postflight_contracts(self):
        for token in (
            'observer_applied="none"',
            'final_liquid_method="L22"',
            'final_liquid_method="I0"',
        ):
            self.assertIn(token, PROFILE_TOOL.read_text(encoding="utf-8"))
        for token in (
            "selected/raw liquid observer is processed-IMU (I0)",
            "fixed_closed_loop",
            "L22_command_history_rollout",
            "solver_consumes_selected_state",
        ):
            self.assertIn(token, self.validator)
        for token in (
            "validate_i0_failclosed_fixed_abba_bag.py",
            "validate_slosh_nowcast_shadow_bag.py",
            "validate_g3_online_rgb_trial.py",
            "analyze_i0_failclosed_fixed_abba_rgb.py",
            "validate_mocap_execution_chain_bag.py",
            "summarize_spmpc_real_trial.py",
            '--require-robot-delay-compensation-applied "${REQUIRE_LEGACY_DELAY_APPLICATION}"',
            '--require-liquid-delay-compensation-applied "${REQUIRE_LEGACY_DELAY_APPLICATION}"',
            "--require-state-diagnostics",
            "--minimum-application-fraction 1.0",
            '--expected-execution-model-code "${EXPECTED_EXECUTION_MODEL_CODE}"',
            '--expected-state-width "${EXPECTED_STATE_WIDTH}"',
        ):
            self.assertIn(token, self.engine)

    def test_rgb_decision_remains_after_unit_marker_and_before_next_block(self):
        marker = self.engine.index('> "${UNIT_PASS}"')
        block1 = self.engine.index(
            "BLOCK1_RAPID_SCREEN PROMOTE_BLOCK2 STOP_BLOCK1_FUTILITY 01,02",
            marker,
        )
        final = self.engine.index(
            "COMPLETE_ABBA DEVELOPMENT_POSITIVE NO_DEVELOPMENT_POSITIVE,RGB_POSITIVE_SLOWDOWN_CONFOUNDED 01,02,03,04",
            marker,
        )
        self.assertLess(marker, block1)
        self.assertLess(marker, final)
        row3_guard = self.engine.index('if [[ "${PAIR_ROW}" == "03" ]]')
        promotion = self.engine.index(
            "BLOCK1_RAPID_SCREEN PASS PROMOTE_BLOCK2 01,02", row3_guard
        )
        motion = self.engine.index('bash "${RUNNER}"', promotion)
        self.assertLess(row3_guard, promotion)
        self.assertLess(promotion, motion)

    def test_short100_cannot_fall_back_through_w5_mapping(self):
        self.assertIn('direct_variant)', self.engine)
        self.assertIn(
            'runner_selector_env=("PILOT_METHOD=" "VARIANT=${VARIANT}" "ALG=${VARIANT}")',
            self.engine,
        )
        self.assertIn(
            'validate_launch_variant "${TREATMENT_VARIANT}"', self.engine
        )
        self.assertIn("slosh_cost_horizon_steps", self.engine)
        self.assertIn("slosh_cost_tail_discount", self.engine)

    def test_short100_postflight_activates_solver_artifact_deep_check(self):
        required = (
            '--expected-variant "${VARIANT}"',
            '--expected-slosh-cost-horizon-steps "${EXPECTED_COST_HORIZON_STEPS}"',
            '--expected-slosh-cost-tail-discount "${EXPECTED_COST_TAIL_DISCOUNT}"',
            "--expected-slosh-eta-dot-ratio 0.3",
            "--expected-robot-horizon-steps 60",
            "--expected-dt-sec 0.0333333333333333",
            "--expected-control-frequency-hz 30.0",
            '--expected-config "w_smooth=${I0FC_RUNTIME_W_SMOOTH}"',
            '--expected-config "w_alpha=${I0FC_RUNTIME_W_ALPHA}"',
            '--expected-config "w_du_a=${I0FC_RUNTIME_W_DU_A}"',
            '--expected-config "w_du_vs=${I0FC_RUNTIME_W_DU_VS}"',
            '--expected-config "slosh_height_max=${I0FC_RUNTIME_SLOSH_HEIGHT_MAX}"',
            '--expected-config "alpha_max=${I0FC_RUNTIME_ALPHA_MAX}"',
            '--report-suffix "${RGB_REPORT_SUFFIX}"',
            '--postflight-suffix "${RGB_REPORT_SUFFIX}"',
            '--report-type "${RGB_ANALYSIS_REPORT_TYPE}"',
        )
        for token in required:
            self.assertIn(token, self.engine)
        validator_tokens = (
            'SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"',
            'HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"',
            '"--expected-config"',
            "validate_snapshot_stage_weights",
            "validate_deep_cycle_coverage",
            "deep_cost_contract",
        )
        for token in validator_tokens:
            self.assertIn(token, self.validator)

    def test_stage_scale_boundary_and_legacy_full_horizon(self):
        self.assertEqual(window.stage_scale(3, 60, 3, 0.0), 1.0)
        self.assertEqual(window.stage_scale(4, 60, 3, 0.0), 0.0)
        self.assertEqual(window.stage_scale(60, 60, -1, 1.0), 1.0)
        self.assertEqual(window.stage_scale(61, 60, 3, 0.0), 0.0)

    def test_cycle_join_keeps_artifacts_older_than_command_stamp(self):
        audit = SimpleNamespace(cycle_id=11, solve_attempted=True)
        selection = SimpleNamespace(cycle_id=11)
        snapshot = SimpleNamespace(cycle_id=11, valid=True)
        horizon = SimpleNamespace(cycle_id=11, valid=True)
        coverage = window.validate_deep_cycle_coverage(
            [(10.0, audit)],
            [(9.7, selection)],
            [(9.8, snapshot)],
            [(9.8, horizon)],
        )
        self.assertEqual(coverage.audit_failures, ())
        self.assertEqual(coverage.selection_failures, ())
        self.assertEqual(coverage.snapshot_failures, ())
        self.assertEqual(coverage.horizon_failures, ())
        self.assertIs(coverage.selection_records[0][1], selection)
        self.assertIs(coverage.valid_snapshot_records[0][1], snapshot)
        self.assertIs(coverage.valid_horizon_records[0][1], horizon)

    def test_cycle_coverage_rejects_duplicate_and_missing_audits(self):
        duplicate = window.validate_deep_cycle_coverage(
            [
                (10.0, SimpleNamespace(cycle_id=20, solve_attempted=True)),
                (10.1, SimpleNamespace(cycle_id=20, solve_attempted=True)),
            ],
            [(9.9, SimpleNamespace(cycle_id=20))],
            [(9.9, SimpleNamespace(cycle_id=20, valid=True))],
            [(9.9, SimpleNamespace(cycle_id=20, valid=True))],
        )
        self.assertTrue(
            any("duplicate motion audit cycle IDs" in item for item in duplicate.audit_failures)
        )

        missing = window.validate_deep_cycle_coverage(
            [
                (10.0, SimpleNamespace(cycle_id=30, solve_attempted=True)),
                (10.2, SimpleNamespace(cycle_id=32, solve_attempted=True)),
            ],
            [
                (9.9, SimpleNamespace(cycle_id=30)),
                (10.1, SimpleNamespace(cycle_id=32)),
            ],
            [
                (9.9, SimpleNamespace(cycle_id=30, valid=True)),
                (10.1, SimpleNamespace(cycle_id=32, valid=True)),
            ],
            [
                (9.9, SimpleNamespace(cycle_id=30, valid=True)),
                (10.1, SimpleNamespace(cycle_id=32, valid=True)),
            ],
        )
        self.assertTrue(
            any("missing motion audit cycle IDs" in item for item in missing.audit_failures)
        )

    def test_cycle_coverage_rejects_duplicate_and_missing_artifacts(self):
        audits = [
            (10.0, SimpleNamespace(cycle_id=40, solve_attempted=True)),
            (10.1, SimpleNamespace(cycle_id=41, solve_attempted=True)),
        ]
        coverage = window.validate_deep_cycle_coverage(
            audits,
            [
                (9.8, SimpleNamespace(cycle_id=40)),
                (9.9, SimpleNamespace(cycle_id=40)),
            ],
            [
                (9.8, SimpleNamespace(cycle_id=40, valid=True)),
                (9.9, SimpleNamespace(cycle_id=40, valid=True)),
            ],
            [(9.8, SimpleNamespace(cycle_id=40, valid=True))],
        )
        self.assertTrue(
            any("cycle 40 observer selection count=2" in item for item in coverage.selection_failures)
        )
        self.assertTrue(
            any("cycle 41 observer selection count=0" in item for item in coverage.selection_failures)
        )
        self.assertTrue(
            any("cycle 40 snapshot count=2" in item for item in coverage.snapshot_failures)
        )
        self.assertTrue(
            any("cycle 41 snapshot count=0" in item for item in coverage.snapshot_failures)
        )
        self.assertTrue(
            any("cycle 41 horizon count=0" in item for item in coverage.horizon_failures)
        )

    def test_expected_config_items_use_resolved_effective_values(self):
        expected, failures = window.parse_expected_config_items(
            [
                "w_smooth=0.1",
                "w_alpha=0.1",
                "w_du_a=0.1",
                "w_du_vs=0.1",
                "alpha_max=1.2",
            ]
        )
        self.assertEqual(failures, [])
        self.assertEqual(expected["w_alpha"], 0.1)
        self.assertEqual(expected["alpha_max"], 1.2)
        config = dict(expected)
        self.assertEqual(
            window.validate_config_fields(config, "effective_config", expected),
            [],
        )
        config["w_du_a"] = -1.0
        self.assertTrue(
            any(
                "w_du_a=-1.0, expected 0.1" in item
                for item in window.validate_config_fields(
                    config, "effective_config", expected
                )
            )
        )

    def make_snapshot(self):
        names = ["w_slosh_eta", "w_slosh_eta_dot"]
        values = []
        for stage in range(61):
            values.extend(
                window.expected_stage_weights(stage, 60, 3, 0.0, 5.0, 0.3)
            )
        return SimpleNamespace(
            schema_version=2,
            variant="B_slosh_short100",
            slosh_enabled=True,
            dt=1.0 / 30.0,
            horizon_steps=60,
            slosh_cost_horizon_steps=3,
            slosh_cost_horizon_sec=0.1,
            slosh_cost_tail_discount=0.0,
            parameter_names=names,
            parameter_width=2,
            stage_parameters=values,
        )

    def test_snapshot_checks_both_liquid_weights_at_stage_three_four_boundary(self):
        snapshot = self.make_snapshot()
        failures = window.validate_snapshot_stage_weights(
            snapshot, "snapshot", 5.0, 0.3, 3, 0.0
        )
        self.assertEqual(failures, [])
        snapshot.stage_parameters[4 * 2] = 5.0
        snapshot.stage_parameters[4 * 2 + 1] = 1.5
        failures = window.validate_snapshot_stage_weights(
            snapshot, "snapshot", 5.0, 0.3, 3, 0.0
        )
        self.assertTrue(any("stage 4 w_slosh_eta=" in item for item in failures))
        self.assertTrue(
            any("stage 4 w_slosh_eta_dot=" in item for item in failures)
        )

    def test_snapshot_metadata_keeps_full_robot_horizon(self):
        snapshot = self.make_snapshot()
        self.assertEqual(
            window.validate_metadata(
                snapshot,
                "snapshot",
                "B_slosh_short100",
                True,
                3,
                0.0,
                60,
                1.0 / 30.0,
            ),
            [],
        )
        snapshot.horizon_steps = 3
        failures = window.validate_metadata(
            snapshot,
            "snapshot",
            "B_slosh_short100",
            True,
            3,
            0.0,
            60,
            1.0 / 30.0,
        )
        self.assertTrue(any("horizon_steps=3" in item for item in failures))

    def test_snapshot_metadata_rejects_self_consistent_150ms_window(self):
        snapshot = self.make_snapshot()
        snapshot.dt = 0.05
        snapshot.slosh_cost_horizon_sec = 0.15
        failures = window.validate_metadata(
            snapshot,
            "snapshot",
            "B_slosh_short100",
            True,
            3,
            0.0,
            60,
            1.0 / 30.0,
        )
        self.assertTrue(any("dt=0.05" in item for item in failures))


if __name__ == "__main__":
    unittest.main()
