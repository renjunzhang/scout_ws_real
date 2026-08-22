#!/usr/bin/env python3

import contextlib
import copy
import csv
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = PACKAGE_ROOT / "tools" / "simulation" / \
    "run_independent_plant_campaign.py"
CONFIG_PATH = PACKAGE_ROOT / "config" / "simulation" / \
    "phase_rejoin_development_v1.yaml"
FORMAL_CONFIG_PATH = PACKAGE_ROOT / "config" / "simulation" / \
    "phase_rejoin_formal_simulation_v1.yaml"
SESSION_TEMPLATE_PATH = PACKAGE_ROOT / "config" / "simulation" / \
    "phase_rejoin_session_template_v1.yaml"
PLANNER_PATH = PACKAGE_ROOT / "config" / "planner" / "common.yaml"

SPEC = importlib.util.spec_from_file_location(
    "run_independent_plant_campaign", TOOL_PATH)
CAMPAIGN = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(CAMPAIGN)


class IndependentPlantCampaignTest(unittest.TestCase):

    @staticmethod
    def write_yaml(path, value, executable=False):
        path.write_text(
            yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
        if executable:
            path.chmod(0o755)

    @staticmethod
    def reference(path):
        return {"path": str(path), "sha256": CAMPAIGN.sha256_file(path)}

    @classmethod
    def create_frozen_session(cls, root, status=None):
        root = Path(root)
        fit_seeds = [4101, 4102]
        tune_seeds = [4201, 4202]
        held_out_seeds = [4301, 4302]
        pilot_seeds = [5101, 5102]
        formal_seeds = [6101, 6102]
        recovery_hash = "a" * 64

        runner = root / "runner.sh"
        runner.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        runner.chmod(0o755)
        executable = root / "closed_loop_runner"
        executable.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        executable.chmod(0o755)
        analysis = root / "analyze.sh"
        analysis.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        analysis.chmod(0o755)
        formal_order = root / "formal_order.csv"
        with formal_order.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("schema", "block", "seed", "position",
                             "condition"))
            orders = (
                ("C0", "C1", "IS", "C2", "C4", "C3"),
                ("C1", "C2", "C0", "C3", "IS", "C4"),
            )
            for block, seed in enumerate(formal_seeds, start=1):
                for position, condition in enumerate(
                        orders[(block - 1) % len(orders)], start=1):
                    writer.writerow((
                        "spmpc_phase_rejoin_formal_order_v1", block, seed,
                        position, condition))
        offline_plan_generator = root / "generate_offline_slosh_ocp_plan.py"
        offline_plan_generator.write_text(
            "#!/usr/bin/env python3\nraise SystemExit(0)\n", encoding="utf-8")
        offline_plan_generator.chmod(0o755)
        nominal_builder = root / "spmpc_build_formal_phase_rejoin_nominal"
        nominal_builder.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        nominal_builder.chmod(0o755)
        artifact_validator = root / "spmpc_phase_rejoin_artifact_tool"
        artifact_validator.write_text(
            "#!/usr/bin/env python3\n"
            "import pathlib, sys\n"
            "ok = len(sys.argv) == 4 and sys.argv[1] == 'validate' "
            "and sys.argv[2] == '--artifact'\n"
            "text = pathlib.Path(sys.argv[3]).read_text() if ok else ''\n"
            "rows = [line for line in text.splitlines() "
            "if line and not line.startswith('#')]\n"
            "ok = ok and len(rows) >= 13 and "
            "'# terminal_contract=publish_zero_settle_hold_v2' in text\n"
            "if not ok:\n"
            "    print('ERROR: invalid synthetic V3', file=sys.stderr)\n"
            "    raise SystemExit(2)\n"
            "print('VALID artifact: schema=phase_rejoin_empirical_augmented_v3 "
            "evidence_level=empirical_held_out "
            "source=simulation_offline_slosh_ocp_held_out_recovery "
            "contract_id=test recovery_artifact_hash={}')\n".format(
                recovery_hash),
            encoding="utf-8")
        artifact_validator.chmod(0o755)

        plant = copy.deepcopy(CAMPAIGN.load_yaml(CONFIG_PATH))
        plant["freeze_id"] = "SIM_PLANT_FORMAL_TEST_V1"
        plant["status"] = CAMPAIGN.FORMAL_STATUS
        plant["provenance"]["runtime_verified"] = True
        plant["provenance"]["identification_known_bias"] = True
        plant["provenance"]["source_limitations_acknowledged"] = True
        plant["provenance"]["physical_parameter_claim"] = False
        plant["scope"]["physical_parameter_claim"] = False
        plant["experiment"]["reserved_formal_seeds"] = formal_seeds
        plant_path = root / "plant.yaml"
        cls.write_yaml(plant_path, plant)

        path_definition = {
            "frame_id": "map",
            "poses": [
                {"x": 0.0, "y": 0.0, "qx": 0.0, "qy": 0.0,
                 "qz": 0.0, "qw": 1.0},
                {"x": 1.0, "y": 0.0, "qx": 0.0, "qy": 0.0,
                 "qz": 0.0, "qw": 1.0},
            ],
        }
        path_path = root / "path.json"
        path_path.write_text(json.dumps(path_definition), encoding="utf-8")

        offline_plan = root / "offline_plan.csv"
        plan_lines = [
            "# schema=spmpc_offline_slosh_ocp_plan_v1",
            "# status=PASS",
            "# simulation_only=true",
            "# formal_robot_release=false",
            "# path_sha256={}".format(CAMPAIGN.sha256_file(path_path)),
            "# path_frame_id=map",
            "# path_length=1",
            "# execution_contract_hash={}".format("b" * 64),
            "# dt=0.033333333333333333",
            "# zero_hold_steps=11",
            "index,t,u_pub_v,u_pub_omega,v_s",
        ]
        plan_lines.extend(
            "{},{},0,0,0".format(index, index / 30.0)
            for index in range(20))
        offline_plan.write_text("\n".join(plan_lines) + "\n", encoding="utf-8")
        offline_plan_report = root / "offline_plan_report.json"
        offline_plan_report.write_text(json.dumps({
            "schema": "spmpc_offline_slosh_ocp_report_v1",
            "status": "PASS",
            "simulation_only": True,
            "formal_robot_release": False,
            "optimizer": {"success": True},
            "terminal_contract": {
                "name": "publish_zero_settle_hold_v2",
                "zero_hold_steps": 11,
                "goal_yaw_preserved_from_path_quaternion": True,
            },
            "plan": {
                "sha256": CAMPAIGN.sha256_file(offline_plan),
                "rows": 20,
            },
        }), encoding="utf-8")

        artifact = root / "phase_rejoin.csv"
        metadata = {
            "schema": "phase_rejoin_empirical_augmented_v3",
            "evidence_level": "empirical_held_out",
            "source": "simulation_offline_slosh_ocp_held_out_recovery",
            "contract_id": "unit_test_v3",
            "frame_id": "map",
            "dt": "0.033333333333333333",
            "path_length": "1",
            "artifact_role": "simulation_phase_rejoin_nominal_and_recovery",
            "nominal_sequence_kind":
                "offline_slosh_ocp_complete_augmented_tail",
            "offline_slosh_ocp": "true",
            "hardware_formal_release": "false",
            "terminal_contract": "publish_zero_settle_hold_v2",
            "recovery_contract": "nominal_command_v1",
            "terminal_zero_hold_steps": "11",
            "terminal_eta_norm_max": "0.001",
            "terminal_eta_dot_norm_max": "0.001",
            "terminal_v_abs_max": "0.001",
            "terminal_omega_abs_max": "0.001",
            "terminal_linear_actuator_output_abs_max": "0.001",
            "terminal_angular_actuator_output_abs_max": "0.001",
            "terminal_linear_pending_command_abs_max": "1e-9",
            "terminal_angular_pending_command_abs_max": "1e-9",
            "two_zeta_omega_n": "0.2",
            "omega_n_sq": "4",
            "kappa_x": "1",
            "kappa_y": "1",
            "dynamics_tolerance": "1e-9",
            "execution_contract_id": "unit_test_execution",
            "execution_contract_hash": "b" * 64,
            "execution_state_width": "22",
            "execution_linear_buffer_count": "5",
            "execution_angular_buffer_count": "7",
            "parameter_schema_version": "2",
            "parameter_schema_id": "unit_test_parameters",
            "parameter_schema_hash": "c" * 64,
            "recovery_artifact_hash": recovery_hash,
            "execution_compatibility_contract":
                "phase_indexed_execution_box_v1",
        }
        artifact_lines = [
            "# {}={}".format(key, value) for key, value in metadata.items()
        ]
        artifact_header = (
            "index,t,s,x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot,"
            "a,alpha,v_s,u_pub_v,u_pub_omega,kappa_v,kappa_omega,"
            "r_x,r_y,r_yaw,r_v,r_omega,r_eta_x,r_eta_x_dot,r_eta_y,"
            "r_eta_y_dot,exec_linear_output,exec_angular_output,"
            "exec_linear_pending,exec_angular_pending,"
            "exec_beta_linear_output,exec_beta_angular_output,"
            "exec_beta_linear_pending,exec_beta_angular_pending")
        artifact_lines.append(artifact_header)
        for index in range(20):
            base_values = [index, index / 30.0, 1.0] + [0.0] * 16 + \
                [1.0] * 9
            artifact_lines.append(",".join(
                [str(value) for value in base_values] +
                ["0", "0", "0;0;0;0;0", "0;0;0;0;0;0;0", "1", "1",
                 "1;1;1;1;1", "1;1;1;1;1;1;1"]))
        artifact.write_text("\n".join(artifact_lines) + "\n", encoding="utf-8")

        recovery_dataset = root / "recovery_dataset.csv"
        dataset_stream = io.StringIO(newline="")
        dataset_writer = csv.writer(dataset_stream, lineterminator="\n")
        dataset_writer.writerow(CAMPAIGN.RECOVERY_DATASET_COLUMNS)
        split_data = (
            ("fit", fit_seeds), ("tune", tune_seeds),
            ("held_out", held_out_seeds),
        )
        for split, seeds in split_data:
            for index, seed in enumerate(seeds):
                for phase_index in range(20):
                    dataset_writer.writerow(
                        [split, "{}_{}".format(split, seed), seed,
                         phase_index, 1 if index == 0 else 0] +
                        [0.0] *
                        (len(CAMPAIGN.RECOVERY_DATASET_COLUMNS) - 5))
        recovery_dataset.write_text(
            dataset_stream.getvalue(), encoding="utf-8")

        recovery_radii = root / "phase_rejoin_recovery_radii_bounds.csv"
        radii_stream = io.StringIO(newline="")
        radii_writer = csv.writer(radii_stream, lineterminator="\n")
        radii_writer.writerow(CAMPAIGN.RECOVERY_RADII_COLUMNS)
        for phase_index in range(20):
            radii_writer.writerow(
                [phase_index, phase_index, phase_index, 0.5] +
                [1.0] * (len(CAMPAIGN.RECOVERY_RADII_COLUMNS) - 4))
        recovery_radii.write_text(radii_stream.getvalue(), encoding="utf-8")

        held_out_report = root / "held_out_report.json"
        held_out_report.write_text(json.dumps({
            "schema": CAMPAIGN.RECOVERY_REPORT_SCHEMA,
            "status": "PASS",
            "safety_certificate": False,
            "formal_robot_release": False,
            "held_out_evaluation_count": 1,
            "held_out_influenced_fit": False,
            "held_out_influenced_tuning": False,
            "input_sha256": CAMPAIGN.sha256_file(recovery_dataset),
            "evaluation": {"global": {}, "per_phase_bin": {}},
        }), encoding="utf-8")

        split_manifest = {}
        for index, (split, seeds) in enumerate(split_data):
            split_manifest[split] = {
                "row_count": 20 * len(seeds),
                "rollout_count": len(seeds),
                "rollout_ids": ["{}_{}".format(split, seed) for seed in seeds],
                "seeds": seeds,
                "phase_indices": list(range(20)),
                "recovered_count": 20,
                "unrecovered_count": 20,
                "canonical_rows_sha256": str(index + 1) * 64,
            }
        recovery_manifest = root / "manifest.json"
        recovery_manifest.write_text(json.dumps({
            "schema": CAMPAIGN.RECOVERY_MANIFEST_SCHEMA,
            "status": "EMPIRICAL_HELD_OUT_PASS",
            "safety_certificate": False,
            "formal_robot_release": False,
            "physical_enforce_authorized": False,
            "input": {
                "path": str(recovery_dataset),
                "sha256": CAMPAIGN.sha256_file(recovery_dataset),
                "schema": CAMPAIGN.RECOVERY_DATASET_SCHEMA,
                "columns": list(CAMPAIGN.RECOVERY_DATASET_COLUMNS),
            },
            "split_contract": dict({
                "unit": "complete_rollout_and_seed",
                "mutually_exclusive": True,
            }, **split_manifest),
            "fit": {"uses_only_split": "fit"},
            "tune": {"uses_only_split": "tune"},
            "held_out": {
                "uses_only_split": "held_out",
                "evaluation_count": 1,
                "status": "PASS",
            },
            "outputs": {
                "scales": {
                    "path": recovery_radii.name,
                    "schema": CAMPAIGN.RECOVERY_RADII_SCHEMA,
                    "sha256": CAMPAIGN.sha256_file(recovery_radii),
                },
                "held_out_report": {
                    "path": held_out_report.name,
                    "schema": CAMPAIGN.RECOVERY_REPORT_SCHEMA,
                    "sha256": CAMPAIGN.sha256_file(held_out_report),
                },
            },
        }), encoding="utf-8")

        artifact_body = [
            line for line in artifact_lines if not line.startswith("#")]
        metadata.update({
            "path_sha256": CAMPAIGN.sha256_file(path_path),
            "offline_plan_sha256": CAMPAIGN.sha256_file(offline_plan),
            "offline_plan_report_sha256":
                CAMPAIGN.sha256_file(offline_plan_report),
            "recovery_dataset_sha256":
                CAMPAIGN.sha256_file(recovery_dataset),
            "recovery_scales_sha256": CAMPAIGN.sha256_file(recovery_radii),
            "recovery_fit_manifest_sha256":
                CAMPAIGN.sha256_file(recovery_manifest),
            "recovery_held_out_report_sha256":
                CAMPAIGN.sha256_file(held_out_report),
        })
        artifact.write_text(
            "\n".join(
                ["# {}={}".format(key, value)
                 for key, value in metadata.items()] + artifact_body) + "\n",
            encoding="utf-8")

        nominal_report = root / "nominal_report.json"
        nominal_report.write_text(json.dumps({
            "schema": CAMPAIGN.NOMINAL_REPORT_SCHEMA,
            "status": "PASS",
            "simulation_only": True,
            "formal_robot_release": False,
            "production_v3_loader_passed": True,
            "artifact": {
                "path": str(artifact),
                "sha256": CAMPAIGN.sha256_file(artifact),
                "recovery_artifact_hash": recovery_hash,
                "rows": 20,
            },
            "path_sha256": CAMPAIGN.sha256_file(path_path),
            "offline_plan_sha256": CAMPAIGN.sha256_file(offline_plan),
            "offline_plan_report_sha256":
                CAMPAIGN.sha256_file(offline_plan_report),
            "recovery_fit_manifest_sha256":
                CAMPAIGN.sha256_file(recovery_manifest),
            "recovery_held_out_report_sha256":
                CAMPAIGN.sha256_file(held_out_report),
            "terminal_contract": "publish_zero_settle_hold_v2",
            "terminal_zero_hold_steps": 11,
        }), encoding="utf-8")

        planner = copy.deepcopy(CAMPAIGN.load_yaml(PLANNER_PATH))
        planner["delay_augmented_phase"]["enabled"] = True
        planner["delay_augmented_phase"][
            "expected_recovery_artifact_hash"] = recovery_hash
        planner["phase_rejoin"]["mode"] = "enforce"
        planner["phase_rejoin"]["artifact_path"] = str(artifact)
        planner["publish_timing"]["enabled"] = True
        planner["publish_timing"]["estimated_dc_sec"] = 0.01
        planner_path = root / "planner.yaml"
        cls.write_yaml(planner_path, planner)

        conditions = {}
        manifest_conditions = {}
        for name in CAMPAIGN.REQUIRED_CONDITIONS:
            implementation_id = "closed_loop_{}_v1".format(name.lower())
            config_path = root / "condition_{}.yaml".format(name.lower())
            cls.write_yaml(config_path, {
                "schema": "spmpc_simulation_condition_v1",
                "condition": name,
                "implementation_id": implementation_id,
            })
            binding = {
                "binding_id": "formal_{}".format(name.lower()),
                "implementation_id": implementation_id,
                "config": cls.reference(config_path),
            }
            binding.update(CAMPAIGN.CONDITION_SEMANTICS[name])
            if name == "C1":
                binding.update({
                    "pilot_tuned_and_frozen": True,
                    "global_time_scale": 0.95,
                })
            if name == "IS":
                binding.update({
                    "shaper": "ZVD",
                    "single_mode_residual_test_passed": True,
                })
            conditions[name] = binding
            manifest_binding = {
                "implementation_id": implementation_id,
            }
            manifest_binding.update(CAMPAIGN.CONDITION_SEMANTICS[name])
            manifest_conditions[name] = manifest_binding

        git_commit = "9" * 40
        controller_manifest = root / "controller_manifest.json"
        controller_manifest.write_text(json.dumps({
            "schema": CAMPAIGN.CONTROLLER_MANIFEST_SCHEMA,
            "git_commit": git_commit,
            "source_tree_clean": True,
            "runner_sha256": CAMPAIGN.sha256_file(runner),
            "executable_sha256": CAMPAIGN.sha256_file(executable),
            "artifact_validator_sha256":
                CAMPAIGN.sha256_file(artifact_validator),
            "solver": {
                "backend": "delay_augmented_phase_acados",
                "state_width": 22,
                "control_width": 3,
                "horizon_steps": 10,
                "execution_contract_hash": "b" * 64,
                "parameter_schema_hash": "c" * 64,
            },
            "conditions": manifest_conditions,
        }), encoding="utf-8")

        session = {
            "schema": CAMPAIGN.FORMAL_SESSION_SCHEMA,
            "freeze_id": "SIM_FORMAL_TEST_V1",
            "status": status or CAMPAIGN.FORMAL_STATUS,
            "formal_trials_started": False,
            "scope": {
                "simulation_only": True,
                "formal_robot_release": False,
                "real_robot_enforce_allowed": False,
                "plant_truth_visible_to_controller": False,
                "physical_parameter_claim": False,
            },
            "runtime": {
                "control_rate_hz": 30.0,
                "final_command_transaction": True,
                "command_history_source": "final_published_command",
                "observer_input": "plant_motion_derived_odom",
                "external_liquid_truth_used_for_control": False,
                "dual_channel_execution_model": True,
                "expected_publish_time_model": True,
                "git_commit": git_commit,
                "source_tree_clean": True,
                "compiler_id": "gcc-test",
                "stl_id": "libstdc++-test",
                "floating_point_contract": "ieee754-double-test",
                "runner": cls.reference(runner),
                "executable": cls.reference(executable),
                "controller_manifest": cls.reference(controller_manifest),
                "artifact_validator": cls.reference(artifact_validator),
            },
            "assets": {
                "plant_config": cls.reference(plant_path),
                "planner_config": cls.reference(planner_path),
                "path": cls.reference(path_path),
                "offline_plan": cls.reference(offline_plan),
                "offline_plan_report": cls.reference(offline_plan_report),
                "offline_plan_generator": cls.reference(
                    offline_plan_generator),
                "nominal_builder": cls.reference(nominal_builder),
                "phase_rejoin_artifact": cls.reference(artifact),
                "nominal_validation_report": cls.reference(nominal_report),
                "recovery_dataset": cls.reference(recovery_dataset),
                "recovery_fit_manifest": cls.reference(recovery_manifest),
                "recovery_radii_bounds": cls.reference(recovery_radii),
                "recovery_held_out_report": cls.reference(held_out_report),
                "analysis_script": cls.reference(analysis),
                "formal_order": cls.reference(formal_order),
            },
            "path_contract": {"frame_id": "map", "goal_yaw_rad": 0.0},
            "measurement_contract": {
                "primary_metric": "external_height_q95_m",
                "window": "motion_plus_fixed_tail",
                "statistics_unit": "complete_trial",
                "physical_metric_alignment":
                    "rgb_max_lcr_motion_plus_fixed_tail",
                "controller_liquid_is_truth": False,
                "external_liquid_truth_used_for_control": False,
                "failures_included": True,
                "fixed_tail_sec": 4.0,
                "minimum_meaningful_difference_m": 0.0005,
                "paired_interval": "paired_bootstrap_95pct",
                "paired_estimator": "paired_mean",
                "bootstrap_replicates": 10000,
                "bootstrap_seed": 20260822,
                "formal_paired_blocks": 16,
                "primary_comparators": ["C0", "C1", "C3"],
                "task_noninferiority_comparators": ["C0", "C1"],
                "completion_time_noninferiority_relative": 0.10,
                "tracking_q95_noninferiority_m": 0.05,
                "failed_trial_rule": "retain_and_count_as_failure",
                "replacement_rule":
                    "infrastructure_failure_only_same_seed_condition",
            },
            "seeds": {
                "locked": True,
                "recovery_fit": fit_seeds,
                "recovery_tune": tune_seeds,
                "recovery_held_out": held_out_seeds,
                "pilot_trials": pilot_seeds,
                "formal_trials": formal_seeds,
            },
            "conditions": conditions,
        }
        session_path = root / "session.yaml"
        cls.write_yaml(session_path, session)
        return session_path, session

    def test_frozen_config_is_simulation_only(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        CAMPAIGN.require_simulation_development_config(config)
        self.assertEqual(config["freeze_id"], "SIM_EXEC_PLANAR_R03_DEV_V1")
        self.assertEqual(
            config["status"], "development_candidate_unbound")
        self.assertEqual(config["experiment"]["control_rate_hz"], 30.0)
        self.assertEqual(config["experiment"]["fixed_tail_sec"], 4.0)
        self.assertFalse(
            config["scope"]["real_robot_enforce_allowed"])
        self.assertEqual(len(config["experiment"]["pilot_seeds"]), 8)
        self.assertTrue(config["experiment"]["formal_seeds_locked"])
        self.assertFalse(config["provenance"]["runtime_verified"])
        self.assertTrue(
            config["provenance"]["identification_known_bias"])
        self.assertEqual(
            config["provenance"]["corrected_tool_schema"],
            "spmpc_planar_execution_identification_v2")
        self.assertNotIn("controller_internal_model", config)
        self.assertNotIn(
            "real_robot_launch_overrides", config["experiment"])

    def test_promoted_simulation_plant_is_not_physical_identification(self):
        config = CAMPAIGN.load_yaml(FORMAL_CONFIG_PATH)
        CAMPAIGN.require_simulation_audit_config(config)
        self.assertEqual(config["status"], CAMPAIGN.FORMAL_STATUS)
        self.assertTrue(config["provenance"]["runtime_verified"])
        self.assertTrue(config["provenance"]["identification_known_bias"])
        self.assertTrue(
            config["provenance"]["source_limitations_acknowledged"])
        self.assertFalse(config["provenance"]["physical_parameter_claim"])
        self.assertFalse(config["scope"]["physical_parameter_claim"])
        self.assertFalse(config["scope"]["result_claim_authorized"])

    def test_physical_authorization_is_rejected(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        config["scope"]["real_robot_enforce_allowed"] = True
        with self.assertRaisesRegex(ValueError, "physical use"):
            CAMPAIGN.require_simulation_development_config(config)

    def test_seed_types_duplicates_and_cross_group_overlap_are_rejected(self):
        base = CAMPAIGN.load_yaml(CONFIG_PATH)
        cases = []

        value = copy.deepcopy(base)
        value["experiment"]["smoke_seed"] = True
        cases.append((value, "invalid smoke seed"))

        value = copy.deepcopy(base)
        value["experiment"]["pilot_seeds"][0] = False
        cases.append((value, "invalid pilot seed"))

        value = copy.deepcopy(base)
        value["experiment"]["reserved_formal_seeds"][0] = True
        cases.append((value, "invalid formal seed"))

        value = copy.deepcopy(base)
        value["experiment"]["smoke_seed"] = 1 << 32
        cases.append((value, "invalid smoke seed"))

        value = copy.deepcopy(base)
        value["experiment"]["pilot_seeds"][0] = 1 << 32
        cases.append((value, "invalid pilot seed"))

        value = copy.deepcopy(base)
        value["experiment"]["reserved_formal_seeds"][0] = 1 << 32
        cases.append((value, "invalid formal seed"))

        value = copy.deepcopy(base)
        value["experiment"]["pilot_seeds"][1] = \
            value["experiment"]["pilot_seeds"][0]
        cases.append((value, "duplicate pilot seed"))

        for left, right in (("smoke", "pilot"),
                            ("smoke", "formal"),
                            ("pilot", "formal")):
            value = copy.deepcopy(base)
            experiment = value["experiment"]
            smoke = experiment["smoke_seed"]
            pilot = experiment["pilot_seeds"][0]
            formal = experiment["reserved_formal_seeds"][0]
            selected = {"smoke": smoke, "pilot": pilot, "formal": formal}
            if left == "smoke":
                experiment["smoke_seed"] = selected[right]
            elif left == "pilot":
                experiment["pilot_seeds"][0] = selected[right]
            cases.append((value, "seeds overlap"))

        for config, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(ValueError, message):
                    CAMPAIGN.require_simulation_development_config(config)

    def test_unlocked_formal_seeds_are_rejected(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        config["experiment"]["formal_seeds_locked"] = False
        with self.assertRaisesRegex(ValueError, "not locked"):
            CAMPAIGN.require_simulation_development_config(config)

    def test_smoke_timing_waits_for_zero_command_arrival_and_uses_ceil(self):
        config = CAMPAIGN.load_yaml(CONFIG_PATH)
        timing = CAMPAIGN.expected_smoke_timing(config)
        self.assertAlmostEqual(timing["zero_command_publish_sec"], 7.5)
        self.assertAlmostEqual(timing["tail_window_start_sec"], 7.606)
        self.assertAlmostEqual(timing["duration_sec"], 11.606)
        self.assertEqual(timing["samples"], 349)
        self.assertEqual(timing["tail_samples"], 121)

        changed = copy.deepcopy(config)
        changed["experiment"]["control_rate_hz"] = 17.0
        changed["experiment"]["fixed_tail_sec"] = 1.25
        changed_timing = CAMPAIGN.expected_smoke_timing(changed)
        self.assertAlmostEqual(
            changed_timing["zero_command_publish_sec"], 128.0 / 17.0)
        self.assertAlmostEqual(
            changed_timing["tail_window_start_sec"], 128.0 / 17.0 + 0.106)
        self.assertNotEqual(changed_timing["samples"], timing["samples"])
        self.assertAlmostEqual(
            changed_timing["tail_window_end_sec"] -
            changed_timing["tail_window_start_sec"], 1.25)

    def test_output_scope_is_restricted_to_identification_root(self):
        accepted = CAMPAIGN.ensure_output_scope(
            Path("/data/a/spmpc_exec_identification/unit_test"))
        self.assertEqual(
            accepted,
            Path("/data/a/spmpc_exec_identification/unit_test"))
        with self.assertRaisesRegex(ValueError, "must remain"):
            CAMPAIGN.ensure_output_scope(Path("/media/a/ZRJ/forbidden"))

    def test_failed_executable_preflight_does_not_consume_output_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "not_consumed"
            with mock.patch.object(CAMPAIGN, "ALLOWED_OUTPUT_ROOT", root), \
                    contextlib.redirect_stderr(io.StringIO()):
                result = CAMPAIGN.main([
                    "--campaign", "smoke",
                    "--config", str(CONFIG_PATH),
                    "--executable", str(root / "missing_executable"),
                    "--out-dir", str(output),
                ])
            self.assertEqual(result, 2)
            self.assertFalse(output.exists())

    def test_existing_campaign_directory_is_not_reused(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "existing"
            output.mkdir()
            sentinel = output / "sentinel"
            sentinel.write_text("keep\n", encoding="utf-8")
            executable = root / "unused_executable"
            executable.write_text("#!/bin/sh\nexit 99\n", encoding="utf-8")
            executable.chmod(0o755)
            stderr = io.StringIO()
            with mock.patch.object(CAMPAIGN, "ALLOWED_OUTPUT_ROOT", root), \
                    contextlib.redirect_stderr(stderr):
                result = CAMPAIGN.main([
                    "--campaign", "smoke",
                    "--config", str(CONFIG_PATH),
                    "--executable", str(executable),
                    "--out-dir", str(output),
                ])
            self.assertEqual(result, 2)
            self.assertIn("already exists", stderr.getvalue())
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "keep\n")

    def test_pilot_candidate_cannot_become_formal_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            session_path, session = self.create_frozen_session(
                directory, CAMPAIGN.PILOT_STATUS)
            reasons = CAMPAIGN.formal_no_go_reasons(session, session_path)
            self.assertIn("session is only a pilot candidate", reasons)

    def test_formal_simulation_keeps_known_bias_without_physical_claim(self):
        with tempfile.TemporaryDirectory() as directory:
            session_path, session = self.create_frozen_session(directory)
            self.assertEqual(
                CAMPAIGN.formal_no_go_reasons(session, session_path), [])

            plant_path = Path(session["assets"]["plant_config"]["path"])
            plant = CAMPAIGN.load_yaml(plant_path)
            self.assertTrue(plant["provenance"]["identification_known_bias"])
            plant["provenance"]["source_limitations_acknowledged"] = False
            self.write_yaml(plant_path, plant)
            session["assets"]["plant_config"] = self.reference(plant_path)
            self.write_yaml(session_path, session)
            reasons = CAMPAIGN.formal_no_go_reasons(session, session_path)
            self.assertIn(
                "plant config is not a promoted formal simulation truth",
                reasons)

            plant["provenance"]["source_limitations_acknowledged"] = True
            plant["provenance"]["physical_parameter_claim"] = True
            self.write_yaml(plant_path, plant)
            session["assets"]["plant_config"] = self.reference(plant_path)
            self.write_yaml(session_path, session)
            reasons = CAMPAIGN.formal_no_go_reasons(session, session_path)
            self.assertIn(
                "plant config is not a promoted formal simulation truth",
                reasons)

    def test_formal_session_cannot_claim_physical_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            session_path, session = self.create_frozen_session(directory)
            session["scope"]["physical_parameter_claim"] = True
            self.write_yaml(session_path, session)
            with self.assertRaisesRegex(ValueError, "simulation-only scope"):
                CAMPAIGN.formal_no_go_reasons(session, session_path)

    def test_repository_session_template_is_explicit_no_go(self):
        session = CAMPAIGN.load_yaml(SESSION_TEMPLATE_PATH)
        reasons = CAMPAIGN.formal_no_go_reasons(
            session, SESSION_TEMPLATE_PATH)
        self.assertIn("session is only a pilot candidate", reasons)
        self.assertIn("C1 smooth-match pilot freeze is absent", reasons)
        self.assertTrue(any("path is absent" in reason for reason in reasons))

    def test_missing_file_and_fake_hash_cannot_become_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            session_path, session = self.create_frozen_session(directory)
            missing = Path(directory) / "missing.csv"
            session["assets"]["phase_rejoin_artifact"] = {
                "path": str(missing), "sha256": "a" * 64,
            }
            self.write_yaml(session_path, session)
            reasons = CAMPAIGN.formal_no_go_reasons(session, session_path)
            self.assertIn("phase rejoin artifact file does not exist", reasons)

            existing = Path(session["assets"]["path"]["path"])
            session["assets"]["path"] = {
                "path": str(existing), "sha256": "f" * 64,
            }
            self.write_yaml(session_path, session)
            reasons = CAMPAIGN.formal_no_go_reasons(session, session_path)
            self.assertIn("path SHA-256 mismatch", reasons)

    def test_seed_overlap_and_condition_semantic_drift_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            session_path, session = self.create_frozen_session(directory)
            session["seeds"]["recovery_tune"][0] = \
                session["seeds"]["recovery_fit"][0]
            session["conditions"]["C3"]["recovery_gate"] = True
            self.write_yaml(session_path, session)
            reasons = CAMPAIGN.formal_no_go_reasons(session, session_path)
            self.assertTrue(any("overlaps" in reason for reason in reasons))
            self.assertIn("C3 semantic recovery_gate is invalid", reasons)

    def test_two_row_self_hashed_v3_is_rejected_by_bound_validator(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_path, session = self.create_frozen_session(root)
            artifact = Path(session["assets"]["phase_rejoin_artifact"]["path"])
            lines = artifact.read_text(encoding="utf-8").splitlines()
            header_index = next(
                index for index, line in enumerate(lines)
                if not line.startswith("#"))
            artifact.write_text(
                "\n".join(lines[:header_index + 3]) + "\n", encoding="utf-8")
            session["assets"]["phase_rejoin_artifact"] = \
                self.reference(artifact)
            nominal_path = Path(
                session["assets"]["nominal_validation_report"]["path"])
            nominal = json.loads(nominal_path.read_text(encoding="utf-8"))
            nominal["artifact"]["sha256"] = CAMPAIGN.sha256_file(artifact)
            nominal_path.write_text(json.dumps(nominal), encoding="utf-8")
            session["assets"]["nominal_validation_report"] = \
                self.reference(nominal_path)
            self.write_yaml(session_path, session)
            reasons = CAMPAIGN.formal_no_go_reasons(session, session_path)
            self.assertTrue(any(
                reason.startswith("production V3 artifact validation failed")
                for reason in reasons))

    def test_v3_gate_values_must_equal_fitted_recovery_bounds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_path, session = self.create_frozen_session(root)
            artifact = Path(session["assets"]["phase_rejoin_artifact"]["path"])
            lines = artifact.read_text(encoding="utf-8").splitlines()
            header_index = next(
                index for index, line in enumerate(lines)
                if not line.startswith("#"))
            header = lines[header_index].split(",")
            first_row = lines[header_index + 1].split(",")
            first_row[header.index("r_x")] = "2.0"
            lines[header_index + 1] = ",".join(first_row)
            artifact.write_text("\n".join(lines) + "\n", encoding="utf-8")
            session["assets"]["phase_rejoin_artifact"] = \
                self.reference(artifact)
            nominal_path = Path(
                session["assets"]["nominal_validation_report"]["path"])
            nominal = json.loads(nominal_path.read_text(encoding="utf-8"))
            nominal["artifact"]["sha256"] = CAMPAIGN.sha256_file(artifact)
            nominal_path.write_text(json.dumps(nominal), encoding="utf-8")
            session["assets"]["nominal_validation_report"] = \
                self.reference(nominal_path)
            self.write_yaml(session_path, session)
            reasons = CAMPAIGN.formal_no_go_reasons(session, session_path)
            self.assertIn(
                "V3 artifact gate/B_exec differs from fitted bounds", reasons)

    def test_complete_formal_audit_reaches_ready_without_running_trials(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session_path, session = self.create_frozen_session(root)
            self.assertEqual(
                CAMPAIGN.formal_no_go_reasons(session, session_path), [])
            output = root / "formal_audit"
            stdout = io.StringIO()
            real_run = subprocess.run
            with mock.patch.object(CAMPAIGN, "ALLOWED_OUTPUT_ROOT", root), \
                    mock.patch.object(
                        CAMPAIGN.subprocess, "run",
                        side_effect=real_run) as run, \
                    contextlib.redirect_stdout(stdout):
                result = CAMPAIGN.main([
                    "--campaign", "formal",
                    "--session", str(session_path),
                    "--out-dir", str(output),
                ])
            self.assertEqual(result, 5)
            run.assert_called_once()
            self.assertEqual(
                run.call_args.args[0][1:3], ["validate", "--artifact"])
            self.assertIn("does not execute C0-C4", stdout.getvalue())
            with (output / "formal_readiness.json").open(
                    "r", encoding="utf-8") as stream:
                report = json.load(stream)
            self.assertEqual(report["status"], "READY_NOT_EXECUTED")
            self.assertFalse(report["formal_trials_started"])
            self.assertEqual(report["reasons"], [])
            self.assertEqual(
                report["schema"], "spmpc_formal_simulation_readiness_v2")
            self.assertIn("asset.phase_rejoin_artifact",
                          report["asset_inventory"])

    def test_metric_summary_is_trial_level(self):
        summary = CAMPAIGN.metric_summary([3.0, 1.0, 2.0, 4.0])
        self.assertEqual(summary, {"min": 1.0, "median": 2.5, "max": 4.0})

    @unittest.skipUnless(
        os.environ.get("SPMPC_INDEPENDENT_PLANT_SMOKE"),
        "set SPMPC_INDEPENDENT_PLANT_SMOKE for the C++ integration contract")
    def test_real_smoke_contract_changes_timing_and_refuses_overwrite(self):
        executable = Path(os.environ["SPMPC_INDEPENDENT_PLANT_SMOKE"])
        self.assertTrue(executable.is_file())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = CAMPAIGN.load_yaml(CONFIG_PATH)
            config["experiment"]["control_rate_hz"] = 17.0
            config["experiment"]["fixed_tail_sec"] = 1.25
            config_path = root / "changed_timing.yaml"
            config_path.write_text(
                yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
            output = root / "campaign"
            with mock.patch.object(CAMPAIGN, "ALLOWED_OUTPUT_ROOT", root):
                self.assertEqual(CAMPAIGN.main([
                    "--campaign", "smoke",
                    "--config", str(config_path),
                    "--executable", str(executable),
                    "--out-dir", str(output),
                ]), 0)

            timing = CAMPAIGN.expected_smoke_timing(config)
            summary_path = output / "seed1001.json"
            csv_path = output / "seed1001.csv"
            with summary_path.open("r", encoding="utf-8") as stream:
                summary = json.load(stream)
            self.assertEqual(
                summary["schema"], "spmpc_independent_plant_smoke_v3")
            self.assertEqual(
                summary["csv_timestamp_contract"],
                "publish_sample_effective_epochs_v1")
            self.assertEqual(summary["samples"], timing["samples"])
            self.assertEqual(summary["tail_samples"], timing["tail_samples"])
            self.assertAlmostEqual(
                summary["tail_window_start_sec"],
                timing["tail_window_start_sec"])
            self.assertAlmostEqual(summary["duration_sec"],
                                   timing["duration_sec"])
            with (output / "campaign_manifest.json").open(
                    "r", encoding="utf-8") as stream:
                manifest = json.load(stream)
            self.assertEqual(manifest["status"],
                             "COMPLETE_DEVELOPMENT_SMOKE")
            self.assertEqual(
                manifest["completion_scope"],
                "execution_and_artifact_integrity_only")
            self.assertFalse(manifest["effect_claim"])
            self.assertEqual(
                manifest["schema"], "spmpc_independent_plant_campaign_v3")

            with csv_path.open(
                    "r", encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(tuple(reader.fieldnames or ()),
                                 CAMPAIGN.CSV_COLUMNS)
                first = next(reader)
            self.assertAlmostEqual(float(first["publish_time_sec"]), 0.0)
            self.assertAlmostEqual(
                float(first["sample_time_sec"]), 1.0 / 17.0)
            for channel in ("linear", "angular"):
                publish = float(first["publish_time_sec"])
                jitter = float(
                    first[channel + "_transport_jitter_sec"])
                effective = float(first[channel + "_effective_time_sec"])
                self.assertAlmostEqual(
                    effective,
                    publish + config["external_plant"][channel]["delay_sec"] +
                    jitter)

            csv_hash = CAMPAIGN.sha256_file(csv_path)
            summary_hash = CAMPAIGN.sha256_file(summary_path)
            repeated = subprocess.run([
                str(executable), str(config_path),
                str(output / "seed1001"), "1001"],
                check=False, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
            self.assertEqual(repeated.returncode, 5)
            self.assertIn("refusing to overwrite", repeated.stderr)
            self.assertEqual(CAMPAIGN.sha256_file(csv_path), csv_hash)
            self.assertEqual(CAMPAIGN.sha256_file(summary_path), summary_hash)


if __name__ == "__main__":
    unittest.main()
