#!/usr/bin/env python3

from __future__ import annotations

import copy
import importlib.util
import json
import subprocess
import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[3]
SCRIPT = PACKAGE_ROOT / "scripts/mainline/validate_stage0_contract.py"
MANIFEST = PACKAGE_ROOT / "config/mainline/contracts/stage0_contract_v1.json"

SPEC = importlib.util.spec_from_file_location("validate_stage0_contract", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MainlineStage0ContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = MODULE.load_contract(MANIFEST)

    def validate(self, value: dict, *, verify_git: bool = False) -> None:
        MODULE.validate_contract(
            value,
            repo_root=WORKSPACE_ROOT if verify_git else None,
            verify_git=verify_git,
        )

    def assert_invalid(self, value: dict, needle: str) -> None:
        with self.assertRaises(MODULE.ContractValidationError) as context:
            self.validate(value)
        self.assertIn(needle, "\n".join(context.exception.errors))

    def test_repository_manifest_and_base_hashes_are_valid(self) -> None:
        self.validate(self.contract, verify_git=True)

    def test_all_sixteen_invariants_are_present_in_order(self) -> None:
        self.assertEqual(
            list(self.contract["frozen_contract"]["invariants"]), MODULE.INVARIANT_IDS
        )

    def test_b0_and_bslosh_only_differ_by_condition_and_scale(self) -> None:
        arms = self.contract["frozen_contract"]["experiment_arms"]
        self.assertEqual(
            arms["only_whitelisted_difference"],
            ["experiment.condition", "liquid_objective_scale"],
        )
        self.assertEqual(arms["B0"]["liquid_objective_scale"], 0.0)
        self.assertEqual(arms["Bslosh"]["liquid_objective_scale"], 1.0)

    def test_unknown_root_key_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["silent_fallback"] = True
        self.assert_invalid(mutated, "unknown keys")

    def test_missing_root_key_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.contract)
        del mutated["stage_status"]
        self.assert_invalid(mutated, "missing keys")

    def test_base_sha_cannot_drift(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["git_baseline"]["base_sha"] = "0" * 40
        self.assert_invalid(mutated, "git_baseline.base_sha")

    def test_unversioned_generated_artifact_cannot_gain_a_fake_hash(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["legacy_generated_artifacts"]["slots"][0]["sha256"] = "a" * 64
        self.assert_invalid(mutated, "slots[0].sha256")

    def test_generated_artifact_slots_reject_duplicates(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["legacy_generated_artifacts"]["slots"].append(
            copy.deepcopy(mutated["legacy_generated_artifacts"]["slots"][0])
        )
        self.assert_invalid(mutated, "exactly four unique slots")

    def test_generated_artifact_tracking_policy_is_frozen(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["legacy_generated_artifacts"]["tracking_policy"] = "TRACKED"
        self.assert_invalid(mutated, "tracking_policy")

    def test_unavailable_failure_bag_cannot_gain_an_unverified_hash(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["evidence_snapshot"]["legacy_negative_result"]["bags"][0]["sha256"] = (
            "b" * 64
        )
        self.assert_invalid(mutated, "legacy_negative_result.bags[0].sha256")

    def test_candidate_evidence_requires_real_sha256(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["evidence_snapshot"]["candidate_actuator_fit"]["bags"][0]["sha256"] = (
            None
        )
        self.assert_invalid(mutated, "candidate_actuator_fit.bags[0].sha256")

    def test_candidate_evidence_rejects_well_formed_but_wrong_identity(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["evidence_snapshot"]["candidate_actuator_fit"]["bags"][0]["sha256"] = (
            "a" * 64
        )
        self.assert_invalid(mutated, "candidate actuator bag identities")

    def test_baseline_asset_roles_cannot_be_repointed(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["baseline_assets"][0]["path"] = mutated["baseline_assets"][1]["path"]
        self.assert_invalid(mutated, "baseline asset role/path mapping")

    def test_state_order_is_frozen(self) -> None:
        mutated = copy.deepcopy(self.contract)
        state = mutated["frozen_contract"]["state_layout"]["robot_progress"]
        state[3], state[4] = state[4], state[3]
        self.assert_invalid(mutated, "robot state order")

    def test_control_order_is_frozen(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["frozen_contract"]["control_layout"]["ordered"].reverse()
        self.assert_invalid(mutated, "control order")

    def test_single_solver_and_execution_choices_are_frozen(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["frozen_contract"]["fixed_production_choices"]["solver_backend"] = (
            "legacy_factory"
        )
        self.assert_invalid(mutated, "fixed solver backend")

    def test_pre_issue_transition_order_is_frozen(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["frozen_contract"]["pre_issue_transition_order"].reverse()
        self.assert_invalid(mutated, "transition order")

    def test_liquid_cost_boundary_and_tail_semantics_are_frozen(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["frozen_contract"]["horizon_and_cost_schema"]["boundary_state"] = "x_N"
        self.assert_invalid(mutated, "boundary state")

        mutated = copy.deepcopy(self.contract)
        mutated["frozen_contract"]["horizon_and_cost_schema"]["running_coefficient"] = (
            "liquid_objective_scale*W_run"
        )
        self.assert_invalid(mutated, "running coefficient")

    def test_numeric_tuning_fields_remain_explicitly_unfrozen(self) -> None:
        self.assertEqual(
            self.contract["unfrozen_parameters"], MODULE.UNFROZEN_PARAMETERS
        )
        mutated = copy.deepcopy(self.contract)
        mutated["unfrozen_parameters"].remove("execution_model.L_max_v_sec")
        self.assert_invalid(mutated, "unfrozen_parameters")

    def test_stage3_remains_prohibited_before_lmax_freeze(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["stage_status"]["stage3"] = "READY"
        self.assert_invalid(mutated, "stage_status.stage3")

    def test_validation_partitions_remain_empty_until_stage1_freeze(self) -> None:
        partitions = self.contract["validation_protocol"]["dataset_partitions"]
        self.assertEqual(
            partitions, {"development": [], "validation": [], "final_test": []}
        )
        mutated = copy.deepcopy(self.contract)
        mutated["validation_protocol"]["dataset_partitions"]["final_test"].append(
            {"path": "unverified.bag"}
        )
        self.assert_invalid(mutated, "final-test partition")

    def test_first_pass_fidelity_and_hardware_gates_are_frozen(self) -> None:
        protocol = self.contract["validation_protocol"]
        self.assertEqual(
            protocol["actuator_fidelity"]["first_pass_final_test_gates"][
                "t90_relative_error_max"
            ],
            0.15,
        )
        self.assertEqual(
            protocol["engineering_gate"]["target_solver_time_p95_strictly_below_ms"],
            25.0,
        )
        self.assertEqual(protocol["B0_hardware_gate"]["abs_delta_v_p95_max_mps"], 0.005)
        self.assertEqual(protocol["Bslosh_gate"]["formal_mean_delta_P95_min_mm"], 0.1)

        mutated = copy.deepcopy(self.contract)
        mutated["validation_protocol"]["Bslosh_gate"][
            "formal_mean_delta_P95_min_mm"
        ] = 0.0
        self.assert_invalid(mutated, "formal P95 gate")

    def test_unknown_nested_contract_key_is_rejected(self) -> None:
        mutated = copy.deepcopy(self.contract)
        mutated["frozen_contract"]["release_grid"]["allow_skip_when_late"] = True
        self.assert_invalid(mutated, "release_grid has unknown keys")

    def test_boolean_values_cannot_impersonate_frozen_numbers(self) -> None:
        numeric_paths = [
            (("frozen_contract", "release_grid", "period_num_sec"), "numerator"),
            (
                (
                    "frozen_contract",
                    "fixed_production_choices",
                    "final_command_publishers",
                ),
                "publisher count",
            ),
            (
                (
                    "frozen_contract",
                    "experiment_arms",
                    "Bslosh",
                    "liquid_objective_scale",
                ),
                "Bslosh arm",
            ),
            (
                (
                    "validation_protocol",
                    "engineering_gate",
                    "solver_success_fraction",
                ),
                "solver success gate",
            ),
        ]
        for path, error_label in numeric_paths:
            with self.subTest(path=path):
                mutated = copy.deepcopy(self.contract)
                parent = mutated
                for key in path[:-1]:
                    parent = parent[key]
                parent[path[-1]] = True
                self.assert_invalid(mutated, error_label)

    def test_cli_reports_machine_readable_success(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                str(MANIFEST),
                "--repo-root",
                str(WORKSPACE_ROOT),
                "--json",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        payload = json.loads(result.stdout)
        self.assertTrue(payload["ok"])
        self.assertEqual(payload["base_sha"], MODULE.BASE_SHA)

    def test_cli_has_no_git_verification_bypass(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                str(MANIFEST),
                "--no-git-verification",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("unrecognized arguments", result.stderr)


if __name__ == "__main__":
    unittest.main()
