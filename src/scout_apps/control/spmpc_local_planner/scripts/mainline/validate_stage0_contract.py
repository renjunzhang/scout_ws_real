#!/usr/bin/env python3
"""Fail-closed validator for the immutable SPMPC mainline Stage 0 contract."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from collections.abc import Iterable, Mapping, MutableMapping, Sequence
from pathlib import Path
from typing import (
    Any,
)

SCHEMA_VERSION = "spmpc_mainline_stage0_contract_v1"
CONTRACT_ID = "SPMPC-MAINLINE-STAGE0-20260902"
CONTRACT_STATUS = "CONTRACT_FROZEN_NUMERICS_UNFROZEN"
BASE_SHA = "e8d10325ac6138042fdb6e707192568a6e12cbbd"
CAPTURED_AT = "2026-09-02T02:22:01+08:00"
SOURCE_PLAN_PATH = (
    "docs/实物实验注意事项/对比试验/解决问题的思路/代码改造方案_唯一主线.md"
)
SOURCE_PLAN_STATUS = (
    "ARCHITECTURE-PROPOSED_CONTRACTS-TO-FREEZE_CODE-UNIMPLEMENTED_HARDWARE-UNVERIFIED"
)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA_RE = re.compile(r"^[0-9a-f]{40}$")

BASELINE_ASSET_PATHS = {
    "build_contract": "src/scout_apps/control/spmpc_local_planner/CMakeLists.txt",
    "legacy_common_config": (
        "src/scout_apps/control/spmpc_local_planner/config/planner/common.yaml"
    ),
    "legacy_variant_config": (
        "src/scout_apps/control/spmpc_local_planner/config/planner/variants.yaml"
    ),
    "legacy_platform_config": (
        "src/scout_apps/control/spmpc_local_planner/config/platforms/scout_mini.yaml"
    ),
    "legacy_fixed_path_config": (
        "src/scout_apps/control/spmpc_local_planner/config/experiments/fixed_path.yaml"
    ),
    "legacy_point_to_point_config": (
        "src/scout_apps/control/spmpc_local_planner/config/experiments/point_to_point.yaml"
    ),
    "container_config": (
        "src/scout_apps/control/spmpc_local_planner/config/containers/tube_default.yaml"
    ),
    "legacy_codegen_entry": (
        "src/scout_apps/control/spmpc_local_planner/scripts/acados/"
        "generate_spmpc_acados.py"
    ),
    "legacy_codegen_model": (
        "src/scout_apps/control/spmpc_local_planner/scripts/acados/"
        "spmpc_acados_model.py"
    ),
    "legacy_codegen_cost": (
        "src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_cost.py"
    ),
    "legacy_codegen_constraints": (
        "src/scout_apps/control/spmpc_local_planner/scripts/acados/"
        "spmpc_acados_constraints.py"
    ),
}

GENERATED_SLOT_PATHS = {
    name: f"src/scout_apps/control/spmpc_local_planner/generated/acados/{name}"
    for name in (
        "spmpc_b0",
        "spmpc_slosh",
        "spmpc_b0_direct_omega_legacy",
        "spmpc_slosh_direct_omega",
    )
}

CANDIDATE_BAGS = [
    (
        "D02_fixed_robot_only_a01",
        (
            "/home/geist/slosh_bags/real/20260830_spmpc_mocap_execution_chain/"
            "delay_diagnostic/DEV_MOCAP_B0_DELAY_D02_fixed_robot_only_a01.bag"
        ),
        "dbad29e7390e2634228ab6325d59b88874244026f9570e8634bf82b83b0219ad",
    ),
    (
        "D03_fixed_robot_only_a02",
        (
            "/home/geist/slosh_bags/real/20260830_spmpc_mocap_execution_chain/"
            "delay_diagnostic/DEV_MOCAP_B0_DELAY_D03_fixed_robot_only_a02.bag"
        ),
        "cfa8f955b97a719367b7e66a8aa8bd6193cfe22b79a502f78c18c5fa3d512ac5",
    ),
    (
        "D04_fixed_robot_only_a01",
        (
            "/home/geist/slosh_bags/real/20260830_spmpc_mocap_execution_chain/"
            "delay_diagnostic/DEV_MOCAP_B0_DELAY_D04_fixed_robot_only_a01.bag"
        ),
        "2d432213e203cca91b8151897a65d3830e1df04a73f068a75687729ac4248c0f",
    ),
]

NEGATIVE_RESULT_BAGS = [
    (
        "legacy_v1_B0_row01",
        (
            "/home/geist/slosh_bags/real/20260901_spmpc_i0_failclosed_fixed_abba/H0/"
            "DEV_I0FC_FIXED_01_B0_b01_p01_a01.bag"
        ),
    ),
    (
        "legacy_v1_Bslosh_row02",
        (
            "/home/geist/slosh_bags/real/20260901_spmpc_i0_failclosed_fixed_abba/H0/"
            "DEV_I0FC_FIXED_02_Bslosh_b01_p02_a01.bag"
        ),
    ),
    (
        "short100_v2_B0_row01",
        (
            "/home/geist/slosh_bags/real/"
            "20260901_spmpc_i0_failclosed_fixed_short100_abba_v2/H0/"
            "DEV_I0FC_FIXED_S100_V2_01_B0_b01_p01_a01.bag"
        ),
    ),
    (
        "short100_v2_Bslosh_row02",
        (
            "/home/geist/slosh_bags/real/"
            "20260901_spmpc_i0_failclosed_fixed_short100_abba_v2/H0/"
            "DEV_I0FC_FIXED_S100_V2_02_BsloshS100_b01_p02_a01.bag"
        ),
    ),
]

IDENTIFICATION_TOOLCHAIN_PATHS = [
    (
        "src/scout_apps/control/spmpc_local_planner/scripts/"
        "run_spmpc_mocap_velocity_step_trial.sh"
    ),
    (
        "src/scout_apps/control/spmpc_local_planner/scripts/analysis/"
        "publish_mocap_velocity_step.py"
    ),
    (
        "src/scout_apps/control/spmpc_local_planner/scripts/analysis/"
        "analyze_mocap_velocity_step.py"
    ),
    (
        "src/scout_apps/control/spmpc_local_planner/scripts/analysis/"
        "velocity_step_response_core.py"
    ),
    (
        "src/scout_apps/control/spmpc_local_planner/scripts/tests/"
        "test_velocity_step_response_core.py"
    ),
]

TOP_LEVEL_KEYS = {
    "schema_version",
    "contract_id",
    "status",
    "captured_at",
    "source_plan",
    "git_baseline",
    "baseline_assets",
    "legacy_generated_artifacts",
    "evidence_snapshot",
    "frozen_contract",
    "validation_protocol",
    "unfrozen_parameters",
    "stage_status",
}

INVARIANTS = {
    "ML-01": "production_links_exactly_one_actuator_aware_solver_artifact",
    "ML-02": "B0_and_Bslosh_share_artifact_only_liquid_objective_scale_differs",
    "ML-03": "both_arms_use_processed_IMU_I0_fail_closed_and_common_epoch",
    "ML-04": "no_legacy_delay_replacement_of_solver_input",
    "ML-05": "cmd_vel_only_from_command_state_never_measured_plus_u0_dt",
    "ML-06": "pose_and_liquid_use_actual_motion_only",
    "ML-07": "independent_channel_FOPDT_with_fixed_delay_queue_and_exact_ZOH",
    "ML-08": "command_acceleration_is_state_and_jerk_is_control",
    "ML-09": "liquid_running_on_first_K_intervals_boundary_only_at_xK_tail_zero",
    "ML-10": "invalid_or_unknown_config_fails_fast_without_default_clamp_or_fallback",
    "ML-11": "history_contains_final_emitted_commands_and_authoritative_publisher_state",
    "ML-12": "diagnostics_are_observation_only",
    "ML-13": "success_requires_RGB_timing_tracking_safety_and_continuity",
    "ML-14": "each_live_release_boundary_has_exactly_one_emitted_event",
    "ML-15": "planning_and_release_are_isolated_and_state_commits_after_release_only",
    "ML-16": "steady_clock_schedules_model_clock_aligns_and_delay_queue_uses_model_grid",
}
INVARIANT_IDS = list(INVARIANTS)
ROBOT_PROGRESS_STATE = ["px", "py", "theta", "s", "v_actual", "omega_actual"]
PUBLISHER_STATE = ["q_prev_v", "q_prev_omega", "a_prev", "alpha_prev"]
DELAY_STATE = ["older_v[0:D_v-1]", "older_omega[0:D_omega-1]"]
LIQUID_STATE = ["eta_x", "eta_x_dot", "eta_y", "eta_y_dot"]
CONTROL_ORDER = ["j_issue_v", "j_issue_omega", "v_s"]
TRANSITION_ORDER = [
    "issue_map_from_publisher_state_and_jerk",
    "emit_q_issue_and_insert_into_delay_line",
    "piecewise_exact_ZOH_FOPDT_pose_and_liquid_propagation",
    "shift_delay_history_to_next_pre_issue_state",
]
EXECUTION_PARAMETER_ORDER = [
    "act_inv_tau_v",
    "act_gain_v",
    "act_inv_tau_omega",
    "act_gain_omega",
    "act_seg_dt[0:2]",
    "act_sel_v[0:2][0:NQ_v-1]",
    "act_sel_omega[0:2][0:NQ_omega-1]",
]
UNFROZEN_PARAMETERS = [
    "release.proposal_handoff_margin_sec",
    "release.publish_jitter_max_sec",
    "release.clock_sample_max_sec",
    "release.clock_map_drift_max_sec",
    "execution_model.linear.delay_sec",
    "execution_model.linear.tau_sec",
    "execution_model.linear.gain",
    "execution_model.angular.delay_sec",
    "execution_model.angular.tau_sec",
    "execution_model.angular.gain",
    "execution_model.L_max_v_sec",
    "execution_model.L_max_omega_sec",
    "command_model.a_cmd_max",
    "command_model.alpha_cmd_max",
    "command_model.jerk_v_max",
    "command_model.jerk_omega_max",
    "slosh_cost.K_liquid_selection",
    "slosh_cost.W_run",
    "slosh_cost.W_boundary",
    "slosh_cost.running_eta_dot_ratio",
    "progress_projection.start_interval",
    "progress_projection.forward_guard",
    "progress_projection.contour_guard",
    "progress_projection.heading_guard",
    "progress_projection.ambiguity_tolerance",
    "stage1.dataset_partition_and_gate_hash",
]


class DuplicateKeyError(ValueError):
    pass


class ContractValidationError(Exception):
    def __init__(self, errors: Sequence[str]):
        self.errors = list(errors)
        super().__init__("; ".join(self.errors))


def _strict_equal(actual: Any, expected: Any) -> bool:
    """Compare contract values without Python's bool/int equivalence."""
    if isinstance(expected, Mapping):
        if not isinstance(actual, Mapping) or set(actual) != set(expected):
            return False
        return all(_strict_equal(actual[key], value) for key, value in expected.items())
    if isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) != len(expected):
            return False
        return all(
            _strict_equal(actual_item, expected_item)
            for actual_item, expected_item in zip(actual, expected)
        )
    return type(actual) is type(expected) and actual == expected


def _unique_object(pairs: Iterable[tuple[str, Any]]) -> MutableMapping[str, Any]:
    result: MutableMapping[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise DuplicateKeyError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load_contract(path: Path) -> Mapping[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object
        )
    except (OSError, UnicodeError, json.JSONDecodeError, DuplicateKeyError) as exc:
        raise ContractValidationError([f"cannot load contract {path}: {exc}"]) from exc
    if not isinstance(value, Mapping):
        raise ContractValidationError(["contract root must be a JSON object"])
    return value


class Validator:
    def __init__(self, repo_root: Path | None, verify_git: bool) -> None:
        self.repo_root = repo_root.resolve() if repo_root is not None else None
        self.verify_git = verify_git
        self.errors: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def mapping(self, value: Any, label: str) -> Mapping[str, Any]:
        if not isinstance(value, Mapping):
            self.error(f"{label} must be an object")
            return {}
        return value

    def sequence(self, value: Any, label: str) -> list[Any]:
        if not isinstance(value, list):
            self.error(f"{label} must be an array")
            return []
        return value

    def exact_keys(
        self, value: Mapping[str, Any], expected: set[str], label: str
    ) -> None:
        actual = set(value)
        missing = sorted(expected - actual)
        unknown = sorted(actual - expected)
        if missing:
            self.error(f"{label} missing keys: {missing}")
        if unknown:
            self.error(f"{label} has unknown keys: {unknown}")

    def equal(self, actual: Any, expected: Any, label: str) -> None:
        if not _strict_equal(actual, expected):
            self.error(f"{label} must equal {expected!r}, got {actual!r}")

    def sha256(self, value: Any, label: str, *, nullable: bool = False) -> str | None:
        if value is None and nullable:
            return None
        if not isinstance(value, str) or not SHA256_RE.fullmatch(value):
            self.error(f"{label} must be a lowercase 64-character SHA-256")
            return None
        return value

    def nonempty_string(self, value: Any, label: str) -> str:
        if not isinstance(value, str) or not value:
            self.error(f"{label} must be a non-empty string")
            return ""
        return value

    def git(
        self, args: Sequence[str], *, binary: bool = False
    ) -> subprocess.CompletedProcess[Any]:
        if self.repo_root is None:
            raise RuntimeError("repo_root is required for Git verification")
        return subprocess.run(
            ["git", "-C", str(self.repo_root), *args],
            capture_output=True,
            text=not binary,
            check=False,
        )

    def verify_blob(
        self, base_sha: str, path: str, expected_sha: str, label: str
    ) -> None:
        if not self.verify_git:
            return
        object_type = self.git(["cat-file", "-t", f"{base_sha}:{path}"])
        if object_type.returncode != 0 or object_type.stdout.strip() != "blob":
            self.error(f"{label} is not a Git blob at base commit: {path}")
            return
        result = self.git(["show", f"{base_sha}:{path}"], binary=True)
        if result.returncode != 0:
            self.error(f"{label} is not readable from base commit: {path}")
            return
        actual_sha = hashlib.sha256(result.stdout).hexdigest()
        if actual_sha != expected_sha:
            self.error(
                f"{label} hash mismatch for {path}: contract={expected_sha}, base={actual_sha}"
            )

    def verify_absent_at_base(self, base_sha: str, path: str, label: str) -> None:
        if not self.verify_git:
            return
        result = self.git(["cat-file", "-e", f"{base_sha}:{path}"])
        if result.returncode == 0:
            self.error(f"{label} claims absence but exists at base: {path}")

    def validate_path_hash_pair(
        self, item: Mapping[str, Any], label: str, base_sha: str
    ) -> None:
        self.exact_keys(item, {"path", "sha256"}, label)
        path = self.nonempty_string(item.get("path"), f"{label}.path")
        digest = self.sha256(item.get("sha256"), f"{label}.sha256")
        if path and digest:
            self.verify_blob(base_sha, path, digest, label)

    def validate(self, root: Mapping[str, Any]) -> None:
        self.exact_keys(root, TOP_LEVEL_KEYS, "root")
        self.equal(root.get("schema_version"), SCHEMA_VERSION, "schema_version")
        self.equal(root.get("contract_id"), CONTRACT_ID, "contract_id")
        self.equal(root.get("status"), CONTRACT_STATUS, "status")
        self.equal(root.get("captured_at"), CAPTURED_AT, "captured_at")

        source_plan = self.mapping(root.get("source_plan"), "source_plan")
        self.exact_keys(
            source_plan, {"path", "sha256", "document_status_at_base"}, "source_plan"
        )
        plan_path = self.nonempty_string(source_plan.get("path"), "source_plan.path")
        plan_sha = self.sha256(source_plan.get("sha256"), "source_plan.sha256")
        self.equal(plan_path, SOURCE_PLAN_PATH, "source_plan.path")
        self.equal(
            source_plan.get("document_status_at_base"),
            SOURCE_PLAN_STATUS,
            "source_plan.document_status_at_base",
        )

        baseline = self.mapping(root.get("git_baseline"), "git_baseline")
        self.exact_keys(
            baseline,
            {
                "new_branch",
                "base_branch",
                "base_sha",
                "base_remote_ref_at_capture",
                "base_remote_sha_at_capture",
                "worktree_clean_at_capture",
                "pushed",
            },
            "git_baseline",
        )
        self.equal(
            baseline.get("new_branch"), "spmpc-mainline", "git_baseline.new_branch"
        )
        self.equal(
            baseline.get("base_branch"),
            "diag/lt-dwa-collision-tracking",
            "git_baseline.base_branch",
        )
        self.equal(
            baseline.get("base_remote_ref_at_capture"),
            "origin/diag/lt-dwa-collision-tracking",
            "git_baseline.base_remote_ref_at_capture",
        )
        base_sha = baseline.get("base_sha")
        self.equal(base_sha, BASE_SHA, "git_baseline.base_sha")
        if not isinstance(base_sha, str) or not GIT_SHA_RE.fullmatch(base_sha):
            self.error("git_baseline.base_sha must be a lowercase 40-character Git SHA")
            base_sha = BASE_SHA
        self.equal(
            baseline.get("base_remote_sha_at_capture"),
            base_sha,
            "git_baseline.base_remote_sha_at_capture",
        )
        self.equal(
            baseline.get("worktree_clean_at_capture"), True, "git baseline clean flag"
        )
        self.equal(baseline.get("pushed"), False, "git baseline pushed flag")

        if self.verify_git:
            if self.repo_root is None:
                self.error("repo_root is required when verify_git is enabled")
            else:
                commit = self.git(["cat-file", "-e", f"{base_sha}^{{commit}}"])
                if commit.returncode != 0:
                    self.error(f"base commit is not present: {base_sha}")
                ancestor = self.git(["merge-base", "--is-ancestor", base_sha, "HEAD"])
                if ancestor.returncode != 0:
                    self.error("base commit is not an ancestor of current HEAD")
                if plan_path and plan_sha:
                    self.verify_blob(base_sha, plan_path, plan_sha, "source_plan")

        assets = self.sequence(root.get("baseline_assets"), "baseline_assets")
        roles: set[str] = set()
        paths: set[str] = set()
        for index, raw_item in enumerate(assets):
            label = f"baseline_assets[{index}]"
            item = self.mapping(raw_item, label)
            self.exact_keys(item, {"role", "path", "sha256"}, label)
            role = self.nonempty_string(item.get("role"), f"{label}.role")
            path = self.nonempty_string(item.get("path"), f"{label}.path")
            digest = self.sha256(item.get("sha256"), f"{label}.sha256")
            if role in roles:
                self.error(f"duplicate baseline asset role: {role}")
            if path in paths:
                self.error(f"duplicate baseline asset path: {path}")
            roles.add(role)
            paths.add(path)
            if path and digest:
                self.verify_blob(base_sha, path, digest, label)
        self.equal(roles, set(BASELINE_ASSET_PATHS), "baseline asset roles")
        self.equal(
            {
                item.get("role"): item.get("path")
                for item in assets
                if isinstance(item, Mapping)
                and isinstance(item.get("role"), str)
                and isinstance(item.get("path"), str)
            },
            BASELINE_ASSET_PATHS,
            "baseline asset role/path mapping",
        )

        generated = self.mapping(
            root.get("legacy_generated_artifacts"), "legacy_generated_artifacts"
        )
        self.exact_keys(
            generated,
            {"tracking_policy", "capture_status", "slots"},
            "legacy_generated_artifacts",
        )
        self.equal(
            generated.get("capture_status"),
            "NO_GENERATED_SOLVER_PRESENT_IN_CAPTURE_WORKTREE",
            "legacy_generated_artifacts.capture_status",
        )
        self.equal(
            generated.get("tracking_policy"),
            "GENERATED_OUTPUTS_ARE_IGNORED_AND_NOT_VERSIONED_AT_BASE",
            "legacy_generated_artifacts.tracking_policy",
        )
        slots = self.sequence(
            generated.get("slots"), "legacy_generated_artifacts.slots"
        )
        if len(slots) != len(GENERATED_SLOT_PATHS):
            self.error(
                "legacy_generated_artifacts.slots must contain exactly four unique slots"
            )
        slot_names: set[str] = set()
        slot_paths: dict[str, str] = {}
        for index, raw_slot in enumerate(slots):
            label = f"legacy_generated_artifacts.slots[{index}]"
            slot = self.mapping(raw_slot, label)
            self.exact_keys(slot, {"name", "path", "status", "sha256"}, label)
            name = self.nonempty_string(slot.get("name"), f"{label}.name")
            path = self.nonempty_string(slot.get("path"), f"{label}.path")
            if name in slot_names:
                self.error(f"duplicate legacy generated artifact name: {name}")
            if path in slot_paths.values():
                self.error(f"duplicate legacy generated artifact path: {path}")
            slot_names.add(name)
            slot_paths[name] = path
            self.equal(
                slot.get("status"),
                "ABSENT_FROM_BASE_TREE_AND_CAPTURE_WORKTREE",
                f"{label}.status",
            )
            self.equal(slot.get("sha256"), None, f"{label}.sha256")
            if path:
                self.verify_absent_at_base(base_sha, path, label)
        self.equal(
            slot_names, set(GENERATED_SLOT_PATHS), "legacy generated artifact names"
        )
        self.equal(slot_paths, GENERATED_SLOT_PATHS, "legacy generated artifact paths")

        self.validate_evidence(root.get("evidence_snapshot"), base_sha)
        self.validate_frozen_contract(root.get("frozen_contract"))
        self.validate_validation_protocol(root.get("validation_protocol"))
        self.equal(
            root.get("unfrozen_parameters"), UNFROZEN_PARAMETERS, "unfrozen_parameters"
        )

        stage_status = self.mapping(root.get("stage_status"), "stage_status")
        self.exact_keys(
            stage_status,
            {"stage0", "stage1", "stage2", "stage3", "hardware"},
            "stage_status",
        )
        self.equal(stage_status.get("stage0"), "CONTRACT_FROZEN", "stage_status.stage0")
        self.equal(
            stage_status.get("stage1"),
            "BLOCKED_PENDING_DEDICATED_IDENTIFICATION_EVIDENCE",
            "stage_status.stage1",
        )
        self.equal(stage_status.get("stage2"), "NOT_STARTED", "stage_status.stage2")
        self.equal(
            stage_status.get("stage3"),
            "PROHIBITED_UNTIL_STAGE1_L_MAX_IS_FROZEN",
            "stage_status.stage3",
        )
        self.equal(stage_status.get("hardware"), "UNVERIFIED", "stage_status.hardware")

    def validate_evidence(self, raw: Any, base_sha: str) -> None:
        evidence = self.mapping(raw, "evidence_snapshot")
        self.exact_keys(
            evidence,
            {
                "candidate_actuator_fit",
                "legacy_negative_result",
                "dedicated_identification_toolchain",
            },
            "evidence_snapshot",
        )

        fit = self.mapping(
            evidence.get("candidate_actuator_fit"), "candidate_actuator_fit"
        )
        self.exact_keys(
            fit,
            {"status", "source_document", "source_sha256", "candidate_only", "bags"},
            "candidate_actuator_fit",
        )
        self.equal(
            fit.get("status"),
            "INCONCLUSIVE_NEED_DEDICATED_IDENTIFICATION_BAG",
            "candidate_actuator_fit.status",
        )
        fit_path = self.nonempty_string(
            fit.get("source_document"), "fit source document"
        )
        self.equal(
            fit_path,
            "docs/实物实验注意事项/对比试验/解决问题的思路/20260830_延迟问题.md",
            "candidate_actuator_fit.source_document",
        )
        fit_sha = self.sha256(fit.get("source_sha256"), "fit source sha256")
        if fit_path and fit_sha:
            self.verify_blob(
                base_sha, fit_path, fit_sha, "candidate_actuator_fit source"
            )
        candidate = self.mapping(
            fit.get("candidate_only"), "candidate_actuator_fit.candidate_only"
        )
        self.exact_keys(candidate, {"linear", "angular"}, "candidate_only")
        self.equal(
            candidate,
            {
                "linear": {"delay_sec": 0.08, "tau_sec": 0.086, "gain": 1.05},
                "angular": {"delay_sec": 0.07, "tau_sec": 0.677, "gain": 0.99},
            },
            "candidate-only actuator values",
        )
        for channel in ("linear", "angular"):
            values = self.mapping(candidate.get(channel), f"candidate_only.{channel}")
            self.exact_keys(
                values, {"delay_sec", "tau_sec", "gain"}, f"candidate_only.{channel}"
            )
            for key in ("delay_sec", "tau_sec", "gain"):
                value = values.get(key)
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or value <= 0
                ):
                    self.error(f"candidate_only.{channel}.{key} must be positive")
        fit_bags = self.sequence(fit.get("bags"), "candidate_actuator_fit.bags")
        if len(fit_bags) != 3:
            self.error("candidate_actuator_fit.bags must contain D02, D03 and D04")
        observed_candidate_bags: list[tuple[str, str, str]] = []
        for index, raw_bag in enumerate(fit_bags):
            label = f"candidate_actuator_fit.bags[{index}]"
            bag = self.mapping(raw_bag, label)
            self.exact_keys(
                bag, {"label", "path", "sha256", "availability_at_capture"}, label
            )
            bag_label = self.nonempty_string(bag.get("label"), f"{label}.label")
            bag_path = self.nonempty_string(bag.get("path"), f"{label}.path")
            bag_sha = self.sha256(bag.get("sha256"), f"{label}.sha256")
            if bag_sha:
                observed_candidate_bags.append((bag_label, bag_path, bag_sha))
            self.equal(
                bag.get("availability_at_capture"),
                "NOT_PRESENT_ON_THIS_HOST",
                f"{label}.availability_at_capture",
            )
        self.equal(
            observed_candidate_bags,
            CANDIDATE_BAGS,
            "candidate actuator bag identities",
        )

        negative = self.mapping(
            evidence.get("legacy_negative_result"), "legacy_negative_result"
        )
        self.exact_keys(
            negative,
            {
                "status",
                "source_document",
                "source_sha256",
                "decision",
                "protocols",
                "bags",
            },
            "legacy_negative_result",
        )
        self.equal(
            negative.get("status"),
            "DOCUMENTED_REFERENCE_ONLY_BAG_SHA256_UNAVAILABLE_ON_THIS_HOST",
            "legacy_negative_result.status",
        )
        self.equal(
            negative.get("decision"), "STOP_BLOCK1_FUTILITY", "negative decision"
        )
        self.equal(
            negative.get("protocols"),
            [
                "SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1",
                "SMPCC_I0_FAILCLOSED_FIXED_SHORT100_ABBA_DEV_V2",
            ],
            "negative protocols",
        )
        negative_path = self.nonempty_string(
            negative.get("source_document"), "negative source document"
        )
        self.equal(
            negative_path,
            "docs/实物实验注意事项/对比试验/实物对比试验分析/"
            "20260901_I0_fail_closed_fixed_closed_loop_B0_Bslosh_Block1分析.md",
            "legacy_negative_result.source_document",
        )
        negative_sha = self.sha256(
            negative.get("source_sha256"), "negative source sha256"
        )
        if negative_path and negative_sha:
            self.verify_blob(
                base_sha, negative_path, negative_sha, "legacy negative source"
            )
        negative_bags = self.sequence(
            negative.get("bags"), "legacy_negative_result.bags"
        )
        if len(negative_bags) != 4:
            self.error(
                "legacy_negative_result.bags must contain the two stopped Block 1 pairs"
            )
        observed_negative_bags: list[tuple[str, str]] = []
        for index, raw_bag in enumerate(negative_bags):
            label = f"legacy_negative_result.bags[{index}]"
            bag = self.mapping(raw_bag, label)
            self.exact_keys(
                bag, {"label", "path", "sha256", "availability_at_capture"}, label
            )
            bag_label = self.nonempty_string(bag.get("label"), f"{label}.label")
            bag_path = self.nonempty_string(bag.get("path"), f"{label}.path")
            observed_negative_bags.append((bag_label, bag_path))
            self.equal(bag.get("sha256"), None, f"{label}.sha256")
            self.equal(
                bag.get("availability_at_capture"),
                "NOT_PRESENT_ON_THIS_HOST",
                f"{label}.availability_at_capture",
            )
        self.equal(
            observed_negative_bags,
            NEGATIVE_RESULT_BAGS,
            "legacy negative-result bag identities",
        )

        toolchain = self.mapping(
            evidence.get("dedicated_identification_toolchain"),
            "dedicated_identification_toolchain",
        )
        self.exact_keys(
            toolchain,
            {"status", "protocol_document", "assets"},
            "dedicated_identification_toolchain",
        )
        self.equal(
            toolchain.get("status"),
            "IMPLEMENTED_BUT_NO_DEDICATED_BAG_CAPTURED_IN_BASE",
            "dedicated_identification_toolchain.status",
        )
        protocol = self.mapping(toolchain.get("protocol_document"), "protocol_document")
        self.validate_path_hash_pair(protocol, "protocol_document", base_sha)
        self.equal(
            protocol.get("path"),
            "docs/实物实验注意事项/对比试验/实物对比实验/"
            "20260901_静止速度阶跃与加速度性能测试.md",
            "identification protocol path",
        )
        assets = self.sequence(
            toolchain.get("assets"), "dedicated_identification_toolchain.assets"
        )
        if len(assets) != 5:
            self.error("dedicated identification toolchain must freeze five assets")
        for index, asset in enumerate(assets):
            self.validate_path_hash_pair(
                self.mapping(asset, f"toolchain.assets[{index}]"),
                f"toolchain.assets[{index}]",
                base_sha,
            )
        self.equal(
            [asset.get("path") for asset in assets if isinstance(asset, Mapping)],
            IDENTIFICATION_TOOLCHAIN_PATHS,
            "identification toolchain paths",
        )

    def validate_frozen_contract(self, raw: Any) -> None:
        contract = self.mapping(raw, "frozen_contract")
        self.exact_keys(
            contract,
            {
                "invariants",
                "fixed_production_choices",
                "experiment_arms",
                "release_grid",
                "clock_roles",
                "state_layout",
                "control_layout",
                "pre_issue_transition_order",
                "execution_parameter_schema",
                "horizon_and_cost_schema",
                "authority_commit_rule",
            },
            "frozen_contract",
        )
        invariants = self.mapping(
            contract.get("invariants"), "frozen_contract.invariants"
        )
        self.equal(list(invariants), INVARIANT_IDS, "invariant IDs and order")
        self.equal(dict(invariants), INVARIANTS, "invariant descriptions")
        for invariant_id, description in invariants.items():
            self.nonempty_string(description, f"invariants.{invariant_id}")

        choices = self.mapping(
            contract.get("fixed_production_choices"), "fixed choices"
        )
        self.exact_keys(
            choices,
            {
                "observer",
                "failure_policy",
                "common_epoch",
                "execution_model",
                "solver_backend",
                "final_command_publishers",
            },
            "fixed choices",
        )
        self.equal(choices.get("observer"), "processed_imu_I0", "fixed observer")
        self.equal(choices.get("failure_policy"), "fail_closed", "fixed failure policy")
        self.equal(choices.get("common_epoch"), "required", "fixed common epoch")
        self.equal(
            choices.get("execution_model"),
            "explicit_discrete_channelwise_FOPDT",
            "fixed execution model",
        )
        self.equal(
            choices.get("solver_backend"),
            "single_mainline_actuator_aware_acados",
            "fixed solver backend",
        )
        self.equal(
            choices.get("final_command_publishers"), 1, "final command publisher count"
        )

        arms = self.mapping(contract.get("experiment_arms"), "experiment_arms")
        self.exact_keys(
            arms, {"only_whitelisted_difference", "B0", "Bslosh"}, "experiment_arms"
        )
        self.equal(
            arms.get("only_whitelisted_difference"),
            ["experiment.condition", "liquid_objective_scale"],
            "arm difference whitelist",
        )
        self.equal(
            arms.get("B0"),
            {"experiment.condition": "B0", "liquid_objective_scale": 0.0},
            "B0 arm",
        )
        self.equal(
            arms.get("Bslosh"),
            {"experiment.condition": "Bslosh", "liquid_objective_scale": 1.0},
            "Bslosh arm",
        )

        release = self.mapping(contract.get("release_grid"), "release_grid")
        self.exact_keys(
            release,
            {
                "clock",
                "period_num_sec",
                "period_den",
                "timestamp_formula",
                "ocp_dt_sec",
                "skip_policy",
                "live_publish_enabled_event_count_per_boundary",
                "dry_run_emitted_event_count_per_boundary",
                "dry_run_would_publish_audit_count_per_boundary",
            },
            "release_grid",
        )
        self.equal(release.get("period_num_sec"), 1, "release numerator")
        self.equal(release.get("period_den"), 30, "release denominator")
        self.equal(release.get("clock"), "steady_monotonic", "release clock")
        self.equal(release.get("ocp_dt_sec"), "1/30", "OCP dt")
        self.equal(
            release.get("timestamp_formula"),
            "T_k_ns=T_0_ns+round_nearest(k*1e9/30)",
            "release timestamp formula",
        )
        self.equal(
            release.get("skip_policy"),
            "ENUMERATE_ALL_BOUNDARIES_NO_SKIP",
            "skip policy",
        )
        self.equal(
            release.get("live_publish_enabled_event_count_per_boundary"),
            1,
            "live event count",
        )
        self.equal(
            release.get("dry_run_emitted_event_count_per_boundary"),
            0,
            "dry emitted count",
        )
        self.equal(
            release.get("dry_run_would_publish_audit_count_per_boundary"),
            1,
            "dry audit count",
        )

        clocks = self.mapping(contract.get("clock_roles"), "clock_roles")
        self.exact_keys(
            clocks, {"steady", "model", "mapping", "anchor_sampling"}, "clock_roles"
        )
        self.equal(
            clocks.get("steady"),
            "deadline_handoff_and_publish_scheduling_only",
            "steady clock role",
        )
        self.equal(
            clocks.get("model"),
            "sensor_common_epoch_prefix_delay_queue_and_OCP_only",
            "model clock role",
        )
        self.equal(
            clocks.get("mapping"),
            "T_k_model=C_model0+(T_k_steady-C_steady0)",
            "clock mapping",
        )
        self.equal(
            clocks.get("anchor_sampling"),
            "steady_before_model_sample_steady_after_bracket",
            "clock anchor sampling",
        )

        state = self.mapping(contract.get("state_layout"), "state_layout")
        self.exact_keys(
            state,
            {
                "stage_semantics",
                "robot_progress",
                "publisher",
                "delay_older",
                "liquid",
                "dimension",
            },
            "state_layout",
        )
        self.equal(
            state.get("stage_semantics"), "pre_issue_at_T_k_minus", "stage semantics"
        )
        self.equal(
            state.get("robot_progress"), ROBOT_PROGRESS_STATE, "robot state order"
        )
        self.equal(state.get("publisher"), PUBLISHER_STATE, "publisher state order")
        self.equal(state.get("delay_older"), DELAY_STATE, "delay state order")
        self.equal(state.get("liquid"), LIQUID_STATE, "liquid state order")
        self.equal(state.get("dimension"), "14+D_v+D_omega", "state dimension")

        control = self.mapping(contract.get("control_layout"), "control_layout")
        self.exact_keys(control, {"ordered", "dimension"}, "control_layout")
        self.equal(control.get("ordered"), CONTROL_ORDER, "control order")
        self.equal(control.get("dimension"), 3, "control dimension")
        self.equal(
            contract.get("pre_issue_transition_order"),
            TRANSITION_ORDER,
            "transition order",
        )

        execution = self.mapping(
            contract.get("execution_parameter_schema"), "execution_parameter_schema"
        )
        self.exact_keys(
            execution,
            {
                "subsegment_slots",
                "D_c",
                "R_c",
                "NQ_c",
                "NP_exec",
                "parameter_order",
                "discretization_schema",
            },
            "execution_parameter_schema",
        )
        self.equal(execution.get("subsegment_slots"), 3, "execution slot count")
        self.equal(execution.get("D_c"), "max(0,ceil(L_max_c/dt)-1)", "D_c formula")
        self.equal(execution.get("R_c"), "ceil(L_max_c/dt)", "R_c formula")
        self.equal(execution.get("NQ_c"), "R_c+1", "NQ_c formula")
        self.equal(execution.get("NP_exec"), "7+3*(NQ_v+NQ_omega)", "NP_exec formula")
        self.equal(
            execution.get("parameter_order"),
            EXECUTION_PARAMETER_ORDER,
            "parameter order",
        )
        self.equal(
            execution.get("discretization_schema"),
            "zoh_fopdt_piecewise_midpoint_pose_rk4_slosh_v1",
            "discretization schema",
        )

        cost = self.mapping(
            contract.get("horizon_and_cost_schema"), "horizon_and_cost_schema"
        )
        self.exact_keys(
            cost,
            {
                "horizon_steps",
                "robot_horizon_sec",
                "cost_schema",
                "cost_scaling_all_stages",
                "K_liquid_constraint",
                "development_candidates",
                "validation_candidates",
                "running_state",
                "running_coefficient",
                "boundary_state",
                "boundary_coefficient",
                "tail_liquid_cost",
                "terminal_xN_liquid_cost",
            },
            "horizon_and_cost_schema",
        )
        self.equal(cost.get("horizon_steps"), 60, "horizon steps")
        self.equal(cost.get("robot_horizon_sec"), 2.0, "robot horizon")
        self.equal(
            cost.get("cost_schema"),
            "right_endpoint_fixed_liquid_weight_v1",
            "cost schema",
        )
        self.equal(cost.get("cost_scaling_all_stages"), 1.0, "cost scaling")
        self.equal(cost.get("K_liquid_constraint"), "1<=K_liquid<N", "K constraint")
        self.equal(
            cost.get("development_candidates"), [8, 10], "development K candidates"
        )
        self.equal(
            cost.get("validation_candidates"), [3, 5, 8, 10], "validation K candidates"
        )
        self.equal(
            cost.get("running_state"),
            "right_endpoint_x_k_plus_1_for_intervals_0_through_K_minus_1",
            "running state",
        )
        self.equal(
            cost.get("running_coefficient"),
            "liquid_objective_scale*W_run/K_liquid",
            "running coefficient",
        )
        self.equal(cost.get("boundary_state"), "x_K_liquid", "boundary state")
        self.equal(
            cost.get("boundary_coefficient"),
            "liquid_objective_scale*W_boundary",
            "boundary coefficient",
        )
        self.equal(cost.get("tail_liquid_cost"), 0.0, "tail liquid cost")
        self.equal(cost.get("terminal_xN_liquid_cost"), 0.0, "terminal liquid cost")

        authority = self.mapping(
            contract.get("authority_commit_rule"), "authority_commit_rule"
        )
        self.exact_keys(
            authority,
            {
                "proposal_is_fact",
                "authoritative_commit_point",
                "nominal_release_commits",
                "non_nominal_release",
            },
            "authority_commit_rule",
        )
        self.equal(authority.get("proposal_is_fact"), False, "proposal fact flag")
        self.equal(
            authority.get("authoritative_commit_point"),
            "after_FinalCommandPublisher_successful_publish_call",
            "authoritative commit point",
        )
        self.equal(
            authority.get("nominal_release_commits"),
            [
                "emitted_history",
                "publisher_state",
                "delay_queue_generation",
                "warm_start_candidate",
                "s_commit",
            ],
            "nominal release commits",
        )
        self.equal(
            authority.get("non_nominal_release"),
            "commit_actual_zero_and_reset_acceleration_discard_warm_start_and_latch_fault",
            "non-nominal release rule",
        )

    def validate_validation_protocol(self, raw: Any) -> None:
        protocol = self.mapping(raw, "validation_protocol")
        self.exact_keys(
            protocol,
            {
                "schema_id",
                "freeze_status",
                "dataset_partitions",
                "partition_rules",
                "actuator_fidelity",
                "liquid_fidelity",
                "engineering_gate",
                "B0_hardware_gate",
                "Bslosh_gate",
            },
            "validation_protocol",
        )
        self.equal(
            protocol.get("schema_id"),
            "spmpc_mainline_validation_protocol_v1",
            "validation protocol schema",
        )
        self.equal(
            protocol.get("freeze_status"),
            "STRUCTURE_AND_FIRST_PASS_GATES_FROZEN_DATASET_IDENTITIES_UNFROZEN",
            "validation protocol freeze status",
        )
        partitions = self.mapping(
            protocol.get("dataset_partitions"), "dataset_partitions"
        )
        self.exact_keys(
            partitions,
            {"development", "validation", "final_test"},
            "dataset_partitions",
        )
        self.equal(partitions.get("development"), [], "development partition")
        self.equal(partitions.get("validation"), [], "validation partition")
        self.equal(partitions.get("final_test"), [], "final-test partition")
        self.equal(
            protocol.get("partition_rules"),
            [
                "partitions_are_pairwise_disjoint_by_bag_sha256",
                "every_entry_requires_path_sha256_motion_window_and_role",
                "development_only_fits_parameters_and_forms_candidates",
                "validation_selects_structure_K_liquid_and_weights",
                "final_test_is_unsealed_once_after_all_choices_and_hashes_are_frozen",
                "any_post_final_change_demotes_that_final_set_to_development_and_requires_new_unseen_final_data",
            ],
            "partition rules",
        )

        actuator = self.mapping(protocol.get("actuator_fidelity"), "actuator_fidelity")
        self.exact_keys(
            actuator,
            {
                "statistical_unit",
                "channels_judged_separately",
                "metrics",
                "first_pass_final_test_gates",
            },
            "actuator_fidelity",
        )
        self.equal(
            actuator.get("statistical_unit"), "one_independent_trial", "actuator unit"
        )
        self.equal(
            actuator.get("channels_judged_separately"),
            ["linear", "angular"],
            "actuator channels",
        )
        self.equal(
            actuator.get("metrics"),
            [
                "velocity_RMSE",
                "velocity_MAE",
                "onset_error",
                "t90_relative_error",
                "steady_state_gain_error",
                "phase_cross_correlation_lag_error",
            ],
            "actuator metrics",
        )
        actuator_gates = self.mapping(
            actuator.get("first_pass_final_test_gates"), "actuator final-test gates"
        )
        self.equal(
            actuator_gates,
            {
                "median_RMSE_new_vs_best_baseline_max_ratio": 0.9,
                "per_trial_RMSE_new_vs_best_baseline_max_ratio": 1.05,
                "absolute_onset_error_max_nominal_steps": 1,
                "t90_relative_error_max": 0.15,
            },
            "actuator final-test gates",
        )

        liquid = self.mapping(protocol.get("liquid_fidelity"), "liquid_fidelity")
        self.exact_keys(
            liquid,
            {
                "statistical_unit",
                "future_bin_edges_ms",
                "skill_formula",
                "external_truth",
                "future_input",
                "uninformative_rule",
                "per_informative_bin_gates",
            },
            "liquid_fidelity",
        )
        self.equal(
            liquid.get("future_bin_edges_ms"), [0, 100, 167, 267, 333], "liquid bins"
        )
        self.equal(
            liquid.get("statistical_unit"),
            "one_horizon_aligned_independent_trial",
            "liquid statistical unit",
        )
        self.equal(
            liquid.get("skill_formula"),
            "1-MAE_model_bin/MAE_free_response_bin",
            "liquid skill formula",
        )
        self.equal(
            liquid.get("external_truth"),
            "future_RGB_or_external_liquid_surface_measurement",
            "liquid external truth",
        )
        self.equal(
            liquid.get("future_input"),
            "subsequent_real_emitted_command_from_bag_only",
            "liquid future input",
        )
        self.equal(
            liquid.get("uninformative_rule"),
            "mark_only_trial_bin_when_free_response_MAE_is_below_preregistered_noise_floor",
            "liquid uninformative-bin rule",
        )
        liquid_gates = self.mapping(
            liquid.get("per_informative_bin_gates"), "liquid informative-bin gates"
        )
        self.equal(
            liquid_gates,
            {
                "median_skill_strictly_positive": True,
                "positive_trial_count": "at_least_ceil(2*N_info_bin/3)",
                "report_worst_trial": True,
                "report_controllable_gradient_nodes": True,
                "report_tail_peak": True,
            },
            "liquid informative-bin gates",
        )

        engineering = self.mapping(protocol.get("engineering_gate"), "engineering_gate")
        self.exact_keys(
            engineering,
            {
                "build_test_pass_fraction",
                "python_cpp_golden_match",
                "solver_success_fraction",
                "nonfinite_count",
                "target_solver_time_p95_strictly_below_ms",
                "proposal_handoff_cutoff_miss_count",
                "manifest_config_schema_mismatch_count",
                "legacy_delay_applied_count",
                "history_prefix_complete_fraction",
                "live_command_state_emitted_mismatch_count",
                "post_limit_change_count",
                "live_scheduled_to_emitted_ratio",
                "dry_run_scheduled_to_would_audit_ratio",
                "dry_run_emitted_event_count",
                "live_early_publish_count",
                "live_publish_jitter_violation_count",
                "late_or_wrong_proposal_state_commit_count",
            },
            "engineering_gate",
        )
        self.equal(engineering.get("build_test_pass_fraction"), 1.0, "build/test gate")
        self.equal(
            engineering.get("python_cpp_golden_match"),
            "within_frozen_tolerance",
            "Python/C++ golden gate",
        )
        self.equal(
            engineering.get("solver_success_fraction"), 1.0, "solver success gate"
        )
        self.equal(
            engineering.get("target_solver_time_p95_strictly_below_ms"),
            25.0,
            "solver P95 gate",
        )
        self.equal(
            engineering.get("history_prefix_complete_fraction"),
            1.0,
            "history prefix gate",
        )
        self.equal(
            engineering.get("live_scheduled_to_emitted_ratio"),
            1.0,
            "live scheduled/emitted gate",
        )
        self.equal(
            engineering.get("dry_run_scheduled_to_would_audit_ratio"),
            1.0,
            "dry-run scheduled/audit gate",
        )
        for key in (
            "nonfinite_count",
            "proposal_handoff_cutoff_miss_count",
            "manifest_config_schema_mismatch_count",
            "legacy_delay_applied_count",
            "live_command_state_emitted_mismatch_count",
            "post_limit_change_count",
            "dry_run_emitted_event_count",
            "live_early_publish_count",
            "live_publish_jitter_violation_count",
            "late_or_wrong_proposal_state_commit_count",
        ):
            self.equal(engineering.get(key), 0, f"engineering_gate.{key}")

        b0 = self.mapping(protocol.get("B0_hardware_gate"), "B0_hardware_gate")
        self.exact_keys(
            b0,
            {
                "abs_delta_v_p95_max_mps",
                "abs_delta_v_ge_0p015_count",
                "strong_acceleration_sign_flip_count",
                "per_channel_acceleration_or_jerk_saturation_fraction_max",
                "contour_p95_max_m",
                "yaw_p95_max_rad",
                "goal_time_max_sec",
                "first_run_v_safe_max_mps",
                "solver_limiter_safety_contamination_count",
            },
            "B0_hardware_gate",
        )
        self.equal(b0.get("abs_delta_v_p95_max_mps"), 0.005, "B0 delta-v P95 gate")
        self.equal(b0.get("abs_delta_v_ge_0p015_count"), 0, "B0 large delta-v count")
        self.equal(
            b0.get("strong_acceleration_sign_flip_count"), 0, "B0 sign-flip gate"
        )
        self.equal(
            b0.get("per_channel_acceleration_or_jerk_saturation_fraction_max"),
            0.05,
            "B0 saturation gate",
        )
        self.equal(b0.get("contour_p95_max_m"), 0.05, "B0 contour gate")
        self.equal(b0.get("yaw_p95_max_rad"), 0.15, "B0 yaw gate")
        self.equal(b0.get("goal_time_max_sec"), 42.0, "B0 goal-time gate")
        self.equal(b0.get("first_run_v_safe_max_mps"), 0.15, "B0 first-run speed gate")
        self.equal(
            b0.get("solver_limiter_safety_contamination_count"),
            0,
            "B0 contamination gate",
        )

        bslosh = self.mapping(protocol.get("Bslosh_gate"), "Bslosh_gate")
        self.exact_keys(
            bslosh,
            {
                "same_artifact_model_common_controller_hash",
                "effective_config_diff_whitelist",
                "Bslosh_to_B0_goal_time_ratio_max",
                "RGB_P95_and_RMS_must_both_improve",
                "formal_delta_sign",
                "formal_mean_delta_P95_min_mm",
                "formal_positive_block_fraction_min",
                "formal_median_delta_P95_strictly_positive",
                "formal_leave_one_block_out_mean_strictly_positive",
                "formal_mean_delta_RMS_strictly_positive",
            },
            "Bslosh_gate",
        )
        self.equal(
            bslosh.get("same_artifact_model_common_controller_hash"),
            True,
            "Bslosh shared identity gate",
        )
        self.equal(
            bslosh.get("effective_config_diff_whitelist"),
            ["experiment.condition", "liquid_objective_scale"],
            "Bslosh diff whitelist",
        )
        self.equal(
            bslosh.get("Bslosh_to_B0_goal_time_ratio_max"), 1.05, "Bslosh time gate"
        )
        self.equal(
            bslosh.get("RGB_P95_and_RMS_must_both_improve"),
            True,
            "Bslosh RGB joint gate",
        )
        self.equal(
            bslosh.get("formal_delta_sign"),
            "metric_B0_minus_metric_Bslosh_positive_means_improvement",
            "formal delta sign",
        )
        self.equal(bslosh.get("formal_mean_delta_P95_min_mm"), 0.1, "formal P95 gate")
        self.equal(
            bslosh.get("formal_positive_block_fraction_min"), "2/3", "formal block gate"
        )
        self.equal(
            bslosh.get("formal_median_delta_P95_strictly_positive"),
            True,
            "formal median gate",
        )
        self.equal(
            bslosh.get("formal_leave_one_block_out_mean_strictly_positive"),
            True,
            "formal leave-one-out gate",
        )
        self.equal(
            bslosh.get("formal_mean_delta_RMS_strictly_positive"),
            True,
            "formal RMS gate",
        )

    def finish(self) -> None:
        if self.errors:
            raise ContractValidationError(self.errors)


def validate_contract(
    contract: Mapping[str, Any],
    *,
    repo_root: Path | None = None,
    verify_git: bool = True,
) -> None:
    validator = Validator(repo_root, verify_git)
    validator.validate(contract)
    validator.finish()


def default_repo_root(script_path: Path) -> Path:
    # scripts/mainline/<script> -> package -> control -> scout_apps -> src -> workspace
    return script_path.resolve().parents[6]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    script = Path(__file__)
    package_root = script.resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "contract",
        nargs="?",
        type=Path,
        default=package_root / "config/mainline/contracts/stage0_contract_v1.json",
    )
    parser.add_argument("--repo-root", type=Path, default=default_repo_root(script))
    parser.add_argument("--json", action="store_true", dest="json_output")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        contract = load_contract(args.contract)
        validate_contract(
            contract,
            repo_root=args.repo_root,
            verify_git=True,
        )
    except ContractValidationError as exc:
        if args.json_output:
            print(
                json.dumps(
                    {"ok": False, "errors": exc.errors}, ensure_ascii=False, indent=2
                )
            )
        else:
            for error in exc.errors:
                print(f"ERROR: {error}", file=sys.stderr)
        return 1
    result: dict[str, Any] = {
        "ok": True,
        "schema_version": contract["schema_version"],
        "contract_id": contract["contract_id"],
        "base_sha": contract["git_baseline"]["base_sha"],
        "stage1": contract["stage_status"]["stage1"],
    }
    if args.json_output:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print("PASS: {contract_id} at {base_sha}; Stage 1: {stage1}".format(**result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
