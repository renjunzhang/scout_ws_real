#!/usr/bin/env python3
"""Dry-run preflight checks for the SPMPC comparison benchmark.

Phase 0 scope: load benchmark policy YAML, check static consistency, and
report whether a planner/run is eligible to continue toward later gates. This
script intentionally does not start ROS/Gazebo, kill processes, edit files, or
change runtime planner behavior.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised only on missing dep
    raise SystemExit(
        "PyYAML missing; install python3-yaml or pip3 install pyyaml"
    ) from exc

from bench_validate_comparison_contracts import ComparisonContractValidator


REQUIRED_BENCHMARK_FILES = {
    "capability_matrix.yaml": "planners",
    "common_limits.yaml": "limits",
    "common_environment.yaml": "common_environment",
    "information_access_policy.yaml": "information_access_policy",
    "tuning_protocol.yaml": "tuning",
    "external_slosh_observer_policy.yaml": "external_slosh_observer",
    "slosh_model_monitor_policy.yaml": "slosh_monitor",
    "failure_taxonomy.yaml": "status_codes",
    "main_table_inclusion.yaml": "main_table_inclusion",
    "profile_tracking_common.yaml": "profile_tracker",
}

REQUIRED_CONTROL_PACKAGES = {
    "spmpc_experiments": "src/scout_apps/control/spmpc_experiments",
    "baseline_local_planner_runner": "src/scout_apps/control/baseline_local_planner_runner",
    "scout_local_planner": "src/scout_apps/control/scout_local_planner",
    "scout_profile_baselines": "src/scout_apps/control/scout_profile_baselines",
    "slosh_models": "src/scout_apps/control/slosh_models",
    "lt_dwa_adapter": "src/scout_apps/control/lt_dwa_adapter",
    "lt_dwa_v2_adapter": "src/scout_apps/control/lt_dwa_v2_adapter",
}

REQUIRED_REUSE_ASSETS = {
    "fixed_path_runner": "src/scout_apps/control/scout_local_planner/scripts/fixed_global_path_runner.py",
    "template_fixed_path_generator": "src/scout_apps/control/scout_local_planner/scripts/template_fixed_path_generator.py",
    "profile_csv_utils": "src/scout_apps/control/scout_profile_baselines/scripts/common/path_profile_utils.py",
    "fixed_path_feasibility_analyzer": "src/scout_apps/control/scout_local_planner/scripts/analysis/analyze_fixed_path_feasibility.py",
    "fixed_path_metrics_extractor": "src/scout_apps/control/spmpc_experiments/scripts/extract_fixed_path_paper_metrics.py",
    "spmpc_suite": "src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh",
    "baseline_suite": "src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_baseline_suite.sh",
    "baseline_runner_launch": "src/scout_apps/control/baseline_local_planner_runner/launch/nav_core_runner.launch",
    "lt_dwa_adapter_launch": "src/scout_apps/control/lt_dwa_adapter/launch/lt_dwa_adapter.launch",
    "lt_dwa_benchmark_launch": "src/scout_apps/control/spmpc_experiments/launch/sim/run_lt_dwa_fixed_path_sim.launch",
    "lt_dwa_benchmark_config": "src/scout_apps/control/spmpc_experiments/config/baselines/lt_dwa_adapter_standalone_sim.yaml",
    "lt_dwa_v2_adapter_launch": "src/scout_apps/control/lt_dwa_v2_adapter/launch/lt_dwa_v2_adapter.launch",
    "lt_dwa_v2_benchmark_launch": "src/scout_apps/control/spmpc_experiments/launch/sim/run_lt_dwa_v2_fixed_path_sim.launch",
    "lt_dwa_v2_benchmark_config": "src/scout_apps/control/spmpc_experiments/config/baselines/lt_dwa_v2_adapter_standalone_sim.yaml",
    "slosh_monitor_launch": "src/scout_apps/control/slosh_models/launch/slosh_monitor.launch",
}

EXPECTED_COMMON_LIMITS = {
    "v_max_mps": 0.8,
    "omega_max_radps": 1.2,
    "a_max_mps2": 0.6,
    "alpha_max_radps2": 1.2,
}

REQUIRED_MONITOR_FORBIDDEN_CONSUMERS = {
    "planner",
    "spmpc_ocp",
    "profile_generator_during_test",
    "command_gate",
    "cmd_vel_chain",
}

REQUIRED_MONITOR_ALLOWED_CONSUMERS = {
    "rosbag",
    "metrics_extractor",
    "rviz_or_plot",
}

REQUIRED_FAILURE_CODES = {
    "MONITOR_USED_FOR_CONTROL",
    "NON_STRICT_FRESH_SIM",
    "COMMAND_GATE_TOO_ACTIVE",
    "PARAMETER_NOT_FROZEN",
    "SLOSH_MONITOR_MISSING",
    "REQUIRED_TOPIC_MISSING",
}

REQUIRED_MAIN_TABLE_EXCLUSIONS = {
    "DEPENDENCY_SKIPPED",
    "PRECHECK_FAILED",
    "FAIRNESS_VIOLATION",
    "PARAMETER_NOT_FROZEN",
    "NON_STRICT_FRESH_SIM",
    "COMMAND_GATE_TOO_ACTIVE",
    "MONITOR_USED_FOR_CONTROL",
    "REQUIRED_TOPIC_MISSING",
}

EXPECTED_PROFILE_COLUMNS = [
    "s_normalized",
    "s_m",
    "t_s",
    "x",
    "y",
    "yaw",
    "v_ref_m_s",
    "a_ref_m_s2",
    "jerk_ref_m_s3",
    "method",
]

PROFILE_BASELINES = {
    "hamaguchi_profile": {
        "config": "hamaguchi_profile.yaml",
        "generator": "src/scout_apps/control/scout_profile_baselines/scripts/generate_hamaguchi_profile.py",
        "method_name": "HAMAGUCHI_STYLE",
    },
    "lim_profile": {
        "config": "lim_profile.yaml",
        "generator": "src/scout_apps/control/scout_profile_baselines/scripts/generate_lim_style_profile.py",
        "method_name": "LIM_STYLE",
    },
}

FORBIDDEN_PROFILE_GENERATOR_TOKENS = (
    "rospy.Subscriber",
    "/benchmark/slosh_monitor",
    "/slosh/",
    "/cmd_vel",
)


class Preflight:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.repo_root = args.repo_root.resolve() if args.repo_root else infer_repo_root()
        self.package_root = self.repo_root / "src/scout_apps/control/spmpc_experiments"
        self.benchmark_dir = self.package_root / "config/benchmark"
        self.profile_baseline_dir = self.package_root / "config/profile_baselines"
        self.report: Dict[str, Any] = {
            "tool": "bench_preflight.py",
            "schema_version": 1,
            "mode": args.mode,
            "planner": args.planner,
            "dry_run": bool(args.dry_run),
            "repo_root": str(self.repo_root),
            "runtime_actions": {
                "starts_ros": False,
                "starts_gazebo": False,
                "kills_processes": False,
                "writes_files": False,
                "changes_cmd_vel_chain": False,
                "changes_spmpc_ocp_inputs": False,
            },
            "checks": [],
            "errors": [],
            "warnings": [],
            "infos": [],
            "main_table_blocked": False,
        }
        self.configs: Dict[str, Dict[str, Any]] = {}

    def check(self) -> Dict[str, Any]:
        if not self.args.dry_run:
            self.error(
                "DRY_RUN_REQUIRED",
                "Phase 0 preflight is implemented as dry-run only; rerun with --dry-run.",
            )

        self.check_benchmark_yaml()
        if self.configs:
            self.check_capability_policy_mapping()
            self.check_common_limits()
            self.check_environment_policy()
            self.check_information_access_policy()
            self.check_external_slosh_observer_policy()
            self.check_slosh_model_monitor_policy()
            self.check_failure_taxonomy()
            self.check_main_table_inclusion()
            self.check_profile_tracking_common()
            self.check_tuning_policy()
            self.check_selected_planner()
            self.check_selected_profile_baseline()
            self.check_comparison_contracts()
        self.check_reuse_assets()
        self.check_optional_future_assets()

        ok = not self.report["errors"]
        self.report["ok"] = ok
        self.report["status"] = "PASS" if ok else "FAIL"
        self.report["stop_main_table"] = (not ok) or bool(self.report.get("main_table_blocked", False))
        return self.report

    def check_benchmark_yaml(self) -> None:
        for filename, top_key in REQUIRED_BENCHMARK_FILES.items():
            path = self.benchmark_dir / filename
            label = f"yaml:{filename}"
            if not path.is_file():
                self.error("YAML_MISSING", f"Missing benchmark config: {path}", path)
                continue
            try:
                with path.open(encoding="utf-8") as handle:
                    data = yaml.safe_load(handle)
            except Exception as exc:  # noqa: BLE001 - report parse failures clearly
                self.error("YAML_PARSE_FAILED", f"Failed to parse {path}: {exc}", path)
                continue
            if not isinstance(data, dict):
                self.error("YAML_NOT_MAPPING", f"{path} must load to a mapping", path)
                continue
            if top_key not in data:
                self.error("YAML_TOP_KEY_MISSING", f"{path} missing top-level key {top_key}", path)
                continue
            self.configs[filename] = data
            self.pass_check(label, f"Loaded {filename} with top-level key {top_key}", path)

    def check_capability_policy_mapping(self) -> None:
        matrix = self.configs.get("capability_matrix.yaml", {}).get("planners", {})
        policy = self.configs.get("information_access_policy.yaml", {}).get("information_access_policy", {})
        categories = set((policy.get("categories") or {}).keys())
        if not categories:
            self.error("POLICY_CATEGORY_MISSING", "No information-access policy categories declared")
            return
        for planner_id, spec in matrix.items():
            if not isinstance(spec, dict):
                self.error("PLANNER_SPEC_INVALID", f"Planner {planner_id} spec must be a mapping")
                continue
            category = spec.get("information_access_category")
            if category not in categories:
                self.error(
                    "INFORMATION_CATEGORY_UNMAPPED",
                    f"Planner {planner_id} maps to {category!r}, not one of {sorted(categories)}",
                )
            if spec.get("online_liquid_feedback") is not False:
                self.error(
                    "ONLINE_LIQUID_FEEDBACK_ENABLED",
                    f"Planner {planner_id} must not enable online liquid feedback for current benchmark policy",
                )
        self.pass_check(
            "capability_policy_mapping",
            "Planner information_access_category values map to declared policy categories",
        )

    def check_common_limits(self) -> None:
        limits = self.configs.get("common_limits.yaml", {}).get("limits", {})
        for key, expected in EXPECTED_COMMON_LIMITS.items():
            value = limits.get(key)
            if value != expected:
                self.error(
                    "COMMON_LIMIT_MISMATCH",
                    f"limits.{key}={value!r}, expected {expected!r}",
                )
        if limits.get("allow_reverse") is not False:
            self.error("REVERSE_NOT_FORBIDDEN", "limits.allow_reverse must be false for fair_common")
        gate = self.configs.get("common_limits.yaml", {}).get("command_gate_policy", {})
        if gate.get("clamp_ratio_warning") != 0.01:
            self.error("COMMAND_GATE_WARNING_THRESHOLD", "command_gate clamp warning threshold must be 0.01")
        if gate.get("clamp_ratio_main_table_exclusion") != 0.05:
            self.error("COMMAND_GATE_EXCLUSION_THRESHOLD", "command_gate exclusion threshold must be 0.05")
        self.pass_check("common_limits", "Common limit targets and command-gate thresholds are present")

    def check_environment_policy(self) -> None:
        env = self.configs.get("common_environment.yaml", {}).get("common_environment", {})
        sim = env.get("simulation", {})
        if self.args.mode == "sim":
            if sim.get("default_root") != "/data/a/scout_sim_replacement":
                self.error("SIM_ROOT_NOT_ISOLATED", "simulation.default_root must be /data/a/scout_sim_replacement")
            if sim.get("allow_current_running_sim_for_formal_gate1") is not False:
                self.error(
                    "CURRENT_SIM_FORMAL_ALLOWED",
                    "Current-running sim must not be allowed for formal Gate 1",
                )
            forbidden = set(sim.get("forbidden_cleanup_patterns") or [])
            if not {"killall", "pkill"}.issubset(forbidden):
                self.error("BROAD_KILL_NOT_FORBIDDEN", "killall/pkill must be forbidden by policy")
            if sim.get("cleanup_policy") != "stop_only_tracked_child_pids":
                self.error("UNSAFE_CLEANUP_POLICY", "cleanup_policy must be stop_only_tracked_child_pids")
        strict = self.configs.get("common_environment.yaml", {}).get("strict_fresh_sim_requirements", {})
        if strict.get("one_case_per_fresh_sim") is not True:
            self.error("FRESH_SIM_ONE_CASE_MISSING", "strict fresh sim must require one case per sim")
        self.pass_check("environment_policy", "Simulation isolation and strict-fresh policy are declared")

    def check_information_access_policy(self) -> None:
        categories = (
            self.configs.get("information_access_policy.yaml", {})
            .get("information_access_policy", {})
            .get("categories", {})
        )
        monitor = categories.get("slosh_monitor", {})
        forbidden = set(monitor.get("forbidden_consumers") or [])
        missing_forbidden = REQUIRED_MONITOR_FORBIDDEN_CONSUMERS - forbidden
        if missing_forbidden:
            self.error(
                "MONITOR_FORBIDDEN_CONSUMER_MISSING",
                f"slosh_monitor.forbidden_consumers missing {sorted(missing_forbidden)}",
            )
        allowed = set(monitor.get("allowed_consumers") or [])
        missing_allowed = REQUIRED_MONITOR_ALLOWED_CONSUMERS - allowed
        if missing_allowed:
            self.error(
                "MONITOR_ALLOWED_CONSUMER_MISSING",
                f"slosh_monitor.allowed_consumers missing {sorted(missing_allowed)}",
            )

        for category_name, category in categories.items():
            if category_name == "slosh_monitor":
                continue
            forbidden_inputs = [str(item) for item in (category.get("forbidden_inputs") or [])]
            mentions_monitor = any(
                "/benchmark/slosh_monitor" in item or "/slosh/" in item
                for item in forbidden_inputs
            )
            if not mentions_monitor:
                self.warn(
                    "MONITOR_TOPIC_NOT_EXPLICITLY_FORBIDDEN",
                    f"Category {category_name} does not explicitly forbid monitor topics",
                )
        self.pass_check("information_access_policy", "Monitor-only consumer boundaries are declared")

    def check_external_slosh_observer_policy(self) -> None:
        observer = self.configs.get("external_slosh_observer_policy.yaml", {}).get("external_slosh_observer", {})
        if observer.get("source_model") != "slosh_models":
            self.error("OBSERVER_SOURCE_MODEL_INVALID", "external_slosh_observer.source_model must be slosh_models")
        if observer.get("mode") != "monitor_only":
            self.error("OBSERVER_NOT_MONITOR_ONLY", "external_slosh_observer.mode must be monitor_only")
        control_usage = observer.get("control_usage", {})
        if control_usage.get("allowed") is not False:
            self.error("OBSERVER_CONTROL_USAGE_ALLOWED", "external observer control_usage.allowed must be false")
        forbidden = set(control_usage.get("forbidden_consumers") or [])
        missing = REQUIRED_MONITOR_FORBIDDEN_CONSUMERS - forbidden
        if missing:
            self.error("OBSERVER_FORBIDDEN_CONSUMER_MISSING", f"external observer missing forbidden consumers {sorted(missing)}")
        self.pass_check("external_slosh_observer_policy", "External observer is declared monitor-only")

    def check_slosh_model_monitor_policy(self) -> None:
        monitor = self.configs.get("slosh_model_monitor_policy.yaml", {}).get("slosh_monitor", {})
        if monitor.get("mode") != "monitor_only":
            self.error("SLOSH_MONITOR_NOT_MONITOR_ONLY", "slosh_monitor.mode must be monitor_only")
        namespace = str(monitor.get("output_namespace", ""))
        if not namespace.startswith("/benchmark/slosh_monitor"):
            self.error("SLOSH_MONITOR_NAMESPACE_INVALID", "slosh_monitor.output_namespace must be under /benchmark/slosh_monitor")
        for key in (
            "consumed_by_planner",
            "consumed_by_command_gate",
            "consumed_by_profile_generator_during_test",
            "consumed_by_spmpc_ocp",
            "consumed_by_cmd_vel_chain",
        ):
            if monitor.get(key) is not False:
                self.error("SLOSH_MONITOR_LEAKAGE_FLAG", f"slosh_monitor.{key} must be false")
        allowed = set(monitor.get("allowed_consumers") or [])
        missing_allowed = REQUIRED_MONITOR_ALLOWED_CONSUMERS - allowed
        if missing_allowed:
            self.error("SLOSH_MONITOR_ALLOWED_CONSUMER_MISSING", f"slosh monitor missing allowed consumers {sorted(missing_allowed)}")
        forbidden = set(monitor.get("forbidden_consumers") or [])
        missing_forbidden = REQUIRED_MONITOR_FORBIDDEN_CONSUMERS - forbidden
        if missing_forbidden:
            self.error("SLOSH_MONITOR_FORBIDDEN_CONSUMER_MISSING", f"slosh monitor missing forbidden consumers {sorted(missing_forbidden)}")
        self.pass_check("slosh_model_monitor_policy", "Slosh monitor leakage flags and consumers are safe")

    def check_failure_taxonomy(self) -> None:
        data = self.configs.get("failure_taxonomy.yaml", {})
        status_codes = data.get("status_codes", {})
        flattened = set()
        for values in status_codes.values():
            if isinstance(values, list):
                flattened.update(str(item) for item in values)
        missing = REQUIRED_FAILURE_CODES - flattened
        if missing:
            self.error("FAILURE_CODE_MISSING", f"failure taxonomy missing required codes {sorted(missing)}")
        blocking = set(data.get("main_table_blocking_codes") or [])
        missing_blocking = REQUIRED_MAIN_TABLE_EXCLUSIONS - blocking
        if missing_blocking:
            self.error("BLOCKING_FAILURE_CODE_MISSING", f"main_table_blocking_codes missing {sorted(missing_blocking)}")
        self.pass_check("failure_taxonomy", "Failure taxonomy includes fairness and monitor blocking codes")

    def check_main_table_inclusion(self) -> None:
        inclusion = self.configs.get("main_table_inclusion.yaml", {}).get("main_table_inclusion", {})
        required = inclusion.get("required", {})
        for key in (
            "strict_fresh",
            "parameters_frozen_before_test",
            "common_limits_loaded",
            "slosh_monitor_enabled_for_sim",
            "slosh_monitor_not_used_for_control",
        ):
            if required.get(key) is not True:
                self.error("MAIN_TABLE_REQUIRED_RULE_MISSING", f"main_table_inclusion.required.{key} must be true")
        thresholds = inclusion.get("thresholds", {})
        if thresholds.get("command_gate_clamp_ratio_max") != 0.05:
            self.error("MAIN_TABLE_CLAMP_THRESHOLD", "main table clamp exclusion threshold must be 0.05")
        if thresholds.get("command_gate_clamp_ratio_warning") != 0.01:
            self.error("MAIN_TABLE_CLAMP_WARNING", "main table clamp warning threshold must be 0.01")
        exclusions = set(inclusion.get("exclusions") or [])
        missing = REQUIRED_MAIN_TABLE_EXCLUSIONS - exclusions
        if missing:
            self.error("MAIN_TABLE_EXCLUSION_MISSING", f"main table exclusions missing {sorted(missing)}")
        self.pass_check("main_table_inclusion", "Main-table strictness and exclusion rules are declared")

    def check_profile_tracking_common(self) -> None:
        profile = self.configs.get("profile_tracking_common.yaml", {})
        tracker = profile.get("profile_tracker", {})
        if tracker.get("implementation") != "scout_local_planner_external_profile":
            self.error("PROFILE_TRACKER_IMPLEMENTATION", "profile tracker must reuse scout_local_planner_external_profile")
        if tracker.get("path_topic") != "/scout/global_path_fixed":
            self.error("PROFILE_TRACKER_PATH_TOPIC", "profile tracker path_topic must be /scout/global_path_fixed")
        if tracker.get("output_cmd_topic") != "/benchmark/cmd_vel_raw":
            self.error("PROFILE_TRACKER_CMD_TOPIC", "profile tracker raw output must be /benchmark/cmd_vel_raw")
        fairness = tracker.get("fairness", {})
        for key in (
            "profile_csv_schema_may_differ",
            "profile_tracker_may_differ",
            "command_gate_may_differ",
            "metrics_analyzer_may_differ",
            "monitor_feedback_during_test_allowed",
        ):
            if fairness.get(key) is not False:
                self.error("PROFILE_FAIRNESS_RULE", f"profile_tracker.fairness.{key} must be false")
        schema = profile.get("profile_csv_schema", {})
        columns = schema.get("required_columns") or []
        if columns != EXPECTED_PROFILE_COLUMNS:
            self.error("PROFILE_CSV_SCHEMA_MISMATCH", "profile CSV columns must match path_profile_utils.py schema")
        source_helper = self.repo_root / str(schema.get("source_helper", ""))
        if not source_helper.is_file():
            self.error("PROFILE_SCHEMA_HELPER_MISSING", f"profile CSV source helper missing: {source_helper}", source_helper)
        self.pass_check("profile_tracking_common", "Profile baseline tracker and CSV schema are shared")

    def check_tuning_policy(self) -> None:
        tuning = self.configs.get("tuning_protocol.yaml", {}).get("tuning", {})
        freeze = tuning.get("freeze", {})
        required_true = [
            "require_parameter_freeze_before_test",
            "require_config_hash",
            "require_git_sha",
            "require_common_limits_hash",
            "require_scenario_hash",
        ]
        for key in required_true:
            if freeze.get(key) is not True:
                self.error("TUNING_FREEZE_RULE_MISSING", f"tuning.freeze.{key} must be true")
        forbidden = set(tuning.get("forbidden") or [])
        if "tune_on_test_set" not in forbidden:
            self.error("TUNE_ON_TEST_NOT_FORBIDDEN", "tuning.forbidden must include tune_on_test_set")
        self.pass_check("tuning_policy", "Parameter freeze and no-test-tuning policy are declared")

    def check_selected_planner(self) -> None:
        matrix = self.configs.get("capability_matrix.yaml", {}).get("planners", {})
        if self.args.planner not in matrix:
            self.error("PLANNER_UNKNOWN", f"Planner {self.args.planner!r} not found in capability_matrix.yaml")
            return
        spec = matrix[self.args.planner]
        eligibility = spec.get("main_table_eligible_if_preflight_passes")
        if eligibility is not True:
            self.report["main_table_blocked"] = True
        if self.args.strict_main_table and eligibility is not True:
            self.error(
                "PLANNER_NOT_MAIN_TABLE_ELIGIBLE",
                f"Planner {self.args.planner} is declared main_table_eligible_if_preflight_passes={eligibility!r}",
            )
        elif eligibility is not True:
            self.warn(
                "PLANNER_NOT_MAIN_TABLE_ELIGIBLE",
                f"Planner {self.args.planner} is not declared as direct main-table eligible: {eligibility!r}",
            )
        if spec.get("ready_to_run") is False:
            self.report["main_table_blocked"] = True
            code = spec.get("dependency_failure_code", "DEPENDENCY_SKIPPED")
            message = f"Planner {self.args.planner} is declared not ready: {code}"
            if self.args.strict_main_table:
                self.error(code, message)
            else:
                self.warn(code, message)
        elif spec.get("ready_to_run") == "conditional":
            self.warn(
                "PLANNER_CONDITIONAL_READY",
                f"Planner {self.args.planner} requires additional runtime/source setup before formal runs",
            )
        self.pass_check("selected_planner", f"Planner {self.args.planner} is declared in capability matrix")

    def check_selected_profile_baseline(self) -> None:
        expected = PROFILE_BASELINES.get(self.args.planner)
        if expected is None:
            return

        config_path = self.profile_baseline_dir / expected["config"]
        if not config_path.is_file():
            self.error("PROFILE_BASELINE_CONFIG_MISSING", f"Profile baseline config missing: {config_path}", config_path)
            return
        try:
            with config_path.open(encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
        except Exception as exc:  # noqa: BLE001 - report parse failures clearly
            self.error("PROFILE_BASELINE_CONFIG_PARSE_FAILED", f"Failed to parse {config_path}: {exc}", config_path)
            return
        if not isinstance(data, dict):
            self.error("PROFILE_BASELINE_CONFIG_INVALID", f"Profile baseline config must be a mapping: {config_path}", config_path)
            return
        profile = data.get("profile_baseline", {})
        if not isinstance(profile, dict):
            self.error("PROFILE_BASELINE_TOP_KEY_MISSING", f"{config_path} missing profile_baseline mapping", config_path)
            return
        if profile.get("id") != self.args.planner:
            self.error("PROFILE_BASELINE_ID_MISMATCH", f"{config_path} id={profile.get('id')!r}, expected {self.args.planner!r}", config_path)
        if profile.get("generator_script") != expected["generator"]:
            self.error(
                "PROFILE_GENERATOR_DECLARATION_MISMATCH",
                f"{self.args.planner} generator_script must be {expected['generator']}",
                config_path,
            )
        if profile.get("method_name") != expected["method_name"]:
            self.error(
                "PROFILE_METHOD_NAME_MISMATCH",
                f"{self.args.planner} method_name must be {expected['method_name']}",
                config_path,
            )
        if profile.get("online") is not False:
            self.error("PROFILE_BASELINE_ONLINE_ENABLED", f"{self.args.planner} must be offline-only", config_path)
        if profile.get("interface_type") != "profile_csv_plus_common_tracker":
            self.error("PROFILE_INTERFACE_INVALID", f"{self.args.planner} must use profile_csv_plus_common_tracker", config_path)
        if profile.get("output_schema") != "profile_tracking_common":
            self.error("PROFILE_OUTPUT_SCHEMA_INVALID", f"{self.args.planner} must use profile_tracking_common schema", config_path)
        if profile.get("main_table_eligible_if_preflight_passes") != "separate_supplementary_table":
            self.error(
                "PROFILE_MAIN_TABLE_ROLE_INVALID",
                f"{self.args.planner} must remain in separate_supplementary_table",
                config_path,
            )

        boundary = profile.get("information_boundary", {})
        for key in (
            "monitor_feedback_during_test_allowed",
            "runtime_profile_regeneration_allowed",
            "ros_topic_subscription_allowed",
            "online_liquid_feedback",
        ):
            if boundary.get(key) is not False:
                self.error("PROFILE_INFORMATION_BOUNDARY_UNSAFE", f"{self.args.planner}.{key} must be false", config_path)

        generator_path = self.repo_root / expected["generator"]
        if not generator_path.is_file():
            self.error("PROFILE_GENERATOR_MISSING", f"Profile generator missing: {generator_path}", generator_path)
            return
        try:
            generator_text = generator_path.read_text(encoding="utf-8")
        except Exception as exc:  # noqa: BLE001 - report parse failures clearly
            self.error("PROFILE_GENERATOR_READ_FAILED", f"Failed to read {generator_path}: {exc}", generator_path)
            return
        forbidden_hits = [token for token in FORBIDDEN_PROFILE_GENERATOR_TOKENS if token in generator_text]
        if forbidden_hits:
            self.error(
                "PROFILE_GENERATOR_FORBIDDEN_RUNTIME_TOKEN",
                f"{self.args.planner} generator contains forbidden runtime/control tokens {forbidden_hits}",
                generator_path,
            )
        self.pass_check("selected_profile_baseline", f"{self.args.planner} profile config and generator boundary are declared", config_path)

    def check_comparison_contracts(self) -> None:
        validator = ComparisonContractValidator(self.repo_root)
        contract_report = validator.check()
        self.report["comparison_contract_summary"] = {
            "status": contract_report.get("status"),
            "checks": len(contract_report.get("checks") or []),
            "warnings": len(contract_report.get("warnings") or []),
            "errors": len(contract_report.get("errors") or []),
        }
        for item in contract_report.get("warnings") or []:
            self.warn(str(item.get("code", "CONTRACT_WARNING")), str(item.get("message", item)))
        for item in contract_report.get("errors") or []:
            self.error(str(item.get("code", "CONTRACT_ERROR")), str(item.get("message", item)))
        if contract_report.get("ok"):
            self.pass_check("comparison_contracts", "Comparison method/profile/monitor/LT-DWA contracts are coherent")

    def check_reuse_assets(self) -> None:
        for name, rel in REQUIRED_CONTROL_PACKAGES.items():
            self.expect_path("package:" + name, self.repo_root / rel, must_be_dir=True)
        for name, rel in REQUIRED_REUSE_ASSETS.items():
            self.expect_path("asset:" + name, self.repo_root / rel, must_be_file=True)

    def check_optional_future_assets(self) -> None:
        mpc_planner = self.repo_root / "src/mpc_planner"
        if mpc_planner.exists():
            self.info("MPC_PLANNER_PRESENT", f"src/mpc_planner exists at {mpc_planner}; still requires its own smoke gate")
        else:
            self.warn("MPC_PLANNER_ABSENT", "src/mpc_planner not found; advanced MPC baseline is dependency-skipped")
        lt_dwa_vendor = self.repo_root / "third_party/LT_DWA"
        lt_dwa_adapter = self.repo_root / "src/scout_apps/control/lt_dwa_adapter"
        lt_dwa_v2_adapter = self.repo_root / "src/scout_apps/control/lt_dwa_v2_adapter"
        if lt_dwa_vendor.is_dir():
            self.info(
                "LT_DWA_REFERENCE_SOURCE_PRESENT",
                f"LT-DWA upstream source is vendored at {lt_dwa_vendor} as source-only reference",
                lt_dwa_vendor,
            )
        else:
            self.warn("LT_DWA_VENDOR_ABSENT", "third_party/LT_DWA not found; restore source reference before paper reporting")
        if lt_dwa_adapter.is_dir():
            self.info(
                "LT_DWA_ADAPTER_PRESENT",
                f"Scout-owned LT-DWA adapter exists at {lt_dwa_adapter}; isolated smoke gate is still required for formal use",
                lt_dwa_adapter,
            )
        else:
            self.warn("LT_DWA_ADAPTER_NOT_READY", "Scout-owned lt_dwa_adapter package not found; LT-DWA baseline is dependency-skipped")
        if lt_dwa_v2_adapter.is_dir():
            self.info(
                "LT_DWA_V2_ADAPTER_PRESENT",
                f"Scout-owned LT-DWA-v2 adapter exists at {lt_dwa_v2_adapter}; strict smoke gate is still required before formal use",
                lt_dwa_v2_adapter,
            )
        else:
            self.warn("LT_DWA_V2_SMOKE_GATE_REQUIRED", "Scout-owned lt_dwa_v2_adapter package not found; LT-DWA-v2 baseline is dependency-skipped")

    def expect_path(
        self,
        label: str,
        path: Path,
        *,
        must_be_file: bool = False,
        must_be_dir: bool = False,
    ) -> None:
        if must_be_file and not path.is_file():
            self.error("REQUIRED_FILE_MISSING", f"Missing required file for {label}: {path}", path)
            return
        if must_be_dir and not path.is_dir():
            self.error("REQUIRED_DIR_MISSING", f"Missing required directory for {label}: {path}", path)
            return
        if not must_be_file and not must_be_dir and not path.exists():
            self.error("REQUIRED_PATH_MISSING", f"Missing required path for {label}: {path}", path)
            return
        self.pass_check(label, f"Found {label}", path)

    def pass_check(self, name: str, message: str, path: Optional[Path] = None) -> None:
        item: Dict[str, Any] = {"name": name, "status": "PASS", "message": message}
        if path is not None:
            item["path"] = str(path)
        self.report["checks"].append(item)

    def error(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["errors"].append(make_item(code, message, path))

    def warn(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["warnings"].append(make_item(code, message, path))

    def info(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["infos"].append(make_item(code, message, path))


def make_item(code: str, message: str, path: Optional[Path] = None) -> Dict[str, Any]:
    item: Dict[str, Any] = {"code": code, "message": message}
    if path is not None:
        item["path"] = str(path)
    return item


def infer_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "src/scout_apps/control/spmpc_experiments").is_dir():
            return parent
    return Path.cwd().resolve()


def emit_report(report: Dict[str, Any], fmt: str) -> None:
    if fmt == "yaml":
        print(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
    else:
        print(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=False))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Dry-run preflight checks for SPMPC comparison benchmark configs."
    )
    parser.add_argument("--mode", choices=("sim", "real"), default="sim")
    parser.add_argument("--planner", default="spmpc_full")
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--dry-run", action="store_true", help="Required in Phase 0; no runtime actions are taken.")
    parser.add_argument("--strict-main-table", action="store_true", help="Treat dependency/eligibility warnings as main-table blockers.")
    parser.add_argument("--format", choices=("json", "yaml"), default="json")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    preflight = Preflight(args)
    report = preflight.check()
    emit_report(report, args.format)
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
