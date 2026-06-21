#!/usr/bin/env python3
"""Static comparison-contract checks for SPMPC benchmark setup.

This validator is intentionally read-only. It does not start ROS/Gazebo, launch
planners, generate profiles, write files, build packages, or change runtime
controller behavior. Its purpose is to catch drift between benchmark method
metadata, profile-baseline configs, CSV schema declarations, suite topic
contracts, and source-only advanced-baseline readiness policy.
"""

from __future__ import annotations

import argparse
import ast
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised only on missing dep
    raise SystemExit("PyYAML missing; install python3-yaml or pip3 install pyyaml") from exc


EXPECTED_PROFILE_BASELINES = {
    "hamaguchi_profile": {
        "config": "hamaguchi_profile.yaml",
        "generator": "src/scout_apps/control/scout_profile_baselines/scripts/generate_hamaguchi_profile.py",
        "implementation": "src/scout_apps/control/scout_profile_baselines/scripts/hamaguchi/generate_profile.py",
        "method_name": "HAMAGUCHI_STYLE",
        "category": "slosh_specific_motion_generation",
    },
    "lim_profile": {
        "config": "lim_profile.yaml",
        "generator": "src/scout_apps/control/scout_profile_baselines/scripts/generate_lim_style_profile.py",
        "implementation": "src/scout_apps/control/scout_profile_baselines/scripts/lim/generate_profile.py",
        "method_name": "LIM_STYLE",
        "category": "slosh_specific_offline_retiming",
    },
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

FORBIDDEN_PROFILE_GENERATOR_TOKENS = (
    "rospy.Subscriber",
    "/benchmark/slosh_monitor",
    "/slosh/",
    "/cmd_vel",
)

SUITE_FILES = {
    "spmpc_suite": "src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh",
    "baseline_suite": "src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_baseline_suite.sh",
    "profile_baseline_suite": "src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_profile_baseline_suite.sh",
}

HELPER_FILES = {
    "advanced_profile_common": "src/scout_apps/control/scout_profile_baselines/scripts/common/advanced_profile_common.py",
    "new_path_profile_utils": "src/scout_apps/control/scout_profile_baselines/scripts/common/path_profile_utils.py",
    "legacy_path_profile_utils": "src/scout_apps/control/scout_local_planner/scripts/analysis/path_profile_utils.py",
}

BASELINE_CONFIGS = {
    "teb": "src/scout_apps/control/spmpc_experiments/config/baselines/teb_local_planner_standalone_sim.yaml",
    "dwa": "src/scout_apps/control/spmpc_experiments/config/baselines/dwa_local_planner_standalone_sim.yaml",
    "mpc_local_planner": "src/scout_apps/control/spmpc_experiments/config/baselines/mpc_local_planner_standalone_sim.yaml",
    "lt_dwa": "src/scout_apps/control/spmpc_experiments/config/baselines/lt_dwa_adapter_standalone_sim.yaml",
    "lt_dwa_v2": "src/scout_apps/control/spmpc_experiments/config/baselines/lt_dwa_v2_adapter_standalone_sim.yaml",
}

LT_DWA_VENDOR_ROOT = "third_party/LT_DWA"
LT_DWA_ADAPTER_ROOT = "src/scout_apps/control/lt_dwa_adapter"
LT_DWA_ADAPTER_CODE = "LT_DWA_SMOKE_GATE_REQUIRED"
LT_DWA_V2_ADAPTER_ROOT = "src/scout_apps/control/lt_dwa_v2_adapter"
LT_DWA_V2_ADAPTER_CODE = "LT_DWA_V2_SMOKE_GATE_REQUIRED"


class ComparisonContractValidator:
    def __init__(self, repo_root: Path) -> None:
        self.repo_root = repo_root.resolve()
        self.package_root = self.repo_root / "src/scout_apps/control/spmpc_experiments"
        self.benchmark_dir = self.package_root / "config/benchmark"
        self.profile_dir = self.package_root / "config/profile_baselines"
        self.report: Dict[str, Any] = {
            "tool": "bench_validate_comparison_contracts.py",
            "schema_version": 1,
            "repo_root": str(self.repo_root),
            "runtime_actions": {
                "starts_ros": False,
                "starts_gazebo": False,
                "kills_processes": False,
                "writes_files": False,
                "builds_packages": False,
                "changes_cmd_vel_chain": False,
                "changes_spmpc_ocp_inputs": False,
            },
            "checks": [],
            "errors": [],
            "warnings": [],
            "infos": [],
        }

    def check(self) -> Dict[str, Any]:
        capability = self.load_yaml(self.benchmark_dir / "capability_matrix.yaml", "capability_matrix.yaml")
        common_limits = self.load_yaml(self.benchmark_dir / "common_limits.yaml", "common_limits.yaml")
        common_environment = self.load_yaml(self.benchmark_dir / "common_environment.yaml", "common_environment.yaml")
        profile_tracking = self.load_yaml(self.benchmark_dir / "profile_tracking_common.yaml", "profile_tracking_common.yaml")
        failure_taxonomy = self.load_yaml(self.benchmark_dir / "failure_taxonomy.yaml", "failure_taxonomy.yaml")
        main_table = self.load_yaml(self.benchmark_dir / "main_table_inclusion.yaml", "main_table_inclusion.yaml")
        monitor_policy = self.load_yaml(self.benchmark_dir / "slosh_model_monitor_policy.yaml", "slosh_model_monitor_policy.yaml")

        if capability:
            self.check_profile_baseline_metadata(capability)
            self.check_lt_dwa_policy(capability, failure_taxonomy, main_table)
            self.check_lt_dwa_v2_policy(capability, failure_taxonomy, main_table)
        if profile_tracking:
            self.check_profile_csv_contract(profile_tracking)
        if common_limits and common_environment:
            self.check_runtime_baseline_fairness(common_limits, common_environment)
        if monitor_policy:
            self.check_suite_monitor_topics(monitor_policy)

        ok = not self.report["errors"]
        self.report["ok"] = ok
        self.report["status"] = "PASS" if ok else "FAIL"
        return self.report

    def load_yaml(self, path: Path, label: str) -> Dict[str, Any]:
        if not path.is_file():
            self.error("CONTRACT_YAML_MISSING", f"Missing contract YAML {label}: {path}", path)
            return {}
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001 - report parse failures clearly
            self.error("CONTRACT_YAML_PARSE_FAILED", f"Failed to parse {label}: {exc}", path)
            return {}
        if not isinstance(data, dict):
            self.error("CONTRACT_YAML_NOT_MAPPING", f"{label} must load to a mapping", path)
            return {}
        self.pass_check(f"yaml:{label}", f"Loaded {label}", path)
        return data

    def check_profile_baseline_metadata(self, capability: Dict[str, Any]) -> None:
        planners = capability.get("planners", {})
        if not isinstance(planners, dict):
            self.error("CAPABILITY_PLANNERS_INVALID", "capability_matrix.planners must be a mapping")
            return

        for method_id, expected in EXPECTED_PROFILE_BASELINES.items():
            cap = planners.get(method_id)
            if not isinstance(cap, dict):
                self.error("PROFILE_METHOD_MISSING_IN_CAPABILITY", f"{method_id} missing from capability_matrix.yaml")
                continue
            if cap.get("implementation") != "scout_profile_baselines_generated_external_profile":
                self.error(
                    "PROFILE_CAPABILITY_IMPLEMENTATION_MISMATCH",
                    f"{method_id} capability implementation must be scout_profile_baselines_generated_external_profile",
                )
            if cap.get("interface_type") != "profile_csv_plus_common_tracker":
                self.error("PROFILE_CAPABILITY_INTERFACE_MISMATCH", f"{method_id} must use profile_csv_plus_common_tracker")
            if cap.get("online") is not False or cap.get("online_liquid_feedback") is not False:
                self.error("PROFILE_CAPABILITY_ONLINE_LEAKAGE", f"{method_id} must remain offline and without online liquid feedback")
            if cap.get("cmd_vel_output") != "via_common_profile_tracker":
                self.error("PROFILE_CAPABILITY_CMD_CHAIN_MISMATCH", f"{method_id} cmd_vel_output must be via_common_profile_tracker")
            if cap.get("main_table_eligible_if_preflight_passes") != "separate_supplementary_table":
                self.error("PROFILE_CAPABILITY_TABLE_ROLE_MISMATCH", f"{method_id} must stay in separate_supplementary_table")

            config_path = self.profile_dir / expected["config"]
            config = self.load_yaml(config_path, f"profile_baselines/{expected['config']}")
            profile = config.get("profile_baseline", {}) if config else {}
            if not isinstance(profile, dict):
                self.error("PROFILE_CONFIG_INVALID", f"{config_path} missing profile_baseline mapping", config_path)
                continue
            self.expect_equal(profile.get("id"), method_id, "PROFILE_CONFIG_ID_MISMATCH", f"{method_id} id", config_path)
            self.expect_equal(profile.get("category"), expected["category"], "PROFILE_CONFIG_CATEGORY_MISMATCH", f"{method_id} category", config_path)
            self.expect_equal(profile.get("generator_script"), expected["generator"], "PROFILE_GENERATOR_PATH_MISMATCH", f"{method_id} generator_script", config_path)
            self.expect_equal(profile.get("implementation_script"), expected["implementation"], "PROFILE_IMPLEMENTATION_PATH_MISMATCH", f"{method_id} implementation_script", config_path)
            self.expect_equal(profile.get("method_name"), expected["method_name"], "PROFILE_METHOD_NAME_MISMATCH", f"{method_id} method_name", config_path)
            self.expect_equal(profile.get("interface_type"), "profile_csv_plus_common_tracker", "PROFILE_INTERFACE_MISMATCH", f"{method_id} interface_type", config_path)
            self.expect_equal(profile.get("output_schema"), "profile_tracking_common", "PROFILE_OUTPUT_SCHEMA_MISMATCH", f"{method_id} output_schema", config_path)
            self.expect_equal(profile.get("main_table_eligible_if_preflight_passes"), "separate_supplementary_table", "PROFILE_TABLE_ROLE_MISMATCH", f"{method_id} table role", config_path)

            boundary = profile.get("information_boundary", {})
            if not isinstance(boundary, dict):
                self.error("PROFILE_INFORMATION_BOUNDARY_MISSING", f"{method_id} missing information_boundary", config_path)
            else:
                for key in (
                    "monitor_feedback_during_test_allowed",
                    "runtime_profile_regeneration_allowed",
                    "ros_topic_subscription_allowed",
                    "online_liquid_feedback",
                ):
                    if boundary.get(key) is not False:
                        self.error("PROFILE_INFORMATION_BOUNDARY_UNSAFE", f"{method_id}.{key} must be false", config_path)

            for role, rel in (("entrypoint", expected["generator"]), ("implementation", expected["implementation"])):
                generator_path = self.repo_root / rel
                if not generator_path.is_file():
                    self.error("PROFILE_GENERATOR_MISSING", f"{method_id} {role} script missing: {generator_path}", generator_path)
                    continue
                source_text = generator_path.read_text(encoding="utf-8", errors="replace")
                hits = [token for token in FORBIDDEN_PROFILE_GENERATOR_TOKENS if token in source_text]
                if hits:
                    self.error("PROFILE_GENERATOR_FORBIDDEN_RUNTIME_TOKEN", f"{method_id} {role} script contains forbidden tokens {hits}", generator_path)
                else:
                    self.pass_check(f"profile_generator_boundary:{method_id}:{role}", f"{method_id} {role} script has no forbidden runtime/control tokens", generator_path)

    def check_profile_csv_contract(self, profile_tracking: Dict[str, Any]) -> None:
        schema = profile_tracking.get("profile_csv_schema", {})
        columns = schema.get("required_columns") or []
        if columns != EXPECTED_PROFILE_COLUMNS:
            self.error("PROFILE_TRACKING_COLUMNS_MISMATCH", "profile_tracking_common required_columns drifted from expected profile CSV schema")
        else:
            self.pass_check("profile_tracking_columns", "profile_tracking_common columns match expected schema")

        source_helper = self.repo_root / str(schema.get("source_helper", ""))
        if not source_helper.is_file():
            self.error("PROFILE_SCHEMA_HELPER_MISSING", f"profile_tracking_common source_helper missing: {source_helper}", source_helper)
        else:
            self.pass_check("profile_schema_helper", "profile_tracking_common source_helper exists", source_helper)

        advanced_path = self.repo_root / HELPER_FILES["advanced_profile_common"]
        advanced_columns = self.extract_assignment_list(advanced_path, "PROFILE_COLUMNS")
        if advanced_columns is None:
            self.error("PROFILE_COLUMNS_CONSTANT_MISSING", "advanced_profile_common.py must define PROFILE_COLUMNS", advanced_path)
        elif advanced_columns != EXPECTED_PROFILE_COLUMNS:
            self.error("PROFILE_COLUMNS_CONSTANT_MISMATCH", "advanced_profile_common.PROFILE_COLUMNS drifted from expected schema", advanced_path)
        else:
            self.pass_check("advanced_profile_columns", "advanced_profile_common.PROFILE_COLUMNS matches expected schema", advanced_path)

        new_fieldnames = self.extract_csv_fieldnames(self.repo_root / HELPER_FILES["new_path_profile_utils"])
        legacy_fieldnames = self.extract_csv_fieldnames(self.repo_root / HELPER_FILES["legacy_path_profile_utils"])
        for label, fieldnames, rel in (
            ("new_path_profile_utils", new_fieldnames, HELPER_FILES["new_path_profile_utils"]),
            ("legacy_path_profile_utils", legacy_fieldnames, HELPER_FILES["legacy_path_profile_utils"]),
        ):
            path = self.repo_root / rel
            if fieldnames is None:
                self.error("PROFILE_CSV_FIELDNAMES_MISSING", f"{label} must define csv.DictWriter fieldnames", path)
            elif fieldnames != EXPECTED_PROFILE_COLUMNS:
                self.error("PROFILE_CSV_FIELDNAMES_MISMATCH", f"{label} CSV fieldnames drifted from expected schema", path)
            else:
                self.pass_check(f"profile_csv_fieldnames:{label}", f"{label} CSV fieldnames match expected schema", path)

    def check_suite_monitor_topics(self, monitor_policy: Dict[str, Any]) -> None:
        monitor = monitor_policy.get("slosh_monitor", {})
        required = [str(item) for item in (monitor.get("required_recorded_topics_for_sim_strict") or [])]
        legacy = [str(item) for item in (monitor.get("transition_compatible_recorded_topics") or [])]
        if not required:
            self.error("MONITOR_REQUIRED_TOPICS_MISSING", "slosh_model_monitor_policy must declare required_recorded_topics_for_sim_strict")
            return

        for label, rel in SUITE_FILES.items():
            path = self.repo_root / rel
            if not path.is_file():
                self.error("SUITE_SCRIPT_MISSING", f"Missing suite script {label}: {path}", path)
                continue
            text = path.read_text(encoding="utf-8", errors="replace")
            missing_required = [topic for topic in required if topic not in text]
            missing_legacy = [topic for topic in legacy if topic not in text]
            if missing_required:
                self.warn(
                    "SUITE_BENCHMARK_MONITOR_TOPICS_MISSING",
                    f"{label} does not record benchmark monitor topics {missing_required}; strict sim must provide them before formal table use",
                    path,
                )
            else:
                self.pass_check(f"suite_monitor_topics:{label}", f"{label} records benchmark monitor topics", path)
            if missing_legacy:
                self.warn(
                    "SUITE_LEGACY_MONITOR_TOPICS_MISSING",
                    f"{label} no longer records transition legacy monitor topics {missing_legacy}",
                    path,
                )

    def check_lt_dwa_policy(self, capability: Dict[str, Any], failure_taxonomy: Dict[str, Any], main_table: Dict[str, Any]) -> None:
        planners = capability.get("planners", {})
        lt = planners.get("lt_dwa", {}) if isinstance(planners, dict) else {}
        if not isinstance(lt, dict):
            self.error("LT_DWA_CAPABILITY_MISSING", "lt_dwa missing from capability_matrix.yaml")
            return
        self.expect_equal(lt.get("implementation"), LT_DWA_ADAPTER_ROOT, "LT_DWA_IMPLEMENTATION_PATH_MISMATCH", "lt_dwa implementation")
        self.expect_equal(lt.get("reference_source"), LT_DWA_VENDOR_ROOT, "LT_DWA_REFERENCE_SOURCE_MISMATCH", "lt_dwa reference_source")
        self.expect_equal(lt.get("interface_type"), "scout_owned_ros_adapter", "LT_DWA_INTERFACE_MISMATCH", "lt_dwa interface_type")
        self.expect_equal(lt.get("present_in_workspace"), "adapter_present", "LT_DWA_PRESENT_STATUS_MISMATCH", "lt_dwa present_in_workspace")
        self.expect_equal(lt.get("ready_to_run"), "conditional", "LT_DWA_READY_STATUS_MISMATCH", "lt_dwa ready_to_run")
        self.expect_equal(lt.get("dependency_failure_code"), LT_DWA_ADAPTER_CODE, "LT_DWA_FAILURE_CODE_MISMATCH", "lt_dwa dependency_failure_code")
        self.expect_equal(lt.get("smoke_gate_required_before_runnable"), True, "LT_DWA_SMOKE_GATE_FLAG_MISMATCH", "lt_dwa smoke gate flag")
        self.expect_equal(lt.get("liquid_model_access"), "none", "LT_DWA_LIQUID_ACCESS_MISMATCH", "lt_dwa liquid_model_access")
        self.expect_equal(lt.get("online_liquid_feedback"), False, "LT_DWA_ONLINE_LIQUID_FEEDBACK", "lt_dwa online_liquid_feedback")
        self.expect_equal(lt.get("main_table_eligible_if_preflight_passes"), False, "LT_DWA_TABLE_ELIGIBILITY_MISMATCH", "lt_dwa table eligibility")

        vendor = self.repo_root / LT_DWA_VENDOR_ROOT
        if vendor.is_dir():
            self.pass_check("lt_dwa_vendor_reference", "LT-DWA vendor root exists outside catkin src as reference source", vendor)
        else:
            self.warn("LT_DWA_VENDOR_ABSENT", "LT-DWA vendor root is absent; paper-reference source should be restored", vendor)

        adapter = self.repo_root / LT_DWA_ADAPTER_ROOT
        required_adapter_files = [
            "package.xml",
            "CMakeLists.txt",
            "launch/lt_dwa_adapter.launch",
            "config/lt_dwa_adapter_sim.yaml",
            "src/lt_dwa_adapter_node.cpp",
            "src/lt_dwa_adapter_ros.cpp",
            "src/lt_dwa_planner.cpp",
        ]
        for rel in required_adapter_files:
            path = adapter / rel
            if path.is_file():
                self.pass_check(f"lt_dwa_adapter_file:{rel}", f"LT-DWA adapter file exists: {rel}", path)
            else:
                self.error("LT_DWA_ADAPTER_FILE_MISSING", f"LT-DWA adapter file missing: {path}", path)

        benchmark_launch = self.repo_root / "src/scout_apps/control/spmpc_experiments/launch/sim/run_lt_dwa_fixed_path_sim.launch"
        benchmark_config = self.repo_root / BASELINE_CONFIGS["lt_dwa"]
        if benchmark_launch.is_file():
            self.pass_check("lt_dwa_benchmark_launch", "LT-DWA benchmark wrapper launch exists", benchmark_launch)
        else:
            self.error("LT_DWA_BENCHMARK_LAUNCH_MISSING", f"LT-DWA benchmark wrapper launch missing: {benchmark_launch}", benchmark_launch)
        lt_config = self.load_yaml(benchmark_config, "baselines/lt_dwa_adapter_standalone_sim.yaml")
        if lt_config:
            self.expect_equal(lt_config.get("publish_cmd_vel"), False, "LT_DWA_DEFAULT_CMD_VEL_UNSAFE", "lt_dwa publish_cmd_vel", benchmark_config)
            self.expect_equal(lt_config.get("allow_reverse"), False, "LT_DWA_REVERSE_NOT_ALLOWED", "lt_dwa allow_reverse", benchmark_config)
            for token in ("/benchmark/slosh_monitor", "/slosh/"):
                for rel in ("launch/lt_dwa_adapter.launch", "config/lt_dwa_adapter_sim.yaml", "src/lt_dwa_adapter_ros.cpp"):
                    path = adapter / rel
                    if path.is_file() and token in path.read_text(encoding="utf-8", errors="replace"):
                        self.error("LT_DWA_MONITOR_INPUT_LEAKAGE", f"LT-DWA adapter runtime file {rel} contains forbidden monitor token {token}", path)
            self.pass_check("lt_dwa_default_shadow_only", "LT-DWA benchmark config defaults to shadow-only and has no monitor input tokens", benchmark_config)

        flattened_codes = set()
        status_codes = failure_taxonomy.get("status_codes", {}) if isinstance(failure_taxonomy, dict) else {}
        if isinstance(status_codes, dict):
            for values in status_codes.values():
                if isinstance(values, list):
                    flattened_codes.update(str(item) for item in values)
        if LT_DWA_ADAPTER_CODE not in flattened_codes:
            self.error("LT_DWA_FAILURE_TAXONOMY_MISSING", f"failure_taxonomy missing {LT_DWA_ADAPTER_CODE}")
        if LT_DWA_ADAPTER_CODE not in set(failure_taxonomy.get("main_table_blocking_codes") or []):
            self.error("LT_DWA_BLOCKING_CODE_MISSING", f"main_table_blocking_codes missing {LT_DWA_ADAPTER_CODE}")
        inclusions = main_table.get("main_table_inclusion", {}) if isinstance(main_table, dict) else {}
        if LT_DWA_ADAPTER_CODE not in set(inclusions.get("exclusions") or []):
            self.error("LT_DWA_MAIN_TABLE_EXCLUSION_MISSING", f"main_table_inclusion exclusions missing {LT_DWA_ADAPTER_CODE}")
        if not any(item.get("code") == "LT_DWA_FAILURE_TAXONOMY_MISSING" for item in self.report["errors"]):
            self.pass_check("lt_dwa_failure_codes", "LT-DWA smoke-gate-required code is declared and blocking")

    def check_lt_dwa_v2_policy(self, capability: Dict[str, Any], failure_taxonomy: Dict[str, Any], main_table: Dict[str, Any]) -> None:
        planners = capability.get("planners", {})
        lt_v2 = planners.get("lt_dwa_v2", {}) if isinstance(planners, dict) else {}
        if not isinstance(lt_v2, dict):
            self.error("LT_DWA_V2_CAPABILITY_MISSING", "lt_dwa_v2 missing from capability_matrix.yaml")
            return
        self.expect_equal(lt_v2.get("implementation"), LT_DWA_V2_ADAPTER_ROOT, "LT_DWA_V2_IMPLEMENTATION_PATH_MISMATCH", "lt_dwa_v2 implementation")
        self.expect_equal(lt_v2.get("reference_source"), LT_DWA_VENDOR_ROOT, "LT_DWA_V2_REFERENCE_SOURCE_MISMATCH", "lt_dwa_v2 reference_source")
        self.expect_equal(lt_v2.get("interface_type"), "scout_owned_ros_adapter", "LT_DWA_V2_INTERFACE_MISMATCH", "lt_dwa_v2 interface_type")
        self.expect_equal(lt_v2.get("present_in_workspace"), "adapter_present", "LT_DWA_V2_PRESENT_STATUS_MISMATCH", "lt_dwa_v2 present_in_workspace")
        self.expect_equal(lt_v2.get("ready_to_run"), "conditional", "LT_DWA_V2_READY_STATUS_MISMATCH", "lt_dwa_v2 ready_to_run")
        self.expect_equal(lt_v2.get("dependency_failure_code"), LT_DWA_V2_ADAPTER_CODE, "LT_DWA_V2_FAILURE_CODE_MISMATCH", "lt_dwa_v2 dependency_failure_code")
        self.expect_equal(lt_v2.get("smoke_gate_required_before_runnable"), True, "LT_DWA_V2_SMOKE_GATE_FLAG_MISMATCH", "lt_dwa_v2 smoke gate flag")
        self.expect_equal(lt_v2.get("liquid_model_access"), "none", "LT_DWA_V2_LIQUID_ACCESS_MISMATCH", "lt_dwa_v2 liquid_model_access")
        self.expect_equal(lt_v2.get("online_liquid_feedback"), False, "LT_DWA_V2_ONLINE_LIQUID_FEEDBACK", "lt_dwa_v2 online_liquid_feedback")
        self.expect_equal(lt_v2.get("main_table_eligible_if_preflight_passes"), False, "LT_DWA_V2_TABLE_ELIGIBILITY_MISMATCH", "lt_dwa_v2 table eligibility")

        adapter = self.repo_root / LT_DWA_V2_ADAPTER_ROOT
        required_adapter_files = [
            "package.xml",
            "CMakeLists.txt",
            "launch/lt_dwa_v2_adapter.launch",
            "config/lt_dwa_v2_adapter_sim.yaml",
            "src/lt_dwa_v2_adapter_node.cpp",
            "src/ros/lt_dwa_v2_adapter_ros.cpp",
            "src/search/lt_dwa_v2_planner.cpp",
        ]
        for rel in required_adapter_files:
            path = adapter / rel
            if path.is_file():
                self.pass_check(f"lt_dwa_v2_adapter_file:{rel}", f"LT-DWA-v2 adapter file exists: {rel}", path)
            else:
                self.error("LT_DWA_V2_ADAPTER_FILE_MISSING", f"LT-DWA-v2 adapter file missing: {path}", path)

        benchmark_launch = self.repo_root / "src/scout_apps/control/spmpc_experiments/launch/sim/run_lt_dwa_v2_fixed_path_sim.launch"
        benchmark_config = self.repo_root / BASELINE_CONFIGS["lt_dwa_v2"]
        if benchmark_launch.is_file():
            self.pass_check("lt_dwa_v2_benchmark_launch", "LT-DWA-v2 benchmark wrapper launch exists", benchmark_launch)
        else:
            self.error("LT_DWA_V2_BENCHMARK_LAUNCH_MISSING", f"LT-DWA-v2 benchmark wrapper launch missing: {benchmark_launch}", benchmark_launch)
        lt_config = self.load_yaml(benchmark_config, "baselines/lt_dwa_v2_adapter_standalone_sim.yaml")
        if lt_config:
            runtime = lt_config.get("runtime", {})
            topics = lt_config.get("topics", {})
            limits = lt_config.get("limits", {})
            self.expect_equal(runtime.get("publish_cmd_vel"), False, "LT_DWA_V2_DEFAULT_CMD_VEL_UNSAFE", "lt_dwa_v2 runtime.publish_cmd_vel", benchmark_config)
            if topics.get("cmd_vel_topic") == "/cmd_vel":
                self.error("LT_DWA_V2_DEFAULT_CMD_VEL_UNSAFE", "lt_dwa_v2 default cmd_vel_topic must not be /cmd_vel", benchmark_config)
            self.expect_equal(limits.get("allow_reverse"), False, "LT_DWA_V2_REVERSE_NOT_ALLOWED", "lt_dwa_v2 limits.allow_reverse", benchmark_config)
            for token in ("/benchmark/slosh_monitor", "/slosh/"):
                for rel in (
                    "launch/lt_dwa_v2_adapter.launch",
                    "config/lt_dwa_v2_adapter_sim.yaml",
                    "src/ros/lt_dwa_v2_adapter_ros.cpp",
                    "src/search/lt_dwa_v2_planner.cpp",
                ):
                    path = adapter / rel
                    if path.is_file() and token in path.read_text(encoding="utf-8", errors="replace"):
                        self.error("LT_DWA_V2_MONITOR_INPUT_LEAKAGE", f"LT-DWA-v2 adapter runtime file {rel} contains forbidden monitor token {token}", path)
            self.pass_check("lt_dwa_v2_default_shadow_only", "LT-DWA-v2 benchmark config defaults to shadow-only and has no monitor input tokens", benchmark_config)

        flattened_codes = set()
        status_codes = failure_taxonomy.get("status_codes", {}) if isinstance(failure_taxonomy, dict) else {}
        if isinstance(status_codes, dict):
            for values in status_codes.values():
                if isinstance(values, list):
                    flattened_codes.update(str(item) for item in values)
        before_errors = len(self.report["errors"])
        if LT_DWA_V2_ADAPTER_CODE not in flattened_codes:
            self.error("LT_DWA_V2_FAILURE_TAXONOMY_MISSING", f"failure_taxonomy missing {LT_DWA_V2_ADAPTER_CODE}")
        if LT_DWA_V2_ADAPTER_CODE not in set(failure_taxonomy.get("main_table_blocking_codes") or []):
            self.error("LT_DWA_V2_BLOCKING_CODE_MISSING", f"main_table_blocking_codes missing {LT_DWA_V2_ADAPTER_CODE}")
        inclusions = main_table.get("main_table_inclusion", {}) if isinstance(main_table, dict) else {}
        if LT_DWA_V2_ADAPTER_CODE not in set(inclusions.get("exclusions") or []):
            self.error("LT_DWA_V2_MAIN_TABLE_EXCLUSION_MISSING", f"main_table_inclusion exclusions missing {LT_DWA_V2_ADAPTER_CODE}")
        if len(self.report["errors"]) == before_errors:
            self.pass_check("lt_dwa_v2_failure_codes", "LT-DWA-v2 smoke-gate-required code is declared and blocking")

    def check_runtime_baseline_fairness(self, common_limits: Dict[str, Any], common_environment: Dict[str, Any]) -> None:
        limits = common_limits.get("limits", {})
        goal = common_environment.get("common_environment", {}).get("goal", {})
        if not isinstance(limits, dict) or not isinstance(goal, dict):
            self.error("COMMON_FAIRNESS_SCHEMA_INVALID", "common_limits.limits and common_environment.goal must be mappings")
            return

        expected = {
            "v_max": float(limits.get("v_max_mps")),
            "omega_max": float(limits.get("omega_max_radps")),
            "a_max": float(limits.get("a_max_mps2")),
            "alpha_max": float(limits.get("alpha_max_radps2")),
            "allow_reverse": bool(limits.get("allow_reverse")),
            "xy_goal_tolerance": float(goal.get("xy_tolerance_m")),
            "yaw_goal_tolerance": float(goal.get("yaw_tolerance_rad")),
        }

        for planner, rel in BASELINE_CONFIGS.items():
            path = self.repo_root / rel
            cfg = self.load_yaml(path, f"baselines/{path.name}")
            if not cfg:
                continue
            if planner == "teb":
                root = cfg.get("TebLocalPlannerROS", {})
                mapping = {
                    "v_max": "max_vel_x",
                    "omega_max": "max_vel_theta",
                    "a_max": "acc_lim_x",
                    "alpha_max": "acc_lim_theta",
                    "xy_goal_tolerance": "xy_goal_tolerance",
                    "yaw_goal_tolerance": "yaw_goal_tolerance",
                }
                reverse_value = root.get("max_vel_x_backwards")
            elif planner == "dwa":
                root = cfg.get("DWAPlannerROS", {})
                mapping = {
                    "v_max": "max_vel_x",
                    "omega_max": "max_vel_theta",
                    "a_max": "acc_lim_x",
                    "alpha_max": "acc_lim_theta",
                    "xy_goal_tolerance": "xy_goal_tolerance",
                    "yaw_goal_tolerance": "yaw_goal_tolerance",
                }
                reverse_value = -1.0 * float(root.get("min_vel_x", 0.0))
            elif planner == "lt_dwa":
                root = cfg
                mapping = {
                    "v_max": "v_max_mps",
                    "omega_max": "omega_max_radps",
                    "a_max": "a_max_mps2",
                    "alpha_max": "alpha_max_radps2",
                    "xy_goal_tolerance": "xy_goal_tolerance",
                    "yaw_goal_tolerance": "yaw_goal_tolerance",
                }
                reverse_value = 0.0 if root.get("allow_reverse") is False else 1.0
            elif planner == "lt_dwa_v2":
                root = cfg
                mapping = {
                    "v_max": "limits/v_max_mps",
                    "omega_max": "limits/omega_max_radps",
                    "a_max": "limits/a_max_mps2",
                    "alpha_max": "limits/alpha_max_radps2",
                    "xy_goal_tolerance": "goal/xy_tolerance_m",
                    "yaw_goal_tolerance": "goal/yaw_tolerance_rad",
                }
                reverse_value = 0.0 if self.nested_get(root, "limits/allow_reverse") is False else 1.0
            else:
                root = cfg.get("MpcLocalPlannerROS", {})
                mapping = {
                    "v_max": "robot/unicycle/max_vel_x",
                    "omega_max": "robot/unicycle/max_vel_theta",
                    "a_max": "robot/unicycle/acc_lim_x",
                    "alpha_max": "robot/unicycle/acc_lim_theta",
                    "xy_goal_tolerance": "controller/xy_goal_tolerance",
                    "yaw_goal_tolerance": "controller/yaw_goal_tolerance",
                }
                reverse_value = self.nested_get(root, "robot/unicycle/max_vel_x_backwards")

            if not isinstance(root, dict):
                self.error("BASELINE_CONFIG_ROOT_MISSING", f"{planner} config root missing", path)
                continue
            before_errors = len(self.report["errors"])
            for label, key_path in mapping.items():
                self.expect_float(self.nested_get(root, key_path), expected[label], "BASELINE_COMMON_LIMIT_MISMATCH", f"{planner}.{key_path}", path)
            if expected["allow_reverse"] is False:
                self.expect_float(reverse_value, 0.0, "BASELINE_REVERSE_NOT_ALLOWED", f"{planner} reverse velocity allowance", path)
            if len(self.report["errors"]) == before_errors:
                self.pass_check(f"baseline_fairness:{planner}", f"{planner} runtime config matches common limits and goal tolerances", path)

    def nested_get(self, data: Dict[str, Any], key_path: str) -> Any:
        cur: Any = data
        for key in key_path.split("/"):
            if not isinstance(cur, dict):
                return None
            cur = cur.get(key)
        return cur

    def expect_float(self, actual: Any, expected: float, code: str, label: str, path: Optional[Path] = None) -> None:
        try:
            actual_float = float(actual)
        except Exception:  # noqa: BLE001 - report validation failures clearly
            self.error(code, f"{label}={actual!r}, expected {expected!r}", path)
            return
        if abs(actual_float - expected) > 1e-9:
            self.error(code, f"{label}={actual_float!r}, expected {expected!r}", path)

    def extract_assignment_list(self, path: Path, name: str) -> Optional[List[str]]:
        if not path.is_file():
            return None
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError:
            return None
        for node in tree.body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id == name:
                        try:
                            value = ast.literal_eval(node.value)
                        except Exception:  # noqa: BLE001 - validation helper
                            return None
                        return [str(item) for item in value] if isinstance(value, list) else None
        return None

    def extract_csv_fieldnames(self, path: Path) -> Optional[List[str]]:
        if not path.is_file():
            return None
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError:
            return None
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            for keyword in node.keywords:
                if keyword.arg == "fieldnames":
                    try:
                        value = ast.literal_eval(keyword.value)
                    except Exception:  # noqa: BLE001 - validation helper
                        continue
                    if isinstance(value, list):
                        return [str(item) for item in value]
        return None

    def expect_equal(
        self,
        actual: Any,
        expected: Any,
        code: str,
        label: str,
        path: Optional[Path] = None,
    ) -> None:
        if actual != expected:
            self.error(code, f"{label}={actual!r}, expected {expected!r}", path)

    def pass_check(self, name: str, message: str, path: Optional[Path] = None) -> None:
        item: Dict[str, Any] = {"name": name, "status": "PASS", "message": message}
        if path is not None:
            item["path"] = str(path)
        self.report["checks"].append(item)

    def error(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["errors"].append(make_item(code, message, path))

    def warn(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["warnings"].append(make_item(code, message, path))



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
    parser = argparse.ArgumentParser(description="Validate static comparison benchmark contracts.")
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--format", choices=("json", "yaml"), default="json")
    return parser



def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    repo_root = args.repo_root.resolve() if args.repo_root else infer_repo_root()
    report = ComparisonContractValidator(repo_root).check()
    emit_report(report, args.format)
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
