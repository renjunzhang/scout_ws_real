#!/usr/bin/env python3
"""Validate effective fixed-path suite settings against canonical scenario YAML.

This script is read-only: it does not start ROS/Gazebo, publish goals, generate
paths, or modify runtime behavior. It exists to prevent canonical scenario drift
between config files and suite environment/default values.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised only on missing dep
    raise SystemExit("PyYAML missing; install python3-yaml or pip3 install pyyaml") from exc


CANONICAL_PATH_ID = "P2_s_curve_current_start"
CANONICAL_PATH_SOURCE_MODE = "stable_goal"
FLOAT_TOL = 1e-9


class CanonicalScenarioValidator:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.report: Dict[str, Any] = {
            "tool": "bench_validate_canonical_scenario.py",
            "schema_version": 1,
            "scenario_file": str(args.scenario_file),
            "path_id": args.path_id,
            "path_source_mode": args.path_source_mode,
            "runtime_actions": {
                "starts_ros": False,
                "starts_gazebo": False,
                "kills_processes": False,
                "writes_files": False,
                "publishes_goals": False,
                "changes_cmd_vel_chain": False,
                "changes_spmpc_ocp_inputs": False,
            },
            "checks": [],
            "errors": [],
            "warnings": [],
            "infos": [],
        }

    def check(self) -> Dict[str, Any]:
        scenario = self.load_scenario(self.args.scenario_file)
        canonical_context = (
            self.args.path_id == CANONICAL_PATH_ID
            and self.args.path_source_mode == CANONICAL_PATH_SOURCE_MODE
        )
        enforcing = canonical_context and not self.args.allow_noncanonical
        self.report["canonical_context"] = canonical_context
        self.report["strict_canonical_enforced"] = enforcing

        if scenario:
            self.compare_values(scenario, enforcing)

        ok = not self.report["errors"]
        self.report["ok"] = ok
        self.report["status"] = "PASS" if ok else "FAIL"
        self.report["canonical_match"] = not self.report["warnings"] and not self.report["errors"]
        self.report["formal_canonical_allowed"] = ok and self.report["canonical_match"] and canonical_context and not self.args.allow_noncanonical
        return self.report

    def load_scenario(self, path: Path) -> Dict[str, Any]:
        if not path.is_file():
            self.error("CANONICAL_SCENARIO_MISSING", f"Scenario file missing: {path}", path)
            return {}
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001 - report parse failures clearly
            self.error("CANONICAL_SCENARIO_PARSE_FAILED", f"Failed to parse scenario file: {exc}", path)
            return {}
        if not isinstance(data, dict):
            self.error("CANONICAL_SCENARIO_INVALID", "Scenario file must load to a mapping", path)
            return {}
        self.pass_check("canonical_scenario_yaml", "Loaded canonical scenario YAML", path)
        return data

    def compare_values(self, scenario: Dict[str, Any], enforcing: bool) -> None:
        goal = scenario.get("goal", {})
        template = scenario.get("path_template", {})
        limits = scenario.get("common_limits", {})
        if not isinstance(goal, dict) or not isinstance(template, dict) or not isinstance(limits, dict):
            self.error("CANONICAL_SCENARIO_SCHEMA_INVALID", "Scenario must define goal, path_template, and common_limits mappings")
            return

        self.expect_str("goal.frame_id", self.args.goal_frame, goal.get("frame_id"), enforcing)
        self.expect_float("goal.x", self.args.goal_x, goal.get("x"), enforcing)
        self.expect_float("goal.y", self.args.goal_y, goal.get("y"), enforcing)
        self.expect_float("goal.yaw", self.args.goal_yaw, goal.get("yaw"), enforcing)

        self.expect_str("path_template.name", self.args.path_template, template.get("name"), enforcing)
        self.expect_str("path_template.start_heading", self.args.path_start_heading, template.get("start_heading"), enforcing)
        self.expect_float("path_template.amplitude_ratio", self.args.path_amplitude_ratio, template.get("amplitude_ratio"), enforcing)
        self.expect_float("path_template.min_amplitude", self.args.path_min_amplitude, template.get("min_amplitude"), enforcing)
        self.expect_float("path_template.max_amplitude", self.args.path_max_amplitude, template.get("max_amplitude"), enforcing)
        self.expect_str("path_template.side", self.args.path_side, template.get("side"), enforcing)
        self.expect_int("path_template.smooth_iterations", self.args.path_smooth_iterations, template.get("smooth_iterations"), enforcing)
        self.expect_float("path_template.spacing_m", self.args.path_spacing, template.get("spacing_m"), enforcing)

        self.expect_str("common_limits.profile", self.args.limit_profile, limits.get("profile"), enforcing)
        self.expect_float("common_limits.v_max_mps", self.args.v_max, limits.get("v_max_mps"), enforcing)
        self.expect_float("common_limits.omega_max_radps", self.args.omega_max, limits.get("omega_max_radps"), enforcing)
        self.expect_float("common_limits.a_max_mps2", self.args.a_max, limits.get("a_max_mps2"), enforcing)
        self.expect_float("common_limits.alpha_max_radps2", self.args.alpha_max, limits.get("alpha_max_radps2"), enforcing)

        if not enforcing and (self.args.path_id != CANONICAL_PATH_ID or self.args.path_source_mode != CANONICAL_PATH_SOURCE_MODE):
            self.info(
                "CANONICAL_CHECK_DIAGNOSTIC_ONLY",
                "Path id/source mode is not the canonical strict profile-baseline scenario; mismatches are diagnostic warnings only.",
            )
        elif self.args.allow_noncanonical:
            self.info(
                "NONCANONICAL_ALLOWED_BY_ENV",
                "ALLOW_NONCANONICAL_SCENARIO requested; mismatches are warnings and this run is not formal canonical evidence.",
            )

    def expect_str(self, label: str, actual: str, expected: Any, enforcing: bool) -> None:
        if str(actual) == str(expected):
            self.pass_check("scenario:" + label, f"{label} matches canonical value {expected!r}")
        else:
            self.mismatch(label, actual, expected, enforcing)

    def expect_int(self, label: str, actual: int, expected: Any, enforcing: bool) -> None:
        try:
            expected_int = int(expected)
        except Exception:  # noqa: BLE001 - validation helper
            self.mismatch(label, actual, expected, enforcing)
            return
        if int(actual) == expected_int:
            self.pass_check("scenario:" + label, f"{label} matches canonical value {expected_int!r}")
        else:
            self.mismatch(label, actual, expected_int, enforcing)

    def expect_float(self, label: str, actual: float, expected: Any, enforcing: bool) -> None:
        try:
            expected_float = float(expected)
        except Exception:  # noqa: BLE001 - validation helper
            self.mismatch(label, actual, expected, enforcing)
            return
        if math.isclose(float(actual), expected_float, rel_tol=0.0, abs_tol=FLOAT_TOL):
            self.pass_check("scenario:" + label, f"{label} matches canonical value {expected_float!r}")
        else:
            self.mismatch(label, actual, expected_float, enforcing)

    def mismatch(self, label: str, actual: Any, expected: Any, enforcing: bool) -> None:
        item = f"{label}={actual!r}, expected canonical {expected!r}"
        if enforcing:
            self.error("CANONICAL_SCENARIO_MISMATCH", item, self.args.scenario_file)
        else:
            self.warn("NONCANONICAL_SCENARIO_VALUE", item, self.args.scenario_file)

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



def infer_default_scenario_file() -> Path:
    here = Path(__file__).resolve()
    return here.parents[1] / "config/benchmark/canonical_fixed_path_p2.yaml"



def emit_report(report: Dict[str, Any], fmt: str) -> None:
    if fmt == "yaml":
        print(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
    else:
        print(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=False))



def env_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}



def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate suite effective values against canonical fixed-path scenario YAML.")
    parser.add_argument("--scenario-file", type=Path, default=infer_default_scenario_file())
    parser.add_argument("--path-id", default=CANONICAL_PATH_ID)
    parser.add_argument("--path-source-mode", default=CANONICAL_PATH_SOURCE_MODE)
    parser.add_argument("--allow-noncanonical", action="store_true")
    parser.add_argument("--goal-frame", default="map")
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-yaw", type=float, required=True)
    parser.add_argument("--path-template", required=True)
    parser.add_argument("--path-start-heading", required=True)
    parser.add_argument("--path-spacing", type=float, required=True)
    parser.add_argument("--path-amplitude-ratio", type=float, required=True)
    parser.add_argument("--path-min-amplitude", type=float, required=True)
    parser.add_argument("--path-max-amplitude", type=float, required=True)
    parser.add_argument("--path-side", required=True)
    parser.add_argument("--path-smooth-iterations", type=int, required=True)
    parser.add_argument("--limit-profile", default="common_v0p8_w1p2_a0p6_alpha1p2")
    parser.add_argument("--v-max", type=float, required=True)
    parser.add_argument("--omega-max", type=float, required=True)
    parser.add_argument("--a-max", type=float, required=True)
    parser.add_argument("--alpha-max", type=float, required=True)
    parser.add_argument("--format", choices=("json", "yaml"), default="json")
    return parser



def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    report = CanonicalScenarioValidator(args).check()
    emit_report(report, args.format)
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
