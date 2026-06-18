#!/usr/bin/env python3
"""Dry-run endpoint/template checker for profile-baseline fixed-path runs.

The checker is intentionally static: it does not start ROS/Gazebo, does not kill
processes, and does not write files. It verifies that a Hamaguchi/Lim profile
run is declared against the canonical fixed-path endpoint/template used in the
previous SPMPC simulation records.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised only on missing dep
    raise SystemExit("PyYAML missing; install python3-yaml or pip3 install pyyaml") from exc


DEFAULT_CONFIG = "src/scout_apps/control/spmpc_experiments/config/benchmark/canonical_fixed_path_p2.yaml"
FLOAT_TOL = 1e-9


class EndpointCheck:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.repo_root = args.repo_root.resolve() if args.repo_root else infer_repo_root()
        self.report: Dict[str, Any] = {
            "tool": "bench_check_profile_endpoint.py",
            "schema_version": 1,
            "repo_root": str(self.repo_root),
            "runtime_actions": {
                "starts_ros": False,
                "starts_gazebo": False,
                "kills_processes": False,
                "writes_files": False,
                "changes_cmd_vel_chain": False,
                "changes_spmpc_ocp_inputs": False,
            },
            "canonical": {},
            "actual": {},
            "checks": [],
            "errors": [],
            "warnings": [],
            "infos": [],
        }

    def check(self) -> Dict[str, Any]:
        config = self.load_config()
        canonical = flatten_config(config)
        actual = self.build_actual(canonical)
        self.report["canonical"] = canonical
        self.report["actual"] = actual

        for key in (
            "goal_x",
            "goal_y",
            "goal_yaw",
            "amplitude_ratio",
            "max_amplitude",
            "smooth_iterations",
            "v_max_mps",
            "omega_max_radps",
            "a_max_mps2",
            "alpha_max_radps2",
        ):
            self.expect_numeric(key, actual.get(key), canonical.get(key))

        for key in ("path_template", "path_start_heading"):
            self.expect_string(key, actual.get(key), canonical.get(key))

        if self.args.path_file:
            self.check_path_endpoint(actual)

        ok = not self.report["errors"]
        self.report["ok"] = ok
        self.report["status"] = "PASS" if ok else "FAIL"
        return self.report

    def load_config(self) -> Dict[str, Any]:
        config_path = self.args.config
        if config_path is None:
            config_path = self.repo_root / DEFAULT_CONFIG
        else:
            config_path = config_path.resolve()
        if not config_path.is_file():
            self.error("CANONICAL_CONFIG_MISSING", f"Canonical endpoint config not found: {config_path}", config_path)
            return {}
        try:
            data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        except Exception as exc:  # noqa: BLE001 - report parse failures clearly
            self.error("CANONICAL_CONFIG_PARSE_FAILED", f"Failed to parse {config_path}: {exc}", config_path)
            return {}
        if not isinstance(data, dict):
            self.error("CANONICAL_CONFIG_INVALID", f"Canonical endpoint config must be a mapping: {config_path}", config_path)
            return {}
        self.info("CANONICAL_CONFIG_LOADED", f"Loaded canonical endpoint config: {config_path}", config_path)
        return data

    def build_actual(self, canonical: Dict[str, Any]) -> Dict[str, Any]:
        values: Dict[str, Any] = {}
        for key, attr in (
            ("goal_x", "goal_x"),
            ("goal_y", "goal_y"),
            ("goal_yaw", "goal_yaw"),
            ("path_template", "path_template"),
            ("path_start_heading", "path_start_heading"),
            ("amplitude_ratio", "amplitude_ratio"),
            ("max_amplitude", "max_amplitude"),
            ("smooth_iterations", "smooth_iterations"),
            ("v_max_mps", "v_max"),
            ("omega_max_radps", "omega_max"),
            ("a_max_mps2", "a_max"),
            ("alpha_max_radps2", "alpha_max"),
        ):
            cli_value = getattr(self.args, attr)
            values[key] = canonical.get(key) if cli_value is None else cli_value
        return values

    def expect_numeric(self, key: str, actual: Any, expected: Any) -> None:
        if actual is None or expected is None:
            self.error("ENDPOINT_FIELD_MISSING", f"Missing numeric field {key}: actual={actual!r} expected={expected!r}")
            return
        try:
            actual_f = float(actual)
            expected_f = float(expected)
        except (TypeError, ValueError):
            self.error("ENDPOINT_FIELD_NOT_NUMERIC", f"Non-numeric field {key}: actual={actual!r} expected={expected!r}")
            return
        if abs(actual_f - expected_f) > FLOAT_TOL:
            self.error("ENDPOINT_MISMATCH", f"{key} mismatch: actual={actual_f} expected={expected_f}")
        else:
            self.pass_check(key, f"{key} matches canonical value {expected_f}")

    def expect_string(self, key: str, actual: Any, expected: Any) -> None:
        if actual is None or expected is None:
            self.error("ENDPOINT_FIELD_MISSING", f"Missing string field {key}: actual={actual!r} expected={expected!r}")
            return
        if str(actual) != str(expected):
            self.error("ENDPOINT_MISMATCH", f"{key} mismatch: actual={actual!r} expected={expected!r}")
        else:
            self.pass_check(key, f"{key} matches canonical value {expected!r}")

    def check_path_endpoint(self, actual: Dict[str, Any]) -> None:
        path_file = self.args.path_file.resolve()
        if not path_file.is_file():
            self.error("PATH_FILE_MISSING", f"Path file not found: {path_file}", path_file)
            return
        try:
            data = json.loads(path_file.read_text(encoding="utf-8"))
            poses = extract_poses(data)
        except Exception as exc:  # noqa: BLE001 - report parse failures clearly
            self.error("PATH_FILE_PARSE_FAILED", f"Failed to parse path JSON {path_file}: {exc}", path_file)
            return
        if len(poses) < 2:
            self.error("PATH_TOO_SHORT", f"Path must contain at least two poses: {path_file}", path_file)
            return
        end = poses[-1]
        end_x = float(end.get("x", 0.0))
        end_y = float(end.get("y", 0.0))
        goal_x = float(actual["goal_x"])
        goal_y = float(actual["goal_y"])
        pos_err = math.hypot(end_x - goal_x, end_y - goal_y)
        self.report["path_endpoint"] = {
            "path_file": str(path_file),
            "end_x": end_x,
            "end_y": end_y,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "position_error_m": pos_err,
            "position_tolerance_m": self.args.position_tol,
        }
        if pos_err > self.args.position_tol:
            self.error(
                "PATH_ENDPOINT_MISMATCH",
                f"Path endpoint position error {pos_err:.4f}m exceeds tolerance {self.args.position_tol:.4f}m",
                path_file,
            )
        else:
            self.pass_check("path_endpoint_position", f"Path endpoint position matches goal within {pos_err:.4f}m", path_file)

        if self.args.check_path_yaw:
            end_yaw = pose_yaw(end, poses[-2])
            goal_yaw = float(actual["goal_yaw"])
            yaw_err = abs(wrap_to_pi(end_yaw - goal_yaw))
            self.report["path_endpoint"].update(
                {
                    "end_yaw": end_yaw,
                    "goal_yaw": goal_yaw,
                    "yaw_error_rad": yaw_err,
                    "yaw_tolerance_rad": self.args.yaw_tol,
                }
            )
            if yaw_err > self.args.yaw_tol:
                self.error(
                    "PATH_ENDPOINT_YAW_MISMATCH",
                    f"Path endpoint yaw error {yaw_err:.4f}rad exceeds tolerance {self.args.yaw_tol:.4f}rad",
                    path_file,
                )
            else:
                self.pass_check("path_endpoint_yaw", f"Path endpoint yaw matches goal within {yaw_err:.4f}rad", path_file)
        else:
            self.info(
                "PATH_YAW_NOT_ENFORCED",
                "Template generator uses path tangent for pose yaw; declared GOAL_YAW is checked as a scenario field only.",
            )

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


def flatten_config(data: Dict[str, Any]) -> Dict[str, Any]:
    goal = data.get("goal", {}) or {}
    path = data.get("path_template", {}) or {}
    limits = data.get("common_limits", {}) or {}
    return {
        "goal_x": goal.get("x"),
        "goal_y": goal.get("y"),
        "goal_yaw": goal.get("yaw"),
        "path_template": path.get("name"),
        "path_start_heading": path.get("start_heading"),
        "amplitude_ratio": path.get("amplitude_ratio"),
        "max_amplitude": path.get("max_amplitude"),
        "smooth_iterations": path.get("smooth_iterations"),
        "v_max_mps": limits.get("v_max_mps"),
        "omega_max_radps": limits.get("omega_max_radps"),
        "a_max_mps2": limits.get("a_max_mps2"),
        "alpha_max_radps2": limits.get("alpha_max_radps2"),
    }


def extract_poses(data: Any) -> List[Dict[str, Any]]:
    if isinstance(data, list):
        poses = data
    elif isinstance(data, dict):
        poses = data.get("poses", [])
    else:
        raise ValueError("path JSON must be a mapping with poses or a pose list")
    if not isinstance(poses, list):
        raise ValueError("path poses must be a list")
    return [pose for pose in poses if isinstance(pose, dict)]


def pose_yaw(pose: Dict[str, Any], previous_pose: Dict[str, Any]) -> float:
    if "yaw" in pose:
        return float(pose["yaw"])
    if "qz" in pose and "qw" in pose:
        qz = float(pose.get("qz", 0.0))
        qw = float(pose.get("qw", 1.0))
        return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
    return math.atan2(float(pose["y"]) - float(previous_pose["y"]), float(pose["x"]) - float(previous_pose["x"]))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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
    parser = argparse.ArgumentParser(description="Check profile-baseline endpoint/template against the canonical P2 fixed-path setup.")
    parser.add_argument("--config", type=Path, default=None, help="Canonical endpoint YAML; defaults to benchmark/canonical_fixed_path_p2.yaml")
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--path-file", type=Path, default=None, help="Optional generated/replayed path JSON for endpoint position validation")
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)
    parser.add_argument("--goal-yaw", type=float, default=None)
    parser.add_argument("--path-template", default=None)
    parser.add_argument("--path-start-heading", default=None)
    parser.add_argument("--amplitude-ratio", type=float, default=None)
    parser.add_argument("--max-amplitude", type=float, default=None)
    parser.add_argument("--smooth-iterations", type=int, default=None)
    parser.add_argument("--v-max", type=float, default=None)
    parser.add_argument("--omega-max", type=float, default=None)
    parser.add_argument("--a-max", type=float, default=None)
    parser.add_argument("--alpha-max", type=float, default=None)
    parser.add_argument("--position-tol", type=float, default=0.05)
    parser.add_argument("--yaw-tol", type=float, default=0.05)
    parser.add_argument("--check-path-yaw", action="store_true", help="Also compare final path tangent/quaternion yaw to GOAL_YAW")
    parser.add_argument("--format", choices=("json", "yaml"), default="json")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    checker = EndpointCheck(args)
    report = checker.check()
    emit_report(report, args.format)
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
