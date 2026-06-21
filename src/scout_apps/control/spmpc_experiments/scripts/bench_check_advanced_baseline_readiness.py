#!/usr/bin/env python3
"""Dry-run readiness checks for advanced comparison baselines.

This script is intentionally read-only. It does not build packages, launch ROS,
generate solvers, install dependencies, kill processes, or modify benchmark
configs. Dependency/readiness failures are reported as metadata, not as algorithm
failures.
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
    raise SystemExit("PyYAML missing; install python3-yaml or pip3 install pyyaml") from exc


MPC_PLANNER_ROOT = "src/mpc_planner"
MPC_PLANNER_PLUGIN_XML = "src/mpc_planner/mpc_planner_rosnavigation/mpc_planner_rosnavigation_plugin.xml"
MPC_PLANNER_PLUGIN_CLASS = "local_planner/ROSNavigationPlanner"
MPC_PLANNER_ADAPTER_SOURCE = "src/mpc_planner/mpc_planner_rosnavigation/src/ros1_rosnavigation.cpp"
MPC_PLANNER_REQUIRED_GENERATED = {
    "solver_cmake": "src/mpc_planner/mpc_planner_solver/solver.cmake",
    "generated_parameters_cpp": "src/mpc_planner/mpc_planner_solver/src/mpc_planner_parameters.cpp",
}
MPC_PLANNER_EXPECTED_DEPS = {
    "ros_tools": "src/ros_tools",
    "guidance_planner": "src/guidance_planner",
    "decomp_util": "src/DecompUtil",
    "pedestrian_simulator": "src/pedestrian_simulator",
    "roadmap": "src/roadmap",
    "costmap_converter": "src/costmap_converter",
}
LT_DWA_VENDOR_ROOT = "third_party/LT_DWA"
LT_DWA_VENDOR_REQUIRED_FILES = {
    "readme": "ReadMe.md",
    "local_planner_package": "local_planner/package.xml",
    "navigation_package": "navigation/package.xml",
    "obstacle_msgs_package": "obstacle_msgs/package.xml",
    "local_map_generation_package": "local_map_generation/package.xml",
    "static_map_package": "static_map/package.xml",
    "crowd_simulator_package": "crowd_simulator/package.xml",
    "demo_node": "local_planner/src/local_planner_node.cpp",
}
LT_DWA_VENDOR_CONFLICTING_PACKAGE_NAMES = {
    "local_planner",
    "navigation",
}
LT_DWA_PACKAGE_NAME_HINTS = {
    "lt_dwa",
    "lt_dwa_planner",
    "lt_dwa_local_planner",
    "ltdwa",
    "ltdwa_planner",
    "lt_dwa_adapter",
}
LT_DWA_ADAPTER_ROOT = "src/scout_apps/control/lt_dwa_adapter"
LT_DWA_ADAPTER_REQUIRED_FILES = {
    "package_xml": "package.xml",
    "cmakelists": "CMakeLists.txt",
    "node_source": "src/lt_dwa_adapter_node.cpp",
    "ros_wrapper_source": "src/lt_dwa_adapter_ros.cpp",
    "planner_source": "src/lt_dwa_planner.cpp",
    "launch": "launch/lt_dwa_adapter.launch",
    "config": "config/lt_dwa_adapter_sim.yaml",
}
LT_DWA_BENCHMARK_REQUIRED_FILES = {
    "wrapper_launch": "src/scout_apps/control/spmpc_experiments/launch/sim/run_lt_dwa_fixed_path_sim.launch",
    "baseline_config": "src/scout_apps/control/spmpc_experiments/config/baselines/lt_dwa_adapter_standalone_sim.yaml",
}
LT_DWA_V2_ADAPTER_ROOT = "src/scout_apps/control/lt_dwa_v2_adapter"
LT_DWA_V2_ADAPTER_REQUIRED_FILES = {
    "package_xml": "package.xml",
    "cmakelists": "CMakeLists.txt",
    "node_source": "src/lt_dwa_v2_adapter_node.cpp",
    "ros_wrapper_source": "src/ros/lt_dwa_v2_adapter_ros.cpp",
    "planner_source": "src/search/lt_dwa_v2_planner.cpp",
    "launch": "launch/lt_dwa_v2_adapter.launch",
    "config": "config/lt_dwa_v2_adapter_sim.yaml",
}
LT_DWA_V2_BENCHMARK_REQUIRED_FILES = {
    "wrapper_launch": "src/scout_apps/control/spmpc_experiments/launch/sim/run_lt_dwa_v2_fixed_path_sim.launch",
    "baseline_config": "src/scout_apps/control/spmpc_experiments/config/baselines/lt_dwa_v2_adapter_standalone_sim.yaml",
}


class ReadinessCheck:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.repo_root = args.repo_root.resolve() if args.repo_root else infer_repo_root()
        self.report: Dict[str, Any] = {
            "tool": "bench_check_advanced_baseline_readiness.py",
            "schema_version": 1,
            "planner": args.planner,
            "repo_root": str(self.repo_root),
            "runtime_actions": {
                "starts_ros": False,
                "starts_gazebo": False,
                "kills_processes": False,
                "writes_files": False,
                "builds_packages": False,
                "generates_solvers": False,
                "installs_dependencies": False,
                "changes_cmd_vel_chain": False,
            },
            "checks": [],
            "missing": [],
            "errors": [],
            "warnings": [],
            "infos": [],
        }

    def check(self) -> Dict[str, Any]:
        if self.args.planner == "lt_dwa":
            self.check_lt_dwa()
        elif self.args.planner == "lt_dwa_v2":
            self.check_lt_dwa_v2()
        elif self.args.planner == "mpc_planner":
            self.check_mpc_planner()
        else:
            self.error("PLANNER_UNSUPPORTED", f"Unsupported planner {self.args.planner!r}")
        ok = not self.report["errors"]
        self.report["ok"] = ok
        self.report["status"] = "PASS" if ok else "FAIL"
        self.report["dependency_skipped"] = not ok
        return self.report

    def check_lt_dwa(self) -> None:
        vendor_root = self.repo_root / LT_DWA_VENDOR_ROOT
        if vendor_root.is_dir():
            self.pass_check("lt_dwa_vendor_root", "LT-DWA source is vendored outside catkin src", vendor_root)
            missing_vendor_files = []
            for name, rel in LT_DWA_VENDOR_REQUIRED_FILES.items():
                path = vendor_root / rel
                if path.is_file():
                    self.pass_check(f"lt_dwa_vendor:{name}", f"Found LT-DWA vendor file {rel}", path)
                else:
                    missing_vendor_files.append({"name": name, "path": str(path)})
            if missing_vendor_files:
                self.report["missing"].extend(missing_vendor_files)
                self.error(
                    "LT_DWA_VENDOR_INCOMPLETE",
                    "LT-DWA vendor tree is present but missing expected upstream files; refresh third_party/LT_DWA.",
                    vendor_root,
                )

            package_names = self.read_package_names(vendor_root)
            conflicts = sorted(LT_DWA_VENDOR_CONFLICTING_PACKAGE_NAMES & set(package_names))
            if conflicts:
                self.warn(
                    "LT_DWA_PACKAGE_NAME_CONFLICT",
                    "Vendored LT-DWA contains catkin package names that conflict with this workspace "
                    f"({conflicts}); keep it under third_party/LT_DWA and do not symlink it into src.",
                    vendor_root,
                )
            self.info(
                "LT_DWA_REFERENCE_SOURCE_ONLY",
                "LT-DWA upstream source is retained under third_party as a reference and is not catkin-built in the main workspace.",
                vendor_root,
            )
        else:
            self.warn("LT_DWA_VENDOR_ABSENT", "third_party/LT_DWA is absent; adapter can still be checked, but paper-reference source is missing", vendor_root)

        adapter_root = self.repo_root / LT_DWA_ADAPTER_ROOT
        if not adapter_root.is_dir():
            self.report["missing"].append({"name": "lt_dwa_adapter_root", "path": str(adapter_root)})
            self.error(
                "LT_DWA_ADAPTER_NOT_READY",
                "Scout-owned lt_dwa_adapter package is missing; keep LT-DWA dependency-skipped.",
                adapter_root,
            )
            return
        self.pass_check("lt_dwa_adapter_root", "Scout-owned LT-DWA adapter package is present", adapter_root)

        missing_adapter_files = []
        for name, rel in LT_DWA_ADAPTER_REQUIRED_FILES.items():
            path = adapter_root / rel
            if path.is_file():
                self.pass_check(f"lt_dwa_adapter:{name}", f"Found LT-DWA adapter file {rel}", path)
            else:
                missing_adapter_files.append({"name": name, "path": str(path)})
        for name, rel in LT_DWA_BENCHMARK_REQUIRED_FILES.items():
            path = self.repo_root / rel
            if path.is_file():
                self.pass_check(f"lt_dwa_benchmark:{name}", f"Found LT-DWA benchmark file {rel}", path)
            else:
                missing_adapter_files.append({"name": name, "path": str(path)})

        if missing_adapter_files:
            self.report["missing"].extend(missing_adapter_files)
            self.error(
                "LT_DWA_ADAPTER_NOT_READY",
                "LT-DWA adapter package is present but required node/config/launch benchmark files are missing.",
                adapter_root,
            )
            return

        packages = self.find_packages_by_name({"lt_dwa_adapter"})
        if "lt_dwa_adapter" not in packages:
            self.error("LT_DWA_ADAPTER_NOT_READY", "lt_dwa_adapter/package.xml does not declare package name lt_dwa_adapter", adapter_root)
            return
        self.pass_check("lt_dwa_adapter_package_name", "lt_dwa_adapter package name is unique and explicit", packages["lt_dwa_adapter"])
        self.warn(
            "LT_DWA_SMOKE_GATE_REQUIRED",
            "Static LT-DWA adapter assets are present; require isolated shadow/closed-loop smoke before formal main-table use.",
            adapter_root,
        )

    def check_lt_dwa_v2(self) -> None:
        vendor_root = self.repo_root / LT_DWA_VENDOR_ROOT
        if vendor_root.is_dir():
            self.info(
                "LT_DWA_REFERENCE_SOURCE_ONLY",
                "LT-DWA upstream source is retained under third_party as an algorithm reference for v2 and is not catkin-built.",
                vendor_root,
            )
        else:
            self.warn("LT_DWA_VENDOR_ABSENT", "third_party/LT_DWA is absent; v2 can still be checked as Scout-owned code, but paper-reference source is missing", vendor_root)

        adapter_root = self.repo_root / LT_DWA_V2_ADAPTER_ROOT
        if not adapter_root.is_dir():
            self.report["missing"].append({"name": "lt_dwa_v2_adapter_root", "path": str(adapter_root)})
            self.error(
                "LT_DWA_V2_SMOKE_GATE_REQUIRED",
                "Scout-owned lt_dwa_v2_adapter package is missing; keep LT-DWA-v2 dependency-skipped.",
                adapter_root,
            )
            return
        self.pass_check("lt_dwa_v2_adapter_root", "Scout-owned LT-DWA-v2 adapter package is present", adapter_root)

        missing_adapter_files = []
        for name, rel in LT_DWA_V2_ADAPTER_REQUIRED_FILES.items():
            path = adapter_root / rel
            if path.is_file():
                self.pass_check(f"lt_dwa_v2_adapter:{name}", f"Found LT-DWA-v2 adapter file {rel}", path)
            else:
                missing_adapter_files.append({"name": name, "path": str(path)})
        for name, rel in LT_DWA_V2_BENCHMARK_REQUIRED_FILES.items():
            path = self.repo_root / rel
            if path.is_file():
                self.pass_check(f"lt_dwa_v2_benchmark:{name}", f"Found LT-DWA-v2 benchmark file {rel}", path)
            else:
                missing_adapter_files.append({"name": name, "path": str(path)})

        if missing_adapter_files:
            self.report["missing"].extend(missing_adapter_files)
            self.error(
                "LT_DWA_V2_SMOKE_GATE_REQUIRED",
                "LT-DWA-v2 adapter package is present but required node/config/launch benchmark files are missing.",
                adapter_root,
            )
            return

        packages = self.find_packages_by_name({"lt_dwa_v2_adapter"})
        if "lt_dwa_v2_adapter" not in packages:
            self.error("LT_DWA_V2_SMOKE_GATE_REQUIRED", "lt_dwa_v2_adapter/package.xml does not declare package name lt_dwa_v2_adapter", adapter_root)
            return
        self.pass_check("lt_dwa_v2_adapter_package_name", "lt_dwa_v2_adapter package name is unique and explicit", packages["lt_dwa_v2_adapter"])
        self.warn(
            "LT_DWA_V2_SMOKE_GATE_REQUIRED",
            "Static LT-DWA-v2 benchmark assets are present; require strict fresh-sim smoke before formal main-table use.",
            adapter_root,
        )

    def check_mpc_planner(self) -> None:
        root = self.repo_root / MPC_PLANNER_ROOT
        if not root.is_dir():
            self.report["missing"].append({"name": "mpc_planner_root", "path": str(root)})
            self.error("MPC_PLANNER_NOT_READY", f"src/mpc_planner root missing: {root}", root)
            return
        self.pass_check("mpc_planner_root", "src/mpc_planner is present", root)

        plugin_xml = self.repo_root / MPC_PLANNER_PLUGIN_XML
        if not plugin_xml.is_file():
            self.report["missing"].append({"name": "rosnavigation_plugin_xml", "path": str(plugin_xml)})
        else:
            self.pass_check("mpc_planner_plugin_xml", "ROS navigation plugin XML is present", plugin_xml)
            text = plugin_xml.read_text(encoding="utf-8", errors="replace")
            if MPC_PLANNER_PLUGIN_CLASS in text:
                self.pass_check("mpc_planner_plugin_class", f"Found plugin class {MPC_PLANNER_PLUGIN_CLASS}", plugin_xml)
            else:
                self.report["missing"].append(
                    {
                        "name": "rosnavigation_plugin_class",
                        "expected": MPC_PLANNER_PLUGIN_CLASS,
                        "path": str(plugin_xml),
                    }
                )

        for name, rel in MPC_PLANNER_REQUIRED_GENERATED.items():
            path = self.repo_root / rel
            if path.is_file():
                self.pass_check("generated:" + name, f"Found generated artifact {name}", path)
            else:
                self.report["missing"].append({"name": name, "path": str(path)})

        for name, rel in MPC_PLANNER_EXPECTED_DEPS.items():
            path = self.repo_root / rel
            if path.exists():
                self.pass_check("dependency:" + name, f"Found expected dependency {name}", path)
            else:
                self.report["missing"].append({"name": name, "path": str(path)})

        adapter_source = self.repo_root / MPC_PLANNER_ADAPTER_SOURCE
        if adapter_source.is_file():
            self.pass_check("adapter_source", "ROS navigation adapter source is present", adapter_source)
            source = adapter_source.read_text(encoding="utf-8", errors="replace")
            side_effect_tokens = [token for token in ("/input/", "pedestrian", "reset") if token in source]
            if side_effect_tokens:
                self.warn(
                    "MPC_PLANNER_ADAPTER_HAS_DEMO_ASSUMPTIONS",
                    f"Adapter source contains demo/environment tokens {side_effect_tokens}; require separate smoke-gate cleanup before runnable benchmark use.",
                    adapter_source,
                )
        else:
            self.report["missing"].append({"name": "adapter_source", "path": str(adapter_source)})

        if self.report["missing"]:
            self.error(
                "MPC_PLANNER_NOT_READY",
                "src/mpc_planner is present but missing generated solver artifacts, dependencies, or adapter declarations; keep dependency-skipped.",
                root,
            )
        else:
            self.warn(
                "MPC_PLANNER_SMOKE_GATE_REQUIRED",
                "Static readiness files are present, but a separate isolated smoke gate is still required before suite integration.",
                root,
            )

    def read_package_names(self, root: Path) -> List[str]:
        names: List[str] = []
        for package_xml in root.rglob("package.xml"):
            try:
                text = package_xml.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            start = text.find("<name>")
            end = text.find("</name>", start + len("<name>")) if start >= 0 else -1
            if start >= 0 and end > start:
                names.append(text[start + len("<name>") : end].strip())
        return names

    def find_packages_by_name(self, names: set[str]) -> Dict[str, Path]:
        src_root = self.repo_root / "src"
        found: Dict[str, Path] = {}
        if not src_root.is_dir():
            return found
        for package_xml in src_root.rglob("package.xml"):
            try:
                text = package_xml.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for name in names:
                if f"<name>{name}</name>" in text:
                    found[name] = package_xml.parent
        return found

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
    parser = argparse.ArgumentParser(description="Dry-run readiness check for advanced benchmark baselines.")
    parser.add_argument("--planner", choices=("lt_dwa", "lt_dwa_v2", "mpc_planner"), required=True)
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--format", choices=("json", "yaml"), default="json")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    checker = ReadinessCheck(args)
    report = checker.check()
    emit_report(report, args.format)
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
