#!/usr/bin/env python3
"""Static and rebuilt-binary boundary tests for the simulation controller fork."""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[3]
REAL_CONTROLLER = WORKSPACE / "src/scout_apps/control/spmpc_local_planner"
SIM_BUILD_WORKSPACE = Path("/data/a/scout_sim_replacement/r8_controller_ws")
SIM_LIBRARY = SIM_BUILD_WORKSPACE / "devel/lib/libspmpc_sim_local_planner.so"
SIM_NODE = SIM_BUILD_WORKSPACE / "devel/lib/spmpc_sim_local_planner/spmpc_sim_local_planner_node"


class SourceIsolationTest(unittest.TestCase):
    def test_real_controller_source_has_no_task_diff(self) -> None:
        completed = subprocess.run(
            ["git", "diff", "--exit-code", "--", str(REAL_CONTROLLER)],
            cwd=WORKSPACE,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(0, completed.returncode, completed.stdout)

    def test_simulation_build_prefix_is_external_and_not_a_symlink(self) -> None:
        self.assertTrue(SIM_BUILD_WORKSPACE.is_dir(), "build the isolated simulation workspace first")
        self.assertFalse(SIM_BUILD_WORKSPACE.is_symlink())
        self.assertFalse(str(SIM_BUILD_WORKSPACE.resolve()).startswith(str(WORKSPACE.resolve()) + "/"))
        self.assertTrue((SIM_BUILD_WORKSPACE / "devel/setup.bash").is_file())

    def test_build_and_package_do_not_depend_on_real_controller(self) -> None:
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        package_xml = (PACKAGE / "package.xml").read_text(encoding="utf-8")
        forbidden = (
            "find_package(spmpc_local_planner",
            "spmpc_local_planner_INCLUDE",
            "spmpc_local_planner_EXPORTED",
            "SPMPC_UPSTREAM_ROOT",
            "../../control/spmpc_local_planner",
            "slosh_models",
        )
        for token in forbidden:
            self.assertNotIn(token, cmake)
        self.assertNotIn(">spmpc_local_planner<", package_xml)
        self.assertNotIn(">slosh_models<", package_xml)

    def test_fork_tree_has_no_symlink_or_old_include_abi(self) -> None:
        for path in PACKAGE.rglob("*"):
            self.assertFalse(path.is_symlink(), f"symlink violates fork boundary: {path}")
        for root in (PACKAGE / "include", PACKAGE / "src"):
            for path in root.rglob("*"):
                if path.suffix not in {".h", ".cpp"}:
                    continue
                text = path.read_text(encoding="utf-8")
                self.assertNotIn("spmpc_local_planner/", text, str(path))
                self.assertNotIn("namespace spmpc_local_planner", text, str(path))
                self.assertNotIn("slosh_models/", text, str(path))
                self.assertNotIn("namespace slosh_models", text, str(path))

    def test_ros_graph_uses_sim_namespace(self) -> None:
        source = (PACKAGE / "src/ros/spmpc_sim_local_planner_ros.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn('NodeHandle sim_spmpc_nh(nh_, "sim_spmpc")', source)
        self.assertNotIn('NodeHandle spmpc_nh(nh_, "spmpc")', source)
        node_main = (PACKAGE / "src/spmpc_sim_local_planner_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("validateSimNodeAdmission", node_main)
        self.assertIn("SMPCC_SIM_CONTROLLER_GATE_HASH", node_main)
        self.assertLess(
            node_main.index("validateSimNodeAdmission"),
            node_main.index("ros::NodeHandle nh"),
        )
        gate = (PACKAGE / "scripts/smpcc_sim_controller_gate.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("_bind_gate_hash_to_node", gate)
        self.assertIn("write/readback mismatch", gate)
        self.assertIn("ENVIRONMENT_OWNER_PARAM", gate)
        self.assertIn("smpcc_sim_environment/owner_package", node_main)
        h0 = (PACKAGE / "scripts/smpcc_sim_h0_runtime_adapter.py").read_text(
            encoding="utf-8"
        )
        toolchain = (PACKAGE / "scripts/smpcc_sim_toolchain.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"/sim_spmpc/slosh_height"', h0)
        self.assertIn('ENV_LAUNCH = SIM_CONTROLLER_ROOT / "scripts/launch_sim_environment.sh"', h0)
        self.assertIn("require_source_separated_r8_execution(freeze, master_document)", toolchain)

    def test_runtime_launches_are_sim_owned(self) -> None:
        launch = (PACKAGE / "launch/smpcc_sim_mechanism_r8.launch").read_text(
            encoding="utf-8"
        )
        h0 = (PACKAGE / "scripts/smpcc_sim_h0_runtime_adapter.py").read_text(
            encoding="utf-8"
        )
        h_proxy = (PACKAGE / "launch/smpcc_sim_h_proxy_monitor.launch").read_text(
            encoding="utf-8"
        )
        environment = (PACKAGE / "scripts/launch_sim_environment.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('pkg="spmpc_sim_local_planner"', launch)
        self.assertIn('pkg="spmpc_sim_local_planner"', h_proxy)
        # The environment has an explicit deny-list, so a forbidden package
        # name is evidence of a fail-closed check rather than a dependency.
        # The active R8/H0 launch surface must carry no dependency on a real
        # controller or old proxy launch, and no environment command may
        # invoke one.
        for text in (launch, h0, h_proxy):
            self.assertNotIn("spmpc_experiments", text)
            self.assertNotIn("proxy_spmpc", text)
            self.assertNotIn('pkg="spmpc_local_planner"', text)
            self.assertNotIn('"roslaunch", "spmpc_local_planner"', text)
            self.assertNotIn('"roslaunch", "slosh_models"', text)
        for command in (
            "roslaunch spmpc_experiments",
            "roslaunch proxy_spmpc",
            "roslaunch scout_mini_proxy_nav_adapter",
            "roslaunch spmpc_local_planner",
        ):
            self.assertNotIn(command, environment)

    def test_rebuilt_binary_has_no_real_controller_runpath_or_link(self) -> None:
        self.assertTrue(SIM_LIBRARY.is_file(), "build spmpc_sim_local_planner before isolation test")
        self.assertTrue(SIM_NODE.is_file(), "build spmpc_sim_local_planner before isolation test")
        dynamic = subprocess.run(
            ["readelf", "-d", str(SIM_LIBRARY)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(0, dynamic.returncode, dynamic.stdout)
        self.assertIn("(RPATH)", dynamic.stdout)
        self.assertNotIn("(RUNPATH)", dynamic.stdout)
        self.assertNotIn("control/spmpc_local_planner", dynamic.stdout)
        self.assertNotIn("libspmpc_local_planner.so", dynamic.stdout)
        self.assertNotIn("libslosh_models.so", dynamic.stdout)
        node_dynamic = subprocess.run(
            ["readelf", "-d", str(SIM_NODE)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(0, node_dynamic.returncode, node_dynamic.stdout)
        self.assertIn("(RPATH)", node_dynamic.stdout)
        self.assertNotIn("(RUNPATH)", node_dynamic.stdout)
        self.assertIn("$ORIGIN/..", node_dynamic.stdout)
        linked = subprocess.run(
            ["ldd", str(SIM_NODE)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            # The developer shell may contain scout_ws/devel/lib, whose
            # same-named simulation library would take precedence over this
            # node's RUNPATH.  Audit the actual isolated runtime contract,
            # not an ambient workstation loader choice.
            env={
                "PATH": "/usr/bin:/bin",
                "LD_LIBRARY_PATH": (
                    str(SIM_BUILD_WORKSPACE / "devel/lib")
                    + ":/opt/ros/noetic/lib"
                ),
            },
        )
        self.assertEqual(0, linked.returncode, linked.stdout)
        resolved_line = next(
            (
                line
                for line in linked.stdout.splitlines()
                if "libspmpc_sim_local_planner.so =>" in line
            ),
            "",
        )
        self.assertTrue(resolved_line, linked.stdout)
        resolved_library = Path(
            resolved_line.split("=>", 1)[1].split("(", 1)[0].strip()
        ).resolve()
        self.assertEqual(SIM_LIBRARY.resolve(), resolved_library)
        self.assertNotIn("control/spmpc_local_planner", linked.stdout)
        self.assertNotIn("libspmpc_local_planner.so", linked.stdout)
        self.assertNotIn("libslosh_models.so", linked.stdout)
        self.assertNotIn("not found", linked.stdout)


if __name__ == "__main__":
    unittest.main()
