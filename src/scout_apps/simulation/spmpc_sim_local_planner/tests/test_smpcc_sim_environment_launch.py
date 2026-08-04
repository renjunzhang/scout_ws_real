#!/usr/bin/env python3
"""Contracts for the fresh, environment-only simulation entry point.

These checks intentionally resolve the launch graph without starting Gazebo.
They make a transitive legacy launch or a re-exposed real-controller package a
test failure before an H0/future matrix runner can acquire a master.
"""

from __future__ import annotations

import os
import shlex
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[3]
SIM_ROOT = Path("/data/a/scout_sim_replacement")
CLASSIC_WS = SIM_ROOT / "classic_ws"
SIM_BUILD_WORKSPACE = SIM_ROOT / "r8_controller_ws"
SIM_DEVEL = SIM_BUILD_WORKSPACE / "devel"
WORLD_FILE = CLASSIC_WS / "src/scout_mini_proxy_gazebo/worlds/open_walled_proxy.world"
MAP_FILE = SIM_ROOT / "maps/proxy_world_manual_saved_20260611_154348.pbstream"
ENVIRONMENT_LAUNCH = PACKAGE / "launch/smpcc_sim_environment.launch"
LOCALIZATION_LAUNCH = PACKAGE / "launch/smpcc_sim_localization.launch"
ENVIRONMENT_SCRIPT = PACKAGE / "scripts/launch_sim_environment.sh"


class SimulationEnvironmentLaunchTest(unittest.TestCase):
    maxDiff = None

    def _launch_nodes(self, path: Path) -> set[tuple[str, str, str]]:
        root = ET.parse(path).getroot()
        return {
            (node.attrib["pkg"], node.attrib["type"], node.attrib["name"])
            for node in root.findall("node")
        }

    def _curated_shell(self, command: str) -> subprocess.CompletedProcess[str]:
        """Run a read-only ROS package/launch inspection with the launcher path."""
        script = "\n".join(
            (
                "set -euo pipefail",
                "source /opt/ros/noetic/setup.bash",
                f"source {shlex.quote(str(SIM_DEVEL / 'setup.bash'))}",
                "source /home/a/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/setup.bash",
                "SMPCC_SIM_CARTO_PREFIX=/home/a/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated",
                "export ROS_PACKAGE_PATH="
                + shlex.quote(
                    ":".join(
                        (
                            str(CLASSIC_WS / "src/scout_mini_proxy_description"),
                            str(CLASSIC_WS / "src/scout_mini_proxy_bringup"),
                            str(PACKAGE),
                            "/home/a/scout_ws/src/scout_apps/sensors/nanoscan3_bringup",
                            "/home/a/scout_ws/src/scout_apps/sensors/nanoscan3_localization",
                        )
                    )
                )
                + ':"${SMPCC_SIM_CARTO_PREFIX}/share:/opt/ros/noetic/share"',
                "export CMAKE_PREFIX_PATH="
                + shlex.quote(str(CLASSIC_WS / "devel"))
                + f":{shlex.quote(str(SIM_DEVEL))}:${{SMPCC_SIM_CARTO_PREFIX}}:/opt/ros/noetic",
                command,
            )
        )
        return subprocess.run(
            ["/bin/bash", "-lc", script],
            cwd=WORKSPACE,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            env={**os.environ, "ROS_MASTER_URI": "http://127.0.0.1:9"},
        )

    def test_environment_launch_has_only_environment_nodes(self) -> None:
        self.assertEqual(
            {
                ("gazebo_ros", "gzserver", "gazebo"),
                ("gazebo_ros", "gzclient", "sim_environment_gzclient"),
                ("robot_state_publisher", "robot_state_publisher", "sim_environment_robot_state_publisher"),
                ("scout_mini_proxy_bringup", "cmd_vel_guard.py", "sim_environment_cmd_vel_guard"),
                ("gazebo_ros", "spawn_model", "sim_environment_spawn_proxy"),
            },
            self._launch_nodes(ENVIRONMENT_LAUNCH),
        )
        root = ET.parse(ENVIRONMENT_LAUNCH).getroot()
        includes = [item.attrib["file"] for item in root.findall("include")]
        self.assertEqual(
            ["$(find spmpc_sim_local_planner)/launch/smpcc_sim_localization.launch"],
            includes,
        )

    def test_localization_launch_has_no_transitive_navigation_launch(self) -> None:
        self.assertEqual(
            {
                ("nanoscan3_bringup", "odom_monotonic_relay.py", "sim_environment_odom_monotonic_relay"),
                ("cartographer_ros", "cartographer_node", "sim_environment_cartographer"),
                ("cartographer_ros", "cartographer_occupancy_grid_node", "sim_environment_cartographer_occupancy_grid"),
                ("rviz", "rviz", "sim_environment_localization_rviz"),
            },
            self._launch_nodes(LOCALIZATION_LAUNCH),
        )
        root = ET.parse(LOCALIZATION_LAUNCH).getroot()
        self.assertEqual([], root.findall("include"))
        text = LOCALIZATION_LAUNCH.read_text(encoding="utf-8")
        for forbidden in (
            'pkg="spmpc_local_planner"',
            "scout_mini_proxy_nav_adapter",
            "proxy_spmpc",
            "stable_world.launch",
        ):
            self.assertNotIn(forbidden, text)

    def test_script_uses_own_launch_with_isolation_and_empty_graph_gates(self) -> None:
        self.assertTrue(os.access(ENVIRONMENT_SCRIPT, os.X_OK))
        syntax = subprocess.run(
            ["bash", "-n", str(ENVIRONMENT_SCRIPT)],
            cwd=WORKSPACE,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(0, syntax.returncode, syntax.stdout)
        text = ENVIRONMENT_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("roslaunch spmpc_sim_local_planner smpcc_sim_environment.launch", text)
        self.assertIn("assert_forbidden_packages_hidden", text)
        self.assertIn("assert_fresh_master", text)
        self.assertIn("assert_no_forbidden_nodes", text)
        self.assertIn("Cartographer setup is required for isolated localization", text)
        self.assertIn('assert_no_topic_publishers "${CMD_VEL_TOPIC}" "controller"', text)
        self.assertIn('assert_no_topic_publishers "${REFERENCE_PATH_TOPIC}" "path"', text)
        self.assertIn("SIM_ACADOS_LIBRARY_PATH", text)
        self.assertIn('SIM_BUILD_WORKSPACE="${SMPCC_SIM_BUILD_WORKSPACE:-${SIM_ROOT}/r8_controller_ws}"', text)
        self.assertIn('source "${SIM_SETUP}"', text)
        self.assertNotIn("SCOUT_WS_SETUP", text)
        self.assertNotIn("/home/a/scout_ws/devel", text)
        self.assertIn('export LD_LIBRARY_PATH="${SIM_ACADOS_LIBRARY_PATH}:', text)
        self.assertIn("inherited real-controller ACADOS loader path is forbidden", text)
        self.assertIn('export ROS_DISTRO="${ROS_DISTRO:-}"', text)
        self.assertIn('log "received SIGTERM"; exit 143', text)
        self.assertIn('log "started ${name} pid=$!"', text)
        self.assertIn('TF_TIMEOUT="${TF_TIMEOUT:-75}"', text)
        self.assertIn("start_new_session=True", text)
        self.assertIn("turning a ready TF graph into a later timeout", text)
        self.assertNotIn('timeout "${timeout_sec}" bash -c', text)
        self.assertIn(
            "for package_name in spmpc_local_planner spmpc_experiments slosh_models scout_mini_proxy_nav_adapter; do",
            text,
        )
        for forbidden in (
            "roslaunch scout_mini_proxy_gazebo",
            "stable_world.launch",
            "roslaunch spmpc_local_planner",
            "roslaunch scout_mini_proxy_nav_adapter",
            "roslaunch proxy_spmpc",
            "killall",
            "pkill",
        ):
            self.assertNotIn(forbidden, text)

    def test_curated_package_search_hides_real_and_legacy_packages(self) -> None:
        completed = self._curated_shell(
            "\n".join(
                (
                    "rospack find spmpc_sim_local_planner",
                    "rospack find scout_mini_proxy_description",
                    "rospack find scout_mini_proxy_bringup",
                    "for forbidden in spmpc_local_planner spmpc_experiments slosh_models scout_mini_proxy_nav_adapter; do",
                    "  if rospack find \"${forbidden}\" >/dev/null 2>&1; then",
                    "    echo \"forbidden package visible: ${forbidden}\" >&2",
                    "    exit 41",
                    "  fi",
                    "done",
                )
            )
        )
        self.assertEqual(0, completed.returncode, completed.stdout)

    def test_launch_resolution_contains_no_controller_or_path_node(self) -> None:
        self.assertTrue(WORLD_FILE.is_file())
        self.assertTrue(MAP_FILE.is_file())
        completed = self._curated_shell(
            "roslaunch --nodes spmpc_sim_local_planner smpcc_sim_environment.launch "
            f"world_file:={shlex.quote(str(WORLD_FILE))} "
            f"map_file:={shlex.quote(str(MAP_FILE))} gui:=false localization_rviz:=false"
        )
        self.assertEqual(0, completed.returncode, completed.stdout)
        nodes = {line.strip() for line in completed.stdout.splitlines() if line.strip().startswith("/")}
        expected = {
            "/gazebo",
            "/sim_environment_robot_state_publisher",
            "/sim_environment_cmd_vel_guard",
            "/sim_environment_spawn_proxy",
            "/sim_environment_odom_monotonic_relay",
            "/sim_environment_cartographer",
            "/sim_environment_cartographer_occupancy_grid",
        }
        self.assertTrue(expected.issubset(nodes), completed.stdout)
        for forbidden in (
            "/spmpc_local_planner",
            "/sim_spmpc_local_planner",
            "/proxy_spmpc",
            "/scout_mini_proxy_nav_adapter",
            "/move_base",
        ):
            self.assertNotIn(forbidden, nodes, completed.stdout)


if __name__ == "__main__":
    unittest.main()
