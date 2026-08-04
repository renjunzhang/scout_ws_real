#!/usr/bin/env python3
"""Unit/static contracts for the simulation-owned H0 path publisher."""

from __future__ import annotations

import importlib.util
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/smpcc_sim_h0_fixed_path_publisher.py"
LAUNCH = ROOT / "launch/smpcc_sim_h0_fixed_path_publisher.launch"
H0_ATTACH = ROOT / "scripts/launch_h0_sim_controller.sh"

SPEC = importlib.util.spec_from_file_location("smpcc_sim_h0_fixed_path_publisher", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
publisher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(publisher)


class H0FixedPathPublisherTest(unittest.TestCase):
    class _AdmissionRospy:
        def __init__(self, parameters):
            self.parameters = dict(parameters)

        def get_param(self, name, default=None):
            return self.parameters.get(name, default)

    def test_s_curve_starts_at_odom_and_ends_at_old_open_field_goal(self):
        points = publisher.build_h0_path_points(
            start_x=-4.0,
            start_y=0.0,
            start_yaw=0.0,
            goal_x=5.0,
            goal_y=0.0,
            goal_yaw=0.0,
            template="s_curve",
            spacing_m=0.05,
            amplitude_ratio=0.18,
            min_amplitude_m=0.25,
            max_amplitude_m=1.20,
            side="left",
            smooth_iterations=3,
            start_heading="current",
        )
        self.assertGreater(len(points), 100)
        self.assertEqual((-4.0, 0.0), points[0][:2])
        self.assertEqual((5.0, 0.0), points[-1][:2])
        self.assertAlmostEqual(0.0, points[-1][2])
        self.assertGreater(max(point[1] for point in points), 0.10)
        self.assertLess(min(point[1] for point in points), -0.10)

    def test_h0_path_rejects_non_h0_geometry_contract(self):
        with self.assertRaises(publisher.H0PathError):
            publisher.build_h0_path_points(
                start_x=0.0,
                start_y=0.0,
                start_yaw=0.0,
                goal_x=5.0,
                goal_y=0.0,
                goal_yaw=0.0,
                template="s_curve",
                spacing_m=0.05,
                amplitude_ratio=0.18,
                min_amplitude_m=0.25,
                max_amplitude_m=1.20,
                side="left",
                smooth_iterations=3,
                start_heading="goal_chord",
            )

    def test_runtime_admission_is_default_deny_and_h0_only(self):
        for parameters in (
            {},
            {"~h0_development_ack": True, "~development_condition": "H1", "/use_sim_time": True},
            {"~h0_development_ack": True, "~development_condition": "H0", "/use_sim_time": False},
        ):
            with self.assertRaises(publisher.H0PathError):
                publisher.H0FixedPathPublisher(self._AdmissionRospy(parameters), object, object, object)

    def test_runtime_wrapper_publishes_latched_odom_frame_path(self):
        class Header:
            def __init__(self):
                self.frame_id = ""
                self.stamp = None

        class Position:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

        class Quaternion:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.w = 1.0

        class Pose:
            def __init__(self):
                self.position = Position()
                self.orientation = Quaternion()

        class PoseStamped:
            def __init__(self):
                self.header = Header()
                self.pose = Pose()

        class PathMessage:
            def __init__(self):
                self.header = Header()
                self.poses = []

        class OdomMessage:
            def __init__(self):
                self.header = Header()
                self.header.frame_id = "odom"
                self.pose = type("PoseWithCovariance", (), {"pose": Pose()})()
                self.pose.pose.position.x = -4.0

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        class FakeSubscriber:
            def unregister(self):
                return None

        class FakeRospy(self._AdmissionRospy):
            class Time:
                @staticmethod
                def now():
                    return "sim-time"

            def __init__(self, parameters):
                super().__init__(parameters)
                self.publishers = []
                self.odom = OdomMessage()

            def Publisher(self, *_args, **_kwargs):
                result = FakePublisher()
                self.publishers.append(result)
                return result

            def Subscriber(self, _topic, _type, callback, **_kwargs):
                callback(self.odom)
                return FakeSubscriber()

            def loginfo(self, *_args):
                return None

            def spin(self):
                return None

        rospy = FakeRospy(
            {
                "~h0_development_ack": True,
                "~development_condition": "H0",
                "/use_sim_time": True,
                "~path_template": "straight",
            }
        )
        runtime = publisher.H0FixedPathPublisher(rospy, OdomMessage, PathMessage, PoseStamped)
        runtime.publish_and_spin()
        path = rospy.publishers[0].messages[0]
        self.assertEqual("/scout/global_path_fixed", runtime.output_topic)
        self.assertEqual("odom", path.header.frame_id)
        self.assertEqual((-4.0, 0.0), (path.poses[0].pose.position.x, path.poses[0].pose.position.y))
        self.assertEqual((5.0, 0.0), (path.poses[-1].pose.position.x, path.poses[-1].pose.position.y))
        self.assertEqual("sim-time", path.header.stamp)

    def test_launch_is_sim_owned_default_deny_h0_only(self):
        root = ET.parse(LAUNCH).getroot()
        args = {element.attrib["name"]: element.attrib.get("default", "") for element in root.findall("arg")}
        self.assertEqual("false", args["h0_development_ack"])
        self.assertEqual("H0", args["development_condition"])
        self.assertEqual("5.0", args["goal_x"])
        self.assertEqual("0.0", args["goal_y"])
        self.assertEqual("0.0", args["goal_yaw"])
        node = root.find("node")
        self.assertIsNotNone(node)
        assert node is not None
        self.assertEqual("spmpc_sim_local_planner", node.attrib["pkg"])
        self.assertEqual("smpcc_sim_h0_fixed_path_publisher.py", node.attrib["type"])
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertNotIn("roslaunch scout_mini_proxy_nav_adapter", text)
        self.assertNotIn("pkg=\"spmpc_local_planner\"", text)

    def test_h0_attach_uses_sim_owned_path_launch_only(self):
        text = H0_ATTACH.read_text(encoding="utf-8")
        self.assertIn(
            "roslaunch spmpc_sim_local_planner smpcc_sim_h0_fixed_path_publisher.launch",
            text,
        )
        self.assertNotIn("roslaunch scout_mini_proxy_nav_adapter", text)
        self.assertNotIn("proxy_spmpc", text)

    def test_h0_attach_replaces_broad_workspace_overlay_with_sim_allowlist(self):
        text = H0_ATTACH.read_text(encoding="utf-8")
        self.assertIn(
            'export ROS_PACKAGE_PATH="${SIM_PACKAGE_SOURCE}:/opt/ros/noetic/share"',
            text,
        )
        self.assertIn("assert_forbidden_packages_hidden", text)
        self.assertIn("export ROS_IP=127.0.0.1", text)
        self.assertIn("export ROS_NAMESPACE=/", text)
        self.assertIn("unset ROS_HOSTNAME", text)
        self.assertIn('export ROS_DISTRO="${ROS_DISTRO:-}"', text)
        self.assertIn("SIM_ACADOS_LIBRARY_PATH", text)
        self.assertIn("inherited real-controller ACADOS loader path is forbidden", text)


if __name__ == "__main__":
    unittest.main()
