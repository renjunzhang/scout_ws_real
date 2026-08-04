#!/usr/bin/env python3
"""No-ROS unit tests for the development-only H0 runtime adapter."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "smpcc_sim_h0_runtime_adapter.py"


def load_adapter():
    spec = importlib.util.spec_from_file_location("smpcc_sim_h0_runtime_adapter_test_target", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot import H0 adapter")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class H0RuntimeAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.adapter = load_adapter()

    def test_static_h0_descriptor_is_development_only_and_preserves_p2_semantics(self):
        descriptor = self.adapter.h0_descriptor()
        self.assertFalse(descriptor["formal"])
        self.assertTrue(descriptor["development_only"])
        self.assertEqual("runtime_s_curve", descriptor["task"]["path_source_mode"])
        self.assertTrue(descriptor["task"]["not_H1_or_L1"])
        self.assertEqual({"x": 5.0, "y": 0.0, "yaw": 0.0}, descriptor["task"]["P2"]["goal"])
        self.assertEqual("/slosh/height", descriptor["measurement_channels"]["H_proxy"]["topic"])
        self.assertEqual(
            "adapter-owned spmpc_sim_local_planner/smpcc_sim_h_proxy_monitor.launch",
            descriptor["measurement_channels"]["H_proxy"]["producer"],
        )
        self.assertEqual("/sim_spmpc/slosh_height", descriptor["measurement_channels"]["H_modal"]["topic"])
        self.assertIsNone(descriptor["measurement_channels"]["H_plant"])
        self.assertFalse(descriptor["measurement_channels"]["physical_primary_eligible"])
        self.assertTrue(set(descriptor["recording"]["topics"]).isdisjoint(self.adapter.LIQUID_PLANT_RECORDED_TOPICS))
        self.assertNotIn("/imu/data", descriptor["recording"]["topics"])
        self.assertIn('ready_topics = ("/odom", "/scan_front", "/map", "/slosh/height")', MODULE_PATH.read_text(encoding="utf-8"))

    def test_explicit_development_liquid_plant_is_hash_bound_record_only_and_nonprimary(self):
        args = self.adapter.build_parser().parse_args(
            [
                "prepare",
                "--output-root",
                "/data/a/scout_sim_replacement/results/test_h0_plant",
                "--spec-output",
                "/data/a/scout_sim_replacement/results/test_h0_plant/spec.json",
                "--ros-master-uri",
                "127.0.0.1:11430",
                "--gazebo-master-uri",
                "127.0.0.1:11464",
                "--with-development-liquid-plant",
                "--development-liquid-plant-config",
                str(self.adapter.LIQUID_PLANT_CONFIG_ROOT / "C2_development_unvalidated.yaml"),
            ]
        )
        capability = self.adapter.resolve_development_liquid_plant(args)
        self.assertIsNotNone(capability)
        assert capability is not None
        self.assertFalse(capability["formal"])
        self.assertTrue(capability["development_only"])
        self.assertEqual("UNVALIDATED", capability["fidelity_validation_status"])
        self.assertFalse(capability["physical_primary_eligible"])
        self.assertTrue(capability["independent_plant"])
        self.assertEqual("C2_DEVELOPMENT_TEMPLATE_UNFROZEN", capability["condition_template_id"])
        self.assertEqual(
            "direct_python_with_PYTHONPATH_no_catkin_whitelist_change",
            capability["launch_mode"],
        )
        for key in (
            "plant_code_hash",
            "plant_parameter_hash",
            "plant_input_schema_hash",
            "plant_output_schema_hash",
        ):
            self.assertRegex(capability[key], r"^[0-9a-f]{64}$")

        descriptor = self.adapter.h0_descriptor(capability)
        h_plant = descriptor["measurement_channels"]["H_plant"]
        self.assertEqual("/sim_truth/liquid_height", h_plant["topic"])
        self.assertEqual("UNVALIDATED", h_plant["fidelity_validation_status"])
        self.assertFalse(h_plant["physical_primary_eligible"])
        self.assertFalse(descriptor["measurement_channels"]["physical_primary_eligible"])
        self.assertTrue(set(self.adapter.LIQUID_PLANT_RECORDED_TOPICS).issubset(descriptor["recording"]["topics"]))
        self.assertEqual(
            self.adapter.RECORDED_TOPICS + self.adapter.LIQUID_PLANT_RECORDED_TOPICS,
            self.adapter.recorded_topics(capability),
        )

    def test_development_liquid_plant_requires_explicit_flag_and_case_manifest_binding(self):
        config = self.adapter.LIQUID_PLANT_DEFAULT_CONFIG
        with self.assertRaises(self.adapter.AdapterError):
            self.adapter.resolve_development_liquid_plant(
                SimpleNamespace(
                    with_development_liquid_plant=False,
                    development_liquid_plant_config=config,
                )
            )
        selected = self.adapter.development_liquid_plant_capability(config)
        enabled = SimpleNamespace(
            with_development_liquid_plant=True,
            development_liquid_plant_config=config,
        )
        self.assertEqual(selected, self.adapter.runtime_development_liquid_plant(enabled, {"liquid_plant_capability": selected}))
        with self.assertRaises(self.adapter.AdapterError):
            self.adapter.runtime_development_liquid_plant(
                SimpleNamespace(
                    with_development_liquid_plant=False,
                    development_liquid_plant_config=None,
                ),
                {"liquid_plant_capability": selected},
            )
        bad = dict(selected, physical_primary_eligible=True)
        with self.assertRaises(self.adapter.AdapterError):
            self.adapter.runtime_development_liquid_plant(enabled, {"liquid_plant_capability": bad})

    def test_direct_plant_launch_uses_pythonpath_not_catkin_discovery(self):
        capability = self.adapter.development_liquid_plant_capability(
            self.adapter.LIQUID_PLANT_DEFAULT_CONFIG
        )
        with mock.patch.dict(os.environ, {}, clear=True):
            environment = self.adapter.runtime_environment(
                "http://127.0.0.1:11430",
                "http://127.0.0.1:11464",
                Path("/data/a/scout_sim_replacement/results/test_h0_plant/case"),
                liquid_plant=capability,
            )
        self.assertEqual("true", environment["SMPCC_H0_DEVELOPMENT_LIQUID_PLANT"])
        self.assertEqual(str(self.adapter.LIQUID_PLANT_PYTHON_ROOT.resolve()), environment["PYTHONPATH"])
        command = self.adapter.direct_development_liquid_plant_command()
        self.assertEqual(("/bin/bash", "-c"), tuple(command[:2]))
        self.assertIn('export ROS_DISTRO="${ROS_DISTRO:-}"', command[2])
        self.assertIn("python3", command[2])
        self.assertIn(str(self.adapter.LIQUID_PLANT_NODE), command[2])
        self.assertNotIn("rosrun scout_liquid_plant", command[2])
        self.assertNotIn("roslaunch scout_liquid_plant", command[2])

    def test_runtime_environment_pins_loopback_ros_identity(self):
        with mock.patch.dict(
            os.environ,
            {"ROS_HOSTNAME": "physical-robot-host", "ROS_NAMESPACE": "/physical"},
            clear=True,
        ):
            environment = self.adapter.runtime_environment(
                "http://127.0.0.1:11430",
                "http://127.0.0.1:11464",
                Path("/data/a/scout_sim_replacement/results/test_h0_loopback/case"),
            )
        self.assertEqual("127.0.0.1", environment["ROS_IP"])
        self.assertEqual("/", environment["ROS_NAMESPACE"])
        self.assertEqual("75", environment["TF_TIMEOUT"])
        self.assertNotIn("ROS_HOSTNAME", environment)
        self.assertEqual(self.adapter.SIM_ROS_PACKAGE_PATH, environment["SMPCC_SIM_ROS_PACKAGE_PATH"])
        attach = self.adapter.SPMPC_ATTACH
        syntax = self.adapter.subprocess.run(
            ["bash", "-n", str(attach)],
            text=True,
            stdout=self.adapter.subprocess.PIPE,
            stderr=self.adapter.subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(0, syntax.returncode, syntax.stdout)
        attach_text = attach.read_text(encoding="utf-8")
        self.assertIn(
            'export ROS_PACKAGE_PATH="${SIM_PACKAGE_SOURCE}:/opt/ros/noetic/share"',
            attach_text,
        )
        self.assertIn(
            "for package_name in spmpc_local_planner spmpc_experiments slosh_models scout_mini_proxy_nav_adapter; do",
            attach_text,
        )
        self.assertNotIn("roslaunch spmpc_local_planner", attach_text)
        self.assertNotIn("roslaunch slosh_models", attach_text)

    def test_direct_plant_parameter_loader_is_bounded_and_uses_fixed_private_namespace(self):
        capability = self.adapter.development_liquid_plant_capability(
            self.adapter.LIQUID_PLANT_DEFAULT_CONFIG
        )
        completed = self.adapter.subprocess.CompletedProcess(["rosparam"], 0, stdout="ok")
        with mock.patch.object(
            self.adapter, "run_checked", side_effect=[completed, completed]
        ) as run_checked:
            self.adapter.load_development_liquid_plant_parameters(capability, {})
        self.assertEqual(2, run_checked.call_count)
        first_command, _first_env, first_timeout, first_label = run_checked.call_args_list[0].args
        second_command, _second_env, second_timeout, second_label = run_checked.call_args_list[1].args
        self.assertEqual(15.0, first_timeout)
        self.assertEqual(15.0, second_timeout)
        self.assertEqual("development liquid-plant parameter load", first_label)
        self.assertEqual("development liquid-plant config-path binding", second_label)
        self.assertIn("rosparam load", first_command[2])
        self.assertIn("/liquid_plant_development", first_command[2])
        self.assertIn("rosparam set", second_command[2])
        self.assertNotIn("roslaunch scout_liquid_plant", first_command[2])

    def test_prepare_opt_in_records_unvalidated_h_plant_and_dynamic_recorder_topics(self):
        with tempfile.TemporaryDirectory() as temporary:
            sim_root = Path(temporary) / "sim"
            sim_root.mkdir()
            output_root = sim_root / "results"
            spec_path = sim_root / "h0_plant_spec.json"
            args = self.adapter.build_parser().parse_args(
                [
                    "prepare",
                    "--output-root",
                    str(output_root),
                    "--spec-output",
                    str(spec_path),
                    "--ros-master-uri",
                    "127.0.0.1:11430",
                    "--gazebo-master-uri",
                    "127.0.0.1:11464",
                    "--with-development-liquid-plant",
                ]
            )
            with mock.patch.object(self.adapter, "SIM_ROOT", sim_root):
                self.assertEqual(0, self.adapter.command_prepare(args))
            spec = json.loads(spec_path.read_text(encoding="utf-8"))
        self.assertTrue(spec["development_liquid_plant"]["enabled"])
        self.assertEqual(
            "UNVALIDATED_DEVELOPMENT_H_PLANT_NOT_PHYSICAL_PRIMARY",
            spec["liquid_plant_capability"]["status"],
        )
        self.assertFalse(spec["liquid_plant_capability"]["physical_primary_eligible"])
        self.assertEqual(
            "/sim_truth/liquid_height",
            spec["runtime_scenario"]["measurement_channels"]["H_plant"]["topic"],
        )
        self.assertTrue(set(self.adapter.LIQUID_PLANT_RECORDED_TOPICS).issubset(spec["recorder_topics"]))
        self.assertIn("--with-development-liquid-plant", spec["launch_command"])
        self.assertIn("--with-development-liquid-plant", spec["recorder_command"])
        firewall = spec["development_firewall"]
        self.assertIsNotNone(firewall)
        assert firewall is not None
        self.assertTrue(firewall["development_only"])
        self.assertFalse(firewall["formal"])
        self.assertFalse(firewall["physical_primary_eligible"])
        self.assertEqual(
            {"controller", "planner", "tracker", "cmd_gate"},
            set(firewall["node_roles"]),
        )
        self.assertEqual(
            set(self.adapter.H0_DEVELOPMENT_FIREWALL_CHECKPOINTS),
            set(spec["development_firewall_snapshot_commands"]),
        )
        for command in spec["development_firewall_snapshot_commands"].values():
            self.assertIn("firewall-snapshot", command)

    def test_development_firewall_snapshot_is_case_hash_bound_and_rejects_truth_subscriber(self):
        with tempfile.TemporaryDirectory() as temporary:
            sim_root = Path(temporary) / "sim"
            case_dir = sim_root / "results" / "case"
            case_dir.mkdir(parents=True)
            with mock.patch.object(self.adapter, "SIM_ROOT", sim_root):
                contract = self.adapter.development_firewall_contract(case_dir)
                pass_graph = {
                    "subscriptions": {
                        "/odom": ["/sim_spmpc_local_planner"],
                        "/cmd_vel": ["/cmd_vel_guard"],
                        # A recorder can consume the development output; it is
                        # not an information-path subscriber in the protected
                        # controller/planner/tracker/cmd-gate set.
                        "/sim_truth/liquid_height": ["/rosbag_record_123"],
                    }
                }
                snapshot = self.adapter.build_development_firewall_snapshot(
                    "ready", case_dir, "a" * 64, contract, graph=pass_graph
                )
                self.assertEqual("PASS", snapshot["status"])
                self.assertFalse(snapshot["physical_primary_eligible"])
                self.assertEqual(
                    [{"topic": "/sim_truth/liquid_height", "subscribers": ["/rosbag_record_123"]}],
                    snapshot["all_sim_truth_subscribers"],
                )
                self.adapter.validate_development_firewall_snapshot(
                    snapshot, "ready", case_dir, "a" * 64, contract
                )
                forbidden_graph = {
                    "subscriptions": {
                        "/odom": ["/sim_spmpc_local_planner"],
                        "/cmd_vel": ["/cmd_vel_guard"],
                        "/sim_truth/liquid_height": ["/sim_spmpc_local_planner"],
                    }
                }
                rejected = self.adapter.build_development_firewall_snapshot(
                    "pre_motion", case_dir, "a" * 64, contract, graph=forbidden_graph
                )
                self.assertEqual("FAIL", rejected["status"])
                self.assertEqual(
                    [{"node": "/sim_spmpc_local_planner", "role": "controller", "topic": "/sim_truth/liquid_height"},
                     {"node": "/sim_spmpc_local_planner", "role": "planner", "topic": "/sim_truth/liquid_height"},
                     {"node": "/sim_spmpc_local_planner", "role": "tracker", "topic": "/sim_truth/liquid_height"}],
                    rejected["forbidden_controller_subscribers"],
                )
                with self.assertRaises(self.adapter.AdapterError):
                    self.adapter.validate_development_firewall_snapshot(
                        rejected, "pre_motion", case_dir, "a" * 64, contract
                    )

    def test_development_firewall_contract_is_required_only_for_selected_plant(self):
        with tempfile.TemporaryDirectory() as temporary:
            sim_root = Path(temporary) / "sim"
            case_dir = sim_root / "case"
            case_dir.mkdir(parents=True)
            capability = self.adapter.development_liquid_plant_capability(
                self.adapter.LIQUID_PLANT_DEFAULT_CONFIG
            )
            with mock.patch.object(self.adapter, "SIM_ROOT", sim_root):
                with self.assertRaises(self.adapter.AdapterError):
                    self.adapter.runtime_development_firewall(
                        {"liquid_plant_capability": capability}, case_dir, capability
                    )
                self.assertIsNone(
                    self.adapter.runtime_development_firewall(
                        {"liquid_plant_capability": {"independent_plant": False}},
                        case_dir,
                        None,
                    )
                )

    def test_generated_row_and_seed_are_h0_only_and_stably_bound(self):
        row = self.adapter.h0_row()
        bundle = self.adapter.h0_seed_bundle()
        self.adapter.require_h0_row(row, "test row")
        self.assertEqual("Bsmooth", row["condition_id"])
        self.assertEqual("H0", row["path_id"])
        self.assertEqual(bundle["seed_bundle_id"], row["seed_bundle_id"])
        self.assertEqual(bundle["seed_bundle_hash"], row["seed_bundle_hash"])
        self.assertEqual(set(self.adapter.SEED_STREAMS), set(bundle["sub_seeds"]))
        self.assertEqual(set(self.adapter.SEED_STREAMS), set(bundle["traces"]))

    def test_active_bag_contract_and_goal_event_parser_are_strict(self):
        expected = Path("/data/a/scout_sim_replacement/results/h0/case/h0_runtime.bag.active")
        self.assertEqual(Path("/data/a/scout_sim_replacement/results/h0/case/h0_runtime"), self.adapter.active_bag_base(expected))
        self.assertEqual(Path("/data/a/scout_sim_replacement/results/h0/case/h0_runtime.bag"), self.adapter.closed_bag_path(expected))
        with self.assertRaises(self.adapter.AdapterError):
            self.adapter.active_bag_base(Path("/data/a/scout_sim_replacement/results/h0/case/h0_runtime.bag"))
        with tempfile.TemporaryDirectory() as temporary:
            event_path = Path(temporary) / self.adapter.GOAL_EVENT_FILENAME
            good = {
                "record_type": "SMPCC_SIM_DEV_GOAL_EVENT",
                "status": "PASS",
                "case_manifest_hash": "case-hash",
                "controller_status": "GOAL_REACHED",
                "first_arrival_utc": "2026-08-02T00:00:00+00:00",
                "ros_time_sec": 12.0,
            }
            event_path.write_text(json.dumps(good), encoding="utf-8")
            parsed = self.adapter.read_development_event(
                event_path,
                "SMPCC_SIM_DEV_GOAL_EVENT",
                "case-hash",
                "first_arrival_utc",
            )
            self.assertEqual("GOAL_REACHED", parsed["controller_status"])
            bad = dict(good, controller_status="NOT_GOAL_REACHED")
            event_path.write_text(json.dumps(bad), encoding="utf-8")
            # The generic reader deliberately checks the immutable identity;
            # command_goal_probe adds the exact GOAL_REACHED requirement.
            self.assertEqual("NOT_GOAL_REACHED", self.adapter.read_development_event(event_path, "SMPCC_SIM_DEV_GOAL_EVENT", "case-hash", "first_arrival_utc")["controller_status"])

    def test_prepare_defaults_leave_room_for_one_global_readiness_budget(self):
        args = self.adapter.build_parser().parse_args(
            [
                "prepare",
                "--output-root",
                "/data/a/scout_sim_replacement/results/test_h0",
                "--spec-output",
                "/data/a/scout_sim_replacement/results/test_h0/spec.json",
                "--ros-master-uri",
                "127.0.0.1:11430",
                "--gazebo-master-uri",
                "127.0.0.1:11464",
            ]
        )
        self.assertEqual(90.0, args.ready_timeout_sec)
        self.assertGreater(args.command_timeout_sec, args.ready_timeout_sec)
        self.assertGreater(args.recorder_ready_timeout_sec, 0.0)
        self.assertFalse(args.visualize)

    def test_visualize_is_explicit_adapter_owned_gazebo_and_tracking_rviz_opt_in(self):
        parser = self.adapter.build_parser()
        visual_args = parser.parse_args(
            [
                "prepare",
                "--output-root",
                "/data/a/scout_sim_replacement/results/test_h0_visual",
                "--spec-output",
                "/data/a/scout_sim_replacement/results/test_h0_visual/spec.json",
                "--ros-master-uri",
                "127.0.0.1:11430",
                "--gazebo-master-uri",
                "127.0.0.1:11464",
                "--visualize",
            ]
        )
        self.assertTrue(visual_args.visualize)
        with mock.patch.dict(os.environ, {}, clear=True):
            headless = self.adapter.runtime_environment(
                "http://127.0.0.1:11430",
                "http://127.0.0.1:11464",
                Path("/data/a/scout_sim_replacement/results/test_h0_visual/case"),
            )
            visual = self.adapter.runtime_environment(
                "http://127.0.0.1:11430",
                "http://127.0.0.1:11464",
                Path("/data/a/scout_sim_replacement/results/test_h0_visual/case"),
                visualize=True,
            )
        self.assertEqual("false", headless["GAZEBO_GUI"])
        self.assertEqual("true", headless["HEADLESS"])
        # The legacy roslaunch GUI remains disabled in visual mode.  A later
        # adapter Popen owns the one direct gzclient instead, so an old GUI
        # cannot be adopted or left behind by the environment wrapper.
        self.assertEqual("false", visual["GAZEBO_GUI"])
        self.assertEqual("false", visual["HEADLESS"])
        self.assertEqual("true", visual["USE_RVIZ"])
        self.assertEqual("true", visual["TRACKING_RVIZ"])
        self.assertEqual("false", headless["SMPCC_H0_OWNED_GZCLIENT"])
        self.assertEqual("true", visual["SMPCC_H0_OWNED_GZCLIENT"])
        self.assertEqual(
            "adapter_direct_gzclient_child",
            self.adapter.visualization_contract(True)["gazebo_gui_owner"],
        )
        self.assertFalse(self.adapter.visualization_contract(True)["roslaunch_gazebo_gui"])

    def test_direct_visual_gzclient_command_is_not_a_roslaunch_child(self):
        command = self.adapter.direct_gzclient_command()
        self.assertEqual(("/bin/bash", "-c"), tuple(command[:2]))
        self.assertIn("exec gzclient", command[2])
        self.assertNotIn("roslaunch", command[2])

    def test_visual_supervisor_tracks_direct_gzclient_and_cleans_only_owned_children(self):
        class FakeProcess:
            def __init__(self, pid, poll_values):
                self.pid = pid
                self._poll_values = list(poll_values)
                self.returncode = None

            def poll(self):
                if self._poll_values:
                    value = self._poll_values.pop(0)
                    if value is not None:
                        self.returncode = value
                    return value
                return self.returncode

        # The environment survives the immediate startup check then exits in
        # the first supervisor pass.  That causes the adapter to exercise its
        # owned-only cleanup tuple without launching any real ROS/Gazebo code.
        environment_child = FakeProcess(101, (None, None, 0))
        gazebo_gui_child = FakeProcess(102, (None,))
        monitor_child = FakeProcess(103, (None,))
        with tempfile.TemporaryDirectory() as temporary:
            case_dir = Path(temporary) / "case"
            environment = self.adapter.runtime_environment(
                "http://127.0.0.1:11430",
                "http://127.0.0.1:11464",
                case_dir,
                visualize=True,
            )
            with mock.patch.object(
                self.adapter.subprocess,
                "Popen",
                side_effect=(environment_child, gazebo_gui_child, monitor_child),
            ) as popen, mock.patch.object(
                self.adapter, "wait_for_ros_socket", return_value=True
            ) as wait_ros, mock.patch.object(
                self.adapter, "wait_for_gazebo_socket", return_value=True
            ) as wait_gazebo, mock.patch.object(
                self.adapter, "signal_owned_groups"
            ) as stop_owned:
                self.assertEqual(
                    1,
                    self.adapter.supervise_environment(
                        environment, case_dir, "http://127.0.0.1:11430"
                    ),
                )

        wait_ros.assert_called_once_with("http://127.0.0.1:11430", 60.0)
        wait_gazebo.assert_called_once_with("http://127.0.0.1:11464", 60.0)
        self.assertEqual(3, popen.call_count)
        self.assertEqual([str(self.adapter.ENV_LAUNCH)], popen.call_args_list[0].args[0])
        self.assertIn("exec gzclient", popen.call_args_list[1].args[0][2])
        self.assertEqual(True, popen.call_args_list[1].kwargs["start_new_session"])
        self.assertIn("roslaunch spmpc_sim_local_planner smpcc_sim_h_proxy_monitor.launch", popen.call_args_list[2].args[0][2])
        self.assertNotIn("slosh_models", popen.call_args_list[2].args[0][2])
        stop_owned.assert_called_once_with(
            (gazebo_gui_child, None, None, monitor_child, environment_child)
        )

    def test_opt_in_motion_reuses_prearmed_controller_and_starts_only_path_publisher(self):
        capability = self.adapter.development_liquid_plant_capability(
            self.adapter.LIQUID_PLANT_DEFAULT_CONFIG
        )
        args = SimpleNamespace()
        case_dir = Path("/data/a/scout_sim_replacement/results/test_h0_plant/case")
        environment = {"START_PATH_PUBLISHER": "false", "START_SPMPC": "true"}
        with mock.patch.object(
            self.adapter,
            "runtime_context",
            return_value=({}, case_dir, "http://127.0.0.1:11430", "http://127.0.0.1:11464"),
        ), mock.patch.object(
            self.adapter, "runtime_development_liquid_plant", return_value=capability
        ), mock.patch.object(
            self.adapter, "runtime_development_firewall", return_value={"contract_hash": "x"}
        ), mock.patch.object(
            self.adapter, "runtime_environment", return_value=environment
        ), mock.patch.object(self.adapter, "supervise_motion", return_value=0) as supervise:
            self.assertEqual(0, self.adapter.command_motion(args))
        self.assertEqual("true", environment["START_PATH_PUBLISHER"])
        self.assertEqual("false", environment["START_SPMPC"])
        supervise.assert_called_once_with(environment, case_dir)

    def test_visual_supervisor_rejects_legacy_roslaunch_gui_before_starting_any_child(self):
        with tempfile.TemporaryDirectory() as temporary:
            environment = {
                "SMPCC_H0_OWNED_GZCLIENT": "true",
                "GAZEBO_GUI": "true",
                "GAZEBO_MASTER_URI": "http://127.0.0.1:11464",
            }
            with mock.patch.object(self.adapter.subprocess, "Popen") as popen:
                with self.assertRaises(self.adapter.AdapterError):
                    self.adapter.supervise_environment(
                        environment,
                        Path(temporary) / "case",
                        "http://127.0.0.1:11430",
                    )
            popen.assert_not_called()

    def test_tf_probe_accepts_first_translation_and_terminates_only_its_probe_group(self):
        class FakeStdout:
            def readline(self):
                return "- Translation: [0.017, 0.002, 0.000]\\n"

        class FakeProcess:
            pid = 4242
            stdout = FakeStdout()

            def poll(self):
                return None

        process = FakeProcess()
        with mock.patch.object(self.adapter.subprocess, "Popen", return_value=process), mock.patch.object(
            self.adapter.select, "select", return_value=([process.stdout], [], [])
        ), mock.patch.object(self.adapter, "signal_owned_group") as stop_group:
            self.adapter.wait_map_tf({}, 1.0)
        stop_group.assert_called_once_with(process, self.adapter.signal.SIGTERM, 2.0)

    def test_zero_twist_uses_bounded_wall_clock_publisher_not_rostopic_handshake(self):
        completed = self.adapter.subprocess.CompletedProcess(["zero"], 0)
        with mock.patch.object(self.adapter, "run_checked", return_value=completed) as run_checked:
            self.assertTrue(self.adapter.publish_zero({}))
        command, _environment, timeout_sec, label = run_checked.call_args.args
        self.assertEqual(("/bin/bash", "-c"), tuple(command[:2]))
        self.assertIn("smpcc_sim_h0_zero_twist", command[2])
        self.assertIn("time.sleep", command[2])
        self.assertNotIn("rostopic pub", command[2])
        self.assertEqual(5.0, timeout_sec)
        self.assertEqual("zero Twist", label)

    def test_runtime_context_refuses_formal_manifest_and_binds_runner_hashes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            case_dir = root / "case"
            case_dir.mkdir()
            seed_path = case_dir / "seed_bundle.json"
            seed_path.write_text(json.dumps(self.adapter.h0_seed_bundle()), encoding="utf-8")
            manifest_path = case_dir / "case_launch_manifest.json"
            manifest = {
                "formal": False,
                "planned_row_id": self.adapter.h0_row()["planned_row_id"],
                "attempt_id": self.adapter.h0_row()["planned_row_id"] + "_r01",
                "seed_bundle_path": str(seed_path),
                "seed_bundle_hash": self.adapter.h0_seed_bundle()["seed_bundle_hash"],
                "ros_master_uri": "http://127.0.0.1:11430",
                "gazebo_master_uri": "http://127.0.0.1:11464",
            }
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            args = SimpleNamespace(
                case_manifest=manifest_path,
                case_dir=case_dir,
                ros_master_uri="127.0.0.1:11430",
                gazebo_master_uri="127.0.0.1:11464",
            )
            env = {
                "ROS_MASTER_URI": "http://127.0.0.1:11430",
                "GAZEBO_MASTER_URI": "http://127.0.0.1:11464",
                "SMPCC_CASE_LAUNCH_MANIFEST_SHA256": self.adapter.sha256_file(manifest_path),
                "SMPCC_SEED_BUNDLE_PATH": str(seed_path),
                "SMPCC_SEED_BUNDLE_SHA256": self.adapter.h0_seed_bundle()["seed_bundle_hash"],
            }
            with mock.patch.object(self.adapter, "SIM_ROOT", root), mock.patch.dict(os.environ, env, clear=True):
                returned, returned_case, ros, gazebo = self.adapter.runtime_context(args)
                self.assertFalse(returned["formal"])
                self.assertEqual(case_dir, returned_case)
                self.assertEqual("http://127.0.0.1:11430", ros)
                self.assertEqual("http://127.0.0.1:11464", gazebo)

                manifest["formal"] = True
                manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
                os.environ["SMPCC_CASE_LAUNCH_MANIFEST_SHA256"] = self.adapter.sha256_file(manifest_path)
                with self.assertRaises(self.adapter.AdapterError):
                    self.adapter.runtime_context(args)

    def make_effective_config_message(self, overrides=None, labels=None):
        values = dict(self.adapter.H0_DECLARED_EFFECTIVE_LAYOUT_VALUES)
        values.update(overrides or {})
        field_names = list(labels or self.adapter.H0_EFFECTIVE_CONFIG_LAYOUT_FIELDS)
        data = [values.get(name, 0.0) for name in field_names]
        return SimpleNamespace(
            _type="std_msgs/Float32MultiArray",
            layout=SimpleNamespace(
                dim=[SimpleNamespace(label=",".join(field_names), size=len(field_names), stride=len(field_names))],
                data_offset=0,
            ),
            data=data,
        )

    def fake_bag_factory(self, messages, topic_type="std_msgs/Float32MultiArray"):
        class FakeStamp:
            def __init__(self, value):
                self.value = value

            def to_sec(self):
                return self.value

        class FakeBag:
            def __init__(self, _path, mode):
                if mode != "r":
                    raise AssertionError("readback must open rosbag read-only")

            def __enter__(self):
                return self

            def __exit__(self, _exc_type, _exc, _traceback):
                return False

            def get_type_and_topic_info(self):
                return SimpleNamespace(
                    topics={
                        "/sim_spmpc/debug/effective_config": SimpleNamespace(
                            msg_type=topic_type,
                            message_count=len(messages),
                        )
                    }
                )

            def read_messages(self, topics):
                if topics != ["/sim_spmpc/debug/effective_config"]:
                    raise AssertionError("readback requested the wrong topic")
                for index, message in enumerate(messages):
                    yield "/sim_spmpc/debug/effective_config", message, FakeStamp(10.0 + index)

        return FakeBag

    def test_closed_bag_readback_emits_full_schema_and_exact_declared_observation(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag_path = Path(temporary) / "h0_runtime.bag"
            bag_path.write_bytes(b"offline-test-bag")
            declared = self.adapter.h0_declared_config()
            report = self.adapter.build_effective_config_readback_report(
                bag_path,
                declared,
                "a" * 64,
                self.fake_bag_factory([self.make_effective_config_message(), self.make_effective_config_message()]),
            )
        self.assertEqual("PASS", report["status"])
        self.assertFalse(report["formal"])
        self.assertEqual("rosbag_readback", report["source"])
        self.assertEqual("SMPCC_SIM_DEV_EFFECTIVE_CONFIG_READBACK", report["record_type"])
        self.assertEqual(declared, report["observed_effective_config"])
        self.assertEqual(report["declared_config_hash"], report["observed_effective_config_hash"])
        self.assertEqual({}, report["declared_vs_observed_diff"])
        self.assertEqual(list(self.adapter.H0_EFFECTIVE_CONFIG_LAYOUT_FIELDS), report["field_schema"]["labels"])
        self.assertEqual(2, report["message_count"])
        self.assertEqual("NOT_APPLICABLE_SLOSH_DISABLED", report["observed_effective_config"]["observer"]["status"])
        self.assertEqual(0.15, report["observed_effective_config"]["delay"]["configured_linear_sec"])
        self.assertEqual(0.22, report["observed_effective_config"]["delay"]["configured_angular_sec"])

    def test_readback_detects_declared_delay_value_and_extra_parameter_differences(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag_path = Path(temporary) / "h0_runtime.bag"
            bag_path.write_bytes(b"offline-test-bag")
            declared = json.loads(json.dumps(self.adapter.h0_declared_config()))
            declared["delay"]["configured_linear_sec"] = -1.0
            declared["fields"]["undeclared_extra_parameter"] = 17.0
            report = self.adapter.build_effective_config_readback_report(
                bag_path,
                declared,
                "b" * 64,
                self.fake_bag_factory([self.make_effective_config_message()]),
            )
        self.assertEqual("FAIL", report["status"])
        self.assertIn("message[0].delay.configured_linear_sec", report["declared_vs_observed_diff"])
        self.assertIn("message[0].fields.undeclared_extra_parameter", report["declared_vs_observed_diff"])
        self.assertGreater(report["mismatch_message_count"], 0)

    def test_readback_rejects_extra_or_reordered_layout_labels_and_active_bags(self):
        labels = list(self.adapter.H0_EFFECTIVE_CONFIG_LAYOUT_FIELDS) + ["unannounced_extra"]
        with self.assertRaises(self.adapter.AdapterError):
            self.adapter.parse_effective_config_message(self.make_effective_config_message(labels=labels))
        with tempfile.TemporaryDirectory() as temporary:
            case_dir = Path(temporary) / "case"
            case_dir.mkdir()
            active_bag = case_dir / "h0_runtime.bag.active"
            active_bag.write_bytes(b"still-recording")
            with self.assertRaises(self.adapter.AdapterError):
                self.adapter.require_closed_case_bag(active_bag, case_dir)

    def test_readback_case_binding_and_receipt_are_case_local_and_non_overwritable(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case_dir = root / "case"
            case_dir.mkdir()
            manifest_path = case_dir / "case_launch_manifest.json"
            manifest = {
                "formal": False,
                "planned_row_id": self.adapter.h0_row()["planned_row_id"],
                "attempt_id": self.adapter.h0_row()["planned_row_id"] + "_r01",
            }
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            args = SimpleNamespace(
                case_dir=case_dir,
                case_manifest=manifest_path,
                case_manifest_sha256=self.adapter.sha256_file(manifest_path),
            )
            with mock.patch.object(self.adapter, "SIM_ROOT", root):
                returned, returned_case, manifest_hash = self.adapter.readback_case_context(args)
                self.assertEqual(manifest, returned)
                self.assertEqual(case_dir, returned_case)
                self.assertEqual(self.adapter.sha256_file(manifest_path), manifest_hash)
                receipt = case_dir / self.adapter.EFFECTIVE_CONFIG_READBACK_FILENAME
                self.adapter.write_json_new(receipt, {"status": "FAIL", "formal": False})
                self.assertEqual(0o444, receipt.stat().st_mode & 0o777)
                with self.assertRaises(self.adapter.AdapterError):
                    self.adapter.write_json_new(receipt, {"status": "FAIL", "formal": False})

    def test_offline_readback_subcommand_binds_case_hash_and_writes_only_case_receipt(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case_dir = root / "case"
            assets_dir = root / "assets"
            case_dir.mkdir()
            assets_dir.mkdir()
            manifest_path = case_dir / "case_launch_manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "formal": False,
                        "planned_row_id": self.adapter.h0_row()["planned_row_id"],
                        "attempt_id": self.adapter.h0_row()["planned_row_id"] + "_r01",
                    }
                ),
                encoding="utf-8",
            )
            bag_path = case_dir / "h0_runtime.bag"
            bag_path.write_bytes(b"closed-offline-bag")
            declared_path = assets_dir / "h0_bsmooth_declared_readback_contract.json"
            declared_path.write_text(json.dumps(self.adapter.h0_declared_config()), encoding="utf-8")
            output_path = case_dir / self.adapter.EFFECTIVE_CONFIG_READBACK_FILENAME
            expected_manifest_hash = self.adapter.sha256_file(manifest_path)
            args = SimpleNamespace(
                case_dir=case_dir,
                case_manifest=manifest_path,
                case_manifest_sha256=expected_manifest_hash,
                bag=bag_path,
                declared_config=declared_path,
                output=output_path,
            )
            with mock.patch.object(self.adapter, "SIM_ROOT", root), mock.patch.object(
                self.adapter,
                "rosbag_factory",
                return_value=self.fake_bag_factory([self.make_effective_config_message()]),
            ):
                self.assertEqual(0, self.adapter.command_effective_config_readback(args))
            receipt = json.loads(output_path.read_text(encoding="utf-8"))
        self.assertEqual("PASS", receipt["status"])
        self.assertEqual(expected_manifest_hash, receipt["case_manifest_hash"])
        self.assertEqual("rosbag_readback", receipt["source"])
        self.assertEqual(receipt["declared_config_hash"], receipt["observed_effective_config_hash"])


if __name__ == "__main__":
    unittest.main()
