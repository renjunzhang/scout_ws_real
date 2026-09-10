#!/usr/bin/env python3
"""Offline source selection and raw-evidence contracts; no ROS nodes or motion."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

import genpy
import rosbag
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from spmpc_local_planner.msg import ControlCycleAudit, SloshObserverDebug, SloshObserverSelectionDebug

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "analysis"))
import validate_spmpc_comparison_recording as recording
from validate_i0_failclosed_fixed_abba_bag import selection_matches_source


def selection(source):
    msg = SloshObserverSelectionDebug()
    msg.configured = msg.valid = msg.imu_pipeline_ready = msg.imu_fresh = True
    msg.odom_snapshot_valid = msg.odom_fresh = True
    msg.nominal_source = msg.effective_source = {"odom": 1, "processed_imu": 2}[source]
    msg.nominal_source_name = msg.effective_source_name = source
    msg.fallback_policy, msg.fallback_policy_name = 2, "fail_closed"
    msg.status = "NOMINAL_" + source.upper()
    return msg


class SourceContractTest(unittest.TestCase):
    def test_only_requested_nominal_source_is_accepted(self):
        for source, other in (("odom", "processed_imu"), ("processed_imu", "odom")):
            self.assertTrue(selection_matches_source(selection(source), source))
            self.assertFalse(selection_matches_source(selection(other), source))

    def test_fallback_or_bad_shared_monitor_cannot_pass(self):
        for source in ("odom", "processed_imu"):
            for field, value in (("fallback_active", True), ("fallback_latched", True),
                                 ("imu_fresh", False), ("imu_pipeline_ready", False),
                                 ("effective_source_name", "wrong"), ("fallback_policy", 1)):
                msg = selection(source)
                setattr(msg, field, value)
                self.assertFalse(selection_matches_source(msg, source), (source, field))
        msg = selection("odom")
        msg.odom_fresh = False
        self.assertFalse(selection_matches_source(msg, "odom"))


def sample_messages(rgb=True):
    messages = []
    for t, status, velocity in ((10., "B_slosh_ACADOS_OK", .1), (11., "GOAL_REACHED", 0.)):
        audit = ControlCycleAudit()
        audit.cycle_start_stamp = audit.command_publish_stamp = genpy.Time.from_sec(t)
        audit.command_was_published = True
        audit.published_cmd_v, audit.solver_status = velocity, status
        messages.append((t, recording.AUDIT, audit))
    for i in range(321):
        t = 9.8 + i*.02
        for topic, source, status in ((recording.IMU, 2, "READY"), (recording.ODOM, 1, "ODOM_READY")):
            msg = SloshObserverDebug()
            msg.header.stamp = msg.state_stamp = msg.measurement_stamp = genpy.Time.from_sec(t)
            msg.header.frame_id = msg.excitation_axes_frame = "base_link"
            msg.excitation_reference_point = "imu_proxy" if source == 2 else "base_link"
            msg.configured = msg.valid = msg.bias_ready = msg.filter_ready = True
            msg.source, msg.input_status, msg.observer_update_count = source, status, i
            messages.append((t, topic, msg))
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = genpy.Time.from_sec(t), "world"
        pose.pose.orientation.w = 1
        messages.append((t, "/vrpn_client_node/Tracker0/pose", pose))
    for topic in (recording.IMU, recording.ODOM):
        startup = SloshObserverDebug()
        startup.header.stamp = genpy.Time.from_sec(9.)
        messages.append((9., topic, startup))
    if rgb:
        for i in range(193):
            t = 9.8 + i/30.
            img, info = Image(), CameraInfo()
            for msg in (img, info):
                msg.header.stamp = genpy.Time.from_sec(t)
                msg.header.frame_id = "camera_color_optical_frame"
                msg.width = msg.height = 2
            img.encoding, img.step, img.data = "rgb8", 6, bytes(12)
            info.K = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            messages.extend(((t, recording.IMAGE, img), (t, recording.INFO, info)))
    return sorted(messages, key=lambda row: row[0])


class RecordingContractTest(unittest.TestCase):
    def validate(self, messages, rgb=True):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"fixture.bag"
            with rosbag.Bag(str(path), "w") as bag:
                for t, topic, msg in messages:
                    bag.write(topic, msg, genpy.Time.from_sec(t))
            return recording.validate_bag(SimpleNamespace(
                bag=path, tracker="Tracker0", expect_rgb=rgb, tail_sec=5.,
                rgb_width=2, rgb_height=2, rgb_fps=30.))

    def test_common_evidence_covers_motion_and_five_second_tail(self):
        for rgb in (True, False):
            report = self.validate(sample_messages(rgb), rgb)
            self.assertEqual(report["status"], "PASS", report)
            self.assertEqual(report["coverage_end_sec"], 16.)

    def test_goal_window_uses_actual_command_publication(self):
        rows = sample_messages()
        goal = next(msg for _, topic, msg in rows
                    if topic == recording.AUDIT and msg.solver_status == "GOAL_REACHED")
        goal.command_publish_stamp = genpy.Time.from_sec(11.05)
        report = self.validate(rows)
        self.assertEqual(report["status"], "PASS", report)
        self.assertEqual(report["goal_sec"], 11.05)
        self.assertEqual(report["coverage_end_sec"], 16.05)

        # Coverage up to cycle_start + 5 s must not hide the missing end of
        # the tail measured from the actual goal command publication.
        report = self.validate([row for row in rows if row[0] <= 16.01])
        self.assertEqual(report["status"], "FAIL", report)
        self.assertTrue(any("boundary coverage" in item for item in report["failures"]))

    def test_unpublished_or_invalid_goal_cannot_close_recording_window(self):
        for published in (False, True):
            with self.subTest(published=published):
                rows = sample_messages()
                goal = next(msg for _, topic, msg in rows
                            if topic == recording.AUDIT and msg.solver_status == "GOAL_REACHED")
                goal.command_was_published = published
                goal.command_publish_stamp = genpy.Time()
                report = self.validate(rows)
                self.assertEqual(report["status"], "FAIL", report)

    def test_unpublished_goal_does_not_preempt_later_published_goal(self):
        rows = sample_messages()
        unpublished = ControlCycleAudit()
        unpublished.cycle_start_stamp = genpy.Time.from_sec(10.5)
        unpublished.solver_status = "GOAL_REACHED"
        rows.append((10.5, recording.AUDIT, unpublished))
        report = self.validate(sorted(rows, key=lambda row: row[0]))
        self.assertEqual(report["status"], "PASS", report)
        self.assertEqual(report["goal_sec"], 11.)
        self.assertEqual(report["coverage_end_sec"], 16.)

    def test_missing_rgb_nokov_or_either_monitor_fails(self):
        for missing in (recording.IMAGE, recording.INFO, recording.IMU, recording.ODOM,
                        "/vrpn_client_node/Tracker0/pose"):
            rows = [r for r in sample_messages() if r[1] != missing]
            self.assertEqual(self.validate(rows)["status"], "FAIL", missing)

    def test_stopping_at_goal_does_not_pass_tail_coverage(self):
        rows = [r for r in sample_messages() if r[0] <= 11.2]
        self.assertEqual(self.validate(rows)["status"], "FAIL")

    def test_bad_time_reset_geometry_payload_and_monitor_status_fail(self):
        changes = (
            (recording.IMAGE, "header.stamp", genpy.Time(10)),
            (recording.IMAGE, "width", 3),
            (recording.IMAGE, "data", bytes()),
            (recording.INFO, "header.frame_id", "wrong_frame"),
            (recording.IMU, "state_stamp", genpy.Time()),
            (recording.IMU, "reset_epoch", 1),
            (recording.ODOM, "valid", False),
            (recording.ODOM, "observer_update_count", 0),
        )
        for topic, field, value in changes:
            rows = sample_messages()
            msg = next(m for t, name, m in rows if name == topic and t > 12.)
            target = msg
            if "." in field:
                parent, field = field.split(".")
                target = getattr(target, parent)
            setattr(target, field, value)
            self.assertEqual(self.validate(rows)["status"], "FAIL", (topic, field))

    def test_image_free_mode_rejects_unexpected_images(self):
        self.assertEqual(self.validate(sample_messages(), False)["status"], "FAIL")


class RgbSwitchContractTest(unittest.TestCase):
    def recorded_rgb(self, switches, pilot=None):
        # Run the real recorder with ROS CLI stubs. Inspect its final topic
        # arguments and metadata, without a master, nodes, or a real bag.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            command_log = root / "rosbag_args.json"
            for name in ("rostopic", "rosparam", "rosnode"):
                stub = root / name
                stub.write_text("#!/bin/sh\nexit 0\n")
                stub.chmod(0o755)
            stub = root / "rosbag"
            stub.write_text(
                "#!/usr/bin/python3\nimport json, os, sys\n"
                "from pathlib import Path\n"
                "Path(os.environ['TEST_ROSBAG_ARGS']).write_text(json.dumps(sys.argv[1:]))\n")
            stub.chmod(0o755)
            environment = {
                "PATH": str(root) + os.pathsep + os.environ["PATH"],
                "OUT_DIR": str(root / "output"), "NAME": "rgb_switch_test",
                "RECORD_SEC": "0", "RECORD_TOPIC_INFO": "false",
                "LIQUID_EXPORT_AFTER_RECORD": "false",
                "TEST_ROSBAG_ARGS": str(command_log), **switches,
            }
            recorder = SCRIPTS / "record_spmpc_full_rgb_bag.sh"
            command = ["bash", str(recorder)]
            if pilot is not None:
                # Execute only the real runner's RGB policy before passing
                # the resolved switches to the recorder; never start a trial.
                runner = (SCRIPTS / "run_spmpc_real_fixed_path_trial.sh").read_text()
                truthy = runner[runner.index("truthy() {"):runner.index("\nfail() {")]
                begin = runner.index('if truthy "${PILOT_MODE}"; then\n  # Pilot image')
                end = runner.index('RECORD_ONLINE_LIQUID_DEBUG_IMAGES=', begin)
                code = (truthy + runner[begin:end]
                        + '\nexport RECORD_RGB RECORD_CAMERA\nexec bash "$1"\n')
                environment.update(PILOT_MODE="true" if pilot else "false",
                                   PILOT_RECORD_RGB=switches.get("PILOT_RECORD_RGB", "false"),
                                   PILOT_RECORD_ONLINE_LIQUID="false")
                command = ["bash", "-eu", "-c", code, "rgb-policy-test", str(recorder)]
            subprocess.run(command, env=environment, check=True, capture_output=True,
                           text=True, timeout=10)
            args = json.loads(command_log.read_text())
            info = (root / "output" / "rgb_switch_test_info.txt").read_text()
            metadata = dict(line.split("=", 1) for line in info.splitlines() if "=" in line)
            self.assertEqual(metadata["record_rgb"], metadata["record_camera"])
            selected = (root / "output" / "rgb_switch_test_selected_topics.txt").read_text().splitlines()
            self.assertEqual(recording.IMAGE in selected, recording.IMAGE in args)
            return recording.IMAGE in args

    def test_rgb_precedence_and_legacy_alias_reach_recorded_topics(self):
        cases = (
            ({}, False),
            ({"RECORD_CAMERA": "true"}, True),
            ({"RECORD_RGB": "true"}, True),
            ({"RECORD_RGB": "true", "RECORD_CAMERA": "false"}, True),
            ({"RECORD_RGB": "false", "RECORD_CAMERA": "true"}, False),
            ({"RECORD_RGB": "", "RECORD_CAMERA": "true"}, True),
        )
        for pilot in (None, False):
            for switches, expected in cases:
                with self.subTest(pilot=pilot, switches=switches):
                    self.assertEqual(self.recorded_rgb(switches, pilot), expected)

    def test_pilot_rgb_policy_overrides_both_stale_aliases(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                switches = {"PILOT_RECORD_RGB": "true" if enabled else "false",
                            "RECORD_RGB": "false" if enabled else "true",
                            "RECORD_CAMERA": "false" if enabled else "true"}
                self.assertEqual(self.recorded_rgb(switches, pilot=True), enabled)


class SourceEntryTest(unittest.TestCase):
    def test_entry_forwards_explicit_flags_and_ignores_stale_exports(self):
        # Substitute only the acquisition executable in a temporary copy.
        # This never invokes the real engine, ROS, recorder or movement.
        with tempfile.TemporaryDirectory() as directory:
            entry = Path(directory)/"entry.sh"
            entry.write_text((SCRIPTS/"run_spmpc_ablation_smoke.sh").read_text())
            engine = Path(directory)/"run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
            keys = ("ABLATION_OBSERVER_SOURCE", "ABLATION_SOURCE_COMPARISON", "ABLATION_RECORD_RGB", "VALIDATE_ONLY")
            engine.write_text("python3 - <<'PY'\nimport json,os\nprint(json.dumps({k:os.environ[k] for k in " + repr(keys) + "}))\nPY\n")
            for args, source, rgb, compare in (([], "processed_imu", "false", "false"),
                    (["--observer-source", "imu", "--record-rgb"], "processed_imu", "true", "true"),
                    (["--observer-source", "odom", "--record-rgb"], "odom", "true", "true")):
                result = subprocess.run(["bash", str(entry), *args], capture_output=True, text=True,
                    check=True, env={**os.environ, "ABLATION_RECORD_RGB": "true", "CURRENT_OBSERVER_SOURCE": "odom"})
                values = json.loads(result.stdout)
                self.assertEqual(values, dict(zip(keys, (source, compare, rgb, "true"))))
            for args in (("--observer-source", "typo"), ("--observer-source",),
                         ("--condition", "nostate", "--observer-source", "odom")):
                result = subprocess.run(["bash", str(entry), *args], capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)

    def test_common_launch_changes_only_liquid_source(self):
        text = (SCRIPTS/"run_spmpc_real_fixed_path_trial.sh").read_text()
        array = text[text.index("planner_cmd=("):text.index("planner_command_string=")]
        results = []
        for source in ("processed_imu", "odom"):
            script = "CURRENT_OBSERVER_SOURCE=$1\n" + array + '\nprintf "%s\\n" "${planner_cmd[@]}"'
            result = subprocess.run(["bash", "-c", script, "test", source],
                                    capture_output=True, text=True, check=True)
            results.append(result.stdout.splitlines())
        differences = [(a, b) for a, b in zip(*results) if a != b]
        self.assertEqual(differences, [("observer_source:=processed_imu", "observer_source:=odom")])


if __name__ == "__main__":
    unittest.main()
