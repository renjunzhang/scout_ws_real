#!/usr/bin/env python3
import json
import tempfile
import unittest
from pathlib import Path
import recording_contract as contract

class RecordingContractTest(unittest.TestCase):
    def write(self, root, name, text):
        path = root / name
        path.write_text(text, encoding="utf-8")
        return str(path)

    def test_later_overlay_wins_and_one_deadline_is_enough(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            common = self.write(root, "common.yaml", "planning: {task_deadline_sec: 0}\nvalue: {source: common}\n")
            profile = self.write(root, "profile.yaml", "planning: {experiment_profile_id: p}\nvalue: {source: profile}\n")
            task = self.write(root, "task.yaml", "value: {source: task}\n")
            task_overlay = self.write(root, "task_overlay.yaml", "value: {source: task_overlay}\n")
            planner_overlay = self.write(root, "planner_overlay.yaml", "value: {source: planner}\nplanning: {task_deadline_sec: 4}\n")
            region = self.write(root, "region.yaml", "planning:\n  region:\n    enabled: true\n    id: r\n    frame_id: map\n    cells:\n      - id: c\n        s_begin: 0\n        s_end: 1\n        vertices: [[0, 0], [1, 0], [1, 1]]\n")
            args = type("Args", (), dict(output_dir=str(root / "out"), name="x", profile="p", planner_variant="",
                launch_args="", repo=str(root), task_config=common, profile_config=profile,
                task_config_extra=task, task_overlay=task_overlay, region=region,
                planner_overlay=planner_overlay, plan_file="", publish_cmd_vel="true"))
            contract.prepare(args)
            manifest = json.loads((root / "out/x_manifest.json").read_text())
            self.assertEqual(manifest["effective_config"]["value"]["source"], "planner")
            self.assertEqual(manifest["effective_config"]["planning"]["task_deadline_sec"], 4)

    def test_live_profile_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            manifest = {"effective_config": {"planning": {"experiment_profile_id": "expected"}}}
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest))
            live = root / "live.yaml"
            live.write_text("planning: {experiment_profile_id: actual}\n")
            with self.assertRaises(SystemExit):
                contract.verify_live(type("Args", (), {"manifest": str(manifest_path), "live_file": str(live)}))

    def test_live_match_succeeds(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config = {"planning": {"experiment_profile_id": "p", "reference": {"mode": "cruise", "plan_file": ""}, "region": {"enabled": False, "id": "", "frame_id": "map", "footprint_radius": .45, "margin": .02}, "geometry": {"enabled": False, "curvature_weight": 0., "curvature_rate_weight": 0., "goal_weight": 0., "goal_position_scale": .3, "goal_yaw_scale": 1.}, "task_deadline_sec": 3.}, "terminal": {"require_goal_yaw": True, "goal_yaw_tolerance": .1}, "ablation": {"jerk_limit_enable": True, "jerk_max": 1.}, "acados": {"rti_iterations": 5}, "planner_variant": "B0", "publish_cmd_vel": True}
            manifest = root / "manifest.json"; manifest.write_text(json.dumps({"effective_config": config}))
            live = root / "live.yaml"; live.write_text(json.dumps(config))
            contract.verify_live(type("Args", (), {"manifest": str(manifest), "live_file": str(live)}))

    def test_duplicate_yaml_key_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "duplicate.yaml"
            path.write_text("planning:\n  task_deadline_sec: 1\nplanning:\n  task_deadline_sec: 2\n")
            with self.assertRaises(ValueError):
                contract.load(str(path))

if __name__ == "__main__":
    unittest.main()
