#!/usr/bin/env python3
"""Offline tests for the immutable H1/L1 frozen JSON Path replay adapter.

The ROS-facing code is tested with tiny in-memory message and publisher
stand-ins.  No test imports rospy, contacts a ROS master, or starts Gazebo.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "smpcc_sim_frozen_path_replay.py"


def load_adapter():
    module_name = "smpcc_sim_frozen_path_replay_test_target"
    spec = importlib.util.spec_from_file_location(module_name, MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot dynamically import frozen path replay adapter")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class _Header:
    def __init__(self):
        self.frame_id = ""
        self.stamp = None


class _Position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Orientation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


class _Pose:
    def __init__(self):
        self.position = _Position()
        self.orientation = _Orientation()


class _FakePoseStamped:
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()


class _FakePath:
    def __init__(self):
        self.header = _Header()
        self.poses = []


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FrozenPathReplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.adapter = load_adapter()

    @staticmethod
    def _write_json(path: Path, value) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True), encoding="utf-8")

    @staticmethod
    def _sha(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def _fixture(self, root: Path, *, path_id: str = "H1"):
        """Build a real-hash, development-only H1/L1 asset graph in /tmp."""
        assets = root / "assets"
        world = {
            "document_type": "SMPCC_SIM_WORLD_CLEARANCE_GEOMETRY",
            "status": "FROZEN",
            "bounds": [-5.0, 5.0, -5.0, 5.0],
            "obstacles": [],
            "robot_footprint_radius_m": 0.1,
            "container_footprint_radius_m": 0.1,
        }
        world_path = assets / "world.sdf"
        world_path.parent.mkdir(parents=True, exist_ok=True)
        world_path.write_text("<?xml version='1.0'?><sdf version='1.6'><world name='fixture'/></sdf>\n", encoding="utf-8")
        world["launched_world_hash"] = self._sha(world_path)
        geometry_path = assets / "world_clearance_geometry.json"
        self._write_json(geometry_path, world)
        zones = {"Z1": [0.0, 0.2], "Z2": [0.2, 0.4], "Z3": [0.4, 0.6], "Z4": [0.6, 0.8], "Z5": [0.8, 1.0]}
        document = {
            "path_id": path_id,
            "source_mode": "frozen_json_replay",
            "frame_id": "map",
            "ros_path_topic": "/smpcc/frozen_replay_path",
            "points": [
                {"x": -1.0, "y": 0.0, "yaw": 0.0},
                {"x": 0.0, "y": 0.5, "yaw": 0.5},
                {"x": 1.0, "y": 0.0, "yaw": 0.0},
            ],
            "zones": zones,
        }
        source_path = assets / f"{path_id}_source.json"
        sim_path = assets / f"{path_id}_sim.json"
        self._write_json(source_path, document)
        self._write_json(sim_path, document)
        transform = {"rotation_rad": 0.0, "tx": 0.0, "ty": 0.0, "yaw_offset_rad": 0.0}
        transform_path = assets / f"{path_id}_transform.json"
        self._write_json(transform_path, transform)
        validation = self.adapter.toolchain.validate_path_replay(source_path, sim_path, transform, world, 0.2)
        fit = {
            "report_type": "SMPCC_SIM_PATH_FIT_CLEARANCE",
            "status": "PASS",
            "path_id": path_id,
            "source_path_hash": validation["source_path_hash"],
            "sim_path_hash": validation["sim_path_hash"],
            "transform_hash": validation["transform_hash"],
            "world_hash": validation["world_hash"],
            "clearance_m": 0.2,
            "minimum_clearance_m": validation["minimum_clearance_m"],
            "clearance_method": validation["clearance_method"],
        }
        fit_path = assets / f"{path_id}_fit_clearance.json"
        self._write_json(fit_path, fit)
        entry = {
            "source_mode": "frozen_json_replay",
            "source_path": str(source_path.resolve()),
            "source_path_hash": self._sha(source_path),
            "sim_path": str(sim_path.resolve()),
            "sim_path_hash": self._sha(sim_path),
            "transform_path": str(transform_path.resolve()),
            "transform_hash": self._sha(transform_path),
            "fit_clearance_report_path": str(fit_path.resolve()),
            "fit_clearance_report_hash": self._sha(fit_path),
            "clearance_m": 0.2,
        }
        freeze = {
            "mode": "development_fixture_not_formal",
            "paths": {path_id: entry},
            "simulator_assets": {
                "world_file": str(world_path.resolve()),
                "world_hash": self._sha(world_path),
                "world_geometry_file": str(geometry_path.resolve()),
                "world_geometry_hash": self._sha(geometry_path),
            },
        }
        freeze_path = root / "frozen_path_registry.json"
        self._write_json(freeze_path, freeze)
        return {
            "freeze": freeze,
            "freeze_path": freeze_path,
            "freeze_hash": self._sha(freeze_path),
            "source_path": source_path,
            "sim_path": sim_path,
            "transform_path": transform_path,
            "fit_path": fit_path,
            "world_path": world_path,
            "geometry_path": geometry_path,
            "path_id": path_id,
        }

    def _rewrite_freeze(self, fixture) -> None:
        self._write_json(fixture["freeze_path"], fixture["freeze"])
        fixture["freeze_hash"] = self._sha(fixture["freeze_path"])

    def _preflight(self, fixture, *, require_formal: bool = False):
        return self.adapter.preflight_frozen_path_replay(
            fixture["freeze_path"].resolve(),
            fixture["freeze_hash"],
            fixture["path_id"],
            require_formal=require_formal,
        )

    def test_preflight_returns_immutable_h1_snapshot_and_nonexperiment_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            replay = self._preflight(fixture)
            self.assertEqual("H1", replay.path_id)
            self.assertEqual("map", replay.frame_id)
            self.assertEqual("/smpcc/frozen_replay_path", replay.ros_path_topic)
            self.assertEqual(3, len(replay.points))
            self.assertFalse(replay.formal_freeze_validated)
            self.assertEqual(self.adapter.ADAPTER_ID, replay.report()["adapter_id"])
            self.assertEqual("FROZEN_PATH_REPLAY_PREFLIGHT_ONLY_NOT_EXPERIMENT", replay.report()["evidence_class"])
            self.assertFalse(replay.report()["formal_execution_authorized"])
            self.assertEqual(fixture["world_path"].resolve(), Path(replay.world_path))
            self.assertEqual(fixture["geometry_path"].resolve(), Path(replay.world_geometry_path))
            self.assertNotEqual(replay.world_path, replay.world_geometry_path)
            with self.assertRaises((AttributeError, TypeError)):
                replay.points[0].x = 9.0

    def test_l1_is_allowed_and_ros_message_uses_snapshot_without_file_reread(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary), path_id="L1")
            replay = self._preflight(fixture)
            original_x = replay.points[0].x
            mutated = json.loads(fixture["sim_path"].read_text(encoding="utf-8"))
            mutated["points"][0]["x"] = 99.0
            self._write_json(fixture["sim_path"], mutated)
            message = self.adapter.build_ros_path_message(replay, _FakePath, _FakePoseStamped, "frozen-stamp")
            self.assertEqual("map", message.header.frame_id)
            self.assertEqual("frozen-stamp", message.header.stamp)
            self.assertEqual(3, len(message.poses))
            self.assertEqual(original_x, message.poses[0].pose.position.x)
            self.assertAlmostEqual(math.sin(0.5 / 2.0), message.poses[1].pose.orientation.z)
            self.assertAlmostEqual(math.cos(0.5 / 2.0), message.poses[1].pose.orientation.w)
            publisher = _FakePublisher()
            returned = self.adapter.publish_immutable_replay(replay, publisher, _FakePath, _FakePoseStamped, "frozen-stamp")
            self.assertIs(returned, publisher.messages[0])

    def test_rejects_h0_runtime_s_curve_and_w5_lineage_before_replay(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "only H1 or L1"):
                self.adapter.preflight_frozen_path_replay(fixture["freeze_path"].resolve(), fixture["freeze_hash"], "H0")

        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            fixture["freeze"]["paths"]["H0"] = {"source_mode": "frozen_json_replay"}
            self._rewrite_freeze(fixture)
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "H0"):
                self._preflight(fixture)

        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            fixture["freeze"]["paths"]["H1"]["source_mode"] = "runtime_s_curve"
            self._rewrite_freeze(fixture)
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "runtime_s_curve"):
                self._preflight(fixture)

        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            fixture["freeze"]["paths"]["H1"]["lineage"] = "W5_S10"
            self._rewrite_freeze(fixture)
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "W5/W5_S10"):
                self._preflight(fixture)

    def test_hash_binding_and_rigid_transform_validation_are_fail_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            mutated = json.loads(fixture["sim_path"].read_text(encoding="utf-8"))
            mutated["points"][1]["y"] = 2.0
            self._write_json(fixture["sim_path"], mutated)
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "hash mismatch"):
                self._preflight(fixture)

        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            mutated = json.loads(fixture["sim_path"].read_text(encoding="utf-8"))
            mutated["points"][1]["y"] = 2.0
            self._write_json(fixture["sim_path"], mutated)
            fixture["freeze"]["paths"]["H1"]["sim_path_hash"] = self._sha(fixture["sim_path"])
            self._rewrite_freeze(fixture)
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "rigid transform"):
                self._preflight(fixture)

    def test_stale_or_failed_fit_report_is_refused(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            report = json.loads(fixture["fit_path"].read_text(encoding="utf-8"))
            report["status"] = "FAIL"
            self._write_json(fixture["fit_path"], report)
            fixture["freeze"]["paths"]["H1"]["fit_clearance_report_hash"] = self._sha(fixture["fit_path"])
            self._rewrite_freeze(fixture)
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "fit/clearance report is not PASS"):
                self._preflight(fixture)

    def test_rejects_legacy_json_as_launched_world_or_geometry_alias(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            fixture["freeze"]["simulator_assets"].pop("world_geometry_file")
            fixture["freeze"]["simulator_assets"].pop("world_geometry_hash")
            self._rewrite_freeze(fixture)
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "world_geometry_file"):
                self._preflight(fixture)

        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            assets = fixture["freeze"]["simulator_assets"]
            assets["world_geometry_file"] = assets["world_file"]
            assets["world_geometry_hash"] = assets["world_hash"]
            self._rewrite_freeze(fixture)
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "distinct files"):
                self._preflight(fixture)

    def test_formal_flag_requires_full_existing_toolchain_gate(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            with self.assertRaisesRegex(self.adapter.ReplayContractError, "formal freeze gate did not PASS"):
                self._preflight(fixture, require_formal=True)

    def test_preflight_cli_is_offline_and_emits_no_formal_execution_authorization(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                status = self.adapter.main(
                    [
                        "preflight",
                        "--freeze",
                        str(fixture["freeze_path"].resolve()),
                        "--freeze-sha256",
                        fixture["freeze_hash"],
                        "--path-id",
                        "H1",
                    ]
                )
            self.assertEqual(0, status)
            report = json.loads(stdout.getvalue())
            self.assertEqual("PASS", report["status"])
            self.assertFalse(report["formal_execution_authorized"])


if __name__ == "__main__":
    unittest.main()
