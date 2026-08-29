#!/usr/bin/env python3

import hashlib
import importlib.util
import json
import math
import pathlib
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


SCRIPT_ROOT = pathlib.Path(__file__).resolve().parents[1]
ANALYSIS_ROOT = SCRIPT_ROOT / "analysis"
REPO_ROOT = SCRIPT_ROOT.parents[4]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MAP_VALIDATOR = load_module(
    "validate_mocap_field_map", ANALYSIS_ROOT / "validate_mocap_field_map.py"
)
LOCALIZATION_VALIDATOR = load_module(
    "validate_mocap_field_localization",
    ANALYSIS_ROOT / "validate_mocap_field_localization.py",
)


class FakeMapMixin:
    def make_map(self, resolution=0.02, image="field.pgm"):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        stem = pathlib.Path(directory.name) / "field"
        pbstream = stem.with_suffix(".pbstream")
        pgm = stem.with_suffix(".pgm")
        yaml_path = stem.with_suffix(".yaml")
        pbstream.write_bytes(b"fake-cartographer-state\x00\x01")
        pgm.write_bytes(b"P5\n# fixture\n3 2\n255\n" + bytes((0, 1, 2, 3, 4, 5)))
        yaml_path.write_text(
            yaml.safe_dump(
                {
                    "image": image,
                    "resolution": resolution,
                    "origin": [-1.0, -2.0, 0.0],
                    "negate": 0,
                    "occupied_thresh": 0.65,
                    "free_thresh": 0.196,
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return stem


class OfflineMapValidationTest(FakeMapMixin, unittest.TestCase):
    def test_valid_asset_set_and_hashes_pass(self):
        stem = self.make_map()
        expected = hashlib.sha256(stem.with_suffix(".pbstream").read_bytes()).hexdigest()
        report = MAP_VALIDATOR.validate_map_assets(
            stem, expected_resolution=0.02, expected_pbstream_sha256=expected
        )
        self.assertTrue(report["pass"], report["failures"])
        self.assertEqual(report["pgm"]["width"], 3)
        self.assertEqual(report["pgm"]["height"], 2)
        self.assertEqual(report["yaml"]["resolved_image"], str(stem.with_suffix(".pgm")))

    def test_yaml_must_resolve_to_matching_pgm(self):
        stem = self.make_map(image="another_map.pgm")
        report = MAP_VALIDATOR.validate_map_assets(stem)
        self.assertFalse(report["pass"])
        self.assertTrue(
            any("image resolves" in failure for failure in report["failures"]),
            report["failures"],
        )

    def test_resolution_mismatch_is_rejected(self):
        stem = self.make_map(resolution=0.05)
        report = MAP_VALIDATOR.validate_map_assets(stem, expected_resolution=0.02)
        self.assertFalse(report["pass"])
        self.assertTrue(
            any("resolution mismatch" in failure for failure in report["failures"])
        )

    def test_wrong_pbstream_hash_is_rejected(self):
        stem = self.make_map()
        report = MAP_VALIDATOR.validate_map_assets(
            stem, expected_pbstream_sha256="0" * 64
        )
        self.assertFalse(report["pass"])
        self.assertTrue(any("SHA-256 mismatch" in item for item in report["failures"]))


class StaticLocalizationMathTest(unittest.TestCase):
    def test_static_pose_metrics_accept_small_jitter(self):
        rows = [(1.0 + index * 1e-4, 2.0, 0.2 + index * 1e-5) for index in range(20)]
        metrics = LOCALIZATION_VALIDATOR.static_pose_metrics(rows)
        self.assertEqual(metrics["count"], 20)
        self.assertLess(metrics["position_deviation_p95_m"], 0.002)
        self.assertLess(metrics["yaw_deviation_p95_rad"], 0.001)

    def test_stream_stats_reports_rate_gap_and_header_regression(self):
        stream = LOCALIZATION_VALIDATOR.StreamStats(True)
        for index in range(11):
            stream.observe(1.0 + index * 0.01, 10.0 + index * 0.01)
        summary = stream.summary()
        self.assertAlmostEqual(summary["arrival"]["rate_hz"], 100.0, delta=1e-6)
        self.assertAlmostEqual(summary["arrival"]["max_gap_sec"], 0.01, delta=1e-9)
        stream.observe(1.2, 9.0)
        self.assertEqual(stream.summary()["header_regressions"], 1)


class FieldMapWiringTest(FakeMapMixin, unittest.TestCase):
    def test_mapping_launch_has_one_dynamic_map_to_odom_owner(self):
        launch_path = (
            REPO_ROOT
            / "src/scout_apps/sensors/nanoscan3_mapping/launch/scout_nanoscan3_cartographer.launch"
        )
        root = ET.parse(str(launch_path)).getroot()
        static_nodes = [
            node
            for node in root.findall("node")
            if node.attrib.get("type") == "static_transform_publisher"
        ]
        self.assertEqual(static_nodes, [])
        args = {node.attrib["name"] for node in root.findall("arg")}
        self.assertIn("occupancy_grid_resolution", args)
        self.assertIn("respawn", args)

    def test_localization_launch_exposes_frozen_map_evidence(self):
        launch_path = (
            REPO_ROOT
            / "src/scout_apps/sensors/nanoscan3_localization/launch/scout_nanoscan3_cartographer_localization.launch"
        )
        text = launch_path.read_text(encoding="utf-8")
        self.assertIn('name="map_expected_sha256"', text)
        self.assertIn('name="frozen_map_file"', text)
        self.assertIn('name="frozen_map_expected_sha256"', text)

    def test_sensor_stack_forwards_map_hash_and_requires_readiness(self):
        stack = (
            REPO_ROOT
            / "src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_stack.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('map_file:="${LOCALIZATION_MAP_FILE}"', stack)
        self.assertIn('map_expected_sha256:="${LOCALIZATION_MAP_EXPECTED_SHA256}"', stack)
        self.assertIn('wait_for_topic /map "Cartographer occupancy grid" true', stack)
        self.assertIn('WAIT_FOR_LOCALIZATION_TF="true"', stack)
        self.assertIn('START_REALSENSE="${START_REALSENSE:-true}"', stack)

        mapping_stack = (
            REPO_ROOT
            / "src/scout_apps/control/scout_local_planner/scripts/launch_real_sensors_no_localization.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("export START_LOCALIZATION=false", mapping_stack)
        self.assertIn("export WAIT_FOR_LOCALIZATION_MAP=false", mapping_stack)

    def test_freeze_is_disarmed_and_validate_only_writes_nothing(self):
        freeze = SCRIPT_ROOT / "freeze_spmpc_mocap_field_map.sh"
        text = freeze.read_text(encoding="utf-8")
        self.assertIn('ARM_MAP_FREEZE="${ARM_MAP_FREEZE:-NO}"', text)
        self.assertIn('[[ "${ARM_MAP_FREEZE}" == "YES" ]]', text)
        self.assertIn("/cmd_vel has a publisher", text)
        self.assertIn('[[ "${finish_code}" == "0" ]]', text)
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                ["bash", str(freeze)],
                env={
                    "PATH": "/usr/bin:/bin",
                    "HOME": directory,
                    "MAP_ID": "map_carto_20990101_unit_test",
                    "MAP_OUTPUT_ROOT": directory,
                    "VALIDATE_ONLY": "true",
                },
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("no ROS query, service call or file write", result.stdout)
            self.assertEqual(list(pathlib.Path(directory).iterdir()), [])

    def test_freeze_refuses_large_field_map_identity(self):
        freeze = SCRIPT_ROOT / "freeze_spmpc_mocap_field_map.sh"
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                ["bash", str(freeze)],
                env={
                    "PATH": "/usr/bin:/bin",
                    "HOME": directory,
                    "MAP_ID": "map_carto_20260629_R0",
                    "MAP_OUTPUT_ROOT": directory,
                    "VALIDATE_ONLY": "true",
                },
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("reserved by the large-field G3R3 map", result.stderr)
            self.assertEqual(list(pathlib.Path(directory).iterdir()), [])

    def test_path_selection_is_low_speed_nonformal_and_validate_only(self):
        selector = SCRIPT_ROOT / "run_spmpc_mocap_path_selection_trial.sh"
        text = selector.read_text(encoding="utf-8")
        self.assertIn('ARM_MOTION="${ARM_MOTION:-NO}"', text)
        self.assertIn("formal_trial_consumed", text)
        self.assertIn("MAX_SELECTION_V_REF", text)
        self.assertIn("PILOT_MODE=true", text)
        self.assertNotIn("TRIAL_ID=R01", text)

        stem = self.make_map()
        map_hash = hashlib.sha256(stem.with_suffix(".pbstream").read_bytes()).hexdigest()
        path = stem.parent / "candidate_s.json"
        points = []
        for index in range(81):
            x = 2.0 * index / 80.0
            points.append(
                {
                    "x": x,
                    "y": 0.22 * math.sin(math.pi * x),
                    "z": 0.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                    "qw": 1.0,
                }
            )
        path.write_text(json.dumps({"frame_id": "map", "poses": points}), encoding="utf-8")
        path_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        result = subprocess.run(
            ["bash", str(selector)],
            env={
                "PATH": "/usr/bin:/bin",
                "HOME": str(stem.parent),
                "VALIDATE_ONLY": "true",
                "SELECTION_ID": "C01",
                "PATH_FILE": str(path),
                "PATH_EXPECTED_SHA256": path_hash,
                "FIELD_MAP_FILE": str(stem.with_suffix(".pbstream")),
                "FIELD_MAP_EXPECTED_SHA256": map_hash,
                "FIELD_MAP_RESOLUTION": "0.02",
                "V_REF": "0.10",
            },
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("does not consume R01--R05", result.stdout)
        self.assertIn("selection=C01 attempt=01", result.stdout)
        self.assertFalse((stem.parent / "slosh_bags").exists())

        retry_env = {
            "PATH": "/usr/bin:/bin",
            "HOME": str(stem.parent),
            "VALIDATE_ONLY": "true",
            "SELECTION_ID": "C01",
            "ATTEMPT": "02",
            "ACQUISITION_RETRY": "true",
            "PATH_FILE": str(path),
            "PATH_EXPECTED_SHA256": path_hash,
            "FIELD_MAP_FILE": str(stem.with_suffix(".pbstream")),
            "FIELD_MAP_EXPECTED_SHA256": map_hash,
            "FIELD_MAP_RESOLUTION": "0.02",
            "V_REF": "0.10",
        }
        missing_reason = subprocess.run(
            ["bash", str(selector)],
            env=retry_env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(missing_reason.returncode, 2)
        self.assertIn("RETRY_REASON_FILE", missing_reason.stderr)

        retry_reason = stem.parent / "startup_abort_reason.log"
        retry_reason.write_text("planner startup contract rejection\n", encoding="utf-8")
        retry_env["RETRY_REASON_FILE"] = str(retry_reason)
        retry = subprocess.run(
            ["bash", str(selector)],
            env=retry_env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(retry.returncode, 0, retry.stderr)
        self.assertIn("selection=C01 attempt=02", retry.stdout)
        self.assertIn("acquisition_retry=true", retry.stdout)

        wrong_weight_env = dict(retry_env)
        wrong_weight_env["W_SLOSH"] = "4.0"
        wrong_weight = subprocess.run(
            ["bash", str(selector)],
            env=wrong_weight_env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(wrong_weight.returncode, 2)
        self.assertIn("W_SLOSH=5.0", wrong_weight.stderr)

    def test_formal_execution_chain_is_bound_to_runtime_field_map(self):
        formal = (SCRIPT_ROOT / "run_spmpc_mocap_execution_chain_trial.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("FIELD_MAP_EXPECTED_SHA256", formal)
        self.assertIn("validate_mocap_field_map.py", formal)
        self.assertIn("/cartographer_node/frozen_map_file", formal)
        self.assertIn("/cartographer_node/frozen_map_expected_sha256", formal)
        self.assertIn("Cartographer runtime map does not match", formal)
        self.assertIn("refuse the reserved large-field G3R3 map", formal)
        self.assertIn("matched release requires 0 < V_REF <= 0.20 m/s", formal)


if __name__ == "__main__":
    unittest.main()
