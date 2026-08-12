#!/usr/bin/env python3
"""Synthetic in-memory tests for the finalized-BI4 surface adapter v1."""

from __future__ import annotations

import ast
import copy
import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SCRIPT = SCRIPTS / "r8_liquid_s5b0_container_surface_proxy_from_bi4_v1.py"
SCHEMA = ROOT / "schema/target_host_s5b0_container_surface_proxy_from_bi4_v1.json"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s5b0_container_surface_proxy_from_bi4_v1 as adapter  # noqa: E402


class S5B0ContainerSurfaceProxyFromBi4V1Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = adapter.synthetic_inputs()

    def produce(self, fixture=None):
        value = self.fixture if fixture is None else fixture
        return adapter.adapt_manifest_particles(
            value["manifest"], value["particle_frames"],
            value["settled_boundary_reference"], **value["kwargs"],
        )

    @staticmethod
    def refresh_limits(particle_frame):
        positions = particle_frame["particles"]["positions_m"]
        particle_frame["particles"]["limits_m"] = {
            axis: [min(row[column] for row in positions), max(row[column] for row in positions)]
            for axis, column in (("x", 0), ("y", 1), ("z", 2))
        }

    def test_golden_binds_manifest_frames_h0_se3_q98_and_non_gauge(self) -> None:
        result = self.produce()
        self.assertEqual(result["source_contract"]["finalized_manifest_sha256"],
                         self.fixture["kwargs"]["expected_manifest_sha256"])
        self.assertEqual(result["baseline"]["h0_m"], 0.2)
        self.assertEqual(result["baseline"]["h0_source_method"], adapter.H0_SOURCE_METHOD)
        self.assertTrue(result["surface_contract"]["explicit_non_gauge"])
        self.assertFalse(result["surface_contract"]["is_solver_gauge"])
        self.assertFalse(result["surface_contract"]["is_jgaugeswl"])
        self.assertFalse(result["surface_contract"]["is_point_probe"])
        self.assertTrue(result["qc"]["algorithm_valid"])
        for output, source in zip(result["frames"], self.fixture["manifest"]["frames"]):
            self.assertEqual(output["source_identity"], source["identity"])
            self.assertEqual(output["particle_ids_sha256"], source["ids_sha256"])
            self.assertEqual(output["time_s"], source["time_s"])
            self.assertEqual(len(output["transform"]["world_to_container_rotation"]), 3)
            self.assertEqual([row["sector_id"] for row in output["sectors"]],
                             [f"s{index:02d}" for index in range(16)])
            self.assertTrue(all(row["valid"] for row in output["sectors"]))
            self.assertAlmostEqual(
                output["sectors"][0]["h_proxy_relative_h0_m"],
                output["sectors"][0]["q98_z_m"] - 0.2,
            )

    def test_manifest_and_frame_identity_hash_drift_fail_closed(self) -> None:
        changed = copy.deepcopy(self.fixture)
        changed["manifest"]["frames"][0]["identity"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "manifest hash drift"):
            self.produce(changed)

        changed = copy.deepcopy(self.fixture)
        changed["particle_frames"][0]["identity"]["inode"] += 1
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "SHA/inode identity drift"):
            self.produce(changed)

        changed = copy.deepcopy(self.fixture)
        changed["manifest"]["integrity"]["frame_manifest_sha256"] = "0" * 64
        changed["kwargs"]["expected_manifest_sha256"] = adapter.sha256_json(changed["manifest"])
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "frame hash"):
            self.produce(changed)

    def test_time_id_class_and_boundary_reference_drift_fail_closed(self) -> None:
        changed = copy.deepcopy(self.fixture)
        changed["particle_frames"][1]["time_s"] += 0.001
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "time drift"):
            self.produce(changed)

        changed = copy.deepcopy(self.fixture)
        changed["particle_frames"][0]["particles"]["ids"][0] = 99
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "ID sequence drift"):
            self.produce(changed)

        changed = copy.deepcopy(self.fixture)
        changed["particle_frames"][0]["particles"]["classes"][0] = "fluid"
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "class assignment drift"):
            self.produce(changed)

        changed = copy.deepcopy(self.fixture)
        changed["settled_boundary_reference"]["particles"][0]["xyz_m"][0] += 0.1
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "reference hash drift"):
            self.produce(changed)

        changed = copy.deepcopy(self.fixture)
        changed["settled_boundary_reference"]["source_identity"]["inode"] += 1
        changed["kwargs"]["expected_boundary_reference_sha256"] = adapter.sha256_json(
            changed["settled_boundary_reference"]
        )
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "source identity drift"):
            self.produce(changed)

    def test_missing_sector_is_explicit_null_and_null_forgery_is_rejected(self) -> None:
        changed = copy.deepcopy(self.fixture)
        first_id = 6 + 5 * 128
        for particle_frame in changed["particle_frames"]:
            positions = particle_frame["particles"]["positions_m"]
            for particle_id in range(first_id, first_id + 128):
                positions[particle_id][0] += 10.0
                positions[particle_id][1] += 10.0
            self.refresh_limits(particle_frame)
        result = self.produce(changed)
        self.assertFalse(result["qc"]["algorithm_valid"])
        for frame in result["frames"]:
            sector = frame["sectors"][5]
            self.assertFalse(sector["valid"])
            self.assertIsNone(sector["q98_z_m"])
            self.assertIsNone(sector["h_proxy_relative_h0_m"])
            self.assertEqual(sector["null_reason"], "INSUFFICIENT_PARTICLES")

        forged = copy.deepcopy(result)
        forged["frames"][0]["sectors"][5]["q98_z_m"] = 1.0
        forged["integrity"]["output_frames_sha256"] = adapter.sha256_json(forged["frames"])
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "schema failure"):
            adapter.validate_result(forged)

    def test_internally_rehashed_result_forgery_is_recomputed_and_rejected(self) -> None:
        result = self.produce()
        forged = copy.deepcopy(result)
        sector = forged["frames"][0]["sectors"][0]
        sector["q98_z_m"] += 0.5
        sector["h_proxy_relative_h0_m"] += 0.5
        forged["integrity"]["output_frames_sha256"] = adapter.sha256_json(forged["frames"])
        adapter.validate_result(forged)
        with self.assertRaisesRegex(adapter.SurfaceProxyFromBi4Error, "fresh BI4 particle derivation"):
            adapter.verify_result(
                self.fixture["manifest"], self.fixture["particle_frames"],
                self.fixture["settled_boundary_reference"], forged, **self.fixture["kwargs"],
            )
        adapter.verify_result(
            self.fixture["manifest"], self.fixture["particle_frames"],
            self.fixture["settled_boundary_reference"], result, **self.fixture["kwargs"],
        )

    def test_schema_ast_cli_surface_and_self_check_are_fixture_only(self) -> None:
        schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        adapter.assert_deep_closed(schema)
        changed = self.produce()
        changed["unexpected"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)

        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = set()
        forbidden_calls = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                if node.func.attr in {"write_bytes", "write_text", "system", "popen", "run", "Popen"}:
                    forbidden_calls.append(node.func.attr)
        self.assertFalse(imports & {"rosbag", "rospy", "socket", "requests", "subprocess"})
        self.assertEqual(forbidden_calls, [])
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("output-root", source)
        report = adapter.self_check()
        self.assertIn("FIXTURE_SELF_CHECK_OK_NOT_ADMITTED", report["status"])
        for key in ("real_external_output_read", "real_bi4_read", "real_bag_read",
                    "solver_or_gpu_executed", "network_used", "sudo_used", "stage5b0_pass"):
            self.assertFalse(report[key], key)


if __name__ == "__main__":
    unittest.main()
