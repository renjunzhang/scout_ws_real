#!/usr/bin/env python3
"""Fail-closed tests for the fixture-only S5B0 container surface proxy v3."""

from __future__ import annotations

import ast
import copy
import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np
from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_s5b0_container_surface_proxy_v3.py"
SCHEMA = ROOT / "schema/target_host_s5b0_container_surface_proxy_v3.json"


def load_module():
    spec = importlib.util.spec_from_file_location("s5b0_container_surface_proxy_v3_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


proxy = load_module()


class S5B0ContainerSurfaceProxyV3Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = proxy.synthetic_fixture()

    def test_full_se3_golden_q98_and_non_gauge_claims(self) -> None:
        result = proxy.analyze_fixture(self.fixture)
        self.assertEqual(result["surface_source"], "OFFLINE_CONTAINER_FRAME_PARTICLE_Q98_PROXY")
        self.assertFalse(result["raw_moving_solver_gauge_available"])
        self.assertFalse(result["is_solver_gauge"])
        self.assertFalse(result["is_jgaugeswl"])
        self.assertFalse(result["is_point_probe"])
        self.assertFalse(result["claims"]["real_bi4_read"])
        self.assertFalse(result["claims"]["solver_or_gpu_executed"])
        self.assertFalse(result["claims"]["stage5b0_pass"])
        self.assertEqual(result["result_status"], "FIXTURE_ONLY_PROXY_VALID_NOT_ADMITTED")
        self.assertTrue(result["qc"]["algorithm_valid"])

        for frame in result["frames"]:
            self.assertEqual(
                [row["sector_id"] for row in frame["sectors"]], list(proxy.SECTOR_IDS)
            )
            self.assertEqual([row["count"] for row in frame["sectors"]], [128] * 16)
            self.assertTrue(all(row["valid"] for row in frame["sectors"]))
            self.assertTrue(all(row["null_reason"] is None for row in frame["sectors"]))
        self.assertAlmostEqual(result["frames"][0]["sectors"][0]["q98_z_m"], 0.32446)
        self.assertAlmostEqual(result["frames"][1]["sectors"][0]["q98_z_m"], 0.32946)
        self.assertAlmostEqual(result["frames"][2]["sectors"][15]["q98_z_m"], 0.48446)

        expected_rotation = proxy._rotation_zyx(0.21, -0.17, 0.13).T
        translation = np.asarray((0.12, -0.08, 0.05), dtype=np.float64)
        expected_translation = -expected_rotation @ translation
        observed = result["frames"][1]["transform"]
        np.testing.assert_allclose(
            np.asarray(observed["world_to_container_rotation"]),
            expected_rotation,
            atol=1e-12,
            rtol=0.0,
        )
        np.testing.assert_allclose(
            np.asarray(observed["world_to_container_translation_m"]),
            expected_translation,
            atol=1e-12,
            rtol=0.0,
        )
        self.assertLess(observed["rms_residual_m"], 1e-12)
        self.assertAlmostEqual(observed["determinant"], 1.0, places=12)
        self.assertEqual(
            proxy.canonical_json(result), proxy.canonical_json(proxy.analyze_fixture(self.fixture))
        )

    def test_shear_and_residual_are_rejected(self) -> None:
        changed = copy.deepcopy(self.fixture)
        changed["frames"][1]["moving_boundary"][0]["xyz_m"][0] += 1e-3
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "residual"):
            proxy.analyze_fixture(changed)

    def test_reflection_and_degenerate_reference_are_rejected(self) -> None:
        reflected = copy.deepcopy(self.fixture)
        for row in reflected["frames"][0]["moving_boundary"]:
            row["xyz_m"][0] = -row["xyz_m"][0]
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "reflection"):
            proxy.analyze_fixture(reflected)

        degenerate = copy.deepcopy(self.fixture)
        for row in degenerate["settled_moving_boundary"]:
            row["xyz_m"][2] = 0.0
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "degenerate"):
            proxy.analyze_fixture(degenerate)

    def test_missing_sector_is_explicit_null_and_cherry_pick_is_rejected(self) -> None:
        changed = copy.deepcopy(self.fixture)
        first_id = 10_000 + 5 * proxy.MINIMUM_PARTICLES_PER_SECTOR
        missing_ids = set(range(first_id, first_id + proxy.MINIMUM_PARTICLES_PER_SECTOR))
        for frame in changed["frames"]:
            frame["fluid"] = [row for row in frame["fluid"] if row["id"] not in missing_ids]
        result = proxy.analyze_fixture(changed)
        self.assertFalse(result["qc"]["algorithm_valid"])
        self.assertEqual(result["qc"]["invalid_sector_count"], 3)
        self.assertEqual(
            result["result_status"], "FIXTURE_ONLY_PROXY_INVALID_SECTORS_NOT_ADMITTED"
        )
        for frame in result["frames"]:
            self.assertEqual(len(frame["sectors"]), 16)
            row = frame["sectors"][5]
            self.assertEqual(row["sector_id"], "s05")
            self.assertEqual(row["count"], 0)
            self.assertFalse(row["valid"])
            self.assertIsNone(row["q98_z_m"])
            self.assertEqual(row["null_reason"], "INSUFFICIENT_PARTICLES")

        cherry_picked = copy.deepcopy(result)
        cherry_picked["frames"][0]["sectors"].pop(5)
        cherry_picked["frames"][0]["sectors"].append(
            copy.deepcopy(cherry_picked["frames"][0]["sectors"][-1])
        )
        cherry_picked["integrity"]["derived_frames_sha256"] = proxy.sha256_json(
            cherry_picked["frames"]
        )
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "cherry-picked"):
            proxy.validate_result(cherry_picked)

        preselected = copy.deepcopy(self.fixture)
        preselected["frames"][0]["sectors"] = ["s00"]
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "extra"):
            proxy.analyze_fixture(preselected)

    def test_nonfinite_duplicate_missing_and_overlapping_ids_are_rejected(self) -> None:
        nonfinite = copy.deepcopy(self.fixture)
        nonfinite["frames"][0]["fluid"][0]["xyz_m"][2] = math.nan
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "non-finite"):
            proxy.analyze_fixture(nonfinite)

        duplicate = copy.deepcopy(self.fixture)
        duplicate["frames"][0]["fluid"][1]["id"] = duplicate["frames"][0]["fluid"][0]["id"]
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "duplicate"):
            proxy.analyze_fixture(duplicate)

        missing = copy.deepcopy(self.fixture)
        missing["frames"][1]["moving_boundary"].pop()
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "missing"):
            proxy.analyze_fixture(missing)

        overlap = copy.deepcopy(self.fixture)
        boundary_id = overlap["settled_moving_boundary"][0]["id"]
        for frame in overlap["frames"]:
            frame["fluid"][0]["id"] = boundary_id
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "overlap"):
            proxy.analyze_fixture(overlap)

    def test_frame_and_time_discontinuity_are_rejected(self) -> None:
        omitted = copy.deepcopy(self.fixture)
        omitted["frames"].pop(1)
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "frame count"):
            proxy.analyze_fixture(omitted)

        reordered = copy.deepcopy(self.fixture)
        reordered["frames"][0], reordered["frames"][1] = (
            reordered["frames"][1], reordered["frames"][0]
        )
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "sequence"):
            proxy.analyze_fixture(reordered)

        time_gap = copy.deepcopy(self.fixture)
        time_gap["frames"][1]["time_s"] += 0.001
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "time sequence"):
            proxy.analyze_fixture(time_gap)

    def test_internally_rehashed_result_forgery_is_recomputed_and_rejected(self) -> None:
        result = proxy.analyze_fixture(self.fixture)
        forged = copy.deepcopy(result)
        forged["frames"][0]["sectors"][0]["q98_z_m"] += 0.5
        forged["integrity"]["derived_frames_sha256"] = proxy.sha256_json(forged["frames"])
        proxy.validate_result(forged)
        with self.assertRaisesRegex(proxy.SurfaceProxyV3Error, "fresh particle derivation"):
            proxy.verify_result(self.fixture, forged)
        proxy.verify_result(self.fixture, result)

    def test_schema_ast_and_self_check_are_closed_static_and_fixture_only(self) -> None:
        schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        proxy.assert_deep_closed(schema)
        result = proxy.analyze_fixture(self.fixture)
        changed = copy.deepcopy(result)
        changed["unexpected"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)

        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("gauge_zsurf", source.lower())
        self.assertNotIn('"probe_id"', source)
        tree = ast.parse(source)
        imports: set[str] = set()
        forbidden_calls: list[str] = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call):
                name = None
                if isinstance(node.func, ast.Name):
                    name = node.func.id
                elif isinstance(node.func, ast.Attribute):
                    name = node.func.attr
                if name in {"open", "write_text", "write_bytes", "system", "popen", "run", "Popen"}:
                    forbidden_calls.append(name)
        self.assertFalse(imports & {"rosbag", "rospy", "socket", "requests", "subprocess"})
        self.assertEqual(forbidden_calls, [])

        report = proxy.self_check()
        self.assertEqual(
            report["status"],
            "S5B0_CONTAINER_SURFACE_PROXY_V3_FIXTURE_SELF_CHECK_OK_NOT_ADMITTED",
        )
        self.assertEqual(report["sector_ids"], list(proxy.SECTOR_IDS))
        self.assertFalse(report["real_bi4_read"])
        self.assertFalse(report["solver_or_gpu_executed"])
        self.assertFalse(report["stage5b0_pass"])


if __name__ == "__main__":
    unittest.main()
