#!/usr/bin/env python3
"""Static and negative tests for frozen CPU/GPU backend-parity QC."""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_cpu_gpu_backend_parity_qc_v1 as qc  # noqa: E402


def frame(offset: float, digest: str) -> dict[str, object]:
    return {
        "sha256": digest,
        "canonical_array_sha256": digest,
        "time_s": 0.0,
        "ids": [2669, 2670],
        "positions": [(offset, 0.0, 0.0), (0.0, offset, 0.0)],
        "velocities": [(0.0, 0.0, offset), (offset, 0.0, 0.0)],
        "densities": [1000.0 + offset, 1000.0],
    }


class CpuGpuBackendParityQcV1Tests(unittest.TestCase):
    def test_schema_is_valid_and_all_object_definitions_are_closed(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertIs(schema["additionalProperties"], False)
        for name, definition in schema["$defs"].items():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False, name)

    def test_seven_tolerances_are_exactly_frozen_in_schema(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        observed = {
            name: value["const"] for name, value in schema["$defs"]["tolerances"]["properties"].items()
        }
        self.assertEqual(observed, qc.TOLERANCES)
        self.assertEqual(qc.TOLERANCES["position_l2_max_m"], 0.05 * 0.001)
        self.assertFalse(schema["$defs"]["contract"]["properties"]["threshold_overrides_allowed"]["const"])

    def test_metrics_at_limits_pass_and_each_exceedance_fails(self) -> None:
        result = qc.adjudicate(True, True, dict(qc.TOLERANCES), dict(qc.TOLERANCES))
        self.assertTrue(result["selected"])
        for name, limit in qc.TOLERANCES.items():
            values = dict(qc.TOLERANCES)
            values[name] = limit + max(abs(limit) * 1e-9, 1e-15)
            result = qc.adjudicate(True, True, values, dict(qc.TOLERANCES))
            self.assertFalse(result["selected"], name)
            self.assertFalse(result["all_particle_metric_pass"][name])

    def test_repeatability_failure_or_invalid_metric_cannot_pass(self) -> None:
        self.assertFalse(qc.adjudicate(False, True, dict(qc.TOLERANCES), dict(qc.TOLERANCES))["selected"])
        self.assertFalse(qc.adjudicate(True, False, dict(qc.TOLERANCES), dict(qc.TOLERANCES))["selected"])
        for invalid in (-1.0, float("nan"), float("inf"), True):
            values = dict(qc.TOLERANCES)
            values["time_abs_max_s"] = invalid
            with self.assertRaises(qc.BackendParityQcError):
                qc.adjudicate(True, True, values, dict(qc.TOLERANCES))

    def test_aggregate_uses_all_pairs_and_l2_metrics(self) -> None:
        left = frame(0.0, "a" * 64)
        right = frame(0.000004, "b" * 64)
        with mock.patch.object(qc, "EXPECTED_PARTICLES", 2), mock.patch.object(qc, "FLUID_FIRST_ID", 0):
            left["ids"] = [0, 1]
            right["ids"] = [1, 0]
            right["positions"] = list(reversed(right["positions"]))
            right["velocities"] = list(reversed(right["velocities"]))
            right["densities"] = list(reversed(right["densities"]))

            def fake_frame(root: Path, part: int) -> dict[str, object]:
                del part
                return left if root.name == "left" else right

            with mock.patch.object(qc, "PARTS", (0, 1)), mock.patch.object(qc, "_frame", side_effect=fake_frame):
                value = qc._aggregate(Path("/left"), Path("/right"), fluid_only=False)
        self.assertEqual(value["frame_count"], 2)
        self.assertEqual(value["particle_pairs"], 4)
        self.assertAlmostEqual(value["metrics"]["position_l2_max_m"], 0.000004)
        self.assertAlmostEqual(value["metrics"]["velocity_l2_max_m_s"], 0.000004)
        self.assertTrue(value["pass"])

    def test_repeatability_distinguishes_raw_and_canonical_requirements(self) -> None:
        left = frame(0.0, "a" * 64)
        right = frame(0.0, "a" * 64)
        right_raw_drift = dict(right)
        right_raw_drift["sha256"] = "b" * 64

        def fake_frame(root: Path, part: int) -> dict[str, object]:
            del part
            return left if root.name == "left" else right_raw_drift

        with mock.patch.object(qc, "PARTS", (0, 1)), mock.patch.object(qc, "_frame", side_effect=fake_frame):
            canonical = qc._repeatability(Path("/left"), Path("/right"), raw_required=False)
            raw = qc._repeatability(Path("/left"), Path("/right"), raw_required=True)
        self.assertTrue(canonical["pass"])
        self.assertFalse(raw["pass"])
        self.assertEqual(canonical["canonical_equal_count"], 2)
        self.assertEqual(canonical["raw_equal_count"], 0)

    def test_exclusive_writer_handles_short_writes_and_rejects_reuse(self) -> None:
        report = {"status": "unit-test", "payload": "x" * 100}
        encoded = (json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode()
        original_write = os.write

        def short_write(descriptor: int, data: bytes | memoryview) -> int:
            return original_write(descriptor, bytes(data[:9]))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "receipt.json"
            with mock.patch.object(qc, "OUTPUT_PATH", path), mock.patch.object(qc.os, "write", side_effect=short_write):
                identity = qc.write_exclusive(path, report)
                with self.assertRaises(FileExistsError):
                    qc.write_exclusive(path, report)
            self.assertEqual(path.read_bytes(), encoded)
            self.assertEqual(identity["mode"], "0640")

    def test_static_self_check_is_read_only(self) -> None:
        value = qc.static_self_check()
        self.assertEqual(value["metric_count"], 7)
        self.assertEqual(value["part_count"], 6)
        self.assertFalse(value["threshold_overrides_allowed"])
        self.assertFalse(value["solver_executed"])
        self.assertFalse(value["gpu_exposed"])
        self.assertFalse(value["network_used"])


if __name__ == "__main__":
    unittest.main()
