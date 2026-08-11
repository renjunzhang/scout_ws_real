#!/usr/bin/env python3
"""Static and negative tests for viscosity-0.3 repeatability/restart QC."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p3_repeatability_restart_qc_v1 as qc  # noqa: E402


class RepeatabilityRestartQcV1Tests(unittest.TestCase):
    def test_schema_is_valid_and_all_object_definitions_are_closed(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertIs(schema["additionalProperties"], False)
        for name, definition in schema["$defs"].items():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False, name)
        self.assertEqual(
            schema["properties"]["schema_version"]["const"],
            "r8-liquid-u3-viscoart0p3-repeatability-restart-qc-v1",
        )

    def test_schema_freezes_cardinality_checkpoint_and_tolerances(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        self.assertEqual(schema["$defs"]["preRepeatability"]["properties"]["part_count"]["const"], 702)
        self.assertEqual(schema["$defs"]["postEquivalence"]["properties"]["frame_count"]["const"], 201)
        self.assertEqual(
            schema["$defs"]["postEquivalence"]["properties"]["particle_pairs"]["const"],
            201 * qc.EXPECTED_PARTICLES,
        )
        observed = {
            name: item["const"] for name, item in schema["$defs"]["tolerances"]["properties"].items()
        }
        self.assertEqual(observed, qc.TOLERANCES)
        self.assertFalse(schema["$defs"]["contract"]["properties"]["threshold_overrides_allowed"]["const"])

    def test_schema_rejects_unknown_identity_fields(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        validator = Draft202012Validator(
            {
                "$schema": schema["$schema"],
                "$defs": schema["$defs"],
                "$ref": "#/$defs/identity",
            }
        )
        value = {
            "path": "/tmp/a",
            "sha256": "a" * 64,
            "size_bytes": 1,
            "mode": "0440",
            "inode": 1,
            "nlink": 1,
            "unexpected": True,
        }
        self.assertTrue(list(validator.iter_errors(value)))

    def test_all_metrics_at_limits_pass_and_each_exceedance_fails(self) -> None:
        result = qc.adjudicate(True, True, dict(qc.TOLERANCES))
        self.assertTrue(result["selected"])
        self.assertEqual(result["status"], "U3_SETTLED_STATE_FROZEN")
        for name in sorted(qc.TOLERANCES):
            values = dict(qc.TOLERANCES)
            values[name] = values[name] + max(abs(values[name]) * 1e-9, 1e-15)
            result = qc.adjudicate(True, True, values)
            self.assertFalse(result["selected"], name)
            self.assertFalse(result["metric_pass"][name], name)

    def test_structural_failure_and_invalid_metrics_cannot_pass(self) -> None:
        self.assertFalse(qc.adjudicate(False, True, dict(qc.TOLERANCES))["selected"])
        self.assertFalse(qc.adjudicate(True, False, dict(qc.TOLERANCES))["selected"])
        for invalid in (-1.0, float("nan"), float("inf"), True):
            values = dict(qc.TOLERANCES)
            values["position_l2_max_m"] = invalid
            with self.assertRaises(qc.RepeatabilityRestartQcError):
                qc.adjudicate(True, True, values)
        values = dict(qc.TOLERANCES)
        values.pop("time_abs_max_s")
        with self.assertRaises(qc.RepeatabilityRestartQcError):
            qc.adjudicate(True, True, values)

    def test_per_id_array_comparison_uses_l2_and_symmetric_density_error(self) -> None:
        left = {
            "ids": [0, 1],
            "time_s": 1.0,
            "positions": [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0)],
            "velocities": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
            "densities": [1000.0, 1000.0],
        }
        right = {
            "ids": [0, 1],
            "time_s": 1.00005,
            "positions": [(3.0, 4.0, 0.0), (1.0, 2.0, 3.0)],
            "velocities": [(0.0, 0.0, 0.0), (1.0, 0.0, 2.0)],
            "densities": [1001.0, 1000.0],
        }
        value = qc.compare_arrays(left, right)
        self.assertAlmostEqual(value["time_abs_s"], 0.00005)
        self.assertEqual(value["position_l2_m"]["max"], 5.0)
        self.assertAlmostEqual(value["position_l2_m"]["rms"], (25.0 / 2.0) ** 0.5)
        self.assertEqual(value["velocity_l2_m_s"]["max"], 2.0)
        self.assertAlmostEqual(value["density_relative"]["max"], 1.0 / 1001.0)

    def test_ordered_id_drift_is_rejected(self) -> None:
        value = {
            "ids": [0, 1],
            "time_s": 0.0,
            "positions": [(0.0, 0.0, 0.0)] * 2,
            "velocities": [(0.0, 0.0, 0.0)] * 2,
            "densities": [1000.0] * 2,
        }
        other = dict(value)
        other["ids"] = [1, 0]
        with self.assertRaises(qc.RepeatabilityRestartQcError):
            qc.compare_arrays(value, other)

    def test_raw_mismatch_count_is_not_truncated_with_prefix(self) -> None:
        def fake_sha(path: Path, *, maximum: int) -> str:
            del maximum
            part = int(path.stem.split("_")[1])
            if path.is_relative_to(qc.B_CONTINUOUS_ROOT) and part < 35:
                return "b" * 64
            return "a" * 64

        with mock.patch.object(qc, "PRE_PARTS", tuple(range(40))), mock.patch.object(
            qc.legacy, "sha256_file", side_effect=fake_sha
        ):
            value = qc._raw_repeatability()
        self.assertEqual(value["mismatch_count"], 35)
        self.assertEqual(value["raw_sha256_equal_count"], 5)
        self.assertEqual(value["mismatch_parts_prefix"], list(range(32)))
        self.assertFalse(value["pass"])

    def test_semantic_validator_rejects_inconsistent_pre_counts(self) -> None:
        report = {
            "pre_checkpoint_repeatability": {
                "mismatch_count": 1,
                "raw_sha256_equal_count": 702,
                "mismatch_parts_prefix": [1],
                "pass": False,
            }
        }
        with self.assertRaisesRegex(qc.RepeatabilityRestartQcError, "pre-checkpoint"):
            qc.validate_report(report)

    def test_exclusive_writer_handles_short_writes(self) -> None:
        report = {"status": "unit-test", "payload": "x" * 100}
        encoded = (json.dumps(report, sort_keys=True, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()
        original_write = os.write

        def short_write(descriptor: int, data: bytes | memoryview) -> int:
            return original_write(descriptor, bytes(data[:7]))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "receipt.json"
            with mock.patch.object(qc, "OUTPUT_PATH", path), mock.patch.object(qc.os, "write", side_effect=short_write):
                identity = qc.write_exclusive(path, report)
                with self.assertRaises(FileExistsError):
                    qc.write_exclusive(path, report)
            self.assertEqual(path.read_bytes(), encoded)
            self.assertEqual(identity["mode"], "0640")

    def test_static_self_check_is_read_only_and_exact(self) -> None:
        result = qc.static_self_check()
        self.assertEqual(result["metric_count"], 7)
        self.assertEqual(result["pre_frame_count"], 702)
        self.assertEqual(result["post_frame_count"], 201)
        self.assertFalse(result["threshold_overrides_allowed"])
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
