#!/usr/bin/env python3
"""Regression and negative tests for the zero-baseline-safe cold QC v3."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p2_cold_ab_qc_v3 as qc  # noqa: E402


def part(index: int, time_s: float, speed: float) -> dict:
    return {"cpart": index, "time_s": time_s, "speed": {"rms_m_s": speed}}


class ColdAbQcV3Tests(unittest.TestCase):
    def test_schema_is_closed_and_valid(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertFalse(schema["additionalProperties"])
        self.assertEqual(
            schema["properties"]["schema_version"]["const"],
            "r8-liquid-u3-viscoart0p2-cold-ab-qc-v3",
        )

    def test_zero_minimum_is_explicit_and_division_safe(self) -> None:
        value = qc._trajectory([part(0, 0.0, 0.0), part(701, 35.05, 0.0005)])
        self.assertTrue(value["minimum_is_zero"])
        self.assertIsNone(value["final_to_minimum_ratio"])
        self.assertTrue(value["rebound_after_minimum"])
        qc._validate_trajectory_representation(value)

    def test_positive_minimum_keeps_original_ratio(self) -> None:
        value = qc._trajectory([part(0, 0.0, 0.25), part(701, 35.05, 0.5)])
        self.assertFalse(value["minimum_is_zero"])
        self.assertEqual(value["final_to_minimum_ratio"], 2.0)
        qc._validate_trajectory_representation(value)

    def test_inconsistent_zero_encodings_are_rejected(self) -> None:
        value = qc._trajectory([part(0, 0.0, 0.0), part(701, 35.05, 0.5)])
        for key, replacement in (("minimum_is_zero", False), ("final_to_minimum_ratio", 1.0)):
            drifted = dict(value)
            drifted[key] = replacement
            with self.assertRaises(qc.ColdAbQcV3Error):
                qc._validate_trajectory_representation(drifted)

    def test_negative_or_nonfinite_speed_is_rejected(self) -> None:
        for speed in (-1.0, float("inf"), float("nan")):
            with self.assertRaises(qc.ColdAbQcV3Error):
                qc._trajectory([part(0, 0.0, speed)])

    def test_only_zero_baseline_schema_representation_changed(self) -> None:
        self.assertTrue(qc._schemas_differ_only_by_zero_baseline_representation())

    def test_static_self_check_has_no_execution_or_threshold_delta(self) -> None:
        result = qc.static_self_check()
        self.assertEqual(result["single_representation_delta"], "ZERO_MINIMUM_TO_EXPLICIT_NULL_RATIO")
        self.assertFalse(result["threshold_changed"])
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
