#!/usr/bin/env python3
"""Static and negative tests for the fresh viscosity-0.3 cold-A/B QC."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p3_cold_ab_qc_v1 as qc  # noqa: E402


class ColdAbQcV1Tests(unittest.TestCase):
    def test_schema_is_closed_and_valid(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertFalse(schema["additionalProperties"])
        for definition in schema["$defs"].values():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False)
        self.assertEqual(
            schema["properties"]["schema_version"]["const"],
            "r8-liquid-u3-viscoart0p3-cold-ab-qc-v1",
        )

    def test_real_inventory_is_admitted_with_exact_frozen_ceiling(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        bound = schema["$defs"]["inventory"]["properties"]["total_bytes"]
        validator = Draft202012Validator(bound)
        self.assertEqual(list(validator.iter_errors(282745515)), [])
        self.assertEqual(bound["maximum"], qc.runner.EXPECTED_OUTPUT["maximum_output_bytes"])

    def test_inventory_above_frozen_runner_limit_is_rejected(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        bound = schema["$defs"]["inventory"]["properties"]["total_bytes"]
        self.assertTrue(list(Draft202012Validator(bound).iter_errors(qc.runner.EXPECTED_OUTPUT["maximum_output_bytes"] + 1)))

    def test_zero_initial_speed_has_explicit_safe_representation(self) -> None:
        trajectory = qc._trajectory([
            {"cpart": 0, "time_s": 0.0, "speed": {"rms_m_s": 0.0}},
            {"cpart": 701, "time_s": 35.05, "speed": {"rms_m_s": 0.0005}},
        ])
        self.assertTrue(trajectory["minimum_is_zero"])
        self.assertIsNone(trajectory["final_to_minimum_ratio"])
        qc._validate_trajectory_representation(trajectory)

    def test_all_17_metrics_at_limits_pass_and_one_exceedance_fails(self) -> None:
        result = qc.adjudicate(dict(qc.METRIC_LIMITS), True, True)
        self.assertTrue(result["selected"])
        metrics = dict(qc.METRIC_LIMITS)
        metrics["speed_p95_m_s"] += 1e-12
        result = qc.adjudicate(metrics, True, True)
        self.assertFalse(result["selected"])
        self.assertEqual(result["failed_absolute_metrics"], ["speed_p95_m_s"])

    def test_structural_or_initial_state_failure_cannot_pass(self) -> None:
        self.assertFalse(qc.adjudicate(dict(qc.METRIC_LIMITS), False, True)["selected"])
        self.assertFalse(qc.adjudicate(dict(qc.METRIC_LIMITS), True, False)["selected"])

    def test_static_self_check_has_no_execution_or_threshold_delta(self) -> None:
        result = qc.static_self_check()
        self.assertEqual(result["inventory_maximum_bytes"], 402653184)
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
