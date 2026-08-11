#!/usr/bin/env python3
"""Static and negative tests for the 35.05--36.05 s extension QC."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p2_settle_extension_qc_v1 as qc  # noqa: E402


class SettleExtensionQcTests(unittest.TestCase):
    def test_schema_is_closed_and_valid(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertFalse(schema["additionalProperties"])
        self.assertEqual(schema["$defs"]["run"]["properties"]["frame_count"]["const"], 21)

    def test_all_metrics_at_limits_pass(self) -> None:
        metrics = dict(qc.METRIC_LIMITS)
        result = qc.adjudicate(metrics, True, True)
        self.assertTrue(result["selected"])
        self.assertEqual(result["failed_absolute_metrics"], [])

    def test_single_over_limit_metric_fails_without_relaxation(self) -> None:
        metrics = dict(qc.METRIC_LIMITS)
        metrics["speed_p95_m_s"] *= 1.000001
        result = qc.adjudicate(metrics, True, True)
        self.assertFalse(result["selected"])
        self.assertEqual(result["failed_absolute_metrics"], ["speed_p95_m_s"])

    def test_zero_minimum_has_explicit_null_ratio(self) -> None:
        parts = [
            {"cpart": 701, "time_s": 35.05, "speed": {"rms_m_s": 0.0}},
            {"cpart": 721, "time_s": 36.05, "speed": {"rms_m_s": 0.0001}},
        ]
        value = qc._trajectory(parts)
        self.assertTrue(value["minimum_is_zero"])
        self.assertIsNone(value["final_to_minimum_ratio"])

    def test_contract_is_only_end_time_delta(self) -> None:
        self.assertEqual(qc.runner.EXPECTED_DELTA["parameter"], "SIMULATION_END_TIME_S")
        self.assertFalse(qc.runner.EXPECTED_DELTA["other_numerical_parameters_changed"])
        self.assertEqual(qc.PART_INDICES, tuple(range(701, 722)))

    def test_static_self_check_has_no_execution(self) -> None:
        result = qc.static_self_check()
        self.assertEqual(result["metric_count"], 17)
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
