#!/usr/bin/env python3
"""Static and negative tests for the v7 viscosity-0.2 probe QC."""

from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p2_probe_qc_v1 as qc  # noqa: E402


class ViscoartProbeQcTests(unittest.TestCase):
    def test_schema_is_valid_and_closed(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertFalse(schema["additionalProperties"])
        for definition in schema["$defs"].values():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False)

    def test_single_delta_and_threshold_vector_are_exact(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        delta = schema["$defs"]["singleDelta"]["properties"]
        self.assertEqual(delta["parameter"]["const"], "ARTIFICIAL_VISCOSITY_COEFFICIENT")
        self.assertEqual(delta["baseline"]["const"], "0.1")
        self.assertEqual(delta["candidate"]["const"], "0.2")
        self.assertEqual(len(qc.METRIC_LIMITS), 17)

    def test_all_17_metrics_at_limits_pass(self) -> None:
        result = qc.adjudicate(dict(qc.METRIC_LIMITS), True, True)
        self.assertTrue(result["selected"])
        self.assertEqual(result["failed_absolute_metrics"], [])
        self.assertTrue(all(result["metric_absolute_pass"].values()))

    def test_one_p95_exceedance_fails(self) -> None:
        metrics = dict(qc.METRIC_LIMITS)
        metrics["speed_p95_m_s"] = math.nextafter(metrics["speed_p95_m_s"], math.inf)
        result = qc.adjudicate(metrics, True, True)
        self.assertFalse(result["selected"])
        self.assertEqual(result["failed_absolute_metrics"], ["speed_p95_m_s"])

    def test_structural_or_restart_failure_cannot_pass(self) -> None:
        self.assertFalse(qc.adjudicate(dict(qc.METRIC_LIMITS), False, True)["selected"])
        self.assertFalse(qc.adjudicate(dict(qc.METRIC_LIMITS), True, False)["selected"])

    def test_missing_or_nonfinite_metric_rejected(self) -> None:
        missing = dict(qc.METRIC_LIMITS)
        missing.pop("speed_rms_m_s")
        with self.assertRaises(qc.ViscoartProbeQcError):
            qc.adjudicate(missing, True, True)
        nonfinite = dict(qc.METRIC_LIMITS)
        nonfinite["speed_rms_m_s"] = float("nan")
        with self.assertRaises(qc.ViscoartProbeQcError):
            qc.adjudicate(nonfinite, True, True)

    def test_static_self_check_never_executes_solver(self) -> None:
        result = qc.static_self_check()
        self.assertEqual(result["metric_count"], 17)
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
