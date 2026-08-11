#!/usr/bin/env python3
"""Regression tests for the v10 floating-tail representation correction."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p2_settle_extension_qc_v3 as qc  # noqa: E402


class SettleExtensionQcV3Tests(unittest.TestCase):
    def test_schema_is_closed_and_valid(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text())
        Draft202012Validator.check_schema(schema)
        self.assertFalse(schema["additionalProperties"])

    def test_observed_coverage_is_admitted_but_excess_relaxation_rejected(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text())
        coverage = schema["$defs"]["tail"]["properties"]["coverage_s"]
        validator = Draft202012Validator(coverage)
        self.assertEqual(list(validator.iter_errors(0.9499972596497273)), [])
        self.assertTrue(list(validator.iter_errors(0.9498)))

    def test_only_coverage_minimum_changed(self) -> None:
        self.assertTrue(qc._schemas_differ_only_by_coverage_minimum())

    def test_static_self_check_does_not_change_thresholds_or_execute(self) -> None:
        result = qc.static_self_check()
        self.assertFalse(result["threshold_changed"])
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
