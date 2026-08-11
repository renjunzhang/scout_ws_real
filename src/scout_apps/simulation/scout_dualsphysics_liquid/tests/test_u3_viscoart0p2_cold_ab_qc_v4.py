#!/usr/bin/env python3
"""Regression tests for the frozen-output-bound cold QC v4."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p2_cold_ab_qc_v4 as qc  # noqa: E402


class ColdAbQcV4Tests(unittest.TestCase):
    def test_schema_is_closed_and_valid(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertFalse(schema["additionalProperties"])
        self.assertEqual(
            schema["properties"]["schema_version"]["const"],
            "r8-liquid-u3-viscoart0p2-cold-ab-qc-v4",
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
        self.assertTrue(list(Draft202012Validator(bound).iter_errors(qc.INVENTORY_MAXIMUM_BYTES + 1)))

    def test_only_inventory_maximum_changed(self) -> None:
        self.assertTrue(qc._schemas_differ_only_by_inventory_maximum())

    def test_static_self_check_has_no_execution_or_threshold_delta(self) -> None:
        result = qc.static_self_check()
        self.assertEqual(result["inventory_maximum_bytes"], 402653184)
        self.assertFalse(result["threshold_changed"])
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
