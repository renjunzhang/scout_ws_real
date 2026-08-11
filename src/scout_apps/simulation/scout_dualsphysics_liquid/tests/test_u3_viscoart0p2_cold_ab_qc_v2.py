#!/usr/bin/env python3
"""Regression and negative tests for the branch-aware cold-A/B QC v2."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p2_cold_ab_qc_v2 as qc  # noqa: E402


class ColdAbQcV2Tests(unittest.TestCase):
    def test_schema_is_closed_and_valid(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertFalse(schema["additionalProperties"])
        self.assertEqual(
            schema["properties"]["schema_version"]["const"],
            "r8-liquid-u3-viscoart0p2-cold-ab-qc-v2",
        )

    def test_role_specific_run_ids_are_exact(self) -> None:
        self.assertEqual(qc._expected_run_id(qc.runner.POLICY_PATHS["a"]), qc.runner.RUN_IDS["a"])
        self.assertEqual(qc._expected_run_id(qc.runner.POLICY_PATHS["b"]), qc.runner.RUN_IDS["b"])
        self.assertNotEqual(qc.runner.RUN_IDS["a"], qc.runner.RUN_IDS["b"])

    def test_unknown_or_alias_policy_is_rejected(self) -> None:
        with self.assertRaises(qc.ColdAbQcV2Error):
            qc._expected_run_id(Path("/tmp/not-a-frozen-policy.json"))

    def test_only_schema_revision_identity_changed(self) -> None:
        self.assertTrue(qc._schemas_differ_only_by_revision_identity())

    def test_static_self_check_has_no_execution_or_threshold_delta(self) -> None:
        result = qc.static_self_check()
        self.assertEqual(result["single_representation_delta"], "RUN_ID_TO_RUN_IDS_BY_FROZEN_ROLE")
        self.assertFalse(result["threshold_changed"])
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
