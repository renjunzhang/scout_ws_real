#!/usr/bin/env python3
"""Regression tests for the v2 tail-window representation correction."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_viscoart0p1_extension_qc_v2 as qc  # noqa: E402


class ExtensionQcV2Tests(unittest.TestCase):
    def test_schema_is_closed_and_valid(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertFalse(schema["additionalProperties"])
        self.assertEqual(schema["properties"]["schema_version"]["const"], "r8-liquid-u3-viscoart0p1-extension-qc-v2")

    def test_real_tail_start_is_admitted_but_excess_is_rejected(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))["$defs"]["tail"]
        probe = {"part_count": 20, "start_time_s": 29.100009555084092, "end_time_s": 30.050009, "coverage_s": 0.95, "surface_valid_part_count": 20}
        self.assertEqual(list(Draft202012Validator(schema).iter_errors(probe)), [])
        probe["start_time_s"] = 29.12
        self.assertTrue(list(Draft202012Validator(schema).iter_errors(probe)))

    def test_only_schema_tail_interval_changed(self) -> None:
        v1 = json.loads((qc.PACKAGE_ROOT / "schema/target_host_u3_viscoart0p1_extension_qc_v1.json").read_text(encoding="utf-8"))
        v2 = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        for document in (v1, v2):
            document.pop("$id")
            document["properties"]["schema_version"].pop("const")
            document["properties"]["document_type"].pop("const")
        v1["$defs"]["tail"]["properties"]["start_time_s"] = v2["$defs"]["tail"]["properties"]["start_time_s"]
        self.assertEqual(v1, v2)

    def test_static_self_check_has_no_execution(self) -> None:
        result = qc.static_self_check()
        self.assertFalse(result["threshold_changed"])
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])


if __name__ == "__main__":
    unittest.main()
