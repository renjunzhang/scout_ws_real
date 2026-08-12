#!/usr/bin/env python3
"""Semantic-separation and fail-closed tests for the S6A bridge v2."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SCRIPT = SCRIPTS / "r8_liquid_s6a_selected_signal_v5_to_v4_bridge_v2.py"
SCHEMA = ROOT / "schema/target_host_s6a_selected_signal_v5_to_v4_bridge_v2.json"
TARGET_SCHEMA = ROOT / "schema/target_host_s6_real_selected_signal_input_contract_v4.json"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s6a_selected_signal_v5_to_v4_bridge_v2 as bridge  # noqa: E402


class S6ASelectedSignalV5ToV4BridgeV2Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = bridge.synthetic_v5_fixture()
        self.grid = [index * 0.1 for index in range(16)]

    def encode(self, source=None) -> bytes:
        return bridge.canonical_json(self.source if source is None else source)

    def kwargs(self, raw=None, grid=None) -> dict:
        source_raw = self.encode() if raw is None else raw
        values = self.grid if grid is None else grid
        return {
            "expected_source_v5_sha256": bridge.sha256_bytes(source_raw),
            "expected_output_grid_sha256": bridge.sha256_json(values),
            "expected_s5a0_receipt_sha256": "a" * 64,
            "expected_s5a1_transfer_sha256": "b" * 64,
            "expected_source_bag_sha256": "c" * 64,
            "fixture_only": True,
            "real_input_consumed": False,
            "optional_admitted": False,
        }

    def produce(self, source=None, grid=None, **changes):
        raw = self.encode(source)
        values = self.grid if grid is None else grid
        arguments = self.kwargs(raw, values)
        arguments.update(changes)
        return bridge.bridge_v5_to_v4(raw, values, **arguments)

    @staticmethod
    def refresh_series(source, name):
        samples = source["series"][name]["samples"]
        source["series"][name]["sample_count"] = len(samples)
        source["integrity"][f"{name}_samples_sha256"] = bridge.source_v5.sha256_json(samples)

    def test_current_fixture_separates_schema_from_unknown_outcome(self) -> None:
        self.assertEqual(self.source["provenance"]["source_outcome"], "SPMPC_NON_FIXED")
        result = self.produce()
        analyzer = result["analyzer_input"]
        evidence = analyzer["provenance"]["source_outcome_evidence"]
        self.assertEqual(result["source_contract"]["source_schema"], "SPMPC_NON_FIXED")
        self.assertEqual(analyzer["provenance"]["source_schema"], "SPMPC_NON_FIXED")
        self.assertEqual(evidence["source_outcome"], "UNKNOWN")
        self.assertEqual(evidence["source_terminal"], "UNKNOWN")
        self.assertFalse(evidence["authoritative_sidecar_bound"])
        self.assertIsNone(evidence["authoritative_sidecar_sha256"])
        self.assertNotIn('"source_outcome":"SPMPC_NON_FIXED"', bridge.canonical_json(result).decode())
        self.assertEqual(result["status"], "S6A_V5_TO_V4_SEMANTIC_SPLIT_VALIDATED_NOT_ADMITTED")
        self.assertFalse(result["claims"]["stage6_pass"])
        self.assertFalse(result["target_v4_compatibility"]["legacy_v4_payload_emitted"])
        self.assertEqual(
            analyzer["time_alignment"]["record_to_odom_offset_ns"],
            self.source["time_alignment"]["record_to_odom_header_offset_ns"],
        )
        projection = bridge.validate_target_v4_shape(analyzer)
        Draft202012Validator(json.loads(TARGET_SCHEMA.read_text(encoding="utf-8"))).validate(
            projection
        )

    def test_non_unknown_outcome_requires_exact_authoritative_sidecar_hash(self) -> None:
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "hash-bound authoritative"):
            self.produce(source_outcome="METHOD_SUCCESS", source_terminal="GOAL_REACHED")

        sidecar = bridge.authoritative_sidecar_fixture("METHOD_FAILURE", "GOAL_TIMEOUT")
        raw = bridge.canonical_json(sidecar)
        result = self.produce(
            source_outcome="METHOD_FAILURE",
            source_terminal="GOAL_TIMEOUT",
            source_outcome_sidecar_raw=raw,
            expected_source_outcome_sidecar_sha256=bridge.sha256_bytes(raw),
        )
        evidence = result["outcome_contract"]
        self.assertEqual(evidence["source_outcome"], "METHOD_FAILURE")
        self.assertEqual(evidence["source_terminal"], "GOAL_TIMEOUT")
        self.assertTrue(evidence["authoritative_sidecar_bound"])
        self.assertEqual(evidence["authoritative_sidecar_sha256"], bridge.sha256_bytes(raw))
        self.assertFalse(result["claims"]["stage6_pass"])

        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "sidecar hash drift"):
            self.produce(
                source_outcome="METHOD_FAILURE",
                source_terminal="GOAL_TIMEOUT",
                source_outcome_sidecar_raw=raw,
                expected_source_outcome_sidecar_sha256="0" * 64,
            )
        mismatch = bridge.authoritative_sidecar_fixture("METHOD_SUCCESS", "GOAL_REACHED")
        mismatch_raw = bridge.canonical_json(mismatch)
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "differs from authoritative"):
            self.produce(
                source_outcome="METHOD_FAILURE",
                source_terminal="GOAL_TIMEOUT",
                source_outcome_sidecar_raw=mismatch_raw,
                expected_source_outcome_sidecar_sha256=bridge.sha256_bytes(mismatch_raw),
            )

    def test_schema_as_outcome_and_automatic_success_timeout_are_rejected(self) -> None:
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "schema must not"):
            self.produce(source_outcome="SPMPC_NON_FIXED")
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "terminal, not a source_outcome"):
            self.produce(source_outcome="GOAL_TIMEOUT")
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "cannot infer or claim a terminal"):
            self.produce(source_terminal="GOAL_TIMEOUT")
        for flag in ("infer_outcome_from_topics", "infer_outcome_from_bag_duration"):
            with self.subTest(flag=flag), self.assertRaisesRegex(
                bridge.SelectedSignalBridgeError, "automatic source outcome inference"
            ):
                self.produce(**{flag: True})

    def test_optional_paired_denominator_and_real_input_drift_are_rejected(self) -> None:
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "optional or paired"):
            self.produce(optional_admitted=True)
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "REAL_INPUT_NOT_CONSUMED"):
            self.produce(real_input_consumed=True)
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "REAL_INPUT_NOT_CONSUMED"):
            self.produce(fixture_only=False)

        optional = copy.deepcopy(self.source)
        optional["provenance"]["optional_bag_read"] = True
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "source v5 validation failed"):
            self.produce(optional)
        paired = copy.deepcopy(self.source)
        paired["planned_denominator"] = 2
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "source v5 validation failed"):
            self.produce(paired)

    def test_time_grid_parent_hash_unit_and_nonfinite_drift_are_rejected(self) -> None:
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "document hash drift"):
            self.produce(expected_source_v5_sha256="0" * 64)
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "output grid SHA-256 drift"):
            self.produce(expected_output_grid_sha256="0" * 64)
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "parent identity drift"):
            self.produce(expected_s5a0_receipt_sha256="d" * 64)

        relative = copy.deepcopy(self.source)
        relative["series"]["H_proxy"]["samples"][3]["time_since_odom_origin_s"] += 0.01
        self.refresh_series(relative, "H_proxy")
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "relative time drift"):
            self.produce(relative)
        units = copy.deepcopy(self.source)
        units["series"]["H_modal"]["samples"][2]["value_comparison_mm"] += 1.0
        self.refresh_series(units, "H_modal")
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "unit conversion"):
            self.produce(units)
        nonfinite = copy.deepcopy(self.source)
        nonfinite["series"]["H_modal"]["samples"][1]["value_native"] = math.nan
        raw = json.dumps(nonfinite, allow_nan=True, sort_keys=True, separators=(",", ":")).encode()
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "invalid or non-finite"):
            bridge.bridge_v5_to_v4(raw, self.grid, **self.kwargs(raw, self.grid))

        nonuniform = list(self.grid)
        nonuniform[8] += 0.01
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "not uniform"):
            self.produce(grid=nonuniform)
        outside = [0.1 + index * 0.1 for index in range(16)]
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "extrapolation"):
            self.produce(grid=outside)

    def test_result_schema_semantics_and_integrity_fail_closed(self) -> None:
        result = self.produce()
        changed = copy.deepcopy(result)
        changed["unexpected"] = True
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "schema failure"):
            bridge.validate_bridge_result(changed)

        changed = copy.deepcopy(result)
        changed["analyzer_input"]["provenance"]["source_outcome_evidence"][
            "source_outcome"
        ] = "METHOD_SUCCESS"
        changed["integrity"]["analyzer_input_sha256"] = bridge.sha256_json(
            changed["analyzer_input"]
        )
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "schema failure|self-contradictory"):
            bridge.validate_bridge_result(changed)

        changed = copy.deepcopy(result)
        changed["analyzer_input"]["time_alignment"]["record_to_odom_offset_ns"] += 1
        changed["integrity"]["analyzer_input_sha256"] = bridge.sha256_json(
            changed["analyzer_input"]
        )
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "source time mapping"):
            bridge.validate_bridge_result(changed)

    def test_schema_ast_dependency_hashes_and_self_check(self) -> None:
        schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        bridge.assert_deep_closed(schema)

        def reject_schema_as_outcome(value):
            if isinstance(value, dict):
                self.assertFalse(
                    "source_outcome" in value
                    and isinstance(value["source_outcome"], dict)
                    and value["source_outcome"].get("const") == "SPMPC_NON_FIXED"
                )
                for child in value.values():
                    reject_schema_as_outcome(child)
            elif isinstance(value, list):
                for child in value:
                    reject_schema_as_outcome(child)

        reject_schema_as_outcome(schema)
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports, forbidden = set(), []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call):
                name = node.func.attr if isinstance(node.func, ast.Attribute) else (
                    node.func.id if isinstance(node.func, ast.Name) else None
                )
                if name in {"open", "write_bytes", "write_text", "system", "popen", "run", "Popen"}:
                    forbidden.append(name)
        self.assertFalse(imports & {"os", "rosbag", "rospy", "socket", "requests", "subprocess"})
        self.assertEqual(forbidden, [])
        self.assertEqual(
            hashlib.sha256(bridge.BRIDGE_V1_SCHEMA_PATH.read_bytes()).hexdigest(),
            bridge.BRIDGE_V1_SCHEMA_SHA256,
        )
        self.assertEqual(
            hashlib.sha256(bridge.BRIDGE_V1_SCRIPT_PATH.read_bytes()).hexdigest(),
            bridge.BRIDGE_V1_SCRIPT_SHA256,
        )
        report = bridge.self_check()
        self.assertIn("NOT_ADMITTED", report["status"])
        self.assertEqual(report["source_schema"], "SPMPC_NON_FIXED")
        self.assertEqual(report["source_outcome"], "UNKNOWN")
        self.assertEqual(report["source_terminal"], "UNKNOWN")
        self.assertFalse(report["authoritative_sidecar_bound"])
        self.assertEqual(report["real_input_status"], "REAL_INPUT_NOT_CONSUMED")
        self.assertFalse(report["real_bag_read"])
        self.assertFalse(report["real_bi4_read"])
        self.assertFalse(report["external_write_performed"])
        self.assertFalse(report["stage6_pass"])


if __name__ == "__main__":
    unittest.main()
