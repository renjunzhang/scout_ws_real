#!/usr/bin/env python3
"""Positive and fail-closed tests for the S6A v5-to-v4 fixture bridge."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SCRIPT = SCRIPTS / "r8_liquid_s6a_selected_signal_v5_to_v4_bridge_v1.py"
SCHEMA = ROOT / "schema/target_host_s6a_selected_signal_v5_to_v4_bridge_v1.json"
TARGET_SCHEMA = ROOT / "schema/target_host_s6_real_selected_signal_input_contract_v4.json"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s6a_selected_signal_v5_to_v4_bridge_v1 as bridge  # noqa: E402


class S6ASelectedSignalV5ToV4BridgeV1Tests(unittest.TestCase):
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

    def test_positive_exact_mapping_grid_parents_and_v4_schema(self) -> None:
        result = self.produce()
        target = result["analyzer_input"]
        Draft202012Validator(json.loads(TARGET_SCHEMA.read_text(encoding="utf-8"))).validate(target)
        self.assertEqual(result["status"], "S6A_V5_TO_V4_BRIDGE_VALIDATED_NOT_ADMITTED")
        self.assertEqual(result["mode"], "FIXTURE_ONLY_REAL_INPUT_NOT_CONSUMED")
        self.assertFalse(result["real_input_consumed"])
        self.assertEqual(
            target["time_alignment"]["record_to_odom_offset_ns"],
            self.source["time_alignment"]["record_to_odom_header_offset_ns"],
        )
        self.assertEqual(target["time_alignment"]["output_grid_sha256"], bridge.sha256_json(self.grid))
        self.assertEqual(target["time_alignment"]["overlap_start_s"], self.grid[0])
        self.assertEqual(target["time_alignment"]["overlap_end_s"], self.grid[-1])
        self.assertEqual(target["parents"]["s5a0_selected_bag_receipt"]["sha256"], "a" * 64)
        self.assertEqual(target["parents"]["s5a1_transfer_manifest"]["sha256"], "b" * 64)
        self.assertEqual(target["parents"]["source_bag"]["sha256"], "c" * 64)
        self.assertEqual(set(target["series"]["H_proxy"]["samples"][0]),
                         {"bag_record_t_ns", "value_native"})
        self.assertFalse(target["claims"]["stage6_pass"])

    def test_optional_denominator_unknown_and_real_input_are_rejected(self) -> None:
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "optional"):
            self.produce(optional_admitted=True)
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "REAL_INPUT_NOT_CONSUMED"):
            self.produce(real_input_consumed=True)
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "REAL_INPUT_NOT_CONSUMED"):
            self.produce(fixture_only=False)

        mutations = []
        optional = copy.deepcopy(self.source)
        optional["provenance"]["optional_bag_read"] = True
        mutations.append(optional)
        denominator = copy.deepcopy(self.source)
        denominator["planned_denominator"] = 2
        mutations.append(denominator)
        unknown = copy.deepcopy(self.source)
        unknown["unknown"] = True
        mutations.append(unknown)
        for changed in mutations:
            with self.subTest(keys=set(changed)), self.assertRaisesRegex(
                bridge.SelectedSignalBridgeError, "source v5 validation failed"
            ):
                self.produce(changed)

    def test_source_grid_and_parent_hash_drift_are_rejected(self) -> None:
        raw = self.encode()
        arguments = self.kwargs(raw, self.grid)
        arguments["expected_source_v5_sha256"] = "0" * 64
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "document hash drift"):
            bridge.bridge_v5_to_v4(raw, self.grid, **arguments)

        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "output grid SHA-256 drift"):
            self.produce(expected_output_grid_sha256="0" * 64)
        for name, value in (
            ("expected_s5a0_receipt_sha256", "d" * 64),
            ("expected_s5a1_transfer_sha256", "d" * 64),
            ("expected_source_bag_sha256", "d" * 64),
        ):
            with self.subTest(name=name), self.assertRaisesRegex(
                bridge.SelectedSignalBridgeError, "parent identity drift"
            ):
                self.produce(**{name: value})

    def test_time_unit_nan_short_series_and_grid_drift_are_rejected(self) -> None:
        relative = copy.deepcopy(self.source)
        relative["series"]["H_proxy"]["samples"][3]["time_since_odom_origin_s"] += 0.01
        self.refresh_series(relative, "H_proxy")
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "relative time drift"):
            self.produce(relative)

        units = copy.deepcopy(self.source)
        units["series"]["H_proxy"]["samples"][2]["value_comparison_mm"] += 1.0
        self.refresh_series(units, "H_proxy")
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "unit conversion"):
            self.produce(units)

        nonfinite = copy.deepcopy(self.source)
        nonfinite["series"]["H_modal"]["samples"][1]["value_native"] = math.nan
        raw = json.dumps(nonfinite, allow_nan=True, sort_keys=True, separators=(",", ":")).encode()
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "invalid or non-finite"):
            bridge.bridge_v5_to_v4(raw, self.grid, **self.kwargs(raw, self.grid))

        short = copy.deepcopy(self.source)
        for name in ("H_proxy", "H_modal"):
            short["series"][name]["samples"] = short["series"][name]["samples"][:15]
            self.refresh_series(short, name)
        short["time_alignment"]["overlap_end_s"] = 1.4
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "fewer than 16"):
            self.produce(short, grid=self.grid[:15])

        nonuniform = list(self.grid)
        nonuniform[8] += 0.01
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "not uniform"):
            self.produce(grid=nonuniform)
        outside = [0.1 + index * 0.1 for index in range(16)]
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "extrapolation"):
            self.produce(grid=outside)

    def test_result_schema_integrity_and_extra_field_fail_closed(self) -> None:
        result = self.produce()
        changed = copy.deepcopy(result)
        changed["unexpected"] = True
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "schema failure"):
            bridge.validate_bridge_result(changed)
        changed = copy.deepcopy(result)
        changed["analyzer_input"]["time_alignment"]["record_to_odom_offset_ns"] += 1
        changed["integrity"]["analyzer_input_sha256"] = bridge.sha256_json(changed["analyzer_input"])
        with self.assertRaisesRegex(bridge.SelectedSignalBridgeError, "source time mapping"):
            bridge.validate_bridge_result(changed)

    def test_schema_ast_and_self_check_are_closed_fixture_only(self) -> None:
        schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        bridge.assert_deep_closed(schema)
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = set()
        forbidden = []
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
        report = bridge.self_check()
        self.assertIn("NOT_ADMITTED", report["status"])
        self.assertEqual(report["real_input_status"], "REAL_INPUT_NOT_CONSUMED")
        self.assertFalse(report["real_bag_read"])
        self.assertFalse(report["real_bi4_read"])
        self.assertFalse(report["external_write_performed"])
        self.assertFalse(report["stage6_pass"])


if __name__ == "__main__":
    unittest.main()
