#!/usr/bin/env python3
"""Static/fail-closed tests for S6 final-delivery execution v2."""

from __future__ import annotations

import copy
import json
import os
import stat
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s6_final_delivery_execution_v2 as execution  # noqa: E402


SHA = "1" * 64


class S6FinalDeliveryExecutionV2Tests(unittest.TestCase):
    def test_policy_schemas_and_exact_s5b0_v4_sixteen_probe_binding(self) -> None:
        policy, schemas = execution.load_contracts()
        self.assertEqual(4, len(schemas))
        for schema in schemas.values():
            Draft202012Validator.check_schema(schema)
            execution.assert_deep_closed(schema)
        source = json.loads(execution.S5B0_POLICY.read_text())["gauge_contract"]
        self.assertEqual(source, policy["gauge_contract"])
        self.assertEqual(16, source["probe_count"])
        self.assertEqual([f"s5b0_p{i:02d}" for i in range(16)], [p["name"] for p in source["probes"]])
        self.assertEqual((0.0145, 0.001, 0.058, 0.001),
                         (source["probe_radius_m"], source["pointdp_m"], source["h0_m"], source["maximum_invalid_ratio"]))

    def test_all_eight_commands_are_closed_and_fail_without_parent(self) -> None:
        admission = execution.admit()
        self.assertEqual(execution.NOT_ADMITTED, admission["status"])
        self.assertIsNone(admission["package_root"])
        self.assertTrue(admission["checks"]["optional_unread"])
        self.assertEqual(execution.COMMANDS, tuple(execution.load_contracts()[0]["execution_contract"]["commands"]))
        for command in execution.COMMANDS[1:]:
            result = execution.static_command(command, admission)
            self.assertEqual(execution.NOT_ADMITTED, result["status"])
            self.assertFalse(result["checks"]["admission_pass"])
            self.assertFalse(result["side_effects"]["optional_bag_read"])
            self.assertFalse(result["side_effects"]["external_write_performed"])

    def test_admitted_pass_branch_rejects_null_identity_or_false_check(self) -> None:
        value = execution.admit()
        value["status"] = execution.ADMITTED
        value["package_root"] = "/exact/package"
        _, schemas = execution.load_contracts()
        with self.assertRaises(ValidationError):
            Draft202012Validator(schemas["admission"]).validate(value)
        identity = {"relative_path": "execution.json", "sha256": SHA, "size_bytes": 1}
        value["identities"] = {key: copy.deepcopy(identity) for key in value["identities"]}
        value["checks"] = {key: True for key in value["checks"]}
        Draft202012Validator(schemas["admission"]).validate(value)
        value["checks"]["no_toctou"] = False
        with self.assertRaises(ValidationError):
            Draft202012Validator(schemas["admission"]).validate(value)

    def gauge_manifest(self) -> dict[str, object]:
        policy = execution.load_contracts()[0]
        return {
            "schema_version": "smpcc-r8-liquid-s5b0-native-gauge-manifest-v1",
            "attempt_id": execution.ATTEMPT,
            "gauge_contract_sha256": policy["s5b0_v4_binding"]["gauge_contract_sha256"],
            "time_grid_sha256": SHA,
            "files": [{"probe_name": f"s5b0_p{i:02d}", "relative_path": f"GaugesSwl_s5b0_p{i:02d}.csv",
                       "sha256": f"{i + 1:064x}", "size_bytes": 1, "time_grid_sha256": SHA} for i in range(16)],
        }

    def test_native_gauge_manifest_rejects_probe_order_grid_and_parent_drift(self) -> None:
        value = self.gauge_manifest()
        execution.validate_native_gauge_manifest(value)
        changed = copy.deepcopy(value)
        changed["files"][0], changed["files"][1] = changed["files"][1], changed["files"][0]
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "order"):
            execution.validate_native_gauge_manifest(changed)
        changed = copy.deepcopy(value)
        changed["files"][8]["time_grid_sha256"] = "2" * 64
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "time grid"):
            execution.validate_native_gauge_manifest(changed)
        changed = copy.deepcopy(value)
        changed["gauge_contract_sha256"] = "2" * 64
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "parent"):
            execution.validate_native_gauge_manifest(changed)

    def media_manifest(self) -> dict[str, object]:
        frames = [{"index": i, "time_s": float(i), "sha256": SHA,
                   "class_counts_sha256": "2" * 64, "probe_overlay_sha256": "3" * 64,
                   "container_frame": "MOVING_CONTAINER_REFERENCE_REF_0"} for i in range(5)]
        return {"schema_version": "smpcc-r8-liquid-s6-media-manifest-v2",
                "frame_manifest_sha256": SHA, "frames": frames,
                "decode_qa": {"mp4_full_decode": True, "gif_full_decode": True, "decoded_frame_count": 5},
                "keyframes": {"first": 0, "middle": 2, "last": 4}}

    def test_media_requires_frame_bindings_full_decode_and_keyframes(self) -> None:
        value = self.media_manifest()
        execution.validate_media_manifest(value)
        changed = copy.deepcopy(value)
        changed["frames"][2]["probe_overlay_sha256"] = None
        with self.assertRaises(execution.S6ExecutionV2Error):
            execution.validate_media_manifest(changed)
        changed = copy.deepcopy(value)
        changed["decode_qa"]["decoded_frame_count"] = 4
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "decode"):
            execution.validate_media_manifest(changed)
        changed = copy.deepcopy(value)
        changed["keyframes"]["middle"] = 1
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "keyframe"):
            execution.validate_media_manifest(changed)

    def test_exact_seventeen_inventory_checksums_and_append_only_ledger(self) -> None:
        inventory = execution.load_contracts()[0]["execution_contract"]["required_inventory"]
        checksums = {name: SHA for name in inventory}
        ledger = [{"entry_sha256": "2" * 64, "previous_entry_sha256": "0" * 64,
                   "attempt_id": execution.ATTEMPT, "stage6_status": execution.FINAL}]
        execution.validate_publication_inventory(inventory, checksums, ledger)
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "inventory"):
            execution.validate_publication_inventory(inventory[:-1], checksums, ledger)
        broken = copy.deepcopy(ledger)
        broken.append({"entry_sha256": "3" * 64, "previous_entry_sha256": "9" * 64,
                       "attempt_id": "duplicate-or-other", "stage6_status": execution.FINAL})
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "chain"):
            execution.validate_publication_inventory(inventory, checksums, broken)
        duplicate = [ledger[0], {**ledger[0], "entry_sha256": "4" * 64, "previous_entry_sha256": "2" * 64}]
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "duplicate"):
            execution.validate_publication_inventory(inventory, checksums, duplicate)

    def test_tree_snapshot_rejects_toctou_hardlink_special_and_unsafe_path(self) -> None:
        regular = stat.S_IFREG | 0o440
        good = {"a.json": (regular, 1, 5, 7)}
        execution.validate_tree_snapshot(good, copy.deepcopy(good))
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "TOCTOU"):
            execution.validate_tree_snapshot(good, {"a.json": (regular, 1, 6, 8)})
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "link"):
            execution.validate_tree_snapshot({"a": (regular, 2, 5, 7)}, {"a": (regular, 2, 5, 7)})
        special = stat.S_IFIFO | 0o440
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "special"):
            execution.validate_tree_snapshot({"a": (special, 1, 0, 7)}, {"a": (special, 1, 0, 7)})
        with self.assertRaisesRegex(execution.S6ExecutionV2Error, "unsafe"):
            execution.validate_tree_snapshot({"../a": (regular, 1, 5, 7)}, {"../a": (regular, 1, 5, 7)})

    def test_final_pass_requires_all_hashes_identities_checks_and_schema(self) -> None:
        gates = {name: {"pass": True, "receipt_sha256": SHA} for name in
                 ("admission", "extract_selected", "analysis", "figure", "media", "publish")}
        identity = {"relative_path": "x", "sha256": SHA, "size_bytes": 1}
        inventory = execution.load_contracts()[0]["execution_contract"]["required_inventory"]
        publication = {"inventory": inventory, "comparison_manifest": identity, "evidence_index": identity,
                       "ledger": identity, "ledger_append_receipt": identity, "checksums": identity,
                       "acceptance_receipt": identity}
        checks = {"all_receipts_schema_valid": True, "all_receipt_hashes_bound": True,
                  "all_gate_passes": True, "inventory_exact": True, "checksums_complete": True,
                  "ledger_append_verified": True, "planned_row_closed_1_of_1": True,
                  "optional_unread": True, "no_ranking": True, "physical_pending": True,
                  "create_new_only": True, "all_delivery_assets_verified": True}
        passed = execution.final_receipt(gates, publication, checks, request_pass=True)
        self.assertEqual(execution.FINAL, passed["status"])
        self.assertTrue(passed["claims"]["stage6_pass"])
        changed = copy.deepcopy(gates)
        changed["media"]["receipt_sha256"] = None
        failed = execution.final_receipt(changed, publication, checks, request_pass=True)
        self.assertEqual(execution.NOT_ADMITTED, failed["status"])
        self.assertFalse(failed["claims"]["stage6_pass"])
        changed_publication = copy.deepcopy(publication)
        changed_publication["ledger_append_receipt"] = None
        failed = execution.final_receipt(gates, changed_publication, checks, request_pass=True)
        self.assertEqual(execution.NOT_ADMITTED, failed["status"])

    def test_self_check_has_no_side_effects_and_all_commands_report_not_admitted(self) -> None:
        report = execution.self_check()
        self.assertEqual("S6_FINAL_DELIVERY_EXECUTION_V2_SELF_CHECK_OK_NOT_ADMITTED", report["status"])
        self.assertEqual(16, report["probe_count"])
        self.assertEqual(17, report["required_inventory_count"])
        self.assertTrue(all(value == execution.NOT_ADMITTED for value in report["commands"].values()))
        self.assertFalse(report["optional_bag_read"])
        self.assertFalse(report["external_write_performed"])


if __name__ == "__main__":
    unittest.main()
