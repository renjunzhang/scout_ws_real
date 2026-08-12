#!/usr/bin/env python3
"""Static and negative tests for the fresh v3_v9 identity adapter."""

import copy
import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_handoff_gate_v11 as gate


class HandoffGateV11Tests(unittest.TestCase):
    def test_effective_policy_and_schema_are_fresh_and_closed(self):
        policy, _ = gate.expand_policy()
        schema, _ = gate.expand_schema()
        Draft202012Validator.check_schema(schema)
        gate.v1.assert_schema_deep_closed(schema)
        self.assertEqual(policy["transfer_id"], gate.TRANSFER_ID)
        self.assertEqual(schema["properties"]["transfer_id"], {"const": gate.TRANSFER_ID})
        self.assertEqual(policy["package"]["planned_partial_root"], gate.PARTIAL_ROOT)
        self.assertEqual(policy["package"]["planned_final_root"], gate.FINAL_ROOT)

    def test_policy_delta_rejects_extra_identity_or_parent_drift(self):
        value = json.loads(gate.POLICY_PATH.read_bytes())
        for mutate in (
            lambda item: item.update(extra=True),
            lambda item: item["overrides"].update(transfer_id="wrong"),
            lambda item: item["base"].update(sha256="0" * 64),
        ):
            changed = copy.deepcopy(value)
            mutate(changed)
            with self.assertRaises(gate.HandoffError):
                gate.expand_policy(json.dumps(changed).encode())

    def test_schema_delta_rejects_extra_identity_or_parent_drift(self):
        value = json.loads(gate.SCHEMA_PATH.read_bytes())
        for mutate in (
            lambda item: item.update(extra=True),
            lambda item: item["overrides"].update(transfer_id="wrong"),
            lambda item: item["base"].update(path="/tmp/wrong"),
        ):
            changed = copy.deepcopy(value)
            mutate(changed)
            with self.assertRaises(gate.HandoffError):
                gate.expand_schema(json.dumps(changed).encode())

    def test_self_check_is_static(self):
        report = gate.self_check()
        self.assertEqual(report["status"], "PASS_S5A1_HANDOFF_GATE_V11_V3_V9_ADAPTER_STATIC_CONTRACT")
        self.assertFalse(report["real_bag_read"])
        self.assertFalse(report["files_written"])


if __name__ == "__main__":
    unittest.main()
