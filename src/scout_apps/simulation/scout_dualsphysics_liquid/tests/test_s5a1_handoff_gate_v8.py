#!/usr/bin/env python3
"""Static and negative tests for the fresh v3_v6 identity adapter."""

import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_handoff_gate_v8 as gate  # noqa: E402


class HandoffGateV8Tests(unittest.TestCase):
    def test_effective_schema_and_identity(self) -> None:
        policy, _ = gate.expand_policy(); schema, _ = gate.expand_schema()
        self.assertEqual(policy["transfer_id"], gate.TRANSFER_ID)
        self.assertEqual(schema["properties"]["transfer_id"], {"const": gate.TRANSFER_ID})
        gate.v1.assert_schema_deep_closed(schema)

    def test_policy_is_closed_and_rejects_identity_drift(self) -> None:
        original = json.loads(gate.POLICY_PATH.read_bytes())
        for mutate in (
            lambda d: d["base"].update(sha256="0" * 64),
            lambda d: d.update(schema_version="wrong"),
            lambda d: d.update(document_type="wrong"),
            lambda d: d["overrides"].update(planned_final_root="/wrong"),
            lambda d: d.update(extra=True),
        ):
            changed = copy.deepcopy(original); mutate(changed)
            with self.assertRaises(gate.HandoffError):
                gate.expand_policy(gate.v1.canonical_json(changed))

    def test_schema_is_closed_and_rejects_identity_drift(self) -> None:
        original = json.loads(gate.SCHEMA_PATH.read_bytes())
        for mutate in (
            lambda d: d["base"].update(path="/wrong"),
            lambda d: d.update(revision_kind="wrong"),
            lambda d: d["overrides"].update(transfer_id="wrong"),
            lambda d: d.update(extra=True),
        ):
            changed = copy.deepcopy(original); mutate(changed)
            with self.assertRaises(gate.HandoffError):
                gate.expand_schema(gate.v1.canonical_json(changed))

    def test_self_check_is_static(self) -> None:
        result = gate.self_check()
        self.assertEqual(result["transfer_id"], gate.TRANSFER_ID)
        self.assertFalse(result["real_bag_read"]); self.assertFalse(result["files_written"])


if __name__ == "__main__":
    unittest.main()
