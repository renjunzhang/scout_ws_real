#!/usr/bin/env python3
"""Static and negative tests for the fail-closed v3_v4 adapter."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_handoff_gate_v6 as gate  # noqa: E402


class HandoffGateV6Tests(unittest.TestCase):
    def test_effective_identity_and_deep_closed_schema(self) -> None:
        policy, _ = gate.expand_policy()
        schema, _ = gate.expand_schema()
        self.assertEqual(policy["transfer_id"], gate.TRANSFER_ID)
        self.assertEqual(schema["properties"]["transfer_id"], {"const": gate.TRANSFER_ID})
        gate.v1.assert_schema_deep_closed(schema)

    def test_policy_delta_rejects_every_non_override_drift(self) -> None:
        original = json.loads(gate.POLICY_PATH.read_bytes())
        mutations = []
        changed = copy.deepcopy(original); changed["base"]["sha256"] = "0" * 64; mutations.append(changed)
        changed = copy.deepcopy(original); changed["schema_version"] = "wrong"; mutations.append(changed)
        changed = copy.deepcopy(original); changed["document_type"] = "wrong"; mutations.append(changed)
        changed = copy.deepcopy(original); changed["extra"] = True; mutations.append(changed)
        for value in mutations:
            with self.subTest(value=value), self.assertRaises(gate.HandoffError):
                gate.expand_policy(gate.v1.canonical_json(value))

    def test_schema_delta_rejects_base_kind_and_extra_drift(self) -> None:
        original = json.loads(gate.SCHEMA_PATH.read_bytes())
        mutations = []
        changed = copy.deepcopy(original); changed["base"]["path"] = "/wrong"; mutations.append(changed)
        changed = copy.deepcopy(original); changed["revision_kind"] = "wrong"; mutations.append(changed)
        changed = copy.deepcopy(original); changed["extra"] = True; mutations.append(changed)
        for value in mutations:
            with self.subTest(value=value), self.assertRaises(gate.HandoffError):
                gate.expand_schema(gate.v1.canonical_json(value))

    def test_self_check_is_static(self) -> None:
        report = gate.self_check()
        self.assertEqual(report["status"],
                         "PASS_S5A1_HANDOFF_GATE_V6_V3_V4_ADAPTER_STATIC_CONTRACT")
        self.assertFalse(report["real_bag_read"])
        self.assertFalse(report["files_written"])


if __name__ == "__main__":
    unittest.main()
