#!/usr/bin/env python3
"""Static and negative tests for the fresh v3_v7 identity adapter."""

import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_handoff_gate_v9 as gate  # noqa: E402


class HandoffGateV9Tests(unittest.TestCase):
    def test_effective_schema_and_identity(self):
        policy, _ = gate.expand_policy(); schema, _ = gate.expand_schema()
        self.assertEqual(policy["transfer_id"], gate.TRANSFER_ID)
        self.assertEqual(schema["properties"]["transfer_id"], {"const": gate.TRANSFER_ID})
        gate.v1.assert_schema_deep_closed(schema)

    def test_deltas_are_closed(self):
        cases = ((gate.POLICY_PATH, gate.expand_policy), (gate.SCHEMA_PATH, gate.expand_schema))
        for path, expand in cases:
            original = json.loads(path.read_bytes())
            for mutate in (lambda d: d.update(extra=True), lambda d: d["base"].update(sha256="0"*64),
                           lambda d: d["overrides"].update(transfer_id="wrong")):
                changed=copy.deepcopy(original);mutate(changed)
                with self.assertRaises(gate.HandoffError): expand(gate.v1.canonical_json(changed))

    def test_self_check_is_static(self):
        result=gate.self_check();self.assertFalse(result["real_bag_read"]);self.assertFalse(result["files_written"])


if __name__ == "__main__": unittest.main()
