#!/usr/bin/env python3
"""Static tests for the deterministic v3_v2 package adapter."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_handoff_gate_v4 as gate  # noqa: E402


class HandoffGateV4Tests(unittest.TestCase):
    def test_effective_policy_schema_and_roots_are_v3_v2(self) -> None:
        policy, _ = gate.expand_policy()
        schema, _ = gate.expand_schema()
        self.assertEqual(policy["transfer_id"], gate.TRANSFER_ID)
        self.assertEqual(policy["package"]["planned_partial_root"], gate.PARTIAL_ROOT)
        self.assertEqual(policy["package"]["planned_final_root"], gate.FINAL_ROOT)
        self.assertEqual(schema["properties"]["transfer_id"], {"const": gate.TRANSFER_ID})
        self.assertNotIn("v3_v1", json.dumps({"transfer_id": policy["transfer_id"],
                                               "roots": policy["package"]}))

    def test_policy_delta_rejects_any_unrelated_override(self) -> None:
        value = json.loads(gate.POLICY_PATH.read_bytes())
        value["overrides"]["source"] = "/optional"
        with self.assertRaisesRegex(gate.HandoffError, "override"):
            gate.expand_policy(gate.v1.canonical_json(value))

    def test_adapter_does_not_read_bag_or_write(self) -> None:
        report = gate.self_check()
        self.assertEqual(report["status"], "PASS_S5A1_HANDOFF_GATE_V4_V3_V2_ADAPTER_STATIC_CONTRACT")
        for key in ("real_bag_read", "optional_bag_read", "files_written", "bwrap_run",
                    "solver_executed", "gpu_exposed", "sudo_used", "network_used"):
            self.assertFalse(report[key], key)

    def test_manifest_transfer_id_v3_v1_is_rejected_by_effective_schema(self) -> None:
        schema, _ = gate.expand_schema()
        validator = gate.Draft202012Validator(schema)
        with self.assertRaises(Exception):
            validator.validate({"transfer_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1"})


if __name__ == "__main__":
    unittest.main()
