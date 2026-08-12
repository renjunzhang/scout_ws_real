#!/usr/bin/env python3
"""Static and negative tests for the Stage-5 C1 compatibility gate v1."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_s5_c1_compatibility_gate_v1.py"
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5_c1_compatibility_v1.json"
SCHEMA = ROOT / "schema/target_host_s5_c1_compatibility_v1.json"


def load_module():
    spec = importlib.util.spec_from_file_location("s5_c1_compatibility_gate_v1", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class S5C1CompatibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gate = load_module()
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        cls.contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)

    def test_real_parent_self_check_passes_without_runtime_actions(self) -> None:
        result = self.gate.self_check()
        self.assertEqual(result["status"], "PASS_S5_C1_TO_C1M_DEVELOPMENT_REPLAY_COMPATIBILITY_V1")
        self.assertEqual(result["verified_parent_count"], 14)
        self.assertEqual(result["particle_evidence"]["particle_ids"], 9078)
        self.assertFalse(result["gpu_replay_authorized"])
        self.assertFalse(result["solver_executed"])

    def test_schema_is_deep_closed_and_contract_validates(self) -> None:
        self.gate.assert_deep_closed(self.schema)
        self.assertEqual([], list(Draft202012Validator(self.schema).iter_errors(self.contract)))

    def test_unknown_fields_and_claim_promotion_are_rejected(self) -> None:
        validator = Draft202012Validator(self.schema)
        mutations = [
            lambda value: value.update({"unreviewed": True}),
            lambda value: value["mount_contract"].update({"unreviewed": True}),
            lambda value: value["claim_ceiling"].__setitem__("formal", True),
            lambda value: value["claim_ceiling"].__setitem__("gpu_replay_authorized", True),
            lambda value: value["source_scope"].__setitem__("source_outcome", "METHOD_SUCCESS"),
            lambda value: value["source_scope"].__setitem__("c2_read_or_admitted", True),
        ]
        for mutate in mutations:
            tampered = copy.deepcopy(self.contract)
            mutate(tampered)
            self.assertTrue(list(validator.iter_errors(tampered)))

    def test_mount_is_explicitly_development_only_identity(self) -> None:
        mount = self.contract["mount_contract"]
        self.assertEqual(mount["parent_frame"], "base_footprint")
        self.assertEqual(mount["translation_m"], [0.0, 0.0, 0.0])
        self.assertEqual(mount["quaternion_xyzw"], [0.0, 0.0, 0.0, 1.0])
        self.assertTrue(mount["development_assumption"])
        self.assertFalse(mount["exact_r7_attempt_sidecar_proof"])
        self.assertFalse(mount["physical_mount_validated"])

    def test_exact_replay_geometry_and_settled_particle_set_are_bound(self) -> None:
        replay = self.contract["replay_c1m"]
        self.assertEqual(replay["geometry_source"], "EXACT_HASHED_C1M_GENCASE_V8")
        self.assertEqual(replay["particle_count"], 9078)
        self.assertEqual(replay["moving_boundary_count"], 2669)
        self.assertEqual(replay["fluid_count"], 6409)
        self.assertEqual(replay["settled_part_index"], 901)
        self.assertFalse(self.contract["compatibility_checks"]["new_exact_settle_required"])
        self.assertFalse(self.contract["compatibility_checks"]["physical_geometry_compatibility_claimed"])


if __name__ == "__main__":
    unittest.main()
