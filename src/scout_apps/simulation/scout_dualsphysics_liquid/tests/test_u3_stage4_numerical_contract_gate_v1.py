#!/usr/bin/env python3
"""Static and negative tests for the Stage-4 numerical contract gate."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_u3_stage4_numerical_contract_gate_v1.py"
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_numerical_contract_v1.json"
SCHEMA = ROOT / "schema/target_host_u3_stage4_numerical_contract_v1.json"


def load_module():
    spec = importlib.util.spec_from_file_location("u3_stage4_contract_gate_v1", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Stage4NumericalContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.gate = load_module()
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        cls.contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)

    def test_self_check_verifies_real_s4a_evidence(self) -> None:
        result = self.gate.self_check()
        self.assertEqual(result["status"], "PASS_U3_STAGE4_NUMERICAL_CONTRACT_V1_SELF_CHECK")
        self.assertEqual(result["s4a_status"], "FAIL_U3_DDT_RAMP_NUMERICAL_CANDIDATE")
        self.assertFalse(result["solver_executed"])

    def test_contract_schema_is_deep_closed(self) -> None:
        self.gate.assert_deep_closed(self.schema)
        self.assertEqual([], list(Draft202012Validator(self.schema).iter_errors(self.contract)))

    def test_unknown_fields_and_weakened_boundaries_are_rejected(self) -> None:
        mutations = [
            lambda value: value.update({"unreviewed": True}),
            lambda value: value["remediation_probe"].update({"unreviewed": True}),
            lambda value: value["execution_limits"].__setitem__("network", True),
            lambda value: value["result_boundary"].__setitem__("formal", True),
        ]
        validator = Draft202012Validator(self.schema)
        for mutate in mutations:
            tampered = copy.deepcopy(self.contract)
            mutate(tampered)
            self.assertTrue(list(validator.iter_errors(tampered)))

    def test_only_shifting_changes_in_remediation_probe(self) -> None:
        delta = self.contract["remediation_probe"]["single_delta"]
        self.assertEqual(delta["parameter"], "Shifting")
        self.assertEqual(delta["candidate"], "None")
        self.assertFalse(delta["other_numerical_parameters_changed"])
        self.assertEqual(self.contract["remediation_probe"]["window"]["part_count"], 41)

    def test_claim_ceiling_preserves_missing_physical_inputs(self) -> None:
        self.assertTrue(self.contract["result_boundary"]["development_only"])
        self.assertFalse(self.contract["result_boundary"]["formal"])
        self.assertFalse(self.contract["result_boundary"]["physical_primary_eligible"])
        self.assertTrue(self.contract["result_boundary"]["missing_measured_physical_inputs_prevent_formal_stage4_pass"])


if __name__ == "__main__":
    unittest.main()
