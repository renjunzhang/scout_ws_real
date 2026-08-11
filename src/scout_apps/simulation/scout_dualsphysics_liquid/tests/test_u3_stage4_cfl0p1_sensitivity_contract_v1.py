#!/usr/bin/env python3
"""Closed-schema and source-evidence tests for the CFL=0.1 sensitivity contract."""

from __future__ import annotations

import copy
import hashlib
import json
import os
import stat
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_cfl0p1_sensitivity_contract_v1.json"
SCHEMA = ROOT / "schema/target_host_u3_stage4_cfl0p1_sensitivity_contract_v1.json"


def file_sha(path: Path) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    digest = hashlib.sha256()
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise AssertionError(f"unsafe parent: {path}")
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            digest.update(block)
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def assert_deep_closed(test: unittest.TestCase, value, path: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" or (isinstance(value.get("type"), list) and "object" in value["type"]):
            test.assertIs(value.get("additionalProperties"), False, path)
        for key, item in value.items():
            assert_deep_closed(test, item, f"{path}/{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            assert_deep_closed(test, item, f"{path}/{index}")


class Cfl0p1SensitivityContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)

    def test_contract_matches_deep_closed_schema(self) -> None:
        assert_deep_closed(self, self.schema)
        errors = list(Draft202012Validator(self.schema).iter_errors(self.contract))
        self.assertEqual([], [f"{list(item.absolute_path)}: {item.message}" for item in errors])

    def test_all_parent_and_checkpoint_identities_match(self) -> None:
        for parent in self.contract["parents"].values():
            self.assertEqual(file_sha(Path(parent["path"])), parent["sha256"])
        for prefix in ("part", "head"):
            item = self.contract["checkpoint"]
            path = Path(item[f"{prefix}_path"])
            metadata = os.lstat(path)
            self.assertEqual(file_sha(path), item[f"{prefix}_sha256"])
            self.assertEqual(metadata.st_ino, item[f"{prefix}_inode"])
            self.assertEqual(metadata.st_nlink, 1)

    def test_readonly_source_proves_exact_cfl_override(self) -> None:
        parser_source = Path(self.contract["parents"]["cli_parser_source"]["path"]).read_text(encoding="utf-8")
        solver_source = Path(self.contract["parents"]["solver_override_source"]["path"]).read_text(encoding="utf-8")
        self.assertIn('printf("    -cfl:<float>', parser_source)
        self.assertIn("CFLnumber=atof(txoptfull.c_str())", parser_source)
        self.assertIn("if(cfg->CFLnumber>0)CFLnumber=cfg->CFLnumber", solver_source)
        self.assertFalse(self.contract["cli_override_evidence"]["candidate_help_executed"])

    def test_only_cfl_changes(self) -> None:
        window = self.contract["paired_window"]
        solver = self.contract["frozen_solver"]
        self.assertEqual((window["baseline_cfl"], window["candidate_cfl"]), (0.2, 0.1))
        self.assertEqual(window["single_delta"], "CFL_0P2_TO_0P1")
        self.assertFalse(solver["other_numerical_parameters_changed"])
        self.assertEqual(solver["shifting"], "None")

    def test_schema_rejects_scope_or_safety_drift(self) -> None:
        mutations = (
            lambda value: value.update({"unreviewed": True}),
            lambda value: value["paired_window"].update({"candidate_cfl": 0.05}),
            lambda value: value["frozen_solver"].update({"shifting": "NoBound"}),
            lambda value: value["execution_limits"].update({"network": True}),
        )
        for mutate in mutations:
            tampered = copy.deepcopy(self.contract)
            mutate(tampered)
            self.assertTrue(list(Draft202012Validator(self.schema).iter_errors(tampered)))


if __name__ == "__main__":
    unittest.main()
