#!/usr/bin/env python3
"""Closed-schema and negative tests for the one-shot extended-settling contract."""

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
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_extended_settling_contract_v1.json"
SCHEMA = ROOT / "schema/target_host_u3_stage4_extended_settling_contract_v1.json"


def sha256(path: Path) -> str:
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


class ExtendedSettlingContractV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)

    def test_contract_matches_deep_closed_schema(self) -> None:
        assert_deep_closed(self, self.schema)
        errors = list(Draft202012Validator(self.schema).iter_errors(self.contract))
        self.assertEqual([], [f"{list(item.absolute_path)}: {item.message}" for item in errors])

    def test_all_parent_and_checkpoint_hashes_match(self) -> None:
        for parent in self.contract["parents"].values():
            self.assertEqual(sha256(Path(parent["path"])), parent["sha256"])
        checkpoint = self.contract["checkpoint"]
        for prefix in ("part", "head"):
            path = Path(checkpoint[f"{prefix}_path"])
            metadata = os.lstat(path)
            self.assertTrue(stat.S_ISREG(metadata.st_mode))
            self.assertEqual(metadata.st_nlink, 1)
            self.assertEqual(sha256(path), checkpoint[f"{prefix}_sha256"])
            self.assertEqual(metadata.st_ino, checkpoint[f"{prefix}_inode"])
            self.assertEqual(metadata.st_size, checkpoint[f"{prefix}_size_bytes"])
            self.assertEqual(f"{stat.S_IMODE(metadata.st_mode):04o}", checkpoint[f"{prefix}_mode"])

    def test_only_observation_window_changes(self) -> None:
        solver = self.contract["frozen_solver"]
        self.assertFalse(solver["numerical_parameters_changed_from_probe"])
        self.assertTrue(solver["only_observation_window_extended"])
        self.assertEqual(solver["shifting"], "None")
        self.assertEqual(solver["ddt"], "DDT2(0.1)")
        self.assertEqual(solver["cfl"], 0.2)
        self.assertEqual(solver["dp_m"], 0.002)

    def test_schema_rejects_unknown_or_parameter_drift(self) -> None:
        mutations = (
            lambda value: value.update({"unreviewed": True}),
            lambda value: value["frozen_solver"].update({"cfl": 0.1}),
            lambda value: value["frozen_solver"].update({"shifting": "NoBound"}),
            lambda value: value["extension"].update({"end_time_s": 25.05}),
            lambda value: value["execution_limits"].update({"network": True}),
        )
        for mutate in mutations:
            tampered = copy.deepcopy(self.contract)
            mutate(tampered)
            self.assertTrue(list(Draft202012Validator(self.schema).iter_errors(tampered)))


if __name__ == "__main__":
    unittest.main()
