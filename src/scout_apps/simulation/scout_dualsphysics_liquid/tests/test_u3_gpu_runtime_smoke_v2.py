#!/usr/bin/env python3
"""Static and negative tests for the FD-pinned GPU smoke v2 revision."""

from __future__ import annotations

import copy
import importlib.util
import json
import unittest
from pathlib import Path

import jsonschema


ROOT = Path(__file__).resolve().parent.parent
RUNNER_PATH = ROOT / "scripts/r8_liquid_u3_gpu_runtime_smoke_v2.py"
REVISION_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v2.json"
SCHEMA_PATH = ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v2.json"


def load_runner():
    spec = importlib.util.spec_from_file_location("gpu_smoke_v2", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuRuntimeSmokeV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()
        cls.revision = json.loads(REVISION_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
        cls.validator = jsonschema.Draft202012Validator(cls.schema)

    def test_closed_revision_and_effective_policy_pass(self) -> None:
        jsonschema.Draft202012Validator.check_schema(self.schema)
        self.validator.validate(self.revision)
        policy, _ = self.runner.load_contract()
        self.runner.base.static_semantic_check(policy)

    def test_revision_rejects_unknown_field(self) -> None:
        revision = copy.deepcopy(self.revision)
        revision["unknown"] = True
        with self.assertRaises(jsonschema.ValidationError):
            self.validator.validate(revision)

    def test_delta_rejects_any_new_device(self) -> None:
        revision = copy.deepcopy(self.revision)
        revision["semantic_delta"]["new_devices"] = 1
        with self.assertRaises(jsonschema.ValidationError):
            self.validator.validate(revision)

    def test_delta_rejects_any_new_write_target(self) -> None:
        revision = copy.deepcopy(self.revision)
        revision["semantic_delta"]["new_write_targets"] = 1
        with self.assertRaises(jsonschema.ValidationError):
            self.validator.validate(revision)

    def test_delta_rejects_solver_argv_change(self) -> None:
        revision = copy.deepcopy(self.revision)
        revision["semantic_delta"]["solver_argv_changed"] = True
        with self.assertRaises(jsonschema.ValidationError):
            self.validator.validate(revision)

    def test_run_uses_only_four_read_fds_and_one_output_fd(self) -> None:
        policy, _ = self.runner.load_contract()
        argv = self.runner.bwrap_argv(
            policy, command=policy["solver"]["argv"], include_run=True
        )
        self.assertEqual(len(self.runner._pairs(argv, "--ro-bind-fd")), 4)
        self.assertEqual(len(self.runner._pairs(argv, "--bind-fd")), 1)
        self.assertEqual(self.runner._pairs(argv, "--bind"), [])
        self.assertEqual(self.runner.ACTIVE_FDS, [])

    def test_fd_guest_destinations_are_exact(self) -> None:
        policy, _ = self.runner.load_contract()
        argv = self.runner.bwrap_argv(
            policy, command=policy["solver"]["argv"], include_run=True
        )
        self.assertEqual(
            [pair[1] for pair in self.runner._pairs(argv, "--ro-bind-fd")],
            [
                "/runtime/DualSPHysics5.4_linux64",
                "/runtime/DsphConfig.xml",
                "/case/C1M_zero.bi4",
                "/case/C1M_zero.xml",
            ],
        )
        self.assertEqual(self.runner._pairs(argv, "--bind-fd")[0][1], "/output")

    def test_base_bytes_are_hash_pinned(self) -> None:
        for parent in self.revision["parents"].values():
            self.assertEqual(
                self.runner.base.sha256_file(Path(parent["path"])), parent["sha256"]
            )


if __name__ == "__main__":
    unittest.main()
