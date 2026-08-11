#!/usr/bin/env python3
"""Static/negative tests for the root-owned /run GPU smoke transport v3."""

from __future__ import annotations

import copy
import importlib.util
import json
import unittest
from pathlib import Path

import jsonschema


ROOT = Path(__file__).resolve().parent.parent
RUNNER_PATH = ROOT / "scripts/r8_liquid_u3_gpu_runtime_smoke_v3.py"
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v3.json"
SCHEMA_PATH = ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v3.json"


def load_runner():
    spec = importlib.util.spec_from_file_location("gpu_smoke_v3", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuRuntimeSmokeV3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()
        cls.revision = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
        cls.validator = jsonschema.Draft202012Validator(cls.schema)

    def test_closed_revision_and_effective_policy_pass(self) -> None:
        jsonschema.Draft202012Validator.check_schema(self.schema)
        self.validator.validate(self.revision)
        policy, _ = self.runner.load_contract()
        self.runner.base.static_semantic_check(policy)

    def test_parent_bytes_are_pinned(self) -> None:
        for parent in self.revision["parents"].values():
            self.assertEqual(self.runner.base.sha256_file(Path(parent["path"])), parent["sha256"])

    def test_no_home_permission_change_or_new_project_write(self) -> None:
        self.assertFalse(self.revision["semantic_delta"]["home_permissions_changed"])
        self.assertEqual(self.revision["semantic_delta"]["new_project_write_targets"], 0)

    def test_delta_rejects_network_or_device_expansion(self) -> None:
        for key, value in (("new_network", True), ("new_devices", 1)):
            with self.subTest(key=key):
                revision = copy.deepcopy(self.revision)
                revision["semantic_delta"][key] = value
                with self.assertRaises(jsonschema.ValidationError):
                    self.validator.validate(revision)

    def test_run_binds_only_run_stage_output_writable(self) -> None:
        policy, _ = self.runner.load_contract()
        argv = self.runner.bwrap_argv(policy, command=policy["solver"]["argv"], include_run=True)
        self.assertEqual(
            self.runner.pairs(argv, "--bind"),
            [[str(self.runner.MAIN_STAGE_ROOT / "output"), "/output"]],
        )
        self.assertFalse(any("/home/zrj/scout_ws" in item for pair in self.runner.pairs(argv, "--bind") for item in pair))

    def test_run_read_binds_are_exact(self) -> None:
        policy, _ = self.runner.load_contract()
        argv = self.runner.bwrap_argv(policy, command=policy["solver"]["argv"], include_run=True)
        self.assertEqual(
            self.runner.pairs(argv, "--ro-bind")[-2:],
            [
                [str(self.runner.MAIN_STAGE_ROOT / "runtime"), "/runtime"],
                [str(self.runner.MAIN_STAGE_ROOT / "case"), "/case"],
            ],
        )

    def test_solver_command_and_isolation_unchanged(self) -> None:
        policy, _ = self.runner.load_contract()
        argv = self.runner.bwrap_argv(policy, command=policy["solver"]["argv"], include_run=True)
        self.assertEqual(argv[-len(policy["solver"]["argv"]):], policy["solver"]["argv"])
        self.assertIn("--unshare-net", argv)
        self.assertEqual(argv[argv.index("--cap-drop") + 1], "ALL")
        self.assertEqual(self.runner.pairs(argv, "--dev-bind"), [[path, path] for path in policy["gpu"]["device_paths"]])

    def test_stage_candidate_postcondition_is_disarmed(self) -> None:
        self.assertEqual(self.revision["transport"]["candidate_stage_final_mode"], "0400")
        self.assertTrue(self.revision["transport"]["preserve_stage"])


if __name__ == "__main__":
    unittest.main()
