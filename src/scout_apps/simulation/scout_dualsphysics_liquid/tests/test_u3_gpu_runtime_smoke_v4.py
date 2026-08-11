#!/usr/bin/env python3
"""Static and negative tests for the V4 staged-input DAC remediation."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import jsonschema


ROOT = Path(__file__).resolve().parent.parent
RUNNER_PATH = ROOT / "scripts/r8_liquid_u3_gpu_runtime_smoke_v4.py"
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v4.json"
SCHEMA_PATH = ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v4.json"


def load_runner():
    spec = importlib.util.spec_from_file_location("gpu_smoke_v4", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuRuntimeSmokeV4Tests(unittest.TestCase):
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

    def test_all_v3_evidence_parents_are_byte_pinned(self) -> None:
        for parent in self.revision["parents"].values():
            self.assertEqual(self.runner.base.sha256_file(Path(parent["path"])), parent["sha256"])

    def test_diagnosis_is_supported_by_preserved_v3_evidence(self) -> None:
        failure = json.loads(Path(self.revision["parents"]["v3_failure"]["path"]).read_text(encoding="utf-8"))
        stderr = Path(failure["run_id"].replace("u3_c1m_gpu_smoke_v3_20260810T184500Z", "/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v3_20260810T184500Z.stderr.log"))
        self.assertEqual(failure["error"], "solver failed rc=1 termination=PROCESS_EXIT")
        self.assertTrue(failure["candidate_mode_restored"])
        self.assertEqual(stderr.read_text(encoding="utf-8").strip(), self.revision["diagnosis"]["observed_stderr"])
        self.assertEqual(
            Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v3_20260810T184500Z.stdout.log").stat().st_size,
            0,
        )

    def test_delta_is_only_guest_ownership_and_required_modes(self) -> None:
        delta = self.revision["semantic_delta"]
        self.assertEqual(delta["stage_root_uid"], 0)
        self.assertEqual(delta["stage_root_mode"], "0700")
        self.assertEqual((delta["staged_file_uid"], delta["staged_file_gid"]), (1000, 1000))
        self.assertEqual(delta["candidate_transient_mode"], "0500")
        self.assertEqual(delta["data_transient_mode"], "0400")
        self.assertEqual(delta["new_devices"], 0)
        self.assertFalse(delta["new_network"])
        self.assertEqual(delta["new_project_write_targets"], 0)
        self.assertFalse(delta["solver_argv_changed"])
        self.assertFalse(delta["home_permissions_changed"])

    def test_schema_rejects_any_permission_or_boundary_drift(self) -> None:
        mutations = (
            ("stage_root_mode", "0755"),
            ("staged_file_uid", 0),
            ("candidate_transient_mode", "0555"),
            ("data_transient_mode", "0444"),
            ("new_devices", 1),
            ("new_network", True),
            ("new_project_write_targets", 1),
            ("solver_argv_changed", True),
            ("home_permissions_changed", True),
        )
        for key, value in mutations:
            with self.subTest(key=key):
                changed = copy.deepcopy(self.revision)
                changed["semantic_delta"][key] = value
                with self.assertRaises(jsonschema.ValidationError):
                    self.validator.validate(changed)

    def test_guest_file_copy_keeps_bytes_and_exact_modes(self) -> None:
        payload = b"v4-guest-access-test\n"
        source_spec = {
            "path": "/mock/source",
            "sha256": hashlib.sha256(payload).hexdigest(),
            "size_bytes": len(payload),
            "inode": 1,
        }

        def fake_copy(_source, destination, mode):
            descriptor = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
            try:
                os.write(descriptor, payload)
            finally:
                os.close(descriptor)
            return self.runner.base.identity(destination)

        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            self.runner.v3, "copy_exclusive", side_effect=fake_copy
        ):
            for name, mode in (("candidate", 0o500), ("data", 0o400)):
                with self.subTest(mode=oct(mode)):
                    destination = Path(directory) / name
                    observed = self.runner.copy_guest_file(source_spec, destination, mode)
                    self.assertEqual(observed["mode"], f"0{mode:03o}")
                    self.assertEqual((observed["uid"], observed["gid"]), (1000, 1000))
                    self.assertEqual(observed["sha256"], source_spec["sha256"])

    def test_solver_command_and_sandbox_boundaries_are_unchanged(self) -> None:
        policy, _ = self.runner.load_contract()
        self.assertEqual(policy["solver"]["argv"], self.runner.V3_EFFECTIVE_POLICY["solver"]["argv"])
        argv = self.runner.v3.bwrap_argv(policy, command=policy["solver"]["argv"], include_run=True)
        self.assertIn("--unshare-net", argv)
        self.assertEqual(argv[argv.index("--cap-drop") + 1], "ALL")
        self.assertEqual(
            self.runner.v3.pairs(argv, "--dev-bind"),
            [[path, path] for path in policy["gpu"]["device_paths"]],
        )
        self.assertEqual(
            self.runner.v3.pairs(argv, "--bind"),
            [[str(self.runner.MAIN_STAGE_ROOT / "output"), "/output"]],
        )
        self.assertFalse(
            any("/home/zrj/scout_ws" in item for pair in self.runner.v3.pairs(argv, "--bind") for item in pair)
        )

    def test_stage_postcondition_remains_disarmed_and_preserved(self) -> None:
        self.assertEqual(self.revision["transport"]["candidate_stage_final_mode"], "0400")
        self.assertTrue(self.revision["transport"]["preserve_stage"])


if __name__ == "__main__":
    unittest.main()
