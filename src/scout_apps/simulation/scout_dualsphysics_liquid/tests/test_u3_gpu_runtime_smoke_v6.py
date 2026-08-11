#!/usr/bin/env python3
"""Static and negative tests for the V6 exec-capable root-only transport."""

from __future__ import annotations

import copy
import importlib.util
import json
import unittest
from pathlib import Path
from unittest import mock

import jsonschema


ROOT = Path(__file__).resolve().parent.parent
RUNNER_PATH = ROOT / "scripts/r8_liquid_u3_gpu_runtime_smoke_v6.py"
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v6.json"
SCHEMA_PATH = ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v6.json"


def load_runner():
    spec = importlib.util.spec_from_file_location("gpu_smoke_v6", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuRuntimeSmokeV6Tests(unittest.TestCase):
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

    def test_all_v5_parents_are_byte_pinned(self) -> None:
        for parent in self.revision["parents"].values():
            self.assertEqual(self.runner.base.sha256_file(Path(parent["path"])), parent["sha256"])

    def test_v5_failure_receipt_and_live_mount_support_diagnosis(self) -> None:
        failure = json.loads(
            Path(self.revision["parents"]["v5_probe_failure"]["path"]).read_text(encoding="utf-8")
        )
        self.assertEqual(failure["status"], "FAIL_NOEXEC_TRANSPORT_ACCESS_PROBE_V5")
        self.assertEqual(failure["mount_target"], "/run")
        self.assertIn("noexec", failure["mount_options"].split(","))
        self.assertFalse(failure["candidate_executed"])
        evidence = self.runner.transport_mount_evidence()
        self.assertEqual((evidence["target"], evidence["fstype"], evidence["noexec"]), ("/var/tmp", "ext4", False))
        self.assertEqual((evidence["directory_uid"], evidence["directory_mode"]), (0, "01777"))

    def test_delta_changes_only_transport_root(self) -> None:
        delta = self.revision["semantic_delta"]
        self.assertEqual((delta["old_transport_root"], delta["new_transport_root"]), ("/run", "/var/tmp"))
        self.assertEqual((delta["new_transport_fstype"], delta["new_transport_noexec"]), ("ext4", False))
        self.assertEqual((delta["stage_root_uid"], delta["stage_root_mode"]), (0, "0700"))
        self.assertEqual((delta["candidate_transient_mode"], delta["data_transient_mode"]), ("0555", "0444"))
        self.assertTrue(delta["guest_bind_read_only"])
        self.assertEqual(delta["new_devices"], 0)
        self.assertFalse(delta["new_network"])
        self.assertEqual(delta["new_project_write_targets"], 0)
        self.assertFalse(delta["solver_argv_changed"])
        self.assertFalse(delta["home_permissions_changed"])

    def test_schema_rejects_transport_or_sandbox_drift(self) -> None:
        mutations = (
            ("new_transport_root", "/home/zrj"),
            ("new_transport_noexec", True),
            ("stage_root_mode", "0777"),
            ("candidate_transient_mode", "0777"),
            ("data_transient_mode", "0666"),
            ("guest_bind_read_only", False),
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

    def test_prepare_stage_keeps_v5_exact_file_modes(self) -> None:
        policy, _ = self.runner.load_contract()
        fake_identity = {
            "path": "/mock", "sha256": "0" * 64, "size_bytes": 1,
            "mode": "0444", "uid": 0, "gid": 0, "inode": 1, "nlink": 1,
        }
        self.runner.v3.STAGE_ROOT = None
        self.runner.v3.STAGE_INPUTS = {}
        self.runner.v3.PROBE_MODE = False
        with (
            mock.patch.object(self.runner.v5.os.path, "lexists", return_value=False),
            mock.patch.object(self.runner.v3, "mkdir_root"),
            mock.patch.object(self.runner.v3, "copy_exclusive", return_value=fake_identity) as copier,
        ):
            self.runner.prepare_stage(policy, self.runner.MAIN_STAGE_ROOT)
        self.assertEqual([call.args[2] for call in copier.call_args_list], [0o555, 0o444, 0o444, 0o444])
        self.runner.v3.STAGE_ROOT = None
        self.runner.v3.STAGE_INPUTS = {}

    def test_solver_argv_devices_and_writable_boundary_are_unchanged(self) -> None:
        policy, _ = self.runner.load_contract()
        self.assertEqual(policy["solver"]["argv"], self.runner.V5_EFFECTIVE_POLICY["solver"]["argv"])
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

    def test_candidate_postcondition_remains_0400(self) -> None:
        self.assertEqual(self.revision["transport"]["candidate_stage_final_mode"], "0400")
        self.assertTrue(self.revision["transport"]["preserve_stage"])


if __name__ == "__main__":
    unittest.main()
