#!/usr/bin/env python3
"""Static and negative tests for the Stage-4 one-shot GPU runner."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_u3_gpu_stage4_runner_v3.py"
SCHEMA = ROOT / "schema/target_host_u3_gpu_stage4_run_policy_v3.json"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_shift_none_extend_from_0201_v3.json"


def load_module():
    spec = importlib.util.spec_from_file_location("u3_gpu_stage4_runner_v3", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def assert_deep_closed(test: unittest.TestCase, value, path: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" or (isinstance(value.get("type"), list) and "object" in value["type"]):
            test.assertIs(value.get("additionalProperties"), False, path)
        for key, item in value.items():
            assert_deep_closed(test, item, f"{path}/{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            assert_deep_closed(test, item, f"{path}/{index}")


class Stage4RunnerV3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_module()
        cls.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        cls.policy = json.loads(POLICY.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(cls.schema)

    def test_policy_validates_against_deep_closed_schema(self) -> None:
        assert_deep_closed(self, self.schema)
        errors = list(Draft202012Validator(self.schema).iter_errors(self.policy))
        self.assertEqual([], [item.message for item in errors])

    def test_static_self_check_verifies_all_parents_and_inputs(self) -> None:
        result = self.runner.self_check(POLICY)
        self.assertEqual(result["status"], "PASS_U3_GPU_STAGE4_RUNNER_V3_SELF_CHECK")
        self.assertFalse(result["candidate_executed"])
        self.assertEqual(set(result["verified"]["inputs"]), set(self.policy["inputs"]))
        self.assertEqual(set(result["verified"]["parents"]), set(self.policy["parents"]))

    def test_bwrap_has_one_output_write_bind_and_no_network(self) -> None:
        argv = self.runner.bwrap_argv(self.policy)
        self.assertIn("--unshare-net", argv)
        self.assertNotIn("--share-net", argv)
        self.assertEqual(argv[argv.index("--cap-drop") + 1], "ALL")
        self.assertEqual(argv[argv.index("--uid") + 1], "1000")
        self.assertEqual(argv[argv.index("--gid") + 1], "1000")
        writes = [argv[index + 1:index + 3] for index, item in enumerate(argv) if item == "--bind"]
        self.assertEqual(writes, [[self.policy["sandbox"]["stage_root"] + "/output", "/output"]])
        self.assertFalse(any("/home/zrj/scout_ws" in item for pair in writes for item in pair))

    def test_restart_is_exact_read_only_bind_and_exact_argv(self) -> None:
        argv = self.runner.bwrap_argv(self.policy)
        read_binds = [argv[index + 1:index + 3] for index, item in enumerate(argv) if item == "--ro-bind"]
        self.assertIn([self.policy["sandbox"]["stage_root"] + "/restart", "/restart"], read_binds)
        self.assertIn("-partbegin:201:201", argv)
        self.assertEqual(argv[argv.index("-partbegin:201:201") + 1], "/restart")
        self.assertIn("-shifting:none", argv)
        self.assertIn("-tmax:20.05", argv)

    def test_schema_rejects_unknown_top_level_and_nested_fields(self) -> None:
        for mutate in (
            lambda value: value.update({"unreviewed": True}),
            lambda value: value["sandbox"].update({"unreviewed": True}),
            lambda value: value["inputs"]["candidate"].update({"unreviewed": True}),
        ):
            tampered = copy.deepcopy(self.policy)
            mutate(tampered)
            self.assertTrue(list(Draft202012Validator(self.schema).iter_errors(tampered)))

    def test_semantics_reject_network_and_project_write(self) -> None:
        for key in ("network", "project_workspace_write_bind_count"):
            tampered = copy.deepcopy(self.policy)
            tampered["sandbox"][key] = True if key == "network" else 1
            with self.assertRaises(self.runner.Stage4RunError):
                self.runner.semantic_validate(tampered, POLICY)

    def test_semantics_reject_gpu_or_restart_drift(self) -> None:
        mutations = [
            lambda value: value["solver"]["argv"].__setitem__(3, "-gpu:1"),
            lambda value: value["solver"]["argv"].remove("-partbegin:201:201"),
            lambda value: value["solver"]["argv"].__setitem__(10, "-shifting:nobound"),
            lambda value: value["single_delta"].update({"candidate": "25.05"}),
        ]
        for mutate in mutations:
            tampered = copy.deepcopy(self.policy)
            mutate(tampered)
            with self.assertRaises(self.runner.Stage4RunError):
                self.runner.semantic_validate(tampered, POLICY)

    def test_source_candidate_remains_disarmed(self) -> None:
        observed = self.runner.verify_spec(self.policy["inputs"]["candidate"], full=True, maximum=128 * 1024 * 1024)
        self.assertEqual(observed["mode"], "0400")
        self.assertEqual(observed["sha256"], self.policy["inputs"]["candidate"]["sha256"])

    def test_v3_closes_over_probe_result_and_extended_contract(self) -> None:
        parents = self.policy["parents"]
        self.assertEqual(
            parents["prior_gpu_final"]["sha256"],
            "66bb86587d7963650cb16685eaae2dc6a8bc5fc0a0a94fbbe71c5316606d6644",
        )
        self.assertEqual(
            parents["prior_gpu_qc"]["sha256"],
            "682de1a6a80697ac01444337e6cae65b380166be7c328b5a613d1c4791570af9",
        )
        self.assertEqual(
            parents["numerical_contract"]["sha256"],
            "d36997e1a4ab31f9a7826062d2fdf08508f6653918827c32fc6968002ac92c16",
        )
        self.assertTrue(self.policy["run_id"].endswith("_v3"))
        self.assertTrue(all("_v3" in self.policy["run"][key] for key in self.policy["run"]))
        self.assertEqual(self.policy["expected_output"]["part_count"], 201)

    def test_exact_runs_parent_is_create_new_and_idempotently_verified(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            lab = Path(temporary).resolve()
            runs = lab / "gpu_stage4_runs"
            patches = (
                mock.patch.object(self.runner, "LAB_ROOT", lab),
                mock.patch.object(self.runner, "RUNS_PARENT", runs),
                mock.patch.object(self.runner, "RUNS_PARENT_UID", os.getuid()),
                mock.patch.object(self.runner, "RUNS_PARENT_GID", os.getgid()),
            )
            with patches[0], patches[1], patches[2], patches[3]:
                self.assertFalse(self.runner.runs_parent_identity(allow_absent=True)["exists"])
                created = self.runner.ensure_runs_parent()
                self.assertTrue(created["created_by_this_call"])
                self.assertEqual(created["mode"], "0700")
                verified = self.runner.ensure_runs_parent()
                self.assertFalse(verified["created_by_this_call"])
                self.assertEqual(verified["inode"], created["inode"])

    def test_runs_parent_rejects_symlink_and_wrong_mode(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            lab = Path(temporary).resolve()
            target = lab / "real"
            target.mkdir(mode=0o700)
            runs = lab / "gpu_stage4_runs"
            runs.symlink_to(target, target_is_directory=True)
            with mock.patch.object(self.runner, "LAB_ROOT", lab), mock.patch.object(self.runner, "RUNS_PARENT", runs):
                with self.assertRaises(self.runner.Stage4RunError):
                    self.runner.runs_parent_identity(allow_absent=False)
            runs.unlink()
            runs.mkdir(mode=0o755)
            with (
                mock.patch.object(self.runner, "LAB_ROOT", lab),
                mock.patch.object(self.runner, "RUNS_PARENT", runs),
                mock.patch.object(self.runner, "RUNS_PARENT_UID", os.getuid()),
                mock.patch.object(self.runner, "RUNS_PARENT_GID", os.getgid()),
            ):
                with self.assertRaises(self.runner.Stage4RunError):
                    self.runner.runs_parent_identity(allow_absent=False)


if __name__ == "__main__":
    unittest.main()
