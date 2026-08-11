#!/usr/bin/env python3
"""Static and negative tests for the rootless Stage-4 experiment runner."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_stage4_experiment_runner_v1 as runner  # noqa: E402


CPU_POLICY = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_backend_parity_cpu_a_20260811T131411Z_v16.json"
GPU_POLICY = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_backend_parity_gpu_a_20260811T131411Z_v18.json"


class Stage4ExperimentRunnerV1Tests(unittest.TestCase):
    def test_schema_is_valid_and_all_object_definitions_are_closed(self) -> None:
        schema = json.loads(runner.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertIs(schema["additionalProperties"], False)
        for name, definition in schema["$defs"].items():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False, name)

    def test_exact_cpu_and_gpu_policies_validate(self) -> None:
        cpu = runner.load_policy(CPU_POLICY)
        gpu = runner.load_policy(GPU_POLICY)
        self.assertEqual(cpu["backend"], {"kind": "CPU", "index": None, "parallel_jobs": 1})
        self.assertEqual(gpu["backend"], {"kind": "GPU", "index": 0, "parallel_jobs": 1})
        self.assertIsNone(cpu["gpu"])
        self.assertEqual(gpu["gpu"]["uuid"], "GPU-d29cd22d-8b25-8f4e-9dcf-f2fc4bb113fb")

    def test_deterministic_paths_are_confined_to_lab_and_audits(self) -> None:
        run_id = "u3_c1m_cpu_backend_parity_a_20260811T131411Z_v16"
        paths = runner.expected_run_paths(run_id)
        self.assertEqual(paths["attempt_root"], f"/home/zrj/scout_liquid_lab/stage4_experiments/{run_id}.partial")
        self.assertEqual(paths["guest_output_root"], paths["attempt_root"] + "/staging/guest-output")
        self.assertEqual(paths["final_receipt"], f"/home/zrj/scout_liquid_lab/audits/{run_id}.final.json")

    def test_cpu_sandbox_has_one_write_bind_no_gpu_and_no_network(self) -> None:
        policy = runner.load_policy(CPU_POLICY)
        argv = runner.bwrap_argv(policy)
        writes = [argv[index + 1 : index + 3] for index, item in enumerate(argv) if item == "--bind"]
        self.assertEqual(writes, [[policy["run"]["guest_output_root"], "/output"]])
        self.assertIn("--unshare-net", argv)
        self.assertNotIn("--share-net", argv)
        self.assertNotIn("--dev-bind", argv)
        self.assertNotIn("/home/zrj/scout_ws", argv)
        self.assertEqual(policy["solver"]["environment"], runner._expected_environment("CPU"))

    def test_gpu_sandbox_exposes_only_four_frozen_device_paths(self) -> None:
        policy = runner.load_policy(GPU_POLICY)
        argv = runner.bwrap_argv(policy)
        devices = [argv[index + 1 : index + 3] for index, item in enumerate(argv) if item == "--dev-bind"]
        self.assertEqual(devices, [[path, path] for path in runner.GPU_DEVICE_PATHS])
        self.assertIn("--unshare-net", argv)
        self.assertEqual(policy["solver"]["environment"], runner._expected_environment("GPU"))

    def test_backend_flag_or_environment_drift_is_rejected(self) -> None:
        policy = runner.load_policy(CPU_POLICY)
        for mutate in ("backend", "environment", "cfl"):
            value = copy.deepcopy(policy)
            if mutate == "backend":
                value["solver"]["argv"][3] = "-gpu:0"
            elif mutate == "environment":
                value["solver"]["environment"]["CUDA_VISIBLE_DEVICES"] = "0"
            else:
                value["solver"]["argv"][value["solver"]["argv"].index("-cfl:0.1")] = "-cfl:0.2"
            with self.assertRaises(runner.Stage4ExperimentError, msg=mutate):
                runner.semantic_validate(value, CPU_POLICY)

    def test_network_project_write_or_sudo_drift_is_rejected(self) -> None:
        policy = runner.load_policy(CPU_POLICY)
        for key, value in (("network", True), ("project_workspace_write_bind_count", 1), ("sudo_used", True)):
            changed = copy.deepcopy(policy)
            changed["sandbox"][key] = value
            with self.assertRaises(runner.Stage4ExperimentError, msg=key):
                runner.semantic_validate(changed, CPU_POLICY)

    def test_restart_is_forbidden_for_fresh_parity(self) -> None:
        policy = runner.load_policy(CPU_POLICY)
        policy["restart"] = {"enabled": True, "part_index": 901, "part_first": 901, "guest_dir": "/restart"}
        with self.assertRaisesRegex(runner.Stage4ExperimentError, "fresh zero-motion"):
            runner.semantic_validate(policy, CPU_POLICY)

    def test_resource_contract_cannot_be_weakened(self) -> None:
        policy = runner.load_policy(CPU_POLICY)
        for key, value in (
            ("minimum_mem_available_bytes", 4294967295),
            ("monitor_interval_seconds", 31),
            ("wall_timeout_seconds", 1801),
        ):
            changed = copy.deepcopy(policy)
            changed["limits"][key] = value
            with self.assertRaises(runner.Stage4ExperimentError, msg=key):
                runner.semantic_validate(changed, CPU_POLICY)

    def test_common_numerical_contract_is_single_and_exact(self) -> None:
        for path in (CPU_POLICY, GPU_POLICY):
            policy = runner.load_policy(path)
            argv = policy["solver"]["argv"]
            for flag in runner.COMMON_FLAGS:
                self.assertEqual(argv.count(flag), 1, (path.name, flag))
            self.assertEqual([item for item in argv if item.startswith("-viscoart:")], ["-viscoart:0.3"])
            self.assertNotIn("-viscolam:", " ".join(argv))


if __name__ == "__main__":
    unittest.main()
