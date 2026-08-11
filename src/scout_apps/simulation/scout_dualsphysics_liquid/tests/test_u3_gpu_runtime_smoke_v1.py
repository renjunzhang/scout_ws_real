#!/usr/bin/env python3
"""Static and negative tests for the U3 RTX 5080 GPU smoke v1 contract."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path

import jsonschema


ROOT = Path(__file__).resolve().parent.parent
RUNNER_PATH = ROOT / "scripts/r8_liquid_u3_gpu_runtime_smoke_v1.py"
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v1.json"
SCHEMA_PATH = ROOT / "schema/target_host_u3_gpu_runtime_smoke_policy_v1.json"


def load_runner():
    spec = importlib.util.spec_from_file_location("gpu_smoke_v1", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuRuntimeSmokeV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
        cls.validator = jsonschema.Draft202012Validator(cls.schema)

    def validate(self, policy: dict) -> None:
        self.validator.validate(policy)

    def test_closed_schema_and_frozen_policy_pass(self) -> None:
        jsonschema.Draft202012Validator.check_schema(self.schema)
        self.validate(self.policy)
        self.runner.static_semantic_check(self.policy)

    def test_closed_schema_rejects_top_level_extra(self) -> None:
        policy = copy.deepcopy(self.policy)
        policy["extra"] = True
        with self.assertRaises(jsonschema.ValidationError):
            self.validate(policy)

    def test_closed_schema_rejects_extra_solver_environment(self) -> None:
        policy = copy.deepcopy(self.policy)
        policy["solver"]["environment"]["LD_PRELOAD"] = "/tmp/inject.so"
        with self.assertRaises(jsonschema.ValidationError):
            self.validate(policy)

    def test_gpu_device_allowlist_is_exact(self) -> None:
        policy = copy.deepcopy(self.policy)
        policy["gpu"]["device_paths"].append("/dev/nvidia-modeset")
        with self.assertRaises(jsonschema.ValidationError):
            self.validate(policy)

    def test_architecture_runtime_flag_cannot_drift_to_cpu(self) -> None:
        policy = copy.deepcopy(self.policy)
        policy["solver"]["argv"][3] = "-cpu"
        with self.assertRaises(self.runner.SmokeError):
            self.runner.static_semantic_check(policy)

    def test_solver_time_and_output_flags_are_frozen(self) -> None:
        for index, replacement in ((8, "-tmax:0.1"), (9, "-tout:0.1"), (10, "-sv:info")):
            with self.subTest(index=index):
                policy = copy.deepcopy(self.policy)
                policy["solver"]["argv"][index] = replacement
                with self.assertRaises(self.runner.SmokeError):
                    self.runner.static_semantic_check(policy)

    def test_probe_has_no_host_writable_bind(self) -> None:
        argv = self.runner.bwrap_argv(
            self.policy, command=["/usr/bin/nvidia-smi", "-L"], include_run=False
        )
        self.assertNotIn("--bind", argv)
        self.assertIn("--unshare-net", argv)
        self.assertIn("--unshare-user", argv)
        self.assertNotIn("--share-net", argv)

    def test_run_has_only_exact_output_writable_bind(self) -> None:
        argv = self.runner.bwrap_argv(
            self.policy, command=self.policy["solver"]["argv"], include_run=True
        )
        pairs = self.runner.pairs(argv, "--bind")
        self.assertEqual(pairs, [[self.policy["run"]["output_root"], "/output"]])
        self.assertFalse(any("/home/zrj/scout_ws" in item for pair in pairs for item in pair))

    def test_run_drops_all_capabilities_and_network(self) -> None:
        argv = self.runner.bwrap_argv(
            self.policy, command=self.policy["solver"]["argv"], include_run=True
        )
        self.assertEqual(argv[argv.index("--cap-drop") + 1], "ALL")
        self.assertEqual(argv.count("--unshare-net"), 1)
        self.assertEqual(argv[argv.index("--uid") + 1], "1000")
        self.assertEqual(argv[argv.index("--gid") + 1], "1000")

    def test_writable_bind_tamper_is_rejected(self) -> None:
        policy = copy.deepcopy(self.policy)
        policy["run"]["output_root"] = "/home/zrj/scout_ws/output"
        with self.assertRaises(self.runner.SmokeError):
            self.runner.static_semantic_check(policy)

    def test_expected_output_contract_is_30_files(self) -> None:
        self.assertEqual(len(self.runner.EXPECTED_ROOT_FILES), 5)
        self.assertEqual(len(self.runner.EXPECTED_DATA_FILES), 25)
        self.assertEqual(
            len([name for name in self.runner.EXPECTED_DATA_FILES if name.startswith("Part_") and name.endswith(".bi4")]),
            21,
        )

    def test_log_scan_accepts_clean_and_rejects_cuda_error(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-gpu-smoke-test-") as tmp:
            root = Path(tmp)
            stdout = root / "stdout.log"
            stderr = root / "stderr.log"
            stdout.write_text("Simulation finished\nFinished execution (code=0).\n", encoding="utf-8")
            stderr.write_bytes(b"\n")
            os.chmod(stdout, 0o600)
            os.chmod(stderr, 0o600)
            self.runner.log_safety(stdout, stderr)
            stdout.write_text("CUDA error: launch failure\n", encoding="utf-8")
            with self.assertRaises(self.runner.SmokeError):
                self.runner.log_safety(stdout, stderr)


if __name__ == "__main__":
    unittest.main()
