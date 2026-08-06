#!/usr/bin/env python3
"""No-ROS and no-upstream-execution tests for the P0-B sandbox gate."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_dependency_gate as dependency  # noqa: E402
import r8_liquid_safety as safety  # noqa: E402
import r8_liquid_sandbox_gate as sandbox  # noqa: E402


class SandboxPolicyTests(unittest.TestCase):
    def test_policies_are_development_only_and_keep_upstream_no_go(self) -> None:
        policies = {
            name: json.loads(path.read_text(encoding="utf-8"))
            for name, path in sandbox.POLICY_FILES.items()
        }
        self.assertEqual(sandbox.validate_frozen_policy_semantics(policies), [])
        for policy in policies.values():
            self.assertTrue(policy["development_only"])
            self.assertFalse(policy["formal"])
            self.assertFalse(policy["physical_primary_eligible"])
        self.assertFalse(policies["vm"]["runtime"]["system_install_authorized"])
        self.assertEqual(
            policies["cpu_build"]["build_recipe_status"],
            "NOT_FROZEN_NOT_AUTHORIZED_TO_RUN",
        )
        self.assertEqual(
            policies["gencase"]["binary"]["current_admission"],
            "NOT_EXECUTED_NOT_ADMITTED",
        )

    def test_minimal_build_disables_gpu_and_optional_components(self) -> None:
        policy = json.loads(sandbox.POLICY_FILES["cpu_build"].read_text(encoding="utf-8"))
        self.assertEqual(
            policy["features"],
            {
                "cpu": True,
                "cuda": False,
                "chrono": False,
                "wavegen": False,
                "moordynplus": False,
            },
        )
        self.assertEqual(policy["sandbox"]["network"], "none")
        self.assertEqual(policy["sandbox"]["gpu_devices"], [])
        self.assertEqual(policy["sandbox"]["output_execution"], "forbidden")

    def test_candidate_recipe_avoids_destructive_targets_and_prebuilt_libraries(self) -> None:
        path = sandbox.PACKAGE_ROOT / "config/sandbox/p0b_cpu_build_recipe_candidate_v1.json"
        recipe = json.loads(path.read_text(encoding="utf-8"))
        argv = recipe["candidate_argv"]
        self.assertEqual(argv[0], "/usr/bin/make")
        self.assertIn("COMPILE_CHRONO=NO", argv)
        self.assertIn("COMPILE_WAVEGEN=NO", argv)
        self.assertIn("COMPILE_MOORDYNPLUS=NO", argv)
        self.assertIn("USE_FAST_MATH=NO", argv)
        self.assertNotIn("all", argv)
        self.assertNotIn("clean", argv)
        self.assertIn("bin/**", recipe["staging"]["copy_exclude"])
        self.assertIn("src/lib/**", recipe["staging"]["copy_exclude"])
        self.assertFalse(recipe["post_build"]["execute_output"])
        self.assertEqual(
            recipe["status"],
            "STATIC_REVIEWED_CANDIDATE_NOT_AUTHORIZED_TO_RUN",
        )

    def test_gencase_hash_and_scope_are_exact(self) -> None:
        policy = json.loads(sandbox.POLICY_FILES["gencase"].read_text(encoding="utf-8"))
        self.assertEqual(
            policy["binary"]["sha256"],
            "a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226",
        )
        self.assertEqual(policy["first_execution"]["environment"], "disposable_kvm_vm")
        self.assertEqual(policy["first_execution"]["host_shared_directories"], [])
        self.assertIn("MeasureTool_linux64", policy["explicitly_not_admitted"])

    def test_vm_policy_forbids_host_shares_network_and_gpu(self) -> None:
        policy = json.loads(sandbox.POLICY_FILES["vm"].read_text(encoding="utf-8"))
        guest = policy["guest"]
        self.assertEqual(guest["network"], "none")
        self.assertFalse(guest["gpu_passthrough"])
        self.assertEqual(guest["shared_directories"], [])
        self.assertTrue(guest["snapshot_mode"])
        self.assertEqual(policy["runtime"]["qemu_system_path"], str(sandbox.QEMU_SYSTEM))

    def test_preflight_fails_closed_without_admitted_qemu_and_never_starts_upstream(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-sandbox-test-") as temporary:
            base = Path(temporary)
            parent = base / "scout_sim_replacement"
            parent.mkdir()
            root = parent / "r8_liquid"
            missing_qemu = base / "missing-qemu"
            fake_host = {
                "status": "PASS",
                "errors": [],
                "resource_policy": {"status": "PASS"},
                "gpu": {"compute_processes": []},
                "simulation_process_check": {"active_ports": [], "active_processes": []},
            }
            fake_dependency = {"status": "PASS", "upstream_code_executed": False}
            with mock.patch.object(safety, "APPROVED_ROOT", root), mock.patch.object(
                sandbox, "QEMU_SYSTEM", missing_qemu
            ), mock.patch.object(sandbox, "QEMU_IMG", base / "missing-qemu-img"), mock.patch.object(
                safety, "build_preflight", return_value=fake_host
            ), mock.patch.object(
                dependency, "verify_existing", return_value=fake_dependency
            ), mock.patch.object(
                dependency, "dependency_paths", return_value={"manifest": base / "missing-manifest"}
            ):
                safety.prepare_layout()
                report = sandbox.build_preflight("vm-host")
            self.assertEqual(report["status"], "NO_GO")
            self.assertFalse(report["upstream_code_executed"])
            self.assertFalse(report["vm_started"])
            self.assertFalse(report["build_started"])
            self.assertFalse(report["gencase_started"])
            self.assertTrue(any("QEMU system runtime is missing" in item for item in report["errors"]))
            self.assertTrue(any("not been explicitly admitted" in item for item in report["errors"]))

    def test_receipt_path_is_fixed_to_audit_directory_and_mode(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-sandbox-path-test-") as temporary:
            parent = Path(temporary) / "scout_sim_replacement"
            parent.mkdir()
            root = parent / "r8_liquid"
            with mock.patch.object(safety, "APPROVED_ROOT", root):
                safety.prepare_layout()
                valid = root / "audits/sandbox/sandbox_preflight_vm-host_20260805T120000Z.json"
                self.assertEqual(sandbox.validate_receipt_path(valid, "vm-host"), valid)
                with self.assertRaises(sandbox.SandboxGateError):
                    sandbox.validate_receipt_path(
                        root / "audits/sandbox/sandbox_preflight_gencase_20260805T120000Z.json",
                        "vm-host",
                    )
                with self.assertRaises(sandbox.SandboxGateError):
                    sandbox.validate_receipt_path(
                        root / "scratch/sandbox_preflight_vm-host_20260805T120000Z.json",
                        "vm-host",
                    )

    def test_meminfo_and_required_layout_contract(self) -> None:
        memory = sandbox.meminfo_bytes()
        self.assertGreater(memory.get("MemTotal", 0), 0)
        self.assertGreater(memory.get("MemAvailable", 0), 0)
        self.assertIn("dependency/vm_images", sandbox.REQUIRED_LAYOUT_DIRS)
        self.assertIn("scratch/build", sandbox.REQUIRED_LAYOUT_DIRS)
        self.assertIn("quarantine", sandbox.REQUIRED_LAYOUT_DIRS)


if __name__ == "__main__":
    unittest.main()
