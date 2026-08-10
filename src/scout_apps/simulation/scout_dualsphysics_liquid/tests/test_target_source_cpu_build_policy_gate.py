#!/usr/bin/env python3
"""Mock-only tests for the static MSI source-only CPU-build policy gate."""

from __future__ import annotations

import copy
import inspect
import json
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_source_cpu_build_policy_gate as build_policy_gate  # noqa: E402


class TargetSourceCpuBuildPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(build_policy_gate.POLICY_PATH.read_text(encoding="utf-8"))
        self.schema = json.loads(build_policy_gate.SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_checked_in_policy_is_static_only_and_exact(self) -> None:
        self.assertEqual(build_policy_gate.validate_policy(self.policy), [])
        self.assertEqual(build_policy_gate.validate_schema(self.schema), [])
        self.assertTrue(self.policy["policy_review_authorized"])
        self.assertFalse(self.policy["source_materialization_authorized"])
        self.assertFalse(self.policy["sandbox_execution_authorized"])
        self.assertFalse(self.policy["build_execution_authorized"])
        self.assertFalse(self.policy["output_execution_authorized"])
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check"])

    def test_policy_rejects_bind_network_and_toolchain_relaxations(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["sandbox"]["network"] = "host"
        changed["sandbox"]["host_writable_binds"].append(
            {"source": "/tmp", "destination": "/tmp", "must_be_new": False}
        )
        changed["toolchain"]["cmake"]["authorized"] = True
        changed["toolchain"]["make_argv"][8] = "USE_FAST_MATH=YES"
        changed["dynamic_library_policy"]["rpath_or_runpath"] = "allowed"
        changed["invariants"].pop("no_make")
        errors = build_policy_gate.validate_policy(changed)
        self.assertIn("sandbox bind, network, or environment policy differs", errors)
        self.assertIn("CMake/Make/compiler argv policy differs", errors)
        self.assertIn("dynamic-library allowlist or output-audit rule differs", errors)
        self.assertIn("source-only build safety invariants are incomplete", errors)

    def test_exact_recipe_keeps_source_read_only_and_optional_libraries_off(self) -> None:
        self.assertEqual(self.policy["source"]["source_mount_mode"], "read_only")
        self.assertEqual(self.policy["source"]["materialized_input_allowlist"], ["src/source/**"])
        self.assertIn("bin/", self.policy["source"]["materialized_input_forbidden_prefixes"])
        self.assertEqual(self.policy["build_layout"]["host_writable_bind_count"], 1)
        self.assertEqual(
            self.policy["sandbox"]["host_writable_binds"],
            [
                {
                    "source": "<LIQUID_ROOT>/build/<build_id>.partial/output",
                    "destination": "/work/output",
                    "must_be_new": True,
                }
            ],
        )
        argv = self.policy["toolchain"]["make_argv"]
        self.assertIn("COMPILE_CHRONO=NO", argv)
        self.assertIn("COMPILE_WAVEGEN=NO", argv)
        self.assertIn("COMPILE_MOORDYNPLUS=NO", argv)
        self.assertIn("USE_FAST_MATH=NO", argv)
        self.assertNotIn("all", argv)
        self.assertNotIn("clean", argv)
        self.assertFalse(self.policy["toolchain"]["cmake"]["authorized"])

    def test_static_report_claims_no_execution_or_state_change(self) -> None:
        report = build_policy_gate.static_report(self.policy, self.schema)
        self.assertEqual(report["status"], "PASS_SOURCE_CPU_BUILD_POLICY_STATIC_ONLY")
        for key in (
            "source_checkout_created",
            "source_materialized",
            "sandbox_created",
            "network_used",
            "cmake_executed",
            "make_executed",
            "compiler_executed",
            "upstream_code_executed",
            "precompiled_binary_executed",
            "output_executed",
        ):
            self.assertFalse(report[key], key)
        self.assertEqual(
            report["next_allowed_stage"],
            "HUMAN_REVIEW_HARMLESS_OUTPUT_BIND_SMOKE_AND_SOURCE_MATERIALIZATION_POLICY",
        )

    def test_gate_has_no_process_or_write_execution_surface(self) -> None:
        source = inspect.getsource(build_policy_gate)
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("os.system", source)
        self.assertNotIn("Popen(", source)
        self.assertNotIn("write_text(", source)
        self.assertNotIn("mkdir(", source)

    def test_schema_and_handoff_keep_build_artifacts_off_the_other_host(self) -> None:
        self.assertFalse(self.schema["additionalProperties"])
        coordination = self.policy["cross_host_coordination"]
        self.assertEqual(coordination["source_or_binary_transfer_to_gazebo_host"], "forbidden")
        self.assertEqual(coordination["build_artifact_transfer_for_execution"], "forbidden")
        self.assertFalse(coordination["future_motion_or_result_transfer_authorized"])
        self.assertTrue(coordination["each_handoff_requires_manifest_and_sha256"])


if __name__ == "__main__":
    unittest.main()
