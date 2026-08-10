"""Static contract checks for the one-shot MSI U3 CPU-build admission."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_cpu_build_gate.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v10.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v10.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_cpu_build_gate", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TargetU3CpuBuildGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_policy_schema_is_closed_and_single_attempt_only(self):
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(set(self.policy), set(self.schema["required"]))
        self.assertTrue(self.policy["user_delegated_review_and_execution_approval"])
        self.assertEqual(self.policy["frozen_attempt"]["build_id"], self.gate.BUILD_ID)
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check", "preflight", "run-source-copy", "run-build"])
        self.assertTrue(self.policy["outer_privilege_handoff"]["must_drop_before_aa_exec"])
        self.assertEqual(self.policy["outer_privilege_handoff"]["required_supplementary_group_count"], 0)
        self.assertEqual(
            self.policy["isolation"]["disable_userns_mechanism"],
            {
                "bubblewrap_package_version": "0.9.0-1ubuntu0.1",
                "proc_path": "/proc/sys/user/max_user_namespaces",
                "open_mode": "O_WRONLY",
                "write_value": "1",
                "write_namespace": "first-level bubblewrap user namespace only",
                "kernel_write_capability": "CAP_SYS_RESOURCE in that first-level user namespace only",
                "host_initial_namespace_postcondition": "exact host value unchanged before and after the child",
            },
        )
        self.assertEqual(
            self.policy["isolation"]["required_profile_capabilities"],
            ["sys_admin", "sys_ptrace", "sys_resource", "setpcap", "net_admin"],
        )
        self.assertTrue(
            self.policy["invariants"]["no_host_initial_namespace_sysctl_or_apparmor_global_setting_change"]
        )
        self.assertTrue(self.policy["invariants"]["no_precompiled_elf_execution"])
        self.assertTrue(self.policy["invariants"]["no_compiled_artifact_execution"])

    def test_profiles_are_frozen_named_and_default_deny_for_artifacts(self):
        for path, expected_hash, profile_name in (
            (self.gate.COPY_PROFILE_PATH, self.gate.COPY_PROFILE_SHA256, self.gate.COPY_PROFILE),
            (self.gate.BUILD_PROFILE_PATH, self.gate.BUILD_PROFILE_SHA256, self.gate.BUILD_PROFILE),
        ):
            text = path.read_text(encoding="utf-8")
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected_hash)
            self.assertIn(f"profile {profile_name} flags=(attach_disconnected,mediate_deleted)", text)
            self.assertIn("REVIEWED_SINGLE_ATTEMPT_ONLY", text)
            effective = "\n".join(line.split("#", 1)[0] for line in text.splitlines())
            self.assertNotIn("flags=(unconfined)", effective)
            self.assertNotIn("change_profile", effective)
            self.assertNotIn("/home/zrj/scout_ws", effective)
            self.assertNotIn("GenCase_linux64", effective)
            self.assertNotIn("DualSPHysics5.4CPU_linux64 rix", effective)
            self.assertIn("/proc/sys/user/max_user_namespaces w,", effective)
            self.assertNotIn("/proc/sys/user/max_user_namespaces rw,", effective)
            self.assertIn("capability sys_resource,", effective)

    def test_v10_header_grant_is_exact_build_only_and_bound_to_the_new_attempt(self):
        copy_effective = "\n".join(
            line.split("#", 1)[0]
            for line in self.gate.COPY_PROFILE_PATH.read_text(encoding="utf-8").splitlines()
        )
        build_effective = "\n".join(
            line.split("#", 1)[0]
            for line in self.gate.BUILD_PROFILE_PATH.read_text(encoding="utf-8").splitlines()
        )
        self.assertNotIn("/usr/include", copy_effective)
        self.assertIn("/usr/include/ r,", build_effective)
        self.assertIn("/usr/include/** r,", build_effective)
        self.assertNotIn("/usr/include/** rw,", build_effective)
        self.assertNotIn("/usr/local/include", build_effective)
        self.assertNotIn("/usr/**", build_effective)
        self.assertIn(str(self.gate.ATTEMPT_ROOT), copy_effective)
        self.assertIn(str(self.gate.ATTEMPT_ROOT), build_effective)

    def test_fixed_commands_have_required_isolation_and_no_artifact_execution(self):
        copy_argv = self.gate.bwrap_prefix(phase="source-copy")
        build_argv = self.gate.bwrap_prefix(phase="build")
        for argv in (copy_argv, build_argv):
            self.assertIn("--unshare-user", argv)
            self.assertIn("--unshare-pid", argv)
            self.assertIn("--unshare-net", argv)
            self.assertIn("--unshare-ipc", argv)
            self.assertIn("--unshare-uts", argv)
            self.assertIn("--disable-userns", argv)
            self.assertIn("--assert-userns-disabled", argv)
            self.assertIn("--clearenv", argv)
            self.assertIn("--ro-bind", argv)
            self.assertIn("--bind", argv)
            self.assertNotIn("--share-net", argv)
        self.assertIn("/usr/bin/cp", copy_argv)
        self.assertIn("/usr/bin/make", build_argv)
        self.assertNotIn("/usr/bin/setpriv", copy_argv)
        self.assertNotIn("/usr/bin/setpriv", build_argv)
        self.assertNotIn("usr/bin", copy_argv[copy_argv.index("--symlink") + 1 :])
        self.assertNotIn("/bin", copy_argv)
        self.assertNotIn("/lib", copy_argv)
        self.assertIn("/lib64", copy_argv)
        self.assertNotIn("--proc", copy_argv)
        self.assertNotIn("--dev", copy_argv)
        self.assertNotIn("--proc", build_argv)
        self.assertNotIn("--dev", build_argv)
        self.assertNotIn("/tmp", copy_argv)
        self.assertNotIn("/tmp", build_argv)
        self.assertIn("/work", copy_argv)
        self.assertIn("SHELL=/usr/bin/dash", build_argv)
        self.assertNotIn("GenCase_linux64", build_argv)
        self.assertNotIn(str(self.gate.ARTIFACT_PATH), build_argv)

    def test_wrapper_restores_only_the_source_suffix_rule(self):
        self.assertEqual(
            self.gate.WRAPPER_CONTENT,
            ".SUFFIXES:\n.SUFFIXES: .cpp .o\ninclude Makefile_cpu\noverride CCFLAGS += -include cstdint\n",
        )
        self.assertEqual(
            self.policy["generated_wrapper"]["content"],
            self.gate.WRAPPER_CONTENT,
        )
        self.assertEqual(
            self.policy["generated_wrapper"]["purpose"],
            "restore only the checked-in .cpp-to-.o suffix rule and append the GCC 13 SIZE_MAX compatibility include without editing sealed source",
        )
        self.assertEqual(self.gate.WRAPPER_CONTENT.count("-include"), 1)
        self.assertIn("-include cstdint", self.gate.WRAPPER_CONTENT)
        self.assertEqual(self.policy["build_command"][:2], ["/usr/bin/make", "--no-builtin-rules"])

    def test_host_userns_postcondition_is_fail_closed(self):
        self.assertGreaterEqual(self.gate.host_userns_limit(), 2)
        self.assertEqual(
            self.gate.host_userns_postcondition(
                {"host_initial_namespace_user_max_user_namespaces_before": self.gate.host_userns_limit()}
            )["unchanged"],
            True,
        )
        with self.assertRaises(self.gate.GateError):
            self.gate.host_userns_postcondition(
                {"host_initial_namespace_user_max_user_namespaces_before": 1}
            )


if __name__ == "__main__":
    unittest.main()
