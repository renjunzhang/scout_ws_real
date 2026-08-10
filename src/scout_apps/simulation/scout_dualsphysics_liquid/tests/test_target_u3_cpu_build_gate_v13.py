"""Static contract checks for the v13 PIE-only artifact-audit correction.

These tests never create an attempt directory, load AppArmor, invoke
bubblewrap, or execute a candidate ELF.  They prove that the new review layer
can only replace v12's ET_EXEC postflight expectation with the narrowly
specified ET_DYN + DF_1_PIE expectation.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_cpu_build_gate_v13.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v13.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v13.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_cpu_build_gate_v13", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def argv_pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == flag]


class TargetU3CpuBuildGateV13Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_policy_schema_is_closed_and_preserves_v12_execution_boundaries(self):
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(set(self.policy), set(self.schema["required"]))
        self.assertEqual(
            self.policy["schema_version"],
            "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v13",
        )
        self.assertEqual(self.policy["frozen_attempt"]["build_id"], self.gate.BUILD_ID)
        self.assertEqual(
            self.policy["isolation"],
            self.gate.previous.core.read_json_object(self.gate.previous.POLICY_PATH)["isolation"],
        )
        self.assertEqual(
            self.policy["resources"],
            self.gate.previous.core.read_json_object(self.gate.previous.POLICY_PATH)["resources"],
        )
        self.assertTrue(self.policy["invariants"]["no_network"])
        self.assertTrue(self.policy["invariants"]["no_gpu"])
        self.assertTrue(self.policy["invariants"]["no_compiled_artifact_execution"])
        self.assertEqual(
            self.policy["static_artifact_audit"],
            {
                "candidate": "output/artifacts/DualSPHysics5.4CPU_linux64",
                "execution": "forbidden_during_and_after_this_gate",
                "elf_type": "ET_DYN_ONLY_WITH_DF_1_PIE",
                "accepted_e_type": self.gate.ET_DYN,
                "required_dt_flags_1_pie_mask": self.gate.DF_1_PIE,
                "entry_point": "nonzero_within_executable_pt_load",
                "gnu_stack": "exactly_one_non_executable_pt_gnu_stack",
                "interpreter": "/lib64/ld-linux-x86-64.so.2",
                "required_needed": ["libgomp.so.1", "libstdc++.so.6", "libgcc_s.so.1", "libc.so.6"],
                "forbidden_needed_substrings": ["chrono", "dsph", "cuda", "nvidia", "ros", "gazebo", "wavegen", "moordyn"],
                "rpath_or_runpath": "forbidden",
                "disable_opened_regular_candidate_before_audit": True,
                "artifact_mode_after_audit": "0400",
            },
        )

    def test_v12_review_layer_and_policy_are_byte_pinned(self):
        self.assertEqual(
            hashlib.sha256(self.gate.PREVIOUS_GATE_PATH.read_bytes()).hexdigest(),
            self.gate.PREVIOUS_GATE_SHA256,
        )
        self.assertEqual(
            hashlib.sha256(self.gate.previous.POLICY_PATH.read_bytes()).hexdigest(),
            self.gate.PREVIOUS_POLICY_SHA256,
        )
        self.assertNotEqual(self.gate.BUILD_ID, self.gate.previous.BUILD_ID)
        self.assertNotEqual(self.gate.ATTEMPT_ROOT, self.gate.previous.ATTEMPT_ROOT)

    def test_all_derived_core_paths_are_rebound_to_v13_output(self):
        self.assertEqual(self.gate.core.ATTEMPT_ROOT, self.gate.ATTEMPT_ROOT)
        self.assertEqual(self.gate.core.OUTPUT_ROOT, self.gate.OUTPUT_ROOT)
        self.assertEqual(self.gate.core.OUTPUT_SOURCE, self.gate.OUTPUT_SOURCE)
        self.assertEqual(self.gate.core.ARTIFACT_PATH, self.gate.ARTIFACT_PATH)
        self.assertTrue(self.gate.OUTPUT_SOURCE.is_relative_to(self.gate.OUTPUT_ROOT))
        self.assertTrue(self.gate.ARTIFACT_PATH.is_relative_to(self.gate.OUTPUT_ROOT))
        self.assertNotIn(str(self.gate.previous.OUTPUT_ROOT), str(self.gate.OUTPUT_SOURCE))
        self.assertNotIn(str(self.gate.previous.OUTPUT_ROOT), str(self.gate.ARTIFACT_PATH))
        self.assertEqual(self.gate.core.__file__, str(self.gate.SCRIPT_PATH))

    def test_profiles_and_bwrap_keep_exact_guest_lib_and_bind_boundaries(self):
        for path, digest, name in (
            (self.gate.COPY_PROFILE_PATH, self.gate.COPY_PROFILE_SHA256, self.gate.COPY_PROFILE),
            (self.gate.BUILD_PROFILE_PATH, self.gate.BUILD_PROFILE_SHA256, self.gate.BUILD_PROFILE),
        ):
            text = path.read_text(encoding="utf-8")
            effective = "\n".join(line.split("#", 1)[0] for line in text.splitlines())
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
            self.assertIn(f"profile {name} flags=(attach_disconnected,mediate_deleted)", text)
            self.assertEqual(
                [line.strip() for line in effective.splitlines() if line.strip().startswith("/newroot/lib")],
                ["/newroot/lib wl,", "/newroot/lib64 wl,"],
            )
            self.assertNotIn("/newroot/lib/**", effective)
            self.assertNotIn("/oldroot/lib", effective)
            self.assertNotIn("flags=(unconfined)", effective)
        for phase in ("source-copy", "build"):
            argv = self.gate.bwrap_prefix(phase=phase)
            expected_ro = [["/usr", "/usr"]]
            if phase == "source-copy":
                expected_ro.append([str(self.gate.SOURCE_ROOT), "/work/input"])
            self.assertEqual(argv_pairs(argv, "--ro-bind"), expected_ro)
            self.assertEqual(argv_pairs(argv, "--symlink"), [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]])
            bind_pairs = argv_pairs(argv, "--ro-bind") + argv_pairs(argv, "--bind")
            self.assertFalse(any(pair[0] in {"/lib", "/lib64"} or pair[1] in {"/lib", "/lib64"} for pair in bind_pairs))
            self.assertEqual(argv_pairs(argv, "--bind"), [[str(self.gate.OUTPUT_ROOT), "/work/output"]])
            self.assertEqual([argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--tmpfs"], ["/work"])
            self.assertIn("--unshare-net", argv)
            self.assertIn("--disable-userns", argv)
            self.assertNotIn("--proc", argv)
            self.assertNotIn("--dev", argv)
            self.assertNotIn(self.gate.previous.COPY_PROFILE, argv)
            self.assertNotIn(self.gate.previous.BUILD_PROFILE, argv)
            self.assertNotIn(str(self.gate.previous.OUTPUT_ROOT), argv)

    def test_pie_contract_accepts_only_one_et_dyn_df_1_pie_marker(self):
        self.assertEqual(
            self.gate.assert_et_dyn_pie(self.gate.ET_DYN, [self.gate.DF_1_PIE]),
            self.gate.DF_1_PIE,
        )
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_et_dyn_pie(2, [self.gate.DF_1_PIE])
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_et_dyn_pie(self.gate.ET_DYN, [0])
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_et_dyn_pie(self.gate.ET_DYN, [])
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_et_dyn_pie(self.gate.ET_DYN, [self.gate.DF_1_PIE, self.gate.DF_1_PIE])

    def test_elf_load_shape_rejects_invalid_entry_or_gnu_stack(self):
        good = [
            (self.gate.PT_LOAD, self.gate.PF_X, 0, 0x1000, 0x200, 0x300),
            (self.gate.PT_GNU_STACK, 0, 0, 0, 0, 0),
        ]
        self.gate.assert_elf_load_shape(0x1100, good)
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_elf_load_shape(0, good)
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_elf_load_shape(0x1300, good)
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_elf_load_shape(0x1100, good[:1])
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_elf_load_shape(0x1100, [good[0], (self.gate.PT_GNU_STACK, self.gate.PF_X, 0, 0, 0, 0)])
        with self.assertRaises(self.gate.core.GateError):
            self.gate.assert_elf_load_shape(0x1100, [good[0], good[1], good[1]])

    def test_static_review_reports_the_pinned_v12_chain_and_new_audit(self):
        review = self.gate.verify_review_artifacts()
        self.assertEqual(review["frozen_v12_gate"]["sha256"], self.gate.PREVIOUS_GATE_SHA256)
        self.assertEqual(review["frozen_v12_policy"]["sha256"], self.gate.PREVIOUS_POLICY_SHA256)
        self.assertEqual(review["source_copy_profile"]["sha256"], self.gate.COPY_PROFILE_SHA256)
        self.assertEqual(review["build_profile"]["sha256"], self.gate.BUILD_PROFILE_SHA256)

    def test_v13_replaces_the_inherited_build_transition_with_pre_audit_disable(self):
        self.assertIs(self.gate.core.execute_build, self.gate.execute_build_v13)
        self.assertIn("static_elf_audit_v13", self.gate.execute_build_v13.__code__.co_names)

    def test_opened_temporary_non_elf_is_disarmed_before_the_parser_rejects_it(self):
        with tempfile.TemporaryDirectory() as temporary:
            for name, contents in (("mock_candidate", b"not-an-elf" * 16), ("short_candidate", b"short")):
                with self.subTest(name=name):
                    candidate = Path(temporary) / name
                    candidate.write_bytes(contents)
                    os.chmod(candidate, 0o700)
                    with self.assertRaises(self.gate.core.GateError):
                        self.gate.static_elf_audit_v13(candidate)
                    self.assertEqual(candidate.stat().st_mode & 0o777, 0o400)


if __name__ == "__main__":
    unittest.main()
