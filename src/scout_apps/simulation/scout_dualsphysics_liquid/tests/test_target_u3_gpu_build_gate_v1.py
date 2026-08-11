"""Mock/static-only tests for the RTX 5080 GPU v1 build contract."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_gpu_build_gate_v1.py"
POLICY_PATH = (
    PACKAGE_DIR
    / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gpu_build_execution_policy_v1.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_gpu_build_gate_v1", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def valid_source_copy_trace(gate, policy):
    contract = policy["source_copy_contract"]
    profile = policy["profiles"]["attempt_a_copy"]["name"]
    evidence = [
        {"create_new": True, "path": contract["start_record"]},
        {"isolated": True, "copied_entry_count": 352},
        {
            "entry_count": 352,
            "extra_count": 0,
            "symlink_count": 0,
            "hardlink_count": 0,
            "elf_count": 0,
            "executable_count": 0,
        },
        {"profile_name": profile, "unloaded": True, "zero_residue": True},
        {
            "create_flags": ["O_WRONLY", "O_CREAT", "O_EXCL", "O_NOFOLLOW"],
            "mode_octal": "0600",
            "sha256": gate.WRAPPER_SHA256,
            "size_bytes": 84,
            "after_unload_sequence": 4,
        },
        {
            "entry_count": 353,
            "sealed_entry_count": 352,
            "wrapper_entry_count": 1,
            "extra_count": 0,
            "symlink_count": 0,
            "hardlink_count": 0,
            "elf_count": 0,
            "executable_count": 0,
            "wrapper_sha256": gate.WRAPPER_SHA256,
            "wrapper_mode_octal": "0600",
        },
        {
            "create_new": True,
            "path": contract["final_receipt"],
            "published": True,
            "after_complete_inventory_sequence": 6,
        },
    ]
    return [
        {
            "state": state,
            "sequence": index,
            "captured_at_ns": index,
            "evidence": evidence[index - 1],
        }
        for index, state in enumerate(gate.SOURCE_COPY_STATES, start=1)
    ]


def valid_inventory(gate):
    sealed_paths = {f"sealed/{index:03d}.src" for index in range(352)}
    entries = [
        {
            "path": path,
            "kind": "regular",
            "symlink": False,
            "nlink": 1,
            "mode_octal": "0644",
            "size_bytes": 1,
            "sha256": hashlib.sha256(path.encode("utf-8")).hexdigest(),
            "is_elf": False,
        }
        for path in sorted(sealed_paths)
    ]
    entries.append(
        {
            "path": "U3GpuBuild.mk",
            "kind": "regular",
            "symlink": False,
            "nlink": 1,
            "mode_octal": "0600",
            "size_bytes": 84,
            "sha256": gate.WRAPPER_SHA256,
            "is_elf": False,
        }
    )
    return entries, sealed_paths


def valid_identity(gate):
    return {
        "realpath": str(gate.OUTPUT_A / "artifacts/DualSPHysics5.4_linux64"),
        "parent_realpath": str(gate.OUTPUT_A / "artifacts"),
        "uid": 1000,
        "gid": 1000,
        "st_dev": 1,
        "st_ino": 2,
        "regular": True,
        "symlink": False,
        "nlink": 1,
        "size": 123,
        "mode_octal": "0400",
        "sha256": "1" * 64,
    }


def valid_b_admission(gate, policy, delta_id):
    contract = policy["b_admission_contract"]
    selected = next(
        item for item in contract["delta_catalog"] if item["id"] == delta_id
    )
    if selected["gxx13_permission"]:
        execute = [item["path"] for item in contract["gxx13_exact_tools"]]
        reads = [selected["gcc13_read_root"]]
    elif delta_id == "cudart_static_read_visibility":
        execute = []
        reads = [selected["only_read_path"]]
    else:
        execute = []
        reads = []
    rules = (
        [
            {
                "path": "/usr/local/cuda-12.8/targets/x86_64-linux/include/exact_header.h",
                "access": "r",
            }
        ]
        if delta_id == "apparmor_exact_evidence_permission"
        else []
    )
    return {
        "schema_version": contract["manifest_schema_version"],
        "document_type": contract["manifest_document_type"],
        "campaign_id": gate.CAMPAIGN_ID,
        "build_id": gate.BUILD_ID_B,
        "attempt_root": str(gate.ROOT_B),
        "selected_delta_id": delta_id,
        "root_cause_evidence_sha256": "2" * 64,
        "parent_hashes": {
            path: "3" * 64 for path in contract["parent_file_paths"]
        },
        "make_argv": gate.expected_b_make_argv(delta_id, policy),
        "wrapper_sha256": contract["wrapper_variants"][
            selected["wrapper_variant"]
        ]["sha256"],
        "copy_profile": {
            "name": policy["profiles"]["attempt_b_copy"]["name"],
            "path": policy["profiles"]["attempt_b_copy"]["planned_path"],
            "sha256": "4" * 64,
            "attempt_root": str(gate.ROOT_B),
        },
        "build_profile": {
            "name": policy["profiles"]["attempt_b_build"]["name"],
            "path": policy["profiles"]["attempt_b_build"]["planned_path"],
            "sha256": "5" * 64,
            "attempt_root": str(gate.ROOT_B),
        },
        "permission_delta": {
            "execute_tools": execute,
            "read_paths": reads,
            "apparmor_rules": rules,
        },
        "created_at_utc": "2026-08-10T12:00:00Z",
        "status": contract["manifest_status"],
        "next_allowed_stage": contract["manifest_next_allowed_stage"],
    }


class TargetU3GpuBuildGateV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_self_check_freezes_contract_without_build_or_system_action(self):
        result = self.gate.self_check()
        self.assertEqual(result["status"], "A_CONTRACT_AND_PROFILES_FROZEN")
        self.assertEqual(result["object_contract"]["total_object_count"], 131)
        self.assertEqual(result["object_contract"]["cpp_object_count"], 120)
        self.assertEqual(result["object_contract"]["cuda_object_count"], 11)
        self.assertEqual(result["tool_identity_count"], 40)
        self.assertFalse(result["system_actions_performed"])
        self.assertFalse(result["gpu_build_started"])
        self.assertFalse(result["profile_loaded"])
        self.assertFalse(result["candidate_executed"])

    def test_recursive_schema_is_closed_and_rejects_nested_extra(self):
        validated = self.gate.validate_policy_schema(self.policy, self.schema)
        self.assertEqual(validated, self.policy)
        mutated = copy.deepcopy(self.policy)
        mutated["resources"]["runtime_parallel_expression"] = "nproc"
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_policy_schema(mutated, self.schema)

    def test_parallel_resources_and_literal_make_argv(self):
        self.assertEqual(self.gate.PARALLEL_JOBS, 1)
        self.assertEqual(self.policy["g0_input"]["parallel_jobs"], 1)
        self.assertEqual(self.policy["resources"]["parallel_jobs"], 1)
        self.assertEqual(self.policy["resources"]["wall_timeout_seconds"], 5400)
        self.assertEqual(self.policy["resources"]["cpu_limit_seconds"], 5400)
        self.assertEqual(
            self.policy["resources"]["minimum_available_memory_bytes"], 4294967296
        )
        self.assertEqual(
            self.policy["resources"]["address_space_limit_bytes"], 8589934592
        )
        self.assertEqual(
            self.policy["resources"]["memory_monitor_interval_seconds"], 20
        )
        argv = self.policy["build_contract"]["make_argv"]
        self.assertEqual(argv, list(self.gate.MAKE_ARGV))
        self.assertEqual(argv.count("-j1"), 1)
        self.assertEqual(argv.count(self.gate.GENCODE_ARG), 1)
        joined = "\n".join(argv)
        for forbidden in (
            "-j2",
            "-j4",
            "-use_fast_math",
            "-ffast-math",
            "-march=native",
            "--allow-unsupported-compiler",
            "sm_61",
            "sm_70",
            "sm_86",
            "compute_61",
            "compute_70",
            "compute_86",
        ):
            self.assertNotIn(forbidden, joined)
        self.assertNotIn("3600", json.dumps(self.policy["resources"]))

    def test_wrapper_is_exact_and_attempt_a_has_no_ncc_fallback(self):
        self.gate.verify_wrapper_contract(self.policy)
        self.assertEqual(len(self.gate.WRAPPER_BYTES), 84)
        self.assertEqual(
            hashlib.sha256(self.gate.WRAPPER_BYTES).hexdigest(),
            self.gate.WRAPPER_SHA256,
        )
        self.assertEqual(self.gate.WRAPPER_MODE, 0o600)
        text = self.gate.WRAPPER_BYTES.decode("utf-8")
        self.assertIn("override CCFLAGS += -include cstdint\n", text)
        self.assertNotIn("NCCFLAGS", text)

    def test_static_makefile_derivation_is_131_120_11_without_make(self):
        observed = self.gate.static_make_object_contract(
            Path(self.policy["source_input"]["makefile_path"])
        )
        self.assertEqual(observed["total_object_count"], 131)
        self.assertEqual(observed["cpp_object_count"], 120)
        self.assertEqual(observed["cuda_object_count"], 11)
        self.assertEqual(observed["duplicate_count"], 0)
        self.assertEqual(
            observed["object_names_canonical_sha256"],
            "38023e8b24f1d1731ba3a7d03bbb5fdf2e74e5623341402d607cba046b08d7e2",
        )
        self.assertEqual(observed["object_names"], self.policy["object_contract"]["object_names"])

    def test_profiles_are_deterministic_exact_root_and_no_permission_superset(self):
        hashes = self.gate.verify_profiles(self.policy)
        self.assertEqual(
            hashes["a_copy"],
            "2461550a249284dafc72e473b745ed829463c239e93e805fff623e54a29de8e1",
        )
        self.assertEqual(
            hashes["a_build"],
            "fbcc9f417103f17f59003b6da1b8bb7f08e59b2a68015b7ac38cdf1db8c19b10",
        )
        self.assertEqual(
            hashes["static_audit"],
            "d23da58b35c69f458d7f9a7fcf03b9fc2493283cd580191d58f7ced460b45168",
        )
        build_text = self.gate.repo_path(
            self.policy["profiles"]["attempt_a_build"]["path"]
        ).read_text(encoding="utf-8")
        for item in self.policy["b_admission_contract"]["gxx13_exact_tools"]:
            self.assertNotIn(item["path"], build_text)
        static_text = self.gate.repo_path(
            self.policy["profiles"]["campaign_static_audit"]["path"]
        ).read_text(encoding="utf-8")
        self.assertNotIn("*.o", static_text)
        for name in self.policy["object_contract"]["object_names"]:
            self.assertIn(f"/source/{name} r,", static_text)
        self.assertFalse(
            self.gate.repo_path(
                self.policy["profiles"]["attempt_b_copy"]["planned_path"]
            ).exists()
        )
        self.assertFalse(
            self.gate.repo_path(
                self.policy["profiles"]["attempt_b_build"]["planned_path"]
            ).exists()
        )
        mock_b = (
            f"{self.gate.ROOT_B}/ rw,\n"
            f"{self.gate.ROOT_B}/output/** rw,\n"
        )
        self.gate.validate_exact_root_profile_text(
            mock_b, str(self.gate.ROOT_B), str(self.gate.ROOT_A), "mock B"
        )
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_exact_root_profile_text(
                mock_b + f"{self.gate.ROOT_A}/output/** rw,\n",
                str(self.gate.ROOT_B),
                str(self.gate.ROOT_A),
                "dual-root mock B",
            )

    def test_source_copy_state_machine_accepts_only_exact_order(self):
        trace = valid_source_copy_trace(self.gate, self.policy)
        self.gate.validate_source_copy_trace(trace, self.policy)

        reordered = copy.deepcopy(trace)
        reordered[2], reordered[3] = reordered[3], reordered[2]
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_source_copy_trace(reordered, self.policy)

        wrapper_too_early = copy.deepcopy(trace)
        wrapper_too_early[4]["evidence"]["after_unload_sequence"] = 3
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_source_copy_trace(wrapper_too_early, self.policy)

        receipt_too_early = copy.deepcopy(trace)
        receipt_too_early[6]["evidence"]["after_complete_inventory_sequence"] = 5
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_source_copy_trace(receipt_too_early, self.policy)

    def test_complete_inventory_rejects_extra_symlink_hardlink_elf_execute_and_wrapper_drift(self):
        entries, sealed = valid_inventory(self.gate)
        self.gate.validate_build_input_inventory(entries, sealed)

        cases = []
        extra = copy.deepcopy(entries)
        extra[0]["path"] = "unexpected.extra"
        cases.append(extra)
        symlink = copy.deepcopy(entries)
        symlink[0]["symlink"] = True
        cases.append(symlink)
        hardlink = copy.deepcopy(entries)
        hardlink[0]["nlink"] = 2
        cases.append(hardlink)
        elf = copy.deepcopy(entries)
        elf[0]["is_elf"] = True
        cases.append(elf)
        executable = copy.deepcopy(entries)
        executable[0]["mode_octal"] = "0755"
        cases.append(executable)
        wrapper_mode = copy.deepcopy(entries)
        wrapper_mode[-1]["mode_octal"] = "0644"
        cases.append(wrapper_mode)
        wrapper_hash = copy.deepcopy(entries)
        wrapper_hash[-1]["sha256"] = "f" * 64
        cases.append(wrapper_hash)
        for mutated in cases:
            with self.subTest(mutated=mutated[0].get("path")):
                with self.assertRaises(self.gate.GateError):
                    self.gate.validate_build_input_inventory(mutated, sealed)

    def test_static_audit_argv_is_read_only_disconnected_and_exact(self):
        candidate = self.policy["static_audit_contract"]["candidate_inputs"][0][
            "host_path"
        ]
        argv = self.gate.build_static_audit_argv(candidate, "file", self.policy)
        result = self.gate.validate_static_audit_argv(argv, self.policy)
        self.assertEqual(result["host_input"], candidate)
        self.assertNotIn("--bind", argv)
        self.assertNotIn("--proc", argv)
        self.assertNotIn("--dev", argv)
        self.assertIn("--unshare-net", argv)
        self.assertIn("--clearenv", argv)
        self.assertIn("--kill-after=5s", argv)
        self.assertEqual(argv.count("--ro-bind"), 3)

        missing_net = list(argv)
        missing_net.remove("--unshare-net")
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_static_audit_argv(missing_net, self.policy)

        missing_kill = list(argv)
        missing_kill.remove("--kill-after=5s")
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_static_audit_argv(missing_kill, self.policy)

        writable = list(argv)
        ro_indices = [i for i, token in enumerate(writable) if token == "--ro-bind"]
        writable[ro_indices[-1]] = "--bind"
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_static_audit_argv(writable, self.policy)

        tool_suffix_bypass = list(argv)
        tool_suffix_bypass[tool_suffix_bypass.index("/usr/bin/file")] = "/tmp/file"
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_static_audit_argv(tool_suffix_bypass, self.policy)

        wrong_host = list(argv)
        wrong_host[ro_indices[-1] + 1] += ".extra"
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_static_audit_argv(wrong_host, self.policy)

    def test_static_audit_cuda_object_and_output_limits(self):
        name = self.policy["object_contract"]["cuda_object_names"][0]
        host = (
            f"{self.gate.ROOT_B}/output/buildtree/src/source/{name}"
        )
        argv = self.gate.build_static_audit_argv(
            host, "cuda_object_list_elf", self.policy
        )
        self.gate.validate_static_audit_argv(argv, self.policy)
        self.gate.validate_stream_summary(0, "0" * 64, b"")
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_stream_summary(
                self.gate.STATIC_OUTPUT_LIMIT_BYTES + 1, "0" * 64, b""
            )
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_stream_summary(
                1, "0" * 64, b"x" * (self.gate.STREAM_PREFIX_LIMIT_BYTES + 1)
            )

    def test_candidate_identity_drift_is_rejected(self):
        identity = valid_identity(self.gate)
        self.gate.validate_candidate_identity_unchanged(
            identity, copy.deepcopy(identity), self.policy
        )
        changed = copy.deepcopy(identity)
        changed["st_ino"] += 1
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_candidate_identity_unchanged(
                identity, changed, self.policy
            )

    def test_b_compiler_delta_requires_exact_gxx13_set_only(self):
        document = valid_b_admission(
            self.gate, self.policy, "host_compiler_gxx13"
        )
        result = self.gate.validate_b_admission(
            document, verify_parent_files=False, policy=self.policy
        )
        self.assertEqual(result["selected_delta_id"], "host_compiler_gxx13")
        self.assertIn(
            "CC=/usr/bin/x86_64-linux-gnu-g++-13", document["make_argv"]
        )
        self.assertNotIn(
            "CC=/usr/bin/x86_64-linux-gnu-g++-11", document["make_argv"]
        )

        missing_helper = copy.deepcopy(document)
        missing_helper["permission_delta"]["execute_tools"].pop()
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_b_admission(
                missing_helper, verify_parent_files=False, policy=self.policy
            )

    def test_noncompiler_b_delta_rejects_gxx13_and_cross_attempt_root(self):
        document = valid_b_admission(
            self.gate, self.policy, "cuda_cstdint_preinclude"
        )
        self.gate.validate_b_admission(
            document, verify_parent_files=False, policy=self.policy
        )
        forbidden = copy.deepcopy(document)
        forbidden["permission_delta"]["execute_tools"] = [
            "/usr/bin/x86_64-linux-gnu-g++-13"
        ]
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_b_admission(
                forbidden, verify_parent_files=False, policy=self.policy
            )

        wrong_root = copy.deepcopy(document)
        wrong_root["copy_profile"]["attempt_root"] = str(self.gate.ROOT_A)
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_b_admission(
                wrong_root, verify_parent_files=False, policy=self.policy
            )

    def test_gxx13_helper_hash_drift_and_j1_oom_retry_are_rejected(self):
        observed = {
            item["path"]: item["sha256"]
            for item in self.policy["b_admission_contract"]["gxx13_exact_tools"]
        }
        self.gate.validate_gxx13_helper_identities(observed, self.policy)
        drifted = dict(observed)
        drifted[next(iter(drifted))] = "0" * 64
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_gxx13_helper_identities(drifted, self.policy)
        with self.assertRaises(self.gate.GateError):
            self.gate.expected_b_make_argv(
                "parallel_jobs_oom_j2_to_j1", self.policy
            )

    def test_apparmor_delta_is_single_exact_rule_and_forbidden_prefixes_fail(self):
        document = valid_b_admission(
            self.gate, self.policy, "apparmor_exact_evidence_permission"
        )
        self.gate.validate_b_admission(
            document, verify_parent_files=False, policy=self.policy
        )
        too_many = copy.deepcopy(document)
        too_many["permission_delta"]["apparmor_rules"].append(
            {
                "path": "/usr/local/cuda-12.8/targets/x86_64-linux/lib/exact.a",
                "access": "mr",
            }
        )
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_b_admission(
                too_many, verify_parent_files=False, policy=self.policy
            )
        forbidden = copy.deepcopy(document)
        forbidden["permission_delta"]["apparmor_rules"] = [
            {"path": "/dev/nvidia0", "access": "rw_guest_tmp_only"}
        ]
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_b_admission(
                forbidden, verify_parent_files=False, policy=self.policy
            )
        extra_key = copy.deepcopy(document)
        extra_key["permission_superset"] = True
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_b_admission(
                extra_key, verify_parent_files=False, policy=self.policy
            )


if __name__ == "__main__":
    unittest.main()
