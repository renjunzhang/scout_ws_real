"""Static/mock-only tests for the fresh U3 C1M cold-A settle v5."""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import os
import subprocess
import tempfile
import unittest
from xml.etree import ElementTree as ET
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_gate_v5.py"
HELPER_PATH = PACKAGE_DIR / "scripts/r8_liquid_u3_solver_cpu_settle_cold_a_bootstrap_helper_v5.py"
SUPERVISOR_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_solver_cpu_settle_cold_a_supervisor_v5.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v5.json"
V4_POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v4.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_solver_cpu_settle_cold_a_execution_policy_v5.json"
PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-solver-cpu-settle-cold-a-v5.profile"
HARNESS_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"
REAL_POPEN = subprocess.Popen
REAL_KILLPG = os.killpg


class FakeStream:
    def __init__(self, descriptor: int):
        self.descriptor = descriptor
        self.closed = False

    def fileno(self) -> int:
        return self.descriptor

    def close(self) -> None:
        self.closed = True


def load_module(name: str, path: Path):
    specification = importlib.util.spec_from_file_location(name, path)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class TargetU3SolverCpuSettleColdAV5Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_module("u3_solver_cpu_settle_cold_a_gate_v5_test", GATE_PATH)
        cls.helper = load_module("u3_solver_cpu_settle_cold_a_helper_v5_test", HELPER_PATH)
        cls.supervisor = load_module("u3_solver_cpu_settle_cold_a_supervisor_v5_test", SUPERVISOR_PATH)
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def success_frame(self, *, console: bytes = b"") -> bytes:
        payloads = {path: (f"payload:{path}\n").encode("ascii") for path in self.supervisor.EXPECTED_PATHS}
        payloads["Run.out"] = (
            b"[Simulation finished]\n"
            b"CaseNfixed=0\n"
            b"CaseNmoving=2,669\n"
            b"Shifting=\"NoBound\"\n"
            b"DtAllParticles=True\n"
            b"Finished execution (code=0).\n"
        )
        payloads["data/PartMotionRef.ibi4"] = (
            b"#FileJBD JPartMotRefBi4".ljust(58, b" ") + b"\n\0\0\0\0\0"
        )
        manifest = [
            {"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)}
            for path, raw in sorted(payloads.items())
        ]
        payload = b"".join(payloads[entry["path"]] for entry in manifest)
        inputs = {
            name: {
                "guest_path": guest_path,
                "sha256": digest,
                "size_bytes": size,
                "mode": f"{mode:04o}",
            }
            for name, guest_path, digest, size, mode in self.helper.INPUTS
        }
        label = self.supervisor.BOOTSTRAP_PROFILE + " (enforce)"
        metadata = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_GUEST_FRAME_V5",
            "status": "GUEST_SOLVER_V5_COLD_A_SETTLE_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT",
            "solver_argv": self.gate.SOLVER_ARGV,
            "environment": self.gate.ENVIRONMENT,
            "guest_inputs": inputs,
            "guest_identity": {
                "uid": [0, 0, 0, 0],
                "gid": [0, 0, 0, 0],
                "groups": [],
                "capabilities": {
                    name: 0 for name in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")
                },
                "no_new_privs": 1,
            },
            "guest_label": label,
            "candidate_label": label,
            "guest_work_tmpfs": {
                "mountpoint": "/work",
                "filesystem": "tmpfs",
                "total_bytes": self.gate.TMPFS_BYTES,
                "inode_ceiling_claimed": False,
            },
            "output_audit": {"file_count": 171, "total_bytes": len(payload)},
            "output_manifest": manifest,
            "console": {"sha256": hashlib.sha256(console).hexdigest(), "size_bytes": len(console)},
            "stdin_consumed_to_eof_then_replaced_by_guest_eof_pipe": True,
            "host_writable_bind_count": 0,
        }
        encoded = json.dumps(metadata, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
        header = self.supervisor.OUTPUT_HEADER.pack(
            self.supervisor.OUTPUT_MAGIC,
            self.supervisor.OUTPUT_VERSION,
            len(encoded),
            len(manifest),
            len(payload),
            len(console),
        )
        return header + encoded + payload + console

    def test_policy_schema_artifact_hashes_and_static_status(self):
        Draft202012Validator(self.schema).validate(self.policy)
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(set(self.policy), set(self.schema["required"]))
        for name, path in {
            "gate": GATE_PATH,
            "helper": HELPER_PATH,
            "supervisor": SUPERVISOR_PATH,
            "profile": PROFILE_PATH,
            "schema": SCHEMA_PATH,
        }.items():
            raw = path.read_bytes()
            entry = self.policy["trusted_artifacts"][name]
            self.assertEqual(entry["sha256"], hashlib.sha256(raw).hexdigest())
            self.assertEqual(entry["size_bytes"], len(raw))
        harness = HARNESS_PATH.read_bytes()
        self.assertEqual(self.policy["lifecycle_harness"]["sha256"], hashlib.sha256(harness).hexdigest())
        self.assertEqual(self.policy["lifecycle_harness"]["size_bytes"], len(harness))
        self.assertEqual(
            self.policy["status"],
            self.gate.POLICY_STATUS,
        )
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check", "internal-run"])
        self.gate.verify_review_artifacts(verify_tools=False)

    def test_static_review_never_reports_production_run_allowed(self):
        with mock.patch.object(self.supervisor, "verify_provenance_and_inputs", return_value=({"verified": True}, {})):
            review = self.supervisor.verify_policy_static()
        self.assertNotIn("production_run_allowed", review)
        self.assertTrue(review["cold_a_one_shot_static_admission_ready"])
        self.assertFalse(review["production_authorized"])
        self.assertFalse(review["execution_performed"])

    def test_snapshot_bootstrap_and_loader_sources_compile(self):
        compile(
            self.supervisor.SNAPSHOT_BOOTSTRAP_BYTES,
            "<solver-cpu-settle-cold-a-v5-snapshot-bootstrap-test>",
            "exec",
        )
        compile(
            self.supervisor.SNAPSHOT_BOOTSTRAP_LOADER_BYTES,
            "<solver-cpu-settle-cold-a-v5-snapshot-loader-test>",
            "exec",
        )

    def test_profile_loader_only_rix_and_empty_runtime_label(self):
        effective = self.gate._effective_profile(PROFILE_PATH.read_text(encoding="utf-8"))
        self.assertIn("/work/runtime/ld-linux-x86-64.so.2 rix,", effective)
        self.assertIn("/work/runtime/DualSPHysics5.4CPU_linux64 mr,", effective)
        self.assertNotIn("/work/runtime/DualSPHysics5.4CPU_linux64 rix,", effective)
        self.assertNotIn("rpx", effective.lower())
        runtime = effective.split(f"profile {self.supervisor.RUNTIME_PROFILE}", 1)[1].split("}", 1)[0]
        for token in (" /work", " /usr", " userns ", " mount ", " capability ", " network ", " signal ", " rix,", " mr,", " rw,"):
            self.assertNotIn(token, runtime)
        self.assertNotIn("/dev/nvidia", effective)
        self.assertNotIn("/home/zrj/scout_ws", effective)
        self.gate.verify_profile()

    def test_profile_has_exact_two_bootstrap_quiet_denies_and_rejects_all_variants(self):
        text = PROFILE_PATH.read_text(encoding="utf-8")
        effective = self.gate._effective_profile(text)
        bootstrap, runtime = effective.split(f"profile {self.supervisor.RUNTIME_PROFILE}", 1)
        cache = self.gate.CACHE_QUIET_DENY_RULE
        capability = self.gate.CAPABILITY_QUIET_DENY_RULE
        self.assertEqual(
            [line.strip() for line in bootstrap.splitlines() if "deny " in line],
            list(self.gate.KNOWN_QUIET_DENY_RULES),
        )
        self.assertFalse(any("deny " in line for line in runtime.splitlines()))
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary) / "profile"

            def assert_rejected(mutated: str) -> None:
                profile.write_text(mutated, encoding="utf-8")
                with mock.patch.object(self.gate, "PROFILE_PATH", profile):
                    with self.assertRaises(self.gate.GateError):
                        self.gate.verify_profile()

            assert_rejected(text.replace(f"  {cache}\n", "", 1))
            assert_rejected(text.replace(f"  {capability}\n", "", 1))
            for variant in (
                "/etc/ld.so.cache r,",
                "owner deny /etc/ld.so.cache r,",
                "deny /etc/ld.so.* r,",
                "audit deny /etc/ld.so.cache r,",
                "deny /etc/ld.so.cache rw,",
            ):
                assert_rejected(text.replace(f"  {cache}\n", f"  {variant}\n", 1))
            for variant in (
                "capability dac_override,",
                "owner deny capability dac_override,",
                "audit deny capability dac_override,",
                "deny capability dac_override audit,",
            ):
                assert_rejected(text.replace(f"  {capability}\n", f"  {variant}\n", 1))
            assert_rejected(text.replace(f"  {cache}\n", f"  {cache}\n  /etc/ld.so.cache r,\n", 1))
            assert_rejected(text.replace(f"  {capability}\n", f"  {capability}\n  capability dac_override,\n", 1))
            assert_rejected(text.replace(f"  {capability}\n", f"  {capability}\n  deny /proc/stat r,\n", 1))
            assert_rejected(text.replace(f"  {cache}\n", f"  {cache}\n  deny /etc/ld.so.* r,\n", 1))
            for rule in self.gate.KNOWN_QUIET_DENY_RULES:
                assert_rejected(text.rsplit("}", 1)[0] + f"  {rule}\n}}\n")

    def test_fresh_cold_a_v5_protocol_identity_and_visibility_boundary(self):
        self.assertEqual(self.gate.CASE_ID, "u3_c1m_solver_cpu_settle_cold_a_v5_20260808T203712Z")
        self.assertEqual(
            self.gate.CAMPAIGN_ID,
            "u3_c1m_solver_cpu_settle_ab_campaign_20260808T203712Z",
        )
        self.assertEqual(self.supervisor.CAMPAIGN_ID, self.gate.CAMPAIGN_ID)
        self.assertEqual(self.helper.CAMPAIGN_ID, self.gate.CAMPAIGN_ID)
        self.assertEqual(self.supervisor.ADMISSION_TOKEN, self.gate.ADMISSION_TOKEN)
        self.assertEqual(
            self.policy["frozen_attempt"]["supervisor_admission_token"],
            self.gate.ADMISSION_TOKEN,
        )
        self.assertEqual(self.gate.HELPER_MAGIC, b"R8SOLVERHELPV5\0\0")
        self.assertEqual(self.gate.INPUT_MAGIC, b"R8SOLVERINPUT5\0\0")
        self.assertEqual(self.helper.INPUT_MAGIC, self.gate.INPUT_MAGIC)
        self.assertEqual(self.helper.FRAME_MAGIC, b"R8SOLVEROUTV5\0\0\0")
        self.assertEqual(self.helper.FRAME_VERSION, 5)
        self.assertEqual(self.supervisor.OUTPUT_MAGIC, self.helper.FRAME_MAGIC)
        self.assertEqual(self.supervisor.OUTPUT_VERSION, self.helper.FRAME_VERSION)
        self.assertEqual(
            self.policy["immutable_transport"]["abi_baseline"],
            "cold_a_v4_normalized",
        )
        self.assertEqual(
            self.policy["immutable_transport"]["abi_delta_from_v4"],
            "version_discriminator_only",
        )
        self.assertIn("_COLD_A_V5_", self.gate.SNAPSHOT_ENV)
        self.assertIn("_COLD_A_V5_", self.supervisor.EXPORT_FD_ENV)
        self.assertEqual(
            self.policy["profile_lifecycle"]["denial_visibility_boundary"],
            self.gate.DENIAL_VISIBILITY_BOUNDARY,
        )
        self.assertEqual(
            self.policy["profile_lifecycle"]["known_quiet_deny_rules"],
            list(self.gate.KNOWN_QUIET_DENY_RULES),
        )
        self.assertTrue(self.policy["invariants"]["zero_denied_operations_not_claimed"])

    def test_v5_normalized_abi_and_scientific_contract_equal_v4(self):
        v4 = json.loads(V4_POLICY_PATH.read_text(encoding="utf-8"))
        self.assertEqual(self.policy["input_contract"], v4["input_contract"])
        self.assertEqual(self.policy["resources"], v4["resources"])
        self.assertEqual(self.policy["output_contract"], v4["output_contract"])
        self.assertEqual(
            self.policy["fixed_guest_command"]["solver_argv"],
            v4["fixed_guest_command"]["solver_argv"],
        )
        self.assertEqual(
            self.policy["fixed_guest_command"]["environment"],
            v4["fixed_guest_command"]["environment"],
        )

        def normalized_transport(policy):
            transport = json.loads(json.dumps(policy["immutable_transport"]))
            transport.pop("abi_baseline", None)
            transport.pop("abi_delta_from_v4", None)
            transport["input_frame"]["helper_magic"] = "R8SOLVERHELPVx\\0\\0"
            transport["input_frame"]["input_magic"] = "R8SOLVERINPUTx\\0\\0"
            transport["output_frame"]["magic"] = "R8SOLVEROUTVx\\0\\0\\0"
            transport["output_frame"]["version"] = 0
            return transport

        self.assertEqual(normalized_transport(self.policy), normalized_transport(v4))
        self.assertEqual(
            self.gate.OUTPUT_HEADER_BYTES,
            self.helper.FRAME_HEADER.size,
        )
        self.assertEqual(
            self.helper.FRAME_HEADER.format,
            self.supervisor.OUTPUT_HEADER.format,
        )

        def difference_paths(left, right, prefix=""):
            if type(left) is not type(right):
                return {prefix}
            if isinstance(left, dict):
                differences = set()
                for key in set(left) | set(right):
                    path = f"{prefix}.{key}" if prefix else key
                    if key not in left or key not in right:
                        differences.add(path)
                    else:
                        differences.update(difference_paths(left[key], right[key], path))
                return differences
            return set() if left == right else {prefix}

        expected_policy_delta = {
            "schema_version",
            "document_type",
            "policy_id",
            "status",
            "fixed_guest_command.guest_loader_argv_canonical_sha256",
            "fixed_guest_command.solver_process_group",
            "provenance.consumed_cold_a_v4_attempt",
            "invariants.v4_identity_consumed_and_never_retried",
            "invariants.v4_snapshot_and_runtime_outputs_are_provenance_only",
            "immutable_transport.abi_baseline",
            "immutable_transport.abi_delta_from_v4",
            "immutable_transport.input_frame.helper_magic",
            "immutable_transport.input_frame.input_magic",
            "immutable_transport.output_frame.magic",
            "immutable_transport.output_frame.version",
        }
        expected_policy_delta.update(
            f"frozen_attempt.{key}"
            for key in {
                "case_id",
                "campaign_id",
                "campaign_role",
                "cold_b_identity_allocated",
                "supervisor_admission_token",
                "attempt_root",
                "output_root",
                "start_receipt",
                "execution_receipt",
                "lifecycle_receipt",
                "lifecycle_failure_receipt",
                "console_log",
                "snapshot_root",
                "bootstrap_profile",
                "runtime_profile",
            }
        )
        for artifact in ("gate", "helper", "supervisor", "profile", "schema"):
            expected_policy_delta.add(f"trusted_artifacts.{artifact}.path")
            expected_policy_delta.add(f"trusted_artifacts.{artifact}.sha256")
            self.assertNotEqual(
                v4["trusted_artifacts"][artifact]["sha256"],
                self.policy["trusted_artifacts"][artifact]["sha256"],
            )
            if (
                v4["trusted_artifacts"][artifact]["size_bytes"]
                != self.policy["trusted_artifacts"][artifact]["size_bytes"]
            ):
                expected_policy_delta.add(f"trusted_artifacts.{artifact}.size_bytes")
        self.assertEqual(difference_paths(v4, self.policy), expected_policy_delta)

    def test_exact_171_file_output_contract_and_per_path_limits(self):
        expected_parts = [f"Part_{index:04d}.bi4" for index in range(162)]
        expected_data = [
            "PartInfo.ibi4",
            "PartOut_000.obi4",
            "Part_Head.ibi4",
            "PartMotionRef.ibi4",
            *expected_parts,
        ]
        output = self.policy["output_contract"]
        self.assertEqual(output["exact_file_count"], 171)
        self.assertEqual(output["exact_data_files"], expected_data)
        self.assertEqual(len(self.supervisor.EXPECTED_ROOT_FILES), 5)
        self.assertEqual(len(self.supervisor.EXPECTED_DATA_FILES), 166)
        self.assertEqual(len(self.supervisor.EXPECTED_PATHS), 171)
        self.assertIn("data/Part_0000.bi4", self.supervisor.EXPECTED_PATHS)
        self.assertIn("data/Part_0161.bi4", self.supervisor.EXPECTED_PATHS)
        self.assertNotIn("data/Part_0162.bi4", self.supervisor.EXPECTED_PATHS)
        self.assertEqual(self.helper.EXPECTED_FILE_COUNT, 171)
        self.assertEqual(self.helper.EXPECTED_EXACT_FILE_SIZES["CfgInit_Domain.vtk"], 1_010)
        self.assertEqual(self.helper.EXPECTED_EXACT_FILE_SIZES["data/PartInfo.ibi4"], 115_257)
        self.assertEqual(self.helper.EXPECTED_EXACT_FILE_SIZES["data/Part_0161.bi4"], 401_318)
        self.assertEqual(self.helper.TEXT_FILE_LIMIT, 65_536)
        self.assertEqual(self.helper.OUTPUT_TOTAL_LIMIT, 83_886_080)
        self.assertEqual(self.supervisor.OUTPUT_FRAME_LIMIT, 88_342_576)

    def test_resource_timeout_gradient_is_frozen(self):
        resources = self.policy["resources"]
        self.assertEqual(
            (
                resources["cpu_seconds"],
                resources["cpu_hard_seconds"],
                resources["guest_runtime_timeout_seconds"],
                resources["bwrap_timeout_seconds"],
                resources["gate_conduit_timeout_seconds"],
                resources["outer_wall_timeout_seconds"],
            ),
            (10_200, 10_210, 10_710, 10_770, 10_790, 10_800),
        )
        self.assertEqual(self.helper.RUNTIME_TIMEOUT_SECONDS, 10_710)
        self.assertEqual(self.gate.BWRAP_TIMEOUT_SECONDS, 10_770)
        self.assertEqual(self.gate.CONDUIT_TIMEOUT_SECONDS, 10_790)
        self.assertEqual(self.supervisor.OUTER_WALL_TIMEOUT_SECONDS, 10_800)
        self.assertEqual(self.supervisor.EXPORT_TIMEOUT_SECONDS, 300)
        self.assertEqual(resources["address_space_bytes"], 2_147_483_648)
        self.assertEqual(resources["guest_work_tmpfs_bytes"], 268_435_456)
        self.assertEqual(resources["file_size_limit_bytes"], 16_777_216)
        self.assertEqual(resources["guest_console_limit_bytes"], 4_194_304)

    def test_v3_is_consumed_provenance_only_and_cannot_be_retried(self):
        predecessor = self.policy["provenance"]["completed_c1m_v3_smoke"]
        self.assertEqual(predecessor, self.gate.V3_COMPLETION_PROVENANCE)
        self.assertEqual(predecessor["case_id"], self.supervisor.V3_CASE_ID)
        self.assertNotEqual(self.gate.CASE_ID, predecessor["case_id"])
        self.assertTrue(predecessor["identity_consumed"])
        self.assertTrue(predecessor["retry_forbidden"])
        self.assertEqual(predecessor["source_use"], "provenance_only_no_v3_solver_output_reuse")
        self.assertFalse(predecessor["output_qc"]["duration_eligible_for_settle_qc"])
        self.assertFalse(predecessor["output_qc"]["numeric_settle_qc_pass"])
        self.assertFalse(predecessor["output_qc"]["settled_state_freeze_eligible"])
        observed = self.supervisor.verify_v3_completion_provenance()
        self.assertTrue(observed["identity_consumed"])
        input_sources = [source for _name, source, _digest, _size in self.supervisor.INPUT_SOURCES]
        self.assertFalse(any(self.supervisor.V3_CASE_ID in source for source in input_sources))

    def test_cold_a_failure_stops_b_and_success_still_requires_separate_qc(self):
        invariants = self.policy["invariants"]
        self.assertTrue(invariants["cold_a_is_fresh_identity"])
        self.assertTrue(invariants["cold_a_failure_forbids_cold_b_policy_creation_or_execution"])
        self.assertTrue(invariants["cold_b_admission_requires_cold_a_lifecycle_and_separate_output_qc_pass"])
        self.assertTrue(invariants["restart_policy_requires_cold_a_and_cold_b_lifecycle_and_qc_pass"])
        self.assertFalse(invariants["settled_state_authorized"])
        self.assertFalse(invariants["u4_authorized"])
        self.assertEqual(
            self.policy["next_allowed_stage"],
            "SEPARATE_COLD_A_OUTPUT_QC_THEN_IF_PASS_FRESH_COLD_B_ADMISSION",
        )
        supervisor_source = SUPERVISOR_PATH.read_text(encoding="utf-8")
        self.assertIn('"next_allowed_stage": "STOP_COLD_A_FAILED_NO_COLD_B_ADMISSION"', supervisor_source)
        self.assertNotIn("U4_", self.policy["status"])

    def test_fresh_identity_paths_and_start_receipt_are_one_shot(self):
        attempt = self.policy["frozen_attempt"]
        self.assertEqual(attempt["case_id"], self.gate.CASE_ID)
        self.assertEqual(attempt["campaign_id"], self.gate.CAMPAIGN_ID)
        self.assertEqual(attempt["campaign_role"], "cold_a")
        self.assertFalse(attempt["cold_b_identity_allocated"])
        self.assertEqual(attempt["attempts_per_identity"], 1)
        self.assertEqual(attempt["same_identity_retry"], "forbidden")
        self.assertIn(self.gate.CASE_ID, attempt["start_receipt"])
        self.assertIn(self.gate.CASE_ID, attempt["snapshot_root"])
        tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
        run_once = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "run_once")
        start_writes = [
            node
            for node in ast.walk(run_once)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "write_json_new"
            and node.args
            and ast.unparse(node.args[0]) == "START_RECEIPT"
        ]
        self.assertEqual(len(start_writes), 1)

    def test_all_v5_receipts_freeze_campaign_and_cold_a_role(self):
        tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
        run_once = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "run_once"
        )
        documents = {}
        for node in ast.walk(run_once):
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if isinstance(target, ast.Name) and target.id in {
                "start_document",
                "execution_document",
                "lifecycle_document",
                "failure_document",
            }:
                documents[target.id] = node.value
        self.assertEqual(
            set(documents),
            {"start_document", "execution_document", "lifecycle_document", "failure_document"},
        )
        for name, document in documents.items():
            with self.subTest(document=name):
                self.assertIsInstance(document, ast.Dict)
                values = {
                    key.value: value
                    for key, value in zip(document.keys, document.values, strict=True)
                    if isinstance(key, ast.Constant) and isinstance(key.value, str)
                }
                self.assertEqual(ast.unparse(values["campaign_id"]), "CAMPAIGN_ID")
                self.assertEqual(ast.literal_eval(values["campaign_role"]), "cold_a")

    def test_consumed_v4_failure_is_exact_and_never_reused(self):
        predecessor = self.policy["provenance"]["consumed_cold_a_v4_attempt"]
        self.assertEqual(predecessor, self.gate.V4_FAILURE_PROVENANCE)
        self.assertEqual(predecessor["case_id"], self.supervisor.V4_CASE_ID)
        self.assertTrue(predecessor["identity_consumed"])
        self.assertTrue(predecessor["retry_forbidden"])
        self.assertFalse(predecessor["output_exported"])
        self.assertFalse(predecessor["cold_b_admission_authorized"])
        self.assertFalse(predecessor["settled_state_authorized"])
        self.assertFalse(predecessor["u4_authorized"])
        self.assertEqual(predecessor["observed_failure"]["gate_returncode"], 2)
        self.assertEqual(
            predecessor["observed_failure"]["stderr_sha256"],
            "cef2979e52eca41b2921183cfa675add76e9a3d95c16ec4e32ed0b9fb167d460",
        )
        observed = self.supervisor.verify_v4_failure_provenance()
        self.assertEqual(observed["case_id"], self.supervisor.V4_CASE_ID)
        self.assertTrue(observed["identity_consumed"])
        input_sources = [source for _name, source, _digest, _size in self.supervisor.INPUT_SOURCES]
        snapshot_sources = [source for source, _name in self.supervisor.SNAPSHOT_SOURCE_PAIRS]
        for source in input_sources + snapshot_sources:
            self.assertNotIn(self.supervisor.V4_CASE_ID, source)
            self.assertNotIn("cold_a_v4", source)
            self.assertNotIn("/run/r8-liquid-", source)
        invariants = self.policy["invariants"]
        self.assertTrue(invariants["v4_identity_consumed_and_never_retried"])
        self.assertTrue(invariants["v4_snapshot_and_runtime_outputs_are_provenance_only"])

    def test_consumed_v1_receipts_are_exact_and_retry_forbidden(self):
        predecessor = self.policy["provenance"]["consumed_v1_attempt"]
        self.assertEqual(predecessor["case_id"], self.gate.V1_CASE_ID)
        self.assertTrue(predecessor["identity_consumed"])
        self.assertTrue(predecessor["retry_forbidden"])
        self.assertFalse(predecessor["output_exported"])
        self.assertEqual(predecessor["receipts"], self.gate.V1_PREDECESSOR_RECEIPTS)
        for receipt in predecessor["receipts"].values():
            raw = Path(receipt["path"]).read_bytes()
            self.assertEqual(len(raw), receipt["size_bytes"])
            self.assertEqual(hashlib.sha256(raw).hexdigest(), receipt["sha256"])
            self.assertEqual(json.loads(raw)["status"], receipt["status"])

    def test_seed_v3_receipt_closes_direct_dsphconfig_provenance(self):
        receipt = self.policy["provenance"]["gencase_seed_v3_receipt"]
        self.assertEqual(receipt, self.gate.SEED_RECEIPT_PROVENANCE)
        raw = Path(receipt["path"]).read_bytes()
        self.assertEqual(len(raw), receipt["size_bytes"])
        self.assertEqual(hashlib.sha256(raw).hexdigest(), receipt["sha256"])
        value = json.loads(raw)
        self.assertEqual(value["status"], receipt["status"])
        self.assertEqual(value["seed_id"], receipt["seed_id"])
        for flag in (
            "compiled_artifact_executed",
            "precompiled_binary_executed",
            "upstream_code_executed",
            "network_used",
            "gpu_device_exposed",
            "sudo_used",
            "system_packages_changed",
            "source_checkout_created",
            "predecessor_seed_used_as_source",
        ):
            self.assertIs(value[flag], False)
        self.assertEqual(
            value["seed_input"]["files"]["DsphConfig.xml"],
            {
                "mode": "0400",
                "sha256": "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd",
                "size_bytes": 293,
            },
        )
        observed = self.supervisor.verify_seed_receipt()
        self.assertEqual(observed["seed_id"], self.gate.SEED_ID)
        self.assertTrue(observed["non_execution_flags_verified"])

        with tempfile.TemporaryDirectory() as temporary:
            missing = Path(temporary) / "missing-seed-receipt.json"
            with mock.patch.object(self.supervisor, "SEED_RECEIPT", missing):
                with self.assertRaises(self.supervisor.SupervisorError):
                    self.supervisor.verify_seed_receipt()
            tampered = Path(temporary) / "tampered-seed-receipt.json"
            tampered.write_bytes(raw + b"\n")
            with mock.patch.object(self.supervisor, "SEED_RECEIPT", tampered):
                with self.assertRaisesRegex(
                    self.supervisor.SupervisorError, "byte identity differs"
                ):
                    self.supervisor.verify_seed_receipt()

    def test_completed_c1_v2_receipts_are_exact_and_not_reused_as_input(self):
        predecessor = self.policy["provenance"]["completed_c1_v2_attempt"]
        self.assertEqual(predecessor["case_id"], self.gate.V2_CASE_ID)
        self.assertTrue(predecessor["identity_consumed"])
        self.assertTrue(predecessor["retry_forbidden"])
        self.assertTrue(predecessor["output_exported"])
        self.assertEqual(predecessor["receipts"], self.gate.V2_PREDECESSOR_RECEIPTS)
        for receipt in predecessor["receipts"].values():
            raw = Path(receipt["path"]).read_bytes()
            self.assertEqual(len(raw), receipt["size_bytes"])
            self.assertEqual(hashlib.sha256(raw).hexdigest(), receipt["sha256"])
            self.assertEqual(json.loads(raw)["status"], receipt["status"])
        input_files = self.policy["input_contract"]["files"]
        self.assertIn("C1M_zero.xml", input_files)
        self.assertIn("C1M_zero.bi4", input_files)
        self.assertNotIn("C1_static.xml", input_files)
        self.assertNotIn("C1_static.bi4", input_files)

    def test_audit_claim_is_zero_unexpected_logged_not_zero_denied_operations(self):
        audit = {
            "capture_valid": True,
            "capture_errors": [],
            "matching_total": 0,
            "stored_count": 0,
            "dropped_count": 0,
            "storage_overflow": False,
            "unexpected_total": 0,
            "sanitized_denials": [],
            "start_cursor": "s=1",
            "end_cursor": "s=2",
            "boot_id_before": "boot",
            "boot_id_after": "boot",
            "denial_accounting": self.supervisor.denial_accounting_evidence(),
        }
        self.supervisor.require_closed_zero_unexpected_logged_denial_audit(audit)
        self.assertFalse(audit["denial_accounting"]["zero_denied_operations_claimed"])
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.require_closed_zero_unexpected_logged_denial_audit({**audit, "matching_total": 1})

    def test_bwrap_has_one_256m_tmpfs_and_no_writable_bind_device_or_gpu(self):
        helper = self.policy["trusted_artifacts"]["helper"]
        argv = self.gate.bwrap_argv(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
        self.assertEqual(self.gate._pairs(argv, "--ro-bind"), [["/usr", "/usr"]])
        for forbidden in ("--bind", "--bind-fd", "--file", "--dev", "--dev-bind", "--share-net", "/dev", "/sys"):
            self.assertNotIn(forbidden, argv)
        self.assertEqual(argv.count("--tmpfs"), 1)
        index = argv.index("--size")
        self.assertEqual(argv[index : index + 4], ["--size", "268435456", "--tmpfs", "/work"])
        self.assertIn("--unshare-net", argv)
        self.assertEqual(argv[argv.index("--cap-drop") + 1], "ALL")
        self.assertEqual(self.policy["isolation"]["host_writable_bind_count"], 0)
        self.assertEqual(self.policy["isolation"]["gpu_device_nodes"], [])

    def test_solver_argv_is_cpu_single_thread_8_05_seconds_and_stable(self):
        argv = self.gate.SOLVER_ARGV
        for required in ("-cpu", "-ompthreads:1", "-stable:1", "-vres:0", "-cellmode:full", "-tmax:8.05", "-tout:0.05"):
            self.assertEqual(argv.count(required), 1)
        self.assertNotIn("-gpu", argv)
        self.assertEqual(argv, self.helper.SOLVER_ARGV)
        self.assertEqual(self.gate.ENVIRONMENT["OMP_NUM_THREADS"], "1")
        self.assertEqual(self.policy["fixed_guest_command"]["solver_argv"], argv)

    def test_c1m_case_has_only_zero_motion_moving_boundary_semantics(self):
        self.assertEqual(
            self.policy["input_contract"]["c1m_case_semantics"],
            self.gate.C1M_CASE_SEMANTICS,
        )
        sources = {name: Path(source) for name, source, _digest, _size in self.supervisor.INPUT_SOURCES}
        root = ET.fromstring(sources["C1M_zero.xml"].read_bytes())
        particles = root.find("./execution/particles")
        self.assertIsNotNone(particles)
        assert particles is not None
        self.assertIsNone(particles.find("./fixed"))
        moving = particles.findall("./moving")
        fluid = particles.findall("./fluid")
        self.assertEqual([item.attrib for item in moving], [
            {"mkbound": "0", "mk": "2", "begin": "0", "count": "2669", "refmotion": "0"}
        ])
        self.assertEqual([item.attrib for item in fluid], [
            {"mkfluid": "0", "mk": "1", "begin": "2669", "count": "6409"}
        ])
        motions = root.findall(".//motion")
        self.assertEqual(len(motions), 2)
        for motion in motions:
            self.assertEqual(motion.attrib, {})
            self.assertEqual(len(motion), 1)
            objreal = motion[0]
            self.assertEqual((objreal.tag, objreal.attrib), ("objreal", {"ref": "0"}))
            self.assertEqual(
                [(child.tag, child.attrib) for child in objreal],
                [("begin", {"mov": "1", "start": "0"}), ("mvnull", {"id": "1"})],
            )
        parameters = {
            item.attrib.get("key"): item.attrib.get("value")
            for item in root.findall("./execution/parameters/parameter")
        }
        self.assertEqual(parameters["Shifting"], "1")
        self.assertEqual(parameters["DtAllParticles"], "1")
        self.assertIn("data/PartMotionRef.ibi4", self.supervisor.EXPECTED_PATHS)
        self.assertEqual(len(self.supervisor.EXPECTED_PATHS), 171)

    def test_fixed_input_frame_has_exact_order_eof_and_rejects_tamper(self):
        helper = b"reviewed-helper"
        contract = (
            ("first", hashlib.sha256(b"A").hexdigest(), 1),
            ("second", hashlib.sha256(b"BC").hexdigest(), 2),
        )
        temporary_policy = {
            "trusted_artifacts": {
                "helper": {"sha256": hashlib.sha256(helper).hexdigest(), "size_bytes": len(helper)}
            }
        }
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "policy.json"
            path.write_text(json.dumps(temporary_policy), encoding="utf-8")
            with mock.patch.object(self.gate, "POLICY_PATH", path), mock.patch.object(self.gate, "INPUT_CONTRACT", contract):
                frame = self.gate.build_input_frame(helper, {"first": b"A", "second": b"BC"})
                self.assertEqual(frame, self.gate.HELPER_MAGIC + helper + self.gate.INPUT_MAGIC + b"ABC")
                with self.assertRaises(self.gate.GateError):
                    self.gate.build_input_frame(helper, {"first": b"A", "second": b"BD"})
                with self.assertRaises(self.gate.GateError):
                    self.gate.build_input_frame(helper, {"first": b"A"})

    def test_success_frame_exact_171_files_and_tamper_rejection(self):
        frame = self.success_frame(console=b"bounded console")
        parsed = self.supervisor.parse_success_frame(frame)
        self.assertEqual(parsed["frame_size_bytes"], len(frame))
        self.assertEqual(tuple(parsed["payloads"]), self.supervisor.EXPECTED_PATHS)
        self.assertEqual(parsed["console"], b"bounded console")
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(frame + b"x")
        changed = bytearray(frame)
        metadata_size = self.supervisor.OUTPUT_HEADER.unpack_from(frame)[2]
        changed[self.supervisor.OUTPUT_HEADER.size + metadata_size] ^= 1
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(bytes(changed))

    def test_guest_auditor_requires_c1m_counts_and_motion_reference_header(self):
        run_csv = (
            b"#Np;PhysicalTime;PartFiles;PartsOut;Nbound;Nfixed\n"
            b"9,078;8.050039;162;0;2,669;0\n"
        )
        self.helper._audit_run_csv(run_csv)
        with self.assertRaises(self.helper.GuestError):
            self.helper._audit_run_csv(run_csv.replace(b";0\n", b";2669\n"))
        run_out = (
            b"DualSPHysics5 v5.4.355\n"
            b"[Simulation finished]\n"
            b"Particles of simulation (initial):\n"
            b"CaseNfixed=0\n"
            b"CaseNmoving=2,669\n"
            b"Excluded particles...............: 0\n"
            b"Finished execution (code=0).\n"
        )
        self.helper._audit_run_out(run_out)
        with self.assertRaises(self.helper.GuestError):
            self.helper._audit_run_out(run_out.replace(b"CaseNmoving=2,669", b"CaseNmoving=0"))
        valid_header = b"#FileJBD JPartMotRefBi4".ljust(58, b" ") + b"\n\0\0\0\0\0"
        self.helper._audit_jbd_header("data/PartMotionRef.ibi4", valid_header)
        with self.assertRaises(self.helper.GuestError):
            self.helper._audit_jbd_header(
                "data/PartMotionRef.ibi4",
                b"#FileJBD JPartDataBi4".ljust(58, b" ") + b"\n\0\0\0\0\0",
            )

    def test_guest_runparts_requires_exact_162_frame_time_coverage(self):
        header = (
            "Part;TimeStep [s];NpSave;NpSim;NpOut;NpbSim;NpfSim;"
            "NpNormal;NpOutPos;NpOutRho;NpOutMov\n"
        )
        rows = [
            f"{part};{part * 0.05:.2f};9078;9078;0;2669;6409;9078;0;0;0\n"
            for part in range(162)
        ]
        raw = (header + "".join(rows)).encode("ascii")
        audit = self.helper._audit_runparts(raw)
        self.assertEqual(audit["rows"], 162)
        self.assertEqual(audit["first_time"], 0.0)
        self.assertEqual(audit["final_time"], 8.05)
        with self.assertRaises(self.helper.GuestError):
            self.helper._audit_runparts((header + "".join(rows[:-1])).encode("ascii"))

    def test_uid1000_o_excl_export_preserves_zero_byte_console_and_reaudits_it(self):
        frame = self.success_frame(console=b"")
        parsed = self.supervisor.parse_success_frame(frame)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cases = root / "cases"
            audits = root / "audits"
            cases.mkdir()
            audits.mkdir()
            attempt = cases / "attempt.partial"
            output = attempt / "output"
            console = audits / "console.log"
            with (
                mock.patch.object(self.supervisor, "ATTEMPT_ROOT", attempt),
                mock.patch.object(self.supervisor, "OUTPUT_ROOT", output),
                mock.patch.object(self.supervisor, "CONSOLE_LOG", console),
                mock.patch.object(self.supervisor, "HOST_UID", os.geteuid()),
                mock.patch.object(self.supervisor, "HOST_GID", os.getegid()),
                mock.patch.object(self.supervisor, "verify_uid1000_nnp", return_value={"verified": True}),
                mock.patch.object(self.supervisor, "consume_capability", return_value={"verified": True}),
            ):
                exported = self.supervisor.export_frame_o_excl(frame)
                self.assertEqual(console.read_bytes(), b"")
                self.assertEqual(exported["console_log"]["size_bytes"], 0)
                self.assertEqual(exported["console_log"]["sha256"], hashlib.sha256(b"").hexdigest())
                verified = self.supervisor.verify_exported_outputs(parsed)
                self.assertEqual(verified["file_count"], 171)
                self.assertEqual(verified["console_log"]["size_bytes"], 0)
                console.chmod(0o640)
                console.write_bytes(b"tampered")
                with self.assertRaises(self.supervisor.SupervisorError):
                    self.supervisor.verify_exported_outputs(parsed)
                with self.assertRaises(self.supervisor.SupervisorError):
                    self.supervisor.export_frame_o_excl(frame)

    def test_host_identity_contract_fails_closed(self):
        valid = {
            "uid": [1000] * 4,
            "gid": [1000] * 4,
            "groups": [],
            "capabilities": {name: 0 for name in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")},
            "no_new_privs": 1,
        }
        self.assertEqual(self.gate.verify_child_identity(valid), valid)
        for changed in (
            {**valid, "groups": [27]},
            {**valid, "no_new_privs": 0},
            {**valid, "capabilities": {**valid["capabilities"], "CapAmb": 1}},
            {**valid, "uid": [1000, 1000, 0, 1000]},
        ):
            with self.assertRaises(self.gate.GateError):
                self.gate.verify_child_identity(changed)

    def test_run_once_passes_gate_handoff_argv_once(self):
        tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
        run_once = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "run_once")
        calls = [
            node
            for node in ast.walk(run_once)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "run_bounded_command"
            and node.args
            and ast.unparse(node.args[0]) == "gate_handoff_argv()"
        ]
        self.assertEqual(len(calls), 1)
        self.assertEqual(len(calls[0].args), 1)

    def test_real_guest_pipe_eof_grace_preserves_output_without_signal(self):
        process_holder = {}

        def spawn(*args, **kwargs):
            process = REAL_POPEN(*args, **kwargs)
            original_wait = process.wait
            process.wait = mock.Mock(wraps=original_wait)
            process_holder["process"] = process
            return process

        child = (
            "import os,sys,time\n"
            "data=sys.stdin.buffer.read()\n"
            "os.write(1,b'kept:'+data)\n"
            "os.close(1)\n"
            "os.close(2)\n"
            "time.sleep(0.15)\n"
        )
        with (
            mock.patch.object(self.gate.subprocess, "Popen", side_effect=spawn),
            mock.patch.object(self.gate.os, "killpg", wraps=REAL_KILLPG) as killpg,
        ):
            returncode, stdout, stderr = self.gate.run_bounded_guest(
                ["/usr/bin/python3.12", "-I", "-B", "-S", "-c", child],
                b"frame-v5",
            )
        self.assertEqual(returncode, 0)
        self.assertEqual(stdout, b"kept:frame-v5")
        self.assertEqual(stderr, b"")
        killpg.assert_not_called()
        self.assertEqual(process_holder["process"].wait.call_count, 1)

    def test_empty_guest_input_closes_stdin_instead_of_spinning(self):
        returncode, stdout, stderr = self.gate.run_bounded_guest(
            [
                "/usr/bin/python3.12",
                "-I",
                "-B",
                "-S",
                "-c",
                "import sys; sys.stdout.buffer.write(sys.stdin.buffer.read())",
            ],
            b"",
        )
        self.assertEqual((returncode, stdout, stderr), (0, b"", b""))

    def test_group_cleanup_reaps_sole_leader_zombie_without_signal(self):
        for module in (self.gate, self.helper):
            with self.subTest(module=module.__name__):
                identity = module.ProcessGroupIdentity(4100, 4100, 4100, 77, 88)
                leader = module.ProcessMember(4100, "Z", 4100, 4100, 77)
                process = mock.Mock()
                with (
                    mock.patch.object(module, "_group_members", return_value=[leader]),
                    mock.patch.object(module, "_signal_owned_group") as send_signal,
                    mock.patch.object(module, "_reap_owned_leader", return_value=0) as reap,
                ):
                    self.assertEqual(module._stop_group(process, identity), 0)
                send_signal.assert_not_called()
                reap.assert_called_once_with(process, identity)

    def test_group_cleanup_term_is_enough_after_leader_becomes_zombie(self):
        for module in (self.gate, self.helper):
            with self.subTest(module=module.__name__):
                identity = module.ProcessGroupIdentity(4200, 4200, 4200, 78, 89)
                leader = module.ProcessMember(4200, "S", 4200, 4200, 78)
                process = mock.Mock()
                with (
                    mock.patch.object(module, "_group_members", return_value=[leader]),
                    mock.patch.object(module, "_signal_owned_group") as send_signal,
                    mock.patch.object(module, "_wait_only_owned_leader_zombie", return_value=True),
                    mock.patch.object(module, "_reap_owned_leader", return_value=-15) as reap,
                ):
                    self.assertEqual(module._stop_group(process, identity), -15)
                self.assertEqual(
                    send_signal.call_args_list,
                    [mock.call(identity, module.signal.SIGTERM)],
                )
                reap.assert_called_once_with(process, identity)

    def test_live_or_zombie_descendant_forces_group_signal_before_reap(self):
        for module in (self.gate, self.helper):
            for child_state in ("S", "Z"):
                with self.subTest(module=module.__name__, child_state=child_state):
                    identity = module.ProcessGroupIdentity(4300, 4300, 4300, 79, 90)
                    members = [
                        module.ProcessMember(4300, "Z", 4300, 4300, 79),
                        module.ProcessMember(4301, child_state, 4300, 4300, 80),
                    ]
                    process = mock.Mock()
                    with (
                        mock.patch.object(module, "_group_members", return_value=members),
                        mock.patch.object(module, "_signal_owned_group") as send_signal,
                        mock.patch.object(
                            module,
                            "_wait_only_owned_leader_zombie",
                            side_effect=[False, True],
                        ),
                        mock.patch.object(module, "_reap_owned_leader", return_value=-9) as reap,
                    ):
                        self.assertEqual(module._stop_group(process, identity), -9)
                    self.assertEqual(
                        send_signal.call_args_list,
                        [
                            mock.call(identity, module.signal.SIGTERM),
                            mock.call(identity, module.signal.SIGKILL),
                        ],
                    )
                    reap.assert_called_once_with(process, identity)

    def test_identity_change_refuses_killpg(self):
        for module in (self.gate, self.helper):
            error_type = self.gate.GateError if module is self.gate else self.helper.GuestError
            identity = module.ProcessGroupIdentity(4400, 4400, 4400, 81, 91)
            baseline = module.ProcessMember(4400, "S", 4400, 4400, 81)
            for field, value in (("pgrp", 4401), ("session", 4401), ("starttime", 82)):
                with self.subTest(module=module.__name__, field=field):
                    changed = baseline._replace(**{field: value})
                    with (
                        mock.patch.object(module, "_read_process_member", side_effect=[changed, changed]),
                        mock.patch.object(module.os, "killpg") as killpg,
                    ):
                        with self.assertRaises(error_type):
                            module._signal_owned_group(identity, module.signal.SIGTERM)
                    killpg.assert_not_called()

    def test_reap_requires_final_empty_process_group(self):
        for module in (self.gate, self.helper):
            with self.subTest(module=module.__name__):
                error_type = self.gate.GateError if module is self.gate else self.helper.GuestError
                identity = module.ProcessGroupIdentity(4500, 4500, 4500, 83, 92)
                leader = module.ProcessMember(4500, "Z", 4500, 4500, 83)
                residue = module.ProcessMember(4501, "S", 4500, 4500, 84)
                process = mock.Mock()
                process.wait.return_value = 0
                process.poll.return_value = 0
                with mock.patch.object(
                    module, "_group_members", side_effect=[[leader], [residue]]
                ):
                    with self.assertRaises(error_type) as caught:
                        module._reap_owned_leader(process, identity)
                self.assertIn("remains after leader reap", str(caught.exception))
                process.wait.assert_called_once_with()

    def test_primary_errors_survive_successful_or_failed_cleanup(self):
        for module, error_type, prefix in (
            (self.gate, self.gate.GateError, "outer guest cleanup failure"),
            (self.helper, self.helper.GuestError, "solver cleanup failure"),
        ):
            for message in (
                "solver guest conduit exceeded 10790 seconds",
                "guest stdout exceeded hard byte ceiling",
                "guest stderr exceeded hard byte ceiling",
            ):
                with self.subTest(module=module.__name__, message=message, cleanup="clean"):
                    with self.assertRaises(error_type) as caught:
                        module._raise_primary_or_cleanup(error_type(message), None)
                    self.assertEqual(str(caught.exception), message)
                with self.subTest(module=module.__name__, message=message, cleanup="failed"):
                    with self.assertRaises(error_type) as caught:
                        module._raise_primary_or_cleanup(
                            error_type(message), error_type("cleanup-boom")
                        )
                    self.assertIn(message, str(caught.exception))
                    self.assertIn(prefix, str(caught.exception))
                    self.assertIn("cleanup-boom", str(caught.exception))

    def test_gate_deadline_and_stream_overflow_always_cleanup_and_keep_primary(self):
        class OneEventSelector:
            def __init__(self, channel: str | None):
                self.channel = channel
                self.closed = False

            def register(self, _descriptor, _events, _name):
                return None

            def get_map(self):
                return {1: 1}

            def select(self, timeout):
                if self.channel is None:
                    return []
                descriptor = 102 if self.channel == "stdout" else 103
                return [(mock.Mock(fd=descriptor, data=self.channel), 1)]

            def close(self):
                self.closed = True

        cases = (
            ("deadline", None, "solver guest conduit exceeded 0 seconds"),
            ("stdout", "stdout", "guest stdout exceeded hard byte ceiling"),
            ("stderr", "stderr", "guest stderr exceeded hard byte ceiling"),
        )
        for name, channel, primary_message in cases:
            for cleanup_fails in (False, True):
                with self.subTest(case=name, cleanup_fails=cleanup_fails):
                    process = mock.Mock()
                    process.pid = 4550
                    process.stdin = FakeStream(101)
                    process.stdout = FakeStream(102)
                    process.stderr = FakeStream(103)
                    identity = self.gate.ProcessGroupIdentity(4550, 4550, 4550, 84, 96)
                    selector = OneEventSelector(channel)
                    cleanup_effect = (
                        self.gate.GateError("cleanup-boom") if cleanup_fails else None
                    )
                    patches = (
                        mock.patch.object(self.gate.subprocess, "Popen", return_value=process),
                        mock.patch.object(self.gate, "_capture_process_group", return_value=identity),
                        mock.patch.object(self.gate, "_preflight_group_scan", return_value=[]),
                        mock.patch.object(self.gate.os, "set_blocking"),
                        mock.patch.object(self.gate.selectors, "DefaultSelector", return_value=selector),
                        mock.patch.object(self.gate.os, "read", return_value=b"12345"),
                        mock.patch.object(self.gate.os, "close"),
                        mock.patch.object(self.gate, "GUEST_STDOUT_LIMIT", 3),
                        mock.patch.object(self.gate, "GUEST_STDERR_LIMIT", 3),
                        mock.patch.object(self.gate, "CONDUIT_TIMEOUT_SECONDS", 0 if name == "deadline" else 100),
                    )
                    with patches[0], patches[1], patches[2], patches[3], patches[4], patches[5], patches[6], patches[7], patches[8], patches[9]:
                        with mock.patch.object(
                            self.gate,
                            "_stop_group",
                            side_effect=cleanup_effect,
                            return_value=-9,
                        ) as stop_group:
                            with self.assertRaises(self.gate.GateError) as caught:
                                self.gate.run_bounded_guest(["ignored"], b"input")
                    message = str(caught.exception)
                    self.assertIn(primary_message, message)
                    if cleanup_fails:
                        self.assertIn("outer guest cleanup failure", message)
                        self.assertIn("cleanup-boom", message)
                    else:
                        self.assertEqual(message, primary_message)
                    stop_group.assert_called_once_with(process, identity)
                    self.assertTrue(selector.closed)

    def test_selector_setup_exception_still_reaps_and_closes_gate_resources(self):
        process = mock.Mock()
        process.pid = 4600
        process.stdin = FakeStream(101)
        process.stdout = FakeStream(102)
        process.stderr = FakeStream(103)
        identity = self.gate.ProcessGroupIdentity(4600, 4600, 4600, 85, 93)
        with (
            mock.patch.object(self.gate.subprocess, "Popen", return_value=process),
            mock.patch.object(self.gate, "_capture_process_group", return_value=identity),
            mock.patch.object(self.gate, "_preflight_group_scan", return_value=[]),
            mock.patch.object(self.gate.os, "set_blocking"),
            mock.patch.object(
                self.gate.selectors,
                "DefaultSelector",
                side_effect=RuntimeError("selector-setup-boom"),
            ),
            mock.patch.object(self.gate, "_stop_group", return_value=-9) as stop_group,
            mock.patch.object(self.gate.os, "close") as close_fd,
        ):
            with self.assertRaisesRegex(RuntimeError, "selector-setup-boom"):
                self.gate.run_bounded_guest(["ignored"], b"input")
        stop_group.assert_called_once_with(process, identity)
        close_fd.assert_called_once_with(identity.pidfd)
        self.assertTrue(process.stdin.closed)
        self.assertTrue(process.stdout.closed)
        self.assertTrue(process.stderr.closed)

    def test_capture_failure_kills_initial_group_and_reaps_direct_child(self):
        for module, runner in (
            (self.gate, lambda: self.gate.run_bounded_guest(["ignored"], b"input")),
            (self.helper, self.helper._run_solver),
        ):
            with self.subTest(module=module.__name__):
                process = mock.Mock()
                process.pid = 4700
                process.stdin = FakeStream(111) if module is self.gate else None
                process.stdout = FakeStream(112)
                process.stderr = FakeStream(113) if module is self.gate else None
                process.wait.return_value = -9
                error_type = module.GateError if module is self.gate else module.GuestError
                with (
                    mock.patch.object(module.subprocess, "Popen", return_value=process),
                    mock.patch.object(
                        module,
                        "_capture_process_group",
                        side_effect=error_type("capture-boom"),
                    ),
                    mock.patch.object(module.os, "killpg") as killpg,
                ):
                    with self.assertRaisesRegex(error_type, "capture-boom"):
                        runner()
                killpg.assert_called_once_with(process.pid, module.signal.SIGKILL)
                process.wait.assert_called_once_with(timeout=2)

    def test_group_scan_preflight_requires_frozen_leader(self):
        for module in (self.gate, self.helper):
            with self.subTest(module=module.__name__):
                error_type = self.gate.GateError if module is self.gate else self.helper.GuestError
                identity = module.ProcessGroupIdentity(4800, 4800, 4800, 86, 94)
                child = module.ProcessMember(4801, "S", 4800, 4800, 87)
                with mock.patch.object(module, "_group_members", return_value=[child]):
                    with self.assertRaises(error_type):
                        module._preflight_group_scan(identity)

    def test_real_proc_scan_skips_ungrouped_kernel_tasks_but_keeps_ours(self):
        current_pid = os.getpid()
        current_pgid = os.getpgrp()
        for module in (self.gate, self.helper):
            with self.subTest(module=module.__name__):
                members = module._scan_group_members_once(current_pgid)
                self.assertTrue(any(member.pid == current_pid for member in members))
                self.assertTrue(all(member.pgrp == current_pgid for member in members))

    def test_zero_group_kernel_stat_is_scan_skippable_but_not_a_valid_leader(self):
        fields = ["S", "1", "0", "0", *(["0"] * 15), "123"]
        kernel_stat = f"2 (kthreadd) {' '.join(fields)}\n"
        for module in (self.gate, self.helper):
            with self.subTest(module=module.__name__):
                error_type = self.gate.GateError if module is self.gate else self.helper.GuestError
                with mock.patch("builtins.open", mock.mock_open(read_data=kernel_stat)):
                    self.assertIsNone(
                        module._read_process_member(
                            2, skip_ungrouped_kernel_task=True
                        )
                    )
                with mock.patch("builtins.open", mock.mock_open(read_data=kernel_stat)):
                    with self.assertRaises(error_type):
                        module._read_process_member(2)
                zero_start_fields = ["S", "1", "0", "0", *(["0"] * 16)]
                zero_start_stat = f"2 (bad-kthread) {' '.join(zero_start_fields)}\n"
                with mock.patch(
                    "builtins.open", mock.mock_open(read_data=zero_start_stat)
                ):
                    with self.assertRaises(error_type):
                        module._read_process_member(
                            2, skip_ungrouped_kernel_task=True
                        )
                for pgrp, session_id in ((0, 2), (2, 0)):
                    malformed_fields = [
                        "S",
                        "1",
                        str(pgrp),
                        str(session_id),
                        *(["0"] * 15),
                        "123",
                    ]
                    malformed_stat = f"2 (not-kthread) {' '.join(malformed_fields)}\n"
                    with mock.patch(
                        "builtins.open", mock.mock_open(read_data=malformed_stat)
                    ):
                        with self.assertRaises(error_type):
                            module._read_process_member(
                                2, skip_ungrouped_kernel_task=True
                            )

    def test_helper_timeout_and_cleanup_failure_keep_both_causes(self):
        process = mock.Mock()
        process.pid = 4900
        process.stdout = FakeStream(121)
        identity = self.helper.ProcessGroupIdentity(4900, 4900, 4900, 88, 95)
        expected_label = self.helper.BOOTSTRAP_PROFILE + " (enforce)"
        with (
            mock.patch.object(self.helper.subprocess, "Popen", return_value=process),
            mock.patch.object(self.helper, "_capture_process_group", return_value=identity),
            mock.patch.object(self.helper, "_preflight_group_scan", return_value=[]),
            mock.patch("builtins.open", mock.mock_open(read_data=expected_label + "\n")),
            mock.patch.object(self.helper.os, "set_blocking"),
            mock.patch.object(self.helper, "RUNTIME_TIMEOUT_SECONDS", 0.0),
            mock.patch.object(
                self.helper,
                "_stop_group",
                side_effect=self.helper.GuestError("cleanup-boom"),
            ),
            mock.patch.object(self.helper.os, "close"),
        ):
            with self.assertRaises(self.helper.GuestError) as caught:
                self.helper._run_solver()
        message = str(caught.exception)
        self.assertIn("timeout=True", message)
        self.assertIn("solver cleanup failure", message)
        self.assertIn("cleanup-boom", message)
        self.assertTrue(process.stdout.closed)

    def test_helper_has_fd_process_group_and_output_tree_barriers(self):
        source = HELPER_PATH.read_text(encoding="utf-8")
        self.assertIn("os.pipe2(os.O_CLOEXEC)", source)
        self.assertIn("os.dup2(read_end, 0, inheritable=True)", source)
        self.assertIn("start_new_session=True", source)
        self.assertIn("os.killpg(identity.pgid, sig)", source)
        self.assertIn("os.killpg(process.pid, signal.SIGKILL)", source)
        self.assertIn("_preflight_group_scan(identity)", source)
        self.assertIn("if set(root_entries) != EXPECTED_ROOT_FILES | {\"data\"}", source)
        self.assertIn('"status": "GUEST_SOLVER_V5_COLD_A_NO_GO"', source)
        self.assertNotIn("subprocess.DEVNULL", source)


if __name__ == "__main__":
    unittest.main()
