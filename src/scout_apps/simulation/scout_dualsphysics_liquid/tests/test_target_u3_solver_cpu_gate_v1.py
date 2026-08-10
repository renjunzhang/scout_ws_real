"""Static/mock-only tests for the fresh U3 C1 CPU-solver smoke v1."""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_solver_cpu_gate_v1.py"
HELPER_PATH = PACKAGE_DIR / "scripts/r8_liquid_u3_solver_cpu_bootstrap_helper_v1.py"
SUPERVISOR_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_solver_cpu_supervisor_v1.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_execution_policy_v1.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_solver_cpu_execution_policy_v1.json"
PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-solver-cpu-v1.profile"
HARNESS_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"


def load_module(name: str, path: Path):
    specification = importlib.util.spec_from_file_location(name, path)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class TargetU3SolverCpuV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_module("u3_solver_cpu_gate_v1_test", GATE_PATH)
        cls.helper = load_module("u3_solver_cpu_helper_v1_test", HELPER_PATH)
        cls.supervisor = load_module("u3_solver_cpu_supervisor_v1_test", SUPERVISOR_PATH)
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def success_frame(self, *, console: bytes = b"") -> bytes:
        payloads = {path: (f"payload:{path}\n").encode("ascii") for path in self.supervisor.EXPECTED_PATHS}
        payloads["Run.out"] = b"[Simulation finished]\nFinished execution (code=0).\n"
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
            "document_type": "SMPCC_R8_LIQUID_U3_C1_CPU_SOLVER_GUEST_FRAME_V1",
            "status": "GUEST_SOLVER_SMOKE_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT",
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
            "output_audit": {"file_count": 29, "total_bytes": len(payload)},
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
            "REVIEWED_FRESH_SINGLE_CPU_SOLVER_SMOKE_V1_PENDING_STATIC_VERIFICATION",
        )
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check", "internal-run"])
        self.gate.verify_review_artifacts(verify_tools=False)

    def test_snapshot_bootstrap_and_loader_sources_compile(self):
        compile(
            self.supervisor.SNAPSHOT_BOOTSTRAP_BYTES,
            "<solver-cpu-v1-snapshot-bootstrap-test>",
            "exec",
        )
        compile(
            self.supervisor.SNAPSHOT_BOOTSTRAP_LOADER_BYTES,
            "<solver-cpu-v1-snapshot-loader-test>",
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

    def test_solver_argv_is_cpu_single_thread_one_second_and_stable(self):
        argv = self.gate.SOLVER_ARGV
        for required in ("-cpu", "-ompthreads:1", "-stable:1", "-vres:0", "-cellmode:full", "-tmax:1.0"):
            self.assertEqual(argv.count(required), 1)
        self.assertNotIn("-gpu", argv)
        self.assertEqual(argv, self.helper.SOLVER_ARGV)
        self.assertEqual(self.gate.ENVIRONMENT["OMP_NUM_THREADS"], "1")
        self.assertEqual(self.policy["fixed_guest_command"]["solver_argv"], argv)

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

    def test_success_frame_exact_29_files_and_tamper_rejection(self):
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
                self.assertEqual(verified["file_count"], 29)
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

    def test_helper_has_fd_process_group_and_output_tree_barriers(self):
        source = HELPER_PATH.read_text(encoding="utf-8")
        self.assertIn("os.pipe2(os.O_CLOEXEC)", source)
        self.assertIn("os.dup2(read_end, 0, inheritable=True)", source)
        self.assertIn("start_new_session=True", source)
        self.assertIn("os.killpg(pid, signal.SIGTERM)", source)
        self.assertIn("os.killpg(pid, signal.SIGKILL)", source)
        self.assertIn("if set(root_entries) != EXPECTED_ROOT_FILES | {\"data\"}", source)
        self.assertNotIn("subprocess.DEVNULL", source)


if __name__ == "__main__":
    unittest.main()
