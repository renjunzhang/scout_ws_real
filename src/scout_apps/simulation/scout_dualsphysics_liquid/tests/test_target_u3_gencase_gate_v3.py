"""Static/mock-only tests for the admitted one-shot U3 C1 GenCase v3 runtime."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import struct
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_gencase_gate_v3.py"
HELPER_PATH = PACKAGE_DIR / "scripts/r8_liquid_u3_gencase_bootstrap_helper_v3.py"
SUPERVISOR_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_gencase_supervisor_v3.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_execution_policy_v3.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gencase_execution_policy_v3.json"
PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-gencase-v3.profile"


def load_module(name: str, path: Path):
    specification = importlib.util.spec_from_file_location(name, path)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def make_bi4() -> bytes:
    raw = bytearray(b" " * 128)
    prefix = b"#FileJBD JPartDataBi4"
    raw[: len(prefix)] = prefix
    raw[58] = 0x0A
    raw[59:64] = b"\0" * 5
    struct.pack_into("<I", raw, 64, 60)
    struct.pack_into("<I", raw, 68, 6)
    raw[72:78] = b"\nITEM\n"
    struct.pack_into("<I", raw, 78, 12)
    raw[82:94] = b"JPartDataBi4"
    return bytes(raw)


def make_xml() -> bytes:
    return b"""<case><casedef><geometry><definition dp="0.002"/><commands><mainlist>
    <drawcylinder mask="2"/></mainlist></commands></geometry></casedef><execution><parameters>
    <parameter key="MinFluidStop" value="1"/></parameters></execution></case>"""


class TargetU3GenCaseV3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_module("u3_gencase_gate_v3_test", GATE_PATH)
        cls.helper = load_module("u3_gencase_helper_v3_test", HELPER_PATH)
        cls.supervisor = load_module("u3_gencase_supervisor_v3_test", SUPERVISOR_PATH)
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def success_frame(self, *, bi4: bytes | None = None, xml: bytes | None = None, console: bytes = b"console") -> bytes:
        bi4 = make_bi4() if bi4 is None else bi4
        xml = make_xml() if xml is None else xml
        outputs = {
            "C1_static.bi4": {"sha256": hashlib.sha256(bi4).hexdigest(), "size_bytes": len(bi4)},
            "C1_static.xml": {"sha256": hashlib.sha256(xml).hexdigest(), "size_bytes": len(xml)},
        }
        metadata = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_GUEST_FRAME_V3",
            "status": "GUEST_FRAME_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT",
            "gencase_argv": self.supervisor.GENCASE_ARGV,
            "guest_inputs": {
                name: {"sha256": digest, "size_bytes": size}
                for name, (digest, size) in self.supervisor.INPUT_CONTRACT.items()
            },
            "guest_identity": {
                "uid": [0, 0, 0, 0],
                "gid": [0, 0, 0, 0],
                "groups": [],
                "capabilities": {name: 0 for name in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")},
                "no_new_privs": 1,
            },
            "guest_label": self.supervisor.BOOTSTRAP_PROFILE + " (enforce)",
            "candidate_label": self.supervisor.BOOTSTRAP_PROFILE + " (enforce)",
            "guest_work_tmpfs": {
                "mountpoint": "/work",
                "mount_options": ["rw", "nosuid", "nodev"],
                "filesystem": "tmpfs",
                "super_options": ["rw"],
                "total_bytes": 67108864,
                "inode_ceiling_claimed": False,
            },
            "guest_outputs": outputs,
            "candidate_console": {
                "sha256": hashlib.sha256(console).hexdigest(),
                "size_bytes": len(console),
                "framed_for_separate_0440_console_log": True,
            },
            "stdin_frame_consumed_to_eof_then_replaced_by_guest_eof_pipe": True,
            "candidate_stdout_stderr_were_internal_pipe_only": True,
        }
        encoded = json.dumps(metadata, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()
        header = self.supervisor.OUTPUT_HEADER.pack(
            self.supervisor.OUTPUT_MAGIC,
            self.supervisor.OUTPUT_VERSION,
            len(encoded),
            len(bi4),
            len(xml),
            len(console),
        )
        return header + encoded + bi4 + xml + console

    def test_policy_schema_artifact_hashes_and_admitted_status(self):
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
            entry = self.policy["trusted_artifacts"][name]
            raw = path.read_bytes()
            self.assertEqual(hashlib.sha256(raw).hexdigest(), entry["sha256"])
            self.assertEqual(len(raw), entry["size_bytes"])
        self.assertEqual(self.policy["status"], "ADMITTED_SINGLE_GENCASE_RUNTIME_V3_AFTER_FROZEN_V11_ZERO_DENIAL_PROBE")
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check", "internal-run"])
        self.assertEqual(self.policy["required_harmless_probes"]["admission"], "PASS_FROZEN_V11_ZERO_DENIAL_STDIO_APPARMOR_TRANSPORT_PROBE")
        self.gate.verify_review_artifacts(run_parser=False)
        self.supervisor.verify_policy_static()

    def test_profile_uses_rix_exact_v11_mount_grammar_and_empty_runtime_label(self):
        text = PROFILE_PATH.read_text(encoding="utf-8")
        effective = self.gate._effective_profile(text)
        self.assertIn("/work/runtime/GenCase_linux64 rix,", effective)
        self.assertNotIn("rpx", effective.lower())
        runtime = effective.split(f"profile {self.supervisor.RUNTIME_PROFILE}", 1)[1]
        for token in ("/work", "/usr", "userns ", "mount ", "capability ", "network ", "/proc", "/dev", "/home/", "signal ", "rix"):
            self.assertNotIn(token, runtime)
        self.assertIn(f"signal (send,receive) set=(term,kill,exists) peer={self.supervisor.BOOTSTRAP_PROFILE},", effective)
        self.assertIn("deny capability dac_override,", effective)
        self.assertEqual(sum(1 for line in effective.splitlines() if line.strip().startswith(("mount ", "remount ", "pivot_root ", "umount "))), 12)
        self.assertTrue(self.gate.verify_profile(run_parser=False)["production_load_allowed"])

    def test_bwrap_has_one_64m_work_tmpfs_no_host_input_output_fd_or_dev(self):
        helper = self.policy["trusted_artifacts"]["helper"]
        argv = self.gate.bwrap_argv(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
        self.assertEqual(self.gate._pairs(argv, "--ro-bind"), [["/usr", "/usr"]])
        for forbidden in ("--bind", "--bind-fd", "--file", "--dev", "--dev-bind"):
            self.assertNotIn(forbidden, argv)
        self.assertEqual(argv.count("--tmpfs"), 1)
        size_index = argv.index("--size")
        self.assertEqual(argv[size_index:size_index + 4], ["--size", "67108864", "--tmpfs", "/work"])
        self.assertEqual(argv[argv.index("--cap-drop") + 1], "ALL")
        self.assertIn("--assert-userns-disabled", argv)
        self.assertEqual(argv[argv.index("--proc") + 1], "/proc")
        command = self.gate.guest_command(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
        for required in ("--clear-groups", "--inh-caps=-all", "--ambient-caps=-all", "--bounding-set=-all", "--no-new-privs"):
            self.assertIn(required, command)
        self.assertIn("-S", command)
        self.assertIn(["--setenv", "TZ", "UTC0"], [argv[index:index + 3] for index in range(len(argv) - 2)])
        self.assertNotIn("--dir", argv)
        self.assertEqual(argv[-len(command):], command)

    def test_fixed_input_frame_uses_reviewed_bytes_fixed_order_and_no_lengths(self):
        helper = HELPER_PATH.read_bytes()
        inputs = {name: (self.supervisor.SEED_INPUT_ROOT / name).read_bytes() for name in self.gate.INPUT_CONTRACT}
        frame = self.gate.build_input_frame(helper, inputs)
        expected = len(self.gate.HELPER_MAGIC) + len(helper) + len(self.gate.INPUT_MAGIC)
        expected += sum(size for _digest, size in self.gate.INPUT_CONTRACT.values())
        self.assertEqual(len(frame), expected)
        self.assertTrue(frame.startswith(self.gate.HELPER_MAGIC + helper + self.gate.INPUT_MAGIC))
        changed = dict(inputs)
        changed["DsphConfig.xml"] = changed["DsphConfig.xml"][:-1] + b"x"
        with self.assertRaises(self.gate.GateError):
            self.gate.build_input_frame(helper, changed)

    def test_host_and_guest_identity_contracts_fail_closed(self):
        host = {
            "uid": [1000] * 4,
            "gid": [1000] * 4,
            "groups": [],
            "capabilities": {name: 0 for name in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")},
            "no_new_privs": 1,
        }
        self.assertEqual(self.gate.verify_child_identity(host), host)
        for mutation in (
            {**host, "groups": [27]},
            {**host, "no_new_privs": 0},
            {**host, "capabilities": {**host["capabilities"], "CapAmb": 1}},
            {**host, "uid": [1000, 1000, 0, 1000]},
        ):
            with self.assertRaises(self.gate.GateError):
                self.gate.verify_child_identity(mutation)

    def test_success_frame_exact_parse_and_minimum_bi4_xml_reaudit(self):
        frame = self.success_frame()
        parsed = self.supervisor.parse_success_frame(frame)
        self.assertEqual(parsed["frame_size_bytes"], len(frame))
        self.assertEqual(parsed["console"], b"console")
        self.helper._audit_bi4_header(make_bi4())
        self.assertEqual(self.supervisor.OUTPUT_FRAME_LIMIT, 18_415_664)

    def test_frame_rejects_trailing_overdeclared_bad_bi4_and_bad_xml(self):
        frame = self.success_frame()
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(frame + b"x")
        header = bytearray(frame[: self.supervisor.OUTPUT_HEADER.size])
        magic, version, meta, _bi4, xml, console = self.supervisor.OUTPUT_HEADER.unpack(header)
        over = self.supervisor.OUTPUT_HEADER.pack(magic, version, meta, self.supervisor.OUTPUT_BI4_LIMIT + 1, xml, console)
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(over + frame[self.supervisor.OUTPUT_HEADER.size :])
        bad_bi4 = bytearray(make_bi4())
        bad_bi4[60] = 1
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(self.success_frame(bi4=bytes(bad_bi4)))
        bad_xml = make_xml().replace(b'mask="2"', b'mask="1"')
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(self.success_frame(xml=bad_xml))

    def test_label_residue_uses_attr_current_not_cmdline_spoof(self):
        with tempfile.TemporaryDirectory() as temporary:
            proc = Path(temporary)
            spoof = proc / "101"
            (spoof / "attr").mkdir(parents=True)
            (spoof / "attr/current").write_text("unconfined\n", encoding="utf-8")
            (spoof / "cmdline").write_bytes(b"GenCase_linux64\0")
            real = proc / "202"
            (real / "attr").mkdir(parents=True)
            (real / "attr/current").write_text(f"{self.supervisor.RUNTIME_PROFILE} (enforce)\n", encoding="utf-8")
            (real / "cmdline").write_bytes(b"harmless-name\0")
            self.assertEqual(
                self.supervisor.labeled_processes(proc),
                [{"pid": 202, "label": self.supervisor.RUNTIME_PROFILE, "attr_current": f"{self.supervisor.RUNTIME_PROFILE} (enforce)"}],
            )

    def test_uid1000_o_excl_export_preserves_partial_and_rejects_reuse(self):
        frame = self.success_frame(console=b"bounded console")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            attempt = root / "attempt.partial"
            output = attempt / "output"
            console = root / "console.log"
            with (
                mock.patch.object(self.supervisor, "ATTEMPT_ROOT", attempt),
                mock.patch.object(self.supervisor, "OUTPUT_ROOT", output),
                mock.patch.object(self.supervisor, "CONSOLE_LOG", console),
                mock.patch.object(self.supervisor, "verify_host_export_identity", return_value={"verified": True}),
                mock.patch.object(self.supervisor, "consume_export_capability", return_value={"verified": True}),
            ):
                result = self.supervisor.export_frame_o_excl(frame)
                self.assertEqual(sorted(path.name for path in output.iterdir()), ["C1_static.bi4", "C1_static.xml"])
                self.assertEqual(console.read_bytes(), b"bounded console")
                self.assertEqual(result["console_log"]["mode"], "0440")
                with self.assertRaises(self.supervisor.SupervisorError):
                    self.supervisor.export_frame_o_excl(frame)

    def test_bounded_conduit_hard_stops_stdout_overflow(self):
        argv = ["/usr/bin/python3.12", "-I", "-B", "-c", "import sys; d=sys.stdin.buffer.read(); sys.stdout.buffer.write(d)"]
        with mock.patch.object(self.gate, "GUEST_STDOUT_LIMIT", 8):
            returncode, stdout, stderr = self.gate.run_bounded_guest(argv, b"12345678")
            self.assertEqual((returncode, stdout, stderr), (0, b"12345678", b""))
            with self.assertRaises(self.gate.GateError):
                self.gate.run_bounded_guest(argv, b"123456789")

    def test_internal_gate_requires_snapshot_and_root_fd_before_launch(self):
        with self.assertRaises(self.gate.GateError):
            self.gate.verify_snapshot_runtime()
        with self.assertRaises(self.gate.GateError):
            self.gate.consume_root_admission_capability()

    def test_helper_static_fd_and_process_group_barriers(self):
        source = HELPER_PATH.read_text(encoding="utf-8")
        self.assertIn("os.pipe2(os.O_CLOEXEC)", source)
        self.assertIn("os.dup2(read_end, 0, inheritable=True)", source)
        self.assertIn("start_new_session=True", source)
        self.assertIn("os.killpg(process.pid, signal.SIGTERM)", source)
        self.assertIn("os.killpg(process.pid, signal.SIGKILL)", source)
        self.assertNotIn("subprocess.DEVNULL", source)
        self.assertIn("os.statvfs(\"/work\")", source)
        self.assertIn("/proc/self/mountinfo", source)

    def test_v2_is_append_only_rejected_and_seed_v2_receipt_is_pinned(self):
        rejected = self.policy["rejected_predecessor"]
        self.assertEqual(rejected["permanent_status"], "NO_GO_V2_STATIC_IDENTITY_NEVER_EXECUTE_OR_REUSE")
        self.assertNotEqual(self.policy["frozen_attempt"]["case_id"], "u3_c1_gencase_20260807T032831Z")
        seed_receipt = self.supervisor.SEED_RECEIPT.read_bytes()
        self.assertEqual(hashlib.sha256(seed_receipt).hexdigest(), self.supervisor.SEED_RECEIPT_SHA256)
        self.assertEqual(self.policy["seed_provenance"]["receipt_sha256"], self.supervisor.SEED_RECEIPT_SHA256)


if __name__ == "__main__":
    unittest.main()
