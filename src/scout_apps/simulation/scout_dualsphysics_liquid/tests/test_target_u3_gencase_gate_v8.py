"""Static/mock-only tests for the admitted one-shot U3 C1M GenCase v8 runtime."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_gencase_gate_v8.py"
HELPER_PATH = PACKAGE_DIR / "scripts/r8_liquid_u3_gencase_bootstrap_helper_v8.py"
SUPERVISOR_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_gencase_supervisor_v8.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_execution_policy_v8.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gencase_execution_policy_v8.json"
PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-gencase-v8.profile"


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
    <drawcylinder mask="2"/></mainlist></commands></geometry>
    <motion><objreal ref="0"><begin mov="1" start="0"/><mvnull id="1"/></objreal></motion>
    </casedef><execution><parameters>
    <parameter key="MinFluidStop" value="1"/><parameter key="Shifting" value="1"/>
    <parameter key="DtAllParticles" value="1"/></parameters>
    <particles np="9078" nb="2669" nbf="0" mkboundfirst="2" mkfluidfirst="1">
    <moving mkbound="0" mk="2" begin="0" count="2669" refmotion="0"/>
    <fluid mkfluid="0" mk="1" begin="2669" count="6409"/></particles>
    <motion><objreal ref="0"><begin mov="1" start="0"/><mvnull id="1"/></objreal></motion>
    </execution></case>"""


def make_out() -> bytes:
    return b"""GenCase v5.4.354.01
Distance between points (Dp): 0.002
Fixed....: 0
Moving...: 2,669  id:(0-2668)
Floating.: 0
Total particles: 9,078 (bound=2669 (fx=0 mv=2669 ft=0) fluid=6409)
Particle limits:
  X range: -0.017 to 0.019 [m]
  Y range: -0.017 to 0.019 [m]
  Z range: 0 to 0.066 [m]
Finished execution
"""


class TargetU3GenCaseV8Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_module("u3_gencase_gate_v8_test", GATE_PATH)
        cls.helper = load_module("u3_gencase_helper_v8_test", HELPER_PATH)
        cls.supervisor = load_module("u3_gencase_supervisor_v8_test", SUPERVISOR_PATH)
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def success_frame(
        self,
        *,
        bi4: bytes | None = None,
        xml: bytes | None = None,
        out: bytes | None = None,
        console: bytes = b"console",
    ) -> bytes:
        bi4 = make_bi4() if bi4 is None else bi4
        xml = make_xml() if xml is None else xml
        out = make_out() if out is None else out
        outputs = {
            "C1M_zero.bi4": {"sha256": hashlib.sha256(bi4).hexdigest(), "size_bytes": len(bi4)},
            "C1M_zero.xml": {"sha256": hashlib.sha256(xml).hexdigest(), "size_bytes": len(xml)},
            "C1M_zero.out": {"sha256": hashlib.sha256(out).hexdigest(), "size_bytes": len(out)},
        }
        metadata = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1M_GENCASE_GUEST_FRAME_V8",
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
            len(out),
            len(console),
        )
        return header + encoded + bi4 + xml + out + console

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
        self.assertEqual(
            self.policy["status"],
            "ADMITTED_SINGLE_C1M_ZERO_MOTION_GENCASE_RUNTIME_V8_AFTER_FROZEN_V7_DUAL_MOTION_AUDIT_FAILURE_AND_V11_ZERO_DENIAL_PROBE",
        )
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
        self.assertIn("deny /proc/stat r,", effective)
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
        self.assertEqual(command[:4], ["/usr/bin/python3.12", "-I", "-B", "-S"])
        self.assertNotIn("/usr/bin/setpriv", command)
        for redundant in ("--clear-groups", "--inh-caps=-all", "--ambient-caps=-all", "--bounding-set=-all", "--no-new-privs"):
            self.assertNotIn(redundant, command)
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

    def test_success_frame_exact_parse_and_minimum_bi4_xml_out_reaudit(self):
        frame = self.success_frame()
        parsed = self.supervisor.parse_success_frame(frame)
        self.assertEqual(parsed["frame_size_bytes"], len(frame))
        self.assertEqual(parsed["console"], b"console")
        self.assertEqual(parsed["payloads"]["C1M_zero.out"], make_out())
        self.helper._audit_bi4_header(make_bi4())
        self.helper._audit_generated_out(make_out())
        self.helper._audit_zero_motion_generated_features(ET.fromstring(make_xml()))
        self.assertEqual(self.supervisor.OUTPUT_FRAME_LIMIT, 19_464_248)

    def test_frame_rejects_trailing_overdeclared_bad_bi4_xml_and_out(self):
        frame = self.success_frame()
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(frame + b"x")
        header = bytearray(frame[: self.supervisor.OUTPUT_HEADER.size])
        magic, version, meta, _bi4, xml, out, console = self.supervisor.OUTPUT_HEADER.unpack(header)
        over = self.supervisor.OUTPUT_HEADER.pack(
            magic, version, meta, self.supervisor.OUTPUT_BI4_LIMIT + 1, xml, out, console
        )
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(over + frame[self.supervisor.OUTPUT_HEADER.size :])
        bad_bi4 = bytearray(make_bi4())
        bad_bi4[60] = 1
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(self.success_frame(bi4=bytes(bad_bi4)))
        bad_xml = make_xml().replace(b'mask="2"', b'mask="1"')
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(self.success_frame(xml=bad_xml))
        invalid_motion_xmls = (
            make_xml().replace(b"<motion>", b'<motion active="true">'),
            make_xml().replace(b'<objreal ref="0">', b'<objreal ref="1">'),
            make_xml().replace(b'<mvnull id="1"/>', b'<mvrect id="1"/>'),
            make_xml().replace(
                b'<motion><objreal ref="0"><begin mov="1" start="0"/><mvnull id="1"/></objreal></motion>',
                b"",
                1,
            ),
            make_xml().replace(
                b'<mvnull id="1"/>',
                b'<mvnull id="2"/>',
                1,
            ),
            make_xml().replace(b'start="0"', b'start="nan"', 1),
            make_xml().replace(b'start="0"', b'start="0" finish="1"', 1),
            make_xml().replace(b'<mvnull id="1"/>', b'<mvnull id="1" extra="1"/>', 1),
            make_xml().replace(b'<mvnull id="1"/>', b'<mvnull id="1"/>TAIL', 1),
            make_xml().replace(
                b"</casedef>",
                b'<objreal ref="0"><begin mov="1" start="0"/><mvnull id="1"/></objreal></casedef>',
            ),
            make_xml().replace(b"</execution>", b"<special/></execution>"),
            make_xml().replace(
                b"</execution>",
                b'<motion><objreal ref="0"><begin mov="1" start="0"/><mvnull id="1"/></objreal></motion></execution>',
            ),
            make_xml().replace(b"</motion>", b"</motion><floating/>", 1),
            make_xml().replace(b'nbf="0"', b'nbf="2669"'),
            make_xml().replace(b'refmotion="0"', b'refmotion="1"'),
        )
        for invalid_motion_xml in invalid_motion_xmls:
            with self.assertRaises(self.supervisor.SupervisorError):
                self.supervisor.parse_success_frame(self.success_frame(xml=invalid_motion_xml))
            with self.assertRaises(self.helper.GuestError):
                self.helper._audit_zero_motion_generated_features(ET.fromstring(invalid_motion_xml))
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(
                self.success_frame(
                    xml=make_xml().replace(
                        b'key="Shifting" value="1"',
                        b'key="Shifting" value="2"',
                    )
                )
            )
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(self.success_frame(out=make_out() + b"\0"))
        with self.assertRaises(self.supervisor.SupervisorError):
            self.supervisor.parse_success_frame(self.success_frame(out=make_out().replace(b"9,078", b"9,079")))

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
                self.assertEqual(
                    sorted(path.name for path in output.iterdir()),
                    ["C1M_zero.bi4", "C1M_zero.out", "C1M_zero.xml"],
                )
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

    def test_v5_and_v7_failures_are_frozen_and_c1m_seed_v3_receipt_is_pinned(self):
        rejected = self.policy["rejected_predecessor"]
        self.assertEqual(rejected["permanent_status"], "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY")
        self.assertEqual(rejected["identity"], "u3_c1_gencase_v5_20260808T070530Z")
        self.assertEqual(rejected["start_receipt_sha256"], "9b26b2334eefc6549d315176be69823fc40e4de87495dc07daeefd38e8d0bac3")
        self.assertEqual(rejected["execution_receipt_sha256"], "49b23780ff1d6085c7ae1c5073ca1ee71fcf806657da6138d3dbfd39d893ba83")
        self.assertEqual(rejected["lifecycle_receipt_sha256"], "7bf446a6aacf58b6459f40c6d3214f3c9317d0c76a3fbb67af66a66d539bd374")
        self.assertNotEqual(self.policy["frozen_attempt"]["case_id"], "u3_c1_gencase_20260807T032831Z")
        rejected_v7 = self.policy["rejected_v7_attempt"]
        self.assertEqual(rejected_v7["identity"], "u3_c1m_gencase_v7_20260808T151500Z")
        self.assertEqual(
            rejected_v7["failure_class"],
            "SOURCE_PROVEN_DUAL_MOTION_COPY_AUDITOR_FALSE_NEGATIVE",
        )
        self.assertEqual(rejected_v7["start_receipt_sha256"], self.supervisor.V7_START_RECEIPT_SHA256)
        self.assertEqual(
            rejected_v7["execution_receipt_sha256"],
            self.supervisor.V7_EXECUTION_RECEIPT_SHA256,
        )
        self.assertEqual(
            rejected_v7["lifecycle_receipt_sha256"],
            self.supervisor.V7_LIFECYCLE_RECEIPT_SHA256,
        )
        self.assertFalse(rejected_v7["case_exported"])
        self.assertTrue(rejected_v7["cleanup_complete"])
        seed_receipt = self.supervisor.SEED_RECEIPT.read_bytes()
        self.assertEqual(hashlib.sha256(seed_receipt).hexdigest(), self.supervisor.SEED_RECEIPT_SHA256)
        self.assertEqual(self.policy["seed_provenance"]["receipt_sha256"], self.supervisor.SEED_RECEIPT_SHA256)
        self.assertEqual(self.policy["seed_provenance"]["seed_id"], self.supervisor.SEED_ID)
        self.assertEqual(
            self.policy["seed_provenance"]["receipt_status"],
            "PASS_NONEXECUTABLE_GENCASE_SEED_V3_MATERIALIZATION",
        )
        self.assertEqual(
            self.supervisor.SNAPSHOT_SOURCE_PAIRS[-1],
            (
                str(self.supervisor.SEED_INPUT_ROOT / "C1M_moving_zero_Def.xml"),
                "C1M_moving_zero_Def.xml",
            ),
        )


if __name__ == "__main__":
    unittest.main()
