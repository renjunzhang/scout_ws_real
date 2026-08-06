#!/usr/bin/env python3
"""Mock-only tests for the QEMU TCG machine-none gate."""

from __future__ import annotations

import copy
import hashlib
import io
import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_qemu_machine_none_gate as gate  # noqa: E402


ZERO_CAPS = {
    "CapInh": "0000000000000000",
    "CapPrm": "0000000000000000",
    "CapEff": "0000000000000000",
    "CapBnd": "0000000000000000",
    "CapAmb": "0000000000000000",
}
REAL_WRITE_ATTEMPT_MARKER = gate._write_attempt_marker


class MockOnlyTestCase(unittest.TestCase):
    """Default-deny all process creation and external-command APIs."""

    def setUp(self) -> None:
        super().setUp()
        targets = [
            (gate.os, "fork"), (gate.os, "execve"), (gate.os, "_exit"),
            (gate.os, "setgroups"), (gate.os, "setsid"), (gate.os, "fchdir"),
            (gate.os, "dup2"), (gate.os, "kill"), (gate.os, "killpg"), (gate.os, "umask"),
            (gate.resource, "setrlimit"),
            (gate.subprocess, "Popen"), (gate.subprocess, "run"),
            (gate.subprocess, "call"), (gate.subprocess, "check_call"),
            (gate.subprocess, "check_output"),
        ]
        for optional in ("execv", "execvp", "execvpe", "posix_spawn", "posix_spawnp"):
            if hasattr(gate.os, optional):
                targets.append((gate.os, optional))
        self.live_api_patchers = []
        for owner, name in targets:
            patcher = mock.patch.object(owner, name, side_effect=AssertionError(f"mock-only poison reached: {name}"))
            patcher.start()
            self.addCleanup(patcher.stop)
            self.live_api_patchers.append(patcher)
        unshare_patcher = mock.patch.object(gate, "_unshare", side_effect=AssertionError("mock-only poison reached: unshare"))
        unshare_patcher.start()
        self.addCleanup(unshare_patcher.stop)
        self.live_api_patchers.append(unshare_patcher)
        for name in ("_drop_all_capabilities", "_child_hardening", "_write_proc_control", "_write_attempt_marker"):
            patcher = mock.patch.object(gate, name, side_effect=AssertionError(f"mock-only poison reached: {name}"))
            patcher.start()
            self.addCleanup(patcher.stop)
            self.live_api_patchers.append(patcher)
        receipt_patcher = mock.patch.object(
            gate.safety, "atomic_write_json_new", side_effect=AssertionError("mock-only poison reached: receipt write")
        )
        receipt_patcher.start()
        self.addCleanup(receipt_patcher.stop)
        self.live_api_patchers.append(receipt_patcher)


def safe_namespace_evidence() -> dict:
    return {
        "pid": 4242,
        "pgid": 4242,
        "starttime_ticks": 123,
        "pidfd_opened": True,
        "parent_user_namespace": "user:[1]",
        "parent_network_namespace": "net:[2]",
        "child_user_namespace": "user:[3]",
        "child_network_namespace": "net:[4]",
        "child_user_namespace_device": 5,
        "child_user_namespace_inode": 3,
        "child_network_namespace_device": 5,
        "child_network_namespace_inode": 4,
        "user_namespace_distinct": True,
        "network_namespace_distinct": True,
        "uid_map": "1000 1000 1\n",
        "gid_map": "1000 1000 1\n",
        "setgroups": "deny\n",
        "supplementary_groups": [],
        "capabilities": dict(ZERO_CAPS),
        "no_new_privs": "1",
        "interface_names": ["lo"],
        "ipv4_routes": [],
        "ipv6_routes": [],
        "inet_socket_rows": {name: [] for name in ("tcp", "tcp6", "udp", "udp6", "raw", "raw6")},
        "loopback_up": False,
        "loopback_ipv4_configured": False,
        "loopback_ipv6_configured": False,
        "verified_before_exec": True,
        "namespace_fds_pinned_during_verification": True,
        "namespace_fds_closed_before_exec_ack": True,
        "pidfd_pinned_until_child_reap": True,
        "persistent_bind_mount": False,
        "external_namespace_helper_executed": False,
    }


class MachineNonePolicyTests(MockOnlyTestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, _ = gate.read_json_and_hash(gate.POLICY_PATH)
        cls.admission, _ = gate.read_json_and_hash(gate.ADMISSION_SCHEMA_PATH)
        cls.attempt_schema, _ = gate.read_json_and_hash(gate.ATTEMPT_SCHEMA_PATH)
        cls.receipt_schema, _ = gate.read_json_and_hash(gate.RECEIPT_SCHEMA_PATH)

    def test_policy_and_all_schemas_are_valid_and_fail_closed(self) -> None:
        gate.install_gate.Draft202012Validator.check_schema(self.admission)
        gate.install_gate.Draft202012Validator.check_schema(self.attempt_schema)
        gate.install_gate.Draft202012Validator.check_schema(self.receipt_schema)
        gate.validate_with_schema(self.policy, self.admission, "policy")
        gate.validate_policy_semantics(self.policy)
        self.assertFalse(self.policy["host_kernel"]["live_namespace_or_qemu_execution_authorized"])
        self.assertTrue(self.policy["host_kernel"]["namespace_security_update_required"])

    def test_policy_schema_rejects_unknown_missing_and_command_drift(self) -> None:
        unknown = copy.deepcopy(self.policy)
        unknown["namespace_boundary"]["mount_host"] = True
        with self.assertRaises(gate.QemuMachineNoneError):
            gate.validate_with_schema(unknown, self.admission, "unknown")
        missing = copy.deepcopy(self.policy)
        missing["execution_boundary"].pop("pidfd_open_required")
        with self.assertRaises(gate.QemuMachineNoneError):
            gate.validate_with_schema(missing, self.admission, "missing")
        drift = copy.deepcopy(self.policy)
        drift["qmp_contract"]["commands_exact"][1]["execute"] = "query-version"
        with self.assertRaises(gate.QemuMachineNoneError):
            gate.validate_with_schema(drift, self.admission, "drift")

    def test_exact_argv_is_minimal_supported_qemu_42_contract(self) -> None:
        self.assertEqual(tuple(self.policy["binary"]["argv_exact"]), gate.EXACT_QEMU_ARGV)
        self.assertNotIn("-nodefconfig", gate.EXACT_QEMU_ARGV)
        required = {
            "-no-user-config", "-machine", "none", "-accel", "tcg,thread=single",
            "-S", "-nodefaults", "-display", "-monitor", "-serial", "-parallel",
            "-nic", "-sandbox", "-qmp", "stdio",
        }
        self.assertTrue(required.issubset(set(gate.EXACT_QEMU_ARGV)))
        forbidden = {
            "-drive", "-blockdev", "-device", "-bios", "-kernel", "-initrd",
            "-option-rom", "-netdev", "-enable-kvm", "-daemonize", "-chardev",
            "-nographic", "-readconfig", "-incoming", "-pflash",
        }
        self.assertFalse(forbidden.intersection(gate.EXACT_QEMU_ARGV))
        self.assertNotIn("/dev/kvm", "\0".join(gate.EXACT_QEMU_ARGV))

    def test_parser_has_no_generic_execution_surface(self) -> None:
        parser = gate.build_parser()
        self.assertEqual(parser.parse_args(["self-check"]).command, "self-check")
        parsed = parser.parse_args(["run-smoke", "--receipt", str(gate.AUDIT_DIR / gate.RECEIPT_NAME)])
        self.assertEqual(parsed.command, "run-smoke")
        serialized = Path(gate.__file__).read_text(encoding="utf-8")
        self.assertNotIn("qemu-img", " ".join(gate.EXACT_QEMU_ARGV))
        self.assertNotIn("add_argument(\"--argv\"", serialized)
        self.assertNotIn("add_argument(\"--qemu", serialized)

    def test_import_disables_bytecode_before_local_gate_imports(self) -> None:
        source = Path(gate.__file__).read_text(encoding="utf-8")
        self.assertLess(source.index("sys.dont_write_bytecode = True"), source.index("import r8_liquid_qemu_binary_probe_gate"))
        self.assertTrue(sys.dont_write_bytecode)

    def test_invalid_receipt_schema_fails_during_contract_load(self) -> None:
        invalid_receipt_schema = {"$schema": "https://json-schema.org/draft/2020-12/schema", "type": "not-a-json-schema-type"}
        values = iter((
            (self.policy, "a" * 64),
            (self.admission, "b" * 64),
            (self.attempt_schema, "c" * 64),
            (invalid_receipt_schema, "d" * 64),
        ))
        with mock.patch.object(gate, "read_json_and_hash", side_effect=lambda path: next(values)):
            with self.assertRaises(gate.SchemaError):
                gate.load_contract()

    def test_current_kernel_gate_blocks_before_marker_fork_unshare_or_exec(self) -> None:
        receipt = gate.AUDIT_DIR / gate.RECEIPT_NAME
        with mock.patch.object(gate, "_write_attempt_marker") as marker, mock.patch.object(
            gate.os, "fork", side_effect=AssertionError("fork reached")
        ) as fork, mock.patch.object(
            gate, "_unshare", side_effect=AssertionError("unshare reached")
        ) as unshare, mock.patch.object(
            gate.os, "execve", side_effect=AssertionError("exec reached")
        ) as execve:
            with self.assertRaisesRegex(gate.QemuMachineNoneError, "does not authorize"):
                gate.build_smoke_receipt(receipt)
        marker.assert_not_called()
        fork.assert_not_called()
        unshare.assert_not_called()
        execve.assert_not_called()

    def test_mock_only_poison_is_active(self) -> None:
        with self.assertRaisesRegex(AssertionError, "mock-only poison"):
            gate.os.fork()
        with self.assertRaisesRegex(AssertionError, "mock-only poison"):
            gate._unshare(gate.CLONE_NEWUSER)
        with self.assertRaisesRegex(AssertionError, "mock-only poison"):
            gate.os.execve("/forbidden", [], {})
        with self.assertRaisesRegex(AssertionError, "mock-only poison"):
            gate.subprocess.run(["/forbidden"])
        with self.assertRaisesRegex(AssertionError, "mock-only poison"):
            gate._drop_all_capabilities()
        with self.assertRaisesRegex(AssertionError, "mock-only poison"):
            gate.os.kill(os.getpid(), 0)

    def test_self_check_mock_path_cannot_reach_live_apis(self) -> None:
        keys = (
            "boot_id_sha256", "runtime_uid", "runtime_gid", "runtime_thread_count",
            "qemu_kvm_service", "autostart_link_exists", "autostart_link_is_symlink",
            "ksm_value", "qemu_processes", "simulation_processes", "simulation_ports",
            "process_visibility_errors", "process_visibility_exceptions", "package_state",
            "scratch", "lock", "host_kernel", "host_namespaces", "storage_mount", "storage_findmnt",
            "storage_anchor_identity", "approved_root_identity", "audit_dir_identity",
            "mount_points_affecting_root", "implementation", "binary",
        )
        snapshots = [dict({key: None for key in keys}, phase=phase) for phase in ("A_PRELOCKED", "B_PREEXEC", "C_POSTCLEANUP")]
        fake_contract = (self.policy, "a" * 64, self.admission, "b" * 64, self.attempt_schema, "c" * 64, self.receipt_schema, "d" * 64)
        with mock.patch.object(gate, "validate_runtime_paths"), mock.patch.object(
            gate, "load_contract", return_value=fake_contract
        ), mock.patch.object(
            gate, "verify_predecessor", return_value={}
        ), mock.patch.object(
            gate, "predecessor_errors", return_value=[]
        ), mock.patch.object(
            gate, "collect_snapshot", side_effect=snapshots
        ), mock.patch.object(
            gate, "snapshot_errors", return_value=["kernel NO-GO"]
        ), mock.patch.object(
            gate, "_residue", return_value=[]
        ), mock.patch.object(
            gate, "_base_report", return_value={"status": "NO_GO", "errors": ["kernel NO-GO"]}
        ), mock.patch.object(
            gate.os, "fork", side_effect=AssertionError("fork reached")
        ) as fork, mock.patch.object(
            gate, "_unshare", side_effect=AssertionError("unshare reached")
        ) as unshare, mock.patch.object(
            gate.os, "execve", side_effect=AssertionError("exec reached")
        ) as execve, mock.patch.object(
            gate, "_write_attempt_marker", side_effect=AssertionError("marker reached")
        ) as marker:
            result = gate.build_self_check()
        self.assertEqual(result["status"], "NO_GO")
        for poisoned in (fork, unshare, execve, marker):
            poisoned.assert_not_called()

    def test_old_version_probe_assets_remain_byte_exact(self) -> None:
        expected = {
            "config/sandbox/p0b_qemu_binary_probe_admission_v1.json": "d35e472138cfa7143257b6919a28ce93c4646fe06b3ff93ad83c99865ca733e9",
            "schema/qemu_binary_probe_admission_v1.json": "69eecee779a65117d967c0a6b62525b868b0cd990e28fdc21fce00e23945b188",
            "schema/qemu_binary_probe_receipt_v1.json": "23942cc42259a423517b66d175cf17bb37855020a36b9627fc4dd4ab322c1875",
            "scripts/r8_liquid_qemu_binary_probe_gate.py": "0ce5537967ee56e324d211b2034c85e9163de1409a1eb2b0e3a00c68c3cfaa59",
            "tests/test_qemu_binary_probe_gate.py": "2fe8c3c1ab24cb796b8e99d61004a631fdf1473354519695e98ad19e88c485a5",
        }
        for relative, digest in expected.items():
            self.assertEqual(hashlib.sha256((gate.PACKAGE_ROOT / relative).read_bytes()).hexdigest(), digest)

    def test_self_check_reports_consumed_attempt_and_never_ready(self) -> None:
        keys = (
            "boot_id_sha256", "runtime_uid", "runtime_gid", "runtime_thread_count",
            "qemu_kvm_service", "autostart_link_exists", "autostart_link_is_symlink",
            "ksm_value", "qemu_processes", "simulation_processes", "simulation_ports",
            "process_visibility_errors", "process_visibility_exceptions", "package_state",
            "scratch", "lock", "host_kernel", "host_namespaces", "storage_mount", "storage_findmnt",
            "storage_anchor_identity", "approved_root_identity", "audit_dir_identity",
            "mount_points_affecting_root", "implementation", "binary",
        )
        snapshots = [dict({key: None for key in keys}, phase=phase) for phase in ("A_PRELOCKED", "B_PREEXEC", "C_POSTCLEANUP")]
        fake_contract = (self.policy, "a" * 64, self.admission, "b" * 64, self.attempt_schema, "c" * 64, self.receipt_schema, "d" * 64)
        with mock.patch.object(gate, "validate_runtime_paths"), mock.patch.object(
            gate, "load_contract", return_value=fake_contract
        ), mock.patch.object(gate, "verify_predecessor", return_value={}), mock.patch.object(
            gate, "predecessor_errors", return_value=[]
        ), mock.patch.object(gate, "collect_snapshot", side_effect=snapshots), mock.patch.object(
            gate, "snapshot_errors", return_value=[]
        ), mock.patch.object(gate, "_residue", return_value=[gate.ATTEMPT_NAME]), mock.patch.object(
            gate, "_base_report", side_effect=lambda *args: {"status": args[-1], "errors": list(args[-2])}
        ):
            result = gate.build_self_check()
        self.assertEqual(result["status"], "ATTEMPT_CONSUMED_NO_OUTCOME")
        self.assertTrue(any("retry forbidden" in item for item in result["errors"]))


class NamespaceAndAttemptTests(MockOnlyTestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, _ = gate.read_json_and_hash(gate.POLICY_PATH)

    def test_namespace_evidence_accepts_only_exact_isolation(self) -> None:
        evidence = safe_namespace_evidence()
        gate.validate_namespace_evidence(evidence, self.policy)
        mutations = (
            ("user_namespace_distinct", False),
            ("uid_map", "0 1000 1\n"),
            ("setgroups", "allow\n"),
            ("supplementary_groups", [27]),
            ("no_new_privs", "0"),
            ("interface_names", ["eth0", "lo"]),
            ("ipv4_routes", ["route"]),
            ("loopback_up", True),
            ("namespace_fds_closed_before_exec_ack", False),
        )
        for key, value in mutations:
            changed = copy.deepcopy(evidence)
            changed[key] = value
            with self.subTest(key=key), self.assertRaises(gate.QemuMachineNoneError):
                gate.validate_namespace_evidence(changed, self.policy)
        changed = copy.deepcopy(evidence)
        changed["capabilities"]["CapEff"] = "0000000000000001"
        with self.assertRaises(gate.QemuMachineNoneError):
            gate.validate_namespace_evidence(changed, self.policy)
        changed = copy.deepcopy(evidence)
        changed["inet_socket_rows"]["tcp"] = ["socket"]
        with self.assertRaises(gate.QemuMachineNoneError):
            gate.validate_namespace_evidence(changed, self.policy)

    def test_namespace_setup_order_is_user_map_then_net_then_cap_drop(self) -> None:
        order = []
        with mock.patch.object(gate, "_unshare", side_effect=lambda flag: order.append(("unshare", flag))), mock.patch.object(
            gate.os, "setgroups", side_effect=lambda groups: order.append(("setgroups", groups))
        ), mock.patch.object(
            gate, "_write_proc_control", side_effect=lambda path, data: order.append((path, data))
        ), mock.patch.object(
            gate, "_child_loopback_claim", side_effect=lambda: order.append(("loopback", None)) or {
                "loopback_up": False, "loopback_ipv4_configured": False, "loopback_ipv6_configured": False
            }
        ), mock.patch.object(
            gate, "_drop_all_capabilities", side_effect=lambda: order.append(("drop_caps", None))
        ), mock.patch.object(gate.binary_gate, "_child_parent_death_guard") as pdeath:
            claim = gate._child_namespace_setup(os.getpid(), 1000, 1000)
        self.assertFalse(claim["loopback_up"])
        self.assertEqual(order[0], ("unshare", gate.CLONE_NEWUSER))
        self.assertEqual(order[1], ("setgroups", []))
        self.assertEqual(order[2], ("/proc/self/setgroups", b"deny\n"))
        self.assertEqual(order[3], ("/proc/self/uid_map", b"1000 1000 1\n"))
        self.assertEqual(order[4], ("/proc/self/gid_map", b"1000 1000 1\n"))
        self.assertEqual(order[5], ("unshare", gate.CLONE_NEWNET))
        self.assertEqual(order[-1], ("drop_caps", None))
        pdeath.assert_called_once()

    def test_parent_launcher_uses_namespace_and_exec_ack_and_closes_ns_fds(self) -> None:
        pairs = iter(((10, 11), (12, 13), (14, 15), (16, 17), (18, 19), (20, 21)))
        leader = {"pid": 4242, "state": "S", "ppid": os.getpid(), "pgrp": 4242, "starttime_ticks": 123}
        streams = [mock.Mock(spec=io.RawIOBase) for _ in range(3)]
        events = []
        with mock.patch.object(gate.binary_gate, "_thread_count", return_value=1), mock.patch.object(
            gate, "_pipe_cloexec", side_effect=lambda: next(pairs)
        ), mock.patch.object(gate.os, "fork", return_value=4242), mock.patch.object(
            gate, "_read_stage_one"
        ), mock.patch.object(gate.binary_gate, "_pid_group_record", return_value=leader), mock.patch.object(
            gate, "_pidfd_open", return_value=70
        ), mock.patch.object(
            gate, "_read_ready_payload", return_value={"loopback_up": False, "loopback_ipv4_configured": False, "loopback_ipv6_configured": False}
        ), mock.patch.object(
            gate, "_namespace_observation", return_value=(safe_namespace_evidence(), (80, 81))
        ), mock.patch.object(
            gate.os, "write", side_effect=lambda fd, data: events.append(("write", fd, data)) or len(data)
        ), mock.patch.object(
            gate.os, "close", side_effect=lambda fd: events.append(("close", fd))
        ), mock.patch.object(
            gate.os, "fdopen", side_effect=streams
        ):
            pre_exec = mock.Mock()
            process = gate._spawn_machine_none(90, 91, self.policy, pre_exec)
        self.assertEqual([event[2] for event in events if event[0] == "write"], [b"N", b"E"])
        pre_exec.assert_called_once()
        exec_ack_index = events.index(next(event for event in events if event[0] == "write" and event[2] == b"E"))
        self.assertLess(events.index(("close", 80)), exec_ack_index)
        self.assertLess(events.index(("close", 81)), exec_ack_index)
        self.assertEqual(process.pinned_fds, [70])
        self.assertEqual(process.namespace["verified_before_exec"], True)

    def test_expired_stage_one_deadline_sends_no_ack(self) -> None:
        with mock.patch.object(gate.time, "monotonic", return_value=10.0), mock.patch.object(
            gate.select, "select", side_effect=AssertionError("select reached after deadline")
        ) as selected, mock.patch.object(gate.os, "read", side_effect=AssertionError("read reached")):
            with self.assertRaisesRegex(gate.QemuMachineNoneError, "deadline"):
                gate._read_stage_one(99, 9.0)
        selected.assert_not_called()

    def test_publication_umask_is_fixed_and_restored(self) -> None:
        with mock.patch.object(gate.binary_gate, "_thread_count", return_value=1), mock.patch.object(
            gate.os, "umask", side_effect=[0o022, 0o027]
        ) as changed:
            with gate.fixed_publication_umask():
                pass
        self.assertEqual(changed.call_args_list, [mock.call(0o027), mock.call(0o022)])

    def test_multithreaded_gate_is_rejected_before_umask_change(self) -> None:
        with mock.patch.object(gate.binary_gate, "_thread_count", return_value=2), mock.patch.object(
            gate.os, "umask", side_effect=AssertionError("umask changed")
        ) as changed:
            with self.assertRaisesRegex(gate.QemuMachineNoneError, "single-threaded"):
                with gate.fixed_publication_umask():
                    pass
        changed.assert_not_called()

    def test_attempt_marker_is_o_excl_self_hashed_and_never_overwritten(self) -> None:
        with tempfile.TemporaryDirectory(prefix="machine-none-attempt-") as temporary:
            audit = Path(temporary)
            _, attempt_schema_hash = gate.read_json_and_hash(gate.ATTEMPT_SCHEMA_PATH)
            with mock.patch.object(gate, "AUDIT_DIR", audit):
                result = REAL_WRITE_ATTEMPT_MARKER("a" * 64, "b" * 64, attempt_schema_hash, "d" * 64)
                path = audit / gate.ATTEMPT_NAME
                before = path.read_bytes()
                marker = json.loads(before)
                core = dict(marker)
                inner = core.pop("marker_hash")
                self.assertEqual(inner, gate.safety.canonical_hash(core))
                self.assertEqual(result["file_sha256"], hashlib.sha256(before).hexdigest())
                with self.assertRaises(FileExistsError):
                    REAL_WRITE_ATTEMPT_MARKER("a" * 64, "b" * 64, attempt_schema_hash, "d" * 64)
                self.assertEqual(path.read_bytes(), before)

    def test_residue_treats_marker_or_partial_as_consumed(self) -> None:
        with tempfile.TemporaryDirectory(prefix="machine-none-residue-") as temporary:
            audit = Path(temporary)
            (audit / gate.ATTEMPT_NAME).write_text("{}\n", encoding="utf-8")
            with mock.patch.object(gate, "AUDIT_DIR", audit):
                self.assertIn(gate.ATTEMPT_NAME, gate._residue())
            (audit / gate.ATTEMPT_NAME).unlink()
            partial = f".{gate.ATTEMPT_NAME}.partial.crash"
            (audit / partial).write_text("", encoding="utf-8")
            with mock.patch.object(gate, "AUDIT_DIR", audit):
                self.assertIn(partial, gate._residue())

    def test_published_receipt_readback_is_nofollow_strict_and_mode_bound(self) -> None:
        with tempfile.TemporaryDirectory(prefix="machine-none-receipt-") as temporary:
            audit = Path(temporary)
            path = audit / gate.RECEIPT_NAME
            payload = b'{"status":"test"}\n'
            path.write_bytes(payload)
            path.chmod(0o640)
            with mock.patch.object(gate, "AUDIT_DIR", audit):
                decoded, digest, observed = gate.read_published_receipt(path)
            self.assertEqual(decoded, {"status": "test"})
            self.assertEqual(digest, hashlib.sha256(payload).hexdigest())
            self.assertEqual(observed["mode"], "0640")
            path.write_bytes(b'{"x":1,"x":2}\n')
            path.chmod(0o640)
            with mock.patch.object(gate, "AUDIT_DIR", audit), self.assertRaises(gate.QemuMachineNoneError):
                gate.read_published_receipt(path)
            path.unlink()
            target = audit / "target"
            target.write_text("{}\n", encoding="utf-8")
            target.chmod(0o640)
            path.symlink_to(target.name)
            with mock.patch.object(gate, "AUDIT_DIR", audit), self.assertRaises(OSError):
                gate.read_published_receipt(path)

    def test_success_path_revalidates_published_receipt_before_return(self) -> None:
        source = Path(gate.__file__).read_text(encoding="utf-8")
        published = source.index("safety.atomic_write_json_new(receipt_path, report)")
        reread = source.index("stored, _, _ = read_published_receipt(receipt_path)")
        stored_schema = source.index('validate_with_schema(stored, receipt_schema, "stored machine-none receipt")')
        stored_semantic = source.index("validate_report_semantics(stored, policy)")
        returned = source.index("return stored", stored_semantic)
        self.assertLess(published, reread)
        self.assertLess(reread, stored_schema)
        self.assertLess(stored_schema, stored_semantic)
        self.assertLess(stored_semantic, returned)


class QmpProtocolTests(MockOnlyTestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, _ = gate.read_json_and_hash(gate.POLICY_PATH)

    def test_qmp_wire_is_canonical_bounded_and_exact(self) -> None:
        wires = [gate._canonical_qmp_wire(item) for item in gate.QMP_COMMANDS]
        self.assertTrue(all(item.endswith(b"\r\n") for item in wires))
        self.assertLessEqual(sum(map(len, wires)), gate.MAX_COMMAND_BYTES)
        self.assertEqual(json.loads(wires[0]), {"execute": "qmp_capabilities", "id": "capabilities"})

    def test_strict_qmp_parser_rejects_duplicate_nan_invalid_utf8_and_prequit_event(self) -> None:
        bad = (
            b'{"id":"x","id":"y","return":{}}\n',
            b'{"id":"x","return":NaN}\n',
            b'\xff\n',
            b'{"event":"STOP"}\n',
            b'{"return":{}}\n{"return":{}}\n',
        )
        for payload in bad:
            with self.subTest(payload=payload), self.assertRaises(gate.QemuMachineNoneError):
                gate._strict_qmp_frame(payload)

    def test_only_exact_post_quit_shutdown_event_is_allowed(self) -> None:
        valid = {
            "event": "SHUTDOWN",
            "data": {"guest": False, "reason": "host-qmp-quit"},
            "timestamp": {"seconds": 1, "microseconds": 2},
        }
        self.assertTrue(gate._is_exact_shutdown_event(valid))
        for changed in (
            dict(valid, event="STOP"),
            dict(valid, data={"guest": True, "reason": "host-qmp-quit"}),
            dict(valid, timestamp={"seconds": 1, "microseconds": 1_000_000}),
        ):
            self.assertFalse(gate._is_exact_shutdown_event(changed))

    def test_qmp_transcript_rejects_wrong_id_error_event_extra_and_duplicate(self) -> None:
        good = [gate.EXPECTED_QMP_GREETING, *gate.EXPECTED_QMP_PRE_QUIT_RESPONSES]
        gate.validate_qmp_frames(good)
        mutations = []
        wrong_id = copy.deepcopy(good)
        wrong_id[1]["id"] = "wrong"
        mutations.append(wrong_id)
        error = copy.deepcopy(good)
        error[2] = {"error": {"class": "GenericError", "desc": "x"}, "id": "kvm"}
        mutations.append(error)
        event = copy.deepcopy(good)
        event[3] = {"event": "STOP"}
        mutations.append(event)
        mutations.append([*good, {"return": {}, "id": "unknown"}])
        mutations.append([*good, gate.EXPECTED_QMP_QUIT_RESPONSE, gate.EXPECTED_QMP_QUIT_RESPONSE])
        for frames in mutations:
            with self.subTest(frames=frames), self.assertRaises(gate.QemuMachineNoneError):
                gate.validate_qmp_frames(frames)

    def test_qmp_parser_rejects_overlong_or_unterminated_frame(self) -> None:
        with self.assertRaises(gate.QemuMachineNoneError):
            gate._strict_qmp_frame(b'{"return":{}}')
        payload = b'{"x":"' + b"a" * gate.MAX_QMP_LINE + b'"}\n'
        with self.assertRaises(gate.QemuMachineNoneError):
            gate._strict_qmp_frame(payload)

    def _run_fake_qmp(self, include_quit: bool, include_shutdown: bool, *, stderr_payload: bytes = b"", returncode: int = 0) -> dict:
        def wire(frame: dict) -> bytes:
            return json.dumps(frame, sort_keys=True, separators=(",", ":")).encode("utf-8") + b"\r\n"

        greeting = wire(gate.EXPECTED_QMP_GREETING)
        chunks = [
            ("exec_error", b""),
            ("stdout", greeting[:11]),
            ("stdout", greeting[11:]),
        ]
        for response in gate.EXPECTED_QMP_PRE_QUIT_RESPONSES:
            encoded = wire(response)
            chunks.extend((("stdout", encoded[:5]), ("stdout", encoded[5:])))
        post = b""
        if include_quit:
            post += wire(gate.EXPECTED_QMP_QUIT_RESPONSE)
        if include_shutdown:
            post += wire({
                "event": "SHUTDOWN",
                "data": {"guest": False, "reason": "host-qmp-quit"},
                "timestamp": {"seconds": 1, "microseconds": 2},
            })
        if post:
            chunks.append(("stdout", post))
        chunks.append(("stdout", b""))
        if stderr_payload:
            chunks.append(("stderr", stderr_payload))
        chunks.append(("stderr", b""))

        class FakeSelector:
            def __init__(self) -> None:
                self.registered = {}
                self.current = b""

            def register(self, stream, _events, label) -> None:
                self.registered[label] = stream

            def unregister(self, stream) -> None:
                for label, current in list(self.registered.items()):
                    if current is stream:
                        del self.registered[label]

            def get_map(self):
                return self.registered

            def select(self, _timeout):
                label, payload = chunks.pop(0)
                self.current = payload
                return [(SimpleNamespace(data=label, fileobj=self.registered[label]), None)]

            def close(self) -> None:
                pass

        selector = FakeSelector()
        streams = []
        for fd in (10, 11, 12):
            stream = mock.Mock()
            stream.fileno.return_value = fd
            streams.append(stream)
        process = SimpleNamespace(
            pid=4242,
            qmp_write=13,
            stdout=streams[0], stderr=streams[1], exec_error=streams[2],
            identity={"pid": 4242, "pgid": 4242, "starttime_ticks": 123, "started_monotonic_ns": 1},
            namespace=safe_namespace_evidence(),
            deadline_monotonic=time.monotonic() + 60,
            returncode=None,
            close_pipes=mock.Mock(),
            wait=mock.Mock(side_effect=lambda timeout: setattr(process, "returncode", returncode) or returncode),
        )
        zombie = [{"pid": 4242, "state": "Z", "starttime_ticks": 123, "pgrp": 4242}]
        with mock.patch.object(gate.selectors, "DefaultSelector", return_value=selector), mock.patch.object(
            gate.os, "set_blocking"
        ), mock.patch.object(
            gate.os, "read", side_effect=lambda fd, limit: selector.current
        ), mock.patch.object(
            gate.os, "write", side_effect=lambda fd, data: len(data)
        ), mock.patch.object(
            gate.os, "close"
        ), mock.patch.object(
            gate, "_post_greeting_attestation", return_value={
                "pid_starttime_pgid_stable": True, "user_namespace_unchanged": True,
                "network_namespace_unchanged": True, "supplementary_groups_empty": True,
                "capability_sets_zero": True, "no_new_privs": True, "seccomp_mode_filter": True,
                "fd_classes": {"pipe": 3, "anon_inode": 0, "dev_null": 0, "other": 0},
                "forbidden_fd_count": 0, "verified_after_qmp_greeting": True,
            }
        ), mock.patch.object(
            gate.binary_gate, "_process_group_members", side_effect=[zombie, []]
        ), mock.patch.object(
            gate.binary_gate, "_only_leader_zombie", return_value=True
        ), mock.patch.object(
            gate.binary_gate, "_ensure_probe_stopped"
        ):
            return gate._execute_qmp(process, gate.EXPECTED_QEMU_SHA256, self.policy)

    def test_qmp_stream_handles_fragmentation_coalescing_response_and_shutdown(self) -> None:
        result = self._run_fake_qmp(include_quit=True, include_shutdown=True)
        self.assertEqual(result["qmp_frame_count"], 7)
        self.assertTrue(result["quit_response_observed"])
        self.assertTrue(result["shutdown_event_observed"])
        self.assertEqual(result["returncode"], 0)

    def test_qmp_quit_clean_eof_without_response_is_accepted(self) -> None:
        result = self._run_fake_qmp(include_quit=False, include_shutdown=False)
        self.assertEqual(result["qmp_frame_count"], 5)
        self.assertFalse(result["quit_response_observed"])
        self.assertFalse(result["shutdown_event_observed"])
        self.assertTrue(result["quit_command_write_complete"])

    def test_qmp_stderr_and_nonzero_exit_fail_closed(self) -> None:
        with self.assertRaisesRegex(gate.QemuMachineNoneError, "stderr"):
            self._run_fake_qmp(False, False, stderr_payload=b"warning")
        with self.assertRaisesRegex(gate.QemuMachineNoneError, "exit zero"):
            self._run_fake_qmp(False, False, returncode=1)

    def test_qmp_absolute_deadline_fails_before_any_command(self) -> None:
        streams = []
        for fd in (10, 11, 12):
            stream = mock.Mock()
            stream.fileno.return_value = fd
            streams.append(stream)
        process = SimpleNamespace(
            pid=4242, qmp_write=13, stdout=streams[0], stderr=streams[1], exec_error=streams[2],
            identity={"pid": 4242, "pgid": 4242, "starttime_ticks": 123, "started_monotonic_ns": 1},
            namespace=safe_namespace_evidence(), deadline_monotonic=0.0, returncode=None,
            close_pipes=mock.Mock(),
        )
        selector = mock.Mock()
        with mock.patch.object(gate.selectors, "DefaultSelector", return_value=selector), mock.patch.object(
            gate.os, "set_blocking"
        ), mock.patch.object(gate.binary_gate, "_ensure_probe_stopped") as stopped:
            with self.assertRaisesRegex(gate.QemuMachineNoneError, "deadline"):
                gate._execute_qmp(process, gate.EXPECTED_QEMU_SHA256, self.policy)
        stopped.assert_called_once_with(process)

    def test_source_sets_qemu_fd_cloexec_before_exact_fd_exec(self) -> None:
        source = Path(gate.__file__).read_text(encoding="utf-8")
        cloexec = source.index("fcntl.fcntl(qemu_fd, fcntl.F_SETFD, fcntl.FD_CLOEXEC)")
        execve = source.index('os.execve(f"/proc/self/fd/{qemu_fd}"')
        self.assertLess(cloexec, execve)
        self.assertNotIn("RLIMIT_NPROC", source)


if __name__ == "__main__":
    unittest.main()
