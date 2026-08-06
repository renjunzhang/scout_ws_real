#!/usr/bin/env python3
"""Mock-only tests for the QEMU binary-load/version probe gate."""

from __future__ import annotations

import contextlib
import copy
import io
import json
import os
import hashlib
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_qemu_binary_probe_gate as gate  # noqa: E402
import r8_liquid_safety as safety  # noqa: E402


class QemuBinaryProbePolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, _ = gate.read_json_and_hash(gate.POLICY_PATH)
        cls.admission, _ = gate.read_json_and_hash(gate.ADMISSION_SCHEMA_PATH)
        cls.receipt_schema, _ = gate.read_json_and_hash(gate.RECEIPT_SCHEMA_PATH)

    def test_policy_and_schemas_are_strict_and_semantically_frozen(self) -> None:
        gate.validate_with_schema(self.policy, self.admission, "policy")
        gate.validate_policy_semantics(self.policy)
        gate.install_gate.Draft202012Validator.check_schema(self.admission)
        gate.install_gate.Draft202012Validator.check_schema(self.receipt_schema)
        self.assertEqual(tuple(self.policy["binary"]["argv_exact"]), gate.EXACT_QEMU_ARGV)
        self.assertFalse(self.policy["receipt"]["authorization_reusable"])
        self.assertEqual(self.policy["execution_boundary"]["rlimits"]["cpu_seconds_soft"], 2)
        self.assertEqual(self.policy["execution_boundary"]["rlimits"]["cpu_seconds_hard"], 3)
        self.assertEqual(self.policy["execution_boundary"]["rlimits"]["address_space_bytes"], 1024**3)
        self.assertNotIn("processes", self.policy["execution_boundary"]["rlimits"])
        self.assertNotIn("RLIMIT_NPROC", Path(gate.__file__).read_text(encoding="utf-8"))
        self.assertIn("implementation", self.receipt_schema["required"])
        self.assertIn("stability", self.receipt_schema["required"])
        hardening = self.receipt_schema["$defs"]["hardening"]
        self.assertFalse(hardening["additionalProperties"])
        self.assertEqual(hardening["properties"]["rlimits"]["$ref"], "#/$defs/rlimits")
        self.assertFalse(self.receipt_schema["$defs"]["snapshot"]["additionalProperties"])
        self.assertEqual(self.policy["receipt"]["filename_exact"], gate.RECEIPT_NAME)

    def test_unknown_missing_duplicate_and_nonfinite_json_fail_closed(self) -> None:
        unknown = copy.deepcopy(self.policy)
        unknown["command"] = "qemu"
        with self.assertRaises(gate.QemuBinaryProbeError):
            gate.validate_with_schema(unknown, self.admission, "unknown")
        missing = copy.deepcopy(self.policy)
        missing.pop("forbidden_effects")
        with self.assertRaises(gate.QemuBinaryProbeError):
            gate.validate_with_schema(missing, self.admission, "missing")
        with tempfile.TemporaryDirectory(prefix="qemu-probe-json-") as temporary:
            duplicate = Path(temporary) / "duplicate.json"
            duplicate.write_text('{"x":1,"x":2}\n', encoding="utf-8")
            with self.assertRaises(gate.QemuBinaryProbeError):
                gate.read_json_and_hash(duplicate)
            nonfinite = Path(temporary) / "nonfinite.json"
            nonfinite.write_text('{"x":NaN}\n', encoding="utf-8")
            with self.assertRaises(gate.QemuBinaryProbeError):
                gate.read_json_and_hash(nonfinite)

    def test_all_non_probe_effects_and_future_authority_are_false(self) -> None:
        self.assertTrue(all(value is False for value in self.policy["forbidden_effects"].values()))
        self.assertFalse(self.policy["execution_boundary"]["self_check_executes_qemu"])
        serialized = json.dumps(self.policy, sort_keys=True)
        for forbidden in ('"machine_id":', '"account":', '"contract":', '"cmdline":'):
            self.assertNotIn(forbidden, serialized)
        self.assertNotIn("/home/", serialized)
        implementation = gate._implementation_evidence()
        self.assertNotIn("/home/", json.dumps(implementation, sort_keys=True))
        self.assertTrue(implementation["gate_path"].startswith("scripts/"))

    def test_snapshot_errors_cover_package_scratch_ports_and_visibility(self) -> None:
        snapshot = {
            "boot_id_sha256": self.policy["boot_session"]["boot_id_sha256"],
            "runtime_uid": 1000,
            "runtime_gid": 1000,
            "runtime_thread_count": 1,
            "mem_available_bytes": 8 * 1024**3,
            "qemu_kvm_service": dict(gate.EXPECTED_SERVICE),
            "autostart_link_exists": False,
            "autostart_link_is_symlink": False,
            "ksm_value": "0",
            "qemu_processes": [],
            "simulation_processes": [],
            "simulation_ports": [],
            "process_visibility_errors": [],
            "process_visibility_exceptions": [dict(item) for item in gate.UNREADABLE_PROCESS_ALLOWLIST],
            "package_state": {
                "required_package": dict(gate.EXPECTED_QEMU_PACKAGE),
                "qemu_namespace_installed": list(gate.install_gate.EXPECTED_QEMU_NAMESPACE),
                "dpkg_verify": {"returncode": 0, "stdout_empty": True, "stderr_empty": True},
                "binary_dpkg_owner": "qemu-system-x86",
                "binary_diverted": False,
            },
            "scratch": {"path": str(gate.SCRATCH_DIR), "uid": 1000, "gid": 1000, "mode": "0750", "device_id": 1, "inode": 3, "entries": []},
            "lock": {"path": str(gate.LOCK_PATH), "uid": 1000, "gid": 1000, "mode": "0750", "device_id": 1, "inode": 4, "entries": []},
            "implementation": gate._implementation_evidence(),
            "binary": {
                "path": str(gate.QEMU), "uid": 0, "gid": 0, "mode": "0755",
                "size": self.policy["binary"]["size"], "sha256": self.policy["binary"]["sha256"],
                "security_capabilities": [], "file_type": "regular", "symlink": False,
                "device_id": 1, "inode": 2, "mtime_ns": 3,
            },
        }
        self.assertEqual(gate.snapshot_errors(snapshot, self.policy), [])
        for path, value, expected in (
            (("package_state", "dpkg_verify", "returncode"), 1, "dpkg verification"),
            (("scratch", "entries"), ["unexpected"], "scratch directory"),
            (("simulation_ports",), [11311], "reserved port"),
            (("process_visibility_errors",), ["hidden"], "visibility"),
        ):
            changed = copy.deepcopy(snapshot)
            cursor = changed
            for key in path[:-1]:
                cursor = cursor[key]
            cursor[path[-1]] = value
            self.assertTrue(any(expected in item for item in gate.snapshot_errors(changed, self.policy)))

    def test_stable_projection_ignores_only_phase_and_available_memory(self) -> None:
        base = {key: None for key in (
            "boot_id_sha256", "runtime_uid", "runtime_gid", "runtime_thread_count", "qemu_kvm_service",
            "autostart_link_exists", "autostart_link_is_symlink", "ksm_value",
            "qemu_processes", "simulation_processes", "simulation_ports",
            "process_visibility_errors", "process_visibility_exceptions", "package_state",
            "scratch", "lock", "implementation", "binary",
        )}
        one = dict(base, phase="A_PRELOCKED", mem_available_bytes=7)
        two = dict(base, phase="B_PREEXEC", mem_available_bytes=8)
        self.assertEqual(gate.stable_snapshot_projection(one), gate.stable_snapshot_projection(two))
        two["ksm_value"] = "1"
        self.assertNotEqual(gate.stable_snapshot_projection(one), gate.stable_snapshot_projection(two))


class QemuBinaryProbeExecutionMockTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, _ = gate.read_json_and_hash(gate.POLICY_PATH)

    def test_spawn_freezes_argv_environment_cwd_and_same_fd_exec(self) -> None:
        leader = {"pid": 4242, "state": "S", "ppid": os.getpid(), "pgrp": 4242, "starttime_ticks": 123}
        with mock.patch.object(gate, "_thread_count", return_value=1), mock.patch.object(
            gate.os, "fork", return_value=4242
        ), mock.patch.object(gate.select, "select", return_value=([99], [], [])), mock.patch.object(
            gate.os, "read", return_value=b"R"
        ), mock.patch.object(gate.os, "write", return_value=1), mock.patch.object(
            gate, "_pid_group_record", return_value=leader
        ):
            process = gate._spawn_probe(77, 78)
        try:
            self.assertEqual(process.pid, 4242)
            self.assertEqual(process.identity["pgid"], 4242)
            self.assertEqual(tuple(gate.EXACT_QEMU_ARGV), (str(gate.QEMU), "-no-user-config", "-version"))
            self.assertEqual(gate.EXACT_ENV, {"PATH": "/usr/bin:/bin", "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8"})
        finally:
            process.close_pipes()

    def test_execute_probe_is_mocked_and_records_exact_hardening(self) -> None:
        stdout = (self.policy["binary"]["expected_version_line"] + "\n").encode()
        process = mock.Mock(
            returncode=0,
            identity={
                "pid": 4242,
                "pgid": 4242,
                "starttime_ticks": 123,
                "started_monotonic_ns": 100,
            },
        )
        with mock.patch.object(gate, "_spawn_probe", return_value=process) as spawn, mock.patch.object(gate, "_bounded_wait", return_value=(stdout, b"")):
            result = gate.execute_probe(88, 89, gate.EXPECTED_QEMU_SHA256, self.policy)
        spawn.assert_called_once_with(88, 89)
        self.assertEqual(tuple(result["argv"]), gate.EXACT_QEMU_ARGV)
        self.assertEqual(result["environment"], gate.EXACT_ENV)
        self.assertTrue(result["hardening"]["no_new_privs"])
        self.assertTrue(result["hardening"]["same_open_fd_hash_and_exec"])
        self.assertFalse(result["cleanup"]["owned_process_group_residual"])

    def test_timeout_terminates_owned_group_and_never_returns_result(self) -> None:
        class Pipe:
            def fileno(self) -> int:
                return 10

        class Selector:
            def register(self, *args: object) -> None:
                pass
            def get_map(self) -> dict[str, object]:
                return {"still-open": object()}
            def select(self, timeout: float) -> list[object]:
                return []
            def close(self) -> None:
                pass

        process = mock.Mock(
            pid=4242,
            stdout=Pipe(),
            stderr=Pipe(),
            exec_error=Pipe(),
            deadline_monotonic=1.0,
            identity={"pid": 4242, "pgid": 4242, "starttime_ticks": 123},
        )
        with mock.patch.object(gate.selectors, "DefaultSelector", return_value=Selector()), mock.patch.object(
            gate.os, "set_blocking"
        ), mock.patch.object(gate.time, "monotonic", return_value=6.0), mock.patch.object(
            gate, "_ensure_probe_stopped"
        ) as ensure:
            with self.assertRaises(gate.QemuBinaryProbeError):
                gate._bounded_wait(process)
        ensure.assert_called_once_with(process)

    def test_term_then_kill_cleans_descendant_process_group(self) -> None:
        process = mock.Mock(
            pid=4242,
            identity={
                "pid": 4242,
                "pgid": 4242,
                "starttime_ticks": 123,
                "started_monotonic_ns": 100,
            },
        )
        process.wait.return_value = 0
        leader = {"pid": 4242, "state": "R", "ppid": 1, "pgrp": 4242, "starttime_ticks": 123}
        active = [leader, {"pid": 4243, "state": "R", "ppid": 4242, "pgrp": 4242, "starttime_ticks": 124}]
        zombie = [{"pid": 4242, "state": "Z", "ppid": 1, "pgrp": 4242, "starttime_ticks": 123}]
        with mock.patch.object(gate, "_pid_group_record", return_value=leader), mock.patch.object(
            gate.os, "killpg"
        ) as killpg, mock.patch.object(
            gate, "_process_group_members", side_effect=(active, zombie, [])
        ), mock.patch.object(gate.time, "monotonic", side_effect=(0.0, 2.0, 3.0)):
            gate._terminate_owned_group(process)
        signals = [item.args[1] for item in killpg.call_args_list]
        self.assertEqual(signals[0], gate.signal.SIGTERM)
        self.assertIn(gate.signal.SIGKILL, signals[1:])

    def test_self_check_entry_cannot_reach_spawn(self) -> None:
        ready = {"status": "PASS_READY_EXECUTION_NOT_PERFORMED", "qemu_executed": False}
        with mock.patch.object(gate, "build_self_check", return_value=ready), mock.patch.object(gate, "_spawn_probe") as spawn, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(gate.main(["self-check"]), 0)
        spawn.assert_not_called()

    def test_parser_has_no_vm_image_qemu_img_build_or_upstream_entry(self) -> None:
        parser = gate.build_parser()
        subparser_action = next(action for action in parser._actions if isinstance(action, gate.argparse._SubParsersAction))
        self.assertEqual(set(subparser_action.choices), {"self-check", "run-probe"})
        source = Path(gate.__file__).read_text(encoding="utf-8")
        self.assertNotIn("unshare", source.lower())
        self.assertNotIn("qemu-img\",", source)
        self.assertNotIn("subprocess.Popen", source)
        self.assertNotIn("preexec_fn=", source)
        self.assertIn('os.execve(f"/proc/self/fd/{qemu_fd}", list(EXACT_QEMU_ARGV), dict(EXACT_ENV))', source)
        self.assertIn("PR_SET_PDEATHSIG", source)
        self.assertIn("argument_names.update(Path(token).name.lower() for token in argv if token)", source)

    def test_post_ack_handoff_exception_cleans_owned_group(self) -> None:
        leader = {"pid": 4242, "state": "S", "ppid": os.getpid(), "pgrp": 4242, "starttime_ticks": 123}
        with mock.patch.object(gate, "_thread_count", return_value=1), mock.patch.object(
            gate.os, "fork", return_value=4242
        ), mock.patch.object(gate.select, "select", return_value=([99], [], [])), mock.patch.object(
            gate.os, "read", return_value=b"R"
        ), mock.patch.object(gate.os, "write", return_value=1), mock.patch.object(
            gate, "_pid_group_record", return_value=leader
        ), mock.patch.object(
            gate.signal, "pthread_sigmask", side_effect=(set(), KeyboardInterrupt())
        ), mock.patch.object(gate, "_ensure_probe_stopped") as ensure:
            with self.assertRaises(KeyboardInterrupt):
                gate._spawn_probe(77, 78)
        ensure.assert_called_once()

    def test_expired_absolute_deadline_never_sends_exec_ack(self) -> None:
        leader = {"pid": 4242, "state": "S", "ppid": os.getpid(), "pgrp": 4242, "starttime_ticks": 123}
        with mock.patch.object(gate, "_thread_count", return_value=1), mock.patch.object(
            gate.os, "fork", return_value=4242
        ), mock.patch.object(gate.time, "monotonic", side_effect=(0.0, 1.0, 6.0)), mock.patch.object(
            gate.select, "select", return_value=([99], [], [])
        ), mock.patch.object(gate.os, "read", return_value=b"R"), mock.patch.object(
            gate.os, "write"
        ) as write, mock.patch.object(gate, "_pid_group_record", return_value=leader), mock.patch.object(
            gate, "_kill_unready_direct_child"
        ) as kill_direct:
            with self.assertRaises(gate.QemuBinaryProbeError):
                gate._spawn_probe(77, 78)
        write.assert_not_called()
        kill_direct.assert_called_once_with(4242)

    def test_pid_reuse_refuses_to_signal_any_process_group(self) -> None:
        process = mock.Mock(
            pid=4242,
            identity={"pid": 4242, "pgid": 4242, "starttime_ticks": 123, "started_monotonic_ns": 1},
        )
        reused = {"pid": 4242, "state": "R", "ppid": 1, "pgrp": 4242, "starttime_ticks": 999}
        with mock.patch.object(gate, "_pid_group_record", return_value=reused), mock.patch.object(
            gate.os, "killpg"
        ) as killpg:
            with self.assertRaises(gate.QemuBinaryProbeError):
                gate._terminate_owned_group(process)
        killpg.assert_not_called()

    def test_open_fd_remains_bound_after_path_replacement(self) -> None:
        with tempfile.TemporaryDirectory(prefix="qemu-probe-fd-") as temporary:
            path = Path(temporary) / "qemu-system-x86_64"
            original = b"first-audited-elf"
            replacement = b"replacement"
            path.write_bytes(original)
            path.chmod(0o755)
            digest = hashlib.sha256(original).hexdigest()
            policy = copy.deepcopy(self.policy)
            policy["binary"].update({
                "path": str(path), "sha256": digest, "size": len(original),
                "mode": "0755", "uid": os.getuid(), "gid": os.getgid(),
                "security_capabilities": [],
            })
            with mock.patch.object(gate, "QEMU", path), mock.patch.object(
                gate, "EXPECTED_QEMU_SHA256", digest
            ), mock.patch.object(gate.install_gate, "_capabilities", return_value=[]):
                fd, observation = gate.open_verified_qemu_fd(policy)
                replacement_path = path.with_name("replacement")
                replacement_path.write_bytes(replacement)
                os.replace(replacement_path, path)
                try:
                    os.lseek(fd, 0, os.SEEK_SET)
                    self.assertEqual(os.read(fd, len(original)), original)
                    self.assertEqual(observation["sha256"], digest)
                finally:
                    os.close(fd)

    def test_directory_lock_creates_no_lock_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="qemu-probe-lock-") as temporary:
            lock_dir = Path(temporary) / "locks"
            lock_dir.mkdir(mode=0o700)
            with mock.patch.object(gate, "LOCK_PATH", lock_dir):
                before = list(lock_dir.iterdir())
                with gate.exclusive_probe_lock():
                    self.assertEqual(list(lock_dir.iterdir()), before)
                self.assertEqual(list(lock_dir.iterdir()), before)


class QemuBinaryProbeReceiptTests(unittest.TestCase):
    def test_receipt_path_is_create_new_and_preserves_existing_bytes(self) -> None:
        with tempfile.TemporaryDirectory(prefix="qemu-probe-receipt-") as temporary:
            root = Path(temporary) / "r8_liquid"
            with mock.patch.object(safety, "APPROVED_ROOT", root):
                safety.prepare_layout()
                receipt = root / f"audits/sandbox/{gate.RECEIPT_NAME}"
                self.assertEqual(gate.validate_receipt_path(receipt), receipt)
                with self.assertRaises(gate.QemuBinaryProbeError):
                    gate.validate_receipt_path(root / "audits/sandbox/qemu_binary_probe_v1_other.json")
                sentinel = b"do-not-overwrite\n"
                receipt.write_bytes(sentinel)
                with self.assertRaises(gate.QemuBinaryProbeError):
                    gate.validate_receipt_path(receipt)
                self.assertEqual(receipt.read_bytes(), sentinel)

    def test_failed_run_publishes_no_receipt(self) -> None:
        with tempfile.TemporaryDirectory(prefix="qemu-probe-no-pass-") as temporary:
            root = Path(temporary) / "r8_liquid"
            with mock.patch.object(safety, "APPROVED_ROOT", root):
                safety.prepare_layout()
                receipt = root / f"audits/sandbox/{gate.RECEIPT_NAME}"
                with mock.patch.object(gate, "build_probe_receipt", side_effect=gate.QemuBinaryProbeError("mock failure")), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(gate.main(["run-probe", "--receipt", str(receipt)]), 2)
                self.assertFalse(receipt.exists())


if __name__ == "__main__":
    unittest.main()
