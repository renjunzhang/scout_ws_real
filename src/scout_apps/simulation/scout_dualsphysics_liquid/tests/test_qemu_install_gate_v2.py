#!/usr/bin/env python3
"""No-QEMU tests for the append-only QEMU installation gate v2."""

from __future__ import annotations

import contextlib
import copy
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_qemu_install_gate_v2 as gate  # noqa: E402
import r8_liquid_safety as safety  # noqa: E402


class QemuInstallPolicyV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, _ = gate.read_json_and_hash(gate.POLICY_PATH)
        cls.admission_schema, _ = gate.read_json_and_hash(gate.ADMISSION_SCHEMA_PATH)
        cls.preflight_schema, _ = gate.read_json_and_hash(gate.PREFLIGHT_SCHEMA_PATH)

    def test_legacy_v1_artifacts_remain_byte_exact_and_install_false(self) -> None:
        for relative, expected_hash in gate.LEGACY_ARTIFACTS:
            actual_hash, _ = gate.file_hash_and_observation(gate.PACKAGE_ROOT / relative)
            self.assertEqual(actual_hash, expected_hash)
        legacy_vm = json.loads(
            (gate.PACKAGE_ROOT / gate.LEGACY_ARTIFACTS[0][0]).read_text(encoding="utf-8")
        )
        self.assertFalse(legacy_vm["runtime"]["system_install_authorized"])

    def test_golden_policy_and_both_schemas_are_valid(self) -> None:
        gate.validate_with_schema(self.policy, self.admission_schema, "admission")
        gate.validate_policy_semantics(self.policy)
        gate.Draft202012Validator.check_schema(self.admission_schema)
        gate.Draft202012Validator.check_schema(self.preflight_schema)

    def test_schema_rejects_unknown_missing_and_command_fields(self) -> None:
        mutations = []
        unknown = copy.deepcopy(self.policy)
        unknown["unexpected"] = True
        mutations.append(unknown)
        missing = copy.deepcopy(self.policy)
        missing.pop("decision")
        mutations.append(missing)
        command = copy.deepcopy(self.policy)
        command["recorded_install_transaction"]["command"] = "qemu-system-x86_64"
        mutations.append(command)
        hook = copy.deepcopy(self.policy)
        hook["decision"]["hook"] = "/tmp/run"
        mutations.append(hook)
        for mutation in mutations:
            with self.subTest(keys=sorted(mutation)):
                with self.assertRaises(gate.QemuInstallGateError):
                    gate.validate_with_schema(mutation, self.admission_schema, "mutated")

    def test_strict_loader_rejects_duplicate_keys_and_nonfinite_numbers(self) -> None:
        with tempfile.TemporaryDirectory(prefix="qemu-json-v2-") as temporary:
            duplicate = Path(temporary) / "duplicate.json"
            duplicate.write_text('{"value":1,"value":2}\n', encoding="utf-8")
            with self.assertRaises(gate.QemuInstallGateError):
                gate.read_json_and_hash(duplicate)
            nonfinite = Path(temporary) / "nonfinite.json"
            nonfinite.write_text('{"value":NaN}\n', encoding="utf-8")
            with self.assertRaises(gate.QemuInstallGateError):
                gate.read_json_and_hash(nonfinite)

    def test_each_required_package_identity_is_frozen(self) -> None:
        for index, expected in enumerate(gate.EXPECTED_PACKAGES):
            for field, replacement in (
                ("name", "changed-package"),
                ("architecture", "all" if expected[1] == "amd64" else "amd64"),
                ("version", expected[2] + "+changed"),
                ("status", "unknown ok not-installed"),
            ):
                mutation = copy.deepcopy(self.policy)
                mutation["required_packages_exact"][index][field] = replacement
                with self.subTest(package=expected[0], field=field):
                    with self.assertRaises(gate.QemuInstallGateError):
                        gate.validate_policy_semantics(mutation)

    def test_package_closure_and_critical_file_fields_are_frozen(self) -> None:
        mutations = []
        missing = copy.deepcopy(self.policy)
        missing["required_packages_exact"].pop()
        mutations.append(missing)
        namespace = copy.deepcopy(self.policy)
        namespace["required_qemu_namespace_exact"].append("qemu-system-gui")
        mutations.append(namespace)
        absent = copy.deepcopy(self.policy)
        absent["required_absent_packages_exact"].remove("qemu-system-gui")
        mutations.append(absent)
        for field, value in (
            ("sha256", "0" * 64),
            ("mode", "4755"),
            ("uid", 1000),
            ("gid", 1000),
            ("symlink", True),
            ("security_capabilities", ["0102"]),
            ("diverted", True),
        ):
            mutation = copy.deepcopy(self.policy)
            mutation["critical_files_exact"][0][field] = value
            mutations.append(mutation)
        for mutation in mutations:
            with self.subTest():
                with self.assertRaises(gate.QemuInstallGateError):
                    gate.validate_policy_semantics(mutation)

    def test_decision_never_admits_execution_or_future_mutation(self) -> None:
        decision = self.policy["decision"]
        self.assertTrue(decision["observed_installation_state_admitted"])
        for key, value in decision.items():
            if key not in ("observed_installation_state_admitted", "status"):
                self.assertFalse(value, key)


class QemuInstallCollectorV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, _ = gate.read_json_and_hash(gate.POLICY_PATH)

    def test_query_allowlist_never_allows_qemu_apt_sudo_or_shell(self) -> None:
        completed = subprocess.CompletedProcess(
            [str(gate.DPKG), "--print-architecture"], 0, "amd64\n", ""
        )
        with mock.patch.object(gate.subprocess, "run", return_value=completed) as runner:
            self.assertEqual(
                gate.run_collector("dpkg-architecture").stdout,
                "amd64\n",
            )
        kwargs = runner.call_args.kwargs
        self.assertFalse(kwargs["shell"])
        self.assertIs(kwargs["stdin"], subprocess.DEVNULL)
        for forbidden in (
            (str(gate.SYSTEMCTL), "start", "qemu-kvm.service"),
            (str(gate.PRO), "enable", "esm-apps"),
            (str(gate.DPKG), "--configure", "-a"),
            ("/usr/bin/qemu-system-x86_64", "--version"),
            ("/usr/bin/qemu-img", "--version"),
            ("/usr/bin/apt-get", "update"),
            ("/usr/bin/sudo", "true"),
            ("/bin/sh", "-c", "true"),
        ):
            with self.subTest(forbidden=forbidden):
                with self.assertRaises(gate.QemuInstallGateError):
                    gate._execute_readonly_query(forbidden)
        for name, argument in (
            ("package", "unapproved-package"),
            ("owner", "/tmp/not-approved"),
            ("diversion", "/tmp/not-approved"),
            ("unknown", None),
        ):
            with self.subTest(collector=name):
                with self.assertRaises(gate.QemuInstallGateError):
                    gate.run_collector(name, argument)

    def test_quiescence_deviations_all_fail(self) -> None:
        required = self.policy["required_quiescence"]
        service = required["qemu_kvm_service"]
        self.assertEqual(
            gate.quiescence_errors(
                service=service,
                autostart_exists=False,
                autostart_is_symlink=False,
                ksm_start="0",
                ksm_end="0",
                processes=[],
                required=required,
            ),
            [],
        )
        cases = []
        for key, value in (
            ("unit_file_state", "enabled"),
            ("active_state", "active"),
            ("load_state", "not-found"),
            ("sub_state", "failed"),
        ):
            changed = dict(service, **{key: value})
            cases.append(dict(service=changed, autostart_exists=False, autostart_is_symlink=False, ksm_start="0", ksm_end="0", processes=[]))
        cases.extend(
            (
                dict(service=service, autostart_exists=True, autostart_is_symlink=True, ksm_start="0", ksm_end="0", processes=[]),
                dict(service=service, autostart_exists=False, autostart_is_symlink=False, ksm_start="1", ksm_end="1", processes=[]),
                dict(service=service, autostart_exists=False, autostart_is_symlink=False, ksm_start="0", ksm_end="1", processes=[]),
                dict(service=service, autostart_exists=False, autostart_is_symlink=False, ksm_start="0", ksm_end="0", processes=[{"pid": 1}]),
            )
        )
        for case in cases:
            with self.subTest(case=case):
                self.assertTrue(gate.quiescence_errors(required=required, **case))

    def test_process_scan_catches_comm_exe_path_and_inode_without_cmdline(self) -> None:
        def proc_stat(pid: int, starttime: int) -> str:
            return f"{pid} (fixture name) " + " ".join(["S"] + ["0"] * 18 + [str(starttime)]) + "\n"

        with tempfile.TemporaryDirectory(prefix="qemu-proc-v2-") as temporary:
            proc_root = Path(temporary)
            by_comm = proc_root / "100"
            by_comm.mkdir()
            (by_comm / "comm").write_text("qemu-system-x86_64\n", encoding="utf-8")
            (by_comm / "cmdline").write_bytes(b"worker\0")
            (by_comm / "stat").write_text(proc_stat(100, 1000), encoding="ascii")

            by_path = proc_root / "101"
            by_path.mkdir()
            (by_path / "comm").write_text("worker\n", encoding="utf-8")
            (by_path / "cmdline").write_bytes(b"worker\0")
            (by_path / "stat").write_text(proc_stat(101, 1001), encoding="ascii")
            (by_path / "exe").symlink_to("/usr/bin/qemu-img")

            fake_executable = proc_root / "fake-executable"
            fake_executable.write_bytes(b"fixture")
            fake_stat = fake_executable.stat()
            by_inode = proc_root / "102"
            by_inode.mkdir()
            (by_inode / "comm").write_text("worker\n", encoding="utf-8")
            (by_inode / "cmdline").write_bytes(b"worker\0")
            (by_inode / "stat").write_text(proc_stat(102, 1002), encoding="ascii")
            (by_inode / "exe").symlink_to(fake_executable)

            by_other_qemu = proc_root / "103"
            by_other_qemu.mkdir()
            (by_other_qemu / "comm").write_text("worker\n", encoding="utf-8")
            (by_other_qemu / "cmdline").write_bytes(b"worker\0")
            (by_other_qemu / "stat").write_text(proc_stat(103, 1003), encoding="ascii")
            (by_other_qemu / "exe").symlink_to("/usr/bin/qemu-system-i386")

            observations = gate.qemu_process_observations(
                [
                    {
                        "path": "/usr/bin/qemu-system-x86_64",
                        "device_id": fake_stat.st_dev,
                        "inode": fake_stat.st_ino,
                    },
                    {
                        "path": "/usr/bin/qemu-img",
                        "device_id": os.stat("/usr/bin/qemu-img").st_dev,
                        "inode": os.stat("/usr/bin/qemu-img").st_ino,
                    },
                ],
                proc_root=proc_root,
            )
        self.assertEqual([item["pid"] for item in observations], [100, 101, 102, 103])
        self.assertIn("comm", observations[0]["reasons"])
        self.assertIn("exe_path", observations[1]["reasons"])
        self.assertIn("exe_inode", observations[2]["reasons"])
        self.assertIn("exe_path", observations[3]["reasons"])
        self.assertTrue(all("cmdline" not in item for item in observations))

    def test_pid_identity_ignores_dynamic_stat_fields(self) -> None:
        with tempfile.TemporaryDirectory(prefix="qemu-pid-identity-v2-") as temporary:
            entry = Path(temporary) / "300"
            entry.mkdir()
            first = "300 (busy ) name) " + " ".join(["R"] + ["1"] * 18 + ["987654"]) + "\n"
            second = "300 (busy ) name) " + " ".join(["S"] + ["9"] * 18 + ["987654"]) + "\n"
            (entry / "stat").write_text(first, encoding="ascii")
            identity_first = gate._pid_identity(entry)
            (entry / "stat").write_text(second, encoding="ascii")
            identity_second = gate._pid_identity(entry)
        self.assertEqual(identity_first, (300, 987654))
        self.assertEqual(identity_second, identity_first)

    def test_process_visibility_restrictions_fail_closed(self) -> None:
        hidepid_mount = "1 0 0:1 / /proc rw,hidepid=2 - proc proc rw,hidepid=2\n"
        with mock.patch.object(Path, "read_text", return_value=hidepid_mount):
            self.assertTrue(gate._proc_mount_visibility(Path("/proc")))

        with tempfile.TemporaryDirectory(prefix="qemu-proc-hidden-v2-") as temporary:
            proc_root = Path(temporary)
            entry = proc_root / "200"
            entry.mkdir()
            (entry / "stat").write_text(
                "200 (renamed worker) "
                + " ".join(["S"] + ["0"] * 18 + ["2000"])
                + "\n",
                encoding="ascii",
            )
            (entry / "comm").write_text("renamed-worker\n", encoding="utf-8")
            original_read_bytes = Path.read_bytes
            original_readlink = gate.os.readlink

            def denied_cmdline(path: Path) -> bytes:
                if path.name == "cmdline":
                    raise PermissionError("injected")
                return original_read_bytes(path)

            def denied_exe(path: object) -> str:
                if Path(path).name == "exe":
                    raise PermissionError("injected")
                return original_readlink(path)

            with mock.patch.object(Path, "read_bytes", denied_cmdline), mock.patch.object(
                gate.os, "readlink", side_effect=denied_exe
            ):
                snapshot = gate.qemu_process_snapshot([], proc_root=proc_root)
        self.assertTrue(snapshot["visibility_errors"])
        with self.assertRaises(gate.QemuInstallGateError):
            with mock.patch.object(gate, "qemu_process_snapshot", return_value=snapshot):
                gate.qemu_process_observations([])

        with tempfile.TemporaryDirectory(prefix="qemu-proc-spoofed-v2-") as temporary:
            proc_root = Path(temporary)
            entry = proc_root / "201"
            entry.mkdir()
            (entry / "stat").write_text(
                "201 (worker) " + " ".join(["S"] + ["0"] * 18 + ["2001"]) + "\n",
                encoding="ascii",
            )
            (entry / "comm").write_text("ssh-agent\n", encoding="utf-8")
            (entry / "cmdline").write_bytes(
                b"/usr/bin/ssh-agent\0-name\0process=ssh-agent\0"
            )
            with mock.patch.object(gate.os, "readlink", side_effect=PermissionError("injected")):
                spoofed = gate.qemu_process_snapshot([], proc_root=proc_root)
        self.assertTrue(spoofed["visibility_errors"])

    def test_receipt_is_pass_only_create_new_and_preserves_sentinel(self) -> None:
        with tempfile.TemporaryDirectory(prefix="qemu-receipt-v2-") as temporary:
            parent = Path(temporary) / "scout_sim_replacement"
            parent.mkdir()
            root = parent / "r8_liquid"
            with mock.patch.object(safety, "APPROVED_ROOT", root):
                safety.prepare_layout()
                receipt = root / "audits/sandbox/qemu_install_preflight_v2_20260805T160000Z.json"
                with mock.patch.object(gate, "build_report", return_value={"status": "NO_GO"}):
                    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                        self.assertEqual(gate.main(["write-receipt", "--receipt", str(receipt)]), 2)
                self.assertFalse(receipt.exists())

                sentinel = b"do-not-overwrite\n"
                receipt.write_bytes(sentinel)
                with self.assertRaises(gate.QemuInstallGateError):
                    gate.validate_receipt_path(receipt)
                self.assertEqual(receipt.read_bytes(), sentinel)


class QemuInstallReportSemanticV2Tests(unittest.TestCase):
    def _assert_semantic_reject(self, mutation: dict[str, object]) -> None:
        report = copy.deepcopy(self.report)
        cursor: object = report
        path = mutation["path"]
        assert isinstance(path, tuple)
        for key in path[:-1]:
            cursor = cursor[key]  # type: ignore[index]
        cursor[path[-1]] = mutation["value"]  # type: ignore[index]
        core = dict(report)
        core.pop("receipt_hash", None)
        report["receipt_hash"] = safety.canonical_hash(core)
        gate.validate_with_schema(report, self.preflight_schema, "mutated report")
        with self.assertRaises(gate.QemuInstallGateError):
            gate.validate_report_semantics(report, self.policy)

    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, _ = gate.read_json_and_hash(gate.POLICY_PATH)
        cls.preflight_schema, _ = gate.read_json_and_hash(gate.PREFLIGHT_SCHEMA_PATH)
        audit_dir = safety.APPROVED_ROOT / "audits/sandbox"
        before = {path.name for path in audit_dir.iterdir()}
        cls.report = gate.build_report()
        after = {path.name for path in audit_dir.iterdir()}
        if before != after:
            raise AssertionError("read-only build_report changed the audit directory")
        if cls.report["status"] != "PASS":
            raise AssertionError(f"live read-only v2 report is not PASS: {cls.report['errors']}")

    def test_live_report_is_strict_pass_without_pii_or_execution(self) -> None:
        gate.validate_report_semantics(self.report, self.policy)
        serialized = json.dumps(self.report, sort_keys=True)
        for forbidden in ("account", "machine_id", "contract", "cmdline"):
            self.assertNotIn(forbidden, serialized)
        for key in (
            "qemu_executed",
            "qemu_img_executed",
            "qemu_execution_admitted",
            "qemu_img_execution_admitted",
            "vm_started",
            "image_created",
            "image_creation_admitted",
            "build_started",
            "build_admitted",
            "gencase_started",
            "gencase_admitted",
            "upstream_code_executed",
        ):
            self.assertFalse(self.report[key], key)

    def test_contradictory_pass_reports_are_rejected_after_rehash(self) -> None:
        mutations = (
            {"path": ("package_state", "required_packages", 0, "version"), "value": "changed"},
            {"path": ("package_state", "required_absent_packages", 0, "installed"), "value": True},
            {"path": ("package_state", "qemu_namespace_installed"), "value": list(gate.EXPECTED_QEMU_NAMESPACE) + ["qemu-system-gui"]},
            {"path": ("package_state", "dpkg_verify", "stdout_empty"), "value": False},
            {"path": ("critical_files", 0, "dpkg_owner"), "value": "wrong-owner"},
            {"path": ("critical_files", 0, "diverted"), "value": True},
            {"path": ("critical_files", 0, "security_capabilities"), "value": ["0102"]},
            {"path": ("critical_files", 0, "sha256"), "value": "0" * 64},
            {"path": ("quiescence", "qemu_kvm_service", "active_state"), "value": "active"},
            {"path": ("quiescence", "ksm_end_value"), "value": "1"},
            {"path": ("quiescence", "qemu_processes"), "value": [{"pid": 999, "uid": 1000, "comm": "qemu", "exe": None, "reasons": ["comm"]}]},
            {"path": ("quiescence", "process_visibility_errors"), "value": ["hidden"]},
            {"path": ("collection_consistency", "snapshots_equal"), "value": False},
        )
        for mutation in mutations:
            with self.subTest(path=mutation["path"]):
                self._assert_semantic_reject(mutation)

    def test_ab_hashes_are_anchored_to_canonical_snapshots_and_public_evidence(self) -> None:
        both_hashes = copy.deepcopy(self.report)
        both_hashes["collection_consistency"]["snapshot_a_sha256"] = "0" * 64
        both_hashes["collection_consistency"]["snapshot_b_sha256"] = "0" * 64
        core = dict(both_hashes)
        core.pop("receipt_hash", None)
        both_hashes["receipt_hash"] = safety.canonical_hash(core)
        gate.validate_with_schema(both_hashes, self.preflight_schema, "both hashes changed")
        with self.assertRaises(gate.QemuInstallGateError):
            gate.validate_report_semantics(both_hashes, self.policy)

        detached = copy.deepcopy(self.report)
        consistency = detached["collection_consistency"]
        snapshots = []
        for label in ("a", "b"):
            snapshot = json.loads(consistency[f"snapshot_{label}_canonical"])
            snapshot["critical_files"][0]["inode"] += 1
            canonical = safety.canonical_bytes(snapshot).decode("utf-8")
            consistency[f"snapshot_{label}_canonical"] = canonical
            consistency[f"snapshot_{label}_sha256"] = safety.canonical_hash(snapshot)
            snapshots.append(snapshot)
        self.assertEqual(snapshots[0], snapshots[1])
        core = dict(detached)
        core.pop("receipt_hash", None)
        detached["receipt_hash"] = safety.canonical_hash(core)
        gate.validate_with_schema(detached, self.preflight_schema, "detached snapshots")
        with self.assertRaises(gate.QemuInstallGateError):
            gate.validate_report_semantics(detached, self.policy)

    def test_collector_failure_matrix_is_no_go(self) -> None:
        quiescence = self.report["quiescence"]
        package_state = self.report["package_state"]
        base = {
            "required_packages": copy.deepcopy(package_state["required_packages"]),
            "required_absent_packages": copy.deepcopy(package_state["required_absent_packages"]),
            "qemu_namespace_installed": copy.deepcopy(package_state["qemu_namespace_installed"]),
            "dpkg_verify": copy.deepcopy(package_state["dpkg_verify"]),
            "critical_files": copy.deepcopy(self.report["critical_files"]),
            "service_start": copy.deepcopy(quiescence["qemu_kvm_service"]),
            "service_end": copy.deepcopy(quiescence["qemu_kvm_service"]),
            "autostart_start": {"exists": False, "is_symlink": False},
            "autostart_end": {"exists": False, "is_symlink": False},
            "ksm_start": "0",
            "ksm_end": "0",
            "process_start": {"offenders": [], "visibility_errors": []},
            "process_end": {"offenders": [], "visibility_errors": []},
        }
        self.assertEqual(gate.install_snapshot_errors(base, self.policy), [])
        mutations = (
            (("required_packages", 0, "version"), "changed"),
            (("required_absent_packages", 0, "installed"), True),
            (("qemu_namespace_installed",), list(gate.EXPECTED_QEMU_NAMESPACE) + ["qemu-user"]),
            (("dpkg_verify", "returncode"), 1),
            (("critical_files", 0, "dpkg_owner"), "ambiguous"),
            (("critical_files", 0, "diverted"), True),
            (("critical_files", 0, "security_capabilities"), ["0102"]),
            (("critical_files", 0, "sha256"), "0" * 64),
            (("critical_files", 0, "inode"), int(base["critical_files"][0]["inode"]) + 1),
            (("service_end", "active_state"), "active"),
            (("ksm_end",), "1"),
            (("process_end", "offenders"), [{"pid": 999}]),
            (("process_end", "visibility_errors"), ["hidden"]),
        )
        for path, value in mutations:
            snapshot = copy.deepcopy(base)
            cursor: object = snapshot
            for key in path[:-1]:
                cursor = cursor[key]  # type: ignore[index]
            cursor[path[-1]] = value  # type: ignore[index]
            with self.subTest(path=path):
                if path[:2] == ("critical_files", 0) and path[-1] == "inode":
                    self.assertNotEqual(snapshot, base, "A/B identity mutation must differ")
                else:
                    self.assertTrue(gate.install_snapshot_errors(snapshot, self.policy))

    def test_ownership_ambiguity_and_diversion_collectors_fail(self) -> None:
        path = Path("/usr/bin/qemu-img")
        ambiguous = subprocess.CompletedProcess(
            [],
            0,
            "qemu-utils: /usr/bin/qemu-img\nother-owner: /usr/bin/qemu-img\n",
            "",
        )
        with mock.patch.object(gate, "run_collector", return_value=ambiguous):
            with self.assertRaises(gate.QemuInstallGateError):
                gate.package_owner(path)
        diverted = subprocess.CompletedProcess([], 0, "diversion of /usr/bin/qemu-img\n", "")
        with mock.patch.object(gate, "run_collector", return_value=diverted):
            self.assertTrue(gate.is_diverted(path))


if __name__ == "__main__":
    unittest.main()
