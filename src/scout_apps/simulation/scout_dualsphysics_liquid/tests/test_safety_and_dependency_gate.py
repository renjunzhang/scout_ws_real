#!/usr/bin/env python3
"""No-ROS tests for the P0 liquid dependency and filesystem gates."""

from __future__ import annotations

import contextlib
import hashlib
import io
import json
import os
import signal
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_dependency_gate as dependency  # noqa: E402
import r8_liquid_safety as safety  # noqa: E402


class IsolatedApprovedRoot(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory(prefix="r8-liquid-test-")
        self.base = Path(self.tempdir.name)
        self.parent = self.base / "scout_sim_replacement"
        self.parent.mkdir()
        self.root = self.parent / "r8_liquid"
        self.root_patch = mock.patch.object(safety, "APPROVED_ROOT", self.root)
        self.root_patch.start()

    def tearDown(self) -> None:
        self.root_patch.stop()
        self.tempdir.cleanup()

    def prepare(self) -> None:
        safety.prepare_layout()


class SafetyPathTests(IsolatedApprovedRoot):
    def test_only_exact_narrow_root_is_accepted(self) -> None:
        self.assertEqual(safety.validate_approved_root(self.root), self.root)
        for rejected in (
            Path("relative/r8_liquid"),
            self.parent,
            self.base,
            Path("/data/a"),
            Path("/data"),
            Path("/"),
        ):
            with self.subTest(rejected=rejected):
                with self.assertRaises(safety.LiquidSafetyError):
                    safety.validate_approved_root(rejected)

    def test_symlink_component_and_escape_are_rejected(self) -> None:
        real_parent = self.base / "real"
        real_parent.mkdir()
        link_parent = self.base / "link"
        link_parent.symlink_to(real_parent, target_is_directory=True)
        linked_root = link_parent / "r8_liquid"
        with mock.patch.object(safety, "APPROVED_ROOT", linked_root):
            with self.assertRaises(safety.LiquidSafetyError):
                safety.validate_approved_root(linked_root)

        self.prepare()
        with self.assertRaises(safety.LiquidSafetyError):
            safety.ensure_within_approved_root(self.parent / "outside.json")
        escape = self.root / "scratch" / "escape"
        escape.symlink_to(self.base, target_is_directory=True)
        with self.assertRaises(safety.LiquidSafetyError):
            safety.ensure_within_approved_root(escape / "bad.json")

    def test_prepare_creates_only_frozen_layout(self) -> None:
        result = safety.prepare_layout()
        expected = {self.root / item for item in safety.LAYOUT_DIRS}
        actual = {path for path in self.root.rglob("*") if path.is_dir()}
        self.assertEqual(actual, expected)
        self.assertEqual(set(map(Path, result["created_directories"])), {self.root} | expected)
        self.assertEqual(safety.prepare_layout()["created_directories"], [])

    def test_invalid_prepare_receipt_causes_no_layout_mutation(self) -> None:
        invalid = self.root / "dependency/manifests/not_allowed.json"
        fake_report = {
            "status": "PASS",
            "errors": [],
            "approved_root_exists": False,
            "receipt_hash": "0" * 64,
        }
        with mock.patch.object(safety, "build_preflight", return_value=fake_report):
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(
                    safety.main(["prepare-root", "--receipt", str(invalid)]),
                    2,
                )
        self.assertFalse(self.root.exists())

    def test_prepare_receipt_records_post_creation_state(self) -> None:
        receipt = self.root / "dependency/manifests/safety_preflight_prepare_root_20260805T120000Z.json"

        def fake_preflight(**_kwargs: object) -> dict[str, object]:
            core = {
                "status": "PASS",
                "errors": [],
                "approved_root_exists": self.root.is_dir(),
            }
            return dict(core, receipt_hash=safety.canonical_hash(core))

        with mock.patch.object(safety, "build_preflight", side_effect=fake_preflight):
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(
                    safety.main(["prepare-root", "--receipt", str(receipt)]),
                    0,
                )
        stored = json.loads(receipt.read_text(encoding="utf-8"))
        self.assertTrue(stored["approved_root_exists"])
        self.assertFalse(stored["preflight_before"]["approved_root_exists"])

    def test_atomic_json_never_overwrites_and_retains_failed_partial(self) -> None:
        self.prepare()
        output = self.root / "dependency/manifests/evidence.json"
        safety.atomic_write_json_new(output, {"version": 1})
        first = output.read_bytes()
        with self.assertRaises(FileExistsError):
            safety.atomic_write_json_new(output, {"version": 2})
        self.assertEqual(output.read_bytes(), first)
        partials = list(output.parent.glob(f".{output.name}.partial.*"))
        self.assertEqual(len(partials), 1)

    def test_atomic_json_write_failure_never_publishes_final_name(self) -> None:
        self.prepare()
        output = self.root / "dependency/manifests/failure.json"
        with mock.patch.object(safety.json, "dumps", side_effect=RuntimeError("injected")):
            with self.assertRaises(RuntimeError):
                safety.atomic_write_json_new(output, {"version": 1})
        self.assertFalse(output.exists())
        self.assertEqual(len(list(output.parent.glob(f".{output.name}.partial.*"))), 1)

    def test_directory_publication_is_no_replace(self) -> None:
        self.prepare()
        source = self.root / "scratch/source.partial"
        destination = self.root / "dependency/source/final"
        safety.mkdir_exact(source)
        safety.rename_directory_noreplace(source, destination)
        self.assertFalse(source.exists())
        self.assertTrue(destination.is_dir())

        second_source = self.root / "scratch/second.partial"
        safety.mkdir_exact(second_source)
        with self.assertRaises(safety.LiquidSafetyError):
            safety.rename_directory_noreplace(second_source, destination)
        self.assertTrue(second_source.is_dir())

    def test_resource_reservation_policy(self) -> None:
        capacity = 1000 * safety.GIB
        passed = safety.resource_policy(capacity, 500 * safety.GIB, 100 * safety.GIB)
        self.assertEqual(passed["reservation_bytes"], 150 * safety.GIB)
        self.assertEqual(passed["status"], "PASS")
        failed = safety.resource_policy(capacity, 200 * safety.GIB, 100 * safety.GIB)
        self.assertEqual(failed["status"], "NO_GO")

    def test_mount_table_failure_and_nested_ancestor_fail_closed(self) -> None:
        fixed_root = Path("/data/a/scout_sim_replacement/r8_liquid")
        with mock.patch.object(Path, "read_text", side_effect=OSError("injected")):
            with self.assertRaises(safety.LiquidSafetyError):
                safety._mount_points_affecting_root(fixed_root)
        mountinfo = (
            "1 0 259:7 / /data rw - xfs /dev/nvme1n1 rw\n"
            "2 1 259:7 /a /data/a rw - xfs /dev/nvme1n1 rw\n"
        )
        with mock.patch.object(Path, "read_text", return_value=mountinfo):
            self.assertEqual(
                safety._mount_points_affecting_root(fixed_root),
                ("/data/a",),
            )

    def test_findmnt_failure_and_identity_mismatch_fail_closed(self) -> None:
        with mock.patch.object(safety.subprocess, "run", side_effect=subprocess.TimeoutExpired("findmnt", 5)):
            with self.assertRaises(safety.LiquidSafetyError):
                safety._findmnt_snapshot(Path("/data/a"))
        with mock.patch.object(
            safety,
            "_path_identity",
            return_value={"device_id": 1, "is_symlink": False},
        ), mock.patch.object(
            safety,
            "_findmnt_snapshot",
            side_effect=safety.LiquidSafetyError("injected findmnt failure"),
        ), mock.patch.object(safety, "_listening_ports_from_proc", return_value=()), mock.patch.object(
            safety, "_simulation_processes", return_value=()
        ), mock.patch.object(safety, "_gpu_snapshot", return_value={"compute_processes": []}):
            report = safety.build_preflight(root=self.root)
        self.assertEqual(report["status"], "NO_GO")
        self.assertTrue(any("injected findmnt failure" in error for error in report["errors"]))
        wrong = dict(safety.EXPECTED_STORAGE_IDENTITY, uuid="wrong")
        with self.assertRaises(safety.LiquidSafetyError):
            safety._require_expected_storage_identity(wrong, "test")


class DependencyGateTests(IsolatedApprovedRoot):
    def test_dependency_pin_is_exact(self) -> None:
        self.assertEqual(
            dependency.REPOSITORY_URL,
            "https://github.com/DualSPHysics/DualSPHysics.git",
        )
        self.assertEqual(
            dependency.PINNED_COMMIT,
            "ef3721a861fda961f0e2f9ec4cd317b19de99086",
        )

    def test_command_timeout_stops_owned_descendant_process_group(self) -> None:
        child_pid_path = self.base / "child.pid"
        child_code = (
            "import os,signal,time; os.close(1); os.close(2); "
            "signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)"
        )
        parent_code = (
            "import subprocess,time; "
            f"p=subprocess.Popen(['/usr/bin/python3','-c',{child_code!r}]); "
            f"open({str(child_pid_path)!r},'w').write(str(p.pid)); "
            "time.sleep(30)"
        )
        child_pid = None
        try:
            with self.assertRaises(dependency.DependencyGateError):
                dependency.run_checked(
                    ("/usr/bin/python3", "-c", parent_code),
                    timeout_sec=0.2,
                )
            child_pid = int(child_pid_path.read_text(encoding="utf-8"))
            proc_stat = Path(f"/proc/{child_pid}/stat")
            if proc_stat.exists():
                state = proc_stat.read_text(encoding="utf-8").split()[2]
                self.assertEqual(state, "Z", "owned descendant remained running after timeout")
        finally:
            if child_pid is not None:
                try:
                    os.kill(child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def test_inventory_hashes_elf_license_and_safe_symlink(self) -> None:
        source = self.base / "fixture"
        source.mkdir()
        (source / "LICENSE").write_text("license\n", encoding="utf-8")
        (source / "solver").write_bytes(b"\x7fELFfixture")
        (source / "ordinary.txt").write_text("data\n", encoding="utf-8")
        (source / "ordinary-link").symlink_to("ordinary.txt")
        inventory = dependency.inventory_source(source)
        self.assertEqual(inventory["source_file_count"], 4)
        self.assertEqual([item["path"] for item in inventory["precompiled_elf_artifacts"]], ["solver"])
        self.assertEqual([item["path"] for item in inventory["license_artifacts"]], ["LICENSE"])
        self.assertRegex(inventory["source_inventory_hash"], r"^[0-9a-f]{64}$")

        (source / "outside-link").symlink_to(self.base / "outside")
        (self.base / "outside").write_text("outside\n", encoding="utf-8")
        with self.assertRaises(dependency.DependencyGateError):
            dependency.inventory_source(source)

    def test_inventory_hash_and_elf_magic_share_one_nofollow_descriptor(self) -> None:
        source = self.base / "race-fixture"
        source.mkdir()
        original = b"\x7fELForiginal"
        replacement = b"plain replacement"
        target = source / "solver"
        target.write_bytes(original)
        original_os_open = os.open
        swapped = False

        def swap_after_open(path: object, flags: int, *args: object, **kwargs: object) -> int:
            nonlocal swapped
            fd = original_os_open(path, flags, *args, **kwargs)
            if path == target.name and not swapped:
                swapped = True
                target.rename(source / "solver.opened")
                target.write_bytes(replacement)
            return fd

        with mock.patch.object(dependency.os, "open", side_effect=swap_after_open):
            inventory = dependency.inventory_source(source)
        self.assertTrue(swapped)
        self.assertEqual(
            inventory["precompiled_elf_artifacts"],
            [
                {
                    "path": "solver",
                    "size": len(original),
                    "sha256": hashlib.sha256(original).hexdigest(),
                    "execution_status": "NOT_EXECUTED_NOT_ADMITTED",
                }
            ],
        )

    def test_inventory_rejects_symlink_into_git_metadata(self) -> None:
        source = self.base / "fixture"
        (source / ".git").mkdir(parents=True)
        (source / ".git/config").write_text("metadata\n", encoding="utf-8")
        (source / "metadata-link").symlink_to(".git/config")
        with self.assertRaises(dependency.DependencyGateError):
            dependency.inventory_source(source)

    def test_existing_partial_blocks_before_network_or_receipt_write(self) -> None:
        self.prepare()
        paths = dependency.dependency_paths()
        safety.mkdir_exact(paths["partial"])
        receipt = self.root / "dependency/manifests/dependency_acquire_preflight_20260805T120000Z.json"
        with mock.patch.object(dependency, "run_checked") as runner:
            with self.assertRaises(dependency.DependencyGateError):
                dependency.acquire(
                    preflight_receipt=receipt,
                    manifest_output=paths["manifest"],
                )
        runner.assert_not_called()
        self.assertFalse(receipt.exists())
        self.assertFalse(paths["source"].exists())

    def _create_local_pinned_source(self) -> tuple[dict[str, Path], str]:
        self.prepare()
        paths = dependency.dependency_paths()
        paths["source"].mkdir()
        commands = (
            [str(dependency.GIT), "init", str(paths["source"])],
            [str(dependency.GIT), "-C", str(paths["source"]), "config", "user.name", "Test"],
            [str(dependency.GIT), "-C", str(paths["source"]), "config", "user.email", "test@example.invalid"],
        )
        for command in commands:
            subprocess.run(command, check=True, capture_output=True, text=True)
        (paths["source"] / "LICENSE").write_text("fixture license\n", encoding="utf-8")
        subprocess.run(
            [str(dependency.GIT), "-C", str(paths["source"]), "add", "LICENSE"],
            check=True,
            capture_output=True,
            text=True,
        )
        subprocess.run(
            [str(dependency.GIT), "-C", str(paths["source"]), "commit", "-m", "fixture"],
            check=True,
            capture_output=True,
            text=True,
        )
        commit = subprocess.run(
            [str(dependency.GIT), "-C", str(paths["source"]), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        subprocess.run(
            [
                str(dependency.GIT),
                "-C",
                str(paths["source"]),
                "remote",
                "add",
                "origin",
                dependency.REPOSITORY_URL,
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        return paths, commit

    def test_verify_rejects_manifest_field_tampering_even_if_rehashed(self) -> None:
        paths, commit = self._create_local_pinned_source()
        source_name = f"DualSPHysics_{commit}"
        renamed_source = paths["source"].with_name(source_name)
        paths["source"].rename(renamed_source)
        manifest_path = self.root / "dependency/manifests" / f"DualSPHysics_{commit}.json"
        with mock.patch.object(dependency, "PINNED_COMMIT", commit), mock.patch.object(
            dependency, "SOURCE_DIR_NAME", source_name
        ):
            manifest = dependency.build_manifest(renamed_source)
            safety.atomic_write_json_new(manifest_path, manifest)
            self.assertEqual(dependency.verify_existing()["status"], "PASS")

            tampered = dict(manifest)
            tampered["commit_signature_status"] = "G"
            tampered_core = dict(tampered)
            tampered_core.pop("manifest_hash")
            tampered["manifest_hash"] = safety.canonical_hash(tampered_core)
            manifest_path.write_text(
                json.dumps(tampered, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            with self.assertRaises(dependency.DependencyGateError):
                dependency.verify_existing()

            for invalid_timestamp in (
                "2026-08-05 12:00:00.000000+00:00",
                (datetime.now(timezone.utc) + timedelta(days=1)).isoformat(timespec="microseconds"),
            ):
                tampered = dict(manifest, created_utc=invalid_timestamp)
                tampered_core = dict(tampered)
                tampered_core.pop("manifest_hash")
                tampered["manifest_hash"] = safety.canonical_hash(tampered_core)
                manifest_path.write_text(
                    json.dumps(tampered, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                with self.assertRaises(dependency.DependencyGateError):
                    dependency.verify_existing()

            tampered = dict(manifest)
            tampered["unexpected_claim"] = "not frozen"
            tampered_core = dict(tampered)
            tampered_core.pop("manifest_hash")
            tampered["manifest_hash"] = safety.canonical_hash(tampered_core)
            manifest_path.write_text(
                json.dumps(tampered, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            with self.assertRaises(dependency.DependencyGateError):
                dependency.verify_existing()

    def test_local_fsmonitor_config_cannot_execute_during_git_verification(self) -> None:
        paths, commit = self._create_local_pinned_source()
        marker = self.base / "fsmonitor-ran"
        monitor = self.base / "fsmonitor.sh"
        monitor.write_text(
            f"#!/bin/sh\n/usr/bin/touch {marker}\nexit 0\n",
            encoding="utf-8",
        )
        monitor.chmod(0o700)
        subprocess.run(
            [
                str(dependency.GIT),
                "-C",
                str(paths["source"]),
                "config",
                "core.fsmonitor",
                str(monitor),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        with mock.patch.object(dependency, "PINNED_COMMIT", commit):
            dependency.verify_git_source(paths["source"])
        self.assertFalse(marker.exists(), "repository-local fsmonitor was executed")
        clean_env = dependency._clean_git_environment()
        self.assertEqual(clean_env["GIT_CONFIG_NOSYSTEM"], "1")
        self.assertEqual(clean_env["GIT_CONFIG_GLOBAL"], "/dev/null")
        self.assertEqual(clean_env["GIT_NO_REPLACE_OBJECTS"], "1")
        self.assertEqual(clean_env["GIT_OPTIONAL_LOCKS"], "0")
        self.assertIn("--no-replace-objects", dependency.git_command("status"))


if __name__ == "__main__":
    unittest.main()
