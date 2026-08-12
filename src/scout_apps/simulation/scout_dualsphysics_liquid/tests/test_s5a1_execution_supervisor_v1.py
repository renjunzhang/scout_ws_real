#!/usr/bin/env python3
"""Mock/static-only tests for the S5A1 one-shot execution layer."""

from __future__ import annotations

import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_execution_supervisor_v1 as supervisor  # noqa: E402


class S5A1ExecutionSupervisorV1Tests(unittest.TestCase):
    def test_schema_is_valid_and_deep_closed(self) -> None:
        schema = json.loads(supervisor.RECEIPT_SCHEMA.read_bytes())
        Draft202012Validator.check_schema(schema)
        supervisor.gate.assert_schema_deep_closed(schema)
        self.assertIs(schema["additionalProperties"], False)
        mock_receipt = supervisor.mock_failure_receipt()
        Draft202012Validator(schema).validate(mock_receipt)
        changed = dict(mock_receipt)
        changed["unexpected"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)

    def test_self_check_is_static_and_reverifies_all_committed_parents(self) -> None:
        report = supervisor.static_self_check()
        self.assertEqual(report["status"], "PASS_S5A1_EXECUTION_SUPERVISOR_V1_SELF_CHECK")
        self.assertEqual(report["verified_parent_count"], 10)
        self.assertFalse(report["real_bag_read_attempted"])
        self.assertFalse(report["real_bag_read_confirmed"])
        self.assertFalse(report["worker_run"])
        self.assertFalse(report["bwrap_run"])
        self.assertFalse(report["files_written"])
        self.assertFalse(report["network_used"])
        self.assertFalse(report["sudo_used"])

    def test_exact_roots_match_frozen_incoming_policy(self) -> None:
        policy = json.loads(supervisor.POLICY.read_bytes())
        self.assertEqual(Path(policy["package"]["planned_partial_root"]), supervisor.PARTIAL_ROOT)
        self.assertEqual(Path(policy["package"]["planned_final_root"]), supervisor.FINAL_ROOT)
        self.assertEqual(supervisor.PARTIAL_ROOT.parent, Path("/home/zrj/scout_liquid_lab/incoming"))
        self.assertEqual(supervisor.FINAL_ROOT.parent, supervisor.PARTIAL_ROOT.parent)

    def test_bwrap_argv_is_networkless_clearenv_bounded_and_exact_one_shot(self) -> None:
        receipt = Path("/home/zrj/scout_liquid_lab/audits/exact.json")
        source = Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag")
        argv = supervisor.build_bwrap_argv(receipt, source, supervisor.PARTIAL_ROOT)
        joined = "\0".join(argv)
        self.assertEqual(argv.count("--unshare-all"), 1)
        self.assertEqual(argv.count("--clearenv"), 1)
        self.assertEqual(argv.count("--die-with-parent"), 1)
        self.assertIn("--as=1073741824", argv)
        self.assertIn("180s", argv)
        self.assertEqual(argv.count(str(source)), 3)
        self.assertEqual(argv.count(str(receipt)), 3)
        self.assertEqual(argv.count(str(supervisor.PARTIAL_ROOT)), 3)
        self.assertNotIn("sudo", joined.lower())
        self.assertNotIn("/dev/nvidia", joined.lower())
        self.assertNotIn("--share-net", argv)
        self.assertNotIn("/proc", argv)

    def test_only_partial_root_is_writable_bind(self) -> None:
        receipt = Path("/home/zrj/scout_liquid_lab/audits/exact.json")
        source = Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag")
        argv = supervisor.build_bwrap_argv(receipt, source, supervisor.PARTIAL_ROOT)
        writable = []
        for index, token in enumerate(argv):
            if token == "--bind":
                writable.append((argv[index + 1], argv[index + 2]))
        self.assertEqual(writable, [(str(supervisor.PARTIAL_ROOT), str(supervisor.PARTIAL_ROOT))])

    def test_actual_fd_bind_closes_receipt_and_source_toctou_window(self) -> None:
        argv = supervisor.build_bwrap_argv(
            supervisor.S5A0_INNER_RECEIPT,
            Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
            supervisor.PARTIAL_ROOT,
            receipt_fd=41,
            source_fd=42,
        )
        self.assertIn("/proc/self/fd/41", argv)
        self.assertIn("/proc/self/fd/42", argv)
        self.assertEqual(argv.count("/proc/self/fd/41"), 1)
        self.assertEqual(argv.count("/proc/self/fd/42"), 1)

    def test_s5a0_outer_and_inner_paths_are_exact(self) -> None:
        s5a0_policy = json.loads(supervisor.S5A0_SUPERVISOR_POLICY.read_bytes())
        self.assertEqual(
            supervisor.S5A0_FINAL_RECEIPT,
            Path(s5a0_policy["paths"]["final_receipt_path"]),
        )
        self.assertEqual(
            supervisor.S5A0_EVIDENCE_ROOT,
            Path(s5a0_policy["paths"]["final_evidence_root"]),
        )
        self.assertEqual(
            supervisor.S5A0_INNER_RECEIPT,
            supervisor.S5A0_EVIDENCE_ROOT / "selected_bag_inner_receipt_v1.json",
        )
        self.assertEqual(
            supervisor.S5A0_FINAL_SCHEMA.name,
            "target_host_s5a0_primary_supervisor_execution_receipt_v1.json",
        )
        with self.assertRaisesRegex(supervisor.ExecutionError, "frozen exact path"):
            supervisor.resolve_s5a0_final_receipt(
                Path("/home/zrj/scout_liquid_lab/audits/wrong.final.json"),
                "0" * 64,
                json.loads(supervisor.POLICY.read_bytes()),
            )

    def test_worker_code_freezes_uid_and_real_bag_truth(self) -> None:
        source = supervisor.SCRIPT.read_text(encoding="utf-8")
        self.assertIn('os.getuid() != 1000 or os.getgid() != 1000', source)
        self.assertIn('real_bag_read=True', source)
        self.assertIn('source_bag_executed": False', source)
        self.assertIn('optional_bag_read": False', source)
        self.assertIn('ros_started": False', source)
        self.assertIn('solver_executed": False', source)

    def test_file_identity_rejects_symlink_and_hash_drift(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "target"
            target.write_bytes(b"safe")
            link = root / "link"
            link.symlink_to(target)
            with self.assertRaises(OSError):
                supervisor.file_identity(link)
            with self.assertRaises(supervisor.ExecutionError):
                supervisor.file_identity(target, "0" * 64)

    def test_opened_fd_identity_is_hashed_without_offset_drift(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "input"
            target.write_bytes(b"opened-fd-contract")
            descriptor = os.open(target, os.O_RDONLY | os.O_NOFOLLOW)
            try:
                os.lseek(descriptor, 3, os.SEEK_SET)
                identity = supervisor.opened_fd_identity(
                    descriptor, target, supervisor.sha256_bytes(target.read_bytes())
                )
                self.assertEqual(os.lseek(descriptor, 0, os.SEEK_CUR), 3)
                self.assertEqual(identity["inode"], target.stat().st_ino)
                with self.assertRaisesRegex(supervisor.ExecutionError, "hash drifted"):
                    supervisor.opened_fd_identity(descriptor, target, "0" * 64)
            finally:
                os.close(descriptor)

    def test_failure_schema_allows_source_drift_and_rejects_pass_contradictions(self) -> None:
        schema = json.loads(supervisor.RECEIPT_SCHEMA.read_bytes())
        failure = supervisor.mock_failure_receipt()
        failure["inputs"]["source_unchanged"] = False
        Draft202012Validator(schema).validate(failure)
        contradictory = supervisor.mock_failure_receipt()
        contradictory["status"] = "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(contradictory)

    def test_reserved_receipt_publishes_noreplace_and_preserves_collision(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            partial, final = root / "receipt.partial", root / "receipt.json"
            descriptor = supervisor.reserve_receipt(partial)
            supervisor.publish_reserved_receipt(
                descriptor, partial, final, supervisor.mock_failure_receipt()
            )
            self.assertFalse(partial.exists())
            self.assertTrue(final.is_file())
            self.assertEqual(stat.S_IMODE(final.stat().st_mode), 0o440)
            collision = root / "collision.json"
            collision.write_text("occupied", encoding="utf-8")
            second_partial = root / "second.partial"
            second = supervisor.reserve_receipt(second_partial)
            with self.assertRaises(OSError):
                supervisor.publish_reserved_receipt(
                    second, second_partial, collision, supervisor.mock_failure_receipt()
                )
            self.assertTrue(second_partial.is_file())

    def test_partial_writer_is_exact_0600_and_collision_preserving(self) -> None:
        policy, identity = supervisor.gate.validate_policy()
        receipt, records, clock, tf = supervisor.gate.mock_inputs(policy)
        package = supervisor.gate.build_package(
            policy=policy, policy_identity=identity, s5a0_receipt_raw=receipt,
            s5a0_receipt_path="/mock/s5a0.json", odom_records=records,
            clock_alignment=clock, tf_alignment=tf, real_bag_read=True)
        with tempfile.TemporaryDirectory() as temporary:
            partial = Path(temporary) / "transfer.partial"
            partial.mkdir(mode=0o700)
            supervisor.write_existing_partial(partial, package)
            self.assertEqual(sorted(path.name for path in partial.iterdir()), sorted(supervisor.gate.PACKAGE_INVENTORY))
            self.assertTrue(all(stat.S_IMODE(path.stat().st_mode) == 0o600 for path in partial.iterdir()))
            with self.assertRaises(supervisor.ExecutionError):
                supervisor.write_existing_partial(partial, package)

    def test_host_package_validation_requires_real_bag_read_true(self) -> None:
        policy, identity = supervisor.gate.validate_policy()
        receipt, records, clock, tf = supervisor.gate.mock_inputs(policy)
        package = supervisor.gate.build_package(
            policy=policy, policy_identity=identity, s5a0_receipt_raw=receipt,
            s5a0_receipt_path="/mock/s5a0.json", odom_records=records,
            clock_alignment=clock, tf_alignment=tf, real_bag_read=False)
        with tempfile.TemporaryDirectory() as temporary:
            partial = Path(temporary) / "transfer.partial"
            partial.mkdir(mode=0o700)
            supervisor.write_existing_partial(partial, package)
            with self.assertRaisesRegex(supervisor.ExecutionError, "real_bag_read=false"):
                supervisor.load_package(partial)

    def test_atomic_rename_is_noreplace_and_preserves_collision(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            target = root / "target"
            source.mkdir()
            target.mkdir()
            with self.assertRaises(OSError) as context:
                supervisor.rename_noreplace(source, target)
            self.assertEqual(context.exception.errno, os.errno.EEXIST if hasattr(os, "errno") else 17)
            self.assertTrue(source.is_dir())
            self.assertTrue(target.is_dir())

    def test_root_identity_rejects_symlink_and_non0700(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bad = root / "bad"
            bad.mkdir(mode=0o755)
            with self.assertRaises(supervisor.ExecutionError):
                supervisor.root_identity(bad)
            link = root / "link"
            link.symlink_to(bad)
            with self.assertRaises(supervisor.ExecutionError):
                supervisor.root_identity(link)


if __name__ == "__main__":
    unittest.main()
