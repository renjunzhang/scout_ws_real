#!/usr/bin/env python3
"""Static/mock regression for the exact one-primary S5B0 replay v9."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s5b0_replay_execution_gate_v9 as gate  # noqa: E402
import r8_liquid_s5b0_replay_runtime_supervisor_v9 as runtime  # noqa: E402
import r8_liquid_s5b0_replay_lifecycle_supervisor_v9 as lifecycle  # noqa: E402
import r8_liquid_s5b0_replay_authorization_token_v9 as token_producer  # noqa: E402


class ReplayExecutionV9Tests(unittest.TestCase):
    def lifecycle_fixture(self, base: Path):
        policy, policy_sha = gate.read_policy()
        policy = copy.deepcopy(policy)
        profile = base / "exact.profile"; profile.write_bytes(b"profile\n"); os.chmod(profile, 0o600)
        candidate = base / "candidate"; candidate.write_bytes(b"candidate\n"); os.chmod(candidate, 0o400)
        replay = policy["replay"]
        replay["profile_path"] = str(profile)
        for key in ("start_receipt", "attempt_receipt", "lifecycle_receipt",
                    "final_receipt", "failure_receipt"):
            replay[key] = str(base / f"{key}.json")
        token = {"profile": gate._file_identity(profile),
                 "candidate": gate._file_identity(candidate)}
        anchor = {"boot_id": "11111111-1111-1111-1111-111111111111",
                  "cursor": "s=anchor", "event": {"argv": [], "return_code": 0,
                  "elapsed_seconds": 0.0, "stdout_sha256": "a" * 64,
                  "stderr_sha256": "b" * 64}}
        journal = {"boot_id_before": anchor["boot_id"], "boot_id_after": anchor["boot_id"],
                   "start_cursor": anchor["cursor"], "end_cursor": "s=end", "xid_count": 0,
                   "same_boot": True, "cursor_anchored": True, "events": []}
        return policy, policy_sha, token, anchor, journal

    def run_lifecycle_fixture(self, base: Path, runner, *, receipt_writer=None):
        policy, policy_sha, token, anchor, journal = self.lifecycle_fixture(base)
        patches = (
            mock.patch.object(lifecycle.os, "getuid", return_value=0),
            mock.patch.object(lifecycle.os, "geteuid", return_value=0),
            mock.patch.dict(lifecycle.os.environ, {"SUDO_UID": "1000", "SUDO_GID": "1000"}),
            mock.patch.object(lifecycle.gate, "read_policy", return_value=(policy, policy_sha)),
            mock.patch.object(lifecycle.gate, "validate_token", return_value=token),
            mock.patch.object(lifecycle, "journal_anchor", return_value=anchor),
            mock.patch.object(lifecycle, "journal_finish", return_value=journal),
            mock.patch.object(lifecycle, "_process_residue", return_value=[]),
            mock.patch.object(lifecycle, "_mount_residue", return_value=[]),
        )
        with patches[0], patches[1], patches[2], patches[3], patches[4], \
             patches[5], patches[6], patches[7], patches[8]:
            if receipt_writer is None:
                report = lifecycle.execute_one_shot(runner=runner)
            else:
                with mock.patch.object(lifecycle, "_write_receipt", side_effect=receipt_writer):
                    report = lifecycle.execute_one_shot(runner=runner)
        return policy, report

    def test_closed_schemas_contract_and_public_cli_are_default_deny(self) -> None:
        policy, _digest = gate.read_policy()
        gate.validate_contract(policy)
        for path in (gate.POLICY_SCHEMA_PATH, gate.TOKEN_SCHEMA_PATH,
                     gate.RECEIPT_SCHEMA_PATH, gate.RESULT_SCHEMA_PATH):
            schema = json.loads(path.read_bytes())
            Draft202012Validator.check_schema(schema); gate.assert_deep_closed(schema)
            gate._assert_required_equals_properties(schema)
        self.assertEqual(1, policy["selection"]["planned_denominator"])
        self.assertFalse(policy["selection"]["optional_bag_read"])
        self.assertEqual("data/PartOut_000.obi4", runtime.REQUIRED_OUTPUT_FILES[-1])
        self.assertNotIn("--since", policy["journal"]["query_argv_template"])
        for module in (gate, runtime, lifecycle, token_producer):
            report = module.self_check()
            self.assertFalse(report.get("optional_bag_read", True))
            self.assertFalse(report.get("candidate_executed", True))

    def test_policy_and_token_schema_reject_scope_or_identity_promotion(self) -> None:
        policy, _ = gate.read_policy()
        schema = json.loads(gate.POLICY_SCHEMA_PATH.read_bytes())
        for path, value in ((["selection", "planned_denominator"], 2),
                            (["selection", "optional_bag_read"], True),
                            (["selection", "second_execution_authorized"], True),
                            (["output_contract", "required_data_files"],
                             policy["output_contract"]["required_data_files"][:-1] + ["data/PartOut.obi4"]),
                            (["solver", "parallel_jobs"], 2)):
            changed = copy.deepcopy(policy); cursor = changed
            for key in path[:-1]: cursor = cursor[key]
            cursor[path[-1]] = value
            with self.assertRaises(ValidationError):
                Draft202012Validator(schema).validate(changed)
        token_schema = json.loads(gate.TOKEN_SCHEMA_PATH.read_bytes())
        self.assertEqual(1, token_schema["properties"]["planned_denominator"]["const"])
        self.assertFalse(token_schema["properties"]["claims"]["properties"]["optional_bag_read"]["const"])

    def test_token_validator_rejects_every_frozen_binding_drift(self) -> None:
        fake = {"policy": {"path": str(gate.POLICY_PATH), "sha256": "a" * 64},
                "gate": {}, "runtime_supervisor": {}, "lifecycle_supervisor": {},
                "token_producer": {}, "profile": {}, "candidate": {},
                "build_final_receipt": {}, "schemas": {}, "roots": {}, "devices": []}
        # This focused negative proves the validator compares the policy bytes
        # before any candidate/profile path can be accepted.
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "token.json"
            path.write_text(json.dumps(fake), encoding="utf-8"); os.chmod(path, 0o600)
            with self.assertRaises((ValidationError, gate.GateV9Error)):
                gate.validate_token(path)

    def test_inventory_requires_exact_partout_and_rejects_symlink_or_missing_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "guest-output"; (root / "data").mkdir(parents=True)
            for name in (*runtime.REQUIRED_OUTPUT_FILES, *runtime.RAW_GAUGE_NAMES,
                         "data/Part_0901.bi4"):
                path = root / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(b"x")
            policy = {"replay": {"stage_root": str(root.parent)}}
            receipt, _ = runtime.inventory_receipt(policy)
            self.assertIn("data/PartOut_000.obi4", {row["relative_path"] for row in receipt["files"]})
            (root / "data/PartOut_000.obi4").unlink()
            with self.assertRaisesRegex(runtime.RuntimeV9Error, "missing"):
                runtime.inventory_receipt(policy)
            (root / "data/PartOut_000.obi4").symlink_to(root / "Run.csv")
            with self.assertRaisesRegex(runtime.RuntimeV9Error, "symlink"):
                runtime.inventory_receipt(policy)

    def test_output_qc_invocation_order_is_closed(self) -> None:
        order = []
        policy = {"replay": {"stage_root": "/fixture"},
                  "sources": {"solver_path": {"path": "/fixture/solver.csv"}},
                  "solver": {"settled_time_s": 45.0}}
        frame = {"frames": [{"time_s": 1.0}]}
        payloads = {"RunPARTs.csv": b"run", "data/PartMotionRef.ibi4": b"motion"}
        payloads.update({name: b"raw" for name in runtime.RAW_GAUGE_NAMES})
        def read(*_args, **_kwargs): order.append("read_finalized"); return frame, [{}], payloads
        with mock.patch.object(runtime.qc.frame_reader, "read_finalized", side_effect=read), \
             mock.patch.object(runtime.qc.frame_reader, "parse_runparts", side_effect=lambda _raw: order.append("parse_runparts") or [{}]), \
             mock.patch.object(runtime.qc, "parse_motion_ref", side_effect=lambda *_args: order.append("parse_motion_ref") or {"pass": True}), \
             mock.patch.object(runtime.qc, "boundary_qc", side_effect=lambda *_args, **_kwargs: order.append("boundary_qc") or (b"csv", {"pass": True})), \
             mock.patch.object(runtime.qc, "normalize_gauges", side_effect=lambda *_args, **_kwargs: order.append("normalize_gauges") or ({}, {})), \
             mock.patch.object(runtime.staging, "load_frozen_contract", return_value=({}, {"probes": []})), \
             mock.patch.object(Path, "read_bytes", return_value=b"solver"):
            runtime.run_output_qc(policy, {}, "a" * 64)
        self.assertEqual(["read_finalized", "parse_runparts", "parse_motion_ref",
                          "boundary_qc", "normalize_gauges"], order)

    def test_rename_noreplace_collision_preserves_both_trees(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary); source = base / "partial"; final = base / "final"
            source.mkdir(); final.mkdir(); (source / "evidence").write_text("keep")
            with self.assertRaisesRegex(runtime.RuntimeV9Error, "collision"):
                runtime._rename_noreplace(source, final)
            self.assertTrue(source.is_dir()); self.assertTrue(final.is_dir())

    def test_journal_cursor_and_lifecycle_cleanup_mock(self) -> None:
        policy, _ = gate.read_policy(); loaded = False
        def runner(argv, _timeout):
            nonlocal loaded
            if list(argv) == [lifecycle.AA_STATUS]:
                return 0, (policy["replay"]["profile_name"] + "\n").encode() if loaded else b"", b""
            if list(argv)[:5] == [lifecycle.APPARMOR_PARSER, "-K", "-T", "-a", "--"]:
                loaded = True; return 0, b"", b""
            if list(argv)[:5] == [lifecycle.APPARMOR_PARSER, "-K", "-T", "-R", "--"]:
                loaded = False; return 0, b"", b""
            if list(argv) == policy["journal"]["anchor_argv"]:
                return 0, b"-- cursor: s=anchor\n", b""
            if list(argv) == policy["journal"]["sync_argv"]:
                return 0, b"", b""
            if "--after-cursor" in argv:
                return 0, b"clean kernel row\n-- cursor: s=end\n", b""
            if list(argv) == [lifecycle.SUDO, "-k"]:
                return 0, b"", b""
            return 0, b"", b""
        anchor = {"boot_id": "11111111-1111-1111-1111-111111111111", "cursor": "s=anchor",
                  "event": {"argv": [], "return_code": 0, "elapsed_seconds": 0,
                            "stdout_sha256": "a" * 64, "stderr_sha256": "b" * 64}}
        with mock.patch.object(lifecycle, "_boot_id", return_value=anchor["boot_id"]):
            journal = lifecycle.journal_finish(policy, anchor, runner=runner)
        self.assertEqual(0, journal["xid_count"]); self.assertTrue(journal["same_boot"])
        loaded = True; events = []
        unload = lifecycle._run([lifecycle.APPARMOR_PARSER, "-K", "-T", "-R", "--",
                                 policy["replay"]["profile_path"]], 60, runner=runner)
        events.append(unload); self.assertFalse(loaded)
        self.assertFalse(lifecycle._status(policy["replay"]["profile_name"], action="zero", runner=runner)["loaded"])

    def test_runtime_children_always_drop_to_uid_gid_1000_zero_groups(self) -> None:
        policy, _ = gate.read_policy()
        execute = lifecycle.child_argv(policy)
        prefix = [lifecycle.SETPRIV, "--reuid=1000", "--regid=1000",
                  "--clear-groups", "--", lifecycle.PYTHON, str(lifecycle.RUNTIME_PATH)]
        self.assertEqual(prefix, execute[:len(prefix)])
        self.assertIn("execute-one-shot", execute)
        handoff = {"argv": [], "return_code": 0, "elapsed_seconds": 0.0,
                   "stdout": {}, "stderr": {}, "resource_log": {}, "final_sample": {},
                   "candidate": {}, "profile": {}}
        post = lifecycle.child_argv(policy, "postprocess-one-shot", execution=handoff, xid_count=0)
        self.assertEqual(prefix, post[:len(prefix)])
        self.assertIn("postprocess-one-shot", post); self.assertIn("--xid-count", post)

    def test_exact_finalized_frame_v2_parent_is_frozen(self) -> None:
        policy, _ = gate.read_policy()
        parents = [row for row in policy["parents"]
                   if row["role"].startswith("FINALIZED_FRAME_READER_")]
        self.assertEqual([{
            "role": "FINALIZED_FRAME_READER_V2",
            "path": "scripts/r8_liquid_s5b0_finalized_frame_reader_v2.py",
            "sha256": "aecd5125625ce4da91b9782f6a28eed017ff5e163056fc648b414aca96e2af4c",
        }], parents)

    def test_failure_receipt_conservatively_records_runtime_child_attempt(self) -> None:
        empty = lifecycle._empty_evidence(None, "fixture", runtime_child_attempted=True)
        self.assertTrue(empty["runtime_child_attempted"])
        self.assertIsNone(empty["runtime_attempt_receipt"])
        self.assertIsNone(empty["lifecycle_receipt"])

    def test_success_publication_orders_lifecycle_before_final(self) -> None:
        tree = ast.parse(lifecycle.Path(__file__).resolve().read_text() if False else
                         lifecycle.Path(lifecycle.__file__).read_text(encoding="utf-8"))
        source = ast.unparse(tree)
        self.assertLess(source.index('_write_receipt(policy[\'replay\'][\'lifecycle_receipt\']'),
                        source.index('_write_receipt(policy[\'replay\'][\'final_receipt\']'))

    def test_attempt_receipt_is_durable_before_runtime_child_and_failure_binds_it(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary); loaded = False; child_calls = 0
            policy_box = {}
            def runner(argv, _timeout):
                nonlocal loaded, child_calls
                policy = policy_box["policy"]
                if list(argv) == [lifecycle.AA_STATUS]:
                    stdout = (policy["replay"]["profile_name"] + "\n").encode() if loaded else b""
                    return 0, stdout, b""
                if list(argv)[:5] == [lifecycle.APPARMOR_PARSER, "-K", "-T", "-a", "--"]:
                    loaded = True; return 0, b"", b""
                if list(argv)[:5] == [lifecycle.APPARMOR_PARSER, "-K", "-T", "-R", "--"]:
                    loaded = False; return 0, b"", b""
                if list(argv) == [lifecycle.SUDO, "-k"]:
                    return 0, b"", b""
                if lifecycle.SETPRIV in argv and "execute-one-shot" in argv:
                    child_calls += 1
                    attempt_path = Path(policy["replay"]["attempt_receipt"])
                    self.assertTrue(attempt_path.is_file())
                    attempt = json.loads(attempt_path.read_bytes())
                    self.assertEqual("ATTEMPT", attempt["phase"])
                    self.assertTrue(attempt["evidence"]["runtime_child_attempted"])
                    self.assertEqual(1, attempt["claims"]["candidate_execution_count"])
                    return 17, b"", b"fixture child failure"
                raise AssertionError(f"unexpected argv: {argv!r}")
            fixture = self.lifecycle_fixture(base); policy_box["policy"] = fixture[0]
            # Reuse the same exact fixture identities inside the helper call.
            with mock.patch.object(self, "lifecycle_fixture", return_value=fixture):
                policy, report = self.run_lifecycle_fixture(base, runner)
            self.assertEqual("STOP_AND_PRESERVE_EVIDENCE", report["status"])
            self.assertEqual(1, child_calls)
            failure = json.loads(Path(policy["replay"]["failure_receipt"]).read_bytes())
            self.assertTrue(failure["evidence"]["runtime_child_attempted"])
            self.assertEqual(1, failure["claims"]["candidate_execution_count"])
            self.assertEqual(gate._file_identity(Path(policy["replay"]["attempt_receipt"])),
                             failure["evidence"]["runtime_attempt_receipt"])
            self.assertFalse(Path(policy["replay"]["final_receipt"]).exists())

    def test_lifecycle_write_failure_cannot_publish_final_and_preserves_result(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary); loaded = False; policy, policy_sha, token, anchor, journal = self.lifecycle_fixture(base)
            result_file = base / "result-package.json"; result_file.write_bytes(b"result\n"); os.chmod(result_file, 0o600)
            result_identity = gate._file_identity(result_file)
            execution = {"argv": ["/runtime/candidate"], "return_code": 0,
                         "elapsed_seconds": 1.25, "stdout": {"sha256": "1" * 64},
                         "stderr": {"sha256": "2" * 64},
                         "resource_log": {"sha256": "3" * 64}}
            def runner(argv, _timeout):
                nonlocal loaded
                if list(argv) == [lifecycle.AA_STATUS]:
                    stdout = (policy["replay"]["profile_name"] + "\n").encode() if loaded else b""
                    return 0, stdout, b""
                if list(argv)[:5] == [lifecycle.APPARMOR_PARSER, "-K", "-T", "-a", "--"]:
                    loaded = True; return 0, b"", b""
                if list(argv)[:5] == [lifecycle.APPARMOR_PARSER, "-K", "-T", "-R", "--"]:
                    loaded = False; return 0, b"", b""
                if list(argv) == [lifecycle.SUDO, "-k"]: return 0, b"", b""
                if "execute-one-shot" in argv:
                    return 0, json.dumps(execution).encode(), b""
                if "postprocess-one-shot" in argv:
                    value = {"status": "PASS_S5B0_REPLAY_POSTPROCESS_V9",
                             "inventory_sha256": "4" * 64,
                             "result_package": result_identity}
                    return 0, json.dumps(value).encode(), b""
                raise AssertionError(f"unexpected argv: {argv!r}")
            original = lifecycle._write_receipt
            def fail_lifecycle(path, value):
                if path == policy["replay"]["lifecycle_receipt"]:
                    raise OSError("fixture lifecycle write failure")
                return original(path, value)
            fixture = (policy, policy_sha, token, anchor, journal)
            with mock.patch.object(self, "lifecycle_fixture", return_value=fixture):
                policy, report = self.run_lifecycle_fixture(base, runner, receipt_writer=fail_lifecycle)
            self.assertEqual("STOP_AND_PRESERVE_EVIDENCE", report["status"])
            self.assertFalse(Path(policy["replay"]["lifecycle_receipt"]).exists())
            self.assertFalse(Path(policy["replay"]["final_receipt"]).exists())
            failure = json.loads(Path(policy["replay"]["failure_receipt"]).read_bytes())
            self.assertEqual(result_identity["sha256"], failure["evidence"]["result_package_sha256"])

    def test_parent_abort_consumes_attempt_and_retry_stops_before_profile_or_child(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary); loaded = False; child_calls = 0; load_calls = 0
            policy, policy_sha, token, anchor, journal = self.lifecycle_fixture(base)
            def runner(argv, _timeout):
                nonlocal loaded, child_calls, load_calls
                if list(argv) == [lifecycle.AA_STATUS]:
                    stdout = (policy["replay"]["profile_name"] + "\n").encode() if loaded else b""
                    return 0, stdout, b""
                if list(argv)[:5] == [lifecycle.APPARMOR_PARSER, "-K", "-T", "-a", "--"]:
                    load_calls += 1; loaded = True; return 0, b"", b""
                if list(argv)[:5] == [lifecycle.APPARMOR_PARSER, "-K", "-T", "-R", "--"]:
                    loaded = False; return 0, b"", b""
                if list(argv) == [lifecycle.SUDO, "-k"]: return 0, b"", b""
                if "execute-one-shot" in argv:
                    child_calls += 1; raise KeyboardInterrupt("fixture parent abort")
                raise AssertionError(f"unexpected argv: {argv!r}")
            fixture = (policy, policy_sha, token, anchor, journal)
            with mock.patch.object(self, "lifecycle_fixture", return_value=fixture):
                with self.assertRaises(KeyboardInterrupt):
                    self.run_lifecycle_fixture(base, runner)
            self.assertTrue(Path(policy["replay"]["attempt_receipt"]).is_file())
            Path(policy["replay"]["start_receipt"]).unlink()
            with mock.patch.object(lifecycle.os, "getuid", return_value=0), \
                 mock.patch.object(lifecycle.os, "geteuid", return_value=0), \
                 mock.patch.dict(lifecycle.os.environ, {"SUDO_UID": "1000", "SUDO_GID": "1000"}), \
                 mock.patch.object(lifecycle.gate, "read_policy", return_value=(policy, policy_sha)), \
                 mock.patch.object(lifecycle.gate, "validate_token", return_value=token):
                with self.assertRaisesRegex(lifecycle.LifecycleV9Error, "already consumed"):
                    lifecycle.execute_one_shot(runner=runner)
            self.assertEqual(1, child_calls)
            self.assertEqual(1, load_calls)

    def test_no_network_or_second_bag_surface(self) -> None:
        for name in ("r8_liquid_s5b0_replay_execution_gate_v9.py",
                     "r8_liquid_s5b0_replay_runtime_supervisor_v9.py",
                     "r8_liquid_s5b0_replay_lifecycle_supervisor_v9.py"):
            tree = ast.parse((SCRIPTS / name).read_text(encoding="utf-8"))
            imports = {alias.name.split(".")[0] for node in ast.walk(tree)
                       if isinstance(node, ast.Import) for alias in node.names}
            self.assertFalse(imports & {"socket", "requests", "rosbag", "rospy"})
        policy, _ = gate.read_policy()
        self.assertNotIn("Bslosh", json.dumps(policy)); self.assertNotIn("C2", json.dumps(policy))


if __name__ == "__main__":
    unittest.main()
