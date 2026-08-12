#!/usr/bin/env python3
"""Static/mock-only tests for the S5A0 one-shot host supervisor."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import os
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SUPERVISOR_PATH = SCRIPTS / "r8_liquid_s5a0_primary_one_shot_supervisor_v1.py"
WORKER_PATH = SCRIPTS / "r8_liquid_s5a0_selected_bag_worker_v3.py"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s5a0_primary_one_shot_supervisor_v1 as supervisor  # noqa: E402
import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate_v1  # noqa: E402


class S5A0PrimarySupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy, self.policy_sha = supervisor.load_policy()
        self.schema, self.schema_sha = supervisor.load_schema()

    def test_self_check_never_reads_primary_or_executes_bwrap(self) -> None:
        with mock.patch.object(supervisor, "capture_source_state", side_effect=AssertionError("primary touched")), \
             mock.patch.object(supervisor.subprocess, "run", side_effect=AssertionError("process launched")):
            result = supervisor.self_check()
        self.assertEqual(result["status"], "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_STATIC_CONTRACT")
        self.assertFalse(result["real_bag_read"])
        self.assertFalse(result["bwrap_executed"])

    def test_worker_imports_under_exact_isolated_python_mode(self) -> None:
        completed = subprocess.run(
            ["/usr/bin/python3", "-I", "-B", str(WORKER_PATH), "self-check"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=5,
        )
        self.assertEqual(
            completed.returncode,
            0,
            completed.stderr.decode("utf-8", "replace"),
        )
        report = json.loads(completed.stdout)
        self.assertEqual(
            report["status"],
            "PASS_S5A0_SELECTED_BAG_WORKER_V3_STATIC_CONTRACT",
        )
        self.assertFalse(report["real_bag_opened"])
        self.assertFalse(report["process_executed"])

    def test_supervisor_imports_under_exact_isolated_python_mode(self) -> None:
        completed = subprocess.run(
            ["/usr/bin/python3", "-I", "-B", str(SUPERVISOR_PATH), "self-check"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=5,
            env={
                "PATH": "/usr/bin",
                "HOME": "/nonexistent",
                "PYTHONDONTWRITEBYTECODE": "1",
                "PYTHONHASHSEED": "0",
            },
        )
        self.assertEqual(
            completed.returncode,
            0,
            completed.stderr.decode("utf-8", "replace"),
        )
        report = json.loads(completed.stdout)
        self.assertEqual(
            report["status"],
            "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_STATIC_CONTRACT",
        )
        self.assertFalse(report["real_bag_read"])
        self.assertFalse(report["bwrap_executed"])

    def test_worker_imports_in_proc_dev_free_bwrap(self) -> None:
        inputs = self.policy["inputs"]
        argv = [
            "/usr/bin/timeout", "--signal=KILL", "5s",
            "/usr/bin/bwrap", "--die-with-parent", "--new-session",
            "--unshare-all", "--unshare-net", "--clearenv",
            "--ro-bind", "/usr", "/usr", "--ro-bind", "/lib", "/lib",
            "--ro-bind", "/lib64", "/lib64", "--dir", "/app",
            "--ro-bind", inputs["reader"]["path"], "/app/r8_liquid_ros1_bag_v2_reader_v1.py",
            "--ro-bind", inputs["gate_v1"]["path"], "/app/r8_liquid_s5a0_selected_bag_intake_gate_v1.py",
            "--ro-bind", inputs["gate_v2"]["path"], "/app/r8_liquid_s5a0_selected_bag_intake_gate_v2.py",
            "--ro-bind", inputs["worker_v3"]["path"], "/app/worker.py",
            "--dir", "/config", "--dir", "/config/target_hosts",
            "--ro-bind", inputs["base_policy"]["path"],
            "/config/target_hosts/liquid_zrj_msi_u2404_s5a0_selected_bag_policy_v1.json",
            "--dir", "/schema", "--ro-bind", inputs["inner_schema"]["path"],
            "/schema/target_host_s5a0_selected_bag_receipt_v1.json",
            "--setenv", "PYTHONDONTWRITEBYTECODE", "1", "--chdir", "/app", "--",
            "/usr/bin/python3", "-I", "-B", "/app/worker.py", "self-check",
        ]
        self.assertNotIn("--proc", argv)
        self.assertNotIn("--dev", argv)
        self.assertNotIn("--bind", argv)
        completed = subprocess.run(
            argv,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=7,
        )
        self.assertEqual(
            completed.returncode,
            0,
            completed.stderr.decode("utf-8", "replace"),
        )
        self.assertEqual(
            json.loads(completed.stdout)["status"],
            "PASS_S5A0_SELECTED_BAG_WORKER_V3_STATIC_CONTRACT",
        )

    def test_exact_argv_has_one_ro_bag_one_evidence_bind_and_no_proc_dev_optional(self) -> None:
        argv, digest = supervisor.build_bwrap_argv(self.policy)
        delimiter = argv.index("--")
        selection, paths = self.policy["selection"], self.policy["paths"]
        self.assertEqual(digest, supervisor._normalised_argv_sha256(argv))
        self.assertEqual(argv.count(selection["absolute_path"]), 1)
        self.assertEqual(argv.count(paths["partial_evidence_root"]), 1)
        self.assertNotIn(paths["audit_root"], argv)
        self.assertNotIn(selection["forbidden_optional_path"], argv)
        self.assertNotIn("--proc", argv[:delimiter])
        self.assertNotIn("--dev", argv[:delimiter])
        self.assertIn("--unshare-net", argv[:delimiter])
        self.assertIn("--ro-bind", argv[:delimiter])
        joined = "\n".join(argv).lower()
        for forbidden in ("sudo", "roscore", "rosbag play", "nvidia-smi", "dualsphysics5.2", "*.bag"):
            self.assertNotIn(forbidden, joined)

    def test_policy_rejects_path_role_optional_and_glob_drift(self) -> None:
        mutations = [
            lambda value: value["selection"].__setitem__("absolute_path", "/tmp/latest.bag"),
            lambda value: value["selection"].__setitem__("selected_role", "OPTIONAL_PAIR"),
            lambda value: value["selection"].__setitem__("optional_pair_authorized", True),
            lambda value: value["selection"].__setitem__("relative_path", "*/capture.bag"),
        ]
        for mutate in mutations:
            changed = copy.deepcopy(self.policy)
            mutate(changed)
            with self.subTest(mutate=mutate), self.assertRaises(supervisor.SupervisorError):
                supervisor.validate_policy(changed)

    def test_execution_schema_is_deep_closed_and_rejects_claim_promotion(self) -> None:
        gate_v1.assert_deep_closed(self.schema)
        context = supervisor._empty_context(self.policy, self.policy_sha, self.schema_sha)
        failure = supervisor.SupervisorError("MOCK_FAILURE", "mock", "static failure")
        receipt = supervisor.build_execution_receipt(
            self.policy, self.schema_sha, context, failure=failure,
            published_path=self.policy["paths"]["failure_receipt_path"],
        )
        self.assertEqual([], list(Draft202012Validator(self.schema).iter_errors(receipt)))
        for mutation in (
            lambda value: value.update({"unknown": True}),
            lambda value: value["claims"].__setitem__("formal", True),
            lambda value: value["sandbox"].__setitem__("proc_mounted", True),
            lambda value: value["selection"].__setitem__("optional_pair_exposed", True),
        ):
            changed = copy.deepcopy(receipt)
            mutation(changed)
            self.assertTrue(list(Draft202012Validator(self.schema).iter_errors(changed)))

    def test_atomic_publish_is_mode_0600_and_never_replaces(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "final.json"
            supervisor.atomic_publish_json(path, {"status": "fixture"}, 4096)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            original = path.read_bytes()
            with self.assertRaisesRegex(supervisor.SupervisorError, "ATOMIC_PUBLISH_FAILED"):
                supervisor.atomic_publish_json(path, {"status": "replacement"}, 4096)
            self.assertEqual(path.read_bytes(), original)

    def test_host_source_state_freezes_parent_mount_and_hash_on_temp_fixture(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "source"
            attempt = root / self.policy["selection"]["attempt_id"]
            attempt.mkdir(parents=True)
            source = attempt / "capture.bag"
            source.write_bytes(b"fixture-only-not-a-real-bag")
            source.chmod(0o755)
            policy = copy.deepcopy(self.policy)
            policy["selection"].update({
                "source_root": str(root), "absolute_path": str(source),
                "relative_path": f"{attempt.name}/capture.bag",
                "expected_size_bytes": source.stat().st_size,
                "expected_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            })
            mountinfo = f"101 1 0:9 / {root} rw - tmpfs fixture rw\n"
            base_policy, _ = gate_v1.load_policy()
            with mock.patch.object(gate_v1, "load_policy", return_value=(base_policy, "0" * 64)):
                state = supervisor.capture_source_state(policy, mountinfo)
            self.assertEqual(state["file"]["sha256"], policy["selection"]["expected_sha256"])
            self.assertEqual(state["mount"]["mount_point"], str(root))
            self.assertEqual(state["path_chain"][-1]["path"], str(source))
            self.assertEqual(state["root"]["root"], str(attempt))
            self.assertEqual(state["root"]["entry_count"], 1)

    def test_host_source_snapshot_never_enumerates_corpus_or_optional_attempt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            corpus = Path(temporary) / "matrix_bags"
            primary = corpus / self.policy["selection"]["attempt_id"]
            optional = corpus / "SIM-S1_CORE_H1_C1_Bslosh_b01_r01"
            primary.mkdir(parents=True)
            optional.mkdir()
            source = primary / "capture.bag"
            source.write_bytes(b"primary-fixture")
            source.chmod(0o755)
            (optional / "capture.bag").write_bytes(b"optional-must-not-be-enumerated")
            policy = copy.deepcopy(self.policy)
            policy["selection"].update({
                "source_root": str(corpus), "absolute_path": str(source),
                "relative_path": f"{primary.name}/capture.bag",
                "expected_size_bytes": source.stat().st_size,
                "expected_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            })
            mountinfo = f"101 1 0:9 / {corpus} rw - tmpfs fixture rw\n"
            base_policy, _ = gate_v1.load_policy()
            original_snapshot = gate_v1.snapshot_source_root
            observed_roots: list[Path] = []

            def bounded_snapshot(root: Path, maximum_entries: int):
                observed_roots.append(Path(root))
                return original_snapshot(root, maximum_entries)

            with mock.patch.object(gate_v1, "load_policy", return_value=(base_policy, "0" * 64)), \
                 mock.patch.object(gate_v1, "snapshot_source_root", side_effect=bounded_snapshot):
                state = supervisor.capture_source_state(policy, mountinfo)
            self.assertEqual(observed_roots, [primary])
            self.assertEqual(state["root"]["root"], str(primary))
            self.assertEqual(state["root"]["entry_count"], 1)

    def _runtime_policy(self, temporary: str) -> dict[str, object]:
        policy = copy.deepcopy(self.policy)
        audit = Path(temporary) / "audits"
        audit.mkdir()
        stem = policy["execution_id"]
        policy["paths"].update({
            "audit_root": str(audit),
            "partial_evidence_root": str(audit / f"{stem}.host.partial"),
            "final_evidence_root": str(audit / f"{stem}.host.evidence"),
            "final_receipt_path": str(audit / f"{stem}.host.final.json"),
            "failure_receipt_path": str(audit / f"{stem}.host.failure.json"),
        })
        return policy

    @staticmethod
    def _source_state() -> dict[str, object]:
        path_row = {"path": "/fixture/capture.bag", "kind": "regular", "mode": "0755", "device": 1, "inode": 2, "nlink": 1, "size_bytes": 10, "mtime_ns": 1, "ctime_ns": 1, "symlink": False}
        mount = {"mount_id": 1, "device": "0:1", "root": "/", "mount_point": "/fixture", "filesystem_type": "tmpfs", "mount_source": "fixture", "mount_options": ["rw"], "super_options": ["rw"]}
        file_row = {"path": "/fixture/capture.bag", "sha256": "1" * 64, "size_bytes": 10, "mode": "0755", "uid": 0, "gid": 0, "device": 1, "inode": 2, "nlink": 1, "mtime_ns": 1, "ctime_ns": 1}
        return {"path_chain": [path_row], "mount": mount, "file": file_row, "root": {"root": "/fixture", "entry_count": 1, "sha256": "2" * 64}}

    def test_mock_nonzero_preserves_partial_and_publishes_failure_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            policy = self._runtime_policy(temporary)
            state = self._source_state()
            def runner(_argv, stdout, stderr, _timeout):
                stdout.write("fixture stdout\n")
                stderr.write("fixture failure\n")
                return 7
            with mock.patch.object(supervisor, "load_policy", return_value=(policy, "3" * 64)), \
                 mock.patch.object(supervisor, "capture_source_state", return_value=state):
                receipt = supervisor.run_one_shot(runner=runner)
            self.assertEqual(receipt["status"], supervisor.FAIL_STATUS)
            self.assertTrue(Path(policy["paths"]["partial_evidence_root"]).is_dir())
            failure_path = Path(policy["paths"]["failure_receipt_path"])
            self.assertTrue(failure_path.is_file())
            self.assertEqual(stat.S_IMODE(failure_path.stat().st_mode), 0o600)
            self.assertFalse(Path(policy["paths"]["final_receipt_path"]).exists())

    def test_unexpected_runner_error_preserves_partial_and_publishes_failure_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            policy = self._runtime_policy(temporary)
            state = self._source_state()

            def runner(_argv, _stdout, _stderr, _timeout):
                raise OSError("fixture launch failure")

            with mock.patch.object(supervisor, "load_policy", return_value=(policy, "3" * 64)), \
                 mock.patch.object(supervisor, "capture_source_state", return_value=state):
                receipt = supervisor.run_one_shot(runner=runner)
            self.assertEqual(receipt["status"], supervisor.FAIL_STATUS)
            self.assertEqual(receipt["failure"]["code"], "UNEXPECTED_EXECUTION_ERROR")
            self.assertTrue(Path(policy["paths"]["partial_evidence_root"]).is_dir())
            self.assertTrue(Path(policy["paths"]["failure_receipt_path"]).is_file())

    def test_mock_success_validates_inner_renames_evidence_and_publishes_final(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            policy = self._runtime_policy(temporary)
            state = self._source_state()
            partial = Path(policy["paths"]["partial_evidence_root"])
            def runner(argv, stdout, stderr, _timeout):
                stdout.write("fixture success\n")
                inner = {
                    "status": gate_v1.PASS_STATUS,
                    "selection": {"absolute_path": policy["selection"]["absolute_path"]},
                    "sandbox": {"bwrap_argv_sha256": argv[argv.index("--expected-argv-sha256") + 1]},
                }
                (partial / policy["paths"]["inner_receipt_name"]).write_text(json.dumps(inner), encoding="utf-8")
                return 0
            with mock.patch.object(supervisor, "load_policy", return_value=(policy, "4" * 64)), \
                 mock.patch.object(supervisor, "capture_source_state", return_value=state), \
                 mock.patch.object(supervisor.gate_v2, "validate_runtime_receipt", return_value=None):
                receipt = supervisor.run_one_shot(runner=runner)
            self.assertEqual(receipt["status"], supervisor.PASS_STATUS)
            self.assertTrue(Path(policy["paths"]["final_evidence_root"]).is_dir())
            final_path = Path(policy["paths"]["final_receipt_path"])
            self.assertTrue(final_path.is_file())
            self.assertEqual(stat.S_IMODE(final_path.stat().st_mode), 0o600)
            self.assertFalse(Path(policy["paths"]["partial_evidence_root"]).exists())

    def test_worker_and_supervisor_static_source_boundaries(self) -> None:
        worker_tree = ast.parse(WORKER_PATH.read_text(encoding="utf-8"))
        supervisor_tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
        worker_roots = set()
        for node in ast.walk(worker_tree):
            if isinstance(node, ast.Import):
                worker_roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                worker_roots.add(node.module.split(".")[0])
        self.assertFalse(worker_roots & {"subprocess", "socket", "rospy", "rosbag", "rclpy"})
        frozen_worker_modules = [
            WORKER_PATH,
            SCRIPTS / "r8_liquid_s5a0_selected_bag_intake_gate_v1.py",
            SCRIPTS / "r8_liquid_s5a0_selected_bag_intake_gate_v2.py",
            SCRIPTS / "r8_liquid_ros1_bag_v2_reader_v1.py",
        ]
        for path in frozen_worker_modules:
            tree = ast.parse(path.read_text(encoding="utf-8"))
            roots: set[str] = set()
            forbidden_calls: list[str] = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    roots.update(alias.name.split(".")[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    roots.add(node.module.split(".")[0])
                elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    name = node.func.attr
                    if name.startswith(("exec", "spawn")) or name in {
                        "system", "popen", "fork", "forkpty",
                    }:
                        forbidden_calls.append(name)
            with self.subTest(path=path.name):
                self.assertFalse(
                    roots & {"subprocess", "socket", "ctypes", "importlib", "runpy", "rospy", "rosbag", "rclpy"}
                )
                self.assertEqual(forbidden_calls, [])
        run_calls = [node for node in ast.walk(supervisor_tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "run"]
        self.assertEqual(len(run_calls), 1)
        self.assertEqual(ast.unparse(run_calls[0].func), "subprocess.run")


if __name__ == "__main__":
    unittest.main()
