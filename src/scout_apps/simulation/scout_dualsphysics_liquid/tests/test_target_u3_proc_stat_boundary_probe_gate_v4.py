"""Pure static/mock tests for the fresh proc-stat AppArmor boundary probe v4.

The suite must never invoke sudo, AppArmor tooling, bwrap, namespaces, the
probe payload, a solver, GenCase, ROS, Gazebo, or GPU tooling.
"""

from __future__ import annotations

import ast
import copy
import errno
import hashlib
import importlib.util
import json
import os
import signal
import stat
import struct
import subprocess
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import pytest
from jsonschema import Draft202012Validator


PACKAGE = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE / "scripts/r8_liquid_target_u3_proc_stat_boundary_probe_gate_v4.py"
SUPERVISOR_PATH = PACKAGE / "scripts/r8_liquid_target_u3_proc_stat_boundary_probe_supervisor_v4.py"
PROFILE_PATH = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-proc-stat-boundary-probe-v4.profile"
POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_proc_stat_boundary_probe_policy_v4.json"
SCHEMA_PATH = PACKAGE / "schema/target_host_u3_proc_stat_boundary_probe_policy_v4.json"
V6_PROFILE_PATH = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-solver-cpu-settle-cold-a-v6.profile"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GATE = load_module(GATE_PATH, "u3_proc_stat_boundary_gate_v4_test")
SUPERVISOR = load_module(SUPERVISOR_PATH, "u3_proc_stat_boundary_supervisor_v4_test")


def policy_and_schema() -> tuple[dict, dict]:
    return (
        json.loads(POLICY_PATH.read_text(encoding="utf-8")),
        json.loads(SCHEMA_PATH.read_text(encoding="utf-8")),
    )


def function_source(path: Path, name: str) -> str:
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    node = next(
        item for item in tree.body
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)) and item.name == name
    )
    lines = source.splitlines()
    assert node.end_lineno is not None
    return "\n".join(lines[node.lineno - 1 : node.end_lineno])


def helper_function_source(name: str) -> str:
    source = GATE.HELPER_SOURCE
    tree = ast.parse(source)
    node = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == name)
    lines = source.splitlines()
    assert node.end_lineno is not None
    return "\n".join(lines[node.lineno - 1 : node.end_lineno])


def guest_identity() -> dict:
    return {
        "uid": [0, 0, 0, 0],
        "gid": [0, 0, 0, 0],
        "groups": [],
        "capabilities": {
            "CapInh": 0,
            "CapPrm": 0,
            "CapEff": 0,
            "CapBnd": 0,
            "CapAmb": 0,
        },
        "no_new_privs": 1,
    }


def member_record(*, pid: int = 3, starttime: int = 12345, state: str = "S") -> dict:
    return {
        "pid": pid,
        "state": state,
        "pgrp": pid,
        "session": pid,
        "starttime": starttime,
        "stat_uid": 65534,
        "stat_gid": 65534,
    }


def valid_success_payload(*, pid: int = 3) -> dict:
    live = member_record(pid=pid, state="S")
    zombie = member_record(pid=pid, state="Z")
    return {
        "status": "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V4",
        "bootstrap_profile": GATE.BOOTSTRAP_PROFILE,
        "bootstrap_label": GATE.BOOTSTRAP_PROFILE + " (enforce)",
        "bootstrap_identity": guest_identity(),
        "work_tmpfs": {
            "filesystem_type": "tmpfs",
            "total_bytes": GATE.TMPFS_BYTES,
            "mount_options": ["rw", "nosuid", "nodev", "relatime"],
        },
        "host_payload_consumed_and_fd0_replaced_with_eof_pipe": True,
        "fds_after_payload": [0, 1, 2],
        "proc_stat_probe": {
            "child_identity": {
                **guest_identity(),
                "pid": pid,
                "pgrp": pid,
                "session": pid,
                "dumpable": 0,
            },
            "frozen_identity": {
                "pid": pid,
                "pgrp": pid,
                "session": pid,
                "starttime": 12345,
                "pidfd_opened": True,
            },
            "live_group_scan": [copy.deepcopy(live)],
            "live_stat_pair": [copy.deepcopy(live), copy.deepcopy(live)],
            "inner_proc_owner_projection": {"uid": 65534, "gid": 65534},
            "status_denial": {
                "attempt_count": 1,
                "path": f"/proc/{pid}/status",
                "errno": errno.EACCES,
            },
            "waitid_wnowait": {"pid": pid, "code": os.CLD_EXITED, "status": 0},
            "zombie_stat_pair": [copy.deepcopy(zombie), copy.deepcopy(zombie)],
            "final_reap": {"pid": pid, "code": os.CLD_EXITED, "status": 0},
            "post_reap_group_scans": [[], []],
        },
        "fds_before_success": [0, 1, 2],
        "host_writable_mounts": [],
    }


def success_frame(value: dict | None = None) -> bytes:
    payload = json.dumps(
        valid_success_payload() if value is None else value,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("ascii")
    return GATE.SUCCESS_MAGIC + struct.pack(">I", len(payload)) + payload + hashlib.sha256(payload).digest()


def failure_frame(
    failure_code: str = "STATUS_UNEXPECTEDLY_READABLE",
    boundary_status: str = "STATUS_READABLE",
) -> bytes:
    payload = json.dumps(
        {"boundary_status": boundary_status, "failure_code": failure_code},
        sort_keys=True,
        separators=(",", ":"),
    ).encode("ascii")
    assert len(payload) <= GATE.FAILURE_PAYLOAD_BYTES
    padding = b"\x00" * (GATE.FAILURE_PAYLOAD_BYTES - len(payload))
    frame = (
        GATE.FAILURE_MAGIC
        + struct.pack(">H", len(payload))
        + payload
        + padding
        + hashlib.sha256(payload).digest()
    )
    assert len(frame) == GATE.EXACT_FAILURE_FRAME_BYTES
    return frame


BOOT_ID = "00112233-4455-6677-8899-aabbccddeeff"
BOOT_COMPACT = BOOT_ID.replace("-", "")
START_CURSOR = f"s=start;b={BOOT_COMPACT};m=1"
END_CURSOR = f"s=end;b={BOOT_COMPACT};m=2"


def exact_status_audit_line(*, child_pid: int = 3, audit_pid: str = "4242") -> str:
    return (
        '2026-08-09T00:00:00.000000+00:00 host kernel: audit: type=1400 '
        'apparmor="DENIED" operation="open" class="file" '
        f'profile="{SUPERVISOR.BOOTSTRAP_PROFILE}" '
        f'name="/proc/{child_pid}/status" pid={audit_pid} comm="python3.12" '
        'requested_mask="r" denied_mask="r" fsuid=1000 ouid=0'
    )


def parsed_status_entry(*, child_pid: int = 3) -> dict:
    return SUPERVISOR._parse_audit_line(exact_status_audit_line(child_pid=child_pid))


def profile_state(*, loaded: bool) -> dict:
    count = 1 if loaded else 0
    return {
        "kernel_exact_counts": {label: count for label in SUPERVISOR.LABELS},
        "kernel_exact_lines": {
            label: ([f"{label} (enforce)"] if loaded else [])
            for label in SUPERVISOR.LABELS
        },
        "kernel_exact_enforce": {label: loaded for label in SUPERVISOR.LABELS},
        "aa_status_exact_presence": {label: loaded for label in SUPERVISOR.LABELS},
        "aa_status_exact_modes": {
            label: ("enforce" if loaded else None) for label in SUPERVISOR.LABELS
        },
        "aa_status_stdout_sha256": "0" * 64,
        "aa_status_stdout_size_bytes": 2,
    }


def empty_label_cleanup() -> dict:
    return {
        "initial": [],
        "term_sent": [],
        "after_term": [],
        "kill_sent": [],
        "stable_zero_scans": [[], [], []],
    }


def journal_sync_evidence() -> dict:
    empty = hashlib.sha256(b"").hexdigest()
    return {
        "argv": ["/usr/bin/journalctl", "--sync"],
        "returncode": 0,
        "stdout_size_bytes": 0,
        "stdout_sha256": empty,
        "stderr_size_bytes": 0,
        "stderr_sha256": empty,
        "failure": None,
        "start_new_session": True,
    }


def valid_audit(*, child_pid: int = 3) -> dict:
    entry = parsed_status_entry(child_pid=child_pid)
    return {
        "capture_valid": True,
        "capture_errors": [],
        "boot_id_before": BOOT_ID,
        "boot_id_after": BOOT_ID,
        "start_cursor": START_CURSOR,
        "end_cursor": END_CURSOR,
        "journal_anchor_argv": [
            "/usr/bin/journalctl", "-k", "--no-pager", "--quiet",
            f"--boot={BOOT_COMPACT}", "--lines=0", "--show-cursor",
        ],
        "journal_query_argv": [
            "/usr/bin/journalctl", "-k", "--no-pager", "--quiet",
            "--output=short-iso-precise", f"--boot={BOOT_COMPACT}",
            f"--after-cursor={START_CURSOR}", "--show-cursor",
        ],
        "raw_stdout_sha256": "1" * 64,
        "raw_stdout_size_bytes": 512,
        "storage_ceiling": SUPERVISOR.AUDIT_STORAGE_CEILING,
        "matching_total": 1,
        "stored_count": 1,
        "dropped_count": 0,
        "storage_overflow": False,
        "expected_status_total": 1,
        "stat_denial_total": 0,
        "unexpected_total": 0,
        "audit_loss_marker_status": "NOT_OBSERVED",
        "audit_loss_marker_matching_total": 0,
        "audit_loss_marker_storage_overflow": False,
        "audit_loss_markers": [],
        "expected_child_pid": child_pid,
        "sanitized_denials": [copy.deepcopy(entry)],
        "expected_status_denials": [copy.deepcopy(entry)],
        "unexpected_denials": [],
        "run_started_epoch": 1.0,
        "run_ended_epoch": 2.0,
        "prequery_label_cleanup": empty_label_cleanup(),
        "profiles_before_audit_query": profile_state(loaded=True),
        "admission_journal_anchor_sha256": "2" * 64,
        "pre_run_journal_sync": journal_sync_evidence(),
        "journal_sync": journal_sync_evidence(),
        "postquery_stable_zero_labels": [[], [], []],
        "profiles_after_audit_query": profile_state(loaded=True),
    }


def valid_gate_result() -> dict:
    frame = success_frame()
    return {
        "argv": ["gate"],
        "returncode": 0,
        "stdout": frame,
        "stderr": b"",
        "stdin_size_bytes": 1,
        "stdin_fully_written": True,
        "failure": None,
        "start_new_session": True,
    }


def valid_sudo_evidence() -> dict:
    clear_argv, verify_argv = SUPERVISOR.sudo_timestamp_argvs()
    empty = hashlib.sha256(b"").hexdigest()
    password = b"sudo: a password is required\n"
    return {
        "scope": "ALL_UID1000_SUDO_TIMESTAMPS_INVALIDATED_BY_SUDO_-K_OTHER_TERMINALS_MUST_REAUTHENTICATE",
        "bounding_mode": SUPERVISOR.SUDO_CLEANUP_BOUNDING_MODE,
        "pty": {
            "stdin_isatty": True,
            "stdin_device": 1,
            "stdin_inode": 2,
            "stdin_rdev": 3,
            "session_id": 44,
            "process_group": 44,
            "foreground_process_group": 44,
        },
        "membership": copy.deepcopy(SUPERVISOR.SUDO_GROUP_MEMBERSHIP_CONTRACT),
        "identity_contract": copy.deepcopy(SUPERVISOR.SUDO_CLEANUP_IDENTITY_CONTRACT),
        "clear": {
            "argv": clear_argv,
            "returncode": 0,
            "stdout_size_bytes": 0,
            "stdout_sha256": empty,
            "stderr_size_bytes": 0,
            "stderr_sha256": empty,
            "failure": None,
            "start_new_session": False,
        },
        "noninteractive_true_must_fail": {
            "argv": verify_argv,
            "returncode": 1,
            "stdout_size_bytes": 0,
            "stdout_sha256": empty,
            "stderr_size_bytes": len(password),
            "stderr_sha256": hashlib.sha256(password).hexdigest(),
            "failure": None,
            "start_new_session": False,
            "stderr_utf8_prefix": password.decode("ascii"),
        },
    }


def test_schema_is_deep_closed_and_policy_is_valid() -> None:
    policy, schema = policy_and_schema()
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(policy)
    GATE._assert_schema_objects_closed(schema)
    GATE._validate_schema_instance(policy, schema, schema)
    SUPERVISOR._assert_schema_objects_closed(schema)
    SUPERVISOR._validate_schema_instance(policy, schema, schema)
    assert set(policy) == set(schema["required"])


def test_fresh_v4_identity_and_policy_boundary_are_exact() -> None:
    policy, _schema = policy_and_schema()
    assert policy["schema_version"] == "smpcc-r8-liquid-target-u3-proc-stat-apparmor-boundary-probe-policy-v4"
    assert policy["policy_id"] == "LIQUID_ZRJ_MSI_U2404_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V4"
    assert GATE.PROBE_ID == SUPERVISOR.PROBE_ID == policy["frozen_identity"]["probe_id"]
    assert GATE.PROBE_ID == "u3_proc_stat_apparmor_boundary_probe_v4_20260809T031752Z"
    assert policy["status"] == (
        "STATIC_READY_V4_PENDING_INDEPENDENT_REVIEW_AND_EXPLICIT_SINGLE_ATTEMPT"
    )
    assert policy["next_allowed_stage"] == (
        "INDEPENDENT_STATIC_REVIEW_ONLY_THEN_EXPLICIT_ROOT_SNAPSHOT_SINGLE_ATTEMPT"
    )
    assert policy["trusted_artifacts"]["all_frozen"] is True
    assert policy["reviewed_bytes"]["all_frozen"] is True
    assert policy["snapshot_bootstrap"]["all_frozen"] is True
    assert policy["snapshot_bootstrap"]["reviewed_minimal_bootstrap_executed_as_root"] is True
    assert policy["fixed_commands"]["argv_hashes_frozen"] is True
    expected_bwrap_hash = hashlib.sha256(
        json.dumps(GATE.bwrap_argv(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    assert policy["fixed_commands"]["bwrap_argv_sha256"] == expected_bwrap_hash
    assert GATE.PAYLOAD_MAGIC == b"R8PROCSTATPROBE4\x00"
    assert GATE.SUCCESS_MAGIC == b"R8PROCSTATPASSV4\x00"
    assert GATE.ADMISSION_TOKEN == SUPERVISOR.ADMISSION_TOKEN == policy["authorization"]["admission_token"]
    assert policy["authorization"]["attempts_per_identity"] == 1
    assert policy["authorization"]["same_identity_retry"] == "forbidden"
    assert policy["authorization"]["solver_or_gencase_authorized"] is False
    assert policy["authorization"]["production_authorized"] is False
    candidate = policy["candidate_boundary"]
    assert candidate["candidate_rule"] == "/proc/[0-9]*/stat r,"
    assert candidate["candidate_rule_count"] == 1
    assert candidate["candidate_rule_profile"] == "bootstrap_only"
    assert candidate["candidate_delta_from_v6"] == (
        "full_effective_authority_additions_after_exact_label_normalization_equal_only_the_bootstrap_numeric_pid_stat_rule"
    )
    assert candidate["status_rule_present"] is False
    assert candidate["status_boundary_control"] == {
        "path_template": "/proc/<CHILD_PID>/status",
        "open_attempts": 1,
        "required_errno": errno.EACCES,
        "required_errno_name": "EACCES",
        "no_retry_in_cleanup": True,
    }
    assert candidate["live_phase"]["direct_stat_reads"] == 2
    assert candidate["zombie_phase"]["direct_stat_reads"] == 2
    assert candidate["zombie_phase"]["wait_operation"] == "waitid_P_PIDFD_WEXITED_WNOWAIT"
    assert candidate["reap_phase"]["post_reap_group_scans"] == 2


def test_evidence_contract_requires_one_status_denial_and_zero_stat_denials() -> None:
    policy, _schema = policy_and_schema()
    contract = policy["evidence_classification"]
    assert contract == GATE.EVIDENCE_CLASSIFICATION_CONTRACT
    assert contract == SUPERVISOR.EVIDENCE_CLASSIFICATION_CONTRACT
    assert contract["expected_counts"] == {
        "matching_total": 1,
        "stored_count": 1,
        "dropped_count": 0,
        "expected_status_total": 1,
        "stat_denial_total": 0,
        "unexpected_total": 0,
        "storage_overflow": False,
    }
    assert contract["zero_logged_denials_required"] is False
    assert contract["zero_denied_operations_claimed"] is False
    assert contract["expected_status_denial"]["name_must_match_success_frame_child_pid"] is True


def test_failure_diagnostics_and_two_anchor_journal_contracts_are_frozen() -> None:
    policy, schema = policy_and_schema()
    diagnostics = GATE.failure_diagnostics_contract()
    assert diagnostics == SUPERVISOR.failure_diagnostics_contract()
    assert policy["failure_diagnostics"] == diagnostics
    assert schema["properties"]["failure_diagnostics"]["const"] == diagnostics
    journal = policy["journal_contract"]
    assert journal == GATE.JOURNAL_CONTRACT == SUPERVISOR.JOURNAL_CONTRACT
    assert schema["properties"]["journal_contract"]["const"] == journal
    assert journal["admission_anchor_before_profile_load"] is True
    assert journal["pre_run_sync_after_profile_load"] is True
    assert journal["execution_anchor_after_profile_load_and_pre_run_sync"] is True
    assert journal["execution_window_excludes_profile_load_events"] is True
    assert journal["final_sync_before_execution_query"] is True
    assert journal["audit_suppression_or_loss_marker_fail_closed"] is True
    assert journal["audit_loss_marker_codes"] == [
        "KAUDITD_CALLBACKS_SUPPRESSED",
        "AUDIT_BACKLOG_LIMIT_EXCEEDED",
        "AUDIT_RECORDS_LOST",
    ]


def test_generated_helper_loader_and_bwrap_argv_are_hash_seed_deterministic() -> None:
    gate_source = GATE_PATH.read_text(encoding="utf-8")
    assert (
        "POST_EACCES_FAILURE_CODES = "
        "frozenset({tuple(sorted(POST_EACCES_FAILURE_CODES))!r})"
    ) in gate_source
    assert (
        "POST_EACCES_ONLY_FAILURE_CODES = "
        "frozenset({tuple(sorted(POST_EACCES_ONLY_FAILURE_CODES))!r})"
    ) in gate_source
    expected_post = (
        "POST_EACCES_FAILURE_CODES = "
        f"frozenset({tuple(sorted(GATE.POST_EACCES_FAILURE_CODES))!r})"
    )
    expected_only = (
        "POST_EACCES_ONLY_FAILURE_CODES = "
        f"frozenset({tuple(sorted(GATE.POST_EACCES_ONLY_FAILURE_CODES))!r})"
    )
    assert expected_post in GATE.HELPER_SOURCE
    assert expected_only in GATE.HELPER_SOURCE


def test_no_legacy_transport_or_mount_discovery_semantics_remain() -> None:
    legacy_tokens = (
        "u3_" + "stdio_apparmor_transport_probe",
        "STDIO_" + "APPARMOR",
        "STATIC_READY_" + "V11",
        "PASS_" + "V11",
        "PREDECESSOR_" + "V8",
        "PREDECESSOR_" + "V9",
        "PREDECESSOR_" + "V10",
        "expected_" + "mount_total",
        "expected_" + "mount_denials",
        "_is_exact_" + "mount_discovery",
        "/usr/bin/" + "sleep",
    )
    for path in (GATE_PATH, SUPERVISOR_PATH, POLICY_PATH, SCHEMA_PATH, PROFILE_PATH):
        text = path.read_text(encoding="utf-8")
        assert not any(token in text for token in legacy_tokens), (path, [t for t in legacy_tokens if t in text])


def test_static_review_is_subprocess_free(monkeypatch) -> None:
    monkeypatch.setattr(subprocess, "Popen", lambda *_a, **_k: pytest.fail("static review attempted Popen"))
    gate_review = GATE.verify_static()
    supervisor_review = SUPERVISOR.repository_static_review()
    assert gate_review["execution_performed"] is False
    assert supervisor_review["execution_performed"] is False
    assert gate_review["freeze_ready"] is True
    assert supervisor_review["freeze_ready"] is True


def test_gate_snapshot_import_reads_snapshot_bytes_but_keeps_canonical_policy_paths(
    tmp_path: Path, monkeypatch
) -> None:
    snapshot = tmp_path / "snapshot"
    snapshot.mkdir()
    sources = {
        "gate": GATE_PATH,
        "supervisor": SUPERVISOR_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
        "policy": POLICY_PATH,
    }
    snapshot_paths = {}
    for name, source in sources.items():
        destination = snapshot / source.name
        destination.write_bytes(source.read_bytes())
        snapshot_paths[name] = destination

    monkeypatch.setenv(GATE.SNAPSHOT_ENV, str(snapshot))
    snapshot_gate = load_module(
        snapshot_paths["gate"], "u3_proc_stat_boundary_gate_v4_snapshot_test"
    )
    assert snapshot_gate.artifact_paths() == {
        name: snapshot_paths[name]
        for name in ("gate", "supervisor", "profile", "schema")
    }

    canonical_paths = {
        "gate": PACKAGE / "scripts" / GATE_PATH.name,
        "supervisor": PACKAGE / "scripts" / SUPERVISOR_PATH.name,
        "profile": PACKAGE / "config/apparmor_drafts" / PROFILE_PATH.name,
        "schema": PACKAGE / "schema" / SCHEMA_PATH.name,
    }
    assert snapshot_gate.artifact_policy_paths() == canonical_paths
    policy = json.loads(snapshot_paths["policy"].read_text(encoding="utf-8"))
    assert {
        name: policy["trusted_artifacts"][name]["path"]
        for name in canonical_paths
    } == {name: str(path) for name, path in canonical_paths.items()}

    review = snapshot_gate.verify_static()
    assert {
        name: record["path"] for name, record in review["artifacts"].items()
    } == {
        name: str(snapshot_paths[name])
        for name in ("gate", "supervisor", "profile", "schema")
    }


def test_supervisor_import_from_arbitrary_run_like_path_keeps_canonical_projection(
    tmp_path: Path,
) -> None:
    run_like = tmp_path / "run-like" / "detached-v4-snapshot"
    run_like.mkdir(parents=True)
    detached_path = run_like / SUPERVISOR_PATH.name
    detached_path.write_bytes(SUPERVISOR_PATH.read_bytes())
    detached = load_module(
        detached_path, "u3_proc_stat_boundary_supervisor_v4_run_like_test"
    )

    expected_repository = {
        "gate": PACKAGE / "scripts" / GATE_PATH.name,
        "supervisor": PACKAGE / "scripts" / SUPERVISOR_PATH.name,
        "profile": PACKAGE / "config/apparmor_drafts" / PROFILE_PATH.name,
        "schema": PACKAGE / "schema" / SCHEMA_PATH.name,
        "policy": PACKAGE / "config/target_hosts" / POLICY_PATH.name,
    }
    expected_snapshot = {
        name: detached.SNAPSHOT_ROOT / path.name
        for name, path in expected_repository.items()
    }
    assert detached.SCRIPT_PATH == detached_path
    assert detached.REPOSITORY_PATHS == expected_repository
    assert detached.SNAPSHOT_PATHS == expected_snapshot
    review = detached.verify_repository_path_contract()
    assert review["repository_paths"] == {
        name: str(path) for name, path in expected_repository.items()
    }
    assert review["snapshot_paths"] == {
        name: str(path) for name, path in expected_snapshot.items()
    }
    assert review["v2_package_dir_or_script_path_derivation"] is False


def supervisor_path_mutation_cases():
    fixed_repository = """REPOSITORY_PATHS = {
    "gate": WORKSPACE_ROOT / "scripts" / GATE_NAME,
    "supervisor": WORKSPACE_ROOT / "scripts" / SUPERVISOR_NAME,
    "profile": WORKSPACE_ROOT / "config/apparmor_drafts" / PROFILE_NAME,
    "schema": WORKSPACE_ROOT / "schema" / SCHEMA_NAME,
    "policy": WORKSPACE_ROOT / "config/target_hosts" / POLICY_NAME,
}"""
    v2_repository = """REPOSITORY_PATHS = {
    "gate": PACKAGE_DIR / "scripts" / GATE_NAME,
    "supervisor": SCRIPT_PATH,
    "profile": PACKAGE_DIR / "config/apparmor_drafts" / PROFILE_NAME,
    "schema": PACKAGE_DIR / "schema" / SCHEMA_NAME,
    "policy": PACKAGE_DIR / "config/target_hosts" / POLICY_NAME,
}"""
    return [
        (
            "script_path_supervisor_name",
            lambda source: source.replace(
                f'SUPERVISOR_NAME = "{SUPERVISOR_PATH.name}"',
                "SUPERVISOR_NAME = SCRIPT_PATH.name",
                1,
            ),
        ),
        (
            "package_dir_repository_paths",
            lambda source: source.replace(
                "SCRIPT_PATH = Path(__file__).resolve()\n",
                "SCRIPT_PATH = Path(__file__).resolve()\n"
                "PACKAGE_DIR = SCRIPT_PATH.parent.parent\n",
                1,
            ).replace(fixed_repository, v2_repository, 1),
        ),
    ]


@pytest.mark.parametrize(
    "_case,mutate",
    supervisor_path_mutation_cases(),
    ids=[case for case, _mutate in supervisor_path_mutation_cases()],
)
def test_v2_import_location_path_bug_is_rejected_by_both_reviewers(
    tmp_path: Path, _case, mutate
) -> None:
    original = SUPERVISOR_PATH.read_text(encoding="utf-8")
    changed = mutate(original)
    assert changed != original
    run_like = tmp_path / "run-like" / _case
    run_like.mkdir(parents=True)
    changed_path = run_like / SUPERVISOR_PATH.name
    changed_path.write_text(changed, encoding="utf-8")

    with pytest.raises(GATE.GateError):
        GATE.verify_supervisor_repository_path_contract(changed_path)

    changed_supervisor = load_module(
        changed_path, f"u3_proc_stat_boundary_supervisor_v4_{_case}_test"
    )
    with pytest.raises(changed_supervisor.SupervisorError):
        changed_supervisor.verify_repository_path_contract()



def test_reviewed_artifacts_are_placeholders_only_before_freeze_or_exact_after_freeze() -> None:
    policy, _schema = policy_and_schema()
    artifacts = {
        "gate": GATE_PATH,
        "supervisor": SUPERVISOR_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
    }
    frozen = policy["trusted_artifacts"]["all_frozen"]
    for name, path in artifacts.items():
        record = policy["trusted_artifacts"][name]
        raw = path.read_bytes()
        if frozen:
            assert record["size_bytes"] == len(raw)
            assert record["sha256"] == hashlib.sha256(raw).hexdigest()
        else:
            assert record["size_bytes"] == 0
            assert record["sha256"] == "0" * 64
    reviewed = policy["reviewed_bytes"]
    if reviewed["all_frozen"]:
        assert reviewed["helper"] == {"size_bytes": GATE.HELPER_SIZE_BYTES, "sha256": GATE.HELPER_SHA256}
        assert reviewed["loader"] == {"size_bytes": GATE.LOADER_SIZE_BYTES, "sha256": GATE.LOADER_SHA256}
    else:
        assert reviewed["helper"] == {"size_bytes": 0, "sha256": "0" * 64}
        assert reviewed["loader"] == {"size_bytes": 0, "sha256": "0" * 64}
        assert reviewed["payload_frame"] == {
            "size_bytes": 0,
            "sha256": "0" * 64,
            "maximum_size_bytes": GATE.MAX_PAYLOAD_FRAME_BYTES,
        }
    bootstrap = policy["snapshot_bootstrap"]
    if frozen:
        assert bootstrap["all_frozen"] is True
        assert bootstrap["source_size_bytes"] == len(SUPERVISOR.SNAPSHOT_BOOTSTRAP_BYTES)
        assert bootstrap["source_sha256"] == SUPERVISOR.SNAPSHOT_BOOTSTRAP_SHA256
        assert bootstrap["loader_size_bytes"] == len(SUPERVISOR.SNAPSHOT_BOOTSTRAP_LOADER_BYTES)
        assert bootstrap["loader_sha256"] == SUPERVISOR.SNAPSHOT_BOOTSTRAP_LOADER_SHA256
        assert bootstrap["reviewed_minimal_bootstrap_executed_as_root"] is True
    else:
        assert bootstrap["all_frozen"] is False
        assert bootstrap["source_size_bytes"] == 0
        assert bootstrap["source_sha256"] == "0" * 64
        assert bootstrap["loader_size_bytes"] == 0
        assert bootstrap["loader_sha256"] == "0" * 64
        assert bootstrap["reviewed_minimal_bootstrap_executed_as_root"] is False


def test_fresh_snapshot_and_all_receipts_are_absent() -> None:
    paths = [
        GATE.SNAPSHOT_ROOT,
        SUPERVISOR.START_RECEIPT,
        SUPERVISOR.PREFLIGHT_FAILURE_RECEIPT,
        SUPERVISOR.EXECUTION_RECEIPT,
        SUPERVISOR.LIFECYCLE_RECEIPT,
        SUPERVISOR.LIFECYCLE_FAILURE_RECEIPT,
        SUPERVISOR.RECOVERY_RECEIPT,
    ]
    assert len(paths) == len(set(paths))
    assert all(GATE.PROBE_ID in str(path) for path in paths)
    assert all(not path.exists() for path in paths)


def test_payload_frame_is_in_memory_helper_only_and_strict() -> None:
    frame = GATE.build_payload_frame()
    observed = GATE.parse_payload_frame(frame)
    assert observed["helper"] == {
        "size_bytes": GATE.HELPER_SIZE_BYTES,
        "sha256": GATE.HELPER_SHA256,
    }
    assert observed["size_bytes"] == len(frame)
    assert observed["sha256"] == hashlib.sha256(frame).hexdigest()
    for changed in (frame[:-1], frame + b"x", bytes([frame[0] ^ 1]) + frame[1:]):
        with pytest.raises(GATE.GateError):
            GATE.parse_payload_frame(changed)


def test_helper_child_sets_and_verifies_dumpable_zero_before_ready() -> None:
    child = helper_function_source("child_main")
    ordered = (
        "os.setsid()",
        "libc.prctl(4, 0, 0, 0, 0)",
        "libc.prctl(3, 0, 0, 0, 0)",
        '"dumpable": 0',
        "write_all(ready_fd, raw)",
    )
    positions = [child.index(token) for token in ordered]
    assert positions == sorted(positions)
    assert "os.exec" not in child
    assert "subprocess" not in GATE.HELPER_SOURCE


def test_helper_state_machine_orders_live_status_wnowait_zombie_reap_and_empty_scans() -> None:
    run = helper_function_source("run_proc_stat_probe")
    ordered = (
        'live_stat_pair = require_pair(pid, identity, "live")',
        "status_denial = require_status_eacces_once(pid)",
        'write_all(release_write, b"R")',
        "os.WEXITED | os.WNOWAIT",
        'zombie_stat_pair = require_pair(pid, identity, "zombie")',
        "reaped_result = os.waitid(os.P_PIDFD, pidfd, os.WEXITED)",
        "final_scans = [",
    )
    positions = [run.index(token) for token in ordered]
    assert positions == sorted(positions)
    assert run.count("require_status_eacces_once(pid)") == 1
    assert run.count("scan_group_members_once(identity[1])") == 2
    assert run.count("os.fork()") == 1
    assert run.index("os.waitpid") > run.index('zombie_stat_pair = require_pair(pid, identity, "zombie")')


def test_helper_pair_and_group_scans_are_literal_double_reads() -> None:
    pair = helper_function_source("require_pair")
    group = helper_function_source("group_members")
    status = helper_function_source("require_status_eacces_once")
    assert pair.count("read_process_member(pid)") == 2
    assert group.count("scan_group_members_once(pgid)") == 2
    assert "if first != second:" in group
    assert 'raise ProbeError("process_group_double_scan_unstable")' in group
    assert "return first" in group
    assert status.count("os.open(") == 1
    assert "except OSError as exc" in status
    assert "exc.errno != errno.EACCES" in status
    assert 'STATUS_OBSERVATION = "STATUS_OTHER_ERRNO"' in status
    assert 'STATUS_OBSERVATION = "STATUS_READABLE"' in status
    assert "status_unexpectedly_readable" in status
    assert '"attempt_count": 1' in status


def test_success_frame_parsers_accept_exact_proc_stat_evidence() -> None:
    frame = success_frame()
    assert GATE.parse_success_frame(frame) == valid_success_payload()
    assert SUPERVISOR.parse_success_frame(frame) == valid_success_payload()


def test_helper_probe_error_enum_is_closed_and_success_systemexit_is_not_caught() -> None:
    tree = ast.parse(GATE.HELPER_SOURCE)
    observed = {
        node.exc.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Raise)
        and isinstance(node.exc, ast.Call)
        and isinstance(node.exc.func, ast.Name)
        and node.exc.func.id == "ProbeError"
        and len(node.exc.args) == 1
        and isinstance(node.exc.args[0], ast.Constant)
        and isinstance(node.exc.args[0].value, str)
    }
    assert observed == set(GATE.PROBE_ERROR_CODES)
    tail = GATE.HELPER_SOURCE[GATE.HELPER_SOURCE.rindex("try:\n    raise SystemExit(main())"):]
    assert "except Exception:" in tail
    assert "except BaseException:" not in tail


@pytest.mark.parametrize(
    "failure_code,boundary_status",
    [
        ("BOOTSTRAP_LABEL", "STATUS_NOT_REACHED"),
        ("STATUS_UNEXPECTEDLY_READABLE", "STATUS_READABLE"),
        ("STATUS_OTHER_ERRNO", "STATUS_OTHER_ERRNO"),
        ("CHILD_WAITID_WNOWAIT", "STATUS_EACCES_THEN_LATER_FAILURE"),
        ("UNEXPECTED_INTERNAL_ERROR", "STATUS_NOT_REACHED"),
        ("UNEXPECTED_INTERNAL_ERROR", "STATUS_EACCES_THEN_LATER_FAILURE"),
    ],
)
def test_failure_frame_is_exact_length_enum_only_and_both_parsers_agree(
    failure_code: str, boundary_status: str,
) -> None:
    frame = failure_frame(failure_code, boundary_status)
    assert len(frame) == GATE.EXACT_FAILURE_FRAME_BYTES == SUPERVISOR.EXACT_FAILURE_FRAME_BYTES
    expected = {"boundary_status": boundary_status, "failure_code": failure_code}
    assert GATE.parse_failure_frame(frame) == expected
    assert SUPERVISOR.parse_failure_frame(frame) == expected
    assert all(token.encode("ascii") not in frame for token in (
        "/proc/", "pid=", "errno=", "--uid", "HOME=", "nonce",
    ))


@pytest.mark.parametrize(
    "failure_code,boundary_status",
    [
        ("STATUS_UNEXPECTEDLY_READABLE", "STATUS_NOT_REACHED"),
        ("STATUS_OTHER_ERRNO", "STATUS_READABLE"),
        ("BOOTSTRAP_LABEL", "STATUS_EACCES_THEN_LATER_FAILURE"),
        ("CHILD_WAITID_WNOWAIT", "STATUS_NOT_REACHED"),
    ],
)
def test_failure_frame_rejects_semantically_incompatible_enum_pairs(
    failure_code: str, boundary_status: str,
) -> None:
    frame = failure_frame(failure_code, boundary_status)
    with pytest.raises(GATE.GateError):
        GATE.parse_failure_frame(frame)
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.parse_failure_frame(frame)


def test_failure_frame_rejects_truncation_tail_magic_length_padding_and_digest_mutations() -> None:
    original = failure_frame()
    prefix_size = len(GATE.FAILURE_MAGIC)
    payload_offset = prefix_size + 2
    payload_size = struct.unpack(">H", original[prefix_size:payload_offset])[0]
    mutations = []
    mutations.append(original[:-1])
    mutations.append(original + b"x")
    value = bytearray(original); value[0] ^= 1; mutations.append(bytes(value))
    value = bytearray(original); value[prefix_size:payload_offset] = struct.pack(">H", payload_size - 1); mutations.append(bytes(value))
    value = bytearray(original); value[payload_offset + payload_size] = 1; mutations.append(bytes(value))
    value = bytearray(original); value[-1] ^= 1; mutations.append(bytes(value))
    for frame in mutations:
        with pytest.raises(GATE.GateError):
            GATE.parse_failure_frame(frame)
        with pytest.raises(SUPERVISOR.SupervisorError):
            SUPERVISOR.parse_failure_frame(frame)


def success_mutation_cases():
    return [
        ("extra_top_key", lambda value: value.__setitem__("extra", False)),
        ("host_write", lambda value: value["host_writable_mounts"].append("/host")),
        ("payload_not_consumed", lambda value: value.__setitem__("host_payload_consumed_and_fd0_replaced_with_eof_pipe", False)),
        ("fd_bool", lambda value: value["fds_after_payload"].__setitem__(0, False)),
        ("child_dumpable", lambda value: value["proc_stat_probe"]["child_identity"].__setitem__("dumpable", 1)),
        ("child_group", lambda value: value["proc_stat_probe"]["child_identity"].__setitem__("pgrp", 4)),
        ("pidfd_false", lambda value: value["proc_stat_probe"]["frozen_identity"].__setitem__("pidfd_opened", False)),
        ("starttime_bool", lambda value: value["proc_stat_probe"]["frozen_identity"].__setitem__("starttime", True)),
        ("owner_zero", lambda value: value["proc_stat_probe"]["inner_proc_owner_projection"].__setitem__("uid", 0)),
        ("live_once", lambda value: value["proc_stat_probe"].__setitem__("live_stat_pair", value["proc_stat_probe"]["live_stat_pair"][:1])),
        ("live_zombie", lambda value: value["proc_stat_probe"]["live_stat_pair"][0].__setitem__("state", "Z")),
        ("live_unstable", lambda value: value["proc_stat_probe"]["live_stat_pair"][1].__setitem__("state", "R")),
        ("group_extra", lambda value: value["proc_stat_probe"]["live_group_scan"].append(copy.deepcopy(value["proc_stat_probe"]["live_group_scan"][0]))),
        ("status_attempt_zero", lambda value: value["proc_stat_probe"]["status_denial"].__setitem__("attempt_count", 0)),
        ("status_attempt_bool", lambda value: value["proc_stat_probe"]["status_denial"].__setitem__("attempt_count", True)),
        ("status_errno_eperm", lambda value: value["proc_stat_probe"]["status_denial"].__setitem__("errno", errno.EPERM)),
        ("status_wrong_pid", lambda value: value["proc_stat_probe"]["status_denial"].__setitem__("path", "/proc/4/status")),
        ("wnowait_wrong_pid", lambda value: value["proc_stat_probe"]["waitid_wnowait"].__setitem__("pid", 4)),
        ("wnowait_wrong_code", lambda value: value["proc_stat_probe"]["waitid_wnowait"].__setitem__("code", 0)),
        ("zombie_once", lambda value: value["proc_stat_probe"].__setitem__("zombie_stat_pair", value["proc_stat_probe"]["zombie_stat_pair"][:1])),
        ("zombie_live", lambda value: value["proc_stat_probe"]["zombie_stat_pair"][0].__setitem__("state", "S")),
        ("zombie_identity", lambda value: value["proc_stat_probe"]["zombie_stat_pair"][0].__setitem__("starttime", 999)),
        ("reap_nonzero", lambda value: value["proc_stat_probe"]["final_reap"].__setitem__("status", 1)),
        ("one_empty_scan", lambda value: value["proc_stat_probe"].__setitem__("post_reap_group_scans", [[]])),
        ("residual_group", lambda value: value["proc_stat_probe"]["post_reap_group_scans"][1].append({"pid": 3})),
    ]


@pytest.mark.parametrize("_case,mutate", success_mutation_cases(), ids=[case for case, _ in success_mutation_cases()])
def test_success_frame_parsers_reject_every_boundary_mutation(_case, mutate) -> None:
    value = valid_success_payload()
    mutate(value)
    frame = success_frame(value)
    with pytest.raises(GATE.GateError):
        GATE.parse_success_frame(frame)
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.parse_success_frame(frame)


def test_bwrap_argv_is_bounded_isolated_and_has_one_fresh_proc_mount() -> None:
    argv = GATE.bwrap_argv()
    GATE.verify_argv_contract(argv)
    for token in ("--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts", "--disable-userns", "--assert-userns-disabled", "--cap-drop"):
        assert argv.count(token) == 1
    assert argv[argv.index("--cap-drop") : argv.index("--cap-drop") + 2] == ["--cap-drop", "ALL"]
    assert argv.count("--proc") == 1
    assert argv[argv.index("--proc") : argv.index("--proc") + 2] == ["--proc", "/proc"]
    assert argv.count("--ro-bind") == 1
    assert not any(token in argv for token in ("--bind", "--bind-fd", "--file", "--dev", "--dev-bind", "--share-net"))
    joined = "\0".join(argv)
    assert all(token not in joined for token in ("DualSPHysics", "GenCase", "/dev/nvidia", "/opt/ros"))


def effective_profile_lines(text: str) -> tuple[str, ...]:
    return tuple(
        line.strip()
        for line in (raw.split("#", 1)[0] for raw in text.splitlines())
        if line.strip()
    )


def normalize_profile_lines(
    text: str, bootstrap_label: str, runtime_label: str,
) -> tuple[str, ...]:
    return tuple(
        line.replace(bootstrap_label, "r8-liquid-u3-NORMALIZED-bootstrap").replace(
            runtime_label, "r8-liquid-u3-NORMALIZED-runtime"
        )
        for line in effective_profile_lines(text)
    )


def test_candidate_rule_is_the_only_full_authority_addition_from_frozen_v6() -> None:
    candidate = "/proc/[0-9]*/stat r,"
    v6_raw = V6_PROFILE_PATH.read_bytes()
    assert len(v6_raw) == 3540
    assert hashlib.sha256(v6_raw).hexdigest() == "a6841cd0fad78fa31e1a2a7b04eab0eb9158856884162112216ddcb8a01bfe0a"
    v6 = normalize_profile_lines(
        v6_raw.decode("utf-8"),
        GATE.V6_PROFILE_BASELINE["bootstrap_profile"],
        GATE.V6_PROFILE_BASELINE["runtime_profile"],
    )
    v4 = normalize_profile_lines(
        PROFILE_PATH.read_text(encoding="utf-8"),
        GATE.BOOTSTRAP_PROFILE,
        GATE.RUNTIME_PROFILE,
    )
    assert len(v6) == 64
    assert len(v4) == 60
    assert hashlib.sha256(("\n".join(v4) + "\n").encode()).hexdigest() == (
        "93bd45c83d3a8cece8f9acd327f478322e0d85dc35a01ba725da86635e3097b6"
    )
    assert Counter(v4) - Counter(v6) == Counter({candidate: 1})
    assert Counter(v6) - Counter(v4) == Counter(GATE.EXPECTED_FULL_AUTHORITY_REMOVALS)
    expected = list(v6)
    for line in GATE.EXPECTED_FULL_AUTHORITY_REMOVALS:
        expected.remove(line)
    insert_after = expected.index("owner /proc/*/setgroups w,") + 1
    expected.insert(insert_after, candidate)
    assert v4 == tuple(expected)
    gate_review = GATE.verify_profile()
    supervisor_review = SUPERVISOR.verify_profile_bytes()
    assert gate_review["candidate_rule"] == candidate
    assert gate_review["full_effective_authority_delta"] == (
        supervisor_review["full_effective_authority_delta"]
    )
    assert gate_review["full_effective_authority_delta"]["additions"] == [candidate]
    assert gate_review["full_effective_authority_delta"]["removals"] == list(
        GATE.EXPECTED_FULL_AUTHORITY_REMOVALS
    )


def profile_mutation_cases():
    candidate = "/proc/[0-9]*/stat r,"

    def replace_rule(replacement: str):
        return lambda text: text.replace(f"  {candidate}\n", f"  {replacement}\n", 1)

    cases = [
        ("missing", lambda text: text.replace(f"  {candidate}\n", "", 1)),
        ("duplicate", lambda text: text.replace(f"  {candidate}\n", f"  {candidate}\n  {candidate}\n", 1)),
        ("runtime", lambda text: text.replace(f"profile {GATE.RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted) {{", f"profile {GATE.RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted) {{\n  {candidate}", 1)),
        ("owner", replace_rule("owner /proc/[0-9]*/stat r,")),
        ("deny", replace_rule("deny /proc/[0-9]*/stat r,")),
        ("audit", replace_rule("audit /proc/[0-9]*/stat r,")),
        ("allow", replace_rule("allow /proc/[0-9]*/stat r,")),
        ("file", replace_rule("file /proc/[0-9]*/stat r,")),
        ("write", replace_rule("/proc/[0-9]*/stat rw,")),
        ("execute", replace_rule("/proc/[0-9]*/stat rix,")),
        ("all_pid_children", replace_rule("/proc/[0-9]*/** r,")),
        ("nonnumeric_pid", replace_rule("/proc/*/stat r,")),
        ("recursive_pid", replace_rule("/proc/**/stat r,")),
        ("stat_prefix", replace_rule("/proc/[0-9]*/stat* r,")),
        ("wildcard_basename", replace_rule("/proc/[0-9]*/st?? r,")),
        ("status_brace", replace_rule("/proc/[0-9]*/{stat,status} r,")),
        ("self_brace", replace_rule("/proc/{[0-9]*,self}/stat r,")),
        ("root_brace", replace_rule("/{,newroot/}proc/[0-9]*/stat r,")),
        ("global_stat", replace_rule("/**/stat r,")),
        ("variable", replace_rule("@{PROC}/[0-9]*/stat r,")),
        ("extra_status", lambda text: text.replace(f"  {candidate}\n", f"  {candidate}\n  /proc/[0-9]*/status r,\n", 1)),
        ("ld_cache_deny_to_allow", lambda text: text.replace("  deny /etc/ld.so.cache r,\n", "  /etc/ld.so.cache r,\n", 1)),
        ("ld_so_conf_allow", lambda text: text.replace("  deny /etc/ld.so.cache r,\n", "  deny /etc/ld.so.cache r,\n  /etc/ld.so.conf r,\n", 1)),
        ("ld_so_conf_d_allow", lambda text: text.replace("  deny /etc/ld.so.cache r,\n", "  deny /etc/ld.so.cache r,\n  /etc/ld.so.conf.d/** r,\n", 1)),
        ("ld_cache_deny_removed", lambda text: text.replace("  deny /etc/ld.so.cache r,\n", "", 1)),
        ("ld_cache_deny_and_allow", lambda text: text.replace("  deny /etc/ld.so.cache r,\n", "  deny /etc/ld.so.cache r,\n  /etc/ld.so.cache r,\n", 1)),
        *(
            (
                "restored_" + str(index),
                lambda text, rule=rule: text.replace("  /work/ rw,\n", f"  /work/ rw,\n  {rule}\n", 1),
            )
            for index, rule in enumerate(GATE.EXPECTED_FULL_AUTHORITY_REMOVALS)
        ),
        ("broad_owner_removed", lambda text: text.replace("  owner /proc/** r,\n", "  /proc/** r,\n", 1)),
        ("include_hash", lambda text: "#include <abstractions/base>\n" + text),
        ("include_spaced", lambda text: "# include <abstractions/base>\n" + text),
        ("include_tab", lambda text: "#\tinclude <abstractions/base>\n" + text),
        ("include_bare", lambda text: "include <abstractions/base>\n" + text),
        ("abi", lambda text: "abi <abi/4.0>,\n" + text),
    ]
    return cases


@pytest.mark.parametrize("_case,mutate", profile_mutation_cases(), ids=[case for case, _ in profile_mutation_cases()])
def test_profile_mutations_fail_both_independent_reviewers(tmp_path, monkeypatch, _case, mutate) -> None:
    changed = mutate(PROFILE_PATH.read_text(encoding="utf-8"))
    assert changed != PROFILE_PATH.read_text(encoding="utf-8")
    path = tmp_path / "mutated.profile"
    path.write_text(changed, encoding="utf-8")
    monkeypatch.setattr(GATE, "PROFILE_PATH", path)
    with pytest.raises(GATE.GateError):
        GATE.verify_profile()
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.verify_profile_bytes(path)


def test_frozen_v6_failure_provenance_is_verified_without_snapshot_or_output_reuse() -> None:
    policy, _schema = policy_and_schema()
    assert policy["provenance"]["cold_a_v6_failure"] == GATE.FROZEN_V6_FAILURE
    assert GATE.FROZEN_V6_FAILURE == SUPERVISOR.FROZEN_V6_FAILURE
    for verifier in (GATE.verify_v6_failure_provenance, SUPERVISOR.verify_v6_failure_provenance):
        observed = verifier()
        assert observed["identity_consumed"] is True
        assert observed["retry_forbidden"] is True
        assert observed["snapshot_read"] is False
        assert observed["output_reused"] is False
        assert observed["stat_denials"]["name"] == "/proc/3/stat"
        assert observed["stat_denials"]["fsuid"] == "1000"
        assert observed["stat_denials"]["ouid"] == "0"
        assert len(observed["stat_denials"]["line_sha256"]) == 2


def test_frozen_v1_static_no_go_provenance_is_preserved_and_never_reused() -> None:
    policy, _schema = policy_and_schema()
    frozen = GATE.FROZEN_V1_NO_GO
    assert frozen == SUPERVISOR.FROZEN_V1_NO_GO
    assert policy["provenance"]["proc_stat_v1_static_no_go"] == frozen
    assert frozen["decision"] == "STATIC_NO_GO_FULL_PROFILE_AUTHORITY_DELTA_VIOLATION"
    assert [item["v1_rule"] for item in frozen["blocker"]["exact_differences"]] == [
        "/etc/ld.so.cache r,",
        "/etc/ld.so.conf r,",
        "/etc/ld.so.conf.d/** r,",
    ]
    assert frozen["observed_facts"] == {
        "snapshot_created": True,
        "profile_loaded": False,
        "probe_executed": False,
        "receipt_created": False,
    }
    assert frozen["source_use"] == (
        "static_no_go_provenance_only_no_runtime_baseline_or_snapshot_artifact_reuse"
    )
    assert len(frozen["snapshot"]["files"]) == 5
    assert len(frozen["receipt_paths"]) == 6
    for verifier in (GATE.verify_v1_no_go_provenance, SUPERVISOR.verify_v1_no_go_provenance):
        observed = verifier()
        assert observed["decision"] == frozen["decision"]
        assert observed["receipts_absent"] is True
        assert observed["runtime_baseline_used"] is False
        assert observed["snapshot_artifact_reused"] is False
        assert observed["identity_consumed"] is True
        assert observed["retry_forbidden"] is True
        assert observed["snapshot"]["preserved_unchanged"] is True


def test_frozen_v2_prestart_no_go_provenance_is_exact_and_never_reused() -> None:
    policy, _schema = policy_and_schema()
    frozen = GATE.FROZEN_V2_PRESTART_NO_GO
    assert frozen == SUPERVISOR.FROZEN_V2_PRESTART_NO_GO
    assert policy["provenance"]["proc_stat_v2_prestart_no_go"] == frozen
    assert frozen["decision"] == "PRE_START_NO_GO_IDENTITY_CONSUMED_RETRY_FORBIDDEN"
    assert frozen["source_use"] == (
        "pre_start_no_go_snapshot_and_receipt_provenance_only_"
        "no_runtime_baseline_or_snapshot_artifact_reuse"
    )
    assert frozen["observed_facts"] == {
        "snapshot_created": True,
        "parser_invoked": False,
        "profile_loaded": False,
        "probe_executed": False,
        "preflight_receipt_created": True,
        "sudo_timestamp_cleanup_proven": True,
    }
    assert frozen["runtime_baseline_authorized"] is False
    assert frozen["snapshot_artifact_reuse_authorized"] is False
    assert frozen["solver_or_gencase_authorized"] is False
    assert frozen["cold_a_successor_execution_authorized"] is False
    assert frozen["u4_authorized"] is False
    assert frozen["production_authorized"] is False
    assert len(frozen["snapshot"]["files"]) == 5
    assert len(frozen["other_receipt_paths"]) == 5
    receipt = frozen["preflight_receipt"]
    assert receipt["status"] == (
        "PRE_START_FAILURE_IDENTITY_CONSUMED_NO_RETRY_SUDO_TIMESTAMP_CLEANED_"
        "NO_PROFILE_LOAD_OR_PROBE_EXECUTED"
    )
    assert receipt["primary_error"] == (
        "SupervisorError: snapshot artifact policy path differs: gate"
    )
    for key in (
        "parser_invoked_by_this_run",
        "profile_load_attempted_by_this_run",
        "probe_executed_by_this_run",
        "start_receipt_created_by_this_run",
    ):
        assert receipt[key] is False
    assert receipt["sudo_timestamp_cleanup_proven"] is True

    for verifier in (
        GATE.verify_v2_prestart_no_go_provenance,
        SUPERVISOR.verify_v2_prestart_no_go_provenance,
    ):
        observed = verifier()
        assert observed["decision"] == frozen["decision"]
        assert observed["identity_consumed"] is True
        assert observed["retry_forbidden"] is True
        assert observed["runtime_baseline_used"] is False
        assert observed["snapshot_artifact_reused"] is False
        assert observed["production_authorized"] is False
        assert observed["other_receipts_absent"] is True
        assert observed["snapshot"]["preserved_unchanged"] is True
        assert observed["preflight_receipt"]["status"] == receipt["status"]


def test_frozen_v3_runtime_no_go_provenance_is_exact_and_never_reused() -> None:
    policy, schema = policy_and_schema()
    frozen = GATE.FROZEN_V3_NO_GO
    assert frozen == SUPERVISOR.FROZEN_V3_NO_GO
    assert policy["provenance"]["proc_stat_v3_runtime_no_go"] == frozen
    assert schema["properties"]["provenance"]["const"] == policy["provenance"]
    assert frozen["decision"] == (
        "RUNTIME_UNCLASSIFIED_NO_GO_LIFECYCLE_CLEANUP_PASS_IDENTITY_CONSUMED"
    )
    assert frozen["source_use"] == (
        "runtime_no_go_provenance_only_no_snapshot_output_or_runtime_baseline_reuse"
    )
    assert len(frozen["snapshot"]["files"]) == 5
    assert set(frozen["receipts"]) == {"start", "execution", "lifecycle"}
    assert set(frozen["other_receipt_paths"]) == {
        "preflight_failure", "lifecycle_failure", "recovery",
    }
    assert frozen["evidence_interpretation"] == {
        "root_cause_claimed": False,
        "zero_logged_denials_is_lossless_authority_proof": False,
        "lifecycle_pass_is_probe_pass": False,
        "next_allowed_stage":
            "REVIEW_THIS_SINGLE_ATTEMPT_RECEIPT_AND_CREATE_FRESH_SUCCESSOR_ONLY",
    }
    assert frozen["runtime_baseline_authorized"] is False
    assert frozen["snapshot_artifact_reuse_authorized"] is False
    assert frozen["output_reuse_authorized"] is False
    assert frozen["identity_consumed"] is True
    assert frozen["retry_forbidden"] is True
    assert frozen["cold_a_successor_execution_authorized"] is False
    assert frozen["u4_authorized"] is False
    assert frozen["production_authorized"] is False
    for verifier in (
        GATE.verify_v3_runtime_no_go_provenance,
        SUPERVISOR.verify_v3_runtime_no_go_provenance,
    ):
        observed = verifier()
        assert observed["decision"] == frozen["decision"]
        assert observed["receipt_chain_verified"] is True
        assert observed["other_receipts_absent"] is True
        assert observed["runtime_baseline_used"] is False
        assert observed["snapshot_artifact_reused"] is False
        assert observed["output_reused"] is False
        assert observed["identity_consumed"] is True
        assert observed["retry_forbidden"] is True
        assert observed["production_authorized"] is False
        assert observed["snapshot"]["preserved_unchanged"] is True


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
@pytest.mark.parametrize("artifact", ["gate", "supervisor", "profile", "schema", "policy"])
def test_each_frozen_v3_snapshot_artifact_is_tamper_sensitive(
    monkeypatch, module, artifact
) -> None:
    frozen = module.FROZEN_V3_NO_GO
    target = Path(frozen["snapshot"]["root"]) / frozen["snapshot"]["files"][artifact]["name"]
    original = module.read_regular_bytes

    def tampered(path, *args, **kwargs):
        raw = original(path, *args, **kwargs)
        if Path(path) == target:
            assert raw
            return bytes([raw[0] ^ 1]) + raw[1:]
        return raw

    monkeypatch.setattr(module, "read_regular_bytes", tampered)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v3_runtime_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
@pytest.mark.parametrize("receipt_name", ["start", "execution", "lifecycle"])
def test_each_frozen_v3_receipt_is_tamper_sensitive(
    monkeypatch, module, receipt_name
) -> None:
    target = Path(module.FROZEN_V3_NO_GO["receipts"][receipt_name]["path"])
    original = module.read_regular_bytes

    def tampered(path, *args, **kwargs):
        raw = original(path, *args, **kwargs)
        if Path(path) == target:
            assert raw
            return raw[:-1] + bytes([raw[-1] ^ 1])
        return raw

    monkeypatch.setattr(module, "read_regular_bytes", tampered)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v3_runtime_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
@pytest.mark.parametrize("receipt_name", ["start", "execution", "lifecycle"])
def test_each_frozen_v3_receipt_absence_is_rejected(
    monkeypatch, module, receipt_name
) -> None:
    target = Path(module.FROZEN_V3_NO_GO["receipts"][receipt_name]["path"])
    original = module.os.lstat

    def missing(path, *args, **kwargs):
        if Path(path) == target:
            raise FileNotFoundError(str(path))
        return original(path, *args, **kwargs)

    monkeypatch.setattr(module.os, "lstat", missing)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v3_runtime_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
def test_any_frozen_v3_non_authoritative_receipt_appearance_is_rejected(
    monkeypatch, module
) -> None:
    target = next(iter(module.FROZEN_V3_NO_GO["other_receipt_paths"].values()))
    original = module.os.lstat

    def appeared(path, *args, **kwargs):
        if str(path) == target:
            return SimpleNamespace(st_mode=stat.S_IFREG | 0o440)
        return original(path, *args, **kwargs)

    monkeypatch.setattr(module.os, "lstat", appeared)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v3_runtime_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
@pytest.mark.parametrize("artifact", ["gate", "supervisor", "profile", "schema", "policy"])
def test_each_frozen_v2_snapshot_artifact_is_tamper_sensitive(
    monkeypatch, module, artifact
) -> None:
    frozen = module.FROZEN_V2_PRESTART_NO_GO
    target = Path(frozen["snapshot"]["root"]) / frozen["snapshot"]["files"][artifact]["name"]
    original = module.read_regular_bytes

    def tampered(path, *args, **kwargs):
        raw = original(path, *args, **kwargs)
        if Path(path) == target:
            assert raw
            return bytes([raw[0] ^ 1]) + raw[1:]
        return raw

    monkeypatch.setattr(module, "read_regular_bytes", tampered)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v2_prestart_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
def test_frozen_v2_preflight_receipt_is_tamper_sensitive(monkeypatch, module) -> None:
    target = Path(module.FROZEN_V2_PRESTART_NO_GO["preflight_receipt"]["path"])
    original = module.read_regular_bytes

    def tampered(path, *args, **kwargs):
        raw = original(path, *args, **kwargs)
        if Path(path) == target:
            assert raw
            return raw[:-1] + bytes([raw[-1] ^ 1])
        return raw

    monkeypatch.setattr(module, "read_regular_bytes", tampered)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v2_prestart_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
def test_frozen_v2_preflight_receipt_absence_is_rejected(monkeypatch, module) -> None:
    target = Path(module.FROZEN_V2_PRESTART_NO_GO["preflight_receipt"]["path"])
    original = module.os.lstat

    def missing(path, *args, **kwargs):
        if Path(path) == target:
            raise FileNotFoundError(str(path))
        return original(path, *args, **kwargs)

    monkeypatch.setattr(module.os, "lstat", missing)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v2_prestart_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
def test_any_frozen_v2_nonpreflight_receipt_appearance_is_rejected(
    monkeypatch, module
) -> None:
    target = next(iter(module.FROZEN_V2_PRESTART_NO_GO["other_receipt_paths"].values()))
    original = module.os.lstat

    def appeared(path, *args, **kwargs):
        if str(path) == target:
            return SimpleNamespace(st_mode=stat.S_IFREG | 0o440)
        return original(path, *args, **kwargs)

    monkeypatch.setattr(module.os, "lstat", appeared)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v2_prestart_no_go_provenance()



@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
@pytest.mark.parametrize("artifact", ["gate", "supervisor", "profile", "schema", "policy"])
def test_each_frozen_v1_snapshot_artifact_is_tamper_sensitive(
    monkeypatch, module, artifact
) -> None:
    frozen = module.FROZEN_V1_NO_GO
    target = Path(frozen["snapshot"]["root"]) / frozen["snapshot"]["files"][artifact]["name"]
    original = module.read_regular_bytes

    def tampered(path, *args, **kwargs):
        raw = original(path, *args, **kwargs)
        if Path(path) == target:
            assert raw
            return bytes([raw[0] ^ 1]) + raw[1:]
        return raw

    monkeypatch.setattr(module, "read_regular_bytes", tampered)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v1_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
def test_any_v1_receipt_appearance_invalidates_no_go_provenance(monkeypatch, module) -> None:
    target = next(iter(module.FROZEN_V1_NO_GO["receipt_paths"].values()))
    original = module.os.lstat

    def receipt_appeared(path, *args, **kwargs):
        if str(path) == target:
            return SimpleNamespace(st_mode=stat.S_IFREG | 0o440)
        return original(path, *args, **kwargs)

    monkeypatch.setattr(module.os, "lstat", receipt_appeared)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v1_no_go_provenance()


@pytest.mark.parametrize("module", [GATE, SUPERVISOR], ids=["gate", "supervisor"])
@pytest.mark.parametrize("record_name", ["policy", "start", "execution", "lifecycle_incomplete"])
def test_each_of_four_frozen_v6_files_is_tamper_sensitive(monkeypatch, module, record_name) -> None:
    frozen = module.FROZEN_V6_FAILURE
    target = Path(
        frozen["policy"]["path"]
        if record_name == "policy"
        else frozen["receipts"][record_name]["path"]
    )
    original = module.read_regular_bytes

    def tampered(path, *args, **kwargs):
        raw = original(path, *args, **kwargs)
        if Path(path) == target:
            assert raw
            return bytes([raw[0] ^ 1]) + raw[1:]
        return raw

    monkeypatch.setattr(module, "read_regular_bytes", tampered)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        module.verify_v6_failure_provenance()


def test_exact_status_denial_parser_and_classifier_accept_only_full_field_set() -> None:
    entry = parsed_status_entry()
    assert entry["parse_status"] == "PARSED_UNIQUE_KV"
    assert entry["parse_error"] is None
    assert entry["line_truncated"] is False
    assert set(SUPERVISOR._entry_fields(entry)) == SUPERVISOR.AUDIT_STATUS_ALLOWED_FIELDS
    assert SUPERVISOR._is_exact_status_denial(entry, 3)


def audit_line_mutation_cases():
    return [
        ("apparmor", lambda line: line.replace('apparmor="DENIED"', 'apparmor="ALLOWED"')),
        ("operation", lambda line: line.replace('operation="open"', 'operation="getattr"')),
        ("class", lambda line: line.replace('class="file"', 'class="net"')),
        ("profile", lambda line: line.replace(SUPERVISOR.BOOTSTRAP_PROFILE, SUPERVISOR.RUNTIME_PROFILE)),
        ("name_pid", lambda line: line.replace('/proc/3/status', '/proc/4/status')),
        ("name_stat", lambda line: line.replace('/proc/3/status', '/proc/3/stat')),
        ("audit_pid_zero", lambda line: line.replace('pid=4242', 'pid=0')),
        ("audit_pid_text", lambda line: line.replace('pid=4242', 'pid=abc')),
        ("comm", lambda line: line.replace('comm="python3.12"', 'comm="python3"')),
        ("requested", lambda line: line.replace('requested_mask="r"', 'requested_mask="rw"')),
        ("denied", lambda line: line.replace('denied_mask="r"', 'denied_mask="w"')),
        ("fsuid", lambda line: line.replace('fsuid=1000', 'fsuid=0')),
        ("ouid", lambda line: line.replace('ouid=0', 'ouid=1000')),
        ("extra", lambda line: line + ' error="-13"'),
        ("missing", lambda line: line.replace(' denied_mask="r"', '')),
    ]


@pytest.mark.parametrize("_case,mutate", audit_line_mutation_cases(), ids=[case for case, _ in audit_line_mutation_cases()])
def test_status_denial_rejects_every_field_mutation(_case, mutate) -> None:
    line = mutate(exact_status_audit_line())
    assert line != exact_status_audit_line()
    assert not SUPERVISOR._is_exact_status_denial(SUPERVISOR._parse_audit_line(line), 3)


def test_audit_parser_rejects_duplicate_escaped_truncated_or_control_fields() -> None:
    base = exact_status_audit_line()
    changed = (
        base + ' pid=5',
        base.replace('name="/proc/3/status"', 'name="/proc/3\\x2fstatus"'),
        base + " x=" + ("a" * 5000),
        base.replace('comm="python3.12"', 'comm="python\\n3.12"'),
    )
    for line in changed:
        entry = SUPERVISOR._parse_audit_line(line)
        assert entry["parse_status"] != "PARSED_UNIQUE_KV" or entry["line_truncated"] is True
        assert not SUPERVISOR._is_exact_status_denial(entry, 3)


def test_closed_audit_window_accepts_exactly_one_status_denial() -> None:
    audit = valid_audit()
    assert SUPERVISOR._audit_is_closed_success_window(audit, 3)


def test_audit_loss_or_suppression_marker_is_fail_closed() -> None:
    audit = valid_audit()
    audit["capture_valid"] = False
    audit["capture_errors"] = ["AUDIT_SUPPRESSION_OR_LOSS_MARKER"]
    audit["audit_loss_marker_status"] = "OBSERVED_FAIL_CLOSED"
    audit["audit_loss_marker_matching_total"] = 1
    audit["audit_loss_markers"] = [{
        "marker_codes": ["KAUDITD_CALLBACKS_SUPPRESSED"],
        "line_sha256": "3" * 64,
        "line_size_bytes": 80,
    }]
    assert not SUPERVISOR._audit_is_closed_success_window(audit, 3)
    status, evidence = SUPERVISOR.classify_execution(valid_gate_result(), audit)
    assert status == "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"
    assert evidence["root_cause_status"] == "NOT_CLAIMED"


@pytest.mark.parametrize(
    "marker,code",
    [
        ("host kernel: kauditd_printk_skb: 34 callbacks suppressed",
         "KAUDITD_CALLBACKS_SUPPRESSED"),
        ("host kernel: audit: backlog limit exceeded",
         "AUDIT_BACKLOG_LIMIT_EXCEEDED"),
        ("host kernel: audit: backlog=64 lost=3 rate_limit=0",
         "AUDIT_RECORDS_LOST"),
        ("host kernel: audit: audit_lost=3 audit_rate_limit=0 audit_backlog_limit=64",
         "AUDIT_RECORDS_LOST"),
    ],
)
def test_capture_apparmor_denials_directly_detects_audit_loss_markers(
    monkeypatch, marker, code
) -> None:
    anchor = {
        "boot_id": BOOT_ID,
        "cursor": START_CURSOR,
        "journal_anchor_argv": [
            "/usr/bin/journalctl", "-k", "--no-pager", "--quiet",
            f"--boot={BOOT_COMPACT}", "--lines=0", "--show-cursor",
        ],
    }
    raw = (
        marker + "\n" + exact_status_audit_line() +
        "\n-- cursor: " + END_CURSOR + "\n"
    ).encode("utf-8")
    monkeypatch.setattr(SUPERVISOR, "read_boot_id", lambda: BOOT_ID)
    monkeypatch.setattr(
        SUPERVISOR,
        "run_bounded_command",
        lambda *_args, **_kwargs: {
            "returncode": 0, "failure": None, "stdout": raw, "stderr": b"",
        },
    )
    audit = SUPERVISOR.capture_apparmor_denials(anchor, 3)
    assert audit["capture_valid"] is False
    assert audit["capture_errors"] == ["AUDIT_SUPPRESSION_OR_LOSS_MARKER"]
    assert audit["audit_loss_marker_status"] == "OBSERVED_FAIL_CLOSED"
    assert audit["audit_loss_marker_matching_total"] == 1
    assert audit["audit_loss_marker_storage_overflow"] is False
    assert audit["audit_loss_markers"][0]["marker_codes"] == [code]
    assert audit["expected_status_total"] == 1
    assert not SUPERVISOR._audit_is_closed_success_window(audit, 3)


def test_run_once_uses_post_load_execution_anchor_and_two_ordered_syncs() -> None:
    source = function_source(SUPERVISOR_PATH, "_run_once_guarded")
    ordered_tokens = (
        "audit_anchor = capture_journal_anchor()",
        'run_checked(["/usr/sbin/apparmor_parser", "-a", "-K", "-T", profile_path]',
        "pre_run_journal_sync_result = run_checked(",
        "execution_audit_anchor = capture_journal_anchor()",
        "gate_result = run_bounded_command(",
        "\n        journal_sync_result = run_checked(",
        "audit = capture_apparmor_denials(execution_audit_anchor, success_child_pid)",
    )
    positions = [source.index(token) for token in ordered_tokens]
    assert positions == sorted(positions)
    assert source.count("execution_audit_anchor = capture_journal_anchor()") == 1
    assert source.count(
        "capture_apparmor_denials(execution_audit_anchor, success_child_pid)"
    ) == 1


def audit_summary_mutation_cases():
    return [
        ("capture", lambda value: value.__setitem__("capture_valid", False)),
        ("capture_error", lambda value: value["capture_errors"].append("x")),
        ("boot", lambda value: value.__setitem__("boot_id_after", "11111111-2222-3333-4444-555555555555")),
        ("start_cursor", lambda value: value.__setitem__("start_cursor", "bad")),
        ("equal_cursors", lambda value: value.__setitem__("end_cursor", value["start_cursor"])),
        ("anchor_argv", lambda value: value["journal_anchor_argv"].append("--lines=1")),
        ("query_argv", lambda value: value["journal_query_argv"].append("--since=-1s")),
        ("raw_size", lambda value: value.__setitem__("raw_stdout_size_bytes", 0)),
        ("raw_hash", lambda value: value.__setitem__("raw_stdout_sha256", "g" * 64)),
        ("matching_zero", lambda value: value.__setitem__("matching_total", 0)),
        ("matching_two", lambda value: value.__setitem__("matching_total", 2)),
        ("stored", lambda value: value.__setitem__("stored_count", 0)),
        ("dropped", lambda value: value.__setitem__("dropped_count", 1)),
        ("expected_zero", lambda value: value.__setitem__("expected_status_total", 0)),
        ("expected_two", lambda value: value.__setitem__("expected_status_total", 2)),
        ("stat_denial", lambda value: value.__setitem__("stat_denial_total", 1)),
        ("unexpected", lambda value: value.__setitem__("unexpected_total", 1)),
        ("overflow", lambda value: value.__setitem__("storage_overflow", True)),
        ("bool_count", lambda value: value.__setitem__("matching_total", True)),
        ("sanitized_duplicate", lambda value: value["sanitized_denials"].append(copy.deepcopy(value["sanitized_denials"][0]))),
        ("expected_duplicate", lambda value: value["expected_status_denials"].append(copy.deepcopy(value["expected_status_denials"][0]))),
        ("unexpected_list", lambda value: value["unexpected_denials"].append(copy.deepcopy(value["sanitized_denials"][0]))),
        ("time_order", lambda value: value.__setitem__("run_ended_epoch", 0.5)),
        ("cleanup_signal", lambda value: value["prequery_label_cleanup"]["term_sent"].append({"pid": 1})),
        ("prequery_scan", lambda value: value["prequery_label_cleanup"]["stable_zero_scans"].__setitem__(1, [{"pid": 1}])),
        ("profile_mode", lambda value: value["profiles_before_audit_query"]["aa_status_exact_modes"].__setitem__(SUPERVISOR.BOOTSTRAP_PROFILE, "complain")),
        ("sync_rc", lambda value: value["journal_sync"].__setitem__("returncode", 1)),
        ("postquery_scan", lambda value: value["postquery_stable_zero_labels"].__setitem__(2, [{"pid": 1}])),
    ]


@pytest.mark.parametrize("_case,mutate", audit_summary_mutation_cases(), ids=[case for case, _ in audit_summary_mutation_cases()])
def test_closed_audit_window_rejects_every_count_boundary_or_cleanup_mutation(_case, mutate) -> None:
    audit = valid_audit()
    mutate(audit)
    assert not SUPERVISOR._audit_is_closed_success_window(audit, 3)


@pytest.mark.parametrize("_case,mutate", audit_line_mutation_cases(), ids=[case for case, _ in audit_line_mutation_cases()])
def test_closed_audit_window_rejects_semantically_mutated_entry_even_when_lists_agree(_case, mutate) -> None:
    entry = SUPERVISOR._parse_audit_line(mutate(exact_status_audit_line()))
    audit = valid_audit()
    audit["sanitized_denials"] = [copy.deepcopy(entry)]
    audit["expected_status_denials"] = [copy.deepcopy(entry)]
    assert not SUPERVISOR._audit_is_closed_success_window(audit, 3)


def test_classification_requires_exact_frame_and_exact_one_denial_window() -> None:
    status, evidence = SUPERVISOR.classify_execution(valid_gate_result(), valid_audit())
    assert status == "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V4_CLEANUP_PENDING"
    assert evidence["success_frame"] == valid_success_payload()


def test_classification_preserves_enum_only_helper_failure_without_claiming_root_cause() -> None:
    frame = failure_frame("STATUS_UNEXPECTEDLY_READABLE", "STATUS_READABLE")
    result = {
        **valid_gate_result(),
        "returncode": 2,
        "stdout": b"",
        "stderr": frame,
    }
    status, evidence = SUPERVISOR.classify_execution(result, valid_audit())
    assert status == "ENUMERATED_HELPER_FAILURE_CLEANUP_PENDING"
    assert evidence["failure_frame"] == {
        "boundary_status": "STATUS_READABLE",
        "failure_code": "STATUS_UNEXPECTEDLY_READABLE",
    }
    assert evidence["root_cause_status"] == "NOT_CLAIMED"
    assert evidence["diagnostic_disclosure"] == "ENUM_ONLY_NO_PID_PATH_ERRNO_TEXT_ARGV_ENV_OR_NONCE"

    for mutate_result in (
        lambda value: value.__setitem__("returncode", True),
        lambda value: value.__setitem__("returncode", 1),
        lambda value: value.__setitem__("stderr", b"NO_GO\n"),
        lambda value: value.__setitem__("stdout", value["stdout"][:-1]),
        lambda value: value.__setitem__("failure", "timeout"),
    ):
        result = valid_gate_result()
        mutate_result(result)
        observed, _ = SUPERVISOR.classify_execution(result, valid_audit())
        assert observed == "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"

    for mutate_audit in (
        lambda value: value.__setitem__("matching_total", 0),
        lambda value: value.__setitem__("expected_status_total", 0),
        lambda value: value.__setitem__("stat_denial_total", 1),
        lambda value: value.__setitem__("unexpected_total", 1),
    ):
        audit = valid_audit()
        mutate_audit(audit)
        observed, _ = SUPERVISOR.classify_execution(valid_gate_result(), audit)
        assert observed == "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"


def test_three_stable_process_label_zero_scans_are_exact(monkeypatch) -> None:
    calls: list[int] = []
    monkeypatch.setattr(SUPERVISOR, "labeled_processes", lambda: calls.append(1) or [])
    monkeypatch.setattr(SUPERVISOR.time, "sleep", lambda _seconds: None)
    assert SUPERVISOR.require_stable_zero_labels() == [[], [], []]
    assert len(calls) == 3

    sequence = iter(([], [{"pid": 3}], []))
    monkeypatch.setattr(SUPERVISOR, "labeled_processes", lambda: next(sequence))
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.require_stable_zero_labels()


def test_nominal_label_cleanup_requires_no_signal_and_three_empty_scans(monkeypatch) -> None:
    monkeypatch.setattr(SUPERVISOR, "labeled_processes", lambda: [])
    monkeypatch.setattr(SUPERVISOR, "require_stable_zero_labels", lambda: [[], [], []])
    monkeypatch.setattr(SUPERVISOR, "_signal_labeled", lambda *_a, **_k: [])
    assert SUPERVISOR.terminate_labeled_processes() == empty_label_cleanup()


def test_profile_count_validator_requires_exact_loaded_and_unloaded_states() -> None:
    SUPERVISOR.require_profile_counts(profile_state(loaded=True), 1)
    SUPERVISOR.require_profile_counts(profile_state(loaded=False), 0)
    for loaded, mutation in (
        (True, lambda value: value["aa_status_exact_modes"].__setitem__(SUPERVISOR.BOOTSTRAP_PROFILE, "complain")),
        (True, lambda value: value["kernel_exact_counts"].__setitem__(SUPERVISOR.BOOTSTRAP_PROFILE, 2)),
        (False, lambda value: value["kernel_exact_lines"].__setitem__(SUPERVISOR.BOOTSTRAP_PROFILE, ["residue"])),
        (False, lambda value: value["aa_status_exact_presence"].__setitem__(SUPERVISOR.BOOTSTRAP_PROFILE, True)),
    ):
        value = profile_state(loaded=loaded)
        mutation(value)
        with pytest.raises(SUPERVISOR.SupervisorError):
            SUPERVISOR.require_profile_counts(value, 1 if loaded else 0)


def test_sudo_cleanup_evidence_is_exact_and_mutation_sensitive() -> None:
    evidence = valid_sudo_evidence()
    SUPERVISOR.validate_sudo_cleanup_evidence(evidence)
    mutations = (
        lambda value: value.__setitem__("bounding_mode", "drop_all"),
        lambda value: value["pty"].__setitem__("foreground_process_group", 45),
        lambda value: value["membership"].__setitem__("gid", 28),
        lambda value: value["identity_contract"].__setitem__("supplementary_groups", []),
        lambda value: value["clear"].__setitem__("returncode", True),
        lambda value: value["noninteractive_true_must_fail"].__setitem__("stderr_utf8_prefix", ""),
    )
    for mutate in mutations:
        changed = valid_sudo_evidence()
        mutate(changed)
        with pytest.raises(SUPERVISOR.SupervisorError):
            SUPERVISOR.validate_sudo_cleanup_evidence(changed)


def test_lifecycle_pass_requires_unload_sysctl_sudo_and_both_three_scan_sets() -> None:
    cleanup = {**empty_label_cleanup(), "post_unload_stable_zero_scans": [[], [], []], "termination_signals": []}
    sysctls = {"fixture": {"value": "1\n", "sha256": "0" * 64}}
    document = SUPERVISOR.lifecycle_document(
        execution_receipt={"status": "fixture"},
        cleanup=cleanup,
        profiles_after=profile_state(loaded=False),
        sysctls_before=sysctls,
        sysctls_after=copy.deepcopy(sysctls),
        sudo_clear=valid_sudo_evidence(),
    )
    assert document["status"] == "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V4_LIFECYCLE_CLEANUP"
    assert document["production_authorized"] is False

    mutations = (
        lambda args: args["cleanup"]["term_sent"].append({"pid": 3}),
        lambda args: args["cleanup"].__setitem__("stable_zero_scans", [[], []]),
        lambda args: args["cleanup"].__setitem__("post_unload_stable_zero_scans", [[], []]),
        lambda args: args.__setitem__("profiles_after", profile_state(loaded=True)),
        lambda args: args["sysctls_after"].__setitem__("fixture", {"value": "2\n", "sha256": "1" * 64}),
        lambda args: args["sudo_clear"].__setitem__("bounding_mode", "drop_all"),
    )
    for mutate in mutations:
        arguments = {
            "execution_receipt": {"status": "fixture"},
            "cleanup": copy.deepcopy(cleanup),
            "profiles_after": profile_state(loaded=False),
            "sysctls_before": copy.deepcopy(sysctls),
            "sysctls_after": copy.deepcopy(sysctls),
            "sudo_clear": valid_sudo_evidence(),
        }
        mutate(arguments)
        with pytest.raises(SUPERVISOR.SupervisorError):
            SUPERVISOR.lifecycle_document(**arguments)


def test_receipt_writer_is_dirfd_nofollow_o_excl_and_durable() -> None:
    source = function_source(SUPERVISOR_PATH, "write_json_new")
    for token in ("os.O_WRONLY", "os.O_CREAT", "os.O_EXCL", "os.O_NOFOLLOW", "os.O_CLOEXEC", "dir_fd=directory"):
        assert token in source
    assert source.count("os.fsync(") >= 3
    assert "os.fchown(" in source
    assert "os.fchmod(" in source
    assert "metadata.st_nlink != 1" in source


def test_run_once_cleanup_orders_zero_scan_unload_postscan_sudo_and_sysctl() -> None:
    source = function_source(SUPERVISOR_PATH, "_run_once_guarded")
    ordered = (
        "cleanup = terminate_labeled_processes()",
        "pre_unload_zero_proven",
        '"/usr/sbin/apparmor_parser", "-R", "-K"',
        "require_profile_counts(profiles_after, 0)",
        "final_zero = require_stable_zero_labels()",
        "sudo_clear = clear_invoking_user_sudo_timestamp()",
        "sysctls_after = read_sysctls()",
    )
    positions = [source.index(token) for token in ordered]
    assert positions == sorted(positions)
    assert "profile unload forbidden until all-task label zero is proven" in source


def test_document_types_and_statuses_are_fresh_v4_only() -> None:
    source = SUPERVISOR_PATH.read_text(encoding="utf-8")
    prefix = "SMPCC_R8_LIQUID_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V4_"
    for suffix in (
        "START_RECEIPT",
        "PREFLIGHT_FAILURE_RECEIPT",
        "EXECUTION_RECEIPT",
        "LIFECYCLE_RECEIPT",
        "LIFECYCLE_INCOMPLETE_RECEIPT",
        "RECOVERY_RECEIPT",
    ):
        assert prefix + suffix in source
    assert "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V4_CLEANUP_PENDING" in source
    assert "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V4_LIFECYCLE_CLEANUP" in source


def test_static_sources_contain_no_privileged_or_probe_execution_in_tests() -> None:
    tree = ast.parse(Path(__file__).read_text(encoding="utf-8"))
    forbidden_calls = {"sudo", "aa-exec", "apparmor_parser", "bwrap"}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Attribute) and node.func.attr in {"run", "call", "check_call", "check_output"}:
            assert not (
                isinstance(node.func.value, ast.Name)
                and node.func.value.id == "subprocess"
            )
    source = Path(__file__).read_text(encoding="utf-8")
    assert not any(f"['{command}'" in source for command in forbidden_calls)
