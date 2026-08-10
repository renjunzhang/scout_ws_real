"""Pure static/mock tests for the fresh v8 harmless denial-discovery successor.

These tests must never call sudo, AppArmor tools, bwrap, namespaces, the
liquid binaries, ROS, or GPU tooling.
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
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from jsonschema import Draft202012Validator


PACKAGE = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v8.py"
SUPERVISOR_PATH = PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v8.py"
PROFILE_PATH = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v8.profile"
POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v8.json"
SCHEMA_PATH = PACKAGE / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v8.json"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GATE = load_module(GATE_PATH, "u3_stdio_probe_gate_v8_test")
SUPERVISOR = load_module(SUPERVISOR_PATH, "u3_stdio_probe_supervisor_v8_test")


def policy_and_schema():
    return (
        json.loads(POLICY_PATH.read_text(encoding="utf-8")),
        json.loads(SCHEMA_PATH.read_text(encoding="utf-8")),
    )


def valid_identity():
    return {
        "uid": [0, 0, 0, 0],
        "gid": [0, 0, 0, 0],
        "groups": [],
        "capabilities": {"CapInh": 0, "CapPrm": 0, "CapEff": 0, "CapBnd": 0, "CapAmb": 0},
        "no_new_privs": 1,
    }


def identity_mutation_cases():
    cases = [
        ("extra_identity_key", lambda value: value.__setitem__("unexpected", 0)),
        ("uid_bool", lambda value: value["uid"].__setitem__(0, False)),
        ("gid_bool", lambda value: value["gid"].__setitem__(0, False)),
        ("groups_bool", lambda value: value.__setitem__("groups", [False])),
        ("nnp_bool", lambda value: value.__setitem__("no_new_privs", True)),
        ("uid_numeric_mismatch", lambda value: value["uid"].__setitem__(0, 1)),
        ("gid_numeric_mismatch", lambda value: value["gid"].__setitem__(0, 1)),
        ("nnp_zero", lambda value: value.__setitem__("no_new_privs", 0)),
        ("uid_short", lambda value: value.__setitem__("uid", value["uid"][:-1])),
        ("uid_long", lambda value: value["uid"].append(0)),
        ("gid_short", lambda value: value.__setitem__("gid", value["gid"][:-1])),
        ("gid_long", lambda value: value["gid"].append(0)),
        ("groups_nonempty", lambda value: value.__setitem__("groups", [0])),
        ("uid_not_list", lambda value: value.__setitem__("uid", False)),
        ("gid_not_list", lambda value: value.__setitem__("gid", False)),
        ("groups_not_list", lambda value: value.__setitem__("groups", False)),
        ("capabilities_not_object", lambda value: value.__setitem__("capabilities", False)),
    ]
    for key in ("uid", "gid", "groups", "capabilities", "no_new_privs"):
        cases.append((f"missing_{key}", lambda value, key=key: value.pop(key)))
    for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"):
        cases.append((f"missing_{key}", lambda value, key=key: value["capabilities"].pop(key)))
        cases.append((f"bool_{key}", lambda value, key=key: value["capabilities"].__setitem__(key, False)))
        cases.append((f"nonzero_{key}", lambda value, key=key: value["capabilities"].__setitem__(key, 1)))
    cases.append(("extra_capability", lambda value: value["capabilities"].__setitem__("CapAuditRead", 0)))
    return cases


IDENTITY_MUTATIONS = identity_mutation_cases()


def valid_success_payload():
    return {
        "status": "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V8",
        "bootstrap_profile": GATE.BOOTSTRAP_PROFILE,
        "bootstrap_label": GATE.BOOTSTRAP_PROFILE + " (enforce)",
        "bootstrap_identity": valid_identity(),
        "work_tmpfs": {
            "filesystem_type": "tmpfs",
            "total_bytes": GATE.TMPFS_BYTES,
            "mount_options": ["rw", "nosuid", "nodev", "relatime"],
        },
        "inputs": {name: {"size_bytes": size, "sha256": digest} for name, (size, digest) in GATE.INPUT_CONTRACT.items()},
        "host_stdin_consumed_and_fd0_replaced_with_eof_pipe": True,
        "fds_after_stdin": [0, 1, 2],
        "runtime": {
            "label": GATE.RUNTIME_PROFILE + " (enforce)",
            "identity": valid_identity(),
            "returncode": -int(signal.SIGTERM),
            "stdin": "guest_internal_eof_pipe_fd0",
            "stdout_stderr": "internal_empty_pipes",
        },
        "fds_before_success": [0, 1, 2],
        "host_writable_mounts": [],
    }


def success_frame(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("ascii")
    return GATE.SUCCESS_MAGIC + struct.pack(">I", len(payload)) + payload + hashlib.sha256(payload).digest()


def proc_stat(pid: int, starttime: int, pgrp: int | None = None) -> str:
    fields = ["S", "1", str(pid if pgrp is None else pgrp)] + ["0"] * 16 + [str(starttime)]
    assert len(fields) == 20
    return f"{pid} (fixture) " + " ".join(fields) + "\n"


def make_task(proc_root: Path, tgid: int, tid: int, label: str, *, tgid_start: int = 100, tid_start: int = 101, uid: int = 1000) -> None:
    process = proc_root / str(tgid)
    task = process / "task" / str(tid)
    task.mkdir(parents=True, exist_ok=True)
    (process / "status").write_text(f"Uid:\t{uid}\t{uid}\t{uid}\t{uid}\n", encoding="ascii")
    (process / "stat").write_text(proc_stat(tgid, tgid_start), encoding="ascii")
    (task / "stat").write_text(proc_stat(tid, tid_start, tgid), encoding="ascii")
    (task / "attr").mkdir(exist_ok=True)
    (task / "attr/current").write_text(label + " (enforce)\n", encoding="utf-8")


def test_schema_is_deep_closed_and_current_policy_validates() -> None:
    policy, schema = policy_and_schema()
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(policy)
    GATE._assert_schema_objects_closed(schema)
    GATE._validate_schema_instance(policy, schema, schema)
    SUPERVISOR._validate_schema_instance(policy, schema, schema)
    assert set(policy) == set(schema["required"])


@pytest.mark.parametrize(
    "mutate",
    [
        lambda value: value["authorization"].__setitem__("static_task_execution_performed", 0),
        lambda value: value["authorization"].__setitem__("root_owned_fd_capability_required", 1),
        lambda value: value["identity_contract"]["host_capability_sets"].__setitem__("CapEff", False),
        lambda value: value["receipt_contract"].__setitem__("same_uid_tamper_resistance_claimed", 0),
        lambda value: value["recovery_contract"].__setitem__("probe_execution", 0),
        lambda value: value["invariants"].__setitem__("no_gencase_solver", 1),
        lambda value: value["authorization"].__setitem__("unexpected", False),
    ],
)
def test_stdlib_schema_validator_rejects_nested_mutations_and_bool_int_aliases(mutate) -> None:
    policy, schema = policy_and_schema()
    mutate(policy)
    with pytest.raises(GATE.GateError):
        GATE._validate_schema_instance(policy, schema, schema)
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR._validate_schema_instance(policy, schema, schema)


def test_artifact_hashes_and_static_checks_are_subprocess_free(monkeypatch) -> None:
    policy, schema = policy_and_schema()
    for name, path in {
        "gate": GATE_PATH,
        "supervisor": SUPERVISOR_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
    }.items():
        raw = path.read_bytes()
        assert policy["trusted_artifacts"][name]["sha256"] == hashlib.sha256(raw).hexdigest()
        assert policy["trusted_artifacts"][name]["size_bytes"] == len(raw)
    monkeypatch.setattr(subprocess, "Popen", lambda *_a, **_k: pytest.fail("static check attempted Popen"))
    assert GATE.verify_static()["execution_performed"] is False
    assert SUPERVISOR.repository_static_review()["execution_performed"] is False


def test_v8_identity_is_disjoint_and_frozen_v3_through_v7_bytes_and_v7_failure_evidence_are_preserved() -> None:
    assert GATE.PROBE_ID == SUPERVISOR.PROBE_ID == "u3_stdio_apparmor_transport_probe_v8_20260807T183304Z"
    assert str(GATE.SNAPSHOT_ROOT) == "/run/r8-liquid-u3_stdio_apparmor_transport_probe_v8_20260807T183304Z.snapshot"
    assert "v8-20260807t183304z" in GATE.BOOTSTRAP_PROFILE
    assert "v8-20260807t183304z" in GATE.RUNTIME_PROFILE
    frozen_v3 = {
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v3.py": (56980, "0fed7400461bd2adf1012aeb3624b7e6a919ac3b7c50fd301c0be6e6f182935f"),
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v3.py": (82443, "4f6d1c2266a67d960aa08a1b664431bc957cba5d0e5d27252d4f6432f3419dcc"),
        PACKAGE / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v3.profile": (3620, "c392228bfb37c2e7b7569e4025f1e643c96d3b8681366de4acf451f34c5dfdf5"),
        PACKAGE / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v3.json": (22411, "fd9655aa97f4c337c90cff92ba484bbd002fc5bb27db4855d41e76071621d6a3"),
        PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v3.json": (17293, "07fb5d84afddec937874e0df4bf4d59be1a95475af213de5face078a1c5c3d22"),
        PACKAGE / "tests/test_target_u3_stdio_apparmor_transport_probe_gate_v3.py": (40692, "90b4082ed6b80dbce0ab4d3f1cbf5b6e5f7c6e5911f3e395d725a9085f1df4da"),
    }
    for path, (size, digest) in frozen_v3.items():
        raw = path.read_bytes()
        assert len(raw) == size
        assert hashlib.sha256(raw).hexdigest() == digest
    frozen_v4 = {
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v4.py": (59629, "c831d42aa05cfe5f3b167e3aa5fe2e26914fa231b9184e77bd51c6c73caed304"),
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v4.py": (86160, "f7d02a577b7d1193ea151e020d78fe63c59581730a366df56ff18840f01c2c17"),
        PACKAGE / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v4.profile": (3630, "08abe9ccc30cde97d23a7b49dd5c8810ec7e86d85f782c28bb5c75b12d77e5cb"),
        PACKAGE / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v4.json": (24669, "ea9a5ed12c71f1503ec879b1efe0821d702a90066857e032a52ab0d9efbb04d4"),
        PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v4.json": (18658, "5111ce31713aa7f61e2d1921617101a853ed410bb6e99c55472c7f212000f0df"),
        PACKAGE / "tests/test_target_u3_stdio_apparmor_transport_probe_gate_v4.py": (48299, "5719c987b2e1dbf1cc2bdf1a332b5891d45d719e909984a647d88e05ad914929"),
    }
    for path, (size, digest) in frozen_v4.items():
        raw = path.read_bytes()
        assert len(raw) == size
        assert hashlib.sha256(raw).hexdigest() == digest
    frozen_v5 = {
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v5.py": (61309, "88f48cdae677687f19346d50459ac32afaca5df86c670cde5839f9026cadb413"),
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v5.py": (90757, "c40e58056ae88e2f209ea7cadf062381df9c3fdea45e97bce080c3b6cda3f52b"),
        PACKAGE / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v5.profile": (3630, "5c3a53a3c35ecbc0cc5f0b7a0a7e72398d787eeda4d5f9bef3973a4638723826"),
        PACKAGE / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v5.json": (26754, "1c498220f464c9a6ab40a22e172aae8f8503871e14ef6ef940b954a8438cef19"),
        PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v5.json": (19514, "acaee4e4c6bebba7069dfeb0776ca98ed94d3642fd0dd1cf11862a77e7026e31"),
        PACKAGE / "tests/test_target_u3_stdio_apparmor_transport_probe_gate_v5.py": (61876, "3ea0ec60853f151b58a6b141a83da7894123e5bde20970460ca1bcf8d770a743"),
    }
    for path, (size, digest) in frozen_v5.items():
        raw = path.read_bytes()
        assert len(raw) == size
        assert hashlib.sha256(raw).hexdigest() == digest
    frozen_v6 = {
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v6.py": (61770, "711da838735d49c09762e410db667d99d78392af6f010ae4936498b99e4e2be9"),
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v6.py": (91220, "56a6c615cdee1db6ee9d1a548f2b43cbd36870fee593ac8c39748f29948845e7"),
        PACKAGE / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v6.profile": (3630, "7e0beab340f276d42eb3cbd80f291520b3c397cd1dcceeb39817bf3cfbeb0d47"),
        PACKAGE / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v6.json": (27035, "ce5bcf1e5f1ccdeb2b0807dbe14998856fff292afe9b6da7b8799e0092b4aacd"),
        PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v6.json": (19667, "4fd942eb40f84f3ac1442f10aa35bb2d7341144320aba00084d78c0e2e027b2d"),
        PACKAGE / "tests/test_target_u3_stdio_apparmor_transport_probe_gate_v6.py": (65380, "0fc821ea213ffe4dbd534240e69a13ac288aebf15defa9d7323690807cbda779"),
    }
    for path, (size, digest) in frozen_v6.items():
        raw = path.read_bytes()
        assert len(raw) == size
        assert hashlib.sha256(raw).hexdigest() == digest
    frozen_v7 = {
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v7.py": (66509, "57265e792840d77ae02a79f0fc1de21e81acbb8df1cbb787af491ab3b3e40aa4"),
        PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v7.py": (98694, "c44714dc0669d93fcc1dc026add632e686adcb75eba5372326ef10265393c0f4"),
        PACKAGE / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v7.profile": (3630, "14912c90f752d027ab5fa4fe09c6c0f7d21fcb1887d4fe545fe2539705b03169"),
        PACKAGE / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v7.json": (30076, "55dbc8c0b452014e8020e3f9ad1c06c5075195074ecf6d905ec63c76f7d449e5"),
        PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v7.json": (23567, "621585ec3b52a4f71d5e3951893fca1a07f68fc7a3655e65f5480fbe3ee50aef"),
        PACKAGE / "tests/test_target_u3_stdio_apparmor_transport_probe_gate_v7.py": (84688, "a7af75096d5da37faf05e547e6b971cec3ca9267e77865ea7a24c3079466c23c"),
    }
    for path, (size, digest) in frozen_v7.items():
        raw = path.read_bytes()
        assert len(raw) == size
        assert hashlib.sha256(raw).hexdigest() == digest
    policy, _schema = policy_and_schema()
    assert policy["provenance"]["predecessor_no_go"] == GATE.PREDECESSOR_NO_GO == SUPERVISOR.PREDECESSOR_NO_GO
    predecessor = GATE.PREDECESSOR_NO_GO
    assert predecessor["status"] == "PRE_START_SUDO_CLEANUP_BOUNDING_SET_NO_GO"
    assert predecessor["no_retry_same_identity"] is predecessor["identity_consumed"] is True
    assert predecessor["snapshot"] == {
        "created": True,
        "preserve": True,
        "directory_owner": [0, 0],
        "directory_mode": "0555",
        "file_owner": [0, 0],
        "file_mode": "0444",
        "workspace_bytes_match_snapshot": True,
    }
    snapshot_root = Path(predecessor["snapshot_root"])
    snapshot_metadata = snapshot_root.stat()
    assert (snapshot_metadata.st_uid, snapshot_metadata.st_gid, stat.S_IMODE(snapshot_metadata.st_mode)) == (0, 0, 0o555)
    assert set(predecessor["frozen_workspace_artifacts"]) == {"gate", "supervisor", "profile", "schema", "policy", "tests"}
    for name, entry in predecessor["frozen_workspace_artifacts"].items():
        raw = Path(entry["path"]).read_bytes()
        assert len(raw) == entry["size_bytes"]
        assert hashlib.sha256(raw).hexdigest() == entry["sha256"]
        if name != "tests":
            snapshot_path = snapshot_root / Path(entry["path"]).name
            snapshot_raw = snapshot_path.read_bytes()
            snapshot_file_metadata = snapshot_path.stat()
            assert snapshot_raw == raw
            assert (snapshot_file_metadata.st_uid, snapshot_file_metadata.st_gid, stat.S_IMODE(snapshot_file_metadata.st_mode)) == (0, 0, 0o444)
    receipt = predecessor["preflight_receipt"]
    receipt_path = Path(receipt["path"])
    receipt_raw = receipt_path.read_bytes()
    receipt_metadata = receipt_path.stat()
    assert len(receipt_raw) == receipt["size_bytes"] == 921
    assert hashlib.sha256(receipt_raw).hexdigest() == receipt["sha256"]
    assert (receipt_metadata.st_uid, receipt_metadata.st_gid, stat.S_IMODE(receipt_metadata.st_mode)) == (1000, 1000, 0o440)
    assert json.loads(receipt_raw)["status"] == receipt["status"] == (
        "PRE_START_FAILURE_SUDO_TIMESTAMP_CLEANUP_INCOMPLETE_NO_PROFILE_LOAD_OR_PROBE_IDENTITY_CONSUMED"
    )
    for suffix in predecessor["other_receipts_absent"]:
        assert not Path(f"/home/zrj/scout_liquid_lab/audits/{predecessor['probe_id']}.{suffix}.json").exists()
    assert predecessor["execution_boundary"] == {
        "start_receipt_created": False,
        "parser_invoked": False,
        "profile_load_attempted": False,
        "profile_loaded": False,
        "gate_executed": False,
        "bwrap_executed": False,
        "probe_payload_executed": False,
    }
    assert predecessor["v7_cleanup"] == {
        "attempted": True,
        "proven": False,
        "error": "SupervisorError: UID1000 sudo -K all-timestamp invalidation failed",
        "observation_empty": True,
        "frozen_argv_contract_contains": "--bounding-set=-all",
    }
    diagnostic = predecessor["independent_bounding_diagnostic"]
    assert diagnostic["single_diagnostic_not_v7_retry"] is True
    assert diagnostic["persisted_receipt_created"] is False
    assert diagnostic["operator_transcribed_live_pty_only"] is True
    assert diagnostic["pty_combined_only"] is True
    assert diagnostic["stdout_stderr_separation_claimed"] is False
    prompt = b"[sudo] password for zrj: "
    a_post = b"\r\nsudo: unable to change to root gid: Operation not permitted\r\nsudo: error initializing audit plugin sudoers_audit\r\n"
    b_post = b"\r\n"
    password_required = b"sudo: a password is required\r\n"
    assert len(prompt) == diagnostic["prompt"]["size_bytes"] == 25
    assert hashlib.sha256(prompt).hexdigest() == diagnostic["prompt"]["sha256"]
    a = diagnostic["a_with_drop_all_bounding"]
    b = diagnostic["b_preserve_host_bounding"]
    assert a["returncode"] == 1 and b["returncode"] == 0
    assert "--bounding-set=-all" in a["argv"]
    assert all(not token.startswith("--bounding-set") for token in b["argv"])
    assert len(a_post) == a["post_auth_pty_combined_size_bytes"] == 116
    assert hashlib.sha256(a_post).hexdigest() == a["post_auth_pty_combined_sha256"]
    assert hashlib.sha256(prompt + a_post).hexdigest() == a["full_pty_combined_sha256"]
    assert len(prompt + a_post) == a["full_pty_combined_size_bytes"] == 141
    assert len(b_post) == b["post_auth_pty_combined_size_bytes"] == 2
    assert hashlib.sha256(b_post).hexdigest() == b["post_auth_pty_combined_sha256"]
    assert hashlib.sha256(prompt + b_post).hexdigest() == b["full_pty_combined_sha256"]
    assert len(prompt + b_post) == b["full_pty_combined_size_bytes"] == 27
    post_b = diagnostic["post_b_noninteractive_true"]
    assert post_b["returncode"] == 1 and len(password_required) == post_b["size_bytes"] == 30
    assert hashlib.sha256(password_required).hexdigest() == post_b["sha256"]
    final = diagnostic["final_plain_user_cleanup"]
    assert final["clear"]["returncode"] == 0
    assert final["clear"]["pty_combined_size_bytes"] == 0
    assert final["clear"]["pty_combined_sha256"] == hashlib.sha256(b"").hexdigest()
    assert final["independent_noninteractive_true"]["returncode"] == 1
    assert final["independent_noninteractive_true"]["sha256"] == hashlib.sha256(password_required).hexdigest()
    assert final["closed"] is True
    assert final["closed_scope"] == "independent_diagnostic_observation_only"
    assert final["v7_lifecycle_cleanup_proven"] is False
    assert predecessor["production_authorized"] is False


def test_fresh_v8_snapshot_and_all_six_receipts_are_absent_before_any_authorized_attempt() -> None:
    assert not GATE.SNAPSHOT_ROOT.exists()
    assert set(GATE.FROZEN_RECEIPT_PATHS) == set(SUPERVISOR.FROZEN_RECEIPT_PATHS)
    for path in GATE.FROZEN_RECEIPT_PATHS.values():
        assert not Path(path).exists()


def test_root_owned_0444_snapshot_supervisor_uses_only_pinned_python_entrypoint() -> None:
    policy, _schema = policy_and_schema()
    supervisor = str(SUPERVISOR.SNAPSHOT_PATHS["supervisor"])
    for command, subcommand, token_flag, token in (
        ("run_argv_template", "run", "--admission-token", SUPERVISOR.ADMISSION_TOKEN),
        ("recover_argv_template", "recover-only", "--recovery-token", SUPERVISOR.RECOVERY_TOKEN),
    ):
        argv = policy["fixed_commands"][command]
        assert argv == [
            "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", supervisor,
            subcommand, "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>",
            token_flag, token,
        ]
        assert argv[1] != supervisor
        assert argv.count(supervisor) == 1 and argv.index(supervisor) == 4
    bootstrap = policy["snapshot_bootstrap"]
    assert bootstrap["file_mode"] == "0444"
    assert bootstrap["runtime_supervisor_entrypoint"] == "/usr/bin/python3.12 -I -B reads root-owned 0444 snapshot supervisor"
    assert bootstrap["direct_snapshot_supervisor_exec_forbidden"] is True


def test_aa_status_json_argv_is_one_frozen_primary_option_everywhere() -> None:
    policy, schema = policy_and_schema()
    expected = ("/usr/sbin/aa-status", "--json")
    assert GATE.AA_STATUS_JSON_ARGV == SUPERVISOR.AA_STATUS_JSON_ARGV == expected
    assert policy["fixed_commands"]["aa_status_json_argv"] == list(expected)
    assert schema["properties"]["fixed_commands"]["properties"]["aa_status_json_argv"]["const"] == list(expected)
    tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "profile_state")
    source = ast.get_source_segment(SUPERVISOR_PATH.read_text(encoding="utf-8"), function)
    assert source is not None
    assert "list(AA_STATUS_JSON_ARGV)" in source
    assert '"--show"' not in source


@pytest.mark.parametrize(
    "bad_argv",
    [
        ["/usr/sbin/aa-status", "--json", "--show", "profiles"],
        ["/usr/sbin/aa-status", "--json", "--show"],
        ["/usr/sbin/aa-status", "--json", "profiles"],
        ["/usr/sbin/aa-status"],
        ["/usr/sbin/aa-status", "--pretty-json"],
    ],
)
def test_aa_status_json_argv_mutations_fail_schema_and_both_static_reviewers(
    tmp_path, monkeypatch, bad_argv,
) -> None:
    policy, schema = policy_and_schema()
    policy["fixed_commands"]["aa_status_json_argv"] = bad_argv
    assert list(Draft202012Validator(schema).iter_errors(policy))
    changed = tmp_path / "mutated-aa-status-policy.json"
    changed.write_text(json.dumps(policy), encoding="utf-8")
    monkeypatch.setattr(GATE, "POLICY_PATH", changed)
    repository_paths = dict(SUPERVISOR.REPOSITORY_PATHS)
    repository_paths["policy"] = changed
    monkeypatch.setattr(SUPERVISOR, "REPOSITORY_PATHS", repository_paths)
    with pytest.raises(GATE.GateError, match=r"aa[_-]status"):
        GATE.verify_static()
    with pytest.raises(SUPERVISOR.SupervisorError, match="aa_status_json_argv"):
        SUPERVISOR.repository_static_review()


def insert_argv_tokens_before_delimiter(argv, *tokens) -> None:
    index = argv.index("--")
    argv[index:index] = list(tokens)


@pytest.mark.parametrize(
    "section,key,mutate",
    [
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: value.remove("--groups=27")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: value.__setitem__(value.index("--groups=27"), "--groups=28")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: value.__setitem__(value.index("--groups=27"), "--groups=27,4")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: value.__setitem__(value.index("--groups=27"), "--clear-groups")),
        ("fixed_commands", "sudo_timestamp_verify_argv", lambda value: value.__setitem__(value.index("--groups=27"), "--init-groups")),
        ("fixed_commands", "sudo_timestamp_verify_argv", lambda value: value.__setitem__(value.index("--groups=27"), "--keep-groups")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: insert_argv_tokens_before_delimiter(value, "--bounding-set=-all")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: insert_argv_tokens_before_delimiter(value, "--bounding-set=+all")),
        ("fixed_commands", "sudo_timestamp_verify_argv", lambda value: insert_argv_tokens_before_delimiter(value, "--bounding-set=-cap_sys_admin,+cap_chown")),
        ("fixed_commands", "sudo_timestamp_verify_argv", lambda value: insert_argv_tokens_before_delimiter(value, "--bounding-set", "-all")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: value.remove("--inh-caps=-all")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: value.__setitem__(value.index("--inh-caps=-all"), "--inh-caps=+all")),
        ("fixed_commands", "sudo_timestamp_verify_argv", lambda value: value.remove("--ambient-caps=-all")),
        ("fixed_commands", "sudo_timestamp_verify_argv", lambda value: value.__setitem__(value.index("--ambient-caps=-all"), "--ambient-caps=+all")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: insert_argv_tokens_before_delimiter(value, "--no-new-privs")),
        ("fixed_commands", "sudo_timestamp_clear_argv", lambda value: value.insert(value.index("--") + 1, "/bin/sh")),
        ("fixed_commands", "sudo_timestamp_verify_argv", lambda value: value.insert(value.index("--"), "--")),
        ("receipt_contract", "sudo_group_membership", lambda value: value.__setitem__("record", "sudo:x:27:")),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("supplementary_groups", [])),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("supplementary_groups", [27, 4])),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("bounding_mode", "drop_all")),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value["bounding_argv_tokens"].append("--bounding-set=-all")),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("explicit_bounding_change_forbidden", False)),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("inheritable_capabilities", "+all")),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("ambient_capabilities", "+all")),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("no_new_privs_option_present", True)),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("shell", True)),
        ("receipt_contract", "sudo_cleanup_identity", lambda value: value.__setitem__("start_new_session", True)),
    ],
)
def test_sudo_cleanup_policy_mutations_fail_schema_and_manual_checks_with_relaxed_const(
    tmp_path, monkeypatch, section, key, mutate,
) -> None:
    policy, schema = policy_and_schema()
    mutate(policy[section][key])
    assert list(Draft202012Validator(schema).iter_errors(policy))
    relaxed_schema = copy.deepcopy(schema)
    relaxed_schema["properties"][section]["properties"][key] = {"const": copy.deepcopy(policy[section][key])}
    changed_policy = tmp_path / f"mutated-{section}-{key}.json"
    changed_schema = tmp_path / f"relaxed-{section}-{key}.json"
    changed_policy.write_text(json.dumps(policy), encoding="utf-8")
    changed_schema.write_text(json.dumps(relaxed_schema), encoding="utf-8")
    monkeypatch.setattr(GATE, "POLICY_PATH", changed_policy)
    monkeypatch.setattr(GATE, "SCHEMA_PATH", changed_schema)
    repository_paths = dict(SUPERVISOR.REPOSITORY_PATHS)
    repository_paths["policy"] = changed_policy
    repository_paths["schema"] = changed_schema
    monkeypatch.setattr(SUPERVISOR, "REPOSITORY_PATHS", repository_paths)
    with pytest.raises(GATE.GateError, match="sudo"):
        GATE.verify_static()
    with pytest.raises(SUPERVISOR.SupervisorError, match="sudo"):
        SUPERVISOR.repository_static_review()


def test_sudo_cleanup_bounding_mode_policy_mutation_fails_schema_and_both_manual_reviewers(
    tmp_path, monkeypatch,
) -> None:
    policy, schema = policy_and_schema()
    policy["receipt_contract"]["sudo_cleanup_bounding_mode"] = "drop_all"
    assert list(Draft202012Validator(schema).iter_errors(policy))
    relaxed_schema = copy.deepcopy(schema)
    relaxed_schema["properties"]["receipt_contract"]["properties"]["sudo_cleanup_bounding_mode"] = {
        "const": "drop_all",
    }
    changed_policy = tmp_path / "mutated-cleanup-bounding-mode-policy.json"
    changed_schema = tmp_path / "relaxed-cleanup-bounding-mode-schema.json"
    changed_policy.write_text(json.dumps(policy), encoding="utf-8")
    changed_schema.write_text(json.dumps(relaxed_schema), encoding="utf-8")
    monkeypatch.setattr(GATE, "POLICY_PATH", changed_policy)
    monkeypatch.setattr(GATE, "SCHEMA_PATH", changed_schema)
    repository_paths = dict(SUPERVISOR.REPOSITORY_PATHS)
    repository_paths["policy"] = changed_policy
    repository_paths["schema"] = changed_schema
    monkeypatch.setattr(SUPERVISOR, "REPOSITORY_PATHS", repository_paths)
    with pytest.raises(GATE.GateError, match="sudo"):
        GATE.verify_static()
    with pytest.raises(SUPERVISOR.SupervisorError, match="sudo"):
        SUPERVISOR.repository_static_review()


@pytest.mark.parametrize(
    "field",
    [
        "start_receipt",
        "preflight_failure_receipt",
        "execution_receipt",
        "lifecycle_receipt",
        "lifecycle_failure_receipt",
        "recovery_receipt",
    ],
)
def test_every_frozen_receipt_path_is_exact_and_rejects_v4_alias(tmp_path, monkeypatch, field) -> None:
    policy, schema = policy_and_schema()
    assert policy["frozen_identity"][field] == GATE.FROZEN_RECEIPT_PATHS[field]
    assert policy["frozen_identity"][field] == SUPERVISOR.FROZEN_RECEIPT_PATHS[field]
    policy["frozen_identity"][field] = policy["frozen_identity"][field].replace(
        "probe_v8_20260807T183304Z",
        "probe_v4_20260807T163041Z",
    )
    assert list(Draft202012Validator(schema).iter_errors(policy))
    changed = tmp_path / f"mutated-{field}.json"
    changed.write_text(json.dumps(policy), encoding="utf-8")
    monkeypatch.setattr(GATE, "POLICY_PATH", changed)
    repository_paths = dict(SUPERVISOR.REPOSITORY_PATHS)
    repository_paths["policy"] = changed
    monkeypatch.setattr(SUPERVISOR, "REPOSITORY_PATHS", repository_paths)
    with pytest.raises(GATE.GateError):
        GATE.verify_static()
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.repository_static_review()


@pytest.mark.parametrize(
    "mutate",
    [
        lambda policy: policy["fixed_commands"].__setitem__("run_argv_template", [
            "/usr/bin/sudo", str(SUPERVISOR.SNAPSHOT_PATHS["supervisor"]), "run",
            "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>", "--admission-token",
            SUPERVISOR.ADMISSION_TOKEN, "PAD1", "PAD2", "PAD3",
        ]),
        lambda policy: policy["fixed_commands"].__setitem__("recover_argv_template", [
            "/usr/bin/sudo", str(SUPERVISOR.SNAPSHOT_PATHS["supervisor"]), "recover-only",
            "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>", "--recovery-token",
            SUPERVISOR.RECOVERY_TOKEN, "PAD1", "PAD2", "PAD3",
        ]),
        lambda policy: policy["snapshot_bootstrap"].__setitem__("file_mode", "0555"),
        lambda policy: policy["snapshot_bootstrap"].__setitem__("direct_snapshot_supervisor_exec_forbidden", False),
        lambda policy: policy["snapshot_bootstrap"].__setitem__("workspace_snapshot_producer_root_forbidden", False),
    ],
)
def test_direct_exec_or_snapshot_metadata_mutations_fail_both_static_reviewers(tmp_path, monkeypatch, mutate) -> None:
    policy, _schema = policy_and_schema()
    mutate(policy)
    changed = tmp_path / "mutated-policy.json"
    changed.write_text(json.dumps(policy), encoding="utf-8")
    monkeypatch.setattr(GATE, "POLICY_PATH", changed)
    repository_paths = dict(SUPERVISOR.REPOSITORY_PATHS)
    repository_paths["policy"] = changed
    monkeypatch.setattr(SUPERVISOR, "REPOSITORY_PATHS", repository_paths)
    with pytest.raises(GATE.GateError):
        GATE.verify_static()
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.repository_static_review()


def test_snapshot_runtime_metadata_ast_keeps_0555_directory_and_0444_files() -> None:
    tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
    verify = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "verify_snapshot")
    integer_constants = {node.value for node in ast.walk(verify) if isinstance(node, ast.Constant) and type(node.value) is int}
    assert 0o555 in integer_constants
    assert 0o444 in integer_constants
    assert "os.fchmod(f,0o444)" in SUPERVISOR.SNAPSHOT_BOOTSTRAP_SOURCE


def snapshot_producer_status(*, cap_eff: int = 0) -> str:
    return (
        "Uid:\t1000\t1000\t1000\t1000\n"
        "Gid:\t1000\t1000\t1000\t1000\n"
        "Groups:\t4 24 27 1000\n"
        "CapInh:\t0000000000000000\n"
        "CapPrm:\t0000000000000000\n"
        f"CapEff:\t{cap_eff:016x}\n"
        "CapBnd:\t000001ffffffffff\n"
        "CapAmb:\t0000000000000000\n"
        "NoNewPrivs:\t0\n"
    )


def configure_snapshot_producer_identity(monkeypatch, *, uid: int = 1000, cap_eff: int = 0, canonical_path: bool = True) -> None:
    monkeypatch.setattr(SUPERVISOR.os, "getuid", lambda: uid)
    monkeypatch.setattr(SUPERVISOR.os, "geteuid", lambda: uid)
    monkeypatch.setattr(SUPERVISOR.os, "getgid", lambda: uid)
    monkeypatch.setattr(SUPERVISOR.os, "getegid", lambda: uid)
    monkeypatch.setattr(
        SUPERVISOR,
        "SCRIPT_PATH",
        SUPERVISOR.WORKSPACE_SUPERVISOR_PATH if canonical_path else Path("/tmp/not-the-reviewed-producer.py"),
    )
    original_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if path == Path("/proc/self/status"):
            return snapshot_producer_status(cap_eff=cap_eff)
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)


def test_snapshot_producer_accepts_only_canonical_unprivileged_zero_active_caps(monkeypatch) -> None:
    configure_snapshot_producer_identity(monkeypatch)
    observed = SUPERVISOR.verify_snapshot_producer_identity()
    assert observed["uid"] == [1000] * 4
    assert observed["gid"] == [1000] * 4
    assert observed["active_capabilities"] == {"CapInh": 0, "CapPrm": 0, "CapEff": 0, "CapAmb": 0}
    assert observed["root_forbidden"] is True


@pytest.mark.parametrize("kwargs", [{"uid": 0}, {"canonical_path": False}, {"cap_eff": 1}])
def test_snapshot_producer_rejects_root_wrong_path_or_active_caps(monkeypatch, kwargs) -> None:
    configure_snapshot_producer_identity(monkeypatch, **kwargs)
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.verify_snapshot_producer_identity()


def test_snapshot_import_context_separates_runtime_and_workspace_producer_paths_and_static_review_passes(monkeypatch) -> None:
    monkeypatch.setenv(GATE.SNAPSHOT_ENV, str(GATE.SNAPSHOT_ROOT))
    snapshot_gate = load_module(GATE_PATH, "u3_stdio_probe_gate_v8_snapshot_context_test")
    assert snapshot_gate.SUPERVISOR_PATH == GATE.SNAPSHOT_ROOT / GATE.SUPERVISOR_NAME
    assert snapshot_gate.WORKSPACE_SUPERVISOR_PATH == GATE.WORKSPACE_SUPERVISOR_PATH
    assert snapshot_gate.SUPERVISOR_PATH != snapshot_gate.WORKSPACE_SUPERVISOR_PATH
    source_for_snapshot_path = {
        snapshot_gate.POLICY_PATH: POLICY_PATH,
        snapshot_gate.SCHEMA_PATH: SCHEMA_PATH,
        snapshot_gate.PROFILE_PATH: PROFILE_PATH,
        snapshot_gate.SUPERVISOR_PATH: SUPERVISOR_PATH,
    }

    def mapped_read(path, *, limit=4 * 1024 * 1024):
        raw = source_for_snapshot_path.get(Path(path), Path(path)).read_bytes()
        if len(raw) > limit:
            raise snapshot_gate.GateError("fixture exceeds ceiling")
        return raw

    monkeypatch.setattr(snapshot_gate, "read_regular_bytes", mapped_read)
    assert snapshot_gate.verify_static()["execution_performed"] is False


@pytest.mark.parametrize(
    "bad_path",
    [
        "/run/r8-liquid-u3_stdio_apparmor_transport_probe_v8_20260807T183304Z.snapshot/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v8.py",
        "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/../scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v8.py",
        "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v6.py",
    ],
)
def test_snapshot_producer_policy_rejects_snapshot_alias_parent_alias_and_v6_path_even_with_relaxed_schema(
    tmp_path, monkeypatch, bad_path,
) -> None:
    policy, schema = policy_and_schema()
    policy["snapshot_bootstrap"]["workspace_snapshot_producer_exact_path"] = bad_path
    assert list(Draft202012Validator(schema).iter_errors(policy))
    relaxed_schema = copy.deepcopy(schema)
    relaxed_schema["properties"]["snapshot_bootstrap"]["properties"]["workspace_snapshot_producer_exact_path"] = {
        "type": "string",
    }
    changed_policy = tmp_path / "bad-producer-policy.json"
    changed_schema = tmp_path / "relaxed-producer-schema.json"
    changed_policy.write_text(json.dumps(policy), encoding="utf-8")
    changed_schema.write_text(json.dumps(relaxed_schema), encoding="utf-8")
    monkeypatch.setattr(GATE, "POLICY_PATH", changed_policy)
    monkeypatch.setattr(GATE, "SCHEMA_PATH", changed_schema)
    repository_paths = dict(SUPERVISOR.REPOSITORY_PATHS)
    repository_paths["policy"] = changed_policy
    repository_paths["schema"] = changed_schema
    monkeypatch.setattr(SUPERVISOR, "REPOSITORY_PATHS", repository_paths)
    with pytest.raises(GATE.GateError, match="workspace snapshot producer identity"):
        GATE.verify_static()
    with pytest.raises(SUPERVISOR.SupervisorError, match="workspace snapshot producer identity"):
        SUPERVISOR.repository_static_review()


def test_root_emit_bootstrap_failure_has_strictly_empty_stdout(monkeypatch, capsys) -> None:
    configure_snapshot_producer_identity(monkeypatch, uid=0)
    assert SUPERVISOR.main(["emit-bootstrap"]) == 2
    captured = capsys.readouterr()
    assert captured.out == ""
    assert '"status": "NO_GO"' in captured.err


def test_profile_effective_semantics_are_exact_and_have_no_external_include(tmp_path, monkeypatch) -> None:
    observed = GATE.verify_profile()
    assert observed["effective_line_count"] == 70
    assert observed["effective_lines_sha256"] == GATE.EXPECTED_PROFILE_LINES_SHA256
    policy, schema = policy_and_schema()
    assert policy["mount_discovery"]["effective_profile_lines_sha256"] == observed["effective_lines_sha256"]
    assert policy["profile_semantics"]["effective_lines_sha256"] == observed["effective_lines_sha256"]
    assert schema["properties"]["mount_discovery"]["properties"]["effective_profile_lines_sha256"]["const"] == observed["effective_lines_sha256"]
    assert schema["properties"]["profile_semantics"]["properties"]["effective_lines_sha256"]["const"] == observed["effective_lines_sha256"]
    effective = GATE._effective_lines(PROFILE_PATH.read_text(encoding="utf-8"))
    assert effective == GATE.EXPECTED_PROFILE_LINES
    assert not any(line.startswith(("abi ", "include ")) for line in effective)
    mutated = tmp_path / "mutated.profile"
    mutated.write_text(PROFILE_PATH.read_text(encoding="utf-8").replace("/usr/bin/bwrap rix,", "/usr/bin/bwrap rix,\n  /home/** rw,"), encoding="utf-8")
    monkeypatch.setattr(GATE, "PROFILE_PATH", mutated)
    with pytest.raises(GATE.GateError, match="exact reviewed tuple"):
        GATE.verify_profile()


@pytest.mark.parametrize(
    "bad_digest",
    [
        "24294511ed489c0ac821a83c54232ad9397927fcb38466f89397ff557162ae4f",
        "0" * 64,
    ],
)
def test_profile_semantics_old_or_arbitrary_digest_fails_schema_and_both_reviewers(
    tmp_path, monkeypatch, bad_digest,
) -> None:
    policy, schema = policy_and_schema()
    policy["profile_semantics"]["effective_lines_sha256"] = bad_digest
    assert list(Draft202012Validator(schema).iter_errors(policy))

    relaxed_schema = copy.deepcopy(schema)
    relaxed_schema["properties"]["profile_semantics"]["properties"]["effective_lines_sha256"] = {
        "$ref": "#/$defs/sha256",
    }
    changed_policy = tmp_path / "mutated-profile-semantics-policy.json"
    changed_schema = tmp_path / "relaxed-profile-semantics-schema.json"
    changed_policy.write_text(json.dumps(policy), encoding="utf-8")
    changed_schema.write_text(json.dumps(relaxed_schema), encoding="utf-8")
    monkeypatch.setattr(GATE, "POLICY_PATH", changed_policy)
    monkeypatch.setattr(GATE, "SCHEMA_PATH", changed_schema)
    repository_paths = dict(SUPERVISOR.REPOSITORY_PATHS)
    repository_paths["policy"] = changed_policy
    repository_paths["schema"] = changed_schema
    monkeypatch.setattr(SUPERVISOR, "REPOSITORY_PATHS", repository_paths)
    with pytest.raises(GATE.GateError, match="profile_semantics"):
        GATE.verify_static()
    with pytest.raises(SUPERVISOR.SupervisorError, match="semantics digests disagree"):
        SUPERVISOR.repository_static_review()


def test_mount_baseline_is_direct_literals_exact_and_has_no_proc_guess() -> None:
    tree = ast.parse(GATE_PATH.read_text(encoding="utf-8"))
    assignment = next(node for node in tree.body if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "BASELINE_MOUNT_RULES" for target in node.targets))
    assert isinstance(assignment.value, ast.Tuple)
    assert all(isinstance(element, ast.Constant) and isinstance(element.value, str) for element in assignment.value.elts)
    policy, _schema = policy_and_schema()
    assert tuple(policy["mount_discovery"]["effective_mount_rules"]) == GATE.BASELINE_MOUNT_RULES
    assert policy["mount_discovery"]["proc_rules"] == []
    profile = "\n".join(GATE.EXPECTED_PROFILE_LINES)
    for forbidden in ("/newroot/proc", "fstype=proc", "/dev/", "--bind-fd", "GenCase", "DualSPHysics"):
        assert forbidden not in profile


def test_fixed_frame_round_trip_and_strict_mutations() -> None:
    frame = GATE.build_stdin_frame()
    observed = GATE.parse_stdin_frame(frame)
    assert observed["size_bytes"] == len(frame)
    assert observed["sha256"] == hashlib.sha256(frame).hexdigest()
    for changed in (frame[:-1], frame + b"x", bytes([frame[0] ^ 1]) + frame[1:]):
        with pytest.raises(GATE.GateError):
            GATE.parse_stdin_frame(changed)


@pytest.mark.parametrize(
    "mutate",
    [
        lambda value: value.__setitem__("extra", 1),
        lambda value: value["bootstrap_identity"]["uid"].__setitem__(3, 1),
        lambda value: value["bootstrap_identity"]["capabilities"].__setitem__("CapBnd", 1),
        lambda value: value["runtime"].__setitem__("label", "unconfined"),
        lambda value: value["runtime"].__setitem__("returncode", True),
        lambda value: value["inputs"]["probe_beta.bin"].__setitem__("size_bytes", 518),
        lambda value: value.__setitem__("fds_before_success", [0, 1, 2, 9]),
        lambda value: value.__setitem__("fds_after_stdin", [False, 1, 2]),
        lambda value: value.__setitem__("fds_before_success", [0, True, 2]),
        lambda value: value.__setitem__("fds_after_stdin", [0, 1]),
        lambda value: value.__setitem__("fds_before_success", [0, 1, 2, 3]),
        lambda value: value["work_tmpfs"].__setitem__("filesystem_type", "ext4"),
        lambda value: value["work_tmpfs"].__setitem__("mount_options", ["rw", "nodev"]),
        lambda value: value.__setitem__("host_stdin_consumed_and_fd0_replaced_with_eof_pipe", 1),
    ],
)
def test_success_frame_is_deep_closed_in_gate_and_supervisor(mutate) -> None:
    value = valid_success_payload()
    mutate(value)
    frame = success_frame(value)
    with pytest.raises(GATE.GateError):
        GATE.parse_success_frame(frame)
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.parse_success_frame(frame)


def test_valid_success_frame_is_accepted_by_both_closed_parsers() -> None:
    frame = success_frame(valid_success_payload())
    assert GATE.parse_success_frame(frame)["host_writable_mounts"] == []
    assert SUPERVISOR.parse_success_frame(frame)["payload"]["runtime"]["returncode"] == -15


@pytest.mark.parametrize("context", ["bootstrap", "runtime"])
@pytest.mark.parametrize("_case,mutate", IDENTITY_MUTATIONS, ids=[case for case, _mutate in IDENTITY_MUTATIONS])
def test_success_identity_is_type_sensitive_closed_and_exact_for_both_parsers(context, _case, mutate) -> None:
    value = valid_success_payload()
    identity = value["bootstrap_identity"] if context == "bootstrap" else value["runtime"]["identity"]
    mutate(identity)
    frame = success_frame(value)
    with pytest.raises(GATE.GateError):
        GATE.parse_success_frame(frame)
    with pytest.raises(SUPERVISOR.SupervisorError):
        SUPERVISOR.parse_success_frame(frame)


@pytest.mark.parametrize("_case,mutate", IDENTITY_MUTATIONS, ids=[case for case, _mutate in IDENTITY_MUTATIONS])
def test_host_identity_is_type_sensitive_closed_and_exact(_case, mutate) -> None:
    identity = {
        "uid": [GATE.HOST_UID] * 4,
        "gid": [GATE.HOST_GID] * 4,
        "groups": [],
        "capabilities": {"CapInh": 0, "CapPrm": 0, "CapEff": 0, "CapBnd": 0, "CapAmb": 0},
        "no_new_privs": 1,
    }
    mutate(identity)
    with pytest.raises(GATE.GateError):
        GATE.verify_host_identity(identity)


@pytest.mark.parametrize("module", [GATE, SUPERVISOR])
def test_bounded_extend_never_exceeds_hard_limit(module) -> None:
    target = bytearray(b"abc")
    assert module._bounded_extend(target, b"0123456789", 8) is True
    assert target == b"abc01234"
    assert len(target) == 8
    assert module._bounded_extend(target, b"x", 8) is True
    assert len(target) == 8


def test_gate_scans_every_uid1000_thread_not_only_leader(tmp_path) -> None:
    make_task(tmp_path, 400, 400, "unconfined", tgid_start=90, tid_start=90)
    make_task(tmp_path, 400, 401, GATE.RUNTIME_PROFILE, tgid_start=90, tid_start=91)
    observed = GATE._proc_labeled_members(tmp_path)
    assert [(item["tgid"], item["tid"], item["label"]) for item in observed] == [(400, 401, GATE.RUNTIME_PROFILE)]
    assert observed[0]["tgid_starttime"] == 90
    assert observed[0]["tid_starttime"] == 91


def test_root_supervisor_scans_nonleader_threads_all_uids(tmp_path) -> None:
    make_task(tmp_path, 500, 501, SUPERVISOR.BOOTSTRAP_PROFILE, tgid_start=120, tid_start=121, uid=0)
    observed = SUPERVISOR.labeled_processes(tmp_path)
    assert [(item["tgid"], item["tid"]) for item in observed] == [(500, 501)]
    assert observed[0]["attr_current"].endswith(" (enforce)")


def test_gate_attr_permission_error_fails_closed(tmp_path, monkeypatch) -> None:
    make_task(tmp_path, 600, 600, GATE.BOOTSTRAP_PROFILE)
    original = Path.read_text

    def denied(path, *args, **kwargs):
        if str(path).endswith("attr/current"):
            raise PermissionError(errno.EACCES, "denied", str(path))
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", denied)
    with pytest.raises(GATE.GateError, match="authority"):
        GATE._proc_labeled_members(tmp_path)


@pytest.mark.parametrize("module,function", [(GATE, "_signal_pid"), (SUPERVISOR, "_signal_pidfd")])
def test_signaling_refuses_missing_or_nonpositive_starttime(module, function) -> None:
    call = getattr(module, function)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        if module is GATE:
            call(123, signal.SIGTERM, None)
        else:
            call(123, signal.SIGTERM, expected_starttime=None)
    with pytest.raises((GATE.GateError, SUPERVISOR.SupervisorError)):
        if module is GATE:
            call(123, signal.SIGTERM, 0)
        else:
            call(123, signal.SIGTERM, expected_starttime=0)


def test_pidfd_starttime_mismatch_never_signals_reused_tgid(monkeypatch) -> None:
    sent = []
    monkeypatch.setattr(SUPERVISOR.os, "pidfd_open", lambda _pid, _flags: 77)
    monkeypatch.setattr(SUPERVISOR.os, "close", lambda _fd: None)
    monkeypatch.setattr(SUPERVISOR, "_proc_starttime", lambda _pid: 999)
    monkeypatch.setattr(SUPERVISOR.signal, "pidfd_send_signal", lambda *_args: sent.append(_args))
    assert SUPERVISOR._signal_pidfd(123, signal.SIGTERM, expected_starttime=111) is False
    assert sent == []


def test_root_admission_fd_capability_is_strict_single_consumer(monkeypatch) -> None:
    payload = b"a" * 32
    digest = hashlib.sha256(payload).hexdigest()
    monkeypatch.setenv(GATE.ADMISSION_FD_ENV, "9")
    monkeypatch.setenv(GATE.ADMISSION_SHA256_ENV, digest)
    metadata = SimpleNamespace(st_mode=stat.S_IFIFO | 0o600, st_uid=0, st_gid=0)
    monkeypatch.setattr(GATE.os, "fstat", lambda _fd: metadata)
    monkeypatch.setattr(GATE.os, "get_inheritable", lambda _fd: True)
    blocks = iter((payload[:3], payload[3:17], payload[17:], b""))
    monkeypatch.setattr(GATE.os, "read", lambda _fd, _size: next(blocks))
    closed = []
    monkeypatch.setattr(GATE.os, "close", lambda fd: closed.append(fd))
    observed = GATE.consume_root_admission_capability()
    assert observed["sha256"] == digest
    assert closed == [9]
    assert GATE.ADMISSION_FD_ENV not in os.environ


def test_user_owned_or_trailing_admission_pipe_fails_closed_and_closes(monkeypatch) -> None:
    payload = b"b" * 32
    monkeypatch.setenv(GATE.ADMISSION_FD_ENV, "10")
    monkeypatch.setenv(GATE.ADMISSION_SHA256_ENV, hashlib.sha256(payload).hexdigest())
    monkeypatch.setattr(GATE.os, "fstat", lambda _fd: SimpleNamespace(st_mode=stat.S_IFIFO | 0o600, st_uid=1000, st_gid=1000))
    monkeypatch.setattr(GATE.os, "get_inheritable", lambda _fd: True)
    closed = []
    monkeypatch.setattr(GATE.os, "close", lambda fd: closed.append(fd))
    with pytest.raises(GATE.GateError, match="inode"):
        GATE.consume_root_admission_capability()
    assert closed == [10]


class ChunkedReader:
    def __init__(self, raw: bytes, chunk: int, trailing: bytes = b"") -> None:
        self.raw = raw + trailing
        self.chunk = chunk
        self.position = 0

    def read(self, size: int = -1) -> bytes:
        if self.position >= len(self.raw):
            return b""
        count = len(self.raw) - self.position if size < 0 else min(size, self.chunk, len(self.raw) - self.position)
        result = self.raw[self.position:self.position + count]
        self.position += count
        return result


def test_snapshot_bootstrap_loader_accepts_fragmented_exact_stream_without_running_bootstrap(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(buffer=ChunkedReader(SUPERVISOR.SNAPSHOT_BOOTSTRAP_BYTES, 7)))
    real_compile = compile
    called = []

    def harmless_compile(source, *_args, **_kwargs):
        called.append(bytes(source))
        return real_compile("raise RuntimeError('validated_bootstrap_not_executed')", "<mock>", "exec")

    with pytest.raises(RuntimeError, match="validated_bootstrap_not_executed"):
        exec(SUPERVISOR.SNAPSHOT_BOOTSTRAP_LOADER_SOURCE, {"compile": harmless_compile})
    assert called == [SUPERVISOR.SNAPSHOT_BOOTSTRAP_BYTES]


def test_snapshot_bootstrap_loader_rejects_trailing_stream_before_compile(monkeypatch) -> None:
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(buffer=ChunkedReader(SUPERVISOR.SNAPSHOT_BOOTSTRAP_BYTES, 11, b"x")))
    with pytest.raises(SystemExit) as failure:
        exec(SUPERVISOR.SNAPSHOT_BOOTSTRAP_LOADER_SOURCE, {"compile": lambda *_a, **_k: pytest.fail("compiled trailing stream")})
    assert failure.value.code == 81


def test_receipt_write_is_o_excl_owned_mode_and_fsynced(tmp_path, monkeypatch) -> None:
    target = tmp_path / "receipt.json"
    monkeypatch.setattr(SUPERVISOR, "HOST_UID", os.getuid())
    monkeypatch.setattr(SUPERVISOR, "HOST_GID", os.getgid())
    monkeypatch.setattr(SUPERVISOR, "_open_absolute_directory", lambda _path: os.open(tmp_path, os.O_RDONLY | os.O_DIRECTORY))
    real_fsync = os.fsync
    fsync_calls = []
    monkeypatch.setattr(SUPERVISOR.os, "fsync", lambda fd: fsync_calls.append(fd) or real_fsync(fd))
    record = SUPERVISOR.write_json_new(target, {"safe": True})
    metadata = target.stat()
    assert stat.S_IMODE(metadata.st_mode) == 0o440
    assert metadata.st_uid == os.getuid() and metadata.st_gid == os.getgid()
    assert record["creation"].startswith("O_EXCL_NOFOLLOW")
    assert record["file_and_parent_fsynced"] is True
    assert len(fsync_calls) >= 3
    with pytest.raises(FileExistsError):
        SUPERVISOR.write_json_new(target, {"safe": True})


def test_sudo_cleanup_without_tty_fails_before_any_command(monkeypatch) -> None:
    monkeypatch.setattr(SUPERVISOR.os, "isatty", lambda _fd: False)
    monkeypatch.setattr(SUPERVISOR, "run_bounded_command", lambda *_a, **_k: pytest.fail("sudo command attempted without PTY"))
    with pytest.raises(SUPERVISOR.SupervisorError, match="PTY"):
        SUPERVISOR.clear_invoking_user_sudo_timestamp()


def test_preflight_failure_clears_sudo_and_writes_distinct_o_excl_evidence(monkeypatch) -> None:
    cleanup_evidence = valid_sudo_lifecycle_evidence()
    captured = {}
    monkeypatch.setattr(SUPERVISOR, "clear_invoking_user_sudo_timestamp", lambda: cleanup_evidence)

    def write(path, document, **_kwargs):
        captured["path"] = path
        captured["document"] = copy.deepcopy(document)
        return {"path": str(path), "creation": "O_EXCL_NOFOLLOW_ONE_SHOT_NOT_IMMUTABLE_PARENT_OWNER_CAN_REMOVE"}

    monkeypatch.setattr(SUPERVISOR, "write_json_new", write)
    with pytest.raises(SUPERVISOR.SupervisorError, match="sudo timestamp cleanup proven; preserved receipt"):
        SUPERVISOR.run_once(policy_sha256="0" * 64, admission_token="wrong-token")
    assert captured["path"] == SUPERVISOR.PREFLIGHT_FAILURE_RECEIPT
    document = captured["document"]
    assert document["status"] == "PRE_START_FAILURE_SUDO_TIMESTAMP_CLEANED_NO_PROFILE_LOAD_OR_PROBE_IDENTITY_CONSUMED"
    assert document["sudo_timestamp_cleanup_proven"] is True
    assert document["sudo_timestamp_cleanup_error"] is None
    assert document["sudo_timestamp_observation"] == cleanup_evidence
    assert document["start_receipt_created_by_this_run"] is False
    assert document["parser_invoked_by_this_run"] is False
    assert document["profile_load_attempted_by_this_run"] is False
    assert document["probe_executed_by_this_run"] is False
    assert document["identity_consumed"] is True


def test_preflight_cleanup_tool_failure_still_writes_fail_closed_receipt_with_empty_observation(monkeypatch) -> None:
    calls = []
    captured = {}

    def reject_tool(name):
        calls.append(name)
        raise SUPERVISOR.SupervisorError("fixture tool hash mismatch")

    monkeypatch.setattr(SUPERVISOR, "require_tool", reject_tool)
    monkeypatch.setattr(
        SUPERVISOR,
        "run_bounded_command",
        lambda *_a, **_k: pytest.fail("unverified cleanup tool reached Popen path"),
    )
    def write(path, document, **_kwargs):
        captured["path"] = path
        captured["document"] = copy.deepcopy(document)
        return {"path": str(path), "creation": "O_EXCL_NOFOLLOW_ONE_SHOT_NOT_IMMUTABLE_PARENT_OWNER_CAN_REMOVE"}

    monkeypatch.setattr(SUPERVISOR, "write_json_new", write)
    with pytest.raises(SUPERVISOR.SupervisorError, match="cleanup incomplete; preserved receipt"):
        SUPERVISOR.run_once(policy_sha256="0" * 64, admission_token="wrong-token")
    assert calls == ["setpriv"]
    assert captured["path"] == SUPERVISOR.PREFLIGHT_FAILURE_RECEIPT
    document = captured["document"]
    assert document["status"] == "PRE_START_FAILURE_SUDO_TIMESTAMP_CLEANUP_INCOMPLETE_NO_PROFILE_LOAD_OR_PROBE_IDENTITY_CONSUMED"
    assert document["sudo_timestamp_cleanup_attempted"] is True
    assert document["sudo_timestamp_cleanup_proven"] is False
    assert document["sudo_timestamp_cleanup_error"] == "SupervisorError: fixture tool hash mismatch"
    assert document["sudo_timestamp_observation"] == {}
    assert document["start_receipt_created_by_this_run"] is False
    assert document["parser_invoked_by_this_run"] is False
    assert document["profile_load_attempted_by_this_run"] is False
    assert document["probe_executed_by_this_run"] is False


def test_admission_cleanup_failure_is_not_retried_and_reaches_no_sysctl_profile_parser_start_or_probe(monkeypatch) -> None:
    captured = {}
    cleanup_calls = []
    monkeypatch.setattr(SUPERVISOR, "verify_snapshot", lambda _digest: ({"authorization": {"attempts_per_identity": 1}}, {"snapshot": "fixture"}))
    monkeypatch.setattr(SUPERVISOR, "_assert_one_shot_paths_absent", lambda: None)
    monkeypatch.setattr(SUPERVISOR, "require_tool", lambda name: {"name": name})

    def fail_cleanup():
        cleanup_calls.append("cleanup")
        raise SUPERVISOR.SupervisorError("fixture admission cleanup failure")

    monkeypatch.setattr(SUPERVISOR, "clear_invoking_user_sudo_timestamp", fail_cleanup)
    for name in ("read_sysctls", "profile_state", "require_stable_zero_labels", "run_checked", "create_root_admission_pipe", "gate_handoff_argv"):
        monkeypatch.setattr(SUPERVISOR, name, lambda *_a, _name=name, **_k: pytest.fail(f"admission failure reached {_name}"))

    def write(path, document, **_kwargs):
        assert path == SUPERVISOR.PREFLIGHT_FAILURE_RECEIPT
        captured["document"] = copy.deepcopy(document)
        return {"path": str(path), "creation": "O_EXCL_NOFOLLOW_ONE_SHOT_NOT_IMMUTABLE_PARENT_OWNER_CAN_REMOVE"}

    monkeypatch.setattr(SUPERVISOR, "write_json_new", write)
    with pytest.raises(SUPERVISOR.SupervisorError, match="cleanup incomplete; preserved receipt"):
        SUPERVISOR.run_once(policy_sha256="0" * 64, admission_token=SUPERVISOR.ADMISSION_TOKEN)
    assert cleanup_calls == ["cleanup"]
    document = captured["document"]
    assert document["sudo_timestamp_cleanup_error"] == "SupervisorError: fixture admission cleanup failure"
    assert document["sudo_timestamp_observation"] == {}
    assert all(document[key] is False for key in (
        "start_receipt_created_by_this_run", "parser_invoked_by_this_run",
        "profile_load_attempted_by_this_run", "probe_executed_by_this_run",
    ))


def test_post_start_cleanup_exception_does_not_double_clear_sudo(monkeypatch) -> None:
    def fail_after_cleanup(*, termination_guard, **_kwargs):
        termination_guard.begin_cleanup()
        raise SUPERVISOR.SupervisorError("fixture post-start lifecycle failure")

    monkeypatch.setattr(SUPERVISOR, "_run_once_guarded", fail_after_cleanup)
    monkeypatch.setattr(
        SUPERVISOR,
        "clear_invoking_user_sudo_timestamp",
        lambda: pytest.fail("post-start exception attempted a second sudo cleanup"),
    )
    with pytest.raises(SUPERVISOR.SupervisorError, match="post-start lifecycle failure"):
        SUPERVISOR.run_once(policy_sha256="0" * 64, admission_token=SUPERVISOR.ADMISSION_TOKEN)


def configure_foreground_tty(monkeypatch, *, foreground_pgrp: int = 44, current_pgrp: int = 44) -> None:
    real_fstat = os.fstat
    monkeypatch.setattr(SUPERVISOR.os, "isatty", lambda _fd: True)
    monkeypatch.setattr(
        SUPERVISOR.os,
        "fstat",
        lambda fd: SimpleNamespace(st_dev=1, st_ino=2, st_rdev=3) if fd == 0 else real_fstat(fd),
    )
    monkeypatch.setattr(SUPERVISOR.os, "tcgetpgrp", lambda _fd: foreground_pgrp)
    monkeypatch.setattr(SUPERVISOR.os, "getpgrp", lambda: current_pgrp)
    monkeypatch.setattr(SUPERVISOR.os, "getsid", lambda _pid: 33)


def valid_sudo_cleanup_result(argv):
    verify = argv[-3:] == ["/usr/bin/sudo", "-n", "/usr/bin/true"]
    return {
        "argv": list(argv),
        "returncode": 1 if verify else 0,
        "stdout": b"",
        "stderr": b"sudo: a password is required\n" if verify else b"",
        "failure": None,
        "start_new_session": False,
    }


def test_sudo_group_membership_is_exact_root_owned_0644_record() -> None:
    observed = SUPERVISOR.verify_sudo_group_membership()
    assert observed == SUPERVISOR.SUDO_GROUP_MEMBERSHIP_CONTRACT
    assert observed["record"] == "sudo:x:27:zrj"
    assert observed["members"] == ["zrj"]


@pytest.mark.parametrize(
    "raw,uid,gid,mode",
    [
        (b"sudo:x:27:\n", 0, 0, 0o644),
        (b"sudo:x:27:zrj,other\n", 0, 0, 0o644),
        (b"sudo:x:27:zrj\nother:x:27:\n", 0, 0, 0o644),
        (b"sudo:x:27:zrj\n", 1000, 0, 0o644),
        (b"sudo:x:27:zrj\n", 0, 1000, 0o644),
        (b"sudo:x:27:zrj\n", 0, 0, 0o664),
    ],
)
def test_sudo_group_membership_rejects_record_owner_or_mode_mutation(monkeypatch, raw, uid, gid, mode) -> None:
    monkeypatch.setattr(SUPERVISOR, "read_regular_bytes", lambda *_a, **_k: raw)
    metadata = SimpleNamespace(st_uid=uid, st_gid=gid, st_mode=stat.S_IFREG | mode)
    monkeypatch.setattr(SUPERVISOR.os, "stat", lambda *_a, **_k: metadata)
    with pytest.raises(SUPERVISOR.SupervisorError, match="group|sudo"):
        SUPERVISOR.verify_sudo_group_membership()


def test_sudo_cleanup_rejects_nonforeground_process_group_before_any_command(monkeypatch) -> None:
    configure_foreground_tty(monkeypatch, foreground_pgrp=43, current_pgrp=44)
    monkeypatch.setattr(SUPERVISOR, "run_bounded_command", lambda *_a, **_k: pytest.fail("sudo command attempted outside foreground pgrp"))
    with pytest.raises(SUPERVISOR.SupervisorError, match="foreground process group"):
        SUPERVISOR.clear_invoking_user_sudo_timestamp()


def test_sudo_cleanup_uses_uid1000_all_timestamp_K_and_no_setsid(monkeypatch) -> None:
    configure_foreground_tty(monkeypatch)
    calls = []

    def bounded(argv, **kwargs):
        calls.append((argv, kwargs))
        result = valid_sudo_cleanup_result(argv)
        result["start_new_session"] = kwargs["start_new_session"]
        return result

    monkeypatch.setattr(SUPERVISOR, "run_bounded_command", bounded)
    observed = SUPERVISOR.clear_invoking_user_sudo_timestamp()
    assert observed["scope"].startswith("ALL_UID1000_SUDO_TIMESTAMPS")
    assert observed["bounding_mode"] == SUPERVISOR.SUDO_CLEANUP_BOUNDING_MODE == "preserve_host_only"
    assert len(calls) == 2
    prefix = [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--groups=27",
        "--inh-caps=-all", "--ambient-caps=-all", "--",
    ]
    assert calls[0][0] == prefix + ["/usr/bin/sudo", "-K"]
    assert calls[1][0] == prefix + ["/usr/bin/sudo", "-n", "/usr/bin/true"]
    assert all(kwargs["stdin_bytes"] is None and kwargs["start_new_session"] is False for _argv, kwargs in calls)
    assert observed["membership"] == SUPERVISOR.SUDO_GROUP_MEMBERSHIP_CONTRACT
    assert observed["identity_contract"] == SUPERVISOR.SUDO_CLEANUP_IDENTITY_CONTRACT
    for argv, _kwargs in calls:
        assert all(token != "--bounding-set" and not token.startswith("--bounding-set=") for token in argv)
        assert all(option not in argv for option in ("--clear-groups", "--init-groups", "--keep-groups", "--no-new-privs"))
    policy, schema = policy_and_schema()
    assert policy["receipt_contract"]["sudo_timestamp_scope"] == observed["scope"]
    assert policy["receipt_contract"]["sudo_group_membership"] == observed["membership"]
    assert policy["receipt_contract"]["sudo_cleanup_identity"] == observed["identity_contract"]
    assert policy["receipt_contract"]["sudo_cleanup_bounding_mode"] == observed["bounding_mode"]
    assert policy["fixed_commands"]["sudo_timestamp_clear_argv"] == calls[0][0]
    assert policy["fixed_commands"]["sudo_timestamp_verify_argv"] == calls[1][0]
    fixed_schema = schema["properties"]["fixed_commands"]["properties"]
    assert fixed_schema["sudo_timestamp_clear_argv"]["const"] == calls[0][0]
    assert fixed_schema["sudo_timestamp_verify_argv"]["const"] == calls[1][0]


def mutate_argv(result, old: str, new: str) -> None:
    result["argv"][result["argv"].index(old)] = new


def remove_argv_token(result, token: str) -> None:
    result["argv"].remove(token)


def insert_result_argv_tokens_before_delimiter(result, *tokens) -> None:
    insert_argv_tokens_before_delimiter(result["argv"], *tokens)


@pytest.mark.parametrize(
    "mutate",
    [
        lambda argv: insert_argv_tokens_before_delimiter(argv, "--bounding-set=-all"),
        lambda argv: insert_argv_tokens_before_delimiter(argv, "--bounding-set=+all"),
        lambda argv: insert_argv_tokens_before_delimiter(argv, "--bounding-set=-cap_sys_admin,+cap_chown"),
        lambda argv: insert_argv_tokens_before_delimiter(argv, "--bounding-set", "-all"),
        lambda argv: argv.remove("--groups=27"),
        lambda argv: argv.__setitem__(argv.index("--groups=27"), "--groups=28"),
        lambda argv: insert_argv_tokens_before_delimiter(argv, "--groups=27"),
        lambda argv: argv.__setitem__(argv.index("--groups=27"), "--clear-groups"),
        lambda argv: argv.__setitem__(argv.index("--groups=27"), "--init-groups"),
        lambda argv: argv.__setitem__(argv.index("--groups=27"), "--keep-groups"),
        lambda argv: argv.remove("--inh-caps=-all"),
        lambda argv: argv.__setitem__(argv.index("--inh-caps=-all"), "--inh-caps=+all"),
        lambda argv: argv.remove("--ambient-caps=-all"),
        lambda argv: argv.__setitem__(argv.index("--ambient-caps=-all"), "--ambient-caps=+all"),
        lambda argv: insert_argv_tokens_before_delimiter(argv, "--no-new-privs"),
        lambda argv: argv.insert(argv.index("--"), "--"),
        lambda argv: argv.insert(argv.index("--") + 1, "/bin/sh"),
    ],
)
def test_gate_and_supervisor_cleanup_argv_validators_fail_closed_on_every_mutation(mutate) -> None:
    clear_argv, _verify_argv = SUPERVISOR.sudo_timestamp_argvs()
    mutated = list(clear_argv)
    mutate(mutated)
    for validator, error_type in (
        (GATE.validate_sudo_cleanup_argv_contract, GATE.GateError),
        (SUPERVISOR.validate_sudo_cleanup_argv_contract, SUPERVISOR.SupervisorError),
    ):
        with pytest.raises(error_type, match="sudo cleanup"):
            validator(list(mutated), ["/usr/bin/sudo", "-K"])


def test_gate_and_supervisor_cleanup_argv_validators_allow_only_two_exact_inner_commands() -> None:
    clear_argv, verify_argv = SUPERVISOR.sudo_timestamp_argvs()
    for validator in (GATE.validate_sudo_cleanup_argv_contract, SUPERVISOR.validate_sudo_cleanup_argv_contract):
        validator(list(clear_argv), ["/usr/bin/sudo", "-K"])
        validator(list(verify_argv), ["/usr/bin/sudo", "-n", "/usr/bin/true"])
    for validator, error_type in (
        (GATE.validate_sudo_cleanup_argv_contract, GATE.GateError),
        (SUPERVISOR.validate_sudo_cleanup_argv_contract, SUPERVISOR.SupervisorError),
    ):
        with pytest.raises(error_type, match="inner command"):
            validator(list(clear_argv), ["/bin/sh", "-c", "/usr/bin/sudo -K"])


@pytest.mark.parametrize(
    "mutate",
    [
        lambda result: result.__setitem__("returncode", 1),
        lambda result: result.__setitem__("returncode", False),
        lambda result: result.__setitem__("stdout", b"unexpected"),
        lambda result: result.__setitem__("stderr", b"unexpected"),
        lambda result: result.__setitem__("failure", "deadline"),
        lambda result: result.__setitem__("start_new_session", True),
        lambda result: mutate_argv(result, "--reuid=1000", "--reuid=0"),
        lambda result: mutate_argv(result, "--regid=1000", "--regid=0"),
        lambda result: remove_argv_token(result, "--groups=27"),
        lambda result: mutate_argv(result, "--groups=27", "--groups=28"),
        lambda result: mutate_argv(result, "--groups=27", "--groups=27,4"),
        lambda result: mutate_argv(result, "--groups=27", "--clear-groups"),
        lambda result: mutate_argv(result, "--groups=27", "--init-groups"),
        lambda result: mutate_argv(result, "--groups=27", "--keep-groups"),
        lambda result: insert_result_argv_tokens_before_delimiter(result, "--bounding-set=-all"),
        lambda result: insert_result_argv_tokens_before_delimiter(result, "--bounding-set=+all"),
        lambda result: insert_result_argv_tokens_before_delimiter(result, "--bounding-set=-cap_sys_admin,+cap_chown"),
        lambda result: insert_result_argv_tokens_before_delimiter(result, "--bounding-set", "-all"),
        lambda result: mutate_argv(result, "--inh-caps=-all", "--inh-caps=+all"),
        lambda result: mutate_argv(result, "--ambient-caps=-all", "--ambient-caps=+all"),
    ],
)
def test_sudo_K_cleanup_rejects_any_result_or_identity_argv_mutation(monkeypatch, mutate) -> None:
    configure_foreground_tty(monkeypatch)

    def bounded(argv, **_kwargs):
        result = valid_sudo_cleanup_result(argv)
        if argv[-2:] == ["/usr/bin/sudo", "-K"]:
            mutate(result)
        return result

    monkeypatch.setattr(SUPERVISOR, "run_bounded_command", bounded)
    with pytest.raises(SUPERVISOR.SupervisorError, match="sudo -K"):
        SUPERVISOR.clear_invoking_user_sudo_timestamp()


@pytest.mark.parametrize(
    "mutate",
    [
        lambda result: result.__setitem__("returncode", 0),
        lambda result: result.__setitem__("returncode", 2),
        lambda result: result.__setitem__("returncode", True),
        lambda result: result.__setitem__("stdout", b"unexpected"),
        lambda result: result.__setitem__("stderr", b""),
        lambda result: result.__setitem__("stderr", b"sudo: a password is required"),
        lambda result: result.__setitem__("stderr", b"sudo: a password is required\nextra"),
        lambda result: result.__setitem__("failure", "deadline"),
        lambda result: result.__setitem__("start_new_session", True),
        lambda result: mutate_argv(result, "--reuid=1000", "--reuid=0"),
        lambda result: remove_argv_token(result, "--groups=27"),
        lambda result: mutate_argv(result, "--groups=27", "--groups=28"),
        lambda result: mutate_argv(result, "--groups=27", "--groups=27,4"),
        lambda result: mutate_argv(result, "--groups=27", "--clear-groups"),
        lambda result: mutate_argv(result, "--groups=27", "--init-groups"),
        lambda result: mutate_argv(result, "--groups=27", "--keep-groups"),
        lambda result: insert_result_argv_tokens_before_delimiter(result, "--bounding-set=-all"),
        lambda result: insert_result_argv_tokens_before_delimiter(result, "--bounding-set=+all"),
        lambda result: insert_result_argv_tokens_before_delimiter(result, "--bounding-set=-cap_sys_admin,+cap_chown"),
        lambda result: insert_result_argv_tokens_before_delimiter(result, "--bounding-set", "-all"),
        lambda result: mutate_argv(result, "--inh-caps=-all", "--inh-caps=+all"),
        lambda result: mutate_argv(result, "--ambient-caps=-all", "--ambient-caps=+all"),
    ],
)
def test_sudo_noninteractive_proof_requires_exact_rc_stdout_stderr_and_argv(monkeypatch, mutate) -> None:
    configure_foreground_tty(monkeypatch)

    def bounded(argv, **_kwargs):
        result = valid_sudo_cleanup_result(argv)
        if argv[-3:] == ["/usr/bin/sudo", "-n", "/usr/bin/true"]:
            mutate(result)
        return result

    monkeypatch.setattr(SUPERVISOR, "run_bounded_command", bounded)
    with pytest.raises(SUPERVISOR.SupervisorError, match="not proven"):
        SUPERVISOR.clear_invoking_user_sudo_timestamp()


def valid_sudo_lifecycle_evidence():
    clear_argv, verify_argv = SUPERVISOR.sudo_timestamp_argvs()
    return {
        "scope": "ALL_UID1000_SUDO_TIMESTAMPS_INVALIDATED_BY_SUDO_-K_OTHER_TERMINALS_MUST_REAUTHENTICATE",
        "bounding_mode": SUPERVISOR.SUDO_CLEANUP_BOUNDING_MODE,
        "pty": {
            "stdin_isatty": True,
            "stdin_device": 1,
            "stdin_inode": 2,
            "stdin_rdev": 3,
            "session_id": 33,
            "process_group": 44,
            "foreground_process_group": 44,
        },
        "membership": copy.deepcopy(SUPERVISOR.SUDO_GROUP_MEMBERSHIP_CONTRACT),
        "identity_contract": copy.deepcopy(SUPERVISOR.SUDO_CLEANUP_IDENTITY_CONTRACT),
        "clear": SUPERVISOR._command_evidence(valid_sudo_cleanup_result(clear_argv)),
        "noninteractive_true_must_fail": SUPERVISOR._command_evidence(
            valid_sudo_cleanup_result(verify_argv), include_prefix=True,
        ),
    }


def valid_unloaded_profile_state():
    return {
        "kernel_exact_counts": {label: 0 for label in SUPERVISOR.LABELS},
        "kernel_exact_lines": {label: [] for label in SUPERVISOR.LABELS},
        "kernel_exact_enforce": {label: False for label in SUPERVISOR.LABELS},
        "aa_status_exact_presence": {label: False for label in SUPERVISOR.LABELS},
        "aa_status_exact_modes": {label: None for label in SUPERVISOR.LABELS},
    }


def mock_profile_state_query(monkeypatch, *, kernel_entries, document=None, stdout=None, returncode=0, stderr=b"", failure=None):
    calls = []
    if stdout is None:
        stdout = json.dumps(document, sort_keys=True).encode("utf-8")

    def bounded(argv, **kwargs):
        calls.append((list(argv), kwargs))
        return {
            "argv": list(argv),
            "returncode": returncode,
            "stdout": stdout,
            "stderr": stderr,
            "failure": failure,
            "start_new_session": True,
        }

    monkeypatch.setattr(SUPERVISOR, "read_kernel_profile_entries", lambda: copy.deepcopy(kernel_entries))
    monkeypatch.setattr(SUPERVISOR, "run_bounded_command", bounded)
    return calls


def test_profile_state_accepts_representative_unloaded_json_and_ignores_process_labels(monkeypatch) -> None:
    label = SUPERVISOR.LABELS[0]
    document = {
        "version": "4.1.0",
        "profiles": {"unrelated-profile": "complain"},
        "processes": {
            "/usr/bin/fixture": [
                {"profile": label, "pid": "123", "status": "enforce"},
            ],
        },
    }
    entries = {name: [] for name in SUPERVISOR.LABELS}
    calls = mock_profile_state_query(monkeypatch, kernel_entries=entries, document=document)
    state = SUPERVISOR.profile_state()
    assert calls[0][0] == ["/usr/sbin/aa-status", "--json"]
    assert len(calls[0][0]) == 2
    assert state["aa_status_exact_presence"] == {name: False for name in SUPERVISOR.LABELS}
    assert state["aa_status_exact_modes"] == {name: None for name in SUPERVISOR.LABELS}
    SUPERVISOR.require_profile_counts(state, 0)


def test_profile_state_accepts_representative_loaded_enforce_json_matching_kernel(monkeypatch) -> None:
    document = {
        "version": "4.1.0",
        "profiles": {
            "unrelated-profile": "enforce",
            **{label: "enforce" for label in SUPERVISOR.LABELS},
        },
        "processes": {},
    }
    entries = {label: [f"{label} (enforce)"] for label in SUPERVISOR.LABELS}
    calls = mock_profile_state_query(monkeypatch, kernel_entries=entries, document=document)
    state = SUPERVISOR.profile_state()
    assert calls[0][0] == list(SUPERVISOR.AA_STATUS_JSON_ARGV)
    assert state["aa_status_exact_modes"] == {label: "enforce" for label in SUPERVISOR.LABELS}
    SUPERVISOR.require_profile_counts(state, 1)


@pytest.mark.parametrize(
    "stdout",
    [
        b"apparmor module is loaded.\n0 profiles are loaded.\n",
        b"not-json\n",
        b"[]",
        b'{"version":"4.1.0","profiles":[],"processes":{}}',
        b'{"version":"4.1.0","profiles":{"fixture":false},"processes":{}}',
    ],
)
def test_profile_state_rejects_human_non_json_or_wrong_json_shapes(monkeypatch, stdout) -> None:
    entries = {label: [] for label in SUPERVISOR.LABELS}
    mock_profile_state_query(monkeypatch, kernel_entries=entries, stdout=stdout)
    with pytest.raises(SUPERVISOR.SupervisorError, match="aa-status"):
        SUPERVISOR.profile_state()


@pytest.mark.parametrize(
    "result_kwargs",
    [
        {"returncode": 1},
        {"stderr": b"unexpected\n"},
        {"failure": "deadline"},
    ],
)
def test_profile_state_rejects_nonclean_command_result(monkeypatch, result_kwargs) -> None:
    entries = {label: [] for label in SUPERVISOR.LABELS}
    document = {"version": "4.1.0", "profiles": {}, "processes": {}}
    mock_profile_state_query(monkeypatch, kernel_entries=entries, document=document, **result_kwargs)
    with pytest.raises(SUPERVISOR.SupervisorError, match="query failed"):
        SUPERVISOR.profile_state()


def test_profile_state_rejects_complain_mode_or_kernel_json_disagreement(monkeypatch) -> None:
    entries = {label: [f"{label} (enforce)"] for label in SUPERVISOR.LABELS}
    complain = {
        "version": "4.1.0",
        "profiles": {label: ("complain" if index == 0 else "enforce") for index, label in enumerate(SUPERVISOR.LABELS)},
        "processes": {},
    }
    mock_profile_state_query(monkeypatch, kernel_entries=entries, document=complain)
    with pytest.raises(SUPERVISOR.SupervisorError, match="mode is not exact enforce"):
        SUPERVISOR.profile_state()

    absent = {"version": "4.1.0", "profiles": {}, "processes": {}}
    mock_profile_state_query(monkeypatch, kernel_entries=entries, document=absent)
    with pytest.raises(SUPERVISOR.SupervisorError, match="disagrees"):
        SUPERVISOR.profile_state()


def make_lifecycle_document(sudo_evidence):
    return SUPERVISOR.lifecycle_document(
        execution_receipt={"status": "fixture"},
        cleanup={"stable_zero_scans": [[], [], []], "post_unload_stable_zero_scans": [[], [], []]},
        profiles_after=valid_unloaded_profile_state(),
        sysctls_before={"fixture": {"value": "1"}},
        sysctls_after={"fixture": {"value": "1"}},
        sudo_clear=sudo_evidence,
    )


def sudo_lifecycle_mutation_cases():
    cases = [
        ("scope", lambda value: value.__setitem__("scope", "wrong")),
        ("bounding_mode", lambda value: value.__setitem__("bounding_mode", "drop_all")),
        ("top_extra", lambda value: value.__setitem__("unexpected", False)),
        ("pty_not_object", lambda value: value.__setitem__("pty", False)),
        ("pty_extra", lambda value: value["pty"].__setitem__("unexpected", 0)),
        ("pty_isatty", lambda value: value["pty"].__setitem__("stdin_isatty", False)),
        ("pty_session_zero", lambda value: value["pty"].__setitem__("session_id", 0)),
        ("pty_process_group_zero", lambda value: value["pty"].__setitem__("process_group", 0)),
        ("pty_not_foreground", lambda value: value["pty"].__setitem__("foreground_process_group", 45)),
        ("membership_record", lambda value: value["membership"].__setitem__("record", "sudo:x:27:")),
        ("membership_owner", lambda value: value["membership"].__setitem__("owner", [1000, 1000])),
        ("membership_extra_member", lambda value: value["membership"].__setitem__("members", ["zrj", "other"])),
        ("identity_group_missing", lambda value: value["identity_contract"].__setitem__("supplementary_groups", [])),
        ("identity_group_changed", lambda value: value["identity_contract"].__setitem__("supplementary_groups", [28])),
        ("identity_group_extra", lambda value: value["identity_contract"].__setitem__("supplementary_groups", [27, 4])),
        ("identity_nnp", lambda value: value["identity_contract"].__setitem__("no_new_privs_option_present", True)),
        ("identity_bounding_mode", lambda value: value["identity_contract"].__setitem__("bounding_mode", "drop_all")),
        ("identity_bounding_tokens", lambda value: value["identity_contract"]["bounding_argv_tokens"].append("--bounding-set=-all")),
        ("identity_bounding_forbidden", lambda value: value["identity_contract"].__setitem__("explicit_bounding_change_forbidden", False)),
        ("identity_inheritable_caps", lambda value: value["identity_contract"].__setitem__("inheritable_capabilities", "+all")),
        ("identity_ambient_caps", lambda value: value["identity_contract"].__setitem__("ambient_capabilities", "+all")),
        ("identity_shell", lambda value: value["identity_contract"].__setitem__("shell", True)),
        ("identity_setsid", lambda value: value["identity_contract"].__setitem__("start_new_session", True)),
    ]
    for key in ("scope", "bounding_mode", "pty", "membership", "identity_contract", "clear", "noninteractive_true_must_fail"):
        cases.append((f"top_missing_{key}", lambda value, key=key: value.pop(key)))
    for key in ("stdin_isatty", "stdin_device", "stdin_inode", "stdin_rdev", "session_id", "process_group", "foreground_process_group"):
        cases.append((f"pty_missing_{key}", lambda value, key=key: value["pty"].pop(key)))
    for key in ("stdin_device", "stdin_inode", "stdin_rdev", "session_id", "process_group", "foreground_process_group"):
        cases.append((f"pty_bool_{key}", lambda value, key=key: value["pty"].__setitem__(key, False)))
    for section in ("clear", "noninteractive_true_must_fail"):
        cases.extend([
            (f"{section}_argv", lambda value, section=section: mutate_argv(value[section], "--reuid=1000", "--reuid=0")),
            (f"{section}_stdout_size", lambda value, section=section: value[section].__setitem__("stdout_size_bytes", True)),
            (f"{section}_stdout_hash", lambda value, section=section: value[section].__setitem__("stdout_sha256", "0" * 64)),
            (f"{section}_stderr_size", lambda value, section=section: value[section].__setitem__("stderr_size_bytes", True)),
            (f"{section}_stderr_hash", lambda value, section=section: value[section].__setitem__("stderr_sha256", "0" * 64)),
            (f"{section}_failure", lambda value, section=section: value[section].__setitem__("failure", "deadline")),
            (f"{section}_setsid", lambda value, section=section: value[section].__setitem__("start_new_session", True)),
            (f"{section}_extra", lambda value, section=section: value[section].__setitem__("unexpected", False)),
        ])
    cases.extend([
        ("clear_rc", lambda value: value["clear"].__setitem__("returncode", 1)),
        ("clear_rc_bool", lambda value: value["clear"].__setitem__("returncode", False)),
        ("verify_rc", lambda value: value["noninteractive_true_must_fail"].__setitem__("returncode", 2)),
        ("verify_rc_bool", lambda value: value["noninteractive_true_must_fail"].__setitem__("returncode", True)),
        ("verify_stderr_prefix", lambda value: value["noninteractive_true_must_fail"].__setitem__("stderr_utf8_prefix", "sudo: a password is required")),
    ])
    for section, keys in (
        ("clear", ("argv", "returncode", "stdout_size_bytes", "stdout_sha256", "stderr_size_bytes", "stderr_sha256", "failure", "start_new_session")),
        ("noninteractive_true_must_fail", ("argv", "returncode", "stdout_size_bytes", "stdout_sha256", "stderr_size_bytes", "stderr_sha256", "failure", "start_new_session", "stderr_utf8_prefix")),
    ):
        for key in keys:
            cases.append((f"{section}_missing_{key}", lambda value, section=section, key=key: value[section].pop(key)))
    return cases


SUDO_LIFECYCLE_MUTATIONS = sudo_lifecycle_mutation_cases()


def test_lifecycle_accepts_only_complete_exact_sudo_evidence() -> None:
    document = make_lifecycle_document(valid_sudo_lifecycle_evidence())
    assert document["status"] == "PASS_V8_PROFILE_PROCESS_SYSCTL_SUDO_LIFECYCLE_CLEANUP"
    assert document["sudo_cleanup_bounding_mode"] == "preserve_host_only"
    assert document["sudo_timestamp"]["bounding_mode"] == "preserve_host_only"


@pytest.mark.parametrize("_case,mutate", SUDO_LIFECYCLE_MUTATIONS, ids=[case for case, _mutate in SUDO_LIFECYCLE_MUTATIONS])
def test_lifecycle_rejects_every_single_field_sudo_evidence_mutation(_case, mutate) -> None:
    evidence = valid_sudo_lifecycle_evidence()
    mutate(evidence)
    with pytest.raises(SUPERVISOR.SupervisorError, match="sudo timestamp cleanup evidence"):
        make_lifecycle_document(evidence)


class FakeStream:
    def __init__(self, descriptor: int) -> None:
        self.descriptor = descriptor
        self.closed = False

    def fileno(self) -> int:
        return self.descriptor

    def close(self) -> None:
        self.closed = True


class FakeProcess:
    def __init__(self) -> None:
        self.pid = 700
        self.stdin = FakeStream(10)
        self.stdout = FakeStream(11)
        self.stderr = FakeStream(12)
        self.returncode = None
        self.killed = False
        self.wait_calls = []

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        if self.returncode is not None:
            return self.returncode
        if timeout == 0.2 and self.returncode is None:
            raise subprocess.TimeoutExpired("fixture", timeout)
        self.returncode = -int(signal.SIGKILL)
        return self.returncode

    def kill(self):
        self.killed = True
        self.returncode = -int(signal.SIGKILL)


def test_supervisor_setup_exception_after_popen_kills_and_reaps_birth_pidfd(monkeypatch) -> None:
    process = FakeProcess()
    sent = []
    closed = []
    monkeypatch.setattr(SUPERVISOR.subprocess, "Popen", lambda *_a, **_k: process)
    monkeypatch.setattr(SUPERVISOR.os, "pidfd_open", lambda _pid, _flags: 88)
    monkeypatch.setattr(SUPERVISOR.os, "close", lambda fd: closed.append(fd))
    monkeypatch.setattr(SUPERVISOR.signal, "pidfd_send_signal", lambda fd, sig: sent.append((fd, sig)))
    monkeypatch.setattr(SUPERVISOR.selectors, "DefaultSelector", lambda: (_ for _ in ()).throw(RuntimeError("selector setup")))
    with pytest.raises(RuntimeError, match="selector setup"):
        SUPERVISOR.run_bounded_command(["fixture"])
    assert any(sig == signal.SIGKILL for _fd, sig in sent)
    assert process.returncode == -int(signal.SIGKILL)
    assert 88 in closed
    assert process.stdin.closed and process.stdout.closed and process.stderr.closed


def test_gate_setup_exception_after_popen_kills_and_reaps_birth_pidfd(monkeypatch) -> None:
    process = FakeProcess()
    closed = []
    monkeypatch.setattr(GATE.subprocess, "Popen", lambda *_a, **_k: process)
    monkeypatch.setattr(GATE.os, "pidfd_open", lambda _pid, _flags: 89)
    monkeypatch.setattr(GATE.os, "close", lambda fd: closed.append(fd))
    monkeypatch.setattr(GATE.os, "set_blocking", lambda _fd, _blocking: None)
    monkeypatch.setattr(GATE, "_proc_labeled_members", lambda: [])

    def terminate(_process, _pidfd):
        process.returncode = -int(signal.SIGTERM)
        return {"stable_zero_scans": [[], [], []]}

    monkeypatch.setattr(GATE, "_terminate_probe_tree", terminate)
    monkeypatch.setattr(GATE.selectors, "DefaultSelector", lambda: (_ for _ in ()).throw(RuntimeError("selector setup")))
    with pytest.raises(RuntimeError, match="selector setup"):
        GATE.run_bounded_guest(["fixture"], b"")
    assert process.returncode == -int(signal.SIGTERM)
    assert 89 in closed
    assert process.stdin.closed and process.stdout.closed and process.stderr.closed


def denial_result(stderr: bytes = b"bwrap: permission denied\n"):
    return {
        "argv": ["gate"],
        "returncode": 1,
        "stdout": b"",
        "stderr": stderr,
        "failure": None,
        "start_new_session": True,
    }


def exact_audit():
    line = 'apparmor="DENIED" operation="mkdir" profile="' + SUPERVISOR.BOOTSTRAP_PROFILE + '" name="/newroot/proc/"'
    return {
        "matching_total": 1,
        "stored_count": 1,
        "storage_overflow": False,
        "expected_proc_mkdir_total": 1,
        "unexpected_total": 0,
        "expected_proc_mkdir_denials": [line],
    }


def test_denial_classification_requires_one_exact_boundary_and_no_internal_no_go() -> None:
    status, _evidence = SUPERVISOR.classify_execution(denial_result(), exact_audit())
    assert status == "PASS_FAIL_CLOSED_APPARMOR_DENIAL_DISCOVERY_CLEANUP_PENDING"
    for stderr in (b'{"status": "NO_GO"}\n', b"R8_STDIO_PROBE_V8_NO_GO\n"):
        status, _ = SUPERVISOR.classify_execution(denial_result(stderr), exact_audit())
        assert status == "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"
    for key, value in (("storage_overflow", True), ("unexpected_total", 1), ("matching_total", 2)):
        audit = exact_audit()
        audit[key] = value
        status, _ = SUPERVISOR.classify_execution(denial_result(), audit)
        assert status == "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"


def test_execution_classification_rejects_bool_aliases_for_rc_and_audit_counts() -> None:
    success = denial_result(stderr=b"")
    success.update({"returncode": False, "stdout": success_frame(valid_success_payload())})
    status, _ = SUPERVISOR.classify_execution(success, exact_audit())
    assert status == "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"
    denial = denial_result()
    denial["returncode"] = True
    status, _ = SUPERVISOR.classify_execution(denial, exact_audit())
    assert status == "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"
    for key, value in (("matching_total", True), ("stored_count", True), ("expected_proc_mkdir_total", True), ("unexpected_total", False)):
        audit = exact_audit()
        audit[key] = value
        status, _ = SUPERVISOR.classify_execution(denial_result(), audit)
        assert status == "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"


def test_audit_capture_counts_all_lines_and_marks_storage_overflow(monkeypatch) -> None:
    line = ('apparmor="DENIED" operation="mkdir" profile="' + SUPERVISOR.BOOTSTRAP_PROFILE
            + '" name="/newroot/proc/" pid=123')
    raw = ((line + "\n") * 20).encode()
    monkeypatch.setattr(SUPERVISOR, "run_bounded_command", lambda *_a, **_k: {"returncode": 0, "failure": None, "stdout": raw, "stderr": b""})
    observed = SUPERVISOR.capture_apparmor_denials(1.0, 2.0)
    assert observed["matching_total"] == 20
    assert observed["stored_count"] == 16
    assert observed["storage_overflow"] is True
    assert observed["expected_proc_mkdir_total"] == 20


def test_profile_state_requires_exact_enforce_mode() -> None:
    base = {
        "kernel_exact_counts": {label: 1 for label in SUPERVISOR.LABELS},
        "kernel_exact_lines": {label: [f"{label} (enforce)"] for label in SUPERVISOR.LABELS},
        "kernel_exact_enforce": {label: True for label in SUPERVISOR.LABELS},
        "aa_status_exact_presence": {label: True for label in SUPERVISOR.LABELS},
        "aa_status_exact_modes": {label: "enforce" for label in SUPERVISOR.LABELS},
    }
    SUPERVISOR.require_profile_counts(base, 1)
    changed = copy.deepcopy(base)
    label = SUPERVISOR.LABELS[0]
    changed["kernel_exact_lines"][label] = [f"{label} (complain)"]
    changed["kernel_exact_enforce"][label] = False
    with pytest.raises(SUPERVISOR.SupervisorError, match="enforce"):
        SUPERVISOR.require_profile_counts(changed, 1)


def test_loaded_profile_state_is_sampled_once_before_probe_handoff() -> None:
    tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_run_once_guarded")
    assignments = [
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "profiles_loaded" for target in node.targets)
        and isinstance(node.value, ast.Call)
        and isinstance(node.value.func, ast.Name)
        and node.value.func.id == "profile_state"
    ]
    assert len(assignments) == 1


def test_sudo_admission_cleanup_is_flagged_before_call_and_precedes_sysctl_profile_start_parser_and_probe() -> None:
    tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_run_once_guarded")
    source = ast.get_source_segment(SUPERVISOR_PATH.read_text(encoding="utf-8"), function)
    assert source is not None
    attempted = source.index("termination_guard.preflight_sudo_cleanup_attempted = True")
    admission = source.index("sudo_admission = clear_invoking_user_sudo_timestamp()")
    sysctls = source.index("sysctls_before = read_sysctls()")
    profiles = source.index("profiles_before = profile_state()")
    start = source.index("start_record = write_json_new(START_RECEIPT, start_document)")
    parser = source.index('run_checked(["/usr/sbin/apparmor_parser", "-Q"')
    probe = source.index("gate_result = run_bounded_command(")
    final_cleanup = source.rindex("sudo_clear = clear_invoking_user_sudo_timestamp()")
    assert attempted < admission < sysctls < profiles < start < parser < probe < final_cleanup
    assert source.count("clear_invoking_user_sudo_timestamp()") == 2


def test_gate_and_probe_identity_keep_empty_groups_zero_caps_and_nnp_while_group27_is_cleanup_only() -> None:
    handoff = SUPERVISOR.gate_handoff_argv()
    assert "--clear-groups" in handoff and "--no-new-privs" in handoff
    assert "--groups=27" not in handoff
    assert all(option not in handoff for option in ("--init-groups", "--keep-groups"))
    assert handoff[handoff.index("--bounding-set=-all"):handoff.index("--") + 1] == [
        "--bounding-set=-all", "--inh-caps=-all", "--ambient-caps=-all", "--no-new-privs", "--",
    ]
    policy, _schema = policy_and_schema()
    assert policy["fixed_commands"]["gate_handoff_argv"] == handoff
    clear_argv, verify_argv = SUPERVISOR.sudo_timestamp_argvs()
    assert "--groups=27" in clear_argv and "--groups=27" in verify_argv
    assert "--no-new-privs" not in clear_argv and "--no-new-privs" not in verify_argv
    assert all(
        token != "--bounding-set" and not token.startswith("--bounding-set=")
        for argv in (clear_argv, verify_argv)
        for token in argv
    )
    assert SUPERVISOR.SUDO_CLEANUP_IDENTITY_CONTRACT == GATE.SUDO_CLEANUP_IDENTITY_CONTRACT == {
        "reuid": 1000,
        "regid": 1000,
        "supplementary_groups": [27],
        "groups_mode": "EXACT_NUMERIC_GROUP_LIST_ONLY",
        "bounding_mode": "preserve_host_only",
        "bounding_argv_tokens": [],
        "explicit_bounding_change_forbidden": True,
        "inheritable_capabilities": "-all",
        "ambient_capabilities": "-all",
        "no_new_privs_option_present": False,
        "shell": False,
        "start_new_session": False,
        "forbidden_group_options": ["--clear-groups", "--init-groups", "--keep-groups"],
    }
    assert policy["identity_contract"]["host_groups"] == []
    assert policy["identity_contract"]["guest_groups"] == []
    assert policy["identity_contract"]["host_no_new_privs"] == 1
    assert policy["identity_contract"]["guest_no_new_privs"] == 1
    assert set(policy["identity_contract"]["host_capability_sets"].values()) == {0}
    assert set(policy["identity_contract"]["guest_capability_sets"].values()) == {0}


def test_ast_keeps_bounding_drop_and_nnp_in_gate_handoff_but_forbids_bounding_in_cleanup_builders() -> None:
    for path in (GATE_PATH, SUPERVISOR_PATH):
        source_text = path.read_text(encoding="utf-8")
        tree = ast.parse(source_text)
        cleanup = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "sudo_timestamp_argvs")
        cleanup_source = ast.get_source_segment(source_text, cleanup)
        assert cleanup_source is not None
        assert "--bounding-set" not in cleanup_source
        assert "--no-new-privs" not in cleanup_source
        assert "--groups=27" in cleanup_source
        assert "--inh-caps=-all" in cleanup_source
        assert "--ambient-caps=-all" in cleanup_source
    supervisor_source = SUPERVISOR_PATH.read_text(encoding="utf-8")
    supervisor_tree = ast.parse(supervisor_source)
    handoff = next(node for node in supervisor_tree.body if isinstance(node, ast.FunctionDef) and node.name == "gate_handoff_argv")
    handoff_source = ast.get_source_segment(supervisor_source, handoff)
    assert handoff_source is not None
    assert "--clear-groups" in handoff_source
    assert "--bounding-set=-all" in handoff_source
    assert "--inh-caps=-all" in handoff_source
    assert "--ambient-caps=-all" in handoff_source
    assert "--no-new-privs" in handoff_source


def test_termination_guard_records_only_then_raises_at_safe_checkpoint() -> None:
    guard = SUPERVISOR.TerminationGuard()
    guard._handle(int(signal.SIGTERM), None)
    assert guard.received == [int(signal.SIGTERM)]
    with pytest.raises(SUPERVISOR.SupervisorTermination):
        guard.checkpoint()
    guard.begin_cleanup()
    guard._handle(int(signal.SIGTSTP), None)
    guard.checkpoint()


def test_recover_only_source_has_no_probe_or_admission_call_and_unload_is_zero_gated() -> None:
    tree = ast.parse(SUPERVISOR_PATH.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "recover_only")
    called = {
        node.func.id
        for node in ast.walk(function)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert "create_root_admission_pipe" not in called
    assert "gate_handoff_argv" not in called
    source = ast.get_source_segment(SUPERVISOR_PATH.read_text(encoding="utf-8"), function)
    assert source is not None
    assert "label_zero_proven" in source
    assert "profile_unload_skipped_until_label_zero_is_proven" in source


def test_self_check_code_paths_never_offer_parser_bwrap_sudo_or_run() -> None:
    gate_source = GATE_PATH.read_text(encoding="utf-8")
    supervisor_source = SUPERVISOR_PATH.read_text(encoding="utf-8")
    assert 'choices=("self-check", "internal-run")' in gate_source
    assert "repository_static_review()" in supervisor_source
    assert "PASS_V8_SUPERVISOR_STATIC_ONLY_EXECUTION_NOT_PERFORMED" in supervisor_source
    assert GATE.verify_static()["execution_performed"] is False
