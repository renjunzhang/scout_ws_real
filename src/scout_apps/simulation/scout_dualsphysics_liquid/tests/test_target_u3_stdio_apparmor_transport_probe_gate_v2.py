"""Static-only tests for the harmless U3 stdio/AppArmor probe v2 draft."""

from __future__ import annotations

import ast
import importlib.util
import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts/r8_liquid_target_u3_stdio_apparmor_transport_probe_gate_v2.py"
POLICY = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v2.json"
SCHEMA = PACKAGE / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v2.json"
PROFILE = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v2.profile"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_stdio_apparmor_probe_v2_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_static_review_passes_but_every_execution_authority_remains_false() -> None:
    gate = load_gate()
    review = gate.verify_review_artifacts()
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    assert review["status"] == "STATIC_NO_GO_V2_MOUNT_RULES_UNOBSERVED_EXECUTION_FORBIDDEN"
    assert review["static_only"] is True
    assert review["helper_executed"] is False
    assert review["subprocess_started"] is False
    assert review["profile_parser_invoked"] is False
    assert review["profile_loaded"] is False
    assert review["namespace_attempted"] is False
    assert review["mount_attempted"] is False
    assert review["host_state_changed"] is False
    assert policy["allowed_gate_commands"] == ["self-check"]
    assert policy["profile_parse_load_select_authorized"] is False
    assert policy["harmless_probe_execution_authorized"] is False
    assert policy["production_runtime_authorized"] is False


def test_policy_matches_gate_and_schema_is_valid_closed_draft_2020_12() -> None:
    gate = load_gate()
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    assert policy == gate.expected_policy()
    assert schema["additionalProperties"] is False
    assert set(schema["required"]) == set(policy)
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(policy)
    gate.validate_schema_instance(policy, schema)


def test_versioned_frame_round_trip_is_exact_and_bounded() -> None:
    gate = load_gate()
    frame = gate.build_stdin_frame()
    observed = gate.parse_stdin_frame(frame)
    assert frame.startswith(gate.STDIN_MAGIC)
    assert len(gate.STDIN_MAGIC) == 16
    assert len(gate.SUCCESS_MAGIC) == 16
    assert observed["frame_size_bytes"] == 11_574
    assert observed["frame_sha256"] == "b72884d08a6f579e63641991a56bc3f06596e6bac4f1aa768d455bfc21fc9844"
    assert observed["helper"] == {
        "size_bytes": 9_626,
        "sha256": "ecd8cac26d8e988ecf818181fbd856299a3b05e34e999ef16c094f5c48fbfd06",
    }
    assert list(observed["inputs"]) == list(gate.INPUT_CONTRACT)
    assert len(frame) < 65_536


@pytest.mark.parametrize("mutation", ["magic", "helper", "first_size", "trailing"])
def test_frame_mutations_fail_closed(mutation: str) -> None:
    gate = load_gate()
    frame = bytearray(gate.build_stdin_frame())
    if mutation == "magic":
        frame[0] ^= 1
    elif mutation == "helper":
        frame[16 + 4 + 32] ^= 1
    elif mutation == "first_size":
        frame[16 + 4 + 32 + gate.HELPER_SIZE_BYTES] ^= 1
    else:
        frame.extend(b"x")
    with pytest.raises(gate.GateError):
        gate.parse_stdin_frame(bytes(frame))


def test_loader_helper_bytes_are_reviewed_but_never_executed_by_tests_or_gate() -> None:
    gate = load_gate()
    observed = gate.verify_loader_and_helper()
    assert observed["loader"] == {
        "sha256": "541d9bd1e9bcb50f278c7ecedd351620582d5f89b71969538f76564e54e7d194",
        "size_bytes": 560,
    }
    assert observed["helper"] == {
        "sha256": "ecd8cac26d8e988ecf818181fbd856299a3b05e34e999ef16c094f5c48fbfd06",
        "size_bytes": 9_626,
    }
    ast.parse(gate.LOADER_SOURCE)
    ast.parse(gate.HELPER_SOURCE)
    assert "rPx" not in gate.LOADER_SOURCE
    assert "rPx" not in gate.HELPER_SOURCE
    assert "GenCase" not in gate.HELPER_SOURCE
    assert "solver" not in gate.HELPER_SOURCE


def test_fd0_is_replaced_by_inheritable_eof_pipe_and_runtime_inherits_it() -> None:
    gate = load_gate()
    helper = gate.HELPER_SOURCE
    for required in (
        'if STREAM.read(1) != b"":',
        "STREAM.close()",
        "os.close(0)",
        "read_end, write_end = os.pipe()",
        "os.close(write_end)",
        "os.dup2(read_end, 0, inheritable=True)",
        "os.set_inheritable(0, True)",
        "live_fds({0, 1, 2})",
        "stdin=None",
        "stdout=subprocess.PIPE",
        "stderr=subprocess.PIPE",
        "close_fds=True",
    ):
        assert required in helper
    assert "stdin=subprocess.PIPE" not in helper
    assert "subprocess.DEVNULL" not in helper
    transport = gate.expected_policy()["stdio_transport"]
    assert transport["host_stdin_consumed_to_strict_eof"] is True
    assert transport["fd0_replaced_by_guest_internal_eof_pipe"] is True
    assert transport["runtime_inherits_only_eof_pipe_on_fd0"] is True


def test_work_mount_is_exact_64mib_tmpfs_and_has_no_guest_dev() -> None:
    gate = load_gate()
    helper = gate.HELPER_SOURCE
    assert 'os.statvfs("/work")' in helper
    assert "filesystem.f_blocks * filesystem.f_frsize" in helper
    assert "total_bytes != 67_108_864" in helper
    assert 'open("/proc/self/mountinfo"' in helper
    assert 'matches[0]["filesystem_type"] != "tmpfs"' in helper
    bwrap = gate.bwrap_argv_template()
    assert bwrap.count("--size") == 1
    index = bwrap.index("--size")
    assert bwrap[index : index + 4] == ["--size", "67108864", "--tmpfs", "/work"]
    assert "--dev" not in bwrap
    assert gate.expected_policy()["isolation"]["guest_dev"] == "absent"


def test_profile_has_lowercase_rpx_only_zero_mount_rules_and_narrow_signals() -> None:
    gate = load_gate()
    text = PROFILE.read_text(encoding="utf-8")
    effective = gate.effective_profile_lines(text)
    profile = gate.verify_profile()
    transition = f"/usr/bin/sleep rpx -> {gate.RUNTIME_PROFILE},"
    assert transition in effective
    assert all("rPx" not in line for line in effective)
    assert profile["effective_mount_rules"] == []
    for prefix in ("mount ", "remount ", "pivot_root ", "umount "):
        assert not any(line.startswith(prefix) for line in effective)
    assert (
        f"signal (send) set=(term,kill,exists) peer={gate.RUNTIME_PROFILE},"
        in effective
    )
    assert (
        f"signal (receive) set=(term,kill,exists) peer={gate.BOOTSTRAP_PROFILE},"
        in effective
    )


def test_bwrap_and_outer_argv_are_stdio_only_and_drop_privilege_explicitly() -> None:
    gate = load_gate()
    bwrap = gate.bwrap_argv_template()
    outer = gate.outer_argv_template()
    for forbidden in (
        "--file",
        "--bind-fd",
        "--ro-bind-fd",
        "--bind",
        "--dev-bind",
        "--dev",
        "--share-net",
    ):
        assert forbidden not in bwrap
    assert bwrap.count("--ro-bind") == 1
    assert bwrap[bwrap.index("--ro-bind") + 1 : bwrap.index("--ro-bind") + 3] == [
        "/usr",
        "/usr",
    ]
    assert bwrap[bwrap.index("--cap-drop") + 1] == "ALL"
    for required in (
        "--unshare-net",
        "--clear-groups",
        "--no-new-privs",
        "--bounding-set=-all",
        "--inh-caps=-all",
        "--ambient-caps=-all",
    ):
        assert required in outer
    assert outer.count("<PINNED_LOADER_SOURCE>") == 1
    assert gate.verify_argv_contract()["host_writable_mounts"] == []


def test_static_gate_ast_has_no_process_write_mount_or_profile_load_surface() -> None:
    tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
    imported_modules = {
        alias.name.split(".", 1)[0]
        for node in tree.body
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    assert "subprocess" not in imported_modules
    assert "signal" not in imported_modules
    forbidden_calls = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name):
            qualified = f"{node.func.value.id}.{node.func.attr}"
            if qualified in {
                "os.mkdir",
                "os.makedirs",
                "os.write",
                "os.pipe",
                "os.execv",
                "os.execve",
                "os.system",
                "subprocess.run",
                "subprocess.Popen",
            }:
                forbidden_calls.add(qualified)
    assert forbidden_calls == set()
    source = SCRIPT.read_text(encoding="utf-8")
    assert 'choices=("self-check",)' in source
    assert "apparmor_parser\", \"-r\"" not in source
    assert "apparmor_parser\", \"-R\"" not in source


def test_frozen_identity_is_new_and_static_check_creates_nothing() -> None:
    gate = load_gate()
    paths = (
        gate.ATTEMPT_ROOT,
        gate.EXECUTION_RECEIPT,
        gate.LIFECYCLE_RECEIPT,
        gate.LIFECYCLE_FAILURE_RECEIPT,
        gate.SNAPSHOT_ROOT,
    )
    before = {path: path.exists() for path in paths}
    assert not any(before.values())
    observed = gate.require_fresh_identity()
    after = {path: path.exists() for path in paths}
    assert after == before
    assert not any(observed.values())
