#!/usr/bin/env python3
"""Create-new S5A0 remediation supervisor v2 (static until exact authorization).

The future run path binds an already-open, O_NOFOLLOW-verified source fd with
``--ro-bind-data`` and guest mode 0400.  No real bag or bwrap is touched by
``self-check``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence, TextIO

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v2 as reader_v2
import r8_liquid_s5a0_primary_one_shot_supervisor_v1 as base
import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate_v1
import r8_liquid_s5a0_selected_bag_intake_gate_v2 as gate_v2


sys.dont_write_bytecode = True
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a0_primary_supervisor_policy_v2.json"
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_s5a0_primary_supervisor_execution_receipt_v2.json"
INNER_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_s5a0_selected_bag_receipt_v2.json"
EXECUTION_ID = "s5a0_primary_bsmooth_b01_20260811T192108Z_v2"
SOURCE_FD = 9
_ACTIVE_SOURCE_FD: int | None = None
_BASE_BUILD_RECEIPT = base.build_execution_receipt


def _fail(code: str, phase: str, message: str) -> None:
    raise base.SupervisorError(code, phase, message)


def validate_policy(policy: Mapping[str, Any]) -> None:
    required = {"schema_version", "document_type", "execution_id", "status", "selection", "inputs", "tools", "paths", "limits", "contract", "expected_evidence_inventory"}
    if set(policy) != required:
        _fail("POLICY_KEYS_INVALID", "static", "v2 policy root is not closed")
    if policy["schema_version"] != "smpcc-r8-liquid-s5a0-primary-supervisor-policy-v2" or policy["execution_id"] != EXECUTION_ID:
        _fail("POLICY_IDENTITY_INVALID", "static", "v2 policy identity differs")
    selection = policy["selection"]
    expected = "/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"
    if selection.get("absolute_path") != expected or selection.get("selected_role") != "PRIMARY" or selection.get("selected_count") != 1 or selection.get("optional_pair_authorized") is not False:
        _fail("SELECTION_INVALID", "static", "exact primary-only selection differs")
    if not gate_v1._is_lower_hex(selection.get("expected_sha256"), 64):
        _fail("SELECTION_HASH_INVALID", "static", "source hash is invalid")
    if os.path.join(selection["source_root"], selection["relative_path"]) != expected:
        _fail("SELECTION_PATH_INVALID", "static", "source path binding differs")
    contract = policy["contract"]
    for name in (
        "one_shot", "exact_path_only", "glob_forbidden", "dedicated_evidence_root_only",
        "audit_root_bind_forbidden", "source_fd_o_nofollow", "source_fd_pread_hash",
        "source_ro_bind_data", "sandbox_source_mode_0400", "network_unshared",
        "proc_mount_forbidden", "dev_mount_forbidden", "inner_receipt_required",
        "host_closed_schema_validation", "final_receipt_o_excl",
        "atomic_noreplace_publish", "failure_preserves_partial",
    ):
        if contract.get(name) is not True:
            _fail("CONTRACT_CONTROL_DISABLED", "static", name)
    for name in ("source_bag_executed", "ros_started", "gpu_exposed", "solver_started", "sudo_used"):
        if contract.get(name) is not False:
            _fail("FORBIDDEN_ACTION_ENABLED", "static", name)
    if contract.get("source_fd_number") != SOURCE_FD or contract.get("final_receipt_mode") != "0600":
        _fail("FD_OR_MODE_INVALID", "static", "source fd/final mode differs")
    paths = policy["paths"]
    audit = Path(paths["audit_root"])
    targets = [Path(paths[name]) for name in ("partial_evidence_root", "final_evidence_root", "final_receipt_path", "failure_receipt_path")]
    if any(path.parent != audit for path in targets) or len(set(targets)) != 4:
        _fail("AUDIT_TARGET_INVALID", "static", "four create-new audit targets differ or alias")
    if sorted(policy["expected_evidence_inventory"]) != sorted([
        paths["reservation_name"], paths["inner_receipt_name"], paths["stdout_name"], paths["stderr_name"], paths["rc_name"],
    ]):
        _fail("EVIDENCE_INVENTORY_INVALID", "static", "evidence inventory differs")
    expected_inputs = {"base_policy", "reader_v1", "reader_v2", "gate_v1", "gate_v2", "worker_v4", "inner_schema_v2", "base_supervisor_v1", "outer_schema_v2"}
    if set(policy["inputs"]) != expected_inputs:
        _fail("INPUT_SET_INVALID", "static", "parent hash set differs")
    for item in list(policy["inputs"].values()) + list(policy["tools"]):
        gate_v1._absolute_normalized(item["path"], "input/tool path")
        if not gate_v1._is_lower_hex(item["sha256"], 64):
            _fail("STATIC_HASH_INVALID", "static", item["path"])


def load_policy(path: Path = POLICY_PATH) -> tuple[dict[str, Any], str]:
    value, raw = base._read_json(path, base.MAX_STATIC_BYTES, "supervisor v2 policy")
    validate_policy(value)
    return value, hashlib.sha256(raw).hexdigest()


def load_schema(path: Path = SCHEMA_PATH) -> tuple[dict[str, Any], str]:
    value, raw = base._read_json(path, base.MAX_STATIC_BYTES, "execution receipt schema v2")
    gate_v1.assert_deep_closed(value)
    Draft202012Validator.check_schema(value)
    return value, hashlib.sha256(raw).hexdigest()


def load_inner_schema() -> dict[str, Any]:
    value, _ = base._read_json(INNER_SCHEMA_PATH, base.MAX_STATIC_BYTES, "inner receipt schema v2")
    gate_v1.assert_deep_closed(value)
    Draft202012Validator.check_schema(value)
    return value


def verify_static_inputs(policy: Mapping[str, Any]) -> list[dict[str, Any]]:
    for name, item in policy["inputs"].items():
        observed = base._sha256_file(Path(item["path"]), base.MAX_STATIC_BYTES)
        if observed != item["sha256"]:
            _fail("STATIC_INPUT_HASH_DRIFT", "preflight", name)
    witnesses = []
    for item in policy["tools"]:
        observed = base._sha256_file(Path(item["path"]))
        if observed != item["sha256"]:
            _fail("TOOL_HASH_DRIFT", "preflight", item["name"])
        witnesses.append({"name": item["name"], "path": item["path"], "expected_sha256": item["sha256"], "observed_sha256": observed, "matched": True})
    return witnesses


def _normalised_argv_sha256(argv: Sequence[str]) -> str:
    normalized = list(argv)
    normalized[normalized.index("--expected-argv-sha256") + 1] = "0" * 64
    return gate_v1.canonical_sha256(normalized)


def build_bwrap_argv(policy: Mapping[str, Any]) -> tuple[list[str], str]:
    validate_policy(policy)
    source_fd = _ACTIVE_SOURCE_FD if _ACTIVE_SOURCE_FD is not None else SOURCE_FD
    inputs, paths, limits = policy["inputs"], policy["paths"], policy["limits"]
    argv = [
        "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-all", "--unshare-net", "--clearenv", "--cap-drop", "ALL",
        "--ro-bind", "/usr", "/usr", "--ro-bind", "/lib", "/lib", "--ro-bind", "/lib64", "/lib64",
        "--tmpfs", "/tmp", "--dir", "/app",
        "--ro-bind", inputs["reader_v1"]["path"], "/app/r8_liquid_ros1_bag_v2_reader_v1.py",
        "--ro-bind", inputs["reader_v2"]["path"], "/app/r8_liquid_ros1_bag_v2_reader_v2.py",
        "--ro-bind", inputs["gate_v1"]["path"], "/app/r8_liquid_s5a0_selected_bag_intake_gate_v1.py",
        "--ro-bind", inputs["gate_v2"]["path"], "/app/r8_liquid_s5a0_selected_bag_intake_gate_v2.py",
        "--ro-bind", inputs["worker_v4"]["path"], "/app/worker.py",
        "--dir", "/policy", "--ro-bind", inputs["base_policy"]["path"], "/policy/base.json",
        "--dir", "/schema", "--ro-bind", inputs["inner_schema_v2"]["path"], "/schema/inner.json",
        "--dir", "/selected", "--perms", "0400", "--ro-bind-data", str(source_fd), "/selected/capture.bag",
        "--dir", "/evidence", "--bind", paths["partial_evidence_root"], "/evidence",
        "--setenv", "PATH", "/usr/bin", "--setenv", "HOME", "/nonexistent",
        "--setenv", "PYTHONDONTWRITEBYTECODE", "1", "--setenv", "PYTHONHASHSEED", "0", "--chdir", "/app", "--",
        "/usr/bin/timeout", "--foreground", "--signal=KILL", f"--kill-after={limits['kill_after_seconds']}s", f"{limits['timeout_seconds']}s",
        "/usr/bin/prlimit", f"--rss={limits['maximum_rss_bytes']}:{limits['maximum_rss_bytes']}",
        f"--as={limits['maximum_address_space_bytes']}:{limits['maximum_address_space_bytes']}",
        f"--fsize={max(limits['maximum_stdout_bytes'], limits['maximum_stderr_bytes'])}:{max(limits['maximum_stdout_bytes'], limits['maximum_stderr_bytes'])}",
        f"--nofile={limits['nofile_limit']}:{limits['nofile_limit']}", f"--nproc={limits['nproc_limit']}:{limits['nproc_limit']}",
        "/usr/bin/python3", "-I", "-B", "/app/worker.py", "sandbox-inspect",
        "--policy", "/policy/base.json", "--schema", "/schema/inner.json", "--source", "/selected/capture.bag",
        "--receipt", "/evidence/selected_bag_inner_receipt_v2.json", "--final-evidence-root", paths["final_evidence_root"],
        "--expected-argv-sha256", "0" * 64,
    ]
    digest = _normalised_argv_sha256(argv)
    argv[argv.index("--expected-argv-sha256") + 1] = digest
    forbidden = [policy["selection"]["absolute_path"], policy["selection"]["forbidden_optional_path"], "--proc", "--dev", "sudo", "nvidia-smi"]
    if any(token in argv for token in forbidden) or paths["audit_root"] in argv:
        _fail("BWRAP_EXPOSURE_INVALID", "static", "forbidden source/path/action exposed")
    return argv, digest


def _open_frozen_source(policy: Mapping[str, Any]) -> int:
    try:
        os.fstat(SOURCE_FD)
    except OSError:
        pass
    else:
        _fail("SOURCE_FD_COLLISION", "preflight", f"fd {SOURCE_FD} is already open")
    source = policy["selection"]
    descriptor = os.open(source["absolute_path"], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    keep_descriptor = False
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or stat.S_IMODE(before.st_mode) != int(source["expected_mode"], 8) or before.st_size != source["expected_size_bytes"]:
            _fail("SOURCE_FD_IDENTITY_INVALID", "preflight", "source fd identity differs")
        digest = hashlib.sha256()
        offset = 0
        while offset < before.st_size:
            block = os.pread(descriptor, min(1024 * 1024, before.st_size - offset), offset)
            if not block:
                _fail("SOURCE_FD_SHORT_READ", "preflight", "source pread ended early")
            digest.update(block)
            offset += len(block)
        stable = lambda value: (
            value.st_mode, value.st_uid, value.st_gid, value.st_dev, value.st_ino,
            value.st_nlink, value.st_size, value.st_mtime_ns, value.st_ctime_ns,
        )
        if digest.hexdigest() != source["expected_sha256"] or stable(os.fstat(descriptor)) != stable(before):
            _fail("SOURCE_FD_HASH_OR_IDENTITY_DRIFT", "preflight", "source fd changed or hash differs")
        if descriptor == SOURCE_FD:
            os.set_inheritable(descriptor, False)
            keep_descriptor = True
        else:
            os.dup2(descriptor, SOURCE_FD, inheritable=False)
    finally:
        if not keep_descriptor:
            os.close(descriptor)
    return SOURCE_FD


def _fd_runner(argv: Sequence[str], stdout: TextIO, stderr: TextIO, timeout: int) -> int:
    if _ACTIVE_SOURCE_FD != SOURCE_FD:
        _fail("SOURCE_FD_NOT_ACTIVE", "worker", "frozen source fd is absent")
    completed = subprocess.run(argv, stdout=stdout, stderr=stderr, check=False, timeout=timeout, pass_fds=(SOURCE_FD,))
    return completed.returncode


def build_execution_receipt(*args: Any, **kwargs: Any) -> dict[str, Any]:
    receipt = _BASE_BUILD_RECEIPT(*args, **kwargs)
    receipt["schema_version"] = "smpcc-r8-liquid-s5a0-primary-supervisor-execution-receipt-v2"
    receipt["document_type"] = "SMPCC_R8_LIQUID_S5A0_PRIMARY_SUPERVISOR_EXECUTION_RECEIPT_V2"
    return receipt


def _install_base_revision() -> None:
    base.POLICY_PATH, base.SCHEMA_PATH, base.EXECUTION_ID = POLICY_PATH, SCHEMA_PATH, EXECUTION_ID
    base.validate_policy, base.load_policy, base.load_schema = validate_policy, load_policy, load_schema
    base.verify_static_inputs, base.build_bwrap_argv = verify_static_inputs, build_bwrap_argv
    base.build_execution_receipt = build_execution_receipt
    base.bag_reader = reader_v2
    base.gate_v1.bag_reader = reader_v2
    base.gate_v1.load_schema = load_inner_schema


def run_one_shot() -> dict[str, Any]:
    global _ACTIVE_SOURCE_FD
    policy, _ = load_policy()
    _ACTIVE_SOURCE_FD = _open_frozen_source(policy)
    _install_base_revision()
    try:
        return base.run_one_shot(POLICY_PATH, runner=_fd_runner)
    finally:
        os.close(_ACTIVE_SOURCE_FD)
        _ACTIVE_SOURCE_FD = None


def self_check() -> dict[str, Any]:
    policy, policy_sha = load_policy()
    schema, schema_sha = load_schema()
    load_inner_schema()
    tools = verify_static_inputs(policy)
    argv, argv_sha = build_bwrap_argv(policy)
    delimiter = argv.index("--")
    if any(token in argv[:delimiter] for token in ("--proc", "--dev")) or argv.count("--ro-bind-data") != 1:
        _fail("SANDBOX_STATIC_INVALID", "static", "proc/dev or fd bind cardinality differs")
    if [argv[argv.index("--perms") + 1], argv[argv.index("--ro-bind-data") + 1]] != ["0400", str(SOURCE_FD)]:
        _fail("SOURCE_FD_STATIC_INVALID", "static", "sandbox fd/mode differs")
    return {
        "status": "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_V2_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID, "policy_sha256": policy_sha, "schema_sha256": schema_sha,
        "argv_sha256": argv_sha, "tool_count": len(tools), "source_fd_number": SOURCE_FD,
        "source_guest_mode": "0400", "real_bag_read": False, "bwrap_executed": False,
        "proc_mounted": False, "dev_mounted": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("self-check")
    run = commands.add_parser("run")
    run.add_argument("--authorize-execution-id", required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            result = self_check()
        else:
            if args.authorize_execution_id != EXECUTION_ID:
                _fail("EXECUTION_NOT_AUTHORIZED", "authorization", "exact execution id differs")
            result = run_one_shot()
    except base.SupervisorError as exc:
        print(json.dumps({"status": base.FAIL_STATUS, "code": exc.code, "phase": exc.phase, "error": exc.message}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0 if args.command == "self-check" or result["status"] == base.PASS_STATUS else 2


if __name__ == "__main__":
    raise SystemExit(main())
