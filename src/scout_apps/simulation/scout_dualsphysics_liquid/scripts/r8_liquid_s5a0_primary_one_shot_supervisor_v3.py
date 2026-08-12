#!/usr/bin/env python3
"""Create-new S5A0 supervisor v3 for an immutable private-tmpfs bag copy.

Revision v2 correctly froze the host source through an ``O_NOFOLLOW`` file
descriptor, but bubblewrap's ``--ro-bind-data`` exposes an anonymous file with
``st_nlink == 0``.  The frozen ROS1 reader intentionally requires a regular
single-link input.  This revision changes only the sandbox materialization:
``--file`` copies the verified descriptor to a private tmpfs with mode 0400,
then ``--remount-ro`` makes that tmpfs immutable before the worker starts.

The failed v2 execution and its evidence remain immutable parents.  Static
``self-check`` never opens the real bag and never executes bubblewrap.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v2 as reader_v2
import r8_liquid_s5a0_primary_one_shot_supervisor_v2 as v2


sys.dont_write_bytecode = True
PACKAGE_ROOT = MODULE_DIR.parent
POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a0_primary_supervisor_policy_v3.json"
SCHEMA_PATH = v2.SCHEMA_PATH
INNER_SCHEMA_PATH = v2.INNER_SCHEMA_PATH
EXECUTION_ID = "s5a0_primary_bsmooth_b01_20260811T193602Z_v3"
SOURCE_FD = v2.SOURCE_FD


def _fail(code: str, phase: str, message: str) -> None:
    raise v2.base.SupervisorError(code, phase, message)


def validate_policy(policy: Mapping[str, Any]) -> None:
    required = {
        "schema_version", "document_type", "execution_id", "status",
        "selection", "inputs", "tools", "paths", "limits", "contract",
        "expected_evidence_inventory",
    }
    if set(policy) != required:
        _fail("POLICY_KEYS_INVALID", "static", "v3 policy root is not closed")
    if (
        policy["schema_version"] != "smpcc-r8-liquid-s5a0-primary-supervisor-policy-v3"
        or policy["document_type"] != "SMPCC_R8_LIQUID_S5A0_PRIMARY_SUPERVISOR_POLICY_V3"
        or policy["execution_id"] != EXECUTION_ID
    ):
        _fail("POLICY_IDENTITY_INVALID", "static", "v3 policy identity differs")

    selection = policy["selection"]
    expected = "/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"
    if (
        selection.get("absolute_path") != expected
        or selection.get("selected_role") != "PRIMARY"
        or selection.get("selected_count") != 1
        or selection.get("optional_pair_authorized") is not False
        or os.path.join(selection.get("source_root", ""), selection.get("relative_path", "")) != expected
        or selection.get("expected_mode") != "0755"
        or selection.get("expected_size_bytes") != 13996902
        or selection.get("expected_sha256") != "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07"
    ):
        _fail("SELECTION_INVALID", "static", "exact primary-only source identity differs")

    expected_inputs = {
        "base_policy", "reader_v1", "reader_v2", "gate_v1", "gate_v2",
        "worker_v4", "worker_v5", "inner_schema_v2", "base_supervisor_v1",
        "outer_schema_v2", "supervisor_v3", "prior_v2_failure_receipt",
        "prior_v2_worker_stderr",
    }
    if set(policy["inputs"]) != expected_inputs:
        _fail("INPUT_SET_INVALID", "static", "v3 parent/input set differs")
    for item in list(policy["inputs"].values()) + list(policy["tools"]):
        v2.gate_v1._absolute_normalized(item["path"], "input/tool path")
        if not v2.gate_v1._is_lower_hex(item.get("sha256"), 64):
            _fail("STATIC_HASH_INVALID", "static", item["path"])

    contract = policy["contract"]
    true_flags = (
        "one_shot", "exact_path_only", "glob_forbidden",
        "dedicated_evidence_root_only", "audit_root_bind_forbidden",
        "source_fd_o_nofollow", "source_fd_pread_hash",
        "source_fd_copy_to_private_tmpfs", "sandbox_source_mode_0400",
        "sandbox_source_nlink_one", "sandbox_source_tmpfs_remount_ro",
        "network_unshared", "proc_mount_forbidden", "dev_mount_forbidden",
        "inner_receipt_required", "host_closed_schema_validation",
        "final_receipt_o_excl", "atomic_noreplace_publish",
        "failure_preserves_partial", "prior_v2_failure_preserved",
    )
    if any(contract.get(name) is not True for name in true_flags):
        _fail("CONTRACT_CONTROL_DISABLED", "static", "required v3 control is disabled")
    for name in ("source_bag_executed", "ros_started", "gpu_exposed", "solver_started", "sudo_used"):
        if contract.get(name) is not False:
            _fail("FORBIDDEN_ACTION_ENABLED", "static", name)
    if contract.get("source_fd_number") != SOURCE_FD or contract.get("final_receipt_mode") != "0600":
        _fail("FD_OR_MODE_INVALID", "static", "source fd/final receipt mode differs")

    paths = policy["paths"]
    audit = Path(paths["audit_root"])
    targets = [Path(paths[name]) for name in (
        "partial_evidence_root", "final_evidence_root", "final_receipt_path", "failure_receipt_path"
    )]
    if any(path.parent != audit for path in targets) or len(set(targets)) != 4:
        _fail("AUDIT_TARGET_INVALID", "static", "four create-new targets differ or alias")
    expected_inventory = [
        paths["reservation_name"], paths["inner_receipt_name"], paths["stdout_name"],
        paths["stderr_name"], paths["rc_name"],
    ]
    if sorted(policy["expected_evidence_inventory"]) != sorted(expected_inventory):
        _fail("EVIDENCE_INVENTORY_INVALID", "static", "evidence inventory differs")

    limits = policy["limits"]
    expected_limits = {
        "timeout_seconds": 30,
        "kill_after_seconds": 2,
        "maximum_rss_bytes": 536870912,
        "maximum_address_space_bytes": 805306368,
        "maximum_stdout_bytes": 65536,
        "maximum_stderr_bytes": 262144,
        "maximum_inner_receipt_bytes": 2097152,
        "maximum_execution_receipt_bytes": 1048576,
        "maximum_source_entries": 512,
        "maximum_parent_components": 32,
        "nofile_limit": 64,
        "nproc_limit": 1,
    }
    if limits != expected_limits:
        _fail("LIMITS_INVALID", "static", "v3 resource limits differ")


def load_policy(path: Path = POLICY_PATH) -> tuple[dict[str, Any], str]:
    value, raw = v2.base._read_json(path, v2.base.MAX_STATIC_BYTES, "supervisor v3 policy")
    validate_policy(value)
    return value, hashlib.sha256(raw).hexdigest()


def verify_static_inputs(policy: Mapping[str, Any]) -> list[dict[str, Any]]:
    witnesses: list[dict[str, Any]] = []
    for name, item in policy["inputs"].items():
        maximum = v2.base.MAX_STATIC_BYTES
        observed = v2.base._sha256_file(Path(item["path"]), maximum)
        if observed != item["sha256"]:
            _fail("STATIC_INPUT_HASH_DRIFT", "preflight", name)
    for item in policy["tools"]:
        observed = v2.base._sha256_file(Path(item["path"]))
        if observed != item["sha256"]:
            _fail("TOOL_HASH_DRIFT", "preflight", item["name"])
        witnesses.append({
            "name": item["name"], "path": item["path"],
            "expected_sha256": item["sha256"], "observed_sha256": observed,
            "matched": True,
        })
    return witnesses


def build_bwrap_argv(policy: Mapping[str, Any]) -> tuple[list[str], str]:
    validate_policy(policy)
    source_fd = v2._ACTIVE_SOURCE_FD if v2._ACTIVE_SOURCE_FD is not None else SOURCE_FD
    inputs, paths, limits = policy["inputs"], policy["paths"], policy["limits"]
    argv = [
        "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-all",
        "--unshare-net", "--clearenv", "--cap-drop", "ALL",
        "--ro-bind", "/usr", "/usr", "--ro-bind", "/lib", "/lib",
        "--ro-bind", "/lib64", "/lib64", "--tmpfs", "/tmp", "--dir", "/app",
        "--ro-bind", inputs["reader_v1"]["path"], "/app/r8_liquid_ros1_bag_v2_reader_v1.py",
        "--ro-bind", inputs["reader_v2"]["path"], "/app/r8_liquid_ros1_bag_v2_reader_v2.py",
        "--ro-bind", inputs["gate_v1"]["path"], "/app/r8_liquid_s5a0_selected_bag_intake_gate_v1.py",
        "--ro-bind", inputs["gate_v2"]["path"], "/app/r8_liquid_s5a0_selected_bag_intake_gate_v2.py",
        "--ro-bind", inputs["worker_v4"]["path"], "/app/r8_liquid_s5a0_selected_bag_worker_v4.py",
        "--ro-bind", inputs["worker_v5"]["path"], "/app/worker.py",
        "--dir", "/policy", "--ro-bind", inputs["base_policy"]["path"], "/policy/base.json",
        "--dir", "/schema", "--ro-bind", inputs["inner_schema_v2"]["path"], "/schema/inner.json",
        "--tmpfs", "/selected", "--perms", "0400", "--file", str(source_fd),
        "/selected/capture.bag", "--remount-ro", "/selected",
        "--dir", "/evidence", "--bind", paths["partial_evidence_root"], "/evidence",
        "--setenv", "PATH", "/usr/bin", "--setenv", "HOME", "/nonexistent",
        "--setenv", "PYTHONDONTWRITEBYTECODE", "1", "--setenv", "PYTHONHASHSEED", "0",
        "--chdir", "/app", "--",
        "/usr/bin/timeout", "--foreground", "--signal=KILL",
        f"--kill-after={limits['kill_after_seconds']}s", f"{limits['timeout_seconds']}s",
        "/usr/bin/prlimit", f"--rss={limits['maximum_rss_bytes']}:{limits['maximum_rss_bytes']}",
        f"--as={limits['maximum_address_space_bytes']}:{limits['maximum_address_space_bytes']}",
        f"--fsize={max(limits['maximum_stdout_bytes'], limits['maximum_stderr_bytes'])}:{max(limits['maximum_stdout_bytes'], limits['maximum_stderr_bytes'])}",
        f"--nofile={limits['nofile_limit']}:{limits['nofile_limit']}",
        f"--nproc={limits['nproc_limit']}:{limits['nproc_limit']}",
        "/usr/bin/python3", "-I", "-B", "/app/worker.py", "sandbox-inspect",
        "--policy", "/policy/base.json", "--schema", "/schema/inner.json",
        "--source", "/selected/capture.bag",
        "--receipt", "/evidence/selected_bag_inner_receipt_v2.json",
        "--final-evidence-root", paths["final_evidence_root"],
        "--expected-argv-sha256", "0" * 64,
    ]
    digest = v2._normalised_argv_sha256(argv)
    argv[argv.index("--expected-argv-sha256") + 1] = digest
    forbidden = {
        policy["selection"]["absolute_path"], policy["selection"]["forbidden_optional_path"],
        "--ro-bind-data", "--proc", "--dev", "sudo", "nvidia-smi",
    }
    if any(token in forbidden for token in argv) or paths["audit_root"] in argv:
        _fail("BWRAP_EXPOSURE_INVALID", "static", "forbidden source/path/action exposed")
    if argv.count("--file") != 1 or argv.count("--remount-ro") != 1:
        _fail("PRIVATE_COPY_CARDINALITY_INVALID", "static", "private file/remount cardinality differs")
    return argv, digest


def _install_revision() -> None:
    base = v2.base
    base.POLICY_PATH, base.SCHEMA_PATH, base.EXECUTION_ID = POLICY_PATH, SCHEMA_PATH, EXECUTION_ID
    base.validate_policy, base.load_policy = validate_policy, load_policy
    base.load_schema = v2.load_schema
    base.verify_static_inputs, base.build_bwrap_argv = verify_static_inputs, build_bwrap_argv
    base.build_execution_receipt = v2.build_execution_receipt
    base.bag_reader = reader_v2
    base.gate_v1.bag_reader = reader_v2
    base.gate_v1.load_schema = v2.load_inner_schema


def run_one_shot() -> dict[str, Any]:
    policy, _ = load_policy()
    v2._ACTIVE_SOURCE_FD = v2._open_frozen_source(policy)
    _install_revision()
    try:
        return v2.base.run_one_shot(POLICY_PATH, runner=v2._fd_runner)
    finally:
        if v2._ACTIVE_SOURCE_FD is not None:
            os.close(v2._ACTIVE_SOURCE_FD)
        v2._ACTIVE_SOURCE_FD = None


def self_check() -> dict[str, Any]:
    policy, policy_sha = load_policy()
    _schema, schema_sha = v2.load_schema()
    v2.load_inner_schema()
    tools = verify_static_inputs(policy)
    argv, argv_sha = build_bwrap_argv(policy)
    delimiter = argv.index("--")
    setup = argv[:delimiter]
    expected_sequence = ["--tmpfs", "/selected", "--perms", "0400", "--file", str(SOURCE_FD), "/selected/capture.bag", "--remount-ro", "/selected"]
    start = setup.index("--tmpfs", setup.index("--tmpfs") + 1)
    if setup[start:start + len(expected_sequence)] != expected_sequence:
        _fail("PRIVATE_COPY_SEQUENCE_INVALID", "static", "private tmpfs copy/remount sequence differs")
    if any(token in setup for token in ("--proc", "--dev", "--ro-bind-data")):
        _fail("SANDBOX_STATIC_INVALID", "static", "proc/dev/ro-bind-data appears")
    return {
        "status": "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_V3_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID,
        "policy_sha256": policy_sha,
        "schema_sha256": schema_sha,
        "argv_sha256": argv_sha,
        "tool_count": len(tools),
        "source_fd_number": SOURCE_FD,
        "source_guest_mode": "0400",
        "source_guest_nlink": 1,
        "source_tmpfs_read_only": True,
        "prior_v2_failure_preserved": True,
        "real_bag_read": False,
        "bwrap_executed": False,
        "proc_mounted": False,
        "dev_mounted": False,
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
    except v2.base.SupervisorError as exc:
        print(json.dumps({
            "status": v2.base.FAIL_STATUS, "code": exc.code,
            "phase": exc.phase, "error": exc.message,
        }, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if args.command == "self-check" or result["status"] == v2.base.PASS_STATUS else 2


if __name__ == "__main__":
    raise SystemExit(main())
