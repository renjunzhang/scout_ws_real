#!/usr/bin/env python3
"""Create-new S5A0 supervisor v5 for legal ROS1 file-header padding.

V5 preserves every v1-v4 failure artifact.  Its only parser delta is reader
v4's exact treatment of the standard space padding that extends a ROS1 bag
file-header record to 4 KiB.  The bytes are retained in memory for boundary
accounting; the source bag is never repaired, rewritten, or executed.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v4 as reader_v4
import r8_liquid_s5a0_primary_one_shot_supervisor_v4 as v4


sys.dont_write_bytecode = True
PACKAGE_ROOT = MODULE_DIR.parent
POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a0_primary_supervisor_policy_v5.json"
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_s5a0_primary_supervisor_execution_receipt_v5.json"
INNER_SCHEMA_PATH = v4.INNER_SCHEMA_PATH
EXECUTION_ID = "s5a0_primary_bsmooth_b01_20260811T200154Z_v5"
SOURCE_FD = v4.SOURCE_FD


def _fail(code: str, phase: str, message: str) -> None:
    raise v4.v3.v2.base.SupervisorError(code, phase, message)


def _v4_projected_policy(policy: Mapping[str, Any]) -> dict[str, Any]:
    projected = copy.deepcopy(dict(policy))
    projected["schema_version"] = "smpcc-r8-liquid-s5a0-primary-supervisor-policy-v4"
    projected["document_type"] = "SMPCC_R8_LIQUID_S5A0_PRIMARY_SUPERVISOR_POLICY_V4"
    projected["execution_id"] = v4.EXECUTION_ID
    keys = {
        "base_policy", "reader_v1", "reader_v2", "reader_v3", "gate_v1",
        "gate_v2", "worker_v4", "worker_v5", "worker_v6",
        "inner_schema_v2", "base_supervisor_v1", "outer_schema_v2",
        "outer_schema_v4", "supervisor_v3", "supervisor_v4",
        "prior_v3_policy", "prior_v2_failure_receipt",
        "prior_v2_worker_stderr", "prior_v3_reservation",
        "prior_v3_worker_rc", "prior_v3_worker_stderr",
        "prior_v3_worker_stdout",
    }
    projected["inputs"] = {
        name: copy.deepcopy(policy["inputs"][name]) for name in keys
    }
    return projected


def validate_policy(policy: Mapping[str, Any]) -> None:
    required = {
        "schema_version", "document_type", "execution_id", "status",
        "selection", "inputs", "tools", "paths", "limits", "contract",
        "expected_evidence_inventory",
    }
    if set(policy) != required:
        _fail("POLICY_KEYS_INVALID", "static", "v5 policy root is not closed")
    if (
        policy["schema_version"]
        != "smpcc-r8-liquid-s5a0-primary-supervisor-policy-v5"
        or policy["document_type"]
        != "SMPCC_R8_LIQUID_S5A0_PRIMARY_SUPERVISOR_POLICY_V5"
        or policy["execution_id"] != EXECUTION_ID
    ):
        _fail("POLICY_IDENTITY_INVALID", "static", "v5 policy identity differs")

    expected_inputs = {
        "base_policy", "reader_v1", "reader_v2", "reader_v3", "reader_v4",
        "gate_v1", "gate_v2", "worker_v4", "worker_v5", "worker_v6",
        "worker_v7", "inner_schema_v2", "base_supervisor_v1",
        "outer_schema_v2", "outer_schema_v4", "outer_schema_v5",
        "supervisor_v3", "supervisor_v4", "supervisor_v5",
        "prior_v3_policy", "prior_v4_policy", "prior_v2_failure_receipt",
        "prior_v2_worker_stderr", "prior_v3_reservation",
        "prior_v3_worker_rc", "prior_v3_worker_stderr",
        "prior_v3_worker_stdout", "prior_v4_failure_receipt",
        "prior_v4_reservation", "prior_v4_worker_rc",
        "prior_v4_worker_stderr", "prior_v4_worker_stdout",
    }
    if set(policy["inputs"]) != expected_inputs:
        _fail("INPUT_SET_INVALID", "static", "v5 parent/input set differs")
    for item in list(policy["inputs"].values()) + list(policy["tools"]):
        v4.v3.v2.gate_v1._absolute_normalized(item["path"], "input/tool path")
        if not v4.v3.v2.gate_v1._is_lower_hex(item.get("sha256"), 64):
            _fail("STATIC_HASH_INVALID", "static", item["path"])

    v4.validate_policy(_v4_projected_policy(policy))
    contract = policy["contract"]
    for name in (
        "prior_v4_failure_preserved", "prior_v4_worker_failure_bound",
        "file_header_padding_preserved_in_memory",
        "file_header_padding_exact_boundary_verified",
        "outer_schema_execution_identity_exact",
    ):
        if contract.get(name) is not True:
            _fail("V5_REMEDIATION_CONTROL_DISABLED", "static", name)
    if contract.get("source_repaired_or_written") is not False:
        _fail(
            "V5_SOURCE_MUTATION_CONTROL_INVALID", "static",
            "source_repaired_or_written must remain false",
        )


def load_policy(path: Path = POLICY_PATH) -> tuple[dict[str, Any], str]:
    value, raw = v4.v3.v2.base._read_json(
        path, v4.v3.v2.base.MAX_STATIC_BYTES, "supervisor v5 policy"
    )
    validate_policy(value)
    return value, hashlib.sha256(raw).hexdigest()


def load_schema(path: Path = SCHEMA_PATH) -> tuple[dict[str, Any], str]:
    value, raw = v4.v3.v2.base._read_json(
        path, v4.v3.v2.base.MAX_STATIC_BYTES, "execution receipt schema v5"
    )
    v4.v3.v2.gate_v1.assert_deep_closed(value)
    Draft202012Validator.check_schema(value)
    return value, hashlib.sha256(raw).hexdigest()


def verify_static_inputs(policy: Mapping[str, Any]) -> list[dict[str, Any]]:
    return v4.verify_static_inputs(policy)


def build_bwrap_argv(policy: Mapping[str, Any]) -> tuple[list[str], str]:
    validate_policy(policy)
    projected = _v4_projected_policy(policy)
    projected["inputs"]["worker_v6"] = copy.deepcopy(policy["inputs"]["worker_v7"])
    argv, _old_digest = v4.build_bwrap_argv(projected)
    worker_path = policy["inputs"]["worker_v7"]["path"]
    marker = argv.index(worker_path)
    if argv[marker - 1] != "--ro-bind" or argv[marker + 1] != "/app/worker.py":
        _fail("WORKER_BIND_INVALID", "static", "worker v7 bind is not exact")
    argv[marker - 1:marker - 1] = [
        "--ro-bind", policy["inputs"]["reader_v4"]["path"],
        "/app/r8_liquid_ros1_bag_v2_reader_v4.py",
        "--ro-bind", policy["inputs"]["worker_v6"]["path"],
        "/app/r8_liquid_s5a0_selected_bag_worker_v6.py",
    ]
    digest_marker = argv.index("--expected-argv-sha256") + 1
    argv[digest_marker] = "0" * 64
    digest = v4.v3.v2._normalised_argv_sha256(argv)
    argv[digest_marker] = digest
    forbidden = {
        policy["selection"]["absolute_path"],
        policy["selection"]["forbidden_optional_path"],
        "--ro-bind-data", "--proc", "--dev", "sudo", "nvidia-smi",
    }
    if any(token in forbidden for token in argv) or policy["paths"]["audit_root"] in argv:
        _fail("BWRAP_EXPOSURE_INVALID", "static", "v5 argv exposes a forbidden path/action")
    for name in ("reader_v4", "worker_v6", "worker_v7"):
        if argv.count(policy["inputs"][name]["path"]) != 1:
            _fail("V5_BIND_CARDINALITY_INVALID", "static", f"{name} bind differs")
    return argv, digest


def build_execution_receipt(*args: Any, **kwargs: Any) -> dict[str, Any]:
    receipt = v4.build_execution_receipt(*args, **kwargs)
    receipt["schema_version"] = (
        "smpcc-r8-liquid-s5a0-primary-supervisor-execution-receipt-v5"
    )
    receipt["document_type"] = (
        "SMPCC_R8_LIQUID_S5A0_PRIMARY_SUPERVISOR_EXECUTION_RECEIPT_V5"
    )
    return receipt


def _install_revision() -> None:
    base = v4.v3.v2.base
    base.POLICY_PATH, base.SCHEMA_PATH, base.EXECUTION_ID = (
        POLICY_PATH, SCHEMA_PATH, EXECUTION_ID
    )
    base.validate_policy, base.load_policy, base.load_schema = (
        validate_policy, load_policy, load_schema
    )
    base.verify_static_inputs, base.build_bwrap_argv = (
        verify_static_inputs, build_bwrap_argv
    )
    base.build_execution_receipt = build_execution_receipt
    base.bag_reader = reader_v4
    base.gate_v1.bag_reader = reader_v4
    base.gate_v1.load_schema = v4.v3.v2.load_inner_schema


def run_one_shot() -> dict[str, Any]:
    policy, _ = load_policy()
    v4.v3.v2._ACTIVE_SOURCE_FD = v4.v3.v2._open_frozen_source(policy)
    _install_revision()
    try:
        return v4.v3.v2.base.run_one_shot(
            POLICY_PATH, runner=v4.v3.v2._fd_runner
        )
    finally:
        if v4.v3.v2._ACTIVE_SOURCE_FD is not None:
            os.close(v4.v3.v2._ACTIVE_SOURCE_FD)
        v4.v3.v2._ACTIVE_SOURCE_FD = None


def self_check() -> dict[str, Any]:
    policy, policy_sha = load_policy()
    _schema, schema_sha = load_schema()
    v4.v3.v2.load_inner_schema()
    tools = verify_static_inputs(policy)
    argv, argv_sha = build_bwrap_argv(policy)
    delimiter = argv.index("--")
    setup = argv[:delimiter]
    if any(token in setup for token in ("--proc", "--dev", "--ro-bind-data")):
        _fail("SANDBOX_STATIC_INVALID", "static", "proc/dev/ro-bind-data appears")
    if argv.count("--file") != 1 or argv.count("--remount-ro") != 1:
        _fail("PRIVATE_COPY_CARDINALITY_INVALID", "static", "private copy/remount differs")
    return {
        "status": "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_V5_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID,
        "policy_sha256": policy_sha,
        "schema_sha256": schema_sha,
        "argv_sha256": argv_sha,
        "tool_count": len(tools),
        "reader_revision": "v4",
        "worker_revision": "v7",
        "source_fd_number": SOURCE_FD,
        "source_guest_mode": "0400",
        "source_guest_nlink": 1,
        "source_tmpfs_read_only": True,
        "file_header_padding_preserved_in_memory": True,
        "source_repaired_or_written": False,
        "prior_v1_v2_v3_v4_evidence_preserved": True,
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
                _fail(
                    "EXECUTION_NOT_AUTHORIZED", "authorization",
                    "exact execution id differs",
                )
            result = run_one_shot()
    except v4.v3.v2.base.SupervisorError as exc:
        print(json.dumps({
            "status": v4.v3.v2.base.FAIL_STATUS,
            "code": exc.code,
            "phase": exc.phase,
            "error": exc.message,
        }, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return (
        0 if args.command == "self-check"
        or result["status"] == v4.v3.v2.base.PASS_STATUS else 2
    )


if __name__ == "__main__":
    raise SystemExit(main())
