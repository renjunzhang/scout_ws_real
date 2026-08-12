#!/usr/bin/env python3
"""Create-new S5A1 one-shot execution revision for the S5A0 v5 parent.

The proven v1 package producer remains byte-bound and is reused as a pure
builder.  This revision replaces only the ROS1 reader/receipt adapters,
freezes every parent in an execution policy, drops sandbox capabilities, and
publishes an execution-v3 closed receipt.  It never executes the bag, ROS,
GPU, candidate, or solver.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_execution_supervisor_v1 as base
import r8_liquid_s5a1_handoff_gate_v2 as gate_v2
import r8_liquid_s5a1_ros1_signal_extractor_v2 as extractor_v2


sys.dont_write_bytecode = True
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T202140Z_v3"
EXECUTION_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_execution_policy_v3.json"
PACKAGE_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v1.json"
PACKAGE_SCHEMA = ROOT / "schema/target_host_s5a1_handoff_package_v1.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v3.json"
RECEIPT_SCHEMA_V1 = ROOT / "schema/target_host_s5a1_execution_receipt_v1.json"
S5A0_INNER_SCHEMA = ROOT / "schema/target_host_s5a0_selected_bag_receipt_v2.json"
S5A0_OUTER_SCHEMA = ROOT / "schema/target_host_s5a0_primary_supervisor_execution_receipt_v5.json"
S5A0_SUPERVISOR_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a0_primary_supervisor_policy_v5.json"
GATE_V1 = ROOT / "scripts/r8_liquid_s5a1_handoff_gate_v1.py"
GATE_V2 = ROOT / "scripts/r8_liquid_s5a1_handoff_gate_v2.py"
EXTRACTOR_V1 = ROOT / "scripts/r8_liquid_s5a1_ros1_signal_extractor_v1.py"
EXTRACTOR_V2 = ROOT / "scripts/r8_liquid_s5a1_ros1_signal_extractor_v2.py"
BRIDGE_V1 = ROOT / "scripts/r8_liquid_s5a1_motion_bridge_v1.py"
READERS = tuple(
    ROOT / f"scripts/r8_liquid_ros1_bag_v2_reader_v{revision}.py"
    for revision in range(1, 5)
)
COMPAT_GATE = ROOT / "scripts/r8_liquid_s5_c1_compatibility_gate_v1.py"
BASE_SUPERVISOR = ROOT / "scripts/r8_liquid_s5a1_execution_supervisor_v1.py"
CONTINUATION_PLAN = ROOT.parents[3] / "docs/实物实验注意事项/对比试验/仿真接入液体/20260811_RTX5080_DualSPHysics_GPU液体仿真阶段3-6续接方案_Agent执行版.md"
TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1"
PARTIAL_ROOT = Path(f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}.partial")
FINAL_ROOT = Path(f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}")
EXECUTION_RECEIPT = Path(
    f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v3.json"
)
EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT.with_name(
    EXECUTION_RECEIPT.name + ".partial"
)
S5A0_FINAL_RECEIPT = Path(
    "/home/zrj/scout_liquid_lab/audits/"
    "s5a0_primary_bsmooth_b01_20260811T200154Z_v5.host.final.json"
)
S5A0_EVIDENCE_ROOT = Path(
    "/home/zrj/scout_liquid_lab/audits/"
    "s5a0_primary_bsmooth_b01_20260811T200154Z_v5.host.evidence"
)
S5A0_INNER_RECEIPT = S5A0_EVIDENCE_ROOT / "selected_bag_inner_receipt_v2.json"
RECEIPT_PARENT_NAMES = (
    "continuation_plan", "execution_policy", "package_policy",
    "package_schema", "handoff_gate_v1", "handoff_gate_v2",
    "signal_extractor_v1", "signal_extractor_v2", "motion_bridge_v1",
    "reader_v1", "reader_v2", "reader_v3", "reader_v4",
    "s5a0_inner_receipt_schema_v2", "s5a0_outer_receipt_schema_v5",
    "s5a0_supervisor_policy_v5", "compatibility_gate_v1",
    "execution_supervisor_v1", "execution_supervisor_v3",
    "execution_receipt_schema_v1", "execution_receipt_schema_v3",
)

_BASE_BUILD_BWRAP = base.build_bwrap_argv
_BASE_RESERVE_RECEIPT = base.reserve_receipt
_BASE_PUBLISH_RECEIPT = base.publish_reserved_receipt
_BASE_MOCK_FAILURE = base.mock_failure_receipt
_ACTIVE_POLICY: dict[str, Any] | None = None


class ExecutionV3Error(ValueError):
    """Static identity, one-shot admission, or receipt transformation error."""


def _sha256_regular(path: Path, maximum: int = 64 * 1024 * 1024) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    digest = hashlib.sha256()
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or not 0 < metadata.st_size <= maximum
        ):
            raise ExecutionV3Error(f"unsafe frozen input: {path}")
        remaining = metadata.st_size
        while remaining:
            block = os.read(descriptor, min(remaining, 1024 * 1024))
            if not block:
                raise ExecutionV3Error(f"short frozen input read: {path}")
            digest.update(block)
            remaining -= len(block)
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def load_execution_policy() -> tuple[dict[str, Any], str]:
    raw = EXECUTION_POLICY.read_bytes()
    policy = json.loads(raw)
    expected_keys = {
        "schema_version", "document_type", "execution_id", "status",
        "selection", "parents", "tools", "paths", "limits", "contract",
    }
    if set(policy) != expected_keys:
        raise ExecutionV3Error("execution policy root is not closed")
    if (
        policy["schema_version"] != "smpcc-r8-liquid-s5a1-primary-execution-policy-v3"
        or policy["document_type"] != "SMPCC_R8_LIQUID_S5A1_PRIMARY_EXECUTION_POLICY_V3"
        or policy["execution_id"] != EXECUTION_ID
        or policy["status"] != "FROZEN_ONE_SHOT_NOT_EXECUTED"
    ):
        raise ExecutionV3Error("execution policy identity differs")
    selection = policy["selection"]
    if (
        selection != {
            "attempt_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01",
            "role": "PRIMARY",
            "selected_count": 1,
            "optional_authorized": False,
            "source_path": "/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag",
            "source_sha256": "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07",
            "source_size_bytes": 13996902,
            "source_mode": "0755",
        }
    ):
        raise ExecutionV3Error("primary-only selection differs")
    expected_parents = (set(RECEIPT_PARENT_NAMES) - {"execution_policy"}) | {
        "s5a0_outer_final_receipt", "s5a0_inner_receipt"
    }
    if set(policy["parents"]) != expected_parents:
        raise ExecutionV3Error("execution parent set differs")
    for name, identity in policy["parents"].items():
        if set(identity) != {"path", "sha256"}:
            raise ExecutionV3Error(f"open parent identity: {name}")
        path = Path(identity["path"])
        if not path.is_absolute() or _sha256_regular(path) != identity["sha256"]:
            raise ExecutionV3Error(f"parent hash drifted: {name}")
    for name, identity in policy["tools"].items():
        if set(identity) != {"path", "sha256"}:
            raise ExecutionV3Error(f"open tool identity: {name}")
        if _sha256_regular(Path(identity["path"])) != identity["sha256"]:
            raise ExecutionV3Error(f"tool hash drifted: {name}")
    if policy["paths"] != {
        "partial_root": str(PARTIAL_ROOT),
        "final_root": str(FINAL_ROOT),
        "execution_receipt_partial": str(EXECUTION_RECEIPT_PARTIAL),
        "execution_receipt_final": str(EXECUTION_RECEIPT),
    }:
        raise ExecutionV3Error("execution output paths differ")
    limits = policy["limits"]
    if limits != {
        "timeout_seconds": 180,
        "kill_after_seconds": 5,
        "address_space_limit_bytes": 1073741824,
        "rss_limit_bytes": 805306368,
        "nofile_limit": 256,
        "stdout_limit_bytes": 65536,
        "stderr_limit_bytes": 65536,
    }:
        raise ExecutionV3Error("execution limits differ")
    contract = policy["contract"]
    required_true = {
        "one_shot", "create_new", "source_fd_o_nofollow",
        "s5a0_receipt_fd_o_nofollow", "network_unshared",
        "capabilities_dropped", "proc_mount_forbidden", "dev_mount_forbidden",
        "odom_only_motion", "quaternion_slerp", "solver_path_roundtrip",
        "c1_compatibility_required", "atomic_rename_noreplace",
        "closed_receipt_validation", "source_unchanged_required",
    }
    if any(contract.get(name) is not True for name in required_true):
        raise ExecutionV3Error("required execution control is disabled")
    for name in (
        "optional_bag_read", "source_bag_executed", "ros_started",
        "gpu_exposed", "solver_executed", "network_used", "sudo_used",
    ):
        if contract.get(name) is not False:
            raise ExecutionV3Error(f"forbidden execution control enabled: {name}")
    return policy, hashlib.sha256(raw).hexdigest()


def _expected_map(policy: Mapping[str, Any]) -> dict[Path, str]:
    return {
        Path(identity["path"]): identity["sha256"]
        for identity in policy["parents"].values()
    }


def build_bwrap_argv(
    receipt: Path,
    source: Path,
    partial: Path,
    *,
    receipt_fd: int | None = None,
    source_fd: int | None = None,
) -> list[str]:
    argv = _BASE_BUILD_BWRAP(
        receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd
    )
    clear_index = argv.index("--clearenv") + 1
    argv[clear_index:clear_index] = ["--cap-drop", "ALL"]
    extra = (
        EXECUTION_POLICY, BASE_SUPERVISOR, GATE_V1, EXTRACTOR_V1,
        READERS[0], READERS[1], READERS[2], RECEIPT_SCHEMA_V1,
    )
    bind_index = argv.index("--bind")
    bindings: list[str] = []
    for path in extra:
        if str(path) not in argv:
            bindings.extend(("--ro-bind", str(path), str(path)))
    argv[bind_index:bind_index] = bindings
    joined = "\0".join(argv).lower()
    for forbidden in (
        "sudo", "/dev/nvidia", "nvidia-smi", "roscore",
        "dualsphysics5.4_linux64", "--proc", "--dev",
    ):
        if forbidden in joined:
            raise ExecutionV3Error(f"forbidden sandbox token: {forbidden}")
    if argv.count("--cap-drop") != 1 or argv[argv.index("--cap-drop") + 1] != "ALL":
        raise ExecutionV3Error("capability drop differs")
    return argv


def reserve_receipt() -> int:
    return _BASE_RESERVE_RECEIPT(EXECUTION_RECEIPT_PARTIAL)


def _receipt_parent_map(policy: Mapping[str, Any], policy_sha: str) -> dict[str, Any]:
    parents = {
        name: dict(policy["parents"][name])
        for name in RECEIPT_PARENT_NAMES
        if name != "execution_policy"
    }
    parents["execution_policy"] = {
        "path": str(EXECUTION_POLICY), "sha256": policy_sha
    }
    return parents


def transform_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    if _ACTIVE_POLICY is None:
        raise ExecutionV3Error("execution policy is not installed")
    policy_sha = hashlib.sha256(EXECUTION_POLICY.read_bytes()).hexdigest()
    return {
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v3",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V3",
        "execution_id": EXECUTION_ID,
        "receipt_id": f"{TRANSFER_ID}_execution_v3",
        "status": value["status"],
        "parents": _receipt_parent_map(_ACTIVE_POLICY, policy_sha),
        "execution": value["execution"],
        "inputs": value["inputs"],
        "roots": value["roots"],
        "package": value["package"],
        "compatibility": value["compatibility"],
        "safety": value["safety"],
        "failure": value["failure"],
        "next": value["next"],
    }


def publish_reserved_receipt(
    descriptor: int, partial_path: Path, final_path: Path,
    value: Mapping[str, Any],
) -> None:
    transformed = transform_receipt(value)
    schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator(schema).validate(transformed)
    _BASE_PUBLISH_RECEIPT(
        descriptor, partial_path, final_path, transformed
    )


def mock_failure_receipt() -> dict[str, Any]:
    return transform_receipt(_BASE_MOCK_FAILURE())


def _install_revision(policy: dict[str, Any]) -> None:
    global _ACTIVE_POLICY
    _ACTIVE_POLICY = policy
    base.SCRIPT = SCRIPT
    base.POLICY = PACKAGE_POLICY
    base.PACKAGE_SCHEMA = PACKAGE_SCHEMA
    base.RECEIPT_SCHEMA = RECEIPT_SCHEMA
    base.S5A0_SCHEMA = S5A0_INNER_SCHEMA
    base.S5A0_FINAL_SCHEMA = S5A0_OUTER_SCHEMA
    base.S5A0_SUPERVISOR_POLICY = S5A0_SUPERVISOR_POLICY
    base.GATE = GATE_V2
    base.EXTRACTOR = EXTRACTOR_V2
    base.BRIDGE = BRIDGE_V1
    base.READER = READERS[3]
    base.COMPAT_GATE = COMPAT_GATE
    base.TRANSFER_ID = TRANSFER_ID
    base.PARTIAL_ROOT = PARTIAL_ROOT
    base.FINAL_ROOT = FINAL_ROOT
    base.EXECUTION_RECEIPT = EXECUTION_RECEIPT
    base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT_PARTIAL
    base.S5A0_FINAL_RECEIPT = S5A0_FINAL_RECEIPT
    base.S5A0_EVIDENCE_ROOT = S5A0_EVIDENCE_ROOT
    base.S5A0_INNER_RECEIPT = S5A0_INNER_RECEIPT
    base.EXPECTED = _expected_map(policy)
    base.gate = gate_v2
    base.extractor = extractor_v2
    base.build_bwrap_argv = build_bwrap_argv
    base.reserve_receipt = reserve_receipt
    base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt


def static_self_check() -> dict[str, Any]:
    policy, policy_sha = load_execution_policy()
    _install_revision(policy)
    result = base.static_self_check()
    schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    gate_v2.assert_schema_deep_closed(schema)
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(mock_failure_receipt())
    argv = build_bwrap_argv(
        S5A0_INNER_RECEIPT,
        Path(policy["selection"]["source_path"]),
        PARTIAL_ROOT,
    )
    return {
        "status": "PASS_S5A1_EXECUTION_SUPERVISOR_V3_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID,
        "execution_policy_sha256": policy_sha,
        "receipt_schema_sha256": hashlib.sha256(RECEIPT_SCHEMA.read_bytes()).hexdigest(),
        "base_status": result["status"],
        "verified_parent_count": len(policy["parents"]),
        "argv_sha256": hashlib.sha256("\0".join(argv).encode()).hexdigest(),
        "source_parent_present": S5A0_FINAL_RECEIPT.is_file(),
        "transfer_roots_fresh": not any(
            os.path.lexists(path)
            for path in (
                PARTIAL_ROOT, FINAL_ROOT, EXECUTION_RECEIPT,
                EXECUTION_RECEIPT_PARTIAL,
            )
        ),
        "real_bag_read": False,
        "optional_bag_read": False,
        "bwrap_run": False,
        "files_written": False,
        "gpu_exposed": False,
        "solver_executed": False,
        "sudo_used": False,
    }


def execute_one_shot(receipt_path: Path, receipt_sha256: str) -> dict[str, Any]:
    policy, _ = load_execution_policy()
    _install_revision(policy)
    base.execute_one_shot(receipt_path, receipt_sha256)
    raw = EXECUTION_RECEIPT.read_bytes()
    value = json.loads(raw)
    schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator(schema).validate(value)
    return value


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-check")
    run = sub.add_parser("run-one-shot")
    run.add_argument("--s5a0-receipt", type=Path, required=True)
    run.add_argument("--s5a0-receipt-sha256", required=True)
    run.add_argument("--authorize-execution-id", required=True)
    worker = sub.add_parser("_worker")
    worker.add_argument("--s5a0-receipt", type=Path, required=True)
    worker.add_argument("--source", type=Path, required=True)
    worker.add_argument("--partial-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        policy, _ = load_execution_policy()
        _install_revision(policy)
        if args.command == "self-check":
            result = static_self_check()
        elif args.command == "run-one-shot":
            if args.authorize_execution_id != EXECUTION_ID:
                raise ExecutionV3Error("exact execution authorization differs")
            result = execute_one_shot(
                args.s5a0_receipt, args.s5a0_receipt_sha256
            )
        else:
            result = base.worker(
                args.s5a0_receipt, args.source, args.partial_root
            )
    except Exception as exc:
        print(json.dumps({
            "status": "FAIL_CLOSED", "execution_id": EXECUTION_ID,
            "error": str(exc),
        }, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
