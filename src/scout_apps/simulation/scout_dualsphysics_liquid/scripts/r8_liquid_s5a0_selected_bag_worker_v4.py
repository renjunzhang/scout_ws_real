#!/usr/bin/env python3
"""Proc/dev-free S5A0 worker v4 pinned to the remediated Bag V2 reader."""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v2 as reader_v2
import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate_v1
import r8_liquid_s5a0_selected_bag_intake_gate_v2 as gate_v2


sys.dont_write_bytecode = True
gate_v1.bag_reader = reader_v2
INNER_RECEIPT_NAME = "selected_bag_inner_receipt_v2.json"
VISIBLE_SOURCE = Path("/selected/capture.bag")
VISIBLE_RECEIPT = Path("/evidence") / INNER_RECEIPT_NAME


class WorkerV4Error(ValueError):
    """The exact proc/dev-free remediation worker failed closed."""


def sandbox_inspect(
    *, policy_path: Path, schema_path: Path, source_path: Path, receipt_path: Path,
    final_evidence_root: Path, expected_argv_sha256: str,
) -> dict[str, Any]:
    if source_path != VISIBLE_SOURCE or receipt_path != VISIBLE_RECEIPT:
        raise WorkerV4Error("worker paths differ from the exact sandbox contract")
    if not gate_v1._is_lower_hex(expected_argv_sha256, 64):
        raise WorkerV4Error("expected argv SHA-256 is invalid")
    if not final_evidence_root.is_absolute() or final_evidence_root.name.endswith(".partial"):
        raise WorkerV4Error("final evidence root is invalid")
    policy, policy_sha256 = gate_v1.load_policy(policy_path)
    gate_v1.validate_frozen_policy(policy)
    schema, _ = gate_v1._read_bounded_json(schema_path, gate_v1.MAXIMUM_SCHEMA_BYTES, "inner receipt schema v2")
    flags = os.statvfs(source_path).f_flag
    if not flags & getattr(os, "ST_RDONLY", 1):
        raise WorkerV4Error("sandbox source statvfs is not read-only")
    runtime_policy = copy.deepcopy(policy)
    runtime_policy["sandbox_contract"]["audit_root"] = str(final_evidence_root)
    # The outer supervisor binds a verified host 0755 inode through an
    # anonymous fd.  The sandbox sees a distinct 0400 regular-file copy.
    runtime_policy["selection"]["expected_mode"] = "0400"
    gate_v1.validate_policy(runtime_policy)
    receipt = gate_v1.inspect_selected(
        runtime_policy, source_path=source_path, receipt_path=final_evidence_root / INNER_RECEIPT_NAME,
        mountinfo_text="1 0 0:0 / /selected ro,nosuid,nodev - bwrap exact-ro-bind ro\n",
        statvfs_flags=flags, policy_sha256=policy_sha256,
        captured_at_utc=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        snapshot_root=source_path.parent, source_is_sandbox_visible_path=True,
    )
    receipt["schema_version"] = "smpcc-r8-liquid-s5a0-selected-bag-receipt-v2"
    receipt["document_type"] = "SMPCC_R8_LIQUID_S5A0_SELECTED_BAG_RECEIPT_V2"
    receipt["host_source_provenance"] = {
        "identity_scope": "OUTER_SUPERVISOR_VERIFIED_HOST_SOURCE",
        "absolute_path": policy["selection"]["absolute_path"],
        "expected_mode": policy["selection"]["expected_mode"],
        "expected_size_bytes": policy["selection"]["expected_size_bytes"],
        "expected_sha256": policy["selection"]["expected_sha256"],
        "guest_copy_distinct_inode": True,
        "guest_copy_mode": "0400",
    }
    receipt["sandbox"]["bwrap_argv_sha256"] = expected_argv_sha256
    receipt["output"]["receipt_path"] = str(final_evidence_root / INNER_RECEIPT_NAME)
    gate_v2.validate_runtime_receipt(receipt, schema)
    return gate_v1.write_receipt_create_new(
        receipt_path, receipt, maximum_bytes=policy["limits"]["maximum_receipt_bytes"],
    )


def self_check() -> dict[str, Any]:
    policy, _ = gate_v1.load_policy()
    gate_v1.validate_frozen_policy(policy)
    return {
        "status": "PASS_S5A0_SELECTED_BAG_WORKER_V4_STATIC_CONTRACT",
        "reader_revision": "v2", "proc_mounted": False, "dev_mounted": False,
        "real_bag_opened": False, "process_executed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("self-check")
    worker = commands.add_parser("sandbox-inspect")
    for name in ("policy", "schema", "source", "receipt", "final-evidence-root", "expected-argv-sha256"):
        worker.add_argument(f"--{name}", required=True)
    args = parser.parse_args(argv)
    try:
        result = self_check() if args.command == "self-check" else sandbox_inspect(
            policy_path=Path(args.policy), schema_path=Path(args.schema), source_path=Path(args.source),
            receipt_path=Path(args.receipt), final_evidence_root=Path(args.final_evidence_root),
            expected_argv_sha256=args.expected_argv_sha256,
        )
    except (WorkerV4Error, gate_v1.S5A0GateError, gate_v2.RuntimeReceiptError, OSError, ValueError) as exc:
        print(json.dumps({"status": "STOP_AND_PRESERVE_EVIDENCE", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
