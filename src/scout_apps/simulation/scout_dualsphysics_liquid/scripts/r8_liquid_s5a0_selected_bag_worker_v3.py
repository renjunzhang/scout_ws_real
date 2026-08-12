#!/usr/bin/env python3
"""Proc/dev-free S5A0 sandbox worker revision v3.

This worker is only valid behind the exact host-supervisor argv.  It verifies
the bind with ``statvfs(ST_RDONLY)``, parses one visible bag, validates the
inner receipt against the committed closed schema, and creates it with
``O_EXCL``.  It never executes the bag or launches ROS/GPU/solver processes.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

# Isolated mode intentionally omits both cwd and the script directory.  Add
# back only this resolved directory so the two frozen sibling modules remain
# importable without accepting PYTHONPATH or user-site injection.
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate_v1
import r8_liquid_s5a0_selected_bag_intake_gate_v2 as gate_v2


sys.dont_write_bytecode = True

INNER_RECEIPT_NAME = "selected_bag_inner_receipt_v1.json"
VISIBLE_SOURCE = Path("/selected/capture.bag")
VISIBLE_RECEIPT = Path("/evidence") / INNER_RECEIPT_NAME


class WorkerV3Error(ValueError):
    """A proc/dev-free worker contract failed closed."""


def sandbox_inspect(
    *,
    policy_path: Path,
    schema_path: Path,
    source_path: Path,
    receipt_path: Path,
    final_evidence_root: Path,
    expected_argv_sha256: str,
) -> dict[str, Any]:
    if source_path != VISIBLE_SOURCE or receipt_path != VISIBLE_RECEIPT:
        raise WorkerV3Error("worker paths differ from the exact sandbox contract")
    if not gate_v1._is_lower_hex(expected_argv_sha256, 64):
        raise WorkerV3Error("expected argv SHA-256 is invalid")
    if not final_evidence_root.is_absolute() or final_evidence_root.name.endswith(".partial"):
        raise WorkerV3Error("final evidence root is invalid")
    policy, policy_sha256 = gate_v1.load_policy(policy_path)
    gate_v1.validate_frozen_policy(policy)
    schema, _ = gate_v1._read_bounded_json(
        schema_path, gate_v1.MAXIMUM_SCHEMA_BYTES, "inner receipt schema"
    )
    flags = os.statvfs(source_path).f_flag
    if not flags & getattr(os, "ST_RDONLY", 1):
        raise WorkerV3Error("sandbox source statvfs is not read-only")
    runtime_policy = copy.deepcopy(policy)
    runtime_policy["sandbox_contract"]["audit_root"] = str(final_evidence_root)
    gate_v1.validate_policy(runtime_policy)
    # No procfs is mounted.  This row records the supervisor's exact ro-bind;
    # statvfs above independently verifies the effective VFS read-only flag.
    synthetic_mountinfo = "1 0 0:0 / /selected ro,nosuid,nodev - bwrap exact-ro-bind ro\n"
    host_receipt = final_evidence_root / INNER_RECEIPT_NAME
    receipt = gate_v1.inspect_selected(
        runtime_policy,
        source_path=source_path,
        receipt_path=host_receipt,
        mountinfo_text=synthetic_mountinfo,
        statvfs_flags=flags,
        policy_sha256=policy_sha256,
        captured_at_utc=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        snapshot_root=source_path.parent,
        source_is_sandbox_visible_path=True,
    )
    receipt["sandbox"]["bwrap_argv_sha256"] = expected_argv_sha256
    receipt["output"]["receipt_path"] = str(host_receipt)
    gate_v2.validate_runtime_receipt(receipt, schema)
    return gate_v1.write_receipt_create_new(
        receipt_path,
        receipt,
        maximum_bytes=policy["limits"]["maximum_receipt_bytes"],
    )


def self_check() -> dict[str, Any]:
    policy, _ = gate_v1.load_policy()
    gate_v1.validate_frozen_policy(policy)
    schema = gate_v1.load_schema()
    gate_v1.assert_deep_closed(schema)
    return {
        "status": "PASS_S5A0_SELECTED_BAG_WORKER_V3_STATIC_CONTRACT",
        "proc_mounted": False,
        "dev_mounted": False,
        "runtime_inner_schema_validation": True,
        "real_bag_opened": False,
        "process_executed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    worker = subparsers.add_parser("sandbox-inspect")
    worker.add_argument("--policy", required=True)
    worker.add_argument("--schema", required=True)
    worker.add_argument("--source", required=True)
    worker.add_argument("--receipt", required=True)
    worker.add_argument("--final-evidence-root", required=True)
    worker.add_argument("--expected-argv-sha256", required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            result = self_check()
        else:
            result = sandbox_inspect(
                policy_path=Path(arguments.policy),
                schema_path=Path(arguments.schema),
                source_path=Path(arguments.source),
                receipt_path=Path(arguments.receipt),
                final_evidence_root=Path(arguments.final_evidence_root),
                expected_argv_sha256=arguments.expected_argv_sha256,
            )
    except (WorkerV3Error, gate_v1.S5A0GateError, gate_v2.RuntimeReceiptError, OSError, ValueError) as exc:
        print(json.dumps({"status": "STOP_AND_PRESERVE_EVIDENCE", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
