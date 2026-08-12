#!/usr/bin/env python3
"""S5A0 sandbox worker v2 with mandatory runtime receipt validation."""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator

import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate_v1


sys.dont_write_bytecode = True

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
INNER_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_s5a0_selected_bag_receipt_v1.json"
INNER_RECEIPT_NAME = "selected_bag_inner_receipt_v1.json"


class RuntimeReceiptError(ValueError):
    """The worker could not prove a closed, valid inner receipt."""


def validate_runtime_receipt(receipt: Mapping[str, Any], schema: Mapping[str, Any]) -> None:
    """Validate the exact closed schema before any receipt write."""

    gate_v1.assert_deep_closed(schema)
    Draft202012Validator.check_schema(schema)
    errors = sorted(
        Draft202012Validator(schema).iter_errors(receipt),
        key=lambda error: tuple(str(part) for part in error.absolute_path),
    )
    if errors:
        first = errors[0]
        location = "/".join(str(part) for part in first.absolute_path) or "$"
        raise RuntimeReceiptError(f"inner receipt schema violation at {location}: {first.message}")


def sandbox_inspect_v2(
    *,
    policy_path: Path,
    schema_path: Path,
    source_path: Path,
    receipt_path: Path,
    final_evidence_root: Path,
    expected_argv_sha256: str,
) -> dict[str, Any]:
    if not gate_v1._is_lower_hex(expected_argv_sha256, 64):
        raise RuntimeReceiptError("expected argv SHA-256 is invalid")
    policy, policy_sha256 = gate_v1.load_policy(policy_path)
    gate_v1.validate_frozen_policy(policy)
    if str(source_path) != policy["sandbox_contract"]["source_visible_path"]:
        raise RuntimeReceiptError("sandbox source path differs from the frozen visible path")
    if receipt_path != Path("/evidence") / INNER_RECEIPT_NAME:
        raise RuntimeReceiptError("inner receipt path is not the exact dedicated evidence path")
    if not final_evidence_root.is_absolute() or final_evidence_root.name.endswith(".partial"):
        raise RuntimeReceiptError("final evidence root is invalid")
    schema, _ = gate_v1._read_bounded_json(
        schema_path, gate_v1.MAXIMUM_SCHEMA_BYTES, "sandbox inner receipt schema"
    )
    runtime_policy = copy.deepcopy(policy)
    runtime_policy["sandbox_contract"]["audit_root"] = str(final_evidence_root)
    gate_v1.validate_policy(runtime_policy)
    reader_hash = gate_v1._sha256_file_bounded(
        Path("/app/r8_liquid_ros1_bag_v2_reader_v1.py"),
        2 * 1024 * 1024,
        "sandbox ROS1 reader",
    )
    if reader_hash != policy["parents"]["reader_sha256"]:
        raise RuntimeReceiptError("sandbox ROS1 reader hash differs")
    mountinfo = Path("/proc/self/mountinfo").read_text(encoding="utf-8")
    statvfs_flags = os.statvfs(source_path).f_flag
    host_receipt_path = final_evidence_root / INNER_RECEIPT_NAME
    receipt = gate_v1.inspect_selected(
        runtime_policy,
        source_path=source_path,
        receipt_path=host_receipt_path,
        mountinfo_text=mountinfo,
        statvfs_flags=statvfs_flags,
        policy_sha256=policy_sha256,
        captured_at_utc=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        snapshot_root=source_path.parent,
        source_is_sandbox_visible_path=True,
    )
    receipt["sandbox"]["bwrap_argv_sha256"] = expected_argv_sha256
    receipt["output"]["receipt_path"] = str(host_receipt_path)
    validate_runtime_receipt(receipt, schema)
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
    Draft202012Validator.check_schema(schema)
    return {
        "status": "PASS_S5A0_SANDBOX_WORKER_V2_STATIC_CONTRACT",
        "runtime_schema_validation_required": True,
        "inner_receipt_name": INNER_RECEIPT_NAME,
        "real_bag_opened": False,
        "bwrap_executed": False,
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
            result = sandbox_inspect_v2(
                policy_path=Path(arguments.policy),
                schema_path=Path(arguments.schema),
                source_path=Path(arguments.source),
                receipt_path=Path(arguments.receipt),
                final_evidence_root=Path(arguments.final_evidence_root),
                expected_argv_sha256=arguments.expected_argv_sha256,
            )
    except (gate_v1.S5A0GateError, RuntimeReceiptError, OSError, ValueError) as exc:
        print(json.dumps({"status": "STOP_AND_PRESERVE_EVIDENCE", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "INNER_RECEIPT_NAME",
    "RuntimeReceiptError",
    "sandbox_inspect_v2",
    "self_check",
    "validate_runtime_receipt",
]
