#!/usr/bin/env python3
"""S5A0 worker v5 with canonical same-topic definition validation.

ROS publishers may register multiple connections for one topic with message
definitions differing only by terminal CR/LF bytes.  Reader v2 preserves every
raw definition hash and separately computes a canonical hash that removes only
those terminal bytes.  This worker uses the canonical identity solely for the
same-topic conflict decision; the receipt retains all raw witnesses.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate_v1
import r8_liquid_s5a0_selected_bag_intake_gate_v2 as gate_v2
import r8_liquid_s5a0_selected_bag_worker_v4 as worker_v4


sys.dont_write_bytecode = True
_BASE_VALIDATE_TOPIC_CONTRACT = gate_v1.validate_topic_contract


class WorkerV5Error(ValueError):
    """Canonical definition or isolated worker validation failed closed."""


def validate_topic_contract_v3(
    policy: Mapping[str, Any], bag: Mapping[str, Any]
) -> dict[str, Any]:
    """Validate with canonical definitions while retaining raw receipt data."""

    normalized = copy.deepcopy(dict(bag))
    original_connections = bag.get("connections")
    normalized_connections = normalized.get("connections")
    if not isinstance(original_connections, list) or not isinstance(normalized_connections, list):
        raise WorkerV5Error("connection census is absent")
    if len(original_connections) != len(normalized_connections):
        raise WorkerV5Error("connection census copy differs")

    canonical_by_topic: dict[str, set[tuple[str, str, str]]] = {}
    raw_variant_count_by_topic: dict[str, set[str]] = {}
    for original, projected in zip(original_connections, normalized_connections):
        if not isinstance(original, Mapping) or not isinstance(projected, dict):
            raise WorkerV5Error("connection row is malformed")
        topic = original.get("topic")
        raw_sha = original.get("message_definition_sha256")
        canonical_sha = original.get("canonical_message_definition_sha256")
        raw_size = original.get("message_definition_size_bytes")
        canonical_size = original.get("canonical_message_definition_size_bytes")
        if (
            not isinstance(topic, str)
            or not gate_v1._is_lower_hex(raw_sha, 64)
            or not gate_v1._is_lower_hex(canonical_sha, 64)
            or not isinstance(raw_size, int)
            or not isinstance(canonical_size, int)
            or raw_size <= 0
            or canonical_size <= 0
            or canonical_size > raw_size
        ):
            raise WorkerV5Error("raw/canonical message-definition witness is invalid")
        identity = (
            str(original.get("type")), str(original.get("md5sum")), str(canonical_sha)
        )
        canonical_by_topic.setdefault(topic, set()).add(identity)
        raw_variant_count_by_topic.setdefault(topic, set()).add(str(raw_sha))
        projected["message_definition_sha256"] = canonical_sha

    conflicts = sorted(topic for topic, identities in canonical_by_topic.items() if len(identities) != 1)
    if conflicts:
        raise WorkerV5Error(f"canonical same-topic definitions conflict: {', '.join(conflicts)}")
    result = _BASE_VALIDATE_TOPIC_CONTRACT(policy, normalized)
    # A raw variant is allowed only because reader v2 proved that all parsed
    # type/MD5/canonical identities agree.  No whitespace other than terminal
    # CR/LF is removed by that reader.
    if result.get("validated") is not True:
        raise WorkerV5Error("base topic contract did not validate")
    return result


def sandbox_inspect(
    *, policy_path: Path, schema_path: Path, source_path: Path,
    receipt_path: Path, final_evidence_root: Path, expected_argv_sha256: str,
) -> dict[str, Any]:
    gate_v1.validate_topic_contract = validate_topic_contract_v3
    return worker_v4.sandbox_inspect(
        policy_path=policy_path,
        schema_path=schema_path,
        source_path=source_path,
        receipt_path=receipt_path,
        final_evidence_root=final_evidence_root,
        expected_argv_sha256=expected_argv_sha256,
    )


def self_check() -> dict[str, Any]:
    base = worker_v4.self_check()
    return {
        **base,
        "status": "PASS_S5A0_SELECTED_BAG_WORKER_V5_STATIC_CONTRACT",
        "same_topic_definition_identity": "TYPE_MD5_CANONICAL_TRAILING_CRLF_ONLY",
        "raw_definition_witnesses_preserved": True,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("self-check")
    worker = commands.add_parser("sandbox-inspect")
    for name in (
        "policy", "schema", "source", "receipt", "final-evidence-root",
        "expected-argv-sha256",
    ):
        worker.add_argument(f"--{name}", required=True)
    args = parser.parse_args(argv)
    try:
        result = self_check() if args.command == "self-check" else sandbox_inspect(
            policy_path=Path(args.policy), schema_path=Path(args.schema),
            source_path=Path(args.source), receipt_path=Path(args.receipt),
            final_evidence_root=Path(args.final_evidence_root),
            expected_argv_sha256=args.expected_argv_sha256,
        )
    except (
        WorkerV5Error, worker_v4.WorkerV4Error, gate_v1.S5A0GateError,
        gate_v2.RuntimeReceiptError, OSError, ValueError,
    ) as exc:
        print(json.dumps({
            "status": "STOP_AND_PRESERVE_EVIDENCE", "error": str(exc)
        }, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
