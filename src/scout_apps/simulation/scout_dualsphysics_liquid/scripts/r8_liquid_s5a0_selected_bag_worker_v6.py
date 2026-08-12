#!/usr/bin/env python3
"""S5A0 worker v6 pinned to reader v3 and canonical topic validation."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Sequence


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v3 as reader_v3
import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate_v1
import r8_liquid_s5a0_selected_bag_intake_gate_v2 as gate_v2
import r8_liquid_s5a0_selected_bag_worker_v4 as worker_v4
import r8_liquid_s5a0_selected_bag_worker_v5 as worker_v5


sys.dont_write_bytecode = True


def sandbox_inspect(
    *, policy_path: Path, schema_path: Path, source_path: Path,
    receipt_path: Path, final_evidence_root: Path, expected_argv_sha256: str,
) -> dict[str, Any]:
    gate_v1.bag_reader = reader_v3
    gate_v1.validate_topic_contract = worker_v5.validate_topic_contract_v3
    return worker_v4.sandbox_inspect(
        policy_path=policy_path,
        schema_path=schema_path,
        source_path=source_path,
        receipt_path=receipt_path,
        final_evidence_root=final_evidence_root,
        expected_argv_sha256=expected_argv_sha256,
    )


def self_check() -> dict[str, Any]:
    base = worker_v5.self_check()
    return {
        **base,
        "status": "PASS_S5A0_SELECTED_BAG_WORKER_V6_STATIC_CONTRACT",
        "reader_revision": "v3",
        "connection_data_topic_required_and_header_matched": True,
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
        worker_v5.WorkerV5Error, worker_v4.WorkerV4Error,
        gate_v1.S5A0GateError, gate_v2.RuntimeReceiptError,
        reader_v3.BagV2Error, OSError, ValueError,
    ) as exc:
        print(json.dumps({
            "status": "STOP_AND_PRESERVE_EVIDENCE", "error": str(exc)
        }, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
