#!/usr/bin/env python3
"""Assemble a formal liquid-plant capability only from external evidence.

This command is offline.  It neither launches ROS/Gazebo nor computes a
fidelity result.  A missing, changed, development-only, proxy, or unapproved
artifact exits non-zero and creates no capability output.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from scout_liquid_plant.formal_intake import (  # noqa: E402
    FormalEvidenceError,
    assemble_formal_capability,
    write_formal_capability_bundle,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate caller-supplied formal liquid-plant release/fidelity/reference/approval "
            "evidence and write a toolchain-compatible capability binding. "
            "Never launches ROS/Gazebo and never creates a fidelity report."
        )
    )
    parser.add_argument("--intake-request", required=True, help="absolute formal intake-request JSON path")
    parser.add_argument(
        "--intake-request-sha256",
        required=True,
        help="expected lowercase SHA-256 of the exact intake request",
    )
    parser.add_argument(
        "--capability-report-output",
        help="absolute new formal capability-report JSON path (required unless --validate-only)",
    )
    parser.add_argument(
        "--toolchain-binding-output",
        help="absolute new toolchain liquid_plant_capability JSON path (required unless --validate-only)",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate artifacts and print provenance; do not write any output",
    )
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.validate_only:
        if args.capability_report_output is not None or args.toolchain_binding_output is not None:
            print("--validate-only cannot be combined with output paths", file=sys.stderr)
            return 2
    elif not args.capability_report_output or not args.toolchain_binding_output:
        print(
            "--capability-report-output and --toolchain-binding-output are both required unless --validate-only",
            file=sys.stderr,
        )
        return 2
    try:
        assembly = assemble_formal_capability(args.intake_request, args.intake_request_sha256)
        if args.validate_only:
            report = assembly.capability_report
            print(
                json.dumps(
                    {
                        "status": "PASS",
                        "formal": True,
                        "runtime_execution_performed": False,
                        "formal_release_manifest_hash": report["formal_release_manifest_hash"],
                        "fidelity_report_hash": report["fidelity_report_hash"],
                        "external_approval_hash": report["external_approval_hash"],
                        "formal_reference_evidence_set_hash": report["formal_reference_evidence_set_hash"],
                        "formal_plant_signal_evidence_set_hash": report["formal_plant_signal_evidence_set_hash"],
                        "capability_report_payload_hash": report["capability_report_payload_hash"],
                    },
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            return 0
        result = write_formal_capability_bundle(
            assembly,
            args.capability_report_output,
            args.toolchain_binding_output,
        )
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        return 0
    except FormalEvidenceError as exc:
        print(json.dumps({"status": "NO_GO", "error": {"code": exc.code, "message": exc.message}}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
