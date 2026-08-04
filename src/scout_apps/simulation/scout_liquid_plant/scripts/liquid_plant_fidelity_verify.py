#!/usr/bin/env python3
"""Command-line wrapper for the offline development plant-fidelity verifier.

This command never starts ROS/Gazebo and never returns a formal PASS.  It
always writes a schema-valid report when the output location is writable; a
missing real/frozen reference is represented by a deterministic ``NO_GO``
report and a non-zero exit status.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from scout_liquid_plant.fidelity import (  # noqa: E402
    FidelityValidationError,
    verify_development_fidelity,
    write_report,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Offline hash-bound development comparison of independent liquid-plant "
            "signals against frozen real references. Never emits a formal PASS."
        )
    )
    parser.add_argument("--comparison-manifest", required=True, help="absolute comparison manifest JSON path")
    parser.add_argument("--comparison-manifest-sha256", required=True, help="expected manifest SHA-256")
    parser.add_argument("--threshold-policy", required=True, help="absolute threshold policy JSON path")
    parser.add_argument("--threshold-policy-sha256", required=True, help="expected threshold policy SHA-256")
    parser.add_argument("--output", required=True, help="absolute output report JSON path")
    parser.add_argument("--overwrite", action="store_true", help="allow replacement of an existing output report")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report = verify_development_fidelity(
        comparison_manifest_path=args.comparison_manifest,
        comparison_manifest_sha256=args.comparison_manifest_sha256,
        threshold_policy_path=args.threshold_policy,
        threshold_policy_sha256=args.threshold_policy_sha256,
    )
    try:
        write_report(Path(args.output), report, overwrite=args.overwrite)
    except FidelityValidationError as exc:
        print(json.dumps({"status": "NO_GO", "error": {"code": exc.code, "message": exc.message}}, ensure_ascii=False), file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "report": str(Path(args.output).resolve()),
                "status": report["status"],
                "formal": report["formal"],
                "validation_dimensions": report["validation_dimensions"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0 if report["status"] == "DEVELOPMENT_METRICS_COMPLETE_NOT_FORMAL" else 2


if __name__ == "__main__":
    raise SystemExit(main())
