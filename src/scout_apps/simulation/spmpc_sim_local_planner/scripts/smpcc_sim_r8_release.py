#!/usr/bin/env python3
"""R8 source-separated freeze/master/GO-evidence entry point.

This is intentionally narrower than a matrix runner.  It can create the
source-separation GO receipt by actually running the sim-owned build/isolation
tests, or inspect an already-finalized R8 freeze/master pair.  It cannot
select a path, controller condition, profile, seed, liquid truth model, or
planned row, and it never starts ROS/Gazebo.

A passing ``gate`` means only that the R8 *source identity* is internally
bound.  Timing, independent-liquid-plant, matrix, and all other formal
admission gates remain separate requirements.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import stat
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional


SCRIPT_DIR = Path(__file__).resolve().parent


class R8ReleaseError(RuntimeError):
    """An R8 source evidence error that must remain a NO-GO."""


def _load_source_gate():
    path = SCRIPT_DIR / "smpcc_sim_source_separation.py"
    spec = importlib.util.spec_from_file_location("smpcc_sim_source_separation_r8_release", path)
    if spec is None or spec.loader is None:
        raise R8ReleaseError(f"cannot load R8 source-separation gate: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


source_gate = _load_source_gate()
WRITE_BITS = stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH


def _read_immutable_json(path: Path, label: str) -> Mapping[str, Any]:
    path = Path(path)
    if not path.is_absolute():
        raise R8ReleaseError(f"{label} must be an absolute path")
    if path.is_symlink() or not path.is_file():
        raise R8ReleaseError(f"{label} must be an existing ordinary file: {path}")
    if stat.S_IMODE(path.stat().st_mode) & WRITE_BITS:
        raise R8ReleaseError(f"{label} must be read-only: {path}")
    try:
        with path.open(encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise R8ReleaseError(f"cannot read {label}: {exc}") from exc
    if not isinstance(value, Mapping):
        raise R8ReleaseError(f"{label} must be a JSON object")
    return value


def gate(freeze_path: Path, master_path: Path) -> Mapping[str, Any]:
    """Validate finalized R8 source evidence without starting a row."""

    freeze = _read_immutable_json(freeze_path, "R8 freeze")
    master = _read_immutable_json(master_path, "R8 master")
    try:
        binding = source_gate.require_execution_identity(freeze, master)
    except source_gate.SourceSeparationError as exc:
        raise R8ReleaseError(str(exc)) from exc
    return {
        "status": "PASS_R8_SOURCE_SEPARATION_ONLY_NOT_MATRIX_GO",
        "formal": False,
        "matrix_execution_authorized": False,
        "physical_primary_eligible": False,
        "release_id": source_gate.SOURCE_SEPARATED_RELEASE_ID,
        "target_id": source_gate.SOURCE_SEPARATED_TARGET_ID,
        "source_separation_hash": source_gate.canonical_hash(binding),
        "execution_artifact_registry_hash": binding[
            "execution_artifact_registry_hash"
        ],
        "source_separation_go_receipt": binding[
            "source_separation_go_receipt"
        ],
        "no_go_remaining": [
            "R8 source gate is not the full matrix formal gate",
            "independent liquid-plant/fidelity/firewall evidence is still required",
            "timing admission and finalized R8 matrix/master evidence are still required",
        ],
    }


def create_go_receipt(
    freeze_path: Path, master_path: Path, output: Path
) -> Mapping[str, Any]:
    """Run the bounded sim-owned R8 source checks and create one receipt.

    The caller must later attach the immutable descriptor to a new finalized
    freeze and cross-bind its semantic receipt hash into the matching master.
    This command never edits those input files in place.
    """

    freeze = _read_immutable_json(freeze_path, "R8 pre-GO freeze")
    master = _read_immutable_json(master_path, "R8 pre-GO master")
    try:
        receipt = source_gate.create_r8_go_receipt(freeze, master, Path(output))
    except source_gate.SourceSeparationError as exc:
        raise R8ReleaseError(str(exc)) from exc
    return {
        "status": "R8_SOURCE_GO_RECEIPT_CREATED_NOT_FINALIZED",
        "formal": False,
        "matrix_execution_authorized": False,
        "physical_primary_eligible": False,
        "release_id": source_gate.SOURCE_SEPARATED_RELEASE_ID,
        "go_receipt_path": receipt["receipt_path"],
        "go_receipt_file_hash": receipt["receipt_file_hash"],
        "go_receipt_hash": receipt["go_receipt_hash"],
        "next_required_binding": (
            "attach this read-only receipt descriptor to a new immutable R8 "
            "freeze and cross-bind go_receipt_hash in its immutable master"
        ),
    }


def _emit(value: Mapping[str, Any]) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    gate_parser = sub.add_parser("gate", help="validate finalized R8 source evidence")
    gate_parser.add_argument("--freeze", required=True, type=Path)
    gate_parser.add_argument("--master", required=True, type=Path)
    gate_parser.set_defaults(func=lambda args: gate(args.freeze, args.master))
    create_parser = sub.add_parser("create-go-receipt", help="run sim-owned R8 source admission checks")
    create_parser.add_argument("--freeze", required=True, type=Path)
    create_parser.add_argument("--master", required=True, type=Path)
    create_parser.add_argument("--output", required=True, type=Path)
    create_parser.set_defaults(
        func=lambda args: create_go_receipt(args.freeze, args.master, args.output)
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    try:
        args = _parser().parse_args(argv)
        result = args.func(args)
        _emit(result)
        return 0
    except (R8ReleaseError, OSError, ValueError) as exc:
        _emit(
            {
                "status": "NO_GO",
                "formal": False,
                "matrix_execution_authorized": False,
                "physical_primary_eligible": False,
                "errors": [str(exc)],
            }
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
