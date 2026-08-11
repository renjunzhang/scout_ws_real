#!/usr/bin/env python3
"""Inventory-corrected v2 entry point for the frozen damped-restart runner.

DualSPHysics writes one documented ``CfgDamping_Scheme.vtk`` whenever a
standard damping configuration is active.  V1 correctly failed closed because
that file was not in its exact root inventory.  This create-new revision makes
one bounded change: the initialization inventory requires that one file, while
the undamped-tail inventory continues to reject it.  Execution, isolation,
resource monitoring, XML semantics, solver argv and receipt production remain
the reviewed v1 implementation.
"""

from __future__ import annotations

import hashlib
import json
import os
import stat
from pathlib import Path
from typing import Any

import r8_liquid_u3_gpu_damped_restart_runner_v1 as base
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_gpu_damped_restart_runner_v2.py"
QC_PATH = SCRIPT_PATH.parent / "r8_liquid_u3_damped_restart_qc_v2.py"
BASE_RUNNER_V1_SHA256 = "07cce967e6ad0b5e2e094fc9803ae6d7c5dd22e8224b19096f3ca78ac3b79ec7"
LEGACY_RUNNER_V4_SHA256 = "bd3a21d818e2b104256462ae6b34a3bf4844ab2298adaa5ecedad1482f43c3c3"


def safe_output_inventory_v2(root: Path, expected: dict[str, Any]) -> dict[str, Any]:
    initialization = expected["part_first"] == 0 and expected["part_last"] == 201 and expected["part_count"] == 202
    exact_root_names = set(legacy.ROOT_OUTPUT_NAMES)
    if initialization:
        exact_root_names.add("CfgDamping_Scheme.vtk")
    entries = {item.name for item in os.scandir(root)}
    if entries != exact_root_names:
        raise legacy.Stage4RunError(f"output root names differ: {sorted(entries)}")
    data = root / "data"
    data_entries = {item.name for item in os.scandir(data)}
    part_names = sorted(name for name in data_entries if legacy.PART_RE.fullmatch(name))
    if data_entries != legacy.STATIC_DATA_NAMES | set(part_names):
        raise legacy.Stage4RunError(f"output data names differ: {sorted(data_entries)}")
    part_indices = [int(legacy.PART_RE.fullmatch(name).group(1)) for name in part_names]
    wanted = list(range(expected["part_first"], expected["part_last"] + 1))
    if part_indices != wanted or len(part_indices) != expected["part_count"]:
        raise legacy.Stage4RunError(
            f"Part range differs: observed={part_indices[:2]}..{part_indices[-2:] if part_indices else []}"
        )
    records: dict[str, dict[str, Any]] = {}
    total = 0
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in dirs:
            path = Path(directory) / name
            if not stat.S_ISDIR(os.lstat(path).st_mode):
                raise legacy.Stage4RunError(f"unsafe output directory: {path}")
        for name in files:
            path = Path(directory) / name
            item = legacy.identity(path)
            relative = str(path.relative_to(root))
            records[relative] = {key: item[key] for key in ("size_bytes", "sha256", "mode", "nlink")}
            total += item["size_bytes"]
    if total > expected["maximum_output_bytes"]:
        raise legacy.Stage4RunError(f"output exceeds bound: {total}")
    if initialization and "CfgDamping_Scheme.vtk" not in records:
        raise legacy.Stage4RunError("initialization damping scheme evidence is absent")
    if not initialization and "CfgDamping_Scheme.vtk" in records:
        raise legacy.Stage4RunError("undamped tail unexpectedly produced damping scheme evidence")
    return {
        "file_count": len(records),
        "part_file_count": len(part_indices),
        "part_first": part_indices[0],
        "part_last": part_indices[-1],
        "total_bytes": total,
        "canonical_sha256": hashlib.sha256(
            json.dumps(records, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest(),
        "files": dict(sorted(records.items())),
    }


def configure() -> None:
    if legacy.sha256_file(base.SCRIPT_PATH, maximum=4 * 1024 * 1024) != BASE_RUNNER_V1_SHA256:
        raise base.DampedRestartRunError("v1 runner dependency identity drifted")
    if legacy.sha256_file(legacy.SCRIPT_PATH, maximum=4 * 1024 * 1024) != LEGACY_RUNNER_V4_SHA256:
        raise base.DampedRestartRunError("legacy v4 runner dependency identity drifted")
    base.SCRIPT_PATH = SCRIPT_PATH
    base.TEST_PATH = TEST_PATH
    base.QC_PATH = QC_PATH
    legacy.safe_output_inventory = safe_output_inventory_v2


def main(argv: list[str] | None = None) -> int:
    configure()
    return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
