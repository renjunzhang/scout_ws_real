#!/usr/bin/env python3
"""Resource-revalidated v3 entry point for the damped-restart experiment.

V2 stopped safely after one transient MemAvailable sample fell 168 MiB below
the original 4 GiB floor while memory PSI remained zero and more than 25 GiB
of swap remained free.  This revision explicitly freezes a 3 GiB hard floor;
all solver, isolation, output-inventory, monitoring and numerical contracts are
unchanged.  Falling below 3 GiB still terminates the owned process group.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import r8_liquid_u3_gpu_damped_restart_runner_v1 as base
import r8_liquid_u3_gpu_damped_restart_runner_v2 as inventory_v2
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_stage4_damped_restart_policy_v2.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_gpu_damped_restart_runner_v3.py"
QC_PATH = SCRIPT_PATH.parent / "r8_liquid_u3_damped_restart_qc_v3.py"
INVENTORY_RUNNER_V2_SHA256 = "fb74d8b155feb5de1a63d73d221ccdfaaaee56842c7e3ae645a31aade7bd3d42"
HARD_MEMORY_FLOOR_BYTES = 3221225472
LEGACY_VALIDATION_FLOOR_BYTES = 4294967296
ORIGINAL_SEMANTIC_VALIDATE = base.semantic_validate


def semantic_validate_v3(policy: dict[str, Any], policy_path: Path) -> dict[str, Any]:
    actual_floors = {
        name: phase["limits"]["minimum_mem_available_bytes"]
        for name, phase in policy["phases"].items()
    }
    if set(actual_floors.values()) != {HARD_MEMORY_FLOOR_BYTES}:
        raise base.DampedRestartRunError(f"v3 resource floor differs: {actual_floors}")
    shadow = copy.deepcopy(policy)
    for phase in shadow["phases"].values():
        phase["limits"]["minimum_mem_available_bytes"] = LEGACY_VALIDATION_FLOOR_BYTES
    result = ORIGINAL_SEMANTIC_VALIDATE(shadow, policy_path)
    result["resource_revision"] = {
        "minimum_mem_available_bytes": HARD_MEMORY_FLOOR_BYTES,
        "reason": "V2_TRANSIENT_4GIB_FLOOR_WITH_ZERO_PSI_AND_25GIB_SWAP_FREE",
        "numerical_contract_changed": False,
    }
    return result


def configure() -> None:
    if legacy.sha256_file(inventory_v2.SCRIPT_PATH, maximum=4 * 1024 * 1024) != INVENTORY_RUNNER_V2_SHA256:
        raise base.DampedRestartRunError("v2 inventory runner dependency identity drifted")
    inventory_v2.configure()
    base.SCRIPT_PATH = SCRIPT_PATH
    base.SCHEMA_PATH = SCHEMA_PATH
    base.TEST_PATH = TEST_PATH
    base.QC_PATH = QC_PATH
    base.semantic_validate = semantic_validate_v3
    legacy.safe_output_inventory = inventory_v2.safe_output_inventory_v2


def main(argv: list[str] | None = None) -> int:
    configure()
    return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
