#!/usr/bin/env python3
"""S5A1 v2 extractor/worker guards and parent-identity validators."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


sys.dont_write_bytecode = True
EXPECTED_TOKEN = "s5a1_primary_bsmooth_b01_20260811T193016Z_v2:sandbox-worker"
HOST_PATH = "/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"
HOST_SHA = "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07"
HOST_SIZE = 13996902
GUEST_PATH = "/selected/capture.bag"


class WorkerV2Error(ValueError):
    pass


def validate_exact_preroll(window_start_s: object, first_effective_s: object, required_s: float = 1.0) -> None:
    values = (window_start_s, first_effective_s, required_s)
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) for value in values):
        raise WorkerV2Error("pre-roll values must be finite numbers")
    observed = float(first_effective_s) - float(window_start_s)
    if not math.isclose(observed, float(required_s), rel_tol=0.0, abs_tol=1e-9):
        raise WorkerV2Error(f"pre-roll must be exactly {required_s}, got {observed}")


def _identity(value: Mapping[str, Any], *, path: str, mode: str) -> None:
    if value.get("path") != path or value.get("mode") != mode or value.get("size_bytes") != HOST_SIZE or value.get("sha256") != HOST_SHA or value.get("nlink") != 1:
        raise WorkerV2Error(f"identity differs for {path}")


def validate_s5a0_v2_parent(outer: Mapping[str, Any], inner: Mapping[str, Any]) -> dict[str, Any]:
    if outer.get("status") != "PASS_S5A0_PRIMARY_ONE_SHOT_DEVELOPMENT_ONLY" or outer.get("failure") is not None:
        raise WorkerV2Error("S5A0 v2 outer receipt is not PASS")
    host = outer.get("host_source", {})
    _identity(host.get("file_before", {}), path=HOST_PATH, mode="0755")
    _identity(host.get("file_after", {}), path=HOST_PATH, mode="0755")
    if any(host.get(name) is not True for name in ("path_unchanged", "mount_unchanged", "file_unchanged", "root_unchanged")):
        raise WorkerV2Error("outer host identity is not unchanged")
    if inner.get("status") != "S5A0_SELECTED_R7_BAG_SEALED_DEVELOPMENT_ONLY":
        raise WorkerV2Error("S5A0 v2 inner receipt is not PASS")
    guest = inner.get("source", {}).get("selected_after", {})
    _identity(guest, path=GUEST_PATH, mode="0400")
    provenance = inner.get("host_source_provenance", {})
    if provenance.get("absolute_path") != HOST_PATH or provenance.get("expected_mode") != "0755" or provenance.get("guest_copy_mode") != "0400" or provenance.get("guest_copy_distinct_inode") is not True:
        raise WorkerV2Error("host/guest provenance bridge differs")
    return {"host_sha256": HOST_SHA, "host_mode": "0755", "guest_mode": "0400", "same_hash_and_size": True}


def synthetic_self_check() -> dict[str, Any]:
    identity_host = {"path": HOST_PATH, "mode": "0755", "size_bytes": HOST_SIZE, "sha256": HOST_SHA, "nlink": 1}
    identity_guest = {"path": GUEST_PATH, "mode": "0400", "size_bytes": HOST_SIZE, "sha256": HOST_SHA, "nlink": 1}
    outer = {"status": "PASS_S5A0_PRIMARY_ONE_SHOT_DEVELOPMENT_ONLY", "failure": None, "host_source": {
        "file_before": identity_host, "file_after": dict(identity_host), "path_unchanged": True,
        "mount_unchanged": True, "file_unchanged": True, "root_unchanged": True,
    }}
    inner = {"status": "S5A0_SELECTED_R7_BAG_SEALED_DEVELOPMENT_ONLY", "source": {"selected_after": identity_guest},
             "host_source_provenance": {"absolute_path": HOST_PATH, "expected_mode": "0755", "guest_copy_mode": "0400", "guest_copy_distinct_inode": True}}
    validate_s5a0_v2_parent(outer, inner)
    validate_exact_preroll(10.0, 11.0)
    return {"status": "PASS_S5A1_V2_WORKER_SYNTHETIC_SELF_CHECK", "real_bag_read": False, "external_write": False, "process_run": False}


def sandbox_worker(token: str) -> dict[str, Any]:
    if os.environ.get("S5A1_V2_SANDBOX") != "1" or token != EXPECTED_TOKEN:
        raise WorkerV2Error("sandbox worker is not directly host-runnable")
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups():
        raise WorkerV2Error("sandbox worker identity must be uid/gid 1000 with zero supplementary groups")
    return {"status": "SANDBOX_ENTRY_GUARDS_PASS_INPUT_PROCESSING_REQUIRES_AUTHORIZED_SUPERVISOR"}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("self-check")
    worker = commands.add_parser("sandbox-worker")
    worker.add_argument("--token", required=True)
    args = parser.parse_args(argv)
    try:
        result = synthetic_self_check() if args.command == "self-check" else sandbox_worker(args.token)
    except WorkerV2Error as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
