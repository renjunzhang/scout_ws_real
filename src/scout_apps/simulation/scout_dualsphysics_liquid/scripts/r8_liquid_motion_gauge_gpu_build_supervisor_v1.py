#!/usr/bin/env python3
"""Non-executing lifecycle planner for the future motion-Gauge GPU build."""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Mapping, Sequence

import r8_liquid_motion_gauge_gpu_build_gate_v1 as gate


sys.dont_write_bytecode = True


class SupervisorError(ValueError):
    pass


def build_plan(policy: Mapping[str, Any]) -> dict[str, Any]:
    gate.validate_static_contract(policy)
    if policy["parents"]["motion_patch_v2"]["sha256"] is not None or any(policy["authorization"].values()):
        raise SupervisorError("repository skeleton must remain pending and unauthorized")
    return {
        "status": "NOT_ADMITTED_MOTION_PATCH_V2_PARENT_HASH_REQUIRED",
        "ordered_phases": [
            "RESERVE_CREATE_NEW_RECEIPTS",
            "SOURCE_COPY_352_AND_FULL_INVENTORY",
            "PATCH_EXACT_SIX_AND_VERIFY_346_UNCHANGED",
            "CREATE_84_BYTE_WRAPPER_O_EXCL_AND_VERIFY_353",
            "ONE_MAKE_GXX11_CUDA12_8_SM120_J1_5400S",
            "DISARM_CANDIDATE_0400",
            "READ_ONLY_STATIC_AUDIT_11_PLUS_131_COMMANDS",
            "VERIFY_CANDIDATE_AND_OBJECT_IDENTITIES_UNCHANGED",
            "UNLOAD_EACH_PROFILE_AND_VERIFY_ZERO_RESIDUE",
        ],
        "authorization_breaks": ["PROFILE_QUERY", "PROFILE_LOAD", "SOURCE_COPY",
                                 "PATCH_WRITE", "MAKE_COMPILER", "STATIC_AUDIT"],
        "runtime_attempted": False,
    }


def run_one_shot(*_args: object, **_kwargs: object) -> None:
    raise SupervisorError("NOT_ADMITTED: patch v2 hash, exact profile hashes, and user authorization required")


def self_check() -> dict[str, Any]:
    policy, policy_sha = gate.load_and_validate()
    plan = build_plan(policy)
    return {**plan, "policy_sha256": policy_sha, "files_written": False,
            "external_root_created": False, "sudo_used": False,
            "apparmor_loaded": False, "make_run": False,
            "candidate_executed": False, "gpu_exposed": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "run"))
    args = parser.parse_args(argv)
    try:
        if args.command == "run":
            run_one_shot()
        report = self_check()
    except Exception as exc:
        print(json.dumps({"status": "FAIL_MOTION_GAUGE_GPU_BUILD_SUPERVISOR_V1", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
