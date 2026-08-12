#!/usr/bin/env python3
"""Non-executing S5B0 v2 supervisor skeleton and exact argv/lifecycle planner."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


sys.dont_write_bytecode = True
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))
import r8_liquid_s5b0_replay_admission_gate_v2 as gate


class SupervisorV2Error(ValueError):
    pass


def validate_fresh_targets(paths: Mapping[str, str], exists: Callable[[str], bool]) -> None:
    required = {"partial_root", "final_root", "start_receipt", "final_receipt", "failure_receipt"}
    if set(paths) != required or len(set(paths.values())) != len(required):
        raise SupervisorV2Error("fresh target set aliases or is incomplete")
    for name, value in paths.items():
        if not value.startswith("/") or str(Path(value)) != value or exists(value):
            raise SupervisorV2Error(f"non-fresh/unsafe target: {name}")


def build_solver_argv(*, staged_candidate: str, staged_case_prefix: str,
                      output_root: str, restart_root: str, solver_path_last_t_s: float) -> list[str]:
    paths = [staged_candidate, staged_case_prefix, output_root, restart_root]
    if any(not value.startswith("/") or str(Path(value)) != value for value in paths):
        raise SupervisorV2Error("solver path is not exact absolute")
    if "/source/" in staged_candidate or not staged_candidate.endswith("/runtime/candidate"):
        raise SupervisorV2Error("only staged candidate may be executed")
    last_t = gate._finite(solver_path_last_t_s, "solver_path_last_t_s")
    if last_t <= 1.0:
        raise SupervisorV2Error("complete motion plus included solver tail is missing")
    tmax = 45.05001991890928 + last_t
    argv = [staged_candidate, staged_case_prefix, output_root, "-gpu:0", *gate.COMMON_FLAGS,
            "-partbegin:901:901", restart_root, f"-tmax:{format(tmax, '.15g')}", "-tout:0.05"]
    if any(token == "-j" or token.startswith("-j:") for token in argv):
        raise SupervisorV2Error("-j is irrelevant/forbidden for GPU")
    if argv.count("-gpu:0") != 1 or any(argv.count(flag) != 1 for flag in gate.COMMON_FLAGS):
        raise SupervisorV2Error("GPU/Stage4 common flag cardinality differs")
    return argv


def build_static_execution_plan(policy: Mapping[str, Any]) -> dict[str, Any]:
    if policy["parent_transfer"]["finalized"] or any(policy["authorization"].values()):
        raise SupervisorV2Error("repository template must remain unfinalized and unauthorized")
    return {
        "status": "NOT_ADMITTED_S5A1_FINALIZED_REQUIRED",
        "ordered_lifecycle": [
            "VERIFY_FINALIZED_TRANSFER_AND_C1_PARENTS",
            "RESERVE_START_FINAL_FAILURE_RECEIPTS_O_EXCL",
            "CREATE_FRESH_STAGING_AND_PARTIAL_ROOTS",
            "VERIFY_DYNAMIC_FREE_VRAM_AND_HEADROOM",
            "LOAD_EXACT_PROFILE_AFTER_NEW_AUTHORIZATION",
            "SPAWN_STAGED_CANDIDATE_WITH_NEW_PGID",
            "MONITOR_EVERY_10_TO_30_SECONDS",
            "ON_FAILURE_KILL_OWN_PGID_AND_PRESERVE_PARTIAL",
            "FINALLY_UNLOAD_PROFILE_AND_VERIFY_ZERO_RESIDUE",
            "VALIDATE_BOUNDARY_GAUGE_PARTICLES_XID_AND_HASHES",
            "RENAME_NOREPLACE_FINAL_RESULT",
        ],
        "spawn_contract": {"start_new_session": True, "owned_pgid": True,
                           "wall_timeout_seconds": 5400, "kill_sequence": ["SIGTERM", "SIGKILL"],
                           "monitor_interval_min_seconds": 10, "monitor_interval_max_seconds": 30},
        "candidate_contract": {"source_executed": False, "staged_executed_only_after_authorization": True},
        "profile_contract": {"load_authorized": False, "device_count": 3,
                             "uvm_tools_default": False, "only_host_writable_bind": "OUTPUT_ROOT"},
        "runtime_attempted": False,
    }


def run_one_shot(*_args: object, **_kwargs: object) -> None:
    """Intentional hard stop until a create-new finalized policy and authorization exist."""
    raise SupervisorV2Error("NOT_ADMITTED: static v2 template has no replay identity or execution authorization")


def self_check() -> dict[str, Any]:
    policy, policy_sha = gate.load_and_validate_policy()
    plan = build_static_execution_plan(policy)
    argv = build_solver_argv(staged_candidate="/fixture/staging/runtime/candidate",
                             staged_case_prefix="/fixture/staging/case/C1M_case",
                             output_root="/fixture/output", restart_root="/fixture/staging/restart",
                             solver_path_last_t_s=2.0)
    validate_fresh_targets({
        "partial_root": "/fixture/out/replay.partial", "final_root": "/fixture/out/replay",
        "start_receipt": "/fixture/audit/replay.start.json", "final_receipt": "/fixture/audit/replay.final.json",
        "failure_receipt": "/fixture/audit/replay.failure.json",
    }, lambda _path: False)
    return {**plan, "policy_sha256": policy_sha, "solver_argv": argv,
            "stage4_common_flag_count": len(gate.COMMON_FLAGS), "files_written": False,
            "candidate_executed": False, "gpu_exposed": False, "profile_loaded": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_SUPERVISOR_V2", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
