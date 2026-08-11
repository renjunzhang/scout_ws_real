#!/usr/bin/env python3
"""One-shot 35.05--36.05 s GPU settling extension from cold-A.

This development-only runner changes exactly the simulation end time while
keeping the accepted ``CFL=0.1``, ``Shifting=None``, ``DDT2(0.1)`` and
artificial viscosity 0.2 contract.  The inherited stage-4 runner supplies
O_EXCL receipts, root-only staging, uid/gid 1000 execution, a network namespace,
one writable output bind, resource limits and immutable input verification.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any

import r8_liquid_u3_gpu_stage4_runner_v4 as base


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p2_settle_extension_policy_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_gpu_viscoart0p2_settle_extension_runner_v1.py"
QC_PATH = SCRIPT_PATH.parent / "r8_liquid_u3_viscoart0p2_settle_extension_qc_v1.py"
QC_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p2_settle_extension_qc_v1.json"
QC_TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p2_settle_extension_qc_v1.py"
POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_viscoart0p2_settle_extend_from_0701_20260811T092059Z_v9.json"
RUN_ID = "u3_c1m_gpu_viscoart0p2_settle_extend_from_0701_20260811T092059Z_v9"
PRIOR_POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_viscoart0p2_cold_a_20260811T080852Z_v8.json"
PRIOR_POLICY_SHA256 = "860c23f5d5bae5b93dc42ffb4064d2e0ee1523a45bc73c5473f6ba5c01dd3c29"
PRIOR_QC_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p2_cold_a_20260811T080852Z_v8.qc_v4.json")
PRIOR_QC_SHA256 = "b3b96fec53a075645775d6b958c7d8632624efde3b28b5539872dbb749ec8493"
PRIOR_FINAL_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p2_cold_a_20260811T080852Z_v8.final.json")
PRIOR_FINAL_SHA256 = "1800a0a4080f9d1d1204d3d6cab6540737a8518bfeaab5da2e55c8e2f6cbad87"
RESTART_PART_PATH = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_viscoart0p2_cold_a_20260811T080852Z_v8.partial/output/data/Part_0701.bi4")
RESTART_PART_SHA256 = "8bbbcf1c4f8e8611d68ecd14fbe4f7617b0df3f5cdd5df68dee665d0153951ba"
RESTART_HEAD_PATH = RESTART_PART_PATH.parent / "Part_Head.ibi4"
RESTART_HEAD_SHA256 = "be5c93d2a797d65e6d349faa6f52b967dbd3aefc7694003974daf4761aa7fce6"
EXPECTED_OUTPUT = {
    "start_time_s": 35.05,
    "end_time_s": 36.05,
    "output_period_s": 0.05,
    "part_first": 701,
    "part_last": 721,
    "part_count": 21,
    "maximum_output_bytes": 67108864,
}
EXPECTED_LIMITS = {
    "wall_timeout_seconds": 600,
    "kill_after_seconds": 10,
    "monitor_interval_seconds": 10,
    "minimum_mem_available_bytes": 4294967296,
    "maximum_output_bytes": 67108864,
}
EXPECTED_DELTA = {
    "parameter": "SIMULATION_END_TIME_S",
    "baseline": "35.05",
    "candidate": "36.05",
    "other_numerical_parameters_changed": False,
}


class SettleExtensionRunError(base.Stage4RunError):
    """Unsafe, drifted or non-exclusive settling-extension contract."""


def exact_solver_argv() -> list[str]:
    return [
        "/runtime/DualSPHysics5.4_linux64",
        "/case/C1M_zero",
        "/output",
        "-gpu:0",
        "-ompthreads:1",
        "-stable:1",
        "-vres:0",
        "-cellmode:full",
        "-cfl:0.1",
        "-partbegin:701:701",
        "/restart",
        "-shifting:none",
        "-viscoart:0.2",
        "-tmax:36.05",
        "-tout:0.05",
        "-sv:binx,info",
        "-svres:1",
        "-svtimers:0",
        "-svdomainvtk:0",
        "-saveposdouble:1",
        "-nortimes:1",
        "-createdirs:1",
        "-csvsep:0",
    ]


def _exact_run_paths() -> dict[str, str]:
    attempt = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs") / f"{RUN_ID}.partial"
    audit = Path("/home/zrj/scout_liquid_lab/audits")
    return {
        "attempt_root": str(attempt),
        "output_root": str(attempt / "output"),
        "start_receipt": str(audit / f"{RUN_ID}.start.json"),
        "final_receipt": str(audit / f"{RUN_ID}.final.json"),
        "failure_receipt": str(audit / f"{RUN_ID}.failure.json"),
        "stdout_log": str(audit / f"{RUN_ID}.stdout.log"),
        "stderr_log": str(audit / f"{RUN_ID}.stderr.log"),
        "resource_log": str(audit / f"{RUN_ID}.resources.jsonl"),
    }


def semantic_validate(policy: dict[str, Any], policy_path: Path) -> None:
    if policy_path != POLICY_PATH:
        raise SettleExtensionRunError("policy path is not the frozen v9 path")
    if policy["run_id"] != RUN_ID or policy["purpose"] != "viscoart0p2_settling_extension":
        raise SettleExtensionRunError("run identity or purpose differs")
    if policy["run"] != _exact_run_paths():
        raise SettleExtensionRunError("run and audit paths differ")
    wanted_stage = Path("/var/tmp") / f"r8-liquid-u3-stage4-{RUN_ID.lower().replace('_', '-')}"
    sandbox = policy["sandbox"]
    if Path(sandbox["stage_root"]) != wanted_stage:
        raise SettleExtensionRunError("stage root differs")
    if sandbox["network"] or sandbox["project_workspace_write_bind_count"] != 0 or sandbox["host_write_bind_count"] != 1:
        raise SettleExtensionRunError("sandbox write/network boundary differs")
    if sandbox["apparmor_profile_used"]:
        raise SettleExtensionRunError("settling extension must not use an AppArmor profile")
    if policy["solver"]["environment"] != base.EXPECTED_ENVIRONMENT:
        raise SettleExtensionRunError("sanitized environment differs")
    if policy["solver"]["argv"] != exact_solver_argv():
        raise SettleExtensionRunError("solver argv differs from the one-shot extension command")
    argv = policy["solver"]["argv"]
    viscosity = [item for item in argv if item.startswith(("-viscoart:", "-viscolam:", "-viscolamsps:"))]
    if viscosity != ["-viscoart:0.2"] or argv.count("-cfl:0.1") != 1 or argv.count("-shifting:none") != 1:
        raise SettleExtensionRunError("viscosity/CFL/Shifting invariants drifted")
    if any(token in item.lower() for item in argv for token in ("-cpu", "-gpu:1", "http://", "https://", "/home/", "..")):
        raise SettleExtensionRunError("solver argv contains a forbidden fragment")
    if policy["restart"] != {"enabled": True, "part_index": 701, "part_first": 701, "guest_dir": "/restart"}:
        raise SettleExtensionRunError("restart contract differs")
    if policy["expected_output"] != EXPECTED_OUTPUT or policy["limits"] != EXPECTED_LIMITS:
        raise SettleExtensionRunError("output or resource limits differ")
    if policy["single_delta"] != EXPECTED_DELTA:
        raise SettleExtensionRunError("single end-time delta differs")
    if not math.isclose(base._float_arg(argv, "-tmax:"), 36.05, rel_tol=0.0, abs_tol=1e-12):
        raise SettleExtensionRunError("tmax differs")
    if not math.isclose(base._float_arg(argv, "-tout:"), 0.05, rel_tol=0.0, abs_tol=1e-12):
        raise SettleExtensionRunError("tout differs")
    parent_paths = {
        "runner": SCRIPT_PATH,
        "schema": SCHEMA_PATH,
        "tests": TEST_PATH,
        "qc": QC_PATH,
        "qc_schema": QC_SCHEMA_PATH,
        "qc_tests": QC_TEST_PATH,
        "prior_policy": PRIOR_POLICY_PATH,
        "prior_qc": PRIOR_QC_PATH,
        "prior_tail_final": PRIOR_FINAL_PATH,
        "legacy_runner": Path(base.__file__).resolve(),
    }
    for name, path in parent_paths.items():
        if policy["parents"][name]["path"] != str(path):
            raise SettleExtensionRunError(f"parent path differs: {name}")
    if policy["parents"]["prior_policy"]["sha256"] != PRIOR_POLICY_SHA256:
        raise SettleExtensionRunError("cold-A policy identity differs")
    if policy["parents"]["prior_qc"]["sha256"] != PRIOR_QC_SHA256:
        raise SettleExtensionRunError("cold-A QC identity differs")
    if policy["parents"]["prior_tail_final"]["sha256"] != PRIOR_FINAL_SHA256:
        raise SettleExtensionRunError("cold-A final receipt identity differs")
    inputs = policy["inputs"]
    if inputs["restart_part"]["path"] != str(RESTART_PART_PATH) or inputs["restart_part"]["sha256"] != RESTART_PART_SHA256:
        raise SettleExtensionRunError("Part_0701 restart identity differs")
    if inputs["restart_head"]["path"] != str(RESTART_HEAD_PATH) or inputs["restart_head"]["sha256"] != RESTART_HEAD_SHA256:
        raise SettleExtensionRunError("restart head identity differs")
    if inputs["case_xml"]["sha256"] != "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb":
        raise SettleExtensionRunError("undamped XML identity differs")


def configure() -> None:
    base.SCRIPT_PATH = SCRIPT_PATH
    base.SCHEMA_PATH = SCHEMA_PATH
    base.semantic_validate = semantic_validate


def self_check(policy_path: Path) -> dict[str, Any]:
    configure()
    policy = base.load_policy(policy_path)
    verified = base.verify_parents_and_inputs(policy)
    argv = base.bwrap_argv(policy)
    if [argv[index + 1:index + 3] for index, item in enumerate(argv) if item == "--bind"] != [
        [policy["sandbox"]["stage_root"] + "/output", "/output"]
    ]:
        raise SettleExtensionRunError("writable bind proof differs")
    return {
        "status": "PASS_U3_VISCOART0P2_SETTLE_EXTENSION_RUNNER_V1_SELF_CHECK",
        "policy": base.identity(policy_path, maximum=base.MAX_POLICY_BYTES),
        "verified": verified,
        "sandbox_argv": argv,
        "single_delta": policy["single_delta"],
        "network_used": False,
        "candidate_executed": False,
        "project_workspace_writable_in_guest": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("policy", type=Path)
    result.add_argument("command", choices=("self-check", "preflight", "run"))
    return result


def main(argv: list[str] | None = None) -> int:
    configure()
    args = parser().parse_args(argv)
    try:
        if args.command == "self-check":
            result = self_check(args.policy)
        elif args.command == "preflight":
            result = {"status": "PASS_U3_VISCOART0P2_SETTLE_EXTENSION_PREFLIGHT", "evidence": base.preflight(args.policy, require_fresh=True)}
        else:
            result = base.execute(args.policy)
    except (SettleExtensionRunError, base.Stage4RunError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "FAIL_U3_VISCOART0P2_SETTLE_EXTENSION_RUNNER", "error_type": type(exc).__name__, "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
