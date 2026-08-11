#!/usr/bin/env python3
"""Fresh GPU artificial-viscosity 0.3 cold-A/B runner.

The two independent policies start from the exact sealed C1M initial state and
run the accepted development candidate through 35.05 s with identical
``CFL=0.1``, ``Shifting=None``, ``DDT2(0.1)`` and artificial viscosity 0.3.
The inherited Stage-4 runner provides O_EXCL receipts, root-only transport,
uid/gid 1000 execution, network isolation, one writable output bind, resource
limits and immutable source-candidate verification.
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
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p3_cold_ab_policy_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_gpu_viscoart0p3_cold_ab_runner_v1.py"
QC_PATH = SCRIPT_PATH.parent / "r8_liquid_u3_viscoart0p3_cold_ab_qc_v1.py"
QC_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p3_cold_ab_qc_v1.json"
QC_TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p3_cold_ab_qc_v1.py"
POLICY_PATHS = {
    "a": PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_viscoart0p3_cold_a_20260811T101427Z_v12.json",
    "b": PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_viscoart0p3_cold_b_20260811T101427Z_v12.json",
}
RUN_IDS = {
    "a": "u3_c1m_gpu_viscoart0p3_cold_a_20260811T101427Z_v12",
    "b": "u3_c1m_gpu_viscoart0p3_cold_b_20260811T101427Z_v12",
}
PURPOSES = {"a": "viscoart0p3_fresh_cold_a", "b": "viscoart0p3_fresh_cold_b"}
PRIOR_QC_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_probe_from_0801_20260811T095804Z_v11.qc_v1.json")
PRIOR_QC_SHA256 = "a91cda7eebcc72e2cdde71fe660f21e6e364c417a02e7200cc007c1dd945bfed"
PRIOR_FINAL_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_probe_from_0801_20260811T095804Z_v11.final.json")
PRIOR_FINAL_SHA256 = "868e212b95a96bdd29991cf4c772a0c68a6edcb8694b50797149eab1f0abf4b4"
VISCO_ARG = "-viscoart:0.3"
EXPECTED_OUTPUT = {
    "start_time_s": 0.0,
    "end_time_s": 35.05,
    "output_period_s": 0.05,
    "part_first": 0,
    "part_last": 701,
    "part_count": 702,
    "maximum_output_bytes": 402653184,
}
EXPECTED_LIMITS = {
    "wall_timeout_seconds": 3600,
    "kill_after_seconds": 10,
    "monitor_interval_seconds": 10,
    "minimum_mem_available_bytes": 4294967296,
    "maximum_output_bytes": 402653184,
}
EXPECTED_DELTA = {
    "parameter": "RUN_INITIALIZATION",
    "baseline": "V11_RESTARTED_SENSITIVITY_PROBE",
    "candidate": "FRESH_CASE_INITIAL_STATE",
    "other_numerical_parameters_changed": False,
}


class ColdAbRunError(base.Stage4RunError):
    """Unsafe, drifted or non-exclusive cold-A/B contract."""


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
        "-shifting:none",
        VISCO_ARG,
        "-tmax:35.05",
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


def _exact_run_paths(run_id: str) -> dict[str, str]:
    attempt = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs") / f"{run_id}.partial"
    audit = Path("/home/zrj/scout_liquid_lab/audits")
    return {
        "attempt_root": str(attempt),
        "output_root": str(attempt / "output"),
        "start_receipt": str(audit / f"{run_id}.start.json"),
        "final_receipt": str(audit / f"{run_id}.final.json"),
        "failure_receipt": str(audit / f"{run_id}.failure.json"),
        "stdout_log": str(audit / f"{run_id}.stdout.log"),
        "stderr_log": str(audit / f"{run_id}.stderr.log"),
        "resource_log": str(audit / f"{run_id}.resources.jsonl"),
    }


def semantic_validate(policy: dict[str, Any], policy_path: Path) -> None:
    roles = [
        role
        for role in ("a", "b")
        if policy_path == POLICY_PATHS[role]
        and policy.get("run_id") == RUN_IDS[role]
        and policy.get("purpose") == PURPOSES[role]
    ]
    if len(roles) != 1:
        raise ColdAbRunError("run identity or purpose differs")
    role = roles[0]
    run_id = RUN_IDS[role]
    if policy["run"] != _exact_run_paths(run_id):
        raise ColdAbRunError("run and audit paths differ")
    wanted_stage = Path("/var/tmp") / f"r8-liquid-u3-stage4-{run_id.lower().replace('_', '-')}"
    sandbox = policy["sandbox"]
    if Path(sandbox["stage_root"]) != wanted_stage:
        raise ColdAbRunError("stage root differs")
    if sandbox["network"] or sandbox["project_workspace_write_bind_count"] != 0 or sandbox["host_write_bind_count"] != 1:
        raise ColdAbRunError("sandbox write/network boundary differs")
    if sandbox["apparmor_profile_used"]:
        raise ColdAbRunError("cold-A/B must not use an AppArmor profile")
    if policy["solver"]["environment"] != base.EXPECTED_ENVIRONMENT:
        raise ColdAbRunError("sanitized environment differs")
    if policy["solver"]["argv"] != exact_solver_argv():
        raise ColdAbRunError("solver argv differs from the frozen cold command")
    argv = policy["solver"]["argv"]
    viscosity = [item for item in argv if item.startswith(("-viscoart:", "-viscolam:", "-viscolamsps:"))]
    if viscosity != [VISCO_ARG] or argv.count("-cfl:0.1") != 1 or argv.count("-shifting:none") != 1:
        raise ColdAbRunError("viscosity/CFL/Shifting invariants drifted")
    if any(token in item.lower() for item in argv for token in ("-cpu", "-gpu:1", "http://", "https://", "/home/", "..")):
        raise ColdAbRunError("solver argv contains a forbidden fragment")
    if policy["restart"] != {"enabled": False, "part_index": None, "part_first": None, "guest_dir": None}:
        raise ColdAbRunError("fresh initialization contract differs")
    if policy["inputs"]["restart_part"] is not None or policy["inputs"]["restart_head"] is not None:
        raise ColdAbRunError("fresh cold run carries restart inputs")
    if policy["expected_output"] != EXPECTED_OUTPUT or policy["limits"] != EXPECTED_LIMITS:
        raise ColdAbRunError("output or resource limits differ")
    if policy["single_delta"] != EXPECTED_DELTA:
        raise ColdAbRunError("fresh-initialization validation axis differs")
    if not math.isclose(base._float_arg(argv, "-tmax:"), 35.05, rel_tol=0.0, abs_tol=1e-12):
        raise ColdAbRunError("tmax differs")
    if not math.isclose(base._float_arg(argv, "-tout:"), 0.05, rel_tol=0.0, abs_tol=1e-12):
        raise ColdAbRunError("tout differs")
    parent_paths = {
        "runner": SCRIPT_PATH,
        "schema": SCHEMA_PATH,
        "tests": TEST_PATH,
        "qc": QC_PATH,
        "qc_schema": QC_SCHEMA_PATH,
        "qc_tests": QC_TEST_PATH,
        "prior_qc": PRIOR_QC_PATH,
        "prior_tail_final": PRIOR_FINAL_PATH,
        "legacy_runner": Path(base.__file__).resolve(),
    }
    for name, path in parent_paths.items():
        if policy["parents"][name]["path"] != str(path):
            raise ColdAbRunError(f"parent path differs: {name}")
    if policy["parents"]["prior_qc"]["sha256"] != PRIOR_QC_SHA256:
        raise ColdAbRunError("v11 QC identity differs")
    if policy["parents"]["prior_tail_final"]["sha256"] != PRIOR_FINAL_SHA256:
        raise ColdAbRunError("v11 final receipt identity differs")
    inputs = policy["inputs"]
    if inputs["case_xml"]["sha256"] != "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb":
        raise ColdAbRunError("undamped XML identity differs")


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
        raise ColdAbRunError("writable bind proof differs")
    return {
        "status": "PASS_U3_VISCOART0P3_COLD_AB_RUNNER_V1_SELF_CHECK",
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
            result = {"status": "PASS_U3_VISCOART0P3_COLD_AB_PREFLIGHT", "evidence": base.preflight(args.policy, require_fresh=True)}
        else:
            result = base.execute(args.policy)
    except (ColdAbRunError, base.Stage4RunError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "FAIL_U3_VISCOART0P3_COLD_AB_RUNNER", "error_type": type(exc).__name__, "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
