#!/usr/bin/env python3
"""One-shot GPU artificial-viscosity 0.2 sensitivity probe from v6.

This development-only execution revision restarts the exact v6 ``Part_0601``
state at 30.05 s and changes exactly one numerical parameter: artificial
viscosity 0.1 to 0.2, observing through 35.05 s.  The inherited Stage-4 runner
provides O_EXCL receipts, a root-only transport, uid/gid 1000 execution,
network isolation, one writable output bind, resource limits and immutable
source-candidate verification.
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
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p2_probe_policy_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_gpu_viscoart0p2_probe_runner_v1.py"
QC_PATH = SCRIPT_PATH.parent / "r8_liquid_u3_viscoart0p2_probe_qc_v1.py"
QC_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p2_probe_qc_v1.json"
QC_TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p2_probe_qc_v1.py"
POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_viscoart0p2_probe_from_0601_20260811T074334Z_v7.json"
RUN_ID = "u3_c1m_gpu_viscoart0p2_probe_from_0601_20260811T074334Z_v7"
PRIOR_QC_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p1_extend_from_0401_20260811T065812Z_v6.qc_v2.json")
PRIOR_QC_SHA256 = "887819e71a6723d10a02106c60ad6aefd5fdff6e331ac192bdd1333849378f4f"
PRIOR_FINAL_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p1_extend_from_0401_20260811T065812Z_v6.final.json")
PRIOR_FINAL_SHA256 = "78789de2d5598693e4fcdcce3832b21501e1c23aef3a91107827710deb6614d2"
RESTART_PART_PATH = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_viscoart0p1_extend_from_0401_20260811T065812Z_v6.partial/output/data/Part_0601.bi4")
RESTART_PART_SHA256 = "49b0877c82c283a9424aed078f1ccf37861e464ead1e138b96bcc9ee86fd8152"
RESTART_HEAD_PATH = RESTART_PART_PATH.parent / "Part_Head.ibi4"
RESTART_HEAD_SHA256 = "99c25ee24c3c3dedd4147e8585ca042d67f7f2f083a6783561e1474d855cb67a"
VISCO_ARG = "-viscoart:0.2"
EXPECTED_OUTPUT = {
    "start_time_s": 30.05,
    "end_time_s": 35.05,
    "output_period_s": 0.05,
    "part_first": 601,
    "part_last": 701,
    "part_count": 101,
    "maximum_output_bytes": 268435456,
}
EXPECTED_LIMITS = {
    "wall_timeout_seconds": 1800,
    "kill_after_seconds": 10,
    "monitor_interval_seconds": 10,
    "minimum_mem_available_bytes": 4294967296,
    "maximum_output_bytes": 268435456,
}
EXPECTED_DELTA = {
    "parameter": "ARTIFICIAL_VISCOSITY_COEFFICIENT",
    "baseline": "0.1",
    "candidate": "0.2",
    "other_numerical_parameters_changed": False,
}


class ViscoartProbeRunError(base.Stage4RunError):
    """Unsafe, drifted or non-exclusive viscosity-probe contract."""


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
        "-partbegin:601:601",
        "/restart",
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
        raise ViscoartProbeRunError("policy path is not the frozen v7 path")
    if policy["run_id"] != RUN_ID or policy["purpose"] != "viscoart0p2_sensitivity_probe":
        raise ViscoartProbeRunError("run identity or purpose differs")
    if policy["run"] != _exact_run_paths():
        raise ViscoartProbeRunError("run and audit paths differ")
    wanted_stage = Path("/var/tmp") / f"r8-liquid-u3-stage4-{RUN_ID.lower().replace('_', '-')}"
    sandbox = policy["sandbox"]
    if Path(sandbox["stage_root"]) != wanted_stage:
        raise ViscoartProbeRunError("stage root differs")
    if sandbox["network"] or sandbox["project_workspace_write_bind_count"] != 0 or sandbox["host_write_bind_count"] != 1:
        raise ViscoartProbeRunError("sandbox write/network boundary differs")
    if sandbox["apparmor_profile_used"]:
        raise ViscoartProbeRunError("viscosity probe must not use an AppArmor profile")
    if policy["solver"]["environment"] != base.EXPECTED_ENVIRONMENT:
        raise ViscoartProbeRunError("sanitized environment differs")
    if policy["solver"]["argv"] != exact_solver_argv():
        raise ViscoartProbeRunError("solver argv differs from the single viscosity-probe command")
    argv = policy["solver"]["argv"]
    viscosity = [item for item in argv if item.startswith(("-viscoart:", "-viscolam:", "-viscolamsps:"))]
    if viscosity != [VISCO_ARG] or argv.count("-cfl:0.1") != 1 or argv.count("-shifting:none") != 1:
        raise ViscoartProbeRunError("viscosity/CFL/Shifting invariants drifted")
    if any(token in item.lower() for item in argv for token in ("-cpu", "-gpu:1", "http://", "https://", "/home/", "..")):
        raise ViscoartProbeRunError("solver argv contains a forbidden fragment")
    if policy["restart"] != {"enabled": True, "part_index": 601, "part_first": 601, "guest_dir": "/restart"}:
        raise ViscoartProbeRunError("restart contract differs")
    if policy["expected_output"] != EXPECTED_OUTPUT or policy["limits"] != EXPECTED_LIMITS:
        raise ViscoartProbeRunError("output or resource limits differ")
    if policy["single_delta"] != EXPECTED_DELTA:
        raise ViscoartProbeRunError("single artificial-viscosity delta differs")
    if not math.isclose(base._float_arg(argv, "-tmax:"), 35.05, rel_tol=0.0, abs_tol=1e-12):
        raise ViscoartProbeRunError("tmax differs")
    if not math.isclose(base._float_arg(argv, "-tout:"), 0.05, rel_tol=0.0, abs_tol=1e-12):
        raise ViscoartProbeRunError("tout differs")
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
            raise ViscoartProbeRunError(f"parent path differs: {name}")
    if policy["parents"]["prior_qc"]["sha256"] != PRIOR_QC_SHA256:
        raise ViscoartProbeRunError("v6 QC identity differs")
    if policy["parents"]["prior_tail_final"]["sha256"] != PRIOR_FINAL_SHA256:
        raise ViscoartProbeRunError("v6 final receipt identity differs")
    inputs = policy["inputs"]
    if inputs["restart_part"]["path"] != str(RESTART_PART_PATH) or inputs["restart_part"]["sha256"] != RESTART_PART_SHA256:
        raise ViscoartProbeRunError("Part_0601 restart identity differs")
    if inputs["restart_head"]["path"] != str(RESTART_HEAD_PATH) or inputs["restart_head"]["sha256"] != RESTART_HEAD_SHA256:
        raise ViscoartProbeRunError("restart head identity differs")
    if inputs["case_xml"]["sha256"] != "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb":
        raise ViscoartProbeRunError("undamped XML identity differs")


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
        raise ViscoartProbeRunError("writable bind proof differs")
    return {
        "status": "PASS_U3_VISCOART0P2_PROBE_RUNNER_V1_SELF_CHECK",
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
            result = {"status": "PASS_U3_VISCOART0P2_PROBE_PREFLIGHT", "evidence": base.preflight(args.policy, require_fresh=True)}
        else:
            result = base.execute(args.policy)
    except (ViscoartProbeRunError, base.Stage4RunError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "FAIL_U3_VISCOART0P2_PROBE_RUNNER", "error_type": type(exc).__name__, "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
