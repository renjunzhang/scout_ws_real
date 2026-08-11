#!/usr/bin/env python3
"""QC wrapper for the official 3D-example viscosity-strength remediation.

The inherited QC still computes the original 17 metrics on the undamped tail.
This wrapper proves that the only numerical delta from v4 is the coefficient
change ``-viscoart:0.01`` to ``-viscoart:0.1`` in each phase, verifies the
sealed upstream v5.4 3D-example evidence, and publishes a create-new,
closed-schema adjudication.
It never invokes DualSPHysics or exposes a GPU.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_u3_damped_restart_qc_v1 as inherited_qc
import r8_liquid_u3_gpu_damped_restart_runner_v1 as base
import r8_liquid_u3_gpu_damped_restart_runner_v4 as prior_runner_v4
import r8_liquid_u3_gpu_damped_restart_runner_v5 as runner
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_damped_restart_viscoart0p1_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_damped_restart_qc_v5.py"
PRIOR_POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_damped_restart_viscoart_20260811T052342Z_v4.json"
PRIOR_POLICY_SHA256 = "20a6600e456dc5cc103e8b8d55d0f9a682a16ed1266f7b39d3b5e84531d8c7e9"
PRIOR_QC_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_damped_init_undamped_tail_viscoart_20260811T052342Z_v4.qc_v1.json")
PRIOR_QC_SHA256 = runner.PRIOR_FAILED_QC_SHA256
UPSTREAM_GIT = Path("/home/zrj/scout_liquid_lab/dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git")
UPSTREAM_EXAMPLE = runner.UPSTREAM_EXAMPLE_PATH
MAX_JSON_BYTES = 4 * 1024 * 1024
METRIC_NAMES = tuple(sorted(inherited_qc.metric_contract.METRIC_LIMITS))
METRIC_LIMITS = dict(sorted(inherited_qc.metric_contract.METRIC_LIMITS.items()))


class ViscoartQcError(ValueError):
    """Unsafe, drifted, incomplete or semantically inconsistent evidence."""


def canonical_sha256(value: Any) -> str:
    data = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def _read_exact_json(path: Path, expected_sha256: str) -> dict[str, Any]:
    observed = legacy.identity(path, maximum=MAX_JSON_BYTES)
    if observed["sha256"] != expected_sha256:
        raise ViscoartQcError(f"JSON identity drifted: {path}")
    return base.read_json(path)


def _run_viscosity(path: Path, expected_name: str, expected_value: float) -> dict[str, Any]:
    text = inherited_qc._run_text(path)
    names = re.findall(r'^Viscosity="([^"]+)"\s*$', text, flags=re.MULTILINE)
    values = re.findall(r'^\s*Visco=([^\s]+)\s*$', text, flags=re.MULTILINE)
    if len(names) != 1 or len(values) != 1:
        raise ViscoartQcError(f"viscosity output cardinality differs: {path}")
    try:
        value = float(values[0])
    except ValueError as exc:
        raise ViscoartQcError(f"viscosity output is not numeric: {path}") from exc
    if names[0] != expected_name or not math.isclose(value, expected_value, rel_tol=0.0, abs_tol=1.0e-12):
        raise ViscoartQcError(f"viscosity output differs: name={names[0]!r} value={value!r}")
    return {"name": names[0], "value": value, "run_out": legacy.identity(path, maximum=64 * 1024 * 1024)}


def _upstream_example_proof() -> dict[str, Any]:
    base_argv = ["/usr/bin/git", f"--git-dir={UPSTREAM_GIT}"]
    environment = {"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"}
    revision = f"{runner.UPSTREAM_COMMIT}:{UPSTREAM_EXAMPLE}"
    blob = subprocess.run(
        [*base_argv, "rev-parse", revision], stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10, check=False, env=environment,
    )
    content = subprocess.run(
        [*base_argv, "show", revision], stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10, check=False, env=environment,
    )
    if blob.returncode != 0 or content.returncode != 0:
        raise ViscoartQcError("sealed upstream 3D example cannot be read")
    blob_text = blob.stdout.decode("ascii", "strict").strip()
    digest = hashlib.sha256(content.stdout).hexdigest()
    text = content.stdout.decode("utf-8")
    if blob_text != runner.UPSTREAM_EXAMPLE_BLOB or digest != runner.UPSTREAM_EXAMPLE_SHA256:
        raise ViscoartQcError("sealed upstream 3D example identity drifted")
    required = (
        'key="ViscoTreatment" value="1"',
        'key="Visco" value="0.1"',
        '<size x="0.12" y="0.12" z="0.45"',
    )
    if any(token not in text for token in required):
        raise ViscoartQcError("upstream 3D artificial-viscosity evidence is absent")
    return {
        "commit": runner.UPSTREAM_COMMIT,
        "path": UPSTREAM_EXAMPLE,
        "blob": blob_text,
        "content_sha256": digest,
        "declared_visco_treatment": 1,
        "declared_visco": 0.1,
        "three_dimensional_geometry": True,
        "network_used": False,
    }


def policy_delta_proof(policy_path: Path) -> dict[str, Any]:
    current = base.load_policy(policy_path)
    prior = _read_exact_json(PRIOR_POLICY_PATH, PRIOR_POLICY_SHA256)
    phase_checks: dict[str, bool] = {}
    argv_hashes: dict[str, dict[str, str]] = {}
    for phase_name in base.PHASE_KEYS:
        prior_phase = prior["phases"][phase_name]
        current_phase = current["phases"][phase_name]
        normalized = [
            prior_runner_v4.VISCO_ARG if item == runner.VISCO_ARG else item
            for item in current_phase["solver_argv"]
        ]
        phase_checks[f"{phase_name}_argv_only_changes_viscoart_strength"] = (
            current_phase["solver_argv"].count(runner.VISCO_ARG) == 1
            and normalized == prior_phase["solver_argv"]
        )
        for key in ("purpose", "case_xml_input", "restart", "expected_output", "limits"):
            phase_checks[f"{phase_name}_{key}_unchanged"] = current_phase[key] == prior_phase[key]
        prior_sandbox = dict(prior_phase["sandbox"])
        current_sandbox = dict(current_phase["sandbox"])
        prior_sandbox.pop("stage_root", None)
        current_sandbox.pop("stage_root", None)
        phase_checks[f"{phase_name}_sandbox_boundary_unchanged"] = current_sandbox == prior_sandbox
        argv_hashes[phase_name] = {
            "prior_canonical_sha256": canonical_sha256(prior_phase["solver_argv"]),
            "current_canonical_sha256": canonical_sha256(current_phase["solver_argv"]),
            "current_normalized_to_prior_canonical_sha256": canonical_sha256(normalized),
        }
    invariant_checks = {
        f"{key}_unchanged": current[key] == prior[key]
        for key in ("tools", "gpu", "inputs", "damping_contract", "solver_invariants", "acceptance", "result_boundary")
    }
    checks = {**phase_checks, **invariant_checks}
    if not all(checks.values()):
        raise ViscoartQcError(f"v5 policy delta is not isolated: {[key for key, value in checks.items() if not value]}")
    return {
        "parameter": "ARTIFICIAL_VISCOSITY_COEFFICIENT",
        "baseline": 0.01,
        "candidate": 0.1,
        "baseline_argument": prior_runner_v4.VISCO_ARG,
        "candidate_argument": runner.VISCO_ARG,
        "applied_to_phases": list(base.PHASE_KEYS),
        "other_numerical_parameters_changed": False,
        "checks": dict(sorted(checks.items())),
        "argv_canonical_sha256": argv_hashes,
    }


def _static_self_check(policy_path: Path) -> dict[str, Any]:
    runner.configure()
    delta = policy_delta_proof(policy_path)
    upstream = _upstream_example_proof()
    schema = base.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    return {
        "status": "PASS_U3_DAMPED_RESTART_VISCOART0P1_QC_V5_STATIC_SELF_CHECK",
        "policy_delta": delta,
        "upstream_example": upstream,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def build_report(policy_path: Path) -> dict[str, Any]:
    runner.configure()
    inherited_qc.SCRIPT_PATH = SCRIPT_PATH
    inherited = inherited_qc.build_report(policy_path)
    inherited_qc.validate_report(inherited)
    policy = base.load_policy(policy_path)
    delta = policy_delta_proof(policy_path)
    upstream = _upstream_example_proof()
    prior_policy = _read_exact_json(PRIOR_POLICY_PATH, PRIOR_POLICY_SHA256)
    prior_qc = _read_exact_json(PRIOR_QC_PATH, PRIOR_QC_SHA256)
    prior_verdict = prior_qc.get("verdict", {})
    prior_tail = prior_qc.get("settling_evaluation", {})
    if (
        prior_verdict.get("status") != "FAIL_U3_VISCOART_DAMPED_INITIALIZATION_UNDAMPED_TAIL_NOT_SETTLED"
        or prior_verdict.get("u3_remediation_candidate_pass") is not False
        or len(prior_tail.get("failed_absolute_metrics", [])) != 9
    ):
        raise ViscoartQcError("prior failed damped-tail adjudication semantics differ")
    prior_viscosity = {
        phase_name: _run_viscosity(
            Path(prior_policy["phases"][phase_name]["run"]["output_root"]) / "Run.out", "Artificial", 0.01
        )
        for phase_name in base.PHASE_KEYS
    }
    current_viscosity = {
        phase_name: _run_viscosity(
            Path(policy["phases"][phase_name]["run"]["output_root"]) / "Run.out", "Artificial", 0.1
        )
        for phase_name in base.PHASE_KEYS
    }
    tail = inherited["undamped_tail"]
    if tuple(sorted(tail["metrics"])) != METRIC_NAMES:
        raise ViscoartQcError("inherited QC metric vector differs from the frozen 17 items")
    selected = bool(inherited["verdict"]["u3_remediation_candidate_pass"])
    inputs = {
        "policy": legacy.identity(policy_path, maximum=MAX_JSON_BYTES),
        "runner": legacy.identity(runner.SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "qc_script": legacy.identity(SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "qc_schema": legacy.identity(SCHEMA_PATH, maximum=MAX_JSON_BYTES),
        "qc_tests": legacy.identity(TEST_PATH, maximum=MAX_JSON_BYTES),
        "prior_policy": legacy.identity(PRIOR_POLICY_PATH, maximum=MAX_JSON_BYTES),
        "prior_qc": legacy.identity(PRIOR_QC_PATH, maximum=MAX_JSON_BYTES),
        "init_receipt": inherited["inputs"]["init_receipt"],
        "tail_receipt": inherited["inputs"]["tail_receipt"],
    }
    return {
        "schema_version": "r8-liquid-u3-damped-restart-viscoart0p1-qc-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_VISCOART0P1_QC_V1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": inputs,
        "remediation_delta": delta,
        "upstream_example_proof": upstream,
        "runtime_viscosity_proof": {
            "prior": prior_viscosity,
            "candidate": current_viscosity,
            "candidate_init_and_tail_exact": True,
        },
        "inherited_qc_canonical_sha256": canonical_sha256(inherited),
        "settling_evaluation": {
            "status": tail["status"],
            "structural_checks_all_pass": tail["structural_checks_all_pass"],
            "frame_count": tail["frame_count"],
            "first_time_s": tail["first_time_s"],
            "last_time_s": tail["last_time_s"],
            "metrics": tail["metrics"],
            "metric_limits": tail["metric_limits"],
            "metric_absolute_pass": tail["metric_absolute_pass"],
            "failed_absolute_metrics": tail["failed_absolute_metrics"],
            "trajectory": tail["trajectory"],
        },
        "verdict": {
            "status": (
                "PASS_U3_VISCOART0P1_DAMPED_INITIALIZATION_UNDAMPED_TAIL_SETTLED_CANDIDATE"
                if selected else "FAIL_U3_VISCOART0P1_DAMPED_INITIALIZATION_UNDAMPED_TAIL_NOT_SETTLED"
            ),
            "u3_remediation_candidate_pass": selected,
            "settled_state_frozen": False,
            "stage4_complete": False,
            "phase5_admitted": False,
            "exact_blocker": "NONE" if selected else f"UNDAMPED_TAIL_FAILS_{len(tail['failed_absolute_metrics'])}_OF_17_ABSOLUTE_SETTLING_LIMITS",
            "next": "COLD_B_RESTART_EQUIVALENCE_THEN_PARITY_U4" if selected else "STOP_AND_PRESERVE_REMEDIATION_EVIDENCE",
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "solver_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
        },
    }


def validate_report(report: dict[str, Any]) -> None:
    runtime = report.get("runtime_viscosity_proof", {})
    for group, expected_name, expected_value in (
        ("prior", "Artificial", 0.01),
        ("candidate", "Artificial", 0.1),
    ):
        phases = runtime.get(group, {})
        for phase_name in base.PHASE_KEYS:
            evidence = phases.get(phase_name, {})
            if (
                evidence.get("name") != expected_name
                or not isinstance(evidence.get("value"), (int, float))
                or not math.isclose(float(evidence["value"]), expected_value, rel_tol=0.0, abs_tol=1.0e-12)
            ):
                raise ViscoartQcError(f"{group}/{phase_name} viscosity proof differs")
    settling = report.get("settling_evaluation", {})
    metrics = settling.get("metrics", {})
    limits = settling.get("metric_limits", {})
    passes = settling.get("metric_absolute_pass", {})
    failed = settling.get("failed_absolute_metrics", [])
    if tuple(sorted(metrics)) != METRIC_NAMES or tuple(sorted(passes)) != METRIC_NAMES:
        raise ViscoartQcError("QC metric or pass vector is not the frozen 17-item set")
    if limits != METRIC_LIMITS:
        raise ViscoartQcError("QC metric limits drifted from the frozen 17-item contract")
    expected_failed = sorted(name for name in METRIC_NAMES if passes.get(name) is False)
    if failed != expected_failed:
        raise ViscoartQcError("failed metric list differs from the boolean pass vector")
    selected = not expected_failed
    verdict = report.get("verdict", {})
    expected_settling_status = (
        "PASS_UNDAMPED_TAIL_ALL_17_LIMITS"
        if selected else "FAIL_UNDAMPED_TAIL_ABSOLUTE_SETTLING_LIMITS"
    )
    expected_verdict_status = (
        "PASS_U3_VISCOART0P1_DAMPED_INITIALIZATION_UNDAMPED_TAIL_SETTLED_CANDIDATE"
        if selected else "FAIL_U3_VISCOART0P1_DAMPED_INITIALIZATION_UNDAMPED_TAIL_NOT_SETTLED"
    )
    if (
        settling.get("status") != expected_settling_status
        or verdict.get("u3_remediation_candidate_pass") is not selected
        or verdict.get("status") != expected_verdict_status
    ):
        raise ViscoartQcError("settling and verdict fields are internally inconsistent")
    schema = base.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(report), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise ViscoartQcError(f"QC schema failure at {list(first.absolute_path)}: {first.message}")


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    parent_metadata = os.lstat(path.parent)
    if not os.path.isdir(path.parent) or os.path.islink(path.parent):
        raise ViscoartQcError(f"QC output parent is not a real directory: mode={parent_metadata.st_mode:o}")
    data = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    if len(data) > MAX_JSON_BYTES:
        raise ViscoartQcError("QC result exceeds output bound")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ViscoartQcError("short QC output write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return legacy.identity(path, maximum=MAX_JSON_BYTES)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("policy", type=Path)
    result.add_argument("--output", type=Path)
    result.add_argument("--self-check", action="store_true")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.self_check:
            print(json.dumps(_static_self_check(args.policy), ensure_ascii=False, sort_keys=True))
            return 0
        if args.output is None:
            raise ViscoartQcError("--output is required unless --self-check is used")
        report = build_report(args.policy)
        validate_report(report)
        identity = write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_remediation_candidate_pass"] else 1
    except (
        ViscoartQcError,
        inherited_qc.DampedRestartQcError,
        base.DampedRestartRunError,
        legacy.Stage4RunError,
        OSError,
        ValueError,
        subprocess.SubprocessError,
    ) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
