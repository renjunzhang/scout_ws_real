#!/usr/bin/env python3
"""Read-only per-Id cold-A/B repeatability and restart-equivalence QC.

The contract is frozen before cross-run values are inspected.  Cold-A is the
fresh 0--35.05 s run followed by its 35.05--45.05 s checkpoint continuation;
cold-B is the independent fresh continuous 0--45.05 s run.  The tool requires
raw-byte equality through Part_0701, exact canonical checkpoint arrays, and
bounded per-Id Pos/Vel/Rhop error after the restart.  It never executes a
solver, exposes a GPU, or modifies any input.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import struct
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy
import r8_liquid_u3_solver_output_qc_v3 as qc_v3


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_viscoart0p3_repeatability_restart_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_viscoart0p3_repeatability_restart_qc_v1.py"
PLAN_PATH = PACKAGE_ROOT.parents[3] / "docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md"
OUTPUT_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_repeatability_restart_20260811T124800Z_v15.qc_v1.json")

A_BASE_ROOT = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_viscoart0p3_cold_a_20260811T101427Z_v12.partial/output")
A_RESTART_ROOT = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_viscoart0p3_cold_a_extend_from_0701_20260811T110912Z_v13.partial/output")
B_CONTINUOUS_ROOT = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_viscoart0p3_cold_b_20260811T113902Z_v14.partial/output")

FROZEN_INPUTS = {
    "plan": (PLAN_PATH, "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"),
    "a_base_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_viscoart0p3_cold_a_20260811T101427Z_v12.json",
        "cf63dbccd33e778e7099f8b475030092b42269ff7b77b321ba70073a9bb17262",
    ),
    "a_base_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_cold_a_20260811T101427Z_v12.final.json"),
        "0176099ea2b261ba79c2fb1d05a757e7dd6e4df7a04b5005994b1dfd04d74d2b",
    ),
    "a_base_qc": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_cold_a_20260811T101427Z_v12.qc_v1.json"),
        "7bb34f2a280572963b4bb993e5247415f0a6d98e649ef6849f76c1462d59fbec",
    ),
    "a_restart_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_viscoart0p3_cold_a_extend_from_0701_20260811T110912Z_v13.json",
        "dfaecfe33d2f170bea9c67808b8aab0a81dfba48a4eb43b1627f0afed799c0a1",
    ),
    "a_restart_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_cold_a_extend_from_0701_20260811T110912Z_v13.final.json"),
        "6126ff3dd61d031a36db5d4d87188fde46daad86be2abeeef1f7f078dabf9099",
    ),
    "a_restart_qc": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_cold_a_extend_from_0701_20260811T110912Z_v13.qc_v1.json"),
        "982d1a5a5bcebc4d85bfdb09f9d9e9fe03d84729a0c84f84f0f9cbd66990dd25",
    ),
    "b_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_viscoart0p3_cold_b_20260811T113902Z_v14.json",
        "9f361d4e21d1ec97548c6e8b3587e8545b8ea4f48d53c2c930bdd711bb818064",
    ),
    "b_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_cold_b_20260811T113902Z_v14.final.json"),
        "98ad12bb4c7539b6b4ed78f578f39334e972b3c9bc90bd81cb15924e17be9acf",
    ),
    "b_qc": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_cold_b_20260811T113902Z_v14.qc_v1.json"),
        "ae00ad76cffc7a2461e14a44bd06379c058bda73e8b35d2a868822147ec7d249",
    ),
}

TOLERANCES = {
    "time_abs_max_s": 0.0001,
    "position_l2_max_m": 0.00005,
    "position_l2_rms_m": 0.000005,
    "velocity_l2_max_m_s": 0.0001,
    "velocity_l2_rms_m_s": 0.00002,
    "density_relative_max": 0.0001,
    "density_relative_rms": 0.00002,
}
PRE_PARTS = tuple(range(0, 702))
POST_PARTS = tuple(range(701, 902))
EXPECTED_PARTICLES = 9078
MAX_PART_BYTES = 1024 * 1024
MAX_JSON_BYTES = 4 * 1024 * 1024


class RepeatabilityRestartQcError(ValueError):
    """Drifted input, unsafe BI4, or failed frozen equivalence contract."""


def _identity(path: Path, *, maximum: int = MAX_JSON_BYTES) -> dict[str, Any]:
    value = legacy.identity(path, maximum=maximum)
    return {key: value[key] for key in ("path", "sha256", "size_bytes", "mode", "inode", "nlink")}


def _verify_frozen_inputs() -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for name, (path, expected) in FROZEN_INPUTS.items():
        observed = _identity(path, maximum=MAX_JSON_BYTES)
        if observed["sha256"] != expected:
            raise RepeatabilityRestartQcError(f"frozen input hash differs: {name}")
        result[name] = observed
    for name in ("a_base_final", "a_restart_final", "b_final"):
        value = legacy.read_json(FROZEN_INPUTS[name][0])
        if value.get("status") != "PASS_GPU_STAGE4_RUN_RAW_OUTPUT" or value.get("compute_processes_after") != []:
            raise RepeatabilityRestartQcError(f"raw run receipt is not a clean PASS: {name}")
    a_base_qc = legacy.read_json(FROZEN_INPUTS["a_base_qc"][0])
    a_restart_qc = legacy.read_json(FROZEN_INPUTS["a_restart_qc"][0])
    b_qc = legacy.read_json(FROZEN_INPUTS["b_qc"][0])
    a_base_verdict = a_base_qc.get("verdict", {})
    if (
        a_base_verdict.get("status") != "FAIL_U3_VISCOART0P3_COLD_NOT_SETTLED"
        or a_base_verdict.get("u3_remediation_candidate_pass") is not False
        or a_base_verdict.get("next") != "STOP_OR_FREEZE_ONE_NEW_NUMERICAL_DELTA"
    ):
        raise RepeatabilityRestartQcError("cold-A 35.05 s historical QC semantics differ")
    if a_restart_qc.get("verdict", {}).get("u3_remediation_candidate_pass") is not True:
        raise RepeatabilityRestartQcError("cold-A 45.05 s settling QC is not PASS")
    if b_qc.get("verdict", {}).get("u3_remediation_candidate_pass") is not True:
        raise RepeatabilityRestartQcError("cold-B 45.05 s settling QC is not PASS")
    return result


def _read_frame(path: Path, expected_part: int) -> dict[str, Any]:
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise RepeatabilityRestartQcError(f"unsafe Part file: {path}")
    if stat.S_IMODE(metadata.st_mode) != 0o440 or not 1 <= metadata.st_size <= MAX_PART_BYTES:
        raise RepeatabilityRestartQcError(f"Part mode/size differs: {path}")
    source = bi4.read_regular_file(path, max_bytes=MAX_PART_BYTES)
    root = bi4.parse_jpartdata_bi4(source.data)
    part, ids, positions, velocities, densities = qc_v3._part_arrays(root)
    cpart = int(part.values.get("Cpart"))
    time_s = float(part.values.get("TimeStep"))
    if cpart != expected_part or len(ids) != EXPECTED_PARTICLES:
        raise RepeatabilityRestartQcError(f"Part identity/count differs: {path}")
    if len(set(ids)) != EXPECTED_PARTICLES or set(ids) != set(range(EXPECTED_PARTICLES)):
        raise RepeatabilityRestartQcError(f"Part Id set differs: {path}")
    if any(not math.isfinite(value) for vector in positions for value in vector):
        raise RepeatabilityRestartQcError(f"non-finite position: {path}")
    if any(not math.isfinite(value) for vector in velocities for value in vector):
        raise RepeatabilityRestartQcError(f"non-finite velocity: {path}")
    if any(not math.isfinite(value) for value in densities):
        raise RepeatabilityRestartQcError(f"non-finite density: {path}")
    digest = hashlib.sha256()
    for particle_id, position, velocity, density in zip(ids, positions, velocities, densities):
        digest.update(struct.pack(">Q7d", particle_id, *position, *velocity, density))
    return {
        "sha256": source.sha256,
        "time_s": time_s,
        "ids": ids,
        "positions": positions,
        "velocities": velocities,
        "densities": densities,
        "canonical_array_sha256": digest.hexdigest(),
    }


def _raw_repeatability() -> dict[str, Any]:
    mismatches: list[int] = []
    mismatch_count = 0
    digest = hashlib.sha256()
    for part in PRE_PARTS:
        left = A_BASE_ROOT / "data" / f"Part_{part:04d}.bi4"
        right = B_CONTINUOUS_ROOT / "data" / f"Part_{part:04d}.bi4"
        left_sha = legacy.sha256_file(left, maximum=MAX_PART_BYTES)
        right_sha = legacy.sha256_file(right, maximum=MAX_PART_BYTES)
        digest.update(f"{part}:{left_sha}:{right_sha}\n".encode("ascii"))
        if left_sha != right_sha:
            mismatch_count += 1
            if len(mismatches) < 32:
                mismatches.append(part)
    return {
        "part_first": PRE_PARTS[0],
        "part_last": PRE_PARTS[-1],
        "part_count": len(PRE_PARTS),
        "raw_sha256_equal_count": len(PRE_PARTS) - mismatch_count,
        "mismatch_count": mismatch_count,
        "mismatch_parts_prefix": mismatches,
        "comparison_canonical_sha256": digest.hexdigest(),
        "pass": not mismatches,
    }


def _checkpoint_equivalence() -> dict[str, Any]:
    values = {
        "a_base": _read_frame(A_BASE_ROOT / "data/Part_0701.bi4", 701),
        "a_restart_output": _read_frame(A_RESTART_ROOT / "data/Part_0701.bi4", 701),
        "b_continuous": _read_frame(B_CONTINUOUS_ROOT / "data/Part_0701.bi4", 701),
    }
    digests = {name: value["canonical_array_sha256"] for name, value in values.items()}
    times = {name: value["time_s"] for name, value in values.items()}
    return {
        "part": 701,
        "canonical_array_sha256": digests,
        "time_s": times,
        "pass": len(set(digests.values())) == 1 and max(times.values()) - min(times.values()) <= TOLERANCES["time_abs_max_s"],
    }


def _metric_result(maximum: float, sum_squares: float, count: int) -> dict[str, float]:
    if count <= 0:
        raise RepeatabilityRestartQcError("empty comparison metric")
    return {"max": maximum, "rms": math.sqrt(sum_squares / count)}


def compare_arrays(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    if left["ids"] != right["ids"]:
        raise RepeatabilityRestartQcError("ordered Id vector differs")
    pos_max = vel_max = rho_max = 0.0
    pos_ss = vel_ss = rho_ss = 0.0
    for left_pos, right_pos, left_vel, right_vel, left_rho, right_rho in zip(
        left["positions"], right["positions"], left["velocities"], right["velocities"], left["densities"], right["densities"]
    ):
        pos = math.sqrt(sum((a - b) ** 2 for a, b in zip(left_pos, right_pos)))
        vel = math.sqrt(sum((a - b) ** 2 for a, b in zip(left_vel, right_vel)))
        rho = abs(left_rho - right_rho) / max(abs(left_rho), abs(right_rho), 1.0)
        pos_max = max(pos_max, pos)
        vel_max = max(vel_max, vel)
        rho_max = max(rho_max, rho)
        pos_ss += pos * pos
        vel_ss += vel * vel
        rho_ss += rho * rho
    count = len(left["ids"])
    return {
        "particle_pairs": count,
        "time_abs_s": abs(left["time_s"] - right["time_s"]),
        "position_l2_m": _metric_result(pos_max, pos_ss, count),
        "velocity_l2_m_s": _metric_result(vel_max, vel_ss, count),
        "density_relative": _metric_result(rho_max, rho_ss, count),
    }


def _post_restart_equivalence() -> dict[str, Any]:
    pos_max = vel_max = rho_max = time_max = 0.0
    pos_ss = vel_ss = rho_ss = 0.0
    pairs = 0
    frame_digest = hashlib.sha256()
    for part in POST_PARTS:
        left = _read_frame(A_RESTART_ROOT / "data" / f"Part_{part:04d}.bi4", part)
        right = _read_frame(B_CONTINUOUS_ROOT / "data" / f"Part_{part:04d}.bi4", part)
        value = compare_arrays(left, right)
        count = value["particle_pairs"]
        pairs += count
        time_max = max(time_max, value["time_abs_s"])
        pos_max = max(pos_max, value["position_l2_m"]["max"])
        vel_max = max(vel_max, value["velocity_l2_m_s"]["max"])
        rho_max = max(rho_max, value["density_relative"]["max"])
        pos_ss += value["position_l2_m"]["rms"] ** 2 * count
        vel_ss += value["velocity_l2_m_s"]["rms"] ** 2 * count
        rho_ss += value["density_relative"]["rms"] ** 2 * count
        frame_digest.update(
            f"{part}:{left['canonical_array_sha256']}:{right['canonical_array_sha256']}\n".encode("ascii")
        )
    metrics = {
        "time_abs_max_s": time_max,
        "position_l2_max_m": pos_max,
        "position_l2_rms_m": math.sqrt(pos_ss / pairs),
        "velocity_l2_max_m_s": vel_max,
        "velocity_l2_rms_m_s": math.sqrt(vel_ss / pairs),
        "density_relative_max": rho_max,
        "density_relative_rms": math.sqrt(rho_ss / pairs),
    }
    passes = {name: metrics[name] <= TOLERANCES[name] for name in sorted(TOLERANCES)}
    return {
        "part_first": POST_PARTS[0],
        "part_last": POST_PARTS[-1],
        "frame_count": len(POST_PARTS),
        "particle_pairs": pairs,
        "metrics": metrics,
        "limits": TOLERANCES,
        "metric_pass": passes,
        "frame_comparison_canonical_sha256": frame_digest.hexdigest(),
        "pass": all(passes.values()),
    }


def adjudicate(pre_pass: bool, checkpoint_pass: bool, post_metrics: dict[str, float]) -> dict[str, Any]:
    if set(post_metrics) != set(TOLERANCES):
        raise RepeatabilityRestartQcError("post metric vector differs")
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < 0
        for value in post_metrics.values()
    ):
        raise RepeatabilityRestartQcError("post metric vector contains invalid values")
    metric_pass = {name: post_metrics[name] <= TOLERANCES[name] for name in sorted(TOLERANCES)}
    selected = pre_pass and checkpoint_pass and all(metric_pass.values())
    return {
        "selected": selected,
        "metric_pass": metric_pass,
        "status": "U3_SETTLED_STATE_FROZEN" if selected else "FAIL_U3_REPEATABILITY_OR_RESTART_EQUIVALENCE",
    }


def build_report() -> dict[str, Any]:
    inputs = _verify_frozen_inputs()
    pre = _raw_repeatability()
    checkpoint = _checkpoint_equivalence()
    post = _post_restart_equivalence()
    result = adjudicate(pre["pass"], checkpoint["pass"], post["metrics"])
    settled = _read_frame(B_CONTINUOUS_ROOT / "data/Part_0901.bi4", 901)
    report = {
        "schema_version": "r8-liquid-u3-viscoart0p3-repeatability-restart-qc-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_VISCOART0P3_REPEATABILITY_RESTART_QC_V1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            **inputs,
            "script": _identity(SCRIPT_PATH),
            "schema": _identity(SCHEMA_PATH),
            "tests": _identity(TEST_PATH),
        },
        "contract": {
            "dp_m": 0.001,
            "pre_checkpoint_raw_byte_equality_required": True,
            "checkpoint_canonical_array_equality_required": True,
            "post_part_first": 701,
            "post_part_last": 901,
            "post_frame_count": 201,
            "expected_particles": EXPECTED_PARTICLES,
            "tolerances": TOLERANCES,
            "threshold_overrides_allowed": False,
        },
        "pre_checkpoint_repeatability": pre,
        "checkpoint_equivalence": checkpoint,
        "post_checkpoint_equivalence": post,
        "settled_checkpoint": {
            "path": str(B_CONTINUOUS_ROOT / "data/Part_0901.bi4"),
            "file": _identity(B_CONTINUOUS_ROOT / "data/Part_0901.bi4", maximum=MAX_PART_BYTES),
            "canonical_array_sha256": settled["canonical_array_sha256"],
            "time_s": settled["time_s"],
            "particle_count": len(settled["ids"]),
            "backend": "GPU_0",
            "device_uuid": "GPU-d29cd22d-8b25-8f4e-9dcf-f2fc4bb113fb",
            "candidate_sha256": "cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f",
            "case_bi4_sha256": "b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7",
            "viscoart": 0.3,
            "cfl": 0.1,
            "shifting": "None",
        },
        "verdict": {
            "status": result["status"],
            "u3_settled_state_frozen": result["selected"],
            "cold_ab_repeatability_pass": pre["pass"],
            "checkpoint_restart_equivalence_pass": checkpoint["pass"] and post["pass"],
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "phase5_admitted": False,
            "solver_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
            "next": "FREEZE_CPU_GPU_PARITY_CONTRACT" if result["selected"] else "STOP",
        },
    }
    validate_report(report)
    return report


def validate_report(report: dict[str, Any]) -> None:
    pre = report.get("pre_checkpoint_repeatability", {})
    mismatch_count = pre.get("mismatch_count")
    equal_count = pre.get("raw_sha256_equal_count")
    mismatch_prefix = pre.get("mismatch_parts_prefix")
    if (
        not isinstance(mismatch_count, int)
        or isinstance(mismatch_count, bool)
        or not isinstance(equal_count, int)
        or isinstance(equal_count, bool)
        or not isinstance(mismatch_prefix, list)
        or mismatch_count < 0
        or equal_count < 0
        or mismatch_count + equal_count != len(PRE_PARTS)
        or len(mismatch_prefix) != min(mismatch_count, 32)
        or len(set(mismatch_prefix)) != len(mismatch_prefix)
        or any(not isinstance(part, int) or isinstance(part, bool) or part not in PRE_PARTS for part in mismatch_prefix)
        or pre.get("pass") is not (mismatch_count == 0)
    ):
        raise RepeatabilityRestartQcError("pre-checkpoint repeatability evidence is inconsistent")

    checkpoint = report.get("checkpoint_equivalence", {})
    digests = checkpoint.get("canonical_array_sha256", {})
    times = checkpoint.get("time_s", {})
    expected_checkpoint_pass = (
        isinstance(digests, dict)
        and set(digests) == {"a_base", "a_restart_output", "b_continuous"}
        and len(set(digests.values())) == 1
        and isinstance(times, dict)
        and set(times) == {"a_base", "a_restart_output", "b_continuous"}
        and all(
            not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))
            for value in times.values()
        )
        and max(times.values()) - min(times.values()) <= TOLERANCES["time_abs_max_s"]
    )
    if checkpoint.get("pass") is not expected_checkpoint_pass:
        raise RepeatabilityRestartQcError("checkpoint equivalence evidence is inconsistent")

    post = report.get("post_checkpoint_equivalence", {})
    metrics = post.get("metrics", {})
    result = adjudicate(
        pre.get("pass") is True,
        checkpoint.get("pass") is True,
        metrics,
    )
    expected_metric_pass = {name: metrics[name] <= TOLERANCES[name] for name in sorted(TOLERANCES)}
    if (
        post.get("limits") != TOLERANCES
        or post.get("metric_pass") != expected_metric_pass
        or post.get("pass") is not all(expected_metric_pass.values())
        or post.get("particle_pairs") != len(POST_PARTS) * EXPECTED_PARTICLES
    ):
        raise RepeatabilityRestartQcError("post-checkpoint equivalence evidence is inconsistent")
    verdict = report.get("verdict", {})
    if (
        verdict.get("status") != result["status"]
        or verdict.get("u3_settled_state_frozen") is not result["selected"]
        or verdict.get("cold_ab_repeatability_pass") is not (pre.get("pass") is True)
        or verdict.get("checkpoint_restart_equivalence_pass")
        is not (checkpoint.get("pass") is True and post.get("pass") is True)
        or verdict.get("next") != ("FREEZE_CPU_GPU_PARITY_CONTRACT" if result["selected"] else "STOP")
    ):
        raise RepeatabilityRestartQcError("verdict differs from frozen adjudication")
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(report), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise RepeatabilityRestartQcError(f"schema failure at {list(first.absolute_path)}: {first.message}")


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    if path != OUTPUT_PATH:
        raise RepeatabilityRestartQcError("output path differs from frozen v15 identity")
    data = (json.dumps(report, sort_keys=True, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()
    if len(data) > MAX_JSON_BYTES:
        raise RepeatabilityRestartQcError("receipt exceeds bound")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise RepeatabilityRestartQcError("short QC write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return _identity(path)


def static_self_check() -> dict[str, Any]:
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    if TOLERANCES["position_l2_max_m"] != 0.05 * 0.001:
        raise RepeatabilityRestartQcError("position tolerance is not frozen to 5% dp")
    if len(TOLERANCES) != 7 or len(PRE_PARTS) != 702 or len(POST_PARTS) != 201:
        raise RepeatabilityRestartQcError("comparison contract cardinality differs")
    return {
        "status": "PASS_U3_VISCOART0P3_REPEATABILITY_RESTART_QC_V1_STATIC_SELF_CHECK",
        "metric_count": len(TOLERANCES),
        "pre_frame_count": len(PRE_PARTS),
        "post_frame_count": len(POST_PARTS),
        "threshold_overrides_allowed": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--output", type=Path)
    result.add_argument("--self-check", action="store_true")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.self_check:
            print(json.dumps(static_self_check(), sort_keys=True))
            return 0
        if args.output is None:
            raise RepeatabilityRestartQcError("--output is required")
        report = build_report()
        identity = write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["u3_settled_state_frozen"] else 1
    except (
        RepeatabilityRestartQcError,
        legacy.Stage4RunError,
        bi4.Bi4FormatError,
        qc_v3.QcError,
        OSError,
        ValueError,
        UnicodeError,
        subprocess.SubprocessError,
    ) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
