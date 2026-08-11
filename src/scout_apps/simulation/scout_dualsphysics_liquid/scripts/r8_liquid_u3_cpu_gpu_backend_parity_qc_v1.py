#!/usr/bin/env python3
"""Read-only frozen CPU/GPU backend-parity QC for the 0--0.25 s window."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import stat
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_u3_gpu_stage4_runner_v4 as legacy
import r8_liquid_u3_solver_output_qc_v3 as qc_v3
import r8_liquid_u3_viscoart0p3_repeatability_restart_qc_v1 as settled_qc


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_cpu_gpu_backend_parity_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_cpu_gpu_backend_parity_qc_v1.py"
OUTPUT_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_cpu_gpu_backend_parity_20260811T133800Z_v19.qc_v1.json")

CPU_A_ROOT = Path("/home/zrj/scout_liquid_lab/stage4_experiments/u3_c1m_cpu_backend_parity_a_20260811T131411Z_v16.partial/output")
CPU_B_ROOT = Path("/home/zrj/scout_liquid_lab/stage4_experiments/u3_c1m_cpu_backend_parity_b_20260811T131411Z_v17.partial/output")
GPU_A_ROOT = Path("/home/zrj/scout_liquid_lab/stage4_experiments/u3_c1m_gpu_backend_parity_a_20260811T131411Z_v18.partial/output")
GPU_REFERENCE_ROOT = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_viscoart0p3_cold_b_20260811T113902Z_v14.partial/output")
PARTS = tuple(range(6))
EXPECTED_PARTICLES = 9078
FLUID_FIRST_ID = 2669
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_PART_BYTES = 1024 * 1024

TOLERANCES = {
    "time_abs_max_s": 0.0001,
    "position_l2_max_m": 0.00005,
    "position_l2_rms_m": 0.000005,
    "velocity_l2_max_m_s": 0.001,
    "velocity_l2_rms_m_s": 0.0001,
    "density_relative_max": 0.0005,
    "density_relative_rms": 0.00005,
}

FROZEN_INPUTS = {
    "plan": (
        PACKAGE_ROOT.parents[3] / "docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md",
        "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d",
    ),
    "settled_qc": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_repeatability_restart_20260811T124800Z_v15.qc_v1.json"),
        "4703c08e09b33fb16ad68b368dd5e99c2b2c930a3091008fa399241fc58a7fd8",
    ),
    "cpu_a_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_backend_parity_cpu_a_20260811T131411Z_v16.json",
        "072da1b10f6fa471300a91841e5fd70b1aa3d81b2566f07fcf4f73fc26d00942",
    ),
    "cpu_b_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_backend_parity_cpu_b_20260811T131411Z_v17.json",
        "4550caaf0d0b220dd220fc5e0b8a6e93b471c108e892a0a49408701b12b8ad0f",
    ),
    "gpu_a_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_backend_parity_gpu_a_20260811T131411Z_v18.json",
        "a8197eaa0f83ce35bac7b73a9a426f0e24f01ff5d62007905df27ffa8534c055",
    ),
    "cpu_a_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_cpu_backend_parity_a_20260811T131411Z_v16.final.json"),
        "4b459d209db18f745badb722a7fabffc66f9feccb0a981e5fdf59064c11d5ee4",
    ),
    "cpu_b_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_cpu_backend_parity_b_20260811T131411Z_v17.final.json"),
        "5d80776f0e17c76db4ef107e1c516cb08efc7d47d4207d1e9fedade96adc71ec",
    ),
    "gpu_a_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_backend_parity_a_20260811T131411Z_v18.final.json"),
        "bb39f66645fd25ccdb8498190001fbd88efe2696f3dd5878e15a79ebbbe2ee99",
    ),
    "gpu_reference_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_cold_b_20260811T113902Z_v14.final.json"),
        "98ad12bb4c7539b6b4ed78f578f39334e972b3c9bc90bd81cb15924e17be9acf",
    ),
}


class BackendParityQcError(ValueError):
    """Drifted, unsafe, incomplete or out-of-tolerance parity evidence."""


def _identity(path: Path, *, maximum: int = MAX_JSON_BYTES) -> dict[str, Any]:
    value = legacy.identity(path, maximum=maximum)
    return {key: value[key] for key in ("path", "sha256", "size_bytes", "mode", "inode", "nlink")}


def _verify_inputs() -> dict[str, dict[str, Any]]:
    values: dict[str, dict[str, Any]] = {}
    for name, (path, digest) in FROZEN_INPUTS.items():
        observed = _identity(path)
        if observed["sha256"] != digest:
            raise BackendParityQcError(f"frozen input hash differs: {name}")
        values[name] = observed
    expected = {
        "cpu_a_final": ("CPU", "u3_c1m_cpu_backend_parity_a_20260811T131411Z_v16", "072da1b10f6fa471300a91841e5fd70b1aa3d81b2566f07fcf4f73fc26d00942"),
        "cpu_b_final": ("CPU", "u3_c1m_cpu_backend_parity_b_20260811T131411Z_v17", "4550caaf0d0b220dd220fc5e0b8a6e93b471c108e892a0a49408701b12b8ad0f"),
        "gpu_a_final": ("GPU", "u3_c1m_gpu_backend_parity_a_20260811T131411Z_v18", "a8197eaa0f83ce35bac7b73a9a426f0e24f01ff5d62007905df27ffa8534c055"),
    }
    for name, (backend, run_id, policy_sha) in expected.items():
        receipt = legacy.read_json(FROZEN_INPUTS[name][0])
        valid = (
            receipt.get("document_type") == "SMPCC_R8_LIQUID_U3_STAGE4_EXPERIMENT_FINAL_V1"
            and receipt.get("status") == "PASS_U3_STAGE4_EXPERIMENT_RAW_OUTPUT"
            and receipt.get("backend") == backend
            and receipt.get("run_id") == run_id
            and receipt.get("returncode") == 0
            and receipt.get("termination_reason") == "PROCESS_EXIT"
            and receipt.get("policy", {}).get("sha256") == policy_sha
            and receipt.get("candidate_source_executed") is False
            and receipt.get("staged_candidate_executed") is True
            and receipt.get("network_used") is False
            and receipt.get("project_workspace_writable_in_guest") is False
            and receipt.get("apparmor_profile_used") is False
            and receipt.get("sudo_used") is False
            and receipt.get("compute_processes_after") == []
            and receipt.get("stderr", {}).get("sha256") == hashlib.sha256(b"").hexdigest()
            and receipt.get("output_inventory", {}).get("part_file_count") == 6
        )
        if not valid:
            raise BackendParityQcError(f"run receipt semantics differ: {name}")
    reference = legacy.read_json(FROZEN_INPUTS["gpu_reference_final"][0])
    if reference.get("status") != "PASS_GPU_STAGE4_RUN_RAW_OUTPUT" or reference.get("compute_processes_after") != []:
        raise BackendParityQcError("GPU repeatability reference semantics differ")
    settled = legacy.read_json(FROZEN_INPUTS["settled_qc"][0])
    if settled.get("verdict", {}).get("u3_settled_state_frozen") is not True:
        raise BackendParityQcError("settled-state parent is not PASS")
    return values


def _frame(root: Path, part: int) -> dict[str, Any]:
    return settled_qc._read_frame(root / "data" / f"Part_{part:04d}.bi4", part)


def _subset(frame: dict[str, Any], first_id: int) -> dict[str, Any]:
    offset = frame["ids"].index(first_id)
    return {
        "ids": frame["ids"][offset:],
        "time_s": frame["time_s"],
        "positions": frame["positions"][offset:],
        "velocities": frame["velocities"][offset:],
        "densities": frame["densities"][offset:],
    }


def _ordered_by_id(frame: dict[str, Any]) -> dict[str, Any]:
    order = sorted(range(len(frame["ids"])), key=frame["ids"].__getitem__)
    ids = [frame["ids"][index] for index in order]
    if ids != list(range(EXPECTED_PARTICLES)):
        raise BackendParityQcError("frame Id set differs before backend alignment")
    return {
        "ids": ids,
        "time_s": frame["time_s"],
        "positions": [frame["positions"][index] for index in order],
        "velocities": [frame["velocities"][index] for index in order],
        "densities": [frame["densities"][index] for index in order],
    }


def _aggregate(left_root: Path, right_root: Path, *, fluid_only: bool) -> dict[str, Any]:
    maxima = {name: 0.0 for name in TOLERANCES}
    sums = {"position_l2_rms_m": 0.0, "velocity_l2_rms_m_s": 0.0, "density_relative_rms": 0.0}
    pairs = 0
    digest = hashlib.sha256()
    for part in PARTS:
        left = _ordered_by_id(_frame(left_root, part))
        right = _ordered_by_id(_frame(right_root, part))
        if fluid_only:
            left = _subset(left, FLUID_FIRST_ID)
            right = _subset(right, FLUID_FIRST_ID)
        value = settled_qc.compare_arrays(left, right)
        count = value["particle_pairs"]
        pairs += count
        maxima["time_abs_max_s"] = max(maxima["time_abs_max_s"], value["time_abs_s"])
        maxima["position_l2_max_m"] = max(maxima["position_l2_max_m"], value["position_l2_m"]["max"])
        maxima["velocity_l2_max_m_s"] = max(maxima["velocity_l2_max_m_s"], value["velocity_l2_m_s"]["max"])
        maxima["density_relative_max"] = max(maxima["density_relative_max"], value["density_relative"]["max"])
        sums["position_l2_rms_m"] += value["position_l2_m"]["rms"] ** 2 * count
        sums["velocity_l2_rms_m_s"] += value["velocity_l2_m_s"]["rms"] ** 2 * count
        sums["density_relative_rms"] += value["density_relative"]["rms"] ** 2 * count
        digest.update(f"{part}:{left['time_s']:.17g}:{right['time_s']:.17g}:{count}\n".encode("ascii"))
    metrics = dict(maxima)
    for name, total in sums.items():
        metrics[name] = math.sqrt(total / pairs)
    passes = {name: metrics[name] <= TOLERANCES[name] for name in sorted(TOLERANCES)}
    return {
        "fluid_only": fluid_only,
        "frame_count": len(PARTS),
        "particle_pairs": pairs,
        "metrics": metrics,
        "limits": TOLERANCES,
        "metric_pass": passes,
        "comparison_canonical_sha256": digest.hexdigest(),
        "pass": all(passes.values()),
    }


def _repeatability(left_root: Path, right_root: Path, *, raw_required: bool) -> dict[str, Any]:
    raw_equal = canonical_equal = 0
    digest = hashlib.sha256()
    for part in PARTS:
        left = _frame(left_root, part)
        right = _frame(right_root, part)
        if left["sha256"] == right["sha256"]:
            raw_equal += 1
        if left["canonical_array_sha256"] == right["canonical_array_sha256"] and left["time_s"] == right["time_s"]:
            canonical_equal += 1
        digest.update(
            f"{part}:{left['sha256']}:{right['sha256']}:{left['canonical_array_sha256']}:{right['canonical_array_sha256']}\n".encode("ascii")
        )
    selected = canonical_equal == len(PARTS) and (not raw_required or raw_equal == len(PARTS))
    return {
        "frame_count": len(PARTS),
        "raw_equality_required": raw_required,
        "raw_equal_count": raw_equal,
        "canonical_equal_count": canonical_equal,
        "comparison_canonical_sha256": digest.hexdigest(),
        "pass": selected,
    }


def _resource_summary(final_path: Path) -> dict[str, Any]:
    final = legacy.read_json(final_path)
    path = Path(final["resource_log"]["path"])
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 < metadata.st_size <= MAX_JSON_BYTES:
        raise BackendParityQcError(f"unsafe resource log: {path}")
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
    if not rows:
        raise BackendParityQcError("resource log is empty")
    gpu_rows = [row["gpu"] for row in rows if isinstance(row.get("gpu"), dict)]
    return {
        "elapsed_seconds": final["elapsed_seconds"],
        "sample_count": len(rows),
        "minimum_mem_available_bytes": min(row["mem_available_bytes"] for row in rows),
        "minimum_swap_free_bytes": min(row["swap_free_bytes"] for row in rows),
        "observed_process_group_rss_peak_bytes": max(row["process_group_rss_bytes"] for row in rows),
        "gpu_memory_used_peak_mib": max((row["memory_used_mib"] for row in gpu_rows), default=None),
        "gpu_temperature_peak_c": max((row["temperature_c"] for row in gpu_rows), default=None),
        "resource_log": _identity(path),
        "rss_scope": "HOST_BWRAP_PROCESS_GROUP_OBSERVED_NOT_SOLVER_PEAK",
    }


def _solver_summary(root: Path, backend: str) -> dict[str, Any]:
    source = qc_v3._read(root / "Run.out", maximum=MAX_JSON_BYTES)
    text = source.data.decode("utf-8")
    runtime = re.findall(r"^Total Runtime\.+:\s*([0-9.]+) sec\.$", text, flags=re.MULTILINE)
    simulation = re.findall(r"^Simulation Runtime\.+:\s*([0-9.]+) sec\.$", text, flags=re.MULTILINE)
    cpu_memory = re.findall(r"^CPU Memory\.+:\s*([0-9,]+) \(", text, flags=re.MULTILINE)
    gpu_memory = re.findall(r"^GPU Memory\.+:\s*([0-9,]+) \(", text, flags=re.MULTILINE)
    wanted_mode = "Single-GPU" if backend == "GPU" else "OpenMP"
    if len(runtime) != 1 or len(simulation) != 1 or len(cpu_memory) != 1 or wanted_mode not in text:
        raise BackendParityQcError(f"solver summary differs: {backend}")
    partout = qc_v3.parse_partout(qc_v3._read(root / "data/PartOut_000.obi4", maximum=MAX_PART_BYTES))
    excluded = re.findall(r"^Excluded particles\.+:\s*([0-9,]+)$", text, flags=re.MULTILINE)
    if excluded != ["0"] or partout["total_nout"] != 0:
        raise BackendParityQcError(f"excluded-particle evidence differs: {backend}")
    return {
        "backend": backend,
        "run_out": _identity(root / "Run.out"),
        "total_runtime_seconds": float(runtime[0]),
        "simulation_runtime_seconds": float(simulation[0]),
        "reported_cpu_memory_bytes": int(cpu_memory[0].replace(",", "")),
        "reported_gpu_memory_bytes": None if not gpu_memory else int(gpu_memory[0].replace(",", "")),
        "excluded_particles": 0,
        "partout_total_nout": 0,
    }


def adjudicate(cpu_repeat: bool, gpu_repeat: bool, all_metrics: dict[str, float], fluid_metrics: dict[str, float]) -> dict[str, Any]:
    for metrics in (all_metrics, fluid_metrics):
        if set(metrics) != set(TOLERANCES) or any(
            isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or value < 0
            for value in metrics.values()
        ):
            raise BackendParityQcError("parity metric vector differs")
    all_pass = {name: all_metrics[name] <= TOLERANCES[name] for name in sorted(TOLERANCES)}
    fluid_pass = {name: fluid_metrics[name] <= TOLERANCES[name] for name in sorted(TOLERANCES)}
    selected = cpu_repeat and gpu_repeat and all(all_pass.values()) and all(fluid_pass.values())
    return {
        "selected": selected,
        "all_particle_metric_pass": all_pass,
        "fluid_metric_pass": fluid_pass,
        "status": "PASS_CPU_GPU_PARITY_DEVELOPMENT_ONLY" if selected else "FAIL_CPU_GPU_PARITY",
    }


def build_report() -> dict[str, Any]:
    inputs = _verify_inputs()
    cpu_repeat = _repeatability(CPU_A_ROOT, CPU_B_ROOT, raw_required=True)
    gpu_repeat = _repeatability(GPU_A_ROOT, GPU_REFERENCE_ROOT, raw_required=False)
    all_particles = _aggregate(CPU_A_ROOT, GPU_A_ROOT, fluid_only=False)
    fluid = _aggregate(CPU_A_ROOT, GPU_A_ROOT, fluid_only=True)
    result = adjudicate(cpu_repeat["pass"], gpu_repeat["pass"], all_particles["metrics"], fluid["metrics"])
    resources = {
        "cpu_a": _resource_summary(FROZEN_INPUTS["cpu_a_final"][0]),
        "cpu_b": _resource_summary(FROZEN_INPUTS["cpu_b_final"][0]),
        "gpu_a": _resource_summary(FROZEN_INPUTS["gpu_a_final"][0]),
    }
    solver = {
        "cpu_a": _solver_summary(CPU_A_ROOT, "CPU"),
        "cpu_b": _solver_summary(CPU_B_ROOT, "CPU"),
        "gpu_a": _solver_summary(GPU_A_ROOT, "GPU"),
    }
    cpu_elapsed_mean = (resources["cpu_a"]["elapsed_seconds"] + resources["cpu_b"]["elapsed_seconds"]) / 2
    report = {
        "schema_version": "r8-liquid-u3-cpu-gpu-backend-parity-qc-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_CPU_GPU_BACKEND_PARITY_QC_V1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {**inputs, "script": _identity(SCRIPT_PATH), "schema": _identity(SCHEMA_PATH), "tests": _identity(TEST_PATH)},
        "contract": {
            "window_start_s": 0.0,
            "window_end_s": 0.25,
            "output_period_s": 0.05,
            "part_first": 0,
            "part_last": 5,
            "part_count": 6,
            "expected_particles": EXPECTED_PARTICLES,
            "expected_fluid": EXPECTED_PARTICLES - FLUID_FIRST_ID,
            "tolerances": TOLERANCES,
            "threshold_overrides_allowed": False,
            "determinism_class": "ENVIRONMENT_SENSITIVE_BACKEND_WITH_DETERMINISTIC_WITHIN_BACKEND_REPEATS",
        },
        "repeatability": {"cpu": cpu_repeat, "gpu": gpu_repeat},
        "backend_comparison": {"all_particles": all_particles, "fluid_particles": fluid},
        "resources": {
            **resources,
            "cpu_elapsed_mean_seconds": cpu_elapsed_mean,
            "gpu_elapsed_seconds": resources["gpu_a"]["elapsed_seconds"],
            "wall_time_speedup_cpu_mean_over_gpu": cpu_elapsed_mean / resources["gpu_a"]["elapsed_seconds"],
        },
        "solver_summaries": solver,
        "verdict": {
            "status": result["status"],
            "backend_parity_pass": result["selected"],
            "cpu_repeatability_pass": cpu_repeat["pass"],
            "gpu_repeatability_pass": gpu_repeat["pass"],
            "all_particle_metric_pass": result["all_particle_metric_pass"],
            "fluid_metric_pass": result["fluid_metric_pass"],
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "phase5_admitted": False,
            "solver_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
            "next": "FREEZE_SETTLED_REPLAY_AND_SYNTHETIC_MOTION_CONTRACT" if result["selected"] else "STOP",
        },
    }
    validate_report(report)
    return report


def validate_report(report: dict[str, Any]) -> None:
    repeat = report.get("repeatability", {})
    comparison = report.get("backend_comparison", {})
    result = adjudicate(
        repeat.get("cpu", {}).get("pass") is True,
        repeat.get("gpu", {}).get("pass") is True,
        comparison.get("all_particles", {}).get("metrics", {}),
        comparison.get("fluid_particles", {}).get("metrics", {}),
    )
    verdict = report.get("verdict", {})
    if (
        verdict.get("status") != result["status"]
        or verdict.get("backend_parity_pass") is not result["selected"]
        or verdict.get("next") != ("FREEZE_SETTLED_REPLAY_AND_SYNTHETIC_MOTION_CONTRACT" if result["selected"] else "STOP")
    ):
        raise BackendParityQcError("verdict differs from frozen adjudication")
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(report), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise BackendParityQcError(f"schema failure at {list(first.absolute_path)}: {first.message}")


def write_exclusive(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    if path != OUTPUT_PATH:
        raise BackendParityQcError("output path differs from frozen v19 identity")
    data = (json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")
    if len(data) > MAX_JSON_BYTES:
        raise BackendParityQcError("QC receipt exceeds bound")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise BackendParityQcError("short QC write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return _identity(path)


def static_self_check() -> dict[str, Any]:
    schema = legacy.read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    if len(TOLERANCES) != 7 or TOLERANCES["position_l2_max_m"] != 0.05 * 0.001:
        raise BackendParityQcError("frozen parity tolerance contract differs")
    return {
        "status": "PASS_U3_CPU_GPU_BACKEND_PARITY_QC_V1_STATIC_SELF_CHECK",
        "metric_count": len(TOLERANCES),
        "part_count": len(PARTS),
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
            raise BackendParityQcError("--output is required")
        report = build_report()
        identity = write_exclusive(args.output, report)
        print(json.dumps({"status": report["verdict"]["status"], "result": identity}, sort_keys=True))
        return 0 if report["verdict"]["backend_parity_pass"] else 1
    except (
        BackendParityQcError,
        settled_qc.RepeatabilityRestartQcError,
        legacy.Stage4RunError,
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
