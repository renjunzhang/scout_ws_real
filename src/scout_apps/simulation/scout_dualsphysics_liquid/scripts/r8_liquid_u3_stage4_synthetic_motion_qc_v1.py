#!/usr/bin/env python3
"""Frozen read-only QC for the U3 Stage-4 zero/translation/yaw runs.

The tool parses existing BI4 outputs only.  It validates every particle and
motion-reference frame, reconstructs the prescribed 1 Hz rigid motion,
computes a 16-sector container-frame free-surface proxy, and adjudicates the
frozen response/free-decay limits.  It never executes DualSPHysics, exposes a
GPU, uses a network, or modifies any raw input.  Its CSV and JSON outputs are
create-new evidence and remain development-only numerical evidence.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import stat
import struct
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from jsonschema import Draft202012Validator

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy
import r8_liquid_u3_solver_output_qc_v3 as solver_qc
import r8_liquid_u3_viscoart0p3_repeatability_restart_qc_v1 as frame_reader


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_stage4_synthetic_motion_qc_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_stage4_synthetic_motion_qc_v1.py"
CONTRACT_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_synthetic_motion_contract_20260811T135543Z_v20.json"
OUTPUT_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_motion_20260811T143346Z_v23.qc_v1.json")
SERIES_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_motion_20260811T143346Z_v23.metrics_v1.csv")
PLAN_PATH = PACKAGE_ROOT.parents[3] / "docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md"

MAX_JSON_BYTES = 8 * 1024 * 1024
MAX_PART_BYTES = 1024 * 1024
MAX_MOTION_BYTES = 1024 * 1024
MAX_INPUT_BYTES = 256 * 1024 * 1024
EXPECTED_PARTICLES = 9078
MOVING_COUNT = 2669
FLUID_COUNT = 6409
START_TIME_S = 45.05001991890928
ACTIVE_DURATION_S = 2.0
TAIL_DURATION_S = 1.0
OUTPUT_PERIOD_S = 0.05
CONTAINER_RADIUS_M = 0.0185
NOMINAL_SURFACE_Z_M = 0.058
SECTOR_COUNT = 16

FROZEN_HASHES = {
    "plan": (PLAN_PATH, "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"),
    "motion_contract": (CONTRACT_PATH, "bd09947b118bcc2d4f13a292df517d7e3d8f2a643b9ecdfc2ab0dae68918d4dc"),
    "zero_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_settled_zero_replay_20260811T135543Z_v20.json",
        "09ab8c77e7dc0f4d284f1b65f3909d31a45cd5bfe6cac0fd8c8ddfe8faab8f8c",
    ),
    "translation_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_synthetic_translation_20260811T135543Z_v21.json",
        "ad088255a780145c3cb69644ce7b79463d37e08bca8d3cf7697b7a29c70f5668",
    ),
    "yaw_policy": (
        PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_synthetic_yaw_20260811T135543Z_v22.json",
        "57b190b647287004898d1c876a1eab3aadf3f52170a1cb4416df35b962210949",
    ),
    "zero_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_settled_zero_replay_20260811T135543Z_v20.final.json"),
        "f304c64d4f8babb096efa94aa822346f304800e1011024f5347ab9ee9bd6e0ff",
    ),
    "translation_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_translation_20260811T135543Z_v21.final.json"),
        "3e3cc024090b8bdeb362aaddcc335b5903e7ef935aac6da480d010c01b8c2df5",
    ),
    "yaw_final": (
        Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_yaw_20260811T135543Z_v22.final.json"),
        "9fb82f04a8e200dbe3b1cbc58bc0dfa64836f98adaa9ebe6e50aff257b00e082",
    ),
    "bi4_reader": (PACKAGE_ROOT / "scripts/r8_liquid_bi4_reader_v1.py", "104d271b750eed15985bf29f743a59678d66c50b9dfae9d1a3fd1663510dbc14"),
    "frame_reader": (PACKAGE_ROOT / "scripts/r8_liquid_u3_viscoart0p3_repeatability_restart_qc_v1.py", "f9bc035bc2e0ee3fdc18296b520d7292d9da1de289bca78a5d07eada06a0b627"),
    "solver_qc": (PACKAGE_ROOT / "scripts/r8_liquid_u3_solver_output_qc_v3.py", "ca2b4784731587f99e5a38e0f1f8ef6e3c6f0a103c81f6e2bbdc97c006619ced"),
    "legacy_identity": (PACKAGE_ROOT / "scripts/r8_liquid_u3_gpu_stage4_runner_v4.py", "bd3a21d818e2b104256462ae6b34a3bf4844ab2298adaa5ecedad1482f43c3c3"),
    "experiment_runner": (PACKAGE_ROOT / "scripts/r8_liquid_u3_stage4_experiment_runner_v1.py", "283ab03508c34142fa3fa19e687d571979ee8eec9e3f21c187689fe93d2e38ce"),
}

RUNS = {
    "zero": {
        "run_id": "u3_c1m_gpu_settled_zero_replay_20260811T135543Z_v20",
        "root": Path("/home/zrj/scout_liquid_lab/stage4_experiments/u3_c1m_gpu_settled_zero_replay_20260811T135543Z_v20.partial/output"),
        "policy_key": "zero_policy",
        "final_key": "zero_final",
        "motion_kind": "ZERO",
        "part_first": 901,
        "part_last": 921,
        "frequency_hz": 0.0,
        "amplitude": 0.0,
        "phase_rad": 0.0,
    },
    "translation": {
        "run_id": "u3_c1m_gpu_synthetic_translation_20260811T135543Z_v21",
        "root": Path("/home/zrj/scout_liquid_lab/stage4_experiments/u3_c1m_gpu_synthetic_translation_20260811T135543Z_v21.partial/output"),
        "policy_key": "translation_policy",
        "final_key": "translation_final",
        "motion_kind": "TRANSLATION",
        "part_first": 901,
        "part_last": 961,
        "frequency_hz": 1.0,
        "amplitude": 0.002,
        "phase_rad": 0.0,
    },
    "yaw": {
        "run_id": "u3_c1m_gpu_synthetic_yaw_20260811T135543Z_v22",
        "root": Path("/home/zrj/scout_liquid_lab/stage4_experiments/u3_c1m_gpu_synthetic_yaw_20260811T135543Z_v22.partial/output"),
        "policy_key": "yaw_policy",
        "final_key": "yaw_final",
        "motion_kind": "YAW",
        "part_first": 901,
        "part_last": 961,
        "frequency_hz": 1.0,
        "amplitude": 2.0,
        "phase_rad": 0.0,
    },
}

CANDIDATE_PATH = Path("/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T170339Z_a.partial/output/artifacts/DualSPHysics5.4_linux64")
CANDIDATE_SHA256 = "cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f"


class SyntheticMotionQcError(ValueError):
    """A frozen input, BI4 invariant, numerical contract, or output failed."""


def _identity(path: Path, *, maximum: int = MAX_INPUT_BYTES) -> dict[str, Any]:
    value = legacy.identity(path, maximum=maximum)
    return {key: value[key] for key in ("path", "sha256", "size_bytes", "mode", "inode", "nlink")}


def _canonical_sha256(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")).hexdigest()


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise SyntheticMotionQcError(f"non-finite numeric value: {name}")
    return float(value)


def _read_json(path: Path) -> dict[str, Any]:
    value = legacy.read_json(path)
    if not isinstance(value, dict):
        raise SyntheticMotionQcError(f"JSON root is not an object: {path}")
    return value


def _verify_contract(contract: dict[str, Any]) -> None:
    if contract.get("contract_id") != "u3_c1m_stage4_synthetic_motion_20260811T135543Z_v20":
        raise SyntheticMotionQcError("synthetic-motion contract identity differs")
    if contract.get("claim_ceiling") != "DEVELOPMENT_ONLY_SOFTWARE_NUMERICAL_NOT_PHYSICAL_FIDELITY":
        raise SyntheticMotionQcError("claim ceiling differs")
    checkpoint = contract.get("checkpoint", {})
    if (
        checkpoint.get("part_index") != 901
        or checkpoint.get("time_s") != START_TIME_S
        or checkpoint.get("particle_count") != EXPECTED_PARTICLES
        or checkpoint.get("moving_count") != MOVING_COUNT
        or checkpoint.get("fluid_count") != FLUID_COUNT
    ):
        raise SyntheticMotionQcError("checkpoint contract differs")
    expected_surface = {
        "method": "FLUID_PARTICLE_Z_QUANTILE_PER_CONTAINER_FRAME_AZIMUTH_SECTOR",
        "sector_count": 16,
        "annulus_inner_radius_fraction": 0.45,
        "annulus_outer_radius_fraction": 0.95,
        "surface_quantile": 0.98,
        "minimum_particles_per_sector": 128,
        "minimum_valid_fraction": 0.95,
        "frame": "INVERSE_ACTUAL_BOUNDARY_RIGID_TRANSFORM_CONTAINER_BODY",
        "center_radius_fraction": 0.25,
        "mid_annulus_inner_fraction": 0.25,
        "mid_annulus_outer_fraction": 0.55,
    }
    if contract.get("surface_proxy") != expected_surface:
        raise SyntheticMotionQcError("surface-proxy contract differs")
    acceptance = contract.get("acceptance", {})
    exact_acceptance = {
        "all_frames_required": True,
        "particle_id_set_exact": True,
        "nout_required": 0,
        "finite_required": True,
        "time_abs_tolerance_s": 0.0001,
        "zero_boundary_displacement_max_m": 1e-10,
        "translation_boundary_position_error_max_m": 0.00005,
        "yaw_boundary_position_error_max_m": 0.00005,
        "yaw_angle_error_max_deg": 0.05,
        "translation_surface_first_harmonic_min_m": 0.00001,
        "dynamic_fluid_speed_rms_over_zero_ratio_min": 1.1,
        "free_decay_initial_window_s": 0.4,
        "free_decay_final_window_s": 0.2,
        "free_decay_final_to_initial_peak_ke_ratio_max": 0.95,
        "input_frequency_abs_tolerance_hz": 0.05,
        "phase_report_required": True,
        "surface_metrics_required": ["crest", "absolute", "peak_to_peak", "peak", "p95", "rms", "first_azimuthal_harmonic"],
    }
    if acceptance != exact_acceptance:
        raise SyntheticMotionQcError("acceptance thresholds differ")
    for name, spec in RUNS.items():
        case = contract.get("cases", {}).get(name, {})
        if (
            case.get("run_id") != spec["run_id"]
            or case.get("motion_kind") != spec["motion_kind"]
            or case.get("part_first") != spec["part_first"]
            or case.get("part_last") != spec["part_last"]
            or case.get("frequency_hz") != spec["frequency_hz"]
        ):
            raise SyntheticMotionQcError(f"run case differs: {name}")


def _verify_output_inventory(root: Path, receipt: dict[str, Any]) -> str:
    inventory = receipt.get("output_inventory", {})
    expected = inventory.get("files")
    if not isinstance(expected, dict) or not expected:
        raise SyntheticMotionQcError("run receipt lacks output inventory")
    observed_paths: set[str] = set()
    for directory, dirnames, filenames in os.walk(root, topdown=True, followlinks=False):
        current = Path(directory)
        for dirname in list(dirnames):
            metadata = os.lstat(current / dirname)
            if not stat.S_ISDIR(metadata.st_mode):
                raise SyntheticMotionQcError(f"unsafe output directory entry: {current / dirname}")
        for filename in filenames:
            path = current / filename
            relative = path.relative_to(root).as_posix()
            observed_paths.add(relative)
            if relative not in expected:
                raise SyntheticMotionQcError(f"unexpected raw output: {relative}")
            metadata = os.lstat(path)
            item = expected[relative]
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise SyntheticMotionQcError(f"unsafe raw output identity: {relative}")
            if stat.S_IMODE(metadata.st_mode) != int(item["mode"], 8) or metadata.st_size != item["size_bytes"]:
                raise SyntheticMotionQcError(f"raw output mode/size differs: {relative}")
            digest = legacy.sha256_file(path, maximum=MAX_INPUT_BYTES)
            if digest != item["sha256"]:
                raise SyntheticMotionQcError(f"raw output hash differs: {relative}")
    if observed_paths != set(expected):
        missing = sorted(set(expected) - observed_paths)
        raise SyntheticMotionQcError(f"raw output inventory is incomplete: {missing[:4]}")
    canonical = inventory.get("canonical_sha256")
    if not isinstance(canonical, str) or len(canonical) != 64:
        raise SyntheticMotionQcError("output inventory canonical hash differs")
    return canonical


def verify_inputs(*, include_local_revision: bool = True) -> tuple[dict[str, dict[str, Any]], dict[str, Any], dict[str, dict[str, Any]]]:
    inputs: dict[str, dict[str, Any]] = {}
    for name, (path, expected) in FROZEN_HASHES.items():
        observed = _identity(path)
        if observed["sha256"] != expected:
            raise SyntheticMotionQcError(f"frozen input hash differs: {name}")
        inputs[name] = observed
    contract = _read_json(CONTRACT_PATH)
    _verify_contract(contract)
    inventories: dict[str, dict[str, Any]] = {}
    for name, spec in RUNS.items():
        receipt = _read_json(FROZEN_HASHES[spec["final_key"]][0])
        policy_sha = FROZEN_HASHES[spec["policy_key"]][1]
        part_count = spec["part_last"] - spec["part_first"] + 1
        if not (
            receipt.get("document_type") == "SMPCC_R8_LIQUID_U3_STAGE4_EXPERIMENT_FINAL_V1"
            and receipt.get("status") == "PASS_U3_STAGE4_EXPERIMENT_RAW_OUTPUT"
            and receipt.get("run_id") == spec["run_id"]
            and receipt.get("backend") == "GPU"
            and receipt.get("returncode") == 0
            and receipt.get("termination_reason") == "PROCESS_EXIT"
            and receipt.get("policy", {}).get("sha256") == policy_sha
            and receipt.get("candidate_source_executed") is False
            and receipt.get("staged_candidate_executed") is True
            and receipt.get("network_used") is False
            and receipt.get("sudo_used") is False
            and receipt.get("apparmor_profile_used") is False
            and receipt.get("project_workspace_writable_in_guest") is False
            and receipt.get("compute_processes_after") == []
            and receipt.get("stderr", {}).get("sha256") == hashlib.sha256(b"").hexdigest()
            and receipt.get("output_inventory", {}).get("part_file_count") == part_count
            and receipt.get("output_inventory", {}).get("part_first") == spec["part_first"]
            and receipt.get("output_inventory", {}).get("part_last") == spec["part_last"]
        ):
            raise SyntheticMotionQcError(f"raw run receipt semantics differ: {name}")
        source = receipt.get("candidate_source_after", {})
        if source.get("path") != str(CANDIDATE_PATH) or source.get("sha256") != CANDIDATE_SHA256 or source.get("mode") != "0400":
            raise SyntheticMotionQcError(f"candidate source identity differs in receipt: {name}")
        inventories[name] = {
            "canonical_sha256": _verify_output_inventory(spec["root"], receipt),
            "receipt": receipt,
        }
    candidate = _identity(CANDIDATE_PATH)
    if candidate["sha256"] != CANDIDATE_SHA256 or candidate["mode"] != "0400":
        raise SyntheticMotionQcError("candidate source changed or was re-armed")
    inputs["candidate_source"] = candidate
    if include_local_revision:
        for name, path in (("script", SCRIPT_PATH), ("schema", SCHEMA_PATH), ("tests", TEST_PATH)):
            inputs[name] = _identity(path)
    return inputs, contract, inventories


def read_frame(path: Path, expected_part: int) -> dict[str, Any]:
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise SyntheticMotionQcError(f"unsafe Part file: {path}")
    if stat.S_IMODE(metadata.st_mode) != 0o440 or not 1 <= metadata.st_size <= MAX_PART_BYTES:
        raise SyntheticMotionQcError(f"Part mode/size differs: {path}")
    source = bi4.read_regular_file(path, max_bytes=MAX_PART_BYTES)
    root = bi4.parse_jpartdata_bi4(source.data)
    part, ids, positions, velocities, densities = solver_qc._part_arrays(root)
    cpart = int(part.values.get("Cpart", -1))
    step = int(part.values.get("Step", -1))
    nout = int(part.values.get("Nout", -1))
    time_s = _finite(part.values.get("TimeStep"), "Part TimeStep")
    if cpart != expected_part or step < 0 or nout != 0 or len(ids) != EXPECTED_PARTICLES:
        raise SyntheticMotionQcError(f"Part identity/count/Nout differs: {path}")
    order = sorted(range(len(ids)), key=ids.__getitem__)
    ordered_ids = [ids[index] for index in order]
    if ordered_ids != list(range(EXPECTED_PARTICLES)):
        raise SyntheticMotionQcError(f"Part Id set differs: {path}")
    ordered_positions = [positions[index] for index in order]
    ordered_velocities = [velocities[index] for index in order]
    ordered_densities = [densities[index] for index in order]
    all_values: Iterable[float] = (
        value
        for collection in (ordered_positions, ordered_velocities)
        for vector in collection
        for value in vector
    )
    if any(not math.isfinite(value) for value in all_values) or any(not math.isfinite(value) for value in ordered_densities):
        raise SyntheticMotionQcError(f"Part contains non-finite data: {path}")
    digest = hashlib.sha256()
    for particle_id, position, velocity, density in zip(ordered_ids, ordered_positions, ordered_velocities, ordered_densities):
        digest.update(struct.pack(">Q7d", particle_id, *position, *velocity, density))
    return {
        "path": path,
        "sha256": source.sha256,
        "cpart": cpart,
        "step": step,
        "nout": nout,
        "time_s": time_s,
        "ids": ordered_ids,
        "positions": ordered_positions,
        "velocities": ordered_velocities,
        "densities": ordered_densities,
        "positions_by_id": dict(zip(ordered_ids, ordered_positions)),
        "canonical_array_sha256": digest.hexdigest(),
    }


def quantile(values: list[float], probability: float) -> float:
    if not values or not 0 <= probability <= 1 or any(not math.isfinite(value) for value in values):
        raise SyntheticMotionQcError("invalid quantile input")
    ordered = sorted(values)
    location = (len(ordered) - 1) * probability
    lower = int(math.floor(location))
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (location - lower) * (ordered[upper] - ordered[lower])


def _actual_transform(frame: dict[str, Any], baseline: dict[int, tuple[float, float, float]], motion_kind: str) -> dict[str, float]:
    positions = frame["positions"]
    velocities = frame["velocities"]
    if motion_kind == "TRANSLATION":
        dx = math.fsum(positions[index][0] - baseline[index][0] for index in range(MOVING_COUNT)) / MOVING_COUNT
        dy = math.fsum(positions[index][1] - baseline[index][1] for index in range(MOVING_COUNT)) / MOVING_COUNT
        dz = math.fsum(positions[index][2] - baseline[index][2] for index in range(MOVING_COUNT)) / MOVING_COUNT
        vx = math.fsum(velocities[index][0] for index in range(MOVING_COUNT)) / MOVING_COUNT
        vy = math.fsum(velocities[index][1] for index in range(MOVING_COUNT)) / MOVING_COUNT
        vz = math.fsum(velocities[index][2] for index in range(MOVING_COUNT)) / MOVING_COUNT
        return {"dx": dx, "dy": dy, "dz": dz, "theta": 0.0, "vx": vx, "vy": vy, "vz": vz, "omega": 0.0}
    if motion_kind == "YAW":
        cosine = math.fsum(
            baseline[index][0] * positions[index][0] + baseline[index][1] * positions[index][1]
            for index in range(MOVING_COUNT)
        )
        sine = math.fsum(
            baseline[index][0] * positions[index][1] - baseline[index][1] * positions[index][0]
            for index in range(MOVING_COUNT)
        )
        theta = math.atan2(sine, cosine)
        denominator = math.fsum(positions[index][0] ** 2 + positions[index][1] ** 2 for index in range(MOVING_COUNT))
        omega = math.fsum(
            -positions[index][1] * velocities[index][0] + positions[index][0] * velocities[index][1]
            for index in range(MOVING_COUNT)
        ) / denominator
        return {"dx": 0.0, "dy": 0.0, "dz": 0.0, "theta": theta, "vx": 0.0, "vy": 0.0, "vz": 0.0, "omega": omega}
    return {"dx": 0.0, "dy": 0.0, "dz": 0.0, "theta": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "omega": 0.0}


def expected_signal(spec: dict[str, Any], relative_time_s: float) -> float:
    if spec["motion_kind"] == "ZERO" or relative_time_s > ACTIVE_DURATION_S:
        return 0.0
    return spec["amplitude"] * math.sin(2 * math.pi * spec["frequency_hz"] * max(relative_time_s, 0.0) + spec["phase_rad"])


def _surface_metrics(positions: list[tuple[float, float, float]], transform: dict[str, float]) -> tuple[dict[str, Any], list[tuple[float, float, float]]]:
    cosine = math.cos(transform["theta"])
    sine = math.sin(transform["theta"])
    inner = 0.45 * CONTAINER_RADIUS_M
    outer = 0.95 * CONTAINER_RADIUS_M
    sectors: list[list[float]] = [[] for _ in range(SECTOR_COUNT)]
    body_positions: list[tuple[float, float, float]] = []
    for position in positions[MOVING_COUNT:]:
        x = position[0] - transform["dx"]
        y = position[1] - transform["dy"]
        body_x = cosine * x + sine * y
        body_y = -sine * x + cosine * y
        body_z = position[2] - transform["dz"]
        body_positions.append((body_x, body_y, body_z))
        radius = math.hypot(body_x, body_y)
        if inner <= radius <= outer:
            angle = math.atan2(body_y, body_x) % (2 * math.pi)
            index = min(SECTOR_COUNT - 1, int(angle * SECTOR_COUNT / (2 * math.pi)))
            sectors[index].append(body_z)
    if any(not sector for sector in sectors):
        raise SyntheticMotionQcError("surface proxy has an empty azimuth sector")
    heights = [quantile(sector, 0.98) for sector in sectors]
    counts = [len(sector) for sector in sectors]
    valid = sum(count >= 128 for count in counts)
    mean_height = math.fsum(heights) / SECTOR_COUNT
    cosine_component = math.fsum((height - mean_height) * math.cos(2 * math.pi * index / SECTOR_COUNT) for index, height in enumerate(heights))
    sine_component = math.fsum((height - mean_height) * math.sin(2 * math.pi * index / SECTOR_COUNT) for index, height in enumerate(heights))
    harmonic = 2 * math.hypot(cosine_component, sine_component) / SECTOR_COUNT
    phase = math.atan2(sine_component, cosine_component)
    deviations = [height - NOMINAL_SURFACE_Z_M for height in heights]
    return {
        "sector_counts": counts,
        "sector_heights_m": heights,
        "valid_fraction": valid / SECTOR_COUNT,
        "crest_above_nominal_m": max(heights) - NOMINAL_SURFACE_Z_M,
        "absolute_deviation_max_m": max(abs(value) for value in deviations),
        "peak_to_peak_m": max(heights) - min(heights),
        "peak_height_m": max(heights),
        "p95_height_m": quantile(heights, 0.95),
        "rms_deviation_m": math.sqrt(math.fsum(value * value for value in deviations) / SECTOR_COUNT),
        "first_azimuthal_harmonic_m": harmonic,
        "first_azimuthal_harmonic_phase_rad": phase,
    }, body_positions


def _frame_metric(frame: dict[str, Any], baseline: dict[int, tuple[float, float, float]], spec: dict[str, Any]) -> dict[str, Any]:
    relative = frame["time_s"] - START_TIME_S
    transform = _actual_transform(frame, baseline, spec["motion_kind"])
    signal_expected = expected_signal(spec, relative)
    if spec["motion_kind"] == "TRANSLATION":
        signal_name = "translation_x_m"
        signal_observed = transform["dx"]
        expected_theta = 0.0
        expected_dx = signal_expected
    elif spec["motion_kind"] == "YAW":
        signal_name = "yaw_solver_deg"
        signal_observed = -math.degrees(transform["theta"])
        expected_theta = -math.radians(signal_expected)
        expected_dx = 0.0
    else:
        signal_name = "zero"
        signal_observed = 0.0
        expected_theta = 0.0
        expected_dx = 0.0
    cosine_expected = math.cos(expected_theta)
    sine_expected = math.sin(expected_theta)
    position_error = 0.0
    for index in range(MOVING_COUNT):
        original = baseline[index]
        expected = (
            cosine_expected * original[0] - sine_expected * original[1] + expected_dx,
            sine_expected * original[0] + cosine_expected * original[1],
            original[2],
        )
        observed = frame["positions"][index]
        error = math.sqrt(math.fsum((observed[axis] - expected[axis]) ** 2 for axis in range(3)))
        position_error = max(position_error, error)
    surface, body_positions = _surface_metrics(frame["positions"], transform)
    cosine = math.cos(transform["theta"])
    sine = math.sin(transform["theta"])
    lab_sum = container_sum = tangential_sum = 0.0
    for offset, index in enumerate(range(MOVING_COUNT, EXPECTED_PARTICLES)):
        x, y, _ = frame["positions"][index]
        vel_x, vel_y, vel_z = frame["velocities"][index]
        lab_sum += vel_x * vel_x + vel_y * vel_y + vel_z * vel_z
        rigid_x = transform["vx"] - transform["omega"] * y
        rigid_y = transform["vy"] + transform["omega"] * x
        relative_x = vel_x - rigid_x
        relative_y = vel_y - rigid_y
        relative_z = vel_z - transform["vz"]
        body_vx = cosine * relative_x + sine * relative_y
        body_vy = -sine * relative_x + cosine * relative_y
        container_sum += body_vx * body_vx + body_vy * body_vy + relative_z * relative_z
        body_x, body_y, _ = body_positions[offset]
        radius = math.hypot(body_x, body_y)
        if radius:
            tangent = (-body_y * body_vx + body_x * body_vy) / radius
            tangential_sum += tangent * tangent
    lab_mean_square = lab_sum / FLUID_COUNT
    container_mean_square = container_sum / FLUID_COUNT
    return {
        "part": frame["cpart"],
        "time_s": frame["time_s"],
        "relative_time_s": relative,
        "step": frame["step"],
        "particle_count": EXPECTED_PARTICLES,
        "nout": frame["nout"],
        "all_finite": True,
        "boundary_signal_name": signal_name,
        "boundary_observed": signal_observed,
        "boundary_expected": signal_expected,
        "actual_translation_x_m": transform["dx"],
        "actual_yaw_ccw_rad": transform["theta"],
        "boundary_position_error_max_m": position_error,
        "fluid_speed_rms_lab_m_s": math.sqrt(lab_mean_square),
        "fluid_speed_rms_container_m_s": math.sqrt(container_mean_square),
        "fluid_tangential_speed_rms_container_m_s": math.sqrt(tangential_sum / FLUID_COUNT),
        "specific_kinetic_energy_lab_j_kg": 0.5 * lab_mean_square,
        "specific_kinetic_energy_container_j_kg": 0.5 * container_mean_square,
        "surface": surface,
    }


def _solve_three(matrix: list[list[float]], vector: list[float]) -> list[float]:
    values = [row[:] + [item] for row, item in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda index: abs(values[index][column]))
        if abs(values[pivot][column]) < 1e-18:
            raise SyntheticMotionQcError("singular harmonic fit")
        values[column], values[pivot] = values[pivot], values[column]
        scale = values[column][column]
        for index in range(column, 4):
            values[column][index] /= scale
        for row in range(3):
            if row == column:
                continue
            factor = values[row][column]
            for index in range(column, 4):
                values[row][index] -= factor * values[column][index]
    return [values[index][3] for index in range(3)]


def _fit_at_frequency(times: list[float], signals: list[float], frequency: float) -> tuple[float, list[float]]:
    rows = [[1.0, math.sin(2 * math.pi * frequency * time), math.cos(2 * math.pi * frequency * time)] for time in times]
    matrix = [[math.fsum(row[left] * row[right] for row in rows) for right in range(3)] for left in range(3)]
    vector = [math.fsum(row[index] * signal for row, signal in zip(rows, signals)) for index in range(3)]
    coefficients = _solve_three(matrix, vector)
    residual = math.fsum((signal - math.fsum(coefficient * value for coefficient, value in zip(coefficients, row))) ** 2 for signal, row in zip(signals, rows))
    return residual, coefficients


def _phase_distance(left: float, right: float) -> float:
    return abs((left - right + math.pi) % (2 * math.pi) - math.pi)


def frequency_fit(frames: list[dict[str, Any]], spec: dict[str, Any]) -> dict[str, Any]:
    signal_name = {"ZERO": "zero", "TRANSLATION": "translation_x_m", "YAW": "yaw_solver_deg"}[spec["motion_kind"]]
    if spec["motion_kind"] == "ZERO":
        return {
            "signal_name": signal_name,
            "requested_frequency_hz": 0.0,
            "observed_frequency_hz": 0.0,
            "frequency_abs_error_hz": 0.0,
            "requested_amplitude": 0.0,
            "observed_amplitude": 0.0,
            "requested_phase_rad": 0.0,
            "observed_phase_rad": 0.0,
            "phase_abs_error_rad": 0.0,
            "offset": 0.0,
            "fit_rmse": 0.0,
            "fit_r_squared": 1.0,
            "pass": True,
        }
    active = [frame for frame in frames if frame["relative_time_s"] <= ACTIVE_DURATION_S + 0.0001]
    times = [frame["relative_time_s"] for frame in active]
    signals = [frame["boundary_observed"] for frame in active]
    coarse = min(((_fit_at_frequency(times, signals, index / 1000), index / 1000) for index in range(500, 1501)), key=lambda item: item[0][0])
    center = coarse[1]
    refined = min(((_fit_at_frequency(times, signals, center - 0.002 + index / 100000), center - 0.002 + index / 100000) for index in range(401)), key=lambda item: item[0][0])
    (residual, coefficients), observed_frequency = refined
    observed_amplitude = math.hypot(coefficients[1], coefficients[2])
    observed_phase = math.atan2(coefficients[2], coefficients[1])
    mean_signal = math.fsum(signals) / len(signals)
    total = math.fsum((signal - mean_signal) ** 2 for signal in signals)
    r_squared = max(0.0, min(1.0, 1 - residual / total))
    frequency_error = abs(observed_frequency - spec["frequency_hz"])
    result = {
        "signal_name": signal_name,
        "requested_frequency_hz": spec["frequency_hz"],
        "observed_frequency_hz": observed_frequency,
        "frequency_abs_error_hz": frequency_error,
        "requested_amplitude": spec["amplitude"],
        "observed_amplitude": observed_amplitude,
        "requested_phase_rad": spec["phase_rad"],
        "observed_phase_rad": observed_phase,
        "phase_abs_error_rad": _phase_distance(observed_phase, spec["phase_rad"]),
        "offset": coefficients[0],
        "fit_rmse": math.sqrt(residual / len(signals)),
        "fit_r_squared": r_squared,
        "pass": frequency_error <= 0.05 and math.isfinite(observed_phase),
    }
    return result


def parse_motion_ref(path: Path, raw_frames: list[dict[str, Any]]) -> dict[str, Any]:
    source = bi4.read_regular_file(path, max_bytes=MAX_MOTION_BYTES)
    items = solver_qc._parse_jbd_list(source, "JPartMotRefBi4")
    root, appended = items[0], items[1:]
    if root.hidden or root.values_hidden or root.items:
        raise SyntheticMotionQcError("PartMotionRef root structure differs")
    expected_value_types = {
        "AppName": 1,
        "FormatVer": 8,
        "MainFile": 2,
        "TimeOut": 12,
        "MkBoundFirst": 6,
        "MkMovingCount": 8,
        "MkFloatCount": 8,
        "MkCount": 8,
    }
    if set(root.values) != set(expected_value_types):
        raise SyntheticMotionQcError("PartMotionRef root values differ")
    for name, type_code in expected_value_types.items():
        solver_qc._require_value_type(root, name, type_code)
    if (
        root.values.get("FormatVer") != 230729
        or root.values.get("MainFile") is not True
        or root.values.get("MkMovingCount") != 1
        or root.values.get("MkFloatCount") != 0
        or root.values.get("MkCount") != 1
        or root.values.get("MkBoundFirst") != 2
        or not solver_qc._same_float(_finite(root.values.get("TimeOut"), "PartMotionRef TimeOut"), OUTPUT_PERIOD_S)
    ):
        raise SyntheticMotionQcError("PartMotionRef root identity differs")
    expected_arrays = {"MkBound": (6, 1), "Nid": (8, 1), "Id": (8, 3), "Ps": (23, 3), "Dis": (12, 3)}
    if set(root.arrays) != set(expected_arrays):
        raise SyntheticMotionQcError("PartMotionRef root arrays differ")
    for name, (type_code, count) in expected_arrays.items():
        array = root.arrays[name]
        if array.type_code != type_code or array.count != count or array.hidden:
            raise SyntheticMotionQcError(f"PartMotionRef array identity differs: {name}")
    mkbounds = root.arrays["MkBound"].records()
    nids = root.arrays["Nid"].records()
    reference_ids = root.arrays["Id"].records()
    root_positions = root.arrays["Ps"].records()
    distances = root.arrays["Dis"].records()
    if mkbounds != [0] or nids != [3] or len(set(reference_ids)) != 3 or not all(0 <= value < MOVING_COUNT for value in reference_ids):
        raise SyntheticMotionQcError("PartMotionRef moving identity differs")
    expected_root_positions = [raw_frames[0]["positions_by_id"][value] for value in reference_ids]
    if root_positions != expected_root_positions:
        raise SyntheticMotionQcError("PartMotionRef root reference positions differ")
    expected_distances = [
        1.1 * math.sqrt(math.fsum((position[axis] - root_positions[0][axis]) ** 2 for axis in range(3)))
        for position in root_positions
    ]
    if not all(solver_qc._same_float(observed, expected) for observed, expected in zip(distances, expected_distances)):
        raise SyntheticMotionQcError("PartMotionRef reference distances differ")
    if len(appended) != len(raw_frames):
        raise SyntheticMotionQcError("PartMotionRef frame count differs")
    for item, frame in zip(appended, raw_frames):
        expected_name = f"PART_{frame['cpart']:04d}"
        if item.name != expected_name or not item.hidden or item.values_hidden or item.items or set(item.arrays) != {"PosRef"}:
            raise SyntheticMotionQcError(f"PartMotionRef item structure differs: {expected_name}")
        if set(item.values) != {"Cpart", "TimeStep", "Step"}:
            raise SyntheticMotionQcError(f"PartMotionRef item values differ: {expected_name}")
        for name, type_code in (("Cpart", 8), ("TimeStep", 12), ("Step", 8)):
            solver_qc._require_value_type(item, name, type_code)
        array = item.arrays["PosRef"]
        if array.type_code != 23 or array.count != 3 or array.hidden:
            raise SyntheticMotionQcError(f"PartMotionRef PosRef identity differs: {expected_name}")
        positions = array.records()
        if any(not math.isfinite(value) for vector in positions for value in vector):
            raise SyntheticMotionQcError(f"PartMotionRef PosRef is non-finite: {expected_name}")
        if (
            solver_qc._required_uint(item.values, "Cpart") != frame["cpart"]
            or solver_qc._required_uint(item.values, "Step") != frame["step"]
            or abs(_finite(item.values["TimeStep"], "PartMotionRef TimeStep") - frame["time_s"]) > 0.0001
            or positions != [frame["positions_by_id"][value] for value in reference_ids]
        ):
            raise SyntheticMotionQcError(f"PartMotionRef is misaligned: {expected_name}")
    return {
        "sha256": source.sha256,
        "size_bytes": source.size_bytes,
        "frame_count": len(appended),
        "reference_ids": reference_ids,
        "all_posref_finite": True,
        "aligned_with_particle_parts": True,
        "root_geometry_matched": True,
        "pass": True,
    }


def _root_mean_square(frames: list[dict[str, Any]], field: str) -> float:
    if not frames:
        raise SyntheticMotionQcError("empty RMS frame window")
    return math.sqrt(math.fsum(frame[field] ** 2 for frame in frames) / len(frames))


def analyze_run(name: str, spec: dict[str, Any], inventory_sha256: str) -> dict[str, Any]:
    raw_frames = [read_frame(spec["root"] / "data" / f"Part_{part:04d}.bi4", part) for part in range(spec["part_first"], spec["part_last"] + 1)]
    baseline = raw_frames[0]["positions_by_id"]
    frames = [_frame_metric(frame, baseline, spec) for frame in raw_frames]
    expected_times = [START_TIME_S + index * OUTPUT_PERIOD_S for index in range(len(frames))]
    time_error = max(abs(frame["time_s"] - expected) for frame, expected in zip(frames, expected_times))
    boundary_error = max(frame["boundary_position_error_max_m"] for frame in frames)
    zero_displacement = 0.0
    translation_yz_error = 0.0
    yaw_angle_error = 0.0
    for frame, raw in zip(frames, raw_frames):
        if spec["motion_kind"] == "ZERO":
            zero_displacement = max(
                zero_displacement,
                max(math.sqrt(math.fsum((raw["positions"][index][axis] - baseline[index][axis]) ** 2 for axis in range(3))) for index in range(MOVING_COUNT)),
            )
        if spec["motion_kind"] == "TRANSLATION":
            translation_yz_error = max(
                translation_yz_error,
                max(abs(raw["positions"][index][axis] - baseline[index][axis]) for index in range(MOVING_COUNT) for axis in (1, 2)),
            )
        if spec["motion_kind"] == "YAW":
            yaw_angle_error = max(yaw_angle_error, abs(frame["boundary_observed"] - frame["boundary_expected"]))
    if spec["motion_kind"] == "ZERO":
        boundary_pass = zero_displacement <= 1e-10 and boundary_error <= 1e-10
    elif spec["motion_kind"] == "TRANSLATION":
        boundary_pass = boundary_error <= 0.00005 and translation_yz_error <= 0.00005
    else:
        boundary_pass = boundary_error <= 0.00005 and yaw_angle_error <= 0.05
    fit = frequency_fit(frames, spec)
    motion_ref = parse_motion_ref(spec["root"] / "data/PartMotionRef.ibi4", raw_frames)
    active = [frame for frame in frames if frame["relative_time_s"] <= ACTIVE_DURATION_S + 0.0001]
    fluid = {
        "active_speed_rms_lab_m_s": _root_mean_square(active, "fluid_speed_rms_lab_m_s"),
        "active_speed_rms_container_m_s": _root_mean_square(active, "fluid_speed_rms_container_m_s"),
        "active_tangential_speed_rms_container_m_s": _root_mean_square(active, "fluid_tangential_speed_rms_container_m_s"),
        "peak_tangential_speed_rms_container_m_s": max(frame["fluid_tangential_speed_rms_container_m_s"] for frame in frames),
        "series_finite": True,
        "pass": True,
    }
    minimum_sector = min(min(frame["surface"]["sector_counts"]) for frame in frames)
    maximum_sector = max(max(frame["surface"]["sector_counts"]) for frame in frames)
    minimum_valid_fraction = min(frame["surface"]["valid_fraction"] for frame in frames)
    valid_count = sum(frame["surface"]["valid_fraction"] >= 0.95 for frame in frames)
    surface = {
        "valid_frame_count": valid_count,
        "frame_count": len(frames),
        "minimum_sector_particles_observed": minimum_sector,
        "maximum_sector_particles_observed": maximum_sector,
        "minimum_valid_fraction_observed": minimum_valid_fraction,
        "peak_first_azimuthal_harmonic_m": max(frame["surface"]["first_azimuthal_harmonic_m"] for frame in frames),
        "peak_to_peak_max_m": max(frame["surface"]["peak_to_peak_m"] for frame in frames),
        "all_required_metrics_finite": True,
        "pass": valid_count == len(frames) and minimum_sector >= 128,
    }
    result = {
        "run_id": spec["run_id"],
        "motion_kind": spec["motion_kind"],
        "output_inventory_canonical_sha256": inventory_sha256,
        "part_first": spec["part_first"],
        "part_last": spec["part_last"],
        "frame_count": len(frames),
        "particle_count_all_frames": EXPECTED_PARTICLES,
        "exact_id_set_all_frames": True,
        "nout_zero_all_frames": True,
        "finite_all_frames": True,
        "time_alignment_max_abs_s": time_error,
        "boundary": {
            "motion_kind": spec["motion_kind"],
            "position_error_max_m": boundary_error,
            "zero_displacement_max_m": zero_displacement,
            "translation_yz_error_max_m": translation_yz_error,
            "yaw_angle_error_max_deg": yaw_angle_error,
            "observed_peak_translation_m": max(abs(frame["actual_translation_x_m"]) for frame in frames),
            "observed_peak_yaw_deg": max(abs(math.degrees(frame["actual_yaw_ccw_rad"])) for frame in frames),
            "all_expected_rigid_transform": boundary_pass,
            "pass": boundary_pass,
        },
        "frequency_fit": fit,
        "motion_ref": motion_ref,
        "fluid": fluid,
        "surface": surface,
        "frames_canonical_sha256": _canonical_sha256(frames),
        "frames": frames,
        "pass": boundary_pass and fit["pass"] and motion_ref["pass"] and fluid["pass"] and surface["pass"] and time_error <= 0.0001,
    }
    return result


def _free_decay(run: dict[str, Any]) -> dict[str, Any]:
    initial_start = ACTIVE_DURATION_S
    initial_end = ACTIVE_DURATION_S + 0.4
    final_start = ACTIVE_DURATION_S + TAIL_DURATION_S - 0.2
    final_end = ACTIVE_DURATION_S + TAIL_DURATION_S
    initial = [frame["specific_kinetic_energy_lab_j_kg"] for frame in run["frames"] if initial_start - 0.0001 <= frame["relative_time_s"] <= initial_end + 0.0001]
    final = [frame["specific_kinetic_energy_lab_j_kg"] for frame in run["frames"] if final_start - 0.0001 <= frame["relative_time_s"] <= final_end + 0.0001]
    if not initial or not final or max(initial) <= 0:
        raise SyntheticMotionQcError("free-decay window is empty or invalid")
    final_mean = math.fsum(final) / len(final)
    ratio = final_mean / max(initial)
    return {
        "initial_window_start_s": initial_start,
        "initial_window_end_s": initial_end,
        "final_window_start_s": final_start,
        "final_window_end_s": final_end,
        "initial_peak_specific_ke_lab_j_kg": max(initial),
        "final_mean_specific_ke_lab_j_kg": final_mean,
        "final_to_initial_peak_ratio": ratio,
        "pass": ratio <= 0.95,
    }


def build_comparisons(runs: dict[str, Any]) -> dict[str, Any]:
    zero_frames = runs["zero"]["frames"]
    zero_speed = _root_mean_square(zero_frames, "fluid_speed_rms_lab_m_s")
    first_surface = zero_frames[0]["surface"]["sector_heights_m"]
    maximum_surface_drift = max(
        abs(height - first_surface[index])
        for frame in zero_frames
        for index, height in enumerate(frame["surface"]["sector_heights_m"])
    )
    initial_ke = zero_frames[0]["specific_kinetic_energy_lab_j_kg"]
    final_ke = zero_frames[-1]["specific_kinetic_energy_lab_j_kg"]
    zero_stability = {
        "initial_specific_ke_lab_j_kg": initial_ke,
        "final_specific_ke_lab_j_kg": final_ke,
        "final_to_initial_ke_ratio": final_ke / initial_ke,
        "maximum_sector_height_drift_m": maximum_surface_drift,
        "boundary_displacement_max_m": runs["zero"]["boundary"]["zero_displacement_max_m"],
        "non_growth_observed": final_ke <= initial_ke,
        "pass": runs["zero"]["pass"] and final_ke <= initial_ke,
    }
    responses: dict[str, dict[str, Any]] = {}
    for name in ("translation", "yaw"):
        run = runs[name]
        ratio = run["fluid"]["active_speed_rms_lab_m_s"] / zero_speed
        harmonic_required = name == "translation"
        harmonic_pass = (run["surface"]["peak_first_azimuthal_harmonic_m"] >= 0.00001) if harmonic_required else True
        speed_pass = ratio >= 1.1
        responses[name] = {
            "active_speed_rms_lab_m_s": run["fluid"]["active_speed_rms_lab_m_s"],
            "zero_speed_rms_lab_m_s": zero_speed,
            "dynamic_over_zero_ratio": ratio,
            "peak_surface_first_harmonic_m": run["surface"]["peak_first_azimuthal_harmonic_m"],
            "peak_tangential_speed_rms_container_m_s": run["fluid"]["peak_tangential_speed_rms_container_m_s"],
            "speed_ratio_pass": speed_pass,
            "surface_harmonic_required": harmonic_required,
            "surface_harmonic_pass": harmonic_pass,
            "interpretation": (
                "Translation response is evidenced by lab-frame fluid speed and the frozen first azimuthal surface harmonic."
                if name == "translation"
                else "Yaw of an axisymmetric cylinder is adjudicated by tangential flow, kinetic energy, and rigid-motion reconstruction; a large surface tilt is not required."
            ),
            "pass": run["pass"] and speed_pass and harmonic_pass,
        }
    translation_decay = _free_decay(runs["translation"])
    yaw_decay = _free_decay(runs["yaw"])
    return {
        "zero_stability": zero_stability,
        "translation_response": responses["translation"],
        "yaw_response": responses["yaw"],
        "free_decay": {"translation": translation_decay, "yaw": yaw_decay, "pass": translation_decay["pass"] and yaw_decay["pass"]},
        "input_reconstruction": {
            "translation": runs["translation"]["frequency_fit"],
            "yaw": runs["yaw"]["frequency_fit"],
            "pass": runs["translation"]["frequency_fit"]["pass"] and runs["yaw"]["frequency_fit"]["pass"],
        },
    }


def adjudicate(runs: dict[str, Any], comparisons: dict[str, Any]) -> bool:
    if set(runs) != {"zero", "translation", "yaw"} or not all(run.get("pass") is True for run in runs.values()):
        return False
    if runs["zero"]["boundary"]["zero_displacement_max_m"] > 1e-10:
        return False
    if runs["translation"]["boundary"]["position_error_max_m"] > 0.00005:
        return False
    if runs["yaw"]["boundary"]["position_error_max_m"] > 0.00005 or runs["yaw"]["boundary"]["yaw_angle_error_max_deg"] > 0.05:
        return False
    if any(run["time_alignment_max_abs_s"] > 0.0001 or run["surface"]["minimum_valid_fraction_observed"] < 0.95 for run in runs.values()):
        return False
    if any(runs[name]["frequency_fit"]["frequency_abs_error_hz"] > 0.05 for name in ("translation", "yaw")):
        return False
    if comparisons.get("zero_stability", {}).get("pass") is not True:
        return False
    if comparisons.get("translation_response", {}).get("dynamic_over_zero_ratio", 0) < 1.1:
        return False
    if comparisons.get("translation_response", {}).get("peak_surface_first_harmonic_m", 0) < 0.00001:
        return False
    if comparisons.get("translation_response", {}).get("pass") is not True:
        return False
    if comparisons.get("yaw_response", {}).get("dynamic_over_zero_ratio", 0) < 1.1 or comparisons.get("yaw_response", {}).get("pass") is not True:
        return False
    decay = comparisons.get("free_decay", {})
    if decay.get("pass") is not True or any(decay.get(name, {}).get("final_to_initial_peak_ratio", math.inf) > 0.95 for name in ("translation", "yaw")):
        return False
    if comparisons.get("input_reconstruction", {}).get("pass") is not True:
        return False
    return True


def _contract_receipt(contract: dict[str, Any]) -> dict[str, Any]:
    return {
        "contract_id": contract["contract_id"],
        "claim_ceiling": contract["claim_ceiling"],
        "start_time_s": START_TIME_S,
        "active_duration_s": ACTIVE_DURATION_S,
        "tail_duration_s": TAIL_DURATION_S,
        "output_period_s": OUTPUT_PERIOD_S,
        "particle_count": EXPECTED_PARTICLES,
        "moving_count": MOVING_COUNT,
        "fluid_count": FLUID_COUNT,
        "container_radius_m": CONTAINER_RADIUS_M,
        "nominal_surface_z_m": NOMINAL_SURFACE_Z_M,
        "part_ranges": {
            name: {"first": spec["part_first"], "last": spec["part_last"], "count": spec["part_last"] - spec["part_first"] + 1}
            for name, spec in RUNS.items()
        },
        "surface_proxy": contract["surface_proxy"],
        "acceptance": contract["acceptance"],
        "threshold_overrides_allowed": False,
    }


def build_report(
    inputs: dict[str, dict[str, Any]],
    contract: dict[str, Any],
    inventories: dict[str, dict[str, Any]],
    series_identity: dict[str, Any],
    captured_at_utc: str,
) -> dict[str, Any]:
    runs = {name: analyze_run(name, spec, inventories[name]["canonical_sha256"]) for name, spec in RUNS.items()}
    comparisons = build_comparisons(runs)
    selected = adjudicate(runs, comparisons)
    report = {
        "schema_version": "r8-liquid-u3-stage4-synthetic-motion-qc-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_STAGE4_SYNTHETIC_MOTION_QC_V1",
        "captured_at_utc": captured_at_utc,
        "inputs": inputs,
        "contract": _contract_receipt(contract),
        "runs": runs,
        "comparisons": comparisons,
        "series_output": series_identity,
        "verdict": {
            "status": "PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION" if selected else "FAIL_U3_STAGE4_SYNTHETIC_MOTION_QC",
            "stage4_liquid_only_validation_complete": selected,
            "development_only": True,
            "formal": False,
            "physical_fidelity_validated": False,
            "production_eligible": False,
            "stage5_admitted": False,
            "all_frozen_thresholds_pass": selected,
            "candidate_source_executed": False,
            "solver_executed_by_this_tool": False,
            "gpu_exposed_by_this_tool": False,
            "network_used_by_this_tool": False,
            "inputs_modified": False,
            "next": "STAGE5_REQUIRES_SEPARATE_USER_AUTHORIZATION_AND_PHYSICAL_INPUTS" if selected else "STOP_REVIEW_STAGE4_QC_FAILURE",
        },
    }
    validate_report(report)
    return report


def _all_finite(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_all_finite(item) for item in value.values())
    if isinstance(value, list):
        return all(_all_finite(item) for item in value)
    return True


def validate_report(report: dict[str, Any]) -> None:
    if not _all_finite(report):
        raise SyntheticMotionQcError("report contains a non-finite number")
    selected = adjudicate(report.get("runs", {}), report.get("comparisons", {}))
    verdict = report.get("verdict", {})
    if (
        verdict.get("all_frozen_thresholds_pass") is not selected
        or verdict.get("stage4_liquid_only_validation_complete") is not selected
        or verdict.get("status") != ("PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION" if selected else "FAIL_U3_STAGE4_SYNTHETIC_MOTION_QC")
        or verdict.get("next") != ("STAGE5_REQUIRES_SEPARATE_USER_AUTHORIZATION_AND_PHYSICAL_INPUTS" if selected else "STOP_REVIEW_STAGE4_QC_FAILURE")
    ):
        raise SyntheticMotionQcError("verdict differs from frozen adjudication")
    schema = _read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(report), key=lambda error: list(error.absolute_path))
    if errors:
        first = errors[0]
        raise SyntheticMotionQcError(f"schema failure at {list(first.absolute_path)}: {first.message}")


CSV_FIELDS = [
    "run",
    "part",
    "time_s",
    "relative_time_s",
    "boundary_signal_name",
    "boundary_observed",
    "boundary_expected",
    "boundary_position_error_max_m",
    "fluid_speed_rms_lab_m_s",
    "fluid_speed_rms_container_m_s",
    "fluid_tangential_speed_rms_container_m_s",
    "specific_kinetic_energy_lab_j_kg",
    "specific_kinetic_energy_container_j_kg",
    "surface_crest_above_nominal_m",
    "surface_absolute_deviation_max_m",
    "surface_peak_to_peak_m",
    "surface_peak_height_m",
    "surface_p95_height_m",
    "surface_rms_deviation_m",
    "surface_first_azimuthal_harmonic_m",
    "surface_first_azimuthal_harmonic_phase_rad",
] + [f"surface_sector_{index:02d}_height_m" for index in range(SECTOR_COUNT)]


def series_csv_bytes(runs: dict[str, Any]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    for name in ("zero", "translation", "yaw"):
        for frame in runs[name]["frames"]:
            surface = frame["surface"]
            row = {
                "run": name,
                "part": frame["part"],
                "time_s": format(frame["time_s"], ".17g"),
                "relative_time_s": format(frame["relative_time_s"], ".17g"),
                "boundary_signal_name": frame["boundary_signal_name"],
                "boundary_observed": format(frame["boundary_observed"], ".17g"),
                "boundary_expected": format(frame["boundary_expected"], ".17g"),
                "boundary_position_error_max_m": format(frame["boundary_position_error_max_m"], ".17g"),
                "fluid_speed_rms_lab_m_s": format(frame["fluid_speed_rms_lab_m_s"], ".17g"),
                "fluid_speed_rms_container_m_s": format(frame["fluid_speed_rms_container_m_s"], ".17g"),
                "fluid_tangential_speed_rms_container_m_s": format(frame["fluid_tangential_speed_rms_container_m_s"], ".17g"),
                "specific_kinetic_energy_lab_j_kg": format(frame["specific_kinetic_energy_lab_j_kg"], ".17g"),
                "specific_kinetic_energy_container_j_kg": format(frame["specific_kinetic_energy_container_j_kg"], ".17g"),
                "surface_crest_above_nominal_m": format(surface["crest_above_nominal_m"], ".17g"),
                "surface_absolute_deviation_max_m": format(surface["absolute_deviation_max_m"], ".17g"),
                "surface_peak_to_peak_m": format(surface["peak_to_peak_m"], ".17g"),
                "surface_peak_height_m": format(surface["peak_height_m"], ".17g"),
                "surface_p95_height_m": format(surface["p95_height_m"], ".17g"),
                "surface_rms_deviation_m": format(surface["rms_deviation_m"], ".17g"),
                "surface_first_azimuthal_harmonic_m": format(surface["first_azimuthal_harmonic_m"], ".17g"),
                "surface_first_azimuthal_harmonic_phase_rad": format(surface["first_azimuthal_harmonic_phase_rad"], ".17g"),
            }
            row.update({f"surface_sector_{index:02d}_height_m": format(value, ".17g") for index, value in enumerate(surface["sector_heights_m"])})
            writer.writerow(row)
    return stream.getvalue().encode("utf-8")


def _planned_identity(path: Path, data: bytes, mode: int) -> dict[str, Any]:
    return {
        "path": str(path),
        "sha256": hashlib.sha256(data).hexdigest(),
        "size_bytes": len(data),
        "mode": f"{mode:04o}",
        "inode": 1,
        "nlink": 1,
    }


def write_exclusive(path: Path, data: bytes, *, allowed: set[Path]) -> dict[str, Any]:
    if path not in allowed:
        raise SyntheticMotionQcError("output path differs from the frozen QC identity")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise SyntheticMotionQcError("short create-new evidence write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return _identity(path)


def static_self_check() -> dict[str, Any]:
    schema = _read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    if schema.get("additionalProperties") is not False:
        raise SyntheticMotionQcError("receipt schema is not closed")
    for name, definition in schema.get("$defs", {}).items():
        if definition.get("type") == "object" and definition.get("additionalProperties") is not False:
            raise SyntheticMotionQcError(f"schema object is not closed: {name}")
    _, contract, _ = verify_inputs(include_local_revision=False)
    _verify_contract(contract)
    if len(RUNS) != 3 or SECTOR_COUNT != 16 or MOVING_COUNT + FLUID_COUNT != EXPECTED_PARTICLES:
        raise SyntheticMotionQcError("static cardinality contract differs")
    return {
        "status": "PASS_U3_STAGE4_SYNTHETIC_MOTION_QC_V1_STATIC_SELF_CHECK",
        "run_count": 3,
        "frame_count": sum(spec["part_last"] - spec["part_first"] + 1 for spec in RUNS.values()),
        "particle_count": EXPECTED_PARTICLES,
        "surface_sector_count": SECTOR_COUNT,
        "minimum_surface_particles_per_sector": 128,
        "threshold_overrides_allowed": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("command", choices=("self-check", "run"))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "self-check":
            print(json.dumps(static_self_check(), sort_keys=True))
            return 0
        for path in (SERIES_PATH, OUTPUT_PATH):
            try:
                os.lstat(path)
            except FileNotFoundError:
                continue
            raise SyntheticMotionQcError(f"create-new output already exists: {path}")
        inputs, contract, inventories = verify_inputs()
        captured_at = datetime.now(timezone.utc).isoformat()
        placeholder = _planned_identity(SERIES_PATH, b"placeholder\n", 0o640)
        report = build_report(inputs, contract, inventories, placeholder, captured_at)
        csv_data = series_csv_bytes(report["runs"])
        planned = _planned_identity(SERIES_PATH, csv_data, 0o640)
        report = build_report(inputs, contract, inventories, planned, captured_at)
        csv_data_check = series_csv_bytes(report["runs"])
        if csv_data_check != csv_data:
            raise SyntheticMotionQcError("series CSV is not deterministic")
        series_identity = write_exclusive(SERIES_PATH, csv_data, allowed={SERIES_PATH, OUTPUT_PATH})
        report["series_output"] = series_identity
        validate_report(report)
        json_data = (json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")
        if len(json_data) > MAX_JSON_BYTES:
            raise SyntheticMotionQcError("QC receipt exceeds size bound")
        receipt_identity = write_exclusive(OUTPUT_PATH, json_data, allowed={SERIES_PATH, OUTPUT_PATH})
        print(json.dumps({"status": report["verdict"]["status"], "receipt": receipt_identity, "series": series_identity}, sort_keys=True))
        return 0 if report["verdict"]["all_frozen_thresholds_pass"] else 1
    except (
        SyntheticMotionQcError,
        frame_reader.RepeatabilityRestartQcError,
        solver_qc.QcError,
        legacy.Stage4RunError,
        bi4.Bi4FormatError,
        FileExistsError,
        OSError,
        ValueError,
        UnicodeError,
        subprocess.SubprocessError,
    ) as exc:
        print(json.dumps({"status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
