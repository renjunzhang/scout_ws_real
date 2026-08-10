#!/usr/bin/env python3
"""Read-only U3 restart-equivalence QC for four DualSPHysics PART files.

The continuous run and the fresh restart each supply ``Part_0160.bi4`` and
``Part_0161.bi4``.  Retaining the original part numbering is required by the
case's simple-motion clock.  Particle
arrays are aligned by Idp before comparison.  The only write is a new JSON
result created with O_EXCL.  Results are always development-only and labelled
SIM_ONLY_UNVALIDATED.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import r8_liquid_bi4_reader_v1 as bi4


CLASSIFICATION = "SIM_ONLY_UNVALIDATED"
EXPECTED_PARTICLES = 9_078
EXPECTED_MOVING = 2_669
EXPECTED_NOUT = 0
EXPECTED_CHECKPOINT_TIME_S = 8.0
EXPECTED_EVOLUTION_INTERVAL_S = 0.05
TIME_TOLERANCE_S = 1e-4
ARRAY_TYPES = {"Idp": 8, "Posd": 23, "Vel": 22, "Rhop": 11}
ARRAY_RECORD_SIZES = {"Posd": 24, "Vel": 12, "Rhop": 4}

EVOLUTION_TOLERANCES = {
    "Posd": {"atol": 1e-12, "rtol": 1e-12},
    "Vel": {"atol": 2e-7, "rtol": 2e-6},
    "Rhop": {"atol": 2e-4, "rtol": 2e-7},
    "SymplecticDtPre": {"atol": 1e-15, "rtol": 1e-12},
}


class RestartQcError(ValueError):
    """Raised when an input cannot satisfy the fixed U3 data contract."""


@dataclass(frozen=True)
class Frame:
    path: Path
    sha256: str
    size_bytes: int
    cpart: int
    time_s: float
    step: int
    symplectic_dt_pre: float
    ids: tuple[int, ...]
    values_by_id: dict[str, dict[int, Any]]
    raw_by_id: dict[str, dict[int, bytes]]

    def summary(self) -> dict[str, Any]:
        return {
            "path": str(self.path),
            "sha256": self.sha256,
            "size_bytes": self.size_bytes,
            "cpart": self.cpart,
            "time_s": self.time_s,
            "step": self.step,
            "symplectic_dt_pre_s": self.symplectic_dt_pre,
            "particle_count": len(self.ids),
            "moving_particle_count": EXPECTED_MOVING,
            "nout": EXPECTED_NOUT,
        }


def _required_uint(values: dict[str, Any], name: str, label: str) -> int:
    try:
        return bi4.require_int(values, name)
    except bi4.Bi4FormatError as exc:
        raise RestartQcError(f"{label}: {exc}") from exc


def _finite_float(values: dict[str, Any], name: str, label: str) -> float:
    value = values.get(name)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise RestartQcError(f"{label}: required numeric value {name!r} is invalid")
    result = float(value)
    if not math.isfinite(result):
        raise RestartQcError(f"{label}: required value {name!r} is not finite")
    return result


def _require_data_directory(path: str | os.PathLike[str], label: str) -> Path:
    result = Path(path)
    try:
        metadata = result.lstat()
    except OSError as exc:
        raise RestartQcError(f"{label}: cannot inspect data directory {result}: {exc}") from exc
    if not result.is_dir() or result.is_symlink():
        raise RestartQcError(f"{label}: data directory must be a real directory: {result}")
    if not metadata.st_mode:
        raise RestartQcError(f"{label}: invalid directory metadata: {result}")
    return result


def _load_frame(data_dir: Path, filename: str, expected_cpart: int) -> Frame:
    path = data_dir / filename
    try:
        source = bi4.read_regular_file(path)
        root = bi4.parse_jpartdata_bi4(source.data)
    except bi4.Bi4FormatError as exc:
        raise RestartQcError(f"{path}: invalid BI4 input: {exc}") from exc

    if len(root.items) != 1:
        raise RestartQcError(f"{path}: expected exactly one PART item")
    part = root.items[0]
    for name, expected_type in ARRAY_TYPES.items():
        array = part.arrays.get(name)
        if array is None or array.type_code != expected_type:
            observed = None if array is None else array.type_code
            raise RestartQcError(
                f"{path}: array {name!r} type is {observed}, expected {expected_type}"
            )

    ids = tuple(int(value) for value in part.arrays["Idp"].records())
    positions = tuple(part.arrays["Posd"].records())
    velocities = tuple(part.arrays["Vel"].records())
    densities = tuple(part.arrays["Rhop"].records())
    lengths = {len(ids), len(positions), len(velocities), len(densities)}
    if lengths != {EXPECTED_PARTICLES}:
        raise RestartQcError(
            f"{path}: particle array counts must all equal {EXPECTED_PARTICLES}"
        )
    if len(set(ids)) != EXPECTED_PARTICLES:
        raise RestartQcError(f"{path}: Idp values are not unique")
    if set(ids) != set(range(EXPECTED_PARTICLES)):
        raise RestartQcError(
            f"{path}: Idp set is not exactly 0..{EXPECTED_PARTICLES - 1}"
        )

    exact_values = {
        "CaseNp": (root.values, EXPECTED_PARTICLES),
        "CaseNmoving": (root.values, EXPECTED_MOVING),
        "Npok": (part.values, EXPECTED_PARTICLES),
        "NpTotal": (part.values, EXPECTED_PARTICLES),
        "Nout": (part.values, EXPECTED_NOUT),
    }
    for name, (values, expected) in exact_values.items():
        observed = _required_uint(values, name, str(path))
        if observed != expected:
            raise RestartQcError(
                f"{path}: {name}={observed}, expected exact {expected}"
            )
    if part.value_types.get("SymplecticDtPre") != 12:
        raise RestartQcError(f"{path}: SymplecticDtPre is not binary64")

    values_by_id: dict[str, dict[int, Any]] = {
        "Posd": dict(zip(ids, positions)),
        "Vel": dict(zip(ids, velocities)),
        "Rhop": dict(zip(ids, densities)),
    }
    raw_by_id: dict[str, dict[int, bytes]] = {}
    for name, record_size in ARRAY_RECORD_SIZES.items():
        payload = part.arrays[name].payload
        raw_by_id[name] = {
            particle_id: payload[index * record_size : (index + 1) * record_size]
            for index, particle_id in enumerate(ids)
        }

    cpart = _required_uint(part.values, "Cpart", str(path))
    if cpart != expected_cpart:
        raise RestartQcError(
            f"{path}: embedded Cpart={cpart}, expected exact {expected_cpart}"
        )

    return Frame(
        path=path,
        sha256=source.sha256,
        size_bytes=source.size_bytes,
        cpart=cpart,
        time_s=_finite_float(part.values, "TimeStep", str(path)),
        step=_required_uint(part.values, "Step", str(path)),
        symplectic_dt_pre=_finite_float(
            part.values, "SymplecticDtPre", str(path)
        ),
        ids=ids,
        values_by_id=values_by_id,
        raw_by_id=raw_by_id,
    )


def _compare_exact_array(first: Frame, second: Frame, name: str) -> dict[str, Any]:
    mismatches = [
        particle_id
        for particle_id in range(EXPECTED_PARTICLES)
        if first.raw_by_id[name][particle_id] != second.raw_by_id[name][particle_id]
    ]
    return {
        "comparison": "bitwise_exact_after_Idp_alignment",
        "particle_count": EXPECTED_PARTICLES,
        "mismatch_particle_count": len(mismatches),
        "first_mismatch_ids": mismatches[:16],
        "pass": not mismatches,
    }


def _components(value: Any) -> tuple[float, ...]:
    if isinstance(value, tuple):
        return tuple(float(component) for component in value)
    return (float(value),)


def _compare_tolerant_array(
    first: Frame, second: Frame, name: str, *, atol: float, rtol: float
) -> dict[str, Any]:
    mismatch_particles: set[int] = set()
    mismatch_components = 0
    examples: list[dict[str, Any]] = []
    max_abs = 0.0
    max_rel = 0.0
    for particle_id in range(EXPECTED_PARTICLES):
        left = _components(first.values_by_id[name][particle_id])
        right = _components(second.values_by_id[name][particle_id])
        if len(left) != len(right):
            raise RestartQcError(f"internal component-count mismatch for {name}")
        for component, (continuous, restart) in enumerate(zip(left, right)):
            if not math.isfinite(continuous) or not math.isfinite(restart):
                raise RestartQcError(
                    f"non-finite {name} value for Idp {particle_id}, component {component}"
                )
            absolute = abs(continuous - restart)
            scale = max(abs(continuous), abs(restart))
            relative = absolute / scale if scale else 0.0
            max_abs = max(max_abs, absolute)
            max_rel = max(max_rel, relative)
            if not math.isclose(continuous, restart, abs_tol=atol, rel_tol=rtol):
                mismatch_particles.add(particle_id)
                mismatch_components += 1
                if len(examples) < 16:
                    examples.append(
                        {
                            "idp": particle_id,
                            "component": component,
                            "continuous": continuous,
                            "restart": restart,
                            "absolute_difference": absolute,
                            "allowed_difference": max(atol, rtol * scale),
                        }
                    )
    return {
        "comparison": "math.isclose_after_Idp_alignment",
        "atol": atol,
        "rtol": rtol,
        "max_absolute_difference": max_abs,
        "max_relative_difference": max_rel,
        "mismatch_particle_count": len(mismatch_particles),
        "mismatch_component_count": mismatch_components,
        "first_mismatches": examples,
        "pass": not mismatch_particles,
    }


def _compare_exact_scalar(first: float, second: float) -> dict[str, Any]:
    first_raw = struct.pack("<d", first)
    second_raw = struct.pack("<d", second)
    return {
        "comparison": "binary64_bitwise_exact",
        "continuous_binary64_le_hex": first_raw.hex(),
        "restart_binary64_le_hex": second_raw.hex(),
        "pass": first_raw == second_raw,
    }


def _compare_tolerant_scalar(
    first: float, second: float, *, atol: float, rtol: float
) -> dict[str, Any]:
    absolute = abs(first - second)
    scale = max(abs(first), abs(second))
    return {
        "comparison": "math.isclose",
        "atol": atol,
        "rtol": rtol,
        "continuous": first,
        "restart": second,
        "absolute_difference": absolute,
        "relative_difference": absolute / scale if scale else 0.0,
        "allowed_difference": max(atol, rtol * scale),
        "pass": math.isclose(first, second, abs_tol=atol, rel_tol=rtol),
    }


def build_report(
    continuous_data_dir: str | os.PathLike[str],
    restart_data_dir: str | os.PathLike[str],
) -> dict[str, Any]:
    continuous_dir = _require_data_directory(continuous_data_dir, "continuous")
    restart_dir = _require_data_directory(restart_data_dir, "fresh restart")
    frames = {
        "continuous_checkpoint": _load_frame(
            continuous_dir, "Part_0160.bi4", 160
        ),
        "continuous_evolved": _load_frame(continuous_dir, "Part_0161.bi4", 161),
        "restart_checkpoint": _load_frame(restart_dir, "Part_0160.bi4", 160),
        "restart_evolved": _load_frame(restart_dir, "Part_0161.bi4", 161),
    }
    checkpoint_a = frames["continuous_checkpoint"]
    checkpoint_b = frames["restart_checkpoint"]
    evolved_a = frames["continuous_evolved"]
    evolved_b = frames["restart_evolved"]

    checkpoint = {
        name: _compare_exact_array(checkpoint_a, checkpoint_b, name)
        for name in ("Posd", "Vel", "Rhop")
    }
    checkpoint["SymplecticDtPre"] = _compare_exact_scalar(
        checkpoint_a.symplectic_dt_pre, checkpoint_b.symplectic_dt_pre
    )
    evolved = {
        name: _compare_tolerant_array(
            evolved_a, evolved_b, name, **EVOLUTION_TOLERANCES[name]
        )
        for name in ("Posd", "Vel", "Rhop")
    }
    evolved["SymplecticDtPre"] = _compare_tolerant_scalar(
        evolved_a.symplectic_dt_pre,
        evolved_b.symplectic_dt_pre,
        **EVOLUTION_TOLERANCES["SymplecticDtPre"],
    )
    continuous_interval = evolved_a.time_s - checkpoint_a.time_s
    restart_interval = evolved_b.time_s - checkpoint_b.time_s
    timeline = {
        "continuous_checkpoint_near_8_s": math.isclose(
            checkpoint_a.time_s,
            EXPECTED_CHECKPOINT_TIME_S,
            abs_tol=TIME_TOLERANCE_S,
            rel_tol=0.0,
        ),
        "restart_checkpoint_near_8_s": math.isclose(
            checkpoint_b.time_s,
            EXPECTED_CHECKPOINT_TIME_S,
            abs_tol=TIME_TOLERANCE_S,
            rel_tol=0.0,
        ),
        "continuous_interval_near_0_05_s": math.isclose(
            continuous_interval,
            EXPECTED_EVOLUTION_INTERVAL_S,
            abs_tol=TIME_TOLERANCE_S,
            rel_tol=0.0,
        ),
        "restart_interval_near_0_05_s": math.isclose(
            restart_interval,
            EXPECTED_EVOLUTION_INTERVAL_S,
            abs_tol=TIME_TOLERANCE_S,
            rel_tol=0.0,
        ),
        "intervals_match": math.isclose(
            continuous_interval,
            restart_interval,
            abs_tol=TIME_TOLERANCE_S,
            rel_tol=0.0,
        ),
        "continuous_interval_s": continuous_interval,
        "restart_interval_s": restart_interval,
        "absolute_tolerance_s": TIME_TOLERANCE_S,
    }
    checkpoint_pass = all(result["pass"] for result in checkpoint.values())
    evolved_pass = all(result["pass"] for result in evolved.values())
    timeline_pass = all(
        timeline[key]
        for key in (
            "continuous_checkpoint_near_8_s",
            "restart_checkpoint_near_8_s",
            "continuous_interval_near_0_05_s",
            "restart_interval_near_0_05_s",
            "intervals_match",
        )
    )
    overall_pass = checkpoint_pass and evolved_pass and timeline_pass
    return {
        "schema_version": 1,
        "document_type": "SMPCC_R8_LIQUID_U3_RESTART_EQUIVALENCE_QC_V1",
        "classification": CLASSIFICATION,
        "inputs": {
            "continuous_data_dir": str(continuous_dir),
            "restart_data_dir": str(restart_dir),
            "frames": {name: frame.summary() for name, frame in frames.items()},
        },
        "fixed_invariants": {
            "particle_count_exact": EXPECTED_PARTICLES,
            "moving_particle_count_exact": EXPECTED_MOVING,
            "nout_exact": EXPECTED_NOUT,
            "idp_set_exact": f"0..{EXPECTED_PARTICLES - 1}",
            "all_frames_pass": True,
        },
        "checkpoint_bitwise_comparison": checkpoint,
        "one_step_evolution_comparison": evolved,
        "timeline_mapping": timeline,
        "verdict": {
            "status": (
                "PASS_U3_RESTART_EQUIVALENCE_SIM_ONLY_UNVALIDATED"
                if overall_pass
                else "FAIL_U3_RESTART_EQUIVALENCE_SIM_ONLY_UNVALIDATED"
            ),
            "pass": overall_pass,
            "checkpoint_bitwise_pass": checkpoint_pass,
            "one_step_tolerance_pass": evolved_pass,
            "timeline_mapping_pass": timeline_pass,
            "formal_safety_certification": False,
            "physical_primary_eligible": False,
        },
    }


def _write_json_exclusive(path: str | os.PathLike[str], report: dict[str, Any]) -> None:
    output = Path(path)
    raw = (
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        + "\n"
    ).encode("utf-8")
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(output, flags, 0o440)
    try:
        offset = 0
        while offset < len(raw):
            written = os.write(descriptor, raw[offset:])
            if written <= 0:
                raise OSError("short O_EXCL JSON write")
            offset += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--continuous-data-dir", required=True)
    parser.add_argument("--restart-data-dir", required=True)
    parser.add_argument("--output-json", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = build_report(args.continuous_data_dir, args.restart_data_dir)
        result_code = 0 if report["verdict"]["pass"] else 1
    except RestartQcError as exc:
        report = {
            "schema_version": 1,
            "document_type": "SMPCC_R8_LIQUID_U3_RESTART_EQUIVALENCE_QC_V1",
            "classification": CLASSIFICATION,
            "verdict": {
                "status": "ERROR_U3_RESTART_EQUIVALENCE_INPUT_SIM_ONLY_UNVALIDATED",
                "pass": False,
                "formal_safety_certification": False,
                "physical_primary_eligible": False,
            },
            "error": str(exc),
        }
        result_code = 2
    try:
        _write_json_exclusive(args.output_json, report)
    except OSError as exc:
        print(f"cannot create O_EXCL output JSON {args.output_json}: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "classification": CLASSIFICATION,
                "output_json": args.output_json,
                "status": report["verdict"]["status"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
