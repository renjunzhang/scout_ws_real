#!/usr/bin/env python3
"""Static S5B0 v2 replay admission, moving-envelope, resource and result-QC gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))
import r8_liquid_s5b0_profile_generator_v2 as profile_v2

ROOT = MODULE_DIR.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_admission_policy_v2.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5b0_replay_admission_policy_v2.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5b0_replay_execution_receipt_v2.json"
QC_SCHEMA = ROOT / "schema/target_host_s5b0_replay_result_qc_v2.json"
COMMON_FLAGS = (
    "-ompthreads:1", "-stable:1", "-vres:0", "-cellmode:full", "-cfl:0.1",
    "-shifting:none", "-viscoart:0.3", "-sv:binx,info", "-svres:1",
    "-svtimers:0", "-svdomainvtk:0", "-saveposdouble:1", "-nortimes:1",
    "-createdirs:1", "-csvsep:0",
)


class AdmissionV2Error(ValueError):
    pass


def _read_json(path: Path) -> tuple[dict[str, Any], str]:
    raw = path.read_bytes()
    if not raw or len(raw) > 4 * 1024 * 1024:
        raise AdmissionV2Error(f"unbounded/empty static JSON: {path}")
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise AdmissionV2Error(f"JSON root is not object: {path}")
    return value, hashlib.sha256(raw).hexdigest()


def assert_deep_closed(node: object, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise AdmissionV2Error(f"open schema object: {location}")
        for key, value in node.items():
            assert_deep_closed(value, f"{location}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            assert_deep_closed(value, f"{location}/{index}")


def load_and_validate_policy() -> tuple[dict[str, Any], str]:
    policy, digest = _read_json(POLICY_PATH)
    schema, _ = _read_json(POLICY_SCHEMA)
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    Draft202012Validator(schema).validate(policy)
    if tuple(policy["solver_contract"]["common_flags"]) != COMMON_FLAGS:
        raise AdmissionV2Error("Stage4 COMMON_FLAGS differ")
    if policy["selection"] != {
        "attempt_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01", "role": "PRIMARY_BASELINE",
        "container": "C1", "planned_denominator": 1, "optional_authorized": False,
        "c2_authorized": False,
    }:
        raise AdmissionV2Error("selection/denominator drifted")
    if policy["parent_transfer"]["finalized"] is not False or any(
        policy["authorization"][name] is not False for name in policy["authorization"]
    ):
        raise AdmissionV2Error("static template promoted execution")
    if policy["profile_contract"]["devices"] != ["/dev/nvidia0", "/dev/nvidiactl", "/dev/nvidia-uvm"]:
        raise AdmissionV2Error("GPU device set differs")
    return policy, digest


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise AdmissionV2Error(f"non-finite {label}")
    return float(value)


def validate_solver_path(rows: Sequence[Sequence[object]], solver_tail_s: float = 1.0) -> tuple[tuple[float, ...], ...]:
    if len(rows) < 3:
        raise AdmissionV2Error("solver path requires motion plus final tail row")
    checked = tuple(tuple(_finite(value, f"row {index}") for value in row) for index, row in enumerate(rows))
    if any(len(row) != 7 for row in checked):
        raise AdmissionV2Error("solver path field count differs")
    if any(abs(value) > 1e-12 for value in checked[0]):
        raise AdmissionV2Error("solver path first row is not identity")
    if any(right[0] <= left[0] for left, right in zip(checked, checked[1:])):
        raise AdmissionV2Error("solver path time is not strict")
    penultimate, final = checked[-2], checked[-1]
    if abs(final[0] - penultimate[0] - solver_tail_s) > 1e-12 or final[1:] != penultimate[1:]:
        raise AdmissionV2Error("last row is not the already-included exact solver tail")
    return checked


def _rotation_zyx(yaw_deg: float, pitch_deg: float, roll_deg: float) -> tuple[tuple[float, float, float], ...]:
    yaw, pitch, roll = map(math.radians, (yaw_deg, pitch_deg, roll_deg))
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def query_times(rows: Sequence[Sequence[float]], subdivisions: int, output_period_s: float) -> tuple[float, ...]:
    times = {row[0] for row in rows}
    for left, right in zip(rows, rows[1:]):
        for part in range(1, subdivisions):
            times.add(left[0] + (right[0] - left[0]) * part / subdivisions)
    slot = 0
    while slot * output_period_s < rows[-1][0] - 1e-12:
        times.add(slot * output_period_s)
        slot += 1
    times.add(rows[-1][0])
    return tuple(sorted(times))


def _interpolate(rows: Sequence[Sequence[float]], time_s: float) -> tuple[float, ...]:
    if time_s <= rows[0][0]:
        return tuple(rows[0])
    for left, right in zip(rows, rows[1:]):
        if time_s <= right[0] + 1e-12:
            ratio = (time_s - left[0]) / (right[0] - left[0])
            return (time_s, *(left[index] + ratio * (right[index] - left[index]) for index in range(1, 7)))
    return tuple(rows[-1])


def moving_envelope(policy: Mapping[str, Any], source_rows: Sequence[Sequence[object]], *, dynamic_free_bytes: int | None) -> dict[str, Any]:
    rows = validate_solver_path(source_rows, policy["case_contract"]["solver_tail_s"])
    contract = policy["moving_envelope"]
    times = query_times(rows, contract["interval_subdivisions"], policy["solver_contract"]["output_period_s"])
    radius, height = contract["container_radius_m"], contract["container_height_m"]
    lower, upper = [math.inf]*3, [-math.inf]*3
    for time_s in times:
        _, x, y, z, yaw, pitch, roll = _interpolate(rows, time_s)
        rotation = _rotation_zyx(yaw, pitch, roll)
        for axis, translation in enumerate((x, y, z)):
            radial = radius * math.hypot(rotation[axis][0], rotation[axis][1])
            axial = rotation[axis][2] * height
            lower[axis] = min(lower[axis], translation + min(0.0, axial) - radial)
            upper[axis] = max(upper[axis], translation + max(0.0, axial) + radial)
    margin = contract["margin_m"]
    lower, upper = [value-margin for value in lower], [value+margin for value in upper]
    spans = [high-low for low, high in zip(lower, upper)]
    if any(span > contract["maximum_axis_span_m"] for span in spans):
        raise AdmissionV2Error("8 m swept axis-span gate exceeded")
    counts = [max(1, math.ceil(span/contract["cell_size_m"])) for span in spans]
    cells = math.prod(counts)
    if cells > contract["maximum_cells"]:
        raise AdmissionV2Error("moving cell budget exceeded")
    resources = policy["resource_contract"]
    estimate = resources["fixed_gpu_bytes"] + cells*resources["bytes_per_cell"] + policy["c1_parents"]["particle_count"]*resources["bytes_per_particle"]
    if dynamic_free_bytes is None:
        admitted, headroom = False, None
    else:
        if isinstance(dynamic_free_bytes, bool) or dynamic_free_bytes < resources["minimum_dynamic_free_bytes"]:
            raise AdmissionV2Error("dynamic free VRAM floor failed")
        headroom = dynamic_free_bytes-estimate
        if headroom < resources["minimum_dynamic_headroom_bytes"]:
            raise AdmissionV2Error("dynamic VRAM headroom failed")
        admitted = True
    cumulative = sum(math.dist(left[1:4], right[1:4]) for left, right in zip(rows, rows[1:]))
    return {"bbox_min_m": lower, "bbox_max_m": upper, "axis_spans_m": spans,
            "cumulative_path_length_m": cumulative, "path_length_is_axis_span": False,
            "query_count": len(times), "interval_subdivisions": 4, "output_grid_included": True,
            "cell_counts": counts, "total_cells": cells, "estimated_vram_bytes": estimate,
            "estimate_label": resources["estimate_label"], "dynamic_free_bytes": dynamic_free_bytes,
            "dynamic_headroom_bytes": headroom, "dynamic_resource_admitted": admitted,
            "solver_path_last_t_s": rows[-1][0],
            "tmax_s": policy["c1_parents"]["settled_time_s"] + rows[-1][0],
            "tail_added_again": False}


def validate_result_qc(value: Mapping[str, Any]) -> dict[str, Any]:
    schema, _ = _read_json(QC_SCHEMA)
    Draft202012Validator(schema).validate(value)
    b, g, p, r, i = (value[name] for name in ("boundary", "gauge", "particles", "resources", "integrity"))
    expected_pass = (
        b["finite"] and b["complete"] and b["expected_slots"] == b["observed_slots"]
        and g["complete"] and g["expected_slots"] == g["observed_slots"]
        and g["invalid_ratio"] <= g["maximum_invalid_ratio"]
        and p["observed_count"] == 9078 and p["moving_boundary_count"] == 2669
        and p["fluid_count"] == 6409 and p["ids_complete_unique"] and p["mass_conserved"]
        and p["nout"] == 0 and p["finite"] and not r["timed_out"]
        and r["xid_count"] == r["reset_count"] == r["cuda_error_count"] == 0 and not r["oom"]
        and i["inputs_unchanged"] and i["inventory_exact"] and i["checksums_valid"]
    )
    if value["pass"] is not expected_pass:
        raise AdmissionV2Error("QC pass contradicts boundary/Gauge/particle/resource/integrity evidence")
    return dict(value)


def fixture_qc() -> dict[str, Any]:
    return {"schema_version": "smpcc-r8-liquid-s5b0-replay-result-qc-v2",
            "boundary": {"expected_slots": 41, "observed_slots": 41, "max_position_error_m": 1e-8, "max_angular_error_rad": 1e-7, "finite": True, "complete": True},
            "gauge": {"expected_slots": 41, "observed_slots": 41, "invalid_count": 0, "invalid_ratio": 0.0, "maximum_invalid_ratio": 0.001, "complete": True},
            "particles": {"expected_count": 9078, "observed_count": 9078, "moving_boundary_count": 2669, "fluid_count": 6409, "ids_complete_unique": True, "mass_conserved": True, "nout": 0, "finite": True},
            "resources": {"timed_out": False, "xid_count": 0, "reset_count": 0, "cuda_error_count": 0, "oom": False, "peak_vram_bytes": 3_000_000_000, "dynamic_free_bytes_at_start": 12_000_000_000, "estimate_label": "PREFLIGHT_ESTIMATE_NOT_OBSERVED_USAGE"},
            "integrity": {"inputs_unchanged": True, "inventory_exact": True, "checksums_valid": True, "source_outcome_overridden": False}, "pass": True}


def static_receipt(policy_sha: str) -> dict[str, Any]:
    return {"schema_version": "smpcc-r8-liquid-s5b0-replay-execution-receipt-v2",
            "document_type": "SMPCC_R8_LIQUID_S5B0_REPLAY_EXECUTION_RECEIPT_V2",
            "replay_id": None, "status": "NOT_ADMITTED_S5A1_FINALIZED_REQUIRED", "phase": "STATIC",
            "policy_sha256": policy_sha, "transfer": {"finalized": False, "transfer_id": None, "manifest_sha256": None},
            "roots": {"partial": None, "final": None, "fresh": False, "failure_preserved": False, "atomic_noreplace": False},
            "execution": {"authorized": False, "candidate_source_executed": False, "staged_candidate_executed": False, "gpu_flag": "-gpu:0", "j_flag_present": False, "wall_timeout_seconds": 5400, "monitor_interval_seconds": 10, "pgid_owned": False},
            "lifecycle": {"profile_loaded": False, "profile_unloaded": False, "process_residue": 0, "mount_residue": 0, "device_residue": 0, "lock_residue": 0, "sudo_residue": 0},
            "result_qc": None, "failure": None}


def self_check() -> dict[str, Any]:
    policy, policy_sha = load_and_validate_policy()
    for schema_path in (RECEIPT_SCHEMA, QC_SCHEMA):
        schema, _ = _read_json(schema_path)
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    receipt = static_receipt(policy_sha)
    Draft202012Validator(_read_json(RECEIPT_SCHEMA)[0]).validate(receipt)
    validate_result_qc(fixture_qc())
    sample = ((0,0,0,0,0,0,0), (1,0.2,0,0,5,0,0), (2,0.2,0,0,5,0,0))
    envelope = moving_envelope(policy, sample, dynamic_free_bytes=12_000_000_000)
    profile = profile_v2.self_check()
    return {"status": receipt["status"], "policy_sha256": policy_sha,
            "schemas_deep_closed": True, "stage4_common_flag_count": len(COMMON_FLAGS),
            "planned_denominator": 1, "moving_query_count": envelope["query_count"],
            "tail_added_again": False, "profile_status": profile["status"],
            "files_written": False, "gpu_exposed": False, "solver_executed": False,
            "bwrap_executed": False, "profile_loaded": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_V2_STATIC", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
