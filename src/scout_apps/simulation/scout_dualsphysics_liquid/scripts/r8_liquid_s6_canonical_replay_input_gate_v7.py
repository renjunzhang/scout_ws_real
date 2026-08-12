#!/usr/bin/env python3
"""Strict one-bag S6 v7 admission for canonical BI4 frames and 16 Gauges.

The public CLI is fixture-only.  Real use is an explicit-bytes library call;
it has no discovery, optional-bag, solver, GPU, sudo, profile, or write path.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
SCHEMA_PATH = ROOT / "schema/target_host_s6_canonical_replay_input_v7.json"
FRAME_READER_PATH = ROOT / "scripts/r8_liquid_s5b0_finalized_frame_reader_v2.py"
FRAME_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_finalized_solver_frames_manifest_v2.json"
BI4_READER_PATH = ROOT / "scripts/r8_liquid_bi4_reader_v1.py"
FRAME_READER_SHA256 = "aecd5125625ce4da91b9782f6a28eed017ff5e163056fc648b414aca96e2af4c"
FRAME_SCHEMA_SHA256 = "4f75694305cd8530ebc0b3c744ec8f3e9d880a0c63d7160cb9a7fb6b6b92853a"
BI4_READER_SHA256 = "104d271b750eed15985bf29f743a59678d66c50b9dfae9d1a3fd1663510dbc14"
CANONICAL_IDS_SHA256 = "dbce99d36546067ed3a903e5892383885b6bb9b94feb1f4171411743fec9bdcf"
EXPECTED_PARTICLE_COUNT = 9078
EXPECTED_CLASS_COUNTS = {
    "fixed_boundary": 0,
    "moving_boundary": 2669,
    "floating": 0,
    "fluid": 6409,
}
PROBE_RADIUS_M = 0.0145
ANGULAR_SPACING_DEG = 22.5
PROBES = tuple(f"s5b0_p{index:02d}" for index in range(16))
MAXIMUM_INVALID_RATIO = 0.001
MINIMUM_VALID_PROBES_PER_SLOT = 16


class CanonicalReplayInputV7Error(ValueError):
    """A dependency, manifest, Gauge, grid, or claim invariant failed."""


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def frame_canonical_sha256(value: Any) -> str:
    """Match the already-frozen frame reader's no-newline canonical encoding."""
    return hashlib.sha256(json.dumps(
        value, ensure_ascii=False, allow_nan=False, sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")).hexdigest()


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if (kind == "object" or isinstance(kind, list) and "object" in kind) and value.get("additionalProperties") is not False:
            raise CanonicalReplayInputV7Error(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def _json(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(raw, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    except (UnicodeError, ValueError) as exc:
        raise CanonicalReplayInputV7Error(f"invalid JSON: {label}") from exc
    if not isinstance(value, dict):
        raise CanonicalReplayInputV7Error(f"JSON root is not object: {label}")
    return value


def _dependency(path: Path, expected: str) -> None:
    if sha256_bytes(path.read_bytes()) != expected:
        raise CanonicalReplayInputV7Error(f"canonical dependency drift: {path.name}")


def expected_probe_grid() -> list[dict[str, Any]]:
    result = []
    for slot, name in enumerate(PROBES):
        angle = slot * ANGULAR_SPACING_DEG
        radians = math.radians(angle)
        x = PROBE_RADIUS_M * math.cos(radians)
        y = PROBE_RADIUS_M * math.sin(radians)
        if abs(x) < 1e-17: x = 0.0
        if abs(y) < 1e-17: y = 0.0
        result.append({"slot": slot, "name": name, "angle_deg": angle, "x_m": x, "y_m": y})
    return result


def validate_frame_manifest(frame: Mapping[str, Any]) -> list[float]:
    schema = json.loads(FRAME_SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema); assert_deep_closed(schema)
    Draft202012Validator(schema).validate(dict(frame))
    frames = frame["frames"]
    expected_start = frame["contract"]["expected_start_index"]
    indices = [row["index"] for row in frames]
    if indices != list(range(expected_start, expected_start + len(frames))):
        raise CanonicalReplayInputV7Error("frame indices are not exact contiguous order")
    times = [float(row["time_s"]) for row in frames]
    if any(not math.isfinite(value) for value in times) or any(right <= left for left, right in zip(times, times[1:])):
        raise CanonicalReplayInputV7Error("frame time grid is not strict finite")
    if frame_canonical_sha256(times) != frame["contract"]["expected_times_sha256"]:
        raise CanonicalReplayInputV7Error("frame external time grid hash differs")
    contract = frame["contract"]
    if (contract["expected_frame_count"] != len(frames)
            or contract["expected_particle_count"] != EXPECTED_PARTICLE_COUNT
            or contract["expected_canonical_ids_sha256"] != CANONICAL_IDS_SHA256
            or contract["expected_class_counts"] != EXPECTED_CLASS_COUNTS):
        raise CanonicalReplayInputV7Error("v2 frame contract identity differs")
    if (not frame["integrity"]["external_inventory_receipt_bound"]
            or not frame["integrity"]["exact_inventory"]
            or not frame["integrity"]["particle_id_set_canonical"]
            or not frame["integrity"]["particle_array_order_normalized"]):
        raise CanonicalReplayInputV7Error("v2 receipt/inventory/ID integrity differs")
    if any(
        row["particle_count"] != EXPECTED_PARTICLE_COUNT
        or row["canonical_ids_sha256"] != CANONICAL_IDS_SHA256
        or row["class_counts"] != EXPECTED_CLASS_COUNTS
        or row["nout"] != 0
        or not row["finite"]
        for row in frames
    ):
        raise CanonicalReplayInputV7Error("frame particle/ID/class/Nout/finite contract differs")
    if frame["integrity"]["frame_manifest_sha256"] != frame_canonical_sha256(frames):
        raise CanonicalReplayInputV7Error("canonical frame-list hash differs")
    return times


def parse_gauge_csv(raw: bytes, *, expected_times: Sequence[float]) -> dict[str, Any]:
    try:
        reader = csv.DictReader(io.StringIO(raw.decode("utf-8-sig"), newline=""), strict=True)
    except UnicodeError as exc:
        raise CanonicalReplayInputV7Error("Gauge CSV encoding invalid") from exc
    if tuple(reader.fieldnames or ()) != ("time_s", "zsurf_m"):
        raise CanonicalReplayInputV7Error("Gauge CSV header differs")
    times: list[float] = []
    counts = {"valid": 0, "missing": 0, "invalid": 0}
    try:
        for index, row in enumerate(reader):
            if None in row or set(row) != {"time_s", "zsurf_m"} or index >= len(expected_times):
                raise CanonicalReplayInputV7Error("Gauge CSV row shape/count differs")
            time_s = float(row["time_s"])
            if not math.isfinite(time_s) or abs(time_s - float(expected_times[index])) > 1e-9:
                raise CanonicalReplayInputV7Error("Gauge CSV actual time grid differs from frame grid")
            token = row["zsurf_m"].strip()
            if token in ("", "NA", "N/A", "null"):
                counts["missing"] += 1
            else:
                try: value = float(token)
                except ValueError: counts["invalid"] += 1
                else:
                    if not math.isfinite(value): counts["invalid"] += 1
                    else: counts["valid"] += 1
            times.append(time_s)
    except CanonicalReplayInputV7Error:
        raise
    except (csv.Error, ValueError) as exc:
        raise CanonicalReplayInputV7Error("Gauge CSV malformed") from exc
    if times != list(expected_times):
        raise CanonicalReplayInputV7Error("Gauge CSV slot count/time order differs")
    invalid_ratio = (counts["missing"] + counts["invalid"]) / len(times)
    if invalid_ratio > MAXIMUM_INVALID_RATIO:
        raise CanonicalReplayInputV7Error("Gauge invalid ratio exceeds frozen threshold")
    return {**counts, "row_count": len(times), "time_grid_sha256": sha256_json(times)}


def admit(*, frame_manifest_bytes: bytes, frame_manifest_path: str,
          external_inventory_receipt_sha256: str,
          expected_inventory_sha256: str,
          gauge_csv_bytes: Mapping[str, bytes], attachment_frame: str,
          probe_grid: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    _dependency(FRAME_READER_PATH, FRAME_READER_SHA256)
    _dependency(FRAME_SCHEMA_PATH, FRAME_SCHEMA_SHA256)
    _dependency(BI4_READER_PATH, BI4_READER_SHA256)
    if list(gauge_csv_bytes) != list(PROBES):
        raise CanonicalReplayInputV7Error("Gauge names are missing, duplicate, or reordered")
    if [dict(row) for row in probe_grid] != expected_probe_grid():
        raise CanonicalReplayInputV7Error("Gauge slot/coordinate/radius/spacing drift")
    if attachment_frame != "MOVING_CONTAINER_REFERENCE_REF_0":
        raise CanonicalReplayInputV7Error("Gauge attachment frame differs")
    frame = _json(frame_manifest_bytes, "finalized frame manifest")
    times = validate_frame_manifest(frame)
    contract = frame["contract"]
    if (contract["external_inventory_receipt_sha256"] != external_inventory_receipt_sha256
            or contract["expected_inventory_sha256"] != expected_inventory_sha256):
        raise CanonicalReplayInputV7Error("external receipt/inventory SHA binding differs")
    file_rows = []
    for slot, name in enumerate(PROBES):
        raw = gauge_csv_bytes[name]
        parsed = parse_gauge_csv(raw, expected_times=times)
        file_rows.append({"slot": slot, "probe_name": name,
                          "relative_path": f"GaugesSwl_{name}.csv",
                          "sha256": sha256_bytes(raw), "size_bytes": len(raw),
                          "time_grid_sha256": parsed["time_grid_sha256"],
                          "row_count": parsed["row_count"], "valid_count": parsed["valid"],
                          "missing_count": parsed["missing"], "invalid_count": parsed["invalid"]})
    time_grid_sha = sha256_json(times)
    if any(row["time_grid_sha256"] != time_grid_sha for row in file_rows):
        raise CanonicalReplayInputV7Error("Gauge/frame time-grid hash differs")
    grid = expected_probe_grid()
    native = {
        "schema_version": "smpcc-r8-liquid-s5b0-native-gauge-manifest-v1",
        "status": "PASS_S5B0_NATIVE_GAUGE_MANIFEST_V1",
        "fact_source": "SIXTEEN_RAW_NATIVE_JGAUGESWL_CSV",
        "attachment_frame": attachment_frame, "probe_count": 16,
        "probe_radius_m": PROBE_RADIUS_M, "angular_spacing_deg": ANGULAR_SPACING_DEG,
        "probe_grid": grid, "probe_grid_sha256": sha256_json(grid),
        "time_grid_sha256": time_grid_sha, "slot_count": len(times),
        "minimum_valid_probes_per_slot": MINIMUM_VALID_PROBES_PER_SLOT,
        "maximum_invalid_ratio": MAXIMUM_INVALID_RATIO,
        "manifest_sha256": sha256_json(file_rows),
        "actual_csv_inventory_sha256": sha256_json([
            {key: row[key] for key in ("slot", "probe_name", "relative_path", "sha256", "size_bytes")}
            for row in file_rows]),
        "files": file_rows,
    }
    result = {
        "schema_version": "smpcc-r8-liquid-s6-canonical-replay-input-v7",
        "document_type": "SMPCC_R8_LIQUID_S6_CANONICAL_REPLAY_INPUT_V7",
        "status": "PASS_S6_CANONICAL_REPLAY_INPUT_ADMISSION_V7",
        "mode": "EXPLICIT_READ_ONLY_BYTES", "attempt_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01",
        "planned_denominator": 1, "source_outcome": "UNKNOWN",
        "pinned_dependencies": {
            "finalized_frame_reader": {"relative_path": "scripts/r8_liquid_s5b0_finalized_frame_reader_v2.py", "sha256": FRAME_READER_SHA256},
            "finalized_frame_manifest_schema": {"relative_path": "schema/target_host_s5b0_finalized_solver_frames_manifest_v2.json", "sha256": FRAME_SCHEMA_SHA256},
            "bi4_reader": {"relative_path": "scripts/r8_liquid_bi4_reader_v1.py", "sha256": BI4_READER_SHA256}},
        "finalized_frames": {"root": frame["root"], "schema_version": frame["schema_version"],
            "status": frame["status"], "manifest_sha256": sha256_bytes(frame_manifest_bytes),
            "external_inventory_receipt_sha256": external_inventory_receipt_sha256,
            "expected_inventory_sha256": expected_inventory_sha256,
            "inventory_sha256": frame["inventory"]["canonical_sha256"],
            "frame_manifest_sha256": frame["integrity"]["frame_manifest_sha256"],
            "start_index": frame["contract"]["expected_start_index"], "frame_count": len(times),
            "time_grid_sha256": time_grid_sha, "particle_count": EXPECTED_PARTICLE_COUNT,
            "canonical_ids_sha256": CANONICAL_IDS_SHA256,
            "class_counts": dict(EXPECTED_CLASS_COUNTS)},
        "native_gauges": native,
        "integrity": {"dependencies_byte_pinned": True, "frame_schema_deep_closed": True,
            "external_inventory_receipt_bound": True, "expected_inventory_bound": True,
            "frame_inventory_exact": True, "frame_indices_contiguous": True,
            "frame_times_match_external_grid": True, "particle_count_stable": True,
            "particle_id_set_canonical": True, "particle_array_order_normalized": True,
            "particle_classes_stable": True, "nout_zero": True,
            "all_numeric_values_finite": True, "gauge_names_exact_and_ordered": True,
            "gauge_coordinates_exact": True, "gauge_attachment_exact": True,
            "gauge_csv_hashes_and_sizes_exact": True, "gauge_time_grids_identical": True,
            "frame_and_gauge_time_grid_identical": True},
        "claims": {"optional_not_read": True, "PHYSICAL_REFERENCE_PENDING": True,
            "stage6_pass": False, "development_only": True, "physical_fidelity_validated": False,
            "paired_ranking": False, "cross_method_ranking": False,
            "selected_trajectory_cpu_comparison": False,
            "candidate_or_solver_executed": False, "gpu_exposed": False,
            "sudo_used": False, "network_used": False, "external_persistent_write": False},
    }
    schema = json.loads(SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema); assert_deep_closed(schema)
    Draft202012Validator(schema).validate(result)
    return result


def self_check() -> dict[str, Any]:
    _dependency(FRAME_READER_PATH, FRAME_READER_SHA256)
    _dependency(FRAME_SCHEMA_PATH, FRAME_SCHEMA_SHA256)
    _dependency(BI4_READER_PATH, BI4_READER_SHA256)
    schema = json.loads(SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema); assert_deep_closed(schema)
    grid = expected_probe_grid()
    if len(grid) != 16 or sha256_json(grid) == "0" * 64:
        raise CanonicalReplayInputV7Error("probe fixture drift")
    return {"status": "PASS_S6_CANONICAL_REPLAY_INPUT_GATE_V7_STATIC_ONLY",
            "probe_count": 16, "probe_radius_m": PROBE_RADIUS_M,
            "angular_spacing_deg": ANGULAR_SPACING_DEG, "planned_denominator": 1,
            "finalized_frame_schema_version": "smpcc-r8-liquid-s5b0-finalized-solver-frames-manifest-v2",
            "canonical_ids_sha256": CANONICAL_IDS_SHA256,
            "external_inventory_receipt_required": True,
            "real_solver_output_read": False, "optional_bag_read": False,
            "external_write": False, "candidate_executed": False,
            "solver_or_gpu_executed": False, "stage6_pass": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",), nargs="?", default="self-check")
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), sort_keys=True, separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S6_CANONICAL_REPLAY_INPUT_GATE_V7", "error": str(exc)},
                         sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["CanonicalReplayInputV7Error", "admit", "expected_probe_grid",
           "parse_gauge_csv", "validate_frame_manifest", "self_check"]
