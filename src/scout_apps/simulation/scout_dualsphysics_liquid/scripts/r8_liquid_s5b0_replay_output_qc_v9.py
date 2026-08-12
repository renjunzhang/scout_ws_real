#!/usr/bin/env python3
"""Pure post-execution QC for the one primary S5B0 replay.

It aligns Part frames, RunPARTs, PartMotionRef and the frozen solver path,
normalizes all 16 native Gauge files, and produces bounded in-memory evidence.
No discovery, execution, sudo, GPU, profile, optional bag, or write API exists.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
import math
import struct
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
PROBES = tuple(f"s5b0_p{index:02d}" for index in range(16))
PROBE_GRID = (
    {"name":"s5b0_p00","angle_deg":0.0,"x_m":0.0145,"y_m":0.0},
    {"name":"s5b0_p01","angle_deg":22.5,"x_m":0.013396253221413659,"y_m":0.005548909769293802},
    {"name":"s5b0_p02","angle_deg":45.0,"x_m":0.010253048327204941,"y_m":0.01025304832720494},
    {"name":"s5b0_p03","angle_deg":67.5,"x_m":0.005548909769293803,"y_m":0.013396253221413659},
    {"name":"s5b0_p04","angle_deg":90.0,"x_m":0.0,"y_m":0.0145},
    {"name":"s5b0_p05","angle_deg":112.5,"x_m":-0.005548909769293802,"y_m":0.013396253221413659},
    {"name":"s5b0_p06","angle_deg":135.0,"x_m":-0.01025304832720494,"y_m":0.010253048327204941},
    {"name":"s5b0_p07","angle_deg":157.5,"x_m":-0.013396253221413659,"y_m":0.005548909769293803},
    {"name":"s5b0_p08","angle_deg":180.0,"x_m":-0.0145,"y_m":0.0},
    {"name":"s5b0_p09","angle_deg":202.5,"x_m":-0.01339625322141366,"y_m":-0.005548909769293801},
    {"name":"s5b0_p10","angle_deg":225.0,"x_m":-0.010253048327204943,"y_m":-0.01025304832720494},
    {"name":"s5b0_p11","angle_deg":247.5,"x_m":-0.00554890976929381,"y_m":-0.013396253221413655},
    {"name":"s5b0_p12","angle_deg":270.0,"x_m":0.0,"y_m":-0.0145},
    {"name":"s5b0_p13","angle_deg":292.5,"x_m":0.005548909769293805,"y_m":-0.013396253221413657},
    {"name":"s5b0_p14","angle_deg":315.0,"x_m":0.010253048327204937,"y_m":-0.010253048327204943},
    {"name":"s5b0_p15","angle_deg":337.5,"x_m":0.013396253221413655,"y_m":-0.005548909769293811},
)
MAX_POSITION_ERROR_M = 5e-5
MAX_INVALID_RATIO = 0.001


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path); assert spec and spec.loader
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    spec.loader.exec_module(module); return module


frame_reader = _load("s5b0_frame_v2_for_qc", SCRIPTS / "r8_liquid_s5b0_finalized_frame_reader_v2.py")
gauge_reader = _load("s5b0_gauge_v1_for_qc", SCRIPTS / "r8_liquid_s5b0_native_gauge_normalizer_v1.py")
motion_bridge = _load("s5a1_motion_bridge_v1_for_qc", SCRIPTS / "r8_liquid_s5a1_motion_bridge_v1.py")
bi4 = frame_reader.bi4


class ReplayOutputQcV9Error(ValueError):
    """A frame/motion/Gauge alignment or frozen metric contract failed."""


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _parse_jbd_list(raw: bytes, expected_code: str) -> list[Any]:
    if len(raw) < bi4.JBD_HEADER_SIZE + 4:
        raise ReplayOutputQcV9Error("PartMotionRef is too small")
    header = raw[:bi4.JBD_HEADER_SIZE]
    try: title = header[:58].decode("ascii").rstrip(" ")
    except UnicodeError as exc: raise ReplayOutputQcV9Error("PartMotionRef header encoding") from exc
    if title != f"#FileJBD {expected_code}" or header[58:60] != b"\n\0":
        raise ReplayOutputQcV9Error("PartMotionRef header differs")
    byte_order, si64, reserved2, reserved3 = header[60:64]
    if byte_order not in (0, 10) or bool(byte_order == 10) != bool(si64) or reserved2 or reserved3:
        raise ReplayOutputQcV9Error("PartMotionRef byte order/size mode differs")
    reader = bi4._Reader(raw[bi4.JBD_HEADER_SIZE:]); items = []
    try:
        while reader.remaining: items.append(bi4._read_item(reader, si64=bool(si64), depth=0))
    except bi4.Bi4FormatError as exc: raise ReplayOutputQcV9Error(f"PartMotionRef parse: {exc}") from exc
    if not items or items[0].name != expected_code:
        raise ReplayOutputQcV9Error("PartMotionRef root item differs")
    return items


def parse_motion_ref(raw: bytes, frame_rows: Sequence[Mapping[str, Any]],
                     positions_by_frame: Sequence[Mapping[int, Sequence[float]]]) -> dict[str, Any]:
    items = _parse_jbd_list(raw, "JPartMotRefBi4"); root, appended = items[0], items[1:]
    if root.hidden or root.values_hidden or root.items:
        raise ReplayOutputQcV9Error("PartMotionRef root structure differs")
    expected_value_types = {"AppName": 1, "FormatVer": 8, "MainFile": 2,
        "TimeOut": 12, "MkBoundFirst": 6, "MkMovingCount": 8,
        "MkFloatCount": 8, "MkCount": 8}
    if set(root.values) != set(expected_value_types) or any(
            root.value_types.get(name) != code for name, code in expected_value_types.items()):
        raise ReplayOutputQcV9Error("PartMotionRef root values/types differ")
    if (not isinstance(root.values["AppName"], str) or not root.values["AppName"]
            or root.values["FormatVer"] != 230729 or root.values["MainFile"] is not True
            or abs(float(root.values["TimeOut"]) - 0.05) > frame_reader.TIME_TOLERANCE_S
            or root.values["MkBoundFirst"] != 2 or root.values["MkMovingCount"] != 1
            or root.values["MkFloatCount"] != 0 or root.values["MkCount"] != 1):
        raise ReplayOutputQcV9Error("PartMotionRef root identity differs")
    if len(appended) != len(frame_rows):
        raise ReplayOutputQcV9Error("PartMotionRef frame cardinality differs")
    arrays = root.arrays
    if set(arrays) != {"MkBound", "Nid", "Id", "Ps", "Dis"}:
        raise ReplayOutputQcV9Error("PartMotionRef root arrays differ")
    expected_arrays = {"MkBound": (6, 1), "Nid": (8, 1), "Id": (8, 3),
                       "Ps": (23, 3), "Dis": (12, 3)}
    if any(arrays[name].type_code != kind or arrays[name].count != count
           or arrays[name].hidden for name, (kind, count) in expected_arrays.items()):
        raise ReplayOutputQcV9Error("PartMotionRef root array identity differs")
    if arrays["MkBound"].records() != [0] or arrays["Nid"].records() != [3]:
        raise ReplayOutputQcV9Error("PartMotionRef moving identity differs")
    reference_ids = arrays["Id"].records()
    if len(reference_ids) != 3 or len(set(reference_ids)) != 3 or any(not 0 <= value < 2669 for value in reference_ids):
        raise ReplayOutputQcV9Error("PartMotionRef reference IDs differ")
    root_positions = arrays["Ps"].records(); distances = arrays["Dis"].records()
    if root_positions != [positions_by_frame[0][value] for value in reference_ids]:
        raise ReplayOutputQcV9Error("PartMotionRef root Ps differs from first Part")
    expected_distances = [1.1 * math.sqrt(math.fsum(
        (float(point[axis]) - float(root_positions[0][axis])) ** 2 for axis in range(3)))
        for point in root_positions]
    if any(not math.isfinite(float(value)) or abs(float(value) - expected) > 1e-12
           for value, expected in zip(distances, expected_distances, strict=True)):
        raise ReplayOutputQcV9Error("PartMotionRef root Dis differs")
    maximum_error = 0.0
    for item, frame, positions in zip(appended, frame_rows, positions_by_frame, strict=True):
        if (item.name != f"PART_{frame['index']:04d}" or not item.hidden
                or item.values_hidden or item.items
                or set(item.values) != {"Cpart", "TimeStep", "Step"}
                or item.value_types != {"Cpart": 8, "TimeStep": 12, "Step": 8}
                or set(item.arrays) != {"PosRef"}):
            raise ReplayOutputQcV9Error("PartMotionRef item identity differs")
        posref = item.arrays["PosRef"]
        if posref.type_code != 23 or posref.count != 3 or posref.hidden:
            raise ReplayOutputQcV9Error("PartMotionRef PosRef identity differs")
        if int(item.values["Cpart"]) != frame["index"] or int(item.values["Step"]) != frame["step"] or abs(float(item.values["TimeStep"]) - frame["time_s"]) > frame_reader.TIME_TOLERANCE_S:
            raise ReplayOutputQcV9Error("PartMotionRef item is not frame aligned")
        observed = posref.records()
        expected = [positions[value] for value in reference_ids]
        for left, right in zip(observed, expected, strict=True):
            error = math.sqrt(math.fsum((float(left[axis]) - float(right[axis])) ** 2 for axis in range(3)))
            maximum_error = max(maximum_error, error)
    if maximum_error > MAX_POSITION_ERROR_M:
        raise ReplayOutputQcV9Error("PartMotionRef position differs from Part arrays")
    return {"reference_ids": reference_ids, "frame_count": len(appended),
            "maximum_position_error_m": maximum_error,
            "root_geometry_matched": True, "moving_identity_matched": True,
            "part_motion_ref_frame_aligned": True, "pass": True}


def _solver_rows(raw: bytes) -> list[tuple[float, tuple[float, float, float], tuple[float, float, float]]]:
    try: text = raw.decode("utf-8", "strict")
    except UnicodeError as exc: raise ReplayOutputQcV9Error("solver_path encoding") from exc
    rows = []
    for line in text.splitlines():
        if not line or line.startswith("#"): continue
        fields = line.split(",")
        if len(fields) != 7: raise ReplayOutputQcV9Error("solver_path field count differs")
        try: values = tuple(float(value) for value in fields)
        except ValueError as exc: raise ReplayOutputQcV9Error("solver_path numeric token") from exc
        if any(not math.isfinite(value) for value in values): raise ReplayOutputQcV9Error("solver_path non-finite")
        rows.append((values[0], values[1:4], values[4:7]))
    if len(rows) < 3 or rows[0] != (0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)):
        raise ReplayOutputQcV9Error("solver_path start identity differs")
    if any(right[0] <= left[0] for left, right in zip(rows, rows[1:])):
        raise ReplayOutputQcV9Error("solver_path time is not strictly increasing")
    return rows


def _interpolate(rows: Sequence[tuple[float, Sequence[float], Sequence[float]]],
                 t_s: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if not math.isfinite(t_s) or t_s < rows[0][0] - frame_reader.TIME_TOLERANCE_S \
            or t_s > rows[-1][0] + frame_reader.TIME_TOLERANCE_S:
        raise ReplayOutputQcV9Error("solver_path interpolation coverage differs")
    if t_s <= rows[0][0]: return tuple(rows[0][1]), tuple(rows[0][2])
    if t_s >= rows[-1][0]: return tuple(rows[-1][1]), tuple(rows[-1][2])
    for left, right in zip(rows, rows[1:]):
        if left[0] <= t_s <= right[0]:
            ratio = (t_s - left[0]) / (right[0] - left[0])
            translation = tuple(float(left[1][axis]) + ratio *
                (float(right[1][axis]) - float(left[1][axis])) for axis in range(3))
            angles = tuple(float(left[2][axis]) + ratio *
                (float(right[2][axis]) - float(left[2][axis])) for axis in range(3))
            return translation, angles
    raise ReplayOutputQcV9Error("solver_path interpolation coverage differs")


def _rigid_point(point: Sequence[float], translation: Sequence[float],
                 angles_deg: Sequence[float], center: Sequence[float]) -> tuple[float, float, float]:
    if len(point) != 3 or len(translation) != 3 or len(angles_deg) != 3 or len(center) != 3:
        raise ReplayOutputQcV9Error("rigid-transform vector width differs")
    rotation = motion_bridge.intrinsic_zyx_to_rotation_matrix(
        math.radians(float(angles_deg[0])), math.radians(float(angles_deg[1])),
        math.radians(float(angles_deg[2])))
    relative = tuple(float(point[axis]) - float(center[axis]) for axis in range(3))
    rotated = tuple(math.fsum(rotation[row][axis] * relative[axis] for axis in range(3))
                    for row in range(3))
    return tuple(float(center[axis]) + float(translation[axis]) + rotated[axis]
                 for axis in range(3))


def boundary_qc(frame_rows: Sequence[Mapping[str, Any]], positions_by_frame: Sequence[Mapping[int, Sequence[float]]],
                solver_path_raw: bytes, *, settled_time_s: float,
                rotation_center_m: Sequence[float] = (0.0, 0.0, 0.0)) -> tuple[bytes, dict[str, Any]]:
    if len(frame_rows) != len(positions_by_frame) or not frame_rows:
        raise ReplayOutputQcV9Error("boundary frame/position cardinality differs")
    rows = _solver_rows(solver_path_raw); baseline = positions_by_frame[0]
    moving_ids = range(2669); output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(("part", "time_s", "motion_t_s", "expected_translation_x_m",
                     "expected_translation_y_m", "expected_translation_z_m",
                     "expected_yaw_z_deg", "expected_pitch_y_deg", "expected_roll_x_deg",
                     "observed_centroid_x_m", "observed_centroid_y_m", "observed_centroid_z_m",
                     "max_particle_rigid_residual_m"))
    maximum = 0.0
    for frame, positions in zip(frame_rows, positions_by_frame, strict=True):
        motion_t = max(0.0, frame["time_s"] - settled_time_s)
        translation, angles = _interpolate(rows, motion_t)
        expected_positions = {value: _rigid_point(baseline[value], translation, angles,
                                                   rotation_center_m) for value in moving_ids}
        centroid = tuple(math.fsum(float(positions[value][axis]) for value in moving_ids) / 2669
                         for axis in range(3))
        residual = max(math.sqrt(math.fsum(
            (float(positions[value][axis]) - expected_positions[value][axis]) ** 2
            for axis in range(3))) for value in moving_ids)
        maximum = max(maximum, residual)
        writer.writerow((frame["index"], format(frame["time_s"], ".17g"), format(motion_t, ".17g"),
                         *[format(value, ".17g") for value in translation],
                         *[format(value, ".17g") for value in angles],
                         *[format(value, ".17g") for value in centroid], format(residual, ".17g")))
    if maximum > MAX_POSITION_ERROR_M:
        raise ReplayOutputQcV9Error("moving boundary differs from frozen solver path")
    raw = output.getvalue().encode("utf-8")
    return raw, {"maximum_rigid_residual_m": maximum,
                 "maximum_allowed_m": MAX_POSITION_ERROR_M, "frame_count": len(frame_rows),
                 "moving_particle_count": 2669,
                 "moving_particle_checks": len(frame_rows) * 2669,
                 "transform": "TRANSLATION_PLUS_ACTIVE_INTRINSIC_ZYX_ABOUT_FROZEN_CENTER",
                 "rotation_center_m": [float(value) for value in rotation_center_m],
                 "executed_boundary_motion_sha256": sha256_bytes(raw), "pass": True}


def expected_probe_rays(frame_rows: Sequence[Mapping[str, Any]], solver_path_raw: bytes,
                        probe_grid: Sequence[Mapping[str, Any]], *, settled_time_s: float,
                        rotation_center_m: Sequence[float] = (0.0, 0.0, 0.0)
                        ) -> dict[str, list[dict[str, tuple[float, float, float]]]]:
    """Build each attached probe center from the same solver rigid transform.

    The body-fixed ray midpoint follows full intrinsic-ZYX motion, but the
    patched JGaugeSwl intentionally reconstructs its measurement segment as a
    global-Z ray of frozen half-length after every motion update.
    """
    if list(probe_grid) != list(PROBE_GRID):
        raise ReplayOutputQcV9Error("Gauge name/order/grid differs")
    rows = _solver_rows(solver_path_raw); result = {name: [] for name in PROBES}
    for frame in frame_rows:
        motion_t = max(0.0, float(frame["time_s"]) - float(settled_time_s))
        translation, angles = _interpolate(rows, motion_t)
        for probe in probe_grid:
            if set(probe) != {"name", "angle_deg", "x_m", "y_m"}:
                raise ReplayOutputQcV9Error("Gauge grid row is not closed")
            base_center = (float(probe["x_m"]), float(probe["y_m"]),
                           (gauge_reader.MIN_Z_M + gauge_reader.MAX_Z_M) / 2.0)
            center = _rigid_point(base_center, translation, angles, rotation_center_m)
            if any(not math.isfinite(value) for value in center):
                raise ReplayOutputQcV9Error("Gauge transformed center is non-finite")
            half = (gauge_reader.MAX_Z_M - gauge_reader.MIN_Z_M) / 2.0
            result[probe["name"]].append({
                "point0_m": (center[0], center[1], center[2] - half),
                "point2_m": (center[0], center[1], center[2] + half),
            })
    return result


def normalize_gauges(native_gauges: Mapping[str, bytes], *, expected_times_s: Sequence[float],
                     probe_grid: Sequence[Mapping[str, Any]], solver_path_raw: bytes,
                     settled_time_s: float,
                     rotation_center_m: Sequence[float] = (0.0, 0.0, 0.0)
                     ) -> tuple[dict[str, bytes], dict[str, Any]]:
    if list(native_gauges) != list(PROBES) or list(probe_grid) != list(PROBE_GRID):
        raise ReplayOutputQcV9Error("Gauge name/order/grid differs")
    frame_rows = [{"time_s": float(value)} for value in expected_times_s]
    rays = expected_probe_rays(frame_rows, solver_path_raw, probe_grid,
                               settled_time_s=settled_time_s,
                               rotation_center_m=rotation_center_m)
    normalized: dict[str, bytes] = {}; files = []; total_invalid = 0; total_rows = 0
    for slot, probe in enumerate(probe_grid):
        name = probe["name"]; raw = native_gauges[name]
        derived, evidence = gauge_reader.normalize_native(raw, expected_x_m=float(probe["x_m"]),
                                                          expected_y_m=float(probe["y_m"]),
                                                          expected_times_s=expected_times_s,
                                                          expected_rays_by_time=rays[name])
        relative = f"gauges/normalized/GaugesSwl_{name}.csv"
        normalized[relative] = derived; total_invalid += evidence["invalid_count"]; total_rows += evidence["row_count"]
        files.append({"slot": slot, "probe_name": name,
                      "raw_relative_path": f"gauges/native/GaugesSwl_{name}.csv",
                      "raw_sha256": evidence["raw_sha256"], "raw_size_bytes": evidence["raw_size_bytes"],
                      "normalized_relative_path": relative,
                      "normalized_sha256": evidence["normalized_sha256"],
                      "normalized_size_bytes": evidence["normalized_size_bytes"],
                      "row_count": evidence["row_count"], "invalid_count": evidence["invalid_count"],
                      "time_grid_sha256": evidence["time_grid_sha256"]})
    ratio = total_invalid / total_rows
    if ratio > MAX_INVALID_RATIO or len({row["time_grid_sha256"] for row in files}) != 1:
        raise ReplayOutputQcV9Error("Gauge invalid ratio/time grid differs")
    manifest = {"schema_version": "smpcc-r8-liquid-s5b0-native-gauge-provenance-v9",
        "attempt_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01", "probe_count": 16,
        "raw_format": "DUALSPHYSICS_5_4_JGAUGESWL_SEMICOLON_10_COLUMN",
        "normalized_format": "RFC4180_TIME_S_ZSURF_M_V1", "raw_preserved": True,
        "file_count": 16, "total_rows": total_rows, "invalid_count": total_invalid,
        "invalid_ratio": ratio, "maximum_invalid_ratio": MAX_INVALID_RATIO,
        "time_grid_sha256": files[0]["time_grid_sha256"],
        "attachment_frame": "MOVING_CONTAINER_REFERENCE_REF_0",
        "ray_contract": "BODY_FIXED_CENTER_TRANSFORMED_THEN_GLOBAL_Z_RECONSTRUCTED",
        "files": files, "manifest_sha256": sha256_bytes(canonical_json(files)), "pass": True}
    return normalized, manifest


def self_check() -> dict[str, Any]:
    native = {}; grid = [dict(value) for value in PROBE_GRID]
    times = (0.0, 0.05)
    for probe in grid:
        name = str(probe["name"]); x = float(probe["x_m"]); y = float(probe["y_m"])
        header = ";".join(gauge_reader.NATIVE_HEADER)
        native[name] = (header + "\n" +
            f"0;{x:.17g};{y:.17g};0.058;{x:.17g};{y:.17g};0.002;{x:.17g};{y:.17g};0.064\n" +
            f"0.05;{x:.17g};{y:.17g};0.059;{x:.17g};{y:.17g};0.002;{x:.17g};{y:.17g};0.064\n").encode()
    solver_path = (b"0,0,0,0,0,0,0\n0.05,0,0,0,0,0,0\n"
                   b"0.1,0,0,0,0,0,0\n")
    normalized, manifest = normalize_gauges(native, expected_times_s=times,
        probe_grid=grid, solver_path_raw=solver_path, settled_time_s=0.0)
    if len(normalized) != 16 or not manifest["pass"]:
        raise ReplayOutputQcV9Error("output QC fixture drift")
    point = _rigid_point((1.0, 0.0, 0.0), (2.0, 3.0, 0.0), (90.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0))
    if max(abs(point[index] - (2.0, 4.0, 0.0)[index]) for index in range(3)) > 1e-12:
        raise ReplayOutputQcV9Error("intrinsic-ZYX rigid fixture drift")
    return {"status": "PASS_S5B0_REPLAY_OUTPUT_QC_V9_STATIC_ONLY", "probe_count": 16,
            "native_format_enforced": True, "frame_reader_v2_required": True,
            "runparts_motionref_boundary_alignment_required": True,
            "intrinsic_zyx_rigid_boundary_required": True,
            "real_solver_output_read": False, "files_written": False, "candidate_executed": False,
            "gpu_exposed": False, "optional_bag_read": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("command", choices=("self-check",), nargs="?", default="self-check"); parser.parse_args(argv)
    try: print(json.dumps(self_check(), sort_keys=True, separators=(",", ":"))); return 0
    except Exception as exc: print(json.dumps({"status": "FAIL_S5B0_REPLAY_OUTPUT_QC_V9", "error": str(exc)}, sort_keys=True), file=sys.stderr); return 2


if __name__ == "__main__": raise SystemExit(main())
