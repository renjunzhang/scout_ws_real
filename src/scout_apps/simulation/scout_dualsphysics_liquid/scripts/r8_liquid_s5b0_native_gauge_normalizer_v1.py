#!/usr/bin/env python3
"""Strict native DualSPHysics JGaugeSwl CSV normalizer.

The parser preserves the raw-byte identity in its manifest and emits a
deterministic two-column derivative.  It accepts the exact semicolon header
written by DualSPHysics 5.4; the older ``time_s,zsurf_m`` fiction is rejected.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import sys
from typing import Any, Mapping, Sequence


sys.dont_write_bytecode = True
NATIVE_HEADER = (
    "time [s]", "swlx [m]", "swly [m]", "swlz [m]",
    "pos0x [m]", "pos0y [m]", "pos0z [m]",
    "pos2x [m]", "pos2y [m]", "pos2z [m]",
)
POINT_TOLERANCE_M = 2e-6
TIME_TOLERANCE_S = 2e-5
MIN_Z_M = 0.002
MAX_Z_M = 0.064


class NativeGaugeError(ValueError):
    """The native Gauge bytes or their frozen geometry/time contract differ."""


def _sha(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _finite(token: str, label: str) -> float:
    try:
        value = float(token.strip())
    except ValueError as exc:
        raise NativeGaugeError(f"invalid numeric token: {label}") from exc
    if not math.isfinite(value):
        raise NativeGaugeError(f"non-finite numeric token: {label}")
    return value


def canonical_time_sha256(times: Sequence[float]) -> str:
    raw = json.dumps(list(times), allow_nan=False, sort_keys=True,
                     separators=(",", ":")).encode("utf-8")
    return _sha(raw)


def normalize_native(
    raw: bytes,
    *,
    expected_x_m: float,
    expected_y_m: float,
    expected_times_s: Sequence[float] | None = None,
    expected_rays_by_time: Sequence[Mapping[str, Sequence[float]]] | None = None,
) -> tuple[bytes, dict[str, Any]]:
    """Return deterministic ``time_s,zsurf_m`` CSV plus exact provenance."""

    if not raw or len(raw) > 16 * 1024 * 1024 or b"\0" in raw:
        raise NativeGaugeError("native Gauge byte bound/NUL contract differs")
    try:
        text = raw.decode("utf-8-sig", "strict")
    except UnicodeError as exc:
        raise NativeGaugeError("native Gauge is not strict UTF-8") from exc
    reader = csv.reader(io.StringIO(text, newline=""), delimiter=";", strict=True)
    try:
        header = tuple(next(reader))
    except (StopIteration, csv.Error) as exc:
        raise NativeGaugeError("native Gauge header is absent") from exc
    if header != NATIVE_HEADER:
        raise NativeGaugeError("native Gauge header differs from DualSPHysics 5.4")

    expected = None if expected_times_s is None else [float(value) for value in expected_times_s]
    rays = None if expected_rays_by_time is None else list(expected_rays_by_time)
    if rays is not None and (expected is None or len(rays) != len(expected)):
        raise NativeGaugeError("moving-attached ray/time cardinality differs")
    if rays is None:
        raise NativeGaugeError("moving-attached expected rays are required")
    rows: list[tuple[float, float]] = []
    invalid = 0
    previous = -math.inf
    try:
        for index, tokens in enumerate(reader):
            if len(tokens) != len(NATIVE_HEADER):
                raise NativeGaugeError("native Gauge row width differs")
            values = [_finite(token, f"row {index}") for token in tokens]
            time_s, swlx, swly, swlz, p0x, p0y, p0z, p2x, p2y, p2z = values
            if time_s <= previous:
                raise NativeGaugeError("native Gauge time is not strictly increasing")
            if expected is not None and (
                index >= len(expected) or abs(time_s - expected[index]) > TIME_TOLERANCE_S
            ):
                raise NativeGaugeError("native Gauge time differs from frozen frame grid")
            if index >= len(rays):
                raise NativeGaugeError("moving-attached ray row is absent")
            ray = rays[index]
            if set(ray) != {"point0_m", "point2_m"} or len(ray["point0_m"]) != 3 or len(ray["point2_m"]) != 3:
                raise NativeGaugeError("moving-attached ray row is not closed XYZ")
            expected_p0 = tuple(float(value) for value in ray["point0_m"])
            expected_p2 = tuple(float(value) for value in ray["point2_m"])
            if any(not math.isfinite(value) for value in (*expected_p0, *expected_p2)):
                raise NativeGaugeError("moving-attached expected ray is non-finite")
            if abs(expected_p0[0] - expected_p2[0]) > POINT_TOLERANCE_M or abs(expected_p0[1] - expected_p2[1]) > POINT_TOLERANCE_M or expected_p2[2] <= expected_p0[2]:
                raise NativeGaugeError("moving-attached expected ray is not global-Z")
            for actual, frozen in ((p0x, expected_p0[0]), (p0y, expected_p0[1]),
                                   (p2x, expected_p2[0]), (p2y, expected_p2[1]),
                                   (p0z, expected_p0[2]), (p2z, expected_p2[2])):
                if abs(actual - frozen) > POINT_TOLERANCE_M:
                    raise NativeGaugeError("native Gauge moving-attached ray geometry differs")
            if abs(swlx - expected_p0[0]) > POINT_TOLERANCE_M or abs(swly - expected_p0[1]) > POINT_TOLERANCE_M:
                raise NativeGaugeError("native Gauge SWL x/y differs from attached ray")
            relative_z = MIN_Z_M + (swlz - expected_p0[2])
            if not expected_p0[2] <= swlz <= expected_p2[2] or not MIN_Z_M <= relative_z <= MAX_Z_M:
                invalid += 1
            rows.append((time_s, relative_z))
            previous = time_s
    except csv.Error as exc:
        raise NativeGaugeError("native Gauge CSV is malformed") from exc
    if not rows or expected is not None and len(rows) != len(expected):
        raise NativeGaugeError("native Gauge row count differs")
    times = [row[0] for row in rows]
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(("time_s", "zsurf_m"))
    for time_s, zsurf_m in rows:
        writer.writerow((format(time_s, ".17g"), format(zsurf_m, ".17g")))
    normalized = output.getvalue().encode("utf-8")
    provenance = {
        "native_format": "DUALSPHYSICS_5_4_JGAUGESWL_SEMICOLON_10_COLUMN",
        "raw_sha256": _sha(raw), "raw_size_bytes": len(raw),
        "normalized_format": "RFC4180_TIME_S_ZSURF_M_V1",
        "normalized_sha256": _sha(normalized), "normalized_size_bytes": len(normalized),
        "row_count": len(rows), "invalid_count": invalid,
        "invalid_ratio": invalid / len(rows),
        "time_grid_sha256": canonical_time_sha256(times),
        "raw_preserved": True, "conversion_deterministic": True,
        "moving_attached_ray_validated": rays is not None,
        "zsurf_coordinate": "CONTAINER_RELATIVE_GLOBAL_Z_RAY",
    }
    return normalized, provenance


def self_check() -> dict[str, Any]:
    raw = (";".join(NATIVE_HEADER) + "\n" +
           "0;0.0145;0;0.058;0.0145;0;0.002;0.0145;0;0.064\n" +
           "0.05;0.0145;0;0.059;0.0145;0;0.002;0.0145;0;0.064\n").encode()
    rays = tuple({"point0_m": (0.0145, 0.0, 0.002),
                  "point2_m": (0.0145, 0.0, 0.064)} for _ in range(2))
    normalized, provenance = normalize_native(raw, expected_x_m=0.0145,
                                               expected_y_m=0.0,
                                               expected_times_s=(0.0, 0.05),
                                               expected_rays_by_time=rays)
    if not normalized.startswith(b"time_s,zsurf_m\n") or provenance["row_count"] != 2:
        raise NativeGaugeError("normalizer fixture drift")
    return {"status": "PASS_S5B0_NATIVE_GAUGE_NORMALIZER_V1_STATIC_ONLY",
            "raw_sha256": provenance["raw_sha256"],
            "normalized_sha256": provenance["normalized_sha256"],
            "real_solver_output_read": False, "files_written": False,
            "candidate_executed": False, "gpu_exposed": False,
            "optional_bag_read": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",), nargs="?", default="self-check")
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), sort_keys=True, separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_NATIVE_GAUGE_NORMALIZER_V1",
                          "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
