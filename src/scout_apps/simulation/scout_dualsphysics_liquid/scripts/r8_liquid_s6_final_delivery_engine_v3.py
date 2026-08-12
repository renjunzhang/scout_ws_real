#!/usr/bin/env python3
"""S6 v3 real-data engine with a fail-closed, parent-null public CLI.

The library implements fd-safe package admission, selected-signal and native
Gauge readers, primary-only analysis, content-bound media validation, atomic
publication, and hash-chain ledger helpers.  The repository v3 policy has no
materialized S5B0 parent; consequently its public CLI performs only static
self-checks and never reads external inputs or writes delivery artifacts.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import ctypes
import errno
import fcntl
import hashlib
import io
import json
import math
import os
import re
import stat
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s6_final_delivery_policy_v3.json"
S5B0_V4_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_admission_policy_v4.json"
S6_V2_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s6_final_delivery_policy_v2.json"
SCHEMA_PATHS = {
    "policy": ROOT / "schema/target_host_s6_final_delivery_policy_v3.json",
    "analysis": ROOT / "schema/target_host_s6_final_delivery_analysis_v3.json",
    "engine": ROOT / "schema/target_host_s6_final_delivery_engine_receipt_v3.json",
}
COMMANDS = ("admit", "extract-selected", "analyze", "render-figure", "render-media", "publish", "accept", "run-one-shot")
ATTEMPT = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
NOT_ADMITTED = "NOT_ADMITTED_FINALIZED_S5B0_REPLAY_REQUIRED"
SURFACES = ("H_crest", "H_abs", "H_peak_to_peak")
SECONDARIES = ("H_proxy", "H_modal")
WINDOWS = ("first15", "full_motion", "recorded_tail", "solver_tail")
SHA_RE = re.compile(r"[0-9a-f]{64}\Z")
MAX_FILE_BYTES = 128 * 1024 * 1024
AT_FDCWD = -100
RENAME_NOREPLACE = 1
SYS_RENAMEAT2 = 316


class S6EngineV3Error(ValueError):
    """A closed input, identity, numeric invariant, or write contract differs."""


def canonical_json(value: Any) -> bytes:
    try:
        return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                           separators=(",", ":")) + "\n").encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise S6EngineV3Error("value is not finite canonical JSON") from exc


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"),
                       parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    if not isinstance(value, dict):
        raise S6EngineV3Error(f"JSON root is not object: {path.name}")
    return value


def _sha_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise S6EngineV3Error(f"schema object open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def load_contracts() -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    policy = _read_json(POLICY_PATH)
    schemas = {name: _read_json(path) for name, path in SCHEMA_PATHS.items()}
    for schema in schemas.values():
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    Draft202012Validator(schemas["policy"]).validate(policy)
    bindings = policy["bindings"]
    if _sha_file(S5B0_V4_PATH) != bindings["s5b0_v4_policy_sha256"] or _sha_file(S6_V2_PATH) != bindings["s6_v2_policy_sha256"]:
        raise S6EngineV3Error("parent policy identity drift")
    gauge = _read_json(S5B0_V4_PATH)["gauge_contract"]
    if sha256_json(gauge) != bindings["s5b0_v4_gauge_contract_sha256"]:
        raise S6EngineV3Error("S5B0 v4 Gauge digest drift")
    if [row["name"] for row in gauge["probes"]] != policy["gauge"]["probe_names"]:
        raise S6EngineV3Error("S5B0 v4 probe order drift")
    return policy, schemas


def _safe_relative(value: str) -> str:
    path = PurePosixPath(value)
    if not value or path.is_absolute() or ".." in path.parts or str(path) != value:
        raise S6EngineV3Error("unsafe relative path")
    return value


def _read_fd_all(fd: int, maximum: int = MAX_FILE_BYTES) -> bytes:
    chunks, total = [], 0
    while True:
        chunk = os.read(fd, min(1024 * 1024, maximum + 1 - total))
        if not chunk:
            return b"".join(chunks)
        chunks.append(chunk)
        total += len(chunk)
        if total > maximum:
            raise S6EngineV3Error("file exceeds bound")


def _scan_dir_fd(directory_fd: int, prefix: str = "") -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for name in sorted(os.listdir(directory_fd)):
        if name in (".", "..") or "/" in name:
            raise S6EngineV3Error("unsafe directory entry")
        relative = f"{prefix}/{name}" if prefix else name
        metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        if stat.S_ISDIR(metadata.st_mode):
            child = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=directory_fd)
            try:
                result.update(_scan_dir_fd(child, relative))
            finally:
                os.close(child)
        elif stat.S_ISREG(metadata.st_mode) and metadata.st_nlink == 1:
            fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=directory_fd)
            try:
                payload = _read_fd_all(fd)
                after = os.fstat(fd)
            finally:
                os.close(fd)
            before_id = (metadata.st_dev, metadata.st_ino, metadata.st_size, metadata.st_mtime_ns, metadata.st_ctime_ns)
            after_id = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
            if before_id != after_id:
                raise S6EngineV3Error("file TOCTOU detected")
            result[relative] = {"sha256": sha256_bytes(payload), "size_bytes": len(payload),
                                "mode": stat.S_IMODE(metadata.st_mode), "identity": before_id,
                                "bytes": payload}
        else:
            raise S6EngineV3Error("symlink, hardlink, or special file rejected")
    return result


def admit_finalized_package(package_root: Path, *, expected_root: Path,
                             expected_inventory: Mapping[str, str]) -> dict[str, Any]:
    """Read one exact package through directory fds and validate its frozen parents."""
    root = Path(package_root).absolute()
    expected = Path(expected_root).absolute()
    if root != expected or not root.is_absolute():
        raise S6EngineV3Error("package root differs")
    cursor = Path(root.anchor)
    for part in root.parts[1:]:
        cursor /= part
        if stat.S_ISLNK(os.lstat(cursor).st_mode):
            raise S6EngineV3Error("symlink path component")
    expected_map = {_safe_relative(name): digest for name, digest in expected_inventory.items()}
    if any(SHA_RE.fullmatch(str(value)) is None for value in expected_map.values()):
        raise S6EngineV3Error("expected inventory digest invalid")
    root_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        root_before = os.fstat(root_fd)
        first = _scan_dir_fd(root_fd)
        second = _scan_dir_fd(root_fd)
        root_after = os.fstat(root_fd)
    finally:
        os.close(root_fd)
    projection = lambda rows: {name: (row["sha256"], row["size_bytes"], row["mode"], row["identity"]) for name, row in rows.items()}
    if projection(first) != projection(second) or (root_before.st_dev, root_before.st_ino, root_before.st_mtime_ns, root_before.st_ctime_ns) != (root_after.st_dev, root_after.st_ino, root_after.st_mtime_ns, root_after.st_ctime_ns):
        raise S6EngineV3Error("package TOCTOU detected")
    if set(first) != set(expected_map) or any(first[name]["sha256"] != expected_map[name] for name in expected_map):
        raise S6EngineV3Error("package inventory/hash differs")
    required = {"execution_receipt.json", "result_qc.json", "frame_manifest.json", "native_gauge_manifest.json", "checksums.sha256"}
    if not required.issubset(first):
        raise S6EngineV3Error("required finalized package evidence absent")
    documents = {}
    for name in required - {"checksums.sha256"}:
        documents[name] = json.loads(first[name]["bytes"], parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    execution = documents["execution_receipt.json"]
    qc = documents["result_qc.json"]
    frames = documents["frame_manifest.json"]
    gauge = documents["native_gauge_manifest.json"]
    policy, _ = load_contracts()
    if execution.get("status") != policy["parent"]["required_execution_status"] or execution.get("finalized") is not True or execution.get("attempt_id") != ATTEMPT:
        raise S6EngineV3Error("execution receipt differs")
    if qc.get("status") != policy["parent"]["required_qc_status"] or qc.get("pass") is not True or qc.get("attempt_id") != ATTEMPT:
        raise S6EngineV3Error("result QC differs")
    if frames.get("status") != policy["parent"]["required_frame_status"] or frames.get("integrity_pass") is not True or frames.get("attempt_id") != ATTEMPT:
        raise S6EngineV3Error("frame manifest differs")
    validate_native_gauge_manifest(gauge)
    checksum_lines = first["checksums.sha256"]["bytes"].decode("ascii").splitlines()
    parsed = {}
    for line in checksum_lines:
        if not re.fullmatch(r"[0-9a-f]{64}  (?!/)(?!.*(?:^|/)\.\.(?:/|$)).+", line):
            raise S6EngineV3Error("checksums syntax invalid")
        digest, name = line.split("  ", 1)
        if name in parsed:
            raise S6EngineV3Error("duplicate checksum")
        parsed[name] = digest
    checksum_targets = set(first) - {"checksums.sha256"}
    if set(parsed) != checksum_targets or any(first[name]["sha256"] != parsed[name] for name in parsed):
        raise S6EngineV3Error("checksums are incomplete or differ")
    return {"status": "ADMITTED_FINALIZED_S5B0_PRIMARY_PACKAGE_V3", "attempt_id": ATTEMPT,
            "root": str(root), "inventory_sha256": sha256_json({name: expected_map[name] for name in sorted(expected_map)}),
            "identities": {name: {"sha256": first[name]["sha256"], "size_bytes": first[name]["size_bytes"]} for name in sorted(required)},
            "checks": {"exact_root": True, "exact_inventory": True, "no_symlinks": True,
                       "no_hardlinks": True, "no_special_files": True, "no_toctou": True,
                       "checksums_complete": True, "attempt_exact": True, "optional_unread": True}}


def validate_native_gauge_manifest(value: Mapping[str, Any]) -> None:
    policy, _ = load_contracts()
    if set(value) != {"schema_version", "attempt_id", "gauge_contract_sha256", "time_grid_sha256", "files"}:
        raise S6EngineV3Error("native Gauge manifest open")
    if value["schema_version"] != "smpcc-r8-liquid-s5b0-native-gauge-manifest-v1" or value["attempt_id"] != ATTEMPT or value["gauge_contract_sha256"] != policy["bindings"]["s5b0_v4_gauge_contract_sha256"]:
        raise S6EngineV3Error("native Gauge parent differs")
    files = value["files"]
    if not isinstance(files, list) or [row.get("probe_name") for row in files] != policy["gauge"]["probe_names"]:
        raise S6EngineV3Error("native Gauge file order differs")
    for row in files:
        if set(row) != {"probe_name", "relative_path", "sha256", "size_bytes", "time_grid_sha256"} or row["time_grid_sha256"] != value["time_grid_sha256"]:
            raise S6EngineV3Error("native Gauge file binding differs")
        _safe_relative(row["relative_path"])
        if SHA_RE.fullmatch(str(row["sha256"])) is None or not isinstance(row["size_bytes"], int) or row["size_bytes"] < 1:
            raise S6EngineV3Error("native Gauge identity invalid")


def read_selected_signals_csv(raw: bytes, *, source_schema: str = "SPMPC_NON_FIXED",
                              source_outcome: str = "UNKNOWN") -> dict[str, Any]:
    if source_schema != "SPMPC_NON_FIXED" or source_outcome not in ("UNKNOWN", "AUTHORITATIVE_SIDECAR"):
        raise S6EngineV3Error("selected-signal provenance differs")
    try:
        reader = csv.DictReader(io.StringIO(raw.decode("utf-8-sig"), newline=""), strict=True)
    except UnicodeError as exc:
        raise S6EngineV3Error("selected CSV encoding invalid") from exc
    if tuple(reader.fieldnames or ()) != ("time_s", "H_proxy_m", "H_modal_m"):
        raise S6EngineV3Error("selected CSV header differs")
    rows, previous = [], -math.inf
    try:
        for row in reader:
            if None in row or set(row) != {"time_s", "H_proxy_m", "H_modal_m"}:
                raise S6EngineV3Error("selected CSV row open")
            values = [float(row[name]) for name in ("time_s", "H_proxy_m", "H_modal_m")]
            if not all(math.isfinite(value) for value in values) or values[0] <= previous:
                raise S6EngineV3Error("selected CSV time/value invalid")
            rows.append({"time_s": values[0], "H_proxy_m": values[1], "H_modal_m": values[2]})
            previous = values[0]
    except (csv.Error, ValueError) as exc:
        raise S6EngineV3Error("selected CSV malformed") from exc
    if len(rows) < 4:
        raise S6EngineV3Error("selected CSV too short")
    return {"source_sha256": sha256_bytes(raw), "source_schema": source_schema,
            "source_outcome": source_outcome, "rows": rows,
            "registered_overlap": {"start_s": rows[0]["time_s"], "end_s": rows[-1]["time_s"]},
            "optional_unread": True, "comparison_only": True}


def read_native_gauge_csvs(payloads: Mapping[str, bytes]) -> dict[str, Any]:
    policy, _ = load_contracts()
    probes = policy["gauge"]["probe_names"]
    if set(payloads) != set(probes):
        raise S6EngineV3Error("exact sixteen Gauge payloads required")
    series, counters, common_times = {}, {}, None
    for probe in probes:
        reader = csv.DictReader(io.StringIO(payloads[probe].decode("utf-8-sig"), newline=""), strict=True)
        if tuple(reader.fieldnames or ()) != ("time_s", "zsurf_m"):
            raise S6EngineV3Error("native Gauge CSV header differs")
        values, times, counts, previous = [], [], {"valid": 0, "missing": 0, "invalid": 0}, -math.inf
        for row in reader:
            if None in row or set(row) != {"time_s", "zsurf_m"}:
                raise S6EngineV3Error("native Gauge CSV row open")
            time_s = float(row["time_s"])
            if not math.isfinite(time_s) or time_s <= previous:
                raise S6EngineV3Error("native Gauge time invalid")
            token = row["zsurf_m"].strip()
            if token in ("", "NA", "N/A", "null"):
                value, reason = None, "MISSING"; counts["missing"] += 1
            else:
                try:
                    parsed = float(token)
                except ValueError:
                    value, reason = None, "INVALID_NUMERIC"; counts["invalid"] += 1
                else:
                    if not math.isfinite(parsed) or not policy["gauge"]["point0_z_m"] <= parsed <= policy["gauge"]["point2_z_m"]:
                        value, reason = None, "INVALID_RANGE_OR_NONFINITE"; counts["invalid"] += 1
                    else:
                        value, reason = parsed, None; counts["valid"] += 1
            times.append(time_s); values.append({"value_m": value, "reason": reason}); previous = time_s
        if len(times) < 4 or (common_times is not None and times != common_times):
            raise S6EngineV3Error("native Gauge grids differ or are too short")
        common_times = common_times or times
        total = len(times)
        if (counts["missing"] + counts["invalid"]) / total > policy["gauge"]["maximum_invalid_ratio"]:
            raise S6EngineV3Error("native Gauge invalid ratio exceeds v4")
        series[probe], counters[probe] = values, counts
    rows = []
    for index, time_s in enumerate(common_times or []):
        valid = [series[probe][index]["value_m"] for probe in probes if series[probe][index]["value_m"] is not None]
        if len(valid) < policy["gauge"]["minimum_valid_probes_per_slot"]:
            raise S6EngineV3Error("Gauge slot has insufficient probes")
        eta = [value - policy["gauge"]["h0_m"] for value in valid]
        rows.append({"time_s": time_s, "H_crest_m": max(eta), "H_abs_m": max(abs(value) for value in eta),
                     "H_peak_to_peak_m": max(valid) - min(valid), "valid_probe_count": len(valid)})
    qc = []
    for probe in probes:
        counts, total = counters[probe], len(common_times or [])
        qc.append({"name": probe, **counts, "missing_ratio": counts["missing"] / total,
                   "invalid_ratio": counts["invalid"] / total, "pass": True})
    return {"source_sha256": sha256_json({name: sha256_bytes(payloads[name]) for name in probes}),
            "time_grid": common_times, "time_grid_sha256": sha256_json(common_times),
            "rows": rows, "per_probe": qc}


def _interpolate(rows: Sequence[Mapping[str, float]], key: str, query: float) -> float:
    times = [row["time_s"] for row in rows]
    index = bisect.bisect_left(times, query)
    if index < len(times) and abs(times[index] - query) <= 1e-12:
        return float(rows[index][key])
    if index == 0 or index == len(times):
        raise S6EngineV3Error("extrapolation forbidden")
    left, right = rows[index - 1], rows[index]
    weight = (query - left["time_s"]) / (right["time_s"] - left["time_s"])
    return float(left[key] + weight * (right[key] - left[key]))


def _quantile(values: Sequence[float], q: float) -> float:
    ordered = sorted(values); position = (len(ordered) - 1) * q
    lower, upper = math.floor(position), math.ceil(position)
    return ordered[lower] if lower == upper else ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _stats(rows: Sequence[Mapping[str, float]], key: str) -> dict[str, float]:
    values = [abs(float(row[key])) for row in rows]
    peak_index = max(range(len(values)), key=values.__getitem__)
    return {"peak": values[peak_index], "p95": _quantile(values, .95),
            "rms": math.sqrt(sum(value * value for value in values) / len(values)),
            "peak_time_s": float(rows[peak_index]["time_s"])}


def _metric(times: Sequence[float], values: Sequence[float]) -> dict[str, float]:
    if len(times) < 4:
        raise S6EngineV3Error("metric grid too short")
    dt = (times[-1] - times[0]) / (len(times) - 1)
    if dt <= 0 or any(abs((times[i] - times[i - 1]) - dt) > max(1e-9, dt * 1e-6) for i in range(1, len(times))):
        raise S6EngineV3Error("comparison grid must be uniform")
    mean = sum(values) / len(values); centered = [value - mean for value in values]
    best = (0, 0.0, 0.0, -1.0)
    for k in range(1, len(values) // 2 + 1):
        real = sum(value * math.cos(2 * math.pi * k * i / len(values)) for i, value in enumerate(centered))
        imag = -sum(value * math.sin(2 * math.pi * k * i / len(values)) for i, value in enumerate(centered))
        power = real * real + imag * imag
        if power > best[3]: best = (k, real, imag, power)
    split = len(centered) // 2
    first = math.sqrt(sum(value * value for value in centered[:split]) / max(1, split))
    second = math.sqrt(sum(value * value for value in centered[split:]) / max(1, len(centered) - split))
    damping = math.log(max(first, 1e-15) / max(second, 1e-15)) / max(dt, times[-1] - times[split])
    return {"amplitude": max(abs(value) for value in centered), "frequency_hz": best[0] / (len(values) * dt),
            "damping_per_s": damping, "phase_rad": math.atan2(best[2], best[1])}


def analyze(gauge: Mapping[str, Any], selected: Mapping[str, Any],
            windows: Mapping[str, Mapping[str, float]]) -> dict[str, Any]:
    policy, schemas = load_contracts()
    if set(windows) != set(WINDOWS):
        raise S6EngineV3Error("four windows required")
    ranges = {name: {"start_s": float(windows[name]["start_s"]), "end_s": float(windows[name]["end_s"])} for name in WINDOWS}
    if any(value["end_s"] < value["start_s"] for value in ranges.values()):
        raise S6EngineV3Error("window reversed")
    if ranges["first15"]["start_s"] != ranges["full_motion"]["start_s"] or ranges["first15"]["end_s"] > min(ranges["full_motion"]["end_s"], ranges["first15"]["start_s"] + 15.0) + 1e-9:
        raise S6EngineV3Error("first15 topology differs")
    if ranges["recorded_tail"]["start_s"] < ranges["full_motion"]["end_s"] or ranges["solver_tail"]["start_s"] < ranges["recorded_tail"]["end_s"]:
        raise S6EngineV3Error("tail topology overlaps")
    overlap = selected["registered_overlap"]
    solver_rows, comparison_rows = [], []
    for source in gauge["rows"]:
        inside = overlap["start_s"] - 1e-12 <= source["time_s"] <= overlap["end_s"] + 1e-12
        row = dict(source)
        for name in SECONDARIES:
            row[f"{name}_m"] = _interpolate(selected["rows"], f"{name}_m", source["time_s"]) if inside else None
            row[f"{name}_coverage"] = "IN_REGISTERED_OVERLAP" if inside else "NA_OUTSIDE_REGISTERED_OVERLAP"
        solver_rows.append(row)
        if inside:
            comparison_rows.append({key: row[key] for key in ("time_s", "H_crest_m", "H_abs_m", "H_peak_to_peak_m", "H_proxy_m", "H_modal_m")})
    if len(comparison_rows) < 4:
        raise S6EngineV3Error("comparison overlap too short")
    window_stats = {}
    for name, limits in ranges.items():
        subset = [row for row in solver_rows if limits["start_s"] - 1e-12 <= row["time_s"] <= limits["end_s"] + 1e-12]
        if not subset: raise S6EngineV3Error("window empty")
        window_stats[name] = {surface: _stats(subset, f"{surface}_m") for surface in SURFACES}
    comparison_times = [row["time_s"] for row in comparison_rows]
    values = {name: [float(row[f"{name}_m"]) for row in comparison_rows] for name in (*SURFACES, *SECONDARIES)}
    metrics = {name: _metric(comparison_times, series) for name, series in values.items()}
    comparisons = []
    for surface in SURFACES:
        for secondary in SECONDARIES:
            x, y = values[surface], values[secondary]
            xm, ym = sum(x) / len(x), sum(y) / len(y)
            covariance = sum((a - xm) * (b - ym) for a, b in zip(x, y))
            xv, yv = sum((a - xm) ** 2 for a in x), sum((b - ym) ** 2 for b in y)
            corr = covariance / math.sqrt(xv * yv) if xv and yv else 0.0
            comparisons.append({"surface": surface, "secondary": secondary,
                "amplitude_error": metrics[secondary]["amplitude"] - metrics[surface]["amplitude"],
                "frequency_error_hz": metrics[secondary]["frequency_hz"] - metrics[surface]["frequency_hz"],
                "damping_error_per_s": metrics[secondary]["damping_per_s"] - metrics[surface]["damping_per_s"],
                "phase_error_rad": (metrics[secondary]["phase_rad"] - metrics[surface]["phase_rad"] + math.pi) % (2 * math.pi) - math.pi,
                "correlation": max(-1.0, min(1.0, corr)),
                "rmse": math.sqrt(sum((a - b) ** 2 for a, b in zip(x, y)) / len(x)), "ranking_claimed": False})
    solver_times = [row["time_s"] for row in solver_rows]
    result = {"schema_version": "smpcc-r8-liquid-s6-final-delivery-analysis-v3",
      "document_type": "SMPCC_R8_LIQUID_S6_FINAL_DELIVERY_ANALYSIS_V3",
      "status": "S6_REAL_ENGINE_ANALYSIS_COMPLETE_NOT_PUBLISHED_NOT_ACCEPTED", "attempt_id": ATTEMPT, "planned_denominator": 1,
      "provenance": {"gauge_sha256": gauge["source_sha256"], "selected_sha256": selected["source_sha256"],
        "gauge_contract_sha256": policy["bindings"]["s5b0_v4_gauge_contract_sha256"], "source_schema": selected["source_schema"],
        "source_outcome": selected["source_outcome"], "optional_unread": True},
      "grids": {"solver": {"count": len(solver_times), "start_s": solver_times[0], "end_s": solver_times[-1], "sha256": sha256_json(solver_times)},
        "comparison": {"count": len(comparison_times), "start_s": comparison_times[0], "end_s": comparison_times[-1], "sha256": sha256_json(comparison_times)},
        "overlap": dict(overlap), "outside_overlap_encoding": "EXPLICIT_NULL_NA", "interpolation": "LINEAR_WITHIN_REGISTERED_OVERLAP_ONLY", "extrapolation": False, "smoothing": False},
      "windows": ranges, "probe_qc": {"registered_probe_count": 16, "row_count": len(solver_rows), "per_probe": gauge["per_probe"]},
      "solver_rows": solver_rows, "comparison_rows": comparison_rows, "window_statistics": window_stats,
      "series_metrics": metrics, "comparisons": comparisons,
      "figure_model": {"layout": "THREE_VERTICAL_PANELS", "shared_x": True, "dual_y_axes": False,
        "palette": "OKABE_ITO_WITH_REDUNDANT_LINE_STYLE", "panels": ["SURFACES", "SECONDARIES", "HCREST_RESIDUALS"], "physical_reference_label": "PENDING"},
      "claims": {"stage6_pass": False, "development_only": True, "single_row": True, "optional_unread": True,
        "paired_ranking": False, "cross_method_ranking": False, "selected_trajectory_cpu_comparison": False,
        "physical_reference_pending": True, "physical_fidelity_validated": False, "formal": False, "production": False, "physical_primary": False}}
    Draft202012Validator(schemas["analysis"]).validate(result)
    return result


def validate_media_content(frame_manifest: Mapping[str, Any], rendered_frames: Mapping[str, bytes],
                           media_manifest: Mapping[str, Any]) -> dict[str, Any]:
    if frame_manifest.get("status") != "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1" or frame_manifest.get("integrity_pass") is not True:
        raise S6EngineV3Error("finalized frame manifest not admitted")
    frames = frame_manifest.get("frames")
    if not isinstance(frames, list) or len(frames) < 3 or set(media_manifest) != {"frame_manifest_sha256", "frames", "mp4_sha256", "gif_sha256", "decoded_mp4_frames", "decoded_gif_frames", "keyframe_indices"}:
        raise S6EngineV3Error("media manifest shape differs")
    if media_manifest["frame_manifest_sha256"] != sha256_json(frame_manifest) or len(media_manifest["frames"]) != len(frames):
        raise S6EngineV3Error("media parent/count differs")
    previous = -math.inf
    for source, media in zip(frames, media_manifest["frames"]):
        if set(media) != {"relative_path", "sha256", "time_s", "class_counts_sha256", "probe_overlay_sha256", "container_frame"}:
            raise S6EngineV3Error("media frame binding open")
        payload = rendered_frames.get(media["relative_path"])
        if payload is None or sha256_bytes(payload) != media["sha256"] or media["time_s"] != source["time_s"] or media["class_counts_sha256"] != source["class_counts_sha256"]:
            raise S6EngineV3Error("media frame content/parent differs")
        if SHA_RE.fullmatch(str(media["probe_overlay_sha256"])) is None or media["container_frame"] != "MOVING_CONTAINER_REFERENCE_REF_0" or media["time_s"] <= previous:
            raise S6EngineV3Error("media probe/container/time binding differs")
        previous = media["time_s"]
    if media_manifest["decoded_mp4_frames"] != len(frames) or media_manifest["decoded_gif_frames"] != len(frames) or media_manifest["keyframe_indices"] != [0, len(frames)//2, len(frames)-1]:
        raise S6EngineV3Error("media decode/keyframes differ")
    if any(SHA_RE.fullmatch(str(media_manifest[name])) is None for name in ("mp4_sha256", "gif_sha256")):
        raise S6EngineV3Error("media hashes invalid")
    return {"status": "MEDIA_CONTENT_BINDING_VALIDATED_NOT_FINAL_ACCEPTANCE", "pass": True,
            "frame_manifest_sha256": media_manifest["frame_manifest_sha256"], "frame_count": len(frames),
            "stage6_pass": False}


def _rename_noreplace(source: Path, destination: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    result = libc.syscall(SYS_RENAMEAT2, AT_FDCWD, os.fsencode(source), AT_FDCWD, os.fsencode(destination), RENAME_NOREPLACE)
    if result != 0:
        number = ctypes.get_errno()
        raise OSError(number, os.strerror(number), str(destination))


def atomic_publish_inventory(final_root: Path, artifacts: Mapping[str, bytes], required_inventory: Sequence[str]) -> dict[str, Any]:
    """Atomically publish an exact inventory; this helper never claims S6 PASS."""
    final = Path(final_root).absolute(); partial = final.with_name(final.name + ".partial")
    required = list(required_inventory)
    if len(required) != 17 or len(set(required)) != 17 or set(artifacts) != set(required) or final.exists() or partial.exists():
        raise S6EngineV3Error("publication root/inventory not create-new exact")
    for name, payload in artifacts.items():
        _safe_relative(name)
        if not isinstance(payload, bytes): raise S6EngineV3Error("artifact payload not bytes")
    partial.mkdir(mode=0o700, parents=False, exist_ok=False)
    try:
        for name in sorted(required):
            target = partial / name; target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o440)
            try:
                view = memoryview(artifacts[name])
                while view:
                    written = os.write(fd, view); view = view[written:]
                os.fsync(fd)
            finally: os.close(fd)
        dir_fd = os.open(partial, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
        try: os.fsync(dir_fd)
        finally: os.close(dir_fd)
        _rename_noreplace(partial, final)
        parent_fd = os.open(final.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
        try: os.fsync(parent_fd)
        finally: os.close(parent_fd)
    except Exception:
        raise
    identities = {name: {"sha256": sha256_bytes(artifacts[name]), "size_bytes": len(artifacts[name])} for name in required}
    return {"status": "PUBLISHED_CREATE_NEW_NOT_ACCEPTED", "root": str(final), "inventory": identities,
            "inventory_sha256": sha256_json(identities), "stage6_pass": False}


def append_hash_chain_ledger(path: Path, payload: Mapping[str, Any], *, expected_previous_sha256: str) -> dict[str, Any]:
    """Serialize one canonical payload under an exclusive append lock and fsync."""
    if SHA_RE.fullmatch(expected_previous_sha256) is None or path.is_symlink():
        raise S6EngineV3Error("ledger parent/path invalid")
    fd = os.open(path, os.O_RDWR | os.O_APPEND | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        before = os.fstat(fd)
        os.lseek(fd, 0, os.SEEK_SET); existing = _read_fd_all(fd)
        lines = [json.loads(line) for line in existing.splitlines() if line]
        observed = "0" * 64
        seen = set()
        for entry in lines:
            if set(entry) != {"entry_sha256", "previous_entry_sha256", "payload"} or entry["previous_entry_sha256"] != observed or entry["entry_sha256"] != sha256_json(entry["payload"]):
                raise S6EngineV3Error("ledger chain invalid")
            key = entry["payload"].get("attempt_id")
            if key in seen: raise S6EngineV3Error("ledger duplicate attempt")
            seen.add(key); observed = entry["entry_sha256"]
        if observed != expected_previous_sha256:
            raise S6EngineV3Error("ledger compare-and-append failed")
        if payload.get("attempt_id") in seen:
            raise S6EngineV3Error("ledger duplicate attempt")
        entry = {"entry_sha256": sha256_json(payload), "previous_entry_sha256": observed, "payload": dict(payload)}
        encoded = canonical_json(entry); os.write(fd, encoded); os.fsync(fd)
        after = os.fstat(fd)
        if (before.st_dev, before.st_ino) != (after.st_dev, after.st_ino) or after.st_size != before.st_size + len(encoded):
            raise S6EngineV3Error("ledger append TOCTOU/size mismatch")
        return {"status": "APPENDED_HASH_CHAIN_ENTRY_NOT_STAGE6_ACCEPTANCE", "entry_sha256": entry["entry_sha256"],
                "previous_entry_sha256": observed, "before_size": before.st_size, "after_size": after.st_size,
                "stage6_pass": False}
    finally:
        try: fcntl.flock(fd, fcntl.LOCK_UN)
        finally: os.close(fd)


def planned_delivery_assets(analysis_result: Mapping[str, Any], opaque_assets: Mapping[str, bytes]) -> dict[str, bytes]:
    """Build all 17 bytes in memory; media/figure bytes must come from bound engines."""
    policy, _ = load_contracts(); required = policy["delivery"]["required_inventory"]
    base = dict(opaque_assets)
    base["reports/analysis_result.json"] = canonical_json(analysis_result)
    comparison = {"schema_version": "smpcc-r8-liquid-s6-comparison-manifest-v3", "attempt_id": ATTEMPT,
                  "analysis_sha256": sha256_json(analysis_result), "ranking_claimed": False,
                  "physical_reference_pending": True, "stage6_pass": False}
    base["comparison_manifest.json"] = canonical_json(comparison)
    evidence = {"schema_version": "smpcc-r8-liquid-s6-evidence-index-v3",
                "entries": [{"relative_path": name, "sha256": sha256_bytes(payload)} for name, payload in sorted(base.items())],
                "stage6_pass": False}
    base["evidence_index.json"] = canonical_json(evidence)
    ledger_payload = {"attempt_id": ATTEMPT, "analysis_sha256": comparison["analysis_sha256"], "stage6_pass": False}
    ledger_entry = {"entry_sha256": sha256_json(ledger_payload), "previous_entry_sha256": "0"*64, "payload": ledger_payload}
    base["liquid_secondary_ledger.jsonl"] = canonical_json(ledger_entry)
    base["secondary_ledger_append_receipt.json"] = canonical_json({"status": "PLANNED_LOCAL_LEDGER_ENTRY_NOT_EXTERNAL_APPEND", "entry_sha256": ledger_entry["entry_sha256"], "stage6_pass": False})
    base["acceptance_receipt.json"] = canonical_json({"status": "NOT_ACCEPTED_RUNTIME_PARENT_POLICY_REQUIRED", "attempt_id": ATTEMPT, "stage6_pass": False})
    missing_before_checksum = set(required) - {"checksums.sha256"} - set(base)
    if missing_before_checksum or set(base) - set(required):
        raise S6EngineV3Error(f"opaque delivery assets differ: {sorted(missing_before_checksum)}")
    base["checksums.sha256"] = "".join(f"{sha256_bytes(base[name])}  {name}\n" for name in sorted(base)).encode("ascii")
    if set(base) != set(required): raise S6EngineV3Error("17-item delivery build differs")
    return base


def static_receipt(command: str) -> dict[str, Any]:
    policy, schemas = load_contracts()
    if command not in COMMANDS: raise S6EngineV3Error("command differs")
    if policy["parent"]["state"] != "NOT_MATERIALIZED": raise S6EngineV3Error("v3 static policy drift")
    value = {"schema_version": "smpcc-r8-liquid-s6-final-delivery-engine-receipt-v3",
             "document_type": "SMPCC_R8_LIQUID_S6_FINAL_DELIVERY_ENGINE_RECEIPT_V3",
             "status": NOT_ADMITTED, "command": command, "attempt_id": ATTEMPT,
             "policy_sha256": _sha_file(POLICY_PATH), "parent_materialized": False,
             "external_write_performed": False, "optional_bag_read": False, "stage6_pass": False}
    Draft202012Validator(schemas["engine"]).validate(value); return value


def self_check() -> dict[str, Any]:
    policy, schemas = load_contracts()
    receipts = {command: static_receipt(command)["status"] for command in COMMANDS}
    return {"status": "S6_FINAL_DELIVERY_ENGINE_V3_SELF_CHECK_OK_NOT_ADMITTED", "commands": receipts,
            "schemas_deep_closed": len(schemas) == 3, "probe_count": len(policy["gauge"]["probe_names"]),
            "required_inventory_count": len(policy["delivery"]["required_inventory"]),
            "parent_materialized": False, "external_write_performed": False, "optional_bag_read": False,
            "solver_or_gpu_executed": False, "network_used": False, "sudo_used": False, "stage6_pass": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", *COMMANDS)); args = parser.parse_args(argv)
    try:
        value = self_check() if args.command == "self-check" else static_receipt(args.command)
        print(json.dumps(value, ensure_ascii=False, sort_keys=True))
    except (S6EngineV3Error, ValidationError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "S6_FINAL_DELIVERY_ENGINE_V3_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr); return 1
    return 0


if __name__ == "__main__": raise SystemExit(main())


__all__ = ["COMMANDS", "S6EngineV3Error", "admit_finalized_package", "analyze", "append_hash_chain_ledger",
           "assert_deep_closed", "atomic_publish_inventory", "canonical_json", "load_contracts",
           "planned_delivery_assets", "read_native_gauge_csvs", "read_selected_signals_csv", "self_check",
           "sha256_bytes", "sha256_json", "static_receipt", "validate_media_content", "validate_native_gauge_manifest"]
