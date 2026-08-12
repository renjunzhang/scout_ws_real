#!/usr/bin/env python3
"""Create-new S6 primary real-runtime delivery engine.

The repository policy is default-deny.  Real operation is exposed as library
functions which require an externally materialized, closed v6 runtime contract.
The public CLI is self-check only and never opens external inputs.  The engine
does not run ROS, a solver, GPU code, sudo or networking and has no optional-bag
argument or discovery path.
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
MODULE_DIR = Path(__file__).resolve().parent
ROOT = MODULE_DIR.parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_bi4_reader_v1 as bi4  # noqa: E402
import r8_liquid_s6_real_selected_signal_extractor_v5 as selected_v5  # noqa: E402

POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s6_real_runtime_delivery_policy_v6.json"
SCHEMA_PATHS = {
    "policy": ROOT / "schema/target_host_s6_real_runtime_delivery_policy_v6.json",
    "runtime": ROOT / "schema/target_host_s6_real_runtime_contract_v6.json",
    "result": ROOT / "schema/target_host_s6_real_runtime_result_v6.json",
}
ATTEMPT = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
NOT_ADMITTED = "NOT_ADMITTED_FINALIZED_S5B0_RUNTIME_CONTRACT_REQUIRED"
FINAL = "S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY"
PROBES = tuple(f"s5b0_p{i:02d}" for i in range(16))
SURFACES = ("H_crest", "H_abs", "H_peak_to_peak")
SECONDARIES = ("H_proxy", "H_modal")
WINDOWS = ("first15", "full_motion", "recorded_tail", "solver_tail")
SHA_RE = re.compile(r"[0-9a-f]{64}\Z")
PART_RE = re.compile(r"data/Part_([0-9]{4})\.bi4\Z")
MAX_FILE_BYTES = 128 * 1024 * 1024
MAX_BAG_BYTES = selected_v5.MAXIMUM_SOURCE_BYTES
AT_FDCWD = -100
RENAME_NOREPLACE = 1
SYS_RENAMEAT2 = 316


class S6RuntimeV6Error(ValueError):
    """A closed contract, identity, numeric rule or create-new invariant failed."""


def canonical_json(value: Any) -> bytes:
    try:
        return (json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                           separators=(",", ":")) + "\n").encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise S6RuntimeV6Error("value is not finite canonical JSON") from exc


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def _json_bytes(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(raw, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    except (UnicodeError, ValueError) as exc:
        raise S6RuntimeV6Error(f"invalid JSON: {label}") from exc
    if not isinstance(value, dict):
        raise S6RuntimeV6Error(f"JSON root is not object: {label}")
    return value


def _read_static_json(path: Path) -> dict[str, Any]:
    return _json_bytes(path.read_bytes(), path.name)


def assert_deep_closed(node: Any, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise S6RuntimeV6Error(f"schema object open at {location}")
        for key, child in node.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(node, list):
        for index, child in enumerate(node):
            assert_deep_closed(child, f"{location}/{index}")


def load_contracts() -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    policy = _read_static_json(POLICY_PATH)
    schemas = {name: _read_static_json(path) for name, path in SCHEMA_PATHS.items()}
    for schema in schemas.values():
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    Draft202012Validator(schemas["policy"]).validate(policy)
    if tuple(policy["gauge"]["probe_names"]) != PROBES or tuple(policy["delivery"]["required_inventory"]).__len__() != 19:
        raise S6RuntimeV6Error("policy probe/inventory topology differs")
    return policy, schemas


def _safe_relative(value: str) -> str:
    pure = PurePosixPath(value)
    if not value or pure.is_absolute() or str(pure) != value or any(part in ("", ".", "..") for part in pure.parts):
        raise S6RuntimeV6Error("unsafe relative path")
    return value


def _identity_tuple(row: os.stat_result) -> tuple[int, ...]:
    return (row.st_dev, row.st_ino, row.st_mode, row.st_nlink, row.st_size,
            row.st_mtime_ns, row.st_ctime_ns)


def _assert_path_components(path: Path, *, final_may_be_absent: bool = False) -> None:
    if not path.is_absolute() or str(path) != os.path.normpath(str(path)):
        raise S6RuntimeV6Error("path is not normalized absolute")
    cursor = Path(path.anchor)
    parts = path.parts[1:-1] if final_may_be_absent else path.parts[1:]
    for part in parts:
        cursor /= part
        metadata = os.lstat(cursor)
        if stat.S_ISLNK(metadata.st_mode):
            raise S6RuntimeV6Error("symlink path component")


def _read_fd_all(fd: int, maximum: int) -> bytes:
    chunks: list[bytes] = []
    total = 0
    while True:
        block = os.read(fd, min(1 << 20, maximum + 1 - total))
        if not block:
            return b"".join(chunks)
        total += len(block)
        if total > maximum:
            raise S6RuntimeV6Error("file exceeds bound")
        chunks.append(block)


def read_exact_file(path: Path, expected: Mapping[str, Any], *, maximum: int = MAX_FILE_BYTES) -> bytes:
    """Open one exact identity using O_NOFOLLOW and verify before/after fstat."""
    path = Path(path)
    _assert_path_components(path)
    if set(expected) != {"path", "sha256", "size_bytes", "mode", "device", "inode", "nlink", "mtime_ns", "ctime_ns"} or expected["path"] != str(path):
        raise S6RuntimeV6Error("expected file identity is open or path differs")
    if SHA_RE.fullmatch(str(expected["sha256"])) is None or expected["nlink"] != 1:
        raise S6RuntimeV6Error("expected file identity invalid")
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise S6RuntimeV6Error("opened file is not regular single-link")
        observed = {"path": str(path), "size_bytes": before.st_size,
                    "mode": f"{stat.S_IMODE(before.st_mode):04o}", "device": before.st_dev,
                    "inode": before.st_ino, "nlink": before.st_nlink,
                    "mtime_ns": before.st_mtime_ns, "ctime_ns": before.st_ctime_ns}
        if any(expected[key] != observed[key] for key in observed):
            raise S6RuntimeV6Error("opened file metadata differs from contract")
        payload = _read_fd_all(fd, maximum)
        after = os.fstat(fd)
    finally:
        os.close(fd)
    if _identity_tuple(before) != _identity_tuple(after) or len(payload) != before.st_size or sha256_bytes(payload) != expected["sha256"]:
        raise S6RuntimeV6Error("file hash/TOCTOU differs from contract")
    return payload


def _open_relative_at(root_fd: int, relative: str) -> int:
    parts = PurePosixPath(_safe_relative(relative)).parts
    directory_fd = os.dup(root_fd)
    try:
        for part in parts[:-1]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                            dir_fd=directory_fd)
            os.close(directory_fd)
            directory_fd = child
        return os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC,
                       dir_fd=directory_fd)
    finally:
        os.close(directory_fd)


def admit_finalized_s5b0(contract: Mapping[str, Any]) -> dict[str, Any]:
    """Consume exactly the inventory frozen by a deep-closed v6 runtime contract."""
    policy, schemas = load_contracts()
    Draft202012Validator(schemas["runtime"]).validate(dict(contract))
    policy_raw = read_exact_file(Path(contract["parents"]["policy"]["path"]), contract["parents"]["policy"])
    if policy_raw != POLICY_PATH.read_bytes():
        raise S6RuntimeV6Error("runtime contract policy bytes differ from repository v6 policy")
    root = Path(contract["s5b0"]["root"])
    _assert_path_components(root)
    records = contract["s5b0"]["inventory"]
    expected = {row["relative_path"]: row for row in records}
    if len(expected) != len(records) or sha256_json(records) != contract["s5b0"]["inventory_sha256"]:
        raise S6RuntimeV6Error("runtime inventory digest/duplicates differ")
    root_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    payloads: dict[str, bytes] = {}
    try:
        root_before = os.fstat(root_fd)
        observed_names: set[str] = set()

        def walk(directory_fd: int, prefix: str = "") -> None:
            for name in sorted(os.listdir(directory_fd)):
                relative = f"{prefix}/{name}" if prefix else name
                metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
                if stat.S_ISDIR(metadata.st_mode):
                    child = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                                    dir_fd=directory_fd)
                    try:
                        walk(child, relative)
                    finally:
                        os.close(child)
                elif stat.S_ISREG(metadata.st_mode) and metadata.st_nlink == 1:
                    observed_names.add(relative)
                else:
                    raise S6RuntimeV6Error("S5B0 tree contains symlink, hardlink or special file")
        walk(root_fd)
        if observed_names != set(expected):
            raise S6RuntimeV6Error("S5B0 exact inventory has missing/extra files")
        for relative, row in expected.items():
            fd = _open_relative_at(root_fd, relative)
            try:
                before = os.fstat(fd)
                payload = _read_fd_all(fd, MAX_FILE_BYTES)
                after = os.fstat(fd)
            finally:
                os.close(fd)
            observed = {"relative_path": relative, "sha256": sha256_bytes(payload),
                        "size_bytes": len(payload), "mode": f"{stat.S_IMODE(after.st_mode):04o}",
                        "device": after.st_dev, "inode": after.st_ino, "nlink": after.st_nlink,
                        "mtime_ns": after.st_mtime_ns, "ctime_ns": after.st_ctime_ns}
            if _identity_tuple(before) != _identity_tuple(after) or observed != row:
                raise S6RuntimeV6Error("S5B0 file identity/hash/TOCTOU differs")
            payloads[relative] = payload
        root_after = os.fstat(root_fd)
    finally:
        os.close(root_fd)
    if _identity_tuple(root_before) != _identity_tuple(root_after):
        raise S6RuntimeV6Error("S5B0 root TOCTOU detected")
    required = {"execution_receipt.json", "result_qc.json", "frame_manifest.json",
                "native_gauge_manifest.json", "checksums.sha256"}
    if not required.issubset(payloads):
        raise S6RuntimeV6Error("S5B0 required evidence absent")
    parents = contract["s5b0"]["required_parents"]
    mapping = {"execution_receipt.json": "execution_receipt_sha256", "result_qc.json": "result_qc_sha256",
               "frame_manifest.json": "frame_manifest_sha256", "native_gauge_manifest.json": "native_gauge_manifest_sha256",
               "checksums.sha256": "checksums_sha256"}
    if any(sha256_bytes(payloads[path]) != parents[key] for path, key in mapping.items()):
        raise S6RuntimeV6Error("S5B0 required parent digest differs")
    parsed_checksums: dict[str, str] = {}
    for line in payloads["checksums.sha256"].decode("ascii").splitlines():
        if re.fullmatch(r"[0-9a-f]{64}  (?!/)(?!.*(?:^|/)\.\.(?:/|$)).+", line) is None:
            raise S6RuntimeV6Error("S5B0 checksum syntax invalid")
        digest, name = line.split("  ", 1)
        if name in parsed_checksums:
            raise S6RuntimeV6Error("S5B0 duplicate checksum")
        parsed_checksums[name] = digest
    if set(parsed_checksums) != set(payloads) - {"checksums.sha256"} or any(sha256_bytes(payloads[name]) != digest for name, digest in parsed_checksums.items()):
        raise S6RuntimeV6Error("S5B0 checksums incomplete or differ")
    execution = _json_bytes(payloads["execution_receipt.json"], "execution receipt")
    qc = _json_bytes(payloads["result_qc.json"], "result QC")
    frames = _json_bytes(payloads["frame_manifest.json"], "frame manifest")
    gauge_manifest = _json_bytes(payloads["native_gauge_manifest.json"], "Gauge manifest")
    if execution.get("status") != policy["parent_contract"]["required_execution_status"] or execution.get("finalized") is not True or execution.get("attempt_id") != ATTEMPT:
        raise S6RuntimeV6Error("S5B0 execution receipt differs")
    if qc.get("status") != policy["parent_contract"]["required_qc_status"] or qc.get("pass") is not True or qc.get("attempt_id") != ATTEMPT:
        raise S6RuntimeV6Error("S5B0 result QC differs")
    validate_frame_manifest(frames)
    validate_gauge_manifest(gauge_manifest, payloads)
    return {"status": "ADMITTED_FINALIZED_S5B0_PRIMARY_PACKAGE_V6", "payloads": payloads,
            "execution": execution, "qc": qc, "frames": frames, "gauge_manifest": gauge_manifest,
            "checks": {"exact_root": True, "exact_inventory": True, "checksums_complete": True,
                       "no_symlinks": True, "no_hardlinks": True, "no_special_files": True,
                       "no_toctou": True, "optional_unread": True}}


def validate_gauge_manifest(value: Mapping[str, Any], payloads: Mapping[str, bytes]) -> None:
    if set(value) != {"schema_version", "attempt_id", "gauge_contract_sha256", "time_grid_sha256", "files"} or value["schema_version"] != "smpcc-r8-liquid-s5b0-native-gauge-manifest-v1" or value["attempt_id"] != ATTEMPT:
        raise S6RuntimeV6Error("native Gauge manifest is not exact/closed")
    files = value["files"]
    if not isinstance(files, list) or [row.get("probe_name") for row in files] != list(PROBES):
        raise S6RuntimeV6Error("native Gauge probe set/order differs")
    for row in files:
        if set(row) != {"probe_name", "relative_path", "sha256", "size_bytes", "time_grid_sha256"} or row["time_grid_sha256"] != value["time_grid_sha256"]:
            raise S6RuntimeV6Error("native Gauge file identity is open or grid differs")
        relative = _safe_relative(row["relative_path"])
        payload = payloads.get(relative)
        if payload is None or sha256_bytes(payload) != row["sha256"] or len(payload) != row["size_bytes"]:
            raise S6RuntimeV6Error("native Gauge CSV bytes differ from manifest")


def validate_frame_manifest(value: Mapping[str, Any]) -> None:
    required = {"schema_version", "attempt_id", "status", "integrity_pass", "root", "frames"}
    if set(value) != required or value["schema_version"] != "smpcc-r8-liquid-s5b0-finalized-frame-manifest-v1" or value["attempt_id"] != ATTEMPT or value["status"] != "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1" or value["integrity_pass"] is not True:
        raise S6RuntimeV6Error("finalized frame manifest differs")
    frames = value["frames"]
    if not isinstance(frames, list) or len(frames) < 3:
        raise S6RuntimeV6Error("at least three finalized frames required")
    previous_index, previous_time = None, -math.inf
    for row in frames:
        if set(row) != {"index", "time_s", "relative_path", "sha256", "particle_count", "ids_sha256", "class_counts"}:
            raise S6RuntimeV6Error("frame row is not closed")
        index, time_s = row["index"], row["time_s"]
        if isinstance(index, bool) or not isinstance(index, int) or (previous_index is not None and index != previous_index + 1):
            raise S6RuntimeV6Error("frame indices not contiguous")
        if isinstance(time_s, bool) or not isinstance(time_s, (int, float)) or not math.isfinite(time_s) or time_s <= previous_time:
            raise S6RuntimeV6Error("frame times not strict finite")
        _safe_relative(row["relative_path"])
        if SHA_RE.fullmatch(str(row["sha256"])) is None or SHA_RE.fullmatch(str(row["ids_sha256"])) is None:
            raise S6RuntimeV6Error("frame digest invalid")
        if set(row["class_counts"]) != {"fixed_boundary", "moving_boundary", "floating", "fluid"} or sum(row["class_counts"].values()) != row["particle_count"]:
            raise S6RuntimeV6Error("frame class counts differ")
        previous_index, previous_time = index, float(time_s)


def extract_selected_signals(contract: Mapping[str, Any]) -> dict[str, Any]:
    """Read the exact primary bag via fd and reuse the byte-pinned ROS1 parser."""
    _, schemas = load_contracts()
    Draft202012Validator(schemas["runtime"]).validate(dict(contract))
    parents = contract["parents"]
    s5a0_raw = read_exact_file(Path(parents["s5a0_receipt"]["path"]), parents["s5a0_receipt"])
    transfer_raw = read_exact_file(Path(parents["s5a1_transfer_manifest"]["path"]), parents["s5a1_transfer_manifest"])
    bag_raw = read_exact_file(Path(parents["primary_bag"]["path"]), parents["primary_bag"], maximum=MAX_BAG_BYTES)
    s5a0 = _json_bytes(s5a0_raw, "S5A0 receipt")
    transfer = _json_bytes(transfer_raw, "S5A1 transfer")
    if s5a0.get("status") != "S5A0_SELECTED_R7_BAG_SEALED_DEVELOPMENT_ONLY" or s5a0.get("selection", {}).get("attempt_id") != ATTEMPT:
        raise S6RuntimeV6Error("S5A0 selected receipt differs")
    if transfer.get("status") != "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED" or transfer.get("source", {}).get("attempt_id") != ATTEMPT or transfer.get("source", {}).get("bag_sha256") != parents["primary_bag"]["sha256"]:
        raise S6RuntimeV6Error("S5A1 transfer/source bag binding differs")
    try:
        captured = selected_v5._capture_bag_bytes(bag_raw)
        alignment = selected_v5._alignment(captured["odom"])
        proxy = selected_v5._map_series(captured["signals"]["H_proxy"], "H_proxy", alignment, 1000.0)
        modal = selected_v5._map_series(captured["signals"]["H_modal"], "H_modal", alignment, 1.0)
    except selected_v5.SelectedSignalV5Error as exc:
        raise S6RuntimeV6Error(str(exc)) from exc
    if captured.get("reader_anomalies") != []:
        raise S6RuntimeV6Error("primary bag reader reported anomaly")
    overlap_start = max(proxy[0]["time_since_odom_origin_s"], modal[0]["time_since_odom_origin_s"])
    overlap_end = min(proxy[-1]["time_since_odom_origin_s"], modal[-1]["time_since_odom_origin_s"])
    if overlap_end <= overlap_start:
        raise S6RuntimeV6Error("secondary series have no common overlap")
    rows = []
    proxy_by_time = [{"time_s": row["time_since_odom_origin_s"], "value_m": row["value_comparison_mm"] / 1000.0} for row in proxy]
    modal_by_time = [{"time_s": row["time_since_odom_origin_s"], "value_m": row["value_comparison_mm"] / 1000.0} for row in modal]
    union_times = sorted({row["time_s"] for row in proxy_by_time + modal_by_time if overlap_start <= row["time_s"] <= overlap_end})
    for time_s in union_times:
        rows.append({"time_s": time_s, "H_proxy_m": _interpolate(proxy_by_time, "value_m", time_s),
                     "H_modal_m": _interpolate(modal_by_time, "value_m", time_s)})
    result = {"schema_version": "smpcc-r8-liquid-s6-real-selected-signals-v6",
              "status": "PRIMARY_REAL_BAG_SELECTED_SIGNALS_EXTRACTED_V6", "attempt_id": ATTEMPT,
              "source_bag_sha256": sha256_bytes(bag_raw), "source_schema": "SPMPC_NON_FIXED",
              "source_outcome": transfer.get("source", {}).get("source_outcome", "UNKNOWN"),
              "time_alignment": {"motion_time_source": "/odom.header.stamp", "record_to_header_offset_ns": alignment["offset_ns"],
                                 "residual_max_abs_ns": alignment["residual_max_abs_ns"], "overlap_start_s": overlap_start,
                                 "overlap_end_s": overlap_end, "extrapolation": False},
              "rows": rows, "series": {"H_proxy": proxy, "H_modal": modal},
              "claims": {"comparison_only": True, "optional_bag_read": False,
                         "motion_exporter_consumed": False, "solver_forcing_consumed": False,
                         "source_bag_executed": False, "ros_started": False}}
    canonical_json(result)
    return result


def read_native_gauges(admission: Mapping[str, Any]) -> dict[str, Any]:
    manifest, payloads = admission["gauge_manifest"], admission["payloads"]
    validate_gauge_manifest(manifest, payloads)
    policy, _ = load_contracts()
    series: dict[str, list[dict[str, Any]]] = {}
    counters: dict[str, dict[str, int]] = {}
    common_times: list[float] | None = None
    raw_identities = []
    for row in manifest["files"]:
        probe, raw = row["probe_name"], payloads[row["relative_path"]]
        try:
            reader = csv.DictReader(io.StringIO(raw.decode("utf-8-sig"), newline=""), strict=True)
        except UnicodeError as exc:
            raise S6RuntimeV6Error("Gauge CSV encoding invalid") from exc
        if tuple(reader.fieldnames or ()) != ("time_s", "zsurf_m"):
            raise S6RuntimeV6Error("Gauge CSV header differs")
        values, times = [], []
        counts = {"valid": 0, "missing": 0, "invalid": 0}
        previous = -math.inf
        try:
            for item in reader:
                if None in item or set(item) != {"time_s", "zsurf_m"}:
                    raise S6RuntimeV6Error("Gauge CSV row open")
                time_s = float(item["time_s"])
                if not math.isfinite(time_s) or time_s <= previous:
                    raise S6RuntimeV6Error("Gauge time invalid")
                token = item["zsurf_m"].strip()
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
        except (csv.Error, ValueError) as exc:
            raise S6RuntimeV6Error("Gauge CSV malformed") from exc
        if len(times) < 4 or (common_times is not None and times != common_times):
            raise S6RuntimeV6Error("Gauge grids differ or are too short")
        common_times = common_times or times
        if (counts["missing"] + counts["invalid"]) / len(times) > policy["gauge"]["maximum_invalid_ratio"]:
            raise S6RuntimeV6Error("Gauge invalid ratio exceeds contract")
        series[probe], counters[probe] = values, counts
        raw_identities.append({"probe_name": probe, "sha256": sha256_bytes(raw), "size_bytes": len(raw)})
    derived = []
    for index, time_s in enumerate(common_times or []):
        valid = [series[probe][index]["value_m"] for probe in PROBES if series[probe][index]["value_m"] is not None]
        if len(valid) < policy["gauge"]["minimum_valid_probes_per_slot"]:
            raise S6RuntimeV6Error("Gauge slot has insufficient valid probes")
        eta = [value - policy["gauge"]["h0_m"] for value in valid]
        derived.append({"time_s": time_s, "H_crest_m": max(eta), "H_abs_m": max(abs(value) for value in eta),
                        "H_peak_to_peak_m": max(valid) - min(valid), "valid_probe_count": len(valid)})
    per_probe = [{"name": probe, **counters[probe],
                  "missing_ratio": counters[probe]["missing"] / len(common_times or []),
                  "invalid_ratio": counters[probe]["invalid"] / len(common_times or [])} for probe in PROBES]
    return {"schema_version": "smpcc-r8-liquid-s6-native-gauge-derived-v6", "attempt_id": ATTEMPT,
            "fact_source": "SIXTEEN_RAW_NATIVE_JGAUGESWL_CSV", "time_grid": common_times,
            "time_grid_sha256": sha256_json(common_times), "raw_identities": raw_identities,
            "rows": derived, "per_probe": per_probe}


def _interpolate(rows: Sequence[Mapping[str, float]], key: str, query: float) -> float:
    times = [float(row["time_s"]) for row in rows]
    index = bisect.bisect_left(times, query)
    if index < len(times) and abs(times[index] - query) <= 1e-12:
        return float(rows[index][key])
    if index == 0 or index == len(times):
        raise S6RuntimeV6Error("extrapolation forbidden")
    left, right = rows[index - 1], rows[index]
    weight = (query - float(left["time_s"])) / (float(right["time_s"]) - float(left["time_s"]))
    return float(left[key]) + weight * (float(right[key]) - float(left[key]))


def _quantile(values: Sequence[float], q: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lower, upper = math.floor(position), math.ceil(position)
    return ordered[lower] if lower == upper else ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _window_stats(rows: Sequence[Mapping[str, Any]], key: str) -> dict[str, float]:
    values = [abs(float(row[key])) for row in rows]
    peak_index = max(range(len(values)), key=values.__getitem__)
    return {"peak": values[peak_index], "p95": _quantile(values, .95),
            "rms": math.sqrt(sum(value * value for value in values) / len(values)),
            "peak_time_s": float(rows[peak_index]["time_s"])}


def _spectral(times: Sequence[float], values: Sequence[float]) -> dict[str, float]:
    if len(times) < 4:
        raise S6RuntimeV6Error("comparison grid too short")
    dt = (times[-1] - times[0]) / (len(times) - 1)
    if dt <= 0 or any(abs((times[index] - times[index - 1]) - dt) > max(1e-9, dt * 1e-6) for index in range(1, len(times))):
        raise S6RuntimeV6Error("comparison grid must be uniform")
    mean = sum(values) / len(values)
    centered = [value - mean for value in values]
    best = (0, 0.0, 0.0, -1.0)
    for frequency_bin in range(1, len(values) // 2 + 1):
        real = sum(value * math.cos(2 * math.pi * frequency_bin * index / len(values)) for index, value in enumerate(centered))
        imag = -sum(value * math.sin(2 * math.pi * frequency_bin * index / len(values)) for index, value in enumerate(centered))
        power = real * real + imag * imag
        if power > best[3]:
            best = (frequency_bin, real, imag, power)
    split = len(centered) // 2
    first = math.sqrt(sum(value * value for value in centered[:split]) / max(1, split))
    second = math.sqrt(sum(value * value for value in centered[split:]) / max(1, len(centered) - split))
    damping = math.log(max(first, 1e-15) / max(second, 1e-15)) / max(dt, times[-1] - times[split])
    return {"amplitude": max(abs(value) for value in centered),
            "frequency_hz": best[0] / (len(values) * dt), "damping_per_s": damping,
            "phase_rad": math.atan2(best[2], best[1])}


def analyze(gauge: Mapping[str, Any], selected: Mapping[str, Any], windows: Mapping[str, Mapping[str, float]]) -> dict[str, Any]:
    if set(windows) != set(WINDOWS):
        raise S6RuntimeV6Error("four frozen windows required")
    ranges = {name: {"start_s": float(windows[name]["start_s"]), "end_s": float(windows[name]["end_s"])} for name in WINDOWS}
    if any(row["end_s"] < row["start_s"] for row in ranges.values()):
        raise S6RuntimeV6Error("window reversed")
    if ranges["first15"]["start_s"] != ranges["full_motion"]["start_s"] or ranges["first15"]["end_s"] > min(ranges["full_motion"]["end_s"], ranges["first15"]["start_s"] + 15) + 1e-9:
        raise S6RuntimeV6Error("first15 topology differs")
    if ranges["recorded_tail"]["start_s"] < ranges["full_motion"]["end_s"] or ranges["solver_tail"]["start_s"] < ranges["recorded_tail"]["end_s"]:
        raise S6RuntimeV6Error("tail windows overlap")
    secondary_rows = selected["rows"]
    overlap = selected["time_alignment"]
    solver_rows, comparison_rows = [], []
    for source in gauge["rows"]:
        time_s = source["time_s"]
        inside = overlap["overlap_start_s"] - 1e-12 <= time_s <= overlap["overlap_end_s"] + 1e-12
        row = dict(source)
        for name in SECONDARIES:
            row[f"{name}_m"] = _interpolate(secondary_rows, f"{name}_m", time_s) if inside else None
            row[f"{name}_coverage"] = "IN_REGISTERED_OVERLAP" if inside else "NA_OUTSIDE_REGISTERED_OVERLAP"
        solver_rows.append(row)
        if inside:
            comparison_rows.append({key: row[key] for key in ("time_s", "H_crest_m", "H_abs_m", "H_peak_to_peak_m", "H_proxy_m", "H_modal_m")})
    if len(comparison_rows) < 4:
        raise S6RuntimeV6Error("comparison overlap too short")
    window_statistics = {}
    for name, limits in ranges.items():
        subset = [row for row in solver_rows if limits["start_s"] - 1e-12 <= row["time_s"] <= limits["end_s"] + 1e-12]
        if not subset:
            raise S6RuntimeV6Error("window empty")
        window_statistics[name] = {surface: _window_stats(subset, f"{surface}_m") for surface in SURFACES}
    times = [row["time_s"] for row in comparison_rows]
    values = {name: [float(row[f"{name}_m"]) for row in comparison_rows] for name in (*SURFACES, *SECONDARIES)}
    series_metrics = {name: _spectral(times, series) for name, series in values.items()}
    comparisons = []
    for surface in SURFACES:
        for secondary in SECONDARIES:
            x, y = values[surface], values[secondary]
            xm, ym = sum(x) / len(x), sum(y) / len(y)
            covariance = sum((a - xm) * (b - ym) for a, b in zip(x, y))
            xvar, yvar = sum((a - xm) ** 2 for a in x), sum((b - ym) ** 2 for b in y)
            corr = covariance / math.sqrt(xvar * yvar) if xvar and yvar else 0.0
            comparisons.append({"surface": surface, "secondary": secondary,
                                "amplitude_error_m": series_metrics[secondary]["amplitude"] - series_metrics[surface]["amplitude"],
                                "frequency_error_hz": series_metrics[secondary]["frequency_hz"] - series_metrics[surface]["frequency_hz"],
                                "damping_error_per_s": series_metrics[secondary]["damping_per_s"] - series_metrics[surface]["damping_per_s"],
                                "phase_error_rad": (series_metrics[secondary]["phase_rad"] - series_metrics[surface]["phase_rad"] + math.pi) % (2 * math.pi) - math.pi,
                                "correlation": max(-1.0, min(1.0, corr)),
                                "rmse_m": math.sqrt(sum((a - b) ** 2 for a, b in zip(x, y)) / len(x)),
                                "ranking_claimed": False})
    result = {"schema_version": "smpcc-r8-liquid-s6-real-analysis-v6", "status": "S6_PRIMARY_ANALYSIS_COMPLETE_V6",
              "attempt_id": ATTEMPT, "planned_denominator": 1,
              "provenance": {"gauge_time_grid_sha256": gauge["time_grid_sha256"],
                             "source_bag_sha256": selected["source_bag_sha256"], "source_schema": "SPMPC_NON_FIXED",
                             "optional_unread": True},
              "grids": {"solver": {"count": len(solver_rows), "sha256": sha256_json([row["time_s"] for row in solver_rows])},
                        "comparison": {"count": len(comparison_rows), "sha256": sha256_json(times)},
                        "interpolation": "LINEAR_WITHIN_REGISTERED_OVERLAP_ONLY", "outside_overlap": "EXPLICIT_NULL_NA",
                        "smoothing": False, "extrapolation": False},
              "windows": ranges, "probe_qc": gauge["per_probe"], "solver_rows": solver_rows,
              "comparison_rows": comparison_rows, "window_statistics": window_statistics,
              "series_metrics": series_metrics, "comparisons": comparisons,
              "figure_contract": {"layout": "THREE_VERTICAL_SHARED_X_PANELS", "shared_x": True,
                                  "dual_y_axes": False, "physical_reference_label": "PENDING"},
              "claims": {"development_only": True, "single_row": True, "stage6_pass": False,
                         "paired_ranking": False, "cross_method_ranking": False,
                         "selected_trajectory_cpu_comparison": False, "physical_reference_pending": True,
                         "physical_fidelity_validated": False, "formal": False, "production": False}}
    canonical_json(result)
    return result


def render_three_panel(analysis: Mapping[str, Any]) -> dict[str, Any]:
    """Render color PNG, grayscale PNG and PDF entirely in memory."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    rows = analysis["solver_rows"]
    times = [row["time_s"] for row in rows]
    colors = {"H_crest": "#0072B2", "H_abs": "#E69F00", "H_peak_to_peak": "#009E73",
              "H_proxy": "#56B4E9", "H_modal": "#CC79A7"}
    styles = {"H_crest": "-", "H_abs": "--", "H_peak_to_peak": ":", "H_proxy": "-.", "H_modal": (0, (5, 2))}
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8, "axes.labelsize": 8,
                         "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
                         "pdf.fonttype": 42, "axes.unicode_minus": False})

    def draw(grayscale: bool):
        figure, axes = plt.subplots(3, 1, figsize=(7.2, 6.6), sharex=True, constrained_layout=True)
        for name in SURFACES:
            axes[0].plot(times, [row[f"{name}_m"] * 1000 for row in rows], label=name,
                         color="black" if grayscale else colors[name], linestyle=styles[name], linewidth=1.4)
        for name in SECONDARIES:
            axes[1].plot(times, [math.nan if row[f"{name}_m"] is None else row[f"{name}_m"] * 1000 for row in rows],
                         label=name, color="black" if grayscale else colors[name], linestyle=styles[name], linewidth=1.4)
            axes[2].plot(times, [math.nan if row[f"{name}_m"] is None else (row["H_crest_m"] - row[f"{name}_m"]) * 1000 for row in rows],
                         label=f"H_crest - {name}", color="black" if grayscale else colors[name], linestyle=styles[name], linewidth=1.4)
        for index, axis in enumerate(axes):
            axis.set_ylabel("Height (mm)" if index < 2 else "Residual (mm)")
            axis.grid(True, linewidth=.45, alpha=.65); axis.legend(frameon=False)
            axis.spines[["top", "right"]].set_visible(False)
        axes[-1].set_xlabel("Time since /odom.header.stamp origin (s)")
        axes[0].set_title("Primary R7 liquid/model comparison — physical reference PENDING")
        return figure, axes

    artifacts: dict[str, bytes] = {}
    color, axes = draw(False)
    color.canvas.draw()
    shared = all(axes[0].get_shared_x_axes().joined(axes[0], axis) for axis in axes[1:])
    stream = io.BytesIO(); color.savefig(stream, format="png", dpi=300); artifacts["figures/primary_shared_x_timeseries.png"] = stream.getvalue()
    stream = io.BytesIO(); color.savefig(stream, format="pdf"); artifacts["figures/primary_shared_x_timeseries.pdf"] = stream.getvalue()
    plt.close(color)
    gray, _ = draw(True); gray.canvas.draw(); stream = io.BytesIO(); gray.savefig(stream, format="png", dpi=300); artifacts["figures/primary_shared_x_timeseries_grayscale.png"] = stream.getvalue(); plt.close(gray)
    dimensions = []
    for name in ("figures/primary_shared_x_timeseries.png", "figures/primary_shared_x_timeseries_grayscale.png"):
        with Image.open(io.BytesIO(artifacts[name])) as image:
            image.verify()
        with Image.open(io.BytesIO(artifacts[name])) as image:
            dimensions.append(image.size)
    if not shared or len(set(dimensions)) != 1 or not artifacts["figures/primary_shared_x_timeseries.pdf"].startswith(b"%PDF"):
        raise S6RuntimeV6Error("figure render/parse QA failed")
    return {"artifacts": artifacts, "qa": {"pass": True, "three_shared_x_panels": True,
            "dual_y_axes_absent": True, "color_png_parse": True, "grayscale_png_parse": True,
            "pdf_parse": True, "dimensions": list(dimensions[0])}}


def render_particle_frames(admission: Mapping[str, Any], gauge: Mapping[str, Any]) -> dict[str, Any]:
    """Render only BI4 bytes named and hash-bound by the finalized frame manifest."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    manifest, payloads = admission["frames"], admission["payloads"]
    validate_frame_manifest(manifest)
    frame_bytes, rows = [], []
    probe_overlay_sha = sha256_json({"probe_names": list(PROBES), "attachment_frame": "MOVING_CONTAINER_REFERENCE_REF_0"})
    class_counts: Mapping[str, int] | None = None
    for source in manifest["frames"]:
        raw = payloads.get(source["relative_path"])
        if raw is None or sha256_bytes(raw) != source["sha256"]:
            raise S6RuntimeV6Error("BI4 frame bytes differ from finalized manifest")
        try:
            root = bi4.parse_jpartdata_bi4(raw)
            particles = bi4.extract_u3_particles(root)
        except bi4.Bi4FormatError as exc:
            raise S6RuntimeV6Error("invalid finalized BI4 frame") from exc
        if particles["particle_count"] != source["particle_count"] or particles["counts"] != source["class_counts"]:
            raise S6RuntimeV6Error("BI4 particle/class counts differ")
        if class_counts is None:
            class_counts = particles["counts"]
        elif particles["counts"] != class_counts:
            raise S6RuntimeV6Error("BI4 class counts drift")
        positions, classes = particles["positions_m"], particles["classes"]
        figure, axes = plt.subplots(1, 2, figsize=(8, 4), constrained_layout=True)
        palette = {"fixed_boundary": "#666666", "moving_boundary": "#E69F00", "floating": "#CC79A7", "fluid": "#0072B2"}
        for class_name in ("fixed_boundary", "moving_boundary", "floating", "fluid"):
            selected = [position for position, label in zip(positions, classes) if label == class_name]
            if selected:
                axes[0].scatter([row[0] for row in selected], [row[2] for row in selected], s=2, c=palette[class_name], label=class_name)
                axes[1].scatter([row[0] for row in selected], [row[1] for row in selected], s=2, c=palette[class_name])
        axes[0].set(xlabel="container x (m)", ylabel="z (m)", title=f"t={source['time_s']:.3f} s")
        axes[1].set(xlabel="container x (m)", ylabel="container y (m)", title="MOVING_CONTAINER_REFERENCE_REF_0")
        axes[0].legend(frameon=False, fontsize=6)
        for axis in axes:
            axis.set_aspect("equal", adjustable="datalim"); axis.grid(True, linewidth=.3, alpha=.5)
        stream = io.BytesIO(); figure.savefig(stream, format="png", dpi=120); plt.close(figure)
        png = stream.getvalue(); frame_bytes.append(png)
        rows.append({"index": source["index"], "time_s": source["time_s"], "source_bi4_sha256": source["sha256"],
                     "rendered_png_sha256": sha256_bytes(png), "class_counts_sha256": sha256_json(source["class_counts"]),
                     "probe_overlay_sha256": probe_overlay_sha, "container_frame": "MOVING_CONTAINER_REFERENCE_REF_0"})
    return {"frames": frame_bytes, "manifest_rows": rows,
            "frame_manifest_sha256": sha256_json(manifest), "numeric_fact_source": False}


def encode_media(rendered: Mapping[str, Any], *, fps: int) -> dict[str, Any]:
    """Encode MP4/GIF/keyframes in memory-backed temporary files and fully decode."""
    import tempfile
    import cv2
    import numpy as np
    from PIL import Image

    raw_frames = rendered["frames"]
    if len(raw_frames) < 3 or not 1 <= fps <= 30:
        raise S6RuntimeV6Error("media frame/fps contract differs")
    arrays, images = [], []
    size = None
    for raw in raw_frames:
        with Image.open(io.BytesIO(raw)) as image:
            rgb = image.convert("RGB")
            size = size or rgb.size
            if rgb.size != size:
                raise S6RuntimeV6Error("rendered frame dimensions differ")
            images.append(rgb.copy()); arrays.append(cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR))
    with tempfile.TemporaryDirectory(prefix="r8-s6-v6-media-") as temporary:
        mp4_path, gif_path = Path(temporary) / "primary.mp4", Path(temporary) / "primary.gif"
        writer = cv2.VideoWriter(str(mp4_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not writer.isOpened():
            raise S6RuntimeV6Error("MP4 encoder did not open")
        for array in arrays:
            writer.write(array)
        writer.release()
        images[0].save(gif_path, save_all=True, append_images=images[1:], duration=round(1000 / fps), loop=0, disposal=2, optimize=False)
        capture = cv2.VideoCapture(str(mp4_path)); decoded_mp4 = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if (frame.shape[1], frame.shape[0]) != size:
                capture.release(); raise S6RuntimeV6Error("decoded MP4 dimensions differ")
            decoded_mp4 += 1
        capture.release()
        with Image.open(gif_path) as gif:
            decoded_gif = int(getattr(gif, "n_frames", 1))
            for index in range(decoded_gif):
                gif.seek(index); gif.load()
            if gif.size != size:
                raise S6RuntimeV6Error("decoded GIF dimensions differ")
        mp4_raw, gif_raw = mp4_path.read_bytes(), gif_path.read_bytes()
    if decoded_mp4 != len(images) or decoded_gif != len(images):
        raise S6RuntimeV6Error("complete media decode count differs")
    indices = [0, len(images) // 2, len(images) - 1]
    keyframes = {}
    for label, index in zip(("first", "middle", "last"), indices):
        stream = io.BytesIO(); images[index].save(stream, format="PNG")
        keyframes[f"keyframes/primary_{label}.png"] = stream.getvalue()
    artifacts = {"animation/primary.mp4": mp4_raw, "animation/primary_preview.gif": gif_raw, **keyframes}
    manifest = {"schema_version": "smpcc-r8-liquid-s6-media-manifest-v6", "attempt_id": ATTEMPT,
                "frame_manifest_sha256": rendered["frame_manifest_sha256"], "frames": rendered["manifest_rows"],
                "media": {name: {"sha256": sha256_bytes(raw), "size_bytes": len(raw)} for name, raw in artifacts.items()},
                "decode_qa": {"mp4_full_decode": True, "gif_full_decode": True,
                              "decoded_mp4_frames": decoded_mp4, "decoded_gif_frames": decoded_gif},
                "keyframe_indices": indices, "numeric_fact_source": False}
    return {"artifacts": artifacts, "manifest": manifest}


def _aligned_csv(analysis: Mapping[str, Any]) -> bytes:
    output = io.StringIO(newline="")
    fields = ("time_s", "H_crest_m", "H_abs_m", "H_peak_to_peak_m", "H_proxy_m", "H_modal_m")
    writer = csv.DictWriter(output, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    for source in analysis["solver_rows"]:
        writer.writerow({name: "NA" if source.get(name) is None else source[name] for name in fields})
    return output.getvalue().encode("utf-8")


def build_artifacts(analysis: Mapping[str, Any], selected: Mapping[str, Any], gauge: Mapping[str, Any],
                    figures: Mapping[str, Any], media: Mapping[str, Any], *, previous_ledger_sha256: str) -> tuple[dict[str, bytes], dict[str, Any]]:
    policy, _ = load_contracts()
    if SHA_RE.fullmatch(previous_ledger_sha256) is None:
        raise S6RuntimeV6Error("ledger predecessor invalid")
    base = {"data/raw_gauge_zsurf.json": canonical_json(gauge),
            "data/aligned_primary_comparison.csv": _aligned_csv(analysis),
            "reports/analysis_result.json": canonical_json(analysis),
            "reports/selected_signal_provenance.json": canonical_json(selected),
            "reports/media_manifest.json": canonical_json(media["manifest"]),
            **figures["artifacts"], **media["artifacts"]}
    comparison = {"schema_version": "smpcc-r8-liquid-s6-comparison-manifest-v6", "attempt_id": ATTEMPT,
                  "planned_denominator": 1, "status": FINAL, "analysis_sha256": sha256_json(analysis),
                  "selected_signal_sha256": sha256_json(selected), "gauge_sha256": sha256_json(gauge),
                  "media_manifest_sha256": sha256_json(media["manifest"]), "paired_ranking": False,
                  "cross_method_ranking": False, "selected_trajectory_cpu_comparison": False,
                  "physical_reference_pending": True, "physical_fidelity_validated": False}
    base["comparison_manifest.json"] = canonical_json(comparison)
    ledger_payload = {"schema_version": "smpcc-r8-liquid-s6-secondary-ledger-entry-v6", "attempt_id": ATTEMPT,
                      "status": FINAL, "comparison_manifest_sha256": sha256_json(comparison),
                      "planned_denominator": 1, "physical_reference_pending": True, "stage6_pass": True}
    ledger_entry = {"entry_sha256": sha256_json(ledger_payload), "previous_entry_sha256": previous_ledger_sha256,
                    "payload": ledger_payload}
    base["liquid_secondary_ledger.jsonl"] = canonical_json(ledger_entry)
    base["secondary_ledger_append_receipt.json"] = canonical_json({"schema_version": "smpcc-r8-liquid-s6-ledger-append-receipt-v6",
        "status": "PLANNED_COMPARE_APPEND_NOT_YET_PERFORMED", "append_performed": False,
        "entry_sha256": ledger_entry["entry_sha256"], "previous_entry_sha256": previous_ledger_sha256})
    base["acceptance_receipt.json"] = canonical_json({"schema_version": "smpcc-r8-liquid-s6-acceptance-receipt-v6",
        "status": "PENDING_ATOMIC_PUBLISH_AND_EXTERNAL_LEDGER_APPEND", "attempt_id": ATTEMPT,
        "planned_denominator": 1, "stage6_pass": False, "physical_reference_pending": True})
    evidence = {"schema_version": "smpcc-r8-liquid-s6-evidence-index-v6", "attempt_id": ATTEMPT,
                "entries": [{"relative_path": name, "sha256": sha256_bytes(payload), "size_bytes": len(payload)}
                            for name, payload in sorted(base.items())], "physical_reference_pending": True}
    base["evidence_index.json"] = canonical_json(evidence)
    required = policy["delivery"]["required_inventory"]
    if set(base) != set(required) - {"checksums.sha256"}:
        raise S6RuntimeV6Error("delivery inventory before checksum differs")
    base["checksums.sha256"] = "".join(f"{sha256_bytes(base[name])}  {name}\n" for name in sorted(base)).encode("ascii")
    return base, ledger_entry


def _rename_noreplace(source: Path, destination: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    result = libc.syscall(SYS_RENAMEAT2, AT_FDCWD, os.fsencode(source), AT_FDCWD,
                          os.fsencode(destination), RENAME_NOREPLACE)
    if result != 0:
        number = ctypes.get_errno()
        raise OSError(number, os.strerror(number), str(destination))


def atomic_publish(final_root: Path, artifacts: Mapping[str, bytes]) -> dict[str, Any]:
    policy, _ = load_contracts()
    required = policy["delivery"]["required_inventory"]
    if set(artifacts) != set(required):
        raise S6RuntimeV6Error("atomic publication inventory differs")
    final = Path(final_root)
    _assert_path_components(final, final_may_be_absent=True)
    partial = final.with_name(final.name + ".partial")
    if final.exists() or partial.exists():
        raise S6RuntimeV6Error("publication root is not create-new")
    parent_fd = os.open(final.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        os.mkdir(partial.name, mode=0o700, dir_fd=parent_fd)
        partial_fd = os.open(partial.name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                             dir_fd=parent_fd)
        try:
            directories = sorted({str(PurePosixPath(name).parent) for name in required if str(PurePosixPath(name).parent) != "."})
            for directory in directories:
                cursor = partial_fd
                owned: list[int] = []
                try:
                    for part in PurePosixPath(directory).parts:
                        try:
                            os.mkdir(part, mode=0o700, dir_fd=cursor)
                        except FileExistsError:
                            pass
                        child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=cursor)
                        owned.append(child); cursor = child
                finally:
                    for fd in reversed(owned): os.close(fd)
            for relative in sorted(required):
                parts = PurePosixPath(relative).parts
                cursor = os.dup(partial_fd)
                try:
                    for part in parts[:-1]:
                        child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=cursor)
                        os.close(cursor); cursor = child
                    fd = os.open(parts[-1], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
                                 0o440, dir_fd=cursor)
                    try:
                        view = memoryview(artifacts[relative])
                        while view:
                            written = os.write(fd, view)
                            if written <= 0: raise S6RuntimeV6Error("short publication write")
                            view = view[written:]
                        os.fsync(fd)
                    finally:
                        os.close(fd)
                finally:
                    os.close(cursor)
            os.fsync(partial_fd)
        finally:
            os.close(partial_fd)
        _rename_noreplace(partial, final)
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)
    identities = {name: {"sha256": sha256_bytes(artifacts[name]), "size_bytes": len(artifacts[name])} for name in required}
    return {"root": str(final), "inventory": identities, "inventory_sha256": sha256_json(identities),
            "file_count": len(identities), "atomic_noreplace": True}


def append_external_ledger(path: Path, entry: Mapping[str, Any], *, expected_previous_sha256: str) -> dict[str, Any]:
    if set(entry) != {"entry_sha256", "previous_entry_sha256", "payload"} or entry["previous_entry_sha256"] != expected_previous_sha256 or entry["entry_sha256"] != sha256_json(entry["payload"]):
        raise S6RuntimeV6Error("planned ledger entry differs")
    path = Path(path)
    _assert_path_components(path, final_may_be_absent=True)
    if path.exists() and path.is_symlink():
        raise S6RuntimeV6Error("ledger path is symlink")
    fd = os.open(path, os.O_RDWR | os.O_APPEND | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise S6RuntimeV6Error("ledger is not regular single-link")
        os.lseek(fd, 0, os.SEEK_SET); existing = _read_fd_all(fd, MAX_FILE_BYTES)
        previous, attempts = "0" * 64, set()
        for raw in existing.splitlines():
            observed = _json_bytes(raw, "ledger line")
            if set(observed) != {"entry_sha256", "previous_entry_sha256", "payload"} or observed["previous_entry_sha256"] != previous or observed["entry_sha256"] != sha256_json(observed["payload"]):
                raise S6RuntimeV6Error("external ledger chain invalid")
            attempt = observed["payload"].get("attempt_id")
            if attempt in attempts: raise S6RuntimeV6Error("external ledger duplicate attempt")
            attempts.add(attempt); previous = observed["entry_sha256"]
        if previous != expected_previous_sha256 or ATTEMPT in attempts:
            raise S6RuntimeV6Error("external ledger compare-append failed")
        encoded = canonical_json(entry)
        if os.write(fd, encoded) != len(encoded):
            raise S6RuntimeV6Error("short ledger append")
        os.fsync(fd); after = os.fstat(fd)
        if (before.st_dev, before.st_ino) != (after.st_dev, after.st_ino) or after.st_size != before.st_size + len(encoded):
            raise S6RuntimeV6Error("ledger append identity/size differs")
    finally:
        try: fcntl.flock(fd, fcntl.LOCK_UN)
        finally: os.close(fd)
    parent_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try: os.fsync(parent_fd)
    finally: os.close(parent_fd)
    return {"append_performed": True, "entry_sha256": entry["entry_sha256"],
            "previous_entry_sha256": expected_previous_sha256}


def run(contract: Mapping[str, Any]) -> dict[str, Any]:
    """One-shot read/analyze/render/publish/compare-append orchestration."""
    _, schemas = load_contracts()
    Draft202012Validator(schemas["runtime"]).validate(dict(contract))
    contract_sha = sha256_json(contract)
    admission = admit_finalized_s5b0(contract)
    selected = extract_selected_signals(contract)
    gauge = read_native_gauges(admission)
    analysis = analyze(gauge, selected, contract["runtime"]["windows"])
    figures = render_three_panel(analysis)
    rendered = render_particle_frames(admission, gauge)
    media = encode_media(rendered, fps=contract["runtime"]["fps"])
    artifacts, ledger_entry = build_artifacts(analysis, selected, gauge, figures, media,
        previous_ledger_sha256=contract["runtime"]["external_ledger_expected_previous_sha256"])
    publication = atomic_publish(Path(contract["runtime"]["final_root"]), artifacts)
    ledger = append_external_ledger(Path(contract["runtime"]["external_ledger_path"]), ledger_entry,
        expected_previous_sha256=contract["runtime"]["external_ledger_expected_previous_sha256"])
    result = {"schema_version": "smpcc-r8-liquid-s6-real-runtime-result-v6",
              "document_type": "SMPCC_R8_LIQUID_S6_REAL_RUNTIME_RESULT_V6", "status": FINAL,
              "attempt_id": ATTEMPT, "planned_denominator": 1, "runtime_contract_sha256": contract_sha,
              "publication": {"root": publication["root"], "inventory_sha256": publication["inventory_sha256"],
                              "file_count": publication["file_count"], "atomic_noreplace": True},
              "ledger": ledger,
              "checks": {"exact_parent": True, "primary_bag_read": True, "sixteen_gauge_csv": True,
                         "finalized_bi4_frames": True, "analysis_complete": True, "figure_complete": True,
                         "media_complete_decode": True, "inventory_complete": True, "optional_unread": True},
              "claims": {"stage6_pass": True, "development_only": True, "physical_reference_pending": True,
                         "physical_fidelity_validated": False, "paired_ranking": False, "cross_method_ranking": False,
                         "selected_trajectory_cpu_comparison": False, "formal": False, "production": False,
                         "physical_primary": False}}
    Draft202012Validator(schemas["result"]).validate(result)
    return result


def static_receipt() -> dict[str, Any]:
    policy, schemas = load_contracts()
    value = {"schema_version": "smpcc-r8-liquid-s6-real-runtime-result-v6",
             "document_type": "SMPCC_R8_LIQUID_S6_REAL_RUNTIME_RESULT_V6", "status": NOT_ADMITTED,
             "attempt_id": ATTEMPT, "planned_denominator": 1,
             "runtime_contract_sha256": sha256_bytes(canonical_json(policy)),
             "publication": {"root": None, "inventory_sha256": None, "file_count": 0, "atomic_noreplace": False},
             "ledger": {"append_performed": False, "entry_sha256": None, "previous_entry_sha256": None},
             "checks": {"exact_parent": False, "primary_bag_read": False, "sixteen_gauge_csv": False,
                        "finalized_bi4_frames": False, "analysis_complete": False, "figure_complete": False,
                        "media_complete_decode": False, "inventory_complete": False, "optional_unread": True},
             "claims": {"stage6_pass": False, "development_only": True, "physical_reference_pending": True,
                        "physical_fidelity_validated": False, "paired_ranking": False, "cross_method_ranking": False,
                        "selected_trajectory_cpu_comparison": False, "formal": False, "production": False,
                        "physical_primary": False}}
    Draft202012Validator(schemas["result"]).validate(value)
    return value


def self_check() -> dict[str, Any]:
    policy, schemas = load_contracts()
    receipt = static_receipt()
    return {"status": "S6_REAL_RUNTIME_DELIVERY_V6_SELF_CHECK_OK_NOT_ADMITTED",
            "receipt_status": receipt["status"], "schemas_deep_closed": len(schemas) == 3,
            "probe_count": len(policy["gauge"]["probe_names"]),
            "required_inventory_count": len(policy["delivery"]["required_inventory"]),
            "real_parent_read": False, "real_bag_read": False, "optional_bag_read": False,
            "external_write_performed": False, "media_executed": False, "solver_executed": False,
            "gpu_exposed": False, "network_used": False, "sudo_used": False, "stage6_pass": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True, separators=(",", ":")))
        return 0
    except (S6RuntimeV6Error, ValidationError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "S6_REAL_RUNTIME_DELIVERY_V6_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["S6RuntimeV6Error", "admit_finalized_s5b0", "analyze", "append_external_ledger",
           "assert_deep_closed", "atomic_publish", "build_artifacts", "canonical_json", "encode_media",
           "extract_selected_signals", "load_contracts", "read_exact_file", "read_native_gauges",
           "render_particle_frames", "render_three_panel", "run", "self_check", "sha256_bytes",
           "sha256_json", "static_receipt", "validate_frame_manifest", "validate_gauge_manifest"]
