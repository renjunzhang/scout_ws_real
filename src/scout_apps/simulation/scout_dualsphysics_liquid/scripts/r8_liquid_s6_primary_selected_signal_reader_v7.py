#!/usr/bin/env python3
"""Read only the exact primary bag and emit closed H_proxy/H_modal evidence.

The public CLI is self-check only.  ``read_primary`` is the future read-only
library surface: callers must pass the exact three frozen file identities.
There is intentionally no optional-bag argument or path discovery.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s6_real_selected_signal_extractor_v5 as extractor  # noqa: E402


SCHEMA_PATH = ROOT / "schema/target_host_s6_primary_selected_signals_v7.json"
ATTEMPT_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
S5A0_SHA256 = "709ccd9e2e5da97a4d7d71291c566d9c90d8e4f0499ef35c838f55cca0b8cda1"
S5A1_SHA256 = "8e5b9417e9485f0f178849f274dfd605138d4a33af2abe346a3628bd38771c18"
TRANSFER_SHA256 = "8c653953fb121961ba206508fed992c1dee77148b6cd4302052d3bc48fa86cf6"
SOURCE_BAG_SHA256 = "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07"
EXTRACTOR_SHA256 = "991bdc10f89b5056978ead95acb89fb904ffbe199a0c6769cafa64855e327373"
MAX_JSON_BYTES = 16 * 1024 * 1024


class SelectedSignalV7Error(ValueError):
    """A frozen identity, provenance, reader, numeric, or schema gate failed."""


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if (kind == "object" or isinstance(kind, list) and "object" in kind) and value.get("additionalProperties") is not False:
            raise SelectedSignalV7Error(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def _identity(path: Path, expected: Mapping[str, Any], *, maximum: int) -> bytes:
    required = {"path", "sha256", "size_bytes", "mode", "device", "inode", "nlink", "mtime_ns", "ctime_ns"}
    if set(expected) != required or expected["path"] != str(path):
        raise SelectedSignalV7Error("file identity is open or path differs")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or before.st_size > maximum:
            raise SelectedSignalV7Error("file is not bounded regular single-link")
        blocks: list[bytes] = []
        digest = hashlib.sha256()
        total = 0
        while total < before.st_size:
            block = os.read(descriptor, min(1 << 20, before.st_size - total))
            if not block:
                raise SelectedSignalV7Error("short file read")
            blocks.append(block); digest.update(block); total += len(block)
        if os.read(descriptor, 1):
            raise SelectedSignalV7Error("file grew while read")
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    observed = {
        "path": str(path), "sha256": digest.hexdigest(), "size_bytes": total,
        "mode": f"{stat.S_IMODE(after.st_mode):04o}", "device": after.st_dev,
        "inode": after.st_ino, "nlink": after.st_nlink,
        "mtime_ns": after.st_mtime_ns, "ctime_ns": after.st_ctime_ns,
    }
    if observed != dict(expected) or (
        before.st_dev, before.st_ino, before.st_mode, before.st_nlink,
        before.st_size, before.st_mtime_ns, before.st_ctime_ns,
    ) != (
        after.st_dev, after.st_ino, after.st_mode, after.st_nlink,
        after.st_size, after.st_mtime_ns, after.st_ctime_ns,
    ):
        raise SelectedSignalV7Error("file identity/hash/TOCTOU differs")
    return b"".join(blocks)


def _json(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(raw, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    except (UnicodeError, ValueError) as exc:
        raise SelectedSignalV7Error(f"invalid JSON: {label}") from exc
    if not isinstance(value, dict):
        raise SelectedSignalV7Error(f"JSON root is not object: {label}")
    return value


def _tool(relative_path: str, expected_sha256: str) -> dict[str, str]:
    path = ROOT / relative_path
    if sha256_bytes(path.read_bytes()) != expected_sha256:
        raise SelectedSignalV7Error(f"reader dependency drift: {path.name}")
    return {"relative_path": relative_path, "sha256": expected_sha256}


def validate_result(value: Mapping[str, Any]) -> dict[str, Any]:
    schema = json.loads(SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    Draft202012Validator(schema).validate(dict(value))
    if value["integrity"]["source_bag_sha256"] != value["parents"]["source_bag"]["sha256"]:
        raise SelectedSignalV7Error("source bag digest binding differs")
    offset = value["time_alignment"]["record_to_odom_header_offset_ns"]
    starts: list[float] = []
    ends: list[float] = []
    for name, scale in (("H_proxy", 1000.0), ("H_modal", 1.0)):
        series = value["series"][name]
        samples = series["samples"]
        if series["sample_count"] != len(samples) or sha256_bytes(canonical_json(samples)) != value["integrity"][f"{name}_samples_sha256"]:
            raise SelectedSignalV7Error(f"{name} count/hash binding differs")
        previous = -1
        for row in samples:
            if row["bag_record_t_ns"] <= previous or row["mapped_odom_header_t_ns"] != row["bag_record_t_ns"] + offset:
                raise SelectedSignalV7Error(f"{name} time binding differs")
            if abs(row["value_comparison_mm"] - row["value_native"] * scale) > 1e-9:
                raise SelectedSignalV7Error(f"{name} unit conversion differs")
            previous = row["bag_record_t_ns"]
        starts.append(samples[0]["time_since_odom_origin_s"])
        ends.append(samples[-1]["time_since_odom_origin_s"])
    alignment = value["time_alignment"]
    if alignment["overlap_start_s"] != max(starts) or alignment["overlap_end_s"] != min(ends):
        raise SelectedSignalV7Error("overlap binding differs")
    return dict(value)


def read_primary(*, s5a0_identity: Mapping[str, Any], transfer_identity: Mapping[str, Any],
                 bag_identity: Mapping[str, Any]) -> dict[str, Any]:
    s5a0_path = Path(str(s5a0_identity.get("path", "")))
    transfer_path = Path(str(transfer_identity.get("path", "")))
    bag_path = Path(str(bag_identity.get("path", "")))
    s5a0_raw = _identity(s5a0_path, s5a0_identity, maximum=MAX_JSON_BYTES)
    transfer_raw = _identity(transfer_path, transfer_identity, maximum=MAX_JSON_BYTES)
    bag_raw = _identity(bag_path, bag_identity, maximum=extractor.MAXIMUM_SOURCE_BYTES)
    if sha256_bytes(s5a0_raw) != S5A0_SHA256 or sha256_bytes(transfer_raw) != TRANSFER_SHA256 or sha256_bytes(bag_raw) != SOURCE_BAG_SHA256:
        raise SelectedSignalV7Error("frozen primary parent SHA-256 differs")
    s5a0 = _json(s5a0_raw, "S5A0 receipt")
    transfer = _json(transfer_raw, "S5A1 transfer")
    if s5a0.get("status") != "PASS_S5A0_PRIMARY_ONE_SHOT_DEVELOPMENT_ONLY" or s5a0.get("selection", {}).get("attempt_id") != ATTEMPT_ID or s5a0.get("claims", {}).get("source_outcome") != "UNKNOWN":
        raise SelectedSignalV7Error("S5A0 status/attempt/source outcome differs")
    source = transfer.get("source", {})
    if (transfer.get("status") != "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
            or transfer.get("transfer_id") != f"{ATTEMPT_ID}_r8_liquid_handoff_v3_v10"
            or source.get("attempt_id") != ATTEMPT_ID or source.get("source_outcome") != "UNKNOWN"
            or source.get("bag_sha256") != SOURCE_BAG_SHA256):
        raise SelectedSignalV7Error("S5A1 source binding differs")
    captured = extractor._capture_bag_bytes(bag_raw)
    if captured.get("reader_anomalies") != []:
        raise SelectedSignalV7Error("primary bag reader anomaly")
    alignment = extractor._alignment(captured["odom"])
    proxy = extractor._map_series(captured["signals"]["H_proxy"], "H_proxy", alignment, 1000.0)
    modal = extractor._map_series(captured["signals"]["H_modal"], "H_modal", alignment, 1.0)
    overlap_start = max(proxy[0]["time_since_odom_origin_s"], modal[0]["time_since_odom_origin_s"])
    overlap_end = min(proxy[-1]["time_since_odom_origin_s"], modal[-1]["time_since_odom_origin_s"])
    if overlap_end <= overlap_start:
        raise SelectedSignalV7Error("selected signals have no common overlap")
    result = {
        "schema_version": "smpcc-r8-liquid-s6-primary-selected-signals-v7",
        "document_type": "SMPCC_R8_LIQUID_S6_PRIMARY_SELECTED_SIGNALS_V7",
        "status": "PASS_S6_PRIMARY_SELECTED_SIGNALS_V7_READ_ONLY",
        "attempt_id": ATTEMPT_ID, "planned_denominator": 1, "source_outcome": "UNKNOWN",
        "parents": {"s5a0_selected_bag_receipt": dict(s5a0_identity),
                    "s5a1_transfer_manifest": dict(transfer_identity), "source_bag": dict(bag_identity)},
        "reader_contract": {
            "extractor": _tool("scripts/r8_liquid_s6_real_selected_signal_extractor_v5.py", EXTRACTOR_SHA256),
            "reader_core": _tool("scripts/r8_liquid_ros1_bag_v2_reader_v1.py", extractor.READER_CORE_SHA256),
            "reader_v4": _tool("scripts/r8_liquid_ros1_bag_v2_reader_v4.py", extractor.READER_V4_SHA256),
            "extractor_v3": _tool("scripts/r8_liquid_s5a1_ros1_signal_extractor_v3.py", extractor.EXTRACTOR_V3_SHA256),
            "input_surface": "IMMUTABLE_BOUNDED_EXACT_PRIMARY_BAG_BYTES_ONLY",
        },
        "time_alignment": {
            "x_axis": "time_since_odom_header_origin_s", "motion_time_source": "/odom.header.stamp",
            "signal_native_time_source": "ROS1_BAG_RECORD_TIME_NS",
            "mapping_method": "LOWER_MEDIAN_ODOM_HEADER_MINUS_RECORD_OFFSET_V1",
            "odom_header_origin_ns": alignment["origin_ns"], "odom_header_end_ns": alignment["end_ns"],
            "offset_sample_count": alignment["sample_count"],
            "record_to_odom_header_offset_ns": alignment["offset_ns"],
            "residual_mean_ns": alignment["residual_mean_ns"], "residual_rms_ns": alignment["residual_rms_ns"],
            "residual_max_abs_ns": alignment["residual_max_abs_ns"],
            "maximum_allowed_residual_ns": extractor.MAXIMUM_ALIGNMENT_RESIDUAL_NS,
            "residuals_sha256": alignment["residuals_sha256"], "mapping_extrapolation": False,
            "overlap_start_s": overlap_start, "overlap_end_s": overlap_end,
        },
        "series": {
            "H_proxy": {"topic": "/slosh/height", "message_type": "std_msgs/Float32", "native_unit": "m",
                        "comparison_unit": "mm", "scale_to_comparison": 1000.0, "offset_to_comparison": 0.0,
                        "sample_count": len(proxy), "samples": proxy},
            "H_modal": {"topic": "/spmpc/slosh_height", "message_type": "std_msgs/Float32", "native_unit": "mm",
                        "comparison_unit": "mm", "scale_to_comparison": 1.0, "offset_to_comparison": 0.0,
                        "sample_count": len(modal), "samples": modal},
        },
        "integrity": {"source_bag_sha256": SOURCE_BAG_SHA256,
                      "H_proxy_samples_sha256": sha256_bytes(canonical_json(proxy)),
                      "H_modal_samples_sha256": sha256_bytes(canonical_json(modal)),
                      "reader_anomalies_absent": True, "parents_unchanged": True},
        "claims": {"read_only": True, "comparison_only": True, "optional_bag_read": False,
                   "source_bag_executed": False, "ros_started": False,
                   "motion_exporter_consumed_selected_signals": False,
                   "solver_forcing_consumed_selected_signals": False, "stage6_pass": False,
                   "development_only": True, "paired_ranking": False, "cross_method_ranking": False,
                   "selected_trajectory_cpu_comparison": False, "physical_reference_pending": True,
                   "physical_fidelity_validated": False, "formal": False, "production": False},
    }
    return validate_result(result)


def self_check() -> dict[str, Any]:
    schema = json.loads(SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema); assert_deep_closed(schema)
    _tool("scripts/r8_liquid_s6_real_selected_signal_extractor_v5.py", EXTRACTOR_SHA256)
    _tool("scripts/r8_liquid_ros1_bag_v2_reader_v1.py", extractor.READER_CORE_SHA256)
    _tool("scripts/r8_liquid_ros1_bag_v2_reader_v4.py", extractor.READER_V4_SHA256)
    _tool("scripts/r8_liquid_s5a1_ros1_signal_extractor_v3.py", extractor.EXTRACTOR_V3_SHA256)
    return {"status": "PASS_S6_PRIMARY_SELECTED_SIGNAL_READER_V7_STATIC_ONLY",
            "attempt_id": ATTEMPT_ID, "planned_denominator": 1, "source_outcome": "UNKNOWN",
            "real_bag_read": False, "optional_bag_read": False, "external_write": False,
            "candidate_executed": False, "solver_or_gpu_executed": False, "stage6_pass": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",), nargs="?", default="self-check")
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True, separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S6_PRIMARY_SELECTED_SIGNAL_READER_V7", "error": str(exc)},
                         ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["SelectedSignalV7Error", "read_primary", "validate_result", "self_check"]
