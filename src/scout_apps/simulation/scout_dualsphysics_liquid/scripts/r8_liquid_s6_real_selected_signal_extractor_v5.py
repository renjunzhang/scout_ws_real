#!/usr/bin/env python3
"""Fixture-only producer for the real-shaped S6 selected-signal provenance v5.

The public producer accepts immutable synthetic ROS1 Bag V2 bytes only.  It
uses the hardened reader-v4 delegation graph, captures H_proxy/H_modal as
comparison-only values, and maps bag record time onto the common
``/odom.header.stamp`` axis using an audited record/header offset.  It never
opens a bag path, reads an optional bag, starts ROS, writes an artifact, or
feeds any selected signal to motion export or solver forcing.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_ros1_signal_extractor_v3 as hardened  # noqa: E402


reader_core = hardened.reader_core
reader_v4 = hardened.reader_v4
extractor_v1 = hardened.extractor_v1
SCHEMA_PATH = ROOT / "schema/target_host_s6_real_selected_signals_provenance_v5.json"
READER_CORE_PATH = MODULE_DIR / "r8_liquid_ros1_bag_v2_reader_v1.py"
READER_V4_PATH = MODULE_DIR / "r8_liquid_ros1_bag_v2_reader_v4.py"
EXTRACTOR_V3_PATH = MODULE_DIR / "r8_liquid_s5a1_ros1_signal_extractor_v3.py"
READER_CORE_SHA256 = "bc3975ba446e22097c399e10a8f6c4e60b4f8e78c5c7d613394028daa76c94fc"
READER_V4_SHA256 = "dfb4f075504eef25753752179a2d8cf8f5585f2fbeb9260254f9b411d77f1b7b"
EXTRACTOR_V3_SHA256 = "c9671c9b138481dc413b8d26b943c66960c52a62f94adc53aab3aa021e386922"
MAXIMUM_SOURCE_BYTES = 64 * 1024 * 1024
MAXIMUM_MESSAGES_PER_TOPIC = 100_000
MAXIMUM_ALIGNMENT_RESIDUAL_NS = 5_000_000
PRIMARY_ATTEMPT = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
SIGNALS = {
    "/slosh/height": ("H_proxy", "m", 1000.0),
    "/spmpc/slosh_height": ("H_modal", "mm", 1.0),
}
QC_ONLY_TOPICS = tuple(sorted(set(hardened.QC_ONLY_TOPICS) - set(SIGNALS)))


class SelectedSignalV5Error(ValueError):
    """Reader, parent identity, time mapping, signal, or schema failure."""


def canonical_json(value: Any) -> bytes:
    try:
        encoded = json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":"))
    except (TypeError, ValueError) as exc:
        raise SelectedSignalV5Error("value is not finite canonical JSON") from exc
    return (encoded + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def _sha256_path(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _require_sha256(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)):
        raise SelectedSignalV5Error(f"{label} is not an exact lowercase SHA-256")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SelectedSignalV5Error(f"{label} is not numeric")
    number = float(value)
    if not math.isfinite(number):
        raise SelectedSignalV5Error(f"{label} is non-finite")
    return number


def _integer_ns(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SelectedSignalV5Error(f"{label} is not a non-negative integer ns value")
    return value


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise SelectedSignalV5Error(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def load_schema() -> dict[str, Any]:
    value = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SelectedSignalV5Error("schema root is not an object")
    Draft202012Validator.check_schema(value)
    assert_deep_closed(value)
    return value


def _assert_hardened_dependencies() -> None:
    expected = {
        READER_CORE_PATH: READER_CORE_SHA256,
        READER_V4_PATH: READER_V4_SHA256,
        EXTRACTOR_V3_PATH: EXTRACTOR_V3_SHA256,
    }
    for path, digest in expected.items():
        if _sha256_path(path) != digest:
            raise SelectedSignalV5Error(f"hardened dependency hash drifted: {path.name}")
    try:
        hardened._assert_reader_hook_topology()
    except hardened.SignalExtractionError as exc:
        raise SelectedSignalV5Error(str(exc)) from exc


def _capture_bag_bytes(data: bytes) -> dict[str, Any]:
    _assert_hardened_dependencies()
    record_times: dict[int, int] = {}
    odom: list[dict[str, Any]] = []
    signals: dict[str, list[dict[str, Any]]] = {name: [] for name, _, _ in SIGNALS.values()}
    qc_counts = {topic: 0 for topic in QC_ONLY_TOPICS}
    original_message = reader_core._Message
    original_decoder = reader_core._decode_registered_payload
    original_helper_reader = extractor_v1.reader

    def capture_message(*args: Any, **kwargs: Any):
        message = original_message(*args, **kwargs)
        identity = id(message.payload)
        if identity in record_times:
            raise SelectedSignalV5Error("message payload identity was reused")
        record_times[identity] = int(message.record_time_ns)
        return message

    def capture_decoder(topic: str, message_type: str, payload: memoryview, limits: Any):
        record_time = record_times.get(id(payload))
        if record_time is None:
            raise SelectedSignalV5Error("message/decoder provenance association failed")
        if topic == "/odom":
            if message_type != "nav_msgs/Odometry":
                raise SelectedSignalV5Error("/odom message type differs")
            row = extractor_v1._full_odometry(payload, limits)
            row["bag_record_t_ns"] = record_time
            odom.append(row)
        elif topic in SIGNALS:
            if message_type != "std_msgs/Float32":
                raise SelectedSignalV5Error(f"{topic} message type differs")
            cursor = reader_core._PayloadCursor(payload, limits, label=topic)
            value = float(cursor.floats(1, "data")[0])
            cursor.require_eof()
            name, _, _ = SIGNALS[topic]
            signals[name].append({"bag_record_t_ns": record_time, "value_native": value})
        elif topic in qc_counts:
            qc_counts[topic] += 1
        return original_decoder(topic, message_type, payload, limits)

    reader_core._Message = capture_message
    reader_core._decode_registered_payload = capture_decoder
    extractor_v1.reader = reader_core
    try:
        summary = reader_v4.parse_bag_v2(data, limits=reader_v4.ReaderLimits())
    except (reader_core.BagV2Error, hardened.SignalExtractionError, ValueError) as exc:
        raise SelectedSignalV5Error(str(exc)) from exc
    finally:
        reader_core._Message = original_message
        reader_core._decode_registered_payload = original_decoder
        extractor_v1.reader = original_helper_reader

    topic_map = {row["topic"]: row for row in summary["topics"]}
    for topic, (name, _, _) in SIGNALS.items():
        if topic not in topic_map or topic_map[topic]["type"] != "std_msgs/Float32":
            raise SelectedSignalV5Error(f"required topic is absent or wrong type: {topic}")
        if topic_map[topic]["message_count"] != len(signals[name]):
            raise SelectedSignalV5Error(f"reader census differs for {topic}")
    if "/odom" not in topic_map or topic_map["/odom"]["message_count"] != len(odom):
        raise SelectedSignalV5Error("reader census differs for /odom")
    return {"odom": odom, "signals": signals, "qc_counts": qc_counts,
            "reader_anomalies": summary.get("anomalies", [])}


def _alignment(odom: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if len(odom) < 3:
        raise SelectedSignalV5Error("at least three odom samples are required for alignment")
    record_times = [_integer_ns(row.get("bag_record_t_ns"), f"odom[{i}].record")
                    for i, row in enumerate(odom)]
    header_times = [_integer_ns(row.get("odom_header_t_ns"), f"odom[{i}].header")
                    for i, row in enumerate(odom)]
    if any(right <= left for left, right in zip(record_times, record_times[1:])):
        raise SelectedSignalV5Error("odom record time is not strictly increasing")
    if any(right <= left for left, right in zip(header_times, header_times[1:])):
        raise SelectedSignalV5Error("odom header time is not strictly increasing")
    offsets = sorted(header - record for record, header in zip(record_times, header_times))
    offset = offsets[(len(offsets) - 1) // 2]
    residuals = [header - (record + offset) for record, header in zip(record_times, header_times)]
    maximum = max(abs(value) for value in residuals)
    if maximum > MAXIMUM_ALIGNMENT_RESIDUAL_NS:
        raise SelectedSignalV5Error("record/header alignment residual exceeds frozen limit")
    return {
        "origin_ns": header_times[0],
        "end_ns": header_times[-1],
        "sample_count": len(odom),
        "offset_ns": offset,
        "residual_mean_ns": sum(residuals) / len(residuals),
        "residual_rms_ns": math.sqrt(sum(value * value for value in residuals) / len(residuals)),
        "residual_max_abs_ns": maximum,
        "residuals_sha256": sha256_json(residuals),
    }


def _map_series(raw: Sequence[Mapping[str, Any]], name: str, alignment: Mapping[str, Any],
                scale: float) -> list[dict[str, Any]]:
    if not 2 <= len(raw) <= MAXIMUM_MESSAGES_PER_TOPIC:
        raise SelectedSignalV5Error(f"{name} sample count is outside the frozen bound")
    output: list[dict[str, Any]] = []
    previous: int | None = None
    for index, row in enumerate(raw):
        record_time = _integer_ns(row.get("bag_record_t_ns"), f"{name}[{index}].record")
        if previous is not None and record_time <= previous:
            raise SelectedSignalV5Error(f"{name} record time is not strictly increasing")
        previous = record_time
        value = _finite(row.get("value_native"), f"{name}[{index}].value")
        mapped = record_time + int(alignment["offset_ns"])
        if mapped < int(alignment["origin_ns"]) or mapped > int(alignment["end_ns"]):
            raise SelectedSignalV5Error(f"{name} mapping would require extrapolation")
        output.append({
            "bag_record_t_ns": record_time,
            "mapped_odom_header_t_ns": mapped,
            "time_since_odom_origin_s": (mapped - int(alignment["origin_ns"])) / 1e9,
            "value_native": value,
            "value_comparison_mm": value * scale,
        })
    return output


def _build_result(captured: Mapping[str, Any], source: bytes, *, s5a0_sha256: str,
                  s5a1_sha256: str) -> dict[str, Any]:
    if captured.get("reader_anomalies") != []:
        raise SelectedSignalV5Error("reader reported a time anomaly")
    alignment = _alignment(captured["odom"])
    proxy = _map_series(captured["signals"]["H_proxy"], "H_proxy", alignment, 1000.0)
    modal = _map_series(captured["signals"]["H_modal"], "H_modal", alignment, 1.0)
    overlap_start = max(proxy[0]["time_since_odom_origin_s"], modal[0]["time_since_odom_origin_s"])
    overlap_end = min(proxy[-1]["time_since_odom_origin_s"], modal[-1]["time_since_odom_origin_s"])
    if overlap_end < overlap_start:
        raise SelectedSignalV5Error("H_proxy/H_modal have no common odom-header interval")
    qc_topics = [{"topic": topic, "message_count": int(captured["qc_counts"].get(topic, 0)),
                  "consumed_as_forcing": False} for topic in QC_ONLY_TOPICS]
    source_sha = sha256_bytes(source)
    result = {
        "schema_version": "smpcc-r8-liquid-s6-real-selected-signals-provenance-v5",
        "document_type": "SMPCC_R8_LIQUID_S6_REAL_SELECTED_SIGNALS_PROVENANCE_V5",
        "status": "S6_REAL_SELECTED_SIGNALS_SHAPE_FIXTURE_PRODUCED_NOT_ADMITTED",
        "mode": "STATIC_FIXTURE_ONLY_REAL_SHAPE", "fixture_only": True,
        "real_input_admitted": False, "attempt_id": PRIMARY_ATTEMPT, "planned_denominator": 1,
        "parents": {
            "s5a0_selected_bag_receipt": {"sha256": s5a0_sha256, "real_parent_materialized": False},
            "s5a1_transfer_manifest": {"sha256": s5a1_sha256, "real_parent_materialized": False},
            "source_bag": {"kind": "SYNTHETIC_PRIMARY_ROS1_BAG_V2_BYTES", "sha256": source_sha,
                           "size_bytes": len(source), "real_parent_materialized": False},
        },
        "reader_contract": {
            "reader_revision": "ROS1_BAG_V2_READER_V4_WITH_V1_SEMANTIC_HOOK",
            "reader_core_sha256": READER_CORE_SHA256, "reader_v4_sha256": READER_V4_SHA256,
            "extractor_v3_sha256": EXTRACTOR_V3_SHA256, "input_surface": "IMMUTABLE_BOUNDED_BYTES_ONLY",
        },
        "provenance": {
            "source_outcome": "SPMPC_NON_FIXED", "primary_only": True, "optional_bag_read": False,
            "source_bag_executed": False, "ros_started": False, "external_write_performed": False,
            "motion_exporter_consumed_selected_signals": False,
            "solver_forcing_consumed_selected_signals": False,
            "forbidden_forcing_signals_consumed": False, "comparison_only": True,
            "qc_only_topics": qc_topics,
        },
        "time_alignment": {
            "x_axis": "time_since_odom_header_origin_s", "motion_time_source": "/odom.header.stamp",
            "signal_native_time_source": "ROS1_BAG_RECORD_TIME_NS",
            "mapping_method": "LOWER_MEDIAN_ODOM_HEADER_MINUS_RECORD_OFFSET_V1",
            "odom_header_origin_ns": alignment["origin_ns"], "odom_header_end_ns": alignment["end_ns"],
            "offset_sample_count": alignment["sample_count"],
            "record_to_odom_header_offset_ns": alignment["offset_ns"],
            "residual_mean_ns": alignment["residual_mean_ns"],
            "residual_rms_ns": alignment["residual_rms_ns"],
            "residual_max_abs_ns": alignment["residual_max_abs_ns"],
            "maximum_allowed_residual_ns": MAXIMUM_ALIGNMENT_RESIDUAL_NS,
            "residuals_sha256": alignment["residuals_sha256"], "mapping_extrapolation": False,
            "overlap_start_s": overlap_start, "overlap_end_s": overlap_end,
        },
        "series": {
            "H_proxy": {"topic": "/slosh/height", "message_type": "std_msgs/Float32",
                        "native_unit": "m", "comparison_unit": "mm", "scale_to_comparison": 1000.0,
                        "offset_to_comparison": 0.0, "sample_count": len(proxy), "samples": proxy},
            "H_modal": {"topic": "/spmpc/slosh_height", "message_type": "std_msgs/Float32",
                        "native_unit": "mm", "comparison_unit": "mm", "scale_to_comparison": 1.0,
                        "offset_to_comparison": 0.0, "sample_count": len(modal), "samples": modal},
        },
        "integrity": {"source_bag_sha256": source_sha, "H_proxy_samples_sha256": sha256_json(proxy),
                      "H_modal_samples_sha256": sha256_json(modal),
                      "qc_only_topics_sha256": sha256_json(qc_topics)},
        "claims": {"stage6_pass": False, "single_row": True, "paired_ranking": False,
                   "cross_method_ranking": False, "cpu_selected_trajectory_comparison": False,
                   "physical_reference_pending": True, "physical_fidelity_validated": False,
                   "formal": False, "production": False},
    }
    return validate_result(result)


def validate_result(result: Any) -> dict[str, Any]:
    if not isinstance(result, Mapping):
        raise SelectedSignalV5Error("result root is not an object")
    try:
        Draft202012Validator(load_schema()).validate(result)
    except ValidationError as exc:
        location = "/".join(str(item) for item in exc.absolute_path)
        raise SelectedSignalV5Error(f"result schema failure at {location or '$'}: {exc.message}") from exc
    if result["integrity"]["source_bag_sha256"] != result["parents"]["source_bag"]["sha256"]:
        raise SelectedSignalV5Error("source identity hash is self-contradictory")
    qc = result["provenance"]["qc_only_topics"]
    if tuple(row["topic"] for row in qc) != QC_ONLY_TOPICS:
        raise SelectedSignalV5Error("QC-only topic set/order differs")
    if result["integrity"]["qc_only_topics_sha256"] != sha256_json(qc):
        raise SelectedSignalV5Error("QC-only topic hash is self-contradictory")
    offset = result["time_alignment"]["record_to_odom_header_offset_ns"]
    starts, ends = [], []
    for name, scale in (("H_proxy", 1000.0), ("H_modal", 1.0)):
        series = result["series"][name]
        samples = series["samples"]
        if series["sample_count"] != len(samples):
            raise SelectedSignalV5Error(f"{name} sample count is self-contradictory")
        if result["integrity"][f"{name}_samples_sha256"] != sha256_json(samples):
            raise SelectedSignalV5Error(f"{name} sample hash is self-contradictory")
        for row in samples:
            if row["mapped_odom_header_t_ns"] != row["bag_record_t_ns"] + offset:
                raise SelectedSignalV5Error(f"{name} mapped time is self-contradictory")
            if abs(row["value_comparison_mm"] - row["value_native"] * scale) > 1e-9:
                raise SelectedSignalV5Error(f"{name} unit conversion is self-contradictory")
        starts.append(samples[0]["time_since_odom_origin_s"])
        ends.append(samples[-1]["time_since_odom_origin_s"])
    if (result["time_alignment"]["overlap_start_s"] != max(starts)
            or result["time_alignment"]["overlap_end_s"] != min(ends)):
        raise SelectedSignalV5Error("series overlap is self-contradictory")
    canonical_json(result)
    return dict(result)


def produce_selected_signals(data: bytes, *, synthetic_fixture: bool,
                             expected_source_sha256: str, s5a0_receipt_sha256: str,
                             s5a1_transfer_sha256: str) -> dict[str, Any]:
    if not isinstance(data, bytes) or not 0 < len(data) <= MAXIMUM_SOURCE_BYTES:
        raise SelectedSignalV5Error("source bytes are absent or exceed the frozen bound")
    if not synthetic_fixture:
        raise SelectedSignalV5Error("REAL_OR_OPTIONAL_BAG_NOT_ADMITTED_FIXTURE_ONLY")
    expected = _require_sha256(expected_source_sha256, "source bag")
    if sha256_bytes(data) != expected:
        raise SelectedSignalV5Error("source bag SHA-256 differs")
    s5a0 = _require_sha256(s5a0_receipt_sha256, "S5A0 receipt")
    s5a1 = _require_sha256(s5a1_transfer_sha256, "S5A1 transfer")
    return _build_result(_capture_bag_bytes(data), data, s5a0_sha256=s5a0, s5a1_sha256=s5a1)


def self_check() -> dict[str, Any]:
    _assert_hardened_dependencies()
    source = b"s6-v5-in-memory-self-check-fixture"
    captured = {
        "odom": [
            {"bag_record_t_ns": 1_100_000_000, "odom_header_t_ns": 1_000_000_000},
            {"bag_record_t_ns": 2_100_000_000, "odom_header_t_ns": 2_000_000_000},
            {"bag_record_t_ns": 3_100_000_000, "odom_header_t_ns": 3_000_000_000},
        ],
        "signals": {
            "H_proxy": [{"bag_record_t_ns": 1_200_000_000, "value_native": 0.001},
                        {"bag_record_t_ns": 2_200_000_000, "value_native": 0.002}],
            "H_modal": [{"bag_record_t_ns": 1_300_000_000, "value_native": 1.0},
                        {"bag_record_t_ns": 2_300_000_000, "value_native": 2.0}],
        },
        "qc_counts": {topic: 0 for topic in QC_ONLY_TOPICS}, "reader_anomalies": [],
    }
    result = _build_result(captured, source, s5a0_sha256="a" * 64, s5a1_sha256="b" * 64)
    return {"status": "S6_REAL_SELECTED_SIGNAL_EXTRACTOR_V5_FIXTURE_SELF_CHECK_OK_NOT_ADMITTED",
            "schema_id": load_schema()["$id"], "planned_denominator": result["planned_denominator"],
            "motion_time_source": result["time_alignment"]["motion_time_source"],
            "raw_real_bag_read": False, "optional_bag_read": False, "external_write_performed": False,
            "ros_started": False, "solver_or_gpu_executed": False, "stage6_pass": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args(argv)
    if not args.self_check:
        parser.error("only --self-check is supported; no real bag path is admitted")
    print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["SelectedSignalV5Error", "produce_selected_signals", "validate_result",
           "canonical_json", "sha256_bytes", "sha256_json", "load_schema", "self_check"]
