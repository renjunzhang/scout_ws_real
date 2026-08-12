#!/usr/bin/env python3
"""Bounded byte-only reader for S6 H_proxy and H_modal series.

The public entry point accepts an in-memory ROS1 Bag V2 byte string.  It never
opens a bag path, starts ROS, or writes output.  This revision admits synthetic
fixtures only because the finalized S5B0 replay parent is NOT_MATERIALIZED.
"""

from __future__ import annotations

import hashlib
import math
import sys
from pathlib import Path
from typing import Any


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_ros1_bag_v2_reader_v1 as bag_reader  # noqa: E402


MAXIMUM_SOURCE_BYTES = 67_108_864
MAXIMUM_MESSAGES_PER_TOPIC = 100_000
MINIMUM_MESSAGES_PER_TOPIC = 2
SIGNALS = {
    "/slosh/height": ("H_proxy", "m", 1000.0),
    "/spmpc/slosh_height": ("H_modal", "mm", 1.0),
}


class SelectedSignalError(ValueError):
    """Malformed bytes, missing series, bad time, or unauthorized real input."""


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _validate_series(rows: list[dict[str, Any]], name: str) -> None:
    if not MINIMUM_MESSAGES_PER_TOPIC <= len(rows) <= MAXIMUM_MESSAGES_PER_TOPIC:
        raise SelectedSignalError(f"{name} sample count is outside the frozen bound")
    times = [int(row["bag_record_t_ns"]) for row in rows]
    if any(right <= left for left, right in zip(times, times[1:])):
        raise SelectedSignalError(f"{name} record time is not strictly increasing")
    if any(not math.isfinite(float(row["value_native"])) for row in rows):
        raise SelectedSignalError(f"{name} contains a non-finite value")


def extract_selected_signals(
    data: bytes,
    *,
    synthetic_fixture: bool,
    expected_sha256: str | None = None,
) -> dict[str, Any]:
    """Validate a complete bag and return only the two registered Float32 series."""
    if not isinstance(data, bytes) or not 0 < len(data) <= MAXIMUM_SOURCE_BYTES:
        raise SelectedSignalError("source bytes are absent or exceed the frozen bound")
    observed_sha256 = _sha256(data)
    if expected_sha256 is not None and expected_sha256 != observed_sha256:
        raise SelectedSignalError("source SHA-256 differs")
    if not synthetic_fixture:
        raise SelectedSignalError("REAL_SELECTED_BAG_NOT_ADMITTED_PARENT_NOT_MATERIALIZED")

    rows: dict[str, list[dict[str, Any]]] = {name: [] for name, _, _ in SIGNALS.values()}
    record_times: dict[int, int] = {}
    original_message = bag_reader._Message
    original_decoder = bag_reader._decode_registered_payload

    def capture_message(*args: Any, **kwargs: Any):
        message = original_message(*args, **kwargs)
        record_times[id(message.payload)] = int(message.record_time_ns)
        return message

    def capture_decoder(topic: str, message_type: str, payload: memoryview, limits: Any):
        if topic in SIGNALS:
            if message_type != "std_msgs/Float32":
                raise SelectedSignalError(f"{topic} message type differs")
            record_time = record_times.get(id(payload))
            if record_time is None:
                raise SelectedSignalError("message time provenance association failed")
            cursor = bag_reader._PayloadCursor(payload, limits, label=topic)
            value = float(cursor.floats(1, "data")[0])
            cursor.require_eof()
            name, native_unit, scale = SIGNALS[topic]
            rows[name].append({
                "bag_record_t_ns": record_time,
                "value_native": value,
                "value_comparison_mm": value * scale,
            })
        return original_decoder(topic, message_type, payload, limits)

    bag_reader._Message = capture_message
    bag_reader._decode_registered_payload = capture_decoder
    try:
        summary = bag_reader.parse_bag_v2(data)
    except bag_reader.BagV2Error as exc:
        raise SelectedSignalError(str(exc)) from exc
    finally:
        bag_reader._Message = original_message
        bag_reader._decode_registered_payload = original_decoder

    topic_map = {item["topic"]: item for item in summary["topics"]}
    for topic, (name, _, _) in SIGNALS.items():
        if topic not in topic_map or topic_map[topic]["type"] != "std_msgs/Float32":
            raise SelectedSignalError(f"required topic is absent or wrong type: {topic}")
        if topic_map[topic]["message_count"] != len(rows[name]):
            raise SelectedSignalError(f"reader census differs for {topic}")
        _validate_series(rows[name], name)

    return {
        "schema_version": "smpcc-r8-liquid-s6-selected-signals-v1",
        "document_type": "SMPCC_R8_LIQUID_S6_SELECTED_SIGNALS_V1",
        "status": "S6_SELECTED_SIGNALS_SYNTHETIC_FIXTURE_VALIDATED_NOT_FINAL",
        "attempt_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01",
        "planned_denominator": 1,
        "source": {
            "kind": "SYNTHETIC_ROS1_BAG_V2_BYTES",
            "sha256": observed_sha256,
            "size_bytes": len(data),
            "real_bag_read": False,
        },
        "time_source": "ROS1_BAG_RECORD_TIME_NS",
        "series": {
            "H_proxy": {"topic": "/slosh/height", "native_unit": "m", "comparison_unit": "mm", "scale": 1000.0, "samples": rows["H_proxy"]},
            "H_modal": {"topic": "/spmpc/slosh_height", "native_unit": "mm", "comparison_unit": "mm", "scale": 1.0, "samples": rows["H_modal"]},
        },
        "claims": {
            "stage6_pass": False,
            "physical_reference_pending": True,
            "physical_fidelity_validated": False,
            "paired_ranking": False,
            "cpu_selected_trajectory_comparison": False,
        },
    }


__all__ = ["SelectedSignalError", "extract_selected_signals", "SIGNALS"]
