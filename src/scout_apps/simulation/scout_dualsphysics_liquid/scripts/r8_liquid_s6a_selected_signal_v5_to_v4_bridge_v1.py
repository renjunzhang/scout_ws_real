#!/usr/bin/env python3
"""Fixture-only bridge from selected-signal provenance v5 to analyzer input v4.

Only exact in-memory JSON bytes and an in-memory frozen output grid are
accepted.  The bridge never reads a real bag/BI4/path, writes an artifact, or
admits optional/paired input.  Its output remains NOT_ADMITTED and explicitly
records REAL_INPUT_NOT_CONSUMED.
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

import r8_liquid_s6_real_selected_signal_extractor_v5 as source_v5  # noqa: E402


SCHEMA_PATH = ROOT / "schema/target_host_s6a_selected_signal_v5_to_v4_bridge_v1.json"
SOURCE_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_selected_signals_provenance_v5.json"
TARGET_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_selected_signal_input_contract_v4.json"
SOURCE_PRODUCER_PATH = MODULE_DIR / "r8_liquid_s6_real_selected_signal_extractor_v5.py"
TARGET_ANALYZER_PATH = MODULE_DIR / "r8_liquid_s6_real_input_static_layer_v4.py"
SOURCE_SCHEMA_SHA256 = "cd412f7bb7e672a391f95779400432245f738fd87b3e5a7e20ff18020d28a84e"
SOURCE_PRODUCER_SHA256 = "991bdc10f89b5056978ead95acb89fb904ffbe199a0c6769cafa64855e327373"
TARGET_SCHEMA_SHA256 = "b4c916260a4e08464aa4dfa188c922793dfd23d8c04abf0e870c575f91c1ac96"
TARGET_ANALYZER_SHA256 = "03017819818c23ff0881f73666948ab638f7887e7a131874288b8e786689ac1f"
MAXIMUM_SOURCE_DOCUMENT_BYTES = 32 * 1024 * 1024
MINIMUM_GRID_SAMPLES = 16


class SelectedSignalBridgeError(ValueError):
    """Source, grid, parent, semantic mapping, or result failure."""


def canonical_json(value: Any) -> bytes:
    try:
        encoded = json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":"))
    except (TypeError, ValueError) as exc:
        raise SelectedSignalBridgeError("value is not finite canonical JSON") from exc
    return (encoded + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def _sha(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)):
        raise SelectedSignalBridgeError(f"{label} is not a lowercase SHA-256")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise SelectedSignalBridgeError(f"{label} is non-finite or non-numeric")
    return float(value)


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise SelectedSignalBridgeError(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def _load_schema(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SelectedSignalBridgeError(f"schema root is not an object: {path.name}")
    Draft202012Validator.check_schema(value)
    assert_deep_closed(value)
    return value


def load_schemas() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    return _load_schema(SCHEMA_PATH), _load_schema(SOURCE_SCHEMA_PATH), _load_schema(TARGET_SCHEMA_PATH)


def _assert_dependencies() -> None:
    for path, expected in (
        (SOURCE_SCHEMA_PATH, SOURCE_SCHEMA_SHA256),
        (SOURCE_PRODUCER_PATH, SOURCE_PRODUCER_SHA256),
        (TARGET_SCHEMA_PATH, TARGET_SCHEMA_SHA256),
        (TARGET_ANALYZER_PATH, TARGET_ANALYZER_SHA256),
    ):
        if sha256_bytes(path.read_bytes()) != expected:
            raise SelectedSignalBridgeError(f"frozen dependency hash drifted: {path.name}")


def _decode_source(raw: bytes, expected_sha256: str) -> dict[str, Any]:
    if not isinstance(raw, bytes) or not 0 < len(raw) <= MAXIMUM_SOURCE_DOCUMENT_BYTES:
        raise SelectedSignalBridgeError("source v5 document is absent or exceeds the bound")
    if sha256_bytes(raw) != _sha(expected_sha256, "expected source v5 document"):
        raise SelectedSignalBridgeError("source v5 document hash drift")
    try:
        value = json.loads(raw, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    except (json.JSONDecodeError, ValueError) as exc:
        raise SelectedSignalBridgeError("source v5 document is invalid or non-finite JSON") from exc
    if not isinstance(value, Mapping):
        raise SelectedSignalBridgeError("source v5 document root is not an object")
    _, source_schema, _ = load_schemas()
    try:
        Draft202012Validator(source_schema).validate(value)
        checked = source_v5.validate_result(value)
    except (ValidationError, source_v5.SelectedSignalV5Error) as exc:
        raise SelectedSignalBridgeError(f"source v5 validation failed: {exc}") from exc
    return checked


def _validate_grid(values: Any, expected_sha256: str,
                   overlap_start: float, overlap_end: float) -> tuple[list[Any], list[float]]:
    if (not isinstance(values, Sequence) or isinstance(values, (str, bytes, bytearray))
            or len(values) < MINIMUM_GRID_SAMPLES):
        raise SelectedSignalBridgeError("output grid is absent or too short")
    frozen = list(values)
    if sha256_json(frozen) != _sha(expected_sha256, "expected output grid"):
        raise SelectedSignalBridgeError("output grid SHA-256 drift")
    numeric = [_finite(value, f"output_grid[{index}]") for index, value in enumerate(frozen)]
    deltas = [right - left for left, right in zip(numeric, numeric[1:])]
    if any(delta <= 0.0 for delta in deltas):
        raise SelectedSignalBridgeError("output grid is not strictly increasing")
    dt = sum(deltas) / len(deltas)
    if max(abs(delta - dt) for delta in deltas) > max(1e-12, dt * 1e-9):
        raise SelectedSignalBridgeError("output grid is not uniform")
    if numeric[0] < overlap_start - 1e-12 or numeric[-1] > overlap_end + 1e-12:
        raise SelectedSignalBridgeError("output grid requires forbidden extrapolation")
    return frozen, numeric


def _validate_source_semantics(source: Mapping[str, Any]) -> dict[str, list[dict[str, Any]]]:
    alignment = source["time_alignment"]
    origin = int(alignment["odom_header_origin_ns"])
    offset = int(alignment["record_to_odom_header_offset_ns"])
    projected: dict[str, list[dict[str, Any]]] = {}
    for name, expected_scale in (("H_proxy", 1000.0), ("H_modal", 1.0)):
        spec = source["series"][name]
        samples = spec["samples"]
        if len(samples) < MINIMUM_GRID_SAMPLES:
            raise SelectedSignalBridgeError(f"{name} has fewer than 16 samples required by v4")
        previous: int | None = None
        output: list[dict[str, Any]] = []
        for index, row in enumerate(samples):
            record = row["bag_record_t_ns"]
            if previous is not None and record <= previous:
                raise SelectedSignalBridgeError(f"{name} record time drift")
            previous = record
            mapped = record + offset
            expected_time = (mapped - origin) / 1e9
            if row["mapped_odom_header_t_ns"] != mapped:
                raise SelectedSignalBridgeError(f"{name} mapped time drift")
            if abs(_finite(row["time_since_odom_origin_s"], f"{name} time") - expected_time) > 1e-12:
                raise SelectedSignalBridgeError(f"{name} relative time drift")
            native = _finite(row["value_native"], f"{name} native value")
            comparison = _finite(row["value_comparison_mm"], f"{name} comparison value")
            if abs(comparison - native * expected_scale) > 1e-9:
                raise SelectedSignalBridgeError(f"{name} unit conversion drift")
            output.append({"bag_record_t_ns": record, "value_native": native})
        projected[name] = output
    return projected


def validate_bridge_result(result: Any) -> dict[str, Any]:
    if not isinstance(result, Mapping):
        raise SelectedSignalBridgeError("bridge result root is not an object")
    bridge_schema, _, target_schema = load_schemas()
    try:
        Draft202012Validator(bridge_schema).validate(result)
        Draft202012Validator(target_schema).validate(result["analyzer_input"])
    except ValidationError as exc:
        raise SelectedSignalBridgeError(f"bridge result schema failure: {exc.message}") from exc
    source_contract = result["source_contract"]
    expected_dependencies = {
        "source_v5_schema_sha256": SOURCE_SCHEMA_SHA256,
        "source_v5_producer_sha256": SOURCE_PRODUCER_SHA256,
        "target_v4_schema_sha256": TARGET_SCHEMA_SHA256,
        "target_v4_analyzer_sha256": TARGET_ANALYZER_SHA256,
    }
    if any(source_contract[key] != value for key, value in expected_dependencies.items()):
        raise SelectedSignalBridgeError("bridge dependency binding drift")
    analyzer = result["analyzer_input"]
    parents = result["parent_bindings"]
    if (analyzer["parents"]["s5a0_selected_bag_receipt"]["sha256"]
            != parents["s5a0_selected_bag_receipt_sha256"]
            or analyzer["parents"]["s5a1_transfer_manifest"]["sha256"]
            != parents["s5a1_transfer_manifest_sha256"]
            or analyzer["parents"]["source_bag"]["sha256"] != parents["source_bag_sha256"]):
        raise SelectedSignalBridgeError("bridge parent identity is self-contradictory")
    if analyzer["time_alignment"]["output_grid_sha256"] != result["grid_contract"]["output_grid_sha256"]:
        raise SelectedSignalBridgeError("bridge output grid binding is self-contradictory")
    if (analyzer["time_alignment"]["odom_header_origin_ns"]
            != source_contract["source_odom_header_origin_ns"]
            or analyzer["time_alignment"]["record_to_odom_offset_ns"]
            != source_contract["source_record_to_odom_header_offset_ns"]):
        raise SelectedSignalBridgeError("bridge source time mapping is self-contradictory")
    if result["integrity"]["analyzer_input_sha256"] != sha256_json(analyzer):
        raise SelectedSignalBridgeError("bridge analyzer input hash is self-contradictory")
    for name in ("H_proxy", "H_modal"):
        if result["integrity"][f"{name}_samples_sha256"] != sha256_json(analyzer["series"][name]["samples"]):
            raise SelectedSignalBridgeError(f"bridge {name} hash is self-contradictory")
    canonical_json(result)
    return dict(result)


def bridge_v5_to_v4(
    source_raw: bytes,
    output_grid_slots_s: Any,
    *,
    expected_source_v5_sha256: str,
    expected_output_grid_sha256: str,
    expected_s5a0_receipt_sha256: str,
    expected_s5a1_transfer_sha256: str,
    expected_source_bag_sha256: str,
    fixture_only: bool,
    real_input_consumed: bool,
    optional_admitted: bool,
) -> dict[str, Any]:
    if fixture_only is not True or real_input_consumed is not False:
        raise SelectedSignalBridgeError("REAL_INPUT_NOT_CONSUMED fixture-only contract violated")
    if optional_admitted is not False:
        raise SelectedSignalBridgeError("optional or paired input is not admitted")
    _assert_dependencies()
    source = _decode_source(source_raw, expected_source_v5_sha256)
    if source["planned_denominator"] != 1 or not source["provenance"]["primary_only"]:
        raise SelectedSignalBridgeError("primary-only denominator differs from 1")
    if source["provenance"]["optional_bag_read"]:
        raise SelectedSignalBridgeError("optional source was consumed")
    parent_expected = {
        "s5a0_selected_bag_receipt": _sha(expected_s5a0_receipt_sha256, "expected S5A0 receipt"),
        "s5a1_transfer_manifest": _sha(expected_s5a1_transfer_sha256, "expected S5A1 transfer"),
        "source_bag": _sha(expected_source_bag_sha256, "expected source bag"),
    }
    for name, expected in parent_expected.items():
        if source["parents"][name]["sha256"] != expected:
            raise SelectedSignalBridgeError(f"{name} parent identity drift")
    if source["integrity"]["source_bag_sha256"] != parent_expected["source_bag"]:
        raise SelectedSignalBridgeError("source bag integrity/parent identity drift")
    projected = _validate_source_semantics(source)
    source_overlap_start = float(source["time_alignment"]["overlap_start_s"])
    source_overlap_end = float(source["time_alignment"]["overlap_end_s"])
    _, grid = _validate_grid(output_grid_slots_s, expected_output_grid_sha256,
                             source_overlap_start, source_overlap_end)
    output_grid_sha = sha256_json(list(output_grid_slots_s))
    analyzer_input = {
        "schema_version": "smpcc-r8-liquid-s6-real-selected-signal-input-contract-v4",
        "document_type": "SMPCC_R8_LIQUID_S6_REAL_SELECTED_SIGNAL_INPUT_CONTRACT_V4",
        "status": "S6_REAL_SELECTED_SIGNAL_SHAPE_FIXTURE_VALIDATED_NOT_ADMITTED",
        "mode": "STATIC_FIXTURE_FOR_REAL_INPUT_CONTRACT", "real_input_admitted": False,
        "attempt_id": source["attempt_id"], "planned_denominator": 1,
        "parents": {
            "s5a0_selected_bag_receipt": {"kind": "FINALIZED_S5A0_RECEIPT_IDENTITY_FIXTURE",
                                           "sha256": parent_expected["s5a0_selected_bag_receipt"],
                                           "real_parent_materialized": False},
            "s5a1_transfer_manifest": {"kind": "FINALIZED_S5A1_TRANSFER_IDENTITY_FIXTURE",
                                        "sha256": parent_expected["s5a1_transfer_manifest"],
                                        "real_parent_materialized": False},
            "source_bag": {"kind": "SOURCE_ROS1_BAG_IDENTITY_FIXTURE",
                           "sha256": parent_expected["source_bag"],
                           "size_bytes": source["parents"]["source_bag"]["size_bytes"],
                           "real_parent_materialized": False},
        },
        "provenance": {
            "target_s5a0_status": "S5A0_PRIMARY_SELECTED_BAG_ACCEPTED",
            "target_s5a1_status": "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED",
            "source_outcome": source["provenance"]["source_outcome"],
            "original_root_zero_write_required": True, "real_files_read": False,
            "motion_exporter_consumed_selected_signals": False,
            "solver_forcing_consumed_selected_signals": False, "comparison_only": True,
        },
        "time_alignment": {
            "x_axis": source["time_alignment"]["x_axis"],
            "sample_time_source": source["time_alignment"]["signal_native_time_source"],
            "odom_header_origin_ns": source["time_alignment"]["odom_header_origin_ns"],
            "record_to_odom_offset_ns": source["time_alignment"]["record_to_odom_header_offset_ns"],
            "overlap_start_s": grid[0], "overlap_end_s": grid[-1],
            "output_grid_sha256": output_grid_sha,
            "interpolation": "LINEAR_WITHIN_OVERLAP_ONLY", "extrapolation": False,
            "smoothing": False,
        },
        "series": {
            "H_proxy": {"topic": "/slosh/height", "message_type": "std_msgs/Float32",
                        "native_unit": "m", "comparison_unit": "mm",
                        "scale_to_comparison": 1000.0, "offset_to_comparison": 0.0,
                        "samples": projected["H_proxy"]},
            "H_modal": {"topic": "/spmpc/slosh_height", "message_type": "std_msgs/Float32",
                        "native_unit": "mm", "comparison_unit": "mm",
                        "scale_to_comparison": 1.0, "offset_to_comparison": 0.0,
                        "samples": projected["H_modal"]},
        },
        "claims": {"stage6_pass": False, "single_row": True, "paired_ranking": False,
                   "cpu_selected_trajectory_comparison": False,
                   "physical_reference_pending": True, "physical_fidelity_validated": False,
                   "formal": False, "production": False},
    }
    result = {
        "schema_version": "smpcc-r8-liquid-s6a-selected-signal-v5-to-v4-bridge-v1",
        "document_type": "SMPCC_R8_LIQUID_S6A_SELECTED_SIGNAL_V5_TO_V4_BRIDGE_V1",
        "status": "S6A_V5_TO_V4_BRIDGE_VALIDATED_NOT_ADMITTED",
        "mode": "FIXTURE_ONLY_REAL_INPUT_NOT_CONSUMED", "fixture_only": True,
        "real_input_consumed": False, "optional_input_consumed": False,
        "source_contract": {
            "source_v5_document_sha256": expected_source_v5_sha256,
            "source_v5_schema_sha256": SOURCE_SCHEMA_SHA256,
            "source_v5_producer_sha256": SOURCE_PRODUCER_SHA256,
            "target_v4_schema_sha256": TARGET_SCHEMA_SHA256,
            "target_v4_analyzer_sha256": TARGET_ANALYZER_SHA256,
            "source_odom_header_origin_ns": source["time_alignment"]["odom_header_origin_ns"],
            "source_record_to_odom_header_offset_ns": source["time_alignment"]["record_to_odom_header_offset_ns"],
        },
        "grid_contract": {"output_grid_sha256": output_grid_sha, "sample_count": len(grid),
                          "start_s": grid[0], "end_s": grid[-1],
                          "strictly_increasing": True, "uniform": True,
                          "inside_selected_overlap": True},
        "parent_bindings": {
            "s5a0_selected_bag_receipt_sha256": parent_expected["s5a0_selected_bag_receipt"],
            "s5a1_transfer_manifest_sha256": parent_expected["s5a1_transfer_manifest"],
            "source_bag_sha256": parent_expected["source_bag"],
        },
        "analyzer_input": analyzer_input,
        "integrity": {"analyzer_input_sha256": sha256_json(analyzer_input),
                      "H_proxy_samples_sha256": sha256_json(projected["H_proxy"]),
                      "H_modal_samples_sha256": sha256_json(projected["H_modal"])},
        "claims": {"stage6_pass": False, "real_input_admitted": False,
                   "real_input_consumed": False, "optional_admitted": False,
                   "paired_ranking": False, "physical_reference_pending": True,
                   "physical_fidelity_validated": False, "formal": False,
                   "production": False},
    }
    return validate_bridge_result(result)


def synthetic_v5_fixture() -> dict[str, Any]:
    origin, offset = 1_000_000_000, -100_000_000
    proxy_samples, modal_samples = [], []
    for index in range(16):
        record = 1_100_000_000 + index * 100_000_000
        mapped = record + offset
        relative = (mapped - origin) / 1e9
        proxy_native = 0.001 + index * 0.0001
        modal_native = 1.0 + index * 0.1
        proxy_samples.append({"bag_record_t_ns": record, "mapped_odom_header_t_ns": mapped,
                              "time_since_odom_origin_s": relative, "value_native": proxy_native,
                              "value_comparison_mm": proxy_native * 1000.0})
        modal_samples.append({"bag_record_t_ns": record, "mapped_odom_header_t_ns": mapped,
                              "time_since_odom_origin_s": relative, "value_native": modal_native,
                              "value_comparison_mm": modal_native})
    qc = [{"topic": topic, "message_count": 0, "consumed_as_forcing": False}
          for topic in source_v5.QC_ONLY_TOPICS]
    result = {
        "schema_version": "smpcc-r8-liquid-s6-real-selected-signals-provenance-v5",
        "document_type": "SMPCC_R8_LIQUID_S6_REAL_SELECTED_SIGNALS_PROVENANCE_V5",
        "status": "S6_REAL_SELECTED_SIGNALS_SHAPE_FIXTURE_PRODUCED_NOT_ADMITTED",
        "mode": "STATIC_FIXTURE_ONLY_REAL_SHAPE", "fixture_only": True,
        "real_input_admitted": False, "attempt_id": source_v5.PRIMARY_ATTEMPT,
        "planned_denominator": 1,
        "parents": {
            "s5a0_selected_bag_receipt": {"sha256": "a" * 64, "real_parent_materialized": False},
            "s5a1_transfer_manifest": {"sha256": "b" * 64, "real_parent_materialized": False},
            "source_bag": {"kind": "SYNTHETIC_PRIMARY_ROS1_BAG_V2_BYTES",
                           "sha256": "c" * 64, "size_bytes": 4096,
                           "real_parent_materialized": False},
        },
        "reader_contract": {"reader_revision": "ROS1_BAG_V2_READER_V4_WITH_V1_SEMANTIC_HOOK",
                            "reader_core_sha256": source_v5.READER_CORE_SHA256,
                            "reader_v4_sha256": source_v5.READER_V4_SHA256,
                            "extractor_v3_sha256": source_v5.EXTRACTOR_V3_SHA256,
                            "input_surface": "IMMUTABLE_BOUNDED_BYTES_ONLY"},
        "provenance": {"source_outcome": "SPMPC_NON_FIXED", "primary_only": True,
                       "optional_bag_read": False, "source_bag_executed": False,
                       "ros_started": False, "external_write_performed": False,
                       "motion_exporter_consumed_selected_signals": False,
                       "solver_forcing_consumed_selected_signals": False,
                       "forbidden_forcing_signals_consumed": False,
                       "comparison_only": True, "qc_only_topics": qc},
        "time_alignment": {"x_axis": "time_since_odom_header_origin_s",
                           "motion_time_source": "/odom.header.stamp",
                           "signal_native_time_source": "ROS1_BAG_RECORD_TIME_NS",
                           "mapping_method": "LOWER_MEDIAN_ODOM_HEADER_MINUS_RECORD_OFFSET_V1",
                           "odom_header_origin_ns": origin, "odom_header_end_ns": 2_500_000_000,
                           "offset_sample_count": 3,
                           "record_to_odom_header_offset_ns": offset,
                           "residual_mean_ns": 0.0, "residual_rms_ns": 0.0,
                           "residual_max_abs_ns": 0, "maximum_allowed_residual_ns": 5_000_000,
                           "residuals_sha256": source_v5.sha256_json([0, 0, 0]),
                           "mapping_extrapolation": False, "overlap_start_s": 0.0,
                           "overlap_end_s": 1.5},
        "series": {
            "H_proxy": {"topic": "/slosh/height", "message_type": "std_msgs/Float32",
                        "native_unit": "m", "comparison_unit": "mm",
                        "scale_to_comparison": 1000.0, "offset_to_comparison": 0.0,
                        "sample_count": 16, "samples": proxy_samples},
            "H_modal": {"topic": "/spmpc/slosh_height", "message_type": "std_msgs/Float32",
                        "native_unit": "mm", "comparison_unit": "mm",
                        "scale_to_comparison": 1.0, "offset_to_comparison": 0.0,
                        "sample_count": 16, "samples": modal_samples},
        },
        "integrity": {"source_bag_sha256": "c" * 64,
                      "H_proxy_samples_sha256": source_v5.sha256_json(proxy_samples),
                      "H_modal_samples_sha256": source_v5.sha256_json(modal_samples),
                      "qc_only_topics_sha256": source_v5.sha256_json(qc)},
        "claims": {"stage6_pass": False, "single_row": True, "paired_ranking": False,
                   "cross_method_ranking": False, "cpu_selected_trajectory_comparison": False,
                   "physical_reference_pending": True, "physical_fidelity_validated": False,
                   "formal": False, "production": False},
    }
    return source_v5.validate_result(result)


def self_check() -> dict[str, Any]:
    source = synthetic_v5_fixture()
    raw = canonical_json(source)
    grid = [index * 0.1 for index in range(16)]
    result = bridge_v5_to_v4(
        raw, grid, expected_source_v5_sha256=sha256_bytes(raw),
        expected_output_grid_sha256=sha256_json(grid),
        expected_s5a0_receipt_sha256="a" * 64,
        expected_s5a1_transfer_sha256="b" * 64,
        expected_source_bag_sha256="c" * 64,
        fixture_only=True, real_input_consumed=False, optional_admitted=False,
    )
    return {"status": "S6A_V5_TO_V4_BRIDGE_SELF_CHECK_OK_NOT_ADMITTED",
            "real_input_status": "REAL_INPUT_NOT_CONSUMED",
            "bridge_schema_id": load_schemas()[0]["$id"],
            "target_schema_id": load_schemas()[2]["$id"],
            "planned_denominator": result["analyzer_input"]["planned_denominator"],
            "sample_count": result["grid_contract"]["sample_count"],
            "real_bag_read": False, "real_bi4_read": False,
            "external_write_performed": False, "optional_admitted": False,
            "stage6_pass": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args(argv)
    if not args.self_check:
        parser.error("only --self-check is supported; real input is not consumed")
    print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["SelectedSignalBridgeError", "bridge_v5_to_v4", "validate_bridge_result",
           "synthetic_v5_fixture", "self_check", "canonical_json", "sha256_bytes",
           "sha256_json", "load_schemas"]
