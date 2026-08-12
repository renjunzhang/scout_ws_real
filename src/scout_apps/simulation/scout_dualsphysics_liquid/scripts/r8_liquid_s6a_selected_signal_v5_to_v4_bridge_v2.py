#!/usr/bin/env python3
"""Semantically corrected fixture bridge for selected-signal v5 to v4 shape.

The frozen v5 producer and v4 target mislabeled ``SPMPC_NON_FIXED`` as a
source outcome.  The authoritative continuation plan and S5A1 package instead
define it as the selected bag/topic schema; the selected attempt has no exact
outcome sidecar and therefore remains ``UNKNOWN``.  This create-new adapter
keeps the v5-to-v4 time, grid, parent and hash closure while emitting a
corrected analyzer envelope that separates ``source_schema`` from
``source_outcome``.  A future non-UNKNOWN outcome is accepted only from exact
in-memory authoritative sidecar bytes bound by SHA-256.  No real bag, BI4,
optional row, path input or external output is supported.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
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

import r8_liquid_s6a_selected_signal_v5_to_v4_bridge_v1 as legacy  # noqa: E402


SelectedSignalBridgeError = legacy.SelectedSignalBridgeError
SCHEMA_PATH = ROOT / "schema/target_host_s6a_selected_signal_v5_to_v4_bridge_v2.json"
BRIDGE_V1_SCHEMA_PATH = ROOT / "schema/target_host_s6a_selected_signal_v5_to_v4_bridge_v1.json"
BRIDGE_V1_SCRIPT_PATH = MODULE_DIR / "r8_liquid_s6a_selected_signal_v5_to_v4_bridge_v1.py"
BRIDGE_V1_SCHEMA_SHA256 = "6030a1f4941eec795a4ffcd29a7c2549b3151dd33f1bb74be6bc7790b66b8278"
BRIDGE_V1_SCRIPT_SHA256 = "266f763b159b30d3296ef8adb4a1ad26558cc67ac58d339c065d0b9a1910fe98"
SOURCE_SCHEMA = "SPMPC_NON_FIXED"
UNKNOWN_OUTCOME = "UNKNOWN"
KNOWN_OUTCOMES = frozenset({"METHOD_SUCCESS", "METHOD_FAILURE"})
KNOWN_TERMINALS = frozenset({"GOAL_REACHED", "GOAL_TIMEOUT", "OTHER_FROZEN_TERMINAL"})
SIDECAR_SCHEMA_VERSION = "smpcc-r8-liquid-r7-source-outcome-sidecar-v1"
SIDECAR_DOCUMENT_TYPE = "SMPCC_R8_LIQUID_R7_SOURCE_OUTCOME_SIDECAR_V1"
MAXIMUM_SIDECAR_BYTES = 64 * 1024


canonical_json = legacy.canonical_json
sha256_bytes = legacy.sha256_bytes
sha256_json = legacy.sha256_json
assert_deep_closed = legacy.assert_deep_closed
source_v5 = legacy.source_v5


def _path_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _assert_dependencies() -> None:
    for path, expected in (
        (BRIDGE_V1_SCHEMA_PATH, BRIDGE_V1_SCHEMA_SHA256),
        (BRIDGE_V1_SCRIPT_PATH, BRIDGE_V1_SCRIPT_SHA256),
    ):
        if _path_sha256(path) != expected:
            raise SelectedSignalBridgeError(f"frozen bridge-v1 dependency hash drifted: {path.name}")
    legacy._assert_dependencies()


def load_schemas() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    return (
        legacy._load_schema(SCHEMA_PATH),
        legacy._load_schema(legacy.SOURCE_SCHEMA_PATH),
        legacy._load_schema(legacy.TARGET_SCHEMA_PATH),
    )


def _decode_source(raw: bytes, expected_sha256: str) -> dict[str, Any]:
    source = legacy._decode_source(raw, expected_sha256)
    legacy_field = source["provenance"]["source_outcome"]
    if legacy_field != SOURCE_SCHEMA:
        raise SelectedSignalBridgeError("legacy v5 schema carrier differs from SPMPC_NON_FIXED")
    return source


def authoritative_sidecar_fixture(source_outcome: str, source_terminal: str) -> dict[str, Any]:
    """Return a closed synthetic sidecar shape for tests; it grants no admission."""
    return {
        "schema_version": SIDECAR_SCHEMA_VERSION,
        "document_type": SIDECAR_DOCUMENT_TYPE,
        "attempt_id": source_v5.PRIMARY_ATTEMPT,
        "source_outcome": source_outcome,
        "source_terminal": source_terminal,
        "authoritative": True,
        "outcome_recomputed_from_topics": False,
        "outcome_inferred_from_bag_duration": False,
        "outcome_inferred_from_replay": False,
    }


def _decode_sidecar(raw: bytes, expected_sha256: str) -> dict[str, Any]:
    if not isinstance(raw, bytes) or not 0 < len(raw) <= MAXIMUM_SIDECAR_BYTES:
        raise SelectedSignalBridgeError("authoritative outcome sidecar is absent or exceeds the bound")
    expected = legacy._sha(expected_sha256, "expected authoritative outcome sidecar")
    if sha256_bytes(raw) != expected:
        raise SelectedSignalBridgeError("authoritative outcome sidecar hash drift")
    try:
        value = json.loads(raw, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    except (json.JSONDecodeError, ValueError) as exc:
        raise SelectedSignalBridgeError("authoritative outcome sidecar is invalid JSON") from exc
    if not isinstance(value, Mapping):
        raise SelectedSignalBridgeError("authoritative outcome sidecar root is not an object")
    required = {
        "schema_version", "document_type", "attempt_id", "source_outcome",
        "source_terminal", "authoritative", "outcome_recomputed_from_topics",
        "outcome_inferred_from_bag_duration", "outcome_inferred_from_replay",
    }
    if set(value) != required:
        raise SelectedSignalBridgeError("authoritative outcome sidecar is not closed")
    if (
        value["schema_version"] != SIDECAR_SCHEMA_VERSION
        or value["document_type"] != SIDECAR_DOCUMENT_TYPE
        or value["attempt_id"] != source_v5.PRIMARY_ATTEMPT
        or value["authoritative"] is not True
        or value["outcome_recomputed_from_topics"] is not False
        or value["outcome_inferred_from_bag_duration"] is not False
        or value["outcome_inferred_from_replay"] is not False
    ):
        raise SelectedSignalBridgeError("authoritative outcome sidecar identity or authority differs")
    return dict(value)


def _resolve_outcome(
    source_outcome: str,
    source_terminal: str,
    sidecar_raw: bytes | None,
    expected_sidecar_sha256: str | None,
    *,
    infer_outcome_from_topics: bool,
    infer_outcome_from_bag_duration: bool,
) -> dict[str, Any]:
    if infer_outcome_from_topics is not False or infer_outcome_from_bag_duration is not False:
        raise SelectedSignalBridgeError("automatic source outcome inference is forbidden")
    if source_outcome == SOURCE_SCHEMA:
        raise SelectedSignalBridgeError("source schema must not be written into source_outcome")
    if source_outcome == "GOAL_TIMEOUT":
        raise SelectedSignalBridgeError("GOAL_TIMEOUT is a terminal, not a source_outcome")
    if source_outcome == UNKNOWN_OUTCOME:
        if source_terminal != "UNKNOWN":
            raise SelectedSignalBridgeError("UNKNOWN outcome cannot infer or claim a terminal")
        if sidecar_raw is not None or expected_sidecar_sha256 is not None:
            raise SelectedSignalBridgeError("UNKNOWN outcome must not bind an unused sidecar")
        return {
            "source_outcome": "UNKNOWN",
            "source_terminal": "UNKNOWN",
            "authority_mode": "NO_AUTHORITATIVE_SIDECAR_BOUND",
            "authoritative_sidecar_bound": False,
            "authoritative_sidecar_sha256": None,
            "inferred_from_topics": False,
            "inferred_from_bag_duration": False,
            "inferred_from_replay": False,
        }
    if source_outcome not in KNOWN_OUTCOMES:
        raise SelectedSignalBridgeError("source_outcome is outside the closed authoritative enum")
    if source_terminal not in KNOWN_TERMINALS:
        raise SelectedSignalBridgeError("non-UNKNOWN source outcome requires a frozen terminal")
    if sidecar_raw is None or expected_sidecar_sha256 is None:
        raise SelectedSignalBridgeError("non-UNKNOWN source outcome requires a hash-bound authoritative sidecar")
    sidecar = _decode_sidecar(sidecar_raw, expected_sidecar_sha256)
    if sidecar["source_outcome"] != source_outcome or sidecar["source_terminal"] != source_terminal:
        raise SelectedSignalBridgeError("requested source outcome/terminal differs from authoritative sidecar")
    if sidecar["source_outcome"] not in KNOWN_OUTCOMES or sidecar["source_terminal"] not in KNOWN_TERMINALS:
        raise SelectedSignalBridgeError("authoritative sidecar outcome/terminal is outside the closed enum")
    return {
        "source_outcome": source_outcome,
        "source_terminal": source_terminal,
        "authority_mode": "AUTHORITATIVE_SIDECAR_SHA256_BOUND",
        "authoritative_sidecar_bound": True,
        "authoritative_sidecar_sha256": expected_sidecar_sha256,
        "inferred_from_topics": False,
        "inferred_from_bag_duration": False,
        "inferred_from_replay": False,
    }


def validate_target_v4_shape(analyzer: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the unchanged v4 shape through a non-emitted legacy adapter."""
    projection = copy.deepcopy(dict(analyzer))
    projection["schema_version"] = "smpcc-r8-liquid-s6-real-selected-signal-input-contract-v4"
    projection["document_type"] = "SMPCC_R8_LIQUID_S6_REAL_SELECTED_SIGNAL_INPUT_CONTRACT_V4"
    provenance = projection["provenance"]
    if provenance.pop("source_schema") != SOURCE_SCHEMA:
        raise SelectedSignalBridgeError("corrected analyzer source_schema differs")
    evidence = provenance.pop("source_outcome_evidence")
    if evidence["source_outcome"] == SOURCE_SCHEMA:
        raise SelectedSignalBridgeError("source schema leaked into corrected source_outcome")
    provenance["source_outcome"] = SOURCE_SCHEMA
    projection["claims"].pop("source_outcome_inferred")
    try:
        Draft202012Validator(load_schemas()[2]).validate(projection)
    except ValidationError as exc:
        raise SelectedSignalBridgeError(f"target v4 shape projection failed: {exc.message}") from exc
    return projection


def validate_bridge_result(result: Any) -> dict[str, Any]:
    if not isinstance(result, Mapping):
        raise SelectedSignalBridgeError("bridge result root is not an object")
    _assert_dependencies()
    bridge_schema, _, _ = load_schemas()
    try:
        Draft202012Validator(bridge_schema).validate(result)
    except ValidationError as exc:
        raise SelectedSignalBridgeError(f"bridge result schema failure: {exc.message}") from exc
    source_contract = result["source_contract"]
    expected_dependencies = {
        "source_v5_schema_sha256": legacy.SOURCE_SCHEMA_SHA256,
        "source_v5_producer_sha256": legacy.SOURCE_PRODUCER_SHA256,
        "bridge_v1_schema_sha256": BRIDGE_V1_SCHEMA_SHA256,
        "bridge_v1_script_sha256": BRIDGE_V1_SCRIPT_SHA256,
        "target_v4_schema_sha256": legacy.TARGET_SCHEMA_SHA256,
        "target_v4_analyzer_sha256": legacy.TARGET_ANALYZER_SHA256,
    }
    if any(source_contract[key] != value for key, value in expected_dependencies.items()):
        raise SelectedSignalBridgeError("bridge dependency binding drift")
    if (
        source_contract["source_schema"] != SOURCE_SCHEMA
        or source_contract["legacy_v5_field_value"] != SOURCE_SCHEMA
        or source_contract["legacy_v5_field_interpretation"] != "SOURCE_SCHEMA_ONLY_NOT_SOURCE_OUTCOME"
    ):
        raise SelectedSignalBridgeError("legacy v5 semantic correction drift")
    analyzer = result["analyzer_input"]
    outcome = result["outcome_contract"]
    if analyzer["provenance"]["source_schema"] != SOURCE_SCHEMA:
        raise SelectedSignalBridgeError("analyzer source schema drift")
    if analyzer["provenance"]["source_outcome_evidence"] != outcome:
        raise SelectedSignalBridgeError("analyzer outcome evidence is self-contradictory")
    if outcome["source_outcome"] == SOURCE_SCHEMA:
        raise SelectedSignalBridgeError("source schema was emitted as source outcome")
    parents = result["parent_bindings"]
    if (
        analyzer["parents"]["s5a0_selected_bag_receipt"]["sha256"]
        != parents["s5a0_selected_bag_receipt_sha256"]
        or analyzer["parents"]["s5a1_transfer_manifest"]["sha256"]
        != parents["s5a1_transfer_manifest_sha256"]
        or analyzer["parents"]["source_bag"]["sha256"] != parents["source_bag_sha256"]
    ):
        raise SelectedSignalBridgeError("bridge parent identity is self-contradictory")
    if analyzer["time_alignment"]["output_grid_sha256"] != result["grid_contract"]["output_grid_sha256"]:
        raise SelectedSignalBridgeError("bridge output grid binding is self-contradictory")
    if (
        analyzer["time_alignment"]["odom_header_origin_ns"]
        != source_contract["source_odom_header_origin_ns"]
        or analyzer["time_alignment"]["record_to_odom_offset_ns"]
        != source_contract["source_record_to_odom_header_offset_ns"]
    ):
        raise SelectedSignalBridgeError("bridge source time mapping is self-contradictory")
    integrity = result["integrity"]
    if integrity["analyzer_input_sha256"] != sha256_json(analyzer):
        raise SelectedSignalBridgeError("bridge analyzer input hash is self-contradictory")
    if integrity["outcome_contract_sha256"] != sha256_json(outcome):
        raise SelectedSignalBridgeError("bridge outcome contract hash is self-contradictory")
    for name in ("H_proxy", "H_modal"):
        if integrity[f"{name}_samples_sha256"] != sha256_json(analyzer["series"][name]["samples"]):
            raise SelectedSignalBridgeError(f"bridge {name} hash is self-contradictory")
    validate_target_v4_shape(analyzer)
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
    source_outcome: str = "UNKNOWN",
    source_terminal: str = "UNKNOWN",
    source_outcome_sidecar_raw: bytes | None = None,
    expected_source_outcome_sidecar_sha256: str | None = None,
    infer_outcome_from_topics: bool = False,
    infer_outcome_from_bag_duration: bool = False,
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
        "s5a0_selected_bag_receipt": legacy._sha(
            expected_s5a0_receipt_sha256, "expected S5A0 receipt"
        ),
        "s5a1_transfer_manifest": legacy._sha(
            expected_s5a1_transfer_sha256, "expected S5A1 transfer"
        ),
        "source_bag": legacy._sha(expected_source_bag_sha256, "expected source bag"),
    }
    for name, expected in parent_expected.items():
        if source["parents"][name]["sha256"] != expected:
            raise SelectedSignalBridgeError(f"{name} parent identity drift")
    if source["integrity"]["source_bag_sha256"] != parent_expected["source_bag"]:
        raise SelectedSignalBridgeError("source bag integrity/parent identity drift")
    outcome = _resolve_outcome(
        source_outcome,
        source_terminal,
        source_outcome_sidecar_raw,
        expected_source_outcome_sidecar_sha256,
        infer_outcome_from_topics=infer_outcome_from_topics,
        infer_outcome_from_bag_duration=infer_outcome_from_bag_duration,
    )
    projected = legacy._validate_source_semantics(source)
    frozen_grid, grid = legacy._validate_grid(
        output_grid_slots_s,
        expected_output_grid_sha256,
        float(source["time_alignment"]["overlap_start_s"]),
        float(source["time_alignment"]["overlap_end_s"]),
    )
    output_grid_sha = sha256_json(frozen_grid)
    analyzer_input = {
        "schema_version": "smpcc-r8-liquid-s6-real-selected-signal-input-v4-semantic-split-v2",
        "document_type": "SMPCC_R8_LIQUID_S6_REAL_SELECTED_SIGNAL_INPUT_V4_SEMANTIC_SPLIT_V2",
        "status": "S6_REAL_SELECTED_SIGNAL_SHAPE_FIXTURE_VALIDATED_NOT_ADMITTED",
        "mode": "STATIC_FIXTURE_FOR_REAL_INPUT_CONTRACT",
        "real_input_admitted": False,
        "attempt_id": source["attempt_id"],
        "planned_denominator": 1,
        "parents": {
            "s5a0_selected_bag_receipt": {
                "kind": "FINALIZED_S5A0_RECEIPT_IDENTITY_FIXTURE",
                "sha256": parent_expected["s5a0_selected_bag_receipt"],
                "real_parent_materialized": False,
            },
            "s5a1_transfer_manifest": {
                "kind": "FINALIZED_S5A1_TRANSFER_IDENTITY_FIXTURE",
                "sha256": parent_expected["s5a1_transfer_manifest"],
                "real_parent_materialized": False,
            },
            "source_bag": {
                "kind": "SOURCE_ROS1_BAG_IDENTITY_FIXTURE",
                "sha256": parent_expected["source_bag"],
                "size_bytes": source["parents"]["source_bag"]["size_bytes"],
                "real_parent_materialized": False,
            },
        },
        "provenance": {
            "target_s5a0_status": "S5A0_PRIMARY_SELECTED_BAG_ACCEPTED",
            "target_s5a1_status": "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED",
            "source_schema": SOURCE_SCHEMA,
            "source_outcome_evidence": outcome,
            "original_root_zero_write_required": True,
            "real_files_read": False,
            "motion_exporter_consumed_selected_signals": False,
            "solver_forcing_consumed_selected_signals": False,
            "comparison_only": True,
        },
        "time_alignment": {
            "x_axis": source["time_alignment"]["x_axis"],
            "sample_time_source": source["time_alignment"]["signal_native_time_source"],
            "odom_header_origin_ns": source["time_alignment"]["odom_header_origin_ns"],
            "record_to_odom_offset_ns": source["time_alignment"]["record_to_odom_header_offset_ns"],
            "overlap_start_s": grid[0],
            "overlap_end_s": grid[-1],
            "output_grid_sha256": output_grid_sha,
            "interpolation": "LINEAR_WITHIN_OVERLAP_ONLY",
            "extrapolation": False,
            "smoothing": False,
        },
        "series": {
            "H_proxy": {
                "topic": "/slosh/height", "message_type": "std_msgs/Float32",
                "native_unit": "m", "comparison_unit": "mm",
                "scale_to_comparison": 1000.0, "offset_to_comparison": 0.0,
                "samples": projected["H_proxy"],
            },
            "H_modal": {
                "topic": "/spmpc/slosh_height", "message_type": "std_msgs/Float32",
                "native_unit": "mm", "comparison_unit": "mm",
                "scale_to_comparison": 1.0, "offset_to_comparison": 0.0,
                "samples": projected["H_modal"],
            },
        },
        "claims": {
            "stage6_pass": False, "single_row": True, "paired_ranking": False,
            "cpu_selected_trajectory_comparison": False,
            "source_outcome_inferred": False,
            "physical_reference_pending": True, "physical_fidelity_validated": False,
            "formal": False, "production": False,
        },
    }
    validate_target_v4_shape(analyzer_input)
    result = {
        "schema_version": "smpcc-r8-liquid-s6a-selected-signal-v5-to-v4-bridge-v2",
        "document_type": "SMPCC_R8_LIQUID_S6A_SELECTED_SIGNAL_V5_TO_V4_BRIDGE_V2",
        "status": "S6A_V5_TO_V4_SEMANTIC_SPLIT_VALIDATED_NOT_ADMITTED",
        "mode": "FIXTURE_ONLY_REAL_INPUT_NOT_CONSUMED",
        "fixture_only": True,
        "real_input_consumed": False,
        "optional_input_consumed": False,
        "source_contract": {
            "source_v5_document_sha256": expected_source_v5_sha256,
            "source_v5_schema_sha256": legacy.SOURCE_SCHEMA_SHA256,
            "source_v5_producer_sha256": legacy.SOURCE_PRODUCER_SHA256,
            "bridge_v1_schema_sha256": BRIDGE_V1_SCHEMA_SHA256,
            "bridge_v1_script_sha256": BRIDGE_V1_SCRIPT_SHA256,
            "target_v4_schema_sha256": legacy.TARGET_SCHEMA_SHA256,
            "target_v4_analyzer_sha256": legacy.TARGET_ANALYZER_SHA256,
            "legacy_v5_field_path": "provenance.source_outcome",
            "legacy_v5_field_value": SOURCE_SCHEMA,
            "legacy_v5_field_interpretation": "SOURCE_SCHEMA_ONLY_NOT_SOURCE_OUTCOME",
            "source_schema": SOURCE_SCHEMA,
            "source_odom_header_origin_ns": source["time_alignment"]["odom_header_origin_ns"],
            "source_record_to_odom_header_offset_ns": source["time_alignment"][
                "record_to_odom_header_offset_ns"
            ],
        },
        "outcome_contract": outcome,
        "grid_contract": {
            "output_grid_sha256": output_grid_sha,
            "sample_count": len(grid),
            "start_s": grid[0],
            "end_s": grid[-1],
            "strictly_increasing": True,
            "uniform": True,
            "inside_selected_overlap": True,
        },
        "parent_bindings": {
            "s5a0_selected_bag_receipt_sha256": parent_expected["s5a0_selected_bag_receipt"],
            "s5a1_transfer_manifest_sha256": parent_expected["s5a1_transfer_manifest"],
            "source_bag_sha256": parent_expected["source_bag"],
            "s5a0_exact_attempt_sidecar_available": False,
            "s5a1_exact_attempt_sidecar_available": False,
        },
        "analyzer_input": analyzer_input,
        "integrity": {
            "analyzer_input_sha256": sha256_json(analyzer_input),
            "outcome_contract_sha256": sha256_json(outcome),
            "H_proxy_samples_sha256": sha256_json(projected["H_proxy"]),
            "H_modal_samples_sha256": sha256_json(projected["H_modal"]),
        },
        "target_v4_compatibility": {
            "time_grid_parent_series_shape_validated": True,
            "semantic_delta": "SPLIT_SOURCE_SCHEMA_FROM_SOURCE_OUTCOME",
            "legacy_v4_payload_emitted": False,
            "legacy_v4_source_outcome_field_trusted": False,
        },
        "claims": {
            "stage6_pass": False, "real_input_admitted": False,
            "real_input_consumed": False, "optional_admitted": False,
            "paired_ranking": False, "source_outcome_inferred": False,
            "source_schema_emitted_as_outcome": False,
            "physical_reference_pending": True, "physical_fidelity_validated": False,
            "formal": False, "production": False,
        },
    }
    return validate_bridge_result(result)


def synthetic_v5_fixture() -> dict[str, Any]:
    return legacy.synthetic_v5_fixture()


def self_check() -> dict[str, Any]:
    source = synthetic_v5_fixture()
    raw = canonical_json(source)
    grid = [index * 0.1 for index in range(16)]
    result = bridge_v5_to_v4(
        raw,
        grid,
        expected_source_v5_sha256=sha256_bytes(raw),
        expected_output_grid_sha256=sha256_json(grid),
        expected_s5a0_receipt_sha256="a" * 64,
        expected_s5a1_transfer_sha256="b" * 64,
        expected_source_bag_sha256="c" * 64,
        fixture_only=True,
        real_input_consumed=False,
        optional_admitted=False,
    )
    return {
        "status": "S6A_V5_TO_V4_BRIDGE_V2_SELF_CHECK_OK_NOT_ADMITTED",
        "source_schema": result["source_contract"]["source_schema"],
        "source_outcome": result["outcome_contract"]["source_outcome"],
        "source_terminal": result["outcome_contract"]["source_terminal"],
        "authoritative_sidecar_bound": result["outcome_contract"]["authoritative_sidecar_bound"],
        "real_input_status": "REAL_INPUT_NOT_CONSUMED",
        "bridge_schema_id": load_schemas()[0]["$id"],
        "target_v4_schema_id": load_schemas()[2]["$id"],
        "planned_denominator": result["analyzer_input"]["planned_denominator"],
        "sample_count": result["grid_contract"]["sample_count"],
        "real_bag_read": False,
        "real_bi4_read": False,
        "external_write_performed": False,
        "optional_admitted": False,
        "stage6_pass": False,
    }


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


__all__ = [
    "SelectedSignalBridgeError", "bridge_v5_to_v4", "validate_bridge_result",
    "validate_target_v4_shape", "authoritative_sidecar_fixture",
    "synthetic_v5_fixture", "self_check", "canonical_json", "sha256_bytes",
    "sha256_json", "load_schemas",
]
