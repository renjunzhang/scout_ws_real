#!/usr/bin/env python3
"""Synthetic-only, byte-in/JSON-out S6 v2 pure analysis kernel.

This module never opens a bag or BI4, writes an artifact, starts ROS, encodes
media, or executes a solver/GPU command.  Exact input byte hashes are mandatory.
The v2 schemas intentionally admit only closed synthetic fixtures while the real
S5B0 finalized parent remains unavailable.
"""

from __future__ import annotations

import argparse
import bisect
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
SELECTED_SCHEMA_PATH = ROOT / "schema/target_host_s6_selected_signals_provenance_v2.json"
REPLAY_SCHEMA_PATH = ROOT / "schema/target_host_s6_finalized_replay_input_v2.json"
RESULT_SCHEMA_PATH = ROOT / "schema/target_host_s6_primary_analysis_result_v2.json"
MAXIMUM_INPUT_BYTES = 16 * 1024 * 1024
SHA_KEYS = (
    "s5a0_selected_bag_receipt",
    "s5a1_transfer_manifest",
)
SURFACE_NAMES = ("H_crest", "H_abs", "H_peak_to_peak")
MODEL_NAMES = ("H_proxy", "H_modal")
WINDOW_NAMES = ("first15", "full_motion", "recorded_tail", "solver_tail")


class S6V2AnalysisError(ValueError):
    """Closed-schema, provenance, alignment, probe, window, or metric failure."""


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def _read_schema(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise S6V2AnalysisError(f"schema root is not an object: {path.name}")
    return value


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise S6V2AnalysisError(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def load_schemas() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    schemas = tuple(_read_schema(path) for path in (
        SELECTED_SCHEMA_PATH, REPLAY_SCHEMA_PATH, RESULT_SCHEMA_PATH
    ))
    for schema in schemas:
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    return schemas


def _decode_exact(
    raw: bytes,
    *,
    expected_sha256: str,
    schema: Mapping[str, Any],
    label: str,
) -> dict[str, Any]:
    if not isinstance(raw, bytes) or not 0 < len(raw) <= MAXIMUM_INPUT_BYTES:
        raise S6V2AnalysisError(f"{label} bytes are absent or exceed the bound")
    if (not isinstance(expected_sha256, str) or len(expected_sha256) != 64
            or any(character not in "0123456789abcdef" for character in expected_sha256)):
        raise S6V2AnalysisError(f"{label} expected SHA-256 is invalid")
    if sha256_bytes(raw) != expected_sha256:
        raise S6V2AnalysisError(f"{label} SHA-256 differs from the frozen identity")
    try:
        value = json.loads(
            raw,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )
    except (json.JSONDecodeError, ValueError) as exc:
        raise S6V2AnalysisError(f"{label} JSON is invalid or non-finite") from exc
    if not isinstance(value, dict):
        raise S6V2AnalysisError(f"{label} root is not an object")
    try:
        Draft202012Validator(schema).validate(value)
    except ValidationError as exc:
        location = "/".join(str(item) for item in exc.absolute_path)
        raise S6V2AnalysisError(
            f"{label} schema validation failed at {location or '$'}: {exc.message}"
        ) from exc
    return value


def _require_equal(left: Any, right: Any, label: str) -> None:
    if left != right:
        raise S6V2AnalysisError(f"{label} differs across frozen parents")


def _uniform_dt(times: Sequence[float]) -> float:
    if len(times) < 4 or any(not math.isfinite(value) for value in times):
        raise S6V2AnalysisError("output grid is too short or non-finite")
    deltas = [right - left for left, right in zip(times, times[1:])]
    if any(delta <= 0 for delta in deltas):
        raise S6V2AnalysisError("output grid is not strictly increasing")
    dt = sum(deltas) / len(deltas)
    if max(abs(delta - dt) for delta in deltas) > max(1e-12, abs(dt) * 1e-9):
        raise S6V2AnalysisError("output grid is non-uniform")
    return dt


def _validate_contract_bindings(replay: Mapping[str, Any]) -> tuple[list[float], float]:
    time_contract = replay["time_contract"]
    probe_contract = replay["probe_contract"]
    slots = [float(value) for value in time_contract["output_slots_s"]]
    dt = _uniform_dt(slots)
    grid_sha = sha256_json(time_contract["output_slots_s"])
    windows_sha = sha256_json(time_contract["windows"])
    probes_sha = sha256_json(probe_contract["probe_ids"])
    _require_equal(grid_sha, time_contract["output_grid_sha256"], "output grid hash")
    _require_equal(windows_sha, time_contract["window_contract_sha256"], "window contract hash")
    _require_equal(probes_sha, probe_contract["probe_set_sha256"], "probe set hash")
    parent = replay["parents"]["s5b0_finalized_manifest"]
    _require_equal(grid_sha, parent["output_grid_sha256"], "S5B0 output grid hash")
    _require_equal(windows_sha, parent["window_contract_sha256"], "S5B0 window contract hash")
    _require_equal(probes_sha, parent["probe_set_sha256"], "S5B0 probe set hash")

    windows = time_contract["windows"]
    first15, motion = windows["first15"], windows["full_motion"]
    recorded, solver = windows["recorded_tail"], windows["solver_tail"]
    tolerance = 1e-12
    expected_first15_end = min(float(motion["end_s"]), float(motion["start_s"]) + 15.0)
    relations = (
        (float(motion["start_s"]), slots[0], "full-motion start"),
        (float(first15["start_s"]), float(motion["start_s"]), "first15 start"),
        (float(first15["end_s"]), expected_first15_end, "first15 end"),
        (float(recorded["start_s"]), float(motion["end_s"]), "recorded-tail start"),
        (float(solver["start_s"]), float(recorded["end_s"]), "solver-tail start"),
        (float(solver["end_s"]), slots[-1], "solver-tail end"),
    )
    for observed, expected, label in relations:
        if abs(observed - expected) > tolerance:
            raise S6V2AnalysisError(f"{label} violates the pre-registered window chain")
    for name in WINDOW_NAMES:
        window = windows[name]
        start, end = float(window["start_s"]), float(window["end_s"])
        if end < start or not any(start - tolerance <= value <= end + tolerance for value in slots):
            raise S6V2AnalysisError(f"{name} window is empty or reversed")
    return slots, dt


def _validate_cross_provenance(
    selected: Mapping[str, Any], replay: Mapping[str, Any]
) -> None:
    _require_equal(selected["attempt_id"], replay["attempt_id"], "attempt")
    _require_equal(selected["planned_denominator"], replay["planned_denominator"], "denominator")
    for key in SHA_KEYS:
        _require_equal(
            selected["parents"][key]["sha256"],
            replay["parents"][key]["sha256"],
            key,
        )
    _require_equal(selected["source"]["sha256"], replay["parents"]["source_bag"]["sha256"], "source bag")
    _require_equal(
        selected["time_alignment"]["odom_header_origin_ns"],
        replay["time_contract"]["odom_header_origin_ns"],
        "/odom.header.stamp origin",
    )
    _require_equal(
        selected["time_alignment"]["output_grid_sha256"],
        replay["time_contract"]["output_grid_sha256"],
        "selected/replay grid hash",
    )


def _aligned_model_series(
    selected: Mapping[str, Any], grid: Sequence[float]
) -> tuple[dict[str, list[float]], float, float]:
    alignment = selected["time_alignment"]
    origin = int(alignment["odom_header_origin_ns"])
    offset = int(alignment["record_to_odom_offset_ns"])
    converted: dict[str, tuple[list[float], list[float]]] = {}
    for name in MODEL_NAMES:
        spec = selected["series"][name]
        times = [
            (int(row["bag_record_t_ns"]) + offset - origin) / 1e9
            for row in spec["samples"]
        ]
        if any(right <= left for left, right in zip(times, times[1:])):
            raise S6V2AnalysisError(f"{name} time is not strictly increasing")
        scale = float(spec["scale_to_comparison"])
        unit_offset = float(spec["offset_to_comparison"])
        values = [float(row["value_native"]) * scale + unit_offset for row in spec["samples"]]
        if any(not math.isfinite(value) for value in values):
            raise S6V2AnalysisError(f"{name} converted values are non-finite")
        converted[name] = (times, values)
    overlap_start = max(grid[0], *(converted[name][0][0] for name in MODEL_NAMES))
    overlap_end = min(grid[-1], *(converted[name][0][-1] for name in MODEL_NAMES))
    if (abs(overlap_start - float(alignment["overlap_start_s"])) > 1e-12
            or abs(overlap_end - float(alignment["overlap_end_s"])) > 1e-12):
        raise S6V2AnalysisError("declared overlap differs from exact series/grid intersection")
    if overlap_start > grid[0] + 1e-12 or overlap_end < grid[-1] - 1e-12:
        raise S6V2AnalysisError("finalized replay grid requires forbidden extrapolation")
    return {
        name: [_interpolate(*converted[name], query) for query in grid]
        for name in MODEL_NAMES
    }, overlap_start, overlap_end


def _interpolate(times: Sequence[float], values: Sequence[float], query: float) -> float:
    if query < times[0] - 1e-12 or query > times[-1] + 1e-12:
        raise S6V2AnalysisError("query requires forbidden extrapolation")
    index = bisect.bisect_left(times, query)
    if index < len(times) and abs(times[index] - query) <= 1e-12:
        return float(values[index])
    if index == 0 or index == len(times):
        raise S6V2AnalysisError("query requires forbidden extrapolation")
    left, right = index - 1, index
    fraction = (query - times[left]) / (times[right] - times[left])
    return float(values[left]) + fraction * (float(values[right]) - float(values[left]))


def _derive_surface(
    replay: Mapping[str, Any], grid: Sequence[float]
) -> tuple[list[dict[str, Any]], int]:
    probe_contract = replay["probe_contract"]
    probe_ids = tuple(probe_contract["probe_ids"])
    h0 = float(probe_contract["h0_mm"])
    rows = replay["raw_probe_zsurf"]
    if len(rows) != len(grid):
        raise S6V2AnalysisError("raw probe row count differs from output grid")
    derived: list[dict[str, Any]] = []
    total_invalid = 0
    for expected_time, row in zip(grid, rows):
        if abs(float(row["t_s"]) - expected_time) > 1e-12:
            raise S6V2AnalysisError("raw probe row time differs from output grid")
        if tuple(row["zsurf_mm"]) != probe_ids:
            raise S6V2AnalysisError("raw probe row does not contain the complete registered probe set")
        valid = [
            float(row["zsurf_mm"][probe])
            for probe in probe_ids
            if row["zsurf_mm"][probe] is not None
        ]
        if any(not math.isfinite(value) for value in valid):
            raise S6V2AnalysisError("raw probe zsurf contains a non-finite value")
        invalid = len(probe_ids) - len(valid)
        if len(valid) < int(probe_contract["minimum_valid_probes_per_slot"]):
            raise S6V2AnalysisError("raw probe slot has too few valid registered probes")
        eta = [value - h0 for value in valid]
        total_invalid += invalid
        derived.append({
            "t_s": expected_time,
            "H_crest_mm": max(eta),
            "H_abs_mm": max(abs(value) for value in eta),
            "H_peak_to_peak_mm": max(valid) - min(valid),
            "valid_probe_count": len(valid),
            "invalid_probe_count": invalid,
        })
    return derived, total_invalid


def _quantile95(values: Sequence[float]) -> float:
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * 0.95
    left = math.floor(position)
    right = math.ceil(position)
    if left == right:
        return ordered[left]
    fraction = position - left
    return ordered[left] * (1.0 - fraction) + ordered[right] * fraction


def _window_metric(rows: Sequence[Mapping[str, Any]], key: str) -> dict[str, float]:
    values = [float(row[key]) for row in rows]
    peak = max(values)
    peak_index = values.index(peak)
    return {
        "peak_mm": peak,
        "p95_mm": _quantile95(values),
        "rms_mm": math.sqrt(sum(value * value for value in values) / len(values)),
        "peak_time_s": float(rows[peak_index]["t_s"]),
    }


def _window_summaries(
    replay: Mapping[str, Any], derived: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    probe_count = len(replay["probe_contract"]["probe_ids"])
    result: dict[str, Any] = {}
    for name in WINDOW_NAMES:
        contract = replay["time_contract"]["windows"][name]
        start, end = float(contract["start_s"]), float(contract["end_s"])
        rows = [row for row in derived if start - 1e-12 <= float(row["t_s"]) <= end + 1e-12]
        if not rows:
            raise S6V2AnalysisError(f"{name} window contains no finalized output slot")
        invalid = sum(int(row["invalid_probe_count"]) for row in rows)
        result[name] = {
            "start_s": start,
            "end_s": end,
            "slot_count": len(rows),
            "invalid_probe_ratio": invalid / (len(rows) * probe_count),
            "H_crest": _window_metric(rows, "H_crest_mm"),
            "H_abs": _window_metric(rows, "H_abs_mm"),
            "H_peak_to_peak": _window_metric(rows, "H_peak_to_peak_mm"),
        }
    return result


def _wrap_phase(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def series_metrics(
    times: Sequence[float], values: Sequence[float], *, uniform_dt_s: float
) -> dict[str, float]:
    if len(times) != len(values) or len(times) < 4:
        raise S6V2AnalysisError("metric series shape differs")
    if abs(_uniform_dt(times) - uniform_dt_s) > max(1e-12, uniform_dt_s * 1e-9):
        raise S6V2AnalysisError("metric series grid differs")
    if any(not math.isfinite(float(value)) for value in values):
        raise S6V2AnalysisError("metric series contains a non-finite value")
    mean = sum(float(value) for value in values) / len(values)
    centered = [float(value) - mean for value in values]
    best_k, best_real, best_imag, best_power = 0, 0.0, 0.0, -1.0
    for k in range(1, len(values) // 2 + 1):
        real = sum(value * math.cos(2.0 * math.pi * k * index / len(values))
                   for index, value in enumerate(centered))
        imag = -sum(value * math.sin(2.0 * math.pi * k * index / len(values))
                    for index, value in enumerate(centered))
        power = real * real + imag * imag
        if power > best_power:
            best_k, best_real, best_imag, best_power = k, real, imag, power
    peaks = [
        (float(times[index]), abs(float(values[index])))
        for index in range(1, len(values) - 1)
        if (abs(float(values[index])) >= abs(float(values[index - 1]))
            and abs(float(values[index])) > abs(float(values[index + 1]))
            and abs(float(values[index])) > 0)
    ]
    damping = 0.0
    if len(peaks) >= 2:
        xs = [item[0] for item in peaks]
        ys = [math.log(item[1]) for item in peaks]
        xmean, ymean = sum(xs) / len(xs), sum(ys) / len(ys)
        denominator = sum((x - xmean) ** 2 for x in xs)
        if denominator:
            damping = -sum((x - xmean) * (y - ymean) for x, y in zip(xs, ys)) / denominator
    return {
        "amplitude_peak_mm": max(abs(float(value)) for value in values),
        "dominant_frequency_hz": best_k / (len(values) * uniform_dt_s),
        "damping_rate_per_s": damping,
        "phase_rad": _wrap_phase(math.atan2(best_imag, best_real)),
    }


def _model_comparisons(
    grid: Sequence[float],
    derived: Sequence[Mapping[str, Any]],
    models: Mapping[str, Sequence[float]],
    dt: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    surface = {
        name: series_metrics(
            grid,
            [float(row[f"{name}_mm"]) for row in derived],
            uniform_dt_s=dt,
        )
        for name in SURFACE_NAMES
    }
    crest = [float(row["H_crest_mm"]) for row in derived]
    crest_metrics = surface["H_crest"]
    comparisons: dict[str, Any] = {}
    for name in MODEL_NAMES:
        values = [float(value) for value in models[name]]
        metrics = series_metrics(grid, values, uniform_dt_s=dt)
        xmean, ymean = sum(crest) / len(crest), sum(values) / len(values)
        covariance = sum((x - xmean) * (y - ymean) for x, y in zip(crest, values))
        xvar = sum((x - xmean) ** 2 for x in crest)
        yvar = sum((y - ymean) ** 2 for y in values)
        correlation = covariance / math.sqrt(xvar * yvar) if xvar and yvar else 0.0
        comparisons[name] = {
            "sample_count": len(grid),
            "amplitude_error_mm": metrics["amplitude_peak_mm"] - crest_metrics["amplitude_peak_mm"],
            "frequency_error_hz": metrics["dominant_frequency_hz"] - crest_metrics["dominant_frequency_hz"],
            "damping_error_per_s": metrics["damping_rate_per_s"] - crest_metrics["damping_rate_per_s"],
            "phase_error_rad": _wrap_phase(metrics["phase_rad"] - crest_metrics["phase_rad"]),
            "rmse_mm": math.sqrt(sum((x - y) ** 2 for x, y in zip(crest, values)) / len(grid)),
            "correlation": max(-1.0, min(1.0, correlation)),
        }
    return surface, comparisons


def analyze_bytes(
    selected_raw: bytes,
    replay_raw: bytes,
    *,
    expected_selected_sha256: str,
    expected_replay_sha256: str,
) -> dict[str, Any]:
    selected_schema, replay_schema, result_schema = load_schemas()
    selected = _decode_exact(
        selected_raw,
        expected_sha256=expected_selected_sha256,
        schema=selected_schema,
        label="selected signals",
    )
    replay = _decode_exact(
        replay_raw,
        expected_sha256=expected_replay_sha256,
        schema=replay_schema,
        label="finalized replay",
    )
    _validate_cross_provenance(selected, replay)
    grid, dt = _validate_contract_bindings(replay)
    models, overlap_start, overlap_end = _aligned_model_series(selected, grid)
    derived, invalid_count = _derive_surface(replay, grid)
    windows = _window_summaries(replay, derived)
    spectral, comparisons = _model_comparisons(grid, derived, models, dt)
    parent = replay["parents"]
    result = {
        "schema_version": "smpcc-r8-liquid-s6-primary-analysis-result-v2",
        "document_type": "SMPCC_R8_LIQUID_S6_PRIMARY_ANALYSIS_RESULT_V2",
        "status": "S6_PRIMARY_SYNTHETIC_PURE_ANALYSIS_VALIDATED_NOT_FINAL",
        "attempt_id": replay["attempt_id"],
        "planned_denominator": 1,
        "parent_bindings": {
            "selected_signals_document_sha256": expected_selected_sha256,
            "finalized_replay_document_sha256": expected_replay_sha256,
            "s5a0_selected_bag_receipt_sha256": parent["s5a0_selected_bag_receipt"]["sha256"],
            "source_bag_sha256": parent["source_bag"]["sha256"],
            "s5a1_transfer_manifest_sha256": parent["s5a1_transfer_manifest"]["sha256"],
            "s5b0_finalized_manifest_sha256": parent["s5b0_finalized_manifest"]["sha256"],
            "gpu_run_receipt_sha256": parent["gpu_run_receipt"]["sha256"],
            "settled_state_manifest_sha256": parent["settled_state_manifest"]["sha256"],
            "solver_config_sha256": parent["solver_config"]["sha256"],
            "geometry_sha256": parent["geometry"]["sha256"],
            "finalized_bi4_sha256": replay["numeric_sources"]["finalized_bi4"]["sha256"],
            "gauge_zsurf_sha256": replay["numeric_sources"]["gauge_zsurf"]["sha256"],
        },
        "analysis_method": {
            "surface_source": "RAW_PROBE_ZSURF_MINUS_FROZEN_H0",
            "surface_definitions": "H_crest=max(eta);H_abs=max(abs(eta));H_peak_to_peak=max(zsurf)-min(zsurf)",
            "peak_definition": "MAX_VALUE_EARLIEST_TIE",
            "p95_method": "LINEAR_INTERPOLATED_ORDER_STATISTIC",
            "rms_method": "SQRT_MEAN_SQUARE",
            "frequency_method": "MEAN_CENTERED_DFT_DOMINANT_NONZERO_BIN",
            "damping_method": "NEGATIVE_LOG_ABS_LOCAL_PEAK_SLOPE",
            "phase_method": "DOMINANT_DFT_PHASE_WRAPPED_TO_MINUS_PI_PI",
        },
        "alignment": {
            "x_axis": "time_since_odom_header_origin_s",
            "sample_time_source": "ROS1_BAG_RECORD_TIME_NS",
            "odom_header_origin_ns": replay["time_contract"]["odom_header_origin_ns"],
            "record_to_odom_offset_ns": selected["time_alignment"]["record_to_odom_offset_ns"],
            "grid_source": "FINALIZED_REPLAY_OUTPUT_SLOTS",
            "output_grid_sha256": replay["time_contract"]["output_grid_sha256"],
            "uniform_dt_s": dt,
            "interpolation": "LINEAR_WITHIN_OVERLAP_ONLY",
            "overlap_start_s": overlap_start,
            "overlap_end_s": overlap_end,
            "extrapolation": False,
            "smoothing": False,
            "sample_count": len(grid),
        },
        "derived_surface_series": derived,
        "window_summaries": windows,
        "spectral_metrics": spectral,
        "model_comparisons": comparisons,
        "qc": {
            "registered_probe_count": len(replay["probe_contract"]["probe_ids"]),
            "output_slot_count": len(grid),
            "total_probe_values": len(grid) * len(replay["probe_contract"]["probe_ids"]),
            "invalid_probe_values": invalid_count,
            "invalid_probe_ratio": invalid_count / (len(grid) * len(replay["probe_contract"]["probe_ids"])),
            "minimum_valid_probes_per_slot": replay["probe_contract"]["minimum_valid_probes_per_slot"],
            "precomputed_surface_columns_consumed": False,
            "window_cherry_pick_detected": False,
            "probe_cherry_pick_detected": False,
            "nonfinite": False,
            "pass": True,
        },
        "materialization": {
            "status": "NOT_ADMITTED_SYNTHETIC_FIXTURE_ONLY",
            "final_outputs_generated": False,
            "animation_executed": False,
            "external_write_performed": False,
        },
        "claims": {
            "stage6_pass": False,
            "paired_ranking": False,
            "cpu_selected_trajectory_comparison": False,
            "physical_reference_pending": True,
            "physical_fidelity_validated": False,
            "formal": False,
            "production": False,
            "physical_primary": False,
        },
    }
    try:
        Draft202012Validator(result_schema).validate(result)
    except ValidationError as exc:
        location = "/".join(str(item) for item in exc.absolute_path)
        raise S6V2AnalysisError(
            f"derived result schema validation failed at {location or '$'}: {exc.message}"
        ) from exc
    return result


def self_check() -> dict[str, Any]:
    selected_schema, replay_schema, result_schema = load_schemas()
    return {
        "status": "S6_V2_PURE_ANALYZER_STATIC_SELF_CHECK_OK_NOT_ADMITTED",
        "selected_schema_id": selected_schema["$id"],
        "replay_schema_id": replay_schema["$id"],
        "result_schema_id": result_schema["$id"],
        "real_bag_read": False,
        "real_bi4_read": False,
        "solver_or_gpu_executed": False,
        "animation_executed": False,
        "external_write_performed": False,
        "final_materialization_admitted": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True))
    except (S6V2AnalysisError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "S6_V2_SELF_CHECK_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "S6V2AnalysisError",
    "analyze_bytes",
    "canonical_json",
    "load_schemas",
    "self_check",
    "series_metrics",
    "sha256_bytes",
    "sha256_json",
    "_wrap_phase",
]
