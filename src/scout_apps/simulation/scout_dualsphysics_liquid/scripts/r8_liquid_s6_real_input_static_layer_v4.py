#!/usr/bin/env python3
"""Fixture-only static layer for the future S6 finalized real-input contract.

The public analyzer accepts two exact in-memory JSON byte strings shaped like a
finalized S5A1 selected-signal provenance document and a finalized S5B0 result
package.  v4 admits fixture mode only: it never opens a bag/BI4/artifact path,
renders media, writes evidence, appends a ledger, or claims a real S6 PASS.
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
SELECTED_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_selected_signal_input_contract_v4.json"
PACKAGE_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_s5b0_result_package_contract_v4.json"
RESULT_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_static_analysis_delivery_result_v4.json"
MAXIMUM_INPUT_BYTES = 32 * 1024 * 1024
SURFACE_NAMES = ("H_crest", "H_abs", "H_peak_to_peak")
MODEL_NAMES = ("H_proxy", "H_modal")
WINDOW_NAMES = ("first15", "full_motion", "recorded_tail", "solver_tail")
REQUIRED_INVENTORY = (
    "result_manifest.json", "executed_boundary_motion.csv", "gauge_zsurf.csv",
    "slosh_height.csv", "summary.json", "qc_report.json", "resource_samples.jsonl",
    "build_and_runtime.txt", "solver.stdout.log", "solver.stderr.log",
    "finalized_solver_frames_manifest.json", "checksums.sha256",
)


class S6RealStaticError(ValueError):
    """Static real-input-shape, provenance, metric, or claim-ceiling failure."""


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
        raise S6RealStaticError(f"schema root is not an object: {path.name}")
    return value


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise S6RealStaticError(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def load_schemas() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    schemas = tuple(_read_schema(path) for path in (
        SELECTED_SCHEMA_PATH, PACKAGE_SCHEMA_PATH, RESULT_SCHEMA_PATH
    ))
    for schema in schemas:
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    return schemas


def _decode_exact(
    raw: bytes, *, expected_sha256: str, schema: Mapping[str, Any], label: str
) -> dict[str, Any]:
    if not isinstance(raw, bytes) or not 0 < len(raw) <= MAXIMUM_INPUT_BYTES:
        raise S6RealStaticError(f"{label} bytes are absent or exceed the bound")
    if sha256_bytes(raw) != expected_sha256:
        raise S6RealStaticError(f"{label} SHA-256 differs from the frozen identity")
    try:
        value = json.loads(raw, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    except (json.JSONDecodeError, ValueError) as exc:
        raise S6RealStaticError(f"{label} JSON is invalid or non-finite") from exc
    try:
        Draft202012Validator(schema).validate(value)
    except ValidationError as exc:
        location = "/".join(str(item) for item in exc.absolute_path)
        raise S6RealStaticError(
            f"{label} schema validation failed at {location or '$'}: {exc.message}"
        ) from exc
    if value["mode"] != "STATIC_FIXTURE_FOR_REAL_INPUT_CONTRACT" or value["real_input_admitted"]:
        raise S6RealStaticError(f"{label} attempted unauthorized real-input admission")
    return value


def _uniform_dt(times: Sequence[float]) -> float:
    if len(times) < 16 or any(not math.isfinite(value) for value in times):
        raise S6RealStaticError("output grid is too short or non-finite")
    deltas = [right - left for left, right in zip(times, times[1:])]
    if any(delta <= 0 for delta in deltas):
        raise S6RealStaticError("output grid is not strictly increasing")
    dt = sum(deltas) / len(deltas)
    if max(abs(delta - dt) for delta in deltas) > max(1e-12, dt * 1e-9):
        raise S6RealStaticError("output grid is non-uniform")
    return dt


def _validate_inputs(selected: Mapping[str, Any], package: Mapping[str, Any]) -> tuple[list[float], float]:
    if selected["attempt_id"] != package["attempt_id"]:
        raise S6RealStaticError("attempt differs across selected/S5B0 inputs")
    if selected["planned_denominator"] != 1 or package["planned_denominator"] != 1:
        raise S6RealStaticError("planned denominator differs from primary-only 1")
    for key in ("s5a0_selected_bag_receipt", "s5a1_transfer_manifest", "source_bag"):
        if selected["parents"][key]["sha256"] != package["parents"][key]["sha256"]:
            raise S6RealStaticError(f"{key} hash differs across inputs")
    time_contract = package["time_contract"]
    if selected["time_alignment"]["odom_header_origin_ns"] != time_contract["odom_header_origin_ns"]:
        raise S6RealStaticError("/odom.header.stamp origin differs")
    grid = [float(value) for value in time_contract["output_slots_s"]]
    dt = _uniform_dt(grid)
    grid_sha = sha256_json(time_contract["output_slots_s"])
    window_sha = sha256_json(time_contract["windows"])
    probe_sha = sha256_json(package["probe_contract"]["probe_ids"])
    bindings = package["parents"]["s5b0_finalized_manifest"]
    checks = (
        (grid_sha, time_contract["output_grid_sha256"], "output grid"),
        (grid_sha, bindings["output_grid_sha256"], "S5B0 output grid"),
        (grid_sha, selected["time_alignment"]["output_grid_sha256"], "selected output grid"),
        (window_sha, time_contract["window_contract_sha256"], "window contract"),
        (window_sha, bindings["window_contract_sha256"], "S5B0 window contract"),
        (probe_sha, package["probe_contract"]["probe_set_sha256"], "probe set"),
        (probe_sha, bindings["probe_set_sha256"], "S5B0 probe set"),
    )
    for observed, expected, label in checks:
        if observed != expected:
            raise S6RealStaticError(f"{label} hash differs")
    inventory = tuple(package["package_contract"]["required_inventory"])
    artifacts = tuple(item["path"] for item in package["package_contract"]["artifacts"])
    if inventory != REQUIRED_INVENTORY or artifacts != REQUIRED_INVENTORY or len(set(artifacts)) != len(artifacts):
        raise S6RealStaticError("S5B0 finalized result inventory differs")
    windows = time_contract["windows"]
    first15, motion = windows["first15"], windows["full_motion"]
    recorded, solver = windows["recorded_tail"], windows["solver_tail"]
    relations = (
        (float(motion["start_s"]), grid[0], "motion start"),
        (float(first15["start_s"]), float(motion["start_s"]), "first15 start"),
        (float(first15["end_s"]), min(float(motion["end_s"]), float(motion["start_s"]) + 15.0), "first15 end"),
        (float(recorded["start_s"]), float(motion["end_s"]), "recorded-tail start"),
        (float(solver["start_s"]), float(recorded["end_s"]), "solver-tail start"),
        (float(solver["end_s"]), grid[-1], "solver-tail end"),
    )
    for observed, expected, label in relations:
        if abs(observed - expected) > 1e-12:
            raise S6RealStaticError(f"{label} violates the frozen window chain")
    return grid, dt


def _interpolate(times: Sequence[float], values: Sequence[float], query: float) -> float:
    if query < times[0] - 1e-12 or query > times[-1] + 1e-12:
        raise S6RealStaticError("comparison grid requires forbidden extrapolation")
    index = bisect.bisect_left(times, query)
    if index < len(times) and abs(times[index] - query) <= 1e-12:
        return values[index]
    if index == 0 or index == len(times):
        raise S6RealStaticError("comparison grid requires forbidden extrapolation")
    left, right = index - 1, index
    fraction = (query - times[left]) / (times[right] - times[left])
    return values[left] + fraction * (values[right] - values[left])


def _align_models(
    selected: Mapping[str, Any], grid: Sequence[float]
) -> tuple[dict[str, list[float]], float, float]:
    alignment = selected["time_alignment"]
    origin = int(alignment["odom_header_origin_ns"])
    offset = int(alignment["record_to_odom_offset_ns"])
    source: dict[str, tuple[list[float], list[float]]] = {}
    for name in MODEL_NAMES:
        spec = selected["series"][name]
        times = [(int(row["bag_record_t_ns"]) + offset - origin) / 1e9 for row in spec["samples"]]
        if any(right <= left for left, right in zip(times, times[1:])):
            raise S6RealStaticError(f"{name} time is not strict")
        values = [
            float(row["value_native"]) * float(spec["scale_to_comparison"])
            + float(spec["offset_to_comparison"])
            for row in spec["samples"]
        ]
        if any(not math.isfinite(value) for value in values):
            raise S6RealStaticError(f"{name} converted value is non-finite")
        source[name] = (times, values)
    overlap_start = max(grid[0], *(source[name][0][0] for name in MODEL_NAMES))
    overlap_end = min(grid[-1], *(source[name][0][-1] for name in MODEL_NAMES))
    if (abs(overlap_start - float(alignment["overlap_start_s"])) > 1e-12
            or abs(overlap_end - float(alignment["overlap_end_s"])) > 1e-12):
        raise S6RealStaticError("declared overlap differs")
    if overlap_start > grid[0] + 1e-12 or overlap_end < grid[-1] - 1e-12:
        raise S6RealStaticError("comparison grid requires forbidden extrapolation")
    return {
        name: [_interpolate(source[name][0], source[name][1], query) for query in grid]
        for name in MODEL_NAMES
    }, overlap_start, overlap_end


def _derive_surface(
    package: Mapping[str, Any], grid: Sequence[float]
) -> tuple[list[dict[str, Any]], int]:
    contract = package["probe_contract"]
    probe_ids = tuple(contract["probe_ids"])
    rows = package["raw_probe_zsurf"]
    if len(rows) != len(grid):
        raise S6RealStaticError("raw Gauge row count differs from output grid")
    h0 = float(contract["h0_mm"])
    derived, total_invalid = [], 0
    for expected_time, row in zip(grid, rows):
        if abs(float(row["t_s"]) - expected_time) > 1e-12:
            raise S6RealStaticError("raw Gauge time differs from output grid")
        ids = tuple(item["probe_id"] for item in row["probes"])
        if ids != probe_ids or len(set(ids)) != len(ids):
            raise S6RealStaticError("raw Gauge probe set/order differs")
        valid = [float(item["zsurf_mm"]) for item in row["probes"] if item["zsurf_mm"] is not None]
        if len(valid) < int(contract["minimum_valid_probes_per_slot"]):
            raise S6RealStaticError("raw Gauge slot has too few valid probes")
        if any(not math.isfinite(value) for value in valid):
            raise S6RealStaticError("raw Gauge contains non-finite zsurf")
        eta = [value - h0 for value in valid]
        invalid = len(probe_ids) - len(valid)
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
    ordered = sorted(values)
    position = (len(ordered) - 1) * 0.95
    left, right = math.floor(position), math.ceil(position)
    if left == right:
        return ordered[left]
    fraction = position - left
    return ordered[left] * (1.0 - fraction) + ordered[right] * fraction


def _window_metric(rows: Sequence[Mapping[str, Any]], key: str) -> dict[str, float]:
    values = [float(row[key]) for row in rows]
    peak = max(values)
    index = values.index(peak)
    return {
        "peak_mm": peak,
        "p95_mm": _quantile95(values),
        "rms_mm": math.sqrt(sum(value * value for value in values) / len(values)),
        "peak_time_s": float(rows[index]["t_s"]),
    }


def _window_summaries(
    package: Mapping[str, Any], derived: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    probe_count = len(package["probe_contract"]["probe_ids"])
    summaries: dict[str, Any] = {}
    for name in WINDOW_NAMES:
        contract = package["time_contract"]["windows"][name]
        start, end = float(contract["start_s"]), float(contract["end_s"])
        rows = [row for row in derived if start - 1e-12 <= float(row["t_s"]) <= end + 1e-12]
        if not rows:
            raise S6RealStaticError(f"{name} window is empty")
        invalid = sum(int(row["invalid_probe_count"]) for row in rows)
        summaries[name] = {
            "start_s": start, "end_s": end, "slot_count": len(rows),
            "invalid_probe_ratio": invalid / (len(rows) * probe_count),
            "H_crest": _window_metric(rows, "H_crest_mm"),
            "H_abs": _window_metric(rows, "H_abs_mm"),
            "H_peak_to_peak": _window_metric(rows, "H_peak_to_peak_mm"),
        }
    return summaries


def _wrap_phase(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _series_metrics(times: Sequence[float], values: Sequence[float], dt: float) -> dict[str, float]:
    mean = sum(values) / len(values)
    centered = [value - mean for value in values]
    best_k, best_real, best_imag, best_power = 0, 0.0, 0.0, -1.0
    for k in range(1, len(values) // 2 + 1):
        real = sum(value * math.cos(2.0 * math.pi * k * index / len(values)) for index, value in enumerate(centered))
        imag = -sum(value * math.sin(2.0 * math.pi * k * index / len(values)) for index, value in enumerate(centered))
        power = real * real + imag * imag
        if power > best_power:
            best_k, best_real, best_imag, best_power = k, real, imag, power
    peaks = [(times[index], abs(values[index])) for index in range(1, len(values) - 1)
             if abs(values[index]) >= abs(values[index - 1])
             and abs(values[index]) > abs(values[index + 1]) and abs(values[index]) > 0]
    damping = 0.0
    if len(peaks) >= 2:
        xs, ys = [item[0] for item in peaks], [math.log(item[1]) for item in peaks]
        xmean, ymean = sum(xs) / len(xs), sum(ys) / len(ys)
        denominator = sum((x - xmean) ** 2 for x in xs)
        if denominator:
            damping = -sum((x - xmean) * (y - ymean) for x, y in zip(xs, ys)) / denominator
    return {
        "amplitude_peak_mm": max(abs(value) for value in values),
        "dominant_frequency_hz": best_k / (len(values) * dt),
        "damping_rate_per_s": damping,
        "phase_rad": _wrap_phase(math.atan2(best_imag, best_real)),
    }


def _comparisons(
    grid: Sequence[float], derived: Sequence[Mapping[str, Any]],
    models: Mapping[str, Sequence[float]], dt: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    spectral = {
        name: _series_metrics(grid, [float(row[f"{name}_mm"]) for row in derived], dt)
        for name in SURFACE_NAMES
    }
    crest = [float(row["H_crest_mm"]) for row in derived]
    crest_metric = spectral["H_crest"]
    comparisons: dict[str, Any] = {}
    for name in MODEL_NAMES:
        values = [float(value) for value in models[name]]
        metric = _series_metrics(grid, values, dt)
        xmean, ymean = sum(crest) / len(crest), sum(values) / len(values)
        covariance = sum((x - xmean) * (y - ymean) for x, y in zip(crest, values))
        xvar = sum((x - xmean) ** 2 for x in crest)
        yvar = sum((y - ymean) ** 2 for y in values)
        correlation = covariance / math.sqrt(xvar * yvar) if xvar and yvar else 0.0
        comparisons[name] = {
            "sample_count": len(grid),
            "amplitude_error_mm": metric["amplitude_peak_mm"] - crest_metric["amplitude_peak_mm"],
            "frequency_error_hz": metric["dominant_frequency_hz"] - crest_metric["dominant_frequency_hz"],
            "damping_error_per_s": metric["damping_rate_per_s"] - crest_metric["damping_rate_per_s"],
            "phase_error_rad": _wrap_phase(metric["phase_rad"] - crest_metric["phase_rad"]),
            "rmse_mm": math.sqrt(sum((x - y) ** 2 for x, y in zip(crest, values)) / len(grid)),
            "correlation": max(-1.0, min(1.0, correlation)),
        }
    return spectral, comparisons


def analyze_static_fixture_bytes(
    selected_raw: bytes,
    package_raw: bytes,
    *,
    expected_selected_sha256: str,
    expected_package_sha256: str,
) -> dict[str, Any]:
    selected_schema, package_schema, result_schema = load_schemas()
    selected = _decode_exact(selected_raw, expected_sha256=expected_selected_sha256,
                             schema=selected_schema, label="selected provenance")
    package = _decode_exact(package_raw, expected_sha256=expected_package_sha256,
                            schema=package_schema, label="S5B0 result package")
    grid, dt = _validate_inputs(selected, package)
    models, overlap_start, overlap_end = _align_models(selected, grid)
    derived, invalid_count = _derive_surface(package, grid)
    windows = _window_summaries(package, derived)
    spectral, comparisons = _comparisons(grid, derived, models, dt)
    parents = package["parents"]
    probe_count = len(package["probe_contract"]["probe_ids"])
    result = {
        "schema_version": "smpcc-r8-liquid-s6-real-static-analysis-delivery-result-v4",
        "document_type": "SMPCC_R8_LIQUID_S6_REAL_STATIC_ANALYSIS_DELIVERY_RESULT_V4",
        "status": "S6_REAL_INPUT_STATIC_FIXTURE_ANALYSIS_VALIDATED_NOT_REAL_PASS",
        "mode": "STATIC_FIXTURE_FOR_REAL_INPUT_CONTRACT",
        "attempt_id": package["attempt_id"],
        "planned_denominator": 1,
        "input_bindings": {
            "selected_document_sha256": expected_selected_sha256,
            "s5b0_package_document_sha256": expected_package_sha256,
            "s5a0_receipt_sha256": parents["s5a0_selected_bag_receipt"]["sha256"],
            "source_bag_sha256": parents["source_bag"]["sha256"],
            "s5a1_transfer_sha256": parents["s5a1_transfer_manifest"]["sha256"],
            "s5b0_manifest_sha256": parents["s5b0_finalized_manifest"]["sha256"],
            "gpu_run_receipt_sha256": parents["gpu_run_receipt"]["sha256"],
            "settled_state_sha256": parents["settled_state_manifest"]["sha256"],
            "solver_config_sha256": parents["solver_config"]["sha256"],
            "geometry_sha256": parents["geometry"]["sha256"],
        },
        "alignment": {
            "x_axis": "time_since_odom_header_origin_s",
            "odom_header_origin_ns": package["time_contract"]["odom_header_origin_ns"],
            "record_to_odom_offset_ns": selected["time_alignment"]["record_to_odom_offset_ns"],
            "output_grid_sha256": package["time_contract"]["output_grid_sha256"],
            "uniform_dt_s": dt, "interpolation": "LINEAR_WITHIN_OVERLAP_ONLY",
            "overlap_start_s": overlap_start, "overlap_end_s": overlap_end,
            "extrapolation": False, "smoothing": False, "sample_count": len(grid),
        },
        "surface_derivation": {
            "source": "INLINE_FIXTURE_SHAPED_AS_FINALIZED_S5B0_RAW_GAUGE_ZSURF",
            "definitions": "eta=zsurf-h0;H_crest=max(eta);H_abs=max(abs(eta));H_peak_to_peak=max(zsurf)-min(zsurf)",
            "registered_probe_count": probe_count, "row_count": len(derived),
            "derived_rows_sha256": sha256_json(derived),
            "invalid_probe_values": invalid_count,
            "invalid_probe_ratio": invalid_count / (len(derived) * probe_count),
            "precomputed_surface_columns_consumed": False,
        },
        "window_summaries": windows,
        "spectral_metrics": spectral,
        "model_comparisons": comparisons,
        "visualization": {
            "layout": "THREE_VERTICAL_SHARED_X_PANELS", "shared_x": True,
            "axes_count": 3, "panel_y_axis_count": 1, "dual_y_axes": False,
            "panels": [
                {"panel_id": "a", "series": ["H_crest", "H_abs", "H_peak_to_peak"], "unit": "mm"},
                {"panel_id": "b", "series": ["H_proxy", "H_modal"], "unit": "mm"},
                {"panel_id": "c", "series": ["H_crest_minus_H_proxy", "H_crest_minus_H_modal"], "unit": "mm"},
            ],
            "palette": {"name": "OKABE_ITO_COLORBLIND_SAFE", "H_crest": "#0072B2",
                        "H_abs": "#E69F00", "H_peak_to_peak": "#009E73",
                        "H_proxy": "#0072B2", "H_modal": "#CC79A7",
                        "H_crest_minus_H_proxy": "#000000", "H_crest_minus_H_modal": "#E69F00"},
            "line_styles": {"H_crest": "-", "H_abs": "--", "H_peak_to_peak": ":",
                            "H_proxy": "-", "H_modal": "--",
                            "H_crest_minus_H_proxy": "-", "H_crest_minus_H_modal": "--"},
            "rainbow_colormap": False, "figure_size_inches": [7.2, 6.6],
            "dpi": 300, "font_family": "DejaVu Sans", "minimum_font_size_pt": 8,
            "pdf_fonttype": 42, "color_output": "figures/primary_shared_x_timeseries.png",
            "grayscale_output": "figures/primary_shared_x_timeseries_grayscale.png",
            "grayscale_qa": {"required": True, "checks": ["line_distinguishability", "contrast"],
                             "status": "REQUIRED_NOT_EXECUTED", "executed": False, "pass_claimed": False},
            "layout_qa": {"required": True, "checks": ["missing_glyphs", "text_clipping", "tick_overlap", "legend_occlusion", "panel_alignment"],
                          "status": "REQUIRED_NOT_EXECUTED", "executed": False, "pass_claimed": False},
        },
        "producer_interfaces": {
            "data_producer": {"status": "INTERFACE_ONLY_NOT_EXECUTED", "outputs": ["data/derived_surface_metrics.csv", "data/aligned_primary_comparison.csv", "reports/primary_summary.json"]},
            "figure_renderer": {"status": "INTERFACE_ONLY_NOT_EXECUTED", "outputs": ["figures/primary_shared_x_timeseries.png", "figures/primary_shared_x_timeseries_grayscale.png", "figures/primary_shared_x_timeseries.pdf"]},
            "animation_producer": {"status": "INTERFACE_ONLY_NOT_EXECUTED", "fact_source": "FINALIZED_SOLVER_FRAMES_ONLY", "outputs": ["animation/primary.mp4", "animation/primary_preview.gif", "keyframes/primary_t000.png"]},
            "manifest_producer": {"status": "INTERFACE_ONLY_NOT_EXECUTED", "outputs": ["comparison_manifest.json"]},
            "evidence_index_producer": {"status": "INTERFACE_ONLY_NOT_EXECUTED", "outputs": ["evidence_index.json"]},
            "secondary_ledger_producer": {"status": "INTERFACE_ONLY_NOT_EXECUTED", "outputs": ["secondary_ledger_entry.json"]},
            "checksum_producer": {"status": "INTERFACE_ONLY_NOT_EXECUTED", "algorithm": "SHA-256", "outputs": ["checksums.sha256"]},
        },
        "delivery": {
            "write_semantics": "CREATE_NEW_ONLY_AFTER_SEPARATE_REAL_INPUT_ADMISSION",
            "artifacts_materialized": False, "final_artifacts_allowed": False,
            "renderer_executed": False, "animation_executed": False,
            "evidence_written": False, "ledger_appended": False,
            "checksums_materialized": False,
            "planned_inventory": ["comparison_manifest.json", "data/derived_surface_metrics.csv",
                                  "data/aligned_primary_comparison.csv", "reports/primary_summary.json",
                                  "figures/primary_shared_x_timeseries.png",
                                  "figures/primary_shared_x_timeseries_grayscale.png",
                                  "figures/primary_shared_x_timeseries.pdf",
                                  "animation/primary.mp4", "animation/primary_preview.gif",
                                  "keyframes/primary_t000.png", "evidence_index.json",
                                  "secondary_ledger_entry.json", "checksums.sha256"],
        },
        "claims": {
            "stage6_pass": False, "single_row": True, "paired_ranking": False,
            "cross_method_ranking": False, "cpu_selected_trajectory_comparison": False,
            "cpu_selected_trajectory_scope": "OUT_OF_SCOPE",
            "physical_reference_pending": True, "physical_fidelity_validated": False,
            "formal": False, "production": False, "physical_primary": False,
        },
    }
    try:
        Draft202012Validator(result_schema).validate(result)
    except ValidationError as exc:
        raise S6RealStaticError(f"derived result schema differs: {exc.message}") from exc
    return result


def self_check() -> dict[str, Any]:
    selected, package, result = load_schemas()
    return {
        "status": "S6_REAL_INPUT_STATIC_LAYER_V4_SELF_CHECK_OK_NOT_ADMITTED",
        "selected_schema_id": selected["$id"], "package_schema_id": package["$id"],
        "result_schema_id": result["$id"], "real_input_admitted": False,
        "real_bag_read": False, "real_bi4_read": False, "gpu_or_solver_executed": False,
        "renderer_executed": False, "animation_executed": False,
        "external_write_performed": False, "stage6_pass_claimed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True))
    except (S6RealStaticError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "S6_REAL_STATIC_SELF_CHECK_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["S6RealStaticError", "analyze_static_fixture_bytes", "canonical_json",
           "load_schemas", "self_check", "sha256_bytes", "sha256_json", "_wrap_phase"]
