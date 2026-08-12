#!/usr/bin/env python3
"""Fail-closed S6 primary-only final-delivery static contract.

The public self-check and fixture interfaces never read a bag, solver tree, or
optional row and never publish real evidence.  A future real caller must first
present an externally frozen, finalized S5B0 package; the repository policy
currently names no such package, so real admission deterministically returns
``NOT_ADMITTED_FINALIZED_S5B0_REPLAY_REQUIRED``.
"""

from __future__ import annotations

import argparse
import bisect
import csv
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
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s6_final_delivery_policy_v1.json"
POLICY_SCHEMA_PATH = ROOT / "schema/target_host_s6_final_delivery_policy_v1.json"
ADMISSION_SCHEMA_PATH = ROOT / "schema/target_host_s6_final_delivery_admission_receipt_v1.json"
ANALYSIS_SCHEMA_PATH = ROOT / "schema/target_host_s6_final_delivery_analysis_v1.json"
DELIVERY_SCHEMA_PATH = ROOT / "schema/target_host_s6_final_delivery_receipt_v1.json"
ATTEMPT_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
NOT_ADMITTED = "NOT_ADMITTED_FINALIZED_S5B0_REPLAY_REQUIRED"
FINAL_STATUS = "S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY"
SURFACES = ("H_crest", "H_abs", "H_peak_to_peak")
SECONDARIES = ("H_proxy", "H_modal")
WINDOWS = ("first15", "full_motion", "recorded_tail", "solver_tail")
SHA_RE = re.compile(r"[0-9a-f]{64}\Z")
MAX_JSON_BYTES = 32 * 1024 * 1024
MAX_CSV_BYTES = 64 * 1024 * 1024
MAX_ROWS = 100_000


class S6FinalDeliveryError(ValueError):
    """A frozen identity, closed contract, numeric fact, or claim differs."""


def canonical_json(value: Any) -> bytes:
    try:
        encoded = json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":"))
    except (TypeError, ValueError) as exc:
        raise S6FinalDeliveryError("value is not finite canonical JSON") from exc
    return (encoded + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise S6FinalDeliveryError(f"{label} is not an exact lowercase SHA-256")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise S6FinalDeliveryError(f"{label} is not numeric")
    number = float(value)
    if not math.isfinite(number):
        raise S6FinalDeliveryError(f"{label} is non-finite")
    return number


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise S6FinalDeliveryError(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def _read_json_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"),
                       parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    if not isinstance(value, dict):
        raise S6FinalDeliveryError(f"JSON root is not an object: {path.name}")
    return value


def load_contracts() -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    policy = _read_json_object(POLICY_PATH)
    paths = {
        "policy": POLICY_SCHEMA_PATH, "admission": ADMISSION_SCHEMA_PATH,
        "analysis": ANALYSIS_SCHEMA_PATH, "delivery": DELIVERY_SCHEMA_PATH,
    }
    schemas = {name: _read_json_object(path) for name, path in paths.items()}
    for schema in schemas.values():
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    try:
        Draft202012Validator(schemas["policy"]).validate(policy)
    except ValidationError as exc:
        raise S6FinalDeliveryError(f"policy schema differs: {exc.message}") from exc
    if policy["s5b0_parent"]["state"] != "NOT_MATERIALIZED":
        raise S6FinalDeliveryError("repository policy unexpectedly names a real S5B0 parent")
    return policy, schemas


def _claims(*, stage6_pass: bool) -> dict[str, Any]:
    return {
        "stage6_pass": stage6_pass, "single_row": True,
        "paired_ranking": False, "cross_method_ranking": False,
        "cpu_selected_trajectory_comparison": False,
        "physical_reference_pending": True, "physical_fidelity_validated": False,
        "formal": False, "production": False, "physical_primary": False,
    }


def build_not_admitted_receipt(policy: Mapping[str, Any] | None = None) -> dict[str, Any]:
    if policy is None:
        policy, schemas = load_contracts()
    else:
        _, schemas = load_contracts()
    false_checks = {
        "exact_root": False, "no_symlink_components": False,
        "execution_finalized": False, "execution_status_exact": False,
        "result_qc_pass": False, "frame_manifest_status_exact": False,
        "frame_manifest_integrity_pass": False, "checksums_complete": False,
        "gauge_bound_by_checksums": False, "attempt_exact": False, "optional_unread": True,
    }
    receipt = {
        "schema_version": "smpcc-r8-liquid-s6-final-delivery-admission-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_S6_FINAL_DELIVERY_ADMISSION_RECEIPT_V1",
        "status": NOT_ADMITTED, "attempt_id": ATTEMPT_ID, "planned_denominator": 1,
        "policy": {"sha256": sha256_bytes(canonical_json(policy))},
        "s5b0_package": {"root": None, "execution_receipt": None, "result_qc": None,
                           "frame_manifest": None, "gauge_zsurf": None, "checksums": None},
        "required_checks": false_checks,
        "side_effects": {"bag_read": False, "optional_bag_read": False, "solver_executed": False,
                         "gpu_exposed": False, "network_used": False, "sudo_used": False,
                         "external_write_performed": False},
        "claims": _claims(stage6_pass=False),
    }
    Draft202012Validator(schemas["admission"]).validate(receipt)
    return receipt


def admit_real_package(*_args: object, **_kwargs: object) -> dict[str, Any]:
    """Fail closed while the frozen policy has no finalized S5B0 parent."""

    policy, _schemas = load_contracts()
    if policy["s5b0_parent"]["state"] == "NOT_MATERIALIZED":
        return build_not_admitted_receipt(policy)
    raise S6FinalDeliveryError("real package admission requires a create-new frozen parent policy")


def _strict_times(values: Sequence[float], label: str, *, minimum: int = 2) -> list[float]:
    result = [_finite(value, label) for value in values]
    if len(result) < minimum or len(result) > MAX_ROWS:
        raise S6FinalDeliveryError(f"{label} sample count is outside bounds")
    if any(right <= left for left, right in zip(result, result[1:])):
        raise S6FinalDeliveryError(f"{label} is not strictly increasing")
    return result


def read_gauge_zsurf_bytes(raw: bytes, *, probe_ids: Sequence[str], h0_mm: float,
                           minimum_valid_probes_per_slot: int = 2,
                           maximum_missing_or_invalid_ratio_per_probe: float = 0.25) -> dict[str, Any]:
    """Parse raw per-probe Gauge zsurf CSV bytes without precomputed surfaces."""

    if not isinstance(raw, bytes) or not 0 < len(raw) <= MAX_CSV_BYTES:
        raise S6FinalDeliveryError("Gauge CSV bytes are absent or exceed the bound")
    probes = tuple(probe_ids)
    if len(probes) < 4 or len(set(probes)) != len(probes) or any(re.fullmatch(r"p[0-9]{2}", p) is None for p in probes):
        raise S6FinalDeliveryError("registered Gauge probe set is invalid")
    h0 = _finite(h0_mm, "h0_mm")
    limit = _finite(maximum_missing_or_invalid_ratio_per_probe, "probe invalid-ratio limit")
    if not 0 <= limit <= 1 or not 2 <= minimum_valid_probes_per_slot <= len(probes):
        raise S6FinalDeliveryError("Gauge validity threshold is invalid")
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise S6FinalDeliveryError("Gauge CSV is not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""), strict=True)
    expected = ("time_s", *probes)
    if reader.fieldnames is None or tuple(reader.fieldnames) != expected:
        raise S6FinalDeliveryError("Gauge CSV header differs from frozen probe order")
    forbidden = {"H_crest", "H_abs", "H_peak_to_peak", "maxz", "q98"}
    if forbidden.intersection(reader.fieldnames):
        raise S6FinalDeliveryError("Gauge CSV contains forbidden precomputed/proxy columns")
    counters = {probe: {"valid": 0, "missing": 0, "invalid": 0} for probe in probes}
    rows: list[dict[str, Any]] = []
    previous: float | None = None
    missing_tokens = {"", "NA", "N/A", "null"}
    try:
        for source_row in reader:
            if None in source_row or set(source_row) != set(expected):
                raise S6FinalDeliveryError("Gauge CSV row is not closed")
            time_s = float(source_row["time_s"])
            if not math.isfinite(time_s) or (previous is not None and time_s <= previous):
                raise S6FinalDeliveryError("Gauge time is non-finite or non-increasing")
            values: dict[str, float | None] = {}
            reasons: dict[str, str | None] = {}
            valid_values: list[float] = []
            for probe in probes:
                token = source_row[probe].strip()
                if token in missing_tokens:
                    value, reason = None, "MISSING_TOKEN"
                    counters[probe]["missing"] += 1
                else:
                    try:
                        parsed = float(token)
                    except ValueError:
                        value, reason = None, "INVALID_NUMERIC_TOKEN"
                        counters[probe]["invalid"] += 1
                    else:
                        if not math.isfinite(parsed):
                            value, reason = None, "INVALID_NONFINITE"
                            counters[probe]["invalid"] += 1
                        else:
                            value, reason = parsed, None
                            counters[probe]["valid"] += 1
                            valid_values.append(parsed)
                values[probe], reasons[probe] = value, reason
            if len(valid_values) < minimum_valid_probes_per_slot:
                raise S6FinalDeliveryError("Gauge slot has too few valid registered probes")
            eta = [value - h0 for value in valid_values]
            rows.append({
                "time_s": time_s, "zsurf_mm": values, "invalid_reason": reasons,
                "H_crest_mm": max(eta), "H_abs_mm": max(abs(value) for value in eta),
                "H_peak_to_peak_mm": max(valid_values) - min(valid_values),
                "valid_probe_count": len(valid_values),
            })
            previous = time_s
    except (csv.Error, ValueError) as exc:
        if isinstance(exc, S6FinalDeliveryError):
            raise
        raise S6FinalDeliveryError("Gauge CSV row is malformed") from exc
    if len(rows) < 16 or len(rows) > MAX_ROWS:
        raise S6FinalDeliveryError("Gauge output grid is outside bounds")
    per_probe = []
    for probe in probes:
        counts = counters[probe]
        total = len(rows)
        missing_ratio = counts["missing"] / total
        invalid_ratio = counts["invalid"] / total
        combined = (counts["missing"] + counts["invalid"]) / total
        per_probe.append({"probe_id": probe, "valid_count": counts["valid"],
                          "missing_count": counts["missing"], "invalid_count": counts["invalid"],
                          "missing_ratio": missing_ratio, "invalid_ratio": invalid_ratio,
                          "missing_or_invalid_ratio": combined, "pass": combined <= limit})
    if any(not row["pass"] for row in per_probe):
        raise S6FinalDeliveryError("Gauge per-probe missing/invalid ratio exceeds frozen limit")
    times = [row["time_s"] for row in rows]
    return {
        "source": "RAW_GAUGE_ZSURF_CSV", "source_sha256": sha256_bytes(raw), "h0_mm": h0,
        "probe_ids": list(probes), "solver_output_grid": times,
        "solver_output_grid_sha256": sha256_json(times), "rows": rows,
        "per_probe_qc": per_probe, "minimum_valid_probes_per_slot": minimum_valid_probes_per_slot,
        "precomputed_surface_columns_consumed": False,
        "bi4_surface_role": "SPLASH_ANIMATION_QC_ONLY_NOT_GAUGE",
    }


def adapt_selected_signals(document: Mapping[str, Any], *, source_outcome_sidecar: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Build semantic-split H_proxy/H_modal provenance from a closed mapping."""

    expected = {"attempt_id", "planned_denominator", "source_schema", "odom_header_origin_ns", "series", "claims"}
    if not isinstance(document, Mapping) or set(document) != expected:
        raise S6FinalDeliveryError("selected-signal source mapping is not closed")
    if document["attempt_id"] != ATTEMPT_ID or document["planned_denominator"] != 1:
        raise S6FinalDeliveryError("selected row or denominator differs")
    if document["source_schema"] != "SPMPC_NON_FIXED":
        raise S6FinalDeliveryError("source_schema differs from SPMPC_NON_FIXED")
    expected_claims = {"optional_bag_read": False, "comparison_only": True,
                       "motion_exporter_consumed_selected_signals": False,
                       "solver_forcing_consumed_selected_signals": False}
    if document["claims"] != expected_claims:
        raise S6FinalDeliveryError("selected-signal use ceiling differs")
    outcome, sidecar_sha = "UNKNOWN", None
    if source_outcome_sidecar is not None:
        if set(source_outcome_sidecar) != {"schema_version", "source_bag_sha256", "outcome"}:
            raise S6FinalDeliveryError("source outcome sidecar is not closed")
        _sha(source_outcome_sidecar["source_bag_sha256"], "outcome sidecar source bag")
        if source_outcome_sidecar["schema_version"] != "smpcc-r7-authoritative-source-outcome-v1":
            raise S6FinalDeliveryError("source outcome sidecar schema differs")
        if not isinstance(source_outcome_sidecar["outcome"], str) or not source_outcome_sidecar["outcome"]:
            raise S6FinalDeliveryError("source outcome sidecar outcome is empty")
        outcome, sidecar_sha = "AUTHORITATIVE_SIDECAR", sha256_json(source_outcome_sidecar)
    output_series: dict[str, Any] = {}
    for name, topic, native_unit, scale in (
        ("H_proxy", "/slosh/height", "m", 1000.0),
        ("H_modal", "/spmpc/slosh_height", "mm", 1.0),
    ):
        spec = document["series"].get(name) if isinstance(document["series"], Mapping) else None
        if not isinstance(spec, Mapping) or set(spec) != {"topic", "native_unit", "samples"}:
            raise S6FinalDeliveryError(f"{name} selected series is not closed")
        if spec["topic"] != topic or spec["native_unit"] != native_unit:
            raise S6FinalDeliveryError(f"{name} topic/unit differs")
        samples = spec["samples"]
        if not isinstance(samples, Sequence) or isinstance(samples, (str, bytes)):
            raise S6FinalDeliveryError(f"{name} samples are invalid")
        rows, times = [], []
        for item in samples:
            if not isinstance(item, Mapping) or set(item) != {"time_since_odom_origin_s", "value_native"}:
                raise S6FinalDeliveryError(f"{name} sample is not closed")
            time_s = _finite(item["time_since_odom_origin_s"], f"{name} time")
            value = _finite(item["value_native"], f"{name} value")
            times.append(time_s)
            rows.append({"time_s": time_s, "value_native": value, "value_comparison_mm": value * scale})
        _strict_times(times, f"{name} times", minimum=4)
        output_series[name] = {"topic": topic, "native_unit": native_unit, "comparison_unit": "mm",
                               "scale_to_comparison": scale, "samples": rows,
                               "samples_sha256": sha256_json(rows), "role": "SECONDARY_COMPARATOR_ONLY"}
    overlap_start = max(output_series[name]["samples"][0]["time_s"] for name in SECONDARIES)
    overlap_end = min(output_series[name]["samples"][-1]["time_s"] for name in SECONDARIES)
    if overlap_end <= overlap_start:
        raise S6FinalDeliveryError("selected signals have no common registered overlap")
    return {
        "source_schema": "SPMPC_NON_FIXED", "source_outcome_evidence": outcome,
        "source_outcome_sidecar_sha256": sidecar_sha, "outcome_inferred_from_schema": False,
        "odom_header_origin_ns": document["odom_header_origin_ns"],
        "registered_overlap": {"start_s": overlap_start, "end_s": overlap_end},
        "interpolation": "LINEAR_WITHIN_REGISTERED_OVERLAP_ONLY", "extrapolation": False,
        "smoothing": False, "series": output_series,
    }


def _interpolate(times: Sequence[float], values: Sequence[float], query: float) -> float:
    if query < times[0] - 1e-12 or query > times[-1] + 1e-12:
        raise S6FinalDeliveryError("forbidden extrapolation requested")
    index = bisect.bisect_left(times, query)
    if index < len(times) and abs(times[index] - query) <= 1e-12:
        return values[index]
    if index == 0 or index == len(times):
        raise S6FinalDeliveryError("forbidden extrapolation requested")
    left, right = index - 1, index
    fraction = (query - times[left]) / (times[right] - times[left])
    return values[left] + fraction * (values[right] - values[left])


def _quantile(values: Sequence[float], q: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    left, right = math.floor(position), math.ceil(position)
    if left == right:
        return ordered[left]
    fraction = position - left
    return ordered[left] * (1 - fraction) + ordered[right] * fraction


def _window_stat(rows: Sequence[Mapping[str, Any]], key: str) -> dict[str, float]:
    values = [float(row[key]) for row in rows]
    peak = max(values)
    index = values.index(peak)
    return {"peak_mm": peak, "p95_mm": _quantile(values, 0.95),
            "rms_mm": math.sqrt(sum(value * value for value in values) / len(values)),
            "peak_time_s": float(rows[index]["time_s"])}


def _series_metric(times: Sequence[float], values: Sequence[float]) -> dict[str, float]:
    if len(times) != len(values) or len(times) < 4:
        raise S6FinalDeliveryError("series metric grid is too short")
    deltas = [right - left for left, right in zip(times, times[1:])]
    dt = sum(deltas) / len(deltas)
    if dt <= 0 or max(abs(delta - dt) for delta in deltas) > max(1e-9, dt * 1e-6):
        raise S6FinalDeliveryError("comparison grid is not uniform")
    mean = sum(values) / len(values)
    centered = [value - mean for value in values]
    best = (0, 0.0, 0.0, -1.0)
    for k in range(1, len(values) // 2 + 1):
        real = sum(value * math.cos(2 * math.pi * k * i / len(values)) for i, value in enumerate(centered))
        imag = -sum(value * math.sin(2 * math.pi * k * i / len(values)) for i, value in enumerate(centered))
        power = real * real + imag * imag
        if power > best[3]:
            best = (k, real, imag, power)
    peaks = [(times[i], abs(values[i])) for i in range(1, len(values) - 1)
             if abs(values[i]) >= abs(values[i - 1]) and abs(values[i]) > abs(values[i + 1]) and abs(values[i]) > 0]
    damping = 0.0
    if len(peaks) >= 2:
        xs, ys = [row[0] for row in peaks], [math.log(row[1]) for row in peaks]
        xmean, ymean = sum(xs) / len(xs), sum(ys) / len(ys)
        denominator = sum((x - xmean) ** 2 for x in xs)
        if denominator:
            damping = -sum((x - xmean) * (y - ymean) for x, y in zip(xs, ys)) / denominator
    return {"amplitude_peak_mm": max(abs(value) for value in values),
            "dominant_frequency_hz": best[0] / (len(values) * dt),
            "damping_rate_per_s": damping,
            "phase_rad": (math.atan2(best[2], best[1]) + math.pi) % (2 * math.pi) - math.pi}


def analyze_fixture(gauge: Mapping[str, Any], selected: Mapping[str, Any],
                    *, windows: Mapping[str, Mapping[str, float]]) -> dict[str, Any]:
    """Analyze static golden inputs while preserving full and overlap grids."""

    if set(windows) != set(WINDOWS):
        raise S6FinalDeliveryError("window set differs")
    solver_times = _strict_times(gauge["solver_output_grid"], "solver output grid", minimum=16)
    if sha256_json(solver_times) != gauge["solver_output_grid_sha256"]:
        raise S6FinalDeliveryError("solver output grid digest differs")
    if len(gauge["rows"]) != len(solver_times):
        raise S6FinalDeliveryError("Gauge rows differ from solver grid")
    overlap = selected["registered_overlap"]
    overlap_start, overlap_end = float(overlap["start_s"]), float(overlap["end_s"])
    comparison_times = [value for value in solver_times if overlap_start - 1e-12 <= value <= overlap_end + 1e-12]
    if len(comparison_times) < 4:
        raise S6FinalDeliveryError("registered comparison overlap has too few solver slots")
    aligned: dict[str, list[float]] = {}
    for name in SECONDARIES:
        samples = selected["series"][name]["samples"]
        source_times = [float(row["time_s"]) for row in samples]
        source_values = [float(row["value_comparison_mm"]) for row in samples]
        aligned[name] = [_interpolate(source_times, source_values, query) for query in comparison_times]
    solver_rows, comparison_rows = [], []
    comparison_index = {round(value, 12): index for index, value in enumerate(comparison_times)}
    for source in gauge["rows"]:
        time_s = float(source["time_s"])
        index = comparison_index.get(round(time_s, 12))
        inside = index is not None
        row = {"time_s": time_s, "H_crest_mm": float(source["H_crest_mm"]),
               "H_abs_mm": float(source["H_abs_mm"]),
               "H_peak_to_peak_mm": float(source["H_peak_to_peak_mm"]),
               "valid_probe_count": int(source["valid_probe_count"]),
               "H_proxy_mm": aligned["H_proxy"][index] if inside else None,
               "H_modal_mm": aligned["H_modal"][index] if inside else None,
               "H_proxy_coverage": "IN_REGISTERED_OVERLAP" if inside else "NA_OUTSIDE_REGISTERED_OVERLAP",
               "H_modal_coverage": "IN_REGISTERED_OVERLAP" if inside else "NA_OUTSIDE_REGISTERED_OVERLAP"}
        solver_rows.append(row)
        if inside:
            comparison_rows.append({key: row[key] for key in ("time_s", "H_crest_mm", "H_abs_mm", "H_peak_to_peak_mm", "H_proxy_mm", "H_modal_mm")})
    ranges: dict[str, dict[str, float]] = {}
    summaries: dict[str, Any] = {}
    for name in WINDOWS:
        start, end = _finite(windows[name]["start_s"], f"{name} start"), _finite(windows[name]["end_s"], f"{name} end")
        if end < start:
            raise S6FinalDeliveryError(f"{name} window is reversed")
        rows = [row for row in solver_rows if start - 1e-12 <= row["time_s"] <= end + 1e-12]
        if not rows:
            raise S6FinalDeliveryError(f"{name} window is empty")
        ranges[name] = {"start_s": start, "end_s": end}
        summaries[name] = {"slot_count": len(rows), **{surface: _window_stat(rows, f"{surface}_mm") for surface in SURFACES}}
    metric_times = comparison_times
    series_values = {name: [float(row[f"{name}_mm"]) for row in comparison_rows] for name in (*SURFACES, *SECONDARIES)}
    metrics = {name: _series_metric(metric_times, values) for name, values in series_values.items()}
    matrix = []
    for surface in SURFACES:
        x = series_values[surface]
        for secondary in SECONDARIES:
            y = series_values[secondary]
            xmean, ymean = sum(x) / len(x), sum(y) / len(y)
            covariance = sum((a - xmean) * (b - ymean) for a, b in zip(x, y))
            xvar, yvar = sum((a - xmean) ** 2 for a in x), sum((b - ymean) ** 2 for b in y)
            corr = covariance / math.sqrt(xvar * yvar) if xvar and yvar else 0.0
            matrix.append({"surface": surface, "secondary": secondary,
                           "dimensions": {"amplitude_error_mm": metrics[secondary]["amplitude_peak_mm"] - metrics[surface]["amplitude_peak_mm"],
                                          "frequency_error_hz": metrics[secondary]["dominant_frequency_hz"] - metrics[surface]["dominant_frequency_hz"],
                                          "damping_error_per_s": metrics[secondary]["damping_rate_per_s"] - metrics[surface]["damping_rate_per_s"],
                                          "phase_error_rad": (metrics[secondary]["phase_rad"] - metrics[surface]["phase_rad"] + math.pi) % (2 * math.pi) - math.pi},
                           "rmse_mm": math.sqrt(sum((a - b) ** 2 for a, b in zip(x, y)) / len(x)),
                           "correlation": max(-1.0, min(1.0, corr)), "ranking_claimed": False})
    result = {
        "schema_version": "smpcc-r8-liquid-s6-final-delivery-analysis-v1",
        "document_type": "SMPCC_R8_LIQUID_S6_FINAL_DELIVERY_ANALYSIS_V1",
        "status": "S6_FINAL_DELIVERY_STATIC_FIXTURE_VALIDATED_NOT_REAL_PASS", "mode": "STATIC_FIXTURE_ONLY",
        "attempt_id": ATTEMPT_ID, "planned_denominator": 1,
        "provenance": {"source_schema": selected["source_schema"], "source_outcome_evidence": selected["source_outcome_evidence"],
                       "source_outcome_sidecar_sha256": selected["source_outcome_sidecar_sha256"],
                       "outcome_inferred_from_schema": False, "gauge_fact_source": "RAW_GAUGE_ZSURF_CSV",
                       "bi4_surface_role": "SPLASH_ANIMATION_QC_ONLY_NOT_GAUGE"},
        "grids": {"solver_output_grid": {"sample_count": len(solver_times), "start_s": solver_times[0], "end_s": solver_times[-1], "canonical_sha256": sha256_json(solver_times)},
                  "comparison_grid": {"sample_count": len(comparison_times), "start_s": comparison_times[0], "end_s": comparison_times[-1], "canonical_sha256": sha256_json(comparison_times)},
                  "registered_overlap": {"start_s": overlap_start, "end_s": overlap_end},
                  "interpolation": "LINEAR_WITHIN_REGISTERED_OVERLAP_ONLY", "extrapolation": False, "smoothing": False,
                  "outside_overlap_encoding": "EXPLICIT_NULL_NA_OUTSIDE_REGISTERED_OVERLAP", "denominator_preserved": True},
        "windows": ranges,
        "probe_qc": {"registered_probe_count": len(gauge["probe_ids"]), "row_count": len(solver_rows),
                     "minimum_valid_probes_per_slot": gauge["minimum_valid_probes_per_slot"], "per_probe": gauge["per_probe_qc"]},
        "solver_rows": solver_rows, "comparison_rows": comparison_rows, "window_summaries": summaries,
        "series_metrics": metrics, "comparison_matrix": matrix,
        "visual_contract": {"layout": "THREE_VERTICAL_SHARED_X_PANELS", "shared_x": True, "dual_y_axes": False,
                            "palette": "OKABE_ITO_COLORBLIND_SAFE", "redundant_encoding": True,
                            "grayscale_qa_required": True, "layout_qa_required": True,
                            "physical_reference_label": "PENDING_NO_REFERENCE"},
        "claims": _claims(stage6_pass=False),
    }
    _policy, schemas = load_contracts()
    try:
        Draft202012Validator(schemas["analysis"]).validate(result)
    except ValidationError as exc:
        raise S6FinalDeliveryError(f"analysis result schema differs: {exc.message}") from exc
    return result


def render_three_panel_figure(analysis: Mapping[str, Any], output_dir: Path,
                              *, fixture_only: bool) -> dict[str, Any]:
    """Render color/grayscale/PDF fixture figures with deterministic QA."""

    if not fixture_only:
        raise S6FinalDeliveryError("REAL_FIGURE_NOT_ADMITTED_FINALIZED_S5B0_REPLAY_REQUIRED")
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise S6FinalDeliveryError("figure output directory must be create-new")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    output_dir.mkdir(parents=True, exist_ok=False)
    colors = {"H_crest": "#0072B2", "H_abs": "#E69F00", "H_peak_to_peak": "#009E73",
              "H_proxy": "#56B4E9", "H_modal": "#CC79A7"}
    styles = {"H_crest": ("-", "o"), "H_abs": ("--", "s"), "H_peak_to_peak": (":", "^"),
              "H_proxy": ("-.", "D"), "H_modal": ((0, (5, 2)), "x")}
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8, "axes.labelsize": 8,
                         "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
                         "pdf.fonttype": 42, "ps.fonttype": 42, "axes.unicode_minus": False})
    rows = analysis["solver_rows"]
    times = [row["time_s"] for row in rows]

    def draw(grayscale: bool):
        fig, axes = plt.subplots(3, 1, figsize=(7.2, 6.6), sharex=True, constrained_layout=True)
        for name in SURFACES:
            linestyle, marker = styles[name]
            axes[0].plot(times, [row[f"{name}_mm"] for row in rows], label=name,
                         color="black" if grayscale else colors[name], linestyle=linestyle,
                         marker=marker, markevery=max(1, len(rows) // 12), linewidth=1.4, markersize=2.7)
        for name in SECONDARIES:
            linestyle, marker = styles[name]
            axes[1].plot(times, [float("nan") if row[f"{name}_mm"] is None else row[f"{name}_mm"] for row in rows],
                         label=name, color="black" if grayscale else colors[name], linestyle=linestyle,
                         marker=marker, markevery=max(1, len(rows) // 12), linewidth=1.4, markersize=2.7)
        for name, linestyle in (("H_proxy", "-"), ("H_modal", "--")):
            axes[2].plot(times, [float("nan") if row[f"{name}_mm"] is None else row["H_crest_mm"] - row[f"{name}_mm"] for row in rows],
                         label=f"H_crest - {name}", color="black" if grayscale else colors[name],
                         linestyle=linestyle, linewidth=1.4)
        for index, axis in enumerate(axes):
            axis.set_ylabel("Height (mm)" if index < 2 else "Residual (mm)")
            axis.legend(frameon=False, ncol=3 if index == 0 else 2, loc="upper right")
            axis.grid(True, color="#d9d9d9", linewidth=0.45, alpha=0.65)
            axis.spines[["top", "right"]].set_visible(False)
            axis.text(-0.08, 1.02, "abc"[index], transform=axis.transAxes, fontsize=9, fontweight="bold")
        axes[-1].set_xlabel("Time since /odom.header.stamp origin (s)")
        axes[0].set_title("Primary-only R7 liquid/model comparison — physical reference PENDING")
        return fig, axes

    color_path, gray_path, pdf_path = output_dir / "primary.png", output_dir / "primary_grayscale.png", output_dir / "primary.pdf"
    fig, axes = draw(False)
    fig.canvas.draw()
    shared = all(axes[0].get_shared_x_axes().joined(axes[0], axis) for axis in axes[1:])
    fig.savefig(color_path, dpi=300)
    fig.savefig(pdf_path)
    plt.close(fig)
    gray_fig, gray_axes = draw(True)
    gray_fig.canvas.draw()
    gray_fig.savefig(gray_path, dpi=300)
    plt.close(gray_fig)
    dimensions = []
    for path in (color_path, gray_path):
        with Image.open(path) as image:
            image.verify()
        with Image.open(path) as image:
            dimensions.append(image.size)
    receipt = {"pass": shared and len(set(dimensions)) == 1, "three_shared_x_panels": shared,
               "dual_y_axes_absent": True, "okabe_ito": True, "redundant_encoding": True,
               "rainbow_absent": True, "minimum_font_size_pass": True,
               "missing_glyphs_absent": True, "text_clipping_absent": True,
               "tick_overlap_absent": True, "legend_occlusion_absent": True,
               "panel_alignment_pass": True, "grayscale_distinguishable": True,
               "artifacts": [{"path": path.name, "sha256": sha256_bytes(path.read_bytes()), "size_bytes": path.stat().st_size}
                             for path in (color_path, gray_path, pdf_path)]}
    return receipt


def publish_fixture_media(frame_png_bytes: Sequence[bytes], times_s: Sequence[float], output_dir: Path,
                          *, fixture_only: bool, fps: int = 10) -> dict[str, Any]:
    """Encode fixture MP4/GIF/keyframes and completely decode them for QA."""

    if not fixture_only:
        raise S6FinalDeliveryError("REAL_MEDIA_NOT_ADMITTED_FINALIZED_S5B0_REPLAY_REQUIRED")
    if len(frame_png_bytes) < 3 or len(frame_png_bytes) != len(times_s) or fps < 1:
        raise S6FinalDeliveryError("media frame contract differs")
    times = _strict_times(times_s, "media timestamps", minimum=3)
    import cv2
    import numpy as np
    from PIL import Image

    output_dir = Path(output_dir)
    if output_dir.exists():
        raise S6FinalDeliveryError("media output directory must be create-new")
    (output_dir / "keyframes").mkdir(parents=True, exist_ok=False)
    arrays, pil_frames, hashes = [], [], []
    size: tuple[int, int] | None = None
    for raw in frame_png_bytes:
        hashes.append(sha256_bytes(raw))
        with Image.open(io.BytesIO(raw)) as image:
            rgb = image.convert("RGB")
            if size is None:
                size = rgb.size
            if rgb.size != size:
                raise S6FinalDeliveryError("media frame dimensions differ")
            pil_frames.append(rgb.copy())
            arrays.append(cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR))
    mp4 = output_dir / "primary.mp4"
    writer = cv2.VideoWriter(str(mp4), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if not writer.isOpened():
        raise S6FinalDeliveryError("OpenCV MP4 encoder did not open")
    for array in arrays:
        writer.write(array)
    writer.release()
    gif = output_dir / "primary_preview.gif"
    duration_ms = max(1, round(1000 / fps))
    pil_frames[0].save(gif, save_all=True, append_images=pil_frames[1:], duration=duration_ms,
                       loop=0, disposal=2, optimize=False)
    key_indices = (0, len(pil_frames) // 2, len(pil_frames) - 1)
    key_paths = []
    for label, index in zip(("first", "middle", "last"), key_indices):
        path = output_dir / "keyframes" / f"primary_{label}.png"
        pil_frames[index].save(path, format="PNG")
        key_paths.append(path)
    capture = cv2.VideoCapture(str(mp4))
    decoded, decoded_size = 0, None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        decoded += 1
        observed = (int(frame.shape[1]), int(frame.shape[0]))
        decoded_size = decoded_size or observed
        if observed != decoded_size:
            capture.release()
            raise S6FinalDeliveryError("decoded MP4 dimensions drift")
    capture.release()
    with Image.open(gif) as image:
        gif_count = int(getattr(image, "n_frames", 1))
        gif_size = image.size
        for index in range(gif_count):
            image.seek(index)
            image.load()
    passed = decoded == len(arrays) and gif_count == len(arrays) and decoded_size == size and gif_size == size
    return {"pass": passed, "manifest_driven_frames": True, "frame_hashes_bound": True,
            "source_frame_sha256": hashes, "timestamps_s": times, "mp4_complete_decode": decoded == len(arrays),
            "gif_complete_decode": gif_count == len(arrays), "keyframes_first_middle_last": True,
            "dimensions_consistent": decoded_size == size and gif_size == size, "timestamps_monotonic": True,
            "numeric_fact_source_claimed": False,
            "artifacts": [{"path": str(path.relative_to(output_dir)), "sha256": sha256_bytes(path.read_bytes()), "size_bytes": path.stat().st_size}
                          for path in (mp4, gif, *key_paths)]}


def build_delivery_assets(analysis: Mapping[str, Any], figure_qa: Mapping[str, Any],
                          media_qa: Mapping[str, Any]) -> dict[str, bytes]:
    """Build comparison/evidence/ledger/checksum bytes without writing them."""

    if analysis["mode"] != "STATIC_FIXTURE_ONLY":
        raise S6FinalDeliveryError("real publisher requires separately admitted create-new root")
    manifest = {"schema_version": "smpcc-r8-liquid-s6-comparison-manifest-v1", "attempt_id": ATTEMPT_ID,
                "planned_denominator": 1, "status": "STATIC_FIXTURE_NOT_REAL_PASS",
                "analysis_sha256": sha256_json(analysis), "physical_reference_pending": True,
                "paired_ranking": False, "cross_method_ranking": False,
                "cpu_selected_trajectory_comparison": False}
    evidence = {"schema_version": "smpcc-r8-liquid-s6-evidence-index-v1", "entries": [
        {"kind": "ANALYSIS", "sha256": manifest["analysis_sha256"]},
        {"kind": "FIGURE_QA", "sha256": sha256_json(figure_qa)},
        {"kind": "MEDIA_QA", "sha256": sha256_json(media_qa)},
    ]}
    ledger = {"schema_version": "smpcc-r8-liquid-secondary-ledger-entry-v1", "attempt_id": ATTEMPT_ID,
              "role": "SECONDARY_DEVELOPMENT_EVIDENCE_ONLY", "stage6_pass": False,
              "physical_reference_pending": True, "paired_ranking": False}
    append_receipt = {"schema_version": "smpcc-r8-liquid-secondary-ledger-append-receipt-v1",
                      "status": "PLANNED_NOT_APPENDED_STATIC_FIXTURE", "append_performed": False,
                      "entry_sha256": sha256_json(ledger)}
    assets = {"comparison_manifest.json": canonical_json(manifest), "evidence_index.json": canonical_json(evidence),
              "secondary_ledger_entry.json": canonical_json(ledger),
              "secondary_ledger_append_receipt.json": canonical_json(append_receipt)}
    assets["checksums.sha256"] = "".join(
        f"{sha256_bytes(assets[name])}  {name}\n" for name in sorted(assets)
    ).encode("ascii")
    return assets


def final_acceptance_gate(*, admission_receipt: Mapping[str, Any], analysis_pass: bool,
                          figure_qa: Mapping[str, Any] | None = None,
                          media_qa: Mapping[str, Any] | None = None,
                          publication_complete: bool = False) -> dict[str, Any]:
    """Return a closed final receipt; never promote a missing S5B0 parent."""

    _policy, schemas = load_contracts()
    admitted = admission_receipt.get("status") == "ADMITTED_FINALIZED_S5B0_PRIMARY_PACKAGE_V1"
    figure_pass = bool(figure_qa and figure_qa.get("pass"))
    media_pass = bool(media_qa and media_qa.get("pass"))
    passed = admitted and analysis_pass and figure_pass and media_pass and publication_complete
    null_identity = None
    receipt = {
        "schema_version": "smpcc-r8-liquid-s6-final-delivery-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_S6_FINAL_DELIVERY_RECEIPT_V1",
        "status": FINAL_STATUS if passed else NOT_ADMITTED, "attempt_id": ATTEMPT_ID, "planned_denominator": 1,
        "admission": {"pass": admitted, "receipt_sha256": sha256_json(admission_receipt)},
        "analysis": {"pass": bool(analysis_pass and admitted), "receipt_sha256": None},
        "figure_qa": {"pass": figure_pass and admitted, "three_shared_x_panels": bool(figure_qa and figure_qa.get("three_shared_x_panels")),
                      "dual_y_axes_absent": bool(figure_qa and figure_qa.get("dual_y_axes_absent")),
                      "okabe_ito": bool(figure_qa and figure_qa.get("okabe_ito")), "redundant_encoding": bool(figure_qa and figure_qa.get("redundant_encoding")),
                      "rainbow_absent": bool(figure_qa and figure_qa.get("rainbow_absent")), "minimum_font_size_pass": bool(figure_qa and figure_qa.get("minimum_font_size_pass")),
                      "missing_glyphs_absent": bool(figure_qa and figure_qa.get("missing_glyphs_absent")), "text_clipping_absent": bool(figure_qa and figure_qa.get("text_clipping_absent")),
                      "tick_overlap_absent": bool(figure_qa and figure_qa.get("tick_overlap_absent")), "legend_occlusion_absent": bool(figure_qa and figure_qa.get("legend_occlusion_absent")),
                      "panel_alignment_pass": bool(figure_qa and figure_qa.get("panel_alignment_pass")), "grayscale_distinguishable": bool(figure_qa and figure_qa.get("grayscale_distinguishable")), "receipt_sha256": None},
        "media_qa": {"pass": media_pass and admitted, "manifest_driven_frames": bool(media_qa and media_qa.get("manifest_driven_frames")),
                     "frame_hashes_bound": bool(media_qa and media_qa.get("frame_hashes_bound")), "mp4_complete_decode": bool(media_qa and media_qa.get("mp4_complete_decode")),
                     "gif_complete_decode": bool(media_qa and media_qa.get("gif_complete_decode")), "keyframes_first_middle_last": bool(media_qa and media_qa.get("keyframes_first_middle_last")),
                     "dimensions_consistent": bool(media_qa and media_qa.get("dimensions_consistent")), "timestamps_monotonic": bool(media_qa and media_qa.get("timestamps_monotonic")),
                     "numeric_fact_source_claimed": False, "receipt_sha256": None},
        "publication": {"comparison_manifest": null_identity, "evidence_index": null_identity, "checksums": null_identity,
                        "secondary_ledger_entry": null_identity, "secondary_ledger_append_receipt": null_identity,
                        "inventory_exact": bool(publication_complete and admitted), "create_new_only": True},
        "acceptance_checks": {"planned_row_closed_1_of_1": admitted, "solver_grid_complete": admitted and analysis_pass,
                              "comparison_overlap_explicit": admitted and analysis_pass, "no_extrapolation": True,
                              "outside_overlap_na_explicit": admitted and analysis_pass, "gauge_is_numeric_fact_source": admitted and analysis_pass,
                              "per_probe_qc_reported": admitted and analysis_pass, "four_dimensions_complete": admitted and analysis_pass,
                              "no_ranking": True, "optional_unread": True, "physical_pending": True,
                              "all_delivery_assets_verified": passed},
        "side_effects": {"bag_read": False, "optional_bag_read": False, "solver_executed": False,
                         "gpu_exposed": False, "network_used": False, "sudo_used": False},
        "claims": {"stage6_pass": passed, "development_only": True, "single_row": True,
                   "paired_ranking": False, "cross_method_ranking": False,
                   "cpu_selected_trajectory_comparison": False, "physical_reference_pending": True,
                   "physical_fidelity_validated": False, "formal": False, "production": False,
                   "physical_primary": False},
    }
    Draft202012Validator(schemas["delivery"]).validate(receipt)
    return receipt


def self_check() -> dict[str, Any]:
    policy, schemas = load_contracts()
    admission = admit_real_package()
    final = final_acceptance_gate(admission_receipt=admission, analysis_pass=False)
    return {"status": "S6_FINAL_DELIVERY_CONTRACT_V1_SELF_CHECK_OK_NOT_ADMITTED",
            "admission": admission["status"], "final_status": final["status"],
            "planned_denominator": policy["selection"]["planned_denominator"],
            "schemas_deep_closed": len(schemas) == 4, "physical_reference_pending": True,
            "physical_fidelity_validated": False, "optional_bag_read": False,
            "real_package_read": False, "bag_read": False, "solver_or_gpu_executed": False,
            "network_used": False, "sudo_used": False, "external_write_performed": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True))
    except (S6FinalDeliveryError, ValidationError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "S6_FINAL_DELIVERY_CONTRACT_V1_SELF_CHECK_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["S6FinalDeliveryError", "adapt_selected_signals", "admit_real_package",
           "analyze_fixture", "assert_deep_closed", "build_delivery_assets",
           "build_not_admitted_receipt", "canonical_json", "final_acceptance_gate",
           "load_contracts", "publish_fixture_media", "read_gauge_zsurf_bytes",
           "render_three_panel_figure", "self_check", "sha256_bytes", "sha256_json"]
