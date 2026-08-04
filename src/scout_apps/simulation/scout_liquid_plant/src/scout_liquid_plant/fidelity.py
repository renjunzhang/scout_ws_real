"""Offline, fail-closed development fidelity checks for ``scout_liquid_plant``.

The development plant publishes an *unvalidated* surrogate under
``/sim_truth``.  This module deliberately keeps the associated comparison
work offline: it reads exported signal files only and never imports ROS,
Gazebo, a planner, or the controller's liquid model.

It is intentionally impossible for this module to issue a formal PASS.  A
complete numerical comparison can be useful for development, but its report
always remains ``formal=false``, ``development_only=true`` and
``physical_primary_eligible=false``.  A future formal release needs a
separate reviewed verifier and the independent freeze/capability evidence.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


TOOL_ID = "SMPCC-SIM-LIQUID-PLANT-FIDELITY-VERIFY-DEVELOPMENT-v1"
REPORT_TYPE = "SMPCC_SIM_LIQUID_PLANT_FIDELITY_VALIDATION"
REPORT_SCHEMA_VERSION = "smpcc-sim-liquid-plant-fidelity-validation-v1"
COMPARISON_MANIFEST_SCHEMA_VERSION = "smpcc-sim-liquid-plant-fidelity-comparison-manifest-v1"
THRESHOLD_POLICY_SCHEMA_VERSION = "smpcc-sim-liquid-plant-fidelity-threshold-policy-v1"
REFERENCE_EVIDENCE_SCHEMA_VERSION = "smpcc-real-h0-liquid-reference-evidence-v1"
SIGNAL_SCHEMA_VERSION = "smpcc-liquid-plant-signal-v1"

SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
DIMENSIONS = ("amplitude", "frequency", "damping", "phase", "ranking")
DIMENSION_STATUSES = frozenset(("PASS", "FAIL", "NOT_EVALUATED"))
REPORT_STATUSES = frozenset(
    (
        "NO_GO",
        "DEVELOPMENT_METRICS_COMPLETE_NOT_FORMAL",
        "DEVELOPMENT_METRICS_FAILED_NOT_FORMAL",
    )
)
PLANT_HASH_KEYS = (
    "plant_code_hash",
    "plant_parameter_hash",
    "plant_input_schema_hash",
    "plant_output_schema_hash",
)
FORBIDDEN_REFERENCE_TOKENS = (
    "/slosh/height",
    "/spmpc/slosh_height",
    "/sim_truth/",
    "h_proxy",
    "h_modal",
    "liquidsloshmodel",
)


class FidelityValidationError(ValueError):
    """Input or evidence failure that must result in a non-formal NO-GO."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class SignalSeries:
    """A strictly monotonic scalar time series in metres."""

    path: Path
    sha256: str
    times_sec: Tuple[float, ...]
    values_m: Tuple[float, ...]


@dataclass(frozen=True)
class ValidatedCase:
    """One fully hash-bound development comparison pair."""

    case_id: str
    plant: SignalSeries
    reference: SignalSeries
    reference_evidence_path: Path
    reference_evidence_hash: str


def is_sha256(value: Any) -> bool:
    return isinstance(value, str) and bool(SHA256_RE.fullmatch(value))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FidelityValidationError("INVALID_NUMERIC_VALUE", "{} must be a finite number".format(label))
    result = float(value)
    if not math.isfinite(result):
        raise FidelityValidationError("INVALID_NUMERIC_VALUE", "{} must be a finite number".format(label))
    return result


def _signal_number(value: Any, label: str) -> float:
    """Parse numeric CSV cells while keeping JSON control fields typed strictly."""

    if isinstance(value, str):
        try:
            value = float(value.strip())
        except ValueError as exc:
            raise FidelityValidationError("INVALID_NUMERIC_VALUE", "{} must be a finite number".format(label)) from exc
    return _finite(value, label)


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise FidelityValidationError("MALFORMED_INPUT", "{} must be a non-empty string".format(label))
    return value.strip()


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise FidelityValidationError("MALFORMED_INPUT", "{} must be a JSON object".format(label))
    return value


def _absolute_existing_file(value: Any, label: str) -> Path:
    raw = _nonempty_string(value, label)
    path = Path(raw)
    if not path.is_absolute():
        raise FidelityValidationError("MALFORMED_INPUT", "{} path must be absolute: {}".format(label, path))
    if not path.is_file():
        raise FidelityValidationError("MISSING_ARTIFACT", "{} file is missing: {}".format(label, path))
    return path.resolve()


def _bound_file(value: Any, expected_hash: Any, label: str) -> Tuple[Path, str]:
    path = _absolute_existing_file(value, label)
    if not is_sha256(expected_hash):
        raise FidelityValidationError("MISSING_HASH", "{} requires a lowercase SHA-256".format(label))
    actual = sha256_file(path)
    if actual != expected_hash:
        raise FidelityValidationError(
            "HASH_MISMATCH",
            "{} SHA-256 mismatch for {}".format(label, path),
        )
    return path, actual


def _read_json(path: Path, label: str) -> Any:
    try:
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise FidelityValidationError("MALFORMED_JSON", "cannot read {}: {}".format(label, exc)) from exc


def _bound_json(value: Any, expected_hash: Any, label: str) -> Tuple[Path, Mapping[str, Any], str]:
    path, actual_hash = _bound_file(value, expected_hash, label)
    return path, _mapping(_read_json(path, label), label), actual_hash


def _iter_strings(value: Any) -> Iterable[str]:
    if isinstance(value, str):
        yield value
    elif isinstance(value, Mapping):
        for key, item in value.items():
            yield str(key)
            yield from _iter_strings(item)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for item in value:
            yield from _iter_strings(item)


def _contains_forbidden_reference_token(value: Any) -> Optional[str]:
    for text in _iter_strings(value):
        lower = text.strip().lower()
        for token in FORBIDDEN_REFERENCE_TOKENS:
            if token in lower:
                return token
    return None


def _error(code: str, message: str) -> Dict[str, str]:
    return {"code": code, "message": message}


def _artifact_descriptor(path_value: Any, expected_hash: Any, actual_hash: Optional[str] = None) -> Dict[str, Optional[str]]:
    return {
        "path": str(path_value) if isinstance(path_value, str) else None,
        "expected_sha256": expected_hash if is_sha256(expected_hash) else None,
        "actual_sha256": actual_hash if is_sha256(actual_hash) else None,
    }


def _validate_manifest_base(manifest: Mapping[str, Any]) -> Dict[str, Any]:
    if manifest.get("schema_version") != COMPARISON_MANIFEST_SCHEMA_VERSION:
        raise FidelityValidationError(
            "MALFORMED_MANIFEST",
            "comparison manifest schema_version must be {}".format(COMPARISON_MANIFEST_SCHEMA_VERSION),
        )
    if manifest.get("development_only") is not True or manifest.get("formal") is not False:
        raise FidelityValidationError(
            "NONDEVELOPMENT_MANIFEST",
            "comparison manifest must explicitly remain development_only=true and formal=false",
        )
    comparison_id = _nonempty_string(manifest.get("comparison_id"), "comparison_id")
    identity = _mapping(manifest.get("plant_identity"), "plant_identity")
    normalized_identity: Dict[str, str] = {}
    for key in PLANT_HASH_KEYS:
        value = identity.get(key)
        if not is_sha256(value):
            raise FidelityValidationError(
                "MALFORMED_MANIFEST",
                "plant_identity.{} must be a lowercase SHA-256".format(key),
            )
        normalized_identity[key] = value
    cases = manifest.get("cases")
    if not isinstance(cases, Sequence) or isinstance(cases, (str, bytes)) or not cases:
        raise FidelityValidationError("MALFORMED_MANIFEST", "comparison manifest requires a non-empty cases array")
    return {
        "comparison_id": comparison_id,
        "plant_identity": normalized_identity,
        "cases": list(cases),
    }


def _positive_policy_number(value: Any, label: str, *, allow_zero: bool = True) -> float:
    result = _finite(value, label)
    if result < 0.0 or (not allow_zero and result <= 0.0):
        comparator = "non-negative" if allow_zero else "positive"
        raise FidelityValidationError("MALFORMED_POLICY", "{} must be {}".format(label, comparator))
    return result


def validate_threshold_policy(policy: Mapping[str, Any]) -> Dict[str, Any]:
    """Validate explicit metric tolerances; hidden defaults are prohibited."""

    if policy.get("schema_version") != THRESHOLD_POLICY_SCHEMA_VERSION:
        raise FidelityValidationError(
            "MALFORMED_POLICY",
            "threshold policy schema_version must be {}".format(THRESHOLD_POLICY_SCHEMA_VERSION),
        )
    if policy.get("development_only") is not True or policy.get("formal") is not False:
        raise FidelityValidationError(
            "NONDEVELOPMENT_POLICY",
            "threshold policy must explicitly remain development_only=true and formal=false",
        )
    policy_id = _nonempty_string(policy.get("policy_id"), "policy_id")
    amplitude = _mapping(policy.get("amplitude"), "policy.amplitude")
    frequency = _mapping(policy.get("frequency"), "policy.frequency")
    damping = _mapping(policy.get("damping"), "policy.damping")
    phase = _mapping(policy.get("phase"), "policy.phase")
    ranking = _mapping(policy.get("ranking"), "policy.ranking")
    if ranking.get("metric") != "amplitude":
        raise FidelityValidationError("MALFORMED_POLICY", "policy.ranking.metric must be amplitude")
    minimum_cases = ranking.get("minimum_cases")
    if isinstance(minimum_cases, bool) or not isinstance(minimum_cases, int) or minimum_cases < 2:
        raise FidelityValidationError("MALFORMED_POLICY", "policy.ranking.minimum_cases must be an integer >= 2")
    if ranking.get("require_exact_order") is not True:
        raise FidelityValidationError("MALFORMED_POLICY", "policy.ranking.require_exact_order must be true")
    return {
        "policy_id": policy_id,
        "amplitude_relative_error_max": _positive_policy_number(
            amplitude.get("relative_error_max"), "policy.amplitude.relative_error_max"
        ),
        "frequency_relative_error_max": _positive_policy_number(
            frequency.get("relative_error_max"), "policy.frequency.relative_error_max"
        ),
        "damping_absolute_error_max": _positive_policy_number(
            damping.get("absolute_error_max"), "policy.damping.absolute_error_max"
        ),
        "phase_absolute_error_deg_max": _positive_policy_number(
            phase.get("absolute_error_deg_max"), "policy.phase.absolute_error_deg_max"
        ),
        "ranking_metric": "amplitude",
        "ranking_minimum_cases": minimum_cases,
        "ranking_tie_relative_tolerance": _positive_policy_number(
            ranking.get("tie_relative_tolerance"), "policy.ranking.tie_relative_tolerance"
        ),
    }


def _parse_json_signal(path: Path) -> Sequence[Any]:
    value = _read_json(path, "signal")
    if isinstance(value, Mapping):
        schema_version = value.get("schema_version")
        if schema_version is not None and schema_version != SIGNAL_SCHEMA_VERSION:
            raise FidelityValidationError(
                "MALFORMED_SIGNAL",
                "JSON signal schema_version must be {} when present".format(SIGNAL_SCHEMA_VERSION),
            )
        samples = value.get("samples")
    else:
        samples = value
    if not isinstance(samples, Sequence) or isinstance(samples, (str, bytes)):
        raise FidelityValidationError("MALFORMED_SIGNAL", "JSON signal requires a samples array")
    return samples


def _parse_csv_signal(path: Path) -> Sequence[Any]:
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None or "time_sec" not in reader.fieldnames or "value" not in reader.fieldnames:
                raise FidelityValidationError("MALFORMED_SIGNAL", "CSV signal requires time_sec,value headers")
            return list(reader)
    except (OSError, UnicodeDecodeError, csv.Error) as exc:
        raise FidelityValidationError("MALFORMED_SIGNAL", "cannot read CSV signal {}: {}".format(path, exc)) from exc


def load_signal(path: Path, expected_hash: str, label: str) -> SignalSeries:
    """Load one CSV or JSON scalar signal after its exact file hash is checked."""

    actual_hash = sha256_file(path)
    if actual_hash != expected_hash:
        raise FidelityValidationError("HASH_MISMATCH", "{} SHA-256 mismatch for {}".format(label, path))
    suffix = path.suffix.lower()
    if suffix == ".csv":
        samples = _parse_csv_signal(path)
    elif suffix == ".json":
        samples = _parse_json_signal(path)
    else:
        raise FidelityValidationError("MALFORMED_SIGNAL", "{} must be a .csv or .json file".format(label))
    if len(samples) < 16:
        raise FidelityValidationError("INSUFFICIENT_SIGNAL", "{} requires at least 16 samples".format(label))
    times: List[float] = []
    values: List[float] = []
    for index, sample in enumerate(samples):
        row = _mapping(sample, "{} samples[{}]".format(label, index))
        time_sec = _signal_number(row.get("time_sec"), "{} samples[{}].time_sec".format(label, index))
        value_m = _signal_number(row.get("value"), "{} samples[{}].value".format(label, index))
        if times and time_sec <= times[-1]:
            raise FidelityValidationError(
                "MALFORMED_SIGNAL",
                "{} times must be strictly increasing at sample {}".format(label, index),
            )
        times.append(time_sec)
        values.append(value_m)
    if times[-1] - times[0] <= 0.0:
        raise FidelityValidationError("MALFORMED_SIGNAL", "{} has no positive time span".format(label))
    return SignalSeries(path=path, sha256=actual_hash, times_sec=tuple(times), values_m=tuple(values))


def _validate_reference_evidence(
    path_value: Any,
    expected_hash: Any,
    reference_signal_hash: str,
    case_id: str,
) -> Tuple[Path, str]:
    if path_value is None or expected_hash is None:
        raise FidelityValidationError(
            "MISSING_FROZEN_REAL_REFERENCE",
            "case {} has no hash-bound frozen real reference evidence".format(case_id),
        )
    path, evidence, actual_hash = _bound_json(path_value, expected_hash, "reference evidence for {}".format(case_id))
    if evidence.get("schema_version") != REFERENCE_EVIDENCE_SCHEMA_VERSION:
        raise FidelityValidationError(
            "MALFORMED_REFERENCE_EVIDENCE",
            "reference evidence for {} has wrong schema_version".format(case_id),
        )
    if evidence.get("status") != "FROZEN" or evidence.get("real_measurement") is not True:
        raise FidelityValidationError(
            "MISSING_FROZEN_REAL_REFERENCE",
            "reference evidence for {} is not a frozen real measurement".format(case_id),
        )
    if evidence.get("measurement_independent_of_plant") is not True:
        raise FidelityValidationError(
            "MISSING_FROZEN_REAL_REFERENCE",
            "reference evidence for {} is not independent of the plant".format(case_id),
        )
    if evidence.get("reference_kind") not in (
        "REAL_RGB_LIQUID_HEIGHT",
        "REAL_LIQUID_HEIGHT_SENSOR",
    ):
        raise FidelityValidationError(
            "MISSING_FROZEN_REAL_REFERENCE",
            "reference evidence for {} does not identify a supported real liquid measurement".format(case_id),
        )
    if evidence.get("case_id") != case_id:
        raise FidelityValidationError(
            "MALFORMED_REFERENCE_EVIDENCE",
            "reference evidence case_id differs from comparison case {}".format(case_id),
        )
    if evidence.get("reference_signal_sha256") != reference_signal_hash:
        raise FidelityValidationError(
            "HASH_MISMATCH",
            "reference evidence for {} does not bind the supplied reference signal".format(case_id),
        )
    _nonempty_string(evidence.get("reference_freeze_id"), "reference_freeze_id")
    _nonempty_string(evidence.get("source_topic"), "reference source_topic")
    for key in ("source_bag_sha256", "extraction_pipeline_sha256", "calibration_sha256"):
        if not is_sha256(evidence.get(key)):
            raise FidelityValidationError(
                "MALFORMED_REFERENCE_EVIDENCE",
                "reference evidence {}.{} must be a SHA-256".format(case_id, key),
            )
    forbidden = _contains_forbidden_reference_token(evidence)
    if forbidden is not None:
        raise FidelityValidationError(
            "PROXY_REFERENCE_FORBIDDEN",
            "reference evidence for {} contains forbidden proxy/model token {!r}".format(case_id, forbidden),
        )
    return path, actual_hash


def validate_case(case_value: Any, seen_case_ids: set[str]) -> ValidatedCase:
    case = _mapping(case_value, "comparison case")
    case_id = _nonempty_string(case.get("case_id"), "case_id")
    if case_id in seen_case_ids:
        raise FidelityValidationError("MALFORMED_MANIFEST", "duplicate case_id {}".format(case_id))
    seen_case_ids.add(case_id)
    if case.get("signal_unit") != "m":
        raise FidelityValidationError("MALFORMED_MANIFEST", "case {} signal_unit must be m".format(case_id))
    plant_path, plant_hash = _bound_file(
        case.get("plant_signal_path"), case.get("plant_signal_sha256"), "plant signal for {}".format(case_id)
    )
    if case.get("reference_signal_path") is None or case.get("reference_signal_sha256") is None:
        raise FidelityValidationError(
            "MISSING_FROZEN_REAL_REFERENCE",
            "case {} has no hash-bound reference signal".format(case_id),
        )
    reference_path, reference_hash = _bound_file(
        case.get("reference_signal_path"), case.get("reference_signal_sha256"), "reference signal for {}".format(case_id)
    )
    evidence_path, evidence_hash = _validate_reference_evidence(
        case.get("reference_evidence_path"),
        case.get("reference_evidence_sha256"),
        reference_hash,
        case_id,
    )
    return ValidatedCase(
        case_id=case_id,
        plant=load_signal(plant_path, plant_hash, "plant signal for {}".format(case_id)),
        reference=load_signal(reference_path, reference_hash, "reference signal for {}".format(case_id)),
        reference_evidence_path=evidence_path,
        reference_evidence_hash=evidence_hash,
    )


def _centered_values(series: SignalSeries) -> Tuple[float, Tuple[float, ...]]:
    baseline = float(median(series.values_m))
    return baseline, tuple(value - baseline for value in series.values_m)


def _positive_zero_crossings(times: Sequence[float], values: Sequence[float]) -> List[float]:
    crossings: List[float] = []
    for index in range(len(values) - 1):
        left = values[index]
        right = values[index + 1]
        if left <= 0.0 < right and right != left:
            fraction = -left / (right - left)
            crossings.append(times[index] + fraction * (times[index + 1] - times[index]))
    return crossings


def _local_peak_times(times: Sequence[float], values: Sequence[float]) -> List[float]:
    peaks: List[float] = []
    for index in range(1, len(values) - 1):
        if values[index] >= values[index - 1] and values[index] > values[index + 1]:
            peaks.append(times[index])
    return peaks


def _estimate_frequency_hz(times: Sequence[float], centered: Sequence[float]) -> Optional[float]:
    crossings = _positive_zero_crossings(times, centered)
    candidates: List[float] = []
    if len(crossings) >= 2:
        candidates = [right - left for left, right in zip(crossings, crossings[1:]) if right > left]
    if not candidates:
        peaks = _local_peak_times(times, centered)
        if len(peaks) >= 2:
            candidates = [right - left for left, right in zip(peaks, peaks[1:]) if right > left]
    if not candidates:
        return None
    period = float(median(candidates))
    return None if period <= 0.0 else 1.0 / period


def _absolute_peak_sequence(times: Sequence[float], centered: Sequence[float]) -> List[Tuple[float, float]]:
    magnitudes = [abs(value) for value in centered]
    if not magnitudes:
        return []
    maximum = max(magnitudes)
    if maximum <= 1e-12:
        return []
    minimum = maximum * 0.05
    peaks: List[Tuple[float, float]] = []
    for index in range(1, len(magnitudes) - 1):
        current = magnitudes[index]
        if current >= minimum and current >= magnitudes[index - 1] and current > magnitudes[index + 1]:
            peaks.append((times[index], current))
    return peaks


def _estimate_damping_ratio(
    times: Sequence[float], centered: Sequence[float], frequency_hz: Optional[float]
) -> Tuple[Optional[float], int]:
    if frequency_hz is None or frequency_hz <= 0.0:
        return None, 0
    peaks = _absolute_peak_sequence(times, centered)
    decrements: List[float] = []
    for (left_time, left_value), (right_time, right_value) in zip(peaks, peaks[1:]):
        cycles = frequency_hz * (right_time - left_time)
        if cycles <= 0.0 or left_value <= right_value or right_value <= 0.0:
            continue
        decrements.append(math.log(left_value / right_value) / cycles)
    if not decrements:
        return None, len(peaks)
    logarithmic_decrement = float(median(decrements))
    damping = logarithmic_decrement / math.sqrt(4.0 * math.pi * math.pi + logarithmic_decrement * logarithmic_decrement)
    return damping, len(peaks)


def estimate_signal_metrics(series: SignalSeries) -> Dict[str, Any]:
    """Estimate amplitude, dominant frequency and free-decay damping.

    The estimator is intentionally transparent and dependency-free: amplitude
    is the largest baseline-centred excursion; frequency comes from positive
    zero crossings (or same-sign maxima); damping is the median logarithmic
    decrement of successive absolute extrema.  A metric becomes ``None`` when
    the signal does not identify it rather than receiving a fabricated value.
    """

    baseline, centered = _centered_values(series)
    amplitude = max(abs(value) for value in centered)
    frequency = _estimate_frequency_hz(series.times_sec, centered)
    damping, peak_count = _estimate_damping_ratio(series.times_sec, centered, frequency)
    return {
        "sample_count": len(series.times_sec),
        "time_start_sec": series.times_sec[0],
        "time_end_sec": series.times_sec[-1],
        "baseline_estimate_m": baseline,
        "amplitude_m": amplitude,
        "frequency_hz": frequency,
        "damping_ratio": damping,
        "damping_peak_count": peak_count,
    }


def _fit_phase_rad(
    series: SignalSeries,
    frequency_hz: float,
    *,
    start_sec: Optional[float] = None,
    end_sec: Optional[float] = None,
) -> Optional[float]:
    if frequency_hz <= 0.0:
        return None
    _, centered = _centered_values(series)
    angular_frequency = 2.0 * math.pi * frequency_hz
    cos_cos = sin_sin = cos_sin = cos_y = sin_y = 0.0
    sample_count = 0
    for time_sec, value in zip(series.times_sec, centered):
        if start_sec is not None and time_sec < start_sec:
            continue
        if end_sec is not None and time_sec > end_sec:
            continue
        cosine = math.cos(angular_frequency * time_sec)
        sine = math.sin(angular_frequency * time_sec)
        cos_cos += cosine * cosine
        sin_sin += sine * sine
        cos_sin += cosine * sine
        cos_y += cosine * value
        sin_y += sine * value
        sample_count += 1
    if sample_count < 16:
        return None
    determinant = cos_cos * sin_sin - cos_sin * cos_sin
    if abs(determinant) <= 1e-15:
        return None
    cosine_coefficient = (cos_y * sin_sin - sin_y * cos_sin) / determinant
    sine_coefficient = (sin_y * cos_cos - cos_y * cos_sin) / determinant
    if math.hypot(cosine_coefficient, sine_coefficient) <= 1e-12:
        return None
    # y = A cos(w t - phase); this convention is recorded in the report.
    return math.atan2(sine_coefficient, cosine_coefficient)


def _wrap_phase_rad(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _relative_error(actual: Optional[float], reference: Optional[float]) -> Optional[float]:
    if actual is None or reference is None or abs(reference) <= 1e-12:
        return None
    return abs(actual - reference) / abs(reference)


def _status_from_error(error: Optional[float], threshold: float) -> str:
    if error is None:
        return "NOT_EVALUATED"
    return "PASS" if error <= threshold else "FAIL"


def compare_case(case: ValidatedCase, policy: Mapping[str, Any]) -> Dict[str, Any]:
    """Calculate four per-case dimensions from fully verified offline data."""

    plant = estimate_signal_metrics(case.plant)
    reference = estimate_signal_metrics(case.reference)
    amplitude_error = _relative_error(plant["amplitude_m"], reference["amplitude_m"])
    frequency_error = _relative_error(plant["frequency_hz"], reference["frequency_hz"])
    damping_error = (
        None
        if plant["damping_ratio"] is None or reference["damping_ratio"] is None
        else abs(float(plant["damping_ratio"]) - float(reference["damping_ratio"]))
    )
    phase_error_deg: Optional[float] = None
    phase_plant: Optional[float] = None
    phase_reference: Optional[float] = None
    phase_overlap_start_sec = max(case.plant.times_sec[0], case.reference.times_sec[0])
    phase_overlap_end_sec = min(case.plant.times_sec[-1], case.reference.times_sec[-1])
    frequency_for_phase = reference["frequency_hz"]
    if frequency_for_phase is not None and phase_overlap_end_sec > phase_overlap_start_sec:
        phase_plant = _fit_phase_rad(
            case.plant,
            float(frequency_for_phase),
            start_sec=phase_overlap_start_sec,
            end_sec=phase_overlap_end_sec,
        )
        phase_reference = _fit_phase_rad(
            case.reference,
            float(frequency_for_phase),
            start_sec=phase_overlap_start_sec,
            end_sec=phase_overlap_end_sec,
        )
        if phase_plant is not None and phase_reference is not None:
            phase_error_deg = abs(math.degrees(_wrap_phase_rad(phase_plant - phase_reference)))
    statuses = {
        "amplitude": _status_from_error(amplitude_error, float(policy["amplitude_relative_error_max"])),
        "frequency": _status_from_error(frequency_error, float(policy["frequency_relative_error_max"])),
        "damping": _status_from_error(damping_error, float(policy["damping_absolute_error_max"])),
        "phase": _status_from_error(phase_error_deg, float(policy["phase_absolute_error_deg_max"])),
    }
    return {
        "case_id": case.case_id,
        "plant_signal": {"path": str(case.plant.path), "sha256": case.plant.sha256},
        "reference_signal": {"path": str(case.reference.path), "sha256": case.reference.sha256},
        "reference_evidence": {
            "path": str(case.reference_evidence_path),
            "sha256": case.reference_evidence_hash,
        },
        "metrics": {
            "plant": plant,
            "reference": reference,
            "amplitude_relative_error": amplitude_error,
            "frequency_relative_error": frequency_error,
            "damping_absolute_error": damping_error,
            "phase_convention": "y=A*cos(2*pi*f*t-phase)",
            "phase_overlap_start_sec": phase_overlap_start_sec,
            "phase_overlap_end_sec": phase_overlap_end_sec,
            "plant_phase_rad": phase_plant,
            "reference_phase_rad": phase_reference,
            "phase_absolute_error_deg": phase_error_deg,
        },
        "dimension_statuses": statuses,
    }


def _aggregate_dimension(case_metrics: Sequence[Mapping[str, Any]], dimension: str) -> str:
    values = [item["dimension_statuses"][dimension] for item in case_metrics]
    if not values or any(value not in DIMENSION_STATUSES for value in values):
        return "NOT_EVALUATED"
    if "FAIL" in values:
        return "FAIL"
    if all(value == "PASS" for value in values):
        return "PASS"
    return "NOT_EVALUATED"


def evaluate_ranking(case_metrics: Sequence[Mapping[str, Any]], policy: Mapping[str, Any]) -> Dict[str, Any]:
    """Test whether plant and reference agree on strict amplitude ordering."""

    minimum_cases = int(policy["ranking_minimum_cases"])
    if len(case_metrics) < minimum_cases:
        return {
            "status": "NOT_EVALUATED",
            "metric": policy["ranking_metric"],
            "reason": "requires at least {} cases; received {}".format(minimum_cases, len(case_metrics)),
            "reference_order": [],
            "plant_order": [],
        }
    rows: List[Tuple[str, float, float]] = []
    for item in case_metrics:
        plant_value = item["metrics"]["plant"]["amplitude_m"]
        reference_value = item["metrics"]["reference"]["amplitude_m"]
        if not isinstance(plant_value, (int, float)) or not isinstance(reference_value, (int, float)):
            return {
                "status": "NOT_EVALUATED",
                "metric": policy["ranking_metric"],
                "reason": "amplitude is not identifiable for every case",
                "reference_order": [],
                "plant_order": [],
            }
        rows.append((str(item["case_id"]), float(plant_value), float(reference_value)))
    tie_tolerance = float(policy["ranking_tie_relative_tolerance"])
    for left_index, (_, _, left_reference) in enumerate(rows):
        for _, _, right_reference in rows[left_index + 1 :]:
            scale = max(abs(left_reference), abs(right_reference), 1e-12)
            if abs(left_reference - right_reference) <= tie_tolerance * scale:
                return {
                    "status": "NOT_EVALUATED",
                    "metric": policy["ranking_metric"],
                    "reason": "reference amplitude order is tied or below the declared tie tolerance",
                    "reference_order": [],
                    "plant_order": [],
                }
    reference_order = [case_id for case_id, _, _ in sorted(rows, key=lambda item: (-item[2], item[0]))]
    plant_order = [case_id for case_id, _, _ in sorted(rows, key=lambda item: (-item[1], item[0]))]
    return {
        "status": "PASS" if plant_order == reference_order else "FAIL",
        "metric": policy["ranking_metric"],
        "reason": "exact descending amplitude order required",
        "reference_order": reference_order,
        "plant_order": plant_order,
    }


def _base_report(
    comparison_manifest_path: Any,
    comparison_manifest_hash: Any,
    threshold_policy_path: Any,
    threshold_policy_hash: Any,
) -> Dict[str, Any]:
    module_path = Path(__file__).resolve()
    return {
        "report_type": REPORT_TYPE,
        "report_schema_version": REPORT_SCHEMA_VERSION,
        "tool_id": TOOL_ID,
        "generated_at_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "verifier_source": {"path": str(module_path), "sha256": sha256_file(module_path)},
        "status": "NO_GO",
        "formal": False,
        "development_only": True,
        "fidelity_validation_status": "UNVALIDATED_DEVELOPMENT_ONLY",
        "physical_primary_eligible": False,
        "truth_topic": "/sim_truth/liquid_height",
        "comparison_id": None,
        "plant_code_hash": None,
        "plant_parameter_hash": None,
        "plant_input_schema_hash": None,
        "plant_output_schema_hash": None,
        "input_artifacts": {
            "comparison_manifest": _artifact_descriptor(comparison_manifest_path, comparison_manifest_hash),
            "threshold_policy": _artifact_descriptor(threshold_policy_path, threshold_policy_hash),
            "cases": [],
        },
        "threshold_policy": None,
        "case_metrics": [],
        "validation_dimensions": {dimension: "NOT_EVALUATED" for dimension in DIMENSIONS},
        "ranking": {
            "status": "NOT_EVALUATED",
            "metric": "amplitude",
            "reason": "no verified comparison cases were available",
            "reference_order": [],
            "plant_order": [],
        },
        "formal_gate_compatibility": {
            "eligible": False,
            "reasons": [
                "This verifier is hard-coded development_only=true and formal=false.",
                "A separate formal release requires independently reviewed plant/capability/fidelity evidence.",
            ],
        },
        "errors": [],
        "warnings": [
            "Numerical agreement is development evidence only; it is not a physical-primary claim.",
            "H_proxy and H_modal are forbidden as reference sources.",
        ],
    }


def validate_report_schema(report: Mapping[str, Any]) -> None:
    """Validate the strict non-formal report ABI without ``jsonschema``.

    Keeping this validator in the package makes unit tests and future gates
    exercise the same structural contract even in minimal ROS environments.
    """

    required = {
        "report_type",
        "report_schema_version",
        "tool_id",
        "generated_at_utc",
        "verifier_source",
        "status",
        "formal",
        "development_only",
        "fidelity_validation_status",
        "physical_primary_eligible",
        "truth_topic",
        "comparison_id",
        "plant_code_hash",
        "plant_parameter_hash",
        "plant_input_schema_hash",
        "plant_output_schema_hash",
        "input_artifacts",
        "threshold_policy",
        "case_metrics",
        "validation_dimensions",
        "ranking",
        "formal_gate_compatibility",
        "errors",
        "warnings",
    }
    missing = sorted(required.difference(report))
    if missing:
        raise FidelityValidationError("MALFORMED_REPORT", "report misses required fields: {}".format(", ".join(missing)))
    if report.get("report_type") != REPORT_TYPE or report.get("report_schema_version") != REPORT_SCHEMA_VERSION:
        raise FidelityValidationError("MALFORMED_REPORT", "report type/schema version mismatch")
    if report.get("status") not in REPORT_STATUSES:
        raise FidelityValidationError("MALFORMED_REPORT", "report has an unsupported status")
    if report.get("formal") is not False or report.get("development_only") is not True:
        raise FidelityValidationError("MALFORMED_REPORT", "development verifier must never set formal=true")
    if report.get("fidelity_validation_status") != "UNVALIDATED_DEVELOPMENT_ONLY":
        raise FidelityValidationError("MALFORMED_REPORT", "report must remain UNVALIDATED_DEVELOPMENT_ONLY")
    if report.get("physical_primary_eligible") is not False:
        raise FidelityValidationError("MALFORMED_REPORT", "report must remain ineligible for physical primary")
    if report.get("truth_topic") != "/sim_truth/liquid_height":
        raise FidelityValidationError("MALFORMED_REPORT", "report truth_topic mismatch")
    dimensions = _mapping(report.get("validation_dimensions"), "report.validation_dimensions")
    if set(dimensions) != set(DIMENSIONS) or any(value not in DIMENSION_STATUSES for value in dimensions.values()):
        raise FidelityValidationError("MALFORMED_REPORT", "report validation_dimensions are invalid")
    compatibility = _mapping(report.get("formal_gate_compatibility"), "report.formal_gate_compatibility")
    if compatibility.get("eligible") is not False:
        raise FidelityValidationError("MALFORMED_REPORT", "development report cannot be formal-gate eligible")
    for key in PLANT_HASH_KEYS:
        value = report.get(key)
        if value is not None and not is_sha256(value):
            raise FidelityValidationError("MALFORMED_REPORT", "report {} must be SHA-256 or null".format(key))
    if report["status"] == "NO_GO" and not report["errors"]:
        raise FidelityValidationError("MALFORMED_REPORT", "NO_GO report requires at least one error")
    if report["status"] == "DEVELOPMENT_METRICS_COMPLETE_NOT_FORMAL" and any(
        dimensions[name] != "PASS" for name in DIMENSIONS
    ):
        raise FidelityValidationError("MALFORMED_REPORT", "complete development report requires all dimensions PASS")


def verify_development_fidelity(
    *,
    comparison_manifest_path: Any,
    comparison_manifest_sha256: Any,
    threshold_policy_path: Any,
    threshold_policy_sha256: Any,
) -> Dict[str, Any]:
    """Build one strict, permanently non-formal fidelity report.

    All parsing failures are rendered as a schema-valid ``NO_GO`` report, so a
    caller can archive an auditable reason instead of turning a missing real
    reference into a silently skipped check.
    """

    report = _base_report(
        comparison_manifest_path,
        comparison_manifest_sha256,
        threshold_policy_path,
        threshold_policy_sha256,
    )
    try:
        manifest_path, manifest, manifest_hash = _bound_json(
            comparison_manifest_path,
            comparison_manifest_sha256,
            "comparison manifest",
        )
        report["input_artifacts"]["comparison_manifest"] = _artifact_descriptor(
            str(manifest_path), comparison_manifest_sha256, manifest_hash
        )
        base = _validate_manifest_base(manifest)
        report["comparison_id"] = base["comparison_id"]
        report.update(base["plant_identity"])
    except FidelityValidationError as exc:
        report["errors"].append(_error(exc.code, exc.message))
        validate_report_schema(report)
        return report

    try:
        policy_path, policy, policy_hash = _bound_json(
            threshold_policy_path,
            threshold_policy_sha256,
            "threshold policy",
        )
        normalized_policy = validate_threshold_policy(policy)
        report["input_artifacts"]["threshold_policy"] = _artifact_descriptor(
            str(policy_path), threshold_policy_sha256, policy_hash
        )
        report["threshold_policy"] = dict(normalized_policy, path=str(policy_path), sha256=policy_hash)
    except FidelityValidationError as exc:
        report["errors"].append(_error(exc.code, exc.message))
        validate_report_schema(report)
        return report

    validated_cases: List[ValidatedCase] = []
    seen_case_ids: set[str] = set()
    for case in base["cases"]:
        try:
            validated = validate_case(case, seen_case_ids)
            validated_cases.append(validated)
            report["input_artifacts"]["cases"].append(
                {
                    "case_id": validated.case_id,
                    "plant_signal": {"path": str(validated.plant.path), "sha256": validated.plant.sha256},
                    "reference_signal": {"path": str(validated.reference.path), "sha256": validated.reference.sha256},
                    "reference_evidence": {
                        "path": str(validated.reference_evidence_path),
                        "sha256": validated.reference_evidence_hash,
                    },
                }
            )
        except FidelityValidationError as exc:
            report["errors"].append(_error(exc.code, exc.message))
    # Do not emit partially successful dimensions when a case lacks real,
    # frozen or hash-bound evidence.  That prevents a single fabricated
    # reference from sharing a report with valid cases and looking approved.
    if report["errors"]:
        validate_report_schema(report)
        return report

    case_metrics = [compare_case(case, normalized_policy) for case in validated_cases]
    report["case_metrics"] = case_metrics
    dimensions = {
        "amplitude": _aggregate_dimension(case_metrics, "amplitude"),
        "frequency": _aggregate_dimension(case_metrics, "frequency"),
        "damping": _aggregate_dimension(case_metrics, "damping"),
        "phase": _aggregate_dimension(case_metrics, "phase"),
    }
    ranking = evaluate_ranking(case_metrics, normalized_policy)
    dimensions["ranking"] = ranking["status"]
    report["validation_dimensions"] = dimensions
    report["ranking"] = ranking
    if all(dimensions[dimension] == "PASS" for dimension in DIMENSIONS):
        report["status"] = "DEVELOPMENT_METRICS_COMPLETE_NOT_FORMAL"
    else:
        report["status"] = "DEVELOPMENT_METRICS_FAILED_NOT_FORMAL"
    validate_report_schema(report)
    return report


def write_report(path: Path, report: Mapping[str, Any], *, overwrite: bool = False) -> None:
    """Atomically write a report without silently overwriting prior evidence."""

    validate_report_schema(report)
    if not path.is_absolute():
        raise FidelityValidationError("MALFORMED_OUTPUT", "output path must be absolute")
    if path.exists() and not overwrite:
        raise FidelityValidationError("OUTPUT_EXISTS", "refusing to overwrite existing report {}".format(path))
    if not path.parent.is_dir():
        raise FidelityValidationError("MALFORMED_OUTPUT", "output parent does not exist: {}".format(path.parent))
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
        temporary.replace(path)
    except OSError as exc:
        try:
            if temporary.exists():
                temporary.unlink()
        except OSError:
            pass
        raise FidelityValidationError("OUTPUT_WRITE_FAILED", "cannot write report {}: {}".format(path, exc)) from exc
