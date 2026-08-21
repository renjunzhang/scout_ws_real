#!/usr/bin/env python3
"""Fit and audit a phase-indexed empirical recovery admission rule.

The tool consumes one strict CSV whose complete rollout identities and seeds
are already assigned to mutually exclusive ``fit``, ``tune`` and
``held_out`` splits.  Fit data determine positive per-phase-bin scales.  Tune
data may select only one global shrinkage factor.  Held-out data are evaluated
exactly once by each create-new invocation and never influence the fitted
scales or selected factor.

The resulting rule is a diagonal nine-dimensional ellipsoid conjoined with a
fourteen-dimensional execution-state box.  It is empirical classification
evidence for one frozen dataset, not a safety certificate, invariant set, or
authorization for physical use.
"""

import argparse
import csv
import dataclasses
import hashlib
import io
import json
import math
from pathlib import Path
import re
from statistics import NormalDist
import sys
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


SCHEMA = "spmpc_phase_rejoin_recovery_dataset_v1"
MANIFEST_SCHEMA = "spmpc_phase_rejoin_recovery_fit_manifest_v1"
REPORT_SCHEMA = "spmpc_phase_rejoin_recovery_held_out_report_v1"
SCALE_SCHEMA = "spmpc_phase_rejoin_recovery_scales_v1"

SPLITS = ("fit", "tune", "held_out")
STATE_ERROR_COLUMNS = (
    "x",
    "y",
    "yaw",
    "v",
    "omega",
    "eta_x",
    "eta_x_dot",
    "eta_y",
    "eta_y_dot",
)
EXECUTION_ERROR_COLUMNS = (
    "linear_output",
    "angular_output",
    "linear_pending_0",
    "linear_pending_1",
    "linear_pending_2",
    "linear_pending_3",
    "linear_pending_4",
    "angular_pending_0",
    "angular_pending_1",
    "angular_pending_2",
    "angular_pending_3",
    "angular_pending_4",
    "angular_pending_5",
    "angular_pending_6",
)
ERROR_COLUMNS = STATE_ERROR_COLUMNS + EXECUTION_ERROR_COLUMNS
INPUT_COLUMNS = (
    "split",
    "rollout_id",
    "seed",
    "phase_index",
    "recovered",
) + ERROR_COLUMNS

STATE_RADIUS_COLUMNS = tuple("r_" + name for name in STATE_ERROR_COLUMNS)
EXECUTION_BOUND_COLUMNS = tuple(
    "beta_" + name for name in EXECUTION_ERROR_COLUMNS
)
OUTPUT_COLUMNS = (
    "phase_index",
    "phase_bin_start",
    "phase_bin_end",
    "shrinkage",
) + STATE_RADIUS_COLUMNS + EXECUTION_BOUND_COLUMNS

COMPILED_STATE_WIDTH = 22
COMPILED_LINEAR_PENDING_COUNT = 5
COMPILED_ANGULAR_PENDING_COUNT = 7
COMPILED_GATE_RADIUS_COUNT = 9
COMPILED_EXECUTION_BOUND_COUNT = 14
COMPILED_MINIMUM_DENOMINATOR = 1.0e-9
EXECUTION_COMPATIBILITY_CONTRACT = "phase_indexed_execution_box_v1"
GATE_CONTRACT = "phase_indexed_empirical_9d_ellipsoid_v1"

MAX_UINT32 = (1 << 32) - 1
ACCEPTANCE_EPSILON = 1.0e-12
ROLLOUT_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}\Z")
DEFAULT_SHRINKAGE_GRID = (
    1.00,
    0.95,
    0.90,
    0.85,
    0.80,
    0.75,
    0.70,
    0.65,
    0.60,
    0.55,
    0.50,
    0.45,
    0.40,
    0.35,
    0.30,
    0.25,
    0.20,
    0.15,
    0.10,
    0.05,
    0.02,
    0.01,
)


class RecoveryFitError(RuntimeError):
    """A fail-closed dataset, fitting, or integrity error."""


@dataclasses.dataclass(frozen=True)
class RecoveryRow:
    split: str
    rollout_id: str
    seed: int
    phase_index: int
    recovered: bool
    errors: Tuple[float, ...]

    def error(self, name: str) -> float:
        return self.errors[ERROR_COLUMNS.index(name)]


@dataclasses.dataclass(frozen=True)
class FittingOptions:
    phase_bin_width: int = 1
    shrinkage_grid: Tuple[float, ...] = DEFAULT_SHRINKAGE_GRID
    max_false_accept: float = 0.05
    min_coverage: float = 0.50
    confidence: float = 0.95
    minimum_scale: float = COMPILED_MINIMUM_DENOMINATOR


@dataclasses.dataclass(frozen=True)
class PipelineResult:
    phases: Tuple[int, ...]
    base_scales: Mapping[int, Mapping[str, float]]
    selected_shrinkage: float
    tuning_candidates: Tuple[Mapping[str, Any], ...]
    held_out_report: Mapping[str, Any]


def _sha256_bytes(contents: bytes) -> str:
    return hashlib.sha256(contents).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_uint(text: str, label: str, maximum: Optional[int] = None) -> int:
    if not re.fullmatch(r"0|[1-9][0-9]*", text):
        raise RecoveryFitError("{} must be a canonical unsigned integer".format(label))
    value = int(text)
    if maximum is not None and value > maximum:
        raise RecoveryFitError("{} exceeds {}".format(label, maximum))
    return value


def _finite_float(text: str, label: str) -> float:
    if text.strip() != text or not text:
        raise RecoveryFitError("{} is not a strict numeric field".format(label))
    try:
        value = float(text)
    except ValueError as error:
        raise RecoveryFitError("{} is not numeric".format(label)) from error
    if not math.isfinite(value):
        raise RecoveryFitError("{} is non-finite".format(label))
    return value


def _phase_bin(phase_index: int, width: int) -> int:
    return (phase_index // width) * width


def _canonical_row(row: RecoveryRow) -> Mapping[str, Any]:
    return {
        "split": row.split,
        "rollout_id": row.rollout_id,
        "seed": row.seed,
        "phase_index": row.phase_index,
        "recovered": int(row.recovered),
        "errors": [format(value, ".17g") for value in row.errors],
    }


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode(
        "utf-8"
    )


def _canonical_split_sha256(rows: Iterable[RecoveryRow]) -> str:
    ordered = sorted(
        rows,
        key=lambda row: (row.rollout_id, row.seed, row.phase_index),
    )
    return _sha256_bytes(_json_bytes([_canonical_row(row) for row in ordered]))


def load_recovery_csv(path: Path) -> Tuple[RecoveryRow, ...]:
    """Load the exact input schema and reject split leakage or reweighting."""
    try:
        stream = path.open("r", encoding="utf-8", newline="")
    except OSError as error:
        raise RecoveryFitError("cannot open recovery CSV: {}".format(path)) from error

    rows: List[RecoveryRow] = []
    rollout_owners: Dict[str, Tuple[str, int]] = {}
    seed_owners: Dict[int, str] = {}
    observations = set()
    with stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != INPUT_COLUMNS:
            raise RecoveryFitError(
                "input header mismatch: expected {}".format(",".join(INPUT_COLUMNS))
            )
        for line_number, record in enumerate(reader, start=2):
            if None in record:
                raise RecoveryFitError("line {} has extra columns".format(line_number))
            split = record["split"]
            if split not in SPLITS:
                raise RecoveryFitError("line {} has invalid split".format(line_number))
            rollout_id = record["rollout_id"]
            if not ROLLOUT_ID_PATTERN.fullmatch(rollout_id):
                raise RecoveryFitError(
                    "line {} has invalid rollout_id".format(line_number)
                )
            seed = _canonical_uint(
                record["seed"], "line {} seed".format(line_number), MAX_UINT32
            )
            phase_index = _canonical_uint(
                record["phase_index"], "line {} phase_index".format(line_number)
            )
            recovered_text = record["recovered"]
            if recovered_text not in ("0", "1"):
                raise RecoveryFitError(
                    "line {} recovered must be exactly 0 or 1".format(line_number)
                )
            errors = tuple(
                _finite_float(record[name], "line {} {}".format(line_number, name))
                for name in ERROR_COLUMNS
            )
            yaw_error = errors[STATE_ERROR_COLUMNS.index("yaw")]
            if abs(yaw_error) > math.pi + ACCEPTANCE_EPSILON:
                raise RecoveryFitError(
                    "line {} yaw error is not wrapped to [-pi, pi]".format(line_number)
                )

            existing_rollout = rollout_owners.get(rollout_id)
            if existing_rollout is not None and existing_rollout != (split, seed):
                raise RecoveryFitError(
                    "rollout_id {} crosses a split or seed boundary".format(rollout_id)
                )
            rollout_owners[rollout_id] = (split, seed)
            existing_seed = seed_owners.get(seed)
            if existing_seed is not None and existing_seed != split:
                raise RecoveryFitError("seed {} crosses split boundaries".format(seed))
            seed_owners[seed] = split
            observation = (rollout_id, phase_index)
            if observation in observations:
                raise RecoveryFitError(
                    "duplicate rollout/phase observation {}:{}".format(
                        rollout_id, phase_index
                    )
                )
            observations.add(observation)
            rows.append(
                RecoveryRow(
                    split=split,
                    rollout_id=rollout_id,
                    seed=seed,
                    phase_index=phase_index,
                    recovered=recovered_text == "1",
                    errors=errors,
                )
            )

    if not rows:
        raise RecoveryFitError("recovery CSV is empty")
    phase_sets = {
        split: {row.phase_index for row in rows if row.split == split}
        for split in SPLITS
    }
    if any(not phase_sets[split] for split in SPLITS):
        raise RecoveryFitError("fit, tune, and held_out must all be non-empty")
    if not (phase_sets["fit"] == phase_sets["tune"] == phase_sets["held_out"]):
        raise RecoveryFitError(
            "phase coverage differs across fit, tune, and held_out splits"
        )
    phases = sorted(phase_sets["fit"])
    expected = list(range(phases[0], phases[-1] + 1))
    if phases != expected:
        raise RecoveryFitError("phase coverage contains a missing phase index")
    return tuple(rows)


def validate_options(options: FittingOptions) -> None:
    if options.phase_bin_width <= 0:
        raise RecoveryFitError("phase_bin_width must be positive")
    if not (0.0 <= options.max_false_accept < 1.0):
        raise RecoveryFitError("max_false_accept must be in [0, 1)")
    if not (0.0 < options.min_coverage <= 1.0):
        raise RecoveryFitError("min_coverage must be in (0, 1]")
    if not (0.5 < options.confidence < 1.0):
        raise RecoveryFitError("confidence must be in (0.5, 1)")
    if (
        not math.isfinite(options.minimum_scale)
        or options.minimum_scale < COMPILED_MINIMUM_DENOMINATOR
    ):
        raise RecoveryFitError(
            "minimum_scale must be finite and at least the compiled denominator"
        )
    if not options.shrinkage_grid:
        raise RecoveryFitError("shrinkage grid is empty")
    for factor in options.shrinkage_grid:
        if not math.isfinite(factor) or factor <= 0.0 or factor > 1.0:
            raise RecoveryFitError("shrinkage factors must be in (0, 1]")
    if len(set(options.shrinkage_grid)) != len(options.shrinkage_grid):
        raise RecoveryFitError("shrinkage grid contains duplicates")


def _validate_bin_classes(rows: Sequence[RecoveryRow], options: FittingOptions) -> None:
    """Validate only the class support allowed before held-out evaluation."""
    pre_evaluation_rows = [row for row in rows if row.split != "held_out"]
    bins = sorted(
        {
            _phase_bin(row.phase_index, options.phase_bin_width)
            for row in pre_evaluation_rows
        }
    )
    for bin_start in bins:
        for split in ("fit", "tune"):
            selected = [
                row
                for row in pre_evaluation_rows
                if row.split == split
                and _phase_bin(row.phase_index, options.phase_bin_width) == bin_start
            ]
            recovered = sum(1 for row in selected if row.recovered)
            unrecovered = len(selected) - recovered
            if recovered == 0:
                raise RecoveryFitError(
                    "{} phase bin {} has no recovered rollout".format(split, bin_start)
                )
            if split == "tune" and unrecovered == 0:
                raise RecoveryFitError(
                    "{} phase bin {} has no unrecovered rollout".format(
                        split, bin_start
                    )
                )


def fit_base_scales(
    rows: Sequence[RecoveryRow], options: FittingOptions
) -> Mapping[int, Mapping[str, float]]:
    """Fit a positive envelope using only recovered fit rollouts.

    State radii are ``sqrt(9) * max(abs(error))`` per coordinate, which admits
    every recovered fit state into the diagonal ellipsoid at shrinkage 1.  The
    execution box uses coordinate-wise maxima.  Tune may only shrink these
    already-frozen base scales globally.
    """
    recovered_fit = [row for row in rows if row.split == "fit" and row.recovered]
    bins = sorted(
        {_phase_bin(row.phase_index, options.phase_bin_width) for row in rows}
    )
    result: Dict[int, Mapping[str, float]] = {}
    state_multiplier = math.sqrt(float(len(STATE_ERROR_COLUMNS)))
    for bin_start in bins:
        selected = [
            row
            for row in recovered_fit
            if _phase_bin(row.phase_index, options.phase_bin_width) == bin_start
        ]
        if not selected:
            raise RecoveryFitError(
                "fit phase bin {} has no recovered sample".format(bin_start)
            )
        scales: Dict[str, float] = {}
        for name in STATE_ERROR_COLUMNS:
            maximum = max(abs(row.error(name)) for row in selected)
            scale = max(options.minimum_scale, state_multiplier * maximum)
            if not math.isfinite(scale):
                raise RecoveryFitError(
                    "fit phase bin {} produces a non-finite {} radius".format(
                        bin_start, name
                    )
                )
            scales[name] = scale
        for name in EXECUTION_ERROR_COLUMNS:
            maximum = max(abs(row.error(name)) for row in selected)
            scale = max(options.minimum_scale, maximum)
            if not math.isfinite(scale):
                raise RecoveryFitError(
                    "fit phase bin {} produces a non-finite {} bound".format(
                        bin_start, name
                    )
                )
            scales[name] = scale
        result[bin_start] = scales
    return result


def _scaled_value(base: float, shrinkage: float, minimum: float) -> float:
    return max(minimum, base * shrinkage)


def gate_accepts(
    row: RecoveryRow,
    base_scales: Mapping[int, Mapping[str, float]],
    shrinkage: float,
    options: FittingOptions,
) -> Tuple[bool, bool, bool]:
    scales = base_scales[_phase_bin(row.phase_index, options.phase_bin_width)]
    state_metric = 0.0
    for name in STATE_ERROR_COLUMNS:
        radius = _scaled_value(scales[name], shrinkage, options.minimum_scale)
        normalized = row.error(name) / radius
        state_metric += normalized * normalized
        if (
            not math.isfinite(state_metric)
            or state_metric > 1.0 + ACCEPTANCE_EPSILON
        ):
            break
    execution_metric = 0.0
    for name in EXECUTION_ERROR_COLUMNS:
        bound = _scaled_value(scales[name], shrinkage, options.minimum_scale)
        execution_metric = max(execution_metric, abs(row.error(name)) / bound)
    state_accepted = state_metric <= 1.0 + ACCEPTANCE_EPSILON
    execution_accepted = execution_metric <= 1.0 + ACCEPTANCE_EPSILON
    return state_accepted and execution_accepted, state_accepted, execution_accepted


def _wilson_interval(
    successes: int, total: int, confidence: float
) -> Tuple[float, float]:
    if total <= 0:
        raise RecoveryFitError("cannot form a confidence interval with zero trials")
    z = NormalDist().inv_cdf(0.5 + 0.5 * confidence)
    proportion = successes / float(total)
    z_squared = z * z
    denominator = 1.0 + z_squared / total
    center = (proportion + z_squared / (2.0 * total)) / denominator
    half_width = z * math.sqrt(
        proportion * (1.0 - proportion) / total
        + z_squared / (4.0 * total * total)
    ) / denominator
    return max(0.0, center - half_width), min(1.0, center + half_width)


def _confusion_metrics(
    rows: Sequence[RecoveryRow],
    base_scales: Mapping[int, Mapping[str, float]],
    shrinkage: float,
    options: FittingOptions,
) -> Mapping[str, Any]:
    true_accept = false_accept = true_reject = false_reject = 0
    state_accept_count = execution_accept_count = joint_accept_count = 0
    for row in rows:
        accepted, state_accepted, execution_accepted = gate_accepts(
            row, base_scales, shrinkage, options
        )
        state_accept_count += int(state_accepted)
        execution_accept_count += int(execution_accepted)
        joint_accept_count += int(accepted)
        if row.recovered and accepted:
            true_accept += 1
        elif not row.recovered and accepted:
            false_accept += 1
        elif row.recovered and not accepted:
            false_reject += 1
        else:
            true_reject += 1
    recovered_count = true_accept + false_reject
    unrecovered_count = false_accept + true_reject
    if recovered_count == 0 or unrecovered_count == 0:
        raise RecoveryFitError(
            "classification metrics require recovered and unrecovered rollouts"
        )
    coverage = true_accept / float(recovered_count)
    false_accept_rate = false_accept / float(unrecovered_count)
    coverage_interval = _wilson_interval(
        true_accept, recovered_count, options.confidence
    )
    false_accept_interval = _wilson_interval(
        false_accept, unrecovered_count, options.confidence
    )
    accepted_count = true_accept + false_accept
    false_safe_among_accepted = (
        false_accept / float(accepted_count) if accepted_count else 0.0
    )
    return {
        "sample_count": len(rows),
        "recovered_count": recovered_count,
        "unrecovered_count": unrecovered_count,
        "true_accept": true_accept,
        "false_accept": false_accept,
        "true_reject": true_reject,
        "false_reject": false_reject,
        "coverage": coverage,
        "coverage_wilson_lower": coverage_interval[0],
        "coverage_wilson_upper": coverage_interval[1],
        "false_accept_rate": false_accept_rate,
        "false_accept_wilson_lower": false_accept_interval[0],
        "false_accept_wilson_upper": false_accept_interval[1],
        "false_safe_among_accepted": false_safe_among_accepted,
        "admission_rate": accepted_count / float(len(rows)),
        "state_gate_admission_rate": state_accept_count / float(len(rows)),
        "execution_gate_admission_rate": execution_accept_count / float(len(rows)),
        "joint_gate_admission_rate": joint_accept_count / float(len(rows)),
    }


def evaluate_split(
    rows: Sequence[RecoveryRow],
    split: str,
    base_scales: Mapping[int, Mapping[str, float]],
    shrinkage: float,
    options: FittingOptions,
) -> Mapping[str, Any]:
    selected = [row for row in rows if row.split == split]
    by_bin: Dict[str, Any] = {}
    for bin_start in sorted(
        {_phase_bin(row.phase_index, options.phase_bin_width) for row in selected}
    ):
        bin_rows = [
            row
            for row in selected
            if _phase_bin(row.phase_index, options.phase_bin_width) == bin_start
        ]
        by_bin[str(bin_start)] = _confusion_metrics(
            bin_rows, base_scales, shrinkage, options
        )
    return {
        "global": _confusion_metrics(selected, base_scales, shrinkage, options),
        "per_phase_bin": by_bin,
    }


def _passes_thresholds(evaluation: Mapping[str, Any], options: FittingOptions) -> bool:
    groups = [evaluation["global"]] + list(evaluation["per_phase_bin"].values())
    return all(
        group["false_accept_wilson_upper"] <= options.max_false_accept
        + ACCEPTANCE_EPSILON
        and group["coverage"] >= options.min_coverage - ACCEPTANCE_EPSILON
        for group in groups
    )


def _candidate_summary(
    shrinkage: float, evaluation: Mapping[str, Any], passed: bool
) -> Mapping[str, Any]:
    phase_values = list(evaluation["per_phase_bin"].values())
    return {
        "shrinkage": shrinkage,
        "eligible": passed,
        "coverage": evaluation["global"]["coverage"],
        "worst_phase_coverage": min(value["coverage"] for value in phase_values),
        "false_accept_rate": evaluation["global"]["false_accept_rate"],
        "false_accept_wilson_upper": evaluation["global"][
            "false_accept_wilson_upper"
        ],
        "worst_phase_false_accept_wilson_upper": max(
            value["false_accept_wilson_upper"] for value in phase_values
        ),
    }


def run_pipeline(
    rows: Sequence[RecoveryRow], options: FittingOptions
) -> PipelineResult:
    validate_options(options)
    _validate_bin_classes(rows, options)
    phases = tuple(sorted({row.phase_index for row in rows}))
    base_scales = fit_base_scales(rows, options)

    candidates: List[Mapping[str, Any]] = []
    eligible: List[Tuple[Tuple[float, float, float, float], float]] = []
    for shrinkage in options.shrinkage_grid:
        evaluation = evaluate_split(rows, "tune", base_scales, shrinkage, options)
        passed = _passes_thresholds(evaluation, options)
        summary = _candidate_summary(shrinkage, evaluation, passed)
        candidates.append(summary)
        if passed:
            score = (
                summary["coverage"],
                summary["worst_phase_coverage"],
                -summary["worst_phase_false_accept_wilson_upper"],
                -shrinkage,
            )
            eligible.append((score, shrinkage))
    if not eligible:
        raise RecoveryFitError(
            "no tune-only shrinkage satisfies the false-accept and coverage limits"
        )
    selected_shrinkage = max(eligible, key=lambda item: item[0])[1]
    held_out_evaluation = evaluate_split(
        rows, "held_out", base_scales, selected_shrinkage, options
    )
    passed = _passes_thresholds(held_out_evaluation, options)
    report = {
        "schema": REPORT_SCHEMA,
        "status": "PASS" if passed else "NO_GO",
        "claim_scope": (
            "empirical phase-indexed recovery admission on this frozen "
            "held-out dataset only"
        ),
        "safety_certificate": False,
        "formal_robot_release": False,
        "held_out_evaluation_count": 1,
        "held_out_influenced_fit": False,
        "held_out_influenced_tuning": False,
        "selected_shrinkage": selected_shrinkage,
        "thresholds": {
            "max_false_accept_wilson_upper": options.max_false_accept,
            "min_observed_recovered_coverage": options.min_coverage,
            "confidence": options.confidence,
            "scope": "global_and_each_phase_bin",
        },
        "evaluation": held_out_evaluation,
    }
    return PipelineResult(
        phases=phases,
        base_scales=base_scales,
        selected_shrinkage=selected_shrinkage,
        tuning_candidates=tuple(candidates),
        held_out_report=report,
    )


def _format_float(value: float) -> str:
    return format(float(value), ".17g")


def render_scales_csv(result: PipelineResult, options: FittingOptions) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(OUTPUT_COLUMNS)
    for phase in result.phases:
        bin_start = _phase_bin(phase, options.phase_bin_width)
        bin_end = bin_start + options.phase_bin_width - 1
        base = result.base_scales[bin_start]
        values: List[Any] = [
            phase,
            bin_start,
            bin_end,
            _format_float(result.selected_shrinkage),
        ]
        values.extend(
            _format_float(
                _scaled_value(
                    base[name], result.selected_shrinkage, options.minimum_scale
                )
            )
            for name in STATE_ERROR_COLUMNS
        )
        values.extend(
            _format_float(
                _scaled_value(
                    base[name], result.selected_shrinkage, options.minimum_scale
                )
            )
            for name in EXECUTION_ERROR_COLUMNS
        )
        writer.writerow(values)
    return stream.getvalue().encode("utf-8")


def _split_manifest(rows: Sequence[RecoveryRow], split: str) -> Mapping[str, Any]:
    selected = [row for row in rows if row.split == split]
    return {
        "row_count": len(selected),
        "rollout_count": len({row.rollout_id for row in selected}),
        "rollout_ids": sorted({row.rollout_id for row in selected}),
        "seeds": sorted({row.seed for row in selected}),
        "phase_indices": sorted({row.phase_index for row in selected}),
        "recovered_count": sum(1 for row in selected if row.recovered),
        "unrecovered_count": sum(1 for row in selected if not row.recovered),
        "canonical_rows_sha256": _canonical_split_sha256(selected),
    }


def build_manifest(
    input_path: Path,
    input_sha256: str,
    rows: Sequence[RecoveryRow],
    result: PipelineResult,
    options: FittingOptions,
    scales_sha256: str,
    report_sha256: str,
) -> Mapping[str, Any]:
    held_status = result.held_out_report["status"]
    return {
        "schema": MANIFEST_SCHEMA,
        "status": (
            "EMPIRICAL_HELD_OUT_PASS" if held_status == "PASS" else "NO_GO"
        ),
        "claim_scope": (
            "empirical recovery gate and execution-box classification for one "
            "frozen simulation dataset"
        ),
        "safety_certificate": False,
        "robust_invariant_set": False,
        "formal_robot_release": False,
        "physical_enforce_authorized": False,
        "input": {
            "path": str(input_path.resolve()),
            "sha256": input_sha256,
            "schema": SCHEMA,
            "columns": list(INPUT_COLUMNS),
        },
        "compiled_contract": {
            "state_width": COMPILED_STATE_WIDTH,
            "gate_contract": GATE_CONTRACT,
            "gate_radius_count": COMPILED_GATE_RADIUS_COUNT,
            "state_error_columns": list(STATE_ERROR_COLUMNS),
            "execution_compatibility_contract": EXECUTION_COMPATIBILITY_CONTRACT,
            "execution_bound_count": COMPILED_EXECUTION_BOUND_COUNT,
            "linear_pending_count": COMPILED_LINEAR_PENDING_COUNT,
            "angular_pending_count": COMPILED_ANGULAR_PENDING_COUNT,
            "execution_error_columns": list(EXECUTION_ERROR_COLUMNS),
            "minimum_denominator": COMPILED_MINIMUM_DENOMINATOR,
        },
        "split_contract": {
            "unit": "complete_rollout_and_seed",
            "mutually_exclusive": True,
            "fit": _split_manifest(rows, "fit"),
            "tune": _split_manifest(rows, "tune"),
            "held_out": _split_manifest(rows, "held_out"),
        },
        "fit": {
            "method": "recovered_fit_coordinate_envelope_v1",
            "state_radius_multiplier": math.sqrt(len(STATE_ERROR_COLUMNS)),
            "phase_bin_width": options.phase_bin_width,
            "minimum_scale": options.minimum_scale,
            "uses_only_split": "fit",
        },
        "tune": {
            "method": "global_conservative_shrinkage_v1",
            "uses_only_split": "tune",
            "selected_shrinkage": result.selected_shrinkage,
            "candidate_grid": list(options.shrinkage_grid),
            "candidate_results": list(result.tuning_candidates),
            "max_false_accept_wilson_upper": options.max_false_accept,
            "min_observed_recovered_coverage": options.min_coverage,
            "confidence": options.confidence,
        },
        "held_out": {
            "uses_only_split": "held_out",
            "evaluation_count": 1,
            "status": held_status,
            "report": "held_out_report.json",
            "report_sha256": report_sha256,
        },
        "outputs": {
            "scales": {
                "path": "phase_rejoin_recovery_radii_bounds.csv",
                "schema": SCALE_SCHEMA,
                "sha256": scales_sha256,
            },
            "held_out_report": {
                "path": "held_out_report.json",
                "schema": REPORT_SCHEMA,
                "sha256": report_sha256,
            },
        },
        "tool": {
            "path": str(Path(__file__).resolve()),
            "sha256": sha256_file(Path(__file__).resolve()),
        },
    }


def _write_exclusive(path: Path, contents: bytes) -> None:
    try:
        with path.open("xb") as stream:
            stream.write(contents)
            stream.flush()
    except OSError as error:
        raise RecoveryFitError("cannot create output {}".format(path)) from error


def write_outputs(
    out_dir: Path,
    input_path: Path,
    input_sha256: str,
    rows: Sequence[RecoveryRow],
    result: PipelineResult,
    options: FittingOptions,
) -> Mapping[str, Any]:
    scales_bytes = render_scales_csv(result, options)
    report_value = dict(result.held_out_report)
    report_value["input_sha256"] = input_sha256
    report_value["gate_contract"] = GATE_CONTRACT
    report_value["execution_compatibility_contract"] = (
        EXECUTION_COMPATIBILITY_CONTRACT
    )
    report_bytes = _json_bytes(report_value)
    scales_sha256 = _sha256_bytes(scales_bytes)
    report_sha256 = _sha256_bytes(report_bytes)
    manifest = build_manifest(
        input_path,
        input_sha256,
        rows,
        result,
        options,
        scales_sha256,
        report_sha256,
    )
    manifest_bytes = _json_bytes(manifest)
    manifest_sha256 = _sha256_bytes(manifest_bytes)
    sidecar_bytes = (manifest_sha256 + "  manifest.json\n").encode("ascii")

    resolved = out_dir.resolve()
    if resolved.exists():
        raise RecoveryFitError("output directory already exists")
    try:
        resolved.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        raise RecoveryFitError(
            "cannot create output directory {}".format(resolved)
        ) from error
    _write_exclusive(resolved / "phase_rejoin_recovery_radii_bounds.csv", scales_bytes)
    _write_exclusive(resolved / "held_out_report.json", report_bytes)
    _write_exclusive(resolved / "manifest.json", manifest_bytes)
    _write_exclusive(resolved / "manifest.sha256", sidecar_bytes)
    return manifest


def verify_manifest(path: Path) -> Mapping[str, Any]:
    path = path.resolve()
    try:
        manifest_bytes = path.read_bytes()
        manifest = json.loads(manifest_bytes.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RecoveryFitError("cannot parse manifest {}".format(path)) from error
    if not isinstance(manifest, dict) or manifest.get("schema") != MANIFEST_SCHEMA:
        raise RecoveryFitError("unsupported recovery-fit manifest schema")
    sidecar = path.with_suffix(".sha256")
    try:
        sidecar_text = sidecar.read_text(encoding="ascii")
    except OSError as error:
        raise RecoveryFitError("manifest SHA sidecar is missing") from error
    expected_line = _sha256_bytes(manifest_bytes) + "  " + path.name + "\n"
    if sidecar_text != expected_line:
        raise RecoveryFitError("manifest SHA sidecar mismatch")

    input_entry = manifest.get("input")
    if not isinstance(input_entry, dict):
        raise RecoveryFitError("manifest input binding is missing")
    input_path = Path(str(input_entry.get("path", "")))
    if not input_path.is_file() or sha256_file(input_path) != input_entry.get("sha256"):
        raise RecoveryFitError("input recovery CSV hash mismatch")

    outputs = manifest.get("outputs")
    if not isinstance(outputs, dict):
        raise RecoveryFitError("manifest outputs are missing")
    for label in ("scales", "held_out_report"):
        entry = outputs.get(label)
        if not isinstance(entry, dict):
            raise RecoveryFitError("manifest output {} is missing".format(label))
        relative = Path(str(entry.get("path", "")))
        if relative.is_absolute() or ".." in relative.parts or len(relative.parts) != 1:
            raise RecoveryFitError("manifest output path is not a safe relative file")
        output_path = path.parent / relative
        if not output_path.is_file() or sha256_file(output_path) != entry.get("sha256"):
            raise RecoveryFitError("{} hash mismatch".format(label))

    try:
        report = json.loads((path.parent / "held_out_report.json").read_text("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RecoveryFitError("held-out report is unreadable") from error
    held_out = manifest.get("held_out")
    if (
        not isinstance(report, dict)
        or report.get("schema") != REPORT_SCHEMA
        or not isinstance(held_out, dict)
        or report.get("status") != held_out.get("status")
        or report.get("held_out_evaluation_count") != 1
        or report.get("safety_certificate") is not False
        or manifest.get("safety_certificate") is not False
    ):
        raise RecoveryFitError("held-out report contract mismatch")
    return manifest


def parse_shrinkage_grid(text: str) -> Tuple[float, ...]:
    values: List[float] = []
    for item in text.split(","):
        clean = item.strip()
        if not clean:
            raise RecoveryFitError("shrinkage grid contains an empty item")
        try:
            values.append(float(clean))
        except ValueError as error:
            raise RecoveryFitError(
                "invalid shrinkage grid item {}".format(clean)
            ) from error
    return tuple(values)


def _default_grid_text() -> str:
    return ",".join(_format_float(value) for value in DEFAULT_SHRINKAGE_GRID)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    fit = commands.add_parser("fit", help="fit, tune, and evaluate one frozen CSV")
    fit.add_argument("--input", type=Path, required=True)
    fit.add_argument("--out-dir", type=Path, required=True)
    fit.add_argument("--phase-bin-width", type=int, default=1)
    fit.add_argument("--shrinkage-grid", default=_default_grid_text())
    fit.add_argument("--max-false-accept", type=float, default=0.05)
    fit.add_argument("--min-coverage", type=float, default=0.50)
    fit.add_argument("--confidence", type=float, default=0.95)
    fit.add_argument(
        "--minimum-scale",
        type=float,
        default=COMPILED_MINIMUM_DENOMINATOR,
    )
    verify = commands.add_parser("verify", help="verify manifest and bound files")
    verify.add_argument("--manifest", type=Path, required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "verify":
            manifest = verify_manifest(args.manifest)
            print(
                "VALID {} status={} safety_certificate=false".format(
                    MANIFEST_SCHEMA, manifest["status"]
                )
            )
            return 0

        input_path = args.input.resolve()
        input_sha256 = sha256_file(input_path)
        rows = load_recovery_csv(input_path)
        options = FittingOptions(
            phase_bin_width=args.phase_bin_width,
            shrinkage_grid=parse_shrinkage_grid(args.shrinkage_grid),
            max_false_accept=args.max_false_accept,
            min_coverage=args.min_coverage,
            confidence=args.confidence,
            minimum_scale=args.minimum_scale,
        )
        result = run_pipeline(rows, options)
        if sha256_file(input_path) != input_sha256:
            raise RecoveryFitError("input recovery CSV changed during evaluation")
        manifest = write_outputs(
            args.out_dir, input_path, input_sha256, rows, result, options
        )
        print(
            "{} shrinkage={} held_out={} safety_certificate=false".format(
                manifest["status"],
                _format_float(result.selected_shrinkage),
                result.held_out_report["status"],
            )
        )
        return 0 if result.held_out_report["status"] == "PASS" else 4
    except (OSError, RecoveryFitError) as error:
        print("recovery fit rejected: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
