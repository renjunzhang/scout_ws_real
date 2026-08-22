#!/usr/bin/env python3
"""Analyze one paired Phase-Rejoining simulation campaign at trial level.

The input directory must contain exactly one
``spmpc_closed_loop_trial_summary_v1`` JSON document for every
``(seed, condition)`` pair in C0--C4 and IS.  Cycle logs are deliberately not
accepted: the independent motion-plus-fixed-tail trial is the statistical
unit.  Failed trials stay in the paired data and can never be silently
discarded.

Pilot output is an exploratory readiness report only.  Formal output can say
PASS only when every summary is bound to the explicitly supplied frozen
session hash, all 16 pre-registered paired blocks are present, no trial failed,
and the frozen paired-bootstrap effect and task contracts pass.  This script
does not turn a pilot campaign into publishable evidence.
"""

from __future__ import annotations

import argparse
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import random
import statistics
import sys
import tempfile
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


SUMMARY_SCHEMA = "spmpc_closed_loop_trial_summary_v1"
ANALYSIS_SCHEMA = "spmpc_phase_rejoin_simulation_campaign_analysis_v2"
REQUIRED_CONDITIONS = ("C0", "C1", "C2", "C3", "C4", "IS")
EXPECTED_MODES = {
    "C0": "ordinary_mpcc",
    "C1": "smooth_match_mpcc",
    "C2": "offline_replay",
    "C3": "residual_no_gate",
    "C4": "phase_rejoin_full",
    "IS": "input_shaping",
}
COMPARATORS = ("C0", "C1", "C3")
FORMAL_PAIRED_BLOCKS = 16
FORMAL_SEEDS = tuple(range(3101, 3117))
MINIMUM_MEANINGFUL_DIFFERENCE_M = 0.0005
BOOTSTRAP_REPLICATES = 10000
BOOTSTRAP_SEED = 20260822
COMPLETION_TIME_NONINFERIORITY_RELATIVE = 0.10
TRACKING_Q95_NONINFERIORITY_M = 0.05
MAX_UINT32 = (1 << 32) - 1


class CampaignError(ValueError):
    """A campaign violates the frozen trial-level analysis contract."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_text(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _object_without_duplicate_keys(
    pairs: Iterable[Tuple[str, Any]],
) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise CampaignError("duplicate JSON key: {}".format(key))
        result[key] = value
    return result


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(
                stream, object_pairs_hook=_object_without_duplicate_keys
            )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CampaignError("cannot read {}: {}".format(path, error)) from error
    if not isinstance(value, dict):
        raise CampaignError("summary root is not an object: {}".format(path))
    return value


def _exact_bool(value: Any, label: str) -> bool:
    if type(value) is not bool:  # bool is intentionally stricter than 0/1.
        raise CampaignError("{} must be a JSON boolean".format(label))
    return value


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise CampaignError("{} must be a non-empty string".format(label))
    return value


def _finite_number(value: Any, label: str, minimum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CampaignError("{} must be a finite number".format(label))
    number = float(value)
    if not math.isfinite(number):
        raise CampaignError("{} must be a finite number".format(label))
    if minimum is not None and number < minimum:
        raise CampaignError("{} must be >= {}".format(label, minimum))
    return number


def _required_mapping(owner: Mapping[str, Any], key: str, label: str) -> Dict[str, Any]:
    value = owner.get(key)
    if not isinstance(value, dict):
        raise CampaignError("{} must be an object".format(label))
    return value


def _validate_status(root: Mapping[str, Any], trial: Mapping[str, Any], label: str) -> bool:
    status = root.get("status")
    if status not in (
        "TRIAL_COMPLETE",
        "TRIAL_COMPLETE_WITH_FAILURE",
        "RUNTIME_ERROR",
    ):
        raise CampaignError("{}.status is invalid".format(label))
    sequence_completed = _exact_bool(
        trial.get("sequence_completed"),
        "{}.trial.sequence_completed".format(label),
    )
    task_success = _exact_bool(
        trial.get("task_success"), "{}.trial.task_success".format(label)
    )
    runtime_error = trial.get("runtime_error")
    if not isinstance(runtime_error, str):
        raise CampaignError("{}.trial.runtime_error must be a string".format(label))
    if status == "TRIAL_COMPLETE" and (
        not sequence_completed or not task_success or runtime_error
    ):
        raise CampaignError("{} has inconsistent successful status".format(label))
    if status == "TRIAL_COMPLETE_WITH_FAILURE" and (task_success or runtime_error):
        raise CampaignError("{} has inconsistent failed status".format(label))
    if status == "RUNTIME_ERROR" and (task_success or not runtime_error):
        raise CampaignError("{} has inconsistent runtime-error status".format(label))
    return status == "TRIAL_COMPLETE" and sequence_completed and task_success


def _validate_truth_isolation(root: Mapping[str, Any], label: str) -> None:
    required_true = (
        "simulation_only",
        "final_command_transaction",
        "dual_channel_execution_model",
        "plant_controller_parameter_independence",
    )
    required_false = (
        "formal_robot_release",
        "physical_parameter_claim",
        "plant_truth_visible_to_controller",
        "external_liquid_truth_used_for_control",
    )
    for key in required_true:
        if not _exact_bool(root.get(key), "{}.{}".format(label, key)):
            raise CampaignError("{}.{} must be true".format(label, key))
    for key in required_false:
        if _exact_bool(root.get(key), "{}.{}".format(label, key)):
            raise CampaignError("{}.{} must be false".format(label, key))
    if root.get("controller_observer_source") != "plant_motion_derived_odom":
        raise CampaignError(
            "{} controller observer is not isolated from liquid truth".format(label)
        )
    if root.get("command_history_source") != "final_published_command":
        raise CampaignError(
            "{} command history is not the final published command".format(label)
        )


def _validate_primary_metric(root: Mapping[str, Any], label: str) -> Tuple[float, int]:
    metric = _required_mapping(root, "primary_metric", "{}.primary_metric".format(label))
    if metric.get("name") != "external_measured_height_q95_m":
        raise CampaignError("{} does not use external measured liquid height".format(label))
    if metric.get("window") != "motion_plus_fixed_tail":
        raise CampaignError("{} does not use the motion-plus-fixed-tail window".format(label))
    if metric.get("statistics_unit") != "complete_trial":
        raise CampaignError(
            "{} attempts to use a non-trial statistical unit".format(label)
        )
    if metric.get("quantile_method") != "nearest_rank":
        raise CampaignError("{} primary quantile method changed".format(label))
    sample_count = metric.get("sample_count")
    if (
        isinstance(sample_count, bool)
        or not isinstance(sample_count, int)
        or sample_count <= 0
    ):
        raise CampaignError("{} has no primary-window samples".format(label))
    value = _finite_number(metric.get("value_m"), "{}.primary_metric.value_m".format(label), 0.0)
    return value, sample_count


def _validate_summary(
    root: Mapping[str, Any],
    path: Path,
    stage: str,
    frozen_session_sha256: Optional[str],
) -> Dict[str, Any]:
    label = path.name
    if root.get("schema") != SUMMARY_SCHEMA:
        raise CampaignError("{} has the wrong schema".format(label))
    if not _exact_bool(root.get("implementation_complete"), "{}.implementation_complete".format(label)):
        raise CampaignError("{} is not implementation-complete".format(label))

    condition = root.get("condition_id")
    if condition not in REQUIRED_CONDITIONS:
        raise CampaignError("{} has unknown condition_id".format(label))
    if root.get("mode") != EXPECTED_MODES[condition]:
        raise CampaignError("{} condition/mode semantics changed".format(label))
    implementation_id = _nonempty_string(
        root.get("implementation_id"), "{}.implementation_id".format(label)
    )
    seed = root.get("seed")
    if (
        isinstance(seed, bool)
        or not isinstance(seed, int)
        or seed < 0
        or seed > MAX_UINT32
    ):
        raise CampaignError("{}.seed is not a uint32".format(label))

    _validate_truth_isolation(root, label)
    trial = _required_mapping(root, "trial", "{}.trial".format(label))
    success = _validate_status(root, trial, label)
    completion_time_sec = _finite_number(
        trial.get("motion_end_sec"),
        "{}.trial.motion_end_sec".format(label),
        0.0,
    )
    if completion_time_sec <= 0.0:
        raise CampaignError("{} completion time must be non-zero".format(label))
    fixed_tail_sec = _finite_number(
        trial.get("fixed_tail_sec"), "{}.trial.fixed_tail_sec".format(label), 0.0
    )
    if fixed_tail_sec <= 0.0:
        raise CampaignError("{} fixed tail must be non-zero".format(label))
    primary_value_m, sample_count = _validate_primary_metric(root, label)
    secondary = _required_mapping(
        root, "secondary_metrics", "{}.secondary_metrics".format(label)
    )
    tracking_q95_m = _finite_number(
        secondary.get("tracking_q95_m"),
        "{}.secondary_metrics.tracking_q95_m".format(label),
        0.0,
    )

    plant_freeze_id = _nonempty_string(
        root.get("plant_freeze_id"), "{}.plant_freeze_id".format(label)
    )
    artifact_contract_id = _nonempty_string(
        root.get("artifact_contract_id"), "{}.artifact_contract_id".format(label)
    )
    if stage == "formal":
        summary_session_hash = root.get("frozen_session_sha256")
        if not _sha256_text(summary_session_hash):
            raise CampaignError(
                "{} is not bound to a frozen session SHA-256".format(label)
            )
        if summary_session_hash != frozen_session_sha256:
            raise CampaignError("{} frozen session SHA-256 mismatch".format(label))

    baseline = root.get("baseline_contract")
    c3_c4_causal_ready = False
    if isinstance(baseline, dict):
        c3_c4_causal_ready = (
            baseline.get("formal_c3_c4_causal_comparison_ready") is True
            and baseline.get("c3_exact_c4_optimizer_match") is True
        )

    return {
        "source_file": path.name,
        "source_sha256": _sha256_file(path),
        "condition_id": condition,
        "mode": EXPECTED_MODES[condition],
        "implementation_id": implementation_id,
        "seed": seed,
        "status": root["status"],
        "completed_successfully": success,
        "failure_retained": not success,
        "primary_value_m": primary_value_m,
        "primary_sample_count": sample_count,
        "completion_time_sec": completion_time_sec,
        "tracking_q95_m": tracking_q95_m,
        "fixed_tail_sec": fixed_tail_sec,
        "plant_freeze_id": plant_freeze_id,
        "artifact_contract_id": artifact_contract_id,
        "c3_c4_causal_ready": c3_c4_causal_ready,
    }


def _median(values: Sequence[float]) -> float:
    if not values:
        raise CampaignError("cannot compute a median from an empty trial set")
    return float(statistics.median(values))


def _nearest_rank(values: Sequence[float], probability: float) -> float:
    if not values:
        raise CampaignError("cannot compute a quantile from an empty sample")
    ordered = sorted(float(value) for value in values)
    rank = max(1, int(math.ceil(probability * len(ordered))))
    return ordered[min(rank - 1, len(ordered) - 1)]


def _paired_bootstrap_mean_interval(
    values: Sequence[float], seed_offset: int
) -> Dict[str, Any]:
    if not values:
        raise CampaignError("cannot bootstrap an empty paired sample")
    numbers = [float(value) for value in values]
    generator = random.Random(BOOTSTRAP_SEED + seed_offset)
    estimates = []
    for _ in range(BOOTSTRAP_REPLICATES):
        estimates.append(
            statistics.fmean(
                numbers[generator.randrange(len(numbers))]
                for _ in range(len(numbers))
            )
        )
    return {
        "estimator": "paired_mean",
        "method": "percentile_bootstrap_95pct",
        "replicates": BOOTSTRAP_REPLICATES,
        "rng_seed": BOOTSTRAP_SEED + seed_offset,
        "estimate": statistics.fmean(numbers),
        "lower": _nearest_rank(estimates, 0.025),
        "upper": _nearest_rank(estimates, 0.975),
    }


def _condition_summary(records: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    values = [float(record["primary_value_m"]) for record in records]
    failed = sum(not bool(record["completed_successfully"]) for record in records)
    return {
        "trial_count": len(records),
        "successful_trial_count": len(records) - failed,
        "failed_trial_count": failed,
        "failure_inclusive": True,
        "primary_metric_median_m": _median(values),
        "primary_metric_min_m": min(values),
        "primary_metric_max_m": max(values),
    }


def _paired_comparison(
    by_key: Mapping[Tuple[int, str], Mapping[str, Any]],
    seeds: Sequence[int],
    comparator: str,
) -> Dict[str, Any]:
    pairs: List[Dict[str, Any]] = []
    for seed in seeds:
        c4 = by_key[(seed, "C4")]
        baseline = by_key[(seed, comparator)]
        difference = float(c4["primary_value_m"]) - float(
            baseline["primary_value_m"]
        )
        pairs.append(
            {
                "seed": seed,
                "c4_value_m": c4["primary_value_m"],
                "comparator_value_m": baseline["primary_value_m"],
                "difference_m": difference,
                "failure_retained": (
                    not bool(c4["completed_successfully"])
                    or not bool(baseline["completed_successfully"])
                ),
            }
        )
    differences = [float(pair["difference_m"]) for pair in pairs]
    comparison_offset = COMPARATORS.index(comparator)
    completion_relative = [
        (
            float(by_key[(seed, "C4")]["completion_time_sec"])
            - float(by_key[(seed, comparator)]["completion_time_sec"])
        )
        / float(by_key[(seed, comparator)]["completion_time_sec"])
        for seed in seeds
    ]
    tracking_differences = [
        float(by_key[(seed, "C4")]["tracking_q95_m"])
        - float(by_key[(seed, comparator)]["tracking_q95_m"])
        for seed in seeds
    ]
    return {
        "definition": "C4_minus_{}".format(comparator),
        "lower_is_better": True,
        "pairing_key": "seed",
        "statistics_unit": "paired_complete_trial",
        "pair_count": len(pairs),
        "failed_pair_count": sum(bool(pair["failure_retained"]) for pair in pairs),
        "c4_lower_count": sum(value < 0.0 for value in differences),
        "equal_count": sum(value == 0.0 for value in differences),
        "c4_higher_count": sum(value > 0.0 for value in differences),
        "median_difference_m": _median(differences),
        "mean_difference_m": statistics.fmean(differences),
        "primary_paired_bootstrap_95pct":
            _paired_bootstrap_mean_interval(
                differences, 100 + comparison_offset
            ),
        "completion_time_relative_paired_bootstrap_95pct":
            _paired_bootstrap_mean_interval(
                completion_relative, 200 + comparison_offset
            ),
        "tracking_q95_difference_paired_bootstrap_95pct":
            _paired_bootstrap_mean_interval(
                tracking_differences, 300 + comparison_offset
            ),
        "pairs": pairs,
    }


def analyze_campaign(
    input_dir: Path,
    stage: str,
    frozen_session_sha256: Optional[str] = None,
    preregistered_min_effect_m: Optional[float] = None,
    excluded_output: Optional[Path] = None,
) -> Dict[str, Any]:
    """Validate and analyze a complete paired directory without writing it."""
    if stage not in ("pilot", "formal"):
        raise CampaignError("stage must be pilot or formal")
    input_dir = input_dir.resolve()
    if not input_dir.is_dir():
        raise CampaignError("input directory does not exist: {}".format(input_dir))

    threshold: Optional[float] = None
    if preregistered_min_effect_m is not None:
        threshold = _finite_number(
            preregistered_min_effect_m, "preregistered minimum effect", 0.0
        )
        if threshold <= 0.0:
            raise CampaignError("preregistered minimum effect must be > 0")
    if frozen_session_sha256 is not None and not _sha256_text(frozen_session_sha256):
        raise CampaignError("frozen session SHA-256 must be 64 lowercase hex characters")
    if stage == "formal":
        if frozen_session_sha256 is None:
            raise CampaignError("formal analysis requires --frozen-session-sha256")
        if threshold is None:
            raise CampaignError("formal analysis requires --preregistered-min-effect-m")
        if not math.isclose(
            threshold,
            MINIMUM_MEANINGFUL_DIFFERENCE_M,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise CampaignError(
                "formal minimum effect differs from frozen session contract"
            )

    excluded = excluded_output.resolve() if excluded_output is not None else None
    paths = sorted(
        path
        for path in input_dir.iterdir()
        if path.is_file()
        and path.suffix.lower() == ".json"
        and (excluded is None or path.resolve() != excluded)
    )
    if not paths:
        raise CampaignError("input directory contains no trial summary JSON files")

    records: List[Dict[str, Any]] = []
    by_key: Dict[Tuple[int, str], Dict[str, Any]] = {}
    for path in paths:
        record = _validate_summary(
            _load_json(path), path, stage, frozen_session_sha256
        )
        key = (int(record["seed"]), str(record["condition_id"]))
        if key in by_key:
            raise CampaignError(
                "duplicate trial for seed {} condition {}".format(*key)
            )
        by_key[key] = record
        records.append(record)

    seeds = sorted({int(record["seed"]) for record in records})
    expected_keys = {
        (seed, condition) for seed in seeds for condition in REQUIRED_CONDITIONS
    }
    actual_keys = set(by_key)
    if actual_keys != expected_keys:
        missing = sorted(expected_keys - actual_keys)
        extra = sorted(actual_keys - expected_keys)
        raise CampaignError(
            "campaign is not a complete seed-by-condition grid; missing={}, extra={}".format(
                missing, extra
            )
        )

    implementation_by_condition: Dict[str, str] = {}
    for condition in REQUIRED_CONDITIONS:
        identifiers = {
            str(by_key[(seed, condition)]["implementation_id"]) for seed in seeds
        }
        if len(identifiers) != 1:
            raise CampaignError(
                "condition {} changes implementation across paired seeds".format(condition)
            )
        implementation_by_condition[condition] = next(iter(identifiers))
    if len(set(implementation_by_condition.values())) != len(REQUIRED_CONDITIONS):
        raise CampaignError("conditions do not have distinct implementation_id values")

    for field in ("plant_freeze_id", "artifact_contract_id", "fixed_tail_sec"):
        values = {record[field] for record in records}
        if len(values) != 1:
            raise CampaignError("paired trials change {}".format(field))

    condition_summaries = {
        condition: _condition_summary(
            [by_key[(seed, condition)] for seed in seeds]
        )
        for condition in REQUIRED_CONDITIONS
    }
    comparisons = {
        "C4_minus_{}".format(comparator): _paired_comparison(
            by_key, seeds, comparator
        )
        for comparator in COMPARATORS
    }
    failed_trials = [record for record in records if record["failure_retained"]]
    c3_c4_causal_ready = all(
        bool(by_key[(seed, condition)]["c3_c4_causal_ready"])
        for seed in seeds
        for condition in ("C3", "C4")
    )

    if stage == "pilot":
        development_gaps = ["pilot_data_cannot_authorize_a_formal_claim"]
        if failed_trials:
            development_gaps.append("failed_trials_are_present_and_retained")
        if tuple(seeds) != FORMAL_SEEDS:
            development_gaps.append("formal_paired_block_count_not_met")
        if not c3_c4_causal_ready:
            development_gaps.append("c3_c4_causal_contract_not_ready")
        decision = {
            "status": "PILOT_READY_FOR_DEVELOPMENT_REVIEW",
            "paper_claim_authorized": False,
            "formal_pass": False,
            "claim_scope": "none",
            "interpretation": "exploratory_pilot_only",
            "failed_trials_retained": len(failed_trials),
            "development_gaps": development_gaps,
            "note": (
                "Direction and effect sizes are diagnostic only; this pilot "
                "cannot authorize a paper claim."
            ),
        }
    else:
        assert threshold is not None
        checks = {
            "exact_formal_paired_block_count":
                tuple(seeds) == FORMAL_SEEDS,
            "no_failed_trials": not failed_trials,
            "c4_vs_c0_preregistered_effect": (
                comparisons["C4_minus_C0"]
                ["primary_paired_bootstrap_95pct"]["upper"] <= -threshold
            ),
            "c4_vs_c1_preregistered_effect": (
                comparisons["C4_minus_C1"]
                ["primary_paired_bootstrap_95pct"]["upper"] <= -threshold
            ),
            "c4_vs_c3_preregistered_effect": (
                comparisons["C4_minus_C3"]
                ["primary_paired_bootstrap_95pct"]["upper"] <= -threshold
            ),
            "c4_vs_c0_completion_time_noninferior": (
                comparisons["C4_minus_C0"]
                ["completion_time_relative_paired_bootstrap_95pct"]["upper"]
                <= COMPLETION_TIME_NONINFERIORITY_RELATIVE
            ),
            "c4_vs_c1_completion_time_noninferior": (
                comparisons["C4_minus_C1"]
                ["completion_time_relative_paired_bootstrap_95pct"]["upper"]
                <= COMPLETION_TIME_NONINFERIORITY_RELATIVE
            ),
            "c4_vs_c0_tracking_q95_noninferior": (
                comparisons["C4_minus_C0"]
                ["tracking_q95_difference_paired_bootstrap_95pct"]["upper"]
                <= TRACKING_Q95_NONINFERIORITY_M
            ),
            "c4_vs_c1_tracking_q95_noninferior": (
                comparisons["C4_minus_C1"]
                ["tracking_q95_difference_paired_bootstrap_95pct"]["upper"]
                <= TRACKING_Q95_NONINFERIORITY_M
            ),
            "c3_c4_causal_contract_ready": c3_c4_causal_ready,
        }
        formal_pass = all(checks.values())
        decision = {
            "status": "PASS" if formal_pass else "FAIL",
            "paper_claim_authorized": formal_pass,
            "formal_pass": formal_pass,
            "claim_scope": (
                "independent_simulation_campaign_only"
                if formal_pass
                else "none"
            ),
            "interpretation": "frozen_formal_paired_trial_result",
            "preregistered_rule": (
                "upper95(paired-bootstrap mean(C4-comparator)) <= "
                "-preregistered_min_effect_m; C0, C1, and causal C3 "
                "comparisons plus C0/C1 task noninferiority must pass"
            ),
            "preregistered_min_effect_m": threshold,
            "completion_time_noninferiority_relative":
                COMPLETION_TIME_NONINFERIORITY_RELATIVE,
            "tracking_q95_noninferiority_m":
                TRACKING_Q95_NONINFERIORITY_M,
            "checks": checks,
            "failed_trials_retained": len(failed_trials),
        }

    return {
        "schema": ANALYSIS_SCHEMA,
        "stage": stage,
        "analysis_unit": "paired_complete_trial_by_seed",
        "formal_paired_blocks": FORMAL_PAIRED_BLOCKS,
        "formal_seeds": list(FORMAL_SEEDS),
        "cycle_samples_used_as_independent_observations": False,
        "primary_metric": {
            "name": "external_measured_height_q95_m",
            "window": "motion_plus_fixed_tail",
            "statistics_unit": "complete_trial",
            "failure_inclusive": True,
        },
        "frozen_session_sha256": frozen_session_sha256,
        "preregistered_min_effect_m": threshold,
        "campaign": {
            "seed_count": len(seeds),
            "seeds": seeds,
            "condition_ids": list(REQUIRED_CONDITIONS),
            "trial_count": len(records),
            "expected_trial_count": len(seeds) * len(REQUIRED_CONDITIONS),
            "failed_trial_count": len(failed_trials),
            "implementation_ids": implementation_by_condition,
            "plant_freeze_id": records[0]["plant_freeze_id"],
            "artifact_contract_id": records[0]["artifact_contract_id"],
            "c3_c4_causal_contract_ready": c3_c4_causal_ready,
        },
        "condition_summaries": condition_summaries,
        "paired_comparisons": comparisons,
        "trial_records": sorted(
            records, key=lambda record: (record["seed"], REQUIRED_CONDITIONS.index(record["condition_id"]))
        ),
        "decision": decision,
    }


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(
        str(path), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_json_create_new(path: Path, report: Mapping[str, Any]) -> None:
    """Durably publish a complete JSON file without overwriting evidence."""
    contents = (
        json.dumps(
            report,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".{}.tmp.".format(path.name), dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    published_identity: Optional[Tuple[int, int]] = None
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(contents)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(str(temporary), str(path))
        except OSError as error:
            if error.errno == errno.EEXIST:
                raise CampaignError("output already exists: {}".format(path)) from error
            raise CampaignError("cannot publish output: {}".format(path)) from error
        status = path.stat()
        published_identity = (status.st_dev, status.st_ino)
        _fsync_directory(path.parent)
    except CampaignError:
        raise
    except OSError as error:
        if published_identity is not None:
            try:
                status = path.stat()
                if (status.st_dev, status.st_ino) == published_identity:
                    path.unlink()
                    _fsync_directory(path.parent)
            except FileNotFoundError:
                pass
        raise CampaignError("cannot create output: {}".format(path)) from error
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("pilot", "formal"), required=True)
    parser.add_argument("--frozen-session-sha256")
    parser.add_argument("--preregistered-min-effect-m", type=float)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report = analyze_campaign(
            args.input_dir,
            args.stage,
            args.frozen_session_sha256,
            args.preregistered_min_effect_m,
            args.output,
        )
        write_json_create_new(args.output, report)
    except CampaignError as error:
        print("campaign analysis rejected: {}".format(error), file=sys.stderr)
        return 2
    print(
        "{}: {}".format(args.stage, report["decision"]["status"])
    )
    if args.stage == "formal" and not report["decision"]["formal_pass"]:
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
