#!/usr/bin/env python3
"""Apply the frozen one-shot S-MPCC/BT Go/No-Go decision rule."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import sys


SEEDS = (9911, 9912, 9913, 9914, 9915)
DISTURBANCES = ("D1_INITIAL_POSE", "D2_SHORT_LINEAR_SPEED_CAP")
TRACKING_MEDIAN_IMPROVEMENT_MIN = 0.10
TRACKING_IMPROVED_SEEDS_MIN = 4
FULL_HEIGHT_NI_MARGIN_M = 0.000010
TAIL_HEIGHT_NI_MARGIN_M = 0.000025
COMPLETION_TIME_RATIO_MAX = 1.10
EFFECTIVE_CORRECTION_FRACTION_MIN = 0.10


def optional_finite_number(value: object) -> float | None:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return None
    return float(value)


def median(values: list[float]) -> float:
    return float(statistics.median(values))


def optional_median(values: list[float]) -> float | None:
    return median(values) if values else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite decision: {args.output}")

    summaries: dict[tuple[str, str, int], dict] = {}
    for path in sorted(args.input_dir.rglob("summary.json")):
        summary = json.loads(path.read_text(encoding="utf-8"))
        disturbance = summary.get("go_no_go_disturbance", {}).get("id")
        mode = summary.get("mode")
        seed = summary.get("seed")
        method = None
        if mode == "bounded_tracking" and disturbance in DISTURBANCES:
            method = "BT"
        elif mode == "smpcc_bt_direct" and disturbance in DISTURBANCES:
            method = "SMPCC_BT"
        if method is None or seed not in SEEDS:
            continue
        key = (method, disturbance, int(seed))
        if key in summaries:
            raise SystemExit(f"duplicate paired summary {key}")
        summaries[key] = summary

    expected = {
        (method, disturbance, seed)
        for method in ("BT", "SMPCC_BT")
        for disturbance in DISTURBANCES
        for seed in SEEDS
    }
    if set(summaries) != expected:
        missing = sorted(expected - set(summaries))
        extra = sorted(set(summaries) - expected)
        print(json.dumps({"missing": missing, "extra": extra}), file=sys.stderr)
        return 3

    rows: list[dict] = []
    global_reasons: list[str] = []
    for disturbance in DISTURBANCES:
        tracking_improvements: list[float] = []
        full_height_deltas: list[float] = []
        tail_height_deltas: list[float] = []
        time_ratios: list[float] = []
        correction_fractions: list[float] = []
        completed_bt = 0
        completed_smpcc = 0
        controlled_stops_smpcc = 0
        candidate_failures = 0
        paired: list[dict] = []
        for seed in SEEDS:
            bt = summaries[("BT", disturbance, seed)]
            smpcc = summaries[("SMPCC_BT", disturbance, seed)]
            bt_complete = bool(bt["trial"]["sequence_completed"] and
                               bt["trial"]["task_success"])
            smpcc_complete = bool(smpcc["trial"]["sequence_completed"] and
                                  smpcc["trial"]["task_success"])
            completed_bt += int(bt_complete)
            completed_smpcc += int(smpcc_complete)
            controlled_stops_smpcc += int(
                smpcc["controller_audit"]["controlled_stops"])
            candidate_failures += int(
                smpcc["smpcc_bt_go_no_go_audit"]["candidate_failures"])

            bt_tracking = optional_finite_number(
                bt["secondary_metrics"]["tracking_q95_m"])
            smpcc_tracking = optional_finite_number(
                smpcc["secondary_metrics"]["tracking_q95_m"])
            improvement = None
            if (bt_tracking is not None and bt_tracking > 0.0 and
                    smpcc_tracking is not None):
                improvement = (bt_tracking - smpcc_tracking) / bt_tracking
                tracking_improvements.append(improvement)

            bt_full = optional_finite_number(bt["primary_metric"]["value_m"])
            smpcc_full = optional_finite_number(
                smpcc["primary_metric"]["value_m"])
            full_delta = (smpcc_full - bt_full
                          if bt_full is not None and smpcc_full is not None
                          else None)
            if full_delta is not None:
                full_height_deltas.append(full_delta)
            bt_tail = optional_finite_number(
                bt["secondary_metrics"]
                  ["external_measured_height_fixed_tail_q95_m"])
            smpcc_tail = optional_finite_number(
                smpcc["secondary_metrics"]
                     ["external_measured_height_fixed_tail_q95_m"])
            tail_delta = (smpcc_tail - bt_tail
                          if bt_tail is not None and smpcc_tail is not None
                          else None)
            if tail_delta is not None:
                tail_height_deltas.append(tail_delta)

            bt_time = optional_finite_number(
                bt["trial"]["first_settled_goal_sec"])
            smpcc_time = optional_finite_number(
                smpcc["trial"]["first_settled_goal_sec"])
            ratio = (smpcc_time / bt_time
                     if bt_time is not None and bt_time > 0.0 and
                     smpcc_time is not None else None)
            if ratio is not None:
                time_ratios.append(ratio)
            correction = optional_finite_number(
                smpcc["smpcc_bt_go_no_go_audit"]
                     ["effective_correction_fraction"])
            if correction is not None:
                correction_fractions.append(correction)
            paired.append({
                "seed": seed,
                "bt_complete": bt_complete,
                "smpcc_bt_complete": smpcc_complete,
                "tracking_improvement_fraction": improvement,
                "full_height_delta_m": full_delta,
                "tail_height_delta_m": tail_delta,
                "settled_time_ratio": ratio,
                "effective_correction_fraction": correction,
            })

        improved_count = sum(value > 0.0 for value in tracking_improvements)
        tracking_pass = (
            len(tracking_improvements) == len(SEEDS) and
            improved_count >= TRACKING_IMPROVED_SEEDS_MIN and
            median(tracking_improvements) >=
                TRACKING_MEDIAN_IMPROVEMENT_MIN)
        full_height_pass = (
            len(full_height_deltas) == len(SEEDS) and
            sum(value <= FULL_HEIGHT_NI_MARGIN_M
                for value in full_height_deltas) >= 4 and
            median(full_height_deltas) <= FULL_HEIGHT_NI_MARGIN_M)
        tail_height_pass = (
            len(tail_height_deltas) == len(SEEDS) and
            sum(value <= TAIL_HEIGHT_NI_MARGIN_M
                for value in tail_height_deltas) >= 4 and
            median(tail_height_deltas) <= TAIL_HEIGHT_NI_MARGIN_M)
        completion_pass = (
            completed_bt == 5 and completed_smpcc == 5 and
            controlled_stops_smpcc == 0 and candidate_failures == 0)
        time_pass = len(time_ratios) == len(SEEDS) and all(
            value <= COMPLETION_TIME_RATIO_MAX for value in time_ratios)
        correction_pass = len(correction_fractions) == len(SEEDS) and all(
            value >= EFFECTIVE_CORRECTION_FRACTION_MIN
            for value in correction_fractions)
        passes = all((tracking_pass, full_height_pass, tail_height_pass,
                      completion_pass, time_pass, correction_pass))
        failures = [
            name for name, passed in (
                ("COMPLETION", completion_pass),
                ("TRACKING", tracking_pass),
                ("FULL_HEIGHT_NONINFERIORITY", full_height_pass),
                ("TAIL_HEIGHT_NONINFERIORITY", tail_height_pass),
                ("COMPLETION_TIME", time_pass),
                ("EFFECTIVE_CORRECTION", correction_pass),
            ) if not passed
        ]
        global_reasons.extend(f"{disturbance}:{item}" for item in failures)
        rows.append({
            "disturbance": disturbance,
            "bt_completion": f"{completed_bt}/5",
            "smpcc_bt_completion": f"{completed_smpcc}/5",
            "tracking_improved_seeds": improved_count,
            "tracking_median_improvement_fraction":
                optional_median(tracking_improvements),
            "full_height_median_delta_m":
                optional_median(full_height_deltas),
            "tail_height_median_delta_m":
                optional_median(tail_height_deltas),
            "maximum_settled_time_ratio":
                max(time_ratios) if time_ratios else None,
            "minimum_effective_correction_fraction":
                min(correction_fractions) if correction_fractions else None,
            "passes": passes,
            "failures": failures,
            "paired": paired,
        })

    decision = "GO_ROUTE_A" if not global_reasons else "NO_GO_ROUTE_B"
    report = {
        "schema": "spmpc_smpcc_bt_go_no_go_decision_v1",
        "development_only": True,
        "one_shot_failure_stops_route_a": True,
        "decision": decision,
        "reasons": global_reasons,
        "frozen_rule": {
            "seeds": list(SEEDS),
            "tracking_metric": "tracking_q95_m",
            "tracking_improved_seeds_min": TRACKING_IMPROVED_SEEDS_MIN,
            "tracking_median_improvement_min":
                TRACKING_MEDIAN_IMPROVEMENT_MIN,
            "full_height_ni_margin_m": FULL_HEIGHT_NI_MARGIN_M,
            "tail_height_ni_margin_m": TAIL_HEIGHT_NI_MARGIN_M,
            "completion_time_ratio_max": COMPLETION_TIME_RATIO_MAX,
            "effective_correction_fraction_min":
                EFFECTIVE_CORRECTION_FRACTION_MIN,
        },
        "rows": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print(decision)
    return 0 if decision == "GO_ROUTE_A" else 4


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyError, TypeError, ValueError) as exception:
        print(f"invalid campaign evidence: {exception}", file=sys.stderr)
        raise SystemExit(3)
