#!/usr/bin/env python3
"""Aggregate the frozen four-block G3 W5-vs-Bsmooth online-RGB pilot."""

import argparse
import csv
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path


EXPECTED_ROWS = {
    "01": ("01", "01", "Bsmooth"),
    "02": ("01", "02", "W5"),
    "03": ("02", "01", "W5"),
    "04": ("02", "02", "Bsmooth"),
    "05": ("03", "01", "W5"),
    "06": ("03", "02", "Bsmooth"),
    "07": ("04", "01", "Bsmooth"),
    "08": ("04", "02", "W5"),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    parser.add_argument("--out-json", default="")
    parser.add_argument("--delta-h-dev-mm", type=float, default=0.10)
    parser.add_argument("--minimum-positive-blocks", type=int, default=3)
    parser.add_argument("--expected-pairs", type=int, default=4)
    parser.add_argument("--prereg-sha256", required=True)
    parser.add_argument("--outcome-window-rule-sha256", required=True)
    parser.add_argument("--source-report-sha256", required=True)
    return parser.parse_args()


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def load_reports(root):
    reports = {}
    for path in sorted(root.glob("DEV_G3_*_g3_postflight.json")):
        with path.open(encoding="utf-8") as stream:
            report = json.load(stream)
        row = str(report.get("row", ""))
        if row in reports:
            raise RuntimeError("duplicate G3 postflight row {}".format(row))
        reports[row] = (path, report)
    return reports


def evaluate_blocks(reports, delta_h_dev_mm=0.10, minimum_positive_blocks=3):
    failures = []
    expected_rows = set(EXPECTED_ROWS)
    actual_rows = set(reports)
    if actual_rows != expected_rows:
        missing = sorted(expected_rows - actual_rows)
        unexpected = sorted(actual_rows - expected_rows)
        failures.append("row set mismatch: missing={} unexpected={}".format(missing, unexpected))

    normalized = {}
    for row in sorted(expected_rows & actual_rows):
        _, report = reports[row]
        block, position, condition = EXPECTED_ROWS[row]
        if (
            str(report.get("block")) != block
            or str(report.get("position")) != position
            or str(report.get("condition")) != condition
        ):
            failures.append("row {} metadata differs from frozen order".format(row))
        if report.get("status") != "PASS":
            failures.append("row {} postflight is not PASS".format(row))
        value = report.get("online_rgb", {}).get("h_vis_p95_mm")
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            failures.append("row {} lacks finite H_vis P95".format(row))
            continue
        normalized[row] = {
            "block": block,
            "condition": condition,
            "h_vis_p95_mm": float(value),
            "report": report,
        }

    blocks = {}
    for row, item in normalized.items():
        blocks.setdefault(item["block"], {})[item["condition"]] = item

    block_results = []
    for block in ("01", "02", "03", "04"):
        group = blocks.get(block, {})
        if set(group) != {"Bsmooth", "W5"}:
            failures.append("block {} is not a complete Bsmooth/W5 pair".format(block))
            continue
        bsmooth = group["Bsmooth"]["h_vis_p95_mm"]
        w5 = group["W5"]["h_vis_p95_mm"]
        block_results.append(
            {
                "block": block,
                "bsmooth_h_vis_p95_mm": bsmooth,
                "w5_h_vis_p95_mm": w5,
                "delta_b_mm": bsmooth - w5,
            }
        )

    deltas = [item["delta_b_mm"] for item in block_results]
    mean_delta = statistics.mean(deltas) if deltas else math.nan
    median_delta = statistics.median(deltas) if deltas else math.nan
    positive_blocks = sum(delta > 0.0 for delta in deltas)
    if deltas:
        largest = max(deltas)
        sum_without_largest = sum(deltas) - largest
        loo_means = [
            statistics.mean(deltas[:index] + deltas[index + 1 :])
            for index in range(len(deltas))
            if len(deltas) > 1
        ]
    else:
        largest = math.nan
        sum_without_largest = math.nan
        loo_means = []

    effect_pass = len(deltas) == 4 and mean_delta >= delta_h_dev_mm
    direction_pass = positive_blocks >= minimum_positive_blocks
    dominance_pass = bool(deltas) and sum_without_largest > 0.0
    loo_pass = bool(loo_means) and all(value > 0.0 for value in loo_means)
    if not effect_pass:
        failures.append(
            "paired mean effect {:.6f}mm < delta_H_dev {:.6f}mm".format(
                mean_delta, delta_h_dev_mm
            )
        )
    if not direction_pass:
        failures.append(
            "only {}/4 blocks favor W5; require at least {}".format(
                positive_blocks, minimum_positive_blocks
            )
        )
    if not dominance_pass:
        failures.append("effect is single-block dominated")
    if not loo_pass:
        failures.append("leave-one-block-out direction is not uniformly positive")

    bsmooth_values = [item["bsmooth_h_vis_p95_mm"] for item in block_results]
    relative_improvement = (
        mean_delta / statistics.mean(bsmooth_values)
        if bsmooth_values and statistics.mean(bsmooth_values) > 0.0
        else math.nan
    )
    aggregate = {
        "eligible_pair_count": len(block_results),
        "delta_h_dev_mm": delta_h_dev_mm,
        "mean_delta_mm": mean_delta,
        "median_delta_mm": median_delta,
        "relative_improvement": relative_improvement,
        "positive_block_count": positive_blocks,
        "largest_delta_mm": largest,
        "sum_without_largest_delta_mm": sum_without_largest,
        "leave_one_block_out_mean_delta_mm": loo_means,
        "effect_pass": effect_pass,
        "direction_pass": direction_pass,
        "single_block_dominance_pass": dominance_pass,
        "leave_one_block_out_pass": loo_pass,
    }
    return block_results, aggregate, failures


def main():
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    out_json = (
        Path(args.out_json).expanduser().resolve()
        if args.out_json
        else root / "G3_EFFICACY_REPORT.json"
    )
    out_csv = out_json.with_name("G3_EFFICACY_BLOCK_METRICS.csv")
    failures = []
    try:
        reports = load_reports(root)
        block_results, aggregate, evaluation_failures = evaluate_blocks(
            reports,
            delta_h_dev_mm=args.delta_h_dev_mm,
            minimum_positive_blocks=args.minimum_positive_blocks,
        )
        failures.extend(evaluation_failures)
    except Exception as exc:  # noqa: BLE001
        print("[G3 analyzer] {}".format(exc), file=sys.stderr)
        return 2

    if args.expected_pairs != 4:
        failures.append("frozen G3 protocol requires expected_pairs=4")
    if aggregate["eligible_pair_count"] != args.expected_pairs:
        failures.append(
            "eligible pair count {} != {}".format(
                aggregate["eligible_pair_count"], args.expected_pairs
            )
        )

    binding_fields = {
        "prereg_sha256": args.prereg_sha256,
        "outcome_window_rule_sha256": args.outcome_window_rule_sha256,
        "source_report_sha256": args.source_report_sha256,
    }
    for row, (_, trial) in reports.items():
        bindings = trial.get("bindings", {})
        for key, expected in binding_fields.items():
            if bindings.get(key) != expected:
                failures.append("row {} binding mismatch: {}".format(row, key))

    dataset_rows = []
    for row in sorted(reports):
        path, trial = reports[row]
        dataset_rows.append(
            {
                "row": row,
                "block": trial.get("block"),
                "position": trial.get("position"),
                "condition": trial.get("condition"),
                "status": trial.get("status"),
                "bag": trial.get("bag"),
                "bag_sha256": trial.get("bag_sha256"),
                "postflight": str(path),
                "postflight_sha256": sha256_file(path),
            }
        )

    report = {
        "schema_version": 1,
        "report_type": "G3_EFFICACY",
        "protocol": "G3_processed_imu_W5_vs_Bsmooth_online_RGB_development_v1",
        "scope": "DEVELOPMENT_ONLY",
        "status": "PASS" if not failures else "FAIL",
        "decision": "W5_PHYSICAL_DIRECTION_CONFIRMED" if not failures else "NO_FORMAL_RELEASE",
        "planned_row_count": 8,
        "planned_pair_count": 4,
        "postflight_complete_attempt_count": len(reports),
        "method_success_planned_row_count": sum(
            trial.get("status") == "PASS" for _, trial in reports.values()
        ),
        "bindings": binding_fields,
        "dataset_index": dataset_rows,
        "blocks": block_results,
        "aggregate": aggregate,
        "failures": failures,
        "limitations": [
            "G3 is a development efficacy gate and never contributes to formal n.",
            "The bound G2S source report is a three-trial development decision, not a formal four-unit G2S PASS.",
            "Online RGB scalar is image-free; raw frames cannot be reprocessed after acquisition.",
        ],
    }
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with out_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "block",
                "bsmooth_h_vis_p95_mm",
                "w5_h_vis_p95_mm",
                "delta_b_mm",
            ),
        )
        writer.writeheader()
        writer.writerows(block_results)
    out_sha = out_json.with_suffix(out_json.suffix + ".sha256")
    out_sha.write_text(
        "{}  {}\n".format(sha256_file(out_json), out_json), encoding="utf-8"
    )
    csv_sha = out_csv.with_suffix(out_csv.suffix + ".sha256")
    csv_sha.write_text("{}  {}\n".format(sha256_file(out_csv), out_csv), encoding="utf-8")

    print("[G3 analyzer] {}".format(report["status"]))
    print(
        "  pairs={}; mean Delta={:.4f} mm; positive={}/4; relative={:.2%}".format(
            aggregate["eligible_pair_count"],
            aggregate["mean_delta_mm"],
            aggregate["positive_block_count"],
            aggregate["relative_improvement"],
        )
    )
    print("  report = {}".format(out_json))
    for failure in failures:
        print("  FAIL: {}".format(failure), file=sys.stderr)
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
