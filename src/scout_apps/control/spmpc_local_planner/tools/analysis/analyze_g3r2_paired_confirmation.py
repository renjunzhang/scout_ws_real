#!/usr/bin/env python3
"""Analyze the frozen three-block G3R2 W5_S10/Bsmooth paired confirmation."""

import argparse
import csv
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path


PROTOCOL = "G3R2_robot_only_W5_S10_paired_confirmation_v1"
EXPECTED_ROWS = {
    "01": ("01", "01", "Bsmooth"),
    "02": ("01", "02", "W5_S10"),
    "03": ("02", "01", "W5_S10"),
    "04": ("02", "02", "Bsmooth"),
    "05": ("03", "01", "Bsmooth"),
    "06": ("03", "02", "W5_S10"),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    parser.add_argument("--prereg-sha256", required=True)
    parser.add_argument("--outcome-window-rule-sha256", required=True)
    parser.add_argument("--source-report-sha256", required=True)
    parser.add_argument("--screen-report-sha256", required=True)
    parser.add_argument("--minimum-mean-rgb-p95-improvement-mm", type=float, default=0.05)
    parser.add_argument("--minimum-mean-rgb-rms-improvement-mm", type=float, default=0.0)
    parser.add_argument("--maximum-mean-raw-imu-regression-mm", type=float, default=0.05)
    parser.add_argument("--minimum-positive-blocks", type=int, default=2)
    return parser.parse_args()


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def close(left, right, tolerance=1.0e-5):
    return finite(left) and abs(float(left) - float(right)) <= tolerance


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while True:
            block = stream.read(8 * 1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def audit_state_contract(row, report, condition):
    failures = []
    if report.get("status") != "PASS":
        failures.append("row {} postflight is not PASS".format(row))
    config = report.get("effective_config_last", {})
    expected_weight = 0.0 if condition == "Bsmooth" else 5.0
    if not close(config.get("w_slosh"), expected_weight):
        failures.append("row {} w_slosh mismatch".format(row))
    for field in ("w_smooth", "w_alpha", "w_du_a", "w_du_vs"):
        if not close(config.get(field), 1.0):
            failures.append("row {} {} mismatch".format(row, field))
    if not close(config.get("delay_phase_mode_code"), 4.0):
        failures.append("row {} did not run fixed_robot_only".format(row))

    internal = report.get("internal_state", {})
    if not finite(internal.get("robot_delay_compensation_applied_fraction")) or float(
        internal.get("robot_delay_compensation_applied_fraction")
    ) < 0.98:
        failures.append("row {} lacks robot delay compensation".format(row))
    if not finite(internal.get("liquid_delay_compensation_applied_fraction")) or float(
        internal.get("liquid_delay_compensation_applied_fraction")
    ) > 0.02:
        failures.append("row {} applied forbidden liquid delay rollout".format(row))
    if not finite(internal.get("solver_source_code_fraction")) or float(
        internal.get("solver_source_code_fraction")
    ) < 0.98:
        failures.append("row {} lacks processed-IMU solver-source coverage".format(row))

    processed = report.get("processed_imu", {})
    if not finite(processed.get("ready_fraction")) or float(
        processed.get("ready_fraction")
    ) < 0.98:
        failures.append("row {} lacks processed-IMU READY coverage".format(row))
    if processed.get("fallback_samples") != 0:
        failures.append("row {} used observer fallback".format(row))
    if processed.get("reset_epochs") != [0]:
        failures.append("row {} has an observer reset".format(row))
    return failures


def evaluate_confirmation(
    reports,
    minimum_p95=0.05,
    minimum_rms=0.0,
    maximum_raw_regression=0.05,
    minimum_positive_blocks=2,
):
    failures = []
    if set(reports) != set(EXPECTED_ROWS):
        failures.append(
            "row set mismatch: expected={} actual={}".format(
                sorted(EXPECTED_ROWS), sorted(reports)
            )
        )

    normalized = {}
    for row, (block, position, condition) in EXPECTED_ROWS.items():
        report = reports.get(row)
        if report is None:
            continue
        if (
            str(report.get("row")) != row
            or str(report.get("block")) != block
            or str(report.get("position")) != position
            or report.get("condition") != condition
        ):
            failures.append("row {} metadata differs from frozen order".format(row))
        failures.extend(audit_state_contract(row, report, condition))
        values = {
            "rgb_p95_mm": report.get("online_rgb", {}).get("h_vis_p95_mm"),
            "rgb_rms_mm": report.get("online_rgb", {}).get("h_vis_rms_mm"),
            "raw_imu_p95_mm": report.get("internal_state", {}).get(
                "imu_modal_height_p95_mm"
            ),
            "tracking_contour_p95_m": report.get("tracking", {}).get(
                "contour_p95_m"
            ),
            "tracking_yaw_p95_rad": report.get("tracking", {}).get("yaw_p95_rad"),
            "solver_p95_ms": report.get("runtime", {}).get("solver_p95_ms"),
        }
        for field, value in values.items():
            if not finite(value):
                failures.append("row {} lacks finite {}".format(row, field))
        normalized[row] = {
            "row": row,
            "block": block,
            "position": position,
            "condition": condition,
            **{key: float(value) if finite(value) else math.nan for key, value in values.items()},
        }

    grouped = {}
    for item in normalized.values():
        grouped.setdefault(item["block"], {})[item["condition"]] = item
    blocks = []
    for block in ("01", "02", "03"):
        pair = grouped.get(block, {})
        if set(pair) != {"Bsmooth", "W5_S10"}:
            failures.append("block {} is not a complete pair".format(block))
            continue
        comparator = pair["Bsmooth"]
        candidate = pair["W5_S10"]
        blocks.append(
            {
                "block": block,
                "bsmooth_row": comparator["row"],
                "w5_s10_row": candidate["row"],
                "bsmooth_rgb_p95_mm": comparator["rgb_p95_mm"],
                "w5_s10_rgb_p95_mm": candidate["rgb_p95_mm"],
                "rgb_p95_improvement_mm": comparator["rgb_p95_mm"]
                - candidate["rgb_p95_mm"],
                "bsmooth_rgb_rms_mm": comparator["rgb_rms_mm"],
                "w5_s10_rgb_rms_mm": candidate["rgb_rms_mm"],
                "rgb_rms_improvement_mm": comparator["rgb_rms_mm"]
                - candidate["rgb_rms_mm"],
                "bsmooth_raw_imu_p95_mm": comparator["raw_imu_p95_mm"],
                "w5_s10_raw_imu_p95_mm": candidate["raw_imu_p95_mm"],
                "raw_imu_p95_improvement_mm": comparator["raw_imu_p95_mm"]
                - candidate["raw_imu_p95_mm"],
            }
        )

    p95_deltas = [item["rgb_p95_improvement_mm"] for item in blocks]
    rms_deltas = [item["rgb_rms_improvement_mm"] for item in blocks]
    raw_deltas = [item["raw_imu_p95_improvement_mm"] for item in blocks]
    mean_p95 = statistics.mean(p95_deltas) if p95_deltas else math.nan
    median_p95 = statistics.median(p95_deltas) if p95_deltas else math.nan
    mean_rms = statistics.mean(rms_deltas) if rms_deltas else math.nan
    mean_raw = statistics.mean(raw_deltas) if raw_deltas else math.nan
    positive_blocks = sum(delta > 0.0 for delta in p95_deltas)
    largest = max(p95_deltas) if p95_deltas else math.nan
    sum_without_largest = sum(p95_deltas) - largest if p95_deltas else math.nan
    leave_one_out = (
        [
            statistics.mean(p95_deltas[:index] + p95_deltas[index + 1 :])
            for index in range(len(p95_deltas))
        ]
        if len(p95_deltas) > 1
        else []
    )

    gates = {
        "complete_pairs_pass": len(blocks) == 3,
        "mean_rgb_p95_pass": finite(mean_p95) and mean_p95 >= minimum_p95,
        "median_rgb_p95_direction_pass": finite(median_p95) and median_p95 > 0.0,
        "mean_rgb_rms_pass": finite(mean_rms) and mean_rms >= minimum_rms,
        "raw_imu_guard_pass": finite(mean_raw) and mean_raw >= -maximum_raw_regression,
        "positive_blocks_pass": positive_blocks >= minimum_positive_blocks,
        "single_block_dominance_pass": finite(sum_without_largest)
        and sum_without_largest > 0.0,
        "leave_one_block_out_pass": bool(leave_one_out)
        and all(value > 0.0 for value in leave_one_out),
    }
    for gate, passed in gates.items():
        if not passed:
            failures.append("confirmation gate failed: {}".format(gate))

    bsmooth_values = [item["bsmooth_rgb_p95_mm"] for item in blocks]
    aggregate = {
        "eligible_pair_count": len(blocks),
        "mean_rgb_p95_improvement_mm": mean_p95,
        "median_rgb_p95_improvement_mm": median_p95,
        "mean_rgb_rms_improvement_mm": mean_rms,
        "mean_raw_imu_p95_improvement_mm": mean_raw,
        "relative_rgb_p95_improvement": (
            mean_p95 / statistics.mean(bsmooth_values)
            if bsmooth_values and statistics.mean(bsmooth_values) > 0.0
            else math.nan
        ),
        "positive_block_count": positive_blocks,
        "largest_rgb_p95_improvement_mm": largest,
        "sum_without_largest_rgb_p95_improvement_mm": sum_without_largest,
        "leave_one_block_out_mean_rgb_p95_improvement_mm": leave_one_out,
        "gates": gates,
    }
    return {
        "status": "PASS" if not failures else "FAIL",
        "decision": "W5_S10_PAIRED_CONFIRMED" if not failures else "NO_CONFIRMATION",
        "failures": failures,
        "rows": normalized,
        "blocks": blocks,
        "aggregate": aggregate,
    }


def audit_bag(report, root):
    failures = []
    bag_text = report.get("bag")
    if not bag_text:
        return ["postflight does not identify its bag"]
    bag_path = Path(bag_text).expanduser().resolve()
    if bag_path.parent != root:
        return ["bag is outside the confirmation root"]
    if not bag_path.is_file():
        return ["bag is missing: {}".format(bag_path)]
    if report.get("bag_size_bytes") != bag_path.stat().st_size:
        failures.append("bag size differs from postflight")
    expected_sha = report.get("bag_sha256")
    if not isinstance(expected_sha, str) or len(expected_sha) != 64:
        failures.append("postflight lacks a valid bag SHA-256")
    elif sha256_file(bag_path) != expected_sha:
        failures.append("bag SHA-256 differs from postflight")
    return failures


def main():
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    reports = {}
    paths = {}
    integrity_failures = []
    for path in sorted(root.glob("DEV_G3R2C_*_g3r2c_postflight.json")):
        with path.open(encoding="utf-8") as stream:
            report = json.load(stream)
        row = str(report.get("row", ""))
        if row in reports:
            raise SystemExit("duplicate confirmation row {}".format(row))
        reports[row] = report
        paths[row] = path
        if report.get("protocol") != PROTOCOL:
            integrity_failures.append("row {} protocol mismatch".format(row))
        bindings = report.get("bindings", {})
        for key, expected in (
            ("prereg_sha256", args.prereg_sha256),
            ("outcome_window_rule_sha256", args.outcome_window_rule_sha256),
            ("source_report_sha256", args.source_report_sha256),
        ):
            if bindings.get(key) != expected:
                integrity_failures.append("row {} {} binding mismatch".format(row, key))
        integrity_failures.extend(
            "row {} {}".format(row, failure) for failure in audit_bag(report, root)
        )

    result = evaluate_confirmation(
        reports,
        minimum_p95=args.minimum_mean_rgb_p95_improvement_mm,
        minimum_rms=args.minimum_mean_rgb_rms_improvement_mm,
        maximum_raw_regression=args.maximum_mean_raw_imu_regression_mm,
        minimum_positive_blocks=args.minimum_positive_blocks,
    )
    if integrity_failures:
        result["failures"].extend(integrity_failures)
        result["status"] = "FAIL"
        result["decision"] = "NO_CONFIRMATION"

    dataset = [
        {
            "row": row,
            "block": reports[row].get("block"),
            "position": reports[row].get("position"),
            "condition": reports[row].get("condition"),
            "bag": reports[row].get("bag"),
            "bag_sha256": reports[row].get("bag_sha256"),
            "postflight": str(paths[row]),
            "postflight_sha256": sha256_file(paths[row]),
        }
        for row in sorted(reports)
    ]
    output = {
        "schema_version": 1,
        "report_type": "G3R2_PAIRED_CONFIRMATION",
        "protocol": PROTOCOL,
        "scope": "DEVELOPMENT_CONFIRMATION_ONLY",
        "status": result["status"],
        "decision": result["decision"],
        "bindings": {
            "prereg_sha256": args.prereg_sha256,
            "outcome_window_rule_sha256": args.outcome_window_rule_sha256,
            "source_report_sha256": args.source_report_sha256,
            "screen_report_sha256": args.screen_report_sha256,
        },
        "thresholds": {
            "minimum_mean_rgb_p95_improvement_mm": args.minimum_mean_rgb_p95_improvement_mm,
            "minimum_mean_rgb_rms_improvement_mm": args.minimum_mean_rgb_rms_improvement_mm,
            "maximum_mean_raw_imu_regression_mm": args.maximum_mean_raw_imu_regression_mm,
            "minimum_positive_blocks": args.minimum_positive_blocks,
        },
        "dataset": dataset,
        "blocks": result["blocks"],
        "aggregate": result["aggregate"],
        "failures": result["failures"],
        "limitations": [
            "This is a development paired confirmation and does not contribute to formal n.",
            "Online RGB scalar is image-free; raw frames cannot be reprocessed.",
            "A PASS freezes W5_S10 for later development gates; it is not a formal efficacy claim.",
        ],
    }
    out_json = root / "G3R2_PAIRED_CONFIRMATION_REPORT.json"
    out_csv = root / "G3R2_PAIRED_CONFIRMATION_BLOCK_METRICS.csv"
    out_json.write_text(
        json.dumps(output, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with out_csv.open("w", encoding="utf-8", newline="") as stream:
        fields = (
            "block",
            "bsmooth_rgb_p95_mm",
            "w5_s10_rgb_p95_mm",
            "rgb_p95_improvement_mm",
            "bsmooth_rgb_rms_mm",
            "w5_s10_rgb_rms_mm",
            "rgb_rms_improvement_mm",
            "bsmooth_raw_imu_p95_mm",
            "w5_s10_raw_imu_p95_mm",
            "raw_imu_p95_improvement_mm",
        )
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: item.get(field) for field in fields} for item in result["blocks"])
    for artifact in (out_json, out_csv):
        Path(str(artifact) + ".sha256").write_text(
            "{}  {}\n".format(sha256_file(artifact), artifact), encoding="utf-8"
        )
    print(
        "[G3R2-CONFIRM] {}: mean RGB P95 delta={:.6f}mm; positive={}/3".format(
            output["status"],
            result["aggregate"].get("mean_rgb_p95_improvement_mm", math.nan),
            result["aggregate"].get("positive_block_count", 0),
        )
    )
    for failure in output["failures"]:
        print("  FAIL: {}".format(failure), file=sys.stderr)
    return 0 if output["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
