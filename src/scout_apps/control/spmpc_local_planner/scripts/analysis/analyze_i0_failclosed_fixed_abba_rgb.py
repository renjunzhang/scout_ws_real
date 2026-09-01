#!/usr/bin/env python3
"""Summarize the literal B0/B_slosh I0+fixed development ABBA.

The input files are the image-free RGB postflights produced by
``validate_g3_online_rgb_trial.py``.  That validator defines the outcome as a
causal median-of-five ``H_vis`` signal from motion start through the first
GOAL_REACHED event plus a five-second tail.  This analyzer does not recompute
or reinterpret that outcome; it only applies the frozen paired decision rules.

Supported dataset states are deliberately small:

* rows 01/02: evaluate the first-block rapid screen;
* rows 01--04: evaluate the complete two-block ABBA.

Exit codes are 0 for promotion/positive, 10 for a valid method-negative
decision, and 2 for invalid or incomplete evidence.
"""

import argparse
import json
import math
import statistics
import sys
from pathlib import Path


PROTOCOL = "SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1"
REPORT_GLOB = "*_i0_fixed_rgb_postflight.json"
REPORT_SUFFIX = "_i0_fixed_rgb_postflight.json"
UNIT_PASS_SUFFIX = "_unit_pass.env"
DEFAULT_REPORT_NAME = "I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS.json"
EXPECTED_ROWS = {
    "01": {"row": "01", "block": "01", "position": "01", "condition": "B0"},
    "02": {"row": "02", "block": "01", "position": "02", "condition": "Bslosh"},
    "03": {"row": "03", "block": "02", "position": "01", "condition": "Bslosh"},
    "04": {"row": "04", "block": "02", "position": "02", "condition": "B0"},
}
SUPPORTED_ROW_SETS = ({"01", "02"}, set(EXPECTED_ROWS))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, help="Directory containing row postflights")
    parser.add_argument(
        "--report",
        help="Output JSON path (default: ROOT/{})".format(DEFAULT_REPORT_NAME),
    )
    parser.add_argument("--protocol", default=PROTOCOL)
    parser.add_argument("--minimum-p95-improvement-mm", type=float, default=0.05)
    parser.add_argument("--minimum-rms-improvement-mm", type=float, default=0.0)
    parser.add_argument("--maximum-slowdown-ratio", type=float, default=1.05)
    return parser.parse_args()


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def normalized_index(value):
    text = str(value).strip()
    if not text.isdigit():
        return text
    return text.zfill(2)


def close(value, expected, tolerance=1.0e-9):
    return finite(value) and abs(float(value) - float(expected)) <= tolerance


def load_reports(root):
    reports = {}
    paths = {}
    failures = []
    for path in sorted(root.glob(REPORT_GLOB)):
        try:
            report = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            failures.append("could not read {}: {}".format(path, exc))
            continue
        row = normalized_index(report.get("row", ""))
        if row in reports:
            failures.append(
                "duplicate row {}: {} and {}".format(row, paths[row], path)
            )
            continue
        reports[row] = report
        paths[row] = path
        stem = path.name[: -len(REPORT_SUFFIX)]
        unit_pass = path.with_name(stem + UNIT_PASS_SUFFIX)
        if not unit_pass.is_file():
            failures.append("row {} is missing {}".format(row, unit_pass.name))
            continue
        try:
            unit_fields = {}
            for raw_line in unit_pass.read_text(encoding="utf-8").splitlines():
                if "=" not in raw_line:
                    continue
                key, value = raw_line.split("=", 1)
                unit_fields[key.strip()] = value.strip()
        except OSError as exc:
            failures.append("could not read {}: {}".format(unit_pass, exc))
            continue
        expected_unit_fields = {
            "status": "PASS",
            "protocol": report.get("protocol"),
            "row": row,
            "condition": report.get("condition"),
        }
        for field, expected_value in expected_unit_fields.items():
            actual_value = unit_fields.get(field)
            if actual_value != expected_value:
                failures.append(
                    "row {} unit pass {}={!r}, expected {!r}".format(
                        row, field, actual_value, expected_value
                    )
                )
    if not reports and not failures:
        failures.append("no {} files found under {}".format(REPORT_GLOB, root))
    return reports, paths, failures


def audit_row(row, report, expected, protocol):
    failures = []
    if report.get("status") != "PASS":
        failures.append("row {} postflight status is not PASS".format(row))
    if report.get("protocol") != protocol:
        failures.append("row {} protocol mismatch".format(row))
    for field in ("row", "block", "position"):
        if normalized_index(report.get(field, "")) != expected[field]:
            failures.append("row {} {} differs from frozen order".format(row, field))
    if report.get("condition") != expected["condition"]:
        failures.append("row {} condition differs from frozen order".format(row))

    if not close(report.get("t_hvis_tail_sec"), 5.0):
        failures.append("row {} RGB tail is not the frozen 5 seconds".format(row))
    online = report.get("online_rgb", {})
    if normalized_index(online.get("rolling_window", "")) != "05":
        failures.append("row {} RGB rolling window is not causal median5".format(row))
    p95 = online.get("h_vis_p95_mm")
    rms = online.get("h_vis_rms_mm")
    motion_to_arrival = report.get("motion_to_arrival_sec")
    if not finite(p95):
        failures.append("row {} lacks finite h_vis_p95_mm".format(row))
    if not finite(rms):
        failures.append("row {} lacks finite h_vis_rms_mm".format(row))
    if not finite(motion_to_arrival) or float(motion_to_arrival) <= 0.0:
        failures.append("row {} lacks positive motion_to_arrival_sec".format(row))
    return failures


def block_result(block, rows):
    if block == "01":
        b0_row, bslosh_row, order = "01", "02", "B0->Bslosh"
    else:
        b0_row, bslosh_row, order = "04", "03", "Bslosh->B0"
    b0 = rows[b0_row]
    bslosh = rows[bslosh_row]
    return {
        "block": block,
        "order": order,
        "b0_row": b0_row,
        "bslosh_row": bslosh_row,
        "b0_h_vis_p95_mm": b0["h_vis_p95_mm"],
        "bslosh_h_vis_p95_mm": bslosh["h_vis_p95_mm"],
        "delta_p95_mm": b0["h_vis_p95_mm"] - bslosh["h_vis_p95_mm"],
        "b0_h_vis_rms_mm": b0["h_vis_rms_mm"],
        "bslosh_h_vis_rms_mm": bslosh["h_vis_rms_mm"],
        "delta_rms_mm": b0["h_vis_rms_mm"] - bslosh["h_vis_rms_mm"],
        "b0_motion_to_arrival_sec": b0["motion_to_arrival_sec"],
        "bslosh_motion_to_arrival_sec": bslosh["motion_to_arrival_sec"],
        "bslosh_to_b0_goal_time_ratio": (
            bslosh["motion_to_arrival_sec"] / b0["motion_to_arrival_sec"]
        ),
    }


def evaluate_reports(
    reports,
    protocol=PROTOCOL,
    minimum_p95_improvement_mm=0.05,
    minimum_rms_improvement_mm=0.0,
    maximum_slowdown_ratio=1.05,
):
    failures = []
    actual_rows = set(reports)
    if actual_rows not in SUPPORTED_ROW_SETS:
        failures.append(
            "unsupported row set: expected 01/02 or 01/02/03/04, got {}".format(
                ",".join(sorted(actual_rows)) or "none"
            )
        )

    normalized_rows = {}
    for row in sorted(actual_rows & set(EXPECTED_ROWS)):
        report = reports[row]
        failures.extend(audit_row(row, report, EXPECTED_ROWS[row], protocol))
        online = report.get("online_rgb", {})
        normalized_rows[row] = {
            "row": row,
            "block": EXPECTED_ROWS[row]["block"],
            "position": EXPECTED_ROWS[row]["position"],
            "condition": EXPECTED_ROWS[row]["condition"],
            "h_vis_p95_mm": float(online["h_vis_p95_mm"])
            if finite(online.get("h_vis_p95_mm"))
            else None,
            "h_vis_rms_mm": float(online["h_vis_rms_mm"])
            if finite(online.get("h_vis_rms_mm"))
            else None,
            "motion_to_arrival_sec": float(report["motion_to_arrival_sec"])
            if finite(report.get("motion_to_arrival_sec"))
            else None,
        }

    if failures:
        return {
            "status": "INVALID",
            "decision": "INVALID_EVIDENCE",
            "phase": "INVALID",
            "rows": normalized_rows,
            "blocks": [],
            "aggregate": {},
            "gates": {},
            "failures": failures,
            "exit_code": 2,
        }

    blocks = [block_result("01", normalized_rows)]
    block1 = blocks[0]
    block1_gates = {
        "delta_p95_at_least_minimum": (
            block1["delta_p95_mm"] + 1.0e-12
            >= minimum_p95_improvement_mm
        ),
        "delta_rms_nonnegative": (
            block1["delta_rms_mm"] + 1.0e-12
            >= minimum_rms_improvement_mm
        ),
    }
    block1_promotes = all(block1_gates.values())

    if actual_rows == {"01", "02"}:
        return {
            "status": "PASS" if block1_promotes else "STOP",
            "decision": "PROMOTE_BLOCK2"
            if block1_promotes
            else "STOP_BLOCK1_FUTILITY",
            "phase": "BLOCK1_RAPID_SCREEN",
            "rows": normalized_rows,
            "blocks": blocks,
            "aggregate": {
                "mean_delta_p95_mm": block1["delta_p95_mm"],
                "mean_delta_rms_mm": block1["delta_rms_mm"],
                "bslosh_to_b0_goal_time_ratio": block1[
                    "bslosh_to_b0_goal_time_ratio"
                ],
                "slowdown_risk_above_5pct": block1[
                    "bslosh_to_b0_goal_time_ratio"
                ]
                > maximum_slowdown_ratio,
            },
            "gates": block1_gates,
            "failures": [],
            "exit_code": 0 if block1_promotes else 10,
        }

    blocks.append(block_result("02", normalized_rows))
    p95_deltas = [item["delta_p95_mm"] for item in blocks]
    rms_deltas = [item["delta_rms_mm"] for item in blocks]
    mean_p95 = statistics.mean(p95_deltas)
    mean_rms = statistics.mean(rms_deltas)
    goal_time_ratios = [
        item["bslosh_to_b0_goal_time_ratio"] for item in blocks
    ]
    median_goal_time_ratio = statistics.median(goal_time_ratios)
    rgb_effect_gates = {
        "block1_rapid_screen_pass": block1_promotes,
        "both_blocks_delta_p95_positive": all(value > 0.0 for value in p95_deltas),
        "mean_delta_p95_at_least_minimum": (
            mean_p95 + 1.0e-12 >= minimum_p95_improvement_mm
        ),
        "mean_delta_rms_nonnegative": (
            mean_rms + 1.0e-12 >= minimum_rms_improvement_mm
        ),
    }
    rgb_effect_positive = all(rgb_effect_gates.values())
    slowdown_gate = median_goal_time_ratio <= maximum_slowdown_ratio + 1.0e-12
    final_gates = dict(rgb_effect_gates)
    final_gates["median_goal_time_slowdown_at_most_maximum"] = slowdown_gate
    final_positive = rgb_effect_positive and slowdown_gate
    if final_positive:
        decision = "DEVELOPMENT_POSITIVE"
    elif rgb_effect_positive:
        decision = "RGB_POSITIVE_SLOWDOWN_CONFOUNDED"
    else:
        decision = "NO_DEVELOPMENT_POSITIVE"
    return {
        "status": "PASS" if final_positive else "STOP",
        "decision": decision,
        "phase": "COMPLETE_ABBA",
        "rows": normalized_rows,
        "blocks": blocks,
        "aggregate": {
            "mean_delta_p95_mm": mean_p95,
            "mean_delta_rms_mm": mean_rms,
            "positive_p95_block_count": sum(value > 0.0 for value in p95_deltas),
            "block1_rapid_screen_would_promote": block1_promotes,
            "bslosh_to_b0_goal_time_ratios": goal_time_ratios,
            "median_bslosh_to_b0_goal_time_ratio": median_goal_time_ratio,
            "rgb_effect_gates_pass": rgb_effect_positive,
        },
        "gates": final_gates,
        "failures": [],
        "exit_code": 0 if final_positive else 10,
    }


def build_output(args, root, paths, evaluation):
    return {
        "schema_version": 1,
        "report_type": "I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS",
        "protocol": args.protocol,
        "scope": "DEVELOPMENT_ONLY",
        "status": evaluation["status"],
        "decision": evaluation["decision"],
        "phase": evaluation["phase"],
        "metric_contract": {
            "source": "validate_g3_online_rgb_trial.py",
            "signal": "online_rgb.h_vis",
            "window": "motion_start_to_first_GOAL_REACHED_plus_5s_tail",
            "filter": "causal_median_5",
            "primary": "online_rgb.h_vis_p95_mm",
            "secondary": "online_rgb.h_vis_rms_mm",
            "delta_definition": "B0_minus_Bslosh; positive favors Bslosh",
        },
        "thresholds": {
            "minimum_p95_improvement_mm": args.minimum_p95_improvement_mm,
            "minimum_rms_improvement_mm": args.minimum_rms_improvement_mm,
            "maximum_slowdown_ratio": args.maximum_slowdown_ratio,
            "required_order": "B0,Bslosh,Bslosh,B0",
        },
        "root": str(root),
        "postflights": {
            row: str(path) for row, path in sorted(paths.items())
        },
        "rows": evaluation["rows"],
        "blocks": evaluation["blocks"],
        "aggregate": evaluation["aggregate"],
        "gates": evaluation["gates"],
        "failures": evaluation["failures"],
        "limitations": [
            "This is a development decision, not a formal efficacy claim.",
            "The analyzer consumes frozen postflight metrics and does not reprocess RGB.",
            "Slowdown uses first-GOAL_REACHED time, not exact progress T10-90.",
            "A slowdown-confounded RGB result requires a speed-matched audit.",
        ],
    }


def main():
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    report_path = (
        Path(args.report).expanduser().resolve()
        if args.report
        else root / DEFAULT_REPORT_NAME
    )
    reports, paths, load_failures = load_reports(root)
    evaluation = evaluate_reports(
        reports,
        protocol=args.protocol,
        minimum_p95_improvement_mm=args.minimum_p95_improvement_mm,
        minimum_rms_improvement_mm=args.minimum_rms_improvement_mm,
        maximum_slowdown_ratio=args.maximum_slowdown_ratio,
    )
    if load_failures:
        evaluation["failures"] = load_failures + evaluation["failures"]
        evaluation["status"] = "INVALID"
        evaluation["decision"] = "INVALID_EVIDENCE"
        evaluation["phase"] = "INVALID"
        evaluation["exit_code"] = 2

    output = build_output(args, root, paths, evaluation)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(output, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )

    mean_p95 = output.get("aggregate", {}).get("mean_delta_p95_mm")
    delta_text = "n/a" if not finite(mean_p95) else "{:.6f}mm".format(mean_p95)
    print(
        "[I0FC-FIXED-ABBA-RGB] {}: {}; mean delta P95={}".format(
            output["status"], output["decision"], delta_text
        )
    )
    print("[I0FC-FIXED-ABBA-RGB] report: {}".format(report_path))
    for failure in output["failures"]:
        print("  INVALID: {}".format(failure), file=sys.stderr)
    return int(evaluation["exit_code"])


if __name__ == "__main__":
    sys.exit(main())
