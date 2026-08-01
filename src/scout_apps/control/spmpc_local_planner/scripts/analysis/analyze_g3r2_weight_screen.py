#!/usr/bin/env python3
"""Analyze the G3R2 robot-only-delay one-run-per-candidate screen.

The Bsmooth baseline is the separately frozen G3R2 tracking smoke.  Rows
02--05 are collected only after that smoke passes.  A positive result remains
screening evidence and must be replicated in a newly frozen paired release.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


EXPECTED_ROWS = {
    "01": {"condition": "Bsmooth", "w_slosh": 0.0, "smooth": 1.0, "baseline": True},
    "02": {"condition": "W2_S03", "w_slosh": 2.0, "smooth": 0.3, "baseline": False},
    "03": {"condition": "W5_S10", "w_slosh": 5.0, "smooth": 1.0, "baseline": False},
    "04": {"condition": "W2_S10", "w_slosh": 2.0, "smooth": 1.0, "baseline": False},
    "05": {"condition": "W5_S03", "w_slosh": 5.0, "smooth": 0.3, "baseline": False},
}
BASELINE_PROTOCOL = "G3R2_robot_only_delay_smoke_v1"
SCREEN_PROTOCOL = "G3R2_robot_only_weight_screen_v1"
FAILED_ROW03_ATTEMPT_ID = "DEV_G3R2_H0_C1_W5_S10_r03_a01"
RETRY_ROW03_ATTEMPT_ID = "DEV_G3R2_H0_C1_W5_S10_r03_a02"
FAILED_ROW03_REPORT_NAME = FAILED_ROW03_ATTEMPT_ID + "_g3r2_screen_postflight.json"
RETRY_ROW03_REPORT_NAME = RETRY_ROW03_ATTEMPT_ID + "_g3r2_screen_retry_postflight.json"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    parser.add_argument("--baseline-report", required=True)
    parser.add_argument("--baseline-report-sha256", required=True)
    parser.add_argument("--screen-prereg-sha256", required=True)
    parser.add_argument("--source-report-sha256", required=True)
    parser.add_argument("--retry-authorization")
    parser.add_argument("--retry-authorization-sha256")
    parser.add_argument("--minimum-rgb-p95-improvement-mm", type=float, default=0.05)
    parser.add_argument("--minimum-rgb-rms-improvement-mm", type=float, default=0.0)
    parser.add_argument("--maximum-raw-imu-regression-mm", type=float, default=0.05)
    return parser.parse_args()


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def close(left, right, tolerance=1.0e-5):
    return finite(left) and abs(float(left) - float(right)) <= tolerance


def finite_float_or_nan(value):
    return float(value) if finite(value) else math.nan


def valid_sha256(value):
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value.lower())
    )


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while True:
            block = stream.read(8 * 1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def read_unique_env(path):
    result = {}
    with Path(path).open(encoding="utf-8") as stream:
        for line_number, raw in enumerate(stream, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise ValueError("invalid env line {}".format(line_number))
            key, value = line.split("=", 1)
            if key in result:
                raise ValueError("duplicate env key {}".format(key))
            result[key] = value
    return result


def audit_state_contract(row, report, expected):
    failures = []
    if report.get("status") != "PASS":
        failures.append("row {} postflight is not PASS".format(row))
    if str(report.get("row")) != row:
        failures.append("row {} row binding mismatch".format(row))
    if report.get("condition") != expected["condition"]:
        failures.append("row {} condition differs from frozen screen".format(row))

    config = report.get("effective_config_last", {})
    if not close(config.get("w_slosh"), expected["w_slosh"]):
        failures.append("row {} w_slosh differs from frozen screen".format(row))
    for field in ("w_smooth", "w_alpha", "w_du_a", "w_du_vs"):
        if not close(config.get(field), expected["smooth"]):
            failures.append("row {} {} differs from frozen screen".format(row, field))
    if not close(config.get("delay_phase_mode_code"), 4.0):
        failures.append("row {} did not run fixed_robot_only mode".format(row))

    internal = report.get("internal_state", {})
    aggregate = internal.get("delay_compensation_applied_fraction")
    robot = internal.get("robot_delay_compensation_applied_fraction")
    liquid = internal.get("liquid_delay_compensation_applied_fraction")
    source = internal.get("solver_source_code_fraction")
    if not finite(aggregate) or float(aggregate) < 0.98:
        failures.append("row {} lacks aggregate delay application".format(row))
    if not finite(robot) or float(robot) < 0.98:
        failures.append("row {} lacks robot delay application".format(row))
    if not finite(liquid) or float(liquid) > 0.02:
        failures.append("row {} applied forbidden liquid delay rollout".format(row))
    if not finite(source) or float(source) < 0.98:
        failures.append("row {} lacks processed-IMU solver-input coverage".format(row))
    return failures


def normalized_metrics(row, report, expected):
    internal = report.get("internal_state", {})
    rgb = report.get("online_rgb", {})
    return {
        "row": row,
        **expected,
        "rgb_p95_mm": finite_float_or_nan(rgb.get("h_vis_p95_mm")),
        "rgb_rms_mm": finite_float_or_nan(rgb.get("h_vis_rms_mm")),
        "raw_imu_p95_mm": finite_float_or_nan(internal.get("imu_modal_height_p95_mm")),
        "tracking_contour_p95_m": finite_float_or_nan(
            report.get("tracking", {}).get("contour_p95_m")
        ),
        "tracking_yaw_p95_rad": finite_float_or_nan(
            report.get("tracking", {}).get("yaw_p95_rad")
        ),
        "solver_p95_ms": finite_float_or_nan(
            report.get("runtime", {}).get("solver_p95_ms")
        ),
    }


def evaluate_screen(reports, p95_threshold=0.05, rms_threshold=0.0, raw_regression=0.05):
    failures = []
    if set(reports) != set(EXPECTED_ROWS):
        failures.append(
            "row set mismatch: expected={} actual={}".format(
                sorted(EXPECTED_ROWS), sorted(reports)
            )
        )

    normalized = {}
    for row, expected in EXPECTED_ROWS.items():
        report = reports.get(row)
        if report is None:
            continue
        failures.extend(audit_state_contract(row, report, expected))
        normalized[row] = normalized_metrics(row, report, expected)
        for field in (
            "rgb_p95_mm",
            "rgb_rms_mm",
            "raw_imu_p95_mm",
            "tracking_contour_p95_m",
            "tracking_yaw_p95_rad",
            "solver_p95_ms",
        ):
            if not finite(normalized[row][field]):
                failures.append("row {} lacks finite {}".format(row, field))

    baseline = normalized.get("01")
    candidates = []
    if baseline is not None:
        for row in ("02", "03", "04", "05"):
            candidate = normalized.get(row)
            if candidate is None:
                continue
            p95_delta = baseline["rgb_p95_mm"] - candidate["rgb_p95_mm"]
            rms_delta = baseline["rgb_rms_mm"] - candidate["rgb_rms_mm"]
            raw_delta = baseline["raw_imu_p95_mm"] - candidate["raw_imu_p95_mm"]
            positive = (
                p95_delta >= p95_threshold
                and rms_delta >= rms_threshold
                and raw_delta >= -raw_regression
                and reports[row].get("status") == "PASS"
            )
            score = p95_delta + 0.5 * rms_delta + 0.25 * max(0.0, raw_delta)
            candidates.append(
                {
                    **candidate,
                    "rgb_p95_improvement_mm": p95_delta,
                    "rgb_rms_improvement_mm": rms_delta,
                    "raw_imu_p95_improvement_mm": raw_delta,
                    "screen_positive": positive,
                    "ranking_score": score,
                }
            )

    positives = sorted(
        (candidate for candidate in candidates if candidate["screen_positive"]),
        key=lambda candidate: (-candidate["ranking_score"], candidate["row"]),
    )
    selected = positives[0] if positives and not failures else None
    return {
        "status": (
            "SCREEN_INVALID"
            if failures
            else "PROMOTE_FOR_PAIRED_CONFIRMATION"
            if selected
            else "NO_PROMOTION"
        ),
        "failures": failures,
        "baseline": baseline,
        "candidates": candidates,
        "selected": selected,
    }


def audit_bag(report, allowed_parent):
    failures = []
    bag_text = report.get("bag")
    if not bag_text:
        return ["postflight does not identify its bag"]
    bag_path = Path(bag_text).expanduser().resolve()
    if bag_path.parent != allowed_parent:
        failures.append("bag is outside the frozen G3R2 root")
        return failures
    if not bag_path.is_file():
        failures.append("bag is missing: {}".format(bag_path))
        return failures
    if report.get("bag_size_bytes") != bag_path.stat().st_size:
        failures.append("bag size differs from postflight")
    expected_sha = report.get("bag_sha256")
    if not valid_sha256(expected_sha):
        failures.append("postflight lacks a valid bag SHA-256")
    elif sha256_file(bag_path) != expected_sha:
        failures.append("bag SHA-256 differs from postflight")
    return failures


def build_confirmation_plan(selected, baseline_report_sha256, screen_report_sha256):
    if selected is None:
        return None
    order = (
        ("01", "01", "01", "Bsmooth"),
        ("02", "01", "02", selected["condition"]),
        ("03", "02", "01", selected["condition"]),
        ("04", "02", "02", "Bsmooth"),
        ("05", "03", "01", "Bsmooth"),
        ("06", "03", "02", selected["condition"]),
    )
    return {
        "schema_version": 1,
        "status": "DRAFT_REQUIRES_NEW_RELEASE_FREEZE",
        "scope": "G3R2_DEVELOPMENT_PAIRED_CONFIRMATION",
        "selected_condition": selected["condition"],
        "selected_weights": {
            "w_slosh": selected["w_slosh"],
            "w_smooth": selected["smooth"],
            "w_alpha": selected["smooth"],
            "w_du_a": selected["smooth"],
            "w_du_vs": selected["smooth"],
        },
        "bindings": {
            "baseline_report_sha256": baseline_report_sha256,
            "screen_report_sha256": screen_report_sha256,
        },
        "paired_blocks": 3,
        "planned_rows": [
            {"row": row, "block": block, "position": position, "condition": condition}
            for row, block, position, condition in order
        ],
        "instructions": [
            "Generate and commit a new confirmation wrapper before motion.",
            "Do not alter selected weights after viewing confirmation outcomes.",
            "These development blocks do not enter the formal sample count.",
        ],
    }


def main():
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    baseline_path = Path(args.baseline_report).expanduser().resolve()
    retry_authorization_path = (
        Path(args.retry_authorization).expanduser().resolve()
        if args.retry_authorization
        else None
    )
    for name, value in (
        ("baseline-report", args.baseline_report_sha256),
        ("screen-prereg", args.screen_prereg_sha256),
        ("source-report", args.source_report_sha256),
    ):
        if not valid_sha256(value):
            raise SystemExit("{} SHA-256 must be 64 hexadecimal characters".format(name))
    if not baseline_path.is_file() or sha256_file(baseline_path) != args.baseline_report_sha256:
        raise SystemExit("baseline postflight is missing or differs from the frozen hash")
    if bool(retry_authorization_path) != bool(args.retry_authorization_sha256):
        raise SystemExit("retry authorization path and SHA-256 must be provided together")
    retry_authorization = None
    if retry_authorization_path:
        if not valid_sha256(args.retry_authorization_sha256):
            raise SystemExit("retry authorization SHA-256 is invalid")
        if not retry_authorization_path.is_file() or sha256_file(
            retry_authorization_path
        ) != args.retry_authorization_sha256:
            raise SystemExit("retry authorization is missing or differs from its frozen hash")
        retry_authorization = read_unique_env(retry_authorization_path)

    with baseline_path.open(encoding="utf-8") as stream:
        baseline = json.load(stream)
    reports = {"01": baseline}
    report_paths = {"01": baseline_path}
    integrity_failures = []
    if baseline.get("protocol") != BASELINE_PROTOCOL:
        integrity_failures.append("baseline protocol mismatch")
    integrity_failures.extend(audit_bag(baseline, baseline_path.parent))

    for path in sorted(root.glob("DEV_G3R2_*_g3r2_screen_postflight.json")):
        with path.open(encoding="utf-8") as stream:
            report = json.load(stream)
        row = str(report.get("row", ""))
        if row in reports:
            raise SystemExit("duplicate G3R2 row {}".format(row))
        reports[row] = report
        report_paths[row] = path
        if report.get("protocol") != SCREEN_PROTOCOL:
            integrity_failures.append("row {} protocol mismatch".format(row))
        bindings = report.get("bindings", {})
        if bindings.get("prereg_sha256") != args.screen_prereg_sha256:
            integrity_failures.append("row {} screen prereg mismatch".format(row))
        if bindings.get("source_report_sha256") != args.source_report_sha256:
            integrity_failures.append("row {} source report mismatch".format(row))
        integrity_failures.extend(
            "row {} {}".format(row, failure) for failure in audit_bag(report, root)
        )

    excluded_acquisition_attempts = []
    retry_paths = sorted(root.glob("DEV_G3R2_*_g3r2_screen_retry_postflight.json"))
    if retry_paths:
        if len(retry_paths) != 1:
            raise SystemExit("expected exactly one authorized G3R2 retry postflight")
        if retry_authorization is None:
            raise SystemExit("retry postflight exists without frozen authorization")
        retry_path = retry_paths[0]
        with retry_path.open(encoding="utf-8") as stream:
            retry_report = json.load(stream)
        if retry_path.name != RETRY_ROW03_REPORT_NAME:
            integrity_failures.append("retry postflight attempt identity mismatch")
        retry_row = str(retry_report.get("row", ""))
        if retry_row != "03" or retry_report.get("condition") != "W5_S10":
            integrity_failures.append("retry postflight is not frozen Row 03 W5_S10")
        retry_bag = Path(str(retry_report.get("bag", ""))).expanduser()
        if retry_bag.name != RETRY_ROW03_ATTEMPT_ID + ".bag":
            integrity_failures.append("retry bag attempt identity mismatch")
        failed_report = reports.get("03")
        failed_path = report_paths.get("03")
        if failed_report is None or failed_path is None:
            integrity_failures.append("retry has no preserved failed Row 03 attempt 01")
        else:
            if failed_path.name != FAILED_ROW03_REPORT_NAME:
                integrity_failures.append("excluded Row 03 postflight attempt identity mismatch")
            failed_bag = Path(str(failed_report.get("bag", ""))).expanduser()
            if failed_bag.name != FAILED_ROW03_ATTEMPT_ID + ".bag":
                integrity_failures.append("excluded Row 03 bag attempt identity mismatch")
            if retry_authorization.get("report_type") != "DEVELOPMENT_RETRY_AUTHORIZATION":
                integrity_failures.append("retry authorization report type mismatch")
            if retry_authorization.get("status") != "PASS" or retry_authorization.get("retry_authorized") != "true":
                integrity_failures.append("retry authorization is not PASS/authorized")
            if retry_authorization.get("failed_attempt_id") != FAILED_ROW03_ATTEMPT_ID:
                integrity_failures.append("retry authorization failed-attempt mismatch")
            if retry_authorization.get("authorized_attempt_id") != RETRY_ROW03_ATTEMPT_ID:
                integrity_failures.append("retry authorization attempt-02 mismatch")
            if retry_authorization.get("retry_of_attempt_id") != FAILED_ROW03_ATTEMPT_ID:
                integrity_failures.append("retry authorization retry-of mismatch")
            if retry_authorization.get("maximum_authorized_attempt") != "02":
                integrity_failures.append("retry authorization attempt limit mismatch")
            if retry_authorization.get("failure_class") != "METHOD_INDEPENDENT_ACQUISITION":
                integrity_failures.append("retry authorization failure class mismatch")
            if retry_authorization.get("failure_reason_code") != "REALSENSE_SOURCE_TIMESTAMP_UNSTABLE_AT_VISUAL_START":
                integrity_failures.append("retry authorization failure reason mismatch")
            if retry_authorization.get("method_failure") != "false":
                integrity_failures.append("retry authorization incorrectly marks a method failure")
            if retry_authorization.get("screen_prereg_sha256") != args.screen_prereg_sha256:
                integrity_failures.append("retry authorization screen-prereg mismatch")
            if retry_authorization.get("failed_postflight_sha256") != sha256_file(failed_path):
                integrity_failures.append("retry authorization failed-postflight hash mismatch")
            if retry_authorization.get("failed_bag_sha256") != failed_report.get("bag_sha256"):
                integrity_failures.append("retry authorization failed-bag hash mismatch")
            if failed_report.get("status") != "FAIL":
                integrity_failures.append("excluded Row 03 attempt 01 is not FAIL")
            excluded_acquisition_attempts.append(
                {
                    "row": "03",
                    "attempt": "01",
                    "condition": failed_report.get("condition"),
                    "status": failed_report.get("status"),
                    "failure_class": retry_authorization.get("failure_class"),
                    "failure_reason_code": retry_authorization.get("failure_reason_code"),
                    "bag": failed_report.get("bag"),
                    "bag_sha256": failed_report.get("bag_sha256"),
                    "postflight": str(failed_path),
                    "postflight_sha256": sha256_file(failed_path),
                    "eligible_for_selection": False,
                }
            )

        if retry_report.get("protocol") != SCREEN_PROTOCOL:
            integrity_failures.append("retry protocol mismatch")
        bindings = retry_report.get("bindings", {})
        if bindings.get("prereg_sha256") != args.screen_prereg_sha256:
            integrity_failures.append("retry screen prereg mismatch")
        if bindings.get("source_report_sha256") != args.source_report_sha256:
            integrity_failures.append("retry source report mismatch")
        integrity_failures.extend(
            "row 03 retry {}".format(failure) for failure in audit_bag(retry_report, root)
        )
        reports["03"] = retry_report
        report_paths["03"] = retry_path
    elif retry_authorization is not None:
        integrity_failures.append("retry authorization supplied but retry postflight is missing")

    result = evaluate_screen(
        reports,
        p95_threshold=args.minimum_rgb_p95_improvement_mm,
        rms_threshold=args.minimum_rgb_rms_improvement_mm,
        raw_regression=args.maximum_raw_imu_regression_mm,
    )
    if integrity_failures:
        result["failures"].extend(integrity_failures)
        result["status"] = "SCREEN_INVALID"
        result["selected"] = None

    dataset = [
        {
            "row": row,
            "attempt": "02" if row == "03" and retry_paths else "01",
            "condition": reports[row].get("condition"),
            "bag": reports[row].get("bag"),
            "bag_sha256": reports[row].get("bag_sha256"),
            "postflight": str(report_paths[row]),
            "postflight_sha256": sha256_file(report_paths[row]),
        }
        for row in sorted(reports)
    ]
    report = {
        "schema_version": 1,
        "report_type": "G3R2_ROBOT_ONLY_SINGLE_RUN_WEIGHT_SCREEN",
        "scope": "DEVELOPMENT_SCREEN_ONLY",
        "status": result["status"],
        "decision": result["status"],
        "thresholds": {
            "minimum_rgb_p95_improvement_mm": args.minimum_rgb_p95_improvement_mm,
            "minimum_rgb_rms_improvement_mm": args.minimum_rgb_rms_improvement_mm,
            "maximum_raw_imu_regression_mm": args.maximum_raw_imu_regression_mm,
        },
        "bindings": {
            "baseline_report_sha256": args.baseline_report_sha256,
            "screen_prereg_sha256": args.screen_prereg_sha256,
            "source_report_sha256": args.source_report_sha256,
            "retry_authorization_sha256": args.retry_authorization_sha256 or "",
        },
        "dataset": dataset,
        "excluded_acquisition_attempts": excluded_acquisition_attempts,
        "baseline": result["baseline"],
        "candidates": result["candidates"],
        "selected_candidate": result["selected"],
        "failures": result["failures"],
        "limitations": [
            "Each candidate has one run; this screen cannot establish repeatability or efficacy.",
            "Only a selected positive candidate may enter a newly frozen paired confirmation.",
            "Unselected candidates are not repeated after outcomes are observed.",
        ],
    }
    out_json = root / "G3R2_WEIGHT_SCREEN_REPORT.json"
    out_csv = root / "G3R2_WEIGHT_SCREEN_METRICS.csv"
    out_json.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with out_csv.open("w", encoding="utf-8", newline="") as stream:
        fields = (
            "row",
            "condition",
            "w_slosh",
            "smooth",
            "rgb_p95_mm",
            "rgb_rms_mm",
            "raw_imu_p95_mm",
            "rgb_p95_improvement_mm",
            "rgb_rms_improvement_mm",
            "raw_imu_p95_improvement_mm",
            "screen_positive",
            "ranking_score",
        )
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(
            {field: item.get(field) for field in fields} for item in result["candidates"]
        )
    for artifact in (out_json, out_csv):
        artifact.with_suffix(artifact.suffix + ".sha256").write_text(
            "{}  {}\n".format(sha256_file(artifact), artifact), encoding="utf-8"
        )

    if report["selected_candidate"]:
        plan = build_confirmation_plan(
            report["selected_candidate"], args.baseline_report_sha256, sha256_file(out_json)
        )
        plan_path = root / "G3R2_CONFIRMATION_PLAN.draft.json"
        plan_path.write_text(
            json.dumps(plan, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        plan_path.with_suffix(plan_path.suffix + ".sha256").write_text(
            "{}  {}\n".format(sha256_file(plan_path), plan_path), encoding="utf-8"
        )

    print("[G3R2 screen] {}".format(report["status"]))
    if report["selected_candidate"]:
        print("  selected = {}".format(report["selected_candidate"]["condition"]))
    print("  report = {}".format(out_json))
    return {
        "PROMOTE_FOR_PAIRED_CONFIRMATION": 0,
        "NO_PROMOTION": 10,
        "SCREEN_INVALID": 20,
    }[report["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
