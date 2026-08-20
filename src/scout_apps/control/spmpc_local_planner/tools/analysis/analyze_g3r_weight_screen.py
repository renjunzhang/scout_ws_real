#!/usr/bin/env python3
"""Analyze the one-run-per-candidate G3R weight screen.

The screen is deliberately exploratory.  A promoted candidate must still pass
the separately planned paired confirmation stage; this script never upgrades a
single physical run into an efficacy claim.
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
EXPECTED_PROTOCOL = "G3R_raw_processed_imu_weight_screen_v1"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    parser.add_argument("--minimum-rgb-p95-improvement-mm", type=float, default=0.05)
    parser.add_argument("--minimum-rgb-rms-improvement-mm", type=float, default=0.0)
    parser.add_argument("--maximum-raw-imu-regression-mm", type=float, default=0.05)
    parser.add_argument("--alignment-report-sha256", required=True)
    parser.add_argument("--prereg-sha256", required=True)
    parser.add_argument("--source-report-sha256", required=True)
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


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def valid_sha256(value):
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value.lower())
    )


def audit_postflight(root, row, report, prereg_sha256, source_report_sha256):
    failures = []
    if report.get("protocol") != EXPECTED_PROTOCOL:
        failures.append("row {} protocol differs from frozen G3R screen".format(row))
    bindings = report.get("bindings", {})
    if bindings.get("prereg_sha256") != prereg_sha256:
        failures.append("row {} prereg binding mismatch".format(row))
    if bindings.get("source_report_sha256") != source_report_sha256:
        failures.append("row {} source-report binding mismatch".format(row))

    bag_text = report.get("bag")
    if not bag_text:
        failures.append("row {} postflight does not identify its bag".format(row))
        return failures
    bag_path = Path(bag_text).expanduser().resolve()
    if bag_path.parent != root:
        failures.append("row {} bag is outside the frozen G3R root".format(row))
        return failures
    if not bag_path.is_file():
        failures.append("row {} bag is missing: {}".format(row, bag_path))
        return failures
    if report.get("bag_size_bytes") != bag_path.stat().st_size:
        failures.append("row {} bag size differs from postflight".format(row))
    expected_bag_sha = report.get("bag_sha256")
    if not valid_sha256(expected_bag_sha):
        failures.append("row {} postflight lacks a valid bag SHA-256".format(row))
    elif sha256_file(bag_path) != expected_bag_sha:
        failures.append("row {} bag SHA-256 differs from postflight".format(row))
    return failures


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
        if report.get("status") != "PASS":
            failures.append("row {} postflight is not PASS".format(row))
        if report.get("condition") != expected["condition"]:
            failures.append("row {} condition differs from frozen screen".format(row))
        config = report.get("effective_config_last", {})
        if not close(config.get("w_slosh"), expected["w_slosh"]):
            failures.append("row {} w_slosh differs from frozen screen".format(row))
        for field in ("w_smooth", "w_alpha", "w_du_a", "w_du_vs"):
            if not close(config.get(field), expected["smooth"]):
                failures.append("row {} {} differs from frozen screen".format(row, field))
        if not close(config.get("delay_phase_mode_code"), 2.0):
            failures.append("row {} did not run delay shadow mode".format(row))
        internal = report.get("internal_state", {})
        delay_applied = internal.get("delay_compensation_applied_fraction")
        if not finite(delay_applied) or float(delay_applied) > 0.02:
            failures.append("row {} applied legacy delay compensation".format(row))
        source_fraction = internal.get("solver_source_code_fraction")
        if not finite(source_fraction) or float(source_fraction) < 0.98:
            failures.append("row {} lacks processed-IMU solver-input coverage".format(row))
        rgb = report.get("online_rgb", {})
        normalized[row] = {
            "row": row,
            **expected,
            "rgb_p95_mm": finite_float_or_nan(rgb.get("h_vis_p95_mm")),
            "rgb_rms_mm": finite_float_or_nan(rgb.get("h_vis_rms_mm")),
            "raw_imu_p95_mm": finite_float_or_nan(
                internal.get("imu_modal_height_p95_mm")
            ),
            "tracking_contour_p95_m": finite_float_or_nan(
                report.get("tracking", {}).get("contour_p95_m")
            ),
            "solver_p95_ms": finite_float_or_nan(
                report.get("runtime", {}).get("solver_p95_ms")
            ),
        }
        for field in ("rgb_p95_mm", "rgb_rms_mm", "raw_imu_p95_mm"):
            if not finite(normalized[row][field]):
                failures.append("row {} lacks finite {}".format(row, field))
        if not finite(normalized[row]["tracking_contour_p95_m"]):
            failures.append("row {} lacks finite tracking metric".format(row))
        if not finite(normalized[row]["solver_p95_ms"]):
            failures.append("row {} lacks finite runtime metric".format(row))

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


def build_confirmation_plan(selected, alignment_report_sha256, screen_report_sha256):
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
        "scope": "DEVELOPMENT_PAIRED_CONFIRMATION",
        "selected_condition": selected["condition"],
        "selected_weights": {
            "w_slosh": selected["w_slosh"],
            "w_smooth": selected["smooth"],
            "w_alpha": selected["smooth"],
            "w_du_a": selected["smooth"],
            "w_du_vs": selected["smooth"],
        },
        "bindings": {
            "alignment_report_sha256": alignment_report_sha256,
            "screen_report_sha256": screen_report_sha256,
        },
        "paired_blocks": 3,
        "planned_rows": [
            {"row": row, "block": block, "position": position, "condition": condition}
            for row, block, position, condition in order
        ],
        "promotion_gate": {
            "minimum_positive_blocks": 2,
            "minimum_paired_mean_rgb_p95_improvement_mm": 0.05,
            "maximum_mean_raw_imu_regression_mm": 0.05,
            "all_postflights_must_pass": True,
        },
        "instructions": [
            "Generate and commit a confirmation wrapper bound to this plan before motion.",
            "Do not alter the selected weights after viewing confirmation outcomes.",
            "The three paired blocks remain development evidence and do not enter formal n.",
        ],
    }


def main():
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    for name, value in (
        ("alignment-report", args.alignment_report_sha256),
        ("prereg", args.prereg_sha256),
        ("source-report", args.source_report_sha256),
    ):
        if not valid_sha256(value):
            raise SystemExit("{} SHA-256 must be 64 hexadecimal characters".format(name))
    reports = {}
    report_paths = {}
    for path in sorted(root.glob("DEV_G3R_*_g3r_postflight.json")):
        with path.open(encoding="utf-8") as stream:
            report = json.load(stream)
        row = str(report.get("row", ""))
        if row in reports:
            raise SystemExit("duplicate G3R row {}".format(row))
        reports[row] = report
        report_paths[row] = path
    result = evaluate_screen(
        reports,
        p95_threshold=args.minimum_rgb_p95_improvement_mm,
        rms_threshold=args.minimum_rgb_rms_improvement_mm,
        raw_regression=args.maximum_raw_imu_regression_mm,
    )
    integrity_failures = []
    for row, report in reports.items():
        integrity_failures.extend(
            audit_postflight(
                root,
                row,
                report,
                args.prereg_sha256,
                args.source_report_sha256,
            )
        )
    if integrity_failures:
        result["failures"].extend(integrity_failures)
        result["status"] = "SCREEN_INVALID"
        result["selected"] = None
    dataset = []
    for row in sorted(reports):
        path = report_paths[row]
        dataset.append(
            {
                "row": row,
                "condition": reports[row].get("condition"),
                "bag": reports[row].get("bag"),
                "bag_sha256": reports[row].get("bag_sha256"),
                "postflight": str(path),
                "postflight_sha256": sha256_file(path),
            }
        )
    report = {
        "schema_version": 1,
        "report_type": "G3R_SINGLE_RUN_WEIGHT_SCREEN",
        "scope": "DEVELOPMENT_SCREEN_ONLY",
        "status": result["status"],
        "decision": result["status"],
        "thresholds": {
            "minimum_rgb_p95_improvement_mm": args.minimum_rgb_p95_improvement_mm,
            "minimum_rgb_rms_improvement_mm": args.minimum_rgb_rms_improvement_mm,
            "maximum_raw_imu_regression_mm": args.maximum_raw_imu_regression_mm,
        },
        "bindings": {
            "alignment_report_sha256": args.alignment_report_sha256,
            "prereg_sha256": args.prereg_sha256,
            "source_report_sha256": args.source_report_sha256,
        },
        "dataset": dataset,
        "baseline": result["baseline"],
        "candidates": result["candidates"],
        "selected_candidate": result["selected"],
        "failures": result["failures"],
        "limitations": [
            "Every condition has one run; this screen cannot establish repeatability or efficacy.",
            "Only the selected candidate may enter a newly frozen paired confirmation stage.",
            "Unselected candidates are not repeated after outcomes are observed.",
        ],
    }
    out_json = root / "G3R_WEIGHT_SCREEN_REPORT.json"
    out_csv = root / "G3R_WEIGHT_SCREEN_METRICS.csv"
    out_json.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    if report["selected_candidate"]:
        confirmation_plan = build_confirmation_plan(
            report["selected_candidate"],
            args.alignment_report_sha256,
            sha256_file(out_json),
        )
        confirmation_path = root / "G3R_CONFIRMATION_PLAN.draft.json"
        confirmation_path.write_text(
            json.dumps(confirmation_plan, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        confirmation_path.with_suffix(confirmation_path.suffix + ".sha256").write_text(
            "{}  {}\n".format(sha256_file(confirmation_path), confirmation_path),
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
        writer.writerows({field: item.get(field) for field in fields} for item in result["candidates"])
    for artifact in (out_json, out_csv):
        artifact.with_suffix(artifact.suffix + ".sha256").write_text(
            "{}  {}\n".format(sha256_file(artifact), artifact), encoding="utf-8"
        )
    print("[G3R screen] {}".format(report["status"]))
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
