#!/usr/bin/env python3
"""Aggregate the two G5 minimal real confirmation trials."""

import argparse
import hashlib
import json
import math
from pathlib import Path


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()
    prereg_path = root / "G5_COMPARATOR_PREREG_REPORT.json"
    prereg = json.loads(prereg_path.read_text(encoding="utf-8"))
    failures = []
    if prereg.get("status") != "READY_FOR_REAL_CONFIRMATION":
        failures.append("G5 comparator prereg is not READY_FOR_REAL_CONFIRMATION")

    reports = {}
    for path in root.glob("*_g5_postflight.json"):
        report = json.loads(path.read_text(encoding="utf-8"))
        condition = report.get("condition")
        if condition in reports:
            failures.append("duplicate {} postflight".format(condition))
        reports[condition] = (path, report)
    if set(reports) != {"SmoothMatch", "FixedProfile"}:
        failures.append("G5 requires exactly SmoothMatch and FixedProfile postflights")

    metrics = {}
    for condition in ("SmoothMatch", "FixedProfile"):
        item = reports.get(condition)
        if item is None:
            continue
        path, report = item
        if report.get("status") != "PASS":
            failures.append("{} postflight is not PASS".format(condition))
        source = prereg["smooth_match"] if condition == "SmoothMatch" else prereg["fixed_profile"]
        target = float(source["target_w5_completion_sec"])
        tolerance = float(source["real_confirmation_relative_tolerance"])
        actual = float(report.get("completion_sec", math.nan))
        relative_error = abs(actual - target) / target if target > 0.0 else math.inf
        if relative_error > tolerance:
            failures.append("{} real completion is outside +/-{:.1%}".format(condition, tolerance))
        metrics[condition] = {
            "postflight": str(path),
            "postflight_sha256": sha256_file(path),
            "target_completion_sec": target,
            "actual_completion_sec": actual,
            "relative_error": relative_error,
            "tolerance": tolerance,
            "tracking_cross_track_p95_m": report.get("tracking", {}).get("cross_track_p95_m"),
        }

    report = {
        "schema_version": 1,
        "report_type": "G5_COMPARATOR_FAIRNESS",
        "scope": "MINIMAL_DEVELOPMENT_REAL_CONFIRMATION",
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "prereg": str(prereg_path),
        "prereg_sha256": sha256_file(prereg_path),
        "metrics": metrics,
        "limitations": [
            "One real trial per comparator confirms executability and completion matching, not physical efficacy.",
            "G5 trials are development data and never enter formal n.",
        ],
    }
    out = root / "G5_COMPARATOR_FAIRNESS_REPORT.json"
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (root / "G5_COMPARATOR_FAIRNESS_REPORT.json.sha256").write_text(
        "{}  {}\n".format(sha256_file(out), out), encoding="utf-8"
    )
    print("[G5 analyzer] {}".format(report["status"]))
    print("  report={}".format(out))
    for failure in failures:
        print("  FAIL: {}".format(failure))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
