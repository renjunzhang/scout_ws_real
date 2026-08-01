#!/usr/bin/env python3
"""Prepare frozen SmoothMatch and FixedProfile candidates from G3 development data."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import yaml


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def round_to_step(value: float, step: float) -> float:
    if step <= 0.0:
        raise RuntimeError("rounding step must be positive")
    return round(value / step) * step


def completion_seconds(postflight: Mapping[str, Any]) -> float:
    start = float(postflight["motion_start_sec"])
    arrival = float(postflight["first_arrival_sec"])
    duration = arrival - start
    if not math.isfinite(duration) or duration <= 0.0:
        raise RuntimeError("postflight has invalid motion/arrival timestamps")
    return duration


def derive_smooth_match(
    reports: Mapping[str, Mapping[str, Any]], config: Mapping[str, Any]
) -> Dict[str, Any]:
    rows = {
        "01": ("01", "Bsmooth"),
        "02": ("01", "W5"),
        "03": ("02", "W5"),
        "04": ("02", "Bsmooth"),
        "05": ("03", "W5"),
        "06": ("03", "Bsmooth"),
        "07": ("04", "Bsmooth"),
        "08": ("04", "W5"),
    }
    block_values: Dict[str, Dict[str, float]] = {}
    failures: List[str] = []
    for row, (block, condition) in rows.items():
        report = reports.get(row)
        if report is None:
            failures.append("missing G3 row {}".format(row))
            continue
        if str(report.get("condition")) != condition:
            failures.append("G3 row {} condition mismatch".format(row))
            continue
        if report.get("status") != "PASS":
            failures.append("G3 row {} postflight is not PASS".format(row))
            continue
        block_values.setdefault(block, {})[condition] = completion_seconds(report)

    nominal = float(config["nominal_bsmooth_v_ref_m_s"])
    ratios = []
    paired = []
    for block in ("01", "02", "03", "04"):
        values = block_values.get(block, {})
        if set(values) != {"Bsmooth", "W5"}:
            failures.append("G3 block {} lacks an eligible completion pair".format(block))
            continue
        ratio = values["Bsmooth"] / values["W5"]
        ratios.append(ratio)
        paired.append(
            {
                "block": block,
                "bsmooth_completion_sec": values["Bsmooth"],
                "w5_completion_sec": values["W5"],
                "completion_ratio_bsmooth_over_w5": ratio,
            }
        )
    raw_v_ref = nominal * statistics.median(ratios) if ratios else math.nan
    rounded = (
        round_to_step(raw_v_ref, float(config["rounding_step_m_s"]))
        if math.isfinite(raw_v_ref)
        else math.nan
    )
    safe_min = float(config["safe_v_ref_min_m_s"])
    safe_max = float(config["safe_v_ref_max_m_s"])
    if math.isfinite(rounded):
        rounded = min(safe_max, max(safe_min, rounded))
    target = (
        statistics.median(item["w5_completion_sec"] for item in paired)
        if paired
        else math.nan
    )
    predicted = (
        statistics.median(
            item["bsmooth_completion_sec"] * nominal / rounded for item in paired
        )
        if paired and rounded > 0.0
        else math.nan
    )
    relative_error = (
        abs(predicted - target) / target
        if math.isfinite(predicted) and math.isfinite(target) and target > 0.0
        else math.inf
    )
    if not (safe_min <= rounded <= safe_max):
        failures.append("derived SmoothMatch v_ref is outside the frozen safe interval")
    return {
        "status": "READY" if not failures else "NO-GO",
        "failures": failures,
        "paired_blocks": paired,
        "nominal_bsmooth_v_ref_m_s": nominal,
        "raw_v_ref_m_s": raw_v_ref,
        "frozen_v_ref_m_s": rounded,
        "safe_v_ref_min_m_s": safe_min,
        "safe_v_ref_max_m_s": safe_max,
        "rounding_step_m_s": float(config["rounding_step_m_s"]),
        "target_w5_completion_sec": target,
        "predicted_smooth_match_completion_sec": predicted,
        "predicted_relative_error": relative_error,
        "real_confirmation_relative_tolerance": float(
            config["completion_relative_tolerance"]
        ),
    }


def run_generator(
    generator: Path,
    path_file: Path,
    out_csv: Path,
    out_report: Path,
    base_speed: float,
    config: Mapping[str, Any],
) -> Tuple[int, Mapping[str, Any], str]:
    command = [
        sys.executable,
        str(generator),
        "--path-file",
        str(path_file),
        "--out-csv",
        str(out_csv),
        "--report-json",
        str(out_report),
        "--base-speed",
        "{:.12g}".format(base_speed),
        "--v-max",
        str(config["v_max_m_s"]),
        "--omega-max",
        str(config["omega_max_rad_s"]),
        "--a-max",
        str(config["a_max_m_s2"]),
        "--decel-max",
        str(config["decel_max_m_s2"]),
        "--alpha-max",
        str(config["alpha_max_rad_s2"]),
        "--lateral-accel-max",
        str(config["lateral_accel_max_m_s2"]),
        "--omega-n",
        str(config["omega_n_rad_s"]),
        "--damping-ratio",
        str(config["damping_ratio"]),
        "--ds",
        str(config["ds_m"]),
        "--curvature-smooth-passes",
        str(config["curvature_smooth_passes"]),
        "--method-name",
        "FIXED_PROFILE_HAMAGUCHI_ZV",
    ]
    completed = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    report: Mapping[str, Any] = {}
    if out_report.is_file():
        report = json.loads(out_report.read_text(encoding="utf-8"))
    return completed.returncode, report, completed.stdout


def tune_fixed_profile(
    generator: Path,
    path_file: Path,
    out_dir: Path,
    target_completion: float,
    config: Mapping[str, Any],
) -> Dict[str, Any]:
    if not math.isfinite(target_completion) or target_completion <= 0.0:
        return {"status": "NO-GO", "failures": ["target completion is not finite"]}
    lower = float(config["base_speed_min_m_s"])
    upper = float(config["base_speed_max_m_s"])
    step = float(config["base_speed_rounding_step_m_s"])
    candidates: Dict[float, Mapping[str, Any]] = {}
    logs: Dict[float, str] = {}
    with tempfile.TemporaryDirectory(prefix="g5_profile_tune_") as temporary:
        temp = Path(temporary)
        for iteration in range(14):
            speed = 0.5 * (lower + upper)
            csv_path = temp / "candidate_{:02d}.csv".format(iteration)
            report_path = temp / "candidate_{:02d}.json".format(iteration)
            _code, report, log = run_generator(
                generator, path_file, csv_path, report_path, speed, config
            )
            candidates[speed] = report
            logs[speed] = log
            duration = float(report.get("duration_sec", math.inf))
            if duration > target_completion:
                lower = speed
            else:
                upper = speed
        central = round_to_step(0.5 * (lower + upper), step)
        discrete = sorted(
            {
                max(float(config["base_speed_min_m_s"]), min(float(config["base_speed_max_m_s"]), central + offset * step))
                for offset in (-2, -1, 0, 1, 2)
            }
        )
        scored = []
        for index, speed in enumerate(discrete):
            csv_path = temp / "discrete_{:02d}.csv".format(index)
            report_path = temp / "discrete_{:02d}.json".format(index)
            code, report, log = run_generator(
                generator, path_file, csv_path, report_path, speed, config
            )
            duration = float(report.get("duration_sec", math.inf))
            constraint_pass = report.get("status") == "PASS" and code == 0
            scored.append((not constraint_pass, abs(duration - target_completion), speed, report, log))
        scored.sort(key=lambda item: (item[0], item[1], item[2]))
        _invalid, _error, frozen_speed, _candidate_report, _log = scored[0]

    final_csv = out_dir / "G5_FIXED_PROFILE_H0_C1.csv"
    final_report = out_dir / "G5_FIXED_PROFILE_GENERATION_REPORT.json"
    code, generation, generator_log = run_generator(
        generator, path_file, final_csv, final_report, frozen_speed, config
    )
    (out_dir / "G5_FIXED_PROFILE_GENERATOR.log").write_text(generator_log, encoding="utf-8")
    failures = []
    if code != 0 or generation.get("status") != "PASS":
        failures.append("final fixed profile generator/constraint audit failed")
    duration = float(generation.get("duration_sec", math.inf))
    relative_error = abs(duration - target_completion) / target_completion
    if relative_error > float(config["completion_relative_tolerance"]):
        failures.append("offline fixed-profile duration is outside the frozen completion tolerance")
    return {
        "status": "READY" if not failures else "NO-GO",
        "failures": failures,
        "frozen_base_speed_m_s": frozen_speed,
        "target_w5_completion_sec": target_completion,
        "offline_profile_completion_sec": duration,
        "offline_relative_error": relative_error,
        "real_confirmation_relative_tolerance": float(
            config["completion_relative_tolerance"]
        ),
        "profile_csv": str(final_csv),
        "profile_sha256": sha256_file(final_csv) if final_csv.is_file() else "",
        "generation_report": str(final_report),
        "generation_report_sha256": sha256_file(final_report) if final_report.is_file() else "",
        "generation": generation,
    }


def load_postflights(g3_report: Mapping[str, Any]) -> Dict[str, Mapping[str, Any]]:
    reports = {}
    for item in g3_report.get("dataset_index", []):
        row = str(item.get("row", ""))
        path = Path(str(item.get("postflight", ""))).expanduser().resolve()
        if row and path.is_file():
            reports[row] = json.loads(path.read_text(encoding="utf-8"))
    return reports


def write_yaml(path: Path, data: Mapping[str, Any]) -> None:
    path.write_text(yaml.safe_dump(dict(data), sort_keys=True), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g3-report", required=True)
    parser.add_argument("--path-file", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--out-dir", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    g3_report_path = Path(args.g3_report).expanduser().resolve()
    path_file = Path(args.path_file).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    repo_root = Path(args.repo_root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    g3_report = json.loads(g3_report_path.read_text(encoding="utf-8"))
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise RuntimeError("G5 config must be schema_version 1")
    failures = []
    if g3_report.get("status") != "PASS":
        failures.append("G3 efficacy report is not PASS")
    reports = load_postflights(g3_report)
    smooth = derive_smooth_match(reports, config["smooth_match"])
    failures.extend("SmoothMatch: " + item for item in smooth["failures"])

    generator = (repo_root / config["fixed_profile"]["generator"]).resolve()
    tracker_config = (repo_root / config["fixed_profile"]["tracker_config"]).resolve()
    if not generator.is_file() or not tracker_config.is_file():
        raise RuntimeError("FixedProfile generator/tracker config is missing")
    fixed = tune_fixed_profile(
        generator,
        path_file,
        out_dir,
        float(smooth["target_w5_completion_sec"]),
        config["fixed_profile"],
    )
    failures.extend("FixedProfile: " + item for item in fixed["failures"])

    smooth_config_path = out_dir / "G5_SMOOTH_MATCH_CONFIG.yaml"
    fixed_config_path = out_dir / "G5_FIXED_PROFILE_CONFIG.yaml"
    smooth_config = {
        "schema_version": 1,
        "condition": "SmoothMatch",
        "variant": "B_smooth",
        "w_slosh": 0.0,
        "v_ref_m_s": smooth["frozen_v_ref_m_s"],
        "safe_v_ref_min_m_s": smooth["safe_v_ref_min_m_s"],
        "safe_v_ref_max_m_s": smooth["safe_v_ref_max_m_s"],
        "completion_target_sec": smooth["target_w5_completion_sec"],
        "completion_relative_tolerance": smooth["real_confirmation_relative_tolerance"],
        "g3_report_sha256": sha256_file(g3_report_path),
    }
    fixed_config = {
        "schema_version": 1,
        "condition": "FixedProfile",
        "generator": str(generator),
        "generator_sha256": sha256_file(generator),
        "tracker_config": str(tracker_config),
        "tracker_config_sha256": sha256_file(tracker_config),
        "profile_csv": fixed.get("profile_csv", ""),
        "profile_sha256": fixed.get("profile_sha256", ""),
        "generation_report": fixed.get("generation_report", ""),
        "generation_report_sha256": fixed.get("generation_report_sha256", ""),
        "base_speed_m_s": fixed.get("frozen_base_speed_m_s"),
        "completion_target_sec": fixed.get("target_w5_completion_sec"),
        "offline_completion_sec": fixed.get("offline_profile_completion_sec"),
        "completion_relative_tolerance": fixed.get("real_confirmation_relative_tolerance"),
        "runtime_regeneration_allowed": False,
        "online_liquid_feedback_allowed": False,
    }
    write_yaml(smooth_config_path, smooth_config)
    write_yaml(fixed_config_path, fixed_config)

    report = {
        "schema_version": 1,
        "report_type": "G5_COMPARATOR_PREREG",
        "protocol": config["protocol"],
        "scope": "MINIMAL_DEVELOPMENT_REAL_CONFIRMATION",
        "status": "READY_FOR_REAL_CONFIRMATION" if not failures else "NO-GO",
        "failures": failures,
        "bindings": {
            "g3_report": str(g3_report_path),
            "g3_report_sha256": sha256_file(g3_report_path),
            "path_file": str(path_file),
            "path_sha256": sha256_file(path_file),
            "config": str(config_path),
            "config_sha256": sha256_file(config_path),
        },
        "smooth_match": smooth,
        "fixed_profile": fixed,
        "smooth_match_config": str(smooth_config_path),
        "smooth_match_config_sha256": sha256_file(smooth_config_path),
        "fixed_profile_config": str(fixed_config_path),
        "fixed_profile_config_sha256": sha256_file(fixed_config_path),
        "limitations": [
            "G3 completion data tune comparators but never contribute to formal n.",
            "Offline duration matching does not replace the two minimal real confirmation trials.",
            "The generated H0 profile is a G5 smoke artifact, not a formal H1 profile.",
        ],
    }
    report_path = out_dir / "G5_COMPARATOR_PREREG_REPORT.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (out_dir / "G5_COMPARATOR_PREREG_REPORT.json.sha256").write_text(
        "{}  {}\n".format(sha256_file(report_path), report_path), encoding="utf-8"
    )
    print("[G5 prepare] {}".format(report["status"]))
    print("  SmoothMatch v_ref={:.3f}".format(float(smooth["frozen_v_ref_m_s"])))
    print(
        "  FixedProfile base={:.3f}, offline T={:.3f}s".format(
            float(fixed.get("frozen_base_speed_m_s", math.nan)),
            float(fixed.get("offline_profile_completion_sec", math.nan)),
        )
    )
    print("  report={}".format(report_path))
    for failure in failures:
        print("  NO-GO: {}".format(failure), file=sys.stderr)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
