#!/usr/bin/env python3
"""Run simulation-development smoke/pilot campaigns or emit a formal NO-GO.

This orchestrator never turns the independent plant smoke into a C0--C4
comparison.  A formal request is an auditable readiness check and exits
non-zero until separately frozen formal assets and condition bindings exist.
"""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
from typing import Any, Dict, Iterable, List, Sequence

import yaml


ALLOWED_OUTPUT_ROOT = Path("/data/a/spmpc_exec_identification")
DEVELOPMENT_STATUS = "development_candidate_unbound"
MOTION_COMMAND_END_SEC = 7.5
COUNT_EPSILON = 1.0e-12
MAX_UINT32 = (1 << 32) - 1
FORMAL_STATUS = "formal_simulation_release"
SMOKE_SCHEMA = "spmpc_independent_plant_smoke_v3"
CSV_COLUMNS = (
    "publish_time_sec", "sample_time_sec", "cmd_v", "cmd_omega",
    "linear_effective_time_sec", "angular_effective_time_sec",
    "linear_transport_jitter_sec", "angular_transport_jitter_sec",
    "active_v", "active_omega", "x", "y", "yaw", "v", "omega",
    "acceleration", "lateral_acceleration", "primary_eta_x",
    "primary_eta_y", "second_eta_x", "second_eta_y", "true_height_m",
    "measured_height_m",
)
METRICS = (
    "external_height_q95_m",
    "external_height_peak_m",
    "external_height_rms_m",
    "tail_height_rms_m",
    "max_abs_v_mps",
    "max_abs_omega_radps",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("configuration root must be a mapping")
    return value


def _finite_number(value: Any) -> bool:
    return (type(value) in (int, float) and
            math.isfinite(float(value)))


def _uint32_seed(value: Any) -> bool:
    return type(value) is int and 0 <= value <= MAX_UINT32


def _lowercase_sha256(value: Any) -> bool:
    return (isinstance(value, str) and len(value) == 64 and
            all(character in "0123456789abcdef" for character in value))


def _ceil_sample_count(duration_sec: float,
                       control_rate_hz: float) -> int:
    return int(math.ceil(
        duration_sec * control_rate_hz - COUNT_EPSILON))


def expected_smoke_timing(config: Dict[str, Any]) -> Dict[str, Any]:
    """Resolve the smoke/tail contract independently of the C++ runner."""
    experiment = config.get("experiment")
    plant = config.get("external_plant")
    if not isinstance(experiment, dict) or not isinstance(plant, dict):
        raise ValueError("missing smoke timing configuration")
    rate = experiment.get("control_rate_hz")
    fixed_tail = experiment.get("fixed_tail_sec")
    linear = plant.get("linear")
    angular = plant.get("angular")
    jitter_limit = plant.get("command_transport_jitter_limit_sec")
    if (not _finite_number(rate) or float(rate) <= 0.0 or
            not _finite_number(fixed_tail) or float(fixed_tail) <= 0.0 or
            not isinstance(linear, dict) or
            not isinstance(angular, dict) or
            not _finite_number(linear.get("delay_sec")) or
            float(linear["delay_sec"]) < 0.0 or
            not _finite_number(angular.get("delay_sec")) or
            float(angular["delay_sec"]) < 0.0 or
            not _finite_number(jitter_limit) or
            float(jitter_limit) < 0.0):
        raise ValueError("invalid smoke timing configuration")
    rate_value = float(rate)
    zero_cycle = math.ceil(
        MOTION_COMMAND_END_SEC * rate_value - COUNT_EPSILON)
    zero_publish = zero_cycle / rate_value
    tail_start = (zero_publish +
                  max(float(linear["delay_sec"]),
                      float(angular["delay_sec"])) +
                  float(jitter_limit))
    end_sec = tail_start + float(fixed_tail)
    samples = _ceil_sample_count(end_sec, rate_value)
    first_tail_sample = max(
        1, _ceil_sample_count(tail_start, rate_value))
    return {
        "motion_command_end_sec": MOTION_COMMAND_END_SEC,
        "zero_command_publish_sec": zero_publish,
        "tail_window_start_sec": tail_start,
        "tail_window_end_sec": end_sec,
        "end_sec": end_sec,
        "duration_sec": end_sec,
        "samples": samples,
        "tail_samples": samples - first_tail_sample + 1,
    }


def require_simulation_audit_config(config: Dict[str, Any]) -> None:
    if config.get("schema") != "spmpc_independent_simulation_config_v1":
        raise ValueError("unsupported simulation configuration schema")
    if not config.get("freeze_id"):
        raise ValueError("missing simulation freeze_id")
    if config.get("status") not in (DEVELOPMENT_STATUS, FORMAL_STATUS):
        raise ValueError("unsupported simulation release status")
    scope = config.get("scope")
    if not isinstance(scope, dict):
        raise ValueError("missing simulation scope")
    if (scope.get("simulation_only") is not True or
            scope.get("formal_robot_release") is not False or
            scope.get("real_robot_enforce_allowed") is not False):
        raise ValueError("configuration attempts to authorize physical use")
    experiment = config.get("experiment")
    if not isinstance(experiment, dict):
        raise ValueError("missing experiment seed allocation")
    smoke_seed = experiment.get("smoke_seed")
    pilot_seeds = experiment.get("pilot_seeds")
    reserved = experiment.get("reserved_formal_seeds")
    if not _uint32_seed(smoke_seed):
        raise ValueError("invalid smoke seed")
    if (not isinstance(pilot_seeds, list) or not pilot_seeds or
            any(not _uint32_seed(seed) for seed in pilot_seeds)):
        raise ValueError("invalid pilot seed allocation")
    if len(pilot_seeds) != len(set(pilot_seeds)):
        raise ValueError("duplicate pilot seed")
    control_rate_hz = experiment.get("control_rate_hz")
    fixed_tail_sec = experiment.get("fixed_tail_sec")
    if (type(control_rate_hz) not in (int, float) or
            not math.isfinite(float(control_rate_hz)) or
            float(control_rate_hz) <= 0.0 or
            type(fixed_tail_sec) not in (int, float) or
            not math.isfinite(float(fixed_tail_sec)) or
            float(fixed_tail_sec) <= 0.0):
        raise ValueError("invalid experiment timing")
    if (not isinstance(reserved, list) or not reserved or
            any(not _uint32_seed(seed) for seed in reserved) or
            len(reserved) != len(set(reserved))):
        raise ValueError("invalid formal seed allocation")
    if experiment.get("formal_seeds_locked") is not True:
        raise ValueError("formal seed allocation is not locked")
    all_seeds = [smoke_seed] + pilot_seeds + reserved
    if len(all_seeds) != len(set(all_seeds)):
        raise ValueError("smoke, pilot, and formal seeds overlap")
    provenance = config.get("provenance")
    if (not isinstance(provenance, dict) or
            type(provenance.get("runtime_verified")) is not bool):
        raise ValueError("simulation provenance verification flag is invalid")
    if ("controller_internal_model" in config or
            "real_robot_launch_overrides" in experiment):
        raise ValueError("development config contains an unconsumed runtime section")
    expected_smoke_timing(config)


def require_simulation_development_config(config: Dict[str, Any]) -> None:
    require_simulation_audit_config(config)
    if config.get("status") != DEVELOPMENT_STATUS:
        raise ValueError("configuration is not the unbound development identity")
    if config["provenance"].get("runtime_verified") is not False:
        raise ValueError("development provenance must remain runtime-unverified")


def ensure_output_scope(output_dir: Path) -> Path:
    resolved = output_dir.resolve()
    allowed = ALLOWED_OUTPUT_ROOT.resolve()
    if os.path.commonpath((str(resolved), str(allowed))) != str(allowed):
        raise ValueError(
            "campaign output must remain under {}".format(allowed))
    return resolved


def formal_no_go_reasons(config: Dict[str, Any],
                         planner: Dict[str, Any]) -> List[str]:
    reasons: List[str] = []
    if config.get("status") != FORMAL_STATUS:
        reasons.append("simulation config is a development candidate")
    provenance = config.get("provenance")
    if (not isinstance(provenance, dict) or
            provenance.get("runtime_verified") is not True):
        reasons.append("simulation provenance is not runtime-verified")
    formal = config.get("formal_simulation")
    if not isinstance(formal, dict):
        reasons.append("no bound formal simulation session/assets")
    conditions = formal.get("conditions") if isinstance(formal, dict) else None
    required = {"C0", "C1", "C2", "C3", "C4", "IS"}
    if (not isinstance(conditions, dict) or
            not required.issubset(conditions) or
            any(not isinstance(conditions[name], dict) or
                not conditions[name] for name in required)):
        reasons.append("C0-C4/IS condition bindings are absent")
    delay = planner.get("delay_augmented_phase")
    if (not isinstance(delay, dict) or
            not _lowercase_sha256(
                delay.get("expected_recovery_artifact_hash"))):
        reasons.append("frozen recovery artifact hash is absent")
    phase = planner.get("phase_rejoin")
    if not isinstance(phase, dict) or not phase.get("artifact_path"):
        reasons.append("formal nominal/tail artifact path is absent")
    if (not isinstance(formal, dict) or
            not formal.get("held_out_execution_compatibility_report") or
            not _lowercase_sha256(
                formal.get(
                    "held_out_execution_compatibility_report_sha256"))):
        reasons.append("held-out execution compatibility/gate report is absent")
    return reasons


def validate_trial_csv(path: Path, summary: Dict[str, Any],
                       config: Dict[str, Any], seed: int) -> None:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != CSV_COLUMNS:
            raise RuntimeError(
                "seed {} emitted an invalid CSV timestamp contract".format(
                    seed))
        rows = list(reader)
    if len(rows) != summary.get("samples"):
        raise RuntimeError(
            "seed {} emitted an inconsistent CSV sample count".format(seed))
    plant = config["external_plant"]
    jitter_limit = float(plant["command_transport_jitter_limit_sec"])
    previous_publish = -math.inf
    previous_sample = -math.inf
    for row_index, row in enumerate(rows):
        try:
            values = {name: float(row[name]) for name in CSV_COLUMNS}
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(
                "seed {} CSV row {} is not numeric".format(
                    seed, row_index)) from error
        if any(not math.isfinite(value) for value in values.values()):
            raise RuntimeError(
                "seed {} CSV row {} is non-finite".format(seed, row_index))
        publish_time = values["publish_time_sec"]
        sample_time = values["sample_time_sec"]
        if (publish_time <= previous_publish or
                sample_time <= previous_sample or
                publish_time > sample_time + COUNT_EPSILON):
            raise RuntimeError(
                "seed {} CSV row {} has invalid event time order".format(
                    seed, row_index))
        for channel in ("linear", "angular"):
            jitter = values[channel + "_transport_jitter_sec"]
            effective = values[channel + "_effective_time_sec"]
            expected = (publish_time + float(plant[channel]["delay_sec"]) +
                        jitter)
            if (abs(jitter) > jitter_limit + COUNT_EPSILON or
                    not math.isclose(
                        effective, expected, rel_tol=0.0, abs_tol=1.0e-12)):
                raise RuntimeError(
                    "seed {} CSV row {} has invalid {} event epoch".format(
                        seed, row_index, channel))
        previous_publish = publish_time
        previous_sample = sample_time


def metric_summary(values: Iterable[float]) -> Dict[str, float]:
    sequence = list(values)
    return {
        "min": min(sequence),
        "median": statistics.median(sequence),
        "max": max(sequence),
    }


def run_trial(executable: Path, config_path: Path, output_dir: Path,
              config: Dict[str, Any], freeze_id: str,
              seed: int) -> Dict[str, Any]:
    prefix = output_dir / "seed{}".format(seed)
    completed = subprocess.run(
        [str(executable), str(config_path), str(prefix), str(seed)],
        check=False, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)
    if completed.returncode != 0:
        raise RuntimeError(
            "seed {} failed with {}: {}".format(
                seed, completed.returncode, completed.stderr.strip()))
    summary_path = prefix.with_suffix(".json")
    csv_path = prefix.with_suffix(".csv")
    with summary_path.open("r", encoding="utf-8") as stream:
        summary = json.load(stream)
    if (summary.get("schema") != SMOKE_SCHEMA or
            summary.get("csv_timestamp_contract") !=
                "publish_sample_effective_epochs_v1" or
            summary.get("freeze_id") != freeze_id or
            type(summary.get("seed")) is not int or
            summary.get("seed") != seed or
            summary.get("simulation_only") is not True or
            summary.get("formal_method_comparison") is not False):
        raise RuntimeError("seed {} emitted an invalid scope contract".format(seed))
    for metric in METRICS:
        value = summary.get(metric)
        if (type(value) not in (int, float) or
                not math.isfinite(float(value))):
            raise RuntimeError(
                "seed {} is missing metric {}".format(seed, metric))
    timing = expected_smoke_timing(config)
    for field in ("motion_command_end_sec", "zero_command_publish_sec",
                  "tail_window_start_sec", "tail_window_end_sec",
                  "end_sec", "duration_sec"):
        if (not _finite_number(summary.get(field)) or
                not math.isclose(float(summary[field]),
                                 float(timing[field]), abs_tol=1.0e-9)):
            raise RuntimeError(
                "seed {} emitted an inconsistent {}".format(seed, field))
    experiment = config["experiment"]
    if (type(summary.get("samples")) is not int or
            summary["samples"] != timing["samples"] or
            type(summary.get("tail_samples")) is not int or
            summary["tail_samples"] != timing["tail_samples"] or
            not math.isclose(float(summary.get("control_rate_hz", -1.0)),
                             float(experiment["control_rate_hz"]),
                             abs_tol=1.0e-12) or
            not math.isclose(
                float(summary.get("tail_window_end_sec", -1.0)) -
                float(summary.get("tail_window_start_sec", -1.0)),
                float(experiment["fixed_tail_sec"]), abs_tol=1.0e-9)):
        raise RuntimeError(
            "seed {} emitted an inconsistent time window".format(seed))
    validate_trial_csv(csv_path, summary, config, seed)
    return {
        "seed": seed,
        "stdout": completed.stdout.strip(),
        "csv": str(csv_path),
        "csv_sha256": sha256_file(csv_path),
        "summary": str(summary_path),
        "summary_sha256": sha256_file(summary_path),
        "metrics": {name: summary[name] for name in METRICS},
    }


def write_json(path: Path, value: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign", choices=("smoke", "pilot", "formal"),
                        required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--planner-config", type=Path)
    parser.add_argument("--executable", type=Path)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args(argv)

    try:
        output_dir = ensure_output_scope(args.out_dir)
        config_path = args.config.resolve()
        config = load_yaml(config_path)
        if args.campaign == "formal":
            require_simulation_audit_config(config)
        else:
            require_simulation_development_config(config)
        config_sha256 = sha256_file(config_path)
        planner_path = None
        planner = None
        planner_sha256 = None
        executable = None
        executable_sha256 = None
        if args.campaign == "formal":
            if args.planner_config is None:
                raise ValueError("formal audit requires --planner-config")
            planner_path = args.planner_config.resolve()
            planner = load_yaml(planner_path)
            planner_sha256 = sha256_file(planner_path)
        else:
            if args.executable is None:
                raise ValueError("smoke/pilot campaign requires --executable")
            executable = args.executable.resolve()
            if (not executable.is_file() or
                    not os.access(str(executable), os.X_OK)):
                raise ValueError("independent plant executable is not runnable")
            executable_sha256 = sha256_file(executable)

        # All argument and artifact preflight happens before this create-new
        # boundary.  A failed preflight therefore does not consume a campaign
        # directory; a failure after this point intentionally leaves evidence.
        if output_dir.exists():
            raise ValueError("campaign output directory already exists")
        output_dir.mkdir(parents=True, exist_ok=False)
        if args.campaign == "formal":
            assert planner_path is not None
            assert planner is not None
            assert planner_sha256 is not None
            reasons = formal_no_go_reasons(config, planner)
            report = {
                "schema": "spmpc_formal_simulation_readiness_v1",
                "freeze_id": config["freeze_id"],
                "status": "NO_GO" if reasons else "READY_NOT_EXECUTED",
                "formal_trials_started": False,
                "config": str(config_path),
                "config_sha256": config_sha256,
                "planner_config": str(planner_path),
                "planner_config_sha256": planner_sha256,
                "reasons": reasons,
            }
            write_json(output_dir / "formal_readiness.json", report)
            if reasons:
                print("formal simulation NO-GO: " + "; ".join(reasons))
                return 4
            print("formal bindings appear ready; this development runner still "
                  "does not execute C0-C4")
            return 5

        assert executable is not None
        assert executable_sha256 is not None
        experiment = config["experiment"]
        seeds = ([int(experiment["smoke_seed"])]
                 if args.campaign == "smoke"
                 else [int(seed) for seed in experiment["pilot_seeds"]])
        trials = [run_trial(executable, config_path, output_dir, config,
                            str(config["freeze_id"]), seed)
                  for seed in seeds]
        if (sha256_file(config_path) != config_sha256 or
                sha256_file(executable) != executable_sha256):
            raise RuntimeError("campaign input changed while trials were running")
        aggregate = {
            metric: metric_summary(
                float(trial["metrics"][metric]) for trial in trials)
            for metric in METRICS
        }
        manifest = {
            "schema": "spmpc_independent_plant_campaign_v3",
            "freeze_id": config["freeze_id"],
            "campaign": args.campaign,
            "status": "COMPLETE_DEVELOPMENT_{}".format(
                args.campaign.upper()),
            "simulation_only": True,
            "formal_method_comparison": False,
            "completion_scope": "execution_and_artifact_integrity_only",
            "effect_claim": False,
            "config": str(config_path),
            "config_sha256": config_sha256,
            "executable": str(executable),
            "executable_sha256": executable_sha256,
            "seeds": seeds,
            "aggregate": aggregate,
            "trials": trials,
        }
        write_json(output_dir / "campaign_manifest.json", manifest)
        print("{}: {} trials, q95 median={:.9f} m".format(
            manifest["status"], len(trials),
            aggregate["external_height_q95_m"]["median"]))
        return 0
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as error:
        print("campaign rejected: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
