#!/usr/bin/env python3
"""Run plant smoke/pilot campaigns or audit a frozen simulation session.

The plant configuration and the experiment session are deliberately separate.
The former describes simulation truth; the latter binds the controller,
runner, path, nominal/recovery evidence, conditions, seeds and metrics.  A
formal request is an auditable readiness check only: it never starts C0--C4.
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
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import yaml


ALLOWED_OUTPUT_ROOT = Path("/data/a/spmpc_exec_identification")
DEVELOPMENT_STATUS = "development_candidate_unbound"
PILOT_STATUS = "pilot_candidate_bound"
MOTION_COMMAND_END_SEC = 7.5
COUNT_EPSILON = 1.0e-12
MAX_UINT32 = (1 << 32) - 1
FORMAL_STATUS = "formal_simulation_release"
FORMAL_SESSION_SCHEMA = "spmpc_formal_simulation_session_v1"
CONTROLLER_MANIFEST_SCHEMA = "spmpc_controller_runtime_manifest_v1"
NOMINAL_REPORT_SCHEMA = "spmpc_nominal_validation_report_v1"
RECOVERY_DATASET_SCHEMA = "spmpc_phase_rejoin_recovery_dataset_v1"
RECOVERY_MANIFEST_SCHEMA = "spmpc_phase_rejoin_recovery_fit_manifest_v1"
RECOVERY_REPORT_SCHEMA = "spmpc_phase_rejoin_recovery_held_out_report_v1"
RECOVERY_RADII_SCHEMA = "spmpc_phase_rejoin_recovery_scales_v1"
SMOKE_SCHEMA = "spmpc_independent_plant_smoke_v3"
REQUIRED_CONDITIONS = ("C0", "C1", "C2", "C3", "C4", "IS")
REQUIRED_SESSION_ASSETS = (
    "plant_config",
    "planner_config",
    "path",
    "offline_plan",
    "offline_plan_report",
    "offline_plan_generator",
    "nominal_builder",
    "phase_rejoin_artifact",
    "nominal_validation_report",
    "recovery_dataset",
    "recovery_fit_manifest",
    "recovery_radii_bounds",
    "recovery_held_out_report",
    "analysis_script",
    "formal_order",
)
CONDITION_SEMANTICS = {
    "C0": {
        "mode": "ordinary_mpcc",
        "offline_nominal": False,
        "online_residual": False,
        "recovery_gate": False,
        "execution_compatibility_gate": False,
        "stored_recovery_action": False,
        "input_shaping": False,
    },
    "C1": {
        "mode": "smooth_match_mpcc",
        "offline_nominal": False,
        "online_residual": False,
        "recovery_gate": False,
        "execution_compatibility_gate": False,
        "stored_recovery_action": False,
        "input_shaping": False,
    },
    "C2": {
        "mode": "offline_replay",
        "offline_nominal": True,
        "online_residual": False,
        "recovery_gate": False,
        "execution_compatibility_gate": False,
        "stored_recovery_action": False,
        "input_shaping": False,
    },
    "C3": {
        "mode": "residual_no_gate",
        "offline_nominal": True,
        "online_residual": True,
        "recovery_gate": False,
        "execution_compatibility_gate": True,
        "stored_recovery_action": True,
        "input_shaping": False,
    },
    "C4": {
        "mode": "phase_rejoin_full",
        "offline_nominal": True,
        "online_residual": True,
        "recovery_gate": True,
        "execution_compatibility_gate": True,
        "stored_recovery_action": True,
        "input_shaping": False,
    },
    "IS": {
        "mode": "input_shaping",
        "offline_nominal": False,
        "online_residual": False,
        "recovery_gate": False,
        "execution_compatibility_gate": False,
        "stored_recovery_action": False,
        "input_shaping": True,
    },
}
RECOVERY_STATE_ERRORS = (
    "x", "y", "yaw", "v", "omega", "eta_x", "eta_x_dot", "eta_y",
    "eta_y_dot",
)
RECOVERY_EXECUTION_ERRORS = (
    "linear_output", "angular_output",
    "linear_pending_0", "linear_pending_1", "linear_pending_2",
    "linear_pending_3", "linear_pending_4",
    "angular_pending_0", "angular_pending_1", "angular_pending_2",
    "angular_pending_3", "angular_pending_4", "angular_pending_5",
    "angular_pending_6",
)
RECOVERY_DATASET_COLUMNS = (
    "split", "rollout_id", "seed", "phase_index", "recovered",
) + RECOVERY_STATE_ERRORS + RECOVERY_EXECUTION_ERRORS
RECOVERY_RADII_COLUMNS = (
    "phase_index", "phase_bin_start", "phase_bin_end", "shrinkage",
) + tuple("r_" + name for name in RECOVERY_STATE_ERRORS) + tuple(
    "beta_" + name for name in RECOVERY_EXECUTION_ERRORS)
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


def load_structured_file(path: Path) -> Dict[str, Any]:
    """Load a JSON/YAML evidence file without guessing arbitrary formats."""
    suffix = path.suffix.lower()
    if suffix == ".json":
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    elif suffix in (".yaml", ".yml"):
        value = load_yaml(path)
    else:
        raise ValueError("unsupported structured evidence format: {}".format(
            path))
    if not isinstance(value, dict):
        raise ValueError("evidence root must be a mapping: {}".format(path))
    return value


def _nonempty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _git_commit(value: Any) -> bool:
    return (isinstance(value, str) and len(value) == 40 and
            all(character in "0123456789abcdef" for character in value))


def _resolve_session_path(session_path: Path, value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = session_path.parent / path
    return path.resolve()


def _artifact_reference(
        owner: Dict[str, Any], key: str, session_path: Path,
        reasons: List[str], inventory: Dict[str, Dict[str, Any]],
        executable: bool = False,
        inventory_key: Optional[str] = None) -> Optional[Path]:
    """Resolve and hash-check one {path, sha256} reference.

    Missing files and fake hash-looking strings are both reported as NO-GO
    reasons.  A syntactically valid SHA-256 is never treated as evidence by
    itself.
    """
    reference = owner.get(key)
    label = key.replace("_", " ")
    if not isinstance(reference, dict):
        reasons.append("{} reference is absent".format(label))
        return None
    raw_path = reference.get("path")
    expected_hash = reference.get("sha256")
    if not _nonempty_string(raw_path):
        reasons.append("{} path is absent".format(label))
        return None
    if not _lowercase_sha256(expected_hash):
        reasons.append("{} SHA-256 is absent or malformed".format(label))
        return None
    path = _resolve_session_path(session_path, raw_path)
    record = {
        "path": str(path),
        "expected_sha256": expected_hash,
        "exists": path.is_file(),
    }
    inventory[inventory_key or key] = record
    if not path.is_file():
        reasons.append("{} file does not exist".format(label))
        return None
    if executable and not os.access(str(path), os.X_OK):
        reasons.append("{} is not executable".format(label))
        return None
    actual_hash = sha256_file(path)
    record["actual_sha256"] = actual_hash
    record["hash_matches"] = actual_hash == expected_hash
    if actual_hash != expected_hash:
        reasons.append("{} SHA-256 mismatch".format(label))
        return None
    return path


def _csv_metadata(path: Path) -> Dict[str, str]:
    metadata: Dict[str, str] = {}
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.startswith("#"):
                break
            payload = line[1:].strip()
            if "=" in payload:
                key, value = payload.split("=", 1)
                metadata[key.strip()] = value.strip()
    return metadata


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
    if config.get("status") not in (
            DEVELOPMENT_STATUS, PILOT_STATUS, FORMAL_STATUS):
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


def require_formal_session_schema(session: Dict[str, Any]) -> None:
    if session.get("schema") != FORMAL_SESSION_SCHEMA:
        raise ValueError("unsupported formal simulation session schema")
    if not _nonempty_string(session.get("freeze_id")):
        raise ValueError("missing formal simulation freeze_id")
    if session.get("status") not in (PILOT_STATUS, FORMAL_STATUS):
        raise ValueError("unsupported formal simulation session status")
    scope = session.get("scope")
    if (not isinstance(scope, dict) or
            scope.get("simulation_only") is not True or
            scope.get("formal_robot_release") is not False or
            scope.get("real_robot_enforce_allowed") is not False or
            scope.get("plant_truth_visible_to_controller") is not False or
            scope.get("physical_parameter_claim") is not False):
        raise ValueError("formal session violates simulation-only scope")


def _validate_seed_contract(session: Dict[str, Any],
                            reasons: List[str]) -> Dict[str, List[int]]:
    groups = (
        "recovery_fit", "recovery_tune", "recovery_held_out",
        "pilot_trials", "formal_trials",
    )
    seed_contract = session.get("seeds")
    result: Dict[str, List[int]] = {}
    if not isinstance(seed_contract, dict):
        reasons.append("seed partition contract is absent")
        return result
    if seed_contract.get("locked") is not True:
        reasons.append("seed partitions are not locked")
    used: Dict[int, str] = {}
    for group in groups:
        values = seed_contract.get(group)
        if (not isinstance(values, list) or not values or
                any(not _uint32_seed(value) for value in values) or
                len(values) != len(set(values))):
            reasons.append("{} seed partition is invalid".format(group))
            continue
        result[group] = list(values)
        for seed in values:
            if seed in used:
                reasons.append(
                    "seed {} overlaps {} and {}".format(
                        seed, used[seed], group))
            else:
                used[seed] = group
    return result


def _validate_measurement_contract(session: Dict[str, Any],
                                   reasons: List[str]) -> None:
    measurement = session.get("measurement_contract")
    expected = {
        "primary_metric": "external_height_q95_m",
        "window": "motion_plus_fixed_tail",
        "statistics_unit": "complete_trial",
        "physical_metric_alignment": "rgb_max_lcr_motion_plus_fixed_tail",
        "controller_liquid_is_truth": False,
        "external_liquid_truth_used_for_control": False,
        "failures_included": True,
    }
    if not isinstance(measurement, dict):
        reasons.append("motion+tail measurement contract is absent")
        return
    for key, value in expected.items():
        if measurement.get(key) != value:
            reasons.append("measurement contract {} is invalid".format(key))
    tail = measurement.get("fixed_tail_sec")
    if not _finite_number(tail) or float(tail) <= 0.0:
        reasons.append("measurement fixed tail is invalid")
    numeric_contract = {
        "minimum_meaningful_difference_m": 0.0005,
        "formal_paired_blocks": 16,
        "completion_time_noninferiority_relative": 0.10,
        "tracking_q95_noninferiority_m": 0.05,
    }
    for key, expected_value in numeric_contract.items():
        actual = measurement.get(key)
        if (not _finite_number(actual) or
                not math.isclose(float(actual), expected_value,
                                 rel_tol=0.0, abs_tol=1.0e-12)):
            reasons.append("measurement contract {} is invalid".format(key))
    string_contract = {
        "paired_interval": "paired_bootstrap_95pct",
        "failed_trial_rule": "retain_and_count_as_failure",
        "replacement_rule":
            "infrastructure_failure_only_same_seed_condition",
    }
    for key, expected_value in string_contract.items():
        if measurement.get(key) != expected_value:
            reasons.append("measurement contract {} is invalid".format(key))


def _validate_formal_order(path: Optional[Path], formal_seeds: List[int],
                           reasons: List[str]) -> None:
    if path is None:
        return
    expected_conditions = set(REQUIRED_CONDITIONS)
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            rows = list(reader)
            columns = tuple(reader.fieldnames or ())
    except OSError as error:
        reasons.append("formal order cannot be read: {}".format(error))
        return
    if columns != ("schema", "block", "seed", "position", "condition"):
        reasons.append("formal order columns are invalid")
        return
    by_seed: Dict[int, List[Tuple[int, str]]] = {}
    for row in rows:
        try:
            block = int(row["block"])
            seed = int(row["seed"])
            position = int(row["position"])
            condition = row["condition"]
        except (KeyError, TypeError, ValueError):
            reasons.append("formal order row is invalid")
            return
        if (row.get("schema") != "spmpc_phase_rejoin_formal_order_v1" or
                block < 1 or position < 1 or position > 6 or
                condition not in expected_conditions):
            reasons.append("formal order row contract is invalid")
            return
        by_seed.setdefault(seed, []).append((position, condition))
    if list(by_seed) != formal_seeds:
        reasons.append("formal order seeds differ from frozen session")
        return
    position_counts = {
        condition: {position: 0 for position in range(1, 7)}
        for condition in REQUIRED_CONDITIONS
    }
    for block, seed in enumerate(formal_seeds, start=1):
        entries = by_seed.get(seed, [])
        if (len(entries) != 6 or
                {position for position, _ in entries} != set(range(1, 7)) or
                {condition for _, condition in entries} !=
                    expected_conditions):
            reasons.append("formal order block {} is incomplete".format(block))
            return
        for position, condition in entries:
            position_counts[condition][position] += 1
    for condition, counts in position_counts.items():
        values = list(counts.values())
        if max(values) - min(values) > 1:
            reasons.append(
                "formal order position balance is invalid for {}".format(
                    condition))


def _validate_runtime_contract(
        session: Dict[str, Any], session_path: Path, reasons: List[str],
        inventory: Dict[str, Dict[str, Any]]) -> Dict[str, Optional[Path]]:
    runtime = session.get("runtime")
    resolved: Dict[str, Optional[Path]] = {
        "runner": None,
        "executable": None,
        "controller_manifest": None,
        "artifact_validator": None,
    }
    if not isinstance(runtime, dict):
        reasons.append("runtime provenance is absent")
        return resolved
    expected = {
        "control_rate_hz": 30.0,
        "final_command_transaction": True,
        "command_history_source": "final_published_command",
        "observer_input": "plant_motion_derived_odom",
        "external_liquid_truth_used_for_control": False,
        "dual_channel_execution_model": True,
        "expected_publish_time_model": True,
    }
    for key, value in expected.items():
        actual = runtime.get(key)
        if key == "control_rate_hz":
            if (not _finite_number(actual) or
                    not math.isclose(float(actual), value, abs_tol=1.0e-12)):
                reasons.append("runtime control rate must be 30 Hz")
        elif actual != value:
            reasons.append("runtime contract {} is invalid".format(key))
    if (not _git_commit(runtime.get("git_commit")) or
            runtime.get("source_tree_clean") is not True or
            not _nonempty_string(runtime.get("compiler_id")) or
            not _nonempty_string(runtime.get("stl_id")) or
            not _nonempty_string(runtime.get("floating_point_contract"))):
        reasons.append("runtime build provenance is incomplete")
    for key in resolved:
        resolved[key] = _artifact_reference(
            runtime, key, session_path, reasons, inventory,
            executable=(key in ("runner", "executable", "artifact_validator")),
            inventory_key="runtime." + key)
    return resolved


def _validate_condition_bindings(
        session: Dict[str, Any], session_path: Path, reasons: List[str],
        inventory: Dict[str, Dict[str, Any]],
        controller_manifest: Optional[Dict[str, Any]]) -> None:
    conditions = session.get("conditions")
    if not isinstance(conditions, dict) or set(conditions) != set(
            REQUIRED_CONDITIONS):
        reasons.append("C0-C4/IS condition set is not exact")
        return
    manifest_conditions = (
        controller_manifest.get("conditions")
        if isinstance(controller_manifest, dict) else None)
    if not isinstance(manifest_conditions, dict):
        reasons.append("controller manifest condition table is absent")
        manifest_conditions = {}
    implementation_ids = set()
    for name in REQUIRED_CONDITIONS:
        binding = conditions.get(name)
        if not isinstance(binding, dict):
            reasons.append("{} condition binding is absent".format(name))
            continue
        if not _nonempty_string(binding.get("binding_id")):
            reasons.append("{} binding_id is absent".format(name))
        implementation_id = binding.get("implementation_id")
        if not _nonempty_string(implementation_id):
            reasons.append("{} implementation_id is absent".format(name))
        elif implementation_id in implementation_ids:
            reasons.append("condition implementation_id is not unique")
        else:
            implementation_ids.add(implementation_id)
        _artifact_reference(
            binding, "config", session_path, reasons, inventory,
            inventory_key="condition.{}.config".format(name))
        expected = CONDITION_SEMANTICS[name]
        for key, value in expected.items():
            if binding.get(key) != value:
                reasons.append("{} semantic {} is invalid".format(name, key))
        if name == "C1" and (
                binding.get("pilot_tuned_and_frozen") is not True or
                not _finite_number(binding.get("global_time_scale")) or
                float(binding.get("global_time_scale", 0.0)) <= 0.0):
            reasons.append("C1 smooth-match pilot freeze is absent")
        if name == "IS" and (
                binding.get("shaper") not in ("ZV", "ZVD") or
                binding.get("single_mode_residual_test_passed") is not True):
            reasons.append("IS shaper validation is absent")
        manifest_binding = manifest_conditions.get(name)
        if not isinstance(manifest_binding, dict):
            reasons.append("controller manifest does not implement {}".format(
                name))
            continue
        if manifest_binding.get("implementation_id") != implementation_id:
            reasons.append("{} implementation_id differs from manifest".format(
                name))
        for key, value in expected.items():
            if manifest_binding.get(key) != value:
                reasons.append(
                    "{} manifest semantic {} is invalid".format(name, key))


def _validate_controller_manifest(
        path: Optional[Path], runtime_paths: Dict[str, Optional[Path]],
        runtime: Any, reasons: List[str]) -> Optional[Dict[str, Any]]:
    if path is None:
        return None
    try:
        manifest = load_structured_file(path)
    except (OSError, ValueError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        reasons.append("controller manifest cannot be parsed: {}".format(error))
        return None
    if manifest.get("schema") != CONTROLLER_MANIFEST_SCHEMA:
        reasons.append("controller manifest schema is invalid")
    if (manifest.get("git_commit") != runtime.get("git_commit") or
            manifest.get("source_tree_clean") is not True):
        reasons.append("controller manifest git provenance is inconsistent")
    executable_path = runtime_paths.get("executable")
    runner_path = runtime_paths.get("runner")
    artifact_validator_path = runtime_paths.get("artifact_validator")
    if (executable_path is not None and
            manifest.get("executable_sha256") != sha256_file(executable_path)):
        reasons.append("controller manifest executable hash is inconsistent")
    if (runner_path is not None and
            manifest.get("runner_sha256") != sha256_file(runner_path)):
        reasons.append("controller manifest runner hash is inconsistent")
    if (artifact_validator_path is not None and
            manifest.get("artifact_validator_sha256") !=
            sha256_file(artifact_validator_path)):
        reasons.append("controller manifest artifact validator is inconsistent")
    solver = manifest.get("solver")
    if (not isinstance(solver, dict) or
            solver.get("backend") != "delay_augmented_phase_acados" or
            solver.get("state_width") != 22 or
            solver.get("control_width") != 3 or
            solver.get("horizon_steps") != 10 or
            not _lowercase_sha256(solver.get("execution_contract_hash")) or
            not _lowercase_sha256(solver.get("parameter_schema_hash"))):
        reasons.append("controller manifest solver contract is invalid")
    return manifest


def _validate_path_contract(path: Optional[Path], session: Dict[str, Any],
                            reasons: List[str]) -> Optional[Dict[str, Any]]:
    if path is None:
        return None
    try:
        path_data = load_structured_file(path)
    except (OSError, ValueError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        reasons.append("path definition cannot be parsed: {}".format(error))
        return None
    contract = session.get("path_contract")
    goal = path_data.get("goal")
    goal_frame = None
    goal_yaw = None
    has_geometry = False
    if isinstance(goal, dict):
        if (_nonempty_string(goal.get("frame_id")) and
                all(_finite_number(goal.get(key))
                    for key in ("x", "y", "yaw"))):
            goal_frame = goal["frame_id"]
            goal_yaw = float(goal["yaw"])
            has_geometry = (
                isinstance(path_data.get("path_template"), dict) or
                (isinstance(path_data.get("waypoints"), list) and
                 len(path_data["waypoints"]) >= 2))
    else:
        poses = path_data.get("poses")
        frame_id = path_data.get("frame_id")
        if (isinstance(poses, list) and len(poses) >= 2 and
                _nonempty_string(frame_id) and
                isinstance(poses[-1], dict)):
            pose = poses[-1]
            values = [pose.get(key, default) for key, default in (
                ("qx", 0.0), ("qy", 0.0), ("qz", 0.0), ("qw", 1.0),
            )]
            if (all(_finite_number(value) for value in values) and
                    all(_finite_number(pose.get(key)) for key in ("x", "y"))):
                qx, qy, qz, qw = (float(value) for value in values)
                norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
                if norm > 1.0e-12:
                    qx, qy, qz, qw = (
                        value / norm for value in (qx, qy, qz, qw))
                    goal_yaw = math.atan2(
                        2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz))
                    goal_frame = frame_id
                    has_geometry = True
    if (not isinstance(contract, dict) or goal_frame is None or
            goal_yaw is None):
        reasons.append("path and goal-yaw contract is incomplete")
        return path_data
    if (contract.get("frame_id") != goal_frame or
            not _finite_number(contract.get("goal_yaw_rad")) or
            not math.isclose(float(contract["goal_yaw_rad"]),
                             goal_yaw, abs_tol=1.0e-12)):
        reasons.append("path frame/goal yaw differs from frozen session")
    if not has_geometry:
        reasons.append("path definition contains no template or waypoints")
    return path_data


def _validate_phase_rejoin_artifact(
        path: Optional[Path], validator_path: Optional[Path],
        reasons: List[str]) -> Dict[str, str]:
    if path is None or validator_path is None:
        return {}
    if validator_path.name != "spmpc_phase_rejoin_artifact_tool":
        reasons.append("production phase-rejoin artifact validator is not bound")
    try:
        completed = subprocess.run(
            [str(validator_path), "validate", "--artifact", str(path)],
            check=False, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=30.0)
    except (OSError, subprocess.TimeoutExpired) as error:
        reasons.append("phase-rejoin artifact validator failed: {}".format(error))
        completed = None
    if (completed is None or completed.returncode != 0 or
            not completed.stdout.startswith(
                "VALID artifact: schema=phase_rejoin_empirical_augmented_v3 ") or
            "evidence_level=empirical_held_out" not in completed.stdout):
        detail = "" if completed is None else completed.stderr.strip()
        reasons.append("production V3 artifact validation failed{}".format(
            ": " + detail if detail else ""))
    metadata = _csv_metadata(path)
    required = {
        "schema": "phase_rejoin_empirical_augmented_v3",
        "evidence_level": "empirical_held_out",
        "source": "simulation_offline_slosh_ocp_held_out_recovery",
        "artifact_role": "simulation_phase_rejoin_nominal_and_recovery",
        "nominal_sequence_kind":
            "offline_slosh_ocp_complete_augmented_tail",
        "offline_slosh_ocp": "true",
        "hardware_formal_release": "false",
        "terminal_contract": "publish_zero_settle_hold_v2",
        "execution_compatibility_contract":
            "phase_indexed_execution_box_v1",
    }
    for key, value in required.items():
        if metadata.get(key) != value:
            reasons.append(
                "phase-rejoin artifact metadata {} is invalid".format(key))
    if not _lowercase_sha256(metadata.get("recovery_artifact_hash")):
        reasons.append("phase-rejoin artifact recovery payload hash is invalid")
    elif (completed is not None and completed.returncode == 0 and
          "recovery_artifact_hash={}".format(
              metadata["recovery_artifact_hash"]) not in completed.stdout):
        reasons.append("artifact validator recovery hash differs from metadata")
    terminal_thresholds = (
        "terminal_v_abs_max",
        "terminal_omega_abs_max",
        "terminal_linear_actuator_output_abs_max",
        "terminal_angular_actuator_output_abs_max",
        "terminal_linear_pending_command_abs_max",
        "terminal_angular_pending_command_abs_max",
    )
    for key in terminal_thresholds:
        value = metadata.get(key)
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            parsed = math.nan
        if not math.isfinite(parsed) or parsed < 0.0:
            reasons.append(
                "phase-rejoin artifact terminal threshold {} is invalid".format(
                    key))
    return metadata


def _validate_offline_plan_bundle(
        plan_path: Optional[Path], report_path: Optional[Path],
        path_path: Optional[Path], reasons: List[str]) -> None:
    if plan_path is None or report_path is None:
        return
    metadata = _csv_metadata(plan_path)
    expected = {
        "schema": "spmpc_offline_slosh_ocp_plan_v1",
        "status": "PASS",
        "simulation_only": "true",
        "formal_robot_release": "false",
    }
    for key, value in expected.items():
        if metadata.get(key) != value:
            reasons.append("OfflineSloshOCP plan metadata {} is invalid".format(
                key))
    if (path_path is not None and
            metadata.get("path_sha256") != sha256_file(path_path)):
        reasons.append("OfflineSloshOCP path hash is inconsistent")
    if not _lowercase_sha256(metadata.get("execution_contract_hash")):
        reasons.append("OfflineSloshOCP execution contract hash is invalid")
    try:
        dt = float(metadata.get("dt", "nan"))
        zero_hold_steps = int(metadata.get("zero_hold_steps", "0"))
    except ValueError:
        dt = math.nan
        zero_hold_steps = 0
    if not math.isfinite(dt) or dt <= 0.0 or zero_hold_steps < 5:
        reasons.append("OfflineSloshOCP timing/tail contract is invalid")
    try:
        with plan_path.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(
                line for line in stream if not line.startswith("#")))
    except OSError as error:
        reasons.append("OfflineSloshOCP plan cannot be read: {}".format(error))
        rows = []
    columns = ("index", "t", "u_pub_v", "u_pub_omega", "v_s")
    if (not rows or tuple(rows[0]) != columns or len(rows) < 20 or
            zero_hold_steps >= len(rows)):
        reasons.append("OfflineSloshOCP plan row contract is invalid")
    else:
        previous_t = -math.inf
        rows_valid = True
        for index, row in enumerate(rows):
            try:
                row_index = int(row["index"])
                row_t = float(row["t"])
                values = [float(row[key]) for key in columns[2:]]
            except (KeyError, TypeError, ValueError):
                rows_valid = False
                break
            if (row_index != index or not math.isfinite(row_t) or
                    row_t <= previous_t or
                    any(not math.isfinite(value) for value in values)):
                rows_valid = False
                break
            previous_t = row_t
        tail = rows[-zero_hold_steps:]
        if (not rows_valid or any(
                abs(float(row[key])) > 1.0e-12
                for row in tail for key in columns[2:])):
            reasons.append("OfflineSloshOCP zero-command tail is invalid")
    try:
        report = load_structured_file(report_path)
    except (OSError, ValueError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        reasons.append("OfflineSloshOCP report cannot be parsed: {}".format(
            error))
        return
    optimizer = report.get("optimizer")
    plan = report.get("plan")
    terminal = report.get("terminal_contract")
    if (report.get("schema") != "spmpc_offline_slosh_ocp_report_v1" or
            report.get("status") != "PASS" or
            report.get("simulation_only") is not True or
            report.get("formal_robot_release") is not False or
            not isinstance(optimizer, dict) or
            optimizer.get("success") is not True or
            not isinstance(plan, dict) or
            plan.get("sha256") != sha256_file(plan_path) or
            plan.get("rows") != len(rows) or
            not isinstance(terminal, dict) or
            terminal.get("name") != "publish_zero_settle_hold_v2" or
            terminal.get("zero_hold_steps") != zero_hold_steps or
            terminal.get("goal_yaw_preserved_from_path_quaternion") is not True):
        reasons.append("OfflineSloshOCP optimization report did not pass")


def _float_list(text: Any) -> Optional[List[float]]:
    if not isinstance(text, str):
        return None
    try:
        values = [float(item) for item in text.split(";")]
    except ValueError:
        return None
    if not values or any(not math.isfinite(value) for value in values):
        return None
    return values


def _validate_artifact_recovery_binding(
        artifact_path: Optional[Path], radii_path: Optional[Path],
        reasons: List[str]) -> None:
    """Verify the fitted 9D gate/B_exec table was copied into final V3 rows."""
    if artifact_path is None or radii_path is None:
        return
    try:
        with radii_path.open("r", encoding="utf-8", newline="") as stream:
            radii_rows = list(csv.DictReader(stream))
        with artifact_path.open("r", encoding="utf-8", newline="") as stream:
            artifact_rows = list(csv.DictReader(
                line for line in stream if not line.startswith("#")))
    except OSError as error:
        reasons.append("artifact/recovery binding cannot be read: {}".format(
            error))
        return
    bounds_by_phase: Dict[int, Dict[str, str]] = {}
    try:
        for row in radii_rows:
            phase = int(row["phase_index"])
            if phase in bounds_by_phase:
                raise ValueError("duplicate recovery phase")
            bounds_by_phase[phase] = row
    except (KeyError, TypeError, ValueError):
        reasons.append("recovery radii/bounds phase index is invalid")
        return
    seen = set()
    for row in artifact_rows:
        try:
            phase = int(row["index"])
        except (KeyError, TypeError, ValueError):
            reasons.append("V3 artifact phase index is invalid")
            return
        bounds = bounds_by_phase.get(phase)
        if bounds is None or phase in seen:
            reasons.append("V3 artifact and recovery phase coverage differ")
            return
        seen.add(phase)
        comparisons = []
        for name in RECOVERY_STATE_ERRORS:
            comparisons.append((row.get("r_" + name), bounds.get("r_" + name)))
        comparisons.extend((
            (row.get("exec_beta_linear_output"),
             bounds.get("beta_linear_output")),
            (row.get("exec_beta_angular_output"),
             bounds.get("beta_angular_output")),
        ))
        linear_pending = _float_list(row.get("exec_beta_linear_pending"))
        angular_pending = _float_list(row.get("exec_beta_angular_pending"))
        if (linear_pending is None or len(linear_pending) != 5 or
                angular_pending is None or len(angular_pending) != 7):
            reasons.append("V3 artifact execution-bound vectors are invalid")
            return
        comparisons.extend(
            (value, bounds.get("beta_linear_pending_{}".format(index)))
            for index, value in enumerate(linear_pending))
        comparisons.extend(
            (value, bounds.get("beta_angular_pending_{}".format(index)))
            for index, value in enumerate(angular_pending))
        try:
            matched = all(
                math.isclose(float(left), float(right), rel_tol=0.0,
                             abs_tol=1.0e-12)
                for left, right in comparisons)
        except (TypeError, ValueError):
            matched = False
        if not matched:
            reasons.append("V3 artifact gate/B_exec differs from fitted bounds")
            return
    if seen != set(bounds_by_phase):
        reasons.append("V3 artifact and recovery phase coverage differ")


def _validate_nominal_report(
        path: Optional[Path], phase_path: Optional[Path],
        path_path: Optional[Path], offline_plan_path: Optional[Path],
        offline_plan_report_path: Optional[Path],
        recovery_manifest_path: Optional[Path],
        recovery_held_out_path: Optional[Path],
        artifact_metadata: Dict[str, str], reasons: List[str]) -> None:
    if path is None:
        return
    try:
        report = load_structured_file(path)
    except (OSError, ValueError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        reasons.append("nominal validation report cannot be parsed: {}".format(
            error))
        return
    artifact = report.get("artifact")
    if (report.get("schema") != NOMINAL_REPORT_SCHEMA or
            report.get("status") != "PASS" or
            report.get("simulation_only") is not True or
            report.get("formal_robot_release") is not False or
            report.get("production_v3_loader_passed") is not True or
            report.get("terminal_contract") !=
            "publish_zero_settle_hold_v2" or
            not isinstance(artifact, dict)):
        reasons.append("nominal/tail validation report did not pass")
    if (phase_path is not None and
            (not isinstance(artifact, dict) or
             artifact.get("sha256") != sha256_file(phase_path) or
             artifact.get("recovery_artifact_hash") !=
             artifact_metadata.get("recovery_artifact_hash"))):
        reasons.append("nominal report artifact hash is inconsistent")
    if (path_path is not None and
            report.get("path_sha256") != sha256_file(path_path)):
        reasons.append("nominal report path hash is inconsistent")
    if (recovery_manifest_path is not None and
            report.get("recovery_fit_manifest_sha256") !=
            sha256_file(recovery_manifest_path)):
        reasons.append("nominal report recovery manifest hash is inconsistent")
    if (offline_plan_path is not None and
            report.get("offline_plan_sha256") != sha256_file(offline_plan_path)):
        reasons.append("nominal report OfflineSloshOCP hash is inconsistent")
    if (offline_plan_report_path is not None and
            report.get("offline_plan_report_sha256") !=
            sha256_file(offline_plan_report_path)):
        reasons.append("nominal report OfflineSloshOCP report is inconsistent")
    if (recovery_held_out_path is not None and
            report.get("recovery_held_out_report_sha256") !=
            sha256_file(recovery_held_out_path)):
        reasons.append("nominal report held-out recovery hash is inconsistent")
    try:
        zero_hold = int(artifact_metadata.get("terminal_zero_hold_steps", "0"))
    except ValueError:
        zero_hold = 0
    if (report.get("terminal_zero_hold_steps") != zero_hold or
            not isinstance(artifact, dict) or
            type(artifact.get("rows")) is not int or artifact["rows"] < 20):
        reasons.append("nominal report row/tail count is inconsistent")


def _validate_v3_source_bindings(
        metadata: Dict[str, str], asset_paths: Dict[str, Optional[Path]],
        reasons: List[str]) -> None:
    bindings = {
        "path_sha256": "path",
        "offline_plan_sha256": "offline_plan",
        "offline_plan_report_sha256": "offline_plan_report",
        "recovery_dataset_sha256": "recovery_dataset",
        "recovery_scales_sha256": "recovery_radii_bounds",
        "recovery_fit_manifest_sha256": "recovery_fit_manifest",
        "recovery_held_out_report_sha256": "recovery_held_out_report",
    }
    for metadata_key, asset_key in bindings.items():
        path = asset_paths.get(asset_key)
        if path is not None and metadata.get(metadata_key) != sha256_file(path):
            reasons.append("V3 source binding {} is inconsistent".format(
                metadata_key))


def _validate_recovery_dataset(
        path: Optional[Path], seed_groups: Dict[str, List[int]],
        reasons: List[str]) -> Dict[str, Any]:
    if path is None:
        return {}
    split_seeds = {split: set() for split in ("fit", "tune", "held_out")}
    split_rollouts = {split: set() for split in ("fit", "tune", "held_out")}
    split_rows = {split: 0 for split in ("fit", "tune", "held_out")}
    rollout_owner: Dict[str, Tuple[str, int]] = {}
    seed_owner: Dict[int, str] = {}
    observations = set()
    fields: Tuple[str, ...] = ()
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            fields = tuple(reader.fieldnames or ())
            if fields != RECOVERY_DATASET_COLUMNS:
                reasons.append("recovery dataset header is invalid")
                return {}
            for row in reader:
                split = row.get("split")
                rollout_id = row.get("rollout_id")
                try:
                    seed = int(row.get("seed", ""))
                    phase_index = int(row.get("phase_index", ""))
                except ValueError:
                    reasons.append("recovery dataset contains a non-integer key")
                    return {}
                if (split not in split_seeds or
                        not _nonempty_string(rollout_id) or
                        not _uint32_seed(seed) or phase_index < 0 or
                        row.get("recovered") not in ("0", "1")):
                    reasons.append("recovery dataset contains an invalid row")
                    return {}
                owner = rollout_owner.get(rollout_id)
                if owner is not None and owner != (split, seed):
                    reasons.append("recovery rollout crosses split/seed boundaries")
                rollout_owner[rollout_id] = (split, seed)
                previous_split = seed_owner.get(seed)
                if previous_split is not None and previous_split != split:
                    reasons.append("recovery seed crosses split boundaries")
                seed_owner[seed] = split
                split_seeds[split].add(seed)
                split_rollouts[split].add(rollout_id)
                split_rows[split] += 1
                observation = (rollout_id, phase_index)
                if observation in observations:
                    reasons.append("recovery dataset duplicates rollout/phase")
                observations.add(observation)
    except OSError as error:
        reasons.append("recovery dataset cannot be read: {}".format(error))
        return {}
    for split in ("fit", "tune", "held_out"):
        if sorted(split_seeds[split]) != seed_groups.get("recovery_" + split):
            reasons.append("recovery {} dataset seeds differ from session".format(
                split))
    result: Dict[str, Any] = {"columns": list(fields)}
    for split in ("fit", "tune", "held_out"):
        result[split] = {
            "seeds": sorted(split_seeds[split]),
            "rollout_ids": sorted(split_rollouts[split]),
            "row_count": split_rows[split],
        }
    return result


def _manifest_output_path(manifest_path: Path,
                          entry: Dict[str, Any]) -> Optional[Path]:
    value = entry.get("path")
    if not _nonempty_string(value):
        return None
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        return None
    return (manifest_path.parent / relative).resolve()


def _validate_recovery_bundle(
        paths: Dict[str, Optional[Path]], seed_groups: Dict[str, List[int]],
        reasons: List[str]) -> None:
    dataset_path = paths.get("dataset")
    manifest_path = paths.get("manifest")
    radii_path = paths.get("radii")
    report_path = paths.get("report")
    dataset_contract = _validate_recovery_dataset(
        dataset_path, seed_groups, reasons)
    if manifest_path is None:
        return
    try:
        manifest = load_structured_file(manifest_path)
    except (OSError, ValueError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        reasons.append("recovery fit manifest cannot be parsed: {}".format(error))
        return
    if (manifest.get("schema") != RECOVERY_MANIFEST_SCHEMA or
            manifest.get("status") != "EMPIRICAL_HELD_OUT_PASS" or
            manifest.get("safety_certificate") is not False or
            manifest.get("formal_robot_release") is not False or
            manifest.get("physical_enforce_authorized") is not False):
        reasons.append("recovery fit manifest is not a held-out PASS")

    input_entry = manifest.get("input")
    if not isinstance(input_entry, dict):
        reasons.append("recovery manifest input binding is absent")
    elif dataset_path is None:
        pass
    else:
        input_path = Path(str(input_entry.get("path", "")))
        if not input_path.is_absolute():
            input_path = manifest_path.parent / input_path
        if (input_path.resolve() != dataset_path.resolve() or
                input_entry.get("sha256") != sha256_file(dataset_path) or
                input_entry.get("schema") != RECOVERY_DATASET_SCHEMA or
                input_entry.get("columns") != dataset_contract.get("columns")):
            reasons.append("recovery manifest dataset binding is inconsistent")

    split_contract = manifest.get("split_contract")
    rollout_owners = set()
    seed_owners = set()
    if (not isinstance(split_contract, dict) or
            split_contract.get("unit") != "complete_rollout_and_seed" or
            split_contract.get("mutually_exclusive") is not True):
        reasons.append("recovery manifest split contract is invalid")
    else:
        for split in ("fit", "tune", "held_out"):
            entry = split_contract.get(split)
            if not isinstance(entry, dict):
                reasons.append("recovery manifest {} split is absent".format(
                    split))
                continue
            seeds = entry.get("seeds")
            rollout_ids = entry.get("rollout_ids")
            if (seeds != seed_groups.get("recovery_" + split) or
                    not isinstance(rollout_ids, list) or not rollout_ids or
                    any(not _nonempty_string(value) for value in rollout_ids) or
                    len(rollout_ids) != len(set(rollout_ids)) or
                    not _lowercase_sha256(entry.get("canonical_rows_sha256"))):
                reasons.append("recovery manifest {} split is invalid".format(
                    split))
                continue
            observed = dataset_contract.get(split, {})
            if (entry.get("row_count") != observed.get("row_count") or
                    rollout_ids != observed.get("rollout_ids") or
                    seeds != observed.get("seeds")):
                reasons.append(
                    "recovery manifest {} differs from dataset".format(split))
            if (rollout_owners.intersection(rollout_ids) or
                    seed_owners.intersection(seeds)):
                reasons.append("recovery manifest split identities overlap")
            rollout_owners.update(rollout_ids)
            seed_owners.update(seeds)

    fit = manifest.get("fit")
    tune = manifest.get("tune")
    held_out = manifest.get("held_out")
    if (not isinstance(fit, dict) or fit.get("uses_only_split") != "fit" or
            not isinstance(tune, dict) or
            tune.get("uses_only_split") != "tune" or
            not isinstance(held_out, dict) or
            held_out.get("uses_only_split") != "held_out" or
            held_out.get("evaluation_count") != 1 or
            held_out.get("status") != "PASS"):
        reasons.append("recovery fit/tune/held-out isolation is invalid")

    outputs = manifest.get("outputs")
    if not isinstance(outputs, dict):
        reasons.append("recovery manifest output bindings are absent")
    else:
        for label, frozen_path in (
                ("scales", radii_path), ("held_out_report", report_path)):
            entry = outputs.get(label)
            if not isinstance(entry, dict):
                reasons.append("recovery manifest {} output is absent".format(
                    label))
                continue
            output_path = _manifest_output_path(manifest_path, entry)
            if (output_path is None or frozen_path is None or
                    output_path != frozen_path.resolve() or
                    entry.get("sha256") != sha256_file(frozen_path)):
                reasons.append("recovery manifest {} hash is inconsistent".format(
                    label))
            expected_schema = (RECOVERY_RADII_SCHEMA if label == "scales"
                               else RECOVERY_REPORT_SCHEMA)
            if entry.get("schema") != expected_schema:
                reasons.append("recovery manifest {} schema is invalid".format(
                    label))

    if radii_path is not None:
        try:
            with radii_path.open("r", encoding="utf-8", newline="") as stream:
                reader = csv.reader(stream)
                header = tuple(next(reader))
                has_data = next(reader, None) is not None
            if header != RECOVERY_RADII_COLUMNS or not has_data:
                reasons.append("recovery radii/bounds CSV contract is invalid")
        except (OSError, StopIteration) as error:
            reasons.append("recovery radii/bounds CSV cannot be read: {}".format(
                error))

    if report_path is None:
        return
    try:
        report = load_structured_file(report_path)
    except (OSError, ValueError, json.JSONDecodeError,
            yaml.YAMLError) as error:
        reasons.append("held-out recovery report cannot be parsed: {}".format(
            error))
        return
    if (report.get("schema") != RECOVERY_REPORT_SCHEMA or
            report.get("status") != "PASS" or
            report.get("held_out_evaluation_count") != 1 or
            report.get("held_out_influenced_fit") is not False or
            report.get("held_out_influenced_tuning") is not False or
            report.get("safety_certificate") is not False or
            report.get("formal_robot_release") is not False or
            dataset_path is None or
            report.get("input_sha256") != sha256_file(dataset_path)):
        reasons.append("held-out recovery report did not pass")


def audit_formal_session(
        session: Dict[str, Any], session_path: Path
        ) -> Tuple[List[str], Dict[str, Dict[str, Any]]]:
    """Return all readiness failures and a hash inventory.

    The audit is intentionally fail-closed and accumulates independent reasons
    so one run tells the operator which frozen assets are still missing.
    """
    require_formal_session_schema(session)
    reasons: List[str] = []
    inventory: Dict[str, Dict[str, Any]] = {}
    if session.get("status") != FORMAL_STATUS:
        reasons.append("session is only a pilot candidate")
    if session.get("formal_trials_started") is not False:
        reasons.append("session does not declare formal_trials_started=false")
    seed_groups = _validate_seed_contract(session, reasons)
    _validate_measurement_contract(session, reasons)
    runtime_paths = _validate_runtime_contract(
        session, session_path, reasons, inventory)
    runtime = session.get("runtime")
    if not isinstance(runtime, dict):
        runtime = {}
    controller_manifest = _validate_controller_manifest(
        runtime_paths.get("controller_manifest"), runtime_paths, runtime,
        reasons)

    assets = session.get("assets")
    asset_paths: Dict[str, Optional[Path]] = {}
    if not isinstance(assets, dict):
        reasons.append("formal asset table is absent")
        assets = {}
    for key in REQUIRED_SESSION_ASSETS:
        asset_paths[key] = _artifact_reference(
            assets, key, session_path, reasons, inventory,
            executable=(key in (
                "analysis_script", "offline_plan_generator",
                "nominal_builder")),
            inventory_key="asset." + key)

    plant_config: Optional[Dict[str, Any]] = None
    plant_path = asset_paths.get("plant_config")
    if plant_path is not None:
        try:
            plant_config = load_yaml(plant_path)
            require_simulation_audit_config(plant_config)
        except (OSError, ValueError, yaml.YAMLError) as error:
            reasons.append("plant config is invalid: {}".format(error))
            plant_config = None
    if plant_config is not None:
        provenance = plant_config.get("provenance", {})
        # A formal *simulation* Plant is allowed to freeze rough, deliberately
        # mismatched parameters.  Requiring identification_known_bias=false
        # would encourage relabelling preliminary Scout estimates as physical
        # truth.  Instead, keep the limitation visible and forbid any physical
        # parameter claim while requiring that this exact Plant was exercised.
        if (plant_config.get("status") != FORMAL_STATUS or
                provenance.get("runtime_verified") is not True or
                provenance.get("source_limitations_acknowledged") is not True or
                provenance.get("physical_parameter_claim") is not False or
                plant_config.get("scope", {}).get(
                    "physical_parameter_claim") is not False):
            reasons.append("plant config is not a promoted formal simulation truth")
        experiment = plant_config.get("experiment", {})
        if experiment.get("reserved_formal_seeds") != seed_groups.get(
                "formal_trials"):
            reasons.append("formal trial seeds differ from plant allocation")
        development_seeds = ([experiment.get("smoke_seed")] +
                             list(experiment.get("pilot_seeds", [])))
        recovery_and_formal = [
            seed for group in seed_groups.values() for seed in group]
        if set(development_seeds).intersection(recovery_and_formal):
            reasons.append("session seeds overlap plant development seeds")
        measurement = session.get("measurement_contract", {})
        if (isinstance(measurement, dict) and
                (not _finite_number(measurement.get("fixed_tail_sec")) or
                 not math.isclose(
                     float(measurement.get("fixed_tail_sec", -1.0)),
                     float(experiment.get("fixed_tail_sec", -2.0)),
                     abs_tol=1.0e-12))):
            reasons.append("session fixed tail differs from plant config")

    _validate_path_contract(asset_paths.get("path"), session, reasons)
    artifact_metadata = _validate_phase_rejoin_artifact(
        asset_paths.get("phase_rejoin_artifact"),
        runtime_paths.get("artifact_validator"), reasons)
    _validate_offline_plan_bundle(
        asset_paths.get("offline_plan"),
        asset_paths.get("offline_plan_report"), asset_paths.get("path"),
        reasons)
    _validate_v3_source_bindings(artifact_metadata, asset_paths, reasons)
    _validate_nominal_report(
        asset_paths.get("nominal_validation_report"),
        asset_paths.get("phase_rejoin_artifact"), asset_paths.get("path"),
        asset_paths.get("offline_plan"),
        asset_paths.get("offline_plan_report"),
        asset_paths.get("recovery_fit_manifest"),
        asset_paths.get("recovery_held_out_report"), artifact_metadata,
        reasons)
    _validate_recovery_bundle({
        "dataset": asset_paths.get("recovery_dataset"),
        "manifest": asset_paths.get("recovery_fit_manifest"),
        "radii": asset_paths.get("recovery_radii_bounds"),
        "report": asset_paths.get("recovery_held_out_report"),
    }, seed_groups, reasons)
    _validate_artifact_recovery_binding(
        asset_paths.get("phase_rejoin_artifact"),
        asset_paths.get("recovery_radii_bounds"), reasons)
    _validate_formal_order(
        asset_paths.get("formal_order"),
        seed_groups.get("formal_trials", []), reasons)

    planner_path = asset_paths.get("planner_config")
    if planner_path is not None:
        try:
            planner = load_yaml(planner_path)
        except (OSError, ValueError, yaml.YAMLError) as error:
            reasons.append("planner config cannot be parsed: {}".format(error))
            planner = {}
        delay = planner.get("delay_augmented_phase")
        phase = planner.get("phase_rejoin")
        publish = planner.get("publish_timing")
        expected_recovery_hash = artifact_metadata.get(
            "recovery_artifact_hash")
        if (not isinstance(delay, dict) or delay.get("enabled") is not True or
                delay.get("expected_recovery_artifact_hash") !=
                expected_recovery_hash):
            reasons.append("planner is not bound to the frozen recovery payload")
        if (not isinstance(phase, dict) or phase.get("mode") != "enforce" or
                not _nonempty_string(phase.get("artifact_path"))):
            reasons.append("planner is not bound to enforced phase rejoining")
        else:
            configured_artifact = Path(phase["artifact_path"])
            if not configured_artifact.is_absolute():
                configured_artifact = planner_path.parent / configured_artifact
            frozen_artifact = asset_paths.get("phase_rejoin_artifact")
            if (frozen_artifact is None or
                    configured_artifact.resolve() != frozen_artifact.resolve()):
                reasons.append("planner nominal artifact path differs from session")
        if (not isinstance(publish, dict) or
                publish.get("enabled") is not True or
                not _finite_number(publish.get("estimated_dc_sec")) or
                float(publish.get("estimated_dc_sec", -1.0)) < 0.0):
            reasons.append("planner publish-time model is not frozen")

    _validate_condition_bindings(
        session, session_path, reasons, inventory, controller_manifest)
    for label, record in inventory.items():
        path = Path(record["path"])
        expected = record["expected_sha256"]
        if (not path.is_file() or sha256_file(path) != expected):
            reasons.append("{} changed during readiness audit".format(label))
    # Preserve first occurrence ordering while avoiding noisy duplicates.
    return list(dict.fromkeys(reasons)), inventory


def formal_no_go_reasons(session: Dict[str, Any],
                         session_path: Path) -> List[str]:
    return audit_formal_session(session, session_path)[0]


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
    parser.add_argument("--config", type=Path)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--executable", type=Path)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args(argv)

    try:
        output_dir = ensure_output_scope(args.out_dir)
        config_path = None
        config = None
        config_sha256 = None
        session_path = None
        session = None
        session_sha256 = None
        executable = None
        executable_sha256 = None
        if args.campaign == "formal":
            if args.session is None:
                raise ValueError("formal audit requires --session")
            if args.config is not None or args.executable is not None:
                raise ValueError(
                    "formal audit takes assets only from the frozen session")
            session_path = args.session.resolve()
            session = load_yaml(session_path)
            require_formal_session_schema(session)
            session_sha256 = sha256_file(session_path)
        else:
            if args.config is None:
                raise ValueError("smoke/pilot campaign requires --config")
            if args.session is not None:
                raise ValueError("smoke/pilot does not accept a formal session")
            config_path = args.config.resolve()
            config = load_yaml(config_path)
            require_simulation_development_config(config)
            config_sha256 = sha256_file(config_path)
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
            assert session_path is not None
            assert session is not None
            assert session_sha256 is not None
            reasons, inventory = audit_formal_session(session, session_path)
            if sha256_file(session_path) != session_sha256:
                raise RuntimeError("formal session changed during readiness audit")
            report = {
                "schema": "spmpc_formal_simulation_readiness_v2",
                "freeze_id": session["freeze_id"],
                "session_status": session["status"],
                "status": "NO_GO" if reasons else "READY_NOT_EXECUTED",
                "formal_trials_started": False,
                "session": str(session_path),
                "session_sha256": session_sha256,
                "asset_inventory": inventory,
                "reasons": reasons,
            }
            write_json(output_dir / "formal_readiness.json", report)
            if reasons:
                print("formal simulation NO-GO: " + "; ".join(reasons))
                return 4
            print("formal session is hash-bound and ready; this audit command "
                  "does not execute C0-C4")
            return 5

        assert config_path is not None
        assert config is not None
        assert config_sha256 is not None
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
