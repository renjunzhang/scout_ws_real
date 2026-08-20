#!/usr/bin/env python3
"""Export a development-only phase-rejoin artifact from a ROS bag.

This tool deliberately exports a rolling-local-planner proxy.  It does not
run OfflineSloshOCP, does not construct a recovery policy, and does not fit a
recovery gate.  Recovery commands and gate radii therefore have to be supplied
explicitly for every selected cycle in a separate development parameter CSV.
"""

import argparse
import csv
import dataclasses
import hashlib
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


SCHEMA = "phase_rejoin_empirical_v1"
EVIDENCE_LEVEL = "development_only"
SOURCE = "development_proxy_replay"
ARTIFACT_ROLE = "interface_smoke_only"
NOMINAL_SEQUENCE_KIND = "rolling_local_planner_first_stage_proxy"

HORIZON_TYPE = "spmpc_local_planner/PredictedHorizon"
AUDIT_TYPE = "spmpc_local_planner/ControlCycleAudit"

ARTIFACT_HEADER = (
    "index", "t", "s", "x", "y", "yaw", "v", "omega",
    "eta_x", "eta_x_dot", "eta_y", "eta_y_dot",
    "a", "alpha", "v_s", "u_pub_v", "u_pub_omega",
    "kappa_v", "kappa_omega",
    "r_x", "r_y", "r_yaw", "r_v", "r_omega",
    "r_eta_x", "r_eta_x_dot", "r_eta_y", "r_eta_y_dot",
)

PARAMETER_HEADER = (
    "cycle_id", "kappa_v", "kappa_omega",
    "r_x", "r_y", "r_yaw", "r_v", "r_omega",
    "r_eta_x", "r_eta_x_dot", "r_eta_y", "r_eta_y_dot",
)

STATE_ARRAYS = (
    "t", "x", "y", "yaw", "v", "omega", "s",
    "eta_x", "eta_x_dot", "eta_y", "eta_y_dot", "h_modal",
)
CONTROL_ARRAYS = ("a", "alpha_or_omega", "v_s")
JOINED_STAMPS = (
    "cycle_start_stamp",
    "raw_robot_state_stamp",
    "raw_liquid_state_stamp",
    "robot_state_stamp",
    "liquid_state_stamp",
    "solver_input_epoch",
    "solve_start_stamp",
    "solve_end_stamp",
    "horizon_available_stamp",
)
JOINED_FLOATS = ("raw_state_skew_sec", "aligned_state_skew_sec")
JOINED_VALUES = (
    "state_alignment_required",
    "state_time_aligned",
    "robot_state_interpolated",
    "robot_state_extrapolated",
    "state_alignment_status",
    "solver_status",
)


class ArtifactPreparationError(RuntimeError):
    """A fail-closed artifact preparation error."""


@dataclasses.dataclass(frozen=True)
class DevelopmentParameters:
    cycle_id: int
    kappa_v: float
    kappa_omega: float
    radii: Tuple[float, ...]


@dataclasses.dataclass(frozen=True)
class PreparationOptions:
    contract_id: str
    expected_dt: float
    path_length: float
    expected_frame_id: str = ""
    start_cycle_id: Optional[int] = None
    end_cycle_id: Optional[int] = None
    numeric_tolerance: float = 1.0e-6
    stamp_tolerance_ns: int = 1


@dataclasses.dataclass(frozen=True)
class PreparedArtifact:
    metadata: Mapping[str, str]
    rows: Tuple[Tuple[object, ...], ...]


def _finite(value: object, label: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ArtifactPreparationError(f"{label}: not a number") from exc
    if not math.isfinite(parsed):
        raise ArtifactPreparationError(f"{label}: non-finite value")
    return parsed


def _positive(value: object, label: str) -> float:
    parsed = _finite(value, label)
    if parsed <= 0.0:
        raise ArtifactPreparationError(f"{label}: must be > 0")
    return parsed


def _cycle_id(value: object, label: str) -> int:
    if isinstance(value, bool):
        raise ArtifactPreparationError(f"{label}: invalid cycle_id")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ArtifactPreparationError(f"{label}: invalid cycle_id") from exc
    if parsed <= 0 or str(value).strip() != str(parsed):
        raise ArtifactPreparationError(f"{label}: cycle_id must be a positive integer")
    return parsed


def _stamp_ns(stamp: object, label: str) -> int:
    if stamp is None:
        raise ArtifactPreparationError(f"{label}: missing timestamp")
    if hasattr(stamp, "to_nsec"):
        value = int(stamp.to_nsec())
    elif hasattr(stamp, "secs") and hasattr(stamp, "nsecs"):
        value = int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)
    else:
        raise ArtifactPreparationError(f"{label}: unsupported timestamp object")
    if value < 0:
        raise ArtifactPreparationError(f"{label}: negative timestamp")
    return value


def _header_frame_id(message: object, label: str) -> str:
    header = getattr(message, "header", None)
    frame_id = str(getattr(header, "frame_id", "")).strip()
    if not frame_id:
        raise ArtifactPreparationError(f"{label}: empty header.frame_id")
    return frame_id


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _checked_sha256(value: str, label: str) -> str:
    clean = str(value).strip().lower()
    if len(clean) != 64 or any(character not in "0123456789abcdef" for character in clean):
        raise ArtifactPreparationError(f"{label}: invalid SHA-256")
    return clean


def _metadata_text(value: object, label: str) -> str:
    clean = str(value).strip()
    if not clean or "\n" in clean or "\r" in clean:
        raise ArtifactPreparationError(f"{label}: invalid metadata text")
    return clean


def load_development_parameters(path: Path) -> Dict[int, DevelopmentParameters]:
    """Load explicit, per-cycle development gate and recovery inputs."""
    parameters: Dict[int, DevelopmentParameters] = {}
    try:
        stream = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ArtifactPreparationError(f"cannot open parameter CSV: {path}") from exc
    with stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != PARAMETER_HEADER:
            raise ArtifactPreparationError(
                "development parameter header mismatch; no defaults are permitted"
            )
        for line_number, row in enumerate(reader, start=2):
            cycle_id = _cycle_id(row["cycle_id"], f"parameter line {line_number}")
            if cycle_id in parameters:
                raise ArtifactPreparationError(
                    f"duplicate development parameters for cycle {cycle_id}"
                )
            kappa_v = _finite(row["kappa_v"], f"cycle {cycle_id} kappa_v")
            kappa_omega = _finite(
                row["kappa_omega"], f"cycle {cycle_id} kappa_omega"
            )
            radii = tuple(
                _positive(row[name], f"cycle {cycle_id} {name}")
                for name in PARAMETER_HEADER[3:]
            )
            parameters[cycle_id] = DevelopmentParameters(
                cycle_id=cycle_id,
                kappa_v=kappa_v,
                kappa_omega=kappa_omega,
                radii=radii,
            )
    if not parameters:
        raise ArtifactPreparationError("development parameter CSV is empty")
    return parameters


def _unique_messages(messages: Iterable[object], label: str) -> Dict[int, object]:
    result: Dict[int, object] = {}
    for offset, message in enumerate(messages):
        cycle_id = _cycle_id(getattr(message, "cycle_id", None), f"{label}[{offset}]")
        if cycle_id in result:
            raise ArtifactPreparationError(f"duplicate {label} cycle_id {cycle_id}")
        result[cycle_id] = message
    return result


def _selected_ids(
    horizons: Mapping[int, object],
    audits: Mapping[int, object],
    options: PreparationOptions,
) -> List[int]:
    def selected(cycle_id: int) -> bool:
        if options.start_cycle_id is not None and cycle_id < options.start_cycle_id:
            return False
        if options.end_cycle_id is not None and cycle_id > options.end_cycle_id:
            return False
        return True

    horizon_ids = {cycle_id for cycle_id in horizons if selected(cycle_id)}
    audit_ids = {cycle_id for cycle_id in audits if selected(cycle_id)}
    if horizon_ids != audit_ids:
        missing_horizons = sorted(audit_ids - horizon_ids)
        missing_audits = sorted(horizon_ids - audit_ids)
        raise ArtifactPreparationError(
            "cycle_id pairing mismatch: missing_horizons={} missing_audits={}".format(
                missing_horizons, missing_audits
            )
        )
    ids = sorted(horizon_ids)
    if len(ids) < 2:
        raise ArtifactPreparationError("at least two one-to-one cycle pairs are required")
    for previous, current in zip(ids, ids[1:]):
        if current != previous + 1:
            raise ArtifactPreparationError(
                f"missing cycle between {previous} and {current}"
            )
    return ids


def _check_array_contract(horizon: object, cycle_id: int, dt: float) -> None:
    horizon_steps = int(getattr(horizon, "horizon_steps", -1))
    if horizon_steps <= 0:
        raise ArtifactPreparationError(f"cycle {cycle_id}: empty horizon")
    state_count = horizon_steps + 1
    for name in STATE_ARRAYS:
        values = list(getattr(horizon, name, ()))
        if len(values) != state_count:
            raise ArtifactPreparationError(
                f"cycle {cycle_id}: {name} has {len(values)} values, expected {state_count}"
            )
        for index, value in enumerate(values):
            _finite(value, f"cycle {cycle_id} {name}[{index}]")
    for name in CONTROL_ARRAYS:
        values = list(getattr(horizon, name, ()))
        if len(values) != horizon_steps:
            raise ArtifactPreparationError(
                f"cycle {cycle_id}: {name} has {len(values)} values, expected {horizon_steps}"
            )
        for index, value in enumerate(values):
            _finite(value, f"cycle {cycle_id} {name}[{index}]")
    for index, value in enumerate(horizon.t):
        if abs(float(value) - index * dt) > max(1.0e-9, 1.0e-6 * dt):
            raise ArtifactPreparationError(
                f"cycle {cycle_id}: horizon t[{index}] is inconsistent with dt"
            )


def _check_pair(
    horizon: object,
    audit: object,
    cycle_id: int,
    options: PreparationOptions,
) -> Tuple[int, str]:
    if int(getattr(horizon, "schema_version", -1)) != 2:
        raise ArtifactPreparationError(f"cycle {cycle_id}: PredictedHorizon schema != 2")
    if int(getattr(audit, "schema_version", -1)) != 1:
        raise ArtifactPreparationError(f"cycle {cycle_id}: ControlCycleAudit schema != 1")
    if not bool(getattr(horizon, "valid", False)):
        raise ArtifactPreparationError(f"cycle {cycle_id}: invalid PredictedHorizon")
    if not bool(getattr(horizon, "slosh_enabled", False)):
        raise ArtifactPreparationError(f"cycle {cycle_id}: slosh prediction is disabled")
    if str(getattr(horizon, "control_semantics", "")) != "alpha":
        raise ArtifactPreparationError(
            f"cycle {cycle_id}: control semantics must be alpha"
        )
    required_true = (
        "solve_attempted", "solve_success", "command_accepted",
        "publish_cmd_vel", "command_was_published", "state_time_aligned",
    )
    for name in required_true:
        if not bool(getattr(audit, name, False)):
            raise ArtifactPreparationError(f"cycle {cycle_id}: audit.{name} is false")
    forbidden_true = (
        "command_contract_violation",
        "terminal_controller_intervened",
        "safety_gate_intervened",
        "linear_limited",
        "angular_rate_limited",
        "angular_accel_limited",
    )
    for name in forbidden_true:
        if bool(getattr(audit, name, False)):
            raise ArtifactPreparationError(f"cycle {cycle_id}: audit.{name} intervened")

    horizon_variant = str(getattr(horizon, "variant", ""))
    audit_variant = str(getattr(audit, "variant", ""))
    if not horizon_variant or horizon_variant != audit_variant:
        raise ArtifactPreparationError(f"cycle {cycle_id}: variant mismatch")
    frame_id = _header_frame_id(horizon, f"cycle {cycle_id} horizon")
    if frame_id != _header_frame_id(audit, f"cycle {cycle_id} audit"):
        raise ArtifactPreparationError(f"cycle {cycle_id}: frame mismatch")
    if options.expected_frame_id and frame_id != options.expected_frame_id:
        raise ArtifactPreparationError(
            f"cycle {cycle_id}: frame {frame_id} != {options.expected_frame_id}"
        )

    horizon_dt = _positive(getattr(horizon, "dt", None), f"cycle {cycle_id} dt")
    if abs(horizon_dt - options.expected_dt) > options.numeric_tolerance:
        raise ArtifactPreparationError(f"cycle {cycle_id}: horizon dt mismatch")
    _check_array_contract(horizon, cycle_id, horizon_dt)

    for name in JOINED_STAMPS:
        horizon_stamp = _stamp_ns(getattr(horizon, name, None), f"horizon.{name}")
        audit_stamp = _stamp_ns(getattr(audit, name, None), f"audit.{name}")
        if horizon_stamp <= 0 or audit_stamp <= 0:
            raise ArtifactPreparationError(f"cycle {cycle_id}: {name} is zero")
        if abs(horizon_stamp - audit_stamp) > options.stamp_tolerance_ns:
            raise ArtifactPreparationError(
                f"cycle {cycle_id}: {name} differs between horizon and audit"
            )
    for name in JOINED_FLOATS:
        horizon_value = _finite(getattr(horizon, name, None), f"horizon.{name}")
        audit_value = _finite(getattr(audit, name, None), f"audit.{name}")
        if abs(horizon_value - audit_value) > options.numeric_tolerance:
            raise ArtifactPreparationError(
                f"cycle {cycle_id}: {name} differs between horizon and audit"
            )
    for name in JOINED_VALUES:
        if getattr(horizon, name, None) != getattr(audit, name, None):
            raise ArtifactPreparationError(
                f"cycle {cycle_id}: {name} differs between horizon and audit"
            )
    cycle_start = _stamp_ns(horizon.cycle_start_stamp, "cycle_start_stamp")
    solve_start = _stamp_ns(horizon.solve_start_stamp, "solve_start_stamp")
    solve_end = _stamp_ns(horizon.solve_end_stamp, "solve_end_stamp")
    available = _stamp_ns(horizon.horizon_available_stamp, "horizon_available_stamp")
    if cycle_start <= 0 or not (cycle_start <= solve_start <= solve_end <= available):
        raise ArtifactPreparationError(f"cycle {cycle_id}: invalid solve timestamp order")
    solver_epoch = _stamp_ns(horizon.solver_input_epoch, "solver_input_epoch")
    robot_epoch = _stamp_ns(horizon.robot_state_stamp, "robot_state_stamp")
    liquid_epoch = _stamp_ns(horizon.liquid_state_stamp, "liquid_state_stamp")
    if (abs(robot_epoch - solver_epoch) > options.stamp_tolerance_ns or
            abs(liquid_epoch - solver_epoch) > options.stamp_tolerance_ns):
        raise ArtifactPreparationError(
            f"cycle {cycle_id}: aligned states do not share solver input epoch"
        )

    first_a = _finite(horizon.a[0], f"cycle {cycle_id} a[0]")
    first_alpha = _finite(
        horizon.alpha_or_omega[0], f"cycle {cycle_id} alpha[0]"
    )
    if abs(first_a - _finite(audit.solver_u0_a, "audit.solver_u0_a")) > options.numeric_tolerance:
        raise ArtifactPreparationError(f"cycle {cycle_id}: first a does not join")
    if abs(first_alpha - _finite(audit.solver_u0_alpha, "audit.solver_u0_alpha")) > options.numeric_tolerance:
        raise ArtifactPreparationError(f"cycle {cycle_id}: first alpha does not join")
    _finite(audit.published_cmd_v, f"cycle {cycle_id} published_cmd_v")
    _finite(audit.published_cmd_omega, f"cycle {cycle_id} published_cmd_omega")
    return solver_epoch, frame_id


def prepare_artifact(
    horizon_messages: Iterable[object],
    audit_messages: Iterable[object],
    parameters: Mapping[int, DevelopmentParameters],
    options: PreparationOptions,
    bag_sha256: str,
    parameter_sha256: str,
) -> PreparedArtifact:
    """Join messages by cycle_id and prepare strict development-only rows."""
    contract_id = _metadata_text(options.contract_id, "contract_id")
    _positive(options.expected_dt, "expected_dt")
    _positive(options.path_length, "path_length")
    if options.start_cycle_id is not None and options.start_cycle_id <= 0:
        raise ArtifactPreparationError("start_cycle_id must be positive")
    if options.end_cycle_id is not None and options.end_cycle_id <= 0:
        raise ArtifactPreparationError("end_cycle_id must be positive")
    if (options.start_cycle_id is not None and options.end_cycle_id is not None and
            options.start_cycle_id > options.end_cycle_id):
        raise ArtifactPreparationError("start_cycle_id exceeds end_cycle_id")

    horizons = _unique_messages(horizon_messages, "PredictedHorizon")
    audits = _unique_messages(audit_messages, "ControlCycleAudit")
    ids = _selected_ids(horizons, audits, options)
    missing_parameters = [cycle_id for cycle_id in ids if cycle_id not in parameters]
    if missing_parameters:
        raise ArtifactPreparationError(
            f"missing explicit development parameters: {missing_parameters}"
        )

    rows: List[Tuple[object, ...]] = []
    first_epoch: Optional[int] = None
    previous_epoch: Optional[int] = None
    previous_s: Optional[float] = None
    frame_id = ""
    variant = ""
    for index, cycle_id in enumerate(ids):
        horizon = horizons[cycle_id]
        audit = audits[cycle_id]
        epoch, pair_frame = _check_pair(horizon, audit, cycle_id, options)
        if first_epoch is None:
            first_epoch = epoch
            frame_id = pair_frame
            variant = str(horizon.variant)
        elif pair_frame != frame_id or str(horizon.variant) != variant:
            raise ArtifactPreparationError(
                f"cycle {cycle_id}: frame or variant changed within artifact"
            )
        if previous_epoch is not None:
            period = (epoch - previous_epoch) * 1.0e-9
            # The proxy publishes /clock at 50 Hz while this controller runs at
            # 30 Hz, so ROS timer callbacks form a bounded 40/40/20 ms pattern.
            # Accept that quantization locally, then bound cumulative phase
            # error below so a genuinely drifting sample stream still fails.
            period_tolerance = max(1.0e-4, 0.40 * options.expected_dt) + 1.0e-9
            if period <= 0.0 or abs(period - options.expected_dt) > period_tolerance:
                raise ArtifactPreparationError(
                    f"cycle {cycle_id}: solver input epoch period {period:.9g} is inconsistent"
                )
        previous_epoch = epoch

        s = _finite(horizon.s[0], f"cycle {cycle_id} s[0]")
        if s < 0.0 or (previous_s is not None and s + 1.0e-9 < previous_s):
            raise ArtifactPreparationError(f"cycle {cycle_id}: non-monotonic progress")
        if s > options.path_length + 0.10:
            raise ArtifactPreparationError(f"cycle {cycle_id}: progress exceeds path_length")
        previous_s = s
        params = parameters[cycle_id]
        relative_time = (epoch - first_epoch) * 1.0e-9
        phase_tolerance = max(1.0e-4, options.expected_dt) + 1.0e-9
        if abs(relative_time - index * options.expected_dt) > phase_tolerance:
            raise ArtifactPreparationError(
                f"cycle {cycle_id}: cumulative solver epoch drift is inconsistent"
            )
        row = (
            index,
            relative_time,
            s,
            _finite(horizon.x[0], f"cycle {cycle_id} x[0]"),
            _finite(horizon.y[0], f"cycle {cycle_id} y[0]"),
            _finite(horizon.yaw[0], f"cycle {cycle_id} yaw[0]"),
            _finite(horizon.v[0], f"cycle {cycle_id} v[0]"),
            _finite(horizon.omega[0], f"cycle {cycle_id} omega[0]"),
            _finite(horizon.eta_x[0], f"cycle {cycle_id} eta_x[0]"),
            _finite(horizon.eta_x_dot[0], f"cycle {cycle_id} eta_x_dot[0]"),
            _finite(horizon.eta_y[0], f"cycle {cycle_id} eta_y[0]"),
            _finite(horizon.eta_y_dot[0], f"cycle {cycle_id} eta_y_dot[0]"),
            _finite(horizon.a[0], f"cycle {cycle_id} a[0]"),
            _finite(horizon.alpha_or_omega[0], f"cycle {cycle_id} alpha[0]"),
            _finite(horizon.v_s[0], f"cycle {cycle_id} v_s[0]"),
            _finite(audit.published_cmd_v, f"cycle {cycle_id} published_cmd_v"),
            _finite(audit.published_cmd_omega, f"cycle {cycle_id} published_cmd_omega"),
            params.kappa_v,
            params.kappa_omega,
        ) + params.radii
        if len(row) != len(ARTIFACT_HEADER):
            raise ArtifactPreparationError("internal artifact row width error")
        rows.append(row)

    metadata = {
        "schema": SCHEMA,
        "evidence_level": EVIDENCE_LEVEL,
        "source": SOURCE,
        "contract_id": contract_id,
        "frame_id": _metadata_text(frame_id, "frame_id"),
        "dt": _format_float(options.expected_dt),
        "path_length": _format_float(options.path_length),
        "artifact_role": ARTIFACT_ROLE,
        "nominal_sequence_kind": NOMINAL_SEQUENCE_KIND,
        "offline_slosh_ocp": "false",
        "hardware_formal_release": "false",
        "paper_main_result_eligible": "false",
        "cycle_id_first": str(ids[0]),
        "cycle_id_last": str(ids[-1]),
        "cycle_count": str(len(ids)),
        "planner_variant": _metadata_text(variant, "planner_variant"),
        "gate_parameter_source": "operator_supplied_per_cycle_development_csv",
        "recovery_policy_source": "operator_supplied_per_cycle_development_csv",
        "gate_evidence": "none_development_input_only",
        "recovery_policy_evidence": "none_development_input_only",
        "bag_sha256": _checked_sha256(bag_sha256, "bag_sha256"),
        "development_parameter_sha256": _checked_sha256(
            parameter_sha256, "development_parameter_sha256"
        ),
        "row_state_semantics": "predicted_horizon_stage0_at_solver_input_epoch",
        "row_command_semantics": "same_cycle_final_published_command",
    }
    return PreparedArtifact(metadata=metadata, rows=tuple(rows))


def _format_float(value: object) -> str:
    return format(float(value), ".17g")


def _serialized_rows(prepared: PreparedArtifact) -> Iterable[Sequence[str]]:
    yield ARTIFACT_HEADER
    for row in prepared.rows:
        yield tuple(str(value) if isinstance(value, int) else _format_float(value) for value in row)


def _artifact_tool_path() -> Path:
    configured = os.environ.get("SPMPC_PHASE_REJOIN_ARTIFACT_TOOL", "").strip()
    candidates: List[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser())
    executable = shutil.which("spmpc_phase_rejoin_artifact_tool")
    if executable:
        candidates.append(Path(executable))
    for parent in Path(__file__).resolve().parents:
        candidates.append(
            parent
            / "devel/lib/spmpc_local_planner/spmpc_phase_rejoin_artifact_tool"
        )
    for candidate in candidates:
        if candidate.is_file() and os.access(str(candidate), os.X_OK):
            return candidate.resolve()
    raise ArtifactPreparationError(
        "C++ artifact tool is unavailable; build spmpc_phase_rejoin_artifact_tool "
        "or set SPMPC_PHASE_REJOIN_ARTIFACT_TOOL"
    )


def _run_artifact_tool(arguments: Sequence[str]) -> None:
    command = [str(_artifact_tool_path()), *arguments]
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except OSError as exc:
        raise ArtifactPreparationError("failed to execute C++ artifact tool") from exc
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise ArtifactPreparationError(
            detail or f"C++ artifact tool failed with exit code {completed.returncode}"
        )


def _write_candidate_csv(path: Path, prepared: PreparedArtifact) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        for key, value in prepared.metadata.items():
            stream.write(f"# {key}={value}\n")
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerows(_serialized_rows(prepared))
        stream.flush()
        os.fsync(stream.fileno())


def write_artifact(path: Path, prepared: PreparedArtifact, overwrite: bool = False) -> None:
    """Delegate domain validation and atomic final publication to the C++ core."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = ""
    try:
        with tempfile.NamedTemporaryFile(
            prefix=path.name + ".tmp.",
            dir=str(path.parent),
            delete=False,
        ) as stream:
            temporary_name = stream.name
        _write_candidate_csv(Path(temporary_name), prepared)
        arguments = [
            "canonicalize",
            "--input", temporary_name,
            "--output", str(path),
            "--development-only",
        ]
        if overwrite:
            arguments.append("--overwrite")
        _run_artifact_tool(arguments)
    finally:
        if temporary_name:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass


def validate_artifact_csv(path: Path) -> Mapping[str, str]:
    """Validate through the C++ artifact core, then expose parsed metadata."""
    _run_artifact_tool(
        ["validate", "--artifact", str(path), "--development-only"]
    )
    metadata: Dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ArtifactPreparationError(f"cannot read artifact: {path}") from exc
    for line_number, line in enumerate(lines, start=1):
        clean = line.strip()
        if not clean:
            continue
        if clean.startswith("#"):
            item = clean[1:].strip()
            if "=" not in item:
                raise ArtifactPreparationError(f"invalid metadata at line {line_number}")
            key, value = (part.strip() for part in item.split("=", 1))
            if not key or not value or key in metadata:
                raise ArtifactPreparationError(f"invalid metadata at line {line_number}")
            metadata[key] = value
    return metadata


def read_rosbag_pairs(
    bag_path: Path,
    horizon_topic: str,
    audit_topic: str,
) -> Tuple[List[object], List[object]]:
    try:
        import rosbag  # Imported lazily so unit tests do not require ROS.
    except ImportError as exc:
        raise ArtifactPreparationError("rosbag Python module is unavailable") from exc
    horizons: List[object] = []
    audits: List[object] = []
    try:
        with rosbag.Bag(str(bag_path), "r") as bag:
            topic_info = bag.get_type_and_topic_info()[1]
            expected_types = {
                horizon_topic: HORIZON_TYPE,
                audit_topic: AUDIT_TYPE,
            }
            for topic, expected_type in expected_types.items():
                if topic not in topic_info:
                    raise ArtifactPreparationError(f"bag topic missing: {topic}")
                if topic_info[topic].msg_type != expected_type:
                    raise ArtifactPreparationError(
                        f"{topic}: type {topic_info[topic].msg_type} != {expected_type}"
                    )
            for topic, message, _ in bag.read_messages(
                topics=[horizon_topic, audit_topic]
            ):
                if topic == horizon_topic:
                    horizons.append(message)
                elif topic == audit_topic:
                    audits.append(message)
    except ArtifactPreparationError:
        raise
    except Exception as exc:
        raise ArtifactPreparationError(f"failed to read bag: {bag_path}") from exc
    return horizons, audits


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Prepare or validate a DEVELOPMENT-ONLY phase-rejoin artifact. "
            "The export is never an OfflineSloshOCP/formal hardware artifact."
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    export = subparsers.add_parser("export", help="export from a ROS bag")
    export.add_argument("--bag", type=Path, required=True)
    export.add_argument("--output", type=Path, required=True)
    export.add_argument("--development-parameters", type=Path, required=True)
    export.add_argument("--contract-id", required=True)
    export.add_argument("--dt", type=float, required=True)
    export.add_argument("--path-length", type=float, required=True)
    export.add_argument("--frame-id", default="")
    export.add_argument("--start-cycle-id", type=int)
    export.add_argument("--end-cycle-id", type=int)
    export.add_argument(
        "--horizon-topic", default="/spmpc/debug/predicted_horizon"
    )
    export.add_argument(
        "--audit-topic", default="/spmpc/debug/control_cycle_audit"
    )
    export.add_argument("--overwrite", action="store_true")
    validate = subparsers.add_parser("validate", help="validate a development artifact")
    validate.add_argument("--artifact", type=Path, required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "validate":
            metadata = validate_artifact_csv(args.artifact)
            print(
                "VALID DEVELOPMENT_ONLY artifact: source={} contract_id={}".format(
                    metadata["source"], metadata["contract_id"]
                )
            )
            return 0
        horizons, audits = read_rosbag_pairs(
            args.bag.resolve(), args.horizon_topic, args.audit_topic
        )
        parameters = load_development_parameters(
            args.development_parameters.resolve()
        )
        prepared = prepare_artifact(
            horizons,
            audits,
            parameters,
            PreparationOptions(
                contract_id=args.contract_id,
                expected_dt=args.dt,
                path_length=args.path_length,
                expected_frame_id=args.frame_id,
                start_cycle_id=args.start_cycle_id,
                end_cycle_id=args.end_cycle_id,
            ),
            bag_sha256=_sha256(args.bag.resolve()),
            parameter_sha256=_sha256(args.development_parameters.resolve()),
        )
        write_artifact(args.output.resolve(), prepared, overwrite=args.overwrite)
        print("EXPORTED DEVELOPMENT_ONLY artifact: {}".format(args.output.resolve()))
        print("source={}; artifact_role={}".format(SOURCE, ARTIFACT_ROLE))
        print("NOT OfflineSloshOCP; NOT a formal hardware or paper-result artifact")
        return 0
    except ArtifactPreparationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
