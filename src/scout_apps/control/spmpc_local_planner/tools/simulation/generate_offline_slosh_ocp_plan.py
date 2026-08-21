#!/usr/bin/env python3
"""Generate a deterministic, simulation-only OfflineSloshOCP command plan.

This tool optimizes a small set of path-indexed speed variables and rolls the
result through the exact compiled 22-state delay-augmented transition.  The
output is an intermediate command plan, not a Phase-Rejoin recovery artifact:
the C++ nominal builder must still bind fitted recovery radii, validate every
transition and seal the canonical artifact hash.

The path JSON keeps its recorded quaternion yaw.  This is intentional: the
offline plan, the standalone simulation and the future robot runner must all
agree on the same endpoint orientation instead of reconstructing yaw from x/y
after the fact.
"""

import argparse
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
CODEGEN_ROOT = PACKAGE_ROOT / "tools" / "codegen" / "acados"
if str(CODEGEN_ROOT) not in sys.path:
    sys.path.insert(0, str(CODEGEN_ROOT))

from generate_delay_augmented_phase_transition import load_contract  # noqa: E402
from spmpc_delay_augmented_phase_model import export_transition_functions  # noqa: E402


PLAN_SCHEMA = "spmpc_offline_slosh_ocp_plan_v1"
REPORT_SCHEMA = "spmpc_offline_slosh_ocp_report_v1"
PLAN_COLUMNS = ("index", "t", "u_pub_v", "u_pub_omega", "v_s")
EPSILON = 1.0e-12
DEFAULT_MAX_PATH_DEVIATION_M = 0.075
DEFAULT_GOAL_POSITION_TOLERANCE_M = 0.10
DEFAULT_GOAL_YAW_TOLERANCE_RAD = 0.10


class PlanError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(pose: Dict[str, Any]) -> float:
    keys = ("qx", "qy", "qz", "qw")
    if any(key not in pose for key in keys):
        raise PlanError("path pose is missing a recorded quaternion")
    try:
        values = [float(pose[key]) for key in keys]
    except (TypeError, ValueError, OverflowError) as error:
        raise PlanError("path contains an invalid quaternion") from error
    if not all(math.isfinite(value) for value in values):
        raise PlanError("path contains a non-finite quaternion")
    scale = max(abs(value) for value in values)
    if scale <= EPSILON:
        raise PlanError("path contains a zero quaternion")
    scaled = [value / scale for value in values]
    norm = math.sqrt(sum(value * value for value in scaled))
    qx, qy, qz, qw = (value / norm for value in scaled)
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class FrozenPath:
    def __init__(self, path: Path):
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
            raise PlanError("cannot parse path JSON: {}".format(path)) from error
        poses = value.get("poses") if isinstance(value, dict) else None
        if not isinstance(poses, list) or len(poses) < 2:
            raise PlanError("path JSON must contain at least two poses")
        self.frame_id = value.get("frame_id")
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise PlanError("path frame_id is missing")
        points: List[Tuple[float, float]] = []
        yaws: List[float] = []
        for pose in poses:
            if not isinstance(pose, dict):
                raise PlanError("path pose is not a mapping")
            try:
                x = float(pose.get("x", math.nan))
                y = float(pose.get("y", math.nan))
            except (TypeError, ValueError, OverflowError) as error:
                raise PlanError("path contains an invalid position") from error
            if not math.isfinite(x) or not math.isfinite(y):
                raise PlanError("path contains a non-finite position")
            points.append((x, y))
            yaws.append(yaw_from_quaternion(pose))
        self.x = np.asarray([point[0] for point in points], dtype=float)
        self.y = np.asarray([point[1] for point in points], dtype=float)
        segment = np.hypot(np.diff(self.x), np.diff(self.y))
        if np.any(~np.isfinite(segment)):
            raise PlanError("path contains a non-finite segment")
        if np.any(segment <= 1.0e-6):
            raise PlanError("path contains a duplicate or near-duplicate pose")
        self.s = np.concatenate(([0.0], np.cumsum(segment)))
        self.length = float(self.s[-1])
        self.yaw_unwrapped = np.unwrap(np.asarray(yaws, dtype=float))
        # Curvature is used only for feed-forward initialization.  Quaternion
        # yaw remains the authoritative orientation at interpolation points.
        self.curvature = np.gradient(self.yaw_unwrapped, self.s)

    def sample(self, progress: float) -> Tuple[float, float, float, float]:
        s = min(self.length, max(0.0, float(progress)))
        x = float(np.interp(s, self.s, self.x))
        y = float(np.interp(s, self.s, self.y))
        yaw = wrap(float(np.interp(s, self.s, self.yaw_unwrapped)))
        curvature = float(np.interp(s, self.s, self.curvature))
        return x, y, yaw, curvature

    def phase_distance(self, x: float, y: float, progress: float) -> float:
        reference_x, reference_y, _, _ = self.sample(progress)
        return math.hypot(x - reference_x, y - reference_y)


def modal_height_coefficient(contract: Dict[str, Any]) -> float:
    root = 1.8412
    radius = float(contract["slosh"]["container_radius"])
    height = float(contract["slosh"]["liquid_height"])
    argument = root * height / radius
    mass_ratio = (
        2.0 * radius * math.tanh(argument)
        / (root * height * (root * root - 1.0))
    )
    return 4.0 * height * mass_ratio / radius


def initial_state(path: FrozenPath, contract: Dict[str, Any], layout: Dict[str, Any]) -> np.ndarray:
    state = np.zeros(layout["state_width"], dtype=float)
    state[0] = path.x[0]
    state[1] = path.y[0]
    state[2] = wrap(float(path.yaw_unwrapped[0]))
    return state


def slew(current: float, target: float, maximum_delta: float) -> float:
    return max(current - maximum_delta, min(current + maximum_delta, target))


def speed_scale(progress: float, path_length: float, values: Sequence[float]) -> float:
    knots = np.linspace(0.0, path_length, len(values))
    return float(np.interp(progress, knots, np.asarray(values, dtype=float)))


def rollout(
    path: FrozenPath,
    contract: Dict[str, Any],
    transition: Any,
    layout: Dict[str, Any],
    scales: Sequence[float],
    zero_hold_sec: float,
    maximum_motion_sec: float,
    goal_yaw_tolerance_rad: float = DEFAULT_GOAL_YAW_TOLERANCE_RAD,
) -> Dict[str, Any]:
    dt = float(contract["dt"])
    state = initial_state(path, contract, layout)
    rows: List[Tuple[int, float, float, float, float]] = []
    states: List[np.ndarray] = [state.copy()]
    distances: List[float] = [0.0]
    heights: List[float] = [0.0]
    published_v = 0.0
    published_omega = 0.0
    a_max = 0.60
    alpha_max = 1.20
    nominal_v_max = 0.38
    lateral_acceleration_limit = 0.28
    heading_gain = 2.2
    cross_track_gain = 1.6
    max_motion_steps = int(math.ceil(maximum_motion_sec / dt))
    reached = False
    height_coefficient = modal_height_coefficient(contract)
    _, _, goal_yaw, _ = path.sample(path.length)

    for index in range(max_motion_steps):
        progress = float(state[4])
        reference_x, reference_y, reference_yaw, curvature = path.sample(progress)
        remaining = max(0.0, path.length - progress)
        curve_speed = math.sqrt(
            lateral_acceleration_limit / max(abs(curvature), 0.05)
        )
        braking_speed = math.sqrt(max(0.0, 2.0 * 0.42 * remaining))
        target_v = min(nominal_v_max, curve_speed, braking_speed)
        target_v *= speed_scale(progress, path.length, scales)
        at_endpoint = remaining <= 1.0e-7
        if at_endpoint:
            target_v = 0.0
        published_v = slew(published_v, target_v, a_max * dt)

        # The angular command starts acting later than the linear command.
        # Query curvature at that physical lead, while using explicit
        # cross-track feedback to keep the exact delayed rollout on the frozen
        # geometry instead of advancing an arbitrary fixed lookahead.
        if at_endpoint:
            yaw_error = wrap(goal_yaw - float(state[2]))
            target_omega = heading_gain * yaw_error
        else:
            _, _, lookahead_yaw, lookahead_curvature = path.sample(
                min(path.length, progress + published_v * 0.22)
            )
            yaw_error = wrap(lookahead_yaw - float(state[2]))
            cross_track = (
                -math.sin(reference_yaw) * (float(state[0]) - reference_x)
                + math.cos(reference_yaw) * (float(state[1]) - reference_y)
            )
            target_omega = (
                lookahead_curvature * max(published_v, 0.05)
                + heading_gain * yaw_error
                - cross_track_gain * cross_track
            )
        target_omega = max(-0.90, min(0.90, target_omega))
        published_omega = slew(
            published_omega, target_omega, alpha_max * dt
        )

        progress_rate = 0.0 if at_endpoint else max(
            0.0,
            min(
                0.80,
                max(0.0, float(state[3]))
                * max(0.0, math.cos(wrap(float(state[2]) - reference_yaw))),
            ),
        )
        if remaining <= progress_rate * dt + 1.0e-12:
            progress_rate = remaining / dt
        rows.append((index, index * dt, published_v, published_omega, progress_rate))
        previous_v = float(state[layout["linear_buffer_offset"] + layout["linear_buffer_count"] - 1])
        previous_omega = float(state[layout["angular_buffer_offset"] + layout["angular_buffer_count"] - 1])
        control = np.asarray([
            (published_v - previous_v) / dt,
            (published_omega - previous_omega) / dt,
            progress_rate,
        ])
        state = np.asarray(transition(state, control)[0]).reshape(-1)
        states.append(state.copy())
        distances.append(
            path.phase_distance(float(state[0]), float(state[1]), float(state[4]))
        )
        heights.append(height_coefficient * math.hypot(float(state[6]), float(state[8])))
        if (
            float(state[4]) >= path.length - 1.0e-9
            and abs(wrap(float(state[2]) - goal_yaw))
            <= goal_yaw_tolerance_rad
        ):
            reached = True
            break

    if not reached:
        return {
            "valid": False,
            "reason": "path progress/yaw did not reach the endpoint contract",
            "rows": rows,
            "states": states,
            "heights": heights,
            "distances": distances,
        }

    # Rate-limited slowdown first, then a genuine zero-published-command hold.
    while abs(published_v) > 1.0e-12 or abs(published_omega) > 1.0e-12:
        index = len(rows)
        published_v = slew(published_v, 0.0, a_max * dt)
        published_omega = slew(published_omega, 0.0, alpha_max * dt)
        if abs(published_v) < 1.0e-12:
            published_v = 0.0
        if abs(published_omega) < 1.0e-12:
            published_omega = 0.0
        rows.append((index, index * dt, published_v, published_omega, 0.0))
        previous_v = float(state[layout["linear_buffer_offset"] + layout["linear_buffer_count"] - 1])
        previous_omega = float(state[layout["angular_buffer_offset"] + layout["angular_buffer_count"] - 1])
        control = np.asarray([
            (published_v - previous_v) / dt,
            (published_omega - previous_omega) / dt,
            0.0,
        ])
        state = np.asarray(transition(state, control)[0]).reshape(-1)
        states.append(state.copy())
        distances.append(
            path.phase_distance(float(state[0]), float(state[1]), float(state[4]))
        )
        heights.append(height_coefficient * math.hypot(float(state[6]), float(state[8])))

    zero_hold_steps = max(5, int(math.ceil(zero_hold_sec / dt)))
    # Include one final command row because every V3 sample, including the last
    # one, carries a published-command consistency image.
    for _ in range(zero_hold_steps):
        index = len(rows)
        rows.append((index, index * dt, 0.0, 0.0, 0.0))
        previous_v = float(state[layout["linear_buffer_offset"] + layout["linear_buffer_count"] - 1])
        previous_omega = float(state[layout["angular_buffer_offset"] + layout["angular_buffer_count"] - 1])
        control = np.asarray([
            -previous_v / dt,
            -previous_omega / dt,
            0.0,
        ])
        state = np.asarray(transition(state, control)[0]).reshape(-1)
        states.append(state.copy())
        distances.append(
            path.phase_distance(float(state[0]), float(state[1]), float(state[4]))
        )
        heights.append(height_coefficient * math.hypot(float(state[6]), float(state[8])))

    # The plan has one command per nominal sample.  The final command is not
    # advanced by the C++ builder, so discard its extra diagnostic state.
    states = states[: len(rows)]
    heights = heights[: len(rows)]
    distances = distances[: len(rows)]
    final = states[-1]
    goal_x, goal_y, goal_yaw, _ = path.sample(path.length)
    terminal_position_error = math.hypot(float(final[0]) - goal_x, float(final[1]) - goal_y)
    terminal_yaw_error = abs(wrap(float(final[2]) - goal_yaw))
    return {
        "valid": True,
        "reason": "OK",
        "rows": rows,
        "states": states,
        "heights": heights,
        "distances": distances,
        "zero_hold_steps": zero_hold_steps,
        "duration_sec": rows[-1][1],
        "terminal_position_error_m": terminal_position_error,
        "terminal_yaw_error_rad": terminal_yaw_error,
        "terminal_eta_norm": math.hypot(float(final[6]), float(final[8])),
        "terminal_eta_dot_norm": math.hypot(float(final[7]), float(final[9])),
    }


def metrics(result: Dict[str, Any]) -> Dict[str, float]:
    if not result.get("valid"):
        return {
            "objective": 1.0e9,
            "height_q95_m": 1.0,
            "height_peak_m": 1.0,
            "height_rms_m": 1.0,
            "max_path_deviation_m": 1.0,
            "duration_sec": 1.0e3,
        }
    heights = np.asarray(result["heights"], dtype=float)
    distances = np.asarray(result["distances"], dtype=float)
    height_q95 = float(np.quantile(heights, 0.95))
    height_peak = float(np.max(heights))
    height_rms = float(math.sqrt(float(np.mean(heights * heights))))
    path_peak = float(np.max(distances))
    duration = float(result["duration_sec"])
    # All geometric terms are in SI units.  Scaling below only conditions the
    # search; the report retains unscaled physical metrics.
    objective = (
        6.0e5 * height_rms * height_rms
        + 2.0e5 * height_q95 * height_q95
        + 40.0 * path_peak * path_peak
        + 60.0 * float(result["terminal_position_error_m"]) ** 2
        + 4.0 * float(result["terminal_yaw_error_rad"]) ** 2
        + 0.015 * duration
    )
    return {
        "objective": objective,
        "height_q95_m": height_q95,
        "height_peak_m": height_peak,
        "height_rms_m": height_rms,
        "max_path_deviation_m": path_peak,
        "duration_sec": duration,
        "terminal_position_error_m": float(result["terminal_position_error_m"]),
        "terminal_yaw_error_rad": float(result["terminal_yaw_error_rad"]),
        "terminal_eta_norm": float(result["terminal_eta_norm"]),
        "terminal_eta_dot_norm": float(result["terminal_eta_dot_norm"]),
    }


def render_plan(
    rows: Sequence[Tuple[int, float, float, float, float]],
    metadata: Dict[str, str],
) -> bytes:
    lines = ["# {}={}".format(key, metadata[key]) for key in sorted(metadata)]
    output: List[str] = lines + [",".join(PLAN_COLUMNS)]
    for row in rows:
        output.append(",".join(
            str(value) if isinstance(value, int) else format(float(value), ".17g")
            for value in row
        ))
    return ("\n".join(output) + "\n").encode("utf-8")


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    descriptor = os.open(str(path), flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_exclusive(path: Path, contents: bytes) -> Tuple[int, int]:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".{}.tmp.".format(path.name), dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    published_identity = None
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(contents)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(str(temporary), str(path))
        except OSError as error:
            if error.errno == errno.EEXIST:
                raise PlanError("output already exists: {}".format(path)) from error
            raise PlanError("cannot publish output: {}".format(path)) from error
        status = path.stat()
        published_identity = (status.st_dev, status.st_ino)
        _fsync_directory(path.parent)
        return published_identity
    except PlanError:
        raise
    except OSError as error:
        if published_identity is not None:
            unlink_if_same(path, published_identity)
        raise PlanError("cannot create output: {}".format(path)) from error
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def unlink_if_same(path: Path, identity: Tuple[int, int]) -> None:
    try:
        status = path.stat()
        if (status.st_dev, status.st_ino) == identity:
            path.unlink()
            _fsync_directory(path.parent)
    except FileNotFoundError:
        pass


def validate_path_separation(input_path: Path, outputs: Sequence[Path]) -> None:
    paths = (input_path,) + tuple(outputs)
    resolved = [path.resolve() for path in paths]
    if len(set(resolved)) != len(resolved):
        raise PlanError("input and output paths must not alias")
    for left_index, left in enumerate(paths):
        if not left.exists():
            continue
        for right in paths[left_index + 1 :]:
            if right.exists() and left.samefile(right):
                raise PlanError("input and output paths must not alias")


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--path-json", required=True)
    parser.add_argument("--plan-output", required=True)
    parser.add_argument("--report-output", required=True)
    parser.add_argument("--segment-count", type=int, default=8)
    parser.add_argument("--zero-hold-sec", type=float, default=4.0)
    parser.add_argument("--maximum-motion-sec", type=float, default=45.0)
    parser.add_argument(
        "--max-path-deviation-m",
        type=float,
        default=DEFAULT_MAX_PATH_DEVIATION_M,
    )
    parser.add_argument(
        "--goal-position-tolerance-m",
        type=float,
        default=DEFAULT_GOAL_POSITION_TOLERANCE_M,
    )
    parser.add_argument(
        "--goal-yaw-tolerance-rad",
        type=float,
        default=DEFAULT_GOAL_YAW_TOLERANCE_RAD,
    )
    parser.add_argument("--maxiter", type=int, default=24)
    parser.add_argument("--no-optimize", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.segment_count < 2 or args.segment_count > 32:
            raise PlanError("segment-count must be in [2, 32]")
        durations = (args.zero_hold_sec, args.maximum_motion_sec)
        tolerances = (
            args.max_path_deviation_m,
            args.goal_position_tolerance_m,
            args.goal_yaw_tolerance_rad,
        )
        if (
            any(not math.isfinite(value) for value in durations + tolerances)
            or args.zero_hold_sec < 1.0
            or args.maximum_motion_sec <= 0.0
            or args.maxiter <= 0
            or args.max_path_deviation_m <= 0.0
            or args.goal_position_tolerance_m <= 0.0
            or not 0.0 < args.goal_yaw_tolerance_rad <= math.pi
        ):
            raise PlanError("invalid duration option")
        path_path = Path(args.path_json).resolve()
        plan_output = Path(args.plan_output).resolve()
        report_output = Path(args.report_output).resolve()
        validate_path_separation(path_path, (plan_output, report_output))
        if plan_output.exists() or report_output.exists():
            raise PlanError("plan/report outputs must be create-new")
        path_sha256 = sha256_file(path_path)
        path = FrozenPath(path_path)
        contract = load_contract()
        exported = export_transition_functions(contract)
        transition = exported["step"]
        layout = exported["layout"]
        initial = np.full(args.segment_count, 0.90, dtype=float)
        baseline_rollout = rollout(
            path, contract, transition, layout, initial,
            args.zero_hold_sec, args.maximum_motion_sec,
            args.goal_yaw_tolerance_rad,
        )
        baseline_metrics = metrics(baseline_rollout)

        cache: Dict[Tuple[float, ...], Tuple[Dict[str, Any], Dict[str, float]]] = {}

        def evaluate(values: np.ndarray) -> float:
            key = tuple(round(float(value), 12) for value in values)
            if key not in cache:
                trial = rollout(
                    path, contract, transition, layout, values,
                    args.zero_hold_sec, args.maximum_motion_sec,
                    args.goal_yaw_tolerance_rad,
                )
                cache[key] = (trial, metrics(trial))
            # A weak regularizer prevents adjacent speed knots from forming an
            # artificial sawtooth that the rate limiter would merely erase.
            regularizer = 0.01 * float(np.sum(np.diff(values) ** 2))
            return cache[key][1]["objective"] + regularizer

        if args.no_optimize:
            optimized_values = initial
            optimize_success = True
            optimize_message = "optimization explicitly disabled"
            evaluations = 1
        else:
            optimized = minimize(
                evaluate,
                initial,
                method="Powell",
                bounds=[(0.55, 1.05)] * args.segment_count,
                options={"maxiter": args.maxiter, "xtol": 2.0e-3, "ftol": 2.0e-4},
            )
            optimized_values = np.asarray(optimized.x, dtype=float)
            optimize_success = bool(optimized.success)
            optimize_message = str(optimized.message)
            evaluations = int(optimized.nfev)
        optimized_rollout = rollout(
            path, contract, transition, layout, optimized_values,
            args.zero_hold_sec, args.maximum_motion_sec,
            args.goal_yaw_tolerance_rad,
        )
        optimized_metrics = metrics(optimized_rollout)
        if not optimized_rollout.get("valid"):
            raise PlanError("optimized rollout is invalid: {}".format(
                optimized_rollout.get("reason", "unknown")))
        if optimized_metrics["max_path_deviation_m"] > args.max_path_deviation_m:
            raise PlanError("optimized nominal leaves the path envelope")
        if (
            optimized_metrics["terminal_position_error_m"]
            > args.goal_position_tolerance_m
        ):
            raise PlanError("optimized nominal does not reach the goal position")
        if (
            optimized_metrics["terminal_yaw_error_rad"]
            > args.goal_yaw_tolerance_rad
        ):
            raise PlanError("optimized nominal does not reach the goal yaw")
        if sha256_file(path_path) != path_sha256:
            raise PlanError("path JSON changed during plan generation")

        # "PASS" means only that a reproducible, dynamically rolled plan was
        # produced.  A full paper claim still requires independent-Plant C0-C4
        # trials and held-out recovery evidence.
        status = "PASS" if optimize_success or args.no_optimize else "PILOT_ONLY"
        metadata = {
            "schema": PLAN_SCHEMA,
            "status": status,
            "simulation_only": "true",
            "formal_robot_release": "false",
            "path_sha256": path_sha256,
            "path_frame_id": path.frame_id,
            "path_length": format(path.length, ".17g"),
            "execution_contract_hash": contract["contract_hash"],
            "dt": format(float(contract["dt"]), ".17g"),
            "zero_hold_steps": str(int(optimized_rollout["zero_hold_steps"])),
        }
        plan_bytes = render_plan(optimized_rollout["rows"], metadata)
        report = {
            "schema": REPORT_SCHEMA,
            "status": status,
            "simulation_only": True,
            "formal_robot_release": False,
            "physical_parameter_claim": False,
            "source_limitations_acknowledged": True,
            "paper_main_result_eligible": False,
            "claim_scope": "intermediate OfflineSloshOCP nominal generation only",
            "path": {
                "path": str(path_path),
                "sha256": path_sha256,
                "frame_id": path.frame_id,
                "pose_count": int(path.x.size),
                "length_m": path.length,
            },
            "execution_contract_hash": contract["contract_hash"],
            "optimizer": {
                "method": "Powell_path_indexed_speed_ocp_v1",
                "success": optimize_success,
                "message": optimize_message,
                "function_evaluations": evaluations,
                "segment_count": args.segment_count,
                "speed_scales": [float(value) for value in optimized_values],
                "bounds": [0.55, 1.05],
            },
            "baseline": baseline_metrics,
            "optimized": optimized_metrics,
            "optimized_minus_baseline": {
                key: optimized_metrics[key] - baseline_metrics[key]
                for key in optimized_metrics
            },
            "terminal_contract": {
                "name": "publish_zero_settle_hold_v2",
                "zero_hold_steps": int(optimized_rollout["zero_hold_steps"]),
                "goal_yaw_preserved_from_path_quaternion": True,
                "goal_yaw_tolerance_rad": args.goal_yaw_tolerance_rad,
            },
            "acceptance_thresholds": {
                "max_phase_indexed_path_deviation_m":
                    args.max_path_deviation_m,
                "goal_position_tolerance_m": args.goal_position_tolerance_m,
                "goal_yaw_tolerance_rad": args.goal_yaw_tolerance_rad,
            },
            "plan": {
                "path": str(plan_output),
                "sha256": hashlib.sha256(plan_bytes).hexdigest(),
                "rows": len(optimized_rollout["rows"]),
                "schema": PLAN_SCHEMA,
            },
        }
        report_bytes = (
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
        ).encode("utf-8")
        plan_identity = write_exclusive(plan_output, plan_bytes)
        try:
            write_exclusive(report_output, report_bytes)
        except Exception:
            unlink_if_same(plan_output, plan_identity)
            raise
        print("[{}] rows={} objective={:.6g} height_q95={:.6g} m path_dev={:.4f} m".format(
            status,
            len(optimized_rollout["rows"]),
            optimized_metrics["objective"],
            optimized_metrics["height_q95_m"],
            optimized_metrics["max_path_deviation_m"],
        ))
        return 0 if status == "PASS" else 4
    except (OSError, PlanError, TypeError, ValueError, KeyError, OverflowError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
