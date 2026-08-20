#!/usr/bin/env python3
"""Build the fail-closed G4 trajectory and snapshot-replay evidence from G3.

The tool is deliberately offline. It reads rosbag files directly, never
connects to a ROS master, and never publishes a command. Python owns bag and
trajectory analysis; the source-tree ``spmpc_g4_snapshot_replay`` executable
owns generated-solver capsules, sequential iterate restoration, and checkpoint
forks through the same C++ ABI boundary as the runtime backend.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import rosbag
import yaml


SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"
HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"
ODOM_TOPIC = "/odom"
TF_TOPIC = "/tf"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def quaternion_yaw(q: Any) -> float:
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def load_path_xy(path: Path) -> np.ndarray:
    data = json.loads(path.read_text(encoding="utf-8"))
    poses = data.get("poses") if isinstance(data, Mapping) else None
    if not isinstance(poses, list) or len(poses) < 2:
        raise RuntimeError("path JSON must contain at least two poses")
    xy = np.asarray([[float(p["x"]), float(p["y"])] for p in poses], dtype=float)
    if not np.all(np.isfinite(xy)):
        raise RuntimeError("path contains non-finite coordinates")
    if np.any(np.linalg.norm(np.diff(xy, axis=0), axis=1) <= 1e-9):
        raise RuntimeError("path contains duplicate adjacent poses")
    return xy


def path_geometry(xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    segments = np.diff(xy, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    yaw = np.unwrap(np.arctan2(np.gradient(xy[:, 1]), np.gradient(xy[:, 0])))
    curvature = np.gradient(yaw, cumulative)
    return cumulative, lengths, curvature


def project_point_to_polyline(
    point: np.ndarray,
    xy: np.ndarray,
    cumulative: np.ndarray,
    lengths: np.ndarray,
) -> Tuple[float, float, int]:
    start = xy[:-1]
    delta = xy[1:] - start
    denom = np.square(lengths)
    ratios = np.sum((point - start) * delta, axis=1) / denom
    ratios = np.clip(ratios, 0.0, 1.0)
    closest = start + ratios[:, None] * delta
    distance_sq = np.sum(np.square(closest - point), axis=1)
    index = int(np.argmin(distance_sq))
    progress = float(cumulative[index] + ratios[index] * lengths[index])
    return progress, float(math.sqrt(distance_sq[index])), index


@dataclass
class TransformSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass
class OdomSample:
    stamp: float
    x: float
    y: float
    yaw: float
    v: float
    omega: float


def interpolate_transform(samples: Sequence[TransformSample], stamp: float) -> Tuple[TransformSample, float]:
    if not samples:
        raise RuntimeError("bag has no map->odom transforms")
    stamps = [sample.stamp for sample in samples]
    index = int(np.searchsorted(stamps, stamp, side="left"))
    if index <= 0:
        chosen = samples[0]
        return chosen, abs(chosen.stamp - stamp)
    if index >= len(samples):
        chosen = samples[-1]
        return chosen, abs(chosen.stamp - stamp)
    left = samples[index - 1]
    right = samples[index]
    span = right.stamp - left.stamp
    if span <= 1e-12:
        return left, abs(left.stamp - stamp)
    ratio = min(1.0, max(0.0, (stamp - left.stamp) / span))
    dyaw = wrap_angle(right.yaw - left.yaw)
    result = TransformSample(
        stamp=stamp,
        x=left.x + ratio * (right.x - left.x),
        y=left.y + ratio * (right.y - left.y),
        yaw=left.yaw + ratio * dyaw,
    )
    return result, min(abs(left.stamp - stamp), abs(right.stamp - stamp))


def read_trajectory_inputs(bag_path: Path) -> Tuple[List[TransformSample], List[OdomSample]]:
    transforms: List[TransformSample] = []
    odom: List[OdomSample] = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, bag_stamp in bag.read_messages(topics=[TF_TOPIC, ODOM_TOPIC]):
            if topic == TF_TOPIC:
                for transform in msg.transforms:
                    if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                        stamp = float(transform.header.stamp.to_sec() or bag_stamp.to_sec())
                        transforms.append(
                            TransformSample(
                                stamp=stamp,
                                x=float(transform.transform.translation.x),
                                y=float(transform.transform.translation.y),
                                yaw=quaternion_yaw(transform.transform.rotation),
                            )
                        )
            else:
                stamp = float(msg.header.stamp.to_sec() or bag_stamp.to_sec())
                pose = msg.pose.pose
                odom.append(
                    OdomSample(
                        stamp=stamp,
                        x=float(pose.position.x),
                        y=float(pose.position.y),
                        yaw=quaternion_yaw(pose.orientation),
                        v=float(msg.twist.twist.linear.x),
                        omega=float(msg.twist.twist.angular.z),
                    )
                )
    transforms.sort(key=lambda item: item.stamp)
    odom.sort(key=lambda item: item.stamp)
    return transforms, odom


def moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or values.size < window:
        return values.copy()
    if window % 2 == 0:
        raise RuntimeError("smoothing_window_samples must be odd")
    radius = window // 2
    padded = np.pad(values, (radius, radius), mode="edge")
    return np.convolve(padded, np.ones(window, dtype=float) / float(window), mode="valid")


def first_crossing_time(stamps: np.ndarray, progress: np.ndarray, target: float) -> float:
    indices = np.flatnonzero(progress >= target)
    if indices.size == 0:
        return math.nan
    index = int(indices[0])
    if index == 0:
        return float(stamps[0])
    p0 = float(progress[index - 1])
    p1 = float(progress[index])
    if p1 <= p0 + 1e-12:
        return float(stamps[index])
    ratio = (target - p0) / (p1 - p0)
    return float(stamps[index - 1] + ratio * (stamps[index] - stamps[index - 1]))


def extract_trajectory(
    bag_path: Path,
    path_xy: np.ndarray,
    motion_start: float,
    first_arrival: float,
    config: Mapping[str, Any],
) -> Tuple[List[Dict[str, float]], Dict[str, Any]]:
    cumulative, lengths, _curvature = path_geometry(path_xy)
    path_length = float(cumulative[-1])
    transforms, odom = read_trajectory_inputs(bag_path)
    if not odom:
        raise RuntimeError("bag has no odometry")

    rows: List[Dict[str, float]] = []
    raw_progress: List[float] = []
    guarded_progress: List[float] = []
    cross_track: List[float] = []
    tf_gaps: List[float] = []
    backtrack_count = 0
    jump_count = 0
    previous_guarded = 0.0
    previous_stamp: Optional[float] = None
    backtrack_tolerance = float(config["monotonic_backtrack_tolerance_m"])
    jump_margin = float(config["progress_jump_margin_m"])

    for sample in odom:
        transform, gap = interpolate_transform(transforms, sample.stamp)
        cosine = math.cos(transform.yaw)
        sine = math.sin(transform.yaw)
        map_x = transform.x + cosine * sample.x - sine * sample.y
        map_y = transform.y + sine * sample.x + cosine * sample.y
        map_yaw = wrap_angle(transform.yaw + sample.yaw)
        raw_s, distance, _segment = project_point_to_polyline(
            np.asarray([map_x, map_y]), path_xy, cumulative, lengths
        )
        guarded = raw_s
        if rows:
            if raw_s < previous_guarded - backtrack_tolerance:
                backtrack_count += 1
            guarded = max(previous_guarded, raw_s)
            dt = max(1e-6, sample.stamp - float(previous_stamp))
            allowed_jump = max(0.05, 1.5 * max(0.0, abs(sample.v)) * dt + jump_margin)
            if guarded - previous_guarded > allowed_jump:
                jump_count += 1
        previous_guarded = guarded
        previous_stamp = sample.stamp
        raw_progress.append(raw_s)
        guarded_progress.append(guarded)
        cross_track.append(distance)
        tf_gaps.append(gap)
        rows.append(
            {
                "stamp": sample.stamp,
                "x_map": map_x,
                "y_map": map_y,
                "yaw_map": map_yaw,
                "v_odom": sample.v,
                "omega_odom": sample.omega,
                "s_proj_raw_m": raw_s,
                "s_proj_m": guarded,
                "sigma": guarded / path_length,
                "cross_track_m": distance,
                "tf_nearest_gap_sec": gap,
            }
        )

    selected = [row for row in rows if motion_start <= row["stamp"] <= first_arrival]
    minimum_samples = int(config["minimum_motion_samples"])
    if len(selected) < minimum_samples:
        raise RuntimeError(
            "trajectory motion window has {} samples, require {}".format(len(selected), minimum_samples)
        )
    stamps = np.asarray([row["stamp"] for row in selected], dtype=float)
    progress = np.asarray([row["s_proj_m"] for row in selected], dtype=float)
    progress = np.maximum.accumulate(progress)
    v = np.asarray([row["v_odom"] for row in selected], dtype=float)
    omega = np.asarray([row["omega_odom"] for row in selected], dtype=float)

    rate = float(config["resample_hz"])
    uniform_t = np.arange(stamps[0], stamps[-1] + 0.5 / rate, 1.0 / rate)
    uniform_s = np.interp(uniform_t, stamps, progress)
    uniform_v = moving_average(
        np.interp(uniform_t, stamps, v), int(config["smoothing_window_samples"])
    )
    uniform_omega = moving_average(
        np.interp(uniform_t, stamps, omega), int(config["smoothing_window_samples"])
    )
    ax = np.gradient(uniform_v, uniform_t)
    ay = uniform_v * uniform_omega

    boundaries = [float(value) for value in config["zone_boundaries_sigma"]]
    zone_times = []
    for index in range(5):
        enter = first_crossing_time(stamps, progress, boundaries[index] * path_length)
        leave = first_crossing_time(stamps, progress, boundaries[index + 1] * path_length)
        zone_times.append(
            {
                "zone": "Z{}".format(index + 1),
                "sigma_start": boundaries[index],
                "sigma_end": boundaries[index + 1],
                "enter_sec": enter,
                "leave_sec": leave,
                "duration_sec": leave - enter if finite(enter) and finite(leave) else math.nan,
            }
        )
    sigma_grid = np.linspace(0.0, 1.0, 101)
    timing = [
        {
            "sigma": float(sigma),
            "elapsed_sec": first_crossing_time(stamps, progress, sigma * path_length) - stamps[0],
        }
        for sigma in sigma_grid
    ]
    motion_cross_track = [row["cross_track_m"] for row in selected]
    motion_tf_gaps = [row["tf_nearest_gap_sec"] for row in selected]
    tf_limit = float(config["tf_max_nearest_gap_sec"])
    summary = {
        "path_length_m": path_length,
        "motion_sample_count": len(selected),
        "motion_start_sec": motion_start,
        "first_arrival_sec": first_arrival,
        "completion_sec": first_arrival - motion_start,
        "s_proj_start_m": float(progress[0]),
        "s_proj_end_m": float(progress[-1]),
        "sigma_end": float(progress[-1] / path_length),
        "cross_track_p95_m": float(np.percentile(motion_cross_track, 95)),
        "cross_track_max_m": float(max(motion_cross_track)),
        "tf_nearest_gap_max_sec": float(max(motion_tf_gaps)),
        "tf_gap_over_limit_count": int(sum(gap > tf_limit for gap in motion_tf_gaps)),
        "raw_backtrack_count": backtrack_count,
        "progress_jump_count": jump_count,
        "ax_rms_m_s2": float(math.sqrt(float(np.mean(np.square(ax))))),
        "ay_abs_p95_m_s2": float(np.percentile(np.abs(ay), 95)),
        "zone_traversal": zone_times,
        "timing_sigma_grid": timing,
    }
    return selected, summary


@dataclass
class SnapshotPair:
    index: int
    snapshot: Any
    horizon: Any


def read_snapshot_pairs(
    bag_path: Path, motion_start: float, motion_end: float
) -> List[SnapshotPair]:
    pairs: List[SnapshotPair] = []
    previous_horizon: Any = None
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, _bag_stamp in bag.read_messages(topics=[HORIZON_TOPIC, SNAPSHOT_TOPIC]):
            if topic == HORIZON_TOPIC:
                previous_horizon = msg
                continue
            if previous_horizon is None or not bool(msg.valid) or not bool(previous_horizon.valid):
                continue
            stamp = float(msg.header.stamp.to_sec())
            horizon_stamp = float(previous_horizon.header.stamp.to_sec())
            if abs(stamp - horizon_stamp) > 0.01:
                continue
            if not (motion_start <= stamp <= motion_end):
                continue
            if msg.variant != "B_slosh" or previous_horizon.variant != "B_slosh":
                continue
            pairs.append(SnapshotPair(len(pairs), msg, previous_horizon))
    if not pairs:
        raise RuntimeError("no valid B_slosh snapshot/horizon pairs in the motion window")
    return pairs


def select_checkpoints(
    pairs: Sequence[SnapshotPair], selection: Mapping[str, Any]
) -> Dict[str, SnapshotPair]:
    sigma_min = float(selection["minimum_sigma"])
    sigma_max = float(selection["maximum_sigma"])
    longitudinal_limit = float(selection["longitudinal_max_abs_omega_rad_s"])
    lateral_limit = float(selection["lateral_min_abs_omega_rad_s"])
    contiguous_count = int(selection.get("minimum_contiguous_pairs", 1))
    maximum_gap = float(selection.get("maximum_pair_gap_sec", math.inf))

    def contiguous(pair: SnapshotPair) -> bool:
        if pair.index < contiguous_count - 1:
            return False
        if contiguous_count <= 1:
            return True
        start = pair.index - contiguous_count + 1
        stamps = [float(item.snapshot.header.stamp.to_sec()) for item in pairs[start : pair.index + 1]]
        return all(
            0.0 < stamps[index + 1] - stamps[index] <= maximum_gap
            for index in range(len(stamps) - 1)
        )

    common = [
        pair
        for pair in pairs
        if pair.snapshot.reference_length > 0.0
        and sigma_min <= pair.snapshot.s0 / pair.snapshot.reference_length <= sigma_max
        and pair.horizon.solver_status.endswith("ACADOS_OK")
        and len(pair.horizon.a) > 0
        and contiguous(pair)
    ]
    longitudinal = [pair for pair in common if abs(float(pair.snapshot.robot_omega)) <= longitudinal_limit]
    lateral = [pair for pair in common if abs(float(pair.snapshot.robot_omega)) >= lateral_limit]
    if not longitudinal:
        raise RuntimeError("no longitudinal checkpoint candidate")
    if not lateral:
        raise RuntimeError("no lateral checkpoint candidate")
    return {
        "longitudinal": max(longitudinal, key=lambda pair: abs(float(pair.horizon.a[0]))),
        "lateral": max(
            lateral,
            key=lambda pair: abs(float(pair.snapshot.robot_v) * float(pair.snapshot.robot_omega)),
        ),
    }


def replay_executable_path(configured: Optional[Path] = None) -> Path:
    candidates: List[Path] = []
    if configured is not None:
        candidates.append(configured.expanduser())
    environment = os.environ.get("SPMPC_G4_SNAPSHOT_REPLAY", "").strip()
    if environment:
        candidates.append(Path(environment).expanduser())
    discovered = shutil.which("spmpc_g4_snapshot_replay")
    if discovered:
        candidates.append(Path(discovered))
    for parent in Path(__file__).resolve().parents:
        candidates.append(
            parent / "devel/lib/spmpc_local_planner/spmpc_g4_snapshot_replay"
        )
    for candidate in candidates:
        if candidate.is_file() and os.access(str(candidate), os.X_OK):
            return candidate.resolve()
    raise RuntimeError(
        "C++ G4 replay executable is unavailable; build "
        "spmpc_g4_snapshot_replay or set SPMPC_G4_SNAPSHOT_REPLAY"
    )


def linked_replay_libraries(executable: Path) -> Dict[str, Path]:
    required = {
        "libspmpc_g4_snapshot_replay_core.so",
        "libspmpc_solver_acados.so",
        "libacados_ocp_solver_spmpc_slosh.so",
        "libacados.so",
        "libhpipm.so",
        "libblasfeo.so.0",
    }
    completed = subprocess.run(
        ["ldd", str(executable)],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError("cannot resolve C++ replay dependencies")
    resolved: Dict[str, Path] = {}
    for line in completed.stdout.splitlines():
        fields = line.strip().split()
        if len(fields) >= 3 and fields[0] in required and fields[1] == "=>":
            path = Path(fields[2]).resolve()
            if not path.is_file():
                raise RuntimeError("linked replay library is missing: {}".format(path))
            resolved[fields[0]] = path
    missing = sorted(required - set(resolved))
    if missing:
        raise RuntimeError(
            "C++ replay lacks required linked libraries: {}".format(",".join(missing))
        )
    return resolved


def validate_codegen_snapshot_contract(codegen_json: Path, snapshot: Any) -> None:
    contract = json.loads(codegen_json.read_text(encoding="utf-8"))
    dimensions = contract.get("dims", {})
    generated = (
        int(dimensions.get("N", -1)),
        int(dimensions.get("nx", -1)),
        int(dimensions.get("nu", -1)),
        int(dimensions.get("np", -1)),
    )
    recorded = (
        int(snapshot.horizon_steps),
        int(snapshot.state_width),
        int(snapshot.control_width),
        int(snapshot.parameter_width),
    )
    if contract.get("name") != "spmpc_slosh" or generated != recorded:
        raise RuntimeError(
            "codegen/snapshot contract mismatch: generated={} recorded={}".format(
                generated, recorded
            )
        )


def parameter_index(snapshot: Any, name: str) -> int:
    names = list(snapshot.parameter_names)
    if name not in names:
        raise RuntimeError("snapshot parameter list lacks {!r}".format(name))
    return names.index(name)


def modal_override_for_phase(
    snapshot: Any,
    direction: str,
    phase: float,
    phase_height_mm: float,
    slosh_height_ref_m: float,
) -> Tuple[float, float, float, float]:
    width = int(snapshot.parameter_width)
    parameters = np.asarray(snapshot.stage_parameters, dtype=float).reshape(
        int(snapshot.horizon_steps) + 1, width
    )
    omega_sq = float(parameters[0, parameter_index(snapshot, "omega_n_sq")])
    eta_ref = float(parameters[0, parameter_index(snapshot, "eta_ref")])
    if omega_sq <= 0.0 or eta_ref <= 0.0:
        raise RuntimeError("snapshot has invalid omega_n_sq/eta_ref")
    omega_n = math.sqrt(omega_sq)
    height_coeff = slosh_height_ref_m / eta_ref
    eta_amplitude = (phase_height_mm / 1000.0) / height_coeff
    eta = eta_amplitude * math.cos(phase)
    eta_dot = -omega_n * eta_amplitude * math.sin(phase)
    if direction == "longitudinal":
        return eta, eta_dot, 0.0, 0.0
    if direction == "lateral":
        return 0.0, 0.0, eta, eta_dot
    raise RuntimeError("unknown phase direction: {}".format(direction))


def _float_tokens(values: Iterable[Any]) -> str:
    return " ".join(format(float(value), ".17g") for value in values)


def _write_vector(stream: Any, name: str, values: Iterable[Any]) -> None:
    normalized = list(values)
    stream.write("{} {}".format(name, len(normalized)))
    if normalized:
        stream.write(" " + _float_tokens(normalized))
    stream.write("\n")


def _checkpoint_modals(
    snapshot: Any,
    direction: str,
    replay_config: Mapping[str, Any],
) -> List[Tuple[float, float, float, float]]:
    modals = [(0.0, 0.0, 0.0, 0.0)]
    for phase in [float(value) for value in replay_config["phases_rad"]]:
        modals.append(
            modal_override_for_phase(
                snapshot,
                direction,
                phase,
                float(replay_config["constructed_phase_height_mm"]),
                float(replay_config["slosh_height_ref_m"]),
            )
        )
    return modals


def write_replay_request(
    path: Path,
    pairs: Sequence[SnapshotPair],
    checkpoints: Mapping[str, SnapshotPair],
    replay_config: Mapping[str, Any],
) -> None:
    checkpoint_by_index = {
        pair.index: direction for direction, pair in checkpoints.items()
    }
    final_index = max(pair.index for pair in checkpoints.values())
    frames = [pair for pair in pairs if pair.index <= final_index]
    direction_codes = {None: 0, "longitudinal": 1, "lateral": 2}
    with path.open("w", encoding="utf-8") as stream:
        stream.write("SPMPC_G4_SNAPSHOT_REPLAY_V1\n")
        stream.write("frames {}\n".format(len(frames)))
        for pair in frames:
            snapshot = pair.snapshot
            direction = checkpoint_by_index.get(pair.index)
            stream.write("frame\n")
            stream.write("pair_index {}\n".format(pair.index))
            stream.write("direction {}\n".format(direction_codes[direction]))
            stream.write(
                "dimensions {} {} {} {}\n".format(
                    int(snapshot.horizon_steps),
                    int(snapshot.state_width),
                    int(snapshot.control_width),
                    int(snapshot.parameter_width),
                )
            )
            stream.write("dt {}\n".format(format(float(snapshot.dt), ".17g")))
            initial_state = (
                snapshot.robot_x,
                snapshot.robot_y,
                snapshot.robot_yaw,
                snapshot.robot_v,
                snapshot.s0,
                snapshot.robot_omega,
                snapshot.eta_x,
                snapshot.eta_x_dot,
                snapshot.eta_y,
                snapshot.eta_y_dot,
            )
            stream.write("initial_state " + _float_tokens(initial_state) + "\n")
            bounds = (
                snapshot.a_min,
                snapshot.a_max,
                snapshot.alpha_or_omega_min,
                snapshot.alpha_or_omega_max,
                snapshot.v_s_min,
                snapshot.v_s_max,
                snapshot.v_min,
                snapshot.v_max,
                snapshot.omega_min,
                snapshot.omega_max,
            )
            stream.write("bounds " + _float_tokens(bounds) + "\n")
            _write_vector(stream, "stage_parameters", snapshot.stage_parameters)
            _write_vector(
                stream, "initial_guess_states", snapshot.initial_guess_states
            )
            _write_vector(
                stream, "initial_guess_controls", snapshot.initial_guess_controls
            )
            modals = (
                _checkpoint_modals(snapshot, direction, replay_config)
                if direction is not None
                else []
            )
            stream.write("modal_overrides {}".format(len(modals)))
            if modals:
                stream.write(" " + _float_tokens(value for modal in modals for value in modal))
            stream.write("\nend_frame\n")
        stream.write("end_request\n")


def read_replay_result(path: Path) -> Dict[str, Any]:
    tokens = iter(path.read_text(encoding="utf-8").split())

    def expect(expected: str) -> None:
        actual = next(tokens, None)
        if actual != expected:
            raise RuntimeError(
                "invalid C++ replay result: expected {!r}, got {!r}".format(
                    expected, actual
                )
            )

    def named(name: str, converter: Any) -> Any:
        expect(name)
        value = next(tokens, None)
        if value is None:
            raise RuntimeError("truncated C++ replay result at {}".format(name))
        return converter(value)

    def vector(name: str) -> np.ndarray:
        count = named(name, int)
        values = []
        for _index in range(count):
            value = next(tokens, None)
            if value is None:
                raise RuntimeError("truncated C++ replay vector {}".format(name))
            values.append(float(value))
        return np.asarray(values, dtype=float)

    def solution() -> Dict[str, Any]:
        status = named("solution", int)
        states = vector("states")
        controls = vector("controls")
        expect("end_solution")
        return {"status": status, "states": states, "controls": controls}

    expect("SPMPC_G4_SNAPSHOT_REPLAY_RESULT_V1")
    success = bool(named("success", int))
    detail = named("detail", str)
    failed_pair = named("failed_pair", int)
    checkpoint_count = named("checkpoints", int)
    checkpoints: Dict[int, Dict[str, Any]] = {}
    for _index in range(checkpoint_count):
        expect("checkpoint")
        pair_index = named("pair_index", int)
        direction = named("direction", int)
        expect("actual")
        actual = solution()
        counterfactual_count = named("counterfactuals", int)
        counterfactuals = [solution() for _item in range(counterfactual_count)]
        expect("end_checkpoint")
        if pair_index in checkpoints:
            raise RuntimeError("duplicate checkpoint in C++ replay result")
        checkpoints[pair_index] = {
            "direction": direction,
            "actual": actual,
            "counterfactuals": counterfactuals,
        }
    expect("end_result")
    if next(tokens, None) is not None:
        raise RuntimeError("unexpected trailing C++ replay result data")
    return {
        "success": success,
        "detail": detail,
        "failed_pair": failed_pair,
        "checkpoints": checkpoints,
    }


def _solver_result(solution: Mapping[str, Any], snapshot: Any) -> Dict[str, Any]:
    horizon_steps = int(snapshot.horizon_steps)
    state_width = int(snapshot.state_width)
    control_width = int(snapshot.control_width)
    states = np.asarray(solution["states"], dtype=float).reshape(
        horizon_steps + 1, state_width
    )
    controls = np.asarray(solution["controls"], dtype=float).reshape(
        horizon_steps, control_width
    )
    u0 = controls[0]
    raw_cmd = np.asarray(
        [
            float(snapshot.robot_v) + float(u0[0]) * float(snapshot.dt),
            float(snapshot.robot_omega) + float(u0[1]) * float(snapshot.dt),
        ]
    )
    return {"states": states, "controls": controls, "u0": u0, "raw_cmd": raw_cmd}


def online_horizon_arrays(horizon: Any) -> Tuple[np.ndarray, np.ndarray]:
    states = np.column_stack(
        [
            horizon.x,
            horizon.y,
            horizon.yaw,
            horizon.v,
            horizon.s,
            horizon.omega,
            horizon.eta_x,
            horizon.eta_x_dot,
            horizon.eta_y,
            horizon.eta_y_dot,
        ]
    )
    controls = np.column_stack([horizon.a, horizon.alpha_or_omega, horizon.v_s])
    return np.asarray(states, dtype=float), np.asarray(controls, dtype=float)


def branch_metrics(actual: Mapping[str, Any], candidate: Mapping[str, Any]) -> Dict[str, Any]:
    state_delta = np.asarray(candidate["states"]) - np.asarray(actual["states"])
    control_delta = np.asarray(candidate["controls"]) - np.asarray(actual["controls"])
    return {
        "u0_delta": [
            float(value)
            for value in np.asarray(candidate["u0"]) - np.asarray(actual["u0"])
        ],
        "u0_linf": float(np.max(np.abs(np.asarray(candidate["u0"]) - np.asarray(actual["u0"])))),
        "raw_cmd_delta": [
            float(value)
            for value in np.asarray(candidate["raw_cmd"]) - np.asarray(actual["raw_cmd"])
        ],
        "horizon_state_linf": float(np.max(np.abs(state_delta))),
        "horizon_control_linf": float(np.max(np.abs(control_delta))),
        "horizon_s_linf_m": float(np.max(np.abs(state_delta[:, 4]))),
        "horizon_v_linf_m_s": float(np.max(np.abs(state_delta[:, 3]))),
        "horizon_omega_linf_rad_s": float(np.max(np.abs(state_delta[:, 5]))),
    }


def replay_checkpoints(
    pairs: Sequence[SnapshotPair],
    checkpoints: Mapping[str, SnapshotPair],
    replay_config: Mapping[str, Any],
    replay_executable: Path,
) -> Tuple[Dict[str, Any], List[str]]:
    failures: List[str] = []
    output: Dict[str, Any] = {}
    with tempfile.TemporaryDirectory(prefix="spmpc_g4_snapshot_replay_") as temp_dir:
        request_path = Path(temp_dir) / "request.txt"
        result_path = Path(temp_dir) / "result.txt"
        write_replay_request(
            request_path, pairs, checkpoints, replay_config
        )
        completed = subprocess.run(
            [
                str(replay_executable),
                "--input",
                str(request_path),
                "--output",
                str(result_path),
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                "C++ G4 snapshot replay failed (exit {}): {}".format(
                    completed.returncode, completed.stderr.strip()
                )
            )
        replayed = read_replay_result(result_path)

    if not replayed["success"]:
        failures.append(
            "C++ sequential replay failed at pair {}: {}".format(
                replayed["failed_pair"], replayed["detail"]
            )
        )
    direction_codes = {"longitudinal": 1, "lateral": 2}
    phases = [float(value) for value in replay_config["phases_rad"]]
    if not phases:
        raise RuntimeError("replay phases_rad must not be empty")
    for direction, pair in checkpoints.items():
        checkpoint = replayed["checkpoints"].get(pair.index)
        if checkpoint is None:
            failures.append(
                "{} checkpoint is absent from C++ replay output".format(direction)
            )
            continue
        if checkpoint["direction"] != direction_codes[direction]:
            failures.append("{} checkpoint direction code mismatch".format(direction))
            continue
        counterfactuals = checkpoint["counterfactuals"]
        if len(counterfactuals) != len(phases) + 1:
            failures.append(
                "{} counterfactual count mismatch: {} vs {}".format(
                    direction, len(counterfactuals), len(phases) + 1
                )
            )
            continue

        actual_online_states, actual_online_controls = online_horizon_arrays(
            pair.horizon
        )
        actual_status = int(checkpoint["actual"]["status"])
        actual = _solver_result(checkpoint["actual"], pair.snapshot)
        actual_state_error = float(
            np.max(np.abs(np.asarray(actual["states"]) - actual_online_states))
        )
        actual_control_error = float(
            np.max(np.abs(np.asarray(actual["controls"]) - actual_online_controls))
        )
        actual_u0_error = float(
            np.max(np.abs(np.asarray(actual["u0"]) - actual_online_controls[0]))
        )
        if actual_status != 0:
            failures.append(
                "{} actual replay status={}".format(direction, actual_status)
            )
        if actual_u0_error > float(
            replay_config["actual_first_action_abs_tolerance"]
        ):
            failures.append(
                "{} actual first-action error {:.3g} exceeds tolerance".format(
                    direction, actual_u0_error
                )
            )
        if max(actual_state_error, actual_control_error) > float(
            replay_config["actual_horizon_abs_tolerance"]
        ):
            failures.append(
                "{} actual horizon error {:.3g} exceeds tolerance".format(
                    direction, max(actual_state_error, actual_control_error)
                )
            )

        zero_solution = counterfactuals[0]
        zero_status = int(zero_solution["status"])
        zero = _solver_result(zero_solution, pair.snapshot)
        zero_metrics = branch_metrics(actual, zero)
        if zero_status != 0:
            failures.append(
                "{} zero-state replay status={}".format(direction, zero_status)
            )
        if max(
            zero_metrics["u0_linf"], zero_metrics["horizon_state_linf"]
        ) < float(replay_config["actual_zero_detect_tolerance"]):
            failures.append(
                "{} actual/zero branches are numerically indistinguishable".format(
                    direction
                )
            )

        phase_modals = _checkpoint_modals(
            pair.snapshot, direction, replay_config
        )[1:]
        phase_results = []
        for phase, modal, solution in zip(
            phases, phase_modals, counterfactuals[1:]
        ):
            status = int(solution["status"])
            result = _solver_result(solution, pair.snapshot)
            phase_results.append((phase, modal, status, result))
            if status != 0:
                failures.append(
                    "{} phase {:.6f} replay status={}".format(
                        direction, phase, status
                    )
                )
        phase_u0 = np.asarray([item[3]["u0"] for item in phase_results])
        phase_states = np.asarray([item[3]["states"] for item in phase_results])
        phase_controls = np.asarray(
            [item[3]["controls"] for item in phase_results]
        )
        u0_range = np.ptp(phase_u0, axis=0)
        state_range = np.ptp(phase_states, axis=0)
        control_range = np.ptp(phase_controls, axis=0)
        phase_metrics = {
            "u0_range": [float(value) for value in u0_range],
            "u0_linf_range": float(np.max(np.abs(u0_range))),
            "horizon_state_linf_range": float(np.max(np.abs(state_range))),
            "horizon_control_linf_range": float(np.max(np.abs(control_range))),
            "horizon_s_linf_range_m": float(np.max(np.abs(state_range[:, 4]))),
            "horizon_v_linf_range_m_s": float(np.max(np.abs(state_range[:, 3]))),
            "horizon_omega_linf_range_rad_s": float(
                np.max(np.abs(state_range[:, 5]))
            ),
        }
        phase_detected = (
            phase_metrics["u0_linf_range"]
            >= float(replay_config["phase_first_action_detect_tolerance"])
            or phase_metrics["horizon_state_linf_range"]
            >= float(replay_config["phase_horizon_detect_tolerance"])
        )
        if not phase_detected:
            failures.append(
                "{} four-phase sensitivity is below frozen tolerance".format(
                    direction
                )
            )

        branches = {
            "zero_state": {
                "status": zero_status,
                "metrics_vs_actual": zero_metrics,
            },
            "constructed_phases": [
                {
                    "phase_rad": float(phase),
                    "modal_state": [float(value) for value in modal],
                    "status": int(status),
                    "u0": [float(value) for value in result["u0"]],
                    "raw_cmd": [float(value) for value in result["raw_cmd"]],
                }
                for phase, modal, status, result in phase_results
            ],
        }
        output[direction] = {
            "pair_index": pair.index,
            "stamp": float(pair.snapshot.header.stamp.to_sec()),
            "sigma": float(pair.snapshot.s0 / pair.snapshot.reference_length),
            "robot_v_m_s": float(pair.snapshot.robot_v),
            "robot_omega_rad_s": float(pair.snapshot.robot_omega),
            "online_u0": [float(value) for value in actual_online_controls[0]],
            "actual_replay": {
                "status": actual_status,
                "first_action_abs_error": actual_u0_error,
                "horizon_state_abs_error": actual_state_error,
                "horizon_control_abs_error": actual_control_error,
                "u0": [float(value) for value in actual["u0"]],
                "raw_cmd": [float(value) for value in actual["raw_cmd"]],
            },
            "branches": branches,
            "phase_metrics": phase_metrics,
            "phase_sensitivity_pass": phase_detected,
        }
    return output, failures


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise RuntimeError("cannot write empty trajectory CSV")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g3-root", required=True)
    parser.add_argument("--path-file", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--codegen-json", required=True)
    parser.add_argument("--replay-executable")
    parser.add_argument("--allow-incomplete-g3", action="store_true", help="tool smoke only; report remains FAIL")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    g3_root = Path(args.g3_root).expanduser().resolve()
    path_file = Path(args.path_file).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    codegen_json = Path(args.codegen_json).expanduser().resolve()
    configured_replay = (
        Path(args.replay_executable) if args.replay_executable else None
    )
    replay_executable = replay_executable_path(configured_replay)
    out_dir.mkdir(parents=True, exist_ok=True)

    failures: List[str] = []
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, Mapping) or config.get("schema_version") != 1:
        raise RuntimeError("G4 config must be schema_version 1")
    boundaries = config["trajectory"]["zone_boundaries_sigma"]
    if len(boundaries) != 6 or any(
        float(boundaries[index]) >= float(boundaries[index + 1]) for index in range(5)
    ) or not math.isclose(float(boundaries[0]), 0.0) or not math.isclose(float(boundaries[-1]), 1.0):
        raise RuntimeError("zone_boundaries_sigma must be six strictly increasing values from 0 to 1")

    g3_report_path = g3_root / "G3_EFFICACY_REPORT.json"
    if not g3_report_path.is_file():
        failures.append("G3_EFFICACY_REPORT.json is missing")
        if not args.allow_incomplete_g3:
            print("[G4] G3 report is missing; finish Row 08 first", file=sys.stderr)
            return 2
        postflights = sorted(g3_root.glob("DEV_G3_*_g3_postflight.json"))
        dataset = []
        for postflight in postflights:
            item = json.loads(postflight.read_text(encoding="utf-8"))
            dataset.append(
                {
                    "row": str(item.get("row", "")),
                    "bag": item.get("bag"),
                    "bag_sha256": item.get("bag_sha256"),
                    "postflight": str(postflight),
                }
            )
        g3_report: Mapping[str, Any] = {"status": "INCOMPLETE", "dataset_index": dataset}
    else:
        g3_report = json.loads(g3_report_path.read_text(encoding="utf-8"))
        if g3_report.get("status") != "PASS":
            failures.append("G3 efficacy report is not PASS")

    source_row = str(config["checkpoint_source_row"])
    source_item = next(
        (item for item in g3_report.get("dataset_index", []) if str(item.get("row")) == source_row),
        None,
    )
    if source_item is None:
        raise RuntimeError("G3 dataset lacks checkpoint source row {}".format(source_row))
    bag_path = Path(str(source_item["bag"])).expanduser().resolve()
    postflight_path = Path(str(source_item["postflight"])).expanduser().resolve()
    if not bag_path.is_file() or not postflight_path.is_file():
        raise RuntimeError("source-row bag/postflight is missing")
    postflight = json.loads(postflight_path.read_text(encoding="utf-8"))
    if postflight.get("status") != "PASS":
        failures.append("checkpoint source row postflight is not PASS")
    if source_item.get("bag_sha256") and sha256_file(bag_path) != source_item.get("bag_sha256"):
        raise RuntimeError("source-row bag hash mismatch")

    trajectory_rows, trajectory_summary = extract_trajectory(
        bag_path,
        load_path_xy(path_file),
        float(postflight["motion_start_sec"]),
        float(postflight["first_arrival_sec"]),
        config["trajectory"],
    )
    trajectory_csv = out_dir / "G4_SOURCE_ROW_TRAJECTORY.csv"
    write_csv(trajectory_csv, trajectory_rows)
    if trajectory_summary["tf_gap_over_limit_count"]:
        failures.append("trajectory TF interpolation gap exceeded frozen limit")
    if trajectory_summary["cross_track_p95_m"] > float(
        config["trajectory"]["maximum_cross_track_p95_m"]
    ):
        failures.append("trajectory cross-track P95 exceeded frozen limit")
    if trajectory_summary["sigma_end"] < 0.95:
        failures.append("trajectory did not reach 95% projected progress")

    # Replay starts at the first valid planner snapshot, not first motion.  This
    # restores the capsule's pre-motion SQP/dual history before the motion-window
    # checkpoint is forked.  Checkpoint selection itself still excludes startup
    # through the frozen sigma bounds below.
    pairs = read_snapshot_pairs(
        bag_path,
        -math.inf,
        float(postflight["motion_end_sec"]),
    )
    validate_codegen_snapshot_contract(codegen_json, pairs[0].snapshot)
    replay_libraries = linked_replay_libraries(replay_executable)
    linked_slosh = replay_libraries[
        "libacados_ocp_solver_spmpc_slosh.so"
    ]
    if linked_slosh.parent != codegen_json.parent:
        raise RuntimeError(
            "codegen JSON does not belong to the generated slosh solver "
            "linked by the C++ replay executable"
        )
    checkpoints = select_checkpoints(pairs, config["checkpoint_selection"])
    checkpoint_identity = {
        direction: {
            "pair_index": pair.index,
            "stamp": float(pair.snapshot.header.stamp.to_sec()),
            "sigma": float(pair.snapshot.s0 / pair.snapshot.reference_length),
            "robot_v_m_s": float(pair.snapshot.robot_v),
            "robot_omega_rad_s": float(pair.snapshot.robot_omega),
            "selection_score": (
                abs(float(pair.horizon.a[0]))
                if direction == "longitudinal"
                else abs(float(pair.snapshot.robot_v) * float(pair.snapshot.robot_omega))
            ),
        }
        for direction, pair in checkpoints.items()
    }
    checkpoint_path = out_dir / "G4_CHECKPOINTS.json"
    checkpoint_path.write_text(
        json.dumps(checkpoint_identity, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    replay, replay_failures = replay_checkpoints(
        pairs,
        checkpoints,
        config["replay"],
        replay_executable,
    )
    failures.extend(replay_failures)

    report = {
        "schema_version": 1,
        "report_type": "G4_TRAJECTORY_REPLAY",
        "protocol": config["protocol"],
        "scope": "DEVELOPMENT_TOOLCHAIN_GATE",
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "bindings": {
            "g3_report": str(g3_report_path),
            "g3_report_sha256": sha256_file(g3_report_path) if g3_report_path.is_file() else "",
            "source_row": source_row,
            "source_bag": str(bag_path),
            "source_bag_sha256": sha256_file(bag_path),
            "source_postflight": str(postflight_path),
            "source_postflight_sha256": sha256_file(postflight_path),
            "path_file": str(path_file),
            "path_sha256": sha256_file(path_file),
            "config": str(config_path),
            "config_sha256": sha256_file(config_path),
            "codegen_json": str(codegen_json),
            "codegen_json_sha256": sha256_file(codegen_json),
            "replay_executable": str(replay_executable),
            "replay_executable_sha256": sha256_file(replay_executable),
            "replay_libraries": {
                name: {
                    "path": str(path),
                    "sha256": sha256_file(path),
                }
                for name, path in sorted(replay_libraries.items())
            },
        },
        "trajectory": trajectory_summary,
        "trajectory_csv": str(trajectory_csv),
        "trajectory_csv_sha256": sha256_file(trajectory_csv),
        "checkpoints": checkpoint_identity,
        "checkpoints_file": str(checkpoint_path),
        "checkpoints_sha256": sha256_file(checkpoint_path),
        "replay": replay,
        "limitations": [
            "Constructed four-phase branches prove optimizer sensitivity, not true liquid phase accuracy.",
            "Actual/zero branches compare optimized raw actions; the complete downstream execution chain is not counterfactually replayed.",
            "H0 G3 development zones cannot be relabelled as formal H1 zones.",
        ],
    }
    report_path = out_dir / "G4_TRAJECTORY_REPLAY_REPORT.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (out_dir / "G4_TRAJECTORY_REPLAY_REPORT.json.sha256").write_text(
        "{}  {}\n".format(sha256_file(report_path), report_path), encoding="utf-8"
    )
    print("[G4] {}".format(report["status"]))
    print("  report={}".format(report_path))
    print("  source_row={} pairs={}".format(source_row, len(pairs)))
    for direction, item in replay.items():
        print(
            "  {}: sigma={:.3f}, actual_u0_err={:.3g}, phase_u0_range={:.3g}".format(
                direction,
                item["sigma"],
                item["actual_replay"]["first_action_abs_error"],
                item["phase_metrics"]["u0_linf_range"],
            )
        )
    for failure in failures:
        print("  FAIL: {}".format(failure), file=sys.stderr)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
