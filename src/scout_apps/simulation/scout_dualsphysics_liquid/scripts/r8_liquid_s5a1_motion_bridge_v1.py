#!/usr/bin/env python3
"""Pure-Python S5A1 odometry-to-DualSPHysics motion bridge.

This module contains only deterministic rigid-body mathematics and a strict
``solver_path.csv`` codec.  It never reads a ROS bag, executes a solver,
accesses a GPU, uses the network, or writes an output file.  A later exporter
may call these functions only after S5A0 has sealed the selected bag and
validated its provenance.

The frozen convention is ``R8-LIQUID-HANDOFF-ABI-v3``:

* primary time comes only from ``/odom.header.stamp``;
* the frozen base-to-container mount is applied before taking the relative
  transform, so the first canonical sample is exactly identity;
* position is linearly interpolated and orientation uses normalized SLERP;
* ``solver_path.csv`` contains exactly seven numeric fields ordered as
  time, relative XYZ, continuous intrinsic-ZYX yaw, pitch, roll (degrees).
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import os
import stat
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence


sys.dont_write_bytecode = True

ABI_ID = "R8-LIQUID-HANDOFF-ABI-v3"
SOURCE_DOMAIN = "SIM_R7_EXECUTED_GAZEBO_MOTION"
PRIMARY_TIME_SOURCE = "/odom.header.stamp"
POSE_SOURCE = "/odom.pose.pose"
ALLOWED_MOTION_SIGNALS = frozenset(
    {
        "/odom.header.stamp",
        "/odom.pose.pose.position",
        "/odom.pose.pose.orientation",
    }
)
FORBIDDEN_SIGNAL_MARKERS = (
    "/cmd_vel",
    "path",
    "plan",
    "goal",
    "h_proxy",
    "h_modal",
    "status",
    "fixedprofile",
)
SOLVER_PATH_COMMENT = (
    "# t_s,x_rel_m,y_rel_m,z_rel_m,ang1_yaw_z_deg,"
    "ang2_pitch_y_deg,ang3_roll_x_deg"
)
MAX_CSV_BYTES = 16 * 1024 * 1024
QUATERNION_MIN_NORM = 1.0e-12
IDENTITY_TOLERANCE = 1.0e-12
DEFAULT_GIMBAL_LOCK_COS_THRESHOLD = 1.0e-8


class MotionBridgeError(ValueError):
    """Invalid motion, unsafe provenance, or non-canonical CSV."""


@dataclass(frozen=True)
class OdomPose:
    """One pose driven by the numeric value of ``/odom.header.stamp``."""

    header_t_s: float
    x_m: float
    y_m: float
    z_m: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class RigidPose:
    """A constant transform, used for the frozen base-to-container mount."""

    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


@dataclass(frozen=True)
class MotionSample:
    """Canonical relative pose on the zero-based S5A1 time axis."""

    t_s: float
    x_rel_m: float
    y_rel_m: float
    z_rel_m: float
    qx_rel: float
    qy_rel: float
    qz_rel: float
    qw_rel: float


@dataclass(frozen=True)
class SolverPathRow:
    """Exactly the seven numeric fields accepted by ``mvpathfile``."""

    t_s: float
    x_rel_m: float
    y_rel_m: float
    z_rel_m: float
    ang1_yaw_z_deg: float
    ang2_pitch_y_deg: float
    ang3_roll_x_deg: float

    def numeric_fields(self) -> tuple[float, float, float, float, float, float, float]:
        return (
            self.t_s,
            self.x_rel_m,
            self.y_rel_m,
            self.z_rel_m,
            self.ang1_yaw_z_deg,
            self.ang2_pitch_y_deg,
            self.ang3_roll_x_deg,
        )


@dataclass(frozen=True)
class RoundTripReport:
    sample_count: int
    max_position_error_m: float
    max_quaternion_angular_distance_rad: float
    max_rotation_matrix_angular_distance_rad: float


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Matrix3 = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MotionBridgeError(f"{label} is not a real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise MotionBridgeError(f"{label} is non-finite")
    return converted


def _finite_vector(values: Sequence[object], count: int, label: str) -> tuple[float, ...]:
    if len(values) != count:
        raise MotionBridgeError(f"{label} must contain exactly {count} values")
    return tuple(_finite_number(value, f"{label}[{index}]") for index, value in enumerate(values))


def _dot_quaternion(left: Quaternion, right: Quaternion) -> float:
    return sum(a * b for a, b in zip(left, right))


def normalize_quaternion(quaternion: Sequence[object]) -> Quaternion:
    """Return a finite unit quaternion, rejecting a degenerate input."""

    qx, qy, qz, qw = _finite_vector(quaternion, 4, "quaternion")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < QUATERNION_MIN_NORM:
        raise MotionBridgeError("quaternion norm is zero or too small")
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def _continuous_quaternion(previous: Quaternion | None, current: Quaternion) -> Quaternion:
    if previous is not None and _dot_quaternion(previous, current) < 0.0:
        return tuple(-component for component in current)  # type: ignore[return-value]
    return current


def quaternion_conjugate(quaternion: Sequence[object]) -> Quaternion:
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    return (-qx, -qy, -qz, qw)


def quaternion_multiply(left: Sequence[object], right: Sequence[object]) -> Quaternion:
    """Hamilton product for active rotations in XYZW storage order."""

    ax, ay, az, aw = normalize_quaternion(left)
    bx, by, bz, bw = normalize_quaternion(right)
    return normalize_quaternion(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_vector(quaternion: Sequence[object], vector: Sequence[object]) -> Vector3:
    """Rotate a vector without treating the vector as a unit quaternion."""

    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = _finite_vector(vector, 3, "vector")
    # Equivalent to q * (v, 0) * conjugate(q), without normalizing v.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def quaternion_slerp(left: Sequence[object], right: Sequence[object], fraction: object) -> Quaternion:
    """Shortest-arc normalized SLERP with deterministic sign continuity."""

    amount = _finite_number(fraction, "SLERP fraction")
    if amount < 0.0 or amount > 1.0:
        raise MotionBridgeError("SLERP fraction is outside [0, 1]")
    first = normalize_quaternion(left)
    second = normalize_quaternion(right)
    dot = _dot_quaternion(first, second)
    if dot < 0.0:
        second = tuple(-component for component in second)  # type: ignore[assignment]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        blended = tuple(a + amount * (b - a) for a, b in zip(first, second))
        return normalize_quaternion(blended)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    left_scale = math.sin((1.0 - amount) * theta) / sin_theta
    right_scale = math.sin(amount * theta) / sin_theta
    return normalize_quaternion(
        tuple(left_scale * a + right_scale * b for a, b in zip(first, second))
    )


def intrinsic_zyx_to_quaternion(yaw_z_rad: object, pitch_y_rad: object, roll_x_rad: object) -> Quaternion:
    """Convert intrinsic ZYX yaw/pitch/roll to an active unit quaternion."""

    yaw = _finite_number(yaw_z_rad, "yaw")
    pitch = _finite_number(pitch_y_rad, "pitch")
    roll = _finite_number(roll_x_rad, "roll")
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    return normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def quaternion_to_intrinsic_zyx(
    quaternion: Sequence[object],
    *,
    gimbal_lock_cos_threshold: float = DEFAULT_GIMBAL_LOCK_COS_THRESHOLD,
) -> tuple[float, float, float]:
    """Return yaw, pitch, roll and fail closed at intrinsic-ZYX gimbal lock."""

    threshold = _finite_number(gimbal_lock_cos_threshold, "gimbal-lock threshold")
    if threshold <= 0.0 or threshold >= 1.0:
        raise MotionBridgeError("gimbal-lock threshold must be in (0, 1)")
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sin_pitch)
    if abs(math.cos(pitch)) <= threshold:
        raise MotionBridgeError("intrinsic ZYX conversion reaches gimbal-lock")
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    return (yaw, pitch, roll)


def quaternion_to_rotation_matrix(quaternion: Sequence[object]) -> Matrix3:
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    return (
        (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ),
        (
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ),
        (
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ),
    )


def intrinsic_zyx_to_rotation_matrix(yaw_z_rad: object, pitch_y_rad: object, roll_x_rad: object) -> Matrix3:
    return quaternion_to_rotation_matrix(
        intrinsic_zyx_to_quaternion(yaw_z_rad, pitch_y_rad, roll_x_rad)
    )


def quaternion_angular_distance(left: Sequence[object], right: Sequence[object]) -> float:
    first = normalize_quaternion(left)
    second = normalize_quaternion(right)
    cosine = max(-1.0, min(1.0, abs(_dot_quaternion(first, second))))
    return 2.0 * math.acos(cosine)


def rotation_matrix_angular_distance(left: Matrix3, right: Matrix3) -> float:
    """Geodesic SO(3) distance, stable for rotations close to identity."""

    for name, matrix in (("left matrix", left), ("right matrix", right)):
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            raise MotionBridgeError(f"{name} must be 3 by 3")
        for row in matrix:
            for value in row:
                _finite_number(value, name)
    relative = tuple(
        tuple(
            sum(left[axis][row] * right[axis][column] for axis in range(3))
            for column in range(3)
        )
        for row in range(3)
    )
    cosine = max(
        -1.0,
        min(1.0, (relative[0][0] + relative[1][1] + relative[2][2] - 1.0) / 2.0),
    )
    sine = 0.5 * math.sqrt(
        (relative[2][1] - relative[1][2]) ** 2
        + (relative[0][2] - relative[2][0]) ** 2
        + (relative[1][0] - relative[0][1]) ** 2
    )
    return math.atan2(sine, cosine)


def validate_odom_only_provenance(
    *,
    primary_time_source: str,
    pose_source: str,
    consumed_motion_signals: Iterable[str],
) -> dict[str, object]:
    """Prove that no command/path/goal/proxy/modal/status signal drove motion."""

    if primary_time_source != PRIMARY_TIME_SOURCE:
        raise MotionBridgeError("primary time provenance is not /odom.header.stamp")
    if pose_source != POSE_SOURCE:
        raise MotionBridgeError("pose provenance is not /odom.pose.pose")
    consumed = tuple(consumed_motion_signals)
    if not consumed or any(not isinstance(item, str) or not item for item in consumed):
        raise MotionBridgeError("consumed motion-signal provenance is invalid")
    lowered = tuple(item.casefold() for item in consumed)
    forbidden = sorted(
        item
        for item, normalized in zip(consumed, lowered)
        if any(marker in normalized for marker in FORBIDDEN_SIGNAL_MARKERS)
    )
    if forbidden:
        raise MotionBridgeError(f"forbidden-signal provenance detected: {forbidden}")
    unknown = sorted(set(consumed) - ALLOWED_MOTION_SIGNALS)
    missing = sorted(ALLOWED_MOTION_SIGNALS - set(consumed))
    if unknown or missing or len(consumed) != len(ALLOWED_MOTION_SIGNALS):
        raise MotionBridgeError(
            f"motion-signal provenance is not the closed odom-only set; missing={missing}, unknown={unknown}"
        )
    return {
        "abi": ABI_ID,
        "source_domain": SOURCE_DOMAIN,
        "primary_time_source": primary_time_source,
        "pose_source": pose_source,
        "consumed_motion_signals": sorted(consumed),
        "forbidden_signal_consumed": False,
    }


def _validate_strict_times(times: Sequence[float], *, max_gap_s: float | None, label: str) -> None:
    if not times:
        raise MotionBridgeError(f"{label} is empty")
    maximum_gap = None
    if max_gap_s is not None:
        maximum_gap = _finite_number(max_gap_s, "maximum gap")
        if maximum_gap <= 0.0:
            raise MotionBridgeError("maximum gap must be positive")
    previous = _finite_number(times[0], f"{label}[0] time")
    for index, raw in enumerate(times[1:], start=1):
        current = _finite_number(raw, f"{label}[{index}] time")
        if current == previous:
            raise MotionBridgeError(f"{label} contains duplicate time at index {index}")
        if current < previous:
            raise MotionBridgeError(f"{label} contains time backtrack at index {index}")
        gap = current - previous
        if maximum_gap is not None and gap > maximum_gap:
            raise MotionBridgeError(
                f"{label} gap {gap:.17g}s exceeds maximum {maximum_gap:.17g}s at index {index}"
            )
        previous = current


def make_relative_motion(
    odom_poses: Sequence[OdomPose],
    *,
    maximum_gap_s: float,
    base_to_container: RigidPose = RigidPose(),
) -> tuple[MotionSample, ...]:
    """Apply the frozen mount and compute ``inverse(T0) * T(t)``."""

    if not odom_poses:
        raise MotionBridgeError("odom pose sequence is empty")
    _validate_strict_times(
        [pose.header_t_s for pose in odom_poses], max_gap_s=maximum_gap_s, label="odom sequence"
    )
    mount_position = _finite_vector(
        (base_to_container.x_m, base_to_container.y_m, base_to_container.z_m), 3, "mount position"
    )
    mount_orientation = normalize_quaternion(
        (base_to_container.qx, base_to_container.qy, base_to_container.qz, base_to_container.qw)
    )

    absolute_positions: list[Vector3] = []
    absolute_orientations: list[Quaternion] = []
    previous_base: Quaternion | None = None
    previous_container: Quaternion | None = None
    for index, pose in enumerate(odom_poses):
        base_position = _finite_vector((pose.x_m, pose.y_m, pose.z_m), 3, f"odom position {index}")
        base_orientation = _continuous_quaternion(
            previous_base,
            normalize_quaternion((pose.qx, pose.qy, pose.qz, pose.qw)),
        )
        previous_base = base_orientation
        rotated_mount = rotate_vector(base_orientation, mount_position)
        container_position = tuple(
            base_position[axis] + rotated_mount[axis] for axis in range(3)
        )
        container_orientation = _continuous_quaternion(
            previous_container, quaternion_multiply(base_orientation, mount_orientation)
        )
        previous_container = container_orientation
        absolute_positions.append(container_position)  # type: ignore[arg-type]
        absolute_orientations.append(container_orientation)

    start_position = absolute_positions[0]
    start_inverse_orientation = quaternion_conjugate(absolute_orientations[0])
    start_time = _finite_number(odom_poses[0].header_t_s, "first odom time")
    relative: list[MotionSample] = []
    previous_relative: Quaternion | None = None
    for pose, absolute_position, absolute_orientation in zip(
        odom_poses, absolute_positions, absolute_orientations
    ):
        translated = tuple(
            absolute_position[axis] - start_position[axis] for axis in range(3)
        )
        relative_position = rotate_vector(start_inverse_orientation, translated)
        relative_orientation = _continuous_quaternion(
            previous_relative,
            quaternion_multiply(start_inverse_orientation, absolute_orientation),
        )
        previous_relative = relative_orientation
        relative.append(
            MotionSample(
                t_s=float(pose.header_t_s) - start_time,
                x_rel_m=relative_position[0],
                y_rel_m=relative_position[1],
                z_rel_m=relative_position[2],
                qx_rel=relative_orientation[0],
                qy_rel=relative_orientation[1],
                qz_rel=relative_orientation[2],
                qw_rel=relative_orientation[3],
            )
        )

    # Eliminate harmless floating residue and make the ABI identity exact.
    relative[0] = MotionSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    return _canonical_motion_samples(relative, require_identity=True)


def _canonical_motion_samples(
    samples: Sequence[MotionSample], *, require_identity: bool
) -> tuple[MotionSample, ...]:
    if not samples:
        raise MotionBridgeError("canonical motion is empty")
    _validate_strict_times([sample.t_s for sample in samples], max_gap_s=None, label="canonical motion")
    canonical: list[MotionSample] = []
    previous: Quaternion | None = None
    for index, sample in enumerate(samples):
        position = _finite_vector(
            (sample.x_rel_m, sample.y_rel_m, sample.z_rel_m), 3, f"motion position {index}"
        )
        orientation = _continuous_quaternion(
            previous,
            normalize_quaternion((sample.qx_rel, sample.qy_rel, sample.qz_rel, sample.qw_rel)),
        )
        previous = orientation
        canonical.append(
            MotionSample(
                _finite_number(sample.t_s, f"motion time {index}"),
                position[0],
                position[1],
                position[2],
                orientation[0],
                orientation[1],
                orientation[2],
                orientation[3],
            )
        )
    if require_identity:
        first = canonical[0]
        if first.t_s != 0.0:
            raise MotionBridgeError("canonical motion does not start at t_s=0")
        if max(abs(first.x_rel_m), abs(first.y_rel_m), abs(first.z_rel_m)) > IDENTITY_TOLERANCE:
            raise MotionBridgeError("T_rel(t0) translation is not identity")
        if quaternion_angular_distance(
            (first.qx_rel, first.qy_rel, first.qz_rel, first.qw_rel), (0.0, 0.0, 0.0, 1.0)
        ) > IDENTITY_TOLERANCE:
            raise MotionBridgeError("T_rel(t0) rotation is not identity")
    return tuple(canonical)


def interpolate_motion_sample(samples: Sequence[MotionSample], query_t_s: object) -> MotionSample:
    """Interpolate canonical motion with linear XYZ and normalized SLERP."""

    canonical = _canonical_motion_samples(samples, require_identity=True)
    query = _finite_number(query_t_s, "query time")
    if query < canonical[0].t_s or query > canonical[-1].t_s:
        raise MotionBridgeError("query time would extrapolate canonical motion")
    if query == canonical[-1].t_s:
        return canonical[-1]
    for left, right in zip(canonical, canonical[1:]):
        if query == left.t_s:
            return left
        if left.t_s < query < right.t_s:
            fraction = (query - left.t_s) / (right.t_s - left.t_s)
            orientation = quaternion_slerp(
                (left.qx_rel, left.qy_rel, left.qz_rel, left.qw_rel),
                (right.qx_rel, right.qy_rel, right.qz_rel, right.qw_rel),
                fraction,
            )
            return MotionSample(
                query,
                left.x_rel_m + fraction * (right.x_rel_m - left.x_rel_m),
                left.y_rel_m + fraction * (right.y_rel_m - left.y_rel_m),
                left.z_rel_m + fraction * (right.z_rel_m - left.z_rel_m),
                orientation[0],
                orientation[1],
                orientation[2],
                orientation[3],
            )
    raise MotionBridgeError("query time is not bracketed by canonical motion")


def resample_motion(
    samples: Sequence[MotionSample], grid_times_s: Sequence[float]
) -> tuple[MotionSample, ...]:
    """Resample without smoothing, filtering, path projection, or extrapolation."""

    canonical = _canonical_motion_samples(samples, require_identity=True)
    _validate_strict_times(grid_times_s, max_gap_s=None, label="interpolation grid")
    if grid_times_s[0] != 0.0:
        raise MotionBridgeError("interpolation grid does not start at t_s=0")
    if grid_times_s[-1] > canonical[-1].t_s:
        raise MotionBridgeError("interpolation grid would extrapolate canonical motion")
    return tuple(interpolate_motion_sample(canonical, time_s) for time_s in grid_times_s)


def _unwrap_yaw(yaw_rad: float, previous_unwrapped_rad: float | None) -> float:
    if previous_unwrapped_rad is None:
        return yaw_rad
    candidate = yaw_rad
    while candidate - previous_unwrapped_rad > math.pi:
        candidate -= 2.0 * math.pi
    while candidate - previous_unwrapped_rad <= -math.pi:
        candidate += 2.0 * math.pi
    return candidate


def build_solver_path(
    samples: Sequence[MotionSample],
    *,
    gimbal_lock_cos_threshold: float = DEFAULT_GIMBAL_LOCK_COS_THRESHOLD,
) -> tuple[SolverPathRow, ...]:
    """Convert canonical quaternions to continuous intrinsic-ZYX solver rows."""

    canonical = _canonical_motion_samples(samples, require_identity=True)
    rows: list[SolverPathRow] = []
    previous_yaw: float | None = None
    for sample in canonical:
        yaw, pitch, roll = quaternion_to_intrinsic_zyx(
            (sample.qx_rel, sample.qy_rel, sample.qz_rel, sample.qw_rel),
            gimbal_lock_cos_threshold=gimbal_lock_cos_threshold,
        )
        unwrapped_yaw = _unwrap_yaw(yaw, previous_yaw)
        previous_yaw = unwrapped_yaw
        rows.append(
            SolverPathRow(
                sample.t_s,
                sample.x_rel_m,
                sample.y_rel_m,
                sample.z_rel_m,
                math.degrees(unwrapped_yaw),
                math.degrees(pitch),
                math.degrees(roll),
            )
        )
    return validate_solver_path(rows, gimbal_lock_cos_threshold=gimbal_lock_cos_threshold)


def validate_solver_path(
    rows: Sequence[SolverPathRow],
    *,
    maximum_gap_s: float | None = None,
    gimbal_lock_cos_threshold: float = DEFAULT_GIMBAL_LOCK_COS_THRESHOLD,
) -> tuple[SolverPathRow, ...]:
    """Validate the closed seven-field solver path and first-row identity."""

    if not rows:
        raise MotionBridgeError("solver path is empty")
    _validate_strict_times([row.t_s for row in rows], max_gap_s=maximum_gap_s, label="solver path")
    threshold = _finite_number(gimbal_lock_cos_threshold, "gimbal-lock threshold")
    if threshold <= 0.0 or threshold >= 1.0:
        raise MotionBridgeError("gimbal-lock threshold must be in (0, 1)")
    validated: list[SolverPathRow] = []
    for index, row in enumerate(rows):
        fields = _finite_vector(row.numeric_fields(), 7, f"solver path row {index}")
        pitch_rad = math.radians(fields[5])
        if abs(fields[5]) > 90.0:
            raise MotionBridgeError(f"solver path pitch is outside [-90, 90] at row {index}")
        if abs(math.cos(pitch_rad)) <= threshold:
            raise MotionBridgeError(f"solver path reaches gimbal-lock at row {index}")
        validated.append(SolverPathRow(*fields))
    first = validated[0]
    if first.t_s != 0.0:
        raise MotionBridgeError("solver path does not start at t_s=0")
    if max(abs(value) for value in first.numeric_fields()[1:]) > IDENTITY_TOLERANCE:
        raise MotionBridgeError("solver path first row is not identity")
    return tuple(validated)


def render_solver_path_csv(rows: Sequence[SolverPathRow]) -> str:
    """Render one exact comment plus rows containing exactly seven numbers."""

    validated = validate_solver_path(rows)
    output = io.StringIO(newline="")
    output.write(SOLVER_PATH_COMMENT + "\n")
    writer = csv.writer(output, lineterminator="\n")
    for row in validated:
        writer.writerow(format(value, ".17g") for value in row.numeric_fields())
    return output.getvalue()


def parse_solver_path_csv(
    text: str,
    *,
    maximum_gap_s: float | None = None,
) -> tuple[SolverPathRow, ...]:
    """Parse fail-closed CSV: optional exact comment, then seven numeric fields."""

    if not isinstance(text, str):
        raise MotionBridgeError("solver path CSV must be text")
    encoded = text.encode("utf-8")
    if not encoded or len(encoded) > MAX_CSV_BYTES:
        raise MotionBridgeError("solver path CSV size is outside the bound")
    if "\x00" in text:
        raise MotionBridgeError("solver path CSV contains NUL")
    rows: list[SolverPathRow] = []
    comment_seen = False
    data_seen = False
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        if not raw_line:
            raise MotionBridgeError(f"solver path CSV contains blank line {line_number}")
        if raw_line.startswith("#"):
            if data_seen or comment_seen or raw_line != SOLVER_PATH_COMMENT:
                raise MotionBridgeError(f"solver path CSV has invalid comment at line {line_number}")
            comment_seen = True
            continue
        data_seen = True
        try:
            fields = next(csv.reader([raw_line], strict=True))
        except (csv.Error, StopIteration) as error:
            raise MotionBridgeError(f"solver path CSV syntax error at line {line_number}") from error
        if len(fields) != 7:
            raise MotionBridgeError(
                f"solver path CSV line {line_number} has {len(fields)} fields instead of 7"
            )
        try:
            numbers = tuple(float(field) for field in fields)
        except ValueError as error:
            raise MotionBridgeError(f"solver path CSV has non-numeric field at line {line_number}") from error
        if any(not math.isfinite(value) for value in numbers):
            raise MotionBridgeError(f"solver path CSV has non-finite field at line {line_number}")
        rows.append(SolverPathRow(*numbers))
    return validate_solver_path(rows, maximum_gap_s=maximum_gap_s)


def _interpolate_solver_row(rows: Sequence[SolverPathRow], query_t_s: float) -> SolverPathRow:
    validated = validate_solver_path(rows)
    query = _finite_number(query_t_s, "solver query time")
    if query < validated[0].t_s or query > validated[-1].t_s:
        raise MotionBridgeError("solver query time would extrapolate path")
    if query == validated[-1].t_s:
        return validated[-1]
    for left, right in zip(validated, validated[1:]):
        if query == left.t_s:
            return left
        if left.t_s < query < right.t_s:
            fraction = (query - left.t_s) / (right.t_s - left.t_s)
            values = tuple(
                left.numeric_fields()[index]
                + fraction * (right.numeric_fields()[index] - left.numeric_fields()[index])
                for index in range(1, 7)
            )
            return SolverPathRow(query, *values)
    raise MotionBridgeError("solver query time is not bracketed by path")


def evaluate_solver_round_trip(
    motion: Sequence[MotionSample],
    rows: Sequence[SolverPathRow],
    *,
    query_times_s: Sequence[float] | None = None,
) -> RoundTripReport:
    """Compare quaternion motion against solver Euler reconstruction.

    At non-row query times this intentionally models DualSPHysics' linear
    interpolation of the seven path fields and compares it with the frozen
    canonical linear-position/SLERP contract.
    """

    canonical = _canonical_motion_samples(motion, require_identity=True)
    validated = validate_solver_path(rows)
    if canonical[0].t_s != validated[0].t_s or canonical[-1].t_s != validated[-1].t_s:
        raise MotionBridgeError("motion and solver path time domains differ")
    queries = tuple(sample.t_s for sample in canonical) if query_times_s is None else tuple(query_times_s)
    _validate_strict_times(queries, max_gap_s=None, label="round-trip query grid")
    if queries[0] < canonical[0].t_s or queries[-1] > canonical[-1].t_s:
        raise MotionBridgeError("round-trip query grid would extrapolate")

    maximum_position = 0.0
    maximum_quaternion_angle = 0.0
    maximum_matrix_angle = 0.0
    for query in queries:
        expected = interpolate_motion_sample(canonical, query)
        actual = _interpolate_solver_row(validated, query)
        actual_quaternion = intrinsic_zyx_to_quaternion(
            math.radians(actual.ang1_yaw_z_deg),
            math.radians(actual.ang2_pitch_y_deg),
            math.radians(actual.ang3_roll_x_deg),
        )
        position_error = math.sqrt(
            (expected.x_rel_m - actual.x_rel_m) ** 2
            + (expected.y_rel_m - actual.y_rel_m) ** 2
            + (expected.z_rel_m - actual.z_rel_m) ** 2
        )
        expected_quaternion = (
            expected.qx_rel,
            expected.qy_rel,
            expected.qz_rel,
            expected.qw_rel,
        )
        quaternion_angle = quaternion_angular_distance(expected_quaternion, actual_quaternion)
        matrix_angle = rotation_matrix_angular_distance(
            quaternion_to_rotation_matrix(expected_quaternion),
            intrinsic_zyx_to_rotation_matrix(
                math.radians(actual.ang1_yaw_z_deg),
                math.radians(actual.ang2_pitch_y_deg),
                math.radians(actual.ang3_roll_x_deg),
            ),
        )
        maximum_position = max(maximum_position, position_error)
        maximum_quaternion_angle = max(maximum_quaternion_angle, quaternion_angle)
        maximum_matrix_angle = max(maximum_matrix_angle, matrix_angle)
    return RoundTripReport(
        len(queries), maximum_position, maximum_quaternion_angle, maximum_matrix_angle
    )


def self_check() -> dict[str, object]:
    provenance = validate_odom_only_provenance(
        primary_time_source=PRIMARY_TIME_SOURCE,
        pose_source=POSE_SOURCE,
        consumed_motion_signals=sorted(ALLOWED_MOTION_SIGNALS),
    )
    start = intrinsic_zyx_to_quaternion(math.radians(170.0), 0.0, 0.0)
    finish = intrinsic_zyx_to_quaternion(math.radians(-170.0), 0.0, 0.0)
    odom = (
        OdomPose(100.0, 3.0, -2.0, 0.5, *start),
        OdomPose(102.0, 3.2, -1.8, 0.5, *finish),
    )
    motion = make_relative_motion(odom, maximum_gap_s=2.0)
    motion = resample_motion(motion, (0.0, 1.0, 2.0))
    rows = build_solver_path(motion)
    reparsed = parse_solver_path_csv(render_solver_path_csv(rows), maximum_gap_s=1.0)
    report = evaluate_solver_round_trip(motion, reparsed, query_times_s=(0.0, 0.5, 1.0, 1.5, 2.0))
    if report.max_position_error_m > 1.0e-12:
        raise MotionBridgeError("self-check position round-trip drifted")
    if report.max_quaternion_angular_distance_rad > 1.0e-7:
        raise MotionBridgeError("self-check quaternion round-trip drifted")
    if report.max_rotation_matrix_angular_distance_rad > 1.0e-7:
        raise MotionBridgeError("self-check rotation-matrix round-trip drifted")
    return {
        "status": "PASS_S5A1_MOTION_BRIDGE_V1_SELF_CHECK",
        "abi": ABI_ID,
        "provenance": provenance,
        "round_trip": asdict(report),
        "bag_read": False,
        "files_written": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "sudo_used": False,
        "network_used": False,
    }


def _read_bounded_regular_text(path: Path) -> str:
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise MotionBridgeError("solver path input is not a single-link regular file")
    if not 1 <= metadata.st_size <= MAX_CSV_BYTES:
        raise MotionBridgeError("solver path input size is outside the bound")
    with path.open("r", encoding="utf-8", newline="") as stream:
        return stream.read(MAX_CSV_BYTES + 1)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="run deterministic in-memory checks")
    validate_parser = subparsers.add_parser("validate-solver-path", help="validate one CSV read-only")
    validate_parser.add_argument("path", type=Path)
    validate_parser.add_argument("--maximum-gap-s", type=float, required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            result = self_check()
        else:
            rows = parse_solver_path_csv(
                _read_bounded_regular_text(arguments.path),
                maximum_gap_s=arguments.maximum_gap_s,
            )
            result = {
                "status": "PASS_S5A1_SOLVER_PATH_V1_VALIDATION",
                "path": str(arguments.path),
                "row_count": len(rows),
                "files_written": False,
                "solver_executed": False,
                "gpu_exposed": False,
                "network_used": False,
            }
    except (MotionBridgeError, OSError, UnicodeError) as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
