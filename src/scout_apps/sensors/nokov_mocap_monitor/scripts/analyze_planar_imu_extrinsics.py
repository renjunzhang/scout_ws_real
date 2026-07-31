#!/usr/bin/env python3
"""Estimate and validate the planar IMU-to-tube extrinsic from mocap bags.

The tuning/validation split is intentionally hard-coded by role:

* compact r02 selects the Tracker0-to-base planar yaw and the candidate
  IMU-to-base yaw / IMU-to-tube XY lever arm;
* remote r03 and planar r03 evaluate that locked candidate without feeding
  their results back into the candidate.

This is a planar development calibration, not a six-axis hand-eye calibration.
It uses centered local-polynomial processing for the mocap truth and therefore
must not be confused with the online causal filter implementation.
"""

import argparse
import csv
import hashlib
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
import rosbag


DEFAULT_TUNING_BAG = (
    "/home/geist/slosh_bags/real/20260729_mocap_imu_calib/"
    "imu_mocap_compact_r02bash_133636.bag"
)
DEFAULT_REMOTE_BAG = (
    "/home/geist/slosh_bags/real/20260729_mocap_imu_spin/"
    "imu_filter_remote_r03_142438.bag"
)
DEFAULT_PLANAR_BAG = (
    "/home/geist/slosh_bags/real/20260729_mocap_imu_calib/"
    "imu_mocap_planar_r03_153448.bag"
)
DEFAULT_OUT_DIR = (
    "/home/geist/slosh_bags/real/20260731_imu_planar_extrinsic_analysis"
)


@dataclass
class RawBagData:
    name: str
    path: Path
    bag_start: float
    bag_end: float
    imu: np.ndarray
    mocap: np.ndarray
    odom: np.ndarray
    events: List[Tuple[float, str]]
    sha256: str


@dataclass
class PreparedData:
    name: str
    time: np.ndarray
    accel_imu_x: np.ndarray
    accel_imu_y: np.ndarray
    truth_x: np.ndarray
    truth_y: np.ndarray
    omega: np.ndarray
    alpha: np.ndarray
    speed: np.ndarray
    motion_mask: np.ndarray
    bias: np.ndarray
    static_window: Tuple[float, float]
    sample_count_imu: int
    sample_count_mocap: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tuning-bag", default=DEFAULT_TUNING_BAG)
    parser.add_argument(
        "--validation-bag",
        action="append",
        default=[],
        metavar="NAME=PATH",
        help="May be repeated. Defaults to remote_r03 and planar_r03.",
    )
    parser.add_argument("--output-dir", default=DEFAULT_OUT_DIR)
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument(
        "--mocap-topic", default="/vrpn_client_node/Tracker0/pose"
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--segment-topic", default="/mocap_imu_calib/segment")
    parser.add_argument("--primary-window-sec", type=float, default=0.51)
    parser.add_argument(
        "--sensitivity-windows-sec", default="0.31,0.51,0.71,1.01"
    )
    parser.add_argument("--imu-mocap-lag-sec", type=float, default=0.015)
    parser.add_argument("--gravity-mps2", type=float, default=9.8)
    parser.add_argument("--gyro-scale", type=float, default=1.001773)
    parser.add_argument("--gyro-offset-radps", type=float, default=-0.000297)
    parser.add_argument("--nominal-yaw-deg", type=float, default=0.0)
    parser.add_argument("--nominal-lever-x-m", type=float, default=-0.100)
    parser.add_argument("--nominal-lever-y-m", type=float, default=0.045)
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def message_time(message, bag_time) -> float:
    if hasattr(message, "header"):
        value = message.header.stamp.to_sec()
        if value > 0.0:
            return value
    return bag_time.to_sec()


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def load_bag(
    name: str,
    path: Path,
    imu_topic: str,
    mocap_topic: str,
    odom_topic: str,
    segment_topic: str,
) -> RawBagData:
    if not path.is_file():
        raise FileNotFoundError(str(path))

    imu_rows = []
    mocap_rows = []
    odom_rows = []
    events = []
    topics = [imu_topic, mocap_topic, odom_topic, segment_topic]
    with rosbag.Bag(str(path), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, bag_time in bag.read_messages(topics=topics):
            if topic == imu_topic:
                stamp = message_time(message, bag_time)
                q = message.orientation
                a = message.linear_acceleration
                w = message.angular_velocity
                imu_rows.append(
                    (
                        stamp,
                        q.x,
                        q.y,
                        q.z,
                        q.w,
                        a.x,
                        a.y,
                        a.z,
                        w.z,
                    )
                )
            elif topic == mocap_topic:
                stamp = message_time(message, bag_time)
                p = message.pose.position
                mocap_rows.append(
                    (
                        stamp,
                        p.x,
                        p.y,
                        quaternion_yaw(message.pose.orientation),
                    )
                )
            elif topic == odom_topic:
                stamp = message_time(message, bag_time)
                odom_rows.append(
                    (
                        stamp,
                        message.twist.twist.linear.x,
                        message.twist.twist.angular.z,
                    )
                )
            elif topic == segment_topic:
                events.append((bag_time.to_sec(), message.data))

    if len(imu_rows) < 100 or len(mocap_rows) < 100 or len(odom_rows) < 100:
        raise RuntimeError(
            "{} lacks required data: imu={}, mocap={}, odom={}".format(
                path, len(imu_rows), len(mocap_rows), len(odom_rows)
            )
        )

    imu = np.asarray(imu_rows, dtype=float)
    mocap = np.asarray(mocap_rows, dtype=float)
    mocap[:, 3] = np.unwrap(mocap[:, 3])
    odom = np.asarray(odom_rows, dtype=float)
    return RawBagData(
        name=name,
        path=path,
        bag_start=bag_start,
        bag_end=bag_end,
        imu=imu,
        mocap=mocap,
        odom=odom,
        events=events,
        sha256=sha256_file(path),
    )


def odd_window_samples(window_sec: float, sample_rate_hz: float) -> int:
    count = max(5, int(round(window_sec * sample_rate_hz)))
    if count % 2 == 0:
        count += 1
    return count


def local_polynomial_filter(
    values: np.ndarray,
    sample_rate_hz: float,
    window_sec: float,
    derivative: int,
    polynomial_order: int = 3,
) -> np.ndarray:
    count = odd_window_samples(window_sec, sample_rate_hz)
    if count >= len(values):
        raise ValueError("filter window is longer than the signal")
    half = count // 2
    offsets = np.arange(-half, half + 1, dtype=float) / sample_rate_hz
    design = np.vander(offsets, polynomial_order + 1, increasing=True)
    coefficients = (
        np.linalg.pinv(design)[derivative]
        * math.factorial(derivative)
    )
    return np.convolve(values, coefficients[::-1], mode="same")


def event_time(raw: RawBagData, prefix: str) -> float:
    values = [stamp for stamp, value in raw.events if value.startswith(prefix)]
    if not values:
        raise KeyError(prefix)
    return values[0]


def static_window(raw: RawBagData) -> Tuple[float, float]:
    try:
        start = event_time(raw, "START|static_pre")
        end = event_time(raw, "END|static_pre")
        low = start + 2.0
        high = min(end - 2.0, low + 13.0)
    except KeyError:
        # remote_r03 was recorded with a hardware remote and has no segment
        # markers. Its protocol explicitly requires a static head and tail.
        low = max(raw.bag_start, raw.imu[0, 0]) + 2.0
        high = low + 8.0
    if high - low < 5.0:
        raise RuntimeError("insufficient static bias interval in {}".format(raw.name))
    return low, high


def gravity_removed_acceleration(
    raw: RawBagData, gravity_mps2: float
) -> Tuple[np.ndarray, np.ndarray]:
    quaternion = raw.imu[:, 1:5].copy()
    norms = np.linalg.norm(quaternion, axis=1)
    if np.any(~np.isfinite(norms)) or np.any(norms < 1e-9):
        raise RuntimeError("invalid IMU quaternion in {}".format(raw.name))
    quaternion /= norms[:, None]
    qx, qy, qz, qw = quaternion.T
    gravity_x = gravity_mps2 * 2.0 * (qx * qz - qw * qy)
    gravity_y = gravity_mps2 * 2.0 * (qy * qz + qw * qx)
    gravity_z = gravity_mps2 * (
        1.0 - 2.0 * (qx * qx + qy * qy)
    )
    acceleration = raw.imu[:, 5:8] - np.column_stack(
        (gravity_x, gravity_y, gravity_z)
    )
    low, high = static_window(raw)
    mask = (raw.imu[:, 0] >= low) & (raw.imu[:, 0] <= high)
    if np.count_nonzero(mask) < 100:
        raise RuntimeError("too few static IMU samples in {}".format(raw.name))
    bias = np.median(acceleration[mask], axis=0)
    return acceleration - bias, bias


def mocap_kinematics(
    raw: RawBagData,
    window_sec: float,
    sample_rate_hz: float = 100.0,
) -> Dict[str, np.ndarray]:
    margin = max(1.0, window_sec)
    time = np.arange(
        raw.mocap[0, 0] + margin,
        raw.mocap[-1, 0] - margin,
        1.0 / sample_rate_hz,
    )
    x = np.interp(time, raw.mocap[:, 0], raw.mocap[:, 1])
    y = np.interp(time, raw.mocap[:, 0], raw.mocap[:, 2])
    yaw = np.interp(time, raw.mocap[:, 0], raw.mocap[:, 3])
    return {
        "time": time,
        "x": x,
        "y": y,
        "yaw": local_polynomial_filter(
            yaw, sample_rate_hz, window_sec, 0
        ),
        "vx": local_polynomial_filter(
            x, sample_rate_hz, window_sec, 1
        ),
        "vy": local_polynomial_filter(
            y, sample_rate_hz, window_sec, 1
        ),
        "ax": local_polynomial_filter(
            x, sample_rate_hz, window_sec, 2
        ),
        "ay": local_polynomial_filter(
            y, sample_rate_hz, window_sec, 2
        ),
        "omega": local_polynomial_filter(
            yaw, sample_rate_hz, window_sec, 1
        ),
        "alpha": local_polynomial_filter(
            yaw, sample_rate_hz, window_sec, 2
        ),
    }


def wrap_angle(values: np.ndarray) -> np.ndarray:
    return (values + math.pi) % (2.0 * math.pi) - math.pi


def estimate_tracker_to_base_yaw(
    raw: RawBagData, window_sec: float
) -> Dict[str, float]:
    kin = mocap_kinematics(raw, window_sec)
    time = kin["time"]
    odom_v = np.interp(time, raw.odom[:, 0], raw.odom[:, 1])
    odom_w = np.interp(time, raw.odom[:, 0], raw.odom[:, 2])
    speed = np.hypot(kin["vx"], kin["vy"])
    mask = (
        (np.abs(odom_v) > 0.025)
        & (np.abs(odom_w) < 0.04)
        & (speed > 0.025)
        & (speed < 0.35)
    )
    motion_heading = np.arctan2(kin["vy"], kin["vx"])
    motion_heading -= (odom_v < 0.0) * math.pi
    differences = wrap_angle(motion_heading - kin["yaw"])
    if np.count_nonzero(mask) < 100:
        raise RuntimeError(
            "{} has too few near-straight samples for Tracker0 yaw".format(
                raw.name
            )
        )
    median = float(np.median(differences[mask]))
    residual_from_median = wrap_angle(differences - median)
    mask &= np.abs(residual_from_median) < math.radians(5.0)
    selected = differences[mask]
    mean = math.atan2(np.mean(np.sin(selected)), np.mean(np.cos(selected)))
    residual = wrap_angle(selected - mean)
    return {
        "yaw_base_minus_tracker_rad": mean,
        "yaw_base_minus_tracker_deg": math.degrees(mean),
        "residual_std_deg": math.degrees(float(np.std(residual))),
        "residual_p95_abs_deg": math.degrees(
            float(np.percentile(np.abs(residual), 95.0))
        ),
        "selected_sample_count": int(len(selected)),
    }


def prepare_data(
    raw: RawBagData,
    tracker_to_base_yaw_rad: float,
    window_sec: float,
    imu_mocap_lag_sec: float,
    gravity_mps2: float,
    gyro_scale: float,
    gyro_offset_radps: float,
) -> PreparedData:
    acceleration, bias = gravity_removed_acceleration(raw, gravity_mps2)
    kin = mocap_kinematics(raw, window_sec)
    sample_rate_hz = 50.0
    margin = max(1.0, window_sec)
    start = max(raw.imu[0, 0], raw.mocap[0, 0]) + margin
    end = min(raw.imu[-1, 0], raw.mocap[-1, 0]) - margin
    time = np.arange(start, end, 1.0 / sample_rate_hz)
    accel_x = np.interp(time, raw.imu[:, 0], acceleration[:, 0])
    accel_y = np.interp(time, raw.imu[:, 0], acceleration[:, 1])
    accel_x = local_polynomial_filter(
        accel_x, sample_rate_hz, window_sec, 0
    )
    accel_y = local_polynomial_filter(
        accel_y, sample_rate_hz, window_sec, 0
    )

    truth_time = time - imu_mocap_lag_sec
    world_ax = np.interp(truth_time, kin["time"], kin["ax"])
    world_ay = np.interp(truth_time, kin["time"], kin["ay"])
    base_yaw = (
        np.interp(truth_time, kin["time"], kin["yaw"])
        + tracker_to_base_yaw_rad
    )
    cosine = np.cos(base_yaw)
    sine = np.sin(base_yaw)
    truth_x = cosine * world_ax + sine * world_ay
    truth_y = -sine * world_ax + cosine * world_ay
    omega = np.interp(truth_time, kin["time"], kin["omega"])
    alpha = np.interp(truth_time, kin["time"], kin["alpha"])
    speed = np.hypot(
        np.interp(truth_time, kin["time"], kin["vx"]),
        np.interp(truth_time, kin["time"], kin["vy"]),
    )

    truth_norm = np.hypot(truth_x, truth_y)
    motion_mask = (
        (truth_norm > 0.015)
        | (np.abs(alpha) > 0.03)
        | (np.abs(omega) > 0.08)
        | (speed > 0.02)
    ) & (truth_norm < 3.0)
    try:
        sequence_start = event_time(raw, "SEQUENCE_START")
        sequence_end = event_time(raw, "SEQUENCE_END")
        motion_mask &= (time >= sequence_start + margin) & (
            time <= sequence_end - margin
        )
    except KeyError:
        # remote_r03 has no automatic-sequence markers. Its outer grid margin
        # and the kinematic activity gate above remain the only time bounds.
        pass

    # The affine gyro calibration is not needed by the centered physical fit,
    # which uses mocap omega/alpha. Evaluate it here so the report still binds
    # the exact candidate constants and catches invalid data.
    calibrated_gyro = np.interp(
        time,
        raw.imu[:, 0],
        gyro_scale * raw.imu[:, 8] + gyro_offset_radps,
    )
    if np.any(~np.isfinite(calibrated_gyro)):
        raise RuntimeError("non-finite calibrated gyro in {}".format(raw.name))

    return PreparedData(
        name=raw.name,
        time=time,
        accel_imu_x=accel_x,
        accel_imu_y=accel_y,
        truth_x=truth_x,
        truth_y=truth_y,
        omega=omega,
        alpha=alpha,
        speed=speed,
        motion_mask=motion_mask,
        bias=bias,
        static_window=static_window(raw),
        sample_count_imu=len(raw.imu),
        sample_count_mocap=len(raw.mocap),
    )


def rotated_acceleration(
    data: PreparedData, imu_to_base_yaw_rad: float
) -> np.ndarray:
    cosine = math.cos(imu_to_base_yaw_rad)
    sine = math.sin(imu_to_base_yaw_rad)
    return np.column_stack(
        (
            cosine * data.accel_imu_x - sine * data.accel_imu_y,
            sine * data.accel_imu_x + cosine * data.accel_imu_y,
        )
    )


def lever_design(data: PreparedData, indices: np.ndarray) -> np.ndarray:
    omega_squared = data.omega[indices] ** 2
    alpha = data.alpha[indices]
    design = np.zeros((2 * len(indices), 2), dtype=float)
    design[0::2, 0] = -omega_squared
    design[0::2, 1] = -alpha
    design[1::2, 0] = alpha
    design[1::2, 1] = -omega_squared
    return design


def solve_lever_for_yaw(
    data: PreparedData, imu_to_base_yaw_rad: float
) -> Tuple[np.ndarray, float, float]:
    indices = np.flatnonzero(data.motion_mask)
    rotated = rotated_acceleration(data, imu_to_base_yaw_rad)
    truth = np.column_stack((data.truth_x, data.truth_y))
    delta = truth[indices] - rotated[indices]
    target = np.empty(2 * len(indices), dtype=float)
    target[0::2] = delta[:, 0]
    target[1::2] = delta[:, 1]
    design = lever_design(data, indices)
    lever, _, _, singular_values = np.linalg.lstsq(
        design, target, rcond=None
    )
    residual = target - design.dot(lever)
    condition = (
        float(singular_values[0] / singular_values[-1])
        if singular_values[-1] > 0.0
        else float("inf")
    )
    return lever, float(np.mean(residual ** 2)), condition


def fit_extrinsic(data: PreparedData) -> Dict[str, float]:
    best = None
    # A deterministic 0.05 degree profile is deliberately used instead of a
    # black-box optimizer, making the one-dimensional fit auditable.
    for yaw_deg in np.linspace(-15.0, 15.0, 601):
        yaw_rad = math.radians(float(yaw_deg))
        lever, mse, condition = solve_lever_for_yaw(data, yaw_rad)
        if best is None or mse < best[0]:
            best = (mse, yaw_deg, lever, condition)
    mse, yaw_deg, lever, condition = best
    return {
        "imu_to_base_yaw_deg": float(yaw_deg),
        "imu_to_base_yaw_rad": math.radians(float(yaw_deg)),
        "lever_arm_imu_to_tube_x_m": float(lever[0]),
        "lever_arm_imu_to_tube_y_m": float(lever[1]),
        "joint_rmse_mps2": math.sqrt(float(mse)),
        "lever_design_condition_number": float(condition),
        "motion_sample_count": int(np.count_nonzero(data.motion_mask)),
    }


def predict_target_acceleration(
    data: PreparedData,
    imu_to_base_yaw_rad: float,
    lever_x_m: float,
    lever_y_m: float,
) -> np.ndarray:
    predicted = rotated_acceleration(data, imu_to_base_yaw_rad)
    omega_squared = data.omega ** 2
    predicted[:, 0] += -data.alpha * lever_y_m - omega_squared * lever_x_m
    predicted[:, 1] += data.alpha * lever_x_m - omega_squared * lever_y_m
    return predicted


def finite_correlation(left: np.ndarray, right: np.ndarray) -> float:
    if len(left) < 2 or np.std(left) < 1e-12 or np.std(right) < 1e-12:
        return float("nan")
    return float(np.corrcoef(left, right)[0, 1])


def evaluate_prediction(
    data: PreparedData, prediction: np.ndarray
) -> Dict[str, object]:
    truth = np.column_stack((data.truth_x, data.truth_y))
    mask = data.motion_mask
    error = prediction[mask] - truth[mask]
    rmse = np.sqrt(np.mean(error ** 2, axis=0))
    mae = np.mean(np.abs(error), axis=0)
    correlation = [
        finite_correlation(prediction[mask, axis], truth[mask, axis])
        for axis in (0, 1)
    ]
    return {
        "motion_sample_count": int(np.count_nonzero(mask)),
        "rmse_x_mps2": float(rmse[0]),
        "rmse_y_mps2": float(rmse[1]),
        "mae_x_mps2": float(mae[0]),
        "mae_y_mps2": float(mae[1]),
        "correlation_x": correlation[0],
        "correlation_y": correlation[1],
        "joint_rmse_mps2": math.sqrt(float(np.mean(error ** 2))),
    }


def candidate_parameters(fit: Dict[str, float]) -> Tuple[float, float, float]:
    return (
        fit["imu_to_base_yaw_rad"],
        fit["lever_arm_imu_to_tube_x_m"],
        fit["lever_arm_imu_to_tube_y_m"],
    )


def parse_validation_bags(values: Sequence[str]) -> List[Tuple[str, Path]]:
    if not values:
        return [
            ("remote_r03", Path(DEFAULT_REMOTE_BAG)),
            ("planar_r03", Path(DEFAULT_PLANAR_BAG)),
        ]
    parsed = []
    for value in values:
        if "=" not in value:
            raise ValueError("validation bag must be NAME=PATH: {}".format(value))
        name, path = value.split("=", 1)
        if not name or not path:
            raise ValueError("validation bag must be NAME=PATH: {}".format(value))
        parsed.append((name, Path(path)))
    return parsed


def relative_improvement(reference: float, candidate: float) -> float:
    return (reference - candidate) / reference if reference > 0.0 else 0.0


def write_metrics_csv(path: Path, report: Dict[str, object]) -> None:
    fields = [
        "dataset",
        "role",
        "configuration",
        "motion_sample_count",
        "rmse_x_mps2",
        "rmse_y_mps2",
        "mae_x_mps2",
        "mae_y_mps2",
        "correlation_x",
        "correlation_y",
        "joint_rmse_mps2",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for name, dataset in report["datasets"].items():
            for configuration in ("no_lever", "nominal", "locked_candidate"):
                row = {
                    "dataset": name,
                    "role": dataset["role"],
                    "configuration": configuration,
                }
                row.update(dataset["metrics"][configuration])
                writer.writerow(row)


def write_summary(path: Path, report: Dict[str, object]) -> None:
    candidate = report["locked_tuning_candidate"]
    decision = report["decision"]
    lines = [
        "# Planar IMU extrinsic analysis summary",
        "",
        "- Tuning data: `compact_r02` only.",
        "- Validation data: `remote_r03`, `planar_r03`; not fed back into the candidate.",
        "- Tracker0-to-base yaw from tuning: `{:.4f} deg`.".format(
            report["tracker_to_base_yaw"]["tuning"][
                "yaw_base_minus_tracker_deg"
            ]
        ),
        "- Locked candidate: yaw `{:.4f} deg`, IMU-to-tube lever "
        "`[{:.5f}, {:.5f}] m`.".format(
            candidate["imu_to_base_yaw_deg"],
            candidate["lever_arm_imu_to_tube_x_m"],
            candidate["lever_arm_imu_to_tube_y_m"],
        ),
        "- Runtime recommendation: `{}`.".format(
            decision["runtime_extrinsic_action"]
        ),
        "- G2S recommendation: `{}`.".format(decision["g2s_recommendation"]),
        "",
        "## Locked-candidate validation",
        "",
        "| Dataset | Nominal joint RMSE | Candidate joint RMSE | Relative improvement |",
        "| --- | ---: | ---: | ---: |",
    ]
    for name, dataset in report["datasets"].items():
        if dataset["role"] != "held_out_validation":
            continue
        nominal = dataset["metrics"]["nominal"]["joint_rmse_mps2"]
        candidate_rmse = dataset["metrics"]["locked_candidate"][
            "joint_rmse_mps2"
        ]
        improvement = relative_improvement(nominal, candidate_rmse)
        lines.append(
            "| `{}` | {:.6f} | {:.6f} | {:+.3%} |".format(
                name, nominal, candidate_rmse, improvement
            )
        )
    lines.extend(
        [
            "",
            "## Decision",
            "",
            decision["reason"],
            "",
            "This is a planar external-calibration result. It does not establish a full six-axis IMU hand-eye calibration.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    sensitivity_windows = [
        float(value.strip())
        for value in args.sensitivity_windows_sec.split(",")
        if value.strip()
    ]
    if args.primary_window_sec not in sensitivity_windows:
        sensitivity_windows.append(args.primary_window_sec)
        sensitivity_windows.sort()
    if any(value <= 0.0 for value in sensitivity_windows):
        raise ValueError("all processing windows must be positive")

    validation_paths = parse_validation_bags(args.validation_bag)
    raw_data = {}
    raw_data["compact_r02"] = load_bag(
        "compact_r02",
        Path(args.tuning_bag),
        args.imu_topic,
        args.mocap_topic,
        args.odom_topic,
        args.segment_topic,
    )
    for name, path in validation_paths:
        raw_data[name] = load_bag(
            name,
            path,
            args.imu_topic,
            args.mocap_topic,
            args.odom_topic,
            args.segment_topic,
        )

    tracker_yaw = {}
    for name, raw in raw_data.items():
        tracker_yaw[name] = estimate_tracker_to_base_yaw(
            raw, args.primary_window_sec
        )
    frozen_tracker_yaw = tracker_yaw["compact_r02"][
        "yaw_base_minus_tracker_rad"
    ]

    tuning_sensitivity = []
    tuning_primary = None
    for window_sec in sensitivity_windows:
        prepared = prepare_data(
            raw_data["compact_r02"],
            frozen_tracker_yaw,
            window_sec,
            args.imu_mocap_lag_sec,
            args.gravity_mps2,
            args.gyro_scale,
            args.gyro_offset_radps,
        )
        fit = fit_extrinsic(prepared)
        fit["processing_window_sec"] = window_sec
        tuning_sensitivity.append(fit)
        if abs(window_sec - args.primary_window_sec) < 1e-9:
            tuning_primary = prepared
            locked_candidate = fit
    if tuning_primary is None:
        raise RuntimeError("primary tuning window was not evaluated")

    nominal = (
        math.radians(args.nominal_yaw_deg),
        args.nominal_lever_x_m,
        args.nominal_lever_y_m,
    )
    candidate = candidate_parameters(locked_candidate)
    no_lever = (math.radians(args.nominal_yaw_deg), 0.0, 0.0)

    prepared_data = {"compact_r02": tuning_primary}
    for name, _ in validation_paths:
        prepared_data[name] = prepare_data(
            raw_data[name],
            frozen_tracker_yaw,
            args.primary_window_sec,
            args.imu_mocap_lag_sec,
            args.gravity_mps2,
            args.gyro_scale,
            args.gyro_offset_radps,
        )

    dataset_reports = {}
    independent_fits = {}
    validation_improvements = []
    for name, data in prepared_data.items():
        metrics = {}
        for label, parameters in (
            ("no_lever", no_lever),
            ("nominal", nominal),
            ("locked_candidate", candidate),
        ):
            metrics[label] = evaluate_prediction(
                data, predict_target_acceleration(data, *parameters)
            )
        role = "tuning" if name == "compact_r02" else "held_out_validation"
        dataset_reports[name] = {
            "role": role,
            "bag_path": str(raw_data[name].path),
            "bag_sha256": raw_data[name].sha256,
            "bag_duration_sec": raw_data[name].bag_end
            - raw_data[name].bag_start,
            "imu_sample_count": data.sample_count_imu,
            "mocap_sample_count": data.sample_count_mocap,
            "static_bias_window_sec": list(data.static_window),
            "gravity_removed_static_bias_mps2": data.bias.tolist(),
            "metrics": metrics,
        }
        independent_fits[name] = fit_extrinsic(data)
        if role == "held_out_validation":
            validation_improvements.append(
                relative_improvement(
                    metrics["nominal"]["joint_rmse_mps2"],
                    metrics["locked_candidate"]["joint_rmse_mps2"],
                )
            )

    consistent_validation_improvement = all(
        value > 0.0 for value in validation_improvements
    )
    material_validation_improvement = (
        consistent_validation_improvement
        and min(validation_improvements) >= 0.02
    )
    if material_validation_improvement:
        runtime_action = "REVIEW_CANDIDATE_FOR_RUNTIME_UPDATE"
        g2s_recommendation = "HOLD_UNTIL_RUNTIME_REPLAY_WITH_CANDIDATE"
        reason = (
            "The r02-only candidate improved every held-out validation bag by "
            "at least 2%. Review physical geometry and repeat the no-command "
            "runtime replay before changing TF/configuration."
        )
    else:
        runtime_action = "KEEP_NOMINAL_PLANAR_EXTRINSIC"
        g2s_recommendation = "GO_AFTER_FREEZING_THIS_KEEP_NOMINAL_REPORT"
        reason = (
            "The locked r02 candidate did not produce a consistent material "
            "improvement on both held-out bags: one improved and one worsened. "
            "The observed changes were small relative to the 10% G2S "
            "source-selection margin. Keep the current nominal planar extrinsic; "
            "do not claim a full six-axis calibration."
        )

    report = {
        "schema": "imu_planar_extrinsic_analysis_v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "policy": {
            "tuning_dataset": "compact_r02",
            "held_out_validation_datasets": [
                name for name, _ in validation_paths
            ],
            "validation_feedback_into_candidate": False,
            "full_six_axis_calibration": False,
            "processing": "centered cubic local polynomial",
            "primary_window_sec": args.primary_window_sec,
            "imu_truth_pairing": "mocap(t_imu - lag)",
        },
        "fixed_processing_parameters": {
            "imu_mocap_lag_sec": args.imu_mocap_lag_sec,
            "gravity_mps2": args.gravity_mps2,
            "gyro_scale": args.gyro_scale,
            "gyro_offset_radps": args.gyro_offset_radps,
        },
        "nominal_runtime_extrinsic": {
            "imu_to_base_yaw_deg": args.nominal_yaw_deg,
            "lever_arm_imu_to_tube_x_m": args.nominal_lever_x_m,
            "lever_arm_imu_to_tube_y_m": args.nominal_lever_y_m,
        },
        "tracker_to_base_yaw": {
            "frozen_source": "compact_r02",
            "tuning": tracker_yaw["compact_r02"],
            "validation_diagnostics_not_fed_back": {
                name: tracker_yaw[name] for name, _ in validation_paths
            },
        },
        "locked_tuning_candidate": locked_candidate,
        "tuning_window_sensitivity": tuning_sensitivity,
        "independent_dataset_fits_diagnostic_only": independent_fits,
        "datasets": dataset_reports,
        "decision": {
            "runtime_extrinsic_action": runtime_action,
            "g2s_recommendation": g2s_recommendation,
            "held_out_relative_joint_rmse_improvements": {
                name: validation_improvements[index]
                for index, (name, _) in enumerate(validation_paths)
            },
            "consistent_validation_improvement": consistent_validation_improvement,
            "material_improvement_threshold": 0.02,
            "g2s_source_margin": 0.10,
            "reason": reason,
        },
    }

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "imu_planar_extrinsic_report.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_metrics_csv(output_dir / "imu_planar_extrinsic_metrics.csv", report)
    write_summary(output_dir / "imu_planar_extrinsic_summary.md", report)
    report_sha = sha256_file(report_path)
    (output_dir / "imu_planar_extrinsic_report.sha256").write_text(
        "{}  {}\n".format(report_sha, report_path.name), encoding="utf-8"
    )

    print("report={}".format(report_path))
    print("report_sha256={}".format(report_sha))
    print(
        "tracker0_to_base_yaw_deg={:.6f}".format(
            tracker_yaw["compact_r02"]["yaw_base_minus_tracker_deg"]
        )
    )
    print(
        "locked_candidate_yaw_deg={:.6f} lever_xy_m=[{:.6f},{:.6f}]".format(
            locked_candidate["imu_to_base_yaw_deg"],
            locked_candidate["lever_arm_imu_to_tube_x_m"],
            locked_candidate["lever_arm_imu_to_tube_y_m"],
        )
    )
    for name, improvement in zip(
        [name for name, _ in validation_paths], validation_improvements
    ):
        print("{}_relative_improvement={:+.6%}".format(name, improvement))
    print("runtime_extrinsic_action={}".format(runtime_action))
    print("g2s_recommendation={}".format(g2s_recommendation))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
