#!/usr/bin/env python3
"""Build descriptive G3R2 horizon/observer/native-RGB alignment ledgers.

This analyzer is deliberately offline: it reads ROS1 bags directly, never
connects to a ROS master, and never publishes a command.  RGB samples retain
their original image source stamps.  The script never interpolates RGB; it
instead evaluates each native RGB event against the continuous-time location
of every overlapping solved horizon.

The script fits separate ``delta=0`` and ``delta=+12 ms`` projections from the
two Bsmooth training bags using native RGB events.  ``delta=0`` remains the
primary result and ``delta=+12 ms`` is emitted as a sensitivity result.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import hashlib
import json
import math
import statistics
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import rosbag

from horizon_liquid_replay import (
    ModalParameters,
    ModalState,
    ObserverAnchor,
    ObserverInputSample,
    PlannedControl,
    ReplayContractError,
    cubic_hermite_q,
    replay_observer_inputs,
    replay_planned_controls,
    sample_observer_replay,
    sample_planned_replay,
)


HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"
SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"
SELECTION_TOPIC = "/spmpc/debug/slosh_observer_selection"
OBSERVER_TOPIC = "/spmpc/debug/slosh_observer_imu"
RGB_TOPIC = "/liquid/measurement"

EXPECTED_PROTOCOL = "G3R2_robot_only_W5_S10_paired_confirmation_v1"
EXPECTED_HORIZON_STATUS = "B_slosh_ACADOS_OK"
EXPECTED_VARIANT = "B_slosh"
EXPECTED_SOURCE_CODE = 2

DEFAULT_ROOT = Path(
    "/data/a/slosh_bags/real/"
    "20260801_spmpc_g3r2_w5s10_paired_confirmation/H0"
)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=DEFAULT_ROOT,
        help="G3R2 confirmation directory (default: %(default)s)",
    )
    parser.add_argument(
        "--bag",
        action="append",
        type=Path,
        default=[],
        help="Explicit W5 bag; repeat twice. Overrides root discovery when used.",
    )
    parser.add_argument(
        "--training-bag",
        action="append",
        type=Path,
        default=[],
        help="Explicit Bsmooth projection-training bag; repeat twice.",
    )
    parser.add_argument(
        "--output",
        required=False,
        type=Path,
        help="New output directory. The analyzer refuses to reuse an existing path.",
    )
    parser.add_argument("--sensitivity-lag-ms", type=float, default=12.0)
    parser.add_argument("--observer-max-gap-ms", type=float, default=35.0)
    parser.add_argument("--causal-epsilon-ms", type=float, default=1.0e-3)
    parser.add_argument(
        "--hash-bags",
        action="store_true",
        help="Recompute bag SHA-256 instead of trusting the bound postflight value.",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run deterministic numerical checks and exit without reading bags.",
    )
    return parser.parse_args()


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def ros_time_ns(stamp: Any) -> int:
    return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)


def ns_to_sec(value: int) -> float:
    return float(value) * 1.0e-9


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def current_git_revision() -> Optional[str]:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def percentile(values: Sequence[float], quantile: float) -> float:
    clean = sorted(float(value) for value in values if finite(value))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    position = (len(clean) - 1) * float(quantile)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    weight = position - lower
    return clean[lower] * (1.0 - weight) + clean[upper] * weight


def availability_class(lead_sec: float, epsilon_sec: float) -> str:
    if lead_sec > epsilon_sec:
        return "causal"
    if lead_sec < -epsilon_sec:
        return "hindcast"
    return "boundary"


def linear_projection(q: Sequence[float], coefficients: Sequence[float]) -> float:
    return (
        float(coefficients[0])
        + float(coefficients[1]) * float(q[0])
        + float(coefficients[2]) * float(q[2])
    )


def zero_input_state(
    q0: Sequence[float], tau_sec: float, two_zeta_omega_n: float, omega_n_sq: float
) -> np.ndarray:
    """Exact underdamped free response for the two independent modal axes."""
    tau = max(0.0, float(tau_sec))
    damping_half = 0.5 * float(two_zeta_omega_n)
    stiffness = float(omega_n_sq)
    omega_d_sq = stiffness - damping_half * damping_half
    if not (finite(stiffness) and stiffness > 0.0 and omega_d_sq > 0.0):
        raise RuntimeError("zero-input baseline requires a finite underdamped model")
    omega_d = math.sqrt(omega_d_sq)
    decay = math.exp(-damping_half * tau)
    cosine = math.cos(omega_d * tau)
    sine = math.sin(omega_d * tau)

    output = np.zeros(4, dtype=float)
    for position_index, velocity_index in ((0, 1), (2, 3)):
        position = float(q0[position_index])
        velocity = float(q0[velocity_index])
        output[position_index] = decay * (
            position * cosine
            + (velocity + damping_half * position) * sine / omega_d
        )
        output[velocity_index] = decay * (
            velocity * cosine
            - (stiffness * position + damping_half * velocity) * sine / omega_d
        )
    return output


def interpolate_horizon_q(horizon: Any, tau_sec: float) -> np.ndarray:
    """Evaluate dense modal state with eta/eta_dot cubic Hermite interpolation."""
    duration = float(horizon.horizon_steps) * float(horizon.dt)
    tau = min(duration, max(0.0, float(tau_sec)))
    coordinate = tau / float(horizon.dt)
    lower = min(int(math.floor(coordinate)), int(horizon.horizon_steps))
    upper = min(lower + 1, int(horizon.horizon_steps))
    weight = coordinate - lower if upper > lower else 0.0
    q_lower = np.asarray(
        [horizon.eta_x[lower], horizon.eta_x_dot[lower], horizon.eta_y[lower], horizon.eta_y_dot[lower]],
        dtype=float,
    )
    if upper == lower:
        return q_lower
    q_upper = np.asarray(
        [horizon.eta_x[upper], horizon.eta_x_dot[upper], horizon.eta_y[upper], horizon.eta_y_dot[upper]],
        dtype=float,
    )
    step = float(horizon.dt)
    s = float(weight)
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    dh00 = 6.0 * s2 - 6.0 * s
    dh10 = 3.0 * s2 - 4.0 * s + 1.0
    dh01 = -6.0 * s2 + 6.0 * s
    dh11 = 3.0 * s2 - 2.0 * s
    output = np.zeros(4, dtype=float)
    for position_index, velocity_index in ((0, 1), (2, 3)):
        p0 = q_lower[position_index]
        v0 = q_lower[velocity_index]
        p1 = q_upper[position_index]
        v1 = q_upper[velocity_index]
        output[position_index] = h00 * p0 + h10 * step * v0 + h01 * p1 + h11 * step * v1
        output[velocity_index] = (
            dh00 * p0 + dh10 * step * v0 + dh01 * p1 + dh11 * step * v1
        ) / step
    return output


@dataclass(frozen=True)
class ObserverSample:
    bag_stamp_sec: float
    state_stamp_ns: int
    accel_stamp_ns: int
    sample_dt_sec: float
    update_count: int
    reset_epoch: int
    q: np.ndarray
    excitation: np.ndarray


@dataclass(frozen=True)
class RGBEvent:
    index: int
    source_stamp_ns: int
    bag_stamp_ns: int
    signed_slope_mm: float
    height_l_mm: float
    height_c_mm: float
    height_r_mm: float
    processing_latency_ms: float


@dataclass
class NumericSeries:
    stamps_sec: np.ndarray
    values: np.ndarray

    @classmethod
    def from_records(
        cls, records: Iterable[Tuple[float, Sequence[float]]]
    ) -> "NumericSeries":
        ordered: Dict[float, np.ndarray] = {}
        for stamp, value in records:
            if finite(stamp) and np.all(np.isfinite(np.asarray(value, dtype=float))):
                ordered[float(stamp)] = np.asarray(value, dtype=float)
        if not ordered:
            return cls(np.asarray([], dtype=float), np.empty((0, 0), dtype=float))
        stamps = np.asarray(sorted(ordered), dtype=float)
        values = np.asarray([ordered[stamp] for stamp in stamps], dtype=float)
        return cls(stamps, values)

    def interpolate(
        self, query_sec: float, max_bracketing_span_sec: float
    ) -> Tuple[Optional[np.ndarray], float, Optional[float], Optional[float]]:
        if self.stamps_sec.size == 0:
            return None, math.nan, None, None
        index = int(np.searchsorted(self.stamps_sec, float(query_sec), side="left"))
        if index < self.stamps_sec.size and abs(self.stamps_sec[index] - query_sec) <= 1e-9:
            value = self.values[index].copy()
            return value, 0.0, float(self.stamps_sec[index]), float(self.stamps_sec[index])
        if index <= 0 or index >= self.stamps_sec.size:
            return None, math.inf, None, None
        left_stamp = float(self.stamps_sec[index - 1])
        right_stamp = float(self.stamps_sec[index])
        span = right_stamp - left_stamp
        if span <= 0.0 or span > max_bracketing_span_sec:
            return None, span, left_stamp, right_stamp
        ratio = (float(query_sec) - left_stamp) / span
        value = self.values[index - 1] * (1.0 - ratio) + self.values[index] * ratio
        return value, span, left_stamp, right_stamp


def interpolate_modal_series(
    series: NumericSeries,
    query_sec: float,
    max_bracketing_span_sec: float,
) -> Tuple[Optional[np.ndarray], float, Optional[float], Optional[float]]:
    """Interpolate eta with its recorded eta_dot, never across a large gap."""

    stamps = series.stamps_sec
    if stamps.size == 0 or series.values.shape[1:] != (4,):
        return None, math.nan, None, None
    index = int(np.searchsorted(stamps, float(query_sec), side="left"))
    if index < stamps.size and abs(float(stamps[index]) - query_sec) <= 1e-9:
        return series.values[index].copy(), 0.0, float(stamps[index]), float(stamps[index])
    if index <= 0 or index >= stamps.size:
        return None, math.inf, None, None
    left_stamp = float(stamps[index - 1])
    right_stamp = float(stamps[index])
    span = right_stamp - left_stamp
    if span <= 0.0 or span > max_bracketing_span_sec:
        return None, span, left_stamp, right_stamp
    left = ModalState(*[float(value) for value in series.values[index - 1]])
    right = ModalState(*[float(value) for value in series.values[index]])
    state = cubic_hermite_q(left, right, float(query_sec) - left_stamp, span)
    return np.asarray(state.as_tuple(), dtype=float), span, left_stamp, right_stamp


@dataclass
class MetricAccumulator:
    predictions: List[float] = field(default_factory=list)
    targets: List[float] = field(default_factory=list)

    def add(self, prediction: float, target: float) -> None:
        if finite(prediction) and finite(target):
            self.predictions.append(float(prediction))
            self.targets.append(float(target))

    def summary(self) -> Dict[str, float]:
        count = len(self.predictions)
        if count == 0:
            return {
                "n": 0,
                "bias": math.nan,
                "mae": math.nan,
                "rmse": math.nan,
                "nrmse_p95_p05": math.nan,
                "correlation": math.nan,
                "calibration_slope": math.nan,
                "calibration_intercept": math.nan,
                "target_p05": math.nan,
                "target_p95": math.nan,
            }
        prediction = np.asarray(self.predictions, dtype=float)
        target = np.asarray(self.targets, dtype=float)
        error = prediction - target
        target_p05 = float(np.quantile(target, 0.05))
        target_p95 = float(np.quantile(target, 0.95))
        scale = target_p95 - target_p05
        correlation = math.nan
        slope = math.nan
        intercept = math.nan
        if count >= 3 and np.std(prediction) > 0.0 and np.std(target) > 0.0:
            correlation = float(np.corrcoef(prediction, target)[0, 1])
            design = np.column_stack([prediction, np.ones(count, dtype=float)])
            slope, intercept = [
                float(value) for value in np.linalg.lstsq(design, target, rcond=None)[0]
            ]
        rmse = float(math.sqrt(float(np.mean(np.square(error)))))
        return {
            "n": count,
            "bias": float(np.mean(error)),
            "mae": float(np.mean(np.abs(error))),
            "rmse": rmse,
            "nrmse_p95_p05": rmse / scale if scale > 1e-12 else math.nan,
            "correlation": correlation,
            "calibration_slope": slope,
            "calibration_intercept": intercept,
            "target_p05": target_p05,
            "target_p95": target_p95,
        }


class MetricStore:
    def __init__(self) -> None:
        self._data: Dict[Tuple[str, str, str, int, str, str], MetricAccumulator] = {}

    def add(
        self,
        run_id: str,
        target_type: str,
        lag_mode: str,
        bin_j: int,
        availability: str,
        model: str,
        prediction: float,
        target: float,
    ) -> None:
        scopes = ("all", availability)
        for scope in scopes:
            for identity in (run_id, "ALL_POOLED"):
                key = (identity, target_type, lag_mode, int(bin_j), scope, model)
                accumulator = self._data.setdefault(key, MetricAccumulator())
                accumulator.add(prediction, target)

    def rows(self) -> List[Dict[str, Any]]:
        rows: List[Dict[str, Any]] = []
        raw_summaries: Dict[Tuple[str, str, str, int, str, str], Dict[str, float]] = {
            key: accumulator.summary() for key, accumulator in self._data.items()
        }
        for key in sorted(raw_summaries):
            run_id, target_type, lag_mode, bin_j, scope, model = key
            summary = dict(raw_summaries[key])

            def skill_against(baseline_model: str) -> float:
                baseline_key = (
                    run_id,
                    target_type,
                    lag_mode,
                    bin_j,
                    scope,
                    baseline_model,
                )
                baseline = raw_summaries.get(baseline_key)
                if not (
                    baseline
                    and finite(summary["rmse"])
                    and finite(baseline["rmse"])
                ):
                    return math.nan
                baseline_mse = float(baseline["rmse"]) ** 2
                return (
                    1.0 - float(summary["rmse"]) ** 2 / baseline_mse
                    if baseline_mse > 1e-15
                    else math.nan
                )

            rows.append(
                {
                    "run_id": run_id,
                    "target_type": target_type,
                    "lag_mode": lag_mode,
                    "bin_j": bin_j,
                    "scope": scope,
                    "model": model,
                    **summary,
                    "skill_vs_hold_q0_mse": skill_against("hold_q0"),
                    "skill_vs_zero_input_mse": skill_against("zero_input"),
                    "skill_vs_rgb_persistence_mse": skill_against("rgb_persistence"),
                }
            )
        return rows


@dataclass
class AvailabilityAccumulator:
    origin_count: int = 0
    q_leads_ms: List[float] = field(default_factory=list)
    rgb_target_leads_ms: List[float] = field(default_factory=list)
    q_causal: int = 0
    q_hindcast: int = 0
    q_boundary: int = 0
    rgb_causal: int = 0
    rgb_hindcast: int = 0
    rgb_boundary: int = 0
    observer_valid: int = 0
    excitation_valid: int = 0

    def add_origin(
        self,
        q_lead_sec: float,
        rgb_lead_sec: float,
        q_class: str,
        rgb_class: str,
        observer_valid: bool,
        excitation_valid: bool,
    ) -> None:
        self.origin_count += 1
        self.q_leads_ms.append(1000.0 * float(q_lead_sec))
        self.rgb_target_leads_ms.append(1000.0 * float(rgb_lead_sec))
        setattr(self, "q_" + q_class, getattr(self, "q_" + q_class) + 1)
        setattr(self, "rgb_" + rgb_class, getattr(self, "rgb_" + rgb_class) + 1)
        self.observer_valid += int(observer_valid)
        self.excitation_valid += int(excitation_valid)

    def row(self, run_id: str, lag_mode: str, bin_j: int, tau_sec: float) -> Dict[str, Any]:
        def lead_fields(prefix: str, values: Sequence[float]) -> Dict[str, float]:
            return {
                prefix + "_min_ms": min(values) if values else math.nan,
                prefix + "_p05_ms": percentile(values, 0.05),
                prefix + "_p50_ms": percentile(values, 0.50),
                prefix + "_p95_ms": percentile(values, 0.95),
                prefix + "_max_ms": max(values) if values else math.nan,
            }

        denominator = float(self.origin_count) if self.origin_count else math.nan
        return {
            "run_id": run_id,
            "lag_mode": lag_mode,
            "bin_j": bin_j,
            "tau_sec": tau_sec,
            "origin_count": self.origin_count,
            **lead_fields("q_available_lead", self.q_leads_ms),
            **lead_fields("rgb_target_available_lead", self.rgb_target_leads_ms),
            "q_causal_count": self.q_causal,
            "q_hindcast_count": self.q_hindcast,
            "q_boundary_count": self.q_boundary,
            "q_causal_fraction": self.q_causal / denominator if self.origin_count else math.nan,
            "rgb_causal_count": self.rgb_causal,
            "rgb_hindcast_count": self.rgb_hindcast,
            "rgb_boundary_count": self.rgb_boundary,
            "rgb_causal_fraction": self.rgb_causal / denominator if self.origin_count else math.nan,
            "observer_target_valid_count": self.observer_valid,
            "observer_target_coverage": self.observer_valid / denominator
            if self.origin_count
            else math.nan,
            "realized_excitation_valid_count": self.excitation_valid,
            "realized_excitation_coverage": self.excitation_valid / denominator
            if self.origin_count
            else math.nan,
        }


@dataclass
class RunData:
    bag_path: Path
    postflight_path: Path
    postflight: Mapping[str, Any]
    horizons: List[Tuple[float, Any]]
    selections: List[Tuple[float, Any]]
    snapshots: List[Tuple[float, Any]]
    observers: List[ObserverSample]
    rgb_events: List[RGBEvent]
    observer_total: int
    rgb_total: int


def read_run(bag_path: Path, postflight_path: Path) -> RunData:
    postflight = json.loads(postflight_path.read_text(encoding="utf-8"))
    horizons: List[Tuple[float, Any]] = []
    selections: List[Tuple[float, Any]] = []
    snapshots: List[Tuple[float, Any]] = []
    observers: List[ObserverSample] = []
    rgb_events: List[RGBEvent] = []
    observer_total = 0
    rgb_total = 0

    with rosbag.Bag(str(bag_path), "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        required_types = {
            HORIZON_TOPIC: "spmpc_local_planner/PredictedHorizon",
            SNAPSHOT_TOPIC: "spmpc_local_planner/PreSolveSnapshot",
            SELECTION_TOPIC: "spmpc_local_planner/SloshObserverSelectionDebug",
            OBSERVER_TOPIC: "spmpc_local_planner/SloshObserverDebug",
            RGB_TOPIC: "realsense_liquid_measurement/OnlineLiquidMeasurement",
        }
        for topic, expected_type in required_types.items():
            if topic not in topic_info:
                raise RuntimeError("missing required topic {} in {}".format(topic, bag_path))
            if topic_info[topic].msg_type != expected_type:
                raise RuntimeError(
                    "unexpected type for {}: {} != {}".format(
                        topic, topic_info[topic].msg_type, expected_type
                    )
                )

        for topic, msg, bag_stamp in bag.read_messages(topics=list(required_types)):
            bag_stamp_sec = float(bag_stamp.to_sec())
            if topic == HORIZON_TOPIC:
                horizons.append((bag_stamp_sec, msg))
            elif topic == SNAPSHOT_TOPIC:
                snapshots.append((bag_stamp_sec, msg))
            elif topic == SELECTION_TOPIC:
                selections.append((bag_stamp_sec, msg))
            elif topic == OBSERVER_TOPIC:
                observer_total += 1
                if not (
                    bool(msg.valid)
                    and str(msg.input_status) == "READY"
                    and int(msg.source) == EXPECTED_SOURCE_CODE
                    and ros_time_ns(msg.state_stamp) > 0
                    and ros_time_ns(msg.accel_effective_stamp) > 0
                    and finite(msg.sample_dt_sec)
                    and float(msg.sample_dt_sec) > 0.0
                    and int(msg.observer_update_count) > 0
                ):
                    continue
                q = np.asarray(
                    [msg.eta_x, msg.eta_x_dot, msg.eta_y, msg.eta_y_dot], dtype=float
                )
                excitation = np.asarray([msg.ax_mps2, msg.ay_mps2], dtype=float)
                if not (np.all(np.isfinite(q)) and np.all(np.isfinite(excitation))):
                    continue
                observers.append(
                    ObserverSample(
                        bag_stamp_sec=bag_stamp_sec,
                        state_stamp_ns=ros_time_ns(msg.state_stamp),
                        accel_stamp_ns=ros_time_ns(msg.accel_effective_stamp),
                        sample_dt_sec=float(msg.sample_dt_sec),
                        update_count=int(msg.observer_update_count),
                        reset_epoch=int(msg.reset_epoch),
                        q=q,
                        excitation=excitation,
                    )
                )
            else:
                rgb_total += 1
                source_ns = ros_time_ns(msg.header.stamp)
                heights = np.asarray(msg.height_lcr_mm, dtype=float)
                clean = bool(
                    source_ns > 0
                    and msg.valid
                    and msg.zero_locked
                    and int(msg.status_code) == 0
                    and not msg.any_clipped
                    and heights.size == 3
                    and np.all(np.isfinite(heights))
                )
                if not clean:
                    continue
                rgb_events.append(
                    RGBEvent(
                        index=len(rgb_events),
                        source_stamp_ns=source_ns,
                        bag_stamp_ns=int(round(bag_stamp_sec * 1.0e9)),
                        signed_slope_mm=float(heights[2] - heights[0]),
                        height_l_mm=float(heights[0]),
                        height_c_mm=float(heights[1]),
                        height_r_mm=float(heights[2]),
                        processing_latency_ms=float(msg.processing_latency_ms),
                    )
                )

    observers.sort(key=lambda item: item.state_stamp_ns)
    rgb_events.sort(key=lambda item: item.source_stamp_ns)
    return RunData(
        bag_path=bag_path,
        postflight_path=postflight_path,
        postflight=postflight,
        horizons=horizons,
        selections=selections,
        snapshots=snapshots,
        observers=observers,
        rgb_events=rgb_events,
        observer_total=observer_total,
        rgb_total=rgb_total,
    )


def discover_runs(args: argparse.Namespace) -> List[Tuple[Path, Path]]:
    if args.bag:
        bag_paths = [path.expanduser().resolve() for path in args.bag]
    else:
        root = args.root.expanduser().resolve()
        bag_paths = sorted(root.glob("DEV_G3R2C_H0_C1_W5_S10_*.bag"))
    if len(bag_paths) != 2:
        raise RuntimeError("expected exactly two G3R2 W5 bags, found {}".format(len(bag_paths)))
    output: List[Tuple[Path, Path]] = []
    for bag_path in bag_paths:
        if not bag_path.is_file():
            raise RuntimeError("missing bag: {}".format(bag_path))
        postflight = bag_path.with_name(bag_path.stem + "_g3r2c_postflight.json")
        if not postflight.is_file():
            raise RuntimeError("missing postflight: {}".format(postflight))
        report = json.loads(postflight.read_text(encoding="utf-8"))
        if report.get("status") != "PASS":
            raise RuntimeError("postflight is not PASS: {}".format(postflight))
        if report.get("protocol") != EXPECTED_PROTOCOL:
            raise RuntimeError("protocol mismatch: {}".format(postflight))
        if report.get("condition") != "W5_S10":
            raise RuntimeError("condition is not W5_S10: {}".format(postflight))
        output.append((bag_path.resolve(), postflight.resolve()))
    output.sort(key=lambda pair: str(json.loads(pair[1].read_text())["row"]))
    return output


def discover_training_runs(args: argparse.Namespace) -> List[Tuple[Path, Path]]:
    if args.training_bag:
        bag_paths = [path.expanduser().resolve() for path in args.training_bag]
    else:
        root = args.root.expanduser().resolve()
        bag_paths = sorted(root.glob("DEV_G3R2C_H0_C1_Bsmooth_*.bag"))
    if len(bag_paths) != 2:
        raise RuntimeError(
            "expected exactly two G3R2 Bsmooth training bags, found {}".format(
                len(bag_paths)
            )
        )
    output: List[Tuple[Path, Path]] = []
    for bag_path in bag_paths:
        if not bag_path.is_file():
            raise RuntimeError("missing training bag: {}".format(bag_path))
        postflight = bag_path.with_name(bag_path.stem + "_g3r2c_postflight.json")
        if not postflight.is_file():
            raise RuntimeError("missing training postflight: {}".format(postflight))
        report = json.loads(postflight.read_text(encoding="utf-8"))
        if report.get("status") != "PASS":
            raise RuntimeError("training postflight is not PASS: {}".format(postflight))
        if report.get("protocol") != EXPECTED_PROTOCOL:
            raise RuntimeError("training protocol mismatch: {}".format(postflight))
        if report.get("condition") != "Bsmooth":
            raise RuntimeError("training condition is not Bsmooth: {}".format(postflight))
        output.append((bag_path.resolve(), postflight.resolve()))
    output.sort(key=lambda pair: str(json.loads(pair[1].read_text())["row"]))
    return output


def pre_motion_slope_baseline(run: RunData) -> Tuple[float, int]:
    motion_start = float(run.postflight["motion_start_sec"])
    values = [
        event.signed_slope_mm
        for event in run.rgb_events
        if motion_start - 3.0 <= ns_to_sec(event.source_stamp_ns) < motion_start
    ]
    if not values:
        raise RuntimeError("run lacks clean RGB events in the frozen 3 s pre-motion baseline")
    return float(statistics.median(values)), len(values)


def fit_projection_models(
    training_runs: Sequence[RunData],
    lag_modes: Sequence[Tuple[str, float]],
    observer_max_gap_sec: float,
) -> Dict[str, Dict[str, Any]]:
    fits: Dict[str, Dict[str, Any]] = {}
    baseline_rows = []
    baselines: Dict[str, float] = {}
    for run in training_runs:
        baseline, count = pre_motion_slope_baseline(run)
        baselines[run.bag_path.stem] = baseline
        baseline_rows.append(
            {
                "run_id": run.bag_path.stem,
                "bag": str(run.bag_path),
                "baseline_window_sec": 3.0,
                "baseline_event_count": count,
                "static_slope_baseline_mm": baseline,
            }
        )

    for lag_mode, delta_sec in lag_modes:
        design_rows: List[List[float]] = []
        targets: List[float] = []
        run_labels: List[str] = []
        per_run_counts: Dict[str, int] = {}
        for run in training_runs:
            observer_series = NumericSeries.from_records(
                (ns_to_sec(sample.state_stamp_ns), sample.q) for sample in run.observers
            )
            motion_start = float(run.postflight["motion_start_sec"])
            window_end = float(run.postflight["window_end_sec"])
            baseline = baselines[run.bag_path.stem]
            accepted = 0
            for event in run.rgb_events:
                source_sec = ns_to_sec(event.source_stamp_ns)
                if not (motion_start <= source_sec <= window_end):
                    continue
                observer_q, _, _, _ = interpolate_modal_series(
                    observer_series,
                    source_sec + delta_sec, observer_max_gap_sec
                )
                if observer_q is None:
                    continue
                design_rows.append([1.0, float(observer_q[0]), float(observer_q[2])])
                targets.append(float(event.signed_slope_mm - baseline))
                run_labels.append(run.bag_path.stem)
                accepted += 1
            per_run_counts[run.bag_path.stem] = accepted
        if len(targets) < 3:
            raise RuntimeError("projection fit has fewer than three native RGB events")
        design = np.asarray(design_rows, dtype=float)
        target = np.asarray(targets, dtype=float)
        coefficients = np.linalg.lstsq(design, target, rcond=None)[0]
        prediction = design @ coefficients
        residual = prediction - target
        target_variance = float(np.sum(np.square(target - np.mean(target))))
        residual_sum = float(np.sum(np.square(residual)))
        correlation = (
            float(np.corrcoef(prediction, target)[0, 1])
            if np.std(prediction) > 0.0 and np.std(target) > 0.0
            else math.nan
        )
        features = design[:, 1:]
        feature_std = np.std(features, axis=0)
        target_std = float(np.std(target))
        standardized_features = (
            (features - np.mean(features, axis=0)) / feature_std
            if np.all(feature_std > 0.0)
            else np.full_like(features, math.nan)
        )
        per_run_metrics: Dict[str, Dict[str, float]] = {}
        labels_array = np.asarray(run_labels)
        for run_id in sorted(set(run_labels)):
            mask = labels_array == run_id
            run_target = target[mask]
            run_prediction = prediction[mask]
            run_error = run_prediction - run_target
            per_run_metrics[run_id] = {
                "n": int(np.sum(mask)),
                "correlation": float(np.corrcoef(run_prediction, run_target)[0, 1])
                if np.sum(mask) >= 3
                and np.std(run_prediction) > 0.0
                and np.std(run_target) > 0.0
                else math.nan,
                "rmse_mm": float(math.sqrt(float(np.mean(np.square(run_error))))),
                "calibration_gain": float(
                    np.linalg.lstsq(
                        np.column_stack(
                            [run_prediction, np.ones(run_prediction.size, dtype=float)]
                        ),
                        run_target,
                        rcond=None,
                    )[0][0]
                ),
                "calibration_intercept_mm": float(
                    np.linalg.lstsq(
                        np.column_stack(
                            [run_prediction, np.ones(run_prediction.size, dtype=float)]
                        ),
                        run_target,
                        rcond=None,
                    )[0][1]
                ),
            }
        fits[lag_mode] = {
            "lag_mode": lag_mode,
            "observer_rgb_alignment_sec": delta_sec,
            "coefficients": {
                "b0_mm": float(coefficients[0]),
                "eta_x_mm": float(coefficients[1]),
                "eta_y_mm": float(coefficients[2]),
            },
            "native_event_count": len(targets),
            "per_run_event_counts": per_run_counts,
            "per_run_metrics": per_run_metrics,
            "correlation": correlation,
            "r_squared": 1.0 - residual_sum / target_variance
            if target_variance > 0.0
            else math.nan,
            "rmse_mm": math.sqrt(residual_sum / len(targets)),
            "eta_x_eta_y_correlation": float(np.corrcoef(features[:, 0], features[:, 1])[0, 1]),
            "raw_design_condition_number": float(np.linalg.cond(design)),
            "standardized_feature_condition_number": float(
                np.linalg.cond(standardized_features)
            )
            if np.all(np.isfinite(standardized_features))
            else math.nan,
            "standardized_coefficients": {
                "eta_x": float(coefficients[1] * feature_std[0] / target_std)
                if target_std > 0.0
                else math.nan,
                "eta_y": float(coefficients[2] * feature_std[1] / target_std)
                if target_std > 0.0
                else math.nan,
            },
        }
    fits["training_baselines"] = {
        "definition": "median(R-L) over [motion_start-3s, motion_start)",
        "runs": baseline_rows,
    }
    return fits


def projection_coefficients(
    projection_fits: Mapping[str, Mapping[str, Any]], lag_mode: str
) -> Tuple[float, float, float]:
    values = projection_fits[lag_mode]["coefficients"]
    return (
        float(values["b0_mm"]),
        float(values["eta_x_mm"]),
        float(values["eta_y_mm"]),
    )


def snapshot_parameter_map(snapshot: Any, stage: int = 0) -> Dict[str, float]:
    width = int(snapshot.parameter_width)
    names = list(snapshot.parameter_names)
    if width <= 0 or len(names) != width:
        raise RuntimeError("invalid pre-solve parameter schema")
    start = int(stage) * width
    values = list(snapshot.stage_parameters[start : start + width])
    if len(values) != width:
        raise RuntimeError("incomplete pre-solve stage parameter row")
    return {str(name): float(value) for name, value in zip(names, values)}


def validate_horizon(horizon: Any) -> None:
    if not (
        bool(horizon.valid)
        and bool(horizon.slosh_enabled)
        and str(horizon.variant) == EXPECTED_VARIANT
        and str(horizon.solver_status) == EXPECTED_HORIZON_STATUS
        and str(horizon.backend) == "continuous_mpcc_acados"
        and str(horizon.control_semantics) == "alpha"
    ):
        raise RuntimeError("horizon does not satisfy the frozen W5 contract")
    steps = int(horizon.horizon_steps)
    state_fields = (
        "t",
        "x",
        "y",
        "yaw",
        "v",
        "omega",
        "s",
        "eta_x",
        "eta_x_dot",
        "eta_y",
        "eta_y_dot",
        "h_modal",
    )
    control_fields = ("a", "alpha_or_omega", "v_s")
    if steps != 60 or abs(float(horizon.dt) - 1.0 / 30.0) > 1e-8:
        raise RuntimeError("unexpected horizon N/dt")
    if any(len(getattr(horizon, field)) != steps + 1 for field in state_fields):
        raise RuntimeError("incomplete horizon state arrays")
    if any(len(getattr(horizon, field)) != steps for field in control_fields):
        raise RuntimeError("incomplete horizon control arrays")
    expected_t = np.arange(steps + 1, dtype=float) * float(horizon.dt)
    if not np.allclose(np.asarray(horizon.t, dtype=float), expected_t, atol=1e-12, rtol=0.0):
        raise RuntimeError("horizon relative time grid is not exact j*dt")


def model_metrics_add(
    store: MetricStore,
    run_id: str,
    target_type: str,
    lag_mode: str,
    bin_j: int,
    availability: str,
    predictions: Mapping[str, Optional[float]],
    target: float,
) -> None:
    for model, prediction in predictions.items():
        if prediction is not None and finite(prediction):
            store.add(
                run_id,
                target_type,
                lag_mode,
                bin_j,
                availability,
                model,
                float(prediction),
                float(target),
            )


def format_csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    return value


def write_csv(path: Path, fields: Sequence[str], rows: Iterable[Mapping[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: format_csv_value(row.get(field)) for field in fields})


NODE_FIELDS = (
    "run_id",
    "block",
    "row",
    "cycle_index",
    "pair_method",
    "horizon_header_stamp_ns",
    "selection_header_stamp_ns",
    "selected_state_stamp_ns",
    "j",
    "tau_sec",
    "target_q_stamp_sec",
    "q_available_lead_ms",
    "q_availability",
    "target_rgb_primary_stamp_sec",
    "rgb_primary_available_lead_ms",
    "rgb_primary_availability",
    "target_rgb_sensitivity_stamp_sec",
    "rgb_sensitivity_available_lead_ms",
    "rgb_sensitivity_availability",
    "pred_eta_x",
    "pred_eta_x_dot",
    "pred_eta_y",
    "pred_eta_y_dot",
    "pred_projection_mm",
    "planned_replay_eta_x",
    "planned_replay_eta_x_dot",
    "planned_replay_eta_y",
    "planned_replay_eta_y_dot",
    "planned_replay_projection_mm",
    "ocp_planned_replay_projection_error_mm",
    "actual_replay_eta_x",
    "actual_replay_eta_x_dot",
    "actual_replay_eta_y",
    "actual_replay_eta_y_dot",
    "actual_replay_projection_mm",
    "actual_replay_valid",
    "hold_q0_projection_mm",
    "zero_input_projection_mm",
    "obs_eta_x",
    "obs_eta_x_dot",
    "obs_eta_y",
    "obs_eta_y_dot",
    "obs_projection_mm",
    "actual_replay_observer_projection_error_mm",
    "observer_bracketing_span_ms",
    "observer_valid",
    "planned_ax_mps2",
    "planned_ay_mps2",
    "realized_ax_mps2",
    "realized_ay_mps2",
    "excitation_bracketing_span_ms",
    "excitation_valid",
    "stage_slosh_cost",
    "cumulative_slosh_cost_fraction",
    "modal_constraint_margin_eta_sq",
)


EVENT_FIELDS = (
    "run_id",
    "block",
    "row",
    "cycle_index",
    "pair_method",
    "lag_mode",
    "observer_rgb_alignment_ms",
    "horizon_header_stamp_ns",
    "selected_state_stamp_ns",
    "rgb_event_index",
    "rgb_source_stamp_ns",
    "rgb_bag_stamp_ns",
    "rgb_processing_latency_ms",
    "rgb_signed_slope_mm",
    "rgb_height_l_mm",
    "rgb_height_c_mm",
    "rgb_height_r_mm",
    "rgb_static_slope_baseline_mm",
    "rgb_centered_signed_slope_mm",
    "raw_rgb_source_lead_ms",
    "raw_rgb_source_availability",
    "model_available_lead_ms",
    "model_availability",
    "causal_lead_bin_j",
    "rgb_message_arrival_lead_ms",
    "rgb_message_available_at_forecast",
    "state_equivalent_stamp_sec",
    "tau_sec",
    "model_fractional_j",
    "model_nearest_j",
    "pred_projection_mm",
    "planned_replay_projection_mm",
    "actual_replay_projection_mm",
    "actual_replay_valid",
    "future_observer_projection_mm",
    "future_observer_valid",
    "future_observer_bracketing_span_ms",
    "hold_q0_projection_mm",
    "zero_input_projection_mm",
    "rgb_persistence_mm",
    "rgb_persistence_source_age_ms",
    "mpc_error_mm",
    "planned_replay_error_mm",
    "actual_replay_error_mm",
    "future_observer_error_mm",
    "hold_q0_error_mm",
    "zero_input_error_mm",
    "rgb_persistence_error_mm",
)


METRIC_FIELDS = (
    "run_id",
    "target_type",
    "lag_mode",
    "bin_j",
    "scope",
    "model",
    "n",
    "bias",
    "mae",
    "rmse",
    "nrmse_p95_p05",
    "correlation",
    "calibration_slope",
    "calibration_intercept",
    "target_p05",
    "target_p95",
    "skill_vs_hold_q0_mse",
    "skill_vs_zero_input_mse",
    "skill_vs_rgb_persistence_mse",
)


def analyze_run(
    run: RunData,
    node_writer: csv.DictWriter,
    event_writer: csv.DictWriter,
    metrics: MetricStore,
    availability: Dict[Tuple[str, str, int], AvailabilityAccumulator],
    projection_fits: Mapping[str, Mapping[str, Any]],
    lag_modes: Sequence[Tuple[str, float]],
    observer_max_gap_sec: float,
    causal_epsilon_sec: float,
) -> Dict[str, Any]:
    report = run.postflight
    run_id = run.bag_path.stem
    motion_start = float(report["motion_start_sec"])
    motion_end = float(report["motion_end_sec"])
    rgb_static_baseline, rgb_baseline_event_count = pre_motion_slope_baseline(run)
    primary_coefficients = projection_coefficients(projection_fits, "primary_delta0")
    if not (
        len(run.horizons) == len(run.selections) == len(run.snapshots)
    ):
        raise RuntimeError(
            "W5 sequence counts differ: horizon={} selection={} snapshot={}".format(
                len(run.horizons), len(run.selections), len(run.snapshots)
            )
        )

    observer_by_stamp = {sample.state_stamp_ns: sample for sample in run.observers}
    observer_stamps_ns = [sample.state_stamp_ns for sample in run.observers]
    observer_series = NumericSeries.from_records(
        (ns_to_sec(sample.state_stamp_ns), sample.q) for sample in run.observers
    )
    excitation_series = NumericSeries.from_records(
        (ns_to_sec(sample.accel_stamp_ns), sample.excitation) for sample in run.observers
    )
    rgb_source_stamps = [event.source_stamp_ns for event in run.rgb_events]
    rgb_by_arrival = sorted(run.rgb_events, key=lambda event: event.bag_stamp_ns)
    rgb_arrival_stamps = [event.bag_stamp_ns for event in rgb_by_arrival]

    valid_horizon_total = 0
    terminal_empty_total = 0
    eligible_cycles: List[Tuple[int, Any, Any, Any]] = []
    q0_max_abs_error = 0.0
    header_minus_selection_ms: List[float] = []
    header_minus_state_ms: List[float] = []

    model_two_zeta_omega_n: Optional[float] = None
    model_omega_n_sq: Optional[float] = None
    for cycle_index, ((_, horizon), (_, selection), (_, snapshot)) in enumerate(
        zip(run.horizons, run.selections, run.snapshots)
    ):
        if not bool(horizon.valid):
            if str(horizon.solver_status) == "GOAL_REACHED":
                terminal_empty_total += 1
            continue
        valid_horizon_total += 1
        validate_horizon(horizon)
        if not (
            bool(selection.valid)
            and not bool(selection.fallback_active)
            and int(selection.effective_source) == EXPECTED_SOURCE_CODE
            and int(selection.imu_reset_epoch) == 0
            and bool(selection.solver_consumes_selected_state)
        ):
            raise RuntimeError("invalid processed-IMU selection at cycle {}".format(cycle_index))
        if not (bool(snapshot.valid) and bool(snapshot.slosh_enabled)):
            raise RuntimeError("invalid pre-solve snapshot at cycle {}".format(cycle_index))
        if abs(float(snapshot.header.stamp.to_sec()) - float(horizon.header.stamp.to_sec())) > 0.01:
            raise RuntimeError("snapshot/horizon sequence mismatch at cycle {}".format(cycle_index))
        if not (
            int(snapshot.header.seq) == int(horizon.header.seq)
            and int(selection.header.seq) == int(horizon.header.seq)
        ):
            raise RuntimeError("legacy diagnostic header.seq mismatch at cycle {}".format(cycle_index))

        parameter_map = snapshot_parameter_map(snapshot)
        cycle_two_zeta = parameter_map.get("two_zeta_omega_n")
        cycle_omega_sq = parameter_map.get("omega_n_sq")
        if not (finite(cycle_two_zeta) and finite(cycle_omega_sq)):
            raise RuntimeError("snapshot lacks liquid dynamics parameters")
        if model_two_zeta_omega_n is None:
            model_two_zeta_omega_n = float(cycle_two_zeta)
            model_omega_n_sq = float(cycle_omega_sq)
        elif (
            abs(float(cycle_two_zeta) - model_two_zeta_omega_n) > 1e-12
            or abs(float(cycle_omega_sq) - float(model_omega_n_sq)) > 1e-9
        ):
            raise RuntimeError("liquid dynamics parameters changed within a bag")

        state_stamp_ns = ros_time_ns(selection.selected_state_stamp)
        selected_observer = observer_by_stamp.get(state_stamp_ns)
        if selected_observer is None:
            raise RuntimeError("selection state stamp has no READY observer sample")
        q0 = np.asarray(
            [horizon.eta_x[0], horizon.eta_x_dot[0], horizon.eta_y[0], horizon.eta_y_dot[0]],
            dtype=float,
        )
        q0_error = float(np.max(np.abs(q0 - selected_observer.q)))
        q0_max_abs_error = max(q0_max_abs_error, q0_error)
        if q0_error > 1e-12:
            raise RuntimeError("horizon stage 0 differs from selected observer")

        header_stamp = float(horizon.header.stamp.to_sec())
        selection_stamp = float(selection.header.stamp.to_sec())
        state_stamp = ns_to_sec(state_stamp_ns)
        header_minus_selection_ms.append(1000.0 * (header_stamp - selection_stamp))
        header_minus_state_ms.append(1000.0 * (header_stamp - state_stamp))
        if motion_start <= header_stamp <= motion_end:
            eligible_cycles.append((cycle_index, horizon, selection, snapshot))

    if not eligible_cycles:
        raise RuntimeError("no eligible W5 horizons in the motion-to-GOAL window")
    assert model_two_zeta_omega_n is not None and model_omega_n_sq is not None

    node_row_count = 0
    event_row_count = 0
    planned_numeric_projection_errors: List[float] = []
    actual_observer_projection_errors: List[float] = []
    actual_endpoint_state_errors: List[float] = []
    for cycle_index, horizon, selection, snapshot in eligible_cycles:
        header_stamp_ns = ros_time_ns(horizon.header.stamp)
        header_stamp_sec = ns_to_sec(header_stamp_ns)
        selection_stamp_ns = ros_time_ns(selection.header.stamp)
        state_stamp_ns = ros_time_ns(selection.selected_state_stamp)
        state_stamp_sec = ns_to_sec(state_stamp_ns)
        q0 = np.asarray(
            [horizon.eta_x[0], horizon.eta_x_dot[0], horizon.eta_y[0], horizon.eta_y_dot[0]],
            dtype=float,
        )
        hold_projection = linear_projection(q0, primary_coefficients)

        parameter_map = snapshot_parameter_map(snapshot)
        modal_parameters = ModalParameters(
            two_zeta_omega_n=float(parameter_map["two_zeta_omega_n"]),
            omega_n_sq=float(parameter_map["omega_n_sq"]),
            kappa_x=float(parameter_map["kappa_x"]),
            kappa_y=float(parameter_map["kappa_y"]),
        )
        modal_q0 = ModalState(*[float(value) for value in q0])
        planned_controls = [
            PlannedControl(
                a=float(horizon.a[index]),
                alpha=float(horizon.alpha_or_omega[index]),
                duration_sec=float(horizon.dt),
            )
            for index in range(int(horizon.horizon_steps))
        ]
        planned_replay = replay_planned_controls(
            modal_q0,
            float(horizon.v[0]),
            float(horizon.omega[0]),
            planned_controls,
            modal_parameters,
        )

        observer_start = bisect.bisect_left(observer_stamps_ns, state_stamp_ns)
        if (
            observer_start >= len(run.observers)
            or run.observers[observer_start].state_stamp_ns != state_stamp_ns
        ):
            raise RuntimeError("selected q0 has no exact observer replay anchor")
        duration_ns = int(round(float(horizon.horizon_steps) * float(horizon.dt) * 1.0e9))
        observer_end = bisect.bisect_right(
            observer_stamps_ns, state_stamp_ns + duration_ns
        )
        observer_end = min(len(run.observers), observer_end + 1)
        replay_samples = [
            ObserverInputSample(
                state_stamp_ns=sample.state_stamp_ns,
                sample_dt_sec=sample.sample_dt_sec,
                update_count=sample.update_count,
                reset_epoch=sample.reset_epoch,
                ax=float(sample.excitation[0]),
                ay=float(sample.excitation[1]),
            )
            for sample in run.observers[observer_start:observer_end]
        ]
        anchor_sample = run.observers[observer_start]
        actual_replay = replay_observer_inputs(
            ObserverAnchor(
                state_stamp_ns=state_stamp_ns,
                update_count=anchor_sample.update_count,
                reset_epoch=anchor_sample.reset_epoch,
                q=modal_q0,
            ),
            replay_samples,
            modal_parameters,
        )
        for point in actual_replay.points:
            recorded = observer_by_stamp.get(point.state_stamp_ns)
            if recorded is None:
                raise RuntimeError("observer replay endpoint is absent from recorded states")
            actual_endpoint_state_errors.append(
                float(
                    np.max(
                        np.abs(
                            np.asarray(point.q.as_tuple(), dtype=float) - recorded.q
                        )
                    )
                )
            )

        persistence_index = bisect.bisect_right(rgb_arrival_stamps, header_stamp_ns) - 1
        persistence_event: Optional[RGBEvent] = None
        while persistence_index >= 0:
            candidate = rgb_by_arrival[persistence_index]
            if candidate.source_stamp_ns <= header_stamp_ns:
                persistence_event = candidate
                break
            persistence_index -= 1

        parameter_names = list(snapshot.parameter_names)
        parameter_width = int(snapshot.parameter_width)
        if len(parameter_names) != parameter_width:
            raise RuntimeError("invalid per-cycle parameter schema")
        slosh_costs: List[float] = []
        margins: List[float] = []
        for j in range(int(horizon.horizon_steps) + 1):
            parameters = snapshot_parameter_map(snapshot, j)
            eta_norm = math.hypot(float(horizon.eta_x[j]), float(horizon.eta_y[j]))
            eta_dot_norm = math.hypot(
                float(horizon.eta_x_dot[j]), float(horizon.eta_y_dot[j])
            )
            eta_ref = max(1e-12, float(parameters["eta_ref"]))
            eta_dot_ref = max(1e-12, float(parameters["eta_dot_ref"]))
            stage_cost = (
                float(parameters["w_slosh_eta"]) * (eta_norm / eta_ref) ** 2
                + float(parameters["w_slosh_eta_dot"])
                * (eta_dot_norm / eta_dot_ref) ** 2
            ) / float(horizon.horizon_steps)
            slosh_costs.append(stage_cost)
            margins.append(float(parameters["eta_max_sq"]) - eta_norm * eta_norm)
        total_slosh_cost = sum(slosh_costs)
        cumulative_cost = 0.0

        for j in range(int(horizon.horizon_steps) + 1):
            tau_sec = float(horizon.t[j])
            target_q_sec = state_stamp_sec + tau_sec
            q_lead_sec = target_q_sec - header_stamp_sec
            q_class = availability_class(q_lead_sec, causal_epsilon_sec)
            primary_rgb_sec = target_q_sec
            sensitivity_delta = dict(lag_modes)["sensitivity_plus_lag"]
            sensitivity_rgb_sec = target_q_sec - sensitivity_delta
            primary_rgb_lead = primary_rgb_sec - header_stamp_sec
            sensitivity_rgb_lead = sensitivity_rgb_sec - header_stamp_sec
            primary_rgb_class = availability_class(primary_rgb_lead, causal_epsilon_sec)
            sensitivity_rgb_class = availability_class(
                sensitivity_rgb_lead, causal_epsilon_sec
            )

            pred_q = np.asarray(
                [
                    horizon.eta_x[j],
                    horizon.eta_x_dot[j],
                    horizon.eta_y[j],
                    horizon.eta_y_dot[j],
                ],
                dtype=float,
            )
            pred_projection = linear_projection(pred_q, primary_coefficients)
            planned_replay_q = sample_planned_replay(planned_replay, tau_sec).q
            planned_replay_array = np.asarray(planned_replay_q.as_tuple(), dtype=float)
            planned_replay_projection = linear_projection(
                planned_replay_array, primary_coefficients
            )
            planned_numeric_error = pred_projection - planned_replay_projection
            planned_numeric_projection_errors.append(planned_numeric_error)

            target_q_stamp_ns = state_stamp_ns + int(round(tau_sec * 1.0e9))
            actual_replay_q = sample_observer_replay(
                actual_replay.points, target_q_stamp_ns, modal_parameters
            )
            actual_replay_array = np.asarray(actual_replay_q.as_tuple(), dtype=float)
            actual_replay_projection = linear_projection(
                actual_replay_array, primary_coefficients
            )
            zero_q = zero_input_state(
                q0, tau_sec, model_two_zeta_omega_n, model_omega_n_sq
            )
            zero_projection = linear_projection(zero_q, primary_coefficients)
            observer_q, observer_span, _, _ = interpolate_modal_series(
                observer_series,
                target_q_sec, observer_max_gap_sec
            )
            observer_valid = observer_q is not None
            observer_projection = (
                linear_projection(observer_q, primary_coefficients)
                if observer_valid
                else math.nan
            )
            actual_observer_error = (
                actual_replay_projection - observer_projection
                if observer_valid
                else math.nan
            )
            if finite(actual_observer_error):
                actual_observer_projection_errors.append(actual_observer_error)

            excitation, excitation_span, _, _ = excitation_series.interpolate(
                target_q_sec, observer_max_gap_sec
            )
            excitation_valid = excitation is not None
            planned_ax = float(horizon.a[j]) if j < int(horizon.horizon_steps) else math.nan
            planned_ay = (
                float(horizon.v[j]) * float(horizon.omega[j])
                if j < int(horizon.horizon_steps)
                else math.nan
            )

            cumulative_cost += slosh_costs[j]
            cumulative_fraction = (
                cumulative_cost / total_slosh_cost if total_slosh_cost > 0.0 else math.nan
            )
            node_row = {
                "run_id": run_id,
                "block": report["block"],
                "row": report["row"],
                "cycle_index": cycle_index,
                "pair_method": "legacy_global_sequence_seq_and_q0_identity",
                "horizon_header_stamp_ns": header_stamp_ns,
                "selection_header_stamp_ns": selection_stamp_ns,
                "selected_state_stamp_ns": state_stamp_ns,
                "j": j,
                "tau_sec": tau_sec,
                "target_q_stamp_sec": target_q_sec,
                "q_available_lead_ms": 1000.0 * q_lead_sec,
                "q_availability": q_class,
                "target_rgb_primary_stamp_sec": primary_rgb_sec,
                "rgb_primary_available_lead_ms": 1000.0 * primary_rgb_lead,
                "rgb_primary_availability": primary_rgb_class,
                "target_rgb_sensitivity_stamp_sec": sensitivity_rgb_sec,
                "rgb_sensitivity_available_lead_ms": 1000.0 * sensitivity_rgb_lead,
                "rgb_sensitivity_availability": sensitivity_rgb_class,
                "pred_eta_x": pred_q[0],
                "pred_eta_x_dot": pred_q[1],
                "pred_eta_y": pred_q[2],
                "pred_eta_y_dot": pred_q[3],
                "pred_projection_mm": pred_projection,
                "planned_replay_eta_x": planned_replay_array[0],
                "planned_replay_eta_x_dot": planned_replay_array[1],
                "planned_replay_eta_y": planned_replay_array[2],
                "planned_replay_eta_y_dot": planned_replay_array[3],
                "planned_replay_projection_mm": planned_replay_projection,
                "ocp_planned_replay_projection_error_mm": planned_numeric_error,
                "actual_replay_eta_x": actual_replay_array[0],
                "actual_replay_eta_x_dot": actual_replay_array[1],
                "actual_replay_eta_y": actual_replay_array[2],
                "actual_replay_eta_y_dot": actual_replay_array[3],
                "actual_replay_projection_mm": actual_replay_projection,
                "actual_replay_valid": 1,
                "hold_q0_projection_mm": hold_projection,
                "zero_input_projection_mm": zero_projection,
                "obs_eta_x": observer_q[0] if observer_valid else math.nan,
                "obs_eta_x_dot": observer_q[1] if observer_valid else math.nan,
                "obs_eta_y": observer_q[2] if observer_valid else math.nan,
                "obs_eta_y_dot": observer_q[3] if observer_valid else math.nan,
                "obs_projection_mm": observer_projection,
                "actual_replay_observer_projection_error_mm": actual_observer_error,
                "observer_bracketing_span_ms": 1000.0 * observer_span,
                "observer_valid": int(observer_valid),
                "planned_ax_mps2": planned_ax,
                "planned_ay_mps2": planned_ay,
                "realized_ax_mps2": excitation[0] if excitation_valid else math.nan,
                "realized_ay_mps2": excitation[1] if excitation_valid else math.nan,
                "excitation_bracketing_span_ms": 1000.0 * excitation_span,
                "excitation_valid": int(excitation_valid),
                "stage_slosh_cost": slosh_costs[j],
                "cumulative_slosh_cost_fraction": cumulative_fraction,
                "modal_constraint_margin_eta_sq": margins[j],
            }
            node_writer.writerow(
                {field: format_csv_value(node_row.get(field)) for field in NODE_FIELDS}
            )
            node_row_count += 1

            if observer_valid:
                model_metrics_add(
                    metrics,
                    run_id,
                    "observer_projection",
                    "state_time",
                    j,
                    q_class,
                    {
                        "mpc_horizon": pred_projection,
                        "planned_high_accuracy": planned_replay_projection,
                        "actual_input_replay": actual_replay_projection,
                        "hold_q0": hold_projection,
                        "zero_input": zero_projection,
                    },
                    observer_projection,
                )
            metrics.add(
                run_id,
                "planned_numeric_projection",
                "state_time",
                j,
                q_class,
                "mpc_horizon",
                pred_projection,
                planned_replay_projection,
            )
            for lag_mode, delta_sec in lag_modes:
                rgb_target_lead = q_lead_sec - delta_sec
                rgb_target_class = availability_class(
                    rgb_target_lead, causal_epsilon_sec
                )
                ledger = availability.setdefault(
                    (run_id, lag_mode, j), AvailabilityAccumulator()
                )
                ledger.add_origin(
                    q_lead_sec,
                    rgb_target_lead,
                    q_class,
                    rgb_target_class,
                    observer_valid,
                    excitation_valid,
                )

        duration_sec = float(horizon.horizon_steps) * float(horizon.dt)
        for lag_mode, delta_sec in lag_modes:
            event_coefficients = projection_coefficients(projection_fits, lag_mode)
            event_hold_projection = linear_projection(q0, event_coefficients)
            lower_source_ns = int(math.ceil((state_stamp_sec - delta_sec) * 1.0e9))
            upper_source_ns = int(
                math.floor((state_stamp_sec + duration_sec - delta_sec) * 1.0e9)
            )
            left = bisect.bisect_left(rgb_source_stamps, lower_source_ns)
            right = bisect.bisect_right(rgb_source_stamps, upper_source_ns)
            for event in run.rgb_events[left:right]:
                event_source_sec = ns_to_sec(event.source_stamp_ns)
                state_equivalent_sec = event_source_sec + delta_sec
                tau_sec = state_equivalent_sec - state_stamp_sec
                if tau_sec < -1e-7 or tau_sec > duration_sec + 1e-7:
                    continue
                tau_sec = min(duration_sec, max(0.0, tau_sec))
                fractional_j = tau_sec / float(horizon.dt)
                nearest_j = min(
                    int(horizon.horizon_steps), int(math.floor(fractional_j + 0.5))
                )
                raw_source_lead_sec = event_source_sec - header_stamp_sec
                raw_source_class = availability_class(
                    raw_source_lead_sec, causal_epsilon_sec
                )
                model_lead_sec = state_equivalent_sec - header_stamp_sec
                model_class = availability_class(model_lead_sec, causal_epsilon_sec)
                causal_lead_bin_j = int(math.floor(model_lead_sec / float(horizon.dt)))
                event_arrival_lead_sec = ns_to_sec(event.bag_stamp_ns) - header_stamp_sec

                pred_q = interpolate_horizon_q(horizon, tau_sec)
                pred_projection = linear_projection(pred_q, event_coefficients)
                planned_event_q = sample_planned_replay(planned_replay, tau_sec).q
                planned_event_projection = linear_projection(
                    planned_event_q.as_tuple(), event_coefficients
                )
                state_equivalent_ns = event.source_stamp_ns + int(
                    round(delta_sec * 1.0e9)
                )
                actual_event_q = sample_observer_replay(
                    actual_replay.points, state_equivalent_ns, modal_parameters
                )
                actual_event_projection = linear_projection(
                    actual_event_q.as_tuple(), event_coefficients
                )
                future_observer_q, future_observer_span, _, _ = interpolate_modal_series(
                    observer_series,
                    state_equivalent_sec,
                    observer_max_gap_sec,
                )
                future_observer_valid = future_observer_q is not None
                future_observer_projection = (
                    linear_projection(future_observer_q, event_coefficients)
                    if future_observer_valid
                    else math.nan
                )
                zero_q = zero_input_state(
                    q0, tau_sec, model_two_zeta_omega_n, model_omega_n_sq
                )
                zero_projection = linear_projection(zero_q, event_coefficients)
                persistence_value = (
                    persistence_event.signed_slope_mm - rgb_static_baseline
                    if persistence_event
                    else None
                )
                persistence_age_ms = (
                    1000.0
                    * (header_stamp_sec - ns_to_sec(persistence_event.source_stamp_ns))
                    if persistence_event
                    else math.nan
                )
                target_raw = event.signed_slope_mm
                target = target_raw - rgb_static_baseline
                event_row = {
                    "run_id": run_id,
                    "block": report["block"],
                    "row": report["row"],
                    "cycle_index": cycle_index,
                    "pair_method": "legacy_global_sequence_seq_and_q0_identity",
                    "lag_mode": lag_mode,
                    "observer_rgb_alignment_ms": 1000.0 * delta_sec,
                    "horizon_header_stamp_ns": header_stamp_ns,
                    "selected_state_stamp_ns": state_stamp_ns,
                    "rgb_event_index": event.index,
                    "rgb_source_stamp_ns": event.source_stamp_ns,
                    "rgb_bag_stamp_ns": event.bag_stamp_ns,
                    "rgb_processing_latency_ms": event.processing_latency_ms,
                    "rgb_signed_slope_mm": target_raw,
                    "rgb_height_l_mm": event.height_l_mm,
                    "rgb_height_c_mm": event.height_c_mm,
                    "rgb_height_r_mm": event.height_r_mm,
                    "rgb_static_slope_baseline_mm": rgb_static_baseline,
                    "rgb_centered_signed_slope_mm": target,
                    "raw_rgb_source_lead_ms": 1000.0 * raw_source_lead_sec,
                    "raw_rgb_source_availability": raw_source_class,
                    "model_available_lead_ms": 1000.0 * model_lead_sec,
                    "model_availability": model_class,
                    "causal_lead_bin_j": causal_lead_bin_j,
                    "rgb_message_arrival_lead_ms": 1000.0 * event_arrival_lead_sec,
                    "rgb_message_available_at_forecast": int(event.bag_stamp_ns <= header_stamp_ns),
                    "state_equivalent_stamp_sec": state_equivalent_sec,
                    "tau_sec": tau_sec,
                    "model_fractional_j": fractional_j,
                    "model_nearest_j": nearest_j,
                    "pred_projection_mm": pred_projection,
                    "planned_replay_projection_mm": planned_event_projection,
                    "actual_replay_projection_mm": actual_event_projection,
                    "actual_replay_valid": 1,
                    "future_observer_projection_mm": future_observer_projection,
                    "future_observer_valid": int(future_observer_valid),
                    "future_observer_bracketing_span_ms": 1000.0
                    * future_observer_span,
                    "hold_q0_projection_mm": event_hold_projection,
                    "zero_input_projection_mm": zero_projection,
                    "rgb_persistence_mm": persistence_value,
                    "rgb_persistence_source_age_ms": persistence_age_ms,
                    "mpc_error_mm": pred_projection - target,
                    "planned_replay_error_mm": planned_event_projection - target,
                    "actual_replay_error_mm": actual_event_projection - target,
                    "future_observer_error_mm": future_observer_projection - target
                    if future_observer_valid
                    else math.nan,
                    "hold_q0_error_mm": event_hold_projection - target,
                    "zero_input_error_mm": zero_projection - target,
                    "rgb_persistence_error_mm": persistence_value - target
                    if persistence_value is not None
                    else math.nan,
                }
                event_writer.writerow(
                    {field: format_csv_value(event_row.get(field)) for field in EVENT_FIELDS}
                )
                event_row_count += 1
                predictions: Dict[str, Optional[float]] = {
                    "mpc_horizon": pred_projection,
                    "planned_high_accuracy": planned_event_projection,
                    "actual_input_replay": actual_event_projection,
                    "future_observer": future_observer_projection
                    if future_observer_valid
                    else None,
                    "hold_q0": event_hold_projection,
                    "zero_input": zero_projection,
                    "rgb_persistence": persistence_value,
                }
                model_metrics_add(
                    metrics,
                    run_id,
                    "native_rgb_event",
                    lag_mode,
                    causal_lead_bin_j,
                    model_class,
                    predictions,
                    target,
                )
    return {
        "run_id": run_id,
        "row": report["row"],
        "block": report["block"],
        "bag": str(run.bag_path),
        "horizon_messages": len(run.horizons),
        "valid_acados_horizons": valid_horizon_total,
        "terminal_empty_horizons": terminal_empty_total,
        "eligible_origin_count": len(eligible_cycles),
        "selection_messages": len(run.selections),
        "snapshot_messages": len(run.snapshots),
        "observer_messages": run.observer_total,
        "ready_observer_messages": len(run.observers),
        "rgb_messages": run.rgb_total,
        "clean_native_rgb_events": len(run.rgb_events),
        "rgb_static_slope_baseline_mm": rgb_static_baseline,
        "rgb_baseline_event_count": rgb_baseline_event_count,
        "per_cycle_node_rows": node_row_count,
        "per_cycle_rgb_event_rows": event_row_count,
        "q0_max_abs_error": q0_max_abs_error,
        "ocp_vs_high_accuracy_projection_rmse_mm": math.sqrt(
            statistics.mean(value * value for value in planned_numeric_projection_errors)
        )
        if planned_numeric_projection_errors
        else math.nan,
        "ocp_vs_high_accuracy_projection_max_abs_mm": max(
            abs(value) for value in planned_numeric_projection_errors
        )
        if planned_numeric_projection_errors
        else math.nan,
        "actual_replay_vs_observer_projection_rmse_mm": math.sqrt(
            statistics.mean(value * value for value in actual_observer_projection_errors)
        )
        if actual_observer_projection_errors
        else math.nan,
        "actual_replay_vs_observer_projection_max_abs_mm": max(
            abs(value) for value in actual_observer_projection_errors
        )
        if actual_observer_projection_errors
        else math.nan,
        "actual_replay_endpoint_state_max_abs": max(actual_endpoint_state_errors)
        if actual_endpoint_state_errors
        else math.nan,
        "horizon_minus_selection_p50_ms": percentile(header_minus_selection_ms, 0.50),
        "horizon_minus_selection_p95_ms": percentile(header_minus_selection_ms, 0.95),
        "horizon_minus_selected_state_p50_ms": percentile(header_minus_state_ms, 0.50),
        "horizon_minus_selected_state_p95_ms": percentile(header_minus_state_ms, 0.95),
        "two_zeta_omega_n": model_two_zeta_omega_n,
        "omega_n_sq": model_omega_n_sq,
    }


def run_self_test() -> None:
    q0 = np.asarray([0.01, -0.02, -0.03, 0.04], dtype=float)
    if not np.allclose(zero_input_state(q0, 0.0, 0.1, 4.0), q0, atol=1e-15):
        raise RuntimeError("zero-input baseline fails tau=0 identity")
    undamped = zero_input_state([1.0, 0.0, 0.0, 0.0], math.pi / 4.0, 0.0, 4.0)
    if abs(undamped[0]) > 1e-12 or abs(undamped[1] + 2.0) > 1e-12:
        raise RuntimeError("zero-input baseline fails analytic oscillator check")
    series = NumericSeries.from_records([(0.0, [0.0]), (1.0, [2.0])])
    value, span, _, _ = series.interpolate(0.25, 1.1)
    if value is None or abs(float(value[0]) - 0.5) > 1e-12 or span != 1.0:
        raise RuntimeError("linear interpolation self-test failed")
    missing, _, _, _ = series.interpolate(0.25, 0.5)
    if missing is not None:
        raise RuntimeError("interpolation gap gate self-test failed")
    if availability_class(-0.1, 1e-6) != "hindcast":
        raise RuntimeError("hindcast classification self-test failed")
    if availability_class(0.1, 1e-6) != "causal":
        raise RuntimeError("causal classification self-test failed")

    class SyntheticHorizon:
        horizon_steps = 1
        dt = 1.0
        eta_x = [0.0, 1.0]
        eta_x_dot = [1.0, 1.0]
        eta_y = [2.0, 2.0]
        eta_y_dot = [0.0, 0.0]

    dense = interpolate_horizon_q(SyntheticHorizon(), 0.25)
    if not np.allclose(dense, [0.25, 1.0, 2.0, 0.0], atol=1e-12):
        raise RuntimeError("cubic Hermite dense-horizon self-test failed")
    if int(math.floor(0.032 / (1.0 / 30.0))) != 0:
        raise RuntimeError("causal lead floor-bin self-test failed")
    if int(math.floor(-0.001 / (1.0 / 30.0))) != -1:
        raise RuntimeError("negative hindcast floor-bin self-test failed")
    print("[PASS] deterministic self-tests")


def main() -> int:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return 0
    if args.output is None:
        raise RuntimeError("--output is required unless --self-test is used")
    output_dir = args.output.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=False)

    lag_modes = (
        ("primary_delta0", 0.0),
        ("sensitivity_plus_lag", 1.0e-3 * float(args.sensitivity_lag_ms)),
    )
    observer_max_gap_sec = 1.0e-3 * float(args.observer_max_gap_ms)
    causal_epsilon_sec = 1.0e-3 * float(args.causal_epsilon_ms)
    if observer_max_gap_sec <= 0.0 or causal_epsilon_sec < 0.0:
        raise RuntimeError("gap and causal epsilon must be non-negative")

    training_discovered = discover_training_runs(args)
    training_runs = []
    for bag_path, postflight_path in training_discovered:
        print("[read training] {}".format(bag_path))
        training_runs.append(read_run(bag_path, postflight_path))
    projection_fits = fit_projection_models(
        training_runs, lag_modes, observer_max_gap_sec
    )
    projection_fit_payload = {
        "schema_version": 1,
        "fit_scope": "Bsmooth_train_only_native_RGB_events",
        "rgb_target": "(height_R-height_L)-per_run_pre_motion_median",
        "baseline_window_sec": 3.0,
        "fit_window": "motion_start..window_end",
        "observer_time": "RGB source stamp + observer_rgb_alignment",
        "observer_max_bracketing_span_sec": observer_max_gap_sec,
        "fits": projection_fits,
        "training_inputs": [
            {
                "bag": str(run.bag_path),
                "postflight": str(run.postflight_path),
                "bag_sha256_postflight": run.postflight.get("bag_sha256"),
                "postflight_sha256": sha256_file(run.postflight_path),
            }
            for run in training_runs
        ],
    }
    projection_fit_path = output_dir / "projection_fit.json"
    projection_fit_path.write_text(
        json.dumps(projection_fit_payload, indent=2, sort_keys=True, ensure_ascii=False)
        + "\n",
        encoding="utf-8",
    )
    discovered = discover_runs(args)

    metrics = MetricStore()
    availability: Dict[Tuple[str, str, int], AvailabilityAccumulator] = {}
    run_summaries: List[Dict[str, Any]] = []
    input_manifest: List[Dict[str, Any]] = []

    node_path = output_dir / "per_cycle_node.csv"
    event_path = output_dir / "per_cycle_rgb_event.csv"
    with node_path.open("w", encoding="utf-8", newline="") as node_stream, event_path.open(
        "w", encoding="utf-8", newline=""
    ) as event_stream:
        node_writer = csv.DictWriter(node_stream, fieldnames=list(NODE_FIELDS))
        event_writer = csv.DictWriter(event_stream, fieldnames=list(EVENT_FIELDS))
        node_writer.writeheader()
        event_writer.writeheader()
        for bag_path, postflight_path in discovered:
            print("[read] {}".format(bag_path))
            run = read_run(bag_path, postflight_path)
            summary = analyze_run(
                run,
                node_writer,
                event_writer,
                metrics,
                availability,
                projection_fits,
                lag_modes,
                observer_max_gap_sec,
                causal_epsilon_sec,
            )
            run_summaries.append(summary)
            bound_sha = str(run.postflight.get("bag_sha256", ""))
            actual_sha = sha256_file(bag_path) if args.hash_bags else None
            if actual_sha is not None and actual_sha != bound_sha:
                raise RuntimeError("bag SHA-256 differs from postflight: {}".format(bag_path))
            input_manifest.append(
                {
                    "bag": str(bag_path),
                    "bag_size_bytes": bag_path.stat().st_size,
                    "bag_sha256_postflight": bound_sha,
                    "bag_sha256_recomputed": actual_sha,
                    "postflight": str(postflight_path),
                    "postflight_sha256": sha256_file(postflight_path),
                    "row": run.postflight["row"],
                    "block": run.postflight["block"],
                }
            )

    metric_rows = metrics.rows()
    write_csv(
        output_dir / "per_run_bin_metrics.csv",
        METRIC_FIELDS,
        (row for row in metric_rows if row["run_id"] != "ALL_POOLED"),
    )
    write_csv(
        output_dir / "per_bin_metrics.csv",
        METRIC_FIELDS,
        (row for row in metric_rows if row["run_id"] == "ALL_POOLED"),
    )

    availability_rows = []
    for (run_id, lag_mode, bin_j), accumulator in sorted(availability.items()):
        availability_rows.append(
            accumulator.row(run_id, lag_mode, bin_j, bin_j / 30.0)
        )
    availability_fields = list(availability_rows[0]) if availability_rows else []
    write_csv(output_dir / "availability_ledger.csv", availability_fields, availability_rows)

    run_summary_fields = list(run_summaries[0]) if run_summaries else []
    write_csv(output_dir / "run_summary.csv", run_summary_fields, run_summaries)

    script_path = Path(__file__).resolve()
    replay_module_path = script_path.with_name("horizon_liquid_replay.py")
    manifest = {
        "schema_version": 1,
        "report_type": "G3R2_HORIZON_FUTURE_LIQUID_ALIGNMENT",
        "status": "DESCRIPTIVE_POST_HOC",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "script": str(script_path),
        "script_sha256": sha256_file(script_path),
        "replay_module": str(replay_module_path),
        "replay_module_sha256": sha256_file(replay_module_path),
        "workspace_git_revision": current_git_revision(),
        "inputs": input_manifest,
        "configuration": {
            "projection": projection_fits,
            "lag_modes": [
                {"name": name, "observer_rgb_alignment_sec": delta}
                for name, delta in lag_modes
            ],
            "target_time_contract": {
                "q_target": "selection.selected_state_stamp + j*dt",
                "rgb_node_target": "q_target - observer_rgb_alignment",
                "rgb_event_state_equivalent": "rgb_source_stamp + observer_rgb_alignment",
                "forecast_available": "PredictedHorizon.header.stamp",
            },
            "observer_max_bracketing_span_sec": observer_max_gap_sec,
            "rgb_interpolation": "FORBIDDEN_NATIVE_EVENTS_ONLY",
            "causal_epsilon_sec": causal_epsilon_sec,
            "origin_admission": "horizon.header in [motion_start, motion_end]",
            "pair_method": "legacy global sequence plus matching header.seq and exact horizon-q0/observer identity",
            "baselines": {
                "rgb_persistence": "last clean RGB message arrived by horizon header",
                "hold_q0": "selected liquid q held constant",
                "zero_input": "exact nominal free response from q0 using snapshot parameters",
                "planned_high_accuracy": "piecewise a/alpha RK4 replay with <=3.333ms substeps",
                "actual_input_replay": "software-exact irregular-dt observer ZOH replay; retrospective only",
            },
        },
        "outputs": {
            "per_cycle_node": str(node_path),
            "per_cycle_rgb_event": str(event_path),
            "availability_ledger": str(output_dir / "availability_ledger.csv"),
            "per_run_bin_metrics": str(output_dir / "per_run_bin_metrics.csv"),
            "per_bin_metrics": str(output_dir / "per_bin_metrics.csv"),
            "run_summary": str(output_dir / "run_summary.csv"),
            "projection_fit": str(projection_fit_path),
        },
        "limitations": [
            "Overlapping control cycles are not independent statistical samples.",
            "Horizon header is a post-solve publication stamp, not an authoritative solver origin.",
            "fixed_robot_only robot and liquid x0 do not share a physical epoch.",
            "Single-camera signed slope validates only one fitted liquid projection.",
            "Projection and +12 ms lag are post-hoc development quantities.",
            "Actual-input replay uses future processed IMU and is not an online forecast.",
            "Raw planned and processed-IMU excitation columns differ in reference point and time semantics; they are not subtracted as an actuator mismatch metric.",
            "Per-node cost mass is not a first-action gradient or KKT sensitivity.",
        ],
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    summary_payload = {
        "status": "PASS_DESCRIPTIVE_OUTPUT_BUILT",
        "run_summaries": run_summaries,
        "availability_ledger_rows": len(availability_rows),
        "per_run_metric_rows": sum(row["run_id"] != "ALL_POOLED" for row in metric_rows),
        "pooled_metric_rows": sum(row["run_id"] == "ALL_POOLED" for row in metric_rows),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary_payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print("[PASS] wrote {}".format(output_dir))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001 - fail closed with a concise CLI error.
        print("[FAIL] {}".format(exc), file=sys.stderr)
        sys.exit(2)
