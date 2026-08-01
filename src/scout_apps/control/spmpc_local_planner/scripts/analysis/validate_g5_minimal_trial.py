#!/usr/bin/env python3
"""Validate one image-free G5 SmoothMatch or FixedProfile real trial."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np
import rosbag


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def topic_is_image(name: str, message_type: str) -> bool:
    return message_type in {"sensor_msgs/Image", "sensor_msgs/CompressedImage"} or any(
        token in name.lower() for token in ("image_raw", "image_rect", "compressed", "debug_image", "/depth/")
    )


def layout_map(msg: Any) -> Dict[str, float]:
    if not msg.layout.dim:
        return {}
    names = [value.strip() for value in msg.layout.dim[0].label.split(",")]
    return {
        name: float(msg.data[index])
        for index, name in enumerate(names)
        if name and index < len(msg.data)
    }


def load_g4_module() -> Any:
    path = Path(__file__).resolve().with_name("g4_replay_from_g3.py")
    spec = importlib.util.spec_from_file_location("g4_for_g5", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def read_bag(bag_path: Path) -> Dict[str, Any]:
    commands: List[Tuple[float, float, float]] = []
    statuses: List[Tuple[float, str]] = []
    effective: List[Tuple[float, Dict[str, float]]] = []
    interventions: List[Tuple[float, Dict[str, float]]] = []
    external_modes: List[str] = []
    controller_variants: List[str] = []
    profile_caps: List[Tuple[float, float]] = []
    reference_s: List[Tuple[float, float]] = []
    reference_raw: List[Tuple[float, float]] = []
    reference_eff: List[Tuple[float, float]] = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        start = float(bag.get_start_time())
        end = float(bag.get_end_time())
        info = bag.get_type_and_topic_info().topics
        image_topics = [
            {"topic": topic, "type": item.msg_type, "messages": int(item.message_count)}
            for topic, item in info.items()
            if item.message_count and topic_is_image(topic, item.msg_type)
        ]
        for topic, msg, bag_stamp in bag.read_messages(
            topics=[
                "/cmd_vel",
                "/spmpc/status",
                "/spmpc/debug/effective_config",
                "/spmpc/debug/command_intervention",
                "/mpc_status",
                "/diagnostics/external_profile_mode",
                "/diagnostics/controller_variant",
                "/profile_cap/v_profile",
                "/reference/s",
                "/reference/v_des_raw",
                "/reference/v_des_eff",
            ]
        ):
            stamp = float(bag_stamp.to_sec())
            if topic == "/cmd_vel":
                commands.append((stamp, float(msg.linear.x), float(msg.angular.z)))
            elif topic in {"/spmpc/status", "/mpc_status"}:
                statuses.append((stamp, str(msg.data)))
            elif topic == "/spmpc/debug/effective_config":
                effective.append((stamp, layout_map(msg)))
            elif topic == "/spmpc/debug/command_intervention":
                interventions.append((stamp, layout_map(msg)))
            elif topic == "/diagnostics/external_profile_mode":
                external_modes.append(str(msg.data))
            elif topic == "/diagnostics/controller_variant":
                controller_variants.append(str(msg.data))
            elif topic == "/profile_cap/v_profile":
                profile_caps.append((stamp, float(msg.data)))
            elif topic == "/reference/s":
                reference_s.append((stamp, float(msg.data)))
            elif topic == "/reference/v_des_raw":
                reference_raw.append((stamp, float(msg.data)))
            elif topic == "/reference/v_des_eff":
                reference_eff.append((stamp, float(msg.data)))
    return {
        "start": start,
        "end": end,
        "duration": end - start,
        "commands": commands,
        "statuses": statuses,
        "effective": effective,
        "interventions": interventions,
        "external_modes": external_modes,
        "controller_variants": controller_variants,
        "profile_caps": profile_caps,
        "reference_s": reference_s,
        "reference_raw": reference_raw,
        "reference_eff": reference_eff,
        "image_topics": image_topics,
        "topic_counts": {topic: int(item.message_count) for topic, item in info.items()},
    }


def first_arrival(statuses: Sequence[Tuple[float, str]], condition: str, motion_start: float) -> float:
    accepted = {"GOAL_REACHED"} if condition == "SmoothMatch" else {"REACHED"}
    values = [stamp for stamp, status in statuses if stamp >= motion_start and status in accepted]
    return min(values) if values else math.nan


def values_in_window(samples: Sequence[Tuple[float, float]], start: float, end: float) -> List[float]:
    return [value for stamp, value in samples if start <= stamp <= end]


def validate(args: argparse.Namespace) -> Tuple[Mapping[str, Any], int]:
    bag_path = Path(args.bag).expanduser().resolve()
    path_file = Path(args.path_file).expanduser().resolve()
    config_file = Path(args.config_file).expanduser().resolve()
    failures: List[str] = []
    if not bag_path.is_file():
        raise RuntimeError("bag is missing: {}".format(bag_path))
    if Path(str(bag_path) + ".active").exists():
        failures.append("bag still has .active sidecar")
    if sha256_file(path_file) != args.path_sha256:
        failures.append("path hash mismatch")
    if sha256_file(config_file) != args.config_sha256:
        failures.append("condition config hash mismatch")

    data = read_bag(bag_path)
    commands = data["commands"]
    moving = [
        (stamp, v, omega)
        for stamp, v, omega in commands
        if abs(v) > args.motion_v_threshold or abs(omega) > args.motion_omega_threshold
    ]
    if not moving:
        failures.append("bag contains no effective motion")
        motion_start = math.nan
        motion_end = math.nan
    else:
        motion_start = moving[0][0]
        motion_end = moving[-1][0]
    arrival = first_arrival(data["statuses"], args.condition, motion_start)
    if not math.isfinite(arrival):
        failures.append("goal/reached status is missing")
    completion = arrival - motion_start if math.isfinite(arrival) and math.isfinite(motion_start) else math.nan
    tail = data["end"] - arrival if math.isfinite(arrival) else math.nan
    if data["duration"] > args.max_duration_sec + 0.5:
        failures.append("bag duration exceeds maximum")
    if math.isfinite(tail) and tail < args.minimum_tail_sec:
        failures.append("post-arrival tail is shorter than frozen minimum")
    if data["image_topics"]:
        failures.append("bag contains forbidden image stream(s)")

    peak_v = max((abs(v) for _, v, _ in commands), default=0.0)
    peak_omega = max((abs(omega) for _, _, omega in commands), default=0.0)
    if peak_v > args.v_max + 1e-3:
        failures.append("published linear speed exceeds hard limit")
    if peak_omega > args.omega_max + 1e-3:
        failures.append("published angular speed exceeds hard limit")
    acceleration = []
    angular_acceleration = []
    for index in range(1, len(commands)):
        dt = commands[index][0] - commands[index - 1][0]
        if 1e-4 < dt < 0.5:
            acceleration.append((commands[index][1] - commands[index - 1][1]) / dt)
            angular_acceleration.append((commands[index][2] - commands[index - 1][2]) / dt)
    peak_a = max((abs(value) for value in acceleration), default=0.0)
    peak_alpha = max((abs(value) for value in angular_acceleration), default=0.0)
    # Published emergency/final zero is intentionally excluded from the strict
    # derivative gate; the method-native continuous motion portion is audited by
    # the profile/command-intervention topics below.

    condition_metrics: Dict[str, Any] = {}
    if args.condition == "SmoothMatch":
        if not data["effective"]:
            failures.append("SmoothMatch lacks effective-config snapshots")
        else:
            last_config = data["effective"][-1][1]
            v_ref = last_config.get("v_ref", math.nan)
            if not math.isfinite(v_ref) or abs(v_ref - args.expected_v_ref) > 1e-4:
                failures.append("SmoothMatch effective v_ref mismatch")
            if abs(last_config.get("w_slosh", math.nan)) > 1e-6:
                failures.append("SmoothMatch effective w_slosh is not zero")
            if last_config.get("slosh_enable", math.nan) != 0.0:
                failures.append("SmoothMatch unexpectedly enables slosh OCP")
            condition_metrics["effective_config_last"] = last_config
        unsafe = {"zero_due_to_solver_failure", "zero_due_to_terminal_spin_fail", "zero_due_to_tracking_safety"}
        unsafe_counts = {
            name: sum(item.get(name, 0.0) > 0.5 for _, item in data["interventions"])
            for name in unsafe
        }
        if any(unsafe_counts.values()):
            failures.append("SmoothMatch has unsafe command intervention")
        condition_metrics["unsafe_zero_counts"] = unsafe_counts
    else:
        if set(data["external_modes"]) != {"custom_csv"}:
            failures.append("FixedProfile external_profile_mode is not uniquely custom_csv")
        if set(data["controller_variants"]) != {"mpc"}:
            failures.append("FixedProfile tracker controller_variant is not uniquely mpc")
        cap_values = values_in_window(data["profile_caps"], motion_start, arrival)
        reference_values = values_in_window(data["reference_s"], motion_start, arrival)
        command_count = sum(motion_start <= stamp <= arrival for stamp, _, _ in commands)
        cap_coverage = len(cap_values) / max(1, command_count)
        reference_coverage = len(reference_values) / max(1, command_count)
        if cap_coverage < args.minimum_profile_coverage:
            failures.append("FixedProfile profile-cap coverage is below minimum")
        if reference_coverage < args.minimum_profile_coverage:
            failures.append("FixedProfile reference-progress coverage is below minimum")
        condition_metrics.update(
            {
                "profile_cap_coverage": cap_coverage,
                "reference_progress_coverage": reference_coverage,
                "profile_cap_peak_m_s": max(cap_values, default=math.nan),
                "reference_progress_last": reference_values[-1] if reference_values else math.nan,
                "raw_reference_sample_count": len(
                    values_in_window(data["reference_raw"], motion_start, arrival)
                ),
                "effective_reference_sample_count": len(
                    values_in_window(data["reference_eff"], motion_start, arrival)
                ),
            }
        )

    tracking: Mapping[str, Any] = {}
    if math.isfinite(motion_start) and math.isfinite(arrival):
        g4 = load_g4_module()
        trajectory_config = {
            "zone_boundaries_sigma": [0.0, 0.16, 0.37, 0.64, 0.87, 1.0],
            "tf_max_nearest_gap_sec": 0.10,
            "monotonic_backtrack_tolerance_m": 0.05,
            "progress_jump_margin_m": 0.10,
            "resample_hz": 50.0,
            "smoothing_window_samples": 5,
            "minimum_motion_samples": 100,
        }
        _rows, tracking = g4.extract_trajectory(
            bag_path,
            g4.load_path_xy(path_file),
            motion_start,
            arrival,
            trajectory_config,
        )
        if tracking["cross_track_p95_m"] > args.maximum_cross_track_p95_m:
            failures.append("odometry-derived cross-track P95 exceeds limit")
        if tracking["tf_gap_over_limit_count"]:
            failures.append("trajectory TF gap exceeds limit")

    report = {
        "schema_version": 1,
        "report_type": "G5_MINIMAL_TRIAL_POSTFLIGHT",
        "condition": args.condition,
        "row": args.row,
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "bag": str(bag_path),
        "bag_sha256": sha256_file(bag_path),
        "bag_size_bytes": bag_path.stat().st_size,
        "duration_sec": data["duration"],
        "motion_start_sec": motion_start,
        "motion_end_sec": motion_end,
        "first_arrival_sec": arrival,
        "completion_sec": completion,
        "post_arrival_tail_sec": tail,
        "image_stream_audit": {"count": len(data["image_topics"]), "topics": data["image_topics"]},
        "published_command": {
            "sample_count": len(commands),
            "peak_v_m_s": peak_v,
            "peak_omega_rad_s": peak_omega,
            "naive_peak_a_m_s2_including_gates": peak_a,
            "naive_peak_alpha_rad_s2_including_gates": peak_alpha,
        },
        "tracking": tracking,
        "condition_metrics": condition_metrics,
        "bindings": {
            "path_file": str(path_file),
            "path_sha256": args.path_sha256,
            "condition_config": str(config_file),
            "condition_config_sha256": args.config_sha256,
            "g5_prereg_sha256": args.g5_prereg_sha256,
            "profile_sha256": args.profile_sha256,
        },
        "topic_counts": data["topic_counts"],
    }
    out_path = Path(args.out_json).expanduser().resolve()
    out_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print("[G5 postflight] {} {}".format(args.condition, report["status"]))
    print("  completion={:.3f}s tail={:.3f}s".format(completion, tail))
    print("  report={}".format(out_path))
    for failure in failures:
        print("  FAIL: {}".format(failure), file=sys.stderr)
    return report, 0 if not failures else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--condition", choices=("SmoothMatch", "FixedProfile"), required=True)
    parser.add_argument("--row", required=True)
    parser.add_argument("--path-file", required=True)
    parser.add_argument("--path-sha256", required=True)
    parser.add_argument("--config-file", required=True)
    parser.add_argument("--config-sha256", required=True)
    parser.add_argument("--g5-prereg-sha256", required=True)
    parser.add_argument("--profile-sha256", default="")
    parser.add_argument("--expected-v-ref", type=float, default=0.0)
    parser.add_argument("--out-json", required=True)
    parser.add_argument("--max-duration-sec", type=float, default=70.0)
    parser.add_argument("--minimum-tail-sec", type=float, default=5.0)
    parser.add_argument("--motion-v-threshold", type=float, default=0.01)
    parser.add_argument("--motion-omega-threshold", type=float, default=0.02)
    parser.add_argument("--v-max", type=float, default=0.8)
    parser.add_argument("--omega-max", type=float, default=1.2)
    parser.add_argument("--minimum-profile-coverage", type=float, default=0.90)
    parser.add_argument("--maximum-cross-track-p95-m", type=float, default=0.10)
    return parser.parse_args()


def main() -> int:
    _report, code = validate(parse_args())
    return code


if __name__ == "__main__":
    raise SystemExit(main())
