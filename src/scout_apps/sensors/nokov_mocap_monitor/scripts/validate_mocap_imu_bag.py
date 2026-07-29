#!/usr/bin/env python3
"""Postflight integrity checks for a planar mocap/IMU sequence bag."""

import argparse
import json
import math
import re
import sys
from pathlib import Path

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--mocap-pose-topic", required=True)
    parser.add_argument("--segment-topic", default="/mocap_imu_calib/segment")
    parser.add_argument("--status-topic", default="/mocap_imu_calib/status")
    return parser.parse_args()


def load_config(path):
    config = {}
    with open(path, "r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            config[key] = value
    return config


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def event_parts(event):
    parts = event.split("|")
    fields = {}
    for part in parts[2:]:
        if "=" in part:
            key, value = part.split("=", 1)
            fields[key] = value
    return parts, fields


class StreamStats:
    def __init__(self, has_header):
        self.has_header = has_header
        self.count = 0
        self.first_bag_time = None
        self.last_bag_time = None
        self.max_bag_gap = 0.0
        self.last_header_time = None
        self.max_header_gap = 0.0
        self.header_regressions = 0
        self.invalid_header_stamps = 0
        self.nonfinite_messages = 0

    def update(self, bag_time, header_time, values):
        if self.last_bag_time is not None:
            self.max_bag_gap = max(self.max_bag_gap, bag_time - self.last_bag_time)
        if self.first_bag_time is None:
            self.first_bag_time = bag_time
        self.last_bag_time = bag_time
        self.count += 1
        if not finite(values):
            self.nonfinite_messages += 1
        if self.has_header:
            if not math.isfinite(header_time) or header_time <= 0.0:
                self.invalid_header_stamps += 1
            elif self.last_header_time is not None:
                delta = header_time - self.last_header_time
                if delta < -1e-9:
                    self.header_regressions += 1
                else:
                    self.max_header_gap = max(self.max_header_gap, delta)
            self.last_header_time = header_time

    def duration(self):
        if self.first_bag_time is None or self.last_bag_time is None:
            return 0.0
        return max(0.0, self.last_bag_time - self.first_bag_time)

    def rate(self):
        duration = self.duration()
        return (self.count - 1) / duration if self.count > 1 and duration > 0.0 else 0.0

    def as_dict(self):
        return {
            "count": self.count,
            "duration_sec": self.duration(),
            "rate_hz": self.rate(),
            "max_bag_gap_sec": self.max_bag_gap,
            "max_header_gap_sec": self.max_header_gap,
            "header_regressions": self.header_regressions,
            "invalid_header_stamps": self.invalid_header_stamps,
            "nonfinite_messages": self.nonfinite_messages,
        }


def main():
    args = parse_args()
    config = load_config(args.config)
    errors = []
    topics = {
        "cmd": args.cmd_topic,
        "imu": args.imu_topic,
        "odom": args.odom_topic,
        "mocap": args.mocap_pose_topic,
        "segment": args.segment_topic,
        "status": args.status_topic,
    }
    stats = {
        "cmd": StreamStats(False),
        "imu": StreamStats(True),
        "odom": StreamStats(True),
        "mocap": StreamStats(True),
    }
    events = []
    statuses = []
    last_cmds = []
    cmd_has_positive_v = False
    cmd_has_negative_v = False
    cmd_has_positive_w = False
    cmd_has_negative_w = False
    cmd_limit_violations = 0

    bag_path = Path(args.bag)
    if not bag_path.is_file():
        errors.append("bag file is missing")
    elif Path(str(bag_path) + ".active").exists():
        errors.append("bag still has a .active sibling")

    if not errors:
        with rosbag.Bag(str(bag_path), "r") as bag:
            available_topics = set(bag.get_type_and_topic_info().topics.keys())
            for label, topic in topics.items():
                if topic not in available_topics:
                    errors.append("missing required topic {} ({})".format(topic, label))

            wanted = list(topics.values())
            for topic, message, stamp in bag.read_messages(topics=wanted):
                bag_time = stamp.to_sec()
                if topic == args.cmd_topic:
                    values = (message.linear.x, message.angular.z)
                    stats["cmd"].update(bag_time, 0.0, values)
                    v, w = values
                    cmd_has_positive_v |= v > 1e-6
                    cmd_has_negative_v |= v < -1e-6
                    cmd_has_positive_w |= w > 1e-6
                    cmd_has_negative_w |= w < -1e-6
                    if abs(v) > 0.250001 or abs(w) > 0.500001:
                        cmd_limit_violations += 1
                    last_cmds.append(values)
                    if len(last_cmds) > 20:
                        last_cmds.pop(0)
                elif topic == args.imu_topic:
                    a = message.linear_acceleration
                    w = message.angular_velocity
                    stats["imu"].update(
                        bag_time,
                        message.header.stamp.to_sec(),
                        (a.x, a.y, a.z, w.x, w.y, w.z),
                    )
                elif topic == args.odom_topic:
                    pose = message.pose.pose
                    twist = message.twist.twist
                    stats["odom"].update(
                        bag_time,
                        message.header.stamp.to_sec(),
                        (
                            pose.position.x,
                            pose.position.y,
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                            twist.linear.x,
                            twist.angular.z,
                        ),
                    )
                elif topic == args.mocap_pose_topic:
                    pose = message.pose
                    stats["mocap"].update(
                        bag_time,
                        message.header.stamp.to_sec(),
                        (
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ),
                    )
                elif topic == args.segment_topic:
                    events.append((bag_time, message.data))
                elif topic == args.status_topic:
                    statuses.append((bag_time, message.data))

    age_limits = {
        "imu": float(config.get("imu_max_age", "0.20")),
        "odom": float(config.get("odom_max_age", "0.20")),
        "mocap": float(config.get("mocap_pose_max_age", "0.20")),
        "cmd": float(config.get("max_publish_gap_sec", "0.10")),
    }
    minimum_rates = {"imu": 40.0, "odom": 40.0, "mocap": 60.0, "cmd": 45.0}
    for label, stream in stats.items():
        if stream.count == 0:
            errors.append("{} has no messages".format(label))
            continue
        if stream.rate() < minimum_rates[label]:
            errors.append(
                "{} rate {:.2f}Hz is below {:.2f}Hz".format(
                    label, stream.rate(), minimum_rates[label]
                )
            )
        if stream.max_bag_gap > age_limits[label] + 1e-6:
            errors.append(
                "{} bag gap {:.3f}s exceeds {:.3f}s".format(
                    label, stream.max_bag_gap, age_limits[label]
                )
            )
        if stream.has_header and stream.max_header_gap > age_limits[label] + 1e-6:
            errors.append(
                "{} header gap {:.3f}s exceeds {:.3f}s".format(
                    label, stream.max_header_gap, age_limits[label]
                )
            )
        if stream.nonfinite_messages:
            errors.append("{} has non-finite messages".format(label))
        if stream.has_header and (
            stream.header_regressions or stream.invalid_header_stamps
        ):
            errors.append("{} has invalid/non-monotonic header timestamps".format(label))

    expected_statuses = ["READY", "RUNNING", "COMPLETE"]
    status_values = [value for _stamp, value in statuses]
    if status_values != expected_statuses:
        errors.append("status sequence is {}, expected {}".format(status_values, expected_statuses))

    event_values = [value for _stamp, value in events]
    if event_values.count("SEQUENCE_START") != 1 or event_values.count("SEQUENCE_END") != 1:
        errors.append("SEQUENCE_START/SEQUENCE_END markers are incomplete")
    if any(value.startswith("SEQUENCE_ABORT|") for value in event_values):
        errors.append("bag contains SEQUENCE_ABORT")

    static_minimum = {
        "static_pre": float(config.get("static_pre_sec", "60")),
        "static_post": float(config.get("static_post_sec", "60")),
    }
    for label, expected in static_minimum.items():
        matches = [value for value in event_values if value.startswith("END|{}|".format(label))]
        if len(matches) != 1:
            errors.append("{} END marker count is {}".format(label, len(matches)))
            continue
        _parts, fields = event_parts(matches[0])
        actual = float(fields.get("actual_duration", "nan"))
        if not math.isfinite(actual) or actual + 0.05 < expected:
            errors.append("{} actual duration {:.3f}s is too short".format(label, actual))

    repeats = int(config.get("s_repeats", "3"))
    s_hold = float(config.get("s_hold_sec", "1.0"))
    s_start_re = re.compile(r"^START\|s_(LR|RL)_r\d+_(forward|return)\|")
    s_end_re = re.compile(r"^END\|s_(LR|RL)_r\d+_(forward|return)\|")
    s_switch_re = re.compile(r"^SWITCH\|s_(LR|RL)_r\d+_(forward|return)_reversal\|")
    s_starts = [value for value in event_values if s_start_re.match(value)]
    s_ends = [value for value in event_values if s_end_re.match(value)]
    s_switches = [value for value in event_values if s_switch_re.match(value)]
    expected_s_passes = 4 * repeats
    if (len(s_starts), len(s_switches), len(s_ends)) != (
        expected_s_passes,
        expected_s_passes,
        expected_s_passes,
    ):
        errors.append(
            "S marker counts start/switch/end={}/{}/{}, expected {} each".format(
                len(s_starts), len(s_switches), len(s_ends), expected_s_passes
            )
        )
    for value in s_switches:
        _parts, fields = event_parts(value)
        actual = float(fields.get("previous_actual_duration", "nan"))
        if not math.isfinite(actual) or abs(actual - s_hold) > 0.05:
            errors.append("S first leg duration is out of tolerance: {}".format(value))
    for value in s_ends:
        _parts, fields = event_parts(value)
        actual = float(fields.get("actual_duration", "nan"))
        if not math.isfinite(actual) or abs(actual - 2.0 * s_hold) > 0.08:
            errors.append("S pass duration is out of tolerance: {}".format(value))

    if cmd_limit_violations:
        errors.append("cmd_vel contains {} hard-limit violations".format(cmd_limit_violations))
    if not all(
        (cmd_has_positive_v, cmd_has_negative_v, cmd_has_positive_w, cmd_has_negative_w)
    ):
        errors.append("cmd_vel does not contain all positive/negative linear/angular excitations")
    if len(last_cmds) < 10 or not all(abs(v) < 1e-12 and abs(w) < 1e-12 for v, w in last_cmds[-10:]):
        errors.append("bag does not end with at least 10 zero commands")

    report = {
        "ok": not errors,
        "bag": str(bag_path),
        "errors": errors,
        "streams": {label: stream.as_dict() for label, stream in stats.items()},
        "status_values": status_values,
        "event_count": len(events),
        "s_pass_count": len(s_starts),
        "cmd_limit_violations": cmd_limit_violations,
    }
    with open(args.report, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")

    if errors:
        for error in errors:
            print("[validate_mocap_imu_bag] ERROR: {}".format(error), file=sys.stderr)
        return 1
    print("[validate_mocap_imu_bag] PASS: {}".format(args.bag))
    return 0


if __name__ == "__main__":
    sys.exit(main())
