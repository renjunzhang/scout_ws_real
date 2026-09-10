#!/usr/bin/env python3
"""Check common raw evidence for liquid-source comparisons, without ROS nodes.

Native sensor/state stamps define coverage; bag receipt times never stand in
for measurement times. This gate verifies recording, not physical calibration
or liquid-height efficacy. Images are consumed one at a time, not retained.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

import rosbag

AUDIT = "/spmpc/debug/control_cycle_audit"
IMU = "/spmpc/debug/slosh_observer_imu"
ODOM = "/spmpc/debug/slosh_observer_odom"
IMAGE = "/camera/color/image_raw"
INFO = "/camera/color/camera_info"


def seconds(stamp):
    return float(stamp.to_sec())


def stream_row(topic, message):
    """Keep only timing, health and geometry metadata from each message."""
    row = {"stamp": seconds(message.header.stamp),
           "header_stamp": seconds(message.header.stamp),
           "frame": message.header.frame_id, "valid": True}
    if topic in (IMU, ODOM):
        row.update(stamp=seconds(message.state_stamp),
                   measurement_stamp=seconds(message.measurement_stamp),
                   epoch=int(message.reset_epoch), update=int(message.observer_update_count),
                   reference=message.excitation_reference_point,
                   axes=message.excitation_axes_frame)
        row["valid"] = (
            bool(message.configured) and bool(message.valid)
            and int(message.source) == (2 if topic == IMU else 1)
            and message.input_status == ("READY" if topic == IMU else "ODOM_READY")
            and (topic != IMU or (message.bias_ready and message.filter_ready))
            and all(math.isfinite(float(getattr(message, name))) for name in
                    ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot", "modal_height_m",
                     "ax_mps2", "ay_mps2"))
        )
    elif topic == IMAGE:
        row["geometry"] = (int(message.width), int(message.height), message.encoding)
        bytes_per_pixel = {"rgb8": 3, "bgr8": 3}.get(message.encoding, 0)
        row["valid"] = (bytes_per_pixel > 0 and message.width > 0 and message.height > 0
                        and message.step >= bytes_per_pixel * message.width
                        and len(message.data) == message.height * message.step)
    elif topic == INFO:
        row["geometry"] = (int(message.width), int(message.height))
        calibration = [message.distortion_model, list(message.D), list(message.K),
                       list(message.R), list(message.P), message.binning_x, message.binning_y,
                       [message.roi.x_offset, message.roi.y_offset, message.roi.height,
                        message.roi.width, message.roi.do_rectify]]
        row["camera_info_sha256"] = hashlib.sha256(
            json.dumps(calibration, allow_nan=False).encode()).hexdigest()
        row["valid"] = (message.width > 0 and message.height > 0
                        and message.K[0] > 0 and message.K[4] > 0)
    else:
        p, q = message.pose.position, message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        row["valid"] = (all(math.isfinite(v) for v in values)
                        and abs(sum(v*v for v in values[3:]) - 1.0) < .02)
    return row


def check_stream(rows, start, stop, max_gap, monitor=False, expected_geometry=None):
    failures = []
    if monitor:
        # Startup bias/filter messages legitimately have no liquid state yet.
        # During the task, use their declared header as a second inclusion
        # criterion so an invalid/zero state stamp cannot hide a bad update.
        rows = [r for r in rows if start-max_gap <= r["stamp"] <= stop+max_gap
                or start <= r["header_stamp"] <= stop]
    stamps = [r["stamp"] for r in rows]
    if any(not math.isfinite(t) or t <= 0 for t in stamps):
        return {"failures": ["invalid native timestamp"], "count": len(rows)}
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        return {"failures": ["duplicate/regressing native timestamp"], "count": len(rows)}
    selected = [r for r in rows if start <= r["stamp"] <= stop]
    ts = [r["stamp"] for r in selected]
    if len(ts) < 2:
        return {"failures": ["missing task/tail samples"], "count": len(ts)}
    gaps = [b-a for a, b in zip(ts, ts[1:])]
    if ts[0]-start > max_gap or stop-ts[-1] > max_gap:
        failures.append("incomplete task/tail boundary coverage")
    if max(gaps) > max_gap + 1e-6:
        failures.append("native sampling gap exceeds recording limit")
    if any(not r["valid"] for r in selected):
        failures.append("invalid sensor/observer sample")
    frames = sorted({r["frame"] for r in selected})
    if len(frames) != 1 or not frames[0]:
        failures.append("missing/changing frame")
    result = {"count": len(ts), "first_stamp": ts[0], "last_stamp": ts[-1],
              "rate_hz": (len(ts)-1)/(ts[-1]-ts[0]),
              "max_gap_sec": max(gaps), "allowed_gap_sec": max_gap, "frames": frames}
    if monitor:
        if any(abs(r["measurement_stamp"]-r["stamp"]) > 1e-6 for r in selected):
            failures.append("state/measurement timestamp mismatch")
        if len({r["epoch"] for r in selected}) != 1:
            failures.append("observer reset during task/tail")
        if any(b["update"] != a["update"]+1 for a, b in zip(selected, selected[1:])):
            failures.append("observer update skipped or repeated")
        for field in ("axes", "reference"):
            labels = sorted({r[field] for r in selected})
            result[field] = labels
            if len(labels) != 1 or not labels[0]:
                failures.append("missing/changing observer " + field)
        if any(r["axes"] != r["frame"] for r in selected):
            failures.append("observer axes/header frame mismatch")
    if expected_geometry is not None:
        geometries = sorted({tuple(r["geometry"]) for r in selected})
        result["geometries"] = geometries
        if len(geometries) != 1 or geometries[0][:2] != tuple(expected_geometry):
            failures.append("unexpected/changing RGB geometry")
    if "camera_info_sha256" in selected[0]:
        result["camera_info_sha256"] = sorted({r["camera_info_sha256"] for r in selected})
        if len(result["camera_info_sha256"]) != 1:
            failures.append("camera intrinsics changed during task/tail")
    result["failures"] = failures
    return result


def validate_bag(args):
    mocap = "/vrpn_client_node/{}/pose".format(args.tracker)
    rows = {topic: [] for topic in (IMU, ODOM, mocap, IMAGE, INFO)}
    failures, published, goals = [], [], []
    with rosbag.Bag(str(args.bag)) as bag:
        topics = bag.get_type_and_topic_info().topics
        image_topics = [t for t, info in topics.items()
                        if info.msg_type in ("sensor_msgs/Image", "sensor_msgs/CompressedImage")]
        if not args.expect_rgb and image_topics:
            failures.append("images present in explicitly image-free run")
        # Optional metadata remains in the bag, while the image/header and
        # camera_info streams are the required quantitative recording contract.
        for topic, msg, _ in bag.read_messages(topics=[AUDIT] + list(rows)):
            if topic == AUDIT:
                if msg.command_was_published:
                    stamp = seconds(msg.command_publish_stamp)
                    if stamp <= 0 or not math.isfinite(stamp):
                        failures.append("invalid audited publication stamp")
                        continue
                    if abs(msg.published_cmd_v) > 1e-9 or abs(msg.published_cmd_omega) > 1e-9:
                        published.append(stamp)
                    # Match the metric windows: a cycle status alone is not
                    # a published goal event, and callback start is earlier.
                    if msg.solver_status == "GOAL_REACHED":
                        goals.append(stamp)
            else:
                rows[topic].append(stream_row(topic, msg))
    if not published or not goals or min(goals) <= min(published):
        failures.append("missing/invalid first-motion or GOAL_REACHED timestamp")
        return {"status": "FAIL", "failures": failures}
    start, goal = min(published), min(goals)
    stop = goal + args.tail_sec
    streams = {}
    limits = {IMU: .035, ODOM: .050, mocap: .100}
    if args.expect_rgb:
        limits.update({IMAGE: 3.0/args.rgb_fps, INFO: 3.0/args.rgb_fps})
    for topic, limit in limits.items():
        streams[topic] = check_stream(
            rows[topic], start, stop, limit, monitor=topic in (IMU, ODOM),
            expected_geometry=(args.rgb_width, args.rgb_height) if topic in (IMAGE, INFO) else None)
        failures.extend(topic + ": " + reason for reason in streams[topic]["failures"])
    if args.expect_rgb and rows[IMAGE] and rows[INFO]:
        image_stats, info_stats = streams[IMAGE], streams[INFO]
        if image_stats.get("frames") != info_stats.get("frames"):
            failures.append("RGB image/camera_info frame mismatch")
        if image_stats.get("rate_hz", 0) < .9*args.rgb_fps:
            failures.append("RGB rate below 90% of expected rate")
        info_stamps = [r["stamp"] for r in rows[INFO]]
        # Do not sort or repair malformed time series after checking them.
        if all(b > a for a, b in zip(info_stamps, info_stamps[1:])):
            max_skew = 0.0
            for row in rows[IMAGE]:
                if start <= row["stamp"] <= stop:
                    index = bisect.bisect_left(info_stamps, row["stamp"])
                    near = info_stamps[max(0, index-1):index+1]
                    max_skew = max(max_skew, min(abs(t-row["stamp"]) for t in near))
            image_stats["max_camera_info_stamp_skew_sec"] = max_skew
            if max_skew > .5/args.rgb_fps:
                failures.append("RGB image/camera_info timestamps do not match")
    return {"schema": "spmpc_comparison_recording_v2", "bag": str(args.bag),
            "status": "FAIL" if failures else "PASS", "failures": failures,
            "expect_rgb": args.expect_rgb, "task_start_sec": start, "goal_sec": goal,
            "coverage_end_sec": stop, "tail_sec": args.tail_sec, "streams": streams,
            "scope": "recorded native-time coverage only; no physical clock/extrinsic or RGB height calibration",
            "timebase": "audit command_publish_stamp for first motion and published GOAL_REACHED; observer state_stamp; raw image/mocap header.stamp"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--expect-rgb", choices=("true", "false"), required=True)
    parser.add_argument("--tracker", default="Tracker0")
    parser.add_argument("--tail-sec", type=float, default=5.)
    parser.add_argument("--rgb-width", type=int, default=1920)
    parser.add_argument("--rgb-height", type=int, default=1080)
    parser.add_argument("--rgb-fps", type=float, default=30.)
    args = parser.parse_args()
    args.expect_rgb = args.expect_rgb == "true"
    if not math.isfinite(args.tail_sec) or args.tail_sec < 5 or not math.isfinite(args.rgb_fps) or args.rgb_fps <= 0:
        parser.error("tail must be at least 5 seconds and RGB fps must be positive")
    try:
        if Path(str(args.bag) + ".active").exists():
            raise ValueError("bag is still active")
        report = validate_bag(args)
    except Exception as exc:
        report = {"status": "FAIL", "failures": [str(exc)], "bag": str(args.bag)}
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    print("comparison recording postflight: " + report["status"])
    raise SystemExit(0 if report["status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
