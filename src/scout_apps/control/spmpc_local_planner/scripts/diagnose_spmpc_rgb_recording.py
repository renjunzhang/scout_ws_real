#!/usr/bin/env python3
"""静态 RGB 对照：接收不写盘、只录 RGB、复用完整白名单录包。

只订阅传感器，不启动控制器、不发运动命令、不修改相机参数。
需要先 source ROS 和工作区。full 仅代表静态白名单负载，不模拟求解器。
"""

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time


SCRIPT_DIR = Path(__file__).resolve().parent
IMAGE = "/camera/color/image_raw"
INFO = "/camera/color/camera_info"
META = "/camera/color/metadata"
TOPICS = (IMAGE, INFO, META)


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def header_row(data, receive):
    seq, sec, nsec = struct.unpack_from("<III", data)
    return {"receive": receive, "stamp_ns": sec * 1000000000 + nsec, "seq": seq}


def summarize(rows, max_gap, fps, start=None, end=None):
    """Preserve acquisition order: sorting would hide resets/duplicates."""
    stamps = [r["stamp_ns"] / 1e9 for r in rows]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    seqs = [b["seq"] - a["seq"] for a, b in zip(rows, rows[1:])]
    span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0
    rate = (len(stamps) - 1) / span if span > 0 else 0
    failures = []
    if len(rows) < 2:
        failures.append("fewer than two samples")
    if any(g <= 0 for g in gaps):
        failures.append("source timestamp regression/duplicate")
    if any(s <= 0 for s in stamps):
        failures.append("nonpositive source stamp")
    if any(g > max_gap + 1e-6 for g in gaps):
        failures.append("source gap exceeds limit")
    if rate < 0.9 * fps:
        failures.append("rate below 90% of expected FPS")
    if start is not None and end is not None:
        if not stamps or stamps[0] - start > max_gap or end - stamps[-1] > max_gap:
            failures.append("stream does not cover measurement window")
        if len(rows) < max(2, 0.9 * fps * (end - start) - 2):
            failures.append("too few samples for measurement duration")
    return {"status": "FAIL" if failures else "PASS", "failures": failures,
            "count": len(rows), "rate_hz": rate,
            "max_gap_ms": max(gaps) * 1000 if gaps else None,
            "gap_over_limit_count": sum(g > max_gap + 1e-6 for g in gaps),
            "seq_missing_count": sum(max(0, d - 1) for d in seqs),
            "seq_regression_count": sum(d <= 0 for d in seqs)}


def evidence(live, recorded):
    """Upstream means before the observed ROS publication, not proven USB loss."""
    info = {r["stamp_ns"]: r for r in live.get(INFO, [])}
    matched = [(r, info[r["stamp_ns"]]) for r in live.get(META, [])
               if r["stamp_ns"] in info and r.get("frame_counter") is not None]
    upstream, skipped = [], []
    for (a, ai), (b, bi) in zip(matched, matched[1:]):
        fd = b["frame_counter"] - a["frame_counter"]
        sd = bi["seq"] - ai["seq"]
        if fd > 1 and sd == 1:
            upstream.append({"stamp_ns": b["stamp_ns"], "frame_delta": fd,
                             "driver_seq_delta": sd})
        if sd > 1:
            skipped.append({"stamp_ns": b["stamp_ns"], "frame_delta": fd,
                            "driver_seq_delta": sd})
    differences = {}
    for topic in (INFO, META):
        lr, br = live.get(topic, []), recorded.get(topic, [])
        if not lr or not br:
            continue
        lo = max(min(r["stamp_ns"] for r in lr), min(r["stamp_ns"] for r in br))
        hi = min(max(r["stamp_ns"] for r in lr), max(r["stamp_ns"] for r in br))
        ls = {r["stamp_ns"] for r in lr if lo <= r["stamp_ns"] <= hi}
        bs = {r["stamp_ns"] for r in br if lo <= r["stamp_ns"] <= hi}
        differences[topic] = {"live_present_bag_missing": len(ls - bs),
                              "bag_present_live_missing": len(bs - ls)}
    images, infos = recorded.get(IMAGE, []), recorded.get(INFO, [])
    missing_images = None
    if images and infos:
        lo = max(min(r["stamp_ns"] for r in images), min(r["stamp_ns"] for r in infos))
        hi = min(max(r["stamp_ns"] for r in images), max(r["stamp_ns"] for r in infos))
        missing_images = len({r["stamp_ns"] for r in infos if lo <= r["stamp_ns"] <= hi}
                             - {r["stamp_ns"] for r in images})
    return {"matched_metadata_info": len(matched),
            "counter_skips_with_contiguous_driver_seq": len(upstream),
            "upstream_skip_examples": upstream[:10],
            "driver_seq_skips_seen_live": len(skipped),
            "live_bag_comparison": differences,
            "bag_info_present_image_missing": missing_images,
            "interpretation": "连续驱动 seq 下设备帧号跳跃：缺口在所观测发布之前；设备、SDK、回调阻塞仍需区分。live 有而 bag 无：录包订阅/传输/写盘路径候选。"}


def camera_settings(rows):
    keys = ("auto_exposure", "auto_white_balance_temperature", "actual_exposure",
            "gain_level", "manual_white_balance", "clock_domain")
    return {k: dict(Counter(str(r["metadata"].get(k, "unavailable")) for r in rows))
            for k in keys}


def camera_lock_failures(rows, config):
    expected = {"auto_exposure": 0, "auto_white_balance_temperature": 0,
                "actual_exposure": config["exposure"], "gain_level": config["gain"],
                "manual_white_balance": config["white_balance"]}
    failures = []
    if not rows:
        return ["no metadata to verify camera lock"]
    for key, value in expected.items():
        observed = [r["metadata"].get(key) for r in rows]
        if any(v is None for v in observed):
            failures.append(key + " unavailable")
        if any(v is not None and float(v) != float(value) for v in observed):
            failures.append(key + " changed from frozen configuration")
    return failures


def context_coverage(times, start, end, max_gap=1.0):
    """Check receive-time coverage of the static full-recording sensor load."""
    selected = [t for t in times if start <= t <= end]
    gaps = [b - a for a, b in zip(selected, selected[1:])]
    failures = []
    if len(selected) < 2:
        failures.append("missing or insufficient context samples")
    elif (selected[0] - start > max_gap or end - selected[-1] > max_gap or
          any(g > max_gap or g <= 0 for g in gaps)):
        failures.append("context topic does not continuously cover measurement window")
    return {"status": "FAIL" if failures else "PASS", "failures": failures,
            "count": len(selected), "max_receive_gap_sec": max(gaps) if gaps else None}


def source_window(live, receive_start, receive_end):
    """Find common frame identities even if the camera clock has an offset.

    The independent clock gate still fails on that offset. Never query bags by
    source time as though it were the recorder's receive-time index.
    """
    info = [r for r in live.get(INFO, []) if receive_start <= r["receive"] <= receive_end]
    if len(info) < 2:
        raise RuntimeError("采样窗口内 camera_info 不足")
    start, end = info[0]["stamp_ns"] / 1e9, info[-1]["stamp_ns"] / 1e9
    if end <= start:
        raise RuntimeError("相机时钟回退，不能建立共同窗口")
    return start, end


def clock_only_health(timestamp_gate):
    # The reused gate also checks frame gaps; those belong to continuity here.
    result = dict(timestamp_gate)
    result["failures"] = [f for f in timestamp_gate["failures"]
                          if not f.startswith("source max gap ")]
    result["status"] = "FAIL" if result["failures"] else "PASS"
    result["scope"] = "时基健康；帧间隔由 continuity_status 独立检查"
    return result


def stop_process(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=wait)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


def wait_for_bag_close(path, timeout_sec=30):
    # rosbag's Python launcher can return before its C++ child finishes rename.
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if path.is_file() and not Path(str(path) + ".active").exists():
            return
        time.sleep(0.1)
    raise RuntimeError("bag 未正常封口：" + str(path))


def capture_command(command):
    try:
        p = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, timeout=5)
        return {"returncode": p.returncode, "output": p.stdout}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"error": str(exc)}


def disk_counters():
    # Older distro psutil rejects recent kernels' extra /proc/diskstats fields.
    result = {}
    for line in Path("/proc/diskstats").read_text().splitlines():
        fields = line.split()
        if len(fields) >= 14 and not fields[2].startswith("loop"):
            result[fields[2]] = {"read_bytes": int(fields[5]) * 512,
                                 "write_bytes": int(fields[9]) * 512,
                                 "busy_time_ms": int(fields[12]),
                                 "io_in_progress": int(fields[11])}
    return result


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--modes", default="no_record,rgb,full",
                   help="逗号分隔，可重复，如 no_record,rgb,full,rgb")
    p.add_argument("--duration-sec", type=int, default=30)
    p.add_argument("--settle-sec", type=float, default=2)
    p.add_argument("--buffer-mb", type=int, default=4096)
    p.add_argument("--max-gap-sec", type=float, default=0.1)
    p.add_argument("--expected-fps", type=float, default=30)
    p.add_argument("--mocap-tracker", default="Tracker0")
    p.add_argument("--out-dir", type=Path,
                   default=Path.home() / "slosh_bags/real" /
                   time.strftime("%Y%m%d_rgb_static_diagnostic/%H%M%S"))
    args = p.parse_args()
    args.modes = args.modes.split(",")
    if (not args.modes or any(m not in ("no_record", "rgb", "full") for m in args.modes)
            or not 5 <= args.duration_sec <= 120 or not 0 <= args.settle_sec <= 10
            or args.buffer_mb <= 0 or not 0 < args.max_gap_sec <= 1
            or not 0 < args.expected_fps <= 60):
        p.error("无效模式/时长/缓冲/门限；duration 需 5–120 秒")
    return args


def main():
    args = parse_args()
    import psutil
    import rosbag
    import rospy
    from dynamic_reconfigure.client import Client
    from nav_msgs.msg import Odometry
    from realsense2_camera.msg import Metadata
    from rosgraph_msgs.msg import Log
    from sensor_msgs.msg import CameraInfo
    from validate_realsense_timestamp_health import evaluate_timestamp_records

    rospy.init_node("spmpc_rgb_static_diagnostic", anonymous=True, disable_signals=True)
    if rospy.get_param("/use_sim_time", False):
        raise RuntimeError("仅支持实时 ROS 时钟")
    master = rospy.get_master()
    context_topics = ("/odom", "/imu/data", "/scan_front",
                      "/vrpn_client_node/{}/pose".format(args.mocap_tracker), "/tf")

    def stationary_gate():
        code, _, state = master.getSystemState()
        if code != 1:
            raise RuntimeError("无法查询 ROS master")
        publishers = dict(state[0])
        if publishers.get("/cmd_vel"):
            raise RuntimeError("/cmd_vel 有发布者，请先停止 planner/teleop")

    stationary_gate()
    if "full" in args.modes:
        code, _, state = master.getSystemState()
        publishers = dict(state[0]) if code == 1 else {}
        missing = [t for t in context_topics if not publishers.get(t)]
        if missing:
            raise RuntimeError("完整静态负载缺少发布者：" + ", ".join(missing))
    for proc in psutil.process_iter(["cmdline"]):
        cmd = proc.info["cmdline"] or []
        if cmd and ((Path(cmd[0]).name == "record" and "rosbag" in cmd[0]) or
                    (Path(cmd[0]).name == "rosbag" and "record" in cmd[1:])):
            raise RuntimeError("已有 rosbag recorder，请等当前录制结束")
    client = Client("/camera/rgb_camera", timeout=5)
    before = client.get_configuration(timeout=5)
    if before["enable_auto_exposure"] or before["enable_auto_white_balance"]:
        raise RuntimeError("相机未锁参；先恢复已确认的手动配置")
    camera_info = rospy.wait_for_message(INFO, CameraInfo, timeout=5)
    if (camera_info.width, camera_info.height) != (1920, 1080):
        raise RuntimeError("本轮诊断要求保持 1920x1080")
    odom = rospy.wait_for_message("/odom", Odometry, timeout=5)

    def moving(msg):
        v = msg.twist.twist
        return math.hypot(v.linear.x, v.linear.y) > 0.01 or abs(v.angular.z) > 0.03

    if moving(odom):
        raise RuntimeError("里程计显示机器人在运动")
    args.out_dir = args.out_dir.expanduser().resolve()
    args.out_dir.mkdir(parents=True, exist_ok=False)
    # 6.22 MB/frame; reserve all new bags plus 3 GiB, never remove old data.
    required = sum(m != "no_record" for m in args.modes) * (args.duration_sec +
                 2 * args.settle_sec + 4) * 1920 * 1080 * 3 * args.expected_fps * 1.2
    if shutil.disk_usage(args.out_dir).free < required + 3 * 1024**3:
        raise RuntimeError("诊断录包空间不足")
    save(args.out_dir / "config_before.json", before)
    save(args.out_dir / "environment.json", {
        "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        "usb": capture_command(["lsusb", "-t"]),
        "disk": capture_command(["lsblk", "-o", "NAME,ROTA,TRAN,SIZE,MODEL,MOUNTPOINT"]),
        "nodes": capture_command(["rosnode", "list"]),
        "camera_parameters": rospy.get_param("/camera"),
        "git": capture_command(["git", "-C", str(SCRIPT_DIR), "rev-parse", "HEAD"]),
        "scope": "静态传感器白名单负载，不包含 planner 求解负载；no_record 使用 raw 字节接收器，不写图像。"})
    results = []
    for index, mode in enumerate(args.modes, 1):
        stationary_gate()
        name = "{:02d}_{}".format(index, mode)
        out = args.out_dir / name
        out.mkdir()
        bag_path = out / (name + ".bag")
        print("[RGB诊断] {} 开始，采样 {} 秒".format(name, args.duration_sec), flush=True)
        lock = threading.Lock()
        live = {t: [] for t in TOPICS}
        logs, motion = [], []
        odom_times = []
        telemetry = []

        def callback(msg, topic):
            row = {"receive": time.time(), "stamp_ns": msg.header.stamp.to_nsec(),
                   "seq": msg.header.seq}
            if topic == META:
                data = json.loads(msg.json_data)
                row.update(metadata=data, frame_counter=data.get("frame_counter", data.get("frame_number")))
            with lock:
                live[topic].append(row)

        def raw_callback(msg):
            row = header_row(msg._buff, time.time())
            with lock:
                live[IMAGE].append(row)

        def odom_callback(msg):
            odom_times.append(time.monotonic())
            if moving(msg):
                motion.append(msg.header.stamp.to_sec())

        def log_callback(msg):
            if msg.level >= Log.WARN and ("camera" in msg.name or "record" in msg.name):
                logs.append({"stamp": msg.header.stamp.to_sec(), "node": msg.name, "text": msg.msg})

        subs = [rospy.Subscriber(INFO, CameraInfo, callback, callback_args=INFO, queue_size=200),
                rospy.Subscriber(META, Metadata, callback, callback_args=META, queue_size=200),
                rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=10),
                rospy.Subscriber("/rosout", Log, log_callback, queue_size=200)]
        if mode == "no_record":
            subs.append(rospy.Subscriber(IMAGE, rospy.AnyMsg, raw_callback, queue_size=10,
                                         buff_size=32 * 1024**2))
        recorder = None
        log_file = (out / "recorder.log").open("w")
        try:
            if mode == "rgb":
                command = ["rosbag", "record", "--buffsize={}".format(args.buffer_mb),
                           "-O", str(bag_path), *TOPICS]
                recorder = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT,
                                            start_new_session=True)
            elif mode == "full":
                env = dict(os.environ, OUT_DIR=str(out), NAME=name,
                           RECORD_SEC=str(int(args.duration_sec + 2 * args.settle_sec + 4)),
                           ROSBAG_BUFFER_SIZE_MB=str(args.buffer_mb), RECORD_RGB="true",
                           RECORD_CAMERA="true", RECORD_CAMERA_INFO="true", RECORD_MOCAP="true",
                           MOCAP_TRACKER=args.mocap_tracker, RECORD_MOCAP_PATH="false",
                           RECORD_ALL_EXISTING_TOPICS="false", RECORD_TOPIC_INFO="false",
                           RECORD_CAMERA_COMPRESSED="false", RECORD_DEPTH="false", RECORD_SCAN="true",
                           RECORD_STANDALONE_SLOSH="true", RECORD_ONLINE_LIQUID="false",
                           RECORD_ONLINE_LIQUID_DEBUG_IMAGES="false", FORBID_IMAGE_STREAMS="false",
                           LIQUID_EXPORT_AFTER_RECORD="false", OPERATOR_NOTE="stationary RGB diagnostic")
                command = ["bash", str(SCRIPT_DIR / "record_spmpc_full_rgb_bag.sh")]
                recorder = subprocess.Popen(command, env=env, stdout=log_file,
                                            stderr=subprocess.STDOUT, start_new_session=True)
            if recorder:
                deadline = time.monotonic() + 30
                while not Path(str(bag_path) + ".active").exists():
                    if recorder.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError("录包未启动，见 " + str(out / "recorder.log"))
                    time.sleep(0.1)
            time.sleep(args.settle_sec)
            start = time.time()
            deadline = time.monotonic() + args.duration_sec
            psutil.cpu_percent(interval=None, percpu=True)
            while time.monotonic() < deadline:
                stationary_gate()
                if motion or not odom_times or time.monotonic() - odom_times[-1] > 1:
                    raise RuntimeError("检测到运动或里程计中断，结束诊断")
                if recorder and recorder.poll() is not None:
                    raise RuntimeError("录包提前退出")
                telemetry.append({"time": time.time(), "cpu_percent_per_core": psutil.cpu_percent(percpu=True),
                                  "available_memory": psutil.virtual_memory().available,
                                  "swap": psutil.swap_memory()._asdict(),
                                  "disk": disk_counters()})
                time.sleep(min(1, max(0, deadline - time.monotonic())))
            end = time.time()
            if mode == "full":
                recorder.wait(timeout=30)  # existing recorder closes bag and creates its sidecars
                if recorder.returncode != 0:
                    raise RuntimeError("完整录包脚本失败")
        finally:
            stop_process(recorder)
            for sub in subs:
                sub.unregister()
            log_file.close()
            save(out / "live_samples.json", live)
            save(out / "system_samples.json", telemetry)
            save(out / "driver_warnings.json", logs)
        config = client.get_configuration(timeout=5)
        frozen_keys = ("enable_auto_exposure", "enable_auto_white_balance", "exposure", "gain", "white_balance")
        if any(config[k] != before[k] for k in frozen_keys):
            raise RuntimeError("相机配置发生变化，停止比较")
        save(out / "config_after.json", config)
        # Anchor frame matching to INFO received during the actual measurement.
        # Clock skew is reported independently, never used to offset/correct a bag.
        source_start, source_end = source_window(live, start, end)
        live = {t: [r for r in rows if source_start <= r["stamp_ns"] / 1e9 <= source_end]
                for t, rows in live.items()}
        recorded = {t: [] for t in TOPICS}
        context_times = {t: [] for t in context_topics} if mode == "full" else {}
        if mode != "no_record":
            wait_for_bag_close(bag_path)
            with rosbag.Bag(str(bag_path)) as bag:
                for topic, raw, stamp in bag.read_messages(topics=list(TOPICS) + list(context_times), raw=True):
                    if topic in context_times:
                        context_times[topic].append(stamp.to_sec())
                        continue
                    row = header_row(raw[1], stamp.to_sec())
                    if not source_start <= row["stamp_ns"] / 1e9 <= source_end:
                        continue
                    if topic == META:
                        msg = raw[-1]().deserialize(raw[1])
                        data = json.loads(msg.json_data)
                        row.update(metadata=data, frame_counter=data.get("frame_counter", data.get("frame_number")))
                    recorded[topic].append(row)
            save(out / "bag_samples.json", recorded)
        context_stats = {t: context_coverage(times, start, end) for t, times in context_times.items()}
        live_stats = {t: summarize(rows, args.max_gap_sec, args.expected_fps, source_start, source_end)
                      for t, rows in live.items() if mode == "no_record" or t != IMAGE}
        bag_stats = ({t: summarize(rows, args.max_gap_sec, args.expected_fps, source_start, source_end)
                      for t, rows in recorded.items()} if mode != "no_record" else {})
        timestamp_gate = evaluate_timestamp_records([(r["receive"], r["stamp_ns"] / 1e9)
                                             for r in live[INFO]], max_gap_sec=args.max_gap_sec)
        health = clock_only_health(timestamp_gate)
        measurement_info = [r for r in live[INFO] if start <= r["receive"] <= end]
        coverage_failures = summarize(
            [dict(r, stamp_ns=round(r["receive"] * 1e9)) for r in measurement_info],
            args.max_gap_sec, args.expected_fps, start, end)["failures"]
        result = {"mode": mode, "sample_start": start, "sample_end": end,
                  "source_start": source_start, "source_end": source_end,
                  "measurement_coverage_failures": coverage_failures,
                  "live": live_stats, "bag": bag_stats, "clock_health": health,
                  "timestamp_health_gate": timestamp_gate,
                  "evidence": evidence(live, recorded), "camera_metadata": camera_settings(live[META]),
                  "camera_lock_failures": camera_lock_failures(live[META], before),
                  "full_context_coverage": context_stats,
                  "warnings": logs, "robot_motion_detected": bool(motion),
                  "bag_bytes": bag_path.stat().st_size if bag_path.exists() else 0}
        stream_ok = all(v["status"] == "PASS" for v in list(live_stats.values()) + list(bag_stats.values()))
        result["continuity_status"] = "PASS" if stream_ok and not coverage_failures else "FAIL"
        result["status"] = "PASS" if (result["continuity_status"] == "PASS" and
            health["status"] == "PASS" and not result["camera_lock_failures"] and
            all(v["status"] == "PASS" for v in context_stats.values())) else "FAIL"
        save(out / "report.json", result)
        results.append(result)
        save(args.out_dir / "summary.json", {"schema": "spmpc_rgb_static_diagnostic_v1",
             "scope": "静态诊断；不证明运动录包通过，不放宽既有录制门限。", "rounds": results})
        print("[RGB诊断] {} 连续性 {} / 时钟 {}: live info {:.2f}Hz / max {:.1f}ms; 发布前帧号缺口 {}".format(
            name, result["continuity_status"], health["status"], live_stats[INFO]["rate_hz"], live_stats[INFO]["max_gap_ms"] or 0,
            result["evidence"]["counter_skips_with_contiguous_driver_seq"]), flush=True)
    print("[RGB诊断] 报告：" + str(args.out_dir / "summary.json"), flush=True)
    return 0 if all(r["status"] == "PASS" for r in results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (Exception, KeyboardInterrupt) as exc:
        print("[RGB诊断] ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(2)
