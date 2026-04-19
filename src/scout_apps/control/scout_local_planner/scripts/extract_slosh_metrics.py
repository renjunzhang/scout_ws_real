#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os
import re
import statistics
import sys
from collections import defaultdict

import rosbag

STATUS_BUCKETS = ("TRACKING", "SETTLING", "REACHED", "IDLE")


def parse_args():
    parser = argparse.ArgumentParser(
        description="提取 slosh/MPC 实验 bag 指标，默认仅统计 TRACKING 段。"
    )
    parser.add_argument("inputs", nargs="+", help="bag 文件或包含 bag 的目录")
    parser.add_argument("--csv", dest="csv_path", default="", help="可选：导出汇总 CSV 路径")
    parser.add_argument("--all-segments", action="store_true", help="统计整包而不是仅 TRACKING 段")
    parser.add_argument("--per-episode", action="store_true", help="额外输出每个 TRACKING episode 的明细")
    return parser.parse_args()


def expand_inputs(paths):
    bag_paths = []
    for path in paths:
        if os.path.isdir(path):
            for name in sorted(os.listdir(path)):
                if name.endswith(".bag"):
                    bag_paths.append(os.path.join(path, name))
        elif os.path.isfile(path) and path.endswith(".bag"):
            bag_paths.append(path)
    return sorted(dict.fromkeys(bag_paths))


def rms(values):
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def safe_mean(values):
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return float("nan")
    return statistics.mean(values)


def safe_max(values):
    values = [v for v in values if not math.isnan(v)]
    if not values:
        return float("nan")
    return max(values)


def q_from_name(path):
    match = re.search(r"Q([0-9]+(?:\.[0-9]+)?)", os.path.basename(path))
    return match.group(1) if match else ""


def get_status_segments(bag_path):
    transitions = []
    with rosbag.Bag(bag_path) as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        for _, msg, t in bag.read_messages(topics=["/mpc_status"]):
            status = str(msg.data)
            ts = t.to_sec()
            if not transitions or transitions[-1][1] != status:
                transitions.append((ts, status))

    segments = []
    for idx, (ts, status) in enumerate(transitions):
        end_ts = transitions[idx + 1][0] if idx + 1 < len(transitions) else end_time
        segments.append((ts, end_ts, status))

    return start_time, end_time, transitions, segments


def in_tracking(segments, ts):
    for start, end, status in segments:
        if status == "TRACKING" and start <= ts < end:
            return True
    return False


def status_at(segments, ts):
    for start, end, status in segments:
        if start <= ts < end:
            return status
    if segments and ts >= segments[-1][1]:
        return segments[-1][2]
    return "UNKNOWN"


def summarize_status_buckets(samples):
    summary = {}
    for status, values in samples.items():
        total = len(values)
        if total <= 0:
            continue
        solved = sum(1 for value in values if value == 1)
        summary[status] = {
            "count": total,
            "solve_fail_count": total - solved,
            "success_ratio": round(solved / total, 3),
        }
    return summary


def format_status_breakdown(summary):
    parts = []
    seen = set()
    for status in STATUS_BUCKETS:
        if status not in summary:
            continue
        seen.add(status)
        item = summary[status]
        parts.append(
            f"{status}: n={item['count']} success={item['success_ratio']:.3f} fail={item['solve_fail_count']}"
        )
    for status in sorted(summary.keys()):
        if status in seen:
            continue
        item = summary[status]
        parts.append(
            f"{status}: n={item['count']} success={item['success_ratio']:.3f} fail={item['solve_fail_count']}"
        )
    return " | ".join(parts)


def collect_metrics(bag_path, tracking_only=True):
    start_time, end_time, transitions, segments = get_status_segments(bag_path)

    metrics = defaultdict(list)
    status_val_by_status = defaultdict(list)
    goals = []
    global_path_count = 0
    episodes = set()
    metadata_topics = {"/slosh/episode_id", "/scout/goal", "/scout/global_path"}

    topics = [
        "/slosh/height",
        "/slosh/height_pred_max",
        "/mpc/solve_ms",
        "/mpc/status_val",
        "/slosh/speed_governor_active",
        "/slosh/v_des_eff",
        "/slosh/constraint_active",
        "/slosh/episode_id",
        "/cmd_vel",
        "/scout/goal",
        "/scout/global_path",
    ]

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            if tracking_only and topic not in metadata_topics and not in_tracking(segments, ts):
                continue

            if topic == "/slosh/height":
                metrics["height"].append(float(msg.data))
            elif topic == "/slosh/height_pred_max":
                metrics["pred"].append(float(msg.data))
            elif topic == "/mpc/solve_ms":
                metrics["solve_ms"].append(float(msg.data))
            elif topic == "/mpc/status_val":
                value = int(msg.data)
                metrics["status_val"].append(value)
                status_val_by_status[status_at(segments, ts)].append(value)
            elif topic == "/slosh/speed_governor_active":
                metrics["governor"].append(int(msg.data))
            elif topic == "/slosh/v_des_eff":
                metrics["v_des_eff"].append(float(msg.data))
            elif topic == "/slosh/constraint_active":
                metrics["constraint"].append(int(msg.data))
            elif topic == "/slosh/episode_id":
                episodes.add(int(msg.data))
            elif topic == "/cmd_vel":
                metrics["vx"].append(float(msg.linear.x))
                metrics["wz"].append(float(msg.angular.z))
            elif topic == "/scout/goal":
                p = msg.pose.position
                goals.append((round(p.x, 3), round(p.y, 3)))
            elif topic == "/scout/global_path":
                global_path_count += 1

    tracking_segments = [seg for seg in segments if seg[2] == "TRACKING"]
    tracking_time = sum(end - start for start, end, _ in tracking_segments)
    reached_count = sum(
        1 for index, (_, status) in enumerate(transitions) if index > 0 and status == "REACHED"
    )
    governor_on = sum(1 for x in metrics["governor"] if x == 1)
    governor_total = len(metrics["governor"])
    status_breakdown = summarize_status_buckets(status_val_by_status)

    row = {
        "bag_path": bag_path,
        "bag_name": os.path.basename(bag_path),
        "Q_label": q_from_name(bag_path),
        "mode": "tracking_only" if tracking_only else "whole_bag",
        "duration_s": round(end_time - start_time, 3),
        "tracking_time_s": round(tracking_time, 3),
        "episode_count": len(episodes),
        "goal_count": len(goals),
        "global_path_count": global_path_count,
        "reached_count": reached_count,
        "height_rms_m": round(rms(metrics["height"]), 6),
        "height_max_m": round(safe_max(metrics["height"]), 6),
        "height_pred_rms_m": round(rms(metrics["pred"]), 6),
        "height_pred_max_m": round(safe_max(metrics["pred"]), 6),
        "solve_ms_mean": round(safe_mean(metrics["solve_ms"]), 3),
        "solve_ms_max": round(safe_max(metrics["solve_ms"]), 3),
        "solve_fail_count": sum(1 for x in metrics["status_val"] if x != 1),
        "solve_success_ratio": round(
            sum(1 for x in metrics["status_val"] if x == 1) / len(metrics["status_val"]), 3
        ) if metrics["status_val"] else float("nan"),
        "constraint_active_count": sum(1 for x in metrics["constraint"] if x == 1),
        "governor_active_count": governor_on,
        "governor_active_ratio": round(governor_on / governor_total, 3) if governor_total else 0.0,
        "v_des_eff_mean": round(safe_mean(metrics["v_des_eff"]), 3),
        "v_des_eff_min": round(min(metrics["v_des_eff"]), 3) if metrics["v_des_eff"] else float("nan"),
        "cmd_vx_rms": round(rms(metrics["vx"]), 3),
        "cmd_wz_rms": round(rms(metrics["wz"]), 3),
        "status_val_breakdown": format_status_breakdown(status_breakdown),
        "status_transitions": " | ".join(
            f"{round(ts - start_time, 3)}:{status}" for ts, status in transitions
        ),
        "goals": " | ".join(f"({x},{y})" for x, y in goals),
    }
    for status in STATUS_BUCKETS:
        item = status_breakdown.get(status)
        key = status.lower()
        row[f"{key}_status_val_count"] = item["count"] if item else 0
        row[f"{key}_success_ratio"] = item["success_ratio"] if item else float("nan")
        row[f"{key}_solve_fail_count"] = item["solve_fail_count"] if item else 0

    episode_rows = []
    for index, (seg_start, seg_end, status) in enumerate(tracking_segments, start=1):
        if status != "TRACKING":
            continue
        seg_metrics = defaultdict(list)
        with rosbag.Bag(bag_path) as bag:
            for topic, msg, t in bag.read_messages(topics=[
                "/slosh/height",
                "/slosh/height_pred_max",
                "/mpc/solve_ms",
                "/mpc/status_val",
                "/slosh/speed_governor_active",
                "/slosh/v_des_eff",
                "/cmd_vel",
            ]):
                ts = t.to_sec()
                if not (seg_start <= ts < seg_end):
                    continue
                if topic == "/slosh/height":
                    seg_metrics["height"].append(float(msg.data))
                elif topic == "/slosh/height_pred_max":
                    seg_metrics["pred"].append(float(msg.data))
                elif topic == "/mpc/solve_ms":
                    seg_metrics["solve_ms"].append(float(msg.data))
                elif topic == "/mpc/status_val":
                    seg_metrics["status_val"].append(int(msg.data))
                elif topic == "/slosh/speed_governor_active":
                    seg_metrics["governor"].append(int(msg.data))
                elif topic == "/slosh/v_des_eff":
                    seg_metrics["v_des_eff"].append(float(msg.data))
                elif topic == "/cmd_vel":
                    seg_metrics["vx"].append(float(msg.linear.x))
                    seg_metrics["wz"].append(float(msg.angular.z))

        seg_governor_on = sum(1 for x in seg_metrics["governor"] if x == 1)
        seg_governor_total = len(seg_metrics["governor"])
        episode_rows.append({
            "bag_name": os.path.basename(bag_path),
            "episode_index": index,
            "start_s": round(seg_start - start_time, 3),
            "duration_s": round(seg_end - seg_start, 3),
            "height_rms_m": round(rms(seg_metrics["height"]), 6),
            "height_max_m": round(safe_max(seg_metrics["height"]), 6),
            "height_pred_rms_m": round(rms(seg_metrics["pred"]), 6),
            "height_pred_max_m": round(safe_max(seg_metrics["pred"]), 6),
            "solve_ms_mean": round(safe_mean(seg_metrics["solve_ms"]), 3),
            "solve_fail_count": sum(1 for x in seg_metrics["status_val"] if x != 1),
            "governor_active_ratio": round(seg_governor_on / seg_governor_total, 3) if seg_governor_total else 0.0,
            "v_des_eff_mean": round(safe_mean(seg_metrics["v_des_eff"]), 3),
            "cmd_vx_rms": round(rms(seg_metrics["vx"]), 3),
            "cmd_wz_rms": round(rms(seg_metrics["wz"]), 3),
        })

    return row, episode_rows


def write_csv(csv_path, rows):
    fieldnames = list(rows[0].keys()) if rows else []
    if not fieldnames:
        return
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows, per_episode_rows, per_episode):
    print("汇总结果:")
    for row in rows:
        print(
            f"- {row['bag_name']}: Q={row['Q_label'] or '-'} "
            f"tracking={row['tracking_time_s']}s "
            f"height_rms={row['height_rms_m']}m "
            f"height_max={row['height_max_m']}m "
            f"pred_rms={row['height_pred_rms_m']}m "
            f"solve_mean={row['solve_ms_mean']}ms "
            f"success={row['solve_success_ratio']} "
            f"fail={row['solve_fail_count']} "
            f"gov_ratio={row['governor_active_ratio']}"
        )
        print(f"  status_val by mpc_status: {row['status_val_breakdown']}")
    if per_episode and per_episode_rows:
        print("\n按 episode 明细:")
        for row in per_episode_rows:
            print(
                f"- {row['bag_name']} ep{row['episode_index']}: "
                f"dur={row['duration_s']}s "
                f"height_rms={row['height_rms_m']}m "
                f"pred_rms={row['height_pred_rms_m']}m "
                f"fail={row['solve_fail_count']} "
                f"gov_ratio={row['governor_active_ratio']}"
            )


def main():
    args = parse_args()
    bag_paths = expand_inputs(args.inputs)
    if not bag_paths:
        print("未找到可用的 .bag 文件", file=sys.stderr)
        return 1

    rows = []
    episode_rows = []
    tracking_only = not args.all_segments

    for bag_path in bag_paths:
        row, episodes = collect_metrics(bag_path, tracking_only=tracking_only)
        rows.append(row)
        if args.per_episode:
            episode_rows.extend(episodes)

    print_summary(rows, episode_rows, args.per_episode)

    if args.csv_path:
        write_csv(args.csv_path, rows)
        print(f"\n已写入 CSV: {args.csv_path}")
        if args.per_episode and episode_rows:
            base, ext = os.path.splitext(args.csv_path)
            episode_csv = f"{base}_episodes{ext or '.csv'}"
            write_csv(episode_csv, episode_rows)
            print(f"已写入 episode CSV: {episode_csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
