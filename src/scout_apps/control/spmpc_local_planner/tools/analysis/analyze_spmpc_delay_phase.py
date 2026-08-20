#!/usr/bin/env python3
"""P0 SPMPC command/odom delay and phase diagnostic from rosbag.

This is an offline diagnostic only.  It never publishes commands and never
modifies bags.  Positive lag means the odom response is delayed relative to the
command signal, i.e. cmd(t) best matches odom(t + lag).

Typical use:
  python3 analyze_spmpc_delay_phase.py ~/slosh_bags/real/20260627_spmpc_full_rgb
  python3 analyze_spmpc_delay_phase.py run.bag --out-csv delay_summary.csv
"""

import argparse
import bisect
import csv
import math
import statistics as st
import sys
from pathlib import Path

try:
    import rosbag  # type: ignore
except ImportError:  # pragma: no cover - depends on sourced ROS environment
    rosbag = None


CMD_TOPIC = "/cmd_vel"
SPMPC_CMD_OUTPUT_TOPIC = "/spmpc/debug/cmd_vel_output"
SOLVER_TIME_TOPIC = "/spmpc/solver_time_ms"
STATUS_TOPIC = "/spmpc/status"
DELAY_PHASE_TOPIC = "/spmpc/debug/delay_phase"
EXECUTION_STATE_TOPIC = "/spmpc/debug/execution_state"
EXECUTION_ALIGNMENT_STATUS_TOPIC = "/spmpc/debug/execution_alignment_status"
DELAY_COMPENSATION_TOPIC = "/spmpc/debug/delay_compensation"
ODOM_TOPICS = ["/odom", "/scout/odom"]


def finite(value):
    return value is not None and math.isfinite(float(value))


def percentile(values, q):
    xs = sorted(float(v) for v in values if finite(v))
    if not xs:
        return float("nan")
    if len(xs) == 1:
        return xs[0]
    pos = (len(xs) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return xs[lo]
    frac = pos - lo
    return xs[lo] * (1.0 - frac) + xs[hi] * frac


def mean(values):
    xs = [float(v) for v in values if finite(v)]
    return st.mean(xs) if xs else float("nan")


def median(values):
    xs = [float(v) for v in values if finite(v)]
    return st.median(xs) if xs else float("nan")


def max_value(values):
    xs = [float(v) for v in values if finite(v)]
    return max(xs) if xs else float("nan")


def fmt(value, digits=3):
    if not finite(value):
        return "NA"
    return f"{float(value):.{digits}f}"


def time_deltas(series):
    return [b[0] - a[0] for a, b in zip(series, series[1:]) if b[0] > a[0]]


def msg_header_stamp_sec(msg):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        sec = stamp.to_sec()
    except Exception:
        return None
    if sec <= 0.0:
        return None
    return sec


def clean_series(series):
    cleaned = sorted((float(t), float(v)) for t, v in series if finite(t) and finite(v))
    out = []
    for t, v in cleaned:
        if out and abs(out[-1][0] - t) < 1e-9:
            out[-1] = (t, v)
        else:
            out.append((t, v))
    return out


def make_interpolator(series):
    data = clean_series(series)
    times = [t for t, _ in data]
    values = [v for _, v in data]

    def interp(t):
        if not data or t < times[0] or t > times[-1]:
            return None
        idx = bisect.bisect_left(times, t)
        if idx < len(times) and abs(times[idx] - t) < 1e-9:
            return values[idx]
        if idx == 0 or idx >= len(times):
            return None
        t0, t1 = times[idx - 1], times[idx]
        v0, v1 = values[idx - 1], values[idx]
        if t1 <= t0:
            return None
        r = (t - t0) / (t1 - t0)
        return v0 + r * (v1 - v0)

    return data, interp


def pearson(xs, ys):
    if len(xs) != len(ys) or len(xs) < 3:
        return float("nan")
    mx = st.mean(xs)
    my = st.mean(ys)
    vx = sum((x - mx) ** 2 for x in xs)
    vy = sum((y - my) ** 2 for y in ys)
    if vx <= 1e-12 or vy <= 1e-12:
        return float("nan")
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return cov / math.sqrt(vx * vy)


def estimate_positive_delay(cmd_series, odom_series, sample_dt, max_lag, min_samples):
    """Return best positive lag where cmd(t) correlates with odom(t + lag)."""
    cmd_data, cmd_at = make_interpolator(cmd_series)
    odom_data, odom_at = make_interpolator(odom_series)
    if len(cmd_data) < 3 or len(odom_data) < 3:
        return {"delay_s": float("nan"), "corr": float("nan"), "samples": 0, "confidence": "NO_DATA"}

    best = None
    lag_steps = max(0, int(round(max_lag / sample_dt)))
    for i in range(lag_steps + 1):
        lag = i * sample_dt
        start = max(cmd_data[0][0], odom_data[0][0] - lag)
        end = min(cmd_data[-1][0], odom_data[-1][0] - lag)
        if end <= start:
            continue
        xs = []
        ys = []
        t = start
        # Avoid accumulating floating point drift into an infinite loop.
        max_steps = int(math.floor((end - start) / sample_dt)) + 1
        for _ in range(max_steps):
            cv = cmd_at(t)
            ov = odom_at(t + lag)
            if cv is not None and ov is not None:
                xs.append(cv)
                ys.append(ov)
            t += sample_dt
        if len(xs) < min_samples:
            continue
        corr = pearson(xs, ys)
        if not finite(corr):
            continue
        if best is None or corr > best["corr"]:
            best = {"delay_s": lag, "corr": corr, "samples": len(xs)}

    if best is None:
        return {"delay_s": float("nan"), "corr": float("nan"), "samples": 0, "confidence": "NO_VARIATION"}

    corr = best["corr"]
    if corr >= 0.70:
        confidence = "HIGH"
    elif corr >= 0.45:
        confidence = "MED"
    elif corr >= 0.25:
        confidence = "LOW"
    else:
        confidence = "WEAK"
    best["confidence"] = confidence
    return best


def filter_until(series, cutoff):
    if cutoff is None:
        return series
    return [(t, v) for t, v in series if t <= cutoff]


def find_bags(paths):
    bags = []
    for raw in paths:
        path = Path(raw).expanduser()
        if path.is_file() and path.suffix == ".bag":
            bags.append(path)
        elif path.is_dir():
            bags.extend(sorted(path.rglob("*.bag")))
        else:
            print(f"[WARN] skip non-bag path: {path}", file=sys.stderr)
    return sorted(dict.fromkeys(bags))


def read_bag(path, preferred_odom_topic, include_after_goal, cmd_source):
    topics = [
        CMD_TOPIC,
        SPMPC_CMD_OUTPUT_TOPIC,
        SOLVER_TIME_TOPIC,
        STATUS_TOPIC,
        DELAY_PHASE_TOPIC,
        EXECUTION_STATE_TOPIC,
        EXECUTION_ALIGNMENT_STATUS_TOPIC,
        DELAY_COMPENSATION_TOPIC,
    ]
    topics.extend(ODOM_TOPICS)

    cmd_vel = []
    spmpc_limited = []
    odom_by_topic = {topic: {"v": [], "omega": [], "age": []} for topic in ODOM_TOPICS}
    solver_ms = []
    limiter_linear = []
    limiter_angular_rate = []
    limiter_angular_accel = []
    delay_phase_linear_ms = []
    delay_phase_angular_ms = []
    closed_loop_enabled = []
    shadow_valid = []
    shadow_history_complete = []
    shadow_missing_history_ms = []
    alignment_status_counts = {}
    status_counts = {}
    first_goal_reached = None

    with rosbag.Bag(str(path)) as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            ts = stamp.to_sec()
            if topic == CMD_TOPIC:
                cmd_vel.append((ts, float(msg.linear.x), float(msg.angular.z)))
            elif topic == SPMPC_CMD_OUTPUT_TOPIC and len(getattr(msg, "data", [])) >= 10:
                data = list(msg.data)
                spmpc_limited.append((ts, float(data[2]), float(data[3])))
                limiter_linear.append(1.0 if data[7] > 0.5 else 0.0)
                limiter_angular_rate.append(1.0 if data[8] > 0.5 else 0.0)
                limiter_angular_accel.append(1.0 if data[9] > 0.5 else 0.0)
            elif topic == DELAY_PHASE_TOPIC and len(getattr(msg, "data", [])) >= 11:
                data = list(msg.data)
                delay_phase_linear_ms.append(float(data[5]))
                delay_phase_angular_ms.append(float(data[6]))
            elif topic == EXECUTION_STATE_TOPIC and len(getattr(msg, "data", [])) >= 19:
                data = list(msg.data)
                shadow_valid.append(1.0 if data[0] > 0.5 else 0.0)
                shadow_missing_history_ms.append(float(data[17]))
                shadow_history_complete.append(1.0 if data[18] > 0.5 else 0.0)
            elif topic == DELAY_COMPENSATION_TOPIC and len(getattr(msg, "data", [])) >= 4:
                data = list(msg.data)
                closed_loop_enabled.append(1.0 if len(data) >= 2 and data[1] > 0.5 else 0.0)
                delay_phase_linear_ms.append(float(data[2]))
                delay_phase_angular_ms.append(float(data[3]))
            elif topic == EXECUTION_ALIGNMENT_STATUS_TOPIC:
                value = str(msg.data)
                alignment_status_counts[value] = alignment_status_counts.get(value, 0) + 1
            elif topic in odom_by_topic:
                v = float(msg.twist.twist.linear.x)
                omega = float(msg.twist.twist.angular.z)
                odom_by_topic[topic]["v"].append((ts, v))
                odom_by_topic[topic]["omega"].append((ts, omega))
                hts = msg_header_stamp_sec(msg)
                if hts is not None:
                    age = ts - hts
                    if -0.5 <= age <= 10.0:
                        odom_by_topic[topic]["age"].append(age)
            elif topic == SOLVER_TIME_TOPIC:
                solver_ms.append(float(msg.data))
            elif topic == STATUS_TOPIC:
                value = str(msg.data)
                status_counts[value] = status_counts.get(value, 0) + 1
                if value == "GOAL_REACHED" and first_goal_reached is None:
                    first_goal_reached = ts

    odom_topic = preferred_odom_topic
    if not odom_by_topic.get(odom_topic, {}).get("v"):
        for candidate in ODOM_TOPICS:
            if odom_by_topic[candidate]["v"]:
                odom_topic = candidate
                break

    cutoff = None if include_after_goal else first_goal_reached
    if cmd_source == "spmpc_limited" and spmpc_limited:
        cmd_v = [(t, v) for t, v, _ in spmpc_limited]
        cmd_omega = [(t, w) for t, _, w in spmpc_limited]
        cmd_topic_used = SPMPC_CMD_OUTPUT_TOPIC + "[limited]"
        cmd_period_source = [(t, v) for t, v, _ in spmpc_limited]
    else:
        cmd_v = [(t, v) for t, v, _ in cmd_vel]
        cmd_omega = [(t, w) for t, _, w in cmd_vel]
        cmd_topic_used = CMD_TOPIC
        cmd_period_source = [(t, v) for t, v, _ in cmd_vel]

    return {
        "bag_start": bag_start,
        "bag_end": bag_end,
        "duration_s": bag_end - bag_start,
        "goal_reached_s": None if first_goal_reached is None else first_goal_reached - bag_start,
        "cmd_topic_used": cmd_topic_used,
        "odom_topic_used": odom_topic,
        "cmd_v": filter_until(cmd_v, cutoff),
        "cmd_omega": filter_until(cmd_omega, cutoff),
        "odom_v": filter_until(odom_by_topic[odom_topic]["v"], cutoff),
        "odom_omega": filter_until(odom_by_topic[odom_topic]["omega"], cutoff),
        "odom_age": odom_by_topic[odom_topic]["age"],
        "solver_ms": solver_ms,
        "cmd_periods": time_deltas(cmd_period_source),
        "limiter_linear": limiter_linear,
        "limiter_angular_rate": limiter_angular_rate,
        "limiter_angular_accel": limiter_angular_accel,
        "delay_phase_linear_ms": delay_phase_linear_ms,
        "delay_phase_angular_ms": delay_phase_angular_ms,
        "closed_loop_enabled": closed_loop_enabled,
        "shadow_valid": shadow_valid,
        "shadow_history_complete": shadow_history_complete,
        "shadow_missing_history_ms": shadow_missing_history_ms,
        "alignment_status_counts": alignment_status_counts,
        "status_counts": status_counts,
    }


def analyze_bag(path, args):
    data = read_bag(path, args.odom_topic, args.include_after_goal, args.cmd_source)
    v_delay = estimate_positive_delay(
        data["cmd_v"], data["odom_v"], args.sample_dt, args.max_lag, args.min_samples)
    omega_delay = estimate_positive_delay(
        data["cmd_omega"], data["odom_omega"], args.sample_dt, args.max_lag, args.min_samples)

    cmd_period_ms = [1000.0 * dt for dt in data["cmd_periods"] if dt > 0.0]
    odom_age_ms = [1000.0 * age for age in data["odom_age"]]
    top_status = "-"
    if data["status_counts"]:
        top_status = max(data["status_counts"].items(), key=lambda kv: kv[1])[0]
    top_alignment_status = "-"
    if data["alignment_status_counts"]:
        top_alignment_status = max(data["alignment_status_counts"].items(), key=lambda kv: kv[1])[0]

    return {
        "bag": str(path),
        "duration_s": data["duration_s"],
        "goal_reached_s": data["goal_reached_s"],
        "top_status": top_status,
        "cmd_topic": data["cmd_topic_used"],
        "odom_topic": data["odom_topic_used"],
        "cmd_n": len(data["cmd_v"]),
        "odom_n": len(data["odom_v"]),
        "solver_n": len(data["solver_ms"]),
        "cmd_period_median_ms": median(cmd_period_ms),
        "cmd_period_p95_ms": percentile(cmd_period_ms, 0.95),
        "solver_median_ms": median(data["solver_ms"]),
        "solver_p95_ms": percentile(data["solver_ms"], 0.95),
        "solver_max_ms": max_value(data["solver_ms"]),
        "odom_age_median_ms": median(odom_age_ms),
        "odom_age_p95_ms": percentile(odom_age_ms, 0.95),
        "linear_limited_frac": mean(data["limiter_linear"]),
        "angular_rate_limited_frac": mean(data["limiter_angular_rate"]),
        "angular_accel_limited_frac": mean(data["limiter_angular_accel"]),
        "delay_phase_linear_ms": median(data["delay_phase_linear_ms"]),
        "delay_phase_angular_ms": median(data["delay_phase_angular_ms"]),
        "closed_loop_enabled_frac": mean(data["closed_loop_enabled"]),
        "closed_loop_enabled_count": sum(data["closed_loop_enabled"]),
        "closed_loop_enabled_samples": len(data["closed_loop_enabled"]),
        "shadow_valid_frac": mean(data["shadow_valid"]),
        "shadow_history_complete_frac": mean(data["shadow_history_complete"]),
        "shadow_missing_history_p95_ms": percentile(data["shadow_missing_history_ms"], 0.95),
        "top_alignment_status": top_alignment_status,
        "v_delay_s": v_delay["delay_s"],
        "v_corr": v_delay["corr"],
        "v_delay_samples": v_delay["samples"],
        "v_confidence": v_delay["confidence"],
        "omega_delay_s": omega_delay["delay_s"],
        "omega_corr": omega_delay["corr"],
        "omega_delay_samples": omega_delay["samples"],
        "omega_confidence": omega_delay["confidence"],
    }


def print_summary(rows):
    if not rows:
        return
    header = (
        "bag", "dur", "status", "v_delay", "v_corr", "v_conf",
        "w_delay", "w_corr", "w_conf", "solve_p95", "cmd_p95", "shadow", "closed", "align")
    print("\n" + "  ".join(f"{h:>10}" for h in header))
    print("-" * 128)
    for row in rows:
        name = Path(row["bag"]).name[:30]
        cells = [
            f"{name:>30}",
            f"{fmt(row['duration_s'], 1):>10}",
            f"{row['top_status'][:10]:>10}",
            f"{fmt(row['v_delay_s'], 3):>10}",
            f"{fmt(row['v_corr'], 2):>10}",
            f"{row['v_confidence'][:10]:>10}",
            f"{fmt(row['omega_delay_s'], 3):>10}",
            f"{fmt(row['omega_corr'], 2):>10}",
            f"{row['omega_confidence'][:10]:>10}",
            f"{fmt(row['solver_p95_ms'], 1):>10}",
            f"{fmt(row['cmd_period_p95_ms'], 1):>10}",
            f"{fmt(row['shadow_valid_frac'], 2):>10}",
            f"{fmt(row['closed_loop_enabled_frac'], 2):>10}",
            f"{row['top_alignment_status'][:10]:>10}",
        ]
        print("  ".join(cells))
    print("\n说明: v_delay / w_delay 为正表示 odom 相对 cmd 滞后；confidence 低时只作人工复核线索。")


def write_csv(path, rows):
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description="Estimate SPMPC P0 cmd->odom delay from rosbag files.")
    parser.add_argument("paths", nargs="+", help=".bag files or directories containing bags")
    parser.add_argument("--out-csv", default="", help="Optional CSV summary output path")
    parser.add_argument("--odom-topic", default="/odom", choices=ODOM_TOPICS,
                        help="Preferred odom topic; falls back to the other if missing")
    parser.add_argument("--cmd-source", default="cmd_vel", choices=["cmd_vel", "spmpc_limited"],
                        help="Use /cmd_vel or /spmpc/debug/cmd_vel_output limited command for lag estimation")
    parser.add_argument("--sample-dt", type=float, default=0.02, help="Resampling step for correlation, seconds")
    parser.add_argument("--max-lag", type=float, default=1.5, help="Maximum positive lag to scan, seconds")
    parser.add_argument("--min-samples", type=int, default=30, help="Minimum paired samples per lag")
    parser.add_argument("--include-after-goal", action="store_true",
                        help="Include samples after first /spmpc/status GOAL_REACHED")
    args = parser.parse_args()

    if rosbag is None:
        print("[ERR] Python rosbag is unavailable. Run after sourcing the ROS workspace setup.bash.", file=sys.stderr)
        return 2
    if args.sample_dt <= 0.0 or args.max_lag < 0.0:
        print("[ERR] --sample-dt must be >0 and --max-lag must be >=0", file=sys.stderr)
        return 2

    bags = find_bags(args.paths)
    if not bags:
        print("[WARN] no .bag files found")
        return 0

    rows = []
    for bag in bags:
        try:
            rows.append(analyze_bag(bag, args))
        except Exception as exc:  # keep batch analysis going
            print(f"[ERR] failed to analyze {bag}: {exc}", file=sys.stderr)

    print_summary(rows)
    if args.out_csv:
        write_csv(args.out_csv, rows)
        print(f"[OK] wrote CSV: {args.out_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
